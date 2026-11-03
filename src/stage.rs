//! Rebuilding the images a block references but does not ship with.
//!
//! An `UndistortAT` block points at images Terra generated inside its own
//! working folder — `models/pc/0/.temp/ImageCorrection/undistort`, under hashed
//! filenames. That folder is temporary. A project copied off the machine that
//! produced it arrives without a single one of them, and no amount of
//! coordinate correctness downstream can compensate for having no pixels.
//!
//! What does survive is the SD card. This module pairs the original photographs
//! with the solved poses and regenerates the undistorted imagery from them.
//!
//! Three things make that harder than it sounds.
//!
//! **The filenames are useless.** They are content hashes, so their sort order
//! is unrelated to capture order. Worse, the card is not guaranteed contiguous:
//! the quarry flight holds 1680 photographs numbered 1 to 1681, because number
//! two never made it off the aircraft. Pairing by index or by sort order is
//! therefore wrong for essentially every photograph in the set, while looking
//! entirely reasonable. Matching is done on physical position instead — every
//! DJI frame carries its own GPS, and at 23 m between shots against a couple of
//! metres of GNSS error, the nearest camera centre is unambiguous.
//!
//! **The distortion coefficients are zeroed.** The XML describes Terra's
//! virtual undistorted camera, not the lens. The real one is recorded in
//! `AT/report/report.md`, which is undocumented and has changed shape between
//! Terra versions, so it is parsed defensively: an unfamiliar table costs the
//! feature, never the run.
//!
//! **There are two right answers.** Poses belong to the camera body, not to the
//! lens model, so they are equally valid with undistorted images and a pinhole
//! camera, or with the originals and the full distortion model. The first is
//! what most splat trainers want; the second resamples nothing and so cannot
//! degrade anything. Both are offered.

use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

use anyhow::{bail, Context, Result};
use rayon::prelude::*;
use walkdir::WalkDir;

use crate::block::Block;
use crate::frame::LocalFrame;
use crate::math::Vec3;

/// BlocksExchange and Terra both measure the principal point from the centre of
/// the upper-left pixel; COLMAP, OpenCV and nerfstudio measure from its corner.
/// `block.rs` applies this to the XML's principal point, and the same shift has
/// to reach the report's — applied twice, or not at all, it is a uniform
/// sub-pixel bias that reads as ordinary softness.
const PIXEL_CENTRE_TO_CORNER: f64 = 0.5;

/// How much of a JPEG to read when only the XMP packet is wanted. DJI writes it
/// well inside the first 64 KB; 128 KB is slack for firmware that does not.
const XMP_SCAN_BYTES: usize = 128 * 1024;

// ---------------------------------------------------------------------------
// The physical camera, recovered from the AT report
// ---------------------------------------------------------------------------

/// The lens as actually solved, as opposed to the virtual pinhole the XML
/// describes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    pub focal: f64,
    pub cx: f64,
    pub cy: f64,
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub k4: f64,
    pub p1: f64,
    pub p2: f64,
}

impl Calibration {
    pub fn is_negligible(&self) -> bool {
        [self.k1, self.k2, self.k3, self.k4, self.p1, self.p2]
            .iter()
            .all(|v| v.abs() < 1e-12)
    }

    /// Apply the distortion model to a normalised image coordinate.
    fn distort(&self, x: f64, y: f64) -> (f64, f64) {
        let r2 = x * x + y * y;
        let radial = 1.0 + r2 * (self.k1 + r2 * (self.k2 + r2 * (self.k3 + r2 * self.k4)));
        (
            x * radial + 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x),
            y * radial + self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y,
        )
    }
}

/// Read the camera out of `AT/report/report.md`.
///
/// Prefers the `Optimized` row, which is the calibration the solve finished
/// with; falls back to `Initial`. Returns `Ok(None)` when the section is absent
/// or its shape is unfamiliar — the same stance `report.rs` takes, for the same
/// reason: Terra changes this file between versions and a schema change should
/// cost a feature rather than the run.
pub fn calibration_from_report(at_dir: &Path) -> Result<Option<Calibration>> {
    let path = at_dir.join("report").join("report.md");
    if !path.is_file() {
        return Ok(None);
    }
    let text =
        std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    Ok(parse_calibration(&text))
}

fn parse_calibration(text: &str) -> Option<Calibration> {
    // Try Optimized first, then Initial. The two rows carry a different number
    // of leading columns, so anchor on the label rather than on a column index.
    for want in ["optimized", "initial"] {
        for line in text.lines() {
            if !line.contains('|') {
                continue;
            }
            let cells: Vec<&str> = line.split('|').map(str::trim).collect();
            let Some(at) = cells.iter().position(|c| c.eq_ignore_ascii_case(want)) else {
                continue;
            };
            let nums: Vec<f64> = cells
                .iter()
                .skip(at + 1)
                .map_while(|c| c.parse::<f64>().ok())
                .collect();
            // Focal, Cx, Cy, K1, K2, K3, K4, P1, P2.
            if nums.len() < 9 {
                continue;
            }
            if !(nums[0].is_finite() && nums[0] > 1.0) {
                continue;
            }
            return Some(Calibration {
                focal: nums[0],
                cx: nums[1] + PIXEL_CENTRE_TO_CORNER,
                cy: nums[2] + PIXEL_CENTRE_TO_CORNER,
                k1: nums[3],
                k2: nums[4],
                k3: nums[5],
                k4: nums[6],
                p1: nums[7],
                p2: nums[8],
            });
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Finding the originals
// ---------------------------------------------------------------------------

/// An original photograph on disk, with the position it recorded for itself.
#[derive(Clone, Debug)]
pub struct SourceImage {
    pub path: PathBuf,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    /// False when the packet carried no `AbsoluteAltitude`. `alt` is then 0.0,
    /// which is sea level rather than a missing value, so such an image can
    /// only be compared horizontally -- and the report has to say which field
    /// was absent, or the reader is sent to look at the pairing instead.
    pub alt_known: bool,
    /// The sequence number out of `DJI_<timestamp>_<index>_V.JPG`, when present.
    pub index: Option<u32>,
    pub rtk_fixed: bool,
}

/// Pull a numeric field out of an XMP packet.
///
/// DJI writes these as attributes (`drone-dji:GpsLatitude="-34.26"`) but some
/// firmware uses elements, so both are accepted. The misspelling
/// `GpsLongtitude` is DJI's own and appears in shipping firmware; treating it
/// as a typo to be corrected rather than a spelling to be matched would lose
/// every position on those aircraft.
fn xmp_number(hay: &str, key: &str) -> Option<f64> {
    let at = hay.find(key)?;
    let rest = &hay[at + key.len()..];
    let rest = rest.trim_start();
    let body = if let Some(r) = rest.strip_prefix('=') {
        let r = r.trim_start();
        let q = r.chars().next()?;
        if q != '"' && q != '\'' {
            return None;
        }
        let r = &r[1..];
        &r[..r.find(q)?]
    } else if let Some(r) = rest.strip_prefix('>') {
        &r[..r.find('<')?]
    } else {
        return None;
    };
    // DJI writes an explicit leading '+' on positive values.
    body.trim().trim_start_matches('+').parse().ok()
}

fn index_from_name(path: &Path) -> Option<u32> {
    let stem = path.file_stem()?.to_str()?;
    // DJI_20260726162710_0001_V -> 0001
    stem.split('_').find_map(|p| {
        if p.len() >= 3 && p.chars().all(|c| c.is_ascii_digit()) {
            p.parse().ok()
        } else {
            None
        }
    })
}

fn read_xmp(path: &Path) -> Result<Option<SourceImage>> {
    let mut f = std::fs::File::open(path)?;
    let mut buf = vec![0u8; XMP_SCAN_BYTES];
    let n = f.read(&mut buf)?;
    buf.truncate(n);
    // The packet is ASCII inside a binary container, so a lossy view is exactly
    // right: the binary either side becomes replacement characters and the
    // fields read normally.
    let text = String::from_utf8_lossy(&buf);

    let lat = xmp_number(&text, "drone-dji:GpsLatitude");
    let lon = xmp_number(&text, "drone-dji:GpsLongitude")
        .or_else(|| xmp_number(&text, "drone-dji:GpsLongtitude"));
    let alt = xmp_number(&text, "drone-dji:AbsoluteAltitude");

    let (Some(lat), Some(lon)) = (lat, lon) else {
        return Ok(None);
    };
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return Ok(None);
    }

    Ok(Some(SourceImage {
        path: path.to_path_buf(),
        lat,
        lon,
        alt: alt.unwrap_or(0.0),
        alt_known: alt.is_some(),
        index: index_from_name(path),
        rtk_fixed: xmp_number(&text, "drone-dji:RtkFlag").map(|v| v as i64) == Some(50),
    }))
}

/// What a scan of the card turned up besides the images themselves.
///
/// Two separate absences, because they send the reader to different places: no
/// position at all means the photograph cannot be placed, while a position
/// without an altitude can still be matched on the horizontal.
#[derive(Clone, Copy, Debug, Default)]
pub struct ScanSummary {
    /// Files whose packet carried no usable latitude and longitude.
    pub without_gps: usize,
    /// Images that recorded a position but no `AbsoluteAltitude`.
    pub without_altitude: usize,
}

/// Find every JPEG beneath the given roots and read its recorded position.
///
/// Returns the images found and what was missing from the rest. A photograph
/// without GPS is not an error — screenshots, panoramas and hand-held shots
/// all end up on the same card — it simply cannot be matched and is left out.
pub fn scan_images(roots: &[PathBuf]) -> Result<(Vec<SourceImage>, ScanSummary)> {
    let mut files: Vec<PathBuf> = Vec::new();
    for root in roots {
        if !root.exists() {
            bail!(
                "there is no folder at {}.\n\
                 Point --images at the DCIM folder from the aircraft's card, or at \
                 the flight folder inside it.",
                root.display()
            );
        }
        for e in WalkDir::new(root)
            .max_depth(6)
            .into_iter()
            .filter_map(|e| e.ok())
        {
            if !e.file_type().is_file() {
                continue;
            }
            let is_jpeg = e
                .path()
                .extension()
                .and_then(|s| s.to_str())
                .map(|s| s.eq_ignore_ascii_case("jpg") || s.eq_ignore_ascii_case("jpeg"))
                .unwrap_or(false);
            if is_jpeg {
                files.push(e.path().to_path_buf());
            }
        }
    }
    files.sort();
    files.dedup();

    let results: Vec<Option<SourceImage>> = files
        .par_iter()
        .map(|p| read_xmp(p).unwrap_or(None))
        .collect();

    let summary = ScanSummary {
        without_gps: results.iter().filter(|r| r.is_none()).count(),
        without_altitude: results
            .iter()
            .filter(|r| r.as_ref().is_some_and(|i| !i.alt_known))
            .count(),
    };
    Ok((results.into_iter().flatten().collect(), summary))
}

/// DJI's `.MRK` RTK event table: index, latitude, longitude, ellipsoidal height.
///
/// Only used as a cross-check on the count, since it records what the aircraft
/// captured regardless of what survived onto the card.
pub fn read_mrk(path: &Path) -> Result<Vec<(u32, f64, f64, f64)>> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let mut out = Vec::new();
    for line in text.lines() {
        let fields: Vec<&str> = line.split_whitespace().collect();
        if fields.is_empty() {
            continue;
        }
        let Ok(index) = fields[0].parse::<u32>() else {
            continue;
        };
        let mut lat = None;
        let mut lon = None;
        let mut ellh = None;
        for f in &fields {
            let Some((value, tag)) = f.split_once(',') else {
                continue;
            };
            let Ok(v) = value.parse::<f64>() else {
                continue;
            };
            match tag {
                "Lat" => lat = Some(v),
                "Lon" => lon = Some(v),
                "Ellh" => ellh = Some(v),
                _ => {}
            }
        }
        if let (Some(a), Some(o), Some(h)) = (lat, lon, ellh) {
            out.push((index, a, o, h));
        }
    }
    Ok(out)
}

/// What the aircraft recorded against what actually reached the card.
pub struct CardCheck {
    pub captured: usize,
    /// Capture indices the aircraft logged but which are not on the card.
    pub missing: Vec<u32>,
}

/// Compare the photographs found with the aircraft's own capture log.
///
/// Worth doing before anything else: the quarry flight logged 1681 captures and
/// carries 1680 files, and knowing that a frame is genuinely absent — rather
/// than suspecting the matcher of losing it — is the difference between
/// trusting the result and rerunning the flight.
pub fn check_against_mrk(roots: &[PathBuf], images: &[SourceImage]) -> Option<CardCheck> {
    let mut rows = Vec::new();
    for root in roots {
        for e in WalkDir::new(root)
            .max_depth(6)
            .into_iter()
            .filter_map(|e| e.ok())
        {
            let is_mrk = e
                .path()
                .extension()
                .and_then(|s| s.to_str())
                .map(|s| s.eq_ignore_ascii_case("mrk"))
                .unwrap_or(false);
            if is_mrk && e.file_type().is_file() {
                if let Ok(mut r) = read_mrk(e.path()) {
                    rows.append(&mut r);
                }
            }
        }
    }
    if rows.is_empty() {
        return None;
    }

    let have: std::collections::HashSet<u32> = images.iter().filter_map(|i| i.index).collect();
    let mut missing: Vec<u32> = rows
        .iter()
        .map(|r| r.0)
        .filter(|i| !have.contains(i))
        .collect();
    missing.sort_unstable();
    missing.dedup();

    Some(CardCheck {
        captured: rows.len(),
        missing,
    })
}

/// How many photographs had a fixed RTK solution rather than a floating or
/// standalone one. Terra records the same thing in its report; agreement
/// between the two is a cheap confirmation the right flight is in hand.
pub fn rtk_fixed_count(images: &[SourceImage]) -> usize {
    images.iter().filter(|i| i.rtk_fixed).count()
}

// ---------------------------------------------------------------------------
// Matching poses to photographs
// ---------------------------------------------------------------------------

pub struct MatchReport {
    /// Parallel to `block.photos`.
    pub chosen: Vec<Option<PathBuf>>,
    pub matched: usize,
    pub unmatched: usize,
    pub median_distance_m: f64,
    pub worst_distance_m: f64,
    /// Images that were within range of some pose but lost it to a closer one.
    pub surplus_images: usize,
    /// The constant height difference removed before matching, in metres. A
    /// non-zero value means the block's vertical datum and the aircraft's
    /// altitudes disagree; horizontal positions are untouched either way.
    pub vertical_offset_m: f64,
}

/// The distance that decides a pair: the full 3D separation where the
/// photograph recorded an altitude, the horizontal one where it did not.
///
/// Keeping the vertical term is what stops a nadir pass and an oblique pass
/// over the same ground from swapping photographs. Dropping it where there is
/// no altitude is better than comparing against an assumed sea level, which
/// can never come within the distance budget.
fn pair_distance(v: Vec3, alt_known: bool) -> f64 {
    if alt_known {
        v.length()
    } else {
        (v.x * v.x + v.y * v.y).sqrt()
    }
}

/// The median signed height difference between each photograph and the pose
/// nearest it horizontally, over those pairs that already agree horizontally.
///
/// Horizontal agreement is what identifies the flight; the vertical is what
/// carries the datum. Restricting the estimate to pairs inside the horizontal
/// budget is the load-bearing part: an unrelated card produces no offset at
/// all, so pointing `--images` at the wrong flight still fails loudly rather
/// than being shifted into a false agreement. The median rather than the mean
/// keeps a handful of wild altitudes from dragging the result.
fn estimate_vertical_offset(
    block: &Block,
    local: &[Vec3],
    images: &[SourceImage],
    max_distance_m: f64,
) -> f64 {
    let mut deltas: Vec<f64> = Vec::new();
    for (l, img) in local.iter().zip(images) {
        if !img.alt_known {
            continue;
        }
        let mut nearest = f64::INFINITY;
        let mut dz = 0.0;
        for photo in &block.photos {
            let (dx, dy) = (l.x - photo.c.x, l.y - photo.c.y);
            let horizontal = (dx * dx + dy * dy).sqrt();
            if horizontal < nearest {
                nearest = horizontal;
                dz = l.z - photo.c.z;
            }
        }
        if nearest <= max_distance_m {
            deltas.push(dz);
        }
    }
    if deltas.is_empty() {
        return 0.0;
    }
    deltas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    deltas[deltas.len() / 2]
}

/// Pair every pose with the photograph taken from it.
///
/// Pairs are formed by global shortest-distance-first assignment rather than a
/// per-pose nearest-neighbour scan. The difference matters: a greedy scan will
/// happily give one photograph to two poses and leave a third with nothing,
/// producing a dataset that loads cleanly and trains into a blur. Here a
/// photograph is consumed when it is taken, so a collision is impossible by
/// construction rather than by inspection.
pub fn match_by_position(
    block: &Block,
    frame: &LocalFrame,
    images: &[SourceImage],
    max_distance_m: f64,
) -> Result<MatchReport> {
    if images.is_empty() {
        bail!("no photographs with usable GPS were found to match against");
    }

    let local: Vec<Vec3> = images
        .iter()
        .map(|i| frame.geodetic_to_local(i.lat, i.lon, i.alt))
        .collect::<Option<Vec<_>>>()
        .context(
            "this block has no geodetic anchor, so the photographs' GPS cannot be \
             compared with the camera centres.\n\
             That happens when the block is in a projected CRS whose EPSG code is \
             not a UTM zone. Supply already-matching images instead.",
        )?;

    // The block's heights and the aircraft's altitudes need not share a
    // vertical datum. frame.rs passes a projected block's own vertical axis
    // straight through, while AbsoluteAltitude is the aircraft's own figure,
    // and the difference between them is the geoid separation: about +20 m in
    // south-east Australia, +50 m in the UK, -30 m across much of the USA. A
    // constant bias that size exhausts the whole distance budget for every
    // candidate pair at once, so nothing matches and the failure names the
    // matcher rather than the heights. The quarry validation could not catch
    // this, because the two datums happen to agree closely there.
    let vertical_offset_m = estimate_vertical_offset(block, &local, images, max_distance_m);
    let local: Vec<Vec3> = local
        .iter()
        .zip(images)
        .map(|(l, img)| {
            if img.alt_known {
                Vec3::new(l.x, l.y, l.z - vertical_offset_m)
            } else {
                *l
            }
        })
        .collect();

    // Candidate pairs within range. Capping by distance keeps this far below
    // the n^2 worst case for any real flight.
    let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
    for (pi, photo) in block.photos.iter().enumerate() {
        for (ii, l) in local.iter().enumerate() {
            let d = pair_distance(*l - photo.c, images[ii].alt_known);
            if d <= max_distance_m {
                pairs.push((d, pi, ii));
            }
        }
    }
    pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut chosen: Vec<Option<PathBuf>> = vec![None; block.photos.len()];
    let mut photo_taken = vec![false; block.photos.len()];
    let mut image_taken = vec![false; images.len()];
    let mut distances: Vec<f64> = Vec::new();

    for (d, pi, ii) in pairs {
        if photo_taken[pi] || image_taken[ii] {
            continue;
        }
        photo_taken[pi] = true;
        image_taken[ii] = true;
        chosen[pi] = Some(images[ii].path.clone());
        distances.push(d);
    }

    distances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let matched = distances.len();
    let median = if matched == 0 {
        f64::NAN
    } else {
        distances[matched / 2]
    };

    Ok(MatchReport {
        chosen,
        matched,
        unmatched: block.photos.len() - matched,
        median_distance_m: median,
        worst_distance_m: distances.last().copied().unwrap_or(f64::NAN),
        surplus_images: images.len().saturating_sub(matched),
        vertical_offset_m,
    })
}

// ---------------------------------------------------------------------------
// Writing the images
// ---------------------------------------------------------------------------

pub struct StageOptions<'a> {
    pub out_dir: &'a Path,
    pub jpeg_quality: u8,
    pub overwrite: bool,
}

#[derive(Default)]
pub struct StageReport {
    pub written: usize,
    pub skipped_existing: usize,
    pub failed: Vec<(PathBuf, String)>,
    /// Mean fraction of output pixels that sampled outside the source frame.
    pub mean_out_of_bounds: f64,
}

fn images_dir(opts: &StageOptions) -> Result<PathBuf> {
    let d = opts.out_dir.join("images");
    std::fs::create_dir_all(&d).with_context(|| format!("creating {}", d.display()))?;
    Ok(d)
}

/// Copy the originals across untouched and point the block at them.
///
/// Nothing is resampled, so nothing can be lost. The caller pairs this with the
/// original distortion model written as an OPENCV or FULL_OPENCV camera.
pub fn copy_originals_into(
    block: &mut Block,
    matches: &MatchReport,
    opts: &StageOptions,
) -> Result<StageReport> {
    let dir = images_dir(opts)?;
    let mut report = StageReport::default();

    for (i, src) in matches.chosen.iter().enumerate() {
        let Some(src) = src else { continue };
        let Some(name) = src.file_name() else {
            continue;
        };
        let dest = dir.join(name);

        if dest.exists() && !opts.overwrite {
            report.skipped_existing += 1;
        } else {
            match std::fs::copy(src, &dest) {
                Ok(_) => report.written += 1,
                Err(e) => {
                    report.failed.push((src.clone(), e.to_string()));
                    continue;
                }
            }
        }
        block.photos[i].path = name.to_string_lossy().into_owned();
    }
    Ok(report)
}

/// The remap taking each undistorted output pixel back to a source coordinate.
struct Remap {
    width: u32,
    height: u32,
    xs: Vec<f32>,
    ys: Vec<f32>,
    out_of_bounds: f64,
}

/// Build the output-to-source map for one set of intrinsics.
///
/// This is `initUndistortRectifyMap` with the XML's pinhole as the new camera
/// matrix. Note the direction: output pixels are pushed *through* the
/// distortion model to find where they came from. Implementing the inverse
/// instead yields an image that looks entirely plausible and is wrong.
fn build_remap(
    width: u32,
    height: u32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    src: Calibration,
) -> Remap {
    let n = (width as usize) * (height as usize);
    let mut xs = vec![0f32; n];
    let mut ys = vec![0f32; n];
    let outside = AtomicUsize::new(0);

    xs.par_chunks_mut(width as usize)
        .zip(ys.par_chunks_mut(width as usize))
        .enumerate()
        .for_each(|(row, (xr, yr))| {
            let y = (row as f64 - cy) / fy;
            let mut local_out = 0usize;
            for col in 0..width as usize {
                let x = (col as f64 - cx) / fx;
                let (xd, yd) = src.distort(x, y);
                let us = src.focal * xd + src.cx;
                let vs = src.focal * yd + src.cy;
                if us < 0.0 || vs < 0.0 || us >= width as f64 || vs >= height as f64 {
                    local_out += 1;
                }
                xr[col] = us as f32;
                yr[col] = vs as f32;
            }
            outside.fetch_add(local_out, Ordering::Relaxed);
        });

    Remap {
        width,
        height,
        xs,
        ys,
        out_of_bounds: outside.load(Ordering::Relaxed) as f64 / n as f64,
    }
}

fn sample_bilinear(px: &[u8], w: usize, h: usize, x: f32, y: f32, out: &mut [u8]) {
    if x < 0.0 || y < 0.0 || x >= w as f32 || y >= h as f32 {
        out.copy_from_slice(&[0, 0, 0]);
        return;
    }
    let x0 = x.floor() as usize;
    let y0 = y.floor() as usize;
    let x1 = (x0 + 1).min(w - 1);
    let y1 = (y0 + 1).min(h - 1);
    let fx = x - x0 as f32;
    let fy = y - y0 as f32;

    for c in 0..3 {
        let p00 = px[(y0 * w + x0) * 3 + c] as f32;
        let p10 = px[(y0 * w + x1) * 3 + c] as f32;
        let p01 = px[(y1 * w + x0) * 3 + c] as f32;
        let p11 = px[(y1 * w + x1) * 3 + c] as f32;
        let top = p00 + (p10 - p00) * fx;
        let bot = p01 + (p11 - p01) * fx;
        out[c] = (top + (bot - top) * fy).round().clamp(0.0, 255.0) as u8;
    }
}

/// Undistort every matched original into `out_dir/images`, rewriting the
/// block's image paths to match.
pub fn undistort_into(
    block: &mut Block,
    matches: &MatchReport,
    src: Calibration,
    opts: &StageOptions,
) -> Result<StageReport> {
    let dir = images_dir(opts)?;

    // One map per distinct camera. All 1680 photogroups in the quarry block are
    // identical, so this turns 1680 map builds into one.
    let mut maps: HashMap<(u32, u32, u64, u64, u64, u64), Remap> = HashMap::new();
    let key = |g: &crate::block::Photogroup| {
        (
            g.width,
            g.height,
            g.fx.to_bits(),
            g.fy.to_bits(),
            g.cx.to_bits(),
            g.cy.to_bits(),
        )
    };
    for (i, chosen) in matches.chosen.iter().enumerate() {
        if chosen.is_none() {
            continue;
        }
        let g = &block.groups[block.photos[i].group];
        maps.entry(key(g))
            .or_insert_with(|| build_remap(g.width, g.height, g.fx, g.fy, g.cx, g.cy, src));
    }
    if maps.is_empty() {
        bail!("nothing to undistort: no photograph was matched to a pose");
    }

    let done = AtomicUsize::new(0);
    let total = matches.chosen.iter().filter(|c| c.is_some()).count();

    struct Outcome {
        index: usize,
        name: Option<String>,
        failure: Option<(PathBuf, String)>,
        skipped: bool,
    }

    let work: Vec<(usize, PathBuf)> = matches
        .chosen
        .iter()
        .enumerate()
        .filter_map(|(i, c)| c.as_ref().map(|p| (i, p.clone())))
        .collect();

    let outcomes: Vec<Outcome> = work
        .par_iter()
        .map(|(i, source)| {
            let name = source
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default();
            let dest = dir.join(&name);

            let n = done.fetch_add(1, Ordering::Relaxed) + 1;
            if n.is_multiple_of(100) || n == total {
                eprintln!("  undistorting {n} of {total}");
            }

            if dest.exists() && !opts.overwrite {
                return Outcome {
                    index: *i,
                    name: Some(name),
                    failure: None,
                    skipped: true,
                };
            }

            let g = &block.groups[block.photos[*i].group];
            let map = &maps[&key(g)];

            match undistort_one(source, &dest, map, opts.jpeg_quality) {
                Ok(()) => Outcome {
                    index: *i,
                    name: Some(name),
                    failure: None,
                    skipped: false,
                },
                Err(e) => Outcome {
                    index: *i,
                    name: None,
                    failure: Some((source.clone(), e.to_string())),
                    skipped: false,
                },
            }
        })
        .collect();

    let mut report = StageReport {
        mean_out_of_bounds: maps.values().map(|m| m.out_of_bounds).sum::<f64>() / maps.len() as f64,
        ..Default::default()
    };
    for o in outcomes {
        if let Some(f) = o.failure {
            report.failed.push(f);
            continue;
        }
        if o.skipped {
            report.skipped_existing += 1;
        } else {
            report.written += 1;
        }
        if let Some(name) = o.name {
            block.photos[o.index].path = name;
        }
    }
    Ok(report)
}

fn undistort_one(source: &Path, dest: &Path, map: &Remap, quality: u8) -> Result<()> {
    let bytes = std::fs::read(source)?;
    let img = image::load_from_memory_with_format(&bytes, image::ImageFormat::Jpeg)
        .with_context(|| format!("decoding {}", source.display()))?
        .to_rgb8();

    let (w, h) = (img.width() as usize, img.height() as usize);
    if w != map.width as usize || h != map.height as usize {
        bail!(
            "{} is {}x{} but the block describes {}x{}. Mixing resolutions in one \
             photogroup is not something the solve allows, so this is probably a \
             photograph from a different flight.",
            source.display(),
            w,
            h,
            map.width,
            map.height
        );
    }

    let px = img.as_raw();
    let mut out = vec![0u8; w * h * 3];
    out.par_chunks_mut(w * 3)
        .enumerate()
        .for_each(|(row, dst)| {
            for col in 0..w {
                let i = row * w + col;
                sample_bilinear(
                    px,
                    w,
                    h,
                    map.xs[i],
                    map.ys[i],
                    &mut dst[col * 3..col * 3 + 3],
                );
            }
        });

    // Write beside the target then rename, so an interrupted run never leaves a
    // half-written JPEG that a later pass would mistake for finished work.
    let tmp = dest.with_extension("partial");
    {
        let file = std::fs::File::create(&tmp)?;
        let mut w2 = std::io::BufWriter::new(file);
        let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut w2, quality);
        enc.encode(&out, w as u32, h as u32, image::ExtendedColorType::Rgb8)?;
    }
    std::fs::rename(&tmp, dest)?;
    Ok(())
}

// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// The real table, copied verbatim out of the quarry block's report.md.
    const REPORT: &str = "\
## Camera Calibration Information

Camera Model M4E_WideCamera

| Item     | Focal   | Cx      | Cy      |   K1  |   K2  |   K3  |   K4  |   P1  |   P2  |
| :---:    | :---:   | :---:   | :---:   | :---: | :---: | :---: | :---: | :---: | :---: |
| Initial  | 3713.26| 2678.62| 1961.53| -0.10657032| -0.00197749| -0.01524545| 0.00000000| 0.00018777| -0.00071175|

| Block     | Item     | Focal   | Cx      | Cy      | K1    |   K2  |   K3  |   K4  |   P1  |  P2   |
|     0 | Optimized| 3716.60| 2679.09| 1961.69| -0.10474077| -0.00564897| -0.01441997| 0.00000000| 0.00009975| -0.00071999|
";

    #[test]
    fn prefers_the_optimized_row_over_the_initial_one() {
        let c = parse_calibration(REPORT).expect("should parse the real table");
        assert_eq!(c.focal, 3716.60);
        assert_eq!(c.k1, -0.10474077);
        assert_eq!(c.p2, -0.00071999);
    }

    #[test]
    fn the_half_pixel_shift_is_applied_exactly_once() {
        let c = parse_calibration(REPORT).unwrap();
        assert_eq!(c.cx, 2679.09 + 0.5);
        assert_eq!(c.cy, 1961.69 + 0.5);
    }

    #[test]
    fn falls_back_to_initial_when_the_solve_row_is_missing() {
        let only_initial = REPORT
            .lines()
            .filter(|l| !l.contains("Optimized"))
            .collect::<Vec<_>>()
            .join("\n");
        let c = parse_calibration(&only_initial).unwrap();
        assert_eq!(c.focal, 3713.26);
    }

    #[test]
    fn an_unfamiliar_table_costs_the_feature_not_the_run() {
        assert!(parse_calibration("# nothing here").is_none());
        assert!(parse_calibration("| Optimized | not | numbers |").is_none());
    }

    #[test]
    fn distortion_model_inverts_numerically() {
        let c = Calibration {
            focal: 3716.6,
            cx: 2679.59,
            cy: 1962.19,
            k1: -0.10474077,
            k2: -0.00564897,
            k3: -0.01441997,
            k4: 0.0,
            p1: 0.00009975,
            p2: -0.00071999,
        };
        // Walk the distorted point back to the original by fixed-point iteration.
        for &(x, y) in &[(0.0, 0.0), (0.1, 0.05), (-0.3, 0.2), (0.45, -0.4)] {
            let (xd, yd) = c.distort(x, y);
            let (mut ux, mut uy) = (xd, yd);
            for _ in 0..50 {
                let (px, py) = c.distort(ux, uy);
                ux += xd - px;
                uy += yd - py;
            }
            assert!(
                (ux - x).abs() < 1e-9 && (uy - y).abs() < 1e-9,
                "({x}, {y}) did not survive the round trip: got ({ux}, {uy})"
            );
        }
    }

    #[test]
    fn a_camera_with_no_distortion_maps_to_itself() {
        let c = Calibration {
            focal: 1000.0,
            cx: 500.0,
            cy: 400.0,
            k1: 0.0,
            k2: 0.0,
            k3: 0.0,
            k4: 0.0,
            p1: 0.0,
            p2: 0.0,
        };
        assert!(c.is_negligible());
        let m = build_remap(64, 48, 1000.0, 1000.0, 500.0, 400.0, c);
        for row in 0..48usize {
            for col in 0..64usize {
                let i = row * 64 + col;
                assert!((m.xs[i] - col as f32).abs() < 1e-3, "x at {col},{row}");
                assert!((m.ys[i] - row as f32).abs() < 1e-3, "y at {col},{row}");
            }
        }
    }

    #[test]
    fn xmp_fields_read_from_attributes_and_elements() {
        let attr = r#"<rdf:Description drone-dji:GpsLatitude="-34.26054415" drone-dji:GpsLongitude="+150.59458478" drone-dji:RtkFlag="50"/>"#;
        assert_eq!(
            xmp_number(attr, "drone-dji:GpsLatitude"),
            Some(-34.26054415)
        );
        assert_eq!(
            xmp_number(attr, "drone-dji:GpsLongitude"),
            Some(150.59458478)
        );
        assert_eq!(xmp_number(attr, "drone-dji:RtkFlag"), Some(50.0));

        let elem = "<drone-dji:AbsoluteAltitude>452.032</drone-dji:AbsoluteAltitude>";
        assert_eq!(
            xmp_number(elem, "drone-dji:AbsoluteAltitude"),
            Some(452.032)
        );

        assert_eq!(xmp_number("nothing to see", "drone-dji:GpsLatitude"), None);
    }

    #[test]
    fn the_dji_longitude_misspelling_is_accepted() {
        // Shipping firmware writes "Longtitude". Not matching it would lose
        // every position from those aircraft.
        let s = r#"drone-dji:GpsLongtitude="150.5945""#;
        assert_eq!(
            xmp_number(s, "drone-dji:GpsLongitude")
                .or_else(|| xmp_number(s, "drone-dji:GpsLongtitude")),
            Some(150.5945)
        );
    }

    #[test]
    fn the_capture_index_is_read_out_of_the_filename() {
        assert_eq!(
            index_from_name(Path::new("DJI_20260726162710_0001_V.JPG")),
            Some(1)
        );
        assert_eq!(index_from_name(Path::new("random.jpg")), None);
    }

    #[test]
    fn mrk_rows_parse_including_the_gap_in_the_sequence() {
        let mrk = "1\t23250.500432\t[2429]\t116,N\t19,E\t147,V\t-34.26054415,Lat\t150.59458478,Lon\t452.032,Ellh\t0.08,0.08,0.14\t16,Q\n\
                   3\t23252.023199\t[2429]\t124,N\t20,E\t139,V\t-34.26034253,Lat\t150.59463615,Lon\t451.964,Ellh\t0.08,0.08,0.14\t16,Q\n";
        let p = std::env::temp_dir().join("terrapin_test.MRK");
        std::fs::write(&p, mrk).unwrap();
        let rows = read_mrk(&p).unwrap();
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0].0, 1);
        assert_eq!(rows[1].0, 3, "the missing capture must not be invented");
        assert!((rows[0].1 - -34.26054415).abs() < 1e-9);
        std::fs::remove_file(p).ok();
    }

    // ---- the vertical datum ------------------------------------------------

    /// A block whose poses sit at the given local centres. Only `c` reaches the
    /// offset estimator, so the rest is filler.
    fn poses(centres: &[Vec3]) -> Block {
        Block {
            photos: centres
                .iter()
                .enumerate()
                .map(|(i, c)| crate::block::Photo {
                    id: i as u32,
                    group: 0,
                    path: format!("pose_{i}.JPG"),
                    r: crate::math::Mat3::IDENTITY,
                    c: *c,
                })
                .collect(),
            ..Default::default()
        }
    }

    fn photo_with(alt_known: bool) -> SourceImage {
        SourceImage {
            path: PathBuf::from("DJI_0001.JPG"),
            lat: 0.0,
            lon: 0.0,
            alt: 0.0,
            alt_known,
            index: None,
            rtk_fixed: false,
        }
    }

    #[test]
    fn a_constant_height_bias_is_measured_rather_than_matched_around() {
        // What a geoid separation looks like from here: the photographs agree
        // horizontally to within GNSS error and sit a flat 48.6 m above the
        // block, which is more than the whole 25 m distance budget.
        let block = poses(&[
            Vec3::new(0.0, 0.0, 100.0),
            Vec3::new(30.0, 0.0, 100.0),
            Vec3::new(60.0, 0.0, 100.0),
        ]);
        let local = [
            Vec3::new(0.5, 0.0, 148.6),
            Vec3::new(30.4, 0.0, 148.6),
            Vec3::new(59.6, 0.0, 148.6),
        ];
        let images: Vec<SourceImage> = (0..3).map(|_| photo_with(true)).collect();
        let off = estimate_vertical_offset(&block, &local, &images, 25.0);
        assert!((off - 48.6).abs() < 1e-9, "offset was {off}");
    }

    #[test]
    fn an_unrelated_flight_produces_no_offset_at_all() {
        // The load-bearing case. Taking an offset from pairs that do not even
        // agree horizontally would shift the wrong card into a false
        // agreement, turning a loud and correct refusal into a dataset that
        // trains to a blur.
        let block = poses(&[Vec3::new(0.0, 0.0, 100.0), Vec3::new(30.0, 0.0, 100.0)]);
        let local = [
            Vec3::new(2_000.0, 0.0, 148.6),
            Vec3::new(2_030.0, 0.0, 148.6),
        ];
        let images: Vec<SourceImage> = (0..2).map(|_| photo_with(true)).collect();
        assert_eq!(
            estimate_vertical_offset(&block, &local, &images, 25.0),
            0.0,
            "a flight 2 km away must not contribute a height offset"
        );
    }

    #[test]
    fn a_photograph_without_an_altitude_is_left_out_of_the_estimate() {
        // Its `alt` is 0.0 meaning absent, not sea level. Counting it would
        // drag the offset by the whole elevation of the site.
        let block = poses(&[Vec3::new(0.0, 0.0, 100.0), Vec3::new(30.0, 0.0, 100.0)]);
        let local = [Vec3::new(0.0, 0.0, 148.6), Vec3::new(30.0, 0.0, 0.0)];
        let images = vec![photo_with(true), photo_with(false)];
        let off = estimate_vertical_offset(&block, &local, &images, 25.0);
        assert!((off - 48.6).abs() < 1e-9, "offset was {off}");
    }

    #[test]
    fn height_counts_except_where_no_altitude_was_recorded() {
        // Keeping the vertical term is what stops a nadir pass and an oblique
        // pass over the same ground from swapping photographs.
        let v = Vec3::new(3.0, 4.0, 40.0);
        assert!((pair_distance(v, false) - 5.0).abs() < 1e-12);
        assert!((pair_distance(v, true) - 1625.0_f64.sqrt()).abs() < 1e-12);
    }
}
