//! Reading Terra's point-cloud tiles, for use as a splat initialisation cloud.
//!
//! Terra emits the same cloud as PLY, LAS and PNTS tiles. PLY and LAS are both
//! handled here; PLY is usually an order of magnitude smaller on disk for the
//! same content, so prefer it when both exist.
//!
//! The important safety check lives in [`Cloud::centroid_offset_from`]: the
//! cloud tiles may carry their own coordinate offset that has nothing to do
//! with the AT block's frame. Seeding a trainer with a cloud that sits a
//! thousand kilometres from the cameras produces a splat that trains to noise,
//! slowly, with no obvious error message.

use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use rayon::prelude::*;
use walkdir::WalkDir;

use crate::frame::LocalFrame;
use crate::math::Vec3;

#[derive(Clone, Debug, Default)]
pub struct Cloud {
    pub points: Vec<Vec3>,
    pub colors: Vec<[u8; 3]>,
    /// How many points existed before decimation.
    pub source_count: usize,
    pub tiles_read: usize,
    pub tiles_failed: Vec<(PathBuf, String)>,
}

impl Cloud {
    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn centroid(&self) -> Vec3 {
        if self.points.is_empty() {
            return Vec3::ZERO;
        }
        self.points.iter().fold(Vec3::ZERO, |a, p| a + *p) / self.points.len() as f64
    }

    pub fn bounds(&self) -> (Vec3, Vec3) {
        let mut min = Vec3::splat(f64::INFINITY);
        let mut max = Vec3::splat(f64::NEG_INFINITY);
        for p in &self.points {
            min = min.min(*p);
            max = max.max(*p);
        }
        (min, max)
    }

    pub fn extent(&self) -> Vec3 {
        let (min, max) = self.bounds();
        max - min
    }

    pub fn centroid_offset_from(&self, other: Vec3) -> f64 {
        (self.centroid() - other).length()
    }

    /// Apply a transform to every point, in parallel.
    pub fn transform<F>(&mut self, f: F)
    where
        F: Fn(Vec3) -> Vec3 + Sync + Send,
    {
        self.points.par_iter_mut().for_each(|p| *p = f(*p));
    }

    /// Reduce to at most `target` points, keeping spatial coverage.
    ///
    /// Uses a deterministic strided pick rather than a random sample: Terra
    /// writes tiles in spatial order, so striding preserves coverage while
    /// staying reproducible between runs.
    pub fn decimate(&mut self, target: usize) {
        let n = self.points.len();
        if target == 0 || n <= target {
            return;
        }
        let mut pts = Vec::with_capacity(target);
        let mut cols = Vec::with_capacity(target);
        for i in 0..target {
            // Multiply first, divide once: avoids drift from a float step.
            let idx = i * n / target;
            pts.push(self.points[idx]);
            cols.push(self.colors[idx]);
        }
        self.points = pts;
        self.colors = cols;
    }

    fn merge(mut parts: Vec<Cloud>) -> Cloud {
        let mut out = Cloud::default();
        let total: usize = parts.iter().map(|c| c.points.len()).sum();
        out.points.reserve(total);
        out.colors.reserve(total);
        for p in parts.drain(..) {
            out.source_count += p.source_count;
            out.tiles_read += p.tiles_read;
            out.tiles_failed.extend(p.tiles_failed);
            out.points.extend(p.points);
            out.colors.extend(p.colors);
        }
        out
    }
}

/// Load every readable `.ply` and `.las` under a directory, in parallel.
///
/// A tile that fails to parse is recorded and skipped rather than aborting the
/// run: on a 16-tile quarry block, losing one tile to a truncated write is
/// survivable, whereas losing the whole conversion is not.
pub fn load_dir(dir: &Path, per_tile_cap: Option<usize>) -> Result<Cloud> {
    if !dir.is_dir() {
        bail!("{} is not a directory", dir.display());
    }
    // Terra nests each tile in its own folder — terra_ply/BlockRBABA/BlockRBABA.ply
    // — so a flat listing reports an empty directory for one holding sixteen
    // tiles. Depth is bounded because the same tree also holds the texture
    // pyramid, and walking all of that to find sixteen files would be slow for
    // no gain.
    let mut tiles: Vec<PathBuf> = WalkDir::new(dir)
        .max_depth(4)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_file())
        .map(|e| e.path().to_path_buf())
        .filter(|p| {
            matches!(
                p.extension().and_then(|s| s.to_str()).map(str::to_lowercase).as_deref(),
                Some("ply") | Some("las")
            )
        })
        .collect();
    tiles.sort();

    if tiles.is_empty() {
        bail!(
            "no .ply or .las tiles found in {}, or in anything beneath it.\n\
             Terra writes its cloud to models/pc/0/terra_ply or models/pc/0/terra_las_1_4 \
             — point at one of those. The b3dm and pnts folders hold the same cloud in \
             formats this does not read.",
            dir.display()
        );
    }

    let parts: Vec<Cloud> = tiles
        .par_iter()
        .map(|path| match load_file(path, per_tile_cap) {
            Ok(mut c) => {
                c.tiles_read = 1;
                c
            }
            Err(e) => Cloud {
                tiles_failed: vec![(path.clone(), e.to_string())],
                ..Default::default()
            },
        })
        .collect();

    Ok(Cloud::merge(parts))
}

// ---------------------------------------------------------------------------
// Getting a cloud into the block's frame
// ---------------------------------------------------------------------------

/// The geodetic anchor Terra records beside its tiles.
#[derive(Clone, Copy, Debug)]
pub struct Anchor {
    pub lon: f64,
    pub lat: f64,
    pub h: f64,
}

/// Read `metadata.xml` from a tile directory, or from one level above the
/// tiles, which is where Terra actually puts it.
///
/// The file is UTF-16 with a byte-order mark. Reading it as UTF-8 yields
/// alternating NUL bytes and no matches, which looks exactly like a file that
/// does not contain an anchor rather than one that could not be read.
pub fn read_anchor(dir: &Path) -> Option<Anchor> {
    let mut candidates = vec![dir.join("metadata.xml")];
    if let Some(p) = dir.parent() {
        candidates.push(p.join("metadata.xml"));
    }

    for path in candidates {
        let Ok(bytes) = std::fs::read(&path) else { continue };
        let text = decode_maybe_utf16(&bytes);
        let Some(at) = text.find("<SRSOrigin>") else { continue };
        let rest = &text[at + "<SRSOrigin>".len()..];
        let Some(end) = rest.find("</SRSOrigin>") else { continue };
        let nums: Vec<f64> = rest[..end]
            .split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();
        if nums.len() >= 3 {
            // Terra writes this as longitude, latitude, height.
            return Some(Anchor { lon: nums[0], lat: nums[1], h: nums[2] });
        }
    }
    None
}

fn decode_maybe_utf16(bytes: &[u8]) -> String {
    if bytes.len() >= 2 {
        let (le, be) = (bytes[0] == 0xFF && bytes[1] == 0xFE, bytes[0] == 0xFE && bytes[1] == 0xFF);
        if le || be {
            let units: Vec<u16> = bytes[2..]
                .chunks_exact(2)
                .map(|c| if le { u16::from_le_bytes([c[0], c[1]]) } else { u16::from_be_bytes([c[0], c[1]]) })
                .collect();
            return String::from_utf16_lossy(&units);
        }
    }
    String::from_utf8_lossy(bytes).into_owned()
}

/// How a cloud's raw coordinates relate to the block's.
pub struct Placement {
    pub label: &'static str,
}

/// Put a cloud into the block's working frame.
///
/// Terra emits its cloud in a frame unrelated to the one it solved the block
/// in — the LAS tiles in raw geographic degrees, the mesh tiles as degree
/// offsets from an anchor buried in a UTF-16 `metadata.xml`. Neither is
/// declared anywhere the block can see, and neither can be assumed: a cloud
/// written by a different Terra version may be in the block's own coordinates
/// already.
///
/// Rather than guess, every plausible reading is applied to a sample and the
/// one that lands on the cameras wins. Longitude/latitude order is resolved
/// the same way, because it genuinely varies and picking by convention would
/// mirror the scene about the diagonal while still "working".
pub fn place(
    cloud: &mut Cloud,
    frame: &LocalFrame,
    anchor: Option<Anchor>,
    cameras: Vec3,
) -> Placement {
    type Candidate = (&'static str, Box<dyn Fn(Vec3) -> Option<Vec3> + Sync>);
    let mut candidates: Vec<Candidate> = Vec::new();

    let f = *frame;
    candidates.push(("the block's own coordinates", Box::new(move |p| Some(f.point(p)))));

    // Absolute degrees, both axis orders.
    candidates.push((
        "geographic degrees (x=longitude)",
        Box::new(move |p| f.geodetic_to_local(p.y, p.x, p.z)),
    ));
    candidates.push((
        "geographic degrees (x=latitude)",
        Box::new(move |p| f.geodetic_to_local(p.x, p.y, p.z)),
    ));

    if let Some(a) = anchor {
        candidates.push((
            "degree offsets from the tiles' anchor (x=longitude)",
            Box::new(move |p| f.geodetic_to_local(p.y + a.lat, p.x + a.lon, p.z + a.h)),
        ));
        candidates.push((
            "degree offsets from the tiles' anchor (x=latitude)",
            Box::new(move |p| f.geodetic_to_local(p.x + a.lat, p.y + a.lon, p.z + a.h)),
        ));
    }

    // Judge on a stride through the whole cloud rather than the first points,
    // which in a tiled write all come from one corner.
    let n = cloud.points.len().max(1);
    let step = (n / 2000).max(1);
    let sample: Vec<Vec3> = cloud.points.iter().step_by(step).copied().collect();

    let mut best: Option<(usize, f64)> = None;
    for (i, (_, f)) in candidates.iter().enumerate() {
        let mut sum = Vec3::ZERO;
        let mut count = 0usize;
        for p in &sample {
            if let Some(q) = f(*p) {
                if q.is_finite() {
                    sum = sum + q;
                    count += 1;
                }
            }
        }
        if count == 0 {
            continue;
        }
        let d = (sum / count as f64 - cameras).length();
        if best.map(|(_, bd)| d < bd).unwrap_or(true) {
            best = Some((i, d));
        }
    }

    // The caller measures the real offset against the whole cloud; this only
    // has to pick the reading.
    let (idx, _) = best.unwrap_or((0, f64::INFINITY));
    let (label, chosen) = &candidates[idx];
    cloud.transform(|p| chosen(p).unwrap_or(p));
    Placement { label }
}

pub fn load_file(path: &Path, cap: Option<usize>) -> Result<Cloud> {
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_lowercase();
    let mut cloud = match ext.as_str() {
        "ply" => read_ply(path),
        "las" => read_las(path),
        other => bail!("unsupported point cloud extension '{other}'"),
    }
    .with_context(|| format!("reading {}", path.display()))?;

    if let Some(c) = cap {
        cloud.decimate(c);
    }
    Ok(cloud)
}

// ---------------------------------------------------------------------------
// PLY
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
enum PlyFormat {
    Ascii,
    BinaryLe,
    BinaryBe,
}

#[derive(Clone, Copy)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "char" | "int8" => Self::I8,
            "uchar" | "uint8" => Self::U8,
            "short" | "int16" => Self::I16,
            "ushort" | "uint16" => Self::U16,
            "int" | "int32" => Self::I32,
            "uint" | "uint32" => Self::U32,
            "float" | "float32" => Self::F32,
            "double" | "float64" => Self::F64,
            _ => return None,
        })
    }

    fn size(self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::F64 => 8,
        }
    }

    fn read(self, b: &[u8], le: bool) -> f64 {
        macro_rules! num {
            ($t:ty, $n:expr) => {{
                let mut a = [0u8; $n];
                a.copy_from_slice(&b[..$n]);
                if le {
                    <$t>::from_le_bytes(a) as f64
                } else {
                    <$t>::from_be_bytes(a) as f64
                }
            }};
        }
        match self {
            Self::I8 => b[0] as i8 as f64,
            Self::U8 => b[0] as f64,
            Self::I16 => num!(i16, 2),
            Self::U16 => num!(u16, 2),
            Self::I32 => num!(i32, 4),
            Self::U32 => num!(u32, 4),
            Self::F32 => num!(f32, 4),
            Self::F64 => num!(f64, 8),
        }
    }
}

fn read_ply(path: &Path) -> Result<Cloud> {
    let file = File::open(path)?;
    let mut rd = BufReader::with_capacity(1 << 20, file);

    let mut line = String::new();
    rd.read_line(&mut line)?;
    if line.trim() != "ply" {
        bail!("missing PLY magic");
    }

    let mut format = None;
    let mut count = 0usize;
    let mut props: Vec<(String, Scalar)> = Vec::new();
    let mut in_vertex = false;

    loop {
        line.clear();
        if rd.read_line(&mut line)? == 0 {
            bail!("truncated PLY header");
        }
        let tok: Vec<&str> = line.split_whitespace().collect();
        match tok.as_slice() {
            ["format", f, ..] => {
                format = Some(match *f {
                    "ascii" => PlyFormat::Ascii,
                    "binary_little_endian" => PlyFormat::BinaryLe,
                    "binary_big_endian" => PlyFormat::BinaryBe,
                    other => bail!("unknown PLY format '{other}'"),
                })
            }
            ["element", name, n, ..] => {
                in_vertex = *name == "vertex";
                if in_vertex {
                    count = n.parse().unwrap_or(0);
                }
            }
            ["property", "list", ..] if in_vertex => {
                bail!("list property inside the vertex element is not supported")
            }
            ["property", ty, name] if in_vertex => {
                let s = Scalar::parse(ty)
                    .with_context(|| format!("unknown PLY property type '{ty}'"))?;
                props.push((name.to_string(), s));
            }
            ["end_header"] => break,
            _ => {}
        }
    }

    let format = format.context("PLY header declared no format")?;
    let idx = |want: &str| props.iter().position(|(n, _)| n == want);
    let (ix, iy, iz) = (
        idx("x").context("PLY has no x property")?,
        idx("y").context("PLY has no y property")?,
        idx("z").context("PLY has no z property")?,
    );
    let rgb_idx = [
        ("red", "green", "blue"),
        ("r", "g", "b"),
        ("diffuse_red", "diffuse_green", "diffuse_blue"),
    ]
    .iter()
    .find_map(|(r, g, b)| Some((idx(r)?, idx(g)?, idx(b)?)));

    let mut points = Vec::with_capacity(count);
    let mut colors = Vec::with_capacity(count);

    match format {
        PlyFormat::Ascii => {
            for line in rd.lines().take(count) {
                let line = line?;
                let f: Vec<f64> = line
                    .split_whitespace()
                    .map(|s| s.parse().unwrap_or(f64::NAN))
                    .collect();
                if f.len() < props.len() {
                    continue;
                }
                points.push(Vec3::new(f[ix], f[iy], f[iz]));
                colors.push(pick_color(&f, rgb_idx));
            }
        }
        PlyFormat::BinaryLe | PlyFormat::BinaryBe => {
            let le = format == PlyFormat::BinaryLe;
            let stride: usize = props.iter().map(|(_, s)| s.size()).sum();
            let offsets: Vec<usize> = props
                .iter()
                .scan(0usize, |acc, (_, s)| {
                    let o = *acc;
                    *acc += s.size();
                    Some(o)
                })
                .collect();

            // Read in blocks rather than per-vertex: a 20-million-point tile
            // is 40k syscalls at 512 vertices per read, versus 20 million.
            let per_block = 8192.max(1);
            let mut buf = vec![0u8; stride * per_block];
            let mut remaining = count;
            while remaining > 0 {
                let take = remaining.min(per_block);
                let want = take * stride;
                if let Err(e) = rd.read_exact(&mut buf[..want]) {
                    // A truncated tail is worth keeping what we already have.
                    if points.is_empty() {
                        return Err(e).context("PLY body is shorter than its header claims");
                    }
                    break;
                }
                for v in 0..take {
                    let rec = &buf[v * stride..(v + 1) * stride];
                    let get = |i: usize| props[i].1.read(&rec[offsets[i]..], le);
                    points.push(Vec3::new(get(ix), get(iy), get(iz)));
                    colors.push(match rgb_idx {
                        Some((r, g, b)) => [scale8(get(r)), scale8(get(g)), scale8(get(b))],
                        None => DEFAULT_COLOR,
                    });
                }
                remaining -= take;
            }
        }
    }

    let n = points.len();
    Ok(Cloud {
        points,
        colors,
        source_count: n,
        tiles_read: 1,
        tiles_failed: Vec::new(),
    })
}

const DEFAULT_COLOR: [u8; 3] = [180, 180, 180];

fn scale8(v: f64) -> u8 {
    // Float colour channels are conventionally 0..1; integer ones 0..255.
    let x = if v <= 1.0 && v > 0.0 { v * 255.0 } else { v };
    x.clamp(0.0, 255.0) as u8
}

fn pick_color(f: &[f64], idx: Option<(usize, usize, usize)>) -> [u8; 3] {
    match idx {
        Some((r, g, b)) => [scale8(f[r]), scale8(f[g]), scale8(f[b])],
        None => DEFAULT_COLOR,
    }
}

// ---------------------------------------------------------------------------
// LAS
// ---------------------------------------------------------------------------

fn read_las(path: &Path) -> Result<Cloud> {
    let mut file = File::open(path)?;
    let mut head = vec![0u8; 375]; // LAS 1.4 header length; 1.2 is shorter
    let read = file.read(&mut head)?;
    if read < 227 || &head[0..4] != b"LASF" {
        bail!("not a LAS file");
    }

    let u16at = |o: usize| u16::from_le_bytes([head[o], head[o + 1]]);
    let u32at = |o: usize| u32::from_le_bytes([head[o], head[o + 1], head[o + 2], head[o + 3]]);
    let f64at = |o: usize| {
        let mut a = [0u8; 8];
        a.copy_from_slice(&head[o..o + 8]);
        f64::from_le_bytes(a)
    };

    let version_minor = head[25];
    let offset_to_data = u32at(96) as u64;
    let point_format = head[104] & 0b0011_1111;
    let point_len = u16at(105) as usize;
    let legacy_count = u32at(107) as u64;

    let count = if version_minor >= 4 && read >= 255 {
        let mut a = [0u8; 8];
        a.copy_from_slice(&head[247..255]);
        let extended = u64::from_le_bytes(a);
        if extended > 0 {
            extended
        } else {
            legacy_count
        }
    } else {
        legacy_count
    };

    let (sx, sy, sz) = (f64at(131), f64at(139), f64at(147));
    let (ox, oy, oz) = (f64at(155), f64at(163), f64at(171));

    if point_len < 20 {
        bail!("implausible LAS point record length {point_len}");
    }

    // Colour lives at a format-dependent offset, and only in some formats.
    let color_offset = match point_format {
        2 => Some(20),
        3 | 5 => Some(28),
        7 | 8 | 10 => Some(30),
        _ => None,
    }
    .filter(|o| o + 6 <= point_len);

    use std::io::Seek;
    file.seek(std::io::SeekFrom::Start(offset_to_data))?;
    let mut rd = BufReader::with_capacity(1 << 20, file);

    let count = count as usize;
    let mut points = Vec::with_capacity(count.min(50_000_000));
    let mut colors = Vec::with_capacity(count.min(50_000_000));

    let per_block = 8192usize;
    let mut buf = vec![0u8; point_len * per_block];
    let mut remaining = count;
    while remaining > 0 {
        let take = remaining.min(per_block);
        if rd.read_exact(&mut buf[..take * point_len]).is_err() {
            break;
        }
        for v in 0..take {
            let rec = &buf[v * point_len..(v + 1) * point_len];
            let xi = i32::from_le_bytes([rec[0], rec[1], rec[2], rec[3]]) as f64;
            let yi = i32::from_le_bytes([rec[4], rec[5], rec[6], rec[7]]) as f64;
            let zi = i32::from_le_bytes([rec[8], rec[9], rec[10], rec[11]]) as f64;
            points.push(Vec3::new(xi * sx + ox, yi * sy + oy, zi * sz + oz));
            colors.push(match color_offset {
                Some(o) => {
                    // LAS stores colour as 16-bit; most writers use the full range.
                    let c = |i: usize| {
                        (u16::from_le_bytes([rec[o + i * 2], rec[o + i * 2 + 1]]) >> 8) as u8
                    };
                    [c(0), c(1), c(2)]
                }
                None => DEFAULT_COLOR,
            });
        }
        remaining -= take;
    }

    let n = points.len();
    Ok(Cloud {
        points,
        colors,
        source_count: n,
        tiles_read: 1,
        tiles_failed: Vec::new(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn tmp(name: &str) -> PathBuf {
        let mut p = std::env::temp_dir();
        p.push(format!("terrapin_test_{name}"));
        p
    }

    fn write_binary_ply(path: &Path, pts: &[(f64, f64, f64, u8, u8, u8)]) {
        let mut f = File::create(path).unwrap();
        writeln!(f, "ply").unwrap();
        writeln!(f, "format binary_little_endian 1.0").unwrap();
        writeln!(f, "element vertex {}", pts.len()).unwrap();
        for p in ["x", "y", "z"] {
            writeln!(f, "property double {p}").unwrap();
        }
        for p in ["red", "green", "blue"] {
            writeln!(f, "property uchar {p}").unwrap();
        }
        writeln!(f, "end_header").unwrap();
        for (x, y, z, r, g, b) in pts {
            f.write_all(&x.to_le_bytes()).unwrap();
            f.write_all(&y.to_le_bytes()).unwrap();
            f.write_all(&z.to_le_bytes()).unwrap();
            f.write_all(&[*r, *g, *b]).unwrap();
        }
    }

    #[test]
    fn reads_binary_ply_with_colour() {
        let p = tmp("bin.ply");
        write_binary_ply(&p, &[(1.0, 2.0, 3.0, 10, 20, 30), (4.0, 5.0, 6.0, 40, 50, 60)]);
        let c = read_ply(&p).unwrap();
        assert_eq!(c.len(), 2);
        assert_eq!(c.points[1], Vec3::new(4.0, 5.0, 6.0));
        assert_eq!(c.colors[0], [10, 20, 30]);
        std::fs::remove_file(p).ok();
    }

    #[test]
    fn reads_ascii_ply_without_colour() {
        let p = tmp("ascii.ply");
        let mut f = File::create(&p).unwrap();
        write!(
            f,
            "ply\nformat ascii 1.0\nelement vertex 2\n\
             property float x\nproperty float y\nproperty float z\nend_header\n\
             1 2 3\n4 5 6\n"
        )
        .unwrap();
        drop(f);
        let c = read_ply(&p).unwrap();
        assert_eq!(c.len(), 2);
        assert_eq!(c.points[0], Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(c.colors[0], DEFAULT_COLOR);
        std::fs::remove_file(p).ok();
    }

    #[test]
    fn decimation_preserves_span_and_is_deterministic() {
        let mut c = Cloud::default();
        for i in 0..1000 {
            c.points.push(Vec3::new(i as f64, 0.0, 0.0));
            c.colors.push([0, 0, 0]);
        }
        let mut d = c.clone();
        c.decimate(100);
        d.decimate(100);
        assert_eq!(c.len(), 100);
        assert_eq!(c.points, d.points, "decimation must be reproducible");
        assert_eq!(c.points[0].x, 0.0);
        assert!(c.points[99].x >= 990.0, "tail of the range must survive");
    }

    #[test]
    fn decimation_is_a_no_op_below_target() {
        let mut c = Cloud::default();
        c.points.push(Vec3::ZERO);
        c.colors.push([1, 2, 3]);
        c.decimate(500);
        assert_eq!(c.len(), 1);
    }

    #[test]
    fn a_bad_tile_does_not_abort_the_directory() {
        let dir = tmp("tiles");
        std::fs::create_dir_all(&dir).unwrap();
        write_binary_ply(&dir.join("good.ply"), &[(1.0, 1.0, 1.0, 0, 0, 0)]);
        std::fs::write(dir.join("bad.ply"), b"not a ply at all").unwrap();
        let c = load_dir(&dir, None).unwrap();
        assert_eq!(c.tiles_read, 1);
        assert_eq!(c.tiles_failed.len(), 1);
        assert_eq!(c.len(), 1);
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn centroid_offset_detects_a_frame_mismatch() {
        let mut c = Cloud::default();
        for i in 0..10 {
            c.points.push(Vec3::new(i as f64, 0.0, 0.0));
            c.colors.push([0, 0, 0]);
        }
        // Cameras nearby: small offset.
        assert!(c.centroid_offset_from(Vec3::new(5.0, 0.0, 0.0)) < 1.0);
        // Cameras in a different frame entirely: megametres.
        assert!(c.centroid_offset_from(Vec3::new(-4.6e6, 2.6e6, -3.5e6)) > 1.0e6);
    }
}
