//! terrapin — extract DJI Terra aerotriangulation blocks into open formats.
//!
//! Terra's `AT` folder contains a solved photogrammetric block in Bentley's
//! BlocksExchange XML, alongside several proprietary binaries with no public
//! specification. The XML is the only part that matters downstream, and it
//! carries everything needed to reuse the solve elsewhere: intrinsics,
//! distortion, and per-photo world-to-camera pose.
//!
//! This turns that into COLMAP, nerfstudio, Bundler, PLY, GeoJSON, KML or CSV,
//! so the alignment can drive a Gaussian-splat trainer, a GIS, or a mesher,
//! without a Terra licence and without re-running structure from motion.

mod block;
mod cloud;
mod export;
mod find;
mod frame;
mod geodesy;
mod math;
mod report;
mod stage;

use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};

use crate::block::Block;
use crate::cloud::Cloud;
use crate::export::{ExportOptions, Format};
use crate::frame::LocalFrame;

#[derive(Parser)]
#[command(
    name = "terrapin",
    version,
    about = "Extract DJI Terra aerotriangulation blocks into open, portable formats",
    long_about = None
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Report what a block contains without writing anything.
    Inspect {
        /// BlocksExchange XML, usually AT/BlocksExchange*.xml
        xml: PathBuf,
        /// Print every parsed field, not just the summary.
        #[arg(long)]
        verbose: bool,
    },

    /// Convert a block into one or more open formats.
    Export {
        /// BlocksExchange XML, usually AT/BlocksExchange*.xml
        xml: PathBuf,

        /// Output directory.
        #[arg(short, long, default_value = "terrapin_out")]
        out: PathBuf,

        /// Formats to write. Repeat or comma-separate.
        #[arg(
            short, long,
            value_delimiter = ',',
            default_value = "colmap-txt",
            num_args = 1..
        )]
        format: Vec<Format>,

        /// Directory of PLY or LAS tiles to use as the initialisation cloud.
        #[arg(long)]
        cloud: Option<PathBuf>,

        /// Maximum points to keep from that cloud.
        #[arg(long, default_value_t = 300_000)]
        max_points: usize,

        /// Keep the full ImagePath instead of reducing to a bare filename.
        #[arg(long)]
        full_paths: bool,

        /// Write PINHOLE even when the block declares distortion.
        #[arg(long)]
        force_pinhole: bool,

        /// Write the cloud even if it does not overlap the cameras.
        #[arg(long)]
        allow_disjoint_cloud: bool,

        /// Folders holding the original photographs, usually the card's DCIM.
        /// Given these, the images the block needs are rebuilt from them.
        #[arg(long, num_args = 1..)]
        images: Vec<PathBuf>,

        /// Stage the originals unchanged and carry the lens distortion in the
        /// camera model, instead of undistorting the photographs.
        #[arg(long)]
        keep_distortion: bool,

        /// JPEG quality for rebuilt images.
        #[arg(long, default_value_t = 95)]
        quality: u8,

        /// How far a photograph may sit from a camera centre and still be
        /// considered the same shot, in metres.
        #[arg(long, default_value_t = 25.0)]
        max_distance: f64,

        /// Rewrite images that are already present.
        #[arg(long)]
        overwrite: bool,
    },

    /// Pair the original photographs with the block's poses and report,
    /// without writing anything.
    Match {
        /// Anywhere inside the Terra project.
        xml: PathBuf,
        /// Folders holding the original photographs.
        #[arg(long, num_args = 1.., required = true)]
        images: Vec<PathBuf>,
        /// How far a photograph may sit from a camera centre, in metres.
        #[arg(long, default_value_t = 25.0)]
        max_distance: f64,
    },

    /// Merge and decimate point-cloud tiles into a single PLY.
    Cloud {
        /// Directory of PLY or LAS tiles.
        dir: PathBuf,
        /// Output PLY.
        #[arg(short, long, default_value = "merged.ply")]
        out: PathBuf,
        /// Maximum points to keep.
        #[arg(long, default_value_t = 1_000_000)]
        max_points: usize,
    },

    /// Summarise aerotriangulation quality from the AT report folder.
    Report {
        /// The AT directory itself, not AT/report.
        at_dir: PathBuf,
        /// Print every parsed field.
        #[arg(long)]
        verbose: bool,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Inspect { xml, verbose } => cmd_inspect(&xml, verbose),
        Command::Export {
            xml,
            out,
            format,
            cloud,
            max_points,
            full_paths,
            force_pinhole,
            allow_disjoint_cloud,
            images,
            keep_distortion,
            quality,
            max_distance,
            overwrite,
        } => cmd_export(
            &xml,
            ExportRun {
                out,
                formats: format,
                cloud,
                max_points,
                basenames: !full_paths,
                force_pinhole,
                allow_disjoint: allow_disjoint_cloud,
                images,
                keep_distortion,
                quality,
                max_distance,
                overwrite,
            },
        ),
        Command::Match {
            xml,
            images,
            max_distance,
        } => cmd_match(&xml, &images, max_distance),
        Command::Cloud {
            dir,
            out,
            max_points,
        } => cmd_cloud(&dir, &out, max_points),
        Command::Report { at_dir, verbose } => cmd_report(&at_dir, verbose),
    }
}

// ---------------------------------------------------------------------------

fn rule(title: &str) {
    println!("\n\x1b[1m{title}\x1b[0m");
    println!("{}", "-".repeat(title.len().max(20)));
}

fn warn(msg: &str) {
    eprintln!("\x1b[33m  warning: {msg}\x1b[0m");
}

/// Whether two photogroups describe the same physical camera.
fn same_camera(a: &block::Photogroup, b: &block::Photogroup) -> bool {
    a.width == b.width
        && a.height == b.height
        && a.fx.to_bits() == b.fx.to_bits()
        && a.fy.to_bits() == b.fy.to_bits()
        && a.cx.to_bits() == b.cx.to_bits()
        && a.cy.to_bits() == b.cy.to_bits()
        && a.distortion == b.distortion
        && a.orientation == b.orientation
}

fn describe_block(block: &Block, stats: &block::ParseStats, verbose: bool) {
    rule("block");
    if !block.name.is_empty() {
        println!("  name              {}", block.name);
    }
    if block.srs.is_empty() {
        println!("  spatial reference (none declared)");
    } else {
        for s in &block.srs {
            println!("  spatial reference {} {}", s.name, s.definition);
        }
    }
    let cameras = {
        let mut seen: Vec<&block::Photogroup> = Vec::new();
        for g in &block.groups {
            if !seen.iter().any(|r| same_camera(r, g)) {
                seen.push(g);
            }
        }
        seen.len()
    };
    println!(
        "  photogroups       {} ({} distinct camera{})",
        block.groups.len(),
        cameras,
        if cameras == 1 { "" } else { "s" }
    );
    println!(
        "  posed photos      {} of {} seen",
        block.photos.len(),
        stats.photos_seen
    );
    if stats.photos_unposed > 0 {
        warn(&format!(
            "{} photo(s) had no usable pose and were dropped",
            stats.photos_unposed
        ));
    }

    let detected = frame::SourceFrame::detect(block);
    println!("  coordinate frame  {}", detected.label());
    let e = block.extent();
    println!(
        "  camera extent     {:.2} x {:.2} x {:.2} (source units)",
        e.x, e.y, e.z
    );

    // Terra writes one photogroup per photograph when every frame shares a
    // camera, so a block of 1680 images describes the same lens 1680 times.
    // Printing all of them buries the one line that decides the workflow.
    let mut distinct: Vec<(&block::Photogroup, usize)> = Vec::new();
    for g in &block.groups {
        match distinct.iter_mut().find(|(r, _)| same_camera(r, g)) {
            Some((_, n)) => *n += 1,
            None => distinct.push((g, 1)),
        }
    }

    for (g, count) in &distinct {
        if *count > 1 {
            rule(&format!("camera — {count} photogroups share it"));
        } else {
            rule(&format!("photogroup {} — {}", g.index, g.name));
        }
        println!("  image size        {} x {}", g.width, g.height);
        println!("  focal (px)        {:.4}  /  {:.4}", g.fx, g.fy);
        println!("  principal point   {:.4}, {:.4}", g.cx, g.cy);
        println!("  camera axes       {}", g.orientation.label());
        let d = &g.distortion;
        println!(
            "  distortion        K1 {:.6e}  K2 {:.6e}  K3 {:.6e}",
            d.k1, d.k2, d.k3
        );
        println!("                    P1 {:.6e}  P2 {:.6e}", d.p1, d.p2);
        if d.is_negligible() {
            println!("  \x1b[36m→ distortion is zeroed: this block expects UNDISTORTED images\x1b[0m");
        } else {
            println!("  \x1b[36m→ distortion present: this block expects the ORIGINAL images\x1b[0m");
        }

        let (dx, dy) = g.principal_offset_fraction();
        if dx > 0.05 || dy > 0.05 {
            warn(&format!(
                "principal point is {:.1}% / {:.1}% off centre; the solve may be degenerate",
                dx * 100.0,
                dy * 100.0
            ));
        }
        if !matches!(g.orientation, block::CameraOrientation::XRightYDown) {
            warn("camera axes are not XRightYDown; they will be reoriented on export");
        }
    }

    if verbose {
        rule("photos");
        for p in &block.photos {
            println!("  {:>6}  {:<40}  {}", p.id, p.basename(), p.c);
        }
    }
}

/// Name the file actually opened, and say so plainly when it was not the path
/// the user typed. Quietly substituting a different file would trade one
/// confusion for a worse one.
fn announce(found: &find::Found) {
    println!("\n\x1b[1m{}\x1b[0m", found.path().display());
    if matches!(found, find::Found::Discovered(_)) {
        println!("  (found from the folder you pointed at)");
    }
}

fn cmd_inspect(given: &std::path::Path, verbose: bool) -> Result<()> {
    let found = find::block_xml(given)?;
    announce(&found);
    let (block, stats) = block::parse_file(found.path())?;
    describe_block(&block, &stats, verbose);
    println!("\n  (inspect only — nothing written)\n");
    Ok(())
}

struct ExportRun {
    out: PathBuf,
    formats: Vec<Format>,
    cloud: Option<PathBuf>,
    max_points: usize,
    basenames: bool,
    force_pinhole: bool,
    allow_disjoint: bool,
    images: Vec<PathBuf>,
    keep_distortion: bool,
    quality: u8,
    max_distance: f64,
    overwrite: bool,
}

fn cmd_export(given: &std::path::Path, run: ExportRun) -> Result<()> {
    let ExportRun {
        out,
        formats,
        cloud: cloud_arg,
        max_points,
        basenames,
        force_pinhole,
        allow_disjoint,
        images,
        keep_distortion,
        quality,
        max_distance,
        overwrite,
    } = run;
    let out = out.as_path();
    let formats = formats.as_slice();
    let cloud_dir = cloud_arg.as_deref();

    let found = find::block_xml(given)?;
    let xml = found.path();
    let (mut block, stats) = block::parse_file(xml)?;
    announce(&found);
    describe_block(&block, &stats, false);

    block.normalize_orientation();
    let local = LocalFrame::build_and_apply(&mut block);

    rule("local frame");
    println!("  {}", local.describe());
    let e = block.extent();
    println!("  extent            {:.2} x {:.2} x {:.2} m", e.x, e.y, e.z);

    // ---- the images themselves ------------------------------------------
    if !images.is_empty() {
        let at = xml.parent().unwrap_or(std::path::Path::new("."));
        stage_images(
            &mut block,
            &local,
            at,
            &images,
            out,
            keep_distortion,
            quality,
            max_distance,
            overwrite,
        )?;
    }

    let needs_geo = formats.iter().any(|f| f.needs_georeference());
    if needs_geo && !local.is_georeferenced() {
        warn(
            "GeoJSON and KML need a geographic or geocentric block; this one is in a \
             projected CRS whose EPSG code is unknown here, so those outputs will be empty",
        );
    }

    // ---- initialisation cloud -------------------------------------------
    let mut cloud: Option<Cloud> = None;
    if let Some(dir) = cloud_dir {
        rule("initialisation cloud");
        let mut c = cloud::load_dir(dir, None)?;
        println!("  tiles read        {}", c.tiles_read);
        for (path, err) in &c.tiles_failed {
            warn(&format!(
                "skipped {}: {err}",
                path.file_name().unwrap_or_default().to_string_lossy()
            ));
        }
        println!("  points read       {}", thousands(c.source_count));
        if c.is_empty() {
            bail!("every cloud tile was unreadable");
        }

        let camera_centroid = block.centroid();
        let anchor = cloud::read_anchor(dir);
        let placed = cloud::place(&mut c, &local, anchor, camera_centroid);
        println!("  read as           {}", placed.label);

        let offset = c.centroid_offset_from(camera_centroid);
        let scale = block.extent().max_element().max(1.0);
        let ce = c.extent();
        println!("  cloud extent      {:.2} x {:.2} x {:.2} m", ce.x, ce.y, ce.z);
        println!(
            "  centroid offset   {} m  (scene scale {} m)",
            thousands(offset as usize),
            thousands(scale as usize)
        );

        if offset > 10.0 * scale {
            if allow_disjoint {
                warn("cloud does not overlap the cameras; writing it anyway as requested");
            } else {
                bail!(
                    "the cloud sits {:.0} m from the cameras, far outside the {:.0} m scene.\n\
                     It is probably in a different frame, or carries its own coordinate offset \
                     (check the metadata.xml beside the tiles).\n\
                     Re-run with --allow-disjoint-cloud to write it regardless, or omit \
                     --cloud and let the trainer use random initialisation.",
                    offset, scale
                );
            }
        } else {
            println!("  \x1b[36m→ cloud overlaps the cameras\x1b[0m");
        }

        c.decimate(max_points);
        println!("  points kept       {}", thousands(c.len()));
        cloud = Some(c);
    }

    // ---- write -----------------------------------------------------------
    std::fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;

    let opts = ExportOptions {
        out_dir: out,
        cloud: cloud.as_ref(),
        frame: &local,
        basenames,
        force_pinhole,
    };

    rule("written");
    let mut wanted: Vec<Format> = Vec::new();
    for f in formats {
        if !wanted.contains(f) {
            wanted.push(*f);
        }
    }
    for f in &wanted {
        let w = export::run(*f, &block, &opts)?;
        for file in &w.files {
            println!("  {:<14}  {}", w.format.label(), out.join(file).display());
        }
    }

    let georef = serde_json::json!({
        "source_xml": xml.display().to_string(),
        "block_name": block.name,
        "srs": block.srs.iter().map(|s| s.definition.clone()).collect::<Vec<_>>(),
        "photos": block.photos.len(),
        "frame": local,
    });
    let gpath = out.join("georeference.json");
    std::fs::write(&gpath, serde_json::to_string_pretty(&georef)?)?;
    println!("  {:<14}  {}", "metadata", gpath.display());

    println!(
        "\n  Put the images in {} and point a trainer at {}\n",
        out.join("images").display(),
        out.display()
    );
    Ok(())
}

/// Pair the block's poses with the photographs on the card and report how well
/// they line up, without writing anything.
fn describe_match(
    block: &Block,
    m: &stage::MatchReport,
    images: &[stage::SourceImage],
    skipped: usize,
    card: Option<stage::CardCheck>,
) {
    rule("photographs");
    println!("  found             {}", images.len());
    if skipped > 0 {
        println!("  without GPS       {skipped} (skipped)");
    }
    let fixed = stage::rtk_fixed_count(images);
    println!(
        "  RTK fixed         {} of {}",
        fixed,
        images.len()
    );
    if let Some(c) = card {
        println!("  aircraft logged   {} captures", c.captured);
        if !c.missing.is_empty() {
            let shown: Vec<String> = c.missing.iter().take(6).map(|i| i.to_string()).collect();
            warn(&format!(
                "{} capture(s) never reached the card (index {}{}); \
                 those poses cannot be matched and that is not a fault of the pairing",
                c.missing.len(),
                shown.join(", "),
                if c.missing.len() > shown.len() { ", ..." } else { "" }
            ));
        }
    }
    println!("  matched to poses  {} of {}", m.matched, block.photos.len());
    if m.unmatched > 0 {
        warn(&format!(
            "{} pose(s) had no photograph within range and will be left out",
            m.unmatched
        ));
    }
    if m.surplus_images > 0 {
        println!("  unused photos     {}", m.surplus_images);
    }
    println!("  median offset     {:.2} m", m.median_distance_m);
    println!("  worst offset      {:.2} m", m.worst_distance_m);

    // Shots are ~23 m apart on a normal grid flight while GNSS error is a
    // couple of metres, so a median in the tens of metres means the pairing has
    // slipped rather than that the GPS is poor.
    if m.median_distance_m > 10.0 {
        warn(
            "photographs are sitting further from their poses than GNSS error explains; \
             check that --images points at this flight and not another one",
        );
    } else {
        println!("  \x1b[36m→ photographs line up with the solved positions\x1b[0m");
    }
}

#[allow(clippy::too_many_arguments)]
fn stage_images(
    block: &mut Block,
    local: &LocalFrame,
    at_dir: &std::path::Path,
    images: &[PathBuf],
    out: &std::path::Path,
    keep_distortion: bool,
    quality: u8,
    max_distance: f64,
    overwrite: bool,
) -> Result<()> {
    let calib = stage::calibration_from_report(at_dir)?;

    let (found, skipped) = stage::scan_images(images)?;
    let card = stage::check_against_mrk(images, &found);
    let m = stage::match_by_position(block, local, &found, max_distance)?;
    describe_match(block, &m, &found, skipped, card);

    rule("images");
    let opts = stage::StageOptions {
        out_dir: out,
        jpeg_quality: quality,
        overwrite,
    };

    let report = if keep_distortion {
        let Some(c) = calib else {
            bail!(
                "--keep-distortion needs the lens the block was solved with, and it is \
                 recorded in AT/report/report.md, which could not be read here.\n\
                 Without it the originals would be paired with a camera model that \
                 describes a different lens. Omit --keep-distortion to rebuild \
                 undistorted images instead."
            );
        };
        println!("  staging the originals unchanged, distortion carried in the camera");
        // The XML describes Terra's virtual pinhole; the originals need the real
        // lens instead. Poses are untouched: they belong to the camera body.
        for g in &mut block.groups {
            g.fx = c.focal;
            g.fy = c.focal;
            g.cx = c.cx;
            g.cy = c.cy;
            g.distortion = block::Distortion {
                k1: c.k1,
                k2: c.k2,
                k3: c.k3,
                p1: c.p1,
                p2: c.p2,
            };
        }
        stage::copy_originals_into(block, &m, &opts)?
    } else {
        let Some(c) = calib else {
            bail!(
                "the block expects undistorted images, and the lens needed to make them \
                 is recorded in AT/report/report.md, which could not be read here.\n\
                 Re-run with --keep-distortion to use the original photographs as shot \
                 instead — that path resamples nothing and is always safe."
            );
        };
        if c.is_negligible() {
            warn("the report records no distortion, so the rebuilt images are copies");
        }
        println!(
            "  rebuilding undistorted images (K1 {:.6}, K2 {:.6}, K3 {:.6})",
            c.k1, c.k2, c.k3
        );
        stage::undistort_into(block, &m, c, &opts)?
    };

    println!("  written           {}", thousands(report.written));
    if report.skipped_existing > 0 {
        println!(
            "  already present   {} (use --overwrite to replace)",
            thousands(report.skipped_existing)
        );
    }
    for (p, e) in report.failed.iter().take(5) {
        warn(&format!(
            "{}: {e}",
            p.file_name().unwrap_or_default().to_string_lossy()
        ));
    }
    if report.failed.len() > 5 {
        warn(&format!("and {} more", report.failed.len() - 5));
    }
    if !keep_distortion {
        println!("  outside the frame {:.1}%", report.mean_out_of_bounds * 100.0);
        // A sign error or a wrong convention sends most of the output off the
        // edge of the source. Correct lens correction loses only the corners.
        if report.mean_out_of_bounds > 0.15 {
            warn(
                "much of each rebuilt image sampled from outside the original, which \
                 usually means the distortion model does not match these photographs",
            );
        }
    }
    Ok(())
}

fn cmd_match(given: &std::path::Path, images: &[PathBuf], max_distance: f64) -> Result<()> {
    let found_xml = find::block_xml(given)?;
    announce(&found_xml);
    let (mut block, stats) = block::parse_file(found_xml.path())?;
    describe_block(&block, &stats, false);
    block.normalize_orientation();
    let local = LocalFrame::build_and_apply(&mut block);

    let (found, skipped) = stage::scan_images(images)?;
    let card = stage::check_against_mrk(images, &found);
    let m = stage::match_by_position(&block, &local, &found, max_distance)?;
    describe_match(&block, &m, &found, skipped, card);
    println!("\n  (match only — nothing written)\n");
    Ok(())
}

fn cmd_cloud(dir: &std::path::Path, out: &std::path::Path, max_points: usize) -> Result<()> {
    let mut c = cloud::load_dir(dir, None)?;
    rule("cloud");
    println!("  tiles read        {}", c.tiles_read);
    for (path, err) in &c.tiles_failed {
        warn(&format!(
            "skipped {}: {err}",
            path.file_name().unwrap_or_default().to_string_lossy()
        ));
    }
    println!("  points read       {}", thousands(c.source_count));
    let e = c.extent();
    println!("  extent            {:.2} x {:.2} x {:.2}", e.x, e.y, e.z);
    println!("  centroid          {}", c.centroid());

    c.decimate(max_points);
    println!("  points kept       {}", thousands(c.len()));

    if let Some(parent) = out.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    // Reuse the PLY writer via a one-format export of a synthetic block is
    // overkill; write directly.
    write_ply_direct(out, &c)?;
    println!("  wrote             {}\n", out.display());
    Ok(())
}

fn write_ply_direct(path: &std::path::Path, c: &Cloud) -> Result<()> {
    use std::io::{BufWriter, Write};
    let mut w = BufWriter::with_capacity(1 << 20, std::fs::File::create(path)?);
    writeln!(w, "ply")?;
    writeln!(w, "format binary_little_endian 1.0")?;
    writeln!(w, "comment generated by terrapin")?;
    writeln!(w, "element vertex {}", c.len())?;
    for a in ["x", "y", "z"] {
        writeln!(w, "property double {a}")?;
    }
    for a in ["red", "green", "blue"] {
        writeln!(w, "property uchar {a}")?;
    }
    writeln!(w, "end_header")?;
    for (p, col) in c.points.iter().zip(&c.colors) {
        for v in p.to_array() {
            w.write_all(&v.to_le_bytes())?;
        }
        w.write_all(col)?;
    }
    w.flush()?;
    Ok(())
}

fn cmd_report(given: &std::path::Path, verbose: bool) -> Result<()> {
    let found = find::at_dir(given)?;
    announce(&found);
    let r = report::load(found.path())?;

    if let Some(md) = &r.markdown {
        rule("report.md");
        println!("{md}");
    }

    if r.highlights.is_empty() && r.fields.is_empty() {
        warn("no sfm_report.json found; is this the AT directory?");
    } else {
        rule("quality metrics");
        for (k, v) in &r.highlights {
            println!("  {k:<48} {v}");
        }
    }

    if let Some(s) = &r.pos_residuals {
        rule("POS residuals");
        println!("  rows              {}", s.rows);
        println!("  column used       {}", s.column);
        println!("  mean              {:.6}", s.mean);
        println!("  median            {:.6}", s.median);
        println!("  95th percentile   {:.6}", s.p95);
        println!("  max               {:.6}", s.max);
        println!("  outliers (>3x)    {}", s.outliers);
        if s.outliers > s.rows / 20 {
            warn("more than 5% of cameras are residual outliers; check GNSS quality");
        }
    }

    if verbose && !r.fields.is_empty() {
        rule("all fields");
        for (k, v) in &r.fields {
            println!("  {k:<56} {v}");
        }
    }
    println!();
    Ok(())
}

fn thousands(n: usize) -> String {
    let s = n.to_string();
    let mut out = String::with_capacity(s.len() + s.len() / 3);
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(ch);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn cli_definition_is_valid() {
        Cli::command().debug_assert();
    }

    #[test]
    fn thousands_separates_correctly() {
        assert_eq!(thousands(0), "0");
        assert_eq!(thousands(999), "999");
        assert_eq!(thousands(1000), "1,000");
        assert_eq!(thousands(1_680_000), "1,680,000");
    }

    #[test]
    fn formats_parse_from_a_comma_list() {
        let cli = Cli::try_parse_from([
            "terrapin", "export", "a.xml", "--format", "colmap-txt,nerfstudio,geojson",
        ])
        .unwrap();
        match cli.command {
            Command::Export { format, .. } => {
                assert_eq!(
                    format,
                    vec![Format::ColmapTxt, Format::Nerfstudio, Format::Geojson]
                );
            }
            _ => panic!("wrong subcommand"),
        }
    }
}
