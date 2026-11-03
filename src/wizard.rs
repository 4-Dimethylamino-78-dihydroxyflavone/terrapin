//! The guided path, for people who did not come here to learn a command line.
//!
//! This is what runs when the program is double-clicked, or when a folder is
//! dropped onto it. It asks a short series of questions in plain language,
//! every one with a safe answer on Enter, and works out everything it can for
//! itself first — where the photographs are, where the cloud is, what the
//! project is called — so that most of the questions are confirmations rather
//! than interrogations.
//!
//! It builds a plan and hands it back rather than running anything. The guided
//! path and the flag path then go through exactly one piece of machinery, so
//! there is no second implementation to drift. It also prints the equivalent
//! command at the end: whoever runs this twice should be able to run it a third
//! time without the questions, and nothing teaches that as well as seeing the
//! command their own answers just built.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use dialoguer::{Confirm, Input, Select};
use walkdir::WalkDir;

use crate::block;
use crate::export::Format;
use crate::find;
use crate::frame::UpAxis;
use crate::report;
use crate::ui;

pub struct Plan {
    pub project: PathBuf,
    pub out: PathBuf,
    pub formats: Vec<Format>,
    pub images: Vec<PathBuf>,
    pub cloud: Option<PathBuf>,
    pub keep_distortion: bool,
    pub up: UpAxis,
}

/// What the answer to "which software?" actually means downstream.
struct Target {
    prompt: &'static str,
    formats: &'static [Format],
    /// Whether this target would rather have the photographs as shot.
    keep_distortion: bool,
}

const TARGETS: &[Target] = &[
    Target {
        prompt: "Brush, LichtFeld Studio, Postshot or gsplat  (a COLMAP folder)",
        formats: &[Format::ColmapBin, Format::ColmapTxt],
        keep_distortion: false,
    },
    Target {
        prompt: "nerfstudio or instant-ngp  (transforms.json)",
        formats: &[Format::Nerfstudio, Format::ColmapBin],
        keep_distortion: false,
    },
    Target {
        prompt: "Metashape or RealityScan  (keeps your photographs untouched)",
        formats: &[Format::Bundler, Format::Csv],
        keep_distortion: true,
    },
    Target {
        prompt: "I am not sure — write everything",
        formats: &[
            Format::ColmapBin,
            Format::ColmapTxt,
            Format::Nerfstudio,
            Format::Bundler,
            Format::Ply,
            Format::Geojson,
            Format::Kml,
            Format::Csv,
        ],
        keep_distortion: false,
    },
];

// ---------------------------------------------------------------------------
// Working things out so they do not have to be asked
// ---------------------------------------------------------------------------

fn count_jpegs(dir: &Path) -> usize {
    std::fs::read_dir(dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok())
                .filter(|e| {
                    e.path()
                        .extension()
                        .and_then(|s| s.to_str())
                        .map(|s| s.eq_ignore_ascii_case("jpg") || s.eq_ignore_ascii_case("jpeg"))
                        .unwrap_or(false)
                })
                .count()
        })
        .unwrap_or(0)
}

/// Look for the aircraft's card near the project, and rank what is found by how
/// close its photograph count is to the number of poses.
///
/// A card usually holds several flights, only one of which produced this block,
/// so the count is the strongest cheap signal available for which is which.
fn find_image_candidates(project: &Path, want: usize) -> Vec<(PathBuf, usize)> {
    let mut out: Vec<(PathBuf, usize)> = Vec::new();
    let mut seen: Vec<PathBuf> = Vec::new();

    let mut roots: Vec<PathBuf> = Vec::new();
    let mut cur = project.to_path_buf();
    for _ in 0..3 {
        match cur.parent() {
            Some(p) => {
                roots.push(p.to_path_buf());
                cur = p.to_path_buf();
            }
            None => break,
        }
    }
    roots.push(project.to_path_buf());

    for root in roots {
        let Ok(rd) = std::fs::read_dir(&root) else {
            continue;
        };
        for e in rd.filter_map(|e| e.ok()) {
            let p = e.path();
            if !p.is_dir() {
                continue;
            }
            let is_dcim = p
                .file_name()
                .and_then(|s| s.to_str())
                .map(|s| s.eq_ignore_ascii_case("dcim"))
                .unwrap_or(false);
            if !is_dcim {
                continue;
            }
            // The flights live one level inside DCIM.
            let Ok(inner) = std::fs::read_dir(&p) else {
                continue;
            };
            for f in inner.filter_map(|e| e.ok()) {
                let d = f.path();
                if !d.is_dir() || seen.contains(&d) {
                    continue;
                }
                let n = count_jpegs(&d);
                if n > 0 {
                    seen.push(d.clone());
                    out.push((d, n));
                }
            }
            let n = count_jpegs(&p);
            if n > 0 && !seen.contains(&p) {
                seen.push(p.clone());
                out.push((p, n));
            }
        }
    }

    out.sort_by_key(|(_, n)| (*n as i64 - want as i64).abs());
    out
}

/// Terra's cloud, preferring the tiles that carry colour.
fn find_cloud(project: &Path) -> Option<(PathBuf, bool)> {
    let mut las = None;
    let mut ply = None;
    for e in WalkDir::new(project)
        .max_depth(5)
        .into_iter()
        .filter_map(|e| e.ok())
    {
        if !e.file_type().is_dir() {
            continue;
        }
        match e.path().file_name().and_then(|s| s.to_str()) {
            Some(n) if n.eq_ignore_ascii_case("terra_las_1_4") => {
                las.get_or_insert_with(|| e.path().to_path_buf());
            }
            Some(n) if n.eq_ignore_ascii_case("terra_ply") => {
                ply.get_or_insert_with(|| e.path().to_path_buf());
            }
            _ => {}
        }
    }
    las.map(|p| (p, true)).or_else(|| ply.map(|p| (p, false)))
}

fn clean_input(raw: &str) -> PathBuf {
    find::clean(raw)
}

/// The handful of rows in Terra's `report.md` that say whether the solve is
/// worth spending GPU time on.
///
/// `report.rs` deliberately surfaces every numeric leaf it can find, which is
/// right for `terrapin report` and wrong here: a beginner asked to judge an
/// alignment should not have to pick the two useful numbers out of a
/// calibration serial and a GSD. The table shape is undocumented, so an
/// unfamiliar one yields nothing rather than failing.
fn quality_rows(md: &str) -> Vec<(String, String)> {
    const WANTED: &[(&str, &str)] = &[
        ("# Input Images", "photographs"),
        ("# Calibrated Images", "aligned"),
        ("Georeferencing RMSE", "georeferencing error"),
        ("# Connected Components", "separate pieces"),
    ];
    let mut out = Vec::new();
    for line in md.lines() {
        if !line.contains('|') {
            continue;
        }
        let cells: Vec<&str> = line
            .split('|')
            .map(str::trim)
            .filter(|c| !c.is_empty())
            .collect();
        if cells.len() < 2 {
            continue;
        }
        for (needle, friendly) in WANTED {
            if cells[0].contains(needle) {
                out.push((friendly.to_string(), cells[1].to_string()));
            }
        }
    }
    out
}

/// Input and calibrated counts, when both were found.
fn aligned_counts(rows: &[(String, String)]) -> Option<(u32, u32)> {
    let get = |name: &str| {
        rows.iter()
            .find(|(l, _)| l == name)
            .and_then(|(_, v)| v.trim().parse::<u32>().ok())
    };
    Some((get("photographs")?, get("aligned")?))
}

// ---------------------------------------------------------------------------

fn heading(text: &str) {
    println!("\n{}", ui::bold(text));
    println!("{}", "─".repeat(text.chars().count().max(24)));
}

/// Ask the questions and return the plan, or `None` if the user backed out.
pub fn plan(start: Option<PathBuf>) -> Result<Option<Plan>> {
    println!("\n{}", ui::bold("terrapin"));
    println!(
        "{}",
        ui::dim("Turns a DJI Terra project into something a Gaussian-splat trainer can read.")
    );

    // ---- 1. the project --------------------------------------------------
    let found = loop {
        let given = match &start {
            Some(p) => p.clone(),
            None => {
                println!(
                    "\n{}",
                    ui::dim(
                        "Drag the Terra project folder onto this window and press Enter,\n\
                         or paste its path. It is the folder containing AT, models and records."
                    )
                );
                let raw: String = Input::new()
                    .with_prompt("Project folder")
                    .interact_text()
                    .context("reading the project folder")?;
                clean_input(&raw)
            }
        };

        match find::block_xml(&given) {
            Ok(f) => break f,
            Err(e) => {
                println!("\n{}", ui::red(&format!("{e}")));
                if start.is_some() {
                    // The path came from a drag-and-drop, so there is nothing to
                    // re-ask about unless we start prompting.
                    println!("{}", ui::dim("Try again with the project folder itself."));
                    return Ok(None);
                }
            }
        }
    };

    let xml = found.path().to_path_buf();
    let at_dir = xml.parent().unwrap_or(Path::new(".")).to_path_buf();
    let project = at_dir.parent().unwrap_or(&at_dir).to_path_buf();

    heading("what is in it");
    let (blk, stats) = block::parse_file(&xml)?;
    println!("  project           {}", project.display());
    println!(
        "  photographs       {} posed of {} seen",
        blk.photos.len(),
        stats.photos_seen
    );
    if let Some(g) = blk.groups.first() {
        println!("  camera            {} x {} pixels", g.width, g.height);
    }
    if !blk.srs.is_empty() {
        println!("  coordinates       {}", blk.srs[0].definition);
    }

    let wants_undistorted = blk
        .groups
        .first()
        .map(|g| g.distortion.is_negligible())
        .unwrap_or(false);
    if wants_undistorted {
        println!(
            "  {}",
            ui::cyan("→ this project needs its photographs corrected for lens distortion first")
        );
    } else {
        println!(
            "  {}",
            ui::cyan("→ this project uses your photographs exactly as they were shot")
        );
    }

    // ---- 2. is the solve worth the GPU time? -----------------------------
    heading("how good is the alignment");
    match report::load(&at_dir) {
        Ok(r) => {
            let rows = r.markdown.as_deref().map(quality_rows).unwrap_or_default();
            let mut said = false;
            for (label, value) in &rows {
                println!("  {label:<28} {value}");
                said = true;
            }
            if let Some(s) = &r.pos_residuals {
                println!("  {:<28} {:.3} m", "median GPS residual", s.median);
                if s.outliers > s.rows / 20 {
                    println!(
                        "  {}",
                        ui::yellow(&format!(
                            "{} of {} cameras disagree badly with their GPS",
                            s.outliers, s.rows
                        ))
                    );
                }
                said = true;
            }
            if let Some((input, calibrated)) = aligned_counts(&rows) {
                if calibrated < input {
                    println!(
                        "  {}",
                        ui::yellow(&format!(
                            "{} of {} photographs did not align",
                            input - calibrated,
                            input
                        ))
                    );
                } else {
                    println!("  {}", ui::cyan("→ every photograph aligned"));
                }
            }
            if !said {
                println!("  {}", ui::dim("no quality report found beside the block"));
            }
        }
        Err(_) => println!("  {}", ui::dim("no quality report found beside the block")),
    }
    println!(
        "\n{}",
        ui::dim(
            "If a lot of photographs failed to align, no trainer will rescue that —\n\
             it is cheaper to re-run the alignment in Terra than to train on it."
        )
    );
    if !Confirm::new()
        .with_prompt("Carry on")
        .default(true)
        .interact()?
    {
        return Ok(None);
    }

    // ---- 3. the target ---------------------------------------------------
    heading("what will you open the result in");
    let items: Vec<&str> = TARGETS.iter().map(|t| t.prompt).collect();
    let pick = Select::new()
        .with_prompt("Choose")
        .items(&items)
        .default(0)
        .interact()?;
    let target = &TARGETS[pick];

    // ---- 4. the photographs ---------------------------------------------
    let mut images: Vec<PathBuf> = Vec::new();
    let need_images = wants_undistorted || target.keep_distortion;
    if need_images {
        heading("where are the original photographs");
        println!(
            "{}",
            ui::dim(
                "Terra made its corrected copies in a temporary folder that is not part of\n\
                 the project, so they are rebuilt from the originals on the card."
            )
        );
        let candidates = find_image_candidates(&project, blk.photos.len());
        let chosen = match candidates.first() {
            Some((p, n)) if *n == blk.photos.len() => {
                println!(
                    "\n  found {} photographs in\n  {}",
                    n,
                    ui::bold(&p.display().to_string())
                );
                if Confirm::new()
                    .with_prompt("Use these")
                    .default(true)
                    .interact()?
                {
                    Some(p.clone())
                } else {
                    None
                }
            }
            Some((p, n)) => {
                println!(
                    "\n  closest match is {} photographs in\n  {}\n  {}",
                    n,
                    ui::bold(&p.display().to_string()),
                    ui::yellow(&format!("(the project has {} poses)", blk.photos.len()))
                );
                if Confirm::new()
                    .with_prompt("Use these")
                    .default(false)
                    .interact()?
                {
                    Some(p.clone())
                } else {
                    None
                }
            }
            None => None,
        };

        let chosen = match chosen {
            Some(p) => p,
            None => {
                let raw: String = Input::new()
                    .with_prompt("Folder holding the photographs")
                    .interact_text()?;
                clean_input(&raw)
            }
        };
        images.push(chosen);
    }

    // ---- 5. the cloud ----------------------------------------------------
    let mut cloud = None;
    heading("seed the trainer with terra's point cloud");
    println!(
        "{}",
        ui::dim(
            "Optional. It gives the trainer somewhere to start instead of random noise,\n\
             which usually converges faster and cleaner."
        )
    );
    if let Some((path, coloured)) = find_cloud(&project) {
        println!(
            "\n  found {}\n  {}",
            if coloured {
                ui::cyan("tiles with colour")
            } else {
                ui::yellow("tiles without per-point colour")
            },
            ui::bold(&path.display().to_string())
        );
        if Confirm::new()
            .with_prompt("Use it")
            .default(true)
            .interact()?
        {
            cloud = Some(path);
        }
    } else {
        println!("  {}", ui::dim("none found in this project"));
    }

    // ---- 6. orientation --------------------------------------------------
    heading("which way is up");
    println!(
        "{}",
        ui::dim(
            "Trainers and mapping software want up on Z, which is how the world is\n\
             actually oriented. Viewers like Blender, F3D and three.js want up on Y."
        )
    );
    let up = if Confirm::new()
        .with_prompt("Will you open this in a 3D viewer rather than a trainer")
        .default(false)
        .interact()?
    {
        UpAxis::Y
    } else {
        UpAxis::Z
    };

    // ---- 7. where it goes ------------------------------------------------
    heading("where should the result go");
    let default_out = project.parent().unwrap_or(Path::new(".")).join(format!(
        "{}_splat",
        project
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "terrapin".into())
    ));
    let raw: String = Input::new()
        .with_prompt("Output folder")
        .default(default_out.display().to_string())
        .interact_text()?;
    let out = clean_input(&raw);

    if out.exists()
        && std::fs::read_dir(&out)
            .map(|mut d| d.next().is_some())
            .unwrap_or(false)
    {
        println!(
            "\n  {}",
            ui::yellow("that folder already has something in it; existing files are kept")
        );
    }

    // ---- confirm ---------------------------------------------------------
    heading("ready");
    println!("  project           {}", project.display());
    println!("  output            {}", out.display());
    println!(
        "  writing           {}",
        target
            .formats
            .iter()
            .map(|f| f.label())
            .collect::<Vec<_>>()
            .join(", ")
    );
    if let Some(p) = images.first() {
        println!("  photographs       {}", p.display());
        println!(
            "  {}",
            ui::dim(if target.keep_distortion {
                "  they will be copied across untouched"
            } else {
                "  they will be corrected for lens distortion — this takes a few minutes"
            })
        );
    }
    match &cloud {
        Some(c) => println!("  cloud             {}", c.display()),
        None => println!("  cloud             none — the trainer will start from noise"),
    }
    if up != UpAxis::Z {
        println!("  orientation       {}", up.label());
    }

    if !Confirm::new()
        .with_prompt("\nGo")
        .default(true)
        .interact()?
    {
        return Ok(None);
    }

    Ok(Some(Plan {
        project: xml,
        out,
        formats: target.formats.to_vec(),
        images,
        cloud,
        keep_distortion: target.keep_distortion,
        up,
    }))
}

fn quoted(p: &Path) -> String {
    let s = p.display().to_string();
    if s.contains(' ') {
        format!("\"{s}\"")
    } else {
        s
    }
}

/// Show the command these answers amount to, so the next run need not ask.
pub fn show_equivalent(plan: &Plan) {
    let exe = std::env::current_exe()
        .ok()
        .map(|p| quoted(&p))
        .unwrap_or_else(|| "terrapin".into());

    let mut cmd = format!("{exe} export {}", quoted(&plan.project));
    cmd.push_str(&format!(" --out {}", quoted(&plan.out)));
    cmd.push_str(&format!(
        " --format {}",
        plan.formats
            .iter()
            .map(|f| f.label())
            .collect::<Vec<_>>()
            .join(",")
    ));
    for i in &plan.images {
        cmd.push_str(&format!(" --images {}", quoted(i)));
    }
    if let Some(c) = &plan.cloud {
        cmd.push_str(&format!(" --cloud {}", quoted(c)));
    }
    if plan.keep_distortion {
        cmd.push_str(" --keep-distortion");
    }
    if plan.up != UpAxis::Z {
        cmd.push_str(" --up-axis y");
    }

    heading("the same thing, without the questions");
    println!("{}", ui::dim("Next time you can run this directly:"));
    println!("\n{cmd}\n");
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Copied verbatim out of the quarry block's report.md.
    const REPORT: &str = "\
## Image Information Overview

| Item                    | Value           |
| :---: | :---: |
| # Input Images          |             1680|
| # Image With Position   |             1680|
| # Calibrated Images     |             1680|
| Use Image Position | True|
| Georeferencing RMSE |   0.152 m|
| # Connected Components     |            1|
| # feature quantity | Medium|
";

    #[test]
    fn picks_out_the_rows_that_decide_whether_to_train() {
        let rows = quality_rows(REPORT);
        let get = |k: &str| rows.iter().find(|(l, _)| l == k).map(|(_, v)| v.as_str());
        assert_eq!(get("photographs"), Some("1680"));
        assert_eq!(get("aligned"), Some("1680"));
        assert_eq!(get("georeferencing error"), Some("0.152 m"));
        assert_eq!(get("separate pieces"), Some("1"));
        // Calibration serials and GSD are noise at this point in the flow.
        assert_eq!(rows.len(), 4, "picked up something extra: {rows:?}");
    }

    #[test]
    fn notices_when_photographs_failed_to_align() {
        assert_eq!(aligned_counts(&quality_rows(REPORT)), Some((1680, 1680)));

        let partial = REPORT.replace(
            "| # Calibrated Images     |             1680|",
            "| # Calibrated Images     |             1204|",
        );
        assert_eq!(aligned_counts(&quality_rows(&partial)), Some((1680, 1204)));
    }

    #[test]
    fn an_unfamiliar_report_yields_nothing_rather_than_failing() {
        assert!(quality_rows("no tables here at all").is_empty());
        assert_eq!(aligned_counts(&[]), None);
    }

    #[test]
    fn paths_with_spaces_survive_the_replay_command() {
        assert_eq!(quoted(Path::new("D:\\a b\\c")), "\"D:\\a b\\c\"");
        assert_eq!(quoted(Path::new("D:\\plain")), "D:\\plain");
    }
}
