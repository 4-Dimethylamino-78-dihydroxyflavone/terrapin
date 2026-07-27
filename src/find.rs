//! Finding what the user meant.
//!
//! Nobody arrives holding a path to `AT/BlocksExchange*.xml`. They arrive with
//! the folder Terra made, or the folder above it, or whichever folder they were
//! last looking at — and on Windows, usually with the double quotes that
//! Explorer's "Copy as path" wraps around it.
//!
//! Handing a directory to `File::open` reports `Access is denied. (os error 5)`
//! on Windows. That is not a permissions problem, but it reads exactly like one,
//! and it is the last thing said before the tool gives up. macOS at least says
//! `Is a directory`. Nobody new to a terminal recovers from the Windows version
//! unaided, so this module makes the mistake impossible instead of diagnosable:
//! point at any level of a Terra project and the block is found from there.

use std::path::{Path, PathBuf};

use anyhow::{bail, Result};
use walkdir::WalkDir;

/// How far below a named directory to look. The block sits at `<project>/AT/`,
/// so 4 covers being handed the folder holding several projects.
const DESCEND: usize = 4;

/// How far above a named directory to look, for someone who started inside
/// `models/pc/0/terra_ply` and expected the tool to work it out.
const ASCEND: usize = 5;

/// Strip the decorations a pasted path arrives wearing.
///
/// Quotes survive when a path is pasted into a prompt rather than a shell, and
/// a trailing separator is harmless to the OS but makes `file_name()` return
/// the parent — which turns a good path into a confusing one several steps
/// later, far from the cause.
pub fn clean(raw: &str) -> PathBuf {
    let mut s = raw.trim();
    for q in ['"', '\''] {
        if s.len() >= 2 && s.starts_with(q) && s.ends_with(q) {
            s = &s[1..s.len() - 1];
            break;
        }
    }
    let s = s.trim();
    let trimmed = s.trim_end_matches(['/', '\\']);
    // `C:\` and `/` are legitimately nothing but a separator; leave those be.
    let kept = if trimmed.is_empty() || trimmed.ends_with(':') {
        s
    } else {
        trimmed
    };
    PathBuf::from(kept)
}

/// Same, for a path that arrived through clap rather than a prompt.
pub fn clean_path(p: &Path) -> PathBuf {
    clean(&p.to_string_lossy())
}

fn is_block_xml(p: &Path) -> bool {
    match p.file_name().and_then(|s| s.to_str()) {
        Some(name) => {
            let lower = name.to_ascii_lowercase();
            lower.starts_with("blocksexchange") && lower.ends_with(".xml")
        }
        None => false,
    }
}

fn scan(root: &Path, depth: usize) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = WalkDir::new(root)
        .max_depth(depth)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_file() && is_block_xml(e.path()))
        .map(|e| e.path().to_path_buf())
        .collect();
    out.sort();
    out
}

/// What the search had to do, so the caller can tell the user how a path they
/// did not type was arrived at. Silently substituting a different file would be
/// worse than the error it replaces.
pub enum Found {
    /// The user named the file exactly.
    AsGiven(PathBuf),
    /// It was discovered beneath or above what they named.
    Discovered(PathBuf),
}

impl Found {
    pub fn path(&self) -> &Path {
        match self {
            Self::AsGiven(p) | Self::Discovered(p) => p,
        }
    }

    pub fn into_path(self) -> PathBuf {
        match self {
            Self::AsGiven(p) | Self::Discovered(p) => p,
        }
    }
}

/// Resolve anything inside a Terra project to its BlocksExchange XML.
pub fn block_xml(start: &Path) -> Result<Found> {
    let start = clean_path(start);

    if !start.exists() {
        bail!(
            "there is nothing at {}.\n\
             Check for a typo, and note that a path copied from Explorer may still \
             have quotes around it.",
            start.display()
        );
    }

    if start.is_file() {
        if is_block_xml(&start) || start.extension().is_some_and(|e| e == "xml") {
            return Ok(Found::AsGiven(start));
        }
        bail!(
            "{} is not a BlocksExchange XML.\n\
             Point at the file named BlocksExchange*.xml inside the project's AT \
             folder, or just at the project folder and it will be found for you.",
            start.display()
        );
    }

    let mut hits = scan(&start, DESCEND);

    if hits.is_empty() {
        let mut cur = start.clone();
        for _ in 0..ASCEND {
            let parent = match cur.parent() {
                Some(p) => p.to_path_buf(),
                None => break,
            };
            let up = scan(&parent, 2);
            if !up.is_empty() {
                hits = up;
                break;
            }
            cur = parent;
        }
    }

    match hits.len() {
        0 => bail!(
            "no BlocksExchange XML was found in or around {}.\n\
             A DJI Terra project holds one in its AT folder, named something like \
             AT/BlocksExchangeUndistortAT_WithoutTiePoints.xml. Point at the project \
             folder — the one containing AT, models and records — or at that file \
             directly.",
            start.display()
        ),
        1 => Ok(Found::Discovered(hits.remove(0))),
        _ => {
            let list = hits
                .iter()
                .map(|p| format!("  {}", p.display()))
                .collect::<Vec<_>>()
                .join("\n");
            bail!(
                "{} contains {} BlocksExchange files and they describe different \
                 solves, so picking one for you would be a guess:\n{}\n\
                 Re-run naming the one you want. If you are unsure, the file with \
                 'Undistort' in its name is the one Terra prepared for export.",
                start.display(),
                hits.len(),
                list
            )
        }
    }
}

/// Resolve anything inside a project to its `AT` directory, for `report`.
pub fn at_dir(start: &Path) -> Result<Found> {
    let start = clean_path(start);

    if start.is_dir() {
        if start.join("report").is_dir() || start.join("sfm_report.json").is_file() {
            return Ok(Found::AsGiven(start));
        }
        let at = start.join("AT");
        if at.is_dir() {
            return Ok(Found::Discovered(at));
        }
    }

    // Fall back on wherever the block turned out to live: that folder is the AT
    // directory by construction.
    let xml = block_xml(&start)?;
    match xml.path().parent() {
        Some(p) => Ok(Found::Discovered(p.to_path_buf())),
        None => bail!("could not work out the AT folder from {}", start.display()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clean_strips_the_quotes_explorer_adds() {
        assert_eq!(clean("\"D:\\a b\\c\""), PathBuf::from("D:\\a b\\c"));
        assert_eq!(clean("'/Users/x/y'"), PathBuf::from("/Users/x/y"));
        assert_eq!(clean("  D:\\a  "), PathBuf::from("D:\\a"));
    }

    #[test]
    fn clean_strips_a_trailing_separator_but_keeps_a_bare_root() {
        assert_eq!(clean("D:\\a\\b\\"), PathBuf::from("D:\\a\\b"));
        assert_eq!(clean("/a/b/"), PathBuf::from("/a/b"));
        assert_eq!(clean("C:\\"), PathBuf::from("C:\\"));
        assert_eq!(clean("/"), PathBuf::from("/"));
    }

    #[test]
    fn recognises_block_files_case_insensitively() {
        assert!(is_block_xml(Path::new(
            "AT/BlocksExchangeUndistortAT_WithoutTiePoints.xml"
        )));
        assert!(is_block_xml(Path::new("x/blocksexchange.XML")));
        assert!(!is_block_xml(Path::new("AT/camera_info.json")));
        assert!(!is_block_xml(Path::new("AT/other.xml")));
    }

    fn scratch(name: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("terrapin_find_{name}"));
        std::fs::remove_dir_all(&p).ok();
        p
    }

    #[test]
    fn a_project_folder_resolves_to_the_block_inside_it() {
        let root = scratch("project");
        let at = root.join("Quarry Test 1").join("AT");
        std::fs::create_dir_all(&at).unwrap();
        std::fs::write(at.join("BlocksExchangeUndistortAT.xml"), b"<x/>").unwrap();

        let found = block_xml(&root.join("Quarry Test 1")).unwrap();
        assert!(matches!(found, Found::Discovered(_)));
        assert_eq!(found.path().file_name().unwrap(), "BlocksExchangeUndistortAT.xml");
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn a_folder_deep_inside_the_project_still_finds_it() {
        let root = scratch("ascend");
        let at = root.join("AT");
        let deep = root.join("models").join("pc").join("0").join("terra_ply");
        std::fs::create_dir_all(&at).unwrap();
        std::fs::create_dir_all(&deep).unwrap();
        std::fs::write(at.join("BlocksExchange.xml"), b"<x/>").unwrap();

        assert!(block_xml(&deep).is_ok(), "should ascend to the sibling AT folder");
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn a_folder_with_no_project_says_so_usefully() {
        let root = scratch("empty");
        std::fs::create_dir_all(&root).unwrap();
        let err = block_xml(&root).unwrap_err().to_string();
        assert!(err.contains("no BlocksExchange XML"), "unhelpful: {err}");
        assert!(err.contains("AT folder"), "must say what to look for: {err}");
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn several_candidates_are_listed_rather_than_guessed_between() {
        let root = scratch("ambiguous");
        let at = root.join("AT");
        std::fs::create_dir_all(&at).unwrap();
        std::fs::write(at.join("BlocksExchangeAT.xml"), b"<x/>").unwrap();
        std::fs::write(at.join("BlocksExchangeUndistortAT.xml"), b"<x/>").unwrap();

        let err = block_xml(&root).unwrap_err().to_string();
        assert!(err.contains("BlocksExchangeAT.xml"), "must list them: {err}");
        assert!(err.contains("BlocksExchangeUndistortAT.xml"));
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn a_missing_path_is_not_reported_as_a_permissions_problem() {
        let err = block_xml(Path::new("D:\\definitely\\not\\here\\at\\all"))
            .unwrap_err()
            .to_string();
        assert!(err.contains("nothing at"), "{err}");
        assert!(!err.to_lowercase().contains("denied"));
    }
}
