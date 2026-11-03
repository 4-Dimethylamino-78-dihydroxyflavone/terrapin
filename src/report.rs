//! Reading Terra's aerotriangulation quality files.
//!
//! Terra's `sfm_report.json` schema is undocumented and has changed between
//! versions, so this deliberately does not model it. Instead it flattens the
//! whole document and surfaces the numeric leaves whose key names suggest
//! quality metrics. That degrades gracefully: a schema change costs you a
//! nicely-labelled summary, not a crash.

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result};
use serde_json::Value;

/// Key fragments worth pulling to the top of the summary.
const INTERESTING: &[&str] = &[
    "reproj", "error", "rms", "residual", "calibrat", "register", "align", "tie", "match", "photo",
    "image", "camera", "count", "num", "total", "median", "mean", "max", "min", "gsd", "overlap",
];

#[derive(Debug, Default)]
pub struct AtReport {
    /// Flattened `path -> value` for every numeric or short string leaf.
    pub fields: BTreeMap<String, String>,
    /// The subset whose key looked like a quality metric.
    pub highlights: BTreeMap<String, String>,
    pub pos_residuals: Option<ResidualStats>,
    pub markdown: Option<String>,
}

#[derive(Debug, Clone, Copy)]
pub struct ResidualStats {
    pub rows: usize,
    pub column: usize,
    pub mean: f64,
    pub median: f64,
    pub p95: f64,
    pub max: f64,
    /// Rows whose residual exceeds three times the median.
    pub outliers: usize,
}

pub fn load(at_dir: &Path) -> Result<AtReport> {
    let mut out = AtReport::default();
    let report_dir = at_dir.join("report");

    let sfm = report_dir.join("sfm_report.json");
    if sfm.is_file() {
        let text =
            std::fs::read_to_string(&sfm).with_context(|| format!("reading {}", sfm.display()))?;
        let v: Value = serde_json::from_str(&text)
            .with_context(|| format!("{} is not valid JSON", sfm.display()))?;
        flatten(&v, String::new(), &mut out.fields);
        for (k, val) in &out.fields {
            let lower = k.to_lowercase();
            if INTERESTING.iter().any(|frag| lower.contains(frag)) {
                out.highlights.insert(k.clone(), val.clone());
            }
        }
    }

    let md = report_dir.join("report.md");
    if md.is_file() {
        if let Ok(s) = std::fs::read_to_string(&md) {
            let trimmed = s.trim();
            if !trimmed.is_empty() {
                out.markdown = Some(trimmed.to_string());
            }
        }
    }

    let pos = report_dir.join("POS_residual_of_camera.csv");
    if pos.is_file() {
        out.pos_residuals = residual_stats(&pos).ok();
    }

    Ok(out)
}

fn flatten(v: &Value, prefix: String, out: &mut BTreeMap<String, String>) {
    match v {
        Value::Object(map) => {
            for (k, val) in map {
                let key = if prefix.is_empty() {
                    k.clone()
                } else {
                    format!("{prefix}.{k}")
                };
                flatten(val, key, out);
            }
        }
        Value::Array(items) => {
            // Long arrays are per-photo detail, not summary; record the length
            // and the first element's shape only.
            if items.len() > 8 {
                out.insert(format!("{prefix}.length"), items.len().to_string());
            } else {
                for (i, item) in items.iter().enumerate() {
                    flatten(item, format!("{prefix}[{i}]"), out);
                }
            }
        }
        Value::Number(n) => {
            out.insert(prefix, n.to_string());
        }
        Value::Bool(b) => {
            out.insert(prefix, b.to_string());
        }
        Value::String(s) if s.len() <= 120 => {
            out.insert(prefix, s.clone());
        }
        _ => {}
    }
}

/// Summarise the widest-spread numeric column in the POS residual CSV.
///
/// The column layout is not documented, so rather than guessing which column
/// is the residual, this picks the numeric column with the largest spread —
/// which in practice is the residual rather than an index or a constant.
fn residual_stats(path: &Path) -> Result<ResidualStats> {
    let text = std::fs::read_to_string(path)?;
    let mut rows: Vec<Vec<Option<f64>>> = Vec::new();
    for (i, line) in text.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let parsed: Vec<Option<f64>> = line
            .split(',')
            .map(|f| f.trim().parse::<f64>().ok())
            .collect();
        // Skip a header row: it parses to all-None.
        if i == 0 && parsed.iter().all(Option::is_none) {
            continue;
        }
        rows.push(parsed);
    }
    anyhow::ensure!(!rows.is_empty(), "no data rows");

    let ncols = rows.iter().map(Vec::len).max().unwrap_or(0);
    let mut best: Option<(usize, Vec<f64>, f64)> = None;
    for c in 0..ncols {
        let vals: Vec<f64> = rows
            .iter()
            .filter_map(|r| r.get(c).copied().flatten())
            .filter(|v| v.is_finite())
            .collect();
        if vals.len() < rows.len() / 2 {
            continue;
        }
        let (lo, hi) = vals
            .iter()
            .fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(*v), b.max(*v)));
        let spread = hi - lo;
        // Ignore monotonic index columns, which also have a large spread.
        let looks_like_index = vals.windows(2).all(|w| (w[1] - w[0] - 1.0).abs() < 1e-9);
        if looks_like_index {
            continue;
        }
        if best.as_ref().map(|(_, _, s)| spread > *s).unwrap_or(true) {
            best = Some((c, vals, spread));
        }
    }

    let (column, mut vals, _) = best.context("no usable numeric column")?;
    vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = vals.len();
    let median = vals[n / 2];
    let outliers = vals
        .iter()
        .filter(|v| v.abs() > 3.0 * median.abs().max(1e-9))
        .count();

    Ok(ResidualStats {
        rows: n,
        column,
        mean: vals.iter().sum::<f64>() / n as f64,
        median,
        p95: vals[((n as f64 * 0.95) as usize).min(n - 1)],
        max: vals[n - 1],
        outliers,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn tmpdir(tag: &str) -> std::path::PathBuf {
        let mut p = std::env::temp_dir();
        p.push(format!("terrapin_report_{tag}_{}", std::process::id()));
        fs::create_dir_all(p.join("report")).unwrap();
        p
    }

    #[test]
    fn flattens_nested_json_and_picks_highlights() {
        let d = tmpdir("json");
        fs::write(
            d.join("report/sfm_report.json"),
            r#"{"summary":{"reprojection_error_rms":0.62,"num_calibrated":1677},
                "unrelated":{"colour":"blue"},
                "per_photo":[1,2,3,4,5,6,7,8,9,10]}"#,
        )
        .unwrap();
        let r = load(&d).unwrap();
        assert_eq!(
            r.fields
                .get("summary.reprojection_error_rms")
                .map(String::as_str),
            Some("0.62")
        );
        assert!(r.highlights.contains_key("summary.reprojection_error_rms"));
        assert!(r.highlights.contains_key("summary.num_calibrated"));
        assert!(!r.highlights.contains_key("unrelated.colour"));
        // Long arrays collapse to a length.
        assert_eq!(
            r.fields.get("per_photo.length").map(String::as_str),
            Some("10")
        );
        fs::remove_dir_all(d).ok();
    }

    #[test]
    fn survives_a_missing_report_directory() {
        let d = tmpdir("missing");
        fs::remove_dir_all(d.join("report")).unwrap();
        let r = load(&d).unwrap();
        assert!(r.fields.is_empty());
        assert!(r.pos_residuals.is_none());
        fs::remove_dir_all(d).ok();
    }

    #[test]
    fn residuals_skip_the_index_column() {
        let d = tmpdir("resid");
        let mut csv = String::from("id,name,residual\n");
        for i in 0..100 {
            csv.push_str(&format!("{i},IMG_{i},{}\n", 0.01 * (i % 7) as f64));
        }
        fs::write(d.join("report/POS_residual_of_camera.csv"), csv).unwrap();
        let r = load(&d).unwrap();
        let s = r.pos_residuals.expect("residuals should parse");
        assert_eq!(s.rows, 100);
        assert_eq!(s.column, 2, "must pick the residual, not the id column");
        assert!(s.max <= 0.06 + 1e-9);
        fs::remove_dir_all(d).ok();
    }

    #[test]
    fn malformed_json_is_an_error_not_a_panic() {
        let d = tmpdir("badjson");
        fs::write(d.join("report/sfm_report.json"), "{not json").unwrap();
        assert!(load(&d).is_err());
        fs::remove_dir_all(d).ok();
    }
}
