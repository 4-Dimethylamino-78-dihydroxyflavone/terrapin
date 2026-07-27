//! Output formats.
//!
//! Every exporter consumes the same already-localised [`Block`], so they can
//! be requested in any combination in one pass. The conventions differ more
//! than people expect:
//!
//! | format     | stores        | camera looks down |
//! |------------|---------------|-------------------|
//! | COLMAP     | world→camera  | +Z, Y down        |
//! | nerfstudio | camera→world  | −Z, Y up (OpenGL) |
//! | Bundler    | world→camera  | −Z, Y up          |
//!
//! Getting one of these backwards produces a model that loads without
//! complaint and trains into a cloud of noise, so each conversion is spelled
//! out rather than folded into a constant.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use anyhow::{Context, Result};

use crate::block::{Block, Photogroup};
use crate::cloud::Cloud;
use crate::frame::LocalFrame;
use crate::math::{Mat3, Vec3};

/// Flip between "+Z forward, Y down" and "−Z forward, Y up".
const FLIP_YZ: Mat3 = Mat3::diag(1.0, -1.0, -1.0);

#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum Format {
    /// COLMAP sparse model as text. Read by gsplat, nerfstudio, Brush,
    /// LichtFeld Studio, OpenSplat and Postshot.
    ColmapTxt,
    /// COLMAP sparse model as binary. Smaller and faster to load.
    ColmapBin,
    /// nerfstudio / instant-ngp transforms.json.
    Nerfstudio,
    /// Bundler v0.3 .out plus list.txt. Read by RealityScan and older tools.
    Bundler,
    /// Camera centres as a coloured PLY, for CloudCompare or MeshLab.
    Ply,
    /// Camera positions and flight path as GeoJSON, for QGIS or ArcGIS Pro.
    Geojson,
    /// Camera positions and flight path as KML, for Google Earth.
    Kml,
    /// Pose table as CSV: local coordinates, geodetic, and omega/phi/kappa.
    Csv,
}

impl Format {
    pub fn label(self) -> &'static str {
        match self {
            Self::ColmapTxt => "colmap-txt",
            Self::ColmapBin => "colmap-bin",
            Self::Nerfstudio => "nerfstudio",
            Self::Bundler => "bundler",
            Self::Ply => "ply",
            Self::Geojson => "geojson",
            Self::Kml => "kml",
            Self::Csv => "csv",
        }
    }

    /// Whether this format is meaningless without a georeferenced frame.
    pub fn needs_georeference(self) -> bool {
        matches!(self, Self::Geojson | Self::Kml)
    }
}

pub struct ExportOptions<'a> {
    pub out_dir: &'a Path,
    pub cloud: Option<&'a Cloud>,
    pub frame: &'a LocalFrame,
    /// Strip directories from ImagePath, for a flat images/ folder.
    pub basenames: bool,
    /// Emit PINHOLE even when the block carries distortion coefficients.
    pub force_pinhole: bool,
}

impl ExportOptions<'_> {
    fn image_name<'b>(&self, photo: &'b crate::block::Photo) -> &'b str {
        if self.basenames {
            photo.basename()
        } else {
            &photo.path
        }
    }
}

/// What a single exporter produced, for reporting back to the user.
pub struct Written {
    pub format: Format,
    pub files: Vec<String>,
}

pub fn run(format: Format, block: &Block, opts: &ExportOptions) -> Result<Written> {
    let files = match format {
        Format::ColmapTxt => colmap_txt(block, opts)?,
        Format::ColmapBin => colmap_bin(block, opts)?,
        Format::Nerfstudio => nerfstudio(block, opts)?,
        Format::Bundler => bundler(block, opts)?,
        Format::Ply => ply(block, opts)?,
        Format::Geojson => geojson(block, opts)?,
        Format::Kml => kml(block, opts)?,
        Format::Csv => csv(block, opts)?,
    };
    Ok(Written { format, files })
}

// ---------------------------------------------------------------------------
// Camera model selection, shared by the COLMAP writers
// ---------------------------------------------------------------------------

pub struct CameraModel {
    pub name: &'static str,
    pub id: i32,
    pub params: Vec<f64>,
}

fn camera_model(g: &Photogroup, force_pinhole: bool) -> CameraModel {
    let d = &g.distortion;
    if force_pinhole || d.is_negligible() {
        CameraModel {
            name: "PINHOLE",
            id: 1,
            params: vec![g.fx, g.fy, g.cx, g.cy],
        }
    } else if !d.needs_k3() {
        CameraModel {
            name: "OPENCV",
            id: 4,
            params: vec![g.fx, g.fy, g.cx, g.cy, d.k1, d.k2, d.p1, d.p2],
        }
    } else {
        CameraModel {
            name: "FULL_OPENCV",
            id: 6,
            params: vec![
                g.fx, g.fy, g.cx, g.cy, d.k1, d.k2, d.p1, d.p2, d.k3, 0.0, 0.0, 0.0,
            ],
        }
    }
}

fn sparse_dir(opts: &ExportOptions) -> Result<std::path::PathBuf> {
    let d = opts.out_dir.join("sparse").join("0");
    fs::create_dir_all(&d).with_context(|| format!("creating {}", d.display()))?;
    Ok(d)
}

// ---------------------------------------------------------------------------
// COLMAP, text
// ---------------------------------------------------------------------------

fn colmap_txt(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    let dir = sparse_dir(opts)?;

    let mut w = BufWriter::new(File::create(dir.join("cameras.txt"))?);
    writeln!(w, "# Camera list with one line of data per camera:")?;
    writeln!(w, "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]")?;
    writeln!(w, "# Number of cameras: {}", block.groups.len())?;
    for g in &block.groups {
        let m = camera_model(g, opts.force_pinhole);
        write!(w, "{} {} {} {}", g.index + 1, m.name, g.width, g.height)?;
        for p in &m.params {
            write!(w, " {p:.10}")?;
        }
        writeln!(w)?;
    }
    w.flush()?;

    let mut w = BufWriter::new(File::create(dir.join("images.txt"))?);
    writeln!(w, "# Image list with two lines of data per image:")?;
    writeln!(w, "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME")?;
    writeln!(w, "#   POINTS2D[] as (X, Y, POINT3D_ID)")?;
    writeln!(
        w,
        "# Number of images: {}, mean observations per image: 0",
        block.photos.len()
    )?;
    for (i, p) in block.photos.iter().enumerate() {
        let q = p.r.to_quat();
        let t = p.translation();
        writeln!(
            w,
            "{} {:.12} {:.12} {:.12} {:.12} {:.8} {:.8} {:.8} {} {}",
            i + 1,
            q[0],
            q[1],
            q[2],
            q[3],
            t.x,
            t.y,
            t.z,
            p.group + 1,
            opts.image_name(p)
        )?;
        // The second line is the 2D observation list. It is legitimately
        // empty here: the block carries no tie points.
        writeln!(w)?;
    }
    w.flush()?;

    let mut w = BufWriter::new(File::create(dir.join("points3D.txt"))?);
    let n = opts.cloud.map(Cloud::len).unwrap_or(0);
    writeln!(w, "# 3D point list with one line of data per point:")?;
    writeln!(
        w,
        "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)"
    )?;
    writeln!(w, "# Number of points: {n}, mean track length: 0")?;
    if let Some(c) = opts.cloud {
        for (i, (p, col)) in c.points.iter().zip(&c.colors).enumerate() {
            writeln!(
                w,
                "{} {:.6} {:.6} {:.6} {} {} {} 1.0",
                i + 1,
                p.x,
                p.y,
                p.z,
                col[0],
                col[1],
                col[2]
            )?;
        }
    }
    w.flush()?;

    Ok(vec![
        "sparse/0/cameras.txt".into(),
        "sparse/0/images.txt".into(),
        "sparse/0/points3D.txt".into(),
    ])
}

// ---------------------------------------------------------------------------
// COLMAP, binary
// ---------------------------------------------------------------------------

fn colmap_bin(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    let dir = sparse_dir(opts)?;

    let mut w = BufWriter::new(File::create(dir.join("cameras.bin"))?);
    w.write_all(&(block.groups.len() as u64).to_le_bytes())?;
    for g in &block.groups {
        let m = camera_model(g, opts.force_pinhole);
        w.write_all(&((g.index + 1) as u32).to_le_bytes())?;
        w.write_all(&m.id.to_le_bytes())?;
        w.write_all(&(g.width as u64).to_le_bytes())?;
        w.write_all(&(g.height as u64).to_le_bytes())?;
        for p in &m.params {
            w.write_all(&p.to_le_bytes())?;
        }
    }
    w.flush()?;

    let mut w = BufWriter::new(File::create(dir.join("images.bin"))?);
    w.write_all(&(block.photos.len() as u64).to_le_bytes())?;
    for (i, p) in block.photos.iter().enumerate() {
        let q = p.r.to_quat();
        let t = p.translation();
        w.write_all(&((i + 1) as u32).to_le_bytes())?;
        for v in q {
            w.write_all(&v.to_le_bytes())?;
        }
        for v in t.to_array() {
            w.write_all(&v.to_le_bytes())?;
        }
        w.write_all(&((p.group + 1) as u32).to_le_bytes())?;
        w.write_all(opts.image_name(p).as_bytes())?;
        w.write_all(&[0u8])?; // names are NUL-terminated
        w.write_all(&0u64.to_le_bytes())?; // no 2D observations
    }
    w.flush()?;

    let mut w = BufWriter::new(File::create(dir.join("points3D.bin"))?);
    let n = opts.cloud.map(Cloud::len).unwrap_or(0);
    w.write_all(&(n as u64).to_le_bytes())?;
    if let Some(c) = opts.cloud {
        for (i, (p, col)) in c.points.iter().zip(&c.colors).enumerate() {
            w.write_all(&((i + 1) as u64).to_le_bytes())?;
            for v in p.to_array() {
                w.write_all(&v.to_le_bytes())?;
            }
            w.write_all(col)?;
            w.write_all(&1.0f64.to_le_bytes())?; // reprojection error
            w.write_all(&0u64.to_le_bytes())?; // empty track
        }
    }
    w.flush()?;

    Ok(vec![
        "sparse/0/cameras.bin".into(),
        "sparse/0/images.bin".into(),
        "sparse/0/points3D.bin".into(),
    ])
}

// ---------------------------------------------------------------------------
// nerfstudio / instant-ngp
// ---------------------------------------------------------------------------

fn nerfstudio(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    fs::create_dir_all(opts.out_dir)?;
    let path = opts.out_dir.join("transforms.json");
    let mut w = BufWriter::new(File::create(&path)?);

    let single = block.groups.len() == 1;
    let g0 = &block.groups[0];
    let m0 = camera_model(g0, opts.force_pinhole);

    writeln!(w, "{{")?;
    writeln!(
        w,
        "  \"camera_model\": \"{}\",",
        if m0.name == "PINHOLE" { "PINHOLE" } else { "OPENCV" }
    )?;
    if single {
        writeln!(w, "  \"w\": {},", g0.width)?;
        writeln!(w, "  \"h\": {},", g0.height)?;
        writeln!(w, "  \"fl_x\": {:.10},", g0.fx)?;
        writeln!(w, "  \"fl_y\": {:.10},", g0.fy)?;
        writeln!(w, "  \"cx\": {:.10},", g0.cx)?;
        writeln!(w, "  \"cy\": {:.10},", g0.cy)?;
        let d = &g0.distortion;
        if !opts.force_pinhole && !d.is_negligible() {
            writeln!(w, "  \"k1\": {:.12},", d.k1)?;
            writeln!(w, "  \"k2\": {:.12},", d.k2)?;
            writeln!(w, "  \"p1\": {:.12},", d.p1)?;
            writeln!(w, "  \"p2\": {:.12},", d.p2)?;
        }
    }
    if opts.cloud.is_some() {
        writeln!(w, "  \"ply_file_path\": \"sparse_pc.ply\",")?;
    }
    writeln!(w, "  \"frames\": [")?;

    for (i, p) in block.photos.iter().enumerate() {
        let g = &block.groups[p.group];
        // COLMAP world→camera becomes camera→world, then OpenCV becomes
        // OpenGL by flipping the Y and Z basis vectors.
        let c2w = p.r.transpose() * FLIP_YZ;
        writeln!(w, "    {{")?;
        writeln!(
            w,
            "      \"file_path\": \"images/{}\",",
            opts.image_name(p)
        )?;
        if !single {
            writeln!(w, "      \"w\": {},", g.width)?;
            writeln!(w, "      \"h\": {},", g.height)?;
            writeln!(w, "      \"fl_x\": {:.10},", g.fx)?;
            writeln!(w, "      \"fl_y\": {:.10},", g.fy)?;
            writeln!(w, "      \"cx\": {:.10},", g.cx)?;
            writeln!(w, "      \"cy\": {:.10},", g.cy)?;
        }
        writeln!(w, "      \"transform_matrix\": [")?;
        for r in 0..3 {
            writeln!(
                w,
                "        [{:.12}, {:.12}, {:.12}, {:.8}],",
                c2w.m[r][0],
                c2w.m[r][1],
                c2w.m[r][2],
                p.c[r]
            )?;
        }
        writeln!(w, "        [0.0, 0.0, 0.0, 1.0]")?;
        writeln!(w, "      ]")?;
        write!(w, "    }}")?;
        writeln!(w, "{}", if i + 1 == block.photos.len() { "" } else { "," })?;
    }
    writeln!(w, "  ]")?;
    writeln!(w, "}}")?;
    w.flush()?;

    let mut files = vec!["transforms.json".to_string()];
    if let Some(c) = opts.cloud {
        write_ply(&opts.out_dir.join("sparse_pc.ply"), &c.points, &c.colors)?;
        files.push("sparse_pc.ply".into());
    }
    Ok(files)
}

// ---------------------------------------------------------------------------
// Bundler v0.3
// ---------------------------------------------------------------------------

fn bundler(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    fs::create_dir_all(opts.out_dir)?;
    let npts = opts.cloud.map(Cloud::len).unwrap_or(0);

    let mut w = BufWriter::new(File::create(opts.out_dir.join("bundle.out"))?);
    writeln!(w, "# Bundle file v0.3")?;
    writeln!(w, "{} {}", block.photos.len(), npts)?;
    for p in &block.photos {
        let g = &block.groups[p.group];
        let d = &g.distortion;
        // Bundler has one focal length and only radial k1/k2.
        writeln!(w, "{:.10} {:.12} {:.12}", g.fx, d.k1, d.k2)?;
        // Bundler cameras look down −Z, so flip Y and Z relative to COLMAP.
        let r = FLIP_YZ * p.r;
        let t = FLIP_YZ * p.translation();
        for row in 0..3 {
            writeln!(
                w,
                "{:.12} {:.12} {:.12}",
                r.m[row][0], r.m[row][1], r.m[row][2]
            )?;
        }
        writeln!(w, "{:.8} {:.8} {:.8}", t.x, t.y, t.z)?;
    }
    if let Some(c) = opts.cloud {
        for (p, col) in c.points.iter().zip(&c.colors) {
            writeln!(w, "{:.6} {:.6} {:.6}", p.x, p.y, p.z)?;
            writeln!(w, "{} {} {}", col[0], col[1], col[2])?;
            writeln!(w, "0")?; // no view list
        }
    }
    w.flush()?;

    let mut w = BufWriter::new(File::create(opts.out_dir.join("list.txt"))?);
    for p in &block.photos {
        writeln!(w, "{}", opts.image_name(p))?;
    }
    w.flush()?;

    Ok(vec!["bundle.out".into(), "list.txt".into()])
}

// ---------------------------------------------------------------------------
// PLY
// ---------------------------------------------------------------------------

fn ply(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    fs::create_dir_all(opts.out_dir)?;
    let mut files = Vec::new();

    let pts: Vec<Vec3> = block.photos.iter().map(|p| p.c).collect();
    let cols: Vec<[u8; 3]> = vec![[255, 64, 64]; pts.len()];
    write_ply(&opts.out_dir.join("cameras.ply"), &pts, &cols)?;
    files.push("cameras.ply".into());

    if let Some(c) = opts.cloud {
        write_ply(&opts.out_dir.join("sparse_pc.ply"), &c.points, &c.colors)?;
        files.push("sparse_pc.ply".into());
    }
    Ok(files)
}

fn write_ply(path: &Path, pts: &[Vec3], cols: &[[u8; 3]]) -> Result<()> {
    let mut w = BufWriter::with_capacity(1 << 20, File::create(path)?);
    writeln!(w, "ply")?;
    writeln!(w, "format binary_little_endian 1.0")?;
    writeln!(w, "comment generated by terrapin")?;
    writeln!(w, "element vertex {}", pts.len())?;
    for a in ["x", "y", "z"] {
        writeln!(w, "property float {a}")?;
    }
    for a in ["red", "green", "blue"] {
        writeln!(w, "property uchar {a}")?;
    }
    writeln!(w, "end_header")?;
    for (p, c) in pts.iter().zip(cols) {
        // f32 is fine here: the frame is already local and metric.
        w.write_all(&(p.x as f32).to_le_bytes())?;
        w.write_all(&(p.y as f32).to_le_bytes())?;
        w.write_all(&(p.z as f32).to_le_bytes())?;
        w.write_all(c)?;
    }
    w.flush()?;
    Ok(())
}

// ---------------------------------------------------------------------------
// GeoJSON and KML
// ---------------------------------------------------------------------------

fn geodetic_track(block: &Block, frame: &LocalFrame) -> Vec<(String, f64, f64, f64)> {
    block
        .photos
        .iter()
        .filter_map(|p| {
            let (lat, lon, h) = frame.to_geodetic(p.c)?;
            Some((p.basename().to_string(), lat, lon, h))
        })
        .collect()
}

fn geojson(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    fs::create_dir_all(opts.out_dir)?;
    let track = geodetic_track(block, opts.frame);
    let mut w = BufWriter::new(File::create(opts.out_dir.join("cameras.geojson"))?);

    writeln!(w, "{{")?;
    writeln!(w, "  \"type\": \"FeatureCollection\",")?;
    // RFC 7946 mandates WGS84 lon/lat, so no CRS member is written.
    writeln!(w, "  \"features\": [")?;
    for (i, (name, lat, lon, h)) in track.iter().enumerate() {
        writeln!(w, "    {{")?;
        writeln!(w, "      \"type\": \"Feature\",")?;
        writeln!(
            w,
            "      \"properties\": {{\"name\": \"{}\", \"index\": {}, \"height_m\": {:.3}}},",
            escape_json(name),
            i,
            h
        )?;
        writeln!(
            w,
            "      \"geometry\": {{\"type\": \"Point\", \"coordinates\": [{lon:.9}, {lat:.9}, {h:.3}]}}"
        )?;
        write!(w, "    }}")?;
        writeln!(w, ",")?;
    }
    writeln!(w, "    {{")?;
    writeln!(w, "      \"type\": \"Feature\",")?;
    writeln!(w, "      \"properties\": {{\"name\": \"flight path\"}},")?;
    write!(
        w,
        "      \"geometry\": {{\"type\": \"LineString\", \"coordinates\": ["
    )?;
    for (i, (_, lat, lon, h)) in track.iter().enumerate() {
        if i > 0 {
            write!(w, ", ")?;
        }
        write!(w, "[{lon:.9}, {lat:.9}, {h:.3}]")?;
    }
    writeln!(w, "]}}")?;
    writeln!(w, "    }}")?;
    writeln!(w, "  ]")?;
    writeln!(w, "}}")?;
    w.flush()?;
    Ok(vec!["cameras.geojson".into()])
}

fn kml(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    fs::create_dir_all(opts.out_dir)?;
    let track = geodetic_track(block, opts.frame);
    let mut w = BufWriter::new(File::create(opts.out_dir.join("cameras.kml"))?);

    writeln!(w, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>")?;
    writeln!(w, "<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>")?;
    writeln!(
        w,
        "<name>{} camera stations</name>",
        escape_xml(if block.name.is_empty() { "terrapin" } else { &block.name })
    )?;
    writeln!(w, "<Folder><name>Cameras</name>")?;
    for (name, lat, lon, h) in &track {
        writeln!(w, "<Placemark><name>{}</name>", escape_xml(name))?;
        writeln!(
            w,
            "<Point><altitudeMode>absolute</altitudeMode>\
             <coordinates>{lon:.9},{lat:.9},{h:.3}</coordinates></Point></Placemark>"
        )?;
    }
    writeln!(w, "</Folder>")?;
    writeln!(w, "<Placemark><name>Flight path</name><LineString>")?;
    writeln!(w, "<altitudeMode>absolute</altitudeMode><coordinates>")?;
    for (_, lat, lon, h) in &track {
        writeln!(w, "{lon:.9},{lat:.9},{h:.3}")?;
    }
    writeln!(w, "</coordinates></LineString></Placemark>")?;
    writeln!(w, "</Document></kml>")?;
    w.flush()?;
    Ok(vec!["cameras.kml".into()])
}

// ---------------------------------------------------------------------------
// CSV pose table
// ---------------------------------------------------------------------------

fn csv(block: &Block, opts: &ExportOptions) -> Result<Vec<String>> {
    fs::create_dir_all(opts.out_dir)?;
    let mut w = BufWriter::new(File::create(opts.out_dir.join("cameras.csv"))?);
    writeln!(
        w,
        "name,group,local_x,local_y,local_z,latitude,longitude,height_m,omega_deg,phi_deg,kappa_deg"
    )?;
    for p in &block.photos {
        let (o, ph, k) = p.r.to_opk_degrees();
        let geo = opts.frame.to_geodetic(p.c);
        let (lat, lon, h) = match geo {
            Some(v) => (format!("{:.9}", v.0), format!("{:.9}", v.1), format!("{:.3}", v.2)),
            None => (String::new(), String::new(), String::new()),
        };
        writeln!(
            w,
            "{},{},{:.4},{:.4},{:.4},{},{},{},{:.6},{:.6},{:.6}",
            csv_escape(p.basename()),
            p.group,
            p.c.x,
            p.c.y,
            p.c.z,
            lat,
            lon,
            h,
            o,
            ph,
            k
        )?;
    }
    w.flush()?;
    Ok(vec!["cameras.csv".into()])
}

// ---------------------------------------------------------------------------

fn escape_json(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn escape_xml(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

fn csv_escape(s: &str) -> String {
    if s.contains([',', '"', '\n']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::{CameraOrientation, Distortion, Photo};

    fn tmpdir(tag: &str) -> std::path::PathBuf {
        let mut p = std::env::temp_dir();
        p.push(format!("terrapin_export_{tag}_{}", std::process::id()));
        fs::create_dir_all(&p).unwrap();
        p
    }

    fn demo_block() -> Block {
        let g = Photogroup {
            index: 0,
            name: "cam".into(),
            width: 4000,
            height: 3000,
            fx: 3200.0,
            fy: 3200.0,
            cx: 2000.5,
            cy: 1500.5,
            distortion: Distortion::default(),
            orientation: CameraOrientation::XRightYDown,
        };
        let mut b = Block {
            name: "demo".into(),
            groups: vec![g],
            ..Default::default()
        };
        for i in 0..3 {
            b.photos.push(Photo {
                id: i,
                group: 0,
                path: format!("D:\\photos\\IMG_{i}.JPG"),
                r: Mat3::from_opk(3.0 * i as f64, -2.0, 90.0).orthonormalize(),
                c: Vec3::new(i as f64 * 10.0, 5.0, 80.0),
            });
        }
        b
    }

    fn demo_frame() -> LocalFrame {
        let mut b = demo_block();
        // Force a georeferenced frame so geojson/kml have something to do.
        b.srs.push(crate::block::Srs {
            id: "0".into(),
            name: "WGS 84".into(),
            definition: "EPSG:4978".into(),
        });
        for p in &mut b.photos {
            p.c = crate::geodesy::geodetic_to_ecef(-33.61, 150.75, 200.0) + p.c;
        }
        LocalFrame::build_and_apply(&mut b)
    }

    fn opts<'a>(dir: &'a Path, frame: &'a LocalFrame) -> ExportOptions<'a> {
        ExportOptions {
            out_dir: dir,
            cloud: None,
            frame,
            basenames: true,
            force_pinhole: false,
        }
    }

    #[test]
    fn colmap_text_is_well_formed() {
        let dir = tmpdir("colmap");
        let f = demo_frame();
        let b = demo_block();
        run(Format::ColmapTxt, &b, &opts(&dir, &f)).unwrap();

        let cams = fs::read_to_string(dir.join("sparse/0/cameras.txt")).unwrap();
        let row = cams.lines().find(|l| !l.starts_with('#')).unwrap();
        let f: Vec<&str> = row.split_whitespace().collect();
        assert_eq!(f[1], "PINHOLE");
        assert_eq!(f[2], "4000");
        assert_eq!(f.len(), 8, "PINHOLE takes exactly four parameters");

        let imgs = fs::read_to_string(dir.join("sparse/0/images.txt")).unwrap();
        let data: Vec<&str> = imgs
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            .collect();
        assert_eq!(data.len(), 3);
        for l in &data {
            let f: Vec<&str> = l.split_whitespace().collect();
            assert_eq!(f.len(), 10, "images.txt row has the wrong field count");
            assert!(f[9].starts_with("IMG_"), "basename was not applied");
        }
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn colmap_binary_matches_text_poses() {
        let dir = tmpdir("colmapbin");
        let f = demo_frame();
        let b = demo_block();
        run(Format::ColmapTxt, &b, &opts(&dir, &f)).unwrap();
        run(Format::ColmapBin, &b, &opts(&dir, &f)).unwrap();

        let bin = fs::read(dir.join("sparse/0/images.bin")).unwrap();
        let n = u64::from_le_bytes(bin[0..8].try_into().unwrap());
        assert_eq!(n, 3);
        // First record: u32 id, then 4x f64 quaternion.
        let qw = f64::from_le_bytes(bin[12..20].try_into().unwrap());
        let txt = fs::read_to_string(dir.join("sparse/0/images.txt")).unwrap();
        let first = txt
            .lines()
            .find(|l| !l.starts_with('#') && !l.trim().is_empty())
            .unwrap();
        let qw_txt: f64 = first.split_whitespace().nth(1).unwrap().parse().unwrap();
        assert!((qw - qw_txt).abs() < 1e-10, "{qw} vs {qw_txt}");
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn nerfstudio_uses_opengl_convention() {
        let dir = tmpdir("nerf");
        let f = demo_frame();
        let b = demo_block();
        run(Format::Nerfstudio, &b, &opts(&dir, &f)).unwrap();
        let s = fs::read_to_string(dir.join("transforms.json")).unwrap();
        assert!(s.contains("\"camera_model\": \"PINHOLE\""));
        assert!(s.contains("\"fl_x\": 3200"));
        assert!(s.contains("\"file_path\": \"images/IMG_0.JPG\""));

        // The camera-to-world matrix must be the transpose with Y,Z flipped.
        let p = &b.photos[0];
        let want = p.r.transpose() * FLIP_YZ;
        let first_row = format!("[{:.12}, {:.12}, {:.12}", want.m[0][0], want.m[0][1], want.m[0][2]);
        assert!(s.contains(&first_row), "expected row {first_row} in\n{s}");
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn nerfstudio_json_parses() {
        let dir = tmpdir("nerfjson");
        let f = demo_frame();
        run(Format::Nerfstudio, &demo_block(), &opts(&dir, &f)).unwrap();
        let s = fs::read_to_string(dir.join("transforms.json")).unwrap();
        let v: serde_json::Value = serde_json::from_str(&s).expect("transforms.json is invalid");
        assert_eq!(v["frames"].as_array().unwrap().len(), 3);
        assert_eq!(v["frames"][0]["transform_matrix"].as_array().unwrap().len(), 4);
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn geojson_parses_and_is_lon_lat_ordered() {
        let dir = tmpdir("geo");
        let f = demo_frame();
        run(Format::Geojson, &demo_block(), &opts(&dir, &f)).unwrap();
        let s = fs::read_to_string(dir.join("cameras.geojson")).unwrap();
        let v: serde_json::Value = serde_json::from_str(&s).expect("geojson is invalid");
        let coords = &v["features"][0]["geometry"]["coordinates"];
        let lon = coords[0].as_f64().unwrap();
        let lat = coords[1].as_f64().unwrap();
        assert!((lon - 150.75).abs() < 0.01, "longitude must come first");
        assert!((lat + 33.61).abs() < 0.01, "latitude must come second");
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn bundler_flips_into_its_own_convention() {
        let dir = tmpdir("bundler");
        let f = demo_frame();
        let b = demo_block();
        run(Format::Bundler, &b, &opts(&dir, &f)).unwrap();
        let s = fs::read_to_string(dir.join("bundle.out")).unwrap();
        let lines: Vec<&str> = s.lines().collect();
        assert_eq!(lines[0], "# Bundle file v0.3");
        assert_eq!(lines[1], "3 0");
        // Row 1 of the first camera's matrix is row 1 of R, negated.
        let want = FLIP_YZ * b.photos[0].r;
        let got: Vec<f64> = lines[3]
            .split_whitespace()
            .map(|v| v.parse().unwrap())
            .collect();
        for j in 0..3 {
            assert!((got[j] - want.m[0][j]).abs() < 1e-9);
        }
        let list = fs::read_to_string(dir.join("list.txt")).unwrap();
        assert_eq!(list.lines().count(), 3);
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn csv_has_a_row_per_photo_and_valid_geodetics() {
        let dir = tmpdir("csv");
        let f = demo_frame();
        run(Format::Csv, &demo_block(), &opts(&dir, &f)).unwrap();
        let s = fs::read_to_string(dir.join("cameras.csv")).unwrap();
        let lines: Vec<&str> = s.lines().collect();
        assert_eq!(lines.len(), 4, "header plus three rows");
        assert!(lines[0].starts_with("name,group,"));
        let cols: Vec<&str> = lines[1].split(',').collect();
        assert_eq!(cols.len(), 11);
        let lat: f64 = cols[5].parse().unwrap();
        assert!((lat + 33.61).abs() < 0.01);
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn ply_camera_dump_is_readable_back() {
        let dir = tmpdir("ply");
        let f = demo_frame();
        let b = demo_block();
        run(Format::Ply, &b, &opts(&dir, &f)).unwrap();
        let c = crate::cloud::load_file(&dir.join("cameras.ply"), None).unwrap();
        assert_eq!(c.len(), 3);
        assert_eq!(c.colors[0], [255, 64, 64]);
        fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn distortion_selects_the_right_camera_model() {
        let mut g = demo_block().groups[0].clone();
        assert_eq!(camera_model(&g, false).name, "PINHOLE");

        g.distortion.k1 = -0.012;
        g.distortion.p1 = 1e-5;
        let m = camera_model(&g, false);
        assert_eq!(m.name, "OPENCV");
        assert_eq!(m.params.len(), 8);

        g.distortion.k3 = -4e-4;
        let m = camera_model(&g, false);
        assert_eq!(m.name, "FULL_OPENCV");
        assert_eq!(m.params.len(), 12);
        assert_eq!(m.params[8], -4e-4, "k3 sits after p2 in FULL_OPENCV");

        assert_eq!(camera_model(&g, true).name, "PINHOLE", "--force-pinhole ignored");
    }
}
