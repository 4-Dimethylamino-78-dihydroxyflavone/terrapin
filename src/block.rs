//! The BlocksExchange (Bentley ContextCapture) block model, and a streaming
//! parser for it.
//!
//! DJI Terra writes its aerotriangulation result in this format. The parts
//! that matter downstream are the per-photogroup intrinsics and the per-photo
//! pose. Terra normally emits the pose as a row-major 3x3 matrix, but the
//! format also permits omega/phi/kappa and yaw/pitch/roll, so all three are
//! handled.
//!
//! Two conventions in this format bite people:
//!
//!   * `<Rotation>` is **world-to-camera**, and `<Center>` is the camera
//!     position in world coordinates. Projection is `x_cam = R * (X - C)`.
//!   * The image coordinate origin is the **centre of the upper-left pixel**,
//!     whereas COLMAP, OpenCV and nerfstudio all put it at that pixel's
//!     **corner**. Half a pixel, applied globally, in [`Photogroup::cx`].

use std::io::BufRead;
use std::path::Path;

use anyhow::{bail, Context, Result};
use quick_xml::events::Event;
use quick_xml::Reader;

use crate::math::{Mat3, Vec3};

// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Distortion {
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub p1: f64,
    pub p2: f64,
}

impl Distortion {
    pub fn max_abs(&self) -> f64 {
        [self.k1, self.k2, self.k3, self.p1, self.p2]
            .iter()
            .fold(0.0f64, |a, v| a.max(v.abs()))
    }

    /// True when the block references already-undistorted imagery.
    pub fn is_negligible(&self) -> bool {
        self.max_abs() < 1e-9
    }

    pub fn needs_k3(&self) -> bool {
        self.k3.abs() >= 1e-9
    }
}

/// How the camera's image axes are oriented relative to the optical frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CameraOrientation {
    /// X right, Y down, Z forward. Matches COLMAP and OpenCV.
    #[default]
    XRightYDown,
    /// X right, Y up, Z backward. Matches OpenGL and Blender.
    XRightYUp,
    /// Anything else the file declares that we do not know how to reorient.
    Other(u8),
}

impl CameraOrientation {
    fn parse(s: &str) -> Self {
        match s.trim() {
            "XRightYDown" | "" => Self::XRightYDown,
            "XRightYUp" => Self::XRightYUp,
            other => Self::Other(other.len() as u8),
        }
    }

    /// Post-multiplied correction taking the declared frame into COLMAP's.
    pub fn to_colmap(self) -> Mat3 {
        match self {
            Self::XRightYDown => Mat3::IDENTITY,
            Self::XRightYUp => Mat3::diag(1.0, -1.0, -1.0),
            Self::Other(_) => Mat3::IDENTITY,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::XRightYDown => "XRightYDown",
            Self::XRightYUp => "XRightYUp",
            Self::Other(_) => "unrecognised",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Photogroup {
    pub index: usize,
    pub name: String,
    pub width: u32,
    pub height: u32,
    /// Focal length in pixels along x.
    pub fx: f64,
    /// Focal length in pixels along y (fx divided by the declared aspect).
    pub fy: f64,
    /// Principal point x, already shifted into corner-origin convention.
    pub cx: f64,
    /// Principal point y, already shifted into corner-origin convention.
    pub cy: f64,
    pub distortion: Distortion,
    pub orientation: CameraOrientation,
}

impl Photogroup {
    /// How far the principal point sits from the image centre, as a fraction
    /// of image size. Above about 5% usually indicates a degenerate solve.
    pub fn principal_offset_fraction(&self) -> (f64, f64) {
        let dx = if self.width > 0 {
            (self.cx - self.width as f64 / 2.0).abs() / self.width as f64
        } else {
            0.0
        };
        let dy = if self.height > 0 {
            (self.cy - self.height as f64 / 2.0).abs() / self.height as f64
        } else {
            0.0
        };
        (dx, dy)
    }
}

#[derive(Clone, Debug)]
pub struct Photo {
    pub id: u32,
    pub group: usize,
    pub path: String,
    /// World-to-camera rotation.
    pub r: Mat3,
    /// Camera centre in world coordinates.
    pub c: Vec3,
}

impl Photo {
    /// Filename with any directory component stripped, for trainers that
    /// expect a flat images directory.
    pub fn basename(&self) -> &str {
        let s = self.path.trim_end_matches(['/', '\\']);
        match s.rfind(['/', '\\']) {
            Some(i) => &s[i + 1..],
            None => s,
        }
    }

    /// COLMAP-style translation, `t = -R * C`.
    pub fn translation(&self) -> Vec3 {
        -(self.r * self.c)
    }
}

#[derive(Clone, Debug, Default)]
pub struct Srs {
    pub id: String,
    pub name: String,
    pub definition: String,
}

#[derive(Clone, Debug, Default)]
pub struct Block {
    pub name: String,
    pub srs: Vec<Srs>,
    pub groups: Vec<Photogroup>,
    pub photos: Vec<Photo>,
}

impl Block {
    /// Axis-aligned extent of the camera centres, in source units.
    pub fn extent(&self) -> Vec3 {
        let (min, max) = self.bounds();
        max - min
    }

    pub fn bounds(&self) -> (Vec3, Vec3) {
        let mut min = Vec3::splat(f64::INFINITY);
        let mut max = Vec3::splat(f64::NEG_INFINITY);
        for p in &self.photos {
            min = min.min(p.c);
            max = max.max(p.c);
        }
        (min, max)
    }

    pub fn centroid(&self) -> Vec3 {
        if self.photos.is_empty() {
            return Vec3::ZERO;
        }
        let sum = self.photos.iter().fold(Vec3::ZERO, |acc, p| acc + p.c);
        sum / self.photos.len() as f64
    }

    pub fn srs_blob(&self) -> String {
        self.srs
            .iter()
            .map(|s| format!("{} {}", s.name, s.definition))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Reorient every photo into COLMAP's camera convention, if the file
    /// declares something else.
    pub fn normalize_orientation(&mut self) {
        for photo in &mut self.photos {
            let fix = self.groups[photo.group].orientation.to_colmap();
            photo.r = fix * photo.r;
        }
        for g in &mut self.groups {
            g.orientation = CameraOrientation::XRightYDown;
        }
    }
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

#[derive(Default)]
struct GroupBuilder {
    name: String,
    width: Option<u32>,
    height: Option<u32>,
    focal_px: Option<f64>,
    focal_mm: Option<f64>,
    sensor_mm: Option<f64>,
    aspect: Option<f64>,
    ppx: Option<f64>,
    ppy: Option<f64>,
    dist: Distortion,
    orientation: CameraOrientation,
}

impl GroupBuilder {
    fn new() -> Self {
        Self {
            orientation: CameraOrientation::XRightYDown,
            ..Default::default()
        }
    }

    fn finish(self, index: usize) -> Result<Photogroup> {
        let width = self
            .width
            .context("photogroup is missing ImageDimensions/Width")?;
        let height = self
            .height
            .context("photogroup is missing ImageDimensions/Height")?;

        // Prefer an explicit pixel focal length; otherwise derive it from the
        // millimetre focal length and the sensor's largest dimension, which is
        // what SensorSize means in this format.
        let fx = match (self.focal_px, self.focal_mm, self.sensor_mm) {
            (Some(f), _, _) => f,
            (None, Some(f_mm), Some(sensor)) if sensor > 0.0 => {
                f_mm * width.max(height) as f64 / sensor
            }
            _ => bail!(
                "photogroup {index} has neither FocalLengthPixels nor \
                 FocalLength+SensorSize; cannot recover intrinsics"
            ),
        };
        if !(fx.is_finite() && fx > 0.0) {
            bail!("photogroup {index} has a non-positive focal length ({fx})");
        }

        let aspect = self.aspect.filter(|a| *a > 0.0).unwrap_or(1.0);

        // Half-pixel shift from pixel-centre to pixel-corner origin.
        let cx = self.ppx.unwrap_or(width as f64 / 2.0 - 0.5) + 0.5;
        let cy = self.ppy.unwrap_or(height as f64 / 2.0 - 0.5) + 0.5;

        Ok(Photogroup {
            index,
            name: if self.name.is_empty() {
                format!("group{index}")
            } else {
                self.name
            },
            width,
            height,
            fx,
            fy: fx / aspect,
            cx,
            cy,
            distortion: self.dist,
            orientation: self.orientation,
        })
    }
}

#[derive(Default)]
struct PhotoBuilder {
    id: Option<u32>,
    path: String,
    matrix: [[Option<f64>; 3]; 3],
    omega: Option<f64>,
    phi: Option<f64>,
    kappa: Option<f64>,
    yaw: Option<f64>,
    pitch: Option<f64>,
    roll: Option<f64>,
    cx: Option<f64>,
    cy: Option<f64>,
    cz: Option<f64>,
}

impl PhotoBuilder {
    fn rotation(&self) -> Option<Mat3> {
        if self.matrix.iter().all(|r| r.iter().all(Option::is_some)) {
            let m = self.matrix.map(|row| row.map(|v| v.unwrap()));
            return Some(Mat3::from_rows(m).orthonormalize());
        }
        if let (Some(o), Some(p), Some(k)) = (self.omega, self.phi, self.kappa) {
            return Some(Mat3::from_opk(o, p, k).orthonormalize());
        }
        if let (Some(y), Some(p), Some(r)) = (self.yaw, self.pitch, self.roll) {
            // Yaw/pitch/roll shares the same intrinsic sequence in this format.
            return Some(Mat3::from_opk(r, p, y).orthonormalize());
        }
        None
    }

    fn finish(self, group: usize, fallback_id: u32) -> Option<Photo> {
        let r = self.rotation()?;
        let (x, y, z) = (self.cx?, self.cy?, self.cz?);
        let c = Vec3::new(x, y, z);
        if !c.is_finite() {
            return None;
        }
        Some(Photo {
            id: self.id.unwrap_or(fallback_id),
            group,
            path: self.path,
            r,
            c,
        })
    }
}

/// Statistics about what the parser had to discard.
#[derive(Clone, Copy, Debug, Default)]
pub struct ParseStats {
    pub photos_seen: usize,
    pub photos_unposed: usize,
}

pub fn parse_file(path: &Path) -> Result<(Block, ParseStats)> {
    let mut reader =
        Reader::from_file(path).with_context(|| format!("opening {}", path.display()))?;
    reader.trim_text(true);
    parse(&mut reader).with_context(|| format!("parsing {}", path.display()))
}

pub fn parse<R: BufRead>(reader: &mut Reader<R>) -> Result<(Block, ParseStats)> {
    let mut block = Block::default();
    let mut stats = ParseStats::default();

    let mut stack: Vec<String> = Vec::with_capacity(16);
    let mut buf = Vec::new();

    let mut group: Option<GroupBuilder> = None;
    let mut photo: Option<PhotoBuilder> = None;
    let mut srs: Option<Srs> = None;
    let mut group_index = 0usize;

    loop {
        match reader.read_event_into(&mut buf) {
            Err(e) => bail!("malformed XML at byte {}: {e}", reader.buffer_position()),
            Ok(Event::Eof) => break,

            Ok(Event::Start(e)) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                match name.as_str() {
                    "Photogroup" => group = Some(GroupBuilder::new()),
                    "Photo" => {
                        photo = Some(PhotoBuilder::default());
                        stats.photos_seen += 1;
                    }
                    "SRS" => srs = Some(Srs::default()),
                    _ => {}
                }
                stack.push(name);
            }

            Ok(Event::End(e)) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                match name.as_str() {
                    "Photogroup" => {
                        if let Some(g) = group.take() {
                            block.groups.push(g.finish(group_index)?);
                            group_index += 1;
                        }
                    }
                    "Photo" => {
                        if let Some(p) = photo.take() {
                            let next = block.photos.len() as u32;
                            match p.finish(group_index, next) {
                                Some(ph) => block.photos.push(ph),
                                None => stats.photos_unposed += 1,
                            }
                        }
                    }
                    "SRS" => {
                        if let Some(s) = srs.take() {
                            block.srs.push(s);
                        }
                    }
                    _ => {}
                }
                stack.pop();
            }

            Ok(Event::Text(t)) => {
                let value = t.unescape().unwrap_or_default().trim().to_string();
                if value.is_empty() {
                    buf.clear();
                    continue;
                }
                let leaf = stack.last().map(String::as_str).unwrap_or("");
                let parent = stack
                    .get(stack.len().wrapping_sub(2))
                    .map(String::as_str)
                    .unwrap_or("");

                if let Some(p) = photo.as_mut() {
                    absorb_photo(p, parent, leaf, &value);
                } else if let Some(g) = group.as_mut() {
                    absorb_group(g, parent, leaf, &value);
                } else if let Some(s) = srs.as_mut() {
                    match leaf {
                        "Id" => s.id = value,
                        "Name" => s.name = value,
                        "Definition" => s.definition = value,
                        _ => {}
                    }
                } else if leaf == "Name" && parent == "Block" {
                    block.name = value;
                }
            }

            Ok(_) => {}
        }
        buf.clear();
    }

    if block.groups.is_empty() {
        bail!("no <Photogroup> found; is this really a BlocksExchange file?");
    }
    if block.photos.is_empty() {
        bail!(
            "no photos with a complete pose found ({} seen, all incomplete)",
            stats.photos_seen
        );
    }

    // Photos are emitted inside their photogroup, so the running index is one
    // past the group they belong to once the group closes. Clamp defensively.
    let ngroups = block.groups.len();
    for p in &mut block.photos {
        if p.group >= ngroups {
            p.group = ngroups - 1;
        }
    }

    Ok((block, stats))
}

fn absorb_group(g: &mut GroupBuilder, parent: &str, leaf: &str, value: &str) {
    let num = || value.parse::<f64>().ok();
    match (parent, leaf) {
        (_, "Name") if parent == "Photogroup" => g.name = value.to_string(),
        ("ImageDimensions", "Width") => g.width = value.parse().ok(),
        ("ImageDimensions", "Height") => g.height = value.parse().ok(),
        (_, "FocalLengthPixels") => g.focal_px = num(),
        (_, "FocalLength") => g.focal_mm = num(),
        (_, "SensorSize") => g.sensor_mm = num(),
        (_, "AspectRatio") => g.aspect = num(),
        (_, "CameraOrientation") => g.orientation = CameraOrientation::parse(value),
        ("PrincipalPoint", "x") => g.ppx = num(),
        ("PrincipalPoint", "y") => g.ppy = num(),
        ("Distortion", "K1") => g.dist.k1 = num().unwrap_or(0.0),
        ("Distortion", "K2") => g.dist.k2 = num().unwrap_or(0.0),
        ("Distortion", "K3") => g.dist.k3 = num().unwrap_or(0.0),
        ("Distortion", "P1") => g.dist.p1 = num().unwrap_or(0.0),
        ("Distortion", "P2") => g.dist.p2 = num().unwrap_or(0.0),
        _ => {}
    }
}

fn absorb_photo(p: &mut PhotoBuilder, parent: &str, leaf: &str, value: &str) {
    let num = || value.parse::<f64>().ok();
    match (parent, leaf) {
        ("Photo", "Id") => p.id = value.parse().ok(),
        ("Photo", "ImagePath") => p.path = value.to_string(),
        ("Center", "x") => p.cx = num(),
        ("Center", "y") => p.cy = num(),
        ("Center", "z") => p.cz = num(),
        ("Rotation", "Omega") => p.omega = num(),
        ("Rotation", "Phi") => p.phi = num(),
        ("Rotation", "Kappa") => p.kappa = num(),
        ("Rotation", "Yaw") => p.yaw = num(),
        ("Rotation", "Pitch") => p.pitch = num(),
        ("Rotation", "Roll") => p.roll = num(),
        ("Rotation", m) if m.len() == 4 && m.starts_with("M_") => {
            let bytes = m.as_bytes();
            let i = (bytes[2] - b'0') as usize;
            let j = (bytes[3] - b'0') as usize;
            if i < 3 && j < 3 {
                p.matrix[i][j] = num();
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MINIMAL: &str = r#"<?xml version="1.0"?>
<BlocksExchange version="3.2">
  <SpatialReferenceSystems><SRS>
    <Id>0</Id><Name>WGS 84</Name><Definition>EPSG:4978</Definition>
  </SRS></SpatialReferenceSystems>
  <Block><Name>Quarry Test 1</Name>
    <Photogroups><Photogroup>
      <Name>DJI M3E</Name>
      <ImageDimensions><Width>5472</Width><Height>3648</Height></ImageDimensions>
      <FocalLengthPixels>4000</FocalLengthPixels>
      <CameraOrientation>XRightYDown</CameraOrientation>
      <PrincipalPoint><x>2733</x><y>1826</y></PrincipalPoint>
      <Distortion><K1>0</K1><K2>0</K2><K3>0</K3><P1>0</P1><P2>0</P2></Distortion>
      <Photo>
        <Id>0</Id><ImagePath>D:/photos/DJI_0001.JPG</ImagePath>
        <Pose>
          <Rotation>
            <M_00>1</M_00><M_01>0</M_01><M_02>0</M_02>
            <M_10>0</M_10><M_11>1</M_11><M_12>0</M_12>
            <M_20>0</M_20><M_21>0</M_21><M_22>1</M_22>
          </Rotation>
          <Center><x>10</x><y>20</y><z>30</z></Center>
        </Pose>
      </Photo>
      <Photo>
        <Id>1</Id><ImagePath>D:/photos/DJI_0002.JPG</ImagePath>
        <Pose><Center><x>1</x><y>2</y><z>3</z></Center></Pose>
      </Photo>
    </Photogroup></Photogroups>
  </Block>
</BlocksExchange>"#;

    fn parse_str(s: &str) -> Result<(Block, ParseStats)> {
        let mut r = Reader::from_str(s);
        r.trim_text(true);
        parse(&mut r)
    }

    #[test]
    fn parses_intrinsics_and_applies_half_pixel_shift() {
        let (b, _) = parse_str(MINIMAL).unwrap();
        let g = &b.groups[0];
        assert_eq!(g.width, 5472);
        assert_eq!(g.height, 3648);
        assert_eq!(g.fx, 4000.0);
        assert_eq!(g.fy, 4000.0);
        assert_eq!(g.cx, 2733.5, "principal point must shift by +0.5");
        assert_eq!(g.cy, 1826.5, "principal point must shift by +0.5");
        assert!(g.distortion.is_negligible());
        assert_eq!(b.name, "Quarry Test 1");
        assert_eq!(b.srs[0].definition, "EPSG:4978");
    }

    #[test]
    fn skips_photos_without_a_pose_but_counts_them() {
        let (b, s) = parse_str(MINIMAL).unwrap();
        assert_eq!(b.photos.len(), 1, "the rotation-less photo must be dropped");
        assert_eq!(s.photos_seen, 2);
        assert_eq!(s.photos_unposed, 1);
    }

    #[test]
    fn basename_strips_windows_and_unix_separators() {
        let (b, _) = parse_str(MINIMAL).unwrap();
        assert_eq!(b.photos[0].basename(), "DJI_0001.JPG");
    }

    #[test]
    fn translation_matches_colmap_convention() {
        let (b, _) = parse_str(MINIMAL).unwrap();
        // Identity rotation, so t should be exactly -C.
        let t = b.photos[0].translation();
        assert_eq!(t.x, -10.0);
        assert_eq!(t.y, -20.0);
        assert_eq!(t.z, -30.0);
    }

    #[test]
    fn derives_pixel_focal_from_millimetres() {
        let s = MINIMAL.replace(
            "<FocalLengthPixels>4000</FocalLengthPixels>",
            "<FocalLength>12.29</FocalLength><SensorSize>17.3</SensorSize>",
        );
        let (b, _) = parse_str(&s).unwrap();
        let expect = 12.29 * 5472.0 / 17.3;
        assert!((b.groups[0].fx - expect).abs() < 1e-9);
    }

    #[test]
    fn rejects_a_file_with_no_photogroups() {
        let err = parse_str("<BlocksExchange version=\"3.2\"><Block/></BlocksExchange>")
            .unwrap_err()
            .to_string();
        assert!(err.contains("no <Photogroup>"), "unhelpful error: {err}");
    }

    #[test]
    fn xrightyup_reorients_into_colmap_frame() {
        let s = MINIMAL.replace("XRightYDown", "XRightYUp");
        let (mut b, _) = parse_str(&s).unwrap();
        assert_eq!(b.groups[0].orientation, CameraOrientation::XRightYUp);
        b.normalize_orientation();
        assert_eq!(b.groups[0].orientation, CameraOrientation::XRightYDown);
        // Identity input, so the result is the flip itself.
        let r = b.photos[0].r;
        assert_eq!(r.m[1][1], -1.0);
        assert_eq!(r.m[2][2], -1.0);
    }
}
