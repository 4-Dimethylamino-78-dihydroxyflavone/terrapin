//! Moving a block into a frame that radiance-field trainers can actually use.
//!
//! Terra writes the block in whatever the project CRS was: geocentric ECEF,
//! geographic degrees, or a projected metric system. None of those are
//! directly usable. ECEF coordinates run to 6.4e6 metres, so a scene only a
//! few hundred metres across occupies the last few significant digits of an
//! f32 — trainers that narrow to f32 will visibly quantise. Geographic
//! degrees are not even metric, so a naive consumer computes nonsense
//! baselines.
//!
//! The fix is a local east-north-up frame centred on the block. Crucially the
//! camera *rotations* have to be corrected too, not just the positions:
//! rotating the world under a fixed camera changes the camera's orientation
//! within it.
//!
//! A projected block is the awkward case, and the temptation there has to be
//! named to be resisted. Terra solved such a block in grid metres, so moving
//! it into the working frame is a subtraction and nothing else. But the SRS
//! usually names a UTM zone, and once the zone is known every camera centre
//! *could* be pushed through UTM to geodetic to a properly levelled ENU
//! frame. Doing that would be wrong. UTM's scale factor is 0.9996 on the
//! central meridian and grows past unity towards the zone edges, so the grid
//! is not the metric space it resembles: converting out of it stretches the
//! block by up to about 0.04% — some 40 cm across a kilometre — and stretches
//! it *unevenly*, which no rigid transform can undo. The result still looks
//! like a photogrammetric block and still trains; it is simply a worse solve
//! than the one Terra handed over, with no error message and nothing in the
//! output to point at.
//!
//! So the zone is kept as an *anchor*, not as a transform. It is enough to
//! say where on the earth the block sits, to drag a foreign point cloud into
//! the block's coordinates, and to match a photo's GPS position to a pose —
//! and the camera centres stay exactly where subtraction put them.

use serde::Serialize;

use crate::block::Block;
use crate::geodesy::{
    ecef_to_geodetic, enu_rotation, geodetic_to_ecef, geodetic_to_utm, utm_to_geodetic, Utm,
};
use crate::math::{Mat3, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceFrame {
    /// Earth-centred earth-fixed, e.g. EPSG:4978.
    Ecef,
    /// Longitude/latitude in degrees plus ellipsoidal height, e.g. EPSG:4326.
    Geographic,
    /// Any projected metric CRS, e.g. a UTM or MGA zone.
    Projected,
}

impl SourceFrame {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ecef => "ECEF (geocentric)",
            Self::Geographic => "geographic degrees",
            Self::Projected => "projected metric",
        }
    }

    /// Work out what frame a block is in, from its declared SRS and, failing
    /// that, from the magnitude of the coordinates themselves. The numeric
    /// test matters because Terra does not always write a usable SRS string.
    pub fn detect(block: &Block) -> Self {
        let blob = block.srs_blob().to_lowercase();

        if blob.contains("4978") || blob.contains("geocentric") || blob.contains("earth centered") {
            return Self::Ecef;
        }

        let (min, max) = block.bounds();
        let looks_geographic = min.x >= -180.0
            && max.x <= 180.0
            && min.y >= -90.0
            && max.y <= 90.0
            && min.is_finite()
            && max.is_finite();

        if looks_geographic {
            return Self::Geographic;
        }

        // ECEF is the only common frame where all three axes reach megametres.
        let c = block.centroid();
        if c.x.abs() > 1.0e6 && c.y.abs() > 1.0e6 && c.z.abs() > 1.0e6 {
            return Self::Ecef;
        }

        if blob.contains("4326") {
            return Self::Geographic;
        }

        Self::Projected
    }
}

/// Which axis the caller wants pointing at the sky.
///
/// Photogrammetry, GIS and COLMAP all put up on +Z, and a georeferenced block
/// arrives that way for free: an east-north-up frame *is* Z-up, and so is a
/// UTM grid. Real-time viewers, glTF, three.js and most NeRF tooling put up on
/// +Y instead, so a Z-up cloud lands on its side in them.
///
/// Note what is not happening here. Tools without georeferencing have to
/// *infer* gravity — from a ground plane, or from the null space of the camera
/// axes — and those estimates fail on the captures that least resemble their
/// assumptions. Nothing is being estimated here. The block was placed by its
/// own coordinate reference system, so which way is up is known exactly, and
/// this is a relabelling of axes rather than a correction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum UpAxis {
    /// East-north-up, as the CRS defines it. What COLMAP and GIS expect.
    #[default]
    Z,
    /// +Y up, -Z north. What glTF, three.js and most viewers expect.
    Y,
}

impl UpAxis {
    pub fn label(self) -> &'static str {
        match self {
            Self::Z => "Z up (east, north, up)",
            Self::Y => "Y up (east, up, south)",
        }
    }

    /// The rotation taking the working frame into this convention.
    pub fn rotation(self) -> Mat3 {
        match self {
            Self::Z => Mat3::IDENTITY,
            // up -> +Y, north -> -Z, east stays +X. Determinant +1, so this is
            // a rotation and not a reflection: a mirrored scene would train
            // perfectly happily and be wrong in a way nothing downstream flags.
            Self::Y => Mat3::from_rows([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]),
        }
    }
}

/// Re-express a block in a different up-axis convention.
///
/// Camera *rotations* move too, not just centres. Turning the world under a
/// fixed camera changes that camera's orientation within it, and forgetting the
/// second half leaves poses that point somewhere plausible and wrong.
pub fn apply_up_axis(block: &mut Block, up: UpAxis) {
    let r = up.rotation();
    if r == Mat3::IDENTITY {
        return;
    }
    for photo in &mut block.photos {
        photo.c = r * photo.c;
        photo.r = photo.r * r.transpose();
    }
}

/// The transform from source coordinates into the working local frame.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct LocalFrame {
    pub source: SourceFrame,
    /// Origin expressed in ECEF, for the curved-earth cases.
    pub origin_ecef: Option<[f64; 3]>,
    /// Origin expressed in the source's own units, for the projected case.
    pub origin_source: Option<[f64; 3]>,
    pub lat0: f64,
    pub lon0: f64,
    pub h0: f64,
    /// Set when the block is projected and the zone was recognised. This is
    /// the block's geodetic anchor; it is deliberately *not* part of the
    /// transform applied to the cameras.
    pub utm: Option<Utm>,
    #[serde(skip)]
    rot: Mat3,
}

impl LocalFrame {
    /// Build the frame from the block's camera centres and apply it, rewriting
    /// every photo's centre and rotation in place.
    pub fn build_and_apply(block: &mut Block) -> Self {
        let source = SourceFrame::detect(block);
        let frame = match source {
            SourceFrame::Geographic => {
                // BlocksExchange orders geographic coordinates x=longitude,
                // y=latitude. Getting this backwards silently mirrors the
                // scene, so it is worth stating explicitly.
                let n = block.photos.len() as f64;
                let (mut lon, mut lat, mut h) = (0.0, 0.0, 0.0);
                for p in &block.photos {
                    lon += p.c.x;
                    lat += p.c.y;
                    h += p.c.z;
                }
                let (lat0, lon0, h0) = (lat / n, lon / n, h / n);
                let origin = geodetic_to_ecef(lat0, lon0, h0);
                Self {
                    source,
                    origin_ecef: Some(origin.to_array()),
                    origin_source: None,
                    lat0,
                    lon0,
                    h0,
                    utm: None,
                    rot: enu_rotation(lat0, lon0),
                }
            }
            SourceFrame::Ecef => {
                let origin = block.centroid();
                let (lat0, lon0, h0) = ecef_to_geodetic(origin);
                Self {
                    source,
                    origin_ecef: Some(origin.to_array()),
                    origin_source: None,
                    lat0,
                    lon0,
                    h0,
                    utm: None,
                    rot: enu_rotation(lat0, lon0),
                }
            }
            SourceFrame::Projected => {
                // Subtraction and identity rotation, exactly as before. The
                // zone is read alongside, never applied: see the module
                // header for what routing the centres through UTM would cost.
                let origin = block.centroid();
                Self {
                    source,
                    origin_ecef: None,
                    origin_source: Some(origin.to_array()),
                    lat0: f64::NAN,
                    lon0: f64::NAN,
                    h0: f64::NAN,
                    utm: Utm::parse(&block.srs_blob()),
                    rot: Mat3::IDENTITY,
                }
            }
        };

        for photo in &mut block.photos {
            photo.c = frame.point(photo.c);
            // x_cam = R_src * (X_src - C_src), and X_src - C_src equals
            // rot^T * (X_local - C_local), so the camera rotation must absorb
            // rot^T on the right.
            photo.r = photo.r * frame.rot.transpose();
        }

        frame
    }

    /// Map a point from source coordinates into the local frame.
    pub fn point(&self, p: Vec3) -> Vec3 {
        match self.source {
            SourceFrame::Geographic => {
                let ecef = geodetic_to_ecef(p.y, p.x, p.z);
                let o = Vec3::new(
                    self.origin_ecef.unwrap()[0],
                    self.origin_ecef.unwrap()[1],
                    self.origin_ecef.unwrap()[2],
                );
                self.rot * (ecef - o)
            }
            SourceFrame::Ecef => {
                let o = Vec3::new(
                    self.origin_ecef.unwrap()[0],
                    self.origin_ecef.unwrap()[1],
                    self.origin_ecef.unwrap()[2],
                );
                self.rot * (p - o)
            }
            SourceFrame::Projected => {
                let o = Vec3::new(
                    self.origin_source.unwrap()[0],
                    self.origin_source.unwrap()[1],
                    self.origin_source.unwrap()[2],
                );
                p - o
            }
        }
    }

    /// Map a local-frame point back to geodetic (lat, lon, height).
    ///
    /// Returns `None` for a projected block whose zone could not be
    /// recognised, because then nothing in the file says where on the earth
    /// those grid metres are.
    ///
    /// The height a projected block reports is passed straight through from
    /// the block's own vertical axis. Terra does not state whether that is
    /// ellipsoidal or orthometric, so this is the block's height, not a
    /// WGS84 ellipsoidal height — the horizontal position is authoritative,
    /// the vertical carries whatever datum the survey used.
    pub fn to_geodetic(&self, local: Vec3) -> Option<(f64, f64, f64)> {
        match self.source {
            SourceFrame::Projected => {
                let zone = self.utm?;
                let o = self.origin_source?;
                let (lat, lon) = utm_to_geodetic(local.x + o[0], local.y + o[1], zone);
                Some((lat, lon, local.z + o[2]))
            }
            SourceFrame::Geographic | SourceFrame::Ecef => {
                let o = self.origin_ecef?;
                let origin = Vec3::new(o[0], o[1], o[2]);
                let ecef = origin + self.rot.transpose() * local;
                Some(ecef_to_geodetic(ecef))
            }
        }
    }

    /// Map a geodetic position into the working local frame.
    /// None when the frame has no geodetic anchor (projected, unknown zone).
    ///
    /// This is the inverse of [`to_geodetic`](Self::to_geodetic) and the hinge
    /// on which two later stages turn: bringing a cloud that arrived in
    /// degrees into the block's coordinates, and matching an original photo's
    /// GPS position to the pose that belongs to it. Both compare distances in
    /// the local frame, so both need the same map, computed the same way.
    pub fn geodetic_to_local(&self, lat_deg: f64, lon_deg: f64, h: f64) -> Option<Vec3> {
        match self.source {
            SourceFrame::Projected => {
                // Into the grid the block already lives in, then the same
                // subtraction the cameras got. No rotation, no rescaling.
                let zone = self.utm?;
                let o = self.origin_source?;
                let (easting, northing) = geodetic_to_utm(lat_deg, lon_deg, zone);
                Some(Vec3::new(easting - o[0], northing - o[1], h - o[2]))
            }
            SourceFrame::Geographic | SourceFrame::Ecef => {
                let o = self.origin_ecef?;
                let origin = Vec3::new(o[0], o[1], o[2]);
                Some(self.rot * (geodetic_to_ecef(lat_deg, lon_deg, h) - origin))
            }
        }
    }

    /// Whether the block can be placed on the earth. A recognised UTM zone
    /// counts: it anchors the block even though it never moves it.
    pub fn is_georeferenced(&self) -> bool {
        self.origin_ecef.is_some() || self.utm.is_some()
    }

    pub fn describe(&self) -> String {
        match self.source {
            SourceFrame::Geographic => format!(
                "geographic -> ENU at {:.7}, {:.7}, {:.2} m",
                self.lat0, self.lon0, self.h0
            ),
            SourceFrame::Ecef => format!(
                "ECEF -> ENU at {:.7}, {:.7}, {:.2} m",
                self.lat0, self.lon0, self.h0
            ),
            SourceFrame::Projected => {
                let o = self.origin_source.unwrap_or([0.0; 3]);
                match self.utm {
                    Some(zone) => format!(
                        "projected {} -> recentred on ({:.3}, {:.3}, {:.3})",
                        zone.label(),
                        o[0],
                        o[1],
                        o[2]
                    ),
                    None => format!(
                        "projected -> recentred on ({:.3}, {:.3}, {:.3}), zone unrecognised",
                        o[0], o[1], o[2]
                    ),
                }
            }
        }
    }
}

#[cfg(test)]
mod up_axis_tests {
    use super::*;
    use crate::block::Photo;

    #[test]
    fn y_up_sends_the_vertical_to_y_and_is_a_rotation() {
        let r = UpAxis::Y.rotation();
        assert_eq!(r * Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 1.0, 0.0), "up");
        assert_eq!(r * Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), "east");
        assert_eq!(r * Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, -1.0), "north");
        // A reflection here would mirror the scene, which trains without
        // complaint and is wrong.
        assert!((r.det() - 1.0).abs() < 1e-12, "not a proper rotation");
    }

    #[test]
    fn z_up_changes_nothing_at_all() {
        assert_eq!(UpAxis::Z.rotation(), Mat3::IDENTITY);
        let mut b = Block {
            groups: vec![],
            photos: vec![Photo {
                id: 0,
                group: 0,
                path: "a.jpg".into(),
                r: Mat3::from_opk(11.0, -22.0, 33.0),
                c: Vec3::new(3.0, -4.0, 5.0),
            }],
            ..Default::default()
        };
        let before = b.photos[0].clone();
        apply_up_axis(&mut b, UpAxis::Z);
        assert_eq!(b.photos[0].c, before.c);
        assert_eq!(b.photos[0].r, before.r);
    }

    #[test]
    fn reprojection_survives_the_up_axis_change() {
        // The same standard the frame change is held to: a world point must
        // land on the same pixel before and after.
        let photo = Photo {
            id: 0,
            group: 0,
            path: "a.jpg".into(),
            r: Mat3::from_opk(8.0, -15.0, 120.0).orthonormalize(),
            c: Vec3::new(12.0, -30.0, 95.0),
        };
        let mut b = Block {
            groups: vec![],
            photos: vec![photo.clone()],
            ..Default::default()
        };
        let x_world = Vec3::new(-40.0, 22.0, 3.0);

        let v0 = photo.r * (x_world - photo.c);

        apply_up_axis(&mut b, UpAxis::Y);
        let p1 = &b.photos[0];
        let v1 = p1.r * (UpAxis::Y.rotation() * x_world - p1.c);

        assert!(
            (v0 - v1).length() < 1e-9,
            "the point moved in camera space: {v0} vs {v1}"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::{CameraOrientation, Distortion, Photo, Photogroup};

    fn group() -> Photogroup {
        Photogroup {
            index: 0,
            name: "t".into(),
            width: 4000,
            height: 3000,
            fx: 3000.0,
            fy: 3000.0,
            cx: 2000.5,
            cy: 1500.5,
            distortion: Distortion::default(),
            orientation: CameraOrientation::XRightYDown,
        }
    }

    /// A small nadir grid at a plausible site, expressed in ECEF.
    fn ecef_block() -> (Block, Vec<Vec3>) {
        let (lat0, lon0, h0) = (-33.61_f64, 150.75_f64, 120.0_f64);
        let origin = geodetic_to_ecef(lat0, lon0, h0);
        let renu = enu_rotation(lat0, lon0);
        let mut b = Block {
            groups: vec![group()],
            ..Default::default()
        };
        b.srs.push(crate::block::Srs {
            id: "0".into(),
            name: "WGS 84".into(),
            definition: "EPSG:4978".into(),
        });
        let mut truth = Vec::new();
        for gx in 0..4 {
            for gy in 0..4 {
                let enu = Vec3::new(gx as f64 * 25.0 - 40.0, gy as f64 * 25.0 - 40.0, 90.0);
                truth.push(enu);
                let ecef = origin + renu.transpose() * enu;
                // Nadir camera: x=east, y=south, z=down.
                let r_cam_enu = Mat3::diag(1.0, -1.0, -1.0);
                b.photos.push(Photo {
                    id: b.photos.len() as u32,
                    group: 0,
                    path: format!("IMG_{}.JPG", b.photos.len()),
                    r: r_cam_enu * renu,
                    c: ecef,
                });
            }
        }
        (b, truth)
    }

    /// The Quarry site's own grid origin, in UTM zone 56S metres — the number
    /// Terra recentred that block on. Paired with (-34.254729862,
    /// 150.591903267) in geodesy's tests, so the two modules can be checked
    /// against each other rather than against themselves.
    const QUARRY_EASTING: f64 = 278_265.818;
    const QUARRY_NORTHING: f64 = 6_206_860.195;
    const QUARRY_LAT: f64 = -34.254_729_862;
    const QUARRY_LON: f64 = 150.591_903_267;

    /// A projected block sitting on the Quarry grid. `srs` is the Definition
    /// string Terra would have written, or `None` for a block that declares
    /// no spatial reference at all.
    fn projected_block(srs: Option<&str>) -> Block {
        let mut b = Block {
            groups: vec![group()],
            ..Default::default()
        };
        if let Some(definition) = srs {
            b.srs.push(crate::block::Srs {
                id: "0".into(),
                name: "WGS 84 / UTM zone 56S".into(),
                definition: definition.into(),
            });
        }
        // 3x3 flight grid, symmetric about the origin so the centroid is the
        // grid origin itself and the geodetic checks below have a known answer.
        for i in 0..9u32 {
            let (gx, gy) = ((i % 3) as f64 - 1.0, (i / 3) as f64 - 1.0);
            b.photos.push(Photo {
                id: i,
                group: 0,
                path: format!("DJI_{i:04}.JPG"),
                r: Mat3::from_opk(3.0, -2.0, 91.0).orthonormalize(),
                c: Vec3::new(
                    QUARRY_EASTING + gx * 137.5,
                    QUARRY_NORTHING + gy * 137.5,
                    453.305 + i as f64 * 0.7,
                ),
            });
        }
        b
    }

    #[test]
    fn detects_ecef_from_srs_string() {
        let (b, _) = ecef_block();
        assert_eq!(SourceFrame::detect(&b), SourceFrame::Ecef);
    }

    #[test]
    fn ecef_block_becomes_small_and_metric() {
        let (mut b, truth) = ecef_block();
        let f = LocalFrame::build_and_apply(&mut b);
        assert_eq!(f.source, SourceFrame::Ecef);

        // The test block was laid out on a tangent plane, but the transform
        // works on the ellipsoid, so agreement is limited by the sagitta over
        // the block: d^2 / 2R is roughly 0.1 mm across 75 m. Sub-millimetre
        // agreement is the correct expectation, not bit-exactness.
        let e = b.extent();
        assert!((e.x - 75.0).abs() < 1e-3, "east extent {} != 75", e.x);
        assert!((e.y - 75.0).abs() < 1e-3, "north extent {} != 75", e.y);

        // Cameras end up near the origin, not 6.4 million metres away.
        assert!(b.centroid().length() < 1e-3);

        // Relative geometry preserved against the synthetic truth.
        let t_mean = truth.iter().fold(Vec3::ZERO, |a, v| a + *v) / truth.len() as f64;
        for (p, t) in b.photos.iter().zip(&truth) {
            let want = *t - t_mean;
            assert!((p.c - want).length() < 1e-3, "{} vs {}", p.c, want);
        }
    }

    #[test]
    fn ecef_and_geographic_inputs_agree() {
        let (mut a, _) = ecef_block();

        // Same block, re-expressed in degrees.
        let mut g = a.clone();
        g.srs[0].definition = "EPSG:4326".into();
        for p in &mut g.photos {
            let (lat, lon, h) = ecef_to_geodetic(p.c);
            p.c = Vec3::new(lon, lat, h); // x=lon, y=lat
        }
        assert_eq!(SourceFrame::detect(&g), SourceFrame::Geographic);

        let fa = LocalFrame::build_and_apply(&mut a);
        let fg = LocalFrame::build_and_apply(&mut g);
        assert!((fa.lat0 - fg.lat0).abs() < 1e-6);

        for (pa, pg) in a.photos.iter().zip(&g.photos) {
            assert!(
                (pa.c - pg.c).length() < 1e-3,
                "ECEF and geographic paths disagree: {} vs {}",
                pa.c,
                pg.c
            );
        }
    }

    #[test]
    fn round_trips_back_to_geodetic() {
        let (mut b, _) = ecef_block();
        let f = LocalFrame::build_and_apply(&mut b);
        let (lat, lon, h) = f.to_geodetic(b.photos[0].c).unwrap();
        // Reconstruct and compare in ECEF, which avoids angle-wrap fiddliness.
        let there = geodetic_to_ecef(lat, lon, h);
        let back = f.point(there);
        assert!((back - b.photos[0].c).length() < 1e-6);
    }

    #[test]
    fn projected_input_is_only_recentred() {
        let mut b = Block {
            groups: vec![group()],
            ..Default::default()
        };
        for i in 0..5 {
            b.photos.push(Photo {
                id: i,
                group: 0,
                path: "x.jpg".into(),
                r: Mat3::IDENTITY,
                c: Vec3::new(295_000.0 + i as f64, 6_255_000.0, 300.0),
            });
        }
        assert_eq!(SourceFrame::detect(&b), SourceFrame::Projected);
        let f = LocalFrame::build_and_apply(&mut b);
        assert!(!f.is_georeferenced(), "projected blocks have no ECEF origin");
        assert!(b.centroid().length() < 1e-9);
        // Rotations must be untouched when there is no ENU rotation involved.
        assert_eq!(b.photos[0].r, Mat3::IDENTITY);
    }

    #[test]
    fn reprojection_survives_the_frame_change() {
        // The definitive test: a world point must land on the same pixel
        // before and after the transform.
        let (b0, _) = ecef_block();
        let mut b1 = b0.clone();
        let f = LocalFrame::build_and_apply(&mut b1);
        let g = &b0.groups[0];

        for (p0, p1) in b0.photos.iter().zip(&b1.photos) {
            // A point 90 m below the camera, offset a little.
            let x_src = p0.c + p0.r.transpose() * Vec3::new(12.0, -7.0, 90.0);
            let v0 = p0.r * (x_src - p0.c);
            let v1 = p1.r * f.point(x_src) + p1.translation();
            let (u0, w0) = (g.fx * v0.x / v0.z + g.cx, g.fy * v0.y / v0.z + g.cy);
            let (u1, w1) = (g.fx * v1.x / v1.z + g.cx, g.fy * v1.y / v1.z + g.cy);
            assert!(
                (u0 - u1).abs() < 1e-6 && (w0 - w1).abs() < 1e-6,
                "reprojection moved: ({u0}, {w0}) vs ({u1}, {w1})"
            );
        }
    }
}
