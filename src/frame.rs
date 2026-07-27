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

use serde::Serialize;

use crate::block::Block;
use crate::geodesy::{ecef_to_geodetic, enu_rotation, geodetic_to_ecef};
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
                    rot: enu_rotation(lat0, lon0),
                }
            }
            SourceFrame::Projected => {
                let origin = block.centroid();
                Self {
                    source,
                    origin_ecef: None,
                    origin_source: Some(origin.to_array()),
                    lat0: f64::NAN,
                    lon0: f64::NAN,
                    h0: f64::NAN,
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
    /// Returns `None` for projected blocks, where the CRS is unknown to us.
    pub fn to_geodetic(&self, local: Vec3) -> Option<(f64, f64, f64)> {
        let o = self.origin_ecef?;
        let origin = Vec3::new(o[0], o[1], o[2]);
        let ecef = origin + self.rot.transpose() * local;
        Some(ecef_to_geodetic(ecef))
    }

    pub fn is_georeferenced(&self) -> bool {
        self.origin_ecef.is_some()
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
                format!(
                    "projected -> recentred on ({:.3}, {:.3}, {:.3})",
                    o[0], o[1], o[2]
                )
            }
        }
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
