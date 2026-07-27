//! WGS84 geodesy, hand-rolled to avoid a PROJ dependency.
//!
//! Only what is needed to move a photogrammetric block out of ECEF or
//! geographic degrees and into a small, metric, locally-levelled frame that
//! radiance-field trainers can handle without losing float precision.

use crate::math::{Mat3, Vec3};

pub const WGS84_A: f64 = 6_378_137.0;
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
pub const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);

/// Geodetic (degrees, degrees, metres) to earth-centred earth-fixed.
pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, h: f64) -> Vec3 {
    let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
    let s = lat.sin();
    let n = WGS84_A / (1.0 - WGS84_E2 * s * s).sqrt();
    Vec3::new(
        (n + h) * lat.cos() * lon.cos(),
        (n + h) * lat.cos() * lon.sin(),
        (n * (1.0 - WGS84_E2) + h) * s,
    )
}

/// ECEF to geodetic (lat_deg, lon_deg, height_m) via Bowring's method.
///
/// One Bowring pass is accurate to well under a millimetre for terrestrial
/// heights, which is far tighter than anything downstream needs.
pub fn ecef_to_geodetic(p: Vec3) -> (f64, f64, f64) {
    let lon = p.y.atan2(p.x);
    let r = (p.x * p.x + p.y * p.y).sqrt();
    let ep2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);
    let theta = (p.z * WGS84_A).atan2(r * WGS84_B);
    let lat = (p.z + ep2 * WGS84_B * theta.sin().powi(3))
        .atan2(r - WGS84_E2 * WGS84_A * theta.cos().powi(3));
    let s = lat.sin();
    let n = WGS84_A / (1.0 - WGS84_E2 * s * s).sqrt();
    let h = if lat.cos().abs() > 1e-12 {
        r / lat.cos() - n
    } else {
        p.z.abs() - WGS84_B
    };
    (lat.to_degrees(), lon.to_degrees(), h)
}

/// Rotation taking ECEF *vectors* into the local east-north-up frame at
/// (lat, lon). Note this rotates directions, not positions: subtract the
/// origin first.
pub fn enu_rotation(lat_deg: f64, lon_deg: f64) -> Mat3 {
    let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
    let (sl, cl) = (lat.sin(), lat.cos());
    let (so, co) = (lon.sin(), lon.cos());
    Mat3::from_rows([
        [-so, co, 0.0],
        [-sl * co, -sl * so, cl],
        [cl * co, cl * so, sl],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geodetic_ecef_roundtrip() {
        // Spread across hemispheres, including a polar and an equatorial case.
        for &(lat, lon, h) in &[
            (-33.61_f64, 150.75_f64, 210.0_f64),
            (51.45, -2.58, 60.0),
            (0.0, 0.0, 0.0),
            (89.5, 12.0, 1500.0),
            (-45.0, 179.9, -30.0),
        ] {
            let e = geodetic_to_ecef(lat, lon, h);
            let (lat2, lon2, h2) = ecef_to_geodetic(e);
            assert!((lat - lat2).abs() < 1e-9, "lat {lat} vs {lat2}");
            assert!((lon - lon2).abs() < 1e-9, "lon {lon} vs {lon2}");
            assert!((h - h2).abs() < 1e-6, "h {h} vs {h2}");
        }
    }

    #[test]
    fn enu_axes_point_the_right_way() {
        let (lat, lon) = (-33.61, 150.75);
        let r = enu_rotation(lat, lon);
        let origin = geodetic_to_ecef(lat, lon, 0.0);

        // 1 metre up should map to +Z in ENU.
        let up = geodetic_to_ecef(lat, lon, 1.0) - origin;
        let up_enu = r * up;
        assert!(up_enu.z > 0.999 && up_enu.x.abs() < 1e-6 && up_enu.y.abs() < 1e-6);

        // A small northward step should map to +Y.
        let north = geodetic_to_ecef(lat + 0.001, lon, 0.0) - origin;
        let north_enu = r * north;
        assert!(north_enu.y > 0.0 && north_enu.y.abs() > north_enu.x.abs() * 100.0);

        // A small eastward step should map to +X.
        let east = geodetic_to_ecef(lat, lon + 0.001, 0.0) - origin;
        let east_enu = r * east;
        assert!(east_enu.x > 0.0 && east_enu.x.abs() > east_enu.y.abs() * 100.0);
    }

    #[test]
    fn enu_preserves_distance() {
        let (lat, lon) = (-33.61, 150.75);
        let r = enu_rotation(lat, lon);
        let a = geodetic_to_ecef(lat, lon, 100.0);
        let b = geodetic_to_ecef(lat + 0.002, lon + 0.003, 140.0);
        let d_ecef = (b - a).length();
        let d_enu = ((r * b) - (r * a)).length();
        assert!((d_ecef - d_enu).abs() < 1e-9, "rotation changed scale");
    }
}
