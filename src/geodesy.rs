//! WGS84 geodesy, hand-rolled to avoid a PROJ dependency.
//!
//! Only what is needed to move a photogrammetric block out of ECEF or
//! geographic degrees and into a small, metric, locally-levelled frame that
//! radiance-field trainers can handle without losing float precision.

use serde::Serialize;

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

// ---------------------------------------------------------------------------
// Transverse Mercator, so a UTM block can be understood without PROJ
// ---------------------------------------------------------------------------
//
// Terra writes most projects in a projected CRS, and until this existed the
// EPSG code was an opaque string: GeoJSON and KML came out empty, a cloud in
// degrees could not be brought alongside a block in metres, and a photo's GPS
// could not be compared with a camera centre.
//
// Note what this is *not* for. A projected block is still recentred by pure
// subtraction; see the note on INVARIANT 1 in frame.rs. UTM carries a scale
// factor of 0.9996 on the central meridian that grows outward, so pushing
// camera centres through this projection would non-rigidly warp a solve Terra
// computed in grid coordinates. These functions convert *between* frames for
// metadata and for foreign data; they never move the block.

/// Scale factor on the central meridian.
const K0: f64 = 0.9996;
const FALSE_EASTING: f64 = 500_000.0;
/// Southern-hemisphere northings are offset so they never go negative.
const FALSE_NORTHING_SOUTH: f64 = 10_000_000.0;

/// Third flattening, the expansion parameter of the Krüger series.
const N3: f64 = WGS84_F / (2.0 - WGS84_F);

/// A projected CRS this crate can interpret: UTM on the WGS84 ellipsoid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Utm {
    pub zone: u8,
    pub north: bool,
}

impl Utm {
    /// Longitude of the zone's central meridian, in degrees.
    pub fn central_meridian(self) -> f64 {
        6.0 * self.zone as f64 - 183.0
    }

    pub fn epsg(self) -> u32 {
        let base = if self.north { 32600 } else { 32700 };
        base + self.zone as u32
    }

    pub fn label(self) -> String {
        format!(
            "UTM zone {}{} (EPSG:{})",
            self.zone,
            if self.north { 'N' } else { 'S' },
            self.epsg()
        )
    }

    /// The zone containing a position. Part of the geodesy surface for callers
    /// that have an anchor but no declared CRS; exercised by the tests.
    ///
    /// The Norway and Svalbard exceptions are
    /// deliberately not modelled: a block that lands in one declares its own
    /// EPSG code, and guessing wrong there would be worse than not guessing.
    #[allow(dead_code)]
    pub fn for_lonlat(lon_deg: f64, lat_deg: f64) -> Self {
        let z = ((lon_deg + 180.0) / 6.0).floor() as i32 + 1;
        Self {
            zone: z.clamp(1, 60) as u8,
            north: lat_deg >= 0.0,
        }
    }

    /// Recognise a zone from whatever the file called its SRS.
    ///
    /// Terra is inconsistent here: sometimes a bare EPSG code, sometimes a
    /// WKT-ish name, sometimes the Australian MGA alias. Anything unrecognised
    /// returns None and the caller carries on without georeferencing rather
    /// than guessing a zone, because a wrong zone puts the block in the wrong
    /// hemisphere or several hundred kilometres sideways.
    pub fn parse(blob: &str) -> Option<Self> {
        let s = blob.to_ascii_lowercase();

        // An explicit EPSG code wins: 326xx north, 327xx south.
        for run in digit_runs(&s) {
            if run.len() == 5 {
                if let Ok(code) = run.parse::<u32>() {
                    if (32601..=32660).contains(&code) {
                        return Some(Self {
                            zone: (code - 32600) as u8,
                            north: true,
                        });
                    }
                    if (32701..=32760).contains(&code) {
                        return Some(Self {
                            zone: (code - 32700) as u8,
                            north: false,
                        });
                    }
                }
            }
        }

        // "UTM zone 56S", "MGA zone 56", "zone 56 south".
        let idx = s.find("zone")?;
        let rest = &s[idx + 4..];
        let run = digit_runs(rest).into_iter().find(|r| !r.is_empty())?;
        let zone: u8 = run.parse().ok()?;
        if !(1..=60).contains(&zone) {
            return None;
        }

        let after = rest.split_once(run).map(|(_, a)| a).unwrap_or("");
        let north = if after.trim_start().starts_with('s') || after.contains("south") {
            false
        } else if after.trim_start().starts_with('n') || after.contains("north") {
            true
        } else if s.contains("mga") {
            // The Map Grid of Australia is southern by definition.
            false
        } else {
            return None;
        };

        Some(Self { zone, north })
    }
}

fn digit_runs(s: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let mut start: Option<usize> = None;
    for (i, c) in s.char_indices() {
        if c.is_ascii_digit() {
            if start.is_none() {
                start = Some(i);
            }
        } else if let Some(st) = start.take() {
            out.push(&s[st..i]);
        }
    }
    if let Some(st) = start {
        out.push(&s[st..]);
    }
    out
}

/// Rectifying radius: the radius of a sphere with the same meridian length.
fn rectifying_radius() -> f64 {
    let n = N3;
    WGS84_A / (1.0 + n) * (1.0 + n.powi(2) / 4.0 + n.powi(4) / 64.0 + n.powi(6) / 256.0)
}

/// Krüger series to sixth order. Truncating earlier is the usual source of
/// error that grows with distance from the central meridian — small enough at
/// the meridian to look correct, metres out at the zone edge.
fn alpha() -> [f64; 6] {
    let n = N3;
    let (n2, n3, n4, n5, n6) = (n * n, n.powi(3), n.powi(4), n.powi(5), n.powi(6));
    [
        n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0
            + 7891.0 * n6 / 37800.0,
        13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0
            - 1_983_433.0 * n6 / 1_935_360.0,
        61.0 * n3 / 240.0 - 103.0 * n4 / 140.0
            + 15061.0 * n5 / 26880.0
            + 167_603.0 * n6 / 181_440.0,
        49561.0 * n4 / 161_280.0 - 179.0 * n5 / 168.0 + 6_601_661.0 * n6 / 7_257_600.0,
        34729.0 * n5 / 80640.0 - 3_418_889.0 * n6 / 1_995_840.0,
        212_378_941.0 * n6 / 319_334_400.0,
    ]
}

fn beta() -> [f64; 6] {
    let n = N3;
    let (n2, n3, n4, n5, n6) = (n * n, n.powi(3), n.powi(4), n.powi(5), n.powi(6));
    [
        n / 2.0 - 2.0 * n2 / 3.0 + 37.0 * n3 / 96.0 - n4 / 360.0 - 81.0 * n5 / 512.0
            + 96199.0 * n6 / 604_800.0,
        n2 / 48.0 + n3 / 15.0 - 437.0 * n4 / 1440.0 + 46.0 * n5 / 105.0
            - 1_118_711.0 * n6 / 3_870_720.0,
        17.0 * n3 / 480.0 - 37.0 * n4 / 840.0 - 209.0 * n5 / 4480.0 + 5569.0 * n6 / 90720.0,
        4397.0 * n4 / 161_280.0 - 11.0 * n5 / 504.0 - 830_251.0 * n6 / 7_257_600.0,
        4583.0 * n5 / 161_280.0 - 108_847.0 * n6 / 3_991_680.0,
        20_648_693.0 * n6 / 638_668_800.0,
    ]
}

fn delta() -> [f64; 6] {
    let n = N3;
    let (n2, n3, n4, n5, n6) = (n * n, n.powi(3), n.powi(4), n.powi(5), n.powi(6));
    [
        2.0 * n - 2.0 * n2 / 3.0 - 2.0 * n3 + 116.0 * n4 / 45.0 + 26.0 * n5 / 45.0
            - 2854.0 * n6 / 675.0,
        7.0 * n2 / 3.0 - 8.0 * n3 / 5.0 - 227.0 * n4 / 45.0
            + 2704.0 * n5 / 315.0
            + 2323.0 * n6 / 945.0,
        56.0 * n3 / 15.0 - 136.0 * n4 / 35.0 - 1262.0 * n5 / 105.0 + 73814.0 * n6 / 2835.0,
        4279.0 * n4 / 630.0 - 332.0 * n5 / 35.0 - 399_572.0 * n6 / 14175.0,
        4174.0 * n5 / 315.0 - 144_838.0 * n6 / 6237.0,
        601_676.0 * n6 / 22680.0,
    ]
}

fn wrap_pi(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let mut x = (a + std::f64::consts::PI) % tau;
    if x < 0.0 {
        x += tau;
    }
    x - std::f64::consts::PI
}

/// Geodetic to UTM grid. Returns (easting, northing) in metres.
pub fn geodetic_to_utm(lat_deg: f64, lon_deg: f64, zone: Utm) -> (f64, f64) {
    let phi = lat_deg.to_radians();
    let lam = wrap_pi((lon_deg - zone.central_meridian()).to_radians());

    let n = N3;
    // 2*sqrt(n)/(1+n) is the first eccentricity, to within rounding.
    let ecc = 2.0 * n.sqrt() / (1.0 + n);
    let sp = phi.sin().clamp(-1.0 + 1e-15, 1.0 - 1e-15);

    let t = (sp.atanh() - ecc * (ecc * sp).atanh()).sinh();
    let xi_p = t.atan2(lam.cos());
    let eta_p = (lam.sin() / (1.0 + t * t).sqrt()).atanh();

    let a = rectifying_radius();
    let al = alpha();
    let mut xi = xi_p;
    let mut eta = eta_p;
    for (j, c) in al.iter().enumerate() {
        let two_j = 2.0 * (j + 1) as f64;
        xi += c * (two_j * xi_p).sin() * (two_j * eta_p).cosh();
        eta += c * (two_j * xi_p).cos() * (two_j * eta_p).sinh();
    }

    let easting = FALSE_EASTING + K0 * a * eta;
    let northing = K0 * a * xi
        + if zone.north {
            0.0
        } else {
            FALSE_NORTHING_SOUTH
        };
    (easting, northing)
}

/// UTM grid to geodetic. Returns (latitude, longitude) in degrees.
pub fn utm_to_geodetic(easting: f64, northing: f64, zone: Utm) -> (f64, f64) {
    let a = rectifying_radius();
    let n0 = if zone.north {
        0.0
    } else {
        FALSE_NORTHING_SOUTH
    };

    let xi = (northing - n0) / (K0 * a);
    let eta = (easting - FALSE_EASTING) / (K0 * a);

    let be = beta();
    let mut xi_p = xi;
    let mut eta_p = eta;
    for (j, c) in be.iter().enumerate() {
        let two_j = 2.0 * (j + 1) as f64;
        xi_p -= c * (two_j * xi).sin() * (two_j * eta).cosh();
        eta_p -= c * (two_j * xi).cos() * (two_j * eta).sinh();
    }

    let chi = (xi_p.sin() / eta_p.cosh()).clamp(-1.0, 1.0).asin();
    let de = delta();
    let mut phi = chi;
    for (j, c) in de.iter().enumerate() {
        phi += c * (2.0 * (j + 1) as f64 * chi).sin();
    }

    let lam = eta_p.sinh().atan2(xi_p.cos());
    (phi.to_degrees(), zone.central_meridian() + lam.to_degrees())
}

#[cfg(test)]
mod utm_tests {
    use super::*;

    /// The site the whole project was validated against.
    const QUARRY_LAT: f64 = -34.254_729_862;
    const QUARRY_LON: f64 = 150.591_903_267;

    #[test]
    fn zone_56s_is_where_the_quarry_is() {
        let z = Utm::for_lonlat(QUARRY_LON, QUARRY_LAT);
        assert_eq!(
            z,
            Utm {
                zone: 56,
                north: false
            }
        );
        assert_eq!(z.epsg(), 32756);
        assert_eq!(z.central_meridian(), 153.0);
    }

    #[test]
    fn parses_every_form_terra_and_its_neighbours_write() {
        let s = Utm {
            zone: 56,
            north: false,
        };
        for blob in [
            "EPSG:32756",
            "epsg:32756",
            "WGS 84 / UTM zone 56S EPSG:32756",
            "WGS 84 / UTM zone 56S",
            "utm zone 56 south",
            "GDA94 / MGA zone 56",
        ] {
            assert_eq!(Utm::parse(blob), Some(s), "failed on {blob:?}");
        }
        assert_eq!(
            Utm::parse("WGS 84 / UTM zone 31N"),
            Some(Utm {
                zone: 31,
                north: true
            })
        );
    }

    #[test]
    fn refuses_to_guess_at_an_unrecognised_srs() {
        for blob in ["EPSG:4978", "EPSG:4326", "", "some local grid", "zone 99N"] {
            assert_eq!(Utm::parse(blob), None, "should not have matched {blob:?}");
        }
    }

    #[test]
    fn the_central_meridian_on_the_equator_is_the_grid_origin() {
        for zone in [
            Utm {
                zone: 56,
                north: true,
            },
            Utm {
                zone: 31,
                north: true,
            },
        ] {
            let (e, n) = geodetic_to_utm(0.0, zone.central_meridian(), zone);
            assert!((e - FALSE_EASTING).abs() < 1e-6, "easting {e}");
            assert!(n.abs() < 1e-6, "northing {n}");
        }
    }

    #[test]
    fn southern_hemisphere_carries_the_false_northing() {
        let z = Utm {
            zone: 56,
            north: false,
        };
        let (_, n) = geodetic_to_utm(0.0, 153.0, z);
        assert!(
            (n - FALSE_NORTHING_SOUTH).abs() < 1e-6,
            "the 10,000,000 m offset is missing or doubled: {n}"
        );
    }

    #[test]
    fn utm_roundtrip_holds_across_zones_and_hemispheres() {
        let cases = [
            (
                QUARRY_LAT,
                QUARRY_LON,
                Utm {
                    zone: 56,
                    north: false,
                },
            ),
            (
                -33.8688,
                151.2093,
                Utm {
                    zone: 56,
                    north: false,
                },
            ),
            (
                51.4778,
                -0.0015,
                Utm {
                    zone: 30,
                    north: true,
                },
            ),
            (
                0.0,
                153.0,
                Utm {
                    zone: 56,
                    north: true,
                },
            ),
            (
                60.0,
                5.0,
                Utm {
                    zone: 32,
                    north: true,
                },
            ),
            (
                -45.0,
                170.5,
                Utm {
                    zone: 59,
                    north: false,
                },
            ),
            // Deliberately near the zone edge, where a truncated series shows.
            (
                -34.0,
                155.9,
                Utm {
                    zone: 56,
                    north: false,
                },
            ),
            (
                -34.0,
                150.1,
                Utm {
                    zone: 56,
                    north: false,
                },
            ),
        ];
        for (lat, lon, z) in cases {
            let (e, n) = geodetic_to_utm(lat, lon, z);
            let (lat2, lon2) = utm_to_geodetic(e, n, z);
            assert!(
                (lat - lat2).abs() < 1e-9,
                "lat {lat} -> {lat2} in {}",
                z.label()
            );
            assert!(
                (lon - lon2).abs() < 1e-9,
                "lon {lon} -> {lon2} in {}",
                z.label()
            );
        }
    }

    #[test]
    fn the_projection_agrees_with_terras_own_numbers() {
        // Terra recentred this block on easting 278265.818, northing 6206860.195,
        // which is the camera centroid, while AT/sfm_geo_desc.json gives the site
        // reference as the geodetic anchor below. They are different points a few
        // hundred metres apart, so this asserts they are the same PLACE — which is
        // what catches a hemisphere error, a wrong zone, or a dropped false
        // northing, all of which land kilometres or megametres away.
        let z = Utm {
            zone: 56,
            north: false,
        };
        let (e, n) = geodetic_to_utm(QUARRY_LAT, QUARRY_LON, z);
        let (de, dn) = (e - 278_265.818, n - 6_206_860.195);
        assert!(
            de.abs() < 2000.0 && dn.abs() < 2000.0,
            "projected to ({e:.3}, {n:.3}), which is {de:.1} m east and {dn:.1} m \
             north of where Terra put the same site"
        );
    }

    #[test]
    fn grid_distance_matches_ground_distance_near_the_meridian() {
        // Two points 1000 m apart in northing should be ~1000/k0 apart on the
        // ground. This catches a scale factor applied the wrong way round.
        let z = Utm {
            zone: 56,
            north: false,
        };
        let (e0, n0) = geodetic_to_utm(-34.0, 153.0, z);
        let (lat1, lon1) = utm_to_geodetic(e0, n0 + 1000.0, z);
        let a = geodetic_to_ecef(-34.0, 153.0, 0.0);
        let b = geodetic_to_ecef(lat1, lon1, 0.0);
        let ground = (b - a).length();
        assert!(
            (ground - 1000.0 / K0).abs() < 0.5,
            "1000 m of grid became {ground:.3} m of ground"
        );
    }
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
