# terrapin — module interface contract (ADR-001 implementation)

The work order for the ADR-001 implementation pass, kept for the dataset
findings it records rather than as a description of the code. Where the two
disagree the code wins: the `cloud.rs` section below, `MatchReport`'s
`had_collisions` field, the `stage` subcommand and the `--for <target>`
preset were all specified here and were not built this way.

## Ground truth from the real dataset

Validated against `D:\Papers\Photogrammetry\2026-07-26-1738`:

- Block: `Quarry Test 1\AT\BlocksExchangeUndistortAT_WithoutTiePoints.xml`,
  1680 photos, SRS `WGS 84 / UTM zone 56S EPSG:32756`, all distortion zeroed,
  1680 identical photogroups (one per photo), image size 5280x3956,
  fx 3474.5198, fy 3357.7418, cx 2681.2827, cy 1961.3224 (corner origin,
  the +0.5 shift is already applied by `block.rs`).
- Original calibration in `AT\report\report.md`, "Camera Calibration
  Information" section, `Optimized` row: Focal 3716.60, Cx 2679.09,
  Cy 1961.69, K1 -0.10474077, K2 -0.00564897, K3 -0.01441997, K4 0.0,
  P1 0.00009975, P2 -0.00071999.
- Originals: `DCIM\DJI_202607261624_006_Qaury-test1\*.JPG`, 1680 files named
  `DJI_<timestamp>_<index>_V.JPG` with index 1..1681 — **index 2 is absent**.
  Any index-order or sort-order pairing is therefore wrong; match on position.
- `.MRK` file in the same folder: 1680 tab-separated rows, column 0 is the
  image index, then `<lat>,Lat`, `<lon>,Lon`, `<ellh>,Ellh` fields.
- Each JPEG carries a plain-text XMP packet (`drone-dji` namespace) within the
  first ~64 KB with `drone-dji:GpsLatitude`, `GpsLongitude`,
  `AbsoluteAltitude`, `RtkFlag`. No binary EXIF parsing is required.
- `models\pc\0\terra_ply\Block*/Block*.ply` — nested one level deeper than a
  flat scan finds; **textured meshes**, `element face` non-zero, vertices are
  `float x/y/z` with **no colour**, in a local frame.
- `models\pc\0\terra_ply\metadata.xml` — **UTF-16LE encoded**, contains
  `<SRS>EPSG:4326</SRS>` and
  `<SRSOrigin>150.5919103626999,-34.25427298169999,453.30499999992935</SRSOrigin>`
  (lon, lat, height).
- `models\pc\0\terra_las_1_4\*.las` — 16 tiles, 158 M points, **coordinates in
  geographic degrees**, colour present.
- `AT\sfm_geo_desc.json` — `cs_type: LOCAL_ENU_CS`, `ref_GPS` lat/lon/altitude.

## Two invariants that must not be broken

**INVARIANT 1 — projected blocks stay rigid.** A block in a projected CRS is
recentred by subtraction only. Do **not** convert its camera centres through
UTM→geodetic→ENU: UTM carries a scale factor (0.9996 on the central meridian,
growing outward), so that conversion is a non-rigid warp of up to ~0.04%,
which is ~40 cm across a 1 km block. Terra solved the block in grid
coordinates; warping it silently degrades the solve. UTM knowledge is used
*only* for georeferencing metadata, for bringing foreign clouds into the
block's frame, and for matching images to poses. `frame.rs` tests already pin
this — `reprojection_survives_the_frame_change` must keep passing untouched.

**INVARIANT 2 — poses are lens-independent.** `Photo::r` and `Photo::c` are the
physical camera orientation and position. They are valid with *either* the
undistorted images plus the XML pinhole intrinsics, *or* the original JPEGs
plus the original distortion model. Never adjust a pose to "compensate" for a
change of camera model.

---

## `geodesy.rs` — additions (owner: geodesy agent)

Keep everything already in the file. Add transverse Mercator so UTM zones can
be interpreted without PROJ.

```rust
/// A projected CRS this crate can interpret. UTM on WGS84 only.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub struct Utm { pub zone: u8, pub north: bool }

impl Utm {
    /// Recognise a zone from an SRS blob. Must handle at least:
    ///   "EPSG:32756", "epsg:32756", "32756",
    ///   "WGS 84 / UTM zone 56S", "UTM zone 56N", "MGA zone 56" (=> 327xx,
    ///   since MGA is southern hemisphere).
    /// Returns None when no zone can be established.
    pub fn parse(blob: &str) -> Option<Self>;

    /// The zone that contains this position. Used when a block declares no
    /// usable SRS but a geodetic anchor is known from elsewhere.
    pub fn for_lonlat(lon_deg: f64, lat_deg: f64) -> Self;

    /// 32601..32660 for north, 32701..32760 for south.
    pub fn epsg(self) -> u32;

    /// e.g. "UTM zone 56S (EPSG:32756)".
    pub fn label(self) -> String;

    /// Longitude of the zone's central meridian, degrees.
    pub fn central_meridian(self) -> f64;
}

/// Geodetic to UTM grid. Returns (easting_m, northing_m).
/// Krüger series to 6th order: sub-millimetre within a zone's valid band.
pub fn geodetic_to_utm(lat_deg: f64, lon_deg: f64, zone: Utm) -> (f64, f64);

/// UTM grid to geodetic. Returns (lat_deg, lon_deg).
pub fn utm_to_geodetic(easting: f64, northing: f64, zone: Utm) -> (f64, f64);
```

Constants: scale factor k0 = 0.9996, false easting 500000.0, false northing
0.0 north / 10000000.0 south.

Required tests:
- Round-trip geodetic→UTM→geodetic under 1e-9 deg across zones, both
  hemispheres, near and far from the central meridian, including the Quarry
  anchor (-34.254729862, 150.591903267) which must land in zone 56S.
- The Quarry anchor must produce an easting/northing consistent with the
  block's own recentring origin (278265.818, 6206860.195) to within ~2 m —
  this cross-validates the projection against Terra's own numbers.
- `Utm::parse` accepts every string form listed above and rejects junk.
- A known published control point (any authoritative UTM sample you are
  confident in) to within 1 mm.

## `frame.rs` — changes (owner: frame agent)

Respect INVARIANT 1. `LocalFrame` gains a UTM field and gains the inverse map.

```rust
pub struct LocalFrame {
    // ... all existing fields unchanged ...
    /// Set when the block is projected and the zone was recognised.
    pub utm: Option<crate::geodesy::Utm>,
}
```

- `build_and_apply`: unchanged behaviour for Ecef and Geographic. For
  `Projected`, keep the existing pure-subtraction recentring and identity
  rotation, but additionally set `utm` from `Utm::parse(&block.srs_blob())`.
  If parsing fails, leave `utm: None` — everything must still work.
- `is_georeferenced()` becomes `self.origin_ecef.is_some() || self.utm.is_some()`.
- `to_geodetic(local)` gains the projected branch: add `origin_source`, then
  `utm_to_geodetic`, returning `(lat, lon, local.z + origin_source.z)`.
- **New** — the inverse, needed by cloud unification and image matching:
  ```rust
  /// Map a geodetic position into the working local frame.
  /// None when the frame has no geodetic anchor (projected, unknown zone).
  pub fn geodetic_to_local(&self, lat_deg: f64, lon_deg: f64, h: f64) -> Option<Vec3>;
  ```
  Projected+utm: `geodetic_to_utm` then subtract `origin_source`.
  Geographic/Ecef: `geodetic_to_ecef` then `rot * (p - origin_ecef)`.
- `describe()` for projected should name the zone when known.

Required tests: `geodetic_to_local` and `to_geodetic` round-trip to <1e-6 m for
all three source frames; existing tests still pass verbatim; a projected block
with a recognised zone reports `is_georeferenced() == true` while its camera
centres remain **bit-identical** to the pure-subtraction result.

## `cloud.rs` — changes (owner: cloud agent)

```rust
/// What a directory of tiles turned out to contain.
pub struct CloudScan {
    pub tiles: Vec<PathBuf>,
    pub kind: TileKind,          // Points | Mesh | Mixed
    pub has_color: bool,
    pub meta: Option<TileMeta>,  // from a metadata.xml found near the tiles
}

pub enum TileKind { Points, Mesh, Mixed }

/// Parsed from Terra's metadata.xml beside the tiles. The file is UTF-16LE
/// with a BOM; decode accordingly (and still accept UTF-8).
pub struct TileMeta {
    pub srs: Option<String>,
    /// (lon, lat, height) as written in <SRSOrigin>.
    pub srs_origin: Option<(f64, f64, f64)>,
}

/// Recursive, depth-bounded (max 4) scan for .ply/.las. Sniffs PLY headers
/// without reading bodies.
pub fn scan_dir(dir: &Path) -> Result<CloudScan>;

/// Which frame the loaded points appear to be in.
pub enum CloudFrame {
    /// x=lon, y=lat degrees (or the reverse — see `load_and_place`).
    Geographic,
    /// Local ENU anchored at the metadata SRSOrigin.
    LocalEnu { lat: f64, lon: f64, h: f64 },
    /// Already in the same coordinates as the block.
    SameAsBlock,
}

pub fn detect_frame(cloud: &Cloud, meta: Option<&TileMeta>) -> CloudFrame;
```

Also required:
- `load_dir` keeps its signature but scans recursively via `scan_dir`.
- PLY sniffing: report `element face` > 0 as `Mesh` (still load the vertices —
  they are perfectly good points), and report whether any colour property
  exists. A mesh tile without colour must fall back to `DEFAULT_COLOR`
  rather than failing.
- Per-tile decimation before merging, so 158 M points never all sit in memory
  at once. Add `load_dir(dir, per_tile_cap)` usage accordingly: when a global
  target of N is requested across T tiles, cap each tile at roughly
  `2 * N / T` and decimate again after merge. Say so in the printed output.
- **Longitude/latitude order is ambiguous** in the LAS tiles. Do not guess
  from convention: transform under both orderings and keep whichever centroid
  lands closer to the camera centroid, then report which was used. This is the
  behaviour the users asked for — absorb the ambiguity, state the resolution.

Required tests: a nested temp directory is found; a mesh PLY (with a face
element) loads its vertices; a UTF-16LE metadata.xml parses; the
lon/lat-order chooser picks correctly given a synthetic block and cloud.

## `stage.rs` — new module (owner: stage agent)

The critical path: turn "the images the XML references do not exist" into "a
folder of images that matches the poses".

```rust
/// The physical camera as solved, recovered from AT/report/report.md.
#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    pub focal: f64,
    pub cx: f64, pub cy: f64,       // corner origin: +0.5 already applied
    pub k1: f64, pub k2: f64, pub k3: f64, pub k4: f64,
    pub p1: f64, pub p2: f64,
}

/// Parse the "Camera Calibration Information" table. Prefer the `Optimized`
/// row; fall back to `Initial`. Returns Ok(None) when the section is absent
/// or its shape changed — a schema change must cost a feature, never a crash.
pub fn calibration_from_report(at_dir: &Path) -> Result<Option<Calibration>>;

/// One original photo found on disk, with the position from its XMP packet.
pub struct SourceImage {
    pub path: PathBuf,
    pub lat: f64, pub lon: f64, pub alt: f64,
    pub index: Option<u32>,     // parsed from DJI_<ts>_<index>_V.JPG
    pub rtk_fixed: bool,        // RtkFlag == 50
}

/// Recursively find JPEGs and read their XMP GPS. Depth-bounded (max 6).
/// Reads only the first 128 KB of each file. Files without a usable packet
/// are skipped and counted, never fatal.
pub fn scan_images(roots: &[PathBuf]) -> Result<(Vec<SourceImage>, usize)>;

/// Optional cross-check: DJI .MRK RTK event table, index -> lat/lon/height.
pub fn read_mrk(path: &Path) -> Result<Vec<(u32, f64, f64, f64)>>;

pub struct MatchReport {
    /// Parallel to block.photos: the chosen source file, if any.
    pub chosen: Vec<Option<PathBuf>>,
    pub matched: usize,
    pub unmatched: usize,
    pub median_distance_m: f64,
    pub worst_distance_m: f64,
    /// True when any source image was picked for two different poses.
    pub had_collisions: bool,
}

/// Match poses to files by position. Convert each SourceImage into the local
/// frame with `frame.geodetic_to_local`, then pair with the nearest camera
/// centre. Must be mutual-nearest / collision-aware: a greedy nearest-only
/// pass can assign one file to several poses. Reject a pairing beyond
/// `max_distance_m` (default 25.0) and count it unmatched.
///
/// Returns Err with an explanatory message when the frame has no geodetic
/// anchor, since matching is then impossible.
pub fn match_by_position(
    block: &Block,
    frame: &LocalFrame,
    images: &[SourceImage],
    max_distance_m: f64,
) -> Result<MatchReport>;

pub struct StageOptions<'a> {
    pub out_dir: &'a Path,      // images/ is created beneath this
    pub jpeg_quality: u8,       // default 95
    pub overwrite: bool,
}

pub struct StageReport {
    pub written: usize,
    pub skipped_existing: usize,
    pub failed: Vec<(PathBuf, String)>,
    /// Mean fraction of output pixels sampling outside the source frame.
    /// Above ~0.15 means the distortion model is probably wrong.
    pub mean_out_of_bounds: f64,
}

/// Undistort every matched original into `out_dir/images/<original name>`,
/// in parallel via rayon, and rewrite `block.photos[i].path` to the new
/// basename. `target` is the XML's pinhole (per photogroup); `source` is the
/// physical camera from the report.
pub fn undistort_into(
    block: &mut Block,
    matches: &MatchReport,
    source: Calibration,
    opts: &StageOptions,
) -> Result<StageReport>;

/// The zero-risk alternative: copy (or hard-link) the originals unchanged and
/// rewrite paths, leaving the caller to write the original distortion model
/// as an OPENCV/FULL_OPENCV camera. No resampling, so nothing can be lost.
pub fn copy_originals_into(
    block: &mut Block,
    matches: &MatchReport,
    opts: &StageOptions,
) -> Result<StageReport>;
```

Undistortion math — this is `initUndistortRectifyMap` with the XML pinhole as
the new camera matrix. For each output pixel `(u, v)` of the undistorted image
(dimensions and `fx, fy, cx, cy` from the photogroup):

```
x = (u - cx) / fx
y = (v - cy) / fy
r2 = x*x + y*y
radial = 1 + k1*r2 + k2*r2^2 + k3*r2^3          (k4 is 0 in practice; if
                                                 non-zero use the rational
                                                 form and say so)
x' = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
y' = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
us = focal * x' + cx_src
vs = focal * y' + cy_src
sample the source with bilinear interpolation at (us, vs)
```

Note the source camera has a single `focal` (square pixels) while the XML
target has `fx != fy`; that is expected and correct — it is Terra's chosen
virtual camera. Pixels whose `(us, vs)` fall outside the source get black and
count toward `mean_out_of_bounds`.

Build the map **once per photogroup** and reuse it: all 1680 photogroups here
are identical, so key the cache on the intrinsics tuple. The map is
5280*3956 entries — store as two `f32` planes (~160 MB) or recompute per row;
either is fine, but do not build 1680 copies.

The `+0.5` corner-origin shift applies to the report's Cx/Cy exactly as
`block.rs` applies it to the XML's principal point. Make it a named constant
with a comment, and cover it in a test.

Required tests:
- Distortion model round-trip: for a grid of normalised points, distorting
  then numerically undistorting returns the original to <1e-6.
- Zero distortion coefficients make the map the identity to <1e-9.
- `calibration_from_report` parses the real Quarry table shape (embed the
  literal rows as a test fixture) and returns `Ok(None)` on a mangled table.
- XMP extraction from a synthetic buffer containing a `drone-dji` packet.
- `match_by_position` on a synthetic 10-pose block with one missing and one
  duplicate source: no collisions, correct unmatched count.
- MRK parsing of the real line shape, including the skipped index.

## `main.rs` and `wizard.rs` (owner: integration agent, after the above)

- `inspect` collapses identical photogroups: group by
  `(width, height, fx, fy, cx, cy, distortion, orientation)`, print each
  distinct camera once with a count ("1680 photogroups, 1 distinct camera").
  1680 repeated blocks of output is the bug being fixed.
- New `stage` subcommand exposing the module above.
- `export` gains `--keep-distortion` (originals + original model) and
  `--images <DIR>` (roots to search for originals).
- `--for <target>` preset: `brush|spirulae|lichtfeld|postshot|nerfstudio|colmap|metashape|realityscan|all`.
- Bare invocation, or invocation with a single directory argument (drag onto
  the exe), enters the wizard.
- Windows console: initialise `console::Term`, strip ANSI when unsupported,
  and pause before exit when the process owns its console.
- Path hygiene on every prompted path: trim whitespace, strip matched single
  or double quotes, strip a trailing directory separator, expand `~`, and
  accept any point inside the project tree by searching upward and downward
  for `AT/BlocksExchange*.xml`.
