# terrapin

Extract DJI Terra aerotriangulation blocks into open, portable formats.

Terra's `AT` folder holds a solved photogrammetric block in Bentley's
BlocksExchange XML, alongside several proprietary binaries (`sfm.dmv`,
`adj/0.bin`, `wallis_paramters.wls`) that have no public specification. The XML
is the only part that matters downstream, and it carries everything needed to
reuse the solve elsewhere: intrinsics, distortion, and per-photo world-to-camera
pose.

`terrapin` turns that into COLMAP, nerfstudio, Bundler, PLY, GeoJSON, KML or
CSV — so the alignment can drive a Gaussian-splat trainer, a GIS, or a mesher,
without a Terra licence and without re-running structure from motion.

## Build

```
cargo build --release
```

No system dependencies, no PROJ, no GDAL. Builds on rustc 1.75+.

## Use

```
terrapin inspect  <xml>                    report what a block contains
terrapin export   <xml> --out DIR ...      convert to open formats
terrapin cloud    <dir> --out merged.ply   merge and decimate cloud tiles
terrapin report   <at-dir>                 summarise AT quality
```

Look before you leap. `inspect` tells you the one thing that decides the rest
of the workflow — whether the block's distortion coefficients are zeroed:

```
terrapin inspect "AT/BlocksExchangeUndistortAT_WithoutTiePoints.xml"
```

> `→ distortion is zeroed: this block expects UNDISTORTED images`

means Terra undistorted internally and you must feed matching undistorted
imagery. Distortion *present* means use the original JPGs as shot. Getting this
backwards produces a splat that bows at the frame edges — subtly enough that
you will not notice until training is well underway.

Then convert:

```
terrapin export "AT/BlocksExchangeUndistortAT_WithoutTiePoints.xml" \
    --out ./quarry \
    --format colmap-txt,nerfstudio,geojson \
    --cloud "models/pc/0/terra_ply" \
    --max-points 300000
```

Place the images in `./quarry/images` and point a trainer at `./quarry`.

## Formats

| `--format`   | Writes                                   | Consumed by |
|--------------|------------------------------------------|-------------|
| `colmap-txt` | `sparse/0/{cameras,images,points3D}.txt` | gsplat, nerfstudio, Brush, LichtFeld Studio, OpenSplat, Postshot |
| `colmap-bin` | `sparse/0/{cameras,images,points3D}.bin` | the same, faster to load |
| `nerfstudio` | `transforms.json`, `sparse_pc.ply`       | nerfstudio, instant-ngp |
| `bundler`    | `bundle.out`, `list.txt`                 | RealityScan, older MVS tools |
| `ply`        | `cameras.ply`, `sparse_pc.ply`           | CloudCompare, MeshLab, Blender |
| `geojson`    | `cameras.geojson`                        | QGIS, ArcGIS Pro |
| `kml`        | `cameras.kml`                            | Google Earth |
| `csv`        | `cameras.csv`                            | anything; local + geodetic + omega/phi/kappa |

Every run also writes `georeference.json`, recording the local frame's origin.
Keep it: that is what puts a finished splat back into real-world coordinates,
and it is not recoverable afterwards.

## The conventions this gets right

These are the things that make hand-rolled converters produce models which load
without complaint and then train into noise.

**Rotation sense.** BlocksExchange `<Rotation>` is world-to-camera and
`<Center>` is the camera position in world coordinates, so projection is
`x_cam = R (X − C)`. COLMAP stores the same sense, with `t = −R C`. nerfstudio
and Bundler both want camera-to-world, and both look down −Z with Y up rather
than +Z with Y down, so each needs a transpose *and* an axis flip — in that
order.

**Principal point origin.** BlocksExchange measures from the *centre* of the
upper-left pixel; COLMAP, OpenCV and nerfstudio measure from its *corner*. The
half-pixel shift is applied once, at parse time. Skipping it costs a uniform
0.707 px reprojection bias across the whole block, which is small enough to
look like ordinary noise and large enough to soften fine detail.

**Coordinate frame.** Terra writes the block in whatever the project CRS was:
geocentric ECEF, geographic degrees, or a projected metric system. None are
directly usable. ECEF coordinates run to 6.4 × 10⁶ m, so a scene a few hundred
metres across occupies the last few significant digits of an f32 and visibly
quantises in any trainer that narrows. Geographic degrees are not metric at
all. `terrapin` detects which you have — from the declared SRS, falling back to
the magnitude of the coordinates, because Terra does not always write a usable
SRS string — and builds a local east-north-up frame. Camera *rotations* are
corrected too, not just positions: rotating the world under a fixed camera
changes that camera's orientation within it.

**Camera axes.** If a photogroup declares `XRightYUp` instead of the usual
`XRightYDown`, poses are reoriented on export rather than silently mirrored.

**Cloud frame consistency.** Terra's point-cloud tiles may carry their own
coordinate offset unrelated to the AT block. Seeding a trainer with a cloud
that sits a thousand kilometres from the cameras trains to noise, slowly, with
no error message. `terrapin` compares centroids after transforming and refuses
by default, with `--allow-disjoint-cloud` to override. A local-frame cloud
checked against an ECEF block shows an offset of almost exactly one earth
radius, which is a useful thing to recognise.

## `terrapin report`

`sfm_report.json` is undocumented and its schema has changed between Terra
versions, so `report` does not model it. It flattens the document and surfaces
numeric leaves whose key names suggest quality metrics. A schema change costs
you a nicely-labelled summary rather than a crash.

It also summarises `POS_residual_of_camera.csv`. Since the column layout is
undocumented, it picks the numeric column with the widest spread while skipping
monotonic index columns.

Read this before committing GPU hours. If a meaningful fraction of the block
failed to register, or reprojection RMS is above a pixel, no downstream tool
will rescue it — re-align instead.

## Tests

```
cargo test
```

43 tests. The load-bearing ones are `frame::reprojection_survives_the_frame_change`,
which asserts a world point lands on the same pixel before and after the change
of frame, and `math::quat_roundtrip_is_exact`, which exercises every branch of
the quaternion conversion including the 180° cases where the naive formula
divides by zero.

## Limits

- Only the `AT` block is read. Meshes (B3DM, OSGB) and rasters are untouched;
  use `3d-tiles-tools` and GDAL for those.
- Projected blocks are recentred but not reprojected — the EPSG code is not
  interpreted, so `geojson` and `kml` are empty for them. Use `gdaltransform`
  or QGIS if you need those from a projected block.
- Reusing Terra's solve cannot improve on it. A weak block stays weak.

## Licence

MIT OR Apache-2.0.
