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

## If you have never used a terminal

Double-click `terrapin.exe`, or drag the Terra project folder onto it. It asks
a handful of questions in plain language, works out for itself where the
photographs and the point cloud are, and tells you what it is about to do
before it does it. Enter takes the safe answer to everything.

At the end it prints the command your answers came to, so the second time you
can skip the questions if you want to. There is no separate code path behind
the walkthrough — it fills in the same options the flags below set.

The window stays open when it finishes or fails, which console programs
launched from Explorer otherwise do not: Windows destroys the window the
instant the process exits, taking the error message with it.

## Build

```
cargo build --release
```

No system dependencies, no PROJ, no GDAL, no Python, no CUDA. Builds on rustc
1.75+. On Windows the C runtime is linked statically, so the result is a single
file that depends on nothing but Windows itself — copy `terrapin.exe` to
another machine and it runs.

## Use

```
terrapin inspect  <path>                   report what a block contains
terrapin match    <path> --images DIR      pair photographs with poses
terrapin export   <path> --out DIR ...     convert to open formats
terrapin cloud    <dir> --out merged.ply   merge and decimate cloud tiles
terrapin report   <path>                   summarise AT quality
```

`<path>` is anywhere inside the Terra project — the project folder, its `AT`
folder, somewhere down in `models`, or the XML itself. The block is found from
there, and the file actually opened is printed so nothing is substituted
quietly. Passing a folder where a file is expected is the most common way to
start, and on Windows it used to end at `Access is denied (os error 5)`, which
is what opening a directory as a file reports and sounds like something else
entirely.

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

## The images

An `UndistortAT` block references images Terra made inside its own working
folder, under hashed filenames. That folder is temporary, so a project copied
off the machine that produced it arrives without a single one of them. What
survives is the card, and `--images` rebuilds the rest from it:

```
terrapin export "Quarry Test 1" \
    --out ./quarry \
    --format colmap-bin,nerfstudio \
    --images "F:/DCIM/DJI_202607261624_006"
```

Photographs are paired with poses by position, never by filename or by order.
Both would be wrong: the hashes sort arbitrarily, and a card is not
necessarily contiguous — the quarry flight logged 1681 captures and carries
1680 files, because one never made it off the aircraft. Every DJI frame
records its own GPS, and at 23 m between shots against a couple of metres of
GNSS error the nearest camera centre is unambiguous. The pairing is assigned
shortest-distance-first so a photograph is consumed when it is taken; a greedy
nearest-neighbour pass would hand one photograph to two poses and leave a
third with nothing, which loads cleanly and trains to a blur.

Check before committing to it:

```
terrapin match "Quarry Test 1" --images "F:/DCIM/DJI_202607261624_006"
```

> `matched to poses  1680 of 1680`
> `median offset     0.12 m`

A median in the tens of metres means the pairing has slipped rather than that
the GNSS was poor, and is worth stopping for.

**Undistorted, or as shot.** Poses belong to the camera body, not to the lens,
so they are equally valid two ways. By default the originals are undistorted
into Terra's virtual pinhole camera, which is what every splat trainer wants
and what several accept exclusively. The coefficients for that come from
`AT/report/report.md`, since the XML zeroed them. With `--keep-distortion` the
originals are staged untouched and the real lens is written into the camera
model as `OPENCV` or `FULL_OPENCV` instead — nothing is resampled, so nothing
can be lost, and Metashape, RealityScan and COLMAP all prefer it that way.

Rebuilding 1680 twenty-megapixel frames takes a few minutes and about 17 GB.
Each is written beside its target and renamed into place, so an interrupted
run leaves no half-written file that a later pass would mistake for finished
work.

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

## Which way is up

A georeferenced block already knows. An east-north-up frame is Z-up, and so is
a UTM grid, so that is what comes out by default — which is what COLMAP, QGIS
and every photogrammetry tool expect. Viewers are the other camp: glTF,
three.js, Blender and most NeRF tooling put up on +Y, and a Z-up cloud lies on
its side in them.

```
terrapin export "Quarry Test 1" --out ./quarry --up-axis y
```

That rotates the cameras, their orientations and the cloud together, and
records the rotation in `georeference.json` so it can be undone.

Worth being clear about what this is not. Tools working from bare imagery have
to *infer* gravity — from a ground plane, or from the null space of the camera
axes — and those estimates are only as good as their assumptions about how the
scene was flown. Nothing is inferred here. The CRS already placed the block, so
up is known exactly and this is a relabelling of axes rather than a correction.
The quarry block's camera extent is `861 x 1023 x 3.45 m`: three metres of
vertical across a kilometre of survey, because the flight held its altitude.
Guessing gravity from geometry that flat is exactly where estimators go wrong,
and exactly where knowing the CRS costs nothing.

## Colour in the cloud

Terra writes the same cloud several ways and they do not all carry colour.
`terra_ply` holds textured *meshes* — the colour lives in the JPEGs beside
them, not on the vertices — so a cloud loaded from there is uniformly grey.
`terra_las_1_4` carries per-point RGB. Prefer it, and `--cloud` will say so if
what it loaded came out colourless.

Colour is written as the standard `red` / `green` / `blue` uchar properties,
which is what COLMAP, CloudCompare, MeshLab and Open3D read. Some viewers still
need telling: F3D reports `Unknown scalar array: "color"` and falls back to
flat shading, which looks exactly like a cloud with no colour in it. Press `s`
to cycle scalar arrays, or:

```
f3d sparse_pc.ply --coloring-array=RGB --comp=-2
```

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

**Projected blocks stay put.** A block in a UTM zone is recentred by pure
subtraction and nothing else. It is tempting, now that the EPSG code is
understood, to convert it into a true east-north-up frame — but UTM carries a
scale factor of 0.9996 on the central meridian that grows outward, so that
conversion is a non-rigid warp of roughly 40 cm across a kilometre of quarry.
Terra solved the block in grid coordinates; rectifying it afterwards would
quietly degrade the solve while looking like an improvement. The zone is read
and used for georeferencing, for bringing a foreign cloud alongside, and for
matching photographs — never to move the cameras.

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
- Projected blocks are understood only when the CRS is a UTM zone, which
  covers what Terra writes in practice. Anything else is still recentred and
  still exports to every local format, but `geojson` and `kml` stay empty and
  photographs cannot be matched by position, since there is no anchor to
  compare them against.
- Reusing Terra's solve cannot improve on it. A weak block stays weak.

## Licence

MIT OR Apache-2.0.
