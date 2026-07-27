# ADR-001: A single-file guided pipeline from DJI Terra to Gaussian-splat trainers

**Status:** Proposed
**Date:** 2026-07-28
**Deciders:** Nicholas + partner

## Context

terrapin 0.1.0 already converts a DJI Terra aerotriangulation block into
COLMAP, nerfstudio, Bundler, PLY, GeoJSON, KML and CSV. It is pure Rust with
no system dependencies and 43 tests. What it is not yet is operable by the
two people who need to run it: both new to the command line, one mid-move
from macOS to Windows. The tool must take a Terra project as it actually
arrives — copied off another operator's machine, temp folders stripped — and
produce a dataset that trains in Brush, spirulae-splat, LichtFeld Studio,
Postshot, nerfstudio/gsplat, or imports into COLMAP, Metashape and
RealityScan, without the operators needing to understand any of the above.

Hard constraints:

- **One file.** A standalone executable. No installer, no Python, no CUDA
  toolkit, no COLMAP install, no PROJ/GDAL. (This is already the project's
  stance; this ADR reaffirms it for everything added below.)
- **Suckless UX.** Plain linear prompts with explanations, not a flag zoo
  and not a full-screen TUI. Every refusal must say what happened, why it
  matters, and what to do next — the existing house style.
- **Error-tolerant.** The software absorbs operator sloppiness (wrong
  folder, pasted quotes, Mac path habits) rather than punishing it.

## Evidence: validation against the real dataset (2026-07-28)

Run against `D:\Papers\Photogrammetry\2026-07-26-1738\Quarry Test 1`
(Matrice 4E wide camera, 1680 photos, quarry site, UTM 56S):

| Stage | Result | Finding |
|---|---|---|
| `inspect` | works | 1680/1680 photos posed; SRS + zeroed distortion correctly detected. But Terra wrote **one photogroup per photo** (all identical), so output is 645 KB of repetition — needs collapsing. |
| `report` | works | Surfaces georeferencing RMSE 0.152 m, RTK status (all SINGLE), and — critically — the **original camera calibration with distortion** (F 3716.60, K1 −0.1047, K2 −0.0056, K3 −0.0144, K4 0, P1/P2) from `report.md`. |
| `export` (cameras) | works | 0.5 s. COLMAP two-line `images.txt` structure and PINHOLE model verified correct. |
| images staging | **gap** | The XML references 1680 undistorted images with hash filenames in Terra's `models\pc\0\.temp\ImageCorrection\undistort` — **that folder was never copied** (it is a temp dir). The hashes are not SHA-1 of the originals' content. The DCIM originals are present and the count matches exactly. |
| cloud from `terra_ply` | **gap** | Tiles are nested one level deeper than the loader scans (`terra_ply\BlockXXXX\BlockXXXX.ply`). Worse, in this Terra version they are **textured mesh tiles** (xyz-only vertices, no color) in a **local ENU frame** anchored at the `SRSOrigin` declared in `metadata.xml` (EPSG:4326 · 150.5919, −34.2547, 453.305). |
| cloud from `terra_las_1_4` | **gap** | LAS tiles are in **geographic degrees** (extent 0.01 × 0.01 × 74.8 "m" = lon × lat × alt). The centroid safety check refused correctly — the message even pointed at `metadata.xml` — but there is no path to *success*. Reading 158 M points took 74.8 s. |
| Windows console | **gap** | Output uses raw ANSI escapes. Windows 10 conhost (this machine) does not enable VT processing by default, so a double-clicked exe prints `←[1m` garbage and its window closes before anyone can read it. |

The headline discovery: one Terra project stores its three products in
**three different coordinate frames** — the AT block in projected UTM 56S,
the mesh tiles in local ENU with a metadata anchor, the LAS cloud in
geographic degrees. Any pipeline that "just points a trainer at it" must
unify them, and the anchor data to do so exists (`metadata.xml`,
`sfm_geo_desc.json`, the block SRS).

## Decision

Keep terrapin a **single statically-linked Rust executable** and grow it
into a guided pipeline:

1. **Zero-argument wizard.** Running `terrapin` bare — including
   double-click or dragging a folder onto the exe — enters a guided mode:
   find the Terra project (search for `BlocksExchange*.xml` from any
   starting point given), run inspect → report gate → export → image
   staging → verify, asking one plain-language question at a time with a
   safe default on Enter. Existing subcommands stay unchanged for
   scripting.
2. **Image staging becomes a first-class stage.** Parse the original
   calibration out of `AT/report/report.md`, match XML poses to DCIM
   originals via EXIF GPS (shot spacing ~23 m vs GPS error ~2 m — the
   nearest-neighbour match is unambiguous; filename timestamps break ties),
   then **regenerate the undistorted images in-process** (Brown–Conrady
   inverse remap, one shared LUT since the photogroups are identical,
   rayon across 12 cores ≈ ~2 min for 1680 × 20 MP; ~17 GB disk). Write
   `images/` under the original `DJI_*.JPG` names and rewrite image names
   in every export to match.
3. **Geodetic unification, still without PROJ.** Implement transverse
   Mercator forward/inverse for the UTM zones (EPSG 326xx/327xx) plus the
   existing ENU/ECEF machinery: mesh tiles via their `metadata.xml` anchor,
   geographic LAS via LLA→UTM→local, and as a bonus GeoJSON/KML become
   available for projected blocks and `georeference.json` gains the
   `sfm_geo_desc.json` GPS anchor.
4. **Cloud loading hardened.** Bounded recursive tile scan; sniff PLY
   content (mesh vs point cloud, color present or not) instead of trusting
   folder names; prefer the colored source and say why; stream-decimate
   per tile instead of holding 158 M points.
5. **Target presets.** `--for brush|spirulae|lichtfeld|postshot|nerfstudio|
   colmap|metashape|realityscan|all` selects formats and undistortion
   policy; the wizard asks "which software will you train in?" and maps to
   the same presets.
6. **Windows correctness.** Enable VT processing (or strip escapes when
   unsupported), pause before the console window closes when launched from
   Explorer, strip quotes/trailing spaces from pasted paths, accept any
   entry point inside the project tree.

## Options Considered

### Option A: Single exe + built-in wizard (chosen)

| Dimension | Assessment |
|-----------|------------|
| Complexity | Medium — undistortion remap and transverse Mercator are bounded, well-specified math with exact tests |
| Cost | Zero runtime deps; ~2 min compute + ~17 GB disk per 1680-photo dataset |
| Scalability | Fine to ~10k photos; rayon already in the tree |
| Team familiarity | High — same codebase, same house style |
| Novice safety | High — one file, one double-click, guarded defaults |

**Pros:** preserves "no system dependencies" stance; wizard and CLI share one code path; drag-and-drop works via argv.
**Cons:** we own the correctness of undistortion and map projection math (mitigated: both have closed-form round-trip tests, and the existing test culture fits).

### Option B: GUI wrapper (egui, or Tauri shell)

| Dimension | Assessment |
|-----------|------------|
| Complexity | High — second UI paradigm to build and keep in sync |
| Cost | egui stays single-file but adds MBs; Tauri needs WebView2 present |
| Scalability | Same engine underneath |
| Team familiarity | Low — new stack |
| Novice safety | High once built, lower while it's half-built |

**Pros:** most approachable surface conceivable.
**Cons:** slowest route to airtight; risk lands on UI plumbing instead of the actual gaps (images, frames). Can be added later on top of the wizard's stage API.

### Option C: Status quo CLI + written runbook

**Pros:** no code. **Cons:** the validation above shows the dataset does not currently go end-to-end for *anyone*, expert or not; a runbook cannot conjure missing undistorted images. Rejected.

### Option D: Orchestrate external tools (COLMAP `image_undistorter`, PDAL/PROJ)

**Pros:** outsources proven math.
**Cons:** requires installing COLMAP/PDAL on Windows — exactly the operator burden this project exists to remove; breaks the one-file constraint; version drift across machines. Rejected. (Note: COLMAP's undistorter also could not help here — it needs the distortion coefficients in the model, which Terra zeroed; recovery from `report.md` is required either way.)

## Trade-off Analysis

**Bake undistortion vs carry distortion.** Two geometrically valid datasets
exist: (a) undistorted images + PINHOLE (what the XML describes — its
virtual camera is fx 3474.5 / fy 3357.7); (b) original DCIM images + the
*original* calibration written as a FULL_OPENCV camera. Poses are shared —
they belong to the camera body, not the lens model. **Default to (a)**:
every listed trainer accepts undistorted-pinhole COLMAP, several accept
nothing else. Offer (b) as `--keep-distortion` for Metashape / RealityScan
/ COLMAP-first workflows, which prefer originals and handle distortion
themselves. This costs one flag, not a fork of the pipeline.

**In-exe image processing vs none.** Regenerating undistorted imagery is
the single feature that turns "works if Terra's temp folder survived" into
"works from what people actually have" — DCIM card + project folder. It is
the largest new surface (JPEG decode/encode via the `image` crate,
EXIF read), but it is also the difference between airtight and not.

**Wizard vs more flags.** Two novice operators will not compose
`--format colmap-bin,nerfstudio --cloud ...\terra_las_1_4 --max-points ...`.
The wizard asks at most five questions, each with a recommended default,
and prints the equivalent command line at the end so the workflow is
learnable and repeatable.

## Consequences

- **Easier:** partner can produce a training-ready folder unaided; every
  downstream tool reads from one canonical layout (`images/` + `sparse/0`
  + `transforms.json` + `sparse_pc.ply` + `georeference.json`); finished
  splats can be placed back into UTM via `georeference.json`.
- **Harder:** the crate takes on image decoding and map projection —
  binary size grows (a few MB) and the test matrix widens (round-trip
  tests for Mercator; golden-pixel tests for the remap).
- **Revisit:** GUI shell (Option B) once the stage API stabilises; `.spz`
  or `.ply`-splat *output* conversion if post-training tooling is wanted;
  code signing if SmartScreen friction becomes real; Terra schema drift
  (`report.md` table shape) as new Terra versions appear.

## Action Items

1. [ ] `cloud.rs`: recursive bounded tile scan; PLY content sniffing (mesh
       vs points, color); prefer colored source with printed rationale;
       stream decimation; read `metadata.xml` `SRSOrigin`/`SRS` anchors.
2. [ ] `geodesy.rs`: transverse Mercator fwd/inv keyed by EPSG 326xx/327xx;
       geographic-LAS detection; consume `sfm_geo_desc.json`; extend
       `georeference.json` with the GPS anchor.
3. [ ] New `stage` module: parse `report.md` calibration table; EXIF-GPS ↔
       pose matching; parallel Brown–Conrady undistortion; dimension and
       count verification gates.
4. [ ] `inspect`: collapse identical photogroups into one summarised entry.
5. [ ] Wizard front-end (dialoguer or inquire): bare-invocation and
       drag-onto-exe entry, path sanitising, per-target presets, prints the
       equivalent command when done.
6. [ ] Windows console: enable VT or degrade to plain text; pause-on-exit
       when launched from Explorer.
7. [ ] Release: `-C target-feature=+crt-static`, record SHA-256, SmartScreen
       note in README.
8. [ ] Re-run this whole validation on `Quarry Test 1` end-to-end, train in
       Brush and spirulae-splat, then a cold-run test: watch the partner use
       it with no coaching and fix every stumble found.

## Interop matrix (as of this ADR)

| Target | Reads | Provided by | Status |
|---|---|---|---|
| Brush | COLMAP or nerfstudio dataset | `colmap-bin/txt`, `nerfstudio` | ready once `images/` staged |
| spirulae-splat | COLMAP or nerfstudio (verified against repo README) | same | ready once `images/` staged |
| LichtFeld Studio | COLMAP dataset | `colmap-bin/txt` | ready once `images/` staged |
| Postshot | COLMAP dataset | `colmap-bin/txt` | ready once `images/` staged |
| nerfstudio / gsplat | `transforms.json` + PLY | `nerfstudio` | ready once `images/` staged |
| COLMAP (GUI/tooling) | sparse model + images | `colmap-bin/txt` | ready once `images/` staged |
| Metashape | originals + reference CSV (omega/phi/kappa) | `csv` (+ `--keep-distortion`) | ready-ish; import script to document |
| RealityScan | Bundler `bundle.out` + image list | `bundler` | ready-ish; needs originals variant |

Every row shares the same blocker — image staging (Decision 2). It is the
critical path.
