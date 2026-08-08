# Scene3 scanner-patch refinement

`scripts/refine_scene3_scanner_patches.py` improves the terrestrial scanner
artifacts seen by the saved `Patch 01` through `Patch 06` cameras. Patch 01-03
cover scanner-base holes; Patch 04 covers the density edge of the high-density
ScanID 9 ROCK scan; Patch 06 covers two SAND scanner footprints. The tool
builds complete candidates first and does not alter a canonical PLY until the
numerical and GUI validations have passed.

## Fixed mapping and ScanID contract

| Saved camera | CleanMesh component | Intended host scan |
| --- | ---: | ---: |
| Patch 01 | 1 | 3 |
| Patch 02 | 3 | 5 |
| Patch 03 | 5 | 0 |

- Existing ScanID 10/11 records inside these component footprints keep their
  geometry, become ScanID 12, and may change only RGB, `Intensity`,
  `Composite`, and `Roughness`.
- ScanID 10/11 points outside the three footprints are copied byte-for-byte.
- Genuinely added records are ScanID 13. They are selected only in measured
  connected density deficits and are appended after the original payload.
- Existing `A_R_*` fields are byte-identical. New points copy the entire
  `A_R_*` bundle from the exact nearest eligible 1 mm point, with a 5 mm hard
  distance limit.
- Patch 04 never edits an existing ScanID 0-9 record and never creates a
  ScanID 12 record. Its accepted transition points are ScanID 13 and it
  operates on ROCK only. A staged descendant can replace only the complete,
  hash-identified ScanID 13 tail from an earlier Patch 04 run; the pre-Patch04
  prefix remains byte-identical and the superseded tail is retained for exact
  rollback.
- Patch 06 is append-only. Both accepted scanner-footprint fills are ScanID 13;
  existing ScanID 0-11 records remain byte-identical, so its ScanID 12 subsets
  are intentionally empty. Each addition copies the complete scalar bundle
  from a measured 1 mm SAND neighbour.

## Matching and sampling method

- A separate model is fitted for each component and source ScanID (10 or 11).
  Robust boundary pairs match the patch to its intended host scan in Oklab,
  `Intensity`, `Composite`, and `Roughness`; a smooth residual field prevents a
  single flat colour correction from erasing the natural scan variation.
- The CleanMesh input is already colour-matched, so added points retain its RGB
  values. Their three editable scalar fields are corrected to the local host,
  and the complete `A_R_*` bundle is copied together from one nearest 1 mm
  donor rather than interpolating fields independently.
- Additions are restricted to connected 20 mm cells with a measured density
  deficit. Patch 01 matches the full harmonic continuation of total density
  from its outer ring because its host scan is also sparse inside the scanner
  footprint. Patch 02 and Patch 03 retain the host-scan cap and 80% reserve to
  avoid filling legitimate occlusions that already look natural on screen.
- Candidate points must first clear every existing point by 0.65 times the LOD
  spacing. A deterministic hashed ordering then applies the same minimum
  Poisson separation between additions while honoring each cell's quota. This
  avoids both world-grid contour bands and visible random clusters.

Patch 04 uses a separate edge model:

- The connected ScanID 9 footprint is measured from the canonical 1 mm ROCK
  cloud. Its boundary near the saved camera target is fitted as a straight
  reference, but the default density field is no longer one continuous fade.
  The reviewed Patch 04 profile maps the two small upper islands and the large
  two-lobed “B” outline from the 5 August camera annotation into fitted-edge
  coordinates. Those areas carry 75-230 mm full-density plateaus; each then
  fades for roughly another 100-320 mm with connected noise and local-planarity
  modulation. The gaps retain only a short baseline taper, breaking the
  perceptual line without producing a hard empty seam.
- The default `sealed` revision measures distance from the actual ScanID 9
  footprint on a 5 mm planning grid, includes empty seam cells, and guarantees
  an irregular 32-60 mm full-density ribbon before retaining the reviewed
  lobes. This removes the faded moat and rectangular saw-tooth visible in the
  close `Patch 04_new` camera. The fitted line is retained only to choose the
  intended side and along-edge extent.
- The measured-contact revision additionally fills the sub-cell seam that a
  5 mm XY planning grid cannot represent. It extracts only the exterior
  ScanID 9 contour (internal rock cavities are filled in the contour mask),
  admits mesh samples from the shared boundary cell, and applies an exact 3D
  source-clearance/Poisson pass. The camera-visible authored span is audited
  against the ScanID 9 nearest-neighbour scale: the reviewed 1 mm candidate
  has a 0.66 mm median and 1.13 mm p95 cell contact distance, with 99.5% of
  boundary cells inside the 1.98 mm acceptance limit. Added bridge samples
  remain ScanID 13; no ScanID 0-9 record is edited.
- Quotas for the sealed profile are true cellwise deficits: each 5 mm cell
  is filled only up to its interpolated target between measured ambient and
  ScanID 9 density. This prevents already-dense blobs from receiving the same
  fixed addition count as sparse cells. The earlier `jagged` profile remains
  available for reproduction and varies an 80-440 mm outer reach around a
  30 mm core.
- `Data/Scene3/LinearNoisePAtchPoints.ply` supplies geometry candidates only.
  Candidates farther than 2 mm from measured 1 mm geometry, or whose normal
  agreement is below 0.75, are rejected so smoothed mesh corners cannot replace
  the measured surface.
- Each accepted point copies normals and the complete 31-field scalar record
  from its nearest measured 1 mm ROCK neighbour. RGB is an inverse-distance
  Oklab blend of up to eight measured neighbours within 15 mm; the mesh-sample
  RGB is deliberately not used.
- The same measured transition is independently quota-sampled for the 1, 2, 3,
  and 5 mm display bundles. Existing payload bytes are copied without
  rewriting them.

Patch 06 uses two geometry sources:

- The colour-matched CleanMesh component supplies the first scanner footprint.
  It must remain within 2 mm of measured 1 mm geometry and pass the normal
  agreement gate before density sampling.
- The second footprint is reconstructed from a tightly cropped measured SAND
  neighbourhood with CleanMesh Poisson reconstruction. Its candidates use a
  stricter 2 mm point-to-plane, 10 mm tangent-distance, and 0.75 normal-dot
  gate. If the reconstruction cannot retain a sufficient measured-supported
  subset, the build fails instead of installing a smoothed disc.
- Both footprints are independently quota-sampled against their surrounding
  total density for every 1, 2, 3, and 5 mm cloud. RGB is blended in Oklab from
  up to eight measured 1 mm neighbours; normals and all 31 non-ScanID scalar
  fields come from the closest eligible 1 mm record.

## Build and validate

Run from the repository root with the Homebrew Python that provides NumPy and
OpenCV:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py build \
  --run-dir Data/Scene3/PatchRefinement/<run-name>

/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py verify \
  --run-dir Data/Scene3/PatchRefinement/<run-name>
```

For a quick, non-installable camera preview, build only the rendered 5 mm LOD:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py build \
  --run-dir Data/Scene3/PatchRefinement/<preview-name> --spacings 5
```

If an already-built complete 1/2/3/5 mm run predates the final sampling rule,
regenerate only its appended points while preserving its corrected original
payload and compact backups:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py refresh-additions \
  --run-dir Data/Scene3/PatchRefinement/<run-name>
```

When an installed run already has satisfactory colour and Patch 02/03 density,
build a minimal Patch 01 descendant instead of rewriting those records. The
descendant copies the installed payload byte-for-byte and appends only new
ScanID 13 samples:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py build-augmentation \
  --base-run-dir Data/Scene3/PatchRefinement/<installed-run> \
  --run-dir Data/Scene3/PatchRefinement/<density-run>
```

Its compact restore returns exactly to the installed base run. Restore the
density run first if the original pre-patch clouds are later restored through
the base run.

Build Patch 04 as a ROCK descendant of the installed Patch 01-03 run. The fine
actual-edge `sealed` profile is the default; `--edge-profile lobed` reproduces
the reviewed v3 profile, `--edge-profile jagged` reproduces v2, and
`--edge-profile linear --blend-width 0.22` reproduces the earlier constant
220 mm taper:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py build-edge-augmentation \
  --base-run-dir Data/Scene3/PatchRefinement/<installed-run> \
  --run-dir Data/Scene3/PatchRefinement/<edge-run>

/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py verify \
  --run-dir Data/Scene3/PatchRefinement/<edge-run>
```

The edge build refuses an uninstalled base, a material absence of the ScanID 9
density step, geometry/source-schema disagreement, modified existing records,
or an incomplete nearest-neighbour scalar transfer.

To stage a revised edge while an earlier Patch 04 run remains installed,
identify that installed run explicitly. Tail replacement can be chained while
retaining the complete superseded tail for byte-exact rollback:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py build-edge-augmentation \
  --base-run-dir Data/Scene3/PatchRefinement/<installed-patch01-03-run> \
  --replace-installed-edge-run Data/Scene3/PatchRefinement/<installed-edge-run> \
  --run-dir Data/Scene3/PatchRefinement/<revised-edge-run>
```

This reads the current canonical PLYs but does not rename or write them. Each
candidate consists of the verified pre-edge prefix plus the new ScanID 13 tail;
the previous tail is stored separately. The validation root therefore renders
the exact eventual merged cloud rather than drawing new points on top of the
currently installed edge.

Build both Patch 06 SAND scanner footprints from an installed base run. The
second reconstruction uses the CleanMesh binaries and measured source crop;
`--resume` reuses a completed reconstruction after an interrupted build:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py build-patch06 \
  --base-run-dir Data/Scene3/PatchRefinement/<installed-run> \
  --run-dir Data/Scene3/PatchRefinement/<patch06-run>

/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py verify \
  --run-dir Data/Scene3/PatchRefinement/<patch06-run>
```

Each density folder contains the full candidate, original ID-10/11 backup
records, their source vertex indices, corrected ID-12 records, and ID-13
additions. `manifest.json` records original/candidate hashes, schemas, counts,
component mappings, and correction/density statistics. The run root carries an
`.invisible_places-ignore` marker so staging PLYs are excluded when the normal
`Data` root is discovered; explicitly using `validation-data` still discovers
the candidate scene.

The build also creates a self-contained discovery root made of symlinks. This
allows candidate ROCK or SAND clouds to be rendered without replacing the
production files:

```sh
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places \
  Data/Scene3/PatchRefinement/<run-name>/validation-data \
  --gui-smoke scene3-patch-boundaries \
  --smoke-project Data/Scene3/PatchRefinement/<run-name>/<validation-project>.json \
  --smoke-output Data/Scene3/PatchRefinement/<run-name>/validation-render
```

The generated filename is `ExhibitionFinal_patch-validation_project.json` for
Patch 04 and `ExhibitionFinal_patch06-validation_project.json` for Patch 06;
the manifest records its absolute path.

The generated validation project preselects the full 1 mm Scene3 candidate for
the changed role (ROCK for Patch 04, SAND for Patch 06) and aliases
`Patch 04_new` or `Patch 06` onto the smoke camera slot. It selects the exact
`Projector-01` visual with a fixed 2 mm world-space diameter. The smoke
reasserts those settings, disables gSplat/water/live effects, clears every
Shoreline instance, asserts the resolved render styles have shoreline waves
disabled, and renders all four saved cameras. Inspect the unannotated PNGs and
contact sheet at 100%. The
Patch 01-03 `_boundary.png` copies show the approximate component perimeter
only as a review aid; those overlays are not included in the unannotated
acceptance images. The aliased Patch 04 slot is always rendered unannotated so
the Patch 04 density edge or Patch 06 footprints can be judged directly.

## Install and rollback

Close Invisible Places before any canonical-file operation. After the
candidate smoke report passes and the images show no coherent circular or
linear edge:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py install \
  --run-dir Data/Scene3/PatchRefinement/<run-name>
```

Installation temporarily renames every full original, swaps all four
candidates into their canonical paths, and runs the same smoke against the
production project. A failed smoke restores every full original immediately.
After success, the temporary full copies are removed and the compact indexed
record backups remain. A tail-replacement run also retains the small
superseded-ScanID13 tail, so restoring it reproduces the previously installed
edge byte-for-byte. Field and water caches are not manually deleted; source
size/mtime invalidation causes them to rebuild naturally.

On macOS, when the GUI smoke must be launched as a separate top-level process,
use the guarded two-phase form. The first command swaps all four files but
retains every complete original; the install remains explicitly unfinished:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py install \
  --run-dir Data/Scene3/PatchRefinement/<run-name> \
  --defer-post-install-smoke

build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places Data \
  --gui-smoke scene3-patch-boundaries \
  --smoke-project Saved/ExhibitionFinal_project.json \
  --smoke-output Data/Scene3/PatchRefinement/<run-name>/post-install-render

/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py finalize-install \
  --run-dir Data/Scene3/PatchRefinement/<run-name> \
  --post-install-smoke-report \
    Data/Scene3/PatchRefinement/<run-name>/post-install-render/scene3-patch-boundaries.json
```

`finalize-install` accepts only a passing patch-boundary report, re-hashes all
four live candidates and all four temporary originals, and only then deletes
the full copies. If the external smoke fails, `rollback-install --run-dir
<run-name>` restores all four originals instead. Starting another install is
refused while this transaction is awaiting either decision.

Once an installed result has been accepted, remove its redundant replacement
clouds, addition subsets, validation aliases, and large render intermediates
while retaining the original replaced records and index maps required for
byte-exact rollback:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py compact \
  --run-dir Data/Scene3/PatchRefinement/<installed-run> \
  --keep-final-pngs
```

Omit `--keep-final-pngs` for an older superseded run. The smoke JSON reports
are retained in either case.

To remove only genuinely new points while retaining the blended replacements:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py remove-additions \
  --run-dir Data/Scene3/PatchRefinement/<run-name>
```

To restore the exact pre-refinement PLY bytes:

```sh
/opt/homebrew/bin/python3 scripts/refine_scene3_scanner_patches.py restore \
  --run-dir Data/Scene3/PatchRefinement/<run-name>
```

`restore` drops the current run's ID-13 records, replaces indexed ID-12 records
with their complete original ID-10/11 records, restores the original header
bytes, and refuses the replacement unless the resulting SHA-256 equals the
manifest. For an edge-tail replacement it instead reattaches only the saved
superseded ID-13 tail, reproducing the immediately preceding installed run.
Operations refuse unknown canonical hashes; there is intentionally no force
mode.
