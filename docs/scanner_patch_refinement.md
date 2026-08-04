# Scene3 scanner-patch refinement

`scripts/refine_scene3_scanner_patches.py` improves the terrestrial scanner
artifacts seen by the saved `Patch 01` through `Patch 04` cameras. Patch 01-03
cover scanner-base holes; Patch 04 covers the straight density edge of the
high-density ScanID 9 ROCK scan. The tool builds complete candidates first and
does not alter a canonical PLY until the numerical and GUI validations have
passed.

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
  cloud. Its boundary near the saved camera target is fitted as a straight line,
  but the default fade is not parallel to that line. A deterministic
  three-octave smooth-value field varies its reach from 80 to 440 mm along the
  edge, while a sign-independent local-normal planarity field expands connected
  flat areas by up to 40 mm. A 30 mm full-density core protects the original
  edge and each variable-width lobe ends with a cubic smoothstep.
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

Build Patch 04 as a ROCK descendant of the installed Patch 01-03 run. The
connected jagged profile is the default; `--edge-profile linear --blend-width
0.22` reproduces the earlier constant 220 mm taper:

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

To stage a revised edge while an earlier append-only Patch 04 run remains
installed, identify that installed run explicitly:

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
  --smoke-project Data/Scene3/PatchRefinement/<run-name>/ExhibitionFinal_patch-validation_project.json \
  --smoke-output Data/Scene3/PatchRefinement/<run-name>/validation-render
```

The smoke forces the full 1 mm Scene3 bundle and exact `Projector-01` visual,
overrides its point footprint to a fixed 2 mm world-space diameter, disables
gSplat/water/live effects, clears every Shoreline instance, asserts the
resolved render styles have shoreline waves disabled, and renders all four
saved cameras. Inspect the unannotated PNGs and contact sheet at 100%. The
Patch 01-03 `_boundary.png` copies show the approximate component perimeter
only as a review aid; those overlays are not included in the unannotated
acceptance images. Patch 04 is always rendered unannotated so the density edge
can be judged directly.

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
