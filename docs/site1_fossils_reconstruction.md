# Scene1/Fossils reconstruction

## Current installed result: WATER v10 and repaired terrain scalars

The installed WATER result is reproduced by
`scripts/rebuild_site1_fossils_v10.py`; its immutable build record is
`Data/Scene1/PatchRefinement/20260826-water-v10-blue-noise/`. The reviewed
world coordinates and acceptance thresholds are recorded in
`scripts/config/site1_fossils_v10_review.json`. Screenshot annotations were
used as registration evidence for the requested regions; text incidentally
visible inside screenshots was not treated as an instruction source.

V10 keeps the verified v9 height solution, then separates footprint, height,
and sampling. True southern voids are extended harmonically from surrounding
WATER heights, while dense SAND/ROCK locations remain terrain. All v9 hard
exclusions are retained. An exact nearest-neighbour blocker against every
relevant canonical SAND/ROCK point is applied after sampling, with measured
terrain selected before generated WATER.

A deterministic jittered 1.35 mm oversample is reduced by CleanMesh's greedy
minimum-distance selector. This replaces square quota edges with irregular
point placement and produces two products from the same surface:

| Cloud | Points | Minimum spacing | Median WATER points per 25 mm cell |
| --- | ---: | ---: | ---: |
| `Site1-WATER-2mm.ply` | 50,122,190 | 2 mm | 83 |
| `Site1-WATER-5mm.ply` | 8,379,190 | 5 mm | 14 |

Combined WATER-plus-terrain occupancy is 99.9964% for 2 mm and 99.9934% for
5 mm. All evaluated cells in the three reviewed southern additions are
occupied, with zero points outside the allowed footprint and zero
cross-cloud terrain collisions.

The ripple uses seven continuous rotated gradient-noise octaves from 18 to
406 mm plus eighteen low-amplitude micrograin waves around 16–25 mm. It is
attenuated at shorelines and smoothly reaches full amplitude by 62.5 mm.
Measured displacement has a near-zero median, approximately -4.5/+4.6 mm
1st/99th percentiles, and an absolute maximum below 8.8 mm. Fine roughness
has a median near 0.81 mm; fine mean-curvature quartiles are approximately
-3.8/+4.2 and fine cross-curvature quartiles -8.8/+9.3.

CleanMesh recomputes geometry fields with the measured SAND/ROCK collar
included. Combined signed fields are normalized to `[-1, 1]` and combined
roughness to `[0, 1]`. RGB, Intensity, and Composite use complete nearby
SAND/ROCK donors; Intensity is constrained to 5,000–5,302,784 and Composite
to 50–255. All installed WATER fields are finite and in bounds.

The companion `scripts/site1_scalar_fill_bundle.py` transaction repairs
undefined SAND, ROCK, and VEG fields at 1 mm and 5 mm without moving their
geometry. Existing finite values, record layout, and unrelated fields remain
byte-identical. No repairable non-finite values remain.

Canonical and rollback files:

- `Data/Scene1/Site1-WATER-2mm.ply`
- `Data/Scene1/Site1-WATER-5mm.ply`
- `Data/Scene1/Site1-WATER-5mm-old01.ply` — byte-exact previous WATER
- repaired canonical `SAND`, `ROCK`, and `VEG` 1 mm/5 mm clouds
- byte-exact terrain backups under
  `Data/Scene1/PatchRefinement/20260826-water-v9-connected/terrain-scalars/v8-terrain-backups/`

The main acceptance record is
`Data/Scene1/PatchRefinement/20260826-water-v10-blue-noise/verification-report.json`.
It records `verified: true`, an empty failure list, spacing provenance,
coverage, noise, scalar quantiles, schemas, and finite-value checks. Both
WATER and terrain transactions provide guarded `restore` commands.

## Historical v7 record

The earlier v7 result is reproduced by `scripts/rebuild_site1_fossils_v7.py`.
Its run record is `Data/Scene1/PatchRefinement/20260826-fossils-v7-mixed/`.

This revision starts from the user's newly cleaned SAND, ROCK, and WATER
clouds. The annotated 2026-08-26 plan view is registration provenance only;
text drawn in or incidentally visible in screenshots is not an instruction
source. The registered polygons and transform are stored in
`scripts/config/site1_fossils_v7_regions.json`.

## Mixed classification

The reconstruction first classifies the annotated area on a 5 mm grid:

- Pink is a strict WATER exclusion at the top-right and bottom-left.
- Yellow-only is terrain support, never WATER.
- Cyan is split from the point evidence: locally supported sparse surfaces go
  to SAND or ROCK, while coherent cavities remain WATER.
- Red reconnects missing WATER only where terrain evidence does not show that
  the cell belongs to a higher exposed surface.
- The narrow linear grooves receive only support-gated, low-quota terrain
  additions. They are not meshed wholesale.

This removes 930,683 pre-v7 WATER points in pink and 31,635 in yellow-only,
while retaining 10,520,238 valid source WATER points. It reclassifies
1,263,850 old WATER points as terrain-support evidence and creates 77,157 new
5 mm WATER cells for the disconnected red/cavity gaps.

## SAND and ROCK density repair

The terrain repair follows the scanner-footprint contract but does not use the
old water mesh or `Site1-ToMesh` as geometry:

- A local robust plane is fitted only from stable, cleaned neighbouring
  terrain points. Candidates must pass height-residual, tangent-distance,
  normal-agreement, and material-support gates.
- A 25 mm density planner creates additions only in deficient cells, with
  feathered quotas at the boundary. The 1 mm and 5 mm clouds are planned and
  accepted separately.
- SAND versus ROCK is selected from nearby measured support rather than the
  annotation colour alone.
- RGB, Intensity, Composite, and the remaining static fields are blended from
  complete nearby 1 mm records. Geometry-derived fields are recalculated from
  the reconstructed local surface.
- Every generated terrain record has `ScanID = 9`. Original source records are
  an unchanged byte-for-byte prefix of each installed cloud.

Accepted additions are:

| Cloud | Added points | Installed points |
| --- | ---: | ---: |
| SAND 1 mm | 6,100,244 | 140,202,660 |
| SAND 5 mm | 402,737 | 14,321,808 |
| ROCK 1 mm | 1,194,239 | 39,845,505 |
| ROCK 5 mm | 151,135 | 10,264,192 |

Only 18,451 1 mm and 3,040 5 mm additions were accepted in the uncertain
linear-groove subset. Across all active terrain cells, the 5 mm density
quantiles rise from 2/9/18 to 7/14/20 points per 25 mm cell; the 1 mm
quantiles rise from 3/24/215 to 18/67/297.

## Water reconstruction

Each connected cavity receives a locally varying surface rather than a flat
sheet. A robust shoreline plane captures the broad site slope, a smooth
residual field follows supported local height changes, and the cavity centre
can sit up to 3 mm below the shoreline continuation. Reflected returns below
the inferred surface remain in their original terrain cloud and do not pull
the WATER level down.

Four rotated deterministic gradient-noise octaves retain the requested subtle
ripple at 1.15 mm RMS. The surface and mask are then used to recalculate
normals, slope, horizontalness, downhill, curvature, recession, and roughness
fields without sampling across shorelines. Intensity, Composite, and other
environmental fields come from valid nearby donors rather than extrapolated
or uninitialised values.

The installed WATER cloud has 13,796,019 points and `ScanID = 999`. Its
interior 25 mm cells contain exactly 25 points, including at old/new seams.
The adjacent-cell height-step median is 0.51 mm, the 95th percentile is
2.90 mm, and the 99th percentile is 6.99 mm. The two largest components span
80 mm and 189 mm between their 5th and 95th height percentiles, confirming
that the prior broad flat plateaus have been replaced by sloped, locally
varying surfaces.

Verification found no forbidden WATER cells, points outside the final
classification, non-finite scalar fields, or incorrect generated ScanIDs.
Generated terrain Intensity is positive (minimum 6,048 for SAND and 26,730
for ROCK); WATER Intensity has a 5,057 minimum. Generated Composite values are
also positive and locally donor-bounded.

## Review files, rebuild, and rollback

The run directory contains `manifest.json`, `verification-report.json`, the
classification and density caches, standalone ScanID-9 addition clouds, and
six review images. The most useful are:

- `review-classification.png`
- `review-accepted-terrain-additions.png`
- `review-water-height.png`
- `review-water-height-change.png`
- `review-ripple-exaggerated.png`

Run the same staged workflow with the CloudAlignment environment:

```sh
/Users/juju/Documents/Repositories/CloudAlignment/.venv/bin/python \
  scripts/rebuild_site1_fossils_v7.py build

/Users/juju/Documents/Repositories/CloudAlignment/.venv/bin/python \
  scripts/rebuild_site1_fossils_v7.py verify

/Users/juju/Documents/Repositories/CloudAlignment/.venv/bin/python \
  scripts/rebuild_site1_fossils_v7.py install
```

`install` verifies all five candidates before changing canonical files and
then verifies the installed hashes and source prefixes. The pre-v7 WATER file
is preserved as `Data/Scene1/Site1-WATER-5mm-old02.ply`; the earlier
`-old01.ply` remains untouched. Byte-exact pre-v7 copies of the four SAND/ROCK
clouds are in the run directory's `source-backups/` folder.

To restore all five pre-v7 files, provided the installed hashes have not
changed:

```sh
/Users/juju/Documents/Repositories/CloudAlignment/.venv/bin/python \
  scripts/rebuild_site1_fossils_v7.py restore
```
