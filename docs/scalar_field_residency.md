# Scalar Field Residency

High-resolution site scans carry ~32 `scalar_*` properties per point. At 1 mm
density (~195 M points across SAND/ROCK/VEG) the scalar payload alone is
~128 bytes per point in CPU memory and the same again in the GPU scalar
buffer — roughly 50 GB of the app's footprint — while a typical project
references only 8–12 of those fields. Scalar-field residency loads the
referenced subset and streams the rest in on demand.

## What loads

Every point-cloud load resolves a `PointCloudScalarFieldFilter`
(`CollectScalarFieldLoadFilter`) against the session's authored state at the
moment the load starts:

- the six field-mapped render parameter bindings of the live style and every
  saved point visual (a binding contributes its field even while parked on
  Constant);
- every Visual Feature (colourise/emissive) of every Timing Take, enabled or
  dormant — takes can be selected at any time, and the same field set ships
  in every density variant and role;
- caustic mask/edge/seed assignments, whitelisted by their persisted
  file-order slot;
- the always-resident patterns (`roughness`, `groundid`) the renderer
  resolves by name on every cloud.

Geometry, colour, and normals always load. `LoadedPointCloud` (and the
session) additionally records `availableScalarFields` — every on-disk field
with its file-order `sourceIndex` — whether or not the values are resident.
The GUI smoke runners keep calling the unfiltered loader deliberately.

`Load All Scalar Fields` (Debug window; persisted as
`load_all_scalar_fields`, schema 65) restores the historical
load-everything behaviour on each cloud's next load.

## On-demand streaming

`EnsureRequiredScalarFieldsResident` sweeps sessions on a small frame
cadence (piggybacked on the layer-load poll used by the live loop, smokes,
and benchmarks). When authored state references a field that is available
on disk but not resident — e.g. the user picks an `(on demand)` entry in a
binding's Field combo or a Visual Feature field catalog entry — one
background thread streams that single field via
`StreamPointCloudSelectedValues`, computes its stats, and the main thread
appends it to the resident matrix. Appends never renumber existing slots,
so name-resolved bindings, resolved colourise slots, per-slot histogram
caches, and translated caustic slots all stay valid; GPU-resident sessions
then replace the scalar buffer through the existing
`UploadPointCloudScalarFields` path. Streams never start while a
whole-cloud load, an offline export, or the shared-cache high-memory slot
is active, and at most one field streams at a time.

The Visual Features histogram/bounds editor needs no special handling: its
value visitor already falls back to streaming from the source path whenever
a selector's field is not resident.

## Field-major sidecar cache

PLY interleaves attributes per vertex, so even a filtered load must scan
the full record stream (~21 GB for SAND-1mm), and a single on-demand field
costs the same scan. Each source cloud therefore keeps a best-effort
field-major mirror beside it under
`<dir>/.invisible_places/cache/fields/<stem>-<fnv1a>/`:

- `manifest.json` — source identity (size + mtime), point count, flags,
  bounds/focus, and per-field stats for every field ever materialised;
- `geometry.bin` — positions, packed colours, normals as contiguous
  arrays;
- `field_<index>_<name>.bin` — one float per point.

`LoadPointCloudWithFieldCache` (used by every app load; the smoke runners
still load PLYs directly) assembles a cloud from the cache when the
manifest matches the source file, streaming selected-but-uncached fields
from the PLY and writing them through. Any mismatch, missing artefact, or
schema change falls back to the filtered PLY load, which clears stale
artefacts and rebuilds the cache. The on-demand field loader likewise
reads `field_*.bin` first (one contiguous ~0.5 GB read at 1 mm instead of
a full-stream scan) and writes newly streamed fields through
(`ReadPointCloudCachedField` / `WritePointCloudCachedField`). All writes
are temp-file + rename, so a crash never leaves a truncated artefact.

Restart cost with a warm cache drops from re-parsing tens of GB of PLY to
reading ~28 bytes/point of geometry plus 4 bytes/point per referenced
field — and recently written cache pages are typically still in the OS
page cache, making the reload largely I/O-free.

## Caustic slots

`caustic_*_field_slot` values persist as file-order indices (the resident
slot of the load-everything era) and keep that meaning on disk.
`MakeSceneRenderStyle` translates them to the field's current resident row
(or -1 until the field arrives) for every consumer — live rendering, the
redraw predicate, offline snapshots, and animation export — via
`TranslateCausticFieldSlotsToResident`. Sessions without an
`availableScalarFields` catalog (runtime-generated overlays) skip the
translation because file order and resident order are identical there.

## Exports

`EnsureFullDensityExportSourcesReady` refuses to start until every export
session's required fields are resident, starting the streams itself while
it waits, so frozen exports render with exactly the field data the preview
resolves.

## Diagnostics

The Debug window's **Scalar Field Residency** section reports per-session
resident/available field counts, the total CPU scalar payload, and the
field currently streaming.
