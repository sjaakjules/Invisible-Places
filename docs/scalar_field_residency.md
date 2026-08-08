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

- the six bindings that are currently `FieldMapped` in the live style and
  saved point visuals;
- every Visual Feature (colourise/emissive) in every exportable Timing Take,
  including dormant effects that animation can activate later in a take.

Constant bindings retain their field names in authored state without retaining
their point-count-sized columns. Switching one back to `FieldMapped` invokes
the existing on-demand streamer before rendering. All takes and visuals remain
covered so a queued batch can select a different authored snapshot safely.
There are currently no unconditional name patterns: retired Field/Mesh Flow
rendering no longer pins `roughness` or `groundid`, and inactive Ripple
caustic slots do not contribute to the filter. Active Roughness Motion still
loads matching `roughness`/`groundid` fields for an ungrouped layer that uses
them; grouped VEG motion is the existing full-layer mode and needs neither
column.

Geometry, colour, and normals always load. `LoadedPointCloud` (and the
session) additionally records `availableScalarFields` — every on-disk field
with its file-order `sourceIndex` — whether or not the values are resident.
The GUI smoke runners keep calling the unfiltered loader deliberately.

There is no longer a structural 0..15 residency floor. Live and offline
layer snapshots carry an explicit `generatedWaterOverlay` semantic bit, and
only those generated layers may interpret the fixed Water scalar slots
(jitter seed 12 through feature type 15). An ordinary survey cloud can no
longer be misclassified from its field count or `flowAnimation` style flag,
so every disk-backed source field is compactable and evictable.

`Load All Scalar Fields` (Debug window; persisted as
`load_all_scalar_fields`, schema 65) restores the historical
load-everything behaviour on each cloud's next load.

## On-demand streaming

`EnsureRequiredScalarFieldsResident` sweeps GPU-renderable sessions on a
small frame cadence (piggybacked on the layer-load poll used by the live
loop, smokes, and benchmarks). CPU-only canonical analysis sources and
retired display bundles are deliberately excluded: their point material is
never drawn, so populating their visual fields would duplicate both PLY I/O
and memory. Export readiness checks its selected full-density sessions
explicitly. When authored state references a field that is available
on disk but not resident — e.g. the user picks an `(on demand)` entry in a
binding's Field combo or a Visual Feature field catalog entry — one
background thread streams that single field via
`StreamPointCloudSelectedValues`, computes its stats, and the main thread
appends it to the resident matrix. Appends never renumber existing slots,
so name-resolved bindings, resolved colourise slots, and per-slot histogram
caches stay valid; GPU-resident sessions
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

## Parallel PLY parsing

The first (cache-building) parse of a cloud is also parallel: the record
stride is fixed, so `LoadPointCloud` splits the payload into disjoint
vertex ranges, one worker per range, each seeking directly to its byte
range and decoding into the shared destination arrays. Stats, bounds, and
focus samples merge deterministically — the result is bit-identical to a
single-threaded parse (pinned by a unit test), and small clouds stay
single-threaded automatically. The flattened property program also skips
filter-rejected fields entirely and reads float32 properties without the
historical double round-trip. Measured on the 1.3 GB 5 mm SAND cloud
(8.7 M points, warm file cache, -O2): 1298 ms single-threaded versus
457 ms parallel, 372 ms parallel with a three-field filter.

## Render Current View

The Export tab's **Render Current View** button (beside Render Frame
Preview) renders one frame from the viewport camera exactly as posed at
click time — frozen so it survives the full-density load wait — at full
export density and quality, using the current animation position's water
state and a single sample (a still camera has no motion blur). It shares
Render Frame Preview's readiness gate, so required clouds and scalar
fields load first and the render then fires automatically.

## Inactive caustic slots

`caustic_*_field_slot` values persist as file-order indices (the resident
slot of the load-everything era) and keep that meaning on disk.
`MakeSceneRenderStyle` can still translate them to the field's current
resident row via `TranslateCausticFieldSlotsToResident`, preserving project
compatibility if Ripple is reintroduced. Ripple rendering is currently
inactive, so these slots do not request or consume source columns. Sessions without an
`availableScalarFields` catalog (runtime-generated overlays) skip the
translation because file order and resident order are identical there.

## Exports

`EnsureFullDensityExportSourcesReady` refuses to start until every export
session's required fields are resident, starting the streams itself while
it waits, so frozen exports render with exactly the field data the preview
resolves.

## Budget and eviction

The residency sweep stamps each GPU-renderable session's resident field with
the sweep tick whenever the required-field set still references it. Fields
held only by CPU analysis/retired bundles are not pinned by visual state and
remain evictable. With a non-zero
**Budget (GB)** (Debug window; persisted as `scalar_field_budget_gb`), a
sweep whose combined CPU+GPU scalar payload exceeds the budget evicts the
least-recently-referenced disk-backed fields — never required fields,
never runtime-generated ones — through the same compact-and-replace path
the water systems use, one session's batch per sweep to bound the stall.
Evicted fields stream back on demand (from the field cache when warm) the
moment something references them again, so toggling an effect off and on
costs nothing while the budget has headroom, and a sub-second column read
otherwise. Slot-keyed histogram caches for the compacted session are
dropped because eviction renumbers surviving slots. Eviction defers, like
field appends, while the colourise histogram worker or a Flow trail build
holds the resident cloud.

Field streams run at utility QoS on macOS so backfill never competes with
the render loop for performance cores. The residency sweep covers every
mapped exportable visual/take while excluding dormant Constant bindings and
retired Water-only compatibility fields.

## Diagnostics

The Debug window's **Scalar Field Residency** section reports per-session
resident/available field counts, the total CPU scalar payload, the budget
control, and the field currently streaming.
