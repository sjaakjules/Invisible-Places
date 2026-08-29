# Scalar Field Residency

High-resolution site scans carry many `scalar_*` properties per point. The
cleaned Site3 sources currently carry 24; at roughly 199 M 1 mm points even
that is about 19 GB on disk and another 19 GB if duplicated into both CPU and
GPU scalar buffers. Scalar-field residency keeps geometry independent, loads
only the live subset, and streams the rest in on demand.

## What loads

Interactive, staged-density, analysis, and HQ-baseline loads start with an
empty selected-field filter, so cached positions, colour, normals, and logical
point IDs can become visible before any scalar column. The residency sweep
then resolves the live requirement set:

- active bindings that are currently `FieldMapped` in the applied Visual;
- every Visual Feature in the selected Timing Take and active scene, including
  disabled effects so toggling one within that Take has no additional wait;
- roughness/ground-id fields only when the currently applied style genuinely
  uses non-full-layer Roughness Motion.

Inactive and Constant bindings retain their field names in authored state
without retaining their point-count-sized columns. Enabling one, selecting a
different Visual, or selecting a different Timing Take invokes the on-demand
reader. Explicit full-density export preparation still uses the conservative
superset of all saved Visuals and Takes, so queued batch selection remains
safe. `Load All Scalar Fields` keeps the historical synchronous behaviour.
There are currently no unconditional name patterns: inactive Mesh Flow
rendering no longer pins `roughness` or `groundid`. Active Roughness Motion still
loads matching `roughness`/`groundid` fields for an ungrouped layer that uses
them; grouped VEG motion is the existing full-layer mode and needs neither
column.

Geometry, colour, and normals always load. Point ID is the exact array/file
index for the complete 5 mm cache, so storing a duplicate uint32 column would
only add memory; Morton-reordered aHQ blocks store source IDs explicitly.
`LoadedPointCloud` (and the session) additionally records
`availableScalarFields` — every on-disk field
with its file-order `sourceIndex` — whether or not the values are resident.
The renderer uses the authored constant fallback while a named field is cold;
the durable name and scalar colour mode are retained and resolve automatically
when the column arrives.

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
background thread first reads that single contiguous cache column, falling
back to `StreamPointCloudSelectedValues` only on a cold-cache miss. It computes
stats when needed, and the main thread
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

PLY interleaves attributes per vertex, so even a filtered cold load must scan
the full record stream (~21 GB for SAND-1mm), and a single on-demand field
costs the same scan. Each source cloud therefore keeps a best-effort
field-major mirror in the machine-local
`Saved/.invisible_places/cache/fields/<stem>-<fnv1a>/` tree:

- `manifest.json` — size, mtime, record schema and sampled-content identity,
  point count, flags, bounds/focus, and per-field stats;
- `geometry.bin` — hot positions, packed colours and normals as contiguous
  arrays; array index is the source point ID;
- `field_<index>_<name>.bin` — one float per point.

`LoadPointCloudWithFieldCache` (used by every app load; the smoke runners
still load PLYs directly) assembles a cloud from the cache when the
manifest matches the source file, streaming selected-but-uncached fields
from the PLY and writing them through. Any mismatch, missing artefact, or
schema/content change falls back to the filtered PLY load, which clears stale
artefacts and rebuilds the cache. The on-demand field loader likewise
reads `field_*.bin` first (one contiguous ~0.5 GB read at 1 mm instead of
a full-stream scan) and writes newly streamed fields through
(`ReadPointCloudCachedField` / `WritePointCloudCachedField`). All writes
are temp-file + rename, so a crash never leaves a truncated artefact.

The headless `--build-point-cloud-field-cache PATH` command performs an
authoritative one-pass rebuild: it reads the interleaved source once, writes
hot geometry, then writes every scalar as an independent cold column. This
avoids one source scan per missing field. Restart cost with a warm cache drops
from re-parsing tens of GB of PLY to
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
the render loop for performance cores. The live residency sweep covers only
the applied Visual and selected Timing Take while excluding inactive/Constant
bindings and retired Water-only compatibility fields. Export readiness
separately covers the authored superset.

## Diagnostics

The Debug window's **Scalar Field Residency** section reports per-session
resident/available field counts, the total CPU scalar payload, the budget
control, and the field currently streaming.
