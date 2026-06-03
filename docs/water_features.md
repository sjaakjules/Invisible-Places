# Water Feature Set Report

## Goal

Water v2 is moving loaded LiDAR point clouds toward three active water feature families:

- **Ripples**: sparse GPU/offline runtime effects on the active/base cloud. Legacy Caustics load as Ripples with overlay type `Caustic Lace`.
- **Flow**: emitter-driven path caches plus generated world-aligned stream surfels.
- **Field**: local corridor or region vector fields that generate Field Streamlines and Field Surface Motion.

The base point cloud remains the visual source of truth. Flow and Field Streamlines may add generated stream overlay sessions. Ripples modify the base cloud through sparse region memberships plus small runtime parameter buffers that the viewport and offline renderer evaluate procedurally. Field Surface Motion currently modifies the base cloud through generated `water_effect_*` fields. Neither path should rely on dense permanent scalar fields or newly generated growing point patterns as the primary visual.

Basin and Runoff are removed from the public v2 workflow. Legacy code and tests may still exist for compatibility, but Basin/Runoff tabs, runtime load state, generation, and new project saves are not part of the active water contract. Old Basin/Runoff JSON records are tolerated and ignored.

## Current Status

Implemented in the current repository:

- Active Water tabs for Ripples, Flow, and Field.
- Project schema `24` with v2 Ripple/Flow/Field settings and legacy Caustics-to-Ripples migration.
- Ripple `WaterEffectLayer` records and distinct shader/offline procedural patterns for all `WaterRippleOverlayType` values.
- Sparse Ripple membership uploads for selected region points, with pattern/response edits updating compact GPU params when region membership has not changed. This supports millisecond-scale live modifications instead of CPU-regenerating dense fields.
- Shared region selection for Ripple and Field support, including selected base point indices, edge weights, normals, source scalar values, field vectors, and manual Field control flags.
- Flow path cache reuse, branch hiding, and generated stream surfels with the v2 stream scalar contract.
- Field cache, Field Streamlines, and Field Surface Motion built from Flow path anchors or user-authored Field regions, with region Field caches saved and reused offline.
- Field no-flow, bridge-allowed, and bridge-blocked control regions with visible diagnostics.
- Shared animated trail visualization for Flow and Field streams; Flow moves along baked path anchors, while Field moves along cached vector-field paths seeded from perturbed source points.
- Active-cloud sparse runtime Ripple evaluation and `water_effect_*` composition for Field Surface Motion, with Visuals-tab Water Effect Stack controls for both families.
- Viewport/offline/export rendering of water output without requiring water PLY export.
- Legacy Basin/Runoff removal from the active public UI and new-save contract.

## Architecture Map

- `src/water/WaterFlow.hpp` and `src/water/WaterFlow.cpp`: v2 water structs, shared region selection, flow path generation, shared stream surfel generation, Field cache persistence/streamline generation, Ripple/Field sparse effect generation, legacy water helpers, and point-cloud conversion.
- `src/app/Application.cpp`: Water panel UI, runtime `WaterWorkflowState`, emitter and region editing, cache/bake orchestration, in-memory overlay sessions, and project/source document wiring.
- `src/serialization/ProjectDocument.hpp` and `src/serialization/ProjectDocument.cpp`: project schema, water source schema, v2 water settings/layers, legacy migration, and path cache persistence.
- `src/renderer/pointcloud/PointCloudPreviewState.*`, `src/renderer/core/VulkanViewportShell.cpp`, and root `shaders/pointcloud_*`: water overlay render mode, stream tangent/width/world-length scalar handling, and stream surfel shading.
- `tests/AssetDiscoveryTests.cpp`: serialization, legacy migration, deterministic generation, cache invalidation, and shader/style contract coverage.

## Runtime Model

`WaterWorkflowState` in `src/app/Application.cpp` owns the live workflow state. The v2-relevant fields are:

- `emitters`, `defaultSourceSettings`, `tempDefaultSourceSettings`, and per-emitter settings for Flow path generation.
- `pathCache`, `pathAnchors`, path revisions, dirty flags, and hidden branch IDs for reusable Flow paths.
- Path/Lanes/Trail profiles plus `flowStreamOverlay` for generated Flow trails. Internal `stream_*` names are retained for renderer/offline compatibility.
- `rippleLayers` plus sparse runtime memberships/params for current Ripple evaluation. `rippleEffectOverlay` is kept as selected-region debug/evidence data, not as a generated visible Ripple layer.
- `fieldSettings`, `fieldStreamSettings`, `fieldCache`, `fieldStreamOverlay`, and `fieldSurfaceEffectOverlay` for Field.
- `activeRegionFeature`, `regionEditor`, and placement flags for editable Ripple regions and legacy-safe region editing.

Generated water overlay sessions are excluded from support-layer discovery and from base-cloud look-dev/export visual selection. They are renderable water output, not source LiDAR layers for future water bakes. Ripples no longer create active visible `-Ripples.generated` sessions; their region membership and procedural params are uploaded to the base-cloud renderer instead. Field Surface Motion contributes to active/base cloud composition through `water_effect_*` fields, while Flow trails and Field Streamlines remain generated overlay sessions.

## Active UI Contract

The active Water panel tabs are:

```text
Ripples
Flow
Field
```

Removed as standalone active public workflow tabs:

```text
Basin Haze
Runoff
Trail Shape
Animation Trail Playback
legacy trail particle controls
```

Flow exposes path baking, branch hiding, source profile assignments, Lanes controls, and Trail styling. Ripples exposes region/layer controls and procedural overlay settings. Field exposes field build settings, stream settings, surface-motion output controls, and user-authored Field regions.

## Serialization Contract

Project documents now use schema `25`. New saves write the v2 water keys:

```text
water_emitters
water_source_settings
water_path_cache
water_path_profiles
water_lane_profiles
water_trail_profiles
water_ripple_layers
water_field_layers
water_flow_stream_settings
water_field_settings
water_field_stream_settings
```

Project saves also preserve `water_animation_trail_settings`, `water_animation_trail_profiles`, and caustic look settings for legacy animation/visual compatibility, even though those are no longer standalone Water tabs.

`water_sources.json` mirrors the active source/layer/settings subset for reusable water setup, including the same Ripple/Flow/Field settings and the current Flow path cache when available.

New saves do not write:

```text
water_basin_regions
water_runoff_regions
water_caustic_regions
```

Legacy loading rules:

- `water_caustic_regions` migrate to `water_ripple_layers` with overlay type `Caustic Lace` when native ripple layers are absent.
- `water_basin_regions` and `water_runoff_regions` are ignored.
- Compatibility caustic look settings may still be parsed/written for old visual data, but caustic region geometry is no longer the active public save contract.
- Existing `water_path_cache` records are preserved when their support/settings fingerprint matches.

## Ripples

Ripples use `WaterEffectLayer` records plus sparse region memberships to evaluate procedural effects directly on the active/base cloud. The active workflow does not create visible `-Ripples.generated` point-cloud sessions and does not upload dense `water_effect_*` or `ripple_*` scalar fields for ordinary Ripple recalculation.

The first region recalculation selects base-cloud points and uploads compact membership and parameter buffers. When only procedural settings or contribution controls change, the viewport can update the parameter buffer without rebuilding membership. This keeps editing responsive at millisecond-scale latency: pattern, colour, opacity, size, emission, speed, phase, and blend changes can be previewed live because the expensive region scan and most CPU-side upload work are skipped when the region has not changed. Offline rendering reconstructs the same sparse memberships/params for export.

Supported overlay types are encoded with `WaterRippleOverlayType`; `Caustic Lace` is the migrated legacy Caustics behavior. Layers include:

- region vertices and derived bounds,
- overlay type and feature type,
- response settings for size, opacity, emission, and colour contribution,
- viewport/export enable flags,
- blend mode and procedural parameters.

The Visuals tab exposes Water Effect Stack controls for matching base-cloud Ripple layers, including add, multiply, max, screen, override, colourise, opacity, size, and emission contributions. For Ripples these controls update sparse runtime parameters; for Field Surface Motion they update composed `water_effect_*` fields.

Ripple generation now evaluates containment and edge fade against the clicked polygon boundary. A C-shaped Ripple region excludes the cut-out area rather than falling back to the derived convex hull.

Each Ripple overlay type now produces a distinct sparse runtime contribution:

- `Caustic Lace`: warped cellular ridge lace with bright caustic-like peaks.
- `Linear Ripples`: parallel phase bands along the layer direction.
- `Radial Ripples`: symmetric expanding rings around the region centre.
- `Rain Rings`: seeded local ring impacts across the region.
- `Shoreline`: calm advancing and receding foam wash for shore-like regions.
- `Wet Sheen`: slope-sensitive wet highlights with low-frequency variation.
- `Current Threads`: thin stretched directional streaks.
- `Droplet Glints`: sparse seeded point glints and pulses.
- `Drip Trails`: gravity/normal-guided short streaks for vertical or sparse surfaces.
- `Foam Sparkle`: edge-biased bright pulses and speckles.
- `Salt/Mineral Shimmer`: slow granular residue shimmer.

Changing a complete Ripple layer region refreshes sparse base-cloud membership. Editing pattern or response values after membership exists can update GPU params directly, so ordinary live tweaks do not regenerate topology, rebuild dense fields, or upload full-cloud scalar arrays. Disabling or deleting the last active layer clears sparse membership and stale legacy `-Ripples.generated` sessions.

## Flow

Flow now exposes three profile-backed setting areas: Path, Lanes, and Trail. Flow keeps the existing path bake model:

1. Emitters and source/path settings define bake inputs.
2. `GenerateWaterPathCache` creates or refreshes `WaterPathCache`.
3. `BuildWaterPathAnchorsFromCache` rebuilds visible anchors and applies hidden branch IDs.
4. `BuildFlowStreamOverlayFromPathAnchors` converts path anchors into shared animated trail paths and generates deterministic trail surfels with legacy `stream_*` scalar names.
5. `BuildWaterStreamOverlayPointCloud` exposes generated trails as in-memory point-cloud sessions grouped by resolved Trail profile.

Path-affecting changes dirty the path cache. Lane changes such as trail count, lane count, coverage width, crossing, turbulence, and speed refresh from existing anchors without dirtying the path cache. Trail geometry changes regenerate generated trail samples when needed; Trail colour, opacity, and emission are owned by Water > Flow, not the Visuals tab.

Flow trails replace legacy trail particles as the primary visible water output. The old generated water PLY workflow is no longer required for viewport, EXR, or MP4 water visuals.

Flow Trail geometry stays static while shader/offline playback derives animated age from `point_age`, `point_seed`, `stream_speed`, and render time. Opacity, emission, and colour energy can change over time without rebaking paths or regenerating topology.

## Field

Field is built from Flow path anchors or user-authored Field regions:

1. `BuildFieldCacheFromPathAnchors` creates a local corridor-like `WaterFieldCache`.
2. `BuildFieldCacheFromRegions` creates region-local field nodes from selected surface support and Field control regions.
3. `BuildFieldStreamOverlay` integrates source-point paths through the cached vector field, then emits Field Streamlines using the same animated trail schema as Flow.
4. `GenerateFieldSurfaceEffectOverlay` emits a virtual/effect overlay for Field Surface Motion.

Field output should stay surface-bound. When support is weak, streams should bridge only valid gaps and otherwise fade or terminate. Field caches are local to path corridors or selected regions; never build a whole-scene field over the full point cloud.

Field can now build from user-authored regions stored as `WaterEffectLayer` records with `FieldSurfaceMotion` feature type. Ripple and Field region containment use one shared selection helper, so C-shaped regions exclude the cut-out area and selected point metadata is available for field editing and composition.
Field control regions can mark local support as no-flow, bridge-allowed, or bridge-blocked. No-flow support is excluded from Field Streamlines and Field Surface Motion. Bridge-allowed regions can permit a bounded manual bridge over an otherwise over-limit gap, while bridge-blocked regions force a split.
Field streamlines start from non-disabled water emitters projected into the selected Field support. Each source path receives deterministic seed-based spawn perturbation; if no emitter can seed the field, support points in the selected region seed fallback paths. Field streamlines split across rejected over-limit gaps and use low surface confidence to fade stream opacity/emission through the `stream_confidence` scalar. The generated overlay records accepted bridge, rejected gap, low-confidence fade, hard termination, no-flow, bridge-allowed, and bridge-blocked counters that are shown in the Field panel.
Field Surface Motion currently pre-composes `water_effect_*` fields on the active/base cloud; renderers apply those fields after existing base mappings for size, opacity, emission, and colour. Saved Ripple and Field-region projects regenerate their runtime outputs when the project or target layer loads.

Field should continue moving toward the Ripple performance pattern where practical: region-bounded support should be reused aggressively, uploads should be limited to selected/cache nodes rather than full-cloud fields, and shader/offline-side procedural evaluation should be preferred for editable visual parameters. Field cache generation can remain CPU-side while region selection, Field Surface Motion, and stream styling should avoid whole-cloud recomputation when only visual or playback parameters change.

## Stream Surfel Scalar Contract

Generated Flow trails and Field Streamlines must expose these scalar fields in this order:

```text
stream_role
stream_id
source_id
path_id
branch_id
stream_seed
point_seed
stream_distance
stream_length
route_start_index
route_point_count
route_length
stream_start_phase
stream_lateral_offset
point_age
stream_age
stream_speed
stream_width
stream_world_length
stream_confidence
wetness
feature_type
tangent_x
tangent_y
tangent_z
stream_lane_index
stream_lane_count
stream_lane_pitch
stream_lane_span
stream_lane_crossing
stream_cross_seed
```

The renderer consumes `stream_role`, route fields, `stream_width`, `stream_world_length`, `stream_confidence`, `wetness`, `feature_type`, tangent fields, and lane fields for animated route-following, lane crossing, and world-aligned elongated Gaussian surfels. Do not rename or reorder these fields without a coordinated serialization, shader, offline renderer, visual preset, and test update.

## Rendering Contract

Point-cloud styles now have `waterStreamOverlay` for generated stream layers. Old `flowAnimation` / `waterPathView` styles remain parseable aliases for compatibility, but new stream overlays should use the v2 water overlay path.

Stream samples render as world-aligned elongated surfels:

```text
long axis  = tangent * stream_world_length
short axis = cross(normal, tangent) * stream_width
normal     = local surface normal
```

Generated Flow and Field stream layers participate in viewport rendering and the same EXR/MP4 export path as other visible point-cloud sessions. Water streams and effect layers are not exported as PLY unless a future explicit export feature asks for it. Ripples evaluate through sparse base-cloud runtime memberships/params in viewport and offline export. Field Surface Motion evaluates through active-cloud `water_effect_*` composition.

## Visuals Contract

Base cloud visuals are evaluated first. Ripple contributions then combine through sparse runtime evaluation, while Field Surface Motion contributions combine through Visuals-compatible `water_effect_*` fields. Generated Flow and Field Streamline overlays keep their own stream scalar fields.

Layer-linked saved visuals should keep field availability honest:

- Base-cloud visuals can use base scalar fields.
- Ripple visuals use base-cloud Water Effect Stack controls backed by sparse runtime params.
- Flow visuals can use stream scalar fields.
- Field visuals can use Field stream/effect fields.

When a visual is imported from another layer family, keep it read-only until saved under the active layer with a suffix such as `_baseCloud`, `_ripple`, `_flow`, or `_field`.

The active base-cloud Water Effect Stack supports add, multiply, max, screen, override, colourise, opacity, size, and emission contributions for overlapping Ripple and Field Surface Motion layers while preserving existing base scalar mappings. Ripple settings should stay parameter-only when membership is current; Field Surface Motion currently updates generated base-cloud composition fields.

## Cache And File Strategy

The current saved/reusable caches are:

```text
<source-stem>-WaterPathCache.json
<source-stem>-WaterFieldCache.bin
```

`WaterPathCache.json` is the saved Flow path cache. `WaterFieldCache.bin` is derived output for user-authored region Field caches, not a normal project source layer. It stores the support layer path/signature, field settings fingerprint, region fingerprint, field settings, stale flag, selected region boundary, and serialized field-node records. Region caches are reused when fingerprints match and rebuilt when source support, region geometry/settings, or field settings change. Path-anchor Field caches are currently rebuilt from Flow path anchors and stamped in memory rather than saved as mandatory binary caches.

Reserved v2 cache names for expensive future reloads:

```text
<source-stem>-WaterFlowStreamCache.bin
<source-stem>-WaterFieldStreamCache.bin
<source-stem>-WaterEffectLayerCache-<layer-id>.bin
```

Cache metadata should include source layer signature, point count, bounds, normal availability, relevant scalar availability, settings fingerprint, region/path fingerprint, creation time, and cache type.

## Known Gaps

- Manual site-data tuning may still be needed for Field no-flow and bridge thresholds.
- Field Surface Motion still uses generated `water_effect_*` scalar fields and can benefit from the Ripple approach: region-bounded sparse membership, shader/offline procedural evaluation, and parameter-only updates for visual edits.
- Path-anchor Field caches are not persisted as `WaterFieldCache.bin`; they are regenerated from the Flow path cache.
- Manual application EXR/MP4 acceptance remains useful as a final operator check, but automated tests now cover active-cloud water-effect EXR writing and MP4 frame conversion.

## Change Checklist

Use these checks after water feature changes:

- Serialization: new saves include Ripple/Flow/Field keys and omit Basin/Runoff/Caustic region keys.
- Legacy load: old Caustic regions become Ripple `Caustic Lace`; old Basin/Runoff records are ignored.
- Flow: path-affecting settings dirty `WaterPathCache`; Lane and Trail refreshes preserve Trail profile/style state.
- Stream schema: generated stream scalar fields match the exact order above.
- Rendering: `waterStreamOverlay` styles compile and render tangent-aligned surfels.
- Visuals: base-cloud scalar mappings remain intact after creating Ripples, Flow trails, Field Streamlines, and Field Surface Motion; generated Flow trail sessions are hidden from base-cloud look-dev/export visual selection.
- Regions: Ripple and Field regions preserve concave clicked boundaries.
- Motion: Flow and Field Streams visibly animate through shader/Visuals playback, not only static generated positions.
- Field cache: region Field caches save, reload, and invalidate on support, region, or settings changes; path-derived Field caches rebuild from Flow path anchors.
- Export: visible generated Flow/Field stream layers, sparse Ripple runtime effects, and active-cloud Field Surface `water_effect_*` fields appear in viewport and camera export paths without requiring water PLY export.
