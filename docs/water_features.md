# Water Feature Set Report

## Goal

Water v2 is moving loaded LiDAR point clouds toward three active water feature families:

- **Ripples**: composable visual effects on the active/base cloud. Legacy Caustics load as Ripples with overlay type `Caustic Lace`.
- **Flow**: emitter-driven path caches plus generated world-aligned stream surfels.
- **Field**: local corridor or region vector fields that generate Field Streamlines and Field Surface Motion.

The base point cloud remains the visual source of truth. Flow and Field Streamlines may add generated stream overlay sessions. Ripples and Field Surface Motion should modify the active/base cloud's final evaluated visuals through composable effects, without injecting dense permanent scalar fields or relying on newly generated growing point patterns as the primary visual.

Basin and Runoff are removed from the public v2 workflow. Legacy code and tests may still exist for compatibility, but Basin/Runoff tabs, runtime load state, generation, and new project saves are not part of the active water contract. Old Basin/Runoff JSON records are tolerated and ignored.

## Current Status

Implemented in the current repository:

- Active Water tabs for Ripples, Flow, and Field.
- Project schema `24` with v2 Ripple/Flow/Field settings and legacy Caustics-to-Ripples migration.
- Ripple `WaterEffectLayer` records and distinct base-cloud effect fields for all `WaterRippleOverlayType` values.
- Shared region selection for Ripple and Field support, including selected base point indices, edge weights, normals, source scalar values, field vectors, and manual Field control flags.
- Flow path cache reuse, branch hiding, and generated stream surfels with the v2 stream scalar contract.
- Field cache, Field Streamlines, and Field Surface Motion built from Flow path anchors or user-authored Field regions, with region Field caches saved and reused offline.
- Field no-flow, bridge-allowed, and bridge-blocked control regions with visible diagnostics.
- Shared animated trail visualization for Flow and Field streams; Flow moves along baked path anchors, while Field moves along cached vector-field paths seeded from perturbed source points.
- Active-cloud `water_effect_*` composition for Ripples and Field Surface Motion, with Visuals-tab Water Effect Stack controls.
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
- `flowStreamSettings` and `flowStreamOverlay` for generated Flow Streams.
- `rippleLayers` and `rippleEffectOverlay` for current in-memory Ripple effect evaluation and base-cloud composition.
- `fieldSettings`, `fieldStreamSettings`, `fieldCache`, `fieldStreamOverlay`, and `fieldSurfaceEffectOverlay` for Field.
- `activeRegionFeature`, `regionEditor`, and placement flags for editable Ripple regions and legacy-safe region editing.

Generated water overlay sessions are excluded from support-layer discovery. They are renderable water output, not source LiDAR layers for future water bakes. Ripples no longer create active visible `-Ripples.generated` sessions; their sparse effect points are an internal input to composed `water_effect_*` fields on the active/base cloud. Field Surface Motion also contributes to active/base cloud composition, while Flow and Field Streamlines remain generated stream overlay sessions.

## Active UI Contract

The active Water panel tabs are:

```text
Ripples
Flow
Field
```

Removed from the active public workflow:

```text
Basin Haze
Runoff
Trail Shape
Animation Trail Playback
legacy trail particle controls
```

Flow still exposes path baking, branch hiding, source settings, and stream controls. Ripples exposes region/layer controls and procedural overlay settings. Field exposes field build settings, stream settings, surface-motion output controls, and user-authored Field regions.

## Serialization Contract

Project documents now use schema `24`. New saves write the v2 water keys:

```text
water_emitters
water_source_settings
water_path_cache
water_ripple_layers
water_flow_stream_settings
water_field_settings
water_field_stream_settings
```

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

Ripples use `WaterEffectLayer` and sparse in-memory `WaterEffectOverlay` data to compute composable `water_effect_*` contributions on the active/base cloud. The active workflow does not create visible `-Ripples.generated` point-cloud sessions; the base cloud's final visual evaluation is modified through post-base effect fields rather than by replacing the cloud with copied growing point patterns.

Supported overlay types are encoded with `WaterRippleOverlayType`; `Caustic Lace` is the migrated legacy Caustics behavior. Layers include:

- region vertices and derived bounds,
- overlay type and feature type,
- response settings for size, opacity, emission, and colour contribution,
- viewport/export enable flags,
- blend mode and procedural parameters.

The Visuals tab exposes Water Effect Stack controls for matching base-cloud Ripple layers, including add, multiply, max, screen, override, colourise, opacity, size, and emission contributions.

Ripple generation now evaluates containment and edge fade against the clicked polygon boundary. A C-shaped Ripple region excludes the cut-out area rather than falling back to the derived convex hull.

Each Ripple overlay type now produces a distinct sparse effect field:

- `Caustic Lace`: warped cellular ridge lace with bright caustic-like peaks.
- `Linear Ripples`: parallel phase bands along the layer direction.
- `Radial Ripples`: symmetric expanding rings around the region centre.
- `Rain Rings`: seeded local ring impacts across the region.
- `Tide Bands`: broad slow bands for large sand or sheet-water regions.
- `Wet Sheen`: slope-sensitive wet highlights with low-frequency variation.
- `Current Threads`: thin stretched directional streaks.
- `Droplet Glints`: sparse seeded point glints and pulses.
- `Drip Trails`: gravity/normal-guided short streaks for vertical or sparse surfaces.
- `Foam Sparkle`: edge-biased bright pulses and speckles.
- `Salt/Mineral Shimmer`: slow granular residue shimmer.

Changing a complete Ripple layer in the Water tab recomposes the base cloud immediately. Disabling or deleting the last active layer clears stale legacy `-Ripples.generated` sessions, so the user does not need to press `Refresh Ripples` repeatedly during ordinary editing.

## Flow

Flow keeps the existing path bake model:

1. Emitters and source/path settings define bake inputs.
2. `GenerateWaterPathCache` creates or refreshes `WaterPathCache`.
3. `BuildWaterPathAnchorsFromCache` rebuilds visible anchors and applies hidden branch IDs.
4. `BuildFlowStreamOverlayFromPathAnchors` converts path anchors into shared animated trail paths and generates deterministic stream surfels.
5. `BuildWaterStreamOverlayPointCloud` exposes the generated stream as an in-memory point-cloud session.

Path-affecting changes dirty the path cache. Stream visual/settings changes, such as stream count, width, length, spacing, turbulence, and speed, refresh the stream overlay without dirtying the path cache.

Flow Streams replace legacy trail particles as the primary visible water output. The old generated water PLY workflow is no longer required for viewport, EXR, or MP4 water visuals.

Flow stream geometry stays static while shader/offline playback derives animated age from `point_age`, `point_seed`, `stream_speed`, and render time. Opacity, emission, and colour energy can change over time without rebaking paths or regenerating topology.

## Field

Field is built from Flow path anchors or user-authored Field regions:

1. `BuildFieldCacheFromPathAnchors` creates a local corridor-like `WaterFieldCache`.
2. `BuildFieldCacheFromRegions` creates region-local field nodes from selected surface support and Field control regions.
3. `BuildFieldStreamOverlay` integrates source-point paths through the cached vector field, then emits Field Streamlines using the same animated trail schema as Flow.
4. `GenerateFieldSurfaceEffectOverlay` emits a virtual/effect overlay for Field Surface Motion.

Field output should stay surface-bound. When support is weak, streams should bridge only valid gaps and otherwise fade or terminate. Field caches are local to path corridors or selected regions; never build a whole-scene field over the full point cloud.

Field can now build from user-authored regions stored as `WaterEffectLayer` records with `FieldSurfaceMotion` feature type. Ripple and Field region containment use one shared selection helper, so C-shaped regions exclude the cut-out area and selected point metadata is available for field editing and composition.
Field control regions can mark local support as no-flow, bridge-allowed, or bridge-blocked. No-flow support is excluded from Field Streamlines and Field Surface Motion. Bridge-allowed regions can permit a bounded manual bridge over an otherwise over-limit gap, while bridge-blocked regions force a split.
Field streamlines start from accepted water emitters projected into the selected Field support. Each source path receives deterministic seed-based spawn perturbation; if no emitter can seed the field, support points in the selected region seed fallback paths. Field streamlines split across rejected over-limit gaps and use low surface confidence to fade stream opacity/emission through the `stream_confidence` scalar. The generated overlay records accepted bridge, rejected gap, low-confidence fade, hard termination, no-flow, bridge-allowed, and bridge-blocked counters that are shown in the Field panel.
Ripple and Field Surface Motion now also pre-compose `water_effect_*` fields on the active/base cloud; renderers apply those fields after existing base mappings for size, opacity, emission, and colour. The Visuals tab exposes Water Effect Stack contribution controls for the selected base cloud. Saved Ripple and Field-region projects regenerate those fields when the project or target layer loads.

## Stream Surfel Scalar Contract

Generated Flow Streams and Field Streamlines must expose these scalar fields in this order:

```text
stream_id
source_id
path_id
branch_id
stream_seed
point_seed
stream_distance
stream_length
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
```

The renderer consumes `stream_width`, `stream_world_length`, `stream_confidence`, `wetness`, `feature_type`, and tangent fields for world-aligned elongated Gaussian surfels. Do not rename these fields without a coordinated serialization, shader, visual preset, and test update.

## Rendering Contract

Point-cloud styles now have `waterStreamOverlay` for generated stream layers. Old `flowAnimation` / `waterPathView` styles remain parseable aliases for compatibility, but new stream overlays should use the v2 water overlay path.

Stream samples render as world-aligned elongated surfels:

```text
long axis  = tangent * stream_world_length
short axis = cross(normal, tangent) * stream_width
normal     = local surface normal
```

Generated Flow and Field stream layers participate in viewport rendering and the same EXR/MP4 export path as other visible point-cloud sessions. Water streams and effect layers are not exported as PLY unless a future explicit export feature asks for it. Ripples and Field Surface Motion evaluate through active-cloud `water_effect_*` composition.

## Visuals Contract

Base cloud visuals are evaluated first. Ripple and Field Surface Motion contributions then combine with those values through Visuals-compatible `water_effect_*` fields. Generated Flow and Field Streamline overlays keep their own stream scalar fields.

Layer-linked saved visuals should keep field availability honest:

- Base-cloud visuals can use base scalar fields.
- Ripple visuals use base-cloud Water Effect Stack fields.
- Flow visuals can use stream scalar fields.
- Field visuals can use Field stream/effect fields.

When a visual is imported from another layer family, keep it read-only until saved under the active layer with a suffix such as `_baseCloud`, `_ripple`, `_flow`, or `_field`.

The active base-cloud Water Effect Stack supports add, multiply, max, screen, override, colourise, opacity, size, and emission contributions for overlapping Ripple and Field Surface Motion layers while preserving existing base scalar mappings.

## Cache And File Strategy

The current mandatory saved caches are:

```text
<source-stem>-WaterPathCache.json
<source-stem>-WaterFieldCache.bin
```

`WaterFieldCache.bin` is derived output, not a normal project source layer. It stores the support layer path/signature, field settings fingerprint, region fingerprint, field settings, stale flag, selected region boundary, and serialized field-node records. Region caches are reused when fingerprints match and rebuilt when source support, region geometry/settings, or field settings change.

Reserved v2 cache names for expensive future reloads:

```text
<source-stem>-WaterFlowStreamCache.bin
<source-stem>-WaterFieldStreamCache.bin
<source-stem>-WaterEffectLayerCache-<layer-id>.bin
```

Cache metadata should include source layer signature, point count, bounds, normal availability, relevant scalar availability, settings fingerprint, region/path fingerprint, creation time, and cache type.

## Known Gaps

- Manual site-data tuning may still be needed for Field no-flow and bridge thresholds.
- Manual application EXR/MP4 acceptance remains useful as a final operator check, but automated tests now cover active-cloud water-effect EXR writing and MP4 frame conversion.

## Change Checklist

Use these checks after water feature changes:

- Serialization: new saves include Ripple/Flow/Field keys and omit Basin/Runoff/Caustic region keys.
- Legacy load: old Caustic regions become Ripple `Caustic Lace`; old Basin/Runoff records are ignored.
- Flow: path-affecting settings dirty `WaterPathCache`; stream settings only refresh stream overlays.
- Stream schema: generated stream scalar fields match the exact order above.
- Rendering: `waterStreamOverlay` styles compile and render tangent-aligned surfels.
- Visuals: base-cloud scalar mappings remain intact after creating Ripples, Flow Streams, Field Streamlines, and Field Surface Motion; Ripple/Field Surface Motion effects compose through Visuals-compatible contributions instead of replacing base visuals.
- Regions: Ripple and Field regions preserve concave clicked boundaries.
- Motion: Flow and Field Streams visibly animate through shader/Visuals playback, not only static generated positions.
- Field cache: region Field caches save, reload, and invalidate on support, region, or settings changes.
- Export: visible generated Flow/Field stream layers and active-cloud `water_effect_*` fields appear in viewport and camera export paths without requiring water PLY export.
