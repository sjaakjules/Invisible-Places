# Water Feature Set Report

## Goal

The water feature set turns loaded LiDAR point clouds into editable water-related overlays and render effects. It lets users place flow emitters, draw basin/runoff/caustic regions, bake or refresh feature output, save reusable source/cache data, and tune how water appears in the viewport and exports.

The set currently contains four feature families:

- Flow: emitter-driven downhill water paths, branch caches, animated trail particles, trail lanes, path-view debugging, and branch hiding.
- Basin: polygon-driven basin haze/steam overlays.
- Runoff: polygon-driven dew/light-rain trickle overlays.
- Caustics: polygon-driven selection masks on target LiDAR layers, rendered through caustic shader parameters rather than water overlay PLY files.

This document describes the implementation as it exists now. It is meant to be edited alongside future changes so another agent or engineer can understand where to make changes and which saved artifacts are affected.

## Architecture Map

The water feature set is implemented across a few central areas:

- `src/water/WaterFlow.hpp` and `src/water/WaterFlow.cpp`: shared water data types, flow path generation, trail expansion, basin haze, runoff, caustic masks, overlay point-cloud conversion, and PLY writing.
- `src/app/Application.cpp`: Water panel UI, runtime state, emitter placement, region editing, bake/refresh orchestration, overlay sessions, path branch picking, caustic mask uploads, and visual preset application.
- `src/serialization/ProjectDocument.hpp` and `src/serialization/ProjectDocument.cpp`: project JSON, water sources JSON, animation overrides, point visual styles, caustic look settings, region persistence, and path cache persistence.
- `src/renderer/pointcloud/PointCloudPreviewState.*`, `src/renderer/core/VulkanViewportShell.cpp`, `shaders/pointcloud_fast_basic.frag`, and `shaders/pointcloud_caustics.glsl`: point-style flags, uniform packing, flow role filtering, trail fading, and animated caustic rendering.

The Water UI is one tab with subpanels for Flow, Basin, Runoff, and Caustics. Runtime state is held in `WaterWorkflowState` in `src/app/Application.cpp`; persistent state is split across the project document, `water_sources.json`, generated PLY files, and per-support-layer cache files.

## Shared Runtime Model

`WaterWorkflowState` is the live app-side state for all water features. Important fields include:

- `emitters`: sources for Flow.
- `basinRegions`, `runoffRegions`, `causticRegions`: polygonal feature regions.
- `defaultSourceSettings`, `tempDefaultSourceSettings`, and per-emitter settings: Flow routing/trail shape inputs.
- `defaultAnimationTrailSettings`, animation trail profiles, and temporary animation overrides: particle playback inputs shared by Flow and Runoff, and indirectly by Basin haze generation.
- `defaultCausticLookSettings` and temporary/animation caustic look settings: shader look inputs for Caustics.
- `defaultPointVisualStyle`, `pointVisuals`, and selected point visual name: generated Flow visual style.
- `pathCache`, `pathAnchors`, `flowOverlay`, `trailSurfaceIndex`, and path dirty flags: Flow cache and overlay lifecycle.
- `activeRegionFeature`, `regionEditor`, and placement flags: shared region drawing and viewport interaction state.
- `lastOverlayPath`, `lastBasinOverlayPath`, and `lastRunoffOverlayPath`: last written/generated overlay artifacts.

The active support layer is resolved by `ResolveWaterSupportSessionIndex`. It must be a visible loaded point-cloud layer with CPU data available. Generated water overlay sessions are not treated as support layers.

## Shared Region Editing

Basin, Runoff, and Caustics use the same polygon editing machinery in `src/app/Application.cpp`.

Shared concepts:

- `WaterRegionFeature`: `None`, `Basin`, `Runoff`, or `Caustic`.
- `WaterRegionVertexRef`: identifies a feature, region index, and vertex index.
- `WaterRegionEditorState`: stores hovered/dragged vertices and whether viewport input was consumed.
- Viewport clicks add vertices when the relevant placement mode is armed.
- Vertices can be dragged, snapped to LiDAR surface points, merged when overlapping, and removed from the selected region panel.
- Each region type has a derived-value refresh function in `src/water/WaterFlow.cpp` that recomputes hulls and clamps settings.

Derived refresh functions:

- `RefreshWaterBasinRegionDerivedValues`: builds the hull, computes `baseZ`, clamps `heightAbove`, `depthBelow`, and `density`, and clears invalid outlet edge indices.
- `RefreshWaterRunoffRegionDerivedValues`: builds the hull and clamps ground voxel, high-point fraction, density, path length, and max steps.
- `RefreshWaterCausticRegionDerivedValues`: builds the hull, clamps mask/look selection parameters, and marks the mask dirty/stale.

All three region types are saved in both the project document and `water_sources.json`.

## Shared Overlay Point Model

`WaterOverlayPoint` is the common payload for Flow, Basin, and Runoff overlay points. It stores position, normal, RGB, path identity, animation fields, quality fields, feature identity, and guide/particle metadata.

Important scalar concepts:

- `flowId`: route/path/region-generated stream identifier.
- `emitterId`: Flow source ID, or region-derived ID for feature overlays.
- `pathDistance`: distance along the path.
- `phase`, `speed`, `jitterSeed`, `trailAge`, `trailLength`: animation inputs.
- `confidence`, `accumulation`, `pooling`, `surfaceSteepness`: style and diagnostics inputs.
- `particleRole`: role used by Flow visualization and shaders.
- `featureType`: distinguishes main Flow from Basin/Runoff and branch subtypes.
- `regionId`: basin/runoff/caustic-style region identifier when relevant.

Current Flow particle roles:

- `0.0`: path anchor/main sampled path point.
- `1.0`: visible animated particle or ghost sample.
- `2.0`: path-view guide point.
- `3.0`: path-view trail lane guide point.

These role values are consumed by `shaders/pointcloud_fast_basic.frag` and by path debug/picking code. Change them only with a coordinated update across generation, style presets, shader filtering, and debug overlay logic.

## Shared Saved Artifacts

The water system saves several distinct artifact classes.

### Source And Region State

`SaveWaterSources` writes `water_sources.json`:

- If no project path is active: `Saved/water_sources.json`.
- If a project path is active: `<project-directory>/water_sources.json`.

This file stores:

- Default water source settings and temporary source settings.
- Default caustic look settings and temporary caustic look settings.
- Flow emitters.
- Basin, runoff, and caustic regions.
- Current Flow path cache when available and non-empty.

### Project State

The main project document, usually `Saved/invisible_places_project.json`, can store:

- Flow emitters and all water regions.
- Source settings, temporary source settings, old/legacy water settings, and bake/render compatibility fields.
- Water animation trail settings, profiles, and animation overrides.
- Water point visuals and selected water visual.
- Caustic look settings and temporary/animation overrides.
- Current Flow path cache when loaded.

Animation path JSON can also store water trail, water point visual, caustic look, and legacy water override data.

### Generated Overlay Files

Generated Flow/Basin/Runoff overlay files are written under the water directory:

- If no project path is active: `Saved/water/`.
- If a project path is active: `<project-directory>/water/`.

Current generated file names:

- Flow overlay: `<source-stem>-WaterFlow.ply`.
- Flow cache: `<source-stem>-WaterPathCache.json`.
- Basin haze overlay: `<source-stem>-BasinHaze.ply`.
- Runoff overlay: `<source-stem>-Runoff.ply`.

Caustics are different: they generate scalar fields on the target LiDAR layer and upload those fields to the GPU. They do not write a water overlay PLY.

## Feature 1: Flow

Flow is the largest feature. It starts from point-cloud support geometry and user emitters, bakes reusable downhill branch paths, then expands those paths into animated overlay particles and guide geometry.

### Flow Data Model

Core Flow types live in `src/water/WaterFlow.hpp`:

- `WaterEmitter`: source position, radius, strength, speed, status, origin, confidence, and source setting assignment.
- `WaterPathGenerationSettings`: support voxel size, max bridge distance, path length, sample spacing, branching, coverage, gap tolerance, max steps, support sample cap, smoothing, auto tune, and scale mode.
- `WaterParticleTrailShapeSettings`: particle jitter, spline anchor spacing, trail lane count, trail looseness, trail smoothness, and several persisted legacy shape controls.
- `WaterAnimationTrailSettings`: particle density, speed, color variation, trail length, and optional trail sample spacing.
- `WaterPathCache`: reusable branch cache with support signatures, settings fingerprint, requested/tuned settings, diagnostics, branches, hidden branch IDs, and stale state.
- `WaterPathBranch`: branch identity, role, termination reason, confidence, length, flatness, gap count, and raw anchors.

Emitter settings can be default, custom, temporary, or linked from another emitter. `ResolveWaterSourceSettings` chooses the active settings for an emitter.

### Flow UI And Bake Lifecycle

Flow controls are in `DrawWaterPanel` in `src/app/Application.cpp`.

Typical lifecycle:

1. User selects a loaded LiDAR support layer.
2. User places, moves, suggests, accepts, disables, or edits emitters.
3. User edits source settings, path generation settings, trail shape, animation trail playback, and point visual style.
4. `BakeWaterOverlayForActiveLayer` runs from the `Bake Path` button.
5. The app tries to reuse `WaterPathCache` through `TryLoadWaterPathCacheForSupport`.
6. If no valid cache exists, `GenerateWaterPathCache` computes branches.
7. `BuildWaterPathAnchorsFromCache` rebuilds smoothed path anchors and applies hidden-branch edits.
8. `BuildWaterOverlayFromPathAnchors` expands anchors into guide points, lane guides, particles, and ghost samples.
9. `WriteWaterOverlayPly` writes `<source-stem>-WaterFlow.ply`.
10. `AddOrRefreshWaterFlowOverlaySession` creates or updates the generated water overlay point-cloud session.
11. `SaveWaterPathCacheForSupport` writes `<source-stem>-WaterPathCache.json`.

Settings that affect path routing should mark the path dirty and require a bake. Settings that affect only trail shape, trail playback, or visual style should refresh from cached anchors when possible.

### Flow Cache Reuse

The Flow cache is reusable only when support and settings match. `WaterPathCacheMatchesSupportAndSettings` checks:

- Normalized support layer path.
- Support signature: source path, point count, normal availability, and bounds.
- Emitter/settings fingerprint: default bake settings plus every emitter's status, position, radius, strength, speed, assignment, and resolved bake settings.

If a loaded cache does not match, it is marked stale and `pathDirty` is set. Stale caches can still be inspected, but they should not be treated as current output.

### Flow Path Algorithm

Core functions:

- `TuneWaterPathSettings`
- `BuildSupportGraph`
- `FlowDirection`
- `RankDownhillNeighbours`
- `TraceWaterPathBranch`
- `GenerateWaterPathCache`

Auto tune:

- Estimates point spacing from the support cloud.
- Uses `legacyScaleMode` to choose working spacing behavior.
- Adjusts support voxel size, max bridge distance, path sample spacing, and max steps.
- Records diagnostics shown in the UI.

Support graph:

- Samples the point cloud up to `supportSampleLimit`.
- Stores support point position, optional normal, confidence, and source index.
- Uses a 3D grid for nearby support lookup.
- Confidence can use the `Number_of_neighbors` scalar field if present.

Flow direction:

- Projects gravity onto the local normal's tangent plane.
- Falls back to gravity when normals are missing or degenerate.

Neighbour scoring:

- Searches within `maxBridgeDistance`.
- Scores candidates by downhill drop, alignment with projected gravity, confidence, normal coherence, bridge penalties, gap tolerance, and flatness.
- Penalizes long bridges and uphill moves.
- Allows more flat/spread behavior as `branching` increases.

Branch tracing:

- Main branches target full `pathLength`; secondary/spread branches are shorter.
- Each segment is sampled at `pathSampleSpacing`.
- Anchors store confidence, pooling, accumulation, width, normal, and surface steepness.
- Confidence decays over distance and more strongly across bridge jumps.
- Termination reasons include reached length, no support, max steps, loop, duplicate, and empty.

Branch generation:

- One main branch is traced per enabled emitter.
- Main branches collect branch opportunities from alternate ranked neighbours.
- Secondary/spread branch count is controlled by `branching` and `coverage`.
- Occupied support points reduce duplicate secondary routes.
- Per-emitter cache generation remaps branch IDs into a combined cache.

### From Flow Cache To Anchors

`BuildWaterPathAnchorsFromCache` turns cached branches into current path anchors:

- Skips branches in `hiddenBranchIds`.
- Copies each branch's raw anchors.
- Resolves source settings for that branch's emitter.
- Rewrites `flowId`, `emitterId`, `particleRole`, and branch `featureType`.
- Reduces confidence for low-confidence/gappy branches.
- Applies `SmoothWaterPath`.
- Emits role-0 anchor points into `pathAnchors`.

This is the layer that lets branch hiding and smoothing refresh without rerunning the expensive support graph path bake.

### Flow Trail, Lane, And Particle Generation

`BuildWaterOverlayFromPathAnchors` groups anchors by `flowId`, resolves per-emitter trail shape settings, and calls `IncludeWaterPathWithParticles`.

Trail expansion includes:

- `ResampleSplineAnchors`: path resampling for playback spacing.
- `BuildSplineViewSamples`: Catmull-Rom guide sampling.
- `IncludeWaterPathViewAnchors`: role-2 path guide points.
- `BuildTrailSurfaceIndex`: XY surface projection grid for lane paths.
- `ComputeTrailKnotScores`: curvature, near-return, flatness, and steepness scoring.
- `SimplifyTrailGuidePath`: dynamic-programming simplification for looser lanes.
- `BuildOffsetTrailLanePath`: lateral lane offsets, deterministic wander, surface projection, smoothing, and role-3 lane guide points.
- Particle emission: role-1 particles and optional ghost samples based on density, speed, trail length, and trail age.

Preview quality reduces some sample counts and horizons; final quality emits more particles and richer lane paths.

### Flow Visualization And Editing

Generated Flow sessions are ordinary point-cloud sessions with water-specific style state:

- `flowAnimation = true`.
- `waterPathView` switches between Trail View and Path View.
- `MakeWaterOverlayDisplayStyle` applies the selected water point visual.
- `SeedWaterFlowBuiltInVisuals` makes built-in and project visuals available.

`shaders/pointcloud_fast_basic.frag` filters by `particle_role`:

- Trail View keeps visible particles and uses `trail_age` for fade.
- Path View keeps particles, path guides, and lane guides, with dimmer guide roles.

Path editing is currently branch hiding:

- Path View draws debug polylines from the live overlay or cache.
- The user clicks to select the nearest branch.
- Delete or Backspace hides the selected branch.
- Ctrl-Z restores the previous hidden branch list.
- Hidden branch IDs are saved in `WaterPathCache.hiddenBranchIds`.

There is no persisted manual branch reshape/control-point edit yet.

## Feature 2: Basin

Basin creates soft haze/steam overlays from polygon regions. It uses the shared overlay point and PLY pipeline, but it does not use Flow emitters or Flow path caches.

### Basin Data Model

`WaterBasinRegion` stores:

- `id` and `name`.
- User `vertices`.
- Derived convex-style `hull`.
- Derived `baseZ`.
- `heightAbove` and `depthBelow` for the vertical inclusion envelope.
- `density`.
- Optional `outletEdgeIndex`.
- `outletBlocked`.

`RefreshWaterBasinRegionDerivedValues` rebuilds the hull, recomputes base Z from vertices, clamps the vertical envelope/density, and clears invalid outlet edge references.

### Basin UI And Bake Lifecycle

`DrawWaterBasinPanel` manages Basin UI:

- Create a new basin region.
- Arm/disarm vertex placement.
- Select and rename regions.
- Close the region by refreshing derived values.
- Remove the last vertex.
- Edit height above, depth below, density, outlet edge, outlet edge index, and outlet blocked state.
- Bake the selected basin haze.
- Delete the selected basin.

`BakeBasinHazeOverlayForActiveLayer`:

- Requires a selected basin region with at least three hull points.
- Requires a visible loaded support point cloud with CPU data.
- Calls `GenerateBasinHazeOverlay` for the selected region only.
- Writes `<source-stem>-BasinHaze.ply`.
- Adds or refreshes a generated overlay session using `MakeBasinHazeOverlayStyle`.

### Basin Haze Algorithm

`GenerateBasinHazeOverlay`:

- Refreshes each region's derived values.
- Computes a vertical envelope from `baseZ - depthBelow` to `baseZ + heightAbove`.
- Builds XY cells for points inside the hull and vertical envelope.
- Keeps the lowest point per cell as a plume site.
- Sorts plume sites by height and point index.
- Uses density and deterministic hashing to decide which sites emit plumes.
- Applies outlet masking if an outlet edge is configured and not blocked.
- Builds a short rising path per plume with drift, cross-drift, swirl, rise, speed, confidence, and feature type `1.0`.
- Recomputes path distance and expands the path through `IncludeWaterPathWithParticles`.

Basin haze gets its own trail shape and animation settings internally. It does not use the project's Flow path cache.

### Basin Output And Styling

Basin haze writes a binary PLY overlay with the same `WaterOverlayPoint` scalar schema as Flow. Its display style comes from `MakeBasinHazeOverlayStyle`, which uses soft camera-facing/world-sprite styling, trail-age opacity mapping, low emissive strength, and no Path View.

## Feature 3: Runoff

Runoff creates many short trickle overlays inside polygon regions. It is designed for dew or light-rain movement across local high points and ground cells.

### Runoff Data Model

`WaterRunoffRegion` stores:

- `id` and `name`.
- User `vertices`.
- Derived `hull`.
- `mode`: `Dew` or `LightRain`.
- `groundVoxelSize`.
- `highPointFraction`.
- `density`.
- `pathLength`.
- `maxSteps`.

`RefreshWaterRunoffRegionDerivedValues` rebuilds the hull and clamps all numeric settings.

### Runoff UI And Bake Lifecycle

`DrawWaterRunoffPanel` manages Runoff UI:

- Create a new runoff region.
- Arm/disarm vertex placement.
- Select and rename regions.
- Choose Dew or Light Rain.
- Close the region.
- Remove the last vertex.
- Edit ground voxel size, high point fraction, density, path length, and max steps.
- Bake runoff.
- Delete the selected region.

`BakeRunoffOverlayForActiveLayer`:

- Requires a visible loaded support point cloud with CPU data.
- Runs all runoff regions, not just the selected region.
- Calls `GenerateRunoffOverlay` with the active animation trail settings.
- Writes `<source-stem>-Runoff.ply`.
- Adds or refreshes a generated overlay session using `MakeRunoffOverlayStyle`.

### Runoff Algorithm

`GenerateRunoffOverlay`:

- Refreshes each region.
- Builds XY ground cells from all valid support points inside the region hull.
- Stores Z samples, position sum, and point count per cell.
- Chooses each cell's ground Z from the low Z percentile.
- Finds candidate high points above the cell's ground Z.
- Sorts candidates by height.
- Uses `highPointFraction`, mode density, region density, and deterministic hashing to keep a subset.
- Starts each trickle at the high point, drops along projected gravity/normal toward ground, then walks across neighbouring ground cells.
- Scores next ground cells by local ground drop, diagonal penalty, and small deterministic hash variation.
- Stops when no useful downhill cell exists, max steps is reached, or path length is reached.
- Recomputes path distances and expands the path through `IncludeWaterPathWithParticles`.

Runoff uses feature type `2.0`. Dew is lower density/slower; Light Rain is denser and faster.

### Runoff Output And Styling

Runoff writes a binary PLY overlay with the shared `WaterOverlayPoint` scalar schema. `MakeRunoffOverlayStyle` uses water overlay styling with point size, trail-age opacity, and accumulation emissive mapping. It does not use Path View.

## Feature 4: Caustics

Caustics are water-related render effects applied to existing LiDAR layers. They do not generate water overlay PLYs.

### Caustic Data Model

`WaterCausticRegion` stores:

- `id` and `name`.
- `targetLayerSourcePath`.
- User `vertices`.
- Derived `hull`.
- `maskVoxelSize`.
- `planeMaxResidual`.
- `planeMaxSlope`.
- `heightBand`.
- `edgeBlendWidth`.
- `previewTintMode`: off, pulse after refresh, or always.
- `enabled`.
- `maskDirty` and `maskStale`.

`WaterCausticLookSettings` stores the render look:

- enabled/intensity.
- scale/speed/line sharpness/warp.
- cell size, line width, feather, surface point spacing, warp amplitude.
- tint, emission boost, opacity boost, and point size boost.

Current implementation note: `GenerateCausticMask` currently uses polygon inclusion, edge distance, region ID, enabled state, and edge blend width. Some persisted region fields such as mask voxel size, plane residual, plane slope, and height band are clamped and saved but are not currently central to the mask algorithm.

### Caustic UI And Refresh Lifecycle

`DrawWaterCausticsPanel` manages region UI:

- Create a new caustic area targeting the selected loaded LiDAR layer.
- Arm/disarm boundary vertex placement.
- Select and rename regions.
- Toggle enabled state.
- Close the caustic area.
- Remove the last vertex.
- Edit edge blend and preview tint mode.
- Refresh mask.
- Delete caustics.

The same panel also exposes Caustic Look settings through `ViewedWaterCausticLookSettings` and editable project/animation overrides.

`RefreshSelectedWaterCausticMask` finds the selected region's target layer and calls `RefreshWaterCausticMaskForSession`.

`RefreshWaterCausticMaskForSession`:

- Requires the target point-cloud session to be loaded with CPU data.
- Collects all caustic regions targeting that same source path.
- Calls `GenerateCausticMask`.
- Upserts generated scalar fields into the target cloud:
  - `caustic_mask`
  - `caustic_edge`
  - `caustic_region_id`
  - `caustic_plane_distance`
  - `caustic_seed`
- Sanitizes style, uploads the point cloud to the GPU, clears mask dirty/stale flags for target regions, and optionally starts a preview tint pulse.

### Caustic Mask Algorithm

`GenerateCausticMask`:

- Allocates mask, edge, region ID, plane distance, and seed arrays for the target point count.
- Refreshes region derived values.
- Builds a boundary from region vertices.
- Skips disabled or incomplete regions.
- Tests each valid point for XY polygon inclusion.
- Computes 3D edge distance, falling back to XY edge distance if needed.
- Converts edge distance into a smooth edge value using `edgeBlendWidth`.
- Replaces existing mask values when the new region has a stronger edge value or a lower region ID tie-breaker.
- Stores mask `1.0`, edge, region ID, plane distance `0.0`, and deterministic region seed.

`affectedPointCount` records how many points enter the mask.

### Caustic Rendering

`ApplyWaterCausticRenderStyle` binds caustics into a target LiDAR layer's point style when:

- The session has at least one caustic region targeting it.
- The required scalar fields exist.
- The look is enabled with nonzero intensity, or editor preview tint is active.

`VulkanViewportShell` uploads caustic control slots and parameter vectors. `shaders/pointcloud_fast_basic.frag` loads mask/edge/seed scalar values, computes animated caustic strength, and mixes toward the caustic tint. `shaders/pointcloud_caustics.glsl` provides the Voronoi-ridge pattern, surface UV logic, edge gating, and preview tint.

Caustics affect LiDAR rendering directly. They should be tested on the target LiDAR layer, not by looking for a generated water PLY file.

## Overlay PLY Schema For Flow, Basin, And Runoff

`WriteWaterOverlayPly` writes binary little-endian PLY files with one vertex per `WaterOverlayPoint`.

Properties:

- `x`, `y`, `z`
- `normal_x`, `normal_y`, `normal_z`
- `red`, `green`, `blue`
- `scalar_flow_id`
- `scalar_emitter_id`
- `scalar_path_distance`
- `scalar_phase`
- `scalar_speed`
- `scalar_width`
- `scalar_confidence`
- `scalar_accumulation`
- `scalar_pooling`
- `scalar_particle_role`
- `scalar_path_start_index`
- `scalar_path_point_count`
- `scalar_jitter_seed`
- `scalar_trail_age`
- `scalar_trail_length`
- `scalar_feature_type`
- `scalar_region_id`
- `scalar_surface_steepness`
- `scalar_trail_lane_id`
- `scalar_trail_lateral_offset`

`BuildWaterOverlayPointCloud` exposes the same values as in-memory scalar fields without the `scalar_` prefix, for example `flow_id`, `particle_role`, and `trail_age`.

## Change Guide

### Flow Changes

Routing and cache behavior:

- Edit `TuneWaterPathSettings`, `BuildSupportGraph`, `FlowDirection`, `RankDownhillNeighbours`, `TraceWaterPathBranch`, and `GenerateWaterPathCache`.
- Update `AppendWaterPathBakeSettingsFingerprint` for any new bake-affecting setting.
- Update `WaterPathAutoTuneDiagnostics` and serialization if new diagnostics should persist or appear in UI.
- Be careful with branch IDs because `hiddenBranchIds` persists user branch-hiding edits.

Trail and particle behavior:

- Edit `TrailPlaybackSampleSpacing`, `ResampleSplineAnchors`, `ComputeTrailKnotScores`, `SimplifyTrailGuidePath`, `BuildOffsetTrailLanePath`, `IncludeWaterTrailLaneGuides`, and `IncludeWaterPathWithParticles`.
- Test both preview and final quality paths.
- Keep `particleRole`, `pathStartIndex`, and `pathPointCount` coherent with shader and debug overlay expectations.

Flow UI:

- Edit `DrawWaterPanel`, `BakeWaterOverlayForActiveLayer`, `RefreshWaterOverlayFromAnchors`, emitter placement/move helpers, and source settings helpers.
- Routing changes should mark the path dirty. Trail/visual changes should refresh from cached anchors where possible.

### Basin Changes

Data and persistence:

- Edit `WaterBasinRegion`, `SerializeWaterBasinRegion`, and `ParseWaterBasinRegion`.
- Keep `RefreshWaterBasinRegionDerivedValues` responsible for derived hull/base Z and clamping.

Generation:

- Edit `GenerateBasinHazeOverlay` for plume placement, density, outlet behavior, rise/drift/swirl, and Basin feature point styling.
- Edit `BakeBasinHazeOverlayForActiveLayer` if bake scope changes from selected-only to all regions.
- Edit `MakeBasinHazeOverlayStyle` for viewport style.

UI:

- Edit `DrawWaterBasinPanel` for controls and workflow.

### Runoff Changes

Data and persistence:

- Edit `WaterRunoffRegion`, `SerializeWaterRunoffRegion`, and `ParseWaterRunoffRegion`.
- Keep `RefreshWaterRunoffRegionDerivedValues` responsible for clamping.

Generation:

- Edit `GenerateRunoffOverlay` for ground-cell estimation, high-point filtering, mode behavior, local downhill scoring, and trickle path expansion.
- Edit `BakeRunoffOverlayForActiveLayer` if bake scope should become selected-only or support multiple output files.
- Edit `MakeRunoffOverlayStyle` for viewport style.

UI:

- Edit `DrawWaterRunoffPanel` for controls and workflow.

### Caustic Changes

Data and persistence:

- Edit `WaterCausticRegion`, `WaterCausticLookSettings`, their serializers/parsers, and project/animation load/save paths.
- If plane or height-band fields become algorithmically active, update this document and add acceptance checks around them.

Mask generation and upload:

- Edit `GenerateCausticMask` for selection-mask semantics.
- Edit `RefreshWaterCausticMaskForSession` for scalar field names, upload behavior, and dirty/stale handling.
- Keep scalar names aligned with `FindCaustic*ScalarFieldSlot` helpers and style binding.

Rendering:

- Edit `ApplyWaterCausticRenderStyle`, `VulkanViewportShell` caustic uniform packing, `shaders/pointcloud_fast_basic.frag`, and `shaders/pointcloud_caustics.glsl`.
- Test editor preview tint modes and export/offline render paths if caustic style changes.

### Cross-Cutting Changes

- For new region fields, update region structs, refresh functions, UI controls, serializers, project save/load, water sources save/load, and any animation override path if relevant.
- For new overlay scalar fields, update `WaterOverlayPoint`, `WriteWaterOverlayPly`, `BuildWaterOverlayPointCloud`, visual presets, shader constants, and style serialization if the visual uses the field.
- For new generated files, update path builders and artifact documentation.
- Keep Flow overlay PLYs and Caustic LiDAR scalar fields conceptually separate.

## Legacy And Reassessment Notes

Feature-set-wide notes:

- `water_sources.json` stores sources, regions, caustic look settings, and optionally the Flow path cache, but not every project water visual or animation trail profile.
- The project document carries additional water state, legacy water settings, point visuals, and animation-specific overrides.
- Basin and Runoff are PLY overlay features; Caustics are target-layer scalar fields. Treating all water features as generated overlays will lead to wrong implementation choices.
- Region editing is shared, but generation and persistence details differ by feature.

Flow-specific notes:

- The Flow report was previously the main water document, which made Basin/Runoff/Caustics feel secondary even though they are first-class feature panels.
- `WaterSettingsBundle`, `WaterParticleTrailSettings`, `WaterParticleVisualSettings`, `WaterVisualSettings`, `WaterBakeSettings`, and `WaterRenderSettings` still exist for compatibility.
- `WaterParticleTrailShapeSettings` persists `trailTurbulence`, `trailMomentum`, and `normalTurbulenceResponse`, but the current trail generation mainly uses looseness, smoothness, jitter, spline spacing, and lane count.
- Flow overlay sessions can be refreshed in memory without rewriting the last `-WaterFlow.ply`.
- Branch editing is hide-only; there is no persisted manual path reshape.
- Hidden branch IDs depend on generated branch IDs, so branch-generation changes can invalidate old hide edits.

Basin/Runoff notes:

- Basin bake currently uses the selected basin region only.
- Runoff bake currently uses all runoff regions.
- Both features reuse `IncludeWaterPathWithParticles`, so changes to trail particle generation can affect Flow, Basin, and Runoff together.

Caustic notes:

- Several persisted caustic region fields are currently not central to `GenerateCausticMask`.
- `caustic_plane_distance` is generated but currently written as `0.0`.
- Caustic rendering requires the generated scalar fields to be present on the target LiDAR layer and the look to be active or preview tint to be active.

## Practical Acceptance Checks

Use these checks after water feature changes:

- Flow: place an emitter, bake Flow, confirm `<source-stem>-WaterFlow.ply` and `<source-stem>-WaterPathCache.json` are produced or refreshed.
- Flow: switch Trail View and Path View and confirm particle roles are filtered differently.
- Flow: hide a branch in Path View, undo it, save/reload, and confirm `hiddenBranchIds` behaves as expected.
- Basin: create a basin region, close it, bake Basin Haze, and confirm `<source-stem>-BasinHaze.ply` appears and displays.
- Runoff: create a runoff region, bake Runoff, and confirm `<source-stem>-Runoff.ply` appears and displays.
- Caustics: create a caustic area targeting a loaded LiDAR layer, refresh the mask, and confirm the target session has `caustic_mask`, `caustic_edge`, `caustic_region_id`, `caustic_plane_distance`, and `caustic_seed` fields.
- Persistence: save and reload `water_sources.json` and the project document, then confirm emitters, regions, selected settings, and stale/dirty states are sane.
- Rendering: verify changed visuals in viewport and, when relevant, export/offline paths.
