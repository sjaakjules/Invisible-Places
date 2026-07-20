# Water Feature Set Report

## Goal

Water v2 exposes seven active water tabs, ordered by their authored workflow:

- **Ripples**: region-based procedural water effects selected on the canonical analysis support and presented on the committed display cloud. Legacy Caustics load as Ripples with overlay type `Caustic Lace`; the region overlay set also includes a local polygon `Shoreline` pattern.
- **Shoreline**: a separate SAND-role point-cloud shader effect that uses the sand cloud itself to simulate waves washing toward and away from the beach boundary.
- **Seepage**: manually placed surface nodes that fan downward over ROCK/VEG points, producing persistent dampness, subtle ripples, and moisture glints without generating a stream.
- **Rain**: camera-aware deterministic falling drops and surface-shedding trails over the scene, with Light Mist, Rain, and Heavy Downpour intensities.
- **Flow**: emitter/point-source path caches plus generated world-aligned stream surfels for artist-directed small streams.
- **Mesh Flow**: triangle-mesh-guided sources, attractors, and GPU-previewed surface routes for topology-aware flowing water.
- **Field**: a second point-source/region approach that builds local vector fields to generate Field Streamlines and Field Surface Motion.

The base point cloud remains the visual source of truth. Ripples modify the base cloud through sparse region memberships plus small runtime parameter buffers that the viewport and offline renderer evaluate procedurally. Shoreline modifies the SAND base-cloud draw through point-style shader uniforms and does not allocate a region membership. Seepage evaluates compact node/spatial-hash buffers directly on eligible base-cloud points. Flow, Mesh Flow, Field Streamlines, and Rain may add generated stream overlay sessions. Field Surface Motion currently modifies the base cloud through generated `water_effect_*` fields. None of the active paths should rely on dense permanent scalar fields or newly generated growing point patterns as the primary visual.

Basin and Runoff are removed from the public v2 workflow. Legacy code and tests may still exist for compatibility, but Basin/Runoff tabs, runtime load state, generation, and new project saves are not part of the active water contract. Old Basin/Runoff JSON records are tolerated and ignored.

## Current Status

Implemented in the current repository:

- Active Water tabs ordered Ripples, Shoreline, Seepage, Rain, Flow, Mesh Flow, and Field.
- Project schema `34` with authoritative scene density groups, Seepage nodes/shared looks, v2 Ripple/Flow/Field settings, saved Flow caches, and legacy Caustics-to-Ripples migration.
- Ripple `WaterEffectLayer` records and distinct shader/offline procedural patterns for all `WaterRippleOverlayType` values.
- SAND-role shader Shoreline waves driven by point-cloud style settings, including boundary height, reach, edge fade, direction, pattern scale, wavelength, speed, warp, turbulence, density, seed, colour, opacity, emission, and point-size response.
- Canonical-surface Seepage placement and editing with compact surface-following downhill ribbons, ROCK/VEG/SAND role targeting, shared and local look profiles, rain-responsive dampness, compact spatial hashes, and topology-versus-parameter GPU upload separation.
- Sparse Ripple membership uploads for selected region points, with pattern/response edits updating compact GPU params when region membership has not changed. This supports millisecond-scale live modifications instead of CPU-regenerating dense fields.
- Shared region selection for Ripple and Field support, including selected base point indices, edge weights, normals, source scalar values, field vectors, and manual Field control flags.
- Flow path cache reuse, branch hiding, and generated stream surfels with the v2 stream scalar contract.
- Flow Bake Path performs source-level incremental cache reuse. Branches store per-emitter bake fingerprints, so moving or adding one source rebuilds that source while unchanged source branches are retained.
- Flow Path supports an optional viewport-placed attractor with tunable strength. It biases path ranking in XY while Z remains the vertical downhill axis.
- Project-owned Flow emitters and point sources. Sources are no longer loaded from a separate global save into every project.
- Saved Flow path cache data can reload baked paths and anchors when support/settings signatures still match.
- Combined ROCK/SAND/VEG support sampling for grouped scenes. Flow sources and path baking use CPU-ready canonical ROCK 1 mm, SAND 2 mm, and VEG 1 mm analysis sources as one support surface; changing the visible density does not invalidate support or create a full merged GPU point buffer.
- Field cache, Field Streamlines, and Field Surface Motion built from Flow path anchors or user-authored Field regions, with region Field caches saved and reused offline.
- Field no-flow, bridge-allowed, and bridge-blocked control regions with visible diagnostics.
- Shared animated trail visualization for Flow, Field streams, and Rain; Flow moves along baked path anchors, Field moves along cached vector-field paths seeded from perturbed source points, and Rain phase-wraps falling/surface-shedding routes from above the camera footprint onto scene support.
- Rain generation uses grouped-scene support roles, walks from the first impact through lower ROCK/support points, continues briefly on `SAND` before fading, falls back to lowest reachable support when no sand is present, and encodes camera-distance fade plus wind/noise fields in the existing trail scalar contract. Rain route anchors carry time fractions so falling remains fast while surface runoff is slower.
- Active-cloud sparse runtime Ripple evaluation and `water_effect_*` composition for Field Surface Motion, with Visuals-tab Water Effect Stack controls for both families.
- Viewport/offline/export rendering of water output without requiring water PLY export.
- Legacy Basin/Runoff removal from the active public UI and new-save contract.

## Architecture Map

- `src/water/WaterFlow.hpp` and `src/water/WaterFlow.cpp`: v2 water structs, Seepage look resolution/spatial-grid evaluation, shared region selection, flow path generation, shared stream surfel generation, Field cache persistence/streamline generation, Ripple/Field sparse effect generation, legacy water helpers, and point-cloud conversion.
- `src/app/Application.cpp`: Water panel UI, runtime `WaterWorkflowState`, emitter/node/region editing, Seepage GPU refresh orchestration, cache/bake orchestration, Rain generation, in-memory overlay sessions, and project/source document wiring.
- `src/serialization/ProjectDocument.hpp` and `src/serialization/ProjectDocument.cpp`: schema-34 project state, schema-11 water-source state, Seepage nodes/looks, v2 water settings/layers, legacy migration, and path cache persistence.
- `src/renderer/pointcloud/PointCloudPreviewState.*`, `src/renderer/core/VulkanViewportShell.cpp`, and root `shaders/pointcloud_*`: water overlay render mode, SAND-role Shoreline wave uniforms/shading, compact Seepage buffers/hash evaluation, stream tangent/width/world-length scalar handling, and stream surfel shading.
- `tests/AssetDiscoveryTests.cpp`, `tests/SeepageWaterTests.cpp`, `tests/SeepageOfflineRendererTests.cpp`, and `tests/SeepageSerializationTests.cpp`: serialization, legacy migration, deterministic generation, cache invalidation, bounded Seepage grid/runtime behavior, offline/Fast Basic parity, and shader/style contract coverage.

## Runtime Model

`WaterWorkflowState` in `src/app/Application.cpp` owns the live workflow state. The v2-relevant fields are:

- `emitters`, `defaultSourceSettings`, `tempDefaultSourceSettings`, and per-emitter settings for Flow path generation.
- `pathCache`, `pathAnchors`, path revisions, dirty flags, and hidden branch IDs for reusable Flow paths.
- Path/Lanes/Trail profiles plus `flowStreamOverlay` for generated Flow trails. Internal `stream_*` names are retained for renderer/offline compatibility.
- `rippleLayers` plus sparse runtime memberships/params for current Ripple evaluation. `rippleEffectOverlay` is kept as selected-region debug/evidence data, not as a generated visible Ripple layer.
- Shoreline wave settings live on `PointCloudStyleState` and are mirrored to the SAND role in grouped scenes. The Water > Shoreline panel edits the active LiDAR visual owner, then syncs scene point visuals without touching Ripple region membership.
- `seepageNodes`, `defaultSeepageLook`, and `seepageLookProfiles` own the authored Seepage state; topology/parameter fingerprints, upload revisions, GPU byte counts, and overflow diagnostics own its derived per-layer runtime state.
- `fieldSettings`, `fieldStreamSettings`, `fieldCache`, `fieldStreamOverlay`, and `fieldSurfaceEffectOverlay` for Field.
- `rainSettings`, selected Rain trail profile state, diagnostics, and `rainTrailOverlay` for generated `-RainTrails.generated` sessions.
- `activeRegionFeature`, `regionEditor`, and placement flags for editable Ripple regions and legacy-safe region editing.

Generated water overlay sessions are excluded from support-layer discovery and from base-cloud look-dev/export visual selection. They are renderable water output, not source LiDAR layers for future water bakes. Ripples no longer create active visible `-Ripples.generated` sessions; their display-source-specific membership and procedural params are uploaded to the committed base-cloud renderer instead. Shoreline waves are not generated sessions either; they are evaluated during the committed SAND point-cloud draw. Seepage likewise remains on the base draw and uploads only compact nodes, occupied hash cells, and bounded node references. Field Surface Motion contributes to display-cloud composition through spatially remapped `water_effect_*` fields, while Flow trails, Mesh Flow, Field Streamlines, and Rain remain generated overlay sessions.

For grouped LiDAR scenes, support discovery uses the CPU-ready canonical analysis roles under one folder-level scene. Those sources remain resident even when a sparser display bundle is selected. The renderer uploads and draws only the committed display role layers; staged switch targets are not renderable. Water support builders never substitute a selected display cloud for missing analysis data, and dependent actions report that their analysis input is still loading or unavailable.

Ripple, Seepage, and Field presentation data follows the display bundle without changing authored support. Ripple memberships are rebuilt or restored for each exact display source because point indices differ between density variants. Seepage reuses its world-space nodes while building a compact role-filtered grid for each committed render layer; Auto quality follows effective point invocations. Field solves stay on the analysis set, then presentation fields are spatially remapped to committed display points. Source point indices are never copied from an analysis cloud to a different display cloud. Generated Flow, Mesh Flow, Field Streamline, and Rain sessions are independent overlays and are not replaced by density switching.

## Active UI Contract

The active Water panel tabs are:

```text
Ripples
Shoreline
Seepage
Rain
Flow
Mesh Flow
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

Ripples exposes region/layer controls and procedural overlay settings. Shoreline owns the SAND boundary-wave look. Seepage exposes viewport node placement/selection/movement, fan geometry and role targeting, quality/density/prominence controls, shared look profiles, and local saved/temporary overrides. Rain exposes enablement, Light Mist/Rain/Heavy Downpour intensity, shared trail visual profile editing, seed/count/fall speed, surface run speed, sand run distance, wind/noise, spawn footprint, camera death distance, and regeneration controls. Flow exposes path baking, branch hiding, source profile assignments, Lanes controls, and Trail styling. Mesh Flow exposes mesh-guided sources, attractors, particle state, preview/final route controls, and mesh trail styling. Field exposes field build settings, stream settings, surface-motion output controls, and user-authored Field regions.

## Serialization Contract

Project documents now use schema `34`. The authoritative grouped-density record remains:

```text
scene_point_cloud_groups[]
  scene_group
  display_spacing_meters
  display_loaded
  display_visible
  roles[]
    scene_role
    analysis_source_path
    display_source_path
```

Legacy per-layer scene fields remain compatibility mirrors. Schema-32-and-earlier projects are migrated to a complete display bundle when the scene catalog permits it, while scenes with no complete density remain non-switchable `Mixed` sets. New saves also write the v2 water keys:

```text
scene_group
scene_role
inferred_point_spacing_meters
point_spacing_meters
point_spacing_manual_override
selected_scene_variant_path
water_emitters
water_seepage_nodes
water_seepage_default_look
water_seepage_look_profiles
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
water_rain_settings
selected_water_rain_trail_profile
temp_water_rain_trail_profile
```

Project saves also preserve `water_animation_trail_settings`, `water_animation_trail_profiles`, and caustic look settings for legacy animation/visual compatibility, even though those are no longer standalone Water tabs.

Shoreline wave settings are serialized with the point-cloud style rather than in `water_ripple_layers`. The active style keys include `shoreline_wave_enabled`, `shoreline_boundary_z`, `shoreline_height_reach_meters`, `shoreline_edge_fade_meters`, direction, pattern scale, wavelength, speed, warp, turbulence, density, phase, intensity, response, tint, and seed fields.

Seepage nodes are stored both in the active scene state and in the project compatibility mirror. Each node preserves its world position, surface normal, down axis, fan reach/width/feather/depth tolerance, normal alignment, strength, seed, viewport/export flags, target roles, assigned profile, committed local override, and temporary local override. Shared defaults and named profiles use `water_seepage_default_look` and `water_seepage_look_profiles`.

`water_sources.json` schema `11` mirrors the active source/layer/settings subset for reusable water setup, including Seepage nodes/looks, the same Ripple/Flow/Field/Rain settings, and the current Flow path cache when available.

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
- Water emitters and point sources are stored inside the project document. Loading another project should not import sources from a previous project unless that project explicitly contains them.
- Schema-33-and-earlier documents without Seepage keys load empty node/profile lists and the current subtle-damp default look.

## Ripples

Ripples use `WaterEffectLayer` records plus sparse region memberships to evaluate procedural effects directly on the committed display cloud. Region analysis uses canonical support, but membership indices are rebuilt/restored for each exact display source. The active workflow does not create visible `-Ripples.generated` point-cloud sessions and does not upload dense `water_effect_*` or `ripple_*` scalar fields for ordinary Ripple recalculation.

Do not confuse the region `Shoreline` Ripple overlay with Water > Shoreline. The Ripple overlay remains a local polygon effect using `RippleTideBandsValue` / `RuntimeRippleTideBandsValue` and sparse selected-region memberships. The Water > Shoreline tab is the SAND-role point-cloud shader path described in the next section.

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

## Shoreline

Water > Shoreline is a grouped-scene point-cloud style effect for beach/sand waves. It applies only to the SAND role when scene visuals are mirrored across ROCK/SAND/VEG. ROCK and VEG do not receive the shader wave even when the folder-level visual owner has Shoreline enabled.

The runtime is shader-first. `VulkanViewportShell` packs the active `PointCloudStyleState` shoreline settings into `shorelineWaveControl`, `shorelineWaveParams0..4`, and `shorelineWaveTint`; `EvaluateShorelineWaveContribution` in `shaders/pointcloud_sparse_ripple.glsl` evaluates the contribution during point-cloud rendering. The shader derives its height mask from `boundaryZ - worldPosition.z`, so the editable boundary height marks the wet/dry threshold and `Height Reach` controls how far down the SAND point cloud the wave can travel.

The SAND shader uses `SandCloudShorelineWaveValue`, not the region Ripple `RippleTideBandsValue`. `shoreDistance` drives wave travel and `uv.y` is the along-shore tangent coordinate, so foam breakup runs with the shore instead of producing shore-normal bands. Incoming and receding phases share the same front-local foam grain: the affected foam remains on the deeper side of the moving front, grows as the wave moves landward, and shrinks as it recedes.

Water > Shoreline controls map directly to point-style fields:

- `Boundary Height`, `Height Reach`, and `Edge Fade` define the vertical SAND mask around the shore boundary.
- `Wave Direction`, `Pattern Scale`, `Wavelength`, `Speed`, `Warp`, `Turbulence`, `Band Density`, `Phase`, and `Seed` define the travelling wave pattern.
- `Intensity`, `Emission Add`, `Opacity Add`, `Opacity Multiply`, `Point Size Add`, `Point Size Multiply`, `Colour Mix`, and `Tint` define how foam affects the base SAND points.

Shoreline does not create `WaterEffectLayer` records, does not build region previews, does not emit `water_effect_*` or `ripple_*` scalar fields, and does not create a `-Ripples.generated` or `-RainTrails.generated` point-cloud session. It is saved as point-cloud style data and is rendered by viewport/offline point-cloud shaders wherever the point style is active.

## Seepage

Water > Seepage represents persistent damp areas rather than full streams. `Place Seepage Node` uses the canonical analysis-surface picker shared with Flow sources. Each node stores the picked position and surface normal, then derives a transient, gravity-descending guide across the canonical 3D surface. At most eight stations describe the changing center, normal, and cumulative surface distance, letting the widening damp ribbon turn around cliff contours instead of remaining in the clicked tangent plane. Reach, start/end width, edge feather, surface-depth tolerance, normal alignment, strength, and deterministic seed remain authored controls. Nodes can be selected in the viewport or list, moved back onto support, renamed, role-targeted, hidden from the viewport or export, and deleted. ROCK and VEG are enabled by default; SAND remains an explicit option.

The look separates authored topology from live visual parameters. `WaterSeepageLookSettings` exposes Auto/Low/Balanced/High quality, Base Wetness, Ripple Density, Glisten, Wavelength, Pattern Scale, Speed, Warp, Turbulence, Phase, Rain Response, Prominence, colour/opacity/point-size/emission response, and the shared add/max/multiply/screen/override blend modes. A node can use Default, a named shared profile, a committed local override, or a temporary edited override. Saving a look updates parameters without changing fan topology.

`BuildWaterSeepageSurfaceGuides` samples only a capped canonical support set, prefers ROCK substrate, follows locally projected gravity, rejects rising steps, and stops when trustworthy surface support ends. `BuildWaterSeepageSpatialGrid` filters the small authored node list by scene role and viewport/export enablement, resolves profiles and Auto quality, and builds a compact world-space hash over the swept ribbon bounds. Each occupied cell stores at most `WaterSeepageSpatialGrid::kMaxReferencesPerCell` node references and reports overflow rather than allocating work per point. `VulkanViewportShell` uploads node/guide, hash-cell, and reference buffers; topology fingerprints trigger full buffer replacement, while look/Rain parameter fingerprints update only the node buffer. Point shaders query the local cell, find the closest local ribbon segment, and evaluate dampness, animated ripple breakup, and moisture glints on the GPU, so the cost follows nearby node references instead of the full authored-node count.

The Seepage Nodes section provides a transient `Show Structure Overlay` viewport option. It defaults off so the authored node points and actual wet points remain visible without diagram clutter. Turning it on draws the evaluator-aligned affected ribbons, translucent regions, downhill centrelines, end caps, and support warnings for every viewport-enabled node; the selected node is emphasized. Node markers and the selected-node label remain visible in both states. This is an editor overlay only: it is not serialized and does not alter GPU Seepage topology, look parameters, stills, or animation exports. Incomplete support is capped and warned rather than represented by a misleading vertical fan.

Auto quality resolves High at up to 10 million effective point invocations, Balanced through 50 million, and Low above 50 million; an explicit quality remains fixed. Rain presets raise Seepage visual moisture through `rainResponse` without changing the spatial topology because guides are pretraced to the maximum Rain reach. Seepage creates no generated PLY or growing point overlay: nodes/default/profile state serialize in the project and `water_sources.json`, while surface guides, the spatial grid, and GPU buffers are transient and derived after load.

GUI smoke scenarios `seepage-sample-terrestrial` and `seepage-site3-100m` exercise the real Flow-style click picker, canonical surface-guide tracing, Structure Overlay isolation, eight-node GPU upload, parameter-only look/Rain changes, memory bound, and screen-sprite frame overhead. On the checked-in 100,743,210-point fixture, surface-guided M1 Max reference runs resolved Auto to Low, measured 0.33–5.69% median overhead, completed look updates in 0.179–0.196 ms, and used 0.004 MiB of auxiliary Seepage data.

### Future improvement: mesh-assisted surface guides (not implemented)

Mesh-assisted Seepage tracing is a roadmap option, not part of the current implementation. The current Seepage guide continues to follow capped canonical point support. A future version could use the scene's continuous mesh to bridge otherwise trustworthy scan gaps and follow cliff curvature more consistently, while preserving the point cloud as the visual and role-classification source of truth.

This requires a new orientation-independent 3D triangle index. The existing `MeshSurfaceCache` groups samples in an XY height field, which is useful for its current Mesh Flow workflow but can collapse vertical or overhanging cliff faces and is therefore not a safe Seepage guide surface. The proposed index would retain connected triangle geometry and support nearest-triangle and downhill-adjacency queries in arbitrary orientation. It should be built asynchronously once per scene, shared by every Seepage node, and discarded or rebuilt when the source mesh changes.

The mesh would participate only when a node is placed, moved, or given topology-changing geometry. The guide builder would project gravity through connected surface triangles, validate the route against nearby canonical point roles, and reduce the result to the same maximum eight transient guide stations already consumed by the viewport and offline evaluator. Validation should prefer ROCK support, allow VEG where the node enables it, and include SAND only when the node explicitly opts in. A route that disagrees materially with nearby ROCK points, encounters an invalid mesh region, or cannot establish a trustworthy triangle continuation should reject the questionable section or fall back to the current point-supported tracer.

The full mesh must never be queried by each rendered point and must not be uploaded as part of the Seepage shader contract. Only the compact guide stations and existing spatial hash would reach the GPU, so mesh assistance is expected to add no steady-state Seepage shader cost and no display-density-specific point allocation. The conservative current 100M-point Seepage reference overhead is 5.69%; that remains the upper comparison baseline for a mesh-assisted implementation.

The tradeoff is one-time CPU loading/index construction and retained scene-index memory. Site3's current mesh contains approximately 7.05 million vertices and 14.09 million triangles, so the index must use bounded, compact storage, expose its memory use, warm asynchronously without blocking ordinary point-cloud viewing, and be shared rather than duplicated per node. Existing mesh smoke measurements provide useful scale but are not yet Seepage measurements: the recorded cold mesh operation took about 32.75 seconds, while a warm cached move took about 0.34 ms. Any implementation should separately benchmark cold warm-up time, warm node-edit latency, peak/retained CPU memory, guide agreement with ROCK points, fallback frequency, and confirm that the current steady-state GPU overhead does not regress.

## Flow

Flow is the point-source/emitter path approach for small streams over the site. It exposes three profile-backed setting areas: Path, Lanes, and Trail, and keeps the existing path bake model:

1. Emitters and source/path settings define bake inputs.
2. `GenerateWaterPathCache` creates or refreshes `WaterPathCache`; Bake Path may merge unchanged cached branches with newly rebuilt branches for only the dirty emitters.
3. `BuildWaterPathAnchorsFromCache` rebuilds visible anchors and applies hidden branch IDs.
4. `BuildFlowStreamOverlayFromPathAnchors` converts path anchors into shared animated trail paths and generates deterministic trail surfels with legacy `stream_*` scalar names.
5. `BuildWaterStreamOverlayPointCloud` exposes generated trails as in-memory point-cloud sessions grouped by resolved Trail profile.

Path-affecting changes dirty the path cache. Source movement, source insertion, and source deletion are tracked by emitter ID, so the next Bake Path rebuilds only the changed sources when the existing support signature still matches. Global Path profile edits, support setting changes, and attractor edits dirty all current source paths because they can change every route. Lane changes such as trail count, lane count, coverage width, crossing, turbulence, and speed refresh from existing anchors without dirtying the path cache. Trail geometry changes regenerate generated trail samples when needed; Trail colour, opacity, and emission are owned by Water > Flow, not the Visuals tab.

The Path attractor is an optional global Path setting. `Place Attractor` uses the same detailed grouped-scene support picking as source placement and shows a viewport marker/preview. `Attractor Strength` ranges from 0 to 1; it nudges candidate ranking toward the attractor in horizontal scene coordinates while the downhill score and Z-drop acceptance rules keep Z as the vertical descent axis. The attractor position is included in focused support sampling and bake fingerprints, so moving it causes a correct path rebake instead of a stale anchor refresh.

Flow trails replace legacy trail particles as the primary visible water output. The old generated water PLY workflow is no longer required for viewport, EXR, or MP4 water visuals.

Flow Trail geometry stays static while shader/offline playback derives animated age from `point_age`, `point_seed`, `stream_speed`, and render time. Opacity, emission, and colour energy can change over time without rebaking paths or regenerating topology.

## Mesh Flow

Water > Mesh Flow uses the active scene triangle mesh as an explicit surface. Sources and animated attractor nodes are placed by screen-ray projection into a reusable `MeshSurfaceCache`; route integration follows mesh topology with downhill, attraction, source velocity, curl, branching, eddy, and inertia controls. The tab exposes cache/projection controls, particle-state presets, preview/final particle limits, route geometry/speed, attractor keyframes, source motion, trail profiles, and GPU preview diagnostics. Mesh Flow reuses Flow emitters for sources but remains a separate generated stream workflow and does not change ordinary point-cloud Flow path caches.

## Field

Field is the second point-source/region approach for small streams. Instead of following baked Flow paths directly, it builds local vector-field support from Flow path anchors or user-authored Field regions:

1. `BuildFieldCacheFromPathAnchors` creates a local corridor-like `WaterFieldCache`.
2. `BuildFieldCacheFromRegions` creates region-local field nodes from selected surface support and Field control regions.
3. `BuildFieldStreamOverlay` integrates source-point paths through the cached vector field, then emits Field Streamlines using the same animated trail schema as Flow.
4. `GenerateFieldSurfaceEffectOverlay` emits a virtual/effect overlay for Field Surface Motion.

Field output should stay surface-bound. When support is weak, streams should bridge only valid gaps and otherwise fade or terminate. Field caches are local to path corridors or selected regions; never build a whole-scene field over the full point cloud.

Field can now build from user-authored regions stored as `WaterEffectLayer` records with `FieldSurfaceMotion` feature type. Ripple and Field region containment use one shared selection helper, so C-shaped regions exclude the cut-out area and selected point metadata is available for field editing and composition.
Field control regions can mark local support as no-flow, bridge-allowed, or bridge-blocked. No-flow support is excluded from Field Streamlines and Field Surface Motion. Bridge-allowed regions can permit a bounded manual bridge over an otherwise over-limit gap, while bridge-blocked regions force a split.
Field streamlines start from non-disabled water emitters projected into the selected Field support. Each source path receives deterministic seed-based spawn perturbation; if no emitter can seed the field, support points in the selected region seed fallback paths. Field streamlines split across rejected over-limit gaps and use low surface confidence to fade stream opacity/emission through the `stream_confidence` scalar. The generated overlay records accepted bridge, rejected gap, low-confidence fade, hard termination, no-flow, bridge-allowed, and bridge-blocked counters that are shown in the Field panel.
Field Surface Motion currently solves on canonical analysis support, then spatially remaps and pre-composes `water_effect_*` fields on the committed display cloud; renderers apply those fields after existing base mappings for size, opacity, emission, and colour. Saved Ripple and Field-region projects regenerate their display payloads when the project or target density loads.

Field should continue moving toward the Ripple performance pattern where practical: region-bounded support should be reused aggressively, uploads should be limited to selected/cache nodes rather than full-cloud fields, and shader/offline-side procedural evaluation should be preferred for editable visual parameters. Field cache generation can remain CPU-side while region selection, Field Surface Motion, and stream styling should avoid whole-cloud recomputation when only visual or playback parameters change.

## Rain

Rain simulates water shedding over the visible scene rather than a localized point source. Water > Rain owns `WaterRainSettings`, a selected Rain visual profile, diagnostics, and the generated `rainTrailOverlay`. Enabling Rain builds an in-memory `-RainTrails.generated` stream overlay through `BuildRainTrailOverlay`; disabling Rain unloads those generated sessions.

Rain support is sampled from the grouped scene's canonical analysis roles through `BuildRainSupportLayers`. `BuildRainSupportIndex` creates a lightweight XY grid up to `supportSampleLimit`, preserving support role names so the route builder can recognize SAND. Each drop spawns above the camera footprint, with out-of-frame margin, height, radius, seed, wind direction, wind strength, wind noise, and wind response shaping the falling segment.

After the first support hit, route anchors follow lower nearby support points at `surfaceRunSpeedMetersPerSecond`. When a route reaches SAND it may continue for `sandRunDistanceMeters` and then terminate; without SAND it terminates on fallback support, and without a support hit it falls to a no-hit kill plane below the scene. Route anchors store time fractions, so falling stays fast while surface runoff/shedding is slower. Diagnostics report requested/emitted drops, visible samples, route anchors, first support hits, sand terminations, fallback terminations, and no-support kills.

The generated overlay uses the same `stream_*` scalar contract as Flow and Field, but marks Rain with `feature_type = 4`. Hidden route-anchor samples use `trail_role < 0.5`; visible rain streak samples use `trail_role >= 0.5` and reference their route through `route_start_index`, `route_point_count`, `route_length`, and `stream_start_phase`. Renderer/offline branches use those route fields plus wind/camera-distance values to phase-wrap the streaks and fade them by camera death distance.

Water > Rain controls include enablement, intensity preset, drop count, fall speed, surface run speed, sand run distance, wind direction/strength/noise/response, spawn height/radius, camera death distance, route anchor count, seed, support sample count, and Regenerate Rain. Rain Visual uses the shared trail profile editor for trail length, width, point spacing, streak length, colour, opacity, and emission. Rain intensity presets scale density, speed, width/streak length, wind response/noise, and visual wetness without changing Flow or Field semantics.

## Stream Surfel Scalar Contract

Generated Flow trails, Mesh Flow trails, Field Streamlines, and Rain trails must expose these scalar fields in this order:

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

The renderer consumes `stream_role`, route fields, `stream_width`, `stream_world_length`, `stream_confidence`, `wetness`, `feature_type`, tangent fields, and lane fields for animated route-following, lane crossing, wind response, camera-distance fade, and world-aligned elongated Gaussian surfels. Rain uses `feature_type = 4` and Mesh Flow uses `feature_type = 5`; Flow and Field behavior must remain isolated from feature-specific shader/offline branches. Do not rename or reorder these fields without a coordinated serialization, shader, offline renderer, visual preset, and test update.

## Rendering Contract

Point-cloud styles now have `waterStreamOverlay` for generated stream layers. Old `flowAnimation` / `waterPathView` styles remain parseable aliases for compatibility, but new stream overlays should use the v2 water overlay path.

Stream samples render as world-aligned elongated surfels:

```text
long axis  = tangent * stream_world_length
short axis = cross(normal, tangent) * stream_width
normal     = local surface normal
```

Generated Flow, Mesh Flow, Field, and Rain stream layers participate in viewport rendering and the same EXR/MP4 export path as other visible point-cloud sessions. Water streams and effect layers are not exported as PLY unless a future explicit export feature asks for it. Ripples evaluate through sparse base-cloud runtime memberships/params in viewport and offline export. Seepage evaluates compact spatial-grid buffers in the Vulkan point paths. Field Surface Motion evaluates through active-cloud `water_effect_*` composition.

## Visuals Contract

Base cloud visuals are evaluated first. Ripple contributions then combine through sparse runtime evaluation, Seepage combines its role-filtered damp/ripple/glint response from the local spatial cell, and Field Surface Motion contributions combine through Visuals-compatible `water_effect_*` fields. Generated Flow, Mesh Flow, Field Streamline, and Rain overlays keep their own stream scalar fields.

Grouped scene visuals are folder-level. ROCK is the primary visual owner by default, with SAND and VEG receiving mirrored settings. Authored size, opacity, and emission remain expressed against a 1 mm baseline; the renderer applies transient per-role footprint and measured-count coverage compensation to the committed display bundle. Role-specific gates still apply after mirroring: SAND can show shader shoreline waves, VEG can show roughness surface motion, Seepage follows each node's target-role set, and ROCK remains the primary stationary reference.

The Visuals tab's scene-wide **Visible Point Cloud** selector changes only presentation density. The old ROCK/SAND/VEG bundle stays visible until all target roles and their Ripple/Field payloads are ready, then commits atomically. Seepage nodes remain world-space authored state and their compact grids are rebuilt for the committed role layers without copying point indices. Water support caches and generated stream overlays remain untouched. Viewport and offline/export paths exclude CPU-only analysis sources and staged targets.

Layer-linked saved visuals should keep field availability honest:

- Base-cloud visuals can use base scalar fields.
- Ripple visuals use base-cloud Water Effect Stack controls backed by sparse runtime params.
- Seepage visuals use the selected node's shared or local Seepage look rather than scalar fields.
- Flow visuals can use stream scalar fields.
- Mesh Flow visuals can use stream scalar fields and the Mesh Visual trail profile.
- Field visuals can use Field stream/effect fields.
- Rain visuals can use stream scalar fields and the Rain visual profile controls.

When a visual is imported from another layer family, keep it read-only until saved under the active layer with a suffix such as `_baseCloud`, `_ripple`, `_seepage`, `_flow`, `_meshFlow`, `_field`, or `_rain`.

The active base-cloud Water Effect Stack supports add, multiply, max, screen, override, colourise, opacity, size, and emission contributions for overlapping Ripple and Field Surface Motion layers while preserving existing base scalar mappings. Ripple settings should stay parameter-only when membership is current; Field Surface Motion currently updates generated base-cloud composition fields.

## Cache And File Strategy

The current saved/reusable caches are:

```text
<source-stem>-WaterPathCache.json
<source-stem>-WaterFieldCache.bin
```

`WaterPathCache.json` is the saved Flow path cache. For grouped scenes it is stamped against the canonical analysis support set rather than whichever ROCK/SAND/VEG display bundle is selected. Each Flow branch stores a `bake_fingerprint` derived from the emitter and resolved Path profile so incremental Bake Path can safely reuse unchanged branches after project reload. Path settings also serialize `attractor_enabled`, `attractor_position`, and `attractor_strength`. `WaterFieldCache.bin` is derived output for user-authored region Field caches, not a normal project source layer. It stores the support path/signature, field settings fingerprint, region fingerprint, field settings, stale flag, selected region boundary, and serialized field-node records. Region caches are reused when fingerprints match and rebuilt when source support, region geometry/settings, or field settings change. Path-anchor Field caches are currently rebuilt from Flow path anchors and stamped in memory rather than saved as mandatory binary caches.

Seepage does not add a disk cache. Authored nodes and looks are small project/source JSON records; capped surface guides and the role-filtered spatial hash are rebuilt in memory after load, and topology/parameter fingerprints prevent unnecessary GPU uploads or support retracing during look/Rain editing.

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

- Sample scene: when `Data/SampleScene` exists, discovery validates the local ROCK/SAND/VEG sample files, inferred spacing, scalar fields, and grouping.
- Serialization: schema-34 saves include authoritative scene density groups plus Seepage/Ripple/Flow/Mesh Flow/Field/Rain state, keep SAND Shoreline settings in point-cloud style data, and omit Basin/Runoff/Caustic region keys.
- Project ownership: water emitters/sources load from the active project only; new projects are allowed to have none.
- Cache reload: saved Flow path caches and baked anchors reload only when support/settings signatures match; branch bake fingerprints permit per-source reuse when only a subset of emitters changed.
- Density switch: 1/2/3/5 mm display changes leave analysis support signatures and path/field/trail caches unchanged; Ripple memberships stay within the exact committed display cloud, Seepage rebuilds compact per-role grids from world-space nodes, and Field presentation data is spatially remapped.
- Legacy load: old Caustic regions become Ripple `Caustic Lace`; old Basin/Runoff records are ignored.
- Flow: source-specific edits rebuild only changed source branches where possible; global Path settings and attractor edits dirty `WaterPathCache`; Lane and Trail refreshes preserve Trail profile/style state.
- Seepage: placement/movement and fan or role edits rebuild topology; quality/look/rain edits update compact node parameters when topology is unchanged; hash references remain bounded.
- Stream schema: generated stream scalar fields match the exact order above.
- Rendering: `waterStreamOverlay` styles compile and render tangent-aligned surfels.
- Visuals: base-cloud scalar mappings remain intact after creating Ripples, SAND Shoreline waves, Seepage, Flow/Mesh Flow trails, Rain trails, Field Streamlines, and Field Surface Motion; generated stream sessions are hidden from base-cloud look-dev/export visual selection.
- Regions: Ripple and Field regions preserve concave clicked boundaries.
- Motion: Flow, Field Streams, and Rain visibly animate through shader/Visuals playback, not only static generated positions.
- Attractor: `Water path attractor biases a downhill fork without climbing Z` verifies that attractor strength biases the route in XY while path samples continue descending in Z.
- Field cache: region Field caches save, reload, and invalidate on support, region, or settings changes; path-derived Field caches rebuild from Flow path anchors.
- Export: visible generated Flow/Mesh Flow/Field/Rain stream layers, sparse Ripple runtime effects, SAND Shoreline waves, compact-grid Seepage, and active-cloud Field Surface `water_effect_*` fields appear in viewport and camera export paths without requiring water PLY export.
