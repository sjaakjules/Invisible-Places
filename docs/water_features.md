# Water Feature Set Report

## Goal

Water v2 exposes seven active water tabs, ordered by their authored workflow:

- **Ripples**: region-based procedural water effects selected on the canonical analysis support and presented on the committed display cloud. Legacy Caustics load as Ripples with overlay type `Caustic Lace`; the region overlay set also includes a local polygon `Shoreline` pattern.
- **Shoreline**: a separate SAND-role point-cloud shader effect that uses the sand cloud itself to simulate waves washing toward and away from the beach boundary.
- **Seepage**: manually placed surface nodes that select connected downhill ROCK/VEG cache cells, producing persistent dampness, angle-dependent glints, and optional short wetting trickles without generating a stream.
- **Rain**: camera-aware GPU falling particles that slow and squish against averaged cache normals before collision and can drive SAND rings, irregular ROCK wetness, and descending VEG streams, with Light Mist, Rain, and Heavy Downpour intensities.
- **Flow**: downhill emitter paths or manually authored open splines feeding generated world-aligned stream surfels for artist-directed small streams.
- **Mesh Flow**: triangle-mesh-guided sources, attractors, and GPU-previewed surface routes for topology-aware flowing water.
- **Field**: a second point-source/region approach that builds local vector fields to generate Field Streamlines and Field Surface Motion.

The base point cloud remains the visual source of truth. Ripples modify the base cloud through sparse region memberships plus small runtime parameter buffers that the viewport and offline renderer evaluate procedurally. Shoreline modifies the SAND base-cloud draw through point-style shader uniforms and does not allocate a region membership. Seepage evaluates compact node/spatial-hash buffers directly on eligible base-cloud points. Flow, Mesh Flow, and Field Streamlines add derived stream resources rather than source LiDAR. Rain uses a dedicated particle pipeline and compact impact-event buffers instead of a point-cloud overlay. Field Surface Motion currently modifies the base cloud through generated `water_effect_*` fields. None of the active paths should rely on dense permanent scalar fields or newly generated growing point patterns as the primary visual.

Basin and Runoff are removed from the public v2 workflow. Legacy code and tests may still exist for compatibility, but Basin/Runoff tabs, runtime load state, generation, and new project saves are not part of the active water contract. Old Basin/Runoff JSON records are tolerated and ignored.

## Current Status

Implemented in the current repository:

- Active Water tabs ordered Ripples, Shoreline, Seepage, Rain, Flow, Mesh Flow, and Field.
- Project schema `43`, standalone water-source schema `18`, animation schema `11`, nested Rain settings version `3`, and shared-surface binary schema `3`, with authoritative scene density groups, compact cache manifests, manual Flow spline sources, global/per-source Flow-trail visibility, connected Seepage support and live dimensions, per-node reach/width/prominence/wetting keys, Rain near-surface/ROCK/VEG tuning, delayed Seepage Rain response, keyed Rain and Flow levels, selectable Shoreline algorithms, v2 Ripple/Flow/Field settings, and legacy migration.
- Ripple `WaterEffectLayer` records and distinct shader/offline procedural patterns for all `WaterRippleOverlayType` values.
- SAND-role shader Shoreline waves with independent `Foam Fronts (Current)` and `Height Foam` tuning banks. Height Foam adds absolute run-up and break elevations, persistent offshore foam, incoming gather strength, and fading return strength while retaining the shared wave and response controls.
- Visible-depth/cache Seepage placement and editing with connected downhill support selected from the shared 10 mm authored-role cache, ROCK/VEG/SAND role targeting, shared and local look profiles, Wetting Trickle, rain-responsive dampness, compact sparse hashes, and topology-versus-frame-safe-parameter GPU upload separation.
- Sparse Ripple membership uploads for selected region points, with pattern/response edits updating compact GPU params when region membership has not changed. This supports millisecond-scale live modifications instead of CPU-regenerating dense fields.
- Shared region selection for Ripple and Field support, including selected base point indices, edge weights, normals, source scalar values, field vectors, and manual Field control flags.
- Flow path cache reuse, branch hiding, centripetal Catmull-Rom presentation smoothing, and generated stream surfels with the v2 stream scalar contract.
- Ordinary Flow live topology uses per-source GPU route and Trail compute passes over the resident shared surface cache. Revisions coalesce asynchronously, geometrically sized outputs promote fence-safely, and the deterministic CPU builder remains the offline/fallback reference.
- Flow Bake Path performs source-level incremental cache reuse. Branches store per-emitter bake fingerprints, so moving or adding one source rebuilds that source while unchanged source branches are retained.
- Flow Path supports an optional viewport-placed attractor with tunable strength. It biases path ranking in XY while Z remains the vertical downhill axis.
- Flow supports persistent manual path sources authored as open centripetal Catmull-Rom splines. Manual routes bypass `GenerateWaterPathCache`, build stable 3D lane frames directly, and coexist with generated point-source branches.
- Manual paths can use the shared surface guide. New paths enable it by default; it keeps the authored endpoints and route direction, constrains terrain deviation to Lane Cover Width, and blends unsupported sections back to the spline and its rotation-minimizing frame.
- Flow sources have Maximum Flow Strength, Rain Response, and a parameter-only **Show Trail** flag. **Show All Flow Trails** is the global master; effective trail visibility is the conjunction of both flags, while a keyable global Flow Level remains the animation control. Visibility changes do not regenerate routes or upload geometry.
- Project-owned Flow emitters and point sources. Sources are no longer loaded from a separate global save into every project.
- Clean generated Flow branches reload from validated scene-local `.flowpathcache` sidecars when support/settings signatures still match; schema-43 project JSON stores only the compact manifest.
- Combined ROCK/SAND/VEG support sampling for grouped scenes. Flow sources and path baking use CPU-ready canonical ROCK 1 mm, SAND 2 mm, and VEG 1 mm analysis sources as one support surface; changing the visible density does not invalidate support or create a full merged GPU point buffer.
- Field cache, Field Streamlines, and Field Surface Motion built from Flow path anchors or user-authored Field regions, with region Field caches saved and reused offline.
- Field no-flow, bridge-allowed, and bridge-blocked control regions with visible diagnostics.
- Shared animated trail visualization for Flow and Field streams; Flow moves along baked path anchors and Field moves along cached vector-field paths seeded from perturbed source points.
- Rain uses 32,768 persistent GPU particle slots and a dedicated depth-tested streak draw. One shared, persisted 10 mm `WaterSurfaceCache`, deterministically averaged from the exact complete 2 mm ROCK/SAND/VEG bundle, stores ROCK/SAND top surfaces, VEG 3D occupancy, and orientation-independent authored-role support used by Rain, Flow, and Seepage. It warms after the active scene's visible display upload whether the effects are enabled or not.
- Active-cloud sparse runtime Ripple evaluation and `water_effect_*` composition for Field Surface Motion, with Visuals-tab Water Effect Stack controls for both families.
- Viewport/offline/export rendering of water output without requiring water PLY export.
- Legacy Basin/Runoff removal from the active public UI and new-save contract.

## Architecture Map

- `src/water/WaterFlow.hpp` and `src/water/WaterFlow.cpp`: v2 water structs, Seepage look resolution/spatial-grid evaluation, shared region selection, flow path generation, shared stream surfel generation, Field cache persistence/streamline generation, Ripple/Field sparse effect generation, legacy water helpers, and point-cloud conversion.
- `src/app/Application.cpp`: Water panel UI, runtime `WaterWorkflowState`, emitter/node/region editing, Seepage GPU refresh orchestration, source-local Flow/Field orchestration, shared surface-cache warmup, and project/source document wiring.
- `src/serialization/ProjectDocument.hpp` and `src/serialization/ProjectDocument.cpp`: schema-43 project state, schema-18 water-source state, schema-11 animation paths, manual Flow paths, global/per-source trail visibility, per-source profile locks/activity/surface guidance, connected Seepage node dimensions/selection limits and animation factors, selectable Shoreline styles, Rain settings-v3 tuning, legacy migration, and sidecar path-cache persistence.
- `src/water/RainSimulation.*`, `src/renderer/pointcloud/PointCloudPreviewState.*`, `src/renderer/core/VulkanViewportShell.cpp`, and root `shaders/water_surface_*`, `shaders/water_flow_*`, `shaders/rain_*`, and `shaders/pointcloud_*`: streamed shared-surface construction, persisted GPU hashes, nonblocking GPU preprocessing/swap, per-source GPU Flow route/Trail generation, deterministic CPU offline/fallback generation, persistent GPU Rain, role-indexed impacts, water overlays, SAND Shoreline, compact Seepage, and stream surfel shading.
- `tests/AssetDiscoveryTests.cpp`, `tests/SeepageWaterTests.cpp`, `tests/SeepageOfflineRendererTests.cpp`, and `tests/SeepageSerializationTests.cpp`: serialization and legacy migration, Wetting Trickle determinism, connected cache-cell selection, per-node dimension/activity/wetting interpolation, delayed Rain envelopes, bounded Seepage grid/runtime behavior, offline/Fast Basic parity, and shader/style contract coverage.

## Runtime Model

`WaterWorkflowState` in `src/app/Application.cpp` owns the live workflow state. The v2-relevant fields are:

- `emitters`, `defaultSourceSettings`, `tempDefaultSourceSettings`, and per-emitter settings for Flow path generation; `manualFlowPaths` and the transactional manual path editor own authored spline routes.
- `pathCache`, `pathAnchors`, path revisions, dirty flags, and hidden branch IDs for reusable Flow paths.
- Path/Lanes/Trail profiles plus per-source active/pending/retired GPU Flow resources and deterministic CPU fallback artifacts. The existing 31 Trail scalar fields and their exact order are retained for renderer/offline compatibility.
- `rippleLayers` plus sparse runtime memberships/params for current Ripple evaluation. `rippleEffectOverlay` is kept as selected-region debug/evidence data, not as a generated visible Ripple layer.
- Shoreline wave settings live on `PointCloudStyleState` and are mirrored to the SAND role in grouped scenes. The Water > Shoreline panel edits the active LiDAR visual owner, then syncs scene point visuals without touching Ripple region membership.
- `seepageNodes`, `defaultSeepageLook`, and `seepageLookProfiles` own the authored Seepage state; shared-cache-derived connected support selections, retained per-layer sparse grids, topology/parameter fingerprints, frame-safe parameter generations, upload revisions, GPU byte counts, and overflow diagnostics own its derived runtime state. Animation paths own normalized per-node keys, while the Animation panel caches the small delayed Seepage Rain envelope.
- `fieldSettings`, `fieldStreamSettings`, `fieldCache`, `fieldStreamOverlay`, and `fieldSurfaceEffectOverlay` for Field.
- `collisionRainSettings`, `rainVisual`, one shared `waterSurfaceCache`, asynchronous active-scene warmup/preprocess state, scene manifest status/generations, and neutral Water-surface cache/upload diagnostics. Runtime cache types and renderer entry points use `WaterSurface*` names; Rain naming remains only in legacy file migration and Rain simulation behavior.
- `activeRegionFeature`, `regionEditor`, and placement flags for editable Ripple regions and legacy-safe region editing.

Generated water overlay sessions are excluded from support-layer discovery and from base-cloud look-dev/export visual selection. They are renderable water output, not source LiDAR layers for future water bakes. Ripples no longer create active visible `-Ripples.generated` sessions; their display-source-specific membership and procedural params are uploaded to the committed base-cloud renderer instead. Shoreline waves are not generated sessions either; they are evaluated during the committed SAND point-cloud draw. Seepage likewise remains on the base draw and uploads static connected-cell topology, occupied hash cells, bounded node references, and compact frame-ringed parameters. Field Surface Motion contributes to display-cloud composition through spatially remapped `water_effect_*` fields, while Flow trails, Mesh Flow, and Field Streamlines remain derived stream output. Rain is rendered by its dedicated particle resources while Rain collision, Flow guidance, and Seepage support share the same immutable scene cache.

For grouped LiDAR scenes, the selected three-role display bundle loads and becomes visible first. Canonical analysis roles load on demand for explicit Bake Path and analysis-based Ripple/Field work; Seepage placement uses visible depth plus the resident shared cache. The renderer uploads and draws only committed display roles, and staged switch targets are not renderable. This canonical analysis set is distinct from the shared surface-cache input: after the visible display upload completes, the cache streams each file in the exact complete 2 mm bundle once (or one nearest complete fallback bundle) without loading it as display or analysis data.

A display switch performs no synchronous Ripple, Field, Flow, or Seepage topology rebuild. Ripple may restore an exact-source membership cache after commit; otherwise its state remains dirty until explicit recalculation. Field presentation is likewise display-source-specific and never reuses analysis point indices across variants. Seepage retains its world-space nodes and display-independent connected cache-cell topology, while role filtering and Auto quality use the current render layer. Generated streams, Rain, and the shared surface cache remain independent of display density.

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

Ripples exposes region/layer controls and procedural overlay settings. Shoreline owns the SAND boundary-wave look, with a dropdown for the unchanged Foam Fronts algorithm and the independently tuned Height Foam algorithm. Seepage exposes viewport node placement/selection/movement, live Reach/Width/Prominence, topology-only selection limits and role targeting, the Legacy Ripples/Wet Rock Sheen/Chaotic Bloom/Wetting Trickle selector, common moisture and response controls, project scenarios, shared look profiles, and local saved/temporary overrides. Every Seepage parameter name has a hover tooltip, and only the controls used by the selected pattern remain visible. Rain exposes enablement, Light Mist/Rain/Heavy Downpour intensity, continuous amount, count/density/fall/drop visibility controls, dedicated visual presets, seed, wind/turbulence/gust/front controls, spawn bounds, camera death distance, and parameter-only Near Surface, ROCK Impact, and Vegetation Impact tuning. Its master Impact Effects toggle and SAND Rings, ROCK Wetness, and Vegetation Twinkle switches are also uniform-only edits. The Water Surface Cache section reports source spacing, Missing/Stale/Building/Valid/Failed state, requested/built generations, occupancy, GPU memory, revision/upload counts, hash diagnostics, and preprocessing. **Rebuild Cache** advances the requested generation while retaining the settled cache until replacement. Flow exposes **Show All Flow Trails**, per-point/path **Show Trail**, point-path baking, branch hiding, manual spline creation/editing, per-source strength/Rain response/effective activity, manual Use Surface Guide, source profile assignments, Trail styling, and a Show Source Nodes / Paths toggle for clearing blue authoring guides from the trail view. The topology-changing lane switch is labelled **Generate Trail Geometry** in Advanced UI. Mesh Flow exposes mesh-guided sources, attractors, particle state, preview/final route controls, and mesh trail styling. Field exposes field build settings, stream settings, surface-motion output controls, and user-authored Field regions.

The Animation panel uses the same global water scenario selector and the active animation scrub position. Its global water-track controls add or update a normalized key, capture the current water state, remove the selected key, set Seepage, Rain, or Flow off, edit Flow Level and Seepage Rain Delay/Rise/Recession, choose Smooth/Linear/Hold outgoing interpolation, and copy complete water tracks from another animation without copying its duration or camera keys. A Per-Node Seepage Timing section keys Activity, legacy Local Spread, Wetting Front, Reach Scale, Width Scale, and Prominence for a stable node ID, with Set Node Off, Start Wetting, and Fully Wet shortcuts. Key time is shown and edited in seconds but remains normalized in storage.

## Serialization Contract

Project documents now use schema `43`. Standalone water-source documents use schema `18`, animation paths use schema `11`, and nested Rain settings use version `3`. The authoritative grouped-density record and scene cache manifest are:

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
  water_surface_cache
    relative_path
    cache_schema
    algorithm_id
    source_fingerprint
    payload_bytes
    checksum
    requested_rebuild_generation
    built_rebuild_generation
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
water_manual_flow_paths
water_seepage_nodes
water_seepage_default_look
water_seepage_look_profiles
water_scenarios
selected_water_scenario
water_source_settings
water_path_cache_manifest
water_path_profiles
water_lane_profiles
water_trail_profiles
water_show_flow_trails
water_ripple_layers
water_field_layers
water_flow_stream_settings
water_field_settings
water_field_stream_settings
water_rain_settings
```

The group manifest points to schema-3 `.surfacecache` data; the water-scene manifest points to a validated `.flowpathcache`. Clean generated branches are written atomically by explicit Bake Path or settled project Save, with scene-local storage preferred and project-local hidden fallback. Filenames derive from the complete serialized payload checksum, including hidden branches and analysis state; only the checksum-bearing project manifest is authoritative on reload. Live Trail/style edits do not write sidecars, and stale or orphaned derived arrays are omitted from project JSON.

Project saves also preserve `water_animation_trail_settings`, `water_animation_trail_profiles`, and caustic look settings for legacy animation/visual compatibility, even though those are no longer standalone Water tabs.

Shoreline wave settings are serialized with the point-cloud style rather than in `water_ripple_layers`. The active style keys include `shoreline_wave_enabled`, `shoreline_boundary_z`, `shoreline_height_reach_meters`, `shoreline_edge_fade_meters`, direction, pattern scale, wavelength, speed, warp, turbulence, density, phase, intensity, response, tint, and seed fields.

Seepage nodes are stored both in the active scene state and in the project compatibility mirror. Each node preserves its world position, surface normal, down axis, live reach/width/prominence, topology-only selection reach/width limits, feather/depth tolerance, normal alignment, strength, seed, viewport/export flags, target roles, assigned profile, committed local override, and temporary local override. Schema-17 fan widths migrate with `width = max(startWidth,endWidth)`, `selectionReachLimit = 1.875 * reach`, and `selectionWidthLimit = 1.62 * width`. Shared defaults and named profiles use `water_seepage_default_look` and `water_seepage_look_profiles`. Every stored look includes its pattern, Wetting Trickle patch/length/width/front-softness values, and the corresponding legacy, noise, reflection, and response parameters.

Project-owned `water_scenarios` preserve stable IDs, names, complete base looks, Seepage level/spread, continuous Rain level, global Flow level, and Seepage Rain delay/rise/recession; `selected_water_scenario` selects the preview override or remains empty for Authored Node Looks. Animation-path schema `11` stores `selected_water_scenario_id` and normalized `water_scenario_tracks`. Every global key contains a complete look snapshot, Seepage level/spread, continuous Rain and Flow levels, Seepage Rain timing, and outgoing interpolation. Each track can also contain `seepage_node_tracks`, keyed by stable node ID, whose normalized keys store Activity, legacy Local Spread, Wetting Front, Reach Scale, Width Scale, Prominence, and outgoing interpolation. Schema-10 tracks remain readable and default missing new factors to one. Tracks embed a fallback scenario definition so rendering remains reproducible when their linked project scenario is missing.

`water_sources.json` schema `18` mirrors the active source/layer/settings subset for reusable water setup, including manual Flow paths, global/per-source trail visibility, per-source profile locks/activity/surface guidance, connected Seepage node dimensions/selection limits and looks, and Rain settings-v3 tuning. Schema-11 and older files load with an empty manual-path list.

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
- Existing embedded `water_path_cache` records are accepted as migration input when their fingerprints match, then externalized on a settled schema-43 save.
- Water emitters and point sources are stored inside the project document. Loading another project should not import sources from a previous project unless that project explicitly contains them.
- Schema-33-and-earlier documents without Seepage keys load empty node/profile lists. A new project uses the restrained Chaotic Bloom default; a legacy saved look with no `pattern` field is explicitly restored as Legacy Ripples so its previous appearance is retained.
- Schema-40 projects and schema-16 water-source files load missing Wetting Trickle controls at their defaults. Schema-9 animations without per-node tracks retain Activity 1, Local Spread 0, and a fully advanced Wetting Front; missing Seepage Rain delay/rise/recession remains immediate. Unknown pattern names still fall back to Legacy Ripples.
- Projects before schema 40 and standalone water-source documents before schema 16 load existing manual paths with Use Surface Guide disabled; newly authored paths enable it. Missing source activity values load as Maximum Flow Strength 1 and Rain Response 0.
- Animations before schema 9, or tracks without a stored Flow level, default to full Flow (`flowLevel = 1`). Animations without water scenario tracks continue to use the current static project water state.

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

Shoreline does not create `WaterEffectLayer` records, does not build region previews, does not emit `water_effect_*` or `ripple_*` scalar fields, and does not create a generated point-cloud session. It is saved as point-cloud style data and is rendered by viewport/offline point-cloud shaders wherever the point style is active.

## Seepage

Water > Seepage represents persistent damp areas rather than full streams. `Place Seepage Node` uses visible-scene depth plus the resident shared cache and stores the snapped position and averaged normal; canonical analysis clouds are not required. In a grouped scene the builder snaps to the nearest permitted 10 mm authored surfel, traces a maximum-downhill centreline, and floods connected same-surface cells at or below the node within explicit selection reach/width limits. Disconnected or background cells and generated Flow layers are rejected. VEG cells associate with the connected ROCK substrate; SAND participates only when explicitly enabled. The visible high-density cloud is never scanned for Seepage membership.

Reach, Width, Prominence, edge feather, surface-depth tolerance, normal alignment, strength, and deterministic seed remain authored live controls. Selection Reach Limit and Selection Width Limit explicitly bound topology. Nodes can be selected in the viewport or list, moved back onto support, renamed, role-targeted, hidden from the viewport or export, and deleted. ROCK and VEG are enabled by default; SAND remains an explicit option. Visibility remains a parameter-only enabled factor, so off/on retains settled support and descriptors.

The look separates authored topology from live visual parameters. Auto/Low/Balanced/High quality, Base Wetness, Coverage, Glisten, Rain Response, Prominence, colour/opacity/point-size/emission response, and the shared add/max/multiply/screen/override blend modes are common to every pattern. Pattern-specific controls are shown only when relevant:

- `Legacy Ripples` preserves the original warped sinusoid and its Wavelength, Pattern Scale, Warp, Turbulence, Speed, and Phase controls.
- `Wet Rock Sheen` samples deterministic seed-rotated world-space 3D gradient noise. Patch Size, Contrast, Evolution, Roughness, Angle Response, Micro Detail, Glint Density, and a virtual environment azimuth/elevation shape persistent damp patches, analytic micro-normal variation, broad grazing-angle sheen, and sparse glints.
- `Chaotic Bloom` domain-warps ridged 3D gradient noise along connected downhill metrics. Feature Size, Curl, Breakup, Downhill Drift, Evolution, and the same reflection controls make irregular lobes split and reconnect without regular bands. A restrained form of this pattern is the new-project default.
- `Wetting Trickle` reveals small seeded saturated patches near the node, advances a soft irregular Wetting Front over downstream distance, and leaves short narrow downhill fingers plus persistent angle-dependent damp sheen behind it. Patch Size, Trickle Length, Finger Width, Front Softness, Breakup, Downhill Drift, Evolution, and the reflection controls tune the effect without generated liquid geometry.

The Seepage panel hides controls that do not apply to the selected pattern. Hovering a visible parameter name shows a tooltip describing its visual role, units, and whether it is a topology or parameter-only edit.

A node can use Default, a named shared profile, a committed local override, or a temporary edited override. Saving a look updates parameters without changing connected support. If a point normal is unavailable or unreliable, both the shader and offline evaluator use the packed cache-cell normal; analytic noise variation keeps the reflection from becoming uniform.

Connected support is retained by scene group, authored terrain role, surface-cache identity, and authored topology fingerprint, not display filename or session index. Each 16-byte support reference stores a node index, downstream/lateral metrics, packed normal, role, confidence, and flags. `BuildWaterSeepageSpatialGrid` resolves profiles and Auto quality and builds a cache-resolution sparse hash. Each node is capped at 262,144 references and each occupied cell at eight node references; overflow keeps the last settled node topology and reports that its authored limits must be reduced.

`VulkanViewportShell` separates immutable connected support from animated node parameters. Placement, roles, selection limits, depth/edge constraints, or cache-identity changes rebuild only the affected node asynchronously, retain settled output while pending, and replace compact buffers only after success. Reach, Width, Prominence, look, reflection, quality, strength, visibility, scenario, per-node tracks, Wetting Front, and Seepage Rain changes touch one compact parameter record per active node and retain support/hash/descriptors. Parameter snapshots are copied into a buffer for each frame in flight only when that frame slot is safe, with a separate EXR parameter buffer. Point shaders perform one cell lookup, exit when node activity is zero, compare packed downstream/lateral metrics with live thresholds, and evaluate the selected appearance on the GPU. No display-density-specific Seepage point index is allocated.

During a change between two pattern algorithms, a deterministic point-cell selector cross-dissolves spatial coverage while evaluating only one algorithm for each point. Low quality uses one noise scale and inexpensive glints, Balanced adds a warp/detail scale, and High evaluates the complete stack. The CPU/offline path uses the same seed rotations, noise gradients, guide normals, real camera position, virtual environment direction, reflection response, quality choice, and pattern transition as the point shaders.

The Seepage Nodes section provides a transient `Show Structure Overlay` viewport option. It defaults off so the authored node points and actual wet points remain visible without diagram clutter. Turning it on draws the actual selected cache-cell footprint, downhill connectivity, role/confidence diagnostics, and support warnings; the selected node is emphasized. Node markers and the selected-node label remain visible in both states. This is an editor overlay only: it is not serialized and does not alter GPU Seepage topology, look parameters, stills, or animation exports.

Auto quality resolves High below 10 million effective point invocations, Balanced from 10 through 50 million, and Low above 50 million; World Surfels count as six invocations per point and an explicit quality remains fixed. Scenario spread and per-node Reach/Width factors change live thresholds only inside authored selection limits, so neither animation nor Rain response rebuilds connected support or the spatial grid.

Two editable project scenarios are seeded. `Pre-Colonisation Wet` uses Chaotic Bloom at full Seepage level with broader spread, wetter coverage, stronger evolution and reflection, and high Rain Response. `Contemporary Managed` uses the same nodes, seeds, noise coordinates, approximately `0.20 m` features, and `225 degrees / 55 degrees` environment direction, but halves the level, narrows spread, lowers damp coverage and glisten, and responds much less to Rain. `Authored Node Looks` restores the normal profile/local-look hierarchy.

Water animation keys use normalized `0...1` positions, so duration, frame-rate, and camera-key edits do not move their timing. Smooth interpolation is the default; Linear interpolates directly and Hold retains the source key. Reflection azimuth follows the shortest arc, endpoint values hold outside the keyed interval, and adding within `0.0001` replaces the existing key. Scenario overrides retain node placement, role filters, seed, authored strength, and enablement. Missing linked scenarios use the track's embedded snapshot with a warning.

Each water scenario track can also contain a sparse track for each stable Seepage node ID. Its normalized keys interpolate **Activity**, legacy **Local Spread**, **Wetting Front**, **Reach Scale**, **Width Scale**, and **Prominence** independently. Activity multiplies authored node strength and reaches a shader early-out at zero; Reach/Width compare against already selected support; Wetting Front reveals saturated patches and downhill fingers; Prominence scales the live response. Schema-10 tracks retain Local Spread and default the new factors to one. These keys never rebuild support, hash cells, descriptors, point membership, or generated geometry.

Keyed Rain is continuous: zero hides Rain and contributes no Seepage or Flow rain gain, while increasing values reveal a deterministic fraction of the persistent maximum Rain particles and scale visibility, energy, and response. Seepage adds Delay, Rise, and Recession timing without delaying or filtering the visible Rain draw. A deterministic 120 Hz moisture envelope is rebuilt only when its global water keys, timing, linked scenario, or animation duration change; scrubbing and export interpolate this small table, so delayed groundwater response has no history-dependent per-frame simulation. Keyed Flow independently reveals a deterministic subset of prebuilt source Trails. Preparing scenario playback settles those maximum fixtures once; scrubbing and playback do not regenerate routes, rebuild connected Seepage support, or upload the shared surface cache.

Export jobs freeze the animation track and per-node tracks, delayed Seepage Rain envelope, linked/fallback scenario, Rain settings, Flow source activity/visibility, Seepage topology, and base styles at queue time. Screen Sprites, World Surfels, Fast Basic, viewport EXR, stills, animation/video export, and the CPU/offline evaluator consume the same support metrics, pattern, camera/environment reflection, scenario, node-key, and moisture-envelope state. The scenario name remains appended to animation output filenames.

Seepage creates no generated PLY or growing point overlay: nodes/default/profile/scenario state serialize in project data and nodes/looks serialize in `water_sources.json`, while connected support, the spatial grid, and GPU buffers are transient and derived after load.

Native SampleScene and Scene1 integration smokes exercise visible-depth/cache placement, connected-only selection, actual-cell Structure Overlay diagnostics, compact GPU upload, and parameter-only reach/width/prominence/activity/wetting/Rain changes. They also verify that a 120-second state cycle does not scan cache sources, rebuild support, upload topology, attach descriptors, or create density-specific topology.

### Future improvement: optional mesh-assisted connectivity (not implemented)

Continuous triangle-mesh Seepage routing remains a roadmap option. The current grouped-scene implementation already obtains explicit connectivity from authored 10 mm cache cells, including vertical/overhanging 3D support, so a mesh would be justified only for carefully validated gap bridging. It must preserve cache roles as classification truth, remain asynchronous and scene-shared, and never become a per-rendered-point query or display-density-specific allocation.

## Shared Water Surface Cache

Rain, surface-guided Flow, and connected Seepage support consume one scene-owned `WaterSurfaceCache`; there is no second Flow/Seepage point-cloud scan or duplicate surface allocation. Source selection chooses the exact complete 2 mm ROCK/SAND/VEG bundle as one unit. Scenes without it use the nearest complete bundle with a diagnostic; role spacings are never mixed. Each selected PLY is streamed once without constructing a full `LoadedPointCloud` or loading it as display/analysis data. Positions quantize to 10 mm cells and normals are deterministically hemisphere-aligned before averaging. The algorithm identity is `water-surface-10mm-normal-average-v1`.

The one 10 mm accumulation produces three compatible views:

- Rain ROCK/SAND sparse XY cells retain the highest occupied 10 mm 3D cell, exact maximum height, averaged normal, confidence, and sample count.
- Rain VEG sparse 3D voxels retain foliage occupancy and an averaged normal.
- Flow/Seepage authored-role sparse 3D surfels retain the sub-cell centroid, role, hemisphere-aligned normal, sample count, confidence, normal coherence, roughness, and normal variance. Separate 3D cells preserve vertical and overhanging sheets that an XY height field would collapse. Flow queries them while building routes; Seepage uses their connectivity to derive compact support-cell metrics.

Binary cache schema 3 (`IPWSC003`) persists the scene-relative source signature, aggregate arrays, all three GPU-ready power-of-two hash tables, and a streaming payload checksum. Its identity trailer can be verified during the same sequential read, so a warm launch does not rescan PLY payloads, reconstruct tables, or perform a second whole-payload hash. The signature covers source-relative files, roles, spacings, transforms, file sizes/timestamps, algorithm ID/schema, and 10 mm resolution while excluding all Rain, Flow, Seepage, visibility, animation, and visual parameters. New caches live at `<scene>/.invisible_places/cache/water/<signature>.surfacecache`, with project-local hidden fallback when scene storage is unavailable. Old sidecars remain recoverable but are stale under the new algorithm fingerprint. Payloads above 5 GiB remain runtime-only and do not retain a misleading manifest path.

The active grouped scene starts warming only after its selected display bundle is uploaded, even when Rain, Flow, and Seepage are disabled. Point-cloud loading, surface build/load plus GPU preprocessing, and dynamic-mesh cache warmup share one exclusive high-memory slot; the surface cache takes priority after display commit and releases the slot only when preprocessing completes. The last settled cache remains resident until a replacement or active-scene change. `UploadWaterSurfaceCache` ignores a repeated ready or pending revision, streams persisted GPU-table sections through one reusable staging allocation, verifies the incremental checksum/trailer, uploads immutable tables once, and submits one preprocessing dispatch before promotion. `WaterSurfaceFlowView()` exposes the filtered table directly to GPU Flow route generation. Deterministic CPU Flow fallback and connected Seepage selection use retained aggregates; point shaders receive only compact node-local support.

Upload does not call a device-wide `WaitIdle()`. One reusable mapped staging allocation, capped at 64 MiB, streams chunks into device-local Rain and Flow tables before one preprocessing dispatch with explicit transfer-to-shader barriers. The staging allocation is released immediately after transfer; the CPU copies of GPU-ready tables are released after installation. The old descriptor set and tables remain live until preprocessing completes, and prior resources retire only after their frame fences signal. Rain descriptors and `WaterSurfaceFlowView()` then reference the same immutable scene-cache resource set.

The neutral runtime API is `WaterSurfaceCache`/`WaterSurfaceSource`, `Build`/`Save`/`UploadWaterSurfaceCache`, `WaterSurfaceFlowView()`, and `WaterSurfaceUploadRevision()`; old Rain-cache runtime aliases are not the active contract. Each schema-43 scene manifest records path/size/checksum/source fingerprint and requested/built rebuild generations; runtime derives Missing/Stale/Building/Valid/Failed status from it. The panel exposes those values and **Rebuild Cache**. Rebuild increments the requested generation and keeps the settled GPU resource active until the replacement finishes; Valid is published only after preprocessing, and Failed waits for another explicit request instead of looping every frame.

## Flow

Flow is the point-source/emitter path approach for small streams over the site. It exposes three profile-backed setting areas: Path, Lanes, and Trail, and keeps the existing point-source path bake model:

1. Emitters and source/path settings define bake inputs.
2. `GenerateWaterPathCache` creates or refreshes `WaterPathCache`; Bake Path may merge unchanged cached branches with newly rebuilt branches for only the dirty emitters, then queues Lane/Trail generation instead of synchronously rebuilding and uploading the combined overlay.
3. `BuildWaterPathAnchorsFromCache` rebuilds visible anchors and applies hidden branch IDs.
4. The active runtime gives each source an independent GPU resource. `UploadWaterFlowGpuSource` accepts sampled point-source anchors or manual centripetal-Catmull control points, a source ID/revision, Lane/Trail settings, and Use Surface Guide. It uploads only that compact input and leaves the shared cache and every other source unchanged.
5. `water_flow_routes.comp` writes surface-conforming per-lane route anchors, then `water_flow_trails.comp` writes animated Trail points and the existing field-major 31 Trail scalar slots. `BuildFlowTrailOverlayFromPathAnchors` and `BuildWaterTrailOverlayPointCloud` remain the deterministic CPU reference/offline fallback and use the identical names/order.

Manual path sources are a second Flow source type. Add Path Source starts a draft; Ctrl+left-click adds nodes on the active runtime support surface. The draft and committed route are open centripetal Catmull-Rom splines passing through every control point, with the first point marked as animation start. Save Path requires at least two distinct nodes. Live generation sends those ordered control points to the GPU Catmull-Rom route pass; `BuildManualFlowPathAnchors` remains the deterministic rotation-minimizing CPU validation/offline path. Neither path calls `GenerateWaterPathCache`, seeds Field, or participates in Mesh Flow. Cancel discards the draft without changing the committed source or its derived trail output.

During editing, click a node to select it. Red, green, and blue arrows constrain movement to world X, Y, and Z; the XY/XZ/YZ squares constrain movement to those world planes; dragging the center uses a camera-facing plane fixed at drag start. Delete or Backspace removes the selected node while the viewport has keyboard focus. Double-clicking near the spline inserts a node into the clicked control segment. Spline/node/gizmo hits take editor priority, while empty-space drag keeps ordinary orbit/pan/zoom and double-click away from the spline keeps camera-pivot behavior. Outside editing, the authored guide is shown only for the selected source or in Path View and may be hidden together with point-source markers using Show Source Nodes / Paths. The active draft and gizmo remain visible while editing.

Point and manual sources share the persistent source-ID namespace and may use different Lane and Trail profiles. Manual anchors stay separate from `WaterPathCache` and generated-path analysis; stale generated caches therefore cannot affect manual lane frames. Lane assignment proceeds deterministically from the centre outward, so profiles with enough trails populate every requested lane even without generated-path analysis. Generated point-source anchors use the same centripetal Catmull-Rom presentation interpolation after their profile smoothing pass, reducing angular path and trail segments without changing cached raw branches. Saving a manual edit refreshes trails in memory, while Bake Path continues to rebuild only point-source branches. After project or standalone-source load, valid manual routes regenerate once committed display/shared-surface support is ready, including manual-only projects.

`Use Surface Guide` is stored on each manual path. New paths enable it; paths loaded from project schemas before 40 or standalone source schemas before 16 leave it disabled so an old shot does not change shape unexpectedly. With guidance enabled, the authored spline remains an art-directed corridor: its first point still defines animation start, endpoints remain fixed, the route cannot reverse, and displacement cannot exceed Lane Cover Width. Each sampled interior point queries the continuous ROCK/SAND sheet nearest the prior route frame, flips a candidate normal into the prior hemisphere, projects gravity through that surface, and blends by confidence, coherence, and roughness. Unsupported sections return smoothly to the authored spline and rotation-minimizing frame, including vertical and overhanging routes.

The Lane profile adds `Surface Follow` (default `0.85`), `Downhill Pull` (`0.35`), `Terrain Width Response` (`0.65`), and `Turbulence Scale` (`0.18 m`). Surface Follow blends the 3D support correction, Downhill Pull biases the bounded transverse correction toward projected gravity, and Terrain Width Response bunches lanes on rough/weak support while never exceeding the requested cover. Turbulence is deterministic two-octave noise evaluated from world-space arc distance rather than sample index; its amplitude derives from lane span, not Trail sprite width, so changing point spacing does not change the wiggle pattern. Existing Turbulence, Looseness, Crossing, and Path Attraction retain their roles.

Each point-source Path, Lane, and Trail assignment, and each manual-source Lane and Trail assignment, has a profile lock beside its selector. Unlocked (`Live Edits`) sources use the matching profile's current unsaved edit when one exists and otherwise fall back to its saved settings. Locked (`Saved (Locked)`) sources always use the saved profile. A `Global` assignment follows the currently selected profile under the same rule. Locks are source-owned and persist in project, per-scene, and standalone water-source state.

Lane and Trail topology regeneration is source-local asynchronous GPU work. Each source keeps an active output while one pending output computes. If more live edits arrive, only the newest request is retained; when the fence signals, a result older than that queued revision is discarded and the newest request is dispatched. A settled revision atomically promotes its position, normal, colour, and 31-field scalar buffers, refreshes descriptors per safe frame, and retires the prior output only after all referring frame fences signal. Output capacity grows geometrically and does not shrink while it can satisfy later edits. Deleting a source marks only that resource inactive and defers destruction across its compute fence and frames in flight, without rebuilding paths, other sources, profile groups, or the shared cache.

Edits are classified by cost:

- Appearance, opacity, emission, activity, Maximum Flow Strength, Rain Response, speed, width, and visible length update small mapped style/uniform state only.
- Control points, Lane count/cover, Surface Guide, and geometric turbulence upload only source-local input/settings and enqueue its two compute passes. Generated positions, normals, route anchors, and scalar arrays remain GPU-resident.
- Point-source Path inputs still require Bake Path. Manual routes remain separate from `WaterPathCache`, and cache resolution/support-source changes are startup-cache concerns rather than animation-keyable properties.

`WaterFlowGpuSourceState` reports source ID, requested/completed revision, shared-surface upload revision, cumulative transferred bytes, compute-dispatch count, output capacity, active point count, pending state, and whether Surface Guide was actually available. Live rendering and GPU export consume the settled resident output. CPU-only offline work freezes or builds one deterministic settled snapshot and never regenerates source geometry per animation frame.

A pending Path rebake does not invalidate the last baked route for Lane/Trail-only edits. Assigning a Lane or Trail profile to one source continues to display the existing route snapshot, rebuilds that source's derived trails, and reuses unchanged source artifacts; Bake Path is only required to apply actual Path-input changes.

Path-affecting changes dirty the point-source path cache. Point-source movement, insertion, and deletion are tracked by emitter ID, so the next Bake Path rebuilds only the changed sources when the existing support signature still matches. Global Path profile edits, support setting changes, and attractor edits dirty all current point-source paths because they can change every route. Lane topology changes refresh from existing anchors without dirtying `WaterPathCache`; uniform/style changes do not regenerate Trail samples. Hiding or restoring a Path View branch rebuilds only its lightweight guide immediately and leaves the last settled trails visible while the source-local replacement completes, avoiding a full CPU scan and visibility-index upload. Trail colour, opacity, and emission are owned by Water > Flow, not the Visuals tab.

The Path attractor is an optional global Path setting. `Place Attractor` uses the same detailed grouped-scene support picking as source placement and shows a viewport marker/preview. `Attractor Strength` ranges from 0 to 1; it nudges candidate ranking toward the attractor in horizontal scene coordinates while the downhill score and Z-drop acceptance rules keep Z as the vertical descent axis. The attractor position is included in focused support sampling and bake fingerprints, so moving it causes a correct path rebake instead of a stale anchor refresh.

Flow trails replace legacy trail particles as the primary visible water output. The old generated water PLY workflow is no longer required for viewport, EXR, or MP4 water visuals.

Every point and manual source stores `maximumFlowStrength` (default 1), `rainResponse` (default 0), and `showTrail` (default true). Projects and standalone source files also store the global `showFlowTrails` master, defaulting to true for legacy data. Scenarios and animation keys add global `flowLevel`, defaulting to 1 for old documents. Effective source activity is:

```text
activity = clamp(
    maximumFlowStrength *
    (flowLevel + (1 - flowLevel) * rainLevel * rainResponse),
    0,
    1)
```

The trail draw is eligible only when `showFlowTrails && source.showTrail`; viewport and export snapshots apply the same gate. Toggling it does not clear routes, guides, path caches, or generated Trail geometry. The lane `Enabled` topology switch is exposed as **Generate Trail Geometry** under Advanced so it cannot be mistaken for visibility.

The maximum Trail count is built once. Activity reveals a deterministic monotonic subset using the existing Trail seed with a small soft threshold, while scaling opacity/emission from 30% to 100%, width from 65% to 100%, speed from 60% to 100%, visible length from 55% to 100%, and lateral micro-motion up to 15% of Trail width. `Flow Off`, `Flow Level`, source strength, source Rain Response, and effective activity are visible in the UI. A 120-second Rain/Flow/Seepage key sequence therefore changes only per-source style state during playback and scrubbing: it performs no Flow geometry rebuild, scalar upload, shared-cache upload, or point-cloud upload.

Flow Trail geometry stays static while shader/offline playback derives animated age from `point_age`, `point_seed`, `stream_speed`, effective activity, and render time. Beauty, Surfel, Fast Basic, GPU export, and CPU offline evaluation apply the same deterministic activity gate and buildup scales.

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
Field Surface Motion uses on-demand canonical analysis support, then spatially remaps and pre-composes `water_effect_*` fields on the committed display cloud. A display switch does not run that solve; exact-source cached presentation may be restored, otherwise Field stays dirty until explicit recalculation.

Field should continue moving toward the Ripple performance pattern where practical: region-bounded support should be reused aggressively, uploads should be limited to selected/cache nodes rather than full-cloud fields, and shader/offline-side procedural evaluation should be preferred for editable visual parameters. Field cache generation can remain CPU-side while region selection, Field Surface Motion, and stream styling should avoid whole-cloud recomputation when only visual or playback parameters change.

## Rain

Rain simulates falling water independently from Flow/Field trail generation. Water > Rain owns `RainRuntimeSettings` and `WaterRainVisualSettings`; enabling it advances persistent particles in compute and renders each active drop as a depth-tested six-vertex velocity-aligned streak. Changing amount, count, density, size, opacity, emission, speed, wind, gusts, fronts, spawn bounds, seed, or impact controls updates frame-ring uniforms without route regeneration, `WaitIdle`, CPU point lookup, or buffer reallocation.

Rain reads the Rain view embedded in the shared cache described above; it does not own a separate lifecycle or upload. ROCK and SAND occupy a sparse XY hash with separate top heights, 2 mm-source-averaged normals, and confidence. VEG occupies a sparse 3D hash so a slanted drop can strike foliage at different elevations. Both GPU tables use power-of-two open addressing, no more than 65 percent occupancy, and bounded probes. Each particle performs one bounded 10 mm look-ahead trace and chooses the earliest VEG, ROCK, or SAND hit. Hit distance slows the visible approach while preserving unslowed velocity for impact energy; proximity and packed normal reuse the fixed 64-byte particle ABI. A hit always stops and deterministically respawns the drop; disabling effects never disables collision.

The shared GPU runtime keeps 32,768 particle slots, a 65,536-event ring, and a camera-centred 256x256 impact grid allocated. Active events are binned into separate SAND, ROCK, and VEG references with caps of 8, 8, and 4 per cell. A displayed point checks only its own role list in one directly indexed grid cell. Overflow saturates visually and increments diagnostics rather than increasing shader loop bounds. With Impact Effects disabled, event creation and binning are skipped and point shaders take an early-out.

Approaching streaks shorten and widen into ellipses tangent to the averaged hit normal. The static Near Surface controls default to Approach Distance `0.18 m`, Minimum Speed `0.30`, Squish `0.65`, and Normal Alignment `0.75`; viewport and offline rendering use the same orientation and dimensions.

SAND impacts produce short expanding rings at the real collision position using the Rain Rings kernel. ROCK impacts keep the reduced two-thirds planar area and downhill drift, add seeded inward-only angular notches, radial centre falloff, and stronger/later-fading response below the hit. Static ROCK controls default to Edge Breakup `0.35`, Spread Speed `1.60`, Centre Falloff `0.65`, Height Bias `0.75`, and Persistence `1.35`. VEG impacts now begin with a visible nearby crown, then use deterministic gravity-led hop anchors and descending wandering streams with delayed twinkle pulses and wider sparse-point support. VEG defaults are Twinkle `1.80`, Propagation `0.65 m/s`, Hop Spacing `0.07 m`, Stream Width `0.010 m`, and Stream Spread `0.65`. Intensity, drop width/speed, and impact energy continue to scale effective ROCK footprint and role response without changing fixed capacities or gathering point indices on the CPU.

Rain visual presets `Rain Mist`, `Rain Fine Lines`, and `Rain Downpour` control colour, width, streak length, softness, opacity, emission, and screen-pixel limits. Light Mist, Rain, and Heavy Downpour apply non-destructive density, speed, dimension, visibility, effect-energy, and wind-response multipliers over the selected visual. The continuous scenario `rainLevel` scales the active weather from 0 to 1, while moving world-space fronts and gusts make intensity uneven across the site.

The software fallback uses `OfflineRainSimulationState`: it advances one deterministic `RainSimulator` and builds one role impact grid per output frame, then shares the immutable particle/event frame across every tile. It does not re-simulate rain or rebuild events per tile.

Native GPU Rain validation remains available through `--gui-smoke rain-gpu-sample-scene` and `--gui-smoke rain-gpu-scene1-3mm`. Cross-feature integration uses `--gui-smoke water-integration-sample-scene` with `Saved/validation/SampleSceneValidation_project.json` and `--gui-smoke water-integration-scene1-top-view` with `Saved/exhibitionScene_project.json`/`Top_View`. These scenarios verify display-first startup, one shared-surface upload, responsive edits, and export sampling without per-frame topology work.

## Trail Surfel Scalar Contract

Generated Flow trails, Mesh Flow trails, and Field Streamlines must expose these existing 31 scalar fields in this exact order:

```text
trail_role
trail_id
source_id
path_id
branch_id
trail_seed
point_seed
trail_distance
trail_length
route_start_index
route_point_count
route_length
trail_start_phase
trail_lateral_offset
point_age
trail_age
trail_speed
trail_width
trail_streak_length
trail_confidence
wetness
feature_type
tangent_x
tangent_y
tangent_z
trail_lane_index
trail_lane_count
trail_lane_pitch
trail_lane_span
trail_lane_crossing
trail_cross_seed
```

`kWaterTrailScalarFieldCount` remains 31. CPU point clouds and GPU Flow output use this same contract; the GPU buffer is merely field-major (`field * pointCapacity + point`) and does not introduce, rename, or reorder any scalar. The renderer consumes `trail_role`, route fields, `trail_width`, `trail_streak_length`, `trail_confidence`, `wetness`, `feature_type`, tangent fields, and lane fields for animated route-following, lane crossing, and world-aligned elongated Gaussian surfels. Mesh Flow uses `feature_type = 5`; Flow and Field behavior must remain isolated from feature-specific shader/offline branches. Rain has no `feature_type` or hidden route anchors. Do not rename or reorder these fields without a coordinated serialization, shader, offline renderer, visual preset, and test update.

## Rendering Contract

Point-cloud styles now have `waterStreamOverlay` for generated stream layers. Old `flowAnimation` / `waterPathView` styles remain parseable aliases for compatibility, but new stream overlays should use the v2 water overlay path.

Stream samples render as world-aligned elongated surfels:

```text
long axis  = tangent * stream_world_length
short axis = cross(normal, tangent) * stream_width
normal     = local surface normal
```

Generated Flow, Mesh Flow, and Field stream output participates in viewport rendering and the same EXR/MP4 export path as other visible point-cloud layers. Live Flow and GPU export consume each source's settled GPU buffers; CPU-only offline animation freezes or builds one deterministic settled snapshot rather than regenerating it per frame. Each live/export sample evaluates one immutable `WaterFrameState` with normalized/sample time, raw scenario, delayed Seepage scenario, and per-node states. Rain and Flow consume the raw scenario; only Seepage consumes the delayed moisture envelope, and both render/export consumers receive the same frozen object. Dedicated Rain compute, streak drawing, and role impacts are recorded in those same Vulkan frame/export command buffers. Ripples evaluate through sparse base-cloud runtime memberships/params, while Seepage evaluates retained compact topology/hash plus frame-safe parameters across Screen Sprites, World Surfels, Fast Basic, viewport EXR, still, and animation paths. Field Surface Motion evaluates through active-cloud `water_effect_*` composition.

## Visuals Contract

Base cloud visuals are evaluated first. Ripple contributions then combine through sparse runtime evaluation, Seepage combines its role-filtered damp/ripple/glint response from the local spatial cell, Rain impacts inspect their role-specific local event list, and Field Surface Motion contributions combine through Visuals-compatible `water_effect_*` fields. Generated Flow, Mesh Flow, and Field Streamline overlays keep their own stream scalar fields; Rain streaks use their dedicated visual settings.

Grouped scene visuals are folder-level. ROCK is the primary visual owner by default, with SAND and VEG receiving mirrored settings. Authored size, opacity, and emission remain expressed against a 1 mm baseline; the renderer applies transient per-role footprint and measured-count coverage compensation to the committed display bundle. Role-specific gates still apply after mirroring: SAND can show shader shoreline waves, VEG can show roughness surface motion, Seepage follows each node's target-role set, and ROCK remains the primary stationary reference.

The Visuals tab's scene-wide **Visible Point Cloud** selector changes only presentation density. The old ROCK/SAND/VEG bundle stays visible while all target roles stage in CPU memory. The renderer then performs one fenced mutation batch: it retires the old GPU bundle, uploads the complete target hidden with fresh descriptor generations, reattaches the existing role-specific Seepage topology, and commits atomically. A partial allocation restores the old bundle from retained CPU clouds. Point-buffer byte diagnostics enforce a high-water bound of the larger complete bundle, while Flow output and the shared surface cache remain untouched. Viewport and offline/export paths exclude CPU-only analysis sources and staged targets.

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
<scene>/.invisible_places/cache/water/<surface-signature>.surfacecache
<scene>/.invisible_places/cache/flow/<payload-checksum>.flowpathcache
<source-stem>-WaterFieldCache.bin
```

The schema-2 `.raincache` name is retained only as a legacy reader. Schema-43 projects store a compact shared-cache manifest per scene and a compact Flow-path manifest per water scene. Settled generated point-source branches are pruned and written atomically to content-addressed `.flowpathcache`; stale/orphan data and generated arrays are never embedded in project JSON. Complete-payload addressing prevents one project from overwriting another project's hidden-branch or analysis state, and unmanifested directory candidates are never adopted. The Flow-path sidecar remains unrelated to manual authored routes or the shared surface cache and is stamped against canonical analysis support rather than the selected display bundle. Each Flow branch stores a `bake_fingerprint` derived from the emitter and resolved Path profile so incremental Bake Path can safely reuse unchanged branches after project reload. `WaterFieldCache.bin` is derived output for user-authored region Field caches, not a normal project source layer. It stores the support path/signature, field settings fingerprint, region fingerprint, field settings, stale flag, selected region boundary, and serialized field-node records. Region caches are reused when fingerprints match and rebuilt when source support, region geometry/settings, or field settings change. Path-anchor Field caches are currently rebuilt from Flow path anchors and stamped in memory rather than saved as mandatory binary caches.

Seepage does not add a separate disk cache. Authored nodes and looks are small project/source JSON records; grouped scenes derive connected node-local support from the already persisted shared `WaterSurfaceCache`, then build the role-filtered sparse hash in memory. Topology/parameter fingerprints prevent support rebuilds or topology uploads during reach/width/prominence, look, node-key, reflection, quality, visibility, or Seepage Rain edits.

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
- Continuous triangle-mesh Seepage routing remains a future option. Current grouped scenes use connected authored 10 mm shared-cache cells; the mesh is not queried or uploaded for Seepage.
- Manual application EXR/MP4 acceptance remains useful as a final operator check, but automated tests now cover active-cloud water-effect EXR writing and MP4 frame conversion.

## Change Checklist

Use these checks after water feature changes:

- Sample scene: `tests/fixtures/sample_scene_water_sources.json` durably owns `SampleFlowPoint`, `SampleFlowPath`, and `SampleSeepage`; `scripts/generate_sample_scene_validation.py` rebuilds the schema-43 `Saved/validation/SampleSceneValidation_project.json` with a 3 mm display, complete 2 mm cache input, and no derived cache payloads.
- Serialization: schema-43 saves include authoritative scene density groups, shared surface-cache manifests, compact Flow-path sidecar manifests, global/per-source Flow visibility, Rain settings-v3 controls, and connected Seepage dimensions/selection limits. Standalone water sources use schema 18 and animations use schema 11 with normalized per-node Seepage reach/width/prominence/activity/wetting tracks and Rain timing.
- Project ownership: water emitters/sources load from the active project only; new projects are allowed to have none.
- Cache reload: `.flowpathcache` branches and baked anchors reload only when manifest checksums and support/settings signatures match; branch bake fingerprints permit per-source reuse when only a subset of emitters changed.
- Density switch: 1/2/3/5 mm display changes leave analysis support signatures, connected Seepage topology, settled Flow/Field output, and shared-surface revisions unchanged. An exact-source Ripple cache may restore after commit; otherwise Ripple/Field stays dirty until explicit recalculation, with no synchronous switch-time topology work.
- Legacy load: old Caustic regions become Ripple `Caustic Lace`; old Basin/Runoff records are ignored.
- Shared cache: three exact-complete-2-mm source scans produce deterministic 10 mm Rain and Flow/Seepage tiers; a schema-3 `.surfacecache` warm load performs no PLY scan, CPU table rebuild, second full-payload hash, or duplicate GPU-table copy. Repeated polling uploads one revision, explicit rebuild advances requested/built generations, and display/effect edits leave these counters unchanged.
- Flow: source-specific point edits rebuild only changed point-source branches where possible; global Path settings and attractor edits dirty `WaterPathCache`; manual sources bypass that cache and refresh on Save. Surface-guided topology edits upload/dispatch only the owning GPU source, capacity grows geometrically, stale revisions never replace settled output, and deferred deletion leaves every other source revision unchanged. Global/per-source Show Trail and Activity/style playback touch only mapped state and leave source/cache topology revisions unchanged.
- Seepage: placement, role, selection-limit, depth/edge, or cache-identity edits rebuild node-local connected support asynchronously; live reach, width, prominence, visibility, quality/look/reflection/node-key/Rain-envelope edits update frame-safe compact parameters. Activity zero takes the early-out, references stay capped, and failed replacement retains settled topology.
- Stream schema: generated stream scalar fields match the exact order above.
- Rendering: `waterStreamOverlay` styles compile and render tangent-aligned surfels.
- Visuals: base-cloud scalar mappings remain intact after creating Ripples, SAND Shoreline waves, Seepage, Flow/Mesh Flow trails, GPU Rain and role impacts, Field Streamlines, and Field Surface Motion; generated stream sessions are hidden from base-cloud look-dev/export visual selection.
- Regions: Ripple and Field regions preserve concave clicked boundaries.
- Motion: one immutable `WaterFrameState` supplies raw Rain/Flow and delayed Seepage state per live/export sample. Keyed Flow uses a deterministic Trail subset without geometry uploads; normalized Seepage node keys and its delayed envelope retain connected hash topology; Rain remains immediate.
- Attractor: `Water path attractor biases a downhill fork without climbing Z` verifies that attractor strength biases the route in XY while path samples continue descending in Z.
- Field cache: region Field caches save, reload, and invalidate on support, region, or settings changes; path-derived Field caches rebuild from Flow path anchors.
- Export: visible generated Flow/Mesh Flow/Field stream layers, dedicated GPU Rain and role impacts, sparse Ripple runtime effects, SAND Shoreline waves, all four compact-grid Seepage patterns with matching per-node/Rain timing, and active-cloud Field Surface `water_effect_*` fields appear in viewport, still, EXR, animation/video, and offline paths without requiring water PLY export.
