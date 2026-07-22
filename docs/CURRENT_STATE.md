# Current State Reference

This document describes the implemented project surface for future edits. Evidence logs live in `docs/logs/`.

## Point-Cloud Rendering

- Supported point-cloud renderer modes are Beauty and Fast Basic.
- Raycast/Raytracing is not part of the active renderer, export, shader, CMake, settings, serialization, or test surface.
- Project files that contain `"point_cloud_renderer_mode": "raytraced"` load as Beauty and save back as `"beauty"`.
- Beauty rendering keeps the unified material path for advanced point styling. Fast Basic keeps cheap colour and point-size controls for preview-oriented rendering.
- Role-named sibling PLY files are grouped as one scene in the UI. Complete ROCK/SAND/VEG sets at one quantized spacing are exposed through the scene-wide **Visible Point Cloud** selector; incomplete and ambiguous sets are unavailable.
- Folder-level point visuals are edited once and mirrored to the committed scene roles. Values remain authored against 1 mm spacing, while render-only footprint and measured-count coverage compensation keep sparser display bundles visually comparable without changing the UI values.
- Canonical ROCK/VEG 1 mm and SAND 2 mm sources remain CPU-resident as the analysis set. Only the committed display bundle is renderable/GPU-resident; staged targets switch atomically and never enter viewport or export snapshots.
- Scalar-field bindings resolve by exact name, then one unique case-insensitive name. Missing named fields retain their binding and render its constant fallback instead of silently using the same numeric slot from another density variant.
- Roughness surface motion is role-gated to VEG in grouped scenes. ROCK and SAND remain stationary for that effect.
- Shader shoreline waves are role-gated to SAND in grouped scenes. `Foam Fronts (Current)` preserves the existing `SandCloudShorelineWaveValue` path; `Height Foam` adds independent run-up/break heights, persistent offshore foam, stronger incoming gathers, and fading returns. Neither algorithm allocates region memberships or dense scalar fields.

## Water Workflow

- The Water panel exposes Ripples, Shoreline, Seepage, Rain, Flow, Mesh Flow, and Field in that order.
- Basin and Runoff records in older project or water-source JSON load harmlessly and do not participate in active UI, runtime editing, or new saves.
- Older Caustics region records load into Ripple layers using the Caustic Lace overlay type.
- New project and water-source saves use v2 water keys and omit `water_basin_regions`, `water_runoff_regions`, and `water_caustic_regions`.
- Flow, Mesh Flow, and Field streams are derived render output. Rain uses a dedicated GPU particle draw backed by the shared surface cache and does not create a generated point-cloud session. Ripple effect data is evaluated on the active/base cloud through sparse runtime memberships and compact procedural params; Shoreline waves are evaluated during the SAND point-cloud draw. Seepage is evaluated on eligible base-cloud points through retained compact topology/hash plus frame-safe parameter buffers and does not create a generated point-cloud session. Stale legacy `-Ripples.generated` sessions are cleared rather than refreshed. Normal project layer persistence skips generated water output, and v2 settings plus cache/style data regenerate the needed in-memory output.
- Flow point sources and manual spline sources are project-owned state, not separate global saved state. A new project can therefore start with no sources while retaining reusable path, lane, trail, rain visual, and visual defaults.
- Seepage nodes are placed or moved with the canonical analysis picker and default to ROCK and VEG roles. Grouped scenes trace downhill through the shared orientation-independent 20 mm ROCK/SAND 3D surfels and reduce the accepted route to eight stations; VEG uses ROCK substrate and SAND remains opt-in. The cache is queried only while preparing those guides, never per rendered point, and the visible high-density cloud is not scanned for Seepage membership. Standalone clouds retain the capped point-support fallback. Node points remain visible in the viewport; the transient Show Structure Overlay option toggles evaluator-aligned ribbons, regions, centrelines, caps, and support warnings without affecting renders or exports. New projects use restrained Chaotic Bloom; saved looks without a pattern retain Legacy Ripples.

## Water v2 Behavior

- Ripple contributes to active/base point-cloud visual evaluation through sparse GPU/offline runtime memberships and params. Shoreline contributes through SAND-role point-style shader uniforms. Field Surface Motion currently contributes through composable `water_effect_*` scalar fields.
- Seepage derives at most eight surface-guide stations per node from the shared cache, then builds a bounded spatial hash from the small authored node set. The same guide drives the viewport, offline evaluator, and optional structure ribbons. Geometry/support/role edits replace topology and hash buffers; live look, reflection, quality, strength, scenario, per-node key, and Rain edits retain those buffers and update only compact parameters. Parameter snapshots are ringed by frame in flight with a separate EXR snapshot, and an inactive node exits before guide/noise work. Auto quality resolves from effective point invocations, while explicit Low, Balanced, and High modes remain available.
- Continuous triangle-mesh routing is not part of the current Seepage runtime. It remains a documented future option for explicit connectivity across scan gaps; the implemented grouped-scene path is the 20 mm shared-cache guide reduced to eight stations, and no mesh data reaches the Seepage shaders.
- Seepage supports Legacy Ripples, Wet Rock Sheen, Chaotic Bloom, and Wetting Trickle. Legacy preserves the warped sinusoidal evaluator. Wet Rock uses seed-rotated world-space 3D gradient noise, analytic micro-normal variation, the real camera vector, and a virtual environment direction for damp patches and angle-dependent reflection. Chaotic Bloom domain-warps ridged 3D noise along the surface-guide tangent for irregular downhill evolution. Wetting Trickle blooms small saturated patches, advances a soft irregular front over guide distance, and leaves short narrow downhill fingers with persistent damp sheen. Common wetness, coverage, glisten, Rain Response, blend, tint, opacity, emission, and point-size controls remain shared. The panel shows only controls used by the selected pattern and gives every parameter-name label a hover tooltip.
- Project scenarios globally override node looks while retaining each node's geometry, seed, roles, authored strength, and enablement. The seeded Pre-Colonisation Wet and Contemporary Managed scenarios intentionally share pattern coordinates but differ in level, spread, wetness, coverage, glisten, evolution, and Rain Response. Authored Node Looks restores the profile/local-look hierarchy.
- Animation schema 10 stores normalized global water scenario keys with complete look snapshots, Seepage level/spread, continuous Rain and Flow levels, Seepage Rain delay/rise/recession, outgoing Smooth/Linear/Hold interpolation, and embedded linked-scenario fallbacks. Each track can also store normalized per-node Activity, Local Spread, and Wetting Front keys by stable node ID. A missing node track preserves Activity 1, Local Spread 0, and a complete Wetting Front. The UI displays and edits key time in seconds while storage remains normalized. Water keys retain their timing when animation duration, frame rate, or camera keys change; copying water keying does not copy those animation properties.
- Scenario and look animation changes compact parameter buffers only. Spatial bounds are pre-indexed for maximum scenario-plus-Rain expansion. Continuous keyed Rain directly scales active GPU particle density, visibility, and impact energy. Seepage separately samples a deterministic 120 Hz Delay/Rise/Recession moisture envelope, so visible Rain is not delayed and out-of-order scrubbing remains reproducible. Keyed Flow reveals a deterministic subset of prebuilt trails and scales their opacity/emission, width, speed, visible length, and lateral micro-motion. A long Rain/Flow/Seepage animation therefore does not retrace guides, rebuild Seepage hashes, rebuild Flow geometry, dispatch route generation, or upload point-cloud/shared-cache data.
- The region `Shoreline` Ripple overlay remains separate from Water > Shoreline. Region Shoreline uses the Ripple tide-band runtime path inside selected polygons; Water > Shoreline uses the SAND point cloud to simulate waves at the beach boundary without rebuilding Ripple memberships.
- Ripple pattern and contribution edits can live-update compact runtime params when region membership is current, giving millisecond-scale feedback and avoiding dense full-cloud scalar uploads.
- Water effect composition supports intensity, emission, opacity add/multiply, point-size add/multiply, and colourise contributions with add, max, multiply, screen, and override blend modes.
- Existing base-cloud scalar mappings, including Height and Intensity driven mappings, remain active while water effects compose on top.
- Ripple and Field regions use the authored clicked polygon boundary for containment, so concave regions preserve their cut-out areas.
- Ripple and Field share one region-selection path that exposes selected base point indices, edge weights, normals, scalar values, field vectors, and manual control flags.
- Flow path cache reuse, hidden branch IDs, smoothing refresh, support-layer signatures, and baked path anchors are part of the saved/reloaded water workflow. Bake Path now reuses branch-level emitter fingerprints so moved, added, or deleted sources rebuild independently when the support signature still matches. It saves the refreshed cache and queues source-local Lane/Trail work instead of synchronously building and uploading a complete overlay. Hiding or restoring a branch invalidates only the lightweight Path View guide and keeps the settled trails visible until their asynchronous replacement is ready; it no longer scans or uploads a million-sample visibility mask.
- Manual Flow path sources bypass downhill path generation. Live GPU generation evaluates their open centripetal Catmull-Rom control points directly; deterministic CPU/offline sampling still produces lane-ready anchors with rotation-minimizing frames and the identical 31-field Trail scalar contract. Optional Surface Guide keeps endpoints and first-to-last direction fixed while the shared 3D surface tier bends the interior within Lane Cover Width, projects gravity onto the selected surface sheet, bunches weak/rough support, and falls back to the authored frame when support is missing. Lane assignment fills authored lanes deterministically from the centre outward, including manual routes without generated-path analysis. Generated point-source routes also receive centripetal Catmull-Rom presentation interpolation after profile smoothing while cached raw branches remain unchanged. Generated and manual sources can coexist; Bake Path only rebuilds point-source branches.
- Flow source Path/Lane/Trail selectors have persistent per-assignment locks. Unlocked sources preview matching unsaved profile edits and fall back to the saved profile when no edit exists; locked sources always resolve the saved copy. Manual path sources use the same rule for Lane and Trail.
- Add Path Source opens a transactional viewport draft. Ctrl+left-click adds surface-snapped nodes, a node click selects it, Delete/Backspace removes it, and double-clicking the curve inserts a node in curve order. The selected-node gizmo moves on world X/Y/Z arrows, XY/XZ/YZ squares, or the center's camera-facing plane. Save requires two distinct nodes and refreshes derived trails; Cancel restores the committed source and output. Empty-space navigation and double-click pivot behavior remain available. Show Source Nodes / Paths hides point-source markers and committed authored-path guides for an unobstructed trail view, while an active path draft remains visible.
- Every point and manual Flow source owns Maximum Flow Strength and Rain Response. Effective activity is `clamp(maximumFlowStrength * (flowLevel + (1 - flowLevel) * rainLevel * rainResponse), 0, 1)`; defaults are 1 and 0 respectively. The Flow Level/Flow Off controls are part of the same scenario key as Rain and Seepage.
- Ordinary Flow Lane/Trail topology is GPU-native and source-local. `UploadWaterFlowGpuSource` uploads only sampled point-source anchors or manual Catmull-Rom control points plus one settings record, then runs route and Trail compute passes against the resident shared surface table. Each source owns geometrically growing output capacity and the existing field-major 31-slot Trail scalar layout (`trail_role`, `trail_id`, `source_id`, and the remaining route/Trail fields). One result remains active while a new revision computes; live edits replace the single queued request with the newest revision, stale work is discarded, completed buffers promote after their fence, and prior outputs retire only after referring frame fences. Deletion similarly releases only that source after compute/frame safety. Appearance, source activity, Rain response, opacity, width, speed, and visible length update style state without topology dispatch. Deterministic CPU builders remain for offline/fallback snapshots, and Hiding a Path View branch still affects only its owning source. Per-source diagnostics expose requested/completed revisions, surface-upload revision, transferred bytes, dispatch count, capacity, active samples, pending state, and Surface Guide use.
- Flow Path has an optional viewport-placed attractor. Its strength biases XY candidate ranking toward the attractor, while Z remains the vertical descent axis and attractor edits correctly dirty the path cache.
- Grouped ROCK/SAND/VEG scenes build combined Flow/Field analysis support only from CPU-ready canonical analysis sources, not from the selected display bundle. Separately, one shared `WaterSurfaceCache` streams each exact 5 mm role file once, or the coarsest available fallback, and emits the existing 20 mm Rain ROCK/SAND XY and VEG 3D tiers plus an orientation-independent 20 mm ROCK/SAND 3D tier used by Flow and Seepage guide tracing. The streaming reader also consumes `scalar_Roughness` when present; otherwise roughness derives from hemisphere-aligned normal variance. It does not routinely load the much larger 3 mm clouds. Display switching therefore leaves support signatures, Seepage guides, path/field caches, source placement, Rain collision, Flow guidance, and region analysis unchanged.
- Shared surface-cache schema 2 persists both aggregate records and their power-of-two GPU hash tables. Its optional identity trailer carries the full source signature and a 256-bit digest of the immutable GPU payload; early schema-2 files derive the same identity on load. The active scene warms at project load even when Rain, Flow, and Seepage are disabled; a valid disk cache loads sequentially without rescanning point clouds or rebuilding the hashes. Mapped staging buffers copy the immutable tables into device-local memory and edge-preserving normal/roughness preprocessing runs in the same fenced submission; Rain and Flow atomically switch descriptors only when it completes. Seepage derives only its small guide records from the CPU aggregate and does not pass the global table to its point shaders. The prior cache remains usable until that swap, staging is released after its upload fence, retired buffers wait on only the frames that referenced them, and the resident cache is released only when the active scene changes.
- The Water Surface Cache (Rain + Flow + Seepage) panel reports build/load status, selected source spacing, Rain surface/vegetation occupancy, shared Flow/Seepage 3D-surfel occupancy, cache/upload revisions, resident GPU bytes, table capacity, maximum probe count, preprocess state, and preprocess-dispatch count. Repeated polling of the same scene signature does not increment the upload count.
- Ripple memberships are display-source-specific and are rebuilt/restored for the exact committed cloud. Analysis-based Field presentation data is spatially remapped to display points; analysis point indices are never reused across density variants.
- Flow Lanes animate Trail points from Trail age, seed, speed, wetness, confidence, width, and render time. Playback changes do not require topology regeneration; the existing ordered `trail_*`/route scalar names remain the renderer contract.
- Field supports user-defined Surface Motion, No Flow, Bridge Allowed, and Bridge Blocked regions.
- Region-built Field vector caches are saved under `Saved/water/<source-stem>-WaterFieldCache.bin` and reused when support, settings, and region fingerprints match. Path-anchor Field caches are currently rebuilt from Flow path anchors and kept in memory.
- Flow trails, Mesh Flow trails, and Field Streamlines share the animated 31-field Trail scalar schema. Flow follows path anchors through Lane profiles, Mesh Flow follows triangle-mesh routes with `feature_type = 5`, and Field follows cached vector-field integration from perturbed source points. Rain is isolated in its own compute and six-vertex streak pipelines.
- Rain collision uses the embedded shared-cache view: sparse ROCK/SAND XY top-surface cells plus VEG 3D occupancy voxels. A bounded 20 mm DDA chooses the earliest role hit for wind-slanted drops; collision always stops and respawns the particle, while optional SAND rings, ROCK wet patches, and downward VEG twinkles are evaluated from spatially binned impact events on the displayed cloud.
- GPU Rain smoke coverage uses the 9.2 million-point SampleScene and the 20.7 million-point Scene1 3 mm display bundle. `rain-gpu-sample-scene` and `rain-gpu-scene1-3mm` assert that live weather/visual/effect edits and GPU EXR comparisons reuse one collision upload; the Scene1 smoke resolves the persisted cache from exact 5 mm role files.
- Field streamlines stay surface-bound, split rejected gaps, fade low-confidence support when configured, and report accepted bridge, rejected gap, fade, termination, and manual control diagnostics.
- Viewport and raster export include sparse Ripple runtime effects, SAND Shoreline waves, all four compact-grid Seepage patterns with matching per-node/Rain timing and camera reflection, Flow trails, dedicated GPU Rain plus role-aware impacts, and active-cloud Field Surface Motion without requiring water PLY export. Screen Sprites, World Surfels, Fast Basic, viewport EXR, still, animation/video, and CPU/offline paths consume the same frozen Seepage state.
- Field Surface Motion is a candidate for the same optimization pattern as Ripples: region-bounded support, shader/offline procedural evaluation, and parameter-only updates instead of dense base-cloud field uploads when only visual settings change.

## Serialization

- Project serialization uses schema version 41 for the current project shape; standalone water-source documents use schema version 17. Animation-path saves use schema version 10, and the shared binary water-surface cache uses schema version 2.
- `scene_point_cloud_groups` is authoritative for each grouped scene's display spacing, loaded/visible state, and per-role analysis/display paths. Legacy per-layer grouping and selected-variant fields remain compatibility mirrors.
- Schema-32-and-earlier projects preserve legacy analysis candidates and migrate to the visible primary/ROCK complete bundle when possible, otherwise the nearest complete bundle with a denser tie-break. Scenes without a complete bundle remain non-switchable `Mixed` sets.
- Project documents also persist Water v2 emitters, manual Flow spline sources, per-source Maximum Flow Strength/Rain Response, manual Use Surface Guide, Seepage nodes/default look/look profiles including Wetting Trickle controls, project water scenarios with Seepage Rain timing and their preview selection, Ripple layers, Field layers, Flow Path/Lanes/Trail profiles, Flow path attractor settings, Rain settings/visual selection, Field settings, Field stream settings, base-cloud water visuals, SAND Shoreline point-style settings, and water path cache data with branch bake fingerprints for incremental reuse.
- Rain settings include a master Impact Effects switch and independent SAND, ROCK, and VEG reaction switches. All rain appearance, weather, amount, and effect controls are runtime uniforms and do not invalidate or resize the collision cache and persistent GPU buffers.
- Camera shots and animation paths associate grouped point-cloud work with the scene folder, not the individual ROCK/SAND/VEG child PLY paths. Older child-path associations are canonicalized to the scene when loaded.
- Project documents also preserve water animation trail settings/profiles and caustic look settings for legacy animation and visual compatibility.
- Standalone water-source documents persist the same active Water v2 surface, including `water_manual_flow_paths`, needed to reload sources independently from projects. Manual trail overlays are derived after the active support scene loads and are never written as project layers.
- Animation paths preserve water caustic look settings for current visual style compatibility and schema-10 normalized Seepage/Rain/Flow scenario tracks, nested per-node Seepage keys, and embedded scenario fallbacks for synchronized playback and export. Schema-9 animations without the new node tracks preserve full node activity/front and zero local spread; missing Seepage Rain timing remains immediate. Older documents also default to full Flow (`flowLevel = 1`), source strength 1, Rain response 0, and disabled Surface Guide on legacy manual paths; newly authored manual paths enable Surface Guide. Existing saved looks without a pattern remain Legacy Ripples and unknown pattern names fall back to it.
- Generated/effect water output is treated as derived data and is not stored as normal project source layers.

## UI And Platform Notes

- Ranged-float controls reject non-finite and hard-limit out-of-range typed values with visible inline validation feedback.
- Valid typed values and drag edits keep their existing control behavior.
- The bootstrap Cocoa window assigns layer colours through managed `NSColor` `CGColor` accessors.

## Verification Reference

Normal verification commands:

```text
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug --output-on-failure
```

Focused tests for this state include:

- `SampleScene validates local multi-cloud shoreline fixture`
- `SampleScene shoreline waves animate over time`
- `Sand-cloud shoreline waves use dedicated foam helper`
- `Shoreline shader layouts reserve Height Foam parameters consistently`
- `Discovery groups role-named sibling PLY files by folder`
- `ExhibitionScene project template contains only selected multi-cloud scene layers`
- `Scene role roughness motion only animates vegetation`
- `Shoreline wave height mask fades around the sand-rock boundary`
- `Height Foam shoreline keeps independent defaults and clamps break height`
- `Shoreline wave defaults are visible when enabled`
- `Seepage defaults describe a subtle damp fan`
- `Seepage fan affects only supported points below its node`
- `Seepage guide bends the affected fan along supported surface stations`
- `Seepage surface-guide builder follows curved ROCK support before VEG`
- `Seepage grid remains compact and reports bounded cell overflow`
- `Seepage pattern algorithms are deterministic and respond to camera angle and time`
- `Wetting Trickle reveals short deterministic fingers behind a keyed front`
- `Per-node Seepage keys compose activity spread and wetting without topology changes`
- `Seepage Rain envelopes are immediate by default and deterministic when delayed`
- `Seepage surface-cache guides follow a vertical ROCK sheet`
- `Water scenario tracks interpolate normalized keys and replace duplicate positions`
- `Seepage scenario application changes parameter fingerprints without changing topology`
- `Project documents round-trip Seepage nodes and shared looks`
- `Previous Seepage schemas load trickle and Rain timing defaults`
- `Schema-nine animations default missing Seepage node tracks and Rain timing`
- `Offline export evaluates animated Seepage without point memberships`
- `Fast Basic offline export applies the full Seepage response`
- `Legacy Raytraced renderer mode loads as Beauty and is not re-saved`
- `Legacy water region records load as v2 ripples while basin and runoff are ignored`
- `Ripple and Field effects compose onto base cloud visual evaluation`
- `Water region selections expose shared point metadata for ripples and fields`
- `Ripple effect generation preserves concave clicked region boundaries`
- `Field cache builds from concave selected regions`
- `Field streamlines split rejected gaps and fade low-confidence support`
- `Water field vector caches save reload and expose invalidation fingerprints`
- `Field stream trails use emitter perturbation and follow vector fields`
- `Water path attractor biases a downhill fork without climbing Z`
- `Water path bake inputs ignore refresh-only trail and smoothing settings`
- `Manual Flow splines produce deterministic lane-ready anchors and trails`
- `GPU Flow output layout is deterministic and grows source-locally`
- `Manual Flow turbulence is arc-distance stable and independent of sprite width`
- `Manual Flow CPU routes use the shared surface cache with bounded fallback`
- `Manual Flow paths round-trip through project scenes and water-source documents`
- `Manual Flow path gizmo math constrains axes and planes and preserves insertion order`
- `rain collision input streams only positions and normals`
- `water surface input streams optional roughness in the same pass`
- `shared water cache consumes roughness during its source scan`
- `shared water cache retains orientation independent role surfels`
- `water surface cache derives roughness from normal variance`
- `water surface CPU queries preserve the continuous surface sheet`
- `rain collision DDA cannot tunnel through distant vegetation`
- `rain simulator is deterministic and skips events when effects are off`
- `rain impact grid bounds work and isolates scene roles`
- `Water Flow activity combines keyed level and Rain response deterministically`
- `Water Flow activity scales are deterministic and monotonic`
- `Offline Water Flow activity hides and reveals stable trails`
- `Older project and water-source schemas default Flow activity fields`
- `Animation paths round-trip keyed Flow activity and migrate legacy defaults`
- `Water v2 streams expose deterministic scalar contracts`
- `Offline water stream overlays animate through time playback`
- `Offline water stream overlays use stream tangent and world length`
- `Offline ripple effect overlays render from virtual effect fields`
- `Project document round-trips binding-backed point-cloud styles`
