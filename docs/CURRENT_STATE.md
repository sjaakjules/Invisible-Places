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
- Flow, Mesh Flow, and Field stream sessions are derived render output. Rain uses a dedicated GPU particle draw backed by a shared static collision cache and does not create a generated point-cloud session. Ripple effect data is evaluated on the active/base cloud through sparse runtime memberships and compact procedural params; Shoreline waves are evaluated during the SAND point-cloud draw. Seepage is evaluated on eligible base-cloud points through compact per-layer node/spatial-hash buffers and does not create a generated point-cloud session. Stale legacy `-Ripples.generated` sessions are cleared rather than refreshed. Normal project layer persistence skips generated water output, and v2 settings plus cache/style data regenerate the needed in-memory output.
- Flow point sources and manual spline sources are project-owned state, not separate global saved state. A new project can therefore start with no sources while retaining reusable path, lane, trail, rain visual, and visual defaults.
- Seepage nodes are placed or moved on canonical analysis support, default to ROCK and VEG roles, and trace a compact widening ribbon downhill across the local 3D surface plus a shared profile or committed/temporary local look override. Node points remain visible in the viewport; the transient Show Structure Overlay option toggles evaluator-aligned ribbons, regions, centrelines, caps, and support warnings without affecting renders or exports. New projects use restrained Chaotic Bloom; saved looks without a pattern retain Legacy Ripples.

## Water v2 Behavior

- Ripple contributes to active/base point-cloud visual evaluation through sparse GPU/offline runtime memberships and params. Shoreline contributes through SAND-role point-style shader uniforms. Field Surface Motion currently contributes through composable `water_effect_*` scalar fields.
- Seepage derives at most eight transient surface-guide stations per node, then builds a bounded spatial hash from the small authored node set. The same guide drives the viewport, offline evaluator, and optional structure ribbons; topology uploads only when geometry, support, or role membership changes, while live look and Rain edits update parameters independently. Auto quality resolves from effective point invocations, while explicit Low, Balanced, and High modes remain available.
- Seepage supports Legacy Ripples, Wet Rock Sheen, and Chaotic Bloom. Legacy preserves the warped sinusoidal evaluator. Wet Rock uses seed-rotated world-space 3D gradient noise, analytic micro-normal variation, the real camera vector, and a virtual environment direction for damp patches and angle-dependent reflection. Chaotic Bloom domain-warps ridged 3D noise along the surface-guide tangent for irregular downhill evolution. Common wetness, coverage, glisten, Rain Response, blend, tint, opacity, emission, and point-size controls remain shared.
- Project scenarios globally override node looks while retaining each node's geometry, seed, roles, authored strength, and enablement. The seeded Pre-Colonisation Wet and Contemporary Managed scenarios intentionally share pattern coordinates but differ in level, spread, wetness, coverage, glisten, evolution, and Rain Response. Authored Node Looks restores the profile/local-look hierarchy.
- Animation schema 8 stores normalized water scenario keys with complete look snapshots, Seepage level/spread, continuous Rain level, outgoing Smooth/Linear/Hold interpolation, and embedded linked-scenario fallbacks. Water keys retain their timing when animation duration, frame rate, or camera keys change; copying water keying does not copy those animation properties.
- Scenario and look animation changes compact parameter buffers only. Spatial bounds are pre-indexed for maximum scenario-plus-Rain expansion. Continuous keyed Rain scales active GPU particle density, visibility, and impact energy through frame uniforms, so scrubbing does not rebuild collision data or regenerate geometry.
- The region `Shoreline` Ripple overlay remains separate from Water > Shoreline. Region Shoreline uses the Ripple tide-band runtime path inside selected polygons; Water > Shoreline uses the SAND point cloud to simulate waves at the beach boundary without rebuilding Ripple memberships.
- Ripple pattern and contribution edits can live-update compact runtime params when region membership is current, giving millisecond-scale feedback and avoiding dense full-cloud scalar uploads.
- Water effect composition supports intensity, emission, opacity add/multiply, point-size add/multiply, and colourise contributions with add, max, multiply, screen, and override blend modes.
- Existing base-cloud scalar mappings, including Height and Intensity driven mappings, remain active while water effects compose on top.
- Ripple and Field regions use the authored clicked polygon boundary for containment, so concave regions preserve their cut-out areas.
- Ripple and Field share one region-selection path that exposes selected base point indices, edge weights, normals, scalar values, field vectors, and manual control flags.
- Flow path cache reuse, hidden branch IDs, smoothing refresh, support-layer signatures, and baked path anchors are part of the saved/reloaded water workflow. Bake Path now reuses branch-level emitter fingerprints so moved, added, or deleted sources rebuild independently when the support signature still matches.
- Manual Flow path sources bypass downhill path generation. Their open centripetal Catmull-Rom curves are sampled deterministically into lane-ready anchors with rotation-minimizing frames, then use the same Lane/Trail generation and `stream_*` rendering contract as baked point sources. Lane assignment fills authored lanes deterministically from the centre outward, including manual routes without generated-path analysis. Generated point-source routes also receive centripetal Catmull-Rom presentation interpolation after profile smoothing while cached raw branches remain unchanged. Generated and manual sources can coexist; Bake Path only rebuilds point-source branches.
- Add Path Source opens a transactional viewport draft. Ctrl+left-click adds surface-snapped nodes, a node click selects it, Delete/Backspace removes it, and double-clicking the curve inserts a node in curve order. The selected-node gizmo moves on world X/Y/Z arrows, XY/XZ/YZ squares, or the center's camera-facing plane. Save requires two distinct nodes and refreshes derived trails; Cancel restores the committed source and output. Empty-space navigation and double-click pivot behavior remain available. Show Source Nodes / Paths hides point-source markers and committed authored-path guides for an unobstructed trail view, while an active path draft remains visible.
- Flow Lane/Trail edits use a 150 ms latest-request-wins background refresh. Unchanged source artifacts are reused by fingerprint, path sampling uses prepared cumulative arc distances, and scalar point-cloud packing is cancelable. Hiding a Path View branch masks its current guide/trails immediately and rebuilds only the owning emitter before redistributing the exact trail count. Speed-only edits update affected sources and upload only `trail_speed`; the Flow panel reports build/pack/upload and reuse diagnostics.
- Flow Path has an optional viewport-placed attractor. Its strength biases XY candidate ranking toward the attractor, while Z remains the vertical descent axis and attractor edits correctly dirty the path cache.
- Grouped ROCK/SAND/VEG scenes build combined Flow/Field support only from CPU-ready canonical analysis sources, not from the selected display bundle. Rain independently streams exact 5 mm role files, or the coarsest available fallback, into one persisted 20 mm collision cache. Display switching therefore leaves support signatures, path/field caches, source placement, rain collision, and region analysis unchanged.
- Ripple memberships are display-source-specific and are rebuilt/restored for the exact committed cloud. Analysis-based Field presentation data is spatially remapped to display points; analysis point indices are never reused across density variants.
- Flow Lanes animate Trail points from stream age, seed, speed, wetness, confidence, width, and render time. Playback changes do not require topology regeneration; internal `stream_*` scalar names remain the renderer contract.
- Field supports user-defined Surface Motion, No Flow, Bridge Allowed, and Bridge Blocked regions.
- Region-built Field vector caches are saved under `Saved/water/<source-stem>-WaterFieldCache.bin` and reused when support, settings, and region fingerprints match. Path-anchor Field caches are currently rebuilt from Flow path anchors and kept in memory.
- Flow trails, Mesh Flow trails, and Field Streamlines share the animated `stream_*` scalar schema. Flow follows path anchors through Lane profiles, Mesh Flow follows triangle-mesh routes with `feature_type = 5`, and Field follows cached vector-field integration from perturbed source points. Rain is isolated in its own compute and six-vertex streak pipelines.
- Rain collision uses sparse ROCK/SAND XY top-surface cells plus VEG 3D occupancy voxels. A bounded 20 mm DDA chooses the earliest role hit for wind-slanted drops; collision always stops and respawns the particle, while optional SAND rings, ROCK wet patches, and downward VEG twinkles are evaluated from spatially binned impact events on the displayed cloud.
- GPU Rain smoke coverage uses the 9.2 million-point SampleScene and the 20.7 million-point Scene1 3 mm display bundle. `rain-gpu-sample-scene` and `rain-gpu-scene1-3mm` assert that live weather/visual/effect edits and GPU EXR comparisons reuse one collision upload; the Scene1 smoke resolves the persisted cache from exact 5 mm role files.
- Field streamlines stay surface-bound, split rejected gaps, fade low-confidence support when configured, and report accepted bridge, rejected gap, fade, termination, and manual control diagnostics.
- Viewport and raster export include sparse Ripple runtime effects, SAND Shoreline waves, compact-grid Seepage, Flow trails, dedicated GPU Rain plus role-aware impacts, and active-cloud Field Surface Motion without requiring water PLY export.
- Field Surface Motion is a candidate for the same optimization pattern as Ripples: region-bounded support, shader/offline procedural evaluation, and parameter-only updates instead of dense base-cloud field uploads when only visual settings change.

## Serialization

- Project serialization uses schema version 38 for the current project shape; standalone water-source documents use schema version 14.
- `scene_point_cloud_groups` is authoritative for each grouped scene's display spacing, loaded/visible state, and per-role analysis/display paths. Legacy per-layer grouping and selected-variant fields remain compatibility mirrors.
- Schema-32-and-earlier projects preserve legacy analysis candidates and migrate to the visible primary/ROCK complete bundle when possible, otherwise the nearest complete bundle with a denser tie-break. Scenes without a complete bundle remain non-switchable `Mixed` sets.
- Project documents also persist Water v2 emitters, manual Flow spline sources, Seepage nodes/default look/look profiles, project water scenarios and their preview selection, Ripple layers, Field layers, Flow Path/Lanes/Trail profiles, Flow path attractor settings, Rain settings/visual selection, Field settings, Field stream settings, base-cloud water visuals, SAND Shoreline point-style settings, and water path cache data with branch bake fingerprints for incremental reuse.
- Rain settings include a master Impact Effects switch and independent SAND, ROCK, and VEG reaction switches. All rain appearance, weather, amount, and effect controls are runtime uniforms and do not invalidate or resize the collision cache and persistent GPU buffers.
- Camera shots and animation paths associate grouped point-cloud work with the scene folder, not the individual ROCK/SAND/VEG child PLY paths. Older child-path associations are canonicalized to the scene when loaded.
- Project documents also preserve water animation trail settings/profiles and caustic look settings for legacy animation and visual compatibility.
- Standalone water-source documents persist the same active Water v2 surface, including `water_manual_flow_paths`, needed to reload sources independently from projects. Manual trail overlays are derived after the active support scene loads and are never written as project layers.
- Animation paths preserve water caustic look settings for current visual style compatibility and schema-8 normalized Seepage/Rain scenario tracks for synchronized playback and export.
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
- `Water scenario tracks interpolate normalized keys and replace duplicate positions`
- `Seepage scenario application changes parameter fingerprints without changing topology`
- `Project documents round-trip Seepage nodes and shared looks`
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
- `Manual Flow paths round-trip through project scenes and water-source documents`
- `Manual Flow path gizmo math constrains axes and planes and preserves insertion order`
- `rain collision input streams only positions and normals`
- `rain collision DDA cannot tunnel through distant vegetation`
- `rain simulator is deterministic and skips events when effects are off`
- `rain impact grid bounds work and isolates scene roles`
- `Water v2 streams expose deterministic scalar contracts`
- `Offline water stream overlays animate through time playback`
- `Offline water stream overlays use stream tangent and world length`
- `Offline ripple effect overlays render from virtual effect fields`
- `Project document round-trips binding-backed point-cloud styles`
