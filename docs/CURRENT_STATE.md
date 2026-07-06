# Current State Reference

This document describes the implemented project surface for future edits. Evidence logs live in `docs/logs/`.

## Point-Cloud Rendering

- Supported point-cloud renderer modes are Beauty and Fast Basic.
- Raycast/Raytracing is not part of the active renderer, export, shader, CMake, settings, serialization, or test surface.
- Project files that contain `"point_cloud_renderer_mode": "raytraced"` load as Beauty and save back as `"beauty"`.
- Beauty rendering keeps the unified material path for advanced point styling. Fast Basic keeps cheap colour and point-size controls for preview-oriented rendering.
- Role-named sibling PLY files are grouped as one scene in the UI. ROCK is the default primary role, while SAND and VEG remain separate backend point-cloud layers for density compensation and role-specific effects.
- Folder-level point visuals are edited once and mirrored to selected scene roles. World-sized sprite/surfel dimensions scale by spacing ratio, while pixel-sized styles use spacing-aware opacity/density compensation.
- Roughness surface motion is role-gated to VEG in grouped scenes. ROCK and SAND remain stationary for that effect.
- Shader shoreline waves are role-gated to SAND in grouped scenes. They use `boundaryZ - worldPosition.z` around the editable `z = 1.55 m` boundary and do not allocate region memberships or dense scalar fields.

## Water Workflow

- The Water panel exposes Ripples, Flow, and Field as the active workflow.
- Basin and Runoff records in older project or water-source JSON load harmlessly and do not participate in active UI, runtime editing, or new saves.
- Older Caustics region records load into Ripple layers using the Caustic Lace overlay type.
- New project and water-source saves use v2 water keys and omit `water_basin_regions`, `water_runoff_regions`, and `water_caustic_regions`.
- Flow and Field stream sessions are derived render output. Ripple effect data is evaluated on the active/base cloud through sparse runtime memberships and compact procedural params; stale legacy `-Ripples.generated` sessions are cleared rather than refreshed. Normal project layer persistence skips generated water output, and v2 settings plus cache data regenerate the needed in-memory output.
- Flow emitters and point sources are project-owned state, not separate global saved state. A new project can therefore start with no sources while retaining reusable path, lane, trail, and visual defaults.

## Water v2 Behavior

- Ripple contributes to active/base point-cloud visual evaluation through sparse GPU/offline runtime memberships and params. Field Surface Motion currently contributes through composable `water_effect_*` scalar fields.
- Ripple pattern and contribution edits can live-update compact runtime params when region membership is current, giving millisecond-scale feedback and avoiding dense full-cloud scalar uploads.
- Water effect composition supports intensity, emission, opacity add/multiply, point-size add/multiply, and colourise contributions with add, max, multiply, screen, and override blend modes.
- Existing base-cloud scalar mappings, including Height and Intensity driven mappings, remain active while water effects compose on top.
- Ripple and Field regions use the authored clicked polygon boundary for containment, so concave regions preserve their cut-out areas.
- Ripple and Field share one region-selection path that exposes selected base point indices, edge weights, normals, scalar values, field vectors, and manual control flags.
- Flow path cache reuse, hidden branch IDs, smoothing refresh, support-layer signatures, and baked path anchors are part of the saved/reloaded water workflow.
- Grouped ROCK/SAND/VEG scenes build combined water support from the active scene, not from an individually selected child layer. ROCK is the canonical high-detail support owner, while SAND and VEG use lighter role multipliers by default without creating a full merged GPU point buffer.
- Flow Lanes animate Trail points from stream age, seed, speed, wetness, confidence, width, and render time. Playback changes do not require topology regeneration; internal `stream_*` scalar names remain the renderer contract.
- Field supports user-defined Surface Motion, No Flow, Bridge Allowed, and Bridge Blocked regions.
- Region-built Field vector caches are saved under `Saved/water/<source-stem>-WaterFieldCache.bin` and reused when support, settings, and region fingerprints match. Path-anchor Field caches are currently rebuilt from Flow path anchors and kept in memory.
- Flow trails and Field Streamlines share the animated `stream_*` scalar schema; Flow follows path anchors through Lane profiles while Field follows cached vector-field integration from perturbed source points.
- Field streamlines stay surface-bound, split rejected gaps, fade low-confidence support when configured, and report accepted bridge, rejected gap, fade, termination, and manual control diagnostics.
- Viewport rendering, EXR export, and MP4 preview conversion include sparse Ripple runtime effects, Flow Trails, and active-cloud Field Surface Motion output without requiring water PLY export.
- Field Surface Motion is a candidate for the same optimization pattern as Ripples: region-bounded support, shader/offline procedural evaluation, and parameter-only updates instead of dense base-cloud field uploads when only visual settings change.

## Serialization

- Project serialization uses schema version 26 for the current project shape.
- Project documents persist scene grouping metadata, role names, inferred/manual point spacing, selected scene variants, Water v2 emitters, Ripple layers, Field layers, Flow Path/Lanes/Trail profiles, Field settings, Field stream settings, base-cloud water visuals, and water path cache data.
- Camera shots and animation paths associate grouped point-cloud work with the scene folder, not the individual ROCK/SAND/VEG child PLY paths. Older child-path associations are canonicalized to the scene when loaded.
- Project documents also preserve water animation trail settings/profiles and caustic look settings for legacy animation and visual compatibility.
- Standalone water-source documents persist the same active Water v2 surface needed to reload sources independently from projects.
- Animation paths preserve water caustic look settings for current visual style compatibility.
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
- `Discovery groups role-named sibling PLY files by folder`
- `ExhibitionScene project template contains only selected multi-cloud scene layers`
- `Scene role roughness motion only animates vegetation`
- `Shoreline wave height mask fades around the sand-rock boundary`
- `Shoreline wave defaults are visible when enabled`
- `Legacy Raytraced renderer mode loads as Beauty and is not re-saved`
- `Legacy water region records load as v2 ripples while basin and runoff are ignored`
- `Ripple and Field effects compose onto base cloud visual evaluation`
- `Water region selections expose shared point metadata for ripples and fields`
- `Ripple effect generation preserves concave clicked region boundaries`
- `Field cache builds from concave selected regions`
- `Field streamlines split rejected gaps and fade low-confidence support`
- `Water field vector caches save reload and expose invalidation fingerprints`
- `Field stream trails use emitter perturbation and follow vector fields`
- `Water v2 streams expose deterministic scalar contracts`
- `Offline water stream overlays animate through time playback`
- `Offline water stream overlays use stream tangent and world length`
- `Offline ripple effect overlays render from virtual effect fields`
- `Project document round-trips binding-backed point-cloud styles`
