# Current State Reference

This document describes the implemented project surface for future edits. Evidence logs live in `docs/logs/`.

## Point-Cloud Rendering

- Supported point-cloud renderer modes are Fast Basic, Beauty Adaptive, Beauty Full Source, and Painted Adaptive.
- Raycast/Raytracing is not part of the active renderer, export, shader, CMake, settings, serialization, or test surface.
- Project files that contain `"point_cloud_renderer_mode": "raytraced"` load as Beauty Adaptive and save back as `"beauty_adaptive"`.
- Beauty Adaptive keeps the unified material path with adaptive draw-item LOD. Beauty Full Source uses raw source points for exact/debug viewport rendering. Fast Basic keeps cheap colour and point-size controls for preview-oriented rendering. Painted Adaptive uses the adaptive hierarchy with brush styling.
- Fast Basic adaptive LOD now uses deterministic stable-ranked representatives, traversal hysteresis, and bounded mixed draw-item transitions so representative density, parent/child replacement, and idle refinement change gradually instead of replacing the displayed set in one frame.
- LOD diagnostics include representative delta per frame, promoted/demoted frontier nodes, hysteresis-kept nodes, active transition count/age, idle refinement pending state, and a `--lod-compare` Fast Basic transition trace CSV.
- Stage 03 large-cloud evidence on `Data/Site3-Mid-1mm100M.ply` uses a 500-frame scripted Fast Basic path. Two repeated runs produced 500 trace rows, max absolute representative delta 5,640, 22 transition-active frames, 0 large representative-jump frames, 0 budget-exceeded frames, and 0 full-source fallback frames.
- The same automated 100M trace does not prove exact idle refinement completion: it ends with async refinement still pending and the displayed bounded fallback protected by the no-flash rule. Use the manual Stage 03 checklist below for final visual acceptance of idle detail completion.

## Water Workflow

- The Water panel exposes Ripples, Flow, and Field as the active workflow.
- Basin and Runoff records in older project or water-source JSON load harmlessly and do not participate in active UI, runtime editing, or new saves.
- Older Caustics region records load into Ripple layers using the Caustic Lace overlay type.
- New project and water-source saves use v2 water keys and omit `water_basin_regions`, `water_runoff_regions`, and `water_caustic_regions`.
- Generated and effect water sessions are compatibility preview/export layers. Normal project layer persistence skips them; v2 settings and cache data regenerate the needed in-memory output.

## Water v2 Behavior

- Ripple and Field Surface Motion contribute to active/base point-cloud visual evaluation through composable `water_effect_*` scalar fields.
- Water effect composition supports intensity, emission, opacity add/multiply, point-size add/multiply, and colourise contributions with add, max, multiply, screen, and override blend modes.
- Existing base-cloud scalar mappings, including Height and Intensity driven mappings, remain active while water effects compose on top.
- Ripple and Field regions use the authored clicked polygon boundary for containment, so concave regions preserve their cut-out areas.
- Flow path cache reuse, hidden branch IDs, smoothing refresh, and support-layer signatures are part of the saved/reloaded water workflow.
- Flow Streams animate from stream age, seed, speed, wetness, confidence, width, and render time. Playback changes do not require topology regeneration.
- Field supports user-defined Surface Motion, No Flow, Bridge Allowed, and Bridge Blocked regions.
- Field streamlines stay surface-bound, split rejected gaps, fade low-confidence support when configured, and report accepted bridge, rejected gap, fade, termination, and manual control diagnostics.
- Viewport rendering, EXR export, and MP4 preview conversion include water stream and active-cloud water-effect output without requiring water PLY export.

## Serialization

- Project serialization uses schema version 24 for the current project shape.
- Project documents persist Water v2 emitters, Ripple layers, Field layers, Flow stream settings, Field settings, Field stream settings, water visuals, and water path cache data.
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

- `Legacy Raytraced renderer mode loads as Beauty Adaptive and is not re-saved`
- `Legacy water region records load as v2 ripples while basin and runoff are ignored`
- `Ripple and Field effects compose onto base cloud visual evaluation`
- `Ripple effect generation preserves concave clicked region boundaries`
- `Field cache builds from concave selected regions`
- `Field streamlines split rejected gaps and fade low-confidence support`
- `Water v2 streams expose deterministic scalar contracts`
- `Offline water stream overlays animate through time playback`
- `Offline water stream overlays use stream tangent and world length`
- `Offline ripple effect overlays render from virtual effect fields`
- `Project document round-trips binding-backed point-cloud styles`
- Fast Basic adaptive LOD stable-rank, cache-loaded ordering, hysteresis, and transition diagnostic regressions

Stage 03 manual visual checklist for the current 100M cloud:

- Slow orbit: no visible popping, flicker, or unstable stochastic crawl.
- Slow dolly through LOD thresholds: parent/child replacement appears gradual.
- Fast navigation then stop: detail refines over multiple frames, not as one sudden load.
- Repeated back-and-forth path: the same areas transition consistently.
- HUD/trace: representative deltas stay clamped, transitions age normally, idle refinement eventually completes, and Stage 02 boundedness remains intact.
