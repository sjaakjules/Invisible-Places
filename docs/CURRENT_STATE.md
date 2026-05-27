# Current State Reference

This document describes the implemented project surface for future edits. Evidence logs live in `docs/logs/`.

## Point-Cloud Rendering

- Supported point-cloud renderer modes are Fast Basic, Beauty Adaptive, Beauty Full Source, and Painted Adaptive.
- Raycast/Raytracing is not part of the active renderer, export, shader, CMake, settings, serialization, or test surface.
- Project files that contain `"point_cloud_renderer_mode": "raytraced"` load as Beauty Adaptive and save back as `"beauty_adaptive"`.
- Beauty Adaptive keeps the unified material path with adaptive draw-item LOD. Beauty Full Source uses raw source points for exact/debug viewport rendering. Fast Basic keeps cheap colour and point-size controls for preview-oriented rendering. Painted Adaptive uses the adaptive hierarchy with brush styling.
- Fast Basic adaptive LOD now uses deterministic stable-ranked representatives, traversal hysteresis, and bounded mixed draw-item transitions so representative density, parent/child replacement, and idle refinement change gradually instead of replacing the displayed set in one frame.
- Adaptive hierarchy cache files use `PointCloudLodCache-v4.bin`. v1-v3 hierarchy caches are treated as stale by version validation and are rebuilt into v4 without deleting the older files unless the user explicitly runs the existing rebuild command. Existing source paths are canonicalized before cache hashing so relative and absolute references reuse the same v4 cache.
- Progressive point-cloud startup also writes additive `.ipcloud` v1 bundles under `Saved/PointCloudCache/<stem>.<fingerprint>.ipcloud/`. The bundle contains `manifest.json`, `attribute_schema.bin`, `hierarchy.bin`, `node_pages.bin`, `node_stats.bin`, `lod_representatives.bin`, `scalar_stats.bin`, `raw_chunks/`, `build_status.json`, and `build_log.txt`. Stage 07 `raw_chunks/` entries are validated source range metadata for resume/fingerprint checks; full raw chunk residency and memory reduction are still Stage 08 work.
- `.ipcloud` validation fingerprints the canonical source path hash, file size, nanosecond mtime, PLY header hash, sampled payload hash, point/scalar counts, RGB/normals presence, build settings version, and attribute schema version. Status values explicitly distinguish missing, hit, stale, partial, and corrupt bundles.
- `.ipcloud` writes build into `<bundle>.tmp`, updates `build_status.json` by phase, validates required files, then publishes by directory rename. Temporary bundles are not treated as complete caches; matching partial bundles can be inspected for resumable phases. Legacy v4 hierarchy caches remain untouched and are fallback-only after full source load, not `.ipcloud` warm caches.
- v4 hierarchy nodes store fixed CPU visual statistics: spacing, density, colour variance/contrast, normal variance, scalar range/variance hints, emissive/accent hints, and feature flags.
- v4 representatives preserve `sourcePointIndex` and add deterministic class flags for spatial coverage, colour contrast, normal/edge, scalar min, scalar max, scalar threshold, emissive/accent, and blue-noise fill, plus scalar-field slot, importance, and stored rank data.
- The hierarchy stores per-field and per-node scalar statistics sized by `nodes.size() * scalarFieldCount`, so active scalar styles can refine against the field currently driving the visual.
- CPU adaptive traversal refines nodes from projected spacing/mark diameter and visible feature statistics; Fast Basic, Beauty screen sprites, Beauty world-mm sprites, and Beauty world surfels use separate threshold/cost tuning while GPU compute selection remains out of scope.
- Beauty Adaptive uses renderer-aware traversal profiles: `FastBasicSquare`, `BeautyScreenSprite`, `BeautyWorldScreenSprite`, and `BeautyWorldSurfel`. The 32-byte draw-item ABI is unchanged; existing fields carry source index, represented count, seed, packed metadata, footprint/render area, opacity scale, and emission scale.
- Beauty opacity compensation uses optical-depth scaling from represented count and raw-vs-LOD footprint area. Ordinary representatives keep average emission at 1.0, emissive/accent representatives get capped coverage scaling, and Beauty traversal preserves at least one representative per visible frontier node under fragment pressure so cost limits do not silently drop the rest of the cloud.
- Vulkan viewport diagnostics use timestamp queries where supported. The renderer checks `timestampPeriod` and graphics-queue `timestampValidBits`, writes previous-frame query pairs around Fast Basic point, Beauty depth, Beauty accumulation, composite, and postprocess/EDL passes, and reads them back without `WAIT_BIT`.
- Adaptive viewport density is now driven by a per-session, per-renderer-profile `PointCloudPerformanceGovernor` instead of scene/present FPS thresholds. The governor tracks EWMA point-pass/composite GPU ms, holds cached frames, scales representative, fragment, blended-fragment, target-spacing, and draw-item upload-byte budgets with clamped steps, and reports `performance-limited` only when the floor budget is reached and measured cost remains over target.
- If timestamps are unavailable or invalid, the governor uses a conservative fixed budget with explicit diagnostics (`timestamp unavailable; conservative fixed budget`) rather than hidden FPS demotion.
- LOD diagnostics include representative delta per frame, promoted/demoted frontier nodes, hysteresis-kept nodes, active transition count/age, idle refinement pending state, emitted feature-class counts, feature-triggered refinement counts, renderer cost profile, radius/opacity/emission scale ranges, estimated vertices/fragments/blended fragments, clamp flags, raw/EWMA GPU point-pass and composite timings, governor budget scale/status/active limit, timestamp support/fallback state, and a `--lod-compare` Fast Basic transition trace CSV.
- Progressive cache diagnostics are surfaced in the status HUD, LOD debug panel, and `--lod-cache-check`: `.ipcloud` cache state/reason, load phase, build phase, publish/resume status, preview/full mode, representative preview count, represented source count, time to bounds/placeholder, time to first coarse frame, and time to first refined frame.
- Cold point-cloud loads can upload a dense representative preview `LoadedPointCloud` before the full source parse/build/upload completes. Warm `.ipcloud` loads validate the bundle, load representative payload first, upload that remapped preview, and then hand off to the existing full adaptive renderer once the full source and exact hierarchy are ready. The 32-byte draw-item ABI is preserved by remapping preview source indices into the preview cloud's dense representative buffers.
- Stage 05 sample evidence on `Data/Site3-Sample-Terrestrial.ply` produced a five-case Beauty matrix with exact Adaptive HQ and no adaptive fallback. Matrix luminance ratios were small opaque 0.805370, large translucent Gaussian 0.831266, emissive/scalar 0.967904, world-sized sprite 1.118410, and world surfel 1.139980. The large translucent case improved from the first Stage 05 run's 0.503523 luminance ratio and 218,755 represented source points to 0.831266 and 535,332 represented source points after the Beauty frontier-coverage fix. Manual contact-sheet review found no obvious holes, footprint explosions, emission/scalar loss, normal inversion, or detail-loading artifacts.
- Stage 05 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` uses the v4 cache and reports exact Adaptive HQ matrix output with no fallback. Matrix luminance ratios were small opaque 0.666881, large translucent Gaussian 0.754400, emissive/scalar 0.950587, world-sized sprite 0.880937, and world surfel 0.600746; all cases reported coverage ratio 1. The primary small-sprite case used 1,729,641 Adaptive HQ representatives covering 99,504,849 source points with feature representatives colour/scalar/normal/accent 17,878/138,540/7,775/16,371.
- The same 100M Fast Basic viewport trace produced 500 rows, max submitted 262,132 under an 8,294,400 representative budget, max estimated fragments 1,108,920 under a 796,262,000 fragment budget, max absolute representative delta 5,640, 22 transition-active frames, 0 large representative-jump frames, 0 budget-exceeded frames, 0 full-source fallback frames, and `fast_basic_zoom_out_updated=true`.
- Stage 06 timestamp feature check passed on the local MoltenVK runtime: both sample and 100M `--lod-compare` runs reported `valid previous frame` timestamp state. The conservative timestamp-unavailable fallback was not activated at runtime and is covered by focused governor tests.
- Stage 06 sample evidence on `Data/Site3-Sample-Terrestrial.ply` kept Fast Basic bounded with max GPU point pass 1.02421 ms, EWMA 0.0213727 ms, governor scale 1, max submitted 262,115 under a 4,823,449 representative budget, max estimated fragments 956,657 under a 915,702,000 fragment budget, 0 performance-limited frames, 0 large representative-jump frames, no budget exceedance, and no full-source fallback. The Beauty stress pass used large translucent Gaussian sprites and reported max GPU point pass 1.14775 ms, EWMA 0.482701 ms, governor scale 1, max adaptive representatives 262,132, max estimated blended fragments 46,212,000 under a 152,617,000 blended budget, no budget reached/exceeded state, and no full-source fallback.
- Stage 06 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` kept Fast Basic bounded with max GPU point pass 0.16225 ms, EWMA 0.0172543 ms, governor scale 1, max submitted 262,132 under a 4,823,449 representative budget, max estimated fragments 1,108,920 under a 915,702,000 fragment budget, 0 performance-limited frames, 0 large representative-jump frames, no budget exceedance, and no full-source fallback. The Beauty stress pass reported max GPU point pass 0.934542 ms, EWMA 0.13772 ms, governor scale 1, max adaptive representatives 262,138, max estimated blended fragments 46,213,100 under a 152,617,000 blended budget, no budget reached/exceeded state, and no full-source fallback.
- Stage 07 sample cache evidence on `Data/Site3-Sample-Terrestrial.ply` wrote `Saved/diagnostics/lod_cache/lod_cache_metrics.json`. A clean `.ipcloud` run reported cold cache `missing`, 65,536 source-sampled preview representatives covering 12,183,742 source points, time to bounds/coarse frame 2,663.56 ms, full source load 6,078.93 ms, hierarchy build 82,512.1 ms, time to first refined frame 91,254.6 ms, atomic publish 717.66 ms, warm cache `hit`, warm time to first coarse frame 304.21 ms, and an interrupted `.tmp` probe marked resumable with `raw_chunks_completed=1/4`. The published sample bundle was about 485 MB.
- Stage 07 sample renderer evidence after the cache changes kept the adaptive renderer intact: `--lod-compare Data/Site3-Sample-Terrestrial.ply` reported coverage ratio 1, luminance ratio 0.802810, 1,564,580 Adaptive HQ representatives covering 12,183,742 source points, Fast Basic max submitted 2,880,233 under a 4,823,449 representative budget, max estimated fragments 4,681,200 under a 915,702,000 fragment budget, 0 large representative-jump frames, no budget exceedance, and no full-source fallback.
- The automated 100M viewport trace still ends with async refinement pending and the displayed bounded fallback protected by the no-flash rule. The exact Fast Basic CPU traversal in `--lod-compare` verifies selector feature preservation; use the manual Stage 03 checklist below for final subjective visual acceptance of idle detail completion.

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
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-cache-check Data/Site3-Sample-Terrestrial.ply
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
- Point-cloud LOD cache v3 stale/v4 round-trip behavior, visual node stats, feature representative classes, source-index preservation, deterministic traversal output, and Stage 03 smoothness metrics
- Point-cloud performance governor EWMA/clamp behavior, conservative timestamp fallback, performance-limited status, GPU-timing-over-FPS source audit, Vulkan timestamp diagnostic surfaces, and blended-fragment budget enforcement
- Point-cloud `.ipcloud` v1 manifest validation, stale/corrupt/partial decisions, temporary-bundle handling, atomic publish, interrupted resume probes, source-preview remapping, warm preview payload loading, and CLI cache diagnostics

Stage 03 manual visual checklist for the current 100M cloud:

- Slow orbit: no visible popping, flicker, or unstable stochastic crawl.
- Slow dolly through LOD thresholds: parent/child replacement appears gradual.
- Fast navigation then stop: detail refines over multiple frames, not as one sudden load.
- Repeated back-and-forth path: the same areas transition consistently.
- HUD/trace: representative deltas stay clamped, transitions age normally, idle refinement eventually completes, and Stage 02 boundedness remains intact.

Stage 05 manual/compare checklist:

- `lod_compare_metrics.json` should show no adaptive fallback, no full-source Fast Basic submission, no representative or fragment budget exceedance, and zero large representative-jump frames.
- Applicable clouds should report nonzero colour contrast, scalar, normal/edge, and emissive/accent class counts in Adaptive HQ and exact Fast Basic CPU metrics.
- Feature-triggered refinement counts should be nonzero in Adaptive HQ and exact Fast Basic CPU metrics when the active style exposes visible RGB, scalar, normal/edge, or emissive/accent variation.
- The Beauty matrix should include small opaque sprites, large translucent Gaussian sprites, emissive/scalar sprites, world-sized sprites, and world surfels when normals are present.
- Coverage ratio should remain 1 for the matrix cases. Luminance ratios can shift with optical-depth compensation, but contact-sheet/manual review should show no obvious holes, opacity explosions, emission/scalar loss, normal inversion, footprint blow-up, or detail-loading artifacts.

Stage 06 manual/compare checklist:

- `lod_compare_metrics.json` should show `*_timestamp_supported=true`, `*_timestamp_state="valid previous frame"` on this MoltenVK runtime, or an explicit conservative fixed fallback state if timestamps are unavailable.
- Fast Basic and Beauty stress metrics should show nonzero raw GPU point-pass ms and EWMA ms, stable governor budget scale, zero performance-limited frames unless the measured floor is actually exceeded, no budget exceedance, no full-source fallback, and zero large representative-jump frames.
- Cached presentation frames must hold the governor rather than updating timing state, and UI/present FPS should remain diagnostic context instead of driving density demotion/promotion.

Stage 07 cache/progressive checklist:

- `lod_cache_metrics.json` should show cold cache `missing`, warm cache `hit`, `cold_preview_loaded=true`, `warm_preview_from_persistent_cache=true`, `resume_resumable=true`, and explicit `time_to_first_coarse_frame_ms` / `warm_time_to_first_coarse_frame_ms` values.
- `.ipcloud.tmp` directories must be reported as partial/resumable or rebuild-required, never complete hits.
- Warm `.ipcloud` preview should reach a coarse frame before full source assembly, while full activation should replace the preview with the existing exact adaptive hierarchy without changing the draw-item ABI.
- Legacy `Saved/lod/*PointCloudLodCache-v4.bin` files should remain non-destructive fallback assets after full source load; they should not be counted as `.ipcloud` warm-cache hits.
