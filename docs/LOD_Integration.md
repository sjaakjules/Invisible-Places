# Adaptive Point-Cloud LOD Integration Audit

Date: May 28, 2026

This document compares the ideal LOD system in
`docs/point_cloud_adaptive_lod_fast_beauty.md` with the current project state.
The ideal document remains the target design and should not be edited as part of
this audit.

## Current Build Status

The current worktree builds with the Stage 04 CPU adaptive LOD quality path:
hierarchy cache v4, visual node statistics, class-aware representatives,
per-node scalar stats, and CPU traversal feature triggers. GPU compute
selection remains a later-stage item.

Fresh runtime/performance reports now include the HUD/diagnostics values
described below so sparse, stale, or feature-erasing adaptive output can be
diagnosed from the UI and `--lod-compare` metrics.

## Implemented

The project has a real first-pass adaptive LOD path. It is not just a raw
sample-count cap anymore.

- `PointCloudLodHierarchy`, `PointCloudLodNode`, `PointCloudLodRepresentative`,
  and `PointCloudDrawItemGpu` exist.
- LOD representatives preserve `sourcePointIndex`, so shaders can fetch source
  positions, colors, normals, scalar fields, water fields, and material bindings
  from the original point data.
- A binary hierarchy cache exists under `Saved/lod/` using filenames like
  `<source-stem>-<source-path-hash>-PointCloudLodCache-v4.bin`.
- Cache validation checks version, source path hash, source size, mtime, point
  count, bounds, build config, scalar field count, and scalar-stat payload
  sizing. v1-v3 files are stale and rebuild into v4 without deleting old files
  unless the existing rebuild command is used. Existing source paths are
  canonicalized before cache hashing, so relative and absolute references reuse
  the same v4 cache.
- Hierarchy cache writes go through a temporary file and publish by rename.
- v4 hierarchy nodes store spacing, density, colour variance/contrast, normal
  variance, scalar range/variance hints, emissive/accent hints, and feature
  flags derived from the full `LoadedPointCloud`.
- v4 hierarchy payloads store per-field and per-node scalar statistics sized by
  `nodes.size() * scalarFieldCount`.
- Representatives preserve `sourcePointIndex` and add class flags for spatial
  coverage, colour contrast, normal/edge, scalar min, scalar max, scalar
  threshold, emissive/accent, and blue-noise fill, plus scalar slot, importance,
  and stored rank data.
- Point-cloud activation attempts to load the hierarchy cache, then starts a
  background hierarchy build when the cache is missing or stale.
- Runtime adaptive traversal is scheduled on a background worker for viewport
  use and cached per layer by a compact traversal key.
- The viewport can reuse a previous adaptive draw-item set while async traversal
  is pending.
- Fast Basic adaptive emission uses deterministic stable-ranked representative
  prefixes, so lower budgets keep a spatially distributed subset of higher
  budgets instead of swapping unrelated samples.
- CPU traversal refines from projected spacing/mark diameter and visible
  feature statistics, with separate threshold/cost tuning for Fast Basic, Beauty
  screen sprites, Beauty world-mm sprites, and Beauty world surfels.
- Traversal accepts the previously displayed frontier, applies separate
  promote/demote hysteresis bands, and reports promoted, demoted,
  hysteresis-kept, and representative-delta diagnostics.
- Fast Basic draw-item replacement is bounded by deterministic mixed
  transitions. Idle refinement reports `Refining point cloud detail...` while
  detail is introduced over multiple frames.
- Per-frame draw-item buffers and an EXR draw-item buffer exist. They grow when
  capacity is insufficient and rewrite descriptors only on reallocation.
- Current point, surfel, constant-simple, and Fast Basic vertex shaders can read
  draw items through binding 7 and use `drawItem.sourcePointIndex`.
- The renderer modes are explicit: `Fast Basic`, `Beauty Adaptive`, `Beauty Full
  Source`, and `Painted Adaptive`.
- Export density modes are explicit: `Full Source`, `Adaptive High Quality`,
  `Match Viewport Adaptive`, `Fast Adaptive Preview`, `Artistic As Preview`, and
  `Artistic High Quality`.
- The diagnostics overlay reports scene update time/FPS, frame time/FPS,
  adaptive representative count, represented source count, traversal time,
  draw-item upload time, draw-item reallocations, cache status, runtime status,
  and adaptive requested/displayed density.
- The diagnostics overlay also reports representative delta per frame,
  promoted/demoted frontier nodes, hysteresis-kept nodes, active transition
  count/age, hysteresis band, idle refinement pending/completed state, emitted
  representative class counts, and feature-triggered refinement counts.
- PLY load progress reports points read, payload bytes read, elapsed time, and
  ETA when enough samples exist.
- LOD hierarchy/cache rebuild progress reports elapsed time, approximate ETA,
  source references processed, nodes built, representatives built, and current
  build depth.
- The status HUD and diagnostics expose adaptive requested/displayed density,
  movement quality tier, cache filename/size, hierarchy source/node/rep counts,
  runtime cache hit/miss, async state, and whether the displayed adaptive buffer
  is exact or coarse fallback.
- `--lod-compare` exists and writes full-source/adaptive EXRs plus
  `lod_compare_metrics.json` and a Fast Basic per-frame transition trace CSV.
- `lod_compare_metrics.json` reports Adaptive HQ representative class counts,
  exact Fast Basic CPU representative class counts, viewport Fast Basic
  boundedness/smoothness metrics, and colour, scalar, normal, and
  emissive/accent feature-triggered refinement counts.
- Stage 04 sample evidence on `Data/Site3-Sample-Terrestrial.ply` reports
  coverage ratio 1, luminance ratio 0.966488, 1,932,759 Adaptive HQ
  representatives covering all 12,183,742 source points, feature representatives
  colour/scalar/normal/accent 136/1,070/65/130, Fast Basic max submitted
  4,111,812 under an 8,294,400 representative budget, no budget exceedance, no
  full-source fallback, zero large representative-jump frames, and
  `fast_basic_zoom_out_updated=true`.
- Stage 04 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` reports coverage
  ratio 1, luminance ratio 0.814833, 1,729,641 Adaptive HQ representatives
  covering 99,504,849 source points, Adaptive HQ feature representatives
  colour/scalar/normal/accent 17,878/138,540/7,775/16,371, and exact Fast Basic
  CPU feature representatives 18,484/142,172/8,085/16,761 across 2,325,565
  representatives covering 99,503,058 source points.
- The 100M Fast Basic viewport trace reports deterministic smoothness metrics:
  max submitted 262,132 under an 8,294,400 representative budget, max estimated
  fragments 1,108,920 under a 796,262,000 fragment budget, max absolute
  representative delta 5,640, 22 transition-active frames, 0 large
  representative-jump frames, 0 budget-exceeded frames, 0 full-source fallback
  frames, and `fast_basic_zoom_out_updated=true`.

## Partially Implemented

These pieces exist, but they are not yet the ideal system described in
`point_cloud_adaptive_lod_fast_beauty.md`.

- The persistent cache stores hierarchy nodes, representatives, and scalar
  stats. It does not store a `.ipcloud` manifest, raw chunks, attribute schema,
  node pages, resumable build status, or source-data chunks.
- Loading still parses and uploads the full PLY source before adaptive rendering
  can be useful. There is no progressive first-load path that displays coarse
  representatives before full parse/upload completes.
- Hierarchy build is asynchronous and reports progress, but the build itself is
  still monolithic and not cancellable until `BuildPointCloudLodHierarchy(...)`
  returns.
- Traversal is CPU-only. There is no GPU compute culling, compaction, indirect
  draw generation, or GPU-driven visible-chunk selection.
- Quality demotion/promotion uses scene update FPS and frame/present FPS, not
  measured GPU point-pass timings. It can react to UI/present behavior rather
  than the actual expensive point/surfel passes.
- Manual sampled point budgets still exist and are uploaded as sampled index
  buffers. The adaptive path treats them mostly as a debug/loading cap, but the
  UI still exposes a generic `Budget` control that can be confused with LOD.
- LOD compensation exists as footprint, opacity, and emission fields, but it is
  simplified. It lacks full optical-depth policy controls, Beauty-specific
  `lodBlend`, and feature-preserving palettes.
- Pixel screen sprites, world-mm screen sprites, world surfels, and
  camera-facing world sprites now have separate CPU selection tuning, but they
  still do not have independent optical-depth policies or GPU timing feedback.
- The LOD comparison metrics now report coverage, mean luminance, feature class
  counts, feature-triggered refinement counts, and a Fast Basic transition
  trace, but not image error maps, timing breakdowns, peak memory, upload
  bandwidth, or per-render-pass GPU time.
- The 100M `--lod-compare` path proves bounded replacement and deterministic
  trace metrics, but it does not automatically prove final exact idle
  refinement completion. The current 500-frame harness still ends with async
  refinement pending under the no-flash fallback guard, so final Stage 03 visual
  smoothness acceptance still requires the manual checklist.

## Not Implemented

These are still target-system items from the ideal plan.

- `.ipcloud` cache bundle with manifest, source fingerprint, attribute schema,
  hierarchy pages, raw chunks, LOD representatives, scalar stats, build status,
  and build log.
- Raw chunk streaming and visible-chunk residency.
- Progressive cache build that renders coarse upper hierarchy data while lower
  levels are still building.
- Memory-mapped source chunks and CPU/GPU LRU caches.
- Device-local static point/chunk buffers with staging uploads as the normal
  large-cloud path.
- Beauty-specific parent/child opacity crossfade or optical-depth transition
  policy.
- Dedicated LOD style data block with explicit compensation/footprint policy.
- Vulkan timestamp queries for point pass, depth prepass, accumulation,
  EDL/composite, upload, and render-state submission.
- EWMA performance governor driven by measured GPU point-pass time.
- Tile overdraw estimates and per-tile fragment/blended-fragment budgets.
- Conservative occlusion culling, depth proxy, or Hi-Z pyramid.
- GPU compute traversal/culling/compaction and indirect draw submission.
- Runtime metrics for cache hit load time, time to first coarse frame, peak
  resident memory, and tile budget pressure.

## Likely Lag Sources

Investigate these before adding more ideal-plan features.

- First-run large-cloud loading can still spend a long time in PLY parse, GPU
  upload, or monolithic hierarchy build. The UI now reports which phase is
  active and provides elapsed time plus ETA when a phase has measurable progress.
- Beauty Adaptive full-source fallback is guarded now, but this remains a
  regression risk when future density modes or loading paths bypass adaptive
  draw-item requirements.
- Async traversal discards stale completions now, but new camera/style keys can
  still spend visible time in coarse fallback while replacement work finishes.
- The quality controller reacts to scene/present FPS instead of GPU pass time,
  so it may demote too late, promote too early, or miss point-pass bottlenecks
  hidden by cached presentation.
- Current projected spacing now uses stored node spacing and feature statistics,
  but there is still no tile-pressure model or measured GPU point-pass feedback.
- All source positions are still uploaded and retained in several forms:
  `LoadedPointCloud`, `cpuPositions`, vertex position buffer, storage position
  buffer, color buffer, normal buffer, scalar buffer, and hierarchy data. Large
  clouds can therefore be memory- and bandwidth-heavy before LOD helps.
- Static point buffers are host-visible in the current path. This is simple but
  not the ideal MoltenVK/Apple-GPU path for very large mostly-static data.
- The manual sampled budget path calls `WaitIdle()` during point-budget updates.
  That is acceptable for explicit debug/loading cap changes, but it must not be
  part of normal adaptive updates.
- There is no tile overdraw budget yet. Close, grazing, long-site views with
  large translucent marks can still produce the exact overdraw problem the ideal
  plan is designed to avoid.

## Implementation Order

### 1. Keep Build And Baseline Metrics Healthy

Keep the LOD progress overload, UI diagnostics, and focused tests building
before doing any performance work.

Minimum checks:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
```

Capture a baseline on a representative large cloud:

- scene update ms/FPS
- frame ms/FPS
- adaptive traversal ms
- draw-item upload ms
- draw-item buffer reallocations
- adaptive representative count
- represented source count
- point submitted count
- persistent cache status
- runtime cache status
- cache rebuild phase/progress/ETA
- requested/displayed density
- exact versus fallback display state
- peak resident memory

Done when the project builds, tests pass, and the baseline can be repeated from
a clean app launch or after pressing `Rebuild LOD Cache Now`.

### 2. Remove Immediate Lag Sources

Prevent the current adaptive path from accidentally doing expensive raw work.

Stage 02 status: the adaptive-required path no longer silently submits full
source points, fragment budgets are carried through representative emission,
and stale async traversals are discarded by generation. Keep this section as a
regression checklist.

- When Beauty Adaptive lacks a ready hierarchy/draw-item set, use a bounded
  coarse fallback or show an explicit `waiting on adaptive LOD` state. Do not
  silently submit all source points.
- Pass the estimated fragment budget through representative emission and stop
  drawing low-priority representatives when the budget is reached.
- Cancel, replace, or supersede stale async traversal when a newer camera/style
  key arrives.
- Confirm normal adaptive draw-item updates do not call `WaitIdle()` or
  `vkDeviceWaitIdle()`.

Metrics to watch:

- point submitted count should not jump to full source in Beauty Adaptive
- estimated fragments should fall when quality demotes
- async pending time should not grow unbounded during camera movement
- draw-item GPU idle/wait should remain zero for normal adaptive updates

### 3. Improve CPU LOD Quality Selection

Make the CPU selector match the visual-cost model before moving selection to
GPU compute.

Stage 04 status: implemented in the CPU path with cache v4 node stats,
per-node scalar stats, class-aware representatives, projected spacing/mark
diameter refinement, and feature-triggered traversal diagnostics. Keep the
items below as the regression checklist for future selector work.

- Keep node spacing, density, color variance, scalar hints, normal variance, and
  emissive/accent hints populated for rebuilt and cache-loaded hierarchies.
- Keep projected spacing and projected mark radius active in refinement
  decisions.
- Keep separate cost rules for pixel screen sprites, world-mm screen sprites,
  world surfels, and camera-facing world sprites.
- Keep representative classes for spatial coverage, scalar extremes,
  emissive/accent points, color contrast, and normal/edge variation present in
  low-budget emissions.
- Continue improving deterministic rank toward richer blue-noise/class-aware
  ordering as new statistics become available.

Metrics to watch:

- representative count versus represented source count
- coverage ratio and luminance ratio from `--lod-compare`
- visible popping while moving slowly
- scalar/water/normal/source-color correctness in adaptive mode

### 4. Add Measured Governor

Replace FPS guessing with measured pass timing.

- Add Vulkan timestamp queries for point pass, depth prepass, accumulation,
  EDL/composite, and upload/submission where practical.
- Track EWMA timings per renderer mode and geometry mode.
- Drive adaptive density and fragment budgets from point-pass GPU ms, not
  present FPS alone.
- Surface an explicit `performance-limited` status when interactive quality
  floors cannot be maintained.

Metrics to watch:

- GPU point-pass ms
- GPU composite/EDL ms
- CPU scene update ms
- UI/present FPS
- selected density mode
- budget scale factor

### 5. Move Toward Ideal Cache And Streaming

Only start the `.ipcloud` cache after the current hierarchy path is stable and
measured.

- Add a manifest/chunk cache bundle beside or replacing the current single
  hierarchy cache.
- Store directly streamable raw chunks and upper LOD data.
- Render upper hierarchy data first, then stream visible chunks and deeper LOD.
- Track cache-hit load time, time to first coarse frame, upload bytes per frame,
  and peak resident memory.

Metrics to watch:

- first launch build time
- second launch time to first coarse frame
- cache hit/miss/stale reason
- visible chunk hit rate
- upload bytes per frame
- peak memory before and after chunking

### 6. Advanced Performance Work

Move to GPU-driven features after the CPU path is correct and profiled.

- Add tile overdraw estimates and per-tile fragment budgets.
- Add a conservative depth proxy or Hi-Z pyramid for occlusion culling.
- Move culling, projected-error evaluation, compaction, and indirect draw command
  generation to compute where it beats the CPU path.
- Keep a CPU fallback for portability and debugging.

Metrics to watch:

- tiles over budget
- culled hidden nodes
- compute selection ms
- indirect draw count
- point-pass ms before/after tile and occlusion work

## Checks And Metrics

Use these commands and outputs when validating LOD work:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-compare <cloud>
```

Inspect:

```text
Saved/diagnostics/lod_compare/lod_compare_metrics.json
```

Initial quality targets:

- Adaptive HQ coverage ratio should be near full source for the active style.
- Luminance ratio should stay within an acceptable band for the active style.
- Adaptive renders should not be empty or stuck on coarse fallback.
- Fast Basic should report no full-source fallback, no representative or
  fragment budget exceedance, zero large representative-jump frames, and
  `fast_basic_zoom_out_updated=true`.
- Applicable RGB/scalar/normal/emissive data should produce nonzero Adaptive HQ
  and exact Fast Basic CPU representative class counts plus nonzero
  feature-triggered refinement counts.
- Scalar, water-effect, normal, source-color, depth, and AOV lookups must remain
  source-correct through `drawItem.sourcePointIndex`.
- Stage 03 manual acceptance for the 100M Fast Basic path: slow orbit has no
  popping/flicker/stochastic crawl; slow dolly through thresholds shows gradual
  parent/child replacement; fast navigation then stop refines over multiple
  frames; repeated back-and-forth motion transitions the same areas
  consistently; HUD/trace deltas stay clamped, transitions age normally, idle
  refinement completes, and Stage 02 boundedness remains intact.

Initial performance targets:

- Moving Beauty Adaptive remains responsive on the target large cloud.
- CPU traversal does not block scene update during viewport interaction.
- Draw-item upload stays low and grow-only after initial capacity growth.
- Normal adaptive updates show zero draw-item GPU idle/wait.
- Representative count and estimated fragment count correlate with measured
  point-pass frame time.

## Defaults And Assumptions

- Treat the current branch as the implementation source of truth for the active
  adaptive LOD path.
- Keep `docs/point_cloud_adaptive_lod_fast_beauty.md` as the ideal target plan.
- Keep manual point budgets only as explicit debug/loading caps.
- Keep `Beauty Full Source` / `Full Source` as exact/debug modes, not adaptive
  fallbacks.
- Prioritize measured lag sources before ideal architecture polish.
- Do not present sampled-index controls as adaptive LOD.
