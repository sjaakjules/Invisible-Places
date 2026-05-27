# Adaptive Point-Cloud LOD Integration Audit

Date: May 28, 2026

This document compares the ideal LOD system in
`docs/point_cloud_adaptive_lod_fast_beauty.md` with the current project state.
The ideal document remains the target design and should not be edited as part of
this audit.

## Current Build Status

The current worktree builds with the Stage 07 progressive `.ipcloud` cache path:
hierarchy cache v4, visual node statistics, class-aware representatives,
per-node scalar stats, CPU traversal feature triggers, Beauty optical-depth
compensation, renderer-aware cost profiles, Vulkan timestamp diagnostics,
EWMA-governed adaptive budgets, and a `.ipcloud` v1 representative-preview
bundle that can render coarse point-cloud data before full source load/build
work completes. GPU compute selection and full raw-chunk residency remain
later-stage items.

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
- A second additive `.ipcloud` v1 cache bundle exists under
  `Saved/PointCloudCache/<stem>.<fingerprint>.ipcloud/` with `manifest.json`,
  `attribute_schema.bin`, `hierarchy.bin`, `node_pages.bin`, `node_stats.bin`,
  `lod_representatives.bin`, `scalar_stats.bin`, `raw_chunks/`,
  `build_status.json`, and `build_log.txt`.
- `.ipcloud` validation fingerprints canonical source path hash, source size,
  nanosecond mtime, PLY header hash, sampled payload hash, point/scalar counts,
  RGB/normals presence, LOD build settings version, and attribute schema
  version. Inspections report explicit missing, hit, stale, partial, and corrupt
  states with reason strings.
- `.ipcloud` publish writes into `<bundle>.tmp`, records build phases in
  `build_status.json`, validates all required files, then publishes by directory
  rename. Temporary bundles are ignored as complete caches; partial bundles can
  drive resume/restart decisions.
- Legacy v4 single-file hierarchy caches remain non-destructive fallback assets
  after full source load. They are not counted as `.ipcloud` warm caches because
  they lack the representative attribute payload needed for pre-full-load
  preview.
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
  feature statistics, with separate renderer profiles for Fast Basic square
  points, Beauty screen sprites, Beauty world-mm sprites, and Beauty world
  surfels.
- The 32-byte `PointCloudDrawItemGpu` ABI is unchanged. Existing fields carry
  source index, represented source count, seed, packed renderer/class/clamp
  metadata, raw footprint area, opacity scale, emission scale, and render area.
- Beauty compensation uses optical-depth opacity scaling from represented count
  and raw-vs-LOD footprint area. Ordinary representatives preserve average
  emission, emissive/accent representatives receive capped coverage scaling, and
  Beauty traversal keeps one representative per visible frontier node under
  fragment pressure so expensive styles degrade by adapting representatives
  instead of dropping unvisited cloud regions.
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
  representative class counts, feature-triggered refinement counts, renderer
  cost profile, radius/opacity/emission scale ranges, estimated
  vertices/fragments/blended fragments, and opacity/emission/performance clamp
  status.
- The viewport checks Vulkan timestamp support from `timestampPeriod` and
  graphics-queue `timestampValidBits`, creates per-frame query pools, writes
  timestamp pairs around Fast Basic point, Beauty depth, Beauty accumulation,
  composite, and postprocess/EDL passes, and reads previous-frame results
  without `WAIT_BIT`.
- Adaptive quality no longer uses scene/present FPS thresholds. A pure
  `PointCloudPerformanceGovernor` holds separate EWMA and budget-scale state
  for Fast Basic, Beauty screen sprites, Beauty world-mm sprites, and Beauty
  world surfels. It scales representative, fragment, blended-fragment,
  target-spacing, and draw-item upload-byte budgets gradually, holds cached
  frames, and reports explicit `visually lossless`, `performance-limited`, or
  conservative timestamp fallback status.
- PLY load progress reports points read, payload bytes read, elapsed time, and
  ETA when enough samples exist.
- LOD hierarchy/cache rebuild progress reports elapsed time, approximate ETA,
  source references processed, nodes built, representatives built, and current
  build depth.
- The status HUD and diagnostics expose adaptive requested/displayed density,
  movement quality tier, cache filename/size, hierarchy source/node/rep counts,
  runtime cache hit/miss, async state, and whether the displayed adaptive buffer
  is exact or coarse fallback.
- The status HUD, LOD debug panel, and `--lod-cache-check` expose `.ipcloud`
  cache state/reason, load phase, build phase, publish/resume status,
  preview/full mode, representative preview count, represented source count,
  time to bounds/placeholder, time to first coarse frame, and time to first
  refined frame.
- Cold point-cloud loads can parse header/schema, upload a remapped
  representative preview `LoadedPointCloud`, render that preview through the
  existing viewport upload path, and then replace it with the full source cloud
  and exact adaptive hierarchy when ready. Warm `.ipcloud` loads validate
  `manifest.json`, load representative payload first, upload a dense preview,
  and hand off to the existing adaptive renderer without changing the 32-byte
  `PointCloudDrawItemGpu` ABI.
- `--lod-compare` exists and writes full-source/adaptive EXRs plus
  `lod_compare_metrics.json` and a Fast Basic per-frame transition trace CSV.
  It now renders a repeatable Beauty matrix covering small opaque sprites,
  large translucent Gaussian sprites, emissive/scalar sprites, world-sized
  sprites, and world surfels when normals are present.
- `lod_compare_metrics.json` reports Adaptive HQ representative class counts,
  exact Fast Basic CPU representative class counts, viewport Fast Basic
  boundedness/smoothness metrics, Beauty matrix renderer profiles,
  radius/opacity/emission ranges, estimated vertex/fragment/blended-fragment
  costs, clamp flags, raw/EWMA GPU point-pass timings, governor budget scale,
  timestamp support/fallback state, Beauty stress metrics, and colour, scalar,
  normal, and emissive/accent feature-triggered refinement counts.
- Stage 05 sample evidence on `Data/Site3-Sample-Terrestrial.ply` reports exact
  Adaptive HQ Beauty matrix output with no fallback. Matrix luminance ratios:
  small opaque 0.805370, large translucent Gaussian 0.831266,
  emissive/scalar 0.967904, world-sized sprite 1.118410, and world surfel
  1.139980. The measurable Stage 05 mismatch fixed during implementation was
  the large translucent case: the first run had luminance ratio 0.503523 with
  218,755 represented source points; after preserving Beauty frontier coverage
  under fragment pressure it reran at 0.831266 with 535,332 represented source
  points.
- Stage 05 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` reports exact
  Adaptive HQ Beauty matrix output with no fallback. Matrix luminance ratios:
  small opaque 0.666881, large translucent Gaussian 0.754400,
  emissive/scalar 0.950587, world-sized sprite 0.880937, and world surfel
  0.600746. The primary small-sprite case used 1,729,641 Adaptive HQ
  representatives covering 99,504,849 source points, with feature
  representatives colour/scalar/normal/accent 17,878/138,540/7,775/16,371.
- The 100M Fast Basic viewport trace reports deterministic smoothness metrics:
  max submitted 262,132 under an 8,294,400 representative budget, max estimated
  fragments 1,108,920 under a 796,262,000 fragment budget, max absolute
  representative delta 5,640, 22 transition-active frames, 0 large
  representative-jump frames, 0 budget-exceeded frames, 0 full-source fallback
  frames, and `fast_basic_zoom_out_updated=true`.
- Stage 06 sample evidence on `Data/Site3-Sample-Terrestrial.ply` reports
  Vulkan timestamps supported with `valid previous frame` state. Fast Basic
  reached max GPU point pass 1.02421 ms, EWMA 0.0213727 ms, governor scale 1,
  max submitted 262,115 under a 4,823,449 representative budget, max estimated
  fragments 956,657 under a 915,702,000 fragment budget, 0 performance-limited
  frames, 0 large representative-jump frames, no budget exceedance, and no
  full-source fallback. The Beauty stress pass reached max GPU point pass
  1.14775 ms, EWMA 0.482701 ms, governor scale 1, max estimated blended
  fragments 46,212,000 under a 152,617,000 blended budget, no budget
  reached/exceeded state, and no full-source fallback.
- Stage 06 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` also reports
  Vulkan timestamps supported with `valid previous frame` state. Fast Basic
  reached max GPU point pass 0.16225 ms, EWMA 0.0172543 ms, governor scale 1,
  max submitted 262,132 under a 4,823,449 representative budget, max estimated
  fragments 1,108,920 under a 915,702,000 fragment budget, 0
  performance-limited frames, 0 large representative-jump frames, no budget
  exceedance, and no full-source fallback. Beauty stress reached max GPU point
  pass 0.934542 ms, EWMA 0.13772 ms, governor scale 1, max estimated blended
  fragments 46,213,100 under a 152,617,000 blended budget, no budget
  reached/exceeded state, and no full-source fallback.
- Stage 07 cache evidence on `Data/Site3-Sample-Terrestrial.ply` reports cold
  cache `missing`, 65,536 source-sampled preview representatives covering
  12,183,742 source points, time to bounds/coarse frame 2,663.56 ms, full source
  load 6,078.93 ms, hierarchy build 82,512.1 ms, time to first refined frame
  91,254.6 ms, `.ipcloud` publish 717.66 ms, warm cache `hit`, warm time to
  first coarse frame 304.21 ms, and a synthetic interrupted `.tmp` bundle marked
  resumable with `raw_chunks_completed=1/4`. The run wrote
  `Saved/diagnostics/lod_cache/lod_cache_metrics.json`.
- Stage 07 renderer evidence after the cache changes kept the existing adaptive
  path intact on the sample: coverage ratio 1, luminance ratio 0.802810,
  1,564,580 Adaptive HQ representatives covering 12,183,742 source points,
  Fast Basic max submitted 2,880,233 under a 4,823,449 representative budget,
  max estimated fragments 4,681,200 under a 915,702,000 fragment budget, no
  budget exceedance, and no full-source fallback.

## Partially Implemented

These pieces exist, but they are not yet the ideal system described in
`point_cloud_adaptive_lod_fast_beauty.md`.

- Stage 07 `.ipcloud` `raw_chunks/` files are validated source-range metadata
  rather than full independently streamable attribute chunks. Stage 08 must turn
  those ranges into real raw chunk residency, memory mapping, upload scheduling,
  and LRU behavior.
- Warm `.ipcloud` loading currently accelerates representative preview display.
  The exact full hierarchy handoff still relies on the existing full
  `LoadedPointCloud` path and v4 hierarchy cache/build path once source load is
  complete.
- Hierarchy build is asynchronous and reports progress, but the build itself is
  still monolithic and not cancellable until `BuildPointCloudLodHierarchy(...)`
  returns.
- Traversal is CPU-only. There is no GPU compute culling, compaction, indirect
  draw generation, or GPU-driven visible-chunk selection.
- Manual sampled point budgets still exist and are uploaded as sampled index
  buffers. The adaptive path treats them mostly as a debug/loading cap, but the
  UI still exposes a generic `Budget` control that can be confused with LOD.
- LOD compensation exists as footprint, optical-depth opacity, and emission
  fields, but it still lacks author-facing policy controls, Beauty-specific
  `lodBlend`, parent/child crossfade, and feature-preserving palettes.
- Pixel screen sprites, world-mm screen sprites, world surfels, and
  camera-facing world sprites now have separate CPU selection and cost tuning,
  plus measured GPU timing profiles, but they still do not have independent
  authorable policies.
- The LOD comparison metrics now report coverage, mean luminance, feature class
  counts, feature-triggered refinement counts, raw/EWMA GPU point-pass timing,
  governor state, Beauty stress timing, and a Fast Basic transition trace, but
  not image error maps, peak memory, upload bandwidth, upload timing, or tile
  overdraw budgets.
- The 100M `--lod-compare` path proves bounded replacement and deterministic
  trace metrics, but it does not automatically prove final exact idle
  refinement completion. The current 500-frame harness still ends with async
  refinement pending under the no-flash fallback guard, so final Stage 03 visual
  smoothness acceptance still requires the manual checklist.

## Not Implemented

These are still target-system items from the ideal plan.

- Raw chunk streaming and visible-chunk residency from `.ipcloud` bundles.
- Direct exact-hierarchy attach from `.ipcloud` without first assembling the
  full source cloud.
- Memory-mapped source chunks and CPU/GPU LRU caches.
- Device-local static point/chunk buffers with staging uploads as the normal
  large-cloud path.
- Beauty-specific parent/child opacity crossfade transition policy.
- Dedicated LOD style data block with explicit compensation/footprint policy.
- Upload and render-state submission timestamp diagnostics; point/depth/
  accumulation/composite/postprocess GPU timing is implemented for viewport
  diagnostics.
- Tile overdraw estimates and per-tile fragment/blended-fragment budgets.
- Conservative occlusion culling, depth proxy, or Hi-Z pyramid.
- GPU compute traversal/culling/compaction and indirect draw submission.
- Runtime metrics for peak resident memory and tile budget pressure.

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
- The quality controller now reacts to measured GPU point-pass time, but upload
  stalls and tile-local overdraw are still only inferred through draw-item byte
  and global fragment/blended-fragment budgets.
- Current projected spacing now uses stored node spacing and feature statistics,
  but there is still no tile-pressure model.
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
- Pass the estimated fragment budget through representative emission. Fast Basic
  stops drawing low-priority representatives when the budget is reached; Beauty
  records fragment pressure while still preserving at least one representative
  for each visible frontier node.
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

Stage 05 status: implemented in the CPU path with cache v4 node stats,
per-node scalar stats, class-aware representatives, projected spacing/mark
diameter refinement, renderer-specific cost profiles, Beauty compensation, and
feature-triggered traversal diagnostics. Keep the items below as the regression
checklist for future selector work.

- Keep node spacing, density, color variance, scalar hints, normal variance, and
  emissive/accent hints populated for rebuilt and cache-loaded hierarchies.
- Keep projected spacing and projected mark radius active in refinement
  decisions.
- Keep separate cost rules for pixel screen sprites, world-mm screen sprites,
  world surfels, and camera-facing world sprites.
- Keep Beauty optical-depth opacity, ordinary emission averaging,
  emissive/accent emission scaling, and performance clamp flags active without
  widening the draw-item ABI.
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

### 4. Extend Measured Governor

Stage 06 replaced FPS guessing with measured pass timing for viewport point
passes. Keep this section for the remaining timing and policy follow-up work.

- Add direct upload/submission timing where practical; Stage 06 currently
  governs draw-item upload bytes from point-pass timing rather than measured
  upload stalls.
- Add tile/overdraw budget diagnostics if Beauty styles need more local
  pressure control than the current global fragment/blended-fragment budgets.
- Keep exercising over-target hardware/views so the clamped budget-scale path
  is validated by runtime stress as well as focused governor tests.
- Preserve the explicit `performance-limited` status when interactive quality
  floors cannot be maintained.

Metrics to watch:

- GPU point-pass ms
- GPU composite/EDL ms
- CPU scene update ms
- UI/present FPS
- selected density mode
- budget scale factor

### 5. Move Toward Ideal Cache And Streaming

Stage 07 started the `.ipcloud` cache after the hierarchy path was stable and
measured. Keep this section as the Stage 08 streaming checklist.

- Preserve the additive `.ipcloud` bundle beside the v4 single-file hierarchy
  cache until full streaming has its own validation evidence.
- Replace Stage 07 raw range metadata with directly streamable raw attribute
  chunks and visible-chunk residency.
- Attach exact hierarchy/page data directly from `.ipcloud` when possible,
  while keeping representative preview first.
- Track upload bytes per frame, warm-cache exact handoff time, and peak resident
  memory before and after chunking.

Metrics to watch:

- first launch build time
- second launch time to first coarse frame
- cache hit/miss/stale reason
- publish/resume status
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
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-cache-check <cloud>
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-compare <cloud>
```

Inspect:

```text
Saved/diagnostics/lod_compare/lod_compare_metrics.json
Saved/diagnostics/lod_cache/lod_cache_metrics.json
```

Initial quality targets:

- Adaptive HQ coverage ratio should be near full source for the active style.
- Luminance ratio should stay within an acceptable band for the active style.
- Beauty matrix coverage should remain 1, with exact/no-fallback results for
  small opaque, large translucent Gaussian, emissive/scalar, world-sized sprite,
  and world-surfel cases when normals are available.
- Adaptive renders should not be empty or stuck on coarse fallback.
- Fast Basic should report no full-source fallback, no representative or
  fragment budget exceedance, zero large representative-jump frames, and
  `fast_basic_zoom_out_updated=true`.
- Applicable RGB/scalar/normal/emissive data should produce nonzero Adaptive HQ
  and exact Fast Basic CPU representative class counts plus nonzero
  feature-triggered refinement counts.
- Scalar, water-effect, normal, source-color, depth, and AOV lookups must remain
  source-correct through `drawItem.sourcePointIndex`.
- `.ipcloud` checks should report explicit cold/warm cache state and reason,
  representative preview count, time to first coarse frame, publish status, and
  interrupted/resume decision. A legacy v4 hierarchy cache alone should not be
  reported as a `.ipcloud` warm hit.
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
