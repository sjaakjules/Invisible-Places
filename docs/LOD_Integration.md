# Adaptive Point-Cloud LOD Integration Audit

Date: May 28, 2026

This document compares the ideal LOD system in
`docs/point_cloud_adaptive_lod_fast_beauty.md` with the current project state.
The ideal document remains the target design and should not be edited as part of
this audit.

## Current Build Status

The current worktree builds with the progressive `.ipcloud` streaming path:
hierarchy cache v4, visual node statistics, class-aware representatives,
per-node scalar stats, CPU traversal feature triggers, Beauty optical-depth
compensation, renderer-aware cost profiles, Vulkan timestamp diagnostics,
EWMA-governed adaptive budgets, and a `.ipcloud` v2 bundle that can render
coarse representative data first, attach the cached exact hierarchy, and stream
visible raw chunks into compact resident buffers without requiring a permanent
full PLY parse/upload for normal Fast Basic or Beauty Adaptive viewport use.
Warm streaming-preview viewports are held in bounded Fast Adaptive Preview until
exact source data is resident, so startup cannot immediately promote a coarse
preview into a multi-million-item Adaptive HQ resident rebuild while raw chunks
are still queued.
Stage 09 adds explicit export density policy semantics, deterministic adaptive
export traversal, Full Source exact-source guards, Match Viewport still-export
snapshotting, Fast Adaptive Preview Quick MP4 labeling, export logs, and
deterministic/error-map diagnostics.
Stage 10 adds conservative 32 px tile-pressure diagnostics and Beauty
tile-budget limiting for low-priority representatives, keeps Fast Basic
diagnostic-only, keeps Full Source exact, and reports conservative culling as
disabled or uncertain until a safe depth proxy or reliable normal metadata
exists. `--lod-compare` now writes schema v3 tile/culling fields.
Stage 11 adds runtime GPU-driven selection feature checks, CPU/GPU parity policy
helpers, indirect draw diagnostics, `vkCmdDrawIndirect` submission for large
adaptive draw-item paths when supported, and compute-generated indirect command
output for eligible viewport draws. GPU compute selection remains guarded behind
feature, parity, and timing proof; on the local MoltenVK runtime the current
reported path is `cpu-selection+gpu-generated-indirect`. Memory-mapped chunks and
fully device-local chunk residency remain later-stage items.

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
- A second additive `.ipcloud` v2 cache bundle exists under
  `Saved/PointCloudCache/<stem>.<fingerprint>.ipcloud/` with `manifest.json`,
  `attribute_schema.bin`, `hierarchy.bin`, `node_pages.bin`, `node_stats.bin`,
  `lod_representatives.bin`, `scalar_stats.bin`, `node_raw_chunks.bin`,
  `raw_chunks/`, `build_status.json`, and `build_log.txt`. Stage 07 v1
  bundles are stale for streaming and are rebuilt into v2.
- `.ipcloud` v2 raw chunks are independently decodable spatial payloads with
  chunk id, bounds, point/scalar counts, encoded/decoded byte counts, attribute
  flags, original source IDs, quantized chunk-local positions, packed RGBA,
  optional packed normals, and float32 scalar field blocks. `node_raw_chunks.bin`
  sidecars map hierarchy nodes to intersecting chunks without changing the
  hierarchy node struct.
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
- Adaptive traversal includes a 32 px tile accumulator. Fast Basic records tile
  pressure diagnostics without using tile pressure to reduce output. Beauty
  Adaptive uses per-tile fragment/blended-fragment budgets only for extra
  low-priority representatives, preserves the first representative per visible
  frontier node, and protects scalar, emissive/accent, color-contrast, and
  normal/edge representatives from tile-only rejection. Full Source bypasses
  tile limiting.
- GPU-driven selection policy support exists. The renderer checks compute-queue
  availability, storage-buffer limits, indirect draw, indirect count, MoltenVK
  portability subset state, parity status, and CPU/GPU timing before allowing a
  compute selector to replace the CPU selector. Large adaptive draw-item paths
  submit through `vkCmdDrawIndirect` when supported; current viewport indirect
  commands can be generated by a compute dispatch from the CPU-selected compact
  draw-item count, so selection semantics remain unchanged. EXR command buffers
  and unsupported/small paths keep the CPU-generated or direct fallback.
- Conservative occlusion/backface/depth-proxy culling diagnostics are wired
  through traversal, cache state, viewport diagnostics, HUD/debug UI, and
  compare metrics. Actual rejection remains disabled when metadata or depth
  confidence is missing; translucent/no-depth Beauty styles report disabled
  culling, and depth-capable styles without a safe proxy report uncertain
  culling with zero rejected nodes/source points.
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
- Export density modes share a policy helper with label, log description,
  full-source flag, preview-quality flag, viewport-snapshot requirement,
  deterministic-selection flag, and artistic flag. Adaptive HQ, Artistic HQ, and
  Fast Preview exports use deterministic fixed traversal parameters; Full Source
  is exact/debug only; Match Viewport modes require a captured displayed
  adaptive selection.
- Adaptive EXR/MP4 export traversal is synchronous and deterministic. It skips
  async viewport reuse, previous-frontier hysteresis, manual point-budget caps,
  and interactive governor scaling. Source/debug renderer modes are normalized
  back to adaptive renderers unless the density mode is explicitly Full Source.
- Full Source export refuses representative-preview/chunk-only state with a
  clear error instead of substituting representatives. Quick MP4 remains
  explicitly `Fast Adaptive Preview`.
- Match Viewport still-camera export captures displayed adaptive draw items,
  frontier/style diagnostics, and camera state at export start. Animation EXR
  export rejects Match Viewport modes until approved per-frame viewport states
  exist.
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
  and exact adaptive hierarchy when a full source path is explicitly needed.
  Warm `.ipcloud` v2 loads validate `manifest.json`, load representative payload
  first, upload a dense preview, attach the exact cached hierarchy plus raw chunk
  catalog, and render normal Fast Basic / Beauty Adaptive viewports by streaming
  compact resident chunks. The 32-byte `PointCloudDrawItemGpu` ABI is unchanged;
  draw-item `sourcePointIndex` values are remapped into resident dense indices.
- While that warm preview is still chunk-streaming and exact source data is not
  resident, viewport density selection stays at `Fast Adaptive Preview` and
  traversal caps draw items/representatives at 131,072 with Fast Preview spacing
  and fragment pressure. Exports remain governed by their explicit density
  policies and do not use this interactive warm-preview throttle.
- The viewport exposes `.ipcloud` residency diagnostics: visible chunks
  requested/resident/missing, CPU resident bytes, GPU resident bytes, peak RSS,
  upload bytes and upload budget, upload queue length, chunk hit rate, eviction
  count/reason, and full-source fallback reason.
- `--lod-stream-check <cloud>` runs a repeatable warm-cache pan path, records
  baseline dense memory estimates, streaming resident memory, upload budget
  use, queue/eviction state, and writes
  `Saved/diagnostics/lod_stream/lod_stream_metrics.json`.
- `--lod-compare` exists and writes full-source/adaptive EXRs plus
  error-map EXRs, `lod_compare_metrics.json`, and a Fast Basic per-frame
  transition trace CSV. It now renders a repeatable Beauty matrix covering small
  opaque sprites, large translucent Gaussian sprites, emissive/scalar sprites,
  world-sized sprites, and world surfels when normals are present.
- `lod_compare_metrics.json` schema v3 reports source/cache fingerprints,
  density policy fields, deterministic adaptive selection and image/error-map
  hashes, per-channel/RGB/luminance/alpha MAE/RMSE/max error, Adaptive HQ
  representative class counts, exact Fast Basic CPU representative class counts,
  viewport Fast Basic boundedness/smoothness metrics, Beauty matrix renderer
  profiles, radius/opacity/emission ranges, estimated vertex/fragment/
  blended-fragment costs, clamp flags, raw/EWMA GPU point-pass timings, governor
  budget scale, timestamp support/fallback state, tile pressure and conservative
  culling fields, Beauty stress metrics, and colour, scalar, normal, and
  emissive/accent feature-triggered refinement counts.
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
- Stage 08 streaming evidence on a warm `.ipcloud` v2 cache for
  `Data/Site3-Sample-Terrestrial.ply` reports cache `hit`, 65,536 preview
  points loaded in 364.151 ms, 2,292 visible chunks requested on each pan-path
  pass, 589-599 chunks resident under budget, CPU residency 120.9 MiB, compact
  GPU residency 128.0 MiB, upload 128.0 MiB / 128.0 MiB, queued/missing chunks
  explicitly evicted for `upload budget`, and peak RSS 960.0 MiB versus a
  1.80 GiB dense full-source CPU+GPU baseline estimate. The repeat center pass
  remapped 16,233 / 56,537 draw items from resident chunks before the rest of
  the visible chunks were resident.
- Stage 08 sample `--lod-compare Data/Site3-Sample-Terrestrial.ply` reports no
  Fast Basic or Beauty Adaptive fallback after streaming changes: coverage ratio
  1, luminance ratio 0.802810, Beauty stress max GPU point pass 0.324792 ms with
  no budget exceedance, and Fast Basic viewport max submitted 2,863,943 under a
  4,823,449 representative budget with 0 large representative-jump frames.
- Stage 08 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` confirms the
  same path at scale. First v2 bundle construction still required the dense
  source path: full source load 54,060.8 ms, hierarchy build 990,733 ms, and raw
  chunk publish 83,108.9 ms. The subsequent warm-cache stream check loaded the
  65,536-point preview in 315.787 ms, requested 1,903 chunks, kept CPU
  residency at 120.9 MiB, compact GPU residency/upload at 128.0 MiB / 128.0 MiB,
  and peaked at 4.57 GiB RSS versus a 14.91 GiB dense CPU+GPU baseline estimate.
  The 100M `--lod-compare` run then reported no Fast Basic or Beauty Adaptive
  full-source fallback, no budget exceedance, and 0 large representative-jump
  frames; coverage ratio 1, luminance ratio 0.649889, Beauty stress max GPU
  point pass 0.202458 ms, and Fast Basic viewport max submitted 1,084,047 under
  a 1,810,284 representative budget.
- Stage 09 sample export-determinism evidence on
  `Data/Site3-Sample-Terrestrial.ply` ran `--lod-compare` twice with identical
  inputs, then reran once after the final Full Source/Quick MP4 guard. The
  deterministic fields matched exactly: adaptive selection hash
  `0x282219d27c49476c`, full-source image hash `0x46256cddd083a685`, adaptive
  image hash `0xfb7a0106ae18cdcb`, error-map hash `0x1f382b69f57ce81c`,
  1,874,048 representatives, 12,183,742 represented source points, coverage
  ratio 1, luminance ratio 0.804748, RGB MAE 0.0152239, RGB RMSE 0.0789348, and
  alpha RMSE 0. All five Beauty matrix cases matched on selection/image/error
  hashes and representative counts. The final stream check passed with center
  and repeat-center chunk residency matching at 599 / 2,292 requested chunks,
  16,233 / 56,537 remapped draw items, 120.9 MiB CPU residency, and 128.0 MiB /
  128.0 MiB compact GPU upload.
- Stage 09 warm-preview responsiveness evidence on the same sample reran after
  the viewport throttle and kept the stream check deterministic at 599 / 2,292
  requested chunks, 16,233 / 56,537 remapped draw items, 120.9 MiB CPU
  residency, 128.0 MiB GPU residency/upload, and 28.7122% chunk hit rate. A
  repeat `--lod-compare` pair matched stable fields exactly: selection hash
  `0xe135407d7f51ed5c`, full-source hash `0x3033fb94fdde875f`, adaptive hash
  `0x6a742042412c541d`, error-map hash `0x1c76cf9d00a8ad1b`, 401,553
  representatives, 12,183,742 represented source points, coverage ratio 1,
  luminance ratio 0.773691, RGB MAE 0.0193535, and alpha MAE 0.
- Stage 09 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` completed one
  warm-cache `--lod-compare` and wrote Full Source, Adaptive HQ, and error-map
  EXRs with metrics schema v2. The run reported selection hash
  `0xc2334f1749bab8d1`, full-source image hash `0xd66034c701054d00`, adaptive
  image hash `0xb4ec01582968a865`, error-map hash `0x482eee74d3c3d993`,
  1,729,641 Adaptive HQ representatives covering 99,504,849 source points out
  of 100,743,210, coverage ratio 1, luminance ratio 0.666881, RGB MAE
  0.0280766, RGB RMSE 0.121346, and alpha RMSE 0. The run used a ready LOD cache
  and reported `.ipcloud` full source loaded.
- Stage 10 sample evidence on `Data/Site3-Sample-Terrestrial.ply` completed
  build, focused LOD tests, full CTest, stream check, and repeat
  `--lod-compare` runs with metrics schema v3. The stream check passed with center/repeat-center
  563 / 2,292 requested chunks, 43,438 / 56,537 remapped draw items, 120.9 MiB
  CPU residency, 128.0 MiB GPU residency/upload, and 76.8311% chunk hit rate.
  The final repeatable compare pair matched stable fields exactly with
  selection/full/adaptive/error-map hashes `0x64f020924ab61fdd` /
  `0x46256cddd083a685` / `0x76a7253132d047ea` / `0x9ff3c47aff352c38`,
  coverage ratio 1, luminance ratio 0.804235, RGB MAE 0.0152765,
  1,824,536 Adaptive HQ representatives covering 11,848,474 source points,
  Beauty stress max GPU point pass 0.267583 ms, tile-limited reps/nodes
  1,457,653 / 15,783, max tile blended pressure 130,769 against the 65,536
  tile budget, culling disabled for the no-depth translucent stress style, no
  adaptive/full-source fallback, and no budget exceedance.
- Stage 10 full-cloud evidence on `Data/Site3-Mid-1mm100M.ply` completed
  `--lod-compare` with coverage ratio 1, luminance ratio 0.658456, RGB MAE
  0.0285489, 1,191,182 Adaptive HQ representatives covering 94,091,952 source
  points, Beauty stress max GPU point pass 0.572459 ms, tile-limited reps/nodes
  1,002,773 / 22,345, max tile blended pressure 364,341 against the 65,536 tile
  budget, culling disabled, no adaptive/full-source fallback, and no budget
  exceedance.
- Stage 11 sample evidence on `Data/Site3-Sample-Terrestrial.ply` completed
  build, focused GPU/LOD tests, full CTest, stream check, and `--lod-compare`.
  The local MoltenVK runtime reported GPU-selection feature support plus
  indirect-count support; the runtime path now reports
  `cpu-selection+gpu-generated-indirect` for eligible viewport draws because a
  compute shader writes the indirect command before the render pass. CPU
  traversal remains authoritative because full CPU/GPU compute-selection parity
  and timing are not yet available. Fast Basic recorded 1 GPU indirect-command
  dispatch at 0.318041 ms, 1 indirect draw, and 262,115 submitted vertices.
  Beauty stress recorded 1 GPU indirect-command dispatch at 0.294209 ms, 2
  indirect draws, and 524,264 submitted vertices. Coverage ratio stayed 1,
  luminance ratio 0.804235, RGB MAE 0.0152765, selection hash
  `0x64f020924ab61fdd`, Adaptive HQ used 1,824,536 representatives covering
  11,848,474 source points, and the run reported no adaptive/full-source
  fallback or budget exceedance. The stream check still passed at
  center/repeat-center 563 / 2,292 chunks, 43,438 / 56,537 remapped draw items,
  120.9 MiB CPU residency, 128.0 MiB GPU residency/upload, and 76.8311% chunk
  hit rate.

## Partially Implemented

These pieces exist, but they are not yet the ideal system described in
`point_cloud_adaptive_lod_fast_beauty.md`.

- Streaming residency is a first CPU implementation: chunks are decoded into
  compact `LoadedPointCloud` resident sets and uploaded through the existing
  host-visible dense descriptor shape. The normal same-capacity streaming update
  avoids a renderer `WaitIdle()`, but initial allocation/growth can still
  synchronize.
- The current visible request set is derived from adaptive frontier/node chunk
  mappings. It preserves progressive coarse rendering and bounded upload, but it
  does not yet include async prefetch, memory mapping, or fine-grained persistent
  GPU LRU reuse across every viewport pass.
- Beauty Full Source, Fast Basic Source, explicit Full Source export, picking,
  and editing keep the dense compatibility path when exact/debug semantics
  require it. Adaptive HQ/Fast Preview export uses deterministic adaptive draw
  items and chunk-streamed resident uploads where available. The smallest safe
  remaining migration is to route exact/debug picking/editing and future Full
  Source chunk iteration through a chunk query API backed by the v2 decoder and
  source-ID tables before deleting the dense `LoadedPointCloud` requirement.
- First-time `.ipcloud` v2 construction still performs a full source parse and
  monolithic hierarchy build. Warm-cache adaptive rendering avoids permanent
  full parse/upload duplication, but cache creation progress/cancel points
  remain limited by `BuildPointCloudLodHierarchy(...)`.
- Hierarchy build is asynchronous and reports progress, but the build itself is
  still monolithic and not cancellable until `BuildPointCloudLodHierarchy(...)`
  returns.
- Traversal selection is still CPU-authored by default. GPU compute culling and
  selection compaction are not active until parity and timing pass, while
  indirect draw submission and compute-generated indirect commands are active for
  large viewport adaptive draw-item paths where Vulkan supports them.
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
  governor state, Beauty stress timing, deterministic selection/image hashes,
  image error-map metrics, tile pressure, conservative culling state, and a Fast
  Basic transition trace. Streaming memory/upload diagnostics are emitted by
  `--lod-stream-check` and summarized in export logs where available.
- The 100M `--lod-compare` path proves bounded replacement and deterministic
  trace metrics, but it does not automatically prove final exact idle
  refinement completion. The current 500-frame harness still ends with async
  refinement pending under the no-flash fallback guard, so final Stage 03 visual
  smoothness acceptance still requires the manual checklist.

## Not Implemented

These are still target-system items from the ideal plan.

- Memory-mapped source chunks and fully persistent async CPU/GPU LRU caches.
- Device-local static point/chunk buffers with staging uploads as the normal
  large-cloud path; current streaming uses compact resident buffers in the
  existing descriptor shape.
- Beauty-specific parent/child opacity crossfade transition policy.
- Dedicated LOD style data block with explicit compensation/footprint policy.
- Upload and render-state submission timestamp diagnostics; point/depth/
  accumulation/composite/postprocess GPU timing is implemented for viewport
  diagnostics.
- Active depth-proxy, backface, or Hi-Z occlusion rejection; Stage 10 only
  reports disabled/uncertain conservative culling states until correctness is
  provable.
- GPU compute traversal/culling/selection compaction and indirect-count
  submission.

## Likely Lag Sources

Investigate these before adding more ideal-plan features.

- First-run large-cloud loading can still spend a long time in PLY parse or
  monolithic hierarchy build before the v2 bundle can publish. The UI now
  reports loader/cache phases where available, but hierarchy construction still
  needs deeper progress/cancel checkpoints.
- Beauty Adaptive full-source fallback is guarded now, but this remains a
  regression risk when future density modes or loading paths bypass adaptive
  draw-item requirements.
- Async traversal discards stale completions now, but new camera/style keys can
  still spend visible time in coarse fallback while replacement work finishes.
- The quality controller now reacts to measured GPU point-pass time, and Stage
  10 reports tile-local pressure. Upload stalls are still inferred through
  draw-item byte budgets, and over-budget tiles can intentionally remain
  over-budget when preserving visible frontier or protected feature reps.
- Current projected spacing now uses stored node spacing and feature
  statistics. The tile-pressure model is conservative and CPU-side; it is not a
  substitute for a future depth proxy, Hi-Z pyramid, or GPU-driven compaction.
- Dense exact/debug/export surfaces still upload and retain source data in
  several forms: `LoadedPointCloud`, `cpuPositions`, vertex position buffer,
  storage position buffer, color buffer, normal buffer, scalar buffer, and
  hierarchy data. Warm `.ipcloud` v2 Fast Basic and Beauty Adaptive viewport
  rendering bypass that permanent full-source duplication by using compact raw
  chunk residency.
- Static point buffers are host-visible in the current path. This is simple but
  not the ideal MoltenVK/Apple-GPU path for very large mostly-static data.
- The manual sampled budget path calls `WaitIdle()` during point-budget updates.
  That is acceptable for explicit debug/loading cap changes, but it must not be
  part of normal adaptive updates.
- Tile budgets now reduce low-priority Beauty representatives, but close,
  grazing, long-site views with large translucent marks can still exceed local
  budgets when conservative correctness rules choose drawing too much over
  culling visible data.

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
- Keep Stage 10 tile-pressure diagnostics active as the local complement to the
  global fragment/blended-fragment budgets.
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

Stage 08 added v2 raw chunk payloads and bounded warm-cache residency after the
hierarchy path was stable. Keep this section as the streaming regression and
Stage 09+ migration checklist.

- Preserve the additive `.ipcloud` bundle beside the v4 single-file hierarchy
  cache until export/debug paths no longer need dense compatibility fallback.
- Keep v2 raw chunks directly streamable and keep v1 bundles stale for streaming.
- Keep exact hierarchy/page data attached directly from `.ipcloud` while the
  representative preview appears first.
- Keep upload bytes per frame, warm-cache panning, resident CPU/GPU bytes, peak
  RSS, and fallback reason diagnostics in `--lod-stream-check`.
- Move exact/debug/export/picking to chunk iteration before deleting the dense
  full-source path.

Metrics to watch:

- first launch build time
- second launch time to first coarse frame
- cache hit/miss/stale reason
- publish/resume status
- visible chunk hit rate
- upload bytes per frame
- peak memory before and after chunking
- full-source fallback reason

### 6. Advanced Performance Work

Move further GPU-driven features only after the CPU path remains correct and
the next GPU candidate is profiled.

- Tile overdraw estimates and first CPU-side per-tile fragment budgets are
  implemented for Beauty Adaptive. Next, add a conservative depth proxy or Hi-Z
  pyramid before enabling actual hidden-node rejection.
- Keep the current indirect draw submission path guarded by runtime diagnostics.
  Move culling, projected-error evaluation, compaction, and GPU-generated
  indirect commands to compute only where they beat the CPU path.
- Keep a CPU fallback for portability and debugging.

Metrics to watch:

- tiles over budget
- culled hidden nodes
- compute selection ms
- indirect draw count
- GPU-selection path/fallback/parity status
- point-pass ms before/after tile and occlusion work

## Checks And Metrics

Use these commands and outputs when validating LOD work:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-cache-check <cloud>
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-stream-check <cloud>
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-compare <cloud>
```

Inspect:

```text
Saved/diagnostics/lod_compare/lod_compare_metrics.json
Saved/diagnostics/lod_compare/*_error.exr
Saved/diagnostics/lod_cache/lod_cache_metrics.json
Saved/diagnostics/lod_stream/lod_stream_metrics.json
```

Initial quality targets:

- Adaptive HQ coverage ratio should be near full source for the active style.
- Luminance ratio should stay within an acceptable band for the active style.
- Beauty matrix coverage should remain 1, with exact/no-fallback results for
  small opaque, large translucent Gaussian, emissive/scalar, world-sized sprite,
  and world-surfel cases when normals are available.
- Adaptive renders should not be empty or stuck on coarse fallback.
- Fast Basic should report no full-source fallback, no representative or
  fragment budget exceedance, no unexplained large representative-jump frames,
  and `fast_basic_zoom_out_updated=true`.
- Repeated `--lod-compare` runs should match on deterministic selection hashes,
  full/adaptive/error-map image hashes, representative counts, represented
  source counts, coverage/luminance ratios, and error metrics. Timing and
  viewport stress values may differ.
- `lod_compare_metrics.json` should include `metrics_schema_version`,
  `density_policy`, `determinism`, source/cache fingerprints, and
  `difference_exr`; schema v3 should also include tile pressure and
  conservative culling fields.
- Applicable RGB/scalar/normal/emissive data should produce nonzero Adaptive HQ
  and exact Fast Basic CPU representative class counts plus nonzero
  feature-triggered refinement counts.
- Scalar, water-effect, normal, source-color, depth, and AOV lookups must remain
  source-correct through `drawItem.sourcePointIndex`.
- `.ipcloud` checks should report explicit cold/warm cache state and reason,
  representative preview count, time to first coarse frame, publish status, and
  interrupted/resume decision. A legacy v4 hierarchy cache alone should not be
  reported as a `.ipcloud` warm hit.
- Export logs should state the density policy, Full Source/preview/viewport
  snapshot/deterministic-selection semantics, selection signature,
  fallback/error frame counts, representative min/max/mean, and streamed
  upload/chunk summaries.
- Full Source export should fail if exact source data is not resident; Match
  Viewport animation should fail until approved per-frame viewport states exist;
  Quick MP4 should stay `Fast Adaptive Preview`.
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
- Treat Match Viewport export as an explicit approval workflow: the user must
  approve the current viewport look before still export, and animation exports
  must use Adaptive HQ or Fast Preview until per-frame viewport approvals exist.
- Prioritize measured lag sources before ideal architecture polish.
- Do not present sampled-index controls as adaptive LOD.
