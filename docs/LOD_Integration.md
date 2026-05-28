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
adaptive draw-item paths when supported, compare-only GPU draw-item
full-range semantic-equivalent selection/count-compaction/source-fingerprint/checksum/class-count diagnostics for
diagnostic viewport draws, a submitted compacted-output buffer capped at 3,145,728
draw items, active renderer-profile, valid representative-class, and represented-source-count validity filtering, ordered draw-index-preserving output writes when the fitted CPU-selected range is complete and semantic-equivalent, order-sensitive output identity parity, compare-only protected feature-class, stable-rank-prefix, hierarchy-depth, projected-area, render-area, represented-count, coverage-compensation, and clamp-flags predicate probes, a dedicated stable-rank-prefix diagnostic output buffer capped at 1,310,720 draw items for unordered compacted-output identity parity, compacted-output descriptor/barrier submission after parity and performance gates pass, diagnostic compacted-count indirect
command output, full CPU-selected-range dispatch, and CPU-count compute-generated submitted indirect
command output only when compacted output is not eligible for the current image. It also tracks CPU/GPU full-range renderer-profile/class/represented-validity masks, protected feature-class, stable-rank-prefix, stable-rank output identity/capacity/fallback, hierarchy-depth, projected-area, render-area, represented-count, coverage-compensation, and clamp-flags predicate
timing per renderer profile and keeps explicit retry-window or over-capacity fallback reasons when the GPU path is slower, unsupported, or larger than a bounded diagnostic output. The frustum-checked shader path remains available for comparison,
but is disabled by default after slower MoltenVK timing. GPU compute selection
remains guarded behind feature, parity, and timing proof; on the local MoltenVK
runtime the current reported path is
`cpu-selection+gpu-full-range-selection-compare+gpu-compacted-indirect-submit`.
Dense full-source comparison is now preflighted before allocation; target-cloud
sources that exceed the local safety limit write a `skipped_dense_full_source_preflight`
diagnostic instead of silently running out of memory.
Memory-mapped chunks and fully device-local chunk residency remain later-stage
items.

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
  submit through `vkCmdDrawIndirect` when supported; current diagnostic
  viewport draw items can have representative class/rank/depth/flag
  metadata decoded, full-range filtered through a semantic-equivalent predicate,
  dispatched over the full CPU-selected range,
  compacted with source-identity fingerprints folded from existing source-index
  accumulators, checksummed, copied into an ordered diagnostic output buffer
  when the full fitted range is selected, and converted into a diagnostic
  indirect command by a compare-only compute path. When previous-frame ordered
  output identity plus current semantic, performance, and candidate-vs-reference
  gates pass, the graphics path binds a compacted draw-item descriptor set and
  submits the compacted indirect command; otherwise it falls back to the
  CPU-selected compact draw-item count or direct draw with an explicit reason. A
  second compare-only pass filters protected feature-class representatives on
  GPU with a class mask and reports CPU/GPU count, source-fingerprint, checksum,
  class-count, and timing parity without submitting the reduced output. A third
  compare-only pass filters a stable rank prefix on GPU with packed rank <= 255
  and reports the same count/fingerprint/checksum/class-count/timing parity
  without submitting the reduced output. A fourth compare-only pass filters a
  projected-footprint area window on GPU with `footprintAreaPixels >= 4` and
  reports the same count/fingerprint/checksum/class-count/timing parity without
  submitting the reduced output. Additional compare-only passes filter
  represented source count, coverage compensation, and packed clamp flags for
  emission/performance compensation clamps with the same parity/timing contract.
  A
  separate shader can project source positions through the frame
  `viewProjection` for frustum comparison, but that predicate is disabled on the
  default path after slower local measurements. Selection semantics remain
  unchanged. EXR command buffers and unsupported/small paths keep the
  CPU-generated or direct fallback.
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
  culling fields, GPU full-range compaction input/dispatched/selection-limit/source-fingerprint fields,
  profile-mask/opacity-window/emission-window/frustum enabled/guard/fallback/output-write/fallback/output-capacity/copied-draw-items/output-probe parity/count/checksum/fingerprint/CPU-vs-GPU representative class-count/position-count fields,
  CPU-reference timing, GPU timing, performance status, protected feature-class
  GPU probe mask/count/checksum/source-fingerprint/timing/performance status,
  stable-rank prefix GPU probe limit/count/checksum/source-fingerprint/timing/performance status,
  hierarchy-depth GPU probe depth window/count/checksum/source-fingerprint/timing/performance status,
  projected-area GPU probe footprint/render-area window/count/checksum/source-fingerprint/timing/performance status,
  render-area GPU probe footprint/render-area window/count/checksum/source-fingerprint/timing/performance status,
  represented-count GPU probe window/count/checksum/source-fingerprint/timing/performance status,
  coverage-compensation GPU probe opacity/emission window/count/checksum/source-fingerprint/timing/performance status,
  clamp-flags GPU probe required/rejected flags/count/checksum/source-fingerprint/timing/performance status,
  Beauty stress metrics, and colour, scalar, normal, and
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
  build, focused GPU/LOD tests, full CTest 189/189, stream check, JSON metrics
  validation, and repeat `--lod-compare` runs after a bounded compacted-output
  write was added for the stable-rank prefix probe. The local MoltenVK runtime
  reported GPU-selection feature support plus indirect-count support; the
  runtime path now reports
  `cpu-selection+gpu-full-range-selection-compare+gpu-compacted-indirect-submit`
  for eligible viewport draws. The metadata compute shader decodes draw-item
  renderer profile/class/rank/depth/flags plus represented-count,
  projected-footprint/render-area metadata, and opacity/emission compensation,
  full-range filters the CPU-selected draw-item range through a semantic-equivalent
  predicate with the active renderer-profile mask, valid representative-class
  mask 255, rank <= 2047, depth 0-255, represented count 1-4,294,967,295,
  full float area/opacity/emission windows, and flags +0/-0, aggregates selected
  items per workgroup before updating global compaction stats, accumulates
  CPU/GPU representative class counts, folds source-identity fingerprints from
  the existing source-index XOR/sum accumulators, checksums CPU-selected draw
  items, writes an ordered output copy when the full fitted range is selected,
  and converts the compacted GPU count into the submitted indirect command after
  the parity gates pass.
  The latest Fast Basic compare recorded 1 metadata full-range dispatch over
  2,876,771 CPU-selected draw items, selected the active Fast Basic
  renderer-profile mask 1, valid representative-class mask 255, and
  represented-source window 1-4,294,967,295, matched previous-frame CPU/GPU
  selected count 2,876,771 / 2,876,771, matched compacted indirect CPU/GPU
  vertices 2,876,771 / 2,876,771, copied all 2,876,771 draw items into the
  submitted compacted output buffer, and passed ordered output identity
  2,876,771 / 2,876,771. Fast Basic measured 235.89 ms for the CPU reference
  predicate and 14.2892 ms for GPU full-range compaction, so the GPU pass is
  measurably faster for the semantic-equivalent predicate. The protected
  feature-class GPU probe filtered colour contrast, normal edge, scalar
  min/max/threshold, and emissive accent representatives with mask 126; it
  matched CPU/GPU count 1,290 / 1,290, source fingerprint 1,646,296,788,
  checksum 3,534,349,174, and measured 8.74642 ms CPU reference vs 0.023875 ms
  GPU. A stable-rank prefix GPU probe filtered packed rank <= 255, matched
  CPU/GPU count 1,195,635 / 1,195,635, source fingerprint 362,019,890,
  checksum 3,671,843,879, and measured 108.262 ms CPU reference vs 0.023625 ms
  GPU. The same stable-rank predicate now writes a diagnostic-only unordered
  compacted output into a dedicated rank-probe buffer when the expected prefix
  fits; on the sample it enabled output writes with capacity 1,310,720, copied
  1,195,635 draw items, passed output identity 1,195,635 / 1,195,635, matched
  output checksum 833,599,839 and output source fingerprint 362,427,488, and
  left the output fallback reason empty. If the selected prefix exceeds the
  bounded capacity, the write is skipped with an explicit reason and the
  submitted full-range compacted buffer is not touched. A hierarchy-depth GPU
  probe filtered packed depth 7-255, matched CPU/GPU count 204,756 / 204,756,
  source fingerprint 1,220,167,939, checksum 4,137,926,082, and measured
  29.3021 ms CPU reference vs 0.021792 ms GPU. A projected-area GPU probe
  filtered `footprintAreaPixels >= 4`, matched CPU/GPU count 102,497 / 102,497,
  source fingerprint 263,410,465, checksum 65,403,358, and measured 33.5083 ms
  CPU reference vs 0.030417 ms GPU. A render-area GPU probe filtered
  `renderAreaPixels >= 4`, matched CPU/GPU count 0 / 0 with zero source
  fingerprint and checksum, measured 32.5013 ms CPU reference vs 0.022583 ms
  GPU, and reports `matched zero render-area candidates; render-area probe remains diagnostic-only`,
  so it remains a diagnostic-only fallback rather than a submission candidate on
  the current sample viewport. A represented-count GPU probe filtered
  `representedSourceCount >= 2`, matched CPU/GPU count 1,652,511 / 1,652,511,
  source fingerprint 1,058,297,337, checksum 1,624,835,333, and measured
  101.324 ms CPU reference vs 0.364208 ms GPU. A coverage-compensation GPU probe
  filtered `opacityCompensation >= 1.25` and `emissionCompensation >= 1.25`,
  matched CPU/GPU count 1,576,869 / 1,576,869, source fingerprint
  2,629,473,851, checksum 1,167,223,779, and measured 89.6419 ms CPU reference
  vs 0.775416 ms GPU. A clamp-flags GPU probe filtered packed metadata flags
  +0x6/-0 for emission and performance compensation clamps, matched CPU/GPU
  count 1,095,739 / 1,095,739, source fingerprint 3,718,856,107, checksum
  3,967,260,594, and measured 65.4086 ms CPU reference vs 0.403792 ms GPU. The
  compacted-submission gate now reports `gpu_compaction_submission_used=true`,
  empty output-write and submission fallback reasons, candidate/reference
  vertices 2,876,771 / 2,876,771, and submitted 2,876,771 compacted indirect
  vertices under the 4,823,449 representative budget. The older CPU-count
  indirect command remains as a first-frame fallback only, with 1 dispatch at
  0.010125 ms in the latest sample run. Beauty stress kept max GPU point pass
  0.094125 ms with EWMA 0.0302545 ms and reported no adaptive/full-source
  fallback or budget exceedance. The frustum-checked shader path still exists,
  but `*_compaction_selection_frustum_enabled=false`, guard band 0, and the
  fallback reason reports that the GPU geometry-frustum predicate is disabled
  because the previous MoltenVK/sample measurement was slower than metadata-only
  full-range compaction. Coverage ratio stayed 1, luminance ratio 0.8166, RGB
  MAE 0.015853, selection hash `0xc56d6360df4e9d71`, Adaptive HQ used
  1,508,358 representatives covering 11,983,509 source points, and the repeat
  compare reported no adaptive/full-source fallback or budget exceedance. CPU
  traversal remains authoritative; submitted compacted output writes stay gated
  by full-range output identity, performance, and candidate-vs-reference parity;
  the stable-rank output probe remains diagnostic-only; protected feature-class,
  stable-rank, hierarchy-depth, projected-area, render-area, represented-count,
  coverage-compensation, and clamp-flags filtering carry the active
  renderer-profile and representative-class masks where semantically equivalent
  but stay compare-only; and CPU-selected direct or CPU-count indirect draw
  submission remains active whenever compacted output is unsupported, slower,
  incomplete, non-equivalent, zero-candidate, or over diagnostic capacity. The
  final hands-on Fast Basic/Beauty/export matrix remains outstanding. The stream
  check still passed at center/repeat-center 563 / 2,292 chunks, 43,438 / 56,537
  remapped draw items, 120.9 MiB CPU residency, 128.0 MiB GPU residency/upload,
  and 76.8311% chunk hit rate. The 100M full-source compare preflight still exits
  13 with `lod_compare_status=skipped_dense_full_source_preflight` because the
  estimated dense resident footprint remains 14,909,995,080 bytes against the
  6,442,450,944-byte safety limit.
- Stage 11 target-cloud evidence on `Data/Site3-Mid-1mm100M.ply` passed
  `--lod-stream-check` after rebuilding the stale v2 `.ipcloud` bundle for
  100,743,210 source points and 1,903 raw chunks. The rebuild loaded full source
  in 47,822 ms, built the hierarchy in 766,039 ms, published in 54,992.3 ms, and
  then warm-loaded the 65,536-point preview in 301.479 ms. Center/pan/repeat
  stream passes stayed bounded at 120.9 MiB CPU residency and 128.0 MiB GPU
  residency/upload: center 101/1,903 chunks with 13,942/27,334 remapped draw
  items, pan-left 132/1,903 chunks with 20,380/34,614 remapped draw items,
  pan-right 121/1,903 chunks with 14,852/27,845 remapped draw items, and
  repeat-center matching center. The rebuild run peaked at 11.1 GiB RSS because
  dense exact-source compatibility is still used for bundle rebuild and exact
  debug/export flows. A full `--lod-compare Data/Site3-Mid-1mm100M.ply` now
  exits before dense allocation with code 13 and
  `lod_compare_status=skipped_dense_full_source_preflight`: estimated dense
  CPU+GPU resident bytes are 14,909,995,080 against the 6,442,450,944-byte
  safety limit, and the valid bundle path is reported as
  `Saved/PointCloudCache/Site3-Mid-1mm100M.1544b2b5635db91e.ipcloud`. This is an
  intentional fallback diagnostic, not a GPU parity failure; full-source 100M
  visual parity remains outstanding until compare/export can iterate exact source
  data chunk-wise.

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
- Traversal selection is still CPU-authored by default. A compare-only GPU
  full-range selection/draw-item compaction/checksum pass is active for large
  eligible viewport paths and its ordered compacted output can be submitted
  after parity/performance gates pass; the GPU predicate now checks the active
  renderer-profile metadata mask, valid representative-class metadata mask, and
  represented-source-count validity before the semantic-equivalent submission gate can pass. Protected feature-class, stable-rank
  prefix, hierarchy-depth, projected-area, render-area, represented-count,
  coverage-compensation, and clamp-flags filtering probes also run on GPU for
  comparison, but they do not author or submit a reduced
  selection. GPU compute culling and
  authoritative traversal selection are not active. The slower geometry-frustum
  predicate is disabled with an explicit
  fallback reason. Indirect draw
  submission and compute-generated indirect commands are active for large
  viewport adaptive draw-item paths where Vulkan supports them.
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
- GPU compute traversal/culling/authoritative selection and GPU-side
  indirect-count submission.

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
  substitute for a future depth proxy, Hi-Z pyramid, or authoritative
  GPU-driven selection.
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
- Keep the current indirect, compacted-output submission, renderer-profile,
  representative-class, and represented-source-count validity filtering, protected
  feature-class probe, stable-rank prefix probe, hierarchy-depth probe,
  projected-area probe, render-area probe, represented-count,
  coverage-compensation, and clamp-flags probe paths
  guarded by runtime diagnostics. Move culling,
  projected-error evaluation, authoritative compaction/selection, and
  indirect-count work to compute only where they beat the CPU path, and keep
  skipping redundant CPU-count command generation when a compacted command is
  already eligible.
- Keep a CPU fallback for portability and debugging.

Metrics to watch:

- tiles over budget
- culled hidden nodes
- compute selection ms
- GPU full-range selection/compaction parity/status/input/dispatched/selection/profile-mask/class-mask/rank-limit/depth-window/projected-footprint-window/opacity-window/emission-window/represented-source-count-window/frustum-enabled/frustum-guard/frustum-fallback-reason/output-write-enabled/output-write-fallback/output-capacity/copied-draw-items/output-probe-parity/submission-eligible/submission-used/submission-fallback/candidate-vs-reference vertices/CPU-vs-GPU class counts/position-count/required-flags/rejected-flags/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- protected feature-class GPU probe use/parity/mask/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- stable-rank prefix GPU probe use/parity/rank-limit/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status/output-write-enabled/output-fallback/output-capacity/copied-draw-items/output-parity/output-count/output-source-fingerprint/output-checksum
- hierarchy-depth GPU probe use/parity/depth-window/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- projected-area GPU probe use/parity/footprint-window/render-window/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- render-area GPU probe use/parity/footprint-window/render-window/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- represented-count GPU probe use/parity/represented-source-window/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- coverage-compensation GPU probe use/parity/opacity-window/emission-window/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- clamp-flags GPU probe use/parity/required-flags/rejected-flags/count/source-fingerprint/checksum/CPU-reference-ms/GPU-ms/performance-status
- diagnostic compacted-indirect command parity/dispatch/CPU-vs-GPU vertices
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
  conservative culling fields, GPU compaction submission use/fallback, protected
  feature-class, stable-rank prefix, hierarchy-depth, projected-area, render-area,
  represented-count, coverage-compensation, and clamp-flags GPU probe parity/timing
  fields, and performance fallback/retry
  fields when compare-only full-range compaction is slower than the CPU
  reference. If a target-cloud dense comparison is unsafe,
  the same path should instead report `lod_compare_status=skipped_dense_full_source_preflight`
  and `full_source_compare_skipped_reason`.
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
