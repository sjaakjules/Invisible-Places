# lod_goal_stage08.md — Raw Chunk Streaming, Residency, and Memory Model

## Codex goal

```text
/goal Implement visible raw-chunk streaming and CPU/GPU residency management so large point clouds no longer require full permanent parse/upload duplication before adaptive rendering is useful, verified by bounded memory, upload-budget diagnostics, warm-cache panning tests, and no regressions in Fast Basic or Beauty Adaptive. Use the `.ipcloud` bundle from Stage 07 and preserve progressive coarse rendering. Between iterations, stream or evict one resource class, measure memory/upload impact, and rerun the same viewport path. If a renderer still requires full source buffers, isolate that requirement and report the smallest safe migration path.
```

## Required prior stage

Stage 07 must be complete. The cache bundle and progressive coarse rendering should exist.

## Why this stage exists

The audit says loading still parses and uploads the full PLY before adaptive rendering can be useful, and source positions may be duplicated in several CPU/GPU forms. The target system uses raw chunks, memory-mapped data, CPU/GPU LRU caches, and device-local static buffers with staging uploads.

## Scope for this stage

In scope:

- Raw chunk format sufficient for streaming.
- CPU chunk residency map and optional decoded CPU LRU.
- GPU visible/recent chunk residency.
- Upload budget per frame.
- Device-local static chunk buffers with staging where practical.
- Visible-chunk scheduling and prioritization.
- Mapping from chunk-local data to global/source IDs.
- Diagnostics for memory, residency, and upload bandwidth.

Out of scope:

- GPU compute traversal.
- Tile overdraw/occlusion.
- Full editing/picking architecture migration unless needed to avoid full duplication.
- Lossy compression tuning beyond a safe initial chunk format.

## Implementation tasks

1. Inspect all permanent source data copies after a large PLY load:
   - `LoadedPointCloud`
   - `cpuPositions`
   - vertex position buffer
   - storage position buffer
   - colour buffer
   - normal buffer
   - scalar buffers
   - hierarchy data
   - any cache-side copies
2. Define raw chunk storage that is directly uploadable or cheaply decoded:
   - local bounds
   - local point count
   - quantized positions relative to local bounds
   - packed RGB/RGBA
   - packed normals if present
   - scalar field blocks
   - source point base/global ID mapping
   - optional compression footer if already supported
3. Implement chunk residency structures:
   - manifest/hierarchy always resident
   - chunk residency map
   - visible chunk request queue
   - CPU decoded chunk LRU if needed
   - GPU chunk LRU for visible/recent chunks
4. Implement visible-chunk scheduling:
   - prioritize current frustum and high-error visible nodes
   - prefetch nearby/recently visible chunks
   - avoid thrashing during camera movement
   - cancel or deprioritize stale requests
5. Implement upload budgets:
   - max upload bytes per frame from Stage 06 governor
   - staged uploads spread across frames
   - no `WaitIdle()` for normal streaming updates
6. Move large static chunk buffers toward device-local memory with staging uploads where the renderer architecture allows it.
7. Ensure adaptive draw items and shaders can resolve source attributes from chunked storage or a compatibility mapping.
8. Keep exact/debug Full Source semantics, but avoid requiring all source data to be resident for adaptive viewport rendering.
9. Add diagnostics:
   - visible chunks requested/resident/missing
   - CPU resident bytes
   - GPU resident bytes
   - upload bytes this frame/EWMA
   - upload queue length
   - chunk hit rate
   - eviction count/reason
   - peak resident memory
   - fallback reason if full buffers are still required

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Add tests for:

- chunk encode/decode roundtrip bounds and source IDs
- chunk residency map state transitions
- upload budget limiting
- LRU eviction preserves visible chunks
- stale request cancellation/deprioritization
- cache validation catches schema/chunk format mismatches

## Manual runtime checkpoint for the user

Use a large cloud that previously stressed memory:

1. Cold launch with completed `.ipcloud` cache.
2. Watch time to coarse frame and memory growth.
3. Pan/zoom through dense areas and watch streaming diagnostics.
4. Stop and let idle refinement load visible detail.
5. Move to a distant area and confirm old chunks are eventually evicted.
6. Compare Fast Basic and Beauty Adaptive.

Expected behaviour:

- Adaptive rendering is useful before all raw chunks are resident.
- Peak resident memory is lower than the pre-streaming baseline.
- Upload bytes per frame stay within budget during navigation.
- Missing chunks refine in without popping beyond Stage 03/05 accepted behaviour.
- Full Source exact/debug modes still work, even if they need more time/memory.

## Completion condition

This stage is complete only when:

- Raw chunks stream from `.ipcloud` according to visible demand.
- CPU/GPU residency is bounded and diagnosable.
- Normal adaptive rendering does not require full source parse/upload duplication.
- Upload bandwidth is budgeted and does not cause visible stalls.

## Stop conditions

Stop and report if:

- Existing shaders fundamentally require global dense storage buffers and cannot be safely chunked in this stage.
- Picking/editing/export requires full CPU residency and no compatibility layer exists.
- Device-local staging changes are unstable on MoltenVK and need a platform decision.

## Handoff to Stage 09

Pass along:

- Export implications of chunk residency.
- Determinism concerns caused by async chunk streaming.
- Final memory/upload metrics for the representative cloud.
