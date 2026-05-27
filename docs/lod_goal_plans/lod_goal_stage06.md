# lod_goal_stage06.md — Measured GPU Timing Governor

## Codex goal

```text
/goal Replace FPS-guess-based adaptive quality control with a measured GPU timing governor for point-cloud passes, verified by Vulkan timestamp diagnostics, tests where practical, and viewport stress runs showing density/fragment budgets adapt to actual Fast Basic and Beauty point-pass cost without oscillation or hidden quality drops. Preserve all previous visual and boundedness behaviour. Between iterations, add one timing surface or governor rule, verify it with diagnostics, and rerun the relevant stress view. If timestamps are unavailable on the target runtime, implement a safe fallback and report the feature check result.
```

## Required prior stage

Stage 05 must be complete. Fast Basic and Beauty Adaptive should already be visually usable and bounded.

## Why this stage exists

The audit says current quality demotion/promotion uses scene update FPS and frame/present FPS, not measured GPU point-pass timings. That can react to UI/present behaviour instead of the actual expensive point/surfel passes. The target system uses measured GPU time and an EWMA governor.

## Scope for this stage

In scope:

- Vulkan timestamp queries or existing GPU timing infrastructure.
- Timings for point-cloud render passes.
- EWMA timing state per renderer/geometry/quality mode.
- Budget scaling for representative count, estimated fragments, blended fragments, and upload bytes.
- Explicit degraded/performance-limited diagnostics.
- Safe fallback if timestamps are unsupported.

Out of scope:

- `.ipcloud` streaming except upload-budget hooks needed for governor state.
- Tile overdraw budgets beyond global/tile placeholders.
- GPU compute traversal.
- Major visual selection redesign.

## Implementation tasks

1. Inspect existing Vulkan timing/query code, frame graph/render pass structure, and diagnostics overlay.
2. Add timestamp query support for relevant passes where practical:
   - Fast Basic point pass
   - Beauty point/surfel pass
   - depth prepass
   - transparency accumulation
   - EDL/composite
   - upload/streaming stalls or submission if measurable separately
3. Avoid high-frequency CPU/GPU synchronization.
   - Use previous-frame query results.
   - Skip/mark invalid timings when unavailable rather than stalling.
4. Add governor state similar to:

```cpp
struct PointCloudPerformanceGovernor {
    float targetFrameMs;
    float targetPointPassMs;
    float gpuPointPassEwmaMs;
    float gpuCompositeEwmaMs;
    uint64_t maxVertices;
    uint64_t maxEstimatedFragments;
    uint64_t maxEstimatedBlendedFragments;
    uint64_t maxUploadBytesPerFrame;
    float targetSpacingPx;
    float emergencySpacingPx;
    bool degradedInteractiveMode;
};
```

5. Maintain separate EWMA/tuning state for at least:
   - Fast Basic
   - Beauty screen sprites
   - Beauty world surfels if present
   - moving vs idle vs final/export if those paths share state
6. Replace scene/present-FPS-driven quality demotion with point-pass timing where available.
   - Present/UI FPS can remain diagnostic context.
   - Do not let cached presentation hide a point-pass bottleneck.
7. Adapt slowly and predictably.
   - Clamp budget scale changes per update, e.g. a narrow range such as 0.80–1.15 unless codebase has better defaults.
   - Avoid oscillation around the target.
   - Preserve Stage 03 hysteresis and smooth transitions.
8. Add explicit status when quality floors cannot be maintained:
   - `Adaptive preview limited by GPU: fragment budget reached.`
   - `Performance-limited` vs `visually lossless` vs `full source`.
9. Expose diagnostics:
   - GPU point-pass ms and EWMA
   - GPU composite/EDL ms and EWMA
   - CPU scene update ms
   - UI/present FPS
   - selected quality/density mode
   - budget scale factor
   - active governor limit: vertices, fragments, blended fragments, upload, tile placeholder
   - timestamp support/fallback state

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Add tests where practical:

- EWMA update math.
- Budget scaling clamps.
- Governor uses GPU point-pass timing when valid.
- Governor falls back safely when timestamps are invalid/unavailable.
- Degraded status is set when target point-pass budget cannot be met at the minimum interactive floor.

## Manual runtime checkpoint for the user

Run Fast Basic and Beauty Adaptive with a representative cloud. Stress the system by changing:

- point size / sprite radius
- opacity/transparency
- emissive style if present
- screen-sprite vs surfel mode
- dense grazing views
- window size / resolution if easy

Expected behaviour:

- GPU point-pass ms rises when point size/Beauty cost rises.
- Budgets adapt to point-pass cost rather than only scene/present FPS.
- Density changes are gradual and do not reintroduce popping.
- If the target cannot be maintained, diagnostics explicitly say performance-limited.
- Idle views refine back toward higher quality when budget allows.

## Completion condition

This stage is complete only when:

- Measured GPU pass timing drives adaptive budgets where supported.
- Safe fallback exists where timestamps are unavailable.
- Diagnostics distinguish point-pass, composite/EDL, CPU update, and present/UI behaviour.
- Fast Basic and Beauty remain smooth, bounded, and visually acceptable.

## Stop conditions

Stop and report if:

- Vulkan/MoltenVK timestamp support is unavailable or unreliable on the target and no proxy timing can be safely used.
- Query insertion requires a render graph rewrite beyond this stage.
- Timing readback introduces visible stalls.

## Handoff to Stage 07

Pass along:

- Upload budget needs and current upload timing gaps.
- Time-to-first-frame diagnostics needed for progressive cache work.
- Known expensive passes for streaming prioritization.
