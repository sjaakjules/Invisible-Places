# lod_goal_stage11.md — GPU-Driven Selection, Indirect Draw, and Final Hardening

## Codex goal

```text
/goal Move adaptive point-cloud culling/selection/compaction toward GPU-driven execution where it is measurably better, with indirect draw submission when supported and a correct CPU fallback, verified by CPU/GPU selection parity tests, MoltenVK feature checks, performance diagnostics, full tests, and final Fast Basic/Beauty/export manual runs. Preserve all prior quality, streaming, cache, and export semantics. Between iterations, port one bounded part of selection to GPU, compare output/performance against CPU fallback, and keep the CPU path active until parity is proven. If GPU-driven work is slower or unsupported, leave a clear fallback and diagnostic reason.
See docs/lod_goal_plans/lod_goal_stage11.md for details of implementation and completion verification.
docs/point_cloud_adaptive_lod_fast_beauty.md describes the full adaptive LOD system with docs/lod_goal_plans/lod_goal_index.md describing the stages of development.
Please update docs/CURRENT_STATE.md, docs/LOD_Integration.md, and push a git update on completion of edits.
You can use Data/Site3-Sample-Terrestrial.ply to do tests initially as it is a section of the main cloud (Data/Site3-Mid-1mm100M.ply) with same density, only 12M as it covers an area 5m x 5m not 10m x 30m
```

## Required prior stage

Stage 10 should be complete. CPU traversal, budgets, streaming, exports, tile pressure, and conservative occlusion should already work and be measurable.

## Why this stage exists

The target design ultimately moves culling, projected-error evaluation, compaction, and indirect draw command generation to GPU compute where it beats the CPU path. The audit notes this is not implemented yet. This is deliberately late because the CPU path must be correct, measured, and trusted first.

## Scope for this stage

In scope:

- Runtime Vulkan/MoltenVK feature checks.
- GPU compute culling/evaluation for frustum, projected error, budget pressure, and/or occlusion where practical.
- GPU compaction of visible draw items.
- Indirect draw command generation where supported.
- CPU fallback and debug comparison mode.
- Final diagnostics and documentation updates.

Out of scope:

- Removing the CPU fallback.
- Relying on unsupported/unstable MoltenVK features without checks.
- Changing user-facing quality semantics.
- Full transparent sorting.

## Implementation tasks

1. Inspect current CPU traversal hot spots and Stage 06/10 timing data.
2. Add runtime feature checks for:
   - compute shader support/features already required by the app
   - storage buffer limits
   - indirect draw support
   - indirect count support such as `vkCmdDrawIndirectCount` if targeted
   - MoltenVK-specific limitations or performance concerns
3. Design GPU buffers for:
   - hierarchy/node pages
   - representatives
   - per-frame camera/style/governor constants
   - visibility flags
   - compacted draw items
   - indirect draw commands/counts if supported
4. Port one selection layer at a time:
   - frustum culling
   - projected spacing/error evaluation
   - representative filtering by rank/class
   - tile/fragment pressure where feasible
   - conservative occlusion/Hi-Z tests if Stage 10 produced GPU-ready data
5. Add GPU compaction for selected draw items.
6. Generate indirect draw commands where supported.
   - If indirect count is unsupported/unreliable, use a CPU-readable count only when it does not stall, or fall back to CPU-generated direct/indirect draws.
7. Keep CPU fallback active for:
   - unsupported devices
   - debug builds/comparison
   - small clouds where CPU is faster
   - failure/recovery paths
8. Add parity/comparison tooling:
   - CPU selected count vs GPU selected count
   - source/representative ID overlap or error tolerance
   - image compare through `--lod-compare`
   - budget compliance comparison
9. Add diagnostics:
   - selection path: CPU/GPU/fallback
   - compute selection ms
   - compaction ms
   - indirect draw count
   - GPU-selected representative count
   - fallback reason
   - CPU/GPU parity status in debug mode
10. Final hardening:
   - Run all tests.
   - Run Fast Basic, Beauty Adaptive, Beauty Full Source/debug, exports, cold/warm cache, and worst-case overdraw views.
   - Update project docs to describe final modes, diagnostics, limitations, and known future improvements.

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Add tests where practical:

- feature checks choose the correct path
- GPU path respects budgets
- CPU/GPU selection parity on deterministic synthetic hierarchy
- fallback activates when indirect count or required feature is unavailable
- no readback stalls in normal GPU-driven mode
- draw-item buffer bounds are respected

## Manual runtime checkpoint for the user

Run the final system matrix:

1. Cold-cache first load.
2. Warm-cache load.
3. Fast Basic moving/idle.
4. Beauty Adaptive screen sprites moving/idle.
5. Beauty world surfels if available.
6. Beauty Full Source exact/debug comparison on a manageable cloud.
7. Dense long/grazing overdraw stress view.
8. Match Viewport export.
9. Adaptive High Quality export.
10. Repeated MP4/camera path if MP4 exists.

Expected behaviour:

- GPU-driven selection is used only when supported and beneficial.
- CPU fallback remains correct and visible in diagnostics.
- No quality regression from Stages 02–10.
- No hidden full-source fallback in adaptive modes.
- No visible popping/sudden detail loading beyond accepted limits.
- Exports remain deterministic and mode-correct.
- Final diagnostics can explain performance, quality, cache, streaming, and fallback state.

## Completion condition

This stage is complete only when:

- GPU-driven selection/compaction/indirect draw is implemented where supported or explicitly disabled with evidence.
- CPU fallback remains maintained and tested.
- Final full test suite passes or unrelated failures are documented.
- Manual final matrix is ready for the user and all prior stage acceptance criteria remain true.
- Project docs are updated with final behaviour and known limitations.

## Stop conditions

Stop and report if:

- GPU-driven selection is slower than CPU for target clouds and no safe optimization remains in scope.
- Required Vulkan/MoltenVK features are unavailable.
- CPU/GPU parity cannot be achieved without changing quality semantics.
- Debugging GPU issues would require a separate graphics capture/tooling stage.

## Final handoff

Report:

- Which path is default on the target machine: CPU or GPU-driven.
- Which features are enabled/disabled by runtime feature checks.
- Final baseline metrics compared to Stage 01.
- Remaining known limitations and suggested future experiments.
