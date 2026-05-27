# Adaptive Point-Cloud LOD Goal Plan Index

Date: 2026-05-27

This folder contains staged Codex goal documents for implementing the adaptive point-cloud LOD system described in:

- `LOD_Integration.md` — current project integration audit.
- `docs/point_cloud_adaptive_lod_fast_beauty.md` — target design.

Use one stage at a time. Do not start the next stage until the current stage's automated checks pass and the user has run the program at the manual checkpoint.

## Why the early stages are split

The user's first desired outcome, “adaptive LOD runs smoothly and automatically without visible popping or sudden detail loading,” is split into three runnable checkpoints:

1. Stage 01 restores the build and establishes trusted diagnostics.
2. Stage 02 makes Fast Basic adaptive rendering bounded and responsive, with no accidental full-source fallback, no per-camera fallback churn during motion, motion-time coverage patches for newly visible regions, no idle-stop demotion flash, and no oversized visible LOD footprint artifacts.
3. Stage 03 makes Fast Basic transitions visually smooth, with stable ranks, hysteresis, and parent/child transition handling.

This keeps performance safety separate from visual smoothness. It also uses Fast Basic first because it avoids Beauty-specific opacity, emission, falloff, EDL, and surfel complexity.

## Stage order

| Stage | File | Runnable checkpoint |
|---:|---|---|
| 01 | `lod_goal_stage01.md` | Project builds, LOD tests pass, and baseline LOD diagnostics are repeatable. |
| 02 | `lod_goal_stage02.md` | Fast Basic adaptive mode is bounded, responsive during camera motion, preserves the displayed high-detail base, applies throttled motion coverage updates, blocks lower-detail idle flashes, and never silently submits full source when adaptive data is unavailable. |
| 03 | `lod_goal_stage03.md` | Fast Basic adaptive transitions are smooth enough that the user does not notice popping, sudden detail loading, or density jumps during normal navigation. |
| 04 | `lod_goal_stage04.md` | CPU selector has useful node statistics and representative classes, preserving rare colour/scalar/normal/emissive detail. |
| 05 | `lod_goal_stage05.md` | Beauty Adaptive has renderer-specific footprint, opacity, emission, and surfel/sprite cost handling. |
| 06 | `lod_goal_stage06.md` | Adaptive quality is governed by measured GPU pass timings rather than scene/present FPS guesses. |
| 07 | `lod_goal_stage07.md` | `.ipcloud` cache bundle exists and supports progressive coarse first-load/subsequent-load rendering. |
| 08 | `lod_goal_stage08.md` | Raw chunk streaming, visible-chunk residency, upload budgets, and memory reductions are working. |
| 09 | `lod_goal_stage09.md` | EXR/MP4/export density modes are deterministic, explicit, and produce useful quality/performance diagnostics. |
| 10 | `lod_goal_stage10.md` | Tile overdraw budgeting and conservative occlusion reduce worst-case Beauty overdraw without causing holes. |
| 11 | `lod_goal_stage11.md` | GPU-driven culling/compaction/indirect draw is implemented where supported, with CPU fallback and final hardening. |

## Standard commands used by stages

Use the actual repo commands if they differ, but start with these commands from the integration audit:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-compare <cloud>
```

For manual viewport checks, use a representative large cloud and test at least:

```text
Fast Basic
Beauty Adaptive
Beauty Full Source, only as an exact/debug comparison
```

## Common rules for all stages

- Keep `docs/point_cloud_adaptive_lod_fast_beauty.md` as the target design. Do not weaken that design to match the current implementation.
- Treat `LOD_Integration.md` as the current audit and starting point, not as a replacement for source inspection.
- Keep manual sampled point budgets as explicit debug/loading caps only.
- Do not present sampled-index controls as adaptive LOD.
- Do not use `Beauty Full Source` or `Full Source` as silent adaptive fallbacks.
- During active camera movement, prefer stable adaptive reuse and throttled
  draw-item updates over rebuilding fallback LOD for every camera key.
- During active camera movement, let bounded traversals complete often enough to
  fill newly visible close regions as coverage patches.
- When camera movement stops, refine from the displayed adaptive base; do not
  replace an equal-or-better exact/high-quality set with a lower-detail coarse
  fallback.
- Keep coverage/budget footprint separate from visible render size so adaptive
  representatives do not cover gaps that exist in the full source cloud.
- Each stage must leave the app runnable.
- Each stage must update or add tests/diagnostics so the next stage has evidence to build on.
- If GUI/manual validation cannot be performed by the agent, the agent must stop with exact commands, expected overlay values, and a concise manual acceptance checklist.
