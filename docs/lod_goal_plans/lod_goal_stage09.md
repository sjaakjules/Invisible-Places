# lod_goal_stage09.md — Export Modes, Determinism, and LOD Compare Diagnostics

## Codex goal

```text
/goal Finish adaptive LOD integration for EXR/MP4/export workflows with explicit density modes, deterministic selection, and expanded quality/performance diagnostics, verified by repeatable exports, lod-compare metrics, and manual viewport-match checks. Preserve viewport responsiveness, Full Source exactness, and chunk-streamed adaptive rendering. Between iterations, make one export mode or metric deterministic and test it twice with the same inputs. If export expectations are ambiguous, keep modes explicit and report what the user must choose.
See docs/lod_goal_plans/lod_goal_stage09.md for details of implementation and completion verification.
docs/point_cloud_adaptive_lod_fast_beauty.md describes the full adaptive LOD system with docs/lod_goal_plans/lod_goal_index.md describing the stages of development.
Please update docs/CURRENT_STATE.md, docs/LOD_Integration.md, and push a git update on completion of edits.
You can use Data/Site3-Sample-Terrestrial.ply to do tests initially as it is a section of the main cloud (Data/Site3-Mid-1mm100M.ply) with same density, only 12M as it covers an area 5m x 5m not 10m x 30m
```

## Required prior stage

Stage 08 should be complete. The adaptive viewport should work with streaming/residency.

## Why this stage exists

The project already has explicit renderer and export density modes, and `--lod-compare` writes full-source/adaptive EXRs plus a JSON metrics file. The audit says metrics are currently minimal and do not include image error maps, timing breakdowns, peak memory, upload bandwidth, or per-render-pass GPU time. The target design requires Adaptive HQ, Match Viewport, Fast Preview, and Full Source semantics to be explicit and deterministic.

## Scope for this stage

In scope:

- EXR export density modes.
- MP4 export density modes if MP4 exists in the project.
- Deterministic LOD selection for exports.
- Match Viewport behaviour.
- Adaptive High Quality export behaviour.
- Expanded `--lod-compare` metrics and artifacts.
- Export logs/diagnostics.

Out of scope:

- Tile overdraw/occlusion unless already exposed through metrics.
- GPU compute traversal.
- New artistic style design beyond preserving existing modes.

## Implementation tasks

1. Inspect current export density modes:
   - `Full Source`
   - `Adaptive High Quality`
   - `Match Viewport Adaptive`
   - `Fast Adaptive Preview`
   - `Artistic As Preview`
   - `Artistic High Quality`
2. Ensure each mode has explicit semantics in code and UI/logs.
   - `Full Source` draws raw source/chunks exactly and uses literal user point size/opacity/emission.
   - `Adaptive High Quality` uses strict visual thresholds and no interactive frame-time relaxation unless explicitly allowed.
   - `Match Viewport` uses the exact selected/approved viewport LOD state/look.
   - `Fast Preview` or artistic preview modes are labelled as preview quality.
3. Make export selection deterministic.
   - Same cache + same camera frame + same style + same seed = same selected representatives.
   - Streaming completion order must not change the final exported selection.
   - MP4 frames must not flicker from unstable ranks, random seeds, or async request ordering.
4. Make export wait/refine rules explicit.
   - Adaptive HQ may wait for needed visible chunks/representatives.
   - Match Viewport should not silently refine beyond the approved viewport unless the mode says so.
   - Full Source may be slow but must not substitute representatives.
5. Expand `--lod-compare` artifacts where practical:
   - full-source EXR
   - adaptive EXR
   - difference/error map image
   - coverage ratio
   - luminance ratio
   - per-channel or mean error metrics
   - representative count
   - represented source count
   - timings by pass
   - peak memory
   - upload bytes/bandwidth
   - cache hit/load time
   - governor/timing mode
   - selected export density mode
6. Ensure source-correct AOVs and metadata:
   - colour/scalar/water/normal/depth/AOVs use `sourcePointIndex` or chunk source IDs correctly.
   - Export logs include cache fingerprint and build/settings version.
7. Add tests or golden-style deterministic checks where practical.

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Run the same export/compare twice with identical inputs and compare:

- selected representative IDs/counts if logged
- output image hashes where deterministic rendering makes that reasonable
- `lod_compare_metrics.json`
- MP4 frame selection logs if MP4 is available

Add tests for:

- density mode mapping
- deterministic selection seeds
- Match Viewport uses captured viewport selection
- Adaptive HQ does not use interactive degraded budgets
- Full Source does not use representatives

## Manual runtime checkpoint for the user

Test at least:

1. Viewport Beauty Adaptive, approve a look, then export Match Viewport.
2. Export Adaptive High Quality from the same camera/style.
3. Export Full Source as an exact/debug reference if the cloud size allows it.
4. Repeat a short MP4/camera path twice if MP4 exists.

Expected behaviour:

- Match Viewport matches the approved viewport look.
- Adaptive HQ may be better/refined than viewport but does not silently degrade for interactivity.
- Full Source is clearly exact/debug and may be slow.
- Repeated MP4/export selections do not flicker or change randomly.
- Metrics are detailed enough to diagnose quality/performance differences.

## Completion condition

This stage is complete only when:

- Export modes are explicit and deterministic.
- `--lod-compare` provides useful quality and performance evidence.
- Match Viewport, Adaptive HQ, and Full Source semantics are distinct and testable.
- Exports remain source-correct for colours/scalars/normals/water/depth/AOVs.

## Stop conditions

Stop and report if:

- Existing export code bypasses the viewport renderer entirely and needs a larger render-path unification.
- Full Source export cannot run on the target cloud due to memory/time limits; preserve the mode and mark the limitation clearly.
- The project lacks MP4 export despite the target design mentioning it.

## Handoff to Stage 10

Pass along:

- Worst-case Beauty views from export/compare metrics.
- Tile/overdraw pressure evidence if already estimated.
- Pass timings and memory/upload data for advanced performance work.
