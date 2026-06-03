# Ripples Tuning Goal

## Goal

Complete GPU-first Ripple tuning so the broken Ripple overlay patterns match `docs/ripples_tuning.md`, live settings edits update sparse GPU params only after cached membership exists, `Tide Bands` becomes user-facing `Shoreline` with legacy loading preserved, and the work is validated by tests, contact-sheet artifacts, and latency evidence.

Target command:

```text
/goal Complete docs/RipplesTuning_GOAL.md: tune the broken Ripple patterns using GPU-first sparse params, preserve existing working patterns, render contact-sheet evidence from tests/Test_Points.txt on Site3-Sample-Terrestrial.ply, and stop only when the required tests and latency checks pass or a concrete blocker is documented.
```

This goal follows the OpenAI Codex goal pattern: one durable objective, concrete verification evidence, explicit constraints, progress checkpoints, and a blocker rule.

Reference guidance:

- https://developers.openai.com/codex/use-cases/follow-goals
- https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex

## Constraints

- Live viewport setting edits must update compact GPU params only after membership exists.
- Do not add per-point CPU pattern recomputation for ordinary settings edits.
- Keep CPU pattern functions only as offline/contact-sheet/test mirrors of the GLSL behavior.
- Keep ordinary Ripple recalculation free of dense active-cloud `water_effect_*` and `ripple_*` scalar uploads.
- Preserve Linear Ripples and Radial Ripples behavior except for safe shared-helper changes.
- Save new Shoreline overlays as `shoreline`; continue loading old `tide_bands`.
- Do not mark complete without concrete evidence from tests, smoke artifacts, and latency metrics.

## Checkpoints

1. Documentation
   - Create `docs/ripples_tuning.md`, this file, and `docs/logs/ripples_tuning_goal_progress.md`.
   - Record visual targets, performance thresholds, and validation artifacts.

2. GPU Params Performance
   - Add p50, p95, max, and total params-only latency metrics to the Ripple smoke report.
   - Confirm settings edits keep sparse membership revision unchanged and advance only params revision.
   - Pass if max is under 1 second, firm-pass if p95 is under 100 ms, desired-pass if p95 is under 10 ms, ideal-pass if p95 is under 1 ms.

3. Shoreline Compatibility
   - Rename UI/contact-sheet label from `Tide Bands` to `Shoreline`.
   - Serialize new overlays as `shoreline`.
   - Parse legacy `tide_bands`.
   - Update docs/tests that refer to Tide Bands.

4. Pattern Tuning
   - Update GLSL pattern math first in `shaders/pointcloud_sparse_ripple.glsl`.
   - Mirror logic in `src/water/WaterFlow.cpp` for offline renders and tests.
   - Tune Caustic Lace, Rain Rings, Shoreline, Wet Sheen, Current Threads, Droplet Glints, Drip Trails, Foam Sparkle, and Mineral Shimmer to the end states in `docs/ripples_tuning.md`.

5. Focused Tests
   - Add tests for temporal motion, parameter sensitivity, edge policy, shared droplet origins, geometry/downhill response, and non-static/non-linear structure.
   - Keep existing distinct-pattern and sparse-membership tests passing.

6. Visual Evidence
   - Run the `water-ripple-patterns-test-points` GUI smoke scenario.
   - Save or reference the JSON, PPM, and EXR artifacts.
   - Compare contact-sheet rows against `tests/Ripple Effect Reference Images`.

## Required Validation

```text
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug -R "Water|Ripple|water-ripple-patterns-test-points" --output-on-failure
ctest --test-dir build/macos-debug --output-on-failure
git diff --check
```

GUI smoke render:

```text
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places Data --gui-smoke water-ripple-patterns-test-points --smoke-output build/macos-debug/water-region-smoke
```

## Agent Coordination

Parent agent owns integration, docs, performance instrumentation, validation, and final acceptance.

Worker groups:

- Performance harness and docs.
- Caustic Lace, Shoreline, Foam Sparkle.
- Rain Rings and Drip Trails shared origins.
- Wet Sheen and Droplet Glints.
- Current Threads and Mineral Shimmer.

Each worker must keep changes narrow, preserve unrelated patterns, and report files changed, tests run, and blockers. Parent integration rejects patches that move live settings edits back to per-point CPU recomputation.

## Progress Policy

After each checkpoint with evidence, append a line to `docs/logs/ripples_tuning_goal_progress.md`:

```text
YYYY-MM-DD | Checkpoint | Proof | Status | Notes
```

## Blocker Rule

Stop and mark blocked only if the same blocker prevents progress across three goal turns, or if completion requires unavailable data, unavailable rendering infrastructure, or an unplanned renderer redesign. A slow or failing test is not a blocker until the likely fixes have been attempted.

