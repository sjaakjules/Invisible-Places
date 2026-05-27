# lod_goal_stage01.md — Restore Build, LOD Tests, and Baseline Evidence

## Codex goal

```text
/goal Restore the adaptive point-cloud LOD build and capture repeatable baseline evidence, verified by a successful macOS debug build, focused point-cloud LOD tests, full ctest, and a documented large-cloud baseline run, while avoiding unrelated LOD feature work. Use LOD_Integration.md and the adaptive LOD target design as the required context. Between iterations, fix the smallest build/test blocker, rerun the relevant command, and record what changed. If blocked by missing source data or an unavailable GUI, stop with the exact blocker, commands already run, and the manual validation steps needed from the user.
```

## Required reading before editing

- `LOD_Integration.md`
- `docs/point_cloud_adaptive_lod_fast_beauty.md`
- Current source around:
  - `src/app/Application.cpp`
  - `src/renderer/pointcloud/PointCloudLodHierarchy.hpp`
  - `src/renderer/pointcloud/PointCloudLodHierarchy.cpp`
  - focused point-cloud LOD tests
  - diagnostics and `--lod-compare` code paths

## Current known issue

The integration audit says the current worktree does not build because `Application.cpp` calls:

```cpp
BuildPointCloudLodHierarchy(cloud, buildConfig, progressCallback)
```

but the `.cpp` currently defines only the two-argument version. Do not trust runtime performance until this is fixed and tests pass.

## Scope for this stage

Implement only the minimum required to make LOD build/test/run evidence trustworthy.

In scope:

- Resolve the missing progress-callback overload or otherwise make declaration/definition/call sites consistent.
- Preserve the two-argument API if current code or tests use it.
- Add or repair tests for the overload and progress callback behaviour if practical.
- Confirm existing cache validation and traversal tests still pass.
- Capture a repeatable baseline using the existing diagnostics overlay and/or `--lod-compare`.
- Document the baseline in a small generated note or console/log output if the repo already has a diagnostics location.

Out of scope:

- Do not add `.ipcloud` bundles.
- Do not implement new renderer features.
- Do not tune visual LOD popping yet.
- Do not add GPU timestamp queries yet.
- Do not remove existing fallback behaviour except to make the app build-safe.

## Implementation tasks

1. Inspect the actual signatures and call sites for `BuildPointCloudLodHierarchy`.
2. Implement the missing three-argument overload in the `.cpp` or adjust the source consistently if the intended API has changed.
3. Make the two-argument overload delegate to the three-argument overload with an empty/no-op callback, or vice versa, so behaviour is centralized.
4. If the progress callback is meant to report monolithic build progress for now, call it at deterministic coarse points such as start, root/upper hierarchy complete, representative build complete, and done. Do not claim real chunk-progress until Stage 07.
5. Ensure failed/cancelled builds do not publish corrupt cache files.
6. Run focused LOD tests. Add a small test if an overload can regress silently.
7. Run the full test suite.
8. Capture a baseline on a representative large cloud. Use existing diagnostics if available rather than inventing a large new subsystem.
9. Save or report these baseline values:
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
   - peak resident memory if available

## Automated verification

Run these commands from a clean worktree state after edits:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
```

If a representative cloud is available:

```text
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Inspect:

```text
Saved/diagnostics/lod_compare/lod_compare_metrics.json
```

## Manual runtime checkpoint for the user

Launch the app with a representative large cloud and check:

- The app starts without a link/runtime failure.
- Fast Basic renders normally.
- Beauty Adaptive renders something useful or shows an explicit pending/cache status.
- The diagnostics overlay shows non-empty adaptive LOD values.
- A clean second launch gives similar baseline metrics.

## Completion condition

This stage is complete only when:

- The build succeeds.
- Focused point-cloud LOD tests pass.
- Full `ctest` passes, or any unrelated failing tests are identified with evidence.
- A repeatable baseline run exists.
- The agent reports any observed full-source adaptive fallback, lag, or missing diagnostics as Stage 02 input.

## Stop conditions

Stop and report instead of continuing if:

- The project cannot build for reasons unrelated to LOD and no safe local fix is available.
- The representative cloud path is unknown.
- The app requires GUI validation the agent cannot perform.
- Fixing the build would require changing public renderer semantics beyond the overload/progress issue.

## Handoff to Stage 02

Pass along:

- Any baseline metrics captured.
- Whether Beauty Adaptive or Fast Basic ever submitted the full source unexpectedly.
- Any diagnostic gaps that make Stage 02 difficult to verify.
