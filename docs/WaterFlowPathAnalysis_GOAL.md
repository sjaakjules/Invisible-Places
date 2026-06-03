# Water Flow Path Analysis Goal

## Goal

Implement cached Path Analysis and dynamic analysed-corridor Lanes so Flow can expand, contract, slow down, speed up, form eddies, expose ripple-prone regions, and stay fast to edit.

Target command:

```text
/goal Complete docs/WaterFlowPathAnalysis_GOAL.md: implement cached path analysis, dynamic analysed-corridor lanes, path-view diagnostics, fast lane/trail edit invalidation, and Site3 fixture validation.
```

This goal follows the OpenAI Codex goal guidance: use one durable objective, define a verifiable stopping condition, keep constraints explicit, work in checkpoints, record evidence, and stop only when validation passes or a concrete blocker is documented.

Reference guidance:

- https://developers.openai.com/codex/use-cases/follow-goals
- https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex

## Outcome

- Flow uses four explicit stages: Path bake, Path Analysis, Lanes, and Trail.
- Path bake remains the only stage that calls `GenerateWaterPathCache`.
- Path Analysis computes cached local values for slope, flatness, curvature, neighbor density, nearest path distance, confluence, channel width, speed, turbulence, eddy potential, and ripple potential.
- Lanes generate dynamic routes through the analysed corridor instead of fixed global path offsets.
- Close top paths merge into wider regions, isolated paths stay thinner, flat areas spread and slow down, steep areas narrow and speed up, bends/roughness create eddies, wavy regions get lateral curl/noise, and ripple-prone regions are exposed.
- Trail style edits do not recalculate lanes or paths. Trail geometry edits may resample surfels from cached lane routes only.
- Path View can colour paths by the cached analysis values so the user can see why Flow behaves differently in different terrain regions.
- The Site3 fixture in `tests/water_flow_path_analysis_site3_fixture.json` validates the reference source and screenshot path settings.

## Constraints

- Preserve the current Water > Flow Path, Lanes, and Trail profile workflow.
- Preserve existing project/source serialization compatibility.
- Preserve generated water sessions being excluded from base-cloud Visuals and export visual selectors.
- Preserve internal `stream_*` scalar names and order unless all renderer, offline, serialization, and tests are migrated together.
- Keep path analysis cached with or beside `WaterPathCache`; do not repeat terrain/path-neighbor analysis per generated trail.
- Keep Lane and Trail editing fast. Do not rebuild or upload full generated trail clouds for every intermediate slider drag.
- Keep the first implementation an artist-directed terrain/path-field model, not a full shallow-water solver.

## Checkpoints

1. Documentation And Fixture
   - Create `docs/water_flow_path_analysis_lanes.md`.
   - Create this goal document.
   - Create `docs/logs/water_flow_path_analysis_goal_progress.md`.
   - Create `tests/water_flow_path_analysis_site3_fixture.json`.

2. Path Analysis Model And Cache
   - Add path-analysis structs and deterministic calculation APIs.
   - Cache analysis with or beside path cache.
   - Recompute analysis when missing, stale, or invalidated by Path settings/source changes.
   - Add serialization/backward-compatible loading if analysis is persisted.

3. Analysed-Corridor Lanes
   - Replace fixed global-offset lane generation for Flow with analysed-corridor route generation.
   - Use channel width, speed, turbulence, eddy potential, and ripple potential from Path Analysis.
   - Keep generated output compatible with current Trail rendering and `stream_*` scalar consumers.

4. Fast Edit Invalidation
   - Path edits invalidate paths, analysis, lanes, and trails.
   - Path-analysis edits invalidate analysis, lanes, and trails only.
   - Lane topology edits invalidate lane routes and trail samples only.
   - Lane param-only edits update scalar/style data where possible.
   - Trail style edits update style only.
   - Trail geometry edits resample Trail surfels without recomputing lanes or paths.
   - Slider drags use apply-on-release, debouncing, preview caps, or scalar-only updates to avoid repeated full rebuild/upload.

5. Path View Diagnostics
   - Add Path View colour modes for Branch, Slope, Flatness, Curvature, Neighbor Density, Confluence, Channel Width, Speed, Turbulence, Eddies, and Ripples.
   - Ensure diagnostic colours come from the same cached values used by Lanes.

6. Validation And Evidence
   - Add focused synthetic tests for steep, flat, nearby, and curved paths.
   - Add Site3 fixture validation.
   - Add invalidation/performance tests for Path/Lanes/Trail edit separation.
   - Run required validation commands and record evidence in the progress log.

## Required Validation

```text
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug -R "Water|Flow|Lane|Path analysis|Offline water" --output-on-failure
ctest --test-dir build/macos-debug --output-on-failure
git diff --check
```

Focused tests must cover:

- path-analysis serialization/cache compatibility,
- legacy caches/projects without analysis recomputing safely,
- deterministic analysis values from the same path cache/settings,
- synthetic steep paths producing higher slope/speed and narrower channel width,
- synthetic flat fans producing higher flatness, wider channel width, and lower speed,
- nearby branches producing higher neighbor density, confluence, and width expansion,
- curved paths producing higher curvature, eddy potential, and ripple potential,
- Site3 fixture path bake and finite normalized analysis values,
- Site3 channel width variation, broad/confluence region detection, and narrow/steeper region detection,
- lane edits not dirtying or recomputing path bake,
- Trail style edits not recomputing lanes or paths,
- Trail geometry edits resampling trail surfels without recomputing lane routes or paths,
- speed-only Lane edits using scalar/style updates where possible,
- slider drags avoiding repeated full trail rebuild/upload.

## Agent Coordination

Parent agent owns integration, docs, fixture, validation, and final acceptance.

Worker A owns path-analysis structs, cache integration, serialization compatibility, and analysis tests.

Worker B owns analysed-corridor lane generation, lane-route caching, invalidation boundaries, and performance behavior.

Worker C owns Path View diagnostics, UI colour modes, edit-latency instrumentation, and UI-adjacent tests.

Workers are not alone in the codebase. Each worker must keep changes scoped to its ownership area, avoid reverting changes made by others, and report files changed, tests run, and blockers.

## Progress Policy

After each checkpoint with evidence, append a line to `docs/logs/water_flow_path_analysis_goal_progress.md`:

```text
YYYY-MM-DD | Checkpoint | Proof | Status | Notes
```

The goal is complete only when required validation passes or any unrun command is documented with a concrete blocker and safe follow-up.

## Blocker Rule

Stop and mark blocked only if the same blocker prevents progress across three goal turns, or if completion requires unavailable data, unavailable rendering infrastructure, or an unplanned renderer redesign. A slow or failing test is not a blocker until likely fixes have been attempted.
