# Goal: Replace Fixed 10M Preview LOD With Adaptive Point-Cloud LOD

## Source Guidance

This document is shaped for Codex Goals, following OpenAI's guidance that a
good goal needs one durable objective, a verifiable stopping condition, files to
read first, progress evidence, checkpoints, and a clear completion contract:

- [Follow a goal](https://developers.openai.com/codex/use-cases/follow-goals)
- [Using Goals in Codex](https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex)

Codex should not mark this goal complete because the implementation seems
plausible. It is complete only when the repository evidence, tests, and audits
below prove that the old fixed 10M sampled-index preview LOD is gone and the
adaptive hierarchy path is implemented.

## Goal Prompt

Use this as the goal prompt for a long-running Codex thread:

```text
/goal Complete the point-cloud LOD migration by removing the fixed 10M sampled-index preview LOD and implementing the adaptive hierarchy/draw-item LOD architecture described in docs/LOD_Integration.md. Stop only when the app builds, the relevant tests pass, no old preview LOD UI/export/serialization/code path remains, and the final audit shows adaptive preview/export behavior is source-index-correct for scalar fields, colors, normals, water-effect fields, opacity, emission, and dynamic point-size geometry modes.
```

## Done Means

The goal is complete when all of these are true:

- No code path uses `kPointCloudPreviewLodTarget`, `PointCloudPreviewLodMode`,
  `PointCloudPreviewLodDecision`, `ResolvePointCloudPreviewLod(...)`, preview
  LOD sample caches, still-camera preview LOD preparation, or interactive
  sampled-index buffers as preview LOD.
- The UI no longer presents `Auto Camera LOD`, `Force LOD`, `Preview LOD`, or
  a hardcoded point LOD target backed by sampled-index caches.
- Export no longer uses `previewDensity` as a boolean quality mode. EXR/MP4
  export uses an explicit point-cloud density mode.
- Manual point budgets, if retained, are labeled and implemented as
  debug/loading caps, not adaptive LOD.
- Adaptive LOD is represented by explicit source-index-preserving hierarchy,
  representative, and draw-item structures.
- LOD shaders and renderer paths fetch source attributes through
  `drawItem.sourcePointIndex`.
- Scalar fields, water-effect fields, normals, source color, style bindings,
  opacity, emission, depth/AOV data, and dynamic point-size behavior stay
  source-correct through adaptive draw items.
- Pixel screen sprites, world-mm screen sprites, world surfels, and
  camera-facing world sprites have distinct footprint/cost rules.
- Viewport adaptive state can report representative count and represented source
  count.
- `FullSource` remains available for exact/debug rendering.
- `AdaptiveHighQuality` is the default/recommended Beauty export density.
- The final branch passes the verification commands listed in this document.

## Read First

Every agent must read these files before editing:

- `AGENT.md`
- `docs/LOD_Integration.md`
- `docs/CURRENT_STATE.md`
- `project_description.md`
- `CMakeLists.txt`

Agents working in a specific area must also read the relevant files listed in
their task brief.

## Operating Rules

- Work in checkpoints. Each checkpoint must leave the tree buildable or clearly
  state why it cannot yet build.
- Keep each agent's branch narrow. Avoid broad opportunistic refactors.
- Do not restore the old 10M sampled-index preview path under a new name.
- Do not present debug sampled budgets as adaptive rendering.
- Preserve project compatibility by ignoring or migrating old serialized
  preview LOD fields instead of crashing on them.
- Prefer adding new focused test files over expanding unrelated tests, but keep
  CMake/test integration simple.
- Update docs when public UI, export, serialization, or architecture contracts
  change.
- If a blocker prevents exact completion, produce a blocker audit with the file,
  command, failing evidence, and smallest next action.

## Progress Log

Maintain `docs/logs/lod_goal_progress.md` during implementation. Each agent
should append entries in this format:

```text
YYYY-MM-DD | Agent | Checkpoint | Files changed | Verification run | Status | Notes/blockers
```

The goal is not complete until this log has a final evidence-backed completion
entry.

## Agent Workstreams

### Agent 0 - Coordinator And Final Auditor

Primary files:

- `LOD_GOAL.md`
- `docs/LOD_Integration.md`
- `docs/CURRENT_STATE.md`
- `docs/logs/lod_goal_progress.md`

Responsibilities:

- Keep the task order coherent across agents.
- Own final acceptance and final audit.
- Resolve overlaps between renderer, app/UI, export, and serialization changes.
- Ensure docs match the implemented state.

Required final audit commands:

```text
rg -n "kPointCloudPreviewLodTarget|PointCloudPreviewLodMode|PointCloudPreviewLodDecision|ResolvePointCloudPreviewLod|Preview LOD|Auto Camera LOD|Force LOD|previewDensity|HqPreviewDensityExr|UpdateInteractivePointSampleBuffer|interactiveSampledIndex" src tests shaders CMakeLists.txt
rg -n "Auto Camera LOD|Force LOD|Preview LOD|Preview Density|Point LOD Target" README.md docs/CURRENT_STATE.md project_description.md PDR.md code_plan.md
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug --output-on-failure
```

Expected audit result:

- The active-code `rg` command must return no matches.
- The public-doc `rg` command must return no matches except deliberate removal
  notes that are immediately updated or moved into `docs/logs/`.
- `LOD_GOAL.md`, `docs/LOD_Integration.md`, and
  `docs/logs/lod_goal_progress.md` may mention old names as deletion targets or
  historical evidence; they must not describe them as usable product features.

### Agent 1 - Remove Old Preview LOD Surface

Primary files:

- `src/renderer/pointcloud/PointCloudPreviewState.hpp`
- `src/renderer/pointcloud/PointCloudPreviewState.cpp`
- `src/app/Application.cpp`
- `src/renderer/core/VulkanViewportShell.hpp`
- `src/renderer/core/VulkanViewportShell.cpp`
- `tests/**`

Responsibilities:

- Delete the fixed 10M preview LOD resolver and session cache fields.
- Delete still-camera preview LOD sample preparation.
- Delete interactive preview sampled-index buffers and draw-plan branches used
  only for old preview LOD.
- Preserve explicit/manual point budget behavior only if it is clearly a
  debug/loading cap.
- Rename or split `PointCloudPreviewState` if useful, but keep diffs scoped.

Required tests:

- Remove or replace `Point preview LOD resolver only applies automatic LOD to
  camera motion`.
- Add a test proving manual point budget sampling, if retained, is not exposed
  as preview LOD.
- Add an audit-style test or static assertion where practical that the old LOD
  resolver enum/types are gone.

Checkpoint evidence:

```text
rg -n "kPointCloudPreviewLodTarget|PointCloudPreviewLodMode|PointCloudPreviewLodDecision|ResolvePointCloudPreviewLod|previewLod" src tests
cmake --build build/macos-debug --target invisible_places_tests
```

### Agent 2 - Serialization And Export Density Modes

Primary files:

- `src/serialization/ProjectDocument.hpp`
- `src/serialization/ProjectDocument.cpp`
- `src/output/RenderPreset.hpp`
- `src/app/Application.cpp`
- `src/renderer/core/VulkanViewportShell.hpp`
- `src/renderer/core/VulkanViewportShell.cpp`
- `tests/**`

Responsibilities:

- Stop saving `point_cloud_preview_lod_mode`.
- Load legacy `point_cloud_preview_lod_mode` harmlessly and drop it on save.
- Replace `previewDensity` with an explicit density enum such as:

```cpp
enum class PointCloudExportDensityMode {
    FullSource,
    AdaptiveHighQuality,
    MatchViewportAdaptive,
    FastAdaptivePreview,
    ArtisticAsPreview,
    ArtisticHighQuality
};
```

- Rename `HqPreviewDensityExr` and UI labels to adaptive-density wording.
- Make Beauty EXR default to `AdaptiveHighQuality`.
- Keep `FullSource` available for exact/debug rendering.

Required tests:

- Legacy project with `point_cloud_preview_lod_mode` loads and saves without
  re-emitting that key.
- Export job defaults select `AdaptiveHighQuality` for Beauty EXR.
- Full-source export selection still forces raw source rendering.
- No user-facing label contains `Preview Density` after migration.

Checkpoint evidence:

```text
rg -n "point_cloud_preview_lod_mode|previewDensity|HqPreviewDensityExr|Preview Density|preview-density" src tests shaders CMakeLists.txt
cmake --build build/macos-debug --target invisible_places_tests
```

### Agent 3 - Adaptive LOD Data Model And CPU Traversal

Primary files:

- `src/renderer/pointcloud/**`
- `src/io/PointCloudData.*`
- `src/scene/**` if layer metadata is needed
- `tests/**`

Responsibilities:

- Add `PointCloudLodHierarchy`, `PointCloudLodNode`,
  `PointCloudLodRepresentative`, and `PointCloudDrawItemGpu`.
- Build a compact hierarchy from loaded point-cloud positions and bounds.
- Keep representatives source-index-preserving.
- Include represented count, spacing, node index, blend seed, and compensation
  fields needed by shaders.
- Implement CPU traversal first, with frustum/screen-error/fragment-cost
  estimates sufficient for deterministic tests.
- Keep raw source buffers canonical. Do not duplicate full point attributes per
  LOD level.

Required tests:

- Hierarchy build preserves total represented point count.
- Representatives reference valid source point indices.
- Traversal emits fewer representatives for far/coarse projected spacing and
  more for near/fine spacing.
- Traversal is deterministic for the same camera/style inputs.
- Draw items preserve `sourcePointIndex`, `representedCount`, `spacingMeters`,
  and `nodeIndex`.
- Empty/single-point/degenerate bounds cases do not crash.

Checkpoint evidence:

```text
cmake --build build/macos-debug --target invisible_places_tests
ctest --test-dir build/macos-debug --output-on-failure -R "lod|pointcloud"
```

### Agent 4 - Renderer Core And Shader Integration

Primary files:

- `src/renderer/core/VulkanViewportShell.hpp`
- `src/renderer/core/VulkanViewportShell.cpp`
- `shaders/pointcloud_lod_preview.vert`
- `shaders/pointcloud_surfel_lod_preview.vert`
- existing point-cloud shader includes as needed
- `CMakeLists.txt`
- `tests/**`

Responsibilities:

- Add per-layer adaptive LOD GPU resources and per-frame draw-item buffers.
- Extend draw planning with explicit draw sources:

```cpp
enum class PointCloudDrawSource {
    RawSequential,
    RawSampledDebugBudget,
    LodDrawItems,
    PaintedDrawItems
};
```

- Record adaptive LOD draws from draw-item buffers.
- Ensure LOD shaders fetch source positions, colors, normals, scalar fields, and
  water-effect fields via `drawItem.sourcePointIndex`.
- Implement point and surfel/camera-facing paths using draw items.
- Add LOD style data or an equivalent explicit LOD parameter block.
- Preserve raw/full-source and Fast Basic behavior.

Required tests/verification:

- Shader compilation includes new LOD shaders.
- Build fails if LOD shader bindings drift from host-side structures.
- CPU-side tests verify draw-source plan selection for raw, debug budget, and
  LOD draw items.
- Rendering integration is validated at least by build and shader compile; add
  screenshot/manual validation notes if automated viewport tests are not
  available.

Checkpoint evidence:

```text
cmake --build build/macos-debug --target invisible_places_shaders
cmake --build build/macos-debug --target invisible_places
cmake --build build/macos-debug --target invisible_places_tests
```

### Agent 5 - Geometry Footprint, Compensation, And Visual Correctness

Primary files:

- `src/renderer/pointcloud/**`
- `src/output/OfflinePointRenderer.*`
- `shaders/pointcloud_lod_preview.vert`
- `shaders/pointcloud_surfel_lod_preview.vert`
- `shaders/pointcloud_*.frag`
- `tests/**`

Responsibilities:

- Implement separate footprint/cost rules for:
  - pixel screen sprites
  - world-mm screen sprites
  - world surfels
  - camera-facing world sprites
- Include dynamic point size in projected footprint and fragment-cost estimates.
- Implement opacity compensation using represented count and footprint area.
- Implement emission compensation with policy hooks for average brightness,
  energy, and accent preservation.
- Keep scalar-driven style bindings source-correct.

Required tests:

- World-mm sprite projected size decreases with depth and affects LOD cost.
- Pixel sprite LOD respects literal user size unless density-fill/performance
  clamp is selected.
- World surfel/camera-facing world sprite LOD diameter grows from node spacing
  when needed to fill holes.
- Opacity compensation stays monotonic and bounded.
- Emission compensation clamps or preserves accents according to policy.
- Scalar-driven point size/opacity/emission resolves from source point indices,
  not draw-item indices.

Checkpoint evidence:

```text
cmake --build build/macos-debug --target invisible_places_tests
ctest --test-dir build/macos-debug --output-on-failure -R "lod|point|surfel|offline"
```

### Agent 6 - UI And User-Facing State

Primary files:

- `src/app/Application.cpp`
- `src/ui/**`
- `docs/CURRENT_STATE.md`
- `README.md` if public workflow language changes
- `tests/**`

Responsibilities:

- Remove old project setting UI for `Point Preview LOD`, `Auto Camera LOD`,
  `Force LOD`, and `Point LOD Target`.
- Add adaptive viewport quality/behavior controls:
  - `Beauty Adaptive`
  - `Beauty Full Source`
  - `Fast Basic`
  - later `Painted Adaptive`
- Add export density UI:
  - `Full Source`
  - `Adaptive HQ`
  - `Match Viewport Adaptive`
  - `Fast Adaptive Preview`
  - `Artistic As Preview`
  - `Artistic HQ`
- Add status overlay text for representative count, represented source count,
  target spacing, and idle refinement state.
- Keep UI labels honest: adaptive means hierarchy/draw items, not sampled-index
  budget.

Required tests/verification:

- Serialization/UI-facing labels no longer contain old LOD names.
- Status string formatting includes representative count and represented source
  count.
- Export density labels round-trip through user-facing helpers.
- Manual budget controls, if retained, are labeled debug/loading cap.

Checkpoint evidence:

```text
rg -n "Auto Camera LOD|Force LOD|Preview LOD|Point LOD Target|Preview Density" src tests README.md docs/CURRENT_STATE.md project_description.md PDR.md code_plan.md
cmake --build build/macos-debug --target invisible_places_tests
```

### Agent 7 - Tests, Benchmarks, And Final Regression Suite

Primary files:

- `tests/**`
- `CMakeLists.txt`
- `docs/logs/lod_goal_progress.md`

Responsibilities:

- Add focused test files if needed:
  - `tests/PointCloudLodTests.cpp`
  - `tests/PointCloudExportDensityTests.cpp`
  - or keep in `AssetDiscoveryTests.cpp` only if that is less disruptive
- Ensure CMake discovers all new tests.
- Add deterministic small-cloud fixtures in code, not large binary test assets.
- Add negative/audit tests for removed old LOD surface where practical.
- Run final full suite and document failures honestly.

Required final verification:

```text
cmake --build build/macos-debug --target invisible_places_shaders
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug --output-on-failure
```

## Dependency Order

Use this order unless a coordinator explicitly changes it:

1. Agent 1 removes old preview LOD types and cache plumbing.
2. Agent 2 migrates serialization/export density away from preview-density
   sampling.
3. Agent 3 adds adaptive LOD data model and CPU traversal.
4. Agent 4 wires renderer resources, draw plans, and shaders.
5. Agent 5 adds compensation and geometry-specific cost correctness.
6. Agent 6 updates UI/status language and docs.
7. Agent 7 runs final tests and strengthens coverage gaps.
8. Agent 0 performs final audit and completion decision.

Agents 2 and 3 may work in parallel after Agent 1 identifies the old surface.
Agents 4, 5, and 6 should not merge before Agent 3's public LOD structures are
stable.

## Evidence Matrix

| Requirement | Evidence |
| --- | --- |
| Old fixed 10M LOD removed | `rg` audit has no active-code matches for old LOD symbols |
| Legacy project compatibility | Tests load old `point_cloud_preview_lod_mode` and save without it |
| Adaptive data model exists | Tests construct hierarchy and validate source-index representatives |
| Source correctness | Tests prove scalar/style fields resolve through `sourcePointIndex` |
| Geometry modes distinct | Tests cover pixel sprites, world-mm sprites, world surfels, camera-facing world sprites |
| Compensation correct | Tests cover opacity/emission coverage scaling and bounds |
| Renderer integrated | Shader target builds and app target builds |
| Export density explicit | Tests cover `PointCloudExportDensityMode` labels/defaults/behavior |
| UI honest | Label audit removes old preview LOD wording |
| End-to-end health | Full build and `ctest` pass |

## Completion Audit Template

The final agent must append this to `docs/logs/lod_goal_progress.md`:

```text
Completion audit
- Objective:
- Final commit/branch:
- Files changed summary:
- Old LOD rg audit result:
- Build commands run:
- Test commands run:
- Manual visual checks, if any:
- Known residual risks:
- Evidence-backed completion decision:
```

Mark the Codex Goal complete only if the completion decision is backed by
passing evidence. If any required command cannot run, record the command,
failure, and the smallest next action instead of declaring completion.
