# Water Features Goal

> Historical note (2026-08): the legacy Ripple and Field features (region effect
> layers, sparse ripple memberships, caustic style parameters, Field vector
> caches, Field Streamlines, Field Surface Motion, and `water_effect_*`
> composition) were removed from the codebase. Their checkpoints and validation
> items have been stripped from this document; the Shoreline, Rain, and Flow
> parts remain as the record of what was completed.

## Goal

Complete the Water feature integration so SAND Shoreline, GPU collision Rain, and Flow share one coherent base-cloud workflow while preserving the active v2 behavior.

Target command:

```text
/goal Complete WaterFeatures_GOAL.md: SAND Shoreline stays a point-style shader path, Rain uses a static role-aware collision cache plus dedicated GPU particles and impacts, Flow uses one animated trail abstraction, and all behavior is verified by focused tests plus full water regression checks.
```

This goal follows the OpenAI Codex goal guidance: keep one durable objective, record evidence as work progresses, state constraints and blocker conditions explicitly, and finish only when validation evidence exists.

Reference guidance:

- https://developers.openai.com/codex/use-cases/follow-goals
- https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex

## Outcome

- Flow creates animated generated trail clouds through one shared trail representation and point-cloud scalar schema. Rain instead uses persistent GPU particles, the static 10 mm role-aware cache built from a complete 2 mm role bundle, and bounded impact-event lookups on the displayed point cloud.
- Flow Bake Path should stay responsive for artist edits by reusing unchanged per-source path-cache branches and rebuilding only moved, added, deleted, or profile-changed emitters where support signatures still match.
- Flow Path can use an optional artist-placed attractor to bias horizontal route selection without changing the rule that Z is the vertical downhill axis.

## Constraints

- Preserve the active Water tabs: `Shoreline`, `Rain`, and `Flow`.
- Preserve base-cloud scalar mappings as the visual source of truth.
- Preserve interactive editing performance by minimizing CPU recomputation and GPU uploads. Prefer region-scoped support data and param-only updates when topology or selected membership has not changed.
- Preserve legacy compatibility: old Caustics/Basin/Runoff records load harmlessly and are omitted from new saves.
- Keep generated Flow output, Rain GPU runtime resources, and rain impact events out of normal project layer persistence.
- Keep the existing stream scalar contract stable unless tests and renderer/offline consumers are updated together.
- Keep changes scoped to water code, water UI orchestration, project/source serialization where needed, point-cloud renderer/offline hooks where needed, tests, and water docs.

## Checkpoints

1. Goal Docs
   - Create this file and `docs/logs/water_features_goal_progress.md`.
   - Record the goal objective, checkpoints, and validation policy.

2. Shared Trails And Rain
   - Keep the generated-trail overlay abstraction for Flow, and isolate Rain in its dedicated compute/draw pipeline.
   - Keep shared scalar names, visualization, renderer style, and offline export behavior.
   - Verify Flow trails animate without topology regeneration, and Rain settings update uniforms without rebuilding its collision cache or persistent buffers.

## Validation

Required evidence before completion:

```text
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug -R "Water|Shoreline|Rain|Offline water" --output-on-failure
ctest --test-dir build/macos-debug --output-on-failure
git diff --check
```

Focused tests must cover:

- SAND Shoreline shader behavior,
- Flow trail scalar schema and Rain feature isolation,
- source-specific Flow path cache invalidation and attractor-biased path ranking,
- Rain collision-cache hits, deterministic respawn, and role-isolated impact effects,
- viewport/offline water motion rendering at different times.

## Remaining Performance Direction

- Keep Flow stream overlays static-topology during playback. Keep Rain particle/event buffers permanently allocated and drive animation, weather, visibility, and impact controls through compact frame uniforms.

## Progress Policy

After each checkpoint with passing evidence, append a line to `docs/logs/water_features_goal_progress.md` with:

```text
YYYY-MM-DD | Checkpoint | Proof | Status | Notes
```

The goal is complete only when the required validation commands pass or any remaining unrun command is documented with a concrete blocker and safe follow-up.

## Blocker Rule

Stop and mark the goal blocked only if the same blocking condition prevents progress across three goal turns, or if completing the requested behavior would require an unplanned renderer redesign, unavailable source data, or tests that cannot be run in this environment.
