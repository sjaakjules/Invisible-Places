# Water Features Goal

## Goal

Complete the Water feature integration so Ripples, Flow, and Field share the same underlying region and trail concepts while preserving the active v2 workflow.

Target command:

```text
/goal Complete WaterFeatures_GOAL.md: Ripples affect only the base cloud through sparse GPU/offline runtime memberships and live parameter updates, Flow and Field share one animated trail abstraction, region Field vector caches are saved/reused offline, and all behavior is verified by focused tests plus full water regression checks.
```

This goal follows the OpenAI Codex goal guidance: keep one durable objective, record evidence as work progresses, state constraints and blocker conditions explicitly, and finish only when validation evidence exists.

Reference guidance:

- https://developers.openai.com/codex/use-cases/follow-goals
- https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex

## Outcome

- Ripples modify the active/base point cloud through sparse region memberships and compact procedural parameter buffers only. Ripple calculations may use sparse in-memory effect/debug points, but the active workflow must not create a visible `-Ripples.generated` particle/cloud session or dense active-cloud `water_effect_*`/`ripple_*` scalar uploads for ordinary Ripple recalculation.
- Ripple pattern and contribution edits should be live-editable at millisecond-scale latency after membership exists. Region membership should upload only when the target layer, region boundary, visibility, or selected point set changes; pattern, response, blend, colour, opacity, size, emission, speed, and phase changes should update compact GPU/offline params.
- Flow and Field both create animated generated trail clouds through one shared trail representation and point-cloud scalar schema. Flow trails follow baked path anchors. Field trails follow paths integrated from a vector field.
- Field regions can be drawn, converted into vector fields, saved to an offline cache, and reused when the source layer, region fingerprint, and field settings still match. Path-anchor Field caches may remain rebuildable from Flow anchors unless they become expensive enough to require their own persisted cache.
- Region selection is reusable across Ripple and Field. It must quickly return selected base-cloud point indices, edge weights, normals, scalar-field values, and editable per-point field metadata.
- Field trail source points come from existing water emitters when possible, with deterministic random spawn perturbation. If no emitter is usable, Field trails seed from selected region support points.
- Field Surface Motion and other future water effects should move toward the Ripple performance model where practical: region-bounded sparse support, small uploads, shader/offline-side procedural evaluation, and parameter-only updates for visual edits instead of whole-cloud recomputation.

## Constraints

- Preserve the active Water tabs: `Ripples`, `Flow`, and `Field`.
- Preserve base-cloud scalar mappings as the visual source of truth.
- Preserve interactive editing performance by minimizing CPU recomputation and GPU uploads. Prefer region-scoped support data and param-only updates when topology or selected membership has not changed.
- Preserve legacy compatibility: old Caustics load as Ripple `Caustic Lace`; old Basin/Runoff records load harmlessly and are omitted from new saves.
- Keep generated Flow and Field output out of normal project layer persistence.
- Keep the existing stream scalar contract stable unless tests and renderer/offline consumers are updated together.
- Keep changes scoped to water code, water UI orchestration, project/source serialization where needed, point-cloud renderer/offline hooks where needed, tests, and water docs.

## Checkpoints

1. Goal Docs
   - Create this file and `docs/logs/water_features_goal_progress.md`.
   - Record the goal objective, checkpoints, and validation policy.

2. Shared Regions
   - Add a reusable region-selection API over `WaterEffectLayer`.
   - Use it for Ripple effect generation and Field cache generation.
   - Verify concave regions, no-flow controls, and selected point metadata.

3. Base-Cloud Ripples
   - Stop the active Ripple refresh path from creating visible generated Ripple sessions.
   - Preserve sparse GPU/offline base-cloud Ripple evaluation and existing Ripple procedural differences.
   - Verify parameter-only live edits avoid dense scalar uploads when region membership has not changed.
   - Verify base Height/Intensity mappings remain active while Ripple and Field effects are applied.

4. Shared Trails
   - Add a shared generated-trail overlay abstraction for Flow and Field.
   - Keep shared scalar names, visualization, renderer style, and offline export behavior.
   - Verify Flow and Field trails animate through time without topology regeneration.

5. Field Cache Persistence
   - Save region Field vector caches to `Saved/water/<source-stem>-WaterFieldCache.bin`.
   - Reuse region caches when source, settings, and region fingerprints match.
   - Mark caches stale and rebuild when any fingerprint changes.

6. Field Source Perturbation
   - Build Field trail paths from emitter source points with deterministic random perturbation.
   - Fall back to selected support points when no emitter is valid.
   - Verify seed stability, seed variation, no-flow/bridge behavior, and vector-following direction.

## Validation

Required evidence before completion:

```text
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug -R "Water|Ripple|Field|Offline water" --output-on-failure
ctest --test-dir build/macos-debug --output-on-failure
git diff --check
```

Focused tests must cover:

- concave region subset selection and shared Ripple/Field containment,
- Field control regions affecting selected/cache nodes,
- Ripple base-cloud-only active output,
- Ripple parameter-only live update behavior and sparse upload counts,
- shared Flow/Field generated trail scalar schema,
- Field cache save/load/invalidation,
- Field source perturbation determinism and vector-following movement,
- viewport/offline water motion rendering at different times.

## Remaining Performance Direction

- Keep Ripples on the sparse runtime path: region changes may rebuild memberships, but visual edits should update params only.
- Use Ripple as the performance template for Field Surface Motion where practical. Field region selection should bound CPU work to selected/cache nodes, and shader/offline-side evaluation should replace dense base-cloud field uploads for editable visual parameters when the implementation is ready.
- Keep Flow and Field stream overlays static-topology during playback; stream animation should come from route/scalar data, shader/offline time, and compact setting changes rather than regenerating points for every frame.

## Progress Policy

After each checkpoint with passing evidence, append a line to `docs/logs/water_features_goal_progress.md` with:

```text
YYYY-MM-DD | Checkpoint | Proof | Status | Notes
```

The goal is complete only when the required validation commands pass or any remaining unrun command is documented with a concrete blocker and safe follow-up.

## Blocker Rule

Stop and mark the goal blocked only if the same blocking condition prevents progress across three goal turns, or if completing the requested behavior would require an unplanned renderer redesign, unavailable source data, or tests that cannot be run in this environment.
