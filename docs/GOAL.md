# Water Features v2 Goal

Companion goal document for `docs/water_features_redesign_v2.md`.

## Goal Statement

/goal Finish the remaining Water Features v2 work described in `docs/water_features_redesign_v2.md` and `docs/GOAL.md`. The final system must keep the active Water workflow as Ripples, Flow, and Field; keep legacy Caustics loading as Ripple / Caustic Lace; remove Basin and Runoff and keep Basin and Runoff absent from active UI/new saves and harmlessly ignored on legacy load; and complete the unfinished v2 behavior: Ripples and Field Surface Motion modify the active/base cloud visual evaluation through composable Visuals-compatible effects, Flow Streams visibly animate through shader/Visuals playback, Field can build from user-defined regions, Ripple and Field regions preserve concave clicked boundaries, and Field streamlines stay surface-bound with valid bridge/fade/termination behavior. Preserve existing non-water rendering, project portability, Flow path cache reuse, branch hiding, legacy project load safety, and the rule that water visuals do not require PLY export. Use only the local repository, docs, source, shaders, tests, CMake/ctest, and generated local artifacts. Between iterations, run the narrowest relevant verification, inspect the first failing evidence, choose the smallest defensible next change, and record a short progress-log checkpoint in `docs/water_features_v2_acceptance_log.md`. If blocked or no valid path remains, stop and report the unmet requirement, evidence, attempted approaches, current risk, and the exact decision, asset, permission, or scope change needed to continue.

## Definition Of Done

Done means all of these are true:

- Existing accepted v2 baseline remains intact: active tabs are Ripples/Flow/Field; schema `24` saves v2 keys; new saves omit Basin/Runoff/Caustic region keys; legacy Caustics migrate to Ripple / Caustic Lace; legacy Basin/Runoff records load harmlessly and are ignored.
- Ripples modify active/base cloud final visual values through composable effect contributions, not primarily through copied generated/effect point overlays or growing replacement point patterns.
- Field Surface Motion modifies active/base cloud final visual values through the same composable effect model.
- Visuals can stack Ripple and Field Surface Motion effects with add, multiply, max/screen, colourise, opacity, size, and emission contributions while preserving existing base mappings.
- Flow Streams visibly animate through shader/Visuals time playback without requiring topology regeneration.
- Flow path cache generation, cache reuse, branch hiding, and stream-setting refresh behavior still work.
- Field can build from Flow paths and from user-defined selected regions.
- Ripple and Field regions preserve concave clicked boundaries; a C-shaped region excludes the cut-out area.
- Field streamlines stay surface-bound, bridge only valid gaps, and terminate/fade on rejected gaps.
- Generated stream scalar fields exactly include the public contract fields.
- Viewport, EXR export, and MP4 export include water output without requiring water PLY export.
- Build and full tests pass.
- Focused tests or artifacts prove every remaining checkpoint below.
- `docs/water_features_v2_acceptance_log.md` records accepted evidence for each checkpoint.

## Checkpoints

Implement and verify in this order unless a smaller dependency-driven order is clearly safer:

1. **Effect Composition**
   Ripples and Field Surface Motion compose with active/base cloud visuals. Existing base `Height`, `Intensity`, colour, opacity, size, and emission mappings remain available and are not replaced.

2. **Concave Regions**
   Ripple and Field region containment uses the authored clicked polygon boundary. C-shaped regions reject points in the cut-out.

3. **Flow Motion**
   Flow Streams visibly change over time through shader/Visuals playback. Stream count/length changes may rebuild stream overlays; colour/emission/opacity/size/speed/phase playback changes must not rebake paths.

4. **Field Regions**
   Field can build from a user-selected region using the Ripple-style region workflow. Generated Field cache/streamline output stays inside the selected region.

5. **Surface Safety**
   Field/stream projection, bridge, no-flow/block, and termination/fade behavior is verifiable through tests or debug artifacts.

## Progress Log Format

Keep a short progress log in `docs/water_features_v2_acceptance_log.md` under `## Progress Log`.

```text
- YYYY-MM-DD | Checkpoint: <name> | Proof: <exact command or artifact> | Status: partial/accepted | Notes: <one sentence>
```

## Required Verification Evidence

Stopping commands:

```text
cmake --build --preset build-macos-debug-home-vcpkg
ctest --test-dir build/macos-debug --output-on-failure
ctest --test-dir build/macos-debug -R "Water v2|Water ripple|Offline ripple|Offline water stream|Legacy water region|Project document round-trips|Point material variant" --output-on-failure
```

Required proof artifacts:

- Tests proving C-shaped Ripple and Field regions exclude the cut-out area.
- Tests proving Ripple and Field Surface Motion compose with active/base cloud mappings.
- A viewport or offline render artifact showing Flow Streams at two different times with visible animation and no topology rebuild.
- A test or artifact showing Field built from a selected user region.
- Tests or debug artifacts showing accepted bridges, rejected gaps, no-flow/block behavior, and termination/fade behavior.
- Manual acceptance log covering: create Ripples, bake Flow paths, generate animated Flow Streams, build Field Streamlines from paths, build Field from a selected region, build Field Surface Motion, reload project, export EXR/MP4, and confirm base-cloud scalar mappings remain intact.

Current acceptance evidence is recorded in `docs/water_features_v2_acceptance_log.md`.

## Blocked Stop Condition

Stop only when no defensible implementation path remains under the current repository, tool, data, or permission limits. The stop report must include:

- The unmet Definition of Done item.
- The exact failing evidence.
- What was attempted.
- Why the remaining paths are unsafe or invalid.
- What would unlock progress.
