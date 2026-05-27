# lod_goal_stage03.md — Fast Basic Smooth Transitions: No Noticeable Popping

## Codex goal

```text
/goal Make Fast Basic adaptive LOD transitions visually smooth during normal navigation, verified by a large-cloud run where density changes, parent/child replacement, and idle refinement do not produce noticeable popping, sudden detail loading, flicker, or large density jumps. Preserve Stage 02 boundedness and do not add Beauty-specific opacity/emission work. Between iterations, use diagnostics and manual/recorded viewport observations to identify the most visible transition artifact, fix that artifact, and rerun the same camera path. If smoothness cannot be verified automatically, stop with a precise manual acceptance checklist.
```

## Required prior stage

Stage 02 must be complete. Fast Basic adaptive mode must already be bounded and responsive.

## Desired user-visible result

The user should be able to move through the large point cloud in Fast Basic without noticing adaptive LOD changing chunks or points. It is acceptable for the view to be lower detail during fast motion, but the transition into and out of that lower detail must feel stable and intentional rather than like popping, flashing, or sudden loading.

## Scope for this stage

In scope:

- Fast Basic adaptive transition stability.
- Continuous/stable representative rank.
- Hysteresis for refinement and demotion.
- Smooth budget/density changes.
- Parent/child transition handling.
- Idle refinement cadence.
- Transition diagnostics.

Out of scope:

- Beauty opacity/emission compensation.
- Representative-class quality improvements beyond what is needed for smoothness.
- `.ipcloud` streaming.
- GPU timestamp governor.
- GPU compute traversal.

## Implementation tasks

1. Add or verify a stable `lodRank` or equivalent per representative.
   - It must be deterministic across frames and cache loads.
   - It should produce spatially useful prefixes, not random clumps.
   - If an ideal blue-noise/farthest-point order is too large for this stage, use a deterministic stratified/Morton/hash rank and document the later improvement for Stage 04.
2. Use rank-based density selection instead of hard arbitrary representative set swaps where possible.
   - For example, drawing 25% of a node's reps should select `lodRank < 0.25`.
3. Add refinement/demotion hysteresis.
   - Use separate thresholds for promoting and demoting nodes.
   - Avoid rapid flip-flop near a projected-spacing threshold.
   - Consider camera velocity and minimum dwell time.
4. Smooth budget changes.
   - Clamp density/budget changes per frame or per second.
   - Avoid abrupt representative-count jumps when FPS/budget changes.
   - Preserve Stage 02 caps and never overshoot safety budgets to look smooth.
5. Add parent/child transition handling for Fast Basic.
   - Prefer stable stochastic/dithered survival if opaque point blending is not available.
   - Use `lodBlend`, transition age, or stable hash fields if the draw-item format supports them.
   - Ensure transitions are deterministic for the same camera path.
6. Make idle refinement incremental.
   - When the camera stops, refine visible detail over multiple frames.
   - Do not load/replace a large visible region in one abrupt frame.
   - Show a small diagnostics/status state such as `Refining point cloud detail...` if present in the UI model.
7. Add transition diagnostics:
   - representative count delta per frame
   - nodes promoted/demoted per traversal
   - active transition count
   - average/max transition age
   - hysteresis state or threshold band
   - idle refinement pending/complete

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
```

Add tests where practical:

- Repeated traversal with the same camera/style returns stable representative IDs/order.
- Small camera movements near a threshold do not alternate promote/demote every traversal.
- Lowering a budget selects a prefix/subset using stable rank rather than a different random set.
- Two identical traversal inputs produce deterministic draw-item IDs.

## Manual runtime checkpoint for the user

Use Fast Basic on a representative large cloud and test:

1. Slow orbit around dense geometry.
2. Slow dolly/zoom through a threshold where LOD changes.
3. Fast navigation down a long dense site, then stop.
4. Repeated back-and-forth movement over the same camera path.
5. Warm-cache launch and cold-cache launch if Stage 01/02 caches are relevant.

Expected behaviour:

- No noticeable popping during slow camera motion.
- No large sudden detail loading when stopping; detail refines gradually.
- No flicker or crawl caused by unstable stochastic selection.
- No obvious checkerboard/noise pattern from dither transitions.
- Representative count changes are gradual or explained by diagnostics.
- Stage 02 boundedness remains intact.

## Completion condition

This stage is complete only when:

- Fast Basic adaptive LOD feels visually smooth enough for the user to navigate without noticing LOD mechanics.
- Automated tests cover determinism/hysteresis basics.
- Diagnostics can explain remaining transitions.
- The user has a clear manual checklist for accepting or rejecting smoothness before Stage 04 begins.

## Stop conditions

Stop and report if:

- The current draw-item/shader format cannot carry transition information and a wider ABI change is required.
- Current representatives are too poor for smoothness even with stable ranking; document what Stage 04 must fix.
- The agent cannot perform the viewport check and no recorded camera path/test harness exists.

## Handoff to Stage 04

Pass along:

- Any visible artifacts caused by low-quality representatives rather than transition mechanics.
- Current rank-generation method and limitations.
- Desired budget/hysteresis defaults from the user's accepted run.
