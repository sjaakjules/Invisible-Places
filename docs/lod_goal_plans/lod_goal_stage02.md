# lod_goal_stage02.md — Fast Basic Adaptive Core: Responsive, Bounded, No Raw Fallback

## Codex goal

```text
/goal Make adaptive LOD stable during camera navigation by preserving the best currently displayed high-detail set, incrementally filling newly visible regions while moving, and preventing idle-stop demotion flashes. During rotate/pan/zoom, reuse the existing high-detail draw items as the base, run throttled motion traversal for uncovered or under-covered screen regions, add close newly visible nodes at high quality, use compensated coarse reps only for distant/small projected regions, and apply updates only as non-destructive patches. When movement stops, refine from the displayed set instead of replacing it with coarse fallback; never display a lower-detail traversal over an equal-or-better existing exact/high-quality set.
```

## Required prior stage

Stage 01 must be complete. The project must build and have baseline diagnostics.

## Why this stage exists

The audit identifies likely lag sources that must be removed before visual polish:

- Beauty Adaptive or adaptive paths may fall back to raw source count when hierarchy/draw items are unavailable.
- A computed fragment budget may not be passed into representative emission.
- Async traversal can stay busy with stale camera/style requests.
- Active camera motion can rebuild and upload new fallback draw-item sets for
  every camera key, making navigation slower than full-source rendering even
  when the displayed adaptive set is otherwise cheap.
- Active camera motion can hold the old refined view but fail to admit newly
  visible close regions until idle.
- Idle-stop refinement can briefly demote to a coarse whole-view fallback even
  when a useful high-detail displayed base already exists.
- LOD coverage footprint can leak into visible point size, creating large square
  patches at node or cloud edges.
- Normal adaptive updates must not call `WaitIdle()` or `vkDeviceWaitIdle()`.

This stage tests the adaptive system in Fast Basic first because Fast Basic avoids Beauty-specific opacity, emission, falloff, EDL, and surfel costs.

## Scope for this stage

In scope:

- Fast Basic adaptive viewport path.
- Existing hierarchy/traversal/draw-item system.
- Budget enforcement for vertices and estimated fragments.
- Safe hierarchy-not-ready behaviour.
- Async traversal request superseding.
- Diagnostics needed to prove bounded adaptive behaviour.

Out of scope:

- No new `.ipcloud` bundle.
- No progressive chunk streaming.
- No raw chunk streaming or `.ipcloud` residency system. Motion coverage patches
  must use the existing hierarchy/draw-item path.
- No Beauty compensation.
- No continuous LOD visual smoothing beyond what is needed to avoid severe flashes. Stage 03 handles polish.
- No GPU compute traversal.

## Implementation tasks

1. Trace the Fast Basic draw path and identify every branch that can submit raw/full-source points when the selected mode is adaptive.
2. Ensure adaptive Fast Basic never silently submits full source because hierarchy or draw items are missing.
   - Use the previous valid adaptive draw-item set while traversal is pending.
   - If no previous set exists, use a bounded coarse fallback or show an explicit `waiting on adaptive LOD` state.
   - Keep `Full Source` / `Beauty Full Source` exact modes available only when explicitly selected.
3. Fix budget propagation so the same traversal/emit budget enforces:
   - `maxVertices` or representative count.
   - `maxEstimatedFragments`.
   - any current renderer-specific caps.
4. Add or repair tests around `TraversePointCloudLodHierarchy(...)` so a non-zero fragment budget actually limits emitted representatives.
5. Replace stale async traversal behaviour.
   - Add a request generation/key if needed.
   - Supersede older camera/style/layer requests with the newest request.
   - Discard stale worker results rather than uploading them.
   - Track max pending age or equivalent diagnostic.
6. Add a motion LOD hold path.
   - During active rotate/pan/zoom, reuse the current exact adaptive set first.
   - If no exact set exists, reuse a prior coarse fallback instead of rebuilding
     one for each new camera key.
   - Throttle async completion apply/upload to a fixed cadence suitable for a
     30 fps target.
   - Resume high-quality refinement after the camera has been idle for roughly
     300 ms.
7. Add motion coverage and no-flash idle refinement.
   - Do not cancel every in-flight traversal just because camera quantization
     changes during active navigation; let throttled traversals complete as
     motion coverage patches.
   - Track displayed base, motion patch, and idle candidate state separately in
     diagnostics.
   - Block any candidate or cached fallback that would demote an exact/high
     quality displayed base to a lower-density or coarse whole-view result.
   - Newly visible close/projected-large regions should enter through exact or
     high-quality motion patches; distant/projected-small regions may remain
     compensated coarse reps until idle refinement catches up.
   - On idle, keep rendering the displayed base while exact traversal refines;
     do not show a coarse interstitial.
8. Split coverage footprint from visible point size.
   - Keep coverage area for fragment budgets and opacity/emission compensation.
   - Use a separate visible render area in the draw item.
   - Preserve literal point size in sparse regions; allow only small capped
     growth where source spacing indicates full-source points already overlap.
9. Confirm normal adaptive draw-item updates do not call `WaitIdle()`, `vkDeviceWaitIdle()`, or equivalent full GPU idle points.
   - Add a debug counter/log/assert if practical.
   - Keep explicit manual debug/loading cap changes separate if they currently need a wait.
10. Update diagnostics so the overlay/log can show:
   - adaptive representative count
   - represented source count
   - submitted point/draw-item count
   - estimated fragments
   - current adaptive budget
   - fallback state: previous set, coarse fallback, waiting, or full-source explicit
   - motion hold/reuse state
   - stale traversal requests discarded
   - async traversal pending age/status
   - throttled adaptive apply/upload count
   - displayed base / motion patch / idle candidate state
   - motion patch count, last patch reps, and newly covered source estimate
   - demotion block count and skipped patch count
   - max adaptive visible render diameter
   - normal adaptive GPU idle/wait count

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
```

If possible, add a test that fails before the fragment-budget fix:

```text
Given a hierarchy and a small maxEstimatedFragments, traversal emits fewer/lower-cost draw items than the unlimited traversal and reports budget pressure.
```

Run `--lod-compare` if available:

```text
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places --lod-compare <cloud>
```

## Manual runtime checkpoint for the user

Run a representative large cloud in Fast Basic and do this:

1. Start from a cold launch with no warmed runtime traversal.
2. Orbit, pan, zoom in, and move quickly down a dense/long part of the site.
3. Watch the diagnostics overlay.
4. Repeat after the hierarchy cache is warm.

Expected behaviour:

- Navigation remains responsive.
- Rotation should target at least 30 fps, with 20 fps treated as the practical
  floor for the immediate hotfix.
- The submitted point/draw-item count does not jump to full source unless the user explicitly selected Full Source.
- Estimated fragments fall or stop increasing when the budget is reached.
- Async traversal does not stay stuck on old camera requests.
- Active camera motion does not build/upload a new coarse fallback every frame.
- Active camera motion does not cancel every in-flight traversal before it can
  complete; completed motion traversals can update coverage at the throttled
  cadence.
- Newly revealed close regions begin filling during motion rather than only
  after idle.
- Stopping camera motion does not flash to a lower-detail fallback before exact
  refinement.
- Sparse cloud edges do not show large square patches of oversized points.
- The view may be coarse while pending, but it should not go black or submit all raw points.
- Normal adaptive updates report zero GPU idle/wait.

## Completion condition

This stage is complete only when:

- Fast Basic adaptive mode is bounded and safe under camera motion.
- Tests prove fragment/representative budgets are active.
- Diagnostics expose enough state to catch future full-source fallback regressions.
- The user can run the viewport and confirm there is no major lag spike or hidden full-source submission.

## Stop conditions

Stop and report if:

- The codebase has no distinct Fast Basic adaptive path and creating one requires a broader renderer refactor.
- Fragment-cost estimates are unavailable and cannot be added without Stage 04 selection/stat work.
- GUI/manual validation is required and cannot be performed by the agent.

## Handoff to Stage 03

Pass along:

- Representative and fragment budget values that feel responsive.
- Any remaining visible popping or sudden coarse/fine transitions.
- Any places where the view becomes too sparse while staying within budget.
