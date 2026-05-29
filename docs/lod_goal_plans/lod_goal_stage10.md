# lod_goal_stage10.md — Tile Overdraw Budgets and Conservative Occlusion

## Codex goal

```text
/goal Reduce worst-case Beauty overdraw and hidden-point work by adding tile-level fragment/blended-fragment budgets and conservative occlusion/depth-proxy culling, verified by diagnostics and stress views showing lower point-pass/accumulation cost without holes, missing visible features, or regressions in Fast Basic, Beauty, streaming, or exports. Between iterations, enable one conservative budget/culling layer, compare before/after metrics on the same camera path, and keep any risky rejection disabled until proven safe. If correctness is uncertain, prefer drawing too much over culling visible data.
See docs/lod_goal_plans/lod_goal_stage10.md for details of implementation and completion verification.
docs/point_cloud_adaptive_lod_fast_beauty.md describes the full adaptive LOD system with docs/lod_goal_plans/lod_goal_index.md describing the stages of development.
Please update docs/CURRENT_STATE.md, docs/LOD_Integration.md, and push a git update on completion of edits.
You can use Data/Site3-Sample-Terrestrial.ply to do tests initially as it is a section of the main cloud (Data/Site3-Mid-1mm100M.ply) with same density, only 12M as it covers an area 5m x 5m not 10m x 30m
```

## Required prior stage

Stage 09 should be complete. The system should have useful compare/export diagnostics and measured pass timings.

## Why this stage exists

The target design says the worst views often come from many translucent splats in the same screen area, not merely from total point count. The audit notes there is no tile overdraw budget yet and close/grazing views with large translucent marks can still overload blending work.

## Scope for this stage

In scope:

- Coarse screen tile fragment/blended-fragment estimates.
- Tile budget pressure diagnostics.
- Selection rules that keep low-priority nodes coarser or delayed when tiles exceed budget.
- Conservative normal-cone/backface culling for surfels where valid.
- Depth proxy or Hi-Z pyramid for conservative occlusion if practical.
- Debug visualizations/logs for tile pressure and culling decisions.

Out of scope:

- GPU compute traversal/compaction unless a minimal compute pass is needed for Hi-Z and is isolated.
- Aggressive occlusion that risks visible holes.
- Full mesh reconstruction or watertight proxy generation.

## Implementation tasks

1. Use Stage 06/09 metrics to identify worst-case Beauty views.
2. Add a coarse tile grid such as 16x16 or 32x32 pixels.
3. Estimate per-tile contribution during traversal/selection:
   - estimated fragments
   - estimated blended fragments
   - representative/node importance
   - renderer geometry mode
4. Define tile budgets by renderer/quality mode:
   - looser for Adaptive HQ/final
   - stricter for moving/interactive
   - explicit status when performance-limited
5. Modify selection when tile budgets are exceeded:
   - keep low-priority nodes coarser
   - reduce/clamp representative footprint in explicit preview modes
   - delay low-priority refinement until idle
   - do not drop high-importance scalar/emissive/edge representatives silently
6. Add conservative surfel culling where normals are reliable:
   - normal-cone/backface culling
   - disable or weaken when normal quality is unknown
7. Add a depth proxy or Hi-Z option if practical:
   - coarse depth shell, simplified proxy, or surfel depth prepass
   - reject only obviously hidden nodes
   - bias conservatively to avoid holes
8. Add diagnostics:
   - tiles over budget
   - percentage of screen/tile pressure
   - fragments/blended fragments before/after tile limiting
   - nodes kept coarse due to tile pressure
   - nodes/fragments culled by occlusion
   - culling disabled/uncertain state
   - optional tile pressure heatmap/debug overlay
9. Ensure export semantics remain correct:
   - Adaptive HQ may use conservative overdraw/occlusion if it does not alter visible results beyond threshold.
   - Full Source should not silently skip visible source data.

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Add tests where practical:

- tile accumulation for synthetic projected bounds
- tile budget pressure ordering keeps high-importance reps
- normal-cone/backface culling only applies with valid normals
- conservative occlusion bias does not reject near/visible test nodes
- Full Source mode bypasses adaptive tile dropping as appropriate

## Manual runtime checkpoint for the user

Use a known worst-case cloud/style:

1. Beauty screen sprites with large translucent radius.
2. Dense long/grazing view.
3. Camera close to dense geometry.
4. World surfels with normals if available.
5. Compare before/after metrics from Stage 09 if a baseline exists.

Expected behaviour:

- Beauty point/surfel or accumulation pass time improves in worst-case views.
- Tile budget diagnostics identify the overloaded areas.
- The image does not develop holes, missing silhouettes, or missing rare features.
- Fast motion may be labelled performance-limited but remains responsive.
- Idle/Adaptive HQ still refines toward acceptable quality.

## Completion condition

This stage is complete only when:

- Tile overdraw pressure is estimated and used by adaptive selection.
- Conservative occlusion/backface culling is implemented where safe or explicitly disabled where not safe.
- Diagnostics prove why the selector reduced work.
- Worst-case Beauty stress views are faster without visible correctness failures.

## Stop conditions

Stop and report if:

- Projected tile estimates are too inaccurate to guide selection safely.
- No reliable depth/normal data exists for conservative occlusion.
- Culling introduces holes that cannot be bounded conservatively in this stage.

## Handoff to Stage 11

Pass along:

- CPU selection hot spots and traversal timings.
- Tile/occlusion data that should move to GPU compute.
- Runtime feature constraints observed on MoltenVK.
