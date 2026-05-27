# Adaptive LOD Suggested Revisions

Date: 2026-05-28

## Summary

The target design in `docs/point_cloud_adaptive_lod_fast_beauty.md` remains valid. The current issue is an implementation gap: active camera motion was still allowed to churn traversal/fallback draw-item sets, newly visible regions were not allowed to refine while moving, idle stop could briefly demote to a coarse fallback, and the draw-item coverage footprint was also being used as the visible point size.

## What Changed

- During active camera rotation, pan, or zoom, prefer holding a reusable adaptive draw-item set instead of rebuilding a new coarse fallback for each camera key.
- Keep async traversal in the background, but throttle completion apply/upload to a 33 ms cadence so the viewport does not spend every moving frame rewriting draw-item buffers.
- Let an in-flight motion traversal finish instead of cancelling it for every quantized camera-key change. The completed traversal becomes a motion coverage update for newly visible or under-covered regions.
- Track displayed base, motion patch, and idle candidate state separately enough for diagnostics and apply policy.
- Block lower-detail candidates from replacing an equal-or-better displayed exact/high-quality base. Coarse fallback may still be used when nothing useful exists, but not as an idle-stop interstitial over an already displayed refined set.
- Split draw-item footprint semantics without changing the GPU layout:
  - `params.x` remains coverage and fragment-budget area.
  - `params.y` remains opacity compensation.
  - `params.z` remains emission compensation.
  - `params.w` is now visible render area.
- Shaders now use `params.w` for point/surfel size, so LOD representatives no longer become visibly huge just because their coverage/budget area is large.

## Why This Should Help

The 3fps motion case matches CPU and upload churn more than raw rendering cost. A stationary adaptive set can render near 60fps, but moving the camera was forcing new traversal keys, fallback generation, draw-item cache churn, and buffer uploads. Holding a dense adaptive set while moving keeps the GPU workload stable and lets the renderer spend frames drawing rather than rebuilding representation.

The black or sparse newly revealed regions came from the first hotfix being too conservative: it held the old displayed set, but it also kept superseding in-flight traversals before they could complete. Allowing bounded motion traversals to finish gives the renderer a way to fill newly visible close regions while navigation is still active.

The idle flash came from allowing a lower-detail cached or coarse fallback to become the displayed view while exact traversal caught up. A quality-aware apply policy treats that as a demotion and keeps the existing base visible until an equal-or-better candidate is ready.

The square large-point artifact came from using node coverage area as visible point area. Coverage area is useful for budget and opacity/emission compensation, but it should not automatically cover holes that exist in the full source cloud.

## Visual Consequences

- Active navigation should look much closer to the last refined still view, with a stable density while the camera moves.
- Newly revealed close regions should begin filling during navigation as motion coverage updates complete.
- Newly revealed distant or small projected regions may remain downsampled/compensated until idle refinement catches up.
- Stopping movement should no longer flash to a lower-detail whole-cloud fallback before refining.
- Sparse source regions keep literal user point size, so real source gaps remain visible.
- Dense overlapping source regions may use a small capped visible-size increase, but only when projected source spacing indicates the full source would already overlap.
- Idle refinement resumes after the camera settles, restoring high-quality adaptive detail.

## Acceptance Notes

Use Constant Update View and Live Visual Effects while testing. During rotation, the target is 30fps with 20fps as the floor. Diagnostics should show motion hold/reuse, async pending age, motion apply holds, motion coverage patches, demotion blocks, skipped patches, draw-item bytes, upload time, requested/displayed density, and max adaptive render diameter.
