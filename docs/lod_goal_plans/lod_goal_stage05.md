# lod_goal_stage05.md — Beauty Adaptive Visual Parity and Compensation

## Codex goal

```text
/goal Implement Beauty Adaptive LOD compensation and renderer-specific cost handling so screen sprites and world surfels preserve the user's intended look while using adaptive representatives, verified by build/tests, lod-compare outputs, and manual Beauty runs with no obvious opacity, emission, footprint, scalar, normal, or detail-loading artifacts. Preserve Fast Basic behaviour from Stages 02–04 and keep Full Source modes exact/debug-only. Between iterations, fix one measurable Beauty mismatch, rerun compare/manual checks, and record the evidence. If blocked by shader ABI or unavailable styles, stop with exact missing inputs.
```

## Required prior stage

Stage 04 must be complete. The CPU selector must provide usable representative stats/classes and source-correct draw items.

## Why this stage exists

Beauty rendering is not just “more points.” It includes Gaussian/screen sprites, world surfels, opacity, emission, falloff, scalar styling, depth contribution, EDL/composite, and AOV/export expectations. The target design requires LOD compensation so representatives preserve the user's canonical style instead of mutating UI settings or becoming too transparent/opaque/bright.

## Scope for this stage

In scope:

- Beauty Adaptive screen-sprite LOD draw path.
- Beauty Adaptive world-surfel LOD draw path if surfels exist in the project.
- Draw-item fields for footprint, opacity, emission, radius, blend, importance, representative class, and random seed.
- Optical-depth opacity compensation.
- Emission compensation policies.
- Separate cost models for screen sprites and world surfels.
- Beauty diagnostics and `--lod-compare` expansion where needed.

Out of scope:

- `.ipcloud` streaming.
- Vulkan timestamp governor unless minimal timings already exist.
- Tile overdraw budgets beyond simple estimated fragment/blended fragment caps.
- GPU compute selection.
- Full transparent sorting of all splats.

## Implementation tasks

1. Inspect current Beauty Adaptive shader(s), surfel/sprite geometry paths, draw-item buffer layout, descriptor binding 7 usage, and AOV/source attribute lookups.
2. Add or verify LOD-aware Beauty shader variants.
   - Screen sprites must fetch `sourcePointIndex` from draw items.
   - Surfels must map six vertices to one draw item if expanded as quads.
   - Raw Full Source shaders should remain available for exact/debug modes.
3. Extend `PointCloudDrawItemGpu` or CPU-side draw-item source to carry:
   - `representedCount`
   - `spacingMeters`
   - `rawFootprintArea`
   - `lodFootprintArea`
   - `opacityCoverageScale`
   - `emissionCoverageScale`
   - `radiusScale`
   - `lodBlend`
   - `importance`
   - `randomSeed`
   - `representativeClass`
   - `flags`
4. Implement footprint policies without mutating user UI settings:
   - `LiteralUserSize`
   - `DensityFill`
   - `PerformanceClamped`
5. Implement opacity compensation using optical-depth style logic:
   - Scale by represented source count and footprint area.
   - Avoid linear opacity explosions.
   - Clamp where needed for preview safety.
6. Implement emission compensation policies:
   - preserve average brightness
   - preserve integrated energy where requested
   - preserve accents using representative classes
   - clamp for preview when explicitly performance-limited
7. Implement separate cost estimation for:
   - Beauty screen sprites: fragment/blended fragment cost from radius in pixels.
   - Beauty world surfels: six vertices per representative plus projected world radius.
8. Make Beauty Adaptive honour the Stage 02 budget propagation and Stage 03/04 transition stability.
9. Verify all source-derived attributes remain correct:
   - positions
   - colours
   - normals
   - scalar fields
   - water fields/effects
   - material bindings, if used
   - depth/AOV outputs
10. Add diagnostics for:
   - screen sprite vs surfel mode
   - radius scale range
   - opacity coverage scale range
   - emission scale range
   - estimated fragments and blended fragments
   - Beauty fallback status
   - compensation policy/mode

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Add tests where practical:

- One representative standing for many points does not simply multiply opacity linearly.
- Changing representative footprint changes coverage scale in the expected direction.
- Emissive/accent representatives are preserved or scaled according to policy.
- Screen-sprite and surfel cost estimates differ for the same representative count.
- LOD-aware Beauty shaders use `sourcePointIndex` rather than assuming `gl_VertexIndex` is the source point.

## Manual runtime checkpoint for the user

Run Beauty Adaptive and compare against Beauty Full Source on representative styles:

1. Small opaque or mostly opaque points.
2. Large translucent Gaussian/screen sprites.
3. Emissive/glow accents.
4. Scalar/water style.
5. World surfels if available.
6. Long dense/grazing view.
7. Slow motion and stop-to-refine.

Expected behaviour:

- Adaptive Beauty does not look obviously too transparent, too opaque, or over-bright.
- Rare emissive/accent features do not disappear.
- Scalar/water/normal/source-colour lookups remain source-correct.
- No sudden detail loading beyond Stage 03 accepted smoothness.
- If preview quality is clamped for performance, the UI/diagnostics say so explicitly.

## Completion condition

This stage is complete only when:

- Beauty Adaptive has renderer-specific LOD compensation.
- Fast Basic behaviour remains stable.
- Beauty Full Source remains exact/debug and is never a silent adaptive fallback.
- Automated tests cover compensation math or shader indexing where practical.
- Manual comparison with Beauty Full Source is acceptable for the target styles.

## Stop conditions

Stop and report if:

- Shader ABI changes require a broader descriptor/pipeline migration.
- No Beauty test styles or reference clouds are available.
- `--lod-compare` cannot render Beauty modes and needs a separate user decision.

## Handoff to Stage 06

Pass along:

- Which renderer/geometry modes are most expensive.
- Current estimated fragment/blended fragment budgets.
- Any cases where FPS/present heuristics disagree with visible point-pass cost.
