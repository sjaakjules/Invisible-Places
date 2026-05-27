# lod_goal_stage04.md — CPU Selector Quality: Node Stats and Representative Classes

## Codex goal

```text
/goal Upgrade the CPU adaptive LOD selector and hierarchy data so Fast Basic and later Beauty modes preserve visually important detail, verified by tests and lod-compare/manual runs showing that rare colour, scalar, normal/edge, and emissive/accent features are not erased while Stage 03 smoothness and Stage 02 boundedness remain intact. Use CPU traversal first; do not move selection to GPU compute in this stage. Between iterations, add one measurable statistic or representative class, verify it affects selection correctly, and rerun tests/compare outputs. If blocked by cache format compatibility, version the cache safely and report migration behaviour.
```

## Required prior stage

Stage 03 must be complete. Fast Basic must already be bounded and visually smooth with the existing representatives.

## Why this stage exists

Smooth LOD can still be wrong if the chosen representatives erase important visual information. The target design calls for node spacing, density, variance, feature hints, and representative classes. This stage makes the CPU selector match the visual-cost model before Beauty compensation and before any GPU-driven selector.

## Scope for this stage

In scope:

- Hierarchy node statistics.
- Representative classes.
- Stable rank improvements.
- CPU traversal/refinement decisions.
- Cache versioning/invalidation for new data.
- Tests and `--lod-compare` improvements if needed for quality evidence.

Out of scope:

- Beauty opacity/emission compensation.
- GPU timestamp governor.
- `.ipcloud` raw chunk streaming.
- GPU compute traversal.
- Tile overdraw and occlusion.

## Implementation tasks

1. Inspect current `PointCloudLodNode` and `PointCloudLodRepresentative` data.
2. Add or persist node statistics needed by selection:
   - `spacingMeters`
   - `densityPointsPerM3`
   - represented source count
   - colour variance or colour contrast hint
   - normal variance or normal cone where normals exist
   - scalar variance / min / max / threshold hints where scalar fields exist
   - emissive/accent importance hint if style data can identify it
3. Add representative classes or equivalent flags:
   - spatial coverage
   - colour contrast
   - normal/edge
   - scalar min
   - scalar max
   - scalar threshold
   - emissive/accent
   - blue-noise/random fill
4. Ensure each representative still preserves `sourcePointIndex` so existing source position, colour, normal, scalar, water-field, and material lookups remain correct.
5. Improve rank generation so every prefix is spatially useful and deterministic.
6. Update CPU traversal refinement rules. Refine or preserve extra representatives when:
   - projected spacing is above target
   - projected mark radius/spacing would create holes
   - colour variance is visible
   - scalar min/max crosses a visible threshold or colour-map boundary
   - normal/edge variation is high
   - emissive/accent importance is high
   - node is close, on silhouette/depth discontinuity, or actively inspected
7. Separate at least the first-pass cost rules for:
   - Fast Basic square points
   - Beauty screen sprites
   - Beauty world surfels
   Beauty may not be fully implemented until Stage 05, but the selector should stop treating all geometry modes as identical.
8. Version the current hierarchy cache if new fields alter binary layout or meaning.
9. Add tests for:
   - stats calculation on synthetic point clouds
   - representative class presence for scalar extremes/colour contrast/normal variation
   - source index correctness through representative selection
   - cache invalidation/versioning
   - deterministic traversal output
10. Extend `--lod-compare` or diagnostics if current coverage/luminance ratios are insufficient to catch erased rare features.

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
./build/macos-debug/invisible_places --lod-compare <cloud>
```

Inspect:

```text
Saved/diagnostics/lod_compare/lod_compare_metrics.json
```

Expected automated evidence:

- Representative classes appear in diagnostics or test output.
- Adaptive HQ coverage/luminance remain in an acceptable band for the active style.
- Synthetic tests prove rare scalar/colour/normal/emissive features survive LOD.
- Cache invalidation works cleanly when the hierarchy format changes.

## Manual runtime checkpoint for the user

Run Fast Basic and, if available, Beauty Adaptive on clouds/styles containing:

- strong colour contrast
- scalar/water styling
- normals or edge/surface features
- rare bright/emissive/accent points
- dense far detail and close inspection areas

Expected behaviour:

- Rare accents and thresholds remain visible instead of being averaged away.
- Smoothness from Stage 03 remains acceptable.
- Fast Basic remains responsive and bounded.
- Diagnostics explain why high-variance nodes refine earlier than low-variance nodes.

## Completion condition

This stage is complete only when:

- CPU selection uses visual statistics, not just distance or raw point count.
- Representative classes exist and are test-covered.
- Source attributes remain correct through `sourcePointIndex`.
- Cache versioning is safe.
- The user can see that important features survive LOD selection.

## Stop conditions

Stop and report if:

- Current source data does not expose required scalar/normal/emissive information.
- Binary cache compatibility requires a migration decision from the user.
- Quality cannot be validated because no suitable test cloud/style is available.

## Handoff to Stage 05

Pass along:

- Final representative/draw-item fields available for Beauty compensation.
- Any feature classes that Beauty should treat specially.
- Updated `--lod-compare` metrics and known limitations.
