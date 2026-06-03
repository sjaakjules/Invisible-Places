# Water Features v2 Acceptance Log

Date: 2026-05-26

Scope: local repository verification for the Water Features v2 migration described in
`docs/water_features_redesign_v2.md`.

## Accepted

- Active Water workflow is Ripples, Flow, and Field.
  Evidence: `DrawWaterPanel` exposes `BeginTabItem("Ripples")`, `BeginTabItem("Flow")`, and `BeginTabItem("Field")`; Basin, Runoff, and Caustics tab items are no longer active in the Water tab bar.

- Basin and Runoff are absent from active UI and new saves.
  Evidence: project and water-source serializers omit `water_basin_regions`, `water_runoff_regions`, and `water_caustic_regions`; round-trip tests verify those keys are absent even when legacy region vectors are populated.

- Legacy Caustics records migrate to Ripple layers.
  Evidence: `DrawWaterRipplesPanel` creates `WaterEffectLayer` records, and test `Legacy water region records load as v2 ripples while basin and runoff are ignored` proves old `water_caustic_regions` become Ripple `Caustic Lace` layers.

- Ripple overlay styles are distinct at the generated/effect-field level.
  Evidence: `GenerateRippleEffectOverlay` dispatches `WaterRippleOverlayType` through type-specific sparse effect-field patterns for Caustic Lace, Linear Ripples, Radial Ripples, Rain Rings, Shoreline, Wet Sheen, Current Threads, Droplet Glints, Drip Trails, Foam Sparkle, and Salt/Mineral Shimmer. Test `Water ripple overlay types generate distinct procedural effect fields` verifies every overlay type emits a non-empty distinct field, with targeted checks for linear bands, radial symmetry, and wet-sheen slope response.

- Ripple concave clicked boundaries are preserved.
  Evidence: `GenerateRippleEffectOverlay` evaluates containment and edge distance against `BuildWaterRegionBoundary(layer.vertices)` rather than the derived hull. Test `Ripple effect generation preserves concave clicked region boundaries` verifies a C-shaped Ripple region includes points in the left bar and bottom arm while excluding a point in the cut-out.

- Flow path cache reuse and branch hiding remain preserved.
  Evidence: `TryLoadWaterPathCacheForSupport`, `CurrentWaterPathCacheForDocument`, and `BuildWaterPathAnchorsFromCache` keep cache reuse and hidden branch IDs; tests `Water path cache tags bridge gaps and round-trips hidden branches`, `Water path smoothing refreshes cached branch anchors without rebaking`, and `Water trail surface index is reusable for preview and final builds` pass.

- Generated Flow/Field stream overlays expose the v2 scalar contract.
  Evidence: `BuildFlowStreamOverlayFromPathAnchors`, `BuildFieldStreamOverlay`, and `BuildWaterStreamOverlayPointCloud` produce deterministic stream surfels and scalar fields in the expected order; test `Water v2 streams expose deterministic scalar contracts` covers this.

- Reload project safely.
  Evidence: schema `24` saves Ripple/Flow/Field settings plus `water_path_cache`; generated/effect water sessions are skipped from normal project layer persistence, so reload uses v2 settings and cached paths instead of requiring generated PLY files; round-trip and legacy-load tests pass.

- Shader/style handling covers world-aligned stream surfels.
  Evidence: `waterStreamOverlay` style state is serialized, forces the unified shader path, and feeds stream width, world length, tangent, confidence, and wetness fields to viewport/offline rendering. Test `Offline water stream overlays use stream tangent and world length` verifies tangent-oriented world-length rendering.

- Flow Trail motion is time-driven without topology regeneration.
  Evidence: viewport/raycast/surfel shaders and offline rendering derive animated stream age from `point_age`, `point_seed`, `stream_speed`, and render time, then modulate opacity/emission/colour energy. Test `Offline water stream overlays animate through time playback` renders the same stream topology at two times and verifies the alpha changes.

- Field can build from user-defined regions with concave containment.
  Evidence: Field regions are persisted as `WaterEffectLayer` records with feature type `FieldSurfaceMotion`; `RefreshWaterFieldOverlays` builds from matching loaded Field region layers before falling back to Flow path anchors; `BuildFieldCacheFromRegions` samples the target cloud inside `BuildWaterRegionBoundary(layer.vertices)` and `BuildFieldStreamOverlay` clips generated stream samples to the authored boundary. Test `Field cache builds from concave selected regions` verifies a C-shaped Field region excludes the cut-out for cache nodes, stream samples, and Field Surface Motion effect points. Project and water-source round-trip tests verify `water_field_layers` persists.

- Field streamlines reject over-limit gaps, fade low-confidence support, and report bridge diagnostics.
  Evidence: `BuildFieldStreamOverlay` splits Field paths when adjacent cache nodes exceed the configured bridge distance and applies `surfaceConfidenceThreshold` / `fadeOnLowConfidence` to `stream_confidence` and wetness. `WaterFieldStreamDiagnostics` reports input nodes, emitted paths/samples, accepted bridges, rejected gaps, low-confidence fades, and hard terminations; the Field panel exposes Bridge Limit, Bridge Aggression, Confidence Fade, Fade Low Confidence, and the latest diagnostics. Test `Field streamlines split rejected gaps and fade low-confidence support` verifies no generated stream samples cross an over-limit gap, low-confidence support produces faded stream confidence, and the diagnostic counters reflect accepted/rejected/faded behavior.

- Field no-flow and manual bridge control regions are implemented.
  Evidence: Field layers can be created as Surface Motion, No Flow, Bridge Allowed, or Bridge Blocked. `BuildFieldCacheFromRegions` applies Field control-region boundaries to local nodes, `BuildFieldStreamOverlay` skips no-flow support, allows bounded manual bridge gaps, rejects bridge-blocked gaps, and records no-flow / bridge-allowed / bridge-blocked diagnostic counters shown in the Field panel. Tests `Field cache builds from concave selected regions`, `Field streamlines split rejected gaps and fade low-confidence support`, and `Project document round-trips binding-backed point-cloud styles` verify control-region masking, manual bridge behavior, and feature-type persistence.

- Ripple and Field Surface Motion contribute to active/base cloud visual evaluation.
  Evidence: `ComposeWaterEffectFields` pre-composes Ripple and Field `WaterEffectOverlay` points into `water_effect_*` fields on the target cloud; viewport, surfel, raycast, fast-basic, and offline render paths apply those fields after the normal base point size, opacity, emission, and colour mappings. Test `Ripple and Field effects compose onto base cloud visual evaluation` verifies Height and Intensity mappings remain active while Ripple and Field contributions alter final rendered alpha and colour, writes the water-effect render through the EXR writer, and converts the same water-effect frame through the MP4 preview conversion path. The Visuals tab exposes a Water Effect Stack for the selected base cloud with blend, intensity, emission, opacity, size, and colourise contribution controls, and `Field cache builds from concave selected regions` verifies Field Surface Motion carries the selected region contribution values.

- Viewport, EXR, and MP4 export paths include water output without requiring water PLY export.
  Evidence: generated/effect water overlays are ordinary visible point-cloud sessions created from in-memory `LoadedPointCloud` data; project save skips them as source layers. Active-cloud `water_effect_*` fields are auto-regenerated after saved Ripple/Field-region reload and are consumed by viewport, fast-basic, raycast, and offline render paths. Offline export coverage is exercised by `Offline water stream overlays use stream tangent and world length`, `Offline ripple effect overlays render from virtual effect fields`, and `Ripple and Field effects compose onto base cloud visual evaluation`, including EXR writer and MP4 frame conversion checks for active-cloud water effects.

## Operator Notes

- Generated/effect point-cloud sessions still exist as compatibility preview/export layers for Ripple and Field Surface Motion. The accepted path is active-cloud `water_effect_*` composition; the compatibility layers are intentionally excluded from support-layer discovery and normal project layer persistence.

- Manual app spot-checks on real site data remain useful for tuning Field bridge/no-flow thresholds and confirming the visual taste of the presets, but the local acceptance requirements above are covered by automated persistence, generation, shader/offline-render, EXR writer, and MP4 frame-conversion tests.

## Progress Log

- 2026-05-26 | Checkpoint: Concave Regions | Proof: `ctest --test-dir build/macos-debug -R "Ripple effect generation preserves concave|Water ripple overlay types|Offline ripple" --output-on-failure` | Status: accepted | Notes: Ripple generation uses the clicked boundary and the C-shaped cut-out test passes; Field region proof was added in the later Field Regions checkpoint.
- 2026-05-26 | Checkpoint: Flow Motion | Proof: `ctest --test-dir build/macos-debug -R "Ripple effect generation preserves concave|Water ripple overlay types|Offline ripple|Offline water stream overlays" --output-on-failure` | Status: accepted | Notes: Static stream topology renders differently at two times through stream age playback.
- 2026-05-26 | Checkpoint: Field Regions | Proof: `ctest --test-dir build/macos-debug -R "Field cache builds from concave|Project document round-trips|Water source documents|Water v2 streams" --output-on-failure` | Status: accepted | Notes: Field builds from persisted user regions and the C-shaped selected-region test excludes the cut-out.
- 2026-05-26 | Checkpoint: Surface Safety | Proof: `ctest --test-dir build/macos-debug -R "Field streamlines split|Field cache builds from concave|Water v2 streams" --output-on-failure` | Status: accepted | Notes: Over-limit Field gaps are rejected, low-confidence support fades, and diagnostic counters are exposed.
- 2026-05-26 | Checkpoint: Effect Composition | Proof: `ctest --test-dir build/macos-debug -R "Ripple and Field effects compose|Water ripple overlay types|Ripple effect generation preserves concave|Field cache builds from concave|Offline ripple|Offline water stream overlays" --output-on-failure` | Status: accepted | Notes: Ripple and Field contributions compose onto active/base cloud render evaluation while preserving base Height/Intensity mappings and Visuals contribution controls.
- 2026-05-26 | Checkpoint: Reloaded Effect Fields | Proof: `ctest --test-dir build/macos-debug -R "Project document round-trips|Legacy water region records|Ripple and Field effects compose|Water ripple overlay types|Ripple effect generation preserves concave|Field cache builds from concave|Offline ripple|Offline water stream overlays" --output-on-failure` | Status: accepted | Notes: Saved Ripple/Field-region outputs regenerate active-cloud `water_effect_*` fields after project load or background layer activation.
- 2026-05-26 | Checkpoint: Effect Contribution Controls | Proof: `ctest --test-dir build/macos-debug -R "Field cache builds from concave|Ripple and Field effects compose|Water ripple overlay types|Project document round-trips|Legacy water region records|Offline ripple|Offline water stream overlays" --output-on-failure` | Status: accepted | Notes: The Visuals tab exposes add/max/multiply/screen/override plus emission, opacity, size, and colourise contributions for selected base-cloud Ripple/Field stacks, and Field Surface Motion carries those region response values through the cache.
- 2026-05-26 | Checkpoint: Field Bridge Diagnostics | Proof: `ctest --test-dir build/macos-debug -R "Field streamlines split|Field cache builds from concave|Ripple and Field effects compose|Water ripple overlay types|Project document round-trips|Offline water stream overlays" --output-on-failure` | Status: accepted | Notes: Field Streamlines expose accepted/rejected bridge, low-confidence fade, termination counters, Bridge Aggression UI, and manual Field control-region counters.
- 2026-05-26 | Checkpoint: Water Effect Export Path | Proof: `ctest --test-dir build/macos-debug -R "Ripple and Field effects compose" --output-on-failure` | Status: accepted | Notes: Active-cloud Ripple/Field composition now writes through the EXR writer and survives MP4 preview frame conversion while preserving base Height/Intensity mappings.
- 2026-05-26 | Checkpoint: Field No-Flow Controls | Proof: `ctest --test-dir build/macos-debug -R "Project document round-trips|Field streamlines split|Field cache builds from concave|Ripple and Field effects compose" --output-on-failure` | Status: accepted | Notes: Field no-flow, bridge-allowed, and bridge-blocked control regions persist, alter cache/stream behavior, and report diagnostics.

## Verification Commands

```text
cmake --build --preset build-macos-debug-home-vcpkg
ninja: no work to do.
```

```text
cmake --build build/macos-debug
[1/1] Linking CXX executable invisible_places.app/Contents/MacOS/invisible_places
INFO | macdeployqtfix terminated with success
```

```text
ctest --test-dir build/macos-debug --output-on-failure
100% tests passed, 0 tests failed out of 127
```

Focused checks also passed:

```text
ctest --test-dir build/macos-debug -R "Water v2|Water ripple|Offline ripple|Offline water stream|Legacy water region|Project document round-trips|Point material variant" --output-on-failure
100% tests passed, 0 tests failed out of 8
```

```text
ctest --test-dir build/macos-debug -R "Project document round-trips|Field streamlines split|Field cache builds from concave|Ripple and Field effects compose" --output-on-failure
100% tests passed, 0 tests failed out of 4
```

```text
ctest --test-dir build/macos-debug -R "Water v2|Water ripple|Field streamlines split|Field cache builds from concave|Ripple and Field effects compose|Offline ripple|Offline water stream|Legacy water region|Project document round-trips|Water source documents|Water gap tolerance|Water path cache" --output-on-failure
100% tests passed, 0 tests failed out of 14
```

```text
git diff --check
no output
```
