# Water Flow Path Analysis And Dynamic Lanes

## Purpose

The current Flow lane system follows baked path anchors with mostly fixed lateral offsets. That is useful for simple directional flow, but it cannot explain or produce broad flat-sheet regions, local narrowing, eddies, wavy flow, or ripple-prone side-to-side movement from the terrain and nearby paths.

The new pipeline separates water route discovery from flow behavior:

```text
Path bake -> Path analysis -> Lanes -> Trail
```

- **Path bake** finds supported downhill water routes.
- **Path analysis** understands local terrain and path behavior once per path cache.
- **Lanes** distribute flow through the analysed corridor.
- **Trail** decides how the resulting flow looks.

The result should be an artist-directed terrain/path-field model, not a full shallow-water solver. It should be fast to edit, understandable in Path View, and deterministic enough for tests and export.

## Pipeline Responsibilities

### Path Bake

Path bake remains responsible for finding supported downhill routes from water sources. It is the only stage that should call `GenerateWaterPathCache`.

Inputs:

- support point cloud,
- water source positions and per-source Path profile,
- Path settings such as reach, branching, coverage, gap tolerance, sample spacing, support voxel size, bridge limit, and smoothing.

Outputs:

- `WaterPathCache`,
- visible path anchors,
- hidden branch state,
- diagnostics about support spacing, bridge distance, branches, confidence, and gaps.

Path bake must not know about Trail colour, Trail opacity, Trail emission, or display-only Lane parameters. Path setting/source changes dirty path bake and invalidate downstream Path Analysis, Lanes, and Trail samples.

### Path Analysis

Path analysis runs after path bake or cache load. It computes behavior values for each path sample and caches them with or beside the path cache. Lanes should sample this analysis instead of recomputing terrain/path-neighbor facts per generated trail.

Path analysis computes:

- `slope`: local path steepness from smoothed path-distance elevation changes.
- `flatness`: smoothed inverse slope.
- `curvature`: signed side-to-side direction change along the branch.
- `neighborDensity`: count/weight of nearby sibling path samples within an analysis radius.
- `nearestPathDistance`: spacing to the nearest sibling path/branch sample.
- `confluence`: where paths merge, run close together, or remain close downstream.
- `channelWidth`: inferred local flow width from path spacing, flatness, confluence, and isolation.
- `speed`: derived from slope, reduced on flats.
- `turbulence`: derived from speed, curvature, roughness/confidence/gaps, and confluence.
- `eddyPotential`: high near bends, slope transitions, roughness, and fast non-flat flow.
- `ripplePotential`: high where flow direction wiggles, crosses laterally, or changes side-to-side.

All analysis values used as public diagnostics should be normalized to `0..1` except distance/width/speed values that are explicitly in meters or meters per second.

### Lanes

Lanes generate animated flow routes by advecting through a cached analysed corridor, not by fixed global offsets from a path.

Lane behavior:

- close paths at the top merge into a wider flow region through high `neighborDensity` and `confluence`,
- isolated paths stay thinner through high `nearestPathDistance` and low confluence,
- flat areas spread wider and slow down through high `flatness`,
- steep channels narrow and speed up through high `slope`,
- bends and rough support create eddy primitives or local curl fields through `eddyPotential`,
- wavy regions get stronger lateral curl/noise through `curvature` and `ripplePotential`,
- ripples appear where curvature, lateral crossing, and side-to-side movement are high.

Lane generation should produce routes and scalar/style metadata that the existing Flow Trail renderer can consume. Existing internal `stream_*` scalar names remain compatible until renderer/offline consumers are deliberately migrated.

### Trail

Trail is the visual layer. It controls point/sprite styling and visible sample geometry:

- colour,
- opacity,
- emission,
- width,
- full trail length,
- point spacing,
- streak/world sprite length.

Trail style edits must not recompute Path Analysis, Lanes, or Path bake. Trail geometry edits may resample trail surfels from cached Lane routes, but must not recompute Lane route analysis or paths.

## Path Analysis Algorithm

### Inputs

- `WaterPathCache` branches and raw anchors.
- Smoothed visible path anchors generated from the resolved Path profile.
- Tuned path settings and support diagnostics.
- Optional support-cloud metadata already available from path bake, such as normals, confidence, gap counts, and estimated spacing.

### Per-Branch Preparation

For each visible branch:

1. Ensure anchors are ordered by `pathDistance`.
2. Compute cumulative distance if missing or stale.
3. Compute tangent at each sample from a symmetric distance window.
4. Smooth tangent, elevation, slope, and curvature over a meter-scale window derived from path sample spacing.
5. Preserve branch id, emitter id, role, confidence, gap count, and hidden state.

### Slope And Flatness

Slope is based on height change over path distance:

```text
slope = clamp(abs(deltaZ) / max(windowDistance, epsilon), 0, slopeHigh) / slopeHigh
flatness = smooth(1 - slope)
```

Use a symmetric window where possible. At branch ends, use the nearest valid forward/backward window. Smooth the final values so a single noisy support sample does not create a fake rapid-water region.

### Curvature

Curvature is derived from direction change along the path:

```text
curvature = clamp(abs(signedTurnAngle) / curvatureHigh, 0, 1)
```

Signed curvature should be retained internally for eddy orientation. Public diagnostic colour can use absolute curvature.

### Neighbor Density And Nearest Path Distance

Build a spatial index over analysed path samples. For each sample:

1. Query sibling samples within `analysisRadius`.
2. Ignore samples from the same branch that are close in path distance to avoid counting the local path against itself.
3. Weight neighbors by distance, downstream-progress similarity, and branch role.
4. Record nearest sibling distance in meters.
5. Convert weighted neighbor count to `neighborDensity`.

This makes close top paths widen the flow corridor without doing expensive neighbor searches per trail.

### Confluence

Confluence is high when nearby branches remain close over distance or merge downstream. For each sample:

1. Use neighbor samples within the analysis radius.
2. Increase score when neighbors have similar downstream direction.
3. Increase score when branches approach the same downstream area or share/approach a parent branch.
4. Smooth confluence along branch distance.

Confluence should be broader and more persistent than raw neighbor density, because the visual effect is a larger water region rather than a one-sample spike.

### Channel Width

Channel width is a local meter value:

```text
baseWidth = max(trailWidth * laneCountHint, minimumWidth)
flatSpread = mix(1.0, flatWidthMultiplier, flatness)
confluenceSpread = mix(1.0, confluenceWidthMultiplier, confluence)
isolationNarrow = mix(isolatedWidthMultiplier, 1.0, neighborDensity)
spacingWidth = clamp(nearestPathDistance * spacingWidthFactor, minWidth, maxWidth)
channelWidth = clamp(max(baseWidth, spacingWidth) * flatSpread * confluenceSpread * isolationNarrow, minWidth, maxWidth)
```

The exact constants should be settings-driven later, but the first implementation should choose stable defaults and expose only the highest-value controls.

### Speed

Speed is a local multiplier or meters-per-second value:

```text
slopeSpeed = mix(flatSpeed, steepSpeed, slope)
spreadDamping = mix(1.0, flatDamping, flatness)
speed = baseLaneSpeed * slopeSpeed * spreadDamping
```

Flat regions should visibly slow down. Steeper exit regions should move faster.

### Turbulence

Turbulence is an artistic proxy motivated by faster flow, bends, roughness, and path convergence:

```text
turbulence = saturate(
    speedWeight * normalizedSpeed +
    curvatureWeight * curvature +
    roughnessWeight * roughnessOrLowConfidence +
    confluenceWeight * confluence)
```

This is not a physical Reynolds-number computation. It uses the same intuition: faster, larger, less-viscous-looking flow reads as more turbulent.

### Eddy Potential

Eddy potential rises around:

- bends,
- slope transitions,
- rough or low-confidence support,
- fast non-flat regions,
- local widening/narrowing transitions,
- side pockets near the analysed corridor.

Use signed curvature to orient eddies. Eddies should be local vortex/curl influences in the Lane stage, not separate path bake branches.

### Ripple Potential

Ripple potential rises when motion has strong side-to-side character:

- high curvature,
- high lateral oscillation,
- lane crossing,
- wavy flow,
- eddy edges,
- shallow flat regions with visible flow.

This value can feed both Trail scalar fields and future Ripple/Flow visual coupling.

## Dynamic Lane Algorithm

### Route Seeding

Distribute lane route seeds across analysed path distance and local width:

1. Allocate route count by branch length, channel width, and confluence.
2. Use deterministic seeds from branch id, emitter id, route id, and Lane profile seed.
3. Start routes inside the local `channelWidth` envelope.
4. Prefer broader seed spread in high-confluence and high-flatness regions.

### Field Advection

Each route advances through a local vector field:

```text
direction =
    pathTangent * pathAttraction +
    downhillDirection * downhillWeight +
    exitDirection * exitWeight +
    lateralWave * ripplePotential +
    curlNoise * turbulence +
    vortexField * eddyPotential
```

Then project the route back to the support surface or nearest analysed corridor sample.

Path attraction should be high in steep/narrow regions and lower in flat/open regions. This lets flow spread in flat basins but still lead toward the exit.

### Width And Lane Spacing

At every route step:

- sample nearest analysed path point,
- read `channelWidth`,
- compute local lateral bounds,
- keep routes inside the envelope with a soft boundary force,
- allow mild crossing where `turbulence`, `confluence`, or `ripplePotential` are high.

Lane count becomes a density target rather than a fixed set of parallel lines. Close paths can merge into a sheet-like region where individual lanes are less visually obvious.

### Eddies

Create deterministic eddy primitives from analysed path samples where `eddyPotential` exceeds a threshold:

- center near bend/roughness/slope-transition sample,
- radius proportional to `channelWidth`,
- rotation sign from signed curvature and seed,
- strength from `eddyPotential * turbulence`,
- lifetime/phase from route seed.

Routes entering an eddy radius receive a tangential velocity component plus a weak downstream pull so they swirl but do not get trapped forever.

### Wavy Flow And Ripples

Use low-frequency lateral curl/noise in high-flatness and high-ripple-potential regions. This should create broad, wavy sheet flow like the flat area in the reference screenshot.

Ripple scalar hints should be emitted where route curvature or lateral oscillation is high. The first implementation may expose these as scalar fields and diagnostic colours before coupling them to Ripple overlays.

## Recalculation And Performance Contract

The implementation must preserve this invalidation model:

- Path setting/source changes: recompute Path bake, Path Analysis, Lane routes, and Trail samples.
- Path-analysis setting changes: recompute Path Analysis, Lane routes, and Trail samples only.
- Lane topology changes: recompute Lane routes and Trail samples only.
- Lane param-only changes: update scalar/style data only where possible.
- Trail style changes: update style only.
- Trail geometry changes: resample Trail surfels from cached Lane routes only.

UI slider drags must avoid rebuilding and uploading the full generated trail cloud for every intermediate value. Use one or more of:

- apply-on-release for topology settings,
- debounce delayed refresh,
- preview-quality cap while dragging,
- explicit `Regenerate Lanes` for expensive changes,
- scalar-only updates for speed/display-only changes.

## Path View Diagnostics

Path View should include a diagnostic colour mode dropdown:

```text
Branch
Slope
Flatness
Curvature
Neighbor Density
Confluence
Channel Width
Speed
Turbulence
Eddies
Ripples
```

These colours must use the same cached analysis values consumed by Lanes. The goal is to make the generated flow explainable: users should be able to see why water widens, narrows, speeds up, slows down, curls, or ripples.

## Validation Fixture

Use `tests/water_flow_path_analysis_site3_fixture.json` as the reference dataset and source. It targets:

- `Data/Site3-Sample-Terrestrial.ply`
- source position `(307.709, 102.395, 2.007)`
- Detail-scale path settings captured from the reference screenshot.

The fixture should support both automated tests and future GUI smoke scenarios.

## Implementation Notes

- Keep the first implementation deterministic and CPU-side.
- Keep `stream_*` scalar names stable for renderer/offline compatibility.
- Prefer compact cached analysis arrays over per-trail repeated analysis.
- Add settings only when needed for art direction; start with strong defaults.
- Treat full shallow-water simulation as out of scope for this phase.
