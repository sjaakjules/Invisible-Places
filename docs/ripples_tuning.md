# GPU-First Ripple Tuning

## Purpose

This document defines the required end states for the Ripple overlay patterns that need tuning. The live viewport implementation is GPU-first: after a region membership has been built, changing overlay settings must update compact sparse Ripple parameters only. Settings edits must not recalculate per-point CPU pattern values or rebuild selected point membership.

CPU-side pattern code exists to mirror shader behavior for offline renders, tests, and contact-sheet generation. The live visual source of truth is `shaders/pointcloud_sparse_ripple.glsl`; matching logic in `src/water/WaterFlow.cpp` is a parity path, not the interactive path.

## Performance Contract

The latency thresholds below apply to editing pattern, response, blend, colour, opacity, size, emission, speed, phase, density, direction, warp, and turbulence after region membership already exists.

- Ideal: less than 1 ms.
- Desired: less than 10 ms.
- Firm limit: less than 100 ms, only when lower latency is genuinely unavoidable.
- Hard boundary: less than 1 second. At or above 1 second blocks completion.

The following actions may rebuild CPU membership because they change topology or selected support points:

- Editing region vertices.
- Changing target base cloud.
- Changing layer enablement in a way that adds/removes active regions.
- Changing seed or max affected points.
- Rebuilding a missing or stale region preview.

Validation must prove that ordinary settings edits keep sparse membership revisions stable and advance only sparse params revisions.

## Shared Implementation Rules

- Keep most visual math in `shaders/pointcloud_sparse_ripple.glsl`.
- Keep `src/water/WaterFlow.cpp` pattern functions numerically close enough to GLSL for contact sheets and offline renders.
- Avoid adding dense `water_effect_*` or `ripple_*` fields for ordinary Ripple updates.
- Preserve Linear Ripples and Radial Ripples unless a shared helper must change.
- Use deterministic procedural seeds so overlapping regions with the same seed and matching droplet amount can line up.
- Use the reference images in `tests/Ripple Effect Reference Images` as visual targets, not pixel-match fixtures.
- Use `tests/Test_Points.txt` on `Data/Site3-Sample-Terrestrial.ply` as the main visual validation region.

## Pattern End States

### Caustic Lace

Caustic Lace should look like sunlight refracted through a moving water surface. It should produce thin, bright, cellular lace ridges with organic curvature and mild shimmer. The pattern should move over time. At region boundaries, the pattern should not fade out; it should continue until region membership clips it.

Validation:

- Temporal samples differ meaningfully.
- Bright ridges are sparse relative to the selected region.
- Edge-adjacent selected points can still be bright when the caustic ridge passes through them.
- Contact-sheet tiles resemble the caustic reference images.

### Rain Rings

Rain Rings should look like many water drops hitting a lake surface. Each droplet creates expanding concentric rings, with a few close rings near each impact and interference when multiple droplets overlap.

Controls:

- `Ring Scale` controls ring reach and spacing before fade.
- `Rain Amount` controls active droplet count.
- `Expansion Speed` controls outward motion.
- `Ring Jitter` and `Ring Breakup` make the rings imperfect without turning them into noise.

Validation:

- Ring centers are deterministic for a given seed.
- Increasing density increases active droplet origins without making the region uniformly noisy.
- Increasing wavelength increases ring reach/spacing.
- Time samples show expanding rings rather than static noise.

### Shoreline

`Tide Bands` is renamed to `Shoreline`. New saves should serialize this overlay as `shoreline`; existing saved `tide_bands` values must still load.

Shoreline should look like calm foam at the end of a wave crest. A soft uneven front comes in, leaves a foam pattern behind it, then pulls back while the foam breaks up and decays.

Controls:

- `Travel Distance` controls how far the wash moves.
- `Tide Speed` controls the in/out cycle.
- `Shoreline Warp` bends the front.
- `Foam Breakup` controls decay and breakup.

Validation:

- UI and contact-sheet labels say `Shoreline`.
- Saved JSON uses `shoreline`; legacy `tide_bands` parses to the same enum.
- Time samples show a front plus trailing decay rather than uniform broad bands.

### Wet Sheen

Wet Sheen already has a good base look, but settings must have visible effect. Drift values around 1 or 2 should move the pattern. Normal bias and surface grain should clearly affect the result. The pattern must avoid basic linear bands.

Controls:

- `Sheen Drift` controls organic temporal movement.
- `Normal Bias` increases geometry/normal-driven variation.
- `Surface Grain` adds small glints and fine wet surface texture.
- `Patch Coverage` changes how much surface gets sheen.

Validation:

- Speed changes produce measurable temporal delta.
- Warp changes normal/slope contrast.
- Turbulence changes fine-grain glint count or contrast.
- Contact-sheet tiles read as organic wet patches, not straight bands.

### Current Threads

Current Threads should look like pulses of current flowing from higher points to lower points, spreading outward/downward over the surface. They should not read as manufactured linear bands. Noise should break regularity and create organic path variation.

Controls:

- `Thread Spacing` controls pulse spacing.
- `Thread Drift` controls pulse travel.
- `Branch Density` controls how many threads appear.
- `Thread Wander` and `Thread Flicker` control path variation and breakup.

Validation:

- Pulses move over time.
- Direction is biased by layer direction projected onto the local surface, with normal/height variation breaking straight lanes.
- Neighbor deltas in x and y stay balanced enough to avoid pure parallel bands.

### Droplet Glints

Droplet Glints should produce clustered sparkles that do not pulse all at once. Cluster size must visibly change the cluster radius. Sparkle timing should be spatially decorrelated and move in compelling waves across the area.

Controls:

- `Cluster Size` controls cluster radius.
- `Sparkle Rate` controls shimmer speed.
- `Glint Density` controls how many clusters appear.
- `Cluster Variation` controls shape and timing irregularity.
- `Surface Bias` biases sparkle by local surface variation.

Validation:

- Changing cluster size changes measured cluster footprint.
- Time samples show staggered sparkle waves, not a single global pulse.
- Density changes active glint count.

### Drip Trails

Drip Trails should be blue-toned, sparse random droplet locations that follow surface contours with a small trail of influence. Trails must have length, width, droplet amount, and age/rate behavior. With the same seed and matching amount, Drip Trails and Rain Rings should align droplet origins so overlapping regions can show a ring and a later trail from the same source.

Controls:

- `Trail Length` controls trail length.
- `Travel Speed` controls head motion and age progression.
- `Origin Density` controls droplet amount.
- `Trail Wiggle` controls contour-following perturbation.
- `Tail Breakup` controls decay and broken tail texture.

Validation:

- Output is sparse trails, not static noise.
- Same seed/density produces origin alignment with Rain Rings.
- Time samples show moving heads and fading tails.
- Trails follow projected layer direction/surface direction and do not lose source point relation.

### Foam Sparkle

Foam Sparkle should mimic sea foam: cellular white patches, flecks, bubbles, and broken edges. Edge fade should be correct; the effect should not become stronger because an edge amount is inverted.

Controls:

- `Fleck Scale` controls foam patch and fleck size.
- `Flicker Speed` controls subtle foam sparkle.
- `Foam Density` controls white coverage.
- `Edge Breakup` controls boundary breakup.
- `Foam Drift` moves and warps flecks along the boundary.

Validation:

- Edge-adjacent values fade correctly when edge membership fades.
- Foam patches are coherent cellular/noise sections, not isolated static pixels.
- Contact-sheet tiles resemble the foam reference images.

### Mineral Shimmer

Mineral Shimmer should look like mineral or gold veins over rock. Veins should shimmer along their length with pulses that are not linear screen-space bands. A phase/drift parameter should make veins join, split, disappear, and reform as if moving a section plane through mineral.

Controls:

- `Grain Scale` controls vein network scale.
- `Shimmer Drift` controls vein motion and pulse travel.
- `Patch Coverage` controls how much vein network appears.
- `Crystal Breakup` controls vein breakup and flecks.
- `Surface Bias` ties the pattern to geometry and normals.

Validation:

- Output forms connected vein-like lines rather than static random noise.
- Time samples shimmer along veins.
- Changing warp/phase changes vein connections.
- Geometry/normal changes bias where veins appear.

## Required Evidence

Run and record:

```text
cmake --build build/macos-debug --target invisible_places_tests
cmake --build build/macos-debug --target invisible_places
ctest --test-dir build/macos-debug -R "Water|Ripple|water-ripple-patterns-test-points" --output-on-failure
ctest --test-dir build/macos-debug --output-on-failure
git diff --check
```

Render and inspect:

```text
build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places Data --gui-smoke water-ripple-patterns-test-points --smoke-output build/macos-debug/water-region-smoke
```

Expected artifacts:

- `build/macos-debug/water-region-smoke/water-ripple-patterns-test-points.json`
- `build/macos-debug/water-region-smoke/ripple-pattern-contact-sheet.ppm`
- `build/macos-debug/water-region-smoke/ripple-pattern-contact-sheet.exr`

