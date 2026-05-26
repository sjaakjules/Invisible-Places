# Water Features Redesign v2: Ripples, Flow, and Field

## Purpose

This document revises the water-feature system around three active features:

1. **Ripples** — the current Caustics system renamed and expanded into region-based water overlays.
2. **Flow** — the current source/path baking system, kept as the macro route generator, but with slow trail particles replaced by fast world-aligned stream surfels.
3. **Field** — a new local surface vector-field system for blue-style streamlines and animated surface-flow effects.

The document also removes the legacy **Basin** and **Runoff** feature families from the redesigned workflow.

The main architectural rule is:

```text
Base point cloud visuals remain the base truth.
Water features add virtual effect layers or generated overlay points.
They should not overwrite the user's existing roughness/colour/opacity/size scalar mappings.
```

A second important rule is:

```text
Water remains on the scanned surface.
No waterfall/freefall mode.
If a stream cannot remain on a valid surface, it either bridges a valid surface gap or dies/fades.
```

---

## Resolved design decisions from the latest notes

### Export and persistence

Flow Streams and Field Streamlines are primarily **viewport and render overlays**. They should appear in interactive preview, EXR stack export, and MP4 export. They do **not** need to be exported as PLY files for external applications.

Expensive generated data should still be saved:

```text
Flow path cache           saved as JSON or current path-cache format
Flow stream settings      saved with project
Flow stream cache         optional binary/JSON cache for fast reload
Field cache               saved as vector-field node cache, probably binary + JSON metadata
Field settigns            saved with project
Field stream cache        optional generated stream-sample cache
Field stream settigns     saved with project
Ripple/Field effect cache saved as sparse virtual effect layer cache
Ripple/field efftect settigns saved with project.
```

### Renderer direction

The current renderer uses point/surfel drawing. The redesigned streams should use the same:

```text
stream point = one world-space surfel
stream surfel = oval / elongated Gaussian points
long axis = stream tangent
short axis = stream width
normal = local surface normal
```

Do not make Flow Streams or Field Streamlines screen-facing particle dots. They should be world-aligned so zooming in does not reveal a dotted billboard trail.

### Animation direction

Generated stream points can store scalar fields such as:

```text
stream_seed
stream_id
stream_distance
stream_speed
stream_width
stream_length
point_age
stream_age
point_seed
confidence
wetness
```

The shader should animate these fields using time uniforms wherever possible. GPU-side animation is preferred. CPU regeneration should only be used when topology changes, such as changing stream count, stream length, field resolution, path bake settings, or corridor radius.

### Base cloud effects

Ripples and Field Surface Motion should normally be **virtual effect layers** over the base point cloud rather than new scalar fields that overwrite or fight with the base cloud's existing visual mappings.

The user may already map base point size, opacity, colour, and emission to fields such as roughness, height, classification, intensity, or others. Water effects should contribute to the final evaluated visual values without replacing those base mappings.

### Legacy features

Remove Basin and Runoff from the new water UI. Do not preserve them as main redesigned features.

For old project loading, legacy Basin/Runoff records may be ignored, hidden, and removed from new project saves, lost. The new document assumes they are no longer part of the active feature set.

---

# 1. Shared architecture

## 1.1 Three output types

Water features should produce one of three output types.

```text
A. Virtual Effect Layer
   A sparse or procedural layer attached to a target point cloud.
   Used by Ripples and Field Surface Motion.
   It modifies final rendered values such as colour, opacity, size, and emission.
   It does not overwrite the base cloud's source scalar fields.

B. Generated Stream Overlay
   A generated in-memory point/surfel cloud with stream-specific attributes.
   Used by Flow Streams and Field Streamlines.
   It is visible in viewport and camera exports.
   It does not need to be written as PLY.

C. Cache / Guide Data
   Expensive non-render data saved for reuse.
   Examples: Flow path cache, water corridor cache, Field vector nodes.
```

The features map to these output types as follows:

```text
Ripples
    → Virtual Effect Layer on the base cloud

Flow
    → Flow Path Cache
    → Generated Stream Overlay

Field
    → Field Cache
    → Option A: Generated Stream Overlay
    → Option B: Virtual Effect Layer on the base cloud
```

---

## 1.2 Base visual evaluation plus water contributions

The base point cloud should continue to evaluate normally:

```text
base_size
base_opacity
base_colour
base_emission
base_gaussian_sharpness
```

Water effects then contribute deltas or multipliers:

```text
water_size_add
water_size_mul
water_opacity_add
water_opacity_mul
water_emission_add
water_colour_mix
water_hue_shift
water_colourise_amount
```

Final composition:

```text
final_size     = max(0, base_size * water_size_mul + water_size_add)
final_opacity  = clamp(base_opacity * water_opacity_mul + water_opacity_add, 0, opacity_limit)
final_emission = max(0, base_emission + water_emission_add)
final_colour   = colour_composite(base_colour, water_colour, water_colour_mix, water_hue_shift)
```

The water tabs should expose the parameters that define the water effect. The Visuals tab should remain the place where generic visual properties are mapped, where possible. Effects over the base cloud can have a virtual point to allow visual tab controls of how the effect is modified with saved visuals, defaults and presets, saved on project save. The user can thus select the base cloud or any water effects created and change their visuals. Saved visuals are linked to the cloud to avoid issues of a mapped field not existing in the other cloud, however if all fields are the same, they should be able to be selected in the saved visuals but not edit it until it is saved anew in the active cloud. where '_baseCloud' or '_ripple' etc are appended to the name of the saved visuals when coming from another cloud.

Rule:

```text
If a property is generic point styling, put it in Visuals.
If it is water-specific procedural behaviour, put it in the water feature tab.
If Visuals cannot yet express a necessary water mapping, temporarily expose it in the water tab, but design the name and data model so it can later move into Visuals.
```

---

## 1.3 Virtual effect layers

A virtual effect layer is not a normal base-cloud scalar field. It is an attached effect object that the renderer can evaluate for affected points.

Suggested schema:

```text
WaterEffectLayer
{
    id
    name
    feature_type              ripple / field_surface_motion / other
    target_layer_id
    enabled_in_viewport
    enabled_in_export
    blend_priority
    blend_mode

    region_definition          polygon / path corridor / selected source / full target
    region_bounds
    chunk_list
    lod_policy

    procedural_settings        pattern, speed, scale, phase, direction, etc.
    response_settings          colour, emission, opacity, size, hue, etc.
    composition_settings       add/multiply/max/screen/override weights

    cached_chunks              optional sparse affected-point buffers
    virtual_fields             field names exposed to Visuals
    dirty_flags
    cache_signature
}
```

Each affected chunk can store a compact cache:

```text
WaterEffectChunk
{
    source_chunk_id
    affected_point_count
    affected_point_indices      optional, if referencing base positions
    copied_positions            optional, if drawing as an overlay pass
    copied_normals              optional
    mask
    edge
    local_u
    local_v
    distance
    phase
    seed
    confidence
    extra_attributes
}
```

Two implementation modes are useful:

```text
Sparse referenced mode
    Store point indices into the base cloud plus effect attributes.
    Best when the renderer can fetch base position/normal/colour by index.

Copied virtual cloud mode
    Copy affected positions/normals into an effect overlay cloud.
    Best when the current renderer expects each drawable to own its buffers.
    More memory, but simpler and still much smaller than duplicating the whole 120M cloud.
```

Start with copied virtual cloud mode if it is easier. It can still be fast enough because only the affected region is copied and LOD can reduce far points.

---

## 1.4 Why virtual effect layers are better than dense scalar fields for base-cloud effects

The base point cloud can be around 120 million points. A single dense float scalar field for every point is roughly:

```text
120M × 4 bytes = 480 MB
```

Several water fields quickly become too expensive, especially with multiple overlapping regions. The effect-layer approach avoids this by storing only points that are affected by a water region, or by evaluating the effect procedurally from region parameters.

Expected scale:

```text
base cloud                         ~120M points
visible in viewport                ~20M points
affected by one water region        ~1M–10M points
animated generated stream overlay   ~10K–200K points, commonly ~100K
```

Recommended performance approach:

```text
Normal chunks with no active effect:
    render using the existing base-cloud path.

Chunks with active water effects:
    either run the base shader with an effect stack,
    or draw a virtual effect overlay pass using affected point buffers.

Generated streams:
    render as their own small point/surfel overlay.
```

---

## 1.5 Layer blending for overlapping effects

Multiple Ripple and Field Surface Motion regions may overlap. Each effect layer should expose a contribution and a blend mode.

Useful blend modes:

```text
Add
    Adds emission, point-size boost, colour contribution, etc.
    Good for caustics, shimmer, sparkles.

Max
    Takes the strongest contribution.
    Good for masks, wetness, edge effects.

Multiply
    Multiplies opacity or size.
    Good for darkening, soaking, or fade masks.

Screen / Lighten
    Useful for glowing water highlights.

Override
    Only for debug or explicit user intent.
    Not recommended as a default.
```

Suggested composition policy:

```text
emission_add      = sum of active emission contributions
size_add          = sum, clamped
size_mul          = product of multipliers, clamped
opacity_add       = sum, clamped
opacity_mul       = product of multipliers, clamped
colour_mix        = weighted average by contribution strength and priority
hue_shift         = weighted average or additive with clamp
```

To keep this fast:

```text
max_live_effect_layers_per_chunk = 4 or 8
```

If more active effects overlap the same chunk, pre-compose them into a cached combined water contribution for that chunk.

---

## 1.6 Cache levels

Use dependency-based cache invalidation.

```text
PointCloudSupportCache
    Depends only on source point cloud and normal/scalar availability.
    Stores chunk bounds, spatial lookup, surfel LODs, optional normal confidence.

FlowPathCache
    Depends on Flow sources and path-generation settings.
    Stores baked branch anchors and diagnostics.

FlowStreamCache
    Depends on FlowPathCache and stream generation settings.
    Stores generated stream surfels or enough data to rebuild them quickly.

WaterCorridorCache
    Depends on visible Flow paths or selected Field region plus corridor settings.
    Stores nearby support surfels and path-relative coordinates.

FieldCache
    Depends on WaterCorridorCache and vector-field build settings.
    Stores field nodes, vectors, wetness, confidence, bridge data.

FieldStreamCache
    Depends on FieldCache and streamline generation settings.
    Stores generated streamline surfels.

VirtualEffectLayerCache
    Depends on region definition, target layer signature, effect resolution, and procedural settings that change baked attributes.
    Stores sparse affected-point buffers or copied virtual effect cloud chunks.
```

Visual-only changes should not invalidate expensive caches.

---

# 2. Feature: Ripples

## 2.1 Purpose

Rename the current **Caustics** feature to **Ripples**.

Caustics becomes one overlay type inside Ripples:

```text
Ripples → Overlay Type → Caustic Lace
```

Ripples are region-based animated overlays on the base point cloud. The user selects or draws a region, chooses a water-related pattern, and the renderer modifies the final visual values of affected base-cloud points.

Ripples are best for:

```text
caustic light on rock or sand
linear tide bands
radial rings
wet shimmer
droplet glints on foliage
foam/sparkle accents
subtle symbolic water movement
```

Ripples should usually be implemented as virtual effect layers, not dense permanent scalar fields on the full base cloud.

---

## 2.2 Ripples UI structure

Suggested UI:

```text
Ripples
    Regions
        New Ripple Region
        Draw Boundary
        Close Region
        Selected Region
        Target Layer
        Enabled in Viewport
        Enabled in Export
        Edge Blend
        Region Strength
        Preview Tint

    Overlay Type
        Caustic Lace
        Linear Ripples
        Radial Ripples
        Rain Rings
        Tide Bands
        Wet Sheen
        Current Threads
        Droplet Glints
        Drip Trails
        Foam Sparkle
        Salt / Mineral Shimmer

    Motion
        Speed
        Direction
        Radial Origin
        Expansion Rate
        Wavelength
        Pattern Scale
        Pattern Phase
        Seed Lock
        Warp
        Turbulence

    Response
        Intensity
        Emission Add
        Opacity Add
        Opacity Multiply
        Point Size Add
        Point Size Multiply
        Hue Shift
        Colourise Colour
        Colourise Amount
        Gaussian Sharpness Bias

    Effect Layer
        Blend Mode
        Priority
        LOD Mode
        Max Affected Points
        Rebuild Effect Cache

    Advanced Masking
        Height Band
        Surface Slope Limit
        Normal Confidence Limit
        Foliage / Sparse Mode
        Plane Fit Tolerance
        Mask Voxel Size
```

Controls that duplicate existing Visuals controls should eventually become visual presets or effect-layer contribution settings rather than independent render controls.

---

## 2.3 Ripple overlay types

### Caustic Lace

Preserves the current caustic look.

Visual character:

```text
thin cellular light ridges
animated shimmer
bright emission peaks
strong edge fade at region boundary
```

Primary parameters:

```text
cell_size
line_width
line_sharpness
speed
warp_amount
emission_add
point_size_add
colourise_amount
```

### Linear Ripples

Parallel wave bands moving across the selected region.

```text
phase = dot(position, direction) / wavelength + time * speed + seed
band  = wave_profile(phase)
```

Best for tide wash, shallow sheet movement, or wind-like surface motion.

### Radial Ripples

Expanding rings from one origin or several seeded origins.

```text
distance = length(position - origin)
phase    = distance / wavelength - time * expansion_speed + seed
ring     = ring_profile(phase)
```

Best for droplets, impact circles, or interactive circular water symbolism.

### Rain Rings

Many small radial ripples born from seeded points inside a region.

Good for light rain, water activation, and subtle disturbed surfaces.

### Tide Bands

Large, slow bands that move across a broad region such as sand.

Useful for the 3 m × 10 m tide/sand regions. This should be cheaper than Field, because it only needs region coordinates and procedural animation.

### Wet Sheen

A broad grazing-angle or slope-sensitive highlight.

Useful for making a rock surface feel wet without drawing obvious lines.

### Current Threads

Thin stretched procedural streaks aligned with a chosen direction. This is a low-cost alternative to Field when the user wants motion but not actual streamlines.

### Droplet Glints

Small seeded points or clusters that pulse in emission/size. This is useful for foliage, where there may not be a coherent surface.

Droplet Glints should not require a clean tangent plane. They can use:

```text
point seed
normal confidence
height band
foliage/sparse mask
random pulse phase
view angle
```

### Drip Trails

Short downward or normal-guided streaks on vertical/sparse surfaces. This can be useful on foliage or cliff surfaces without requiring the full Flow/Field system.

### Foam Sparkle

Small bright pulses near selected edges, wet bands, or high-response regions.

### Salt / Mineral Shimmer

Slow granular shimmer, useful for wet sand, salt traces, mineral streaks, or symbolic water residue.

---

## 2.4 Ripple virtual fields

Ripples can expose virtual fields to the Visuals tab without writing full dense base-cloud scalar fields.

Suggested virtual field names:

```text
ripple_mask
ripple_edge
ripple_value
ripple_seed
ripple_region_id
ripple_distance
ripple_linear_coord
ripple_angle
ripple_speed
ripple_confidence
ripple_emission_hint
ripple_opacity_hint
ripple_size_hint
ripple_colour_mix_hint
```

These fields may come from:

```text
procedural shader evaluation
sparse affected-point buffers
copied virtual effect cloud attributes
pre-composed chunk effect buffers
```

The UI can present them similarly to scalar fields, but internally they do not have to be stored as full dense scalar fields on the 120M point layer.

---

## 2.5 Ripple composition

A Ripple effect computes a time-varying value:

```text
ripple_value = pattern(position, normal, local_coordinates, seed, time)
```

Then it contributes to final visual values:

```text
contribution = ripple_value * ripple_mask * ripple_edge * region_strength

emission_add      += contribution * emission_add_setting
opacity_add       += contribution * opacity_add_setting
opacity_mul       *= mix(1, opacity_multiplier_setting, contribution)
point_size_add    += contribution * point_size_add_setting
point_size_mul    *= mix(1, point_size_multiplier_setting, contribution)
colour_mix_amount += contribution * colourise_amount_setting
hue_shift         += contribution * hue_shift_setting
```

For caustics, emission add and colour mix are usually the most important. For tide bands, opacity and point-size modulation may be more useful.

---

# 3. Feature: Flow

## 3.1 Purpose

Flow remains the source-driven route generator.

The user places sources, adjusts route/path settings, and bakes paths using the current path algorithm. The result is a reusable path cache.

The change is:

```text
Keep Flow path baking.
Remove the slow Trail / Particle generation workflow.
Replace it with fast generated world-aligned streams.
```

The yellow paths are still useful, but they are not the final water visual. They are guides.

---

## 3.2 Flow UI structure

Suggested UI:

```text
Flow
    Sources
        Add Source
        Move Source
        Disable Source
        Source Radius
        Source Strength
        Source Speed
        Source Profile

    Path Generation
        Auto Tune
        Branching Flats
        Dense Coverage
        Gap Tolerance
        Path Reach
        Smoothing
        Advanced Path Controls
        Bake Paths

    Path View
        Show Baked Paths
        Hide / Restore Branch
        Path Colour
        Path Width
        Debug Confidence
        Debug Accumulation
        Debug Termination Reason

    Streams
        Enabled
        Generate Streams
        Stream Count
        Streams Per Metre
        Stream Length
        Stream Point Spacing
        Stream Width
        World Splat Length
        World Splat Aspect
        Surface Offset
        Path Attraction
        Lane Spread
        Stream Smoothness
        Stream Looseness
        Turbulence
        Seed Lock

    Stream Playback
        Speed
        Loop Length
        Head Fade
        Tail Fade
        Pulse Profile
        Point Age Mode
        Stream Lifetime Variation

    Stream Scalars / Attributes
        stream_seed
        point_age
        stream_age
        stream_speed
        stream_width
        stream_confidence
        tangent
        normal
```

Retire or hide:

```text
Trail Shape
Animation Trail Playback
Legacy particle lanes
Legacy ghost particles
```

Old trail settings can be migrated approximately into stream defaults but should not remain a primary workflow.

---

## 3.3 Flow path cache

The Flow path cache remains the bake product.

It should store:

```text
source identity
source settings
support signature
path generation settings
branch anchors
branch confidence
branch accumulation
termination reason
gap count
hidden branch ids
path diagnostics
```

Only path-affecting controls dirty the path cache.

Path-affecting examples:

```text
source position
source radius
source strength if used by route generation
branching flats
dense coverage
gap tolerance
path reach
support voxel size
max bridge distance
path sample spacing
path smoothing if applied to stored anchors
```

Stream controls should not dirty the path cache.

---

## 3.4 Flow stream generation

Flow Streams are generated from the baked path anchors.

Process:

```text
1. Load visible branches from the Flow path cache.
2. Build smoothed guide paths from branch anchors.
3. Seed streams along the guide paths.
4. For each stream, choose a start distance, length, width, speed, seed, and lateral offset.
5. Sample positions along the guide path.
6. Apply smooth lateral variation and optional low-frequency turbulence.
7. Project samples to the nearby surface when projection is enabled.
8. Validate or bridge small surface gaps.
9. Stop the stream if the surface cannot be continued.
10. Emit world-aligned stream surfels with scalar/vector attributes.
```

Flow Streams should be much faster than the current trails because they do not need to generate dense lane guides, ghost particles, path-view particles, and full-path animated particles. They are mostly static stream samples animated by shader time.

---

## 3.5 Flow stream surfel schema

Each generated stream sample should be a world surfel.

Suggested attributes:

```text
position
normal
tangent
side_vector, optional
colour, optional default

stream_id
source_id
path_id
branch_id
stream_seed
point_seed
stream_distance
stream_length
point_age
stream_age
stream_speed
stream_width
stream_world_length
stream_confidence
stream_wetness
stream_role
water_feature_type
```

Minimum useful set:

```text
position
normal
tangent
stream_id
stream_seed
point_age
stream_speed
stream_width
stream_world_length
stream_confidence
```

If tangent attributes are not immediately available, tangent can be reconstructed from neighbouring samples in the same stream during generation and written into an auxiliary buffer.

---

## 3.6 World-aligned stream rendering

Flow streams should render as elongated Gaussian surfels, not screen-facing dots.

For each stream sample:

```text
long axis  = tangent * stream_world_length
short axis = side vector or tangent-normal cross product * stream_width
normal     = local surface normal
```

Recommended draw shape:

```text
oval Gaussian splat
capsule-like falloff
soft feathered edge
surface offset from source cloud
```

The stream should remain continuous when zoomed in. Use this relationship:

```text
stream_world_length >= 2.5 × stream_point_spacing
```

Safer default:

```text
stream_world_length = 3.5–5.0 × stream_point_spacing
```

Avoid screen-facing particles for streams. Screen-facing mode can remain useful for mist, foam sparkle, or debugging, but not for the main Flow Stream visual.

---

## 3.7 Flow stream default values

For the current cliff/rock/sand use case:

```text
main rock edge distance                 ~2 m
common corridor width                   ~0.5 m
larger corridor width                   ~1.0 m
full path beyond rock into sand          additional 2–3 m
```

Good starting values:

```text
stream_count_total          = 300–1200
streams_per_path_metre      = 40–150
stream_length               = 0.25–1.20 m
stream_point_spacing        = 0.008–0.015 m
stream_world_length         = 0.035–0.070 m
stream_width                = 0.003–0.012 m
world_splat_aspect          = 4–10
surface_offset              = 0.002–0.008 m
path_attraction             = 0.70–0.95
lane_spread                 = 0.03–0.30 m
stream_smoothness           = 0.70–0.92
stream_looseness            = 0.02–0.20
turbulence                  = 0.00–0.15
speed                       = 0.10–1.50 m/s
```

A strong first preset:

```text
stream_count_total          = 700
stream_length               = 0.75 m
stream_point_spacing        = 0.010 m
stream_world_length         = 0.045 m
stream_width                = 0.006 m
surface_offset              = 0.004 m
path_attraction             = 0.85
lane_spread                 = 0.12 m
stream_smoothness           = 0.85
stream_looseness            = 0.08
turbulence                  = 0.06
speed                       = 0.45 m/s
```

This creates around:

```text
700 streams × 0.75 m / 0.010 m ≈ 52,500 stream samples
```

That is a reasonable overlay size compared with the base cloud.

---

## 3.8 Flow stream animation

Generated stream geometry should usually be static. Motion is shader-driven.

Each stream sample stores a static `point_age`:

```text
point_age = stream_distance / stream_length
```

The shader derives animated age:

```text
animated_age = fract(point_age - time * stream_speed + stream_phase)
```

Useful profiles:

```text
head_bright_tail_fade
    emission high near leading edge, opacity fades behind.

middle_pulse
    strongest in the middle, faded at both ends.

travelling_dash
    only part of the stream visible at once.

continuous_thread
    entire stream visible with subtle travelling emission.
```

Suggested Visuals mappings:

```text
colour       → stream_seed or animated_age
opacity      → animated_age profile × stream_confidence
emission     → animated_age profile × stream_confidence
point size   → stream_width
splat length → stream_world_length
```

---

# 4. Shared surface projection and gap handling

## 4.1 No freefall

There is no waterfall/freefall mode in the revised design.

Every Flow Stream and Field Streamline sample should be one of:

```text
valid projected surface sample
accepted bridge sample across a valid surface gap
terminating/fading sample at end of valid support
```

A stream should not float through space, pass through an occluded rock, or reattach to arbitrary geometry behind an occluder.

---

## 4.2 Why gap handling is difficult

The point cloud has occlusion and sparse/noisy regions. A missing area can mean different things:

```text
valid ground/surface gap
    The scanner missed some points, but the surface plausibly continues.
    Stream should be allowed to bridge.

rock occlusion / object back side
    The stream would pass through empty space around/behind an object.
    Stream should stop.

foliage / moving shrub noise
    Points are sparse, normals point many directions, and there is no reliable surface sheet.
    Surface streams should avoid or die, but droplet/glint effects may still work.
```

The goal is not perfect physical reconstruction. The goal is a fast, controllable classifier that accepts simple surface gaps and rejects obvious occlusions.

---

## 4.3 Surface confidence

Each support surfel or projection sample should have a `surface_confidence` value.

Useful inputs:

```text
point_count
local point density
normal confidence
normal variance
plane-fit error
roughness
height continuity
neighbour connectivity
distance to Flow path / Field corridor
classification or user mask, if available
```

A foliage-like sparse cloud usually has:

```text
low planarity
high normal variance
low local density stability
many disconnected tiny clusters
```

A clean rock/sand surface usually has:

```text
higher planarity
lower normal variance
more connected support
more stable local tangent planes
```

---

## 4.4 Bridge candidate tests

When projection fails or the next integration step reaches a gap, test whether the stream may bridge to a candidate surfel on the other side.

A bridge candidate has:

```text
start point A
start normal nA
start tangent tA
candidate end point B
candidate normal nB
candidate tangent or field direction tB
bridge direction d = normalize(B - A)
bridge distance L
```

Recommended tests:

### 1. Distance test

```text
L <= max_bridge_distance
```

Default:

```text
max_bridge_distance = 0.02–0.12 m
```

Use smaller values for detailed rock surfaces and larger values only when the user explicitly allows aggressive bridging.

### 2. Tangent-plane continuation test

The bridge direction should lie roughly in the tangent planes at both ends:

```text
abs(dot(d, nA)) < plane_exit_limit
abs(dot(d, nB)) < plane_entry_limit
```

This rejects many cases where the path would shoot away from the surface or jump onto an unrelated surface.

Suggested:

```text
plane_exit_limit  = 0.15–0.35
plane_entry_limit = 0.15–0.35
```

### 3. Normal compatibility test

```text
angle(nA, nB) < max_bridge_normal_angle
```

Suggested:

```text
max_bridge_normal_angle = 25–65 degrees
```

Use stricter values for Flow Streams, looser values for Field if the surface is naturally rough.

### 4. Height continuity test

The end point should not require an implausible height jump:

```text
height_delta = dot(B - A, gravity_up)
```

Allow some downhill movement, but reject sudden jumps that look like crossing to another object.

Suggested:

```text
max_uphill_bridge     = 0.005–0.030 m
max_vertical_mismatch = 0.020–0.100 m
```

### 5. Guide alignment test

The bridge should align with the path tangent or field direction:

```text
dot(d, guide_or_field_direction) > min_bridge_alignment
```

Suggested:

```text
min_bridge_alignment = 0.35–0.85
```

### 6. Surface-sheet test

Fit or sample a local sheet around both sides. Accept if:

```text
plane-fit error is low enough
normal variance is low enough
neighbouring support exists to left and right of the bridge
```

This helps distinguish a missing patch on the ground from a jump around a rock edge.

### 7. Occlusion / obstacle-edge rejection

Reject or penalise bridges that start or end near a suspected obstacle silhouette.

Obstacle-edge hints:

```text
high local curvature
large normal variance
sudden density drop
nearby vertical face
large height discontinuity across the bridge
candidate lies behind a convex rim relative to flow direction
```

This is not a perfect occlusion solver, but it will catch many rock-backside cases.

### 8. Foliage/noise rejection

If either side has very low surface confidence or high normal entropy:

```text
reject surface stream bridge
or require a manually painted Bridge Allowed region
```

For foliage, prefer Ripple effects such as Droplet Glints or Drip Trails rather than Flow/Field surface streams.

---

## 4.5 Bridge confidence and visual fade

If a bridge passes, create interpolated bridge samples instead of leaving a geometric hole.

Use a simple Hermite or tangent-plane interpolation:

```text
position(s) = smooth interpolation from A to B using tA and tB
normal(s)   = normalized interpolation from nA to nB
confidence  = base_confidence × bridge_falloff(L)
```

Bridge samples should have lower confidence:

```text
bridge_confidence = 0.2–0.8 depending on bridge quality
```

Visuals can use this to reduce opacity/emission over uncertain gaps.

If a bridge fails:

```text
terminate the stream
fade the last N samples
record termination_reason = surface_gap_rejected
```

---

## 4.6 User controls for difficult gaps

Automatic gap handling will sometimes fail. Provide simple user overrides.

Suggested controls:

```text
Gap Handling
    Bridge Aggression
    Max Bridge Distance
    Normal Continuity
    Plane Continuation Tolerance
    Height Jump Tolerance
    Foliage / Noise Rejection
    Edge / Occlusion Rejection
    Fade On Failed Gap
```

Suggested region tools:

```text
No Flow Region
    Streams die when entering this region.
    Use for shrubs, occluding rocks, bad scan sections.

Bridge Allowed Region
    Allows more aggressive bridging inside the region.
    Use for known scan holes in otherwise continuous ground.

Bridge Blocked Region
    Prevents bridging across the region.
    Use for back sides of rocks or object occlusions.

Surface Only Region
    Requires high-confidence projection.
    Useful for close-up hero shots.
```

This keeps the automatic solution simple while giving the user a reliable manual fix when the scene geometry is ambiguous.

---

# 5. Feature: Field

## 5.1 Purpose

Field builds a local surface vector field from Flow paths and nearby point-cloud support.

It is meant to produce the dense blue-style flow look while keeping the water on the scanned surface.

Field has two output modes:

```text
Option A: Field Streamlines
    Generate stream surfels that follow the vector field.
    Best for readable blue strands.

Option B: Field Surface Motion
    Add a virtual effect layer to the base cloud.
    Best for fast moving bands, shimmer, or surface activation over many points.
```

Field should not build over the entire 120M cloud. It should build only inside a path corridor or selected region.

---

## 5.2 Direct answer: would Field be faster?

A vector field is likely faster for **visual tuning** and for **large-area animated surface motion**, but not if rebuilt from the full cloud every time.

Fast pattern:

```text
Flow paths baked once
    ↓
extract local corridor only
    ↓
downsample to surfels
    ↓
build FieldCache once
    ↓
retrace/reseed streamlines or animate virtual fields cheaply
```

For the current site scale, a typical Field region might be:

```text
rock edge flow area:       ~2–3 m long × 0.5–1.0 m wide
sand fade area:            additional 2–3 m
large tide/ripple region:  ~3 m × 10 m, better handled by Ripples
foliage droplets:          better handled by Ripples / Droplet Glints
```

A 3 m × 1 m Field corridor at 10 mm field spacing is on the order of tens of thousands of field nodes, not millions. That is small enough to cache and tune interactively.

---

## 5.3 Field UI structure

Suggested UI:

```text
Field
    Source
        Use Current Flow Paths
        Use Selected Flow Branches
        Use Selected Region
        Corridor Radius
        Field Resolution
        Projection Resolution
        Rebuild Field

    Surface Field
        Guide Weight
        Downhill Weight
        Graph Flow Weight
        Lateral Wetness Pull
        Field Smoothing
        Wetness Spread
        Surface Offset
        Surface Confidence Threshold

    Gap Handling
        Bridge Aggression
        Max Bridge Distance
        Normal Continuity
        Plane Continuation Tolerance
        Height Jump Tolerance
        Foliage / Noise Rejection
        Manual No Flow / Bridge Regions

    Turbulence
        Curl Strength
        Curl Scale
        Small Detail Amount
        Eddy Density
        Eddy Strength
        Eddy Radius
        Downstream Escape

    Output Mode
        Field Streamlines
        Field Surface Motion
        Both

    Field Streamlines
        Streamline Count
        Seed Spacing
        Streamline Length
        Step Length
        Streamline Width
        World Splat Length
        Momentum
        Max Turn Angle
        Fade On Low Confidence

    Field Surface Motion
        Target Layer
        Effect Layer Mode
        Flow Band Scale
        Flow Band Speed
        Streak Length
        Surface Emission Add
        Surface Opacity Add / Multiply
        Surface Point Size Add / Multiply
        Directional Noise
        Write / Rebuild Virtual Effect Cache

    Debug
        Show Field Nodes
        Show Field Vectors
        Show Wetness
        Show Surface Confidence
        Show Gap Bridges
        Show Rejected Gaps
        Show No Flow Regions
```

Remove any freefall or reattach controls.

---

## 5.4 Field build overview

Process:

```text
1. Start from visible baked Flow paths or a selected region.
2. Create a corridor around those paths.
3. Query nearby point-cloud support only inside the corridor.
4. Downsample support into surfels.
5. Compute surface confidence for each surfel.
6. Build a local surfel graph.
7. Compute guide, downhill, graph-flow, and lateral-pull vectors.
8. Smooth the vector field.
9. Evaluate gap-bridge candidates on the graph.
10. Generate Field Streamlines or Field Surface Motion effect attributes.
```

The path gives the broad route. The point cloud gives the local surface. The vector field gives the visual motion.

---

## 5.5 Corridor extraction

Use the Flow paths as a local search guide.

For each visible branch:

```text
resample original path by arclength
create a heavily smoothed guide copy
build a tube/capsule corridor around the original path
query support points/surfels inside the corridor
store distance and station relative to the smoothed guide
```

Use two path versions:

```text
original path
    Used to include nearby point-cloud samples.
    Keeps all relevant branches and route areas.

smoothed guide path
    Used for broad downstream direction.
    Prevents the vector field from inheriting janky path knots.
```

Suggested values for this site:

```text
path_resample_spacing       = 0.02–0.05 m
path_smoothing_length       = 0.20–0.60 m
corridor_radius             = 0.20–0.60 m
normal_support_extra_radius = 0.02–0.08 m
```

For the rock-edge region, start with:

```text
corridor_radius = 0.35 m
```

Increase toward 0.50–0.60 m if the streamlines need to spread across a wider surface.

---

## 5.6 Surfel levels

Use multiple local LODs inside the corridor.

```text
Projection surfels
    spacing: 0.003–0.008 m
    used for accurate surface projection and bridge tests

Field surfels
    spacing: 0.008–0.020 m
    used for vector-field nodes and streamline integration

Coarse graph surfels
    spacing: 0.030–0.080 m
    used for wetness spread, broad direction, and fast diagnostics
```

Recommended starting values:

```text
projection_surfel_spacing = 0.005 m
field_surfel_spacing      = 0.012 m
coarse_graph_spacing      = 0.050 m
```

Each surfel stores:

```text
position
normal
point_count
normal_confidence
surface_confidence
roughness
planarity
height_along_gravity
distance_to_guide
path_station
guide_tangent
wetness
field_vector
gap_flags
```

---

## 5.7 Local surfel graph

Connect each field surfel to nearby compatible surfels.

An edge is allowed when:

```text
distance is within neighbour radius
both surfels have enough surface confidence
normals are compatible, unless rough-surface mode loosens this
edge direction is plausible relative to the guide/field direction
candidate bridge tests pass for longer gaps
```

Suggested values:

```text
neighbour_radius       = 2–3 × field_surfel_spacing
max_normal_angle       = 45–70 degrees
max_direct_edge_length = 1.8 × field_surfel_spacing
max_bridge_distance    = 0.02–0.12 m
```

Each edge stores:

```text
distance
height_drop
normal_compatibility
guide_alignment
surface_confidence_pair
gap_penalty
bridge_confidence
edge_type: direct / bridge / rejected
```

---

## 5.8 Vector components

Each field surfel computes several vectors.

### Guide vector

From smoothed Flow paths:

```text
v_guide = weighted average of nearby smoothed path tangents
v_guide = project_to_surface(v_guide, normal)
```

### Downhill vector

Projected gravity:

```text
v_downhill = gravity - dot(gravity, normal) * normal
```

### Graph flow vector

From connected neighbouring surfels:

```text
for each neighbour:
    edge_dir = normalize(neighbour.position - node.position)
    drop = node.height - neighbour.height
    slope = drop / edge_length
    alignment = max(0, dot(edge_dir, v_guide))
    score = max(slope, 0)^flow_exponent
          * alignment^alignment_exponent
          * neighbour.surface_confidence
          * edge.bridge_confidence

v_graph = normalize(sum(score * edge_dir))
```

Suggested:

```text
flow_exponent       = 1.0–3.0
alignment_exponent  = 0.5–2.0
```

### Lateral wetness pull

A weak sideways pull toward the wet corridor centre or higher wetness.

```text
v_pull = direction_toward_higher_wetness
v_lateral = v_pull - dot(v_pull, v_guide) * v_guide
```

This helps streamlines fill the corridor without collapsing into the exact yellow paths.

---

## 5.9 Vector blend

At each field surfel:

```text
V = guide_weight    * v_guide
  + downhill_weight * v_downhill
  + graph_weight    * v_graph
  + lateral_weight  * v_lateral
```

Then:

```text
V = normalize(project_to_surface(V, normal))
```

Recommended starting values:

```text
guide_weight      = 0.60
downhill_weight   = 0.45
graph_weight      = 0.80
lateral_weight    = 0.10
```

Adaptive rules:

```text
confident steep surface:
    increase downhill and graph weights

flat or ambiguous surface:
    increase guide weight

low-confidence surface:
    lower wetness, lower seed probability, increase fade, reject aggressive bridges

near path knots:
    reduce exact path following, use turbulence/eddies lightly
```

---

## 5.10 Field smoothing

Smooth the vector field on the surfel graph before generating streamlines.

```text
V_i_new = normalize(
    (1 - smoothing_amount) * V_i
  + smoothing_amount * weighted_average(project V_neighbour onto tangent plane i)
)
```

Suggested:

```text
field_smoothing_iterations = 2–5
field_smoothing_amount     = 0.25–0.50
along_flow_smoothing_bias  = 1.5–3.0
cross_flow_smoothing_bias  = 0.5–1.0
```

Do not smooth across rejected gaps or low-confidence foliage-like clusters.

---

## 5.11 Turbulence and eddies

Field can add turbulence, but keep it surface-bound.

Use low-frequency curl-like perturbations:

```text
V_final = V_base + curl_strength * project_to_surface(curl_noise(position), normal)
```

Suggested:

```text
large_curl_scale       = 0.10–0.50 m
large_curl_strength    = 0.04–0.18
medium_curl_scale      = 0.03–0.15 m
medium_curl_strength   = 0.02–0.10
small_curl_scale       = 0.005–0.030 m
small_curl_strength    = 0.00–0.04
```

Use less small-scale turbulence than expected. Small high-frequency noise creates scribbly streamlines.

Sparse eddies can be placed near:

```text
path knots
flat pooling areas
branch intersections
rough but still surface-confident regions
```

Eddies must include downstream escape so streamlines do not form perfect closed scribbles.

---

# 6. Field output A: Field Streamlines

## 6.1 Purpose

Field Streamlines are generated strings of world-aligned stream surfels that follow the FieldCache vector field.

They are the main way to create the blue reference style:

```text
many thin coherent streaks
surface-bound
slightly turbulent
not exact copies of the yellow paths
```

---

## 6.2 Seeding

Seed throughout the corridor, not just on the path centerline.

Seed probability should depend on:

```text
wetness
surface_confidence
distance_to_guide
path accumulation
user density setting
manual include/exclude regions
```

Use blue-noise/minimum-distance spacing to avoid clumping.

Suggested for the rock-edge region:

```text
streamline_count = 300–1500
seed_spacing     = 0.015–0.050 m
seed_jitter      = 0.000–0.020 m
```

---

## 6.3 Integration

Use midpoint/RK2-style integration through the vector field, with momentum.

```text
x = seed_position
previous_dir = field_direction(x)

repeat until length reached or terminated:
    d1 = field_direction(x, previous_dir)
    x_mid = project_to_surface_or_bridge(x + 0.5 * step_length * d1)

    d2 = field_direction(x_mid, d1)
    dir = normalize(momentum * previous_dir + (1 - momentum) * d2)
    dir = clamp_turn_angle(previous_dir, dir)

    x_next = project_to_surface_or_bridge(x + step_length * dir)

    if x_next invalid:
        fade and terminate

    append x_next
    previous_dir = dir
```

Suggested:

```text
step_length             = 0.5–1.5 × field_surfel_spacing
momentum                = 0.75–0.92
max_turn_angle_per_step = 5–15 degrees
streamline_length       = 0.30–1.50 m for close rock edge work
```

No freefall. Projection must either succeed, bridge, or terminate.

---

## 6.4 Projection

For each candidate sample:

```text
1. Find nearby projection surfels.
2. Reject low-confidence/noisy support unless a bridge is being tested.
3. Gaussian-blend positions and normals.
4. Project the candidate onto the local tangent plane.
5. Reapply surface offset along the blended normal.
6. Validate plane error and normal continuity.
```

Suggested:

```text
projection_radius       = 0.010–0.050 m
surface_offset          = 0.002–0.008 m
max_projection_distance = 0.010–0.060 m
```

If projection fails, run bridge tests. If bridge tests fail, terminate.

---

## 6.5 Streamline sample schema

Use the same generated stream schema as Flow, plus Field-specific fields.

```text
position
normal
tangent
stream_id
stream_seed
stream_distance
stream_length
point_age
stream_speed
stream_width
stream_world_length
field_id
field_region_id
field_wetness
field_confidence
field_surface_confidence
field_bridge_confidence
field_turbulence
field_eddy_strength
field_alignment
termination_reason, optional debug
```

Optional vector attributes:

```text
tangent_x, tangent_y, tangent_z
normal_x, normal_y, normal_z
side_x, side_y, side_z
```

---

# 7. Field output B: Field Surface Motion

## 7.1 Purpose

Field Surface Motion makes the underlying point cloud appear to flow without generating separate streamlines.

It should be implemented as a virtual effect layer, not as dense scalar fields that overwrite the base cloud.

Best uses:

```text
fast blue/gold water motion over a rock surface
subtle flowing bands in a path corridor
large surface activation where individual stream strands are not required
previewing Field direction before generating streamlines
```

Use Ripples, not Field, for large simple tide bands over sand unless a local vector field is specifically needed.

---

## 7.2 Field Surface Motion virtual attributes

For affected base points or virtual effect points, store or compute:

```text
field_mask
field_edge
field_value
field_seed
field_wetness
field_flow_u
field_flow_v
field_speed
field_confidence
field_surface_confidence
field_tangent
field_normal
field_turbulence
field_eddy
field_region_id
```

`field_flow_u` is the downstream coordinate. It lets the shader animate bands along the field:

```text
phase = field_flow_u * band_scale
      + field_seed * seed_amount
      - time * field_speed
      + noise(position) * field_turbulence

band = streak_profile(phase)
```

Then:

```text
contribution = band * field_mask * field_edge * field_wetness * field_confidence
```

The contribution modifies final visual values:

```text
emission_add
opacity_add / opacity_mul
point_size_add / point_size_mul
colour_mix / hue_shift
```

---

## 7.3 Computing field_flow_u

Fast method:

```text
1. Each field surfel stores nearest smoothed-guide path station.
2. Each affected base point inherits nearby surfel station.
3. Add local projection along the field direction.
```

More accurate method:

```text
1. Traverse the directed surfel graph from source nodes.
2. Assign downstream graph distance.
3. Interpolate graph distance to affected base points.
```

Start with the fast method. It is probably sufficient for animated surface bands.

---

## 7.4 Virtual-effect implementation choices

For Field Surface Motion, there are three practical modes.

### Mode 1: Procedural per affected chunk

Store FieldCache and region bounds. In the shader, evaluate the effect for points in active chunks.

Pros:

```text
low memory
no copied affected-point cloud
```

Cons:

```text
shader must sample field/region data
more complex
```

### Mode 2: Sparse affected-point effect layer

Bake affected point indices and compact attributes.

Pros:

```text
faster shader
smaller than dense scalar fields
works well for 1M–10M affected points
```

Cons:

```text
needs point-index referencing or copied data
cache invalidation required
```

### Mode 3: Copied virtual effect cloud

Copy affected positions/normals into an overlay cloud with effect attributes.

Pros:

```text
simplest to integrate with existing point rendering
works like a generated overlay
can have its own Visuals profile
```

Cons:

```text
duplicates affected points
not ideal for many huge overlapping regions
```

Recommended path:

```text
Start with Mode 3 for reliability.
Move to Mode 2 for large repeated regions.
Use Mode 1 later for highly procedural overlays and many overlapping regions.
```

---

# 8. Visuals tab integration

## 8.1 Visuals should see generated fields and virtual fields

The Visuals tab should allow mapping from both:

```text
real scalar fields on generated stream overlays
virtual water fields on effect layers
```

Examples:

```text
Flow Stream overlay:
    real fields: stream_seed, point_age, stream_speed, stream_width

Field Streamline overlay:
    real fields: field_wetness, field_confidence, stream_seed, point_age

Ripple effect layer:
    virtual fields: ripple_value, ripple_mask, ripple_edge, ripple_seed

Field Surface Motion layer:
    virtual fields: field_value, field_wetness, field_flow_u, field_confidence
```

The user should be able to select these fields in mappings for:

```text
colour / colormap
colourise amount
opacity
emission
point size
Gaussian sharpness
```

Where a field is animated, the UI can expose it as:

```text
static field
animated field
water generated field
```

---

## 8.2 Generated stream preset mappings

### Blue Stream Threads

```text
Geometry
    World Stream Surfels

Colour
    cyan/blue gradient mapped to stream_seed or point_age

Opacity
    animated head/tail profile × stream_confidence

Emission
    high at moving leading band, lower at tail

Point size
    stream_width

World splat length
    stream_world_length

Gaussian sharpness
    8–18
```

### Soft Wet Threads

```text
Colour
    source RGB mixed with pale blue/white

Opacity
    low, 0.04–0.20 contribution

Emission
    subtle, only at moving highlights

Point size
    small width, longer splats
```

### Debug Streams

```text
Colour
    branch_id, confidence, termination_reason, or bridge_confidence

Opacity
    constant

Emission
    low
```

---

## 8.3 Ripple preset mappings

### Caustic Lace

```text
colour_mix      = ripple_value × colourise_amount
emission_add    = ripple_value × emission_add
point_size_add  = subtle ripple_value × point_size_add
opacity_add     = optional, usually low
```

### Tide Bands on Sand

```text
colour_mix      = ripple_value × low blue/gold tint
opacity_mul     = subtle wet/dry pulse
point_size_add  = small pulse
emission_add    = very low unless stylised
```

### Droplet Glints on Foliage

```text
emission_add    = sparse pulses
point_size_add  = pulse size
opacity_add     = optional
colour_mix      = small cool highlight
```

---

## 8.4 Field Surface Motion preset mappings

### Surface Flow Glow

```text
colour_mix      = field_value × field_wetness
emission_add    = field_value × field_confidence
opacity_add     = small field_value
point_size_add  = field_value × small boost
```

### Subtle Flow Bands

```text
colour_mix      = low
emission_add    = low
opacity_mul     = slight pulse
point_size_add  = slight pulse
```

---

# 9. Performance strategy

## 9.1 Avoid full-cloud work during tuning

Never rebuild water by scanning the full 120M cloud in response to visual sliders.

Expensive work should happen only when the user changes geometry or support dependencies:

```text
source placement
path bake settings
region boundary
corridor radius
field resolution
gap handling thresholds that affect graph connectivity
```

Cheap work should include:

```text
colour gradient
opacity/emission/point-size mappings
speed
phase
hue
intensity
stream visual length if shader-driven
layer enable/disable
export enable/disable
```

---

## 9.2 LOD for virtual effect layers

For Ripples and Field Surface Motion, use chunked LOD.

Suggested policy:

```text
near camera / high screen coverage:
    use full affected points or fine virtual effect chunks

medium distance:
    use downsampled affected surfels

far distance:
    use coarse effect surfels or procedural-only effect
```

Each effect layer should have a target budget:

```text
max_preview_points
max_export_points
min_screen_size
lod_bias
```

Export can use higher budgets than preview, but it must still use the same effect definitions so the output matches the viewport.

---

## 9.3 Handling 1M–10M affected base points

For an affected region of 1M–10M points:

```text
Viewport preview:
    use LOD/culling; evaluate only visible affected chunks.

Final EXR/MP4 export:
    use the same effect stack with export-quality LOD settings.

Multiple overlapping regions:
    cap live evaluated layers per chunk;
    pre-compose extras into combined effect chunks.
```

Do not duplicate the full 120M base cloud for each effect. Only cache affected chunks or virtual attributes.

---

## 9.4 Generated stream overlay budgets

Generated streams are small compared with the base cloud.

Suggested budgets:

```text
Flow Streams preview          10K–120K stream surfels
Field Streamlines preview     20K–200K stream surfels
Hero export                   50K–500K stream surfels if needed
```

For the current rock-edge camera shots, start near:

```text
50K–120K stream surfels
```

The blue reference style often benefits more from good splat length, opacity profile, and density than from extreme point count.

---

# 10. Persistence and file/cache strategy

## 10.1 Project JSON

Store user-facing settings in the project:

```text
Ripples
    regions, overlay type, procedural parameters, response settings, layer enable/export flags

Flow
    sources, path-generation settings, visible/hidden branch ids, stream settings, visual profile references

Field
    source mode, corridor settings, surfel resolution, vector weights, gap settings, output mode, stream settings, surface motion settings

Visuals
    water visual presets and selected mappings
```

---

## 10.2 Cache files

Save expensive data separately under the project water cache directory.

Suggested names:

```text
<source-stem>-WaterPathCache.json
<source-stem>-WaterFlowStreamCache.bin
<source-stem>-WaterFieldCache.bin
<source-stem>-WaterFieldStreamCache.bin
<source-stem>-WaterEffectLayerCache-<layer-id>.bin
```

Each cache should have metadata:

```text
version
source layer signature
point count
bounds
normal availability
relevant scalar availability
settings fingerprint
region/path fingerprint
creation time
cache type
```

Do not require generated streams or fields to be PLY.

---

## 10.3 Viewport/export consistency

The viewport and camera export should use the same water feature data:

```text
same FlowPathCache
same FieldCache
same virtual effect layers
same generated stream overlay definitions
same shader animation model
same time value / animation frame
```

EXR stack and MP4 export should not require separate external export of water geometry.

---

# 11. Legacy removal and migration

## 11.1 Remove Basin and Runoff from active UI

The new active water tabs are:

```text
Ripples
Flow
Field
```

Remove from active UI:

```text
Basin
Runoff
legacy water animation
legacy trail particles
```

## 11.2 Loading old projects

When loading an old project:

```text
Caustics
    migrate to Ripples with overlay_type = Caustic Lace.

Flow paths
    keep path cache if compatible.
    old trails are ignored or used only to initialise stream settings.

Basin / Runoff
    do not show as active features.
    optional: keep raw legacy records for backward file safety.
    optional: offer one-way conversion to simple Ripple regions.
```

Suggested conversion options:

```text
Basin → Ripple / Wet Sheen or Tide Pulse
Runoff → Ripple / Current Threads or Drip Trails
```

But this conversion is optional. The redesigned system does not depend on Basin or Runoff.

---

# 12. Recommended implementation order

## Phase 1: Rename Caustics to Ripples

```text
Rename UI and saved concepts from Caustics to Ripples.
Keep Caustic Lace behaviour working first.
Add ripple_* compatibility aliases.
Add virtual effect layer structure, even if the first implementation internally reuses existing caustic scalar slots.
Add Linear Ripples and Radial Ripples.
```

## Phase 2: Add water effect composition

```text
Implement base visual + water contribution composition.
Support additive emission, colour mix, opacity add/multiply, and point-size add/multiply.
Support per-effect enable in viewport and export.
Support overlap with simple Add/Max/Multiply modes.
```

## Phase 3: Replace Flow trails with Flow Streams

```text
Keep current path bake.
Hide or remove Trail Shape and Animation Trail Playback.
Generate world-aligned stream surfels from smoothed path anchors.
Store stream_seed, point_age, stream_speed, stream_width, tangent, normal.
Render as elongated Gaussian world surfels.
Animate with shader time.
```

## Phase 4: Add shared surface projection and gap handling

```text
Add surface confidence.
Add projection validation.
Add bridge candidate tests.
Add stream termination/fade when support fails.
Add user No Flow / Bridge Allowed / Bridge Blocked regions.
```

## Phase 5: Build FieldCache

```text
Extract corridor around Flow paths.
Downsample corridor to projection, field, and coarse surfels.
Build local surfel graph.
Compute guide/downhill/graph/lateral vectors.
Smooth the vector field.
Cache field nodes and debug layers.
```

## Phase 6: Field Streamlines

```text
Seed streamlines across the corridor.
Integrate with RK2/midpoint + momentum.
Project or bridge each step.
Terminate on rejected gaps.
Emit same stream surfel schema as Flow Streams.
```

## Phase 7: Field Surface Motion

```text
Generate virtual effect layer attributes from FieldCache.
Animate field_flow_u bands in shader.
Compose emission/opacity/size/colour contributions with base cloud visuals.
Add LOD and chunk budgets for large affected regions.
```

## Phase 8: Remove legacy Basin/Runoff paths

```text
Remove active UI.
Stop generating Basin/Runoff overlays.
Keep only optional migration/loading safety.
Remove legacy animation paths that are no longer used by Flow Streams.
```

---

# 13. Acceptance checks

## Ripples

```text
Old Caustics project loads as Ripples / Caustic Lace.
Caustic Lace still works in the original selected region.
Linear Ripples move in the selected direction.
Radial Ripples expand from selected origin.
Tide Bands work over a large sand region without generating streamlines.
Droplet Glints work on sparse foliage without requiring a clean surface.
Ripples contribute to base visuals without destroying existing roughness/opacity/colour mappings.
Overlapping Ripple regions can be enabled/disabled independently.
Viewport and export match.
```

## Flow

```text
User places sources and bakes paths as before.
Path View shows yellow/debug paths and supports branch hiding.
Flow Streams generate without rerunning path bake.
Changing stream count/length rebuilds only stream overlay, not path cache.
Changing colour/emission/opacity/point-size mappings does not rebuild streams.
Zoomed-in streams appear as continuous world-aligned streaks, not dotted screen sprites.
Streams stop or fade at rejected surface gaps.
Streams can cross accepted ground/surface scan gaps.
```

## Gap handling

```text
Small missing patches in continuous ground can bridge.
Obvious rock backside occlusion is rejected or can be blocked with a manual region.
Foliage-like noisy regions produce low surface confidence.
No Flow / Bridge Allowed / Bridge Blocked regions override automatic choices.
Debug view shows accepted bridges, rejected gaps, and termination reasons.
```

## Field Streamlines

```text
Field builds only around paths/selected region, not the full 120M cloud.
Debug vectors follow broad path direction and local downhill/graph direction.
Streamlines fill the corridor rather than tracing exact yellow paths.
Streamlines stay on the surface or bridge valid gaps.
No freefall behaviour appears.
Increasing turbulence bends streamlines without making scribbles.
Changing visual mappings does not rebuild FieldCache.
```

## Field Surface Motion

```text
Field Surface Motion adds a virtual effect layer to the target cloud.
The base cloud's existing scalar mappings remain intact.
Moving bands follow field_flow_u.
The effect can be enabled/disabled for preview and export.
Large regions use LOD and affected chunks rather than dense full-cloud fields.
Overlapping effects compose predictably.
```

---

# 14. Practical defaults for the current scene

## Rock-edge Flow Streams

```text
stream_count_total          = 700
stream_length               = 0.75 m
stream_point_spacing        = 0.010 m
stream_world_length         = 0.045 m
stream_width                = 0.006 m
surface_offset              = 0.004 m
path_attraction             = 0.85
lane_spread                 = 0.12 m
stream_smoothness           = 0.85
stream_looseness            = 0.08
turbulence                  = 0.06
speed                       = 0.45 m/s
```

## Rock-edge Field

```text
corridor_radius             = 0.35 m
projection_surfel_spacing   = 0.005 m
field_surfel_spacing        = 0.012 m
coarse_graph_spacing        = 0.050 m
field_smoothing_iterations  = 3
field_smoothing_amount      = 0.35
guide_weight                = 0.60
downhill_weight             = 0.45
graph_weight                = 0.80
lateral_weight              = 0.10
streamline_count            = 800
seed_spacing                = 0.025 m
streamline_length           = 0.60–1.20 m
step_length                 = 0.010–0.018 m
momentum                    = 0.85
max_turn_angle              = 10 degrees
```

## Gap handling

```text
bridge_aggression           = 0.45
max_bridge_distance         = 0.060 m
max_bridge_normal_angle     = 45 degrees
plane_exit_limit            = 0.25
height_jump_tolerance       = 0.040 m
foliage_rejection           = 0.70
fade_on_failed_gap          = enabled
```

## Sand tide Ripples

```text
overlay_type                = Tide Bands or Linear Ripples
region_size                 = 3 m × 10 m
wavelength                  = 0.25–1.20 m
speed                       = 0.03–0.20 m/s
edge_blend                  = 0.20–0.80 m
emission_add                = low
opacity_mul                 = subtle wet/dry pulse
point_size_add              = small
LOD mode                    = enabled
```

## Foliage droplets

```text
overlay_type                = Droplet Glints
surface_mode                = Sparse / Foliage
normal_confidence_required  = low or disabled
spawn_density               = low to medium
pulse_lifetime              = 0.5–3.0 s
emission_add                = medium
point_size_add              = small pulse
colourise_amount            = low cool highlight
```

---

# 15. Summary

The revised system should become:

```text
Ripples
    Region-based virtual effects on the base cloud.
    Caustics becomes one Ripple overlay type.
    Supports caustics, linear/radial ripples, tide bands, wet sheen, droplet glints, and symbolic water overlays.

Flow
    User sources bake macro paths with the existing algorithm.
    Trails/particles are replaced by generated world-aligned stream surfels.
    Streams follow paths, remain surface-bound, and animate through shader fields.

Field
    Paths or regions extract a local corridor.
    Corridor surfels create a cached surface vector field.
    Output can be generated streamlines or a virtual surface-motion effect layer.
    No freefall; streamlines project, bridge valid gaps, or terminate.
```

The main performance strategy is:

```text
Do expensive geometry work once and cache it.
Do visual tuning through shader parameters, virtual fields, and small generated overlays.
Do not write dense replacement scalar fields over the full 120M cloud unless there is a specific reason.
```
