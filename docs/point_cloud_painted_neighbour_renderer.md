# Painted Point Renderer Using LOD Neighbour Switching

## Purpose

This document describes the Painted renderer as an extension of the adaptive point-cloud LOD system described in:

```text
point_cloud_adaptive_lod_fast_beauty.md
```

The Painted renderer should use the same cached chunked hierarchy, the same streaming system and the same draw item infrastructure. The difference is that a drawn point or LOD representative can borrow the visual properties of nearby points over time.

The desired look is:

```text
- solid circles/discs rather than soft Gaussian transparency;
- a living painted surface where marks subtly change identity;
- when very close, every source point can still be drawn;
- when farther away, one visible mark may stand for a local neighbourhood;
- marks switch between close neighbours every 0.5–1.0 seconds;
- positions remain stable by default, while colour/normal/scalar material can change;
- an explicit physically faithful preview toggle can disable the neighbour switching.
```

This is not merely a faster LOD path. It is a painterly material/rendering mode that uses the LOD hierarchy to decide which marks are visible and which local neighbours are available for switching.

---

## Relationship to the main LOD system

The main LOD system decides:

```text
which anchors / representatives are drawn
which chunks are resident
which level of the hierarchy is visually appropriate
which draw items are emitted
```

The Painted renderer adds:

```text
which nearby point each anchor borrows material from this epoch
how large the painted mark is
whether marks use screen-space or world-space circles
how overlapping circles choose front/back order
whether physically faithful preview is enabled
```

The Painted renderer should not need a separate point-cloud cache. It should add painted-specific data to the same cache bundle:

```text
PointCloudCache/
    ExampleSite.<fingerprint>.ipcloud/
        hierarchy.bin
        lod_representatives.bin
        raw_chunks/
        painted_neighbors.bin
        painted_reservoirs.bin
        painted_manifest.json
```

---

## Important behavioural distinction

The Fast Basic and Beauty renderers may adapt LOD because of measured render lag in interactive mode. Painted should behave differently.

Painted LOD should primarily be selected from the intended **paint mark size** and camera view, not from frame-time panic. If the user sets a 5 mm painted mark size, the renderer should choose anchors/representatives appropriate to a 5 mm visible mark.

If Painted becomes slow, the renderer should not silently change the artwork by choosing a coarser mark size. Prefer explicit controls:

```text
- pause neighbour switching;
- reduce switching update rate;
- reduce viewport interaction quality only if the user enables it;
- switch temporarily to Fast Basic while navigating;
- show a performance warning;
- allow Adaptive Painted Preview as an explicit mode.
```

This protects art direction. The user should be able to preview the painted look even if it is slower.

---

## Painted concept

A normal point renderer says:

```text
Draw source point P using P's position, colour, normal and scalar fields.
```

The Painted renderer says:

```text
Draw an anchor point A at A's stable position,
but every paint epoch choose a material point M from A's local neighbourhood.
Use M's colour / scalar fields / optional shading normal for the painted look.
```

So a point can look like one of its neighbours without the geometry swimming through space.

Default rule:

```text
Position/depth anchor:  A
Material source:        M, where M is A or one of A's nearby alternatives
```

This is the most stable version. Optional advanced modes may also jitter position or depth layer, but the default should keep the anchor position stable.

---

## Physically faithful preview toggle

Painted needs an explicit toggle:

```cpp
enum class PaintedPhysicalPreviewMode {
    Off,   // use neighbour switching
    On     // use each anchor's own attributes, no neighbour switching, no layer shuffle
};
```

When enabled:

```text
materialPointIndex = anchorPointIndex
normal = anchor normal
scalar fields = anchor scalar fields
colour = anchor colour
paint depth bias = 0
```

This lets the user compare:

```text
What is the point cloud actually doing?
versus
What is the painted neighbour-switching material doing?
```

---

## Mark size model

The Painted renderer should have a user-facing world-space target mark size:

```text
Paint mark diameter: e.g. 5 mm
```

This mark size defines the neighbourhood from which a point can borrow material and the size at which LOD starts replacing many raw points with a representative.

### Close view

When the camera is close enough that source density is visible:

```text
- draw source-density anchors;
- each source point has a small local neighbour set;
- material source switches among anchor + nearest alternatives;
- mark size may be below, equal to, or near the 5 mm target depending on style settings;
- no geometry reduction is required if raw points are visibly distinct.
```

In this state, every point can still be drawn, but the point is visually alive because its colour/normal/scalar material can change to a close neighbour.

### Mid/far view

When many source points fall inside the 5 mm mark neighbourhood:

```text
- use the LOD hierarchy to select fewer anchors/representatives;
- each representative owns a larger neighbour reservoir;
- the visible mark represents a local cluster;
- material switching chooses among points inside that cluster/neighbourhood.
```

In other words:

```text
close: one source anchor, 4 nearby material alternatives
far: one LOD anchor, many possible material alternatives inside the paint radius/reservoir
```

---

## Draw item extension

Painted can extend the shared draw item:

```cpp
struct PaintedDrawItemGpu {
    uint32_t anchorPointIndex;       // stable position/depth point
    uint32_t nodeIndex;

    uint32_t neighbourBase;          // start of neighbour/reservoir table
    uint16_t neighbourCount;         // available material alternatives
    uint16_t paintFlags;

    float representedCount;
    float spacingMeters;
    float paintRadiusMeters;
    float paintRadiusPx;

    float layerSeed;
    float materialSeed;
    float epochBlend;
    float importance;
};
```

If the renderer wants the cheapest close-up path, it can use a packed four-neighbour format:

```cpp
struct PaintedRawNeighbour4Gpu {
    uint32_t packedNeighbourSlots; // 4 × uint8 slots
};
```

The four bytes are not necessarily global point IDs. They should normally index a small local candidate table.

---

## Memory-efficient neighbour representation

Storing four full 32-bit neighbour IDs per source point costs:

```text
120M points * 4 neighbours * 4 bytes = 1.92 GB
```

That is too expensive as a default always-resident GPU buffer.

Better options:

### Option A — byte slots into local candidate tables

For each leaf chunk or microcell, build a local candidate table of up to 256 nearby point IDs.

```cpp
struct PaintedMicrocell {
    uint32_t firstCandidate;
    uint16_t candidateCount; // <= 256 for byte indexing
    uint16_t flags;
};

struct PaintedRawNeighbour4Gpu {
    uint32_t microcellIndex;
    uint32_t packedCandidateSlots; // four uint8 values
};
```

Shader:

```glsl
uint packed = rawNeighbour4[anchorIndex].packedCandidateSlots;
uint slot = (packed >> (selectedSlot * 8u)) & 0xffu;
uint materialPointIndex = microcellCandidates[microcell.firstCandidate + slot];
```

This gives four alternatives per point using four bytes of slot data, with a shared candidate table.

### Option B — chunk-local 16-bit indices

If a leaf chunk has <= 65,535 points, store four local uint16 offsets:

```cpp
struct PaintedNeighbour4x16Gpu {
    uint16_t localIndex0;
    uint16_t localIndex1;
    uint16_t localIndex2;
    uint16_t localIndex3;
};
```

Cost:

```text
120M * 8 bytes = 960 MB
```

This is simpler than byte-slot tables but heavier.

### Option C — only resident painted neighbours for visible chunks

Do not keep painted neighbour data for all 120M points on the GPU. Stream neighbour data alongside visible point chunks.

```text
visible raw chunk -> upload positions/colours/normals/scalars
visible painted chunk -> upload neighbour slots/candidate table
```

This is recommended. The full cache can exist on disk, but GPU memory should contain only visible/recently visible painted data.

### Recommended default

Use:

```text
visible-chunk streaming + byte slots into local candidate tables
```

This satisfies the goal of four close alternatives while keeping memory manageable.

---

## Neighbour build process

During cache build, construct painted neighbour data in addition to the standard hierarchy.

```text
1. Partition points into leaf chunks and microcells.
2. For each point, find nearby points within a local radius.
3. Select up to four close alternatives for close-up painted switching.
4. For each LOD node, build a larger reservoir of material alternatives.
5. Store candidates with stable order and importance/rank.
6. Write painted_neighbors.bin and painted_reservoirs.bin.
```

The close-up four alternatives should prefer:

```text
- spatial closeness;
- similar normal direction unless the style wants more variation;
- similar depth/surface side to avoid borrowing through walls;
- colour/scalar variation only within a safe local radius;
- at least one same-material/self slot.
```

Recommended four slots:

```text
slot 0: self
slot 1: nearest spatial neighbour
slot 2: nearest neighbour with colour/scalar variation
slot 3: nearest neighbour with similar normal but different local position
```

For safety, do not allow the neighbour set to cross obvious discontinuities unless the user enables an experimental mode:

```text
- large normal difference;
- large depth discontinuity;
- different classification/material region;
- different connected component if known;
- far outside paint radius.
```

---

## LOD node reservoirs

At farther distances, four alternatives may not be enough. Each LOD node should store a reservoir:

```cpp
struct PaintedReservoirHeader {
    uint32_t firstCandidate;
    uint16_t candidateCount;
    uint16_t flags;

    float radiusMeters;
    float colorVariance;
    float scalarVariance;
    float normalVariance;
};
```

Candidate classes:

```text
- self/source representative;
- spatially central samples;
- colour variations;
- scalar min/max/threshold points;
- normal/edge variation;
- emissive/accent samples;
- random flavour samples.
```

When the camera is far enough that a 5 mm mark covers many points, the material source can be chosen from the node reservoir instead of the close-up four-neighbour set.

---

## Selection rule by paint radius

Let:

```text
paintDiameterMeters = user setting, e.g. 0.005 m
paintRadiusMeters = paintDiameterMeters * 0.5
```

For each candidate node:

```cpp
float projectedPaintRadiusPx = paintRadiusMeters * pixelsPerMeter;
float projectedSpacingPx = node.spacingMeters * pixelsPerMeter;
```

LOD target:

```text
Choose anchors such that spacingMeters is near the desired painted mark spacing,
while still refining if source detail would be visibly distinct or feature variance is high.
```

A simple rule:

```cpp
bool canRepresentWithThisNode =
    node.spacingMeters <= paintDiameterMeters * maxSpacingMultiplier &&
    projectedSpacingPx <= targetPaintSpacingPx &&
    !nodeHasImportantUnresolvedFeature;
```

Close-up exception:

```text
If individual source points are visibly distinct, draw raw/source anchors even if the paint mark is 5 mm.
```

Far-field rule:

```text
If many source points fall inside one painted mark and the node's feature stats are resolved,
select the node representative and use its reservoir for material switching.
```

---

## Attribute switching

The renderer should distinguish anchor attributes from material attributes.

```text
Anchor point A:
    position
    depth
    optional geometric normal for surfel orientation
    local spacing / represented count

Material point M:
    RGB
    scalar fields for colour/style
    optional painted shading normal
    emissive/accent fields
```

Default:

```text
position = A.position
surfel orientation normal = A.normal or node aggregate normal
colour = M.colour
scalar style = M.scalar
painted normal shading = mix(A.normal, M.normal, normalBorrowAmount)
```

Why not switch the position by default?

```text
Changing positions causes geometry crawling/swimming.
Borrowing material while keeping anchors stable gives a living paint surface without losing spatial trust.
```

Optional advanced modes:

```text
- position jitter within a fraction of paint radius;
- depth-layer bias for overlap animation;
- full material+position switch for abstract mode only.
```

---

## Random switching over time

Switch material every 0.5–1.0 seconds using a stable epoch.

```cpp
float changeIntervalSeconds = 0.75f;
uint32_t epoch = uint32_t(floor(timeSeconds / changeIntervalSeconds));
```

For each anchor:

```cpp
uint32_t h = Hash(anchorPointIndex, nodeIndex, epoch, styleSeed);
uint32_t selectedSlot = h % neighbourCount;
```

To avoid shimmer:

```text
- do not change every frame;
- use stable hashes, not random CPU updates per frame;
- optionally use blue-noise ordering or spatially coherent hash;
- optionally crossfade over 50–150 ms.
```

Crossfade version:

```cpp
uint32_t previousEpoch = epoch - 1;
uint32_t oldSlot = Hash(anchor, previousEpoch, seed) % count;
uint32_t newSlot = Hash(anchor, epoch, seed) % count;
float epochBlend = smoothstep(0.0f, fadeSeconds, timeSinceEpochStart);
```

Then blend material values:

```glsl
vec3 colour = mix(oldColour, newColour, epochBlend);
float scalar = mix(oldScalar, newScalar, epochBlend);
vec3 normal = normalize(mix(oldNormal, newNormal, epochBlend));
```

For hard-edged painted switching, crossfade can be disabled.

---

## Solid circles and front/back movement

Painted uses solid or near-solid circles/discs.

Modes:

```cpp
enum class PaintedGeometryMode {
    ScreenCircles,
    WorldDiscs
};
```

### Screen circles

```text
- camera-facing circles;
- size in pixels or projected from paint radius;
- hard edge or slight feather;
- good for graphic stipple/pointillist look.
```

### World discs

```text
- normal-oriented discs;
- size in metres;
- better for surface-like paint;
- can share the surfel expansion path.
```

### Moving in front of each other

When close circles overlap, the renderer can create a living painted layer effect with a small stable depth bias.

```cpp
float layer = Hash01(anchorPointIndex, epoch, layerSeed);
float signedLayer = layer * 2.0f - 1.0f;
float depthBiasMeters = signedLayer * paintRadiusMeters * layerStrength;
```

Apply in view space:

```glsl
vec4 viewPosition = uniforms.view * vec4(anchorPosition, 1.0);
viewPosition.z += depthBiasMeters;
gl_Position = uniforms.projection * viewPosition;
```

Rules:

```text
- layerStrength defaults low, e.g. 0.02–0.15 of paint radius;
- disable in physically faithful preview;
- do not use large bias where it breaks depth trust;
- make the bias deterministic per epoch.
```

For full opacity solid circles, this may be enough. For semitransparent painted marks, use the Beauty transparency path or a simple weighted accumulation.

---

## Painted opacity and size rules

Painted should not use the same physical opacity compensation as Beauty by default. Beauty tries to preserve the density of many translucent source points. Painted tries to place readable marks.

Default painted mark rule:

```text
opacity = user painted opacity
size = user painted mark size
neighbour switching affects material identity, not physical density
```

Optional modes:

```cpp
enum class PaintedDensityMode {
    ArtisticMarkDensity,       // default; preserve intended mark layout
    MatchSourceDensity,        // closer to Beauty/raw point density
    PhysicalPreview            // no switching, no stylised density changes
};
```

When using LOD representatives at distance:

```text
- do not multiply opacity by representedCount automatically;
- instead ensure mark spacing/size covers the intended painted surface;
- preserve accents by reservoir selection rather than emission multiplication.
```

This avoids distant painted points becoming too opaque or too emissive.

---

## Colour, scalar, normal and emissive behaviour

### Colour

Colour is usually borrowed from the selected material point:

```text
paintedColour = colour(materialPointIndex)
```

Optionally quantize or stylise after borrowing:

```text
borrowed colour -> palette/paint stylisation -> output colour
```

### Scalar fields

Scalar fields used for visual style should come from the material point:

```text
scalarForColour = scalar(materialPointIndex)
```

Scalar fields used for geometry, culling or depth should stay with the anchor:

```text
scalarForGeometry = scalar(anchorPointIndex)
```

### Normals

Use separate normals:

```text
geometry normal = anchor normal or node aggregate normal
painted shading normal = borrowed material normal, blended with anchor normal
```

Recommended parameter:

```cpp
normalBorrowAmount in [0, 1]
```

Default:

```text
0.25–0.5 for subtle living normal variation
0.0 for physically faithful preview
1.0 for stronger painted abstraction
```

### Emissive

Borrowing emissive blindly can produce flicker. Use a mode:

```cpp
enum class PaintedEmissionBorrowMode {
    AnchorOnly,
    BorrowSubtle,
    BorrowFull,
    PreserveAccents
};
```

Recommended default:

```text
BorrowSubtle or PreserveAccents
```

For accent points, keep reservoir candidates that preserve rare emissive features.

---

## Shader model

The Painted shader needs two point indices:

```glsl
uint anchorIndex;
uint materialIndex;
```

Pseudo-code:

```glsl
PaintedDrawItem item = paintedDrawItems.items[itemIndex];
uint anchorIndex = item.anchorPointIndex;

uint materialIndex = anchorIndex;
if (paintControl.physicalPreview == 0u) {
    materialIndex = SelectPaintMaterialIndex(item, uniforms.time, paintControl.seed);
}

vec3 anchorPosition = positions[anchorIndex].xyz;
vec3 anchorNormal = normals[anchorIndex].xyz;

vec4 materialColour = UnpackColor(colors[materialIndex]);
float materialScalar = LoadScalarFieldValue(fieldSlot, materialIndex);
vec3 materialNormal = normals[materialIndex].xyz;

vec3 paintedNormal = normalize(mix(anchorNormal, materialNormal, paintControl.normalBorrowAmount));
```

Important:

```text
- position/depth uses anchorIndex;
- visible material uses materialIndex;
- physically faithful preview forces materialIndex = anchorIndex;
- scalar loading must support materialIndex, not only gl_VertexIndex.
```

This means the existing raw shaders should not be modified directly. Add Painted-specific shaders or shared helper functions that accept explicit point indices.

---

## Screen-circle vertex path

For screen circles:

```glsl
uint itemIndex = uint(gl_VertexIndex);
PaintedDrawItem item = paintedDrawItems.items[itemIndex];

vec3 pos = positions[item.anchorPointIndex].xyz;
vec4 viewPos = uniforms.view * vec4(pos, 1.0);
viewPos.z += ResolvePaintLayerBias(item);

gl_Position = uniforms.projection * viewPos;
gl_PointSize = ResolvePaintPointSizePx(item);
```

Fragment:

```glsl
vec2 p = gl_PointCoord * 2.0 - 1.0;
if (dot(p, p) > 1.0) discard;

outColor = vec4(paintedColour, paintedOpacity);
```

For a truly solid painted circle:

```text
falloff = hard disc
opacity = 1 or user painted opacity
```

---

## World-disc vertex path

For world discs, reuse the surfel idea:

```glsl
uint encoded = uint(gl_VertexIndex);
uint itemIndex = encoded / 6u;
uint cornerIndex = encoded - itemIndex * 6u;

PaintedDrawItem item = paintedDrawItems.items[itemIndex];
uint anchorIndex = item.anchorPointIndex;

vec3 center = positions[anchorIndex].xyz;
vec3 normal = ResolveAnchorOrAggregateNormal(item);
mat2x3 basis = BuildDiscBasis(normal);
vec2 corner = discCorners[cornerIndex];

float radius = item.paintRadiusMeters;
vec3 world = center + basis.tangent * corner.x * radius + basis.bitangent * corner.y * radius;
```

This should be preferred for surface-like painted rendering and for far views.

---

## Interaction with the main LOD selector

The main LOD selector emits anchors based on:

```text
- source visibility;
- projected spacing;
- paint mark size;
- feature variance;
- viewport mode;
- streaming availability.
```

Painted then selects material neighbours.

Do not choose a coarser anchor just because the neighbour reservoir is larger. The anchor density still needs to satisfy the desired visible mark layout.

A good selection policy:

```text
If raw point spacing is visually resolvable:
    draw raw anchors; use 4-neighbour close switching.

If many raw points fall inside one paint mark:
    draw LOD representatives; use node reservoir switching.

If node contains high scalar/colour/normal variance:
    draw more representatives from different classes.

If physically faithful preview is on:
    draw same anchors, but material source = anchor.
```

---

## Export behaviour

Painted export should be deterministic.

```cpp
enum class PaintedExportMode {
    StillAtFixedEpoch,
    AnimatedEpochs,
    PhysicallyFaithful,
    MatchViewport
};
```

### StillAtFixedEpoch

```text
Use one chosen paint epoch and seed.
Good for EXR stills.
```

### AnimatedEpochs

```text
Switch every 0.5–1.0 seconds during MP4 export.
The same camera path and seed reproduce the same animation.
```

### PhysicallyFaithful

```text
Disable neighbour switching and depth-layer movement.
```

### MatchViewport

```text
Use the same epoch/seed/LOD state that the user saw in the viewport.
```

Unlike Fast/Beauty interactive modes, Painted export should not silently adapt mark density based on render time unless the user explicitly selects an adaptive preview-quality export.

---

## UI suggestions

Renderer selection:

```text
Renderer:
    Fast Basic
    Beauty
    Painted
```

Painted controls:

```text
Paint mark size: 5 mm
Neighbour switching: On / Off
Physically faithful preview: On / Off
Switch interval: 0.5–1.0 s
Switch style: Hard / Crossfade
Normal borrow amount: 0–100%
Scalar borrow: On / Off / Colour only
Emission borrow: Anchor / Subtle / Full / Preserve accents
Layer shuffle: Off / Subtle / Strong
Geometry: Screen circles / World discs
```

Diagnostics:

```text
Painted anchors drawn: 42.1M
Source points represented: 120.0M
Average neighbour count: 4 close / 28 reservoir
Paint epoch: 137
Switch interval: 0.75 s
Physical preview: Off
Layer shuffle: Subtle
```

If performance is poor:

```text
Painted renderer is expensive in this view.
The painted mark density has not been changed automatically.
Switch to Fast Basic while navigating, pause switching, or enable Adaptive Painted Preview.
```

---

## Implementation phases

### Phase 1 — Painted close-up prototype

```text
- Use raw source anchors only.
- Store four neighbours for visible chunks.
- Shader chooses materialIndex from anchor + 4 alternatives.
- Position remains anchor position.
- Solid screen circles only.
- Add physically faithful preview toggle.
```

### Phase 2 — Cache integration

```text
- Build painted_neighbors.bin during cache build.
- Stream neighbour data alongside visible chunks.
- Add byte-slot local candidate tables.
- Add deterministic epoch switching.
```

### Phase 3 — LOD reservoirs

```text
- Add painted_reservoirs.bin per LOD node.
- Select reservoir material points when using LOD representatives.
- Preserve scalar/emissive/normal/colour feature candidates.
```

### Phase 4 — World discs and layer shuffle

```text
- Add world-disc painted shader.
- Reuse surfel expansion structure.
- Add subtle deterministic depth-layer bias.
- Add crossfade/hard-switch modes.
```

### Phase 5 — Export and polish

```text
- Add deterministic EXR/MP4 export modes.
- Add MatchViewport and PhysicallyFaithful outputs.
- Add diagnostics and logs.
- Add optional Adaptive Painted Preview, clearly labelled.
```

---

## Key design choices

```text
1. Anchor geometry stays stable.
2. Material can switch to nearby neighbours.
3. Close-up uses four compact alternatives per visible point.
4. Far view uses LOD node reservoirs.
5. Painted LOD is driven by mark size and visual intent, not hidden render-lag adaptation.
6. Physically faithful preview is always available.
7. The system reuses the main LOD hierarchy and streaming cache.
```

This makes the painted look feel alive while preserving spatial trust and avoiding a separate duplicate point-cloud system.
