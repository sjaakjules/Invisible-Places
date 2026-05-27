# Load On Demand Implementation


## Existing Situation:
Right now the current preview LOD is basically:

1) loaded point cloud
    - optional one-time spatial sample index list
    - preview draw count target, currently 10M
    - indexed draw if the sample buffer is ready

That is visible in:

- PointCloudPreviewState(1).cpp
    - GenerateSpatialSampleIndices(...)
    - ResolvePointCloudPreviewLod(...)

and then in:

- Application.cpp
    - kPointCloudPreviewLodTarget = 10,000,000
    - EffectivePointDrawCount(...)
    - PreparePreviewLodSampleCache(...)

and finally:

- VulkanViewportShell.cpp
    - UpdatePointBudget(...)
    - UpdateInteractivePointSampleBuffer(...)
    - ResolvePointCloudDrawPlan(...)
    - RecordPointCloudLayerDraw(...)

That is a good first-generation system, but the next version should stop thinking in terms of only:

__drawPointCount__

and start thinking in terms of:

__which representatives should be drawn for this camera, style, quality mode, and export mode?__

## The big change 
Add a new structure beside your existing point buffers:

- PointCloudLodHierarchy
- PointCloudLodNode[]
- PointCloudLodRepresentative[]
- PointCloudDrawItem[]

Do not duplicate all 120M point attributes at every LOD. Keep your current raw source buffers:

- positions
- colors
- normals
- scalar fields

Then let LOD representatives refer back to original source points, plus carry extra LOD metadata.

Something like:

```
struct PointCloudLodNode {
    glm::vec3 boundsMin;
    glm::vec3 boundsMax;
    uint32_t firstChild;
    uint32_t childCount;
    uint32_t firstRepresentative;
    uint32_t representativeCount;
    uint32_t representedPointCount;
    float spacingMeters;
    float density;
    float normalVariance;
    float colorVariance;
    uint32_t flags;
};
```
Then:
```
struct PointCloudLodRepresentative {
    uint32_t sourcePointIndex;
    float representedCount;
    float spacingMeters;
    float importance;
    float blueNoiseRank;
    float scalarMin;
    float scalarMax;
    float scalarMean;
    uint32_t styleClass;
    uint32_t nodeIndex;
};
```

At runtime, the camera traversal emits a compact draw list:
```
struct PointCloudDrawItemGpu {
    uint32_t sourcePointIndex;
    uint32_t nodeIndex;
    float representedCount;
    float spacingMeters;
    float radiusScale;
    float opacityCoverageScale;
    float emissionCoverageScale;
    float lodBlend;
    float randomSeed;
    uint32_t styleClass;
};
```
This draw item buffer becomes the input to your advanced LOD renderers.

The important part is sourcePointIndex. Your current shaders assume that the vertex index is also the point index when loading scalar fields. For example, pointcloud_preview.vert uses gl_VertexIndex as the point index, and scalar lookup is based on that point index. That works for your current indexed-draw path because the index buffer redirects gl_VertexIndex to the original source point. But for generated LOD representatives, you need the shader to distinguish:
```
draw item index
source point index
represented point count
LOD node index
```
So I would create new LOD shaders rather than trying to overfit the existing pointcloud_preview.vert.

### Keep the current path

I would keep your current path as:
```
Fast Basic:
    current raw/sequential/indexed draw path
Beauty Raw:
    current point/surfel path
```
Remove the current:
```
Preview Legacy LOD:
    current 10M sampled index path
```
Then add the following to replace the old LOD that wasn't working with:
```
Beauty Adaptive LOD:
    hierarchy traversal -> draw item buffer -> LOD-aware sprite/surfel shaders
Painted / Sketch Renderer:
    same hierarchy traversal -> artistic draw item buffer -> brush/surfel-stroke shaders
```
This lets you migrate gradually without destabilising the current renderer.

### What changes in the renderer

Your current draw plan is roughly:
```
drawPointCount
worldSurfels
sampledBudgetReady
interactiveSampleReady
```
I would extend the concept to something more like:
```
enum class PointCloudDrawSource {
    RawSequential,
    RawSampledIndexBuffer,
    InteractiveSampledIndexBuffer,
    LodDrawItems,
    PaintedDrawItems
};
struct PointCloudDrawPlan {
    ActivePointCloudResources* resources = nullptr;
    PointCloudDrawSource source = PointCloudDrawSource::RawSequential;
    uint32_t drawPointCount = 0;
    uint32_t drawItemCount = 0;
    bool worldSurfels = false;
    bool screenSprites = false;
    bool painted = false;
};
```
Then RecordPointCloudLayerDraw(...) can branch:
```
if (plan.source == PointCloudDrawSource::LodDrawItems) {
    RecordPointCloudLodLayerDraw(...);
} else if (plan.source == PointCloudDrawSource::PaintedDrawItems) {
    RecordPaintedPointCloudLayerDraw(...);
} else {
    RecordCurrentPointCloudLayerDraw(...);
}
```
This avoids forcing everything through vkCmdDrawIndexed against the raw point arrays.

## New shader model

For the LOD sprite shader, instead of:
```
const uint pointIndex = uint(gl_VertexIndex);
vec4 worldPosition = vec4(inPosition, 1.0);
```
use:
```
const uint itemIndex = uint(gl_VertexIndex);
PointCloudDrawItem item = drawItems.items[itemIndex];
const uint pointIndex = item.sourcePointIndex;
vec3 worldPosition = sourcePositions.positions[pointIndex].xyz;
vec4 sourceColor = unpackColor(sourceColors.colors[pointIndex]);
```
Then scalar lookup stays source-correct:
```
float value = LoadScalarFieldValueForPoint(fieldSlot, pointIndex);
```
For surfels, you can keep your existing six-vertices-per-point model, but reinterpret it as six vertices per draw item:
```
const uint encodedVertexIndex = uint(gl_VertexIndex);
const uint itemIndex = encodedVertexIndex / 6u;
const uint cornerIndex = encodedVertexIndex - itemIndex * 6u;
PointCloudDrawItem item = drawItems.items[itemIndex];
const uint pointIndex = item.sourcePointIndex;
```
This is a clean extension of your existing pointcloud_surfel(1).vert, which currently does:
```
pointIndex = encodedVertexIndex / kSurfelVerticesPerPoint;
cornerIndex = encodedVertexIndex - pointIndex * kSurfelVerticesPerPoint;
```
You would simply replace “point index” with “draw item index,” then fetch the original source point from the draw item.

## LOD selection should be style-aware

Do not select LOD only from camera distance. Your real performance problem is:

visible point count * projected splat area * overdraw

For each visible LOD node, estimate:

screenPxPerMeter =
    abs(projection[1][1]) * viewportHeight / (2.0f * viewDepth);
projectedSpacingPx =
    node.spacingMeters * screenPxPerMeter;
projectedNodeAreaPx =
    ProjectBoundsToScreenArea(node.bounds);

Then estimate cost differently for screen sprites and surfels.

For screen sprites:

radiusPx = userPointSizePx * 0.5f;
fragmentCost = representativeCount * pi * radiusPx * radiusPx;

For world surfels:

diameterWorld = max(userSurfelDiameter, node.spacingMeters * fillFactor);
radiusPx = 0.5f * diameterWorld * screenPxPerMeter;
fragmentCost = representativeCount * pi * radiusPx * radiusPx;

Then your LOD traversal decides whether to refine the node:

if (projectedSpacingPx > targetSpacingPx &&
    fragmentCostStillAcceptable &&
    nodeHasChildren) {
    visit children;
} else {
    emit representatives from this node;
}

For preview, targetSpacingPx might be around:

1.0 to 2.0 px

For high-quality export:

0.25 to 0.75 px

For full source:

leaf/raw only

But even in export, I would not think of LOD as automatically “lower quality.” A good adaptive LOD can be visually equivalent when raw points are far denser than pixels.

## The visual compensation problem

Your concern is correct: if one representative stands in for hundreds of points, opacity and emission can blow out or collapse unless you change the math.

The user-facing style value should stay stable:

Point Size
Opacity
Emission
Falloff
Color mode
Scalar mappings

Then the renderer applies an internal LOD compensation layer.

For opacity, use optical depth rather than linear alpha.

Given:

alphaRaw = user opacity after scalar/style evaluation
representedCount = number of original points represented
rawArea = area of the original point/surfel footprint
repArea = area of the LOD representative footprint

compute:

coverageMultiplier = representedCount * rawArea / max(repArea, epsilon);
alphaLod = 1.0 - pow(1.0 - alphaRaw, coverageMultiplier);

Equivalent exponential form:

tauRaw = -log(max(1e-5, 1.0 - alphaRaw));
tauLod = tauRaw * coverageMultiplier;
alphaLod = 1.0 - exp(-tauLod);

That means:

fewer larger representatives do not automatically become too transparent
fewer same-size representatives do not automatically lose density
larger representatives do not automatically become over-opaque

For emission, I would separate two modes:

enum class LodEmissionMode {
    PreserveAverageBrightness,
    PreserveEnergy,
    PreserveAccents
};

For normal glow/diffuse emission:

emissionLod = emissionRaw * coverageMultiplier;

but clamp or tone-map it:

emissionLod = min(emissionLod, maxEmissionBoost);

For rare emissive accents, do not average them into a grey cell. Keep separate accent representatives:

normal representatives
scalar-extreme representatives
emissive/accent representatives
edge/normal-variation representatives

Otherwise your LOD will erase exactly the interesting artistic data.

## Screen sprites and world surfels should not use the same LOD rules

You are right that the new renderer will behave differently for screen sprites versus world surfels.

### Screen sprites

Screen sprites are style marks in image space. Their radius is not naturally connected to the real point spacing. So the LOD selector should be very conservative about automatically increasing their size.

I would give screen sprites three internal policies:

enum class ScreenSpriteLodFootprintMode {
    LiteralUserSize,
    DensityFill,
    PerformanceClamped
};

### LiteralUserSize
Uses exactly the user’s point size. This is best for full-quality export, but it can be extremely slow and can blow out if 120M points all overlap.

### DensityFill
Allows LOD representatives to grow enough to hide holes, but uses opacity compensation so the result does not become too bright or too opaque.

### PerformanceClamped
During navigation, clamps maximum point size and maximum accumulated density per screen region. This may not exactly match final quality, but it keeps the viewport alive.

For screen sprites, I would use:

lodRadiusPx =
    clamp(
        max(userRadiusPx, projectedSpacingPx * fillFactor),
        minRadiusPx,
        maxPreviewRadiusPx);

Then:

coverageMultiplier =
    representedCount * userRadiusPx * userRadiusPx /
    max(lodRadiusPx * lodRadiusPx, epsilon);

During interactive preview, I would allow the renderer to say:

I am not going to draw huge 20 px translucent sprites for every visible dense cell.
I will draw representative marks and preserve approximate visual density.

During export, the user can choose whether they want:

literal full source
adaptive high quality
preview-matched density
artistic/stochastic look

### World surfels

World surfels are much easier to make physically stable because the footprint can be tied to world spacing.

For world surfels:

lodDiameterWorld =
    max(userSurfelDiameterWorld,
        node.spacingMeters * fillFactor);

Then project that into pixels.

For opacity:

rawDiameterWorld =
    max(userSurfelDiameterWorld,
        rawSpacingMeters * fillFactor);
coverageMultiplier =
    representedCount * rawDiameterWorld * rawDiameterWorld /
    max(lodDiameterWorld * lodDiameterWorld, epsilon);

This makes coarse surfels cover holes without becoming automatically over-bright.

The world-surfel rule should usually be:

draw enough surfels so projected spacing is around the target pixel spacing
increase surfel diameter only enough to cover holes
compensate opacity by represented density and area ratio

This is why world surfels feel better when looking down the site: they can naturally become a coarser surface representation instead of a pile of huge screen-space circles.

## New artistic renderer: use the same hierarchy

The painted/sketch renderer should not use a separate LOD structure. It should use the same hierarchy, but select representatives differently.

For each node, store a small palette:

spatial coverage representatives
color-contrast representatives
scalar-extreme representatives
emissive representatives
edge/normal-variation representatives
random flavour representatives

Then at runtime:

desiredMarks =
    projectedNodeAreaPx * style.paintDensity;
desiredMarks =
    clamp(desiredMarks, minMarksPerNode, maxMarksPerNode);

Pick the first desiredMarks representatives using a stable blue-noise/farthest-point order:

if (rep.blueNoiseRank < desiredMarks) {
    emit draw item;
}

The draw item becomes:

struct PaintedPointDrawItemGpu {
    uint32_t sourcePointIndex;
    uint32_t nodeIndex;
    float representedCount;
    float spacingMeters;
    float brushSizePxOrMeters;
    float brushOpacity;
    float brushAngle;
    float brushSeed;
    float lodBlend;
    uint32_t brushClass;
};

Then you have two painted variants.

### Painted screen-sprite renderer

This is image-space painting.

Each representative becomes a brush dab:

screen-facing textured quad or point sprite
brush radius in pixels
brush angle from hash/scalar/normal
opacity from style + LOD compensation
color from source RGB/scalar style
jitter from stable seed

This mode is good for:

sketch
pastel
pointillism
watercolour dots
animated stipple
glowing accent particles

But it must have a strict fragment budget, because large soft screen brushes can recreate the original overdraw problem.

For this mode, use:

maxBrushRadiusPreviewPx
maxBrushRadiusExportPx
maxMarksPerTile
maxEstimatedFragments

The artistic advantage is that you can intentionally reduce density and make it look like a style rather than a degraded point cloud.

### Painted world-surfel renderer

This is more surface-like.

Each representative becomes a world-oriented brush stroke:

quad oriented by normal/tangent
size based on LOD spacing
stroke length based on normal/scalar/roughness/flow
opacity compensated by represented surface density

This is better for:

painted surface
charcoal hatching
ink wash
architectural sketch
scan-as-surface look

For surfel painting:

brushDiameterWorld = node.spacingMeters * paintFillFactor;
brushLengthWorld = brushDiameterWorld * brushAspect;

Near the camera, the hierarchy refines. Far away, a few larger strokes represent many raw points.

This should perform much better than trying to draw millions of large translucent screen sprites.

## Parent/child LOD blending

Do not hard-switch between LOD levels.

Each emitted draw item should have:

lodBlend

Then either multiply opacity:

alpha *= lodBlend;
emission *= lodBlend;

or use stochastic dither:

float n = StableBlueNoiseOrHash(pixelCoord, item.randomSeed, frameIndex);
if (n > lodBlend) discard;

For the painted renderer, the stochastic transition can become part of the look: the drawing “repaints” itself as the camera moves.

But keep the randomness stable. Do not use pure per-frame random noise. Use:

hash(nodeId, representativeId, cameraCellId, slowStyleFrame)

where slowStyleFrame changes at maybe 6–12 Hz for live painting, not every GPU frame.

## Export modes should become explicit

Your current export system already has the idea of previewDensity. In Application.cpp, EffectiveAnimationExportPointDrawCount(...) uses preview density to decide whether to use the preview LOD target or full primitives. In VulkanViewportShell.cpp, raster EXR export uses:

forceFullSource = !request.previewDensity && !fastBasicPointRenderer;

That is a good starting point, but I would replace the boolean with an enum.

Something like:

enum class PointCloudExportDensityMode {
    FullSource,
    AdaptiveHighQuality,
    MatchViewportPreview,
    FastPreviewBudget,
    ArtisticAsPreview,
    ArtisticHighQuality
};

Then your render/export UI becomes clearer.

### FullSource

Draw raw source points.
sampleWeight = 1.
No LOD compensation.
User point size/opacity/emission are literal.
Slow but exact.

This is the mode the user expects when they say:

LOD off

However, it is important to understand that this can look more blown out than preview if the preview was using LOD. That is not a bug. It means the preview was not showing all 120M overlapping translucent sprites.

### AdaptiveHighQuality

Use hierarchy.
Very low screen-space error.
Much higher point/fragment budget.
Use visual compensation.
Suitable for EXR/MP4.

This is the mode I would recommend as the default for Beauty exports.

It does not mean “low quality.” It means:

do not spend minutes drawing 1 mm points that collapse below a pixel
do preserve opacity, colour, emission, and scalar features

### MatchViewportPreview

Use the same LOD/visual compensation as the preview.
Useful when the director approved the viewport look.
Fast and predictable.

### FastPreviewBudget

Old preview-density behaviour.
Useful for quick MP4s.

### ArtisticAsPreview

Use the painted/sketch renderer exactly as previewed.
Same seed and same stroke cadence.

### ArtisticHighQuality

Same style, but more marks, lower LOD error, higher resolution brush accumulation, optional temporal accumulation.

## Separate “artist settings” from “renderer compensation”

This is the main design principle I would use.

The user should set:

Point Size
Surfel Diameter
Opacity
Emission
Falloff
Colour/scalar style
Stylisation

Those are artistic settings.

The renderer should internally derive:

actual radius
actual opacity
actual emission
actual representative count
actual LOD blend
actual brush/stroke count

Those are implementation settings.

So instead of changing the user’s point size when LOD changes, add a compensation layer:

struct PointCloudLodCompensationGpu {
    uint32_t enabled;
    float representedCount;
    float rawFootprintArea;
    float lodFootprintArea;
    float lodBlend;
    float opacityCoverageScale;
    float emissionCoverageScale;
    float radiusScale;
};

In shader terms:

float ApplyLodOpacity(float alphaRaw, float coverageScale, float lodBlend) {
    alphaRaw = clamp(alphaRaw, 0.0, 0.999);
    float alphaLod = 1.0 - pow(1.0 - alphaRaw, max(0.0, coverageScale));
    return alphaLod * clamp(lodBlend, 0.0, 1.0);
}

And for emission:

float ApplyLodEmission(float emissionRaw, float coverageScale, float lodBlend) {
    float e = emissionRaw * coverageScale * clamp(lodBlend, 0.0, 1.0);
    return min(e, styleData.maxLodEmissionBoost);
}

You may eventually want the emission clamp exposed as:

Preserve glow energy
Preserve accent brightness
Clamp for preview

because different styles want different behaviour.

## How this maps to your current style system

Your current PointCloudStyleGpu already has useful places for this kind of extension:

pointMeta
renderControl
renderParams0-3
stylisationControl
stylisationParams0-2

But I would not overpack everything into the existing fields forever. I would add a second uniform/storage block for LOD:

layout(set = 0, binding = X, std140) uniform PointCloudLodStyleData {
    uvec4 lodControl;
    vec4 lodParams0;
    vec4 lodParams1;
    vec4 paintParams0;
    vec4 paintParams1;
} lodStyle;

Example:

lodControl.x = lod enabled
lodControl.y = compensation mode
lodControl.z = footprint mode
lodControl.w = artistic mode
lodParams0.x = targetSpacingPx
lodParams0.y = maxRadiusPx
lodParams0.z = maxEmissionBoost
lodParams0.w = fillFactor
paintParams0.x = paintDensity
paintParams0.y = brushAspect
paintParams0.z = strokeJitter
paintParams0.w = temporalBlend

This keeps your current Beauty material logic intact.

## Idle refinement for still camera preview

Your concern about users wanting to stop the camera and see the high-quality result is exactly right.

I would add this behaviour:

While navigating:
    strict point budget
    strict fragment budget
    coarser LOD
    preview compensation
After camera idle for 300 ms:
    progressively refine hierarchy
    reduce target spacing
    increase fragment budget
    update draw item buffer over several frames
When user requests final preview:
    use AdaptiveHighQuality or FullSource depending on setting

You already have:

kPerformanceInteractionHold = 300 ms

so the app already has the conceptual hook for this.

In the viewport overlay, show the state clearly:

Preview: Adaptive LOD, 8.2M reps, representing 120M points
Idle refine: 34M reps, target 0.5 px
Export: Adaptive HQ, target 0.25 px

This avoids the user thinking the renderer is secretly lowering quality.

## Important current pitfall: raycast export

Your raster EXR path has a previewDensity/forceFullSource concept. But the raycast path currently calls:

ResolvePointCloudDrawPlan(layer, true, &plan)

which forces full source. Then PrepareRaycastLayerResources(...) loops over:

resources->cpuPositions[pointIndex]

for pointIndex < drawPointCount.

That means if you want LOD-aware raycast export later, the raycast BVH builder must accept the same draw item list, not just the first N CPU positions.

Eventually it should build from:

drawItems[i].sourcePointIndex
drawItems[i].representedCount
drawItems[i].spacingMeters
computed radius

not from sequential source points.

## Memory strategy

For 120M points, be careful not to create full duplicate LOD attribute arrays.

I would use:

raw source buffers:
    positions, colors, normals, scalar fields
LOD hierarchy:
    compact nodes
representative palette:
    sourcePointIndex + weight + spacing + style metadata
per-frame draw items:
    compact visible reps only

So the hierarchy points back to raw data wherever possible.

Only create aggregate attribute buffers when you truly need them, for example:

mean scalar
min/max scalar
mean normal
normal cone
emissive maximum
dominant class

Even then, store those per representative or per node, not per original point.

## A practical staged implementation

I would implement this in phases.

### Phase 1 — View-dependent LOD draw items, no artistic renderer yet

Add:

PointCloudLodHierarchy
PointCloudDrawItemGpu
LodDrawItemBuffer

Build hierarchy once from offlinePointCloud.

Use CPU traversal first. For one 120M static cloud, CPU traversal over nodes should be manageable if the tree is compact. Emit draw items each frame, upload them to a host-visible or staged GPU buffer.

Add two new shaders:

pointcloud_lod_preview.vert
pointcloud_surfel_lod_preview.vert

Leave your current fast/basic/raw paths alone.

### Phase 2 — LOD compensation

Add:

representedCount
spacingMeters
radiusScale
opacityCoverageScale
emissionCoverageScale
lodBlend

Modify the LOD shaders only.

Do not change the user-facing style yet. Just make adaptive LOD look close to full source.

### Phase 3 — Export density modes

Replace previewDensity with:

PointCloudExportDensityMode

Keep old behaviour as one enum value.

Recommended defaults:

Fast MP4: MatchViewportPreview or FastPreviewBudget
Beauty EXR: AdaptiveHighQuality
Debug/exact: FullSource
Painted renderer: ArtisticHighQuality

### Phase 4 — Painted/sketch renderer

Use the same hierarchy and draw item infrastructure, but a different selection policy and different shaders.

Add:

PointCloudRendererMode::PaintedSketch

or possibly:

PointCloudBeautyVariant::Physical
PointCloudBeautyVariant::Painted
PointCloudBeautyVariant::Sketch

For screen-painted mode:

LOD draw item -> screen brush sprite

For world-painted mode:

LOD draw item -> world brush surfel/stroke

### Phase 5 — GPU selection / tile budgets

Once the CPU version works visually, move traversal/culling/budgeting to compute if needed.

Especially useful later:

frustum culling
node screen-error calculation
draw item compaction
indirect draw generation
tile overdraw budget

But I would not start there. First prove the visual model.


## Suggested UI wording

This is not a LOD on/off feature. 
Instead the user facing model should have the following:
1) Viewport Quality:
    - Fast Basic
    - Beauty Adaptive
    - Beauty Full Source
    - Painted Adaptive

2) Viewport LOD:
    - Auto while moving
    - Always adaptive
    - Full source when idle
    - Full source always

3) Export Density:
    - Full Source
    - Adaptive HQ
    - Match Viewport
    - Fast Preview
    - Artistic HQ

4) Visual Matching:
    - Preserve density/opacity across LOD
    - Preserve literal per-point opacity
    - Preserve emissive accents


For advanced settings:
```
Target projected spacing: 0.25–2.0 px
Max preview sprite radius
Max export sprite radius
Max estimated fragments
Max marks per tile
LOD opacity compensation
LOD emission compensation
```

## Take away message
One concern with implementing a LOD is that they can make export quality go down or create discrepancy between preview and render. This generally happens if it is treated as a blunt point-count reduction.

So this will be be built so that the LOD is:
- view-dependent hierarchy
- style-aware representatives
- opacity/emission compensation
- feature-preserving palettes
- different preview/export quality thresholds

then it becomes more like a rendering acceleration structure, not merely a quality reduction.

For your renderer make three distinct concepts:

1) Preview performance LOD:
    - allowed to deviate slightly to keep interaction smooth
2) Adaptive beauty LOD:
    - intended to visually match full source
3) Painted Adaptive - Artistic stochastic LOD:
    - intentionally uses representative marks as the style

That distinction solves the UX problem. The user can work interactively with LOD, stop the camera to refine, then export either the exact full source or the adaptive/high-quality equivalent. For the painted renderer, LOD is not hidden optimisation — it is the renderer’s actual artistic language.
