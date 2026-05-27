# Adaptive Point Cloud LOD System for Fast Basic and Beauty Rendering

## Purpose

This document describes the target LOD system for a Vulkan renderer running on macOS through MoltenVK. The current assumption is that the renderer has **no active adaptive LOD system** and can currently draw the raw point cloud directly. The aim is not to replace raw rendering with a crude point-count cap. The aim is:

> Draw all detail that can make a visible difference, but no more than the current machine and current renderer can afford this frame.

That principle means the renderer should happily draw 50M, 100M, or 150M points when the current view and GPU can afford it. It should reduce work only where the lost work is visually redundant, hidden, subpixel, overdraw-saturated, or explicitly allowed by an interactive performance mode.

The system is shared by two existing renderers:

```text
1. Fast Basic
   Square solid points, fastest navigation, depth-tested, minimal shader cost.

2. Beauty
   Gaussian circles and/or world surfels, opacity, emissive, falloff, scalar styling,
   depth contribution, EDL and high-quality compositing.
```

The Painted renderer is intentionally separated into a second document. It uses the same cache and hierarchy, but adds neighbour-switching material data and different rules.

---

## Non-goals

This design is **not**:

```text
- a fixed sampled preview target;
- a global distance-only LOD;
- framebuffer pixel downsampling;
- a replacement for full-source export;
- a silent quality reduction system;
- a system that treats point count as the only budget.
```

The ideal viewport remains full-resolution. Far/subpixel regions may be represented by fewer samples, but the final framebuffer should not be downscaled merely to hide cost.

---

## High-level architecture

```text
PLY / source cloud
    -> persistent cache bundle on disk
    -> chunked spatial hierarchy
    -> representative LOD data and per-node statistics
    -> streamed CPU/GPU chunk cache
    -> adaptive selector using measured GPU time and visual error
    -> renderer-specific draw items
    -> Fast Basic or Beauty draw pass
```

The same cache can serve:

```text
- interactive viewport rendering;
- idle refinement when the camera stops;
- EXR still export;
- MP4 export;
- future Painted mode;
- future raycast/BVH export if draw items are supported there too.
```

---

## Core design rule

Every frame follows this order:

```text
1. Select the detail needed for the current view.
2. Estimate cost in vertices, fragments, overdraw, uploads, memory bandwidth, and shader complexity.
3. If cost fits the measured machine budget, draw it.
4. If not, reduce only in the least visible places first.
5. If still not enough, enter an explicit degraded interactive mode.
```

The key is that **quality selection happens before performance reduction**. The renderer should first decide what would be visually sufficient, then decide whether the device can afford it.

---

## Why point count is not enough

A fixed point count cannot describe the cost of a point-cloud renderer.

These are completely different workloads:

```text
50M 1 px square points
50M 2 px hard discs
50M 8 px Gaussian translucent sprites
50M 20 px Gaussian translucent sprites
50M world surfels expanded to 6 vertices each
50M emissive transparent sprites with EDL/depth/stylisation
```

For screen sprites, a rough fragment estimate is:

```cpp
estimatedFragments = pointCount * pi * radiusPx * radiusPx;
```

So:

```text
N points at 2 px radius     ≈ N * 12.6 fragment candidates
N points at 10 px radius    ≈ N * 314 fragment candidates
```

This explains the problem case: a close camera looking down a long dense site can project millions of points into a small region of the screen, and large translucent sprites turn that into an enormous overdraw problem. The correct budget is therefore a combination of:

```text
- selected vertices / representatives;
- estimated fragments;
- estimated blended fragments;
- per-screen-tile overdraw;
- shader variant cost;
- upload/streaming cost;
- measured GPU time.
```

---

## Cache bundle built from PLY

When a PLY is opened for the first time, build a persistent binary cache. On later opens, validate the source file fingerprint and load the cache instead of reparsing the PLY.

Suggested layout:

```text
ExampleSite.ply

PointCloudCache/
    ExampleSite.<fingerprint>.ipcloud/
        manifest.json
        attribute_schema.bin
        hierarchy.bin
        node_pages.bin
        node_stats.bin
        raw_chunks/
            chunk_000000.bin
            chunk_000001.bin
            ...
        lod_representatives.bin
        scalar_stats.bin
        build_status.json
        build_log.txt
```

The cache fingerprint should include:

```text
- cache format version;
- source file size;
- source modified time;
- source header hash;
- source content hash or sampled block hashes;
- attribute schema version;
- build settings version;
- coordinate system / transform settings if these affect stored positions.
```

The manifest should include:

```cpp
struct PointCloudCacheManifest {
    uint32_t cacheFormatVersion;
    uint64_t sourceFileSize;
    uint64_t sourceModifiedTime;
    uint64_t sourceContentHash;

    uint64_t pointCount;
    uint32_t scalarFieldCount;
    bool hasRgb;
    bool hasNormals;

    Bounds3f bounds;
    float estimatedRawSpacingMeters;

    uint32_t hierarchyNodeCount;
    uint32_t leafChunkCount;
    uint32_t maxTreeDepth;

    AttributeSchema attributes;
    BuildSettings buildSettings;
};
```

### First-load behaviour

The first load should not require a long black-screen preprocessing stage. Build progressively:

```text
1. Parse header and show bounds/placeholder.
2. Build coarse root/upper hierarchy first.
3. Render coarse representatives while the full cache is still building.
4. Stream leaf chunks as they become available.
5. Refine the viewport as chunks finish.
6. Mark the cache complete only after all files are atomically written.
```

Build into a temporary directory first:

```text
ExampleSite.<fingerprint>.ipcloud.tmp/
```

Then rename to:

```text
ExampleSite.<fingerprint>.ipcloud/
```

when complete. A partial cache can be resumed if `build_status.json` records completed chunks and hierarchy pages.

### Subsequent-load behaviour

On a valid cache:

```text
1. Load manifest and hierarchy immediately.
2. Load root/upper LOD nodes first.
3. Start rendering coarse cloud.
4. Stream visible raw chunks and deeper LOD data asynchronously.
5. Refine the view while loading.
```

A good user-visible status is:

```text
Loading point cloud hierarchy...
Rendering coarse preview...
Streaming visible chunks: 38 / 412...
Refining visible detail...
```

---

## Spatial hierarchy

Use an octree, loose octree, or Morton-coded bricked hierarchy. Because the example site is long and thin, a strict cubic octree may waste empty space. A Morton-bricked hierarchy or loose octree is usually better.

Each node stores spatial bounds, children, raw chunks, representatives and visual statistics.

```cpp
struct PointCloudLodNode {
    Bounds3f bounds;

    uint32_t firstChild;
    uint32_t childCount;

    uint32_t firstRawChunk;
    uint32_t rawChunkCount;

    uint32_t firstRepresentative;
    uint32_t representativeCount;

    uint32_t representedPointCount;

    float spacingMeters;
    float densityPointsPerM3;

    float colorVariance;
    float normalVariance;
    float scalarVarianceHint;
    float emissiveImportanceHint;

    NormalCone normalCone;
    uint32_t flags;
};
```

A representative is usually a real source point, not only an average. This allows existing colour, normal and scalar-field logic to continue using a source point index.

```cpp
struct PointCloudLodRepresentative {
    uint32_t sourcePointIndex;
    uint32_t nodeIndex;

    float representedCount;
    float spacingMeters;
    float representativeRadiusMeters;

    float lodRank;       // stable continuous LOD / blue-noise rank
    float importance;    // edge, scalar, emissive, contrast, user priority

    uint32_t representativeClass;
    uint32_t flags;
};
```

### Representative classes

Do not build LOD from a single average or a single arbitrary point per voxel. Store a small mix of representative classes:

```text
- spatial coverage representatives;
- colour-contrast representatives;
- normal/edge representatives;
- scalar-min representatives;
- scalar-max representatives;
- scalar-threshold representatives;
- emissive/accent representatives;
- random / blue-noise representatives.
```

This prevents LOD from erasing rare but meaningful data, such as a bright emissive accent, a scalar threshold, or a sharp edge.

---

## Continuous LOD

Discrete node switching can cause visible popping. Give each representative a stable rank:

```text
lodRank in [0, 1]
```

If a node needs 25% of its representatives, draw:

```cpp
representative.lodRank < 0.25f
```

The rank should be ordered so every prefix is spatially useful. Blue-noise, farthest-point, or stratified Morton ordering are suitable. This gives smooth density changes instead of hard jumps.

For parent/child transitions, use either:

```text
- alpha/opacity crossfade; or
- stochastic dithered crossfade using a stable hash.
```

For Beauty, opacity crossfade is usually safer. For Fast Basic, dithered point survival is often cheaper.

---

## Chunk format

Raw chunks should be directly uploadable or cheaply decoded into GPU buffers.

Recommended per-chunk storage:

```text
- local bounds;
- local point count;
- quantized positions relative to local bounds;
- packed RGB/RGBA;
- packed normals if present;
- scalar field blocks;
- source point base/global ID mapping;
- optional compression block footer.
```

For positions:

```text
local quantized 16-bit or 24-bit coordinates + per-chunk scale/offset
```

For normals:

```text
snorm16x3 or octahedral snorm16x2
```

For colours:

```text
RGBA8 or RGB10A2 if needed
```

For scalars:

```text
float32 for exact fields;
float16/quantized for display-only fields;
per-field min/max stats stored separately.
```

Compression should be chosen for fast decode. LZ4 or Zstd-fast are reasonable. The renderer should not wait for an entire 120M point cloud to decompress before drawing visible chunks.

---

## Runtime memory model

### CPU resident data

Keep resident:

```text
- manifest;
- hierarchy nodes;
- node page table;
- attribute schema;
- scalar/global stats;
- chunk residency map;
- streaming job queues;
- small decoded CPU chunk LRU, if needed.
```

Avoid permanently duplicating every position on the CPU once the cache exists, unless editing, picking, or CPU raycasting requires it. Prefer memory-mapped chunks and load CPU data on demand.

### GPU resident data

Keep resident:

```text
- visible and recently visible raw chunks;
- visible and recently visible LOD representatives;
- draw item buffers;
- indirect draw buffers;
- depth/accumulation/EDL attachments;
- per-frame uniform/ring buffers.
```

Large static buffers should normally be uploaded through staging into device-local memory. On Apple GPUs through MoltenVK, this generally maps better than leaving huge static point arrays in host-visible memory.

### Per-frame dynamic buffers

Use ring buffers:

```text
Frame N draw items
Frame N+1 draw items
Frame N+2 draw items
```

This avoids waiting for the GPU before rewriting a buffer.

---

## Draw items

The LOD selector emits draw items. A draw item is the bridge between the hierarchy and the renderer.

```cpp
struct PointCloudDrawItemGpu {
    uint32_t sourcePointIndex;
    uint32_t nodeIndex;

    float representedCount;
    float spacingMeters;

    float rawFootprintArea;
    float lodFootprintArea;

    float opacityCoverageScale;
    float emissionCoverageScale;
    float radiusScale;

    float lodBlend;
    float importance;
    float randomSeed;

    uint32_t representativeClass;
    uint32_t flags;
};
```

The key point is that shaders should not assume `gl_VertexIndex` is the source point index when drawing LOD data. The LOD shader should do:

```glsl
uint drawItemIndex = uint(gl_VertexIndex);
PointCloudDrawItem item = drawItems.items[drawItemIndex];
uint sourcePointIndex = item.sourcePointIndex;
```

Then existing source buffers can be fetched using `sourcePointIndex`:

```glsl
vec3 position = positions[sourcePointIndex].xyz;
uint rgba = colors[sourcePointIndex];
float scalar = scalarFieldValues.values[fieldSlot * pointCount + sourcePointIndex];
```

For surfels, six vertices form one draw item:

```glsl
uint encoded = uint(gl_VertexIndex);
uint drawItemIndex = encoded / 6u;
uint cornerIndex = encoded - drawItemIndex * 6u;
PointCloudDrawItem item = drawItems.items[drawItemIndex];
uint sourcePointIndex = item.sourcePointIndex;
```

Keep the current raw shaders for full-source rendering. Add LOD-aware shader variants rather than forcing the existing raw path to support every mode.

---

## Per-frame pipeline

```text
1. Read camera, style and renderer mode.
2. Read previous-frame GPU timings.
3. Update performance governor.
4. Traverse hierarchy.
5. Compute visual error for candidate nodes.
6. Estimate cost for the current renderer.
7. Select raw chunks or representatives.
8. Request missing chunks from streaming system.
9. Emit draw items.
10. Upload or compute compact draw item buffer.
11. Draw Fast Basic or Beauty pass.
12. Record timings and diagnostics.
```

The first implementation can do traversal on the CPU. Later, move culling, projection, compaction and indirect command generation to compute.

---

## Performance governor

The governor adapts to the current machine by measuring GPU time, not by assuming a fixed point count.

Use previous-frame GPU timestamps for:

```text
- Fast Basic point pass;
- Beauty point/surfel pass;
- depth prepass;
- transparency accumulation;
- EDL/composite;
- upload/streaming stalls, if measurable separately.
```

The governor maintains an exponential moving average:

```cpp
struct PointCloudPerformanceGovernor {
    float targetFrameMs;             // 16.6 for 60 fps, 33.3 for 30 fps
    float targetPointPassMs;         // renderer-specific budget

    float gpuPointPassEwmaMs;
    float gpuCompositeEwmaMs;

    uint64_t maxVertices;
    uint64_t maxEstimatedFragments;
    uint64_t maxEstimatedBlendedFragments;
    uint64_t maxUploadBytesPerFrame;

    float targetSpacingPx;
    float emergencySpacingPx;

    float maxSpriteRadiusPreviewPx;
    float maxSurfelRadiusPreviewPx;

    bool degradedInteractiveMode;
};
```

Adapt slowly:

```cpp
float scale = clamp(targetPointPassMs / max(gpuPointPassEwmaMs, 0.1f), 0.80f, 1.15f);
maxEstimatedFragments = uint64_t(float(maxEstimatedFragments) * scale);
```

Do not let the governor silently violate quality floors in still/export modes. In interactive navigation, if the quality floor cannot be maintained at the target frame rate, enter an explicit mode:

```text
Adaptive preview limited by GPU: fragment budget reached.
```

---

## Quality modes

Use explicit modes instead of a single LOD on/off switch.

```cpp
enum class PointCloudQualityMode {
    FullSourceLiteral,
    AdaptiveHighQuality,
    AdaptiveInteractive,
    MatchViewport,
    FastBasicInteractive
};
```

### FullSourceLiteral

```text
- Draw raw source points/chunks.
- No representative substitution.
- No LOD opacity/emission compensation.
- User point size, opacity and emission are literal.
- Slow but trustworthy.
```

### AdaptiveHighQuality

```text
- Use hierarchy only where it is visually equivalent or below threshold.
- Strict projected-spacing threshold.
- Preserve colour/scalar/normal/emissive features.
- Use opacity/emission/size compensation.
- Default for Beauty EXR/MP4 exports.
```

### AdaptiveInteractive

```text
- Same principle, but allowed to relax quality while the camera is moving.
- Explicitly reports when it is performance-limited.
- Refines again when the camera stops.
```

### MatchViewport

```text
- Export the same selected LOD state/look the user approved in the viewport.
- Important when an art-directed preview must match MP4/EXR output.
```

### FastBasicInteractive

```text
- Navigation-first.
- Square solid points.
- Minimal shader.
- May simplify aggressively, but should say so in diagnostics.
```

---

## Visual error tests

A node should refine if any visible difference is likely.

For each node:

```cpp
float depth = distance(camera, node.bounds);
float pixelsPerMeter = viewportHeight * projection[1][1] / (2.0f * depth);
float spacingPx = node.spacingMeters * pixelsPerMeter;
float screenAreaPx = ProjectBoundsArea(node.bounds, viewProjection, viewport);
```

Refine if:

```text
- projected spacing is above target;
- node covers enough pixels and colour variance is high;
- scalar variance crosses a visible colour-map/threshold boundary;
- normal variance/edge importance is high;
- emissive/accent importance is high;
- the node is close to the camera;
- the node is on a silhouette or depth discontinuity;
- the user is inspecting/selecting this region.
```

Do not refine if:

```text
- node is outside the frustum;
- node is conservatively occluded;
- source spacing is far below pixel visibility and statistics are low variance;
- the tile is already overdraw-saturated and the node is low importance;
- the node contributes only to an already opaque/depth-covered region.
```

---

## Cost model

### Fast Basic square points

Fast Basic is mostly vertex/bandwidth limited until point size grows.

```cpp
vertexCost = representativeCount;
fragmentCost = representativeCount * squarePointSizePx * squarePointSizePx;
shaderCost = cheap;
```

Fast Basic can choose more points than Beauty because it avoids expensive falloff, blending, opacity, emission and EDL work.

### Beauty screen sprites

Screen sprites are usually fragment and overdraw limited.

```cpp
radiusPx = 0.5f * userPointSizePx;
fragmentCost = representativeCount * pi * radiusPx * radiusPx;
blendedCost = fragmentCost * opacityBlendPenalty;
```

If LOD representatives grow to fill holes:

```cpp
lodRadiusPx = clamp(max(rawRadiusPx, spacingPx * fillFactor), minRadiusPx, maxRadiusPxForMode);
fragmentCost = representativeCount * pi * lodRadiusPx * lodRadiusPx;
```

### Beauty world surfels

World surfels cost more vertices but can be visually more stable for dense scan surfaces.

```cpp
radiusWorld = max(userSurfelRadiusWorld, node.spacingMeters * fillFactor);
radiusPx = radiusWorld * pixelsPerMeter;
vertexCost = representativeCount * 6;
fragmentCost = representativeCount * pi * radiusPx * radiusPx;
```

World surfels should be preferred when the user wants a surface-like result and normals are reliable.

---

## Screen-tile overdraw budget

The worst views are not necessarily those with the most visible points. They are often views with many translucent splats in the same screen area.

Maintain a coarse tile grid, such as 16×16 or 32×32 pixels:

```cpp
tileEstimatedFragments[tile] += nodeEstimatedFragments;
tileEstimatedBlendedFragments[tile] += nodeEstimatedBlendedFragments;
```

If a tile exceeds budget, low-priority nodes contributing mostly to that tile should be:

```text
- kept coarser;
- drawn as smaller reps;
- skipped if occluded;
- delayed until idle refinement;
- or moved to explicit degraded preview if still needed.
```

This is especially important on Apple GPUs because large blended overdraw and render-pass bandwidth can dominate performance.

---

## Occlusion and depth

Use multiple levels of occlusion, increasing in sophistication over time:

```text
Phase 1: frustum culling only.
Phase 2: normal-cone/backface culling for surfels where valid.
Phase 3: coarse depth proxy pass.
Phase 4: Hi-Z depth pyramid and node occlusion culling.
Phase 5: static-site visibility hints if camera regions are predictable.
```

For a static scanned site, an approximate proxy can be very effective:

```text
- voxel depth shell;
- coarse surfel depth prepass;
- simplified mesh/TSDF only for occlusion and EDL.
```

Do not require a watertight mesh. Conservative occlusion that rejects only obviously hidden nodes is enough to help the long-grazing-view case.

---

## LOD compensation for Beauty

The user-facing style should remain canonical:

```text
point size
surfel diameter
opacity
emissive strength
falloff
colour/scalar style
```

LOD must not mutate those settings in the UI. Instead, apply an internal compensation layer.

### Opacity compensation

If one representative stands for many source points, use optical-depth style compensation.

```cpp
float coverageScale = representedCount * rawFootprintArea / max(lodFootprintArea, epsilon);
float alphaLod = 1.0f - pow(1.0f - alphaRaw, coverageScale);
```

Equivalent form:

```cpp
float tauRaw = -log(max(1e-5f, 1.0f - alphaRaw));
float tauLod = tauRaw * coverageScale;
float alphaLod = 1.0f - exp(-tauLod);
```

This avoids the common failure modes:

```text
- fewer reps become too transparent;
- larger reps become too opaque;
- emission/opacity explode linearly.
```

### Screen-sprite footprint compensation

For screen sprites:

```cpp
rawRadiusPx = 0.5f * userPointSizePx;
lodRadiusPx = clamp(max(rawRadiusPx, spacingPx * fillFactor), minRadiusPx, maxRadiusPxForMode);

rawArea = pi * rawRadiusPx * rawRadiusPx;
lodArea = pi * lodRadiusPx * lodRadiusPx;
coverageScale = representedCount * rawArea / max(lodArea, epsilon);
```

Modes:

```cpp
enum class ScreenSpriteFootprintMode {
    LiteralUserSize,       // exact raw point-size semantics
    DensityFill,           // grow reps to cover gaps, compensate opacity
    PerformanceClamped     // interactive safety only
};
```

### World-surfel footprint compensation

For world surfels:

```cpp
rawRadiusWorld = 0.5f * userSurfelDiameterWorld;
lodRadiusWorld = max(rawRadiusWorld, node.spacingMeters * surfelFillFactor);

rawArea = pi * rawRadiusWorld * rawRadiusWorld;
lodArea = pi * lodRadiusWorld * lodRadiusWorld;
coverageScale = representedCount * rawArea / max(lodArea, epsilon);
```

This is usually more stable than screen-sprite compensation because the footprint is tied to world-space sample spacing.

### Emission compensation

Emission needs a separate rule from opacity.

```cpp
enum class LodEmissionMode {
    PreserveAverageBrightness,
    PreserveIntegratedEnergy,
    PreserveAccents,
    ClampForPreview
};
```

A practical default:

```cpp
float emissionScale = clamp(coverageScale, 0.0f, maxLodEmissionBoost);
float emissionLod = emissionRaw * emissionScale;
```

But rare emissive features should be preserved as accent representatives instead of averaged into normal representatives.

### Colour and scalar compensation

Colour should be handled through selection before averaging.

```text
If colour/scalar variance is visible:
    refine or draw multiple representatives.

If the node is far/subpixel and low variance:
    weighted mean colour is acceptable.

If scalar thresholds drive style:
    preserve min/max/threshold representatives, not only mean.
```

---

## Fast Basic renderer integration

Fast Basic should remain the guaranteed responsive path.

```text
Primitive: point list
Shape: square solid point
Depth: depth test on, usually depth write on
Transparency: none or minimal
Shader: minimal colour/scalar path
Goal: navigation and playback
```

LOD behaviour:

```text
- Draw raw points when cost is acceptable.
- Use representatives when source density is subpixel, occluded, or performance-limited.
- Avoid expensive compensation except optional point-size growth to hide holes.
- Prefer depth-correctness and responsiveness over exact Beauty matching.
```

Implementation:

```text
- Keep existing raw Fast Basic shader for FullSourceLiteral.
- Add pointcloud_fast_basic_lod.vert for draw-item rendering.
- Use the same fragment shader if possible.
- No geometry shader.
```

LOD vertex pattern:

```glsl
uint itemIndex = uint(gl_VertexIndex);
PointCloudDrawItem item = drawItems.items[itemIndex];
uint sourceIndex = item.sourcePointIndex;

vec3 position = pointPositions[sourceIndex].xyz;
vec4 colour = UnpackColor(pointColors[sourceIndex]);

gl_Position = uniforms.viewProjection * vec4(position, 1.0);
gl_PointSize = max(1.0, fastPointSizePx * item.radiusScale);
```

---

## Beauty renderer integration

Beauty has two geometry paths:

```text
1. Screen sprites
   Camera-facing Gaussian/circle sprites with point size in pixels.

2. World surfels
   Normal-oriented discs/quads with size in metres.
```

### Beauty screen sprites

Use when the user wants:

```text
- Gaussian circles;
- glow/dust/particle style;
- screen-space artistic point size;
- scalar-driven opacity/emissive accents.
```

Risks:

```text
- large translucent sprites are extremely overdraw-heavy;
- exact per-point transparency sorting is impractical for 100M+ points;
- point size is decoupled from real sample spacing.
```

Best practice:

```text
- estimate fragment cost;
- use tile overdraw budgets;
- prefer weighted blended OIT or density accumulation over full sorting;
- apply opacity/emission compensation when reps stand for multiple source points;
- clamp huge preview radii only in explicit interactive mode.
```

### Beauty world surfels

Use when the user wants:

```text
- surface-like scan rendering;
- normals to matter;
- stable world-space footprint;
- better far/grazing behaviour.
```

Best practice:

```text
- expand quads in vertex shader, not geometry shader;
- orient from normal/tangent/bitangent;
- radius follows max(user diameter, LOD spacing * fill factor);
- compensate opacity by represented count and footprint area;
- use normal-cone/backface culling where valid;
- use depth/proxy pass for EDL and occlusion.
```

---

## Transparency and compositing

For Beauty transparency, prefer:

```text
- weighted blended order-independent transparency;
- additive/density accumulation for glow/dust modes;
- optional layered approximation for high-quality export;
- not full sorting of all point sprites.
```

EDL should usually happen after depth has been resolved:

```text
1. depth/proxy pass;
2. point/surfel accumulation pass;
3. transparency resolve;
4. EDL/composite/post;
5. EXR/AOV write.
```

Avoid doing expensive EDL-like work in every translucent fragment.

---

## Idle refinement

Use three runtime states:

```text
Moving:
    strict GPU time budget
    limited upload budget
    AdaptiveInteractive or FastBasicInteractive

Idle:
    progressively refine visible hierarchy
    lower target spacing threshold
    stream missing child chunks
    increase fragment budget

Final/export:
    use explicit export mode
    no interactive frame-time relaxation unless the selected mode says so
```

When the camera stops:

```text
- keep the current image responsive;
- refine selected nodes over multiple frames;
- crossfade parent/child representatives;
- display a small status such as "Refining point cloud detail...".
```

---

## Export modes

Use explicit density/quality modes for EXR/MP4.

```cpp
enum class PointCloudExportDensityMode {
    FullSourceLiteral,
    AdaptiveHighQuality,
    MatchViewport,
    FastPreview
};
```

### FullSourceLiteral

```text
Draw raw source points exactly.
No representative compensation.
Can be very slow.
Useful for trust/debug/final exactness.
```

### AdaptiveHighQuality

```text
Use hierarchy with strict visual thresholds.
Preserve opacity, emission, colour statistics, scalar extrema and accents.
Allow seconds/minutes per frame if requested, but do not waste time on invisible/subpixel redundancy.
Recommended default for Beauty EXR/MP4.
```

### MatchViewport

```text
Use the same selected LOD and settings as the approved viewport preview.
Useful for art direction.
```

### FastPreview

```text
Use navigation/playback settings for quick MP4s.
Clearly labelled as preview quality.
```

For MP4, selection must be deterministic to avoid flicker:

```text
same cache + same camera frame + same seed = same selected representatives
```

---

## Vulkan/MoltenVK implementation constraints

Design for Vulkan, but assume the backend is Metal through MoltenVK.

Recommended:

```text
- storage buffers for chunks, representatives and draw items;
- device-local static buffers with staging uploads;
- compute shaders for culling/compaction when ready;
- indirect draw buffers where supported;
- vertex-expanded surfel quads;
- full-resolution render targets;
- few render passes and careful attachment load/store use;
- pipeline cache for shader conversion/pipeline creation.
```

Avoid relying on:

```text
- geometry shaders;
- per-frame rebuilding of huge CPU index buffers;
- full transparent sorting of 100M+ splats;
- giant host-visible static point buffers as the final architecture;
- high-frequency CPU/GPU synchronization for timing or culling results.
```

Use runtime feature checks for indirect count and related GPU-driven paths. If `vkCmdDrawIndirectCount` is not available or does not behave well on a target machine, fall back to CPU-generated counts and normal indirect/direct draws.

---

## Migration plan from current raw renderer

### Phase 1 — Cache and hierarchy

```text
- Add persistent .ipcloud cache beside PLY loading.
- Build hierarchy and representatives.
- Render coarse hierarchy while building/loading.
- Keep existing raw renderer untouched.
```

### Phase 2 — Draw item path

```text
- Add PointCloudDrawItemGpu.
- Add draw item GPU buffer.
- Add LOD-aware Fast Basic shader.
- CPU traversal emits draw items.
- Verify raw and LOD draw paths produce aligned positions/colours.
```

### Phase 3 — Beauty LOD path

```text
- Add LOD-aware Beauty screen-sprite shader.
- Add LOD-aware surfel shader.
- Add opacity/size/emission compensation.
- Add screen-sprite and surfel cost models.
```

### Phase 4 — Adaptive governor

```text
- Add GPU timestamps.
- Add EWMA timing governor.
- Add estimated fragment and tile budgets.
- Add explicit degraded interactive status.
```

### Phase 5 — Occlusion and GPU-driven selection

```text
- Add depth proxy/Hi-Z pyramid.
- Add compute culling/compaction.
- Add indirect draw command generation.
- Keep CPU fallback.
```

### Phase 6 — Export integration

```text
- Replace boolean preview/full-source assumptions with explicit export density modes.
- Add AdaptiveHighQuality export path.
- Make MP4 selection deterministic.
- Add diagnostics to EXR/MP4 logs.
```

---

## Diagnostics

The viewport should show enough information to build trust:

```text
Renderer: Beauty / Screen Sprites
Quality: Adaptive Interactive
Visible source points represented: 146.2M
Drawn representatives: 38.7M
Estimated fragments: 1.2B
GPU point pass: 18.4 ms
Tile budget: limited in 12% of tiles
Refinement: idle pending / streaming / complete
Mode: visually lossless / performance-limited / full source
```

If the renderer has to deviate visibly, say so:

```text
Adaptive preview is performance-limited: fragment budget reached.
Stop camera to refine, switch to Full Source, or reduce sprite size/opacity.
```

---

## Practical defaults

```text
Fast Basic moving target: 30–60 fps
Beauty moving target: 30 fps minimum, 60 fps when possible
Idle target: refine toward AdaptiveHighQuality
EXR default: AdaptiveHighQuality
Debug/exact export: FullSourceLiteral
MP4 default: AdaptiveHighQuality or MatchViewport
```

Suggested thresholds:

```text
AdaptiveInteractive target spacing: 1.0–2.0 px
AdaptiveHighQuality target spacing: 0.25–0.75 px
FullSourceLiteral: raw chunks only
Screen tile size: 16×16 or 32×32 px
Idle refinement start: 250–500 ms after camera stops
```

These are starting points. The governor should learn actual throughput per renderer and per machine.

---

## References and design influences

- Potree: a large point-cloud renderer based on hierarchical LOD and progressive loading. <https://www.cg.tuwien.ac.at/research/publications/2016/SCHUETZ-2016-POT/>
- COPC: Cloud Optimized Point Cloud, a chunked octree layout with hierarchy entries pointing to directly seekable chunks. <https://copc.io/>
- Layered Point Clouds: multiresolution point clouds rendered coarse-to-fine with view-dependent refinement and out-of-core streaming. <https://diglib.eg.org/items/dbfe6de2-26d5-44d6-b8b0-7d5f79c477c2>
- Continuous LOD for point clouds: gradual density transitions instead of discrete popping. <https://www.cg.tuwien.ac.at/research/publications/2019/schuetz-2019-CLOD/>
- SimLOD: incremental LOD generation and rendering while points are still loading. <https://arxiv.org/abs/2310.03567>
- MoltenVK runtime guide: Vulkan-on-Metal implementation and SPIR-V to MSL conversion/pipeline cache considerations. <https://raw.githubusercontent.com/KhronosGroup/MoltenVK/master/Docs/MoltenVK_Runtime_UserGuide.md>
- Apple Metal guidance for Apple GPUs and tile-based deferred rendering: minimize overdraw and manage pass/attachment traffic carefully. <https://developer.apple.com/videos/play/wwdc2020/10632/>
- Vulkan timestamp queries: GPU timing for measured adaptation. <https://docs.vulkan.org/samples/latest/samples/api/timestamp_queries/README.html>
- Vulkan indirect-count drawing: device-read draw counts for GPU-driven draw submission when supported. <https://docs.vulkan.org/refpages/latest/refpages/source/vkCmdDrawIndirectCount.html>
- Weighted blended order-independent transparency: practical transparency approximation without sorting every primitive. <https://jcgt.org/published/0002/02/09/>
