# Project Description — Field-Driven Point Cloud + Gaussian Splat Renderer

## Working Title
Field-Driven Point + Splat Renderer

## Summary
This project is a desktop renderer and shot-authoring tool for combining very large point clouds and Gaussian splats in one 3D scene. It is designed for exhibition image-sequence production, with fast look-development on Apple Silicon M1 and higher-end offline rendering on stronger Windows GPUs when required.

The renderer is not just a viewer. It is an authoring environment for stylised interpretation of point-cloud data.

It must let the user:
- load CloudCompare-exported point clouds as binary PLY,
- load Gaussian splat assets in the same world space,
- navigate the scene like CloudCompare,
- define camera shots and interpolate between them smoothly,
- render preview shots interactively at 1080p on M1,
- render final shots at 8K for downsampled 4K exhibition output,
- drive visual styling and subtle motion from static scalar fields authored in CloudCompare.

## Core Creative Goal
Treat point clouds as expressive visual media rather than neutral survey data.

Static scalar fields such as roughness, height, curvature, density proxies, classification, or custom derived metrics should be able to control how points look and move. The renderer should make it easy to turn these data fields into visual decisions.

Examples:
- map `roughness` to point size,
- map `height` to colour ramp,
- map `classification` to opacity bands,
- map `density` to emissive strength,
- map `curvature` to vibration amplitude,
- map `roughness` to vibration frequency.

## Core Technical Direction
- **Language:** C++
- **Graphics API:** Vulkan
- **macOS path:** MoltenVK on Apple Silicon M1
- **GS subsystem:** integrate `3dgs-vulkan-cpp` or a close derivative rather than flattening splats into plain points
- **Point cloud renderer:** custom Vulkan renderer with explicit support for scalar-field-driven styling
- **UI:** immediate-mode side panel tooling suitable for rapid iteration

## Primary Use Case
A user loads:
- a large point cloud exported from CloudCompare as binary PLY,
- one or more scalar fields already computed in CloudCompare,
- a Gaussian splat aligned into the same coordinate system,
- one or more saved camera shots,
- one or more style presets.

The user then:
- explores the scene interactively,
- edits visual styling through a side panel,
- switches any editable parameter between constant and field-driven control,
- saves camera poses and interpolated shot segments,
- previews results at 1080p,
- renders final 8K image sequences or tiled 8K output.

## Rendering Strategy
### Development / Look-Development
During design on M1:
- interactive preview at 1080p is sufficient,
- preview quality can be adaptive,
- motion and styling changes should feel immediate,
- decimation, point budgets, and lower-cost shader paths are acceptable.

### Final Output
For final exhibition output:
- render high-resolution stills or sequences at 8K,
- allow tiled rendering to manage VRAM limits,
- keep output deterministic,
- preserve project portability so the same scene can be batch rendered on stronger Windows GPUs.

The intended workflow is to render larger than delivery size, then downsample and finish in Houdini or Adobe postproduction tools.

## Input Constraints
### Version 1 input
- Binary PLY point clouds
- Scalar fields stored in or associated with the PLY
- Gaussian splat assets supported by the GS subsystem

### Not required in version 1
- native E57 import
- native LAS/LAZ import

CloudCompare remains the main preprocessing and scalar-field authoring tool for v1.

## Output Requirements
- Interactive preview viewport
- Offline still and sequence rendering
- Beauty pass
- Standard AOVs
- Optional scalar-field AOVs
- Optional mapped-style AOVs
- Project file containing shots, layers, styles, and render settings

## Scene Layer Model
The scene supports multiple layers.

### 1. Static Point Cloud Layers
These contain millions to hundreds of millions of points and remain fixed in world space.

They support:
- scalar-field-driven appearance,
- point budgets,
- chunking or paging,
- culling,
- screen-space and world-space sizing,
- AOV export,
- optional subtle procedural motion evaluated from rest position.

### 2. Dynamic Point Cloud Layers
These are smaller or selectively animated subsets.

They support:
- vibration,
- wobble,
- drift,
- pulse,
- field-driven opacity/colour/size changes,
- motion evaluated procedurally from rest position rather than destructive edits.

### 3. Gaussian Splat Layers
These remain true Gaussian splats and render in the same camera and shot system as the point clouds.

### 4. Helper Layers
Optional overlays:
- grids,
- bounds,
- pivot markers,
- shot paths,
- camera frustums,
- target markers,
- debug field overlays.

## Navigation and Camera System
The application should feel familiar to a CloudCompare user while also supporting shot authoring.

### Navigation modes
- orbit
- pan
- dolly / zoom
- free-fly
- explicit look-at / target camera
- orbit around inferred surface target

### Surface-inferred targeting
The user should be able to orbit around what they are looking at.

Expected behaviour:
- cast a ray from screen centre or mouse position,
- find the nearest visible points,
- optionally average a small cluster of the closest valid hits for stability,
- use that position as orbit pivot,
- if no reliable point is found, fall back to current pivot, object centre, or scene centre,
- zoom should feel centred around the inferred target rather than blindly pushing camera forward.

### Shot system
The user should be able to save and interpolate camera shots.

Shot data includes:
- camera position,
- orientation quaternion,
- optional target point,
- FOV,
- clip planes,
- easing or interpolation settings,
- timing,
- optional per-shot style/render overrides.

Default frame rate: **30 fps**.

## Side Panel UI
The main artist-facing control surface is a side panel.

### Panel behaviour
- hidden by default at screen edge,
- reveals when the mouse approaches the edge,
- remains visible while hovered,
- double-click pins it open,
- double-click again unpins it,
- optional hotkey toggles visibility,
- pinned state is saved per project.

### Panel purpose
The side panel controls:
- scene and layer visibility,
- camera settings,
- shot management,
- point-cloud visual styling,
- scalar-field bindings,
- motion settings,
- preview quality,
- render settings,
- AOV selection,
- preset save/load.

## Parameter Binding Model
Every editable visual parameter should support at least two source modes.

### 1. Constant mode
A fixed user-entered value.

Examples:
- point size = 2.5 px
- opacity = 0.7
- vibration amplitude = 0.03

### 2. Field-mapped mode
A scalar field from the point cloud drives the parameter.

The user can set:
- field source,
- input minimum,
- input maximum,
- output minimum,
- output maximum,
- clamp,
- invert,
- gamma or curve shaping,
- optional blend with a constant.

Example:
- parameter: point size
- source field: `roughness`
- input range: `-10.0 .. 125.0`
- output range: `0.5 .. 6.0 px`
- clamp: on
- invert: off

This allows the smallest roughness values to appear as small dots and the highest roughness values to appear as large dots.

## Parameters That Must Be Mappable
At minimum, point-cloud parameters should support constant and field-mapped control for:
- point size,
- opacity,
- colour ramp position,
- emissive strength,
- X-ray strength,
- depth fade,
- jitter amplitude,
- vibration amplitude,
- vibration frequency,
- phase offset,
- additive blend weight,
- world-space scale multiplier,
- screen-space scale multiplier.

## Visual Styles
Expected visual looks include:
- solid dots,
- circular discs,
- small-to-large dots by field,
- colour-by-field,
- opacity-by-field,
- X-ray / semi-transparent look,
- additive glow / emissive accents,
- high-contrast silhouette modes,
- monochrome with selective highlights,
- subtle procedural motion,
- depth-based attenuation or fog.

The style system should be extensible rather than hard-coded to a single look.

## Procedural Motion
Motion should be lightweight, deterministic, and evaluated from rest position.

Examples:
- points vibrate more when roughness is high,
- points pulse faster when density is high,
- points wobble within a bounded radius derived from a field,
- points offset along pseudo-random directions seeded by point ID.

The goal is subtle motion that preserves the point-cloud character and remains affordable during look-development.

## AOV / Data Output Strategy
The renderer should output standard compositing passes and optional data-rich passes.

### Standard AOVs
- beauty,
- alpha,
- depth,
- layer or object ID,
- world position when feasible,
- motion vectors when feasible.

### Point-cloud data passes
The user should be able to export selected scalar fields and selected mapped results, such as:
- roughness,
- height,
- classification,
- density,
- mapped point size,
- mapped opacity,
- mapped emissive strength,
- mapped motion amplitude.

These outputs allow Houdini or Adobe post workflows to retain access to point-cloud data after rendering.

## Performance Philosophy
### On M1
Prioritise:
- responsiveness,
- fast style iteration,
- smooth navigation,
- practical 1080p preview.

### On stronger Windows GPUs
Prioritise:
- final quality,
- higher point budgets,
- full AOV output,
- high-resolution sequence rendering,
- batch or command-line render support.

## Success Criteria
The project is successful when the user can:
- load a CloudCompare-exported PLY with scalar fields,
- load a Gaussian splat in the same space,
- navigate comfortably,
- edit styles from a side panel,
- switch parameters between constant and field-mapped control,
- save and interpolate shots smoothly,
- preview on M1,
- render export-quality sequences for exhibition postproduction.

## Version 1 Out of Scope
- full DCC parity with Blender/Houdini/Unreal,
- native E57/LAS/LAZ pipeline,
- node graph material editor,
- physics simulation,
- deep Adobe or Houdini integration beyond exported passes and sequences,
- farm orchestration beyond command-line batch render.
