# PDR.md — Product Design Requirements

## 1. Purpose
Define the functional, UX, and technical requirements for a desktop application that renders point clouds and Gaussian splats in the same scene, supports cinematic shot authoring, and exposes artist-facing field-driven styling through a slide-out side panel.

## 2. Product Goals
### 2.1 Primary goals
- Render large point clouds and Gaussian splats together.
- Support stylised point-cloud rendering driven by scalar fields authored in CloudCompare.
- Provide a side panel for live editing of render and style parameters.
- Allow each eligible parameter to be edited as either a constant or a field-driven mapping.
- Support smooth camera shot interpolation for exhibition rendering.
- Prioritise responsive iteration on Apple Silicon M1.
- Support higher-end offline rendering on Windows machines if desired.

### 2.2 Secondary goals
- Preserve project portability between macOS and Windows.
- Export standard AOVs and selected point-cloud data passes.
- Support reusable style presets and shot-based workflows.

## 3. User Profile
Primary user:
- technically proficient visual researcher / designer,
- comfortable using CloudCompare for preprocessing,
- wants to artistically interpret point clouds using scalar fields,
- needs both exploratory navigation and controlled shot authoring,
- wants M1-friendly look-development and optional stronger offline rendering elsewhere.

## 4. Version 1 Scope
### In scope
- C++ desktop application.
- Vulkan rendering, using MoltenVK on macOS.
- Integration of a GS subsystem based on `3dgs-vulkan-cpp` or equivalent.
- PLY point-cloud import.
- Scalar-field discovery and statistics.
- Point-cloud styling driven by constants or scalar-field mappings.
- Slide-out / pinnable side panel.
- Saved style presets.
- Saved camera shots and interpolated shot playback.
- Interactive preview at approximately 1080p.
- Offline render at 8K, including tiled render path.
- Standard AOV output plus selected field-based passes.

### Out of scope
- Native E57 import in v1.
- Native LAS/LAZ import in v1.
- Full node-based material graph.
- Physics simulation.
- Distributed farm management beyond command-line batch rendering.
- Full nonlinear sequence editor.

## 5. Functional Requirements
### 5.1 File IO
#### FR-IO-1
The application shall import point clouds from binary PLY.

#### FR-IO-2
The application shall read point positions and colours when present.

#### FR-IO-3
The application shall discover and expose scalar fields from the imported point cloud.

#### FR-IO-4
The application shall import one or more Gaussian splat assets via the GS subsystem.

#### FR-IO-5
The application shall save scene, shot, style, side-panel, and render configuration to disk.

### 5.2 Scene and Layer Model
#### FR-SCENE-1
The application shall support multiple point-cloud layers.

#### FR-SCENE-2
The application shall support multiple Gaussian splat layers.

#### FR-SCENE-3
Each layer shall support visibility, transform, naming, and selection.

#### FR-SCENE-4
Point-cloud layers shall support static and dynamic motion modes.

#### FR-SCENE-5
Point-cloud layers shall preserve rest position for non-destructive procedural motion.

### 5.3 Navigation and Camera
#### FR-CAM-1
The application shall support orbit, pan, dolly, and free-fly navigation.

#### FR-CAM-2
The application shall support explicit look-at / target cameras.

#### FR-CAM-3
The application shall support surface-inferred pivots from screen centre or cursor-based picking.

#### FR-CAM-4
Surface-inferred pivot selection shall estimate a stable target from the closest valid visible points when available.

#### FR-CAM-5
If no stable pivot is found, the system shall fall back to current pivot, object centre, or scene centre.

#### FR-CAM-6
The application shall allow the user to save named shots.

#### FR-CAM-7
Shot rotation shall be stored and interpolated using quaternions.

#### FR-CAM-8
The default playback and render timebase shall be 30 fps.

#### FR-CAM-9
The shot system should support both free-orientation shots and target-driven shots.

### 5.4 Side Panel Behaviour
#### FR-UI-1
The application shall include a side panel for render and style controls.

#### FR-UI-2
The side panel shall reveal when the mouse enters an activation strip near its edge.

#### FR-UI-3
The side panel shall remain visible while hovered.

#### FR-UI-4
The side panel shall pin open when double-clicked on its tab, title area, or equivalent interaction target.

#### FR-UI-5
The side panel shall unpin when double-clicked again or explicitly closed.

#### FR-UI-6
The side panel state shall be serializable in the project file.

#### FR-UI-7
The side panel shall remain usable during navigation and playback when pinned.

### 5.5 Side Panel Sections
#### FR-UI-8
The side panel shall expose sections for:
- Scene
- Layers
- Camera
- Shot
- Style
- Field Mapping
- Motion
- Preview
- Render
- Output / AOV
- Presets

#### FR-UI-9
The side panel shall expose both global render controls and per-layer overrides.

#### FR-UI-10
The side panel shall provide a reusable parameter editor for all field-drivable values.

### 5.6 Parameter Source Modes
#### FR-PARAM-1
Every eligible numeric or vector render parameter shall support `Constant` mode.

#### FR-PARAM-2
Every eligible parameter shall support `Field-Mapped` mode when the active layer exposes scalar fields.

#### FR-PARAM-3
Field-Mapped mode shall allow selection of a scalar field source.

#### FR-PARAM-4
Field-Mapped mode shall allow independent input range and output range editing.

#### FR-PARAM-5
Field-Mapped mode shall support clamp and invert toggles.

#### FR-PARAM-6
Field-Mapped mode should support gamma or curve shaping.

#### FR-PARAM-7
The renderer shall evaluate field mappings on GPU when feasible.

#### FR-PARAM-8
The UI shall show field statistics such as discovered min and max values.

#### FR-PARAM-9
The UI should provide reset-to-constant and quick-normalise actions.

### 5.7 Point-Cloud Styling
#### FR-STYLE-1
The renderer shall support at least the following point style parameters:
- point size,
- opacity,
- colour control,
- emissive intensity,
- X-ray strength,
- depth fade,
- additive weight,
- jitter amplitude,
- vibration amplitude,
- vibration frequency,
- phase offset.

#### FR-STYLE-2
Point size shall support screen-space and world-space modes.

#### FR-STYLE-3
Colour shall support source colour, constant colour, field ramp, and source-colour modulation.

#### FR-STYLE-4
Opacity shall support constant and field-driven control.

#### FR-STYLE-5
The renderer should support multiple visual modes including solid dots, discs, X-ray, additive accents, and depth-cued looks.

#### FR-STYLE-6
Styles shall be saveable and reusable as presets.

### 5.8 Procedural Motion
#### FR-MOTION-1
Dynamic point motion shall be procedural and non-destructive.

#### FR-MOTION-2
Motion shall be evaluated from rest position, time, and selected scalar-field mappings.

#### FR-MOTION-3
Motion controls shall include amplitude and frequency at minimum.

#### FR-MOTION-4
Amplitude and frequency shall each support constant and field-mapped control.

#### FR-MOTION-5
The motion system should support phase variation derived from point index, position hash, or scalar fields.

#### FR-MOTION-6
The motion system should support subtle vibration or wobble without requiring full geometry rebuild each frame.

### 5.9 AOV and Data Output
#### FR-AOV-1
The renderer shall output a beauty pass.

#### FR-AOV-2
The renderer shall output depth and alpha.

#### FR-AOV-3
The renderer should support layer ID and world position passes.

#### FR-AOV-4
The renderer should support motion vectors when feasible.

#### FR-AOV-5
The renderer shall allow selected scalar fields to be exported as additional passes.

#### FR-AOV-6
The renderer shall allow export of mapped style results such as final point size factor or final opacity factor when feasible.

### 5.10 Render Output
#### FR-RENDER-1
The renderer shall support interactive preview at approximately 1080p.

#### FR-RENDER-2
The renderer shall support offline rendering at 8K.

#### FR-RENDER-3
The renderer shall support tiled rendering for large outputs or constrained VRAM.

#### FR-RENDER-4
The renderer shall support image sequence output.

#### FR-RENDER-5
The renderer should support headless or command-line render invocation.

## 6. Non-Functional Requirements
### 6.1 Performance
#### NFR-PERF-1
The application shall prioritise interactive response on Apple Silicon M1 during look-development.

#### NFR-PERF-2
The application should maintain comfortable navigation on representative scene subsets at 1080p.

#### NFR-PERF-3
Parameter edits in the side panel should propagate quickly enough to feel interactive.

#### NFR-PERF-4
Offline rendering may be slower but shall preserve visual consistency with preview where practical.

### 6.2 Portability
#### NFR-PORT-1
The project shall build on macOS Apple Silicon.

#### NFR-PORT-2
The project should build on Windows for stronger render hardware.

#### NFR-PORT-3
Project files shall remain portable across supported platforms.

### 6.3 Reliability
#### NFR-REL-1
Missing scalar fields shall not crash the renderer.

#### NFR-REL-2
Corrupt or unsupported point attributes shall produce actionable errors.

#### NFR-REL-3
Render queue failures shall identify frame, pass, and shot context.

## 7. UI Requirements
### 7.1 General UI stack
- immediate-mode UI is acceptable and preferred for fast tooling development,
- the UI must not block camera navigation,
- panel state must be serializable per project.

### 7.2 Side panel sections
Recommended sections:
- Scene
- Layer
- Camera
- Shot
- Style
- Field Mapping
- Motion
- Preview Quality
- Render Output
- Output / AOV
- Presets

### 7.3 Parameter editor requirements
Each reusable parameter editor shall expose:
- source mode,
- constant value control,
- field selection,
- discovered field min/max display,
- editable input min/max,
- editable output min/max,
- clamp,
- invert,
- gamma or remap curve,
- reset to constant.

## 8. Data Model Requirements
### 8.1 Point-cloud attribute model
Each point should be able to expose:
- position,
- colour,
- point index,
- rest position,
- any imported scalar field,
- derived normalized fields when generated.

### 8.2 Render parameter binding model
A render parameter binding shall store:
- parameter name,
- source mode,
- constant value if used,
- field name or field index if used,
- input range,
- output range,
- remap options,
- enabled flag.

### 8.3 Style preset model
A style preset shall store:
- visual mode,
- parameter bindings,
- colour ramp references,
- X-ray and depth settings,
- motion settings,
- preview flags.

## 9. Acceptance Criteria
### AC-1
A CloudCompare-exported PLY with scalar fields can be loaded and the fields are visible in the UI.

### AC-2
A point-cloud parameter such as point size can be switched between Constant and Field-Mapped mode.

### AC-3
A user can map a field such as roughness from a source range to an output size range.

### AC-4
The side panel reveals on hover-near-edge and can be pinned or unpinned by double-click.

### AC-5
A user can create at least two shots and interpolate smoothly between them.

### AC-6
A user can render a shot sequence with beauty, depth, and at least one selected scalar-field pass.

### AC-7
A dynamic layer can apply subtle field-driven vibration without changing source data.

## 10. Risks
- 8K rendering may exceed practical M1 memory/performance without tiling.
- GS subsystem integration may constrain some custom shading paths.
- Very large point clouds may require careful paging, chunking, and point-budget design.
- Motion vectors for stylised point and splat rendering may be non-trivial.
- UI complexity may grow if the parameter-binding system becomes inconsistent.

## 11. Future Extensions
- native E57/LAS/LAZ import,
- node or graph editor for style logic,
- curve editors and histograms for field remapping,
- per-shot style overrides,
- render-farm orchestration,
- additional camera rigs,
- volumetric passes and fog.
