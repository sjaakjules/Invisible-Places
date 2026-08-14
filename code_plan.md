# code_plan.md — Implementation Plan

## 1. Technical Direction
Build a C++ desktop application around Vulkan with a modular architecture that keeps Gaussian splat rendering and point-cloud rendering as separate but coordinated subsystems.

Target priorities:
- **macOS Apple Silicon M1** for look-development,
- **Windows** for optional offline render acceleration,
- **shared scene package** for portability.

Recommended implementation approach:
- Vulkan renderer abstraction.
- MoltenVK for macOS.
- GS subsystem wrapped from `3dgs-vulkan-cpp` or a close derivative.
- Custom point-cloud renderer with explicit support for scalar-field-driven styling.
- Dear ImGui or similar immediate-mode UI for the side panel and tooling.

## 2. Top-Level Architecture
Suggested code layout:

```text
/src
  /app
  /camera
  /io
  /motion
  /output
  /platform
  /renderer
    /core
    /gsplat
    /pointcloud
  /scene
  /serialization
  /style
  /ui
/tests
/shaders
/docs
/Data
/Saved
```

Root-level `/shaders` is the active shader source location. Add `/src/renderer/passes`, `/src/rendergraph`, `/src/jobs`, or `/assets` only when a concrete implementation needs them.

## 3. Module Breakdown
### 3.1 app
Responsibilities:
- application shell lifecycle,
- main loop,
- subsystem init/shutdown,
- project open/save,
- playback state,
- frame orchestration.

Key classes:
- `Application`
- `AppConfig`
- `FrameContext`
- `ProjectSession`

### 3.2 platform
Responsibilities:
- window creation,
- input,
- timing,
- monitor and DPI info,
- platform-specific paths.

Key classes:
- `Window`
- `InputState`
- `TimeState`

### 3.3 renderer/core
Responsibilities:
- Vulkan instance/device/swapchain,
- command buffers,
- synchronisation,
- descriptor management,
- transient resource pools,
- render target setup,
- tiled offline render support.

Key classes:
- `VulkanContext`
- `RenderDevice`
- `FrameResources`
- `DescriptorAllocator`
- `RenderTargetPool`

### 3.4 renderer/pointcloud
Responsibilities:
- point-cloud buffers,
- chunking / paging,
- draw submission,
- style evaluation,
- motion evaluation,
- field-driven shader parameter binding,
- culling / point budget / LOD.

Key classes:
- `PointCloudLayer`
- `PointChunk`
- `PointCloudRenderer`
- `PointStyleEvaluator`
- `PointMotionEvaluator`
- `FieldBindingRuntime`

### 3.5 renderer/gsplat
Responsibilities:
- wrap GS subsystem,
- synchronise camera and transforms,
- expose visibility and output-pass hooks,
- integrate with project serialization.

Key classes:
- `GsplatLayer`
- `GsplatRendererAdapter`

### 3.6 scene
Responsibilities:
- layer registry,
- transforms,
- bounds,
- selection state,
- helper overlays,
- scene statistics.

Key classes:
- `Scene`
- `SceneLayer`
- `LayerHandle`
- `Bounds3D`

### 3.7 camera
Responsibilities:
- viewport navigation,
- orbit and free-fly controls,
- explicit target camera handling,
- surface-inferred pivot resolution,
- shot interpolation.

Key classes:
- `CameraState`
- `CameraController`
- `OrbitPivotResolver`
- `Shot`
- `ShotTrack`

### 3.8 ui
Responsibilities:
- side panel state machine,
- scene controls,
- style controls,
- parameter binding editor,
- shot editor,
- layer inspector.

Key classes:
- `UiManager`
- `SidePanelController`
- `SidePanelState`
- `RenderSidePanel`
- `ParameterBindingWidget`
- `StyleEditorPanel`
- `LayerPanel`

### 3.9 io
Responsibilities:
- PLY import,
- scalar-field parsing,
- project import/export,
- image sequence output writing.

Key classes:
- `PlyImporter`
- `PointAttributeLayout`
- `ProjectSerializer`
- `ImageSequenceWriter`

### 3.10 style
Responsibilities:
- style preset definition,
- parameter binding definitions,
- colour ramps,
- style serialization,
- runtime conversion to GPU-friendly structures.

Key classes:
- `StylePreset`
- `RenderParameterBinding`
- `FieldMapConfig`
- `ColorRamp`
- `StylePresetLibrary`

### 3.11 motion
Responsibilities:
- procedural point motion model,
- GPU parameter packing,
- noise/hash phase generation,
- preview-safe motion evaluation.

Key classes:
- `MotionProfile`
- `MotionBinding`
- `MotionRuntimeParams`

### 3.12 output
Responsibilities:
- render orchestration,
- AOV selection,
- tiled render assembly,
- pass writing,
- metadata sidecar export.

Key classes:
- `RenderJob`
- `RenderQueue`
- `AovOutputSpec`
- `TileAssembler`

## 4. Project File Format
Use JSON or binary+JSON hybrid storage.

The project file should store:
- scene layers,
- file paths,
- transforms,
- point-cloud field metadata,
- style presets,
- render parameter bindings,
- motion settings,
- side panel state,
- saved shots,
- LiDAR-file associations for saved shots and project-registered animations,
- render presets,
- selected AOVs.

Suggested root structure:

```text
/project_name
  project.json
  /shots
  /styles
  /cache
  /renders
```

The implemented authored-workspace layer keeps this logical structure
relocatable with `@data`, `@workspace`, and `@local-renders` path tokens. The
workspace root can come from `INVISIBLE_PLACES_WORKSPACE_DIR` or the ignored
local `.invisible_places-workspace` marker. A distinct production source root
comes from `INVISIBLE_PLACES_SHARED_DATA_DIR` or
`.invisible_places-data-workspace`; it may be cloud-backed and currently holds
only Scene3 1 mm/5 mm ROCK/SAND/VEG plus MESH/MESHSampled PLYs. Field, water,
Flow, and histogram caches are redirected to local `Saved/`, alongside renders,
render history, validation, and build products. Water cache source selection is
explicitly 2 mm first and complete 1 mm second, so a new machine with only the
shared subset cold-builds the same 10 mm representation locally instead of
requiring unsynchronized 2 mm files. Project/animation saves keep a
content fingerprint from load, verify it before and after staging, refuse stale
OneDrive overwrites, and copy mid-save conflicts to local recovery. The Project
panel writes both local markers for the next launch, avoiding a destructive
live context switch.

## 5. Point Attribute Strategy
### 5.1 CPU-side canonical layout
Each point record should expose at minimum:
- `position : float3`
- `color : packed rgba or float4`
- `rest_position : float3`
- `point_id : uint32`
- `scalar_fields[] : float`

### 5.2 GPU-side strategy
Avoid excessively fat vertex records.

Recommended approach:
- static position and colour buffer,
- separate storage buffer for scalar fields,
- per-layer metadata buffer with field lookup offsets,
- optional compact or normalised field buffers for common fields.

### 5.3 Scalar-field indexing
Maintain stable mappings:
- `field name -> field slot index`
- `field slot index -> min/max/statistics`

This allows the UI and shaders to resolve bindings consistently.

## 6. Render Parameter Binding System
### 6.1 Core concept
Every stylable parameter resolves from a generic source binding.

Suggested enum:

```cpp
enum class ParameterSourceMode {
    Constant,
    FieldMapped,
    // future:
    Expression,
    NoiseBlend
};
```

### 6.2 Binding structures

```cpp
struct FieldMapConfig {
    uint32_t fieldIndex;
    float inputMin;
    float inputMax;
    float outputMin;
    float outputMax;
    float gamma;
    float bias;
    uint32_t flags; // clamp, invert, useLayerStats, etc.
};

struct RenderParameterBinding {
    ParameterSourceMode mode;
    float constantValue[4];
    FieldMapConfig fieldMap;
};
```

### 6.3 Parameters that must use the shared binding model
At minimum:
- point size,
- opacity,
- emissive intensity,
- X-ray strength,
- depth fade,
- jitter amplitude,
- vibration amplitude,
- vibration frequency,
- additive weight,
- colour-ramp position.

### 6.4 GPU evaluation helper
All shaders should evaluate field mappings through shared utility code:
1. fetch source value,
2. normalise from input range,
3. optionally invert,
4. clamp if enabled,
5. apply bias/gamma/remap,
6. scale to output range.

Keep this centralised to avoid inconsistent behaviour between parameters.

## 7. Side Panel UI Implementation
### 7.1 UI framework
Use Dear ImGui or an equivalent immediate-mode toolkit.

### 7.2 Side panel state machine
Implement a dedicated controller with states:
- `Collapsed`
- `RevealOnHover`
- `Expanded`
- `Pinned`
- `AnimatingIn`
- `AnimatingOut`

Rules:
- when the mouse enters an edge activation strip, transition to reveal,
- while hovered, keep panel expanded,
- on double-click of the tab/title region, toggle pinned state,
- when not pinned and mouse leaves, collapse after a short delay,
- hotkey toggles open/closed.

### 7.3 Side panel sections
Implement collapsible groups:
- Scene
- Layers
- Camera
- Shot
- Style
- Field Mapping
- Motion
- Preview Quality
- Render Output
- Output / AOV
- Presets

### 7.4 Reusable parameter-binding widget
Implement one widget used by every mappable parameter.

Widget features:
- `Constant` or `Field-Mapped` mode selector,
- constant numeric or colour control,
- field dropdown,
- discovered field min/max display,
- editable input min/max,
- editable output min/max,
- clamp toggle,
- invert toggle,
- gamma slider,
- reset to constant,
- copy/paste mapping settings.

This widget is the main scaling mechanism for the style system.

Water feature sliders use a scoped wrapper around Dear ImGui's native slider:
double-clicking the bar requests its numeric input mode, exact `0..1` ranges
render with three decimals and `NoRoundToFormat`, and the underlying float is
passed unchanged to Water key storage and interpolation. Existing Water drag
and ranged controls retain their native/custom double-click numeric entry.

### 7.5 Layer override model
Each layer should expose:
- base style preset,
- per-parameter overrides,
- preview-only overrides,
- optional shot-level override hooks.

The UI should clearly show whether a parameter is:
- inherited,
- locally overridden,
- field-mapped,
- animated or motion-linked.

## 8. Camera and Shot System
### 8.1 Camera representation
Store:
- position,
- orientation quaternion,
- optional target point,
- FOV,
- near/far,
- navigation mode metadata.

### 8.2 OrbitPivotResolver
Implement GPU-assisted or CPU-assisted pivot inference.

Recommended algorithm for v1:
1. cast from screen centre or cursor into the scene,
2. read depth or point-ID buffer at the hit pixel,
3. sample a small neighbourhood,
4. reconstruct valid world positions,
5. average nearest valid samples for a stable pivot,
6. fall back if invalid.

This avoids expensive full-scene nearest-point queries during interaction.

### 8.3 Shot interpolation
- position: linear or spline,
- orientation: quaternion slerp,
- target point: linear or spline,
- FOV: linear,
- optional easing later.

## 9. Point Rendering Strategy
### 9.1 Render modes
Implement separate render paths or shader branches for:
- solid point discs,
- simple dots,
- X-ray or depth-attenuated points,
- emissive or additive style.

### 9.2 Size modes
- screen-space size,
- world-space size,
- field-driven multiplier.

### 9.3 Colour modes
- source colour,
- constant colour,
- field ramp,
- source colour multiplied by field result.

### 9.4 X-ray mode
Possible implementation options:
- depth softening,
- attenuated alpha through depth,
- additive or screened blend,
- edge emphasis by depth discontinuity.

Keep v1 simple and predictable.

### 9.5 Timed scalar-effect stack
- Store Colourise and Emissive as two immutable kinds in one ordered timing
  effect list, with shared enable and inclusive activation-range state.
- Reuse scalar selector, bounds, signed edge fade, smooth key evaluation, and
  derived activation-boundary sampling for both kinds. Emissive is scalar-only
  and owns a non-negative level track; Colourise owns palette/phase/amount
  tracks.
- Pack both kinds into the fixed eight-entry point-style payload. The effect
  type and emissive level occupy existing unused per-entry components, so the
  uniform ABI does not grow. Beauty adds masked emission before exposure;
  Fast Basic uses a bounded colour multiplier as its preview approximation.
- Preserve the output-identical Colourise/off path, mirror the evaluation in
  CPU offline rendering, and count both kinds against the same five-effect
  responsiveness recommendation.
- Project schema 59 stores the canonical heterogeneous `timing_effects` list
  and sequence while retaining a Colourise-only legacy projection. Render
  setup schema 3 snapshots the same canonical state.
- Keep Water and Timings feature timelines on the shared animation playhead
  while separating their camera policy: follow while the live camera matches
  the pre-scrub animation frame, preserve an orbited inspection view otherwise,
  and expose one shared **Always Follow Camera** override in both tabs. Global
  Animation Position and camera-key timelines remain unconditional.
- Project schema 74 / water-sources schema 28 add per-feature-timeline
  `clips[]` (id, name, start, end, source_profile) and a keyed-settings
  profile `native_length`. Settings clips are authoring metadata over the
  same normalized key domain: every clip operation (offset, stretch,
  duplicate, cross-feature transfer, package capture/apply into a window)
  rewrites the grouped keys through shared water-model helpers, clip-less
  documents parse unchanged, and Timing Take retiming moves clip bounds with
  their keys.
- Project schema 75 / water-sources schema 29 add segment-relative incoming
  and outgoing cubic-Bezier handle coordinates to every keyed Water setting.
  Add **Spline Handles** to the per-key and per-setting interpolation menus,
  invert its monotone time curve during evaluation, expose only the controls
  belonging to the selected key's adjacent manual segments, and preserve the
  handles through value edits, key retiming, clip retiming, save/load, preview,
  and export. Reserve a distinct lower graph rail for time-only key handles so
  minimum-valued curve dots cannot capture retime gestures.
- Project schema 76 / water-sources schema 30 add `clip_id` to every keyed
  Water timeline key (including zero for a deliberately loose key). Derive
  keyed clip bounds from their first/last explicit members, allow clip windows
  to overlap and pass one another, keep same-track/time collision checks local
  to the keys, route new authoring to a single selected stored clip, and retain
  a simultaneous dashed loose-key block for unowned keys. Clip-owned graph
  keys use square/diamond markers while loose keys remain round/linear.
- Keep a one-edit session snapshot around water key authoring. Start the
  transaction before a slider, key, spline-handle, or settings-clip gesture,
  commit its final state once on release/Apply, and let Ctrl+Z (Cmd+Z on macOS)
  toggle between the two snapshots. Do not intercept the shortcut while an
  ImGui text or numeric input owns keyboard editing.

## 10. Procedural Motion System
### 10.1 Motion objective
Allow subtle per-point movement driven by scalar fields without mutating source data.

### 10.2 Motion expression
Suggested v1 model:

```text
offset = direction(point_id, rest_position)
       * amplitude(bindingA, point)
       * sin(time * frequency(bindingB, point) + phase(point_id, point))
```

Possible direction sources:
- pseudo-random vector from point ID,
- axis-aligned directions,
- radial from layer centre,
- field-derived direction later.

### 10.3 Performance strategy
- evaluate in vertex shader or compute pre-pass,
- keep branching light,
- limit dynamic motion to selected layers or subsets,
- allow global preview disable for motion.

## 11. AOV and Field Pass Implementation
### 11.1 Standard passes
Implement first:
- beauty,
- depth,
- alpha,
- layer ID.

### 11.2 Extended passes
Implement next:
- world position,
- motion vector if feasible,
- selected scalar field pass,
- selected mapped parameter pass.

### 11.3 Field pass policy
Allow the user to tag specific scalar fields for output to avoid exploding pass count.

## 12. Render Pipeline Phases
### Phase A — Preview pipeline
- swapchain rendering,
- point budget active,
- motion optional,
- reduced AOV count,
- lower precision acceptable where safe.

### Phase B — Offline pipeline
- full output resolution,
- tiled rendering if needed,
- deterministic output,
- all requested AOVs.

## 13. Implementation Milestones
### Current completed / substantially implemented slices
- Foundation app, windowing, Vulkan viewport, Dear ImGui side panel, CMake/vcpkg build, shader compilation, and Catch2 test wiring.
- Asset discovery, binary PLY point-cloud loading, normal parsing, scalar-field statistics, gSplat asset loading, and same-stem transform pairing.
- Multi-layer point-cloud and gSplat preview rendering in the same camera system.
- Point-cloud style controls for screen sprites, world surfels, colour ramps, X-ray, emissive, weighted-transparent, density, falloff, point-size, opacity, emissive, X-ray, depth-fade, and colormap field bindings.
- Deterministic point-budget sampling plus automatic preview LOD during camera movement/playback.
- Slide-out/pinnable side panel split into Lidar, Visuals, gSplat, Camera, Animation, and Project tabs, with LiDAR lookdev isolated in Visuals.
- Camera shot save/load, ordered camera paths, quaternion interpolation, 30 fps timing, CPU-assisted surface pivot picking, view-frustum-clipped selected-key camera/focus relationship lines that remain visible with off-screen endpoints, one-third-position shared-rig cube gizmos with standard axis/plane translation, retained focus-key world-Z orbit handles, and transactional focus-node Live Edit Mode with full camera/lens capture.
- Preview-only live-view composition guides with a top-toolbar Grid toggle, distinct thirds and emphasized halfway marks, plus independent line-group, colour, opacity, and weight controls.
- Animation paths derived from camera paths, editable camera/focus keys, playback/scrubbing, save/load, focus distance, aperture metadata, LiDAR associations, a default live-view window size, per-animation Quick MP4 export settings/visual selections, and an unlinked preferred Blend Partner in `.ipanim.json` schema v23. Local Water and Timings feature scrubs preserve a manually orbited inspection camera while continuing to evaluate the shared playhead; attached cameras follow normally, the global/camera-key timelines always follow, and both feature tabs expose the same always-follow override. Ordinary Play/Space transport acquires camera follow only from an animation-aligned live view, permanently detaches for that run after manual navigation, and continues advancing the shared playhead and keyed features while detached.
- Matching-Frame Key Alignment keeps **Fix A+B Lens** beside **Extend Both Seams…**. Its ordinary paired timelines support stable right-click key selection plus an immutable camera-alignment snapshot. A live/selected/copied comparison exposes focus distance, world-Z polar angle, local path-tangent-relative horizontal angle, and camera/focus clearances above a shared ground reference beginning one-third of the way from focus to camera and falling forward when that cell is missing. The 10 mm cached authored-SAND channel is authoritative; MESH is permitted only for vegetation-supported retained Ground cells. Selected-node notices expose the chosen source, fallback, or cache/coverage failure. Paste masks select those five geometry values independently plus authored horizon/roll; missing tangent/Ground measurements gate only the dependent paste choices. The standalone repair creates paired edited paths with one median fixed-lens profile per animation and a conservative pair-wide clip union; the extension wizard can instead hold that repair privately until Apply or Cancel.
- Reciprocal pan extension for target-driven, fixed-lens paths: editable working-scene triangles define the existing A-start/B-end and B-start/A-end 50%-blend midpoints. Each inward seam span generates both a source pre-roll and partner tail, using two keys at each simple end or three for a crossed key/inflection, plus localized first/last endpoint alignment. Signed fitted-seam review includes A/B switching, a real two-camera hard split whose adjustable boundary follows the projected anchors, and two selectable key timelines for one-seam spatial smoothing around the paired midpoint before the opposite seam is captured; water work is suppressed while the assistant is active. Final Preview replays those authored-key selections and exposes iterative green-key spatial editing with a viewport gizmo, exact-knot/per-segment spline redraw, coordinated triangle-pair movement, a selected-reference force-align action, selected-only X/Y/rotation/X+rotation/perceived-speed passes, and no timing-weight writes. Four A/B motion graphs mark the 50% keys. A signed -1..1 full-cycle transport selects the appropriate hard-split seam, keeps a separate feature-relative offset for each blend, wraps repeated drags, and loops with Space. Its default 4:00 fit retimes the two unique bulk spans plus every generated seam half with one shared scale, apportions exact integer segment frames to a 7,200-frame cycle, and scales reciprocal overlaps/export bounds without moving camera geometry or triangle constraints. Apply creates a reciprocal blend link for an unlinked pair or updates its existing pair and stores the generated spans as reciprocal start/end blend durations. Pair-aware Save As retargets the two new filenames and commits A, B, and project together without detaching the link. Schema v22 stores explicit correction tangents and a deprecated migration marker. Current extensions affinely shift/rescale normalized timing/water/effect keys so they remain attached to the same old camera poses. Camera position selects those keyed values but never drives procedural motion: live effects use steady elapsed time even with a stationary camera, and export uses deterministic full output time.
- Valid reciprocal links retain a live **Seamed View** after the assistant closes. A pure linked-seam sampler maps either member's current playhead into the partner at both reciprocal overlap bands. The alternating two-camera compositor renders the moving hard split. It follows transient wizard anchors when available and otherwise uses overlap progress, with endpoint-role side assignment, one session-stable offset per physical seam, and ordinary single-camera rendering outside the bands.
- Project schema 77 / animation schema 24 give every reciprocal member a common cyclic timing length plus a signed local window. Apply creates one shared Timing Take transactionally (or **Share Effect Timeline** upgrades an existing pair); full-loop editing stays canonical while ordinary A/B views project loop keys into their camera-local bounds. Water and TimingColourise evaluators use virtual adjacent-cycle keys so all supported interpolation modes retain their boundary rate without adding local endpoint nodes. Procedural Water/Rain/Flow clocks remain independent of camera position.
- Project JSON round-trip for layer load/visibility state, point budgets, point-cloud styles, current camera, camera shots, camera path, project-registered animations, selected layer, render settings, last animation path, preferred/locked live-view window size, side panel state, preview LOD mode, and background/gSplat quality settings.
- Point-cloud style preset save/load.
- GPU animation export for batched saved-animation Quick MP4s per selected saved visual and preview-density EXR stacks, with EXR beauty/alpha/depth channels.
- CPU/offline point tile renderer and multichannel EXR writer test coverage.

### Active gaps / next slices
- Current focus is LiDAR visual polish; gSplat preview remains available, but deeper gSplat export/polish is deliberately less urgent for now.
- Field-driven procedural motion is still a data stub; amplitude/frequency bindings exist only as `MotionProfile` data and are not wired into UI, shaders, project serialization, or export.
- AOV coverage is currently beauty, alpha, and depth. Layer ID, world position, motion vectors, selected scalar-field passes, and mapped-style passes remain to be implemented.
- Final-output validation still needs full-density/high-resolution EXR rendering at exhibition sizes, including clear rules for when preview-density export is acceptable.
- gSplat participation in offline/animation exports is deferred until the LiDAR lookdev path is settled; current animation export focuses on point-cloud layers.
- Command-line/headless rendering for saved projects and animation paths is not implemented yet.
- Large-scene paging/chunk streaming beyond deterministic sampling and preview LOD remains open.

## 14. Codex Agent Breakdown
### Agent A — Core app and Vulkan foundation
Owns:
- `/src/app`
- `/src/platform`
- `/src/renderer/core`

Tasks:
- maintain app shell,
- wire swapchain and frame loop,
- integrate UI backend.

### Agent B — PLY import and attribute model
Owns:
- `/src/io`
- `/src/scene`

Tasks:
- import PLY,
- detect scalar fields,
- compute/store field statistics,
- serialise layer metadata.

### Agent C — Point-cloud renderer
Owns:
- `/src/renderer/pointcloud`
- `/src/renderer/shaders/pointcloud*`

Tasks:
- point draw path,
- point size modes,
- colour modes,
- point budget and basic culling.

### Agent D — Style and parameter binding system
Owns:
- `/src/style`
- `/src/motion`
- shared shader binding helpers.

Tasks:
- implement `RenderParameterBinding`,
- field map math,
- preset system,
- shader evaluation helpers.

### Agent E — UI side panel
Owns:
- `/src/ui`

Tasks:
- side panel state machine,
- parameter binding widget,
- layer/style/render controls,
- override indicators.

### Agent F — Camera and shots
Owns:
- `/src/camera`
- `/src/scene/shots` if separated.

Tasks:
- orbit and fly controls,
- pivot inference,
- shot save/load,
- quaternion interpolation.

### Agent G — Offline output
Owns:
- `/src/output`
- render queue logic.

Tasks:
- tiled rendering,
- AOV export,
- headless render path,
- metadata sidecars.

### Agent H — GS integration
Owns:
- `/src/renderer/gsplat`

Tasks:
- wrap `3dgs-vulkan-cpp`,
- align cameras and transforms,
- expose layer controls and shot integration.

## 15. Initial Acceptance Tests
- Load a PLY and list scalar fields.
- Toggle side panel by hover and double-click pin.
- Map `roughness` to point size via min/max range mapping.
- Save a style preset and reload it.
- Save two shots and interpolate between them.
- Render a preview frame and an offline frame with at least one scalar-field AOV.
- Apply field-driven motion to a dynamic subset.
- Render point cloud and GS in the same scene.
