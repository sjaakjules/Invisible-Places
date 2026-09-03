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
requiring unsynchronized 2 mm files. Project saves keep their loaded JSON as a
three-way merge ancestor: identifiable packages, profiles, Timing Takes,
features, settings and key positions merge recursively, with UI choices only
for fields changed on both computers. Animation camera files retain their
strict content fingerprint. Every target is verified again after staging and
mid-save conflicts are copied to local recovery. The Project panel writes both
local markers for the next launch, avoiding a destructive live context switch.

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
- Keep one `DrawTimingKeyLaneGroup` interaction renderer for every Colourise,
  Emissive, Bounds, palette-stop, and falloff value graph. Feed each instance
  lane descriptors and route graph insertion through the existing typed key
  APIs so interpolation and saved-key schemas stay authoritative. Its gesture
  contract matches the Water settings graph (double-click open space adds to
  the only/nearest curve; double-click a node edits exact time), while Water
  remains a second renderer because it additionally owns run transactions,
  clips, dormant tracks, and manual spline handles. Absolute Colourise scalar
  controls reuse `DrawRangedFloatControl`; Palette Phase keeps its relative
  signed-turn rail and accumulated wrapped graph.
- Keep palette stops in an uncapped vector with stable ids. Palette-surface
  selection owns Delete/Backspace, and stop removal erases every Position,
  Colour, and Colourise Amount key for that id. Adding a stop to an animated
  stop-parameter palette backfills its Position at every existing marker-group
  key time; active legacy whole-palette snapshots retain fixed topology.
- Treat the palette's vertical `+ / < / > / X` Position rail as group
  authoring without changing the stored per-stop key format, while direct
  per-marker graph edits remain independent. Build adaptive lane descriptors:
  Palette-only visibility emits every Position/Amount marker plus three
  Colour-coordinate lanes in the effect's selected interpolation space; mixed
  visibility emits a directly editable per-marker lane when exactly one track
  varies, or a read-only arithmetic-mean lane when several do. Expand an
  aggregate's lower time marker back to its real stop keys for selection,
  deletion, and retiming, and exclude its summary nodes from value gestures.
  Compute Position range only from tracks with differing authored values so
  invariant group members do not widen it. Keep legacy snapshot palettes on
  their existing whole-palette path.
- Treat each Emissive falloff key time as one complete curve snapshot without
  changing the stored per-node key format. The curve editor owns click and
  marquee node selection and removes selected nodes plus their keys on Delete
  or Backspace. Typed falloff insertion/autokey stores Position and Level for
  every node, group deletion/retiming operates on the full instant, and the
  unified graph emits lanes only for node-coordinate tracks whose stored
  values differ between snapshots.
- Compose the selected Visual Feature editor as Bounds/profile controls,
  histogram, enabled Colourise palette, enabled Emissive falloff, one unified
  keyframe timeline, then one numeric-settings block. Put the Colourise preset
  and saved-palette selectors directly below Bounds Profile and above
  Histogram Axis. The unified graph has session-only visibility controls for
  Position, Fade, Skew, Palette, and Intensity; Palette and Intensity appear
  only when their output aspect is enabled. `VisualFeatureTimeline.hpp`
  assigns a separate snap domain to every group, so keys snap within their own
  kind but never across the shared graph. Add a session-only Palette Map toggle
  that samples the fully evaluated LUT over time and draws it vertically behind
  the curves. Visibility is UI state only, so
  existing effect parameters, typed key APIs, and serialized key schemas
  remain unchanged.
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
- Point-cloud style controls for screen sprites, world surfels, colour ramps, X-ray, emissive, weighted-transparent, density, falloff, point-size, opacity, emissive, X-ray, depth-fade, and colormap field bindings. The soft-depth prepass defaults to a stable half-alpha hard core while preserving explicitly authored legacy thresholds. Stored-normal fading can use the perspective point-to-camera direction, a pan-stable camera axis, or fixed world +Z; the last pairs with fixed-vertical GPU sorting for camera-independent top-down culling and order.
- Deterministic point-budget sampling plus opt-in session-only camera-adaptive HQ for compatible grouped scenes. A focused unlinked Feature Run View remains a separate conservative fixed-density frustum index mask, and animation playback reuses a motion-fingerprinted prepared camera spline rather than rebuilding it every frame.
- Slide-out/pinnable side panel split into Lidar, Visuals, gSplat, Camera, Animation, and Project tabs, with LiDAR lookdev isolated in Visuals.
- Camera shot save/load, ordered camera paths, quaternion interpolation, 30 fps timing, CPU-assisted surface pivot picking, view-frustum-clipped selected-key camera/focus relationship lines that remain visible with off-screen endpoints, one-third-position shared-rig cube gizmos with standard axis/plane translation, retained focus-key world-Z orbit handles, and transactional focus-node Live Edit Mode with full camera/lens capture.
- Preview-only live-view composition guides with a top-toolbar Grid toggle, distinct thirds and emphasized halfway marks, plus independent line-group, colour, opacity, and weight controls.
- Animation paths derived from camera paths, editable camera/focus keys, playback/scrubbing, save/load, focus distance, aperture metadata, LiDAR associations, a default live-view window size, per-animation Quick MP4 export settings/visual selections, and an unlinked preferred Blend Partner in `.ipanim.json` schema v23. Local Water and Timings feature scrubs preserve a manually orbited inspection camera while continuing to evaluate the shared playhead; attached cameras follow normally, the global/camera-key timelines always follow, and both feature tabs expose the same always-follow override. Ordinary Play/Space transport acquires camera follow only from an animation-aligned live view, permanently detaches for that run after manual navigation, and continues advancing the shared playhead and keyed features while detached.
- Matching-Frame Key Alignment keeps **Fix A+B Lens** beside **Extend Both Seams…**. Its ordinary paired timelines support stable right-click key selection plus an immutable camera-alignment snapshot. A live/selected/copied comparison exposes focus distance, world-Z polar angle, local path-tangent-relative horizontal angle, and camera/focus clearances above a shared ground reference beginning one-third of the way from focus to camera and falling forward when that cell is missing. The 10 mm cached authored-SAND channel is authoritative; MESH is permitted only for vegetation-supported retained Ground cells. Selected-node notices expose the chosen source, fallback, or cache/coverage failure. Paste masks select those five geometry values independently plus authored horizon/roll; missing tangent/Ground measurements gate only the dependent paste choices. The standalone repair creates paired edited paths with one median fixed-lens profile per animation and a conservative pair-wide clip union; the extension wizard can instead hold that repair privately until Apply or Cancel.
- Reciprocal pan extension for target-driven, fixed-lens paths: editable working-scene triangles define the existing A-start/B-end and B-start/A-end 50%-blend midpoints. Each inward seam span generates both a source pre-roll and partner tail, using two keys at each simple end or three for a crossed key/inflection, plus localized first/last endpoint alignment. Signed fitted-seam review includes A/B switching, a real two-camera hard split whose adjustable boundary follows the projected anchors, and two selectable key timelines for one-seam spatial smoothing around the paired midpoint before the opposite seam is captured; water work is suppressed while the assistant is active. Final Preview replays those authored-key selections and exposes iterative green-key spatial editing with a viewport gizmo, exact-knot/per-segment spline redraw, coordinated triangle-pair movement, a selected-reference force-align action, selected-only X/Y/rotation/X+rotation/perceived-speed passes, and no timing-weight writes. Four A/B motion graphs mark the 50% keys. A signed -1..1 full-cycle transport selects the appropriate hard-split seam, keeps a separate feature-relative offset for each blend, wraps repeated drags, and loops with Space. Its default 4:00 fit retimes the two unique bulk spans plus every generated seam half with one shared scale, apportions exact integer segment frames to a 7,200-frame cycle, and scales reciprocal overlaps/export bounds without moving camera geometry or triangle constraints. Apply creates a reciprocal blend link for an unlinked pair or updates its existing pair and stores the generated spans as reciprocal start/end blend durations. Pair-aware Save As retargets the two new filenames and commits A, B, and project together without detaching the link. Schema v22 stores explicit correction tangents and a deprecated migration marker. Current extensions affinely shift/rescale normalized timing/water/effect keys so they remain attached to the same old camera poses. Camera position selects those keyed values but never drives procedural motion: live effects use steady elapsed time even with a stationary camera, and export uses deterministic full output time.
- Valid reciprocal links retain a live **Seam** view after the assistant closes. The canonical linked transport resolves exact member occurrences at both reciprocal overlap bands. The alternating two-camera compositor renders the moving hard split, follows transient wizard anchors when available and otherwise uses overlap progress, with endpoint-role side assignment and one session-stable offset per physical seam. Outside the bands, the sole exact owner renders full-screen.
- Project schema 77 / animation schema 24 give every reciprocal member a common cyclic timing length plus a signed local window. Apply creates one shared Timing Take transactionally (or **Share Effect Timeline** upgrades an existing pair); full-loop editing stays canonical while ordinary A/B views project loop keys into their camera-local bounds. Water and TimingColourise evaluators use virtual adjacent-cycle keys so all supported interpolation modes retain their boundary rate without adding local endpoint nodes. Procedural Water/Rain/Flow clocks remain independent of camera position.
- `AnimationReciprocalLoopTransport` in `src/camera/AnimationPath.*` is the pure transport boundary for the final linked view. It validates the two exact reciprocal timing windows, derives the unique cycle, keeps the canonical shared-frame-zero owner separate from display identity, and converts signed position, cycle frame/phase, and member-local occurrences without clamping. Its path-aware resolver orders normalized full animation file paths lexically, making stored member 0 the stable A slot whichever member was loaded: signed 0 is member-0-start/member-1-end, while signed -1/+1 is member-0-end/member-1-start. Ambiguous repeated occurrences select the one nearest the prior local playhead.
- Application orchestration adds a session-only **Seam / A / B** header over Global Animation Position. A mode switch retains the canonical cycle frame and changes only its displayed local coordinate and isolated/composited source. Seam playback wraps the complete cycle; A/B playback is finite and stops at the selected member end. `CyclicTimelineViewRange` in `src/timing/TimelineView.hpp` stores a start/span lens over `kLinkedSignedCyclicTimelineViewDomain`; it can cross zero or the -1/+1 wrap, is keyed to the canonical pair so ranges are never inherited by a different pair, drives the linked Water, Timing, and visual-setting key timelines, and remains outside project/animation serialization. This is independent of the compiled linked-animation **Live Camera** Overlay/A/B facility.
- `LinkedHighQualityPreview` is the pure live-density boundary for the independent **HQ** control. A reciprocal pair supplies its two authored normalized-0.5 view-projection matrices; an unlinked animation supplies nine uniform start-to-end samples plus every authored camera-key time. The arbitrary-size union evaluates exact point centres, pads X/Y by 5% of the full viewport on each side while retaining clip depth, partitions each included 5 mm role into exact ordered inside/outside complements, fingerprints source bytes/timestamps plus all contributing cameras and the saved Sand-selection mode, maps the project-saved patch spacing (1/2/3 mm) onto the kept fraction of the 1 mm scan's density-preserving cell stratification (`DecimatePointCloudSubsetByGrid` in `PointCloudData`), and maps Waiting/Scanning/Organising/Uploading/Ready onto monotonic UI progress. `PointCloudData` supplies cancellable filtered PLY streaming with compact field-major attributes and ordered original source indices, plus indexed scalar gathering for later Visual/Timing field selections.
- Application orchestration resolves the animation's grouped scene (common to both members for a pair), privately loads/reuses its 5 mm bundle, and runs selected 1 mm role scans at utility priority only after display/shared-water work. ROCK/VEG are always selected; the project-saved Sand toggle is off by default and adds SAND when enabled. With it off, 1 mm SAND is neither fingerprinted nor opened. Two or three high synthetic layer ids hold the compact patches; complementary masks remain dormant until every requested upload succeeds. HQ rendering is opted into only by the main live `BuildRenderState` call, shares each role's Visual/effects and Shoreline eligibility with full-role field ranges, applies ordinary compensation to 5 mm and the selected spacing compensation to patches, and adds descriptor-only Seepage attachments from the settled role topology. The default `BuildRenderState` and all frozen export builders remain canonical, so no HQ id, mask, or hidden baseline can enter still/video/EXR output or serialization; only the user-facing spacing and Sand preferences are project fields.
- The same patch pipeline also owns opt-in **aHQ**. Adaptive resolution does not require an animation: it chooses the selected or uniquely visible compatible grouped scene and inserts the current camera with a 20% viewport guard. Playback advances that camera-owned guard instead of loading the complete path into the draw. Fine preparation stops beyond a screen-projected 5 mm transition; stable GPU stochastic gates cross-fade fine and complete 5 mm roles without a hard density ring. Each 1 mm role has machine-local Morton-ordered geometry/scalar binaries and a JSON sidecar under `Saved/.invisible_places/cache/adaptive_hq/`. Guard requests reject blocks by indexed bounds and seek directly to intersecting 1--8 MiB ranges. Active decoded blocks remain unconditional; recently visited inactive blocks fill a decoded 6 GiB-per-role LRU working set. Strict source identity validation rebuilds stale generations; transactional publication and path guards keep canonical/OneDrive sources untouched. Leaving a published guard restores complete 5 mm for new pixels while retaining the prior depth-faded fine patch until its replacement publishes. Fixed HQ remains a mutually exclusive fallback and all decoded/GPU aHQ state remains live-session/export-isolated.
- Grouped-scene residency has one authoritative owner. Every selector or LiDAR control that makes a scene active releases all direct-folder CPU/GPU sessions and queued loads from every other scene; stale whole-file completions are discarded, and project restore replays loaded state for only its resolved active scene.
- Delimiter-bounded WATER files form a density-aware auxiliary family. Discovery excludes recovery copies (`old`, `backup`, `archive`, and legacy `WATERold`); live display selects one exact/nearest sibling for the committed base density, while output selects the nearest sibling to the finest complete terrain bundle. WATER uses its finest installed sibling as a measured compensation reference, applying nominal footprint scaling plus actual-count correction to opacity and emission, and export teardown frees a canonical WATER source loaded only for output.
- Live preview and frozen render-package/export evaluation feed the transport's canonical cycle phase into the shared Timing Take, Water keyed settings, and Timing Colourise state. Virtual previous/next-cycle keys preserve values and derivatives at phase zero and both member boundaries; procedural Water/Rain/Flow motion retains its independent live/export clock. The view transport never alters a member's finite local export frames. In particular, `Proj_A_09S01` and `Proj_B_09S01` already form an exact 7,200-frame/240-second loop because `6,868 + 6,230 - 3,080 - 2,818 = 7,200`; their 6,868- and 6,230-frame exports remain intentional and no `09S02` asset retime is required. The optional wizard 4:00 fit above remains available for other pairs.
- Focused regressions anchor this contract in `tests/AnimationFocusTests.cpp` (`Reciprocal loop transport preserves asymmetric S01 seam timing`, tag `[reciprocal-loop-transport]`, and `Asymmetric reciprocal boundaries drive identical cyclic Timing Take samples`, tag `[phase-integration]`), `tests/WaterTimingTests.cpp` (`Signed cyclic timeline views cross the seam as a short interval`, `Timeline view sanitization is stable for cyclic invalid and tiny spans`, and `Cyclic keyed settings use virtual neighbours across the loop seam`), and `tests/TimingColouriseTests.cpp` (`Timing Colourise cyclic evaluation interpolates through loop zero`, tag `[timing][colourise][cyclic]`).
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
- Spatial paging for the general renderer and non-aHQ analysis paths remains open. aHQ now has persistent Morton-block paging, direct range seeks, and recent-fringe retention; canonical density loading and other whole-cloud tools still consume monolithic PLY inputs.

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
