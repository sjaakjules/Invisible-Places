# Code Review: Unfinished, Unconnected, and Obvious Bugs

Date: 2026-05-27

Scope reviewed: every file returned by `rg --files src shaders tests | sort`:
70 files in `src/`, 35 files in `shaders/`, and 1 file in `tests/`.

Verification run:

- `cmake --build build/macos-debug --target invisible_places_tests`: target was already up to date.
- `cmake --build build/macos-debug --target invisible_places`: target was already up to date.
- `ctest --test-dir build/macos-debug --output-on-failure`: 130/130 tests passed.

## Summary

The current tree is buildable and the test suite passes. The main risks are not compile failures; they are product wiring gaps. Several water features still have runtime state, editing functions, bake paths, and tests, but the active UI and persistence layer now expose only the newer ripple/field model. There is also a raytraced preview mode that can be serialized but cannot be selected from the UI and is not handled by the live renderer as a separate mode.

Shader review did not find an unlisted shader file: the 32 standalone shader stages are listed in `INVISIBLE_PLACES_SHADER_SOURCES`, and the 3 GLSL include files are listed in `INVISIBLE_PLACES_SHADER_INCLUDES` in `CMakeLists.txt:36-74`. The app and test targets are also wired in `CMakeLists.txt:105-209`.

## Chosen Direction

These directions were chosen after the review. The implemented state is summarized in `docs/CURRENT_STATE.md`, with completion evidence archived in `docs/logs/fixbugs_f1_f7_completion_log.md`:

- F1: Remove the legacy Basin/Runoff/Caustics editor UI and dead runtime/editor paths. Keep Water v2 as Ripples, Flow, and Field. Preserve legacy Caustics migration to Ripple / Caustic Lace, and keep legacy Basin/Runoff ignored on load.
- F2: Remove the hard-disabled Trail Shape and Animation Trail Playback panels. The currently visible Stream, Field, and Visual settings are the supported controls.
- F3: Remove all raycast/raytracing functionality, including preview mode, Beauty Raycast export, Vulkan compute raycast code, raycast shaders, BVH helpers, settings, serialization fields, and tests.
- F4: Remove `MotionProfile` entirely.
- F5-F7: Fix directly as identified: correct Cocoa color ownership, remove duplicate water-load resets, and add visible ranged-float validation feedback.

## Findings

### F1. Legacy water basin, runoff, and caustic region editors are compiled but no longer reachable or persisted

Type: unconnected feature / migration hazard

Evidence:

- Runtime state still owns legacy water region collections and placement flags: `src/app/Application.cpp:608-612` and `src/app/Application.cpp:680-688`.
- The legacy region data models still exist: `src/water/WaterFlow.hpp:532-573`.
- Full editor panels still exist for these features: `DrawWaterBasinPanel` at `src/app/Application.cpp:16070`, `DrawWaterRunoffPanel` at `src/app/Application.cpp:16190`, and `DrawWaterCausticsPanel` at `src/app/Application.cpp:16300`.
- The active `DrawWaterPanel` only opens `Ripples`, `Flow`, and `Field` tabs, and only calls `DrawWaterRipplesPanel` and `DrawWaterFieldPanel`: `src/app/Application.cpp:17130-17855`. Search confirms the legacy panel functions are only defined, not called.
- The old viewport input hooks remain active if their flags are set: `src/app/Application.cpp:18370-18390`.
- Bake and mask functions remain: `BakeBasinHazeOverlayForActiveLayer` at `src/app/Application.cpp:6275`, `BakeRunoffOverlayForActiveLayer` at `src/app/Application.cpp:6326`, and `RefreshWaterCausticMaskForSession` at `src/app/Application.cpp:6568`.
- Project save and standalone water-source save only write emitters, ripple layers, field layers, and current settings: `src/app/Application.cpp:5747-5758` and `src/app/Application.cpp:7073-7075`.
- Loading explicitly clears the legacy arrays: `src/app/Application.cpp:5785-5789` and `src/app/Application.cpp:7302-7307`.
- Document structs still expose legacy arrays: `src/serialization/ProjectDocument.hpp:74-77` and `src/serialization/ProjectDocument.hpp:109-116`.
- Writers omit `water_basin_regions`, `water_runoff_regions`, and `water_caustic_regions`: `src/serialization/ProjectDocument.cpp:2702-2787` and `src/serialization/ProjectDocument.cpp:3078-3113`.
- Loaders migrate legacy caustic regions into v2 ripple layers only when native ripple layers are absent, and do not restore basin/runoff/caustic arrays: `src/serialization/ProjectDocument.cpp:2879-2926` and `src/serialization/ProjectDocument.cpp:3196-3242`.
- Tests currently assert that the legacy keys are absent and the arrays load empty: `tests/AssetDiscoveryTests.cpp:1391-1393`, `tests/AssetDiscoveryTests.cpp:1541-1543`, `tests/AssetDiscoveryTests.cpp:4675-4677`, and `tests/AssetDiscoveryTests.cpp:4736-4738`.

Impact:

If basin/runoff/legacy caustic editing is still intended, the product path is broken: the panels cannot be reached, and reconnected edits would not survive save/load. If the v2 ripple/field model fully replaces them, this is dead public state and UI code that should be removed or moved behind an explicit legacy migration boundary.

Suggested next step:

Decide whether these are deprecated. If yes, remove the legacy UI/input/runtime fields and keep only the loader migration. If no, reconnect the panels and restore persistence/tests before exposing them.

### F2. Trail Shape and Animation Trail Playback panels are hard-disabled

Type: unfinished UI wiring

Evidence:

- `Trail Shape` is guarded by `if (false && BeginPanelSection("Trail Shape"))`: `src/app/Application.cpp:17547`.
- `Animation Trail Playback` is guarded by `if (false && BeginPanelSection("Animation Trail Playback"))`: `src/app/Application.cpp:17603`.
- Both blocks contain live editing logic that updates source/trail settings and refreshes water overlays: `src/app/Application.cpp:17547-17718`.
- Serialization and tests cover many of these settings, for example trail shape and animation trail profile roundtrips at `tests/AssetDiscoveryTests.cpp:1473-1491`.

Impact:

The code supports trail shape and animation trail settings, but users cannot edit those controls in the active UI. This makes those settings mostly load/save artifacts unless another UI path edits them.

Suggested next step:

Either remove the disabled blocks and obsolete profile code, or re-enable them after checking their layout and interaction with the current water workflow.

### F3. Raytraced point renderer mode is accepted by data model and serialization but not connected as a live preview renderer

Type: unconnected feature / misleading state

Evidence:

- `PointCloudRendererMode` includes `Raytraced`: `src/renderer/pointcloud/PointCloudPreviewState.hpp:84-87`.
- Serialization names and parses `"raytraced"`: `src/serialization/ProjectDocument.cpp:237-256`.
- The UI presents `Raytraced (later)`, but selecting it resets the mode to `Beauty` and shows a reservation message: `src/app/Application.cpp:15887-15902`.
- The mode label still reports `Raytraced`: `src/app/Application.cpp:1079-1089`.
- The live renderer only branches on `FastBasic`; everything else follows the beauty material path: `src/renderer/core/VulkanViewportShell.cpp:1082-1087` and `src/renderer/core/VulkanViewportShell.cpp:6248-6249`.
- Raycast export is real and separately exposed under animation export: `src/app/Application.cpp:11693-11724`, `src/renderer/core/VulkanViewportShell.hpp:154-163`, and `src/renderer/core/VulkanViewportShell.cpp:1585-1598`.

Impact:

A project file can contain `point_cloud_renderer_mode: "raytraced"`, and the status panel can label it Raytraced, but live preview rendering still behaves as Beauty unless the user changes the combo. That is confusing and could cause stale or misleading project state.

Suggested next step:

Until live raytraced preview exists, normalize parsed/project-applied `Raytraced` to `Beauty`, or hide it from project serialization. If live raytraced preview is planned, add an explicit renderer branch and UI path.

### F4. `MotionProfile` is a declared placeholder with no integration

Type: explicit placeholder

Evidence:

- `src/motion/MotionProfile.hpp:7-13` says UI, shader evaluation, project serialization, and export integration are still outstanding.
- Search for `MotionProfile` in `src`, `tests`, and `CMakeLists.txt` finds only the struct declaration itself.

Impact:

This is harmless at runtime, but it is intentionally unfinished and should not be mistaken for a connected feature.

Suggested next step:

Either leave it documented as future scaffolding, or remove it until the motion slice is ready to connect.

### F5. Cocoa bootstrap view leaks created Core Graphics colors

Type: obvious memory-management bug

Evidence:

- `src/platform/WindowBootstrapView.mm:53-67` assigns `CGColorCreateGenericRGB(...)` results directly to layer properties.
- The file is compiled with ARC (`CMakeLists.txt:140-148`), but Core Foundation objects returned by `Create` still need ownership handling.

Impact:

Each bootstrap content install retains three `CGColorRef` objects without release. The leak is small, but the code can run more than once while rebuilding bootstrap content.

Suggested next step:

Use `NSColor` `CGColor` accessors, or release/bridge the created `CGColorRef` values after assigning them.

### F6. `LoadWaterSources` has duplicate reset assignments

Type: low-risk copy/paste bug

Evidence:

- `nextFieldLayerId` is assigned twice: `src/app/Application.cpp:5803-5804`.
- `selectedFieldLayerIndex` is reset twice: `src/app/Application.cpp:5809-5811`.
- `fieldRegionPlacementArmed` is reset twice: `src/app/Application.cpp:5816-5818`.

Impact:

This is currently harmless, but duplicated reset code makes future changes easier to miss and is worth cleaning while touching the water-load path.

Suggested next step:

Remove the duplicate assignments and add a small load-state regression check if this code gets changed.

### F7. Ranged float editing silently rejects out-of-range typed values

Type: unfinished UX feedback

Evidence:

- `src/app/Application.cpp:2295-2297` has a TODO to surface an out-of-range popup after `TryAssignRangedFloatValue` rejects invalid typed input.

Impact:

Dragging the custom ranged control is constrained, but typed invalid values can fail without direct feedback. This is not a correctness bug, but it is a user-facing unfinished behavior.

Suggested next step:

Add transient validation feedback for rejected typed values once the panel notification pattern is chosen.

## Test Coverage Notes

The single test file is broad and currently healthy: `tests/AssetDiscoveryTests.cpp` contains 130 Catch2 test cases and all passed. It covers discovery, loading, serialization, water algorithms, camera paths, output encoders, EXR, EDL, offline point rendering, gSplat helpers, point style defaults, and legacy migrations.

Known gaps from this review:

- The tests assert that legacy basin/runoff/caustic data is not saved and loads empty, so they will not catch the legacy editor being unreachable or non-persistent.
- There are no UI interaction tests for disabled water panels, renderer mode selection, or save/load panel behavior.
- Shader compilation is covered by the build target, but there are no GPU screenshot/pixel tests for live shader behavior.

## Reviewed File Inventory

```text
shaders/edl_postprocess.frag
shaders/gsplat_accumulation.frag
shaders/gsplat_accumulation.vert
shaders/gsplat_composite.frag
shaders/gsplat_composite.vert
shaders/gsplat_high_quality.frag
shaders/gsplat_high_quality.vert
shaders/pointcloud_accumulation.frag
shaders/pointcloud_caustics.glsl
shaders/pointcloud_colormaps.glsl
shaders/pointcloud_constant_simple.vert
shaders/pointcloud_constant_simple_accumulation.frag
shaders/pointcloud_export_depth.frag
shaders/pointcloud_exr_accumulation.frag
shaders/pointcloud_exr_composite.frag
shaders/pointcloud_exr_constant_simple_accumulation.frag
shaders/pointcloud_fast_basic.frag
shaders/pointcloud_fast_basic.vert
shaders/pointcloud_fast_basic_depth.frag
shaders/pointcloud_opaque_hard_disc.frag
shaders/pointcloud_preview.frag
shaders/pointcloud_preview.vert
shaders/pointcloud_raycast_accumulate.comp
shaders/pointcloud_raycast_clear.comp
shaders/pointcloud_raycast_composite.comp
shaders/pointcloud_stylisation.glsl
shaders/pointcloud_surfel.vert
shaders/pointcloud_surfel_accumulation.frag
shaders/pointcloud_surfel_constant_simple.vert
shaders/pointcloud_surfel_constant_simple_accumulation.frag
shaders/pointcloud_surfel_export_depth.frag
shaders/pointcloud_surfel_exr_accumulation.frag
shaders/pointcloud_surfel_exr_constant_simple_accumulation.frag
shaders/pointcloud_surfel_opaque_hard_disc.frag
shaders/pointcloud_surfel_preview.frag
src/app/Application.cpp
src/app/Application.hpp
src/app/PointVisualSelection.cpp
src/app/PointVisualSelection.hpp
src/camera/AnimationPath.cpp
src/camera/AnimationPath.hpp
src/camera/CameraPath.cpp
src/camera/CameraPath.hpp
src/camera/CameraShot.hpp
src/camera/CameraState.hpp
src/camera/OrbitCamera.cpp
src/camera/OrbitCamera.hpp
src/io/AssetDiscovery.cpp
src/io/AssetDiscovery.hpp
src/io/GaussianSplatData.cpp
src/io/GaussianSplatData.hpp
src/io/PlyHeader.cpp
src/io/PlyHeader.hpp
src/io/PointCloudData.cpp
src/io/PointCloudData.hpp
src/io/TransformMatrix.cpp
src/io/TransformMatrix.hpp
src/main.cpp
src/motion/MotionProfile.hpp
src/output/ExrWriter.cpp
src/output/ExrWriter.hpp
src/output/EyeDomeLighting.cpp
src/output/EyeDomeLighting.hpp
src/output/HoudiniCameraExport.cpp
src/output/HoudiniCameraExport.hpp
src/output/OfflinePointRenderer.cpp
src/output/OfflinePointRenderer.hpp
src/output/RenderPreset.cpp
src/output/RenderPreset.hpp
src/output/VideoWriter.cpp
src/output/VideoWriter.hpp
src/platform/MacWindowingRuntime.cpp
src/platform/MacWindowingRuntime.hpp
src/platform/MacWindowingRuntime.mm
src/platform/VulkanRuntimeConfig.cpp
src/platform/VulkanRuntimeConfig.hpp
src/platform/Window.cpp
src/platform/Window.hpp
src/platform/WindowBootstrapView.cpp
src/platform/WindowBootstrapView.hpp
src/platform/WindowBootstrapView.mm
src/platform/WindowTitle.cpp
src/platform/WindowTitle.hpp
src/renderer/core/RendererBootstrap.hpp
src/renderer/core/VulkanViewportShell.cpp
src/renderer/core/VulkanViewportShell.hpp
src/renderer/gsplat/GsplatLayer.hpp
src/renderer/gsplat/HighQualityGaussianScene.cpp
src/renderer/gsplat/HighQualityGaussianScene.hpp
src/renderer/pointcloud/Colormap.cpp
src/renderer/pointcloud/Colormap.hpp
src/renderer/pointcloud/PointCloudLayer.hpp
src/renderer/pointcloud/PointCloudPreviewState.cpp
src/renderer/pointcloud/PointCloudPreviewState.hpp
src/renderer/pointcloud/RaycastBvh.cpp
src/renderer/pointcloud/RaycastBvh.hpp
src/scene/SceneCatalog.cpp
src/scene/SceneCatalog.hpp
src/serialization/ProjectDocument.cpp
src/serialization/ProjectDocument.hpp
src/style/RenderParameterBinding.cpp
src/style/RenderParameterBinding.hpp
src/ui/SidePanelState.hpp
src/water/WaterFlow.cpp
src/water/WaterFlow.hpp
tests/AssetDiscoveryTests.cpp
```
