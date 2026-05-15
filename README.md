# Invisible Places

Invisible Places is a desktop renderer and shot-authoring tool for turning large spatial captures into cinematic images. It combines CloudCompare-exported point clouds and Gaussian splats in one Vulkan scene, then gives the artist controls for styling, camera movement, preview playback, and offline export.

The project is built for visual research and exhibition workflows: fast look-development on Apple Silicon through MoltenVK, deterministic renders for repeatable output, and a rendering model that can move to stronger Windows GPUs when final output needs more headroom.

## What it is

Invisible Places is not just a point-cloud viewer. It is an authoring environment for making measured places expressive.

It lets a user load LiDAR-style point clouds, preserve their scalar fields, mix in aligned Gaussian splat captures, design visual treatments, save camera shots, turn those shots into animation paths, and export preview movies or EXR image stacks for postproduction.

The central idea is that data already present in the point cloud can become visual direction. Height, roughness, classification, curvature, density proxies, or any CloudCompare-authored scalar field can drive colour, point size, opacity, emission, X-ray strength, depth fade, and related style parameters.

## Why it is useful

- **Fast render iteration:** the app has deterministic point budgets, automatic camera-motion preview LOD, and a Fast Basic point-render path for keeping navigation, playback, and look-development responsive.
- **Beauty render control:** the Beauty path supports richer point-cloud styling, screen sprites, world surfels, falloff profiles, depth-aware rendering, X-ray looks, emissive accents, eye-dome lighting, stylisation, and Gaussian splat preview.
- **Smooth camera paths:** camera shots store orientation as quaternions, use shortest-path quaternion interpolation for rotations, and evaluate saved shot paths on a 30 fps timebase. Editable animation paths spline camera and focus positions for controlled fly-throughs.
- **Data-driven art direction:** render parameters can be constant values or field-mapped controls with input/output ranges, layer statistics, clamp, invert, and gamma shaping.
- **Hybrid capture scenes:** point-cloud layers and Gaussian splat layers share the same camera, scene, and project workflow, so survey geometry and photogrammetric/3DGS material can be composed together.
- **Postproduction-friendly output:** preview-density EXR stacks currently write `beauty.RGB`, `alpha.A`, and `depth.Z`, while Quick MP4 export gives fast review movies through `ffmpeg`.
- **Portable project state:** scenes, camera shots, animation paths, style presets, render settings, and panel state are serialized to JSON so a session can be reopened and rendered consistently.

## Current capabilities

### Scene and assets

- Discovers point-cloud and Gaussian splat assets from `Data/`.
- Loads CloudCompare-style binary PLY point clouds with RGB and optional `scalar_*` fields.
- Reads scalar-field statistics for field-driven styling.
- Loads Gaussian splat PLY files named with the `gSplat-` prefix.
- Applies same-stem `.txt` 4x4 transform matrices for Gaussian splat alignment.
- Supports multiple LiDAR and gSplat layers in the same scene.

### Interactive rendering

- Runs a Vulkan viewport on macOS using MoltenVK.
- Renders point clouds and Gaussian splats together.
- Provides two main point-cloud renderer modes:
  - **Fast Basic** for responsive preview, navigation, and quick exports.
  - **Beauty** for richer surfel/sprite materials, depth behaviour, and stylised looks.
- Supports screen-space sprites, world surfels, and camera-facing world sprites.
- Supports source RGB, solid colour, and scalar colormap modes.
- Includes colormaps such as Viridis, Plasma, Inferno, Magma, Cividis, Turbo, topographic, land-surface, fire, ice, and high-contrast ramps.
- Includes point-cloud stylisation controls for watercolor, living wash, cartoon ink, brush dabs, pencil hatch, grainy pigment, and related painterly looks.
- Supports eye-dome lighting for depth readability outside the Fast Basic path.
- Uses deterministic point sampling and spatial sampling so large clouds can be reduced predictably.
- Has automatic preview LOD during camera navigation or animation playback.

### Camera and animation

- Supports orbit-style scene navigation with inferred pivots from visible point/splat samples near the cursor or screen centre.
- Saves named camera shots with camera position, target, orbit centre, orientation, FOV, clip planes, and depth-of-field settings.
- Interpolates shot rotations with normalized quaternions and spherical interpolation where appropriate.
- Builds weighted camera paths from ordered shots, using segment distance and rotation to distribute timing.
- Converts shots into editable animation paths.
- Plays, scrubs, edits, and saves animation paths.
- Uses a 30 fps project timebase for camera and animation evaluation.

### Styling and parameter binding

- Exposes point-cloud style controls through the side panel.
- Lets major style parameters switch between constant and scalar-field-mapped modes.
- Field-mapped bindings support:
  - field selection,
  - input minimum and maximum,
  - output minimum and maximum,
  - layer-statistics normalization,
  - clamp,
  - invert,
  - gamma shaping.
- Field-driven controls currently cover point size, surfel diameter, opacity, emissive strength, X-ray strength, depth fade, and colormap position.
- Saves and reloads point-cloud style presets.

### Export and persistence

- Saves and reloads project JSON containing scene state, point-cloud styles, camera shots, animation paths, saved visuals, and export selections.
- Exports selected saved animations and saved visuals as batched Quick MP4 files.
- Exports preview-density EXR animation stacks.
- Writes EXR `beauty.RGB`, `alpha.A`, and `depth.Z` channels.
- Includes tiled offline point-rendering support for LiDAR-first output experiments.

## Repository shape

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
/shaders
/tests
/docs
/Data
/Saved
```

## Build strategy

- Language: C++
- Graphics API: Vulkan
- macOS runtime path: MoltenVK
- Build system: CMake
- Package strategy: `vcpkg` manifest mode
- CMake package style: `find_package()`
- Shared render path goal: Vulkan on both macOS and Windows, with MoltenVK acting as the macOS portability layer.

The `vcpkg.json` manifest currently includes:

- `glfw3`
- `imgui`
- `glm`
- `fmt`
- `spdlog`
- `nlohmann-json`
- `openexr`
- `catch2`

## Local build flow

1. Install and configure a working Vulkan SDK on macOS. A complete LunarG SDK is the cleanest path because shader compilation requires `glslc`.
2. Install `vcpkg`, clone the repo root, and either export `VCPKG_ROOT="$HOME/vcpkg"` or use the preset that points at `~/vcpkg`.
3. Configure:

```bash
cmake --preset macos-debug-home-vcpkg
```

4. Build:

```bash
cmake --build --preset build-macos-debug-home-vcpkg
```

5. Run the preview app against the local data folder:

```bash
./build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places ./Data
```

## Debugging

Debugger setup is included for macOS LLDB and VS Code in:

- [.vscode/launch.json](/Users/juju/Documents/Repositories/Invisible%20Places/.vscode/launch.json)
- [.vscode/tasks.json](/Users/juju/Documents/Repositories/Invisible%20Places/.vscode/tasks.json)
- [docs/debugging.md](/Users/juju/Documents/Repositories/Invisible%20Places/docs/debugging.md)

The simplest path is `Debug Invisible Places App`, which builds first and runs against the local `Data/` folder automatically.

## Data assumptions

- Point-cloud PLY files are standard CloudCompare exports with RGB and optional `scalar_*` properties.
- Gaussian splat files are PLY files whose filename starts with `gSplat-` and whose header exposes Gaussian attributes such as `f_dc_0`, `opacity`, `scale_0`, and `rot_0`.
- Each gSplat file is paired with a same-stem `.txt` file containing a 4x4 transform matrix.
- `ffmpeg` is expected at `/opt/homebrew/bin/ffmpeg` for Fast Preview / Quick MP4 export.

## Current status

The repository is in an interactive preview/export phase. It can load local assets, render point clouds and Gaussian splats together, author visual styles, save project state, create camera shots, edit animation paths, and export preview movies or EXR stacks.

The active implementation focus is LiDAR visual polish. gSplat preview is available, but deeper gSplat export polish is parked until the LiDAR look-development workflow is more settled.

## Near-term roadmap

1. Field-driven procedural motion for point-cloud layers.
2. Fuller AOV selection beyond beauty/alpha/depth, especially layer ID, scalar-field passes, and mapped-style passes.
3. Full-density or tiled final EXR export validation at exhibition resolutions for LiDAR-first output.
4. Command-line/headless render invocation for saved projects and animation paths.
5. Deeper large-scene chunking and paging beyond the current point-budget and preview-LOD path.
6. Deferred gSplat participation in offline/animation exports once LiDAR polish is no longer the main priority.

## Validated on this machine

The following setup path was validated in this workspace on April 30, 2026:

- Homebrew `vcpkg` executable installed at `/opt/homebrew/bin/vcpkg`
- cloned `vcpkg` root at `~/vcpkg`
- `cmake --preset macos-debug-vcpkg` configured successfully when `VCPKG_ROOT="$HOME/vcpkg"` was set
- the preview executable built and discovered 8 point-cloud layers plus 10 gSplat layers from `Data/`
- `ctest --test-dir build/macos-debug --output-on-failure` passed
