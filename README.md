# Invisible Places

Invisible Places is a desktop renderer and shot-authoring tool for combining very large point clouds and Gaussian splats in the same scene. The early priority is responsive look-development on Apple Silicon M1 using Vulkan through MoltenVK, while keeping the same rendering model portable to stronger Windows GPUs for final output.

## Current status

This repository is now in an interactive preview/export phase:

- the architecture matches the planning docs,
- the build is wired for `CMake + vcpkg manifest mode`,
- the codebase can already discover local point-cloud and gSplat assets from the `Data/` folder,
- the app now renders point clouds and Gaussian splats together in a Vulkan viewport on macOS,
- point-cloud lookdev supports screen sprites, world surfels, colour ramps, X-ray, emissive, transparent, and field-mapped bindings for major scalar parameters,
- project state, camera shots, and point-cloud style presets can be saved to JSON and loaded back into the session,
- the side panel is split into Lidar, gSplat, Camera, Animation, and Project work areas,
- camera shots can be saved, loaded, interpolated at the 30 fps project timebase, and converted into editable animation paths,
- saved animation paths can be played, scrubbed, edited, and exported as Fast Preview MP4 or preview-density EXR stacks,
- EXR output currently writes beauty, alpha, and depth channels,
- orbit pivots can be inferred from visible point/splat samples near the cursor or screen centre,
- large point clouds have deterministic budget sampling and an automatic camera-motion preview LOD path.

## Current repository shape

```text
/src
  /app
  /camera
  /io
  /motion
  /output
  /renderer
    /core
    /gsplat
    /pointcloud
  /scene
  /serialization
  /style
  /ui
/tests
/docs
/Data
```

## Build strategy

- Default package strategy: `vcpkg` manifest mode
- CMake package style: `find_package()`
- Allowed exception path: `FetchContent` only for a GS dependency that is missing from vcpkg or requires a pinned fork
- Shared render path goal: Vulkan on both macOS and Windows, with MoltenVK only acting as the macOS portability layer

## Recommended dependencies

The `vcpkg.json` manifest is set up for these first-step dependencies:

- `glfw3`
- `imgui`
- `glm`
- `fmt`
- `spdlog`
- `nlohmann-json`
- `openexr`
- `catch2`

## Local build flow

1. Install and configure a working Vulkan SDK on macOS. A complete LunarG SDK is the cleanest path because the current machine state is still missing `glslc`, and the older Homebrew MoltenVK path was not enough on its own to validate runtime setup.
2. Install `vcpkg`, clone the repo root, and either:
   - export `VCPKG_ROOT="$HOME/vcpkg"`, or
   - use the preset that points at `~/vcpkg` directly.
3. Configure:

```bash
cmake --preset macos-debug-home-vcpkg
```

4. Build:

```bash
cmake --build --preset build-macos-debug-home-vcpkg
```

5. Run the preview app:

```bash
./build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places ./Data
```

## Debugging

Debugger setup is included for macOS LLDB and VS Code in:

- [.vscode/launch.json](/Users/juju/Documents/Repositories/Invisible%20Places/.vscode/launch.json)
- [.vscode/tasks.json](/Users/juju/Documents/Repositories/Invisible%20Places/.vscode/tasks.json)
- [docs/debugging.md](/Users/juju/Documents/Repositories/Invisible%20Places/docs/debugging.md)

The simplest path is `Debug Invisible Places App`, which builds first and runs against the local `Data/` folder automatically.

## Data assumptions currently encoded

- Point-cloud PLY files are standard CloudCompare exports with RGB and optional `scalar_*` properties.
- Gaussian splat files are PLY files whose filename starts with `gSplat-` and whose header exposes Gaussian attributes such as `f_dc_0`, `opacity`, `scale_0`, and `rot_0`.
- Each gSplat file is paired with a same-stem `.txt` file containing a 4x4 transform matrix.

## Immediate next milestone

The next implementation slice should be:

1. field-driven procedural motion for point-cloud layers,
2. fuller AOV selection beyond beauty/alpha/depth, especially layer ID, scalar-field passes, and mapped-style passes,
3. full-density or tiled final EXR export validation at exhibition resolutions, including gSplat participation where required,
4. command-line/headless render invocation for saved projects and animation paths,
5. deeper large-scene chunking / paging beyond the current point-budget and preview-LOD path.

## Validated on this machine

The following setup path was validated in this workspace on April 30, 2026:

- Homebrew `vcpkg` executable installed at `/opt/homebrew/bin/vcpkg`
- cloned `vcpkg` root at `~/vcpkg`
- `cmake --preset macos-debug-vcpkg` configured successfully when `VCPKG_ROOT="$HOME/vcpkg"` was set
- the preview executable built and discovered 8 point-cloud layers plus 10 gSplat layers from `Data/`
- `ctest --test-dir build/macos-debug --output-on-failure` passed
