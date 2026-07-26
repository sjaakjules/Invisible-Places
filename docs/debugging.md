# Debugging Guide

This repository is set up for a debug-first workflow on macOS Apple Silicon using LLDB and CMake presets.

## Quick path

If you are using VS Code:

1. Install the recommended extensions from `.vscode/extensions.json`.
2. Open the Run and Debug panel.
3. Choose one of:
   - `Debug Invisible Places App`
   - `Debug Invisible Places Tests`
   - `Debug Asset Discovery Test Only`
4. Press `F5`.

The debug launch configurations automatically:

- build the debug target first,
- run from the workspace root,
- pass the `Data/` directory as the program argument,
- disable most macOS unified-log noise in the VS Code debug console.

## Available tasks

The workspace also includes reusable tasks:

- `configure-debug`
- `build-debug`
- `run-app`
- `run-tests`

These are useful even if you are not stepping through code yet.

## Terminal LLDB

If you want to debug from the terminal instead of an editor:

```bash
cmake --preset macos-debug-home-vcpkg
cmake --build --preset build-macos-debug-home-vcpkg
lldb -- ./build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places "/Users/juju/Documents/Repositories/Invisible Places/Data"
```

Inside LLDB:

```text
run
```

To debug the tests instead:

```bash
lldb ./build/macos-debug/invisible_places_tests
```

And for a single test:

```text
run "Data discovery finds both point clouds and gaussian splats"
```

## Current scope

Right now the debug target is most useful for:

- PLY header parsing,
- gSplat transform pairing,
- asset discovery,
- scene catalog generation,
- Vulkan viewport initialization,
- point-cloud and gSplat preview rendering,
- side-panel lookdev controls,
- camera shot save/load/interpolation and CPU-assisted pivot picking,
- animation-path save/load/scrub/edit behaviour,
- Fast Preview MP4 and preview-density EXR animation export,
- EXR writer and offline LiDAR tile-rendering tests.

As richer AOVs, full-density final-output validation, command-line rendering, and procedural motion are added, the same debug flow should keep working with the same debug preset and LLDB launch configs.

## Console noise on macOS

The current app now runs as a proper `.app` bundle, so the missing bundle identifier warning should be gone.

You may still occasionally see system-side macOS messages such as:

- `Unable to obtain a task name port right for pid 400`
- `fopen failed for data file`
- `Errors found! Invalidating cache...`

These are typically OS or framework diagnostics from the debug environment rather than failures in the app itself. The signal to watch for in this project is our own renderer line:

```text
Renderer: Apple M1 Max | 2880x1800 | Vulkan viewport
```

That line means the Vulkan viewport shell initialized successfully; loaded layers should then render through the Lidar and gSplat panels.

## Refocusing animation focus points onto the surface

Camera focus points authored before surface snapping existed can sit far past
the terrain (the orbit target drifts with dolly/pan). Saving a camera shot or
saving a camera path as an animation now snaps the focus point onto the first
point-cloud surface hit along the existing view ray, and the Animation tab's
**Refocus To Surface** button does the same for the loaded animation's keys
(framing is unchanged; only the focus depth moves, which also places depth of
field on the surface). A headless utility applies it to a saved animation:

```
./build/macos-debug/invisible_places.app/Contents/MacOS/invisible_places \
    "$PWD/Data" --refocus-animation Exhibition
```

It loads the default project's display scene, backs up the animation file and
the project file (`*.before_refocus.<timestamp>.bak` beside the originals),
rewrites the animation keys, and patches only the linked `camera_shots`
entries inside the project JSON.
