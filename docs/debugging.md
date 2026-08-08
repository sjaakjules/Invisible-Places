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

## Export memory telemetry

Animation export logs record macOS `TASK_VM_INFO` memory ledgers at export
start, after GPU readback, when each frame is ready for the writer, and at
export finish. The live export summary shows the latest and peak process
footprint plus graphics and compressed memory. It also shows macOS system
thermal pressure (`nominal`, `fair`, `serious`, or `critical`) and retains the
worst state sampled during the export. This is the operating system's thermal
pressure signal, not a raw temperature sensor reading; `serious` or `critical`
alongside slower GPU samples is strong evidence of protective throttling.

On macOS, the log's pressure metric uses the full physical footprint. Export
admission, queue sizing, and the hard memory safety threshold deliberately use
ordinary resident memory: Apple Silicon's physical-footprint ledger includes
the scene's large shared Vulkan/MoltenVK allocations and could otherwise reject
a valid export before it starts. Linux currently has resident-memory telemetry
only.

Completed export logs end with a `Memory Samples (CSV)` section. For repeated
render comparisons, use the same scene, resolution, supersampling, renderer,
and frame range, then compare:

- `physical_footprint_bytes` for total process pressure,
- `graphics_footprint_bytes` for GPU/driver retention,
- `compressed_bytes` for accumulated compressed allocations,
- `resident_bytes` for ordinary mapped process pages,
- `thermal_state` for OS-reported thermal pressure at that sample,
- readback and frame-ready rows to distinguish temporary per-frame allocation
  cycles from a rising baseline.

The memory summary also records the app's directly tracked point-cloud
allocations at export start: CPU geometry/colour capacity, CPU scalar-field
capacity, and Vulkan point-buffer bytes. These values explain the stable scene
payload; growth in the OS physical or graphics footprint while the tracked
point buffers stay fixed points instead to export/driver retention or
per-frame churn.

The resident, compressed, and graphics values are overlapping accounting
views; do not add them together. Use `physical_footprint_bytes` as the total
pressure metric and the other columns to identify its likely source.

### Camera-path culling

`Camera-path frustum mask: yes` means the export computed one conservative
union of the frusta for every requested frame, then submitted only point-cloud
cells that can contribute somewhere along that frozen path. This is still full
authored density inside the retained cells; it is not playback-density
sampling. `Frustum-mask retained` and the retained/full point counts show the
actual vertex-work reduction. The mask deliberately expands every retained
grid cell by one neighbouring cell in each direction so boundary points are
not lost.

The union is prepared once and uploaded once. A separate point-index upload on
every frame could cull more aggressively, but would add large CPU work,
transfer traffic, and GPU synchronization to the frame loop. If a useful union
cannot be formed, the log says `Camera-path frustum mask: no` and the renderer
falls back to the full source; GPU clip-space rejection still protects the
image, but it does not avoid fetching and shading those off-camera vertices.

Full-density scene sessions loaded only for export are reused across one batch
to avoid needless reloads, then released after an independent export or the
final batch item. A coarser committed display bundle stays GPU resident, CPU
analysis sources used by Shoreline/Seepage/Rain/Flow remain available, and the
shared 10 mm water-surface cache stays warm. This bounds back-to-back GPU
residency without changing output or making every item in a batch pay the
full-density load cost again. The export log's finish sample is deliberately
taken before teardown so comparisons still record the complete render's end
pressure; the UI status and console report the subsequent release.

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
