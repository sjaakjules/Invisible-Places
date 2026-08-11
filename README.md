# Invisible Places

Invisible Places is a desktop renderer and shot-authoring tool for turning large spatial captures into cinematic images. It combines CloudCompare-exported point clouds and Gaussian splats in one Vulkan scene, then gives the artist controls for styling, camera movement, preview playback, and offline export.

The project is built for visual research and exhibition workflows: fast look-development on Apple Silicon through MoltenVK, deterministic renders for repeatable output, and a rendering model that can move to stronger Windows GPUs when final output needs more headroom.

## What it is

Invisible Places is not just a point-cloud viewer. It is an authoring environment for making measured places expressive.

It lets a user load LiDAR-style point clouds, preserve their scalar fields, mix in aligned Gaussian splat captures, design visual treatments, save camera shots, turn those shots into animation paths, and export preview movies or EXR image stacks for postproduction.

The central idea is that data already present in the point cloud can become visual direction. Height, roughness, classification, curvature, density proxies, or any CloudCompare-authored scalar field can drive colour, point size, opacity, emission, depth fade, and related style parameters.

## Why it is useful

- **Fast render iteration:** the app has deterministic point budgets, automatic camera-motion preview LOD, and a Fast Basic point-render path for keeping navigation, playback, and look-development responsive.
- **Beauty render control:** the Beauty path supports richer point-cloud styling, screen sprites, world surfels, falloff profiles, depth-aware rendering, emissive accents, eye-dome lighting, stylisation, and Gaussian splat preview.
- **Smooth camera paths:** camera shots store orientation as quaternions, use shortest-path quaternion interpolation for rotations, and evaluate saved shot paths on a 30 fps timebase. Editable animation paths spline camera and focus positions for controlled fly-throughs.
- **Data-driven art direction:** render parameters can be constant values or field-mapped controls with input/output ranges, layer statistics, clamp, invert, and gamma shaping.
- **Hybrid capture scenes:** point-cloud layers and Gaussian splat layers share the same camera, scene, and project workflow, so survey geometry and photogrammetric/3DGS material can be composed together.
- **Postproduction-friendly output:** preview-density EXR stacks currently write `beauty.RGB`, `alpha.A`, and `depth.Z`, while Quick MP4 export gives fast review movies through `ffmpeg`.
- **Portable project state:** schema-73 project JSON, schema-22 animation JSON, and schema-27 standalone water-source JSON store authored scenes, explicit active water-scene ownership, animation, styles, water controls, live-view window preferences, and compact cache manifests; derived shared-surface and settled Flow-path payloads live in validated sidecars rather than bloating the project file.

## Current capabilities

### Scene and assets

- Discovers point-cloud and Gaussian splat assets from `Data/`.
- Loads CloudCompare-style binary PLY point clouds with RGB and optional `scalar_*` fields.
- Reads scalar-field statistics for field-driven styling.
- Groups sibling role-named PLY files into one folder-level scene, such as ROCK/SAND/VEG layers under `Data/ExhibitionScene`.
- Infers point spacing from filenames such as `1mm` and `2mm`, builds complete scene-wide density bundles, and exposes one **Visible Point Cloud** selector in the Visuals tab.
- Loads and commits the selected ROCK/SAND/VEG display bundle first. Canonical ROCK/VEG 1 mm and SAND 2 mm sources load CPU-only on demand for explicit Bake Path and analysis-based Ripple/Field operations; only the committed display bundle is renderable/GPU-resident.
- Treats the ROCK role as the primary visual/style reference in grouped scenes while keeping role-specific backend behavior available.
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
- Includes folder-level multi-cloud visuals so grouped ROCK/SAND/VEG scenes are edited as one cloud. Authored values stay on a 1 mm baseline while per-role footprint and measured-count coverage compensation makes 2/3/5 mm display bundles retain a similar appearance.
- Includes SAND-only shader shoreline waves for grouped coastal scenes, with crashing Foam Fronts, an always-active Continuous Bands bay of overlapping gentle crashes and mid-band fades, and height-driven foam. These use the editable boundary height, defaulting to `z = 1.55 m`, and do not require region polygons or CPU point membership.
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
- Extends both directions around both visual seams of two slow pans. A-start/B-end and B-start/A-end remain the original 50%-blend midpoints: each editable anchor/front/side triangle pair drives both a pre-roll before the source start and a tail after the matching destination end. A synchronized signed-offset review follows each seam, with an A/B toggle and a two-camera hard split whose adjustable boundary follows the tracked anchors even beyond frame. Water work is suppressed during the assistant. Each end receives two generated keys, or three for a crossed key/inflection. Final Preview can smooth abrupt pan/rotation accommodation across user-selected generated and neighbouring authored keys. The paired 50% cameras move together while a hard three-node screen constraint preserves their triangle alignment, allowing signed pan speed and rotation to remain smooth through the seam. The immutable fit remains available for Reset before both paths are applied atomically. Apply creates a reciprocal blend link when the pair was unlinked, or updates the existing pair, and writes the generated start/end spans as reciprocal blend durations. Save As on a pending pair saves both named copies plus the project atomically, preserving that link and its durations.
- Keeps every pre-extension timing, water, and keyed-effect event attached to the same old camera pose by affinely shifting/rescaling its normalized position when pre-roll and tail frames are added. Procedural waves, rain, and trails remain independent of camera position: they use the steady clock in live view, including while paused, and deterministic full output time during export.
- Stores a default live-view window size per animation and applies it when the animation loads; a project-level size lock overrides animation defaults.
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
- Field-driven controls currently cover point size, surfel diameter, opacity, emissive strength, depth fade, and colormap position.
- Saves and reloads point-cloud style presets.

### Export and persistence

- Saves and reloads schema-73 project JSON containing authoritative scene density groups, explicit active water-scene ownership, point-cloud styles, camera shots, animation paths, saved visuals, the live-view window preference and lock, water state/cache manifests, and export selections. Animation documents use schema 22 and standalone water-source documents use schema 27.
- Persists the shared schema-4 water surface cache as a scene-local `.surfacecache` and settled generated Flow branches as `.flowpathcache`, with project-local fallback paths when scene storage is unavailable.
- Builds the shared 10 mm Rain/Flow/Seepage surface from one complete 2 mm ROCK/SAND/VEG bundle (or the nearest complete fallback bundle), and folds the explicitly active scene's unambiguous 5 mm `MESHSampled` point cloud into a separate Ground-v3 tier for Mesh Flow. Density changes and effect tuning therefore do not rescan or mix role sources.
- Keeps Rain near-surface squish, ROCK/VEG response tuning, Flow trail visibility, and Seepage reach/width/prominence as live parameter updates over settled cache-derived support.
- Runs Mesh Flow automatically over the active scene's resident Ground table with a fixed 4,096-particle, 24-history GPU allocation. Dry particles emerge from vegetation-supported, convergent cells along each connected component's highest-+X edge; VEG Rain impacts feed a bounded GPU seed ring for distributed downhill emergence and recession. Ordinary Flow sources never seed Mesh Flow, and live activity, moisture, and appearance edits neither rebuild the cache nor resize GPU storage.
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
- Multi-cloud scene folders may contain sibling role-named point clouds. `ROCK`, `SAND`, and `VEG` tokens define the role; `1mm`, `2mm`, and similar tokens define inferred point spacing in meters.
- A display spacing is selectable only when exactly one ROCK, SAND, and VEG file exists at that spacing. Incomplete or duplicate density sets are rejected rather than mixed during switching.
- `Data/Scene3` is the default combined full-site scene when `Saved/ExhibitionFinal_project.json` exists. Canonical analysis uses `Site3-ROCK-1mm.ply`, `Site3-SAND-2mm.ply`, and `Site3-VEG-1mm.ply`; its complete same-spacing sets provide the selectable display densities.
- `Data/SampleScene` is the local validation fixture for the same multi-cloud contract. It contains complete `Site1-{ROCK,SAND,VEG}-{1,2,3,5}mm. SampleScene.ply` display bundles, `Site1-Mesh-SampleScene.ply`, and the 5 mm `Site1-MeshSampled-5mm-SampleScene.ply` Ground input; validation displays the 3 mm bundle, keeps canonical 1/2/1 mm paths available for on-demand analysis, and streams the complete 2 mm terrain bundle plus the scene-local sampled Ground cloud into the shared 10 mm water-surface cache.
- Gaussian splat files are PLY files whose filename starts with `gSplat-` and whose header exposes Gaussian attributes such as `f_dc_0`, `opacity`, `scale_0`, and `rot_0`.
- Each gSplat file is paired with a same-stem `.txt` file containing a 4x4 transform matrix.
- `ffmpeg` is expected at `/opt/homebrew/bin/ffmpeg` for Fast Preview / Quick MP4 export.

See [Scene-Wide Point-Cloud Density Switching](docs/scene_point_cloud_density_switching.md) for the scene catalog, transactional loading, rendering compensation, water routing, and current schema contracts.

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

The ExhibitionScene multi-cloud workflow was additionally validated locally on July 4, 2026:

- `Data/SampleScene` discovered as one grouped ROCK/SAND/VEG scene with complete `1mm`, `2mm`, `3mm`, and `5mm` density bundles.
- `scripts/generate_sample_scene_validation.py` rebuilds `Saved/validation/SampleSceneValidation_project.json` from the durable `tests/fixtures/sample_scene_water_sources.json` objects (`SampleFlowPoint`, `SampleFlowPath`, and `SampleSeepage`) without embedding derived caches.
- Native integration scenarios are `--gui-smoke water-integration-sample-scene` for the lightweight validation project and `--gui-smoke water-integration-scene3-exhibition` for the final Scene3 project and its `Exhibition` animation. The full-site scenario uses the live 5 mm display and is registered only with `-DINVISIBLE_PLACES_ENABLE_FULL_SITE_WATER_STRESS_TESTS=ON` because it may build the canonical shared-water cache.
- `Saved/ExhibitionFinal_project.json` loads the combined Scene3 project by default when present. Startup falls back to `Saved/exhibitionScene_project.json`, then `Saved/invisible_places_project.json`.
- `RGB-Ghost`, `Roughness`, and `ghosted` are folder-level point visuals with density-compensated role rendering.
- SAND shoreline waves compile through the point-cloud shader path and are covered by focused shoreline tests.
