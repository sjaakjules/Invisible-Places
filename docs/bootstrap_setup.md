# Bootstrap Setup Notes

## Current local findings

The current workspace already contains representative data:

- CloudCompare point clouds in `Data/*.ply`
- Gaussian splat PLY assets in `Data/gSplat-*.ply`
- same-stem 4x4 transforms in `Data/gSplat-*.txt`

The current machine also already has:

- Apple Clang 17
- CMake 3.30.4
- Homebrew Vulkan loader headers

The following `vcpkg` setup was validated locally:

- Homebrew installs a working `vcpkg` executable at `/opt/homebrew/bin/vcpkg`
- that executable still needs a cloned root, because manifest mode and the CMake toolchain file are not bundled into the Homebrew shim alone
- cloning `https://github.com/microsoft/vcpkg` to `~/vcpkg` provides the required root and `scripts/buildsystems/vcpkg.cmake`
- the repository configured successfully with `VCPKG_ROOT="$HOME/vcpkg" cmake --preset macos-debug-vcpkg`

The machine is still not ready for renderer work yet:

- `glslc` is not installed
- `vulkaninfo` previously failed to create a Vulkan instance through the older Homebrew MoltenVK path
- real render-path validation still depends on choosing and integrating the GS runtime codebase

## Why Vulkan parity matters here

To keep preview and final output visually aligned:

- use Vulkan as the renderer API on both platforms,
- use the same shader logic and parameter binding model on both platforms,
- keep MoltenVK as the macOS portability layer only,
- avoid a separate Metal renderer path unless the project later proves Vulkan-on-MoltenVK is a blocker.

That gives the M1 machine and the final Windows GPU machine the best chance of matching shot framing, field mapping, AOV behavior, and tone response.

## Recommended macOS toolchain fix order

1. Install a full Vulkan SDK that includes validation layers and shader tools.
2. Set `VULKAN_SDK` so CMake can resolve the SDK consistently.
3. Install the Homebrew `vcpkg` executable, clone `~/vcpkg`, and either export `VCPKG_ROOT="$HOME/vcpkg"` or use the `macos-debug-home-vcpkg` preset.
4. Configure with the `macos-debug-home-vcpkg` preset.
5. Validate that `vulkaninfo` works before investing in render-path code.

## Open integration decision

Real Gaussian splat integration is part of the plan, but the exact external codebase still needs to be pinned. The project docs mention `3dgs-vulkan-cpp` or a close derivative; once that repo or fork is chosen, it should be the one approved `FetchContent` exception if vcpkg does not package it.
