#pragma once

#include "camera/AnimationPath.hpp"
#include "camera/CameraShot.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::output {

struct RenderPreset {
    std::uint32_t width = 1920;
    std::uint32_t height = 1080;
    bool tiledRendering = false;
    bool exportDepth = true;
    bool exportAlpha = true;
};

struct RenderJobSettings {
    std::string outputDirectory;
    std::uint32_t width = 1920;
    std::uint32_t height = 1080;
    std::uint32_t framesPerSecond = 30;
    float stillCameraDurationSeconds = 5.0F;
    std::uint32_t tileSize = 512;
    std::uint32_t startFrame = 0;
    std::uint32_t endFrame = 0;
    std::size_t fromShotIndex = 0;
    std::size_t toShotIndex = 1;
};

enum class AnimationExportMode {
    FastPreviewMp4,
    HqAdaptiveExr,
};

enum class PointCloudExportDensityMode {
    FullSource,
    AdaptiveHighQuality,
    MatchViewportAdaptive,
    FastAdaptivePreview,
    ArtisticAsPreview,
    ArtisticHighQuality,
};

const char* PointCloudExportDensityModeName(PointCloudExportDensityMode mode);
bool PointCloudExportDensityModeUsesFullSource(PointCloudExportDensityMode mode);

std::vector<invisible_places::camera::CameraState> BuildCameraRenderSequence(
    const std::vector<invisible_places::camera::CameraShot>& shots,
    const RenderJobSettings& settings);

std::vector<invisible_places::camera::CameraState> BuildAnimationRenderSequence(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings);

std::vector<invisible_places::camera::CameraState> BuildStillCameraRenderSequence(
    const invisible_places::camera::CameraState& cameraState,
    const RenderJobSettings& settings);

float ComputePointSizePixelScale(
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    std::uint32_t setupViewportWidth,
    std::uint32_t setupViewportHeight);

std::filesystem::path RenderFramePath(
    const RenderJobSettings& settings,
    std::uint32_t frameIndex);

}  // namespace invisible_places::output
