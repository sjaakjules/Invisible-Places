#include "output/RenderPreset.hpp"

#include "camera/CameraPath.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace invisible_places::output {

namespace {

std::uint32_t ScaleThirtyFpsFramesToOutputFps(
    std::uint32_t sourceDurationFrames,
    std::uint32_t framesPerSecond) {
    return std::max<std::uint32_t>(
        1U,
        static_cast<std::uint32_t>(
            ((static_cast<std::uint64_t>(std::max<std::uint32_t>(1U, sourceDurationFrames)) *
              std::max<std::uint32_t>(1U, framesPerSecond)) +
             15ULL) /
            30ULL));
}

std::vector<invisible_places::camera::CameraState> SliceFrameRange(
    const std::vector<invisible_places::camera::CameraState>& frames,
    const RenderJobSettings& settings) {
    if (frames.empty()) {
        return {};
    }

    const std::uint32_t startFrame = std::min<std::uint32_t>(
        settings.startFrame,
        static_cast<std::uint32_t>(frames.size() - 1U));
    const std::uint32_t endFrame = settings.endFrame == 0
                                       ? static_cast<std::uint32_t>(frames.size() - 1U)
                                       : std::min<std::uint32_t>(
                                             settings.endFrame,
                                             static_cast<std::uint32_t>(frames.size() - 1U));
    if (startFrame > endFrame) {
        return {};
    }

    return std::vector<invisible_places::camera::CameraState>{
        frames.begin() + static_cast<std::ptrdiff_t>(startFrame),
        frames.begin() + static_cast<std::ptrdiff_t>(endFrame + 1U)};
}

}  // namespace

const char* PointCloudExportDensityModeName(PointCloudExportDensityMode mode) {
    switch (mode) {
        case PointCloudExportDensityMode::FullSource:
            return "Full Source";
        case PointCloudExportDensityMode::AdaptiveHighQuality:
            return "Adaptive High Quality";
        case PointCloudExportDensityMode::MatchViewportAdaptive:
            return "Match Viewport Adaptive";
        case PointCloudExportDensityMode::FastAdaptivePreview:
            return "Fast Adaptive Preview";
        case PointCloudExportDensityMode::ArtisticAsPreview:
            return "Artistic As Preview";
        case PointCloudExportDensityMode::ArtisticHighQuality:
            return "Artistic High Quality";
    }

    return "Adaptive High Quality";
}

bool PointCloudExportDensityModeUsesFullSource(PointCloudExportDensityMode mode) {
    switch (mode) {
        case PointCloudExportDensityMode::FullSource:
            return true;
        case PointCloudExportDensityMode::AdaptiveHighQuality:
        case PointCloudExportDensityMode::MatchViewportAdaptive:
        case PointCloudExportDensityMode::FastAdaptivePreview:
        case PointCloudExportDensityMode::ArtisticAsPreview:
        case PointCloudExportDensityMode::ArtisticHighQuality:
            return false;
    }

    return false;
}

std::vector<invisible_places::camera::CameraState> BuildCameraRenderSequence(
    const std::vector<invisible_places::camera::CameraShot>& shots,
    const RenderJobSettings& settings) {
    if (shots.size() < 2) {
        return {};
    }

    const std::size_t fromIndex = std::min(settings.fromShotIndex, shots.size() - 1U);
    const std::size_t toIndex = std::min(settings.toShotIndex, shots.size() - 1U);
    if (fromIndex >= toIndex) {
        return {};
    }

    const auto timing = invisible_places::camera::BuildCameraPathTiming(shots, fromIndex, toIndex);
    if (!timing.IsValid()) {
        return {};
    }

    std::vector<invisible_places::camera::CameraState> frames;
    for (std::size_t shotIndex = fromIndex; shotIndex < toIndex; ++shotIndex) {
        const auto& toShot = shots[shotIndex + 1U];
        const auto sourceDurationFrames = std::max<std::uint32_t>(1U, toShot.durationFrames);
        const auto durationFrames = ScaleThirtyFpsFramesToOutputFps(sourceDurationFrames, settings.framesPerSecond);
        const auto timingIndex = shotIndex - fromIndex;
        const float segmentStartSeconds = timing.knotSeconds[timingIndex];
        const float segmentEndSeconds = timing.knotSeconds[timingIndex + 1U];
        for (std::uint32_t frameIndex = 0; frameIndex < durationFrames; ++frameIndex) {
            const float t = static_cast<float>(frameIndex) / static_cast<float>(durationFrames);
            const float timeSeconds =
                segmentStartSeconds + ((segmentEndSeconds - segmentStartSeconds) * t);
            frames.push_back(invisible_places::camera::EvaluateCameraPath(shots, timing, timeSeconds));
        }
    }
    frames.push_back(invisible_places::camera::EvaluateCameraPath(shots, timing, timing.DurationSeconds()));

    return SliceFrameRange(frames, settings);
}

std::vector<invisible_places::camera::CameraState> BuildAnimationRenderSequence(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings) {
    if (path.keys.empty()) {
        return {};
    }

    const auto minimumFrames = path.keys.size() > 1U
                                   ? static_cast<std::uint32_t>(path.keys.size() - 1U)
                                   : 1U;
    const auto sourceDurationFrames = std::max(path.durationFrames, minimumFrames);
    const auto outputDurationFrames =
        ScaleThirtyFpsFramesToOutputFps(sourceDurationFrames, settings.framesPerSecond);
    const float durationSeconds = invisible_places::camera::AnimationPathDurationSeconds(path);

    std::vector<invisible_places::camera::CameraState> frames;
    frames.reserve(static_cast<std::size_t>(outputDurationFrames) + 1U);
    for (std::uint32_t frameIndex = 0; frameIndex < outputDurationFrames; ++frameIndex) {
        const float t = static_cast<float>(frameIndex) / static_cast<float>(outputDurationFrames);
        frames.push_back(invisible_places::camera::EvaluateAnimationPath(path, durationSeconds * t).camera);
    }
    frames.push_back(invisible_places::camera::EvaluateAnimationPath(path, durationSeconds).camera);

    return SliceFrameRange(frames, settings);
}

std::vector<invisible_places::camera::CameraState> BuildStillCameraRenderSequence(
    const invisible_places::camera::CameraState& cameraState,
    const RenderJobSettings& settings) {
    const auto frameCount = std::max<std::uint32_t>(
        1U,
        static_cast<std::uint32_t>(
            std::ceil(
                std::max(0.001F, settings.stillCameraDurationSeconds) *
                static_cast<float>(std::max<std::uint32_t>(1U, settings.framesPerSecond)))));
    return std::vector<invisible_places::camera::CameraState>(frameCount, cameraState);
}

float ComputePointSizePixelScale(
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    std::uint32_t setupViewportWidth,
    std::uint32_t setupViewportHeight) {
    if (outputWidth == 0 ||
        outputHeight == 0 ||
        setupViewportWidth == 0 ||
        setupViewportHeight == 0) {
        return 1.0F;
    }

    const float widthScale = static_cast<float>(outputWidth) / static_cast<float>(setupViewportWidth);
    const float heightScale = static_cast<float>(outputHeight) / static_cast<float>(setupViewportHeight);
    return std::max(0.001F, std::sqrt(std::max(0.0F, widthScale * heightScale)));
}

std::filesystem::path RenderFramePath(
    const RenderJobSettings& settings,
    std::uint32_t frameIndex) {
    std::ostringstream filename;
    filename << "frame_" << std::setw(6) << std::setfill('0') << (frameIndex + 1U) << ".exr";
    return std::filesystem::path{settings.outputDirectory} / filename.str();
}

}  // namespace invisible_places::output
