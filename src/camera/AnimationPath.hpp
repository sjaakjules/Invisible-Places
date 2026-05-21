#pragma once

#include "camera/CameraShot.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "water/WaterFlow.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace invisible_places::camera {

struct AnimationPathKey {
    std::string id;
    std::array<float, 3> cameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 3> focusPoint{0.0F, 0.0F, 0.0F};
    bool hasOrientation = false;
    std::array<float, 4> orientation{0.0F, 0.0F, 0.0F, 1.0F};
    bool hasFocusDistance = false;
    float focusDistance = 1.0F;
    bool hasApertureFStops = false;
    float apertureFStops = 8.0F;
    float fovDegrees = 60.0F;
    float nearPlane = 0.01F;
    float farPlane = 1000.0F;
    std::uint32_t durationFrames = 90;
    std::string sourceShotName;
    std::string linkedCameraId;
    std::string linkedCameraName;
};

struct AnimationExportSettings {
    std::string outputDirectory;
    std::uint32_t width = 1920;
    std::uint32_t height = 1080;
    std::uint32_t framesPerSecond = 30;
    float stillCameraDurationSeconds = 5.0F;
    std::uint32_t startFrame = 0;
    std::uint32_t endFrame = 0;
};

struct AnimationPath {
    std::string name = "Animation";
    std::uint32_t durationFrames = 180;
    std::vector<AnimationPathKey> keys;
    std::vector<std::filesystem::path> associatedLayerPaths;
    bool depthOfFieldEnabled = false;
    float apertureFStops = 8.0F;
    float depthOfFieldMaxBlurPixels = 24.0F;
    AnimationExportSettings exportSettings{};
    std::vector<std::string> exportVisualNames;
    std::optional<invisible_places::water::WaterAnimationTrailSettings> waterAnimationTrailSettings;
    std::optional<invisible_places::water::WaterAnimationTrailSettings> tempWaterAnimationTrailSettings;
    std::optional<invisible_places::renderer::pointcloud::PointCloudStyleState> waterPointVisualStyle;
    std::optional<invisible_places::renderer::pointcloud::PointCloudStyleState> tempWaterPointVisualStyle;
    std::optional<invisible_places::water::WaterVisualSettings> waterVisualSettings;
    std::optional<invisible_places::water::WaterVisualSettings> tempWaterVisualSettings;
    std::optional<invisible_places::water::WaterSettingsBundle> waterSettings;
    std::optional<invisible_places::water::WaterSettingsBundle> tempWaterSettings;
};

struct AnimationPathEvaluation {
    CameraState camera;
    std::array<float, 3> focusPoint{0.0F, 0.0F, 0.0F};
    float focusDistance = 1.0F;
};

enum class AnimationPathMotionTarget {
    Camera,
    Target
};

struct AnimationPathMotionStats {
    float durationSeconds = 0.0F;
    float cameraDistance = 0.0F;
    float targetDistance = 0.0F;
    float averageCameraSpeed = 0.0F;
    float averageTargetSpeed = 0.0F;
    float currentCameraSpeed = 0.0F;
    float currentTargetSpeed = 0.0F;
};

AnimationPath BuildAnimationPathFromCameraShots(
    const std::string& name,
    const std::vector<CameraShot>& orderedShots,
    std::uint32_t durationFrames,
    float apertureFStops = 8.0F);

[[nodiscard]] float AnimationPathDurationSeconds(const AnimationPath& path);
[[nodiscard]] AnimationPathMotionStats MeasureAnimationPathMotion(
    const AnimationPath& path,
    float normalizedTime,
    std::uint32_t sampleCount = 240U);
[[nodiscard]] std::uint32_t AnimationDurationFramesForAverageSpeed(
    const AnimationPath& path,
    AnimationPathMotionTarget target,
    float worldUnitsPerSecond,
    std::uint32_t sampleCount = 240U);

AnimationPathEvaluation EvaluateAnimationPath(
    const AnimationPath& path,
    float timeSeconds);

void MoveAnimationCameraKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& cameraPosition);

void MoveAnimationFocusKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& focusPoint);

}  // namespace invisible_places::camera
