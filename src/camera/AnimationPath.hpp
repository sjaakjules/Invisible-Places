#pragma once

#include "camera/CameraShot.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "timing/TimingColourise.hpp"
#include "water/WaterFlow.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
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
    std::string selectedTimingTakeId =
        std::string{invisible_places::timing::kAuthoredTimingTakeId};
    // Retained for legacy animation round-trip. Runtime timing resolves
    // selectedTimingTakeId instead.
    std::string selectedWaterScenarioId;
    std::vector<invisible_places::water::WaterScenarioTrack> waterScenarioTracks;
    std::optional<invisible_places::water::WaterAnimationTrailSettings> waterAnimationTrailSettings;
    std::optional<invisible_places::water::WaterAnimationTrailSettings> tempWaterAnimationTrailSettings;
    std::optional<invisible_places::renderer::pointcloud::PointCloudStyleState> waterPointVisualStyle;
    std::optional<invisible_places::renderer::pointcloud::PointCloudStyleState> tempWaterPointVisualStyle;
    std::optional<invisible_places::water::WaterCausticLookSettings> waterCausticLookSettings;
    std::optional<invisible_places::water::WaterCausticLookSettings> tempWaterCausticLookSettings;
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

struct AnimationPreparedScalarSpline {
    std::vector<float> values;
    std::vector<float> secondDerivatives;
};

struct PreparedAnimationPathEvaluationContext {
    bool valid = false;
    bool singleKey = false;
    float durationSeconds = 0.0F;
    bool depthOfFieldEnabled = false;
    float apertureFStops = 8.0F;
    float depthOfFieldMaxBlurPixels = 24.0F;
    bool hasOrientation = false;
    bool hasFocusDistance = false;
    bool hasApertureFStops = false;
    AnimationPathKey singleKeySnapshot{};
    std::vector<float> knots;
    AnimationPreparedScalarSpline cameraX;
    AnimationPreparedScalarSpline cameraY;
    AnimationPreparedScalarSpline cameraZ;
    AnimationPreparedScalarSpline focusX;
    AnimationPreparedScalarSpline focusY;
    AnimationPreparedScalarSpline focusZ;
    AnimationPreparedScalarSpline fovDegrees;
    AnimationPreparedScalarSpline nearPlane;
    AnimationPreparedScalarSpline farPlane;
    AnimationPreparedScalarSpline focusDistance;
    AnimationPreparedScalarSpline apertureFStopsSpline;
    std::vector<std::array<float, 4>> orientationQuaternions;
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
[[nodiscard]] PreparedAnimationPathEvaluationContext PrepareAnimationPathEvaluation(
    const AnimationPath& path);
[[nodiscard]] AnimationPathEvaluation EvaluatePreparedAnimationPath(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds);
[[nodiscard]] AnimationPathMotionStats MeasureAnimationPathMotion(
    const AnimationPath& path,
    float normalizedTime,
    std::uint32_t sampleCount = 240U);
[[nodiscard]] AnimationPathMotionStats MeasurePreparedAnimationPathMotion(
    const PreparedAnimationPathEvaluationContext& context,
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

// Surface-focus probing. CollectRayHitDistancesAlongRay appends the along-ray
// distance of every point lying within perpendicularRadiusMeters of the ray
// (direction must be normalized) and inside [minDistanceMeters,
// maxDistanceMeters]. ResolveFirstRayHitCluster then picks the nearest depth
// cluster holding at least minimumClusterSamples candidates so one stray
// point floating in front of the terrain cannot claim focus; when no cluster
// qualifies it falls back to the nearest candidate.
void CollectRayHitDistancesAlongRay(
    std::span<const invisible_places::io::Float3> points,
    const std::array<float, 3>& origin,
    const std::array<float, 3>& normalizedDirection,
    float perpendicularRadiusMeters,
    float minDistanceMeters,
    float maxDistanceMeters,
    std::vector<float>* distances);
[[nodiscard]] std::optional<float> ResolveFirstRayHitCluster(
    std::vector<float> distances,
    float clusterDepthMeters,
    std::size_t minimumClusterSamples);

void MoveAnimationCameraKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& cameraPosition);

void MoveAnimationFocusKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& focusPoint);

}  // namespace invisible_places::camera
