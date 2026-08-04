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

struct AnimationLoopSmoothingMetadata {
    std::string pairId;
    std::string partnerFileName;
    std::uint32_t sequenceIndex = 0U;
    float maxEndMoveFraction = 0.10F;
    std::string firstKeyId;
    std::string lastKeyId;
    std::array<float, 3> originalFirstCameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 3> originalFirstFocusPoint{0.0F, 0.0F, 0.0F};
    std::array<float, 3> originalLastCameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 3> originalLastFocusPoint{0.0F, 0.0F, 0.0F};
};

// A linked loop is a renderable snapshot compiled from two source animation
// files. The source names and phase controls remain attached so the snapshot
// can be rebuilt after either source changes. Negative padding overlaps and
// crossfades both source clips; positive padding holds the outgoing endpoint
// before the next clip begins.
struct AnimationLinkedLoopMetadata {
    std::string firstFileName;
    std::string secondFileName;
    std::string firstStartKeyId;
    float firstStartPosition = 0.0F;
    std::int32_t paddingFrames = 0;
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
    // Present only while reversible loop-end smoothing is applied. The
    // visible endpoint keys contain the adjusted poses; these originals are
    // used to evaluate the unchanged middle spline and to unapply exactly.
    std::optional<AnimationLoopSmoothingMetadata> loopTransitionSmoothing;
    std::optional<AnimationLinkedLoopMetadata> linkedLoop;
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
    bool hasLoopEndpointCorrections = false;
    std::array<float, 3> firstCameraCorrection{0.0F, 0.0F, 0.0F};
    std::array<float, 3> firstFocusCorrection{0.0F, 0.0F, 0.0F};
    std::array<float, 3> lastCameraCorrection{0.0F, 0.0F, 0.0F};
    std::array<float, 3> lastFocusCorrection{0.0F, 0.0F, 0.0F};
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

struct AnimationPerceivedFlowSample {
    float normalizedPosition = 0.0F;
    float screenSpeed = 0.0F;
    // Signed image-plane movement in screen heights per second. X is screen
    // right and Y is screen up; its length equals screenSpeed when a stable
    // direction can be resolved.
    std::array<float, 2> screenVelocity{0.0F, 0.0F};
};

struct AnimationLoopSmoothingOptions {
    float maxEndMoveFraction = 0.10F;
    std::string pairId;
    std::string firstFileName;
    std::string secondFileName;
};

struct AnimationLinkedLoopBuildOptions {
    std::string name;
    std::string firstFileName;
    std::string secondFileName;
    std::size_t firstStartKeyIndex = 0U;
    std::int32_t paddingFrames = 0;
};

struct AnimationLinkedLoopTiming {
    std::uint32_t firstDurationFrames = 0U;
    std::uint32_t secondDurationFrames = 0U;
    std::uint32_t firstTerminalStartFrame = 0U;
    std::uint32_t secondTerminalStartFrame = 0U;
    std::int32_t paddingFrames = 0;
    std::uint32_t periodFrames = 0U;
    float secondStartFrame = 0.0F;
};

struct AnimationLinkedLoopSourceSample {
    AnimationPathEvaluation blended;
    AnimationPathEvaluation first;
    AnimationPathEvaluation second;
    bool valid = false;
    bool firstActive = false;
    bool secondActive = false;
    float firstWeight = 0.0F;
    float secondWeight = 0.0F;
};

struct AnimationLoopSmoothingResult {
    bool succeeded = false;
    bool changed = false;
    float beforeMismatch = 0.0F;
    float afterMismatch = 0.0F;
    std::array<float, 2> beforeSeamMismatch{0.0F, 0.0F};
    std::array<float, 2> afterSeamMismatch{0.0F, 0.0F};
    // Weighted RMS change from each animation's original terminal perceived-
    // speed curve, normalized by the local seam speed. Index 0 is `first`.
    std::array<float, 2> terminalSpeedRmsChange{0.0F, 0.0F};
    float maxCameraMove = 0.0F;
    float maxFocusMove = 0.0F;
    float maxCameraCapUsage = 0.0F;
    float maxFocusCapUsage = 0.0F;
    std::string errorMessage;
};

// Read-only validation for a selected loop pair. With no applied smoothing,
// before and after are the same current baseline. With matching reversible
// metadata, the preserved endpoints reconstruct the exact pre-smoothing
// baseline so the improvement remains measurable after saving and restarting.
struct AnimationLoopTransitionMetrics {
    bool valid = false;
    bool hasAppliedSmoothing = false;
    float beforeMismatch = 0.0F;
    float afterMismatch = 0.0F;
    std::array<float, 2> beforeSeamMismatch{0.0F, 0.0F};
    std::array<float, 2> afterSeamMismatch{0.0F, 0.0F};
    std::array<float, 2> terminalSpeedRmsChange{0.0F, 0.0F};
    float maxCameraMove = 0.0F;
    float maxFocusMove = 0.0F;
    float maxCameraCapUsage = 0.0F;
    float maxFocusCapUsage = 0.0F;
    std::string errorMessage;
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

// Perceived-motion probing. MeasurePreparedAnimationPathPerceivedFlow samples
// an optical-flow proxy across normalized time 0..1: angular view speed plus
// the view-perpendicular camera speed divided by the focus distance, all
// normalized by the vertical field of view so screenSpeed reads in screen
// heights per second. Degenerate paths (invalid context, a single key, or a
// zero duration) still return the requested sample count with screenSpeed 0.
// ComputeConstantPerceivedSpeedSegmentFrames redistributes the path's total
// frames (max(path.durationFrames, keys - 1), preserved exactly via
// largest-remainder rounding) so each segment's share is proportional to its
// midpoint-rule integrated perceived flow, holding perceived speed roughly
// constant. Every segment keeps at least one frame; ~zero integrated flow
// falls back to an even split; fewer than two keys or zero
// path.durationFrames returns an empty vector.
[[nodiscard]] std::vector<AnimationPerceivedFlowSample> MeasurePreparedAnimationPathPerceivedFlow(
    const PreparedAnimationPathEvaluationContext& context,
    std::uint32_t sampleCount = 160U);
[[nodiscard]] std::vector<std::uint32_t> ComputeConstantPerceivedSpeedSegmentFrames(
    const AnimationPath& path,
    std::uint32_t samplesPerSegment = 24U);

// Builds one closed, phase-rotated animation from two source paths. The
// resulting path contains one ordinary key per 30 fps source frame, so it can
// be previewed and exported without the source files being present. At zero
// padding, each incoming first key aligns with the outgoing penultimate key
// and the complete terminal edge is smooth-crossfaded. Signed padding offsets
// that anchor; a positive value becomes an endpoint hold only after it exceeds
// the remaining terminal-edge duration.
[[nodiscard]] std::int32_t ClampLinkedLoopPaddingFrames(
    const AnimationPath& first,
    const AnimationPath& second,
    std::int32_t requestedPaddingFrames);
[[nodiscard]] AnimationLinkedLoopTiming ResolveLinkedLoopTiming(
    const AnimationPath& first,
    const AnimationPath& second,
    std::int32_t requestedPaddingFrames);
[[nodiscard]] float AnimationPathKeyNormalizedPosition(
    const AnimationPath& path,
    std::size_t keyIndex);
[[nodiscard]] AnimationLinkedLoopSourceSample EvaluateLinkedLoopSourceSample(
    const AnimationPath& first,
    const AnimationPath& second,
    float firstStartPosition,
    std::int32_t paddingFrames,
    float linkedNormalizedPosition);
[[nodiscard]] std::optional<AnimationPath> BuildLinkedLoopAnimation(
    const AnimationPath& first,
    const AnimationPath& second,
    const AnimationLinkedLoopBuildOptions& options,
    std::string* errorMessage = nullptr);

// Jointly adjusts only the first and last camera/focus keys of two paths.
// durationFrames and every per-key durationFrames value are never written.
// Applied paths evaluate their original natural cubic spline everywhere,
// plus a quadratic correction confined to their first and last segments.
[[nodiscard]] AnimationLoopSmoothingResult SmoothAnimationLoopTransitions(
    AnimationPath* first,
    AnimationPath* second,
    const AnimationLoopSmoothingOptions& options = {});
[[nodiscard]] AnimationLoopTransitionMetrics MeasureAnimationLoopTransitions(
    const AnimationPath& first,
    const AnimationPath& second);
[[nodiscard]] bool UnapplyAnimationLoopSmoothing(
    AnimationPath* path,
    std::string* errorMessage = nullptr);

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
