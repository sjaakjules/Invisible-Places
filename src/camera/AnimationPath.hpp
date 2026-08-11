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
#include <stop_token>
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

struct AnimationLocalizedKeyCorrection {
    std::string keyId;
    std::array<float, 3> splineCameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 3> splineFocusPoint{0.0F, 0.0F, 0.0F};
};

struct AnimationVelocityBlendLinkMetadata {
    std::string pairId;
    std::string partnerFileName;
    float maxEndMoveFraction = 0.10F;
    float strongAlignMaxMoveFraction = 0.50F;
    // Path-local overlap extents. This path's start overlaps the partner's
    // end and this path's end overlaps the partner's start for the same
    // number of seconds.
    float startOverlapSeconds = 0.0F;
    float endOverlapSeconds = 0.0F;
    // When enabled, seam scoring follows a horizontal cross-wipe instead of
    // treating the complete frame as equally visible. A rightward pan keeps
    // the outgoing left side and introduces the incoming right side; a
    // leftward pan mirrors those regions.
    bool horizontalBlend = false;
    bool panRight = true;
    std::vector<std::string> movableKeyIds;
};

struct AnimationPath {
    // Original file schema retained only for application-level migration
    // bookkeeping. Serialization always writes the current schema.
    std::uint32_t sourceSchemaVersion = 19U;
    std::string name = "Animation";
    std::uint32_t durationFrames = 180;
    std::vector<AnimationPathKey> keys;
    std::vector<std::filesystem::path> associatedLayerPaths;
    bool depthOfFieldEnabled = false;
    float apertureFStops = 8.0F;
    float depthOfFieldMaxBlurPixels = 24.0F;
    AnimationExportSettings exportSettings{};
    // Pair ownership and localized path evaluation are intentionally
    // independent. A confirmed pose remains localized after the animation is
    // unlinked or paired with another animation.
    std::optional<AnimationVelocityBlendLinkMetadata> velocityBlendLink;
    std::vector<AnimationLocalizedKeyCorrection> localizedKeyCorrections;
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

// One rigid source-camera -> destination-camera frame transform. It is the
// same mapping used by the matching-point ghost: no point-specific warping
// occurs. Applying it to a complete partner spline therefore shows exactly
// how that path is positioned around the destination animation frame.
struct AnimationCameraFrameTransform {
    std::array<float, 3> sourceCameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 3> destinationCameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 4> sourceToDestinationRotation{
        0.0F,
        0.0F,
        0.0F,
        1.0F,
    };
};

struct AnimationPreparedScalarSpline {
    std::vector<float> values;
    std::vector<float> secondDerivatives;
};

struct PreparedAnimationPathEvaluationContext {
    bool valid = false;
    bool singleKey = false;
    float durationSeconds = 0.0F;
    float aspectRatio = 16.0F / 9.0F;
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
    // Localized confirmed corrections. The authored path's preserved
    // spline stays the base; compact cubic-Hermite offsets affect only
    // segments touching a user-enabled key and terminate with zero velocity
    // at every locked key.
    bool hasLoopKeyCorrections = false;
    std::vector<std::uint8_t> loopCorrectionKeyEnabled;
    std::vector<std::array<float, 3>> loopCameraCorrections;
    std::vector<std::array<float, 3>> loopFocusCorrections;
    std::vector<std::array<float, 3>> loopCameraCorrectionTangents;
    std::vector<std::array<float, 3>> loopFocusCorrectionTangents;
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
    // Diagnostic focus-plane flow. The legacy scalar and signed velocity
    // above stay unchanged so perceived-speed equalization and velocity-blend
    // scoring retain their established behavior.
    std::array<float, 2> topScreenVelocity{0.0F, 0.0F};
    std::array<float, 2> middleScreenVelocity{0.0F, 0.0F};
    std::array<float, 2> bottomScreenVelocity{0.0F, 0.0F};
    float imageRotationDegreesPerSecond = 0.0F;
};

enum class AnimationSpeedEqualizationMode : std::uint8_t {
    // Preserves the established angular-view plus focus-distance-scaled
    // sideways-motion proxy exactly.
    PerceivedMotion = 0,
    // Uses the projected motion at the centre of the focus plane. This is
    // useful when the viewer's attention stays near the centre of a pan.
    CenterScreenPan,
    // Adds penalties for motion away from the dominant pan axis, focus-plane
    // flow variation, and image roll so visually complex sections receive
    // more time.
    StabilizedPan,
};

struct AnimationSpeedEqualizationOptions {
    AnimationSpeedEqualizationMode mode =
        AnimationSpeedEqualizationMode::PerceivedMotion;
    std::uint32_t samplesPerSegment = 24U;
    float offAxisWeight = 0.75F;
    float flowVariationWeight = 0.50F;
    float rollWeight = 0.25F;
};

struct AnimationVelocitySpatialEqualizationResult {
    bool succeeded = false;
    bool changed = false;
    std::size_t movedKeyCount = 0U;
    float totalScreenTravel = 0.0F;
    std::string errorMessage;
};

struct AnimationCameraSpatialSmoothingOptions {
    // When false, the optimizer minimizes image rotation and direction
    // reversals. When true, it also strongly penalizes variation from the
    // animation's signed mean centre-screen X velocity.
    bool equalizeScreenXVelocity = false;
    std::uint32_t sampleCount = 129U;
    std::uint32_t optimizationSweeps = 18U;
};

struct AnimationCameraSpatialSmoothingResult {
    bool succeeded = false;
    bool changed = false;
    std::size_t movedKeyCount = 0U;
    float beforeRotationRmsDegreesPerSecond = 0.0F;
    float afterRotationRmsDegreesPerSecond = 0.0F;
    float beforeXVelocityDeviation = 0.0F;
    float afterXVelocityDeviation = 0.0F;
    std::size_t beforeRotationDirectionChanges = 0U;
    std::size_t afterRotationDirectionChanges = 0U;
    std::string errorMessage;
};

struct AnimationLoopSmoothingOptions {
    float maxEndMoveFraction = 0.10F;
    std::string pairId;
    std::string firstFileName;
    std::string secondFileName;
    // Omitted selections preserve the legacy endpoint-only behavior used by
    // older callers. The UI sets this true even if one path has no enabled
    // keys, allowing camera-A-only or camera-B-only correction.
    bool useExplicitKeySelection = false;
    std::vector<std::string> firstMovableKeyIds;
    std::vector<std::string> secondMovableKeyIds;
    float firstStartOverlapSeconds = 0.0F;
    float firstEndOverlapSeconds = 0.0F;
    float secondStartOverlapSeconds = 0.0F;
    float secondEndOverlapSeconds = 0.0F;
    // Horizontal cross-wipe scoring compares the portions that remain
    // visible during the overlap: outgoing 2/3 -> 1/3 and incoming
    // 1/3 -> 2/3. `panRight` selects outgoing-left/incoming-right; false
    // mirrors the regions.
    bool horizontalBlend = false;
    bool panRight = true;
    // A small value is used only for the interactive rough preview. Full
    // apply retains the default multi-resolution search.
    std::uint32_t maxOptimizationSweeps = 40U;
    float minimumStepFraction = 0.005F;
    // Runtime-only cancellation for immutable background previews. It is
    // never serialized and has no effect for ordinary foreground calls.
    std::stop_token stopToken{};
};

struct AnimationLoopHorizontalBlendRegions {
    float outgoingVisibleFraction = 2.0F / 3.0F;
    float incomingVisibleFraction = 1.0F / 3.0F;
    // Normalized horizontal screen coordinates in [-1, 1].
    std::array<float, 2> outgoingRange{-1.0F, 1.0F / 3.0F};
    std::array<float, 2> incomingRange{1.0F / 3.0F, 1.0F};
};

struct AnimationLoopScreenDisplacementSample {
    float normalizedPosition = 0.0F;
    // Approximate framing displacement from the preserved path in screen
    // heights. Multiply by a viewport height for an equivalent pixel value.
    float screenDisplacement = 0.0F;
};

struct AnimationLoopKeyMovement {
    std::string keyId;
    float cameraMove = 0.0F;
    float focusMove = 0.0F;
    float cameraCapUsage = 0.0F;
    float focusCapUsage = 0.0F;
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
    std::array<std::vector<AnimationLoopKeyMovement>, 2> keyMovements;
    std::array<std::vector<AnimationLoopScreenDisplacementSample>, 2>
        screenDisplacementSamples;
    std::array<float, 2> maxScreenDisplacement{0.0F, 0.0F};
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
    std::array<std::vector<AnimationLoopKeyMovement>, 2> keyMovements;
    std::array<std::vector<AnimationLoopScreenDisplacementSample>, 2>
        screenDisplacementSamples;
    std::array<float, 2> maxScreenDisplacement{0.0F, 0.0F};
    std::string errorMessage;
};

struct AnimationStrongAlignmentOptions {
    std::string destinationKeyId;
    float referenceNormalizedPosition = 0.0F;
    float aspectRatio = 16.0F / 9.0F;
    float maxMoveFraction = 0.50F;
    // Normalized height measured upward from the bottom of each image. A
    // small margin above one half retains the rock/sand transition when the
    // two cameras are initially vertically misregistered.
    float lowerFrameFraction = 0.55F;
    std::size_t maximumPointSamples = 65'536U;
    // Runtime-only cancellation for resident-point background matching.
    std::stop_token stopToken{};
};

struct AnimationStrongAlignmentMetrics {
    float beforeForegroundReprojectionRms1080 = 0.0F;
    float afterForegroundReprojectionRms1080 = 0.0F;
    float beforeHorizontalOffset1080 = 0.0F;
    float afterHorizontalOffset1080 = 0.0F;
    float beforeVerticalOffset1080 = 0.0F;
    float afterVerticalOffset1080 = 0.0F;
    float beforeScaleMismatchPercent = 0.0F;
    float afterScaleMismatchPercent = 0.0F;
    float beforeRotationMismatchDegrees = 0.0F;
    float afterRotationMismatchDegrees = 0.0F;
    std::size_t foregroundSampleCount = 0U;
    std::size_t destinationOccupiedCellCount = 0U;
    std::size_t referenceOccupiedCellCount = 0U;
    float destinationCoverage = 0.0F;
    float referenceCoverage = 0.0F;
    float cameraMove = 0.0F;
    float focusMove = 0.0F;
    float cameraCapUsage = 0.0F;
    float focusCapUsage = 0.0F;
};

struct AnimationStrongAlignmentResult {
    bool succeeded = false;
    bool changed = false;
    AnimationStrongAlignmentMetrics metrics{};
    std::string errorMessage;
};

struct AnimationMatchingFrameGhostOptions {
    float aspectRatio = 16.0F / 9.0F;
    std::size_t screenGridWidth = 160U;
    std::size_t screenGridHeight = 90U;
    // Retain the nearest surface band in each screen cell. The absolute
    // tolerance keeps thin nearby surfaces intact; the relative tolerance
    // scales for distant survey geometry.
    float frontDepthToleranceMeters = 0.01F;
    float frontDepthToleranceFraction = 0.015F;
    std::size_t maximumPointSamples = 65'536U;
    // Runtime-only cancellation for automatic matching-frame refreshes.
    std::stop_token stopToken{};
};

struct AnimationMatchingFrameGhostResult {
    bool succeeded = false;
    std::size_t inputPointCount = 0U;
    std::size_t sampledPointCount = 0U;
    std::size_t frustumVisiblePointCount = 0U;
    std::vector<invisible_places::io::Float3> positions;
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
// Keeps every timeline frame weight unchanged. Each interior key's existing
// time fraction selects the same fraction of cumulative absolute centre-screen
// X or Y travel on the current evaluated path, then both its camera and focus
// positions are sampled from that source location. Endpoints remain fixed.
// These spatially redistribute the authored controls along their current
// curves; they do not retime them. Direction reversals still pass through zero.
[[nodiscard]] AnimationVelocitySpatialEqualizationResult
RedistributeAnimationPathKeysForConstantScreenXVelocity(
    AnimationPath* path,
    std::uint32_t sampleCount = 1025U);
[[nodiscard]] AnimationVelocitySpatialEqualizationResult
RedistributeAnimationPathKeysForConstantScreenYVelocity(
    AnimationPath* path,
    std::uint32_t sampleCount = 1025U);
// Slides only interior camera controls along the current evaluated camera
// curve. Focus controls, endpoints, and all timeline frame weights remain
// exact. The optimizer reduces rotation magnitude, roughness, and minority-
// direction energy; the optional combined mode also holds centre-screen X
// velocity close to its signed mean. This is a bounded best effort because a
// fixed focus curve and fixed endpoints can make a perfectly flat solution
// impossible.
[[nodiscard]] AnimationCameraSpatialSmoothingResult
OptimizeAnimationCameraKeysForSmoothRotation(
    AnimationPath* path,
    const AnimationCameraSpatialSmoothingOptions& options = {});
// Compatibility entry point for the established PerceivedMotion mode.
[[nodiscard]] std::vector<std::uint32_t> ComputeConstantPerceivedSpeedSegmentFrames(
    const AnimationPath& path,
    std::uint32_t samplesPerSegment = 24U);
// Retimes segment frame counts only. CenterScreenPan follows the focus-plane
// centre; StabilizedPan also gives extra time to off-axis motion, roll, and
// focus-plane flow variation. Total frames and authored key poses are not
// changed by this calculation.
[[nodiscard]] std::vector<std::uint32_t> ComputeEqualizedAnimationSegmentFrames(
    const AnimationPath& path,
    const AnimationSpeedEqualizationOptions& options);

[[nodiscard]] float AnimationPathKeyNormalizedPosition(
    const AnimationPath& path,
    std::size_t keyIndex);

// Jointly adjusts the explicitly selected camera/focus keys of two paths
// (or endpoints for legacy callers). durationFrames and every per-key
// durationFrames value are never written. Applied paths evaluate their
// preserved natural cubic spline plus compact Hermite corrections confined
// to segments touching a selected key; locked-key positions and correction
// derivatives remain exact.
[[nodiscard]] AnimationLoopSmoothingResult SmoothAnimationLoopTransitions(
    AnimationPath* first,
    AnimationPath* second,
    const AnimationLoopSmoothingOptions& options = {});
[[nodiscard]] AnimationLoopHorizontalBlendRegions
ResolveAnimationLoopHorizontalBlendRegions(
    float blendProgress,
    bool panRight);
[[nodiscard]] AnimationLoopTransitionMetrics MeasureAnimationLoopTransitions(
    const AnimationPath& first,
    const AnimationPath& second,
    const AnimationLoopSmoothingOptions& options = {});
[[nodiscard]] AnimationStrongAlignmentResult StrongAlignAnimationKeyToReference(
    AnimationPath* destination,
    const AnimationPath& reference,
    std::span<const invisible_places::io::Float3> residentPoints,
    const AnimationStrongAlignmentOptions& options);
// Captures the front-visible resident surface from `reference`, then applies
// the rigid reference-camera -> destination-camera transform. The returned
// world positions therefore remain frozen while the live camera orbits.
[[nodiscard]] AnimationMatchingFrameGhostResult BuildAnimationMatchingFrameGhost(
    const AnimationPathEvaluation& destination,
    const AnimationPathEvaluation& reference,
    std::span<const invisible_places::io::Float3> residentPoints,
    const AnimationMatchingFrameGhostOptions& options = {});
AnimationPathEvaluation EvaluateAnimationPath(
    const AnimationPath& path,
    float timeSeconds);
[[nodiscard]] AnimationCameraFrameTransform
BuildAnimationCameraFrameTransform(
    const AnimationPathEvaluation& destination,
    const AnimationPathEvaluation& source);
[[nodiscard]] std::array<float, 3> ApplyAnimationCameraFrameTransform(
    const AnimationCameraFrameTransform& transform,
    const std::array<float, 3>& sourcePoint);
[[nodiscard]] std::array<float, 3> InvertAnimationCameraFrameTransform(
    const AnimationCameraFrameTransform& transform,
    const std::array<float, 3>& destinationPoint);

// Structural key edits retain the path's total duration. Insertion splits
// the segment containing `frame`; interior removal merges the two adjacent
// segments; reordering moves poses while retaining the existing timing slots.
// The first and last frames cannot host duplicate keys because every segment
// must contain at least one frame.
[[nodiscard]] std::optional<std::size_t> InsertAnimationPathKeyAtFrame(
    AnimationPath* path,
    AnimationPathKey key,
    std::uint32_t frame,
    std::string* errorMessage = nullptr);
[[nodiscard]] bool RemoveAnimationPathKey(
    AnimationPath* path,
    std::size_t keyIndex,
    std::string* errorMessage = nullptr);
[[nodiscard]] bool ReorderAnimationPathKey(
    AnimationPath* path,
    std::size_t sourceIndex,
    std::size_t destinationIndex);

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
