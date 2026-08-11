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
    // Incoming distance-domain span used only by camera/focus/orientation
    // geometry. Zero is a legacy/automatic value derived from adjacent key
    // poses. Insert-at-playhead materializes and splits these spans so adding
    // an evaluated key does not reshape the existing curve.
    float splineParameterWeight = 0.0F;
    // Materialized d(position)/d(spline parameter) for a path endpoint.
    // Evaluated-key insertion preserves these so splitting the first or last
    // segment cannot alter the existing endpoint direction. Structural pose
    // changes clear them and derive a fresh one-sided chord tangent.
    bool hasSplineEndpointTangent = false;
    std::array<float, 3> splineCameraEndpointTangent{0.0F, 0.0F, 0.0F};
    std::array<float, 3> splineFocusEndpointTangent{0.0F, 0.0F, 0.0F};
    // Quaternion component order is x, y, z, w. Lens order is field of view,
    // near plane, far plane, focus distance, and aperture f-stop.
    std::array<float, 4> splineOrientationEndpointTangent{
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    };
    std::array<float, 5> splineLensEndpointTangent{
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    };
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
    // Optional d(correction)/dt values in world units per second. Legacy
    // records derive their tangent from adjacent corrections. Materialized
    // tangents make a correction independent of later terminal-key appends.
    bool hasCameraCorrectionTangent = false;
    std::array<float, 3> cameraCorrectionTangent{0.0F, 0.0F, 0.0F};
    bool hasFocusCorrectionTangent = false;
    std::array<float, 3> focusCorrectionTangent{0.0F, 0.0F, 0.0F};
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
    std::uint32_t sourceSchemaVersion = 22U;
    std::string name = "Animation";
    std::uint32_t durationFrames = 180;
    // Deprecated schema-22 migration marker. A nonzero value identifies the
    // old frame domain of unscaled timing/water positions written by the
    // initial reciprocal-extension implementation. Extension/migration
    // physically retimes those positions and clears this back to zero.
    std::uint32_t authoredTrackDurationFrames = 0U;
    // Zero means a legacy/unset preference. Newly authored animations
    // capture the live application window and request it again when loaded.
    std::uint32_t defaultLiveViewWindowWidth = 0U;
    std::uint32_t defaultLiveViewWindowHeight = 0U;
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
    // Authored key times remain the public timeline knots. Camera, focus,
    // lens, and orientation geometry is solved over independent cumulative
    // motion-distance knots so retiming cannot bend the spatial curve.
    std::vector<float> knots;
    std::vector<float> geometryKnots;
    // Positive d(geometry)/dt values for the monotone quintic time map.
    // Every key has zero time-map acceleration, keeping the composed path C2
    // while carrying non-zero velocity through non-degenerate motion.
    std::vector<float> geometryVelocities;
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
    AnimationPreparedScalarSpline orientationX;
    AnimationPreparedScalarSpline orientationY;
    AnimationPreparedScalarSpline orientationZ;
    AnimationPreparedScalarSpline orientationW;
    // Localized confirmed corrections. The authored path's preserved spline
    // stays the base; compact quintic-Hermite offsets affect only segments
    // touching a user-enabled key. Their shared acceleration is zero at each
    // key, so the correction layer is C2 as well as velocity-continuous.
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

// A stable, ordered world-space triangle observed while authoring a pan
// extension. Point zero is the anchor; points one and two preserve user node
// identity and winding while constraining projected translation, rotation,
// scale, shear, and perspective.
struct AnimationSurfacePatchObservation {
    std::uint32_t pointCount = 0U;
    std::array<std::array<float, 3>, 3> worldPoints{};
};

// The source motion from frame zero through sourceTailFrame is appended to
// the other animation. The two ordered three-point observations describe
// corresponding geometry at the source seam and destination terminal seam.
struct AnimationTerminalExtensionSpec {
    // The source path always begins at frame zero, which is the existing 50%
    // seam against the destination's current terminal frame. This later
    // source frame defines the length and motion of the appended tail.
    std::uint32_t sourceTailFrame = 0U;
    AnimationSurfacePatchObservation sourcePatch{};
    AnimationSurfacePatchObservation destinationEndPatch{};
};

struct AnimationReciprocalPanExtensionOptions {
    // firstDrivesSecond appends the observed first-path motion to second.
    // secondDrivesFirst performs the reciprocal append to first.
    AnimationTerminalExtensionSpec firstDrivesSecond{};
    AnimationTerminalExtensionSpec secondDrivesFirst{};
    float aspectRatio = 16.0F / 9.0F;
    std::uint32_t sampleCount = 17U;
    std::uint32_t optimizationSweeps = 24U;
};

struct AnimationReciprocalPanExtensionMetrics {
    // Array order is first candidate, then second candidate.
    std::array<std::uint32_t, 2> extensionFrames{};
    std::array<std::uint32_t, 2> appendedKeyCount{};
    std::array<std::size_t, 2> sourceInteriorKeyCount{};
    std::array<std::size_t, 2> sourceInflectionCount{};
    std::array<bool, 2> rotationConstrained{};
    std::array<float, 2> beforeVelocityRmsScreenHeightsPerSecond{};
    std::array<float, 2> afterVelocityRmsScreenHeightsPerSecond{};
    std::array<float, 2> beforeRotationRmsDegreesPerSecond{};
    std::array<float, 2> afterRotationRmsDegreesPerSecond{};
    std::array<float, 2> anchorOverlayRmsScreenHeights{};
    std::array<float, 2> anchorOverlayMaxScreenHeights{};
    std::array<float, 2> patchNodeOverlayRmsScreenHeights{};
    std::array<float, 2> patchNodeOverlayMaxScreenHeights{};
    std::array<std::array<float, 2>, 2>
        signedVelocityResidualScreenHeightsPerSecond{};
    std::array<float, 2> rotationRateResidualDegreesPerSecond{};
    std::array<float, 2> perspectiveScaleResidualPercent{};
    std::array<float, 2> patchConfidence{};
    std::array<std::string, 2> patchDiagnostic{};
    std::array<float, 2> maxPrefixPositionError{};
    std::array<float, 2> formerTerminalCameraMove{};
    std::array<float, 2> formerTerminalFocusMove{};
};

struct AnimationReciprocalPanExtensionResult {
    bool succeeded = false;
    bool changed = false;
    AnimationPath firstCandidate{};
    AnimationPath secondCandidate{};
    AnimationReciprocalPanExtensionMetrics metrics{};
    std::string errorMessage;
};

// One-side preview used by the guided reciprocal workflow after each seam is
// captured. It runs the same terminal solver as the final atomic reciprocal
// build, but returns only the destination candidate and never mutates either
// input path.
struct AnimationPanTerminalExtensionResult {
    bool succeeded = false;
    bool changed = false;
    AnimationPath candidate{};
    std::uint32_t extensionFrames = 0U;
    std::uint32_t appendedKeyCount = 0U;
    float anchorOverlayRmsScreenHeights = 0.0F;
    float patchNodeOverlayRmsScreenHeights = 0.0F;
    float velocityResidualScreenHeightsPerSecond = 0.0F;
    float rotationRateResidualDegreesPerSecond = 0.0F;
    std::string errorMessage;
};

// Conservative, pair-wide clip normalization used before reciprocal pan
// fitting. Candidate copies retain every authored pose, timing, lens value,
// and metadata field except near/far clipping. The smallest valid near plane
// and largest valid far plane preserve the union of both paths' visibility.
struct AnimationClipPlaneNormalizationResult {
    bool succeeded = false;
    bool changed = false;
    float nearPlane = 0.0F;
    float farPlane = 0.0F;
    AnimationPath firstCandidate{};
    AnimationPath secondCandidate{};
    std::vector<std::string> linkedCameraIds;
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

struct AnimationFocusRelativeCameraAlignmentOptions {
    std::string destinationKeyId;
    float referenceNormalizedPosition = 0.0F;
};

struct AnimationFocusRelativeCameraAlignmentResult {
    bool succeeded = false;
    bool changed = false;
    float referenceFocusDistance = 0.0F;
    float referenceAlongPathOffset = 0.0F;
    float referenceLateralOffset = 0.0F;
    float referenceHeightOffset = 0.0F;
    float cameraMove = 0.0F;
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

// Builds both terminal extensions from immutable A/B snapshots. The returned
// candidates are atomic: failure returns neither partially extended path.
// The current implementation accepts target-driven, fixed-lens paths only.
[[nodiscard]] AnimationReciprocalPanExtensionResult
BuildAnimationReciprocalPanExtension(
    const AnimationPath& first,
    const AnimationPath& second,
    const AnimationReciprocalPanExtensionOptions& options = {});

[[nodiscard]] AnimationPanTerminalExtensionResult
BuildAnimationPanTerminalExtensionPreview(
    const AnimationPath& destination,
    const AnimationPath& source,
    const AnimationTerminalExtensionSpec& specification,
    float aspectRatio = 16.0F / 9.0F,
    std::uint32_t sampleCount = 17U,
    std::uint32_t optimizationSweeps = 24U);

[[nodiscard]] AnimationClipPlaneNormalizationResult
BuildConservativeAnimationClipPlaneNormalization(
    const AnimationPath& first,
    const AnimationPath& second);

// Copies an ordered source anchor/front/side triangle around a newly picked
// destination anchor. The two offsets retain their physical lengths and
// camera-local directions under the source-camera -> destination-camera
// rotation. A zero-point result indicates invalid input.
[[nodiscard]] AnimationSurfacePatchObservation
BuildAnimationCameraLocalSurfacePatch(
    const AnimationPathEvaluation& source,
    const AnimationPathEvaluation& destination,
    const AnimationSurfacePatchObservation& sourcePatch,
    const std::array<float, 3>& destinationAnchor);

// Jointly adjusts the explicitly selected camera/focus keys of two paths
// (or endpoints for legacy callers). durationFrames and every per-key
// durationFrames value are never written. Applied paths evaluate their
// preserved distance-parameterized C2 spline plus compact quintic-Hermite
// corrections confined to segments touching a selected key; locked-key
// positions and correction derivatives remain exact.
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
// Moves only one destination camera key. The destination focus stays fixed;
// the reference frame's focus-to-camera vector is decomposed into horizontal
// along-path, horizontal lateral, and world-Z components, then reconstructed
// in the destination path's local travel frame. This preserves the matching
// rig's distance, height, and forward/perpendicular/backward relationship
// without moving or modifying the reference animation.
[[nodiscard]] AnimationFocusRelativeCameraAlignmentResult
AlignAnimationKeyCameraToReferenceRig(
    AnimationPath* destination,
    const AnimationPath& reference,
    const AnimationFocusRelativeCameraAlignmentOptions& options);
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

// Makes the currently visible adjusted key poses the clean C2 spline baseline
// and removes the localized correction overlay. Key poses and timing are not
// changed. Structural/manual edits call this automatically so old velocity-
// alignment weighting cannot survive a topology or control-point change.
bool BakeAnimationPathLocalizedCorrections(AnimationPath* path);
// Drops materialized distance-domain spans and endpoint derivatives so
// evaluation derives a fresh, chord-aware parameterization and forward
// one-sided endpoint tangents from the current key poses.
void RebuildAnimationPathGeometryFromKeys(AnimationPath* path);

// Removes authored/equalized timing bias by sharing the path's effective
// total frame count as evenly as integer frames allow across every segment.
// Camera/focus poses and total duration remain unchanged. Geometry is
// distance-parameterized independently of these frame weights, so this only
// resets traversal timing; localized corrections are baked first. Returns
// true when either correction metadata or an incoming frame count changed.
[[nodiscard]] bool ResetAnimationPathTimingWeights(AnimationPath* path);

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

// Uses a finite positive surface hit when one exists; otherwise preserves the
// caller-provided focus distance. Invalid hit/fallback inputs resolve to a
// conservative one-metre distance so focus placement always remains usable.
[[nodiscard]] float ResolveSurfaceFocusDistance(
    std::optional<float> surfaceHitDistance,
    float fallbackFocusDistance);

void MoveAnimationCameraKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& cameraPosition);

void MoveAnimationFocusKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& focusPoint);

}  // namespace invisible_places::camera
