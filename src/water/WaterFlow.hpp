#pragma once

#include "io/MeshData.hpp"
#include "io/PointCloudData.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace invisible_places::water {

enum class WaterScaleMode {
    Aerial,
    Mid,
    Detail
};

enum class WaterEmitterOrigin {
    Manual,
    AutoSuggested,
    Propagated
};

enum class WaterEmitterStatus {
    Candidate,
    Accepted,
    Disabled
};

enum class WaterSourceSettingsAssignment {
    Default,
    Custom,
    LinkedEmitter
};

enum class WaterEffectFeatureType {
    Ripple,
    FieldSurfaceMotion,
    FieldNoFlowRegion,
    FieldBridgeAllowedRegion,
    FieldBridgeBlockedRegion
};

enum class WaterRippleOverlayType {
    CausticLace,
    LinearRipples,
    RadialRipples,
    RainRings,
    TideBands,
    WetSheen,
    CurrentThreads,
    DropletGlints,
    DripTrails,
    FoamSparkle,
    SaltMineralShimmer
};

constexpr std::size_t kWaterRippleOverlayTypeCount = 11U;

enum class WaterRipplePatternControl {
    PatternScale,
    WavelengthMeters,
    Speed,
    Warp,
    Turbulence,
    Density,
    Direction
};

struct WaterRipplePatternControlSpec {
    WaterRipplePatternControl control = WaterRipplePatternControl::PatternScale;
    std::string_view label;
    std::string_view tooltip;
    float minimum = 0.0F;
    float maximum = 1.0F;
    bool logarithmic = false;
};

struct WaterRipplePatternSettings {
    float patternScale = 0.0F;
    float wavelengthMeters = 0.0F;
    float speed = 0.0F;
    float warp = 0.0F;
    float turbulence = 0.0F;
    float density = 0.0F;
    float phase = 0.0F;
    float directionX = 0.0F;
    float directionY = 0.0F;
    float directionZ = 0.0F;
};

struct WaterEffectLayer;

[[nodiscard]] std::array<WaterRippleOverlayType, 11> AllWaterRippleOverlayTypes();
[[nodiscard]] std::string_view WaterRippleOverlayTypeDescription(WaterRippleOverlayType type);
[[nodiscard]] std::string_view WaterRippleOverlayTypeNameForStorage(WaterRippleOverlayType type);
[[nodiscard]] std::optional<WaterRippleOverlayType> ParseWaterRippleOverlayTypeName(std::string_view value);
[[nodiscard]] std::size_t WaterRippleOverlayTypeIndex(WaterRippleOverlayType type);
[[nodiscard]] WaterRipplePatternSettings DefaultWaterRipplePatternSettings(WaterRippleOverlayType type);
[[nodiscard]] std::span<const WaterRipplePatternControlSpec> WaterRipplePatternControlSpecs(
    WaterRippleOverlayType type);
[[nodiscard]] WaterRipplePatternSettings ActiveWaterRipplePatternSettings(const WaterEffectLayer& layer);
void StoreActiveWaterRipplePatternSettings(WaterEffectLayer* layer);
void ApplyWaterRipplePatternSettings(WaterEffectLayer* layer, const WaterRipplePatternSettings& settings);
void ApplyActiveWaterRipplePatternSettings(WaterEffectLayer* layer);
void InitializeWaterRipplePatternSettings(WaterEffectLayer* layer);

enum class WaterEffectBlendMode {
    Add,
    Max,
    Multiply,
    Screen,
    Override
};

enum class WaterSeepageQuality {
    Auto,
    Low,
    Balanced,
    High
};

enum class WaterFieldOutputMode {
    Trails,
    SurfaceMotion,
    Both
};

struct WaterPathGenerationSettings {
    WaterScaleMode legacyScaleMode = WaterScaleMode::Mid;
    bool autoTune = true;
    float supportVoxelSize = 0.008F;
    float maxBridgeDistance = 0.080F;
    float smoothing = 0.45F;
    float pathLength = 14.0F;
    float pathSampleSpacing = 0.008F;
    float branching = 0.70F;
    float coverage = 0.65F;
    float gapTolerance = 0.60F;
    bool attractorEnabled = false;
    invisible_places::io::Float3 attractorPosition{};
    float attractorStrength = 0.0F;
    std::uint32_t maxSteps = 2200;
    std::uint32_t supportSampleLimit = 450000;
};

struct WaterParticleTrailShapeSettings {
    float particleJitter = 0.35F;
    float splineAnchorSpacing = 0.5F;
    std::uint32_t trailLaneCount = 7;
    float trailLooseness = 0.45F;
    float trailSmoothness = 0.55F;
    float trailTurbulence = 0.45F;
    float trailMomentum = 0.60F;
    float normalTurbulenceResponse = 0.60F;
};

struct WaterAnimationTrailSettings {
    float particleDensity = 1.0F;
    float particleSpeed = 1.0F;
    float colorVariation = 0.65F;
    float trailLengthMeters = 0.75F;
    float trailSampleSpacingMeters = 0.0F;
};

struct WaterCausticLookSettings {
    bool enabled = false;
    float intensity = 0.75F;
    float scale = 4.0F;
    float speed = 0.55F;
    float lineSharpness = 0.72F;
    float warp = 0.35F;
    float cellSizeMeters = 0.20F;
    float lineWidthMeters = 0.015F;
    float featherMeters = 0.006F;
    float surfacePointSpacingMeters = 0.005F;
    float warpAmplitudeMeters = 0.045F;
    float tintRed = 0.62F;
    float tintGreen = 0.88F;
    float tintBlue = 1.0F;
    float emissionBoost = 1.15F;
    float opacityBoost = 0.08F;
    float pointSizeBoost = 0.0F;
};

struct WaterEffectResponseSettings {
    float intensity = 0.75F;
    float emissionAdd = 0.85F;
    float opacityAdd = 0.0F;
    float opacityMultiply = 1.0F;
    float pointSizeAdd = 0.0F;
    float pointSizeMultiply = 1.0F;
    float hueShift = 0.0F;
    float colouriseRed = 0.62F;
    float colouriseGreen = 0.88F;
    float colouriseBlue = 1.0F;
    float colouriseAmount = 0.35F;
    float gaussianSharpnessBias = 0.0F;
};

struct WaterSeepageLookSettings {
    WaterSeepageQuality quality = WaterSeepageQuality::Auto;
    float baseWetness = 0.35F;
    float density = 0.45F;
    float glisten = 0.55F;
    float wavelengthMeters = 0.16F;
    float patternScale = 1.0F;
    float speed = 0.18F;
    float warp = 0.40F;
    float turbulence = 0.22F;
    float phase = 0.0F;
    float rainResponse = 0.50F;
    WaterEffectResponseSettings response{
        .intensity = 0.85F,
        .emissionAdd = 0.35F,
        .opacityAdd = 0.04F,
        .opacityMultiply = 1.12F,
        .pointSizeAdd = 0.0F,
        .pointSizeMultiply = 1.08F,
        .hueShift = 0.0F,
        .colouriseRed = 0.28F,
        .colouriseGreen = 0.42F,
        .colouriseBlue = 0.46F,
        .colouriseAmount = 0.22F,
        .gaussianSharpnessBias = 0.0F,
    };
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Max;
};

struct WaterSeepageLookProfile {
    std::string name = "Default";
    WaterSeepageLookSettings settings{};
};

struct WaterSeepageNode {
    std::uint32_t id = 0U;
    std::string name = "Seepage";
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 surfaceNormal{0.0F, 1.0F, 0.0F};
    invisible_places::io::Float3 downAxis{0.0F, 0.0F, -1.0F};
    float reachMeters = 1.25F;
    float startWidthMeters = 0.12F;
    float endWidthMeters = 0.75F;
    float edgeFeatherMeters = 0.10F;
    float depthToleranceMeters = 0.15F;
    float normalAlignment = 0.20F;
    float strength = 1.0F;
    std::uint32_t seed = 1U;
    bool enabledInViewport = true;
    bool enabledInExport = true;
    std::vector<std::string> targetSceneRoles{"ROCK", "VEG"};
    std::string lookProfileName = "Default";
    std::optional<WaterSeepageLookSettings> lookOverride;
    std::optional<WaterSeepageLookSettings> tempLookOverride;
};

inline constexpr std::size_t kWaterSeepageMaximumGuideSamples = 8U;

struct WaterSeepageGuideSample {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 1.0F, 0.0F};
    float station = 0.0F;
    float confidence = 1.0F;
};

struct WaterSeepageSurfaceGuide {
    std::uint32_t nodeId = 0U;
    std::uint32_t sampleCount = 0U;
    std::array<WaterSeepageGuideSample, kWaterSeepageMaximumGuideSamples> samples{};
    float requestedReachMeters = 0.0F;
    float achievedReachMeters = 0.0F;
    bool valid = false;
    bool complete = false;
};

struct WaterSeepageRuntimeNode {
    std::uint32_t id = 0U;
    std::uint32_t seed = 1U;
    glm::vec3 position{0.0F, 0.0F, 0.0F};
    glm::vec3 surfaceNormal{0.0F, 1.0F, 0.0F};
    glm::vec3 downAxis{0.0F, 0.0F, -1.0F};
    glm::vec3 lateralAxis{1.0F, 0.0F, 0.0F};
    float reachMeters = 1.25F;
    float startHalfWidthMeters = 0.06F;
    float endHalfWidthMeters = 0.375F;
    float edgeFeatherMeters = 0.10F;
    float depthToleranceMeters = 0.15F;
    float normalAlignment = 0.20F;
    float strength = 1.0F;
    float rainVisualStrength = 0.0F;
    WaterSeepageQuality resolvedQuality = WaterSeepageQuality::Balanced;
    WaterSeepageLookSettings look{};
    std::uint32_t guideSampleCount = 0U;
    std::array<WaterSeepageGuideSample, kWaterSeepageMaximumGuideSamples> guideSamples{};
    float guideRequestedReachMeters = 0.0F;
    float guideAchievedReachMeters = 0.0F;
    bool guideValid = false;
    bool guideComplete = false;
};

struct WaterSeepageSpatialHashCell {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;
    std::uint32_t referenceOffset = 0U;
    std::uint32_t referenceCount = 0U;
    bool occupied = false;
};

struct WaterSeepageSpatialGridDiagnostics {
    std::uint32_t inputNodeCount = 0U;
    std::uint32_t activeNodeCount = 0U;
    std::uint32_t occupiedCellCount = 0U;
    std::uint32_t hashCellCapacity = 0U;
    std::uint32_t nodeReferenceCount = 0U;
    std::uint32_t overflowCellCount = 0U;
    std::uint32_t droppedNodeReferenceCount = 0U;
    std::uint32_t maxReferencesPerCell = 8U;
};

struct WaterSeepageSpatialGrid {
    static constexpr std::uint32_t kMaxReferencesPerCell = 8U;

    std::vector<WaterSeepageRuntimeNode> nodes;
    std::vector<WaterSeepageSpatialHashCell> hashCells;
    std::vector<std::uint32_t> nodeReferences;
    invisible_places::io::Bounds3f unionBounds{};
    float cellSizeMeters = 0.50F;
    WaterSeepageSpatialGridDiagnostics diagnostics{};
};

struct WaterSeepageRuntimeContribution {
    float mask = 0.0F;
    float damp = 0.0F;
    float ripple = 0.0F;
    float glint = 0.0F;
    float scale = 0.0F;
    float colourMix = 0.0F;
    float emissionAdd = 0.0F;
    float opacityAdd = 0.0F;
    float opacityMultiply = 1.0F;
    float pointSizeAdd = 0.0F;
    float pointSizeMultiply = 1.0F;
    glm::vec3 colour{0.28F, 0.42F, 0.46F};
};

struct WaterEffectLayer {
    std::uint32_t id = 0;
    std::string name = "Ripple";
    WaterEffectFeatureType featureType = WaterEffectFeatureType::Ripple;
    WaterRippleOverlayType rippleOverlayType = WaterRippleOverlayType::CausticLace;
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Add;
    std::filesystem::path targetLayerSourcePath;
    std::vector<std::string> targetSceneRoles;
    std::vector<invisible_places::io::Float3> vertices;
    std::vector<invisible_places::io::Float3> hull;
    bool enabledInViewport = true;
    bool enabledInExport = true;
    std::uint32_t blendPriority = 0;
    float edgeBlendWidth = 0.60F;
    float regionStrength = 1.0F;
    float patternScale = 1.0F;
    float speed = 0.55F;
    float wavelengthMeters = 0.25F;
    float warp = 0.35F;
    float turbulence = 0.06F;
    float phase = 0.0F;
    float directionX = 1.0F;
    float directionY = 0.0F;
    float directionZ = 0.0F;
    float density = 0.55F;
    std::array<WaterRipplePatternSettings, kWaterRippleOverlayTypeCount> overlayPatternSettings{};
    std::uint32_t seed = 1;
    std::uint32_t maxAffectedPoints = 250000;
    WaterEffectResponseSettings response{};
};

struct WaterRegionSelection;

struct WaterRippleRuntimeMembership {
    std::uint32_t pointIndex = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t paramIndex = 0;
    float edgeDistance = 0.0F;
    float seed = 0.0F;
    float shoreDistance = 0.0F;
};

struct WaterRippleRuntimeParams {
    WaterRippleOverlayType overlayType = WaterRippleOverlayType::CausticLace;
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Add;
    std::uint32_t layerId = 0;
    std::uint32_t seed = 1;
    glm::vec3 regionCenter{0.0F, 0.0F, 0.0F};
    glm::vec3 direction{1.0F, 0.0F, 0.0F};
    float regionStrength = 1.0F;
    float edgeBlendWidth = 0.60F;
    float patternScale = 1.0F;
    float wavelengthMeters = 0.25F;
    float speed = 0.55F;
    float warp = 0.35F;
    float turbulence = 0.06F;
    float density = 0.55F;
    float phase = 0.0F;
    WaterEffectResponseSettings response{};
};

struct WaterRippleRuntimeContribution {
    float scale = 0.0F;
    float colourMix = 0.0F;
    float emissionAdd = 0.0F;
    float opacityAdd = 0.0F;
    float opacityMultiply = 1.0F;
    float pointSizeAdd = 0.0F;
    float pointSizeMultiply = 1.0F;
    glm::vec3 colour{0.62F, 0.88F, 1.0F};
};

[[nodiscard]] WaterRippleRuntimeParams BuildWaterRippleRuntimeParams(
    const WaterEffectLayer& layer,
    const WaterRegionSelection& selection);
[[nodiscard]] WaterRippleRuntimeParams BuildWaterRippleRuntimeParams(const WaterEffectLayer& layer);

[[nodiscard]] std::vector<WaterRippleRuntimeMembership> BuildWaterRippleRuntimeMemberships(
    const WaterRegionSelection& selection,
    std::uint32_t paramIndex);

[[nodiscard]] WaterRippleRuntimeContribution EvaluateWaterRippleRuntimeContribution(
    const WaterRippleRuntimeParams& params,
    const WaterRippleRuntimeMembership& membership,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds);

struct WaterFlowTrailSettings {
    bool enabled = true;
    std::uint32_t trailCountTotal = 700;
    std::uint32_t laneCount = 0;
    float trailLengthMeters = 0.75F;
    float trailPointSpacingMeters = 0.010F;
    float trailWidthMeters = 0.006F;
    float trailStreakLengthMeters = 0.045F;
    float surfaceOffsetMeters = 0.004F;
    float pathAttraction = 0.85F;
    float laneSpreadMeters = 0.12F;
    float laneCrossing = 0.22F;
    float trailSmoothness = 0.85F;
    float trailLooseness = 0.08F;
    float turbulence = 0.06F;
    float speedMetersPerSecond = 0.45F;
    std::uint32_t seed = 1;
};

[[nodiscard]] bool WaterFlowLaneRouteInputsEqual(
    const WaterFlowTrailSettings& left,
    const WaterFlowTrailSettings& right);
[[nodiscard]] bool WaterFlowLaneSpeedOnlyEdit(
    const WaterFlowTrailSettings& before,
    const WaterFlowTrailSettings& after);

struct WaterTrailGeometrySettings {
    float trailLengthMeters = 0.75F;
    float pointSpacingMeters = 0.010F;
    float widthMeters = 0.006F;
    float streakLengthMeters = 0.045F;
};

[[nodiscard]] WaterTrailGeometrySettings DefaultWaterTrailGeometrySettings();
[[nodiscard]] float AutoWaterTrailPointSpacingMeters(float trailLengthMeters, float widthMeters);
[[nodiscard]] float AutoWaterTrailStreakLengthMeters(float pointSpacingMeters, float widthMeters);
[[nodiscard]] WaterTrailGeometrySettings FitWaterTrailGeometryForContinuousLines(
    WaterTrailGeometrySettings geometry);
[[nodiscard]] WaterTrailGeometrySettings WaterTrailGeometryFromFlowTrailSettings(
    const WaterFlowTrailSettings& settings);
[[nodiscard]] WaterFlowTrailSettings ApplyWaterTrailGeometryToFlowTrailSettings(
    WaterFlowTrailSettings settings,
    const WaterTrailGeometrySettings& geometry);
[[nodiscard]] bool WaterTrailGeometryGenerationInputsEqual(
    const WaterTrailGeometrySettings& left,
    const WaterTrailGeometrySettings& right);
[[nodiscard]] bool WaterTrailGeometryLiveVisualOnlyEdit(
    const WaterTrailGeometrySettings& before,
    const WaterTrailGeometrySettings& after);

struct WaterFieldSettings {
    bool enabled = true;
    WaterFieldOutputMode outputMode = WaterFieldOutputMode::Both;
    float corridorRadiusMeters = 0.35F;
    float fieldResolutionMeters = 0.012F;
    float projectionResolutionMeters = 0.005F;
    float guideWeight = 0.60F;
    float downhillWeight = 0.45F;
    float graphWeight = 0.80F;
    float lateralWeight = 0.10F;
    float fieldSmoothing = 0.35F;
    float wetnessSpread = 0.55F;
    float surfaceOffsetMeters = 0.004F;
    float surfaceConfidenceThreshold = 0.25F;
    float maxBridgeDistanceMeters = 0.08F;
    float bridgeAggression = 0.45F;
    float turbulence = 0.08F;
    std::uint32_t seed = 7;
};

struct WaterFieldTrailSettings {
    bool enabled = true;
    std::uint32_t trailCount = 850;
    float seedSpacingMeters = 0.025F;
    float trailLengthMeters = 0.85F;
    float trailPointSpacingMeters = 0.012F;
    float trailWidthMeters = 0.005F;
    float trailStreakLengthMeters = 0.045F;
    float momentum = 0.84F;
    float maxTurnAngleDegrees = 12.0F;
    float speedMetersPerSecond = 0.38F;
    bool fadeOnLowConfidence = true;
};

enum class WaterRainIntensityPreset {
    LightMist,
    Rain,
    HeavyDownpour
};

struct WaterRainSettings {
    bool enabled = false;
    WaterRainIntensityPreset intensityPreset = WaterRainIntensityPreset::Rain;
    std::uint32_t dropCount = 900;
    float fallSpeedMetersPerSecond = 8.0F;
    float spawnHeightMeters = 4.0F;
    float spawnRadiusMeters = 6.0F;
    float spawnOutOfFrameMargin = 0.18F;
    float surfaceSearchRadiusMeters = 0.20F;
    float downhillSearchRadiusMeters = 1.20F;
    float killBelowSceneMeters = 3.0F;
    float cameraDeathDistanceMeters = 28.0F;
    float surfaceRunSpeedMetersPerSecond = 1.05F;
    float sandRunDistanceMeters = 0.65F;
    float windDirectionX = 1.0F;
    float windDirectionY = 0.0F;
    float windStrengthMeters = 0.30F;
    float windNoise = 0.45F;
    float windResponse = 0.50F;
    float trailLengthMeters = 0.75F;
    float trailPointSpacingMeters = 0.18F;
    float trailWidthMeters = 0.004F;
    float trailStreakLengthMeters = 0.12F;
    std::uint32_t routeAnchorCount = 10;
    std::uint32_t supportSampleLimit = 250000;
    std::uint32_t seed = 101U;
};

struct WaterRainCameraFrame {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 target{};
    float fovDegrees = 55.0F;
    float aspectRatio = 1.0F;
};

struct WaterRainDiagnostics {
    std::uint32_t requestedDropCount = 0;
    std::uint32_t emittedDropCount = 0;
    std::uint32_t emittedSampleCount = 0;
    std::uint32_t routeAnchorCount = 0;
    std::uint32_t firstSupportHitCount = 0;
    std::uint32_t sandTerminationCount = 0;
    std::uint32_t fallbackTerminationCount = 0;
    std::uint32_t noSupportKillCount = 0;
};

inline constexpr float kWaterTrailFeatureTypeRain = 4.0F;
inline constexpr float kWaterTrailFeatureTypeDynamicMesh = 5.0F;

struct WaterDynamicMeshMotionKeyframe {
    float timeSeconds = 0.0F;
    invisible_places::io::Float3 position{};
};

struct WaterDynamicMeshAttractor {
    std::uint32_t id = 1;
    std::string name = "Attractor";
    invisible_places::io::Float3 position{};
    float radiusMeters = 0.65F;
    float strength = 0.75F;
    bool enabled = true;
    std::vector<WaterDynamicMeshMotionKeyframe> keyframes;
};

struct WaterDynamicMeshEmitterMotion {
    std::uint32_t emitterId = 0;
    std::string name = "Emitter Motion";
    bool enabled = true;
    std::vector<WaterDynamicMeshMotionKeyframe> keyframes;
};

struct WaterDynamicMeshParticlePreset {
    std::string_view name;
    std::string_view label;
};

struct WaterDynamicMeshFlowSettings {
    bool enabled = false;
    bool gpuPreviewEnabled = true;
    std::filesystem::path meshPath{};
    float cacheCellSizeMeters = 0.08F;
    float projectionSearchRadiusMeters = 1.25F;
    float ambiguityHeightMeters = 0.18F;
    std::uint32_t previewParticleLimit = 560;
    std::uint32_t finalParticleLimit = 2400;
    float trailLengthMeters = 18.0F;
    float stepMeters = 0.12F;
    float trailWidthMeters = 0.005F;
    float trailStreakLengthMeters = 0.18F;
    float surfaceOffsetMeters = 0.020F;
    float speedMetersPerSecond = 0.62F;
    float downhillWeight = 1.35F;
    float attractorWeight = 1.0F;
    float sourceVelocityWeight = 0.35F;
    float curlStrength = 0.18F;
    float branchingStrength = 0.36F;
    float eddyStrength = 0.08F;
    float topologyResponse = 0.65F;
    float inertia = 0.64F;
    float animationDurationSeconds = 4.0F;
    std::uint32_t seed = 29U;
    std::string particlePresetName = "Default";
    std::string trailProfileName = "Default";
    std::vector<WaterDynamicMeshAttractor> attractors;
    std::vector<WaterDynamicMeshEmitterMotion> emitterMotions;
};

struct MeshSurfaceCacheCell {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 downhill{1.0F, 0.0F, 0.0F};
    int cellX = 0;
    int cellY = 0;
    float minZ = 0.0F;
    float maxZ = 0.0F;
    float confidence = 1.0F;
    std::uint32_t sampleCount = 0;
    bool ambiguous = false;
};

struct MeshSurfaceCache {
    std::uint32_t schemaVersion = 1;
    std::filesystem::path meshPath;
    std::string meshSignature;
    WaterDynamicMeshFlowSettings settings{};
    std::vector<MeshSurfaceCacheCell> cells;
    std::unordered_map<std::uint64_t, std::uint32_t> cellLookup;
    invisible_places::io::Bounds3f bounds{};
    std::uint64_t sourceVertexCount = 0;
    std::uint64_t sourceTriangleCount = 0;
    double buildMilliseconds = 0.0;
    bool stale = false;
};

struct MeshSurfaceProjection {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 downhill{1.0F, 0.0F, 0.0F};
    float confidence = 0.0F;
    bool ambiguous = false;
    bool hit = false;
};

struct WaterDynamicMeshFlowDiagnostics {
    double meshLoadMilliseconds = 0.0;
    double cacheBuildMilliseconds = 0.0;
    double solveMilliseconds = 0.0;
    double gpuStaticUploadMilliseconds = 0.0;
    double gpuLiveUploadMilliseconds = 0.0;
    double gpuDispatchMilliseconds = 0.0;
    std::uint64_t sourceVertexCount = 0;
    std::uint64_t sourceTriangleCount = 0;
    std::uint32_t cacheCellCount = 0;
    std::uint32_t emittedPathCount = 0;
    std::uint32_t emittedSampleCount = 0;
    std::uint32_t projectionMissCount = 0;
    std::uint32_t ambiguousHitCount = 0;
    bool gpuStaticBuffersReused = false;
    bool gpuAsynchronousDispatch = false;
};

[[nodiscard]] std::array<WaterRainIntensityPreset, 3> AllWaterRainIntensityPresets();
[[nodiscard]] std::string_view WaterRainIntensityPresetLabel(WaterRainIntensityPreset preset);
[[nodiscard]] std::string_view WaterRainIntensityPresetNameForStorage(WaterRainIntensityPreset preset);
[[nodiscard]] std::optional<WaterRainIntensityPreset> ParseWaterRainIntensityPresetName(std::string_view value);
[[nodiscard]] WaterRainSettings DefaultWaterRainSettings();
[[nodiscard]] WaterDynamicMeshFlowSettings DefaultWaterDynamicMeshFlowSettings();
[[nodiscard]] std::array<WaterDynamicMeshParticlePreset, 4> AllWaterDynamicMeshParticlePresets();
[[nodiscard]] WaterDynamicMeshFlowSettings ApplyWaterDynamicMeshParticlePreset(
    WaterDynamicMeshFlowSettings settings,
    std::string_view presetName);
[[nodiscard]] std::string_view NormalizeWaterDynamicMeshParticlePresetName(std::string_view presetName);
[[nodiscard]] WaterRainSettings ApplyWaterRainIntensityPreset(
    WaterRainSettings settings,
    WaterRainIntensityPreset preset);
[[nodiscard]] float WaterRainEffectiveWindResponse(const WaterRainSettings& settings);
[[nodiscard]] float WaterRainPseudoWindDisplacementMeters(const WaterRainSettings& settings);
[[nodiscard]] float WaterRainFallAngleDegrees(const WaterRainSettings& settings);
[[nodiscard]] float WaterRainPresetVisualStrength(WaterRainIntensityPreset preset);

[[nodiscard]] WaterSeepageLookSettings DefaultWaterSeepageLookSettings();
[[nodiscard]] WaterSeepageQuality ResolveWaterSeepageQuality(
    WaterSeepageQuality quality,
    std::uint64_t effectivePointInvocations);
[[nodiscard]] WaterSeepageLookSettings ResolveWaterSeepageLook(
    std::span<const WaterSeepageLookProfile> profiles,
    const WaterSeepageLookSettings& defaultLook,
    std::string_view profileName,
    const std::optional<WaterSeepageLookSettings>& lookOverride,
    const std::optional<WaterSeepageLookSettings>& tempLookOverride);
[[nodiscard]] WaterSeepageLookSettings ResolveWaterSeepageLook(
    const WaterSeepageNode& node,
    std::span<const WaterSeepageLookProfile> profiles,
    const WaterSeepageLookSettings& defaultLook);
[[nodiscard]] std::string WaterSeepageLocalLookName(std::string_view baseName, std::uint32_t nodeId);
[[nodiscard]] invisible_places::io::Float3 DeriveWaterSeepageDownAxis(
    const invisible_places::io::Float3& surfaceNormal,
    const invisible_places::io::Float3& fallbackAxis = {0.0F, 0.0F, -1.0F});
[[nodiscard]] WaterSeepageSpatialGrid BuildWaterSeepageSpatialGrid(
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSeepageLookProfile> profiles,
    const WaterSeepageLookSettings& defaultLook,
    std::string_view targetSceneRole,
    bool forExport,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    std::span<const WaterSeepageSurfaceGuide> guides = {});
[[nodiscard]] WaterSeepageRuntimeContribution EvaluateWaterSeepageRuntimeContribution(
    const WaterSeepageRuntimeNode& node,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds);
[[nodiscard]] WaterSeepageRuntimeContribution EvaluateWaterSeepageGridContribution(
    const WaterSeepageSpatialGrid& grid,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds);
[[nodiscard]] std::string WaterSeepageTopologyFingerprint(const WaterSeepageSpatialGrid& grid);
[[nodiscard]] std::string WaterSeepageParamsFingerprint(const WaterSeepageSpatialGrid& grid);

struct WaterFieldNode {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 vector{1.0F, 0.0F, 0.0F};
    std::uint32_t sourcePointIndex = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t sourceLayerId = 0;
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Add;
    WaterEffectResponseSettings response{};
    float effectSpeed = 0.55F;
    bool flowBlocked = false;
    bool bridgeAllowed = false;
    bool bridgeBlocked = false;
    float wetness = 1.0F;
    float confidence = 1.0F;
    float surfaceConfidence = 1.0F;
    float pathStation = 0.0F;
    float distanceToGuide = 0.0F;
};

struct WaterFieldCache {
    std::uint32_t schemaVersion = 1;
    std::filesystem::path supportLayerPath;
    std::string supportSignature;
    std::string settingsFingerprint;
    std::string regionFingerprint;
    WaterFieldSettings settings{};
    std::vector<invisible_places::io::Float3> regionBoundary;
    std::vector<WaterFieldNode> nodes;
    bool stale = false;
};

struct WaterRegionSelectedPoint {
    std::uint32_t pointIndex = std::numeric_limits<std::uint32_t>::max();
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    float edgeDistance = 0.0F;
    float edgeWeight = 1.0F;
    std::vector<float> scalarValues;
    invisible_places::io::Float3 fieldVector{1.0F, 0.0F, 0.0F};
    float fieldWetness = 1.0F;
    float fieldConfidence = 1.0F;
    bool flowBlocked = false;
    bool bridgeAllowed = false;
    bool bridgeBlocked = false;
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Add;
    WaterEffectResponseSettings response{};
    std::uint32_t sourceLayerId = 0;
    float effectSpeed = 0.55F;
};

struct WaterRegionSelection {
    std::uint32_t layerId = 0;
    WaterEffectFeatureType featureType = WaterEffectFeatureType::Ripple;
    std::filesystem::path targetLayerSourcePath;
    std::string targetLayerKey;
    std::vector<invisible_places::io::Float3> boundary;
    std::vector<invisible_places::io::Float3> hull;
    invisible_places::io::Bounds3f bounds{};
    std::vector<WaterRegionSelectedPoint> points;

    [[nodiscard]] bool Valid() const { return boundary.size() >= 3U; }
};

struct WaterRegionSelectionOptions {
    bool previewOnly = false;
    std::span<const std::uint32_t> candidatePointIndices{};
    const glm::mat4* visibleViewProjection = nullptr;
    const std::stop_token* stopToken = nullptr;
    std::function<void(std::size_t, std::size_t)> progress;
};

struct WaterFieldSourcePoint {
    invisible_places::io::Float3 position{};
    std::uint32_t sourceId = 0;
    float radiusMeters = 0.10F;
    float strength = 1.0F;
    std::uint32_t seed = 1;
};

struct WaterTrailSample {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 tangent{1.0F, 0.0F, 0.0F};
    std::uint8_t red = 40;
    std::uint8_t green = 210;
    std::uint8_t blue = 255;
    float trailId = 0.0F;
    float sourceId = 0.0F;
    float pathId = 0.0F;
    float branchId = 0.0F;
    float trailSeed = 0.0F;
    float pointSeed = 0.0F;
    float trailDistance = 0.0F;
    float trailLength = 1.0F;
    float pointAge = 0.0F;
    float trailAge = 0.0F;
    float trailSpeed = 1.0F;
    float trailWidth = 0.006F;
    float trailStreakLength = 0.045F;
    float trailConfidence = 1.0F;
    float wetness = 1.0F;
    float featureType = 0.0F;
    float trailRole = 1.0F;
    float routeStartIndex = 0.0F;
    float routePointCount = 0.0F;
    float routeLength = 1.0F;
    float trailStartPhase = 0.0F;
    float trailLateralOffset = 0.0F;
    float trailLaneIndex = 0.0F;
    float trailLaneCount = 1.0F;
    float trailLanePitch = 0.00025F;
    float trailLaneSpan = 0.0F;
    float trailLaneCrossing = 0.22F;
    float trailCrossSeed = 0.0F;
};

struct WaterFieldTrailDiagnostics {
    std::uint32_t inputNodeCount = 0;
    std::uint32_t emittedPathCount = 0;
    std::uint32_t emittedSampleCount = 0;
    std::uint32_t acceptedBridgeCount = 0;
    std::uint32_t rejectedGapCount = 0;
    std::uint32_t manualNoFlowBlockCount = 0;
    std::uint32_t manualBridgeAllowedCount = 0;
    std::uint32_t manualBridgeBlockedCount = 0;
    std::uint32_t lowConfidenceFadeCount = 0;
    std::uint32_t lowConfidenceTerminationCount = 0;
    float maxAcceptedBridgeMeters = 0.0F;
    float minRejectedGapMeters = 0.0F;
};

struct WaterTrailOverlay {
    std::vector<WaterTrailSample> samples;
    invisible_places::io::Bounds3f bounds{};
    WaterFieldTrailDiagnostics fieldDiagnostics{};
};

struct WaterEffectPoint {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 tangent{1.0F, 0.0F, 0.0F};
    std::uint32_t sourcePointIndex = std::numeric_limits<std::uint32_t>::max();
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Add;
    std::uint8_t red = 120;
    std::uint8_t green = 220;
    std::uint8_t blue = 255;
    float mask = 1.0F;
    float edge = 1.0F;
    float value = 1.0F;
    float seed = 0.0F;
    float regionId = 0.0F;
    float distance = 0.0F;
    float linearCoord = 0.0F;
    float angle = 0.0F;
    float speed = 1.0F;
    float confidence = 1.0F;
    float emissionHint = 0.0F;
    float opacityHint = 0.0F;
    float opacityMultiplyHint = 1.0F;
    float sizeHint = 0.0F;
    float sizeMultiplyHint = 1.0F;
    float colourMixHint = 0.0F;
    float ripplePotential = 0.0F;
    float rippleEmissionHint = 0.0F;
    float rippleOpacityHint = 0.0F;
    float rippleOpacityMultiplyHint = 1.0F;
    float rippleSizeHint = 0.0F;
    float rippleSizeMultiplyHint = 1.0F;
    float rippleColourMixHint = 0.0F;
    float wavelength = 0.25F;
    float warp = 0.0F;
    float phase = 0.0F;
    float fieldFlowU = 0.0F;
    float fieldWetness = 1.0F;
    float fieldSurfaceConfidence = 1.0F;
    float featureType = 0.0F;
};

struct WaterEffectOverlay {
    std::vector<WaterEffectPoint> points;
    invisible_places::io::Bounds3f bounds{};
};

struct WaterEffectCompositionFields {
    std::vector<float> value;
    std::vector<float> emissionAdd;
    std::vector<float> opacityAdd;
    std::vector<float> opacityMultiply;
    std::vector<float> pointSizeAdd;
    std::vector<float> pointSizeMultiply;
    std::vector<float> colourRed;
    std::vector<float> colourGreen;
    std::vector<float> colourBlue;
    std::vector<float> colourMix;
    std::vector<float> rippleMask;
    std::vector<float> rippleEdge;
    std::vector<float> rippleValue;
    std::vector<float> rippleSeed;
    std::vector<float> rippleRegionId;
    std::vector<float> rippleDistance;
    std::vector<float> rippleLinearCoord;
    std::vector<float> rippleAngle;
    std::vector<float> rippleSpeed;
    std::vector<float> rippleConfidence;
    std::vector<float> rippleWavelength;
    std::vector<float> rippleWarp;
    std::vector<float> ripplePhase;
    std::size_t affectedPointCount = 0;
};

struct WaterParticleTrailSettings {
    float particleDensity = 1.0F;
    float particleJitter = 0.35F;
    float particleSpeed = 1.0F;
    float splineAnchorSpacing = 0.5F;
};

struct WaterParticleVisualSettings {
    float particleSizePixels = 14.0F;
    float particleOpacity = 0.24F;
    float colorVariation = 0.65F;
    float glow = 0.35F;
};

struct WaterSourceSettings {
    WaterPathGenerationSettings path{};
    WaterParticleTrailShapeSettings trailShape{};
};

using WaterVisualSettings = WaterParticleVisualSettings;

struct WaterSettingsBundle {
    WaterPathGenerationSettings path{};
    WaterParticleTrailSettings trail{};
    WaterParticleVisualSettings visual{};
};

using WaterBakeSettings = WaterPathGenerationSettings;
using WaterRenderSettings = WaterSettingsBundle;

struct WaterEmitter {
    std::uint32_t id = 0;
    std::string name = "Water Source";
    invisible_places::io::Float3 position{};
    float radius = 0.35F;
    float strength = 1.0F;
    float speed = 1.0F;
    WaterScaleMode scope = WaterScaleMode::Mid;
    WaterEmitterOrigin origin = WaterEmitterOrigin::Manual;
    WaterEmitterStatus status = WaterEmitterStatus::Accepted;
    float confidence = 1.0F;
    std::optional<std::uint32_t> parentId;
    WaterSourceSettingsAssignment sourceSettingsAssignment = WaterSourceSettingsAssignment::Default;
    std::optional<std::uint32_t> linkedSourceSettingsEmitterId;
    std::optional<WaterSourceSettings> sourceSettings;
    std::optional<WaterSourceSettings> tempSourceSettings;
    std::string pathProfileName = "Global";
    std::string laneProfileName = "Global";
    std::string trailProfileName = "Global";
};

struct WaterSceneSupportLayer {
    const invisible_places::io::LoadedPointCloud* cloud = nullptr;
    std::string role;
    float pointSpacingMeters = 0.0F;
    float samplingMultiplier = 1.0F;
};

struct WaterOverlayPoint {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    std::uint8_t red = 40;
    std::uint8_t green = 210;
    std::uint8_t blue = 255;
    float flowId = 0.0F;
    float emitterId = 0.0F;
    float pathDistance = 0.0F;
    float phase = 0.0F;
    float speed = 1.0F;
    float width = 1.0F;
    float confidence = 1.0F;
    float accumulation = 0.0F;
    float pooling = 0.0F;
    float particleRole = 0.0F;
    float pathStartIndex = 0.0F;
    float pathPointCount = 0.0F;
    float jitterSeed = 0.0F;
    float trailAge = 0.0F;
    float trailLength = 0.0F;
    float featureType = 0.0F;
    float regionId = 0.0F;
    float surfaceSteepness = 0.0F;
    float trailLaneId = 0.0F;
    float trailLateralOffset = 0.0F;
};

struct WaterOverlay {
    std::vector<WaterOverlayPoint> points;
    invisible_places::io::Bounds3f bounds{};
};

enum class WaterAnimatedTrailMotionMode {
    Path,
    VectorField
};

struct WaterAnimatedTrailPath {
    WaterAnimatedTrailMotionMode motionMode = WaterAnimatedTrailMotionMode::Path;
    std::uint32_t sourceId = 0;
    std::vector<WaterOverlayPoint> anchors;
};

struct WaterAnimatedTrailBuildSettings {
    std::uint32_t trailCountTotal = 700;
    std::uint32_t laneCount = 0;
    float trailLengthMeters = 0.75F;
    float trailPointSpacingMeters = 0.010F;
    float trailWidthMeters = 0.006F;
    float trailStreakLengthMeters = 0.045F;
    float surfaceOffsetMeters = 0.004F;
    float pathAttraction = 0.85F;
    float laneSpreadMeters = 0.12F;
    float turbulence = 0.06F;
    float laneCrossing = 0.22F;
    float trailSmoothness = 0.85F;
    float trailLooseness = 0.08F;
    float speedMetersPerSecond = 0.45F;
    std::uint32_t seed = 1;
    float featureType = 0.0F;
};

enum class WaterTrailBuildQuality {
    Preview,
    Final
};

struct WaterTrailBuildDiagnostics {
    double surfaceIndexBuildMs = 0.0;
    double routeMs = 0.0;
    double laneMs = 0.0;
    double particleMs = 0.0;
    std::uint64_t surfaceSampleCount = 0;
    std::uint32_t routedPathCount = 0;
    std::uint32_t emittedLaneCount = 0;
    std::uint32_t emittedParticleCount = 0;
};

struct TrailSurfaceIndexCell {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    float minZ = 0.0F;
    float maxZ = 0.0F;
    float confidence = 1.0F;
    std::uint32_t count = 0;
    bool hasNormal = false;
};

struct TrailSurfaceIndex {
    std::vector<TrailSurfaceIndexCell> cells;
    std::unordered_map<std::uint64_t, std::uint32_t> cellLookup;
    float cellSize = 0.05F;
    float searchRadius = 0.12F;
    float surfaceLift = 0.004F;
    std::uint64_t sampledPointCount = 0;
    double buildMilliseconds = 0.0;
};

enum class WaterPathBranchRole {
    Main,
    Secondary,
    Spread
};

enum class WaterPathTerminationReason {
    ReachedLength,
    NoSupport,
    MaxSteps,
    Loop,
    Duplicate,
    Empty
};

struct WaterPathAutoTuneDiagnostics {
    float estimatedPointSpacing = 0.0F;
    float supportVoxelSize = 0.0F;
    float maxBridgeDistance = 0.0F;
    float pathSampleSpacing = 0.0F;
    float branchSearchRadius = 0.0F;
    float averageConfidence = 0.0F;
    std::uint32_t iterationCount = 0;
    std::uint32_t pilotTraceCount = 0;
    std::uint32_t branchCount = 0;
    std::uint32_t lowConfidenceBranchCount = 0;
    std::string summary;
};

struct WaterPathBranch {
    std::uint32_t id = 0;
    std::optional<std::uint32_t> parentId;
    std::uint32_t emitterId = 0;
    WaterPathBranchRole role = WaterPathBranchRole::Main;
    WaterPathTerminationReason terminationReason = WaterPathTerminationReason::Empty;
    float confidence = 1.0F;
    float length = 0.0F;
    float flatness = 0.0F;
    std::uint32_t gapCount = 0;
    std::string bakeFingerprint;
    std::vector<WaterOverlayPoint> rawAnchors;
};

struct WaterPathAnalysisSample {
    std::uint32_t branchId = 0;
    std::uint32_t sampleIndex = 0;
    float pathDistance = 0.0F;
    float slope = 0.0F;
    float flatness = 0.0F;
    float curvature = 0.0F;
    float neighborDensity = 0.0F;
    float nearestPathDistance = 0.0F;
    float confluence = 0.0F;
    float channelWidth = 0.0F;
    float speed = 0.0F;
    float turbulence = 0.0F;
    float eddyPotential = 0.0F;
    float ripplePotential = 0.0F;
};

struct WaterPathBranchAnalysis {
    std::uint32_t branchId = 0;
    std::vector<WaterPathAnalysisSample> samples;
};

struct WaterPathAnalysisCache {
    std::uint32_t schemaVersion = 1;
    float analysisRadiusMeters = 0.0F;
    std::vector<WaterPathBranchAnalysis> branches;
};

struct WaterPathCache {
    std::uint32_t schemaVersion = 2;
    std::filesystem::path supportLayerPath;
    std::string supportSignature;
    std::string emitterSettingsFingerprint;
    WaterPathGenerationSettings requestedSettings{};
    WaterPathGenerationSettings tunedSettings{};
    WaterPathAutoTuneDiagnostics diagnostics{};
    std::vector<WaterPathBranch> branches;
    std::vector<std::uint32_t> hiddenBranchIds;
    std::optional<WaterPathAnalysisCache> analysis;
    bool stale = false;
};

[[nodiscard]] const char* WaterScaleModeName(WaterScaleMode mode);
[[nodiscard]] const char* WaterEmitterOriginName(WaterEmitterOrigin origin);
[[nodiscard]] const char* WaterEmitterStatusName(WaterEmitterStatus status);
[[nodiscard]] WaterPathGenerationSettings DefaultWaterPathGenerationSettings(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterSourceSettings DefaultWaterSourceSettings(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterAnimationTrailSettings DefaultWaterAnimationTrailSettings();
[[nodiscard]] WaterCausticLookSettings DefaultWaterCausticLookSettings();
[[nodiscard]] WaterVisualSettings DefaultWaterVisualSettings();
[[nodiscard]] WaterSettingsBundle DefaultWaterSettingsBundle(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterBakeSettings DefaultWaterBakeSettings(WaterScaleMode mode);
[[nodiscard]] bool WaterPathBakeInputsEqual(
    const WaterPathGenerationSettings& left,
    const WaterPathGenerationSettings& right);
[[nodiscard]] bool WaterSourceBakeInputsEqual(
    const WaterSourceSettings& left,
    const WaterSourceSettings& right);
[[nodiscard]] const WaterSourceSettings& ResolveWaterSourceSettings(
    const WaterEmitter& emitter,
    const WaterSourceSettings& defaultSettings);
[[nodiscard]] const WaterSourceSettings& ResolveWaterSourceSettings(
    const WaterEmitter& emitter,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] std::vector<WaterEmitter> SuggestWaterEmitters(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& existingEmitters,
    const WaterPathGenerationSettings& settings,
    std::uint32_t firstEmitterId,
    std::uint32_t maxSuggestions);

[[nodiscard]] std::optional<invisible_places::io::Float3> SnapEmitterToCloud(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::io::Float3& position,
    const WaterPathGenerationSettings& settings);

[[nodiscard]] WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterPathGenerationSettings& settings);

[[nodiscard]] WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] WaterPathCache GenerateWaterPathCache(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterPathGenerationSettings& settings);

[[nodiscard]] WaterPathCache GenerateWaterPathCache(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] invisible_places::io::LoadedPointCloud BuildCombinedWaterSupportCloud(
    std::span<const WaterSceneSupportLayer> layers,
    const WaterPathGenerationSettings& settings);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildCombinedWaterSupportCloud(
    std::span<const WaterSceneSupportLayer> layers,
    const WaterPathGenerationSettings& settings,
    std::span<const invisible_places::io::Float3> priorityPoints);

[[nodiscard]] std::vector<WaterSeepageSurfaceGuide> BuildWaterSeepageSurfaceGuides(
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSceneSupportLayer> supportLayers,
    std::size_t sampleLimit = 300'000U);

[[nodiscard]] WaterPathAnalysisCache BuildWaterPathAnalysis(const WaterPathCache& cache);
[[nodiscard]] bool WaterPathAnalysisCacheCompatible(const WaterPathCache& cache);
void EnsureWaterPathAnalysis(WaterPathCache* cache);

[[nodiscard]] std::shared_ptr<const TrailSurfaceIndex> BuildTrailSurfaceIndex(
    const invisible_places::io::LoadedPointCloud& cloud);
[[nodiscard]] std::shared_ptr<const TrailSurfaceIndex> BuildTrailSurfaceIndex(
    const invisible_places::io::LoadedPointCloud* cloud);
[[nodiscard]] std::uint64_t TrailSurfaceIndexSampleCount(const TrailSurfaceIndex& index);
[[nodiscard]] double TrailSurfaceIndexBuildMilliseconds(const TrailSurfaceIndex& index);

[[nodiscard]] WaterOverlay BuildWaterPathAnchorsFromCache(
    const WaterPathCache& cache,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const invisible_places::io::LoadedPointCloud* supportCloud,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailSettings& legacyTrailSettings,
    const WaterParticleVisualSettings& legacyVisualSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const invisible_places::io::LoadedPointCloud* supportCloud,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSourceSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSettingsBundle& settings);
[[nodiscard]] WaterRegionSelection BuildWaterRegionSelection(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterEffectLayer& layer,
    const WaterRegionSelectionOptions& options = {});
[[nodiscard]] std::vector<WaterRegionSelection> BuildWaterRegionSelections(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers,
    WaterEffectFeatureType featureType,
    const WaterRegionSelectionOptions& options = {});
[[nodiscard]] std::string WaterEffectLayersFingerprint(const std::vector<WaterEffectLayer>& layers);
[[nodiscard]] std::string WaterFieldSettingsFingerprint(const WaterFieldSettings& settings);
[[nodiscard]] WaterTrailOverlay BuildAnimatedWaterTrailOverlay(
    const std::vector<WaterAnimatedTrailPath>& paths,
    const WaterAnimatedTrailBuildSettings& settings);
[[nodiscard]] WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings);
[[nodiscard]] WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings,
    const WaterPathAnalysisCache* analysis);
[[nodiscard]] WaterFieldCache BuildFieldCacheFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFieldSettings& settings);
[[nodiscard]] WaterFieldCache BuildFieldCacheFromRegions(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers,
    const WaterFieldSettings& settings);
[[nodiscard]] WaterTrailOverlay BuildFieldTrailOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldTrailSettings& settings);
[[nodiscard]] WaterTrailOverlay BuildFieldTrailOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldTrailSettings& settings,
    const std::vector<WaterEmitter>& emitters);
[[nodiscard]] WaterTrailOverlay BuildRainTrailOverlay(
    std::span<const WaterSceneSupportLayer> supportLayers,
    const WaterRainCameraFrame& cameraFrame,
    const WaterRainSettings& settings,
    WaterRainDiagnostics* diagnostics = nullptr);
[[nodiscard]] MeshSurfaceCache BuildMeshSurfaceCache(
    const invisible_places::io::LoadedTriangleMesh& mesh,
    const WaterDynamicMeshFlowSettings& settings);
[[nodiscard]] MeshSurfaceProjection ProjectToMeshSurface(
    const MeshSurfaceCache& cache,
    const invisible_places::io::Float3& position);
[[nodiscard]] MeshSurfaceProjection ProjectRayToMeshSurface(
    const MeshSurfaceCache& cache,
    const invisible_places::io::Float3& rayOrigin,
    const invisible_places::io::Float3& rayDirection);
[[nodiscard]] WaterTrailOverlay BuildDynamicMeshWaterTrailOverlay(
    const MeshSurfaceCache& cache,
    const std::vector<WaterEmitter>& emitters,
    const WaterDynamicMeshFlowSettings& settings,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterDynamicMeshFlowDiagnostics* diagnostics = nullptr);
[[nodiscard]] WaterEffectOverlay GenerateRippleEffectOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers);
[[nodiscard]] WaterEffectOverlay GenerateRippleEffectOverlayFromPointIndices(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterEffectLayer& layer,
    const std::vector<std::uint32_t>& pointIndices);
[[nodiscard]] WaterEffectOverlay GenerateRippleEffectOverlayFromSelection(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterEffectLayer& layer,
    const WaterRegionSelection& selection);
[[nodiscard]] WaterEffectOverlay GenerateFieldSurfaceEffectOverlay(
    const WaterFieldCache& fieldCache,
    const WaterEffectLayer& layer);
[[nodiscard]] WaterEffectCompositionFields ComposeWaterEffectFields(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectOverlay>& overlays);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterTrailOverlayPointCloud(
    const WaterTrailOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName);
[[nodiscard]] std::vector<invisible_places::io::ScalarFieldStats> WaterTrailOverlayScalarFieldsForPointCount(
    std::uint64_t pointCount);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterEffectOverlayPointCloud(
    const WaterEffectOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterOverlayPointCloud(
    const WaterOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName);
[[nodiscard]] std::vector<invisible_places::io::Float3> BuildWaterRegionHull(
    const std::vector<invisible_places::io::Float3>& vertices);

bool SaveWaterFieldCacheBinary(
    const WaterFieldCache& cache,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);
[[nodiscard]] std::optional<WaterFieldCache> LoadWaterFieldCacheBinary(
    const std::filesystem::path& inputPath,
    std::string* errorMessage);
bool WriteWaterOverlayPly(
    const WaterOverlay& overlay,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);

}  // namespace invisible_places::water
