#pragma once

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

enum class WaterFieldOutputMode {
    Streamlines,
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

struct WaterEffectLayer {
    std::uint32_t id = 0;
    std::string name = "Ripple";
    WaterEffectFeatureType featureType = WaterEffectFeatureType::Ripple;
    WaterRippleOverlayType rippleOverlayType = WaterRippleOverlayType::CausticLace;
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Add;
    std::filesystem::path targetLayerSourcePath;
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

[[nodiscard]] std::vector<WaterRippleRuntimeMembership> BuildWaterRippleRuntimeMemberships(
    const WaterRegionSelection& selection,
    std::uint32_t paramIndex);

[[nodiscard]] WaterRippleRuntimeContribution EvaluateWaterRippleRuntimeContribution(
    const WaterRippleRuntimeParams& params,
    const WaterRippleRuntimeMembership& membership,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds);

struct WaterFlowStreamSettings {
    bool enabled = true;
    std::uint32_t streamCountTotal = 700;
    float streamLengthMeters = 0.75F;
    float streamPointSpacingMeters = 0.010F;
    float streamWidthMeters = 0.006F;
    float streamWorldLengthMeters = 0.045F;
    float surfaceOffsetMeters = 0.004F;
    float pathAttraction = 0.85F;
    float laneSpreadMeters = 0.12F;
    float laneCrossing = 0.22F;
    float streamSmoothness = 0.85F;
    float streamLooseness = 0.08F;
    float turbulence = 0.06F;
    float speedMetersPerSecond = 0.45F;
    std::uint32_t seed = 1;
};

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

struct WaterFieldStreamSettings {
    bool enabled = true;
    std::uint32_t streamlineCount = 850;
    float seedSpacingMeters = 0.025F;
    float streamlineLengthMeters = 0.85F;
    float stepLengthMeters = 0.012F;
    float streamlineWidthMeters = 0.005F;
    float streamWorldLengthMeters = 0.045F;
    float momentum = 0.84F;
    float maxTurnAngleDegrees = 12.0F;
    float speedMetersPerSecond = 0.38F;
    bool fadeOnLowConfidence = true;
};

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

struct WaterStreamSample {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 tangent{1.0F, 0.0F, 0.0F};
    std::uint8_t red = 40;
    std::uint8_t green = 210;
    std::uint8_t blue = 255;
    float streamId = 0.0F;
    float sourceId = 0.0F;
    float pathId = 0.0F;
    float branchId = 0.0F;
    float streamSeed = 0.0F;
    float pointSeed = 0.0F;
    float streamDistance = 0.0F;
    float streamLength = 1.0F;
    float pointAge = 0.0F;
    float streamAge = 0.0F;
    float streamSpeed = 1.0F;
    float streamWidth = 0.006F;
    float streamWorldLength = 0.045F;
    float streamConfidence = 1.0F;
    float wetness = 1.0F;
    float featureType = 0.0F;
    float streamRole = 1.0F;
    float routeStartIndex = 0.0F;
    float routePointCount = 0.0F;
    float routeLength = 1.0F;
    float streamStartPhase = 0.0F;
    float streamLateralOffset = 0.0F;
    float streamLaneIndex = 0.0F;
    float streamLaneCount = 1.0F;
    float streamLanePitch = 0.00025F;
    float streamLaneSpan = 0.0F;
    float streamLaneCrossing = 0.22F;
    float streamCrossSeed = 0.0F;
};

struct WaterFieldStreamDiagnostics {
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

struct WaterStreamOverlay {
    std::vector<WaterStreamSample> samples;
    invisible_places::io::Bounds3f bounds{};
    WaterFieldStreamDiagnostics fieldDiagnostics{};
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
    float trailLengthMeters = 0.75F;
    float trailPointSpacingMeters = 0.010F;
    float trailWidthMeters = 0.006F;
    float trailWorldLengthMeters = 0.045F;
    float surfaceOffsetMeters = 0.004F;
    float laneSpreadMeters = 0.12F;
    float turbulence = 0.06F;
    float laneCrossing = 0.22F;
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
    std::vector<WaterOverlayPoint> rawAnchors;
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
[[nodiscard]] WaterStreamOverlay BuildAnimatedWaterTrailOverlay(
    const std::vector<WaterAnimatedTrailPath>& paths,
    const WaterAnimatedTrailBuildSettings& settings);
[[nodiscard]] WaterStreamOverlay BuildFlowStreamOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowStreamSettings& settings);
[[nodiscard]] WaterFieldCache BuildFieldCacheFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFieldSettings& settings);
[[nodiscard]] WaterFieldCache BuildFieldCacheFromRegions(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers,
    const WaterFieldSettings& settings);
[[nodiscard]] WaterStreamOverlay BuildFieldStreamOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldStreamSettings& settings);
[[nodiscard]] WaterStreamOverlay BuildFieldStreamOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldStreamSettings& settings,
    const std::vector<WaterEmitter>& emitters);
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
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterStreamOverlayPointCloud(
    const WaterStreamOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName);
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
