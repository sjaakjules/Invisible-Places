#pragma once

#include "io/PointCloudData.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

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

enum class WaterRunoffMode {
    Dew,
    LightRain
};

enum class WaterCausticPreviewTintMode {
    Off,
    PulseAfterRefresh,
    Always
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

struct WaterBasinRegion {
    std::uint32_t id = 0;
    std::string name = "Basin";
    std::vector<invisible_places::io::Float3> vertices;
    std::vector<invisible_places::io::Float3> hull;
    float baseZ = 0.0F;
    float heightAbove = 0.20F;
    float depthBelow = 0.20F;
    float density = 1.0F;
    std::optional<std::uint32_t> outletEdgeIndex;
    bool outletBlocked = true;
};

struct WaterRunoffRegion {
    std::uint32_t id = 0;
    std::string name = "Runoff";
    std::vector<invisible_places::io::Float3> vertices;
    std::vector<invisible_places::io::Float3> hull;
    WaterRunoffMode mode = WaterRunoffMode::Dew;
    float groundVoxelSize = 0.40F;
    float highPointFraction = 0.20F;
    float density = 1.0F;
    float pathLength = 16.0F;
    float maxSteps = 72.0F;
};

struct WaterCausticRegion {
    std::uint32_t id = 0;
    std::string name = "Caustics";
    std::filesystem::path targetLayerSourcePath;
    std::vector<invisible_places::io::Float3> vertices;
    std::vector<invisible_places::io::Float3> hull;
    float maskVoxelSize = 0.05F;
    float planeMaxResidual = 0.08F;
    float planeMaxSlope = 0.55F;
    float heightBand = 0.22F;
    float edgeBlendWidth = 0.60F;
    WaterCausticPreviewTintMode previewTintMode = WaterCausticPreviewTintMode::PulseAfterRefresh;
    bool enabled = true;
    bool maskDirty = true;
    bool maskStale = true;
};

struct WaterCausticMaskResult {
    std::vector<float> mask;
    std::vector<float> edge;
    std::vector<float> regionId;
    std::vector<float> planeDistance;
    std::vector<float> seed;
    std::uint64_t affectedPointCount = 0;
    bool hasAnyEnabledRegion = false;
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
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterOverlayPointCloud(
    const WaterOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName);
[[nodiscard]] std::vector<invisible_places::io::Float3> BuildWaterRegionHull(
    const std::vector<invisible_places::io::Float3>& vertices);
[[nodiscard]] float AverageWaterRegionZ(
    const std::vector<invisible_places::io::Float3>& vertices);
void RefreshWaterBasinRegionDerivedValues(WaterBasinRegion* region);
void RefreshWaterRunoffRegionDerivedValues(WaterRunoffRegion* region);
void RefreshWaterCausticRegionDerivedValues(WaterCausticRegion* region);
[[nodiscard]] WaterOverlay GenerateBasinHazeOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterBasinRegion>& regions);
[[nodiscard]] WaterOverlay GenerateRunoffOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterRunoffRegion>& regions,
    const WaterAnimationTrailSettings& animationTrailSettings);
[[nodiscard]] WaterCausticMaskResult GenerateCausticMask(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterCausticRegion>& regions);

bool WriteWaterOverlayPly(
    const WaterOverlay& overlay,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);

}  // namespace invisible_places::water
