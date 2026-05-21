#pragma once

#include "io/PointCloudData.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
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

struct WaterPathGenerationSettings {
    WaterScaleMode legacyScaleMode = WaterScaleMode::Mid;
    float supportVoxelSize = 0.05F;
    float maxBridgeDistance = 0.30F;
    float smoothing = 0.35F;
    float pathLength = 14.0F;
    float pathSampleSpacing = 0.06F;
    std::uint32_t maxSteps = 220;
    std::uint32_t supportSampleLimit = 180000;
};

struct WaterParticleTrailShapeSettings {
    float particleJitter = 0.35F;
    float splineAnchorSpacing = 0.5F;
};

struct WaterAnimationTrailSettings {
    float particleDensity = 1.0F;
    float particleSpeed = 1.0F;
    float colorVariation = 0.65F;
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
};

struct WaterOverlay {
    std::vector<WaterOverlayPoint> points;
    invisible_places::io::Bounds3f bounds{};
};

[[nodiscard]] const char* WaterScaleModeName(WaterScaleMode mode);
[[nodiscard]] const char* WaterEmitterOriginName(WaterEmitterOrigin origin);
[[nodiscard]] const char* WaterEmitterStatusName(WaterEmitterStatus status);
[[nodiscard]] WaterPathGenerationSettings DefaultWaterPathGenerationSettings(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterSourceSettings DefaultWaterSourceSettings(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterAnimationTrailSettings DefaultWaterAnimationTrailSettings();
[[nodiscard]] WaterVisualSettings DefaultWaterVisualSettings();
[[nodiscard]] WaterSettingsBundle DefaultWaterSettingsBundle(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterBakeSettings DefaultWaterBakeSettings(WaterScaleMode mode);
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

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailSettings& legacyTrailSettings,
    const WaterParticleVisualSettings& legacyVisualSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSourceSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSettingsBundle& settings);

bool WriteWaterOverlayPly(
    const WaterOverlay& overlay,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);

}  // namespace invisible_places::water
