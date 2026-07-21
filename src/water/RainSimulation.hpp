#pragma once

#include "io/PointCloudData.hpp"
#include "io/TransformMatrix.hpp"
#include "scene/PointCloudVariants.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::water {

inline constexpr float kRainCollisionResolutionMeters = 0.020F;
inline constexpr std::uint32_t kRainParticleCapacity = 32'768U;
inline constexpr std::uint32_t kRainImpactEventCapacity = 65'536U;
inline constexpr std::uint32_t kRainImpactGridDimension = 256U;
inline constexpr std::uint32_t kRainSandEventsPerCell = 8U;
inline constexpr std::uint32_t kRainRockEventsPerCell = 8U;
inline constexpr std::uint32_t kRainVegetationEventsPerCell = 4U;

enum class RainCollisionRole : std::uint32_t {
    None = 0U,
    Rock = 1U,
    Sand = 2U,
    Vegetation = 3U,
};

enum class RainIntensityPreset : std::uint32_t {
    LightMist = 0U,
    Rain = 1U,
    HeavyDownpour = 2U,
};

struct WaterRainVisualSettings {
    std::array<float, 3> colour{0.68F, 0.82F, 0.92F};
    float widthMeters = 0.0030F;
    float streakLengthMeters = 0.16F;
    float softness = 0.42F;
    float opacity = 0.58F;
    float emission = 0.16F;
    float minimumScreenPixels = 0.65F;
    float maximumScreenPixels = 4.0F;
};

struct RainRuntimeSettings {
    bool enabled = false;
    bool impactEffectsEnabled = true;
    bool sandEffectsEnabled = true;
    bool rockEffectsEnabled = true;
    bool vegetationEffectsEnabled = true;
    RainIntensityPreset intensityPreset = RainIntensityPreset::Rain;
    std::string visualProfileName = "Rain Fine Lines";
    std::uint32_t activeParticleCount = 9'000U;
    std::uint32_t seed = 101U;
    float rainLevel = 1.0F;
    float density = 0.55F;
    float fallSpeedMetersPerSecond = 8.0F;
    float dropletSizeScale = 1.0F;
    float opacityScale = 1.0F;
    float emissionScale = 1.0F;
    float spawnHeightMeters = 4.0F;
    float spawnRadiusMeters = 6.0F;
    float cameraDeathDistanceMeters = 28.0F;
    float windDirectionX = 1.0F;
    float windDirectionY = 0.0F;
    float windSpeedMetersPerSecond = 0.30F;
    float turbulence = 0.45F;
    float gustStrength = 0.35F;
    float gustScaleMeters = 8.0F;
    float gustSpeedMetersPerSecond = 2.5F;
    float weatherFrontStrength = 0.40F;
    float weatherFrontScaleMeters = 12.0F;
    float weatherFrontSpeedMetersPerSecond = 1.5F;
    float sandEffectScale = 1.0F;
    float rockEffectScale = 1.0F;
    float vegetationEffectScale = 1.0F;
};

struct RainIntensityMultipliers {
    float density = 1.0F;
    float speed = 1.0F;
    float width = 1.0F;
    float length = 1.0F;
    float opacity = 1.0F;
    float emission = 1.0F;
    float effectEnergy = 1.0F;
    float windResponse = 1.0F;
};

[[nodiscard]] RainRuntimeSettings DefaultRainRuntimeSettings();
[[nodiscard]] RainIntensityMultipliers RainIntensityValues(RainIntensityPreset preset);
[[nodiscard]] WaterRainVisualSettings RainVisualPreset(std::string_view name);
[[nodiscard]] std::array<std::string_view, 3> RainVisualPresetNames();
[[nodiscard]] float RainImpactGridWorldSpan(const RainRuntimeSettings& settings);

struct RainCollisionSource {
    std::filesystem::path sourcePath;
    RainCollisionRole role = RainCollisionRole::None;
    io::Matrix4d localToWorld{};
    std::uint32_t spacingMicrometres = 0U;
    bool hasTransform = false;
    bool isFallback = false;
};

struct RainCollisionSourceMetadata {
    std::filesystem::path sourcePath;
    RainCollisionRole role = RainCollisionRole::None;
    std::uint32_t spacingMicrometres = 0U;
    std::uint64_t fileSize = 0U;
    std::int64_t modificationTicks = 0;
    bool isFallback = false;
};

struct RainCollisionSample {
    io::Float3 position{};
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    RainCollisionRole role = RainCollisionRole::None;
};

struct RainSurfaceCell {
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    float rockHeight = -std::numeric_limits<float>::infinity();
    float sandHeight = -std::numeric_limits<float>::infinity();
    io::Float3 rockNormal{0.0F, 0.0F, 1.0F};
    io::Float3 sandNormal{0.0F, 0.0F, 1.0F};
    float rockConfidence = 0.0F;
    float sandConfidence = 0.0F;
    std::uint32_t rockSampleCount = 0U;
    std::uint32_t sandSampleCount = 0U;
};

struct RainVegetationVoxel {
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    std::int32_t cellZ = 0;
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    std::uint32_t sampleCount = 0U;
};

struct RainCollisionCache {
    std::uint32_t schemaVersion = 1U;
    float resolutionMeters = kRainCollisionResolutionMeters;
    std::string signature;
    std::vector<RainCollisionSourceMetadata> sources;
    std::vector<RainSurfaceCell> surfaceCells;
    std::vector<RainVegetationVoxel> vegetationVoxels;
    io::Bounds3f bounds;
    std::uint64_t sourcePointCount = 0U;
    double buildMilliseconds = 0.0;
    std::uint64_t revision = 0U;
};

struct RainCollisionBuildResult {
    RainCollisionCache cache;
    std::vector<std::string> warnings;
    std::string errorMessage;
    bool loadedFromDisk = false;
    bool success = false;
    bool cancelled = false;
};

struct RainCollisionHit {
    io::Float3 position{};
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    RainCollisionRole role = RainCollisionRole::None;
    float segmentTime = 1.0F;
    bool hit = false;
};

struct RainGpuSurfaceSlot {
    std::int32_t cellX = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellY = std::numeric_limits<std::int32_t>::min();
    float rockHeight = -std::numeric_limits<float>::infinity();
    float sandHeight = -std::numeric_limits<float>::infinity();
    std::uint32_t packedRockNormal = 0U;
    std::uint32_t packedSandNormal = 0U;
    float rockConfidence = 0.0F;
    float sandConfidence = 0.0F;
};

struct RainGpuVegetationSlot {
    std::int32_t cellX = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellY = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellZ = std::numeric_limits<std::int32_t>::min();
    std::uint32_t packedNormal = 0U;
};

struct RainCollisionGpuData {
    std::vector<RainGpuSurfaceSlot> surfaceTable;
    std::vector<RainGpuVegetationSlot> vegetationTable;
    std::uint32_t surfaceMask = 0U;
    std::uint32_t vegetationMask = 0U;
    std::uint32_t maximumProbeCount = 0U;
    std::uint64_t sourceRevision = 0U;
};

[[nodiscard]] std::vector<RainCollisionSource> SelectRainCollisionSources(
    const scene::ScenePointCloudGroup& group,
    std::uint32_t preferredSpacingMicrometres = 5'000U);
[[nodiscard]] std::string RainCollisionCacheSignature(
    std::span<const RainCollisionSource> sources,
    float resolutionMeters = kRainCollisionResolutionMeters);
[[nodiscard]] std::filesystem::path RainCollisionCachePath(
    const std::filesystem::path& cacheRoot,
    std::string_view signature);
[[nodiscard]] RainCollisionBuildResult BuildRainCollisionCache(
    std::span<const RainCollisionSource> sources,
    const std::filesystem::path& cacheRoot = {},
    const std::atomic_bool* cancelRequested = nullptr);
[[nodiscard]] RainCollisionCache BuildRainCollisionCacheFromSamples(
    std::span<const RainCollisionSample> samples,
    float resolutionMeters = kRainCollisionResolutionMeters);
[[nodiscard]] bool SaveRainCollisionCache(
    const RainCollisionCache& cache,
    const std::filesystem::path& filePath,
    std::string* errorMessage = nullptr);
[[nodiscard]] bool LoadRainCollisionCache(
    const std::filesystem::path& filePath,
    std::string_view expectedSignature,
    RainCollisionCache* cache,
    std::string* errorMessage = nullptr);
[[nodiscard]] RainCollisionHit TraceRainCollision(
    const RainCollisionCache& cache,
    const io::Float3& segmentStart,
    const io::Float3& segmentEnd);
[[nodiscard]] RainCollisionGpuData BuildRainCollisionGpuData(const RainCollisionCache& cache);

struct RainParticle {
    io::Float3 position{};
    io::Float3 previousPosition{};
    io::Float3 velocity{};
    float ageSeconds = 0.0F;
    float visibility = 1.0F;
    std::uint32_t generation = 0U;
    std::uint32_t randomState = 0U;
    bool active = false;
};

struct RainImpactEvent {
    io::Float3 position{};
    float birthTimeSeconds = 0.0F;
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    float radiusMeters = 0.04F;
    RainCollisionRole role = RainCollisionRole::None;
    float lifetimeSeconds = 1.0F;
    float energy = 1.0F;
    std::uint32_t seed = 0U;
};

struct RainSimulationFrame {
    RainRuntimeSettings settings{};
    WaterRainVisualSettings visual{};
    io::Float3 cameraPosition{};
    io::Float3 spawnCentre{};
    float timeSeconds = 0.0F;
    float deltaSeconds = 1.0F / 30.0F;
};

struct RainSimulationDiagnostics {
    std::uint32_t activeParticles = 0U;
    std::uint32_t collisionCount = 0U;
    std::uint32_t respawnCount = 0U;
    std::uint32_t emittedEvents = 0U;
    std::uint32_t sandEvents = 0U;
    std::uint32_t rockEvents = 0U;
    std::uint32_t vegetationEvents = 0U;
    std::uint32_t escapedParticles = 0U;
};

class RainSimulator {
public:
    explicit RainSimulator(std::uint32_t particleCapacity = kRainParticleCapacity);

    void Reset(std::uint32_t seed = 0U);
    [[nodiscard]] RainSimulationDiagnostics Advance(
        const RainSimulationFrame& frame,
        const RainCollisionCache& collisionCache);

    [[nodiscard]] std::span<const RainParticle> Particles() const { return particles_; }
    [[nodiscard]] std::span<const RainImpactEvent> Events() const { return events_; }
    [[nodiscard]] std::uint32_t EventWriteIndex() const { return eventWriteIndex_; }
    [[nodiscard]] std::uint32_t Capacity() const { return static_cast<std::uint32_t>(particles_.size()); }

private:
    void SpawnParticle(std::uint32_t index, const RainSimulationFrame& frame);
    void EmitImpact(
        const RainCollisionHit& hit,
        const RainSimulationFrame& frame,
        const RainParticle& particle,
        RainSimulationDiagnostics* diagnostics);

    std::vector<RainParticle> particles_;
    std::vector<RainImpactEvent> events_;
    std::uint32_t eventWriteIndex_ = 0U;
    std::uint32_t seed_ = 0U;
    float previousTimeSeconds_ = -std::numeric_limits<float>::infinity();
};

struct RainImpactGridCell {
    std::array<std::uint32_t, kRainSandEventsPerCell> sand{};
    std::array<std::uint32_t, kRainRockEventsPerCell> rock{};
    std::array<std::uint32_t, kRainVegetationEventsPerCell> vegetation{};
    std::uint32_t sandCount = 0U;
    std::uint32_t rockCount = 0U;
    std::uint32_t vegetationCount = 0U;
};

struct RainImpactGrid {
    io::Float3 origin{};
    float cellSizeMeters = 0.125F;
    std::uint32_t dimension = kRainImpactGridDimension;
    std::vector<RainImpactGridCell> cells;
    std::vector<RainImpactEvent> events;
    std::uint32_t overflowCount = 0U;
};

struct RainImpactEffect {
    float opacity = 0.0F;
    float emission = 0.0F;
    float sizeScale = 1.0F;
    float colourBlend = 0.0F;
};

[[nodiscard]] RainImpactGrid BuildRainImpactGrid(
    std::span<const RainImpactEvent> events,
    const io::Float3& cameraPosition,
    float timeSeconds,
    float worldSpanMeters = 32.0F);
[[nodiscard]] RainImpactEffect EvaluateRainImpact(
    const RainImpactGrid& grid,
    RainCollisionRole pointRole,
    const io::Float3& point,
    const io::Float3& normal,
    float timeSeconds);

}  // namespace invisible_places::water
