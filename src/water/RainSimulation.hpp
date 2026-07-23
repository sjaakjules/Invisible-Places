#pragma once

#include "water/WaterSurfaceCache.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::water {

inline constexpr std::uint32_t kRainParticleCapacity = 32'768U;
inline constexpr std::uint32_t kRainImpactEventCapacity = 65'536U;
inline constexpr std::uint32_t kRainImpactGridDimension = 256U;
inline constexpr std::uint32_t kRainSandEventsPerCell = 8U;
inline constexpr std::uint32_t kRainRockEventsPerCell = 16U;
inline constexpr std::uint32_t kRainVegetationEventsPerCell = 4U;

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

// These controls are evaluated from the already-resident Rain collision
// tables. They deliberately do not alter particle/event capacities, cache
// identity, or any point-cloud topology.
struct RainNearSurfaceSettings {
    float approachDistanceMeters = 0.18F;
    float minimumSpeedFactor = 0.30F;
    float squish = 0.65F;
    float normalAlignment = 0.75F;
};

// World-space dimensions shared by the CPU/offline path and mirrored in the
// Rain vertex shader. Slowdown shortens the motion streak while squish widens
// it and progressively changes its coverage from a streak to an ellipse.
struct RainParticleVisualShape {
    float widthMeters = 0.003F;
    float lengthMeters = 0.16F;
    float ellipseBlend = 0.0F;
};

[[nodiscard]] RainParticleVisualShape EvaluateRainParticleVisualShape(
    float authoredWidthMeters,
    float authoredLengthMeters,
    float surfaceProximity,
    const RainNearSurfaceSettings& settings);

struct RainRockImpactSettings {
    float edgeBreakup = 0.35F;
    float spreadSpeed = 1.60F;
    float centreFalloff = 0.65F;
    float heightBias = 0.75F;
    float persistence = 1.35F;
};

struct RainVegetationImpactSettings {
    float twinkle = 1.80F;
    float propagationMetersPerSecond = 0.65F;
    float hopSpacingMeters = 0.070F;
    float streamWidthMeters = 0.010F;
    float streamSpread = 0.65F;
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
    RainNearSurfaceSettings nearSurface{};
    RainRockImpactSettings rockImpact{};
    RainVegetationImpactSettings vegetationImpact{};
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

struct RainCollisionHit {
    io::Float3 position{};
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    WaterSurfaceRole role = WaterSurfaceRole::None;
    float segmentTime = 1.0F;
    bool hit = false;
};

[[nodiscard]] RainCollisionHit TraceRainCollision(
    const WaterSurfaceCache& cache,
    const io::Float3& segmentStart,
    const io::Float3& segmentEnd);

struct RainParticle {
    io::Float3 position{};
    io::Float3 previousPosition{};
    io::Float3 velocity{};
    float ageSeconds = 0.0F;
    float visibility = 1.0F;
    io::Float3 surfaceNormal{0.0F, 0.0F, 1.0F};
    float surfaceProximity = 0.0F;
    std::uint32_t generation = 0U;
    std::uint32_t randomState = 0U;
    bool active = false;
};

struct RainImpactEvent {
    io::Float3 position{};
    float birthTimeSeconds = 0.0F;
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    float radiusMeters = 0.04F;
    WaterSurfaceRole role = WaterSurfaceRole::None;
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
        const WaterSurfaceCache& surfaceCache);

    [[nodiscard]] std::span<const RainParticle> Particles() const { return particles_; }
    [[nodiscard]] std::span<const RainImpactEvent> Events() const { return events_; }
    [[nodiscard]] std::uint32_t EventWriteIndex() const { return eventWriteIndex_; }
    [[nodiscard]] std::uint32_t Capacity() const {
        return static_cast<std::uint32_t>(particles_.size());
    }

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
    std::uint32_t sandMask = 0U;
    std::uint32_t rockMask = 0U;
    std::uint32_t vegetationMask = 0U;
};

struct RainImpactGrid {
    io::Float3 origin{};
    float cellSizeMeters = 0.125F;
    std::uint32_t dimension = kRainImpactGridDimension;
    std::vector<RainImpactGridCell> cells;
    std::vector<RainImpactEvent> events;
    RainRockImpactSettings rockImpact{};
    RainVegetationImpactSettings vegetationImpact{};
    std::uint32_t overflowCount = 0U;
};

struct RainImpactEffect {
    float opacity = 0.0F;
    float emission = 0.0F;
    float sizeScale = 1.0F;
    float colourBlend = 0.0F;
};

// Pure narrow-phase ROCK evaluator used by the offline renderer and by the
// deterministic Vulkan shader-equivalence test. The result is before event
// energy and material response coefficients are applied.
[[nodiscard]] float EvaluateRockRainImpactValue(
    const RainImpactEvent& event,
    const io::Float3& point,
    const io::Float3& pointNormal,
    float timeSeconds,
    const RainRockImpactSettings& settings = {});

// Pure SAND narrow phase. sandWaterMask is one on the flooded/downhill side
// of the shoreline and zero on dry, uphill sand.
[[nodiscard]] float EvaluateSandRainImpactValue(
    const RainImpactEvent& event,
    const io::Float3& point,
    float timeSeconds,
    float sandWaterMask = 1.0F);

[[nodiscard]] RainImpactGrid BuildRainImpactGrid(
    std::span<const RainImpactEvent> events,
    const io::Float3& cameraPosition,
    float timeSeconds,
    float worldSpanMeters = 32.0F,
    const RainRockImpactSettings& rockImpact = {},
    const RainVegetationImpactSettings& vegetationImpact = {});
[[nodiscard]] RainImpactEffect EvaluateRainImpact(
    const RainImpactGrid& grid,
    WaterSurfaceRole pointRole,
    const io::Float3& point,
    const io::Float3& normal,
    float timeSeconds,
    float sandWaterMask = 1.0F);

}  // namespace invisible_places::water
