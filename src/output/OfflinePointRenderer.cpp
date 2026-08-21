#include "output/OfflinePointRenderer.hpp"

#include "renderer/pointcloud/Colormap.hpp"

#include "camera/OrbitCamera.hpp"
#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::output {

namespace {

constexpr std::size_t kOfflinePointChunkSize = 1'000'000U;
constexpr std::size_t kWaterPhaseFieldSlot = 3U;
constexpr std::size_t kWaterSpeedFieldSlot = 4U;
constexpr std::size_t kWaterWidthFieldSlot = 5U;
constexpr std::size_t kWaterParticleRoleFieldSlot = 9U;
constexpr std::size_t kWaterPathStartFieldSlot = 10U;
constexpr std::size_t kWaterPathCountFieldSlot = 11U;
constexpr std::size_t kWaterJitterSeedFieldSlot = 12U;
constexpr std::size_t kWaterAgeFieldSlot = 13U;
constexpr std::size_t kWaterFeatureTypeFieldSlot = 15U;
constexpr std::size_t kWaterTrailRoleFieldSlot = 0U;
constexpr std::size_t kWaterTrailSeedFieldSlot = 5U;
constexpr std::size_t kWaterTrailDistanceFieldSlot = 7U;
constexpr std::size_t kWaterTrailLengthFieldSlot = 8U;
constexpr std::size_t kWaterTrailRouteStartFieldSlot = 9U;
constexpr std::size_t kWaterTrailRouteCountFieldSlot = 10U;
constexpr std::size_t kWaterTrailRouteLengthFieldSlot = 11U;
constexpr std::size_t kWaterTrailStartPhaseFieldSlot = 12U;
constexpr std::size_t kWaterTrailLateralOffsetFieldSlot = 13U;
constexpr std::size_t kWaterTrailAgeFieldSlot = 15U;
constexpr std::size_t kWaterTrailSpeedFieldSlot = 16U;
constexpr std::size_t kWaterTrailWidthFieldSlot = 17U;
constexpr std::size_t kWaterTrailStreakLengthFieldSlot = 18U;
constexpr std::size_t kWaterTrailTangentXFieldSlot = 22U;
constexpr std::size_t kWaterTrailTangentYFieldSlot = 23U;
constexpr std::size_t kWaterTrailTangentZFieldSlot = 24U;
constexpr std::size_t kWaterTrailEndpointFadeFlagsFieldSlot = 31U;
constexpr std::size_t kWaterTrailStartFadeFullDistanceFieldSlot = 32U;
constexpr std::size_t kWaterTrailStartFadeRandomBeginDistanceFieldSlot = 33U;
constexpr std::size_t kWaterTrailEndFadeFullDistanceFieldSlot = 34U;
constexpr std::size_t kWaterTrailEndFadeRandomBeginDistanceFieldSlot = 35U;
constexpr float kWaterParticleSpeedScale = 0.12F;
constexpr float kPointCloudAntialiasFeatherPixels = 1.0F;

float Clamp01(float value) {
    return std::clamp(value, 0.0F, 1.0F);
}

float SmoothStep(float edge0, float edge1, float value);
glm::vec3 SurfaceMotionNoiseVector(const glm::vec3& position, float time);

float SandRainWaterMask(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    float worldZ) {
    if (!invisible_places::renderer::pointcloud::
            PointCloudStyleHasShorelineWaveRegion(style)) {
        return 1.0F;
    }
    const bool heightFoam =
        style.shorelineWaveAlgorithm ==
        invisible_places::renderer::pointcloud::
            PointCloudShorelineWaveAlgorithm::HeightFoam;
    const float boundaryZ = heightFoam
        ? style.shorelineHeightFoam.runupZ
        : style.shorelineBoundaryZ;
    const float edgeFade = std::max(
        0.001F,
        heightFoam
            ? std::clamp(
                  style.shorelineHeightFoam.edgeFadeMeters,
                  0.0F,
                  10.0F)
            : std::clamp(
                  style.shorelineEdgeFadeMeters,
                  0.0F,
                  10.0F));
    return SmoothStep(
        -edgeFade,
        edgeFade,
        boundaryZ - worldZ);
}

glm::vec3 ToGlm(const invisible_places::io::Float3& value) {
    return {value.x, value.y, value.z};
}

invisible_places::io::Float3 FromGlm(const glm::vec3& value) {
    return {value.x, value.y, value.z};
}

float ScalarFieldValue(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::style::RenderParameterBinding& binding,
    std::size_t pointIndex) {
    if (binding.fieldMap.fieldSlot < 0 ||
        static_cast<std::size_t>(binding.fieldMap.fieldSlot) >= cloud.scalarFields.size()) {
        return 0.0F;
    }

    const auto fieldIndex = static_cast<std::size_t>(binding.fieldMap.fieldSlot);
    const auto valueIndex = cloud.ScalarFieldValueIndex(fieldIndex, pointIndex);
    if (valueIndex >= cloud.scalarFieldValues.size()) {
        return 0.0F;
    }

    return cloud.scalarFieldValues[valueIndex];
}

float ScalarFieldValueBySlot(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t fieldSlot,
    std::size_t pointIndex) {
    if (fieldSlot >= cloud.scalarFields.size() || pointIndex >= cloud.PointCount()) {
        return 0.0F;
    }
    const auto valueIndex = cloud.ScalarFieldValueIndex(fieldSlot, pointIndex);
    if (valueIndex >= cloud.scalarFieldValues.size()) {
        return 0.0F;
    }
    return cloud.scalarFieldValues[valueIndex];
}

bool HasWaterParticleFields(const OfflinePointLayer& layer) {
    return layer.generatedWaterOverlay &&
           layer.cloud != nullptr &&
           layer.style.flowAnimation &&
           !layer.style.waterTrailOverlay &&
           layer.cloud->scalarFields.size() > kWaterJitterSeedFieldSlot;
}

bool HasWaterTrailFields(const OfflinePointLayer& layer) {
    return layer.generatedWaterOverlay &&
           layer.cloud != nullptr &&
           layer.style.waterTrailOverlay &&
           layer.cloud->scalarFields.size() > kWaterTrailTangentZFieldSlot;
}

float WaterParticleTravel(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float timeSeconds) {
    const float phase = ScalarFieldValueBySlot(cloud, kWaterPhaseFieldSlot, pointIndex);
    const float speed = std::max(0.02F, ScalarFieldValueBySlot(cloud, kWaterSpeedFieldSlot, pointIndex));
    return std::fmod(
        phase + std::max(0.0F, timeSeconds) * speed * kWaterParticleSpeedScale + 1.0F,
        1.0F);
}

bool IsWaterSteam(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex) {
    if (cloud.scalarFields.size() <= kWaterFeatureTypeFieldSlot) {
        return false;
    }
    const float featureType = ScalarFieldValueBySlot(cloud, kWaterFeatureTypeFieldSlot, pointIndex);
    return featureType > 0.5F && featureType < 1.5F;
}

float WaterTrailFade(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex) {
    if (cloud.scalarFields.size() <= kWaterAgeFieldSlot) {
        return 1.0F;
    }
    const float age = Clamp01(ScalarFieldValueBySlot(cloud, kWaterAgeFieldSlot, pointIndex));
    return std::pow(1.0F - SmoothStep(0.0F, 1.0F, age), 1.35F);
}

float WaterParticleFade(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float timeSeconds) {
    const float travel = WaterParticleTravel(cloud, pointIndex, timeSeconds);
    const float seed = ScalarFieldValueBySlot(cloud, kWaterJitterSeedFieldSlot, pointIndex);
    if (IsWaterSteam(cloud, pointIndex)) {
        const float birth = SmoothStep(0.0F, 0.10F, travel);
        const float dissipate = 1.0F - SmoothStep(0.70F, 1.0F, travel);
        const float turbulence = 0.82F + 0.18F * std::sin((travel + seed * 1.618F) * 6.28318530718F);
        return birth * dissipate * turbulence * WaterTrailFade(cloud, pointIndex);
    }
    const float inFade = std::clamp(travel / 0.08F, 0.0F, 1.0F);
    const float outFade = 1.0F - std::clamp((travel - 0.92F) / 0.08F, 0.0F, 1.0F);
    const float shimmer = 0.78F + 0.22F * std::sin((travel + seed * 1.618F) * 6.28318530718F);
    return inFade * outFade * shimmer * WaterTrailFade(cloud, pointIndex);
}

float WaterParticleSizeScale(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float timeSeconds) {
    if (!IsWaterSteam(cloud, pointIndex)) {
        return 1.0F;
    }
    const float travel = WaterParticleTravel(cloud, pointIndex, timeSeconds);
    const float seed = ScalarFieldValueBySlot(cloud, kWaterJitterSeedFieldSlot, pointIndex);
    return 0.72F + SmoothStep(0.0F, 1.0F, travel) * (1.43F + seed * 0.35F);
}

float HashWater01(std::uint32_t value) {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    value ^= value >> 16U;
    return static_cast<float>(value & 0x00ffffffU) / 16777215.0F;
}

float PositiveFract(float value) {
    return value - std::floor(value);
}

glm::vec3 SafeWaterLateral(const glm::vec3& tangent, const glm::vec3& fallback) {
    glm::vec3 lateral = glm::cross(tangent, glm::vec3{0.0F, 0.0F, 1.0F});
    if (glm::dot(lateral, lateral) <= 1.0e-8F) {
        lateral = glm::cross(tangent, glm::vec3{0.0F, 1.0F, 0.0F});
    }
    if (glm::dot(lateral, lateral) <= 1.0e-8F) {
        lateral = fallback;
    }
    return glm::normalize(lateral);
}

glm::vec3 CatmullRomWater(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& p2,
    const glm::vec3& p3,
    float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    return 0.5F * (
        (2.0F * p1) +
        (-p0 + p2) * t +
        ((2.0F * p0) - (5.0F * p1) + (4.0F * p2) - p3) * t2 +
        (-p0 + (3.0F * p1) - (3.0F * p2) + p3) * t3);
}

glm::vec3 SafeCentripetalWaterMix(
    const glm::vec3& a,
    const glm::vec3& b,
    float ta,
    float tb,
    float t) {
    const float denominator = tb - ta;
    return std::abs(denominator) > 1.0e-6F
        // Barry-Goldman requires A1/A3 to extrapolate while t is on the
        // P1-P2 knot interval. A saturated weight reduces the curve to the
        // straight control polygon and disagrees with the CPU trail builder.
        ? glm::mix(a, b, (t - ta) / denominator)
        : a;
}

glm::vec3 CentripetalCatmullRomWater(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& p2,
    const glm::vec3& p3,
    float u) {
    const float t0 = 0.0F;
    const float t1 = t0 + std::sqrt(std::max(glm::length(p1 - p0), 1.0e-6F));
    const float t2 = t1 + std::sqrt(std::max(glm::length(p2 - p1), 1.0e-6F));
    const float t3 = t2 + std::sqrt(std::max(glm::length(p3 - p2), 1.0e-6F));
    const float t = std::lerp(t1, t2, std::clamp(u, 0.0F, 1.0F));
    const glm::vec3 a1 = SafeCentripetalWaterMix(p0, p1, t0, t1, t);
    const glm::vec3 a2 = SafeCentripetalWaterMix(p1, p2, t1, t2, t);
    const glm::vec3 a3 = SafeCentripetalWaterMix(p2, p3, t2, t3, t);
    const glm::vec3 b1 = SafeCentripetalWaterMix(a1, a2, t0, t2, t);
    const glm::vec3 b2 = SafeCentripetalWaterMix(a2, a3, t1, t3, t);
    return SafeCentripetalWaterMix(b1, b2, t1, t2, t);
}

std::size_t WaterTrailRouteStart(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex) {
    return static_cast<std::size_t>(
        std::max(0.0F, std::floor(ScalarFieldValueBySlot(cloud, kWaterTrailRouteStartFieldSlot, pointIndex) + 0.5F)));
}

std::size_t WaterTrailRouteCount(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex) {
    return static_cast<std::size_t>(
        std::max(0.0F, std::floor(ScalarFieldValueBySlot(cloud, kWaterTrailRouteCountFieldSlot, pointIndex) + 0.5F)));
}

float WaterTrailTravelPhase(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    std::size_t pointIndex,
    float timeSeconds) {
    const float routeLength = std::max(0.001F, ScalarFieldValueBySlot(cloud, kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailDistance = std::max(0.0F, ScalarFieldValueBySlot(cloud, kWaterTrailDistanceFieldSlot, pointIndex));
    const float trailAge = ScalarFieldValueBySlot(cloud, kWaterTrailAgeFieldSlot, pointIndex);
    const float baseStartPhase = ScalarFieldValueBySlot(cloud, kWaterTrailStartPhaseFieldSlot, pointIndex);
    const auto activity = invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales(
        style.waterFlowActivity,
        ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex));
    // Strength fades the settled trail population but must not alter this
    // absolute-time phase. The authored runtime speed scale is the only live
    // velocity control, matching all three point vertex shaders.
    const float speed =
        std::max(0.0F, ScalarFieldValueBySlot(cloud, kWaterTrailSpeedFieldSlot, pointIndex)) *
        activity.speed *
        invisible_places::renderer::pointcloud::SanitizeWaterFlowSpeedScale(
            style.waterFlowSpeedScale);
    const float trailStartPhase = PositiveFract(
        baseStartPhase +
        trailAge +
        std::max(0.0F, timeSeconds) * speed / routeLength);
    return trailStartPhase + trailDistance / routeLength;
}

bool WaterTrailStyleGeometryAvailable(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    return style.waterTrailStyleGeometry &&
           style.surfelDiameter.active &&
           style.surfelDiameter.mode == invisible_places::style::ParameterSourceMode::Constant &&
           style.surfelDiameter.constantValue[0] > 0.0F &&
           style.waterStreakAspect > 0.0F;
}

float WaterTrailWidth(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    const float baseWidth = WaterTrailStyleGeometryAvailable(style)
                                ? std::max(0.0001F, style.surfelDiameter.constantValue[0])
                                : std::max(
                                      0.0001F,
                                      ScalarFieldValueBySlot(
                                          cloud,
                                          kWaterTrailWidthFieldSlot,
                                          pointIndex));
    const auto activity = invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales(
        style.waterFlowActivity,
        ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex));
    return baseWidth * activity.width;
}

float WaterTrailStreakLength(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    const float baseLength = WaterTrailStyleGeometryAvailable(style)
                                 ? std::max(0.0001F, style.surfelDiameter.constantValue[0]) *
                                       std::max(1.0F, style.waterStreakAspect)
                                 : std::max(
                                       0.001F,
                                       ScalarFieldValueBySlot(
                                           cloud,
                                           kWaterTrailStreakLengthFieldSlot,
                                           pointIndex));
    const auto activity = invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales(
        style.waterFlowActivity,
        ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex));
    return baseLength * activity.visibleLength;
}

float WaterTrailVisibility(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    std::size_t pointIndex,
    float timeSeconds) {
    if (ScalarFieldValueBySlot(cloud, kWaterTrailRoleFieldSlot, pointIndex) < 0.5F) {
        return 0.0F;
    }
    const auto activity = invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales(
        style.waterFlowActivity,
        ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex));
    if (activity.trailVisibility <= 0.0F) {
        return 0.0F;
    }
    const float phase = WaterTrailTravelPhase(cloud, style, pointIndex, timeSeconds);
    const float routeLength = std::max(0.001F, ScalarFieldValueBySlot(cloud, kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailStreakLength = WaterTrailStreakLength(cloud, pointIndex, style);
    const float endFeather = std::clamp(trailStreakLength / routeLength, 0.001F, 0.10F);
    float endpointVisibility = 1.0F;
    if (cloud.scalarFields.size() > kWaterTrailEndFadeRandomBeginDistanceFieldSlot) {
        const auto flags = static_cast<std::uint32_t>(std::max(
            0.0F,
            std::floor(
                ScalarFieldValueBySlot(
                    cloud,
                    kWaterTrailEndpointFadeFlagsFieldSlot,
                    pointIndex) +
                0.5F)));
        const float trailSeed = std::clamp(
            ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex),
            0.0F,
            1.0F);
        const float distanceFromStart = PositiveFract(phase) * routeLength;
        const auto fadeRamp = [](float distance, float fullDistance, float randomDistance, float randomAmount) {
            const float full = std::max(0.0F, fullDistance);
            if (full <= 1.0e-5F) {
                return 1.0F;
            }
            const float begin = std::min(
                std::max(0.0F, randomDistance) * std::clamp(randomAmount, 0.0F, 1.0F),
                std::max(0.0F, full - 1.0e-5F));
            return SmoothStep(begin, full, std::max(0.0F, distance));
        };
        if ((flags & 1U) != 0U) {
            endpointVisibility *= fadeRamp(
                distanceFromStart,
                ScalarFieldValueBySlot(
                    cloud,
                    kWaterTrailStartFadeFullDistanceFieldSlot,
                    pointIndex),
                ScalarFieldValueBySlot(
                    cloud,
                    kWaterTrailStartFadeRandomBeginDistanceFieldSlot,
                    pointIndex),
                PositiveFract(trailSeed * 0.754877666F + 0.137F));
        }
        if ((flags & 2U) != 0U) {
            endpointVisibility *= fadeRamp(
                routeLength - distanceFromStart,
                ScalarFieldValueBySlot(
                    cloud,
                    kWaterTrailEndFadeFullDistanceFieldSlot,
                    pointIndex),
                ScalarFieldValueBySlot(
                    cloud,
                    kWaterTrailEndFadeRandomBeginDistanceFieldSlot,
                    pointIndex),
                PositiveFract(trailSeed * 0.569840291F + 0.731F));
        }
    }
    return activity.trailVisibility * endpointVisibility *
           (1.0F - SmoothStep(1.0F - endFeather, 1.0F, phase));
}

bool WaterTrailRouteValid(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t routeStart,
    std::size_t routeCount) {
    return routeCount >= 2U &&
           routeStart < cloud.positions.size() &&
           routeStart + routeCount <= cloud.positions.size();
}

std::pair<std::size_t, float> WaterTrailRouteSegment(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float phase) {
    const auto routeStart = WaterTrailRouteStart(cloud, pointIndex);
    const auto routeCount = WaterTrailRouteCount(cloud, pointIndex);
    if (!WaterTrailRouteValid(cloud, routeStart, routeCount)) {
        return {0U, 0.0F};
    }

    const float routePhase = PositiveFract(phase);
    const float routePosition = routePhase * static_cast<float>(routeCount - 1U);
    const auto anchorOffset = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(routePosition)),
        routeCount - 2U);
    return {anchorOffset, PositiveFract(routePosition)};
}

glm::vec3 WaterTrailRoutePosition(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float phase,
    glm::vec3 fallbackPosition) {
    const auto routeStart = WaterTrailRouteStart(cloud, pointIndex);
    const auto routeCount = WaterTrailRouteCount(cloud, pointIndex);
    if (!WaterTrailRouteValid(cloud, routeStart, routeCount)) {
        return fallbackPosition;
    }

    const auto [anchorOffset, t] = WaterTrailRouteSegment(cloud, pointIndex, phase);
    const glm::vec3 p1 = ToGlm(cloud.positions[routeStart + anchorOffset]);
    const glm::vec3 p2 = ToGlm(cloud.positions[routeStart + anchorOffset + 1U]);
    const glm::vec3 p0 = anchorOffset > 0U
        ? ToGlm(cloud.positions[routeStart + anchorOffset - 1U])
        : p1 + (p1 - p2);
    const glm::vec3 p3 = anchorOffset + 2U < routeCount
        ? ToGlm(cloud.positions[routeStart + anchorOffset + 2U])
        : p2 + (p2 - p1);
    return CentripetalCatmullRomWater(p0, p1, p2, p3, t);
}

glm::vec3 WaterTrailRouteTangent(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float phase) {
    const auto routeStart = WaterTrailRouteStart(cloud, pointIndex);
    const auto routeCount = WaterTrailRouteCount(cloud, pointIndex);
    if (!WaterTrailRouteValid(cloud, routeStart, routeCount)) {
        const glm::vec3 tangent{
            ScalarFieldValueBySlot(cloud, kWaterTrailTangentXFieldSlot, pointIndex),
            ScalarFieldValueBySlot(cloud, kWaterTrailTangentYFieldSlot, pointIndex),
            ScalarFieldValueBySlot(cloud, kWaterTrailTangentZFieldSlot, pointIndex)};
        return glm::dot(tangent, tangent) > 1.0e-8F ? glm::normalize(tangent) : glm::vec3{1.0F, 0.0F, 0.0F};
    }

    const auto [anchorOffset, t] = WaterTrailRouteSegment(cloud, pointIndex, phase);
    const glm::vec3 p1 = ToGlm(cloud.positions[routeStart + anchorOffset]);
    const glm::vec3 p2 = ToGlm(cloud.positions[routeStart + anchorOffset + 1U]);
    const glm::vec3 p0 = anchorOffset > 0U
        ? ToGlm(cloud.positions[routeStart + anchorOffset - 1U])
        : p1 + (p1 - p2);
    const glm::vec3 p3 = anchorOffset + 2U < routeCount
        ? ToGlm(cloud.positions[routeStart + anchorOffset + 2U])
        : p2 + (p2 - p1);
    constexpr float kTangentProbe = 0.01F;
    const glm::vec3 tangent =
        CentripetalCatmullRomWater(p0, p1, p2, p3, std::min(1.0F, t + kTangentProbe)) -
        CentripetalCatmullRomWater(p0, p1, p2, p3, std::max(0.0F, t - kTangentProbe));
    const glm::vec3 fallbackTangent = p2 - p1;
    return glm::dot(tangent, tangent) > 1.0e-8F
        ? glm::normalize(tangent)
        : (glm::dot(fallbackTangent, fallbackTangent) > 1.0e-8F
            ? glm::normalize(fallbackTangent)
            : glm::vec3{1.0F, 0.0F, 0.0F});
}

glm::vec3 WaterTrailRouteNormal(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float phase) {
    if (!cloud.hasNormals || cloud.normals.empty()) {
        return {0.0F, 0.0F, 1.0F};
    }

    const auto routeStart = WaterTrailRouteStart(cloud, pointIndex);
    const auto routeCount = WaterTrailRouteCount(cloud, pointIndex);
    if (!WaterTrailRouteValid(cloud, routeStart, routeCount)) {
        const glm::vec3 normal = pointIndex < cloud.normals.size() ? ToGlm(cloud.normals[pointIndex]) : glm::vec3{0.0F, 0.0F, 1.0F};
        return glm::dot(normal, normal) > 1.0e-8F ? glm::normalize(normal) : glm::vec3{0.0F, 0.0F, 1.0F};
    }

    const auto [anchorOffset, t] = WaterTrailRouteSegment(cloud, pointIndex, phase);
    const auto p1Offset = anchorOffset;
    const auto p2Offset = std::min<std::size_t>(anchorOffset + 1U, routeCount - 1U);
    const glm::vec3 p1 = routeStart + p1Offset < cloud.normals.size()
                             ? ToGlm(cloud.normals[routeStart + p1Offset])
                             : glm::vec3{0.0F, 0.0F, 1.0F};
    const glm::vec3 p2 = routeStart + p2Offset < cloud.normals.size()
                             ? ToGlm(cloud.normals[routeStart + p2Offset])
                             : glm::vec3{0.0F, 0.0F, 1.0F};
    const glm::vec3 normal = glm::mix(p1, p2, t);
    return glm::dot(normal, normal) > 1.0e-8F ? glm::normalize(normal) : glm::vec3{0.0F, 0.0F, 1.0F};
}

glm::vec3 ResolveWaterTrailPosition(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    std::size_t pointIndex,
    float timeSeconds,
    glm::vec3 basePosition) {
    const float phase = WaterTrailTravelPhase(cloud, style, pointIndex, timeSeconds);
    const glm::vec3 routePosition = WaterTrailRoutePosition(cloud, pointIndex, phase, basePosition);
    const glm::vec3 routeTangent = WaterTrailRouteTangent(cloud, pointIndex, phase);
    const glm::vec3 routeNormal = WaterTrailRouteNormal(cloud, pointIndex, phase);
    glm::vec3 lateral = glm::cross(routeNormal, routeTangent);
    if (glm::dot(lateral, lateral) <= 1.0e-8F) {
        lateral = SafeWaterLateral(routeTangent, glm::vec3{1.0F, 0.0F, 0.0F});
    } else {
        lateral = glm::normalize(lateral);
    }
    const float lateralOffset = ScalarFieldValueBySlot(cloud, kWaterTrailLateralOffsetFieldSlot, pointIndex);
    const float trailSeed = std::clamp(
        ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex),
        0.0F,
        1.0F);
    const auto activity = invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales(
        style.waterFlowActivity,
        trailSeed);
    const float baseWidth = WaterTrailStyleGeometryAvailable(style)
                                ? std::max(0.0001F, style.surfelDiameter.constantValue[0])
                                : std::max(
                                      0.0001F,
                                      ScalarFieldValueBySlot(
                                          cloud,
                                          kWaterTrailWidthFieldSlot,
                                          pointIndex));
    const float motionPhase =
        (phase * 1.70F + trailSeed * 3.17F + std::max(0.0F, timeSeconds) * 0.35F) *
        6.28318530718F;
    const float microMotion = std::sin(motionPhase) * baseWidth * activity.lateralMotion;
    return routePosition + lateral * (lateralOffset + microMotion);
}

glm::vec3 JitteredWaterAnchorPosition(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pathStart,
    std::size_t pathCount,
    std::size_t anchorOffset,
    float particleSeed,
    float pathJitter) {
    const auto clampedOffset = std::min<std::size_t>(anchorOffset, pathCount - 1U);
    const auto anchorIndex = pathStart + clampedOffset;
    const glm::vec3 basePosition = ToGlm(cloud.positions[anchorIndex]);
    if (pathJitter <= 0.0001F) {
        return basePosition;
    }

    const auto prevOffset = clampedOffset > 0U ? clampedOffset - 1U : clampedOffset;
    const auto nextOffset = std::min<std::size_t>(clampedOffset + 1U, pathCount - 1U);
    const glm::vec3 prevPosition = ToGlm(cloud.positions[pathStart + prevOffset]);
    const glm::vec3 nextPosition = ToGlm(cloud.positions[pathStart + nextOffset]);
    glm::vec3 tangent = nextPosition - prevPosition;
    if (glm::dot(tangent, tangent) <= 1.0e-8F) {
        return basePosition;
    }

    tangent = glm::normalize(tangent);
    const glm::vec3 lateral = SafeWaterLateral(tangent, glm::vec3{1.0F, 0.0F, 0.0F});
    glm::vec3 secondary = glm::cross(tangent, lateral);
    if (glm::dot(secondary, secondary) <= 1.0e-8F) {
        secondary = {0.0F, 0.0F, 1.0F};
    }
    secondary = glm::normalize(secondary);

    const auto seedBits = static_cast<std::uint32_t>(std::clamp(particleSeed, 0.0F, 1.0F) * 16777215.0F);
    const auto hashBase = seedBits ^ (static_cast<std::uint32_t>(clampedOffset) * 747796405U);
    const float lateralNoise = (HashWater01(hashBase ^ 0x9e3779b9U) - 0.5F) * 2.0F;
    const float secondaryNoise = (HashWater01(hashBase ^ 0x85ebca6bU) - 0.5F) * 2.0F;
    const float startFade = glm::smoothstep(0.0F, 2.0F, static_cast<float>(clampedOffset));
    const float endFade =
        glm::smoothstep(0.0F, 2.0F, static_cast<float>((pathCount - 1U) - clampedOffset));
    const float endpointFade = std::min(startFade, endFade);
    const float anchorWidth = std::clamp(
        ScalarFieldValueBySlot(cloud, kWaterWidthFieldSlot, anchorIndex),
        0.001F,
        100.0F);
    const float amplitude = anchorWidth * std::clamp(pathJitter, 0.0F, 3.0F) * 0.45F * endpointFade;
    return basePosition + ((lateral * lateralNoise) + (secondary * secondaryNoise * 0.22F)) * amplitude;
}

glm::vec3 ResolveWaterParticlePosition(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float timeSeconds,
    glm::vec3 basePosition) {
    const auto pathStart = static_cast<std::size_t>(
        std::max(0.0F, std::floor(ScalarFieldValueBySlot(cloud, kWaterPathStartFieldSlot, pointIndex) + 0.5F)));
    const auto pathCount = static_cast<std::size_t>(
        std::max(0.0F, std::floor(ScalarFieldValueBySlot(cloud, kWaterPathCountFieldSlot, pointIndex) + 0.5F)));
    if (pathCount < 2U || pathStart >= cloud.positions.size() || pathStart + pathCount > cloud.positions.size()) {
        return basePosition;
    }

    const float pathPosition = WaterParticleTravel(cloud, pointIndex, timeSeconds) * static_cast<float>(pathCount - 1U);
    const auto anchorOffset = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(pathPosition)),
        pathCount - 1U);
    const float t = pathPosition - std::floor(pathPosition);
    const auto p0Offset = anchorOffset > 0U ? anchorOffset - 1U : anchorOffset;
    const auto p1Offset = anchorOffset;
    const auto p2Offset = std::min<std::size_t>(anchorOffset + 1U, pathCount - 1U);
    const auto p3Offset = std::min<std::size_t>(anchorOffset + 2U, pathCount - 1U);
    const float seed = ScalarFieldValueBySlot(cloud, kWaterJitterSeedFieldSlot, pointIndex);
    const float pathJitter = std::clamp(ScalarFieldValueBySlot(cloud, kWaterWidthFieldSlot, pointIndex), 0.0F, 3.0F);
    const glm::vec3 p0 = JitteredWaterAnchorPosition(cloud, pathStart, pathCount, p0Offset, seed, pathJitter);
    const glm::vec3 p1 = JitteredWaterAnchorPosition(cloud, pathStart, pathCount, p1Offset, seed, pathJitter);
    const glm::vec3 p2 = JitteredWaterAnchorPosition(cloud, pathStart, pathCount, p2Offset, seed, pathJitter);
    const glm::vec3 p3 = JitteredWaterAnchorPosition(cloud, pathStart, pathCount, p3Offset, seed, pathJitter);
    return CatmullRomWater(p0, p1, p2, p3, t);
}

glm::vec3 ResolveWaterParticleTangent(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float timeSeconds) {
    const auto pathStart = static_cast<std::size_t>(
        std::max(0.0F, std::floor(ScalarFieldValueBySlot(cloud, kWaterPathStartFieldSlot, pointIndex) + 0.5F)));
    const auto pathCount = static_cast<std::size_t>(
        std::max(0.0F, std::floor(ScalarFieldValueBySlot(cloud, kWaterPathCountFieldSlot, pointIndex) + 0.5F)));
    if (pathCount < 2U || pathStart >= cloud.positions.size() || pathStart + pathCount > cloud.positions.size()) {
        return {0.0F, 0.0F, 0.0F};
    }

    const float role = ScalarFieldValueBySlot(cloud, kWaterParticleRoleFieldSlot, pointIndex);
    auto anchorOffset = std::min<std::size_t>(pointIndex > pathStart ? pointIndex - pathStart : 0U, pathCount - 1U);
    if (role >= 0.5F && role < 1.5F) {
        const float pathPosition = WaterParticleTravel(cloud, pointIndex, timeSeconds) * static_cast<float>(pathCount - 1U);
        anchorOffset = std::min<std::size_t>(
            static_cast<std::size_t>(std::floor(pathPosition)),
            pathCount - 1U);
    }

    const auto previousOffset = anchorOffset > 0U ? anchorOffset - 1U : anchorOffset;
    const auto nextOffset = std::min<std::size_t>(anchorOffset + 1U, pathCount - 1U);
    const float seed = ScalarFieldValueBySlot(cloud, kWaterJitterSeedFieldSlot, pointIndex);
    const float pathJitter = std::clamp(ScalarFieldValueBySlot(cloud, kWaterWidthFieldSlot, pointIndex), 0.0F, 3.0F);
    const glm::vec3 previous = JitteredWaterAnchorPosition(cloud, pathStart, pathCount, previousOffset, seed, pathJitter);
    const glm::vec3 next = JitteredWaterAnchorPosition(cloud, pathStart, pathCount, nextOffset, seed, pathJitter);
    const glm::vec3 tangent = next - previous;
    return glm::dot(tangent, tangent) > 1.0e-8F ? glm::normalize(tangent) : glm::vec3{0.0F, 0.0F, 0.0F};
}

float EvaluateBinding(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::style::RenderParameterBinding& binding,
    std::size_t pointIndex) {
    const invisible_places::io::ScalarFieldStats* fieldStats = nullptr;
    if (binding.mode == invisible_places::style::ParameterSourceMode::FieldMapped) {
        if (binding.fieldMap.fieldSlot < 0 ||
            static_cast<std::size_t>(binding.fieldMap.fieldSlot) >= cloud.scalarFields.size() ||
            pointIndex >= cloud.PointCount()) {
            return binding.constantValue[0];
        }

        const auto fieldSlot = static_cast<std::size_t>(binding.fieldMap.fieldSlot);
        const auto valueIndex = cloud.ScalarFieldValueIndex(fieldSlot, pointIndex);
        if (valueIndex >= cloud.scalarFieldValues.size()) {
            return binding.constantValue[0];
        }
        fieldStats = &cloud.scalarFields[fieldSlot];
    }

    return invisible_places::style::EvaluateScalarBinding(
        binding,
        ScalarFieldValue(cloud, binding, pointIndex),
        fieldStats);
}

float EvaluateBindingOrDefault(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::style::RenderParameterBinding& binding,
    std::size_t pointIndex,
    float inactiveDefault) {
    if (!binding.active) {
        return inactiveDefault;
    }
    return EvaluateBinding(cloud, binding, pointIndex);
}

std::uint32_t InactivePointBindingCount(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    std::uint32_t count = 0;
    count += style.pointSize.active ? 0U : 1U;
    count += style.surfelDiameter.active ? 0U : 1U;
    count += style.opacity.active ? 0U : 1U;
    count += style.emissiveStrength.active ? 0U : 1U;
    count += style.depthFade.active ? 0U : 1U;
    count += style.colormapPosition.active ? 0U : 1U;
    return count;
}

glm::vec3 SourceRgb(std::uint32_t packedColor) {
    return {
        static_cast<float>(packedColor & 0xFFU) / 255.0F,
        static_cast<float>((packedColor >> 8U) & 0xFFU) / 255.0F,
        static_cast<float>((packedColor >> 16U) & 0xFFU) / 255.0F,
    };
}

glm::vec3 RgbToHsl(glm::vec3 color) {
    color = glm::clamp(color, glm::vec3{0.0F}, glm::vec3{1.0F});
    const float maxChannel = std::max(std::max(color.r, color.g), color.b);
    const float minChannel = std::min(std::min(color.r, color.g), color.b);
    const float delta = maxChannel - minChannel;
    const float lightness = (maxChannel + minChannel) * 0.5F;
    if (delta <= 1.0e-5F) {
        return {0.0F, 0.0F, lightness};
    }

    const float saturation =
        lightness > 0.5F
            ? delta / std::max(1.0e-5F, 2.0F - maxChannel - minChannel)
            : delta / std::max(1.0e-5F, maxChannel + minChannel);
    float hue = 0.0F;
    if (maxChannel == color.r) {
        hue = ((color.g - color.b) / delta) + (color.g < color.b ? 6.0F : 0.0F);
    } else if (maxChannel == color.g) {
        hue = ((color.b - color.r) / delta) + 2.0F;
    } else {
        hue = ((color.r - color.g) / delta) + 4.0F;
    }
    return {hue / 6.0F, saturation, lightness};
}

float HueToRgb(float p, float q, float t) {
    if (t < 0.0F) {
        t += 1.0F;
    }
    if (t > 1.0F) {
        t -= 1.0F;
    }
    if (t < (1.0F / 6.0F)) {
        return p + ((q - p) * 6.0F * t);
    }
    if (t < 0.5F) {
        return q;
    }
    if (t < (2.0F / 3.0F)) {
        return p + ((q - p) * ((2.0F / 3.0F) - t) * 6.0F);
    }
    return p;
}

glm::vec3 HslToRgb(glm::vec3 hsl) {
    if (hsl.y <= 1.0e-5F) {
        return glm::vec3{hsl.z};
    }

    const float q = hsl.z < 0.5F
                        ? hsl.z * (1.0F + hsl.y)
                        : hsl.z + hsl.y - (hsl.z * hsl.y);
    const float p = (2.0F * hsl.z) - q;
    return {
        HueToRgb(p, q, hsl.x + (1.0F / 3.0F)),
        HueToRgb(p, q, hsl.x),
        HueToRgb(p, q, hsl.x - (1.0F / 3.0F)),
    };
}

glm::vec3 ApplyColorize(
    glm::vec3 baseColor,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    const float amount = std::clamp(style.colorizeAmount, 0.0F, 1.0F);
    if (amount <= 1.0e-5F) {
        return baseColor;
    }

    const auto sourceHsl = RgbToHsl(baseColor);
    const auto tintHsl = RgbToHsl({
        style.colorizeColor[0],
        style.colorizeColor[1],
        style.colorizeColor[2],
    });
    // Carry most of the selected colour's lightness (matching the GPU
    // colorize) so brightness and vividness of the selection participate.
    const float colorizedLightness = std::clamp(
        std::lerp(sourceHsl.z, tintHsl.z, 0.65F),
        0.0F,
        1.0F);
    const auto colorized =
        HslToRgb({tintHsl.x, tintHsl.y, colorizedLightness});
    return glm::mix(baseColor, colorized, amount);
}

std::optional<float> ResolveTimingColouriseValue(
    const OfflinePointLayer& layer,
    const invisible_places::renderer::pointcloud::ResolvedTimingColouriseEffect&
        effect,
    std::size_t pointIndex) {
    using invisible_places::renderer::pointcloud::TimingColouriseSource;
    if (layer.cloud == nullptr) {
        return std::nullopt;
    }
    const auto& cloud = *layer.cloud;
    switch (effect.source) {
        case TimingColouriseSource::ScalarField:
            if (effect.scalarFieldSlot < 0 ||
                static_cast<std::size_t>(effect.scalarFieldSlot) >=
                    cloud.scalarFields.size()) {
                return std::nullopt;
            }
            return ScalarFieldValueBySlot(
                cloud,
                static_cast<std::size_t>(effect.scalarFieldSlot),
                pointIndex);
        case TimingColouriseSource::NormalX:
        case TimingColouriseSource::NormalY:
        case TimingColouriseSource::NormalZ:
            if (!cloud.hasNormals || pointIndex >= cloud.normals.size()) {
                return std::nullopt;
            }
            if (effect.source == TimingColouriseSource::NormalX) {
                return cloud.normals[pointIndex].x;
            }
            if (effect.source == TimingColouriseSource::NormalY) {
                return cloud.normals[pointIndex].y;
            }
            return cloud.normals[pointIndex].z;
    }
    return std::nullopt;
}

struct ResolvedTimingColouriseMask {
    float normalizedValue = 0.0F;
    float edgeMask = 0.0F;
};

std::optional<ResolvedTimingColouriseMask> ResolveTimingColouriseMask(
    const OfflinePointLayer& layer,
    const invisible_places::renderer::pointcloud::ResolvedTimingColouriseEffect&
        effect,
    std::size_t pointIndex) {
    if (!effect.enabled ||
        !std::isfinite(effect.lowerBound) ||
        !std::isfinite(effect.upperBound) ||
        effect.upperBound <= effect.lowerBound) {
        return std::nullopt;
    }
    const auto value = ResolveTimingColouriseValue(
        layer,
        effect,
        pointIndex);
    if (!value.has_value() || !std::isfinite(value.value())) {
        return std::nullopt;
    }

    const float span = effect.upperBound - effect.lowerBound;
    const float fadeFraction = std::clamp(
        std::isfinite(effect.edgeFadeFraction)
            ? effect.edgeFadeFraction
            : 0.10F,
        -0.5F,
        0.5F);
    const float outwardWidth =
        span * std::max(-fadeFraction, 0.0F);
    if (value.value() < effect.lowerBound - outwardWidth ||
        value.value() > effect.upperBound + outwardWidth) {
        return std::nullopt;
    }

    ResolvedTimingColouriseMask result;
    result.normalizedValue = std::clamp(
        (value.value() - effect.lowerBound) / span,
        0.0F,
        1.0F);
    result.edgeMask = 1.0F;
    if (fadeFraction > 1.0e-6F) {
        result.edgeMask = std::min(
            SmoothStep(0.0F, fadeFraction, result.normalizedValue),
            SmoothStep(
                0.0F,
                fadeFraction,
                1.0F - result.normalizedValue));
    } else if (fadeFraction < -1.0e-6F) {
        result.edgeMask = std::min(
            SmoothStep(
                effect.lowerBound - outwardWidth,
                effect.lowerBound,
                value.value()),
            1.0F - SmoothStep(
                effect.upperBound,
                effect.upperBound + outwardWidth,
                value.value()));
    }
    result.edgeMask = Clamp01(result.edgeMask);
    return result;
}

glm::vec4 SampleTimingColouriseLut(
    const invisible_places::renderer::pointcloud::ResolvedTimingColouriseEffect&
        effect,
    float normalizedValue) {
    constexpr std::size_t kLastSample =
        invisible_places::renderer::pointcloud::kTimingColouriseLutSamples - 1U;
    const float scaled =
        Clamp01(normalizedValue) * static_cast<float>(kLastSample);
    const std::size_t lowerIndex = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(scaled)),
        kLastSample);
    const std::size_t upperIndex =
        std::min<std::size_t>(lowerIndex + 1U, kLastSample);
    const float fraction = scaled - static_cast<float>(lowerIndex);
    const auto& lower = effect.rgbaLut[lowerIndex];
    const auto& upper = effect.rgbaLut[upperIndex];
    return glm::mix(
        glm::vec4{lower[0], lower[1], lower[2], lower[3]},
        glm::vec4{upper[0], upper[1], upper[2], upper[3]},
        fraction);
}

glm::vec3 ApplyTimingColourise(
    glm::vec3 baseColor,
    const OfflinePointLayer& layer,
    std::size_t pointIndex) {
    const std::size_t effectCount = std::min<std::size_t>(
        layer.timingColourise.effectCount,
        layer.timingColourise.effects.size());
    for (std::size_t effectIndex = 0; effectIndex < effectCount; ++effectIndex) {
        const auto& effect = layer.timingColourise.effects[effectIndex];
        const auto mask = ResolveTimingColouriseMask(
            layer,
            effect,
            pointIndex);
        if (!mask.has_value()) {
            continue;
        }
        if (effect.output ==
            invisible_places::renderer::pointcloud::
                TimingColouriseOutput::Emissive) {
            const float level = std::isfinite(effect.emissiveLevel)
                                    ? effect.emissiveLevel
                                    : 0.0F;
            if (level < 0.0F) {
                baseColor *= std::clamp(
                    1.0F + level * mask->edgeMask,
                    0.0F,
                    1.0F);
            }
            continue;
        }
        if (effect.output !=
            invisible_places::renderer::pointcloud::
                TimingColouriseOutput::Colourise) {
            continue;
        }
        const glm::vec4 lut = SampleTimingColouriseLut(
            effect,
            mask->normalizedValue);
        const float amount = Clamp01(lut.a) * mask->edgeMask;
        if (amount > 1.0e-5F) {
            baseColor = glm::mix(
                baseColor,
                glm::clamp(glm::vec3{lut}, glm::vec3{0.0F}, glm::vec3{1.0F}),
                amount);
        }
    }
    return baseColor;
}

float ResolveTimingColouriseEmissionAdd(
    const OfflinePointLayer& layer,
    std::size_t pointIndex) {
    const std::size_t effectCount = std::min<std::size_t>(
        layer.timingColourise.effectCount,
        layer.timingColourise.effects.size());
    float emissionAdd = 0.0F;
    for (std::size_t effectIndex = 0; effectIndex < effectCount; ++effectIndex) {
        const auto& effect = layer.timingColourise.effects[effectIndex];
        if (effect.output !=
            invisible_places::renderer::pointcloud::
                TimingColouriseOutput::Emissive) {
            continue;
        }
        const auto mask = ResolveTimingColouriseMask(
            layer,
            effect,
            pointIndex);
        if (!mask.has_value()) {
            continue;
        }
        const float level = std::isfinite(effect.emissiveLevel)
                                ? effect.emissiveLevel
                                : 0.0F;
        if (level > 0.0F) {
            emissionAdd += level * mask->edgeMask;
        }
    }
    return emissionAdd;
}

glm::vec3 ResolvePointColor(
    const OfflinePointLayer& layer,
    std::size_t pointIndex) {
    if (layer.cloud == nullptr) {
        return ApplyColorize({1.0F, 1.0F, 1.0F}, layer.style);
    }

    const auto& cloud = *layer.cloud;
    glm::vec3 baseColor{
        layer.style.solidColor[0],
        layer.style.solidColor[1],
        layer.style.solidColor[2],
    };
    if (layer.style.colorMode == invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb &&
        layer.hasSourceRgb &&
        pointIndex < cloud.packedColors.size()) {
        baseColor = SourceRgb(cloud.packedColors[pointIndex]);
    } else if (layer.style.colorMode ==
               invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap) {
        const auto color = invisible_places::renderer::pointcloud::SampleColormap(
            layer.style.colormap,
            EvaluateBindingOrDefault(
                cloud,
                layer.style.colormapPosition,
                pointIndex,
                invisible_places::renderer::pointcloud::kInactiveColormapPositionDefault));
        const auto customColor = layer.style.colormap ==
                                         invisible_places::renderer::pointcloud::PointCloudColormapId::CustomGradient
                                     ? invisible_places::renderer::pointcloud::SampleGradient(
                                           layer.style.gradientStartColor,
                                           layer.style.gradientEndColor,
                                           EvaluateBindingOrDefault(
                                               cloud,
                                               layer.style.colormapPosition,
                                               pointIndex,
                                               invisible_places::renderer::pointcloud::kInactiveColormapPositionDefault))
                                     : color;
        baseColor = {customColor[0], customColor[1], customColor[2]};
    }

    return ApplyTimingColourise(
        ApplyColorize(baseColor, layer.style),
        layer,
        pointIndex);
}

struct OfflinePointSample {
    glm::vec3 worldCenter{0.0F, 0.0F, 0.0F};
    std::uint32_t pointIndex = 0;
    float pixelCenterX = 0.0F;
    float pixelCenterY = 0.0F;
    float viewDepth = 0.0F;
    float pointSize = 1.0F;
    float surfelDiameter = 0.005F;
    float surfelAspect = 1.0F;
    float opacity = 1.0F;
    float emissive = 0.0F;
    float depthFade = 0.0F;
    glm::vec3 color{1.0F, 1.0F, 1.0F};
    bool worldSurfels = false;
    bool hasNormal = false;
    bool hasPreferredTangent = false;
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
    glm::vec3 tangent{1.0F, 0.0F, 0.0F};
    glm::vec3 bitangent{0.0F, 1.0F, 0.0F};
};

float SmoothStep(float edge0, float edge1, float value) {
    const float width = edge1 - edge0;
    if (std::abs(width) <= 1.0e-6F) {
        return value < edge0 ? 0.0F : 1.0F;
    }

    const float t = std::clamp((value - edge0) / width, 0.0F, 1.0F);
    return t * t * (3.0F - (2.0F * t));
}

float PointFalloff(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    float normalizedRadius,
    float normalizedRadiusSquared) {
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;

    if (normalizedRadiusSquared > 1.0F) {
        return 0.0F;
    }

    auto profile = style.falloffProfile;

    switch (profile) {
        case PointCloudFalloffProfile::HardDisc:
            return 1.0F;
        case PointCloudFalloffProfile::Gaussian:
            return std::exp(-normalizedRadiusSquared * std::max(0.001F, style.gaussianSharpness));
        case PointCloudFalloffProfile::Rim:
            return std::pow(
                std::max(0.0F, 1.0F - normalizedRadius),
                std::max(0.001F, style.featherPower));
        case PointCloudFalloffProfile::SoftDisc:
            return SmoothStep(1.0F, std::clamp(style.innerRadius, 0.0F, 0.99F), normalizedRadius);
    }

    return 1.0F;
}

float PointCloudDiscCoverage(float normalizedRadius, float footprintDiameterPixels) {
    if (normalizedRadius >= 1.0F) {
        return 0.0F;
    }

    const float safeDiameter = std::max(1.0F, footprintDiameterPixels);
    const float coreRadius = std::clamp(
        (safeDiameter - kPointCloudAntialiasFeatherPixels) / safeDiameter,
        0.0F,
        1.0F);
    return 1.0F - SmoothStep(coreRadius, 1.0F, normalizedRadius);
}

float ResolveDepthFadeAlpha(
    const OfflinePointSample& sample,
    const invisible_places::camera::CameraState& cameraState,
    float viewDepth) {
    const float depthNorm = std::clamp(
        (viewDepth - cameraState.nearPlane) /
            std::max(1.0e-5F, cameraState.farPlane - cameraState.nearPlane),
        0.0F,
        1.0F);
    return std::lerp(
        1.0F,
        1.0F - depthNorm,
        std::clamp(sample.depthFade, 0.0F, 1.0F));
}

float ResolveDepthOfFieldBlurPixels(
    const invisible_places::camera::CameraState& cameraState,
    float viewDepth) {
    if (!cameraState.hasDepthOfField) {
        return 0.0F;
    }

    const float focusDistance = std::max(0.001F, cameraState.focusDistance);
    const float apertureFStops = std::max(0.1F, cameraState.apertureFStops);
    const float maxBlurPixels = std::max(0.0F, cameraState.depthOfFieldMaxBlurPixels);
    const float distanceFromFocus =
        std::abs(viewDepth - focusDistance) /
        std::max(std::max(viewDepth, focusDistance), 0.001F);
    return std::clamp(
        distanceFromFocus * (8.0F / apertureFStops) * maxBlurPixels,
        0.0F,
        maxBlurPixels);
}

float ScreenPixelWorldSpan(
    float viewDepth,
    float pixels,
    float projectionScaleY,
    float viewportHeight) {
    return std::max(0.0F, pixels) *
           2.0F *
           std::max(0.001F, viewDepth) /
           (std::max(std::abs(projectionScaleY), 1.0e-5F) * std::max(1.0F, viewportHeight));
}

float AlphaClampMax(const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    return style.solidCenters ? 1.0F : 0.995F;
}

bool PointStylisationActive(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    return style.stylisationMode !=
               invisible_places::renderer::pointcloud::PointCloudStylisationMode::Off &&
           style.stylisationStrength > 1.0e-5F;
}

float PointHash01(std::uint32_t value) {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    value ^= value >> 16U;
    return static_cast<float>(value & 0x00ffffffU) / 16777215.0F;
}

float PointCoordNoise(glm::vec2 coord, std::uint32_t pointIndex) {
    const auto shifted = glm::clamp(coord, glm::vec2{-2.0F}, glm::vec2{2.0F}) + glm::vec2{2.0F};
    const auto cellX = static_cast<std::uint32_t>(std::floor(shifted.x * 31.0F));
    const auto cellY = static_cast<std::uint32_t>(std::floor(shifted.y * 31.0F));
    return PointHash01((cellX * 1973U) ^ (cellY * 9277U) ^ (pointIndex * 26699U));
}

float Fract(float value) {
    return value - std::floor(value);
}

float SurfaceHash13(const glm::vec3& value) {
    return Fract(std::sin(glm::dot(value, glm::vec3{127.1F, 311.7F, 74.7F})) * 43758.5453123F);
}

float SurfaceValueNoise(const glm::vec3& value) {
    const glm::vec3 cell = glm::floor(value);
    const glm::vec3 local = value - cell;
    const glm::vec3 blend = local * local * (glm::vec3{3.0F} - (2.0F * local));
    const float c000 = SurfaceHash13(cell + glm::vec3{0.0F, 0.0F, 0.0F});
    const float c100 = SurfaceHash13(cell + glm::vec3{1.0F, 0.0F, 0.0F});
    const float c010 = SurfaceHash13(cell + glm::vec3{0.0F, 1.0F, 0.0F});
    const float c110 = SurfaceHash13(cell + glm::vec3{1.0F, 1.0F, 0.0F});
    const float c001 = SurfaceHash13(cell + glm::vec3{0.0F, 0.0F, 1.0F});
    const float c101 = SurfaceHash13(cell + glm::vec3{1.0F, 0.0F, 1.0F});
    const float c011 = SurfaceHash13(cell + glm::vec3{0.0F, 1.0F, 1.0F});
    const float c111 = SurfaceHash13(cell + glm::vec3{1.0F, 1.0F, 1.0F});
    const float x00 = std::lerp(c000, c100, blend.x);
    const float x10 = std::lerp(c010, c110, blend.x);
    const float x01 = std::lerp(c001, c101, blend.x);
    const float x11 = std::lerp(c011, c111, blend.x);
    const float y0 = std::lerp(x00, x10, blend.y);
    const float y1 = std::lerp(x01, x11, blend.y);
    return std::lerp(y0, y1, blend.z);
}

float SurfaceFbm(glm::vec3 value) {
    float sum = 0.0F;
    float amplitude = 0.5F;
    float normalizer = 0.0F;
    for (int octave = 0; octave < 3; ++octave) {
        sum += SurfaceValueNoise(value) * amplitude;
        normalizer += amplitude;
        value = value * 2.03F + glm::vec3{17.1F, 31.7F, 11.3F};
        amplitude *= 0.5F;
    }
    return sum / std::max(0.0001F, normalizer);
}

glm::vec3 SurfaceMotionNoiseVector(const glm::vec3& position, float time) {
    return (glm::vec3{
                SurfaceFbm(position + glm::vec3{13.1F, 0.0F, time}),
                SurfaceFbm(position + glm::vec3{0.0F, 29.7F, time * 1.13F}),
                SurfaceFbm(position + glm::vec3{41.3F, 19.1F, time * 0.83F})} *
            2.0F) -
           glm::vec3{1.0F};
}

float SurfaceMotionMask(
    const OfflinePointLayer& layer,
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex) {
    if (layer.style.roughnessMotionStrength <= 1.0e-5F) {
        return 0.0F;
    }
    if (layer.roughnessMotionFullLayer) {
        return 1.0F;
    }
    if (layer.roughnessMotionFieldSlot >= cloud.scalarFields.size()) {
        return 0.0F;
    }

    const float roughness = ScalarFieldValueBySlot(cloud, layer.roughnessMotionFieldSlot, pointIndex);
    const float roughnessNormalized = std::clamp(
        (roughness - layer.roughnessMotionMinimum) * layer.roughnessMotionInvRange,
        0.0F,
        1.0F);
    float mask = SmoothStep(
        std::clamp(layer.style.roughnessMotionThreshold, 0.0F, 1.0F),
        1.0F,
        roughnessNormalized);
    if (layer.groundIdMotionFieldSlot < cloud.scalarFields.size()) {
        const float groundId = ScalarFieldValueBySlot(cloud, layer.groundIdMotionFieldSlot, pointIndex);
        const float distanceToTarget =
            std::abs(groundId - std::clamp(layer.style.roughnessMotionGroundId, 0.0F, 1.0F));
        mask *= 1.0F - SmoothStep(0.25F, 0.50F, distanceToTarget);
    }
    return mask;
}

glm::vec3 ResolveSurfaceMotionPosition(
    const OfflinePointLayer& layer,
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    glm::vec3 basePosition,
    float timeSeconds) {
    const float mask = SurfaceMotionMask(layer, cloud, pointIndex);
    if (mask <= 1.0e-5F) {
        return basePosition;
    }

    const float scale = std::max(0.01F, layer.style.roughnessMotionScale);
    const float speed = std::max(0.0F, layer.style.roughnessMotionSpeed);
    const glm::vec3 noisePosition = basePosition * scale;
    const glm::vec3 animatedNoise = SurfaceMotionNoiseVector(noisePosition, std::max(0.0F, timeSeconds) * speed);
    const glm::vec3 restNoise = SurfaceMotionNoiseVector(noisePosition, 0.0F);
    glm::vec3 offset = (animatedNoise - restNoise) * layer.style.roughnessMotionStrength * mask;
    offset.z *= 0.35F;
    return basePosition + offset;
}

float PointTemporalPigmentNoise(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    std::uint32_t pointIndex,
    std::uint32_t salt,
    float timeSeconds) {
    const float speed = std::clamp(style.pigmentAnimationSpeed, 0.0F, 4.0F);
    if (speed <= 1.0e-5F) {
        return PointHash01((pointIndex * 747796405U) ^ salt);
    }

    const float temporal = std::max(0.0F, timeSeconds) * speed * 12.0F;
    const auto frame = static_cast<std::uint32_t>(std::floor(temporal));
    const float blend = SmoothStep(0.0F, 1.0F, temporal - std::floor(temporal));
    const float current = PointHash01((pointIndex * 747796405U) ^ (frame * 2891336453U) ^ salt);
    const float next = PointHash01((pointIndex * 747796405U) ^ ((frame + 1U) * 2891336453U) ^ salt);
    return std::lerp(current, next, blend);
}

float PointWatercolorGranulationMask(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    glm::vec3 color,
    float surfaceAngleMask) {
    const float luma = glm::dot(glm::clamp(color, glm::vec3{0.0F}, glm::vec3{1.0F}), glm::vec3{0.299F, 0.587F, 0.114F});
    const float luminanceMask = 1.0F - SmoothStep(0.18F, 0.92F, luma);
    const float angleStrength = std::clamp(style.granulationAngleStrength, 0.0F, 1.0F);
    const float grazingMask =
        0.35F + (0.65F * SmoothStep(0.05F, 0.85F, std::clamp(surfaceAngleMask, 0.0F, 1.0F)));
    return std::clamp((0.25F + (0.75F * luminanceMask)) * std::lerp(1.0F, grazingMask, angleStrength), 0.0F, 1.0F);
}

glm::vec2 PointBrushCoord(
    glm::vec2 coord,
    std::uint32_t pointIndex,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    const float jitter = std::clamp(style.strokeJitter, 0.0F, 1.0F);
    const glm::vec2 jitterOffset{
        PointHash01(pointIndex * 1664525U + 1013904223U) - 0.5F,
        PointHash01(pointIndex * 22695477U + 1U) - 0.5F,
    };
    coord -= jitterOffset * (jitter * 0.38F);

    const float angle = PointHash01(pointIndex * 747796405U + 2891336453U) * 6.28318530718F;
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    return {
        (cosine * coord.x) - (sine * coord.y),
        (sine * coord.x) + (cosine * coord.y),
    };
}

float PointBrushRadius(
    glm::vec2 brushCoord,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    const float aspect = std::max(0.25F, style.brushAspect);
    const glm::vec2 ellipse{brushCoord.x / aspect, brushCoord.y * aspect};
    return glm::length(ellipse);
}

float PointStylisationCoverage(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    glm::vec2 coord,
    float radius,
    std::uint32_t pointIndex,
    float timeSeconds) {
    if (!PointStylisationActive(style)) {
        return 1.0F;
    }

    const float strength = std::clamp(style.stylisationStrength, 0.0F, 1.0F);
    const float bleed = std::clamp(style.stylisationPigmentBleed, 0.0F, 1.0F);
    const float grainAmount = std::clamp(style.stylisationPaperGrain, 0.0F, 1.0F);

    if (style.stylisationMode ==
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::NprStylisation) {
        if (style.nprPreset == invisible_places::renderer::pointcloud::PointCloudNprPreset::Watercolor) {
            const float edgeDryness = SmoothStep(0.58F, 1.0F, radius) * bleed;
            const float grain = std::lerp(
                PointCoordNoise(coord, pointIndex),
                PointTemporalPigmentNoise(style, pointIndex, 0x9e3779b9U, timeSeconds),
                std::clamp(style.pigmentVariation, 0.0F, 1.0F) * 0.45F);
            const float pigmentGap = std::clamp(0.78F + (0.44F * grain), 0.0F, 1.25F);
            return std::clamp(1.0F - (edgeDryness * pigmentGap * 0.55F * strength), 0.0F, 1.0F);
        }
        return 1.0F;
    }

    const auto brushCoord = PointBrushCoord(coord, pointIndex, style);
    const float brushRadius = PointBrushRadius(brushCoord, style);
    if (brushRadius > 1.0F) {
        return 0.0F;
    }

    const float edgeWidth = std::max(0.04F, 0.48F * bleed);
    float coverage = SmoothStep(1.0F, 1.0F - edgeWidth, brushRadius);
    const float brushGrain = std::lerp(
        PointCoordNoise(brushCoord, pointIndex + 17U),
        PointTemporalPigmentNoise(style, pointIndex, 0x85ebca6bU, timeSeconds),
        std::clamp(style.pigmentVariation, 0.0F, 1.0F) * 0.55F);
    coverage *= std::lerp(
        1.0F,
        0.68F + (0.64F * brushGrain),
        grainAmount * strength);
    coverage *= 1.0F -
                (std::clamp(style.strokeOpacityVariance, 0.0F, 1.0F) *
                 PointTemporalPigmentNoise(style, pointIndex, 0xc2b2ae35U, timeSeconds) * 0.55F * strength);
    return std::clamp(coverage, 0.0F, 1.0F);
}

glm::vec3 QuantizePointColor(
    glm::vec3 color,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    const float levels = std::max(2.0F, std::floor(style.stylisationColorLevels + 0.5F));
    color = glm::clamp(color, glm::vec3{0.0F}, glm::vec3{1.0F});
    return glm::floor((color * levels) + glm::vec3{0.5F}) / levels;
}

glm::vec3 ApplyPointHatch(
    glm::vec3 color,
    glm::vec2 coord,
    std::uint32_t pointIndex,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    float strength) {
    const float hatchStrength = std::clamp(style.hatchStrength, 0.0F, 1.0F) * strength;
    if (hatchStrength <= 1.0e-5F) {
        return color;
    }

    const float phase = PointHash01(pointIndex * 374761393U + 668265263U);
    const float stripe = std::abs(std::fmod(((coord.x + coord.y) * 8.0F) + phase, 1.0F) - 0.5F);
    const float line = 1.0F - SmoothStep(0.035F, 0.12F, stripe);
    return color * (1.0F - (line * hatchStrength * 0.55F));
}

glm::vec3 ApplyPointStylisationColor(
    glm::vec3 color,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    glm::vec2 coord,
    std::uint32_t pointIndex,
    float surfaceAngleMask,
    float timeSeconds) {
    if (!PointStylisationActive(style)) {
        return color;
    }

    const float strength = std::clamp(style.stylisationStrength, 0.0F, 1.0F);
    glm::vec2 styleCoord = coord;
    float styleRadius = glm::length(coord);
    if (style.stylisationMode ==
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::BrushParticles) {
        styleCoord = PointBrushCoord(coord, pointIndex, style);
        styleRadius = PointBrushRadius(styleCoord, style);
    }

    glm::vec3 stylised = color;
    if (style.nprPreset == invisible_places::renderer::pointcloud::PointCloudNprPreset::Cartoon) {
        stylised = QuantizePointColor(color, style);
        const float ink = SmoothStep(0.58F, 1.0F, styleRadius) *
                          std::clamp(style.stylisationInkStrength, 0.0F, 1.0F);
        stylised *= 1.0F - (ink * 0.78F);
    } else {
        const float luma = glm::dot(color, glm::vec3{0.299F, 0.587F, 0.114F});
        stylised = glm::mix(glm::vec3{luma}, color, 0.72F);
        stylised = glm::mix(stylised, glm::vec3{1.0F}, 0.08F);
        const float variation = std::clamp(style.pigmentVariation, 0.0F, 1.0F);
        const float granulationMask = PointWatercolorGranulationMask(style, color, surfaceAngleMask);
        const float temporalGrain = PointTemporalPigmentNoise(style, pointIndex, 0x27d4eb2dU, timeSeconds);
        const float grain = std::lerp(PointCoordNoise(styleCoord, pointIndex), temporalGrain, variation);
        const float pigmentShift = ((temporalGrain - 0.5F) * 2.0F) * variation * granulationMask;
        stylised *= 1.0F + (pigmentShift * 0.18F);
        stylised *= std::lerp(
            1.0F,
            0.80F + (0.42F * grain),
            std::clamp(style.stylisationPaperGrain, 0.0F, 1.0F) * granulationMask);
    }

    stylised = glm::mix(color, glm::clamp(stylised, glm::vec3{0.0F}, glm::vec3{1.0F}), strength);
    stylised = ApplyPointHatch(stylised, styleCoord, pointIndex, style, strength);
    return glm::clamp(stylised, glm::vec3{0.0F}, glm::vec3{1.0F});
}

float WeightedAlphaWeight(
    float alpha,
    float viewDepth,
    const invisible_places::camera::CameraState& cameraState) {
    const float depthNorm = std::clamp(
        (viewDepth - cameraState.nearPlane) /
            std::max(1.0e-5F, cameraState.farPlane - cameraState.nearPlane),
        0.0F,
        1.0F);
    const float opacityBase = std::min(1.0F, alpha * 8.0F) + 0.01F;
    const float opacityWeight = opacityBase * opacityBase * opacityBase;
    const float frontBase = 1.0F - depthNorm;
    const float frontSquared = frontBase * frontBase;
    const float frontWeight = frontSquared * frontSquared;
    return std::clamp((opacityWeight * 0.5F) + (opacityWeight * frontWeight * 128.0F), 1.0e-3F, 256.0F);
}

glm::vec3 CameraRight(const invisible_places::camera::OrbitCameraMatrices& matrices) {
    return glm::normalize(glm::vec3{matrices.view[0][0], matrices.view[1][0], matrices.view[2][0]});
}

glm::vec3 CameraUp(const invisible_places::camera::OrbitCameraMatrices& matrices) {
    return glm::normalize(glm::vec3{matrices.view[0][1], matrices.view[1][1], matrices.view[2][1]});
}

bool IsFinite(glm::vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool ProjectWorldToPixel(
    const glm::vec3& worldPosition,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const ExrImage& image,
    float* pixelX,
    float* pixelY) {
    const glm::vec4 clip = matrices.viewProjection * glm::vec4{worldPosition, 1.0F};
    if (clip.w <= 1.0e-6F) {
        return false;
    }

    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    if (!IsFinite(ndc)) {
        return false;
    }

    if (pixelX != nullptr) {
        *pixelX = (ndc.x * 0.5F + 0.5F) * static_cast<float>(image.width);
    }
    if (pixelY != nullptr) {
        *pixelY = (ndc.y * 0.5F + 0.5F) * static_cast<float>(image.height);
    }
    return true;
}

glm::vec3 PixelRayDirection(
    std::uint32_t x,
    std::uint32_t y,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const ExrImage& image,
    const glm::mat4& inverseViewProjection) {
    const float ndcX =
        ((static_cast<float>(x) + 0.5F) / static_cast<float>(std::max<std::uint32_t>(1U, image.width))) * 2.0F - 1.0F;
    const float ndcY =
        ((static_cast<float>(y) + 0.5F) / static_cast<float>(std::max<std::uint32_t>(1U, image.height))) * 2.0F - 1.0F;
    glm::vec4 farWorld = inverseViewProjection * glm::vec4{ndcX, ndcY, 1.0F, 1.0F};
    if (std::abs(farWorld.w) > 1.0e-6F) {
        farWorld /= farWorld.w;
    }
    return glm::normalize(glm::vec3{farWorld} - matrices.position);
}

void ResolveSurfelBasis(
    OfflinePointSample* sample,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    bool forceCameraFacing) {
    if (sample == nullptr) {
        return;
    }

    const glm::vec3 cameraRight = CameraRight(matrices);
    const glm::vec3 cameraUp = CameraUp(matrices);
    if (forceCameraFacing || !sample->hasNormal || glm::dot(sample->normal, sample->normal) <= 1.0e-8F) {
        sample->normal = matrices.position - sample->worldCenter;
        if (glm::dot(sample->normal, sample->normal) <= 1.0e-8F) {
            sample->normal = glm::normalize(glm::cross(cameraRight, cameraUp));
        } else {
            sample->normal = glm::normalize(sample->normal);
        }
        if (sample->hasPreferredTangent) {
            sample->tangent -= sample->normal * glm::dot(sample->tangent, sample->normal);
            if (glm::dot(sample->tangent, sample->tangent) > 1.0e-8F) {
                sample->tangent = glm::normalize(sample->tangent);
                sample->bitangent = glm::normalize(glm::cross(sample->normal, sample->tangent));
                return;
            }
            sample->hasPreferredTangent = false;
        }
        sample->tangent = cameraRight;
        sample->bitangent = cameraUp;
        return;
    }

    sample->normal = glm::normalize(sample->normal);
    if (sample->hasPreferredTangent) {
        sample->tangent -= sample->normal * glm::dot(sample->tangent, sample->normal);
        if (glm::dot(sample->tangent, sample->tangent) > 1.0e-8F) {
            sample->tangent = glm::normalize(sample->tangent);
            sample->bitangent = glm::normalize(glm::cross(sample->normal, sample->tangent));
            return;
        }
        sample->hasPreferredTangent = false;
    }
    sample->tangent = cameraRight - (sample->normal * glm::dot(cameraRight, sample->normal));
    if (glm::dot(sample->tangent, sample->tangent) <= 1.0e-8F) {
        sample->tangent = cameraUp - (sample->normal * glm::dot(cameraUp, sample->normal));
    }
    if (glm::dot(sample->tangent, sample->tangent) <= 1.0e-8F) {
        sample->tangent = std::abs(sample->normal.z) < 0.999F
                               ? glm::cross(glm::vec3{0.0F, 0.0F, 1.0F}, sample->normal)
                               : glm::cross(glm::vec3{0.0F, 1.0F, 0.0F}, sample->normal);
    }
    sample->tangent = glm::normalize(sample->tangent);
    sample->bitangent = glm::normalize(glm::cross(sample->normal, sample->tangent));
}

float PointSurfaceAngleMask(
    const OfflinePointSample& sample,
    const invisible_places::camera::OrbitCameraMatrices& matrices) {
    if (!sample.hasNormal || glm::dot(sample.normal, sample.normal) <= 1.0e-8F) {
        return 0.0F;
    }

    const glm::vec3 viewDirection = matrices.position - sample.worldCenter;
    if (glm::dot(viewDirection, viewDirection) <= 1.0e-8F) {
        return 0.0F;
    }

    return std::clamp(
        1.0F - std::abs(glm::dot(glm::normalize(sample.normal), glm::normalize(viewDirection))),
        0.0F,
        1.0F);
}

invisible_places::water::WaterSeepageRuntimeContribution ResolveOfflineSeepageContribution(
    const OfflinePointLayer& layer,
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    const glm::vec3& worldPosition,
    float timeSeconds,
    const glm::vec3& cameraPosition) {
    glm::vec3 worldNormal{0.0F, 0.0F, 1.0F};
    bool hasWorldNormal = false;
    if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
        const glm::vec3 localNormal = ToGlm(cloud.normals[pointIndex]);
        if (glm::dot(localNormal, localNormal) > 1.0e-8F) {
            worldNormal = glm::normalize(glm::transpose(glm::inverse(glm::mat3{layer.localToWorld})) * localNormal);
            hasWorldNormal = IsFinite(worldNormal) && glm::dot(worldNormal, worldNormal) > 1.0e-8F;
        }
    }
    if (!hasWorldNormal) {
        worldNormal = {0.0F, 0.0F, 1.0F};
    }
    const invisible_places::io::Float3 position{
        worldPosition.x,
        worldPosition.y,
        worldPosition.z,
    };
    const invisible_places::io::Float3 normal{
        worldNormal.x,
        worldNormal.y,
        worldNormal.z,
    };
    const invisible_places::io::Float3 seepageNormal =
        hasWorldNormal ? normal : invisible_places::io::Float3{};

    auto result = invisible_places::water::EvaluateWaterSeepageGridContribution(
        layer.seepageGrid,
        position,
        seepageNormal,
        timeSeconds,
        invisible_places::water::WaterSeepageViewContext{
            .cameraPosition = {
                cameraPosition.x,
                cameraPosition.y,
                cameraPosition.z,
            },
            .hasCameraPosition = true,
        });
    result.opacityMultiply = std::max(0.0F, result.opacityMultiply);
    result.pointSizeMultiply = std::max(0.0F, result.pointSizeMultiply);
    result.colourMix = Clamp01(result.colourMix);
    return result;
}

bool BuildOfflinePointSample(
    const OfflinePointLayer& layer,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const invisible_places::camera::CameraState& cameraState,
    std::size_t pointIndex,
    const ExrImage& image,
    OfflinePointSample* sample,
    bool resolveMaterial,
    float stylisationTimeSeconds) {
    if (layer.cloud == nullptr || sample == nullptr) {
        return false;
    }

    const auto& cloud = *layer.cloud;
    const auto densityCompensation =
        invisible_places::renderer::pointcloud::SanitizePointCloudDensityCompensation(
            layer.densityCompensation);
    const bool waterTrails = HasWaterTrailFields(layer);
    float waterTrailPhase = 0.0F;
    float waterTrailVisibility = 1.0F;
    invisible_places::renderer::pointcloud::WaterFlowActivityScales waterFlowActivity;
    if (waterTrails) {
        waterFlowActivity = invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales(
            layer.style.waterFlowActivity,
            ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex));
        waterTrailPhase = WaterTrailTravelPhase(
            cloud,
            layer.style,
            pointIndex,
            stylisationTimeSeconds);
    }
    const bool waterParticles = HasWaterParticleFields(layer);
    float waterParticleRole = 0.0F;
    if (waterParticles) {
        waterParticleRole = ScalarFieldValueBySlot(cloud, kWaterParticleRoleFieldSlot, pointIndex);
        if (layer.style.waterPathView) {
            if (!((waterParticleRole >= 0.5F && waterParticleRole < 1.5F) ||
                  (waterParticleRole >= 1.5F && waterParticleRole < 2.5F) ||
                  (waterParticleRole >= 2.5F && waterParticleRole < 3.5F))) {
                return false;
            }
        } else if (waterParticleRole < 0.5F || waterParticleRole >= 1.5F) {
            return false;
        }
    }

    const auto& point = cloud.positions[pointIndex];
    glm::vec3 localPoint{point.x, point.y, point.z};
    if (waterTrails) {
        localPoint = ResolveWaterTrailPosition(
            cloud,
            layer.style,
            pointIndex,
            stylisationTimeSeconds,
            localPoint);
    }
    if (waterParticles && !layer.style.waterPathView && waterParticleRole >= 0.5F && waterParticleRole < 1.5F) {
        localPoint = ResolveWaterParticlePosition(cloud, pointIndex, stylisationTimeSeconds, localPoint);
    }
    localPoint = ResolveSurfaceMotionPosition(layer, cloud, pointIndex, localPoint, stylisationTimeSeconds);
    const glm::vec4 worldPosition =
        layer.localToWorld * glm::vec4{localPoint, 1.0F};
    if (std::abs(worldPosition.w) <= 1.0e-6F) {
        return false;
    }

    const glm::vec4 normalizedWorld = worldPosition / worldPosition.w;
    if (waterTrails) {
        waterTrailVisibility = WaterTrailVisibility(
            cloud,
            layer.style,
            pointIndex,
            stylisationTimeSeconds);
        if (waterTrailVisibility <= 0.0F) {
            return false;
        }
    }
    const glm::vec4 viewPosition = matrices.view * normalizedWorld;
    const float viewDepth = -viewPosition.z;
    if (viewDepth <= cameraState.nearPlane || viewDepth >= cameraState.farPlane) {
        return false;
    }

    const glm::vec4 clip = matrices.viewProjection * normalizedWorld;
    if (clip.w <= 1.0e-6F) {
        return false;
    }

    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    const bool worldSurfels =
        layer.style.geometryMode !=
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::ScreenSprites;
    if (!worldSurfels &&
        (ndc.x < -1.0F || ndc.x > 1.0F || ndc.y < -1.0F || ndc.y > 1.0F ||
         ndc.z < -1.0F || ndc.z > 1.0F)) {
        return false;
    }

    sample->pixelCenterX = (ndc.x * 0.5F + 0.5F) * static_cast<float>(image.width);
    sample->pixelCenterY = (ndc.y * 0.5F + 0.5F) * static_cast<float>(image.height);
    sample->worldCenter = glm::vec3{normalizedWorld};
    sample->pointIndex = static_cast<std::uint32_t>(std::min<std::size_t>(
        pointIndex,
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    sample->viewDepth = viewDepth;
    const auto seepage = ResolveOfflineSeepageContribution(
        layer,
        cloud,
        pointIndex,
        sample->worldCenter,
        stylisationTimeSeconds,
        matrices.position);
    glm::vec3 rainImpactNormal{0.0F, 0.0F, 1.0F};
    if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
        const auto localNormal = ToGlm(cloud.normals[pointIndex]);
        if (glm::dot(localNormal, localNormal) > 1.0e-8F) {
            rainImpactNormal = glm::normalize(
                glm::transpose(glm::inverse(glm::mat3{layer.localToWorld})) * localNormal);
        }
    }
    // Decoupled from the layer's collision role: every enabled model shades
    // this point inside its own height band, matching the viewport shader.
    const auto rainImpact =
        layer.rainImpactGrid != nullptr && layer.rainEffectMask != 0U
            ? invisible_places::water::EvaluateRainImpact(
                  *layer.rainImpactGrid,
                  layer.rainEffectMask,
                  FromGlm(sample->worldCenter),
                  FromGlm(rainImpactNormal),
                  stylisationTimeSeconds,
                  SandRainWaterMask(layer.style, sample->worldCenter.z),
                  layer.rainRingsBand,
                  layer.rainWetnessBand,
                  layer.rainDropletsBand)
            : invisible_places::water::RainImpactEffect{};
    const float waterParticleSizeScale =
        waterParticles ? WaterParticleSizeScale(cloud, pointIndex, stylisationTimeSeconds) : 1.0F;
    const float baseSurfelDiameter = waterTrails
                                         ? WaterTrailWidth(
                                               cloud,
                                               pointIndex,
                                               layer.style)
                                         : std::max(
                                               0.0F,
                                               EvaluateBindingOrDefault(
                                                   cloud,
                                                   layer.style.surfelDiameter,
                                                   pointIndex,
                                                   invisible_places::renderer::pointcloud::
                                                       kInactiveSurfelDiameterDefault));
    const float authoredSurfelDiameter =
        (baseSurfelDiameter *
             waterParticleSizeScale *
             seepage.pointSizeMultiply *
             rainImpact.sizeScale) +
        seepage.pointSizeAdd;
    const float authoredPointSize =
        (std::max(
             0.0F,
             EvaluateBindingOrDefault(
                 cloud,
                 layer.style.pointSize,
                 pointIndex,
                 invisible_places::renderer::pointcloud::kInactivePointSizeDefault)) *
             waterParticleSizeScale *
             seepage.pointSizeMultiply *
             rainImpact.sizeScale) +
        seepage.pointSizeAdd;
    const float depthOfFieldBlurPixels = ResolveDepthOfFieldBlurPixels(cameraState, viewDepth);
    const bool worldSizedScreenSprites =
        invisible_places::renderer::pointcloud::PointCloudStyleUsesWorldSizedScreenSprites(layer.style);
    const float kernelPixels =
        invisible_places::renderer::pointcloud::ResolvePointCloudDensityAdjustedFootprint(
            worldSizedScreenSprites
                ? invisible_places::renderer::pointcloud::WorldDiameterToScreenPointSizePixels(
                      authoredSurfelDiameter,
                      viewDepth,
                      matrices.projection[1][1],
                      static_cast<float>(image.height))
                : authoredPointSize,
            0.0F,
            0.0F,
            densityCompensation);
    sample->pointSize = std::clamp(
        invisible_places::renderer::pointcloud::ResolvePointCloudDensityAdjustedFootprint(
            worldSizedScreenSprites
                ? invisible_places::renderer::pointcloud::WorldDiameterToScreenPointSizePixels(
                      authoredSurfelDiameter,
                      viewDepth,
                      matrices.projection[1][1],
                      static_cast<float>(image.height))
                : authoredPointSize,
            kPointCloudAntialiasFeatherPixels,
            depthOfFieldBlurPixels,
            densityCompensation),
        1.0F,
        64.0F);
    // Floor-only energy conservation, mirroring the GPU vertex shaders: a
    // point clamped up to the 1 px minimum must not gain integrated
    // brightness, while everything above the floor keeps its authored
    // opacity. (The GPU additionally normalises the falloff kernel to the
    // authored footprint inside the padded sprite; the CPU rasteriser is
    // analytic, so its padded disc introduces no sampling inflation.)
    const float paddedKernelPixels =
        kernelPixels + kPointCloudAntialiasFeatherPixels;
    const float kernelEnergyRatio =
        !worldSurfels
            ? std::clamp(paddedKernelPixels / 1.0F, 0.0F, 1.0F)
            : 1.0F;
    const float kernelEnergyScale = kernelEnergyRatio * kernelEnergyRatio;
    sample->worldSurfels = worldSurfels;
    sample->surfelDiameter =
        invisible_places::renderer::pointcloud::ClampPointCloudResolvedSurfelDiameter(
            invisible_places::renderer::pointcloud::ResolvePointCloudDensityAdjustedFootprint(
                authoredSurfelDiameter,
                ScreenPixelWorldSpan(
                    viewDepth,
                    kPointCloudAntialiasFeatherPixels,
                    matrices.projection[1][1],
                    static_cast<float>(image.height)),
                2.0F * ScreenPixelWorldSpan(
                           viewDepth,
                           depthOfFieldBlurPixels,
                           matrices.projection[1][1],
                           static_cast<float>(image.height)),
                densityCompensation),
            ScreenPixelWorldSpan(
                viewDepth,
                64.0F,
                matrices.projection[1][1],
                static_cast<float>(image.height)));
    sample->surfelAspect = layer.style.flowAnimation
                                ? std::clamp(layer.style.waterStreakAspect, 1.0F, 32.0F)
                                : 1.0F;
    if (waterTrails) {
        const float trailStreakLength = std::max(
            sample->surfelDiameter,
            WaterTrailStreakLength(cloud, pointIndex, layer.style));
        sample->surfelAspect = std::clamp(
            trailStreakLength / std::max(sample->surfelDiameter, 0.0001F),
            1.0F,
            64.0F);
    }
    sample->opacity = Clamp01(
        (EvaluateBindingOrDefault(
             cloud,
             layer.style.opacity,
             pointIndex,
             invisible_places::renderer::pointcloud::kInactiveOpacityDefault) *
             seepage.opacityMultiply) +
        seepage.opacityAdd +
        rainImpact.opacity);
    sample->opacity *= kernelEnergyScale;
    if (waterTrails) {
        sample->opacity *= waterTrailVisibility * waterFlowActivity.appearance;
    }
    if (waterParticles) {
        sample->opacity *= WaterParticleFade(cloud, pointIndex, stylisationTimeSeconds);
    }
    sample->depthFade = Clamp01(
        EvaluateBindingOrDefault(
            cloud,
            layer.style.depthFade,
            pointIndex,
            invisible_places::renderer::pointcloud::kInactiveDepthFadeDefault));
    if (resolveMaterial) {
        sample->emissive = std::max(
            0.0F,
            EvaluateBindingOrDefault(
                cloud,
                layer.style.emissiveStrength,
                pointIndex,
                invisible_places::renderer::pointcloud::kInactiveEmissionDefault));
        if (waterParticles) {
            sample->emissive *= WaterParticleFade(cloud, pointIndex, stylisationTimeSeconds);
        }
        sample->emissive += seepage.emissionAdd;
        sample->emissive += rainImpact.emission;
        if (waterTrails) {
            sample->emissive *= waterTrailVisibility * waterFlowActivity.appearance;
        }
        sample->emissive += ResolveTimingColouriseEmissionAdd(
            layer,
            pointIndex);
        sample->color = ResolvePointColor(layer, pointIndex);
        sample->color = glm::mix(
            sample->color,
            seepage.colour,
            Clamp01(seepage.colourMix));
        // Mirrors ApplyRainImpactColour: the shared wet tint first, then the
        // brighter Droplets tint on top.
        sample->color = glm::mix(
            sample->color,
            glm::vec3{0.24F, 0.48F, 0.62F},
            std::clamp(rainImpact.colourBlend, 0.0F, 0.72F));
        sample->color = glm::mix(
            sample->color,
            glm::vec3{0.54F, 0.80F, 0.82F},
            std::clamp(rainImpact.dropletBlend, 0.0F, 0.72F));
    }
    sample->hasNormal = false;
    sample->hasNormal = cloud.hasNormals && pointIndex < cloud.normals.size();
    if (sample->hasNormal) {
        const glm::vec3 localNormal = ToGlm(cloud.normals[pointIndex]);
        sample->hasNormal = glm::dot(localNormal, localNormal) > 1.0e-8F;
        if (sample->hasNormal) {
            sample->normal =
                glm::normalize(glm::transpose(glm::inverse(glm::mat3{layer.localToWorld})) * localNormal);
            sample->hasNormal = IsFinite(sample->normal) && glm::dot(sample->normal, sample->normal) > 1.0e-8F;
        }
    }
    sample->hasPreferredTangent = false;
    if (waterTrails) {
        const glm::vec3 localTangent = WaterTrailRouteTangent(
            cloud,
            pointIndex,
            waterTrailPhase);
        if (glm::dot(localTangent, localTangent) > 1.0e-8F) {
            const glm::vec3 worldTangent = glm::mat3{layer.localToWorld} * localTangent;
            if (IsFinite(worldTangent) && glm::dot(worldTangent, worldTangent) > 1.0e-8F) {
                sample->tangent = glm::normalize(worldTangent);
                sample->hasPreferredTangent = true;
            }
        }
    } else if (waterParticles && layer.style.waterStreakAspect > 1.0001F) {
        const glm::vec3 localTangent = ResolveWaterParticleTangent(cloud, pointIndex, stylisationTimeSeconds);
        if (glm::dot(localTangent, localTangent) > 1.0e-8F) {
            const glm::vec3 worldTangent = glm::mat3{layer.localToWorld} * localTangent;
            if (IsFinite(worldTangent) && glm::dot(worldTangent, worldTangent) > 1.0e-8F) {
                sample->tangent = glm::normalize(worldTangent);
                sample->hasPreferredTangent = true;
            }
        }
    }
    if (sample->worldSurfels) {
        if (sample->surfelDiameter <= 1.0e-6F) {
            return false;
        }
        ResolveSurfelBasis(
            sample,
            matrices,
            layer.style.geometryMode ==
                invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites);
    }
    return sample->opacity > 0.0F;
}

template <typename PixelCallback>
void VisitCoveredPixels(
    const OfflinePointSample& sample,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const ExrImage& image,
    const OfflineRenderTile& tile,
    std::uint32_t tileWidth,
    float stylisationTimeSeconds,
    PixelCallback callback) {
    if (sample.worldSurfels) {
        const float radiusWorld = sample.surfelDiameter * 0.5F;
        const float tangentRadiusWorld = radiusWorld * std::max(1.0F, sample.surfelAspect);
        const float bitangentRadiusWorld = radiusWorld;
        const std::array<glm::vec3, 4> quadCorners = {
            sample.worldCenter - (sample.tangent * tangentRadiusWorld) - (sample.bitangent * bitangentRadiusWorld),
            sample.worldCenter + (sample.tangent * tangentRadiusWorld) - (sample.bitangent * bitangentRadiusWorld),
            sample.worldCenter + (sample.tangent * tangentRadiusWorld) + (sample.bitangent * bitangentRadiusWorld),
            sample.worldCenter - (sample.tangent * tangentRadiusWorld) + (sample.bitangent * bitangentRadiusWorld),
        };

        float minPixelX = std::numeric_limits<float>::max();
        float minPixelY = std::numeric_limits<float>::max();
        float maxPixelX = std::numeric_limits<float>::lowest();
        float maxPixelY = std::numeric_limits<float>::lowest();
        bool projectedAnyCorner = false;
        for (const auto& corner : quadCorners) {
            float pixelX = 0.0F;
            float pixelY = 0.0F;
            if (!ProjectWorldToPixel(corner, matrices, image, &pixelX, &pixelY)) {
                continue;
            }
            projectedAnyCorner = true;
            minPixelX = std::min(minPixelX, pixelX);
            minPixelY = std::min(minPixelY, pixelY);
            maxPixelX = std::max(maxPixelX, pixelX);
            maxPixelY = std::max(maxPixelY, pixelY);
        }
        if (!projectedAnyCorner) {
            return;
        }

        float bitangentPixelX = sample.pixelCenterX;
        float bitangentPixelY = sample.pixelCenterY;
        const bool projectedBitangent = ProjectWorldToPixel(
            sample.worldCenter + sample.bitangent * bitangentRadiusWorld,
            matrices,
            image,
            &bitangentPixelX,
            &bitangentPixelY);
        const float footprintDiameterPixels = projectedBitangent
                                                  ? 2.0F * glm::length(glm::vec2{
                                                        bitangentPixelX - sample.pixelCenterX,
                                                        bitangentPixelY - sample.pixelCenterY})
                                                  : std::max(1.0F, std::min(
                                                        maxPixelX - minPixelX,
                                                        maxPixelY - minPixelY));

        const int minX = std::max<int>(static_cast<int>(tile.x0), static_cast<int>(std::floor(minPixelX)) - 1);
        const int maxX = std::min<int>(static_cast<int>(tile.x1) - 1, static_cast<int>(std::ceil(maxPixelX)) + 1);
        const int minY = std::max<int>(static_cast<int>(tile.y0), static_cast<int>(std::floor(minPixelY)) - 1);
        const int maxY = std::min<int>(static_cast<int>(tile.y1) - 1, static_cast<int>(std::ceil(maxPixelY)) + 1);
        if (minX > maxX || minY > maxY) {
            return;
        }

        const glm::mat4 inverseViewProjection = glm::inverse(matrices.viewProjection);
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const glm::vec3 rayDirection = PixelRayDirection(
                    static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y),
                    matrices,
                    image,
                    inverseViewProjection);
                const float denominator = glm::dot(sample.normal, rayDirection);
                if (std::abs(denominator) <= 1.0e-6F) {
                    continue;
                }

                const float distanceAlongRay =
                    glm::dot(sample.worldCenter - matrices.position, sample.normal) / denominator;
                if (distanceAlongRay <= 0.0F) {
                    continue;
                }

                const glm::vec3 hitPoint = matrices.position + (rayDirection * distanceAlongRay);
                const glm::vec3 localOffset = hitPoint - sample.worldCenter;
                const float u = glm::dot(localOffset, sample.tangent);
                const float v = glm::dot(localOffset, sample.bitangent);
                const float normalizedRadiusSquared =
                    ((u * u) / std::max(1.0e-8F, tangentRadiusWorld * tangentRadiusWorld)) +
                    ((v * v) / std::max(1.0e-8F, bitangentRadiusWorld * bitangentRadiusWorld));
                if (normalizedRadiusSquared > 1.0F) {
                    continue;
                }

                const float normalizedRadius = std::sqrt(normalizedRadiusSquared);
                const glm::vec2 normalizedCoord{
                    u / std::max(1.0e-8F, tangentRadiusWorld),
                    v / std::max(1.0e-8F, bitangentRadiusWorld)};
                const float falloff = PointFalloff(style, normalizedRadius, normalizedRadiusSquared);
                const float stylisationCoverage =
                    PointStylisationCoverage(
                        style,
                        normalizedCoord,
                        normalizedRadius,
                        sample.pointIndex,
                        stylisationTimeSeconds);
                const float antialiasCoverage =
                    PointCloudDiscCoverage(normalizedRadius, footprintDiameterPixels);
                if (falloff <= 1.0e-5F ||
                    stylisationCoverage <= 1.0e-5F ||
                    antialiasCoverage <= 1.0e-5F) {
                    continue;
                }

                const glm::vec4 viewPosition = matrices.view * glm::vec4{hitPoint, 1.0F};
                const float coveredViewDepth = -viewPosition.z;
                if (coveredViewDepth <= 0.0F) {
                    continue;
                }

                const auto localIndex =
                    static_cast<std::size_t>(y - static_cast<int>(tile.y0)) * static_cast<std::size_t>(tileWidth) +
                    static_cast<std::size_t>(x - static_cast<int>(tile.x0));
                callback(
                    static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y),
                    localIndex,
                    falloff * stylisationCoverage * antialiasCoverage,
                    normalizedCoord,
                    coveredViewDepth);
            }
        }
        return;
    }

    const auto radiusPixels = static_cast<int>(std::ceil(sample.pointSize * 0.5F));
    const int centerX = static_cast<int>(std::floor(sample.pixelCenterX));
    const int centerY = static_cast<int>(std::floor(sample.pixelCenterY));
    const int minX = std::max<int>(static_cast<int>(tile.x0), centerX - radiusPixels);
    const int maxX = std::min<int>(static_cast<int>(tile.x1) - 1, centerX + radiusPixels);
    const int minY = std::max<int>(static_cast<int>(tile.y0), centerY - radiusPixels);
    const int maxY = std::min<int>(static_cast<int>(tile.y1) - 1, centerY + radiusPixels);
    if (minX > maxX || minY > maxY) {
        return;
    }

    const float safeRadius = std::max(0.5F, sample.pointSize * 0.5F);
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const float dx = (static_cast<float>(x) + 0.5F) - sample.pixelCenterX;
            const float dy = (static_cast<float>(y) + 0.5F) - sample.pixelCenterY;
            const float normalizedRadiusSquared = ((dx * dx) + (dy * dy)) / (safeRadius * safeRadius);
            if (normalizedRadiusSquared > 1.0F) {
                continue;
            }

            const float normalizedRadius = std::sqrt(normalizedRadiusSquared);
            const glm::vec2 normalizedCoord{dx / safeRadius, dy / safeRadius};
            const float falloff = PointFalloff(style, normalizedRadius, normalizedRadiusSquared);
            const float stylisationCoverage =
                PointStylisationCoverage(
                    style,
                    normalizedCoord,
                    normalizedRadius,
                    sample.pointIndex,
                    stylisationTimeSeconds);
            const float antialiasCoverage =
                PointCloudDiscCoverage(normalizedRadius, sample.pointSize);
            if (falloff <= 1.0e-5F ||
                stylisationCoverage <= 1.0e-5F ||
                antialiasCoverage <= 1.0e-5F) {
                continue;
            }

            const auto localIndex =
                static_cast<std::size_t>(y - static_cast<int>(tile.y0)) * static_cast<std::size_t>(tileWidth) +
                static_cast<std::size_t>(x - static_cast<int>(tile.x0));
            callback(
                static_cast<std::uint32_t>(x),
                static_cast<std::uint32_t>(y),
                localIndex,
                falloff * stylisationCoverage * antialiasCoverage,
                normalizedCoord,
                sample.viewDepth);
        }
    }
}

}  // namespace

void InitializeExrImage(ExrImage* image, std::uint32_t width, std::uint32_t height) {
    if (image == nullptr) {
        return;
    }

    image->width = width;
    image->height = height;
    const auto pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    image->beautyR.assign(pixelCount, 0.0F);
    image->beautyG.assign(pixelCount, 0.0F);
    image->beautyB.assign(pixelCount, 0.0F);
    image->alpha.assign(pixelCount, 0.0F);
    image->depth.assign(pixelCount, std::numeric_limits<float>::infinity());
}

void AdvanceOfflineRainFrame(
    OfflineRainSimulationState* state,
    const invisible_places::water::WaterSurfaceCache& surfaceCache,
    const invisible_places::water::RainRuntimeSettings& settings,
    const invisible_places::water::WaterRainVisualSettings& visual,
    const invisible_places::camera::CameraState& cameraState,
    float timeSeconds,
    float deltaSeconds) {
    if (state == nullptr) {
        return;
    }

    const auto& centre = cameraState.hasOrbitCenter ? cameraState.orbitCenter : cameraState.target;
    invisible_places::water::RainSimulationFrame simulationFrame;
    simulationFrame.settings = settings;
    simulationFrame.visual = visual;
    simulationFrame.cameraPosition = {
        cameraState.position[0], cameraState.position[1], cameraState.position[2]};
    simulationFrame.spawnCentre = {
        centre[0],
        centre[1],
        surfaceCache.bounds.valid ? surfaceCache.bounds.maximum.z : centre[2]};
    simulationFrame.timeSeconds = timeSeconds;
    simulationFrame.deltaSeconds = deltaSeconds;
    state->diagnostics = state->simulator.Advance(simulationFrame, surfaceCache);
    state->impactGrid = settings.enabled && settings.impactEffectsEnabled
                            ? invisible_places::water::BuildRainImpactGrid(
                              state->simulator.Events(),
                              simulationFrame.cameraPosition,
                              timeSeconds,
                              invisible_places::water::RainImpactGridWorldSpan(settings),
                              settings.rockImpact,
                              settings.vegetationImpact,
                              settings.ringImpact,
                              settings.sandEffectScale,
                              settings.rockEffectScale,
                              settings.vegetationEffectScale)
                            : invisible_places::water::RainImpactGrid{};
    state->frame = {
        .settings = settings,
        .visual = visual,
        .particles = state->simulator.Particles(),
        .timeSeconds = timeSeconds,
    };
}

std::vector<OfflineRenderTile> BuildOfflineRenderTiles(
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t tileSize) {
    const auto safeTileSize = std::max<std::uint32_t>(1U, tileSize);
    std::vector<OfflineRenderTile> tiles;
    for (std::uint32_t y = 0; y < height; y += safeTileSize) {
        for (std::uint32_t x = 0; x < width; x += safeTileSize) {
            tiles.push_back(
                {.x0 = x,
                 .y0 = y,
                 .x1 = std::min(width, x + safeTileSize),
                 .y1 = std::min(height, y + safeTileSize)});
        }
    }
    return tiles;
}

template <typename PixelCallback>
void VisitOfflineRainPixels(
    const OfflineRainFrame* rainFrame,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const OfflineRenderTile& tile,
    const ExrImage& image,
    PixelCallback&& callback) {
    if (rainFrame == nullptr || !rainFrame->settings.enabled || rainFrame->particles.empty()) {
        return;
    }
    const auto intensity = invisible_places::water::RainIntensityValues(
        rainFrame->settings.intensityPreset);
    const float widthMeters = std::max(
        0.0001F,
        rainFrame->visual.widthMeters * rainFrame->settings.dropletSizeScale * intensity.width);
    const float authoredStreakLength =
        rainFrame->visual.streakLengthMeters * rainFrame->settings.dropletSizeScale * intensity.length;
    const float opacity = Clamp01(
        rainFrame->visual.opacity * rainFrame->settings.opacityScale * intensity.opacity);
    const float emission = std::max(
        0.0F,
        rainFrame->visual.emission * rainFrame->settings.emissionScale * intensity.emission);
    const glm::vec3 colour{
        rainFrame->visual.colour[0],
        rainFrame->visual.colour[1],
        rainFrame->visual.colour[2],
    };

    for (const auto& particle : rainFrame->particles) {
        if (!particle.active) {
            continue;
        }
        const glm::vec3 head = ToGlm(particle.position);
        const glm::vec3 velocity = ToGlm(particle.velocity);
        if (glm::dot(velocity, velocity) <= 1.0e-8F) {
            continue;
        }
        const float proximity = std::clamp(particle.surfaceProximity, 0.0F, 1.0F);
        const float alignment = std::clamp(
            rainFrame->settings.nearSurface.normalAlignment,
            0.0F,
            1.0F) * proximity;
        glm::vec3 direction = glm::normalize(velocity);
        glm::vec3 surfaceNormal = ToGlm(particle.surfaceNormal);
        surfaceNormal = glm::dot(surfaceNormal, surfaceNormal) > 1.0e-8F
            ? glm::normalize(surfaceNormal)
            : glm::vec3{0.0F, 0.0F, 1.0F};
        glm::vec3 viewDirection = head - matrices.position;
        viewDirection = glm::dot(viewDirection, viewDirection) > 1.0e-8F
            ? glm::normalize(viewDirection)
            : glm::vec3{0.0F, 1.0F, 0.0F};
        glm::vec3 surfaceForward =
            direction - surfaceNormal * glm::dot(direction, surfaceNormal);
        if (glm::dot(surfaceForward, surfaceForward) <= 1.0e-8F) {
            glm::vec3 surfaceSideFallback = glm::cross(surfaceNormal, viewDirection);
            if (glm::dot(surfaceSideFallback, surfaceSideFallback) <= 1.0e-8F) {
                const glm::vec3 basis = std::abs(surfaceNormal.z) < 0.90F
                    ? glm::vec3{0.0F, 0.0F, 1.0F}
                    : glm::vec3{1.0F, 0.0F, 0.0F};
                surfaceSideFallback = glm::cross(surfaceNormal, basis);
            }
            surfaceSideFallback = glm::normalize(surfaceSideFallback);
            surfaceForward = glm::normalize(glm::cross(surfaceSideFallback, surfaceNormal));
        } else {
            surfaceForward = glm::normalize(surfaceForward);
        }
        const glm::vec3 alignedDirection = glm::mix(direction, surfaceForward, alignment);
        if (glm::dot(alignedDirection, alignedDirection) > 1.0e-8F) {
            direction = glm::normalize(alignedDirection);
        }
        const auto particleShape =
            invisible_places::water::EvaluateRainParticleVisualShape(
                widthMeters,
                authoredStreakLength,
                proximity,
                rainFrame->settings.nearSurface);
        const float particleWidthMeters = particleShape.widthMeters;
        const float streakLength = particleShape.lengthMeters;
        const glm::vec3 tail = glm::mix(
            head - direction * streakLength,
            head - direction * streakLength * 0.5F,
            particleShape.ellipseBlend);
        const glm::vec3 tip = glm::mix(
            head,
            head + direction * streakLength * 0.5F,
            particleShape.ellipseBlend);
        float headX = 0.0F;
        float headY = 0.0F;
        float tailX = 0.0F;
        float tailY = 0.0F;
        if (!ProjectWorldToPixel(tip, matrices, image, &headX, &headY) ||
            !ProjectWorldToPixel(tail, matrices, image, &tailX, &tailY)) {
            continue;
        }
        const float headDepth = -(matrices.view * glm::vec4{tip, 1.0F}).z;
        const float tailDepth = -(matrices.view * glm::vec4{tail, 1.0F}).z;
        if (headDepth <= 0.0F || tailDepth <= 0.0F) {
            continue;
        }
        const float viewDepth = 0.5F * (headDepth + tailDepth);
        const float diameterPixels = std::clamp(
            invisible_places::renderer::pointcloud::WorldDiameterToScreenPointSizePixels(
                particleWidthMeters,
                viewDepth,
                matrices.projection[1][1],
                static_cast<float>(image.height)),
            std::max(0.0F, rainFrame->visual.minimumScreenPixels),
            std::max(rainFrame->visual.minimumScreenPixels, rainFrame->visual.maximumScreenPixels));
        const float radius = std::max(0.5F, diameterPixels * 0.5F);
        const glm::vec2 start{tailX, tailY};
        const glm::vec2 end{headX, headY};
        const glm::vec2 segment = end - start;
        const float segmentLengthSquared = std::max(1.0e-5F, glm::dot(segment, segment));
        const int minimumX = std::max<int>(
            static_cast<int>(tile.x0),
            static_cast<int>(std::floor(std::min(start.x, end.x) - radius - 1.0F)));
        const int maximumX = std::min<int>(
            static_cast<int>(tile.x1) - 1,
            static_cast<int>(std::ceil(std::max(start.x, end.x) + radius + 1.0F)));
        const int minimumY = std::max<int>(
            static_cast<int>(tile.y0),
            static_cast<int>(std::floor(std::min(start.y, end.y) - radius - 1.0F)));
        const int maximumY = std::min<int>(
            static_cast<int>(tile.y1) - 1,
            static_cast<int>(std::ceil(std::max(start.y, end.y) + radius + 1.0F)));
        for (int y = minimumY; y <= maximumY; ++y) {
            for (int x = minimumX; x <= maximumX; ++x) {
                const glm::vec2 pixel{static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F};
                const float along = std::clamp(glm::dot(pixel - start, segment) / segmentLengthSquared, 0.0F, 1.0F);
                const float distance = glm::length(pixel - (start + segment * along));
                if (distance > radius) {
                    continue;
                }
                const float featherStart = std::lerp(
                    0.90F,
                    0.05F,
                    std::clamp(rainFrame->visual.softness, 0.0F, 1.0F));
                const float edge = 1.0F - SmoothStep(radius * featherStart, radius, distance);
                const float endFade = SmoothStep(0.0F, 0.08F, along) *
                                      (1.0F - SmoothStep(0.88F, 1.0F, along));
                const float ellipseDistance = std::hypot(
                    distance / std::max(0.5F, radius),
                    (along - 0.5F) * 2.0F);
                const float ellipseCoverage = 1.0F - SmoothStep(
                    featherStart,
                    1.0F,
                    ellipseDistance);
                const float coverage = std::lerp(
                    edge * endFade,
                    ellipseCoverage,
                    particleShape.ellipseBlend);
                const float pixelAlpha = opacity * std::max(0.0F, particle.visibility) * coverage;
                if (pixelAlpha <= 1.0e-5F) {
                    continue;
                }
                const float depth = std::lerp(tailDepth, headDepth, along);
                const auto pixelIndex =
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width) +
                    static_cast<std::size_t>(x);
                callback(pixelIndex, static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), depth,
                         pixelAlpha, colour, emission);
            }
        }
    }
}

void CompositeOfflineRainDirect(
    const OfflineRainFrame* rainFrame,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const OfflineRenderTile& tile,
    ExrImage* image) {
    if (image == nullptr) {
        return;
    }
    VisitOfflineRainPixels(
        rainFrame,
        matrices,
        tile,
        *image,
        [&](std::size_t pixelIndex, std::uint32_t, std::uint32_t, float depth, float alpha,
            const glm::vec3& colour, float emission) {
            if (pixelIndex >= image->depth.size() || depth > image->depth[pixelIndex] + 0.02F) {
                return;
            }
            const glm::vec3 source = colour * (1.0F + emission);
            const glm::vec3 destination{
                image->beautyR[pixelIndex], image->beautyG[pixelIndex], image->beautyB[pixelIndex]};
            const float destinationAlpha = image->alpha[pixelIndex];
            image->beautyR[pixelIndex] = source.r * alpha + destination.r * (1.0F - alpha);
            image->beautyG[pixelIndex] = source.g * alpha + destination.g * (1.0F - alpha);
            image->beautyB[pixelIndex] = source.b * alpha + destination.b * (1.0F - alpha);
            image->alpha[pixelIndex] = alpha + destinationAlpha * (1.0F - alpha);
        });
}

void RenderFastBasicPointCloudTile(
    const std::vector<OfflinePointLayer>& layers,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const OfflineRenderTile& tile,
    ExrImage* image,
    OfflinePointRenderDiagnostics* diagnostics,
    float stylisationTimeSeconds) {
    if (image == nullptr) {
        return;
    }

    const auto start = std::chrono::steady_clock::now();
    for (const auto& layer : layers) {
        if (layer.cloud == nullptr || layer.cloud->positions.empty()) {
            continue;
        }
        if (diagnostics != nullptr) {
            ++diagnostics->accumulationPassLayers;
        }

        const auto& cloud = *layer.cloud;
        const bool waterTrails = HasWaterTrailFields(layer);
        const auto sourcePointCount = cloud.positions.size();
        const auto drawPointCount =
            static_cast<std::size_t>(std::min<std::uint64_t>(
                layer.drawPointCount == 0 ? static_cast<std::uint64_t>(sourcePointCount) : layer.drawPointCount,
                static_cast<std::uint64_t>(sourcePointCount)));
        if (drawPointCount == 0) {
            continue;
        }

        const bool worldSizedScreenSprites =
            invisible_places::renderer::pointcloud::PointCloudStyleUsesWorldSizedScreenSprites(layer.style);
        const float footprintScale =
            invisible_places::renderer::pointcloud::SanitizePointCloudDensityCompensation(
                layer.densityCompensation)
                .footprintScale;
        for (std::size_t sampleIndex = 0; sampleIndex < drawPointCount; ++sampleIndex) {
            const auto pointIndex =
                drawPointCount < sourcePointCount
                    ? static_cast<std::size_t>(
                          (static_cast<std::uint64_t>(sampleIndex) *
                           static_cast<std::uint64_t>(sourcePointCount)) /
                          static_cast<std::uint64_t>(drawPointCount))
                    : sampleIndex;
            if (waterTrails && ScalarFieldValueBySlot(cloud, kWaterTrailRoleFieldSlot, pointIndex) < 0.5F) {
                continue;
            }
            glm::vec3 localPosition = ToGlm(cloud.positions[pointIndex]);
            float waterTrailVisibility = 1.0F;
            invisible_places::renderer::pointcloud::WaterFlowActivityScales waterFlowActivity;
            if (waterTrails) {
                waterFlowActivity = invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales(
                    layer.style.waterFlowActivity,
                    ScalarFieldValueBySlot(cloud, kWaterTrailSeedFieldSlot, pointIndex));
                localPosition = ResolveWaterTrailPosition(
                    cloud,
                    layer.style,
                    pointIndex,
                    stylisationTimeSeconds,
                    localPosition);
            }
            const glm::vec4 worldPosition4 = layer.localToWorld * glm::vec4{localPosition, 1.0F};
            const glm::vec3 worldPosition{worldPosition4};
            if (waterTrails) {
                waterTrailVisibility = WaterTrailVisibility(
                    cloud,
                    layer.style,
                    pointIndex,
                    stylisationTimeSeconds);
                if (waterTrailVisibility <= 0.0F) {
                    continue;
                }
            }
            const glm::vec4 viewPosition = matrices.view * glm::vec4{worldPosition, 1.0F};
            const float viewDepth = -viewPosition.z;
            if (viewDepth <= 0.0F) {
                continue;
            }

            float pixelX = 0.0F;
            float pixelY = 0.0F;
            if (!ProjectWorldToPixel(worldPosition, matrices, *image, &pixelX, &pixelY)) {
                continue;
            }

            const auto seepage = ResolveOfflineSeepageContribution(
                layer,
                cloud,
                pointIndex,
                worldPosition,
                stylisationTimeSeconds,
                matrices.position);
            glm::vec3 rainNormal{0.0F, 0.0F, 1.0F};
            if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
                const auto localNormal = ToGlm(cloud.normals[pointIndex]);
                if (glm::dot(localNormal, localNormal) > 1.0e-8F) {
                    rainNormal = glm::normalize(
                        glm::transpose(glm::inverse(glm::mat3{layer.localToWorld})) * localNormal);
                }
            }
            // Decoupled from the layer's collision role: every enabled
            // model shades this point inside its own height band, matching
            // the viewport shader.
            const auto rainImpact =
                layer.rainImpactGrid != nullptr && layer.rainEffectMask != 0U
                    ? invisible_places::water::EvaluateRainImpact(
                          *layer.rainImpactGrid,
                          layer.rainEffectMask,
                          FromGlm(worldPosition),
                          FromGlm(rainNormal),
                          stylisationTimeSeconds,
                          SandRainWaterMask(layer.style, worldPosition.z),
                          layer.rainRingsBand,
                          layer.rainWetnessBand,
                          layer.rainDropletsBand)
                    : invisible_places::water::RainImpactEffect{};
            glm::vec3 color =
                glm::mix(
                    ResolvePointColor(layer, pointIndex),
                    seepage.colour,
                    Clamp01(seepage.colourMix)) *
                (1.0F + std::max(0.0F, seepage.emissionAdd));
            // Mirrors ApplyRainImpactColour: the shared wet tint first, then
            // the brighter Droplets tint on top.
            color = glm::mix(
                color,
                glm::vec3{0.24F, 0.48F, 0.62F},
                std::clamp(rainImpact.colourBlend, 0.0F, 0.72F));
            color = glm::mix(
                color,
                glm::vec3{0.54F, 0.80F, 0.82F},
                std::clamp(rainImpact.dropletBlend, 0.0F, 0.72F));
            color *= 1.0F + std::max(0.0F, rainImpact.emission);
            const float timingEmissionAdd =
                ResolveTimingColouriseEmissionAdd(layer, pointIndex);
            if (timingEmissionAdd > 0.0F) {
                color *= 1.0F +
                         0.35F * std::min(1.0F, timingEmissionAdd);
            }
            // Fast Basic intentionally ignores the authored material opacity,
            // while procedural water effects can still attenuate its opaque base.
            const float flowCoverage = waterTrails
                                           ? waterTrailVisibility * waterFlowActivity.appearance
                                           : 1.0F;
            const float opacity = Clamp01(
                (seepage.opacityMultiply + seepage.opacityAdd + rainImpact.opacity) *
                flowCoverage);
            if (opacity <= 1.0e-5F) {
                continue;
            }
            const float basePointSize =
                worldSizedScreenSprites
                    ? invisible_places::renderer::pointcloud::WorldDiameterToScreenPointSizePixels(
                          invisible_places::style::ScalarConstant(layer.style.surfelDiameter),
                          viewDepth,
                          matrices.projection[1][1],
                          static_cast<float>(image->height))
                    : invisible_places::style::ScalarConstant(layer.style.pointSize);
            const float flowWidthScale = waterTrails ? waterFlowActivity.width : 1.0F;
            const float pointSize =
                std::clamp(
                    ((basePointSize * flowWidthScale * seepage.pointSizeMultiply * rainImpact.sizeScale) +
                     seepage.pointSizeAdd) *
                        footprintScale,
                    1.0F,
                    64.0F);
            if (worldSizedScreenSprites || pointSize > 1.0F) {
                const float safeRadius = std::max(0.5F, pointSize * 0.5F);
                const auto radiusPixels = static_cast<int>(std::ceil(safeRadius));
                const int centerX = static_cast<int>(std::floor(pixelX));
                const int centerY = static_cast<int>(std::floor(pixelY));
                const int minX = std::max<int>(static_cast<int>(tile.x0), centerX - radiusPixels);
                const int maxX = std::min<int>(static_cast<int>(tile.x1) - 1, centerX + radiusPixels);
                const int minY = std::max<int>(static_cast<int>(tile.y0), centerY - radiusPixels);
                const int maxY = std::min<int>(static_cast<int>(tile.y1) - 1, centerY + radiusPixels);
                for (int y = minY; y <= maxY; ++y) {
                    for (int x = minX; x <= maxX; ++x) {
                        const float dx = (static_cast<float>(x) + 0.5F) - pixelX;
                        const float dy = (static_cast<float>(y) + 0.5F) - pixelY;
                        if (((dx * dx) + (dy * dy)) > safeRadius * safeRadius) {
                            continue;
                        }
                        const auto pixelIndex =
                            static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) +
                            static_cast<std::size_t>(x);
                        if (pixelIndex >= image->depth.size() || viewDepth >= image->depth[pixelIndex]) {
                            continue;
                        }
                        image->beautyR[pixelIndex] = color.r;
                        image->beautyG[pixelIndex] = color.g;
                        image->beautyB[pixelIndex] = color.b;
                        image->alpha[pixelIndex] = opacity;
                        image->depth[pixelIndex] = viewDepth;
                        if (diagnostics != nullptr) {
                            ++diagnostics->accumulationCoveredPixels;
                        }
                    }
                }
            } else {
                const int x = static_cast<int>(std::floor(pixelX));
                const int y = static_cast<int>(std::floor(pixelY));
                if (x < static_cast<int>(tile.x0) ||
                    y < static_cast<int>(tile.y0) ||
                    x >= static_cast<int>(tile.x1) ||
                    y >= static_cast<int>(tile.y1)) {
                    continue;
                }

                const auto pixelIndex =
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) +
                    static_cast<std::size_t>(x);
                if (pixelIndex >= image->depth.size() || viewDepth >= image->depth[pixelIndex]) {
                    continue;
                }

                image->beautyR[pixelIndex] = color.r;
                image->beautyG[pixelIndex] = color.g;
                image->beautyB[pixelIndex] = color.b;
                image->alpha[pixelIndex] = opacity;
                image->depth[pixelIndex] = viewDepth;
                if (diagnostics != nullptr) {
                    ++diagnostics->accumulationCoveredPixels;
                }
            }
        }

        if (diagnostics != nullptr) {
            diagnostics->accumulationVisitedPoints += drawPointCount;
        }
    }

    if (diagnostics != nullptr) {
        diagnostics->accumulationPassMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    }
}

void RenderPointCloudTile(
    const std::vector<OfflinePointLayer>& layers,
    const invisible_places::camera::CameraState& cameraState,
    const OfflineRenderTile& tile,
    ExrImage* image,
    OfflinePointRenderDiagnostics* diagnostics,
    OfflinePointRenderScratch* scratch,
    float stylisationTimeSeconds,
    const OfflineRainFrame* rainFrame) {
    if (image == nullptr || image->width == 0 || image->height == 0 || tile.x0 >= tile.x1 || tile.y0 >= tile.y1) {
        return;
    }
    if (diagnostics != nullptr) {
        *diagnostics = {};
    }

    invisible_places::camera::OrbitCamera camera;
    camera.ApplyState(cameraState);
    const float aspectRatio = static_cast<float>(image->width) / static_cast<float>(image->height);
    const auto matrices = camera.Matrices(aspectRatio);
    const std::uint32_t tileWidth = tile.x1 - tile.x0;
    const std::uint32_t tileHeight = tile.y1 - tile.y0;
    const auto tilePixelCount = static_cast<std::size_t>(tileWidth) * static_cast<std::size_t>(tileHeight);
    const bool fastBasicOnly = !layers.empty() && std::all_of(
        layers.begin(),
        layers.end(),
        [](const OfflinePointLayer& layer) { return layer.fastBasic; });
    if (fastBasicOnly) {
        RenderFastBasicPointCloudTile(layers, matrices, tile, image, diagnostics, stylisationTimeSeconds);
        CompositeOfflineRainDirect(rainFrame, matrices, tile, image);
        return;
    }

    OfflinePointRenderScratch localScratch;
    auto& activeScratch = scratch != nullptr ? *scratch : localScratch;
    activeScratch.accumR.assign(tilePixelCount, 0.0F);
    activeScratch.accumG.assign(tilePixelCount, 0.0F);
    activeScratch.accumB.assign(tilePixelCount, 0.0F);
    activeScratch.accumA.assign(tilePixelCount, 0.0F);
    activeScratch.revealage.assign(tilePixelCount, 1.0F);
    activeScratch.emissionR.assign(tilePixelCount, 0.0F);
    activeScratch.emissionG.assign(tilePixelCount, 0.0F);
    activeScratch.emissionB.assign(tilePixelCount, 0.0F);
    activeScratch.emissionA.assign(tilePixelCount, 0.0F);

    auto& accumR = activeScratch.accumR;
    auto& accumG = activeScratch.accumG;
    auto& accumB = activeScratch.accumB;
    auto& accumA = activeScratch.accumA;
    auto& revealage = activeScratch.revealage;
    auto& emissionR = activeScratch.emissionR;
    auto& emissionG = activeScratch.emissionG;
    auto& emissionB = activeScratch.emissionB;
    auto& emissionA = activeScratch.emissionA;

    const auto depthStart = std::chrono::steady_clock::now();
    for (const auto& layer : layers) {
        if (layer.cloud == nullptr ||
            layer.cloud->positions.empty()) {
            continue;
        }
        if (diagnostics != nullptr) {
            ++diagnostics->depthPassLayers;
            diagnostics->skippedInactiveBindings += InactivePointBindingCount(layer.style);
        }

        const auto& cloud = *layer.cloud;
        const float coverageCorrection =
            invisible_places::renderer::pointcloud::SanitizePointCloudDensityCompensation(
                layer.densityCompensation)
                .coverageCorrection;
        for (std::size_t chunkStart = 0; chunkStart < cloud.positions.size(); chunkStart += kOfflinePointChunkSize) {
            const auto chunkEnd = std::min(cloud.positions.size(), chunkStart + kOfflinePointChunkSize);
            for (std::size_t pointIndex = chunkStart; pointIndex < chunkEnd; ++pointIndex) {
                OfflinePointSample sample;
                if (!BuildOfflinePointSample(
                        layer,
                        matrices,
                        cameraState,
                        pointIndex,
                        *image,
                        &sample,
                        false,
                        stylisationTimeSeconds)) {
                    continue;
                }
                if (diagnostics != nullptr) {
                    ++diagnostics->depthVisitedPoints;
                }

                VisitCoveredPixels(
                    sample,
                    layer.style,
                    matrices,
                    *image,
                    tile,
                    tileWidth,
                    stylisationTimeSeconds,
                    [&](std::uint32_t x,
                        std::uint32_t y,
                        std::size_t,
                        float falloff,
                        glm::vec2,
                        float coveredViewDepth) {
                        const auto pixelIndex =
                            static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) +
                            static_cast<std::size_t>(x);
                        const float rawAlpha =
                            sample.opacity *
                            falloff *
                            ResolveDepthFadeAlpha(sample, cameraState, coveredViewDepth);
                        const float alpha = std::clamp(
                            rawAlpha * coverageCorrection,
                            0.0F,
                            AlphaClampMax(layer.style));
                        if (pixelIndex < image->depth.size() &&
                            coveredViewDepth < image->depth[pixelIndex] &&
                            invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(
                                alpha)) {
                            image->depth[pixelIndex] = coveredViewDepth;
                        }
                        if (diagnostics != nullptr) {
                            ++diagnostics->depthCoveredPixels;
                        }
                    });
            }
        }
    }

    const auto depthEnd = std::chrono::steady_clock::now();
    if (diagnostics != nullptr) {
        diagnostics->depthPassMs = std::chrono::duration<double, std::milli>(depthEnd - depthStart).count();
    }

    const auto accumulationStart = std::chrono::steady_clock::now();
    for (const auto& layer : layers) {
        if (layer.cloud == nullptr || layer.cloud->positions.empty()) {
            continue;
        }
        if (diagnostics != nullptr) {
            ++diagnostics->accumulationPassLayers;
            diagnostics->skippedInactiveBindings += InactivePointBindingCount(layer.style);
        }

        const auto& cloud = *layer.cloud;
        const float coverageCorrection =
            invisible_places::renderer::pointcloud::SanitizePointCloudDensityCompensation(
                layer.densityCompensation)
                .coverageCorrection;
        for (std::size_t chunkStart = 0; chunkStart < cloud.positions.size(); chunkStart += kOfflinePointChunkSize) {
            const auto chunkEnd = std::min(cloud.positions.size(), chunkStart + kOfflinePointChunkSize);
            for (std::size_t pointIndex = chunkStart; pointIndex < chunkEnd; ++pointIndex) {
                OfflinePointSample sample;
                if (!BuildOfflinePointSample(
                        layer,
                        matrices,
                        cameraState,
                        pointIndex,
                        *image,
                        &sample,
                        true,
                        stylisationTimeSeconds)) {
                    continue;
                }
                if (diagnostics != nullptr) {
                    ++diagnostics->accumulationVisitedPoints;
                }
                const float surfaceAngleMask = PointSurfaceAngleMask(sample, matrices);

                VisitCoveredPixels(
                    sample,
                    layer.style,
                    matrices,
                    *image,
                    tile,
                    tileWidth,
                    stylisationTimeSeconds,
                    [&](std::uint32_t x,
                        std::uint32_t y,
                        std::size_t localIndex,
                        float falloff,
                        glm::vec2 stylisationCoord,
                        float coveredViewDepth) {
                        const auto pixelIndex =
                            static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) +
                            static_cast<std::size_t>(x);
                        if (pixelIndex >= image->depth.size()) {
                            return;
                        }

                        const float rawAlpha =
                            sample.opacity *
                            falloff *
                            ResolveDepthFadeAlpha(sample, cameraState, coveredViewDepth);
                        const float compensatedRawAlpha = rawAlpha * coverageCorrection;
                        const float alpha = std::clamp(
                            compensatedRawAlpha,
                            0.0F,
                            AlphaClampMax(layer.style));
                        if (alpha <= 1.0e-5F) {
                            return;
                        }
                        if (diagnostics != nullptr) {
                            ++diagnostics->accumulationCoveredPixels;
                        }

                        const glm::vec3 stylisedColor = ApplyPointStylisationColor(
                            sample.color,
                            layer.style,
                            stylisationCoord,
                            sample.pointIndex,
                            surfaceAngleMask,
                            stylisationTimeSeconds);
                        const float weightedAlpha = std::clamp(
                            alpha,
                            0.0F,
                            AlphaClampMax(layer.style));
                        const float weight = WeightedAlphaWeight(weightedAlpha, coveredViewDepth, cameraState);
                        accumR[localIndex] += stylisedColor.r * weightedAlpha * weight;
                        accumG[localIndex] += stylisedColor.g * weightedAlpha * weight;
                        accumB[localIndex] += stylisedColor.b * weightedAlpha * weight;
                        accumA[localIndex] += weightedAlpha * weight;
                        revealage[localIndex] *= (1.0F - weightedAlpha);

                        const float emissionGain = sample.emissive * std::max(0.0F, layer.style.exposure);
                        if (emissionGain > 1.0e-5F) {
                            emissionR[localIndex] += stylisedColor.r * compensatedRawAlpha * emissionGain;
                            emissionG[localIndex] += stylisedColor.g * compensatedRawAlpha * emissionGain;
                            emissionB[localIndex] += stylisedColor.b * compensatedRawAlpha * emissionGain;
                            emissionA[localIndex] += compensatedRawAlpha * emissionGain;
                        }

                    });
            }
        }
    }

    VisitOfflineRainPixels(
        rainFrame,
        matrices,
        tile,
        *image,
        [&](std::size_t pixelIndex, std::uint32_t x, std::uint32_t y, float depth, float alpha,
            const glm::vec3& colour, float emission) {
            if (pixelIndex >= image->depth.size() || depth > image->depth[pixelIndex] + 0.02F) {
                return;
            }
            const auto localIndex =
                static_cast<std::size_t>(y - tile.y0) * static_cast<std::size_t>(tileWidth) +
                static_cast<std::size_t>(x - tile.x0);
            const float weight = WeightedAlphaWeight(alpha, depth, cameraState);
            accumR[localIndex] += colour.r * alpha * weight;
            accumG[localIndex] += colour.g * alpha * weight;
            accumB[localIndex] += colour.b * alpha * weight;
            accumA[localIndex] += alpha * weight;
            revealage[localIndex] *= 1.0F - alpha;
            emissionR[localIndex] += colour.r * alpha * emission;
            emissionG[localIndex] += colour.g * alpha * emission;
            emissionB[localIndex] += colour.b * alpha * emission;
            emissionA[localIndex] += alpha * emission;
        });

    const auto accumulationEnd = std::chrono::steady_clock::now();
    if (diagnostics != nullptr) {
        diagnostics->accumulationPassMs =
            std::chrono::duration<double, std::milli>(accumulationEnd - accumulationStart).count();
    }

    const auto compositeStart = std::chrono::steady_clock::now();
    for (std::uint32_t y = tile.y0; y < tile.y1; ++y) {
        for (std::uint32_t x = tile.x0; x < tile.x1; ++x) {
            const auto localIndex =
                static_cast<std::size_t>(y - tile.y0) * static_cast<std::size_t>(tileWidth) +
                static_cast<std::size_t>(x - tile.x0);
            const auto pixelIndex =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) + static_cast<std::size_t>(x);
            if (pixelIndex >= image->alpha.size()) {
                continue;
            }

            const float transparentAlpha = std::clamp(1.0F - revealage[localIndex], 0.0F, 1.0F);
            const glm::vec3 transparentColor =
                accumA[localIndex] > 1.0e-5F
                    ? glm::vec3{
                          accumR[localIndex] / accumA[localIndex],
                          accumG[localIndex] / accumA[localIndex],
                          accumB[localIndex] / accumA[localIndex]}
                    : glm::vec3{0.0F, 0.0F, 0.0F};
            const glm::vec3 emission{
                1.0F - std::exp(-std::max(0.0F, emissionR[localIndex])),
                1.0F - std::exp(-std::max(0.0F, emissionG[localIndex])),
                1.0F - std::exp(-std::max(0.0F, emissionB[localIndex])),
            };
            const float emissionAlpha =
                std::clamp(1.0F - std::exp(-std::max(0.0F, emissionA[localIndex])), 0.0F, 1.0F);
            const float sourceAlpha = std::max(transparentAlpha, emissionAlpha);
            if (sourceAlpha <= 1.0e-5F) {
                continue;
            }

            const glm::vec3 desiredContribution = (transparentColor * transparentAlpha) + emission;
            const glm::vec3 sourceColor = desiredContribution / std::max(sourceAlpha, 1.0e-5F);
            const float destinationAlpha = image->alpha[pixelIndex];
            const glm::vec3 destinationColor{
                image->beautyR[pixelIndex],
                image->beautyG[pixelIndex],
                image->beautyB[pixelIndex],
            };
            const float outputAlpha = sourceAlpha + (destinationAlpha * (1.0F - sourceAlpha));
            const glm::vec3 outputColor =
                (sourceColor * sourceAlpha) + (destinationColor * (1.0F - sourceAlpha));

            image->beautyR[pixelIndex] = outputColor.r;
            image->beautyG[pixelIndex] = outputColor.g;
            image->beautyB[pixelIndex] = outputColor.b;
            image->alpha[pixelIndex] = outputAlpha;
        }
    }
    if (diagnostics != nullptr) {
        diagnostics->compositePassMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compositeStart).count();
    }
}

}  // namespace invisible_places::output
