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
constexpr std::size_t kWaterTrailDistanceFieldSlot = 7U;
constexpr std::size_t kWaterTrailLengthFieldSlot = 8U;
constexpr std::size_t kWaterTrailRouteStartFieldSlot = 9U;
constexpr std::size_t kWaterTrailRouteCountFieldSlot = 10U;
constexpr std::size_t kWaterTrailRouteLengthFieldSlot = 11U;
constexpr std::size_t kWaterTrailStartPhaseFieldSlot = 12U;
constexpr std::size_t kWaterTrailLateralOffsetFieldSlot = 13U;
constexpr std::size_t kWaterTrailPointAgeFieldSlot = 14U;
constexpr std::size_t kWaterTrailAgeFieldSlot = 15U;
constexpr std::size_t kWaterTrailSpeedFieldSlot = 16U;
constexpr std::size_t kWaterTrailWidthFieldSlot = 17U;
constexpr std::size_t kWaterTrailStreakLengthFieldSlot = 18U;
constexpr std::size_t kWaterTrailTangentXFieldSlot = 22U;
constexpr std::size_t kWaterTrailTangentYFieldSlot = 23U;
constexpr std::size_t kWaterTrailTangentZFieldSlot = 24U;
constexpr std::size_t kWaterTrailLaneIndexFieldSlot = 25U;
constexpr std::size_t kWaterTrailLaneCountFieldSlot = 26U;
constexpr std::size_t kWaterTrailLanePitchFieldSlot = 27U;
constexpr std::size_t kWaterTrailLaneSpanFieldSlot = 28U;
constexpr std::size_t kWaterTrailLaneCrossingFieldSlot = 29U;
constexpr std::size_t kWaterTrailCrossSeedFieldSlot = 30U;
constexpr float kWaterParticleSpeedScale = 0.12F;

float Clamp01(float value) {
    return std::clamp(value, 0.0F, 1.0F);
}

float SmoothStep(float edge0, float edge1, float value);

glm::vec3 ToGlm(const invisible_places::io::Float3& value) {
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

float WaterEffectFieldValue(
    const OfflinePointLayer& layer,
    std::size_t fieldSlot,
    std::size_t pointIndex,
    float fallback) {
    if (layer.cloud == nullptr || fieldSlot >= layer.cloud->scalarFields.size()) {
        return fallback;
    }
    return ScalarFieldValueBySlot(*layer.cloud, fieldSlot, pointIndex);
}

bool HasWaterEffectComposition(const OfflinePointLayer& layer) {
    return layer.cloud != nullptr &&
           layer.waterEffectEmissionAddFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectOpacityAddFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectOpacityMultiplyFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectPointSizeAddFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectPointSizeMultiplyFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectColourRedFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectColourGreenFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectColourBlueFieldSlot < layer.cloud->scalarFields.size() &&
           layer.waterEffectColourMixFieldSlot < layer.cloud->scalarFields.size();
}

glm::vec3 ApplyWaterEffectColour(
    const OfflinePointLayer& layer,
    std::size_t pointIndex,
    glm::vec3 baseColor) {
    if (!HasWaterEffectComposition(layer)) {
        return baseColor;
    }
    const float mixAmount = Clamp01(WaterEffectFieldValue(
        layer,
        layer.waterEffectColourMixFieldSlot,
        pointIndex,
        0.0F));
    if (mixAmount <= 1.0e-5F) {
        return baseColor;
    }
    const glm::vec3 effectColor{
        Clamp01(WaterEffectFieldValue(layer, layer.waterEffectColourRedFieldSlot, pointIndex, 0.62F)),
        Clamp01(WaterEffectFieldValue(layer, layer.waterEffectColourGreenFieldSlot, pointIndex, 0.88F)),
        Clamp01(WaterEffectFieldValue(layer, layer.waterEffectColourBlueFieldSlot, pointIndex, 1.0F)),
    };
    return glm::mix(baseColor, effectColor, mixAmount);
}

bool HasWaterParticleFields(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    return style.flowAnimation &&
           !style.waterTrailOverlay &&
           cloud.scalarFields.size() > kWaterJitterSeedFieldSlot;
}

bool HasWaterTrailFields(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style) {
    return style.waterTrailOverlay &&
           cloud.scalarFields.size() > kWaterTrailTangentZFieldSlot;
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

float HashCaustic01(const glm::vec2& p) {
    const glm::vec3 p3 = glm::fract(glm::vec3{p.x, p.y, p.x} * 0.1031F);
    const float mixed = glm::dot(p3, glm::vec3{p3.y, p3.z, p3.x} + glm::vec3{33.33F});
    const glm::vec3 folded = p3 + glm::vec3{mixed};
    return PositiveFract((folded.x + folded.y) * folded.z);
}

glm::vec2 HashCaustic2(const glm::vec2& p) {
    return {HashCaustic01(p), HashCaustic01(p + glm::vec2{19.19F})};
}

float StableCausticSeed(float regionOrSeed) {
    if (regionOrSeed >= 1.0F) {
        return HashCaustic01(glm::vec2{std::floor(regionOrSeed + 0.5F), 401.0F});
    }
    return 0.38196601125F;
}

glm::vec2 CausticSurfaceUv(const glm::vec3& worldPosition, const glm::vec3& normal) {
    if (glm::dot(normal, normal) <= 1.0e-8F) {
        return {worldPosition.x, worldPosition.y};
    }

    glm::vec3 n = glm::normalize(normal);
    if (n.z < 0.0F) {
        n = -n;
    }

    const float steepness = SmoothStep(0.18F, 0.82F, 1.0F - std::clamp(n.z, 0.0F, 1.0F));
    const glm::vec2 slope = glm::clamp(
        glm::vec2{-n.x, -n.y} / std::max(0.35F, n.z),
        glm::vec2{-1.75F},
        glm::vec2{1.75F});
    return glm::vec2{worldPosition.x, worldPosition.y} + slope * worldPosition.z * steepness * 0.32F;
}

float CausticVoronoiRidge(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    const glm::vec2& metersUv,
    float seed,
    float timeSeconds,
    float edge) {
    const float stableSeed = StableCausticSeed(seed);
    const float cellSize = std::max(0.005F, style.causticCellSizeMeters);
    const float lineWidth = std::max(0.0005F, style.causticLineWidthMeters);
    const float warpAmplitude = std::clamp(style.causticWarpAmplitudeMeters, 0.0F, 2.0F);
    const float feather = std::max(0.0005F, style.causticFeatherMeters);
    const float pointSpacing = std::max(0.0005F, style.causticSurfacePointSpacingMeters);
    const float edgeCore = SmoothStep(0.08F, 0.82F, Clamp01(edge));
    const glm::vec2 warpUv = metersUv / std::max(0.001F, cellSize * 1.7F);
    const glm::vec2 warpedMeters =
        metersUv +
        warpAmplitude *
            glm::vec2{
                std::sin(warpUv.y * 1.73F + timeSeconds * 0.71F + stableSeed * 6.28318530718F) +
                    0.35F * std::sin(warpUv.x * 3.19F - timeSeconds * 0.23F + stableSeed * 5.31F),
                std::cos(warpUv.x * 1.31F - timeSeconds * 0.63F + stableSeed * 4.398229715F) +
                    0.35F * std::cos(warpUv.y * 2.17F + timeSeconds * 0.29F + stableSeed * 7.13F),
            };
    const glm::vec2 uv = warpedMeters / cellSize;
    const glm::vec2 base{std::floor(uv.x), std::floor(uv.y)};
    const glm::vec2 f{PositiveFract(uv.x), PositiveFract(uv.y)};
    float first = 16.0F;
    float second = 16.0F;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            const glm::vec2 cell{static_cast<float>(x), static_cast<float>(y)};
            auto feature = HashCaustic2(base + cell + glm::vec2{stableSeed * 37.0F});
            feature = glm::vec2{0.5F} + glm::vec2{0.5F} *
                                      glm::vec2{
                                          std::sin((feature.x + timeSeconds * 0.17F + stableSeed) * 6.28318530718F),
                                          std::sin((feature.y - timeSeconds * 0.13F + stableSeed) * 6.28318530718F),
                                      };
            const float distance = glm::length(cell + feature - f);
            if (distance < first) {
                second = first;
                first = distance;
            } else if (distance < second) {
                second = distance;
            }
        }
    }
    const float ridgeDistanceMeters = std::max(0.0F, second - first) * cellSize;
    const float edgeWidthScale = 0.55F + (1.0F - 0.55F) * edgeCore;
    const float resolvedWidth = std::max(lineWidth, pointSpacing * 1.15F) * edgeWidthScale;
    const float resolvedFeather = std::max(feather, pointSpacing * 0.5F) * (0.75F + (1.0F - 0.75F) * edgeCore);
    const float core = 1.0F - SmoothStep(resolvedWidth, resolvedWidth + resolvedFeather, ridgeDistanceMeters);
    const float halo = 1.0F - SmoothStep(
        resolvedWidth + resolvedFeather * 2.0F,
        resolvedWidth + resolvedFeather * 7.0F,
        ridgeDistanceMeters);
    const float dissolveNoise = HashCaustic01(
        glm::floor((metersUv / std::max(0.001F, cellSize * 2.0F)) + glm::vec2{stableSeed * 23.0F}));
    const float dissolve = SmoothStep(0.04F, 0.74F, edgeCore + (dissolveNoise - 0.5F) * 0.18F);
    const float shimmer = 0.82F + 0.18F * std::sin(timeSeconds * 1.7F + stableSeed * 17.0F + first * 11.0F);
    return std::clamp((core * 1.12F + halo * 0.32F) * shimmer * dissolve, 0.0F, 1.0F);
}

float CausticEdgeGate(const invisible_places::renderer::pointcloud::PointCloudStyleState& style, const glm::vec2& metersUv, float edge, float seed) {
    const float stableSeed = StableCausticSeed(seed);
    const float cellSize = std::max(0.005F, style.causticCellSizeMeters);
    const float broadBreakup =
        HashCaustic01(glm::floor((metersUv / std::max(0.001F, cellSize * 2.5F)) + glm::vec2{stableSeed * 11.0F}));
    return SmoothStep(0.04F, 0.92F, edge + broadBreakup * 0.14F - 0.06F);
}

float CausticPreviewTint(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    float mask,
    float edge,
    float regionOrSeed) {
    const float amount = std::clamp(style.causticPreviewTintAmount, 0.0F, 1.0F);
    if (amount <= 1.0e-5F) {
        return 0.0F;
    }
    const float targetRegion = style.causticPreviewTintRegionId;
    if (targetRegion >= 0.5F &&
        std::abs(std::floor(regionOrSeed + 0.5F) - std::floor(targetRegion + 0.5F)) > 0.5F) {
        return 0.0F;
    }
    return std::clamp(mask * edge * amount, 0.0F, 1.0F);
}

float CausticColorMixAmount(float causticStrength, float previewTint) {
    return std::clamp(causticStrength * 0.55F + previewTint * 0.24F, 0.0F, 1.0F);
}

float ResolveCausticStrength(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    std::size_t pointIndex,
    const glm::vec3& worldPosition,
    float timeSeconds,
    float* previewTint) {
    if (previewTint != nullptr) {
        *previewTint = 0.0F;
    }
    if (!invisible_places::renderer::pointcloud::PointCloudStyleHasActiveCaustics(style)) {
        return 0.0F;
    }
    const float mask = Clamp01(ScalarFieldValueBySlot(
        cloud,
        static_cast<std::size_t>(style.causticMaskFieldSlot),
        pointIndex));
    if (mask <= 1.0e-5F) {
        return 0.0F;
    }
    const float edge = Clamp01(ScalarFieldValueBySlot(
        cloud,
        static_cast<std::size_t>(style.causticEdgeFieldSlot),
        pointIndex));
    const float seed = ScalarFieldValueBySlot(
        cloud,
        static_cast<std::size_t>(style.causticSeedFieldSlot),
        pointIndex);
    if (previewTint != nullptr) {
        *previewTint = CausticPreviewTint(style, mask, edge, seed);
    }
    const float temporal = std::max(0.0F, timeSeconds) * std::max(0.0F, style.causticSpeed);
    glm::vec3 normal{0.0F};
    if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
        normal = {
            cloud.normals[pointIndex].x,
            cloud.normals[pointIndex].y,
            cloud.normals[pointIndex].z,
        };
    }
    const glm::vec2 metersUv = CausticSurfaceUv(worldPosition, normal);
    const float ridge = CausticVoronoiRidge(style, metersUv, seed, temporal, edge);
    const float edgeGate = CausticEdgeGate(style, metersUv, edge, seed);
    return std::clamp(ridge * mask * edgeGate * std::max(0.0F, style.causticIntensity), 0.0F, 6.0F);
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
    std::size_t pointIndex,
    float timeSeconds) {
    const float routeLength = std::max(0.001F, ScalarFieldValueBySlot(cloud, kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailDistance = std::max(0.0F, ScalarFieldValueBySlot(cloud, kWaterTrailDistanceFieldSlot, pointIndex));
    const float trailAge = ScalarFieldValueBySlot(cloud, kWaterTrailAgeFieldSlot, pointIndex);
    const float baseStartPhase = ScalarFieldValueBySlot(cloud, kWaterTrailStartPhaseFieldSlot, pointIndex);
    const float speed = std::max(0.0F, ScalarFieldValueBySlot(cloud, kWaterTrailSpeedFieldSlot, pointIndex));
    const float trailStartPhase = PositiveFract(
        baseStartPhase +
        trailAge +
        std::max(0.0F, timeSeconds) * speed / routeLength);
    return trailStartPhase + trailDistance / routeLength;
}

float WaterTrailVisibility(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    float timeSeconds) {
    if (ScalarFieldValueBySlot(cloud, kWaterTrailRoleFieldSlot, pointIndex) < 0.5F) {
        return 0.0F;
    }
    const float phase = WaterTrailTravelPhase(cloud, pointIndex, timeSeconds);
    const float routeLength = std::max(0.001F, ScalarFieldValueBySlot(cloud, kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailStreakLength =
        std::max(0.001F, ScalarFieldValueBySlot(cloud, kWaterTrailStreakLengthFieldSlot, pointIndex));
    const float endFeather = std::clamp(trailStreakLength / routeLength, 0.001F, 0.08F);
    return 1.0F - SmoothStep(1.0F - endFeather, 1.0F, phase);
}

bool WaterTrailRouteValid(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t routeStart,
    std::size_t routeCount) {
    return routeCount >= 2U &&
           routeStart < cloud.positions.size() &&
           routeStart + routeCount <= cloud.positions.size();
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

    const float routePosition = PositiveFract(phase) * static_cast<float>(routeCount - 1U);
    const auto anchorOffset = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(routePosition)),
        routeCount - 1U);
    const float t = PositiveFract(routePosition);
    const auto p0Offset = anchorOffset > 0U ? anchorOffset - 1U : anchorOffset;
    const auto p1Offset = anchorOffset;
    const auto p2Offset = std::min<std::size_t>(anchorOffset + 1U, routeCount - 1U);
    const auto p3Offset = std::min<std::size_t>(anchorOffset + 2U, routeCount - 1U);
    return CatmullRomWater(
        ToGlm(cloud.positions[routeStart + p0Offset]),
        ToGlm(cloud.positions[routeStart + p1Offset]),
        ToGlm(cloud.positions[routeStart + p2Offset]),
        ToGlm(cloud.positions[routeStart + p3Offset]),
        t);
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

    const float routePosition = PositiveFract(phase) * static_cast<float>(routeCount - 1U);
    const auto anchorOffset = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(routePosition)),
        routeCount - 1U);
    const auto previousOffset = anchorOffset > 0U ? anchorOffset - 1U : anchorOffset;
    const auto nextOffset = std::min<std::size_t>(anchorOffset + 1U, routeCount - 1U);
    const glm::vec3 tangent =
        ToGlm(cloud.positions[routeStart + nextOffset]) -
        ToGlm(cloud.positions[routeStart + previousOffset]);
    return glm::dot(tangent, tangent) > 1.0e-8F ? glm::normalize(tangent) : glm::vec3{1.0F, 0.0F, 0.0F};
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

    const float routePosition = PositiveFract(phase) * static_cast<float>(routeCount - 1U);
    const auto anchorOffset = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(routePosition)),
        routeCount - 1U);
    const float t = PositiveFract(routePosition);
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
    std::size_t pointIndex,
    float timeSeconds,
    glm::vec3 basePosition) {
    const float phase = WaterTrailTravelPhase(cloud, pointIndex, timeSeconds);
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
    return routePosition + lateral * lateralOffset;
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
    if (binding.fieldMap.fieldSlot >= 0 &&
        static_cast<std::size_t>(binding.fieldMap.fieldSlot) < cloud.scalarFields.size()) {
        fieldStats = &cloud.scalarFields[static_cast<std::size_t>(binding.fieldMap.fieldSlot)];
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
    count += style.xrayStrength.active ? 0U : 1U;
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
    const auto colorized = HslToRgb({tintHsl.x, tintHsl.y, sourceHsl.z});
    return glm::mix(baseColor, colorized, amount);
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
    } else if (layer.style.colorMode == invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap &&
               !cloud.scalarFields.empty()) {
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

    return ApplyWaterEffectColour(layer, pointIndex, ApplyColorize(baseColor, layer.style));
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
    float xray = 0.0F;
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
    if (layer.roughnessMotionFieldSlot >= cloud.scalarFields.size() ||
        layer.style.roughnessMotionStrength <= 1.0e-5F) {
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

float ScreenBlend01(float baseValue, float contribution) {
    const float a = Clamp01(baseValue);
    const float b = Clamp01(contribution);
    return 1.0F - ((1.0F - a) * (1.0F - b));
}

void BlendRippleContribution(
    invisible_places::water::WaterRippleRuntimeContribution* target,
    const invisible_places::water::WaterRippleRuntimeContribution& contribution,
    invisible_places::water::WaterEffectBlendMode blendMode) {
    if (target == nullptr || contribution.scale <= 1.0e-5F) {
        return;
    }

    if (blendMode == invisible_places::water::WaterEffectBlendMode::Max) {
        target->scale = std::max(target->scale, contribution.scale);
        target->emissionAdd = std::max(target->emissionAdd, contribution.emissionAdd);
        target->opacityAdd = std::max(target->opacityAdd, contribution.opacityAdd);
        target->opacityMultiply = std::max(target->opacityMultiply, contribution.opacityMultiply);
        target->pointSizeAdd = std::max(target->pointSizeAdd, contribution.pointSizeAdd);
        target->pointSizeMultiply = std::max(target->pointSizeMultiply, contribution.pointSizeMultiply);
        if (contribution.colourMix >= target->colourMix) {
            target->colourMix = contribution.colourMix;
            target->colour = contribution.colour;
        }
        return;
    }
    if (blendMode == invisible_places::water::WaterEffectBlendMode::Override) {
        *target = contribution;
        return;
    }

    if (blendMode == invisible_places::water::WaterEffectBlendMode::Multiply) {
        target->scale = std::max(target->scale, contribution.scale);
        target->opacityMultiply *= contribution.opacityMultiply;
        target->pointSizeMultiply *= contribution.pointSizeMultiply;
        target->emissionAdd += contribution.emissionAdd;
        target->opacityAdd += contribution.opacityAdd;
        target->pointSizeAdd += contribution.pointSizeAdd;
    } else if (blendMode == invisible_places::water::WaterEffectBlendMode::Screen) {
        target->scale = ScreenBlend01(target->scale, contribution.scale);
        target->emissionAdd = ScreenBlend01(target->emissionAdd, contribution.emissionAdd);
        target->opacityAdd = ScreenBlend01(target->opacityAdd, contribution.opacityAdd);
        target->opacityMultiply *= contribution.opacityMultiply;
        target->pointSizeAdd = ScreenBlend01(target->pointSizeAdd, contribution.pointSizeAdd);
        target->pointSizeMultiply *= contribution.pointSizeMultiply;
    } else {
        target->scale = Clamp01(target->scale + contribution.scale);
        target->emissionAdd += contribution.emissionAdd;
        target->opacityAdd += contribution.opacityAdd;
        target->opacityMultiply *= contribution.opacityMultiply;
        target->pointSizeAdd += contribution.pointSizeAdd;
        target->pointSizeMultiply *= contribution.pointSizeMultiply;
    }

    const float nextMix = Clamp01(target->colourMix + contribution.colourMix);
    if (nextMix > 1.0e-5F) {
        target->colour = glm::mix(target->colour, contribution.colour, contribution.colourMix / nextMix);
    }
    target->colourMix = nextMix;
}

invisible_places::water::WaterRippleRuntimeContribution ResolveOfflineRippleContribution(
    const OfflinePointLayer& layer,
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    const glm::vec3& worldPosition,
    float timeSeconds) {
    invisible_places::water::WaterRippleRuntimeContribution result;
    if (pointIndex >= layer.rippleMembershipRanges.size() ||
        layer.rippleMemberships.empty() ||
        layer.rippleParams.empty()) {
        return result;
    }

    glm::vec3 worldNormal{0.0F, 0.0F, 1.0F};
    if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
        const glm::vec3 localNormal = ToGlm(cloud.normals[pointIndex]);
        if (glm::dot(localNormal, localNormal) > 1.0e-8F) {
            worldNormal = glm::normalize(glm::transpose(glm::inverse(glm::mat3{layer.localToWorld})) * localNormal);
        }
    }
    if (!IsFinite(worldNormal) || glm::dot(worldNormal, worldNormal) <= 1.0e-8F) {
        worldNormal = {0.0F, 0.0F, 1.0F};
    }

    const glm::uvec2 range = layer.rippleMembershipRanges[pointIndex];
    const std::size_t start = range.x;
    const std::size_t count = range.y;
    if (count == 0U || start >= layer.rippleMemberships.size()) {
        return result;
    }

    const std::size_t end = std::min(start + count, layer.rippleMemberships.size());
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
    for (std::size_t entryIndex = start; entryIndex < end; ++entryIndex) {
        const auto& membership = layer.rippleMemberships[entryIndex];
        if (membership.paramIndex >= layer.rippleParams.size()) {
            continue;
        }
        const auto& params = layer.rippleParams[membership.paramIndex];
        const auto contribution = invisible_places::water::EvaluateWaterRippleRuntimeContribution(
            params,
            membership,
            position,
            normal,
            timeSeconds);
        BlendRippleContribution(&result, contribution, params.blendMode);
    }
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
    const bool waterTrails = HasWaterTrailFields(cloud, layer.style);
    const bool waterParticles = HasWaterParticleFields(cloud, layer.style);
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
        if (WaterTrailVisibility(cloud, pointIndex, stylisationTimeSeconds) <= 0.0F) {
            return false;
        }
        localPoint = ResolveWaterTrailPosition(cloud, pointIndex, stylisationTimeSeconds, localPoint);
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
    float causticPreviewTint = 0.0F;
    const float caustic = ResolveCausticStrength(
        cloud,
        layer.style,
        pointIndex,
        sample->worldCenter,
        stylisationTimeSeconds,
        &causticPreviewTint);
    const bool waterEffects = HasWaterEffectComposition(layer);
    const float waterEffectPointSizeAdd = waterEffects
                                              ? WaterEffectFieldValue(
                                                    layer,
                                                    layer.waterEffectPointSizeAddFieldSlot,
                                                    pointIndex,
                                                    0.0F)
                                              : 0.0F;
    const float waterEffectPointSizeMultiply = waterEffects
                                                   ? std::max(
                                                         0.0F,
                                                         WaterEffectFieldValue(
                                                             layer,
                                                             layer.waterEffectPointSizeMultiplyFieldSlot,
                                                             pointIndex,
                                                             1.0F))
                                                   : 1.0F;
    const float waterEffectOpacityAdd = waterEffects
                                            ? WaterEffectFieldValue(
                                                  layer,
                                                  layer.waterEffectOpacityAddFieldSlot,
                                                  pointIndex,
                                                  0.0F)
                                            : 0.0F;
    const float waterEffectOpacityMultiply = waterEffects
                                                 ? std::max(
                                                       0.0F,
                                                       WaterEffectFieldValue(
                                                           layer,
                                                           layer.waterEffectOpacityMultiplyFieldSlot,
                                                           pointIndex,
                                                           1.0F))
                                                 : 1.0F;
    const float waterEffectEmissionAdd = waterEffects
                                             ? std::max(
                                                   0.0F,
                                                   WaterEffectFieldValue(
                                                       layer,
                                                       layer.waterEffectEmissionAddFieldSlot,
                                                       pointIndex,
                                                       0.0F))
                                             : 0.0F;
    const auto sparseRipple = ResolveOfflineRippleContribution(
        layer,
        cloud,
        pointIndex,
        sample->worldCenter,
        stylisationTimeSeconds);
    const float waterParticleSizeScale =
        waterParticles ? WaterParticleSizeScale(cloud, pointIndex, stylisationTimeSeconds) : 1.0F;
    const bool worldSizedScreenSprites =
        invisible_places::renderer::pointcloud::PointCloudStyleUsesWorldSizedScreenSprites(layer.style);
    if (worldSizedScreenSprites) {
        const float diameterMeters =
            (EvaluateBindingOrDefault(
                 cloud,
                 layer.style.surfelDiameter,
                 pointIndex,
                 invisible_places::renderer::pointcloud::kInactiveSurfelDiameterDefault) *
                 waterParticleSizeScale *
                 (1.0F + caustic * std::max(0.0F, layer.style.causticPointSizeBoost)) *
                 waterEffectPointSizeMultiply *
                 sparseRipple.pointSizeMultiply) +
            waterEffectPointSizeAdd +
            sparseRipple.pointSizeAdd;
        sample->pointSize = std::clamp(
            invisible_places::renderer::pointcloud::WorldDiameterToScreenPointSizePixels(
                diameterMeters,
                viewDepth,
                matrices.projection[1][1],
                static_cast<float>(image.height)),
            1.0F,
            64.0F);
    } else {
        sample->pointSize = std::clamp(
            (EvaluateBindingOrDefault(
                 cloud,
                 layer.style.pointSize,
                 pointIndex,
                 invisible_places::renderer::pointcloud::kInactivePointSizeDefault) *
                 waterParticleSizeScale *
                 (1.0F + caustic * std::max(0.0F, layer.style.causticPointSizeBoost)) *
                 waterEffectPointSizeMultiply *
                 sparseRipple.pointSizeMultiply) +
                waterEffectPointSizeAdd +
                sparseRipple.pointSizeAdd,
            1.0F,
            64.0F);
    }
    sample->worldSurfels = worldSurfels;
    sample->surfelDiameter = std::max(
        0.0F,
        (EvaluateBindingOrDefault(
             cloud,
             layer.style.surfelDiameter,
             pointIndex,
             invisible_places::renderer::pointcloud::kInactiveSurfelDiameterDefault) *
             waterParticleSizeScale *
             (1.0F + caustic * std::max(0.0F, layer.style.causticPointSizeBoost)) *
             waterEffectPointSizeMultiply *
             sparseRipple.pointSizeMultiply) +
            waterEffectPointSizeAdd +
            sparseRipple.pointSizeAdd);
    sample->surfelAspect = layer.style.flowAnimation
                                ? std::clamp(layer.style.waterStreakAspect, 1.0F, 32.0F)
                                : 1.0F;
    if (waterTrails) {
        sample->surfelDiameter = std::max(
            0.0001F,
            ScalarFieldValueBySlot(cloud, kWaterTrailWidthFieldSlot, pointIndex));
        const float trailStreakLength = std::max(
            sample->surfelDiameter,
            ScalarFieldValueBySlot(cloud, kWaterTrailStreakLengthFieldSlot, pointIndex));
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
             (1.0F + caustic * std::max(0.0F, layer.style.causticOpacityBoost)) *
             waterEffectOpacityMultiply *
             sparseRipple.opacityMultiply) +
        waterEffectOpacityAdd +
        sparseRipple.opacityAdd);
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
        sample->emissive += caustic * std::max(0.0F, layer.style.causticEmissionBoost);
        sample->emissive += waterEffectEmissionAdd;
        sample->emissive += sparseRipple.emissionAdd;
        sample->xray = Clamp01(
            EvaluateBindingOrDefault(
                cloud,
                layer.style.xrayStrength,
                pointIndex,
                invisible_places::renderer::pointcloud::kInactiveXrayDefault));
        sample->color = ResolvePointColor(layer, pointIndex);
        sample->color = glm::mix(
            sample->color,
            glm::vec3{
                layer.style.causticTint[0],
                layer.style.causticTint[1],
                layer.style.causticTint[2],
            },
            CausticColorMixAmount(caustic, causticPreviewTint));
        sample->color = glm::mix(
            sample->color,
            sparseRipple.colour,
            Clamp01(sparseRipple.colourMix));
    }
    sample->hasNormal = cloud.hasNormals && pointIndex < cloud.normals.size();
    if (sample->hasNormal) {
        const glm::vec3 localNormal = ToGlm(cloud.normals[pointIndex]);
        sample->hasNormal = glm::dot(localNormal, localNormal) > 1.0e-8F;
        if (sample->hasNormal) {
            sample->normal = glm::normalize(glm::transpose(glm::inverse(glm::mat3{layer.localToWorld})) * localNormal);
            sample->hasNormal = IsFinite(sample->normal) && glm::dot(sample->normal, sample->normal) > 1.0e-8F;
        }
    }
    sample->hasPreferredTangent = false;
    if (waterTrails) {
        const glm::vec3 localTangent = WaterTrailRouteTangent(
            cloud,
            pointIndex,
            WaterTrailTravelPhase(cloud, pointIndex, stylisationTimeSeconds));
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
                if (falloff <= 1.0e-5F || stylisationCoverage <= 1.0e-5F) {
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
                    falloff * stylisationCoverage,
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
            if (falloff <= 1.0e-5F || stylisationCoverage <= 1.0e-5F) {
                continue;
            }

            const auto localIndex =
                static_cast<std::size_t>(y - static_cast<int>(tile.y0)) * static_cast<std::size_t>(tileWidth) +
                static_cast<std::size_t>(x - static_cast<int>(tile.x0));
            callback(
                static_cast<std::uint32_t>(x),
                static_cast<std::uint32_t>(y),
                localIndex,
                falloff * stylisationCoverage,
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
        const bool waterTrails = HasWaterTrailFields(cloud, layer.style);
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
            if (waterTrails) {
                localPosition = ResolveWaterTrailPosition(cloud, pointIndex, stylisationTimeSeconds, localPosition);
            }
            const glm::vec4 worldPosition4 = layer.localToWorld * glm::vec4{localPosition, 1.0F};
            const glm::vec3 worldPosition{worldPosition4};
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

            const glm::vec3 color = ResolvePointColor(layer, pointIndex);
            if (worldSizedScreenSprites) {
                const float pointSize = std::clamp(
                    invisible_places::renderer::pointcloud::WorldDiameterToScreenPointSizePixels(
                        invisible_places::style::ScalarConstant(layer.style.surfelDiameter),
                        viewDepth,
                        matrices.projection[1][1],
                        static_cast<float>(image->height)),
                    1.0F,
                    64.0F);
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
                        image->alpha[pixelIndex] = 1.0F;
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
                image->alpha[pixelIndex] = 1.0F;
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
    float stylisationTimeSeconds) {
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

    const bool sceneHasActiveXray = std::any_of(
        layers.begin(),
        layers.end(),
        [](const OfflinePointLayer& layer) {
            return layer.cloud != nullptr &&
                   !layer.cloud->positions.empty() &&
                   invisible_places::renderer::pointcloud::PointCloudStyleHasActiveXray(layer.style);
        });

    const auto depthStart = std::chrono::steady_clock::now();
    for (const auto& layer : layers) {
        if (layer.cloud == nullptr ||
            layer.cloud->positions.empty() ||
            !invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(
                layer.style,
                sceneHasActiveXray)) {
            continue;
        }
        if (diagnostics != nullptr) {
            ++diagnostics->depthPassLayers;
            diagnostics->skippedInactiveBindings += InactivePointBindingCount(layer.style);
        }

        const auto& cloud = *layer.cloud;
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
                        const float alpha =
                            std::clamp(
                                sample.opacity * falloff * ResolveDepthFadeAlpha(sample, cameraState, coveredViewDepth),
                                0.0F,
                                AlphaClampMax(layer.style));
                        if (pixelIndex < image->depth.size() &&
                            coveredViewDepth < image->depth[pixelIndex] &&
                            invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(
                                layer.style,
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

                        const float alpha =
                            std::clamp(
                                sample.opacity * falloff * ResolveDepthFadeAlpha(sample, cameraState, coveredViewDepth),
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
                        const float densityScale = std::max(1.0F, layer.style.densityScale);
                        const float densityClamp = std::max(0.0F, layer.style.densityClamp);
                        const float weightedAlpha = std::clamp(
                            densityClamp > 0.0F ? std::min(alpha * densityScale, densityClamp) : alpha,
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
                            emissionR[localIndex] += stylisedColor.r * alpha * emissionGain;
                            emissionG[localIndex] += stylisedColor.g * alpha * emissionGain;
                            emissionB[localIndex] += stylisedColor.b * alpha * emissionGain;
                            emissionA[localIndex] += alpha * emissionGain;
                        }

                        if (sample.xray > 1.0e-5F) {
                            const float sceneDepth = image->depth[pixelIndex];
                            if (!std::isfinite(sceneDepth) || sample.xray <= 1.0e-5F) {
                                return;
                            }

                            const float behind = std::max(
                                coveredViewDepth - sceneDepth - std::max(0.0F, layer.style.depthBias),
                                0.0F);
                            const float hiddenFade =
                                std::exp(-behind * std::max(0.0F, layer.style.depthFalloff));
                            const float frontMask =
                                coveredViewDepth <= sceneDepth + std::max(0.0F, layer.style.depthBias) ? 1.0F : 0.0F;
                            const float xrayAlpha =
                                alpha * sample.xray *
                                std::lerp(
                                    layer.style.hiddenAlpha * hiddenFade,
                                    layer.style.frontAlpha,
                                    frontMask);
                            if (xrayAlpha <= 1.0e-5F) {
                                return;
                            }

                            const float gain = std::max(0.0F, layer.style.exposure);
                            emissionR[localIndex] += stylisedColor.r * xrayAlpha * gain;
                            emissionG[localIndex] += stylisedColor.g * xrayAlpha * gain;
                            emissionB[localIndex] += stylisedColor.b * xrayAlpha * gain;
                            emissionA[localIndex] += xrayAlpha * gain;
                        }
                    });
            }
        }
    }

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
