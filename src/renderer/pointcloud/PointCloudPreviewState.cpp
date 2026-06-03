#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::renderer::pointcloud {

namespace {

constexpr std::uint32_t kSurfelVerticesPerPoint = 6U;
constexpr float kMaterialEpsilon = 1.0e-5F;

void SortForCoherentDraw(std::vector<std::uint32_t>* indices) {
    if (indices == nullptr) {
        return;
    }

    std::sort(indices->begin(), indices->end());
}

std::uint64_t GreatestCommonDivisor(std::uint64_t left, std::uint64_t right) {
    while (right != 0) {
        const auto remainder = left % right;
        left = right;
        right = remainder;
    }

    return left;
}

std::uint64_t MakeRelativelyPrimeStep(std::uint64_t totalPoints) {
    if (totalPoints <= 1) {
        return 1;
    }

    std::uint64_t candidate = (totalPoints / 2U) | 1U;
    while (GreatestCommonDivisor(candidate, totalPoints) != 1U) {
        candidate += 2U;
    }

    return candidate;
}

struct VoxelKey {
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    std::uint32_t z = 0;

    [[nodiscard]] bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

std::uint64_t MortonSortKey(const VoxelKey& key, std::uint32_t depth) {
    std::uint64_t morton = 0;
    for (std::uint32_t bitIndex = depth; bitIndex > 0; --bitIndex) {
        const auto shift = bitIndex - 1U;
        morton = (morton << 1U) | ((key.x >> shift) & 1U);
        morton = (morton << 1U) | ((key.y >> shift) & 1U);
        morton = (morton << 1U) | ((key.z >> shift) & 1U);
    }
    return morton;
}

struct VoxelKeyHash {
    [[nodiscard]] std::size_t operator()(const VoxelKey& key) const {
        std::uint64_t value =
            (static_cast<std::uint64_t>(key.x) * 73856093ULL) ^
            (static_cast<std::uint64_t>(key.y) * 19349663ULL) ^
            (static_cast<std::uint64_t>(key.z) * 83492791ULL);
        value ^= value >> 33U;
        value *= 0xff51afd7ed558ccdULL;
        value ^= value >> 33U;
        return static_cast<std::size_t>(value);
    }
};

struct VoxelCandidate {
    VoxelKey key{};
    std::uint32_t pointIndex = 0;
    float distanceToCenterSquared = std::numeric_limits<float>::max();
};

struct OctreeSampleConfig {
    float xExtent = 0.0F;
    float yExtent = 0.0F;
    float zExtent = 0.0F;
    bool xActive = false;
    bool yActive = false;
    bool zActive = false;
    std::uint32_t activeDimensions = 0;
};

constexpr std::uint32_t kMaxOctreeDepth = 21U;

std::uint32_t VoxelDimensionForDepth(std::uint32_t depth, bool activeDimension) {
    if (!activeDimension || depth == 0U) {
        return 1U;
    }

    return 1U << std::min(depth, kMaxOctreeDepth);
}

std::uint32_t VoxelCoordinate(
    float value,
    float minimum,
    float extent,
    std::uint32_t dimension) {
    if (dimension <= 1U || extent <= 0.0F) {
        return 0U;
    }

    const auto normalized = std::clamp(
        (static_cast<double>(value) - static_cast<double>(minimum)) / static_cast<double>(extent),
        0.0,
        0.999999999999);
    return std::min<std::uint32_t>(
        dimension - 1U,
        static_cast<std::uint32_t>(normalized * static_cast<double>(dimension)));
}

float SquaredDistance(
    const invisible_places::io::Float3& point,
    const invisible_places::io::Float3& center) {
    const float dx = point.x - center.x;
    const float dy = point.y - center.y;
    const float dz = point.z - center.z;
    return (dx * dx) + (dy * dy) + (dz * dz);
}

struct FrustumPlane {
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
    float distance = 0.0F;
};

std::array<FrustumPlane, 6> FrustumPlanes(const glm::mat4& viewProjection) {
    const glm::vec4 row0{
        viewProjection[0][0],
        viewProjection[1][0],
        viewProjection[2][0],
        viewProjection[3][0]};
    const glm::vec4 row1{
        viewProjection[0][1],
        viewProjection[1][1],
        viewProjection[2][1],
        viewProjection[3][1]};
    const glm::vec4 row2{
        viewProjection[0][2],
        viewProjection[1][2],
        viewProjection[2][2],
        viewProjection[3][2]};
    const glm::vec4 row3{
        viewProjection[0][3],
        viewProjection[1][3],
        viewProjection[2][3],
        viewProjection[3][3]};
    const std::array<glm::vec4, 6> rawPlanes{
        row3 + row0,
        row3 - row0,
        row3 + row1,
        row3 - row1,
        row3 + row2,
        row3 - row2,
    };

    std::array<FrustumPlane, 6> planes{};
    for (std::size_t index = 0; index < rawPlanes.size(); ++index) {
        const glm::vec3 normal{rawPlanes[index].x, rawPlanes[index].y, rawPlanes[index].z};
        const float length = glm::length(normal);
        if (length <= 1.0e-7F || !std::isfinite(length)) {
            continue;
        }
        planes[index].normal = normal / length;
        planes[index].distance = rawPlanes[index].w / length;
    }
    return planes;
}

bool BoundsIntersectsFrustumPlanes(
    const invisible_places::io::Float3& minimum,
    const invisible_places::io::Float3& maximum,
    const std::array<FrustumPlane, 6>& planes) {
    for (const auto& plane : planes) {
        const glm::vec3 positive{
            plane.normal.x >= 0.0F ? maximum.x : minimum.x,
            plane.normal.y >= 0.0F ? maximum.y : minimum.y,
            plane.normal.z >= 0.0F ? maximum.z : minimum.z,
        };
        if (glm::dot(plane.normal, positive) + plane.distance < 0.0F) {
            return false;
        }
    }
    return true;
}

std::size_t FrustumCellIndex(std::uint32_t x, std::uint32_t y, std::uint32_t z, std::uint32_t dimension) {
    return static_cast<std::size_t>(x) +
           (static_cast<std::size_t>(y) * static_cast<std::size_t>(dimension)) +
           (static_cast<std::size_t>(z) * static_cast<std::size_t>(dimension) * static_cast<std::size_t>(dimension));
}

invisible_places::io::Float3 VoxelCenter(
    const invisible_places::io::Bounds3f& bounds,
    const VoxelKey& key,
    std::uint32_t xDimension,
    std::uint32_t yDimension,
    std::uint32_t zDimension) {
    const auto centerComponent = [](float minimum, float maximum, std::uint32_t coordinate, std::uint32_t dimension) {
        if (dimension <= 1U) {
            return 0.5F * (minimum + maximum);
        }

        const float extent = maximum - minimum;
        return minimum + ((static_cast<float>(coordinate) + 0.5F) / static_cast<float>(dimension)) * extent;
    };

    return {
        centerComponent(bounds.minimum.x, bounds.maximum.x, key.x, xDimension),
        centerComponent(bounds.minimum.y, bounds.maximum.y, key.y, yDimension),
        centerComponent(bounds.minimum.z, bounds.maximum.z, key.z, zDimension),
    };
}

OctreeSampleConfig MakeOctreeSampleConfig(
    const invisible_places::io::Bounds3f& bounds) {
    OctreeSampleConfig config;
    config.xExtent = bounds.maximum.x - bounds.minimum.x;
    config.yExtent = bounds.maximum.y - bounds.minimum.y;
    config.zExtent = bounds.maximum.z - bounds.minimum.z;

    const float largestExtent = std::max({config.xExtent, config.yExtent, config.zExtent});
    if (largestExtent <= 0.0F) {
        return config;
    }

    constexpr float kMinimumExtentRatio = 1.0e-5F;
    config.xActive = config.xExtent > largestExtent * kMinimumExtentRatio;
    config.yActive = config.yExtent > largestExtent * kMinimumExtentRatio;
    config.zActive = config.zExtent > largestExtent * kMinimumExtentRatio;
    config.activeDimensions =
        (config.xActive ? 1U : 0U) +
        (config.yActive ? 1U : 0U) +
        (config.zActive ? 1U : 0U);
    return config;
}

std::uint32_t InitialOctreeDepth(
    std::uint64_t requestedPoints,
    std::uint32_t activeDimensions) {
    if (requestedPoints <= 1U || activeDimensions == 0U) {
        return 0U;
    }

    const double cellsPerAxis = std::pow(
        static_cast<double>(requestedPoints),
        1.0 / static_cast<double>(activeDimensions));
    const double depth = std::ceil(std::log2(std::max(1.0, cellsPerAxis)));
    return static_cast<std::uint32_t>(
        std::clamp<double>(depth, 0.0, static_cast<double>(kMaxOctreeDepth)));
}

std::uint32_t RefinementDepthStep(
    std::uint64_t requestedPoints,
    std::size_t occupiedCells,
    std::uint32_t activeDimensions) {
    if (occupiedCells == 0 || activeDimensions == 0U) {
        return 1U;
    }

    const double fillRatio = static_cast<double>(requestedPoints) / static_cast<double>(occupiedCells);
    if (fillRatio <= 1.0) {
        return 1U;
    }

    const double cellsPerAxis = std::pow(fillRatio, 1.0 / static_cast<double>(activeDimensions));
    return static_cast<std::uint32_t>(
        std::clamp<double>(std::ceil(std::log2(std::max(1.0, cellsPerAxis))), 1.0, 4.0));
}

std::vector<VoxelCandidate> BuildOctreeCandidates(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    const OctreeSampleConfig& config,
    std::uint32_t depth,
    std::uint64_t requestedPoints) {
    const std::uint32_t xDimension = VoxelDimensionForDepth(depth, config.xActive);
    const std::uint32_t yDimension = VoxelDimensionForDepth(depth, config.yActive);
    const std::uint32_t zDimension = VoxelDimensionForDepth(depth, config.zActive);

    const auto reserveTarget = requestedPoints > (std::numeric_limits<std::uint64_t>::max() / 2U)
                                   ? requestedPoints
                                   : requestedPoints * 2U;
    std::unordered_map<VoxelKey, VoxelCandidate, VoxelKeyHash> candidates;
    candidates.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(
        std::max<std::uint64_t>(reserveTarget, 1024ULL),
        static_cast<std::uint64_t>(positions.size()))));

    const auto pointCount = static_cast<std::uint32_t>(positions.size());
    for (std::uint32_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
        const auto& point = positions[pointIndex];
        const VoxelKey key{
            .x = VoxelCoordinate(point.x, bounds.minimum.x, config.xExtent, xDimension),
            .y = VoxelCoordinate(point.y, bounds.minimum.y, config.yExtent, yDimension),
            .z = VoxelCoordinate(point.z, bounds.minimum.z, config.zExtent, zDimension),
        };
        const auto center = VoxelCenter(bounds, key, xDimension, yDimension, zDimension);
        const float distanceToCenter = SquaredDistance(point, center);

        auto candidateIt = candidates.find(key);
        if (candidateIt == candidates.end()) {
            candidates.emplace(
                key,
                VoxelCandidate{
                    .key = key,
                    .pointIndex = pointIndex,
                    .distanceToCenterSquared = distanceToCenter});
            continue;
        }

        auto& candidate = candidateIt->second;
        if (distanceToCenter < candidate.distanceToCenterSquared ||
            (distanceToCenter == candidate.distanceToCenterSquared && pointIndex < candidate.pointIndex)) {
            candidate.pointIndex = pointIndex;
            candidate.distanceToCenterSquared = distanceToCenter;
        }
    }

    std::vector<VoxelCandidate> orderedCandidates;
    orderedCandidates.reserve(candidates.size());
    for (const auto& [key, candidate] : candidates) {
        orderedCandidates.push_back(candidate);
    }

    std::sort(
        orderedCandidates.begin(),
        orderedCandidates.end(),
        [depth](const VoxelCandidate& left, const VoxelCandidate& right) {
            const auto leftKey = MortonSortKey(left.key, depth);
            const auto rightKey = MortonSortKey(right.key, depth);
            if (leftKey != rightKey) {
                return leftKey < rightKey;
            }
            return left.pointIndex < right.pointIndex;
        });
    return orderedCandidates;
}

std::vector<std::uint32_t> SelectStratifiedCandidateIndices(
    const std::vector<VoxelCandidate>& orderedCandidates,
    std::uint64_t requestedPoints) {
    if (orderedCandidates.empty()) {
        return {};
    }

    const auto requested = static_cast<std::size_t>(
        std::min<std::uint64_t>(requestedPoints, orderedCandidates.size()));
    std::vector<std::uint32_t> indices;
    indices.reserve(requested);

    if (orderedCandidates.size() <= requested) {
        for (const auto& candidate : orderedCandidates) {
            indices.push_back(candidate.pointIndex);
        }
        SortForCoherentDraw(&indices);
        return indices;
    }

    const auto candidateCount = static_cast<std::uint64_t>(orderedCandidates.size());
    const auto requestedCount = static_cast<std::uint64_t>(requested);
    for (std::uint64_t sampleIndex = 0; sampleIndex < requestedCount; ++sampleIndex) {
        const auto binBegin = (sampleIndex * candidateCount) / requestedCount;
        const auto binEnd = ((sampleIndex + 1U) * candidateCount) / requestedCount;
        const auto candidateIndex = std::min<std::uint64_t>(
            candidateCount - 1U,
            binBegin + ((std::max<std::uint64_t>(binEnd, binBegin + 1U) - binBegin) / 2U));
        indices.push_back(orderedCandidates[static_cast<std::size_t>(candidateIndex)].pointIndex);
    }

    SortForCoherentDraw(&indices);
    return indices;
}

}  // namespace

PointCloudStyleState::PointCloudStyleState() {
    invisible_places::style::SetScalarConstant(&pointSize, kInactivePointSizeDefault);
    invisible_places::style::SetScalarConstant(&surfelDiameter, kInactiveSurfelDiameterDefault);
    invisible_places::style::SetScalarConstant(&opacity, kInactiveOpacityDefault);
    invisible_places::style::SetScalarConstant(&emissiveStrength, kInactiveEmissionDefault);
    invisible_places::style::SetScalarConstant(&xrayStrength, kInactiveXrayDefault);
    invisible_places::style::SetScalarConstant(&depthFade, kInactiveDepthFadeDefault);
    invisible_places::style::SetScalarConstant(&colormapPosition, kInactiveColormapPositionDefault);
    xrayStrength.active = false;
    depthFade.active = false;
}

bool PointCloudStyleUsesDepthPrepass(const PointCloudStyleState& style) {
    return style.depthContribution != PointCloudDepthContribution::None;
}

bool PointCloudStyleUsesDepthPrepass(const PointCloudStyleState& style, bool sceneHasActiveXray) {
    return sceneHasActiveXray && PointCloudStyleUsesDepthPrepass(style);
}

bool PointCloudAlphaContributesDepth(const PointCloudStyleState& style, float alpha) {
    if (alpha <= 1.0e-5F || !PointCloudStyleUsesDepthPrepass(style)) {
        return false;
    }
    if (style.depthContribution == PointCloudDepthContribution::AlphaThreshold) {
        return alpha >= std::clamp(style.depthAlphaThreshold, 0.0F, 1.0F);
    }
    return true;
}

bool PointCloudStyleHasActiveXray(const PointCloudStyleState& style) {
    if (!style.xrayStrength.active) {
        return false;
    }
    if (style.xrayStrength.mode == invisible_places::style::ParameterSourceMode::Constant) {
        return style.xrayStrength.constantValue[0] > kMaterialEpsilon;
    }
    return std::max(style.xrayStrength.fieldMap.outputMin, style.xrayStrength.fieldMap.outputMax) >
           kMaterialEpsilon;
}

bool PointCloudStyleHasActiveStylisation(const PointCloudStyleState& style) {
    return style.stylisationMode != PointCloudStylisationMode::Off &&
           style.stylisationStrength > kMaterialEpsilon;
}

bool PointCloudStyleHasActiveRoughnessMotion(const PointCloudStyleState& style) {
    return style.roughnessMotionStrength > kMaterialEpsilon &&
           style.roughnessMotionSpeed > kMaterialEpsilon;
}

bool PointCloudStyleHasActiveCaustics(const PointCloudStyleState& style) {
    return style.causticAnimation &&
           (style.causticIntensity > kMaterialEpsilon ||
            style.causticPreviewTintAmount > kMaterialEpsilon) &&
           style.causticMaskFieldSlot >= 0 &&
           style.causticEdgeFieldSlot >= 0 &&
           style.causticSeedFieldSlot >= 0;
}

bool PointCloudStyleUsesWorldSizedScreenSprites(const PointCloudStyleState& style) {
    return style.geometryMode == PointCloudGeometryMode::ScreenSprites &&
           style.screenSpriteSizeMode == PointCloudScreenSpriteSizeMode::WorldMillimeters;
}

float WorldDiameterToScreenPointSizePixels(
    float diameterMeters,
    float viewDepth,
    float projectionScaleY,
    float viewportHeight) {
    const float safeDepth = std::max(0.001F, viewDepth);
    const float safeViewportHeight = std::max(1.0F, viewportHeight);
    return std::max(0.0F, diameterMeters) *
           std::abs(projectionScaleY) *
           safeViewportHeight /
           (2.0F * safeDepth);
}

PointCloudStyleState MakeFastBasicPointCloudStyle(
    const PointCloudStyleState& sourceStyle,
    bool hasSourceRgb) {
    PointCloudStyleState style;
    style.geometryMode = PointCloudGeometryMode::ScreenSprites;
    style.screenSpriteSizeMode = sourceStyle.geometryMode == PointCloudGeometryMode::ScreenSprites
                                     ? sourceStyle.screenSpriteSizeMode
                                     : PointCloudScreenSpriteSizeMode::Pixels;
    style.depthContribution = PointCloudDepthContribution::None;
    style.falloffProfile = PointCloudFalloffProfile::HardDisc;
    style.stylisationMode = PointCloudStylisationMode::Off;
    style.nprPreset = sourceStyle.nprPreset;
    style.colorMode = sourceStyle.colorMode == PointCloudColorMode::ScalarColormap
                          ? PointCloudColorMode::ScalarColormap
                          : (hasSourceRgb ? PointCloudColorMode::SourceRgb : PointCloudColorMode::SolidColor);
    style.colormap = sourceStyle.colormap;
    style.solidColor = sourceStyle.solidColor;
    style.gradientStartColor = sourceStyle.gradientStartColor;
    style.gradientEndColor = sourceStyle.gradientEndColor;
    style.colorizeColor = sourceStyle.colorizeColor;
    style.colorizeAmount = sourceStyle.colorizeAmount;
    style.stylisationStrength = 0.0F;
    style.roughnessMotionStrength = 0.0F;
    style.waterStreakAspect = sourceStyle.waterStreakAspect;
    style.flowAnimation = false;
    style.waterPathView = false;
    style.waterStreamOverlay = false;
    style.causticAnimation = sourceStyle.causticAnimation;
    style.causticIntensity = sourceStyle.causticIntensity;
    style.causticScale = sourceStyle.causticScale;
    style.causticSpeed = sourceStyle.causticSpeed;
    style.causticLineSharpness = sourceStyle.causticLineSharpness;
    style.causticWarp = sourceStyle.causticWarp;
    style.causticCellSizeMeters = sourceStyle.causticCellSizeMeters;
    style.causticLineWidthMeters = sourceStyle.causticLineWidthMeters;
    style.causticFeatherMeters = sourceStyle.causticFeatherMeters;
    style.causticSurfacePointSpacingMeters = sourceStyle.causticSurfacePointSpacingMeters;
    style.causticWarpAmplitudeMeters = sourceStyle.causticWarpAmplitudeMeters;
    style.causticTint = sourceStyle.causticTint;
    style.causticEmissionBoost = sourceStyle.causticEmissionBoost;
    style.causticOpacityBoost = sourceStyle.causticOpacityBoost;
    style.causticPointSizeBoost = 0.0F;
    style.causticPreviewTintAmount = sourceStyle.causticPreviewTintAmount;
    style.causticPreviewTintRegionId = sourceStyle.causticPreviewTintRegionId;
    style.causticMaskFieldSlot = sourceStyle.causticMaskFieldSlot;
    style.causticEdgeFieldSlot = sourceStyle.causticEdgeFieldSlot;
    style.causticSeedFieldSlot = sourceStyle.causticSeedFieldSlot;
    if (style.screenSpriteSizeMode == PointCloudScreenSpriteSizeMode::WorldMillimeters) {
        invisible_places::style::SetScalarConstant(&style.pointSize, 1.0F);
        style.surfelDiameter = sourceStyle.surfelDiameter;
    } else {
        invisible_places::style::SetScalarConstant(&style.pointSize, 1.0F);
        invisible_places::style::SetScalarConstant(&style.surfelDiameter, kInactiveSurfelDiameterDefault);
    }
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);
    style.colormapPosition = sourceStyle.colorMode == PointCloudColorMode::ScalarColormap
                                 ? sourceStyle.colormapPosition
                                 : style.colormapPosition;
    if (sourceStyle.colorMode != PointCloudColorMode::ScalarColormap) {
        invisible_places::style::SetScalarConstant(&style.colormapPosition, kInactiveColormapPositionDefault);
    }
    return style;
}

PointCloudMaterialVariant ResolvePointCloudMaterialVariant(const PointCloudStyleState& style) {
    const bool simpleColor =
        style.colorMode == PointCloudColorMode::SourceRgb ||
        style.colorMode == PointCloudColorMode::SolidColor;
    const bool constantPointGeometry =
        (!style.pointSize.active ||
         style.pointSize.mode == invisible_places::style::ParameterSourceMode::Constant) &&
        (!style.surfelDiameter.active ||
         style.surfelDiameter.mode == invisible_places::style::ParameterSourceMode::Constant);
    const bool constantOpacity =
        !style.opacity.active ||
        style.opacity.mode == invisible_places::style::ParameterSourceMode::Constant;
    const bool constantEmission =
        !style.emissiveStrength.active ||
        style.emissiveStrength.mode == invisible_places::style::ParameterSourceMode::Constant;
    const bool noColormapField =
        !style.colormapPosition.active ||
        style.colormapPosition.mode == invisible_places::style::ParameterSourceMode::Constant;
    const bool noDepthFade =
        !style.depthFade.active ||
        (style.depthFade.mode == invisible_places::style::ParameterSourceMode::Constant &&
         style.depthFade.constantValue[0] <= kMaterialEpsilon);
    const bool opaqueOpacity =
        !style.opacity.active ||
        (style.opacity.mode == invisible_places::style::ParameterSourceMode::Constant &&
         style.opacity.constantValue[0] >= 1.0F - kMaterialEpsilon);
    const bool noEmission =
        !style.emissiveStrength.active ||
        (style.emissiveStrength.mode == invisible_places::style::ParameterSourceMode::Constant &&
         style.emissiveStrength.constantValue[0] <= kMaterialEpsilon);
    const bool noColorize = style.colorizeAmount <= kMaterialEpsilon;

    if (style.flowAnimation || style.waterStreamOverlay) {
        return PointCloudMaterialVariant::Unified;
    }

    if (PointCloudStyleHasActiveStylisation(style)) {
        return PointCloudMaterialVariant::Unified;
    }

    if (PointCloudStyleHasActiveRoughnessMotion(style)) {
        return PointCloudMaterialVariant::Unified;
    }

    if (PointCloudStyleHasActiveCaustics(style)) {
        return PointCloudMaterialVariant::Unified;
    }

    if (style.falloffProfile == PointCloudFalloffProfile::HardDisc &&
        simpleColor &&
        constantPointGeometry &&
        opaqueOpacity &&
        noEmission &&
        noColormapField &&
        noDepthFade &&
        noColorize &&
        !PointCloudStyleHasActiveXray(style)) {
        return PointCloudMaterialVariant::OpaqueHardDisc;
    }

    if (simpleColor &&
        constantPointGeometry &&
        constantOpacity &&
        constantEmission &&
        noColormapField &&
        noDepthFade &&
        !PointCloudStyleHasActiveXray(style)) {
        return PointCloudMaterialVariant::ConstantSimple;
    }
    return PointCloudMaterialVariant::Unified;
}

const char* PointCloudMaterialVariantName(PointCloudMaterialVariant variant) {
    switch (variant) {
        case PointCloudMaterialVariant::OpaqueHardDisc:
            return "opaque-hard-disc";
        case PointCloudMaterialVariant::ConstantSimple:
            return "constant-simple";
        case PointCloudMaterialVariant::Unified:
            return "unified";
    }
    return "unified";
}

std::uint64_t ClampPointBudget(std::uint64_t totalPoints, std::uint64_t requestedPoints) {
    if (totalPoints == 0) {
        return 0;
    }

    if (requestedPoints == 0) {
        return 1;
    }

    return std::min(totalPoints, requestedPoints);
}

std::vector<std::uint32_t> GenerateDeterministicSampleIndices(
    std::uint64_t totalPoints,
    std::uint64_t requestedPoints) {
    const auto clampedRequested = ClampPointBudget(totalPoints, requestedPoints);
    if (clampedRequested == 0 || clampedRequested >= totalPoints) {
        return {};
    }

    std::vector<std::uint32_t> indices;
    indices.reserve(static_cast<std::size_t>(clampedRequested));

    const auto step = MakeRelativelyPrimeStep(totalPoints);
    const std::uint64_t seed = 0x9E3779B97F4A7C15ULL % totalPoints;

    for (std::uint64_t sampleIndex = 0; sampleIndex < clampedRequested; ++sampleIndex) {
        const auto pointIndex = (seed + (sampleIndex * step)) % totalPoints;
        indices.push_back(static_cast<std::uint32_t>(pointIndex));
    }

    SortForCoherentDraw(&indices);
    return indices;
}

PointBudgetState MakePointBudgetState(std::uint64_t totalPoints, std::uint64_t requestedPoints) {
    PointBudgetState state;
    state.totalPoints = totalPoints;
    state.activePoints = ClampPointBudget(totalPoints, requestedPoints);

    if (state.totalPoints > 0) {
        state.activeFraction =
            static_cast<float>(state.activePoints) / static_cast<float>(state.totalPoints);
    }

    state.sampledIndices = GenerateDeterministicSampleIndices(totalPoints, state.activePoints);
    return state;
}

std::vector<std::uint32_t> GenerateSpatialSampleIndices(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::uint64_t requestedPoints) {
    const auto totalPoints = static_cast<std::uint64_t>(positions.size());
    const auto clampedRequested = ClampPointBudget(totalPoints, requestedPoints);
    if (clampedRequested == 0 || clampedRequested >= totalPoints) {
        return {};
    }

    if (!bounds.valid || totalPoints > std::numeric_limits<std::uint32_t>::max()) {
        return GenerateDeterministicSampleIndices(totalPoints, clampedRequested);
    }

    const auto config = MakeOctreeSampleConfig(bounds);
    if (config.activeDimensions == 0U) {
        return GenerateDeterministicSampleIndices(totalPoints, clampedRequested);
    }

    auto depth = InitialOctreeDepth(clampedRequested, config.activeDimensions);
    std::vector<VoxelCandidate> bestCandidates;
    while (depth <= kMaxOctreeDepth) {
        auto candidates = BuildOctreeCandidates(positions, bounds, config, depth, clampedRequested);
        if (candidates.empty()) {
            break;
        }

        if (candidates.size() > bestCandidates.size()) {
            bestCandidates = std::move(candidates);
        }

        if (bestCandidates.size() >= clampedRequested ||
            bestCandidates.size() >= positions.size() ||
            depth == kMaxOctreeDepth) {
            break;
        }

        const auto step = RefinementDepthStep(
            clampedRequested,
            bestCandidates.size(),
            config.activeDimensions);
        depth = std::min(kMaxOctreeDepth, depth + step);
    }

    if (bestCandidates.empty()) {
        return GenerateDeterministicSampleIndices(totalPoints, clampedRequested);
    }

    return SelectStratifiedCandidateIndices(bestCandidates, clampedRequested);
}

std::vector<std::uint32_t> GenerateFrustumUnionPointIndices(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::span<const glm::mat4> viewProjections,
    std::uint32_t gridDimension) {
    if (positions.empty() ||
        !bounds.valid ||
        viewProjections.empty() ||
        positions.size() > std::numeric_limits<std::uint32_t>::max()) {
        return {};
    }

    const std::uint32_t dimension = std::clamp(gridDimension, 1U, 128U);
    const auto cellCount =
        static_cast<std::size_t>(dimension) *
        static_cast<std::size_t>(dimension) *
        static_cast<std::size_t>(dimension);
    if (cellCount == 0) {
        return {};
    }

    const float xExtent = bounds.maximum.x - bounds.minimum.x;
    const float yExtent = bounds.maximum.y - bounds.minimum.y;
    const float zExtent = bounds.maximum.z - bounds.minimum.z;
    const float safeXExtent = std::max(xExtent, 1.0e-6F);
    const float safeYExtent = std::max(yExtent, 1.0e-6F);
    const float safeZExtent = std::max(zExtent, 1.0e-6F);
    const float xCellExtent = safeXExtent / static_cast<float>(dimension);
    const float yCellExtent = safeYExtent / static_cast<float>(dimension);
    const float zCellExtent = safeZExtent / static_cast<float>(dimension);

    std::vector<std::array<FrustumPlane, 6>> frustums;
    frustums.reserve(viewProjections.size());
    for (const auto& viewProjection : viewProjections) {
        frustums.push_back(FrustumPlanes(viewProjection));
    }

    std::vector<std::uint8_t> visibleCells(cellCount, 0U);
    std::size_t visibleCellCount = 0;
    for (std::uint32_t z = 0; z < dimension; ++z) {
        for (std::uint32_t y = 0; y < dimension; ++y) {
            for (std::uint32_t x = 0; x < dimension; ++x) {
                const invisible_places::io::Float3 cellMinimum{
                    bounds.minimum.x + (static_cast<float>(x) * xCellExtent) - xCellExtent,
                    bounds.minimum.y + (static_cast<float>(y) * yCellExtent) - yCellExtent,
                    bounds.minimum.z + (static_cast<float>(z) * zCellExtent) - zCellExtent,
                };
                const invisible_places::io::Float3 cellMaximum{
                    bounds.minimum.x + (static_cast<float>(x + 1U) * xCellExtent) + xCellExtent,
                    bounds.minimum.y + (static_cast<float>(y + 1U) * yCellExtent) + yCellExtent,
                    bounds.minimum.z + (static_cast<float>(z + 1U) * zCellExtent) + zCellExtent,
                };
                bool visible = false;
                for (const auto& frustum : frustums) {
                    if (BoundsIntersectsFrustumPlanes(cellMinimum, cellMaximum, frustum)) {
                        visible = true;
                        break;
                    }
                }
                if (visible) {
                    visibleCells[FrustumCellIndex(x, y, z, dimension)] = 1U;
                    ++visibleCellCount;
                }
            }
        }
    }

    if (visibleCellCount == 0) {
        return {};
    }

    std::vector<std::uint32_t> indices;
    const double visibleFraction =
        static_cast<double>(visibleCellCount) / static_cast<double>(visibleCells.size());
    const auto reserveCount = static_cast<std::size_t>(std::min<double>(
        static_cast<double>(positions.size()),
        std::max(1024.0, static_cast<double>(positions.size()) * visibleFraction * 1.5)));
    indices.reserve(reserveCount);

    const auto pointCount = static_cast<std::uint32_t>(positions.size());
    for (std::uint32_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
        const auto& point = positions[pointIndex];
        const std::uint32_t x = VoxelCoordinate(point.x, bounds.minimum.x, safeXExtent, dimension);
        const std::uint32_t y = VoxelCoordinate(point.y, bounds.minimum.y, safeYExtent, dimension);
        const std::uint32_t z = VoxelCoordinate(point.z, bounds.minimum.z, safeZExtent, dimension);
        if (visibleCells[FrustumCellIndex(x, y, z, dimension)] != 0U) {
            indices.push_back(pointIndex);
        }
    }

    if (indices.size() >= positions.size()) {
        return {};
    }
    return indices;
}

std::vector<std::uint32_t> GenerateSurfelEncodedSampleIndices(
    const std::vector<std::uint32_t>& sampledPointIndices) {
    std::vector<std::uint32_t> indices;
    indices.reserve(sampledPointIndices.size() * kSurfelVerticesPerPoint);

    for (const auto pointIndex : sampledPointIndices) {
        if (pointIndex > (std::numeric_limits<std::uint32_t>::max() / kSurfelVerticesPerPoint)) {
            return {};
        }

        const std::uint32_t encodedBase = pointIndex * kSurfelVerticesPerPoint;
        for (std::uint32_t cornerIndex = 0; cornerIndex < kSurfelVerticesPerPoint; ++cornerIndex) {
            indices.push_back(encodedBase + cornerIndex);
        }
    }

    return indices;
}

PointBudgetState MakePointBudgetState(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::uint64_t requestedPoints) {
    PointBudgetState state;
    state.totalPoints = static_cast<std::uint64_t>(cloud.PointCount());
    state.activePoints = ClampPointBudget(state.totalPoints, requestedPoints);

    if (state.totalPoints > 0) {
        state.activeFraction =
            static_cast<float>(state.activePoints) / static_cast<float>(state.totalPoints);
    }

    state.sampledIndices = GenerateSpatialSampleIndices(cloud.positions, cloud.bounds, state.activePoints);
    return state;
}

std::uint64_t ResolveInteractivePointBudget(
    const PointBudgetState& budget,
    bool interactionActive,
    std::uint64_t interactivePointCap) {
    if (!interactionActive || interactivePointCap == 0 || budget.activePoints == 0) {
        return budget.activePoints;
    }

    return std::max<std::uint64_t>(1U, std::min(budget.activePoints, interactivePointCap));
}

PointCloudPreviewLodDecision ResolvePointCloudPreviewLod(
    const PointBudgetState& budget,
    PointCloudPreviewLodMode mode,
    bool cameraNavigationActive,
    bool cameraPlaybackActive,
    std::uint64_t lodTargetPoints) {
    PointCloudPreviewLodDecision decision;
    decision.drawPointCount = budget.activePoints;

    if (budget.activePoints == 0 ||
        lodTargetPoints == 0 ||
        budget.UsesSampledIndices() ||
        budget.activePoints <= lodTargetPoints) {
        return decision;
    }

    const bool cameraDriven = cameraNavigationActive || cameraPlaybackActive;
    switch (mode) {
        case PointCloudPreviewLodMode::FullResolution:
            return decision;
        case PointCloudPreviewLodMode::AutoCameraLod:
            decision.usesPreviewLod = cameraDriven;
            break;
        case PointCloudPreviewLodMode::ForceLod:
            decision.usesPreviewLod = true;
            break;
    }

    if (decision.usesPreviewLod) {
        decision.drawPointCount = std::max<std::uint64_t>(1U, std::min(budget.activePoints, lodTargetPoints));
    }
    return decision;
}

}  // namespace invisible_places::renderer::pointcloud
