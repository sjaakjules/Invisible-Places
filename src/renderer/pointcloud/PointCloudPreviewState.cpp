#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <thread>
#include <unordered_map>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/matrix.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::renderer::pointcloud {

namespace {

constexpr std::uint32_t kSurfelVerticesPerPoint = 6U;
constexpr float kMaterialEpsilon = 1.0e-5F;

std::string NormalizePointCloudSceneRole(std::string_view sceneRole) {
    std::string normalized;
    normalized.reserve(sceneRole.size());
    for (const char character : sceneRole) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) == 0) {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(byte)));
    }
    return normalized;
}

std::string TrimShorelineProfileName(std::string_view value) {
    std::size_t begin = 0U;
    while (begin < value.size() &&
           std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
        ++begin;
    }
    std::size_t end = value.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1U])) != 0) {
        --end;
    }
    return std::string{value.substr(begin, end - begin)};
}

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
    invisible_places::style::SetScalarConstant(&depthFade, kInactiveDepthFadeDefault);
    invisible_places::style::SetScalarConstant(&colormapPosition, kInactiveColormapPositionDefault);
    depthFade.active = false;
}

PointCloudShorelineWaveSettings ExtractPointCloudShorelineWaveSettings(
    const PointCloudStyleState& style) {
    PointCloudShorelineWaveSettings settings;
    settings.enabled = style.shorelineWaveEnabled;
    settings.algorithm = style.shorelineWaveAlgorithm;
    settings.heightFoam = style.shorelineHeightFoam;
    auto& foam = settings.foamFronts;
    foam.boundaryZ = style.shorelineBoundaryZ;
    foam.heightReachMeters = style.shorelineHeightReachMeters;
    foam.edgeFadeMeters = style.shorelineEdgeFadeMeters;
    foam.directionX = style.shorelineDirectionX;
    foam.directionY = style.shorelineDirectionY;
    foam.patternScale = style.shorelinePatternScale;
    foam.wavelengthMeters = style.shorelineWavelengthMeters;
    foam.speed = style.shorelineSpeed;
    foam.warp = style.shorelineWarp;
    foam.turbulence = style.shorelineTurbulence;
    foam.density = style.shorelineDensity;
    foam.backgroundWash = style.shorelineBackgroundWash;
    foam.phase = style.shorelinePhase;
    foam.intensity = style.shorelineIntensity;
    foam.emissionAdd = style.shorelineEmissionAdd;
    foam.opacityAdd = style.shorelineOpacityAdd;
    foam.opacityMultiply = style.shorelineOpacityMultiply;
    foam.pointSizeAdd = style.shorelinePointSizeAdd;
    foam.pointSizeMultiply = style.shorelinePointSizeMultiply;
    foam.colourMix = style.shorelineColourMix;
    foam.colour = style.shorelineColour;
    foam.seed = style.shorelineSeed;
    return settings;
}

void ApplyPointCloudShorelineWaveSettings(
    PointCloudStyleState* style,
    const PointCloudShorelineWaveSettings& settings) {
    if (style == nullptr) {
        return;
    }
    style->shorelineWaveEnabled = settings.enabled;
    style->shorelineWaveAlgorithm = settings.algorithm;
    style->shorelineHeightFoam = settings.heightFoam;
    const auto& foam = settings.foamFronts;
    style->shorelineBoundaryZ = foam.boundaryZ;
    style->shorelineHeightReachMeters = foam.heightReachMeters;
    style->shorelineEdgeFadeMeters = foam.edgeFadeMeters;
    style->shorelineDirectionX = foam.directionX;
    style->shorelineDirectionY = foam.directionY;
    style->shorelinePatternScale = foam.patternScale;
    style->shorelineWavelengthMeters = foam.wavelengthMeters;
    style->shorelineSpeed = foam.speed;
    style->shorelineWarp = foam.warp;
    style->shorelineTurbulence = foam.turbulence;
    style->shorelineDensity = foam.density;
    style->shorelineBackgroundWash = foam.backgroundWash;
    style->shorelinePhase = foam.phase;
    style->shorelineIntensity = foam.intensity;
    style->shorelineEmissionAdd = foam.emissionAdd;
    style->shorelineOpacityAdd = foam.opacityAdd;
    style->shorelineOpacityMultiply = foam.opacityMultiply;
    style->shorelinePointSizeAdd = foam.pointSizeAdd;
    style->shorelinePointSizeMultiply = foam.pointSizeMultiply;
    style->shorelineColourMix = foam.colourMix;
    style->shorelineColour = foam.colour;
    style->shorelineSeed = foam.seed;
}

PointCloudShorelineWaveSettings CalmPointCloudShorelineWaveSettings() {
    PointCloudShorelineWaveSettings settings;
    settings.enabled = true;
    settings.algorithm = PointCloudShorelineWaveAlgorithm::FoamFronts;
    auto& foam = settings.foamFronts;
    foam.boundaryZ = 1.595F;
    foam.heightReachMeters = 2.0F;
    foam.edgeFadeMeters = 0.117F;
    constexpr float kDirectionRadians = 81.0F * 3.14159265358979323846F / 180.0F;
    foam.directionX = std::cos(kDirectionRadians);
    foam.directionY = std::sin(kDirectionRadians);
    foam.wavelengthMeters = 0.10F;
    foam.patternScale = 0.33F;
    foam.speed = 0.55F;
    foam.warp = 1.05F;
    foam.turbulence = 0.64F;
    foam.density = 1.0F;
    foam.intensity = 0.97F;
    foam.emissionAdd = 0.65F;
    foam.opacityAdd = 0.08F;
    foam.opacityMultiply = 1.25F;
    foam.pointSizeAdd = 0.0F;
    foam.pointSizeMultiply = 1.35F;
    foam.colourMix = 0.75F;
    foam.colour = {
        158.0F / 255.0F,
        224.0F / 255.0F,
        1.0F,
    };
    return settings;
}

const PointCloudShorelineWaveProfile* FindPointCloudShorelineWaveProfile(
    std::span<const PointCloudShorelineWaveProfile> profiles,
    std::uint32_t shorelineInstanceId,
    std::string_view profileName) {
    const std::string normalized = TrimShorelineProfileName(profileName);
    if (normalized.empty()) {
        return nullptr;
    }
    const auto found = std::find_if(
        profiles.begin(),
        profiles.end(),
        [&](const PointCloudShorelineWaveProfile& profile) {
            return TrimShorelineProfileName(profile.name) == normalized &&
                   (!profile.objectOverride ||
                    profile.shorelineInstanceId == shorelineInstanceId);
        });
    return found != profiles.end() ? &*found : nullptr;
}

std::string UniquePointCloudShorelineObjectProfileName(
    std::span<const PointCloudShorelineWaveProfile> profiles,
    std::string_view baseProfileName,
    std::string_view objectName,
    std::uint32_t shorelineInstanceId) {
    const std::string base = TrimShorelineProfileName(baseProfileName);
    if (base.empty()) {
        return {};
    }
    const std::string object = TrimShorelineProfileName(objectName);
    const std::string preferred =
        base + "_" + (object.empty() ? std::string{"Shoreline"} : object);

    const auto nameInUse = [&](std::string_view candidate) {
        bool ignoredExistingCopy = false;
        for (const auto& profile : profiles) {
            const bool sameOwnedCopy =
                profile.objectOverride &&
                profile.shorelineInstanceId == shorelineInstanceId &&
                TrimShorelineProfileName(profile.baseProfileName) == base;
            if (sameOwnedCopy && !ignoredExistingCopy) {
                ignoredExistingCopy = true;
                continue;
            }
            if (TrimShorelineProfileName(profile.name) == candidate) {
                return true;
            }
        }
        return false;
    };

    if (!nameInUse(preferred)) {
        return preferred;
    }
    for (std::uint32_t suffix = 2U; suffix < 10000U; ++suffix) {
        const std::string candidate =
            preferred + " " + std::to_string(suffix);
        if (!nameInUse(candidate)) {
            return candidate;
        }
    }
    return preferred + " Copy";
}

bool PointCloudAlphaContributesDepth(float alpha) {
    return alpha > kMaterialEpsilon;
}

bool PointCloudStyleHasActiveStylisation(const PointCloudStyleState& style) {
    return style.stylisationMode != PointCloudStylisationMode::Off &&
           style.stylisationStrength > kMaterialEpsilon;
}

bool TimingColouriseStackHasActiveEffects(
    const ResolvedTimingColouriseStack& stack) {
    const std::size_t effectCount = std::min<std::size_t>(
        stack.effectCount,
        stack.effects.size());
    for (std::size_t effectIndex = 0; effectIndex < effectCount; ++effectIndex) {
        const auto& effect = stack.effects[effectIndex];
        if (!effect.enabled ||
            !std::isfinite(effect.lowerBound) ||
            !std::isfinite(effect.upperBound) ||
            effect.upperBound <= effect.lowerBound) {
            continue;
        }
        if (effect.source != TimingColouriseSource::ScalarField ||
            effect.scalarFieldSlot >= 0) {
            return true;
        }
    }
    return false;
}

bool PointCloudStyleHasActiveRoughnessMotion(const PointCloudStyleState& style) {
    return style.roughnessMotionStrength > kMaterialEpsilon &&
           style.roughnessMotionSpeed > kMaterialEpsilon;
}

bool PointCloudStyleHasActiveShorelineWaves(const PointCloudStyleState& style) {
    if (!PointCloudStyleHasShorelineWaveRegion(style)) {
        return false;
    }
    if (style.shorelineWaveAlgorithm == PointCloudShorelineWaveAlgorithm::HeightFoam) {
        return style.shorelineHeightFoam.speed > kMaterialEpsilon;
    }
    return style.shorelineSpeed > kMaterialEpsilon;
}

bool PointCloudShorelineWaveSettingsHasActiveMotion(
    const PointCloudShorelineWaveSettings& settings) {
    if (!settings.enabled) {
        return false;
    }
    if (settings.algorithm ==
        PointCloudShorelineWaveAlgorithm::HeightFoam) {
        return settings.heightFoam.speed > kMaterialEpsilon &&
               settings.heightFoam.offshoreReachMeters >
                   kMaterialEpsilon &&
               settings.heightFoam.wavelengthMeters >
                   kMaterialEpsilon &&
               settings.heightFoam.intensity > kMaterialEpsilon;
    }
    return settings.foamFronts.speed > kMaterialEpsilon &&
           settings.foamFronts.heightReachMeters >
               kMaterialEpsilon &&
           settings.foamFronts.wavelengthMeters >
               kMaterialEpsilon &&
           settings.foamFronts.intensity > kMaterialEpsilon;
}

bool PointCloudStyleHasShorelineWaveRegion(const PointCloudStyleState& style) {
    if (!style.shorelineWaveEnabled) {
        return false;
    }
    if (style.shorelineWaveAlgorithm == PointCloudShorelineWaveAlgorithm::HeightFoam) {
        return style.shorelineHeightFoam.offshoreReachMeters > kMaterialEpsilon &&
               style.shorelineHeightFoam.wavelengthMeters > kMaterialEpsilon &&
               style.shorelineHeightFoam.intensity > kMaterialEpsilon;
    }
    return style.shorelineHeightReachMeters > kMaterialEpsilon &&
           style.shorelineWavelengthMeters > kMaterialEpsilon &&
           style.shorelineIntensity > kMaterialEpsilon;
}

bool PointCloudSceneRoleAllowsRoughnessMotion(std::string_view sceneRole) {
    if (sceneRole.empty()) {
        return true;
    }
    const auto normalized = NormalizePointCloudSceneRole(sceneRole);
    return normalized == "veg" || normalized == "vegetation";
}

PointCloudStyleState MakePointCloudStyleForSceneRole(
    PointCloudStyleState style,
    std::string_view sceneRole) {
    const bool groupedSceneRole = !sceneRole.empty();
    if (!PointCloudSceneRoleAllowsRoughnessMotion(sceneRole)) {
        style.roughnessMotionStrength = 0.0F;
    }
    if (groupedSceneRole) {
        const auto normalized = NormalizePointCloudSceneRole(sceneRole);
        if (normalized != "sand") {
            style.shorelineWaveEnabled = false;
        }
        style.roughnessMotionFullLayer = normalized == "veg" || normalized == "vegetation";
        if (style.depthRolePolicy == PointCloudDepthRolePolicy::RockOccluder) {
            if (normalized == "rock") {
                style.effectiveDepthParticipation =
                    PointCloudDepthParticipation::WriteAndTest;
            } else if (normalized == "sand") {
                style.effectiveDepthParticipation =
                    PointCloudDepthParticipation::TestOnly;
            } else if (normalized == "veg" || normalized == "vegetation") {
                style.effectiveDepthParticipation =
                    PointCloudDepthParticipation::Disabled;
            }
        } else if (style.depthRolePolicy ==
                   PointCloudDepthRolePolicy::Custom) {
            if (normalized == "rock") {
                style.effectiveDepthParticipation =
                    style.rockDepthParticipation;
            } else if (normalized == "sand") {
                style.effectiveDepthParticipation =
                    style.sandDepthParticipation;
            } else if (normalized == "veg" || normalized == "vegetation") {
                style.effectiveDepthParticipation =
                    style.vegetationDepthParticipation;
            }
        } else {
            style.effectiveDepthParticipation =
                PointCloudDepthParticipation::WriteAndTest;
        }
        if (style.surfaceStabilityPolicy ==
            PointCloudSurfaceStabilityPolicy::StableRoles) {
            if (normalized == "rock") {
                style.effectiveSurfaceStabilityMode =
                    PointCloudSurfaceStabilityMode::SoftSeparation;
            } else if (normalized == "sand") {
                style.effectiveSurfaceStabilityMode =
                    PointCloudSurfaceStabilityMode::DensityContinuity;
            } else {
                style.effectiveSurfaceStabilityMode =
                    PointCloudSurfaceStabilityMode::DrawAll;
            }
        } else if (style.surfaceStabilityPolicy ==
                   PointCloudSurfaceStabilityPolicy::Custom) {
            if (normalized == "rock") {
                style.effectiveSurfaceStabilityMode =
                    style.rockSurfaceStabilityMode;
            } else if (normalized == "sand") {
                style.effectiveSurfaceStabilityMode =
                    style.sandSurfaceStabilityMode;
            } else if (normalized == "veg" || normalized == "vegetation") {
                style.effectiveSurfaceStabilityMode =
                    style.vegetationSurfaceStabilityMode;
            } else {
                style.effectiveSurfaceStabilityMode =
                    style.uniformSurfaceStabilityMode;
            }
        } else {
            style.effectiveSurfaceStabilityMode =
                style.uniformSurfaceStabilityMode;
        }
    } else {
        style.roughnessMotionFullLayer = false;
        // Role policies apply only to canonical grouped scene roles. Generated
        // overlays and ordinary standalone point clouds retain the historical
        // write-and-test behaviour.
        style.effectiveDepthParticipation =
            PointCloudDepthParticipation::WriteAndTest;
        style.effectiveSurfaceStabilityMode =
            style.uniformSurfaceStabilityMode;
    }
    return style;
}

bool PointCloudDepthPrepassWrites(const PointCloudStyleState& style) {
    return style.depthPrepassEnabled &&
           style.effectiveDepthParticipation ==
               PointCloudDepthParticipation::WriteAndTest;
}

bool PointCloudDepthPrepassTests(const PointCloudStyleState& style) {
    return style.depthPrepassEnabled &&
           style.effectiveDepthParticipation !=
               PointCloudDepthParticipation::Disabled;
}

float ResolvePointCloudSurfaceStabilityWeight(
    PointCloudSurfaceStabilityMode mode,
    std::uint32_t packedWeights,
    float influence) {
    std::uint32_t channel = 0U;
    switch (mode) {
        case PointCloudSurfaceStabilityMode::Original:
        case PointCloudSurfaceStabilityMode::DrawAll:
            return 1.0F;
        case PointCloudSurfaceStabilityMode::DensityContinuity:
            channel = 0U;
            break;
        case PointCloudSurfaceStabilityMode::PreferLower:
            channel = 1U;
            break;
        case PointCloudSurfaceStabilityMode::PreferUpper:
            channel = 2U;
            break;
        case PointCloudSurfaceStabilityMode::SoftSeparation:
            channel = 3U;
            break;
    }
    const float selected = static_cast<float>(
        (packedWeights >> (channel * 8U)) & 0xffU) / 255.0F;
    return std::lerp(
        1.0F,
        selected,
        std::clamp(
            std::isfinite(influence) ? influence : 1.0F,
            0.0F,
            1.0F));
}

bool PointCloudStyleUsesWorldSizedScreenSprites(const PointCloudStyleState& style) {
    return style.geometryMode == PointCloudGeometryMode::ScreenSprites &&
           style.screenSpriteSizeMode == PointCloudScreenSpriteSizeMode::WorldMillimeters;
}

WaterFlowActivityScales ResolveWaterFlowActivityScales(
    float effectiveActivity,
    float trailSeed) {
    WaterFlowActivityScales scales;
    scales.activity = std::clamp(
        std::isfinite(effectiveActivity) ? effectiveActivity : 1.0F,
        0.0F,
        1.0F);
    // Every sample in a trail shares one seed, so using that seed as an
    // activity threshold switches whole trails on and off while a keyed value
    // crosses it. Fade the settled population uniformly instead: the routes
    // and their deterministic seed variation remain stable at every strength.
    (void)trailSeed;
    scales.trailVisibility = scales.activity;
    scales.appearance = 0.30F + 0.70F * scales.activity;
    scales.width = 0.65F + 0.35F * scales.activity;
    // Travel phase is derived from absolute render time. Modulating its speed
    // with a keyed strength would therefore jump the phase whenever strength
    // changes. Trail Speed remains the sole motion-speed control.
    scales.speed = 1.0F;
    scales.visibleLength = 0.55F + 0.45F * scales.activity;
    scales.lateralMotion = 0.15F;
    return scales;
}

float SanitizeWaterFlowSpeedScale(float speedScale) {
    return std::isfinite(speedScale) ? std::max(0.0F, speedScale) : 1.0F;
}

float ShorelineWaveHeightMask(
    float boundaryZ,
    float reachMeters,
    float edgeFadeMeters,
    float worldZ) {
    const float safeReach = std::max(0.001F, reachMeters);
    const float safeFade = std::max(0.0F, edgeFadeMeters);
    const float shoreDistance = boundaryZ - worldZ;
    if (shoreDistance < -safeFade || shoreDistance > safeReach + safeFade) {
        return 0.0F;
    }
    const auto smoothstep = [](float edge0, float edge1, float value) {
        const float width = std::max(edge1 - edge0, 1.0e-5F);
        const float t = std::clamp((value - edge0) / width, 0.0F, 1.0F);
        return t * t * (3.0F - 2.0F * t);
    };
    const float waterSide = smoothstep(-safeFade, std::max(safeFade, 1.0e-5F), shoreDistance);
    const float reachFade =
        1.0F - smoothstep(safeReach, safeReach + std::max(safeFade, 1.0e-5F), shoreDistance);
    return std::clamp(waterSide * reachFade, 0.0F, 1.0F);
}

float NormalizeHeightFoamBreakZ(
    float runupZ,
    float offshoreReachMeters,
    float edgeFadeMeters,
    float breakZ) {
    const float safeReach = std::max(0.001F, offshoreReachMeters);
    const float offshoreZ = runupZ - safeReach;
    const float margin = std::min(std::max(0.001F, edgeFadeMeters), safeReach * 0.45F);
    const float minimumBreakZ = offshoreZ + margin;
    const float maximumBreakZ = runupZ - margin;
    if (minimumBreakZ >= maximumBreakZ) {
        return offshoreZ + safeReach * 0.5F;
    }
    return std::clamp(breakZ, minimumBreakZ, maximumBreakZ);
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

PointCloudRendererMode ResolvePointCloudLayerRendererMode(
    PointCloudRendererMode requestedMode,
    bool generatedWaterOverlay,
    const PointCloudStyleState& style) {
    if (requestedMode == PointCloudRendererMode::FastBasic &&
        generatedWaterOverlay &&
        style.waterTrailOverlay) {
        return PointCloudRendererMode::Beauty;
    }
    return requestedMode;
}

PointCloudStyleState MakeFastBasicPointCloudStyle(
    const PointCloudStyleState& sourceStyle,
    bool hasSourceRgb) {
    PointCloudStyleState style;
    style.geometryMode = PointCloudGeometryMode::ScreenSprites;
    style.screenSpriteSizeMode = sourceStyle.geometryMode == PointCloudGeometryMode::ScreenSprites
                                     ? sourceStyle.screenSpriteSizeMode
                                     : PointCloudScreenSpriteSizeMode::Pixels;
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
    style.waterTrailStyleGeometry = sourceStyle.waterTrailStyleGeometry;
    style.waterFlowActivity = sourceStyle.waterFlowActivity;
    style.waterFlowSpeedScale = sourceStyle.waterFlowSpeedScale;
    // Fast Basic is explicitly opaque. Beauty-only transparency controls
    // neither change its material nor add sorting/depth passes.
    style.gpuBackToFrontSorting = false;
    style.depthPrepassEnabled = false;
    style.depthPrepassAlphaThreshold = sourceStyle.depthPrepassAlphaThreshold;
    style.depthPrepassToleranceMeters = sourceStyle.depthPrepassToleranceMeters;
    style.depthWeightStrength = 1.0F;
    style.emissionResponse = PointCloudEmissionResponse::Accumulated;
    style.normalCullEnabled = false;
    style.normalCullStartDegrees = sourceStyle.normalCullStartDegrees;
    style.normalCullEndDegrees = sourceStyle.normalCullEndDegrees;
    // The linked overlap analysis is a geometry selection, not a Beauty
    // material effect. Preserve it so Fast Basic can apply the same stable
    // choice as fixed opaque point coverage in its vertex path.
    style.surfaceStabilityPolicy = sourceStyle.surfaceStabilityPolicy;
    style.uniformSurfaceStabilityMode =
        sourceStyle.uniformSurfaceStabilityMode;
    style.rockSurfaceStabilityMode =
        sourceStyle.rockSurfaceStabilityMode;
    style.sandSurfaceStabilityMode =
        sourceStyle.sandSurfaceStabilityMode;
    style.vegetationSurfaceStabilityMode =
        sourceStyle.vegetationSurfaceStabilityMode;
    style.effectiveSurfaceStabilityMode =
        sourceStyle.effectiveSurfaceStabilityMode;
    style.surfaceStabilityInfluence =
        sourceStyle.surfaceStabilityInfluence;
    style.flowAnimation = false;
    style.waterPathView = false;
    style.waterTrailOverlay = false;
    style.shorelineWaveEnabled = sourceStyle.shorelineWaveEnabled;
    style.shorelineWaveAlgorithm = sourceStyle.shorelineWaveAlgorithm;
    style.shorelineHeightFoam = sourceStyle.shorelineHeightFoam;
    style.shorelineBoundaryZ = sourceStyle.shorelineBoundaryZ;
    style.shorelineHeightReachMeters = sourceStyle.shorelineHeightReachMeters;
    style.shorelineEdgeFadeMeters = sourceStyle.shorelineEdgeFadeMeters;
    style.shorelineDirectionX = sourceStyle.shorelineDirectionX;
    style.shorelineDirectionY = sourceStyle.shorelineDirectionY;
    style.shorelinePatternScale = sourceStyle.shorelinePatternScale;
    style.shorelineWavelengthMeters = sourceStyle.shorelineWavelengthMeters;
    style.shorelineSpeed = sourceStyle.shorelineSpeed;
    style.shorelineWarp = sourceStyle.shorelineWarp;
    style.shorelineTurbulence = sourceStyle.shorelineTurbulence;
    style.shorelineDensity = sourceStyle.shorelineDensity;
    style.shorelineBackgroundWash = sourceStyle.shorelineBackgroundWash;
    style.shorelinePhase = sourceStyle.shorelinePhase;
    style.shorelineIntensity = sourceStyle.shorelineIntensity;
    style.shorelineEmissionAdd = sourceStyle.shorelineEmissionAdd;
    style.shorelineOpacityAdd = sourceStyle.shorelineOpacityAdd;
    style.shorelineOpacityMultiply = sourceStyle.shorelineOpacityMultiply;
    style.shorelinePointSizeAdd = sourceStyle.shorelinePointSizeAdd;
    style.shorelinePointSizeMultiply = sourceStyle.shorelinePointSizeMultiply;
    style.shorelineColourMix = sourceStyle.shorelineColourMix;
    style.shorelineColour = sourceStyle.shorelineColour;
    style.shorelineSeed = sourceStyle.shorelineSeed;
    if (style.screenSpriteSizeMode == PointCloudScreenSpriteSizeMode::WorldMillimeters) {
        invisible_places::style::SetScalarConstant(&style.pointSize, 1.0F);
        style.surfelDiameter = sourceStyle.surfelDiameter;
    } else {
        invisible_places::style::SetScalarConstant(&style.pointSize, 1.0F);
        invisible_places::style::SetScalarConstant(&style.surfelDiameter, kInactiveSurfelDiameterDefault);
    }
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);
    style.colormapPosition = sourceStyle.colorMode == PointCloudColorMode::ScalarColormap
                                 ? sourceStyle.colormapPosition
                                 : style.colormapPosition;
    if (sourceStyle.colorMode != PointCloudColorMode::ScalarColormap) {
        invisible_places::style::SetScalarConstant(&style.colormapPosition, kInactiveColormapPositionDefault);
    }
    return style;
}

PointCloudDensityCompensation ResolvePointCloudDensityCompensation(
    float displaySpacingMeters,
    std::uint64_t displayPointCount,
    float referenceSpacingMeters,
    std::uint64_t referencePointCount) {
    PointCloudDensityCompensation compensation;
    if (std::isfinite(displaySpacingMeters) && displaySpacingMeters > 0.0F) {
        compensation.footprintScale = displaySpacingMeters / 0.001F;
    }

    if (!std::isfinite(displaySpacingMeters) || displaySpacingMeters <= 0.0F ||
        !std::isfinite(referenceSpacingMeters) || referenceSpacingMeters <= 0.0F ||
        displayPointCount == 0U || referencePointCount == 0U) {
        return SanitizePointCloudDensityCompensation(compensation);
    }

    const double spacingRatio =
        static_cast<double>(displaySpacingMeters) / static_cast<double>(referenceSpacingMeters);
    const double relativeCoverage =
        (static_cast<double>(displayPointCount) / static_cast<double>(referencePointCount)) *
        spacingRatio * spacingRatio;
    if (std::isfinite(relativeCoverage) && relativeCoverage > 0.0) {
        const auto areaCorrection = static_cast<float>(std::clamp(
            1.0 / relativeCoverage,
            1.0 / 16.0,
            16.0));
        // An under-covered source (fewer points than its spacing implies)
        // grows its linear footprint so the covered area matches the
        // reference while keeping the authored per-fragment alpha.
        //
        // An over-covered source must never shrink below the nominal spacing
        // ratio: a grid-decimated bundle keeps its nominal pitch regardless of
        // how many cells the irregular reference filled, and a kernel that is
        // narrower than that pitch renders as discrete dots with gaps (the
        // 2026-08-19 Scene3 5 mm "speckle" regression). The residual
        // over-coverage is therefore applied per fragment as alpha so the
        // accumulated coverage still matches the reference.
        if (areaCorrection > 1.0F) {
            compensation.footprintScale *= std::sqrt(areaCorrection);
        } else {
            // A display bundle is never assumed to over-cover its reference by
            // more than 5x. Beyond that the reference is treated as
            // unrepresentative -- Site1's pseudo-1 mm sources are only ~10x the
            // 5 mm counts instead of 25x, which sent per-role alpha to 0.06-0.15
            // and made ROCK/SAND/VEG visibly diverge in the live view while the
            // full-density render kept them cohesive. The floor bounds that
            // divergence and sits below every measured Scene3 correction
            // (0.64/0.68/0.246), so validated Scene3 parity is unchanged.
            compensation.coverageCorrection =
                std::max(areaCorrection, kPointCloudCoverageCorrectionFloor);
        }
    }
    return SanitizePointCloudDensityCompensation(compensation);
}

PointCloudDensityCompensation SanitizePointCloudDensityCompensation(
    PointCloudDensityCompensation compensation) {
    if (!std::isfinite(compensation.footprintScale) || compensation.footprintScale <= 0.0F) {
        compensation.footprintScale = 1.0F;
    }
    if (!std::isfinite(compensation.coverageCorrection) || compensation.coverageCorrection <= 0.0F) {
        compensation.coverageCorrection = 1.0F;
    } else {
        compensation.coverageCorrection = std::clamp(compensation.coverageCorrection, 1.0F / 16.0F, 16.0F);
    }
    return compensation;
}

PointCloudAdaptiveDensityTransition
ResolvePointCloudAdaptiveDensityTransition(
    float projectionScaleY,
    float viewportHeightPixels,
    float coarseSpacingMeters,
    float targetCoarsePixels) {
    PointCloudAdaptiveDensityTransition result;
    if (!std::isfinite(projectionScaleY) || projectionScaleY <= 0.0F ||
        !std::isfinite(viewportHeightPixels) || viewportHeightPixels <= 0.0F ||
        !std::isfinite(coarseSpacingMeters) || coarseSpacingMeters <= 0.0F ||
        !std::isfinite(targetCoarsePixels) || targetCoarsePixels <= 0.0F) {
        return result;
    }

    // Perspective projection: projected diameter in pixels is
    // spacing * projectionY * viewportHeight / (2 * depth).
    const float switchDepth =
        coarseSpacingMeters * projectionScaleY * viewportHeightPixels /
        (2.0F * targetCoarsePixels);
    if (!std::isfinite(switchDepth) || switchDepth <= 0.0F) {
        return result;
    }
    result.startDepthMeters = std::max(0.01F, switchDepth * 0.65F);
    result.switchDepthMeters = std::max(
        result.startDepthMeters,
        switchDepth);
    result.endDepthMeters = std::max(
        result.switchDepthMeters + 0.01F,
        switchDepth * 1.35F);
    result.preparedFineDepthMeters = result.endDepthMeters * 1.30F;
    return result;
}

float PointCloudAdaptiveDensityCoarseWeight(
    float viewDepthMeters,
    const PointCloudAdaptiveDensityTransition& transition) {
    if (!transition.Valid() || !std::isfinite(viewDepthMeters)) {
        return 1.0F;
    }
    const float width = std::max(
        1.0e-6F,
        transition.endDepthMeters - transition.startDepthMeters);
    const float linear = std::clamp(
        (viewDepthMeters - transition.startDepthMeters) / width,
        0.0F,
        1.0F);
    return linear * linear * (3.0F - 2.0F * linear);
}

float PointCloudAdaptiveDensityKeepProbability(
    PointCloudAdaptiveDensityRole role,
    float viewDepthMeters,
    const PointCloudAdaptiveDensityTransition& transition) {
    if (role == PointCloudAdaptiveDensityRole::Disabled ||
        !transition.Valid()) {
        return 1.0F;
    }
    const float coarseWeight = PointCloudAdaptiveDensityCoarseWeight(
        viewDepthMeters,
        transition);
    if (role == PointCloudAdaptiveDensityRole::Coarse) {
        return coarseWeight;
    }
    // An outgoing fine layer only draws its complementary handoff share
    // while a publish crossfade is mid-ramp (a GPU-only runtime blend); at
    // the steady state this function models it has handed everything off.
    if (role == PointCloudAdaptiveDensityRole::FineOutgoing) {
        return 0.0F;
    }
    // Squaring the remaining fine weight reduces vertex work through the
    // middle band more quickly than a linear cross-fade. The coarse points
    // ramp in concurrently, so the visual transition stays covered without
    // carrying roughly half of a 1 mm cloud at the switch depth.
    const float fineWeight = 1.0F - coarseWeight;
    return fineWeight * fineWeight;
}

float ResolvePointCloudDensityAdjustedFootprint(
    float authoredFootprint,
    float antialiasFootprint,
    float postDensityExpansion,
    PointCloudDensityCompensation compensation) {
    const auto finiteOrZero = [](float value) {
        return std::isfinite(value) ? value : 0.0F;
    };
    const auto finiteNonNegative = [](float value) {
        return std::isfinite(value) ? std::max(0.0F, value) : 0.0F;
    };
    compensation = SanitizePointCloudDensityCompensation(compensation);
    // Size additions are intentionally signed. Only the authored kernel is
    // density-scaled; the antialias support is a fixed screen-space margin
    // shared by every display density, so the live coarse bundle and the
    // fine-bundle export pad sprites identically at every camera distance.
    // The geometry-specific lower bound stays with the caller, matching the
    // unified GPU paths' final clamp.
    return finiteOrZero(authoredFootprint) * compensation.footprintScale +
           finiteNonNegative(antialiasFootprint) +
           finiteNonNegative(postDensityExpansion);
}

float ClampPointCloudResolvedSurfelDiameter(
    float resolvedDiameter,
    float maximumDiameter) {
    if (!std::isfinite(resolvedDiameter)) {
        return 0.0F;
    }
    const float safeMaximum = std::isfinite(maximumDiameter)
                                  ? std::max(1.0e-6F, maximumDiameter)
                                  : 1.0e-6F;
    return std::clamp(resolvedDiameter, 0.0F, safeMaximum);
}

PointCloudMaterialVariant ResolvePointCloudMaterialVariant(const PointCloudStyleState& style) {
    return ResolvePointCloudMaterialVariant(style, {});
}

PointCloudMaterialVariant ResolvePointCloudMaterialVariant(
    const PointCloudStyleState& style,
    PointCloudDensityCompensation densityCompensation) {
    return ResolvePointCloudMaterialVariant(
        style,
        densityCompensation,
        false);
}

PointCloudMaterialVariant ResolvePointCloudMaterialVariant(
    const PointCloudStyleState& style,
    PointCloudDensityCompensation densityCompensation,
    bool requiresUnifiedProceduralEffects) {
    if (requiresUnifiedProceduralEffects) {
        return PointCloudMaterialVariant::Unified;
    }
    if (style.gpuBackToFrontSorting || style.depthPrepassEnabled ||
        std::abs(style.depthWeightStrength - 1.0F) > kMaterialEpsilon ||
        style.emissionResponse != PointCloudEmissionResponse::Accumulated ||
        style.normalCullEnabled) {
        return PointCloudMaterialVariant::Unified;
    }
    densityCompensation = SanitizePointCloudDensityCompensation(densityCompensation);
    // A density-adjusted footprint represents multiple reference points. Keep
    // Beauty on the same accumulation path used by EXR and CPU output so a
    // coarse display cannot silently switch to nearest-fragment opaque depth
    // while the final 1 mm source is composited. Fast Basic selects its own
    // explicitly opaque renderer before material resolution.
    if (densityCompensation.footprintScale != 1.0F ||
        densityCompensation.coverageCorrection != 1.0F) {
        return PointCloudMaterialVariant::Unified;
    }

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

    if (style.flowAnimation || style.waterTrailOverlay || style.rainImpactEffects) {
        return PointCloudMaterialVariant::Unified;
    }

    if (PointCloudStyleHasActiveStylisation(style)) {
        return PointCloudMaterialVariant::Unified;
    }

    if (PointCloudStyleHasActiveRoughnessMotion(style)) {
        return PointCloudMaterialVariant::Unified;
    }

    if (style.falloffProfile == PointCloudFalloffProfile::HardDisc &&
        simpleColor &&
        constantPointGeometry &&
        opaqueOpacity &&
        noEmission &&
        noColormapField &&
        noDepthFade &&
        noColorize) {
        return PointCloudMaterialVariant::OpaqueHardDisc;
    }

    if (simpleColor &&
        constantPointGeometry &&
        constantOpacity &&
        constantEmission &&
        noColormapField &&
        noDepthFade) {
        return PointCloudMaterialVariant::ConstantSimple;
    }
    return PointCloudMaterialVariant::Unified;
}

bool PointCloudStyleSupportsPreviewDepthCulling(
    const PointCloudStyleState& style) {
    return ResolvePointCloudMaterialVariant(
               style,
               {},
               false) ==
           PointCloudMaterialVariant::OpaqueHardDisc;
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

bool FrustumPointGridLookup::Matches(
    const std::vector<invisible_places::io::Float3>& positions,
    std::uint32_t requestedDimension) const {
    return positionsIdentity == positions.data() &&
        pointCount == positions.size() &&
        dimension == std::clamp(requestedDimension, 1U, 128U) &&
        pointCellIndices.size() == positions.size();
}

FrustumPointGridLookup BuildFrustumPointGridLookup(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::uint32_t gridDimension,
    std::stop_token stopToken) {
    FrustumPointGridLookup lookup;
    if (positions.empty() || !bounds.valid ||
        positions.size() > std::numeric_limits<std::uint32_t>::max() ||
        stopToken.stop_requested()) {
        return lookup;
    }
    lookup.dimension = std::clamp(gridDimension, 1U, 128U);
    lookup.bounds = bounds;
    lookup.pointCount = positions.size();
    lookup.pointCellIndices.resize(positions.size());
    const float safeXExtent = std::max(
        bounds.maximum.x - bounds.minimum.x,
        1.0e-6F);
    const float safeYExtent = std::max(
        bounds.maximum.y - bounds.minimum.y,
        1.0e-6F);
    const float safeZExtent = std::max(
        bounds.maximum.z - bounds.minimum.z,
        1.0e-6F);
    const auto pointCount = static_cast<std::uint32_t>(positions.size());
    constexpr std::uint32_t kMinimumPointsPerWorker = 1'000'000U;
    const auto hardwareThreads =
        std::max(1U, std::thread::hardware_concurrency());
    const auto workerCount = static_cast<std::size_t>(
        std::clamp<std::uint32_t>(
            pointCount / kMinimumPointsPerWorker,
            1U,
            std::min(hardwareThreads, 8U)));
    const auto pointsPerWorker = static_cast<std::uint32_t>(
        (pointCount + workerCount - 1U) / workerCount);
    const auto buildRange = [&](std::uint32_t begin, std::uint32_t end) {
        for (std::uint32_t pointIndex = begin;
             pointIndex < end;
             ++pointIndex) {
            if ((pointIndex & 4095U) == 0U &&
                stopToken.stop_requested()) {
                return;
            }
            const auto& point = positions[pointIndex];
            const auto x = VoxelCoordinate(
                point.x,
                bounds.minimum.x,
                safeXExtent,
                lookup.dimension);
            const auto y = VoxelCoordinate(
                point.y,
                bounds.minimum.y,
                safeYExtent,
                lookup.dimension);
            const auto z = VoxelCoordinate(
                point.z,
                bounds.minimum.z,
                safeZExtent,
                lookup.dimension);
            lookup.pointCellIndices[pointIndex] =
                static_cast<std::uint32_t>(FrustumCellIndex(
                    x,
                    y,
                    z,
                    lookup.dimension));
        }
    };
    {
        std::vector<std::jthread> workers;
        workers.reserve(workerCount - 1U);
        for (std::size_t worker = 1U; worker < workerCount; ++worker) {
            const auto begin = static_cast<std::uint32_t>(
                std::min<std::uint64_t>(
                    pointCount,
                    static_cast<std::uint64_t>(worker) *
                        pointsPerWorker));
            const auto end = static_cast<std::uint32_t>(
                std::min<std::uint64_t>(
                    pointCount,
                    static_cast<std::uint64_t>(begin) +
                        pointsPerWorker));
            workers.emplace_back([&, begin, end]() {
                buildRange(begin, end);
            });
        }
        buildRange(0U, std::min(pointCount, pointsPerWorker));
    }
    if (stopToken.stop_requested()) {
        return {};
    }
    lookup.positionsIdentity = positions.data();
    return lookup;
}

std::vector<std::uint32_t> GenerateFrustumUnionPointIndices(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::span<const glm::mat4> viewProjections,
    std::uint32_t gridDimension,
    std::stop_token stopToken,
    const FrustumPointGridLookup* pointGridLookup) {
    if (positions.empty() ||
        !bounds.valid ||
        viewProjections.empty() ||
        positions.size() > std::numeric_limits<std::uint32_t>::max() ||
        stopToken.stop_requested()) {
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

    // Each frustum only marks cells inside its own conservative cell range,
    // derived from the world-space corner AABB. A cell outside that padded
    // range cannot intersect the frustum, so skipping it never drops a
    // visible point; when the corners cannot be recovered the frustum
    // conservatively covers the whole grid.
    struct RangedFrustum {
        std::array<FrustumPlane, 6> planes{};
        std::array<std::uint32_t, 3> begin{0U, 0U, 0U};
        std::array<std::uint32_t, 3> end{0U, 0U, 0U};
    };
    const auto cellRangeFor = [&](float minimumValue,
                                  float maximumValue,
                                  float boundsMinimum,
                                  float cellExtent) {
        // Cells are tested with one cell of bloat on each side; padding the
        // range by two cells keeps the skip conservative under float slop.
        const float relativeMinimum = (minimumValue - boundsMinimum) / cellExtent;
        const float relativeMaximum = (maximumValue - boundsMinimum) / cellExtent;
        const auto lastCell = dimension - 1U;
        const std::uint32_t begin =
            !std::isfinite(relativeMinimum) || relativeMinimum < 2.0F
                ? 0U
                : (relativeMinimum - 2.0F >= static_cast<float>(lastCell)
                       ? lastCell
                       : static_cast<std::uint32_t>(relativeMinimum - 2.0F));
        const std::uint32_t end =
            !std::isfinite(relativeMaximum) ||
                    relativeMaximum + 2.0F >= static_cast<float>(lastCell)
                ? lastCell
                : (relativeMaximum + 2.0F <= 0.0F
                       ? 0U
                       : static_cast<std::uint32_t>(relativeMaximum + 2.0F));
        return std::pair{begin, std::max(begin, end)};
    };
    std::vector<RangedFrustum> frustums;
    frustums.reserve(viewProjections.size());
    for (const auto& viewProjection : viewProjections) {
        if (stopToken.stop_requested()) {
            return {};
        }
        RangedFrustum frustum;
        frustum.planes = FrustumPlanes(viewProjection);
        frustum.end = {dimension - 1U, dimension - 1U, dimension - 1U};
        const glm::mat4 inverse = glm::inverse(viewProjection);
        glm::vec3 cornerMinimum{std::numeric_limits<float>::max()};
        glm::vec3 cornerMaximum{std::numeric_limits<float>::lowest()};
        bool cornersValid = true;
        // Depth spans -1..1 and 0..1 clip conventions; the superset stays
        // conservative for either projection style.
        for (const float x : {-1.0F, 1.0F}) {
            for (const float y : {-1.0F, 1.0F}) {
                for (const float z : {-1.0F, 0.0F, 1.0F}) {
                    const glm::vec4 clip = inverse * glm::vec4{x, y, z, 1.0F};
                    if (!std::isfinite(clip.w) ||
                        std::abs(clip.w) <= 1.0e-9F) {
                        cornersValid = false;
                        break;
                    }
                    const glm::vec3 world = glm::vec3{clip} / clip.w;
                    if (!std::isfinite(world.x) ||
                        !std::isfinite(world.y) ||
                        !std::isfinite(world.z)) {
                        cornersValid = false;
                        break;
                    }
                    cornerMinimum = glm::min(cornerMinimum, world);
                    cornerMaximum = glm::max(cornerMaximum, world);
                }
                if (!cornersValid) {
                    break;
                }
            }
            if (!cornersValid) {
                break;
            }
        }
        if (cornersValid) {
            const auto xRange = cellRangeFor(
                cornerMinimum.x, cornerMaximum.x, bounds.minimum.x, xCellExtent);
            const auto yRange = cellRangeFor(
                cornerMinimum.y, cornerMaximum.y, bounds.minimum.y, yCellExtent);
            const auto zRange = cellRangeFor(
                cornerMinimum.z, cornerMaximum.z, bounds.minimum.z, zCellExtent);
            frustum.begin = {xRange.first, yRange.first, zRange.first};
            frustum.end = {xRange.second, yRange.second, zRange.second};
        }
        frustums.push_back(frustum);
    }

    // Frusta are marked concurrently into per-worker cell masks that merge
    // with a commutative OR, so the visible set is deterministic regardless
    // of scheduling. The calling thread always processes the first chunk.
    const auto hardwareThreads =
        std::max(1U, std::thread::hardware_concurrency());
    const auto maskWorkerCount = static_cast<std::size_t>(std::clamp<std::size_t>(
        frustums.size() / 32U,
        1U,
        std::min(hardwareThreads, 8U)));
    std::vector<std::vector<std::uint8_t>> workerCells(
        maskWorkerCount,
        std::vector<std::uint8_t>(cellCount, 0U));
    const auto markFrustumRange = [&](std::size_t frustumBegin,
                                      std::size_t frustumEnd,
                                      std::vector<std::uint8_t>* cells) {
        for (std::size_t frustumIndex = frustumBegin;
             frustumIndex < frustumEnd;
             ++frustumIndex) {
            if (stopToken.stop_requested()) {
                return;
            }
            const auto& frustum = frustums[frustumIndex];
            for (std::uint32_t z = frustum.begin[2]; z <= frustum.end[2]; ++z) {
                if (stopToken.stop_requested()) {
                    return;
                }
                for (std::uint32_t y = frustum.begin[1]; y <= frustum.end[1]; ++y) {
                    for (std::uint32_t x = frustum.begin[0]; x <= frustum.end[0]; ++x) {
                        auto& cell = (*cells)[FrustumCellIndex(x, y, z, dimension)];
                        if (cell != 0U) {
                            continue;
                        }
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
                        if (BoundsIntersectsFrustumPlanes(cellMinimum, cellMaximum, frustum.planes)) {
                            cell = 1U;
                        }
                    }
                }
            }
        }
    };
    {
        std::vector<std::jthread> workers;
        workers.reserve(maskWorkerCount - 1U);
        const auto frustaPerWorker =
            (frustums.size() + maskWorkerCount - 1U) / maskWorkerCount;
        for (std::size_t workerIndex = 1U; workerIndex < maskWorkerCount; ++workerIndex) {
            const auto frustumBegin = std::min(
                frustums.size(),
                workerIndex * frustaPerWorker);
            const auto frustumEnd = std::min(
                frustums.size(),
                frustumBegin + frustaPerWorker);
            workers.emplace_back(
                [&markFrustumRange, frustumBegin, frustumEnd, &workerCells, workerIndex]() {
                    markFrustumRange(frustumBegin, frustumEnd, &workerCells[workerIndex]);
                });
        }
        markFrustumRange(
            0U,
            std::min(frustums.size(), frustaPerWorker),
            &workerCells[0]);
    }
    if (stopToken.stop_requested()) {
        return {};
    }
    auto visibleCells = std::move(workerCells.front());
    for (std::size_t workerIndex = 1U; workerIndex < maskWorkerCount; ++workerIndex) {
        const auto& cells = workerCells[workerIndex];
        for (std::size_t cellIndex = 0; cellIndex < cellCount; ++cellIndex) {
            visibleCells[cellIndex] |= cells[cellIndex];
        }
    }
    const auto visibleCellCount = static_cast<std::size_t>(std::count(
        visibleCells.begin(),
        visibleCells.end(),
        std::uint8_t{1U}));

    if (visibleCellCount == 0) {
        return {};
    }

    // Points are classified in ascending index ranges and concatenated in
    // range order, so the result matches a single-threaded pass exactly.
    const double visibleFraction =
        static_cast<double>(visibleCellCount) / static_cast<double>(visibleCells.size());
    const auto pointCount = static_cast<std::uint32_t>(positions.size());
    constexpr std::uint32_t kMinimumPointsPerClassifyThread = 1'000'000U;
    const auto classifyWorkerCount = static_cast<std::size_t>(std::clamp<std::uint32_t>(
        pointCount / kMinimumPointsPerClassifyThread,
        1U,
        std::min(hardwareThreads, 8U)));
    const bool usePointGridLookup =
        pointGridLookup != nullptr &&
        pointGridLookup->Matches(positions, dimension);
    std::vector<std::vector<std::uint32_t>> workerIndices(classifyWorkerCount);
    const auto classifyRange = [&](std::uint32_t rangeBegin,
                                   std::uint32_t rangeEnd,
                                   std::vector<std::uint32_t>* out) {
        out->reserve(static_cast<std::size_t>(std::min<double>(
            static_cast<double>(rangeEnd - rangeBegin),
            std::max(
                1024.0,
                static_cast<double>(rangeEnd - rangeBegin) * visibleFraction * 1.5))));
        for (std::uint32_t pointIndex = rangeBegin; pointIndex < rangeEnd; ++pointIndex) {
            if ((pointIndex & 4095U) == 0U &&
                stopToken.stop_requested()) {
                return;
            }
            std::size_t cellIndex = 0U;
            if (usePointGridLookup) {
                cellIndex = pointGridLookup->pointCellIndices[pointIndex];
            } else {
                const auto& point = positions[pointIndex];
                const std::uint32_t x = VoxelCoordinate(
                    point.x,
                    bounds.minimum.x,
                    safeXExtent,
                    dimension);
                const std::uint32_t y = VoxelCoordinate(
                    point.y,
                    bounds.minimum.y,
                    safeYExtent,
                    dimension);
                const std::uint32_t z = VoxelCoordinate(
                    point.z,
                    bounds.minimum.z,
                    safeZExtent,
                    dimension);
                cellIndex = FrustumCellIndex(x, y, z, dimension);
            }
            if (cellIndex < visibleCells.size() &&
                visibleCells[cellIndex] != 0U) {
                out->push_back(pointIndex);
            }
        }
    };
    {
        std::vector<std::jthread> workers;
        workers.reserve(classifyWorkerCount - 1U);
        const auto pointsPerWorker = static_cast<std::uint32_t>(
            (pointCount + classifyWorkerCount - 1U) / classifyWorkerCount);
        for (std::size_t workerIndex = 1U; workerIndex < classifyWorkerCount; ++workerIndex) {
            const auto rangeBegin = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                pointCount,
                static_cast<std::uint64_t>(workerIndex) * pointsPerWorker));
            const auto rangeEnd = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                pointCount,
                static_cast<std::uint64_t>(rangeBegin) + pointsPerWorker));
            workers.emplace_back(
                [&classifyRange, rangeBegin, rangeEnd, &workerIndices, workerIndex]() {
                    classifyRange(rangeBegin, rangeEnd, &workerIndices[workerIndex]);
                });
        }
        classifyRange(
            0U,
            std::min(pointCount, pointsPerWorker),
            &workerIndices[0]);
    }
    if (stopToken.stop_requested()) {
        return {};
    }

    std::size_t totalIndexCount = 0;
    for (const auto& ranges : workerIndices) {
        totalIndexCount += ranges.size();
    }
    if (totalIndexCount >= positions.size()) {
        return {};
    }
    auto indices = std::move(workerIndices.front());
    indices.reserve(totalIndexCount);
    for (std::size_t workerIndex = 1U; workerIndex < classifyWorkerCount; ++workerIndex) {
        indices.insert(
            indices.end(),
            workerIndices[workerIndex].begin(),
            workerIndices[workerIndex].end());
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
