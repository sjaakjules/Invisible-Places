#include "renderer/pointcloud/PointCloudLodHierarchy.hpp"

#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <stop_token>
#include <string>
#include <system_error>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::renderer::pointcloud {

namespace {

constexpr std::uint64_t kPointCloudLodCacheMagic = 0x3145434143444f4cULL;
constexpr std::uint32_t kPointCloudLodCacheVersion = 3U;
constexpr std::uint32_t kMaxRepresentativeVoxelDepth = 21U;

struct PointCloudLodCacheHeader {
    std::uint64_t magic = kPointCloudLodCacheMagic;
    std::uint32_t version = kPointCloudLodCacheVersion;
    std::uint32_t headerSize = sizeof(PointCloudLodCacheHeader);
    std::uint64_t sourcePathHash = 0;
    std::uint64_t sourceSizeBytes = 0;
    std::int64_t sourceWriteTimeNs = 0;
    std::uint32_t pointCount = 0;
    std::uint32_t boundsValid = 0;
    float boundsMinimum[3]{};
    float boundsMaximum[3]{};
    std::uint32_t maxLeafSourcePoints = 0;
    std::uint32_t maxDepth = 0;
    std::uint32_t maxInternalRepresentatives = 0;
    std::uint32_t reserved0 = 0;
    std::uint64_t nodeCount = 0;
    std::uint64_t representativeCount = 0;
};

static_assert(std::is_trivially_copyable_v<PointCloudLodNode>);
static_assert(std::is_trivially_copyable_v<PointCloudLodRepresentative>);
static_assert(std::is_trivially_copyable_v<PointCloudLodCacheHeader>);

float DistanceSquared(const invisible_places::io::Float3& left, const invisible_places::io::Float3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return (dx * dx) + (dy * dy) + (dz * dz);
}

invisible_places::io::Float3 BoundsCenter(const invisible_places::io::Bounds3f& bounds) {
    return {
        (bounds.minimum.x + bounds.maximum.x) * 0.5F,
        (bounds.minimum.y + bounds.maximum.y) * 0.5F,
        (bounds.minimum.z + bounds.maximum.z) * 0.5F,
    };
}

float BoundsDiagonal(const invisible_places::io::Bounds3f& bounds) {
    if (!bounds.valid) {
        return 0.0F;
    }
    return std::sqrt(DistanceSquared(bounds.minimum, bounds.maximum));
}

std::uint32_t NearestPointIndex(
    const std::vector<invisible_places::io::Float3>& positions,
    const std::vector<std::uint32_t>& pointIndices,
    const invisible_places::io::Float3& target) {
    std::uint32_t nearest = pointIndices.empty() ? 0U : pointIndices.front();
    float nearestDistance = std::numeric_limits<float>::max();
    for (const auto pointIndex : pointIndices) {
        if (pointIndex >= positions.size()) {
            continue;
        }
        const float distance = DistanceSquared(positions[pointIndex], target);
        if (distance < nearestDistance) {
            nearest = pointIndex;
            nearestDistance = distance;
        }
    }
    return nearest;
}

invisible_places::io::Bounds3f BoundsForIndices(
    const std::vector<invisible_places::io::Float3>& positions,
    const std::vector<std::uint32_t>& pointIndices) {
    invisible_places::io::Bounds3f bounds;
    for (const auto pointIndex : pointIndices) {
        if (pointIndex < positions.size()) {
            bounds.Expand(positions[pointIndex]);
        }
    }
    return bounds;
}

struct RepresentativeVoxelKey {
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    std::uint32_t z = 0;

    [[nodiscard]] bool operator==(const RepresentativeVoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct RepresentativeVoxelKeyHash {
    [[nodiscard]] std::size_t operator()(const RepresentativeVoxelKey& key) const {
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

struct RepresentativeVoxelCandidate {
    RepresentativeVoxelKey key{};
    std::uint32_t sourcePointIndex = 0;
    std::uint32_t representedSourceCount = 0;
    float distanceToCenterSquared = std::numeric_limits<float>::max();
};

struct RepresentativeVoxelConfig {
    float xExtent = 0.0F;
    float yExtent = 0.0F;
    float zExtent = 0.0F;
    bool xActive = false;
    bool yActive = false;
    bool zActive = false;
    std::uint32_t activeDimensions = 0;
};

RepresentativeVoxelConfig MakeRepresentativeVoxelConfig(const invisible_places::io::Bounds3f& bounds) {
    RepresentativeVoxelConfig config;
    if (!bounds.valid) {
        return config;
    }

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

std::uint32_t VoxelDimensionForDepth(std::uint32_t depth, bool activeDimension) {
    if (!activeDimension || depth == 0U) {
        return 1U;
    }
    return 1U << std::min(depth, kMaxRepresentativeVoxelDepth);
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

invisible_places::io::Float3 VoxelCenter(
    const invisible_places::io::Bounds3f& bounds,
    const RepresentativeVoxelKey& key,
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

std::uint64_t RepresentativeMortonKey(const RepresentativeVoxelKey& key, std::uint32_t depth) {
    std::uint64_t morton = 0;
    for (std::uint32_t bitIndex = depth; bitIndex > 0; --bitIndex) {
        const auto shift = bitIndex - 1U;
        morton = (morton << 1U) | ((key.x >> shift) & 1U);
        morton = (morton << 1U) | ((key.y >> shift) & 1U);
        morton = (morton << 1U) | ((key.z >> shift) & 1U);
    }
    return morton;
}

std::uint64_t MixStableLodHash(std::uint64_t value) {
    value ^= value >> 30U;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27U;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31U;
    return value;
}

std::uint64_t ReverseBits64(std::uint64_t value) {
    value = ((value & 0x5555555555555555ULL) << 1U) | ((value >> 1U) & 0x5555555555555555ULL);
    value = ((value & 0x3333333333333333ULL) << 2U) | ((value >> 2U) & 0x3333333333333333ULL);
    value = ((value & 0x0f0f0f0f0f0f0f0fULL) << 4U) | ((value >> 4U) & 0x0f0f0f0f0f0f0f0fULL);
    value = ((value & 0x00ff00ff00ff00ffULL) << 8U) | ((value >> 8U) & 0x00ff00ff00ff00ffULL);
    value = ((value & 0x0000ffff0000ffffULL) << 16U) | ((value >> 16U) & 0x0000ffff0000ffffULL);
    value = (value << 32U) | (value >> 32U);
    return value;
}

std::uint32_t QuantizedRepresentativeCoordinate(float value, float minimum, float maximum) {
    const float extent = maximum - minimum;
    if (extent <= std::numeric_limits<float>::epsilon()) {
        return 0U;
    }
    const auto normalized = std::clamp(
        (static_cast<double>(value) - static_cast<double>(minimum)) / static_cast<double>(extent),
        0.0,
        0.999999999999);
    return static_cast<std::uint32_t>(normalized * 1024.0);
}

std::uint64_t StableRepresentativeRankKey(
    const PointCloudLodNode& node,
    const PointCloudLodRepresentative& representative) {
    RepresentativeVoxelKey key{
        .x = QuantizedRepresentativeCoordinate(representative.position.x, node.bounds.minimum.x, node.bounds.maximum.x),
        .y = QuantizedRepresentativeCoordinate(representative.position.y, node.bounds.minimum.y, node.bounds.maximum.y),
        .z = QuantizedRepresentativeCoordinate(representative.position.z, node.bounds.minimum.z, node.bounds.maximum.z),
    };
    constexpr std::uint32_t kRankMortonDepth = 10U;
    const std::uint64_t morton = RepresentativeMortonKey(key, kRankMortonDepth);
    const std::uint64_t lowDiscrepancySpatialOrder = ReverseBits64(morton << (64U - (kRankMortonDepth * 3U)));
    const std::uint64_t tieBreak =
        MixStableLodHash((static_cast<std::uint64_t>(representative.sourcePointIndex) << 32U) ^
                         static_cast<std::uint64_t>(representative.representedSourceCount));
    return lowDiscrepancySpatialOrder ^ (tieBreak >> 24U);
}

std::uint32_t StableRepresentativeSeed(
    std::uint32_t nodeIndex,
    const PointCloudLodRepresentative& representative,
    std::uint64_t rankKey) {
    const auto mixed = MixStableLodHash(
        rankKey ^
        (static_cast<std::uint64_t>(nodeIndex) << 32U) ^
        static_cast<std::uint64_t>(representative.sourcePointIndex));
    return static_cast<std::uint32_t>(mixed ^ (mixed >> 32U));
}

struct RankedRepresentativeOffset {
    std::uint32_t offset = 0;
    std::uint64_t rankKey = 0;
};

std::vector<RankedRepresentativeOffset> RankedRepresentativeOffsets(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodNode& node,
    std::uint32_t availableCount) {
    std::vector<RankedRepresentativeOffset> ranked;
    ranked.reserve(availableCount);
    for (std::uint32_t offset = 0; offset < availableCount; ++offset) {
        const auto representativeIndex = node.firstRepresentative + offset;
        if (representativeIndex >= hierarchy.representatives.size()) {
            continue;
        }
        ranked.push_back(
            {.offset = offset,
             .rankKey = StableRepresentativeRankKey(node, hierarchy.representatives[representativeIndex])});
    }
    std::sort(
        ranked.begin(),
        ranked.end(),
        [](const RankedRepresentativeOffset& left, const RankedRepresentativeOffset& right) {
            if (left.rankKey != right.rankKey) {
                return left.rankKey < right.rankKey;
            }
            return left.offset < right.offset;
        });
    return ranked;
}

std::uint32_t InitialRepresentativeVoxelDepth(
    std::uint32_t requestedRepresentatives,
    std::uint32_t activeDimensions) {
    if (requestedRepresentatives <= 1U || activeDimensions == 0U) {
        return 0U;
    }

    const double cellsPerAxis = std::pow(
        static_cast<double>(requestedRepresentatives),
        1.0 / static_cast<double>(activeDimensions));
    const double depth = std::ceil(std::log2(std::max(1.0, cellsPerAxis)));
    return static_cast<std::uint32_t>(
        std::clamp<double>(depth, 0.0, static_cast<double>(kMaxRepresentativeVoxelDepth)));
}

std::vector<RepresentativeVoxelCandidate> BuildRepresentativeVoxelCandidates(
    const std::vector<invisible_places::io::Float3>& positions,
    const std::vector<std::uint32_t>& pointIndices,
    const invisible_places::io::Bounds3f& bounds,
    const RepresentativeVoxelConfig& config,
    std::uint32_t depth,
    std::uint32_t requestedRepresentatives) {
    const std::uint32_t xDimension = VoxelDimensionForDepth(depth, config.xActive);
    const std::uint32_t yDimension = VoxelDimensionForDepth(depth, config.yActive);
    const std::uint32_t zDimension = VoxelDimensionForDepth(depth, config.zActive);

    std::unordered_map<RepresentativeVoxelKey, RepresentativeVoxelCandidate, RepresentativeVoxelKeyHash> candidates;
    candidates.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(
        std::max<std::uint64_t>(static_cast<std::uint64_t>(requestedRepresentatives) * 2ULL, 64ULL),
        pointIndices.size())));

    for (const auto pointIndex : pointIndices) {
        if (pointIndex >= positions.size()) {
            continue;
        }
        const auto& point = positions[pointIndex];
        const RepresentativeVoxelKey key{
            .x = VoxelCoordinate(point.x, bounds.minimum.x, config.xExtent, xDimension),
            .y = VoxelCoordinate(point.y, bounds.minimum.y, config.yExtent, yDimension),
            .z = VoxelCoordinate(point.z, bounds.minimum.z, config.zExtent, zDimension),
        };
        const auto center = VoxelCenter(bounds, key, xDimension, yDimension, zDimension);
        const float distanceToCenter = DistanceSquared(point, center);
        auto [candidateIt, inserted] = candidates.emplace(
            key,
            RepresentativeVoxelCandidate{
                .key = key,
                .sourcePointIndex = pointIndex,
                .representedSourceCount = 0U,
                .distanceToCenterSquared = distanceToCenter});
        auto& candidate = candidateIt->second;
        ++candidate.representedSourceCount;
        if (!inserted &&
            (distanceToCenter < candidate.distanceToCenterSquared ||
             (distanceToCenter == candidate.distanceToCenterSquared && pointIndex < candidate.sourcePointIndex))) {
            candidate.sourcePointIndex = pointIndex;
            candidate.distanceToCenterSquared = distanceToCenter;
        }
    }

    std::vector<RepresentativeVoxelCandidate> orderedCandidates;
    orderedCandidates.reserve(candidates.size());
    for (const auto& [key, candidate] : candidates) {
        orderedCandidates.push_back(candidate);
    }
    std::sort(
        orderedCandidates.begin(),
        orderedCandidates.end(),
        [depth](const RepresentativeVoxelCandidate& left, const RepresentativeVoxelCandidate& right) {
            const auto leftKey = RepresentativeMortonKey(left.key, depth);
            const auto rightKey = RepresentativeMortonKey(right.key, depth);
            if (leftKey != rightKey) {
                return leftKey < rightKey;
            }
            return left.sourcePointIndex < right.sourcePointIndex;
        });
    return orderedCandidates;
}

std::vector<PointCloudLodRepresentative> MakeInternalRepresentatives(
    const std::vector<invisible_places::io::Float3>& positions,
    const std::vector<std::uint32_t>& pointIndices,
    const invisible_places::io::Bounds3f& bounds,
    std::uint32_t maxInternalRepresentatives) {
    const auto validPointCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        pointIndices.size(),
        std::numeric_limits<std::uint32_t>::max()));
    if (validPointCount == 0U) {
        return {};
    }

    const std::uint32_t requestedRepresentatives = std::clamp<std::uint32_t>(
        maxInternalRepresentatives,
        1U,
        validPointCount);
    const float sourceSpacing = BoundsDiagonal(bounds) /
                                std::sqrt(static_cast<float>(std::max<std::uint32_t>(1U, validPointCount)));
    const auto config = MakeRepresentativeVoxelConfig(bounds);
    if (config.activeDimensions == 0U) {
        const auto center = BoundsCenter(bounds);
        const auto pointIndex = NearestPointIndex(positions, pointIndices, center);
        return {
            {.sourcePointIndex = pointIndex,
             .representedSourceCount = validPointCount,
             .position = pointIndex < positions.size() ? positions[pointIndex] : center,
             .sourceSpacingMeters = sourceSpacing}};
    }

    auto depth = InitialRepresentativeVoxelDepth(requestedRepresentatives, config.activeDimensions);
    std::vector<RepresentativeVoxelCandidate> bestCandidates;
    while (depth <= kMaxRepresentativeVoxelDepth) {
        auto candidates = BuildRepresentativeVoxelCandidates(
            positions,
            pointIndices,
            bounds,
            config,
            depth,
            requestedRepresentatives);
        if (candidates.empty()) {
            break;
        }

        if (candidates.size() > bestCandidates.size()) {
            bestCandidates = std::move(candidates);
        }
        if (bestCandidates.size() >= requestedRepresentatives ||
            bestCandidates.size() >= pointIndices.size() ||
            depth == kMaxRepresentativeVoxelDepth) {
            break;
        }
        ++depth;
    }

    if (bestCandidates.empty()) {
        const auto center = BoundsCenter(bounds);
        const auto pointIndex = NearestPointIndex(positions, pointIndices, center);
        return {
            {.sourcePointIndex = pointIndex,
             .representedSourceCount = validPointCount,
             .position = pointIndex < positions.size() ? positions[pointIndex] : center,
             .sourceSpacingMeters = sourceSpacing}};
    }

    const auto representativeCount = static_cast<std::size_t>(
        std::min<std::uint32_t>(requestedRepresentatives, static_cast<std::uint32_t>(bestCandidates.size())));
    std::vector<PointCloudLodRepresentative> representatives;
    representatives.reserve(representativeCount);
    if (bestCandidates.size() <= representativeCount) {
        for (const auto& candidate : bestCandidates) {
            representatives.push_back(
                {.sourcePointIndex = candidate.sourcePointIndex,
                 .representedSourceCount = candidate.representedSourceCount,
                 .position = candidate.sourcePointIndex < positions.size()
                                 ? positions[candidate.sourcePointIndex]
                                 : BoundsCenter(bounds),
                 .sourceSpacingMeters = sourceSpacing});
        }
        return representatives;
    }

    const auto candidateCount = static_cast<std::uint64_t>(bestCandidates.size());
    const auto requestedCount = static_cast<std::uint64_t>(representativeCount);
    for (std::uint64_t representativeIndex = 0; representativeIndex < requestedCount; ++representativeIndex) {
        const auto binBegin = (representativeIndex * candidateCount) / requestedCount;
        const auto binEnd = ((representativeIndex + 1U) * candidateCount) / requestedCount;
        const auto clampedBinEnd = std::max<std::uint64_t>(binEnd, binBegin + 1U);
        const auto selectedIndex = std::min<std::uint64_t>(
            candidateCount - 1U,
            binBegin + ((clampedBinEnd - binBegin) / 2U));
        std::uint32_t representedSourceCount = 0;
        for (auto candidateIndex = binBegin; candidateIndex < clampedBinEnd && candidateIndex < candidateCount;
             ++candidateIndex) {
            representedSourceCount += bestCandidates[static_cast<std::size_t>(candidateIndex)].representedSourceCount;
        }
        const auto& selected = bestCandidates[static_cast<std::size_t>(selectedIndex)];
        representatives.push_back(
            {.sourcePointIndex = selected.sourcePointIndex,
             .representedSourceCount = representedSourceCount,
             .position = selected.sourcePointIndex < positions.size()
                             ? positions[selected.sourcePointIndex]
                             : BoundsCenter(bounds),
             .sourceSpacingMeters = sourceSpacing});
    }
    return representatives;
}

std::uint64_t EstimateBuildSourceReferences(
    std::uint64_t sourcePointCount,
    const PointCloudLodBuildConfig& config) {
    if (sourcePointCount == 0U) {
        return 0U;
    }

    const double leafTarget = static_cast<double>(std::max<std::uint32_t>(1U, config.maxLeafSourcePoints));
    const double pointCount = static_cast<double>(sourcePointCount);
    const double octreeLevels =
        pointCount <= leafTarget
            ? 1.0
            : std::ceil(std::log(pointCount / leafTarget) / std::log(8.0)) + 1.0;
    const auto levels = static_cast<std::uint64_t>(
        std::clamp<double>(
            octreeLevels,
            1.0,
            static_cast<double>(std::max<std::uint32_t>(1U, config.maxDepth + 1U))));
    constexpr std::uint64_t kApproximateScansPerLevel = 2ULL;
    if (sourcePointCount > std::numeric_limits<std::uint64_t>::max() /
                               std::max<std::uint64_t>(1ULL, levels * kApproximateScansPerLevel)) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return std::max<std::uint64_t>(1ULL, sourcePointCount * levels * kApproximateScansPerLevel);
}

struct PointCloudLodBuildProgressTracker {
    PointCloudLodBuildProgress progress{};
    PointCloudLodBuildProgressCallback callback;
    std::uint64_t nextReferenceReport = 0;
    std::uint64_t referenceReportStep = 1;

    PointCloudLodBuildProgressTracker(
        std::uint64_t sourcePointCount,
        const PointCloudLodBuildConfig& config,
        PointCloudLodBuildProgressCallback progressCallback)
        : callback(std::move(progressCallback)) {
        progress.sourcePointCount = sourcePointCount;
        progress.estimatedTotalSourceReferences = EstimateBuildSourceReferences(sourcePointCount, config);
        referenceReportStep = std::max<std::uint64_t>(1ULL, progress.estimatedTotalSourceReferences / 256ULL);
        nextReferenceReport = referenceReportStep;
    }

    void Report(bool force = false) {
        if (!callback) {
            return;
        }
        if (!force && progress.processedSourceReferences < nextReferenceReport) {
            return;
        }
        while (progress.processedSourceReferences >= nextReferenceReport) {
            nextReferenceReport += referenceReportStep;
        }
        callback(progress);
    }

    void RecordNode(std::uint64_t sourceReferences, std::uint32_t depth, bool internalNode) {
        ++progress.nodesBuilt;
        progress.currentDepth = depth;
        progress.maxDepthReached = std::max(progress.maxDepthReached, depth);
        const auto weight = internalNode ? 2ULL : 1ULL;
        const auto weightedReferences =
            sourceReferences > std::numeric_limits<std::uint64_t>::max() / weight
                ? std::numeric_limits<std::uint64_t>::max()
                : sourceReferences * weight;
        progress.processedSourceReferences =
            weightedReferences > std::numeric_limits<std::uint64_t>::max() - progress.processedSourceReferences
                ? std::numeric_limits<std::uint64_t>::max()
                : progress.processedSourceReferences + weightedReferences;
        Report();
    }

    void RecordRepresentatives(std::uint64_t count) {
        progress.representativesBuilt += count;
    }

    void Finish() {
        progress.finished = true;
        progress.processedSourceReferences =
            std::max(progress.processedSourceReferences, progress.estimatedTotalSourceReferences);
        Report(true);
    }
};

std::uint32_t BuildNode(
    const std::vector<invisible_places::io::Float3>& positions,
    const std::vector<std::uint32_t>& pointIndices,
    std::uint32_t depth,
    const PointCloudLodBuildConfig& config,
    PointCloudLodHierarchy* hierarchy,
    PointCloudLodBuildProgressTracker* progressTracker) {
    const auto nodeIndex = static_cast<std::uint32_t>(hierarchy->nodes.size());
    hierarchy->nodes.push_back({});
    auto& node = hierarchy->nodes.back();
    node.bounds = BoundsForIndices(positions, pointIndices);
    node.representedSourceCount = static_cast<std::uint32_t>(pointIndices.size());
    node.depth = depth;

    const bool makeLeaf =
        pointIndices.size() <= std::max<std::uint32_t>(1U, config.maxLeafSourcePoints) ||
        depth >= config.maxDepth ||
        !node.bounds.valid ||
        BoundsDiagonal(node.bounds) <= std::numeric_limits<float>::epsilon();
    if (progressTracker != nullptr) {
        progressTracker->RecordNode(pointIndices.size(), depth, !makeLeaf);
    }

    if (makeLeaf) {
        node.firstRepresentative = static_cast<std::uint32_t>(hierarchy->representatives.size());
        node.representativeCount = static_cast<std::uint32_t>(pointIndices.size());
        const float sourceSpacing = pointIndices.empty()
                                        ? 0.0F
                                        : BoundsDiagonal(node.bounds) /
                                              std::sqrt(static_cast<float>(std::max<std::size_t>(1U, pointIndices.size())));
        for (const auto pointIndex : pointIndices) {
            if (pointIndex >= positions.size()) {
                continue;
            }
            hierarchy->representatives.push_back(
                {.sourcePointIndex = pointIndex,
                 .representedSourceCount = 1U,
                 .position = positions[pointIndex],
                 .sourceSpacingMeters = sourceSpacing});
        }
        node.representativeCount =
            static_cast<std::uint32_t>(hierarchy->representatives.size()) - node.firstRepresentative;
        if (progressTracker != nullptr) {
            progressTracker->RecordRepresentatives(node.representativeCount);
        }
        return nodeIndex;
    }

    const auto center = BoundsCenter(node.bounds);
    std::array<std::vector<std::uint32_t>, 8> octants;
    for (const auto pointIndex : pointIndices) {
        if (pointIndex >= positions.size()) {
            continue;
        }
        const auto& point = positions[pointIndex];
        std::uint32_t octant = 0;
        if (point.x >= center.x) {
            octant |= 1U;
        }
        if (point.y >= center.y) {
            octant |= 2U;
        }
        if (point.z >= center.z) {
            octant |= 4U;
        }
        octants[octant].push_back(pointIndex);
    }

    std::uint32_t nonEmptyOctants = 0;
    for (const auto& octant : octants) {
        if (!octant.empty()) {
            ++nonEmptyOctants;
        }
    }
    if (nonEmptyOctants <= 1U) {
        auto leafConfig = config;
        leafConfig.maxDepth = depth;
        hierarchy->nodes.pop_back();
        return BuildNode(positions, pointIndices, depth, leafConfig, hierarchy, progressTracker);
    }

    node.firstRepresentative = static_cast<std::uint32_t>(hierarchy->representatives.size());
    auto representatives = MakeInternalRepresentatives(
        positions,
        pointIndices,
        node.bounds,
        config.maxInternalRepresentatives);
    hierarchy->representatives.insert(
        hierarchy->representatives.end(),
        representatives.begin(),
        representatives.end());
    node.representativeCount =
        static_cast<std::uint32_t>(hierarchy->representatives.size()) - node.firstRepresentative;
    if (progressTracker != nullptr) {
        progressTracker->RecordRepresentatives(node.representativeCount);
    }

    for (const auto& octant : octants) {
        if (octant.empty()) {
            continue;
        }
        const auto childNodeIndex = BuildNode(positions, octant, depth + 1U, config, hierarchy, progressTracker);
        auto& parentNode = hierarchy->nodes[nodeIndex];
        parentNode.childIndices[parentNode.childCount] = childNodeIndex;
        ++parentNode.childCount;
    }
    return nodeIndex;
}

float DensityTargetPixels(invisible_places::output::PointCloudExportDensityMode mode, float overrideTarget) {
    if (overrideTarget > 0.0F) {
        return overrideTarget;
    }
    switch (mode) {
        case invisible_places::output::PointCloudExportDensityMode::FullSource:
            return 0.0F;
        case invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticHighQuality:
            return 0.60F;
        case invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticAsPreview:
            return 0.90F;
        case invisible_places::output::PointCloudExportDensityMode::FastAdaptivePreview:
            return 1.75F;
    }
    return 1.0F;
}

float DensityMaxRepresentativeDiameterPixels(
    invisible_places::output::PointCloudExportDensityMode mode,
    float overrideDiameter) {
    if (overrideDiameter > 0.0F) {
        return overrideDiameter;
    }
    switch (mode) {
        case invisible_places::output::PointCloudExportDensityMode::FullSource:
            return 0.0F;
        case invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticHighQuality:
            return 24.0F;
        case invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticAsPreview:
            return 32.0F;
        case invisible_places::output::PointCloudExportDensityMode::FastAdaptivePreview:
            return 48.0F;
    }
    return 32.0F;
}

std::array<glm::vec3, 8> BoundsCorners(const invisible_places::io::Bounds3f& bounds) {
    return {
        glm::vec3{bounds.minimum.x, bounds.minimum.y, bounds.minimum.z},
        glm::vec3{bounds.maximum.x, bounds.minimum.y, bounds.minimum.z},
        glm::vec3{bounds.minimum.x, bounds.maximum.y, bounds.minimum.z},
        glm::vec3{bounds.maximum.x, bounds.maximum.y, bounds.minimum.z},
        glm::vec3{bounds.minimum.x, bounds.minimum.y, bounds.maximum.z},
        glm::vec3{bounds.maximum.x, bounds.minimum.y, bounds.maximum.z},
        glm::vec3{bounds.minimum.x, bounds.maximum.y, bounds.maximum.z},
        glm::vec3{bounds.maximum.x, bounds.maximum.y, bounds.maximum.z},
    };
}

bool BoundsIntersectsClipFrustum(
    const invisible_places::io::Bounds3f& bounds,
    const glm::mat4& viewProjection) {
    if (!bounds.valid) {
        return true;
    }

    bool allOutsideLeft = true;
    bool allOutsideRight = true;
    bool allOutsideBottom = true;
    bool allOutsideTop = true;
    bool allOutsideNear = true;
    bool allOutsideFar = true;
    for (const auto& corner : BoundsCorners(bounds)) {
        const glm::vec4 clip = viewProjection * glm::vec4{corner, 1.0F};
        allOutsideLeft &= clip.x < -clip.w;
        allOutsideRight &= clip.x > clip.w;
        allOutsideBottom &= clip.y < -clip.w;
        allOutsideTop &= clip.y > clip.w;
        allOutsideNear &= clip.z < -clip.w;
        allOutsideFar &= clip.z > clip.w;
    }

    return !(allOutsideLeft || allOutsideRight || allOutsideBottom ||
             allOutsideTop || allOutsideNear || allOutsideFar);
}

struct ProjectedBoundsFootprint {
    bool visible = false;
    float widthPixels = 0.0F;
    float heightPixels = 0.0F;
    float areaPixels = 0.0F;

    [[nodiscard]] float DiameterPixels() const {
        return std::max(widthPixels, heightPixels);
    }
};

struct RepresentativeCompensation {
    float opacity = 1.0F;
    float emission = 1.0F;
};

struct EmittedRepresentativeFootprint {
    float coverageAreaPixels = 1.0F;
    float renderAreaPixels = 1.0F;
};

ProjectedBoundsFootprint ProjectedBoundsFootprintPixels(
    const invisible_places::io::Bounds3f& bounds,
    const glm::mat4& viewProjection,
    std::uint32_t viewportWidth,
    std::uint32_t viewportHeight) {
    if (!bounds.valid || viewportWidth == 0 || viewportHeight == 0) {
        return {};
    }

    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    bool projectedAny = false;
    for (const auto& corner : BoundsCorners(bounds)) {
        const glm::vec4 clip = viewProjection * glm::vec4{corner, 1.0F};
        if (std::abs(clip.w) <= std::numeric_limits<float>::epsilon()) {
            continue;
        }
        const glm::vec3 ndc = glm::vec3{clip} / clip.w;
        const float pixelX = (ndc.x * 0.5F + 0.5F) * static_cast<float>(viewportWidth);
        const float pixelY = (ndc.y * 0.5F + 0.5F) * static_cast<float>(viewportHeight);
        minX = std::min(minX, pixelX);
        minY = std::min(minY, pixelY);
        maxX = std::max(maxX, pixelX);
        maxY = std::max(maxY, pixelY);
        projectedAny = true;
    }
    if (!projectedAny) {
        return {};
    }

    const float width = std::max(1.0F, maxX - minX);
    const float height = std::max(1.0F, maxY - minY);
    return {
        .visible = true,
        .widthPixels = width,
        .heightPixels = height,
        .areaPixels = width * height,
    };
}

float SourceFootprintAreaPixels(
    const PointCloudLodRepresentative& representative,
    const PointCloudLodTraversalParams& params) {
    static_cast<void>(representative);
    if (params.style.geometryMode == PointCloudGeometryMode::WorldSurfels) {
        const float diameter = std::max(
            {1.0F,
             invisible_places::style::ScalarConstant(params.style.surfelDiameter) * 1000.0F});
        return diameter * diameter;
    }
    const float pointSizePixels = std::max(
        {1.0F,
         invisible_places::style::ScalarConstant(params.style.pointSize)});
    return pointSizePixels * pointSizePixels;
}

EmittedRepresentativeFootprint EmittedRepresentativeFootprintPixels(
    const PointCloudLodRepresentative& representative,
    std::uint32_t representedSourceCount,
    const PointCloudLodNode& node,
    const PointCloudLodTraversalParams& params,
    const ProjectedBoundsFootprint& nodeFootprint,
    float maxRepresentativeDiameterPixels,
    std::uint32_t emittedRepresentativeCount) {
    const float sourceArea = SourceFootprintAreaPixels(representative, params);
    if (!nodeFootprint.visible || maxRepresentativeDiameterPixels <= 0.0F) {
        return {.coverageAreaPixels = sourceArea, .renderAreaPixels = sourceArea};
    }
    if (!node.IsLeaf() && node.representativeCount <= 1U) {
        return {.coverageAreaPixels = sourceArea, .renderAreaPixels = sourceArea};
    }

    const float nodeDiameter = nodeFootprint.DiameterPixels();
    const float cappedDiameter = std::clamp(
        nodeDiameter,
        1.0F,
        std::max(1.0F, maxRepresentativeDiameterPixels));
    const float nodeArea = cappedDiameter * cappedDiameter;
    float coverageArea = nodeArea;
    if (node.representativeCount <= 1U) {
        coverageArea = std::max(sourceArea, nodeArea);
    } else {
        const float representativeArea =
            nodeArea / static_cast<float>(std::max<std::uint32_t>(1U, emittedRepresentativeCount));
        coverageArea = std::max(sourceArea, representativeArea);
    }

    float renderArea = sourceArea;
    if (representedSourceCount > 1U && node.representedSourceCount > 1U) {
        const float sourceDiameter = std::sqrt(std::max(1.0F, sourceArea));
        const float projectedSourceSpacing = std::sqrt(
            std::max(1.0F, nodeFootprint.areaPixels) /
            static_cast<float>(std::max<std::uint32_t>(1U, node.representedSourceCount)));
        if (projectedSourceSpacing <= sourceDiameter) {
            constexpr float kHybridVisibleDiameterScale = 1.35F;
            const float coverageDiameter = std::sqrt(std::max(1.0F, coverageArea));
            const float maxHybridDiameter = std::min(
                std::max(1.0F, maxRepresentativeDiameterPixels),
                sourceDiameter * kHybridVisibleDiameterScale);
            const float renderDiameter = std::clamp(
                coverageDiameter,
                sourceDiameter,
                std::max(sourceDiameter, maxHybridDiameter));
            renderArea = renderDiameter * renderDiameter;
        }
    }

    return {.coverageAreaPixels = coverageArea, .renderAreaPixels = renderArea};
}

RepresentativeCompensation RepresentativeAreaCompensation(
    const PointCloudLodRepresentative& representative,
    std::uint32_t representedSourceCount,
    const PointCloudLodTraversalParams& params,
    float footprintAreaPixels) {
    if (footprintAreaPixels <= std::numeric_limits<float>::epsilon()) {
        return {};
    }
    const float representedArea =
        SourceFootprintAreaPixels(representative, params) *
        static_cast<float>(std::max<std::uint32_t>(1U, representedSourceCount));
    const float rawCompensation = std::max(0.0F, representedArea / footprintAreaPixels);
    float opacityLimit = 1.0F;
    float emissionLimit = 1.0F;
    switch (params.densityMode) {
        case invisible_places::output::PointCloudExportDensityMode::FastAdaptivePreview:
            opacityLimit = 1.5F;
            emissionLimit = 1.0F;
            break;
        case invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticAsPreview:
            opacityLimit = 4.0F;
            emissionLimit = 1.25F;
            break;
        case invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticHighQuality:
            opacityLimit = 8.0F;
            emissionLimit = 1.5F;
            break;
        case invisible_places::output::PointCloudExportDensityMode::FullSource:
            opacityLimit = 1.0F;
            emissionLimit = 1.0F;
            break;
    }
    return {
        .opacity = std::clamp(rawCompensation, 0.0F, opacityLimit),
        .emission = std::clamp(rawCompensation, 0.0F, emissionLimit)};
}

std::uint32_t DefaultRepresentativeBudget(
    const PointCloudLodTraversalParams& params) {
    if (invisible_places::output::PointCloudExportDensityModeUsesFullSource(params.densityMode)) {
        return 0U;
    }

    const auto pixelCount = static_cast<std::uint64_t>(std::max<std::uint32_t>(1U, params.viewportWidth)) *
                            static_cast<std::uint64_t>(std::max<std::uint32_t>(1U, params.viewportHeight));
    std::uint64_t budget = 0;
    switch (params.densityMode) {
        case invisible_places::output::PointCloudExportDensityMode::FastAdaptivePreview:
            budget = pixelCount / 96ULL;
            break;
        case invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticAsPreview:
            budget = pixelCount / 8ULL;
            break;
        case invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticHighQuality:
            budget = pixelCount / 3ULL;
            break;
        case invisible_places::output::PointCloudExportDensityMode::FullSource:
            return 0U;
    }

    budget = std::clamp<std::uint64_t>(budget, 1024ULL, 8'000'000ULL);
    return static_cast<std::uint32_t>(
        std::min<std::uint64_t>(budget, std::numeric_limits<std::uint32_t>::max()));
}

std::uint32_t EffectiveRepresentativeBudget(const PointCloudLodTraversalParams& params) {
    std::uint32_t budget = params.maxRepresentatives;
    if (params.maxDrawItems > 0) {
        budget = budget == 0 ? params.maxDrawItems : std::min(budget, params.maxDrawItems);
    }
    if (budget == 0) {
        budget = DefaultRepresentativeBudget(params);
    }
    return budget;
}

float DefaultFragmentBudget(const PointCloudLodTraversalParams& params) {
    if (invisible_places::output::PointCloudExportDensityModeUsesFullSource(params.densityMode)) {
        return 0.0F;
    }

    const float pixelCount = static_cast<float>(std::max<std::uint32_t>(1U, params.viewportWidth)) *
                             static_cast<float>(std::max<std::uint32_t>(1U, params.viewportHeight));
    switch (params.densityMode) {
        case invisible_places::output::PointCloudExportDensityMode::FastAdaptivePreview:
            return pixelCount * 1.5F;
        case invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticAsPreview:
            return pixelCount * 8.0F;
        case invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality:
        case invisible_places::output::PointCloudExportDensityMode::ArtisticHighQuality:
            return pixelCount * 16.0F;
        case invisible_places::output::PointCloudExportDensityMode::FullSource:
            return 0.0F;
    }
    return pixelCount * 4.0F;
}

struct TraversalBudgetState {
    std::uint32_t maxRepresentatives = 0;
    float maxEstimatedFragments = 0.0F;
    float emittedEstimatedFragments = 0.0F;
    bool representativeBudgetReached = false;
    bool fragmentBudgetReached = false;

    [[nodiscard]] bool CanEmitMore(const std::vector<PointCloudDrawItemGpu>& drawItems) const {
        return maxRepresentatives == 0 || drawItems.size() < maxRepresentatives;
    }

    bool CanEmitFootprint(
        const std::vector<PointCloudDrawItemGpu>& drawItems,
        float footprintAreaPixels) {
        if (!CanEmitMore(drawItems)) {
            representativeBudgetReached = true;
            return false;
        }
        if (maxEstimatedFragments <= 0.0F || drawItems.empty()) {
            return true;
        }
        if (emittedEstimatedFragments + footprintAreaPixels > maxEstimatedFragments) {
            fragmentBudgetReached = true;
            return false;
        }
        return true;
    }
};

bool TraversalCancelled(const std::stop_token& stopToken) {
    return stopToken.stop_requested();
}

std::uint32_t RepresentativeEstimate(
    const PointCloudLodHierarchy& hierarchy,
    std::uint32_t nodeIndex) {
    if (nodeIndex >= hierarchy.nodes.size()) {
        return 0U;
    }
    return std::max<std::uint32_t>(1U, hierarchy.nodes[nodeIndex].representativeCount);
}

std::uint32_t ChildRepresentativeEstimate(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodNode& node) {
    std::uint32_t estimate = 0;
    for (std::uint32_t childOffset = 0; childOffset < node.childCount; ++childOffset) {
        estimate += RepresentativeEstimate(hierarchy, node.childIndices[childOffset]);
    }
    return estimate;
}

struct AdaptiveFrontierNode {
    std::uint32_t nodeIndex = 0;
    ProjectedBoundsFootprint footprint{};
    float projectedSpacingPixels = 0.0F;
    bool violatesTargetSpacing = false;
    bool violatesRepresentativeDiameter = false;
};

struct FrontierEmissionAllocation {
    AdaptiveFrontierNode node{};
    std::uint32_t availableRepresentatives = 0;
    std::uint32_t allocatedRepresentatives = 0;
    float weight = 1.0F;
};

struct FrontierSplitCandidate {
    std::size_t frontierIndex = 0;
    AdaptiveFrontierNode node{};
    std::vector<AdaptiveFrontierNode> children;
    std::uint32_t oldRepresentativeEstimate = 0;
    std::uint32_t replacementRepresentativeEstimate = 0;
    float score = 0.0F;
    bool keptByHysteresis = false;
};

struct FrontierSplitCandidateCompare {
    [[nodiscard]] bool operator()(const FrontierSplitCandidate& left, const FrontierSplitCandidate& right) const {
        if (left.score != right.score) {
            return left.score < right.score;
        }
        return left.node.nodeIndex > right.node.nodeIndex;
    }
};

bool MakeAdaptiveFrontierNode(
    const PointCloudLodHierarchy& hierarchy,
    std::uint32_t nodeIndex,
    const PointCloudLodTraversalParams& params,
    float targetSpacingPixels,
    float maxRepresentativeDiameterPixels,
    AdaptiveFrontierNode* frontierNode) {
    if (frontierNode == nullptr || nodeIndex >= hierarchy.nodes.size()) {
        return false;
    }
    const auto& node = hierarchy.nodes[nodeIndex];
    if (!BoundsIntersectsClipFrustum(node.bounds, params.viewProjection)) {
        return false;
    }
    const auto footprint = ProjectedBoundsFootprintPixels(
        node.bounds,
        params.viewProjection,
        params.viewportWidth,
        params.viewportHeight);
    if (!footprint.visible) {
        return false;
    }

    const float representedCount = static_cast<float>(std::max<std::uint32_t>(1U, node.representedSourceCount));
    const float projectedSpacing = std::sqrt(std::max(1.0F, footprint.areaPixels) / representedCount);
    *frontierNode = {
        .nodeIndex = nodeIndex,
        .footprint = footprint,
        .projectedSpacingPixels = projectedSpacing,
        .violatesTargetSpacing = targetSpacingPixels > 0.0F && projectedSpacing > targetSpacingPixels,
        .violatesRepresentativeDiameter = maxRepresentativeDiameterPixels > 0.0F &&
                                           footprint.DiameterPixels() > maxRepresentativeDiameterPixels,
    };
    return true;
}

std::vector<AdaptiveFrontierNode> MakeVisibleChildFrontierNodes(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodNode& node,
    const PointCloudLodTraversalParams& params,
    float targetSpacingPixels,
    float maxRepresentativeDiameterPixels,
    PointCloudLodTraversalDiagnostics* diagnostics) {
    std::vector<AdaptiveFrontierNode> children;
    children.reserve(node.childCount);
    for (std::uint32_t childOffset = 0; childOffset < node.childCount; ++childOffset) {
        const auto childIndex = node.childIndices[childOffset];
        AdaptiveFrontierNode child;
        if (MakeAdaptiveFrontierNode(
                hierarchy,
                childIndex,
                params,
                targetSpacingPixels,
                maxRepresentativeDiameterPixels,
                &child)) {
            children.push_back(child);
        } else if (diagnostics != nullptr && childIndex < hierarchy.nodes.size()) {
            diagnostics->culledRepresentedSourceCount += hierarchy.nodes[childIndex].representedSourceCount;
        }
    }
    return children;
}

std::uint32_t FrontierRepresentativeEstimate(
    const PointCloudLodHierarchy& hierarchy,
    const std::vector<AdaptiveFrontierNode>& frontier) {
    std::uint32_t estimate = 0;
    for (const auto& node : frontier) {
        estimate += RepresentativeEstimate(hierarchy, node.nodeIndex);
    }
    return estimate;
}

std::uint64_t FrontierRepresentedSourceCount(
    const PointCloudLodHierarchy& hierarchy,
    const std::vector<AdaptiveFrontierNode>& frontier) {
    std::uint64_t count = 0;
    for (const auto& node : frontier) {
        if (node.nodeIndex < hierarchy.nodes.size()) {
            count += hierarchy.nodes[node.nodeIndex].representedSourceCount;
        }
    }
    return count;
}

std::uint64_t DrawItemRepresentedSourceCount(const std::vector<PointCloudDrawItemGpu>& drawItems) {
    std::uint64_t count = 0;
    for (const auto& drawItem : drawItems) {
        count += drawItem.representedSourceCount;
    }
    return count;
}

std::uint32_t FrontierRepresentativeEstimate(
    const PointCloudLodHierarchy& hierarchy,
    const std::vector<std::uint32_t>& frontierNodeIndices) {
    std::uint32_t estimate = 0;
    for (const auto nodeIndex : frontierNodeIndices) {
        estimate += RepresentativeEstimate(hierarchy, nodeIndex);
    }
    return estimate;
}

bool ContainsPreviousFrontierDescendant(
    const PointCloudLodHierarchy& hierarchy,
    std::uint32_t nodeIndex,
    const std::unordered_set<std::uint32_t>& previousFrontier,
    std::unordered_map<std::uint32_t, bool>* cache) {
    if (nodeIndex >= hierarchy.nodes.size()) {
        return false;
    }
    if (previousFrontier.contains(nodeIndex)) {
        return true;
    }
    if (cache != nullptr) {
        if (const auto cached = cache->find(nodeIndex); cached != cache->end()) {
            return cached->second;
        }
    }

    const auto& node = hierarchy.nodes[nodeIndex];
    bool contains = false;
    for (std::uint32_t childOffset = 0; childOffset < node.childCount; ++childOffset) {
        if (ContainsPreviousFrontierDescendant(
                hierarchy,
                node.childIndices[childOffset],
                previousFrontier,
                cache)) {
            contains = true;
            break;
        }
    }
    if (cache != nullptr) {
        (*cache)[nodeIndex] = contains;
    }
    return contains;
}

std::vector<AdaptiveFrontierNode> BuildAdaptiveTraversalFrontier(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodTraversalParams& params,
    float targetSpacingPixels,
    float maxRepresentativeDiameterPixels,
    std::uint32_t maxRepresentatives,
    const std::stop_token& stopToken,
    PointCloudLodTraversalDiagnostics* diagnostics) {
    std::vector<AdaptiveFrontierNode> frontier;
    if (TraversalCancelled(stopToken)) {
        return frontier;
    }
    const float promoteScale = std::max(1.0F, params.hysteresisPromoteScale);
    const float demoteScale = std::clamp(params.hysteresisDemoteScale, 0.05F, promoteScale);
    std::unordered_set<std::uint32_t> previousFrontier{
        params.previousFrontierNodeIndices.begin(),
        params.previousFrontierNodeIndices.end()};
    std::unordered_map<std::uint32_t, bool> previousDescendantCache;
    if (diagnostics != nullptr) {
        diagnostics->previousFrontierNodeCount = static_cast<std::uint32_t>(
            std::min<std::size_t>(previousFrontier.size(), std::numeric_limits<std::uint32_t>::max()));
        diagnostics->hysteresisPromoteScale = promoteScale;
        diagnostics->hysteresisDemoteScale = demoteScale;
    }
    AdaptiveFrontierNode root;
    if (!MakeAdaptiveFrontierNode(
            hierarchy,
            0U,
            params,
            targetSpacingPixels,
            maxRepresentativeDiameterPixels,
            &root)) {
        if (diagnostics != nullptr) {
            diagnostics->culledRepresentedSourceCount += hierarchy.sourcePointCount;
        }
        return frontier;
    }

    frontier.push_back(root);
    std::vector<bool> activeFrontierNodes;
    activeFrontierNodes.push_back(true);
    std::uint32_t estimatedRepresentatives = RepresentativeEstimate(hierarchy, 0U);
    std::uint32_t activeFrontierNodeCount = 1U;
    std::priority_queue<
        FrontierSplitCandidate,
        std::vector<FrontierSplitCandidate>,
        FrontierSplitCandidateCompare>
        splitQueue;

    auto enqueueSplitCandidate = [&](std::size_t frontierIndex) {
        if (TraversalCancelled(stopToken) ||
            frontierIndex >= frontier.size() ||
            frontierIndex >= activeFrontierNodes.size() ||
            !activeFrontierNodes[frontierIndex]) {
            return;
        }
        const auto nodeIndex = frontier[frontierIndex].nodeIndex;
        if (nodeIndex >= hierarchy.nodes.size()) {
            return;
        }
        const auto& node = hierarchy.nodes[nodeIndex];
        if (node.IsLeaf()) {
            return;
        }

        const bool wasPreviouslyRefined =
            !previousFrontier.empty() &&
            ContainsPreviousFrontierDescendant(
                hierarchy,
                nodeIndex,
                previousFrontier,
                &previousDescendantCache);
        const float activeScale = wasPreviouslyRefined ? demoteScale : promoteScale;
        const float promoteSpacingThreshold = targetSpacingPixels * promoteScale;
        const float promoteDiameterThreshold = maxRepresentativeDiameterPixels * promoteScale;
        const float activeSpacingThreshold = targetSpacingPixels * activeScale;
        const float activeDiameterThreshold = maxRepresentativeDiameterPixels * activeScale;
        const bool promoteSpacingViolated =
            targetSpacingPixels > 0.0F && frontier[frontierIndex].projectedSpacingPixels > promoteSpacingThreshold;
        const bool promoteDiameterViolated =
            maxRepresentativeDiameterPixels > 0.0F &&
            frontier[frontierIndex].footprint.DiameterPixels() > promoteDiameterThreshold;
        const bool activeSpacingViolated =
            targetSpacingPixels > 0.0F && frontier[frontierIndex].projectedSpacingPixels > activeSpacingThreshold;
        const bool activeDiameterViolated =
            maxRepresentativeDiameterPixels > 0.0F &&
            frontier[frontierIndex].footprint.DiameterPixels() > activeDiameterThreshold;
        if (!activeSpacingViolated && !activeDiameterViolated) {
            return;
        }
        frontier[frontierIndex].violatesTargetSpacing = activeSpacingViolated;
        frontier[frontierIndex].violatesRepresentativeDiameter = activeDiameterViolated;

        const bool keptByHysteresis =
            wasPreviouslyRefined &&
            !promoteSpacingViolated &&
            !promoteDiameterViolated &&
            (activeSpacingViolated || activeDiameterViolated);

        auto visibleChildren = MakeVisibleChildFrontierNodes(
            hierarchy,
            node,
            params,
            targetSpacingPixels,
            maxRepresentativeDiameterPixels,
            diagnostics);
        const std::uint32_t replacementEstimate = FrontierRepresentativeEstimate(hierarchy, visibleChildren);
        if (replacementEstimate == 0U) {
            return;
        }

        const float spacingScore =
            targetSpacingPixels > 0.0F ? frontier[frontierIndex].projectedSpacingPixels / targetSpacingPixels : 1.0F;
        const float diameterScore =
            maxRepresentativeDiameterPixels > 0.0F
                ? frontier[frontierIndex].footprint.DiameterPixels() / maxRepresentativeDiameterPixels
                : frontier[frontierIndex].footprint.DiameterPixels();
        splitQueue.push(
            FrontierSplitCandidate{
                .frontierIndex = frontierIndex,
                .node = frontier[frontierIndex],
                .children = std::move(visibleChildren),
                .oldRepresentativeEstimate = RepresentativeEstimate(hierarchy, nodeIndex),
                .replacementRepresentativeEstimate = replacementEstimate,
                .score = std::max(spacingScore, diameterScore),
                .keptByHysteresis = keptByHysteresis});
    };

    enqueueSplitCandidate(0U);
    while (!TraversalCancelled(stopToken) && !splitQueue.empty()) {
        auto candidate = splitQueue.top();
        splitQueue.pop();
        if (candidate.frontierIndex >= frontier.size() ||
            candidate.frontierIndex >= activeFrontierNodes.size() ||
            !activeFrontierNodes[candidate.frontierIndex]) {
            continue;
        }

        const std::uint32_t oldEstimate = candidate.oldRepresentativeEstimate;
        const std::uint32_t replacementEstimate = candidate.replacementRepresentativeEstimate;
        const std::uint32_t nextEstimate = replacementEstimate >= oldEstimate
                                               ? estimatedRepresentatives + replacementEstimate - oldEstimate
                                               : estimatedRepresentatives - (oldEstimate - replacementEstimate);
        const auto childCount = static_cast<std::uint32_t>(
            std::min<std::size_t>(candidate.children.size(), std::numeric_limits<std::uint32_t>::max()));
        const auto nextFrontierNodeCount = activeFrontierNodeCount - 1U + childCount;
        const bool keepsVisibleNodeCoverage =
            maxRepresentatives == 0 ||
            (nextFrontierNodeCount <= maxRepresentatives && candidate.node.violatesRepresentativeDiameter);
        if (maxRepresentatives > 0 && nextEstimate > maxRepresentatives && !keepsVisibleNodeCoverage) {
            continue;
        }

        activeFrontierNodes[candidate.frontierIndex] = false;
        if (candidate.keptByHysteresis && diagnostics != nullptr) {
            ++diagnostics->hysteresisKeptNodeCount;
        }
        activeFrontierNodeCount = nextFrontierNodeCount;
        if (replacementEstimate >= oldEstimate) {
            estimatedRepresentatives += replacementEstimate - oldEstimate;
        } else {
            estimatedRepresentatives -= oldEstimate - replacementEstimate;
        }

        for (const auto& child : candidate.children) {
            frontier.push_back(child);
            activeFrontierNodes.push_back(true);
            enqueueSplitCandidate(frontier.size() - 1U);
        }
    }

    if (TraversalCancelled(stopToken)) {
        return {};
    }

    std::vector<AdaptiveFrontierNode> compactFrontier;
    compactFrontier.reserve(activeFrontierNodeCount);
    for (std::size_t index = 0; index < frontier.size() && index < activeFrontierNodes.size(); ++index) {
        if (activeFrontierNodes[index]) {
            compactFrontier.push_back(frontier[index]);
        }
    }
    if (diagnostics != nullptr) {
        diagnostics->frontierNodeIndices.clear();
        diagnostics->frontierNodeIndices.reserve(compactFrontier.size());
        std::unordered_set<std::uint32_t> currentFrontier;
        currentFrontier.reserve(compactFrontier.size());
        for (const auto& node : compactFrontier) {
            diagnostics->frontierNodeIndices.push_back(node.nodeIndex);
            currentFrontier.insert(node.nodeIndex);
            if (!previousFrontier.empty() && !previousFrontier.contains(node.nodeIndex)) {
                ++diagnostics->promotedNodeCount;
            }
        }
        for (const auto previousNodeIndex : previousFrontier) {
            if (!currentFrontier.contains(previousNodeIndex)) {
                ++diagnostics->demotedNodeCount;
            }
        }
        const auto currentRepresentativeEstimate = static_cast<std::int64_t>(
            FrontierRepresentativeEstimate(hierarchy, compactFrontier));
        const auto previousRepresentativeEstimate = static_cast<std::int64_t>(
            FrontierRepresentativeEstimate(hierarchy, params.previousFrontierNodeIndices));
        diagnostics->representativeDelta = currentRepresentativeEstimate - previousRepresentativeEstimate;
    }
    return compactFrontier;
}

void EmitRepresentatives(
    const PointCloudLodHierarchy& hierarchy,
    std::uint32_t nodeIndex,
    const PointCloudLodNode& node,
    const PointCloudLodTraversalParams& params,
    const ProjectedBoundsFootprint& nodeFootprint,
    float maxRepresentativeDiameterPixels,
    std::uint32_t nodeRepresentativeLimit,
    TraversalBudgetState* budget,
    std::vector<PointCloudDrawItemGpu>* drawItems,
    const std::stop_token& stopToken = {}) {
    if (drawItems == nullptr || budget == nullptr) {
        return;
    }
    if (TraversalCancelled(stopToken)) {
        return;
    }
    if (node.representativeCount == 0U || node.firstRepresentative >= hierarchy.representatives.size()) {
        return;
    }

    const auto availableCount = std::min<std::uint32_t>(
        node.representativeCount,
        static_cast<std::uint32_t>(hierarchy.representatives.size() - node.firstRepresentative));
    const std::uint32_t nodeLimit = nodeRepresentativeLimit == 0U
                                        ? availableCount
                                        : std::min(nodeRepresentativeLimit, availableCount);
    const std::uint32_t remainingBudget =
        budget->maxRepresentatives == 0
            ? nodeLimit
            : (drawItems->size() >= budget->maxRepresentatives
                   ? 0U
                   : static_cast<std::uint32_t>(budget->maxRepresentatives - drawItems->size()));
    const std::uint32_t emitCount = std::min(nodeLimit, remainingBudget);
    if (emitCount == 0U) {
        budget->representativeBudgetReached =
            budget->representativeBudgetReached ||
            (budget->maxRepresentatives > 0U && drawItems->size() >= budget->maxRepresentatives);
        return;
    }

    const auto rankedOffsets = RankedRepresentativeOffsets(hierarchy, node, availableCount);
    if (rankedOffsets.empty()) {
        return;
    }

    const auto pushDrawItem = [&](const PointCloudLodRepresentative& representative,
                                  std::uint32_t representedSourceCount,
                                  std::uint64_t rankKey,
                                  std::uint32_t rankOrdinal) {
        const auto footprint = EmittedRepresentativeFootprintPixels(
            representative,
            representedSourceCount,
            node,
            params,
            nodeFootprint,
            maxRepresentativeDiameterPixels,
            emitCount);
        if (!budget->CanEmitFootprint(*drawItems, footprint.coverageAreaPixels)) {
            return false;
        }
        const auto compensation = RepresentativeAreaCompensation(
            representative,
            representedSourceCount,
            params,
            footprint.coverageAreaPixels);
        drawItems->push_back(
            {.sourcePointIndex = representative.sourcePointIndex,
             .representedSourceCount = representedSourceCount,
             .reserved0 = StableRepresentativeSeed(nodeIndex, representative, rankKey),
             .reserved1 =
                 ((node.depth & 0xffU) << 24U) |
                 (availableCount <= 1U
                      ? 0U
                      : static_cast<std::uint32_t>(
                            (static_cast<std::uint64_t>(rankOrdinal) * 0x00ffffffULL) /
                            static_cast<std::uint64_t>(availableCount - 1U))),
             .footprintAreaPixels = footprint.coverageAreaPixels,
             .opacityCompensation = compensation.opacity,
             .emissionCompensation = compensation.emission,
             .renderAreaPixels = footprint.renderAreaPixels});
        budget->emittedEstimatedFragments += footprint.coverageAreaPixels;
        return true;
    };

    if (emitCount >= availableCount) {
        for (std::uint32_t rankOrdinal = 0; rankOrdinal < static_cast<std::uint32_t>(rankedOffsets.size());
             ++rankOrdinal) {
            if (TraversalCancelled(stopToken)) {
                return;
            }
            const auto representativeIndex = node.firstRepresentative + rankedOffsets[rankOrdinal].offset;
            if (representativeIndex >= hierarchy.representatives.size()) {
                continue;
            }
            if (!budget->CanEmitMore(*drawItems)) {
                budget->representativeBudgetReached = true;
                return;
            }
            const auto& representative = hierarchy.representatives[representativeIndex];
            if (!pushDrawItem(
                    representative,
                    representative.representedSourceCount,
                    rankedOffsets[rankOrdinal].rankKey,
                    rankOrdinal)) {
                return;
            }
        }
        return;
    }

    std::vector<std::uint32_t> representedCounts(emitCount, 0U);
    for (std::uint32_t rankOrdinal = 0; rankOrdinal < static_cast<std::uint32_t>(rankedOffsets.size());
         ++rankOrdinal) {
        const auto representativeIndex = node.firstRepresentative + rankedOffsets[rankOrdinal].offset;
        if (representativeIndex >= hierarchy.representatives.size()) {
            continue;
        }
        const auto bucket = rankOrdinal < emitCount ? rankOrdinal : rankOrdinal % emitCount;
        representedCounts[bucket] += hierarchy.representatives[representativeIndex].representedSourceCount;
    }

    for (std::uint32_t emittedIndex = 0; emittedIndex < emitCount; ++emittedIndex) {
        if (TraversalCancelled(stopToken)) {
            return;
        }
        if (!budget->CanEmitMore(*drawItems)) {
            budget->representativeBudgetReached = true;
            return;
        }
        const auto selectedRepresentativeIndex = node.firstRepresentative + rankedOffsets[emittedIndex].offset;
        if (selectedRepresentativeIndex >= hierarchy.representatives.size()) {
            continue;
        }
        if (!pushDrawItem(
                hierarchy.representatives[selectedRepresentativeIndex],
                std::max<std::uint32_t>(1U, representedCounts[emittedIndex]),
                rankedOffsets[emittedIndex].rankKey,
                emittedIndex)) {
            return;
        }
    }
}

void TraverseFullSourceNode(
    const PointCloudLodHierarchy& hierarchy,
    std::uint32_t nodeIndex,
    const PointCloudLodTraversalParams& params,
    TraversalBudgetState* budget,
    std::vector<PointCloudDrawItemGpu>* drawItems,
    const std::stop_token& stopToken) {
    if (nodeIndex >= hierarchy.nodes.size() || drawItems == nullptr || budget == nullptr) {
        return;
    }
    if (TraversalCancelled(stopToken)) {
        return;
    }
    const auto& node = hierarchy.nodes[nodeIndex];
    if (node.IsLeaf()) {
        EmitRepresentatives(hierarchy, nodeIndex, node, params, {}, 0.0F, 0U, budget, drawItems, stopToken);
        return;
    }

    for (std::uint32_t childOffset = 0; childOffset < node.childCount; ++childOffset) {
        TraverseFullSourceNode(
            hierarchy,
            node.childIndices[childOffset],
            params,
            budget,
            drawItems,
            stopToken);
    }
}

void AllocateAdaptiveFrontierRepresentatives(
    const PointCloudLodHierarchy& hierarchy,
    const std::vector<AdaptiveFrontierNode>& frontier,
    std::uint32_t maxRepresentatives,
    std::vector<FrontierEmissionAllocation>* allocations) {
    if (allocations == nullptr) {
        return;
    }
    allocations->clear();
    allocations->reserve(frontier.size());

    std::uint64_t totalAvailable = 0;
    for (const auto& frontierNode : frontier) {
        if (frontierNode.nodeIndex >= hierarchy.nodes.size()) {
            continue;
        }
        const auto available = RepresentativeEstimate(hierarchy, frontierNode.nodeIndex);
        if (available == 0U) {
            continue;
        }
        totalAvailable += available;
        allocations->push_back(
            {.node = frontierNode,
             .availableRepresentatives = available,
             .allocatedRepresentatives = 0U,
             .weight = std::max(1.0F, frontierNode.footprint.areaPixels)});
    }
    if (allocations->empty()) {
        return;
    }

    if (maxRepresentatives == 0U || totalAvailable <= maxRepresentatives) {
        for (auto& allocation : *allocations) {
            allocation.allocatedRepresentatives = allocation.availableRepresentatives;
        }
        return;
    }

    std::sort(
        allocations->begin(),
        allocations->end(),
        [](const FrontierEmissionAllocation& left, const FrontierEmissionAllocation& right) {
            if (left.weight != right.weight) {
                return left.weight > right.weight;
            }
            return left.node.nodeIndex < right.node.nodeIndex;
        });

    std::uint32_t allocated = 0;
    const auto guaranteedNodes = std::min<std::size_t>(allocations->size(), maxRepresentatives);
    for (std::size_t index = 0; index < guaranteedNodes; ++index) {
        (*allocations)[index].allocatedRepresentatives = 1U;
        ++allocated;
    }
    if (allocated >= maxRepresentatives) {
        return;
    }

    float totalWeight = 0.0F;
    for (const auto& allocation : *allocations) {
        if (allocation.allocatedRepresentatives < allocation.availableRepresentatives) {
            totalWeight += allocation.weight;
        }
    }
    if (totalWeight <= 0.0F) {
        return;
    }

    const auto remainingStart = maxRepresentatives - allocated;
    std::uint32_t distributed = 0;
    for (auto& allocation : *allocations) {
        if (allocation.allocatedRepresentatives >= allocation.availableRepresentatives) {
            continue;
        }
        const auto capacity = allocation.availableRepresentatives - allocation.allocatedRepresentatives;
        const auto share = static_cast<std::uint32_t>(
            std::floor((allocation.weight / totalWeight) * static_cast<float>(remainingStart)));
        const auto extra = std::min(capacity, share);
        allocation.allocatedRepresentatives += extra;
        distributed += extra;
    }
    allocated += distributed;

    for (auto& allocation : *allocations) {
        if (allocated >= maxRepresentatives) {
            break;
        }
        if (allocation.allocatedRepresentatives < allocation.availableRepresentatives) {
            ++allocation.allocatedRepresentatives;
            ++allocated;
        }
    }
}

std::string NormalizedCachePathString(const std::filesystem::path& path) {
    return path.lexically_normal().generic_string();
}

bool BoundsEqual(
    const invisible_places::io::Bounds3f& left,
    const invisible_places::io::Bounds3f& right) {
    return left.valid == right.valid &&
           (!left.valid ||
            (left.minimum.x == right.minimum.x &&
             left.minimum.y == right.minimum.y &&
             left.minimum.z == right.minimum.z &&
             left.maximum.x == right.maximum.x &&
             left.maximum.y == right.maximum.y &&
             left.maximum.z == right.maximum.z));
}

PointCloudLodCacheHeader MakeCacheHeader(
    const PointCloudLodCacheSource& source,
    const PointCloudLodBuildConfig& config,
    const PointCloudLodHierarchy& hierarchy) {
    PointCloudLodCacheHeader header;
    header.sourcePathHash = source.normalizedPathHash;
    header.sourceSizeBytes = source.sourceSizeBytes;
    header.sourceWriteTimeNs = source.sourceWriteTimeNs;
    header.pointCount = source.pointCount;
    header.boundsValid = source.bounds.valid ? 1U : 0U;
    header.boundsMinimum[0] = source.bounds.minimum.x;
    header.boundsMinimum[1] = source.bounds.minimum.y;
    header.boundsMinimum[2] = source.bounds.minimum.z;
    header.boundsMaximum[0] = source.bounds.maximum.x;
    header.boundsMaximum[1] = source.bounds.maximum.y;
    header.boundsMaximum[2] = source.bounds.maximum.z;
    header.maxLeafSourcePoints = config.maxLeafSourcePoints;
    header.maxDepth = config.maxDepth;
    header.maxInternalRepresentatives = config.maxInternalRepresentatives;
    header.nodeCount = hierarchy.nodes.size();
    header.representativeCount = hierarchy.representatives.size();
    return header;
}

invisible_places::io::Bounds3f BoundsFromHeader(const PointCloudLodCacheHeader& header) {
    invisible_places::io::Bounds3f bounds;
    bounds.valid = header.boundsValid != 0U;
    bounds.minimum = {header.boundsMinimum[0], header.boundsMinimum[1], header.boundsMinimum[2]};
    bounds.maximum = {header.boundsMaximum[0], header.boundsMaximum[1], header.boundsMaximum[2]};
    return bounds;
}

bool HeaderMatchesSource(
    const PointCloudLodCacheHeader& header,
    const PointCloudLodCacheSource& source,
    const PointCloudLodBuildConfig& config,
    std::string* message) {
    auto reject = [message](std::string text) {
        if (message != nullptr) {
            *message = std::move(text);
        }
        return false;
    };

    if (header.magic != kPointCloudLodCacheMagic || header.version != kPointCloudLodCacheVersion) {
        return reject("LOD cache version mismatch");
    }
    if (header.headerSize != sizeof(PointCloudLodCacheHeader)) {
        return reject("LOD cache header size mismatch");
    }
    if (header.sourcePathHash != source.normalizedPathHash ||
        header.sourceSizeBytes != source.sourceSizeBytes ||
        header.sourceWriteTimeNs != source.sourceWriteTimeNs) {
        return reject("LOD cache source metadata mismatch");
    }
    if (header.pointCount != source.pointCount) {
        return reject("LOD cache point count mismatch");
    }
    if (!BoundsEqual(BoundsFromHeader(header), source.bounds)) {
        return reject("LOD cache bounds mismatch");
    }
    if (header.maxLeafSourcePoints != config.maxLeafSourcePoints ||
        header.maxDepth != config.maxDepth ||
        header.maxInternalRepresentatives != config.maxInternalRepresentatives) {
        return reject("LOD cache build config mismatch");
    }
    return true;
}

}  // namespace

PointCloudLodHierarchy BuildPointCloudLodHierarchy(
    const invisible_places::io::LoadedPointCloud& cloud,
    const PointCloudLodBuildConfig& config,
    const PointCloudLodBuildProgressCallback& progressCallback) {
    PointCloudLodHierarchy hierarchy;
    hierarchy.sourcePointCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(cloud.positions.size(), std::numeric_limits<std::uint32_t>::max()));
    if (hierarchy.sourcePointCount == 0) {
        return hierarchy;
    }

    PointCloudLodBuildProgressTracker progressTracker{
        hierarchy.sourcePointCount,
        config,
        progressCallback};
    progressTracker.Report(true);

    std::vector<std::uint32_t> pointIndices(hierarchy.sourcePointCount);
    std::iota(pointIndices.begin(), pointIndices.end(), 0U);
    BuildNode(cloud.positions, pointIndices, 0U, config, &hierarchy, &progressTracker);
    progressTracker.Finish();
    return hierarchy;
}

std::vector<PointCloudDrawItemGpu> TraversePointCloudLodHierarchy(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodTraversalParams& params,
    std::stop_token stopToken,
    PointCloudLodTraversalDiagnostics* diagnostics) {
    if (diagnostics != nullptr) {
        *diagnostics = {};
    }
    std::vector<PointCloudDrawItemGpu> drawItems;
    if (hierarchy.Empty() || TraversalCancelled(stopToken)) {
        return drawItems;
    }

    const float targetSpacingPixels = DensityTargetPixels(params.densityMode, params.targetProjectedSpacingPixels);
    const float maxRepresentativeDiameterPixels = DensityMaxRepresentativeDiameterPixels(
        params.densityMode,
        params.maxRepresentativeDiameterPixels);
    if (targetSpacingPixels <= 0.0F) {
        drawItems.reserve(hierarchy.sourcePointCount);
    }
    TraversalBudgetState budget{
        .maxRepresentatives = EffectiveRepresentativeBudget(params),
        .maxEstimatedFragments = params.maxEstimatedFragments > 0.0F
                                     ? params.maxEstimatedFragments
                                     : DefaultFragmentBudget(params),
    };

    if (invisible_places::output::PointCloudExportDensityModeUsesFullSource(params.densityMode)) {
        TraverseFullSourceNode(
            hierarchy,
            0U,
            params,
            &budget,
            &drawItems,
            stopToken);
        if (diagnostics != nullptr) {
            diagnostics->visibleFrontierNodeCount = 1U;
            diagnostics->frontierNodeIndices = {0U};
            diagnostics->hysteresisPromoteScale = std::max(1.0F, params.hysteresisPromoteScale);
            diagnostics->hysteresisDemoteScale = std::clamp(
                params.hysteresisDemoteScale,
                0.05F,
                diagnostics->hysteresisPromoteScale);
            diagnostics->visibleFrontierRepresentedSourceCount = hierarchy.sourcePointCount;
            diagnostics->emittedRepresentativeCount = static_cast<std::uint32_t>(drawItems.size());
            diagnostics->emittedRepresentedSourceCount = DrawItemRepresentedSourceCount(drawItems);
            diagnostics->representativeBudgetReached =
                budget.representativeBudgetReached ||
                (budget.maxRepresentatives > 0U && drawItems.size() >= budget.maxRepresentatives);
            diagnostics->fragmentBudgetReached = budget.fragmentBudgetReached;
        }
        return drawItems;
    }

    const auto frontier = BuildAdaptiveTraversalFrontier(
        hierarchy,
        params,
        targetSpacingPixels,
        maxRepresentativeDiameterPixels,
        budget.maxRepresentatives,
        stopToken,
        diagnostics);
    if (TraversalCancelled(stopToken)) {
        return {};
    }
    if (diagnostics != nullptr) {
        diagnostics->visibleFrontierNodeCount = static_cast<std::uint32_t>(
            std::min<std::size_t>(frontier.size(), std::numeric_limits<std::uint32_t>::max()));
        diagnostics->visibleFrontierRepresentedSourceCount =
            FrontierRepresentedSourceCount(hierarchy, frontier);
    }
    TraversalBudgetState emitBudget{
        .maxRepresentatives = budget.maxRepresentatives,
        .maxEstimatedFragments = budget.maxEstimatedFragments,
    };
    std::vector<FrontierEmissionAllocation> allocations;
    AllocateAdaptiveFrontierRepresentatives(
        hierarchy,
        frontier,
        budget.maxRepresentatives,
        &allocations);
    drawItems.reserve(
        budget.maxRepresentatives == 0U
            ? frontier.size()
            : std::min<std::size_t>(allocations.size(), budget.maxRepresentatives));
    for (const auto& allocation : allocations) {
        if (TraversalCancelled(stopToken)) {
            return {};
        }
        if (allocation.node.nodeIndex >= hierarchy.nodes.size() ||
            allocation.allocatedRepresentatives == 0U) {
            continue;
        }
        EmitRepresentatives(
            hierarchy,
            allocation.node.nodeIndex,
            hierarchy.nodes[allocation.node.nodeIndex],
            params,
            allocation.node.footprint,
            maxRepresentativeDiameterPixels,
            allocation.allocatedRepresentatives,
            &emitBudget,
            &drawItems,
            stopToken);
    }
    if (diagnostics != nullptr) {
        diagnostics->emittedRepresentativeCount = static_cast<std::uint32_t>(
            std::min<std::size_t>(drawItems.size(), std::numeric_limits<std::uint32_t>::max()));
        diagnostics->emittedRepresentedSourceCount = DrawItemRepresentedSourceCount(drawItems);
        diagnostics->representativeBudgetReached =
            emitBudget.representativeBudgetReached ||
            (emitBudget.maxRepresentatives > 0U && drawItems.size() >= emitBudget.maxRepresentatives);
        diagnostics->fragmentBudgetReached = emitBudget.fragmentBudgetReached;
    }
    return drawItems;
}

std::vector<PointCloudDrawItemGpu> BuildCoarsePointCloudLodFallbackDrawItems(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodTraversalParams& params,
    std::uint32_t targetRepresentatives) {
    std::vector<PointCloudDrawItemGpu> drawItems;
    if (hierarchy.Empty()) {
        return drawItems;
    }

    const float maxRepresentativeDiameterPixels = DensityMaxRepresentativeDiameterPixels(
        params.densityMode,
        params.maxRepresentativeDiameterPixels);
    std::vector<std::uint32_t> frontier{0U};
    std::uint32_t estimatedRepresentatives = RepresentativeEstimate(hierarchy, 0U);
    const std::uint32_t target = std::max<std::uint32_t>(1U, targetRepresentatives);
    while (estimatedRepresentatives < target) {
        std::size_t bestIndex = frontier.size();
        float bestDiameter = -1.0F;
        for (std::size_t index = 0; index < frontier.size(); ++index) {
            const auto nodeIndex = frontier[index];
            if (nodeIndex >= hierarchy.nodes.size()) {
                continue;
            }
            const auto& node = hierarchy.nodes[nodeIndex];
            if (node.IsLeaf()) {
                continue;
            }
            const std::uint32_t replacementEstimate = ChildRepresentativeEstimate(hierarchy, node);
            const std::uint32_t oldEstimate = RepresentativeEstimate(hierarchy, nodeIndex);
            if (replacementEstimate <= oldEstimate ||
                estimatedRepresentatives + replacementEstimate - oldEstimate > target) {
                continue;
            }
            if (!BoundsIntersectsClipFrustum(node.bounds, params.viewProjection)) {
                continue;
            }
            const auto footprint = ProjectedBoundsFootprintPixels(
                node.bounds,
                params.viewProjection,
                params.viewportWidth,
                params.viewportHeight);
            const float diameter = footprint.visible ? footprint.DiameterPixels() : 0.0F;
            if (diameter > bestDiameter) {
                bestIndex = index;
                bestDiameter = diameter;
            }
        }

        if (bestIndex >= frontier.size()) {
            break;
        }

        const auto nodeIndex = frontier[bestIndex];
        const auto& node = hierarchy.nodes[nodeIndex];
        const std::uint32_t oldEstimate = RepresentativeEstimate(hierarchy, nodeIndex);
        const std::uint32_t replacementEstimate = ChildRepresentativeEstimate(hierarchy, node);
        frontier.erase(frontier.begin() + static_cast<std::ptrdiff_t>(bestIndex));
        for (std::uint32_t childOffset = 0; childOffset < node.childCount; ++childOffset) {
            frontier.push_back(node.childIndices[childOffset]);
        }
        estimatedRepresentatives += replacementEstimate - oldEstimate;
    }

    TraversalBudgetState budget{
        .maxRepresentatives = 0,
        .maxEstimatedFragments = 0.0F,
    };
    drawItems.reserve(estimatedRepresentatives);
    for (const auto nodeIndex : frontier) {
        if (nodeIndex >= hierarchy.nodes.size()) {
            continue;
        }
        const auto& node = hierarchy.nodes[nodeIndex];
        if (!BoundsIntersectsClipFrustum(node.bounds, params.viewProjection)) {
            continue;
        }
        const auto footprint = ProjectedBoundsFootprintPixels(
            node.bounds,
            params.viewProjection,
            params.viewportWidth,
            params.viewportHeight);
        if (!footprint.visible) {
            continue;
        }
        EmitRepresentatives(
            hierarchy,
            nodeIndex,
            node,
            params,
            footprint,
            maxRepresentativeDiameterPixels,
            0U,
            &budget,
            &drawItems);
    }

    return drawItems;
}

std::uint64_t HashPointCloudLodCachePath(const std::filesystem::path& sourcePath) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const char character : NormalizedCachePathString(sourcePath)) {
        hash ^= static_cast<unsigned char>(character);
        hash *= 1099511628211ULL;
    }
    return hash;
}

PointCloudLodCacheSource MakePointCloudLodCacheSource(
    const std::filesystem::path& sourcePath,
    const invisible_places::io::LoadedPointCloud& cloud) {
    PointCloudLodCacheSource source;
    source.sourcePath = sourcePath;
    source.normalizedPathHash = HashPointCloudLodCachePath(sourcePath);
    source.pointCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(cloud.positions.size(), std::numeric_limits<std::uint32_t>::max()));
    source.bounds = cloud.bounds;

    std::error_code error;
    if (std::filesystem::exists(sourcePath, error) && std::filesystem::is_regular_file(sourcePath, error)) {
        source.sourceSizeBytes = std::filesystem::file_size(sourcePath, error);
        const auto writeTime = std::filesystem::last_write_time(sourcePath, error);
        if (!error) {
            source.sourceWriteTimeNs =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    writeTime.time_since_epoch())
                    .count();
        }
    }
    return source;
}

std::filesystem::path BuildPointCloudLodCachePath(
    const std::filesystem::path& cacheDirectory,
    const std::filesystem::path& sourcePath) {
    std::ostringstream hashText;
    hashText << std::hex << std::setfill('0') << std::setw(16) << HashPointCloudLodCachePath(sourcePath);
    const auto stem = sourcePath.stem().empty() ? std::string{"PointCloud"} : sourcePath.stem().string();
    return cacheDirectory / (stem + "-" + hashText.str() + "-PointCloudLodCache-v3.bin");
}

PointCloudLodCacheLoadResult LoadPointCloudLodHierarchyCache(
    const std::filesystem::path& cachePath,
    const PointCloudLodCacheSource& source,
    const PointCloudLodBuildConfig& config) {
    PointCloudLodCacheLoadResult result;
    std::ifstream input{cachePath, std::ios::binary};
    if (!input) {
        result.message = "LOD cache missing";
        return result;
    }

    PointCloudLodCacheHeader header;
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    std::string validationMessage;
    if (!input || !HeaderMatchesSource(header, source, config, &validationMessage)) {
        result.stale = true;
        result.message = validationMessage.empty() ? "LOD cache header invalid" : validationMessage;
        return result;
    }

    if (header.nodeCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) ||
        header.representativeCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
        result.stale = true;
        result.message = "LOD cache payload is too large";
        return result;
    }

    result.hierarchy.sourcePointCount = header.pointCount;
    result.hierarchy.nodes.resize(static_cast<std::size_t>(header.nodeCount));
    result.hierarchy.representatives.resize(static_cast<std::size_t>(header.representativeCount));
    if (!result.hierarchy.nodes.empty()) {
        input.read(
            reinterpret_cast<char*>(result.hierarchy.nodes.data()),
            static_cast<std::streamsize>(result.hierarchy.nodes.size() * sizeof(PointCloudLodNode)));
    }
    if (!result.hierarchy.representatives.empty()) {
        input.read(
            reinterpret_cast<char*>(result.hierarchy.representatives.data()),
            static_cast<std::streamsize>(
                result.hierarchy.representatives.size() * sizeof(PointCloudLodRepresentative)));
    }
    if (!input) {
        result.hierarchy = {};
        result.stale = true;
        result.message = "LOD cache payload truncated";
        return result;
    }

    result.loaded = !result.hierarchy.Empty();
    result.stale = !result.loaded;
    result.message = result.loaded ? "LOD cache ready" : "LOD cache empty";
    return result;
}

bool SavePointCloudLodHierarchyCache(
    const std::filesystem::path& cachePath,
    const PointCloudLodCacheSource& source,
    const PointCloudLodBuildConfig& config,
    const PointCloudLodHierarchy& hierarchy,
    std::string* errorMessage) {
    auto fail = [errorMessage](const std::string& message) {
        if (errorMessage != nullptr) {
            *errorMessage = message;
        }
        return false;
    };

    std::error_code error;
    std::filesystem::create_directories(cachePath.parent_path(), error);
    if (error) {
        return fail("Could not create LOD cache directory: " + error.message());
    }

    auto tempPath = cachePath;
    tempPath += ".tmp";
    {
        std::ofstream output{tempPath, std::ios::binary | std::ios::trunc};
        if (!output) {
            return fail("Could not open temporary LOD cache for writing.");
        }

        const auto header = MakeCacheHeader(source, config, hierarchy);
        output.write(reinterpret_cast<const char*>(&header), sizeof(header));
        if (!hierarchy.nodes.empty()) {
            output.write(
                reinterpret_cast<const char*>(hierarchy.nodes.data()),
                static_cast<std::streamsize>(hierarchy.nodes.size() * sizeof(PointCloudLodNode)));
        }
        if (!hierarchy.representatives.empty()) {
            output.write(
                reinterpret_cast<const char*>(hierarchy.representatives.data()),
                static_cast<std::streamsize>(
                    hierarchy.representatives.size() * sizeof(PointCloudLodRepresentative)));
        }
        if (!output) {
            return fail("Could not write LOD cache payload.");
        }
    }

    std::filesystem::rename(tempPath, cachePath, error);
    if (error) {
        std::filesystem::remove(cachePath, error);
        error.clear();
        std::filesystem::rename(tempPath, cachePath, error);
    }
    if (error) {
        std::filesystem::remove(tempPath, error);
        return fail("Could not publish LOD cache: " + error.message());
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

}  // namespace invisible_places::renderer::pointcloud
