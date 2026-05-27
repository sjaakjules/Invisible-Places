#include "water/WaterFlow.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace invisible_places::water {

namespace {

constexpr glm::vec3 kGravity{0.0F, 0.0F, -1.0F};
constexpr float kNormalEpsilon = 1.0e-6F;

struct SupportPoint {
    std::uint32_t sourceIndex = 0;
    glm::vec3 position{0.0F, 0.0F, 0.0F};
    glm::vec3 normal{0.0F, 0.0F, 0.0F};
    float confidence = 1.0F;
    bool hasNormal = false;
};

struct GridKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey& key) const {
        const auto hx = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x)) * 73856093ULL;
        const auto hy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.y)) * 19349663ULL;
        const auto hz = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.z)) * 83492791ULL;
        return static_cast<std::size_t>(hx ^ hy ^ hz);
    }
};

struct SupportGraph {
    std::vector<SupportPoint> points;
    std::unordered_map<GridKey, std::vector<std::uint32_t>, GridKeyHash> grid;
    float cellSize = 0.1F;
};

struct CandidateScore {
    std::uint32_t supportIndex = 0;
    float score = 0.0F;
    float confidence = 0.0F;
};

struct RankedNeighbour {
    std::uint32_t supportIndex = 0;
    float score = 0.0F;
    float confidence = 0.0F;
    float distance = 0.0F;
    float zDrop = 0.0F;
    float flatness = 0.0F;
    bool bridgeJump = false;
};

struct BranchOpportunity {
    std::uint32_t parentBranchId = 0;
    std::uint32_t fromSupportIndex = 0;
    std::uint32_t startSupportIndex = 0;
    float parentDistance = 0.0F;
    float score = 0.0F;
    float flatness = 0.0F;
    WaterPathBranchRole role = WaterPathBranchRole::Secondary;
};

struct TraceResult {
    WaterPathBranch branch;
    std::vector<std::uint32_t> visitedSupportIndices;
    std::vector<BranchOpportunity> opportunities;
};

glm::vec3 ToGlm(const invisible_places::io::Float3& point) {
    return {point.x, point.y, point.z};
}

invisible_places::io::Float3 FromGlm(const glm::vec3& point) {
    return {point.x, point.y, point.z};
}

glm::vec3 SafeOverlayNormal(glm::vec3 normal) {
    if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z) ||
        glm::dot(normal, normal) <= kNormalEpsilon) {
        return {0.0F, 0.0F, 1.0F};
    }
    return glm::normalize(normal);
}

float Clamp01(float value) {
    return std::clamp(value, 0.0F, 1.0F);
}

float SmoothStep(float edge0, float edge1, float value) {
    const float t = Clamp01((value - edge0) / std::max(1.0e-6F, edge1 - edge0));
    return t * t * (3.0F - 2.0F * t);
}

float PositiveOr(float value, float fallback) {
    return std::isfinite(value) && value > 0.0F ? value : fallback;
}

float TrailPlaybackSampleSpacing(
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings) {
    return std::clamp(
        PositiveOr(animationTrailSettings.trailSampleSpacingMeters, trailShapeSettings.splineAnchorSpacing),
        0.01F,
        25.0F);
}

float SafeLength(const glm::vec3& value) {
    return glm::length(value);
}

bool IsValidPoint(const glm::vec3& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

GridKey MakeGridKey(const glm::vec3& point, float cellSize) {
    const float safeCellSize = std::max(1.0e-4F, cellSize);
    return {
        static_cast<int>(std::floor(point.x / safeCellSize)),
        static_cast<int>(std::floor(point.y / safeCellSize)),
        static_cast<int>(std::floor(point.z / safeCellSize)),
    };
}

std::optional<std::size_t> FindScalarFieldSlot(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::string_view nameNeedle) {
    for (std::size_t index = 0; index < cloud.scalarFields.size(); ++index) {
        const auto& name = cloud.scalarFields[index].name;
        if (name.find(nameNeedle) != std::string::npos) {
            return index;
        }
    }
    return std::nullopt;
}

float ScalarValue(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t scalarSlot,
    std::size_t pointIndex) {
    const auto valueIndex = cloud.ScalarFieldValueIndex(scalarSlot, pointIndex);
    if (valueIndex >= cloud.scalarFieldValues.size()) {
        return 0.0F;
    }
    return cloud.scalarFieldValues[valueIndex];
}

float NormalizedScalarValue(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t scalarSlot,
    std::size_t pointIndex) {
    if (scalarSlot >= cloud.scalarFields.size()) {
        return 0.5F;
    }
    const auto& stats = cloud.scalarFields[scalarSlot];
    if (!stats.valid || std::abs(stats.maximum - stats.minimum) <= 1.0e-6F) {
        return 0.5F;
    }
    return Clamp01((ScalarValue(cloud, scalarSlot, pointIndex) - stats.minimum) / (stats.maximum - stats.minimum));
}

float SupportConfidence(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex,
    std::optional<std::size_t> neighbourSlot) {
    float confidence = 0.72F;
    if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
        const auto normal = ToGlm(cloud.normals[pointIndex]);
        confidence += glm::dot(normal, normal) > kNormalEpsilon ? 0.18F : -0.22F;
    }
    if (neighbourSlot.has_value()) {
        confidence *= 0.35F + (0.65F * NormalizedScalarValue(cloud, neighbourSlot.value(), pointIndex));
    }
    return Clamp01(confidence);
}

std::size_t SampleStride(std::size_t totalCount, std::uint32_t sampleLimit) {
    const auto limit = static_cast<std::size_t>(std::max<std::uint32_t>(1U, sampleLimit));
    if (totalCount <= limit) {
        return 1U;
    }
    return std::max<std::size_t>(1U, (totalCount + limit - 1U) / limit);
}

float EstimatePointSpacing(const invisible_places::io::LoadedPointCloud& cloud) {
    if (cloud.positions.size() < 2U) {
        return 0.0F;
    }

    constexpr std::size_t maxSamples = 768U;
    const std::size_t stride = std::max<std::size_t>(1U, cloud.positions.size() / maxSamples);
    std::vector<glm::vec3> samples;
    samples.reserve(std::min(maxSamples, cloud.positions.size()));
    for (std::size_t index = 0; index < cloud.positions.size() && samples.size() < maxSamples; index += stride) {
        const glm::vec3 point = ToGlm(cloud.positions[index]);
        if (IsValidPoint(point)) {
            samples.push_back(point);
        }
    }
    if (samples.size() < 2U) {
        return 0.0F;
    }

    std::vector<float> nearestDistances;
    nearestDistances.reserve(samples.size());
    for (std::size_t left = 0; left < samples.size(); ++left) {
        float bestDistanceSquared = std::numeric_limits<float>::max();
        for (std::size_t right = 0; right < samples.size(); ++right) {
            if (left == right) {
                continue;
            }
            const glm::vec3 delta = samples[left] - samples[right];
            const float distanceSquared = glm::dot(delta, delta);
            if (distanceSquared > 1.0e-12F) {
                bestDistanceSquared = std::min(bestDistanceSquared, distanceSquared);
            }
        }
        if (bestDistanceSquared < std::numeric_limits<float>::max()) {
            nearestDistances.push_back(std::sqrt(bestDistanceSquared));
        }
    }
    if (nearestDistances.empty()) {
        return 0.0F;
    }

    std::sort(nearestDistances.begin(), nearestDistances.end());
    const auto percentileIndex = static_cast<std::size_t>(
        std::clamp(
            static_cast<float>(nearestDistances.size() - 1U) * 0.35F,
            0.0F,
            static_cast<float>(nearestDistances.size() - 1U)));
    return nearestDistances[percentileIndex];
}

float WorkingAutoTuneSpacing(float estimatedSpacing, WaterScaleMode mode) {
    float spacing = estimatedSpacing > 0.0F ? estimatedSpacing : 0.005F;
    switch (mode) {
        case WaterScaleMode::Detail:
            return 0.005F;
        case WaterScaleMode::Mid:
            return std::clamp(spacing, 0.005F, 0.008F);
        case WaterScaleMode::Aerial:
            return std::clamp(spacing, 0.005F, 10.0F);
    }
    return std::clamp(spacing, 0.001F, 0.008F);
}

WaterPathGenerationSettings TuneWaterPathSettings(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterPathGenerationSettings& requested,
    WaterPathAutoTuneDiagnostics* diagnostics) {
    WaterPathGenerationSettings tuned = requested;
    tuned.supportVoxelSize = std::clamp(PositiveOr(tuned.supportVoxelSize, 0.05F), 0.001F, 20.0F);
    tuned.maxBridgeDistance = std::clamp(PositiveOr(tuned.maxBridgeDistance, tuned.supportVoxelSize * 4.0F), 0.001F, 50.0F);
    tuned.smoothing = std::clamp(tuned.smoothing, 0.0F, 1.0F);
    tuned.pathLength = std::clamp(PositiveOr(tuned.pathLength, 1.0F), 0.05F, 1000.0F);
    tuned.pathSampleSpacing = std::clamp(PositiveOr(tuned.pathSampleSpacing, tuned.supportVoxelSize), 0.001F, 20.0F);
    tuned.branching = std::clamp(tuned.branching, 0.0F, 1.0F);
    tuned.coverage = std::clamp(tuned.coverage, 0.0F, 1.0F);
    tuned.gapTolerance = std::clamp(tuned.gapTolerance, 0.0F, 1.0F);
    tuned.maxSteps = std::clamp<std::uint32_t>(std::max<std::uint32_t>(8U, tuned.maxSteps), 8U, 20000U);
    tuned.supportSampleLimit = std::max<std::uint32_t>(512U, tuned.supportSampleLimit);

    const float estimatedSpacing = EstimatePointSpacing(cloud);
    if (tuned.autoTune) {
        const float spacing = WorkingAutoTuneSpacing(estimatedSpacing, tuned.legacyScaleMode);
        const float coverage = std::clamp(tuned.coverage, 0.0F, 1.0F);
        const float gapTolerance = std::clamp(tuned.gapTolerance, 0.0F, 1.0F);
        const bool detailLikeScale = tuned.legacyScaleMode != WaterScaleMode::Aerial;

        const float denseVoxel =
            detailLikeScale
                ? spacing * (1.15F + (1.0F - coverage) * 0.90F)
                : spacing * (8.0F - coverage * 3.5F);
        const float minimumVoxel = detailLikeScale ? spacing * 1.05F : spacing * 2.0F;
        tuned.supportVoxelSize = std::clamp(
            std::min(tuned.supportVoxelSize, std::max(minimumVoxel, denseVoxel)),
            0.001F,
            20.0F);

        const float occlusionBridge = std::max(
            tuned.supportVoxelSize * (2.0F + gapTolerance * 7.0F),
            spacing * (3.0F + gapTolerance * 9.0F));
        tuned.maxBridgeDistance = std::clamp(
            std::min(tuned.maxBridgeDistance, std::max(tuned.supportVoxelSize * 2.5F, occlusionBridge)),
            tuned.supportVoxelSize * 1.5F,
            50.0F);

        const float denseSpacing =
            detailLikeScale
                ? std::max(spacing * (0.95F + (1.0F - coverage) * 0.75F), tuned.supportVoxelSize * 0.75F)
                : std::max(spacing * (3.5F + (1.0F - coverage) * 2.5F), tuned.supportVoxelSize * 0.55F);
        tuned.pathSampleSpacing = std::clamp(
            std::min(tuned.pathSampleSpacing, denseSpacing),
            0.001F,
            20.0F);

        const float expectedStepLength = std::max(tuned.pathSampleSpacing, tuned.supportVoxelSize * 0.75F);
        const auto neededSteps = static_cast<std::uint32_t>(
            std::ceil(tuned.pathLength / std::max(0.001F, expectedStepLength)) * 1.25F);
        tuned.maxSteps = std::clamp<std::uint32_t>(
            std::max(tuned.maxSteps, neededSteps),
            8U,
            20000U);
    }

    if (diagnostics != nullptr) {
        diagnostics->estimatedPointSpacing = estimatedSpacing;
        diagnostics->supportVoxelSize = tuned.supportVoxelSize;
        diagnostics->maxBridgeDistance = tuned.maxBridgeDistance;
        diagnostics->pathSampleSpacing = tuned.pathSampleSpacing;
        diagnostics->branchSearchRadius = tuned.maxBridgeDistance;
        diagnostics->iterationCount = tuned.autoTune ? 2U : 1U;
        std::ostringstream summary;
        summary << (tuned.autoTune ? "Auto tuned" : "Manual")
                << " spacing=" << estimatedSpacing
                << " working=" << (tuned.autoTune ? WorkingAutoTuneSpacing(estimatedSpacing, tuned.legacyScaleMode) : estimatedSpacing)
                << " voxel=" << tuned.supportVoxelSize
                << " bridge=" << tuned.maxBridgeDistance
                << " sample=" << tuned.pathSampleSpacing;
        diagnostics->summary = summary.str();
    }

    return tuned;
}

SupportGraph BuildSupportGraph(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterBakeSettings& settings) {
    SupportGraph graph;
    graph.cellSize = std::max(settings.supportVoxelSize, settings.maxBridgeDistance);
    graph.cellSize = std::max(0.01F, graph.cellSize);

    const auto stride = SampleStride(cloud.positions.size(), settings.supportSampleLimit);
    graph.points.reserve((cloud.positions.size() + stride - 1U) / stride);
    const auto neighbourSlot = FindScalarFieldSlot(cloud, "Number_of_neighbors");

    for (std::size_t pointIndex = 0; pointIndex < cloud.positions.size(); pointIndex += stride) {
        const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
        if (!IsValidPoint(position)) {
            continue;
        }

        SupportPoint support;
        support.sourceIndex = static_cast<std::uint32_t>(
            std::min<std::size_t>(pointIndex, std::numeric_limits<std::uint32_t>::max()));
        support.position = position;
        support.confidence = SupportConfidence(cloud, pointIndex, neighbourSlot);
        if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
            const glm::vec3 normal = ToGlm(cloud.normals[pointIndex]);
            if (glm::dot(normal, normal) > kNormalEpsilon) {
                support.normal = glm::normalize(normal);
                support.hasNormal = true;
            }
        }
        const auto supportIndex = static_cast<std::uint32_t>(graph.points.size());
        graph.points.push_back(support);
        graph.grid[MakeGridKey(position, graph.cellSize)].push_back(supportIndex);
    }

    return graph;
}

std::vector<std::uint32_t> NearbySupportIndices(
    const SupportGraph& graph,
    const glm::vec3& position,
    float radius) {
    std::vector<std::uint32_t> indices;
    if (graph.points.empty()) {
        return indices;
    }

    const int reach = std::max(1, static_cast<int>(std::ceil(radius / std::max(1.0e-4F, graph.cellSize))));
    const auto baseKey = MakeGridKey(position, graph.cellSize);
    for (int dz = -reach; dz <= reach; ++dz) {
        for (int dy = -reach; dy <= reach; ++dy) {
            for (int dx = -reach; dx <= reach; ++dx) {
                const GridKey key{baseKey.x + dx, baseKey.y + dy, baseKey.z + dz};
                const auto it = graph.grid.find(key);
                if (it == graph.grid.end()) {
                    continue;
                }
                indices.insert(indices.end(), it->second.begin(), it->second.end());
            }
        }
    }
    return indices;
}

std::optional<std::uint32_t> NearestSupportIndex(
    const SupportGraph& graph,
    const glm::vec3& position,
    float searchRadius) {
    if (graph.points.empty()) {
        return std::nullopt;
    }

    auto nearby = NearbySupportIndices(graph, position, std::max(searchRadius, graph.cellSize * 2.0F));
    if (nearby.empty()) {
        nearby.resize(graph.points.size());
        for (std::uint32_t index = 0; index < graph.points.size(); ++index) {
            nearby[index] = index;
        }
    }

    std::optional<std::uint32_t> bestIndex;
    float bestDistanceSquared = std::numeric_limits<float>::max();
    for (const auto index : nearby) {
        if (index >= graph.points.size()) {
            continue;
        }
        const float distanceSquared = glm::dot(graph.points[index].position - position, graph.points[index].position - position);
        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            bestIndex = index;
        }
    }
    return bestIndex;
}

glm::vec3 FlowDirection(const SupportPoint& point) {
    if (!point.hasNormal) {
        return kGravity;
    }
    glm::vec3 direction = kGravity - (point.normal * glm::dot(kGravity, point.normal));
    if (glm::dot(direction, direction) <= kNormalEpsilon) {
        direction = kGravity;
    }
    return glm::normalize(direction);
}

std::vector<RankedNeighbour> RankDownhillNeighbours(
    const SupportGraph& graph,
    std::uint32_t currentIndex,
    const WaterBakeSettings& settings,
    const std::vector<std::uint32_t>& visited,
    const std::unordered_set<std::uint32_t>* occupied = nullptr) {
    std::vector<RankedNeighbour> ranked;
    if (currentIndex >= graph.points.size()) {
        return ranked;
    }

    const auto& current = graph.points[currentIndex];
    const auto direction = FlowDirection(current);
    const float searchRadius = std::max(0.001F, settings.maxBridgeDistance);
    const float gapTolerance = std::clamp(settings.gapTolerance, 0.0F, 1.0F);
    const float preferredStep = std::max(
        std::max(0.001F, settings.pathSampleSpacing),
        std::max(0.001F, settings.supportVoxelSize));
    const float bridgeStart = std::min(
        searchRadius,
        std::max(preferredStep * 1.65F, settings.supportVoxelSize * 1.85F));
    const float softBridgeLimit = std::min(
        searchRadius,
        std::max(
            preferredStep * (1.65F + gapTolerance * 6.50F),
            settings.supportVoxelSize * (1.85F + gapTolerance * 5.75F)));
    const auto candidates = NearbySupportIndices(graph, current.position, searchRadius);

    for (const auto candidateIndex : candidates) {
        if (candidateIndex == currentIndex || candidateIndex >= graph.points.size()) {
            continue;
        }
        if (std::find(visited.begin(), visited.end(), candidateIndex) != visited.end()) {
            continue;
        }
        if (occupied != nullptr && occupied->contains(candidateIndex)) {
            continue;
        }

        const auto& candidate = graph.points[candidateIndex];
        const glm::vec3 delta = candidate.position - current.position;
        const float distance = SafeLength(delta);
        if (distance <= 1.0e-5F || distance > searchRadius) {
            continue;
        }

        const float zDrop = current.position.z - candidate.position.z;
        const float alignment = glm::dot(glm::normalize(delta), direction);
        const float downhillScore = zDrop / std::max(0.02F, distance);
        const float uphillPenalty = zDrop < -settings.maxBridgeDistance * 0.12F ? std::abs(zDrop) * 8.0F : 0.0F;
        const float bridgeAmount = SmoothStep(bridgeStart, searchRadius, distance);
        const float beyondSoftBridge = SmoothStep(softBridgeLimit, searchRadius, distance);
        const float bridgePenalty =
            (distance / std::max(0.01F, searchRadius)) * 0.55F +
            bridgeAmount * (0.30F + (1.0F - gapTolerance) * 1.05F) +
            beyondSoftBridge * (2.35F - gapTolerance * 1.45F);
        const float normalCoherence = current.hasNormal && candidate.hasNormal
                                          ? Clamp01((glm::dot(current.normal, candidate.normal) + 1.0F) * 0.5F)
                                          : 0.55F;
        const float flatness = Clamp01(1.0F - (std::abs(zDrop) / std::max(0.01F, settings.maxBridgeDistance * 0.35F)));
        const bool bridgeJump = distance > bridgeStart;
        const bool longBridgeJump = distance > softBridgeLimit;
        if (longBridgeJump && gapTolerance < 0.18F) {
            continue;
        }
        const float bridgeConfidence =
            bridgeJump
                ? std::clamp(0.52F + gapTolerance * 0.34F - beyondSoftBridge * 0.18F, 0.28F, 0.92F)
                : 1.0F;
        const float score =
            (downhillScore * 3.2F) +
            (alignment * 1.9F) +
            (candidate.confidence * 1.6F) +
            (normalCoherence * 0.55F) -
            bridgePenalty -
            uphillPenalty +
            (flatness * std::clamp(settings.branching, 0.0F, 1.0F) * 0.22F);
        const float acceptanceThreshold = 1.45F + beyondSoftBridge * (1.35F - gapTolerance * 0.85F);
        if (zDrop > -settings.maxBridgeDistance * 0.08F || score > acceptanceThreshold) {
            ranked.push_back({
                .supportIndex = candidateIndex,
                .score = score,
                .confidence = Clamp01(candidate.confidence * bridgeConfidence),
                .distance = distance,
                .zDrop = zDrop,
                .flatness = flatness,
                .bridgeJump = bridgeJump,
            });
        }
    }

    std::sort(
        ranked.begin(),
        ranked.end(),
        [](const RankedNeighbour& left, const RankedNeighbour& right) {
            if (std::abs(left.score - right.score) > 1.0e-6F) {
                return left.score > right.score;
            }
            return left.supportIndex < right.supportIndex;
        });
    return ranked;
}

void IncludeOverlayPoint(WaterOverlay* overlay, WaterOverlayPoint point) {
    if (overlay == nullptr) {
        return;
    }
    point.normal = FromGlm(SafeOverlayNormal(ToGlm(point.normal)));
    overlay->bounds.Expand(point.position);
    overlay->points.push_back(point);
}

float Hash01(std::uint32_t value);
std::uint8_t FloatToByte(float value);
WaterOverlayPoint BlendPathAnchor(
    const WaterOverlayPoint& left,
    const WaterOverlayPoint& right,
    float amount);

std::uint32_t PackRgba8(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
    return static_cast<std::uint32_t>(red) |
           (static_cast<std::uint32_t>(green) << 8U) |
           (static_cast<std::uint32_t>(blue) << 16U) |
           (0xFFU << 24U);
}

glm::vec3 CatmullRomPosition(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& p2,
    const glm::vec3& p3,
    float amount) {
    const float t = Clamp01(amount);
    const float t2 = t * t;
    const float t3 = t2 * t;
    return 0.5F * (
        (2.0F * p1) +
        (-p0 + p2) * t +
        ((2.0F * p0) - (5.0F * p1) + (4.0F * p2) - p3) * t2 +
        (-p0 + (3.0F * p1) - (3.0F * p2) + p3) * t3);
}

void RecomputePathDistances(
    std::vector<WaterOverlayPoint>* path,
    float normalizationLength) {
    if (path == nullptr || path->empty()) {
        return;
    }

    float distance = 0.0F;
    (*path)[0].pathDistance = 0.0F;
    (*path)[0].accumulation = 0.0F;
    for (std::size_t index = 1U; index < path->size(); ++index) {
        distance += SafeLength(ToGlm((*path)[index].position) - ToGlm((*path)[index - 1U].position));
        auto& point = (*path)[index];
        point.pathDistance = distance;
        point.accumulation = Clamp01(distance / std::max(0.001F, normalizationLength));
        point.phase = std::fmod((distance * 0.37F) + (point.emitterId * 0.173F), 1.0F);
    }
}

void SmoothWaterPath(
    std::vector<WaterOverlayPoint>* path,
    float smoothing,
    float normalizationLength) {
    if (path == nullptr || path->size() < 3U) {
        return;
    }

    const float clampedSmoothing = Clamp01(smoothing);
    if (clampedSmoothing <= 1.0e-4F) {
        RecomputePathDistances(path, normalizationLength);
        return;
    }

    const std::uint32_t iterations =
        1U + static_cast<std::uint32_t>(std::round(clampedSmoothing * 5.0F));
    const float amount = 0.12F + clampedSmoothing * 0.48F;
    std::vector<WaterOverlayPoint> working = *path;
    for (std::uint32_t iteration = 0; iteration < iterations; ++iteration) {
        auto next = working;
        for (std::size_t index = 1U; index + 1U < working.size(); ++index) {
            const glm::vec3 previous = ToGlm(working[index - 1U].position);
            const glm::vec3 current = ToGlm(working[index].position);
            const glm::vec3 following = ToGlm(working[index + 1U].position);
            const glm::vec3 smoothed = glm::mix(current, (previous + following) * 0.5F, amount);
            next[index].position = FromGlm(smoothed);
        }
        working = std::move(next);
    }

    *path = std::move(working);
    RecomputePathDistances(path, normalizationLength);
}

std::vector<WaterOverlayPoint> BuildSplineViewSamples(
    const std::vector<WaterOverlayPoint>& anchors,
    float sampleSpacingMeters) {
    if (anchors.size() < 3U) {
        return anchors;
    }

    constexpr std::size_t maxSampleCount = 32768U;
    std::vector<WaterOverlayPoint> samples;
    samples.reserve(std::min<std::size_t>(maxSampleCount, anchors.size() * 4U));
    samples.push_back(anchors.front());
    for (std::size_t index = 0U; index + 1U < anchors.size() && samples.size() < maxSampleCount; ++index) {
        const auto& p0 = anchors[index > 0U ? index - 1U : index];
        const auto& p1 = anchors[index];
        const auto& p2 = anchors[index + 1U];
        const auto& p3 = anchors[index + 2U < anchors.size() ? index + 2U : index + 1U];
        const float segmentLength = std::max(0.001F, p2.pathDistance - p1.pathDistance);
        const bool explicitSpacing = std::isfinite(sampleSpacingMeters) && sampleSpacingMeters > 0.0F;
        const float spacing = PositiveOr(sampleSpacingMeters, std::max(0.03F, segmentLength * 0.25F));
        const std::uint32_t subdivisions = std::clamp<std::uint32_t>(
            static_cast<std::uint32_t>(std::ceil(segmentLength / std::max(0.001F, spacing))),
            explicitSpacing ? 1U : 3U,
            12U);
        for (std::uint32_t step = 1U; step <= subdivisions && samples.size() < maxSampleCount; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(subdivisions);
            auto sample = BlendPathAnchor(p1, p2, t);
            sample.position = FromGlm(CatmullRomPosition(
                ToGlm(p0.position),
                ToGlm(p1.position),
                ToGlm(p2.position),
                ToGlm(p3.position),
                t));
            samples.push_back(sample);
        }
    }
    return samples;
}

void IncludeWaterPathViewAnchors(
    WaterOverlay* overlay,
    const std::vector<WaterOverlayPoint>& path) {
    if (overlay == nullptr || path.empty()) {
        return;
    }

    const auto pathStartIndex = static_cast<std::uint32_t>(
        std::min<std::size_t>(overlay->points.size(), std::numeric_limits<std::uint32_t>::max()));
    const auto pathPointCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(path.size(), std::numeric_limits<std::uint32_t>::max()));
    for (auto anchor : path) {
        anchor.particleRole = 2.0F;
        anchor.pathStartIndex = static_cast<float>(pathStartIndex);
        anchor.pathPointCount = static_cast<float>(pathPointCount);
        anchor.jitterSeed = Hash01(
            static_cast<std::uint32_t>(anchor.flowId * 4099.0F + anchor.pathDistance * 6553.0F));
        IncludeOverlayPoint(overlay, anchor);
    }
}

glm::vec3 WaterPathTangent(
    const std::vector<WaterOverlayPoint>& path,
    std::size_t index) {
    if (path.size() < 2U) {
        return {0.0F, 0.0F, -1.0F};
    }
    const auto previous = index > 0U ? index - 1U : index;
    const auto next = std::min<std::size_t>(index + 1U, path.size() - 1U);
    glm::vec3 tangent = ToGlm(path[next].position) - ToGlm(path[previous].position);
    if (glm::dot(tangent, tangent) <= 1.0e-8F) {
        tangent = {0.0F, 0.0F, -1.0F};
    }
    return glm::normalize(tangent);
}

glm::vec3 WaterPathLateral(const glm::vec3& tangent) {
    glm::vec3 lateral = glm::cross(tangent, glm::vec3{0.0F, 0.0F, 1.0F});
    if (glm::dot(lateral, lateral) <= 1.0e-8F) {
        lateral = glm::cross(tangent, glm::vec3{0.0F, 1.0F, 0.0F});
    }
    if (glm::dot(lateral, lateral) <= 1.0e-8F) {
        lateral = {1.0F, 0.0F, 0.0F};
    }
    return glm::normalize(lateral);
}

float FallbackPathTurbulence(
    const std::vector<WaterOverlayPoint>& path,
    std::size_t index) {
    if (path.size() < 2U) {
        return 0.0F;
    }

    const auto previous = index > 0U ? index - 1U : index;
    const auto next = std::min<std::size_t>(index + 1U, path.size() - 1U);
    const glm::vec3 previousPosition = ToGlm(path[previous].position);
    const glm::vec3 currentPosition = ToGlm(path[index].position);
    const glm::vec3 nextPosition = ToGlm(path[next].position);
    const float distance = SafeLength(nextPosition - previousPosition);
    const float slope = distance > 1.0e-5F
                            ? std::abs(previousPosition.z - nextPosition.z) / distance
                            : 0.0F;
    float curvature = 0.0F;
    if (previous != index && next != index) {
        const glm::vec3 incoming = currentPosition - previousPosition;
        const glm::vec3 outgoing = nextPosition - currentPosition;
        if (glm::dot(incoming, incoming) > 1.0e-8F && glm::dot(outgoing, outgoing) > 1.0e-8F) {
            curvature = 1.0F - Clamp01((glm::dot(glm::normalize(incoming), glm::normalize(outgoing)) + 1.0F) * 0.5F);
        }
    }
    return Clamp01(std::max(slope, curvature * 1.4F));
}

struct TrailSurfaceSample {
    glm::vec3 position{0.0F};
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
    float xyDistance = 0.0F;
    float minZ = 0.0F;
    float maxZ = 0.0F;
    float confidence = 1.0F;
};

struct TrailProjectionResult {
    WaterOverlayPoint point{};
    bool projected = false;
};

enum class TrailSurfaceSampleMode {
    Projection,
    Constraint
};

struct WaterTrailGuidePath {
    std::vector<WaterOverlayPoint> points;
    std::uint32_t startIndex = 0U;
    std::uint32_t pointCount = 0U;
};

struct TrailSurfaceCellAccumulator {
    glm::vec3 positionSum{0.0F};
    glm::vec3 normalSum{0.0F};
    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();
    std::uint32_t count = 0;
    std::uint32_t normalCount = 0;
};

std::uint64_t EncodeTrailSurfaceGridKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
}

GridKey MakeXyGridKey(const glm::vec3& point, float cellSize) {
    const float safeCellSize = std::max(1.0e-4F, cellSize);
    return {
        static_cast<int>(std::floor(point.x / safeCellSize)),
        static_cast<int>(std::floor(point.y / safeCellSize)),
        0,
    };
}

std::optional<TrailSurfaceSample> SampleTrailSurface(
    const TrailSurfaceIndex& surfaceIndex,
    const glm::vec3& position,
    float radiusScale = 1.0F,
    TrailSurfaceSampleMode mode = TrailSurfaceSampleMode::Projection) {
    if (surfaceIndex.cells.empty()) {
        return std::nullopt;
    }

    const float searchRadius = std::max(0.001F, surfaceIndex.searchRadius * std::max(0.25F, radiusScale));
    const int reach = std::max(1, static_cast<int>(std::ceil(searchRadius / std::max(1.0e-4F, surfaceIndex.cellSize))));
    const auto baseKey = MakeXyGridKey(position, surfaceIndex.cellSize);
    std::optional<TrailSurfaceSample> bestSample;
    float bestScore = std::numeric_limits<float>::max();
    const float targetSurfaceZ = position.z - surfaceIndex.surfaceLift;
    const float maxProjectionSnap = std::max(0.045F, surfaceIndex.searchRadius * 0.40F);
    for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
            const auto key = EncodeTrailSurfaceGridKey(baseKey.x + dx, baseKey.y + dy);
            const auto gridIt = surfaceIndex.cellLookup.find(key);
            if (gridIt == surfaceIndex.cellLookup.end() || gridIt->second >= surfaceIndex.cells.size()) {
                continue;
            }
            const auto& candidate = surfaceIndex.cells[gridIt->second];
            const glm::vec3 candidatePosition = ToGlm(candidate.position);
            const glm::vec2 delta{
                candidatePosition.x - position.x,
                candidatePosition.y - position.y};
            const float distanceSquared = glm::dot(delta, delta);
            if (distanceSquared > searchRadius * searchRadius) {
                continue;
            }

            float surfaceZ = candidatePosition.z;
            if (std::isfinite(candidate.minZ) &&
                std::isfinite(candidate.maxZ) &&
                candidate.minZ <= candidate.maxZ) {
                surfaceZ = std::clamp(targetSurfaceZ, candidate.minZ, candidate.maxZ);
            }
            const float snappedZ = surfaceZ + surfaceIndex.surfaceLift;
            const float verticalDelta = snappedZ - position.z;
            if (mode == TrailSurfaceSampleMode::Projection &&
                std::abs(verticalDelta) > maxProjectionSnap) {
                continue;
            }

            const float score =
                mode == TrailSurfaceSampleMode::Projection
                    ? distanceSquared + verticalDelta * verticalDelta * 4.0F
                    : distanceSquared;
            if (score <= bestScore) {
                bestScore = score;
                bestSample = TrailSurfaceSample{
                    .position = {candidatePosition.x, candidatePosition.y, surfaceZ},
                    .normal = SafeOverlayNormal(
                        candidate.hasNormal ? ToGlm(candidate.normal) : glm::vec3{0.0F, 0.0F, 1.0F}),
                    .xyDistance = std::sqrt(distanceSquared),
                    .minZ = candidate.minZ,
                    .maxZ = candidate.maxZ,
                    .confidence = candidate.confidence,
                };
            }
        }
    }
    return bestSample;
}

TrailProjectionResult ProjectTrailPointToSurface(
    WaterOverlayPoint point,
    const TrailSurfaceIndex* surfaceIndex,
    float radiusScale = 1.0F) {
    if (surfaceIndex == nullptr || surfaceIndex->cells.empty()) {
        return {.point = point, .projected = false};
    }
    const auto sample = SampleTrailSurface(
        *surfaceIndex,
        ToGlm(point.position),
        radiusScale,
        TrailSurfaceSampleMode::Projection);
    if (!sample.has_value()) {
        return {.point = point, .projected = false};
    }

    glm::vec3 position = ToGlm(point.position);
    position.z = sample->position.z +
                 std::max(0.25F, std::abs(sample->normal.z)) * surfaceIndex->surfaceLift;
    point.position = FromGlm(position);
    point.normal = FromGlm(sample->normal);
    point.surfaceSteepness = std::max(point.surfaceSteepness, Clamp01(1.0F - std::abs(sample->normal.z)));
    point.confidence *= 1.0F - Clamp01(sample->xyDistance / std::max(0.001F, surfaceIndex->searchRadius)) * 0.12F;
    point.confidence *= 0.88F + Clamp01(sample->confidence) * 0.12F;
    return {.point = point, .projected = true};
}

void ProjectTrailPathToSurface(
    std::vector<WaterOverlayPoint>* path,
    const TrailSurfaceIndex* surfaceIndex,
    float radiusScale = 1.0F) {
    if (path == nullptr || path->empty() || surfaceIndex == nullptr || surfaceIndex->cells.empty()) {
        return;
    }

    std::vector<bool> projected(path->size(), false);
    for (std::size_t index = 0; index < path->size(); ++index) {
        auto projection = ProjectTrailPointToSurface((*path)[index], surfaceIndex, radiusScale);
        (*path)[index] = projection.point;
        projected[index] = projection.projected;
    }

    std::size_t firstProjected = path->size();
    for (std::size_t index = 0; index < projected.size(); ++index) {
        if (projected[index]) {
            firstProjected = index;
            break;
        }
    }
    if (firstProjected == path->size()) {
        return;
    }

    for (std::size_t index = 0; index < firstProjected; ++index) {
        auto& point = (*path)[index];
        auto position = ToGlm(point.position);
        position.z = (*path)[firstProjected].position.z;
        point.position = FromGlm(position);
        point.normal = (*path)[firstProjected].normal;
        point.surfaceSteepness = (*path)[firstProjected].surfaceSteepness;
        point.confidence *= 0.90F;
    }

    std::size_t previousProjected = firstProjected;
    for (std::size_t index = firstProjected + 1U; index < path->size(); ++index) {
        if (!projected[index]) {
            continue;
        }
        if (index > previousProjected + 1U) {
            const auto previousPoint = (*path)[previousProjected];
            const auto nextPoint = (*path)[index];
            const float distanceRange = std::max(
                0.001F,
                nextPoint.pathDistance - previousPoint.pathDistance);
            for (std::size_t fillIndex = previousProjected + 1U; fillIndex < index; ++fillIndex) {
                const float t = Clamp01(((*path)[fillIndex].pathDistance - previousPoint.pathDistance) / distanceRange);
                auto& fillPoint = (*path)[fillIndex];
                auto position = ToGlm(fillPoint.position);
                position.z = glm::mix(previousPoint.position.z, nextPoint.position.z, t);
                fillPoint.position = FromGlm(position);
                fillPoint.normal = FromGlm(SafeOverlayNormal(
                    glm::mix(ToGlm(previousPoint.normal), ToGlm(nextPoint.normal), t)));
                fillPoint.surfaceSteepness = glm::mix(previousPoint.surfaceSteepness, nextPoint.surfaceSteepness, t);
                fillPoint.confidence *= 0.92F;
            }
        }
        previousProjected = index;
    }
    for (std::size_t index = previousProjected + 1U; index < path->size(); ++index) {
        auto& point = (*path)[index];
        auto position = ToGlm(point.position);
        position.z = (*path)[previousProjected].position.z;
        point.position = FromGlm(position);
        point.normal = (*path)[previousProjected].normal;
        point.surfaceSteepness = (*path)[previousProjected].surfaceSteepness;
        point.confidence *= 0.90F;
    }
}

float SegmentPathLength(
    const std::vector<WaterOverlayPoint>& path,
    std::size_t left,
    std::size_t right) {
    if (path.empty() || left >= path.size() || right >= path.size() || right <= left) {
        return 0.0F;
    }
    float length = 0.0F;
    for (std::size_t index = left + 1U; index <= right; ++index) {
        length += SafeLength(ToGlm(path[index].position) - ToGlm(path[index - 1U].position));
    }
    return length;
}

std::vector<float> ComputeTrailKnotScores(
    const std::vector<WaterOverlayPoint>& path,
    float sampleSpacing) {
    std::vector<float> scores(path.size(), 0.0F);
    if (path.size() < 4U) {
        return scores;
    }

    float averageSegment = 0.0F;
    for (std::size_t index = 1U; index < path.size(); ++index) {
        averageSegment += SafeLength(ToGlm(path[index].position) - ToGlm(path[index - 1U].position));
    }
    averageSegment /= static_cast<float>(path.size() - 1U);
    const float spacing = PositiveOr(sampleSpacing, PositiveOr(averageSegment, 0.05F));

    for (std::size_t index = 1U; index + 1U < path.size(); ++index) {
        const glm::vec3 previous = ToGlm(path[index - 1U].position);
        const glm::vec3 current = ToGlm(path[index].position);
        const glm::vec3 next = ToGlm(path[index + 1U].position);
        const glm::vec3 incoming = current - previous;
        const glm::vec3 outgoing = next - current;
        float curvature = 0.0F;
        if (glm::dot(incoming, incoming) > 1.0e-8F && glm::dot(outgoing, outgoing) > 1.0e-8F) {
            curvature = 1.0F - Clamp01((glm::dot(glm::normalize(incoming), glm::normalize(outgoing)) + 1.0F) * 0.5F);
        }
        const float flatness = 1.0F - Clamp01(std::max(path[index].surfaceSteepness, FallbackPathTurbulence(path, index) * 0.35F));
        scores[index] = std::max(scores[index], curvature * flatness * 0.75F);
    }

    const float proximityRadius = std::max(spacing * 3.5F, averageSegment * 5.5F);
    const float minimumDistanceAlongPath = std::max(spacing * 8.0F, averageSegment * 12.0F);
    const float verticalTolerance = std::max(0.035F, proximityRadius * 0.45F);
    std::unordered_map<GridKey, std::vector<std::size_t>, GridKeyHash> proximityGrid;
    proximityGrid.reserve(path.size());
    for (std::size_t index = 0; index < path.size(); ++index) {
        proximityGrid[MakeXyGridKey(ToGlm(path[index].position), proximityRadius)].push_back(index);
    }

    for (std::size_t index = 0; index < path.size(); ++index) {
        const glm::vec3 position = ToGlm(path[index].position);
        const auto baseKey = MakeXyGridKey(position, proximityRadius);
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const GridKey key{baseKey.x + dx, baseKey.y + dy, 0};
                const auto gridIt = proximityGrid.find(key);
                if (gridIt == proximityGrid.end()) {
                    continue;
                }
                for (const auto otherIndex : gridIt->second) {
                    if (otherIndex == index) {
                        continue;
                    }
                    const float pathDistanceDelta =
                        std::abs(path[otherIndex].pathDistance - path[index].pathDistance);
                    if (pathDistanceDelta < minimumDistanceAlongPath) {
                        continue;
                    }
                    const glm::vec3 otherPosition = ToGlm(path[otherIndex].position);
                    const glm::vec2 delta{otherPosition.x - position.x, otherPosition.y - position.y};
                    const float xyDistance = glm::length(delta);
                    if (xyDistance > proximityRadius ||
                        std::abs(otherPosition.z - position.z) > verticalTolerance) {
                        continue;
                    }
                    const float flatness = 1.0F - Clamp01(std::max(path[index].surfaceSteepness, path[otherIndex].surfaceSteepness));
                    const float proximity = 1.0F - Clamp01(xyDistance / std::max(0.001F, proximityRadius));
                    scores[index] = std::max(scores[index], proximity * flatness);
                }
            }
        }
    }

    return scores;
}

bool RelaxedTrailSegmentAllowed(
    const std::vector<WaterOverlayPoint>& path,
    std::size_t left,
    std::size_t right,
    const TrailSurfaceIndex* surfaceIndex,
    float looseness) {
    if (right <= left + 1U || right >= path.size()) {
        return true;
    }

    const glm::vec3 start = ToGlm(path[left].position);
    const glm::vec3 end = ToGlm(path[right].position);
    const float segmentLength = SafeLength(end - start);
    const float ridgeTolerance = std::max(0.025F, segmentLength * (0.035F + (1.0F - looseness) * 0.055F));
    const std::uint32_t sampleCount = std::clamp<std::uint32_t>(
        static_cast<std::uint32_t>(std::ceil(segmentLength / std::max(0.01F, segmentLength * 0.16F))),
        3U,
        16U);

    for (std::uint32_t sampleIndex = 1U; sampleIndex < sampleCount; ++sampleIndex) {
        const float t = static_cast<float>(sampleIndex) / static_cast<float>(sampleCount);
        const glm::vec3 expected = glm::mix(start, end, t);
        if (surfaceIndex != nullptr && !surfaceIndex->cells.empty()) {
            const auto surface = SampleTrailSurface(
                *surfaceIndex,
                expected,
                1.0F + looseness * 2.0F,
                TrailSurfaceSampleMode::Constraint);
            if (!surface.has_value()) {
                return false;
            }
            const float ridgeHeight = surface->position.z - std::max(start.z, end.z);
            const float contourDeviation = surface->position.z - expected.z;
            if (ridgeHeight > ridgeTolerance || contourDeviation > ridgeTolerance * 1.35F) {
                return false;
            }
            continue;
        }

        const auto interiorIndex = static_cast<std::size_t>(
            std::clamp(
                std::round(static_cast<float>(left) + (static_cast<float>(right - left) * t)),
                static_cast<float>(left + 1U),
                static_cast<float>(right - 1U)));
        const glm::vec3 interior = ToGlm(path[interiorIndex].position);
        if (interior.z - std::max(start.z, end.z) > ridgeTolerance ||
            path[interiorIndex].surfaceSteepness > 0.42F + looseness * 0.18F) {
            return false;
        }
    }
    return true;
}

std::vector<WaterOverlayPoint> SimplifyTrailGuidePath(
    const std::vector<WaterOverlayPoint>& path,
    const std::vector<float>& knotScores,
    const TrailSurfaceIndex* surfaceIndex,
    float looseness,
    float sampleSpacing,
    float smoothness,
    WaterTrailBuildQuality quality) {
    if (path.size() < 3U || looseness <= 1.0e-4F) {
        return path;
    }

    const float averageSpacing = PositiveOr(sampleSpacing, 0.05F);
    std::vector<float> prefixLength(path.size(), 0.0F);
    std::vector<float> prefixKnot(path.size() + 1U, 0.0F);
    std::vector<float> prefixSteepness(path.size() + 1U, 0.0F);
    for (std::size_t index = 1U; index < path.size(); ++index) {
        prefixLength[index] =
            prefixLength[index - 1U] + SafeLength(ToGlm(path[index].position) - ToGlm(path[index - 1U].position));
    }
    for (std::size_t index = 0U; index < path.size(); ++index) {
        prefixKnot[index + 1U] = prefixKnot[index] + (index < knotScores.size() ? knotScores[index] : 0.0F);
        prefixSteepness[index + 1U] = prefixSteepness[index] + path[index].surfaceSteepness;
    }

    const float qualityHorizon = quality == WaterTrailBuildQuality::Preview ? 22.0F : 54.0F;
    const auto maxHorizon = static_cast<std::size_t>(std::clamp(
        1.0F + looseness * (3.0F + qualityHorizon) + smoothness * looseness * 8.0F,
        1.0F,
        quality == WaterTrailBuildQuality::Preview ? 28.0F : 72.0F));
    std::vector<float> bestCost(path.size(), std::numeric_limits<float>::max());
    std::vector<std::size_t> previousIndex(path.size(), 0U);
    bestCost[0] = 0.0F;

    for (std::size_t right = 1U; right < path.size(); ++right) {
        const std::size_t leftBegin = right > maxHorizon ? right - maxHorizon : 0U;
        for (std::size_t left = leftBegin; left < right; ++left) {
            if (!std::isfinite(bestCost[left])) {
                continue;
            }
            if (!RelaxedTrailSegmentAllowed(path, left, right, surfaceIndex, looseness)) {
                continue;
            }

            const glm::vec3 start = ToGlm(path[left].position);
            const glm::vec3 end = ToGlm(path[right].position);
            const float directLength = SafeLength(end - start);
            const float pathLength = std::max(directLength, prefixLength[right] - prefixLength[left]);
            const float skipped = static_cast<float>(right - left - 1U);
            const float count = static_cast<float>(right - left + 1U);
            const float knotMean = Clamp01((prefixKnot[right + 1U] - prefixKnot[left]) / std::max(1.0F, count));
            const float steepnessMean =
                Clamp01((prefixSteepness[right + 1U] - prefixSteepness[left]) / std::max(1.0F, count));

            const float shortcutAmount = Clamp01(skipped / std::max(1.0F, static_cast<float>(maxHorizon)));
            const float detailPenalty =
                (1.0F - knotMean) * skipped * averageSpacing * (2.85F - looseness * 1.10F);
            const float contourPenalty = steepnessMean * skipped * averageSpacing * (2.25F - looseness * 0.65F);
            const float smoothnessReward = smoothness * shortcutAmount * averageSpacing * (0.20F + looseness);
            const float knotReward = knotMean * skipped * averageSpacing * (1.0F + looseness * 3.75F);
            const float lengthReward = (pathLength - directLength) * looseness * (0.35F + knotMean);
            const float edgeCost =
                directLength + detailPenalty + contourPenalty - knotReward - lengthReward - smoothnessReward;
            const float candidateCost = bestCost[left] + edgeCost;
            if (candidateCost < bestCost[right]) {
                bestCost[right] = candidateCost;
                previousIndex[right] = left;
            }
        }
        if (!std::isfinite(bestCost[right])) {
            bestCost[right] = bestCost[right - 1U] +
                              SafeLength(ToGlm(path[right].position) - ToGlm(path[right - 1U].position));
            previousIndex[right] = right - 1U;
        }
    }

    std::vector<WaterOverlayPoint> simplified;
    for (std::size_t cursor = path.size() - 1U;; cursor = previousIndex[cursor]) {
        simplified.push_back(path[cursor]);
        if (cursor == 0U) {
            break;
        }
    }
    std::reverse(simplified.begin(), simplified.end());

    RecomputePathDistances(&simplified, std::max(0.001F, path.back().pathDistance - path.front().pathDistance));
    return simplified;
}

std::vector<WaterOverlayPoint> BuildOffsetTrailLanePath(
    const std::vector<WaterOverlayPoint>& guidePath,
    const TrailSurfaceIndex* surfaceIndex,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    std::uint32_t laneIndex,
    std::uint32_t laneCount,
    WaterTrailBuildQuality quality) {
    if (guidePath.empty()) {
        return {};
    }

    const float jitter = std::clamp(trailShapeSettings.particleJitter, 0.0F, 3.0F);
    const float looseness = std::clamp(trailShapeSettings.trailLooseness, 0.0F, 1.0F);
    const float smoothness = std::clamp(trailShapeSettings.trailSmoothness, 0.0F, 1.0F);
    const float laneAmount =
        laneCount <= 1U
            ? 0.0F
            : (static_cast<float>(laneIndex) / static_cast<float>(laneCount - 1U)) * 2.0F - 1.0F;
    const float laneSeed = Hash01(
        (static_cast<std::uint32_t>(guidePath.front().flowId * 4099.0F) * 747796405U) ^
        (laneIndex * 2891336453U) ^
        static_cast<std::uint32_t>(guidePath.front().emitterId * 1973.0F));
    constexpr float twoPi = 6.28318530718F;

    std::vector<WaterOverlayPoint> lanePath;
    lanePath.reserve(guidePath.size());
    for (std::size_t index = 0; index < guidePath.size(); ++index) {
        auto lanePoint = guidePath[index];
        const glm::vec3 tangent = WaterPathTangent(guidePath, index);
        const glm::vec3 lateral = WaterPathLateral(tangent);
        const float guideSteepness = std::max(lanePoint.surfaceSteepness, FallbackPathTurbulence(guidePath, index));
        const float endpointFade =
            std::min(
                SmoothStep(0.0F, 2.0F, static_cast<float>(index)),
                SmoothStep(0.0F, 2.0F, static_cast<float>((guidePath.size() - 1U) - index)));
        const float maxOffset = std::max(0.0F, lanePoint.width * jitter * (0.28F + looseness * 0.34F) * endpointFade);
        const float lowFrequencyWander =
            std::sin(
                lanePoint.pathDistance * (0.72F + laneSeed * 1.45F) +
                laneSeed * twoPi +
                static_cast<float>(laneIndex) * 1.618F);
        const float lateralOffset =
            (laneAmount * maxOffset * (0.76F + laneSeed * 0.10F)) +
            (lowFrequencyWander * maxOffset * looseness * 0.22F);
        lanePoint.position = FromGlm(ToGlm(lanePoint.position) + lateral * lateralOffset);
        lanePoint.particleRole = 3.0F;
        lanePoint.jitterSeed = laneSeed;
        lanePoint.trailAge = 0.0F;
        lanePoint.trailLaneId = static_cast<float>(laneIndex);
        lanePoint.trailLateralOffset = lateralOffset;
        lanePoint.surfaceSteepness = std::max(lanePoint.surfaceSteepness, guideSteepness);
        lanePoint.red = FloatToByte(0.02F + guideSteepness * 0.08F);
        lanePoint.green = FloatToByte(0.58F + guideSteepness * 0.20F);
        lanePoint.blue = FloatToByte(0.96F + laneSeed * 0.04F);
        lanePath.push_back(lanePoint);
    }
    ProjectTrailPathToSurface(&lanePath, surfaceIndex, 1.0F + looseness);
    if (lanePath.size() >= 3U && smoothness > 1.0e-4F) {
        SmoothWaterPath(
            &lanePath,
            smoothness * (quality == WaterTrailBuildQuality::Preview ? 0.55F : 0.75F),
            std::max(0.001F, guidePath.back().pathDistance - guidePath.front().pathDistance));
        const float splineSpacing = std::max(
            0.01F,
            trailShapeSettings.splineAnchorSpacing *
                (quality == WaterTrailBuildQuality::Preview ? (1.65F - smoothness * 0.35F)
                                                            : (1.05F - smoothness * 0.28F)));
        lanePath = BuildSplineViewSamples(lanePath, splineSpacing);
        ProjectTrailPathToSurface(&lanePath, surfaceIndex, 1.0F + looseness);
        for (auto& lanePoint : lanePath) {
            lanePoint.particleRole = 3.0F;
            lanePoint.trailLaneId = static_cast<float>(laneIndex);
        }
    }
    RecomputePathDistances(&lanePath, std::max(0.001F, guidePath.back().pathDistance - guidePath.front().pathDistance));
    return lanePath;
}

std::vector<WaterTrailGuidePath> IncludeWaterTrailLaneGuides(
    WaterOverlay* overlay,
    const std::vector<WaterOverlayPoint>& guidePath,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality,
    WaterTrailBuildDiagnostics* diagnostics) {
    std::vector<WaterTrailGuidePath> laneGuides;
    if (overlay == nullptr || guidePath.size() < 2U) {
        return laneGuides;
    }

    const std::uint32_t laneCount = std::clamp<std::uint32_t>(trailShapeSettings.trailLaneCount, 0U, 32U);
    if (laneCount == 0U) {
        return laneGuides;
    }

    const float looseness = std::clamp(trailShapeSettings.trailLooseness, 0.0F, 1.0F);
    const float smoothness = std::clamp(trailShapeSettings.trailSmoothness, 0.0F, 1.0F);
    const auto routeStart = std::chrono::steady_clock::now();
    const auto knotScores = ComputeTrailKnotScores(guidePath, trailShapeSettings.splineAnchorSpacing);
    const auto relaxedPath = SimplifyTrailGuidePath(
        guidePath,
        knotScores,
        surfaceIndex,
        looseness,
        trailShapeSettings.splineAnchorSpacing,
        smoothness,
        quality);
    if (diagnostics != nullptr) {
        diagnostics->routeMs += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - routeStart).count();
        ++diagnostics->routedPathCount;
    }

    laneGuides.reserve(laneCount);
    const auto laneStart = std::chrono::steady_clock::now();
    for (std::uint32_t laneIndex = 0; laneIndex < laneCount; ++laneIndex) {
        auto lanePath = BuildOffsetTrailLanePath(
            relaxedPath,
            surfaceIndex,
            trailShapeSettings,
            laneIndex,
            laneCount,
            quality);
        if (lanePath.size() < 2U) {
            continue;
        }
        const auto laneStartIndex = static_cast<std::uint32_t>(
            std::min<std::size_t>(overlay->points.size(), std::numeric_limits<std::uint32_t>::max()));
        const auto lanePointCount = static_cast<std::uint32_t>(
            std::min<std::size_t>(lanePath.size(), std::numeric_limits<std::uint32_t>::max()));
        for (auto& lanePoint : lanePath) {
            lanePoint.pathStartIndex = static_cast<float>(laneStartIndex);
            lanePoint.pathPointCount = static_cast<float>(lanePointCount);
            IncludeOverlayPoint(overlay, lanePoint);
        }
        laneGuides.push_back({
            .points = std::move(lanePath),
            .startIndex = laneStartIndex,
            .pointCount = lanePointCount,
        });
        if (diagnostics != nullptr) {
            ++diagnostics->emittedLaneCount;
        }
    }
    if (diagnostics != nullptr) {
        diagnostics->laneMs += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - laneStart).count();
    }
    return laneGuides;
}

std::vector<WaterTrailGuidePath> IncludeWaterPathViewGuides(
    WaterOverlay* overlay,
    const std::vector<WaterOverlayPoint>& path,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality,
    WaterTrailBuildDiagnostics* diagnostics) {
    if (overlay == nullptr || path.empty()) {
        return {};
    }

    const auto guidePath = BuildSplineViewSamples(
        path,
        animationTrailSettings.trailSampleSpacingMeters > 0.0F
            ? TrailPlaybackSampleSpacing(trailShapeSettings, animationTrailSettings)
            : 0.0F);
    IncludeWaterPathViewAnchors(overlay, guidePath);
    return IncludeWaterTrailLaneGuides(overlay, guidePath, trailShapeSettings, surfaceIndex, quality, diagnostics);
}

std::uint8_t FloatToByte(float value) {
    return static_cast<std::uint8_t>(std::clamp(std::round(value * 255.0F), 0.0F, 255.0F));
}

WaterOverlayPoint MakeOverlayPoint(
    const glm::vec3& position,
    const WaterEmitter& emitter,
    std::uint32_t flowId,
    float distanceAlongPath,
    float pathLength,
    float confidence,
    float pooling,
    float width) {
    const float accumulation = Clamp01(distanceAlongPath / std::max(0.001F, pathLength));
    const float cyanMix = Clamp01(0.35F + accumulation * 0.45F + pooling * 0.2F);
    WaterOverlayPoint point;
    point.position = FromGlm(position);
    point.red = FloatToByte(0.03F + pooling * 0.12F);
    point.green = FloatToByte(0.48F + cyanMix * 0.42F);
    point.blue = FloatToByte(0.86F + confidence * 0.12F);
    point.flowId = static_cast<float>(flowId);
    point.emitterId = static_cast<float>(emitter.id);
    point.pathDistance = distanceAlongPath;
    point.phase = std::fmod((distanceAlongPath * 0.37F) + (static_cast<float>(emitter.id) * 0.173F), 1.0F);
    point.speed = std::max(0.05F, emitter.speed);
    point.width = std::max(0.05F, width);
    point.confidence = Clamp01(confidence);
    point.accumulation = accumulation;
    point.pooling = Clamp01(pooling);
    return point;
}

WaterPathTerminationReason TerminationForStepLimit(
    std::uint32_t step,
    std::uint32_t maxSteps,
    float distanceAlongPath,
    float pathLength) {
    if (distanceAlongPath >= pathLength) {
        return WaterPathTerminationReason::ReachedLength;
    }
    if (step >= maxSteps) {
        return WaterPathTerminationReason::MaxSteps;
    }
    return WaterPathTerminationReason::NoSupport;
}

TraceResult TraceWaterPathBranch(
    const SupportGraph& graph,
    const WaterEmitter& emitter,
    const WaterPathGenerationSettings& settings,
    std::uint32_t branchId,
    WaterPathBranchRole role,
    std::optional<std::uint32_t> parentId,
    std::uint32_t startIndex,
    const std::unordered_set<std::uint32_t>* occupied,
    bool collectOpportunities) {
    TraceResult result;
    result.branch.id = branchId;
    result.branch.parentId = parentId;
    result.branch.emitterId = emitter.id;
    result.branch.role = role;
    result.branch.terminationReason = WaterPathTerminationReason::Empty;
    if (startIndex >= graph.points.size()) {
        return result;
    }

    const std::uint32_t maxSteps = settings.maxSteps;
    const float pathLength =
        role == WaterPathBranchRole::Main
            ? settings.pathLength
            : settings.pathLength * (0.35F + std::clamp(settings.coverage, 0.0F, 1.0F) * 0.35F);
    const float pathSampleSpacing = std::max(0.005F, settings.pathSampleSpacing);
    const float surfaceLift = std::max(0.003F, settings.supportVoxelSize * 0.18F);
    const std::uint32_t maxOpportunities =
        collectOpportunities
            ? static_cast<std::uint32_t>(2U + std::round(std::clamp(settings.branching, 0.0F, 1.0F) * 8.0F))
            : 0U;

    std::uint32_t currentIndex = startIndex;
    float distanceAlongPath = 0.0F;
    float confidence = Clamp01(emitter.confidence);
    float confidenceSum = 0.0F;
    float flatnessSum = 0.0F;
    std::uint32_t flatnessCount = 0U;
    std::uint32_t step = 0U;
    std::vector<std::uint32_t> visited;
    visited.reserve(maxSteps);
    result.branch.rawAnchors.reserve(maxSteps * 2U);

    for (; step < maxSteps && distanceAlongPath < pathLength; ++step) {
        if (currentIndex >= graph.points.size()) {
            result.branch.terminationReason = WaterPathTerminationReason::NoSupport;
            break;
        }
        if (std::find(visited.begin(), visited.end(), currentIndex) != visited.end()) {
            result.branch.terminationReason = WaterPathTerminationReason::Loop;
            break;
        }

        visited.push_back(currentIndex);
        result.visitedSupportIndices.push_back(currentIndex);
        const auto& current = graph.points[currentIndex];
        const auto ranked = RankDownhillNeighbours(
            graph,
            currentIndex,
            settings,
            visited,
            role == WaterPathBranchRole::Main ? nullptr : occupied);

        if (ranked.empty()) {
            const glm::vec3 lift =
                current.hasNormal ? current.normal * surfaceLift : glm::vec3{0.0F, 0.0F, surfaceLift};
            const glm::vec3 overlayNormal =
                SafeOverlayNormal(current.hasNormal ? current.normal : glm::vec3{0.0F, 0.0F, 1.0F});
            result.branch.rawAnchors.push_back(
                [&]() {
                    auto point = MakeOverlayPoint(
                        current.position + lift,
                        emitter,
                        branchId,
                        distanceAlongPath,
                        pathLength,
                        confidence * current.confidence,
                        0.85F,
                        std::max(pathSampleSpacing, emitter.radius * 0.55F));
                    point.normal = FromGlm(overlayNormal);
                    point.surfaceSteepness =
                        current.hasNormal ? Clamp01(1.0F - std::abs(current.normal.z)) : 0.0F;
                    return point;
                }());
            result.branch.terminationReason = WaterPathTerminationReason::NoSupport;
            break;
        }

        if (collectOpportunities &&
            result.opportunities.size() < maxOpportunities &&
            ranked.size() >= 2U &&
            distanceAlongPath > pathSampleSpacing * 2.0F) {
            const auto& best = ranked.front();
            const std::uint32_t candidateLimit = std::min<std::uint32_t>(8U, static_cast<std::uint32_t>(ranked.size()));
            for (std::uint32_t candidateIndex = 1U;
                 candidateIndex < candidateLimit && result.opportunities.size() < maxOpportunities;
                 ++candidateIndex) {
                const auto& candidate = ranked[candidateIndex];
                const bool scoreClose = candidate.score >= best.score - (1.35F + settings.branching * 1.25F);
                const bool spreadFlat = candidate.flatness >= 0.40F && best.flatness >= 0.25F;
                if (!scoreClose && !spreadFlat) {
                    continue;
                }
                result.opportunities.push_back({
                    .parentBranchId = branchId,
                    .fromSupportIndex = currentIndex,
                    .startSupportIndex = candidate.supportIndex,
                    .parentDistance = distanceAlongPath,
                    .score = candidate.score,
                    .flatness = candidate.flatness,
                    .role = spreadFlat ? WaterPathBranchRole::Spread : WaterPathBranchRole::Secondary,
                });
            }
        }

        const auto& nextCandidate = ranked.front();
        if (nextCandidate.supportIndex >= graph.points.size()) {
            result.branch.terminationReason = WaterPathTerminationReason::NoSupport;
            break;
        }

        const auto& next = graph.points[nextCandidate.supportIndex];
        const glm::vec3 segment = next.position - current.position;
        const float segmentLength = SafeLength(segment);
        if (segmentLength <= 1.0e-5F) {
            result.branch.terminationReason = WaterPathTerminationReason::Duplicate;
            break;
        }

        const float drop = current.position.z - next.position.z;
        const float pooling = Clamp01(1.0F - (std::abs(drop) / std::max(0.01F, settings.maxBridgeDistance * 0.35F)));
        flatnessSum += pooling;
        ++flatnessCount;
        const std::uint32_t segmentSamples = std::max<std::uint32_t>(
            1U,
            static_cast<std::uint32_t>(std::ceil(segmentLength / pathSampleSpacing)));
        for (std::uint32_t sample = 0; sample <= segmentSamples; ++sample) {
            const float t = static_cast<float>(sample) / static_cast<float>(segmentSamples);
            const glm::vec3 position = current.position + segment * t;
            const glm::vec3 normal =
                current.hasNormal ? current.normal : (next.hasNormal ? next.normal : glm::vec3{0.0F, 0.0F, 1.0F});
            const glm::vec3 overlayNormal = SafeOverlayNormal(normal);
            const glm::vec3 lift = normal * surfaceLift;
            const float sampleDistance = distanceAlongPath + segmentLength * t;
            const float width =
                std::max(pathSampleSpacing, emitter.radius * (0.35F + emitter.strength * 0.28F)) *
                (0.7F + Clamp01(sampleDistance / std::max(0.001F, pathLength)) * 0.55F + pooling * 0.35F);
            const float sampleConfidence =
                confidence * ((current.confidence * (1.0F - t)) + (nextCandidate.confidence * t));
            const float normalSteepness =
                current.hasNormal && next.hasNormal
                    ? Clamp01(
                          ((1.0F - std::abs(current.normal.z)) * (1.0F - t)) +
                          ((1.0F - std::abs(next.normal.z)) * t))
                    : Clamp01(std::abs(drop) / std::max(0.001F, segmentLength));
            confidenceSum += Clamp01(sampleConfidence);
            result.branch.rawAnchors.push_back(
                [&]() {
                    auto point = MakeOverlayPoint(
                        position + lift,
                        emitter,
                        branchId,
                        sampleDistance,
                        pathLength,
                        sampleConfidence,
                        pooling,
                        width);
                    point.normal = FromGlm(overlayNormal);
                    point.surfaceSteepness = normalSteepness;
                    return point;
                }());
        }

        confidence *= 0.985F;
        if (nextCandidate.bridgeJump) {
            confidence *= 0.76F + std::clamp(settings.gapTolerance, 0.0F, 1.0F) * 0.16F;
            ++result.branch.gapCount;
        }
        distanceAlongPath += segmentLength;
        currentIndex = nextCandidate.supportIndex;
    }

    if (result.branch.terminationReason == WaterPathTerminationReason::Empty) {
        result.branch.terminationReason = TerminationForStepLimit(step, maxSteps, distanceAlongPath, pathLength);
    }
    RecomputePathDistances(&result.branch.rawAnchors, pathLength);
    result.branch.length =
        result.branch.rawAnchors.empty()
            ? 0.0F
            : result.branch.rawAnchors.back().pathDistance - result.branch.rawAnchors.front().pathDistance;
    result.branch.confidence =
        result.branch.rawAnchors.empty()
            ? 0.0F
            : Clamp01(confidenceSum / static_cast<float>(result.branch.rawAnchors.size()));
    result.branch.flatness =
        flatnessCount == 0U ? 0.0F : Clamp01(flatnessSum / static_cast<float>(flatnessCount));
    return result;
}

float Hash01(std::uint32_t value) {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    value ^= value >> 16U;
    return static_cast<float>(value & 0x00ffffffU) / 16777215.0F;
}

WaterOverlayPoint BlendPathAnchor(
    const WaterOverlayPoint& left,
    const WaterOverlayPoint& right,
    float amount) {
    const float t = Clamp01(amount);
    WaterOverlayPoint point = left;
    point.position = FromGlm(glm::mix(ToGlm(left.position), ToGlm(right.position), t));
    point.normal = FromGlm(SafeOverlayNormal(glm::mix(ToGlm(left.normal), ToGlm(right.normal), t)));
    point.pathDistance = left.pathDistance + ((right.pathDistance - left.pathDistance) * t);
    point.width = left.width + ((right.width - left.width) * t);
    point.confidence = left.confidence + ((right.confidence - left.confidence) * t);
    point.accumulation = left.accumulation + ((right.accumulation - left.accumulation) * t);
    point.pooling = left.pooling + ((right.pooling - left.pooling) * t);
    point.surfaceSteepness = left.surfaceSteepness + ((right.surfaceSteepness - left.surfaceSteepness) * t);
    point.trailLaneId = left.trailLaneId + ((right.trailLaneId - left.trailLaneId) * t);
    point.trailLateralOffset =
        left.trailLateralOffset + ((right.trailLateralOffset - left.trailLateralOffset) * t);
    return point;
}

WaterOverlayPoint InterpolatePathAnchor(
    const std::vector<WaterOverlayPoint>& path,
    float normalizedDistance) {
    if (path.empty()) {
        return {};
    }
    if (path.size() == 1U) {
        return path.front();
    }

    const float scaled = Clamp01(normalizedDistance) * static_cast<float>(path.size() - 1U);
    const auto leftIndex = static_cast<std::size_t>(std::floor(scaled));
    const auto rightIndex = std::min<std::size_t>(leftIndex + 1U, path.size() - 1U);
    return BlendPathAnchor(path[leftIndex], path[rightIndex], scaled - static_cast<float>(leftIndex));
}

WaterOverlayPoint InterpolatePathAnchorByDistance(
    const std::vector<WaterOverlayPoint>& path,
    float pathDistance) {
    if (path.empty()) {
        return {};
    }
    if (path.size() == 1U || pathDistance <= path.front().pathDistance) {
        return path.front();
    }
    if (pathDistance >= path.back().pathDistance) {
        return path.back();
    }

    for (std::size_t rightIndex = 1U; rightIndex < path.size(); ++rightIndex) {
        const auto& left = path[rightIndex - 1U];
        const auto& right = path[rightIndex];
        if (pathDistance > right.pathDistance && rightIndex + 1U < path.size()) {
            continue;
        }

        const float segmentLength = right.pathDistance - left.pathDistance;
        if (segmentLength <= 1.0e-5F) {
            return right;
        }
        return BlendPathAnchor(left, right, (pathDistance - left.pathDistance) / segmentLength);
    }

    return path.back();
}

std::vector<WaterOverlayPoint> ResampleSplineAnchors(
    const std::vector<WaterOverlayPoint>& path,
    float anchorSpacing) {
    if (path.size() <= 2U) {
        return path;
    }

    const float startDistance = path.front().pathDistance;
    const float endDistance = path.back().pathDistance;
    const float pathLength = std::max(0.0F, endDistance - startDistance);
    if (pathLength <= 1.0e-5F) {
        return path;
    }

    const float spacing = std::clamp(PositiveOr(anchorSpacing, 0.5F), 0.01F, 25.0F);
    constexpr std::size_t maxAnchorCount = 16384U;
    std::vector<WaterOverlayPoint> anchors;
    anchors.reserve(std::min<std::size_t>(
        maxAnchorCount,
        static_cast<std::size_t>(std::ceil(pathLength / spacing)) + 1U));

    anchors.push_back(InterpolatePathAnchorByDistance(path, startDistance));
    for (float distance = startDistance + spacing;
         distance < endDistance && anchors.size() + 1U < maxAnchorCount;
         distance += spacing) {
        anchors.push_back(InterpolatePathAnchorByDistance(path, distance));
    }
    if (anchors.back().pathDistance < endDistance - 1.0e-4F && anchors.size() < maxAnchorCount) {
        anchors.push_back(InterpolatePathAnchorByDistance(path, endDistance));
    }

    return anchors.size() >= 2U ? anchors : path;
}

void ApplyParticleBlue(
    WaterOverlayPoint* point,
    float seed,
    float colorVariation) {
    if (point == nullptr) {
        return;
    }
    const float variation = Clamp01(colorVariation);
    const float hueSeed = Hash01(static_cast<std::uint32_t>(seed * 16777215.0F) ^ 0x9e3779b9U);
    const float brightSeed = Hash01(static_cast<std::uint32_t>(seed * 1103515245.0F) ^ 0x85ebca6bU);
    constexpr glm::vec3 palette[] = {
        glm::vec3{0.00F, 0.10F, 0.42F},
        glm::vec3{0.00F, 0.25F, 0.88F},
        glm::vec3{0.00F, 0.48F, 1.00F},
        glm::vec3{0.08F, 0.78F, 1.00F},
        glm::vec3{0.38F, 0.90F, 1.00F},
        glm::vec3{0.72F, 0.97F, 1.00F},
    };
    constexpr std::size_t paletteCount = sizeof(palette) / sizeof(palette[0]);
    const float scaled = hueSeed * static_cast<float>(paletteCount - 1U);
    const auto leftIndex = static_cast<std::size_t>(std::floor(scaled));
    const auto rightIndex = std::min<std::size_t>(leftIndex + 1U, paletteCount - 1U);
    const glm::vec3 paletteColor = glm::mix(palette[leftIndex], palette[rightIndex], scaled - std::floor(scaled));
    const glm::vec3 baseColor{0.02F, 0.58F, 1.0F};
    const float paletteAmount = std::clamp(0.18F + variation * 0.82F, 0.0F, 1.0F);
    glm::vec3 color = glm::mix(baseColor, paletteColor, paletteAmount);
    color *= 0.86F + brightSeed * 0.20F;
    color.b = std::max(color.b, color.g + 0.04F);
    point->red = FloatToByte(color.r);
    point->green = FloatToByte(color.g);
    point->blue = FloatToByte(color.b);
}

float CrossXy(
    const invisible_places::io::Float3& origin,
    const invisible_places::io::Float3& left,
    const invisible_places::io::Float3& right) {
    return ((left.x - origin.x) * (right.y - origin.y)) -
           ((left.y - origin.y) * (right.x - origin.x));
}

float DistanceToSegmentXy(
    const glm::vec3& point,
    const invisible_places::io::Float3& start,
    const invisible_places::io::Float3& end) {
    const glm::vec2 p{point.x, point.y};
    const glm::vec2 a{start.x, start.y};
    const glm::vec2 b{end.x, end.y};
    const glm::vec2 ab = b - a;
    const float lengthSquared = glm::dot(ab, ab);
    if (lengthSquared <= 1.0e-8F) {
        return glm::length(p - a);
    }
    const float t = std::clamp(glm::dot(p - a, ab) / lengthSquared, 0.0F, 1.0F);
    return glm::length(p - (a + ab * t));
}

float DistanceToSegment3d(
    const glm::vec3& point,
    const invisible_places::io::Float3& start,
    const invisible_places::io::Float3& end) {
    const glm::vec3 a = ToGlm(start);
    const glm::vec3 b = ToGlm(end);
    const glm::vec3 ab = b - a;
    const float lengthSquared = glm::dot(ab, ab);
    if (lengthSquared <= 1.0e-8F) {
        return glm::length(point - a);
    }
    const float t = std::clamp(glm::dot(point - a, ab) / lengthSquared, 0.0F, 1.0F);
    return glm::length(point - (a + ab * t));
}

bool PointInPolygonXy(
    const glm::vec3& point,
    const std::vector<invisible_places::io::Float3>& polygon) {
    if (polygon.size() < 3U) {
        return false;
    }

    bool inside = false;
    for (std::size_t current = 0U, previous = polygon.size() - 1U;
         current < polygon.size();
         previous = current++) {
        const auto& a = polygon[current];
        const auto& b = polygon[previous];
        const float denominator = b.y - a.y;
        const bool crosses =
            ((a.y > point.y) != (b.y > point.y)) &&
            std::abs(denominator) > 1.0e-8F &&
            (point.x < (b.x - a.x) * (point.y - a.y) / denominator + a.x);
        if (crosses) {
            inside = !inside;
        }
    }
    return inside;
}

float RegionHash01(std::uint32_t regionId, std::uint32_t pointIndex, std::uint32_t salt = 0U) {
    return Hash01((regionId * 747796405U) ^ (pointIndex * 2891336453U) ^ (salt * 277803737U));
}

WaterOverlayPoint MakeFeaturePoint(
    const glm::vec3& position,
    float featureType,
    std::uint32_t regionId,
    float phase,
    float speed,
    float confidence,
    float accumulation,
    float trailAge,
    float trailLength) {
    WaterOverlayPoint point;
    point.position = FromGlm(position);
    point.red = FloatToByte(featureType > 1.5F ? 0.04F : 0.72F);
    point.green = FloatToByte(featureType > 1.5F ? 0.74F : 0.92F);
    point.blue = FloatToByte(1.0F);
    point.flowId = static_cast<float>(regionId);
    point.emitterId = 0.0F;
    point.pathDistance = 0.0F;
    point.phase = phase - std::floor(phase);
    point.speed = std::max(0.01F, speed);
    point.width = 0.0F;
    point.confidence = Clamp01(confidence);
    point.accumulation = Clamp01(accumulation);
    point.pooling = 0.0F;
    point.particleRole = 1.0F;
    point.jitterSeed = RegionHash01(regionId, static_cast<std::uint32_t>(phase * 16777215.0F));
    point.trailAge = Clamp01(trailAge);
    point.trailLength = std::max(0.0F, trailLength);
    point.featureType = featureType;
    point.regionId = static_cast<float>(regionId);
    return point;
}

void IncludeWaterPathWithParticles(
    WaterOverlay* overlay,
    std::vector<WaterOverlayPoint> path,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    float featureType = 0.0F,
    float regionId = 0.0F,
    const TrailSurfaceIndex* surfaceIndex = nullptr,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr) {
    if (overlay == nullptr || path.empty()) {
        return;
    }

    const bool previewQuality = quality == WaterTrailBuildQuality::Preview;
    const float trailSampleSpacing =
        TrailPlaybackSampleSpacing(trailShapeSettings, animationTrailSettings) *
        (previewQuality ? 1.75F : 1.0F);
    path = ResampleSplineAnchors(path, trailSampleSpacing);
    if (path.empty()) {
        return;
    }

    std::vector<WaterTrailGuidePath> laneGuides;
    if (featureType < 0.5F) {
        laneGuides = IncludeWaterPathViewGuides(
            overlay,
            path,
            trailShapeSettings,
            animationTrailSettings,
            surfaceIndex,
            quality,
            diagnostics);
    }

    const float pathLength =
        std::max(0.0F, path.back().pathDistance - path.front().pathDistance);
    const auto pathStartIndex = static_cast<std::uint32_t>(
        std::min<std::size_t>(overlay->points.size(), std::numeric_limits<std::uint32_t>::max()));
    const auto pathPointCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(path.size(), std::numeric_limits<std::uint32_t>::max()));

    for (auto& anchor : path) {
        anchor.particleRole = 0.0F;
        anchor.pathStartIndex = static_cast<float>(pathStartIndex);
        anchor.pathPointCount = static_cast<float>(pathPointCount);
        anchor.jitterSeed = Hash01(
            static_cast<std::uint32_t>(anchor.flowId * 4099.0F + anchor.pathDistance * 6553.0F));
        anchor.featureType = featureType;
        anchor.regionId = regionId;
        IncludeOverlayPoint(overlay, anchor);
    }

    if (pathPointCount < 2U) {
        return;
    }

    const float density = std::clamp(animationTrailSettings.particleDensity, 0.05F, 10.0F);
    const float particleSpacing = std::max(0.001F, PositiveOr(trailShapeSettings.splineAnchorSpacing, 0.5F));
    const std::uint32_t maxParticleCount = previewQuality ? 2048U : 8192U;
    const std::uint32_t particleCount = std::clamp<std::uint32_t>(
        static_cast<std::uint32_t>(
            std::ceil(std::max(pathLength, particleSpacing) / particleSpacing *
                      density * (previewQuality ? 0.58F : 1.0F))),
        1U,
        maxParticleCount);
    const float jitter = std::clamp(trailShapeSettings.particleJitter, 0.0F, 3.0F);
    const float speed = std::clamp(animationTrailSettings.particleSpeed, 0.05F, 8.0F);
    const float trailLength = std::clamp(animationTrailSettings.trailLengthMeters, 0.0F, 25.0F);
    const std::uint32_t ghostSampleCount =
        trailLength > 0.01F
            ? std::clamp<std::uint32_t>(
                  static_cast<std::uint32_t>(std::ceil(trailLength / std::max(0.004F, trailSampleSpacing))),
                  1U,
                  previewQuality ? 16U : 48U)
            : 0U;
    const float normalizedTrailLength =
        pathLength > 1.0e-4F ? std::min(0.95F, trailLength / pathLength) : 0.0F;

    const auto particleStart = std::chrono::steady_clock::now();
    for (std::uint32_t particleIndex = 0; particleIndex < particleCount; ++particleIndex) {
        const float seed = Hash01(
            (pathStartIndex * 747796405U) ^
            (particleIndex * 2891336453U) ^
            static_cast<std::uint32_t>(path.front().emitterId * 1973.0F));
        const float basePhase = static_cast<float>(particleIndex) / static_cast<float>(particleCount);
        const float spacingJitter =
            ((seed - 0.5F) * std::min(jitter, 1.0F) * 0.25F) /
            static_cast<float>(particleCount);
        const float rawPhase = basePhase + spacingJitter;
        const float phase = rawPhase - std::floor(rawPhase);
        for (std::uint32_t trailIndex = 0; trailIndex <= ghostSampleCount; ++trailIndex) {
            const float trailAge =
                ghostSampleCount > 0U
                    ? static_cast<float>(trailIndex) / static_cast<float>(ghostSampleCount + 1U)
                    : 0.0F;
            float trailPhase = phase - (normalizedTrailLength * trailAge);
            trailPhase -= std::floor(trailPhase);
            const WaterTrailGuidePath* laneGuide =
                laneGuides.empty() ? nullptr : &laneGuides[particleIndex % laneGuides.size()];
            const auto& particlePath = laneGuide == nullptr ? path : laneGuide->points;
            WaterOverlayPoint particle = InterpolatePathAnchor(particlePath, trailPhase);
            particle.phase = trailPhase;
            particle.speed = std::max(0.02F, particle.speed * speed * (0.82F + seed * 0.36F));
            particle.width = jitter;
            particle.particleRole = 1.0F;
            particle.pathStartIndex =
                static_cast<float>(laneGuide == nullptr ? pathStartIndex : laneGuide->startIndex);
            particle.pathPointCount =
                static_cast<float>(laneGuide == nullptr ? pathPointCount : laneGuide->pointCount);
            particle.jitterSeed = seed;
            particle.trailAge = trailAge;
            particle.trailLength = trailLength;
            particle.featureType = featureType;
            particle.regionId = regionId;
            ApplyParticleBlue(&particle, seed, animationTrailSettings.colorVariation);
            IncludeOverlayPoint(overlay, particle);
            if (diagnostics != nullptr) {
                ++diagnostics->emittedParticleCount;
            }
        }
    }
    if (diagnostics != nullptr) {
        diagnostics->particleMs += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - particleStart).count();
    }
}

void IncludeWaterPathAnchorsOnly(
    WaterOverlay* overlay,
    std::vector<WaterOverlayPoint> path) {
    if (overlay == nullptr || path.empty()) {
        return;
    }

    const auto pathStartIndex = static_cast<std::uint32_t>(
        std::min<std::size_t>(overlay->points.size(), std::numeric_limits<std::uint32_t>::max()));
    const auto pathPointCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(path.size(), std::numeric_limits<std::uint32_t>::max()));
    for (auto& anchor : path) {
        anchor.particleRole = 0.0F;
        anchor.pathStartIndex = static_cast<float>(pathStartIndex);
        anchor.pathPointCount = static_cast<float>(pathPointCount);
        anchor.jitterSeed = Hash01(
            static_cast<std::uint32_t>(anchor.flowId * 4099.0F + anchor.pathDistance * 6553.0F));
        IncludeOverlayPoint(overlay, anchor);
    }
}

void WriteFloat(std::ofstream& output, float value) {
    output.write(reinterpret_cast<const char*>(&value), sizeof(float));
}

void WriteUchar(std::ofstream& output, std::uint8_t value) {
    output.write(reinterpret_cast<const char*>(&value), sizeof(std::uint8_t));
}

}  // namespace

const char* WaterScaleModeName(WaterScaleMode mode) {
    switch (mode) {
        case WaterScaleMode::Aerial:
            return "aerial";
        case WaterScaleMode::Detail:
            return "detail";
        case WaterScaleMode::Mid:
            return "mid";
    }
    return "mid";
}

const char* WaterEmitterOriginName(WaterEmitterOrigin origin) {
    switch (origin) {
        case WaterEmitterOrigin::AutoSuggested:
            return "auto";
        case WaterEmitterOrigin::Propagated:
            return "propagated";
        case WaterEmitterOrigin::Manual:
            return "manual";
    }
    return "manual";
}

const char* WaterEmitterStatusName(WaterEmitterStatus status) {
    switch (status) {
        case WaterEmitterStatus::Candidate:
            return "candidate";
        case WaterEmitterStatus::Disabled:
            return "disabled";
        case WaterEmitterStatus::Accepted:
            return "accepted";
    }
    return "accepted";
}

WaterPathGenerationSettings DefaultWaterPathGenerationSettings(WaterScaleMode mode) {
    WaterPathGenerationSettings settings;
    settings.legacyScaleMode = mode;
    switch (mode) {
        case WaterScaleMode::Aerial:
            settings.autoTune = true;
            settings.supportVoxelSize = 1.25F;
            settings.maxBridgeDistance = 4.0F;
            settings.smoothing = 0.65F;
            settings.pathLength = 180.0F;
            settings.pathSampleSpacing = 1.2F;
            settings.branching = 0.55F;
            settings.coverage = 0.45F;
            settings.gapTolerance = 0.78F;
            settings.maxSteps = 260;
            settings.supportSampleLimit = 160000;
            break;
        case WaterScaleMode::Detail:
            settings.autoTune = true;
            settings.supportVoxelSize = 0.006F;
            settings.maxBridgeDistance = 0.065F;
            settings.smoothing = 0.46F;
            settings.pathLength = 3.5F;
            settings.pathSampleSpacing = 0.006F;
            settings.branching = 0.78F;
            settings.coverage = 0.85F;
            settings.gapTolerance = 0.70F;
            settings.maxSteps = 1000;
            settings.supportSampleLimit = 600000;
            break;
        case WaterScaleMode::Mid:
            settings.autoTune = true;
            settings.supportVoxelSize = 0.008F;
            settings.maxBridgeDistance = 0.080F;
            settings.smoothing = 0.45F;
            settings.pathSampleSpacing = 0.008F;
            settings.branching = 0.70F;
            settings.coverage = 0.65F;
            settings.gapTolerance = 0.62F;
            settings.maxSteps = 2200;
            settings.supportSampleLimit = 450000;
            break;
    }
    return settings;
}

WaterSourceSettings DefaultWaterSourceSettings(WaterScaleMode mode) {
    WaterSourceSettings settings;
    settings.path = DefaultWaterPathGenerationSettings(mode);
    switch (mode) {
        case WaterScaleMode::Aerial:
            settings.trailShape.splineAnchorSpacing = 0.65F;
            settings.trailShape.particleJitter = 0.28F;
            settings.trailShape.trailLaneCount = 5U;
            settings.trailShape.trailLooseness = 0.55F;
            settings.trailShape.trailSmoothness = 0.55F;
            break;
        case WaterScaleMode::Detail:
            settings.trailShape.splineAnchorSpacing = 0.035F;
            settings.trailShape.particleJitter = 0.24F;
            settings.trailShape.trailLaneCount = 9U;
            settings.trailShape.trailLooseness = 0.32F;
            settings.trailShape.trailSmoothness = 0.55F;
            break;
        case WaterScaleMode::Mid:
            settings.trailShape.splineAnchorSpacing = 0.075F;
            settings.trailShape.particleJitter = 0.28F;
            settings.trailShape.trailLaneCount = 7U;
            settings.trailShape.trailLooseness = 0.45F;
            settings.trailShape.trailSmoothness = 0.55F;
            break;
    }
    return settings;
}

WaterAnimationTrailSettings DefaultWaterAnimationTrailSettings() {
    return {};
}

WaterCausticLookSettings DefaultWaterCausticLookSettings() {
    return {};
}

WaterVisualSettings DefaultWaterVisualSettings() {
    return {};
}

WaterSettingsBundle DefaultWaterSettingsBundle(WaterScaleMode mode) {
    WaterSettingsBundle settings;
    settings.path = DefaultWaterPathGenerationSettings(mode);
    const auto sourceSettings = DefaultWaterSourceSettings(mode);
    settings.trail.particleJitter = sourceSettings.trailShape.particleJitter;
    settings.trail.splineAnchorSpacing = sourceSettings.trailShape.splineAnchorSpacing;
    return settings;
}

WaterBakeSettings DefaultWaterBakeSettings(WaterScaleMode mode) {
    return DefaultWaterPathGenerationSettings(mode);
}

bool WaterPathBakeInputsEqual(
    const WaterPathGenerationSettings& left,
    const WaterPathGenerationSettings& right) {
    return left.legacyScaleMode == right.legacyScaleMode &&
           left.autoTune == right.autoTune &&
           left.supportVoxelSize == right.supportVoxelSize &&
           left.maxBridgeDistance == right.maxBridgeDistance &&
           left.pathLength == right.pathLength &&
           left.pathSampleSpacing == right.pathSampleSpacing &&
           left.branching == right.branching &&
           left.coverage == right.coverage &&
           left.gapTolerance == right.gapTolerance &&
           left.maxSteps == right.maxSteps &&
           left.supportSampleLimit == right.supportSampleLimit;
}

bool WaterSourceBakeInputsEqual(
    const WaterSourceSettings& left,
    const WaterSourceSettings& right) {
    return WaterPathBakeInputsEqual(left.path, right.path);
}

const WaterSourceSettings& ResolveWaterSourceSettings(
    const WaterEmitter& emitter,
    const WaterSourceSettings& defaultSettings) {
    if (emitter.tempSourceSettings.has_value()) {
        return emitter.tempSourceSettings.value();
    }
    if (emitter.sourceSettingsAssignment == WaterSourceSettingsAssignment::Custom &&
        emitter.sourceSettings.has_value()) {
        return emitter.sourceSettings.value();
    }
    return defaultSettings;
}

const WaterSourceSettings& ResolveWaterSourceSettings(
    const WaterEmitter& emitter,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings) {
    if (emitter.tempSourceSettings.has_value()) {
        return emitter.tempSourceSettings.value();
    }
    if (emitter.sourceSettingsAssignment == WaterSourceSettingsAssignment::Custom &&
        emitter.sourceSettings.has_value()) {
        return emitter.sourceSettings.value();
    }
    if (emitter.sourceSettingsAssignment == WaterSourceSettingsAssignment::LinkedEmitter &&
        emitter.linkedSourceSettingsEmitterId.has_value() &&
        emitter.linkedSourceSettingsEmitterId.value() != emitter.id) {
        const auto linkedIt = std::find_if(
            emitters.begin(),
            emitters.end(),
            [&](const WaterEmitter& candidate) {
                return candidate.id == emitter.linkedSourceSettingsEmitterId.value() &&
                       candidate.sourceSettings.has_value();
            });
        if (linkedIt != emitters.end()) {
            return linkedIt->sourceSettings.value();
        }
    }
    return defaultSettings;
}

std::vector<WaterEmitter> SuggestWaterEmitters(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& existingEmitters,
    const WaterPathGenerationSettings& settings,
    std::uint32_t firstEmitterId,
    std::uint32_t maxSuggestions) {
    std::vector<WaterEmitter> suggestions;
    if (cloud.positions.empty() || maxSuggestions == 0) {
        return suggestions;
    }

    auto graph = BuildSupportGraph(cloud, settings);
    if (graph.points.empty()) {
        return suggestions;
    }

    std::vector<CandidateScore> scored;
    scored.reserve(graph.points.size() / 8U + 1U);
    const float duplicateRadius = std::max(settings.maxBridgeDistance * 4.0F, settings.supportVoxelSize * 6.0F);
    const float duplicateRadiusSquared = duplicateRadius * duplicateRadius;

    for (std::uint32_t index = 0; index < graph.points.size(); ++index) {
        const auto& point = graph.points[index];
        if (point.confidence < 0.56F) {
            continue;
        }

        bool nearExisting = false;
        for (const auto& emitter : existingEmitters) {
            const auto delta = point.position - ToGlm(emitter.position);
            if (glm::dot(delta, delta) <= duplicateRadiusSquared) {
                nearExisting = true;
                break;
            }
        }
        if (nearExisting) {
            continue;
        }

        const auto neighbours = NearbySupportIndices(graph, point.position, std::max(settings.maxBridgeDistance, graph.cellSize));
        float bestDrop = 0.0F;
        float supportedDownhillCount = 0.0F;
        const auto direction = FlowDirection(point);
        for (const auto neighbourIndex : neighbours) {
            if (neighbourIndex == index || neighbourIndex >= graph.points.size()) {
                continue;
            }
            const auto delta = graph.points[neighbourIndex].position - point.position;
            const float distance = SafeLength(delta);
            if (distance <= 1.0e-5F || distance > settings.maxBridgeDistance) {
                continue;
            }
            const float drop = point.position.z - graph.points[neighbourIndex].position.z;
            const float alignment = glm::dot(glm::normalize(delta), direction);
            if (drop > settings.maxBridgeDistance * 0.08F && alignment > 0.12F) {
                bestDrop = std::max(bestDrop, drop / distance);
                supportedDownhillCount += 1.0F;
            }
        }

        if (supportedDownhillCount < 2.0F || bestDrop <= 0.05F) {
            continue;
        }

        const float verticalFace = point.hasNormal ? 1.0F - std::abs(point.normal.z) : 0.35F;
        if (settings.legacyScaleMode != WaterScaleMode::Aerial && verticalFace < 0.28F) {
            continue;
        }
        const float score =
            (point.confidence * 0.42F) +
            (Clamp01(verticalFace) * 0.22F) +
            (Clamp01(bestDrop) * 0.24F) +
            (Clamp01(supportedDownhillCount / 8.0F) * 0.12F);
        if (score >= 0.62F) {
            scored.push_back({.supportIndex = index, .score = score, .confidence = Clamp01(score)});
        }
    }

    std::sort(
        scored.begin(),
        scored.end(),
        [](const CandidateScore& left, const CandidateScore& right) {
            if (std::abs(left.score - right.score) > 1.0e-6F) {
                return left.score > right.score;
            }
            return left.supportIndex < right.supportIndex;
        });

    std::vector<glm::vec3> acceptedPositions;
    acceptedPositions.reserve(maxSuggestions);
    for (const auto& candidate : scored) {
        if (suggestions.size() >= maxSuggestions || candidate.supportIndex >= graph.points.size()) {
            break;
        }

        const auto& point = graph.points[candidate.supportIndex];
        bool tooClose = false;
        for (const auto& position : acceptedPositions) {
            const auto delta = point.position - position;
            if (glm::dot(delta, delta) <= duplicateRadiusSquared) {
                tooClose = true;
                break;
            }
        }
        if (tooClose) {
            continue;
        }

        WaterEmitter emitter;
        emitter.id = firstEmitterId + static_cast<std::uint32_t>(suggestions.size());
        emitter.name = "Auto Source " + std::to_string(emitter.id);
        emitter.position = FromGlm(point.position);
        emitter.radius = std::max(settings.supportVoxelSize * 3.0F, settings.maxBridgeDistance * 0.75F);
        emitter.strength = 0.75F + candidate.confidence * 0.5F;
        emitter.speed = settings.legacyScaleMode == WaterScaleMode::Aerial ? 0.45F : 1.0F;
        emitter.scope = settings.legacyScaleMode;
        emitter.origin = WaterEmitterOrigin::AutoSuggested;
        emitter.status = WaterEmitterStatus::Candidate;
        emitter.confidence = candidate.confidence;
        suggestions.push_back(emitter);
        acceptedPositions.push_back(point.position);
    }

    return suggestions;
}

std::optional<invisible_places::io::Float3> SnapEmitterToCloud(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::io::Float3& position,
    const WaterPathGenerationSettings& settings) {
    auto graph = BuildSupportGraph(cloud, settings);
    const auto nearest = NearestSupportIndex(
        graph,
        ToGlm(position),
        std::max(settings.maxBridgeDistance * 8.0F, settings.supportVoxelSize * 8.0F));
    if (!nearest.has_value() || nearest.value() >= graph.points.size()) {
        return std::nullopt;
    }
    return FromGlm(graph.points[nearest.value()].position);
}

WaterPathCache GenerateWaterPathCache(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterPathGenerationSettings& settings) {
    WaterPathCache cache;
    cache.supportLayerPath = cloud.sourcePath;
    cache.requestedSettings = settings;
    cache.tunedSettings = TuneWaterPathSettings(cloud, settings, &cache.diagnostics);
    if (cloud.positions.empty() || emitters.empty()) {
        cache.diagnostics.summary = "No water cache generated: missing support points or emitters.";
        return cache;
    }

    auto graph = BuildSupportGraph(cloud, cache.tunedSettings);
    if (graph.points.empty()) {
        cache.diagnostics.summary = "No water cache generated: support graph was empty.";
        return cache;
    }

    std::uint32_t branchId = 0;
    std::uint32_t pilotTraceCount = 0;
    std::unordered_set<std::uint32_t> occupiedSupport;
    for (const auto& emitter : emitters) {
        if (emitter.status == WaterEmitterStatus::Disabled) {
            continue;
        }
        const auto startIndex = NearestSupportIndex(
            graph,
            ToGlm(emitter.position),
            std::max(cache.tunedSettings.maxBridgeDistance * 4.0F, emitter.radius * 2.0F));
        if (!startIndex.has_value() || startIndex.value() >= graph.points.size()) {
            continue;
        }

        ++branchId;
        TraceResult mainTrace = TraceWaterPathBranch(
            graph,
            emitter,
            cache.tunedSettings,
            branchId,
            WaterPathBranchRole::Main,
            std::nullopt,
            startIndex.value(),
            nullptr,
            true);
        ++pilotTraceCount;
        if (mainTrace.branch.rawAnchors.size() >= 2U) {
            for (const auto supportIndex : mainTrace.visitedSupportIndices) {
                occupiedSupport.insert(supportIndex);
            }
            cache.branches.push_back(std::move(mainTrace.branch));
        }

        std::sort(
            mainTrace.opportunities.begin(),
            mainTrace.opportunities.end(),
            [](const BranchOpportunity& left, const BranchOpportunity& right) {
                if (std::abs(left.score - right.score) > 1.0e-6F) {
                    return left.score > right.score;
                }
                return left.startSupportIndex < right.startSupportIndex;
            });

        const std::uint32_t branchLimit = static_cast<std::uint32_t>(
            1U + std::round(cache.tunedSettings.branching * 5.0F) +
            std::round(cache.tunedSettings.coverage * 5.0F));
        std::uint32_t emittedBranches = 0U;
        std::vector<glm::vec3> branchStarts;
        branchStarts.reserve(branchLimit);
        for (const auto& opportunity : mainTrace.opportunities) {
            if (emittedBranches >= branchLimit ||
                opportunity.startSupportIndex >= graph.points.size()) {
                continue;
            }
            if (occupiedSupport.contains(opportunity.startSupportIndex) &&
                opportunity.flatness < 0.55F) {
                continue;
            }

            const auto startPosition = graph.points[opportunity.startSupportIndex].position;
            const float duplicateRadius = std::max(
                cache.tunedSettings.supportVoxelSize * 1.2F,
                cache.tunedSettings.maxBridgeDistance * 0.12F);
            bool duplicateStart = false;
            for (const auto& previousStart : branchStarts) {
                const auto delta = previousStart - startPosition;
                if (glm::dot(delta, delta) <= duplicateRadius * duplicateRadius) {
                    duplicateStart = true;
                    break;
                }
            }
            if (duplicateStart) {
                continue;
            }

            ++branchId;
            auto branchSettings = cache.tunedSettings;
            branchSettings.pathLength *= 0.35F + cache.tunedSettings.coverage * 0.35F;
            branchSettings.maxSteps = std::max<std::uint32_t>(
                12U,
                static_cast<std::uint32_t>(
                    std::ceil(static_cast<float>(cache.tunedSettings.maxSteps) *
                              (0.35F + cache.tunedSettings.coverage * 0.30F))));
            TraceResult branchTrace = TraceWaterPathBranch(
                graph,
                emitter,
                branchSettings,
                branchId,
                opportunity.role,
                opportunity.parentBranchId,
                opportunity.startSupportIndex,
                &occupiedSupport,
                false);
            ++pilotTraceCount;
            if (branchTrace.branch.rawAnchors.size() < 2U ||
                branchTrace.branch.length < std::max(0.02F, cache.tunedSettings.pathSampleSpacing * 2.0F)) {
                continue;
            }
            for (const auto supportIndex : branchTrace.visitedSupportIndices) {
                occupiedSupport.insert(supportIndex);
            }
            branchStarts.push_back(startPosition);
            cache.branches.push_back(std::move(branchTrace.branch));
            ++emittedBranches;
        }
    }

    float confidenceSum = 0.0F;
    for (const auto& branch : cache.branches) {
        confidenceSum += branch.confidence;
        if (branch.confidence < 0.45F || branch.gapCount >= 2U) {
            ++cache.diagnostics.lowConfidenceBranchCount;
        }
    }
    cache.diagnostics.pilotTraceCount = pilotTraceCount;
    cache.diagnostics.branchCount = static_cast<std::uint32_t>(cache.branches.size());
    cache.diagnostics.averageConfidence =
        cache.branches.empty() ? 0.0F : Clamp01(confidenceSum / static_cast<float>(cache.branches.size()));
    if (cache.diagnostics.summary.empty() || cache.branches.empty()) {
        cache.diagnostics.summary =
            cache.branches.empty()
                ? "No supported water branches reached from the selected emitters."
                : "Generated water branch cache.";
    }
    return cache;
}

WaterPathCache GenerateWaterPathCache(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings) {
    WaterPathCache combined;
    combined.supportLayerPath = cloud.sourcePath;
    combined.requestedSettings = defaultSettings.path;
    combined.tunedSettings = defaultSettings.path;
    std::uint32_t nextBranchId = 1U;
    std::uint32_t totalPilotTraces = 0U;
    float confidenceSum = 0.0F;

    for (const auto& emitter : emitters) {
        if (emitter.status == WaterEmitterStatus::Disabled) {
            continue;
        }
        const auto& settings = ResolveWaterSourceSettings(emitter, emitters, defaultSettings);
        auto cache = GenerateWaterPathCache(cloud, std::vector<WaterEmitter>{emitter}, settings.path);
        if (combined.diagnostics.estimatedPointSpacing <= 0.0F) {
            combined.diagnostics.estimatedPointSpacing = cache.diagnostics.estimatedPointSpacing;
        }
        combined.diagnostics.supportVoxelSize =
            std::max(combined.diagnostics.supportVoxelSize, cache.diagnostics.supportVoxelSize);
        combined.diagnostics.maxBridgeDistance =
            std::max(combined.diagnostics.maxBridgeDistance, cache.diagnostics.maxBridgeDistance);
        combined.diagnostics.pathSampleSpacing =
            combined.diagnostics.pathSampleSpacing <= 0.0F
                ? cache.diagnostics.pathSampleSpacing
                : std::min(combined.diagnostics.pathSampleSpacing, cache.diagnostics.pathSampleSpacing);
        combined.diagnostics.branchSearchRadius =
            std::max(combined.diagnostics.branchSearchRadius, cache.diagnostics.branchSearchRadius);
        combined.diagnostics.iterationCount =
            std::max(combined.diagnostics.iterationCount, cache.diagnostics.iterationCount);
        totalPilotTraces += cache.diagnostics.pilotTraceCount;

        std::unordered_map<std::uint32_t, std::uint32_t> remappedIds;
        for (const auto& branch : cache.branches) {
            remappedIds[branch.id] = nextBranchId++;
        }
        for (auto branch : cache.branches) {
            const auto originalId = branch.id;
            branch.id = remappedIds[originalId];
            if (branch.parentId.has_value()) {
                const auto parentIt = remappedIds.find(branch.parentId.value());
                if (parentIt != remappedIds.end()) {
                    branch.parentId = parentIt->second;
                } else {
                    branch.parentId.reset();
                }
            }
            for (auto& point : branch.rawAnchors) {
                point.flowId = static_cast<float>(branch.id);
            }
            confidenceSum += branch.confidence;
            if (branch.confidence < 0.45F || branch.gapCount >= 2U) {
                ++combined.diagnostics.lowConfidenceBranchCount;
            }
            combined.branches.push_back(std::move(branch));
        }
    }
    combined.diagnostics.pilotTraceCount = totalPilotTraces;
    combined.diagnostics.branchCount = static_cast<std::uint32_t>(combined.branches.size());
    combined.diagnostics.averageConfidence =
        combined.branches.empty() ? 0.0F : Clamp01(confidenceSum / static_cast<float>(combined.branches.size()));
    combined.diagnostics.summary =
        combined.branches.empty()
            ? "No supported water branches reached from the selected emitters."
            : "Generated per-source water branch cache.";
    return combined;
}

WaterOverlay BuildWaterPathAnchorsFromCache(
    const WaterPathCache& cache,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings) {
    std::unordered_set<std::uint32_t> hiddenIds{
        cache.hiddenBranchIds.begin(),
        cache.hiddenBranchIds.end()};
    std::unordered_map<std::uint32_t, const WaterEmitter*> emitterById;
    emitterById.reserve(emitters.size());
    for (const auto& emitter : emitters) {
        emitterById[emitter.id] = &emitter;
    }

    WaterOverlay overlay;
    for (const auto& branch : cache.branches) {
        if (hiddenIds.contains(branch.id) || branch.rawAnchors.empty()) {
            continue;
        }
        std::vector<WaterOverlayPoint> path = branch.rawAnchors;
        const auto emitterIt = emitterById.find(branch.emitterId);
        const auto& sourceSettings =
            emitterIt == emitterById.end()
                ? defaultSettings
                : ResolveWaterSourceSettings(*emitterIt->second, emitters, defaultSettings);
        for (auto& point : path) {
            point.flowId = static_cast<float>(branch.id);
            point.emitterId = static_cast<float>(branch.emitterId);
            point.particleRole = 0.0F;
            point.featureType =
                branch.role == WaterPathBranchRole::Main
                    ? 0.0F
                    : (branch.role == WaterPathBranchRole::Spread ? 0.35F : 0.25F);
            if (branch.confidence < 0.45F || branch.gapCount >= 2U) {
                point.confidence = std::min(point.confidence, branch.confidence);
            }
        }
        SmoothWaterPath(&path, sourceSettings.path.smoothing, std::max(0.001F, branch.length));
        IncludeWaterPathAnchorsOnly(&overlay, std::move(path));
    }
    return overlay;
}

WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterPathGenerationSettings& settings) {
    WaterSourceSettings defaultSettings;
    defaultSettings.path = settings;
    return BuildWaterPathAnchorsFromCache(
        GenerateWaterPathCache(cloud, emitters, settings),
        emitters,
        defaultSettings);
}

WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings) {
    return BuildWaterPathAnchorsFromCache(
        GenerateWaterPathCache(cloud, emitters, defaultSettings),
        emitters,
        defaultSettings);
}

std::shared_ptr<const TrailSurfaceIndex> BuildTrailSurfaceIndex(
    const invisible_places::io::LoadedPointCloud& cloud) {
    const auto startedAt = std::chrono::steady_clock::now();
    auto index = std::make_shared<TrailSurfaceIndex>();
    if (cloud.positions.empty()) {
        return index;
    }

    const float estimatedSpacing = PositiveOr(EstimatePointSpacing(cloud), 0.02F);
    index->searchRadius = std::clamp(std::max(estimatedSpacing * 7.5F, 0.035F), 0.008F, 5.0F);
    index->cellSize = std::max(index->searchRadius * 0.75F, 0.005F);
    index->surfaceLift = std::clamp(estimatedSpacing * 0.10F, 0.0025F, 0.012F);

    constexpr std::uint32_t kTrailSurfaceSampleLimit = 220000U;
    const auto stride = SampleStride(cloud.positions.size(), kTrailSurfaceSampleLimit);
    std::unordered_map<std::uint64_t, TrailSurfaceCellAccumulator> accumulators;
    accumulators.reserve((cloud.positions.size() + stride - 1U) / stride);
    for (std::size_t pointIndex = 0; pointIndex < cloud.positions.size(); pointIndex += stride) {
        const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
        if (!IsValidPoint(position)) {
            continue;
        }
        const auto key = MakeXyGridKey(position, index->cellSize);
        auto& accumulator = accumulators[EncodeTrailSurfaceGridKey(key.x, key.y)];
        accumulator.positionSum += position;
        accumulator.minZ = std::min(accumulator.minZ, position.z);
        accumulator.maxZ = std::max(accumulator.maxZ, position.z);
        ++accumulator.count;
        ++index->sampledPointCount;
        if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
            const glm::vec3 normal = ToGlm(cloud.normals[pointIndex]);
            if (glm::dot(normal, normal) > kNormalEpsilon) {
                accumulator.normalSum += glm::normalize(normal);
                ++accumulator.normalCount;
            }
        }
    }

    index->cells.reserve(accumulators.size());
    index->cellLookup.reserve(accumulators.size());
    for (const auto& [key, accumulator] : accumulators) {
        if (accumulator.count == 0U) {
            continue;
        }
        TrailSurfaceIndexCell cell;
        const float count = static_cast<float>(accumulator.count);
        cell.position = FromGlm(accumulator.positionSum / count);
        cell.minZ = accumulator.minZ;
        cell.maxZ = accumulator.maxZ;
        cell.confidence = Clamp01(std::sqrt(count) * 0.35F);
        cell.count = accumulator.count;
        if (accumulator.normalCount > 0U && glm::dot(accumulator.normalSum, accumulator.normalSum) > kNormalEpsilon) {
            cell.normal = FromGlm(glm::normalize(accumulator.normalSum));
            cell.hasNormal = true;
        }
        const auto cellIndex = static_cast<std::uint32_t>(
            std::min<std::size_t>(index->cells.size(), std::numeric_limits<std::uint32_t>::max()));
        index->cellLookup[key] = cellIndex;
        index->cells.push_back(cell);
    }

    index->buildMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - startedAt).count();
    return index;
}

std::shared_ptr<const TrailSurfaceIndex> BuildTrailSurfaceIndex(
    const invisible_places::io::LoadedPointCloud* cloud) {
    return cloud == nullptr ? nullptr : BuildTrailSurfaceIndex(*cloud);
}

std::uint64_t TrailSurfaceIndexSampleCount(const TrailSurfaceIndex& index) {
    return index.sampledPointCount;
}

double TrailSurfaceIndexBuildMilliseconds(const TrailSurfaceIndex& index) {
    return index.buildMilliseconds;
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings) {
    return BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        trailShapeSettings,
        animationTrailSettings,
        static_cast<const TrailSurfaceIndex*>(nullptr));
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const invisible_places::io::LoadedPointCloud* supportCloud,
    WaterTrailBuildQuality quality,
    WaterTrailBuildDiagnostics* diagnostics) {
    const auto surfaceIndex = BuildTrailSurfaceIndex(supportCloud);
    if (diagnostics != nullptr && surfaceIndex != nullptr) {
        diagnostics->surfaceIndexBuildMs += surfaceIndex->buildMilliseconds;
        diagnostics->surfaceSampleCount = surfaceIndex->sampledPointCount;
    }
    return BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        trailShapeSettings,
        animationTrailSettings,
        surfaceIndex == nullptr ? nullptr : surfaceIndex.get(),
        quality,
        diagnostics);
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality,
    WaterTrailBuildDiagnostics* diagnostics) {
    if (diagnostics != nullptr && surfaceIndex != nullptr) {
        diagnostics->surfaceSampleCount = surfaceIndex->sampledPointCount;
    }
    WaterOverlay overlay;
    std::vector<WaterOverlayPoint> currentPath;
    float currentFlowId = -1.0F;
    for (const auto& point : pathAnchors.points) {
        if (point.particleRole >= 0.5F) {
            continue;
        }
        if (!currentPath.empty() && std::abs(point.flowId - currentFlowId) > 1.0e-4F) {
            IncludeWaterPathWithParticles(
                &overlay,
                std::move(currentPath),
                trailShapeSettings,
                animationTrailSettings,
                0.0F,
                0.0F,
                surfaceIndex,
                quality,
                diagnostics);
            currentPath.clear();
        }
        currentFlowId = point.flowId;
        currentPath.push_back(point);
    }
    if (!currentPath.empty()) {
        IncludeWaterPathWithParticles(
            &overlay,
            std::move(currentPath),
            trailShapeSettings,
            animationTrailSettings,
            0.0F,
            0.0F,
            surfaceIndex,
            quality,
            diagnostics);
    }
    return overlay;
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailSettings& legacyTrailSettings,
    const WaterParticleVisualSettings& legacyVisualSettings) {
    WaterParticleTrailShapeSettings trailShapeSettings;
    trailShapeSettings.particleJitter = legacyTrailSettings.particleJitter;
    trailShapeSettings.splineAnchorSpacing = legacyTrailSettings.splineAnchorSpacing;
    WaterAnimationTrailSettings animationTrailSettings;
    animationTrailSettings.particleDensity = legacyTrailSettings.particleDensity;
    animationTrailSettings.particleSpeed = legacyTrailSettings.particleSpeed;
    animationTrailSettings.colorVariation = legacyVisualSettings.colorVariation;
    return BuildWaterOverlayFromPathAnchors(pathAnchors, trailShapeSettings, animationTrailSettings);
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings) {
    return BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        emitters,
        defaultSettings,
        animationTrailSettings,
        static_cast<const TrailSurfaceIndex*>(nullptr));
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const invisible_places::io::LoadedPointCloud* supportCloud,
    WaterTrailBuildQuality quality,
    WaterTrailBuildDiagnostics* diagnostics) {
    const auto surfaceIndex = BuildTrailSurfaceIndex(supportCloud);
    if (diagnostics != nullptr && surfaceIndex != nullptr) {
        diagnostics->surfaceIndexBuildMs += surfaceIndex->buildMilliseconds;
        diagnostics->surfaceSampleCount = surfaceIndex->sampledPointCount;
    }
    return BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        emitters,
        defaultSettings,
        animationTrailSettings,
        surfaceIndex == nullptr ? nullptr : surfaceIndex.get(),
        quality,
        diagnostics);
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality,
    WaterTrailBuildDiagnostics* diagnostics) {
    if (diagnostics != nullptr && surfaceIndex != nullptr) {
        diagnostics->surfaceSampleCount = surfaceIndex->sampledPointCount;
    }
    std::unordered_map<std::uint32_t, const WaterEmitter*> emitterById;
    emitterById.reserve(emitters.size());
    for (const auto& emitter : emitters) {
        emitterById[emitter.id] = &emitter;
    }

    WaterOverlay overlay;
    std::vector<WaterOverlayPoint> currentPath;
    float currentFlowId = -1.0F;
    auto flushPath = [&]() {
        if (currentPath.empty()) {
            return;
        }
        const auto emitterId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(currentPath.front().emitterId + 0.5F)));
        const auto emitterIt = emitterById.find(emitterId);
        const auto& sourceSettings =
            emitterIt == emitterById.end()
                ? defaultSettings
                : ResolveWaterSourceSettings(*emitterIt->second, emitters, defaultSettings);
        IncludeWaterPathWithParticles(
            &overlay,
            std::move(currentPath),
            sourceSettings.trailShape,
            animationTrailSettings,
            0.0F,
            0.0F,
            surfaceIndex,
            quality,
            diagnostics);
        currentPath.clear();
    };

    for (const auto& point : pathAnchors.points) {
        if (point.particleRole >= 0.5F) {
            continue;
        }
        if (!currentPath.empty() && std::abs(point.flowId - currentFlowId) > 1.0e-4F) {
            flushPath();
        }
        currentFlowId = point.flowId;
        currentPath.push_back(point);
    }
    flushPath();
    return overlay;
}

WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSourceSettings,
    const WaterAnimationTrailSettings& animationTrailSettings) {
    return BuildWaterOverlayFromPathAnchors(
        GenerateWaterPathAnchors(cloud, emitters, defaultSourceSettings),
        emitters,
        defaultSourceSettings,
        animationTrailSettings,
        &cloud);
}

WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSettingsBundle& settings) {
    WaterSourceSettings sourceSettings;
    sourceSettings.path = settings.path;
    sourceSettings.trailShape.particleJitter = settings.trail.particleJitter;
    sourceSettings.trailShape.splineAnchorSpacing = settings.trail.splineAnchorSpacing;
    WaterAnimationTrailSettings animationTrailSettings;
    animationTrailSettings.particleDensity = settings.trail.particleDensity;
    animationTrailSettings.particleSpeed = settings.trail.particleSpeed;
    animationTrailSettings.colorVariation = settings.visual.colorVariation;
    return GenerateWaterOverlay(cloud, emitters, sourceSettings, animationTrailSettings);
}

std::vector<invisible_places::io::Float3> BuildWaterRegionBoundary(
    const std::vector<invisible_places::io::Float3>& vertices);

namespace {

void IncludeStreamSample(WaterStreamOverlay* overlay, WaterStreamSample sample) {
    if (overlay == nullptr) {
        return;
    }
    sample.normal = FromGlm(SafeOverlayNormal(ToGlm(sample.normal)));
    glm::vec3 tangent = ToGlm(sample.tangent);
    if (!IsValidPoint(tangent) || glm::dot(tangent, tangent) <= kNormalEpsilon) {
        tangent = {1.0F, 0.0F, 0.0F};
    }
    sample.tangent = FromGlm(glm::normalize(tangent));
    overlay->bounds.Expand(sample.position);
    overlay->samples.push_back(sample);
}

void IncludeEffectPoint(WaterEffectOverlay* overlay, WaterEffectPoint point) {
    if (overlay == nullptr) {
        return;
    }
    point.normal = FromGlm(SafeOverlayNormal(ToGlm(point.normal)));
    glm::vec3 tangent = ToGlm(point.tangent);
    if (!IsValidPoint(tangent) || glm::dot(tangent, tangent) <= kNormalEpsilon) {
        tangent = {1.0F, 0.0F, 0.0F};
    }
    point.tangent = FromGlm(glm::normalize(tangent));
    overlay->bounds.Expand(point.position);
    overlay->points.push_back(point);
}

std::vector<std::vector<WaterOverlayPoint>> GroupAnchorPaths(const WaterOverlay& pathAnchors) {
    std::vector<std::vector<WaterOverlayPoint>> paths;
    std::vector<WaterOverlayPoint> currentPath;
    float currentFlowId = -1.0F;
    for (const auto& point : pathAnchors.points) {
        if (point.particleRole >= 0.5F) {
            continue;
        }
        if (!currentPath.empty() && std::abs(point.flowId - currentFlowId) > 1.0e-4F) {
            if (currentPath.size() >= 2U) {
                paths.push_back(std::move(currentPath));
            }
            currentPath.clear();
        }
        currentFlowId = point.flowId;
        currentPath.push_back(point);
    }
    if (currentPath.size() >= 2U) {
        paths.push_back(std::move(currentPath));
    }
    return paths;
}

float PathLengthMeters(const std::vector<WaterOverlayPoint>& path) {
    if (path.size() < 2U) {
        return 0.0F;
    }
    float length = 0.0F;
    for (std::size_t index = 1U; index < path.size(); ++index) {
        length += glm::length(ToGlm(path[index].position) - ToGlm(path[index - 1U].position));
    }
    return length;
}

WaterOverlayPoint InterpolatePathByArcLength(const std::vector<WaterOverlayPoint>& path, float distanceMeters) {
    if (path.empty()) {
        return {};
    }
    if (path.size() == 1U || distanceMeters <= 0.0F) {
        return path.front();
    }

    float travelled = 0.0F;
    for (std::size_t index = 1U; index < path.size(); ++index) {
        const glm::vec3 previous = ToGlm(path[index - 1U].position);
        const glm::vec3 next = ToGlm(path[index].position);
        const float segmentLength = glm::length(next - previous);
        if (segmentLength <= 1.0e-6F) {
            continue;
        }
        if (travelled + segmentLength >= distanceMeters) {
            const float t = std::clamp((distanceMeters - travelled) / segmentLength, 0.0F, 1.0F);
            return BlendPathAnchor(path[index - 1U], path[index], t);
        }
        travelled += segmentLength;
    }
    return path.back();
}

glm::vec3 TangentAtPathDistance(const std::vector<WaterOverlayPoint>& path, float distanceMeters, float probeMeters) {
    const float beforeDistance = std::max(0.0F, distanceMeters - std::max(0.005F, probeMeters));
    const float afterDistance = distanceMeters + std::max(0.005F, probeMeters);
    const glm::vec3 before = ToGlm(InterpolatePathByArcLength(path, beforeDistance).position);
    const glm::vec3 after = ToGlm(InterpolatePathByArcLength(path, afterDistance).position);
    const glm::vec3 tangent = after - before;
    return glm::dot(tangent, tangent) > kNormalEpsilon ? glm::normalize(tangent) : glm::vec3{1.0F, 0.0F, 0.0F};
}

std::uint8_t StreamColorByte(float value) {
    return FloatToByte(std::clamp(value, 0.0F, 1.0F));
}

float EffectPolygonEdgeDistanceXy(
    const glm::vec3& point,
    const std::vector<invisible_places::io::Float3>& polygon) {
    if (polygon.size() < 2U) {
        return 0.0F;
    }
    float distance = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const auto next = (index + 1U) % polygon.size();
        distance = std::min(distance, DistanceToSegmentXy(point, polygon[index], polygon[next]));
    }
    return distance == std::numeric_limits<float>::max() ? 0.0F : distance;
}

float EffectPolygonEdgeDistance3d(
    const glm::vec3& point,
    const std::vector<invisible_places::io::Float3>& polygon) {
    if (polygon.size() < 2U) {
        return 0.0F;
    }
    float distance = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const auto next = (index + 1U) % polygon.size();
        distance = std::min(distance, DistanceToSegment3d(point, polygon[index], polygon[next]));
    }
    return distance == std::numeric_limits<float>::max() ? 0.0F : distance;
}

constexpr float kRippleTwoPi = 6.28318530718F;

float Fract01(float value) {
    return value - std::floor(value);
}

float RippleWavePeak(float phase, float sharpness) {
    const float wave = 0.5F + 0.5F * std::cos(phase * kRippleTwoPi);
    return std::pow(Clamp01(wave), std::max(0.25F, sharpness));
}

float RippleLine(float distance, float width) {
    return 1.0F - SmoothStep(0.0F, std::max(1.0e-5F, width), std::abs(distance));
}

glm::vec3 RippleRegionCentroid(const std::vector<invisible_places::io::Float3>& boundary) {
    if (boundary.empty()) {
        return {0.0F, 0.0F, 0.0F};
    }
    glm::vec3 sum{0.0F};
    for (const auto& point : boundary) {
        sum += ToGlm(point);
    }
    return sum / static_cast<float>(boundary.size());
}

std::uint32_t RippleOverlayTypeSalt(WaterRippleOverlayType type) {
    switch (type) {
        case WaterRippleOverlayType::CausticLace:
            return 101U;
        case WaterRippleOverlayType::LinearRipples:
            return 211U;
        case WaterRippleOverlayType::RadialRipples:
            return 307U;
        case WaterRippleOverlayType::RainRings:
            return 401U;
        case WaterRippleOverlayType::TideBands:
            return 503U;
        case WaterRippleOverlayType::WetSheen:
            return 601U;
        case WaterRippleOverlayType::CurrentThreads:
            return 701U;
        case WaterRippleOverlayType::DropletGlints:
            return 809U;
        case WaterRippleOverlayType::DripTrails:
            return 907U;
        case WaterRippleOverlayType::FoamSparkle:
            return 1009U;
        case WaterRippleOverlayType::SaltMineralShimmer:
            return 1103U;
    }
    return 101U;
}

float RippleCellHash(int cellX, int cellY, std::uint32_t seed, std::uint32_t salt) {
    const auto x = static_cast<std::uint32_t>(cellX);
    const auto y = static_cast<std::uint32_t>(cellY);
    return Hash01((x * 374761393U) ^ (y * 668265263U) ^ (seed * 2246822519U) ^ (salt * 3266489917U));
}

glm::vec2 RippleCellHash2(int cellX, int cellY, std::uint32_t seed, std::uint32_t salt) {
    return {
        RippleCellHash(cellX, cellY, seed, salt),
        RippleCellHash(cellX, cellY, seed, salt + 17U),
    };
}

float RippleBlockNoise(const glm::vec2& uv, float cellSize, std::uint32_t seed, std::uint32_t salt) {
    const float safeCellSize = std::max(0.001F, cellSize);
    const auto cellX = static_cast<int>(std::floor(uv.x / safeCellSize));
    const auto cellY = static_cast<int>(std::floor(uv.y / safeCellSize));
    return RippleCellHash(cellX, cellY, seed, salt);
}

float RippleCausticLaceValue(const glm::vec2& uv, const WaterEffectLayer& layer, std::uint32_t seed) {
    const float cellSize = std::max(0.005F, layer.wavelengthMeters);
    glm::vec2 p = uv / cellSize;
    const float warp = std::clamp(layer.warp, 0.0F, 8.0F);
    p += glm::vec2{
             std::sin((p.y + RegionHash01(seed, 1U, 19U)) * 2.19F),
             std::cos((p.x + RegionHash01(seed, 2U, 23U)) * 2.41F)}
         * warp * 0.22F;

    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    float nearest = std::numeric_limits<float>::max();
    float secondNearest = std::numeric_limits<float>::max();
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const glm::vec2 feature = glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} +
                                      RippleCellHash2(cx, cy, seed, 31U);
            const float distance = glm::length(p - feature);
            if (distance < nearest) {
                secondNearest = nearest;
                nearest = distance;
            } else if (distance < secondNearest) {
                secondNearest = distance;
            }
        }
    }

    const float ridgeDistance = secondNearest - nearest;
    const float lineWidth = std::clamp(0.028F + layer.turbulence * 0.045F, 0.012F, 0.18F);
    const float ridge = 1.0F - SmoothStep(lineWidth, lineWidth * 4.0F, ridgeDistance);
    const float shimmer = 0.72F + 0.28F * RippleBlockNoise(uv, cellSize * 0.55F, seed, 43U);
    return Clamp01(std::pow(Clamp01(ridge), 1.35F) * shimmer);
}

float RippleRainRingValue(const glm::vec2& uv, const WaterEffectLayer& layer, std::uint32_t seed) {
    const float wavelength = std::max(0.005F, layer.wavelengthMeters);
    const float cellSize = std::max(wavelength * 2.8F, 0.025F);
    const glm::vec2 p = uv / cellSize;
    const int baseX = static_cast<int>(std::floor(p.x));
    const int baseY = static_cast<int>(std::floor(p.y));
    float best = 0.0F;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const glm::vec2 center = (glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} +
                                      RippleCellHash2(cx, cy, seed, 59U)) *
                                     cellSize;
            const float distance = glm::length(uv - center);
            const float ring = RippleWavePeak((distance / wavelength) - layer.phase, 8.0F);
            const float envelope = 1.0F - SmoothStep(wavelength * 0.25F, cellSize * 0.82F, distance);
            const float dropSeed = 0.55F + 0.45F * RippleCellHash(cx, cy, seed, 71U);
            best = std::max(best, ring * envelope * dropSeed);
        }
    }
    return Clamp01(best);
}

float RippleDropletValue(const glm::vec2& uv, const WaterEffectLayer& layer, std::uint32_t seed) {
    const float wavelength = std::max(0.005F, layer.wavelengthMeters);
    const float cellSize = std::max(wavelength * 1.65F, 0.025F);
    const glm::vec2 p = uv / cellSize;
    const int cellX = static_cast<int>(std::floor(p.x));
    const int cellY = static_cast<int>(std::floor(p.y));
    const glm::vec2 center =
        (glm::vec2{static_cast<float>(cellX), static_cast<float>(cellY)} +
         RippleCellHash2(cellX, cellY, seed, 83U)) *
        cellSize;
    const float sparseGate = RippleCellHash(cellX, cellY, seed, 89U);
    const float keep = SmoothStep(0.42F - std::clamp(layer.turbulence, 0.0F, 1.0F) * 0.22F, 1.0F, sparseGate);
    const float distance = glm::length(uv - center);
    const float glint = 1.0F - SmoothStep(0.0F, cellSize * 0.22F, distance);
    const float pulse = 0.72F + 0.28F * RippleWavePeak(layer.phase + sparseGate, 2.0F);
    return Clamp01(std::pow(Clamp01(glint), 2.2F) * keep * pulse);
}

float RippleCurrentThreadValue(const glm::vec2& uv, const WaterEffectLayer& layer, std::uint32_t seed) {
    const float wavelength = std::max(0.005F, layer.wavelengthMeters);
    const float threadSpacing = std::max(wavelength * 0.42F, 0.008F);
    const float lane = std::floor(uv.y / threadSpacing);
    const float laneSeed = RippleCellHash(static_cast<int>(lane), 0, seed, 97U);
    const float laneCenter = (lane + 0.18F + laneSeed * 0.64F) * threadSpacing;
    const float line = RippleLine(uv.y - laneCenter, threadSpacing * (0.055F + layer.turbulence * 0.08F));
    const float broken =
        SmoothStep(0.15F, 0.92F, 0.5F + 0.5F * std::sin((uv.x / (wavelength * 2.4F)) + laneSeed * kRippleTwoPi));
    return Clamp01(std::pow(Clamp01(line), 1.5F) * broken);
}

float RippleFoamSparkleValue(
    const glm::vec2& uv,
    const WaterEffectLayer& layer,
    std::uint32_t seed,
    float edge) {
    const float wavelength = std::max(0.005F, layer.wavelengthMeters);
    const float edgeBand = 1.0F - Clamp01(edge);
    const float sparkle = RippleBlockNoise(uv, std::max(wavelength * 0.45F, 0.008F), seed, 107U);
    const float pulse = SmoothStep(0.72F, 1.0F, sparkle);
    return Clamp01((edgeBand * 0.75F + pulse * 0.45F) * SmoothStep(0.12F, 1.0F, edgeBand + pulse * 0.5F));
}

float RippleSaltShimmerValue(const glm::vec2& uv, const WaterEffectLayer& layer, std::uint32_t seed) {
    const float wavelength = std::max(0.005F, layer.wavelengthMeters);
    const float coarse = RippleBlockNoise(uv, std::max(wavelength * 0.70F, 0.012F), seed, 113U);
    const float fine = RippleBlockNoise(uv, std::max(wavelength * 0.18F, 0.004F), seed, 127U);
    const float grain = SmoothStep(0.50F, 1.0F, fine);
    const float slowBand = 0.45F + 0.55F * RippleWavePeak((uv.x + uv.y * 0.37F) / (wavelength * 5.0F) + layer.phase, 1.2F);
    return Clamp01((coarse * 0.28F + grain * 0.72F) * slowBand);
}

struct RipplePatternResult {
    float value = 0.0F;
    float confidence = 1.0F;
    float distance = 0.0F;
    float linearCoord = 0.0F;
    float angle = 0.0F;
    glm::vec3 tangent{1.0F, 0.0F, 0.0F};
};

RipplePatternResult EvaluateRipplePattern(
    const WaterEffectLayer& layer,
    const glm::vec3& position,
    const glm::vec3& normal,
    const glm::vec3& baseTangent,
    const glm::vec3& regionCenter,
    float edge,
    float edgeDistance,
    std::uint32_t pointIndex) {
    RipplePatternResult result;
    result.tangent = baseTangent;
    result.distance = edgeDistance;

    glm::vec3 tangent = baseTangent - normal * glm::dot(baseTangent, normal);
    if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
        tangent = WaterPathLateral(normal);
    } else {
        tangent = glm::normalize(tangent);
    }
    glm::vec3 lateral = glm::cross(normal, tangent);
    if (glm::dot(lateral, lateral) <= kNormalEpsilon) {
        lateral = WaterPathLateral(tangent);
    } else {
        lateral = glm::normalize(lateral);
    }

    const glm::vec3 relative = position - regionCenter;
    const float scale = std::clamp(layer.patternScale, 0.05F, 100.0F);
    const glm::vec2 uv{
        glm::dot(relative, tangent) * scale,
        glm::dot(relative, lateral) * scale,
    };
    const glm::vec2 regionUv{
        relative.x * scale,
        relative.y * scale,
    };
    const float wavelength = std::max(0.005F, layer.wavelengthMeters);
    const float layerPhase = layer.phase + RegionHash01(layer.id + layer.seed, 0U, RippleOverlayTypeSalt(layer.rippleOverlayType));
    const float radialDistance = glm::length(uv);
    const float regionRadialDistance = glm::length(regionUv);
    const float slope = Clamp01(1.0F - std::abs(normal.z));
    const std::uint32_t seed = layer.seed ^ (layer.id * 747796405U) ^ RippleOverlayTypeSalt(layer.rippleOverlayType);

    switch (layer.rippleOverlayType) {
        case WaterRippleOverlayType::CausticLace:
            result.value = RippleCausticLaceValue(uv, layer, seed);
            result.confidence = 0.72F + result.value * 0.28F;
            break;
        case WaterRippleOverlayType::LinearRipples:
            result.value = RippleWavePeak((uv.x / wavelength) + layerPhase, 4.0F);
            result.confidence = 0.65F + result.value * 0.35F;
            break;
        case WaterRippleOverlayType::RadialRipples: {
            result.value = RippleWavePeak((regionRadialDistance / wavelength) - layerPhase, 6.0F);
            result.distance = regionRadialDistance;
            if (regionRadialDistance > 1.0e-5F) {
                glm::vec3 radialTangent{regionUv.x, regionUv.y, 0.0F};
                radialTangent -= normal * glm::dot(radialTangent, normal);
                result.tangent = glm::dot(radialTangent, radialTangent) > kNormalEpsilon
                                     ? glm::normalize(radialTangent)
                                     : tangent;
            }
            result.angle = std::atan2(regionUv.y, regionUv.x);
            result.confidence = 0.60F + result.value * 0.40F;
            break;
        }
        case WaterRippleOverlayType::RainRings:
            result.value = RippleRainRingValue(regionUv, layer, seed);
            result.distance = regionRadialDistance;
            result.confidence = 0.58F + result.value * 0.42F;
            break;
        case WaterRippleOverlayType::TideBands: {
            const float broad = RippleWavePeak((uv.x / (wavelength * 5.0F)) + layerPhase, 1.7F);
            const float secondary = RippleWavePeak(((uv.x + uv.y * 0.22F) / (wavelength * 9.0F)) - layerPhase, 1.1F);
            result.value = Clamp01((broad * 0.72F) + (secondary * 0.28F));
            result.confidence = 0.70F + result.value * 0.30F;
            break;
        }
        case WaterRippleOverlayType::WetSheen: {
            const float lowNoise = RippleBlockNoise(uv, wavelength * 2.5F, seed, 131U);
            result.value = Clamp01((0.32F + SmoothStep(0.04F, 0.75F, slope) * 0.58F) * (0.78F + lowNoise * 0.22F));
            result.confidence = 0.50F + result.value * 0.50F;
            break;
        }
        case WaterRippleOverlayType::CurrentThreads:
            result.value = RippleCurrentThreadValue(uv, layer, seed);
            result.confidence = 0.55F + result.value * 0.45F;
            break;
        case WaterRippleOverlayType::DropletGlints:
            result.value = RippleDropletValue(uv, layer, seed);
            result.confidence = 0.45F + result.value * 0.55F;
            break;
        case WaterRippleOverlayType::DripTrails: {
            glm::vec3 dripTangent = kGravity - normal * glm::dot(kGravity, normal);
            if (glm::dot(dripTangent, dripTangent) <= kNormalEpsilon) {
                dripTangent = tangent;
            } else {
                dripTangent = glm::normalize(dripTangent);
            }
            glm::vec3 dripLateral = glm::cross(normal, dripTangent);
            if (glm::dot(dripLateral, dripLateral) <= kNormalEpsilon) {
                dripLateral = lateral;
            } else {
                dripLateral = glm::normalize(dripLateral);
            }
            const glm::vec2 dripUv{
                glm::dot(relative, dripTangent) * scale,
                glm::dot(relative, dripLateral) * scale,
            };
            const float spacing = std::max(wavelength * 0.55F, 0.010F);
            const auto lane = static_cast<int>(std::floor(dripUv.y / spacing));
            const float laneSeed = RippleCellHash(lane, 0, seed, 149U);
            const float center = (static_cast<float>(lane) + 0.15F + laneSeed * 0.70F) * spacing;
            const float line = RippleLine(dripUv.y - center, spacing * 0.10F);
            const float trail = 1.0F - SmoothStep(0.22F, 1.0F, Fract01((dripUv.x / (wavelength * 2.6F)) + laneSeed));
            result.value = Clamp01(line * trail * (0.25F + slope * 0.75F));
            result.tangent = dripTangent;
            result.linearCoord = dripUv.x;
            result.angle = std::atan2(dripTangent.y, dripTangent.x);
            result.confidence = 0.42F + result.value * 0.58F;
            break;
        }
        case WaterRippleOverlayType::FoamSparkle:
            result.value = RippleFoamSparkleValue(uv, layer, seed, edge);
            result.confidence = 0.50F + result.value * 0.50F;
            break;
        case WaterRippleOverlayType::SaltMineralShimmer:
            result.value = RippleSaltShimmerValue(uv, layer, seed);
            result.confidence = 0.48F + result.value * 0.52F;
            break;
    }

    if (result.linearCoord == 0.0F) {
        result.linearCoord = uv.x;
    }
    if (result.angle == 0.0F) {
        result.angle = std::atan2(tangent.y, tangent.x);
    }
    const float turbulenceNoise =
        RippleBlockNoise(uv, std::max(wavelength * 0.75F, 0.006F), seed, 163U) - 0.5F;
    const float turbulence = std::clamp(layer.turbulence, 0.0F, 4.0F);
    result.value = Clamp01(result.value + turbulenceNoise * turbulence * 0.08F);
    return result;
}

WaterStreamOverlay BuildStreamOverlayFromPaths(
    const std::vector<std::vector<WaterOverlayPoint>>& paths,
    std::uint32_t requestedStreamCount,
    float streamLengthMeters,
    float pointSpacingMeters,
    float streamWidthMeters,
    float streamWorldLengthMeters,
    float surfaceOffsetMeters,
    float laneSpreadMeters,
    float turbulence,
    float speedMetersPerSecond,
    std::uint32_t seed,
    float featureType) {
    WaterStreamOverlay overlay;
    if (paths.empty() || requestedStreamCount == 0U) {
        return overlay;
    }

    std::vector<float> pathLengths;
    pathLengths.reserve(paths.size());
    float totalLength = 0.0F;
    for (const auto& path : paths) {
        const float length = PathLengthMeters(path);
        pathLengths.push_back(length);
        totalLength += std::max(0.0F, length);
    }
    if (totalLength <= 1.0e-5F) {
        return overlay;
    }

    const float safeLength = std::clamp(streamLengthMeters, 0.02F, 100.0F);
    const float safeSpacing = std::clamp(pointSpacingMeters, 0.001F, 5.0F);
    const std::uint32_t samplesPerStream = std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(std::ceil(safeLength / safeSpacing)) + 1U);
    std::uint32_t streamId = 1U;

    for (std::size_t pathIndex = 0; pathIndex < paths.size(); ++pathIndex) {
        const auto& path = paths[pathIndex];
        const float pathLength = pathLengths[pathIndex];
        if (path.size() < 2U || pathLength <= 1.0e-5F) {
            continue;
        }
        const std::uint32_t streamsForPath = std::max<std::uint32_t>(
            1U,
            static_cast<std::uint32_t>(
                std::round(static_cast<float>(requestedStreamCount) * (pathLength / totalLength))));
        const auto branchId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(path.front().flowId + 0.5F)));
        const auto sourceId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(path.front().emitterId + 0.5F)));

        for (std::uint32_t laneIndex = 0; laneIndex < streamsForPath && streamId <= requestedStreamCount; ++laneIndex, ++streamId) {
            const float streamSeed = RegionHash01(seed + branchId, streamId, 7001U);
            const float laneSeed = RegionHash01(seed + branchId, streamId, 7003U);
            const float laneOffset = (laneSeed - 0.5F) * 2.0F * std::max(0.0F, laneSpreadMeters);
            const float maxStart = std::max(0.0F, pathLength - safeLength);
            const float startDistance = maxStart * RegionHash01(seed + branchId, streamId, 7009U);
            const float speed = std::max(0.01F, speedMetersPerSecond * (0.72F + streamSeed * 0.58F));
            for (std::uint32_t sampleIndex = 0; sampleIndex < samplesPerStream; ++sampleIndex) {
                const float localDistance = std::min(
                    safeLength,
                    static_cast<float>(sampleIndex) * safeSpacing);
                const float pathDistance = std::min(pathLength, startDistance + localDistance);
                WaterOverlayPoint anchor = InterpolatePathByArcLength(path, pathDistance);
                glm::vec3 position = ToGlm(anchor.position);
                const glm::vec3 normal = SafeOverlayNormal(ToGlm(anchor.normal));
                glm::vec3 tangent = TangentAtPathDistance(path, pathDistance, safeSpacing * 2.0F);
                tangent -= normal * glm::dot(tangent, normal);
                if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
                    tangent = TangentAtPathDistance(path, pathDistance, safeSpacing * 2.0F);
                }
                tangent = glm::normalize(tangent);
                glm::vec3 lateral = glm::cross(normal, tangent);
                if (glm::dot(lateral, lateral) <= kNormalEpsilon) {
                    lateral = WaterPathLateral(tangent);
                } else {
                    lateral = glm::normalize(lateral);
                }
                const float wobble =
                    std::sin((static_cast<float>(sampleIndex) * 0.43F + streamSeed * 6.28318530718F)) *
                    std::max(0.0F, turbulence) * std::max(0.0F, laneSpreadMeters);
                position += lateral * (laneOffset + wobble);
                position += normal * std::max(0.0F, surfaceOffsetMeters);

                const float pointAge = safeLength > 1.0e-5F ? localDistance / safeLength : 0.0F;
                WaterStreamSample sample;
                sample.position = FromGlm(position);
                sample.normal = FromGlm(normal);
                sample.tangent = FromGlm(tangent);
                sample.red = featureType >= 3.0F ? StreamColorByte(0.06F + streamSeed * 0.10F)
                                                  : StreamColorByte(0.04F + streamSeed * 0.12F);
                sample.green = featureType >= 3.0F ? StreamColorByte(0.52F + pointAge * 0.26F)
                                                    : StreamColorByte(0.68F + pointAge * 0.20F);
                sample.blue = StreamColorByte(0.95F + streamSeed * 0.05F);
                sample.streamId = static_cast<float>(streamId);
                sample.sourceId = static_cast<float>(sourceId);
                sample.pathId = static_cast<float>(pathIndex + 1U);
                sample.branchId = static_cast<float>(branchId);
                sample.streamSeed = streamSeed;
                sample.pointSeed = RegionHash01(seed + branchId, streamId + sampleIndex, 7013U);
                sample.streamDistance = localDistance;
                sample.streamLength = safeLength;
                sample.pointAge = pointAge;
                sample.streamAge = RegionHash01(seed + branchId, streamId, 7019U);
                sample.streamSpeed = speed;
                sample.streamWidth = std::max(0.0005F, streamWidthMeters * (0.80F + streamSeed * 0.42F));
                sample.streamWorldLength = std::max(
                    sample.streamWidth * 2.0F,
                    std::max(streamWorldLengthMeters, safeSpacing * 2.5F));
                sample.streamConfidence = std::clamp(anchor.confidence * (0.72F + 0.28F * streamSeed), 0.0F, 1.0F);
                sample.wetness = std::clamp(0.35F + anchor.accumulation * 0.65F, 0.0F, 1.0F);
                sample.featureType = featureType;
                IncludeStreamSample(&overlay, sample);
            }
        }
    }

    return overlay;
}

}  // namespace

WaterStreamOverlay BuildFlowStreamOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowStreamSettings& settings) {
    if (!settings.enabled) {
        return {};
    }
    return BuildStreamOverlayFromPaths(
        GroupAnchorPaths(pathAnchors),
        settings.streamCountTotal,
        settings.streamLengthMeters,
        settings.streamPointSpacingMeters,
        settings.streamWidthMeters,
        settings.streamWorldLengthMeters,
        settings.surfaceOffsetMeters,
        settings.laneSpreadMeters,
        settings.turbulence,
        settings.speedMetersPerSecond,
        settings.seed,
        0.0F);
}

WaterFieldCache BuildFieldCacheFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFieldSettings& settings) {
    WaterFieldCache cache;
    cache.settings = settings;
    if (!settings.enabled) {
        return cache;
    }
    const auto paths = GroupAnchorPaths(pathAnchors);
    const float spacing = std::clamp(settings.fieldResolutionMeters, 0.003F, 1.0F);
    for (const auto& path : paths) {
        const float length = PathLengthMeters(path);
        if (length <= 1.0e-5F) {
            continue;
        }
        const std::uint32_t sampleCount = std::max<std::uint32_t>(
            2U,
            static_cast<std::uint32_t>(std::ceil(length / spacing)) + 1U);
        for (std::uint32_t index = 0; index < sampleCount; ++index) {
            const float station = std::min(length, static_cast<float>(index) * spacing);
            WaterOverlayPoint anchor = InterpolatePathByArcLength(path, station);
            const glm::vec3 normal = SafeOverlayNormal(ToGlm(anchor.normal));
            glm::vec3 guide = TangentAtPathDistance(path, station, spacing * 2.0F);
            guide -= normal * glm::dot(guide, normal);
            if (glm::dot(guide, guide) <= kNormalEpsilon) {
                guide = TangentAtPathDistance(path, station, spacing * 2.0F);
            }
            guide = glm::normalize(guide);
            glm::vec3 downhill = kGravity - normal * glm::dot(kGravity, normal);
            if (glm::dot(downhill, downhill) > kNormalEpsilon) {
                downhill = glm::normalize(downhill);
            } else {
                downhill = guide;
            }
            glm::vec3 fieldVector =
                guide * std::max(0.0F, settings.guideWeight) +
                downhill * std::max(0.0F, settings.downhillWeight);
            if (glm::dot(fieldVector, fieldVector) <= kNormalEpsilon) {
                fieldVector = guide;
            }
            fieldVector = glm::normalize(fieldVector - normal * glm::dot(fieldVector, normal));

            WaterFieldNode node;
            node.position = anchor.position;
            node.normal = FromGlm(normal);
            node.vector = FromGlm(fieldVector);
            node.wetness = std::clamp(0.30F + anchor.accumulation * 0.70F, 0.0F, 1.0F);
            node.confidence = std::clamp(anchor.confidence, 0.0F, 1.0F);
            node.surfaceConfidence = std::clamp(anchor.confidence, 0.0F, 1.0F);
            node.pathStation = station;
            node.distanceToGuide = 0.0F;
            cache.nodes.push_back(node);
        }
    }
    return cache;
}

WaterFieldCache BuildFieldCacheFromRegions(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers,
    const WaterFieldSettings& settings) {
    WaterFieldCache cache;
    cache.settings = settings;
    if (!settings.enabled || cloud.positions.empty()) {
        return cache;
    }

    const bool singleRegion = std::count_if(
                                  layers.begin(),
                                  layers.end(),
                                  [](const WaterEffectLayer& layer) {
                                      return layer.featureType == WaterEffectFeatureType::FieldSurfaceMotion &&
                                             (layer.enabledInViewport || layer.enabledInExport) &&
                                             layer.vertices.size() >= 3U;
                                  }) == 1;
    struct FieldControlBoundary {
        WaterEffectFeatureType type = WaterEffectFeatureType::FieldNoFlowRegion;
        std::vector<invisible_places::io::Float3> boundary;
    };
    std::vector<FieldControlBoundary> controlBoundaries;
    for (const auto& layer : layers) {
        if ((!layer.enabledInViewport && !layer.enabledInExport) || layer.vertices.size() < 3U) {
            continue;
        }
        if (layer.featureType != WaterEffectFeatureType::FieldNoFlowRegion &&
            layer.featureType != WaterEffectFeatureType::FieldBridgeAllowedRegion &&
            layer.featureType != WaterEffectFeatureType::FieldBridgeBlockedRegion) {
            continue;
        }
        auto boundary = BuildWaterRegionBoundary(layer.vertices);
        if (boundary.size() >= 3U) {
            FieldControlBoundary control;
            control.type = layer.featureType;
            control.boundary = std::move(boundary);
            controlBoundaries.push_back(std::move(control));
        }
    }

    for (auto layer : layers) {
        if (layer.featureType != WaterEffectFeatureType::FieldSurfaceMotion ||
            (!layer.enabledInViewport && !layer.enabledInExport)) {
            continue;
        }
        const auto boundary = BuildWaterRegionBoundary(layer.vertices);
        if (boundary.size() < 3U) {
            continue;
        }
        if (singleRegion && cache.regionBoundary.empty()) {
            cache.regionBoundary = boundary;
        }

        const glm::vec3 regionCenter = RippleRegionCentroid(boundary);
        glm::vec3 requestedDirection{layer.directionX, layer.directionY, layer.directionZ};
        if (glm::dot(requestedDirection, requestedDirection) <= kNormalEpsilon) {
            requestedDirection = {1.0F, 0.0F, 0.0F};
        }
        requestedDirection = glm::normalize(requestedDirection);

        const auto maxAffected = std::max<std::uint32_t>(1U, layer.maxAffectedPoints);
        const auto stride = SampleStride(cloud.positions.size(), maxAffected);
        for (std::size_t pointIndex = 0; pointIndex < cloud.positions.size(); pointIndex += stride) {
            const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
            if (!IsValidPoint(position) || !PointInPolygonXy(position, boundary)) {
                continue;
            }

            float edgeDistance = EffectPolygonEdgeDistance3d(position, boundary);
            if (!std::isfinite(edgeDistance)) {
                edgeDistance = EffectPolygonEdgeDistanceXy(position, boundary);
            }
            const float edge = SmoothStep(0.0F, std::max(0.001F, layer.edgeBlendWidth), edgeDistance);
            glm::vec3 normal{0.0F, 0.0F, 1.0F};
            if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
                normal = SafeOverlayNormal(ToGlm(cloud.normals[pointIndex]));
            }

            glm::vec3 guide = requestedDirection - normal * glm::dot(requestedDirection, normal);
            if (glm::dot(guide, guide) <= kNormalEpsilon) {
                guide = WaterPathLateral(normal);
            } else {
                guide = glm::normalize(guide);
            }

            glm::vec3 downhill = kGravity - normal * glm::dot(kGravity, normal);
            if (glm::dot(downhill, downhill) > kNormalEpsilon) {
                downhill = glm::normalize(downhill);
            } else {
                downhill = guide;
            }

            glm::vec3 fieldVector =
                guide * std::max(0.0F, settings.guideWeight) +
                downhill * std::max(0.0F, settings.downhillWeight);
            fieldVector -= normal * glm::dot(fieldVector, normal);
            if (glm::dot(fieldVector, fieldVector) <= kNormalEpsilon) {
                fieldVector = guide;
            } else {
                fieldVector = glm::normalize(fieldVector);
            }

            WaterFieldNode node;
            node.position = cloud.positions[pointIndex];
            node.normal = FromGlm(normal);
            node.vector = FromGlm(fieldVector);
            node.sourcePointIndex = static_cast<std::uint32_t>(std::min<std::size_t>(
                pointIndex,
                static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
            node.sourceLayerId = layer.id;
            node.blendMode = layer.blendMode;
            node.response = layer.response;
            node.effectSpeed = layer.speed;
            for (const auto& control : controlBoundaries) {
                if (!PointInPolygonXy(position, control.boundary)) {
                    continue;
                }
                switch (control.type) {
                    case WaterEffectFeatureType::FieldNoFlowRegion:
                        node.flowBlocked = true;
                        break;
                    case WaterEffectFeatureType::FieldBridgeAllowedRegion:
                        node.bridgeAllowed = true;
                        break;
                    case WaterEffectFeatureType::FieldBridgeBlockedRegion:
                        node.bridgeBlocked = true;
                        break;
                    case WaterEffectFeatureType::FieldSurfaceMotion:
                    case WaterEffectFeatureType::Ripple:
                        break;
                }
            }
            node.wetness = std::clamp(layer.regionStrength * edge, 0.0F, 1.0F);
            node.confidence = std::clamp(edge * layer.response.intensity, 0.0F, 1.0F);
            node.surfaceConfidence = node.confidence;
            if (node.flowBlocked) {
                node.wetness = 0.0F;
                node.confidence = 0.0F;
                node.surfaceConfidence = 0.0F;
            }
            node.pathStation = glm::dot(position - regionCenter, guide);
            node.distanceToGuide = edgeDistance;
            cache.nodes.push_back(node);
        }
    }

    std::sort(
        cache.nodes.begin(),
        cache.nodes.end(),
        [](const WaterFieldNode& left, const WaterFieldNode& right) {
            if (std::abs(left.pathStation - right.pathStation) > 1.0e-5F) {
                return left.pathStation < right.pathStation;
            }
            if (std::abs(left.position.y - right.position.y) > 1.0e-5F) {
                return left.position.y < right.position.y;
            }
            return left.position.x < right.position.x;
        });
    if (!cache.nodes.empty()) {
        const float firstStation = cache.nodes.front().pathStation;
        for (auto& node : cache.nodes) {
            node.pathStation -= firstStation;
        }
    }
    return cache;
}

WaterStreamOverlay BuildFieldStreamOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldStreamSettings& settings) {
    if (!settings.enabled || fieldCache.nodes.size() < 2U) {
        return {};
    }
    std::vector<std::vector<WaterOverlayPoint>> paths;
    std::vector<WaterOverlayPoint> path;
    path.reserve(fieldCache.nodes.size());
    float distance = 0.0F;
    WaterFieldStreamDiagnostics diagnostics;
    diagnostics.inputNodeCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        fieldCache.nodes.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    const float bridgeStart = std::max(0.0F, fieldCache.settings.fieldResolutionMeters * 1.25F);
    const float maxBridgeDistance = std::max(
        fieldCache.settings.maxBridgeDistanceMeters,
        fieldCache.settings.fieldResolutionMeters * (1.25F + fieldCache.settings.bridgeAggression * 4.0F));
    const float confidenceThreshold = std::clamp(fieldCache.settings.surfaceConfidenceThreshold, 0.0F, 1.0F);
    bool lastAcceptedBridgeAllowed = false;
    bool lastAcceptedBridgeBlocked = false;
    auto flushPath = [&]() {
        if (path.size() >= 2U) {
            paths.push_back(std::move(path));
        }
        path = {};
        distance = 0.0F;
        lastAcceptedBridgeAllowed = false;
        lastAcceptedBridgeBlocked = false;
    };
    for (std::size_t index = 0; index < fieldCache.nodes.size(); ++index) {
        const auto& node = fieldCache.nodes[index];
        if (node.flowBlocked) {
            ++diagnostics.manualNoFlowBlockCount;
            flushPath();
            continue;
        }
        if (settings.fadeOnLowConfidence && node.surfaceConfidence < confidenceThreshold * 0.35F) {
            ++diagnostics.lowConfidenceTerminationCount;
            flushPath();
            continue;
        }
        if (!path.empty()) {
            const float gapDistance = glm::length(ToGlm(node.position) - ToGlm(path.back().position));
            const bool manualBridgeAllowed = lastAcceptedBridgeAllowed || node.bridgeAllowed;
            const bool manualBridgeBlocked = lastAcceptedBridgeBlocked || node.bridgeBlocked;
            const float manualAllowedBridgeDistance = maxBridgeDistance * 4.0F;
            if (manualBridgeBlocked) {
                ++diagnostics.manualBridgeBlockedCount;
                ++diagnostics.rejectedGapCount;
                diagnostics.minRejectedGapMeters =
                    diagnostics.minRejectedGapMeters <= 0.0F
                        ? gapDistance
                        : std::min(diagnostics.minRejectedGapMeters, gapDistance);
                flushPath();
            } else if (gapDistance > maxBridgeDistance &&
                       (!manualBridgeAllowed || gapDistance > manualAllowedBridgeDistance)) {
                ++diagnostics.rejectedGapCount;
                diagnostics.minRejectedGapMeters =
                    diagnostics.minRejectedGapMeters <= 0.0F
                        ? gapDistance
                        : std::min(diagnostics.minRejectedGapMeters, gapDistance);
                flushPath();
            } else {
                if (gapDistance > bridgeStart) {
                    ++diagnostics.acceptedBridgeCount;
                    diagnostics.maxAcceptedBridgeMeters =
                        std::max(diagnostics.maxAcceptedBridgeMeters, gapDistance);
                    if (manualBridgeAllowed && gapDistance > maxBridgeDistance) {
                        ++diagnostics.manualBridgeAllowedCount;
                    }
                }
                distance += gapDistance;
            }
        }

        WaterOverlayPoint point;
        point.position = node.position;
        point.normal = node.normal;
        point.flowId = static_cast<float>(paths.size() + 1U);
        point.emitterId = 0.0F;
        point.pathDistance = distance;
        point.confidence = node.confidence;
        point.accumulation = node.wetness;
        if (settings.fadeOnLowConfidence) {
            const float supportFade = SmoothStep(
                confidenceThreshold * 0.35F,
                std::max(confidenceThreshold, confidenceThreshold * 0.35F + 1.0e-4F),
                node.surfaceConfidence);
            if (supportFade < 0.999F) {
                ++diagnostics.lowConfidenceFadeCount;
            }
            point.confidence *= supportFade;
            point.accumulation *= supportFade;
        }
        path.push_back(point);
        lastAcceptedBridgeAllowed = node.bridgeAllowed;
        lastAcceptedBridgeBlocked = node.bridgeBlocked;
    }
    flushPath();
    diagnostics.emittedPathCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        paths.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    auto overlay = BuildStreamOverlayFromPaths(
        paths,
        settings.streamlineCount,
        settings.streamlineLengthMeters,
        settings.stepLengthMeters,
        settings.streamlineWidthMeters,
        settings.streamWorldLengthMeters,
        fieldCache.settings.surfaceOffsetMeters,
        std::max(settings.seedSpacingMeters, fieldCache.settings.corridorRadiusMeters * 0.30F),
        fieldCache.settings.turbulence,
        settings.speedMetersPerSecond,
        fieldCache.settings.seed,
        3.0F);
    if (fieldCache.regionBoundary.size() < 3U) {
        diagnostics.emittedSampleCount = static_cast<std::uint32_t>(std::min<std::size_t>(
            overlay.samples.size(),
            static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
        overlay.fieldDiagnostics = diagnostics;
        return overlay;
    }

    WaterStreamOverlay clipped;
    for (const auto& sample : overlay.samples) {
        if (PointInPolygonXy(ToGlm(sample.position), fieldCache.regionBoundary)) {
            IncludeStreamSample(&clipped, sample);
        }
    }
    diagnostics.emittedSampleCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        clipped.samples.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    clipped.fieldDiagnostics = diagnostics;
    return clipped;
}

WaterEffectOverlay GenerateRippleEffectOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers) {
    WaterEffectOverlay overlay;
    if (cloud.positions.empty()) {
        return overlay;
    }
    for (auto layer : layers) {
        if (!layer.enabledInViewport && !layer.enabledInExport) {
            continue;
        }
        const auto boundary = BuildWaterRegionBoundary(layer.vertices);
        if (boundary.size() < 3U) {
            continue;
        }
        layer.hull = BuildWaterRegionHull(layer.vertices);
        const glm::vec3 regionCenter = RippleRegionCentroid(boundary);
        const std::uint32_t maxAffected = std::max<std::uint32_t>(1U, layer.maxAffectedPoints);
        const std::size_t stride = SampleStride(cloud.positions.size(), maxAffected);
        const glm::vec3 direction = [&]() {
            glm::vec3 value{layer.directionX, layer.directionY, layer.directionZ};
            if (glm::dot(value, value) <= kNormalEpsilon) {
                value = {1.0F, 0.0F, 0.0F};
            }
            return glm::normalize(value);
        }();
        for (std::size_t pointIndex = 0; pointIndex < cloud.positions.size(); pointIndex += stride) {
            const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
            if (!IsValidPoint(position) || !PointInPolygonXy(position, boundary)) {
                continue;
            }
            float edgeDistance = EffectPolygonEdgeDistance3d(position, boundary);
            if (!std::isfinite(edgeDistance)) {
                edgeDistance = EffectPolygonEdgeDistanceXy(position, boundary);
            }
            const float edge = SmoothStep(0.0F, std::max(0.001F, layer.edgeBlendWidth), edgeDistance);
            glm::vec3 normal{0.0F, 0.0F, 1.0F};
            if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
                normal = SafeOverlayNormal(ToGlm(cloud.normals[pointIndex]));
            }
            glm::vec3 tangent = direction - normal * glm::dot(direction, normal);
            if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
                tangent = WaterPathLateral(normal);
            } else {
                tangent = glm::normalize(tangent);
            }
            const float seedValue = RegionHash01(layer.id + layer.seed, static_cast<std::uint32_t>(pointIndex), 901U);
            const auto pattern = EvaluateRipplePattern(
                layer,
                position,
                normal,
                tangent,
                regionCenter,
                edge,
                edgeDistance,
                static_cast<std::uint32_t>(pointIndex));
            WaterEffectPoint effect;
            effect.position = cloud.positions[pointIndex];
            effect.normal = FromGlm(normal);
            effect.tangent = FromGlm(pattern.tangent);
            effect.sourcePointIndex = static_cast<std::uint32_t>(std::min<std::size_t>(
                pointIndex,
                static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
            effect.blendMode = layer.blendMode;
            effect.red = FloatToByte(layer.response.colouriseRed);
            effect.green = FloatToByte(layer.response.colouriseGreen);
            effect.blue = FloatToByte(layer.response.colouriseBlue);
            effect.mask = std::clamp(layer.regionStrength, 0.0F, 1.0F);
            effect.edge = edge;
            effect.value = Clamp01(pattern.value * std::max(0.0F, layer.response.intensity)) * effect.mask * edge;
            effect.seed = seedValue;
            effect.regionId = static_cast<float>(layer.id);
            effect.distance = pattern.distance;
            effect.linearCoord = pattern.linearCoord;
            effect.angle = pattern.angle;
            effect.speed = std::max(0.0F, layer.speed);
            effect.confidence = Clamp01(pattern.confidence * edge);
            effect.emissionHint = effect.value * layer.response.emissionAdd;
            effect.opacityHint = effect.value * layer.response.opacityAdd;
            effect.opacityMultiplyHint = std::lerp(
                1.0F,
                std::max(0.0F, layer.response.opacityMultiply),
                effect.value);
            effect.sizeHint = effect.value * layer.response.pointSizeAdd;
            effect.sizeMultiplyHint = std::lerp(
                1.0F,
                std::max(0.0F, layer.response.pointSizeMultiply),
                effect.value);
            effect.colourMixHint = effect.value * layer.response.colouriseAmount;
            effect.featureType = 1.0F;
            IncludeEffectPoint(&overlay, effect);
        }
    }
    return overlay;
}

WaterEffectOverlay GenerateFieldSurfaceEffectOverlay(
    const WaterFieldCache& fieldCache,
    const WaterEffectLayer& layer) {
    WaterEffectOverlay overlay;
    if (fieldCache.nodes.empty()) {
        return overlay;
    }
    const std::uint32_t maxAffected = std::max<std::uint32_t>(1U, layer.maxAffectedPoints);
    const std::size_t stride = SampleStride(fieldCache.nodes.size(), maxAffected);
    for (std::size_t index = 0; index < fieldCache.nodes.size(); index += stride) {
        const auto& node = fieldCache.nodes[index];
        if (node.flowBlocked) {
            continue;
        }
        const glm::vec3 position = ToGlm(node.position);
        const glm::vec3 normal = SafeOverlayNormal(ToGlm(node.normal));
        glm::vec3 tangent = ToGlm(node.vector);
        tangent -= normal * glm::dot(tangent, normal);
        if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
            tangent = {1.0F, 0.0F, 0.0F};
        } else {
            tangent = glm::normalize(tangent);
        }
        const bool hasNodeEffectLayer = node.sourceLayerId != 0U;
        const auto& response = hasNodeEffectLayer ? node.response : layer.response;
        const auto blendMode = hasNodeEffectLayer ? node.blendMode : layer.blendMode;
        const auto regionId = hasNodeEffectLayer ? node.sourceLayerId : layer.id;
        const float effectSpeed = hasNodeEffectLayer ? node.effectSpeed : layer.speed;
        const float seedValue = RegionHash01(regionId + layer.seed, static_cast<std::uint32_t>(index), 1301U);
        WaterEffectPoint effect;
        effect.position = FromGlm(position + normal * std::max(0.0F, fieldCache.settings.surfaceOffsetMeters));
        effect.normal = FromGlm(normal);
        effect.tangent = FromGlm(tangent);
        effect.sourcePointIndex = node.sourcePointIndex;
        effect.blendMode = blendMode;
        effect.red = FloatToByte(response.colouriseRed);
        effect.green = FloatToByte(response.colouriseGreen);
        effect.blue = FloatToByte(response.colouriseBlue);
        effect.mask = std::clamp(layer.regionStrength, 0.0F, 1.0F);
        effect.edge = 1.0F;
        effect.value = node.wetness * node.confidence * effect.mask;
        effect.seed = seedValue;
        effect.regionId = static_cast<float>(regionId);
        effect.distance = node.distanceToGuide;
        effect.linearCoord = node.pathStation;
        effect.angle = std::atan2(tangent.y, tangent.x);
        effect.speed = std::max(0.0F, effectSpeed);
        effect.confidence = node.confidence;
        effect.emissionHint = effect.value * response.emissionAdd;
        effect.opacityHint = effect.value * response.opacityAdd;
        effect.opacityMultiplyHint = std::lerp(
            1.0F,
            std::max(0.0F, response.opacityMultiply),
            effect.value);
        effect.sizeHint = effect.value * response.pointSizeAdd;
        effect.sizeMultiplyHint = std::lerp(
            1.0F,
            std::max(0.0F, response.pointSizeMultiply),
            effect.value);
        effect.colourMixHint = effect.value * response.colouriseAmount;
        effect.fieldFlowU = node.pathStation;
        effect.fieldWetness = node.wetness;
        effect.fieldSurfaceConfidence = node.surfaceConfidence;
        effect.featureType = 2.0F;
        IncludeEffectPoint(&overlay, effect);
    }
    return overlay;
}

WaterEffectCompositionFields ComposeWaterEffectFields(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectOverlay>& overlays) {
    WaterEffectCompositionFields fields;
    const auto pointCount = cloud.PointCount();
    fields.value.assign(pointCount, 0.0F);
    fields.emissionAdd.assign(pointCount, 0.0F);
    fields.opacityAdd.assign(pointCount, 0.0F);
    fields.opacityMultiply.assign(pointCount, 1.0F);
    fields.pointSizeAdd.assign(pointCount, 0.0F);
    fields.pointSizeMultiply.assign(pointCount, 1.0F);
    fields.colourRed.assign(pointCount, 0.62F);
    fields.colourGreen.assign(pointCount, 0.88F);
    fields.colourBlue.assign(pointCount, 1.0F);
    fields.colourMix.assign(pointCount, 0.0F);

    if (pointCount == 0U || overlays.empty()) {
        return fields;
    }

    std::vector<float> colourWeight(pointCount, 0.0F);
    std::vector<float> colourRedSum(pointCount, 0.0F);
    std::vector<float> colourGreenSum(pointCount, 0.0F);
    std::vector<float> colourBlueSum(pointCount, 0.0F);
    std::vector<std::uint8_t> touched(pointCount, 0U);
    std::unordered_map<GridKey, std::vector<std::uint32_t>, GridKeyHash> pointGrid;
    bool pointGridBuilt = false;

    constexpr float kEffectMatchCellSize = 0.025F;
    constexpr float kEffectMatchMaxDistance = 0.060F;
    constexpr float kEffectMatchMaxDistanceSquared = kEffectMatchMaxDistance * kEffectMatchMaxDistance;
    auto buildPointGrid = [&]() {
        if (pointGridBuilt) {
            return;
        }
        pointGrid.reserve(pointCount);
        for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
            const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
            if (!IsValidPoint(position)) {
                continue;
            }
            pointGrid[MakeGridKey(position, kEffectMatchCellSize)].push_back(static_cast<std::uint32_t>(
                std::min<std::size_t>(
                    pointIndex,
                    static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))));
        }
        pointGridBuilt = true;
    };

    auto resolvePointIndex = [&](const WaterEffectPoint& effect) -> std::optional<std::size_t> {
        if (effect.sourcePointIndex < pointCount) {
            return static_cast<std::size_t>(effect.sourcePointIndex);
        }
        const glm::vec3 position = ToGlm(effect.position);
        if (!IsValidPoint(position)) {
            return std::nullopt;
        }
        buildPointGrid();
        const auto baseKey = MakeGridKey(position, kEffectMatchCellSize);
        std::optional<std::size_t> bestIndex;
        float bestDistanceSquared = kEffectMatchMaxDistanceSquared;
        for (int dz = -2; dz <= 2; ++dz) {
            for (int dy = -2; dy <= 2; ++dy) {
                for (int dx = -2; dx <= 2; ++dx) {
                    const GridKey key{baseKey.x + dx, baseKey.y + dy, baseKey.z + dz};
                    const auto bucketIt = pointGrid.find(key);
                    if (bucketIt == pointGrid.end()) {
                        continue;
                    }
                    for (const auto candidateIndex : bucketIt->second) {
                        if (candidateIndex >= pointCount) {
                            continue;
                        }
                        const glm::vec3 delta = ToGlm(cloud.positions[candidateIndex]) - position;
                        const float distanceSquared = glm::dot(delta, delta);
                        if (distanceSquared < bestDistanceSquared) {
                            bestDistanceSquared = distanceSquared;
                            bestIndex = static_cast<std::size_t>(candidateIndex);
                        }
                    }
                }
            }
        }
        return bestIndex;
    };

    auto screen = [](float baseValue, float contribution) {
        const float a = std::clamp(baseValue, 0.0F, 1.0F);
        const float b = std::clamp(contribution, 0.0F, 1.0F);
        return 1.0F - ((1.0F - a) * (1.0F - b));
    };

    auto addTint = [&](std::size_t pointIndex, const WaterEffectPoint& effect, float amount) {
        const float weight = std::clamp(amount, 0.0F, 1.0F);
        if (weight <= 1.0e-6F) {
            return;
        }
        colourRedSum[pointIndex] += (static_cast<float>(effect.red) / 255.0F) * weight;
        colourGreenSum[pointIndex] += (static_cast<float>(effect.green) / 255.0F) * weight;
        colourBlueSum[pointIndex] += (static_cast<float>(effect.blue) / 255.0F) * weight;
        colourWeight[pointIndex] += weight;
    };

    auto markTouched = [&](std::size_t pointIndex) {
        if (touched[pointIndex] == 0U) {
            touched[pointIndex] = 1U;
            ++fields.affectedPointCount;
        }
    };

    for (const auto& overlay : overlays) {
        for (const auto& effect : overlay.points) {
            const auto pointIndex = resolvePointIndex(effect);
            if (!pointIndex.has_value()) {
                continue;
            }
            const auto index = pointIndex.value();
            const float value = std::clamp(effect.value, 0.0F, 1.0F);
            if (value <= 1.0e-6F) {
                continue;
            }
            markTouched(index);
            const float emissionAdd = std::max(0.0F, effect.emissionHint);
            const float opacityAdd = effect.opacityHint;
            const float opacityMul = std::max(0.0F, effect.opacityMultiplyHint);
            const float sizeAdd = effect.sizeHint;
            const float sizeMul = std::max(0.0F, effect.sizeMultiplyHint);
            const float colourMix = std::clamp(effect.colourMixHint, 0.0F, 1.0F);

            switch (effect.blendMode) {
                case WaterEffectBlendMode::Max:
                    fields.value[index] = std::max(fields.value[index], value);
                    fields.emissionAdd[index] = std::max(fields.emissionAdd[index], emissionAdd);
                    fields.opacityAdd[index] = std::max(fields.opacityAdd[index], opacityAdd);
                    fields.opacityMultiply[index] = std::max(fields.opacityMultiply[index], opacityMul);
                    fields.pointSizeAdd[index] = std::max(fields.pointSizeAdd[index], sizeAdd);
                    fields.pointSizeMultiply[index] = std::max(fields.pointSizeMultiply[index], sizeMul);
                    if (colourMix >= fields.colourMix[index]) {
                        fields.colourMix[index] = colourMix;
                        fields.colourRed[index] = static_cast<float>(effect.red) / 255.0F;
                        fields.colourGreen[index] = static_cast<float>(effect.green) / 255.0F;
                        fields.colourBlue[index] = static_cast<float>(effect.blue) / 255.0F;
                    }
                    break;
                case WaterEffectBlendMode::Multiply:
                    fields.value[index] = std::max(fields.value[index], value);
                    fields.opacityMultiply[index] *= opacityMul;
                    fields.pointSizeMultiply[index] *= sizeMul;
                    fields.emissionAdd[index] += emissionAdd;
                    fields.opacityAdd[index] += opacityAdd;
                    fields.pointSizeAdd[index] += sizeAdd;
                    fields.colourMix[index] = std::clamp(fields.colourMix[index] + colourMix, 0.0F, 1.0F);
                    addTint(index, effect, colourMix);
                    break;
                case WaterEffectBlendMode::Screen:
                    fields.value[index] = screen(fields.value[index], value);
                    fields.emissionAdd[index] = screen(fields.emissionAdd[index], emissionAdd);
                    fields.opacityAdd[index] = screen(fields.opacityAdd[index], opacityAdd);
                    fields.opacityMultiply[index] *= opacityMul;
                    fields.pointSizeAdd[index] = screen(fields.pointSizeAdd[index], sizeAdd);
                    fields.pointSizeMultiply[index] *= sizeMul;
                    fields.colourMix[index] = screen(fields.colourMix[index], colourMix);
                    addTint(index, effect, colourMix);
                    break;
                case WaterEffectBlendMode::Override:
                    fields.value[index] = value;
                    fields.emissionAdd[index] = emissionAdd;
                    fields.opacityAdd[index] = opacityAdd;
                    fields.opacityMultiply[index] = opacityMul;
                    fields.pointSizeAdd[index] = sizeAdd;
                    fields.pointSizeMultiply[index] = sizeMul;
                    fields.colourMix[index] = colourMix;
                    fields.colourRed[index] = static_cast<float>(effect.red) / 255.0F;
                    fields.colourGreen[index] = static_cast<float>(effect.green) / 255.0F;
                    fields.colourBlue[index] = static_cast<float>(effect.blue) / 255.0F;
                    colourWeight[index] = 0.0F;
                    break;
                case WaterEffectBlendMode::Add:
                    fields.value[index] = std::clamp(fields.value[index] + value, 0.0F, 1.0F);
                    fields.emissionAdd[index] += emissionAdd;
                    fields.opacityAdd[index] += opacityAdd;
                    fields.opacityMultiply[index] *= opacityMul;
                    fields.pointSizeAdd[index] += sizeAdd;
                    fields.pointSizeMultiply[index] *= sizeMul;
                    fields.colourMix[index] = std::clamp(fields.colourMix[index] + colourMix, 0.0F, 1.0F);
                    addTint(index, effect, colourMix);
                    break;
            }
        }
    }

    for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
        fields.opacityMultiply[pointIndex] = std::max(0.0F, fields.opacityMultiply[pointIndex]);
        fields.pointSizeMultiply[pointIndex] = std::max(0.0F, fields.pointSizeMultiply[pointIndex]);
        if (colourWeight[pointIndex] > 1.0e-6F) {
            fields.colourRed[pointIndex] = colourRedSum[pointIndex] / colourWeight[pointIndex];
            fields.colourGreen[pointIndex] = colourGreenSum[pointIndex] / colourWeight[pointIndex];
            fields.colourBlue[pointIndex] = colourBlueSum[pointIndex] / colourWeight[pointIndex];
        }
    }

    return fields;
}

invisible_places::io::LoadedPointCloud BuildWaterStreamOverlayPointCloud(
    const WaterStreamOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = sourcePath;
    cloud.layerName = std::string{layerName};
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    cloud.positions.reserve(overlay.samples.size());
    cloud.normals.reserve(overlay.samples.size());
    cloud.packedColors.reserve(overlay.samples.size());
    cloud.bounds = overlay.bounds;
    for (const auto& sample : overlay.samples) {
        cloud.positions.push_back(sample.position);
        cloud.normals.push_back(sample.normal);
        cloud.packedColors.push_back(PackRgba8(sample.red, sample.green, sample.blue));
        if (!cloud.bounds.valid) {
            cloud.bounds.Expand(sample.position);
        }
    }
    if (!cloud.positions.empty()) {
        cloud.focusPoint = cloud.bounds.valid
                               ? invisible_places::io::Float3{
                                     (cloud.bounds.minimum.x + cloud.bounds.maximum.x) * 0.5F,
                                     (cloud.bounds.minimum.y + cloud.bounds.maximum.y) * 0.5F,
                                     (cloud.bounds.minimum.z + cloud.bounds.maximum.z) * 0.5F}
                               : cloud.positions.front();
        cloud.hasFocusPoint = true;
    }

    struct StreamScalarField {
        std::string_view name;
        float (*value)(const WaterStreamSample&);
    };
    const StreamScalarField fields[] = {
        {"stream_id", [](const WaterStreamSample& sample) { return sample.streamId; }},
        {"source_id", [](const WaterStreamSample& sample) { return sample.sourceId; }},
        {"path_id", [](const WaterStreamSample& sample) { return sample.pathId; }},
        {"branch_id", [](const WaterStreamSample& sample) { return sample.branchId; }},
        {"stream_seed", [](const WaterStreamSample& sample) { return sample.streamSeed; }},
        {"point_seed", [](const WaterStreamSample& sample) { return sample.pointSeed; }},
        {"stream_distance", [](const WaterStreamSample& sample) { return sample.streamDistance; }},
        {"stream_length", [](const WaterStreamSample& sample) { return sample.streamLength; }},
        {"point_age", [](const WaterStreamSample& sample) { return sample.pointAge; }},
        {"stream_age", [](const WaterStreamSample& sample) { return sample.streamAge; }},
        {"stream_speed", [](const WaterStreamSample& sample) { return sample.streamSpeed; }},
        {"stream_width", [](const WaterStreamSample& sample) { return sample.streamWidth; }},
        {"stream_world_length", [](const WaterStreamSample& sample) { return sample.streamWorldLength; }},
        {"stream_confidence", [](const WaterStreamSample& sample) { return sample.streamConfidence; }},
        {"wetness", [](const WaterStreamSample& sample) { return sample.wetness; }},
        {"feature_type", [](const WaterStreamSample& sample) { return sample.featureType; }},
        {"tangent_x", [](const WaterStreamSample& sample) { return sample.tangent.x; }},
        {"tangent_y", [](const WaterStreamSample& sample) { return sample.tangent.y; }},
        {"tangent_z", [](const WaterStreamSample& sample) { return sample.tangent.z; }},
    };

    cloud.scalarFields.reserve(std::size(fields));
    cloud.scalarFieldValues.reserve(overlay.samples.size() * std::size(fields));
    for (const auto& field : fields) {
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::string{field.name};
        for (const auto& sample : overlay.samples) {
            const float value = field.value(sample);
            cloud.scalarFieldValues.push_back(value);
            stats.Include(value);
        }
        cloud.scalarFields.push_back(stats);
    }
    return cloud;
}

invisible_places::io::LoadedPointCloud BuildWaterEffectOverlayPointCloud(
    const WaterEffectOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = sourcePath;
    cloud.layerName = std::string{layerName};
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    cloud.positions.reserve(overlay.points.size());
    cloud.normals.reserve(overlay.points.size());
    cloud.packedColors.reserve(overlay.points.size());
    cloud.bounds = overlay.bounds;
    for (const auto& point : overlay.points) {
        cloud.positions.push_back(point.position);
        cloud.normals.push_back(point.normal);
        cloud.packedColors.push_back(PackRgba8(point.red, point.green, point.blue));
        if (!cloud.bounds.valid) {
            cloud.bounds.Expand(point.position);
        }
    }
    if (!cloud.positions.empty()) {
        cloud.focusPoint = cloud.bounds.valid
                               ? invisible_places::io::Float3{
                                     (cloud.bounds.minimum.x + cloud.bounds.maximum.x) * 0.5F,
                                     (cloud.bounds.minimum.y + cloud.bounds.maximum.y) * 0.5F,
                                     (cloud.bounds.minimum.z + cloud.bounds.maximum.z) * 0.5F}
                               : cloud.positions.front();
        cloud.hasFocusPoint = true;
    }

    struct EffectScalarField {
        std::string_view name;
        float (*value)(const WaterEffectPoint&);
    };
    const EffectScalarField fields[] = {
        {"ripple_mask", [](const WaterEffectPoint& point) { return point.mask; }},
        {"ripple_edge", [](const WaterEffectPoint& point) { return point.edge; }},
        {"ripple_value", [](const WaterEffectPoint& point) { return point.value; }},
        {"ripple_seed", [](const WaterEffectPoint& point) { return point.seed; }},
        {"ripple_region_id", [](const WaterEffectPoint& point) { return point.regionId; }},
        {"ripple_distance", [](const WaterEffectPoint& point) { return point.distance; }},
        {"ripple_linear_coord", [](const WaterEffectPoint& point) { return point.linearCoord; }},
        {"ripple_angle", [](const WaterEffectPoint& point) { return point.angle; }},
        {"ripple_speed", [](const WaterEffectPoint& point) { return point.speed; }},
        {"ripple_confidence", [](const WaterEffectPoint& point) { return point.confidence; }},
        {"ripple_emission_hint", [](const WaterEffectPoint& point) { return point.emissionHint; }},
        {"ripple_opacity_hint", [](const WaterEffectPoint& point) { return point.opacityHint; }},
        {"ripple_size_hint", [](const WaterEffectPoint& point) { return point.sizeHint; }},
        {"ripple_colour_mix_hint", [](const WaterEffectPoint& point) { return point.colourMixHint; }},
        {"field_flow_u", [](const WaterEffectPoint& point) { return point.fieldFlowU; }},
        {"field_wetness", [](const WaterEffectPoint& point) { return point.fieldWetness; }},
        {"field_surface_confidence", [](const WaterEffectPoint& point) { return point.fieldSurfaceConfidence; }},
        {"feature_type", [](const WaterEffectPoint& point) { return point.featureType; }},
        {"tangent_x", [](const WaterEffectPoint& point) { return point.tangent.x; }},
        {"tangent_y", [](const WaterEffectPoint& point) { return point.tangent.y; }},
        {"tangent_z", [](const WaterEffectPoint& point) { return point.tangent.z; }},
        {"source_point_index", [](const WaterEffectPoint& point) {
             return point.sourcePointIndex == std::numeric_limits<std::uint32_t>::max()
                        ? -1.0F
                        : static_cast<float>(point.sourcePointIndex);
         }},
        {"effect_blend_mode", [](const WaterEffectPoint& point) {
             return static_cast<float>(point.blendMode);
         }},
        {"ripple_opacity_multiply_hint", [](const WaterEffectPoint& point) { return point.opacityMultiplyHint; }},
        {"ripple_size_multiply_hint", [](const WaterEffectPoint& point) { return point.sizeMultiplyHint; }},
    };
    cloud.scalarFields.reserve(std::size(fields));
    cloud.scalarFieldValues.reserve(overlay.points.size() * std::size(fields));
    for (const auto& field : fields) {
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::string{field.name};
        for (const auto& point : overlay.points) {
            const float value = field.value(point);
            cloud.scalarFieldValues.push_back(value);
            stats.Include(value);
        }
        cloud.scalarFields.push_back(stats);
    }
    return cloud;
}

std::vector<invisible_places::io::Float3> BuildWaterRegionHull(
    const std::vector<invisible_places::io::Float3>& vertices) {
    std::vector<invisible_places::io::Float3> points;
    points.reserve(vertices.size());
    for (const auto& vertex : vertices) {
        if (std::isfinite(vertex.x) && std::isfinite(vertex.y) && std::isfinite(vertex.z)) {
            points.push_back(vertex);
        }
    }
    std::sort(points.begin(), points.end(), [](const auto& left, const auto& right) {
        if (left.x == right.x) {
            return left.y < right.y;
        }
        return left.x < right.x;
    });
    points.erase(
        std::unique(points.begin(), points.end(), [](const auto& left, const auto& right) {
            return std::abs(left.x - right.x) <= 1.0e-5F &&
                   std::abs(left.y - right.y) <= 1.0e-5F;
        }),
        points.end());
    if (points.size() <= 3U) {
        return points;
    }

    std::vector<invisible_places::io::Float3> hull;
    hull.reserve(points.size() * 2U);
    for (const auto& point : points) {
        while (hull.size() >= 2U &&
               CrossXy(hull[hull.size() - 2U], hull.back(), point) <= 0.0F) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const auto lowerCount = hull.size();
    for (std::size_t index = points.size(); index-- > 0U;) {
        const auto& point = points[index];
        while (hull.size() > lowerCount &&
               CrossXy(hull[hull.size() - 2U], hull.back(), point) <= 0.0F) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    if (!hull.empty()) {
        hull.pop_back();
    }
    return hull.size() >= 3U ? hull : points;
}

std::vector<invisible_places::io::Float3> BuildWaterRegionBoundary(
    const std::vector<invisible_places::io::Float3>& vertices) {
    std::vector<invisible_places::io::Float3> boundary;
    boundary.reserve(vertices.size());
    for (const auto& vertex : vertices) {
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z)) {
            continue;
        }
        if (!boundary.empty() &&
            glm::length(ToGlm(vertex) - ToGlm(boundary.back())) <= 1.0e-5F) {
            continue;
        }
        boundary.push_back(vertex);
    }
    if (boundary.size() >= 2U &&
        glm::length(ToGlm(boundary.front()) - ToGlm(boundary.back())) <= 1.0e-5F) {
        boundary.pop_back();
    }
    return boundary;
}

float AverageWaterRegionZ(const std::vector<invisible_places::io::Float3>& vertices) {
    if (vertices.empty()) {
        return 0.0F;
    }
    double sum = 0.0;
    std::size_t count = 0U;
    for (const auto& vertex : vertices) {
        if (std::isfinite(vertex.z)) {
            sum += static_cast<double>(vertex.z);
            ++count;
        }
    }
    return count == 0U ? 0.0F : static_cast<float>(sum / static_cast<double>(count));
}

void RefreshWaterBasinRegionDerivedValues(WaterBasinRegion* region) {
    if (region == nullptr) {
        return;
    }
    region->hull = BuildWaterRegionHull(region->vertices);
    region->baseZ = AverageWaterRegionZ(region->vertices);
    region->heightAbove = std::clamp(region->heightAbove, 0.0F, 20.0F);
    region->depthBelow = std::clamp(region->depthBelow, 0.0F, 20.0F);
    region->density = std::clamp(region->density, 0.01F, 20.0F);
    if (region->outletEdgeIndex.has_value() &&
        (region->hull.size() < 2U || region->outletEdgeIndex.value() >= region->hull.size())) {
        region->outletEdgeIndex.reset();
    }
}

void RefreshWaterRunoffRegionDerivedValues(WaterRunoffRegion* region) {
    if (region == nullptr) {
        return;
    }
    region->hull = BuildWaterRegionHull(region->vertices);
    region->groundVoxelSize = std::clamp(region->groundVoxelSize, 0.03F, 10.0F);
    region->highPointFraction = std::clamp(region->highPointFraction, 0.01F, 0.95F);
    region->density = std::clamp(region->density, 0.01F, 20.0F);
    region->pathLength = std::clamp(region->pathLength, 0.5F, 300.0F);
    region->maxSteps = std::clamp(region->maxSteps, 4.0F, 500.0F);
}

void RefreshWaterCausticRegionDerivedValues(WaterCausticRegion* region) {
    if (region == nullptr) {
        return;
    }
    region->hull = BuildWaterRegionHull(region->vertices);
    region->maskVoxelSize = std::clamp(region->maskVoxelSize, 0.005F, 20.0F);
    region->planeMaxResidual = std::clamp(region->planeMaxResidual, 0.005F, 5.0F);
    region->planeMaxSlope = std::clamp(region->planeMaxSlope, 0.02F, 5.0F);
    region->heightBand = std::clamp(region->heightBand, 0.01F, 20.0F);
    region->edgeBlendWidth = std::clamp(region->edgeBlendWidth, 0.01F, 50.0F);
    region->maskDirty = true;
    region->maskStale = true;
}

namespace {

float PolygonEdgeDistanceXy(
    const glm::vec3& point,
    const std::vector<invisible_places::io::Float3>& polygon) {
    if (polygon.size() < 2U) {
        return 0.0F;
    }
    float distance = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const auto next = (index + 1U) % polygon.size();
        distance = std::min(distance, DistanceToSegmentXy(point, polygon[index], polygon[next]));
    }
    return distance == std::numeric_limits<float>::max() ? 0.0F : distance;
}

float PolygonEdgeDistance3d(
    const glm::vec3& point,
    const std::vector<invisible_places::io::Float3>& polygon) {
    if (polygon.size() < 2U) {
        return 0.0F;
    }
    float distance = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const auto next = (index + 1U) % polygon.size();
        distance = std::min(distance, DistanceToSegment3d(point, polygon[index], polygon[next]));
    }
    return distance == std::numeric_limits<float>::max() ? 0.0F : distance;
}

}  // namespace

WaterCausticMaskResult GenerateCausticMask(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterCausticRegion>& regions) {
    WaterCausticMaskResult result;
    const auto pointCount = cloud.PointCount();
    result.mask.assign(pointCount, 0.0F);
    result.edge.assign(pointCount, 0.0F);
    result.regionId.assign(pointCount, 0.0F);
    result.planeDistance.assign(pointCount, 0.0F);
    result.seed.assign(pointCount, 0.0F);
    if (pointCount == 0U) {
        return result;
    }

    for (auto region : regions) {
        RefreshWaterCausticRegionDerivedValues(&region);
        const auto boundary = BuildWaterRegionBoundary(region.vertices);
        if (!region.enabled || boundary.size() < 3U) {
            continue;
        }
        result.hasAnyEnabledRegion = true;
        const float regionSeed = RegionHash01(region.id, 0U, 401U);
        for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
            const auto& source = cloud.positions[pointIndex];
            const glm::vec3 position = ToGlm(source);
            if (!IsValidPoint(position)) {
                continue;
            }
            if (!PointInPolygonXy(position, boundary)) {
                continue;
            }
            float edgeDistance = PolygonEdgeDistance3d(position, boundary);
            if (!std::isfinite(edgeDistance)) {
                edgeDistance = PolygonEdgeDistanceXy(position, boundary);
            }
            const float edge = SmoothStep(0.0F, region.edgeBlendWidth, edgeDistance);
            const bool replacesExisting =
                result.mask[pointIndex] <= 0.0F ||
                edge > result.edge[pointIndex] + 1.0e-5F ||
                (std::abs(edge - result.edge[pointIndex]) <= 1.0e-5F &&
                 (result.regionId[pointIndex] <= 0.0F ||
                  static_cast<float>(region.id) < result.regionId[pointIndex]));
            if (!replacesExisting) {
                continue;
            }
            if (result.mask[pointIndex] <= 0.0F) {
                ++result.affectedPointCount;
            }
            result.mask[pointIndex] = 1.0F;
            result.edge[pointIndex] = edge;
            result.regionId[pointIndex] = static_cast<float>(region.id);
            result.planeDistance[pointIndex] = 0.0F;
            result.seed[pointIndex] = regionSeed;
        }
    }
    return result;
}

WaterOverlay GenerateBasinHazeOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterBasinRegion>& regions) {
    WaterOverlay overlay;
    struct BasinSteamCell {
        glm::vec3 position{0.0F};
        std::size_t pointIndex = 0U;
        std::uint32_t count = 0U;
        bool valid = false;
    };

    for (auto region : regions) {
        RefreshWaterBasinRegionDerivedValues(&region);
        if (region.hull.size() < 3U) {
            continue;
        }
        const float minZ = region.baseZ - std::max(0.0F, region.depthBelow);
        const float maxZ = region.baseZ + std::max(0.0F, region.heightAbove);
        const float density = std::clamp(region.density, 0.01F, 20.0F);
        const float plumeHeight = std::clamp(region.heightAbove, 0.03F, 20.0F);
        const float cellSize = std::clamp(0.34F / std::sqrt(density), 0.035F, 0.65F);
        std::unordered_map<GridKey, BasinSteamCell, GridKeyHash> cells;

        for (std::size_t pointIndex = 0U; pointIndex < cloud.positions.size(); ++pointIndex) {
            const glm::vec3 source = ToGlm(cloud.positions[pointIndex]);
            if (!IsValidPoint(source) ||
                source.z < minZ ||
                source.z > maxZ ||
                !PointInPolygonXy(source, region.hull)) {
                continue;
            }

            const GridKey key{
                static_cast<int>(std::floor(source.x / cellSize)),
                static_cast<int>(std::floor(source.y / cellSize)),
                0};
            auto& cell = cells[key];
            ++cell.count;
            if (!cell.valid || source.z < cell.position.z) {
                cell.position = source;
                cell.pointIndex = pointIndex;
                cell.valid = true;
            }
        }

        std::vector<BasinSteamCell> sites;
        sites.reserve(cells.size());
        for (const auto& [key, cell] : cells) {
            if (cell.valid) {
                sites.push_back(cell);
            }
        }
        std::sort(sites.begin(), sites.end(), [](const BasinSteamCell& left, const BasinSteamCell& right) {
            if (std::abs(left.position.z - right.position.z) > 1.0e-5F) {
                return left.position.z < right.position.z;
            }
            return left.pointIndex < right.pointIndex;
        });

        const float keepProbability = std::min(1.0F, 0.18F * density);
        const std::uint32_t maxPlumes = static_cast<std::uint32_t>(std::clamp<float>(
            20.0F * density,
            4.0F,
            800.0F));
        std::uint32_t emittedPlumes = 0U;
        for (std::size_t siteIndex = 0U; siteIndex < sites.size(); ++siteIndex) {
            if (emittedPlumes >= maxPlumes) {
                break;
            }
            const auto& site = sites[siteIndex];
            const bool forceFallbackPlume = emittedPlumes == 0U && siteIndex + 1U == sites.size();
            const auto sourceIndex = static_cast<std::uint32_t>(
                std::min<std::size_t>(site.pointIndex, std::numeric_limits<std::uint32_t>::max()));
            if (!forceFallbackPlume && RegionHash01(region.id, sourceIndex, 17U) > keepProbability) {
                continue;
            }
            float outletMask = 1.0F;
            if (!region.outletBlocked && region.outletEdgeIndex.has_value() && region.hull.size() >= 2U) {
                const auto edgeIndex = region.outletEdgeIndex.value() % static_cast<std::uint32_t>(region.hull.size());
                const auto nextIndex = (edgeIndex + 1U) % region.hull.size();
                const float edgeDistance = DistanceToSegmentXy(site.position, region.hull[edgeIndex], region.hull[nextIndex]);
                outletMask = 0.35F + 0.65F * SmoothStep(0.03F, 0.45F, edgeDistance);
                if (!forceFallbackPlume && RegionHash01(region.id, sourceIndex, 31U) > outletMask) {
                    continue;
                }
            }

            const float edgeDistance = PolygonEdgeDistanceXy(site.position, region.hull);
            const float edgeMask = SmoothStep(cellSize * 0.5F, cellSize * 2.5F, edgeDistance);
            const float siteStrength = std::clamp(0.35F + edgeMask * 0.65F, 0.0F, 1.0F) * outletMask;
            const float angle = RegionHash01(region.id, sourceIndex, 43U) * 6.28318530718F;
            const float bend = (RegionHash01(region.id, sourceIndex, 59U) - 0.5F) * 0.45F;
            const float driftLength =
                plumeHeight * (0.10F + RegionHash01(region.id, sourceIndex, 71U) * 0.24F);
            const glm::vec3 drift{
                std::cos(angle) * driftLength,
                std::sin(angle) * driftLength,
                0.0F};
            const glm::vec3 crossDrift{
                -std::sin(angle) * driftLength * bend,
                std::cos(angle) * driftLength * bend,
                0.0F};
            const float rise = plumeHeight * (0.75F + RegionHash01(region.id, sourceIndex, 83U) * 0.55F);
            const float speed = 0.60F + RegionHash01(region.id, sourceIndex, 97U) * 0.85F;
            const std::uint32_t flowId = (region.id * 100000U) + emittedPlumes + 1U;

            std::vector<WaterOverlayPoint> path;
            path.reserve(5U);
            for (std::uint32_t anchorIndex = 0U; anchorIndex < 5U; ++anchorIndex) {
                const float t = static_cast<float>(anchorIndex) / 4.0F;
                const float eased = t * t * (3.0F - 2.0F * t);
                const float swirl = std::sin((t + RegionHash01(region.id, sourceIndex, 109U)) * 6.28318530718F);
                glm::vec3 position =
                    site.position +
                    drift * eased +
                    crossDrift * eased * swirl +
                    glm::vec3{0.0F, 0.0F, rise * t};
                position.z = std::clamp(position.z + 0.012F, minZ, region.baseZ + plumeHeight * 1.35F);

                WaterOverlayPoint point;
                point.position = FromGlm(position);
                point.red = FloatToByte(0.76F + t * 0.12F);
                point.green = FloatToByte(0.91F + t * 0.07F);
                point.blue = FloatToByte(1.0F);
                point.flowId = static_cast<float>(flowId);
                point.emitterId = static_cast<float>(region.id);
                point.phase = RegionHash01(region.id, flowId + anchorIndex, 127U);
                point.speed = speed;
                point.width = std::max(0.025F, plumeHeight * (0.055F + t * 0.18F));
                point.confidence = Clamp01(siteStrength * (1.0F - t * 0.35F));
                point.accumulation = t;
                point.pooling = 0.0F;
                point.featureType = 1.0F;
                point.regionId = static_cast<float>(region.id);
                point.surfaceSteepness = t;
                path.push_back(point);
            }

            RecomputePathDistances(&path, std::max(0.1F, rise));
            WaterParticleTrailShapeSettings shape;
            shape.particleJitter = std::clamp(0.70F + density * 0.035F, 0.70F, 1.35F);
            shape.splineAnchorSpacing = std::clamp(plumeHeight * 0.20F, 0.045F, 0.18F);
            WaterAnimationTrailSettings trail;
            trail.particleDensity = std::clamp(0.50F + density * 0.18F, 0.35F, 4.0F);
            trail.particleSpeed = std::clamp(speed, 0.05F, 8.0F);
            trail.colorVariation = 0.30F;
            trail.trailLengthMeters = std::clamp(plumeHeight * 0.42F, 0.04F, 2.0F);
            IncludeWaterPathWithParticles(
                &overlay,
                std::move(path),
                shape,
                trail,
                1.0F,
                static_cast<float>(region.id));
            ++emittedPlumes;
        }
    }
    return overlay;
}

WaterOverlay GenerateRunoffOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterRunoffRegion>& regions,
    const WaterAnimationTrailSettings& animationTrailSettings) {
    WaterOverlay overlay;
    struct GroundCell {
        std::vector<float> zValues;
        glm::vec3 positionSum{0.0F};
        std::uint32_t count = 0U;
        float groundZ = 0.0F;
    };
    struct Candidate {
        std::size_t pointIndex = 0U;
        GridKey cell{};
        float height = 0.0F;
    };

    for (auto region : regions) {
        RefreshWaterRunoffRegionDerivedValues(&region);
        if (region.hull.size() < 3U) {
            continue;
        }

        const float cellSize = std::max(0.03F, region.groundVoxelSize);
        std::unordered_map<GridKey, GroundCell, GridKeyHash> cells;
        for (std::size_t pointIndex = 0U; pointIndex < cloud.positions.size(); ++pointIndex) {
            const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
            if (!IsValidPoint(position) || !PointInPolygonXy(position, region.hull)) {
                continue;
            }
            const GridKey key{
                static_cast<int>(std::floor(position.x / cellSize)),
                static_cast<int>(std::floor(position.y / cellSize)),
                0};
            auto& cell = cells[key];
            cell.zValues.push_back(position.z);
            cell.positionSum += position;
            ++cell.count;
        }
        if (cells.empty()) {
            continue;
        }
        for (auto& [key, cell] : cells) {
            std::sort(cell.zValues.begin(), cell.zValues.end());
            const auto percentileIndex = static_cast<std::size_t>(
                std::clamp<float>(static_cast<float>(cell.zValues.size() - 1U) * 0.18F, 0.0F, static_cast<float>(cell.zValues.size() - 1U)));
            cell.groundZ = cell.zValues[percentileIndex];
        }

        std::vector<Candidate> candidates;
        for (std::size_t pointIndex = 0U; pointIndex < cloud.positions.size(); ++pointIndex) {
            const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
            if (!IsValidPoint(position) || !PointInPolygonXy(position, region.hull)) {
                continue;
            }
            const GridKey key{
                static_cast<int>(std::floor(position.x / cellSize)),
                static_cast<int>(std::floor(position.y / cellSize)),
                0};
            const auto cellIt = cells.find(key);
            if (cellIt == cells.end()) {
                continue;
            }
            const float height = position.z - cellIt->second.groundZ;
            if (height > std::max(0.08F, cellSize * 0.15F)) {
                candidates.push_back({pointIndex, key, height});
            }
        }
        if (candidates.empty()) {
            continue;
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right) {
            return left.height > right.height;
        });
        const std::size_t candidateLimit = std::max<std::size_t>(
            1U,
            static_cast<std::size_t>(std::ceil(static_cast<float>(candidates.size()) * region.highPointFraction)));
        const float modeDensity = region.mode == WaterRunoffMode::LightRain ? 1.65F : 0.55F;
        const float modeSpeed = region.mode == WaterRunoffMode::LightRain ? 1.45F : 0.65F;
        const float keepProbability = std::min(1.0F, 0.05F * std::clamp(region.density, 0.01F, 20.0F) * modeDensity);
        const std::uint32_t maxPaths = static_cast<std::uint32_t>(std::clamp<float>(
            350.0F * region.density * modeDensity,
            4.0F,
            2500.0F));
        std::uint32_t emittedPaths = 0U;
        for (std::size_t candidateIndex = 0U;
             candidateIndex < candidateLimit && emittedPaths < maxPaths;
             ++candidateIndex) {
            const auto& candidate = candidates[candidateIndex];
            if (RegionHash01(region.id, static_cast<std::uint32_t>(candidate.pointIndex), 151U) > keepProbability) {
                continue;
            }
            const glm::vec3 start = ToGlm(cloud.positions[candidate.pointIndex]);
            const auto startCellIt = cells.find(candidate.cell);
            if (startCellIt == cells.end()) {
                continue;
            }
            const float groundZ = startCellIt->second.groundZ;
            glm::vec3 normal{0.0F, 0.0F, 1.0F};
            if (cloud.hasNormals && candidate.pointIndex < cloud.normals.size()) {
                const auto loadedNormal = ToGlm(cloud.normals[candidate.pointIndex]);
                normal = SafeOverlayNormal(loadedNormal);
            }

            std::vector<WaterOverlayPoint> path;
            const std::uint32_t flowId = (region.id * 100000U) + emittedPaths + 1U;
            auto appendPathPoint = [&](glm::vec3 position, float confidence, float accumulation) {
                WaterOverlayPoint point;
                point.position = FromGlm(position);
                point.normal = FromGlm(normal);
                point.red = FloatToByte(0.03F);
                point.green = FloatToByte(0.68F + accumulation * 0.20F);
                point.blue = FloatToByte(0.95F + confidence * 0.05F);
                point.flowId = static_cast<float>(flowId);
                point.emitterId = 0.0F;
                point.phase = RegionHash01(region.id, flowId, 181U);
                point.speed = modeSpeed;
                point.width = 0.18F;
                point.confidence = Clamp01(confidence);
                point.accumulation = Clamp01(accumulation);
                point.featureType = 2.0F;
                point.regionId = static_cast<float>(region.id);
                path.push_back(point);
            };

            appendPathPoint(start, 0.8F, 0.0F);
            glm::vec3 trickleDir = kGravity - normal * glm::dot(kGravity, normal);
            if (glm::dot(trickleDir, trickleDir) <= kNormalEpsilon) {
                trickleDir = glm::vec3{
                    RegionHash01(region.id, flowId, 191U) - 0.5F,
                    RegionHash01(region.id, flowId, 193U) - 0.5F,
                    -0.35F};
            }
            trickleDir = glm::normalize(trickleDir);
            const float vegetationDrop = std::max(0.05F, std::min(candidate.height * 0.55F, cellSize * 1.8F));
            appendPathPoint(
                start + trickleDir * vegetationDrop + glm::vec3{0.0F, 0.0F, -vegetationDrop * 0.35F},
                0.72F,
                0.18F);
            glm::vec3 current{
                start.x,
                start.y,
                groundZ + 0.018F};
            appendPathPoint(current, 0.65F, 0.34F);

            GridKey currentCell = candidate.cell;
            float travelled = 0.0F;
            const std::uint32_t maxSteps = static_cast<std::uint32_t>(std::clamp(region.maxSteps, 4.0F, 500.0F));
            for (std::uint32_t step = 0U; step < maxSteps && travelled < region.pathLength; ++step) {
                std::optional<GridKey> bestKey;
                float bestScore = -std::numeric_limits<float>::max();
                const auto currentIt = cells.find(currentCell);
                if (currentIt == cells.end()) {
                    break;
                }
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        const GridKey nextKey{currentCell.x + dx, currentCell.y + dy, 0};
                        const auto nextIt = cells.find(nextKey);
                        if (nextIt == cells.end()) {
                            continue;
                        }
                        const glm::vec3 nextCenter = nextIt->second.positionSum /
                                                     static_cast<float>(std::max(1U, nextIt->second.count));
                        if (!PointInPolygonXy(nextCenter, region.hull)) {
                            continue;
                        }
                        const float drop = currentIt->second.groundZ - nextIt->second.groundZ;
                        const float diagonalPenalty = (dx != 0 && dy != 0) ? 0.08F : 0.0F;
                        const float score = drop - diagonalPenalty +
                                            RegionHash01(region.id, flowId + step, static_cast<std::uint32_t>(dx * 13 + dy * 29 + 233U)) * 0.05F;
                        if (score > bestScore) {
                            bestScore = score;
                            bestKey = nextKey;
                        }
                    }
                }
                if (!bestKey.has_value() || bestScore < -cellSize * 0.08F) {
                    break;
                }
                const auto nextIt = cells.find(bestKey.value());
                const glm::vec3 nextCenter = nextIt->second.positionSum /
                                             static_cast<float>(std::max(1U, nextIt->second.count));
                glm::vec3 nextPosition{nextCenter.x, nextCenter.y, nextIt->second.groundZ + 0.018F};
                travelled += glm::length(nextPosition - current);
                current = nextPosition;
                currentCell = bestKey.value();
                appendPathPoint(current, 0.62F, std::min(1.0F, 0.34F + travelled / std::max(0.1F, region.pathLength)));
            }

            if (path.size() >= 3U) {
                RecomputePathDistances(&path, std::max(0.1F, region.pathLength));
                WaterParticleTrailShapeSettings shape;
                shape.particleJitter = region.mode == WaterRunoffMode::LightRain ? 0.18F : 0.10F;
                shape.splineAnchorSpacing = std::max(0.08F, cellSize * 0.75F);
                auto trail = animationTrailSettings;
                trail.particleDensity = std::clamp(trail.particleDensity * region.density * modeDensity, 0.05F, 10.0F);
                trail.particleSpeed = std::clamp(trail.particleSpeed * modeSpeed, 0.05F, 8.0F);
                IncludeWaterPathWithParticles(&overlay, std::move(path), shape, trail, 2.0F, static_cast<float>(region.id));
                ++emittedPaths;
            }
        }
    }
    return overlay;
}

bool WriteWaterOverlayPly(
    const WaterOverlay& overlay,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    if (overlay.points.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "No water overlay points were generated.";
        }
        return false;
    }

    if (const auto parent = outputPath.parent_path(); !parent.empty()) {
        std::error_code createError;
        std::filesystem::create_directories(parent, createError);
        if (createError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to create water output directory: " + createError.message();
            }
            return false;
        }
    }

    std::ofstream output{outputPath, std::ios::binary | std::ios::trunc};
    if (!output.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Unable to open water overlay PLY for writing.";
        }
        return false;
    }

    output << "ply\n";
    output << "format binary_little_endian 1.0\n";
    output << "comment Generated by Invisible Places water flow overlay\n";
    output << "element vertex " << overlay.points.size() << "\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "property float normal_x\n";
    output << "property float normal_y\n";
    output << "property float normal_z\n";
    output << "property uchar red\n";
    output << "property uchar green\n";
    output << "property uchar blue\n";
    output << "property float scalar_flow_id\n";
    output << "property float scalar_emitter_id\n";
    output << "property float scalar_path_distance\n";
    output << "property float scalar_phase\n";
    output << "property float scalar_speed\n";
    output << "property float scalar_width\n";
    output << "property float scalar_confidence\n";
    output << "property float scalar_accumulation\n";
    output << "property float scalar_pooling\n";
    output << "property float scalar_particle_role\n";
    output << "property float scalar_path_start_index\n";
    output << "property float scalar_path_point_count\n";
    output << "property float scalar_jitter_seed\n";
    output << "property float scalar_trail_age\n";
    output << "property float scalar_trail_length\n";
    output << "property float scalar_feature_type\n";
    output << "property float scalar_region_id\n";
    output << "property float scalar_surface_steepness\n";
    output << "property float scalar_trail_lane_id\n";
    output << "property float scalar_trail_lateral_offset\n";
    output << "end_header\n";

    for (const auto& point : overlay.points) {
        WriteFloat(output, point.position.x);
        WriteFloat(output, point.position.y);
        WriteFloat(output, point.position.z);
        const auto normal = FromGlm(SafeOverlayNormal(ToGlm(point.normal)));
        WriteFloat(output, normal.x);
        WriteFloat(output, normal.y);
        WriteFloat(output, normal.z);
        WriteUchar(output, point.red);
        WriteUchar(output, point.green);
        WriteUchar(output, point.blue);
        WriteFloat(output, point.flowId);
        WriteFloat(output, point.emitterId);
        WriteFloat(output, point.pathDistance);
        WriteFloat(output, point.phase);
        WriteFloat(output, point.speed);
        WriteFloat(output, point.width);
        WriteFloat(output, point.confidence);
        WriteFloat(output, point.accumulation);
        WriteFloat(output, point.pooling);
        WriteFloat(output, point.particleRole);
        WriteFloat(output, point.pathStartIndex);
        WriteFloat(output, point.pathPointCount);
        WriteFloat(output, point.jitterSeed);
        WriteFloat(output, point.trailAge);
        WriteFloat(output, point.trailLength);
        WriteFloat(output, point.featureType);
        WriteFloat(output, point.regionId);
        WriteFloat(output, point.surfaceSteepness);
        WriteFloat(output, point.trailLaneId);
        WriteFloat(output, point.trailLateralOffset);
    }

    if (!output.good()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed while writing water overlay PLY.";
        }
        return false;
    }

    return true;
}

invisible_places::io::LoadedPointCloud BuildWaterOverlayPointCloud(
    const WaterOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = sourcePath;
    cloud.layerName = std::string{layerName};
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    cloud.positions.reserve(overlay.points.size());
    cloud.normals.reserve(overlay.points.size());
    cloud.packedColors.reserve(overlay.points.size());
    cloud.bounds = overlay.bounds;

    for (const auto& point : overlay.points) {
        cloud.positions.push_back(point.position);
        cloud.normals.push_back(FromGlm(SafeOverlayNormal(ToGlm(point.normal))));
        cloud.packedColors.push_back(PackRgba8(point.red, point.green, point.blue));
        if (!cloud.bounds.valid) {
            cloud.bounds.Expand(point.position);
        }
    }

    if (!cloud.positions.empty()) {
        cloud.focusPoint = cloud.bounds.valid
                               ? invisible_places::io::Float3{
                                     (cloud.bounds.minimum.x + cloud.bounds.maximum.x) * 0.5F,
                                     (cloud.bounds.minimum.y + cloud.bounds.maximum.y) * 0.5F,
                                     (cloud.bounds.minimum.z + cloud.bounds.maximum.z) * 0.5F}
                               : cloud.positions.front();
        cloud.hasFocusPoint = true;
    }

    struct OverlayScalarField {
        std::string_view name;
        float (*value)(const WaterOverlayPoint&);
    };
    const OverlayScalarField fields[] = {
        {"flow_id", [](const WaterOverlayPoint& point) { return point.flowId; }},
        {"emitter_id", [](const WaterOverlayPoint& point) { return point.emitterId; }},
        {"path_distance", [](const WaterOverlayPoint& point) { return point.pathDistance; }},
        {"phase", [](const WaterOverlayPoint& point) { return point.phase; }},
        {"speed", [](const WaterOverlayPoint& point) { return point.speed; }},
        {"width", [](const WaterOverlayPoint& point) { return point.width; }},
        {"confidence", [](const WaterOverlayPoint& point) { return point.confidence; }},
        {"accumulation", [](const WaterOverlayPoint& point) { return point.accumulation; }},
        {"pooling", [](const WaterOverlayPoint& point) { return point.pooling; }},
        {"particle_role", [](const WaterOverlayPoint& point) { return point.particleRole; }},
        {"path_start_index", [](const WaterOverlayPoint& point) { return point.pathStartIndex; }},
        {"path_point_count", [](const WaterOverlayPoint& point) { return point.pathPointCount; }},
        {"jitter_seed", [](const WaterOverlayPoint& point) { return point.jitterSeed; }},
        {"trail_age", [](const WaterOverlayPoint& point) { return point.trailAge; }},
        {"trail_length", [](const WaterOverlayPoint& point) { return point.trailLength; }},
        {"feature_type", [](const WaterOverlayPoint& point) { return point.featureType; }},
        {"region_id", [](const WaterOverlayPoint& point) { return point.regionId; }},
        {"surface_steepness", [](const WaterOverlayPoint& point) { return point.surfaceSteepness; }},
        {"trail_lane_id", [](const WaterOverlayPoint& point) { return point.trailLaneId; }},
        {"trail_lateral_offset", [](const WaterOverlayPoint& point) { return point.trailLateralOffset; }},
    };

    cloud.scalarFields.reserve(std::size(fields));
    cloud.scalarFieldValues.reserve(overlay.points.size() * std::size(fields));
    for (const auto& field : fields) {
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::string{field.name};
        for (const auto& point : overlay.points) {
            const float value = field.value(point);
            cloud.scalarFieldValues.push_back(value);
            stats.Include(value);
        }
        cloud.scalarFields.push_back(std::move(stats));
    }

    return cloud;
}

}  // namespace invisible_places::water
