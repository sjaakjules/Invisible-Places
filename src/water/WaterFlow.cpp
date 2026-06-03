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
#include <glm/vec4.hpp>

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

float Cross2d(const glm::vec2& left, const glm::vec2& right) {
    return left.x * right.y - left.y * right.x;
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

bool BoundsContainsXy(
    const invisible_places::io::Bounds3f& bounds,
    const invisible_places::io::Float3& point) {
    if (!bounds.valid) {
        return true;
    }
    constexpr float kBoundsEpsilon = 1.0e-5F;
    return point.x >= bounds.minimum.x - kBoundsEpsilon &&
           point.x <= bounds.maximum.x + kBoundsEpsilon &&
           point.y >= bounds.minimum.y - kBoundsEpsilon &&
           point.y <= bounds.maximum.y + kBoundsEpsilon;
}

bool PointVisibleInClip(
    const glm::mat4& viewProjection,
    const invisible_places::io::Float3& point) {
    const glm::vec4 clip = viewProjection * glm::vec4{point.x, point.y, point.z, 1.0F};
    if (clip.w <= 1.0e-6F) {
        return false;
    }
    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    return ndc.x >= -1.0F && ndc.x <= 1.0F &&
           ndc.y >= -1.0F && ndc.y <= 1.0F &&
           ndc.z >= -1.0F && ndc.z <= 1.0F;
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

float FiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

float ClampFinite01(float value) {
    return Clamp01(FiniteOr(value, 0.0F));
}

float SafeAcos(float value) {
    return std::acos(std::clamp(value, -1.0F, 1.0F));
}

std::vector<WaterOverlayPoint> OrderedAnalysisAnchors(const WaterPathBranch& branch) {
    auto anchors = branch.rawAnchors;
    if (anchors.empty()) {
        return anchors;
    }
    std::stable_sort(
        anchors.begin(),
        anchors.end(),
        [](const WaterOverlayPoint& left, const WaterOverlayPoint& right) {
            return left.pathDistance < right.pathDistance;
        });

    bool monotonic = true;
    for (std::size_t index = 1U; index < anchors.size(); ++index) {
        if (anchors[index].pathDistance <= anchors[index - 1U].pathDistance) {
            monotonic = false;
            break;
        }
    }
    if (!monotonic) {
        RecomputePathDistances(&anchors, std::max(0.001F, branch.length));
    }
    return anchors;
}

float AnalysisRadiusForCache(const WaterPathCache& cache) {
    const float spacing = PositiveOr(
        cache.tunedSettings.pathSampleSpacing,
        PositiveOr(cache.diagnostics.pathSampleSpacing, cache.requestedSettings.pathSampleSpacing));
    const float voxel = PositiveOr(
        cache.tunedSettings.supportVoxelSize,
        PositiveOr(cache.diagnostics.supportVoxelSize, cache.requestedSettings.supportVoxelSize));
    const float bridge = PositiveOr(
        cache.tunedSettings.maxBridgeDistance,
        PositiveOr(cache.diagnostics.maxBridgeDistance, cache.requestedSettings.maxBridgeDistance));
    return std::clamp(
        std::max({spacing * 8.0F, voxel * 8.0F, bridge * 0.75F, 0.025F}),
        0.025F,
        2.5F);
}

std::size_t FindAnalysisWindowIndex(
    const std::vector<WaterOverlayPoint>& anchors,
    std::size_t index,
    float targetDistance,
    bool searchLeft) {
    if (anchors.empty()) {
        return 0U;
    }
    if (searchLeft) {
        std::size_t best = index;
        while (best > 0U && anchors[best].pathDistance > targetDistance) {
            --best;
        }
        return best == index && index > 0U ? index - 1U : best;
    }

    std::size_t best = index;
    while (best + 1U < anchors.size() && anchors[best].pathDistance < targetDistance) {
        ++best;
    }
    return best == index && index + 1U < anchors.size() ? index + 1U : best;
}

float AnalysisSlopeAt(
    const std::vector<WaterOverlayPoint>& anchors,
    std::size_t index,
    float windowDistance) {
    if (anchors.size() < 2U || index >= anchors.size()) {
        return 0.0F;
    }
    const float pathDistance = anchors[index].pathDistance;
    const std::size_t left = FindAnalysisWindowIndex(anchors, index, pathDistance - windowDistance, true);
    const std::size_t right = FindAnalysisWindowIndex(anchors, index, pathDistance + windowDistance, false);
    if (left == right || right >= anchors.size()) {
        return 0.0F;
    }
    const float distance = std::max(1.0e-4F, std::abs(anchors[right].pathDistance - anchors[left].pathDistance));
    const float dz = std::abs(anchors[right].position.z - anchors[left].position.z);
    constexpr float slopeHigh = 0.70F;
    return Clamp01((dz / distance) / slopeHigh);
}

float AnalysisCurvatureAt(
    const std::vector<WaterOverlayPoint>& anchors,
    std::size_t index,
    float* signedCurvature) {
    if (signedCurvature != nullptr) {
        *signedCurvature = 0.0F;
    }
    if (anchors.size() < 3U || index == 0U || index + 1U >= anchors.size()) {
        return 0.0F;
    }
    const glm::vec3 previous = ToGlm(anchors[index - 1U].position);
    const glm::vec3 current = ToGlm(anchors[index].position);
    const glm::vec3 next = ToGlm(anchors[index + 1U].position);
    glm::vec3 incoming = current - previous;
    glm::vec3 outgoing = next - current;
    if (glm::dot(incoming, incoming) <= 1.0e-8F || glm::dot(outgoing, outgoing) <= 1.0e-8F) {
        return 0.0F;
    }
    incoming = glm::normalize(incoming);
    outgoing = glm::normalize(outgoing);
    const float angle = SafeAcos(glm::dot(incoming, outgoing));
    constexpr float curvatureHighRadians = 1.57079637F;
    const float curvature = Clamp01(angle / curvatureHighRadians);
    if (signedCurvature != nullptr) {
        const float turnSign = glm::cross(incoming, outgoing).z < 0.0F ? -1.0F : 1.0F;
        *signedCurvature = curvature * turnSign;
    }
    return curvature;
}

void SmoothAnalysisField(
    std::vector<WaterPathAnalysisSample>* samples,
    float WaterPathAnalysisSample::*field,
    bool normalized) {
    if (samples == nullptr || samples->size() < 3U) {
        return;
    }
    std::vector<float> values;
    values.reserve(samples->size());
    for (const auto& sample : *samples) {
        values.push_back(sample.*field);
    }
    for (std::size_t index = 1U; index + 1U < samples->size(); ++index) {
        float value = (values[index - 1U] * 0.25F) + (values[index] * 0.50F) + (values[index + 1U] * 0.25F);
        if (normalized) {
            value = Clamp01(value);
        }
        (*samples)[index].*field = value;
    }
}

void SanitizeAnalysisSample(WaterPathAnalysisSample* sample) {
    if (sample == nullptr) {
        return;
    }
    sample->pathDistance = std::max(0.0F, FiniteOr(sample->pathDistance, 0.0F));
    sample->slope = ClampFinite01(sample->slope);
    sample->flatness = ClampFinite01(sample->flatness);
    sample->curvature = ClampFinite01(sample->curvature);
    sample->neighborDensity = ClampFinite01(sample->neighborDensity);
    sample->nearestPathDistance = std::max(0.0F, FiniteOr(sample->nearestPathDistance, 0.0F));
    sample->confluence = ClampFinite01(sample->confluence);
    sample->channelWidth = std::max(0.001F, FiniteOr(sample->channelWidth, 0.001F));
    sample->speed = std::max(0.0F, FiniteOr(sample->speed, 0.0F));
    sample->turbulence = ClampFinite01(sample->turbulence);
    sample->eddyPotential = ClampFinite01(sample->eddyPotential);
    sample->ripplePotential = ClampFinite01(sample->ripplePotential);
}

struct AnalysisFlatSample {
    std::size_t branchIndex = 0;
    std::size_t sampleIndex = 0;
    std::uint32_t branchId = 0;
    std::optional<std::uint32_t> parentId;
    std::uint32_t emitterId = 0;
    WaterPathBranchRole role = WaterPathBranchRole::Main;
    glm::vec3 position{0.0F};
    glm::vec3 tangent{0.0F, 0.0F, -1.0F};
    float pathDistance = 0.0F;
    float baseWidth = 0.01F;
    float baseSpeed = 0.45F;
    float confidence = 1.0F;
    float roughness = 0.0F;
};

bool BranchesRelatedForConfluence(const AnalysisFlatSample& left, const AnalysisFlatSample& right) {
    return left.emitterId == right.emitterId ||
           (left.parentId.has_value() && left.parentId.value() == right.branchId) ||
           (right.parentId.has_value() && right.parentId.value() == left.branchId) ||
           (left.parentId.has_value() && right.parentId.has_value() && left.parentId.value() == right.parentId.value());
}

void WriteFloat(std::ofstream& output, float value) {
    output.write(reinterpret_cast<const char*>(&value), sizeof(float));
}

void WriteUchar(std::ofstream& output, std::uint8_t value) {
    output.write(reinterpret_cast<const char*>(&value), sizeof(std::uint8_t));
}

}  // namespace

WaterPathAnalysisCache BuildWaterPathAnalysis(const WaterPathCache& cache) {
    WaterPathAnalysisCache analysis;
    analysis.analysisRadiusMeters = AnalysisRadiusForCache(cache);
    if (cache.branches.empty()) {
        return analysis;
    }

    std::vector<std::vector<WaterOverlayPoint>> orderedBranches;
    orderedBranches.reserve(cache.branches.size());
    analysis.branches.reserve(cache.branches.size());

    for (const auto& branch : cache.branches) {
        auto anchors = OrderedAnalysisAnchors(branch);
        WaterPathBranchAnalysis branchAnalysis;
        branchAnalysis.branchId = branch.id;
        branchAnalysis.samples.reserve(anchors.size());

        const float branchLength =
            anchors.size() < 2U
                ? 0.0F
                : std::max(0.0F, anchors.back().pathDistance - anchors.front().pathDistance);
        const float spacing = PositiveOr(
            cache.tunedSettings.pathSampleSpacing,
            PositiveOr(cache.diagnostics.pathSampleSpacing, cache.requestedSettings.pathSampleSpacing));
        const float windowDistance = std::clamp(
            std::max({spacing * 4.0F, analysis.analysisRadiusMeters * 0.35F, branchLength * 0.08F}),
            0.01F,
            std::max(0.01F, branchLength * 0.40F));

        for (std::size_t index = 0U; index < anchors.size(); ++index) {
            const auto& anchor = anchors[index];
            WaterPathAnalysisSample sample;
            sample.branchId = branch.id;
            sample.sampleIndex = static_cast<std::uint32_t>(
                std::min<std::size_t>(index, std::numeric_limits<std::uint32_t>::max()));
            sample.pathDistance = std::max(0.0F, anchor.pathDistance);
            sample.slope = AnalysisSlopeAt(anchors, index, windowDistance);
            sample.flatness = Clamp01(
                (1.0F - sample.slope) * 0.82F +
                Clamp01(anchor.pooling) * 0.10F +
                Clamp01(branch.flatness) * 0.08F);
            sample.curvature = AnalysisCurvatureAt(anchors, index, nullptr);
            sample.nearestPathDistance = analysis.analysisRadiusMeters * 1.5F;
            sample.channelWidth = std::max(0.001F, anchor.width);
            sample.speed = std::max(0.0F, anchor.speed);
            SanitizeAnalysisSample(&sample);
            branchAnalysis.samples.push_back(sample);
        }

        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::slope, true);
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::flatness, true);
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::curvature, true);
        orderedBranches.push_back(std::move(anchors));
        analysis.branches.push_back(std::move(branchAnalysis));
    }

    std::vector<AnalysisFlatSample> flatSamples;
    flatSamples.reserve([&]() {
        std::size_t total = 0U;
        for (const auto& branch : analysis.branches) {
            total += branch.samples.size();
        }
        return total;
    }());

    std::unordered_map<GridKey, std::vector<std::size_t>, GridKeyHash> sampleGrid;
    const float radius = std::max(0.001F, analysis.analysisRadiusMeters);
    for (std::size_t branchIndex = 0U; branchIndex < analysis.branches.size(); ++branchIndex) {
        const auto& branch = cache.branches[branchIndex];
        const auto& anchors = orderedBranches[branchIndex];
        const auto& branchAnalysis = analysis.branches[branchIndex];
        for (std::size_t sampleIndex = 0U;
             sampleIndex < branchAnalysis.samples.size() && sampleIndex < anchors.size();
             ++sampleIndex) {
            const auto& anchor = anchors[sampleIndex];
            AnalysisFlatSample flat;
            flat.branchIndex = branchIndex;
            flat.sampleIndex = sampleIndex;
            flat.branchId = branch.id;
            flat.parentId = branch.parentId;
            flat.emitterId = branch.emitterId;
            flat.role = branch.role;
            flat.position = ToGlm(anchor.position);
            flat.tangent = WaterPathTangent(anchors, sampleIndex);
            flat.pathDistance = anchor.pathDistance;
            flat.baseWidth = std::clamp(PositiveOr(anchor.width, radius * 0.08F), 0.001F, radius);
            flat.baseSpeed = std::clamp(PositiveOr(anchor.speed, 0.45F), 0.02F, 8.0F);
            flat.confidence = Clamp01(anchor.confidence * branch.confidence);
            flat.roughness = std::max(
                Clamp01(1.0F - flat.confidence),
                std::max(Clamp01(anchor.surfaceSteepness * 0.35F), Clamp01(static_cast<float>(branch.gapCount) / 3.0F)));
            const std::size_t flatIndex = flatSamples.size();
            flatSamples.push_back(flat);
            sampleGrid[MakeGridKey(flat.position, radius)].push_back(flatIndex);
        }
    }

    const int searchRadiusCells = 1;
    for (std::size_t flatIndex = 0U; flatIndex < flatSamples.size(); ++flatIndex) {
        const auto& sampleInfo = flatSamples[flatIndex];
        auto& sample = analysis.branches[sampleInfo.branchIndex].samples[sampleInfo.sampleIndex];
        const GridKey key = MakeGridKey(sampleInfo.position, radius);
        float neighborWeightSum = 0.0F;
        float confluenceWeightSum = 0.0F;
        float nearestDistance = std::numeric_limits<float>::max();

        for (int dz = -searchRadiusCells; dz <= searchRadiusCells; ++dz) {
            for (int dy = -searchRadiusCells; dy <= searchRadiusCells; ++dy) {
                for (int dx = -searchRadiusCells; dx <= searchRadiusCells; ++dx) {
                    const GridKey neighborKey{key.x + dx, key.y + dy, key.z + dz};
                    const auto cellIt = sampleGrid.find(neighborKey);
                    if (cellIt == sampleGrid.end()) {
                        continue;
                    }
                    for (const std::size_t otherFlatIndex : cellIt->second) {
                        if (otherFlatIndex == flatIndex || otherFlatIndex >= flatSamples.size()) {
                            continue;
                        }
                        const auto& other = flatSamples[otherFlatIndex];
                        const float sameBranchDistance = std::abs(other.pathDistance - sampleInfo.pathDistance);
                        if (other.branchId == sampleInfo.branchId && sameBranchDistance <= radius * 1.25F) {
                            continue;
                        }
                        const glm::vec3 delta = other.position - sampleInfo.position;
                        const float distance = glm::length(delta);
                        if (!std::isfinite(distance) || distance <= 1.0e-5F || distance > radius) {
                            continue;
                        }
                        nearestDistance = std::min(nearestDistance, distance);

                        const float distanceWeight = 1.0F - (distance / radius);
                        const float progressDelta = std::abs(other.pathDistance - sampleInfo.pathDistance);
                        const float progressWeight = 1.0F - Clamp01(progressDelta / std::max(radius * 2.0F, 1.0e-4F));
                        const float roleWeight =
                            other.role == WaterPathBranchRole::Spread || sampleInfo.role == WaterPathBranchRole::Spread
                                ? 1.15F
                                : 1.0F;
                        const float neighborWeight = distanceWeight * (0.65F + progressWeight * 0.35F) * roleWeight;
                        neighborWeightSum += neighborWeight;

                        const float directionAlignment =
                            Clamp01((glm::dot(sampleInfo.tangent, other.tangent) + 1.0F) * 0.5F);
                        const float relatedWeight = BranchesRelatedForConfluence(sampleInfo, other) ? 1.0F : 0.68F;
                        confluenceWeightSum += neighborWeight * directionAlignment * relatedWeight;
                    }
                }
            }
        }

        const bool hasNeighbor = nearestDistance < std::numeric_limits<float>::max();
        sample.nearestPathDistance = hasNeighbor ? nearestDistance : radius * 1.5F;
        sample.neighborDensity = Clamp01(neighborWeightSum / 2.75F);
        sample.confluence = Clamp01(confluenceWeightSum / 2.25F);
    }

    for (auto& branchAnalysis : analysis.branches) {
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::neighborDensity, true);
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::confluence, true);
    }

    const float minimumWidth = std::max(
        0.002F,
        PositiveOr(cache.tunedSettings.pathSampleSpacing, cache.requestedSettings.pathSampleSpacing) * 0.35F);
    const float maximumWidth = std::max(minimumWidth * 4.0F, radius * 2.50F);
    for (const auto& sampleInfo : flatSamples) {
        auto& sample = analysis.branches[sampleInfo.branchIndex].samples[sampleInfo.sampleIndex];
        const float nearest = std::max(0.0F, sample.nearestPathDistance);
        const bool hasNeighbor = nearest <= radius;
        const float neighborWidth =
            hasNeighbor ? std::clamp(nearest * (0.55F + sample.confluence * 0.35F), minimumWidth, maximumWidth)
                        : sampleInfo.baseWidth;
        const float widthSeed =
            std::max(sampleInfo.baseWidth, sampleInfo.baseWidth + (neighborWidth - sampleInfo.baseWidth) * sample.neighborDensity);
        const float flatSpread = 1.0F + sample.flatness * 0.75F;
        const float confluenceSpread = 1.0F + sample.confluence * 0.90F;
        const float isolationNarrow = 0.72F + sample.neighborDensity * 0.28F;
        const float steepNarrow = 1.0F - sample.slope * 0.36F;
        sample.channelWidth = std::clamp(
            widthSeed * flatSpread * confluenceSpread * isolationNarrow * steepNarrow,
            minimumWidth,
            maximumWidth);

        const float slopeSpeed = 0.35F + sample.slope * 1.65F;
        const float flatDamping = 1.0F - sample.flatness * 0.48F;
        const float confluenceDamping = 1.0F - sample.confluence * 0.16F;
        sample.speed = std::clamp(sampleInfo.baseSpeed * slopeSpeed * flatDamping * confluenceDamping, 0.02F, 8.0F);
    }

    for (const auto& sampleInfo : flatSamples) {
        auto& branchSamples = analysis.branches[sampleInfo.branchIndex].samples;
        auto& sample = branchSamples[sampleInfo.sampleIndex];
        const std::size_t previous = sampleInfo.sampleIndex > 0U ? sampleInfo.sampleIndex - 1U : sampleInfo.sampleIndex;
        const std::size_t next = std::min<std::size_t>(sampleInfo.sampleIndex + 1U, branchSamples.size() - 1U);
        const float slopeTransition =
            branchSamples.empty()
                ? 0.0F
                : std::abs(branchSamples[next].slope - branchSamples[previous].slope);
        const float widthTransition =
            branchSamples.empty()
                ? 0.0F
                : Clamp01(std::abs(branchSamples[next].channelWidth - branchSamples[previous].channelWidth) /
                          std::max(0.001F, sample.channelWidth));
        const float normalizedSpeed = Clamp01(sample.speed / 2.0F);
        const float fastNonFlat = normalizedSpeed * (1.0F - sample.flatness * 0.55F);
        sample.turbulence = Clamp01(
            normalizedSpeed * 0.32F +
            sample.curvature * 0.26F +
            sampleInfo.roughness * 0.22F +
            sample.confluence * 0.20F);
        sample.eddyPotential = Clamp01(
            sample.curvature * 0.42F +
            slopeTransition * 0.24F +
            widthTransition * 0.16F +
            sampleInfo.roughness * 0.12F +
            fastNonFlat * 0.18F);
        sample.ripplePotential = Clamp01(
            sample.curvature * 0.38F +
            sample.eddyPotential * 0.24F +
            sample.turbulence * 0.18F +
            sample.flatness * normalizedSpeed * 0.12F +
            sample.confluence * 0.08F);
    }

    for (auto& branchAnalysis : analysis.branches) {
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::channelWidth, false);
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::speed, false);
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::turbulence, true);
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::eddyPotential, true);
        SmoothAnalysisField(&branchAnalysis.samples, &WaterPathAnalysisSample::ripplePotential, true);
        for (auto& sample : branchAnalysis.samples) {
            SanitizeAnalysisSample(&sample);
        }
    }

    return analysis;
}

bool WaterPathAnalysisCacheCompatible(const WaterPathCache& cache) {
    if (!cache.analysis.has_value() ||
        cache.analysis->schemaVersion != 1U ||
        !std::isfinite(cache.analysis->analysisRadiusMeters) ||
        cache.analysis->branches.size() != cache.branches.size()) {
        return false;
    }
    for (std::size_t index = 0U; index < cache.branches.size(); ++index) {
        const auto& branch = cache.branches[index];
        const auto& analysisBranch = cache.analysis->branches[index];
        if (analysisBranch.branchId != branch.id ||
            analysisBranch.samples.size() != branch.rawAnchors.size()) {
            return false;
        }
        for (const auto& sample : analysisBranch.samples) {
            if (sample.branchId != branch.id) {
                return false;
            }
        }
    }
    return true;
}

void EnsureWaterPathAnalysis(WaterPathCache* cache) {
    if (cache == nullptr) {
        return;
    }
    if (!WaterPathAnalysisCacheCompatible(*cache)) {
        cache->analysis = BuildWaterPathAnalysis(*cache);
    }
}

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

WaterTrailGeometrySettings DefaultWaterTrailGeometrySettings() {
    return {};
}

WaterTrailGeometrySettings WaterTrailGeometryFromFlowStreamSettings(
    const WaterFlowStreamSettings& settings) {
    WaterTrailGeometrySettings geometry;
    geometry.trailLengthMeters = settings.streamLengthMeters;
    geometry.pointSpacingMeters = settings.streamPointSpacingMeters;
    geometry.widthMeters = settings.streamWidthMeters;
    geometry.worldLengthMeters = settings.streamWorldLengthMeters;
    return geometry;
}

WaterFlowStreamSettings ApplyWaterTrailGeometryToFlowStreamSettings(
    WaterFlowStreamSettings settings,
    const WaterTrailGeometrySettings& geometry) {
    settings.streamLengthMeters = geometry.trailLengthMeters;
    settings.streamPointSpacingMeters = geometry.pointSpacingMeters;
    settings.streamWidthMeters = geometry.widthMeters;
    settings.streamWorldLengthMeters = geometry.worldLengthMeters;
    return settings;
}

bool WaterFlowLaneRouteInputsEqual(
    const WaterFlowStreamSettings& left,
    const WaterFlowStreamSettings& right) {
    return left.enabled == right.enabled &&
           left.streamCountTotal == right.streamCountTotal &&
           left.laneCount == right.laneCount &&
           left.streamLengthMeters == right.streamLengthMeters &&
           left.streamPointSpacingMeters == right.streamPointSpacingMeters &&
           left.streamWidthMeters == right.streamWidthMeters &&
           left.streamWorldLengthMeters == right.streamWorldLengthMeters &&
           left.surfaceOffsetMeters == right.surfaceOffsetMeters &&
           left.pathAttraction == right.pathAttraction &&
           left.laneSpreadMeters == right.laneSpreadMeters &&
           left.laneCrossing == right.laneCrossing &&
           left.streamSmoothness == right.streamSmoothness &&
           left.streamLooseness == right.streamLooseness &&
           left.turbulence == right.turbulence &&
           left.seed == right.seed;
}

bool WaterFlowLaneSpeedOnlyEdit(
    const WaterFlowStreamSettings& before,
    const WaterFlowStreamSettings& after) {
    return WaterFlowLaneRouteInputsEqual(before, after) &&
           before.speedMetersPerSecond != after.speedMetersPerSecond;
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
    cache.analysis = BuildWaterPathAnalysis(cache);
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
    combined.analysis = BuildWaterPathAnalysis(combined);
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

float DirectionalDistanceToRegionEdge(
    const glm::vec3& point,
    const glm::vec3& direction,
    const std::vector<invisible_places::io::Float3>& polygon) {
    if (polygon.size() < 2U) {
        return 0.0F;
    }

    glm::vec2 rayDirection{direction.x, direction.y};
    if (glm::dot(rayDirection, rayDirection) <= 1.0e-8F) {
        return EffectPolygonEdgeDistanceXy(point, polygon);
    }
    rayDirection = glm::normalize(rayDirection);

    const glm::vec2 origin{point.x, point.y};
    float nearestHit = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const auto next = (index + 1U) % polygon.size();
        const glm::vec2 start{polygon[index].x, polygon[index].y};
        const glm::vec2 end{polygon[next].x, polygon[next].y};
        const glm::vec2 segment = end - start;
        const float denominator = Cross2d(rayDirection, segment);
        if (std::abs(denominator) <= 1.0e-8F) {
            continue;
        }

        const glm::vec2 delta = start - origin;
        const float rayDistance = Cross2d(delta, segment) / denominator;
        const float segmentPosition = Cross2d(delta, rayDirection) / denominator;
        if (rayDistance >= -1.0e-5F &&
            segmentPosition >= -1.0e-5F &&
            segmentPosition <= 1.0F + 1.0e-5F) {
            nearestHit = std::min(nearestHit, std::max(0.0F, rayDistance));
        }
    }

    if (nearestHit != std::numeric_limits<float>::max() && std::isfinite(nearestHit)) {
        return nearestHit;
    }

    float furthestProjection = -std::numeric_limits<float>::max();
    for (const auto& vertex : polygon) {
        furthestProjection = std::max(furthestProjection, glm::dot(glm::vec2{vertex.x, vertex.y}, rayDirection));
    }
    const float supportDistance = furthestProjection - glm::dot(origin, rayDirection);
    return std::isfinite(supportDistance) ? std::max(0.0F, supportDistance) : 0.0F;
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
    const float warp = std::max(0.0F, layer.warp);
    const float turbulence = std::max(0.0F, layer.turbulence);
    const float density01 = std::clamp(layer.density, 0.0F, 1.0F);
    const float densityCurve = std::sqrt(density01);
    const float cellSize = std::max(wavelength * 1.45F, std::lerp(0.34F, 0.115F, densityCurve));
    const glm::vec2 p = uv / cellSize;
    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    const float t = -layer.phase;
    const float rainDensity = std::clamp(0.10F + density01 * 0.78F, 0.06F, 0.92F);
    const float width = std::max(wavelength * (0.026F + turbulence * 0.024F), 0.0022F);
    const float closeSpacing = std::max(wavelength * (0.15F + turbulence * 0.055F), width * 3.0F);
    const float maxRadius = std::max(
        wavelength * (1.72F + turbulence * 0.38F + std::clamp(warp, 0.0F, 2.0F) * 0.13F),
        cellSize * 0.46F);
    float best = 0.0F;
    float blend = 0.0F;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float dropGate = RippleCellHash(cx, cy, seed, 71U);
            if (dropGate > rainDensity) {
                continue;
            }
            glm::vec2 center =
                (glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} +
                 RippleCellHash2(cx, cy, seed, 59U)) *
                cellSize;
            center += (RippleCellHash2(cx, cy, seed, 67U) - glm::vec2{0.5F}) *
                      cellSize *
                      std::clamp(warp, 0.0F, 2.0F) *
                      0.13F;
            const float distance = glm::length(uv - center);
            const float dropSeed = RippleCellHash(cx, cy, seed, 79U);
            const float life = Fract01(t * (0.16F + dropSeed * 0.07F + density01 * 0.025F) + dropSeed);
            const float radius = maxRadius * SmoothStep(0.0F, 1.0F, life);
            const float fade = std::pow(1.0F - life, 1.22F) * SmoothStep(0.025F, 0.13F, life);
            const float reachEnvelope = 1.0F - SmoothStep(maxRadius * 0.86F, maxRadius * 1.10F, distance);
            const float innerVisible = SmoothStep(closeSpacing * 1.1F, maxRadius * 0.82F, radius);
            const float outerVisible = SmoothStep(closeSpacing * 0.6F, maxRadius * 0.92F, radius);
            const float primary = RippleLine(distance - radius, width);
            const float inner =
                RippleLine(distance - std::max(0.0F, radius - closeSpacing * 0.92F), width * 0.72F) *
                innerVisible *
                0.54F;
            const float outer =
                RippleLine(distance - (radius + closeSpacing * 0.78F), width * 0.82F) *
                outerVisible *
                0.34F;
            const float wave = RippleWavePeak(
                (distance - radius) / std::max(closeSpacing, 0.002F) + dropSeed * 0.37F,
                2.4F);
            const float interference =
                wave *
                (1.0F - SmoothStep(width * 2.0F, closeSpacing * 3.2F, std::abs(distance - radius))) *
                0.22F;
            const float amplitude = (0.66F + dropSeed * 0.24F + (rainDensity - dropGate) * 0.10F) * fade * reachEnvelope;
            const float drop = (primary + inner + outer + interference) * amplitude;
            best = std::max(best, drop);
            blend += drop;
        }
    }
    return Clamp01(std::max(best, blend * (0.26F + density01 * 0.18F)));
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

float RuntimeRippleHash(float value) {
    return Fract01(std::sin(value) * 43758.5453123F);
}

float RuntimeRippleCellHash(int cellX, int cellY, float seed, float salt) {
    return RuntimeRippleHash(
        static_cast<float>(cellX) * 12.9898F +
        static_cast<float>(cellY) * 78.233F +
        seed * 37.719F +
        salt * 19.371F);
}

glm::vec2 RuntimeRippleCellHash2(int cellX, int cellY, float seed, float salt) {
    return {
        RuntimeRippleCellHash(cellX, cellY, seed, salt),
        RuntimeRippleCellHash(cellX, cellY, seed, salt + 17.0F),
    };
}

float RuntimeRippleBlockNoise(const glm::vec2& uv, float cellSize, float seed, float salt) {
    const float safeCellSize = std::max(0.001F, cellSize);
    const auto cellX = static_cast<int>(std::floor(uv.x / safeCellSize));
    const auto cellY = static_cast<int>(std::floor(uv.y / safeCellSize));
    return RuntimeRippleCellHash(cellX, cellY, seed, salt);
}

float RuntimeRippleSmoothBlockNoise(const glm::vec2& uv, float cellSize, float seed, float salt) {
    const float safeCellSize = std::max(0.001F, cellSize);
    const glm::vec2 p = uv / safeCellSize;
    const auto cellX = static_cast<int>(std::floor(p.x));
    const auto cellY = static_cast<int>(std::floor(p.y));
    const glm::vec2 f{Fract01(p.x), Fract01(p.y)};
    const glm::vec2 u = f * f * (glm::vec2{3.0F, 3.0F} - 2.0F * f);
    const float a = RuntimeRippleCellHash(cellX, cellY, seed, salt);
    const float b = RuntimeRippleCellHash(cellX + 1, cellY, seed, salt);
    const float c = RuntimeRippleCellHash(cellX, cellY + 1, seed, salt);
    const float d = RuntimeRippleCellHash(cellX + 1, cellY + 1, seed, salt);
    return std::lerp(std::lerp(a, b, u.x), std::lerp(c, d, u.x), u.y);
}

float RuntimeRippleCausticLaceValue(
    glm::vec2 uv,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float cellSize = std::max(0.005F, wavelength * 0.78F);
    const float t = -phase;
    const float density01 = std::clamp(density, 0.0F, 1.0F);
    const float turbulence01 = std::clamp(turbulence, 0.0F, 1.0F);
    glm::vec2 p = uv / cellSize;
    const float warpAmount = std::clamp(warp, 0.0F, 8.0F);
    p += glm::vec2{
             std::sin((p.y * 0.81F + seed * 1.37F + t * 0.22F) * 2.19F) +
                 0.5F * std::sin((p.y * 1.73F - seed * 0.61F - t * 0.15F) * 1.31F),
             std::cos((p.x * 0.88F + seed * 1.91F - t * 0.24F) * 2.41F) +
                 0.5F * std::sin((p.x * 1.57F + seed * 0.47F + t * 0.18F) * 1.67F)}
         * (0.08F + warpAmount * 0.18F);

    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    float nearest = std::numeric_limits<float>::max();
    float secondNearest = std::numeric_limits<float>::max();
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const glm::vec2 h = RuntimeRippleCellHash2(cx, cy, seed, 31.0F);
            const float angle = (h.x * 1.73F + h.y * 2.41F + t * (0.055F + h.x * 0.050F)) * kRippleTwoPi;
            const glm::vec2 wobble{
                std::cos(angle) * (0.10F + h.y * 0.11F),
                std::sin(angle * 1.13F + h.y * kRippleTwoPi) * (0.10F + h.y * 0.11F),
            };
            const glm::vec2 feature =
                glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} + h + wobble;
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
    const float lineWidth = std::clamp(0.010F + turbulence01 * 0.022F + density01 * 0.012F, 0.008F, 0.085F);
    const float ridge = 1.0F - SmoothStep(lineWidth, lineWidth * 3.7F, ridgeDistance);
    const float broadRidge = 1.0F - SmoothStep(lineWidth * 1.8F, lineWidth * 6.4F, ridgeDistance);
    const float filamentA = RippleWavePeak((p.x * 0.23F + p.y * 0.71F) + t * 0.045F + seed * 0.17F, 7.0F);
    const float filamentB = RippleWavePeak((p.x * -0.52F + p.y * 0.34F) - t * 0.038F + seed * 0.11F, 9.0F);
    const float filament = std::max(filamentA, filamentB);
    const float shimmer = 0.80F + 0.20F * RuntimeRippleSmoothBlockNoise(
                                           uv + glm::vec2{t * 0.021F, -t * 0.017F},
                                           cellSize * 0.33F,
                                           seed,
                                           43.0F);
    const float ridgeEnergy = ridge * 0.88F + broadRidge * ridge * 0.18F + filament * ridge * 0.18F;
    const float coverage = SmoothStep(0.30F - density01 * 0.16F, 0.94F, ridgeEnergy);
    const float activeEnvelope = SmoothStep(0.08F, 0.62F, ridge) * coverage;
    const float lace = std::pow(Clamp01(ridge), 1.65F) * (0.78F + filament * 0.22F);
    const float filamentLift = filament * std::pow(Clamp01(ridge), 1.15F) * (0.14F + turbulence01 * 0.10F);
    const float softGlow = broadRidge * ridge * (0.025F + density01 * 0.045F);
    return Clamp01((lace + filamentLift + softGlow) * activeEnvelope * shimmer);
}

float RuntimeRippleRainRingValue(
    const glm::vec2& uv,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float density01 = std::clamp(density, 0.0F, 1.0F);
    const float densityCurve = std::sqrt(density01);
    const float cellSize = std::max(wavelength * 1.45F, std::lerp(0.34F, 0.115F, densityCurve));
    const glm::vec2 p = uv / cellSize;
    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    const float t = -phase;
    const float rainDensity = std::clamp(0.10F + density01 * 0.78F, 0.06F, 0.92F);
    const float width = std::max(wavelength * (0.026F + turbulence * 0.024F), 0.0022F);
    const float closeSpacing = std::max(wavelength * (0.15F + turbulence * 0.055F), width * 3.0F);
    const float maxRadius = std::max(
        wavelength * (1.72F + turbulence * 0.38F + std::clamp(warp, 0.0F, 2.0F) * 0.13F),
        cellSize * 0.46F);
    float best = 0.0F;
    float blend = 0.0F;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float dropGate = RuntimeRippleCellHash(cx, cy, seed, 71.0F);
            if (dropGate > rainDensity) {
                continue;
            }
            glm::vec2 center =
                (glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} +
                RuntimeRippleCellHash2(cx, cy, seed, 59.0F)) *
                cellSize;
            center += (RuntimeRippleCellHash2(cx, cy, seed, 67.0F) - glm::vec2{0.5F}) *
                      cellSize *
                      std::clamp(warp, 0.0F, 2.0F) *
                      0.13F;
            const float distance = glm::length(uv - center);
            const float dropSeed = RuntimeRippleCellHash(cx, cy, seed, 79.0F);
            const float life = Fract01(t * (0.16F + dropSeed * 0.07F + density01 * 0.025F) + dropSeed);
            const float radius = maxRadius * SmoothStep(0.0F, 1.0F, life);
            const float fade = std::pow(1.0F - life, 1.22F) * SmoothStep(0.025F, 0.13F, life);
            const float reachEnvelope = 1.0F - SmoothStep(maxRadius * 0.86F, maxRadius * 1.10F, distance);
            const float innerVisible = SmoothStep(closeSpacing * 1.1F, maxRadius * 0.82F, radius);
            const float outerVisible = SmoothStep(closeSpacing * 0.6F, maxRadius * 0.92F, radius);
            const float primary = RippleLine(distance - radius, width);
            const float inner =
                RippleLine(distance - std::max(0.0F, radius - closeSpacing * 0.92F), width * 0.72F) *
                innerVisible *
                0.54F;
            const float outer =
                RippleLine(distance - (radius + closeSpacing * 0.78F), width * 0.82F) *
                outerVisible *
                0.34F;
            const float wave = RippleWavePeak(
                (distance - radius) / std::max(closeSpacing, 0.002F) + dropSeed * 0.37F,
                2.4F);
            const float interference =
                wave *
                (1.0F - SmoothStep(width * 2.0F, closeSpacing * 3.2F, std::abs(distance - radius))) *
                0.22F;
            const float amplitude = (0.66F + dropSeed * 0.24F + (rainDensity - dropGate) * 0.10F) * fade * reachEnvelope;
            const float drop = (primary + inner + outer + interference) * amplitude;
            best = std::max(best, drop);
            blend += drop;
        }
    }
    return Clamp01(std::max(best, blend * (0.26F + density01 * 0.18F)));
}

float RuntimeRippleTideBandsValue(
    const glm::vec2& uv,
    float shoreDistance,
    float edgeBlendWidth,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float travelDistance = std::max(wavelength, 0.015F);
    const float t = -phase;
    const float density01 = std::clamp(density, 0.0F, 1.0F);
    const float turbulence01 = std::clamp(turbulence, 0.0F, 1.0F);
    const float clampedWarp = std::clamp(warp, 0.0F, 2.0F);
    const float lateralScale = std::max(wavelength * 1.35F, 0.012F);
    const float frontWidth = std::max(wavelength * (0.046F + turbulence01 * 0.026F), 0.003F);
    const float trailLength = std::max(wavelength * (1.05F + turbulence01 * 0.68F), frontWidth * 7.0F);
    constexpr float incomingShare = 0.58F;
    constexpr float returnShare = 0.30F;
    const float waveRate = 0.070F + density01 * 0.045F;
    const float warpGuard = wavelength * (0.18F + clampedWarp * 0.16F + turbulence01 * 0.08F);
    const float finishOffset = std::max(edgeBlendWidth, edgeBlendWidth + warpGuard);
    float combined = 0.0F;

    for (int waveIndex = 0; waveIndex < 4; ++waveIndex) {
        const float slot = static_cast<float>(waveIndex);
        const float slotSeed = seed + slot * 53.17F;
        const float timingNoise = RuntimeRippleHash(slotSeed * 0.071F + 11.0F);
        const float speedNoise = std::lerp(0.76F, 1.26F, RuntimeRippleHash(slotSeed * 0.097F + 23.0F));
        const float waveGate = RuntimeRippleHash(slotSeed * 0.113F + 31.0F);
        if (waveGate > std::lerp(0.62F, 1.0F, density01)) {
            continue;
        }

        const float offset =
            slot * 0.235F +
            (timingNoise - 0.5F) * (0.12F + turbulence01 * 0.10F) +
            RuntimeRippleSmoothBlockNoise(glm::vec2{t * 0.018F, slotSeed}, 0.23F, seed, 181.0F) * 0.10F;
        const float cycle = Fract01(t * waveRate * speedNoise + offset);
        constexpr float activeEnd = incomingShare + returnShare;
        if (cycle >= activeEnd) {
            continue;
        }

        const float scallopNoise = RuntimeRippleSmoothBlockNoise(
            glm::vec2{uv.y + slot * wavelength * 0.37F, seed * 0.13F + slot * 0.41F},
            std::max(wavelength * 0.48F, 0.008F),
            seed,
            151.0F + slot * 19.0F);
        const float frontWarp =
            (std::sin((uv.y / lateralScale) + seed * 1.17F + slot * 1.91F) * 0.62F +
             std::sin((uv.y / std::max(wavelength * 0.58F, 0.006F)) - seed * 0.73F + slot * 2.37F) * 0.28F +
             (scallopNoise - 0.5F) * 1.15F) *
            wavelength * (0.08F + clampedWarp * 0.10F + turbulence01 * 0.06F);
        const float x = finishOffset - std::max(0.0F, shoreDistance) - frontWarp;
        const float waveTravel =
            travelDistance * std::lerp(1.38F, 1.82F, RuntimeRippleHash(slotSeed * 0.061F + 47.0F));
        const float offshoreStart = -waveTravel * (0.72F + timingNoise * 0.18F);
        constexpr float shoreEnd = 0.0F;
        const float shoreBreakup = SmoothStep(0.24F, 0.92F, scallopNoise + turbulence01 * 0.22F);
        const float foamNoise = RuntimeRippleSmoothBlockNoise(
            glm::vec2{x * 0.37F + slot * 0.19F, uv.y + slot * 0.31F},
            std::max(wavelength * 0.35F, 0.006F),
            seed,
            203.0F + slot * 23.0F);
        const float breakup = SmoothStep(
            0.18F,
            0.96F,
            foamNoise + shoreBreakup * 0.45F + turbulence01 * 0.18F);
        const float shorewardMask =
            1.0F - SmoothStep(frontWidth * 0.45F, frontWidth * 2.20F, x - shoreEnd);

        if (cycle < incomingShare) {
            const float incomingProgress = SmoothStep(0.0F, 1.0F, cycle / incomingShare);
            const float frontPosition = std::lerp(offshoreStart, shoreEnd, incomingProgress);
            const float front = x - frontPosition;
            const float crest = RippleLine(front, frontWidth);
            const float trailDistance = std::max(0.0F, -front);
            const float trailingFoam =
                std::exp(-trailDistance / std::max(trailLength, 1.0e-4F)) *
                SmoothStep(frontWidth * 0.35F, frontWidth * 1.70F, trailDistance) *
                (1.0F - SmoothStep(trailLength * 1.08F, trailLength * 2.15F, trailDistance));
            const float crestFade =
                SmoothStep(0.02F, 0.18F, cycle) *
                (1.0F - SmoothStep(0.91F, 1.0F, incomingProgress));
            const float value =
                crest * (0.78F + shoreBreakup * 0.24F) * crestFade +
                trailingFoam * (0.46F + density01 * 0.34F) * breakup;
            combined = std::max(combined, value * shorewardMask);
        } else {
            const float returnProgress = SmoothStep(0.0F, 1.0F, (cycle - incomingShare) / returnShare);
            const float returnDistance = waveTravel * 0.50F;
            const float clearFront = shoreEnd - returnDistance * returnProgress;
            const float front = x - clearFront;
            const float remainingMask = 1.0F - SmoothStep(-frontWidth * 1.25F, frontWidth * 1.55F, front);
            const float trailDistance = std::max(0.0F, shoreEnd - x);
            const float heldFoam =
                std::exp(-trailDistance / std::max(trailLength, 1.0e-4F)) *
                SmoothStep(frontWidth * 0.35F, frontWidth * 1.70F, trailDistance) *
                (1.0F - SmoothStep(trailLength * 1.08F, trailLength * 2.15F, trailDistance));
            const float clearCrest = RippleLine(front, frontWidth * 1.15F);
            const float returnFade = 1.0F - SmoothStep(0.45F, 1.0F, returnProgress);
            const float value =
                heldFoam * remainingMask * (0.50F + density01 * 0.30F) * breakup * returnFade +
                clearCrest * (0.34F + shoreBreakup * 0.16F) * returnFade;
            combined = std::max(combined, value * shorewardMask);
        }
    }

    return Clamp01(combined);
}

float RuntimeRippleWetSheenValue(
    const glm::vec2& uv,
    const glm::vec3& normal,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float slope = Clamp01(1.0F - std::abs(normal.z));
    const float normalGrain = Clamp01(glm::length(glm::vec2{normal.x, normal.y}));
    const float t = -phase;
    const float safeWavelength = std::max(wavelength, 0.005F);
    const float clampedWarp = std::clamp(warp, 0.0F, 2.0F);
    const glm::vec2 normalBias =
        glm::vec2{normal.x, normal.y} * safeWavelength * (0.30F + clampedWarp * 0.38F);
    const glm::vec2 driftA{
        t * (0.034F + turbulence * 0.018F),
        -t * (0.021F + clampedWarp * 0.012F)};
    const glm::vec2 driftB{
        -t * (0.018F + clampedWarp * 0.016F),
        t * (0.029F + turbulence * 0.020F)};
    const float warpWave =
        std::sin((uv.y / std::max(safeWavelength * 1.15F, 0.010F)) + seed * 0.021F + t * 0.31F) *
        safeWavelength *
        (0.045F + clampedWarp * 0.075F + turbulence * 0.035F);
    const glm::vec2 warpedUv = uv + glm::vec2{warpWave, -warpWave * 0.58F} + normalBias;
    const float lowA = RuntimeRippleSmoothBlockNoise(
        warpedUv + driftA,
        std::max(safeWavelength * 1.70F, 0.018F),
        seed,
        163.0F);
    const float lowB = RuntimeRippleSmoothBlockNoise(
        uv * 0.73F + glm::vec2{normal.y, normal.x} * safeWavelength * (0.50F + clampedWarp * 0.35F) + driftB,
        std::max(safeWavelength * 2.45F, 0.024F),
        seed,
        173.0F);
    const float fine = RuntimeRippleSmoothBlockNoise(
        warpedUv + glm::vec2{-t * 0.046F, t * 0.039F},
        std::max(safeWavelength * (0.30F - turbulence * 0.10F), 0.005F),
        seed,
        167.0F);
    const float micro = RuntimeRippleBlockNoise(
        warpedUv + glm::vec2{normal.y, normal.x} * safeWavelength * 0.22F + glm::vec2{t * 0.062F, -t * 0.047F},
        std::max(safeWavelength * (0.105F - turbulence * 0.030F), 0.0025F),
        seed,
        181.0F);
    const float patch = SmoothStep(
        0.40F - density * 0.22F - turbulence * 0.08F,
        0.90F,
        lowA * 0.56F + lowB * 0.36F + fine * 0.12F);
    const float normalLift = slope * (0.34F + clampedWarp * 0.20F) +
                             normalGrain * (0.08F + clampedWarp * 0.05F);
    const float patchGate = SmoothStep(0.14F, 0.70F, patch + lowB * 0.24F + fine * 0.10F);
    const float coverage = SmoothStep(
        0.14F - density * 0.20F,
        0.90F,
        patch * (0.78F + normalLift * 0.44F) + lowA * 0.18F + fine * 0.14F +
            normalLift * patchGate * (0.16F + clampedWarp * 0.06F));
    const float grain = SmoothStep(0.39F - turbulence * 0.20F, 0.95F, fine * 0.66F + micro * 0.34F + lowA * 0.12F) *
                        patchGate;
    const float shimmerWave =
        0.50F +
        0.50F * std::sin(
                     (uv.x + uv.y * 0.41F) / std::max(safeWavelength * 3.6F, 0.020F) +
                     t * (0.36F + turbulence * 0.22F) +
                     lowB * kRippleTwoPi);
    const float glint = grain * (0.44F + shimmerWave * (0.36F + turbulence * 0.24F));
    const float wetCycle = RippleWavePeak(
        t * (0.24F + turbulence * 0.14F) + lowA * 0.73F + lowB * 0.41F + normalGrain * 0.19F,
        1.35F);
    const float temporalGate = 0.62F + 0.38F * wetCycle;
    const float normalResponse = 0.82F + slope * (0.55F + clampedWarp * 0.12F) + normalGrain * 0.16F;
    const float sheen =
        (patch * (0.18F + slope * 0.42F + normalGrain * 0.12F) +
         patchGate * slope * clampedWarp * 0.08F +
         glint * (0.24F + turbulence * 0.48F)) *
        coverage *
        normalResponse;
    return Clamp01(sheen * temporalGate);
}

float RuntimeRippleDripTrailValue(
    const glm::vec2& uv,
    const glm::vec3& normal,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float density01 = std::clamp(density, 0.0F, 1.0F);
    const float densityCurve = std::sqrt(density01);
    const float cellSize = std::max(wavelength * 1.45F, std::lerp(0.34F, 0.115F, densityCurve));
    const glm::vec2 p = uv / cellSize;
    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    const float t = -phase;
    const float originDensity = std::clamp(0.10F + density01 * 0.78F, 0.06F, 0.92F);
    const float originSoftMargin = 0.12F + std::clamp(turbulence, 0.0F, 1.0F) * 0.14F;
    glm::vec2 flowDir{normal.x * normal.z, normal.y * normal.z};
    if (glm::dot(flowDir, flowDir) <= 1.0e-6F) {
        flowDir = {1.0F, 0.0F};
    } else {
        flowDir = glm::normalize(flowDir);
    }
    const glm::vec2 sideDir{-flowDir.y, flowDir.x};
    float best = 0.0F;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float originGate = RuntimeRippleCellHash(cx, cy, seed, 71.0F);
            if (originGate > originDensity + originSoftMargin) {
                continue;
            }
            const float originWeight = 1.0F - SmoothStep(originDensity, originDensity + originSoftMargin, originGate);
            glm::vec2 origin =
                (glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} +
                 RuntimeRippleCellHash2(cx, cy, seed, 59.0F)) *
                cellSize;
            origin += (RuntimeRippleCellHash2(cx, cy, seed, 67.0F) - glm::vec2{0.5F}) *
                      cellSize *
                      std::clamp(warp, 0.0F, 2.0F) *
                      0.13F;
            const float trailSeed = RuntimeRippleCellHash(cx, cy, seed, 79.0F);
            const float life = Fract01(t * (0.12F + trailSeed * 0.055F + density01 * 0.018F) + trailSeed);
            const float travel = std::max(wavelength * (1.35F + std::clamp(warp, 0.0F, 8.0F) * 0.32F + turbulence * 0.22F), 0.040F);
            const float head = travel * SmoothStep(0.0F, 1.0F, life);
            const float ageFade = std::pow(1.0F - life, 0.74F) * SmoothStep(0.015F, 0.12F, life);
            const glm::vec2 local = uv - origin;
            const float along = glm::dot(local, flowDir);
            const float cross = glm::dot(local, sideDir);
            const float tailLength = travel * (0.28F + 0.72F * SmoothStep(0.05F, 0.80F, life));
            const float behindHead = head - along;
            const float tail = Clamp01(behindHead / std::max(tailLength, 1.0e-5F));
            const float inLength =
                SmoothStep(0.0F, wavelength * 0.08F, along) *
                SmoothStep(0.0F, wavelength * 0.10F, behindHead) *
                (1.0F - SmoothStep(tailLength * 0.72F, tailLength, behindHead));
            const float wiggle =
                std::sin((along / std::max(wavelength * 0.40F, 0.006F)) + trailSeed * kRippleTwoPi + t * (0.42F + turbulence * 0.22F)) *
                wavelength *
                (0.050F + std::clamp(warp, 0.0F, 8.0F) * 0.030F + turbulence * 0.050F) *
                SmoothStep(0.0F, travel * 0.52F, along);
            const float width = std::max(wavelength * (0.038F + turbulence * 0.052F), 0.0038F);
            const float activeWidth = width * (1.0F + tail * (0.65F + turbulence * 0.35F));
            const float lateral = RippleLine(cross - wiggle, activeWidth);
            const float wetWidth = activeWidth * (2.0F + turbulence * 1.1F) + wavelength * 0.010F;
            const float wetTrail = RippleLine(cross - wiggle * 0.55F, wetWidth) *
                                   inLength *
                                   (0.18F + tail * (0.28F + turbulence * 0.16F));
            const float wakeLength = travel * (0.16F + 0.74F * SmoothStep(0.03F, 0.92F, life));
            const float inWake = SmoothStep(-wavelength * 0.035F, wavelength * 0.055F, along) *
                                 (1.0F - SmoothStep(wakeLength * 0.82F, wakeLength, along));
            const float wakeWidth = std::max(wavelength * (0.052F + turbulence * 0.050F), 0.0075F) *
                                    (1.0F + tail * 0.70F);
            const float wake = RippleLine(cross - wiggle * 0.35F, wakeWidth) *
                               inWake *
                               (0.14F + turbulence * 0.16F) *
                               (0.35F + originWeight * 0.65F);
            const float taper = 1.0F - tail * 0.78F;
            const float headDrop = RippleLine(
                glm::length(glm::vec2{along - head, cross - wiggle}),
                width * (3.1F + turbulence * 1.4F));
            const float bead = RippleLine(glm::length(local), width * 2.7F) * (1.0F - SmoothStep(0.18F, 0.42F, life));
            const float trail =
                (lateral * inLength * taper + wetTrail + wake + headDrop * 0.42F + bead * 0.20F) *
                ageFade *
                (0.50F + originWeight * 0.36F + trailSeed * 0.18F);
            best = std::max(best, trail);
        }
    }
    return Clamp01(best);
}

float RuntimeRippleSaltMineralShimmerValue(
    const glm::vec2& regionUv,
    const glm::vec3& normal,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float t = -phase;
    const glm::vec2 normalXy{normal.x, normal.y};
    const float normalBias = Clamp01(glm::length(normalXy));
    const glm::vec2 normalFlow = normalBias > 1.0e-4F ? normalXy / normalBias : glm::vec2{0.37F, -0.21F};
    const glm::vec2 mineralAcross{-normalFlow.y, normalFlow.x};
    const float veinCell = std::max(wavelength * (1.05F + warp * 0.20F - density * 0.18F), 0.018F);
    const glm::vec2 lowWarp =
        glm::vec2{
            RuntimeRippleSmoothBlockNoise(
                regionUv + glm::vec2{t * 0.012F, -t * 0.009F},
                std::max(wavelength * 1.35F, 0.018F),
                seed,
                113.0F),
            RuntimeRippleSmoothBlockNoise(
                regionUv + glm::vec2{-t * 0.010F, t * 0.014F},
                std::max(wavelength * 1.35F, 0.018F),
                seed,
                127.0F)} -
        glm::vec2{0.5F};
    const glm::vec2 mineralUv =
        regionUv +
        normalFlow * wavelength * (0.50F + normalBias * 0.36F + warp * 0.20F) +
        mineralAcross * wavelength * normalBias * 0.22F +
        lowWarp * wavelength * (0.32F + warp * 0.30F + turbulence * 0.18F);

    const float coarse = RuntimeRippleSmoothBlockNoise(
        mineralUv + normalFlow * wavelength * 0.35F,
        std::max(wavelength * 1.80F, 0.020F),
        seed,
        131.0F);
    const float splitPhase =
        0.5F +
        0.5F * std::sin((t * (0.075F + turbulence * 0.045F) + coarse * 0.62F + seed * 0.013F) * kRippleTwoPi);
    const float splitBlend = SmoothStep(0.18F, 0.82F, splitPhase);
    const float reconnect = 1.0F - std::abs(splitBlend * 2.0F - 1.0F);

    const glm::vec2 pA = mineralUv / veinCell;
    const glm::vec2 pB =
        (mineralUv +
         mineralAcross * wavelength * (0.42F + normalBias * 0.26F) * (splitBlend * 2.0F - 1.0F) +
         normalFlow * wavelength * 0.16F * reconnect) /
        veinCell;
    const auto baseAX = static_cast<int>(std::floor(pA.x));
    const auto baseAY = static_cast<int>(std::floor(pA.y));
    const auto baseBX = static_cast<int>(std::floor(pB.x));
    const auto baseBY = static_cast<int>(std::floor(pB.y));
    float nearestA = std::numeric_limits<float>::max();
    float secondA = std::numeric_limits<float>::max();
    float nearestB = std::numeric_limits<float>::max();
    float secondB = std::numeric_limits<float>::max();
    float veinSeedA = 0.0F;
    float veinSeedB = 0.0F;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int ax = baseAX + dx;
            const int ay = baseAY + dy;
            const glm::vec2 hA = RuntimeRippleCellHash2(ax, ay, seed, 149.0F);
            const float angleA = (hA.x * 1.51F + hA.y * 2.07F + t * (0.025F + hA.y * 0.035F)) * kRippleTwoPi;
            const glm::vec2 featureA =
                glm::vec2{static_cast<float>(ax), static_cast<float>(ay)} +
                hA +
                glm::vec2{
                    std::cos(angleA),
                    std::sin(angleA * 1.17F + hA.x * kRippleTwoPi)} *
                    (0.07F + turbulence * 0.045F) +
                normalFlow * normalBias * (hA.x - 0.5F) * 0.22F;
            const float distanceA = glm::length(pA - featureA);
            if (distanceA < nearestA) {
                secondA = nearestA;
                nearestA = distanceA;
                veinSeedA = hA.x;
            } else if (distanceA < secondA) {
                secondA = distanceA;
            }

            const int bx = baseBX + dx;
            const int by = baseBY + dy;
            const glm::vec2 hB = RuntimeRippleCellHash2(bx, by, seed, 181.0F);
            const float angleB = (hB.x * 1.73F + hB.y * 1.39F - t * (0.030F + hB.x * 0.030F)) * kRippleTwoPi;
            const glm::vec2 featureB =
                glm::vec2{static_cast<float>(bx), static_cast<float>(by)} +
                hB +
                glm::vec2{
                    std::sin(angleB * 1.11F + hB.y * kRippleTwoPi),
                    std::cos(angleB)} *
                    (0.08F + turbulence * 0.050F) -
                mineralAcross * normalBias * (hB.y - 0.5F) * 0.20F;
            const float distanceB = glm::length(pB - featureB);
            if (distanceB < nearestB) {
                secondB = nearestB;
                nearestB = distanceB;
                veinSeedB = hB.y;
            } else if (distanceB < secondB) {
                secondB = distanceB;
            }
        }
    }

    const float veinWidth =
        std::clamp(0.024F + turbulence * 0.018F + density * 0.014F + normalBias * 0.010F, 0.014F, 0.085F);
    const float veinA = 1.0F - SmoothStep(veinWidth, veinWidth * 4.0F, secondA - nearestA);
    const float veinB = 1.0F - SmoothStep(veinWidth * 0.82F, veinWidth * 3.8F, secondB - nearestB);
    const float bridge = std::sqrt(std::max(0.0F, veinA * veinB)) * (0.20F + reconnect * 0.48F);
    const float veinNetwork = Clamp01(std::max(
        std::max(veinA * (0.90F - splitBlend * 0.18F), veinB * (0.54F + splitBlend * 0.46F)),
        bridge));
    const float alongVein =
        (mineralUv.x * (0.43F + normalFlow.x * 0.15F) + mineralUv.y * (0.31F + normalFlow.y * 0.15F)) /
        std::max(wavelength * 0.36F, 0.004F);
    const float shimmerWave = RippleWavePeak(
        alongVein + t * (0.22F + turbulence * 0.15F) + veinSeedA * 1.37F + veinSeedB * 0.71F,
        2.2F);
    const float crystal = RuntimeRippleSmoothBlockNoise(
        mineralUv + normalFlow * t * 0.010F + mineralAcross * t * 0.006F,
        std::max(wavelength * 0.18F, 0.004F),
        seed,
        193.0F);
    const float veinCoverage = SmoothStep(
        0.50F - density * 0.24F - normalBias * 0.14F,
        0.98F,
        veinNetwork + coarse * 0.20F);
    const float activityNoise = RuntimeRippleSmoothBlockNoise(
        mineralUv + normalFlow * t * 0.018F - mineralAcross * t * 0.013F,
        std::max(wavelength * 0.30F, 0.006F),
        seed,
        197.0F);
    const float veinActivity = SmoothStep(
        0.18F,
        0.92F,
        shimmerWave + activityNoise * (0.20F + turbulence * 0.12F) + reconnect * 0.08F);
    const float brightSplit = 0.36F + 0.64F * RippleWavePeak(splitPhase + crystal * 0.35F + t * 0.035F, 1.8F);
    const float fineGlint = SmoothStep(0.76F - turbulence * 0.16F - density * 0.10F, 1.0F, crystal + veinNetwork * 0.20F);
    const float softVein = veinNetwork * (0.035F + coarse * 0.025F) * (0.42F + veinActivity * 0.36F);
    const float brightVein =
        veinNetwork *
        veinActivity *
        (0.16F + shimmerWave * 0.46F + fineGlint * 0.20F) *
        brightSplit;
    return Clamp01(
        veinCoverage *
        (softVein + brightVein) *
        (0.72F + normalBias * 0.38F));
}

float RuntimeRippleDropletValue(
    const glm::vec2& uv,
    const glm::vec3& normal,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float safeWavelength = std::max(wavelength, 0.005F);
    const float cellSize = std::max(safeWavelength * 1.45F, 0.018F);
    const glm::vec2 p = uv / cellSize;
    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    const float t = -phase;
    const float normalBias = Clamp01(glm::length(glm::vec2{normal.x, normal.y}));
    const float geometryBias = 0.64F + normalBias * (0.24F + std::clamp(warp, 0.0F, 2.0F) * 0.06F);
    float best = 0.0F;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cellX = baseX + dx;
            const int cellY = baseY + dy;
            const float sparseGate = RuntimeRippleCellHash(cellX, cellY, seed, 89.0F);
            const float keep = SmoothStep(
                0.94F - density * 0.70F - std::clamp(turbulence, 0.0F, 1.0F) * 0.16F,
                1.0F,
                sparseGate);
            const float clusterSeed = RuntimeRippleCellHash(cellX, cellY, seed, 91.0F);
            const glm::vec2 center =
                (glm::vec2{static_cast<float>(cellX), static_cast<float>(cellY)} +
                 RuntimeRippleCellHash2(cellX, cellY, seed, 83.0F)) *
                cellSize;
            const glm::vec2 clusterOffset =
                (RuntimeRippleCellHash2(cellX, cellY, seed, 97.0F) - glm::vec2{0.5F}) *
                cellSize *
                std::clamp(warp, 0.0F, 2.0F) *
                0.24F;
            const glm::vec2 anchor = center + clusterOffset;
            const float clusterRadius =
                std::max(safeWavelength * (0.17F + clusterSeed * 0.16F + turbulence * 0.08F), 0.0035F);
            const float distance = glm::length(uv - anchor);
            const float core = 1.0F - SmoothStep(0.0F, clusterRadius, distance);
            const glm::vec2 satelliteA =
                anchor + (RuntimeRippleCellHash2(cellX, cellY, seed, 101.0F) - glm::vec2{0.5F}) * clusterRadius * 2.45F;
            const glm::vec2 satelliteB =
                anchor + (RuntimeRippleCellHash2(cellX, cellY, seed, 103.0F) - glm::vec2{0.5F}) * clusterRadius * 3.10F;
            const float satellite =
                (1.0F - SmoothStep(
                            0.0F,
                            clusterRadius * (0.42F + turbulence * 0.12F),
                            glm::length(uv - satelliteA))) *
                    0.55F +
                (1.0F - SmoothStep(
                            0.0F,
                            clusterRadius * (0.30F + clusterSeed * 0.18F),
                            glm::length(uv - satelliteB))) *
                    0.34F;
            const float waveA = RippleWavePeak(
                t * (0.82F + clusterSeed * 0.52F) +
                    (anchor.x * 0.67F + anchor.y * 0.31F) / std::max(safeWavelength * 1.85F, 0.012F) +
                    sparseGate * 2.1F,
                2.4F);
            const float waveB = RippleWavePeak(
                t * (1.22F - clusterSeed * 0.32F) +
                    (anchor.x * -0.28F + anchor.y * 0.81F) / std::max(safeWavelength * 2.60F, 0.018F) +
                    clusterSeed * 2.7F,
                3.4F);
            const float twinkle = RippleWavePeak(t * (1.75F + clusterSeed * 0.65F) + sparseGate * 3.6F, 5.0F);
            const float pulse = 0.18F + 0.58F * (waveA * 0.62F + waveB * 0.38F) + 0.24F * twinkle;
            const float cluster = std::pow(Clamp01(core), 2.0F) + satellite;
            best = std::max(best, cluster * keep * pulse * geometryBias);
        }
    }
    return Clamp01(best);
}

float RuntimeRippleCurrentThreadsValue(
    const glm::vec2& uv,
    const glm::vec3& normal,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float t = -phase;
    const float normalBias = Clamp01(glm::length(glm::vec2{normal.x, normal.y}));
    const float cellXSize = std::max(wavelength * (2.10F + warp * 0.35F), 0.036F);
    const float cellYSize = std::max(wavelength * (1.05F + turbulence * 0.35F), 0.024F);
    glm::vec2 streamUv = uv;
    streamUv.x += normalBias * wavelength * (0.10F + warp * 0.06F);
    streamUv.y += std::sin(uv.x / std::max(wavelength * 1.25F, 0.010F) + seed * 1.17F + t * 0.13F) *
                  wavelength * (0.055F + warp * 0.060F + normalBias * 0.035F);
    streamUv.y += std::sin(uv.x / std::max(wavelength * 0.47F, 0.006F) - seed * 0.73F - t * 0.19F) *
                  wavelength * turbulence * 0.055F;
    const glm::vec2 p{streamUv.x / cellXSize, streamUv.y / cellYSize};
    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    const float originDensity = std::clamp(0.16F + density * 0.66F + normalBias * 0.16F, 0.10F, 0.92F);
    float best = 0.0F;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -2; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float originGate = RuntimeRippleCellHash(cx, cy, seed, 199.0F);
            if (originGate > originDensity) {
                continue;
            }
            glm::vec2 origin =
                (glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} +
                 RuntimeRippleCellHash2(cx, cy, seed, 211.0F)) *
                glm::vec2{cellXSize, cellYSize};
            const float pulseSeed = RuntimeRippleCellHash(cx, cy, seed, 223.0F);
            origin.y += (pulseSeed - 0.5F) * cellYSize * (0.22F + turbulence * 0.18F);
            const float life = Fract01(t * (0.095F + pulseSeed * 0.075F + normalBias * 0.030F) + pulseSeed);
            const float travelRange = cellXSize * (0.82F + warp * 0.10F + normalBias * 0.34F);
            const float head = life * travelRange;
            const float trailLength = std::max(wavelength * (1.15F + warp * 0.34F + normalBias * 0.44F), 0.035F);
            const glm::vec2 local = streamUv - origin;
            const float forward = local.x;
            const float tail = head - forward;
            const float inPulse =
                SmoothStep(0.0F, wavelength * 0.11F, forward) *
                SmoothStep(0.0F, wavelength * 0.08F, tail) *
                (1.0F - SmoothStep(trailLength * 0.78F, trailLength, tail));
            const float fan = Clamp01(forward / std::max(trailLength, 1.0e-4F));
            const float wiggle =
                std::sin(forward / std::max(wavelength * 0.42F, 0.006F) + pulseSeed * kRippleTwoPi + t * 0.37F) *
                wavelength * (0.045F + turbulence * 0.055F + warp * 0.022F);
            const float spread =
                wavelength * (0.026F + turbulence * 0.026F + normalBias * 0.020F) +
                fan * wavelength * (0.072F + warp * 0.048F + normalBias * 0.064F);
            const float lateral = local.y - wiggle;
            const float trunk = RippleLine(lateral, spread) * inPulse * (1.0F - fan * 0.42F);
            const float headDrop =
                RippleLine(glm::length(glm::vec2{(forward - head) * 0.72F, lateral}), spread * 2.75F) * 0.36F;
            const float branchSeed = RuntimeRippleCellHash(cx, cy, seed, 227.0F);
            const float branchGate = SmoothStep(0.54F - density * 0.30F, 1.0F, branchSeed + normalBias * 0.12F);
            const float splitWindow = SmoothStep(0.18F, 0.48F, fan) * (1.0F - SmoothStep(0.70F, 1.0F, fan));
            const float branchSlope = std::lerp(-0.72F, 0.72F, RuntimeRippleCellHash(cx, cy, seed, 229.0F));
            const float branchOffset = (fan - 0.20F) * branchSlope * wavelength * (0.72F + warp * 0.38F);
            const float branch =
                std::max(
                    RippleLine(lateral - branchOffset, spread * 0.58F),
                    RippleLine(lateral + branchOffset * 0.68F, spread * 0.48F)) *
                splitWindow *
                branchGate *
                inPulse;
            const float breakupNoise = RuntimeRippleSmoothBlockNoise(
                streamUv + glm::vec2{t * 0.018F + pulseSeed, -t * 0.011F},
                std::max(wavelength * (0.24F + turbulence * 0.16F), 0.006F),
                seed,
                233.0F);
            const float breakupPulse = RippleWavePeak(
                forward / std::max(wavelength * 0.76F, 0.008F) - t * (0.12F + pulseSeed * 0.05F),
                1.8F);
            const float breakup = SmoothStep(
                0.18F - turbulence * 0.10F,
                0.88F,
                breakupNoise + breakupPulse * (0.20F + turbulence * 0.16F));
            const float pulseCore = RippleLine(
                glm::length(glm::vec2{forward - head, lateral * 0.72F}),
                spread * (2.1F + turbulence * 0.7F));
            const float pulse = (trunk + headDrop + branch * 0.68F + pulseCore * (0.16F + density * 0.08F)) *
                                breakup *
                                inPulse *
                                (0.62F + originGate * 0.22F + normalBias * 0.22F);
            best = std::max(best, pulse);
        }
    }
    const float fallbackNoise = RuntimeRippleSmoothBlockNoise(
        streamUv + glm::vec2{t * 0.014F, -t * 0.009F},
        std::max(wavelength * 0.42F, 0.006F),
        seed,
        239.0F);
    const float fallbackPulse =
        RippleWavePeak(
            streamUv.x / std::max(wavelength * 1.9F, 0.012F) - t * 0.22F + fallbackNoise,
            2.1F) *
        SmoothStep(0.42F - density * 0.16F, 0.96F, fallbackNoise);
    const float softFallback =
        fallbackPulse *
        (0.025F + density * 0.020F) *
        (0.35F + 0.65F * RippleWavePeak(t * 0.41F + fallbackNoise * 1.7F, 1.6F));
    return Clamp01((best + softFallback) * (0.78F + normalBias * 0.30F));
}

float RuntimeRippleFoamSparkleValue(
    const glm::vec2& regionUv,
    float edge,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float t = -phase;
    const float density01 = std::clamp(density, 0.0F, 1.0F);
    const float turbulence01 = std::clamp(turbulence, 0.0F, 1.0F);
    const float driftAmount = std::clamp(warp, 0.0F, 2.0F);
    const glm::vec2 drift{
        t * (0.018F + driftAmount * 0.010F) * driftAmount,
        -t * (0.012F + turbulence01 * 0.008F) * driftAmount};
    const glm::vec2 lowWarp =
        glm::vec2{
            RuntimeRippleSmoothBlockNoise(
                regionUv + glm::vec2{t * 0.017F, -t * 0.013F} * driftAmount,
                std::max(wavelength * 1.85F, 0.024F),
                seed,
                109.0F),
            RuntimeRippleSmoothBlockNoise(
                regionUv + glm::vec2{-t * 0.014F, t * 0.019F} * driftAmount,
                std::max(wavelength * 1.85F, 0.024F),
                seed,
                113.0F)} -
        glm::vec2{0.5F};
    const glm::vec2 foamUv =
        regionUv + drift + lowWarp * wavelength * driftAmount * (0.28F + driftAmount * 0.24F + turbulence01 * 0.18F);
    const float patchCellSize = std::max(wavelength * (0.98F + density01 * 0.56F), 0.018F);
    const glm::vec2 p = foamUv / patchCellSize;
    const auto baseX = static_cast<int>(std::floor(p.x));
    const auto baseY = static_cast<int>(std::floor(p.y));
    float nearest = std::numeric_limits<float>::max();
    float secondNearest = std::numeric_limits<float>::max();
    float thirdNearest = std::numeric_limits<float>::max();
    float nearestSeed = 0.0F;
    float secondSeed = 0.0F;
    float nearestPresence = 0.0F;
    float secondPresence = 0.0F;
    float nearestLife = 0.0F;
    float secondLife = 0.0F;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const glm::vec2 h = RuntimeRippleCellHash2(cx, cy, seed, 131.0F);
            const float cycle = t * (0.026F + turbulence01 * 0.034F) + h.x * 2.17F + h.y * 0.83F;
            const float cycleIndex = std::floor(cycle);
            const float life = Fract01(cycle);
            const glm::vec2 cycleH = RuntimeRippleCellHash2(cx, cy, seed, 157.0F + cycleIndex * 29.0F);
            const float rise = SmoothStep(0.05F, 0.24F, life);
            const float fall = 1.0F - SmoothStep(0.58F + density01 * 0.20F, 0.98F, life);
            const float presence = rise * fall;
            const float siteMix = 0.48F + turbulence01 * 0.18F;
            const glm::vec2 site = h * (1.0F - siteMix) + cycleH * siteMix;
            const glm::vec2 cycleOffset =
                (cycleH - glm::vec2{0.5F}) * (0.13F + density01 * 0.08F + turbulence01 * 0.10F);
            const float angle =
                (h.x * 1.91F + h.y * 2.37F + cycleH.x * 1.11F + t * (0.020F + h.x * 0.025F) * driftAmount) *
                kRippleTwoPi;
            const glm::vec2 cellWobble{
                std::cos(angle) * driftAmount * (0.026F + turbulence01 * 0.018F),
                std::sin(angle * 1.07F + h.y * kRippleTwoPi) * driftAmount * (0.026F + turbulence01 * 0.018F),
            };
            const glm::vec2 center =
                glm::vec2{static_cast<float>(cx), static_cast<float>(cy)} + site + cycleOffset + cellWobble;
            const float distance =
                glm::length(p - center) +
                (1.0F - presence) * (0.36F + turbulence01 * 0.20F + (1.0F - density01) * 0.10F);
            const float cellSeed = RuntimeRippleCellHash(cx, cy, seed, 137.0F + cycleIndex * 13.0F);
            if (distance < nearest) {
                thirdNearest = secondNearest;
                secondNearest = nearest;
                secondSeed = nearestSeed;
                secondPresence = nearestPresence;
                secondLife = nearestLife;
                nearest = distance;
                nearestSeed = cellSeed;
                nearestPresence = presence;
                nearestLife = life;
            } else if (distance < secondNearest) {
                thirdNearest = secondNearest;
                secondNearest = distance;
                secondSeed = cellSeed;
                secondPresence = presence;
                secondLife = life;
            } else if (distance < thirdNearest) {
                thirdNearest = distance;
            }
        }
    }
    const float ridgeDistance = secondNearest - nearest;
    const float ridgeWidth = 0.050F + density01 * 0.060F + turbulence01 * 0.036F;
    const float ridgePresence = SmoothStep(0.04F, 0.62F, nearestPresence * secondPresence);
    const float ridgeAge = std::max(nearestLife, secondLife);
    const float cellRidge =
        (1.0F - SmoothStep(ridgeWidth, ridgeWidth * (3.8F + turbulence01 * 1.2F), ridgeDistance)) *
        ridgePresence;
    const float junction =
        (1.0F - SmoothStep(ridgeWidth * 2.0F, ridgeWidth * 7.0F, thirdNearest - nearest)) *
        ridgePresence;
    const float foamNoise = RuntimeRippleSmoothBlockNoise(
        foamUv + glm::vec2{t * 0.026F, -t * 0.018F} * driftAmount,
        std::max(wavelength * (0.36F + turbulence01 * 0.16F), 0.007F),
        seed,
        149.0F);
    const float fineA = RuntimeRippleSmoothBlockNoise(
        foamUv + glm::vec2{foamNoise, -foamNoise} * wavelength * 0.31F + glm::vec2{t * 0.041F, t * 0.027F} * driftAmount,
        std::max(wavelength * (0.13F + turbulence01 * 0.06F), 0.0035F),
        seed,
        107.0F);
    const float fineB = RuntimeRippleSmoothBlockNoise(
        foamUv * 1.37F + glm::vec2{-foamNoise, foamNoise} * wavelength * 0.19F + glm::vec2{-t * 0.033F, t * 0.022F} * driftAmount,
        std::max(wavelength * (0.095F + turbulence01 * 0.045F), 0.003F),
        seed,
        151.0F);
    const float fineFleck = fineA * 0.62F + fineB * 0.38F;
    const float breakupPulse = RippleWavePeak(
        t * (0.10F + turbulence01 * 0.16F) + nearestSeed * 0.71F + secondSeed * 0.29F + foamNoise * 0.35F,
        1.7F);
    const float ageBreakup = SmoothStep(0.30F, 0.90F, ridgeAge);
    const float breakup = SmoothStep(
        0.24F - density01 * 0.16F,
        0.92F,
        foamNoise + breakupPulse * (0.18F + turbulence01 * 0.16F) + cellRidge * 0.18F);
    const float chips = SmoothStep(
        0.74F - density01 * 0.12F - turbulence01 * 0.18F,
        1.0F,
        fineFleck + breakupPulse * 0.18F + ageBreakup * (0.28F + turbulence01 * 0.22F));
    const float brokenRidge = cellRidge * breakup * (1.0F - chips * (0.22F + turbulence01 * 0.32F));
    const float sparkle = SmoothStep(
        0.62F - density01 * 0.22F - turbulence01 * 0.14F,
        1.0F,
        fineFleck + brokenRidge * 0.25F + junction * 0.18F);
    const float pulse =
        0.76F +
        0.24F * RippleWavePeak(
                    nearestSeed + secondSeed * 0.37F + fineFleck + t * (0.17F + turbulence01 * 0.18F),
                    2.4F);
    const float edgeFade = SmoothStep(0.05F, 0.34F, edge);
    const float foam = brokenRidge * (0.82F + junction * 0.42F) + sparkle * (0.14F + turbulence01 * 0.08F);
    return Clamp01(edgeFade * foam * pulse);
}

float RuntimeRipplePatternValue(
    WaterRippleOverlayType overlayType,
    const glm::vec2& uv,
    const glm::vec2& regionUv,
    const glm::vec3& normal,
    float edge,
    float shoreDistance,
    float edgeBlendWidth,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float regionRadialDistance = glm::length(regionUv);
    switch (overlayType) {
        case WaterRippleOverlayType::CausticLace:
            return RuntimeRippleCausticLaceValue(uv, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::LinearRipples:
            return RippleWavePeak((uv.x / wavelength) + phase, 4.0F);
        case WaterRippleOverlayType::RadialRipples:
            return RippleWavePeak((regionRadialDistance / wavelength) + phase, 6.0F);
        case WaterRippleOverlayType::RainRings:
            return RuntimeRippleRainRingValue(regionUv, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::TideBands:
            return RuntimeRippleTideBandsValue(uv, shoreDistance, edgeBlendWidth, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::WetSheen:
            return RuntimeRippleWetSheenValue(uv, normal, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::CurrentThreads:
            return RuntimeRippleCurrentThreadsValue(uv, normal, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::DropletGlints:
            return RuntimeRippleDropletValue(regionUv, normal, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::DripTrails:
            return RuntimeRippleDripTrailValue(regionUv, normal, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::FoamSparkle:
            return RuntimeRippleFoamSparkleValue(regionUv, edge, wavelength, warp, turbulence, density, seed, phase);
        case WaterRippleOverlayType::SaltMineralShimmer:
            return RuntimeRippleSaltMineralShimmerValue(regionUv, normal, wavelength, warp, turbulence, density, seed, phase);
    }
    return 0.0F;
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
    float shoreDistance,
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

    const auto pattern = ActiveWaterRipplePatternSettings(layer);
    const glm::vec3 relative = position - regionCenter;
    const float scale = std::clamp(pattern.patternScale, 0.05F, 100.0F);
    const glm::vec2 uv{
        glm::dot(relative, tangent) * scale,
        glm::dot(relative, lateral) * scale,
    };
    const glm::vec2 regionUv{
        relative.x * scale,
        relative.y * scale,
    };
    const float scaledShoreDistance = std::max(0.0F, shoreDistance) * scale;
    const float scaledEdgeBlendWidth = layer.edgeBlendWidth * scale;
    const float wavelength = std::max(0.005F, pattern.wavelengthMeters);
    const float layerPhase = pattern.phase + RegionHash01(layer.id + layer.seed, 0U, RippleOverlayTypeSalt(layer.rippleOverlayType));
    const float radialDistance = glm::length(uv);
    const float regionRadialDistance = glm::length(regionUv);
    const float slope = Clamp01(1.0F - std::abs(normal.z));
    const std::uint32_t seed = layer.seed ^ (layer.id * 747796405U) ^ RippleOverlayTypeSalt(layer.rippleOverlayType);

    switch (layer.rippleOverlayType) {
        case WaterRippleOverlayType::CausticLace:
            result.value = RuntimeRippleCausticLaceValue(
                uv,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.confidence = 0.72F + result.value * 0.28F;
            break;
        case WaterRippleOverlayType::LinearRipples:
            result.value = RippleWavePeak((uv.x / wavelength) + layerPhase, 4.0F);
            result.confidence = 0.65F + result.value * 0.35F;
            break;
        case WaterRippleOverlayType::RadialRipples: {
            result.value = RippleWavePeak((regionRadialDistance / wavelength) + layerPhase, 6.0F);
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
            result.value = RuntimeRippleRainRingValue(
                regionUv,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.distance = regionRadialDistance;
            result.confidence = 0.58F + result.value * 0.42F;
            break;
        case WaterRippleOverlayType::TideBands: {
            result.value = RuntimeRippleTideBandsValue(
                uv,
                scaledShoreDistance,
                scaledEdgeBlendWidth,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.confidence = 0.70F + result.value * 0.30F;
            break;
        }
        case WaterRippleOverlayType::WetSheen: {
            result.value = RuntimeRippleWetSheenValue(
                uv,
                normal,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.confidence = 0.50F + result.value * 0.50F;
            break;
        }
        case WaterRippleOverlayType::CurrentThreads:
            result.value = RuntimeRippleCurrentThreadsValue(
                uv,
                normal,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.confidence = 0.55F + result.value * 0.45F;
            break;
        case WaterRippleOverlayType::DropletGlints:
            result.value = RuntimeRippleDropletValue(
                regionUv,
                normal,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.confidence = 0.45F + result.value * 0.55F;
            break;
        case WaterRippleOverlayType::DripTrails: {
            result.value = RuntimeRippleDripTrailValue(
                regionUv,
                normal,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.tangent = tangent;
            result.linearCoord = uv.x;
            result.angle = std::atan2(tangent.y, tangent.x);
            result.confidence = 0.42F + result.value * 0.58F;
            break;
        }
        case WaterRippleOverlayType::FoamSparkle:
            result.value = RuntimeRippleFoamSparkleValue(
                regionUv,
                edge,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
            result.confidence = 0.50F + result.value * 0.50F;
            break;
        case WaterRippleOverlayType::SaltMineralShimmer:
            result.value = RuntimeRippleSaltMineralShimmerValue(
                regionUv,
                normal,
                wavelength,
                std::max(0.0F, pattern.warp),
                std::max(0.0F, pattern.turbulence),
                std::clamp(pattern.density, 0.0F, 1.0F),
                static_cast<float>(seed),
                layerPhase);
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
    const float turbulence = std::clamp(pattern.turbulence, 0.0F, 4.0F);
    result.value = Clamp01(result.value + turbulenceNoise * turbulence * 0.08F);
    return result;
}

struct LaneAnalysisSample {
    bool available = false;
    float channelWidth = 0.0F;
    float speed = 1.0F;
    float turbulence = 0.0F;
    float eddyPotential = 0.0F;
    float ripplePotential = 0.0F;
    float curvature = 0.0F;
    float flatness = 0.0F;
    float confluence = 0.0F;
    float neighborDensity = 0.0F;
};

const WaterPathBranchAnalysis* FindPathAnalysisBranch(
    const WaterPathAnalysisCache* analysis,
    std::uint32_t branchId) {
    if (analysis == nullptr || analysis->branches.empty()) {
        return nullptr;
    }
    const auto branchIt = std::find_if(
        analysis->branches.begin(),
        analysis->branches.end(),
        [branchId](const WaterPathBranchAnalysis& branch) {
            return branch.branchId == branchId && !branch.samples.empty();
        });
    return branchIt == analysis->branches.end() ? nullptr : &*branchIt;
}

LaneAnalysisSample SampleLaneAnalysis(
    const WaterPathBranchAnalysis* branch,
    float pathDistance) {
    LaneAnalysisSample result;
    if (branch == nullptr || branch->samples.empty()) {
        return result;
    }

    const auto& samples = branch->samples;
    auto sampleAt = [](const WaterPathAnalysisSample& sample) {
        LaneAnalysisSample value;
        value.available = true;
        value.channelWidth = std::max(0.001F, FiniteOr(sample.channelWidth, 0.001F));
        value.speed = std::max(0.01F, FiniteOr(sample.speed, 1.0F));
        value.turbulence = ClampFinite01(sample.turbulence);
        value.eddyPotential = ClampFinite01(sample.eddyPotential);
        value.ripplePotential = ClampFinite01(sample.ripplePotential);
        value.curvature = ClampFinite01(sample.curvature);
        value.flatness = ClampFinite01(sample.flatness);
        value.confluence = ClampFinite01(sample.confluence);
        value.neighborDensity = ClampFinite01(sample.neighborDensity);
        return value;
    };

    if (samples.size() == 1U || pathDistance <= samples.front().pathDistance) {
        return sampleAt(samples.front());
    }
    if (pathDistance >= samples.back().pathDistance) {
        return sampleAt(samples.back());
    }

    const auto rightIt = std::lower_bound(
        samples.begin(),
        samples.end(),
        pathDistance,
        [](const WaterPathAnalysisSample& sample, float distance) {
            return sample.pathDistance < distance;
        });
    if (rightIt == samples.begin()) {
        return sampleAt(samples.front());
    }
    if (rightIt == samples.end()) {
        return sampleAt(samples.back());
    }
    const auto leftIt = rightIt - 1;
    const float span = std::max(1.0e-5F, rightIt->pathDistance - leftIt->pathDistance);
    const float t = Clamp01((pathDistance - leftIt->pathDistance) / span);
    const auto left = sampleAt(*leftIt);
    const auto right = sampleAt(*rightIt);
    result.available = true;
    result.channelWidth = left.channelWidth + (right.channelWidth - left.channelWidth) * t;
    result.speed = left.speed + (right.speed - left.speed) * t;
    result.turbulence = left.turbulence + (right.turbulence - left.turbulence) * t;
    result.eddyPotential = left.eddyPotential + (right.eddyPotential - left.eddyPotential) * t;
    result.ripplePotential = left.ripplePotential + (right.ripplePotential - left.ripplePotential) * t;
    result.curvature = left.curvature + (right.curvature - left.curvature) * t;
    result.flatness = left.flatness + (right.flatness - left.flatness) * t;
    result.confluence = left.confluence + (right.confluence - left.confluence) * t;
    result.neighborDensity = left.neighborDensity + (right.neighborDensity - left.neighborDensity) * t;
    return result;
}

float LaneAnalysisGuideInfluence(
    float turbulence,
    float laneCrossing,
    float pathAttraction,
    float streamLooseness) {
    const float attractionRelief = 1.0F - std::clamp(pathAttraction, 0.0F, 1.0F);
    return std::clamp(
        std::clamp(laneCrossing, 0.0F, 1.0F) * 0.38F +
            std::clamp(streamLooseness, 0.0F, 1.0F) * 0.30F +
            attractionRelief * 0.24F +
            std::clamp(turbulence, 0.0F, 1.0F) * 0.18F,
        0.0F,
        1.0F);
}

float LaneAnalysisSpan(
    const LaneAnalysisSample& analysis,
    float fallbackSpan,
    float streamWidthMeters,
    float analysisInfluence) {
    if (!analysis.available) {
        return fallbackSpan;
    }
    const float minimumSpan = std::max(streamWidthMeters * 2.0F, 0.001F);
    const float channelWidth = std::max(minimumSpan, analysis.channelWidth);
    if (fallbackSpan <= 1.0e-6F) {
        return channelWidth;
    }
    const float authoredSpan = std::max(minimumSpan, fallbackSpan);
    const float guideInfluence = std::clamp(analysisInfluence, 0.0F, 1.0F);
    const float cappedChannelWidth =
        std::min(channelWidth, authoredSpan * (1.0F + guideInfluence * 12.0F));
    return std::max(
        minimumSpan,
        authoredSpan + (cappedChannelWidth - authoredSpan) * guideInfluence);
}

float LaneAnalysisPitch(
    const LaneAnalysisSample& analysis,
    float lanePitch,
    float localSpan,
    std::uint32_t laneCount) {
    if (!analysis.available || laneCount <= 1U || localSpan <= 1.0e-6F) {
        return lanePitch;
    }
    return std::max(lanePitch, localSpan / static_cast<float>(laneCount));
}

float LaneAnalysisWeight(
    const WaterPathBranchAnalysis* branch,
    float fallbackSpan,
    float analysisInfluence) {
    if (branch == nullptr || branch->samples.empty()) {
        return 1.0F;
    }
    float width = 0.0F;
    float confluence = 0.0F;
    for (const auto& sample : branch->samples) {
        width += std::max(0.001F, FiniteOr(sample.channelWidth, 0.001F));
        confluence += ClampFinite01(sample.confluence);
    }
    const float count = static_cast<float>(branch->samples.size());
    const float averageWidth = width / count;
    const float averageConfluence = confluence / count;
    const float spanReference = std::max(0.001F, fallbackSpan);
    const float analysisWeight =
        std::clamp(
            0.72F + Clamp01(averageWidth / spanReference) * 0.36F + averageConfluence * 0.32F,
            0.55F,
            1.65F);
    return 1.0F + (analysisWeight - 1.0F) * std::clamp(analysisInfluence, 0.0F, 1.0F);
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
    std::uint32_t requestedLaneCount,
    float turbulence,
    float laneCrossing,
    float pathAttraction,
    float streamSmoothness,
    float streamLooseness,
    float speedMetersPerSecond,
    std::uint32_t seed,
    float featureType,
    const WaterPathAnalysisCache* analysisCache) {
    WaterStreamOverlay overlay;
    if (paths.empty() || requestedStreamCount == 0U) {
        return overlay;
    }

    const float safeStreamWidth = std::max(0.0005F, streamWidthMeters);
    const float laneSpan = std::max(0.0F, laneSpreadMeters);
    const float lanePitch = std::max(safeStreamWidth * 0.5F, 0.00025F);
    const float analysisGuideInfluence =
        LaneAnalysisGuideInfluence(turbulence, laneCrossing, pathAttraction, streamLooseness);

    std::vector<float> pathLengths;
    pathLengths.reserve(paths.size());
    std::vector<const WaterPathBranchAnalysis*> pathAnalyses;
    pathAnalyses.reserve(paths.size());
    std::vector<float> pathWeights;
    pathWeights.reserve(paths.size());
    float totalLength = 0.0F;
    float totalWeight = 0.0F;
    for (const auto& path : paths) {
        const float length = PathLengthMeters(path);
        pathLengths.push_back(length);
        const auto branchId =
            path.empty()
                ? 0U
                : static_cast<std::uint32_t>(std::max(0.0F, std::floor(path.front().flowId + 0.5F)));
        const auto* analysisBranch = FindPathAnalysisBranch(analysisCache, branchId);
        pathAnalyses.push_back(analysisBranch);
        const float pathWeight =
            std::max(0.0F, length) *
            LaneAnalysisWeight(analysisBranch, laneSpan, analysisGuideInfluence);
        pathWeights.push_back(pathWeight);
        totalLength += std::max(0.0F, length);
        totalWeight += pathWeight;
    }
    if (totalLength <= 1.0e-5F) {
        return overlay;
    }
    if (totalWeight <= 1.0e-5F) {
        totalWeight = totalLength;
        pathWeights = pathLengths;
    }

    const float safeLength = std::clamp(streamLengthMeters, 0.02F, 100.0F);
    const float safeSpacing = std::clamp(pointSpacingMeters, 0.001F, 5.0F);
    const std::uint32_t samplesPerStream = std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(std::ceil(safeLength / safeSpacing)) + 1U);
    const auto potentialLaneCount =
        requestedLaneCount > 0U
            ? std::max<std::uint32_t>(1U, requestedLaneCount)
            : static_cast<std::uint32_t>(std::max<float>(
                  1.0F,
                  std::ceil(laneSpan / lanePitch)));
    const float laneCrossingAmount = std::clamp(laneCrossing, 0.0F, 1.0F);
    const auto laneCenter = [](std::uint32_t laneIndex, std::uint32_t laneCount, float span) {
        if (laneCount <= 1U || span <= 1.0e-6F) {
            return 0.0F;
        }
        const auto clampedIndex = std::min(laneIndex, laneCount - 1U);
        return (((static_cast<float>(clampedIndex) + 0.5F) / static_cast<float>(laneCount)) - 0.5F) * span;
    };
    std::uint32_t streamId = 1U;

    for (std::size_t pathIndex = 0; pathIndex < paths.size(); ++pathIndex) {
        const auto& path = paths[pathIndex];
        const float pathLength = pathLengths[pathIndex];
        if (path.size() < 2U || pathLength <= 1.0e-5F) {
            continue;
        }
        const auto routeStartIndex = static_cast<std::uint32_t>(std::min<std::size_t>(
            overlay.samples.size(),
            static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
        const auto routePointCount = std::max<std::uint32_t>(
            2U,
            static_cast<std::uint32_t>(std::ceil(pathLength / safeSpacing)) + 1U);
        const auto* analysisBranch = pathIndex < pathAnalyses.size() ? pathAnalyses[pathIndex] : nullptr;
        const std::uint32_t streamsForPath = std::max<std::uint32_t>(
            1U,
            static_cast<std::uint32_t>(
                std::round(static_cast<float>(requestedStreamCount) * (pathWeights[pathIndex] / totalWeight))));
        const auto branchId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(path.front().flowId + 0.5F)));
        const auto sourceId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(path.front().emitterId + 0.5F)));

        for (std::uint32_t routeIndex = 0; routeIndex < routePointCount; ++routeIndex) {
            const float routeDistance = std::min(pathLength, static_cast<float>(routeIndex) * safeSpacing);
            WaterOverlayPoint anchor = InterpolatePathByArcLength(path, routeDistance);
            const glm::vec3 normal = SafeOverlayNormal(ToGlm(anchor.normal));
            glm::vec3 tangent = TangentAtPathDistance(path, routeDistance, safeSpacing * 2.0F);
            tangent -= normal * glm::dot(tangent, normal);
            if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
                tangent = TangentAtPathDistance(path, routeDistance, safeSpacing * 2.0F);
            }
            tangent = glm::normalize(tangent);
            const auto localAnalysis = SampleLaneAnalysis(analysisBranch, routeDistance);
            const float localSpan =
                LaneAnalysisSpan(localAnalysis, laneSpan, safeStreamWidth, analysisGuideInfluence);
            const float localPitch = LaneAnalysisPitch(localAnalysis, lanePitch, localSpan, potentialLaneCount);
            const float localSpeed = std::max(
                0.01F,
                localAnalysis.available
                    ? speedMetersPerSecond * std::clamp(localAnalysis.speed, 0.10F, 8.0F)
                    : speedMetersPerSecond);

            WaterStreamSample routeSample;
            routeSample.position = FromGlm(ToGlm(anchor.position) + normal * std::max(0.0F, surfaceOffsetMeters));
            routeSample.normal = FromGlm(normal);
            routeSample.tangent = FromGlm(tangent);
            routeSample.red = 0U;
            routeSample.green = 0U;
            routeSample.blue = 0U;
            routeSample.streamId = 0.0F;
            routeSample.sourceId = static_cast<float>(sourceId);
            routeSample.pathId = static_cast<float>(pathIndex + 1U);
            routeSample.branchId = static_cast<float>(branchId);
            routeSample.streamDistance = routeDistance;
            routeSample.streamLength = safeLength;
            routeSample.streamSpeed = localSpeed;
            routeSample.streamWidth = std::max(0.0005F, streamWidthMeters);
            routeSample.streamWorldLength = std::max(
                routeSample.streamWidth * 2.0F,
                std::max(streamWorldLengthMeters, safeSpacing * 2.5F));
            routeSample.streamConfidence = std::clamp(anchor.confidence, 0.0F, 1.0F);
            routeSample.wetness = std::clamp(0.35F + anchor.accumulation * 0.65F, 0.0F, 1.0F);
            routeSample.featureType = featureType;
            routeSample.streamRole = 0.0F;
            routeSample.routeStartIndex = static_cast<float>(routeStartIndex);
            routeSample.routePointCount = static_cast<float>(routePointCount);
            routeSample.routeLength = pathLength;
            routeSample.streamStartPhase =
                pathLength > 1.0e-5F ? std::clamp(routeDistance / pathLength, 0.0F, 1.0F) : 0.0F;
            routeSample.streamLaneIndex = 0.0F;
            routeSample.streamLaneCount = static_cast<float>(potentialLaneCount);
            routeSample.streamLanePitch = localPitch;
            routeSample.streamLaneSpan = localSpan;
            routeSample.streamLaneCrossing = laneCrossingAmount;
            routeSample.streamCrossSeed = RegionHash01(seed + branchId, routeIndex, 7029U);
            IncludeStreamSample(&overlay, routeSample);
        }

        for (std::uint32_t laneIndex = 0; laneIndex < streamsForPath && streamId <= requestedStreamCount; ++laneIndex, ++streamId) {
            const float streamSeed = RegionHash01(seed + branchId, streamId, 7001U);
            const float laneSeed = RegionHash01(seed + branchId, streamId, 7003U);
            const std::uint32_t centerLaneLow = (potentialLaneCount - 1U) / 2U;
            const std::uint32_t centerLaneHigh = potentialLaneCount / 2U;
            const bool analysedRoute = analysisBranch != nullptr;
            const std::uint32_t baseLaneIndex =
                analysedRoute
                    ? std::min<std::uint32_t>(
                          potentialLaneCount - 1U,
                          static_cast<std::uint32_t>(std::floor(laneSeed * static_cast<float>(potentialLaneCount))))
                    : (laneSeed < 0.5F ? centerLaneLow : centerLaneHigh);
            const float baseLaneCenter = laneCenter(baseLaneIndex, potentialLaneCount, laneSpan);
            const float baseLaneUnit = laneCenter(baseLaneIndex, potentialLaneCount, 1.0F);
            const float laneCellWidth =
                potentialLaneCount > 0U ? laneSpan / static_cast<float>(potentialLaneCount) : 0.0F;
            const float laneJitterAmplitude =
                laneSpan <= 1.0e-6F ? 0.0F : std::min(lanePitch, laneCellWidth) * 0.18F;
            const float laneJitter =
                (RegionHash01(seed + branchId, streamId, 7011U) - 0.5F) * 2.0F * laneJitterAmplitude;
            const float laneJitterUnit =
                (RegionHash01(seed + branchId, streamId, 7011U) - 0.5F) *
                (potentialLaneCount > 1U ? 0.36F / static_cast<float>(potentialLaneCount) : 0.0F);
            const float laneOffset = baseLaneCenter + laneJitter;
            const float streamCrossSeed = RegionHash01(seed + branchId, streamId, 7027U);
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
                const float pointAge = safeLength > 1.0e-5F ? localDistance / safeLength : 0.0F;
                const auto localAnalysis = SampleLaneAnalysis(analysisBranch, pathDistance);
                const float localSpan =
                    LaneAnalysisSpan(localAnalysis, laneSpan, safeStreamWidth, analysisGuideInfluence);
                const float localPitch = LaneAnalysisPitch(localAnalysis, lanePitch, localSpan, potentialLaneCount);
                const float localTurbulence =
                    localAnalysis.available
                        ? std::clamp(
                              turbulence + localAnalysis.turbulence * 0.70F * analysisGuideInfluence,
                              0.0F,
                              5.0F)
                        : std::max(0.0F, turbulence);
                const float motionLooseness =
                    std::clamp(streamLooseness +
                                   (localAnalysis.ripplePotential * 0.28F +
                                    localAnalysis.eddyPotential * 0.20F) *
                                       analysisGuideInfluence,
                               0.0F,
                               1.5F);
                const float attraction = std::clamp(
                    pathAttraction -
                        (localAnalysis.flatness * 0.34F + localAnalysis.confluence * 0.20F) *
                            analysisGuideInfluence,
                    0.12F,
                    1.0F);
                const float smoothness = std::clamp(streamSmoothness, 0.0F, 1.0F);
                const float dynamicBaseOffset =
                    localAnalysis.available
                        ? (baseLaneUnit + laneJitterUnit) * localSpan * (1.0F - attraction * 0.18F)
                        : laneOffset;
                const float wavePhase =
                    static_cast<float>(sampleIndex) *
                        (0.43F + localAnalysis.ripplePotential * 0.32F * analysisGuideInfluence) +
                    streamSeed * 6.28318530718F +
                    pathDistance * (1.7F + localAnalysis.curvature * 2.2F * analysisGuideInfluence);
                const float wobbleAmplitude =
                    localAnalysis.available
                        ? localSpan * (0.015F + motionLooseness * 0.035F) *
                              (0.25F + localTurbulence +
                               localAnalysis.ripplePotential * 0.85F * analysisGuideInfluence)
                        : std::min(lanePitch * 0.35F, safeStreamWidth * 0.20F) * localTurbulence;
                const float wobble = std::sin(wavePhase) * wobbleAmplitude * (1.0F - smoothness * 0.45F);
                const float curl =
                    localAnalysis.available
                        ? std::cos(
                              pathDistance * (2.1F + localAnalysis.eddyPotential * 4.0F) +
                              streamCrossSeed * 6.28318530718F) *
                              localSpan * 0.12F * localAnalysis.eddyPotential *
                              (0.35F + localTurbulence) * analysisGuideInfluence
                        : 0.0F;
                const float crossing =
                    localAnalysis.available
                        ? std::sin(
                              pointAge * 6.28318530718F +
                              streamCrossSeed * 6.28318530718F) *
                              localSpan * 0.10F * laneCrossingAmount *
                              std::max({localAnalysis.confluence, localAnalysis.ripplePotential, localTurbulence}) *
                              analysisGuideInfluence
                        : 0.0F;
                const float lateralLimit =
                    localAnalysis.available
                        ? localSpan * 0.5F *
                              (1.0F + laneCrossingAmount * 0.18F *
                                           std::max({localAnalysis.confluence, localAnalysis.ripplePotential, localTurbulence}) *
                                           analysisGuideInfluence)
                        : std::numeric_limits<float>::max();
                const float lateralOffset =
                    std::clamp(dynamicBaseOffset + wobble + curl + crossing, -lateralLimit, lateralLimit);
                position += lateral * lateralOffset;
                position += normal * std::max(0.0F, surfaceOffsetMeters);

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
                const float localSpeed =
                    localAnalysis.available
                        ? std::max(0.01F, speed * std::clamp(localAnalysis.speed, 0.10F, 8.0F))
                        : speed;
                const float localWidthFactor =
                    localAnalysis.available
                        ? std::clamp(
                              0.78F +
                                  Clamp01(localSpan / std::max(0.001F, laneSpan + safeStreamWidth)) * 0.28F +
                                  localTurbulence * 0.16F,
                              0.55F,
                              1.85F)
                        : 1.0F;
                sample.streamSpeed = localSpeed;
                sample.streamWidth =
                    std::max(0.0005F, streamWidthMeters * (0.80F + streamSeed * 0.42F) * localWidthFactor);
                sample.streamWorldLength = std::max(
                    sample.streamWidth * 2.0F,
                    std::max(streamWorldLengthMeters, safeSpacing * 2.5F));
                sample.streamConfidence = std::clamp(anchor.confidence * (0.72F + 0.28F * streamSeed), 0.0F, 1.0F);
                sample.wetness = std::clamp(0.35F + anchor.accumulation * 0.65F, 0.0F, 1.0F);
                sample.featureType = featureType;
                sample.streamRole = 1.0F;
                sample.routeStartIndex = static_cast<float>(routeStartIndex);
                sample.routePointCount = static_cast<float>(routePointCount);
                sample.routeLength = pathLength;
                sample.streamStartPhase =
                    pathLength > 1.0e-5F ? Fract01(startDistance / pathLength) : 0.0F;
                sample.streamLateralOffset = lateralOffset;
                sample.streamLaneIndex = static_cast<float>(baseLaneIndex);
                sample.streamLaneCount = static_cast<float>(potentialLaneCount);
                sample.streamLanePitch = localPitch;
                sample.streamLaneSpan = localSpan;
                sample.streamLaneCrossing = laneCrossingAmount;
                sample.streamCrossSeed = streamCrossSeed;
                IncludeStreamSample(&overlay, sample);
            }
        }
    }

    return overlay;
}

}  // namespace

std::array<WaterRippleOverlayType, 11> AllWaterRippleOverlayTypes() {
    return {
        WaterRippleOverlayType::CausticLace,
        WaterRippleOverlayType::LinearRipples,
        WaterRippleOverlayType::RadialRipples,
        WaterRippleOverlayType::RainRings,
        WaterRippleOverlayType::TideBands,
        WaterRippleOverlayType::WetSheen,
        WaterRippleOverlayType::CurrentThreads,
        WaterRippleOverlayType::DropletGlints,
        WaterRippleOverlayType::DripTrails,
        WaterRippleOverlayType::FoamSparkle,
        WaterRippleOverlayType::SaltMineralShimmer,
    };
}

std::string_view WaterRippleOverlayTypeNameForStorage(WaterRippleOverlayType type) {
    switch (type) {
        case WaterRippleOverlayType::LinearRipples:
            return "linear_ripples";
        case WaterRippleOverlayType::RadialRipples:
            return "radial_ripples";
        case WaterRippleOverlayType::RainRings:
            return "rain_rings";
        case WaterRippleOverlayType::TideBands:
            return "shoreline";
        case WaterRippleOverlayType::WetSheen:
            return "wet_sheen";
        case WaterRippleOverlayType::CurrentThreads:
            return "current_threads";
        case WaterRippleOverlayType::DropletGlints:
            return "droplet_glints";
        case WaterRippleOverlayType::DripTrails:
            return "drip_trails";
        case WaterRippleOverlayType::FoamSparkle:
            return "foam_sparkle";
        case WaterRippleOverlayType::SaltMineralShimmer:
            return "salt_mineral_shimmer";
        case WaterRippleOverlayType::CausticLace:
            return "caustic_lace";
    }
    return "caustic_lace";
}

std::optional<WaterRippleOverlayType> ParseWaterRippleOverlayTypeName(std::string_view value) {
    if (value == "tide_bands") {
        return WaterRippleOverlayType::TideBands;
    }
    for (const auto type : AllWaterRippleOverlayTypes()) {
        if (value == WaterRippleOverlayTypeNameForStorage(type)) {
            return type;
        }
    }
    return std::nullopt;
}

std::size_t WaterRippleOverlayTypeIndex(WaterRippleOverlayType type) {
    switch (type) {
        case WaterRippleOverlayType::CausticLace:
            return 0U;
        case WaterRippleOverlayType::LinearRipples:
            return 1U;
        case WaterRippleOverlayType::RadialRipples:
            return 2U;
        case WaterRippleOverlayType::RainRings:
            return 3U;
        case WaterRippleOverlayType::TideBands:
            return 4U;
        case WaterRippleOverlayType::WetSheen:
            return 5U;
        case WaterRippleOverlayType::CurrentThreads:
            return 6U;
        case WaterRippleOverlayType::DropletGlints:
            return 7U;
        case WaterRippleOverlayType::DripTrails:
            return 8U;
        case WaterRippleOverlayType::FoamSparkle:
            return 9U;
        case WaterRippleOverlayType::SaltMineralShimmer:
            return 10U;
    }
    return 0U;
}

WaterRipplePatternSettings DefaultWaterRipplePatternSettings(WaterRippleOverlayType type) {
    WaterRipplePatternSettings settings;
    settings.directionX = 1.0F;
    settings.directionY = 0.0F;
    settings.directionZ = 0.0F;
    switch (type) {
        case WaterRippleOverlayType::CausticLace:
            settings.patternScale = 1.35F;
            settings.wavelengthMeters = 0.10F;
            settings.speed = 0.45F;
            settings.warp = 0.90F;
            settings.turbulence = 0.30F;
            settings.density = 0.55F;
            break;
        case WaterRippleOverlayType::LinearRipples:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.22F;
            settings.speed = 0.70F;
            settings.warp = 0.0F;
            settings.turbulence = 0.05F;
            settings.density = 0.50F;
            break;
        case WaterRippleOverlayType::RadialRipples:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.18F;
            settings.speed = 0.65F;
            settings.warp = 0.10F;
            settings.turbulence = 0.05F;
            settings.density = 0.50F;
            break;
        case WaterRippleOverlayType::RainRings:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.14F;
            settings.speed = 0.85F;
            settings.warp = 0.12F;
            settings.turbulence = 0.25F;
            settings.density = 0.35F;
            break;
        case WaterRippleOverlayType::TideBands:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.55F;
            settings.speed = 0.12F;
            settings.warp = 0.95F;
            settings.turbulence = 0.35F;
            settings.density = 0.50F;
            break;
        case WaterRippleOverlayType::WetSheen:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.30F;
            settings.speed = 0.20F;
            settings.warp = 0.25F;
            settings.turbulence = 0.35F;
            settings.density = 0.45F;
            break;
        case WaterRippleOverlayType::CurrentThreads:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.12F;
            settings.speed = 0.20F;
            settings.warp = 0.80F;
            settings.turbulence = 0.55F;
            settings.density = 0.55F;
            break;
        case WaterRippleOverlayType::DropletGlints:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.10F;
            settings.speed = 1.10F;
            settings.warp = 0.15F;
            settings.turbulence = 0.40F;
            settings.density = 0.35F;
            break;
        case WaterRippleOverlayType::DripTrails:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.35F;
            settings.speed = 0.45F;
            settings.warp = 0.85F;
            settings.turbulence = 0.45F;
            settings.density = 0.42F;
            break;
        case WaterRippleOverlayType::FoamSparkle:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.09F;
            settings.speed = 0.65F;
            settings.warp = 0.25F;
            settings.turbulence = 0.55F;
            settings.density = 0.60F;
            break;
        case WaterRippleOverlayType::SaltMineralShimmer:
            settings.patternScale = 1.0F;
            settings.wavelengthMeters = 0.16F;
            settings.speed = 0.25F;
            settings.warp = 0.25F;
            settings.turbulence = 0.50F;
            settings.density = 0.35F;
            break;
    }
    return settings;
}

namespace {

constexpr WaterRipplePatternControlSpec kScaleSpec{
    WaterRipplePatternControl::PatternScale,
    "Pattern Scale",
    "Scales the procedural pattern in world space.",
    0.05F,
    8.0F,
    true};

constexpr WaterRipplePatternControlSpec kDirectionSpec{
    WaterRipplePatternControl::Direction,
    "Direction",
    "World-space direction used by directional patterns.",
    -1.0F,
    1.0F,
    false};

constexpr std::array<WaterRipplePatternControlSpec, 6> kCausticSpecs{{
    kScaleSpec,
    {WaterRipplePatternControl::WavelengthMeters, "Cell Size", "Spacing for the pseudo-caustic lace cells.", 0.01F, 1.5F, true},
    {WaterRipplePatternControl::Speed, "Drift Speed", "How quickly the caustic lace drifts and shimmers.", 0.0F, 4.0F, false},
    {WaterRipplePatternControl::Warp, "Lace Warp", "Distorts the caustic network into refracted curves.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Turbulence, "Lace Breakup", "Breaks and varies the caustic lines.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Density, "Line Density", "Controls how much of the lace network is active.", 0.0F, 1.0F, false},
}};

constexpr std::array<WaterRipplePatternControlSpec, 4> kLinearSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Wavelength", "Distance between parallel ripple crests.", 0.01F, 3.0F, true},
    {WaterRipplePatternControl::Speed, "Travel Speed", "How quickly crests travel along the direction.", 0.0F, 4.0F, false},
    {WaterRipplePatternControl::Turbulence, "Crest Breakup", "Adds subtle variation to the ripple crests.", 0.0F, 1.0F, false},
    kDirectionSpec,
}};

constexpr std::array<WaterRipplePatternControlSpec, 4> kRadialSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Ring Spacing", "Distance between expanding radial rings.", 0.01F, 3.0F, true},
    {WaterRipplePatternControl::Speed, "Expansion Speed", "Positive values expand rings outward from the region center.", 0.0F, 4.0F, false},
    {WaterRipplePatternControl::Warp, "Ring Warp", "Distorts ring edges away from perfect circles.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Turbulence, "Ring Breakup", "Adds broken variation to ring intensity.", 0.0F, 1.0F, false},
}};

constexpr std::array<WaterRipplePatternControlSpec, 6> kRainSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Ring Scale", "Controls raindrop ring radius and spacing.", 0.01F, 2.0F, true},
    {WaterRipplePatternControl::Speed, "Expansion Speed", "How quickly each raindrop ring expands.", 0.0F, 4.0F, false},
    {WaterRipplePatternControl::Density, "Rain Amount", "Controls how many raindrop origins are active.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Ring Jitter", "Offsets and distorts ring origins slightly.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Turbulence, "Ring Breakup", "Varies ring width and fade.", 0.0F, 1.0F, false},
    kScaleSpec,
}};

constexpr std::array<WaterRipplePatternControlSpec, 7> kTideSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Travel Distance", "How far the shoreline wash travels before receding.", 0.03F, 5.0F, true},
    {WaterRipplePatternControl::Speed, "Tide Speed", "Slow in/out tide speed.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Density, "Wave Crowd", "Controls how many staggered shoreline waves arrive before a break.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Shoreline Warp", "Bends the moving front like an uneven shore.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Turbulence, "Foam Breakup", "Adds irregular breakup along the front.", 0.0F, 1.0F, false},
    kDirectionSpec,
    kScaleSpec,
}};

constexpr std::array<WaterRipplePatternControlSpec, 5> kWetSheenSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Patch Scale", "Size of sheen patches across the surface.", 0.02F, 3.0F, true},
    {WaterRipplePatternControl::Speed, "Sheen Drift", "Subtle temporal movement of glossy patches.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Density, "Patch Coverage", "Controls how much surface receives sheen.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Turbulence, "Surface Grain", "Adds fine glints and grain to the sheen.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Normal Bias", "Increases normal-driven variation.", 0.0F, 2.0F, false},
}};

constexpr std::array<WaterRipplePatternControlSpec, 6> kCurrentSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Thread Spacing", "Spacing between current thread paths.", 0.01F, 1.5F, true},
    {WaterRipplePatternControl::Speed, "Thread Drift", "Slow movement of branching thread paths.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Density, "Branch Density", "Controls how many thread branches appear.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Thread Wander", "Makes threads branch and wander across the surface.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Turbulence, "Thread Flicker", "Controls branch breakup and fading.", 0.0F, 1.0F, false},
    kDirectionSpec,
}};

constexpr std::array<WaterRipplePatternControlSpec, 5> kDropletSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Cluster Size", "Size of glittering droplet clusters.", 0.01F, 1.5F, true},
    {WaterRipplePatternControl::Speed, "Sparkle Rate", "How quickly glitter clusters sparkle.", 0.0F, 4.0F, false},
    {WaterRipplePatternControl::Density, "Glint Density", "Controls how many clustered glints appear.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Turbulence, "Cluster Variation", "Varies cluster shapes and sparkle timing.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Surface Bias", "Biases glitter by normal and local surface variation.", 0.0F, 2.0F, false},
}};

constexpr std::array<WaterRipplePatternControlSpec, 6> kDripSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Trail Length", "Length scale of each dripping trail.", 0.02F, 3.0F, true},
    {WaterRipplePatternControl::Speed, "Travel Speed", "How quickly drip heads move along the surface.", 0.0F, 3.0F, false},
    {WaterRipplePatternControl::Density, "Origin Density", "Controls how many drip origins are active.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Trail Wiggle", "Makes trails wiggle away from their origins.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Turbulence, "Tail Breakup", "Breaks up the fading tail.", 0.0F, 1.0F, false},
    kDirectionSpec,
}};

constexpr std::array<WaterRipplePatternControlSpec, 5> kFoamSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Fleck Scale", "Size of foam sparkle flecks.", 0.01F, 1.0F, true},
    {WaterRipplePatternControl::Speed, "Flicker Speed", "How quickly foam flecks fade and reappear.", 0.0F, 4.0F, false},
    {WaterRipplePatternControl::Density, "Foam Density", "Controls foam fleck coverage near the region edge.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Turbulence, "Edge Breakup", "Breaks up the foam edge band.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Foam Drift", "Moves and warps foam flecks along the boundary.", 0.0F, 2.0F, false},
}};

constexpr std::array<WaterRipplePatternControlSpec, 5> kSaltSpecs{{
    {WaterRipplePatternControl::WavelengthMeters, "Grain Scale", "Scale of crystalline mineral grain.", 0.01F, 1.5F, true},
    {WaterRipplePatternControl::Speed, "Shimmer Drift", "How quickly crystalline shimmer moves.", 0.0F, 2.0F, false},
    {WaterRipplePatternControl::Density, "Patch Coverage", "Controls how many mineral shimmer patches appear.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Turbulence, "Crystal Breakup", "Adds high-frequency crystalline flecks.", 0.0F, 1.0F, false},
    {WaterRipplePatternControl::Warp, "Surface Bias", "Biases shimmer by surface normal and position.", 0.0F, 2.0F, false},
}};

bool RipplePatternSettingsConfigured(const WaterRipplePatternSettings& settings) {
    return std::isfinite(settings.patternScale) &&
           std::isfinite(settings.wavelengthMeters) &&
           settings.patternScale > 0.0F &&
           settings.wavelengthMeters > 0.0F;
}

WaterRipplePatternSettings TopLevelPatternSettings(const WaterEffectLayer& layer) {
    WaterRipplePatternSettings settings;
    settings.patternScale = std::max(0.001F, layer.patternScale);
    settings.wavelengthMeters = std::max(0.001F, layer.wavelengthMeters);
    settings.speed = std::max(0.0F, layer.speed);
    settings.warp = std::max(0.0F, layer.warp);
    settings.turbulence = std::max(0.0F, layer.turbulence);
    settings.density = std::clamp(layer.density, 0.0F, 1.0F);
    settings.phase = layer.phase;
    settings.directionX = layer.directionX;
    settings.directionY = layer.directionY;
    settings.directionZ = layer.directionZ;
    return settings;
}

}  // namespace

std::span<const WaterRipplePatternControlSpec> WaterRipplePatternControlSpecs(WaterRippleOverlayType type) {
    switch (type) {
        case WaterRippleOverlayType::CausticLace:
            return kCausticSpecs;
        case WaterRippleOverlayType::LinearRipples:
            return kLinearSpecs;
        case WaterRippleOverlayType::RadialRipples:
            return kRadialSpecs;
        case WaterRippleOverlayType::RainRings:
            return kRainSpecs;
        case WaterRippleOverlayType::TideBands:
            return kTideSpecs;
        case WaterRippleOverlayType::WetSheen:
            return kWetSheenSpecs;
        case WaterRippleOverlayType::CurrentThreads:
            return kCurrentSpecs;
        case WaterRippleOverlayType::DropletGlints:
            return kDropletSpecs;
        case WaterRippleOverlayType::DripTrails:
            return kDripSpecs;
        case WaterRippleOverlayType::FoamSparkle:
            return kFoamSpecs;
        case WaterRippleOverlayType::SaltMineralShimmer:
            return kSaltSpecs;
    }
    return kCausticSpecs;
}

WaterRipplePatternSettings ActiveWaterRipplePatternSettings(const WaterEffectLayer& layer) {
    const auto index = WaterRippleOverlayTypeIndex(layer.rippleOverlayType);
    if (index < layer.overlayPatternSettings.size() &&
        RipplePatternSettingsConfigured(layer.overlayPatternSettings[index])) {
        return layer.overlayPatternSettings[index];
    }
    return TopLevelPatternSettings(layer);
}

void StoreActiveWaterRipplePatternSettings(WaterEffectLayer* layer) {
    if (layer == nullptr) {
        return;
    }
    const auto index = WaterRippleOverlayTypeIndex(layer->rippleOverlayType);
    if (index < layer->overlayPatternSettings.size()) {
        layer->overlayPatternSettings[index] = TopLevelPatternSettings(*layer);
    }
}

void ApplyWaterRipplePatternSettings(WaterEffectLayer* layer, const WaterRipplePatternSettings& settings) {
    if (layer == nullptr) {
        return;
    }
    layer->patternScale = std::max(0.001F, settings.patternScale);
    layer->wavelengthMeters = std::max(0.001F, settings.wavelengthMeters);
    layer->speed = std::max(0.0F, settings.speed);
    layer->warp = std::max(0.0F, settings.warp);
    layer->turbulence = std::max(0.0F, settings.turbulence);
    layer->density = std::clamp(settings.density, 0.0F, 1.0F);
    layer->phase = settings.phase;
    layer->directionX = settings.directionX;
    layer->directionY = settings.directionY;
    layer->directionZ = settings.directionZ;
    const auto index = WaterRippleOverlayTypeIndex(layer->rippleOverlayType);
    if (index < layer->overlayPatternSettings.size()) {
        layer->overlayPatternSettings[index] = TopLevelPatternSettings(*layer);
    }
}

void ApplyActiveWaterRipplePatternSettings(WaterEffectLayer* layer) {
    if (layer == nullptr) {
        return;
    }
    const auto index = WaterRippleOverlayTypeIndex(layer->rippleOverlayType);
    WaterRipplePatternSettings settings = DefaultWaterRipplePatternSettings(layer->rippleOverlayType);
    if (index < layer->overlayPatternSettings.size() &&
        RipplePatternSettingsConfigured(layer->overlayPatternSettings[index])) {
        settings = layer->overlayPatternSettings[index];
    }
    ApplyWaterRipplePatternSettings(layer, settings);
}

void InitializeWaterRipplePatternSettings(WaterEffectLayer* layer) {
    if (layer == nullptr) {
        return;
    }
    for (const auto type : AllWaterRippleOverlayTypes()) {
        layer->overlayPatternSettings[WaterRippleOverlayTypeIndex(type)] =
            DefaultWaterRipplePatternSettings(type);
    }
    ApplyActiveWaterRipplePatternSettings(layer);
}

std::string_view WaterRippleOverlayTypeDescription(WaterRippleOverlayType type) {
    switch (type) {
        case WaterRippleOverlayType::CausticLace:
            return "Animated warped caustic ridge networks with thin moving lace lines.";
        case WaterRippleOverlayType::LinearRipples:
            return "Parallel traveling crests along the region direction.";
        case WaterRippleOverlayType::RadialRipples:
            return "Concentric expanding rings from the region center.";
        case WaterRippleOverlayType::RainRings:
            return "Sparse raindrop impacts that expand into fading circular rings.";
        case WaterRippleOverlayType::TideBands:
            return "Calm shoreline foam wash that moves in and out along the region direction.";
        case WaterRippleOverlayType::WetSheen:
            return "Normal-driven glossy shimmer with soft surface grain instead of wave bands.";
        case WaterRippleOverlayType::CurrentThreads:
            return "Narrow broken strands aligned to flow direction.";
        case WaterRippleOverlayType::DropletGlints:
            return "Sparse bright point glints.";
        case WaterRippleOverlayType::DripTrails:
            return "Sparse wiggling droplets and short trails moving away from random origins.";
        case WaterRippleOverlayType::FoamSparkle:
            return "Edge-biased flickering flecks.";
        case WaterRippleOverlayType::SaltMineralShimmer:
            return "Granular crystalline flecks with broad irregular mineral patches.";
    }
    return "Animated warped caustic ridge networks with thin moving lace lines.";
}

WaterRegionSelection BuildWaterRegionSelection(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterEffectLayer& layer,
    const WaterRegionSelectionOptions& options) {
    WaterRegionSelection selection;
    selection.layerId = layer.id;
    selection.featureType = layer.featureType;
    selection.targetLayerSourcePath = layer.targetLayerSourcePath;
    selection.targetLayerKey = layer.targetLayerSourcePath.generic_string();
    selection.boundary = BuildWaterRegionBoundary(layer.vertices);
    selection.hull = layer.hull.empty() ? BuildWaterRegionHull(selection.boundary) : layer.hull;
    if (selection.boundary.size() < 3U || cloud.positions.empty()) {
        return selection;
    }

    for (const auto& vertex : selection.boundary) {
        selection.bounds.Expand(vertex);
    }

    const float edgeBlendWidth = std::max(1.0e-5F, layer.edgeBlendWidth);
    const glm::vec3 rawDirection{layer.directionX, layer.directionY, layer.directionZ};
    const glm::vec3 layerDirection =
        glm::dot(rawDirection, rawDirection) > kNormalEpsilon
            ? glm::normalize(rawDirection)
            : glm::vec3{1.0F, 0.0F, 0.0F};

    selection.points.reserve(std::min<std::size_t>(cloud.positions.size(), 1'000'000U));
    std::size_t visitedCandidates = 0;
    const bool useCandidateIndices = !options.candidatePointIndices.empty();
    const std::size_t totalCandidates =
        useCandidateIndices ? options.candidatePointIndices.size() : cloud.positions.size();
    for (std::size_t candidateIndex = 0; candidateIndex < totalCandidates; ++candidateIndex) {
        if (options.stopToken != nullptr && options.stopToken->stop_requested()) {
            return selection;
        }
        ++visitedCandidates;
        if (options.progress && (visitedCandidates % 16384U == 0U || visitedCandidates == totalCandidates)) {
            options.progress(visitedCandidates, totalCandidates);
        }
        const std::size_t index = useCandidateIndices
                                      ? static_cast<std::size_t>(options.candidatePointIndices[candidateIndex])
                                      : candidateIndex;
        if (index >= cloud.positions.size()) {
            continue;
        }
        const auto& sourcePosition = cloud.positions[index];
        if (!BoundsContainsXy(selection.bounds, sourcePosition)) {
            continue;
        }
        if (options.visibleViewProjection != nullptr &&
            !PointVisibleInClip(*options.visibleViewProjection, sourcePosition)) {
            continue;
        }
        const glm::vec3 position = ToGlm(sourcePosition);
        if (!PointInPolygonXy(position, selection.boundary)) {
            continue;
        }

        glm::vec3 normal{0.0F, 0.0F, 1.0F};
        if (cloud.hasNormals && index < cloud.normals.size()) {
            normal = SafeOverlayNormal(ToGlm(cloud.normals[index]));
        }

        const float edgeDistance3d = EffectPolygonEdgeDistance3d(position, selection.boundary);
        const float edgeDistanceXy = EffectPolygonEdgeDistanceXy(position, selection.boundary);
        const float edgeDistance = std::isfinite(edgeDistance3d) && edgeDistance3d > 0.0F
                                       ? edgeDistance3d
                                       : edgeDistanceXy;
        const float edgeWeight = SmoothStep(0.0F, edgeBlendWidth, edgeDistance);

        glm::vec3 fieldVector = layerDirection - normal * glm::dot(layerDirection, normal);
        if (glm::dot(fieldVector, fieldVector) <= kNormalEpsilon) {
            fieldVector = WaterPathLateral(normal);
        } else {
            fieldVector = glm::normalize(fieldVector);
        }

        WaterRegionSelectedPoint selected;
        selected.pointIndex = static_cast<std::uint32_t>(
            std::min<std::size_t>(index, std::numeric_limits<std::uint32_t>::max()));
        selected.position = sourcePosition;
        selected.normal = FromGlm(normal);
        selected.edgeDistance = edgeDistance;
        selected.edgeWeight = std::clamp(edgeWeight, 0.0F, 1.0F);
        selected.fieldVector = FromGlm(fieldVector);
        selected.blendMode = layer.blendMode;
        selected.response = layer.response;
        selected.sourceLayerId = layer.id;
        selected.effectSpeed = layer.speed;
        if (options.previewOnly) {
            selection.points.push_back(std::move(selected));
            continue;
        }
        selected.scalarValues.reserve(cloud.scalarFields.size());
        for (std::size_t scalarSlot = 0; scalarSlot < cloud.scalarFields.size(); ++scalarSlot) {
            selected.scalarValues.push_back(ScalarValue(cloud, scalarSlot, index));
        }
        selected.fieldWetness = std::clamp(layer.regionStrength * selected.edgeWeight, 0.0F, 1.0F);
        selected.fieldConfidence = std::clamp(0.35F + selected.edgeWeight * 0.65F, 0.0F, 1.0F);
        selected.flowBlocked = layer.featureType == WaterEffectFeatureType::FieldNoFlowRegion;
        selected.bridgeAllowed = layer.featureType == WaterEffectFeatureType::FieldBridgeAllowedRegion;
        selected.bridgeBlocked = layer.featureType == WaterEffectFeatureType::FieldBridgeBlockedRegion;
        selection.points.push_back(std::move(selected));
    }

    return selection;
}

std::vector<WaterRegionSelection> BuildWaterRegionSelections(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers,
    WaterEffectFeatureType featureType,
    const WaterRegionSelectionOptions& options) {
    std::vector<WaterRegionSelection> selections;
    selections.reserve(layers.size());
    for (const auto& layer : layers) {
        if (layer.featureType != featureType ||
            (!layer.enabledInViewport && !layer.enabledInExport)) {
            continue;
        }
        auto selection = BuildWaterRegionSelection(cloud, layer, options);
        if (selection.Valid()) {
            selections.push_back(std::move(selection));
        }
    }
    return selections;
}

std::string WaterEffectLayersFingerprint(const std::vector<WaterEffectLayer>& layers) {
    std::ostringstream stream;
    stream << "water-effect-layers-v1|" << layers.size();
    for (const auto& layer : layers) {
        stream << '|'
               << layer.id << ','
               << static_cast<int>(layer.featureType) << ','
               << static_cast<int>(layer.rippleOverlayType) << ','
               << static_cast<int>(layer.blendMode) << ','
               << layer.targetLayerSourcePath.generic_string() << ','
               << layer.enabledInViewport << ','
               << layer.enabledInExport << ','
               << layer.blendPriority << ','
               << layer.edgeBlendWidth << ','
               << layer.regionStrength << ','
               << layer.patternScale << ','
               << layer.speed << ','
               << layer.wavelengthMeters << ','
               << layer.warp << ','
               << layer.turbulence << ','
               << layer.density << ','
               << layer.phase << ','
               << layer.directionX << ','
               << layer.directionY << ','
               << layer.directionZ << ','
               << layer.seed << ','
               << layer.maxAffectedPoints << ','
               << layer.response.intensity << ','
               << layer.response.emissionAdd << ','
               << layer.response.opacityAdd << ','
               << layer.response.opacityMultiply << ','
               << layer.response.pointSizeAdd << ','
               << layer.response.pointSizeMultiply << ','
               << layer.response.hueShift << ','
               << layer.response.colouriseRed << ','
               << layer.response.colouriseGreen << ','
               << layer.response.colouriseBlue << ','
               << layer.response.colouriseAmount << ','
               << layer.response.gaussianSharpnessBias << ','
               << layer.vertices.size();
        for (const auto type : AllWaterRippleOverlayTypes()) {
            const auto index = WaterRippleOverlayTypeIndex(type);
            WaterRipplePatternSettings settings =
                index < layer.overlayPatternSettings.size() && RipplePatternSettingsConfigured(layer.overlayPatternSettings[index])
                    ? layer.overlayPatternSettings[index]
                    : DefaultWaterRipplePatternSettings(type);
            if (type == layer.rippleOverlayType && !RipplePatternSettingsConfigured(layer.overlayPatternSettings[index])) {
                settings = ActiveWaterRipplePatternSettings(layer);
            }
            stream << ";pattern:" << WaterRippleOverlayTypeNameForStorage(type) << ','
                   << settings.patternScale << ','
                   << settings.wavelengthMeters << ','
                   << settings.speed << ','
                   << settings.warp << ','
                   << settings.turbulence << ','
                   << settings.density << ','
                   << settings.phase << ','
                   << settings.directionX << ','
                   << settings.directionY << ','
                   << settings.directionZ;
        }
        for (const auto& vertex : layer.vertices) {
            stream << ':' << vertex.x << ',' << vertex.y << ',' << vertex.z;
        }
    }
    return stream.str();
}

std::string WaterFieldSettingsFingerprint(const WaterFieldSettings& settings) {
    std::ostringstream stream;
    stream << "water-field-settings-v1|"
           << settings.enabled << '|'
           << static_cast<int>(settings.outputMode) << '|'
           << settings.corridorRadiusMeters << '|'
           << settings.fieldResolutionMeters << '|'
           << settings.projectionResolutionMeters << '|'
           << settings.guideWeight << '|'
           << settings.downhillWeight << '|'
           << settings.graphWeight << '|'
           << settings.lateralWeight << '|'
           << settings.fieldSmoothing << '|'
           << settings.wetnessSpread << '|'
           << settings.surfaceOffsetMeters << '|'
           << settings.surfaceConfidenceThreshold << '|'
           << settings.maxBridgeDistanceMeters << '|'
           << settings.bridgeAggression << '|'
           << settings.turbulence << '|'
           << settings.seed;
    return stream.str();
}

WaterStreamOverlay BuildAnimatedWaterTrailOverlay(
    const std::vector<WaterAnimatedTrailPath>& paths,
    const WaterAnimatedTrailBuildSettings& settings) {
    std::vector<std::vector<WaterOverlayPoint>> groupedPaths;
    groupedPaths.reserve(paths.size());
    for (const auto& path : paths) {
        if (path.anchors.size() >= 2U) {
            groupedPaths.push_back(path.anchors);
        }
    }
    return BuildStreamOverlayFromPaths(
        groupedPaths,
        settings.trailCountTotal,
        settings.trailLengthMeters,
        settings.trailPointSpacingMeters,
        settings.trailWidthMeters,
        settings.trailWorldLengthMeters,
        settings.surfaceOffsetMeters,
        settings.laneSpreadMeters,
        settings.laneCount,
        settings.turbulence,
        settings.laneCrossing,
        settings.pathAttraction,
        settings.streamSmoothness,
        settings.streamLooseness,
        settings.speedMetersPerSecond,
        settings.seed,
        settings.featureType,
        nullptr);
}

WaterStreamOverlay BuildFlowStreamOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowStreamSettings& settings) {
    return BuildFlowStreamOverlayFromPathAnchors(pathAnchors, settings, nullptr);
}

WaterStreamOverlay BuildFlowStreamOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowStreamSettings& settings,
    const WaterPathAnalysisCache* analysis) {
    if (!settings.enabled) {
        return {};
    }
    std::vector<WaterAnimatedTrailPath> paths;
    const auto groupedPaths = GroupAnchorPaths(pathAnchors);
    paths.reserve(groupedPaths.size());
    for (const auto& anchors : groupedPaths) {
        WaterAnimatedTrailPath path;
        path.motionMode = WaterAnimatedTrailMotionMode::Path;
        path.sourceId = anchors.empty()
                            ? 0U
                            : static_cast<std::uint32_t>(
                                  std::max(0.0F, std::floor(anchors.front().emitterId + 0.5F)));
        path.anchors = anchors;
        paths.push_back(std::move(path));
    }
    std::vector<std::vector<WaterOverlayPoint>> flowPaths;
    flowPaths.reserve(paths.size());
    for (const auto& path : paths) {
        if (path.anchors.size() >= 2U) {
            flowPaths.push_back(path.anchors);
        }
    }
    return BuildStreamOverlayFromPaths(
        flowPaths,
        settings.streamCountTotal,
        settings.streamLengthMeters,
        settings.streamPointSpacingMeters,
        settings.streamWidthMeters,
        settings.streamWorldLengthMeters,
        settings.surfaceOffsetMeters,
        settings.laneSpreadMeters,
        settings.laneCount,
        settings.turbulence,
        settings.laneCrossing,
        settings.pathAttraction,
        settings.streamSmoothness,
        settings.streamLooseness,
        settings.speedMetersPerSecond,
        settings.seed,
        0.0F,
        analysis);
}

WaterFieldCache BuildFieldCacheFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFieldSettings& settings) {
    WaterFieldCache cache;
    cache.settings = settings;
    cache.settingsFingerprint = WaterFieldSettingsFingerprint(settings);
    cache.regionFingerprint = "path-anchors";
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
    cache.settingsFingerprint = WaterFieldSettingsFingerprint(settings);
    cache.regionFingerprint = WaterEffectLayersFingerprint(layers);
    if (!settings.enabled || cloud.positions.empty()) {
        return cache;
    }

    auto surfaceSelections = BuildWaterRegionSelections(
        cloud,
        layers,
        WaterEffectFeatureType::FieldSurfaceMotion);
    const bool singleRegion = surfaceSelections.size() == 1U;

    struct FieldControlSelection {
        WaterEffectFeatureType type = WaterEffectFeatureType::FieldNoFlowRegion;
        std::unordered_set<std::uint32_t> pointIndices;
    };

    std::vector<FieldControlSelection> controlSelections;
    const WaterEffectFeatureType controlTypes[] = {
        WaterEffectFeatureType::FieldNoFlowRegion,
        WaterEffectFeatureType::FieldBridgeAllowedRegion,
        WaterEffectFeatureType::FieldBridgeBlockedRegion,
    };
    for (const auto type : controlTypes) {
        for (const auto& selection : BuildWaterRegionSelections(cloud, layers, type)) {
            FieldControlSelection control;
            control.type = type;
            control.pointIndices.reserve(selection.points.size());
            for (const auto& point : selection.points) {
                control.pointIndices.insert(point.pointIndex);
            }
            controlSelections.push_back(std::move(control));
        }
    }

    for (const auto& selection : surfaceSelections) {
        if (!selection.Valid()) {
            continue;
        }
        if (singleRegion && cache.regionBoundary.empty()) {
            cache.regionBoundary = selection.boundary;
        }

        const glm::vec3 regionCenter = RippleRegionCentroid(selection.boundary);
        for (const auto& selected : selection.points) {
            const glm::vec3 position = ToGlm(selected.position);
            const glm::vec3 normal = SafeOverlayNormal(ToGlm(selected.normal));

            glm::vec3 guide = ToGlm(selected.fieldVector);
            guide -= normal * glm::dot(guide, normal);
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
            node.position = selected.position;
            node.normal = selected.normal;
            node.vector = FromGlm(fieldVector);
            node.sourcePointIndex = selected.pointIndex;
            node.sourceLayerId = selected.sourceLayerId;
            node.blendMode = selected.blendMode;
            node.response = selected.response;
            node.effectSpeed = selected.effectSpeed;
            for (const auto& control : controlSelections) {
                if (!control.pointIndices.contains(node.sourcePointIndex)) {
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
            node.wetness = selected.fieldWetness;
            node.confidence = selected.fieldConfidence;
            node.surfaceConfidence = node.confidence;
            if (node.flowBlocked) {
                node.wetness = 0.0F;
                node.confidence = 0.0F;
                node.surfaceConfidence = 0.0F;
            }
            node.pathStation = glm::dot(position - regionCenter, guide);
            node.distanceToGuide = selected.edgeDistance;
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

void FilterVisibleStreamSamplesToRegionBoundary(
    WaterStreamOverlay* overlay,
    const std::vector<invisible_places::io::Float3>& boundary) {
    if (overlay == nullptr || overlay->samples.empty() || boundary.size() < 3U) {
        return;
    }

    WaterStreamOverlay filtered;
    filtered.fieldDiagnostics = overlay->fieldDiagnostics;
    filtered.samples.reserve(overlay->samples.size());
    for (const auto& sample : overlay->samples) {
        if (sample.streamRole >= 0.5F && !PointInPolygonXy(ToGlm(sample.position), boundary)) {
            continue;
        }
        filtered.bounds.Expand(sample.position);
        filtered.samples.push_back(sample);
    }
    filtered.fieldDiagnostics.emittedSampleCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        filtered.samples.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    *overlay = std::move(filtered);
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
    std::vector<WaterAnimatedTrailPath> animatedPaths;
    animatedPaths.reserve(paths.size());
    for (const auto& anchors : paths) {
        WaterAnimatedTrailPath animatedPath;
        animatedPath.motionMode = WaterAnimatedTrailMotionMode::VectorField;
        animatedPath.sourceId = 0U;
        animatedPath.anchors = anchors;
        animatedPaths.push_back(std::move(animatedPath));
    }
    auto overlay = BuildAnimatedWaterTrailOverlay(
        animatedPaths,
        {
            .trailCountTotal = settings.streamlineCount,
            .trailLengthMeters = settings.streamlineLengthMeters,
            .trailPointSpacingMeters = settings.stepLengthMeters,
            .trailWidthMeters = settings.streamlineWidthMeters,
            .trailWorldLengthMeters = settings.streamWorldLengthMeters,
            .surfaceOffsetMeters = fieldCache.settings.surfaceOffsetMeters,
            .laneSpreadMeters = std::max(settings.seedSpacingMeters, fieldCache.settings.corridorRadiusMeters * 0.30F),
            .turbulence = fieldCache.settings.turbulence,
            .laneCrossing = 0.22F,
            .speedMetersPerSecond = settings.speedMetersPerSecond,
            .seed = fieldCache.settings.seed,
            .featureType = 3.0F,
        });
    diagnostics.emittedSampleCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        overlay.samples.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    overlay.fieldDiagnostics = diagnostics;
    FilterVisibleStreamSamplesToRegionBoundary(&overlay, fieldCache.regionBoundary);
    return overlay;
}

WaterStreamOverlay BuildFieldStreamOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldStreamSettings& settings,
    const std::vector<WaterEmitter>& emitters) {
    if (!settings.enabled || fieldCache.nodes.size() < 2U) {
        return {};
    }

    struct IndexedNode {
        std::uint32_t index = 0;
        GridKey key{};
    };
    const float cellSize = std::max(
        std::max(0.003F, fieldCache.settings.fieldResolutionMeters),
        std::max(0.003F, settings.stepLengthMeters) * 2.0F);
    std::unordered_map<GridKey, std::vector<std::uint32_t>, GridKeyHash> grid;
    grid.reserve(fieldCache.nodes.size());
    std::vector<IndexedNode> indexedNodes;
    indexedNodes.reserve(fieldCache.nodes.size());
    for (std::size_t index = 0; index < fieldCache.nodes.size(); ++index) {
        const auto& node = fieldCache.nodes[index];
        if (!IsValidPoint(ToGlm(node.position))) {
            continue;
        }
        const auto nodeIndex = static_cast<std::uint32_t>(std::min<std::size_t>(
            index,
            static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
        const auto key = MakeGridKey(ToGlm(node.position), cellSize);
        grid[key].push_back(nodeIndex);
        indexedNodes.push_back({.index = nodeIndex, .key = key});
    }
    if (indexedNodes.size() < 2U) {
        return BuildFieldStreamOverlay(fieldCache, settings);
    }

    auto nearestNode = [&](const glm::vec3& position, float radiusMeters) -> std::optional<std::uint32_t> {
        const float safeRadius = std::max(radiusMeters, cellSize);
        const int cellRadius = std::clamp(
            static_cast<int>(std::ceil(safeRadius / cellSize)),
            1,
            8);
        const auto baseKey = MakeGridKey(position, cellSize);
        std::optional<std::uint32_t> bestIndex;
        float bestDistanceSquared = safeRadius * safeRadius;
        for (int dz = -cellRadius; dz <= cellRadius; ++dz) {
            for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
                for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
                    const GridKey key{baseKey.x + dx, baseKey.y + dy, baseKey.z + dz};
                    const auto bucketIt = grid.find(key);
                    if (bucketIt == grid.end()) {
                        continue;
                    }
                    for (const auto candidateIndex : bucketIt->second) {
                        if (candidateIndex >= fieldCache.nodes.size()) {
                            continue;
                        }
                        const glm::vec3 delta = ToGlm(fieldCache.nodes[candidateIndex].position) - position;
                        const float distanceSquared = glm::dot(delta, delta);
                        if (distanceSquared < bestDistanceSquared) {
                            bestDistanceSquared = distanceSquared;
                            bestIndex = candidateIndex;
                        }
                    }
                }
            }
        }
        return bestIndex;
    };

    auto nextNode = [&](std::uint32_t currentIndex) -> std::optional<std::uint32_t> {
        if (currentIndex >= fieldCache.nodes.size()) {
            return std::nullopt;
        }
        const auto& current = fieldCache.nodes[currentIndex];
        const glm::vec3 position = ToGlm(current.position);
        glm::vec3 direction = ToGlm(current.vector);
        const glm::vec3 normal = SafeOverlayNormal(ToGlm(current.normal));
        direction -= normal * glm::dot(direction, normal);
        if (glm::dot(direction, direction) <= kNormalEpsilon) {
            return std::nullopt;
        }
        direction = glm::normalize(direction);

        const float searchRadius = std::max(
            fieldCache.settings.maxBridgeDistanceMeters,
            std::max(settings.stepLengthMeters * 4.0F, fieldCache.settings.fieldResolutionMeters * 4.0F));
        const int cellRadius = std::clamp(
            static_cast<int>(std::ceil(searchRadius / cellSize)),
            1,
            8);
        const auto baseKey = MakeGridKey(position, cellSize);
        std::optional<std::uint32_t> bestIndex;
        float bestScore = -std::numeric_limits<float>::max();
        for (int dz = -cellRadius; dz <= cellRadius; ++dz) {
            for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
                for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
                    const GridKey key{baseKey.x + dx, baseKey.y + dy, baseKey.z + dz};
                    const auto bucketIt = grid.find(key);
                    if (bucketIt == grid.end()) {
                        continue;
                    }
                    for (const auto candidateIndex : bucketIt->second) {
                        if (candidateIndex == currentIndex || candidateIndex >= fieldCache.nodes.size()) {
                            continue;
                        }
                        const auto& candidate = fieldCache.nodes[candidateIndex];
                        const glm::vec3 delta = ToGlm(candidate.position) - position;
                        const float distance = glm::length(delta);
                        if (distance <= 1.0e-5F || distance > searchRadius) {
                            continue;
                        }
                        const float alignment = glm::dot(glm::normalize(delta), direction);
                        if (alignment < -0.15F) {
                            continue;
                        }
                        const float confidence = std::clamp(candidate.surfaceConfidence, 0.0F, 1.0F);
                        const float distancePenalty = distance / std::max(searchRadius, 1.0e-5F);
                        const float score = alignment * 2.0F + confidence * 0.35F - distancePenalty;
                        if (score > bestScore) {
                            bestScore = score;
                            bestIndex = candidateIndex;
                        }
                    }
                }
            }
        }
        return bestIndex;
    };

    std::vector<WaterFieldSourcePoint> sources;
    sources.reserve(emitters.size());
    for (const auto& emitter : emitters) {
        if (emitter.status == WaterEmitterStatus::Disabled) {
            continue;
        }
        const glm::vec3 emitterPosition = ToGlm(emitter.position);
        const float searchRadius = std::max(
            emitter.radius * 4.0F,
            std::max(fieldCache.settings.corridorRadiusMeters, fieldCache.settings.maxBridgeDistanceMeters) * 2.5F);
        if (!nearestNode(emitterPosition, searchRadius).has_value()) {
            continue;
        }
        sources.push_back({
            .position = emitter.position,
            .sourceId = emitter.id,
            .radiusMeters = std::max(0.001F, emitter.radius),
            .strength = std::max(0.0F, emitter.strength),
            .seed = emitter.id ^ fieldCache.settings.seed,
        });
    }
    if (sources.empty()) {
        return BuildFieldStreamOverlay(fieldCache, settings);
    }

    WaterFieldStreamDiagnostics diagnostics;
    diagnostics.inputNodeCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        fieldCache.nodes.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    const float bridgeStart = std::max(0.0F, fieldCache.settings.fieldResolutionMeters * 1.25F);
    const float maxBridgeDistance = std::max(
        fieldCache.settings.maxBridgeDistanceMeters,
        fieldCache.settings.fieldResolutionMeters * (1.25F + fieldCache.settings.bridgeAggression * 4.0F));
    const float confidenceThreshold = std::clamp(fieldCache.settings.surfaceConfidenceThreshold, 0.0F, 1.0F);
    const std::uint32_t requestedPathCount = std::clamp<std::uint32_t>(
        std::max<std::uint32_t>(1U, settings.streamlineCount / 4U),
        1U,
        96U);
    const std::uint32_t maxSteps = std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(
            std::ceil(std::max(0.02F, settings.streamlineLengthMeters) /
                      std::max(0.001F, settings.stepLengthMeters))) +
            1U);

    std::vector<WaterAnimatedTrailPath> animatedPaths;
    animatedPaths.reserve(requestedPathCount);
    for (std::uint32_t pathIndex = 0; pathIndex < requestedPathCount; ++pathIndex) {
        const auto& source = sources[pathIndex % sources.size()];
        const float seedA = RegionHash01(source.seed, pathIndex, 9101U);
        const float seedB = RegionHash01(source.seed, pathIndex, 9109U);
        const float angle = seedA * 6.28318530718F;
        const float distance = std::sqrt(seedB) *
                               std::max(settings.seedSpacingMeters, source.radiusMeters * 0.45F) *
                               std::max(0.20F, source.strength);
        const glm::vec3 perturbedSource =
            ToGlm(source.position) +
            glm::vec3{std::cos(angle) * distance, std::sin(angle) * distance, 0.0F};
        auto currentIndex = nearestNode(
            perturbedSource,
            std::max(source.radiusMeters * 4.0F, fieldCache.settings.corridorRadiusMeters * 2.0F));
        if (!currentIndex.has_value()) {
            currentIndex = nearestNode(ToGlm(source.position), source.radiusMeters * 4.0F);
        }
        if (!currentIndex.has_value()) {
            continue;
        }

        WaterAnimatedTrailPath path;
        path.motionMode = WaterAnimatedTrailMotionMode::VectorField;
        path.sourceId = source.sourceId;
        path.anchors.reserve(maxSteps);
        float travelled = 0.0F;
        std::optional<std::uint32_t> previousIndex;
        for (std::uint32_t step = 0; step < maxSteps && currentIndex.has_value(); ++step) {
            if (currentIndex.value() >= fieldCache.nodes.size()) {
                break;
            }
            const auto& node = fieldCache.nodes[currentIndex.value()];
            if (node.flowBlocked) {
                ++diagnostics.manualNoFlowBlockCount;
                break;
            }
            if (settings.fadeOnLowConfidence && node.surfaceConfidence < confidenceThreshold * 0.35F) {
                ++diagnostics.lowConfidenceTerminationCount;
                break;
            }

            WaterOverlayPoint point;
            point.position = node.position;
            point.normal = node.normal;
            point.flowId = static_cast<float>(pathIndex + 1U);
            point.emitterId = static_cast<float>(source.sourceId);
            point.pathDistance = travelled;
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
            path.anchors.push_back(point);

            auto candidateIndex = nextNode(currentIndex.value());
            if (!candidateIndex.has_value()) {
                break;
            }
            if (previousIndex.has_value() && candidateIndex.value() == previousIndex.value()) {
                break;
            }
            const auto& candidate = fieldCache.nodes[candidateIndex.value()];
            const float gapDistance = glm::length(ToGlm(candidate.position) - ToGlm(node.position));
            const bool manualBridgeAllowed = node.bridgeAllowed || candidate.bridgeAllowed;
            const bool manualBridgeBlocked = node.bridgeBlocked || candidate.bridgeBlocked;
            const float manualAllowedBridgeDistance = maxBridgeDistance * 4.0F;
            if (manualBridgeBlocked) {
                ++diagnostics.manualBridgeBlockedCount;
                ++diagnostics.rejectedGapCount;
                diagnostics.minRejectedGapMeters =
                    diagnostics.minRejectedGapMeters <= 0.0F
                        ? gapDistance
                        : std::min(diagnostics.minRejectedGapMeters, gapDistance);
                break;
            }
            if (gapDistance > maxBridgeDistance &&
                (!manualBridgeAllowed || gapDistance > manualAllowedBridgeDistance)) {
                ++diagnostics.rejectedGapCount;
                diagnostics.minRejectedGapMeters =
                    diagnostics.minRejectedGapMeters <= 0.0F
                        ? gapDistance
                        : std::min(diagnostics.minRejectedGapMeters, gapDistance);
                break;
            }
            if (gapDistance > bridgeStart) {
                ++diagnostics.acceptedBridgeCount;
                diagnostics.maxAcceptedBridgeMeters = std::max(diagnostics.maxAcceptedBridgeMeters, gapDistance);
                if (manualBridgeAllowed && gapDistance > maxBridgeDistance) {
                    ++diagnostics.manualBridgeAllowedCount;
                }
            }
            travelled += gapDistance;
            previousIndex = currentIndex;
            currentIndex = candidateIndex;
        }
        if (path.anchors.size() >= 2U) {
            animatedPaths.push_back(std::move(path));
        }
    }
    diagnostics.emittedPathCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        animatedPaths.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));

    auto overlay = BuildAnimatedWaterTrailOverlay(
        animatedPaths,
        {
            .trailCountTotal = settings.streamlineCount,
            .trailLengthMeters = settings.streamlineLengthMeters,
            .trailPointSpacingMeters = settings.stepLengthMeters,
            .trailWidthMeters = settings.streamlineWidthMeters,
            .trailWorldLengthMeters = settings.streamWorldLengthMeters,
            .surfaceOffsetMeters = fieldCache.settings.surfaceOffsetMeters,
            .laneSpreadMeters = std::max(settings.seedSpacingMeters, fieldCache.settings.corridorRadiusMeters * 0.30F),
            .turbulence = fieldCache.settings.turbulence,
            .laneCrossing = 0.22F,
            .speedMetersPerSecond = settings.speedMetersPerSecond,
            .seed = fieldCache.settings.seed,
            .featureType = 3.0F,
        });
    diagnostics.emittedSampleCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        overlay.samples.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    overlay.fieldDiagnostics = diagnostics;
    FilterVisibleStreamSamplesToRegionBoundary(&overlay, fieldCache.regionBoundary);
    return overlay;
}

void IncludeRippleEffectPoint(
    WaterEffectOverlay* overlay,
    const WaterEffectLayer& layer,
    std::uint32_t pointIndex,
    const invisible_places::io::Float3& sourcePosition,
    const invisible_places::io::Float3& sourceNormal,
    const invisible_places::io::Float3& fieldVector,
    float edgeWeight,
    float edgeDistance,
    float shoreDistance,
    const glm::vec3& regionCenter) {
    if (overlay == nullptr) {
        return;
    }
    const glm::vec3 position = ToGlm(sourcePosition);
    const glm::vec3 normal = SafeOverlayNormal(ToGlm(sourceNormal));
    glm::vec3 tangent = ToGlm(fieldVector);
    tangent -= normal * glm::dot(tangent, normal);
    if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
        tangent = WaterPathLateral(normal);
    } else {
        tangent = glm::normalize(tangent);
    }
    const float clampedEdgeWeight = std::clamp(edgeWeight, 0.0F, 1.0F);
    const float seedValue = RegionHash01(layer.id + layer.seed, pointIndex, 901U);
    const auto pattern = EvaluateRipplePattern(
        layer,
        position,
        normal,
        tangent,
        regionCenter,
        clampedEdgeWeight,
        edgeDistance,
        shoreDistance,
        pointIndex);
    WaterEffectPoint effect;
    effect.position = sourcePosition;
    effect.normal = sourceNormal;
    effect.tangent = FromGlm(pattern.tangent);
    effect.sourcePointIndex = pointIndex;
    effect.blendMode = layer.blendMode;
    effect.red = FloatToByte(layer.response.colouriseRed);
    effect.green = FloatToByte(layer.response.colouriseGreen);
    effect.blue = FloatToByte(layer.response.colouriseBlue);
    effect.mask = std::clamp(layer.regionStrength, 0.0F, 1.0F);
    effect.edge = clampedEdgeWeight;
    const float patternEdgeWeight =
        layer.rippleOverlayType == WaterRippleOverlayType::CausticLace ? 1.0F : clampedEdgeWeight;
    effect.value =
        Clamp01(pattern.value * std::max(0.0F, layer.response.intensity)) *
        effect.mask *
        patternEdgeWeight;
    effect.ripplePotential =
        Clamp01(std::max(0.0F, layer.response.intensity) * effect.mask * patternEdgeWeight);
    effect.seed = seedValue;
    effect.regionId = static_cast<float>(layer.id);
    effect.distance = pattern.distance;
    effect.linearCoord = pattern.linearCoord;
    effect.angle = pattern.angle;
    effect.speed = std::max(0.0F, layer.speed);
    effect.confidence = Clamp01(pattern.confidence * patternEdgeWeight);
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
    effect.rippleEmissionHint = effect.ripplePotential * layer.response.emissionAdd;
    effect.rippleOpacityHint = effect.ripplePotential * layer.response.opacityAdd;
    effect.rippleOpacityMultiplyHint = std::lerp(
        1.0F,
        std::max(0.0F, layer.response.opacityMultiply),
        effect.ripplePotential);
    effect.rippleSizeHint = effect.ripplePotential * layer.response.pointSizeAdd;
    effect.rippleSizeMultiplyHint = std::lerp(
        1.0F,
        std::max(0.0F, layer.response.pointSizeMultiply),
        effect.ripplePotential);
    effect.rippleColourMixHint = effect.ripplePotential * layer.response.colouriseAmount;
    effect.wavelength = std::max(0.005F, layer.wavelengthMeters);
    effect.warp = std::max(0.0F, layer.warp);
    effect.phase = layer.phase;
    effect.featureType = 1.0F;
    IncludeEffectPoint(overlay, effect);
}

WaterEffectOverlay GenerateRippleEffectOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEffectLayer>& layers) {
    WaterEffectOverlay overlay;
    if (cloud.positions.empty()) {
        return overlay;
    }
    for (auto layer : layers) {
        if (layer.featureType != WaterEffectFeatureType::Ripple ||
            (!layer.enabledInViewport && !layer.enabledInExport)) {
            continue;
        }
        const auto selection = BuildWaterRegionSelection(cloud, layer);
        if (!selection.Valid()) {
            continue;
        }
        layer.hull = selection.hull;
        const glm::vec3 regionCenter = RippleRegionCentroid(selection.boundary);
        for (const auto& selected : selection.points) {
            IncludeRippleEffectPoint(
                &overlay,
                layer,
                selected.pointIndex,
                selected.position,
                selected.normal,
                selected.fieldVector,
                selected.edgeWeight,
                selected.edgeDistance,
                DirectionalDistanceToRegionEdge(
                    ToGlm(selected.position),
                    ToGlm(selected.fieldVector),
                    selection.boundary),
                regionCenter);
        }
    }
    return overlay;
}

WaterEffectOverlay GenerateRippleEffectOverlayFromPointIndices(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterEffectLayer& layer,
    const std::vector<std::uint32_t>& pointIndices) {
    WaterEffectOverlay overlay;
    if (cloud.positions.empty() ||
        pointIndices.empty() ||
        layer.featureType != WaterEffectFeatureType::Ripple ||
        (!layer.enabledInViewport && !layer.enabledInExport)) {
        return overlay;
    }
    const auto boundary = BuildWaterRegionBoundary(layer.vertices);
    if (boundary.size() < 3U) {
        return overlay;
    }
    invisible_places::io::Bounds3f bounds;
    for (const auto& vertex : boundary) {
        bounds.Expand(vertex);
    }
    const float edgeBlendWidth = std::max(1.0e-5F, layer.edgeBlendWidth);
    const glm::vec3 rawDirection{layer.directionX, layer.directionY, layer.directionZ};
    const glm::vec3 layerDirection =
        glm::dot(rawDirection, rawDirection) > kNormalEpsilon
            ? glm::normalize(rawDirection)
            : glm::vec3{1.0F, 0.0F, 0.0F};
    const glm::vec3 regionCenter = RippleRegionCentroid(boundary);

    for (const auto pointIndex : pointIndices) {
        if (pointIndex >= cloud.positions.size()) {
            continue;
        }
        const auto& sourcePosition = cloud.positions[pointIndex];
        if (!BoundsContainsXy(bounds, sourcePosition)) {
            continue;
        }
        const glm::vec3 position = ToGlm(sourcePosition);
        if (!PointInPolygonXy(position, boundary)) {
            continue;
        }

        glm::vec3 normal{0.0F, 0.0F, 1.0F};
        if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
            normal = SafeOverlayNormal(ToGlm(cloud.normals[pointIndex]));
        }
        const float edgeDistance3d = EffectPolygonEdgeDistance3d(position, boundary);
        const float edgeDistanceXy = EffectPolygonEdgeDistanceXy(position, boundary);
        const float edgeDistance = std::isfinite(edgeDistance3d) && edgeDistance3d > 0.0F
                                       ? edgeDistance3d
                                       : edgeDistanceXy;
        const float edgeWeight = SmoothStep(0.0F, edgeBlendWidth, edgeDistance);
        glm::vec3 fieldVector = layerDirection - normal * glm::dot(layerDirection, normal);
        if (glm::dot(fieldVector, fieldVector) <= kNormalEpsilon) {
            fieldVector = WaterPathLateral(normal);
        } else {
            fieldVector = glm::normalize(fieldVector);
        }
        IncludeRippleEffectPoint(
            &overlay,
            layer,
            pointIndex,
            sourcePosition,
            FromGlm(normal),
            FromGlm(fieldVector),
            edgeWeight,
            edgeDistance,
            DirectionalDistanceToRegionEdge(position, fieldVector, boundary),
            regionCenter);
    }
    return overlay;
}

WaterEffectOverlay GenerateRippleEffectOverlayFromSelection(
    const invisible_places::io::LoadedPointCloud& cloud,
    const WaterEffectLayer& layer,
    const WaterRegionSelection& selection) {
    WaterEffectOverlay overlay;
    if (cloud.positions.empty() ||
        selection.points.empty() ||
        selection.boundary.size() < 3U ||
        layer.featureType != WaterEffectFeatureType::Ripple ||
        (!layer.enabledInViewport && !layer.enabledInExport)) {
        return overlay;
    }

    const glm::vec3 regionCenter = RippleRegionCentroid(selection.boundary);
    for (const auto& selected : selection.points) {
        if (selected.pointIndex >= cloud.positions.size()) {
            continue;
        }
        IncludeRippleEffectPoint(
            &overlay,
            layer,
            selected.pointIndex,
            selected.position,
            selected.normal,
            selected.fieldVector,
            selected.edgeWeight,
            selected.edgeDistance,
            DirectionalDistanceToRegionEdge(
                ToGlm(selected.position),
                ToGlm(selected.fieldVector),
                selection.boundary),
            regionCenter);
    }
    return overlay;
}

WaterRippleRuntimeParams BuildWaterRippleRuntimeParams(
    const WaterEffectLayer& layer,
    const WaterRegionSelection& selection) {
    WaterRippleRuntimeParams params;
    const auto pattern = ActiveWaterRipplePatternSettings(layer);
    params.overlayType = layer.rippleOverlayType;
    params.blendMode = layer.blendMode;
    params.layerId = layer.id;
    params.seed = layer.seed;
    params.regionCenter = RippleRegionCentroid(selection.boundary);
    const glm::vec3 rawDirection{pattern.directionX, pattern.directionY, pattern.directionZ};
    params.direction = glm::dot(rawDirection, rawDirection) > kNormalEpsilon
                           ? glm::normalize(rawDirection)
                           : glm::vec3{1.0F, 0.0F, 0.0F};
    params.regionStrength = layer.regionStrength;
    params.edgeBlendWidth = layer.edgeBlendWidth;
    params.patternScale = pattern.patternScale;
    params.wavelengthMeters = pattern.wavelengthMeters;
    params.speed = pattern.speed;
    params.warp = pattern.warp;
    params.turbulence = pattern.turbulence;
    params.density = pattern.density;
    params.phase = pattern.phase;
    params.response = layer.response;
    return params;
}

std::vector<WaterRippleRuntimeMembership> BuildWaterRippleRuntimeMemberships(
    const WaterRegionSelection& selection,
    std::uint32_t paramIndex) {
    std::vector<WaterRippleRuntimeMembership> memberships;
    memberships.reserve(selection.points.size());
    for (const auto& point : selection.points) {
        if (point.pointIndex == std::numeric_limits<std::uint32_t>::max()) {
            continue;
        }
        WaterRippleRuntimeMembership membership;
        membership.pointIndex = point.pointIndex;
        membership.paramIndex = paramIndex;
        membership.edgeDistance = point.edgeDistance;
        membership.seed = RegionHash01(selection.layerId + paramIndex + 1U, point.pointIndex, 901U);
        membership.shoreDistance = DirectionalDistanceToRegionEdge(
            ToGlm(point.position),
            ToGlm(point.fieldVector),
            selection.boundary);
        memberships.push_back(membership);
    }
    return memberships;
}

WaterRippleRuntimeContribution EvaluateWaterRippleRuntimeContribution(
    const WaterRippleRuntimeParams& params,
    const WaterRippleRuntimeMembership& membership,
    const invisible_places::io::Float3& sourcePosition,
    const invisible_places::io::Float3& sourceNormal,
    float timeSeconds) {
    const glm::vec3 position = ToGlm(sourcePosition);
    const glm::vec3 normal = SafeOverlayNormal(ToGlm(sourceNormal));
    glm::vec3 tangent = params.direction - normal * glm::dot(params.direction, normal);
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
    const float edgeWeight = SmoothStep(
        0.0F,
        std::max(1.0e-5F, params.edgeBlendWidth),
        std::max(0.0F, membership.edgeDistance));
    const glm::vec3 relative = position - params.regionCenter;
    const float patternScale = std::clamp(params.patternScale, 0.05F, 100.0F);
    const float scaledShoreDistance = std::max(0.0F, membership.shoreDistance) * patternScale;
    const float scaledEdgeBlendWidth = params.edgeBlendWidth * patternScale;
    const glm::vec2 uv{
        glm::dot(relative, tangent) * patternScale,
        glm::dot(relative, lateral) * patternScale,
    };
    const glm::vec2 regionUv{
        relative.x * patternScale,
        relative.y * patternScale,
    };
    const float wavelength = std::max(0.005F, params.wavelengthMeters);
    const float phase = params.phase - (std::max(0.0F, timeSeconds) * std::max(0.0F, params.speed));
    const float seed =
        static_cast<float>(params.seed) * 0.013F +
        static_cast<float>(params.layerId) * 0.017F +
        static_cast<float>(RippleOverlayTypeSalt(params.overlayType)) * 0.011F;
    const float value = RuntimeRipplePatternValue(
        params.overlayType,
        uv,
        regionUv,
        normal,
        edgeWeight,
        scaledShoreDistance,
        scaledEdgeBlendWidth,
        wavelength,
        std::max(0.0F, params.warp),
        std::max(0.0F, params.turbulence),
        std::clamp(params.density, 0.0F, 1.0F),
        seed,
        phase);

    const float mask = std::clamp(params.regionStrength, 0.0F, 1.0F);
    const float edgeFactor =
        params.overlayType == WaterRippleOverlayType::CausticLace ? 1.0F : edgeWeight;
    const float scale = Clamp01(
        value *
        std::max(0.0F, params.response.intensity) *
        mask *
        edgeFactor);

    WaterRippleRuntimeContribution contribution;
    contribution.scale = scale;
    contribution.colourMix = Clamp01(scale * params.response.colouriseAmount);
    contribution.emissionAdd = std::max(0.0F, params.response.emissionAdd) * scale;
    contribution.opacityAdd = params.response.opacityAdd * scale;
    contribution.opacityMultiply = std::lerp(
        1.0F,
        std::max(0.0F, params.response.opacityMultiply),
        scale);
    contribution.pointSizeAdd = params.response.pointSizeAdd * scale;
    contribution.pointSizeMultiply = std::lerp(
        1.0F,
        std::max(0.0F, params.response.pointSizeMultiply),
        scale);
    contribution.colour = {
        std::clamp(params.response.colouriseRed, 0.0F, 1.0F),
        std::clamp(params.response.colouriseGreen, 0.0F, 1.0F),
        std::clamp(params.response.colouriseBlue, 0.0F, 1.0F),
    };
    return contribution;
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
    fields.rippleMask.assign(pointCount, 0.0F);
    fields.rippleEdge.assign(pointCount, 0.0F);
    fields.rippleValue.assign(pointCount, 0.0F);
    fields.rippleSeed.assign(pointCount, 0.0F);
    fields.rippleRegionId.assign(pointCount, 0.0F);
    fields.rippleDistance.assign(pointCount, 0.0F);
    fields.rippleLinearCoord.assign(pointCount, 0.0F);
    fields.rippleAngle.assign(pointCount, 0.0F);
    fields.rippleSpeed.assign(pointCount, 0.0F);
    fields.rippleConfidence.assign(pointCount, 0.0F);
    fields.rippleWavelength.assign(pointCount, 1.0F);
    fields.rippleWarp.assign(pointCount, 0.0F);
    fields.ripplePhase.assign(pointCount, 0.0F);

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
            const bool rippleEffect = std::abs(effect.featureType - 1.0F) <= 0.25F;
            const float value = std::clamp(
                rippleEffect && effect.ripplePotential > 1.0e-6F ? effect.ripplePotential : effect.value,
                0.0F,
                1.0F);
            if (value <= 1.0e-6F) {
                continue;
            }
            markTouched(index);
            const float emissionAdd = std::max(
                0.0F,
                rippleEffect && effect.ripplePotential > 1.0e-6F ? effect.rippleEmissionHint : effect.emissionHint);
            const float opacityAdd =
                rippleEffect && effect.ripplePotential > 1.0e-6F ? effect.rippleOpacityHint : effect.opacityHint;
            const float opacityMul = std::max(
                0.0F,
                rippleEffect && effect.ripplePotential > 1.0e-6F
                    ? effect.rippleOpacityMultiplyHint
                    : effect.opacityMultiplyHint);
            const float sizeAdd =
                rippleEffect && effect.ripplePotential > 1.0e-6F ? effect.rippleSizeHint : effect.sizeHint;
            const float sizeMul = std::max(
                0.0F,
                rippleEffect && effect.ripplePotential > 1.0e-6F
                    ? effect.rippleSizeMultiplyHint
                    : effect.sizeMultiplyHint);
            const float colourMix = std::clamp(
                rippleEffect && effect.ripplePotential > 1.0e-6F
                    ? effect.rippleColourMixHint
                    : effect.colourMixHint,
                0.0F,
                1.0F);

            if (rippleEffect && value >= fields.rippleValue[index]) {
                fields.rippleMask[index] = std::clamp(effect.mask, 0.0F, 1.0F);
                fields.rippleEdge[index] = std::clamp(effect.edge, 0.0F, 1.0F);
                fields.rippleValue[index] = value;
                fields.rippleSeed[index] = std::isfinite(effect.seed) ? effect.seed : 0.0F;
                fields.rippleRegionId[index] = std::isfinite(effect.regionId) ? effect.regionId : 0.0F;
                fields.rippleDistance[index] = std::isfinite(effect.distance) ? effect.distance : 0.0F;
                fields.rippleLinearCoord[index] = std::isfinite(effect.linearCoord) ? effect.linearCoord : 0.0F;
                fields.rippleAngle[index] = std::isfinite(effect.angle) ? effect.angle : 0.0F;
                fields.rippleSpeed[index] = std::max(0.0F, effect.speed);
                fields.rippleConfidence[index] = std::clamp(effect.confidence, 0.0F, 1.0F);
                fields.rippleWavelength[index] = std::max(0.005F, effect.wavelength);
                fields.rippleWarp[index] = std::max(0.0F, effect.warp);
                fields.ripplePhase[index] = std::isfinite(effect.phase) ? effect.phase : 0.0F;
            }

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
        {"stream_role", [](const WaterStreamSample& sample) { return sample.streamRole; }},
        {"stream_id", [](const WaterStreamSample& sample) { return sample.streamId; }},
        {"source_id", [](const WaterStreamSample& sample) { return sample.sourceId; }},
        {"path_id", [](const WaterStreamSample& sample) { return sample.pathId; }},
        {"branch_id", [](const WaterStreamSample& sample) { return sample.branchId; }},
        {"stream_seed", [](const WaterStreamSample& sample) { return sample.streamSeed; }},
        {"point_seed", [](const WaterStreamSample& sample) { return sample.pointSeed; }},
        {"stream_distance", [](const WaterStreamSample& sample) { return sample.streamDistance; }},
        {"stream_length", [](const WaterStreamSample& sample) { return sample.streamLength; }},
        {"route_start_index", [](const WaterStreamSample& sample) { return sample.routeStartIndex; }},
        {"route_point_count", [](const WaterStreamSample& sample) { return sample.routePointCount; }},
        {"route_length", [](const WaterStreamSample& sample) { return sample.routeLength; }},
        {"stream_start_phase", [](const WaterStreamSample& sample) { return sample.streamStartPhase; }},
        {"stream_lateral_offset", [](const WaterStreamSample& sample) { return sample.streamLateralOffset; }},
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
        {"stream_lane_index", [](const WaterStreamSample& sample) { return sample.streamLaneIndex; }},
        {"stream_lane_count", [](const WaterStreamSample& sample) { return sample.streamLaneCount; }},
        {"stream_lane_pitch", [](const WaterStreamSample& sample) { return sample.streamLanePitch; }},
        {"stream_lane_span", [](const WaterStreamSample& sample) { return sample.streamLaneSpan; }},
        {"stream_lane_crossing", [](const WaterStreamSample& sample) { return sample.streamLaneCrossing; }},
        {"stream_cross_seed", [](const WaterStreamSample& sample) { return sample.streamCrossSeed; }},
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
        {"ripple_wavelength", [](const WaterEffectPoint& point) { return point.wavelength; }},
        {"ripple_warp", [](const WaterEffectPoint& point) { return point.warp; }},
        {"ripple_phase", [](const WaterEffectPoint& point) { return point.phase; }},
        {"ripple_potential", [](const WaterEffectPoint& point) { return point.ripplePotential; }},
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

bool SaveWaterFieldCacheBinary(
    const WaterFieldCache& cache,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    if (const auto parent = outputPath.parent_path(); !parent.empty()) {
        std::error_code createError;
        std::filesystem::create_directories(parent, createError);
        if (createError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Unable to create water field cache directory: " + createError.message();
            }
            return false;
        }
    }

    std::ofstream output{outputPath, std::ios::binary};
    if (!output) {
        if (errorMessage != nullptr) {
            *errorMessage = "Unable to open water field cache for writing: " + outputPath.string();
        }
        return false;
    }

    auto writeU8 = [&output](std::uint8_t value) {
        output.write(reinterpret_cast<const char*>(&value), sizeof(value));
    };
    auto writeU32 = [&output](std::uint32_t value) {
        output.write(reinterpret_cast<const char*>(&value), sizeof(value));
    };
    auto writeFloatValue = [&output](float value) {
        output.write(reinterpret_cast<const char*>(&value), sizeof(value));
    };
    auto writeString = [&](const std::string& value) {
        const auto writeSize = static_cast<std::uint32_t>(std::min<std::size_t>(
            value.size(),
            static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
        writeU32(writeSize);
        output.write(value.data(), static_cast<std::streamsize>(writeSize));
    };
    auto writePoint = [&](const invisible_places::io::Float3& point) {
        writeFloatValue(point.x);
        writeFloatValue(point.y);
        writeFloatValue(point.z);
    };
    auto writeSettings = [&](const WaterFieldSettings& settings) {
        writeU8(settings.enabled ? 1U : 0U);
        writeU32(static_cast<std::uint32_t>(settings.outputMode));
        writeFloatValue(settings.corridorRadiusMeters);
        writeFloatValue(settings.fieldResolutionMeters);
        writeFloatValue(settings.projectionResolutionMeters);
        writeFloatValue(settings.guideWeight);
        writeFloatValue(settings.downhillWeight);
        writeFloatValue(settings.graphWeight);
        writeFloatValue(settings.lateralWeight);
        writeFloatValue(settings.fieldSmoothing);
        writeFloatValue(settings.wetnessSpread);
        writeFloatValue(settings.surfaceOffsetMeters);
        writeFloatValue(settings.surfaceConfidenceThreshold);
        writeFloatValue(settings.maxBridgeDistanceMeters);
        writeFloatValue(settings.bridgeAggression);
        writeFloatValue(settings.turbulence);
        writeU32(settings.seed);
    };
    auto writeResponse = [&](const WaterEffectResponseSettings& response) {
        writeFloatValue(response.intensity);
        writeFloatValue(response.emissionAdd);
        writeFloatValue(response.opacityAdd);
        writeFloatValue(response.opacityMultiply);
        writeFloatValue(response.pointSizeAdd);
        writeFloatValue(response.pointSizeMultiply);
        writeFloatValue(response.hueShift);
        writeFloatValue(response.colouriseRed);
        writeFloatValue(response.colouriseGreen);
        writeFloatValue(response.colouriseBlue);
        writeFloatValue(response.colouriseAmount);
        writeFloatValue(response.gaussianSharpnessBias);
    };

    output.write("IPWFC001", 8);
    writeU32(cache.schemaVersion);
    writeString(cache.supportLayerPath.generic_string());
    writeString(cache.supportSignature);
    writeString(cache.settingsFingerprint);
    writeString(cache.regionFingerprint);
    writeSettings(cache.settings);
    writeU8(cache.stale ? 1U : 0U);
    writeU32(static_cast<std::uint32_t>(std::min<std::size_t>(
        cache.regionBoundary.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))));
    for (const auto& point : cache.regionBoundary) {
        writePoint(point);
    }
    writeU32(static_cast<std::uint32_t>(std::min<std::size_t>(
        cache.nodes.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))));
    for (const auto& node : cache.nodes) {
        writePoint(node.position);
        writePoint(node.normal);
        writePoint(node.vector);
        writeU32(node.sourcePointIndex);
        writeU32(node.sourceLayerId);
        writeU32(static_cast<std::uint32_t>(node.blendMode));
        writeResponse(node.response);
        writeFloatValue(node.effectSpeed);
        writeU8(node.flowBlocked ? 1U : 0U);
        writeU8(node.bridgeAllowed ? 1U : 0U);
        writeU8(node.bridgeBlocked ? 1U : 0U);
        writeFloatValue(node.wetness);
        writeFloatValue(node.confidence);
        writeFloatValue(node.surfaceConfidence);
        writeFloatValue(node.pathStation);
        writeFloatValue(node.distanceToGuide);
    }
    if (!output) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed while writing water field cache: " + outputPath.string();
        }
        return false;
    }
    return true;
}

std::optional<WaterFieldCache> LoadWaterFieldCacheBinary(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    std::ifstream input{inputPath, std::ios::binary};
    if (!input) {
        if (errorMessage != nullptr) {
            *errorMessage = "Unable to open water field cache for reading: " + inputPath.string();
        }
        return std::nullopt;
    }

    auto readU8 = [&input]() {
        std::uint8_t value = 0U;
        input.read(reinterpret_cast<char*>(&value), sizeof(value));
        return value;
    };
    auto readU32 = [&input]() {
        std::uint32_t value = 0U;
        input.read(reinterpret_cast<char*>(&value), sizeof(value));
        return value;
    };
    auto readFloatValue = [&input]() {
        float value = 0.0F;
        input.read(reinterpret_cast<char*>(&value), sizeof(value));
        return value;
    };
    auto readString = [&]() {
        const auto size = readU32();
        constexpr std::uint32_t kMaxWaterFieldCacheStringBytes = 16U * 1024U * 1024U;
        if (size > kMaxWaterFieldCacheStringBytes) {
            input.setstate(std::ios::failbit);
            return std::string{};
        }
        std::string value(size, '\0');
        if (size > 0U) {
            input.read(value.data(), static_cast<std::streamsize>(size));
        }
        return value;
    };
    auto readPoint = [&]() {
        return invisible_places::io::Float3{
            readFloatValue(),
            readFloatValue(),
            readFloatValue(),
        };
    };
    auto readSettings = [&]() {
        WaterFieldSettings settings;
        settings.enabled = readU8() != 0U;
        settings.outputMode = static_cast<WaterFieldOutputMode>(readU32());
        settings.corridorRadiusMeters = readFloatValue();
        settings.fieldResolutionMeters = readFloatValue();
        settings.projectionResolutionMeters = readFloatValue();
        settings.guideWeight = readFloatValue();
        settings.downhillWeight = readFloatValue();
        settings.graphWeight = readFloatValue();
        settings.lateralWeight = readFloatValue();
        settings.fieldSmoothing = readFloatValue();
        settings.wetnessSpread = readFloatValue();
        settings.surfaceOffsetMeters = readFloatValue();
        settings.surfaceConfidenceThreshold = readFloatValue();
        settings.maxBridgeDistanceMeters = readFloatValue();
        settings.bridgeAggression = readFloatValue();
        settings.turbulence = readFloatValue();
        settings.seed = readU32();
        return settings;
    };
    auto readResponse = [&]() {
        WaterEffectResponseSettings response;
        response.intensity = readFloatValue();
        response.emissionAdd = readFloatValue();
        response.opacityAdd = readFloatValue();
        response.opacityMultiply = readFloatValue();
        response.pointSizeAdd = readFloatValue();
        response.pointSizeMultiply = readFloatValue();
        response.hueShift = readFloatValue();
        response.colouriseRed = readFloatValue();
        response.colouriseGreen = readFloatValue();
        response.colouriseBlue = readFloatValue();
        response.colouriseAmount = readFloatValue();
        response.gaussianSharpnessBias = readFloatValue();
        return response;
    };

    char magic[8] = {};
    input.read(magic, 8);
    if (std::string_view{magic, 8} != std::string_view{"IPWFC001", 8}) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water field cache has an unsupported header: " + inputPath.string();
        }
        return std::nullopt;
    }

    WaterFieldCache cache;
    cache.schemaVersion = readU32();
    cache.supportLayerPath = readString();
    cache.supportSignature = readString();
    cache.settingsFingerprint = readString();
    cache.regionFingerprint = readString();
    cache.settings = readSettings();
    cache.stale = readU8() != 0U;
    const auto boundaryCount = readU32();
    cache.regionBoundary.reserve(boundaryCount);
    for (std::uint32_t index = 0; index < boundaryCount; ++index) {
        cache.regionBoundary.push_back(readPoint());
    }
    const auto nodeCount = readU32();
    cache.nodes.reserve(nodeCount);
    for (std::uint32_t index = 0; index < nodeCount; ++index) {
        WaterFieldNode node;
        node.position = readPoint();
        node.normal = readPoint();
        node.vector = readPoint();
        node.sourcePointIndex = readU32();
        node.sourceLayerId = readU32();
        node.blendMode = static_cast<WaterEffectBlendMode>(readU32());
        node.response = readResponse();
        node.effectSpeed = readFloatValue();
        node.flowBlocked = readU8() != 0U;
        node.bridgeAllowed = readU8() != 0U;
        node.bridgeBlocked = readU8() != 0U;
        node.wetness = readFloatValue();
        node.confidence = readFloatValue();
        node.surfaceConfidence = readFloatValue();
        node.pathStation = readFloatValue();
        node.distanceToGuide = readFloatValue();
        cache.nodes.push_back(node);
    }
    if (!input) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed while reading water field cache: " + inputPath.string();
        }
        return std::nullopt;
    }
    return cache;
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
