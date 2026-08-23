#include "water/WaterFlow.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <bit>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/packing.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::water {

namespace {

constexpr glm::vec3 kGravity{0.0F, 0.0F, -1.0F};
constexpr float kNormalEpsilon = 1.0e-6F;
constexpr float kMeshTriangleNormalEpsilon = 1.0e-20F;

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
    tuned.attractorStrength = std::clamp(tuned.attractorStrength, 0.0F, 1.0F);
    if (!tuned.attractorEnabled || tuned.attractorStrength <= 1.0e-4F || !IsValidPoint(ToGlm(tuned.attractorPosition))) {
        tuned.attractorEnabled = false;
        tuned.attractorStrength = 0.0F;
    }
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
        const float inverseDistance = 1.0F / std::max(1.0e-5F, distance);
        const float horizontalDistance = glm::length(glm::vec2{delta.x, delta.y});
        const float horizontalRatio = horizontalDistance * inverseDistance;
        const float dropRatio = zDrop * inverseDistance;
        const float usefulDrop = std::max(preferredStep * 0.10F, searchRadius * 0.012F);
        const float strongDrop = std::max(preferredStep * 0.32F, searchRadius * 0.045F);
        const float alignment = glm::dot(glm::normalize(delta), direction);
        const float downhillScore = zDrop / std::max(0.012F, distance);
        const float uphillPenalty = zDrop < -settings.maxBridgeDistance * 0.08F ? std::abs(zDrop) * 12.0F : 0.0F;
        const float bridgeAmount = SmoothStep(bridgeStart, searchRadius, distance);
        const float beyondSoftBridge = SmoothStep(softBridgeLimit, searchRadius, distance);
        const float bridgePenalty =
            (distance / std::max(0.01F, searchRadius)) * 0.55F +
            bridgeAmount * (0.30F + (1.0F - gapTolerance) * 1.05F) +
            beyondSoftBridge * (2.35F - gapTolerance * 1.45F);
        const float downhillProgress = Clamp01(zDrop / std::max(1.0e-5F, strongDrop));
        const float lateralWithoutDrop = zDrop < usefulDrop ? Clamp01(horizontalRatio) : 0.0F;
        const float lateralBridgePenalty =
            lateralWithoutDrop *
            (0.62F + bridgeAmount * (0.55F + (1.0F - gapTolerance) * 0.65F) + beyondSoftBridge * 1.80F);
        const float contourBridgePenalty =
            (zDrop < usefulDrop)
                ? SmoothStep(preferredStep * 1.35F, searchRadius, horizontalDistance) *
                      (0.42F + beyondSoftBridge * 1.65F)
                : 0.0F;
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
        const float zProgressBias =
            std::clamp(dropRatio, -1.0F, 1.0F) * 1.65F +
            downhillProgress * 1.10F;
        // The attractor is horizontal guidance only; Z descent stays governed by the
        // downhill score and acceptance thresholds so routes do not climb toward it.
        float attractorBias = 0.0F;
        if (settings.attractorEnabled && settings.attractorStrength > 1.0e-4F) {
            const glm::vec3 attractor = ToGlm(settings.attractorPosition);
            const glm::vec2 currentToAttractor{
                attractor.x - current.position.x,
                attractor.y - current.position.y};
            const glm::vec2 candidateToAttractor{
                attractor.x - candidate.position.x,
                attractor.y - candidate.position.y};
            const float currentAttractorDistance = glm::length(currentToAttractor);
            const float candidateAttractorDistance = glm::length(candidateToAttractor);
            if (currentAttractorDistance > 1.0e-4F) {
                const float progressTowardAttractor =
                    (currentAttractorDistance - candidateAttractorDistance) * inverseDistance;
                const float proximityFade = 1.0F - SmoothStep(
                    std::max(searchRadius * 0.50F, preferredStep),
                    std::max(searchRadius * 18.0F, settings.pathLength * 0.85F),
                    currentAttractorDistance);
                attractorBias =
                    std::clamp(progressTowardAttractor, -1.0F, 1.0F) *
                    settings.attractorStrength *
                    (0.55F + proximityFade * 0.70F);
                if (horizontalDistance > 1.0e-5F) {
                    const float axisProgress =
                        ((std::abs(currentToAttractor.x) - std::abs(candidateToAttractor.x)) +
                         (std::abs(currentToAttractor.y) - std::abs(candidateToAttractor.y))) /
                        horizontalDistance;
                    attractorBias +=
                        std::clamp(axisProgress, -1.0F, 1.0F) *
                        settings.attractorStrength *
                        0.85F;
                }
            }
        }
        const float score =
            (downhillScore * 4.15F) +
            (alignment * 1.35F) +
            (candidate.confidence * 1.25F) +
            (normalCoherence * 0.45F) +
            zProgressBias -
            (attractorBias < 0.0F ? -attractorBias * 0.90F : 0.0F) +
            std::max(0.0F, attractorBias) * 2.45F -
            bridgePenalty -
            uphillPenalty -
            lateralBridgePenalty -
            contourBridgePenalty +
            (flatness * std::clamp(settings.branching, 0.0F, 1.0F) * 0.22F);
        const float acceptanceThreshold = 1.45F + beyondSoftBridge * (1.35F - gapTolerance * 0.85F);
        const float uphillAllowance = settings.maxBridgeDistance * (0.012F + gapTolerance * 0.030F);
        const bool acceptedDownhill = zDrop >= usefulDrop;
        const bool acceptedFlatLocal =
            zDrop >= -uphillAllowance &&
            distance <= bridgeStart * (1.15F + gapTolerance * 0.55F);
        const bool acceptedByScore =
            score > acceptanceThreshold &&
            zDrop >= -uphillAllowance * (1.50F + gapTolerance);
        if (acceptedDownhill || acceptedFlatLocal || acceptedByScore) {
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

float CentripetalCatmullRomKnot(float previous, const glm::vec3& left, const glm::vec3& right) {
    return previous + std::sqrt(std::max(glm::length(right - left), 1.0e-8F));
}

glm::vec3 InterpolateCentripetalCatmullRom(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& p2,
    const glm::vec3& p3,
    float amount) {
    const float t0 = 0.0F;
    const float t1 = CentripetalCatmullRomKnot(t0, p0, p1);
    const float t2 = CentripetalCatmullRomKnot(t1, p1, p2);
    const float t3 = CentripetalCatmullRomKnot(t2, p2, p3);
    const float t = t1 + (t2 - t1) * Clamp01(amount);
    const auto blend = [](const glm::vec3& left, const glm::vec3& right, float leftT, float rightT, float valueT) {
        const float denominator = std::max(1.0e-8F, rightT - leftT);
        return left * ((rightT - valueT) / denominator) +
               right * ((valueT - leftT) / denominator);
    };
    const glm::vec3 a1 = blend(p0, p1, t0, t1, t);
    const glm::vec3 a2 = blend(p1, p2, t1, t2, t);
    const glm::vec3 a3 = blend(p2, p3, t2, t3, t);
    const glm::vec3 b1 = blend(a1, a2, t0, t2, t);
    const glm::vec3 b2 = blend(a2, a3, t1, t3, t);
    return blend(b1, b2, t1, t2, t);
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
        const auto& p1 = anchors[index];
        const auto& p2 = anchors[index + 1U];
        const glm::vec3 p1Position = ToGlm(p1.position);
        const glm::vec3 p2Position = ToGlm(p2.position);
        const glm::vec3 p0Position =
            index > 0U ? ToGlm(anchors[index - 1U].position) : p1Position + (p1Position - p2Position);
        const glm::vec3 p3Position =
            index + 2U < anchors.size()
                ? ToGlm(anchors[index + 2U].position)
                : p2Position + (p2Position - p1Position);
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
            sample.position = FromGlm(InterpolateCentripetalCatmullRom(
                p0Position,
                p1Position,
                p2Position,
                p3Position,
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
        const float horizontalTangent = glm::length(glm::vec2{tangent.x, tangent.y});
        const float verticalLaneTightening = 0.22F + SmoothStep(0.18F, 0.70F, horizontalTangent) * 0.78F;
        const float guideSteepness = std::max(lanePoint.surfaceSteepness, FallbackPathTurbulence(guidePath, index));
        const float endpointFade =
            std::min(
                SmoothStep(0.0F, 2.0F, static_cast<float>(index)),
                SmoothStep(0.0F, 2.0F, static_cast<float>((guidePath.size() - 1U) - index)));
        const float maxOffset = std::max(
            0.0F,
            lanePoint.width * jitter * (0.28F + looseness * 0.34F) * endpointFade * verticalLaneTightening);
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
    if (left.laneSpanMeters >= 0.0F && right.laneSpanMeters >= 0.0F) {
        point.laneSpanMeters =
            left.laneSpanMeters + ((right.laneSpanMeters - left.laneSpanMeters) * t);
    } else {
        point.laneSpanMeters =
            left.laneSpanMeters >= 0.0F ? left.laneSpanMeters : right.laneSpanMeters;
    }
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

WaterVisualSettings DefaultWaterVisualSettings() {
    return {};
}

WaterTrailGeometrySettings DefaultWaterTrailGeometrySettings() {
    return {};
}

float AutoWaterTrailPointSpacingMeters(float trailLengthMeters, float widthMeters) {
    const float safeLength = std::clamp(trailLengthMeters, 0.001F, 50.0F);
    const float safeWidth = std::clamp(widthMeters, 0.0005F, 1.0F);
    const float widthDrivenSpacing = safeWidth * 1.65F;
    const float lengthDrivenSpacing = safeLength / 128.0F;
    return std::clamp(std::max(widthDrivenSpacing, lengthDrivenSpacing), 0.002F, 0.20F);
}

float AutoWaterTrailStreakLengthMeters(float pointSpacingMeters, float widthMeters) {
    const float safeSpacing = std::clamp(pointSpacingMeters, 0.001F, 10.0F);
    const float safeWidth = std::clamp(widthMeters, 0.0005F, 1.0F);
    return std::clamp(std::max({safeWidth * 7.5F, safeSpacing * 3.25F, safeWidth}), 0.002F, 5.0F);
}

WaterTrailGeometrySettings FitWaterTrailGeometryForContinuousLines(
    WaterTrailGeometrySettings geometry) {
    geometry.trailLengthMeters = std::clamp(geometry.trailLengthMeters, 0.001F, 50.0F);
    geometry.widthMeters = std::clamp(geometry.widthMeters, 0.0005F, 1.0F);
    geometry.pointSpacingMeters =
        AutoWaterTrailPointSpacingMeters(geometry.trailLengthMeters, geometry.widthMeters);
    geometry.streakLengthMeters =
        AutoWaterTrailStreakLengthMeters(geometry.pointSpacingMeters, geometry.widthMeters);
    geometry.startFadeFullDistanceMeters =
        std::clamp(geometry.startFadeFullDistanceMeters, 0.0F, 50.0F);
    geometry.startFadeRandomBeginDistanceMeters =
        std::clamp(geometry.startFadeRandomBeginDistanceMeters, 0.0F, 50.0F);
    geometry.endFadeFullDistanceMeters =
        std::clamp(geometry.endFadeFullDistanceMeters, 0.0F, 50.0F);
    geometry.endFadeRandomBeginDistanceMeters =
        std::clamp(geometry.endFadeRandomBeginDistanceMeters, 0.0F, 50.0F);
    return geometry;
}

WaterTrailGeometrySettings WaterTrailGeometryFromFlowTrailSettings(
    const WaterFlowTrailSettings& settings) {
    WaterTrailGeometrySettings geometry;
    geometry.trailLengthMeters = settings.trailLengthMeters;
    geometry.pointSpacingMeters = settings.trailPointSpacingMeters;
    geometry.widthMeters = settings.trailWidthMeters;
    geometry.streakLengthMeters = settings.trailStreakLengthMeters;
    geometry.startFadeEnabled = settings.startFadeEnabled;
    geometry.startFadeFullDistanceMeters = settings.startFadeFullDistanceMeters;
    geometry.startFadeRandomBeginDistanceMeters =
        settings.startFadeRandomBeginDistanceMeters;
    geometry.endFadeEnabled = settings.endFadeEnabled;
    geometry.endFadeFullDistanceMeters = settings.endFadeFullDistanceMeters;
    geometry.endFadeRandomBeginDistanceMeters =
        settings.endFadeRandomBeginDistanceMeters;
    return geometry;
}

WaterFlowTrailSettings ApplyWaterTrailGeometryToFlowTrailSettings(
    WaterFlowTrailSettings settings,
    const WaterTrailGeometrySettings& geometry) {
    settings.trailLengthMeters = geometry.trailLengthMeters;
    settings.trailPointSpacingMeters = geometry.pointSpacingMeters;
    settings.trailWidthMeters = geometry.widthMeters;
    settings.trailStreakLengthMeters = geometry.streakLengthMeters;
    settings.startFadeEnabled = geometry.startFadeEnabled;
    settings.startFadeFullDistanceMeters = geometry.startFadeFullDistanceMeters;
    settings.startFadeRandomBeginDistanceMeters =
        geometry.startFadeRandomBeginDistanceMeters;
    settings.endFadeEnabled = geometry.endFadeEnabled;
    settings.endFadeFullDistanceMeters = geometry.endFadeFullDistanceMeters;
    settings.endFadeRandomBeginDistanceMeters =
        geometry.endFadeRandomBeginDistanceMeters;
    return settings;
}

WaterFlowGpuCompactInput BuildWaterFlowGpuSampledInput(
    std::span<const WaterOverlayPoint> anchors) {
    constexpr float kDuplicateDistance = 1.0e-5F;
    WaterFlowGpuCompactInput input;
    input.points.reserve(anchors.size());
    for (const auto& anchor : anchors) {
        const glm::vec3 position = ToGlm(anchor.position);
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) {
            continue;
        }
        if (!input.points.empty()) {
            const glm::vec3 previous = ToGlm(input.points.back().position);
            if (glm::length(position - previous) <= kDuplicateDistance) {
                continue;
            }
        }
        WaterFlowGpuCompactInputPoint point;
        point.position = anchor.position;
        point.normal = FromGlm(SafeOverlayNormal(ToGlm(anchor.normal)));
        point.confidence = std::clamp(
            std::isfinite(anchor.confidence) ? anchor.confidence : 1.0F,
            0.0F,
            1.0F);
        input.points.push_back(point);
    }

    float cumulativeDistance = 0.0F;
    for (std::size_t index = 0U; index + 1U < input.points.size(); ++index) {
        auto& point = input.points[index];
        point.cumulativeDistanceMeters = cumulativeDistance;
        const float segmentLength = glm::length(
            ToGlm(input.points[index + 1U].position) - ToGlm(point.position));
        point.outgoingSegmentArcDistancesMeters = {
            segmentLength * 0.25F,
            segmentLength * 0.50F,
            segmentLength * 0.75F,
            segmentLength,
        };
        cumulativeDistance += segmentLength;
    }
    if (!input.points.empty()) {
        input.points.back().cumulativeDistanceMeters = cumulativeDistance;
    }
    input.routeLengthMeters = cumulativeDistance;
    return input;
}

WaterFlowGpuCompactInput BuildWaterFlowGpuManualSplineInput(
    std::span<const invisible_places::io::Float3> controlPoints) {
    return BuildWaterFlowGpuManualSplineInput(controlPoints, {});
}

WaterFlowGpuCompactInput BuildWaterFlowGpuManualSplineInput(
    std::span<const invisible_places::io::Float3> controlPoints,
    std::span<const WaterManualFlowPathLaneWidth> laneWidths) {
    constexpr float kDuplicateDistance = 1.0e-5F;
    constexpr std::uint32_t kArcLengthSubdivisions = 16U;
    constexpr std::uint32_t kSubdivisionsPerCheckpoint = 4U;
    WaterFlowGpuCompactInput input;
    input.points.reserve(controlPoints.size());
    for (std::size_t controlIndex = 0U;
         controlIndex < controlPoints.size();
         ++controlIndex) {
        const auto& controlPoint = controlPoints[controlIndex];
        const glm::vec3 position = ToGlm(controlPoint);
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) {
            continue;
        }
        if (!input.points.empty()) {
            const glm::vec3 previous = ToGlm(input.points.back().position);
            if (glm::length(position - previous) <= kDuplicateDistance) {
                continue;
            }
        }
        WaterFlowGpuCompactInputPoint point;
        point.position = controlPoint;
        if (controlIndex < laneWidths.size()) {
            point.laneWidth = laneWidths[controlIndex];
            if (!std::isfinite(point.laneWidth.value)) {
                point.laneWidth = {};
            }
            switch (point.laneWidth.mode) {
                case WaterManualFlowPathLaneWidthMode::Inherit:
                    point.laneWidth.value = 1.0F;
                    break;
                case WaterManualFlowPathLaneWidthMode::Absolute:
                    point.laneWidth.value =
                        std::clamp(point.laneWidth.value, 0.0F, 100.0F);
                    break;
                case WaterManualFlowPathLaneWidthMode::Relative:
                    point.laneWidth.value =
                        std::clamp(point.laneWidth.value, 0.0F, 100.0F);
                    break;
                default:
                    point.laneWidth = {};
                    break;
            }
        }
        input.points.push_back(point);
    }

    float cumulativeDistance = 0.0F;
    for (std::size_t segmentIndex = 0U;
         segmentIndex + 1U < input.points.size();
         ++segmentIndex) {
        auto& point = input.points[segmentIndex];
        point.cumulativeDistanceMeters = cumulativeDistance;
        const glm::vec3 p1 = ToGlm(point.position);
        const glm::vec3 p2 = ToGlm(input.points[segmentIndex + 1U].position);
        const glm::vec3 p0 = segmentIndex > 0U
                                 ? ToGlm(input.points[segmentIndex - 1U].position)
                                 : p1 + (p1 - p2);
        const glm::vec3 p3 = segmentIndex + 2U < input.points.size()
                                 ? ToGlm(input.points[segmentIndex + 2U].position)
                                 : p2 + (p2 - p1);
        float segmentDistance = 0.0F;
        glm::vec3 previousPosition = p1;
        for (std::uint32_t step = 1U; step <= kArcLengthSubdivisions; ++step) {
            const float amount = static_cast<float>(step) /
                                 static_cast<float>(kArcLengthSubdivisions);
            const glm::vec3 position = input.points.size() > 2U
                                           ? InterpolateCentripetalCatmullRom(
                                                 p0,
                                                 p1,
                                                 p2,
                                                 p3,
                                                 amount)
                                           : glm::mix(p1, p2, amount);
            segmentDistance += glm::length(position - previousPosition);
            previousPosition = position;
            if ((step % kSubdivisionsPerCheckpoint) == 0U) {
                const std::size_t checkpoint =
                    static_cast<std::size_t>(step / kSubdivisionsPerCheckpoint - 1U);
                point.outgoingSegmentArcDistancesMeters[checkpoint] = segmentDistance;
            }
        }
        cumulativeDistance += segmentDistance;
    }
    if (!input.points.empty()) {
        input.points.back().cumulativeDistanceMeters = cumulativeDistance;
    }
    input.routeLengthMeters = cumulativeDistance;
    return input;
}

bool WaterFlowGpuCompactSourceInput::Valid() const {
    if (points.size() < 2U || branches.empty() ||
        points.size() > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }
    std::uint64_t expectedInputStart = 0U;
    for (const auto& branch : branches) {
        const std::uint64_t inputEnd =
            static_cast<std::uint64_t>(branch.inputStart) + branch.inputCount;
        if (branch.inputStart != expectedInputStart || branch.inputCount < 2U ||
            inputEnd > points.size() || branch.pathId == 0U ||
            !std::isfinite(branch.routeLengthMeters) ||
            branch.routeLengthMeters <= 1.0e-5F ||
            !std::isfinite(branch.allocationWeight) ||
            branch.allocationWeight < 0.0F) {
            return false;
        }
        expectedInputStart = inputEnd;
    }
    return expectedInputStart == points.size();
}

namespace {

std::vector<std::uint32_t> AllocateExactTrailCountsForPaths(
    const std::vector<float>& pathLengths,
    const std::vector<float>& pathWeights,
    std::uint32_t requestedTrailCount) {
    std::vector<std::uint32_t> allocations(pathLengths.size(), 0U);
    if (requestedTrailCount == 0U || pathLengths.empty()) {
        return allocations;
    }

    struct Candidate {
        std::size_t index = 0U;
        float weight = 0.0F;
        double remainder = 0.0;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(pathLengths.size());
    for (std::size_t index = 0U; index < pathLengths.size(); ++index) {
        const float length = pathLengths[index];
        if (length <= 1.0e-5F) {
            continue;
        }
        const float weight =
            index < pathWeights.size() ? std::max(0.0F, pathWeights[index]) : length;
        candidates.push_back({
            .index = index,
            .weight = weight > 1.0e-5F ? weight : length,
        });
    }
    if (candidates.empty()) {
        return allocations;
    }

    const auto byWeight = [](const Candidate& left, const Candidate& right) {
        if (left.weight != right.weight) {
            return left.weight > right.weight;
        }
        return left.index < right.index;
    };
    std::sort(candidates.begin(), candidates.end(), byWeight);

    const std::uint32_t initiallyAssigned = std::min<std::uint32_t>(
        requestedTrailCount,
        static_cast<std::uint32_t>(candidates.size()));
    for (std::uint32_t index = 0U; index < initiallyAssigned; ++index) {
        allocations[candidates[index].index] = 1U;
    }
    if (initiallyAssigned == requestedTrailCount) {
        return allocations;
    }

    const std::uint32_t remaining = requestedTrailCount - initiallyAssigned;
    double totalWeight = 0.0;
    for (const auto& candidate : candidates) {
        totalWeight += static_cast<double>(std::max(candidate.weight, 0.0F));
    }
    if (totalWeight <= 1.0e-9) {
        totalWeight = static_cast<double>(candidates.size());
        for (auto& candidate : candidates) {
            candidate.weight = 1.0F;
        }
    }

    std::uint32_t assignedExtra = 0U;
    for (auto& candidate : candidates) {
        const double exactShare =
            (static_cast<double>(remaining) * static_cast<double>(candidate.weight)) /
            totalWeight;
        const auto extra = static_cast<std::uint32_t>(std::floor(exactShare));
        allocations[candidate.index] += extra;
        assignedExtra += extra;
        candidate.remainder = exactShare - static_cast<double>(extra);
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate& left, const Candidate& right) {
            if (left.remainder != right.remainder) {
                return left.remainder > right.remainder;
            }
            if (left.weight != right.weight) {
                return left.weight > right.weight;
            }
            return left.index < right.index;
        });
    for (std::uint32_t index = 0U; assignedExtra < remaining; ++index, ++assignedExtra) {
        allocations[candidates[index % candidates.size()].index] += 1U;
    }
    return allocations;
}

std::uint32_t WaterFlowGpuPotentialLaneCount(
    const WaterFlowTrailSettings& settings) {
    if (settings.laneCount > 0U) {
        return settings.laneCount;
    }
    const float width = std::clamp(
        std::isfinite(settings.trailWidthMeters) ? settings.trailWidthMeters : 0.006F,
        0.0005F,
        1.0F);
    const float span = std::clamp(
        std::isfinite(settings.laneSpreadMeters) ? settings.laneSpreadMeters : 0.12F,
        0.0F,
        100.0F);
    const float lanePitch = std::max(width * 0.5F, 0.00025F);
    const std::uint32_t automaticLaneCount = static_cast<std::uint32_t>(std::max(
        1.0F,
        std::ceil(span / lanePitch)));
    return std::max(1U, automaticLaneCount);
}

std::uint32_t WaterFlowGpuRoutePointCount(float routeLengthMeters, float spacingMeters) {
    const float requested = std::ceil(routeLengthMeters / spacingMeters) + 1.0F;
    if (!std::isfinite(requested) || requested >= 8192.0F) {
        return 8192U;
    }
    return static_cast<std::uint32_t>(std::max(2.0F, requested));
}

WaterFlowGpuOutputLayout BuildWaterFlowGpuOutputLayoutForBranches(
    std::uint32_t inputPointCount,
    std::span<const WaterFlowGpuCompactBranch> inputBranches,
    const WaterFlowTrailSettings& settings,
    std::uint32_t currentPointCapacity,
    std::uint32_t maximumPointCapacity) {
    WaterFlowGpuOutputLayout layout;
    layout.inputPointCount = inputPointCount;
    layout.branchCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        inputBranches.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    if (!settings.enabled || inputPointCount < 2U || inputBranches.empty() ||
        inputBranches.size() > std::numeric_limits<std::uint32_t>::max() ||
        settings.trailCountTotal == 0U || maximumPointCapacity == 0U) {
        return layout;
    }

    const float spacing = std::clamp(
        std::isfinite(settings.trailPointSpacingMeters)
            ? settings.trailPointSpacingMeters
            : 0.010F,
        0.001F,
        5.0F);
    layout.laneCount = WaterFlowGpuPotentialLaneCount(settings);
    layout.samplesPerTrail = std::clamp<std::uint32_t>(
        static_cast<std::uint32_t>(std::ceil(
            std::clamp(
                std::isfinite(settings.trailLengthMeters)
                    ? settings.trailLengthMeters
                    : 0.75F,
                0.02F,
                100.0F) /
            spacing)) +
            1U,
        2U,
        8192U);

    std::vector<float> routeLengths;
    std::vector<float> allocationWeights;
    routeLengths.reserve(inputBranches.size());
    allocationWeights.reserve(inputBranches.size());
    double totalRouteLength = 0.0;
    for (const auto& branch : inputBranches) {
        const float routeLength =
            std::isfinite(branch.routeLengthMeters)
                ? std::max(0.0F, branch.routeLengthMeters)
                : 0.0F;
        routeLengths.push_back(routeLength);
        allocationWeights.push_back(
            std::isfinite(branch.allocationWeight)
                ? std::max(0.0F, branch.allocationWeight)
                : 0.0F);
        totalRouteLength += routeLength;
    }
    layout.routeLengthMeters = static_cast<float>(std::min<double>(
        totalRouteLength,
        std::numeric_limits<float>::max()));
    const auto trailAllocations = AllocateExactTrailCountsForPaths(
        routeLengths,
        allocationWeights,
        settings.trailCountTotal);

    layout.branches.reserve(inputBranches.size());
    std::uint64_t routePointCount = 0U;
    std::uint64_t allocatedTrailCount = 0U;
    std::uint64_t firstTrailId = 1U;
    for (std::size_t index = 0U; index < inputBranches.size(); ++index) {
        const auto& inputBranch = inputBranches[index];
        const std::uint32_t branchTrailCount = trailAllocations[index];
        const std::uint32_t routePointsPerLane =
            WaterFlowGpuRoutePointCount(routeLengths[index], spacing);
        const std::uint32_t activeRouteLaneCount =
            std::min(layout.laneCount, branchTrailCount);

        WaterFlowGpuBranchLayout branch;
        branch.inputStart = inputBranch.inputStart;
        branch.inputCount = inputBranch.inputCount;
        branch.routePointsPerLane = routePointsPerLane;
        branch.activeRouteLaneCount = activeRouteLaneCount;
        branch.trailCount = branchTrailCount;
        branch.firstTrailId = static_cast<std::uint32_t>(firstTrailId);
        branch.potentialLaneCount = layout.laneCount;
        branch.branchId = inputBranch.branchId;
        branch.pathId = inputBranch.pathId;
        branch.routeLengthMeters = routeLengths[index];
        branch.routeSpacingMeters = spacing;
        layout.branches.push_back(branch);

        routePointCount +=
            static_cast<std::uint64_t>(routePointsPerLane) * activeRouteLaneCount;
        allocatedTrailCount += branchTrailCount;
        firstTrailId += branchTrailCount;
        if (activeRouteLaneCount > 0U) {
            layout.routePointCountPerLane =
                std::max(layout.routePointCountPerLane, routePointsPerLane);
        }
        layout.maxActiveRouteLaneCount =
            std::max(layout.maxActiveRouteLaneCount, activeRouteLaneCount);
        layout.maxTrailsPerBranch =
            std::max(layout.maxTrailsPerBranch, branchTrailCount);
    }
    if (allocatedTrailCount != settings.trailCountTotal ||
        routePointCount > std::numeric_limits<std::uint32_t>::max()) {
        return layout;
    }

    const std::uint64_t trailPointCount =
        allocatedTrailCount * layout.samplesPerTrail;
    const std::uint64_t pointCount = routePointCount + trailPointCount;
    if (trailPointCount > std::numeric_limits<std::uint32_t>::max() ||
        pointCount > maximumPointCapacity ||
        pointCount > std::numeric_limits<std::uint32_t>::max()) {
        return layout;
    }
    layout.routePointCountTotal = static_cast<std::uint32_t>(routePointCount);
    layout.trailCount = static_cast<std::uint32_t>(allocatedTrailCount);
    layout.trailPointCountTotal = static_cast<std::uint32_t>(trailPointCount);
    layout.pointCount = static_cast<std::uint32_t>(pointCount);

    std::uint32_t routeStart = 0U;
    std::uint32_t trailOutputStart = layout.routePointCountTotal;
    for (auto& branch : layout.branches) {
        branch.routeStart = routeStart;
        branch.trailOutputStart = trailOutputStart;
        routeStart += branch.routePointsPerLane * branch.activeRouteLaneCount;
        trailOutputStart += branch.trailCount * layout.samplesPerTrail;
    }

    if (currentPointCapacity >= layout.pointCount && currentPointCapacity <= maximumPointCapacity) {
        layout.pointCapacity = currentPointCapacity;
        return layout;
    }
    const std::uint32_t minimumCapacity = std::max(1024U, layout.pointCount);
    layout.pointCapacity = std::bit_ceil(minimumCapacity);
    if (layout.pointCapacity < minimumCapacity || layout.pointCapacity > maximumPointCapacity) {
        layout.pointCapacity = maximumPointCapacity;
    }
    return layout;
}

}  // namespace

WaterFlowGpuOutputLayout BuildWaterFlowGpuOutputLayout(
    const WaterFlowGpuCompactSourceInput& input,
    const WaterFlowTrailSettings& settings,
    std::uint32_t currentPointCapacity,
    std::uint32_t maximumPointCapacity) {
    if (!input.Valid() || input.points.size() > std::numeric_limits<std::uint32_t>::max()) {
        return {};
    }
    return BuildWaterFlowGpuOutputLayoutForBranches(
        static_cast<std::uint32_t>(input.points.size()),
        input.branches,
        settings,
        currentPointCapacity,
        maximumPointCapacity);
}

WaterFlowGpuOutputLayout BuildWaterFlowGpuOutputLayout(
    std::uint32_t inputPointCount,
    float routeLengthMeters,
    const WaterFlowTrailSettings& settings,
    std::uint32_t currentPointCapacity,
    std::uint32_t maximumPointCapacity) {
    WaterFlowGpuCompactBranch branch;
    branch.inputCount = inputPointCount;
    branch.pathId = 1U;
    branch.routeLengthMeters =
        std::isfinite(routeLengthMeters) ? std::max(0.0F, routeLengthMeters) : 0.0F;
    branch.allocationWeight = branch.routeLengthMeters;
    return BuildWaterFlowGpuOutputLayoutForBranches(
        inputPointCount,
        std::span<const WaterFlowGpuCompactBranch>{&branch, 1U},
        settings,
        currentPointCapacity,
        maximumPointCapacity);
}

bool WaterTrailGeometryGenerationInputsEqual(
    const WaterTrailGeometrySettings& left,
    const WaterTrailGeometrySettings& right) {
    return left.trailLengthMeters == right.trailLengthMeters &&
           left.pointSpacingMeters == right.pointSpacingMeters &&
           left.startFadeEnabled == right.startFadeEnabled &&
           left.startFadeFullDistanceMeters == right.startFadeFullDistanceMeters &&
           left.startFadeRandomBeginDistanceMeters ==
               right.startFadeRandomBeginDistanceMeters &&
           left.endFadeEnabled == right.endFadeEnabled &&
           left.endFadeFullDistanceMeters == right.endFadeFullDistanceMeters &&
           left.endFadeRandomBeginDistanceMeters ==
               right.endFadeRandomBeginDistanceMeters;
}

bool WaterTrailGeometryLiveVisualOnlyEdit(
    const WaterTrailGeometrySettings& before,
    const WaterTrailGeometrySettings& after) {
    return WaterTrailGeometryGenerationInputsEqual(before, after) &&
           (before.widthMeters != after.widthMeters ||
            before.streakLengthMeters != after.streakLengthMeters);
}

bool WaterFlowLaneRouteInputsEqual(
    const WaterFlowTrailSettings& left,
    const WaterFlowTrailSettings& right) {
    return left.enabled == right.enabled &&
           left.trailCountTotal == right.trailCountTotal &&
           left.laneCount == right.laneCount &&
           left.trailLengthMeters == right.trailLengthMeters &&
           left.trailPointSpacingMeters == right.trailPointSpacingMeters &&
           left.startFadeEnabled == right.startFadeEnabled &&
           left.startFadeFullDistanceMeters == right.startFadeFullDistanceMeters &&
           left.startFadeRandomBeginDistanceMeters ==
               right.startFadeRandomBeginDistanceMeters &&
           left.endFadeEnabled == right.endFadeEnabled &&
           left.endFadeFullDistanceMeters == right.endFadeFullDistanceMeters &&
           left.endFadeRandomBeginDistanceMeters ==
               right.endFadeRandomBeginDistanceMeters &&
           left.surfaceOffsetMeters == right.surfaceOffsetMeters &&
           left.pathAttraction == right.pathAttraction &&
           left.laneSpreadMeters == right.laneSpreadMeters &&
           left.laneCrossing == right.laneCrossing &&
           left.trailSmoothness == right.trailSmoothness &&
           left.trailLooseness == right.trailLooseness &&
           left.turbulence == right.turbulence &&
           left.surfaceFollow == right.surfaceFollow &&
           left.downhillPull == right.downhillPull &&
           left.terrainWidthResponse == right.terrainWidthResponse &&
           left.turbulenceScaleMeters == right.turbulenceScaleMeters &&
           left.seed == right.seed;
}

bool WaterFlowLaneSpeedOnlyEdit(
    const WaterFlowTrailSettings& before,
    const WaterFlowTrailSettings& after) {
    return WaterFlowLaneRouteInputsEqual(before, after) &&
           before.trailWidthMeters == after.trailWidthMeters &&
           before.trailStreakLengthMeters == after.trailStreakLengthMeters &&
           before.speedMetersPerSecond != after.speedMetersPerSecond;
}

std::array<WaterRainIntensityPreset, 3> AllWaterRainIntensityPresets() {
    return {
        WaterRainIntensityPreset::LightMist,
        WaterRainIntensityPreset::Rain,
        WaterRainIntensityPreset::HeavyDownpour,
    };
}

std::string_view WaterRainIntensityPresetLabel(WaterRainIntensityPreset preset) {
    switch (preset) {
        case WaterRainIntensityPreset::LightMist:
            return "Light Mist";
        case WaterRainIntensityPreset::Rain:
            return "Rain";
        case WaterRainIntensityPreset::HeavyDownpour:
            return "Heavy Downpour";
    }
    return "Rain";
}

std::string_view WaterRainIntensityPresetNameForStorage(WaterRainIntensityPreset preset) {
    switch (preset) {
        case WaterRainIntensityPreset::LightMist:
            return "light_mist";
        case WaterRainIntensityPreset::Rain:
            return "rain";
        case WaterRainIntensityPreset::HeavyDownpour:
            return "heavy_downpour";
    }
    return "rain";
}

std::optional<WaterRainIntensityPreset> ParseWaterRainIntensityPresetName(std::string_view value) {
    if (value == "light_mist" || value == "Light Mist") {
        return WaterRainIntensityPreset::LightMist;
    }
    if (value == "rain" || value == "Rain") {
        return WaterRainIntensityPreset::Rain;
    }
    if (value == "heavy_downpour" || value == "Heavy Downpour") {
        return WaterRainIntensityPreset::HeavyDownpour;
    }
    return std::nullopt;
}

WaterRainSettings DefaultWaterRainSettings() {
    return DefaultRainRuntimeSettings();
}

WaterDynamicMeshFlowSettings DefaultWaterDynamicMeshFlowSettings() {
    return {};
}

WaterDynamicMeshFlowSettings SanitizeWaterDynamicMeshFlowSettings(
    WaterDynamicMeshFlowSettings settings) {
    const auto finiteOr = [](float value, float fallback) {
        return std::isfinite(value) ? value : fallback;
    };
    settings.cacheCellSizeMeters = std::clamp(
        finiteOr(settings.cacheCellSizeMeters, 0.08F),
        0.005F,
        5.0F);
    settings.projectionSearchRadiusMeters = std::clamp(
        finiteOr(settings.projectionSearchRadiusMeters, 1.25F),
        0.005F,
        25.0F);
    settings.ambiguityHeightMeters = std::clamp(
        finiteOr(settings.ambiguityHeightMeters, 0.18F),
        0.0F,
        25.0F);
    settings.particleCapacity = 4'096U;
    settings.historyLength = 24U;
    // Schema-45 Mesh Flow is always automatic. The legacy switch remains in
    // memory only so older project documents can be read without a bespoke
    // settings type.
    settings.automaticSources = true;
    settings.attractors.clear();
    settings.emitterMotions.clear();
    // The cache owns one physically compact highest-+X table. Legacy band
    // values stay readable, but schema-45 always uses the fixed plan band.
    settings.sourceBandWidthMeters = 0.75F;
    settings.sourceBandFraction = 0.04F;
    settings.dryConcavityFocus = std::clamp(
        finiteOr(settings.dryConcavityFocus, 0.90F),
        0.0F,
        1.0F);
    settings.edgeCoverage = std::clamp(
        finiteOr(settings.edgeCoverage, 0.0F),
        0.0F,
        1.0F);
    settings.activity = std::clamp(
        finiteOr(settings.activity, 1.0F),
        0.0F,
        1.0F);
    settings.rainGain = std::clamp(
        finiteOr(settings.rainGain, 0.0F),
        0.0F,
        4.0F);
    settings.moisturePersistenceMultiplier = std::clamp(
        finiteOr(settings.moisturePersistenceMultiplier, 1.0F),
        0.0F,
        8.0F);
    settings.rainRiseSeconds = std::clamp(
        finiteOr(settings.rainRiseSeconds, 0.0F),
        0.0F,
        86'400.0F);
    settings.rainRecessionSeconds = std::clamp(
        finiteOr(settings.rainRecessionSeconds, 0.0F),
        0.0F,
        86'400.0F);
    settings.surfaceSurge = std::clamp(
        finiteOr(settings.surfaceSurge, 0.6F),
        0.0F,
        1.0F);
    settings.rainSpawnSpread = std::clamp(
        finiteOr(settings.rainSpawnSpread, 0.75F),
        0.0F,
        4.0F);
    settings.rainDistributedSourceFraction = std::clamp(
        finiteOr(settings.rainDistributedSourceFraction, 0.55F),
        0.0F,
        1.0F);
    settings.previewParticleLimit = std::clamp(
        settings.previewParticleLimit,
        1U,
        100'000U);
    settings.finalParticleLimit = std::clamp(
        settings.finalParticleLimit,
        1U,
        250'000U);
    settings.trailLengthMeters = std::clamp(
        finiteOr(settings.trailLengthMeters, 18.0F),
        0.02F,
        100.0F);
    settings.stepMeters = std::clamp(
        finiteOr(settings.stepMeters, 0.12F),
        0.002F,
        5.0F);
    settings.trailWidthMeters = std::clamp(
        finiteOr(settings.trailWidthMeters, 0.0025F),
        0.0005F,
        1.0F);
    settings.trailStreakLengthMeters = std::clamp(
        finiteOr(settings.trailStreakLengthMeters, 0.030F),
        0.001F,
        5.0F);
    settings.surfaceOffsetMeters = std::clamp(
        finiteOr(settings.surfaceOffsetMeters, 0.003F),
        -1.0F,
        1.0F);
    settings.trailOpacityDry = std::clamp(
        finiteOr(settings.trailOpacityDry, 0.025F),
        0.0F,
        1.0F);
    settings.trailOpacityWet = std::clamp(
        finiteOr(settings.trailOpacityWet, 0.14F),
        0.0F,
        1.0F);
    settings.trailEmissionDry = std::clamp(
        finiteOr(settings.trailEmissionDry, 0.04F),
        0.0F,
        4.0F);
    settings.trailEmissionWet = std::clamp(
        finiteOr(settings.trailEmissionWet, 0.45F),
        0.0F,
        4.0F);
    settings.trailExposure = std::clamp(
        finiteOr(settings.trailExposure, 1.25F),
        0.0F,
        8.0F);
    settings.speedMetersPerSecond = std::clamp(
        finiteOr(settings.speedMetersPerSecond, 0.26F),
        0.001F,
        100.0F);
    settings.downhillWeight = std::clamp(
        finiteOr(settings.downhillWeight, 1.75F),
        0.0F,
        10.0F);
    settings.attractorWeight = std::clamp(
        finiteOr(settings.attractorWeight, 1.0F),
        0.0F,
        10.0F);
    settings.sourceVelocityWeight = std::clamp(
        finiteOr(settings.sourceVelocityWeight, 0.35F),
        0.0F,
        10.0F);
    settings.curlStrength = std::clamp(
        finiteOr(settings.curlStrength, 0.18F),
        0.0F,
        10.0F);
    settings.branchingStrength = std::clamp(
        finiteOr(settings.branchingStrength, 0.36F),
        0.0F,
        10.0F);
    settings.eddyStrength = std::clamp(
        finiteOr(settings.eddyStrength, 0.08F),
        0.0F,
        10.0F);
    settings.topologyResponse = std::clamp(
        finiteOr(settings.topologyResponse, 0.65F),
        0.0F,
        10.0F);
    settings.inertia = std::clamp(
        finiteOr(settings.inertia, 0.88F),
        0.0F,
        0.98F);
    settings.particleNoiseStrength = std::clamp(
        finiteOr(settings.particleNoiseStrength, 0.10F),
        0.0F,
        4.0F);
    settings.particleNoiseScaleMeters = std::clamp(
        finiteOr(settings.particleNoiseScaleMeters, 0.45F),
        0.001F,
        100.0F);
    settings.particleNoiseSpeed = std::clamp(
        finiteOr(settings.particleNoiseSpeed, 0.18F),
        0.0F,
        10.0F);
    settings.sharedWindStrength = std::clamp(
        finiteOr(settings.sharedWindStrength, 0.035F),
        0.0F,
        4.0F);
    settings.sharedWindScaleMeters = std::clamp(
        finiteOr(settings.sharedWindScaleMeters, 3.0F),
        0.001F,
        100.0F);
    settings.sharedWindSpeed = std::clamp(
        finiteOr(settings.sharedWindSpeed, 0.025F),
        0.0F,
        10.0F);
    settings.contactFadeSeconds = std::clamp(
        finiteOr(settings.contactFadeSeconds, 0.8F),
        0.0F,
        30.0F);
    settings.animationDurationSeconds = std::clamp(
        finiteOr(settings.animationDurationSeconds, 4.0F),
        0.0F,
        86'400.0F);

    const auto sanitizeColour = [&](invisible_places::io::Float3 value,
                                    invisible_places::io::Float3 fallback) {
        value.x = std::clamp(finiteOr(value.x, fallback.x), 0.0F, 1.0F);
        value.y = std::clamp(finiteOr(value.y, fallback.y), 0.0F, 1.0F);
        value.z = std::clamp(finiteOr(value.z, fallback.z), 0.0F, 1.0F);
        return value;
    };
    settings.rockResponse.radiusMeters = std::clamp(
        finiteOr(settings.rockResponse.radiusMeters, 0.12F),
        0.0F,
        0.75F);
    settings.rockResponse.opacityAdd = std::clamp(
        finiteOr(settings.rockResponse.opacityAdd, 0.16F),
        0.0F,
        4.0F);
    settings.rockResponse.emissionAdd = std::clamp(
        finiteOr(settings.rockResponse.emissionAdd, 0.35F),
        0.0F,
        4.0F);
    settings.rockResponse.colourise = sanitizeColour(
        settings.rockResponse.colourise,
        {0.18F, 0.42F, 0.55F});
    settings.rockResponse.colouriseAmount = std::clamp(
        finiteOr(settings.rockResponse.colouriseAmount, 0.45F),
        0.0F,
        1.0F);
    settings.rockResponse.persistenceSeconds = std::clamp(
        finiteOr(settings.rockResponse.persistenceSeconds, 2.5F),
        0.0F,
        30.0F);

    settings.vegetationResponse.radiusMeters = std::clamp(
        finiteOr(settings.vegetationResponse.radiusMeters, 0.18F),
        0.0F,
        0.75F);
    settings.vegetationResponse.opacityAdd = std::clamp(
        finiteOr(settings.vegetationResponse.opacityAdd, 0.14F),
        0.0F,
        4.0F);
    settings.vegetationResponse.emissionAdd = std::clamp(
        finiteOr(settings.vegetationResponse.emissionAdd, 0.55F),
        0.0F,
        4.0F);
    settings.vegetationResponse.colourise = sanitizeColour(
        settings.vegetationResponse.colourise,
        {0.18F, 0.55F, 0.48F});
    settings.vegetationResponse.colouriseAmount = std::clamp(
        finiteOr(settings.vegetationResponse.colouriseAmount, 0.50F),
        0.0F,
        1.0F);
    settings.vegetationResponse.persistenceSeconds = std::clamp(
        finiteOr(settings.vegetationResponse.persistenceSeconds, 3.0F),
        0.0F,
        30.0F);
    settings.vegetationResponse.twinkle = std::clamp(
        finiteOr(settings.vegetationResponse.twinkle, 1.4F),
        0.0F,
        4.0F);
    settings.vegetationResponse.streamDepthMeters = std::clamp(
        finiteOr(settings.vegetationResponse.streamDepthMeters, 0.45F),
        0.0F,
        2.0F);
    settings.contactUpwardReachMeters = std::clamp(
        finiteOr(settings.contactUpwardReachMeters, 0.6F),
        0.0F,
        3.0F);
    settings.trailWetnessFloor = std::clamp(
        finiteOr(settings.trailWetnessFloor, 0.75F),
        0.0F,
        1.0F);
    return settings;
}

WaterDynamicMeshFlowVisualWeights EvaluateWaterDynamicMeshFlowVisualWeights(
    const WaterDynamicMeshFlowSettings& rawSettings,
    float convergence,
    float moisture,
    float surfaceConfidence) {
    const auto settings = SanitizeWaterDynamicMeshFlowSettings(rawSettings);
    const auto clamp01 = [](float value) {
        return std::clamp(
            std::isfinite(value) ? value : 0.0F,
            0.0F,
            1.0F);
    };
    const auto smoothStep = [](float edge0, float edge1, float value) {
        const float amount = std::clamp(
            (value - edge0) / std::max(1.0e-6F, edge1 - edge0),
            0.0F,
            1.0F);
        return amount * amount * (3.0F - 2.0F * amount);
    };
    const auto lerp = [](float left, float right, float amount) {
        return left + ((right - left) * amount);
    };

    const float wet = clamp01(moisture);
    const float confidence = clamp01(surfaceConfidence);
    const float rill = smoothStep(0.18F, 0.82F, clamp01(convergence));

    // Give the top end of the authored focus control useful precision.  At the
    // default 0.75, a dry low-convergence cell accepts only about 8% of the
    // stable population; Rain progressively relaxes the gate to one.
    const float authoredFocus = clamp01(settings.dryConcavityFocus);
    const float focused = 1.0F - ((1.0F - authoredFocus) *
                                  (1.0F - authoredFocus));
    // Edge Coverage overrides the dry concentration: at 1 every rim cell
    // accepts spawns even in a dry scene, so trails reach the rock edge
    // along the whole +X section instead of only the likeliest rills.
    const float coverage = clamp01(settings.edgeCoverage);
    const float dryFocus = focused * (1.0F - wet) * (1.0F - coverage);
    const float concavityAcceptance = 0.02F + (0.98F * rill);

    WaterDynamicMeshFlowVisualWeights result;
    result.automaticSpawnAcceptance =
        lerp(1.0F, concavityAcceptance, dryFocus);

    // Convergent rills remain coherent while isolated trickles retain enough
    // individual and shared noise to avoid appearing as regular splines.
    const float rillCoherence = rill * lerp(0.68F, 0.92F, wet);
    result.directionalNoiseScale =
        lerp(1.0F, 0.25F, rillCoherence);

    // Population count plus the authored dry opacity/emission already make
    // dry flow sparse and faint. Keep enough coverage on the flat +X entry
    // band for those filaments to survive one-pixel rasterization, then make
    // convergent dry cells markedly more persistent. Rain broadens the
    // population and appearance without turning open Ground into a sheet.
    const float openProminence = 0.12F;
    const float rillProminence = lerp(0.90F, 0.70F, wet);
    result.trailProminence =
        lerp(openProminence, rillProminence, rill) *
        (0.68F + (0.32F * confidence));
    result.trailWidthScale =
        lerp(0.72F, 1.15F, rill) *
        lerp(0.90F, 1.04F, wet);
    result.trailStreakScale =
        lerp(0.78F, 1.05F, rill) *
        lerp(0.94F, 1.03F, wet);
    return result;
}

std::vector<WaterDynamicMeshFlowGroundEntry>
BuildWaterDynamicMeshFlowGroundEntries(const WaterSurfaceCache& cache) {
    // The spawn table follows the sampled Ground's +X rim wherever it curves.
    // Rim cells are walkable cells with no same-component cell in any +X
    // direction (the sampled surface simply ends there); every candidate then
    // stores its geodesic surface distance to the nearest rim cell.  A single
    // per-component maximum-X scalar would drop the whole rim wherever the
    // edge bends away from the component's global +X extreme.
    const auto& cells = cache.groundCells;
    if (cells.empty()) {
        return {};
    }
    const float resolution = std::max(0.001F, cache.resolutionMeters);
    constexpr std::uint32_t kInvalidIndex =
        std::numeric_limits<std::uint32_t>::max();

    // Neighbour probes run tens of millions of times on the cache-install
    // path, so retained cells are indexed by a dense grid over their compact
    // coordinate extent; a hash map only backs the pathological sparse case.
    std::int64_t minimumX = std::numeric_limits<std::int64_t>::max();
    std::int64_t minimumY = std::numeric_limits<std::int64_t>::max();
    std::int64_t maximumX = std::numeric_limits<std::int64_t>::min();
    std::int64_t maximumY = std::numeric_limits<std::int64_t>::min();
    for (const auto& cell : cells) {
        if (cell.componentId == 0U) {
            continue;
        }
        minimumX = std::min<std::int64_t>(minimumX, cell.cellX);
        minimumY = std::min<std::int64_t>(minimumY, cell.cellY);
        maximumX = std::max<std::int64_t>(maximumX, cell.cellX);
        maximumY = std::max<std::int64_t>(maximumY, cell.cellY);
    }
    if (maximumX < minimumX) {
        return {};
    }
    const std::int64_t extentX = maximumX - minimumX + 1LL;
    const std::int64_t extentY = maximumY - minimumY + 1LL;
    const bool useDenseGrid = extentX * extentY <= 100'000'000LL;
    std::vector<std::uint32_t> gridIndex;
    std::unordered_map<std::uint64_t, std::uint32_t> mapIndex;
    const auto packKey = [](std::int32_t x, std::int32_t y) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x))
                << 32U) |
               static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
    };
    if (useDenseGrid) {
        gridIndex.assign(
            static_cast<std::size_t>(extentX * extentY),
            kInvalidIndex);
    } else {
        mapIndex.reserve(cells.size());
    }
    const auto lookupCell = [&](std::int32_t x,
                                std::int32_t y) -> std::uint32_t {
        if (useDenseGrid) {
            if (x < minimumX || x > maximumX || y < minimumY ||
                y > maximumY) {
                return kInvalidIndex;
            }
            return gridIndex[static_cast<std::size_t>(
                (y - minimumY) * extentX + (x - minimumX))];
        }
        const auto found = mapIndex.find(packKey(x, y));
        return found != mapIndex.end() ? found->second : kInvalidIndex;
    };
    for (std::uint32_t index = 0U; index < cells.size(); ++index) {
        if (cells[index].componentId == 0U) {
            continue;
        }
        if (useDenseGrid) {
            gridIndex[static_cast<std::size_t>(
                (cells[index].cellY - minimumY) * extentX +
                (cells[index].cellX - minimumX))] = index;
        } else {
            mapIndex.emplace(
                packKey(cells[index].cellX, cells[index].cellY),
                index);
        }
    }

    // Distance propagates across every retained cell of a component — the
    // terminal rock skin is crossable at a cost premium, so a qualifying
    // bench ringed by contact cells is never stranded without sources.
    // Only vegetation-supported, non-terminal, connected cells become
    // entries.
    const auto walkable = [](const WaterGroundCell& cell) {
        return cell.componentId != 0U &&
               (cell.flags & kWaterGroundTerminalContactFlag) == 0U;
    };
    const auto qualifies = [&](const WaterGroundCell& cell) {
        return walkable(cell) &&
               (cell.flags & kWaterGroundVegetationSupportedFlag) != 0U &&
               cell.connectivityMask != 0U;
    };

    struct ComponentInfo {
        std::int32_t qualifyingMaximumX =
            std::numeric_limits<std::int32_t>::min();
        std::uint32_t maximumCost = 0U;
        bool hasRim = false;
    };
    std::uint32_t maximumComponent = 0U;
    for (const auto& cell : cells) {
        maximumComponent = std::max(maximumComponent, cell.componentId);
    }
    std::vector<ComponentInfo> components(
        static_cast<std::size_t>(maximumComponent) + 1U);

    static constexpr std::array<std::pair<std::int32_t, std::int32_t>, 3>
        kPositiveXNeighbours{{{1, 1}, {1, 0}, {1, -1}}};
    const auto hasPositiveXNeighbour = [&](const WaterGroundCell& cell) {
        for (const auto& offset : kPositiveXNeighbours) {
            const auto neighbourIndex = lookupCell(
                cell.cellX + offset.first,
                cell.cellY + offset.second);
            if (neighbourIndex != kInvalidIndex &&
                cells[neighbourIndex].componentId == cell.componentId) {
                return true;
            }
        }
        return false;
    };

    std::vector<std::uint32_t> rimSeeds;
    for (std::uint32_t index = 0U; index < cells.size(); ++index) {
        const auto& cell = cells[index];
        if (!walkable(cell)) {
            continue;
        }
        if (qualifies(cell)) {
            auto& component = components[cell.componentId];
            component.qualifyingMaximumX =
                std::max(component.qualifyingMaximumX, cell.cellX);
        }
        if (!hasPositiveXNeighbour(cell)) {
            rimSeeds.push_back(index);
            components[cell.componentId].hasRim = true;
        }
    }
    // A component whose entire +X boundary is terminal rock skin has no free
    // rim; fall back to its highest-+X qualifying column so it still spawns.
    for (std::uint32_t index = 0U; index < cells.size(); ++index) {
        const auto& cell = cells[index];
        if (!qualifies(cell)) {
            continue;
        }
        const auto& component = components[cell.componentId];
        if (!component.hasRim &&
            cell.cellX == component.qualifyingMaximumX) {
            rimSeeds.push_back(index);
        }
    }

    // Multi-source Dijkstra over each component's retained cells with
    // {10, 14} step costs (orthogonal, diagonal) in tenth-of-cell units;
    // stepping onto a terminal contact cell costs three times as much, so
    // rock skin is a detour rather than a wall.  A 64-slot circular bucket
    // queue keeps the transform linear in the cell count.
    constexpr std::uint32_t kUnreachableCost =
        std::numeric_limits<std::uint32_t>::max();
    constexpr std::uint32_t kOrthogonalStepCost = 10U;
    constexpr std::uint32_t kDiagonalStepCost = 14U;
    constexpr std::uint32_t kTerminalStepCostMultiplier = 3U;
    std::vector<std::uint32_t> cellCost(cells.size(), kUnreachableCost);
    std::vector<bool> settled(cells.size(), false);
    std::array<std::vector<std::uint32_t>, 64> buckets;
    std::size_t pendingCount = 0U;
    const auto pushCost = [&](std::uint32_t index, std::uint32_t cost) {
        cellCost[index] = cost;
        buckets[cost & 63U].push_back(index);
        ++pendingCount;
    };
    for (const auto index : rimSeeds) {
        if (cellCost[index] != 0U) {
            pushCost(index, 0U);
        }
    }
    static constexpr std::array<std::pair<std::int32_t, std::int32_t>, 8>
        kAllNeighbours{{
            {0, 1},
            {1, 1},
            {1, 0},
            {1, -1},
            {0, -1},
            {-1, -1},
            {-1, 0},
            {-1, 1},
        }};
    std::vector<std::uint32_t> activeBucket;
    for (std::uint32_t currentCost = 0U; pendingCount > 0U; ++currentCost) {
        auto& bucket = buckets[currentCost & 63U];
        activeBucket.clear();
        activeBucket.swap(bucket);
        pendingCount -= activeBucket.size();
        for (const auto index : activeBucket) {
            if (settled[index] || cellCost[index] != currentCost) {
                continue;
            }
            settled[index] = true;
            const auto& cell = cells[index];
            for (const auto& offset : kAllNeighbours) {
                const auto neighbourIndex = lookupCell(
                    cell.cellX + offset.first,
                    cell.cellY + offset.second);
                if (neighbourIndex == kInvalidIndex ||
                    settled[neighbourIndex] ||
                    cells[neighbourIndex].componentId !=
                        cell.componentId) {
                    continue;
                }
                const bool diagonal =
                    offset.first != 0 && offset.second != 0;
                std::uint32_t stepCost =
                    diagonal ? kDiagonalStepCost : kOrthogonalStepCost;
                if (!walkable(cells[neighbourIndex])) {
                    stepCost *= kTerminalStepCostMultiplier;
                }
                const std::uint32_t nextCost = currentCost + stepCost;
                if (nextCost < cellCost[neighbourIndex]) {
                    pushCost(neighbourIndex, nextCost);
                }
            }
        }
    }

    // Order by 0.10 m distance bands, hash-shuffled within each band, so the
    // GPU sampler's cubic bias toward index 0 becomes a bias toward the rim
    // that is spatially uniform along it — never a bias toward one end of
    // the edge. Exact distances stay in the entry for CPU consumers.  Sort
    // keys are precomputed; recomputing hashes inside the comparator costs
    // hundreds of milliseconds at millions of candidates.
    const auto spreadHash = [](std::int32_t x, std::int32_t y) {
        auto hash = static_cast<std::uint32_t>(x) * 0x9E3779B9U;
        hash ^= static_cast<std::uint32_t>(y) * 0x85EBCA6BU;
        hash ^= hash >> 16U;
        hash *= 0x7FEB352DU;
        hash ^= hash >> 15U;
        hash *= 0x846CA68BU;
        return hash ^ (hash >> 16U);
    };
    struct EntryCandidate {
        std::uint64_t sortKey = 0U;
        std::int32_t cellX = 0;
        std::int32_t cellY = 0;
        std::uint32_t cellIndex = 0U;
        std::uint32_t cost = 0U;
    };
    std::vector<EntryCandidate> candidates;
    candidates.reserve(std::min<std::size_t>(cells.size(), 262'144U));
    for (std::uint32_t index = 0U; index < cells.size(); ++index) {
        const auto& cell = cells[index];
        if (!qualifies(cell) || cellCost[index] == kUnreachableCost) {
            continue;
        }
        const std::uint32_t band = cellCost[index] / 100U;
        candidates.push_back({
            .sortKey = (static_cast<std::uint64_t>(band) << 32U) |
                       spreadHash(cell.cellX, cell.cellY),
            .cellX = cell.cellX,
            .cellY = cell.cellY,
            .cellIndex = index,
            .cost = cellCost[index],
        });
        auto& component = components[cell.componentId];
        component.maximumCost =
            std::max(component.maximumCost, cellCost[index]);
    }
    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const EntryCandidate& left, const EntryCandidate& right) {
            return std::tie(left.sortKey, left.cellX, left.cellY) <
                   std::tie(right.sortKey, right.cellX, right.cellY);
        });

    // Cap the table without abandoning the interior: the nearest half of the
    // cap is kept intact and the remainder is stride-sampled so spawns can
    // still start anywhere on the smooth top surface, just more sparsely.
    constexpr std::size_t kMaximumEntryCount = 262'144U;
    constexpr std::size_t kNearBandKeepCount = kMaximumEntryCount / 2U;
    if (candidates.size() > kMaximumEntryCount) {
        std::vector<EntryCandidate> thinned;
        thinned.reserve(kMaximumEntryCount);
        thinned.assign(
            candidates.begin(),
            candidates.begin() +
                static_cast<std::ptrdiff_t>(kNearBandKeepCount));
        const std::size_t tailCount =
            candidates.size() - kNearBandKeepCount;
        const std::size_t tailKeepCount =
            kMaximumEntryCount - kNearBandKeepCount;
        for (std::size_t keep = 0U; keep < tailKeepCount; ++keep) {
            const std::size_t tailIndex =
                (keep * tailCount) / tailKeepCount;
            thinned.push_back(candidates[kNearBandKeepCount + tailIndex]);
        }
        candidates = std::move(thinned);
    }

    std::vector<WaterDynamicMeshFlowGroundEntry> result;
    result.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        const auto& cell = cells[candidate.cellIndex];
        const auto maximumCost =
            components[cell.componentId].maximumCost;
        result.push_back({
            .cellX = cell.cellX,
            .cellY = cell.cellY,
            .edgeDistanceMeters =
                static_cast<float>(candidate.cost) * resolution / 10.0F,
            .edgeDistanceFraction = maximumCost > 0U
                ? static_cast<float>(candidate.cost) /
                      static_cast<float>(maximumCost)
                : 0.0F,
        });
    }
    return result;
}

std::array<WaterDynamicMeshParticlePreset, 4> AllWaterDynamicMeshParticlePresets() {
    return {{
        {.name = "Default", .label = "Default"},
        {.name = "Laminar", .label = "Laminar"},
        {.name = "Branching", .label = "Branching"},
        {.name = "Turbulent", .label = "Turbulent"},
    }};
}

std::string_view NormalizeWaterDynamicMeshParticlePresetName(std::string_view presetName) {
    for (const auto& preset : AllWaterDynamicMeshParticlePresets()) {
        if (presetName == preset.name || presetName == preset.label) {
            return preset.name;
        }
    }
    if (presetName == "laminar") {
        return "Laminar";
    }
    if (presetName == "branching") {
        return "Branching";
    }
    if (presetName == "turbulent") {
        return "Turbulent";
    }
    return "Default";
}

WaterDynamicMeshFlowSettings ApplyWaterDynamicMeshParticlePreset(
    WaterDynamicMeshFlowSettings settings,
    std::string_view presetName) {
    const auto normalized = NormalizeWaterDynamicMeshParticlePresetName(presetName);
    settings.particlePresetName = std::string{normalized};
    if (normalized == "Laminar") {
        settings.previewParticleLimit = 360U;
        settings.finalParticleLimit = 1600U;
        settings.trailLengthMeters = 20.0F;
        settings.stepMeters = 0.13F;
        settings.speedMetersPerSecond = 0.55F;
        settings.downhillWeight = 1.70F;
        settings.attractorWeight = 0.65F;
        settings.sourceVelocityWeight = 0.18F;
        settings.curlStrength = 0.035F;
        settings.branchingStrength = 0.08F;
        settings.eddyStrength = 0.015F;
        settings.topologyResponse = 0.45F;
        settings.inertia = 0.86F;
        return settings;
    }
    if (normalized == "Branching") {
        settings.previewParticleLimit = 760U;
        settings.finalParticleLimit = 3200U;
        settings.trailLengthMeters = 20.0F;
        settings.stepMeters = 0.11F;
        settings.speedMetersPerSecond = 0.66F;
        settings.downhillWeight = 1.18F;
        settings.attractorWeight = 1.05F;
        settings.sourceVelocityWeight = 0.42F;
        settings.curlStrength = 0.28F;
        settings.branchingStrength = 1.05F;
        settings.eddyStrength = 0.10F;
        settings.topologyResponse = 0.95F;
        settings.inertia = 0.48F;
        return settings;
    }
    if (normalized == "Turbulent") {
        settings.previewParticleLimit = 900U;
        settings.finalParticleLimit = 3800U;
        settings.trailLengthMeters = 18.0F;
        settings.stepMeters = 0.10F;
        settings.speedMetersPerSecond = 0.72F;
        settings.downhillWeight = 0.96F;
        settings.attractorWeight = 1.15F;
        settings.sourceVelocityWeight = 0.50F;
        settings.curlStrength = 0.82F;
        settings.branchingStrength = 0.78F;
        settings.eddyStrength = 0.56F;
        settings.topologyResponse = 1.15F;
        settings.inertia = 0.24F;
        return settings;
    }
    settings.previewParticleLimit = 560U;
    settings.finalParticleLimit = 2400U;
    settings.trailLengthMeters = 18.0F;
    settings.stepMeters = 0.12F;
    settings.speedMetersPerSecond = 0.22F;
    settings.surfaceOffsetMeters = 0.006F;
    settings.downhillWeight = 1.35F;
    settings.attractorWeight = 1.0F;
    settings.sourceVelocityWeight = 0.35F;
    settings.curlStrength = 0.18F;
    settings.branchingStrength = 0.36F;
    settings.eddyStrength = 0.08F;
    settings.topologyResponse = 0.65F;
    settings.inertia = 0.76F;
    return settings;
}

float WaterRainPresetVisualStrength(WaterRainIntensityPreset preset) {
    switch (preset) {
        case WaterRainIntensityPreset::LightMist:
            return 0.30F;
        case WaterRainIntensityPreset::Rain:
            return 0.68F;
        case WaterRainIntensityPreset::HeavyDownpour:
            return 1.0F;
    }
    return 0.68F;
}

namespace {

constexpr float kSeepagePi = 3.14159265358979323846F;
constexpr float kSeepageTwoPi = 2.0F * kSeepagePi;
constexpr float kSeepageMaximumRainReachScale = 1.25F;
constexpr float kSeepageMaximumRainWidthScale = 1.20F;
constexpr float kSeepageMaximumScenarioReachScale = 1.50F;
constexpr float kSeepageMaximumScenarioWidthScale = 1.35F;
constexpr float kSeepageMaximumReachScale =
    kSeepageMaximumRainReachScale * kSeepageMaximumScenarioReachScale;
constexpr float kSeepageMaximumWidthScale =
    kSeepageMaximumRainWidthScale * kSeepageMaximumScenarioWidthScale;

struct MutableSeepageHashCell {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;
    std::array<std::uint32_t, WaterSeepageSpatialGrid::kMaxReferencesPerCell> references{};
    std::uint32_t referenceCount = 0U;
    bool occupied = false;
    bool overflowed = false;
};

float SeepageFiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

WaterEffectResponseSettings SanitizeSeepageResponse(
    WaterEffectResponseSettings response,
    const WaterEffectResponseSettings& fallback) {
    response.intensity = std::clamp(SeepageFiniteOr(response.intensity, fallback.intensity), 0.0F, 8.0F);
    response.emissionAdd = std::clamp(SeepageFiniteOr(response.emissionAdd, fallback.emissionAdd), 0.0F, 16.0F);
    response.opacityAdd = std::clamp(SeepageFiniteOr(response.opacityAdd, fallback.opacityAdd), -1.0F, 4.0F);
    response.opacityMultiply = std::clamp(
        SeepageFiniteOr(response.opacityMultiply, fallback.opacityMultiply),
        0.0F,
        16.0F);
    response.pointSizeAdd = std::clamp(
        SeepageFiniteOr(response.pointSizeAdd, fallback.pointSizeAdd),
        -256.0F,
        512.0F);
    response.pointSizeMultiply = std::clamp(
        SeepageFiniteOr(response.pointSizeMultiply, fallback.pointSizeMultiply),
        0.0F,
        16.0F);
    response.hueShift = SeepageFiniteOr(response.hueShift, fallback.hueShift);
    response.colouriseRed = std::clamp(
        SeepageFiniteOr(response.colouriseRed, fallback.colouriseRed),
        0.0F,
        1.0F);
    response.colouriseGreen = std::clamp(
        SeepageFiniteOr(response.colouriseGreen, fallback.colouriseGreen),
        0.0F,
        1.0F);
    response.colouriseBlue = std::clamp(
        SeepageFiniteOr(response.colouriseBlue, fallback.colouriseBlue),
        0.0F,
        1.0F);
    response.colouriseAmount = std::clamp(
        SeepageFiniteOr(response.colouriseAmount, fallback.colouriseAmount),
        0.0F,
        1.0F);
    response.gaussianSharpnessBias = std::clamp(
        SeepageFiniteOr(response.gaussianSharpnessBias, fallback.gaussianSharpnessBias),
        -8.0F,
        8.0F);
    return response;
}

WaterSeepageLookSettings SanitizeSeepageLook(WaterSeepageLookSettings look) {
    const WaterSeepageLookSettings fallback{};
    switch (look.pattern) {
        case WaterSeepagePattern::WetRockSheen:
        case WaterSeepagePattern::ChaoticBloom:
        case WaterSeepagePattern::WettingTrickle:
        case WaterSeepagePattern::ContourPulses:
            break;
        default:
            look.pattern = WaterSeepagePattern::ChaoticBloom;
            break;
    }
    look.baseWetness = std::clamp(SeepageFiniteOr(look.baseWetness, fallback.baseWetness), 0.0F, 1.0F);
    look.density = std::clamp(SeepageFiniteOr(look.density, fallback.density), 0.0F, 1.0F);
    look.glisten = std::clamp(SeepageFiniteOr(look.glisten, fallback.glisten), 0.0F, 1.0F);
    look.rainResponse = std::clamp(SeepageFiniteOr(look.rainResponse, fallback.rainResponse), 0.0F, 1.0F);
    look.featureSizeMeters = std::clamp(
        SeepageFiniteOr(look.featureSizeMeters, fallback.featureSizeMeters),
        0.005F,
        20.0F);
    look.contrast = std::clamp(SeepageFiniteOr(look.contrast, fallback.contrast), 0.0F, 1.0F);
    look.evolution = std::clamp(SeepageFiniteOr(look.evolution, fallback.evolution), 0.0F, 4.0F);
    look.roughness = std::clamp(SeepageFiniteOr(look.roughness, fallback.roughness), 0.02F, 1.0F);
    look.angleResponse = std::clamp(
        SeepageFiniteOr(look.angleResponse, fallback.angleResponse), 0.0F, 1.0F);
    look.microNormalStrength = std::clamp(
        SeepageFiniteOr(look.microNormalStrength, fallback.microNormalStrength), 0.0F, 1.0F);
    look.glintDensity = std::clamp(
        SeepageFiniteOr(look.glintDensity, fallback.glintDensity), 0.0F, 1.0F);
    look.environmentAzimuthDegrees = std::remainder(
        SeepageFiniteOr(look.environmentAzimuthDegrees, fallback.environmentAzimuthDegrees),
        360.0F);
    look.environmentElevationDegrees = std::clamp(
        SeepageFiniteOr(look.environmentElevationDegrees, fallback.environmentElevationDegrees),
        -89.0F,
        89.0F);
    look.curl = std::clamp(SeepageFiniteOr(look.curl, fallback.curl), 0.0F, 2.0F);
    look.breakup = std::clamp(SeepageFiniteOr(look.breakup, fallback.breakup), 0.0F, 1.0F);
    look.downhillDriftMetersPerSecond = std::clamp(
        SeepageFiniteOr(
            look.downhillDriftMetersPerSecond,
            fallback.downhillDriftMetersPerSecond),
        0.0F,
        4.0F);
    look.tricklePatchSizeMeters = std::clamp(
        SeepageFiniteOr(look.tricklePatchSizeMeters, fallback.tricklePatchSizeMeters),
        0.005F,
        20.0F);
    look.trickleWidthMeters = std::clamp(
        SeepageFiniteOr(look.trickleWidthMeters, fallback.trickleWidthMeters),
        0.001F,
        100.0F);
    look.trickleFrontSoftness = std::clamp(
        SeepageFiniteOr(look.trickleFrontSoftness, fallback.trickleFrontSoftness),
        0.001F,
        10.0F);
    look.pulseSpacingMeters = std::clamp(
        SeepageFiniteOr(look.pulseSpacingMeters, fallback.pulseSpacingMeters),
        0.005F,
        20.0F);
    look.pulseWidthMeters = std::clamp(
        SeepageFiniteOr(look.pulseWidthMeters, fallback.pulseWidthMeters),
        0.001F,
        10.0F);
    look.pulseSpeedMetersPerSecond = std::clamp(
        SeepageFiniteOr(
            look.pulseSpeedMetersPerSecond,
            fallback.pulseSpeedMetersPerSecond),
        0.0F,
        4.0F);
    look.pulseIrregularity = std::clamp(
        SeepageFiniteOr(look.pulseIrregularity, fallback.pulseIrregularity),
        0.0F,
        1.0F);
    look.pulseWaveCount = std::clamp(
        SeepageFiniteOr(look.pulseWaveCount, fallback.pulseWaveCount),
        1.0F,
        12.0F);
    look.pulseSpeedVariation = std::clamp(
        SeepageFiniteOr(
            look.pulseSpeedVariation,
            fallback.pulseSpeedVariation),
        0.0F,
        1.0F);
    look.response = SanitizeSeepageResponse(look.response, fallback.response);
    return look;
}

std::string_view TrimSeepageName(std::string_view value) {
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())) != 0) {
        value.remove_prefix(1U);
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
        value.remove_suffix(1U);
    }
    return value;
}

std::string NormalizeSeepageRole(std::string_view role) {
    std::string normalized;
    normalized.reserve(role.size());
    for (const char character : role) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) != 0) {
            normalized.push_back(static_cast<char>(std::tolower(byte)));
        }
    }
    if (normalized == "vegetation") {
        return "veg";
    }
    return normalized;
}

bool SeepageNodeTargetsRole(const WaterSeepageNode& node, std::string_view targetRole) {
    if (node.targetSceneRoles.empty()) {
        return false;
    }
    if (targetRole.empty()) {
        return true;
    }
    const auto normalizedTarget = NormalizeSeepageRole(targetRole);
    return std::any_of(
        node.targetSceneRoles.begin(),
        node.targetSceneRoles.end(),
        [&](const std::string& role) {
            return NormalizeSeepageRole(role) == normalizedTarget;
        });
}

glm::vec3 SafeSeepageNormal(const invisible_places::io::Float3& value) {
    const glm::vec3 normal = ToGlm(value);
    if (!IsValidPoint(normal) || glm::dot(normal, normal) <= kNormalEpsilon) {
        return {0.0F, 1.0F, 0.0F};
    }
    return glm::normalize(normal);
}

std::uint32_t SeepageSpatialHash(std::int32_t x, std::int32_t y, std::int32_t z) {
    std::uint32_t hash =
        (static_cast<std::uint32_t>(x) * 0x8da6b343U) ^
        (static_cast<std::uint32_t>(y) * 0xd8163841U) ^
        (static_cast<std::uint32_t>(z) * 0xcb1ab31fU);
    hash ^= hash >> 16U;
    hash *= 0x7feb352dU;
    hash ^= hash >> 15U;
    hash *= 0x846ca68bU;
    hash ^= hash >> 16U;
    return hash;
}

bool SeepageCellMatches(
    const MutableSeepageHashCell& cell,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z) {
    return cell.occupied && cell.x == x && cell.y == y && cell.z == z;
}

std::size_t SeepageHashSlot(
    std::span<const MutableSeepageHashCell> cells,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z) {
    if (cells.empty()) {
        return 0U;
    }
    const auto mask = cells.size() - 1U;
    std::size_t slot = static_cast<std::size_t>(SeepageSpatialHash(x, y, z)) & mask;
    for (std::size_t probe = 0; probe < cells.size(); ++probe) {
        if (!cells[slot].occupied || SeepageCellMatches(cells[slot], x, y, z)) {
            return slot;
        }
        slot = (slot + 1U) & mask;
    }
    return cells.size();
}

void RehashSeepageCells(std::vector<MutableSeepageHashCell>* cells, std::size_t newCapacity) {
    if (cells == nullptr) {
        return;
    }
    newCapacity = std::max<std::size_t>(16U, newCapacity);
    std::size_t powerOfTwo = 1U;
    while (powerOfTwo < newCapacity) {
        powerOfTwo *= 2U;
    }
    std::vector<MutableSeepageHashCell> replacement(powerOfTwo);
    for (const auto& cell : *cells) {
        if (!cell.occupied) {
            continue;
        }
        const auto slot = SeepageHashSlot(replacement, cell.x, cell.y, cell.z);
        replacement[slot] = cell;
    }
    *cells = std::move(replacement);
}

bool SeepageNodeReferencePreferred(
    std::uint32_t leftIndex,
    std::uint32_t rightIndex,
    std::span<const WaterSeepageRuntimeNode> nodes,
    std::int32_t cellX,
    std::int32_t cellY,
    std::int32_t cellZ,
    float cellSizeMeters) {
    if (leftIndex >= nodes.size()) {
        return false;
    }
    if (rightIndex >= nodes.size()) {
        return true;
    }
    const auto& left = nodes[leftIndex];
    const auto& right = nodes[rightIndex];
    // Candidate membership is immutable topology. Rank only by authored
    // geometry and stable identity so strength, visibility, seed, look, and
    // animation edits cannot change an overflowing cell's retained set.
    const glm::vec3 cellCenter{
        (static_cast<float>(cellX) + 0.5F) * cellSizeMeters,
        (static_cast<float>(cellY) + 0.5F) * cellSizeMeters,
        (static_cast<float>(cellZ) + 0.5F) * cellSizeMeters,
    };
    const float leftDistanceSquared = glm::dot(left.position - cellCenter, left.position - cellCenter);
    const float rightDistanceSquared = glm::dot(right.position - cellCenter, right.position - cellCenter);
    if (std::abs(leftDistanceSquared - rightDistanceSquared) > 1.0e-8F) {
        return leftDistanceSquared < rightDistanceSquared;
    }
    if (left.id != right.id) {
        return left.id < right.id;
    }
    return leftIndex < rightIndex;
}

void InsertSeepageCellReference(
    std::vector<MutableSeepageHashCell>* cells,
    std::size_t* occupiedCount,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z,
    std::uint32_t nodeIndex,
    std::span<const WaterSeepageRuntimeNode> nodes,
    float cellSizeMeters,
    WaterSeepageSpatialGridDiagnostics* diagnostics) {
    if (cells == nullptr || occupiedCount == nullptr || diagnostics == nullptr) {
        return;
    }
    if (cells->empty()) {
        cells->resize(16U);
    }
    auto slot = SeepageHashSlot(*cells, x, y, z);
    const bool insertingCell = slot >= cells->size() || !cells->at(slot).occupied;
    if (insertingCell && ((*occupiedCount + 1U) * 10U >= cells->size() * 7U)) {
        RehashSeepageCells(cells, cells->size() * 2U);
        slot = SeepageHashSlot(*cells, x, y, z);
    }
    if (slot >= cells->size()) {
        return;
    }
    auto& cell = cells->at(slot);
    if (!cell.occupied) {
        cell.x = x;
        cell.y = y;
        cell.z = z;
        cell.occupied = true;
        ++*occupiedCount;
    }
    for (std::uint32_t index = 0U; index < cell.referenceCount; ++index) {
        if (cell.references[index] == nodeIndex) {
            return;
        }
    }
    if (cell.referenceCount < WaterSeepageSpatialGrid::kMaxReferencesPerCell) {
        cell.references[cell.referenceCount++] = nodeIndex;
        return;
    }
    if (!cell.overflowed) {
        cell.overflowed = true;
        ++diagnostics->overflowCellCount;
    }
    ++diagnostics->droppedNodeReferenceCount;
    std::uint32_t leastPreferredSlot = 0U;
    for (std::uint32_t index = 1U; index < cell.referenceCount; ++index) {
        if (SeepageNodeReferencePreferred(
                cell.references[leastPreferredSlot],
                cell.references[index],
                nodes,
                x,
                y,
                z,
                cellSizeMeters)) {
            leastPreferredSlot = index;
        }
    }
    if (SeepageNodeReferencePreferred(
            nodeIndex,
            cell.references[leastPreferredSlot],
            nodes,
            x,
            y,
            z,
            cellSizeMeters)) {
        cell.references[leastPreferredSlot] = nodeIndex;
    }
}

float SeepageRainGain(const WaterSeepageRuntimeNode& node) {
    return Clamp01(node.rainVisualStrength * node.look.rainResponse);
}

const WaterSeepageSurfaceGuide* FindWaterSeepageSurfaceGuide(
    std::span<const WaterSeepageSurfaceGuide> guides,
    std::uint32_t nodeId) {
    const auto guide = std::find_if(
        guides.begin(),
        guides.end(),
        [nodeId](const WaterSeepageSurfaceGuide& candidate) {
            return candidate.nodeId == nodeId;
        });
    return guide == guides.end() ? nullptr : &*guide;
}

void CopyWaterSeepageSurfaceGuide(
    const WaterSeepageSurfaceGuide* guide,
    WaterSeepageRuntimeNode* runtime) {
    if (guide == nullptr || runtime == nullptr || !guide->valid || guide->sampleCount < 2U) {
        return;
    }

    const auto sourceCount = std::min<std::size_t>(
        guide->sampleCount,
        kWaterSeepageMaximumGuideSamples);
    float firstStation = 0.0F;
    float previousStation = -1.0F;
    glm::vec3 previousNormal = runtime->surfaceNormal;
    for (std::size_t sourceIndex = 0U; sourceIndex < sourceCount; ++sourceIndex) {
        const auto& source = guide->samples[sourceIndex];
        const glm::vec3 position = ToGlm(source.position);
        if (!IsValidPoint(position)) {
            break;
        }
        if (sourceIndex == 0U) {
            firstStation = std::isfinite(source.station) ? source.station : 0.0F;
        }
        float station = std::isfinite(source.station)
                            ? std::max(0.0F, source.station - firstStation)
                            : 0.0F;
        if (runtime->guideSampleCount > 0U && station <= previousStation + 1.0e-5F) {
            const glm::vec3 previousPosition = ToGlm(
                runtime->guideSamples[runtime->guideSampleCount - 1U].position);
            const float segmentLength = glm::length(position - previousPosition);
            if (segmentLength <= 1.0e-5F) {
                continue;
            }
            station = previousStation + segmentLength;
        }

        glm::vec3 normal = SafeSeepageNormal(source.normal);
        if (glm::dot(normal, previousNormal) < 0.0F) {
            normal = -normal;
        }
        auto& target = runtime->guideSamples[runtime->guideSampleCount++];
        target.position = source.position;
        target.normal = FromGlm(normal);
        target.station = station;
        target.confidence = std::clamp(
            SeepageFiniteOr(source.confidence, 1.0F),
            0.0F,
            1.0F);
        previousStation = station;
        previousNormal = normal;
    }

    if (runtime->guideSampleCount < 2U) {
        runtime->guideSampleCount = 0U;
        return;
    }
    const float lastStation =
        runtime->guideSamples[runtime->guideSampleCount - 1U].station;
    const float declaredAchieved = std::max(
        0.0F,
        SeepageFiniteOr(guide->achievedReachMeters, lastStation));
    runtime->guideRequestedReachMeters = std::max(
        lastStation,
        SeepageFiniteOr(guide->requestedReachMeters, lastStation));
    runtime->guideAchievedReachMeters = std::min(lastStation, declaredAchieved);
    if (runtime->guideAchievedReachMeters <= 1.0e-5F) {
        runtime->guideSampleCount = 0U;
        runtime->guideRequestedReachMeters = 0.0F;
        runtime->guideAchievedReachMeters = 0.0F;
        return;
    }
    runtime->guideValid = true;
    runtime->guideComplete = guide->complete &&
                             runtime->guideAchievedReachMeters + 1.0e-4F >=
                                 runtime->guideRequestedReachMeters;
}

glm::vec3 WaterSeepageGuideTangent(
    const WaterSeepageRuntimeNode& node,
    std::size_t sampleIndex) {
    if (!node.guideValid || node.guideSampleCount < 2U || sampleIndex >= node.guideSampleCount) {
        return node.downAxis;
    }
    const auto previousIndex = sampleIndex > 0U ? sampleIndex - 1U : sampleIndex;
    const auto nextIndex = std::min<std::size_t>(sampleIndex + 1U, node.guideSampleCount - 1U);
    glm::vec3 tangent = ToGlm(node.guideSamples[nextIndex].position) -
                        ToGlm(node.guideSamples[previousIndex].position);
    if (!IsValidPoint(tangent) || glm::dot(tangent, tangent) <= kNormalEpsilon) {
        tangent = node.downAxis;
    }
    return glm::normalize(tangent);
}

invisible_places::io::Bounds3f SeepageRuntimeNodeBounds(const WaterSeepageRuntimeNode& node) {
    invisible_places::io::Bounds3f bounds;
    const float feather = std::max(0.0F, node.edgeFeatherMeters);
    // The bounds must contain the area envelope's maximum over every live
    // parameter value, because live edits never re-rasterize hash cells. The
    // envelope clamps its run to the selection reach limit and its half-width
    // to half the selection width limit, and its feathers scale with those
    // values (0.10x run, 0.30x half-width), so those maxima are covered here.
    const float halfWidthLimit = std::max(0.0F, node.selectionWidthLimitMeters) * 0.5F;
    const float reachLimit = std::max(0.0F, node.selectionReachLimitMeters);
    const float lateralExtent =
        halfWidthLimit + std::max(feather, halfWidthLimit * 0.30F);
    const float tailExtent = std::max(feather, reachLimit * 0.10F);
    const float depthExtent = node.depthToleranceMeters + feather;
    if (node.guideValid && node.guideSampleCount >= 2U) {
        for (std::size_t sampleIndex = 0U;
             sampleIndex < node.guideSampleCount;
             ++sampleIndex) {
            const auto& sample = node.guideSamples[sampleIndex];
            const glm::vec3 position = ToGlm(sample.position);
            const glm::vec3 normal = SafeSeepageNormal(sample.normal);
            const glm::vec3 tangent = WaterSeepageGuideTangent(node, sampleIndex);
            glm::vec3 lateral = glm::cross(normal, tangent);
            if (!IsValidPoint(lateral) || glm::dot(lateral, lateral) <= kNormalEpsilon) {
                lateral = node.lateralAxis;
            } else {
                lateral = glm::normalize(lateral);
            }
            if (glm::dot(lateral, node.lateralAxis) < 0.0F) {
                lateral = -lateral;
            }
            const glm::vec3 extent =
                glm::abs(lateral) * lateralExtent +
                glm::abs(normal) * depthExtent +
                glm::abs(tangent) * tailExtent;
            bounds.Expand(FromGlm(position - extent));
            bounds.Expand(FromGlm(position + extent));
        }
        return bounds;
    }
    const std::array<float, 2> longitudinal{
        -feather,
        reachLimit + tailExtent};
    const std::array<float, 2> lateral{-lateralExtent, lateralExtent};
    const std::array<float, 2> depth{-depthExtent, depthExtent};
    for (const float along : longitudinal) {
        for (const float across : lateral) {
            for (const float offset : depth) {
                bounds.Expand(FromGlm(
                    node.position +
                    node.downAxis * along +
                    node.lateralAxis * across +
                    node.surfaceNormal * offset));
            }
        }
    }
    return bounds;
}

std::int32_t SeepageCellCoordinate(float value, float cellSize) {
    const double scaled = std::floor(
        static_cast<double>(value) / static_cast<double>(std::max(1.0e-4F, cellSize)));
    return static_cast<std::int32_t>(std::clamp(
        scaled,
        static_cast<double>(std::numeric_limits<std::int32_t>::min()),
        static_cast<double>(std::numeric_limits<std::int32_t>::max())));
}

float SeepageHash01(std::int32_t x, std::int32_t y, std::uint32_t seed) {
    std::uint32_t hash = SeepageSpatialHash(x, y, static_cast<std::int32_t>(seed));
    hash ^= seed * 0x9e3779b9U;
    hash ^= hash >> 16U;
    hash *= 0x7feb352dU;
    hash ^= hash >> 15U;
    return static_cast<float>(hash & 0x00ffffffU) / static_cast<float>(0x01000000U);
}

struct SeepageNoise3Sample {
    float value = 0.5F;
    glm::vec3 gradient{0.0F};
};

glm::vec3 SeepageGradient3(std::uint32_t hash) {
    constexpr std::array<glm::vec3, 12> gradients{{
        {1.0F, 1.0F, 0.0F}, {-1.0F, 1.0F, 0.0F}, {1.0F, -1.0F, 0.0F}, {-1.0F, -1.0F, 0.0F},
        {1.0F, 0.0F, 1.0F}, {-1.0F, 0.0F, 1.0F}, {1.0F, 0.0F, -1.0F}, {-1.0F, 0.0F, -1.0F},
        {0.0F, 1.0F, 1.0F}, {0.0F, -1.0F, 1.0F}, {0.0F, 1.0F, -1.0F}, {0.0F, -1.0F, -1.0F},
    }};
    return gradients[hash % gradients.size()] * 0.70710678118F;
}

std::uint32_t SeepageHash3(const glm::ivec3& coordinate, std::uint32_t seed) {
    std::uint32_t hash = SeepageSpatialHash(coordinate.x, coordinate.y, coordinate.z);
    hash ^= seed * 0x9e3779b9U;
    hash ^= hash >> 16U;
    hash *= 0x7feb352dU;
    hash ^= hash >> 15U;
    hash *= 0x846ca68bU;
    hash ^= hash >> 16U;
    return hash;
}

SeepageNoise3Sample SeepageSimplexNoise3(glm::vec3 coordinate, std::uint32_t seed) {
    constexpr float skew = 1.0F / 3.0F;
    constexpr float unskew = 1.0F / 6.0F;
    const float skewAmount = (coordinate.x + coordinate.y + coordinate.z) * skew;
    const glm::ivec3 lattice = glm::ivec3(glm::floor(coordinate + skewAmount));
    const float unskewAmount = static_cast<float>(lattice.x + lattice.y + lattice.z) * unskew;
    const glm::vec3 x0 = coordinate - (glm::vec3(lattice) - unskewAmount);

    glm::ivec3 i1{0};
    glm::ivec3 i2{0};
    if (x0.x >= x0.y) {
        if (x0.y >= x0.z) {
            i1 = {1, 0, 0};
            i2 = {1, 1, 0};
        } else if (x0.x >= x0.z) {
            i1 = {1, 0, 0};
            i2 = {1, 0, 1};
        } else {
            i1 = {0, 0, 1};
            i2 = {1, 0, 1};
        }
    } else if (x0.y < x0.z) {
        i1 = {0, 0, 1};
        i2 = {0, 1, 1};
    } else if (x0.x < x0.z) {
        i1 = {0, 1, 0};
        i2 = {0, 1, 1};
    } else {
        i1 = {0, 1, 0};
        i2 = {1, 1, 0};
    }

    const std::array<glm::ivec3, 4> offsets{{glm::ivec3{0}, i1, i2, glm::ivec3{1}}};
    const std::array<glm::vec3, 4> deltas{{
        x0,
        x0 - glm::vec3(i1) + unskew,
        x0 - glm::vec3(i2) + 2.0F * unskew,
        x0 - glm::vec3{1.0F} + 3.0F * unskew,
    }};
    float rawValue = 0.0F;
    glm::vec3 rawGradient{0.0F};
    for (std::size_t corner = 0U; corner < deltas.size(); ++corner) {
        const glm::vec3 delta = deltas[corner];
        const float attenuation = 0.60F - glm::dot(delta, delta);
        if (attenuation <= 0.0F) {
            continue;
        }
        const glm::vec3 gradient = SeepageGradient3(SeepageHash3(lattice + offsets[corner], seed));
        const float gradientDot = glm::dot(gradient, delta);
        const float attenuation2 = attenuation * attenuation;
        const float attenuation3 = attenuation2 * attenuation;
        const float attenuation4 = attenuation2 * attenuation2;
        rawValue += attenuation4 * gradientDot;
        rawGradient += attenuation4 * gradient - 8.0F * attenuation3 * gradientDot * delta;
    }
    constexpr float outputScale = 32.0F;
    SeepageNoise3Sample result;
    result.value = Clamp01(0.5F + 0.5F * rawValue * outputScale);
    result.gradient = rawGradient * (0.5F * outputScale);
    return result;
}

SeepageNoise3Sample SeepageFractalNoise3(
    glm::vec3 coordinate,
    std::uint32_t seed,
    WaterSeepageQuality quality) {
    const std::uint32_t octaveCount = quality == WaterSeepageQuality::Low
                                          ? 1U
                                          : (quality == WaterSeepageQuality::High ? 3U : 2U);
    SeepageNoise3Sample result;
    result.value = 0.0F;
    result.gradient = glm::vec3{0.0F};
    float amplitude = 1.0F;
    float frequency = 1.0F;
    float weight = 0.0F;
    for (std::uint32_t octave = 0U; octave < octaveCount; ++octave) {
        const auto sample = SeepageSimplexNoise3(coordinate, seed + octave * 1013U);
        result.value += sample.value * amplitude;
        result.gradient += sample.gradient * (amplitude * frequency);
        weight += amplitude;
        coordinate = coordinate * 2.03F + glm::vec3{17.0F, 31.0F, 47.0F};
        frequency *= 2.03F;
        amplitude *= 0.5F;
    }
    if (weight > 0.0F) {
        result.value /= weight;
        result.gradient /= weight;
    }
    return result;
}

struct SeepageFanSample {
    float mask = 0.0F;
    float downDistance = 0.0F;
    float lateralDistance = 0.0F;
    float signedLateralDistance = 0.0F;
    // The strength- and slope-shaped run and local half-width actually used by
    // the membership test, so pattern evaluators (e.g. the wetting front) can
    // anchor their geometry to the same envelope.
    float effectiveReach = 0.0F;
    float effectiveHalfWidth = 0.0F;
    glm::vec3 surfaceNormal{0.0F, 1.0F, 0.0F};
    glm::vec3 downTangent{0.0F, 0.0F, -1.0F};
};

// Area envelope shared by every membership path (and mirrored in
// shaders/pointcloud_sparse_ripple.glsl — keep the constants in sync).
// Node Strength shapes WHERE seepage lives: the run scales with strength and
// travels further on near-vertical surfaces, while the half-width starts at
// the authored node width and spreads outward with travelled distance —
// faster on flat surfaces, slower on walls. Prominence never enters here; it
// only scales how strongly the effect is applied. Everything reads live
// parameters, so animation keys reshape the area without any topology work.
inline constexpr float kSeepageFlatReachFactor = 0.45F;
inline constexpr float kSeepageSteepReachFactor = 1.15F;
inline constexpr float kSeepageSpreadRateMetersPerMeter = 0.18F;
inline constexpr float kSeepageFlatSpreadFactor = 1.60F;
inline constexpr float kSeepageSteepSpreadFactor = 0.45F;

struct SeepageAreaEnvelope {
    float reachRun = 0.0F;
    float halfWidth = 0.0F;
    float endFeather = 0.0F;
    float lateralFeather = 0.0F;
};

SeepageAreaEnvelope EvaluateSeepageAreaEnvelope(
    const WaterSeepageRuntimeNode& node,
    float downDistance,
    const glm::vec3& surfaceNormal) {
    const float steep = Clamp01(1.0F - std::abs(surfaceNormal.z));
    const float strength = std::clamp(node.strength, 0.0F, 8.0F);
    SeepageAreaEnvelope envelope;
    envelope.reachRun = std::clamp(
        node.reachMeters * strength *
            std::lerp(kSeepageFlatReachFactor, kSeepageSteepReachFactor, steep),
        0.0F,
        std::max(0.0F, node.selectionReachLimitMeters));
    const float spreadRate =
        kSeepageSpreadRateMetersPerMeter * strength *
        std::lerp(kSeepageFlatSpreadFactor, kSeepageSteepSpreadFactor, steep);
    envelope.halfWidth = std::clamp(
        node.widthMeters * 0.5F + std::max(0.0F, downDistance) * spreadRate,
        0.0F,
        std::max(0.0F, node.selectionWidthLimitMeters) * 0.5F);
    const float feather = std::max(1.0e-5F, node.edgeFeatherMeters);
    envelope.endFeather = std::max(feather, envelope.reachRun * 0.10F);
    envelope.lateralFeather = std::max(feather, envelope.halfWidth * 0.30F);
    return envelope;
}

SeepageFanSample EvaluatePlanarSeepageFanMask(
    const WaterSeepageRuntimeNode& node,
    const glm::vec3& position,
    const glm::vec3& pointNormal) {
    SeepageFanSample sample;
    sample.surfaceNormal = node.surfaceNormal;
    sample.downTangent = node.downAxis;
    const glm::vec3 relative = position - node.position;
    sample.downDistance = glm::dot(relative, node.downAxis);
    sample.signedLateralDistance = glm::dot(relative, node.lateralAxis);
    sample.lateralDistance = std::abs(sample.signedLateralDistance);
    const float depthDistance = std::abs(glm::dot(relative, node.surfaceNormal));
    const float feather = std::max(1.0e-5F, node.edgeFeatherMeters);
    const auto envelope = EvaluateSeepageAreaEnvelope(
        node,
        sample.downDistance,
        node.surfaceNormal);
    sample.effectiveReach = envelope.reachRun;
    sample.effectiveHalfWidth = envelope.halfWidth;
    if (sample.downDistance < -feather ||
        sample.downDistance > envelope.reachRun + envelope.endFeather ||
        depthDistance > node.depthToleranceMeters + feather) {
        return sample;
    }

    if (sample.lateralDistance > envelope.halfWidth + envelope.lateralFeather) {
        return sample;
    }

    const float startMask = SmoothStep(-feather, 0.0F, sample.downDistance);
    const float endMask = 1.0F - SmoothStep(
        envelope.reachRun - envelope.endFeather,
        envelope.reachRun + envelope.endFeather,
        sample.downDistance);
    const float lateralMask = 1.0F - SmoothStep(
        envelope.halfWidth,
        envelope.halfWidth + envelope.lateralFeather,
        sample.lateralDistance);
    const float depthMask = 1.0F - SmoothStep(
        node.depthToleranceMeters,
        node.depthToleranceMeters + feather,
        depthDistance);
    const float normalAgreement = std::abs(glm::dot(pointNormal, node.surfaceNormal));
    const float aligned = SmoothStep(0.15F, 0.85F, normalAgreement);
    const float normalMask = std::lerp(1.0F, aligned, Clamp01(node.normalAlignment));
    sample.mask = Clamp01(startMask * endMask * lateralMask * depthMask * normalMask);
    return sample;
}

SeepageFanSample EvaluateGuidedSeepageFanMask(
    const WaterSeepageRuntimeNode& node,
    const glm::vec3& position,
    const glm::vec3& pointNormal) {
    SeepageFanSample sample;
    if (!node.guideValid || node.guideSampleCount < 2U) {
        return sample;
    }

    const float feather = std::max(1.0e-5F, node.edgeFeatherMeters);
    if (position.z > node.position.z + feather) {
        return sample;
    }

    std::size_t closestSegment = 0U;
    float closestRawAmount = 0.0F;
    float closestAmount = 0.0F;
    float closestDistanceSquared = std::numeric_limits<float>::max();
    for (std::size_t segmentIndex = 0U;
         segmentIndex + 1U < node.guideSampleCount;
         ++segmentIndex) {
        const glm::vec3 start = ToGlm(node.guideSamples[segmentIndex].position);
        const glm::vec3 end = ToGlm(node.guideSamples[segmentIndex + 1U].position);
        const glm::vec3 segment = end - start;
        const float lengthSquared = glm::dot(segment, segment);
        if (!IsValidPoint(start) || !IsValidPoint(end) || lengthSquared <= 1.0e-10F) {
            continue;
        }
        const float rawAmount = glm::dot(position - start, segment) / lengthSquared;
        const float amount = Clamp01(rawAmount);
        const glm::vec3 center = start + segment * amount;
        const float distanceSquared = glm::dot(position - center, position - center);
        if (distanceSquared + 1.0e-8F < closestDistanceSquared) {
            closestSegment = segmentIndex;
            closestRawAmount = rawAmount;
            closestAmount = amount;
            closestDistanceSquared = distanceSquared;
        }
    }
    if (closestDistanceSquared == std::numeric_limits<float>::max()) {
        glm::vec3 resolvedNormal = pointNormal;
        if (!IsValidPoint(resolvedNormal) || glm::dot(resolvedNormal, resolvedNormal) <= kNormalEpsilon) {
            resolvedNormal = node.surfaceNormal;
        } else {
            resolvedNormal = glm::normalize(resolvedNormal);
        }
        return EvaluatePlanarSeepageFanMask(node, position, resolvedNormal);
    }

    const auto& startSample = node.guideSamples[closestSegment];
    const auto& endSample = node.guideSamples[closestSegment + 1U];
    const glm::vec3 start = ToGlm(startSample.position);
    const glm::vec3 end = ToGlm(endSample.position);
    const glm::vec3 segment = end - start;
    const float segmentLength = glm::length(segment);
    if (segmentLength <= 1.0e-5F) {
        return sample;
    }
    const glm::vec3 tangent = segment / segmentLength;
    sample.downTangent = tangent;
    const glm::vec3 center = start + segment * closestAmount;
    float stationAmount = closestAmount;
    if ((closestSegment == 0U && closestRawAmount < 0.0F) ||
        (closestSegment + 2U == node.guideSampleCount && closestRawAmount > 1.0F)) {
        stationAmount = closestRawAmount;
    }
    sample.downDistance = std::lerp(
        startSample.station,
        endSample.station,
        stationAmount);

    glm::vec3 startNormal = SafeSeepageNormal(startSample.normal);
    glm::vec3 endNormal = SafeSeepageNormal(endSample.normal);
    if (glm::dot(startNormal, endNormal) < 0.0F) {
        endNormal = -endNormal;
    }
    glm::vec3 surfaceNormal = glm::mix(startNormal, endNormal, closestAmount);
    if (!IsValidPoint(surfaceNormal) || glm::dot(surfaceNormal, surfaceNormal) <= kNormalEpsilon) {
        surfaceNormal = startNormal;
    } else {
        surfaceNormal = glm::normalize(surfaceNormal);
    }
    sample.surfaceNormal = surfaceNormal;
    glm::vec3 lateral = glm::cross(surfaceNormal, tangent);
    if (!IsValidPoint(lateral) || glm::dot(lateral, lateral) <= kNormalEpsilon) {
        lateral = node.lateralAxis;
    } else {
        lateral = glm::normalize(lateral);
    }
    if (glm::dot(lateral, node.lateralAxis) < 0.0F) {
        lateral = -lateral;
    }

    const glm::vec3 relative = position - center;
    sample.signedLateralDistance = glm::dot(relative, lateral);
    sample.lateralDistance = std::abs(sample.signedLateralDistance);
    const float depthDistance = std::abs(glm::dot(relative, surfaceNormal));
    auto envelope = EvaluateSeepageAreaEnvelope(
        node,
        sample.downDistance,
        surfaceNormal);
    // The guide cannot represent stations beyond what it traced, so the run
    // is additionally bounded by the achieved guide extent. The end feather
    // must follow the bounded run (the GPU mirror applies the bound before
    // deriving the feather).
    const float lastStation = node.guideSamples[node.guideSampleCount - 1U].station;
    envelope.reachRun = std::min({
        envelope.reachRun,
        std::max(0.0F, node.guideAchievedReachMeters),
        std::max(0.0F, lastStation),
    });
    envelope.endFeather = std::max(feather, envelope.reachRun * 0.10F);
    sample.effectiveReach = envelope.reachRun;
    sample.effectiveHalfWidth = envelope.halfWidth;
    if (envelope.reachRun <= 1.0e-5F ||
        sample.downDistance < -feather ||
        sample.downDistance > envelope.reachRun + envelope.endFeather ||
        depthDistance > node.depthToleranceMeters + feather) {
        return sample;
    }

    if (sample.lateralDistance > envelope.halfWidth + envelope.lateralFeather) {
        return sample;
    }

    const float startMask = SmoothStep(-feather, 0.0F, sample.downDistance);
    const float endMask = 1.0F - SmoothStep(
        envelope.reachRun - envelope.endFeather,
        envelope.reachRun + envelope.endFeather,
        sample.downDistance);
    const float lateralMask = 1.0F - SmoothStep(
        envelope.halfWidth,
        envelope.halfWidth + envelope.lateralFeather,
        sample.lateralDistance);
    const float depthMask = 1.0F - SmoothStep(
        node.depthToleranceMeters,
        node.depthToleranceMeters + feather,
        depthDistance);
    const bool pointNormalValid = IsValidPoint(pointNormal) &&
                                  glm::dot(pointNormal, pointNormal) > kNormalEpsilon;
    const glm::vec3 resolvedPointNormal = pointNormalValid
                                              ? glm::normalize(pointNormal)
                                              : surfaceNormal;
    const float normalAgreement = std::abs(glm::dot(resolvedPointNormal, surfaceNormal));
    const float aligned = SmoothStep(0.15F, 0.85F, normalAgreement);
    const float normalMask = std::lerp(1.0F, aligned, Clamp01(node.normalAlignment));
    sample.mask = Clamp01(startMask * endMask * lateralMask * depthMask * normalMask);
    return sample;
}

SeepageFanSample EvaluateSeepageFanMask(
    const WaterSeepageRuntimeNode& node,
    const glm::vec3& position,
    const glm::vec3& pointNormal) {
    const float feather = std::max(1.0e-5F, node.edgeFeatherMeters);
    if (position.z > node.position.z + feather) {
        return {};
    }
    if (node.guideValid && node.guideSampleCount >= 2U) {
        return EvaluateGuidedSeepageFanMask(node, position, pointNormal);
    }
    glm::vec3 resolvedNormal = pointNormal;
    if (!IsValidPoint(resolvedNormal) || glm::dot(resolvedNormal, resolvedNormal) <= kNormalEpsilon) {
        resolvedNormal = node.surfaceNormal;
    } else {
        resolvedNormal = glm::normalize(resolvedNormal);
    }
    return EvaluatePlanarSeepageFanMask(node, position, resolvedNormal);
}

struct SeepagePatternSignals {
    float damp = 0.0F;
    float variation = 0.0F;
    float glint = 0.0F;
};

glm::vec3 SeepageEnvironmentDirection(const WaterSeepageLookSettings& look);

glm::vec3 ResolveSeepagePatternNormal(
    const glm::vec3& pointNormal,
    const SeepageFanSample& fan) {
    if (IsValidPoint(pointNormal) && glm::dot(pointNormal, pointNormal) > kNormalEpsilon) {
        return glm::normalize(pointNormal);
    }
    return SafeSeepageNormal(FromGlm(fan.surfaceNormal));
}

glm::vec3 ResolveSeepageViewDirection(
    const glm::vec3& position,
    const glm::vec3& environmentDirection,
    const WaterSeepageViewContext& viewContext) {
    if (viewContext.hasCameraPosition) {
        const glm::vec3 view = ToGlm(viewContext.cameraPosition) - position;
        if (IsValidPoint(view) && glm::dot(view, view) > kNormalEpsilon) {
            return glm::normalize(view);
        }
    }
    return -SafeSeepageNormal(FromGlm(environmentDirection));
}

float SeepageReflectionSignal(
    const WaterSeepageLookSettings& look,
    const glm::vec3& position,
    const glm::vec3& baseNormal,
    const glm::vec3& microGradient,
    const glm::vec3& environmentDirection,
    const WaterSeepageViewContext& viewContext,
    float sparseGate) {
    glm::vec3 tangentGradient = microGradient - baseNormal * glm::dot(microGradient, baseNormal);
    const float tangentLength = glm::length(tangentGradient);
    if (tangentLength > 1.0e-5F) {
        tangentGradient /= tangentLength;
    } else {
        tangentGradient = glm::vec3{0.0F};
    }
    const glm::vec3 microNormal = glm::normalize(
        baseNormal + tangentGradient * (look.microNormalStrength * 0.42F));
    const glm::vec3 viewDirection = ResolveSeepageViewDirection(
        position,
        environmentDirection,
        viewContext);
    glm::vec3 halfVector = viewDirection + environmentDirection;
    if (glm::dot(halfVector, halfVector) <= kNormalEpsilon) {
        halfVector = environmentDirection;
    }
    halfVector = glm::normalize(halfVector);
    const float roughness = std::clamp(look.roughness, 0.02F, 1.0F);
    const float exponent = 2.0F + (1.0F - roughness) * (1.0F - roughness) * 126.0F;
    const float specular = std::pow(
        std::max(0.0F, glm::dot(microNormal, halfVector)),
        exponent);
    const float fresnel = std::pow(
        Clamp01(1.0F - std::abs(glm::dot(microNormal, viewDirection))),
        5.0F);
    const float angleResponse = Clamp01(look.angleResponse);
    return Clamp01(
        specular * (0.35F + angleResponse * 1.15F) * (0.28F + sparseGate * 0.72F) +
        fresnel * (0.08F + angleResponse * 0.32F));
}

SeepagePatternSignals EvaluateWetRockSeepageSignals(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageLookSettings& look,
    const SeepageFanSample& fan,
    const glm::vec3& position,
    const glm::vec3& pointNormal,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext) {
    const float rainGain = Clamp01(node.rainVisualStrength * look.rainResponse);
    const float wetness = Clamp01(look.baseWetness + 0.30F * rainGain);
    const float density = Clamp01(look.density + 0.25F * rainGain);
    const float featureSize = std::max(0.005F, look.featureSizeMeters);
    const float time = std::max(0.0F, timeSeconds) * look.evolution;
    glm::vec3 coordinate = node.noiseRotation * (position / featureSize);
    coordinate += glm::vec3{0.071F, -0.053F, 0.037F} * time;
    const std::uint32_t proceduralSeed = node.seed ^ (node.id * 0x9e3779b9U);
    const auto noise = SeepageFractalNoise3(coordinate, proceduralSeed, node.resolvedQuality);
    const float feather = std::lerp(0.24F, 0.035F, look.contrast);
    const float threshold = 1.0F - density;
    const float patch = SmoothStep(threshold - feather, threshold + feather, noise.value);
    float sparseGate = noise.value;
    if (node.resolvedQuality != WaterSeepageQuality::Low) {
        sparseGate = SeepageSimplexNoise3(
            coordinate * 3.73F + glm::vec3{13.0F, -7.0F, 19.0F},
            proceduralSeed + 4099U).value;
    }
    sparseGate = SmoothStep(0.92F - look.glintDensity * 0.62F, 0.98F, sparseGate);
    const glm::vec3 baseNormal = ResolveSeepagePatternNormal(pointNormal, fan);
    const glm::vec3 environmentDirection = SeepageEnvironmentDirection(look);
    const glm::vec3 worldGradient = glm::transpose(node.noiseRotation) * noise.gradient;
    const float reflection = SeepageReflectionSignal(
        look,
        position,
        baseNormal,
        worldGradient,
        environmentDirection,
        viewContext,
        sparseGate);
    // Strength shapes the area envelope inside the fan mask; the signal
    // amplitude is intensity territory and belongs to prominence alone.
    const float strengthMask = fan.mask;
    return {
        .damp = Clamp01(strengthMask * wetness * (0.68F + noise.value * 0.22F + patch * 0.20F)),
        .variation = Clamp01(strengthMask * patch * (0.08F + noise.value * 0.14F)),
        .glint = Clamp01(strengthMask * patch * look.glisten * reflection),
    };
}

SeepagePatternSignals EvaluateChaoticBloomSeepageSignals(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageLookSettings& look,
    const SeepageFanSample& fan,
    const glm::vec3& position,
    const glm::vec3& pointNormal,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext) {
    const float rainGain = Clamp01(node.rainVisualStrength * look.rainResponse);
    const float wetness = Clamp01(look.baseWetness + 0.30F * rainGain);
    const float density = Clamp01(look.density + 0.25F * rainGain);
    const float featureSize = std::max(0.005F, look.featureSizeMeters);
    const float time = std::max(0.0F, timeSeconds);
    const glm::vec3 advectedPosition =
        position - fan.downTangent * (time * look.downhillDriftMetersPerSecond);
    glm::vec3 coordinate = node.noiseRotation * (advectedPosition / featureSize);
    const std::uint32_t proceduralSeed = node.seed ^ (node.id * 0x9e3779b9U);
    const auto warpNoise = SeepageFractalNoise3(
        coordinate * 0.53F + glm::vec3{0.031F, -0.047F, 0.023F} * (time * look.evolution),
        proceduralSeed + 2053U,
        node.resolvedQuality);
    coordinate += warpNoise.gradient * (look.curl * 0.22F);
    coordinate += glm::vec3{-0.029F, 0.041F, 0.017F} * (time * look.evolution);
    const auto bodyNoise = SeepageFractalNoise3(coordinate, proceduralSeed, node.resolvedQuality);
    const float ridge = 1.0F - std::abs(bodyNoise.value * 2.0F - 1.0F);
    const float organic = Clamp01(ridge * 0.68F + bodyNoise.value * 0.32F);
    const float threshold = 0.76F - density * 0.48F + look.breakup * 0.16F;
    const float bloom = SmoothStep(threshold, threshold + 0.18F, organic);
    const float sparseGate = SmoothStep(
        0.90F - look.glintDensity * 0.58F,
        0.98F,
        warpNoise.value);
    const glm::vec3 baseNormal = ResolveSeepagePatternNormal(pointNormal, fan);
    const glm::vec3 environmentDirection = SeepageEnvironmentDirection(look);
    const glm::vec3 worldGradient = glm::transpose(node.noiseRotation) * bodyNoise.gradient;
    const float reflection = SeepageReflectionSignal(
        look,
        position,
        baseNormal,
        worldGradient,
        environmentDirection,
        viewContext,
        sparseGate);
    // Strength shapes the area envelope inside the fan mask, not the signal
    // amplitude (see EvaluateSeepageAreaEnvelope).
    const float strengthMask = fan.mask;
    return {
        .damp = Clamp01(
            strengthMask *
            (wetness * (0.60F + bodyNoise.value * 0.22F) + bloom * (0.16F + rainGain * 0.14F))),
        .variation = Clamp01(strengthMask * bloom * (0.18F + organic * 0.42F)),
        .glint = Clamp01(strengthMask * bloom * look.glisten * reflection),
    };
}

SeepagePatternSignals EvaluateWettingTrickleSeepageSignals(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageLookSettings& look,
    const SeepageFanSample& fan,
    const glm::vec3& position,
    const glm::vec3& pointNormal,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext) {
    const float progress = Clamp01(node.wettingProgress);
    if (progress <= 1.0e-6F) {
        return {};
    }

    const float rainGain = Clamp01(node.rainVisualStrength * look.rainResponse);
    const float wetness = Clamp01(look.baseWetness + 0.30F * rainGain);
    const float density = Clamp01(look.density + 0.25F * rainGain);
    const float featureSize = std::max(0.005F, look.tricklePatchSizeMeters);
    const float trickleWidth = std::max(0.002F, look.trickleWidthMeters);
    // The wetting front travels the same strength- and slope-shaped run the
    // fan mask uses, so the pattern never re-declares its own extent (the
    // node Reach — already spread/rain scaled — is the single area source).
    const float trickleLength = std::max(0.005F, fan.effectiveReach);
    const float frontSoftness = std::max(0.002F, look.trickleFrontSoftness);
    const float downstream = std::max(0.0F, fan.downDistance);
    const float distanceProgress =
        std::clamp(downstream / trickleLength, 0.0F, 1.0F);
    const float time = std::max(0.0F, timeSeconds);
    glm::vec3 resolvedDown = fan.downTangent;
    if (!IsValidPoint(resolvedDown) ||
        glm::dot(resolvedDown, resolvedDown) <= kNormalEpsilon) {
        resolvedDown = glm::vec3{0.0F, 0.0F, 1.0F};
    } else {
        resolvedDown = glm::normalize(resolvedDown);
    }
    const glm::vec3 advectedPosition =
        position -
        resolvedDown *
            (time * std::max(0.0F, look.downhillDriftMetersPerSecond));
    const std::uint32_t proceduralSeed = node.seed ^ (node.id * 0x9e3779b9U);

    glm::vec3 coordinate =
        node.noiseRotation * (advectedPosition / featureSize);
    coordinate +=
        glm::vec3{0.037F, -0.029F, 0.043F} *
        (time * look.evolution);
    const auto trickleNoise = SeepageFractalNoise3(
        coordinate,
        proceduralSeed + 6151U,
        node.resolvedQuality);
    const float breakup = Clamp01(look.breakup);
    const float patchThreshold =
        0.70F - density * 0.34F + breakup * 0.07F;
    const float patchField = SmoothStep(
        patchThreshold,
        patchThreshold + std::lerp(0.22F, 0.12F, breakup),
        trickleNoise.value);

    const float sourceDistance =
        std::hypot(downstream, fan.signedLateralDistance);
    const float sourceEnvelope = 1.0F - SmoothStep(
        featureSize * 0.30F,
        featureSize * 1.35F,
        sourceDistance);
    const float sourcePatch =
        sourceEnvelope * std::lerp(0.38F, 1.0F, patchField);

    const float fanHalfWidth = std::max(1.0e-4F, fan.effectiveHalfWidth);
    const float lateralWander =
        (trickleNoise.value - 0.5F) *
        featureSize *
        std::lerp(0.16F, 0.85F, breakup);
    const float laneHash0 =
        SeepageHash01(0, 0, proceduralSeed + 7013U);
    const float laneHash1 =
        SeepageHash01(1, 0, proceduralSeed + 7013U);
    const float laneHash2 =
        SeepageHash01(2, 0, proceduralSeed + 7013U);
    const float lane0 =
        (laneHash0 - 0.5F) * fanHalfWidth * 0.55F +
        lateralWander;
    const float lane1 =
        (laneHash1 - 0.5F) * fanHalfWidth * 1.25F -
        lateralWander * 0.58F;
    const float lane2 =
        (laneHash2 - 0.5F) * fanHalfWidth * 1.55F +
        lateralWander * 0.36F;
    const float softWidth =
        trickleWidth * std::lerp(1.65F, 2.65F, breakup);
    float fingers = 1.0F - SmoothStep(
        trickleWidth,
        softWidth,
        std::abs(fan.signedLateralDistance - lane0));
    const float secondaryGate = SmoothStep(
        0.22F,
        0.72F,
        density + laneHash1 * 0.24F);
    const float tertiaryGate = SmoothStep(
        0.48F,
        0.90F,
        density + laneHash2 * 0.18F);
    fingers = std::max(
        fingers,
        (1.0F - SmoothStep(
                    trickleWidth * 0.82F,
                    softWidth * 0.86F,
                    std::abs(
                        fan.signedLateralDistance -
                        lane1))) *
            secondaryGate);
    fingers = std::max(
        fingers,
        (1.0F - SmoothStep(
                    trickleWidth * 0.68F,
                    softWidth * 0.74F,
                    std::abs(
                        fan.signedLateralDistance -
                        lane2))) *
            tertiaryGate);

    const auto delayCellDown = static_cast<std::int32_t>(
        std::floor(downstream / featureSize));
    const auto delayCellLateral = static_cast<std::int32_t>(
        std::floor(fan.signedLateralDistance / featureSize));
    const float saturationDelay = SeepageHash01(
        delayCellDown,
        delayCellLateral,
        proceduralSeed + 8089U);
    const float onset = std::clamp(
        0.03F +
            distanceProgress * 0.68F +
            saturationDelay * 0.14F +
            (1.0F - patchField) * breakup * 0.06F,
        0.0F,
        0.92F);
    const float progressFeather = std::clamp(
        frontSoftness / trickleLength,
        0.008F,
        0.32F);
    const float wetReveal = SmoothStep(
        onset - progressFeather,
        onset + progressFeather,
        progress);
    const float frontPulse = 1.0F - SmoothStep(
        progressFeather,
        progressFeather * 3.2F,
        std::abs(progress - onset));
    const float breakupGate = std::lerp(
        1.0F,
        SmoothStep(
            0.20F,
            0.78F,
            trickleNoise.value + patchField * 0.22F),
        breakup);
    const float trickleBody = fingers * breakupGate;
    const float persistentDamp = std::max(
        sourcePatch,
        trickleBody * (0.50F + patchField * 0.50F));
    const float activeWet = wetReveal * persistentDamp;
    const glm::vec3 baseNormal = ResolveSeepagePatternNormal(pointNormal, fan);
    const glm::vec3 environmentDirection = SeepageEnvironmentDirection(look);
    const glm::vec3 worldGradient =
        glm::transpose(node.noiseRotation) *
        trickleNoise.gradient;
    const float sparseGate = SmoothStep(
        0.90F - look.glintDensity * 0.58F,
        0.98F,
        trickleNoise.value);
    const float reflection = SeepageReflectionSignal(
        look,
        position,
        baseNormal,
        worldGradient,
        environmentDirection,
        viewContext,
        std::max(sparseGate, activeWet * 0.42F));
    // Strength shapes the area envelope inside the fan mask, not the signal
    // amplitude (see EvaluateSeepageAreaEnvelope).
    const float strengthMask = fan.mask;
    return {
        .damp = Clamp01(
            strengthMask * activeWet *
            (wetness * (0.70F + patchField * 0.22F) +
             rainGain * 0.12F)),
        .variation = Clamp01(
            strengthMask * wetReveal *
            (trickleBody *
                 (0.16F + patchField * 0.20F) +
             frontPulse * fingers * 0.52F +
             sourcePatch * 0.08F)),
        .glint = Clamp01(
            strengthMask *
            activeWet *
            look.glisten *
            reflection),
    };
}

float WrapContourPulseDistance(
    float distanceMeters,
    float stableSpanMeters) {
    if (!std::isfinite(distanceMeters) ||
        !std::isfinite(stableSpanMeters) ||
        stableSpanMeters <= 1.0e-6F) {
        return 0.0F;
    }
    float wrapped = std::fmod(distanceMeters, stableSpanMeters);
    if (wrapped < 0.0F) {
        wrapped += stableSpanMeters;
    }
    return wrapped;
}

float SampleAnimatedContourPulseField(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageLookSettings& look,
    bool transitionPulseField,
    float fieldDistanceMeters,
    float timeSeconds) {
    const auto& field = transitionPulseField
                            ? node.transitionPulseField
                            : node.pulseField;
    if (field.sampleCount == 0U ||
        !std::isfinite(field.stableSpanMeters) ||
        field.stableSpanMeters <= 1.0e-6F) {
        return 0.0F;
    }

    const float time = std::max(
        0.0F,
        SeepageFiniteOr(timeSeconds, 0.0F));
    const float speed = std::max(
        0.0F,
        SeepageFiniteOr(look.pulseSpeedMetersPerSecond, 0.0F));
    const float speedVariation = Clamp01(look.pulseSpeedVariation);
    const float irregularity = Clamp01(look.pulseIrregularity);
    const float evolution = std::max(
        0.0F,
        SeepageFiniteOr(look.evolution, 0.0F));
    const float span = field.stableSpanMeters;
    const std::uint32_t proceduralSeed =
        node.seed ^ (node.id * 0x9e3779b9U);
    const float evolutionHash =
        SeepageHash01(7, 19, proceduralSeed + 49201U);
    const float evolutionPhase = std::fmod(
        time * evolution * (0.15F + evolutionHash * 0.25F) +
            evolutionHash * kSeepageTwoPi,
        kSeepageTwoPi);
    const float evolutionShift =
        std::sin(evolutionPhase) *
        std::max(0.001F, look.pulseWidthMeters) *
        irregularity * 0.55F;

    // The reference field contains every authored launch and fractional wave
    // weight. Advancing its lookup coordinate in the shader makes clock-only
    // frames free of CPU field construction and parameter uploads.
    const float primaryDistance = WrapContourPulseDistance(
        fieldDistanceMeters - time * speed + evolutionShift,
        span);
    float pulse = field.Sample(primaryDistance);

    // Balanced/High tiers add one/two deterministic phase populations. They
    // move at nearby speeds and merge additively, retaining the authored
    // catch-up/overlap character without restoring a 7–12-wave point loop.
    if (node.resolvedQuality == WaterSeepageQuality::Balanced ||
        node.resolvedQuality == WaterSeepageQuality::High) {
        const float phaseHash =
            SeepageHash01(23, 41, proceduralSeed + 58309U);
        const float speedHash =
            SeepageHash01(31, 59, proceduralSeed + 61487U);
        const float secondarySpeed =
            speed *
            std::max(
                0.15F,
                1.0F +
                    (0.35F + speedHash * 0.45F) *
                        speedVariation);
        const float secondaryDistance = WrapContourPulseDistance(
            fieldDistanceMeters + phaseHash * span -
                time * secondarySpeed -
                evolutionShift * 0.65F,
            span);
        pulse +=
            field.Sample(secondaryDistance) *
            speedVariation * 0.82F;
    }
    if (node.resolvedQuality == WaterSeepageQuality::High) {
        const float phaseHash =
            SeepageHash01(47, 71, proceduralSeed + 64763U);
        const float speedHash =
            SeepageHash01(61, 83, proceduralSeed + 68371U);
        const float tertiarySpeed =
            speed *
            std::max(
                0.15F,
                1.0F -
                    (0.25F + speedHash * 0.40F) *
                        speedVariation);
        const float tertiaryDistance = WrapContourPulseDistance(
            fieldDistanceMeters + phaseHash * span -
                time * tertiarySpeed +
                evolutionShift * 0.40F,
            span);
        pulse +=
            field.Sample(tertiaryDistance) *
            speedVariation * 0.50F;
    }
    return Clamp01(
        pulse + std::max(0.0F, pulse - 0.82F) * 0.22F);
}

SeepagePatternSignals EvaluateContourPulseSeepageSignals(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageLookSettings& look,
    const SeepageFanSample& fan,
    const glm::vec3& position,
    const glm::vec3& pointNormal,
    float timeSeconds,
    bool transitionPulseField,
    const WaterSeepageViewContext& viewContext) {
    const float rainGain = Clamp01(node.rainVisualStrength * look.rainResponse);
    const float wetness = Clamp01(look.baseWetness + 0.30F * rainGain);
    const float density = Clamp01(look.density + 0.25F * rainGain);
    const float spacing = std::max(0.005F, look.pulseSpacingMeters);
    const float irregularity = Clamp01(look.pulseIrregularity);
    const float downstream = std::max(0.0F, fan.downDistance);
    const float lateralNormalised = std::clamp(
        fan.signedLateralDistance / std::max(0.001F, fan.effectiveHalfWidth),
        -1.5F,
        1.5F);
    const std::uint32_t proceduralSeed = node.seed ^ (node.id * 0x9e3779b9U);

    // The surface field is stationary. Only independently launched fronts
    // move over it, avoiding the sliding-texture look produced by advecting
    // the complete noise domain down the support.
    const glm::vec3 coordinate =
        node.noiseRotation *
        (position / (spacing * 1.65F));
    const auto frontNoise = SeepageFractalNoise3(
        coordinate,
        proceduralSeed + 17431U,
        node.resolvedQuality);
    const float centredNoise = frontNoise.value * 2.0F - 1.0F;
    const float baseBowedFront =
        std::pow(std::abs(lateralNormalised), 1.35F) *
        spacing * (0.10F + irregularity * 0.28F);
    // Authored launches and fractional wave count are composed into a stable
    // reference field. Renderer time supplies motion/evolution below while
    // stationary world noise bends each point's lookup coordinate.
    const float frontWarp =
        centredNoise * spacing * irregularity * 0.25F;
    const float pulse = Clamp01(
        SampleAnimatedContourPulseField(
            node,
            look,
            transitionPulseField,
            downstream + baseBowedFront + frontWarp,
            timeSeconds) *
        std::lerp(0.82F, 1.16F, frontNoise.value));

    const float coverageThreshold = 0.72F - density * 0.38F;
    const float dampPatch = SmoothStep(
        coverageThreshold,
        coverageThreshold + 0.24F,
        frontNoise.value);
    const glm::vec3 baseNormal = ResolveSeepagePatternNormal(pointNormal, fan);
    const glm::vec3 environmentDirection = SeepageEnvironmentDirection(look);
    const glm::vec3 worldGradient =
        glm::transpose(node.noiseRotation) * frontNoise.gradient;
    const float reflection = SeepageReflectionSignal(
        look,
        position,
        baseNormal,
        worldGradient,
        environmentDirection,
        viewContext,
        std::max(pulse, dampPatch * 0.15F));
    const float strengthMask = fan.mask;
    return {
        .damp = Clamp01(
            strengthMask *
            (wetness * (0.06F + dampPatch * 0.10F) +
             pulse *
                 (0.62F + wetness * 0.14F +
                  rainGain * 0.12F))),
        .variation = Clamp01(
            strengthMask * pulse * (0.30F + dampPatch * 0.24F)),
        .glint = Clamp01(
            strengthMask *
            (0.015F + pulse * 0.78F + dampPatch * 0.06F) *
            look.glisten * reflection),
    };
}

struct SelectedSeepageTransitionLook {
    WaterSeepageLookSettings look{};
    bool transition = false;
};

SelectedSeepageTransitionLook SelectSeepageTransitionLook(
    const WaterSeepageRuntimeNode& node,
    const glm::vec3& position) {
    if (!node.transitionLook.has_value() || node.transitionAmount <= 1.0e-6F) {
        return {.look = node.look, .transition = false};
    }
    if (node.transitionAmount >= 1.0F - 1.0e-6F) {
        return {
            .look = node.transitionLook.value(),
            .transition = true,
        };
    }
    const float cellSize = std::max(0.005F, std::min(
        node.look.featureSizeMeters,
        node.transitionLook->featureSizeMeters) * 0.18F);
    const glm::ivec3 coordinate = glm::ivec3(glm::floor(position / cellSize));
    const float selector = static_cast<float>(
        SeepageHash3(coordinate, node.seed ^ node.id) & 0x00ffffffU) /
        static_cast<float>(0x01000000U);
    const bool useTransition = selector < node.transitionAmount;
    return {
        .look = useTransition
                    ? node.transitionLook.value()
                    : node.look,
        .transition = useTransition,
    };
}

void BlendSeepageContribution(
    WaterSeepageRuntimeContribution* target,
    const WaterSeepageRuntimeContribution& contribution,
    WaterEffectBlendMode blendMode) {
    if (target == nullptr || contribution.scale <= 1.0e-6F) {
        return;
    }
    if (target->scale <= 1.0e-6F || blendMode == WaterEffectBlendMode::Override) {
        *target = contribution;
        return;
    }
    if (blendMode == WaterEffectBlendMode::Max) {
        target->mask = std::max(target->mask, contribution.mask);
        target->damp = std::max(target->damp, contribution.damp);
        target->ripple = std::max(target->ripple, contribution.ripple);
        target->glint = std::max(target->glint, contribution.glint);
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

    target->mask = std::max(target->mask, contribution.mask);
    target->damp = std::max(target->damp, contribution.damp);
    target->ripple = std::max(target->ripple, contribution.ripple);
    target->glint = std::max(target->glint, contribution.glint);
    if (blendMode == WaterEffectBlendMode::Multiply) {
        target->scale = std::max(target->scale, contribution.scale);
    } else if (blendMode == WaterEffectBlendMode::Screen) {
        target->scale = 1.0F - ((1.0F - Clamp01(target->scale)) * (1.0F - Clamp01(contribution.scale)));
    } else {
        target->scale = Clamp01(target->scale + contribution.scale);
    }
    target->emissionAdd += contribution.emissionAdd;
    target->opacityAdd += contribution.opacityAdd;
    target->opacityMultiply *= contribution.opacityMultiply;
    target->pointSizeAdd += contribution.pointSizeAdd;
    target->pointSizeMultiply *= contribution.pointSizeMultiply;
    const float combinedMix = Clamp01(target->colourMix + contribution.colourMix);
    if (combinedMix > 1.0e-6F) {
        target->colour = glm::mix(
            target->colour,
            contribution.colour,
            contribution.colourMix / combinedMix);
    }
    target->colourMix = combinedMix;
}

bool SeepageBoundsContain(
    const invisible_places::io::Bounds3f& bounds,
    const invisible_places::io::Float3& point) {
    return bounds.valid &&
           point.x >= bounds.minimum.x && point.x <= bounds.maximum.x &&
           point.y >= bounds.minimum.y && point.y <= bounds.maximum.y &&
           point.z >= bounds.minimum.z && point.z <= bounds.maximum.z;
}

const WaterSeepageSpatialHashCell* FindSeepageHashCell(
    std::span<const WaterSeepageSpatialHashCell> hashCells,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z) {
    if (hashCells.empty() || (hashCells.size() & (hashCells.size() - 1U)) != 0U) {
        return nullptr;
    }
    const auto mask = hashCells.size() - 1U;
    std::size_t slot = static_cast<std::size_t>(SeepageSpatialHash(x, y, z)) & mask;
    for (std::size_t probe = 0U; probe < hashCells.size(); ++probe) {
        const auto& cell = hashCells[slot];
        if (!cell.occupied) {
            return nullptr;
        }
        if (cell.x == x && cell.y == y && cell.z == z) {
            return &cell;
        }
        slot = (slot + 1U) & mask;
    }
    return nullptr;
}

const WaterSeepageSpatialHashCell* FindSeepageHashCell(
    const WaterSeepageSpatialGrid& grid,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z) {
    return FindSeepageHashCell(grid.hashCells, x, y, z);
}

void SeepageFingerprintByte(std::uint64_t* hash, std::uint8_t value) {
    *hash ^= value;
    *hash *= 1099511628211ULL;
}

void SeepageFingerprintU32(std::uint64_t* hash, std::uint32_t value) {
    for (std::uint32_t shift = 0U; shift < 32U; shift += 8U) {
        SeepageFingerprintByte(hash, static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
}

void SeepageFingerprintFloat(std::uint64_t* hash, float value) {
    SeepageFingerprintU32(hash, std::bit_cast<std::uint32_t>(value));
}

std::string SeepageFingerprintString(std::uint64_t hash) {
    std::ostringstream stream;
    stream << std::hex << std::setw(16) << std::setfill('0') << hash;
    return stream.str();
}

float LerpSeepageAngle(float left, float right, float amount) {
    const float delta = std::remainder(right - left, 360.0F);
    return std::remainder(left + delta * amount, 360.0F);
}

WaterEffectResponseSettings LerpSeepageResponse(
    const WaterEffectResponseSettings& left,
    const WaterEffectResponseSettings& right,
    float amount) {
    WaterEffectResponseSettings result;
    result.intensity = std::lerp(left.intensity, right.intensity, amount);
    result.emissionAdd = std::lerp(left.emissionAdd, right.emissionAdd, amount);
    result.opacityAdd = std::lerp(left.opacityAdd, right.opacityAdd, amount);
    result.opacityMultiply = std::lerp(left.opacityMultiply, right.opacityMultiply, amount);
    result.pointSizeAdd = std::lerp(left.pointSizeAdd, right.pointSizeAdd, amount);
    result.pointSizeMultiply = std::lerp(left.pointSizeMultiply, right.pointSizeMultiply, amount);
    result.hueShift = std::lerp(left.hueShift, right.hueShift, amount);
    result.colouriseRed = std::lerp(left.colouriseRed, right.colouriseRed, amount);
    result.colouriseGreen = std::lerp(left.colouriseGreen, right.colouriseGreen, amount);
    result.colouriseBlue = std::lerp(left.colouriseBlue, right.colouriseBlue, amount);
    result.colouriseAmount = std::lerp(left.colouriseAmount, right.colouriseAmount, amount);
    result.gaussianSharpnessBias = std::lerp(
        left.gaussianSharpnessBias,
        right.gaussianSharpnessBias,
        amount);
    return result;
}

WaterSeepageLookSettings LerpSeepageLook(
    const WaterSeepageLookSettings& left,
    const WaterSeepageLookSettings& right,
    float amount) {
    amount = Clamp01(amount);
    WaterSeepageLookSettings result = left;
    result.quality = amount < 0.5F ? left.quality : right.quality;
    result.pattern = left.pattern;
    result.baseWetness = std::lerp(left.baseWetness, right.baseWetness, amount);
    result.density = std::lerp(left.density, right.density, amount);
    result.glisten = std::lerp(left.glisten, right.glisten, amount);
    result.rainResponse = std::lerp(left.rainResponse, right.rainResponse, amount);
    result.featureSizeMeters = std::lerp(left.featureSizeMeters, right.featureSizeMeters, amount);
    result.contrast = std::lerp(left.contrast, right.contrast, amount);
    result.evolution = std::lerp(left.evolution, right.evolution, amount);
    result.roughness = std::lerp(left.roughness, right.roughness, amount);
    result.angleResponse = std::lerp(left.angleResponse, right.angleResponse, amount);
    result.microNormalStrength = std::lerp(
        left.microNormalStrength,
        right.microNormalStrength,
        amount);
    result.glintDensity = std::lerp(left.glintDensity, right.glintDensity, amount);
    result.environmentAzimuthDegrees = LerpSeepageAngle(
        left.environmentAzimuthDegrees,
        right.environmentAzimuthDegrees,
        amount);
    result.environmentElevationDegrees = std::lerp(
        left.environmentElevationDegrees,
        right.environmentElevationDegrees,
        amount);
    result.curl = std::lerp(left.curl, right.curl, amount);
    result.breakup = std::lerp(left.breakup, right.breakup, amount);
    result.downhillDriftMetersPerSecond = std::lerp(
        left.downhillDriftMetersPerSecond,
        right.downhillDriftMetersPerSecond,
        amount);
    result.tricklePatchSizeMeters = std::lerp(
        left.tricklePatchSizeMeters,
        right.tricklePatchSizeMeters,
        amount);
    result.trickleWidthMeters = std::lerp(
        left.trickleWidthMeters,
        right.trickleWidthMeters,
        amount);
    result.trickleFrontSoftness = std::lerp(
        left.trickleFrontSoftness,
        right.trickleFrontSoftness,
        amount);
    result.pulseSpacingMeters = std::lerp(
        left.pulseSpacingMeters,
        right.pulseSpacingMeters,
        amount);
    result.pulseWidthMeters = std::lerp(
        left.pulseWidthMeters,
        right.pulseWidthMeters,
        amount);
    result.pulseSpeedMetersPerSecond = std::lerp(
        left.pulseSpeedMetersPerSecond,
        right.pulseSpeedMetersPerSecond,
        amount);
    result.pulseIrregularity = std::lerp(
        left.pulseIrregularity,
        right.pulseIrregularity,
        amount);
    result.pulseWaveCount = std::lerp(
        left.pulseWaveCount,
        right.pulseWaveCount,
        amount);
    result.pulseSpeedVariation = std::lerp(
        left.pulseSpeedVariation,
        right.pulseSpeedVariation,
        amount);
    result.response = LerpSeepageResponse(left.response, right.response, amount);
    result.blendMode = amount < 0.5F ? left.blendMode : right.blendMode;
    return SanitizeSeepageLook(result);
}

glm::mat3 SeepageNoiseRotation(std::uint32_t seed) {
    const float yaw = SeepageHash01(17, 29, seed ^ 0x45d9f3bU) * kSeepageTwoPi;
    const float pitch = (SeepageHash01(31, 47, seed ^ 0x119de1f3U) - 0.5F) * 1.20F;
    const glm::vec3 forward = glm::normalize(glm::vec3{
        std::cos(yaw) * std::cos(pitch),
        std::sin(yaw) * std::cos(pitch),
        std::sin(pitch),
    });
    const glm::vec3 helper = std::abs(forward.z) < 0.92F
                                 ? glm::vec3{0.0F, 0.0F, 1.0F}
                                 : glm::vec3{0.0F, 1.0F, 0.0F};
    const glm::vec3 right = glm::normalize(glm::cross(helper, forward));
    const glm::vec3 up = glm::normalize(glm::cross(forward, right));
    return glm::mat3{right, up, forward};
}

glm::vec3 SeepageEnvironmentDirection(const WaterSeepageLookSettings& look) {
    constexpr float degreesToRadians = kSeepageTwoPi / 360.0F;
    const float azimuth = look.environmentAzimuthDegrees * degreesToRadians;
    const float elevation = look.environmentElevationDegrees * degreesToRadians;
    return glm::normalize(glm::vec3{
        std::cos(elevation) * std::cos(azimuth),
        std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation),
    });
}

float SanitizeSeepageTimingSeconds(float value) {
    return std::clamp(
        std::isfinite(value) ? value : 0.0F,
        0.0F,
        86'400.0F);
}

WaterScenarioState SanitizeWaterScenarioStateImpl(WaterScenarioState state) {
    state.seepageLevel = Clamp01(SeepageFiniteOr(state.seepageLevel, 1.0F));
    state.seepageSpread = Clamp01(SeepageFiniteOr(state.seepageSpread, 0.0F));
    state.rainLevel = Clamp01(SeepageFiniteOr(state.rainLevel, 0.0F));
    state.flowLevel = Clamp01(SeepageFiniteOr(state.flowLevel, 1.0F));
    state.shorelineLevel = Clamp01(SeepageFiniteOr(state.shorelineLevel, 1.0F));
    state.meshFlowLevel = Clamp01(SeepageFiniteOr(state.meshFlowLevel, 1.0F));
    state.meshFlowRainGain = std::clamp(
        SeepageFiniteOr(state.meshFlowRainGain, 0.0F),
        0.0F,
        4.0F);
    state.meshFlowPersistenceScale = std::clamp(
        SeepageFiniteOr(state.meshFlowPersistenceScale, 1.0F),
        0.0F,
        8.0F);
    state.meshFlowRainRiseSeconds = SanitizeSeepageTimingSeconds(
        state.meshFlowRainRiseSeconds);
    state.meshFlowRainRecessionSeconds = SanitizeSeepageTimingSeconds(
        state.meshFlowRainRecessionSeconds);
    state.seepageRainDelaySeconds = SanitizeSeepageTimingSeconds(
        state.seepageRainDelaySeconds);
    state.seepageRainRiseSeconds = SanitizeSeepageTimingSeconds(
        state.seepageRainRiseSeconds);
    state.seepageRainRecessionSeconds = SanitizeSeepageTimingSeconds(
        state.seepageRainRecessionSeconds);
    state.transitionAmount = Clamp01(SeepageFiniteOr(state.transitionAmount, 0.0F));
    return state;
}

WaterSeepageNodeAnimationState SanitizeSeepageNodeAnimationState(
    WaterSeepageNodeAnimationState state) {
    state.activity = Clamp01(SeepageFiniteOr(state.activity, 1.0F));
    state.localSpread = Clamp01(SeepageFiniteOr(state.localSpread, 0.0F));
    state.wettingProgress = Clamp01(SeepageFiniteOr(state.wettingProgress, 1.0F));
    state.reachScale = std::clamp(SeepageFiniteOr(state.reachScale, 1.0F), 0.0F, 8.0F);
    state.widthScale = std::clamp(SeepageFiniteOr(state.widthScale, 1.0F), 0.0F, 8.0F);
    state.prominence = std::clamp(SeepageFiniteOr(state.prominence, 1.0F), 0.0F, 8.0F);
    const auto sanitizeOverride = [](float value) {
        value = SeepageFiniteOr(value, -1.0F);
        return value < 0.0F ? -1.0F : std::min(value, 8.0F);
    };
    state.strengthOverride = sanitizeOverride(state.strengthOverride);
    state.prominenceOverride = sanitizeOverride(state.prominenceOverride);
    state.sourceWidthOverride = sanitizeOverride(state.sourceWidthOverride);
    state.rainLevelOverride = sanitizeOverride(state.rainLevelOverride);
    if (state.rainLevelOverride >= 0.0F) {
        state.rainLevelOverride = Clamp01(state.rainLevelOverride);
    }
    if (state.lookOverride.has_value()) {
        state.lookOverride = SanitizeSeepageLook(
            std::move(state.lookOverride.value()));
    }
    return state;
}

WaterSeepageNodeAnimationState LerpSeepageNodeAnimationState(
    const WaterSeepageNodeAnimationState& left,
    const WaterSeepageNodeAnimationState& right,
    float amount) {
    amount = Clamp01(amount);
    return SanitizeSeepageNodeAnimationState({
        .activity = std::lerp(left.activity, right.activity, amount),
        .localSpread = std::lerp(left.localSpread, right.localSpread, amount),
        .wettingProgress = std::lerp(left.wettingProgress, right.wettingProgress, amount),
        .reachScale = std::lerp(left.reachScale, right.reachScale, amount),
        .widthScale = std::lerp(left.widthScale, right.widthScale, amount),
        .prominence = std::lerp(left.prominence, right.prominence, amount),
    });
}

void SeepageFingerprintText(std::uint64_t* hash, std::string_view value) {
    SeepageFingerprintU32(hash, static_cast<std::uint32_t>(std::min<std::size_t>(
        value.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))));
    for (const char character : value) {
        SeepageFingerprintByte(hash, static_cast<std::uint8_t>(character));
    }
}

WaterSeepageNodeAnimationState FindSeepageNodeAnimationState(
    std::span<const WaterSeepageNodeAnimationStateEntry> states,
    std::uint32_t nodeId) {
    const auto found = std::find_if(
        states.begin(),
        states.end(),
        [nodeId](const WaterSeepageNodeAnimationStateEntry& candidate) {
            return candidate.nodeId == nodeId;
        });
    return found == states.end()
               ? WaterSeepageNodeAnimationState{}
               : SanitizeSeepageNodeAnimationState(found->state);
}

void ApplySeepageRuntimeScenarioAndAnimation(
    WaterSeepageRuntimeNode* node,
    const std::optional<WaterScenarioState>& scenarioState,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    const WaterSeepageNodeAnimationState& rawNodeState) {
    if (node == nullptr) {
        return;
    }
    const auto nodeState = SanitizeSeepageNodeAnimationState(rawNodeState);
    // Scenario looks are legacy data. Every node begins from its authored
    // profile pair and only an explicit Timings-v2 scalar look overrides it.
    node->look = nodeState.lookOverride.has_value()
                     ? SanitizeSeepageLook(nodeState.lookOverride.value())
                     : SanitizeSeepageLook(node->authoredLook);
    node->resolvedQuality = ResolveWaterSeepageQuality(
        node->look.quality,
        effectivePointInvocations);
    node->look.quality = node->resolvedQuality;
    node->environmentDirection = SeepageEnvironmentDirection(node->look);
    const float scenarioLevel = scenarioState.has_value()
                                    ? Clamp01(SeepageFiniteOr(
                                          scenarioState->seepageLevel,
                                          1.0F))
                                    : 1.0F;
    node->effectiveActivity = nodeState.activity;
    // Global and legacy per-node spread lanes are intentionally inert. The
    // authored Strength and Source Width controls define extent explicitly.
    node->localSpread = 0.0F;
    node->wettingProgress = nodeState.wettingProgress;
    const float baseStrength = nodeState.strengthOverride >= 0.0F
                                   ? nodeState.strengthOverride
                                   : node->authoredStrength;
    node->strength = baseStrength * scenarioLevel * node->effectiveActivity;
    node->scenarioSpread = 0.0F;
    node->rainVisualStrength =
        nodeState.rainLevelOverride >= 0.0F
            ? Clamp01(nodeState.rainLevelOverride)
            : scenarioState.has_value()
                  ? Clamp01(SeepageFiniteOr(
                        scenarioState->rainLevel,
                        0.0F))
                  : (rainSettings.enabled
                         ? WaterRainPresetVisualStrength(
                               rainSettings.intensityPreset)
                         : 0.0F);
    const float rainGain = SeepageRainGain(*node);
    const float reachScale = 1.0F + 0.25F * rainGain;
    const float widthScale = 1.0F + 0.20F * rainGain;
    node->reachMeters = std::clamp(
        node->authoredReachMeters * nodeState.reachScale * reachScale,
        0.0F,
        std::max(0.0F, node->selectionReachLimitMeters));
    const float baseWidth = nodeState.sourceWidthOverride >= 0.0F
                                ? nodeState.sourceWidthOverride
                                : node->authoredWidthMeters;
    node->widthMeters = std::clamp(
        baseWidth * nodeState.widthScale * widthScale,
        0.0F,
        std::max(0.0F, node->selectionWidthLimitMeters));
    const float baseProminence = nodeState.prominenceOverride >= 0.0F
                                     ? nodeState.prominenceOverride
                                     : node->authoredProminence;
    node->prominence = std::clamp(
        baseProminence * nodeState.prominence,
        0.0F,
        8.0F);
    // The connected-support travel budget comes from strength alone (which
    // already folds in scenario level and activity); reach-scale keys and
    // the spread/rain gains keep working by scaling the budget.
    node->budgetMeters = std::clamp(
        kWaterSeepageRunMetersPerStrength * node->strength *
            nodeState.reachScale * reachScale,
        0.0F,
        std::max(0.0F, node->selectionReachLimitMeters));
    node->endHalfWidthMeters = node->widthMeters * 0.5F;
    node->startHalfWidthMeters = std::min(
        node->authoredStartHalfWidthMeters * nodeState.widthScale * widthScale,
        node->endHalfWidthMeters);
    node->transitionLook.reset();
    node->transitionAmount = 0.0F;
}

}  // namespace

WaterScenarioState SanitizeWaterScenarioState(WaterScenarioState state) {
    return SanitizeWaterScenarioStateImpl(std::move(state));
}

std::string WaterDynamicMeshFlowSettingsFingerprint(
    const WaterDynamicMeshFlowSettings& rawSettings) {
    const auto settings = SanitizeWaterDynamicMeshFlowSettings(rawSettings);
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 1U);
    const auto fingerprintBool = [&](bool value) {
        SeepageFingerprintU32(&hash, value ? 1U : 0U);
    };
    const auto fingerprintPoint = [&](const invisible_places::io::Float3& point) {
        SeepageFingerprintFloat(&hash, point.x);
        SeepageFingerprintFloat(&hash, point.y);
        SeepageFingerprintFloat(&hash, point.z);
    };
    fingerprintBool(settings.enabled);
    fingerprintBool(settings.gpuPreviewEnabled);
    fingerprintBool(settings.showTrails);
    SeepageFingerprintText(&hash, settings.meshPath.generic_string());
    SeepageFingerprintFloat(&hash, settings.cacheCellSizeMeters);
    SeepageFingerprintFloat(&hash, settings.projectionSearchRadiusMeters);
    SeepageFingerprintFloat(&hash, settings.ambiguityHeightMeters);
    SeepageFingerprintU32(&hash, settings.particleCapacity);
    SeepageFingerprintU32(&hash, settings.historyLength);
    SeepageFingerprintFloat(&hash, settings.dryConcavityFocus);
    SeepageFingerprintFloat(&hash, settings.edgeCoverage);
    SeepageFingerprintFloat(&hash, settings.activity);
    SeepageFingerprintFloat(&hash, settings.rainGain);
    SeepageFingerprintFloat(
        &hash,
        settings.moisturePersistenceMultiplier);
    SeepageFingerprintFloat(&hash, settings.rainRiseSeconds);
    SeepageFingerprintFloat(&hash, settings.rainRecessionSeconds);
    SeepageFingerprintFloat(&hash, settings.surfaceSurge);
    SeepageFingerprintFloat(&hash, settings.rainSpawnSpread);
    SeepageFingerprintFloat(&hash, settings.rainDistributedSourceFraction);
    SeepageFingerprintU32(&hash, settings.previewParticleLimit);
    SeepageFingerprintU32(&hash, settings.finalParticleLimit);
    SeepageFingerprintFloat(&hash, settings.trailLengthMeters);
    SeepageFingerprintFloat(&hash, settings.stepMeters);
    SeepageFingerprintFloat(&hash, settings.trailWidthMeters);
    SeepageFingerprintFloat(&hash, settings.trailStreakLengthMeters);
    SeepageFingerprintFloat(&hash, settings.surfaceOffsetMeters);
    SeepageFingerprintFloat(&hash, settings.trailOpacityDry);
    SeepageFingerprintFloat(&hash, settings.trailOpacityWet);
    SeepageFingerprintFloat(&hash, settings.trailEmissionDry);
    SeepageFingerprintFloat(&hash, settings.trailEmissionWet);
    SeepageFingerprintFloat(&hash, settings.trailExposure);
    SeepageFingerprintFloat(&hash, settings.speedMetersPerSecond);
    SeepageFingerprintFloat(&hash, settings.downhillWeight);
    SeepageFingerprintFloat(&hash, settings.attractorWeight);
    SeepageFingerprintFloat(&hash, settings.sourceVelocityWeight);
    SeepageFingerprintFloat(&hash, settings.curlStrength);
    SeepageFingerprintFloat(&hash, settings.branchingStrength);
    SeepageFingerprintFloat(&hash, settings.eddyStrength);
    SeepageFingerprintFloat(&hash, settings.topologyResponse);
    SeepageFingerprintFloat(&hash, settings.inertia);
    SeepageFingerprintFloat(&hash, settings.particleNoiseStrength);
    SeepageFingerprintFloat(&hash, settings.particleNoiseScaleMeters);
    SeepageFingerprintFloat(&hash, settings.particleNoiseSpeed);
    SeepageFingerprintFloat(&hash, settings.sharedWindStrength);
    SeepageFingerprintFloat(&hash, settings.sharedWindScaleMeters);
    SeepageFingerprintFloat(&hash, settings.sharedWindSpeed);
    SeepageFingerprintFloat(&hash, settings.contactFadeSeconds);
    SeepageFingerprintFloat(&hash, settings.rockResponse.radiusMeters);
    SeepageFingerprintFloat(&hash, settings.rockResponse.opacityAdd);
    SeepageFingerprintFloat(&hash, settings.rockResponse.emissionAdd);
    fingerprintPoint(settings.rockResponse.colourise);
    SeepageFingerprintFloat(&hash, settings.rockResponse.colouriseAmount);
    SeepageFingerprintFloat(&hash, settings.rockResponse.persistenceSeconds);
    SeepageFingerprintFloat(&hash, settings.vegetationResponse.radiusMeters);
    SeepageFingerprintFloat(&hash, settings.vegetationResponse.opacityAdd);
    SeepageFingerprintFloat(&hash, settings.vegetationResponse.emissionAdd);
    fingerprintPoint(settings.vegetationResponse.colourise);
    SeepageFingerprintFloat(&hash, settings.vegetationResponse.colouriseAmount);
    SeepageFingerprintFloat(&hash, settings.vegetationResponse.persistenceSeconds);
    SeepageFingerprintFloat(&hash, settings.vegetationResponse.twinkle);
    SeepageFingerprintFloat(&hash, settings.vegetationResponse.streamDepthMeters);
    SeepageFingerprintFloat(&hash, settings.animationDurationSeconds);
    SeepageFingerprintU32(&hash, settings.seed);
    SeepageFingerprintText(&hash, settings.particlePresetName);
    return "water-dynamic-mesh-flow-settings-v3-" + SeepageFingerprintString(hash);
}

WaterSeepageLookSettings DefaultWaterSeepageLookSettings() {
    return {};
}

bool ApplyWaterSeepageLookTimingValue(
    WaterSeepageLookSettings* look,
    std::string_view settingId,
    float value) {
    if (look == nullptr || !std::isfinite(value)) {
        return false;
    }
    float* target = nullptr;
    if (settingId == "look.base_wetness") {
        target = &look->baseWetness;
    } else if (settingId == "look.density") {
        target = &look->density;
    } else if (settingId == "look.glisten") {
        target = &look->glisten;
    } else if (settingId == "look.rain_response") {
        target = &look->rainResponse;
    } else if (settingId == "look.feature_size") {
        target = &look->featureSizeMeters;
    } else if (settingId == "look.contrast") {
        target = &look->contrast;
    } else if (settingId == "look.evolution") {
        target = &look->evolution;
    } else if (settingId == "look.roughness") {
        target = &look->roughness;
    } else if (settingId == "look.angle_response") {
        target = &look->angleResponse;
    } else if (settingId == "look.micro_normal_strength") {
        target = &look->microNormalStrength;
    } else if (settingId == "look.glint_density") {
        target = &look->glintDensity;
    } else if (settingId == "look.environment_azimuth") {
        target = &look->environmentAzimuthDegrees;
    } else if (settingId == "look.environment_elevation") {
        target = &look->environmentElevationDegrees;
    } else if (settingId == "look.curl") {
        target = &look->curl;
    } else if (settingId == "look.breakup") {
        target = &look->breakup;
    } else if (settingId == "look.downhill_drift") {
        target = &look->downhillDriftMetersPerSecond;
    } else if (settingId == "look.trickle_patch_size") {
        target = &look->tricklePatchSizeMeters;
    } else if (settingId == "look.trickle_width") {
        target = &look->trickleWidthMeters;
    } else if (settingId == "look.trickle_front_softness") {
        target = &look->trickleFrontSoftness;
    } else if (settingId == "look.pulse_spacing") {
        target = &look->pulseSpacingMeters;
    } else if (settingId == "look.pulse_width") {
        target = &look->pulseWidthMeters;
    } else if (settingId == "look.pulse_speed") {
        target = &look->pulseSpeedMetersPerSecond;
    } else if (settingId == "look.pulse_irregularity") {
        target = &look->pulseIrregularity;
    } else if (settingId == "look.pulse_wave_count") {
        target = &look->pulseWaveCount;
    } else if (settingId == "look.pulse_speed_variation") {
        target = &look->pulseSpeedVariation;
    } else if (settingId == "response.intensity") {
        target = &look->response.intensity;
    } else if (settingId == "response.emission_add") {
        target = &look->response.emissionAdd;
    } else if (settingId == "response.opacity_add") {
        target = &look->response.opacityAdd;
    } else if (settingId == "response.opacity_multiply") {
        target = &look->response.opacityMultiply;
    } else if (settingId == "response.point_size_add") {
        target = &look->response.pointSizeAdd;
    } else if (settingId == "response.point_size_multiply") {
        target = &look->response.pointSizeMultiply;
    } else if (settingId == "response.hue_shift") {
        target = &look->response.hueShift;
    } else if (settingId == "response.colourise_red") {
        target = &look->response.colouriseRed;
    } else if (settingId == "response.colourise_green") {
        target = &look->response.colouriseGreen;
    } else if (settingId == "response.colourise_blue") {
        target = &look->response.colouriseBlue;
    } else if (settingId == "response.colourise_amount") {
        target = &look->response.colouriseAmount;
    } else if (settingId == "response.gaussian_sharpness_bias") {
        target = &look->response.gaussianSharpnessBias;
    }
    if (target == nullptr) {
        return false;
    }
    *target = value;
    *look = SanitizeSeepageLook(std::move(*look));
    return true;
}

std::vector<WaterScenarioDefinition> DefaultWaterScenarioDefinitions() {
    WaterScenarioDefinition historical;
    historical.id = "pre-colonisation-wet";
    historical.name = "Past/Future";
    historical.state.seepageLook = DefaultWaterSeepageLookSettings();
    historical.state.seepageLook.pattern = WaterSeepagePattern::ChaoticBloom;
    historical.state.seepageLook.baseWetness = 0.52F;
    historical.state.seepageLook.density = 0.55F;
    historical.state.seepageLook.glisten = 0.62F;
    historical.state.seepageLook.featureSizeMeters = 0.20F;
    historical.state.seepageLook.evolution = 0.08F;
    historical.state.seepageLook.rainResponse = 0.90F;
    historical.state.seepageLook.response.intensity = 0.90F;
    historical.state.seepageLevel = 1.0F;
    historical.state.seepageSpread = 0.60F;
    historical.state.rainLevel = 0.0F;
    historical.state.meshFlowLevel = 0.45F;
    historical.state.meshFlowRainGain = 1.0F;
    historical.state.meshFlowPersistenceScale = 1.0F;
    historical.state.meshFlowRainRiseSeconds = 8.0F;
    historical.state.meshFlowRainRecessionSeconds = 75.0F;
    historical.state.seepageRainDelaySeconds = 4.0F;
    historical.state.seepageRainRiseSeconds = 12.0F;
    historical.state.seepageRainRecessionSeconds = 60.0F;

    WaterScenarioDefinition contemporary;
    contemporary.id = "contemporary-managed";
    contemporary.name = "Current";
    contemporary.state.seepageLook = historical.state.seepageLook;
    contemporary.state.seepageLook.baseWetness = 0.20F;
    contemporary.state.seepageLook.density = 0.28F;
    contemporary.state.seepageLook.glisten = 0.32F;
    contemporary.state.seepageLook.evolution = 0.02F;
    contemporary.state.seepageLook.rainResponse = 0.18F;
    contemporary.state.seepageLook.response.intensity = 0.55F;
    contemporary.state.seepageLevel = 0.50F;
    contemporary.state.seepageSpread = 0.10F;
    contemporary.state.rainLevel = 0.0F;
    contemporary.state.meshFlowLevel = 0.18F;
    contemporary.state.meshFlowRainGain = 0.30F;
    contemporary.state.meshFlowPersistenceScale = 1.0F;
    contemporary.state.meshFlowRainRiseSeconds = 3.0F;
    contemporary.state.meshFlowRainRecessionSeconds = 18.0F;
    contemporary.state.seepageRainDelaySeconds = 1.0F;
    contemporary.state.seepageRainRiseSeconds = 4.0F;
    contemporary.state.seepageRainRecessionSeconds = 15.0F;
    return {historical, contemporary};
}

WaterScenarioState EvaluateWaterScenarioTrack(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float normalizedPosition) {
    WaterScenarioState result = SanitizeWaterScenarioStateImpl(definition.state);
    result.transitionLook.reset();
    result.transitionAmount = 0.0F;
    if (track.keys.empty()) {
        return result;
    }
    normalizedPosition = Clamp01(normalizedPosition);
    std::vector<const WaterScenarioKey*> ordered;
    ordered.reserve(track.keys.size());
    for (const auto& key : track.keys) {
        ordered.push_back(&key);
    }
    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [](const WaterScenarioKey* left, const WaterScenarioKey* right) {
            return left->position < right->position;
        });
    if (normalizedPosition <= ordered.front()->position) {
        return SanitizeWaterScenarioStateImpl(ordered.front()->state);
    }
    if (normalizedPosition >= ordered.back()->position) {
        return SanitizeWaterScenarioStateImpl(ordered.back()->state);
    }
    for (std::size_t index = 0U; index + 1U < ordered.size(); ++index) {
        const auto& left = *ordered[index];
        const auto& right = *ordered[index + 1U];
        if (normalizedPosition > right.position) {
            continue;
        }
        const float span = std::max(1.0e-6F, right.position - left.position);
        float amount = Clamp01((normalizedPosition - left.position) / span);
        if (left.interpolation == WaterScenarioInterpolation::Hold) {
            return SanitizeWaterScenarioStateImpl(left.state);
        }
        // Scenario keys blend whole multi-channel states, so the spline modes
        // approximate as the eased blend; a per-channel spline is not
        // applicable here.
        if (left.interpolation == WaterScenarioInterpolation::Smooth ||
            left.interpolation == WaterScenarioInterpolation::SplineHandles ||
            left.interpolation == WaterScenarioInterpolation::SmoothVelocity ||
            left.interpolation ==
                WaterScenarioInterpolation::CentripetalCatmullRom) {
            amount = amount * amount * (3.0F - 2.0F * amount);
        }
        result.seepageLevel = std::lerp(left.state.seepageLevel, right.state.seepageLevel, amount);
        result.seepageSpread = std::lerp(left.state.seepageSpread, right.state.seepageSpread, amount);
        result.rainLevel = std::lerp(left.state.rainLevel, right.state.rainLevel, amount);
        result.flowLevel = std::lerp(left.state.flowLevel, right.state.flowLevel, amount);
        result.shorelineLevel = std::lerp(
            left.state.shorelineLevel,
            right.state.shorelineLevel,
            amount);
        result.meshFlowLevel = std::lerp(
            left.state.meshFlowLevel,
            right.state.meshFlowLevel,
            amount);
        result.meshFlowRainGain = std::lerp(
            left.state.meshFlowRainGain,
            right.state.meshFlowRainGain,
            amount);
        result.meshFlowPersistenceScale = std::lerp(
            left.state.meshFlowPersistenceScale,
            right.state.meshFlowPersistenceScale,
            amount);
        result.meshFlowRainRiseSeconds = std::lerp(
            SanitizeSeepageTimingSeconds(left.state.meshFlowRainRiseSeconds),
            SanitizeSeepageTimingSeconds(right.state.meshFlowRainRiseSeconds),
            amount);
        result.meshFlowRainRecessionSeconds = std::lerp(
            SanitizeSeepageTimingSeconds(left.state.meshFlowRainRecessionSeconds),
            SanitizeSeepageTimingSeconds(right.state.meshFlowRainRecessionSeconds),
            amount);
        result.seepageRainDelaySeconds = std::lerp(
            SanitizeSeepageTimingSeconds(left.state.seepageRainDelaySeconds),
            SanitizeSeepageTimingSeconds(right.state.seepageRainDelaySeconds),
            amount);
        result.seepageRainRiseSeconds = std::lerp(
            SanitizeSeepageTimingSeconds(left.state.seepageRainRiseSeconds),
            SanitizeSeepageTimingSeconds(right.state.seepageRainRiseSeconds),
            amount);
        result.seepageRainRecessionSeconds = std::lerp(
            SanitizeSeepageTimingSeconds(left.state.seepageRainRecessionSeconds),
            SanitizeSeepageTimingSeconds(right.state.seepageRainRecessionSeconds),
            amount);
        if (left.state.seepageLook.pattern == right.state.seepageLook.pattern) {
            result.seepageLook = LerpSeepageLook(
                left.state.seepageLook,
                right.state.seepageLook,
                amount);
        } else {
            result.seepageLook = SanitizeSeepageLook(left.state.seepageLook);
            result.transitionLook = SanitizeSeepageLook(right.state.seepageLook);
            result.transitionAmount = amount;
        }
        return SanitizeWaterScenarioStateImpl(result);
    }
    return SanitizeWaterScenarioStateImpl(ordered.back()->state);
}

WaterSeepageNodeAnimationState EvaluateWaterSeepageNodeAnimationTrack(
    const WaterScenarioTrack& track,
    std::uint32_t nodeId,
    float normalizedPosition) {
    const auto nodeTrack = std::find_if(
        track.seepageNodeTracks.begin(),
        track.seepageNodeTracks.end(),
        [nodeId](const WaterSeepageNodeTrack& candidate) {
            return candidate.nodeId == nodeId;
        });
    if (nodeTrack == track.seepageNodeTracks.end() || nodeTrack->keys.empty()) {
        return {};
    }

    normalizedPosition = Clamp01(SeepageFiniteOr(normalizedPosition, 0.0F));
    std::vector<const WaterSeepageNodeKey*> ordered;
    ordered.reserve(nodeTrack->keys.size());
    for (const auto& key : nodeTrack->keys) {
        ordered.push_back(&key);
    }
    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [](const WaterSeepageNodeKey* left, const WaterSeepageNodeKey* right) {
            return left->position < right->position;
        });
    if (normalizedPosition <= ordered.front()->position) {
        return SanitizeSeepageNodeAnimationState(ordered.front()->state);
    }
    if (normalizedPosition >= ordered.back()->position) {
        return SanitizeSeepageNodeAnimationState(ordered.back()->state);
    }
    for (std::size_t index = 0U; index + 1U < ordered.size(); ++index) {
        const auto& left = *ordered[index];
        const auto& right = *ordered[index + 1U];
        if (normalizedPosition > right.position) {
            continue;
        }
        if (left.interpolation == WaterScenarioInterpolation::Hold) {
            return SanitizeSeepageNodeAnimationState(left.state);
        }
        const float span = std::max(1.0e-6F, right.position - left.position);
        float amount = Clamp01((normalizedPosition - left.position) / span);
        // Node keys blend whole multi-channel states, so the spline modes
        // approximate as the eased blend; a per-channel spline is not
        // applicable here.
        if (left.interpolation == WaterScenarioInterpolation::Smooth ||
            left.interpolation == WaterScenarioInterpolation::SplineHandles ||
            left.interpolation == WaterScenarioInterpolation::SmoothVelocity ||
            left.interpolation ==
                WaterScenarioInterpolation::CentripetalCatmullRom) {
            amount = amount * amount * (3.0F - 2.0F * amount);
        }
        return LerpSeepageNodeAnimationState(left.state, right.state, amount);
    }
    return SanitizeSeepageNodeAnimationState(ordered.back()->state);
}

std::vector<WaterSeepageNodeAnimationStateEntry> EvaluateWaterSeepageNodeAnimationTracks(
    const WaterScenarioTrack& track,
    float normalizedPosition) {
    std::vector<WaterSeepageNodeAnimationStateEntry> states;
    states.reserve(track.seepageNodeTracks.size());
    std::unordered_set<std::uint32_t> emitted;
    emitted.reserve(track.seepageNodeTracks.size());
    for (const auto& nodeTrack : track.seepageNodeTracks) {
        if (!emitted.insert(nodeTrack.nodeId).second) {
            continue;
        }
        states.push_back({
            .nodeId = nodeTrack.nodeId,
            .state = EvaluateWaterSeepageNodeAnimationTrack(
                track,
                nodeTrack.nodeId,
                normalizedPosition),
        });
    }
    std::sort(
        states.begin(),
        states.end(),
        [](const WaterSeepageNodeAnimationStateEntry& left,
           const WaterSeepageNodeAnimationStateEntry& right) {
            return left.nodeId < right.nodeId;
        });
    return states;
}

void AddOrUpdateWaterSeepageNodeKey(
    WaterScenarioTrack* track,
    std::uint32_t nodeId,
    WaterSeepageNodeKey key,
    float replacementTolerance) {
    if (track == nullptr) {
        return;
    }
    auto nodeTrack = std::find_if(
        track->seepageNodeTracks.begin(),
        track->seepageNodeTracks.end(),
        [nodeId](const WaterSeepageNodeTrack& candidate) {
            return candidate.nodeId == nodeId;
        });
    if (nodeTrack == track->seepageNodeTracks.end()) {
        track->seepageNodeTracks.push_back({.nodeId = nodeId});
        nodeTrack = std::prev(track->seepageNodeTracks.end());
    }
    key.position = Clamp01(SeepageFiniteOr(key.position, 0.0F));
    key.state = SanitizeSeepageNodeAnimationState(key.state);
    replacementTolerance = std::max(
        0.0F,
        SeepageFiniteOr(replacementTolerance, 0.0001F));
    const auto existing = std::find_if(
        nodeTrack->keys.begin(),
        nodeTrack->keys.end(),
        [&](const WaterSeepageNodeKey& candidate) {
            return std::abs(candidate.position - key.position) <= replacementTolerance;
        });
    if (existing != nodeTrack->keys.end()) {
        const auto preservedId = existing->id;
        *existing = std::move(key);
        if (existing->id.empty()) {
            existing->id = preservedId;
        }
    } else {
        nodeTrack->keys.push_back(std::move(key));
    }
    std::stable_sort(
        nodeTrack->keys.begin(),
        nodeTrack->keys.end(),
        [](const WaterSeepageNodeKey& left, const WaterSeepageNodeKey& right) {
            return left.position < right.position;
        });
}

std::string WaterSeepageRainEnvelopeFingerprint(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 1U);
    SeepageFingerprintText(&hash, definition.id);
    SeepageFingerprintText(&hash, track.scenarioId);
    SeepageFingerprintFloat(
        &hash,
        std::max(0.0F, SeepageFiniteOr(durationSeconds, 0.0F)));
    const auto fingerprintState = [&](const WaterScenarioState& rawState) {
        const auto state = SanitizeWaterScenarioStateImpl(rawState);
        SeepageFingerprintFloat(&hash, Clamp01(SeepageFiniteOr(state.rainLevel, 0.0F)));
        SeepageFingerprintFloat(&hash, state.seepageRainDelaySeconds);
        SeepageFingerprintFloat(&hash, state.seepageRainRiseSeconds);
        SeepageFingerprintFloat(&hash, state.seepageRainRecessionSeconds);
    };
    fingerprintState(definition.state);
    SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(std::min<std::size_t>(
        track.keys.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))));
    for (const auto& key : track.keys) {
        SeepageFingerprintText(&hash, key.id);
        SeepageFingerprintFloat(&hash, SeepageFiniteOr(key.position, 0.0F));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(key.interpolation));
        fingerprintState(key.state);
    }
    return "water-seepage-rain-envelope-v1-" + SeepageFingerprintString(hash);
}

std::string WaterDynamicMeshFlowScenarioFingerprint(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 1U);
    SeepageFingerprintText(&hash, definition.id);
    SeepageFingerprintText(&hash, track.scenarioId);
    const auto fingerprintState = [&](const WaterScenarioState& rawState) {
        const auto state = SanitizeWaterScenarioStateImpl(rawState);
        SeepageFingerprintFloat(&hash, state.rainLevel);
        SeepageFingerprintFloat(&hash, state.meshFlowLevel);
        SeepageFingerprintFloat(&hash, state.meshFlowRainGain);
        SeepageFingerprintFloat(&hash, state.meshFlowPersistenceScale);
        SeepageFingerprintFloat(&hash, state.meshFlowRainRiseSeconds);
        SeepageFingerprintFloat(&hash, state.meshFlowRainRecessionSeconds);
    };
    fingerprintState(definition.state);
    SeepageFingerprintU32(
        &hash,
        static_cast<std::uint32_t>(std::min<std::size_t>(
            track.keys.size(),
            static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))));
    for (const auto& key : track.keys) {
        SeepageFingerprintText(&hash, key.id);
        SeepageFingerprintFloat(
            &hash,
            Clamp01(SeepageFiniteOr(key.position, 0.0F)));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(key.interpolation));
        fingerprintState(key.state);
    }
    return "water-dynamic-mesh-flow-scenario-v1-" +
           SeepageFingerprintString(hash);
}

std::string WaterMeshFlowRainEnvelopeFingerprint(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 1U);
    SeepageFingerprintText(
        &hash,
        WaterDynamicMeshFlowScenarioFingerprint(track, definition));
    SeepageFingerprintFloat(
        &hash,
        std::clamp(
            SeepageFiniteOr(durationSeconds, 0.0F),
            0.0F,
            86'400.0F));
    return "water-mesh-flow-rain-envelope-v1-" + SeepageFingerprintString(hash);
}

WaterSeepageRainEnvelope BuildWaterSeepageRainEnvelope(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds,
    float sampleRateHz,
    std::size_t maxSamples) {
    WaterSeepageRainEnvelope envelope;
    envelope.durationSeconds = std::max(
        0.0F,
        SeepageFiniteOr(durationSeconds, 0.0F));
    const float requestedRate = std::clamp(
        SeepageFiniteOr(sampleRateHz, 120.0F),
        1.0F,
        10'000.0F);
    const std::size_t safeMaximumSamples = std::max<std::size_t>(1U, maxSamples);
    const double requestedIntervals = std::ceil(
        static_cast<double>(envelope.durationSeconds) *
        static_cast<double>(requestedRate));
    const std::size_t intervalCount = static_cast<std::size_t>(std::min(
        requestedIntervals,
        static_cast<double>(safeMaximumSamples - 1U)));
    const std::size_t sampleCount = intervalCount + 1U;
    envelope.sampleRateHz = envelope.durationSeconds > 0.0F && intervalCount > 0U
                                ? static_cast<float>(intervalCount) /
                                      envelope.durationSeconds
                                : requestedRate;
    envelope.samples.resize(sampleCount, 0.0F);
    envelope.fingerprint = WaterSeepageRainEnvelopeFingerprint(
        track,
        definition,
        envelope.durationSeconds);

    const float timeStep = sampleCount > 1U
                               ? envelope.durationSeconds /
                                     static_cast<float>(sampleCount - 1U)
                               : 0.0F;
    float filtered = 0.0F;
    for (std::size_t index = 0U; index < sampleCount; ++index) {
        const float timeSeconds = timeStep * static_cast<float>(index);
        const float normalizedPosition = envelope.durationSeconds > 1.0e-6F
                                             ? timeSeconds / envelope.durationSeconds
                                             : 0.0F;
        const auto timingState = EvaluateWaterScenarioTrack(
            track,
            definition,
            normalizedPosition);
        const float delayedTime = std::max(
            0.0F,
            timeSeconds - timingState.seepageRainDelaySeconds);
        const float delayedPosition = envelope.durationSeconds > 1.0e-6F
                                          ? delayedTime / envelope.durationSeconds
                                          : 0.0F;
        const float target = Clamp01(SeepageFiniteOr(
            EvaluateWaterScenarioTrack(track, definition, delayedPosition).rainLevel,
            0.0F));
        const float responseSeconds = target >= filtered
                                          ? timingState.seepageRainRiseSeconds
                                          : timingState.seepageRainRecessionSeconds;
        if (responseSeconds <= 1.0e-6F || timeStep <= 0.0F) {
            filtered = target;
        } else if (index == 0U) {
            filtered = 0.0F;
        } else {
            const float response = 1.0F - std::exp(-timeStep / responseSeconds);
            filtered = std::lerp(filtered, target, Clamp01(response));
        }
        envelope.samples[index] = Clamp01(filtered);
    }
    return envelope;
}

float EvaluateWaterSeepageRainEnvelope(
    const WaterSeepageRainEnvelope& envelope,
    float timeSeconds) {
    if (envelope.samples.empty()) {
        return 0.0F;
    }
    if (envelope.samples.size() == 1U || envelope.durationSeconds <= 1.0e-6F) {
        return Clamp01(SeepageFiniteOr(envelope.samples.front(), 0.0F));
    }
    const float clampedTime = std::clamp(
        SeepageFiniteOr(timeSeconds, 0.0F),
        0.0F,
        envelope.durationSeconds);
    const float sampleCoordinate =
        clampedTime * static_cast<float>(envelope.samples.size() - 1U) /
        envelope.durationSeconds;
    const auto leftIndex = static_cast<std::size_t>(std::floor(sampleCoordinate));
    const auto rightIndex = std::min<std::size_t>(leftIndex + 1U, envelope.samples.size() - 1U);
    const float amount = sampleCoordinate - static_cast<float>(leftIndex);
    return Clamp01(std::lerp(
        SeepageFiniteOr(envelope.samples[leftIndex], 0.0F),
        SeepageFiniteOr(envelope.samples[rightIndex], 0.0F),
        amount));
}

WaterMeshFlowRainEnvelope BuildWaterMeshFlowRainEnvelope(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds,
    float sampleRateHz,
    std::size_t maxSamples) {
    WaterMeshFlowRainEnvelope envelope;
    envelope.durationSeconds = std::clamp(
        SeepageFiniteOr(durationSeconds, 0.0F),
        0.0F,
        86'400.0F);
    const float requestedRate = std::clamp(
        SeepageFiniteOr(sampleRateHz, 120.0F),
        1.0F,
        10'000.0F);
    const std::size_t safeMaximumSamples = std::max<std::size_t>(1U, maxSamples);
    const double requestedIntervals = std::ceil(
        static_cast<double>(envelope.durationSeconds) *
        static_cast<double>(requestedRate));
    const std::size_t intervalCount = static_cast<std::size_t>(std::min(
        requestedIntervals,
        static_cast<double>(safeMaximumSamples - 1U)));
    const std::size_t sampleCount = intervalCount + 1U;
    envelope.sampleRateHz = envelope.durationSeconds > 0.0F && intervalCount > 0U
                                ? static_cast<float>(intervalCount) /
                                      envelope.durationSeconds
                                : requestedRate;
    envelope.samples.resize(sampleCount, 0.0F);
    envelope.fingerprint = WaterMeshFlowRainEnvelopeFingerprint(
        track,
        definition,
        envelope.durationSeconds);

    const float timeStep = sampleCount > 1U
                               ? envelope.durationSeconds /
                                     static_cast<float>(sampleCount - 1U)
                               : 0.0F;
    float filtered = 0.0F;
    for (std::size_t index = 0U; index < sampleCount; ++index) {
        const float timeSeconds = timeStep * static_cast<float>(index);
        const float normalizedPosition = envelope.durationSeconds > 1.0e-6F
                                             ? timeSeconds / envelope.durationSeconds
                                             : 0.0F;
        const auto state = EvaluateWaterScenarioTrack(
            track,
            definition,
            normalizedPosition);
        const float target = state.rainLevel;
        const float responseSeconds = std::clamp(
            (target >= filtered
                 ? state.meshFlowRainRiseSeconds
                 : state.meshFlowRainRecessionSeconds) *
                state.meshFlowPersistenceScale,
            0.0F,
            86'400.0F);
        if (responseSeconds <= 1.0e-6F || timeStep <= 0.0F) {
            filtered = target;
        } else if (index == 0U) {
            filtered = 0.0F;
        } else {
            const float response = 1.0F - std::exp(-timeStep / responseSeconds);
            filtered = std::lerp(filtered, target, Clamp01(response));
        }
        envelope.samples[index] = Clamp01(filtered);
    }
    return envelope;
}

float EvaluateWaterMeshFlowRainEnvelope(
    const WaterMeshFlowRainEnvelope& envelope,
    float timeSeconds) {
    if (envelope.samples.empty()) {
        return 0.0F;
    }
    if (envelope.samples.size() == 1U || envelope.durationSeconds <= 1.0e-6F) {
        return Clamp01(SeepageFiniteOr(envelope.samples.front(), 0.0F));
    }
    const float clampedTime = std::clamp(
        SeepageFiniteOr(timeSeconds, 0.0F),
        0.0F,
        envelope.durationSeconds);
    const float sampleCoordinate =
        clampedTime * static_cast<float>(envelope.samples.size() - 1U) /
        envelope.durationSeconds;
    const auto leftIndex = static_cast<std::size_t>(std::floor(sampleCoordinate));
    const auto rightIndex = std::min<std::size_t>(
        leftIndex + 1U,
        envelope.samples.size() - 1U);
    const float amount = sampleCoordinate - static_cast<float>(leftIndex);
    return Clamp01(std::lerp(
        SeepageFiniteOr(envelope.samples[leftIndex], 0.0F),
        SeepageFiniteOr(envelope.samples[rightIndex], 0.0F),
        amount));
}

std::uint64_t WaterMeshFlowSampleTick(
    float sampleTimeSeconds,
    float fixedStepSeconds) {
    const float fixedStep = std::clamp(
        SeepageFiniteOr(fixedStepSeconds, 1.0F / 30.0F),
        1.0F / 240.0F,
        1.0F / 15.0F);
    const double sampleTime = static_cast<double>(std::clamp(
        SeepageFiniteOr(sampleTimeSeconds, 0.0F),
        0.0F,
        86'400.0F));
    const double coordinate =
        sampleTime / static_cast<double>(fixedStep);
    const double nearestTick = std::round(coordinate);
    if (std::abs(coordinate - nearestTick) <= 1.0e-3) {
        return static_cast<std::uint64_t>(
            std::max(0.0, nearestTick));
    }
    return static_cast<std::uint64_t>(
        std::floor(coordinate));
}

std::vector<WaterMeshFlowTimelineStep> BuildWaterMeshFlowSampleTimeline(
    std::optional<std::uint64_t> previousCompletedTick,
    float targetSampleTimeSeconds,
    std::uint32_t historyLength,
    float fixedStepSeconds) {
    const float fixedStep = std::clamp(
        SeepageFiniteOr(fixedStepSeconds, 1.0F / 30.0F),
        1.0F / 240.0F,
        1.0F / 15.0F);
    const std::uint32_t safeHistoryLength =
        std::clamp(historyLength, 2U, 128U);
    const std::uint64_t targetTick =
        WaterMeshFlowSampleTick(
            targetSampleTimeSeconds,
            fixedStep);
    if (previousCompletedTick == targetTick) {
        return {};
    }

    const bool reset =
        !previousCompletedTick.has_value() ||
        targetTick < previousCompletedTick.value() ||
        targetTick - previousCompletedTick.value() >
            safeHistoryLength;
    const std::uint64_t startTick =
        reset
            ? targetTick > safeHistoryLength
                  ? targetTick - safeHistoryLength
                  : 0U
            : previousCompletedTick.value();

    std::vector<WaterMeshFlowTimelineStep> steps;
    if (reset) {
        steps.push_back({
            .timeSeconds =
                static_cast<float>(startTick) * fixedStep,
            .deltaSeconds = 0.0F,
            .resetSimulation = true,
        });
    }
    steps.reserve(
        steps.size() +
        static_cast<std::size_t>(targetTick - startTick));
    for (std::uint64_t tick = startTick + 1U;
         tick <= targetTick;
         ++tick) {
        steps.push_back({
            .timeSeconds =
                static_cast<float>(tick) * fixedStep,
            .deltaSeconds = fixedStep,
            .resetSimulation = false,
        });
    }
    return steps;
}

float EffectiveWaterDynamicMeshFlowLevel(
    const WaterScenarioState& rawState,
    float effectiveRainLevel) {
    const auto state = SanitizeWaterScenarioStateImpl(rawState);
    const float rain = Clamp01(SeepageFiniteOr(effectiveRainLevel, 0.0F));
    return Clamp01(
        state.meshFlowLevel +
        (1.0F - state.meshFlowLevel) * rain * state.meshFlowRainGain);
}

float EffectiveWaterDynamicMeshPersistenceSeconds(
    float authoredPersistenceSeconds,
    const WaterScenarioState& rawState) {
    const auto state = SanitizeWaterScenarioStateImpl(rawState);
    const float authored = std::clamp(
        SeepageFiniteOr(authoredPersistenceSeconds, 0.0F),
        0.0F,
        86'400.0F);
    return std::min(86'400.0F, authored * state.meshFlowPersistenceScale);
}

float EffectiveWaterFlowActivity(
    const WaterScenarioState& state,
    float maximumFlowStrength,
    float rainResponse,
    bool sourceShowTrail,
    bool globalShowTrails) {
    if (!sourceShowTrail || !globalShowTrails) {
        return 0.0F;
    }
    const float flowLevel = std::isfinite(state.flowLevel) ? Clamp01(state.flowLevel) : 1.0F;
    const float rainLevel = std::isfinite(state.rainLevel) ? Clamp01(state.rainLevel) : 0.0F;
    const float maximumStrength =
        std::isfinite(maximumFlowStrength) ? Clamp01(maximumFlowStrength) : 1.0F;
    const float response = std::isfinite(rainResponse) ? Clamp01(rainResponse) : 0.0F;
    return Clamp01(
        maximumStrength *
        (flowLevel + (1.0F - flowLevel) * rainLevel * response));
}

float EffectiveAuthoredWaterFlowActivity(
    float effectiveRainLevel,
    float maximumFlowStrength,
    float rainResponse,
    bool sourceShowTrail,
    bool globalShowTrails) {
    if (!sourceShowTrail || !globalShowTrails) {
        return 0.0F;
    }
    const float rain = Clamp01(
        SeepageFiniteOr(effectiveRainLevel, 0.0F));
    const float maximumStrength = Clamp01(
        SeepageFiniteOr(maximumFlowStrength, 1.0F));
    const float response = Clamp01(
        SeepageFiniteOr(rainResponse, 0.0F));
    return Clamp01(
        maximumStrength *
        std::lerp(1.0F - response, 1.0F, rain));
}

void AddOrUpdateWaterScenarioKey(
    WaterScenarioTrack* track,
    WaterScenarioKey key,
    float replacementTolerance) {
    if (track == nullptr) {
        return;
    }
    key.position = Clamp01(SeepageFiniteOr(key.position, 0.0F));
    key.state = SanitizeWaterScenarioStateImpl(std::move(key.state));
    replacementTolerance = std::max(0.0F, replacementTolerance);
    const auto existing = std::find_if(
        track->keys.begin(),
        track->keys.end(),
        [&](const WaterScenarioKey& candidate) {
            return std::abs(candidate.position - key.position) <= replacementTolerance;
        });
    if (existing != track->keys.end()) {
        const auto preservedId = existing->id;
        *existing = std::move(key);
        if (existing->id.empty()) {
            existing->id = preservedId;
        }
    } else {
        track->keys.push_back(std::move(key));
    }
    std::stable_sort(
        track->keys.begin(),
        track->keys.end(),
        [](const WaterScenarioKey& left, const WaterScenarioKey& right) {
            return left.position < right.position;
        });
}

const char* WaterTimingFeatureLabel(WaterTimingFeature feature) {
    switch (feature) {
        case WaterTimingFeature::Shoreline:
            return "Shoreline";
        case WaterTimingFeature::Seepage:
            return "Seepage";
        case WaterTimingFeature::Rain:
            return "Rain";
        case WaterTimingFeature::Flow:
            return "Flow";
        case WaterTimingFeature::MeshFlow:
            return "Mesh Flow";
    }
    return "Rain";
}

WaterTimingRun SanitizeWaterTimingRun(WaterTimingRun run) {
    for (auto& key : run.keys) {
        key.position = Clamp01(SeepageFiniteOr(key.position, 0.0F));
        key.level = Clamp01(SeepageFiniteOr(key.level, 1.0F));
    }
    std::stable_sort(
        run.keys.begin(),
        run.keys.end(),
        [](const WaterTimingKey& left, const WaterTimingKey& right) {
            return left.position < right.position;
        });
    return run;
}

namespace {

// Spline segment evaluation shared by the legacy v1 timing runs and the v2
// keyed setting tracks. Keys arrive as position-ordered pointers; valueAt
// bridges the differing value fields (WaterTimingKey::level versus
// WaterSettingKey::value). The math mirrors the TimingColourise scalar key
// track evaluator so both editors animate identically.
constexpr float kWaterKeySplineTolerance = 1.0e-4F;
constexpr float kWaterSettingSplineHandleDefaultFraction = 1.0F / 3.0F;
constexpr float kWaterSettingSplineHandleMinimumFraction = 0.01F;
constexpr float kWaterSettingSplineHandleMaximumPair = 0.98F;

struct WaterSettingSplineHandleFractions {
    float outgoing = kWaterSettingSplineHandleDefaultFraction;
    float incoming = kWaterSettingSplineHandleDefaultFraction;
};

WaterScenarioInterpolation ResolveWaterSettingInterpolation(
    const WaterKeyedSettingTrack& track,
    const WaterSettingKey& key) {
    if (key.interpolation != WaterScenarioInterpolation::TrackDefault) {
        return key.interpolation;
    }
    return track.defaultInterpolation == WaterScenarioInterpolation::TrackDefault
               ? WaterScenarioInterpolation::SmoothVelocity
               : track.defaultInterpolation;
}

WaterSettingSplineHandleFractions ResolveWaterSettingSplineHandleFractions(
    const WaterSettingKey& left,
    const WaterSettingKey& right) {
    WaterSettingSplineHandleFractions result{
        .outgoing = std::clamp(
            SeepageFiniteOr(
                left.outgoingHandleTime,
                kWaterSettingSplineHandleDefaultFraction),
            kWaterSettingSplineHandleMinimumFraction,
            kWaterSettingSplineHandleMaximumPair -
                kWaterSettingSplineHandleMinimumFraction),
        .incoming = std::clamp(
            SeepageFiniteOr(
                right.incomingHandleTime,
                kWaterSettingSplineHandleDefaultFraction),
            kWaterSettingSplineHandleMinimumFraction,
            kWaterSettingSplineHandleMaximumPair -
                kWaterSettingSplineHandleMinimumFraction),
    };
    const float movableOutgoing =
        result.outgoing - kWaterSettingSplineHandleMinimumFraction;
    const float movableIncoming =
        result.incoming - kWaterSettingSplineHandleMinimumFraction;
    const float movableTotal = movableOutgoing + movableIncoming;
    const float movableCapacity =
        kWaterSettingSplineHandleMaximumPair -
        2.0F * kWaterSettingSplineHandleMinimumFraction;
    if (movableTotal > movableCapacity && movableTotal > 0.0F) {
        const float scale = movableCapacity / movableTotal;
        result.outgoing =
            kWaterSettingSplineHandleMinimumFraction +
            movableOutgoing * scale;
        result.incoming =
            kWaterSettingSplineHandleMinimumFraction +
            movableIncoming * scale;
    }
    return result;
}

float EvaluateCubicBezierCoordinate(
    float point0,
    float point1,
    float point2,
    float point3,
    float amount) {
    const float inverse = 1.0F - amount;
    return inverse * inverse * inverse * point0 +
           3.0F * inverse * inverse * amount * point1 +
           3.0F * inverse * amount * amount * point2 +
           amount * amount * amount * point3;
}

float EvaluateWaterSettingSplineHandleSegment(
    const WaterSettingKey& left,
    const WaterSettingKey& right,
    float normalizedAmount) {
    const auto fractions =
        ResolveWaterSettingSplineHandleFractions(left, right);
    const float target = Clamp01(normalizedAmount);
    // With ordered x control points the Bezier x coordinate is monotone, so
    // bisection gives a stable timeline parameter even for nearly vertical
    // authored handles.
    float low = 0.0F;
    float high = 1.0F;
    for (int iteration = 0; iteration < 28; ++iteration) {
        const float middle = std::midpoint(low, high);
        const float x = EvaluateCubicBezierCoordinate(
            0.0F,
            fractions.outgoing,
            1.0F - fractions.incoming,
            1.0F,
            middle);
        if (x < target) {
            low = middle;
        } else {
            high = middle;
        }
    }
    const float parameter = std::midpoint(low, high);
    const float leftValue = SeepageFiniteOr(left.value, 0.0F);
    const float rightValue = SeepageFiniteOr(right.value, 0.0F);
    const float result = EvaluateCubicBezierCoordinate(
        leftValue,
        leftValue + SeepageFiniteOr(left.outgoingHandleValue, 0.0F),
        rightValue + SeepageFiniteOr(right.incomingHandleValue, 0.0F),
        rightValue,
        parameter);
    return std::isfinite(result)
               ? result
               : std::lerp(leftValue, rightValue, target);
}

template <typename Key, typename ValueAt>
float EvaluateCentripetalCatmullRomKeySegment(
    const std::vector<const Key*>& ordered,
    std::size_t leftIndex,
    float normalizedPosition,
    float amount,
    ValueAt valueAt) {
    struct SplinePoint {
        double x = 0.0;
        double y = 0.0;
    };
    const Key& left = *ordered[leftIndex];
    const Key& right = *ordered[leftIndex + 1U];
    // Values are normalized by the whole track's range so chord lengths
    // weight position and value comparably regardless of the value scale.
    double valueMinimum = std::numeric_limits<double>::max();
    double valueMaximum = std::numeric_limits<double>::lowest();
    for (const Key* key : ordered) {
        const double value = static_cast<double>(valueAt(*key));
        valueMinimum = std::min(valueMinimum, value);
        valueMaximum = std::max(valueMaximum, value);
    }
    const double valueSpan = valueMaximum - valueMinimum;
    if (!std::isfinite(valueSpan) ||
        valueSpan <= std::numeric_limits<double>::epsilon()) {
        return valueAt(left);
    }
    const auto pointForKey = [&](const Key& key) {
        return SplinePoint{
            .x = static_cast<double>(key.position),
            .y = (static_cast<double>(valueAt(key)) - valueMinimum) /
                 valueSpan,
        };
    };
    const SplinePoint point1 = pointForKey(left);
    const SplinePoint point2 = pointForKey(right);
    const SplinePoint point0 =
        leftIndex > 0U
            ? pointForKey(*ordered[leftIndex - 1U])
            : SplinePoint{
                  .x = 2.0 * point1.x - point2.x,
                  .y = 2.0 * point1.y - point2.y,
              };
    const SplinePoint point3 =
        leftIndex + 2U < ordered.size()
            ? pointForKey(*ordered[leftIndex + 2U])
            : SplinePoint{
                  .x = 2.0 * point2.x - point1.x,
                  .y = 2.0 * point2.y - point1.y,
              };
    const auto nextKnot = [](double knot,
                             const SplinePoint& firstPoint,
                             const SplinePoint& secondPoint) {
        // The Euclidean chord length is raised to alpha = 0.5.
        // A small floor keeps coincident control points evaluable.
        const double increment = std::max(
            1.0e-7,
            std::sqrt(std::hypot(
                secondPoint.x - firstPoint.x,
                secondPoint.y - firstPoint.y)));
        return knot + increment;
    };
    const double knot0 = 0.0;
    const double knot1 = nextKnot(knot0, point0, point1);
    const double knot2 = nextKnot(knot1, point1, point2);
    const double knot3 = nextKnot(knot2, point2, point3);
    const auto mixPoint = [](const SplinePoint& firstPoint,
                             const SplinePoint& secondPoint,
                             double firstWeight,
                             double secondWeight,
                             double denominator) {
        denominator = std::max(denominator, 1.0e-12);
        return SplinePoint{
            .x = (firstWeight * firstPoint.x +
                  secondWeight * secondPoint.x) /
                 denominator,
            .y = (firstWeight * firstPoint.y +
                  secondWeight * secondPoint.y) /
                 denominator,
        };
    };
    const auto evaluatePoint = [&](double knot) {
        const auto a1 = mixPoint(
            point0,
            point1,
            knot1 - knot,
            knot - knot0,
            knot1 - knot0);
        const auto a2 = mixPoint(
            point1,
            point2,
            knot2 - knot,
            knot - knot1,
            knot2 - knot1);
        const auto a3 = mixPoint(
            point2,
            point3,
            knot3 - knot,
            knot - knot2,
            knot3 - knot2);
        const auto b1 = mixPoint(
            a1,
            a2,
            knot2 - knot,
            knot - knot0,
            knot2 - knot0);
        const auto b2 = mixPoint(
            a2,
            a3,
            knot3 - knot,
            knot - knot1,
            knot3 - knot1);
        return mixPoint(
            b1,
            b2,
            knot2 - knot,
            knot - knot1,
            knot2 - knot1);
    };

    // The centripetal spline is a 2D curve in animation-position/value
    // space. Invert its x coordinate so evaluation remains keyed to the
    // animation timeline and dy/dx stays continuous at shared nodes.
    const double targetX = static_cast<double>(normalizedPosition);
    constexpr int kBracketSamples = 32;
    double bracketLow = knot1;
    double bracketHigh = knot2;
    bool bracketed = false;
    double previousKnot = knot1;
    double previousX = evaluatePoint(previousKnot).x;
    for (int sample = 1; sample <= kBracketSamples; ++sample) {
        const double candidateKnot = std::lerp(
            knot1,
            knot2,
            static_cast<double>(sample) /
                static_cast<double>(kBracketSamples));
        const double candidateX = evaluatePoint(candidateKnot).x;
        if ((previousX - targetX) * (candidateX - targetX) <= 0.0) {
            bracketLow = previousKnot;
            bracketHigh = candidateKnot;
            bracketed = true;
            break;
        }
        previousKnot = candidateKnot;
        previousX = candidateX;
    }
    double evaluatedKnot = std::lerp(knot1, knot2, static_cast<double>(amount));
    if (bracketed) {
        double lowX = evaluatePoint(bracketLow).x;
        for (int iteration = 0; iteration < 36; ++iteration) {
            const double middle = std::midpoint(bracketLow, bracketHigh);
            const double middleX = evaluatePoint(middle).x;
            if ((lowX - targetX) * (middleX - targetX) <= 0.0) {
                bracketHigh = middle;
            } else {
                bracketLow = middle;
                lowX = middleX;
            }
        }
        evaluatedKnot = std::midpoint(bracketLow, bracketHigh);
    }
    const double evaluated =
        valueMinimum + evaluatePoint(evaluatedKnot).y * valueSpan;
    const float result = static_cast<float>(evaluated);
    return std::isfinite(result)
               ? result
               : std::lerp(valueAt(left), valueAt(right), amount);
}

template <typename Key, typename ValueAt>
float EvaluateSmoothVelocityKeySegment(
    const std::vector<const Key*>& ordered,
    std::size_t leftIndex,
    float amount,
    ValueAt valueAt) {
    const std::size_t keyCount = ordered.size();
    const Key& left = *ordered[leftIndex];
    const Key& right = *ordered[leftIndex + 1U];
    const auto keyAt = [&](std::size_t index) -> const Key& {
        return *ordered[index];
    };
    const auto intervalDuration = [&](std::size_t index) {
        return static_cast<double>(keyAt(index + 1U).position) -
               static_cast<double>(keyAt(index).position);
    };
    const auto intervalVelocity = [&](std::size_t index) {
        const double duration = intervalDuration(index);
        return duration > static_cast<double>(kWaterKeySplineTolerance)
                   ? (static_cast<double>(valueAt(keyAt(index + 1U))) -
                      static_cast<double>(valueAt(keyAt(index)))) /
                         duration
                   : 0.0;
    };
    const auto continuesInSameDirection =
        [](double leftVelocity, double rightVelocity) {
            return leftVelocity != 0.0 && rightVelocity != 0.0 &&
                   std::isfinite(leftVelocity) &&
                   std::isfinite(rightVelocity) &&
                   std::signbit(leftVelocity) ==
                       std::signbit(rightVelocity);
        };
    const auto endpointVelocity =
        [&](double adjacentDuration,
            double nextDuration,
            double adjacentVelocity,
            double nextVelocity) {
            const double denominator = adjacentDuration + nextDuration;
            if (denominator <= 0.0 || !std::isfinite(denominator)) {
                return adjacentVelocity;
            }
            double velocity =
                ((2.0 * adjacentDuration + nextDuration) *
                     adjacentVelocity -
                 adjacentDuration * nextVelocity) /
                denominator;
            if (!continuesInSameDirection(velocity, adjacentVelocity)) {
                return 0.0;
            }
            if (!continuesInSameDirection(
                    adjacentVelocity,
                    nextVelocity) &&
                std::abs(velocity) > 3.0 * std::abs(adjacentVelocity)) {
                velocity = 3.0 * adjacentVelocity;
            }
            return velocity;
        };
    const auto tangentAt = [&](std::size_t index) {
        if (keyCount == 2U) {
            return intervalVelocity(0U);
        }
        if (index == 0U) {
            if (keyAt(0U).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity) {
                return 0.0;
            }
            return endpointVelocity(
                intervalDuration(0U),
                intervalDuration(1U),
                intervalVelocity(0U),
                intervalVelocity(1U));
        }
        if (index + 1U == keyCount) {
            if (keyAt(keyCount - 2U).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity) {
                return 0.0;
            }
            return endpointVelocity(
                intervalDuration(keyCount - 2U),
                intervalDuration(keyCount - 3U),
                intervalVelocity(keyCount - 2U),
                intervalVelocity(keyCount - 3U));
        }
        if (keyAt(index - 1U).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity ||
            keyAt(index).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity) {
            // Changing interpolation styles is an authored velocity break.
            return 0.0;
        }
        const double previousVelocity = intervalVelocity(index - 1U);
        const double nextVelocity = intervalVelocity(index);
        if (!continuesInSameDirection(previousVelocity, nextVelocity)) {
            // Local extrema and flat holds deliberately come to rest.
            return 0.0;
        }
        const double previousDuration = intervalDuration(index - 1U);
        const double nextDuration = intervalDuration(index);
        const double previousWeight =
            2.0 * nextDuration + previousDuration;
        const double nextWeight = nextDuration + 2.0 * previousDuration;
        return (previousWeight + nextWeight) /
               (previousWeight / previousVelocity +
                nextWeight / nextVelocity);
    };

    const double segmentDuration = intervalDuration(leftIndex);
    const double leftTangent = tangentAt(leftIndex);
    const double rightTangent = tangentAt(leftIndex + 1U);
    const double amountValue = static_cast<double>(Clamp01(amount));
    const double amountSquared = amountValue * amountValue;
    const double amountCubed = amountSquared * amountValue;
    const double leftValue = static_cast<double>(valueAt(left));
    const double rightValue = static_cast<double>(valueAt(right));
    const double evaluated =
        (2.0 * amountCubed - 3.0 * amountSquared + 1.0) * leftValue +
        (amountCubed - 2.0 * amountSquared + amountValue) *
            segmentDuration * leftTangent +
        (-2.0 * amountCubed + 3.0 * amountSquared) * rightValue +
        (amountCubed - amountSquared) * segmentDuration * rightTangent;
    const float result = static_cast<float>(evaluated);
    return std::isfinite(result)
               ? result
               : std::lerp(valueAt(left), valueAt(right), amount);
}

}  // namespace

float EvaluateWaterTimingRun(
    const WaterTimingRun& run,
    float normalizedPosition,
    float fallbackLevel) {
    if (run.keys.empty()) {
        return fallbackLevel;
    }
    normalizedPosition = Clamp01(SeepageFiniteOr(normalizedPosition, 0.0F));
    std::vector<const WaterTimingKey*> ordered;
    ordered.reserve(run.keys.size());
    for (const auto& key : run.keys) {
        ordered.push_back(&key);
    }
    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [](const WaterTimingKey* left, const WaterTimingKey* right) {
            return left->position < right->position;
        });
    const auto keyLevel = [](const WaterTimingKey& key) {
        return Clamp01(SeepageFiniteOr(key.level, 1.0F));
    };
    if (normalizedPosition <= ordered.front()->position) {
        return keyLevel(*ordered.front());
    }
    if (normalizedPosition >= ordered.back()->position) {
        return keyLevel(*ordered.back());
    }
    for (std::size_t index = 0U; index + 1U < ordered.size(); ++index) {
        const auto& left = *ordered[index];
        const auto& right = *ordered[index + 1U];
        // An exact interior key position belongs to the segment it starts, so
        // sampling at a key after a Hold segment yields the post-step level and
        // compiled tracks capture the step instead of the stale left limit.
        if (normalizedPosition >= right.position) {
            continue;
        }
        if (left.interpolation == WaterScenarioInterpolation::Hold) {
            return keyLevel(left);
        }
        const float span = std::max(1.0e-6F, right.position - left.position);
        float amount = Clamp01((normalizedPosition - left.position) / span);
        // Splines may overshoot the keyed levels; run levels stay 0..1.
        if (left.interpolation == WaterScenarioInterpolation::SmoothVelocity) {
            return Clamp01(EvaluateSmoothVelocityKeySegment(
                ordered,
                index,
                amount,
                keyLevel));
        }
        if (left.interpolation ==
            WaterScenarioInterpolation::CentripetalCatmullRom) {
            return Clamp01(EvaluateCentripetalCatmullRomKeySegment(
                ordered,
                index,
                normalizedPosition,
                amount,
                keyLevel));
        }
        if (left.interpolation == WaterScenarioInterpolation::Smooth ||
            left.interpolation == WaterScenarioInterpolation::SplineHandles) {
            amount = amount * amount * (3.0F - 2.0F * amount);
        }
        return std::lerp(keyLevel(left), keyLevel(right), amount);
    }
    return keyLevel(*ordered.back());
}

void AddOrUpdateWaterTimingKey(
    WaterTimingRun* run,
    WaterTimingKey key,
    float replacementTolerance) {
    if (run == nullptr) {
        return;
    }
    key.position = Clamp01(SeepageFiniteOr(key.position, 0.0F));
    key.level = Clamp01(SeepageFiniteOr(key.level, 1.0F));
    replacementTolerance = std::max(0.0F, replacementTolerance);
    const auto existing = std::find_if(
        run->keys.begin(),
        run->keys.end(),
        [&](const WaterTimingKey& candidate) {
            return std::abs(candidate.position - key.position) <= replacementTolerance;
        });
    if (existing != run->keys.end()) {
        const auto preservedId = existing->id;
        *existing = std::move(key);
        if (existing->id.empty()) {
            existing->id = preservedId;
        }
    } else {
        run->keys.push_back(std::move(key));
    }
    std::stable_sort(
        run->keys.begin(),
        run->keys.end(),
        [](const WaterTimingKey& left, const WaterTimingKey& right) {
            return left.position < right.position;
        });
}

void ApplyWaterTimingLevelToScenarioState(
    WaterTimingFeature feature,
    float level,
    WaterScenarioState* state) {
    if (state == nullptr) {
        return;
    }
    level = Clamp01(SeepageFiniteOr(level, 1.0F));
    switch (feature) {
        case WaterTimingFeature::Shoreline:
            state->shorelineLevel = level;
            return;
        case WaterTimingFeature::Seepage:
            state->seepageLevel = level;
            return;
        case WaterTimingFeature::Rain:
            state->rainLevel = level;
            return;
        case WaterTimingFeature::Flow:
            state->flowLevel = level;
            return;
        case WaterTimingFeature::MeshFlow:
            state->meshFlowLevel = level;
            return;
    }
}

float WaterTimingLevelFromScenarioState(
    WaterTimingFeature feature,
    const WaterScenarioState& state) {
    switch (feature) {
        case WaterTimingFeature::Shoreline:
            return Clamp01(SeepageFiniteOr(state.shorelineLevel, 1.0F));
        case WaterTimingFeature::Seepage:
            return Clamp01(SeepageFiniteOr(state.seepageLevel, 1.0F));
        case WaterTimingFeature::Rain:
            return Clamp01(SeepageFiniteOr(state.rainLevel, 0.0F));
        case WaterTimingFeature::Flow:
            return Clamp01(SeepageFiniteOr(state.flowLevel, 1.0F));
        case WaterTimingFeature::MeshFlow:
            return Clamp01(SeepageFiniteOr(state.meshFlowLevel, 1.0F));
    }
    return 1.0F;
}

std::vector<WaterScenarioKey> CompileWaterTimingScenarioKeys(
    const WaterScenarioState& baseState,
    std::span<const WaterTimingRun> runs) {
    constexpr float kPositionTolerance = 0.0001F;
    constexpr float kLevelTolerance = 1.0e-4F;
    constexpr std::size_t kMixedSegmentSubdivisions = 3U;

    std::vector<WaterTimingRun> activeRuns;
    activeRuns.reserve(runs.size());
    for (const auto& run : runs) {
        auto sanitized = SanitizeWaterTimingRun(run);
        if (!sanitized.keys.empty()) {
            activeRuns.push_back(std::move(sanitized));
        }
    }
    if (activeRuns.empty()) {
        return {};
    }

    std::vector<float> positions;
    for (const auto& run : activeRuns) {
        for (const auto& key : run.keys) {
            positions.push_back(key.position);
        }
    }
    std::sort(positions.begin(), positions.end());
    positions.erase(
        std::unique(
            positions.begin(),
            positions.end(),
            [&](float left, float right) {
                return std::abs(left - right) <= kPositionTolerance;
            }),
        positions.end());

    const auto runKeyAt = [&](const WaterTimingRun& run,
                              float position) -> const WaterTimingKey* {
        const auto found = std::find_if(
            run.keys.begin(),
            run.keys.end(),
            [&](const WaterTimingKey& key) {
                return std::abs(key.position - position) <= kPositionTolerance;
            });
        return found != run.keys.end() ? &*found : nullptr;
    };
    const auto stateAt = [&](float position) {
        WaterScenarioState state = baseState;
        for (const auto& run : activeRuns) {
            ApplyWaterTimingLevelToScenarioState(
                run.feature,
                EvaluateWaterTimingRun(
                    run,
                    position,
                    WaterTimingLevelFromScenarioState(run.feature, baseState)),
                &state);
        }
        return SanitizeWaterScenarioState(std::move(state));
    };

    struct CompiledSample {
        float position = 0.0F;
        WaterScenarioInterpolation interpolation = WaterScenarioInterpolation::Hold;
    };
    std::vector<CompiledSample> samples;
    samples.reserve(positions.size());
    for (std::size_t index = 0U; index < positions.size(); ++index) {
        const float position = positions[index];
        if (index + 1U == positions.size()) {
            samples.push_back({position, WaterScenarioInterpolation::Hold});
            break;
        }
        const float nextPosition = positions[index + 1U];

        // A run "changes" over this segment when its evaluated level differs
        // between the endpoints. Segments where nothing changes hold exactly.
        bool anyChange = false;
        bool exactSegment = true;
        std::optional<WaterScenarioInterpolation> sharedMode;
        for (const auto& run : activeRuns) {
            const float fallback =
                WaterTimingLevelFromScenarioState(run.feature, baseState);
            const float leftLevel = EvaluateWaterTimingRun(run, position, fallback);
            const float rightLevel = EvaluateWaterTimingRun(run, nextPosition, fallback);
            const auto* leftKey = runKeyAt(run, position);
            const bool leftIsSpline =
                leftKey != nullptr &&
                (leftKey->interpolation ==
                     WaterScenarioInterpolation::SmoothVelocity ||
                 leftKey->interpolation ==
                     WaterScenarioInterpolation::CentripetalCatmullRom);
            bool changes =
                std::abs(leftLevel - rightLevel) > kLevelTolerance;
            if (!changes && leftIsSpline) {
                // Neighbour-aware splines can bend inside a segment whose
                // endpoints are equal; probe the midpoint before compiling
                // the segment to a flat Hold.
                const float midLevel = EvaluateWaterTimingRun(
                    run,
                    std::midpoint(position, nextPosition),
                    fallback);
                changes = std::abs(midLevel - leftLevel) > kLevelTolerance;
            }
            if (!changes) {
                continue;
            }
            anyChange = true;
            // Exact only when this run has its own keys at both endpoints, so
            // the compiled segment coincides with one authored run segment.
            // Spline modes never pass through exactly: the run evaluator is
            // neighbour-aware while a compiled scenario segment eases in
            // isolation, so they take the subdivision path instead.
            const auto* rightKey = runKeyAt(run, nextPosition);
            if (leftKey == nullptr || rightKey == nullptr || leftIsSpline) {
                exactSegment = false;
                continue;
            }
            if (sharedMode.has_value() && *sharedMode != leftKey->interpolation) {
                exactSegment = false;
            } else {
                sharedMode = leftKey->interpolation;
            }
        }

        if (!anyChange) {
            samples.push_back({position, WaterScenarioInterpolation::Hold});
            continue;
        }
        if (exactSegment && sharedMode.has_value()) {
            samples.push_back({position, *sharedMode});
            continue;
        }
        // Mixed curvature cannot be represented by one snapshot segment;
        // approximate with linear subdivisions sampled exactly per feature.
        samples.push_back({position, WaterScenarioInterpolation::Linear});
        const float span = nextPosition - position;
        for (std::size_t subdivision = 1U;
             subdivision <= kMixedSegmentSubdivisions;
             ++subdivision) {
            const float amount = static_cast<float>(subdivision) /
                                 static_cast<float>(kMixedSegmentSubdivisions + 1U);
            samples.push_back({
                position + span * amount,
                WaterScenarioInterpolation::Linear,
            });
        }
    }

    std::vector<WaterScenarioKey> keys;
    keys.reserve(samples.size());
    for (std::size_t index = 0U; index < samples.size(); ++index) {
        WaterScenarioKey key;
        key.id = "timing_key_" + std::to_string(index + 1U);
        key.position = samples[index].position;
        key.state = stateAt(samples[index].position);
        key.interpolation = samples[index].interpolation;
        keys.push_back(std::move(key));
    }
    return keys;
}


// ---- Timings v2: per-feature, per-setting keyframing ----

namespace {

constexpr std::array<WaterKeyableSettingInfo, 1> kWaterGlobalLevelSettings{{
    {.id = "level",
     .label = "Level",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 1.0F},
}};

// Rain simulation, appearance, weather, and impact response all live in
// persistent GPU buffers and frame uniforms. These scalar controls can
// therefore be sampled per frame without reseeding particles or rebuilding
// the collision cache. Discrete presets/toggles, particle capacity, seed,
// and spawn/death bounds intentionally remain authored-only.
constexpr WaterKeyableSettingInfo kWaterRainSettings[]{
    {.id = "level",
     .label = "Rain Amount",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 1.0F},
    {.id = "density",
     .label = "Density",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.55F,
     .showUnauthoredInTimeline = false},
    {.id = "fall_speed",
     .label = "Fall Speed",
     .minimum = 0.2F,
     .maximum = 35.0F,
     .defaultValue = 8.0F,
     .showUnauthoredInTimeline = false},
    {.id = "drop_scale",
     .label = "Drop Scale",
     .minimum = 0.2F,
     .maximum = 4.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "visibility",
     .label = "Visibility",
     .minimum = 0.0F,
     .maximum = 3.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "glow",
     .label = "Glow",
     .minimum = 0.0F,
     .maximum = 4.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.colour_red",
     .label = "Colour Red",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.68F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.colour_green",
     .label = "Colour Green",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.82F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.colour_blue",
     .label = "Colour Blue",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.92F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.width",
     .label = "Width",
     .minimum = 0.0003F,
     .maximum = 0.020F,
     .defaultValue = 0.003F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.streak_length",
     .label = "Streak Length",
     .minimum = 0.005F,
     .maximum = 0.80F,
     .defaultValue = 0.16F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.softness",
     .label = "Softness",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.42F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.opacity",
     .label = "Opacity",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.58F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.emission",
     .label = "Emission",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 0.16F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.minimum_pixels",
     .label = "Minimum Pixels",
     .minimum = 0.0F,
     .maximum = 3.0F,
     .defaultValue = 0.65F,
     .showUnauthoredInTimeline = false},
    {.id = "visual.maximum_pixels",
     .label = "Maximum Pixels",
     .minimum = 0.5F,
     .maximum = 12.0F,
     .defaultValue = 4.0F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.wind_direction_x",
     .label = "Wind Direction X",
     .minimum = -1.0F,
     .maximum = 1.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.wind_direction_y",
     .label = "Wind Direction Y",
     .minimum = -1.0F,
     .maximum = 1.0F,
     .defaultValue = 0.0F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.wind_speed",
     .label = "Wind Speed",
     .minimum = 0.0F,
     .maximum = 8.0F,
     .defaultValue = 0.30F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.turbulence",
     .label = "Turbulence",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 0.45F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.gust_strength",
     .label = "Gust Strength",
     .minimum = 0.0F,
     .maximum = 1.5F,
     .defaultValue = 0.35F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.gust_scale",
     .label = "Gust Scale",
     .minimum = 0.2F,
     .maximum = 50.0F,
     .defaultValue = 8.0F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.gust_speed",
     .label = "Gust Speed",
     .minimum = 0.0F,
     .maximum = 12.0F,
     .defaultValue = 2.5F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.front_strength",
     .label = "Front Strength",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.40F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.front_scale",
     .label = "Front Scale",
     .minimum = 0.5F,
     .maximum = 100.0F,
     .defaultValue = 12.0F,
     .showUnauthoredInTimeline = false},
    {.id = "weather.front_speed",
     .label = "Front Speed",
     .minimum = 0.0F,
     .maximum = 12.0F,
     .defaultValue = 1.5F,
     .showUnauthoredInTimeline = false},
    {.id = "effects.rings_response",
     .label = "Rings Response",
     .minimum = 0.0F,
     .maximum = 3.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "effects.ring_thickness",
     .label = "Ring Thickness",
     .minimum = 0.25F,
     .maximum = 2.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "effects.wetness_response",
     .label = "Wetness Response",
     .minimum = 0.0F,
     .maximum = 3.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "effects.droplets_response",
     .label = "Droplets Response",
     .minimum = 0.0F,
     .maximum = 3.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "near_surface.approach_distance",
     .label = "Approach Distance",
     .minimum = 0.01F,
     .maximum = 1.0F,
     .defaultValue = 0.18F,
     .showUnauthoredInTimeline = false},
    {.id = "near_surface.minimum_speed",
     .label = "Minimum Speed",
     .minimum = 0.05F,
     .maximum = 1.0F,
     .defaultValue = 0.30F,
     .showUnauthoredInTimeline = false},
    {.id = "near_surface.squish",
     .label = "Squish",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.65F,
     .showUnauthoredInTimeline = false},
    {.id = "near_surface.normal_alignment",
     .label = "Normal Alignment",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.75F,
     .showUnauthoredInTimeline = false},
    {.id = "rings.band_min_z",
     .label = "Rings Band Min Z",
     .minimum = -2.0F,
     .maximum = 12.0F,
     .defaultValue = -2.0F,
     .showUnauthoredInTimeline = false},
    {.id = "rings.band_max_z",
     .label = "Rings Band Max Z",
     .minimum = -2.0F,
     .maximum = 12.0F,
     .defaultValue = 2.0F,
     .showUnauthoredInTimeline = false},
    {.id = "rings.band_fade",
     .label = "Rings Band Fade",
     .minimum = 0.01F,
     .maximum = 2.0F,
     .defaultValue = 0.30F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.edge_breakup",
     .label = "Wetness Edge Breakup",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.35F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.spread_speed",
     .label = "Wetness Spread Speed",
     .minimum = 0.25F,
     .maximum = 3.0F,
     .defaultValue = 1.60F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.centre_falloff",
     .label = "Wetness Centre Falloff",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 0.65F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.height_bias",
     .label = "Wetness Height Bias",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 0.75F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.persistence",
     .label = "Wetness Persistence",
     .minimum = 0.25F,
     .maximum = 3.0F,
     .defaultValue = 1.35F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.downhill_stretch",
     .label = "Wetness Downhill Stretch",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 1.0F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.band_min_z",
     .label = "Wetness Band Min Z",
     .minimum = -2.0F,
     .maximum = 12.0F,
     .defaultValue = 1.5F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.band_max_z",
     .label = "Wetness Band Max Z",
     .minimum = -2.0F,
     .maximum = 12.0F,
     .defaultValue = 2.4F,
     .showUnauthoredInTimeline = false},
    {.id = "wetness.band_fade",
     .label = "Wetness Band Fade",
     .minimum = 0.01F,
     .maximum = 2.0F,
     .defaultValue = 0.30F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.twinkle",
     .label = "Droplets Twinkle",
     .minimum = 0.0F,
     .maximum = 4.0F,
     .defaultValue = 1.80F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.propagation",
     .label = "Droplets Propagation",
     .minimum = 0.05F,
     .maximum = 3.0F,
     .defaultValue = 0.65F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.hop_spacing",
     .label = "Droplets Hop Spacing",
     .minimum = 0.01F,
     .maximum = 0.30F,
     .defaultValue = 0.070F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.stream_width",
     .label = "Droplets Stream Width",
     .minimum = 0.001F,
     .maximum = 0.08F,
     .defaultValue = 0.010F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.stream_spread",
     .label = "Droplets Stream Spread",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 0.65F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.band_min_z",
     .label = "Droplets Band Min Z",
     .minimum = -2.0F,
     .maximum = 12.0F,
     .defaultValue = 2.5F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.band_max_z",
     .label = "Droplets Band Max Z",
     .minimum = -2.0F,
     .maximum = 12.0F,
     .defaultValue = 12.0F,
     .showUnauthoredInTimeline = false},
    {.id = "droplets.band_fade",
     .label = "Droplets Band Fade",
     .minimum = 0.01F,
     .maximum = 2.0F,
     .defaultValue = 0.30F,
     .showUnauthoredInTimeline = false},
};

constexpr std::array<WaterKeyableSettingInfo, 5> kWaterMeshFlowSettings{{
    // Keep "level" as the stable identity for existing Timings-v2 tracks;
    // authored Mesh Flow now calls the same parameter Activity.
    {.id = "level",
     .label = "Activity",
     .minimum = 0.0F,
     .maximum = 1.0F,
     .defaultValue = 1.0F},
    {.id = "rain_gain",
     .label = "Rain Gain",
     .minimum = 0.0F,
     .maximum = 4.0F,
     .defaultValue = 0.0F},
    {.id = "moisture_persistence",
     .label = "Moisture Persistence",
     .minimum = 0.0F,
     .maximum = 8.0F,
     .defaultValue = 1.0F},
    {.id = "rain_rise_seconds",
     .label = "Rain Rise",
     .minimum = 0.0F,
     .maximum = 120.0F,
     .defaultValue = 0.0F},
    {.id = "rain_recession_seconds",
     .label = "Rain Recession",
     .minimum = 0.0F,
     .maximum = 300.0F,
     .defaultValue = 0.0F},
}};

constexpr std::array<WaterKeyableSettingInfo, 6> kWaterSeepageNodeSettings{{
    {.id = "strength",
     .label = "Node Strength",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 1.0F},
    {.id = "prominence",
     .label = "Prominence",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 1.0F},
    {.id = "source_width",
     .label = "Source Width",
     .minimum = 0.01F,
     .maximum = 2.0F,
     .defaultValue = 0.10F},
    {.id = "rain_delay_seconds",
     .label = "Rain Delay",
     .minimum = 0.0F,
     .maximum = 120.0F,
     .defaultValue = 0.0F},
    {.id = "rain_rise_seconds",
     .label = "Rain Rise",
     .minimum = 0.0F,
     .maximum = 120.0F,
     .defaultValue = 0.0F},
    {.id = "rain_recession_seconds",
     .label = "Rain Recession",
     .minimum = 0.0F,
     .maximum = 300.0F,
     .defaultValue = 0.0F},
}};

constexpr std::array<WaterKeyableSettingInfo, 2> kWaterFlowSourceSettings{{
    {.id = "strength",
     .label = "Maximum Flow Strength",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 1.0F},
    {.id = "rain_response",
     .label = "Rain Response",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 1.0F},
}};

// Manual paths have authored geometry, so their responsive animation surface
// is deliberately narrower than the lane/trail generation settings. These
// five values are evaluated into point style/uniform state each frame and do
// not resize buffers, reroute lanes, or regenerate trail samples.
constexpr std::array<WaterKeyableSettingInfo, 5> kWaterFlowPathSettings{{
    {.id = "strength",
     .label = "Maximum Flow Strength",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 1.0F},
    {.id = "rain_response",
     .label = "Rain Response",
     .minimum = 0.0F,
     .maximum = 2.0F,
     .defaultValue = 1.0F},
    {.id = "speed",
     .label = "Speed",
     .minimum = 0.001F,
     .maximum = 10.0F,
     .defaultValue = 0.45F},
    {.id = "trail_width",
     .label = "Trail Width",
     .minimum = 0.0005F,
     .maximum = 1.0F,
     .defaultValue = 0.006F},
    {.id = "trail_streak_length",
     .label = "Streak Length",
     .minimum = 0.001F,
     .maximum = 5.0F,
     .defaultValue = 0.045F},
}};

// Additional shoreline instances key a master level plus the visual response
// scalars of the active algorithm bank. Keyed values replace the authored
// bank scalar for the frame; shape and motion parameters stay authored.
constexpr std::array<WaterKeyableSettingInfo, 6>
    kWaterShorelineInstanceSettings{{
        {.id = "level",
         .label = "Level",
         .minimum = 0.0F,
         .maximum = 1.0F,
         .defaultValue = 1.0F},
        {.id = "intensity",
         .label = "Intensity",
         .minimum = 0.0F,
         .maximum = 2.0F,
         .defaultValue = 1.0F},
        {.id = "emission_add",
         .label = "Emission Add",
         .minimum = 0.0F,
         .maximum = 2.0F,
         .defaultValue = 0.0F},
        {.id = "opacity_add",
         .label = "Opacity Add",
         .minimum = -1.0F,
         .maximum = 1.0F,
         .defaultValue = 0.0F},
        {.id = "point_size_multiply",
         .label = "Point Size Multiply",
         .minimum = 0.0F,
         .maximum = 3.0F,
         .defaultValue = 1.0F},
        {.id = "colour_mix",
         .label = "Colour Mix",
         .minimum = 0.0F,
         .maximum = 1.0F,
         .defaultValue = 0.75F},
    }};

}  // namespace

std::span<const WaterKeyableSettingInfo> WaterKeyableSettings(
    WaterKeyedFeatureKind kind) {
    switch (kind) {
        case WaterKeyedFeatureKind::Rain:
            return kWaterRainSettings;
        case WaterKeyedFeatureKind::Shoreline:
            return kWaterGlobalLevelSettings;
        case WaterKeyedFeatureKind::MeshFlow:
            return kWaterMeshFlowSettings;
        case WaterKeyedFeatureKind::SeepageGlobal:
        case WaterKeyedFeatureKind::FlowGlobal:
            // Legacy kinds remain parseable so existing documents can be
            // round-tripped, but are no longer authorable or evaluated.
            return {};
        case WaterKeyedFeatureKind::SeepageNode:
            return kWaterSeepageNodeSettings;
        case WaterKeyedFeatureKind::FlowSource:
            return kWaterFlowSourceSettings;
        case WaterKeyedFeatureKind::FlowPath:
            return kWaterFlowPathSettings;
        case WaterKeyedFeatureKind::ShorelineInstance:
            return kWaterShorelineInstanceSettings;
    }
    return {};
}

const WaterKeyableSettingInfo* FindWaterKeyableSetting(
    WaterKeyedFeatureKind kind,
    std::string_view settingId) {
    for (const auto& info : WaterKeyableSettings(kind)) {
        if (settingId == info.id) {
            return &info;
        }
    }
    return nullptr;
}

std::optional<std::size_t> WaterKeyedSettingDisplayIndex(
    WaterKeyedFeatureKind kind,
    std::string_view settingId,
    std::span<const WaterKeyedSettingTrack> timelineSettings) {
    if (settingId.empty()) {
        return std::nullopt;
    }
    const auto registry = WaterKeyableSettings(kind);
    for (std::size_t index = 0U; index < registry.size(); ++index) {
        if (settingId == registry[index].id) {
            return index;
        }
    }

    std::vector<std::string_view> dynamicIds;
    dynamicIds.reserve(timelineSettings.size() + 1U);
    for (const auto& setting : timelineSettings) {
        if (!setting.settingId.empty() &&
            FindWaterKeyableSetting(kind, setting.settingId) == nullptr) {
            dynamicIds.push_back(setting.settingId);
        }
    }
    // The queried id may describe a track immediately before it is inserted.
    // Including it keeps that first keyed frame on the same deterministic
    // ordering rule as every stored dynamic track.
    dynamicIds.push_back(settingId);
    std::ranges::sort(dynamicIds);
    dynamicIds.erase(
        std::unique(dynamicIds.begin(), dynamicIds.end()),
        dynamicIds.end());
    const auto found = std::lower_bound(
        dynamicIds.begin(),
        dynamicIds.end(),
        settingId);
    if (found == dynamicIds.end() || *found != settingId) {
        return std::nullopt;
    }
    return registry.size() +
           static_cast<std::size_t>(found - dynamicIds.begin());
}

WaterFeatureClipSettingSignature WaterFeatureClipSettingSignatureForId(
    const WaterFeatureTimeline& timeline,
    std::uint32_t clipId) {
    WaterFeatureClipSettingSignature signature;
    for (const auto& setting : timeline.settings) {
        const bool ownsKey = std::any_of(
            setting.keys.begin(),
            setting.keys.end(),
            [&](const WaterSettingKey& key) {
                return key.clipId == clipId;
            });
        if (!ownsKey) {
            continue;
        }
        signature.settingIds.push_back(setting.settingId);
        const auto index = WaterKeyedSettingDisplayIndex(
            timeline.feature.kind,
            setting.settingId,
            timeline.settings);
        if (index.has_value() &&
            (!signature.minimumDisplayIndex.has_value() ||
             *index < *signature.minimumDisplayIndex)) {
            signature.minimumDisplayIndex = *index;
        }
    }
    std::ranges::sort(signature.settingIds);
    signature.settingIds.erase(
        std::unique(
            signature.settingIds.begin(),
            signature.settingIds.end()),
        signature.settingIds.end());
    return signature;
}

std::optional<std::size_t> WaterFeatureClipPrimarySettingDisplayIndex(
    const WaterFeatureTimeline& timeline,
    std::uint32_t clipId) {
    return WaterFeatureClipSettingSignatureForId(timeline, clipId)
        .minimumDisplayIndex;
}

std::pair<float, float> WaterFeatureClipDisplaySpan(
    float start,
    float end) {
    start = std::isfinite(start) ? start : 0.0F;
    end = std::isfinite(end) ? end : start;
    if (end < start) {
        std::swap(start, end);
    }
    if (end <= 1.0F + kWaterFeatureClipPositionTolerance) {
        // Unwrapped input (every loose-key span, most clips): the pre-W1
        // rule, shifting backwards at the end of the normalized domain.
        start = std::clamp(start, 0.0F, 1.0F);
        end = std::clamp(end, 0.0F, 1.0F);
        if (end - start < kWaterFeatureClipMinimumLength) {
            end = std::min(1.0F, start + kWaterFeatureClipMinimumLength);
            start = std::max(0.0F, end - kWaterFeatureClipMinimumLength);
        }
        return {start, end};
    }
    // A wrapped clip keeps its start phase and its length (at most one
    // cycle); only the end exceeds 1.
    const float length = end - start;
    start = WrapWaterClipPhase(start);
    end = start + std::clamp(length, kWaterFeatureClipMinimumLength, 1.0F);
    return {start, end};
}

WaterFeatureClipLaneLayout BuildWaterFeatureClipLaneLayout(
    const WaterFeatureTimeline& timeline) {
    struct Candidate {
        std::uint32_t clipId = 0U;
        float start = 0.0F;
        float end = 0.0F;
        WaterFeatureClipSettingSignature signature;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(timeline.clips.size() + 1U);
    for (const auto& clip : timeline.clips) {
        if (clip.id == 0U) {
            continue;
        }
        float start = std::isfinite(clip.start) ? clip.start : 0.0F;
        float end = std::isfinite(clip.end) ? clip.end : start;
        if (end < start) {
            std::swap(start, end);
        }
        candidates.push_back({
            .clipId = clip.id,
            .start = start,
            .end = end,
            .signature =
                WaterFeatureClipSettingSignatureForId(timeline, clip.id),
        });
    }
    if (const auto looseSpan = WaterFeatureLooseKeySpan(timeline);
        looseSpan.has_value()) {
        const auto displaySpan = WaterFeatureClipDisplaySpan(
            looseSpan->first,
            looseSpan->second);
        candidates.push_back({
            .clipId = 0U,
            .start = displaySpan.first,
            .end = displaySpan.second,
            .signature = WaterFeatureClipSettingSignatureForId(timeline, 0U),
        });
    }

    std::vector<WaterFeatureClipSettingSignature> signatures;
    signatures.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        if (std::find(
                signatures.begin(),
                signatures.end(),
                candidate.signature) == signatures.end()) {
            signatures.push_back(candidate.signature);
        }
    }
    const auto signatureLess =
        [](const WaterFeatureClipSettingSignature& left,
           const WaterFeatureClipSettingSignature& right) {
        const auto noIndex = std::numeric_limits<std::size_t>::max();
        const auto leftIndex = left.minimumDisplayIndex.value_or(noIndex);
        const auto rightIndex = right.minimumDisplayIndex.value_or(noIndex);
        if (leftIndex != rightIndex) {
            return leftIndex < rightIndex;
        }
        if (left.settingIds.size() != right.settingIds.size()) {
            return left.settingIds.size() < right.settingIds.size();
        }
        return std::lexicographical_compare(
            left.settingIds.begin(),
            left.settingIds.end(),
            right.settingIds.begin(),
            right.settingIds.end());
    };
    std::ranges::sort(signatures, signatureLess);

    WaterFeatureClipLaneLayout result;
    result.signatureBandCount = signatures.size();
    constexpr float kTouchTolerance = 1.0e-4F;
    std::size_t laneOffset = 0U;
    for (std::size_t band = 0U; band < signatures.size(); ++band) {
        std::vector<const Candidate*> bandCandidates;
        for (const auto& candidate : candidates) {
            if (candidate.signature == signatures[band]) {
                bandCandidates.push_back(&candidate);
            }
        }
        std::ranges::sort(
            bandCandidates,
            [](const Candidate* left, const Candidate* right) {
                if (left->start != right->start) {
                    return left->start < right->start;
                }
                if (left->end != right->end) {
                    return left->end < right->end;
                }
                return left->clipId < right->clipId;
            });

        // Each spill lane remembers every span placed on it so a wrapped
        // candidate (occupying [start,1] and [0,end-1]) is tested cyclically
        // against the lane's earlier spans rather than only its last end.
        std::vector<std::vector<std::pair<float, float>>> spillSpans;
        const auto spansOverlap =
            [&](std::pair<float, float> left, std::pair<float, float> right) {
                // Exact-touch spans overlap as well, keeping coincident edge
                // handles independently reachable in the UI.
                for (const float shift : {-1.0F, 0.0F, 1.0F}) {
                    if (right.first + shift <= left.second + kTouchTolerance &&
                        right.second + shift >= left.first - kTouchTolerance) {
                        return true;
                    }
                }
                return false;
            };
        for (const auto* candidate : bandCandidates) {
            const std::pair<float, float> span{candidate->start, candidate->end};
            std::size_t spill = 0U;
            for (; spill < spillSpans.size(); ++spill) {
                const bool free = std::none_of(
                    spillSpans[spill].begin(),
                    spillSpans[spill].end(),
                    [&](const std::pair<float, float>& placed) {
                        return spansOverlap(placed, span);
                    });
                if (free) {
                    break;
                }
            }
            if (spill == spillSpans.size()) {
                spillSpans.push_back({span});
            } else {
                spillSpans[spill].push_back(span);
            }
            result.assignments.push_back({
                .clipId = candidate->clipId,
                .signatureBandIndex = band,
                .spillLaneIndex = spill,
                .laneIndex = laneOffset + spill,
            });
        }
        laneOffset += spillSpans.size();
    }
    result.laneCount = laneOffset;
    std::ranges::sort(
        result.assignments,
        [](const WaterFeatureClipLaneAssignment& left,
           const WaterFeatureClipLaneAssignment& right) {
            return left.clipId < right.clipId;
        });
    return result;
}

std::string WaterFeatureClipConciseDisplayName(
    std::string_view clipName,
    std::string_view focusedObjectName) {
    const auto trim = [](std::string_view value) {
        while (!value.empty() &&
               std::isspace(
                   static_cast<unsigned char>(value.front())) != 0) {
            value.remove_prefix(1U);
        }
        while (!value.empty() &&
               std::isspace(
                   static_cast<unsigned char>(value.back())) != 0) {
            value.remove_suffix(1U);
        }
        return value;
    };
    clipName = trim(clipName);
    focusedObjectName = trim(focusedObjectName);
    if (clipName.empty()) {
        return "Clip";
    }

    struct PhaseMarker {
        std::string_view marker;
        std::string_view phase;
    };
    constexpr std::array kPhaseMarkers{
        PhaseMarker{" - Start - ", "Start"},
        PhaseMarker{" - Finish - ", "Finish"},
        PhaseMarker{" — Start — ", "Start"},
        PhaseMarker{" — Finish — ", "Finish"},
        PhaseMarker{" – Start – ", "Start"},
        PhaseMarker{" – Finish – ", "Finish"},
    };
    for (const auto& phase : kPhaseMarkers) {
        if (const auto found = clipName.find(phase.marker);
            found != std::string_view::npos) {
            const auto tail = trim(
                clipName.substr(found + phase.marker.size()));
            std::string concise{phase.phase};
            if (!tail.empty()) {
                concise += " - ";
                concise += tail;
            }
            return concise;
        }
    }

    if (focusedObjectName.empty() ||
        !clipName.starts_with(focusedObjectName) ||
        clipName.size() == focusedObjectName.size()) {
        return std::string{clipName};
    }
    auto remainder = clipName.substr(focusedObjectName.size());
    const auto startsSeparator = [](std::string_view value) {
        if (value.empty()) {
            return false;
        }
        const auto byte = static_cast<unsigned char>(value.front());
        return std::isspace(byte) != 0 || value.front() == '-' ||
               value.front() == '_' || value.front() == ':' ||
               value.front() == '/' || value.starts_with("—") ||
               value.starts_with("–");
    };
    if (!startsSeparator(remainder)) {
        return std::string{clipName};
    }
    while (!remainder.empty()) {
        if (remainder.starts_with("—") || remainder.starts_with("–")) {
            remainder.remove_prefix(3U);
            continue;
        }
        const auto byte = static_cast<unsigned char>(remainder.front());
        if (std::isspace(byte) != 0 || remainder.front() == '-' ||
            remainder.front() == '_' || remainder.front() == ':' ||
            remainder.front() == '/') {
            remainder.remove_prefix(1U);
            continue;
        }
        break;
    }
    remainder = trim(remainder);
    return remainder.empty() ? std::string{clipName}
                             : std::string{remainder};
}

std::optional<float> ResolveWaterRainSettingValue(
    const RainRuntimeSettings& settings,
    const WaterRainVisualSettings& visual,
    std::string_view settingId) {
    if (settingId == "level") {
        return settings.rainLevel;
    }
    if (settingId == "density") {
        return settings.density;
    }
    if (settingId == "fall_speed") {
        return settings.fallSpeedMetersPerSecond;
    }
    if (settingId == "drop_scale") {
        return settings.dropletSizeScale;
    }
    if (settingId == "visibility") {
        return settings.opacityScale;
    }
    if (settingId == "glow") {
        return settings.emissionScale;
    }
    if (settingId == "visual.colour_red") {
        return visual.colour[0];
    }
    if (settingId == "visual.colour_green") {
        return visual.colour[1];
    }
    if (settingId == "visual.colour_blue") {
        return visual.colour[2];
    }
    if (settingId == "visual.width") {
        return visual.widthMeters;
    }
    if (settingId == "visual.streak_length") {
        return visual.streakLengthMeters;
    }
    if (settingId == "visual.softness") {
        return visual.softness;
    }
    if (settingId == "visual.opacity") {
        return visual.opacity;
    }
    if (settingId == "visual.emission") {
        return visual.emission;
    }
    if (settingId == "visual.minimum_pixels") {
        return visual.minimumScreenPixels;
    }
    if (settingId == "visual.maximum_pixels") {
        return visual.maximumScreenPixels;
    }
    if (settingId == "weather.wind_direction_x") {
        return settings.windDirectionX;
    }
    if (settingId == "weather.wind_direction_y") {
        return settings.windDirectionY;
    }
    if (settingId == "weather.wind_speed") {
        return settings.windSpeedMetersPerSecond;
    }
    if (settingId == "weather.turbulence") {
        return settings.turbulence;
    }
    if (settingId == "weather.gust_strength") {
        return settings.gustStrength;
    }
    if (settingId == "weather.gust_scale") {
        return settings.gustScaleMeters;
    }
    if (settingId == "weather.gust_speed") {
        return settings.gustSpeedMetersPerSecond;
    }
    if (settingId == "weather.front_strength") {
        return settings.weatherFrontStrength;
    }
    if (settingId == "weather.front_scale") {
        return settings.weatherFrontScaleMeters;
    }
    if (settingId == "weather.front_speed") {
        return settings.weatherFrontSpeedMetersPerSecond;
    }
    if (settingId == "effects.rings_response") {
        return settings.sandEffectScale;
    }
    if (settingId == "effects.ring_thickness") {
        return settings.ringImpact.thicknessScale;
    }
    if (settingId == "effects.wetness_response") {
        return settings.rockEffectScale;
    }
    if (settingId == "effects.droplets_response") {
        return settings.vegetationEffectScale;
    }
    if (settingId == "near_surface.approach_distance") {
        return settings.nearSurface.approachDistanceMeters;
    }
    if (settingId == "near_surface.minimum_speed") {
        return settings.nearSurface.minimumSpeedFactor;
    }
    if (settingId == "near_surface.squish") {
        return settings.nearSurface.squish;
    }
    if (settingId == "near_surface.normal_alignment") {
        return settings.nearSurface.normalAlignment;
    }
    if (settingId == "rings.band_min_z") {
        return settings.sandImpactBand.minZ;
    }
    if (settingId == "rings.band_max_z") {
        return settings.sandImpactBand.maxZ;
    }
    if (settingId == "rings.band_fade") {
        return settings.sandImpactBand.fadeMeters;
    }
    if (settingId == "wetness.edge_breakup") {
        return settings.rockImpact.edgeBreakup;
    }
    if (settingId == "wetness.spread_speed") {
        return settings.rockImpact.spreadSpeed;
    }
    if (settingId == "wetness.centre_falloff") {
        return settings.rockImpact.centreFalloff;
    }
    if (settingId == "wetness.height_bias") {
        return settings.rockImpact.heightBias;
    }
    if (settingId == "wetness.persistence") {
        return settings.rockImpact.persistence;
    }
    if (settingId == "wetness.downhill_stretch") {
        return settings.rockImpact.downhillStretch;
    }
    if (settingId == "wetness.band_min_z") {
        return settings.rockImpactBand.minZ;
    }
    if (settingId == "wetness.band_max_z") {
        return settings.rockImpactBand.maxZ;
    }
    if (settingId == "wetness.band_fade") {
        return settings.rockImpactBand.fadeMeters;
    }
    if (settingId == "droplets.twinkle") {
        return settings.vegetationImpact.twinkle;
    }
    if (settingId == "droplets.propagation") {
        return settings.vegetationImpact.propagationMetersPerSecond;
    }
    if (settingId == "droplets.hop_spacing") {
        return settings.vegetationImpact.hopSpacingMeters;
    }
    if (settingId == "droplets.stream_width") {
        return settings.vegetationImpact.streamWidthMeters;
    }
    if (settingId == "droplets.stream_spread") {
        return settings.vegetationImpact.streamSpread;
    }
    if (settingId == "droplets.band_min_z") {
        return settings.vegetationImpactBand.minZ;
    }
    if (settingId == "droplets.band_max_z") {
        return settings.vegetationImpactBand.maxZ;
    }
    if (settingId == "droplets.band_fade") {
        return settings.vegetationImpactBand.fadeMeters;
    }
    return std::nullopt;
}

void ApplyWaterFeatureTimingOverlayToRainSettings(
    const WaterFeatureTimingOverlay& overlay,
    WaterRainSettings* settings,
    WaterRainVisualSettings* visual) {
    const WaterKeyedFeatureId feature{
        .kind = WaterKeyedFeatureKind::Rain,
    };
    if (!overlay.Allows(feature)) {
        if (settings != nullptr) {
            settings->enabled = false;
            settings->rainLevel = 0.0F;
        }
        return;
    }
    const auto apply = [&](std::string_view settingId, float* target) {
        if (target == nullptr) {
            return false;
        }
        const auto* value = overlay.Find(feature, settingId);
        const auto* info = FindWaterKeyableSetting(feature.kind, settingId);
        if (value == nullptr || info == nullptr) {
            return false;
        }
        *target = std::clamp(
            SeepageFiniteOr(*value, info->defaultValue),
            info->minimum,
            info->maximum);
        return true;
    };

    if (settings != nullptr) {
        if (apply("level", &settings->rainLevel)) {
            // A keyed zero is still an active Rain runtime: it stops births
            // while allowing impacts emitted by earlier frames to finish.
            settings->enabled = true;
        }
        apply("density", &settings->density);
        apply("fall_speed", &settings->fallSpeedMetersPerSecond);
        apply("drop_scale", &settings->dropletSizeScale);
        apply("visibility", &settings->opacityScale);
        apply("glow", &settings->emissionScale);
        apply("weather.wind_direction_x", &settings->windDirectionX);
        apply("weather.wind_direction_y", &settings->windDirectionY);
        apply("weather.wind_speed", &settings->windSpeedMetersPerSecond);
        apply("weather.turbulence", &settings->turbulence);
        apply("weather.gust_strength", &settings->gustStrength);
        apply("weather.gust_scale", &settings->gustScaleMeters);
        apply("weather.gust_speed", &settings->gustSpeedMetersPerSecond);
        apply("weather.front_strength", &settings->weatherFrontStrength);
        apply("weather.front_scale", &settings->weatherFrontScaleMeters);
        apply("weather.front_speed", &settings->weatherFrontSpeedMetersPerSecond);
        apply("effects.rings_response", &settings->sandEffectScale);
        apply("effects.ring_thickness", &settings->ringImpact.thicknessScale);
        apply("effects.wetness_response", &settings->rockEffectScale);
        apply("effects.droplets_response", &settings->vegetationEffectScale);
        apply(
            "near_surface.approach_distance",
            &settings->nearSurface.approachDistanceMeters);
        apply(
            "near_surface.minimum_speed",
            &settings->nearSurface.minimumSpeedFactor);
        apply("near_surface.squish", &settings->nearSurface.squish);
        apply(
            "near_surface.normal_alignment",
            &settings->nearSurface.normalAlignment);
        apply("rings.band_min_z", &settings->sandImpactBand.minZ);
        apply("rings.band_max_z", &settings->sandImpactBand.maxZ);
        apply("rings.band_fade", &settings->sandImpactBand.fadeMeters);
        apply("wetness.edge_breakup", &settings->rockImpact.edgeBreakup);
        apply("wetness.spread_speed", &settings->rockImpact.spreadSpeed);
        apply("wetness.centre_falloff", &settings->rockImpact.centreFalloff);
        apply("wetness.height_bias", &settings->rockImpact.heightBias);
        apply("wetness.persistence", &settings->rockImpact.persistence);
        apply("wetness.downhill_stretch", &settings->rockImpact.downhillStretch);
        apply("wetness.band_min_z", &settings->rockImpactBand.minZ);
        apply("wetness.band_max_z", &settings->rockImpactBand.maxZ);
        apply("wetness.band_fade", &settings->rockImpactBand.fadeMeters);
        apply("droplets.twinkle", &settings->vegetationImpact.twinkle);
        apply(
            "droplets.propagation",
            &settings->vegetationImpact.propagationMetersPerSecond);
        apply(
            "droplets.hop_spacing",
            &settings->vegetationImpact.hopSpacingMeters);
        apply(
            "droplets.stream_width",
            &settings->vegetationImpact.streamWidthMeters);
        apply(
            "droplets.stream_spread",
            &settings->vegetationImpact.streamSpread);
        apply("droplets.band_min_z", &settings->vegetationImpactBand.minZ);
        apply("droplets.band_max_z", &settings->vegetationImpactBand.maxZ);
        apply("droplets.band_fade", &settings->vegetationImpactBand.fadeMeters);
        settings->sandImpactBand =
            SanitizeRainImpactHeightBand(settings->sandImpactBand);
        settings->rockImpactBand =
            SanitizeRainImpactHeightBand(settings->rockImpactBand);
        settings->vegetationImpactBand =
            SanitizeRainImpactHeightBand(settings->vegetationImpactBand);
    }

    if (visual != nullptr) {
        apply("visual.colour_red", &visual->colour[0]);
        apply("visual.colour_green", &visual->colour[1]);
        apply("visual.colour_blue", &visual->colour[2]);
        apply("visual.width", &visual->widthMeters);
        apply("visual.streak_length", &visual->streakLengthMeters);
        apply("visual.softness", &visual->softness);
        apply("visual.opacity", &visual->opacity);
        apply("visual.emission", &visual->emission);
        apply("visual.minimum_pixels", &visual->minimumScreenPixels);
        apply("visual.maximum_pixels", &visual->maximumScreenPixels);
        visual->maximumScreenPixels = std::max(
            visual->minimumScreenPixels,
            visual->maximumScreenPixels);
    }
}

std::string_view WaterKeyedFeatureKindLabel(WaterKeyedFeatureKind kind) {
    switch (kind) {
        case WaterKeyedFeatureKind::Rain:
            return "Rain";
        case WaterKeyedFeatureKind::MeshFlow:
            return "Mesh Flow";
        case WaterKeyedFeatureKind::Shoreline:
            return "Shoreline";
        case WaterKeyedFeatureKind::SeepageGlobal:
            return "Seepage (Global)";
        case WaterKeyedFeatureKind::FlowGlobal:
            return "Flow (Global)";
        case WaterKeyedFeatureKind::SeepageNode:
            return "Seepage Node";
        case WaterKeyedFeatureKind::FlowSource:
            return "Flow Source";
        case WaterKeyedFeatureKind::FlowPath:
            return "Flow Path";
        case WaterKeyedFeatureKind::ShorelineInstance:
            return "Shoreline Instance";
    }
    return "Water Feature";
}

std::string_view WaterKeyedFeatureKindName(WaterKeyedFeatureKind kind) {
    switch (kind) {
        case WaterKeyedFeatureKind::Rain:
            return "rain";
        case WaterKeyedFeatureKind::MeshFlow:
            return "mesh_flow";
        case WaterKeyedFeatureKind::Shoreline:
            return "shoreline";
        case WaterKeyedFeatureKind::SeepageGlobal:
            return "seepage_global";
        case WaterKeyedFeatureKind::FlowGlobal:
            return "flow_global";
        case WaterKeyedFeatureKind::SeepageNode:
            return "seepage_node";
        case WaterKeyedFeatureKind::FlowSource:
            return "flow_source";
        case WaterKeyedFeatureKind::FlowPath:
            return "flow_path";
        case WaterKeyedFeatureKind::ShorelineInstance:
            return "shoreline_instance";
    }
    return "rain";
}

std::optional<WaterKeyedFeatureKind> ParseWaterKeyedFeatureKindName(
    std::string_view name) {
    static constexpr std::array<WaterKeyedFeatureKind, 9> kKinds{
        WaterKeyedFeatureKind::Rain,
        WaterKeyedFeatureKind::MeshFlow,
        WaterKeyedFeatureKind::Shoreline,
        WaterKeyedFeatureKind::SeepageGlobal,
        WaterKeyedFeatureKind::FlowGlobal,
        WaterKeyedFeatureKind::SeepageNode,
        WaterKeyedFeatureKind::FlowSource,
        WaterKeyedFeatureKind::FlowPath,
        WaterKeyedFeatureKind::ShorelineInstance,
    };
    for (const auto kind : kKinds) {
        if (name == WaterKeyedFeatureKindName(kind)) {
            return kind;
        }
    }
    return std::nullopt;
}

bool WaterKeyedFeatureKindIsGlobal(WaterKeyedFeatureKind kind) {
    switch (kind) {
        case WaterKeyedFeatureKind::SeepageNode:
        case WaterKeyedFeatureKind::FlowSource:
        case WaterKeyedFeatureKind::FlowPath:
        case WaterKeyedFeatureKind::ShorelineInstance:
            return false;
        case WaterKeyedFeatureKind::Rain:
        case WaterKeyedFeatureKind::MeshFlow:
        case WaterKeyedFeatureKind::Shoreline:
        case WaterKeyedFeatureKind::SeepageGlobal:
        case WaterKeyedFeatureKind::FlowGlobal:
            return true;
    }
    return true;
}

WaterKeyedSettingTrack SanitizeWaterKeyedSettingTrack(
    WaterKeyedSettingTrack track) {
    if (track.defaultInterpolation ==
        WaterScenarioInterpolation::TrackDefault) {
        track.defaultInterpolation =
            WaterScenarioInterpolation::SmoothVelocity;
    }
    for (auto& key : track.keys) {
        key.position = Clamp01(SeepageFiniteOr(key.position, 0.0F));
        key.value = SeepageFiniteOr(key.value, 0.0F);
        key.incomingHandleTime = std::clamp(
            SeepageFiniteOr(
                key.incomingHandleTime,
                kWaterSettingSplineHandleDefaultFraction),
            kWaterSettingSplineHandleMinimumFraction,
            kWaterSettingSplineHandleMaximumPair -
                kWaterSettingSplineHandleMinimumFraction);
        key.incomingHandleValue =
            SeepageFiniteOr(key.incomingHandleValue, 0.0F);
        key.outgoingHandleTime = std::clamp(
            SeepageFiniteOr(
                key.outgoingHandleTime,
                kWaterSettingSplineHandleDefaultFraction),
            kWaterSettingSplineHandleMinimumFraction,
            kWaterSettingSplineHandleMaximumPair -
                kWaterSettingSplineHandleMinimumFraction);
        key.outgoingHandleValue =
            SeepageFiniteOr(key.outgoingHandleValue, 0.0F);
    }
    std::stable_sort(
        track.keys.begin(),
        track.keys.end(),
        [](const WaterSettingKey& left, const WaterSettingKey& right) {
            return left.position < right.position;
        });
    for (std::size_t index = 0U; index + 1U < track.keys.size(); ++index) {
        const auto fractions = ResolveWaterSettingSplineHandleFractions(
            track.keys[index],
            track.keys[index + 1U]);
        track.keys[index].outgoingHandleTime = fractions.outgoing;
        track.keys[index + 1U].incomingHandleTime = fractions.incoming;
    }
    return track;
}

bool WaterKeyedSettingTrackProfileEqual(
    const WaterKeyedSettingTrack& left,
    const WaterKeyedSettingTrack& right) {
    if (left.settingId != right.settingId ||
        left.active != right.active ||
        left.label != right.label ||
        left.profileGroup != right.profileGroup ||
        left.profileName != right.profileName ||
        left.defaultInterpolation != right.defaultInterpolation ||
        left.keys.size() != right.keys.size()) {
        return false;
    }
    for (std::size_t index = 0U; index < left.keys.size(); ++index) {
        const auto& a = left.keys[index];
        const auto& b = right.keys[index];
        if (a.position != b.position ||
            a.value != b.value ||
            a.interpolation != b.interpolation ||
            a.incomingHandleTime != b.incomingHandleTime ||
            a.incomingHandleValue != b.incomingHandleValue ||
            a.outgoingHandleTime != b.outgoingHandleTime ||
            a.outgoingHandleValue != b.outgoingHandleValue) {
            return false;
        }
    }
    return true;
}

bool WaterObjectProfileNameIsLegacyEditedShadow(
    std::string_view profileName,
    bool objectOverride) {
    constexpr std::string_view kEditedSuffix = "_edited";
    constexpr std::string_view kLegacyEditedSuffix = "_Edited";
    const auto name = TrimSeepageName(profileName);
    return !objectOverride &&
           (name.ends_with(kEditedSuffix) ||
            name.ends_with(kLegacyEditedSuffix));
}

WaterObjectProfileEditDescriptor DescribeWaterObjectProfileEdit(
    std::string_view assignedProfileName,
    std::uint32_t selectedOwnerObjectId,
    const std::optional<WaterObjectProfileIdentity>& assignedObjectCopy) {
    constexpr std::string_view kPresetSuffix = "_preset";
    constexpr std::string_view kEditedSuffix = "_edited";
    constexpr std::string_view kLegacyEditedSuffix = "_Edited";
    const auto normalize = [](std::string_view name) {
        const auto trimmed = TrimSeepageName(name);
        return trimmed.empty() ? std::string{"Default"}
                               : std::string{trimmed};
    };
    const auto savedName = [&](std::string_view name) {
        auto saved = normalize(name);
        if ((saved.ends_with(kEditedSuffix) ||
             saved.ends_with(kLegacyEditedSuffix)) &&
            saved.size() > kEditedSuffix.size()) {
            saved.erase(saved.size() - kEditedSuffix.size());
        }
        if (saved.ends_with(kPresetSuffix) &&
            saved.size() > kPresetSuffix.size()) {
            saved.erase(saved.size() - kPresetSuffix.size());
        }
        return normalize(saved);
    };

    WaterObjectProfileEditDescriptor descriptor;
    descriptor.assignedProfileName = normalize(assignedProfileName);
    const bool copyMatches =
        assignedObjectCopy.has_value() &&
        normalize(assignedObjectCopy->name) ==
            descriptor.assignedProfileName;
    if (copyMatches) {
        descriptor.assignedObjectCopy = true;
        descriptor.exactBaseProfileName =
            normalize(assignedObjectCopy->baseProfileName);
        descriptor.ownedObjectCopy =
            assignedObjectCopy->ownerObjectId == selectedOwnerObjectId;
        if (descriptor.ownedObjectCopy) {
            descriptor.removableWorkingProfileName =
                descriptor.assignedProfileName;
        }
    } else if (descriptor.assignedProfileName.size() >
                   kEditedSuffix.size() &&
               WaterObjectProfileNameIsLegacyEditedShadow(
                   descriptor.assignedProfileName,
                   false)) {
        descriptor.legacyEditedShadow = true;
        descriptor.exactBaseProfileName =
            descriptor.assignedProfileName.substr(
                0U,
                descriptor.assignedProfileName.size() -
                    kEditedSuffix.size());
        descriptor.removableWorkingProfileName =
            descriptor.assignedProfileName;
    } else {
        descriptor.exactBaseProfileName =
            descriptor.assignedProfileName;
    }
    descriptor.suggestedSaveProfileName =
        savedName(descriptor.exactBaseProfileName);
    return descriptor;
}

WaterObjectProfilePromotionPlan PlanWaterObjectProfilePromotion(
    const WaterObjectProfileEditDescriptor& descriptor,
    WaterObjectProfilePromotionOperation operation,
    WaterObjectProfileBaseKind baseKind,
    std::string_view requestedProfileName,
    std::span<const std::string> reservedProfileNames) {
    WaterObjectProfilePromotionPlan plan;
    plan.operation = operation;
    plan.workingProfileName =
        std::string{TrimSeepageName(
            descriptor.removableWorkingProfileName)};
    if (!descriptor.ownedObjectCopy ||
        plan.workingProfileName.empty()) {
        plan.failure = WaterObjectProfilePromotionFailure::
            NotOwnedWorkingCopy;
        return plan;
    }

    const auto exactBase = [&]() {
        const auto trimmed =
            TrimSeepageName(descriptor.exactBaseProfileName);
        return trimmed.empty() ? std::string{"Default"}
                               : std::string{trimmed};
    }();
    if (operation == WaterObjectProfilePromotionOperation::Save) {
        switch (baseKind) {
            case WaterObjectProfileBaseKind::Default:
            case WaterObjectProfileBaseKind::Shared:
                plan.targetProfileName = exactBase;
                plan.overwriteExisting = true;
                plan.eraseWorkingCopy = true;
                return plan;
            case WaterObjectProfileBaseKind::Protected:
                plan.failure = WaterObjectProfilePromotionFailure::
                    ProtectedBaseRequiresSaveAs;
                return plan;
            case WaterObjectProfileBaseKind::Missing:
                plan.failure = WaterObjectProfilePromotionFailure::
                    MissingBaseRequiresSaveAs;
                return plan;
            case WaterObjectProfileBaseKind::ObjectCopy:
                plan.failure = WaterObjectProfilePromotionFailure::
                    ObjectCopyBaseRequiresSaveAs;
                return plan;
        }
    }

    if (operation == WaterObjectProfilePromotionOperation::Discard) {
        switch (baseKind) {
            case WaterObjectProfileBaseKind::Default:
            case WaterObjectProfileBaseKind::Shared:
            case WaterObjectProfileBaseKind::Protected:
                plan.targetProfileName = exactBase;
                plan.eraseWorkingCopy = true;
                return plan;
            case WaterObjectProfileBaseKind::Missing:
                plan.failure = WaterObjectProfilePromotionFailure::
                    MissingDiscardBase;
                return plan;
            case WaterObjectProfileBaseKind::ObjectCopy:
                plan.failure = WaterObjectProfilePromotionFailure::
                    ObjectCopyDiscardBase;
                return plan;
        }
    }

    const auto reusableName = [](std::string_view name) {
        auto reusable = std::string{TrimSeepageName(name)};
        constexpr std::array suffixes{
            std::string_view{"_edited"},
            std::string_view{"_Edited"},
            std::string_view{"_preset"},
        };
        bool stripped = true;
        while (stripped && !reusable.empty()) {
            stripped = false;
            for (const auto suffix : suffixes) {
                if (reusable.size() >= suffix.size() &&
                    reusable.ends_with(suffix)) {
                    reusable.erase(reusable.size() - suffix.size());
                    stripped = true;
                    break;
                }
            }
        }
        return std::string{TrimSeepageName(reusable)};
    };
    auto preferred = reusableName(requestedProfileName);
    if (preferred.empty()) {
        preferred = reusableName(
            descriptor.suggestedSaveProfileName);
    }
    if (preferred.empty()) {
        preferred = "Profile";
    }
    plan.targetProfileName = AllocateUniqueWaterObjectProfileName(
        preferred,
        reservedProfileNames);
    plan.createShared = true;
    plan.eraseWorkingCopy = true;
    return plan;
}

std::string AllocateUniqueWaterObjectProfileName(
    std::string_view preferredProfileName,
    std::span<const std::string> reservedProfileNames) {
    auto preferred = std::string{TrimSeepageName(preferredProfileName)};
    if (preferred.empty()) {
        preferred = "Profile";
    }
    const auto inUse = [&](std::string_view name) {
        const auto normalized = TrimSeepageName(name);
        if (normalized.ends_with("_preset")) {
            return true;
        }
        return std::any_of(
            reservedProfileNames.begin(),
            reservedProfileNames.end(),
            [&](const std::string& reserved) {
                return TrimSeepageName(reserved) == normalized;
            });
    };
    if (!inUse(preferred)) {
        return preferred;
    }

    // N reservations cannot occupy all N+1 distinct numeric candidates.
    // This bound is deliberately derived from the input rather than capped.
    const auto candidateCount = reservedProfileNames.size() + 1U;
    for (std::size_t offset = 0U; offset < candidateCount; ++offset) {
        const auto candidate =
            preferred + " " + std::to_string(offset + 2U);
        if (!inUse(candidate)) {
            return candidate;
        }
    }

    // The pigeonhole bound above makes this unreachable for a finite span.
    std::terminate();
}

bool RunWaterObjectProfilePromotionTransaction(
    const WaterObjectProfilePromotionPlan& plan,
    const WaterObjectProfilePromotionTransactionCallbacks& callbacks) {
    if (!plan.allowed() || plan.workingProfileName.empty() ||
        plan.targetProfileName.empty() || !plan.eraseWorkingCopy) {
        return false;
    }
    const bool writesTarget =
        plan.operation != WaterObjectProfilePromotionOperation::Discard;
    if ((writesTarget && !callbacks.writeTarget) ||
        !callbacks.rewriteReferences || !callbacks.eraseWorkingCopy) {
        return false;
    }
    if (writesTarget) {
        if (!callbacks.writeTarget(plan.targetProfileName)) {
            return false;
        }
    }
    if (!callbacks.rewriteReferences(
            plan.workingProfileName,
            plan.targetProfileName)) {
        return false;
    }
    if (!callbacks.eraseWorkingCopy(plan.workingProfileName)) {
        return false;
    }
    return true;
}

std::size_t ReplaceWaterSeepageNodeProfileReferences(
    std::span<WaterSeepageNode> nodes,
    WaterSeepageProfileHalf half,
    std::string_view previousProfileName,
    std::string_view nextProfileName) {
    const auto previous = TrimSeepageName(previousProfileName);
    const auto next = TrimSeepageName(nextProfileName);
    if (previous.empty() || next.empty()) {
        return 0U;
    }
    std::size_t changed = 0U;
    for (auto& node : nodes) {
        std::string* reference = nullptr;
        switch (half) {
            case WaterSeepageProfileHalf::NodeSettings:
                reference = &node.settingsProfileName;
                break;
            case WaterSeepageProfileHalf::Look:
                reference = &node.lookProfileName;
                break;
            case WaterSeepageProfileHalf::Response:
                reference = &node.responseProfileName;
                break;
        }
        if (reference == nullptr ||
            TrimSeepageName(*reference) != previous) {
            continue;
        }
        *reference = std::string{next};
        ++changed;
    }
    return changed;
}

WaterFlowProfileAssignmentRewrite ReplaceWaterFlowProfileAssignments(
    std::span<WaterEmitter> emitters,
    std::span<WaterManualFlowPathSource> manualPaths,
    WaterDynamicMeshFlowSettings* dynamicMesh,
    WaterFlowProfileKind kind,
    std::string_view previousProfileName,
    std::string_view nextProfileName,
    std::string_view dynamicMeshEditedTrailProfileName) {
    const auto normalize = [](std::string_view name,
                              std::string_view fallback) {
        const auto trimmed = TrimSeepageName(name);
        return trimmed.empty() ? std::string{fallback}
                               : std::string{trimmed};
    };
    const auto previousTrimmed = TrimSeepageName(previousProfileName);
    const auto nextTrimmed = TrimSeepageName(nextProfileName);
    WaterFlowProfileAssignmentRewrite rewrite;
    if (previousTrimmed.empty() || nextTrimmed.empty() ||
        previousTrimmed == nextTrimmed) {
        return rewrite;
    }
    const std::string previous{previousTrimmed};
    const std::string next{nextTrimmed};
    const auto replace = [&](std::string* assignment,
                             std::uint32_t sourceId,
                             std::vector<std::uint32_t>* changedIds) {
        if (assignment == nullptr || changedIds == nullptr ||
            normalize(*assignment, "Global") != previous) {
            return;
        }
        *assignment = next;
        changedIds->push_back(sourceId);
    };
    for (auto& emitter : emitters) {
        switch (kind) {
            case WaterFlowProfileKind::Path:
                replace(
                    &emitter.pathProfileName,
                    emitter.id,
                    &rewrite.pointSourceIds);
                break;
            case WaterFlowProfileKind::Lane:
                replace(
                    &emitter.laneProfileName,
                    emitter.id,
                    &rewrite.pointSourceIds);
                break;
            case WaterFlowProfileKind::Trail:
                replace(
                    &emitter.trailProfileName,
                    emitter.id,
                    &rewrite.pointSourceIds);
                break;
        }
    }
    if (kind != WaterFlowProfileKind::Path) {
        for (auto& source : manualPaths) {
            replace(
                kind == WaterFlowProfileKind::Lane
                    ? &source.laneProfileName
                    : &source.trailProfileName,
                source.id,
                &rewrite.manualPathSourceIds);
        }
    }
    if (kind == WaterFlowProfileKind::Trail && dynamicMesh != nullptr) {
        const auto unedited = [&](std::string_view name) {
            auto normalized = normalize(name, "Global");
            constexpr std::string_view kEditedSuffix = "_edited";
            constexpr std::string_view kLegacyEditedSuffix = "_Edited";
            if ((normalized.ends_with(kEditedSuffix) ||
                 normalized.ends_with(kLegacyEditedSuffix)) &&
                normalized.size() > kEditedSuffix.size()) {
                normalized.erase(
                    normalized.size() - kEditedSuffix.size());
            }
            return normalized;
        };
        const bool exactAssignment =
            normalize(dynamicMesh->trailProfileName, "Global") == previous;
        const auto editedName =
            TrimSeepageName(dynamicMeshEditedTrailProfileName);
        const bool matchingEditedProfile =
            !editedName.empty() &&
            normalize(dynamicMesh->trailProfileName, "Global") ==
                normalize(editedName, "Global") &&
            unedited(dynamicMesh->trailProfileName) == previous &&
            unedited(editedName) == previous;
        if (exactAssignment || matchingEditedProfile) {
            dynamicMesh->trailProfileName = next;
            rewrite.dynamicMeshTrailChanged = true;
            rewrite.dynamicMeshEditedTrailProfileMatched =
                matchingEditedProfile;
        }
    }
    return rewrite;
}

namespace {

bool WaterKeyedSettingBelongsToProfileGroup(
    const WaterKeyedSettingTrack& track,
    std::optional<WaterKeyedFeatureKind> featureKind,
    std::string_view profileGroup) {
    const auto group = TrimSeepageName(profileGroup);
    const auto existingGroup = TrimSeepageName(track.profileGroup);
    if (!existingGroup.empty()) {
        return existingGroup == group;
    }
    if (!featureKind.has_value()) {
        return false;
    }
    if (*featureKind == WaterKeyedFeatureKind::FlowPath) {
        return group == "flow_path" &&
               FindWaterKeyableSetting(
                   WaterKeyedFeatureKind::FlowPath,
                   track.settingId) != nullptr;
    }
    if (*featureKind != WaterKeyedFeatureKind::SeepageNode) {
        return false;
    }
    // Schema-46 tracks predate profile metadata. Dynamic Look/Response
    // members have stable prefixes; physical node settings come from the
    // fixed Seepage Node registry.
    if (group == "seepage_look") {
        return track.settingId.starts_with("look.");
    }
    if (group == "seepage_response") {
        return track.settingId.starts_with("response.");
    }
    if (group == "seepage_node_settings") {
        if (track.settingId.starts_with("look.") ||
            track.settingId.starts_with("response.")) {
            return false;
        }
        return FindWaterKeyableSetting(
                   WaterKeyedFeatureKind::SeepageNode,
                   track.settingId) != nullptr;
    }
    return false;
}

std::size_t ReplaceWaterKeyedSettingProfileReferencesImpl(
    std::span<WaterKeyedSettingTrack> settings,
    std::optional<WaterKeyedFeatureKind> featureKind,
    std::string_view profileGroup,
    std::string_view previousProfileName,
    std::string_view nextProfileName) {
    const auto group = TrimSeepageName(profileGroup);
    const auto previous = TrimSeepageName(previousProfileName);
    const auto next = TrimSeepageName(nextProfileName);
    if (group.empty() || previous.empty() || next.empty()) {
        return 0U;
    }
    std::size_t replacementCount = 0U;
    for (auto& setting : settings) {
        if (!WaterKeyedSettingBelongsToProfileGroup(
                setting,
                featureKind,
                group) ||
            TrimSeepageName(setting.profileName) != previous) {
            continue;
        }
        setting.profileGroup = std::string{group};
        setting.profileName = std::string{next};
        ++replacementCount;
    }
    return replacementCount;
}

}  // namespace

std::size_t ReplaceWaterKeyedSettingProfileReferences(
    std::span<WaterKeyedSettingTrack> settings,
    std::string_view profileGroup,
    std::string_view previousProfileName,
    std::string_view nextProfileName) {
    return ReplaceWaterKeyedSettingProfileReferencesImpl(
        settings,
        std::nullopt,
        profileGroup,
        previousProfileName,
        nextProfileName);
}

std::size_t ReplaceWaterKeyedSettingProfileReferences(
    std::span<WaterKeyedSettingTrack> settings,
    WaterKeyedFeatureKind featureKind,
    std::string_view profileGroup,
    std::string_view previousProfileName,
    std::string_view nextProfileName) {
    return ReplaceWaterKeyedSettingProfileReferencesImpl(
        settings,
        featureKind,
        profileGroup,
        previousProfileName,
        nextProfileName);
}

std::size_t ReplaceWaterKeyedSettingsProfileBaseReferences(
    std::span<WaterKeyedSettingsProfile> profiles,
    WaterKeyedFeatureKind featureKind,
    std::string_view previousProfileName,
    std::string_view nextProfileName) {
    const auto previous = TrimSeepageName(previousProfileName);
    const auto next = TrimSeepageName(nextProfileName);
    if (previous.empty() || next.empty() || previous == next) {
        return 0U;
    }
    std::size_t changed = 0U;
    for (auto& profile : profiles) {
        if (profile.featureKind != featureKind ||
            TrimSeepageName(profile.baseProfileName) != previous) {
            continue;
        }
        profile.baseProfileName = std::string{next};
        ++changed;
    }
    return changed;
}

std::size_t CanonicalizeWaterFeatureProfileMetadata(
    std::span<WaterFeatureTimingRun> runs,
    std::span<const WaterKeyedFeatureId> features,
    std::string_view profileGroup,
    std::string_view profileName) {
    const auto group = TrimSeepageName(profileGroup);
    const auto name = TrimSeepageName(profileName);
    if (features.empty() || group.empty() || name.empty()) {
        return 0U;
    }
    std::size_t changed = 0U;
    for (auto& run : runs) {
        for (auto& timeline : run.features) {
            if (std::find(features.begin(), features.end(), timeline.feature) ==
                features.end()) {
                continue;
            }
            for (auto& track : timeline.settings) {
                if (!WaterKeyedSettingBelongsToProfileGroup(
                        track,
                        timeline.feature.kind,
                        group) ||
                    (track.profileGroup == group &&
                     track.profileName == name)) {
                    continue;
                }
                track.profileGroup = std::string{group};
                track.profileName = std::string{name};
                ++changed;
            }
        }
    }
    return changed;
}

std::string WaterKeyedSettingsProfileSavedName(
    std::string_view baseProfileName,
    std::string_view objectName) {
    const auto trimmedBase = TrimSeepageName(baseProfileName);
    const auto trimmedObject = TrimSeepageName(objectName);
    const std::string base =
        trimmedBase.empty() ? std::string{"Default"}
                            : std::string{trimmedBase};
    const std::string object =
        trimmedObject.empty() ? std::string{"Path Source"}
                              : std::string{trimmedObject};
    return base + "_" + object;
}

std::string WaterKeyedSettingsProfileEditedName(
    std::string_view baseProfileName,
    std::string_view objectName) {
    return WaterKeyedSettingsProfileSavedName(
               baseProfileName,
               objectName) +
           "_edited";
}

WaterKeyedSettingsProfile SanitizeWaterKeyedSettingsProfile(
    WaterKeyedSettingsProfile profile) {
    const auto trimmedBase = TrimSeepageName(profile.baseProfileName);
    profile.baseProfileName =
        trimmedBase.empty() ? std::string{"Default"}
                            : std::string{trimmedBase};
    profile.ownerObjectName =
        std::string{TrimSeepageName(profile.ownerObjectName)};
    profile.sourceProfileName =
        std::string{TrimSeepageName(profile.sourceProfileName)};
    const auto trimmedName = TrimSeepageName(profile.name);
    profile.name =
        trimmedName.empty()
            ? (profile.edited
                   ? WaterKeyedSettingsProfileEditedName(
                         profile.baseProfileName,
                         profile.ownerObjectName)
                   : WaterKeyedSettingsProfileSavedName(
                         profile.baseProfileName,
                         profile.ownerObjectName))
            : std::string{trimmedName};
    profile.nativeLengthFraction = std::clamp(
        SeepageFiniteOr(profile.nativeLengthFraction, 1.0F),
        kWaterFeatureClipMinimumLength,
        1.0F);

    std::vector<WaterKeyedSettingTrack> kept;
    kept.reserve(profile.settings.size());
    for (auto& setting : profile.settings) {
        setting.settingId =
            std::string{TrimSeepageName(setting.settingId)};
        if (setting.settingId.empty() ||
            std::any_of(
                kept.begin(),
                kept.end(),
                [&](const WaterKeyedSettingTrack& existing) {
                    return existing.settingId == setting.settingId;
                })) {
            continue;
        }
        auto sanitized =
            SanitizeWaterKeyedSettingTrack(std::move(setting));
        // Reusable packages are ownership-neutral. Applying one assigns a
        // fresh destination clip id to every copied key.
        for (auto& key : sanitized.keys) {
            key.clipId = 0U;
        }
        kept.push_back(std::move(sanitized));
    }
    profile.settings = std::move(kept);
    return profile;
}

std::optional<std::size_t> FindWaterKeyedSettingsProfileIndex(
    std::span<const WaterKeyedSettingsProfile> profiles,
    WaterKeyedFeatureKind featureKind,
    std::string_view name) {
    const auto normalizedName = TrimSeepageName(name);
    if (normalizedName.empty()) {
        return std::nullopt;
    }
    for (std::size_t index = 0U; index < profiles.size(); ++index) {
        if (profiles[index].featureKind == featureKind &&
            TrimSeepageName(profiles[index].name) == normalizedName) {
            return index;
        }
    }
    return std::nullopt;
}

std::vector<WaterKeyedSettingsProfile>
SanitizeWaterKeyedSettingsProfileLibrary(
    std::vector<WaterKeyedSettingsProfile> profiles) {
    std::vector<WaterKeyedSettingsProfile> kept;
    kept.reserve(profiles.size());
    for (auto& profile : profiles) {
        auto sanitized = SanitizeWaterKeyedSettingsProfile(
            std::move(profile));
        if (sanitized.name.empty() ||
            FindWaterKeyedSettingsProfileIndex(
                kept,
                sanitized.featureKind,
                sanitized.name)
                .has_value()) {
            continue;
        }
        kept.push_back(std::move(sanitized));
    }
    return kept;
}

WaterFeatureTimingRun SanitizeWaterFeatureTimingRun(
    WaterFeatureTimingRun run) {
    if (run.name.empty()) {
        run.name = "Run";
    }
    std::unordered_set<std::uint32_t> reservedMarkIds;
    for (const auto& mark : run.marks) {
        if (mark.id != 0U) {
            reservedMarkIds.insert(mark.id);
        }
    }
    std::unordered_set<std::uint32_t> assignedMarkIds;
    const auto allocateSanitizedMarkId = [&] {
        std::uint32_t candidate = 1U;
        while ((reservedMarkIds.contains(candidate) ||
                assignedMarkIds.contains(candidate)) &&
               candidate != std::numeric_limits<std::uint32_t>::max()) {
            ++candidate;
        }
        return reservedMarkIds.contains(candidate) ||
                       assignedMarkIds.contains(candidate)
                   ? 0U
                   : candidate;
    };
    std::vector<WaterFeatureRunMark> marks;
    marks.reserve(run.marks.size());
    for (auto mark : run.marks) {
        mark.text = TrimSeepageName(mark.text);
        if (mark.text.empty()) {
            mark.text = "Mark";
        }
        mark.position = std::clamp(
            std::isfinite(mark.position) ? mark.position : 0.0F,
            0.0F,
            1.0F);
        if (mark.id == 0U || assignedMarkIds.contains(mark.id)) {
            mark.id = allocateSanitizedMarkId();
        }
        if (mark.id == 0U) {
            continue;
        }
        assignedMarkIds.insert(mark.id);
        marks.push_back(std::move(mark));
    }
    std::stable_sort(
        marks.begin(),
        marks.end(),
        [](const WaterFeatureRunMark& left,
           const WaterFeatureRunMark& right) {
            if (left.position != right.position) {
                return left.position < right.position;
            }
            return left.id < right.id;
        });
    run.marks = std::move(marks);
    for (auto& timeline : run.features) {
        std::vector<WaterKeyedSettingTrack> kept;
        kept.reserve(timeline.settings.size());
        for (auto& setting : timeline.settings) {
            if (setting.settingId.empty()) {
                continue;
            }
            const bool duplicate = std::any_of(
                kept.begin(),
                kept.end(),
                [&](const WaterKeyedSettingTrack& existing) {
                    return existing.settingId == setting.settingId;
                });
            if (duplicate) {
                continue;
            }
            kept.push_back(
                SanitizeWaterKeyedSettingTrack(std::move(setting)));
        }
        timeline.settings = std::move(kept);
        // Clips sanitize individually, then zero or colliding ids are
        // reassigned so every stored clip stays uniquely addressable.
        std::vector<WaterFeatureSettingsClip> clips;
        clips.reserve(timeline.clips.size());
        for (auto& clip : timeline.clips) {
            clips.push_back(
                SanitizeWaterFeatureSettingsClip(std::move(clip)));
        }
        std::stable_sort(
            clips.begin(),
            clips.end(),
            [](const WaterFeatureSettingsClip& left,
               const WaterFeatureSettingsClip& right) {
                return left.start < right.start;
            });
        std::uint32_t nextId = 1U;
        for (const auto& clip : clips) {
            if (clip.id >= nextId) {
                nextId = clip.id + 1U;
            }
        }
        for (std::size_t index = 0U; index < clips.size(); ++index) {
            auto& clip = clips[index];
            const bool collides =
                clip.id == 0U ||
                std::any_of(
                    clips.begin(),
                    clips.begin() + static_cast<std::ptrdiff_t>(index),
                    [&](const WaterFeatureSettingsClip& earlier) {
                        return earlier.id == clip.id;
                    });
            if (collides) {
                clip.id = nextId++;
            }
        }
        timeline.clips = std::move(clips);

        if (!timeline.clipMembershipExplicit) {
            // Pre-membership documents grouped keys implicitly by a
            // non-overlapping time window. Migrate once. At a shared
            // boundary the later-starting clip owns the key, matching the
            // evaluator rule that an interior key starts the next segment.
            constexpr float kLegacyMembershipTolerance = 1.0e-4F;
            for (auto& setting : timeline.settings) {
                for (auto& key : setting.keys) {
                    const WaterFeatureSettingsClip* owner = nullptr;
                    for (const auto& clip : timeline.clips) {
                        if (!WaterClipContainsPosition(
                                clip.start,
                                clip.end,
                                key.position)) {
                            continue;
                        }
                        if (owner == nullptr ||
                            clip.start > owner->start +
                                             kLegacyMembershipTolerance ||
                            (std::abs(clip.start - owner->start) <=
                                 kLegacyMembershipTolerance &&
                             clip.id < owner->id)) {
                            owner = &clip;
                        }
                    }
                    key.clipId = owner != nullptr ? owner->id : 0U;
                }
            }
        } else {
            // Corrupt or hand-edited references become deliberately loose;
            // they must never be captured by an unrelated clip merely
            // because its window happens to overlap their time.
            for (auto& setting : timeline.settings) {
                for (auto& key : setting.keys) {
                    if (key.clipId == 0U) {
                        continue;
                    }
                    const bool ownerExists = std::any_of(
                        timeline.clips.begin(),
                        timeline.clips.end(),
                        [&](const WaterFeatureSettingsClip& clip) {
                            return clip.id == key.clipId;
                        });
                    if (!ownerExists) {
                        key.clipId = 0U;
                    }
                }
            }
        }
        timeline.clipMembershipExplicit = true;

        // Keyed clip bounds are derived from their actual members, which
        // keeps additions, key drags, and overlapping windows consistent.
        std::vector<std::uint32_t> clipIds;
        clipIds.reserve(timeline.clips.size());
        for (const auto& clip : timeline.clips) {
            clipIds.push_back(clip.id);
        }
        for (const auto clipId : clipIds) {
            (void)SynchronizeWaterFeatureClipBounds(
                &timeline,
                clipId);
        }
    }
    return run;
}

std::uint32_t AllocateWaterFeatureRunMarkId(
    const WaterFeatureTimingRun& run) {
    const auto used = [&](std::uint32_t id) {
        return id == 0U || std::any_of(
            run.marks.begin(),
            run.marks.end(),
            [&](const WaterFeatureRunMark& mark) {
                return mark.id == id;
            });
    };
    std::uint32_t candidate = 1U;
    while (used(candidate) &&
           candidate != std::numeric_limits<std::uint32_t>::max()) {
        ++candidate;
    }
    return used(candidate) ? 0U : candidate;
}

std::string AllocateWaterFeatureRunMarkName(
    std::span<const WaterFeatureTimingRun> runs) {
    std::size_t markCount = 0U;
    for (const auto& run : runs) {
        markCount += run.marks.size();
    }
    for (std::size_t number = 0U; number <= markCount; ++number) {
        std::ostringstream stream;
        stream << "Mark " << std::setfill('0') << std::setw(2) << number;
        const std::string candidate = stream.str();
        const bool used = std::any_of(
            runs.begin(),
            runs.end(),
            [&](const WaterFeatureTimingRun& run) {
                return std::any_of(
                    run.marks.begin(),
                    run.marks.end(),
                    [&](const WaterFeatureRunMark& mark) {
                        return mark.text == candidate;
                    });
            });
        if (!used) {
            return candidate;
        }
    }
    return "Mark";
}

WaterFeatureRunMark* FindWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId) {
    if (run == nullptr || markId == 0U) {
        return nullptr;
    }
    const auto mark = std::find_if(
        run->marks.begin(),
        run->marks.end(),
        [&](const WaterFeatureRunMark& candidate) {
            return candidate.id == markId;
        });
    return mark != run->marks.end() ? &*mark : nullptr;
}

const WaterFeatureRunMark* FindWaterFeatureRunMark(
    const WaterFeatureTimingRun* run,
    std::uint32_t markId) {
    if (run == nullptr || markId == 0U) {
        return nullptr;
    }
    const auto mark = std::find_if(
        run->marks.begin(),
        run->marks.end(),
        [&](const WaterFeatureRunMark& candidate) {
            return candidate.id == markId;
        });
    return mark != run->marks.end() ? &*mark : nullptr;
}

bool MoveWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId,
    float position) {
    auto* mark = FindWaterFeatureRunMark(run, markId);
    if (mark == nullptr) {
        return false;
    }
    const float next = std::clamp(
        std::isfinite(position) ? position : 0.0F,
        0.0F,
        1.0F);
    if (mark->position == next) {
        return false;
    }
    mark->position = next;
    return true;
}

bool RenameWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId,
    std::string_view text) {
    auto* mark = FindWaterFeatureRunMark(run, markId);
    const auto next = TrimSeepageName(text);
    if (mark == nullptr || next.empty() || mark->text == next) {
        return false;
    }
    mark->text = next;
    return true;
}

bool RemoveWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId) {
    if (run == nullptr || markId == 0U) {
        return false;
    }
    const auto previousSize = run->marks.size();
    std::erase_if(
        run->marks,
        [&](const WaterFeatureRunMark& mark) {
            return mark.id == markId;
        });
    return run->marks.size() != previousSize;
}

bool AssignWaterFeatureToTimingRun(
    WaterScenarioFeatureRuns* entry,
    const WaterKeyedFeatureId& feature,
    std::uint32_t targetRunId) {
    if (entry == nullptr) {
        return false;
    }
    const auto targetIt = std::find_if(
        entry->runs.begin(),
        entry->runs.end(),
        [&](const WaterFeatureTimingRun& run) {
            return run.id == targetRunId;
        });
    if (targetIt == entry->runs.end()) {
        return false;
    }

    std::optional<WaterFeatureTimeline> movedTimeline;
    for (auto& run : entry->runs) {
        const auto timelineIt = std::find_if(
            run.features.begin(),
            run.features.end(),
            [&](const WaterFeatureTimeline& timeline) {
                return timeline.feature == feature;
            });
        if (timelineIt == run.features.end()) {
            continue;
        }
        if (run.id == targetRunId) {
            return true;
        }
        movedTimeline = std::move(*timelineIt);
        run.features.erase(timelineIt);
        break;
    }

    auto destination = std::find_if(
        entry->runs.begin(),
        entry->runs.end(),
        [&](const WaterFeatureTimingRun& run) {
            return run.id == targetRunId;
        });
    if (destination == entry->runs.end()) {
        return false;
    }
    destination->features.push_back(
        movedTimeline.has_value()
            ? std::move(movedTimeline.value())
            : WaterFeatureTimeline{.feature = feature});
    return true;
}

std::size_t RemoveWaterFeatureFromTimingRuns(
    std::span<WaterFeatureTimingRun> runs,
    const WaterKeyedFeatureId& feature) {
    std::size_t removed = 0U;
    for (auto& run : runs) {
        removed += std::erase_if(
            run.features,
            [&](const WaterFeatureTimeline& timeline) {
                return timeline.feature == feature;
            });
    }
    return removed;
}

namespace {

float WrapNormalizedLoopPosition(float position) {
    position = SeepageFiniteOr(position, 0.0F);
    position -= std::floor(position);
    return position >= 1.0F ? 0.0F : position;
}

std::optional<float> EvaluateWaterKeyedSettingTrackImpl(
    const WaterKeyedSettingTrack& track,
    float normalizedPosition,
    bool cyclic) {
    if (!track.active || track.keys.empty()) {
        return std::nullopt;
    }
    normalizedPosition = cyclic
        ? WrapNormalizedLoopPosition(normalizedPosition)
        : Clamp01(SeepageFiniteOr(normalizedPosition, 0.0F));
    // TrackDefault keys resolve to the track's default style before any
    // segment math, because the spline segments read neighbouring keys'
    // interpolation to find mode boundaries.
    const auto trackDefault =
        track.defaultInterpolation == WaterScenarioInterpolation::TrackDefault
            ? WaterScenarioInterpolation::SmoothVelocity
            : track.defaultInterpolation;
    std::vector<WaterSettingKey> resolved = track.keys;
    for (auto& key : resolved) {
        if (cyclic) {
            key.position = WrapNormalizedLoopPosition(key.position);
        }
        if (key.interpolation == WaterScenarioInterpolation::TrackDefault) {
            key.interpolation = trackDefault;
        }
    }
    if (cyclic) {
        std::stable_sort(
            resolved.begin(),
            resolved.end(),
            [](const WaterSettingKey& left, const WaterSettingKey& right) {
                return left.position < right.position;
            });
        std::vector<WaterSettingKey> canonical;
        canonical.reserve(resolved.size());
        for (auto& key : resolved) {
            if (!canonical.empty() &&
                std::abs(canonical.back().position - key.position) <=
                    kWaterKeySplineTolerance) {
                canonical.back() = std::move(key);
            } else {
                canonical.push_back(std::move(key));
            }
        }
        resolved.clear();
        resolved.reserve(canonical.size() * 3U);
        for (const float shift : {-1.0F, 0.0F, 1.0F}) {
            for (const auto& key : canonical) {
                auto copy = key;
                copy.position += shift;
                resolved.push_back(std::move(copy));
            }
        }
    }
    std::vector<const WaterSettingKey*> ordered;
    ordered.reserve(resolved.size());
    for (const auto& key : resolved) {
        ordered.push_back(&key);
    }
    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [](const WaterSettingKey* left, const WaterSettingKey* right) {
            return left->position < right->position;
        });
    const auto keyValue = [](const WaterSettingKey& key) {
        return SeepageFiniteOr(key.value, 0.0F);
    };
    if (normalizedPosition <= ordered.front()->position) {
        return keyValue(*ordered.front());
    }
    if (normalizedPosition >= ordered.back()->position) {
        return keyValue(*ordered.back());
    }
    for (std::size_t index = 0U; index + 1U < ordered.size(); ++index) {
        const auto& left = *ordered[index];
        const auto& right = *ordered[index + 1U];
        // An exact interior key position belongs to the segment it starts,
        // so sampling at a key after a Hold segment yields the post-step
        // value instead of the stale left limit.
        if (normalizedPosition >= right.position) {
            continue;
        }
        if (left.interpolation == WaterScenarioInterpolation::Hold) {
            return keyValue(left);
        }
        const float span =
            std::max(1.0e-6F, right.position - left.position);
        float amount =
            Clamp01((normalizedPosition - left.position) / span);
        if (left.interpolation ==
            WaterScenarioInterpolation::SmoothVelocity) {
            return EvaluateSmoothVelocityKeySegment(
                ordered,
                index,
                amount,
                keyValue);
        }
        if (left.interpolation ==
            WaterScenarioInterpolation::CentripetalCatmullRom) {
            return EvaluateCentripetalCatmullRomKeySegment(
                ordered,
                index,
                normalizedPosition,
                amount,
                keyValue);
        }
        if (left.interpolation ==
            WaterScenarioInterpolation::SplineHandles) {
            return EvaluateWaterSettingSplineHandleSegment(
                left,
                right,
                amount);
        }
        if (left.interpolation == WaterScenarioInterpolation::Smooth) {
            amount = amount * amount * (3.0F - 2.0F * amount);
        }
        return std::lerp(keyValue(left), keyValue(right), amount);
    }
    return keyValue(*ordered.back());
}

}  // namespace

std::optional<float> EvaluateWaterKeyedSettingTrack(
    const WaterKeyedSettingTrack& track,
    float normalizedPosition) {
    return EvaluateWaterKeyedSettingTrackImpl(
        track,
        normalizedPosition,
        false);
}

std::optional<float> EvaluateWaterKeyedSettingTrackCyclic(
    const WaterKeyedSettingTrack& track,
    float normalizedLoopPosition) {
    return EvaluateWaterKeyedSettingTrackImpl(
        track,
        normalizedLoopPosition,
        true);
}

void AddOrUpdateWaterSettingKey(
    WaterKeyedSettingTrack* track,
    float position,
    float value,
    WaterScenarioInterpolation interpolation,
    std::optional<std::uint32_t> clipId) {
    if (track == nullptr) {
        return;
    }
    constexpr float kReplacementTolerance = 1.0e-4F;
    WaterSettingKey key{
        .position = Clamp01(SeepageFiniteOr(position, 0.0F)),
        .value = SeepageFiniteOr(value, 0.0F),
        .interpolation = interpolation,
        .clipId = clipId.value_or(0U),
    };
    const auto existing = std::find_if(
        track->keys.begin(),
        track->keys.end(),
        [&](const WaterSettingKey& candidate) {
            return std::abs(candidate.position - key.position) <=
                   kReplacementTolerance;
        });
    if (existing != track->keys.end()) {
        // Value and interpolation edits must not silently reset authored
        // Bezier handles on the same key.
        existing->position = key.position;
        existing->value = key.value;
        existing->interpolation = key.interpolation;
        if (clipId.has_value()) {
            existing->clipId = clipId.value();
        }
    } else {
        track->keys.push_back(key);
    }
    std::stable_sort(
        track->keys.begin(),
        track->keys.end(),
        [](const WaterSettingKey& left, const WaterSettingKey& right) {
            return left.position < right.position;
        });
}

void AddOrUpdateWaterTimelineSettingKey(
    WaterFeatureTimeline* timeline,
    WaterKeyedSettingTrack* track,
    float position,
    float value,
    WaterScenarioInterpolation interpolation,
    std::optional<std::uint32_t> selectedClipId) {
    if (timeline == nullptr || track == nullptr) {
        return;
    }
    constexpr float kReplacementTolerance = 1.0e-4F;
    const auto existing = std::find_if(
        track->keys.begin(),
        track->keys.end(),
        [&](const WaterSettingKey& key) {
            return std::abs(key.position - position) <=
                   kReplacementTolerance;
        });
    const bool replacingExisting = existing != track->keys.end();
    const std::uint32_t previousClipId =
        replacingExisting ? existing->clipId : 0U;
    if (selectedClipId.has_value() &&
        FindWaterFeatureClip(timeline, selectedClipId.value()) == nullptr) {
        selectedClipId.reset();
    }
    // FR-STYLE-12 assigns only newly authored keys to the single selected
    // stored clip. Supplying no id to the lower-level replacement API keeps
    // an existing key's explicit owner intact.
    const auto newKeyClipId = replacingExisting
                                  ? std::nullopt
                                  : selectedClipId;
    if (newKeyClipId.has_value()) {
        timeline->clipMembershipExplicit = true;
    }
    AddOrUpdateWaterSettingKey(
        track,
        position,
        value,
        interpolation,
        newKeyClipId);
    if (previousClipId != 0U) {
        (void)SynchronizeWaterFeatureClipBounds(
            timeline,
            previousClipId);
    }
    if (newKeyClipId.value_or(0U) != 0U &&
        newKeyClipId.value_or(0U) != previousClipId) {
        (void)SynchronizeWaterFeatureClipBounds(
            timeline,
            newKeyClipId.value());
    }
}

bool MoveWaterSettingKey(
    WaterKeyedSettingTrack* track,
    float sourcePosition,
    float destinationPosition) {
    constexpr float kKeyTolerance = 1.0e-4F;
    if (track == nullptr || !std::isfinite(sourcePosition) ||
        !std::isfinite(destinationPosition) ||
        destinationPosition < 0.0F || destinationPosition > 1.0F) {
        return false;
    }
    const auto source = std::find_if(
        track->keys.begin(),
        track->keys.end(),
        [&](const WaterSettingKey& candidate) {
            return std::abs(candidate.position - sourcePosition) <=
                   kKeyTolerance;
        });
    if (source == track->keys.end()) {
        return false;
    }
    const bool destinationOccupied = std::any_of(
        track->keys.begin(),
        track->keys.end(),
        [&](const WaterSettingKey& candidate) {
            return &candidate != &*source &&
                   std::abs(candidate.position - destinationPosition) <=
                       kKeyTolerance;
        });
    if (destinationOccupied) {
        return false;
    }
    source->position = destinationPosition;
    std::stable_sort(
        track->keys.begin(),
        track->keys.end(),
        [](const WaterSettingKey& left, const WaterSettingKey& right) {
            return left.position < right.position;
        });
    return true;
}

std::optional<WaterSettingSplineHandlePoint>
ResolveWaterSettingSplineHandlePoint(
    const WaterKeyedSettingTrack& track,
    float keyPosition,
    WaterSettingSplineHandleSide side,
    bool cyclic) {
    if (!std::isfinite(keyPosition) || track.keys.size() < 2U) {
        return std::nullopt;
    }
    std::vector<const WaterSettingKey*> ordered;
    ordered.reserve(track.keys.size());
    for (const auto& key : track.keys) {
        ordered.push_back(&key);
    }
    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [](const WaterSettingKey* left, const WaterSettingKey* right) {
            return left->position < right->position;
        });
    const auto selected = std::find_if(
        ordered.begin(),
        ordered.end(),
        [&](const WaterSettingKey* key) {
            return std::abs(key->position - keyPosition) <=
                   kWaterKeySplineTolerance;
        });
    if (selected == ordered.end()) {
        return std::nullopt;
    }
    const std::size_t index = static_cast<std::size_t>(
        std::distance(ordered.begin(), selected));
    const WaterSettingKey* left = nullptr;
    const WaterSettingKey* right = nullptr;
    const WaterSettingKey* anchor = *selected;
    float leftPosition = 0.0F;
    float rightPosition = 0.0F;
    if (side == WaterSettingSplineHandleSide::Outgoing) {
        if (ResolveWaterSettingInterpolation(track, **selected) !=
                WaterScenarioInterpolation::SplineHandles) {
            return std::nullopt;
        }
        left = *selected;
        leftPosition = left->position;
        if (index + 1U < ordered.size()) {
            right = ordered[index + 1U];
            rightPosition = right->position;
        } else if (cyclic) {
            right = ordered.front();
            rightPosition = right->position + 1.0F;
        } else {
            return std::nullopt;
        }
    } else {
        if (index > 0U) {
            left = ordered[index - 1U];
            leftPosition = left->position;
        } else if (cyclic) {
            left = ordered.back();
            leftPosition = left->position - 1.0F;
        } else {
            return std::nullopt;
        }
        if (ResolveWaterSettingInterpolation(track, *left) !=
                WaterScenarioInterpolation::SplineHandles) {
            return std::nullopt;
        }
        right = *selected;
        rightPosition = right->position;
    }
    const float span = rightPosition - leftPosition;
    if (span <= kWaterKeySplineTolerance) {
        return std::nullopt;
    }
    const auto fractions =
        ResolveWaterSettingSplineHandleFractions(*left, *right);
    return WaterSettingSplineHandlePoint{
        .anchorPosition = anchor->position,
        .anchorValue = anchor->value,
        .controlPosition =
            side == WaterSettingSplineHandleSide::Outgoing
                ? leftPosition + span * fractions.outgoing
                : rightPosition - span * fractions.incoming,
        .controlValue =
            side == WaterSettingSplineHandleSide::Outgoing
                ? left->value + left->outgoingHandleValue
                : right->value + right->incomingHandleValue,
    };
}

bool MoveWaterSettingSplineHandlePoint(
    WaterKeyedSettingTrack* track,
    float keyPosition,
    WaterSettingSplineHandleSide side,
    float controlPosition,
    float controlValue,
    bool cyclic) {
    if (track == nullptr || !std::isfinite(keyPosition) ||
        !std::isfinite(controlPosition) || !std::isfinite(controlValue) ||
        track->keys.size() < 2U) {
        return false;
    }
    std::vector<WaterSettingKey*> ordered;
    ordered.reserve(track->keys.size());
    for (auto& key : track->keys) {
        ordered.push_back(&key);
    }
    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [](const WaterSettingKey* left, const WaterSettingKey* right) {
            return left->position < right->position;
        });
    const auto selected = std::find_if(
        ordered.begin(),
        ordered.end(),
        [&](const WaterSettingKey* key) {
            return std::abs(key->position - keyPosition) <=
                   kWaterKeySplineTolerance;
        });
    if (selected == ordered.end()) {
        return false;
    }
    const std::size_t index = static_cast<std::size_t>(
        std::distance(ordered.begin(), selected));
    WaterSettingKey* left = nullptr;
    WaterSettingKey* right = nullptr;
    WaterSettingKey* anchor = *selected;
    float leftPosition = 0.0F;
    float rightPosition = 0.0F;
    if (side == WaterSettingSplineHandleSide::Outgoing) {
        if (ResolveWaterSettingInterpolation(*track, **selected) !=
                WaterScenarioInterpolation::SplineHandles) {
            return false;
        }
        left = *selected;
        leftPosition = left->position;
        if (index + 1U < ordered.size()) {
            right = ordered[index + 1U];
            rightPosition = right->position;
        } else if (cyclic) {
            right = ordered.front();
            rightPosition = right->position + 1.0F;
        } else {
            return false;
        }
    } else {
        if (index > 0U) {
            left = ordered[index - 1U];
            leftPosition = left->position;
        } else if (cyclic) {
            left = ordered.back();
            leftPosition = left->position - 1.0F;
        } else {
            return false;
        }
        if (ResolveWaterSettingInterpolation(*track, *left) !=
                WaterScenarioInterpolation::SplineHandles) {
            return false;
        }
        right = *selected;
        rightPosition = right->position;
    }
    const float span = rightPosition - leftPosition;
    if (span <= kWaterKeySplineTolerance) {
        return false;
    }
    const auto fractions =
        ResolveWaterSettingSplineHandleFractions(*left, *right);
    // Materialize any paired normalization before changing one side so the
    // user's new control coordinate remains stable on the following frame.
    left->outgoingHandleTime = fractions.outgoing;
    right->incomingHandleTime = fractions.incoming;
    if (side == WaterSettingSplineHandleSide::Outgoing) {
        const float maximumFraction =
            kWaterSettingSplineHandleMaximumPair - fractions.incoming;
        left->outgoingHandleTime = std::clamp(
            (controlPosition - leftPosition) / span,
            kWaterSettingSplineHandleMinimumFraction,
            maximumFraction);
        left->outgoingHandleValue = controlValue - anchor->value;
    } else {
        const float maximumFraction =
            kWaterSettingSplineHandleMaximumPair - fractions.outgoing;
        right->incomingHandleTime = std::clamp(
            (rightPosition - controlPosition) / span,
            kWaterSettingSplineHandleMinimumFraction,
            maximumFraction);
        right->incomingHandleValue = controlValue - anchor->value;
    }
    return true;
}

std::size_t WaterSettingKeyCountAtPosition(
    const WaterKeyedSettingTrack& track,
    float position) {
    constexpr float kKeyTolerance = 1.0e-4F;
    if (!std::isfinite(position)) {
        return 0U;
    }
    return static_cast<std::size_t>(std::count_if(
        track.keys.begin(),
        track.keys.end(),
        [&](const WaterSettingKey& key) {
            return std::abs(key.position - position) <=
                   kKeyTolerance;
        }));
}

std::size_t RemoveWaterSettingKeysAtPosition(
    WaterKeyedSettingTrack* track,
    float position) {
    if (track == nullptr) {
        return 0U;
    }
    const auto previousSize = track->keys.size();
    std::erase_if(
        track->keys,
        [&](const WaterSettingKey& key) {
            constexpr float kKeyTolerance = 1.0e-4F;
            return std::isfinite(position) &&
                   std::abs(key.position - position) <=
                       kKeyTolerance;
        });
    return previousSize - track->keys.size();
}

std::size_t WaterFeatureKeyCountAtPosition(
    const WaterFeatureTimeline& timeline,
    float position) {
    std::size_t count = 0U;
    for (const auto& setting : timeline.settings) {
        if (!setting.active) {
            continue;
        }
        count += WaterSettingKeyCountAtPosition(setting, position);
    }
    return count;
}

std::size_t RemoveWaterFeatureKeysAtPosition(
    WaterFeatureTimeline* timeline,
    float position) {
    if (timeline == nullptr) {
        return 0U;
    }
    std::vector<std::uint32_t> affectedClipIds;
    for (const auto& setting : timeline->settings) {
        if (!setting.active) {
            continue;
        }
        for (const auto& key : setting.keys) {
            if (key.clipId != 0U && std::isfinite(position) &&
                std::abs(key.position - position) <= 1.0e-4F &&
                std::find(
                    affectedClipIds.begin(),
                    affectedClipIds.end(),
                    key.clipId) == affectedClipIds.end()) {
                affectedClipIds.push_back(key.clipId);
            }
        }
    }
    std::size_t removed = 0U;
    for (auto& setting : timeline->settings) {
        if (!setting.active) {
            continue;
        }
        removed += RemoveWaterSettingKeysAtPosition(
            &setting,
            position);
    }
    for (const auto clipId : affectedClipIds) {
        (void)SynchronizeWaterFeatureClipBounds(timeline, clipId);
    }
    return removed;
}

std::optional<float> PreviousWaterSettingKeyPosition(
    const WaterKeyedSettingTrack& track,
    float position) {
    if (!track.active) {
        return std::nullopt;
    }
    constexpr float kNeighbourTolerance = 1.0e-4F;
    std::optional<float> best;
    for (const auto& key : track.keys) {
        if (key.position < position - kNeighbourTolerance &&
            (!best.has_value() || key.position > *best)) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float> NextWaterSettingKeyPosition(
    const WaterKeyedSettingTrack& track,
    float position) {
    if (!track.active) {
        return std::nullopt;
    }
    constexpr float kNeighbourTolerance = 1.0e-4F;
    std::optional<float> best;
    for (const auto& key : track.keys) {
        if (key.position > position + kNeighbourTolerance &&
            (!best.has_value() || key.position < *best)) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float> PreviousWaterFeatureKeyPosition(
    const WaterFeatureTimeline& timeline,
    float position) {
    std::optional<float> best;
    for (const auto& setting : timeline.settings) {
        const auto candidate =
            PreviousWaterSettingKeyPosition(setting, position);
        if (candidate.has_value() &&
            (!best.has_value() || *candidate > *best)) {
            best = candidate;
        }
    }
    return best;
}

std::optional<float> NextWaterFeatureKeyPosition(
    const WaterFeatureTimeline& timeline,
    float position) {
    std::optional<float> best;
    for (const auto& setting : timeline.settings) {
        const auto candidate =
            NextWaterSettingKeyPosition(setting, position);
        if (candidate.has_value() &&
            (!best.has_value() || *candidate < *best)) {
            best = candidate;
        }
    }
    return best;
}

std::vector<float> WaterFeatureProfileKeyPositions(
    const WaterFeatureTimeline& timeline,
    std::string_view profileGroup) {
    constexpr float kPositionTolerance = 1.0e-4F;
    std::vector<float> positions;
    for (const auto& setting : timeline.settings) {
        if (!setting.active || setting.profileGroup != profileGroup) {
            continue;
        }
        for (const auto& key : setting.keys) {
            if (std::isfinite(key.position)) {
                positions.push_back(Clamp01(key.position));
            }
        }
    }
    std::sort(positions.begin(), positions.end());
    positions.erase(
        std::unique(
            positions.begin(),
            positions.end(),
            [](float left, float right) {
                return std::abs(left - right) <=
                       kPositionTolerance;
            }),
        positions.end());
    return positions;
}

namespace {

constexpr float kWaterClipKeyTolerance = kWaterFeatureClipPositionTolerance;

// Membership shared by every span operation; cyclic for wrapped spans (W1).
bool WaterClipSpanContains(float start, float end, float position) {
    return WaterClipContainsPosition(start, end, position);
}

// Shortest signed distance between two phases on the unit loop.
float CyclicWaterClipDistance(float left, float right) {
    const float difference = std::abs(left - right);
    return std::min(difference, std::abs(1.0F - difference));
}

// The affine map shared by every span operation. Positions inside the
// source range land inside the destination window; the caller guarantees a
// non-degenerate source span. A stored position ahead of a wrapped source
// range's seam is unwrapped first. With wrap only a result past 1 rolls
// back by one cycle; a result landing on 1 itself stays stored at 1, exactly
// like the unlinked path, so a destination span that ends on phase 1 (an
// unwrapped {start, 1}) still contains its own end key (a stored key at 0 is
// not a member of an unwrapped clip ending at 1). Without wrap the pre-W1
// Clamp01 applies, which keeps every unlinked transform bit-identical.
float RemapWaterClipPosition(
    float position,
    float rangeStart,
    float rangeEnd,
    float newStart,
    float newEnd,
    bool wrap = false) {
    const float span = std::max(1.0e-6F, rangeEnd - rangeStart);
    const float unwrapped = UnwrapWaterClipPosition(position, rangeStart);
    const float fraction = (unwrapped - rangeStart) / span;
    const float mapped = newStart + fraction * (newEnd - newStart);
    if (wrap && mapped > 1.0F + kWaterClipKeyTolerance) {
        return WrapWaterClipPhase(mapped - 1.0F);
    }
    return Clamp01(mapped);
}

// Same map in unwrapped coordinates (no wrap, no clamp); used where the
// result is compared cyclically or the span itself is remapped.
float RemapWaterClipPositionUnwrapped(
    float position,
    float rangeStart,
    float rangeEnd,
    float newStart,
    float newEnd) {
    const float span = std::max(1.0e-6F, rangeEnd - rangeStart);
    const float unwrapped = UnwrapWaterClipPosition(position, rangeStart);
    const float fraction = (unwrapped - rangeStart) / span;
    return newStart + fraction * (newEnd - newStart);
}

bool WaterClipRangeIsValid(float start, float end) {
    return std::isfinite(start) && std::isfinite(end) &&
           end - start >= kWaterClipKeyTolerance &&
           end - start <= 1.0F + kWaterClipKeyTolerance;
}

void SortWaterFeatureClips(std::vector<WaterFeatureSettingsClip>* clips) {
    std::stable_sort(
        clips->begin(),
        clips->end(),
        [](const WaterFeatureSettingsClip& left,
           const WaterFeatureSettingsClip& right) {
            return left.start < right.start;
        });
}

bool WaterClipIdIsSelected(
    std::span<const std::uint32_t> clipIds,
    std::uint32_t clipId) {
    return std::find(clipIds.begin(), clipIds.end(), clipId) !=
           clipIds.end();
}

// Runtime-created test/support timelines and pre-membership documents may
// still carry implicit spans. Convert once before any ownership-sensitive
// operation. Current UI-created loose keys are protected by the explicit
// marker and therefore never get captured merely by entering a clip window.
void EnsureWaterFeatureExplicitClipMembership(
    WaterFeatureTimeline* timeline) {
    if (timeline == nullptr || timeline->clipMembershipExplicit) {
        return;
    }
    for (auto& setting : timeline->settings) {
        for (auto& key : setting.keys) {
            const WaterFeatureSettingsClip* owner = nullptr;
            for (const auto& clip : timeline->clips) {
                if (!WaterClipSpanContains(
                        clip.start,
                        clip.end,
                        key.position)) {
                    continue;
                }
                if (owner == nullptr ||
                    clip.start > owner->start + kWaterClipKeyTolerance ||
                    (std::abs(clip.start - owner->start) <=
                         kWaterClipKeyTolerance &&
                     clip.id < owner->id)) {
                    owner = &clip;
                }
            }
            key.clipId = owner != nullptr ? owner->id : 0U;
        }
    }
    timeline->clipMembershipExplicit = true;
    std::vector<std::uint32_t> clipIds;
    clipIds.reserve(timeline->clips.size());
    for (const auto& clip : timeline->clips) {
        clipIds.push_back(clip.id);
    }
    for (const auto clipId : clipIds) {
        (void)SynchronizeWaterFeatureClipBounds(timeline, clipId);
    }
}

// Searches the window in unwrapped coordinates (windowEnd may exceed 1) and
// compares against existing keys cyclically, so a window spanning phase 0
// nudges around keys on both sides of the seam. Returns the unwrapped
// position; the caller stores WrapWaterClipPhase of it.
std::optional<float> AvailableWaterClipKeyPosition(
    const WaterKeyedSettingTrack& track,
    float desired,
    float windowStart,
    float windowEnd) {
    const auto available = [&](float candidate) {
        return candidate >= windowStart && candidate <= windowEnd &&
               std::none_of(
                   track.keys.begin(),
                   track.keys.end(),
                   [&](const WaterSettingKey& existing) {
                       return CyclicWaterClipDistance(
                                  existing.position,
                                  WrapWaterClipPhase(candidate)) <=
                              kWaterClipKeyTolerance;
                   });
    };
    desired = std::clamp(desired, windowStart, windowEnd);
    if (available(desired)) {
        return desired;
    }
    constexpr float kStep = 2.1F * kWaterClipKeyTolerance;
    const std::size_t maximumAttempts = static_cast<std::size_t>(
        std::ceil((windowEnd - windowStart) / kStep)) + 1U;
    for (std::size_t attempt = 1U; attempt <= maximumAttempts; ++attempt) {
        const float offset = static_cast<float>(attempt) * kStep;
        const float inwardFirst =
            desired <= std::midpoint(windowStart, windowEnd)
                ? desired + offset
                : desired - offset;
        const float other =
            desired <= std::midpoint(windowStart, windowEnd)
                ? desired - offset
                : desired + offset;
        if (available(inwardFirst)) {
            return inwardFirst;
        }
        if (available(other)) {
            return other;
        }
    }
    return std::nullopt;
}

}  // namespace

std::pair<float, float> WaterFeatureClipCoveringArc(
    std::span<const float> positions,
    float hintStart) {
    std::vector<float> sorted;
    sorted.reserve(positions.size());
    for (const float position : positions) {
        if (std::isfinite(position)) {
            sorted.push_back(Clamp01(position));
        }
    }
    if (sorted.empty()) {
        return {0.0F, 0.0F};
    }
    std::ranges::sort(sorted);
    sorted.erase(
        std::unique(
            sorted.begin(),
            sorted.end(),
            [](float left, float right) {
                return std::abs(left - right) <=
                       kWaterFeatureClipPositionTolerance;
            }),
        sorted.end());
    // Every member is a candidate start; its arc runs forward around the
    // loop to the member just before it. All candidates cover every member
    // with length <= 1, so the choice is only about which gap is left open.
    // Minimal length alone would flip an ordinary clip whose keys straddle
    // the rail middle (keys at 0.1 and 0.9 -> {0.9,1.1}), so the arc whose
    // start is cyclically nearest the clip's present start wins: unwrapped
    // clips stay unwrapped as their keys are dragged, and a clip that
    // crossed phase 0 stays wrapped. Ties (keys at both 0 and 1, which are
    // one phase but distinct stored keys) keep the earlier, longer arc so a
    // legacy full-rail clip keyed at 0 and 1 stays {0,1}.
    const float hint = std::isfinite(hintStart)
        ? WrapWaterClipPhase(hintStart)
        : 0.0F;
    std::optional<std::pair<float, float>> best;
    float bestDistance = 0.0F;
    const std::size_t count = sorted.size();
    for (std::size_t index = 0U; index < count; ++index) {
        float start = sorted[index];
        float end = index == 0U
            ? sorted[count - 1U]
            : sorted[index - 1U] + 1.0F;
        if (start >= 1.0F) {
            // A key stored exactly at 1 starts its arc at phase 0.
            start -= 1.0F;
            end -= 1.0F;
        }
        const float difference = std::abs(start - hint);
        const float distance =
            std::min(difference, std::abs(1.0F - difference));
        if (!best.has_value() ||
            distance < bestDistance - kWaterFeatureClipPositionTolerance) {
            best = {start, end};
            bestDistance = distance;
        }
    }
    return best.value();
}

WaterFeatureSettingsClip SanitizeWaterFeatureSettingsClip(
    WaterFeatureSettingsClip clip) {
    if (clip.name.empty()) {
        clip.name = "Clip";
    }
    // W1: start lives in [0,1); end in (start, start+1]. A reversed pair is
    // still the legacy hand-edit repair (swap), unambiguous because wrapping
    // is encoded by end > 1, never by start > end.
    float start = SeepageFiniteOr(clip.start, 0.0F);
    float end = SeepageFiniteOr(clip.end, start + 1.0F);
    if (end < start) {
        std::swap(start, end);
    }
    if (end <= 1.0F && start >= 0.0F) {
        // Input already on the 0..1 rail is the pre-W1 shape and keeps the
        // pre-W1 repair exactly: a degenerate span at the rail end is
        // pushed back below 1 rather than grown across the seam, so a
        // document whose clips never wrap sanitizes bit-identically and
        // never acquires a wrapped sliver.
        if (end - start < kWaterFeatureClipMinimumLength) {
            end = std::min(1.0F, start + kWaterFeatureClipMinimumLength);
            start = std::max(0.0F, end - kWaterFeatureClipMinimumLength);
        }
        clip.start = start;
        clip.end = end;
        return clip;
    }
    // A span past the rail end (or with a start outside [0,1)) is wrapped
    // input: the minimum length grows the end only, since pulling the start
    // back would change the clip's phase for no reason and the end is free
    // to exceed 1.
    const float length = std::min(1.0F, end - start);
    if (start < 0.0F || start >= 1.0F) {
        start = WrapWaterClipPhase(start);
    }
    clip.start = start;
    clip.end = start + std::max(length, kWaterFeatureClipMinimumLength);
    return clip;
}

std::uint32_t AllocateWaterFeatureClipId(
    const WaterFeatureTimeline& timeline) {
    std::uint32_t next = 1U;
    for (const auto& clip : timeline.clips) {
        next = std::max(next, clip.id + 1U);
    }
    return next;
}

WaterFeatureSettingsClip* FindWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId) {
    if (timeline == nullptr) {
        return nullptr;
    }
    for (auto& clip : timeline->clips) {
        if (clip.id == clipId) {
            return &clip;
        }
    }
    return nullptr;
}

const WaterFeatureSettingsClip* FindWaterFeatureClip(
    const WaterFeatureTimeline* timeline,
    std::uint32_t clipId) {
    return FindWaterFeatureClip(
        const_cast<WaterFeatureTimeline*>(timeline),
        clipId);
}

bool SynchronizeWaterFeatureClipBounds(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId) {
    if (timeline == nullptr || clipId == 0U) {
        return false;
    }
    auto* clip = FindWaterFeatureClip(timeline, clipId);
    if (clip == nullptr) {
        return false;
    }
    std::vector<float> positions;
    for (const auto& setting : timeline->settings) {
        for (const auto& key : setting.keys) {
            if (key.clipId != clipId || !std::isfinite(key.position)) {
                continue;
            }
            positions.push_back(Clamp01(key.position));
        }
    }
    if (positions.empty()) {
        return false;
    }
    // Bounds are the hull of the members on the clip's own coordinate line.
    // An unwrapped clip keeps the exact pre-W1 linear min/max: plain key
    // drags, reorders, and deletes in the unlinked view must never make a
    // clip wrap, which the nearest-start covering arc did whenever the
    // member nearest the old start was no longer the smallest key (drag
    // 0.2 -> 0.7 inside {0.2,0.8} gave {0.7,1.7}). Only a clip that already
    // wraps is measured cyclically: its members unwrap relative to the
    // member cyclically nearest the clip's current start (keys ahead of
    // that anchor gain a cycle), so it stays wrapped while its keys move
    // and unwraps naturally once every member sits on one side of phase 0.
    // Anchoring at the stale start itself is wrong as soon as the head key
    // moves left of it (0.9 -> 0.85 in {0.9,1.2} keyed 0.9/0.2): the head
    // would gain a cycle and the clip flip to the complementary arc
    // {0.2,0.85}. Transforms write the destination span before calling
    // this, so a drag across the seam is already wrapped here and a drag
    // back is already unwrapped.
    const bool wasWrapped = WaterClipIsWrapped(clip->start, clip->end);
    const float oldStart = clip->start;
    const float oldEnd = clip->end;
    std::ranges::sort(positions);
    std::pair<float, float> arc;
    if (wasWrapped) {
        const float hint = WrapWaterClipPhase(oldStart);
        float anchor = positions.front();
        float anchorDistance = std::numeric_limits<float>::infinity();
        for (const float position : positions) {
            const float difference = std::abs(position - hint);
            const float distance =
                std::min(difference, std::abs(1.0F - difference));
            if (distance < anchorDistance -
                               kWaterFeatureClipPositionTolerance) {
                anchor = position;
                anchorDistance = distance;
            }
        }
        // A member stored exactly at 1 is phase 0 as an anchor; measuring
        // from 0 keeps every other member linear so the result is the
        // self-consistent linear hull rather than an arc starting at 1.
        if (anchor >= 1.0F) {
            anchor = 0.0F;
        }
        arc = {std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity()};
        for (const float position : positions) {
            const float unwrapped =
                UnwrapWaterClipPosition(position, anchor);
            arc.first = std::min(arc.first, unwrapped);
            arc.second = std::max(arc.second, unwrapped);
        }
        // The anchor is its own unwrapped minimum, so the start is already
        // in [0,1); a start within tolerance below 1 is kept rather than
        // wrapped to 0 so the anchoring key stays a member of the result.
        if (arc.first >= 1.0F) {
            arc.first -= 1.0F;
            arc.second -= 1.0F;
        }
    } else {
        // A clip that just unwrapped onto [start, 1] by a member reaching
        // phase 0 (tail key dragged to the seam, stored at 0) would read
        // that member as its smallest key here and flip to {0, start} on
        // the next pass (and on every document load). Phase 0 and phase 1
        // are one time on the loop, so while the clip ends on 1 and starts
        // above 0 a member at 0 measures as 1 -- unless a member already
        // sits on 1, in which case the key at 0 is a real head key (the
        // unlinked drag of a clip's first key to the rail's left edge) and
        // the linear hull must stay {0,1}. Unlinked documents cannot hold
        // the re-measured state otherwise: an unwrapped clip ending on 1
        // always has a member there, and pre-W1 loads flattened anything
        // else to the linear hull, so ordinary unlinked bounds are untouched.
        const bool memberOnPhaseOne = std::ranges::any_of(
            positions,
            [](float position) {
                return position >=
                       1.0F - kWaterFeatureClipPositionTolerance;
            });
        const bool endsOnPhaseOne =
            !memberOnPhaseOne &&
            std::abs(oldEnd - 1.0F) <= kWaterFeatureClipPositionTolerance &&
            oldStart > kWaterFeatureClipPositionTolerance;
        arc = {std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity()};
        for (const float position : positions) {
            const float measured =
                endsOnPhaseOne &&
                        position <= kWaterFeatureClipPositionTolerance
                    ? 1.0F
                    : position;
            arc.first = std::min(arc.first, measured);
            arc.second = std::max(arc.second, measured);
        }
    }
    if (arc.second - arc.first >= kWaterFeatureClipMinimumLength) {
        clip->start = arc.first;
        clip->end = arc.second;
    } else if (wasWrapped) {
        // One authored time still needs a small hit target centred on the
        // key. A clip that was wrapped may keep straddling phase 0 (W1).
        clip->start = WrapWaterClipPhase(
            arc.first - 0.5F * kWaterFeatureClipMinimumLength);
        clip->end = clip->start + kWaterFeatureClipMinimumLength;
    } else {
        // Unwrapped clips keep the pre-W1 marker clamped inside 0..1, so a
        // key at time 0 in an unlinked document never becomes a sliver
        // drawn at the far end of the rail.
        clip->start = std::max(
            0.0F,
            arc.first - 0.5F * kWaterFeatureClipMinimumLength);
        clip->end = std::min(
            1.0F,
            clip->start + kWaterFeatureClipMinimumLength);
        clip->start = std::max(
            0.0F,
            clip->end - kWaterFeatureClipMinimumLength);
    }
    SortWaterFeatureClips(&timeline->clips);
    return true;
}

std::optional<std::pair<float, float>> WaterFeatureLooseKeySpan(
    const WaterFeatureTimeline& timeline) {
    std::optional<std::pair<float, float>> span;
    for (const auto& setting : timeline.settings) {
        for (const auto& key : setting.keys) {
            if (!std::isfinite(key.position) || key.clipId != 0U) {
                continue;
            }
            const float position = Clamp01(key.position);
            if (!span.has_value()) {
                span = {position, position};
            } else {
                span->first = std::min(span->first, position);
                span->second = std::max(span->second, position);
            }
        }
    }
    return span;
}

WaterFeatureSpanLimits WaterFeatureTimelineSpanLimits(
    const WaterFeatureTimeline&,
    float rangeStart,
    float rangeEnd,
    bool cyclic) {
    WaterFeatureSpanLimits limits{};
    if (!WaterClipRangeIsValid(rangeStart, rangeEnd)) {
        return limits;
    }
    if (cyclic) {
        // The linked loop has no rail ends: a span may roll through phase 0
        // indefinitely. Callers bound stretches by end - start <= 1 instead.
        limits.minimumStart = -std::numeric_limits<float>::infinity();
        limits.maximumEnd = std::numeric_limits<float>::infinity();
    }
    return limits;
}

bool TransformWaterFeatureClipSelection(
    WaterFeatureTimeline* timeline,
    std::span<const std::uint32_t> clipIds,
    float rangeStart,
    float rangeEnd,
    float newStart,
    float newEnd,
    bool allowWrap) {
    if (timeline == nullptr || clipIds.empty() ||
        !WaterClipRangeIsValid(rangeStart, rangeEnd) ||
        !std::isfinite(newStart) || !std::isfinite(newEnd) ||
        newEnd - newStart < kWaterClipKeyTolerance ||
        newEnd - newStart > 1.0F + kWaterClipKeyTolerance) {
        return false;
    }
    if (!allowWrap &&
        (newStart < -kWaterClipKeyTolerance ||
         newEnd > 1.0F + kWaterClipKeyTolerance)) {
        // Unlinked editing never wraps: the destination must stay on the
        // 0..1 rail exactly as before W1.
        return false;
    }
    EnsureWaterFeatureExplicitClipMembership(timeline);
    if (allowWrap) {
        // Express the destination with its start in [0,1); the end may then
        // exceed 1, which is the wrapped clip representation.
        const float length = newEnd - newStart;
        newStart = WrapWaterClipPhase(newStart);
        newEnd = newStart + length;
    } else {
        newStart = Clamp01(newStart);
        newEnd = Clamp01(newEnd);
    }
    // Key landings are compared on the loop when wrapping is allowed, so a
    // key arriving at phase 1 collides with one sitting at 0.
    const auto landsOn = [&](float left, float right) {
        return allowWrap
            ? CyclicWaterClipDistance(left, right) <= kWaterClipKeyTolerance
            : std::abs(left - right) <= kWaterClipKeyTolerance;
    };
    const auto keyMovesWithSelection = [&](const WaterSettingKey& key) {
        if (key.clipId != 0U) {
            return WaterClipIdIsSelected(clipIds, key.clipId);
        }
        return WaterClipIdIsSelected(clipIds, 0U) &&
               WaterClipSpanContains(
                   rangeStart,
                   rangeEnd,
                   key.position);
    };
    bool holdsContent = std::any_of(
        timeline->clips.begin(),
        timeline->clips.end(),
        [&](const WaterFeatureSettingsClip& clip) {
            return WaterClipIdIsSelected(clipIds, clip.id);
        });
    for (const auto& setting : timeline->settings) {
        if (holdsContent) {
            break;
        }
        holdsContent = std::any_of(
            setting.keys.begin(),
            setting.keys.end(),
            [&](const WaterSettingKey& key) {
                return keyMovesWithSelection(key);
            });
    }
    if (holdsContent &&
        newEnd - newStart <
            kWaterFeatureClipMinimumLength - kWaterClipKeyTolerance) {
        return false;
    }
    // Two selected keys on one loop phase (the usual 0 and 1 of a clip
    // authored on the plain rail, or a legacy full-rail clip) are a single
    // cyclic instant: every destination maps them onto the same wrapped
    // time, so the collision check below would refuse every move of such a
    // clip and the drag would look stuck. Cyclic evaluation already wraps 1
    // onto 0 and keeps the linear-later key of a coincident pair, so the
    // same twin is coalesced away here (dropped keys are only erased once
    // the transform is known to succeed).
    std::vector<std::vector<bool>> dropped(timeline->settings.size());
    if (allowWrap) {
        for (std::size_t settingIndex = 0U;
             settingIndex < timeline->settings.size();
             ++settingIndex) {
            const auto& keys = timeline->settings[settingIndex].keys;
            auto& drop = dropped[settingIndex];
            drop.assign(keys.size(), false);
            for (std::size_t left = 0U; left < keys.size(); ++left) {
                if (!keyMovesWithSelection(keys[left])) {
                    continue;
                }
                for (std::size_t right = left + 1U;
                     right < keys.size();
                     ++right) {
                    if (!keyMovesWithSelection(keys[right]) ||
                        CyclicWaterClipDistance(
                            keys[left].position,
                            keys[right].position) >
                            kWaterClipKeyTolerance) {
                        continue;
                    }
                    const bool leftEarlier =
                        keys[left].position <= keys[right].position;
                    drop[leftEarlier ? left : right] = true;
                }
            }
        }
    }
    const auto isDropped = [&](std::size_t settingIndex, std::size_t keyIndex) {
        return keyIndex < dropped[settingIndex].size() &&
               dropped[settingIndex][keyIndex];
    };
    // Collision check before any mutation: a remapped key must not land on
    // a key that stays outside the selection on the same track, nor collapse
    // two selected keys to an ambiguous time.
    for (std::size_t settingIndex = 0U;
         settingIndex < timeline->settings.size();
         ++settingIndex) {
        const auto& setting = timeline->settings[settingIndex];
        std::vector<float> remappedPositions;
        for (std::size_t keyIndex = 0U;
             keyIndex < setting.keys.size();
             ++keyIndex) {
            const auto& key = setting.keys[keyIndex];
            if (!keyMovesWithSelection(key) ||
                isDropped(settingIndex, keyIndex)) {
                continue;
            }
            const float remapped = RemapWaterClipPosition(
                key.position,
                rangeStart,
                rangeEnd,
                newStart,
                newEnd,
                allowWrap);
            for (const auto& other : setting.keys) {
                if (keyMovesWithSelection(other)) {
                    continue;
                }
                if (landsOn(other.position, remapped)) {
                    return false;
                }
            }
            if (std::any_of(
                    remappedPositions.begin(),
                    remappedPositions.end(),
                    [&](float other) {
                        return landsOn(other, remapped);
                    })) {
                return false;
            }
            remappedPositions.push_back(remapped);
        }
    }
    for (std::size_t settingIndex = 0U;
         settingIndex < timeline->settings.size();
         ++settingIndex) {
        auto& setting = timeline->settings[settingIndex];
        bool moved = false;
        std::vector<WaterSettingKey> kept;
        kept.reserve(setting.keys.size());
        for (std::size_t keyIndex = 0U;
             keyIndex < setting.keys.size();
             ++keyIndex) {
            auto& key = setting.keys[keyIndex];
            if (isDropped(settingIndex, keyIndex)) {
                moved = true;
                continue;
            }
            if (keyMovesWithSelection(key)) {
                key.position = RemapWaterClipPosition(
                    key.position,
                    rangeStart,
                    rangeEnd,
                    newStart,
                    newEnd,
                    allowWrap);
                moved = true;
            }
            kept.push_back(std::move(key));
        }
        setting.keys = std::move(kept);
        if (moved) {
            std::stable_sort(
                setting.keys.begin(),
                setting.keys.end(),
                [](const WaterSettingKey& left,
                   const WaterSettingKey& right) {
                    return left.position < right.position;
                });
        }
    }
    for (auto& clip : timeline->clips) {
        if (!WaterClipIdIsSelected(clipIds, clip.id)) {
            continue;
        }
        // Remap the span on the unwrapped line so a wrapped destination
        // keeps end > 1; Synchronize then re-derives it from the members
        // with this start as the covering-arc hint.
        const float mappedStart = RemapWaterClipPositionUnwrapped(
            clip.start,
            rangeStart,
            rangeEnd,
            newStart,
            newEnd);
        const float mappedEnd = RemapWaterClipPositionUnwrapped(
            clip.end,
            rangeStart,
            rangeEnd,
            newStart,
            newEnd);
        if (allowWrap) {
            clip.start = WrapWaterClipPhase(mappedStart);
            clip.end = clip.start + std::min(1.0F, mappedEnd - mappedStart);
        } else {
            clip.start = Clamp01(mappedStart);
            clip.end = Clamp01(mappedEnd);
        }
    }
    for (const auto clipId : clipIds) {
        if (clipId != 0U) {
            (void)SynchronizeWaterFeatureClipBounds(timeline, clipId);
        }
    }
    SortWaterFeatureClips(&timeline->clips);
    return true;
}

std::pair<float, float> CyclicWaterFeatureClipMoveSpan(
    float rangeStart,
    float rangeEnd,
    float delta) {
    const auto ordered = WaterFeatureClipDisplaySpan(
        rangeStart,
        rangeEnd);
    if (!std::isfinite(delta)) {
        return ordered;
    }
    const float length = ordered.second - ordered.first;
    // Period-1 roll (W1): the start simply travels around the loop and the
    // span keeps its length, so a clip straddling phase 0 is returned as a
    // wrapped span (end > 1) instead of jumping by its own length. A start
    // landing exactly on 1 is the same phase as 0 and is stored as 0.
    const float movedStart = WrapWaterClipPhase(ordered.first + delta);
    return {movedStart, movedStart + length};
}

bool TransformWaterFeatureTimelineSpan(
    WaterFeatureTimeline* timeline,
    float rangeStart,
    float rangeEnd,
    float newStart,
    float newEnd) {
    if (timeline == nullptr ||
        !WaterClipRangeIsValid(rangeStart, rangeEnd)) {
        return false;
    }
    EnsureWaterFeatureExplicitClipMembership(timeline);
    std::vector<std::uint32_t> selected;
    for (const auto& clip : timeline->clips) {
        if (WaterClipSpanContains(rangeStart, rangeEnd, clip.start) &&
            WaterClipSpanContains(rangeStart, rangeEnd, clip.end)) {
            selected.push_back(clip.id);
        }
    }
    const bool holdsLooseKeys = std::any_of(
        timeline->settings.begin(),
        timeline->settings.end(),
        [&](const WaterKeyedSettingTrack& setting) {
            return std::any_of(
                setting.keys.begin(),
                setting.keys.end(),
                [&](const WaterSettingKey& key) {
                    return key.clipId == 0U &&
                           WaterClipSpanContains(
                               rangeStart,
                               rangeEnd,
                               key.position);
                });
        });
    if (holdsLooseKeys) {
        selected.push_back(0U);
    }
    return TransformWaterFeatureClipSelection(
        timeline,
        selected,
        rangeStart,
        rangeEnd,
        newStart,
        newEnd);
}

bool TransformWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId,
    float newStart,
    float newEnd,
    bool allowWrap) {
    if (timeline == nullptr || clipId == 0U) {
        return false;
    }
    EnsureWaterFeatureExplicitClipMembership(timeline);
    const auto* clip = FindWaterFeatureClip(timeline, clipId);
    if (clip == nullptr) {
        return false;
    }
    const float rangeStart = clip->start;
    const float rangeEnd = clip->end;
    const std::array ids{clipId};
    return TransformWaterFeatureClipSelection(
        timeline,
        ids,
        rangeStart,
        rangeEnd,
        newStart,
        newEnd,
        allowWrap);
}

WaterKeyedSettingsProfile CaptureWaterKeyedSettingsClip(
    const WaterFeatureTimeline& timeline,
    float rangeStart,
    float rangeEnd) {
    WaterKeyedSettingsProfile profile;
    profile.featureKind = timeline.feature.kind;
    profile.ownerObjectId = timeline.feature.objectId;
    if (!WaterClipRangeIsValid(rangeStart, rangeEnd)) {
        return profile;
    }
    profile.nativeLengthFraction = std::clamp(
        rangeEnd - rangeStart,
        kWaterFeatureClipMinimumLength,
        1.0F);
    for (const auto& setting : timeline.settings) {
        WaterKeyedSettingTrack captured;
        captured.settingId = setting.settingId;
        captured.active = setting.active;
        captured.label = setting.label;
        captured.profileGroup = setting.profileGroup;
        captured.profileName = setting.profileName;
        captured.defaultInterpolation = setting.defaultInterpolation;
        for (const auto& key : setting.keys) {
            if (!WaterClipSpanContains(
                    rangeStart,
                    rangeEnd,
                    key.position) ||
                (timeline.clipMembershipExplicit && key.clipId != 0U)) {
                continue;
            }
            auto capturedKey = key;
            capturedKey.position = RemapWaterClipPosition(
                key.position,
                rangeStart,
                rangeEnd,
                0.0F,
                1.0F);
            capturedKey.clipId = 0U;
            captured.keys.push_back(capturedKey);
        }
        if (!captured.keys.empty()) {
            // Members of a wrapped span are stored in loop order, not span
            // order; packages always carry their keys sorted.
            std::stable_sort(
                captured.keys.begin(),
                captured.keys.end(),
                [](const WaterSettingKey& left, const WaterSettingKey& right) {
                    return left.position < right.position;
                });
            profile.settings.push_back(std::move(captured));
        }
    }
    return profile;
}

WaterKeyedSettingsProfile CaptureWaterKeyedSettingsClipById(
    const WaterFeatureTimeline& timeline,
    std::uint32_t clipId) {
    WaterKeyedSettingsProfile profile;
    profile.featureKind = timeline.feature.kind;
    profile.ownerObjectId = timeline.feature.objectId;
    const auto* clip = FindWaterFeatureClip(&timeline, clipId);
    if (clip == nullptr || !WaterClipRangeIsValid(clip->start, clip->end)) {
        return profile;
    }
    profile.nativeLengthFraction = std::clamp(
        clip->end - clip->start,
        kWaterFeatureClipMinimumLength,
        1.0F);
    for (const auto& setting : timeline.settings) {
        WaterKeyedSettingTrack captured;
        captured.settingId = setting.settingId;
        captured.active = setting.active;
        captured.label = setting.label;
        captured.profileGroup = setting.profileGroup;
        captured.profileName = setting.profileName;
        captured.defaultInterpolation = setting.defaultInterpolation;
        for (const auto& key : setting.keys) {
            if (key.clipId != clipId) {
                continue;
            }
            auto capturedKey = key;
            capturedKey.position = RemapWaterClipPosition(
                key.position,
                clip->start,
                clip->end,
                0.0F,
                1.0F);
            capturedKey.clipId = 0U;
            captured.keys.push_back(capturedKey);
        }
        if (!captured.keys.empty()) {
            // Members of a wrapped span are stored in loop order, not span
            // order; packages always carry their keys sorted.
            std::stable_sort(
                captured.keys.begin(),
                captured.keys.end(),
                [](const WaterSettingKey& left, const WaterSettingKey& right) {
                    return left.position < right.position;
                });
            profile.settings.push_back(std::move(captured));
        }
    }
    return profile;
}

std::optional<std::uint32_t> ApplyWaterKeyedSettingsClip(
    WaterFeatureTimeline* timeline,
    const WaterKeyedSettingsProfile& profile,
    float windowStart,
    float windowEnd,
    std::string clipName) {
    if (timeline == nullptr ||
        !WaterClipRangeIsValid(windowStart, windowEnd) ||
        profile.settings.empty()) {
        return std::nullopt;
    }
    {
        // The window may wrap (W1): keep its length, store the start phase.
        const float length = std::min(1.0F, windowEnd - windowStart);
        windowStart = WrapWaterClipPhase(windowStart);
        windowEnd = windowStart + length;
    }
    if (windowEnd - windowStart <
        kWaterFeatureClipMinimumLength - kWaterClipKeyTolerance) {
        return std::nullopt;
    }
    // Stage the complete operation away from the live timeline. Applying a
    // package may need to nudge several keys around existing same-track keys;
    // if even one applicable key has no collision-free position, none of the
    // membership migration, new tracks, keys, or clip metadata may leak out.
    WaterFeatureTimeline candidate = *timeline;
    EnsureWaterFeatureExplicitClipMembership(&candidate);
    const bool crossKind = profile.featureKind != timeline->feature.kind;
    const bool hasApplicableKeys = std::any_of(
        profile.settings.begin(),
        profile.settings.end(),
        [&](const WaterKeyedSettingTrack& packaged) {
            return !packaged.settingId.empty() &&
                   !packaged.keys.empty() &&
                   (!crossKind ||
                    FindWaterKeyableSetting(
                        timeline->feature.kind,
                        packaged.settingId) != nullptr);
        });
    if (!hasApplicableKeys) {
        return std::nullopt;
    }

    WaterFeatureSettingsClip clip{
        .id = AllocateWaterFeatureClipId(candidate),
        .name = !clipName.empty()
                    ? std::move(clipName)
                    : (!profile.baseProfileName.empty()
                           ? profile.baseProfileName
                           : profile.name),
        .start = windowStart,
        .end = windowEnd,
        .sourceProfileName = profile.name,
    };
    const std::uint32_t clipId = clip.id;
    candidate.clips.push_back(
        SanitizeWaterFeatureSettingsClip(std::move(clip)));

    for (const auto& packaged : profile.settings) {
        if (packaged.settingId.empty() || packaged.keys.empty()) {
            continue;
        }
        // Cross-kind packages only apply where the setting id is a known
        // registry setting of the target feature; same-kind packages carry
        // their dynamic profile tracks along verbatim.
        if (crossKind &&
            FindWaterKeyableSetting(
                timeline->feature.kind,
                packaged.settingId) == nullptr) {
            continue;
        }
        auto* track = [&]() -> WaterKeyedSettingTrack* {
            for (auto& setting : candidate.settings) {
                if (setting.settingId == packaged.settingId) {
                    return &setting;
                }
            }
            candidate.settings.push_back(WaterKeyedSettingTrack{
                .settingId = packaged.settingId,
                .active = packaged.active,
                .label = packaged.label,
                .profileGroup = packaged.profileGroup,
                .profileName = packaged.profileName,
                .defaultInterpolation = packaged.defaultInterpolation,
                .keys = {},
            });
            return &candidate.settings.back();
        }();
        for (const auto& key : packaged.keys) {
            const float desiredPosition = RemapWaterClipPositionUnwrapped(
                Clamp01(key.position),
                0.0F,
                1.0F,
                windowStart,
                windowEnd);
            const auto position = AvailableWaterClipKeyPosition(
                *track,
                desiredPosition,
                windowStart,
                windowEnd);
            if (!position.has_value()) {
                return std::nullopt;
            }
            auto appliedKey = key;
            // Available positions are searched on the unwrapped window;
            // keys are stored in [0,1). Only a window that wraps can yield
            // a position >= 1; an unwrapped window's end at exactly 1 stays
            // a stored key at 1 as before.
            appliedKey.position =
                WaterClipIsWrapped(windowStart, windowEnd)
                    ? WrapWaterClipPhase(position.value())
                    : position.value();
            appliedKey.clipId = clipId;
            if (appliedKey.interpolation ==
                WaterScenarioInterpolation::TrackDefault &&
                track->defaultInterpolation !=
                    packaged.defaultInterpolation) {
                // A package carries its own track default. Materialize it on
                // copied keys rather than restyling unrelated keys already
                // present on the destination track.
                appliedKey.interpolation =
                    packaged.defaultInterpolation ==
                            WaterScenarioInterpolation::TrackDefault
                        ? WaterScenarioInterpolation::SmoothVelocity
                        : packaged.defaultInterpolation;
            }
            track->keys.push_back(appliedKey);
        }
        std::stable_sort(
            track->keys.begin(),
            track->keys.end(),
            [](const WaterSettingKey& left, const WaterSettingKey& right) {
                return left.position < right.position;
            });
    }
    (void)SynchronizeWaterFeatureClipBounds(&candidate, clipId);
    SortWaterFeatureClips(&candidate.clips);
    *timeline = std::move(candidate);
    return clipId;
}

std::optional<std::uint32_t> CreateWaterFeatureClipFromSpan(
    WaterFeatureTimeline* timeline,
    float start,
    float end,
    std::string name,
    bool attachLooseKeys) {
    if (timeline == nullptr || !WaterClipRangeIsValid(start, end)) {
        return std::nullopt;
    }
    EnsureWaterFeatureExplicitClipMembership(timeline);
    WaterFeatureSettingsClip clip = SanitizeWaterFeatureSettingsClip({
        .id = AllocateWaterFeatureClipId(*timeline),
        .name = std::move(name),
        .start = start,
        .end = end,
        .sourceProfileName = {},
    });
    const std::uint32_t clipId = clip.id;
    const float clipStart = clip.start;
    const float clipEnd = clip.end;
    timeline->clips.push_back(std::move(clip));
    if (attachLooseKeys) {
        for (auto& setting : timeline->settings) {
            for (auto& key : setting.keys) {
                if (key.clipId == 0U &&
                    WaterClipSpanContains(clipStart, clipEnd, key.position)) {
                    key.clipId = clipId;
                }
            }
        }
    }
    (void)SynchronizeWaterFeatureClipBounds(timeline, clipId);
    SortWaterFeatureClips(&timeline->clips);
    return clipId;
}

std::optional<std::uint32_t> DuplicateWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId,
    float targetStart,
    bool allowWrap) {
    EnsureWaterFeatureExplicitClipMembership(timeline);
    const auto* clip = FindWaterFeatureClip(timeline, clipId);
    if (clip == nullptr || !std::isfinite(targetStart)) {
        return std::nullopt;
    }
    const float length = clip->end - clip->start;
    const std::string name = clip->name;
    const std::string provenance = clip->sourceProfileName;
    // Unlinked copies stay on the rail; linked-cyclic copies roll around it.
    const float start = allowWrap
        ? WrapWaterClipPhase(targetStart)
        : std::clamp(targetStart, 0.0F, 1.0F - length);
    auto package = CaptureWaterKeyedSettingsClipById(*timeline, clipId);
    package.name = provenance;
    package.baseProfileName = name;
    if (package.settings.empty()) {
        // A clip without keys still duplicates as an empty span marker.
        return CreateWaterFeatureClipFromSpan(
            timeline,
            start,
            start + length,
            name,
            /*attachLooseKeys=*/false);
    }
    return ApplyWaterKeyedSettingsClip(
        timeline,
        package,
        start,
        start + length,
        name);
}

std::optional<std::uint32_t> TransferWaterFeatureClip(
    WaterFeatureTimeline* source,
    std::uint32_t clipId,
    WaterFeatureTimeline* destination,
    bool removeFromSource) {
    EnsureWaterFeatureExplicitClipMembership(source);
    EnsureWaterFeatureExplicitClipMembership(destination);
    const auto* clip = FindWaterFeatureClip(source, clipId);
    if (clip == nullptr || destination == nullptr ||
        source == destination ||
        source->feature.kind != destination->feature.kind) {
        return std::nullopt;
    }
    const float start = clip->start;
    const float end = clip->end;
    const std::string name = clip->name;
    const std::string provenance = clip->sourceProfileName;
    auto package = CaptureWaterKeyedSettingsClipById(*source, clipId);
    package.name = provenance;
    package.baseProfileName = name;
    std::optional<std::uint32_t> destinationClipId;
    if (package.settings.empty()) {
        destinationClipId = CreateWaterFeatureClipFromSpan(
            destination,
            start,
            end,
            name,
            /*attachLooseKeys=*/false);
    } else {
        destinationClipId = ApplyWaterKeyedSettingsClip(
            destination,
            package,
            start,
            end,
            name);
    }
    if (destinationClipId.has_value() && removeFromSource) {
        (void)RemoveWaterFeatureClip(source, clipId, /*removeKeys=*/true);
    }
    return destinationClipId;
}

bool RemoveWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId,
    bool removeKeys) {
    EnsureWaterFeatureExplicitClipMembership(timeline);
    auto* clip = FindWaterFeatureClip(timeline, clipId);
    if (clip == nullptr) {
        return false;
    }
    for (auto& setting : timeline->settings) {
        if (removeKeys) {
            std::erase_if(
                setting.keys,
                [&](const WaterSettingKey& key) {
                    return key.clipId == clipId;
                });
        } else {
            for (auto& key : setting.keys) {
                if (key.clipId == clipId) {
                    key.clipId = 0U;
                }
            }
        }
    }
    std::erase_if(
        timeline->clips,
        [&](const WaterFeatureSettingsClip& candidate) {
            return candidate.id == clipId;
        });
    return true;
}

const float* WaterFeatureTimingOverlay::Find(
    const WaterKeyedFeatureId& feature,
    std::string_view settingId) const {
    for (const auto& sample : samples) {
        if (sample.feature == feature && sample.settingId == settingId) {
            return &sample.value;
        }
    }
    return nullptr;
}

bool WaterFeatureTimingOverlay::Allows(
    const WaterKeyedFeatureId& feature) const {
    if (!onlyShowRunFeatures) {
        return true;
    }
    const auto hasKind = [&](WaterKeyedFeatureKind kind) {
        return std::any_of(
            assignedRunFeatures.begin(),
            assignedRunFeatures.end(),
            [&](const WaterKeyedFeatureId& assigned) {
                return assigned.kind == kind;
            });
    };
    if (std::find(
            assignedRunFeatures.begin(),
            assignedRunFeatures.end(),
            feature) != assignedRunFeatures.end()) {
        return true;
    }
    switch (feature.kind) {
        case WaterKeyedFeatureKind::Shoreline:
            return hasKind(WaterKeyedFeatureKind::ShorelineInstance);
        case WaterKeyedFeatureKind::ShorelineInstance:
            return hasKind(WaterKeyedFeatureKind::Shoreline);
        case WaterKeyedFeatureKind::SeepageGlobal:
            return hasKind(WaterKeyedFeatureKind::SeepageNode);
        case WaterKeyedFeatureKind::SeepageNode:
            return hasKind(WaterKeyedFeatureKind::SeepageGlobal);
        case WaterKeyedFeatureKind::FlowGlobal:
            return hasKind(WaterKeyedFeatureKind::FlowSource) ||
                   hasKind(WaterKeyedFeatureKind::FlowPath);
        case WaterKeyedFeatureKind::FlowSource:
        case WaterKeyedFeatureKind::FlowPath:
            return hasKind(WaterKeyedFeatureKind::FlowGlobal);
        case WaterKeyedFeatureKind::Rain:
        case WaterKeyedFeatureKind::MeshFlow:
            return false;
    }
    return false;
}

WaterFeatureTimingOverlay BuildWaterFeatureTimingOverlay(
    std::span<const WaterFeatureTimingRun> runs,
    float normalizedPosition,
    bool cyclic,
    bool onlyShowRunFeatures) {
    WaterFeatureTimingOverlay overlay;
    overlay.onlyShowRunFeatures = onlyShowRunFeatures;
    for (const auto& run : runs) {
        for (const auto& timeline : run.features) {
            if (std::find(
                    overlay.assignedRunFeatures.begin(),
                    overlay.assignedRunFeatures.end(),
                    timeline.feature) ==
                overlay.assignedRunFeatures.end()) {
                overlay.assignedRunFeatures.push_back(timeline.feature);
            }
        }
        if (!run.enabled) {
            continue;
        }
        for (const auto& timeline : run.features) {
            for (const auto& setting : timeline.settings) {
                const auto value = cyclic
                    ? EvaluateWaterKeyedSettingTrackCyclic(
                          setting,
                          normalizedPosition)
                    : EvaluateWaterKeyedSettingTrack(
                          setting,
                          normalizedPosition);
                if (!value.has_value()) {
                    continue;
                }
                overlay.samples.push_back({
                    .feature = timeline.feature,
                    .settingId = setting.settingId,
                    .value = *value,
                });
            }
        }
    }
    return overlay;
}

void ApplyWaterFeatureTimingOverlayToScenario(
    const WaterFeatureTimingOverlay& overlay,
    WaterScenarioState* state) {
    if (state == nullptr) {
        return;
    }
    const auto applyLevel = [&](WaterKeyedFeatureKind kind, float* channel) {
        const auto* value = overlay.Find({.kind = kind}, "level");
        if (value != nullptr) {
            *channel = Clamp01(SeepageFiniteOr(*value, 0.0F));
        }
    };
    applyLevel(WaterKeyedFeatureKind::Rain, &state->rainLevel);
    applyLevel(WaterKeyedFeatureKind::MeshFlow, &state->meshFlowLevel);
    applyLevel(WaterKeyedFeatureKind::Shoreline, &state->shorelineLevel);
}

WaterRainEnvelopeDomain MakeWaterRainEnvelopeDomain(
    float durationSeconds,
    float sampleRateHz,
    std::size_t maxSamples) {
    WaterRainEnvelopeDomain domain;
    domain.durationSeconds = std::clamp(
        SeepageFiniteOr(durationSeconds, 0.0F),
        0.0F,
        86'400.0F);
    const float requestedRate = std::clamp(
        SeepageFiniteOr(
            sampleRateHz,
            kWaterRainEnvelopeSampleRateHz),
        1.0F,
        10'000.0F);
    const std::size_t safeMaximumSamples =
        std::max<std::size_t>(1U, maxSamples);
    const double requestedIntervals = std::ceil(
        static_cast<double>(domain.durationSeconds) *
        static_cast<double>(requestedRate));
    const auto intervalCount = static_cast<std::size_t>(std::min(
        requestedIntervals,
        static_cast<double>(safeMaximumSamples - 1U)));
    domain.sampleCount = intervalCount + 1U;
    domain.stepSeconds =
        intervalCount > 0U
            ? domain.durationSeconds /
                  static_cast<float>(intervalCount)
            : 0.0F;
    domain.sampleRateHz =
        domain.stepSeconds > 0.0F
            ? 1.0F / domain.stepSeconds
            : requestedRate;
    return domain;
}

WaterRainResponseSettings SanitizeWaterRainResponseSettings(
    WaterRainResponseSettings settings) {
    settings.delaySeconds = SanitizeSeepageTimingSeconds(
        settings.delaySeconds);
    settings.riseSeconds = SanitizeSeepageTimingSeconds(
        settings.riseSeconds);
    settings.recessionSeconds = SanitizeSeepageTimingSeconds(
        settings.recessionSeconds);
    return settings;
}

void ApplyWaterFeatureTimingOverlayToSeepageNode(
    const WaterFeatureTimingOverlay& overlay,
    WaterSeepageNode* node) {
    if (node == nullptr) {
        return;
    }
    const WaterKeyedFeatureId feature{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = node->id,
    };
    const auto applySeconds = [&](std::string_view settingId, float* target) {
        if (const auto* value = overlay.Find(feature, settingId);
            value != nullptr) {
            *target = SanitizeSeepageTimingSeconds(*value);
        }
    };
    applySeconds("rain_delay_seconds", &node->rainDelaySeconds);
    applySeconds("rain_rise_seconds", &node->rainRiseSeconds);
    applySeconds(
        "rain_recession_seconds",
        &node->rainRecessionSeconds);
}

WaterRainResponseSettings ResolveWaterSeepageNodeRainResponse(
    const WaterSeepageNode& node,
    const WaterFeatureTimingOverlay* overlay) {
    auto resolvedNode = node;
    if (overlay != nullptr) {
        ApplyWaterFeatureTimingOverlayToSeepageNode(
            *overlay,
            &resolvedNode);
    }
    return SanitizeWaterRainResponseSettings({
        .delaySeconds = resolvedNode.rainDelaySeconds,
        .riseSeconds = resolvedNode.rainRiseSeconds,
        .recessionSeconds = resolvedNode.rainRecessionSeconds,
    });
}

void ApplyWaterFeatureTimingOverlayToDynamicMeshFlowSettings(
    const WaterFeatureTimingOverlay& overlay,
    WaterDynamicMeshFlowSettings* settings) {
    if (settings == nullptr) {
        return;
    }
    const WaterKeyedFeatureId feature{
        .kind = WaterKeyedFeatureKind::MeshFlow,
    };
    const auto apply = [&](std::string_view settingId,
                           float minimum,
                           float maximum,
                           float* target) {
        if (const auto* value = overlay.Find(feature, settingId);
            value != nullptr) {
            *target = std::clamp(
                SeepageFiniteOr(*value, minimum),
                minimum,
                maximum);
        }
    };
    apply("level", 0.0F, 1.0F, &settings->activity);
    apply("rain_gain", 0.0F, 4.0F, &settings->rainGain);
    apply(
        "moisture_persistence",
        0.0F,
        8.0F,
        &settings->moisturePersistenceMultiplier);
    apply(
        "rain_rise_seconds",
        0.0F,
        86'400.0F,
        &settings->rainRiseSeconds);
    apply(
        "rain_recession_seconds",
        0.0F,
        86'400.0F,
        &settings->rainRecessionSeconds);
}

WaterRainResponseSettings ResolveWaterDynamicMeshFlowRainResponse(
    const WaterDynamicMeshFlowSettings& settings,
    const WaterFeatureTimingOverlay* overlay) {
    auto resolved = settings;
    if (overlay != nullptr) {
        ApplyWaterFeatureTimingOverlayToDynamicMeshFlowSettings(
            *overlay,
            &resolved);
    }
    resolved = SanitizeWaterDynamicMeshFlowSettings(std::move(resolved));
    return SanitizeWaterRainResponseSettings({
        .delaySeconds = 0.0F,
        .riseSeconds =
            resolved.rainRiseSeconds *
            resolved.moisturePersistenceMultiplier,
        .recessionSeconds =
            resolved.rainRecessionSeconds *
            resolved.moisturePersistenceMultiplier,
    });
}

float EffectiveWaterDynamicMeshFlowLevel(
    const WaterDynamicMeshFlowSettings& settings,
    float effectiveRainLevel,
    const WaterFeatureTimingOverlay* overlay) {
    if (overlay != nullptr &&
        !overlay->Allows({.kind = WaterKeyedFeatureKind::MeshFlow})) {
        return 0.0F;
    }
    auto resolved = settings;
    if (overlay != nullptr) {
        ApplyWaterFeatureTimingOverlayToDynamicMeshFlowSettings(
            *overlay,
            &resolved);
    }
    resolved = SanitizeWaterDynamicMeshFlowSettings(std::move(resolved));
    const float rain = Clamp01(
        SeepageFiniteOr(effectiveRainLevel, 0.0F));
    return Clamp01(
        resolved.activity +
        (1.0F - resolved.activity) * rain * resolved.rainGain);
}

float EffectiveWaterDynamicMeshPersistenceSeconds(
    float authoredPersistenceSeconds,
    const WaterDynamicMeshFlowSettings& settings,
    const WaterFeatureTimingOverlay* overlay) {
    auto resolved = settings;
    if (overlay != nullptr) {
        ApplyWaterFeatureTimingOverlayToDynamicMeshFlowSettings(
            *overlay,
            &resolved);
    }
    resolved = SanitizeWaterDynamicMeshFlowSettings(std::move(resolved));
    const float authored = std::clamp(
        SeepageFiniteOr(authoredPersistenceSeconds, 0.0F),
        0.0F,
        86'400.0F);
    return std::min(
        86'400.0F,
        authored * resolved.moisturePersistenceMultiplier);
}

namespace {

const WaterKeyedSettingTrack* FindAuthoredRainEnvelopeTrack(
    std::span<const WaterFeatureTimingRun> runs,
    const WaterKeyedFeatureId& feature,
    std::string_view settingId) {
    for (const auto& run : runs) {
        if (!run.enabled) {
            continue;
        }
        const auto* timeline = FindWaterFeatureTimeline(&run, feature);
        if (timeline == nullptr) {
            continue;
        }
        const auto setting = std::find_if(
            timeline->settings.begin(),
            timeline->settings.end(),
            [&](const WaterKeyedSettingTrack& candidate) {
                return candidate.settingId == settingId;
            });
        if (setting != timeline->settings.end()) {
            return &*setting;
        }
    }
    return nullptr;
}

float EvaluateAuthoredRainEnvelopeTrack(
    const WaterKeyedSettingTrack* track,
    float normalizedPosition,
    float fallback,
    float minimum,
    float maximum) {
    const auto value =
        track != nullptr
            ? EvaluateWaterKeyedSettingTrack(
                  *track,
                  normalizedPosition)
            : std::nullopt;
    return std::clamp(
        SeepageFiniteOr(value.value_or(fallback), fallback),
        minimum,
        maximum);
}

void FingerprintAuthoredRainEnvelopeTrack(
    std::uint64_t* hash,
    const WaterKeyedSettingTrack* track) {
    if (track == nullptr || !track->active || track->keys.empty()) {
        SeepageFingerprintU32(hash, 0U);
        return;
    }
    SeepageFingerprintU32(hash, 1U);
    const auto sanitized =
        SanitizeWaterKeyedSettingTrack(*track);
    SeepageFingerprintU32(
        hash,
        static_cast<std::uint32_t>(std::min<std::size_t>(
            sanitized.keys.size(),
            static_cast<std::size_t>(
                std::numeric_limits<std::uint32_t>::max()))));
    for (const auto& key : sanitized.keys) {
        SeepageFingerprintFloat(hash, key.position);
        SeepageFingerprintFloat(hash, key.value);
        SeepageFingerprintU32(
            hash,
            static_cast<std::uint32_t>(key.interpolation));
    }
}

template <typename Envelope, typename ResolveRain, typename ResolveResponse>
Envelope BuildAuthoredRainEnvelope(
    const WaterRainEnvelopeDomain& domain,
    ResolveRain&& resolveRain,
    ResolveResponse&& resolveResponse,
    std::string fingerprint) {
    Envelope envelope;
    envelope.sampleRateHz = domain.sampleRateHz;
    envelope.durationSeconds = domain.durationSeconds;
    envelope.samples.resize(domain.sampleCount, 0.0F);
    envelope.fingerprint = std::move(fingerprint);

    float filtered = 0.0F;
    for (std::size_t index = 0U;
         index < domain.sampleCount;
         ++index) {
        const float timeSeconds =
            domain.stepSeconds * static_cast<float>(index);
        const float normalizedPosition =
            domain.durationSeconds > 1.0e-6F
                ? timeSeconds / domain.durationSeconds
                : 0.0F;
        const auto response = SanitizeWaterRainResponseSettings(
            resolveResponse(normalizedPosition));
        const bool waitingForDelay =
            timeSeconds < response.delaySeconds;
        const float delayedTime = std::max(
            0.0F,
            timeSeconds - response.delaySeconds);
        const float delayedPosition =
            domain.durationSeconds > 1.0e-6F
                ? delayedTime / domain.durationSeconds
                : 0.0F;
        const float target =
            waitingForDelay
                ? 0.0F
                : Clamp01(resolveRain(delayedPosition));
        const float responseSeconds =
            target >= filtered
                ? response.riseSeconds
                : response.recessionSeconds;
        if (responseSeconds <= 1.0e-6F ||
            domain.stepSeconds <= 0.0F) {
            filtered = target;
        } else if (index == 0U) {
            filtered = 0.0F;
        } else {
            const float responseAmount =
                1.0F -
                std::exp(
                    -domain.stepSeconds /
                    responseSeconds);
            filtered = std::lerp(
                filtered,
                target,
                Clamp01(responseAmount));
        }
        envelope.samples[index] = Clamp01(filtered);
    }
    return envelope;
}

}  // namespace

std::string WaterSeepageNodeRainEnvelopeFingerprint(
    const WaterSeepageNode& node,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds) {
    const WaterKeyedFeatureId rainFeature{
        .kind = WaterKeyedFeatureKind::Rain,
    };
    const WaterKeyedFeatureId nodeFeature{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = node.id,
    };
    const auto* rainTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        rainFeature,
        "level");
    const auto* delayTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        nodeFeature,
        "rain_delay_seconds");
    const auto* riseTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        nodeFeature,
        "rain_rise_seconds");
    const auto* recessionTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        nodeFeature,
        "rain_recession_seconds");
    const auto response =
        ResolveWaterSeepageNodeRainResponse(node);

    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 1U);
    SeepageFingerprintFloat(
        &hash,
        Clamp01(SeepageFiniteOr(authoredRainLevel, 0.0F)));
    SeepageFingerprintFloat(
        &hash,
        std::clamp(
            SeepageFiniteOr(durationSeconds, 0.0F),
            0.0F,
            86'400.0F));
    SeepageFingerprintFloat(&hash, response.delaySeconds);
    SeepageFingerprintFloat(&hash, response.riseSeconds);
    SeepageFingerprintFloat(&hash, response.recessionSeconds);
    FingerprintAuthoredRainEnvelopeTrack(&hash, rainTrack);
    FingerprintAuthoredRainEnvelopeTrack(&hash, delayTrack);
    FingerprintAuthoredRainEnvelopeTrack(&hash, riseTrack);
    FingerprintAuthoredRainEnvelopeTrack(&hash, recessionTrack);
    return "water-seepage-node-rain-envelope-v1-" +
           SeepageFingerprintString(hash);
}

WaterSeepageRainEnvelope BuildWaterSeepageNodeRainEnvelope(
    const WaterSeepageNode& node,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds,
    float sampleRateHz,
    std::size_t maxSamples) {
    const WaterKeyedFeatureId rainFeature{
        .kind = WaterKeyedFeatureKind::Rain,
    };
    const WaterKeyedFeatureId nodeFeature{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = node.id,
    };
    const auto* rainTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        rainFeature,
        "level");
    const auto* delayTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        nodeFeature,
        "rain_delay_seconds");
    const auto* riseTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        nodeFeature,
        "rain_rise_seconds");
    const auto* recessionTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        nodeFeature,
        "rain_recession_seconds");
    const auto baseResponse =
        ResolveWaterSeepageNodeRainResponse(node);
    const auto domain = MakeWaterRainEnvelopeDomain(
        durationSeconds,
        sampleRateHz,
        maxSamples);
    return BuildAuthoredRainEnvelope<WaterSeepageRainEnvelope>(
        domain,
        [&](float position) {
            return EvaluateAuthoredRainEnvelopeTrack(
                rainTrack,
                position,
                Clamp01(SeepageFiniteOr(authoredRainLevel, 0.0F)),
                0.0F,
                1.0F);
        },
        [&](float position) {
            return WaterRainResponseSettings{
                .delaySeconds =
                    EvaluateAuthoredRainEnvelopeTrack(
                        delayTrack,
                        position,
                        baseResponse.delaySeconds,
                        0.0F,
                        86'400.0F),
                .riseSeconds =
                    EvaluateAuthoredRainEnvelopeTrack(
                        riseTrack,
                        position,
                        baseResponse.riseSeconds,
                        0.0F,
                        86'400.0F),
                .recessionSeconds =
                    EvaluateAuthoredRainEnvelopeTrack(
                        recessionTrack,
                        position,
                        baseResponse.recessionSeconds,
                        0.0F,
                        86'400.0F),
            };
        },
        WaterSeepageNodeRainEnvelopeFingerprint(
            node,
            runs,
            authoredRainLevel,
            durationSeconds));
}

std::string WaterMeshFlowRainEnvelopeFingerprint(
    const WaterDynamicMeshFlowSettings& settings,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds) {
    const WaterKeyedFeatureId rainFeature{
        .kind = WaterKeyedFeatureKind::Rain,
    };
    const WaterKeyedFeatureId meshFeature{
        .kind = WaterKeyedFeatureKind::MeshFlow,
    };
    const auto* rainTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        rainFeature,
        "level");
    const auto* persistenceTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        meshFeature,
        "moisture_persistence");
    const auto* riseTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        meshFeature,
        "rain_rise_seconds");
    const auto* recessionTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        meshFeature,
        "rain_recession_seconds");
    const auto sanitized =
        SanitizeWaterDynamicMeshFlowSettings(settings);

    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 1U);
    SeepageFingerprintFloat(
        &hash,
        Clamp01(SeepageFiniteOr(authoredRainLevel, 0.0F)));
    SeepageFingerprintFloat(
        &hash,
        std::clamp(
            SeepageFiniteOr(durationSeconds, 0.0F),
            0.0F,
            86'400.0F));
    SeepageFingerprintFloat(
        &hash,
        sanitized.moisturePersistenceMultiplier);
    SeepageFingerprintFloat(&hash, sanitized.rainRiseSeconds);
    SeepageFingerprintFloat(
        &hash,
        sanitized.rainRecessionSeconds);
    FingerprintAuthoredRainEnvelopeTrack(&hash, rainTrack);
    FingerprintAuthoredRainEnvelopeTrack(&hash, persistenceTrack);
    FingerprintAuthoredRainEnvelopeTrack(&hash, riseTrack);
    FingerprintAuthoredRainEnvelopeTrack(&hash, recessionTrack);
    return "water-mesh-flow-authored-rain-envelope-v1-" +
           SeepageFingerprintString(hash);
}

WaterMeshFlowRainEnvelope BuildWaterMeshFlowRainEnvelope(
    const WaterDynamicMeshFlowSettings& settings,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds,
    float sampleRateHz,
    std::size_t maxSamples) {
    const WaterKeyedFeatureId rainFeature{
        .kind = WaterKeyedFeatureKind::Rain,
    };
    const WaterKeyedFeatureId meshFeature{
        .kind = WaterKeyedFeatureKind::MeshFlow,
    };
    const auto* rainTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        rainFeature,
        "level");
    const auto* persistenceTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        meshFeature,
        "moisture_persistence");
    const auto* riseTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        meshFeature,
        "rain_rise_seconds");
    const auto* recessionTrack = FindAuthoredRainEnvelopeTrack(
        runs,
        meshFeature,
        "rain_recession_seconds");
    const auto sanitized =
        SanitizeWaterDynamicMeshFlowSettings(settings);
    const auto domain = MakeWaterRainEnvelopeDomain(
        durationSeconds,
        sampleRateHz,
        maxSamples);
    return BuildAuthoredRainEnvelope<WaterMeshFlowRainEnvelope>(
        domain,
        [&](float position) {
            return EvaluateAuthoredRainEnvelopeTrack(
                rainTrack,
                position,
                Clamp01(SeepageFiniteOr(authoredRainLevel, 0.0F)),
                0.0F,
                1.0F);
        },
        [&](float position) {
            const float persistence =
                EvaluateAuthoredRainEnvelopeTrack(
                    persistenceTrack,
                    position,
                    sanitized.moisturePersistenceMultiplier,
                    0.0F,
                    8.0F);
            return WaterRainResponseSettings{
                .riseSeconds =
                    EvaluateAuthoredRainEnvelopeTrack(
                        riseTrack,
                        position,
                        sanitized.rainRiseSeconds,
                        0.0F,
                        86'400.0F) *
                    persistence,
                .recessionSeconds =
                    EvaluateAuthoredRainEnvelopeTrack(
                        recessionTrack,
                        position,
                        sanitized.rainRecessionSeconds,
                        0.0F,
                        86'400.0F) *
                    persistence,
            };
        },
        WaterMeshFlowRainEnvelopeFingerprint(
            settings,
            runs,
            authoredRainLevel,
            durationSeconds));
}

const WaterFeatureTimingRun* FindWaterFeatureRunContaining(
    std::span<const WaterFeatureTimingRun> runs,
    const WaterKeyedFeatureId& feature,
    bool includeDisabled) {
    for (const auto& run : runs) {
        if (!includeDisabled && !run.enabled) {
            continue;
        }
        if (FindWaterFeatureTimeline(&run, feature) != nullptr) {
            return &run;
        }
    }
    return nullptr;
}

WaterFeatureTimeline* FindWaterFeatureTimeline(
    WaterFeatureTimingRun* run,
    const WaterKeyedFeatureId& feature) {
    if (run == nullptr) {
        return nullptr;
    }
    for (auto& timeline : run->features) {
        if (timeline.feature == feature) {
            return &timeline;
        }
    }
    return nullptr;
}

const WaterFeatureTimeline* FindWaterFeatureTimeline(
    const WaterFeatureTimingRun* run,
    const WaterKeyedFeatureId& feature) {
    return FindWaterFeatureTimeline(
        const_cast<WaterFeatureTimingRun*>(run),
        feature);
}

WaterSeepageQuality ResolveWaterSeepageQuality(
    WaterSeepageQuality quality,
    std::uint64_t effectivePointInvocations) {
    if (quality != WaterSeepageQuality::Auto) {
        return quality;
    }
    if (effectivePointInvocations <= 10'000'000ULL) {
        return WaterSeepageQuality::High;
    }
    if (effectivePointInvocations <= 50'000'000ULL) {
        return WaterSeepageQuality::Balanced;
    }
    return WaterSeepageQuality::Low;
}

WaterSeepageLookSettings ResolveWaterSeepageLook(
    std::span<const WaterSeepageLookProfile> profiles,
    std::span<const WaterSeepageResponseProfile> responseProfiles,
    const WaterSeepageLookSettings& defaultLook,
    std::string_view profileName,
    std::string_view responseProfileName) {
    // Names are trimmed on both sides so a stored name with stray whitespace
    // still resolves to the profile the UI reports as assigned.
    const auto trimmedProfileName = TrimSeepageName(profileName);
    const auto trimmedResponseName = TrimSeepageName(responseProfileName);
    WaterSeepageLookSettings resolved = defaultLook;
    if (!trimmedProfileName.empty()) {
        const auto profile = std::find_if(
            profiles.begin(),
            profiles.end(),
            [&](const WaterSeepageLookProfile& candidate) {
                return TrimSeepageName(candidate.name) == trimmedProfileName;
            });
        if (profile != profiles.end()) {
            resolved = profile->settings;
        }
    }
    // The response half always comes from the response profile (falling back
    // to the default look's response), so switching the seepage effect keeps
    // the chosen visual response and vice versa.
    resolved.response = defaultLook.response;
    resolved.blendMode = defaultLook.blendMode;
    if (!trimmedResponseName.empty()) {
        const auto responseProfile = std::find_if(
            responseProfiles.begin(),
            responseProfiles.end(),
            [&](const WaterSeepageResponseProfile& candidate) {
                return TrimSeepageName(candidate.name) == trimmedResponseName;
            });
        if (responseProfile != responseProfiles.end()) {
            resolved.response = responseProfile->response;
            resolved.blendMode = responseProfile->blendMode;
        }
    }
    return SanitizeSeepageLook(resolved);
}

WaterSeepageLookSettings ResolveWaterSeepageLook(
    const WaterSeepageNode& node,
    std::span<const WaterSeepageLookProfile> profiles,
    std::span<const WaterSeepageResponseProfile> responseProfiles,
    const WaterSeepageLookSettings& defaultLook) {
    return ResolveWaterSeepageLook(
        profiles,
        responseProfiles,
        defaultLook,
        node.lookProfileName,
        node.responseProfileName);
}

WaterSeepageNodeSettings ExtractWaterSeepageNodeSettings(
    const WaterSeepageNode& node) {
    return {
        .widthMeters = node.widthMeters,
        .prominence = node.prominence,
        .selectionReachLimitMeters = node.selectionReachLimitMeters,
        .selectionWidthLimitMeters = node.selectionWidthLimitMeters,
        .edgeFeatherMeters = node.edgeFeatherMeters,
        .depthToleranceMeters = node.depthToleranceMeters,
        .normalAlignment = node.normalAlignment,
        .strength = node.strength,
        .rainDelaySeconds = node.rainDelaySeconds,
        .rainRiseSeconds = node.rainRiseSeconds,
        .rainRecessionSeconds = node.rainRecessionSeconds,
        .targetSceneRoles = node.targetSceneRoles,
    };
}

void ApplyWaterSeepageNodeSettings(
    const WaterSeepageNodeSettings& settings,
    WaterSeepageNode* node) {
    if (node == nullptr) {
        return;
    }
    node->widthMeters = std::max(0.0F, settings.widthMeters);
    // Keep the legacy width aliases coherent for cache-less fallback paths.
    node->startWidthMeters = node->widthMeters;
    node->endWidthMeters = node->widthMeters;
    node->prominence = std::max(0.0F, settings.prominence);
    node->selectionReachLimitMeters =
        std::max(0.05F, settings.selectionReachLimitMeters);
    node->selectionWidthLimitMeters = std::max(
        node->widthMeters,
        settings.selectionWidthLimitMeters);
    node->edgeFeatherMeters = std::max(0.0F, settings.edgeFeatherMeters);
    node->depthToleranceMeters =
        std::max(0.005F, settings.depthToleranceMeters);
    node->normalAlignment = std::clamp(settings.normalAlignment, 0.0F, 1.0F);
    node->strength = std::max(0.0F, settings.strength);
    node->rainDelaySeconds =
        std::clamp(settings.rainDelaySeconds, 0.0F, 86'400.0F);
    node->rainRiseSeconds =
        std::clamp(settings.rainRiseSeconds, 0.0F, 86'400.0F);
    node->rainRecessionSeconds =
        std::clamp(settings.rainRecessionSeconds, 0.0F, 86'400.0F);
    node->targetSceneRoles = settings.targetSceneRoles;
}

std::optional<WaterSeepageNodeSettings>
ResolveWaterSeepageNodeSettingsProfileBaseline(
    const WaterSeepageNodeSettings& defaultSettings,
    std::span<const WaterSeepageNodeSettingsProfile> profiles,
    std::string_view assignedProfileName) {
    const auto assigned = TrimSeepageName(assignedProfileName);
    std::string_view baseName;
    const auto assignedCopy = std::find_if(
        profiles.begin(),
        profiles.end(),
        [&](const WaterSeepageNodeSettingsProfile& profile) {
            return profile.objectOverride &&
                   TrimSeepageName(profile.name) == assigned;
        });
    if (assignedCopy != profiles.end()) {
        baseName = TrimSeepageName(assignedCopy->baseProfileName);
    } else {
        constexpr std::string_view kEditedSuffix = "_edited";
        constexpr std::string_view kLegacyEditedSuffix = "_Edited";
        const auto suffix = assigned.ends_with(kEditedSuffix)
                                ? kEditedSuffix
                                : (assigned.ends_with(
                                       kLegacyEditedSuffix)
                                       ? kLegacyEditedSuffix
                                       : std::string_view{});
        if (suffix.empty() || assigned.size() == suffix.size()) {
            return std::nullopt;
        }
        baseName = assigned.substr(
            0U,
            assigned.size() - suffix.size());
    }

    if (baseName.empty() || baseName == "Default") {
        return defaultSettings;
    }
    const auto base = std::find_if(
        profiles.begin(),
        profiles.end(),
        [&](const WaterSeepageNodeSettingsProfile& profile) {
            return TrimSeepageName(profile.name) == baseName;
        });
    return base != profiles.end()
               ? std::optional<WaterSeepageNodeSettings>{base->settings}
               : std::optional<WaterSeepageNodeSettings>{defaultSettings};
}

WaterSeepageLookSettings ResolveWaterSeepageTimingLookBase(
    const WaterSeepageLookSettings& resolvedAuthoredLook,
    const std::optional<WaterScenarioState>& /*scenarioState*/) {
    // Scenarios are legacy timing containers only. A scalar keyed look always
    // materializes from the node's resolved authored profile/response pair.
    return SanitizeSeepageLook(resolvedAuthoredLook);
}

std::string WaterSeepageLocalLookName(std::string_view baseName, std::uint32_t nodeId) {
    baseName = TrimSeepageName(baseName);
    if (baseName.empty()) {
        baseName = "Default";
    }
    std::ostringstream stream;
    stream << baseName << '_' << std::setw(2) << std::setfill('0') << nodeId;
    return stream.str();
}

invisible_places::io::Float3 DeriveWaterSeepageDownAxis(
    const invisible_places::io::Float3& surfaceNormal,
    const invisible_places::io::Float3& fallbackAxis) {
    const glm::vec3 normal = SafeSeepageNormal(surfaceNormal);
    const auto projectToSurface = [&](glm::vec3 axis) {
        if (!IsValidPoint(axis) || glm::dot(axis, axis) <= kNormalEpsilon) {
            return glm::vec3{0.0F};
        }
        axis -= normal * glm::dot(axis, normal);
        return glm::dot(axis, axis) > kNormalEpsilon ? glm::normalize(axis) : glm::vec3{0.0F};
    };
    glm::vec3 down = projectToSurface(kGravity);
    if (glm::dot(down, down) <= kNormalEpsilon) {
        down = projectToSurface(ToGlm(fallbackAxis));
    }
    if (glm::dot(down, down) <= kNormalEpsilon) {
        const glm::vec3 rawFallback = ToGlm(fallbackAxis);
        down = IsValidPoint(rawFallback) && glm::dot(rawFallback, rawFallback) > kNormalEpsilon
                   ? glm::normalize(rawFallback)
                   : kGravity;
    }
    if (glm::dot(down, kGravity) < 0.0F) {
        down = -down;
    }
    return FromGlm(down);
}

std::uint32_t PackWaterSeepageSupportReferenceMetadata(
    const invisible_places::io::Float3& surfaceNormal,
    WaterSurfaceRole sourceRole,
    float confidence,
    std::uint32_t flags) {
    glm::vec3 normal = SafeSeepageNormal(surfaceNormal);
    normal /= std::max(
        1.0e-6F,
        std::abs(normal.x) + std::abs(normal.y) + std::abs(normal.z));
    glm::vec2 octahedral{normal.x, normal.y};
    if (normal.z < 0.0F) {
        const glm::vec2 original = octahedral;
        octahedral.x = (1.0F - std::abs(original.y)) *
                       (original.x < 0.0F ? -1.0F : 1.0F);
        octahedral.y = (1.0F - std::abs(original.x)) *
                       (original.y < 0.0F ? -1.0F : 1.0F);
    }
    const auto quantizeNormal = [](float value) {
        return static_cast<std::uint32_t>(std::lround(
            std::clamp(value * 0.5F + 0.5F, 0.0F, 1.0F) * 1023.0F));
    };
    const auto packedConfidence = static_cast<std::uint32_t>(std::lround(
        Clamp01(SeepageFiniteOr(confidence, 0.0F)) * 255.0F));
    return quantizeNormal(octahedral.x) |
           (quantizeNormal(octahedral.y) << 10U) |
           (packedConfidence << 20U) |
           ((static_cast<std::uint32_t>(sourceRole) & 0x3U) << 28U) |
           ((flags & 0x3U) << 30U);
}

WaterSeepageSupportReferenceMetadata UnpackWaterSeepageSupportReferenceMetadata(
    std::uint32_t packed) {
    const auto decodeNormal = [](std::uint32_t value) {
        return static_cast<float>(value) / 1023.0F * 2.0F - 1.0F;
    };
    glm::vec3 normal{
        decodeNormal(packed & 0x3ffU),
        decodeNormal((packed >> 10U) & 0x3ffU),
        0.0F,
    };
    normal.z = 1.0F - std::abs(normal.x) - std::abs(normal.y);
    if (normal.z < 0.0F) {
        const glm::vec2 original{normal.x, normal.y};
        normal.x = (1.0F - std::abs(original.y)) *
                   (original.x < 0.0F ? -1.0F : 1.0F);
        normal.y = (1.0F - std::abs(original.x)) *
                   (original.y < 0.0F ? -1.0F : 1.0F);
    }
    if (!IsValidPoint(normal) || glm::dot(normal, normal) <= kNormalEpsilon) {
        normal = {0.0F, 0.0F, 1.0F};
    } else {
        normal = glm::normalize(normal);
    }
    return {
        .surfaceNormal = FromGlm(normal),
        .sourceRole = static_cast<WaterSurfaceRole>((packed >> 28U) & 0x3U),
        .confidence = static_cast<float>((packed >> 20U) & 0xffU) / 255.0F,
        .flags = (packed >> 30U) & 0x3U,
    };
}

std::uint32_t PackWaterSeepageSupportRunCrossMetrics(
    float flowRunMeters,
    float crossContourMeters) {
    constexpr float kMaximumFiniteHalf = 65504.0F;
    const glm::vec2 metrics{
        std::clamp(
            SeepageFiniteOr(flowRunMeters, 0.0F),
            0.0F,
            kMaximumFiniteHalf),
        std::clamp(
            SeepageFiniteOr(crossContourMeters, 0.0F),
            0.0F,
            kMaximumFiniteHalf),
    };
    return glm::packHalf2x16(metrics);
}

WaterSeepageSupportRunCrossMetrics
UnpackWaterSeepageSupportRunCrossMetrics(std::uint32_t packed) {
    const glm::vec2 metrics = glm::unpackHalf2x16(packed);
    return {
        .flowRunMeters = std::max(
            0.0F,
            SeepageFiniteOr(metrics.x, 0.0F)),
        .crossContourMeters = std::max(
            0.0F,
            SeepageFiniteOr(metrics.y, 0.0F)),
    };
}

namespace {

WaterSeepagePulseFieldQuality ResolveSeepagePulseFieldQuality(
    WaterSeepageQuality quality) {
    switch (quality) {
        case WaterSeepageQuality::Low:
            return WaterSeepagePulseFieldQuality::Low;
        case WaterSeepageQuality::High:
            return WaterSeepagePulseFieldQuality::High;
        case WaterSeepageQuality::Auto:
        case WaterSeepageQuality::Balanced:
            return WaterSeepagePulseFieldQuality::Balanced;
    }
    return WaterSeepagePulseFieldQuality::Balanced;
}

WaterSeepagePulseField BuildRuntimeSeepagePulseField(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageLookSettings& look) {
    if (look.pattern != WaterSeepagePattern::ContourPulses) {
        return {};
    }
    WaterSeepagePulseFieldSettings settings;
    settings.spacingMeters = look.pulseSpacingMeters;
    settings.widthMeters = look.pulseWidthMeters;
    settings.speedMetersPerSecond =
        look.pulseSpeedMetersPerSecond;
    settings.irregularity = look.pulseIrregularity;
    settings.evolution = look.evolution;
    settings.waveCount = look.pulseWaveCount;
    settings.speedVariation = look.pulseSpeedVariation;
    settings.seed = node.seed;
    settings.nodeId = node.id;
    settings.quality =
        ResolveSeepagePulseFieldQuality(node.resolvedQuality);
    // Include a fixed idle tail after the immutable support extent. The last
    // reference sample is made equal to the first below, so wrapped shader
    // sampling crosses the cycle boundary without a visible flash.
    settings.stableSpanMeters = std::max(
        0.005F,
        std::max(0.005F, node.pulseStableSpanMeters) +
            std::max(0.005F, look.pulseSpacingMeters) * 2.5F);
    settings.timeSeconds = 0.0F;
    auto field = BuildWaterSeepagePulseField(settings);
    if (field.sampleCount > 1U) {
        field.samples[field.sampleCount - 1U] = field.samples[0U];
    }
    return field;
}

std::uint64_t WaterSeepagePulseFieldPreparationFingerprint(
    const WaterSeepageRuntimeNode& node) {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto fingerprintLook =
        [&](const WaterSeepageLookSettings& look) {
            SeepageFingerprintU32(
                &hash,
                static_cast<std::uint32_t>(look.pattern));
            if (look.pattern != WaterSeepagePattern::ContourPulses) {
                return;
            }
            SeepageFingerprintFloat(&hash, look.pulseSpacingMeters);
            SeepageFingerprintFloat(&hash, look.pulseWidthMeters);
            SeepageFingerprintFloat(&hash, look.pulseIrregularity);
            SeepageFingerprintFloat(&hash, look.pulseWaveCount);
        };
    SeepageFingerprintU32(&hash, node.id);
    SeepageFingerprintU32(&hash, node.seed);
    SeepageFingerprintU32(
        &hash,
        static_cast<std::uint32_t>(node.resolvedQuality));
    SeepageFingerprintFloat(
        &hash,
        node.pulseStableSpanMeters);
    fingerprintLook(node.look);
    SeepageFingerprintU32(
        &hash,
        node.transitionLook.has_value() ? 1U : 0U);
    if (node.transitionLook.has_value()) {
        fingerprintLook(node.transitionLook.value());
    }
    // Zero is reserved for an unprepared runtime node.
    return hash == 0U ? 1U : hash;
}

void PrepareRuntimeSeepagePulseFields(
    WaterSeepageRuntimeNode* node,
    float /*sampleTimeSeconds*/) {
    if (node == nullptr) {
        return;
    }
    const auto preparationFingerprint =
        WaterSeepagePulseFieldPreparationFingerprint(*node);
    if (node->pulseFieldPreparationFingerprint ==
        preparationFingerprint) {
        return;
    }
    node->pulseField =
        BuildRuntimeSeepagePulseField(
            *node,
            node->look);
    node->transitionPulseField = node->transitionLook.has_value()
                                     ? BuildRuntimeSeepagePulseField(
                                           *node,
                                           node->transitionLook.value())
                                     : node->pulseField;
    node->pulseFieldPreparationFingerprint =
        preparationFingerprint;
}

}  // namespace

WaterSeepageSpatialGrid BuildWaterSeepageSpatialGrid(
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSeepageLookProfile> profiles,
    const WaterSeepageLookSettings& defaultLook,
    std::string_view targetSceneRole,
    bool forExport,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    std::span<const WaterSeepageSurfaceGuide> guides,
    const std::optional<WaterScenarioState>& scenarioState,
    std::span<const WaterSeepageNodeAnimationStateEntry> nodeAnimationStates,
    std::span<const WaterSeepageSupportSelection> supportSelections,
    std::span<const WaterSeepageResponseProfile> responseProfiles) {
    WaterSeepageSpatialGrid grid;
    grid.diagnostics.inputNodeCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        nodes.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    constexpr float preferredCellSize = 0.50F;
    float largestSpan = 0.0F;
    for (const auto& node : nodes) {
        const bool enabled = forExport ? node.enabledInExport : node.enabledInViewport;
        // Export snapshots retain their established filtering semantics. Live
        // viewport topology keeps disabled nodes indexed and publishes their
        // visibility through the parameter record instead.
        if ((forExport && !enabled) ||
            !SeepageNodeTargetsRole(node, targetSceneRole) ||
            !IsValidPoint(ToGlm(node.position))) {
            continue;
        }
        const auto authoredLook = ResolveWaterSeepageLook(
            node,
            profiles,
            responseProfiles,
            defaultLook);
        WaterSeepageRuntimeNode runtime;
        runtime.id = node.id;
        runtime.seed = node.seed;
        runtime.position = ToGlm(node.position);
        runtime.surfaceNormal = SafeSeepageNormal(node.surfaceNormal);
        const auto derivedDown = DeriveWaterSeepageDownAxis(node.surfaceNormal, node.downAxis);
        glm::vec3 requestedDown = ToGlm(node.downAxis);
        requestedDown -= runtime.surfaceNormal * glm::dot(requestedDown, runtime.surfaceNormal);
        runtime.downAxis =
            IsValidPoint(requestedDown) && glm::dot(requestedDown, requestedDown) > kNormalEpsilon
                ? glm::normalize(requestedDown)
                : ToGlm(derivedDown);
        if (glm::dot(runtime.downAxis, kGravity) < 0.0F) {
            runtime.downAxis = -runtime.downAxis;
        }
        runtime.lateralAxis = glm::cross(runtime.surfaceNormal, runtime.downAxis);
        if (glm::dot(runtime.lateralAxis, runtime.lateralAxis) <= kNormalEpsilon) {
            const glm::vec3 helper = std::abs(runtime.surfaceNormal.x) < 0.8F
                                         ? glm::vec3{1.0F, 0.0F, 0.0F}
                                         : glm::vec3{0.0F, 1.0F, 0.0F};
            runtime.lateralAxis = glm::cross(runtime.surfaceNormal, helper);
        }
        runtime.lateralAxis = glm::normalize(runtime.lateralAxis);
        runtime.noiseRotation = SeepageNoiseRotation(node.seed ^ (node.id * 0x9e3779b9U));
        runtime.authoredReachMeters = std::clamp(
            SeepageFiniteOr(node.reachMeters, 1.25F),
            0.001F,
            1000.0F);
        runtime.authoredWidthMeters = std::clamp(
            SeepageFiniteOr(node.widthMeters, 0.75F),
            0.002F,
            1000.0F);
        runtime.authoredProminence = std::clamp(
            SeepageFiniteOr(node.prominence, 1.0F),
            0.0F,
            8.0F);
        runtime.selectionReachLimitMeters = std::clamp(
            SeepageFiniteOr(node.selectionReachLimitMeters, 2.34375F),
            0.001F,
            1000.0F);
        runtime.selectionWidthLimitMeters = std::clamp(
            SeepageFiniteOr(node.selectionWidthLimitMeters, 1.215F),
            0.002F,
            1000.0F);
        runtime.pulseStableSpanMeters = std::max(
            0.005F,
            runtime.selectionReachLimitMeters /
                kWaterSeepageDescentCostFactor);
        runtime.authoredStartHalfWidthMeters = 0.5F * std::clamp(
            SeepageFiniteOr(node.startWidthMeters, 0.12F),
            0.002F,
            1000.0F);
        runtime.reachMeters = runtime.authoredReachMeters;
        runtime.widthMeters = runtime.authoredWidthMeters;
        runtime.prominence = runtime.authoredProminence;
        runtime.startHalfWidthMeters = runtime.authoredStartHalfWidthMeters;
        runtime.endHalfWidthMeters = runtime.authoredWidthMeters * 0.5F;
        runtime.edgeFeatherMeters = std::clamp(
            SeepageFiniteOr(node.edgeFeatherMeters, 0.10F),
            0.001F,
            100.0F);
        runtime.depthToleranceMeters = std::clamp(
            SeepageFiniteOr(node.depthToleranceMeters, 0.15F),
            0.001F,
            100.0F);
        runtime.normalAlignment = std::clamp(
            SeepageFiniteOr(node.normalAlignment, 0.20F),
            0.0F,
            1.0F);
        runtime.authoredStrength = std::clamp(
            SeepageFiniteOr(node.strength, 1.0F),
            0.0F,
            8.0F);
        runtime.enabledFactor = enabled ? 1.0F : 0.0F;
        runtime.authoredLook = authoredLook;
        ApplySeepageRuntimeScenarioAndAnimation(
            &runtime,
            scenarioState,
            rainSettings,
            effectivePointInvocations,
            FindSeepageNodeAnimationState(nodeAnimationStates, node.id));
        PrepareRuntimeSeepagePulseFields(&runtime, 0.0F);
        CopyWaterSeepageSurfaceGuide(
            FindWaterSeepageSurfaceGuide(guides, node.id),
            &runtime);
        largestSpan = std::max(
            largestSpan,
            std::max(
                runtime.selectionReachLimitMeters +
                    runtime.edgeFeatherMeters * 2.0F,
                std::max(runtime.startHalfWidthMeters,
                         runtime.selectionWidthLimitMeters * 0.5F) * 2.0F +
                    runtime.edgeFeatherMeters * 2.0F));
        const auto nodeBounds = SeepageRuntimeNodeBounds(runtime);
        if (nodeBounds.valid) {
            grid.unionBounds.Expand(nodeBounds.minimum);
            grid.unionBounds.Expand(nodeBounds.maximum);
        }
        grid.nodes.push_back(std::move(runtime));
    }

    std::sort(
        grid.nodes.begin(),
        grid.nodes.end(),
        [](const WaterSeepageRuntimeNode& left, const WaterSeepageRuntimeNode& right) {
            if (left.id != right.id) {
                return left.id < right.id;
            }
            if (left.position.x != right.position.x) {
                return left.position.x < right.position.x;
            }
            if (left.position.y != right.position.y) {
                return left.position.y < right.position.y;
            }
            if (left.position.z != right.position.z) {
                return left.position.z < right.position.z;
            }
            return left.seed < right.seed;
        });

    // Connected selections are immutable topology derived from the shared
    // surface cache. Build their exact 10 mm hash independently of the coarse
    // fallback fan grid so live reach/width/prominence edits remain compact
    // parameter writes.
    using SupportKey = std::tuple<std::int32_t, std::int32_t, std::int32_t>;
    struct PendingSupportReference {
        std::uint32_t nodeIndex = 0U;
        WaterSeepageSupportCell cell{};
        WaterSurfaceRole sourceRole = WaterSurfaceRole::None;
    };
    std::map<SupportKey, std::vector<PendingSupportReference>> supportByCell;
    const auto normalizedTargetRole = NormalizeSeepageRole(targetSceneRole);
    for (std::size_t nodeIndex = 0U; nodeIndex < grid.nodes.size(); ++nodeIndex) {
        auto& runtime = grid.nodes[nodeIndex];
        const auto selection = std::find_if(
            supportSelections.begin(),
            supportSelections.end(),
            [&](const WaterSeepageSupportSelection& candidate) {
                return candidate.nodeId == runtime.id &&
                       NormalizeSeepageRole(candidate.targetSceneRole) == normalizedTargetRole &&
                       std::abs(candidate.cellSizeMeters -
                                kWaterSeepageSupportCellSizeMeters) <= 1.0e-6F &&
                       !candidate.cells.empty() && !candidate.fingerprint.empty();
            });
        if (selection == supportSelections.end()) {
            continue;
        }
        runtime.usesConnectedSupport = true;
        ++grid.diagnostics.supportSelectionCount;
        for (const auto& cell : selection->cells) {
            if (!std::isfinite(cell.downwardDistanceMeters) ||
                !std::isfinite(cell.flowRunMeters) ||
                !std::isfinite(cell.crossContourMeters)) {
                continue;
            }
            if (cell.upstream) {
                runtime.maximumUpstreamRunMeters = std::max(
                    runtime.maximumUpstreamRunMeters,
                    std::max(0.0F, cell.flowRunMeters));
            }
            supportByCell[{cell.x, cell.y, cell.z}].push_back({
                .nodeIndex = static_cast<std::uint32_t>(nodeIndex),
                .cell = cell,
                .sourceRole = normalizedTargetRole == "veg"
                                  ? WaterSurfaceRole::Vegetation
                                  : selection->sourceRole,
            });
        }
        if (selection->bounds.valid) {
            grid.supportUnionBounds.Expand(selection->bounds.minimum);
            grid.supportUnionBounds.Expand(selection->bounds.maximum);
        }
    }

    if (!supportByCell.empty()) {
        std::size_t supportCapacity = 16U;
        while (supportByCell.size() * 10U >= supportCapacity * 7U) {
            supportCapacity *= 2U;
        }
        grid.supportHashCells.resize(supportCapacity);
        grid.supportCellSizeMeters = kWaterSeepageSupportCellSizeMeters;
        for (auto& [key, references] : supportByCell) {
            std::sort(
                references.begin(),
                references.end(),
                [&](const PendingSupportReference& left,
                    const PendingSupportReference& right) {
                    const auto& leftNode = grid.nodes[left.nodeIndex];
                    const auto& rightNode = grid.nodes[right.nodeIndex];
                    if (leftNode.id != rightNode.id) {
                        return leftNode.id < rightNode.id;
                    }
                    if (left.cell.confidence != right.cell.confidence) {
                        return left.cell.confidence > right.cell.confidence;
                    }
                    return left.nodeIndex < right.nodeIndex;
                });
            references.erase(
                std::unique(
                    references.begin(),
                    references.end(),
                    [](const PendingSupportReference& left,
                       const PendingSupportReference& right) {
                        return left.nodeIndex == right.nodeIndex;
                    }),
                references.end());
            if (references.size() > WaterSeepageSpatialGrid::kMaxReferencesPerCell) {
                ++grid.diagnostics.supportOverflowCellCount;
                grid.diagnostics.droppedSupportReferenceCount +=
                    static_cast<std::uint32_t>(
                        references.size() - WaterSeepageSpatialGrid::kMaxReferencesPerCell);
                references.resize(WaterSeepageSpatialGrid::kMaxReferencesPerCell);
            }

            const auto [x, y, z] = key;
            const auto mask = grid.supportHashCells.size() - 1U;
            std::size_t slot = static_cast<std::size_t>(SeepageSpatialHash(x, y, z)) & mask;
            while (grid.supportHashCells[slot].occupied) {
                slot = (slot + 1U) & mask;
            }
            auto& hashCell = grid.supportHashCells[slot];
            hashCell.x = x;
            hashCell.y = y;
            hashCell.z = z;
            hashCell.referenceOffset =
                static_cast<std::uint32_t>(grid.supportReferences.size());
            hashCell.referenceCount = static_cast<std::uint32_t>(references.size());
            hashCell.occupied = true;
            for (const auto& reference : references) {
                grid.supportReferences.push_back({
                    .nodeIndex = reference.nodeIndex,
                    .downwardDistanceMeters = reference.cell.downwardDistanceMeters,
                    .packedRunCrossMeters =
                        PackWaterSeepageSupportRunCrossMetrics(
                            reference.cell.flowRunMeters,
                            reference.cell.crossContourMeters),
                    .packedNormalRoleConfidenceFlags =
                        PackWaterSeepageSupportReferenceMetadata(
                            reference.cell.surfaceNormal,
                            reference.sourceRole,
                            reference.cell.confidence,
                            kWaterSeepageSupportConnectedFlag |
                                (reference.cell.upstream
                                     ? kWaterSeepageSupportUpstreamFlag
                                     : 0U)),
                });
            }
        }
        grid.diagnostics.supportOccupiedCellCount =
            static_cast<std::uint32_t>(supportByCell.size());
        grid.diagnostics.supportHashCellCapacity =
            static_cast<std::uint32_t>(grid.supportHashCells.size());
        grid.diagnostics.supportReferenceCount =
            static_cast<std::uint32_t>(grid.supportReferences.size());
    }

    grid.cellSizeMeters = std::max(
        preferredCellSize,
        largestSpan > 0.0F ? largestSpan / 64.0F : preferredCellSize);
    grid.cellSizeMeters = std::clamp(grid.cellSizeMeters, 0.05F, 50.0F);
    grid.diagnostics.activeNodeCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        grid.nodes.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    if (grid.nodes.empty()) {
        return grid;
    }

    std::vector<MutableSeepageHashCell> mutableCells(16U);
    std::size_t occupiedCellCount = 0U;
    for (std::size_t nodeIndex = 0U; nodeIndex < grid.nodes.size(); ++nodeIndex) {
        const auto bounds = SeepageRuntimeNodeBounds(grid.nodes[nodeIndex]);
        if (!bounds.valid) {
            continue;
        }
        const auto minX = SeepageCellCoordinate(bounds.minimum.x, grid.cellSizeMeters);
        const auto minY = SeepageCellCoordinate(bounds.minimum.y, grid.cellSizeMeters);
        const auto minZ = SeepageCellCoordinate(bounds.minimum.z, grid.cellSizeMeters);
        const auto maxX = SeepageCellCoordinate(bounds.maximum.x, grid.cellSizeMeters);
        const auto maxY = SeepageCellCoordinate(bounds.maximum.y, grid.cellSizeMeters);
        const auto maxZ = SeepageCellCoordinate(bounds.maximum.z, grid.cellSizeMeters);
        for (std::int64_t x = minX; x <= static_cast<std::int64_t>(maxX); ++x) {
            for (std::int64_t y = minY; y <= static_cast<std::int64_t>(maxY); ++y) {
                for (std::int64_t z = minZ; z <= static_cast<std::int64_t>(maxZ); ++z) {
                    InsertSeepageCellReference(
                        &mutableCells,
                        &occupiedCellCount,
                        static_cast<std::int32_t>(x),
                        static_cast<std::int32_t>(y),
                        static_cast<std::int32_t>(z),
                        static_cast<std::uint32_t>(nodeIndex),
                        grid.nodes,
                        grid.cellSizeMeters,
                        &grid.diagnostics);
                }
            }
        }
    }

    for (auto& cell : mutableCells) {
        if (!cell.occupied || cell.referenceCount < 2U) {
            continue;
        }
        std::sort(
            cell.references.begin(),
            cell.references.begin() + static_cast<std::ptrdiff_t>(cell.referenceCount),
            [&](std::uint32_t left, std::uint32_t right) {
                return SeepageNodeReferencePreferred(
                    left,
                    right,
                    grid.nodes,
                    cell.x,
                    cell.y,
                    cell.z,
                    grid.cellSizeMeters);
            });
    }

    grid.hashCells.resize(mutableCells.size());
    for (std::size_t slot = 0U; slot < mutableCells.size(); ++slot) {
        const auto& source = mutableCells[slot];
        if (!source.occupied) {
            continue;
        }
        auto& target = grid.hashCells[slot];
        target.x = source.x;
        target.y = source.y;
        target.z = source.z;
        target.referenceOffset = static_cast<std::uint32_t>(grid.nodeReferences.size());
        target.referenceCount = source.referenceCount;
        target.occupied = true;
        for (std::uint32_t index = 0U; index < source.referenceCount; ++index) {
            grid.nodeReferences.push_back(source.references[index]);
        }
    }
    grid.diagnostics.occupiedCellCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        occupiedCellCount,
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    grid.diagnostics.hashCellCapacity = static_cast<std::uint32_t>(std::min<std::size_t>(
        grid.hashCells.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    grid.diagnostics.nodeReferenceCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        grid.nodeReferences.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    return grid;
}

void ApplyWaterSeepageRuntimeParameters(
    WaterSeepageSpatialGrid* grid,
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSeepageLookProfile> profiles,
    const WaterSeepageLookSettings& defaultLook,
    const std::optional<WaterScenarioState>& scenarioState,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    std::span<const WaterSeepageNodeAnimationStateEntry> nodeAnimationStates,
    std::span<const WaterSeepageResponseProfile> responseProfiles) {
    if (grid == nullptr) {
        return;
    }
    for (auto& runtime : grid->nodes) {
        const auto authored = std::find_if(
            nodes.begin(),
            nodes.end(),
            [&](const WaterSeepageNode& candidate) {
                return candidate.id == runtime.id;
            });
        if (authored != nodes.end()) {
            runtime.seed = authored->seed;
            runtime.noiseRotation = SeepageNoiseRotation(
                authored->seed ^ (authored->id * 0x9e3779b9U));
            runtime.normalAlignment = std::clamp(
                SeepageFiniteOr(authored->normalAlignment, 0.20F),
                0.0F,
                1.0F);
            runtime.authoredStrength = std::clamp(
                SeepageFiniteOr(authored->strength, 1.0F),
                0.0F,
                8.0F);
            runtime.authoredReachMeters = std::clamp(
                SeepageFiniteOr(authored->reachMeters, 1.25F),
                0.001F,
                std::max(0.001F, runtime.selectionReachLimitMeters));
            runtime.authoredWidthMeters = std::clamp(
                SeepageFiniteOr(authored->widthMeters, 0.75F),
                0.002F,
                std::max(0.002F, runtime.selectionWidthLimitMeters));
            runtime.authoredProminence = std::clamp(
                SeepageFiniteOr(authored->prominence, 1.0F),
                0.0F,
                8.0F);
            runtime.authoredStartHalfWidthMeters = 0.5F * std::clamp(
                SeepageFiniteOr(authored->startWidthMeters, 0.12F),
                0.002F,
                1000.0F);
            runtime.enabledFactor = authored->enabledInViewport ? 1.0F : 0.0F;
            runtime.authoredLook = ResolveWaterSeepageLook(
                *authored,
                profiles,
                responseProfiles,
                defaultLook);
        }
        ApplySeepageRuntimeScenarioAndAnimation(
            &runtime,
            scenarioState,
            rainSettings,
            effectivePointInvocations,
            FindSeepageNodeAnimationState(nodeAnimationStates, runtime.id));
    }
}

void ApplyWaterSeepageScenarioParameters(
    WaterSeepageSpatialGrid* grid,
    const std::optional<WaterScenarioState>& scenarioState,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    std::span<const WaterSeepageNodeAnimationStateEntry> nodeAnimationStates) {
    if (grid == nullptr) {
        return;
    }
    for (auto& node : grid->nodes) {
        ApplySeepageRuntimeScenarioAndAnimation(
            &node,
            scenarioState,
            rainSettings,
            effectivePointInvocations,
            FindSeepageNodeAnimationState(nodeAnimationStates, node.id));
    }
}

void PrepareWaterSeepagePulseFields(
    WaterSeepageSpatialGrid* grid,
    float sampleTimeSeconds) {
    if (grid == nullptr) {
        return;
    }
    for (auto& node : grid->nodes) {
        PrepareRuntimeSeepagePulseFields(
            &node,
            sampleTimeSeconds);
    }
}

namespace {

struct ConnectedSeepageLiveMask {
    float mask = 0.0F;
    float effectiveRun = 0.0F;
    float effectiveHalfWidth = 0.0F;
};

ConnectedSeepageLiveMask EvaluateConnectedSeepageLiveMask(
    const WaterSeepageRuntimeNode& node,
    float costMeters,
    float flowRunMeters,
    float crossContourMeters,
    const glm::vec3& surfaceNormal,
    bool upstream,
    float supportCellSizeMeters) {
    ConnectedSeepageLiveMask result;
    const float budget = std::max(0.0F, node.budgetMeters);
    if (budget <= 1.0e-5F) {
        return result;
    }
    const float cellSize = std::max(
        1.0e-5F,
        SeepageFiniteOr(
            supportCellSizeMeters,
            kWaterSeepageSupportCellSizeMeters));
    const float cost = std::max(
        0.0F,
        SeepageFiniteOr(costMeters, 0.0F));
    const float run = std::max(
        0.0F,
        SeepageFiniteOr(flowRunMeters, 0.0F));
    const float cross = std::max(
        0.0F,
        SeepageFiniteOr(crossContourMeters, 0.0F));
    const float sourceRadius =
        std::max(0.0F, node.widthMeters) * 0.5F;
    const float selectionHalfWidth =
        std::max(0.0F, node.selectionWidthLimitMeters) * 0.5F;
    const float authoredFeather =
        std::max(0.0F, node.edgeFeatherMeters);

    // Both route reveals are remaps of the compact live Strength value. The
    // settled support therefore stays immutable while the real resident
    // upstream tip reveals back toward the node before downhill is released.
    const float maximumBudget = std::max(
        cellSize,
        std::max(0.0F, node.selectionReachLimitMeters));
    const float sequenceProgress = Clamp01(
        budget / maximumBudget);
    const float maximumUpstreamRun = std::max(
        0.0F,
        SeepageFiniteOr(node.maximumUpstreamRunMeters, 0.0F));
    const float upstreamLeadFraction =
        maximumUpstreamRun > cellSize * 0.5F
            ? kWaterSeepageUpstreamLeadFraction
            : 0.0F;
    const float routeProgress =
        upstream
            ? (upstreamLeadFraction > 0.0F
                   ? Clamp01(
                         sequenceProgress /
                         upstreamLeadFraction)
                   : 0.0F)
            : (upstreamLeadFraction > 0.0F
                   ? Clamp01(
                         (sequenceProgress -
                          upstreamLeadFraction) /
                         (1.0F -
                          upstreamLeadFraction))
                   : sequenceProgress);
    const float routeBudget =
        maximumBudget * routeProgress;

    // Source feather is intentionally bounded by two resident support cells.
    // A large artistic edge feather can soften the travelling front but must
    // not create a broad, always-wet disk around the authored node.
    const float sourceFeather = std::max(
        1.0e-5F,
        std::min(authoredFeather, cellSize * 2.0F));
    const glm::vec3 normal =
        IsValidPoint(surfaceNormal) &&
                glm::dot(surfaceNormal, surfaceNormal) >
                    kNormalEpsilon
            ? glm::normalize(surfaceNormal)
            : glm::vec3{0.0F, 0.0F, 1.0F};
    const glm::vec3 projectedGravity =
        kGravity - normal * glm::dot(kGravity, normal);
    const float steepness = std::clamp(
        glm::length(projectedGravity),
        0.0F,
        1.0F);
    const float steepBlend = SmoothStep(
        kWaterSeepageSteepnessBlendStart,
        kWaterSeepageSteepnessBlendEnd,
        steepness);

    float budgetMask = 0.0F;
    float allowedHalfWidth = sourceRadius;
    if (upstream) {
        const float upstreamExtent =
            std::max(cellSize, maximumUpstreamRun);
        const float reverseFrontRun =
            upstreamExtent * (1.0F - routeProgress);
        const float reverseFrontFeather = std::max(
            cellSize,
            std::min(
                std::max(cellSize, authoredFeather),
                upstreamExtent * 0.15F));
        // Reverse the usual flood reveal: the furthest/highest resident cells
        // become wet first, then the front descends toward the authored node.
        budgetMask =
            reverseFrontRun <= 1.0e-5F
                ? 1.0F
                : SmoothStep(
                      std::max(
                          0.0F,
                          reverseFrontRun - reverseFrontFeather),
                      reverseFrontRun,
                      run);
        // Intensity builds with the reveal instead of saturating behind the
        // front, so low Strength no longer pops the whole upstream support
        // to full brightness; it reaches one exactly when the front meets
        // the node and stays there afterwards.
        budgetMask *= SmoothStep(0.0F, 1.0F, routeProgress);

        const float nodewardStation = Clamp01(
            1.0F - run / upstreamExtent);
        const float tipHalfWidth =
            std::min(sourceRadius, cellSize * 0.5F);
        const float fullTaperHalfWidth = std::lerp(
            tipHalfWidth,
            sourceRadius,
            nodewardStation);
        // The first top-down pass is deliberately a narrow centreline except
        // at the node, where it meets the authored Source Width. Once that
        // pass is complete, Strength fills the remaining taper laterally and
        // reaches the complete upstream envelope at Strength 1.
        const float centrelineHalfWidth = std::lerp(
            tipHalfWidth,
            sourceRadius,
            nodewardStation * nodewardStation);
        float lateralFill = 0.0F;
        if (routeProgress >= 1.0F - 1.0e-5F) {
            const float effectiveStrength =
                Clamp01(SeepageFiniteOr(node.strength, 0.0F));
            const float completionStrength =
                sequenceProgress > 1.0e-5F
                    ? std::clamp(
                          effectiveStrength *
                              upstreamLeadFraction /
                              sequenceProgress,
                          0.0F,
                          1.0F)
                    : effectiveStrength;
            lateralFill =
                completionStrength >= 1.0F - 1.0e-6F
                    ? (effectiveStrength >= 1.0F - 1.0e-6F
                           ? 1.0F
                           : 0.0F)
                    : SmoothStep(
                          completionStrength,
                          1.0F,
                          effectiveStrength);
        }
        allowedHalfWidth = std::lerp(
            centrelineHalfWidth,
            fullTaperHalfWidth,
            lateralFill);
        result.effectiveRun = upstreamExtent;
    } else {
        const float frontFeather = std::max(
            std::max(1.0e-5F, authoredFeather),
            routeBudget * 0.15F);
        budgetMask =
            routeBudget > 1.0e-5F
                ? 1.0F - SmoothStep(
                      std::max(
                          0.0F,
                          routeBudget - frontFeather),
                      routeBudget,
                      cost)
                : 0.0F;
        allowedHalfWidth +=
            run * std::lerp(0.55F, 0.10F, steepBlend);
        // `budget` is stored in least-resistance cost-metres. Convert it back
        // to the physical run of a pure downstream route at this cell's slope
        // before patterns use it to normalise a travelling wetting front.
        const float pureRouteCostPerMeter = std::lerp(
            kWaterSeepageContourCostFactor,
            kWaterSeepageDescentCostFactor,
            steepBlend);
        result.effectiveRun =
            routeBudget /
            std::max(1.0e-5F, pureRouteCostPerMeter);
    }
    allowedHalfWidth = std::clamp(
        allowedHalfWidth,
        0.0F,
        selectionHalfWidth);
    result.effectiveHalfWidth = allowedHalfWidth;
    const float lateralFeather = std::max(
        1.0e-5F,
        std::min(
            authoredFeather,
            std::max(
                cellSize * 2.0F,
                allowedHalfWidth * 0.25F)));
    const float widthMask = 1.0F - SmoothStep(
        allowedHalfWidth,
        allowedHalfWidth + lateralFeather,
        cross);
    const float sourceDistance = std::hypot(run, cross);
    const float sourceMask = 1.0F - SmoothStep(
        sourceRadius,
        sourceRadius + sourceFeather,
        sourceDistance);
    const bool upstreamReachedNode =
        upstreamLeadFraction <= 0.0F ||
        sequenceProgress + 1.0e-6F >= upstreamLeadFraction;
    // The below-node half of the source disk eases in over the next half of
    // the lead range once the upstream front arrives, instead of switching
    // on in a single step at the release strength.
    const float releaseEase =
        upstreamLeadFraction > 0.0F
            ? SmoothStep(
                  upstreamLeadFraction,
                  upstreamLeadFraction * 1.5F,
                  sequenceProgress)
            : SmoothStep(0.0F, 0.04F, sequenceProgress);
    const float releasedSourceMask =
        !upstream && upstreamReachedNode
            ? sourceMask * releaseEase
            : 0.0F;
    result.mask = Clamp01(std::max(
        releasedSourceMask,
        budgetMask * widthMask));
    return result;
}

WaterSeepageRuntimeContribution EvaluateWaterSeepageRuntimeContributionWithFan(
    const WaterSeepageRuntimeNode& node,
    const invisible_places::io::Float3& position,
    const glm::vec3& pointNormal,
    const SeepageFanSample& fan,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext) {
    WaterSeepageRuntimeContribution contribution;
    if (fan.mask <= 1.0e-6F) {
        return contribution;
    }

    const auto selectedLook =
        SelectSeepageTransitionLook(node, ToGlm(position));
    auto look = SanitizeSeepageLook(selectedLook.look);
    if (look.quality == WaterSeepageQuality::Auto) {
        look.quality = node.resolvedQuality;
    }
    const float wettingProgress = Clamp01(node.wettingProgress);
    if (wettingProgress <= 1.0e-6F) {
        return contribution;
    }
    auto wettingFan = fan;
    if (wettingProgress < 1.0F - 1.0e-6F) {
        // This is the CPU/offline mirror of the shader's node-level wetting
        // reveal. It gates every look, while Wetting Trickle additionally uses
        // the same progress value to reveal its patch and finger signals.
        const float frontReach =
            std::max(1.0e-4F, wettingFan.effectiveReach);
        const float frontSoftness = std::max(
            std::max(0.0F, node.edgeFeatherMeters),
            look.trickleFrontSoftness);
        const float frontDistance =
            frontReach * wettingProgress;
        const float frontMask = 1.0F - SmoothStep(
            frontDistance - frontSoftness,
            frontDistance + frontSoftness,
            std::max(0.0F, wettingFan.downDistance));
        wettingFan.mask *=
            frontMask *
            SmoothStep(0.0F, 0.04F, wettingProgress);
        if (wettingFan.mask <= 1.0e-6F) {
            return contribution;
        }
    }
    SeepagePatternSignals signals;
    switch (look.pattern) {
        case WaterSeepagePattern::WetRockSheen:
            signals = EvaluateWetRockSeepageSignals(
                node,
                look,
                wettingFan,
                ToGlm(position),
                pointNormal,
                timeSeconds,
                viewContext);
            break;
        case WaterSeepagePattern::ChaoticBloom:
            signals = EvaluateChaoticBloomSeepageSignals(
                node,
                look,
                wettingFan,
                ToGlm(position),
                pointNormal,
                timeSeconds,
                viewContext);
            break;
        case WaterSeepagePattern::WettingTrickle:
            signals = EvaluateWettingTrickleSeepageSignals(
                node,
                look,
                wettingFan,
                ToGlm(position),
                pointNormal,
                timeSeconds,
                viewContext);
            break;
        case WaterSeepagePattern::ContourPulses:
            signals = EvaluateContourPulseSeepageSignals(
                node,
                look,
                wettingFan,
                ToGlm(position),
                pointNormal,
                timeSeconds,
                selectedLook.transition,
                viewContext);
            break;
    }
    contribution.mask = wettingFan.mask;
    contribution.damp = signals.damp;
    contribution.ripple = signals.variation;
    contribution.glint = signals.glint;
    const float rainGain = Clamp01(node.rainVisualStrength * look.rainResponse);
    const float rainProminence = 1.0F + 0.65F * rainGain;
    contribution.scale = Clamp01(
        (contribution.damp * 0.58F + contribution.ripple * 0.34F + contribution.glint * 0.46F) *
        look.response.intensity * rainProminence * node.prominence);
    contribution.colourMix = Clamp01(look.response.colouriseAmount * contribution.scale);
    contribution.emissionAdd = std::max(0.0F, look.response.emissionAdd) * contribution.scale;
    contribution.opacityAdd = look.response.opacityAdd * contribution.scale;
    contribution.opacityMultiply = std::lerp(
        1.0F,
        std::max(0.0F, look.response.opacityMultiply),
        contribution.scale);
    contribution.pointSizeAdd = look.response.pointSizeAdd * contribution.scale;
    contribution.pointSizeMultiply = std::lerp(
        1.0F,
        std::max(0.0F, look.response.pointSizeMultiply),
        contribution.scale);
    contribution.colour = {
        look.response.colouriseRed,
        look.response.colouriseGreen,
        look.response.colouriseBlue,
    };
    return contribution;
}

SeepageFanSample EvaluateConnectedSeepageSupportMask(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageSupportReference& reference,
    const invisible_places::io::Float3& position,
    const glm::vec3& pointNormal) {
    SeepageFanSample sample;
    const auto metadata = UnpackWaterSeepageSupportReferenceMetadata(
        reference.packedNormalRoleConfidenceFlags);
    const auto runCross = UnpackWaterSeepageSupportRunCrossMetrics(
        reference.packedRunCrossMeters);
    const bool upstream =
        (metadata.flags & kWaterSeepageSupportUpstreamFlag) != 0U;
    // Upstream support participates in the source/wick mask but never drives
    // a procedural front uphill.
    sample.downDistance =
        upstream ? 0.0F : runCross.flowRunMeters;
    sample.lateralDistance = runCross.crossContourMeters;
    sample.surfaceNormal = SafeSeepageNormal(metadata.surfaceNormal);
    glm::vec3 projectedGravity =
        kGravity -
        sample.surfaceNormal *
            glm::dot(kGravity, sample.surfaceNormal);
    if (!IsValidPoint(projectedGravity) ||
        glm::dot(projectedGravity, projectedGravity) <=
            kNormalEpsilon) {
        projectedGravity =
            node.downAxis -
            sample.surfaceNormal *
                glm::dot(node.downAxis, sample.surfaceNormal);
    }
    if (!IsValidPoint(projectedGravity) ||
        glm::dot(projectedGravity, projectedGravity) <=
            kNormalEpsilon) {
        const glm::vec3 helper =
            std::abs(sample.surfaceNormal.x) < 0.8F
                ? glm::vec3{1.0F, 0.0F, 0.0F}
                : glm::vec3{0.0F, 1.0F, 0.0F};
        projectedGravity =
            glm::cross(sample.surfaceNormal, helper);
    }
    sample.downTangent = glm::normalize(projectedGravity);
    glm::vec3 localLateral =
        glm::cross(sample.surfaceNormal, sample.downTangent);
    if (!IsValidPoint(localLateral) ||
        glm::dot(localLateral, localLateral) <= kNormalEpsilon) {
        localLateral = node.lateralAxis;
    } else {
        localLateral = glm::normalize(localLateral);
        if (glm::dot(localLateral, node.lateralAxis) < 0.0F) {
            localLateral = -localLateral;
        }
    }
    sample.signedLateralDistance = glm::dot(
        ToGlm(position) - node.position,
        localLateral);
    const auto liveMask = EvaluateConnectedSeepageLiveMask(
        node,
        reference.downwardDistanceMeters,
        runCross.flowRunMeters,
        runCross.crossContourMeters,
        sample.surfaceNormal,
        upstream,
        kWaterSeepageSupportCellSizeMeters);
    sample.effectiveReach = liveMask.effectiveRun;
    sample.effectiveHalfWidth =
        liveMask.effectiveHalfWidth;
    if (liveMask.mask <= 1.0e-6F) {
        return sample;
    }
    const bool normalValid = IsValidPoint(pointNormal) &&
                             glm::dot(pointNormal, pointNormal) > kNormalEpsilon;
    const glm::vec3 resolvedNormal = normalValid
                                         ? glm::normalize(pointNormal)
                                         : sample.surfaceNormal;
    const float normalAgreement = std::abs(glm::dot(resolvedNormal, sample.surfaceNormal));
    const float aligned = SmoothStep(0.15F, 0.85F, normalAgreement);
    const float normalMask = std::lerp(1.0F, aligned, Clamp01(node.normalAlignment));
    const float confidenceMask = std::lerp(0.65F, 1.0F, Clamp01(metadata.confidence));
    sample.mask = Clamp01(
        liveMask.mask * normalMask * confidenceMask);
    return sample;
}

}  // namespace

WaterSeepageRuntimeContribution EvaluateWaterSeepageRuntimeContribution(
    const WaterSeepageRuntimeNode& node,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext) {
    // Threshold matches the GPU gate in EvaluateSeepageContribution so a
    // parameter fading through zero cuts off at the same frame on both paths.
    if (!IsValidPoint(ToGlm(position)) || node.enabledFactor <= 1.0e-5F ||
        node.reachMeters <= 1.0e-5F || node.widthMeters <= 1.0e-5F ||
        node.prominence <= 1.0e-5F || node.strength <= 1.0e-5F ||
        node.effectiveActivity <= 1.0e-5F) {
        return {};
    }
    glm::vec3 pointNormal = ToGlm(normal);
    if (!IsValidPoint(pointNormal) || glm::dot(pointNormal, pointNormal) <= kNormalEpsilon) {
        pointNormal = glm::vec3{0.0F};
    } else {
        pointNormal = glm::normalize(pointNormal);
    }
    return EvaluateWaterSeepageRuntimeContributionWithFan(
        node,
        position,
        pointNormal,
        EvaluateSeepageFanMask(node, ToGlm(position), pointNormal),
        timeSeconds,
        viewContext);
}

WaterSeepageRuntimeContribution EvaluateWaterSeepageGridContribution(
    const WaterSeepageSpatialGrid& grid,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext) {
    WaterSeepageRuntimeContribution result;
    if (grid.nodes.empty()) {
        return result;
    }

    glm::vec3 pointNormal = ToGlm(normal);
    if (!IsValidPoint(pointNormal) || glm::dot(pointNormal, pointNormal) <= kNormalEpsilon) {
        pointNormal = glm::vec3{0.0F};
    } else {
        pointNormal = glm::normalize(pointNormal);
    }

    if (!grid.supportHashCells.empty() &&
        SeepageBoundsContain(grid.supportUnionBounds, position)) {
        const auto supportX = SeepageCellCoordinate(position.x, grid.supportCellSizeMeters);
        const auto supportY = SeepageCellCoordinate(position.y, grid.supportCellSizeMeters);
        const auto supportZ = SeepageCellCoordinate(position.z, grid.supportCellSizeMeters);
        const auto* supportCell = FindSeepageHashCell(
            grid.supportHashCells,
            supportX,
            supportY,
            supportZ);
        if (supportCell != nullptr &&
            supportCell->referenceOffset < grid.supportReferences.size()) {
            const std::size_t supportEnd = std::min<std::size_t>(
                grid.supportReferences.size(),
                static_cast<std::size_t>(supportCell->referenceOffset) +
                    supportCell->referenceCount);
            for (std::size_t referenceIndex = supportCell->referenceOffset;
                 referenceIndex < supportEnd;
                 ++referenceIndex) {
                const auto& reference = grid.supportReferences[referenceIndex];
                if (reference.nodeIndex >= grid.nodes.size()) {
                    continue;
                }
                const auto& node = grid.nodes[reference.nodeIndex];
                if (node.enabledFactor <= 1.0e-5F || node.reachMeters <= 1.0e-5F ||
                    node.widthMeters <= 1.0e-5F || node.prominence <= 1.0e-5F ||
                    node.strength <= 1.0e-5F || node.effectiveActivity <= 1.0e-5F) {
                    continue;
                }
                const auto contribution = EvaluateWaterSeepageRuntimeContributionWithFan(
                    node,
                    position,
                    pointNormal,
                    EvaluateConnectedSeepageSupportMask(
                        node,
                        reference,
                        position,
                        pointNormal),
                    timeSeconds,
                    viewContext);
                BlendSeepageContribution(&result, contribution, node.look.blendMode);
            }
        }
    }

    if (grid.supportHashCells.empty() && !grid.hashCells.empty() &&
        SeepageBoundsContain(grid.unionBounds, position)) {
        const auto x = SeepageCellCoordinate(position.x, grid.cellSizeMeters);
        const auto y = SeepageCellCoordinate(position.y, grid.cellSizeMeters);
        const auto z = SeepageCellCoordinate(position.z, grid.cellSizeMeters);
        const auto* cell = FindSeepageHashCell(grid, x, y, z);
        if (cell != nullptr && cell->referenceOffset < grid.nodeReferences.size()) {
            const std::size_t end = std::min<std::size_t>(
                grid.nodeReferences.size(),
                static_cast<std::size_t>(cell->referenceOffset) + cell->referenceCount);
            for (std::size_t referenceIndex = cell->referenceOffset;
                 referenceIndex < end;
                 ++referenceIndex) {
                const auto nodeIndex = grid.nodeReferences[referenceIndex];
                if (nodeIndex >= grid.nodes.size() ||
                    grid.nodes[nodeIndex].usesConnectedSupport) {
                    continue;
                }
                const auto contribution = EvaluateWaterSeepageRuntimeContribution(
                    grid.nodes[nodeIndex],
                    position,
                    normal,
                    timeSeconds,
                    viewContext);
                BlendSeepageContribution(
                    &result,
                    contribution,
                    grid.nodes[nodeIndex].look.blendMode);
            }
        }
    }
    return result;
}

std::string WaterSeepageAuthoredTopologyFingerprint(
    std::span<const WaterSeepageNode> nodes,
    std::string_view targetSceneRole) {
    const auto normalizedTargetRole = NormalizeSeepageRole(targetSceneRole);
    std::vector<std::string> nodeFingerprints;
    nodeFingerprints.reserve(nodes.size());
    for (const auto& node : nodes) {
        if (!SeepageNodeTargetsRole(node, normalizedTargetRole)) {
            continue;
        }

        std::uint64_t nodeHash = 1469598103934665603ULL;
        SeepageFingerprintU32(&nodeHash, 5U);
        SeepageFingerprintU32(&nodeHash, node.id);
        SeepageFingerprintFloat(&nodeHash, node.position.x);
        SeepageFingerprintFloat(&nodeHash, node.position.y);
        SeepageFingerprintFloat(&nodeHash, node.position.z);
        SeepageFingerprintFloat(&nodeHash, node.surfaceNormal.x);
        SeepageFingerprintFloat(&nodeHash, node.surfaceNormal.y);
        SeepageFingerprintFloat(&nodeHash, node.surfaceNormal.z);
        SeepageFingerprintFloat(&nodeHash, node.downAxis.x);
        SeepageFingerprintFloat(&nodeHash, node.downAxis.y);
        SeepageFingerprintFloat(&nodeHash, node.downAxis.z);
        SeepageFingerprintFloat(&nodeHash, node.selectionReachLimitMeters);
        SeepageFingerprintFloat(&nodeHash, node.selectionWidthLimitMeters);
        SeepageFingerprintFloat(&nodeHash, node.edgeFeatherMeters);
        SeepageFingerprintFloat(&nodeHash, node.depthToleranceMeters);
        nodeFingerprints.push_back(SeepageFingerprintString(nodeHash));
    }
    std::sort(nodeFingerprints.begin(), nodeFingerprints.end());

    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 5U);
    SeepageFingerprintText(&hash, normalizedTargetRole);
    SeepageFingerprintU32(
        &hash,
        static_cast<std::uint32_t>(nodeFingerprints.size()));
    for (const auto& fingerprint : nodeFingerprints) {
        SeepageFingerprintText(&hash, fingerprint);
    }
    return "water-seepage-authored-topology-v5-" + SeepageFingerprintString(hash);
}

namespace {

// The GPU evaluator rejects a node only when its composed contribution
// scale falls at or below 1e-5 after multiplying pattern signals (<= 1.38),
// intensity (<= 8), prominence (<= 8), and rain boost (<= 1.65). For the
// CPU predicates to be at least as permissive as the shader — a false here
// must imply the shader draws nothing — each individually-gated factor uses
// a threshold small enough that even against every other factor at its
// clamp ceiling the composed scale stays under the shader's cutoff
// (5e-7 * 8 * 1.65 * 1.38 ~= 9.1e-6 < 1e-5).
constexpr float kSeepageActiveThreshold = 5.0e-7F;

bool SeepageNodeGatesPass(const WaterSeepageRuntimeNode& node) {
    return node.enabledFactor > kSeepageActiveThreshold &&
           node.reachMeters > kSeepageActiveThreshold &&
           node.widthMeters > kSeepageActiveThreshold &&
           node.prominence > kSeepageActiveThreshold &&
           node.strength > kSeepageActiveThreshold &&
           node.effectiveActivity > kSeepageActiveThreshold;
}

bool SeepageLookCanContribute(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageLookSettings& look) {
    return look.response.intensity > kSeepageActiveThreshold &&
           (look.pattern != WaterSeepagePattern::WettingTrickle ||
            node.wettingProgress > kSeepageActiveThreshold);
}

template <typename LookPredicate>
bool SeepageGridHasQualifyingEffect(
    const WaterSeepageSpatialGrid& grid,
    LookPredicate&& lookQualifies) {
    return std::any_of(
        grid.nodes.begin(),
        grid.nodes.end(),
        [&](const WaterSeepageRuntimeNode& node) {
            if (!SeepageNodeGatesPass(node)) {
                return false;
            }
            if (!node.transitionLook.has_value() ||
                node.transitionAmount <= kSeepageActiveThreshold) {
                return lookQualifies(node, node.look);
            }
            if (node.transitionAmount >= 1.0F - kSeepageActiveThreshold) {
                return lookQualifies(node, node.transitionLook.value());
            }
            return lookQualifies(node, node.look) ||
                   lookQualifies(node, node.transitionLook.value());
        });
}

}  // namespace

bool WaterSeepageGridHasActiveViewportEffect(
    const WaterSeepageSpatialGrid& grid) {
    return SeepageGridHasQualifyingEffect(
        grid,
        [](const WaterSeepageRuntimeNode& node,
           const WaterSeepageLookSettings& look) {
            return SeepageLookCanContribute(node, look);
        });
}

bool WaterSeepageLookIsTimeAnimating(const WaterSeepageLookSettings& look) {
    // Non-finite motion parameters are mapped to animating defaults by
    // SanitizeSeepageLook before they reach the GPU; treat them as animating
    // so an unsanitized look can never freeze a moving effect.
    const auto animates = [](float value) {
        return !std::isfinite(value) || value > 0.0F;
    };
    // These conditions mirror the shader's only time-dependent terms in
    // pointcloud_sparse_ripple.glsl: evolution/drift advection for the
    // organic patterns and front speed (plus irregularity-scaled evolution
    // shift) for Contour Pulses.
    switch (look.pattern) {
        case WaterSeepagePattern::WetRockSheen:
            return animates(look.evolution);
        case WaterSeepagePattern::ChaoticBloom:
        case WaterSeepagePattern::WettingTrickle:
            return animates(look.evolution) ||
                   animates(look.downhillDriftMetersPerSecond);
        case WaterSeepagePattern::ContourPulses:
            return animates(look.pulseSpeedMetersPerSecond) ||
                   (animates(look.evolution) &&
                    animates(look.pulseIrregularity));
    }
    return true;
}

bool WaterSeepageGridHasTimeAnimatingViewportEffect(
    const WaterSeepageSpatialGrid& grid) {
    return SeepageGridHasQualifyingEffect(
        grid,
        [](const WaterSeepageRuntimeNode& node,
           const WaterSeepageLookSettings& look) {
            return SeepageLookCanContribute(node, look) &&
                   WaterSeepageLookIsTimeAnimating(look);
        });
}

std::string WaterSeepageTopologyFingerprint(const WaterSeepageSpatialGrid& grid) {
    std::uint64_t hash = 1469598103934665603ULL;
    // v7 retains the connected support's derived maximum upstream extent,
    // geometrically bounded routes, and station-relative upstream width.
    SeepageFingerprintU32(&hash, 7U);
    SeepageFingerprintFloat(&hash, grid.cellSizeMeters);
    SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(grid.nodes.size()));
    for (const auto& node : grid.nodes) {
        SeepageFingerprintU32(&hash, node.id);
        SeepageFingerprintFloat(&hash, node.position.x);
        SeepageFingerprintFloat(&hash, node.position.y);
        SeepageFingerprintFloat(&hash, node.position.z);
        SeepageFingerprintFloat(&hash, node.surfaceNormal.x);
        SeepageFingerprintFloat(&hash, node.surfaceNormal.y);
        SeepageFingerprintFloat(&hash, node.surfaceNormal.z);
        SeepageFingerprintFloat(&hash, node.downAxis.x);
        SeepageFingerprintFloat(&hash, node.downAxis.y);
        SeepageFingerprintFloat(&hash, node.downAxis.z);
        SeepageFingerprintFloat(&hash, node.lateralAxis.x);
        SeepageFingerprintFloat(&hash, node.lateralAxis.y);
        SeepageFingerprintFloat(&hash, node.lateralAxis.z);
        SeepageFingerprintFloat(&hash, node.selectionReachLimitMeters);
        SeepageFingerprintFloat(&hash, node.selectionWidthLimitMeters);
        SeepageFingerprintFloat(&hash, node.edgeFeatherMeters);
        SeepageFingerprintFloat(&hash, node.depthToleranceMeters);
        SeepageFingerprintFloat(&hash, node.maximumUpstreamRunMeters);
        SeepageFingerprintU32(&hash, node.usesConnectedSupport ? 1U : 0U);
        if (!node.usesConnectedSupport) {
            SeepageFingerprintU32(&hash, node.guideSampleCount);
            SeepageFingerprintU32(&hash, node.guideValid ? 1U : 0U);
            SeepageFingerprintU32(&hash, node.guideComplete ? 1U : 0U);
            SeepageFingerprintFloat(&hash, node.guideRequestedReachMeters);
            SeepageFingerprintFloat(&hash, node.guideAchievedReachMeters);
            for (std::size_t sampleIndex = 0U;
                 sampleIndex < std::min<std::size_t>(
                     node.guideSampleCount,
                     kWaterSeepageMaximumGuideSamples);
                 ++sampleIndex) {
                const auto& sample = node.guideSamples[sampleIndex];
                SeepageFingerprintFloat(&hash, sample.position.x);
                SeepageFingerprintFloat(&hash, sample.position.y);
                SeepageFingerprintFloat(&hash, sample.position.z);
                SeepageFingerprintFloat(&hash, sample.normal.x);
                SeepageFingerprintFloat(&hash, sample.normal.y);
                SeepageFingerprintFloat(&hash, sample.normal.z);
                SeepageFingerprintFloat(&hash, sample.station);
                SeepageFingerprintFloat(&hash, sample.confidence);
            }
        }
    }
    SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(grid.hashCells.size()));
    for (const auto& cell : grid.hashCells) {
        SeepageFingerprintU32(&hash, cell.occupied ? 1U : 0U);
        if (!cell.occupied) {
            continue;
        }
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.x));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.y));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.z));
        SeepageFingerprintU32(&hash, cell.referenceOffset);
        SeepageFingerprintU32(&hash, cell.referenceCount);
    }
    for (const auto reference : grid.nodeReferences) {
        SeepageFingerprintU32(&hash, reference);
    }
    SeepageFingerprintFloat(&hash, grid.supportCellSizeMeters);
    SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(grid.supportHashCells.size()));
    for (const auto& cell : grid.supportHashCells) {
        SeepageFingerprintU32(&hash, cell.occupied ? 1U : 0U);
        if (!cell.occupied) {
            continue;
        }
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.x));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.y));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.z));
        SeepageFingerprintU32(&hash, cell.referenceOffset);
        SeepageFingerprintU32(&hash, cell.referenceCount);
    }
    for (const auto& reference : grid.supportReferences) {
        SeepageFingerprintU32(&hash, reference.nodeIndex);
        SeepageFingerprintFloat(&hash, reference.downwardDistanceMeters);
        SeepageFingerprintU32(&hash, reference.packedRunCrossMeters);
        SeepageFingerprintU32(&hash, reference.packedNormalRoleConfidenceFlags);
    }
    return "water-seepage-topology-v7-" + SeepageFingerprintString(hash);
}

std::string WaterSeepageParamsFingerprint(const WaterSeepageSpatialGrid& grid) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 8U);
    const auto fingerprintLook = [&](const WaterSeepageLookSettings& look) {
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(look.pattern));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(look.blendMode));
        SeepageFingerprintFloat(&hash, look.baseWetness);
        SeepageFingerprintFloat(&hash, look.density);
        SeepageFingerprintFloat(&hash, look.glisten);
        SeepageFingerprintFloat(&hash, look.featureSizeMeters);
        SeepageFingerprintFloat(&hash, look.contrast);
        SeepageFingerprintFloat(&hash, look.evolution);
        SeepageFingerprintFloat(&hash, look.roughness);
        SeepageFingerprintFloat(&hash, look.angleResponse);
        SeepageFingerprintFloat(&hash, look.microNormalStrength);
        SeepageFingerprintFloat(&hash, look.glintDensity);
        SeepageFingerprintFloat(&hash, look.environmentAzimuthDegrees);
        SeepageFingerprintFloat(&hash, look.environmentElevationDegrees);
        SeepageFingerprintFloat(&hash, look.curl);
        SeepageFingerprintFloat(&hash, look.breakup);
        SeepageFingerprintFloat(&hash, look.downhillDriftMetersPerSecond);
        SeepageFingerprintFloat(&hash, look.tricklePatchSizeMeters);
        SeepageFingerprintFloat(&hash, look.trickleWidthMeters);
        SeepageFingerprintFloat(&hash, look.trickleFrontSoftness);
        SeepageFingerprintFloat(&hash, look.pulseSpacingMeters);
        SeepageFingerprintFloat(&hash, look.pulseWidthMeters);
        SeepageFingerprintFloat(&hash, look.pulseSpeedMetersPerSecond);
        SeepageFingerprintFloat(&hash, look.pulseIrregularity);
        SeepageFingerprintFloat(&hash, look.pulseWaveCount);
        SeepageFingerprintFloat(&hash, look.pulseSpeedVariation);
        SeepageFingerprintFloat(&hash, look.rainResponse);
        SeepageFingerprintFloat(&hash, look.response.intensity);
        SeepageFingerprintFloat(&hash, look.response.emissionAdd);
        SeepageFingerprintFloat(&hash, look.response.opacityAdd);
        SeepageFingerprintFloat(&hash, look.response.opacityMultiply);
        SeepageFingerprintFloat(&hash, look.response.pointSizeAdd);
        SeepageFingerprintFloat(&hash, look.response.pointSizeMultiply);
        SeepageFingerprintFloat(&hash, look.response.colouriseRed);
        SeepageFingerprintFloat(&hash, look.response.colouriseGreen);
        SeepageFingerprintFloat(&hash, look.response.colouriseBlue);
        SeepageFingerprintFloat(&hash, look.response.colouriseAmount);
    };
    SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(grid.nodes.size()));
    for (const auto& node : grid.nodes) {
        SeepageFingerprintU32(&hash, node.id);
        SeepageFingerprintU32(&hash, node.seed);
        SeepageFingerprintFloat(&hash, node.reachMeters);
        SeepageFingerprintFloat(&hash, node.widthMeters);
        SeepageFingerprintFloat(&hash, node.prominence);
        SeepageFingerprintFloat(&hash, node.startHalfWidthMeters);
        SeepageFingerprintFloat(&hash, node.edgeFeatherMeters);
        SeepageFingerprintFloat(&hash, node.depthToleranceMeters);
        SeepageFingerprintFloat(&hash, node.normalAlignment);
        SeepageFingerprintFloat(&hash, node.strength);
        SeepageFingerprintFloat(&hash, node.budgetMeters);
        SeepageFingerprintFloat(&hash, node.rainVisualStrength);
        SeepageFingerprintFloat(&hash, node.scenarioSpread);
        SeepageFingerprintFloat(&hash, node.effectiveActivity);
        SeepageFingerprintFloat(&hash, node.enabledFactor);
        SeepageFingerprintFloat(&hash, node.localSpread);
        SeepageFingerprintFloat(&hash, node.wettingProgress);
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(node.resolvedQuality));
        fingerprintLook(node.look);
        SeepageFingerprintFloat(&hash, node.transitionAmount);
        SeepageFingerprintU32(&hash, node.transitionLook.has_value() ? 1U : 0U);
        if (node.transitionLook.has_value()) {
            fingerprintLook(node.transitionLook.value());
        }
        const bool hasContourPulseField =
            node.look.pattern == WaterSeepagePattern::ContourPulses ||
            (node.transitionLook.has_value() &&
             node.transitionLook->pattern ==
                 WaterSeepagePattern::ContourPulses);
        if (hasContourPulseField) {
            SeepageFingerprintFloat(
                &hash,
                node.pulseStableSpanMeters);
            SeepageFingerprintU32(
                &hash,
                node.pulseField.sampleCount);
            SeepageFingerprintU32(
                &hash,
                node.transitionPulseField.sampleCount);
        }
    }
    return "water-seepage-params-v7-" + SeepageFingerprintString(hash);
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
           left.attractorEnabled == right.attractorEnabled &&
           left.attractorPosition.x == right.attractorPosition.x &&
           left.attractorPosition.y == right.attractorPosition.y &&
           left.attractorPosition.z == right.attractorPosition.z &&
           left.attractorStrength == right.attractorStrength &&
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

void AppendCombinedSupportPoint(
    invisible_places::io::LoadedPointCloud* combined,
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t pointIndex) {
    if (combined == nullptr || pointIndex >= cloud.positions.size()) {
        return;
    }

    const auto& position = cloud.positions[pointIndex];
    combined->positions.push_back(position);
    combined->bounds.Expand(position);
    if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
        combined->normals.push_back(cloud.normals[pointIndex]);
    } else {
        combined->normals.push_back({0.0F, 0.0F, 1.0F});
    }
    if (cloud.hasSourceRgb && pointIndex < cloud.packedColors.size()) {
        combined->packedColors.push_back(cloud.packedColors[pointIndex]);
    } else {
        combined->packedColors.push_back(0xFFFFFFFFU);
    }
}

invisible_places::io::LoadedPointCloud BuildCombinedWaterSupportCloud(
    std::span<const WaterSceneSupportLayer> layers,
    const WaterPathGenerationSettings& settings) {
    return BuildCombinedWaterSupportCloud(
        layers,
        settings,
        std::span<const invisible_places::io::Float3>{});
}

invisible_places::io::LoadedPointCloud BuildCombinedWaterSupportCloud(
    std::span<const WaterSceneSupportLayer> layers,
    const WaterPathGenerationSettings& settings,
    std::span<const invisible_places::io::Float3> priorityPoints) {
    invisible_places::io::LoadedPointCloud combined;
    combined.sourcePath = "combined-water-support";
    combined.layerName = "Combined Water Support";

    std::vector<const WaterSceneSupportLayer*> validLayers;
    validLayers.reserve(layers.size());
    double totalWeight = 0.0;
    for (const auto& layer : layers) {
        if (layer.cloud == nullptr || layer.cloud->positions.empty()) {
            continue;
        }
        const double multiplier = std::max(1.0, static_cast<double>(layer.samplingMultiplier));
        const double weight = static_cast<double>(layer.cloud->positions.size()) / multiplier;
        if (weight <= 0.0) {
            continue;
        }
        validLayers.push_back(&layer);
        totalWeight += weight;
    }
    if (validLayers.empty() || totalWeight <= 0.0) {
        return combined;
    }

    const std::size_t sampleLimit = static_cast<std::size_t>(
        std::max<std::uint32_t>(1U, settings.supportSampleLimit));
    combined.positions.reserve(sampleLimit);
    combined.normals.reserve(sampleLimit);
    combined.packedColors.reserve(sampleLimit);

    std::unordered_set<GridKey, GridKeyHash> occupiedVoxels;
    occupiedVoxels.reserve(sampleLimit * 2U);
    const float baseVoxelSize = std::max(0.001F, settings.supportVoxelSize);

    auto appendIfVoxelFree =
        [&](std::unordered_set<GridKey, GridKeyHash>* occupied,
            const invisible_places::io::LoadedPointCloud& cloud,
            std::size_t pointIndex,
            float voxelSize) {
            if (occupied == nullptr || pointIndex >= cloud.positions.size() || combined.positions.size() >= sampleLimit) {
                return false;
            }
            const auto& position = cloud.positions[pointIndex];
            const float invVoxel = 1.0F / std::max(0.001F, voxelSize);
            const GridKey key{
                static_cast<int>(std::floor(position.x * invVoxel)),
                static_cast<int>(std::floor(position.y * invVoxel)),
                static_cast<int>(std::floor(position.z * invVoxel)),
            };
            if (!occupied->insert(key).second) {
                return false;
            }
            AppendCombinedSupportPoint(&combined, cloud, pointIndex);
            return true;
        };

    if (!priorityPoints.empty()) {
        std::unordered_set<GridKey, GridKeyHash> occupiedPriorityVoxels;
        occupiedPriorityVoxels.reserve(std::min<std::size_t>(sampleLimit, 262144U));
        const float priorityVoxelSize = std::max(
            0.001F,
            std::min({
                baseVoxelSize,
                std::max(0.001F, settings.pathSampleSpacing),
                0.010F,
            }));
        const bool aerialScale = settings.legacyScaleMode == WaterScaleMode::Aerial;
        const float priorityRadius = aerialScale
                                         ? std::clamp(settings.maxBridgeDistance * 2.5F, 0.25F, 30.0F)
                                         : std::clamp(
                                               std::max({
                                                   settings.maxBridgeDistance * 6.0F,
                                                   baseVoxelSize * 32.0F,
                                                   std::max(0.001F, settings.pathSampleSpacing) * 32.0F,
                                                   0.06F,
                                               }),
                                               0.06F,
                                               1.25F);
        const float priorityRadiusSquared = priorityRadius * priorityRadius;
        const std::size_t prioritySampleLimit = std::min<std::size_t>(
            sampleLimit,
            std::max<std::size_t>(1U, (sampleLimit * 3U) / 5U));
        const std::size_t priorityLayerTarget = std::max<std::size_t>(
            1U,
            prioritySampleLimit / std::max<std::size_t>(1U, priorityPoints.size() * validLayers.size()));
        std::size_t prioritySamples = 0U;

        for (const auto& priorityPoint : priorityPoints) {
            if (prioritySamples >= prioritySampleLimit || combined.positions.size() >= sampleLimit) {
                break;
            }
            const glm::vec3 priority = ToGlm(priorityPoint);
            for (const auto* layer : validLayers) {
                if (prioritySamples >= prioritySampleLimit || combined.positions.size() >= sampleLimit) {
                    break;
                }
                const auto& cloud = *layer->cloud;
                std::size_t layerPrioritySamples = 0U;
                for (std::size_t pointIndex = 0; pointIndex < cloud.positions.size(); ++pointIndex) {
                    if (prioritySamples >= prioritySampleLimit ||
                        layerPrioritySamples >= priorityLayerTarget ||
                        combined.positions.size() >= sampleLimit) {
                        break;
                    }
                    const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
                    const glm::vec3 delta = position - priority;
                    if (std::abs(delta.x) > priorityRadius ||
                        std::abs(delta.y) > priorityRadius ||
                        std::abs(delta.z) > priorityRadius ||
                        glm::dot(delta, delta) > priorityRadiusSquared) {
                        continue;
                    }
                    if (appendIfVoxelFree(&occupiedPriorityVoxels, cloud, pointIndex, priorityVoxelSize)) {
                        ++prioritySamples;
                        ++layerPrioritySamples;
                    }
                }
            }
        }
    }

    for (const auto* layer : validLayers) {
        const auto& cloud = *layer->cloud;
        const double multiplier = std::max(1.0, static_cast<double>(layer->samplingMultiplier));
        const double layerWeight = static_cast<double>(cloud.positions.size()) / multiplier;
        const std::size_t layerTarget = std::max<std::size_t>(
            1U,
            static_cast<std::size_t>(
                std::ceil((layerWeight / totalWeight) * static_cast<double>(sampleLimit))));
        const std::size_t stride = std::max<std::size_t>(
            1U,
            cloud.positions.size() / std::max<std::size_t>(1U, layerTarget * 4U));
        const float roleVoxelSize = std::max(
            baseVoxelSize * static_cast<float>(multiplier),
            std::max(0.0F, layer->pointSpacingMeters));

        std::size_t layerSamples = 0U;
        for (std::size_t pointIndex = 0; pointIndex < cloud.positions.size(); pointIndex += stride) {
            if (combined.positions.size() >= sampleLimit || layerSamples >= layerTarget) {
                break;
            }
            if (!appendIfVoxelFree(&occupiedVoxels, cloud, pointIndex, roleVoxelSize)) {
                continue;
            }
            ++layerSamples;
        }
    }

    combined.hasNormals = combined.normals.size() == combined.positions.size();
    combined.hasSourceRgb = combined.packedColors.size() == combined.positions.size();
    if (combined.bounds.valid) {
        combined.focusPoint = {
            0.5F * (combined.bounds.minimum.x + combined.bounds.maximum.x),
            0.5F * (combined.bounds.minimum.y + combined.bounds.maximum.y),
            0.5F * (combined.bounds.minimum.z + combined.bounds.maximum.z),
        };
        combined.hasFocusPoint = true;
    }
    return combined;
}

namespace {

struct SeepageGuideSupportGraph {
    SupportGraph graph;
    float pointSpacingMeters = 0.02F;
};

SeepageGuideSupportGraph BuildSeepageGuideSupportGraph(
    std::span<const WaterSceneSupportLayer> layers,
    std::string_view requestedRole,
    std::size_t sampleLimit,
    bool includeAllRoles = false) {
    SeepageGuideSupportGraph result;
    const auto normalizedRole = NormalizeSeepageRole(requestedRole);
    std::vector<const WaterSceneSupportLayer*> selectedLayers;
    selectedLayers.reserve(layers.size());
    std::uint64_t totalPointCount = 0U;
    float minimumSpacing = std::numeric_limits<float>::max();
    for (const auto& layer : layers) {
        if (layer.cloud == nullptr || layer.cloud->positions.empty()) {
            continue;
        }
        const auto layerRole = NormalizeSeepageRole(layer.role);
        if (includeAllRoles) {
            const bool unnamedOrGeneric =
                layerRole.empty() ||
                layerRole == "point" ||
                layerRole == "points" ||
                layerRole == "pointcloud" ||
                layerRole == "unknown";
            if (!unnamedOrGeneric) {
                continue;
            }
        } else if (layerRole != normalizedRole) {
            continue;
        }
        selectedLayers.push_back(&layer);
        totalPointCount += static_cast<std::uint64_t>(layer.cloud->positions.size());
        if (std::isfinite(layer.pointSpacingMeters) && layer.pointSpacingMeters > 0.0F) {
            minimumSpacing = std::min(minimumSpacing, layer.pointSpacingMeters);
        }
    }
    if (selectedLayers.empty() || totalPointCount == 0U) {
        return result;
    }

    const std::size_t safeSampleLimit = std::clamp<std::size_t>(sampleLimit, 64U, 450'000U);
    result.pointSpacingMeters =
        minimumSpacing < std::numeric_limits<float>::max()
            ? std::clamp(minimumSpacing, 0.001F, 0.25F)
            : 0.02F;
    result.graph.cellSize = std::clamp(
        std::max(0.01F, result.pointSpacingMeters * 8.0F),
        0.01F,
        0.25F);
    result.graph.points.reserve(std::min<std::size_t>(
        safeSampleLimit,
        static_cast<std::size_t>(std::min<std::uint64_t>(
            totalPointCount,
            static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())))));

    for (const auto* layer : selectedLayers) {
        if (result.graph.points.size() >= safeSampleLimit) {
            break;
        }
        const auto& cloud = *layer->cloud;
        const std::size_t remainingBudget = safeSampleLimit - result.graph.points.size();
        const auto proportionalTarget = static_cast<std::size_t>(std::max<double>(
            1.0,
            std::floor(
                static_cast<double>(safeSampleLimit) *
                (static_cast<double>(cloud.positions.size()) /
                 static_cast<double>(totalPointCount)))));
        const std::size_t layerTarget = std::min({
            remainingBudget,
            cloud.positions.size(),
            proportionalTarget,
        });
        if (layerTarget == 0U) {
            continue;
        }
        const std::size_t stride = std::max<std::size_t>(
            1U,
            (cloud.positions.size() + layerTarget - 1U) / layerTarget);
        const auto neighbourSlot = FindScalarFieldSlot(cloud, "Number_of_neighbors");
        std::size_t sampledFromLayer = 0U;
        for (std::size_t pointIndex = 0U;
             pointIndex < cloud.positions.size() &&
             sampledFromLayer < layerTarget &&
             result.graph.points.size() < safeSampleLimit;
             pointIndex += stride) {
            const glm::vec3 position = ToGlm(cloud.positions[pointIndex]);
            if (!IsValidPoint(position)) {
                continue;
            }
            SupportPoint support;
            support.sourceIndex = static_cast<std::uint32_t>(std::min<std::size_t>(
                pointIndex,
                static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
            support.position = position;
            support.confidence = SupportConfidence(cloud, pointIndex, neighbourSlot);
            if (cloud.hasNormals && pointIndex < cloud.normals.size()) {
                const glm::vec3 normal = ToGlm(cloud.normals[pointIndex]);
                if (IsValidPoint(normal) && glm::dot(normal, normal) > kNormalEpsilon) {
                    support.normal = glm::normalize(normal);
                    support.hasNormal = true;
                }
            }
            const auto supportIndex = static_cast<std::uint32_t>(result.graph.points.size());
            result.graph.points.push_back(support);
            result.graph.grid[MakeGridKey(position, result.graph.cellSize)].push_back(supportIndex);
            ++sampledFromLayer;
        }
    }
    return result;
}

float SeepageGuideStartSearchRadius(
    const WaterSeepageNode& node,
    const SeepageGuideSupportGraph& support) {
    return std::clamp(
        std::max({
            support.pointSpacingMeters * 14.0F,
            std::max(0.0F, node.depthToleranceMeters) + std::max(0.0F, node.edgeFeatherMeters),
            0.06F,
        }),
        0.03F,
        0.50F);
}

std::optional<std::uint32_t> FindSeepageGuideStartIndex(
    const SeepageGuideSupportGraph& support,
    const WaterSeepageNode& node) {
    if (support.graph.points.empty() || !IsValidPoint(ToGlm(node.position))) {
        return std::nullopt;
    }
    const glm::vec3 nodePosition = ToGlm(node.position);
    const glm::vec3 nodeNormal = SafeSeepageNormal(node.surfaceNormal);
    const float searchRadius = SeepageGuideStartSearchRadius(node, support);
    const float searchRadiusSquared = searchRadius * searchRadius;
    const auto candidates = NearbySupportIndices(
        support.graph,
        nodePosition,
        searchRadius);
    std::optional<std::uint32_t> bestIndex;
    float bestScore = std::numeric_limits<float>::max();
    for (const auto candidateIndex : candidates) {
        if (candidateIndex >= support.graph.points.size()) {
            continue;
        }
        const auto& candidate = support.graph.points[candidateIndex];
        const glm::vec3 delta = candidate.position - nodePosition;
        const float distanceSquared = glm::dot(delta, delta);
        if (distanceSquared > searchRadiusSquared ||
            candidate.position.z > nodePosition.z + std::max(0.004F, node.edgeFeatherMeters)) {
            continue;
        }
        const float normalAgreement = candidate.hasNormal
                                          ? std::abs(glm::dot(candidate.normal, nodeNormal))
                                          : 0.55F;
        if (candidate.hasNormal && normalAgreement < 0.20F) {
            continue;
        }
        const float score = distanceSquared +
                            (1.0F - Clamp01(normalAgreement)) *
                                searchRadiusSquared * 0.35F;
        if (!bestIndex.has_value() || score < bestScore) {
            bestIndex = candidateIndex;
            bestScore = score;
        }
    }
    return bestIndex;
}

bool SeepageGuideGraphSupportsNode(
    const SeepageGuideSupportGraph& support,
    const WaterSeepageNode& node) {
    if (support.graph.points.empty() || !IsValidPoint(ToGlm(node.position))) {
        return false;
    }
    return FindSeepageGuideStartIndex(support, node).has_value();
}

WaterSeepageSurfaceGuide TraceWaterSeepageSurfaceGuide(
    const WaterSeepageNode& node,
    const SeepageGuideSupportGraph& support) {
    WaterSeepageSurfaceGuide guide;
    guide.nodeId = node.id;
    guide.requestedReachMeters = std::clamp(
        SeepageFiniteOr(node.reachMeters, 1.25F),
        0.001F,
        1000.0F) * kSeepageMaximumReachScale;
    if (!SeepageGuideGraphSupportsNode(support, node)) {
        return guide;
    }

    const glm::vec3 nodePosition = ToGlm(node.position);
    const glm::vec3 nodeNormal = SafeSeepageNormal(node.surfaceNormal);
    const auto startIndex = FindSeepageGuideStartIndex(support, node);
    if (!startIndex.has_value() || startIndex.value() >= support.graph.points.size()) {
        return guide;
    }

    WaterPathGenerationSettings routeSettings;
    routeSettings.autoTune = false;
    routeSettings.supportVoxelSize = std::clamp(
        support.pointSpacingMeters * 1.5F,
        0.003F,
        0.08F);
    routeSettings.maxBridgeDistance = std::clamp(
        std::max({
            support.pointSpacingMeters * 5.0F,
            std::min(std::max(0.0F, node.depthToleranceMeters) * 0.35F, 0.08F),
            guide.requestedReachMeters / 48.0F,
            0.020F,
        }),
        0.020F,
        0.12F);
    routeSettings.pathLength = guide.requestedReachMeters;
    routeSettings.pathSampleSpacing = std::clamp(
        std::max(support.pointSpacingMeters * 3.0F, routeSettings.maxBridgeDistance * 0.22F),
        0.005F,
        0.10F);
    routeSettings.branching = 0.0F;
    routeSettings.coverage = 0.0F;
    routeSettings.gapTolerance = 0.40F;
    routeSettings.maxSteps = 256U;
    routeSettings.supportSampleLimit = static_cast<std::uint32_t>(std::min<std::size_t>(
        support.graph.points.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));

    std::vector<WaterSeepageGuideSample> rawSamples;
    rawSamples.reserve(64U);
    glm::vec3 previousNormal = SafeSeepageNormal(node.surfaceNormal);
    rawSamples.push_back({
        .position = node.position,
        .normal = FromGlm(previousNormal),
        .station = 0.0F,
        .confidence = 1.0F,
    });
    std::vector<std::uint32_t> visited;
    visited.reserve(routeSettings.maxSteps);
    std::uint32_t currentIndex = startIndex.value();
    visited.push_back(currentIndex);
    glm::vec3 previousCenter = nodePosition;
    float previousZ = nodePosition.z;
    float station = 0.0F;
    glm::vec3 previousDirection = ToGlm(node.downAxis);
    if (!IsValidPoint(previousDirection) || glm::dot(previousDirection, previousDirection) <= kNormalEpsilon) {
        previousDirection = kGravity;
    } else {
        previousDirection = glm::normalize(previousDirection);
    }
    const float riseEpsilon = std::clamp(
        support.pointSpacingMeters * 0.35F,
        0.0005F,
        0.004F);

    for (std::uint32_t step = 0U;
         step < routeSettings.maxSteps && station < guide.requestedReachMeters;
         ++step) {
        const auto ranked = RankDownhillNeighbours(
            support.graph,
            currentIndex,
            routeSettings,
            visited);
        if (ranked.empty()) {
            break;
        }

        std::optional<RankedNeighbour> selected;
        float selectedScore = -std::numeric_limits<float>::max();
        const auto& current = support.graph.points[currentIndex];
        glm::vec3 currentNormal = current.hasNormal
                                      ? current.normal
                                      : previousNormal;
        if (glm::dot(currentNormal, previousNormal) < 0.0F) {
            currentNormal = -currentNormal;
        }
        glm::vec3 desiredDirection = kGravity - currentNormal * glm::dot(kGravity, currentNormal);
        if (glm::dot(desiredDirection, desiredDirection) <= kNormalEpsilon) {
            desiredDirection = previousDirection -
                               currentNormal * glm::dot(previousDirection, currentNormal);
        }
        if (glm::dot(desiredDirection, desiredDirection) <= kNormalEpsilon) {
            desiredDirection = previousDirection;
        } else {
            desiredDirection = glm::normalize(desiredDirection);
        }
        for (const auto& candidate : ranked) {
            if (candidate.supportIndex >= support.graph.points.size()) {
                continue;
            }
            const auto& next = support.graph.points[candidate.supportIndex];
            if (next.position.z > previousZ + riseEpsilon ||
                next.position.z > nodePosition.z + riseEpsilon) {
                continue;
            }
            const glm::vec3 fromCurrent = next.position - current.position;
            const float distance = glm::length(fromCurrent);
            if (distance <= 1.0e-5F) {
                continue;
            }
            const float planeDistance = std::abs(glm::dot(fromCurrent, currentNormal));
            const float planeTolerance = std::max(
                support.pointSpacingMeters * 5.0F,
                distance * 0.55F);
            if (planeDistance > planeTolerance) {
                continue;
            }
            const float directionalAlignment = glm::dot(
                fromCurrent / distance,
                desiredDirection);
            const float routeScore = candidate.score + directionalAlignment * 1.75F;
            if (!selected.has_value() || routeScore > selectedScore) {
                selected = candidate;
                selectedScore = routeScore;
            }
        }
        if (!selected.has_value()) {
            break;
        }

        currentIndex = selected->supportIndex;
        visited.push_back(currentIndex);
        const auto& next = support.graph.points[currentIndex];
        glm::vec3 nextNormal = next.hasNormal ? next.normal : previousNormal;
        if (glm::dot(nextNormal, previousNormal) < 0.0F) {
            nextNormal = -nextNormal;
        }
        if (glm::dot(nextNormal, nextNormal) <= kNormalEpsilon) {
            nextNormal = previousNormal;
        } else {
            nextNormal = glm::normalize(nextNormal);
        }

        const glm::vec3 segment = next.position - previousCenter;
        const float segmentLength = glm::length(segment);
        if (segmentLength <= 1.0e-5F) {
            continue;
        }
        const float remaining = guide.requestedReachMeters - station;
        if (segmentLength >= remaining) {
            const float amount = remaining / segmentLength;
            glm::vec3 endNormal = glm::mix(previousNormal, nextNormal, amount);
            endNormal = glm::dot(endNormal, endNormal) > kNormalEpsilon
                            ? glm::normalize(endNormal)
                            : previousNormal;
            rawSamples.push_back({
                .position = FromGlm(previousCenter + segment * amount),
                .normal = FromGlm(endNormal),
                .station = guide.requestedReachMeters,
                .confidence = std::clamp(next.confidence, 0.0F, 1.0F),
            });
            station = guide.requestedReachMeters;
            previousCenter = ToGlm(rawSamples.back().position);
            previousNormal = endNormal;
            previousZ = previousCenter.z;
            break;
        }

        station += segmentLength;
        rawSamples.push_back({
            .position = FromGlm(next.position),
            .normal = FromGlm(nextNormal),
            .station = station,
            .confidence = std::clamp(next.confidence, 0.0F, 1.0F),
        });
        previousCenter = next.position;
        previousDirection = segment / segmentLength;
        previousNormal = nextNormal;
        previousZ = next.position.z;
    }

    guide.achievedReachMeters = station;
    guide.complete = station + std::max(0.001F, support.pointSpacingMeters) >=
                     guide.requestedReachMeters;
    if (rawSamples.size() < 2U ||
        guide.achievedReachMeters <= std::max(0.005F, support.pointSpacingMeters * 0.25F)) {
        return guide;
    }

    const std::size_t outputCount = std::min<std::size_t>(
        kWaterSeepageMaximumGuideSamples,
        std::max<std::size_t>(2U, rawSamples.size()));
    std::size_t rawSegment = 0U;
    glm::vec3 outputPreviousNormal = SafeSeepageNormal(rawSamples.front().normal);
    for (std::size_t outputIndex = 0U; outputIndex < outputCount; ++outputIndex) {
        const float targetStation =
            outputIndex + 1U == outputCount
                ? guide.achievedReachMeters
                : guide.achievedReachMeters *
                      (static_cast<float>(outputIndex) /
                       static_cast<float>(outputCount - 1U));
        while (rawSegment + 1U < rawSamples.size() &&
               rawSamples[rawSegment + 1U].station < targetStation) {
            ++rawSegment;
        }
        const auto& left = rawSamples[rawSegment];
        const auto& right = rawSamples[std::min<std::size_t>(rawSegment + 1U, rawSamples.size() - 1U)];
        const float stationSpan = std::max(1.0e-5F, right.station - left.station);
        const float amount = Clamp01((targetStation - left.station) / stationSpan);
        glm::vec3 leftNormal = SafeSeepageNormal(left.normal);
        glm::vec3 rightNormal = SafeSeepageNormal(right.normal);
        if (glm::dot(leftNormal, rightNormal) < 0.0F) {
            rightNormal = -rightNormal;
        }
        glm::vec3 normal = glm::mix(leftNormal, rightNormal, amount);
        normal = glm::dot(normal, normal) > kNormalEpsilon
                     ? glm::normalize(normal)
                     : outputPreviousNormal;
        if (glm::dot(normal, outputPreviousNormal) < 0.0F) {
            normal = -normal;
        }
        const glm::vec3 position = glm::mix(
            ToGlm(left.position),
            ToGlm(right.position),
            amount);
        guide.samples[outputIndex] = {
            .position = FromGlm(position),
            .normal = FromGlm(normal),
            .station = targetStation,
            .confidence = std::lerp(left.confidence, right.confidence, amount),
        };
        outputPreviousNormal = normal;
    }
    guide.sampleCount = static_cast<std::uint32_t>(outputCount);
    guide.valid = true;
    return guide;
}

std::uint32_t SeepageSurfaceGuideRoleMask(const WaterSeepageNode& node) {
    bool prefersRock = false;
    bool allowsSand = false;
    for (const auto& role : node.targetSceneRoles) {
        const auto normalized = NormalizeSeepageRole(role);
        prefersRock = prefersRock || normalized == "rock" || normalized == "veg";
        allowsSand = allowsSand || normalized == "sand";
    }
    if (prefersRock) {
        return kWaterSurfaceRockRoleMask;
    }
    return allowsSand ? kWaterSurfaceSandRoleMask : 0U;
}

WaterSeepageSurfaceGuide TraceWaterSeepageSurfaceCacheGuide(
    const WaterSeepageNode& node,
    const WaterSurfaceCache& cache) {
    WaterSeepageSurfaceGuide guide;
    guide.nodeId = node.id;
    guide.requestedReachMeters = std::clamp(
        SeepageFiniteOr(node.reachMeters, 1.25F),
        0.001F,
        1000.0F) * kSeepageMaximumReachScale;
    const std::uint32_t roleMask = SeepageSurfaceGuideRoleMask(node);
    if (roleMask == 0U || cache.flowSurfaceSurfels.empty() ||
        !std::isfinite(cache.resolutionMeters) || cache.resolutionMeters <= 0.0F ||
        !IsValidPoint(ToGlm(node.position))) {
        return guide;
    }

    const float resolution = std::clamp(cache.resolutionMeters, 0.001F, 1.0F);
    const float startRadius = std::clamp(
        std::max(resolution * 2.25F, std::min(node.depthToleranceMeters, 0.12F)),
        resolution,
        0.25F);
    const auto start = QueryWaterSurfaceCache(
        cache,
        node.position,
        startRadius,
        node.surfaceNormal,
        roleMask);
    if (!start.hit || start.surfel.confidence < 0.10F ||
        start.surfel.normalCoherence < 0.10F) {
        return guide;
    }

    std::vector<WaterSeepageGuideSample> rawSamples;
    rawSamples.reserve(std::min<std::size_t>(
        2050U,
        static_cast<std::size_t>(std::ceil(
            std::min(2048.0F, guide.requestedReachMeters / resolution))) + 2U));
    glm::vec3 currentPosition = ToGlm(node.position);
    glm::vec3 currentNormal = SafeSeepageNormal(start.surfel.normal);
    glm::vec3 nodeNormal = SafeSeepageNormal(node.surfaceNormal);
    if (glm::dot(currentNormal, nodeNormal) < 0.0F) {
        currentNormal = -currentNormal;
    }
    glm::vec3 previousDirection = ToGlm(node.downAxis);
    previousDirection -= currentNormal * glm::dot(previousDirection, currentNormal);
    if (!IsValidPoint(previousDirection) ||
        glm::dot(previousDirection, previousDirection) <= kNormalEpsilon) {
        previousDirection = kGravity - currentNormal * glm::dot(kGravity, currentNormal);
    }
    if (!IsValidPoint(previousDirection) ||
        glm::dot(previousDirection, previousDirection) <= kNormalEpsilon) {
        previousDirection = kGravity;
    } else {
        previousDirection = glm::normalize(previousDirection);
    }
    rawSamples.push_back({
        .position = node.position,
        .normal = FromGlm(currentNormal),
        .station = 0.0F,
        .confidence = Clamp01(start.surfel.confidence *
                              (0.45F + 0.55F * start.surfel.normalCoherence)),
    });

    std::unordered_set<GridKey, GridKeyHash> visited;
    visited.insert({start.surfel.cellX, start.surfel.cellY, start.surfel.cellZ});
    const float stepMeters = resolution * 1.20F;
    const float riseTolerance = std::clamp(resolution * 0.30F, 0.0005F, 0.008F);
    const double requestedSteps = std::ceil(
        static_cast<double>(guide.requestedReachMeters) /
        static_cast<double>(std::max(0.001F, resolution))) * 3.0 + 8.0;
    const std::uint32_t maximumSteps = static_cast<std::uint32_t>(std::clamp(
        requestedSteps,
        2.0,
        2048.0));
    float station = 0.0F;
    for (std::uint32_t step = 0U;
         step < maximumSteps && station < guide.requestedReachMeters;
         ++step) {
        glm::vec3 gravityTangent =
            kGravity - currentNormal * glm::dot(kGravity, currentNormal);
        if (!IsValidPoint(gravityTangent) ||
            glm::dot(gravityTangent, gravityTangent) <= kNormalEpsilon) {
            gravityTangent = previousDirection -
                             currentNormal * glm::dot(previousDirection, currentNormal);
        }
        if (!IsValidPoint(gravityTangent) ||
            glm::dot(gravityTangent, gravityTangent) <= kNormalEpsilon) {
            break;
        }
        gravityTangent = glm::normalize(gravityTangent);
        glm::vec3 guidedDirection = glm::mix(previousDirection, gravityTangent, 0.72F);
        guidedDirection -= currentNormal * glm::dot(guidedDirection, currentNormal);
        if (glm::dot(guidedDirection, guidedDirection) <= kNormalEpsilon) {
            guidedDirection = gravityTangent;
        } else {
            guidedDirection = glm::normalize(guidedDirection);
        }
        glm::vec3 lateral = glm::cross(currentNormal, guidedDirection);
        if (!IsValidPoint(lateral) || glm::dot(lateral, lateral) <= kNormalEpsilon) {
            lateral = glm::cross(currentNormal, nodeNormal);
        }
        if (IsValidPoint(lateral) && glm::dot(lateral, lateral) > kNormalEpsilon) {
            lateral = glm::normalize(lateral);
        } else {
            lateral = glm::vec3{0.0F};
        }

        WaterSurfaceQueryResult selected;
        float selectedScore = -std::numeric_limits<float>::infinity();
        for (std::uint32_t lookAhead = 1U; lookAhead <= 2U && !selected.hit; ++lookAhead) {
            const float directionOffset = lookAhead == 1U ? 0.32F : 0.18F;
            const std::array<glm::vec3, 3> directions{{
                guidedDirection,
                glm::normalize(guidedDirection + lateral * directionOffset),
                glm::normalize(guidedDirection - lateral * directionOffset),
            }};
            for (const auto& direction : directions) {
                const float lookAheadDistance = stepMeters * static_cast<float>(lookAhead);
                const glm::vec3 expectedPosition = currentPosition +
                                                   direction * lookAheadDistance;
                const auto candidate = QueryWaterSurfaceCache(
                    cache,
                    FromGlm(expectedPosition),
                    resolution * (lookAhead == 1U ? 1.05F : 1.25F),
                    FromGlm(currentNormal),
                    roleMask);
                if (!candidate.hit || candidate.surfel.confidence < 0.10F ||
                    candidate.surfel.normalCoherence < 0.10F ||
                    visited.contains({
                        candidate.surfel.cellX,
                        candidate.surfel.cellY,
                        candidate.surfel.cellZ})) {
                    continue;
                }
                const glm::vec3 candidatePosition = ToGlm(candidate.surfel.centroid);
                const glm::vec3 segment = candidatePosition - currentPosition;
                const float segmentLength = glm::length(segment);
                if (!IsValidPoint(segment) || segmentLength <= 1.0e-5F ||
                    candidatePosition.z > currentPosition.z + riseTolerance ||
                    candidatePosition.z > ToGlm(node.position).z + riseTolerance) {
                    continue;
                }
                glm::vec3 candidateNormal = SafeSeepageNormal(candidate.surfel.normal);
                if (glm::dot(candidateNormal, currentNormal) < 0.0F) {
                    candidateNormal = -candidateNormal;
                }
                const float normalAgreement = Clamp01(glm::dot(candidateNormal, currentNormal));
                if (normalAgreement < 0.20F) {
                    continue;
                }
                const float directionAgreement = glm::dot(
                    segment / segmentLength,
                    guidedDirection);
                if (directionAgreement < -0.10F) {
                    continue;
                }
                const float drop = std::max(0.0F, currentPosition.z - candidatePosition.z);
                const float score =
                    directionAgreement * 1.8F +
                    std::min(1.0F, drop / resolution) * 0.55F +
                    Clamp01(candidate.surfel.confidence) * 0.35F +
                    Clamp01(candidate.surfel.normalCoherence) * 0.35F +
                    normalAgreement * 0.45F -
                    Clamp01(candidate.surfel.roughness) * 0.15F -
                    glm::length(candidatePosition - expectedPosition) /
                        std::max(resolution, 1.0e-5F) * 0.12F;
                if (!selected.hit || score > selectedScore) {
                    selected = candidate;
                    selected.surfel.normal = FromGlm(candidateNormal);
                    selectedScore = score;
                }
            }
        }
        if (!selected.hit) {
            break;
        }

        const glm::vec3 nextPosition = ToGlm(selected.surfel.centroid);
        const glm::vec3 segment = nextPosition - currentPosition;
        const float segmentLength = glm::length(segment);
        if (segmentLength <= 1.0e-5F) {
            break;
        }
        glm::vec3 nextNormal = SafeSeepageNormal(selected.surfel.normal);
        if (glm::dot(nextNormal, currentNormal) < 0.0F) {
            nextNormal = -nextNormal;
        }
        const float remaining = guide.requestedReachMeters - station;
        if (segmentLength >= remaining) {
            const float amount = remaining / segmentLength;
            glm::vec3 endNormal = glm::mix(currentNormal, nextNormal, amount);
            endNormal = glm::dot(endNormal, endNormal) > kNormalEpsilon
                            ? glm::normalize(endNormal)
                            : currentNormal;
            rawSamples.push_back({
                .position = FromGlm(currentPosition + segment * amount),
                .normal = FromGlm(endNormal),
                .station = guide.requestedReachMeters,
                .confidence = Clamp01(selected.surfel.confidence *
                                      (0.45F + 0.55F *
                                                   selected.surfel.normalCoherence)),
            });
            station = guide.requestedReachMeters;
            break;
        }

        station += segmentLength;
        rawSamples.push_back({
            .position = selected.surfel.centroid,
            .normal = FromGlm(nextNormal),
            .station = station,
            .confidence = Clamp01(selected.surfel.confidence *
                                  (0.45F + 0.55F * selected.surfel.normalCoherence)),
        });
        visited.insert({
            selected.surfel.cellX,
            selected.surfel.cellY,
            selected.surfel.cellZ});
        previousDirection = segment / segmentLength;
        currentPosition = nextPosition;
        currentNormal = nextNormal;
    }

    guide.achievedReachMeters = station;
    guide.complete = station + resolution >= guide.requestedReachMeters;
    if (rawSamples.size() < 2U || station <= resolution * 0.25F) {
        return guide;
    }

    const std::size_t outputCount = std::min<std::size_t>(
        kWaterSeepageMaximumGuideSamples,
        std::max<std::size_t>(2U, rawSamples.size()));
    std::size_t rawSegment = 0U;
    glm::vec3 previousOutputNormal = SafeSeepageNormal(rawSamples.front().normal);
    for (std::size_t outputIndex = 0U; outputIndex < outputCount; ++outputIndex) {
        const float targetStation = outputIndex + 1U == outputCount
                                        ? station
                                        : station * static_cast<float>(outputIndex) /
                                              static_cast<float>(outputCount - 1U);
        while (rawSegment + 1U < rawSamples.size() &&
               rawSamples[rawSegment + 1U].station < targetStation) {
            ++rawSegment;
        }
        const auto& left = rawSamples[rawSegment];
        const auto& right = rawSamples[std::min<std::size_t>(
            rawSegment + 1U,
            rawSamples.size() - 1U)];
        const float stationSpan = std::max(1.0e-5F, right.station - left.station);
        const float amount = Clamp01((targetStation - left.station) / stationSpan);
        glm::vec3 leftNormal = SafeSeepageNormal(left.normal);
        glm::vec3 rightNormal = SafeSeepageNormal(right.normal);
        if (glm::dot(leftNormal, rightNormal) < 0.0F) {
            rightNormal = -rightNormal;
        }
        glm::vec3 normal = glm::mix(leftNormal, rightNormal, amount);
        normal = glm::dot(normal, normal) > kNormalEpsilon
                     ? glm::normalize(normal)
                     : previousOutputNormal;
        if (glm::dot(normal, previousOutputNormal) < 0.0F) {
            normal = -normal;
        }
        guide.samples[outputIndex] = {
            .position = FromGlm(glm::mix(
                ToGlm(left.position),
                ToGlm(right.position),
                amount)),
            .normal = FromGlm(normal),
            .station = targetStation,
            .confidence = std::lerp(left.confidence, right.confidence, amount),
        };
        previousOutputNormal = normal;
    }
    guide.sampleCount = static_cast<std::uint32_t>(outputCount);
    guide.valid = true;
    return guide;
}

using SeepageSupportSourceKey =
    std::tuple<std::int32_t, std::int32_t, std::int32_t, WaterSurfaceRole>;
using SeepageSupportStateKey =
    std::tuple<
        std::int32_t,
        std::int32_t,
        std::int32_t,
        WaterSurfaceRole,
        bool>;
using SeepageSupportCellKey = std::tuple<std::int32_t, std::int32_t, std::int32_t>;

struct SeepageSupportStateKeyHash {
    std::size_t operator()(const SeepageSupportStateKey& key) const noexcept {
        const auto [x, y, z, role, upstream] = key;
        std::uint64_t hash = 1469598103934665603ULL;
        const auto mix = [&](std::uint32_t value) {
            hash ^= value;
            hash *= 1099511628211ULL;
        };
        mix(static_cast<std::uint32_t>(x));
        mix(static_cast<std::uint32_t>(y));
        mix(static_cast<std::uint32_t>(z));
        mix(static_cast<std::uint32_t>(role));
        mix(upstream ? 1U : 0U);
        return static_cast<std::size_t>(hash);
    }
};

struct SeepageSupportCellKeyHash {
    std::size_t operator()(const SeepageSupportCellKey& key) const noexcept {
        const auto [x, y, z] = key;
        std::uint64_t hash = 1469598103934665603ULL;
        const auto mix = [&](std::uint32_t value) {
            hash ^= value;
            hash *= 1099511628211ULL;
        };
        mix(static_cast<std::uint32_t>(x));
        mix(static_cast<std::uint32_t>(y));
        mix(static_cast<std::uint32_t>(z));
        return static_cast<std::size_t>(hash);
    }
};

std::optional<WaterSurfaceRole> SeepageSupportSourceRole(
    const WaterSeepageNode& node,
    std::string_view targetSceneRole) {
    if (!SeepageNodeTargetsRole(node, targetSceneRole)) {
        return std::nullopt;
    }
    const auto normalized = NormalizeSeepageRole(targetSceneRole);
    if (normalized == "sand") {
        return WaterSurfaceRole::Sand;
    }
    // Vegetation deliberately follows its ROCK substrate. The shared cache's
    // VEG voxels remain Rain occupancy rather than a surface graph.
    if (normalized == "rock" || normalized == "veg") {
        return WaterSurfaceRole::Rock;
    }
    return std::nullopt;
}

struct PendingSeepageSupportSurfel {
    const WaterSurfaceSurfel* surfel = nullptr;
    // The two labels are deliberately separate Dijkstra states. An upstream
    // wick can therefore never inherit a cheaper route that first travelled
    // downstream and came back up around a ledge.
    bool upstream = false;
    float costMeters = 0.0F;
    float flowRunMeters = 0.0F;
    float crossContourMeters = 0.0F;
    glm::vec3 downTangent{0.0F, 0.0F, -1.0F};
};

struct PendingSeepageSupportSurfelGreater {
    bool operator()(
        const PendingSeepageSupportSurfel& left,
        const PendingSeepageSupportSurfel& right) const {
        if (left.costMeters != right.costMeters) {
            return left.costMeters > right.costMeters;
        }
        return std::tie(
                   left.surfel->cellX,
                   left.surfel->cellY,
                   left.surfel->cellZ,
                   left.surfel->role,
                   left.upstream) >
               std::tie(
                   right.surfel->cellX,
                   right.surfel->cellY,
                   right.surfel->cellZ,
                   right.surfel->role,
                   right.upstream);
    }
};

std::string WaterSeepageSupportSelectionFingerprint(
    const WaterSeepageSupportSelection& selection,
    const WaterSurfaceCache& cache,
    const WaterSeepageNode& node) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 6U);
    SeepageFingerprintU32(&hash, selection.nodeId);
    SeepageFingerprintText(&hash, selection.targetSceneRole);
    SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(selection.sourceRole));
    SeepageFingerprintFloat(&hash, selection.cellSizeMeters);
    SeepageFingerprintFloat(&hash, selection.reachLimitMeters);
    SeepageFingerprintFloat(&hash, selection.widthLimitMeters);
    SeepageFingerprintFloat(&hash, node.edgeFeatherMeters);
    SeepageFingerprintFloat(&hash, node.depthToleranceMeters);
    SeepageFingerprintText(
        &hash,
        cache.cacheIdentity.sourceSignature.empty()
            ? std::string_view{cache.signature}
            : std::string_view{cache.cacheIdentity.sourceSignature});
    for (const auto word : cache.cacheIdentity.contentDigest) {
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(word));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(word >> 32U));
    }
    SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(selection.cells.size()));
    for (const auto& cell : selection.cells) {
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.x));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.y));
        SeepageFingerprintU32(&hash, static_cast<std::uint32_t>(cell.z));
        SeepageFingerprintFloat(&hash, cell.downwardDistanceMeters);
        SeepageFingerprintFloat(&hash, cell.flowRunMeters);
        SeepageFingerprintFloat(&hash, cell.crossContourMeters);
        SeepageFingerprintFloat(&hash, cell.surfaceNormal.x);
        SeepageFingerprintFloat(&hash, cell.surfaceNormal.y);
        SeepageFingerprintFloat(&hash, cell.surfaceNormal.z);
        SeepageFingerprintFloat(&hash, cell.confidence);
        SeepageFingerprintU32(&hash, cell.upstream ? 1U : 0U);
    }
    // v6: the upstream metric is station-relative branch excess. A winding
    // centre route can therefore stay one cell wide at the high tip without
    // discarding the cross-contour price that bounded the immutable support.
    return "water-seepage-support-v6-" + SeepageFingerprintString(hash);
}

}  // namespace

WaterSeepageSupportBuildResult BuildWaterSeepageSupportSelection(
    const WaterSeepageNode& node,
    std::string_view targetSceneRole,
    const WaterSurfaceCache& surfaceCache,
    const WaterSeepageSupportBuildOptions& options) {
    WaterSeepageSupportBuildResult result;
    result.selection.nodeId = node.id;
    result.selection.targetSceneRole = NormalizeSeepageRole(targetSceneRole);
    // The selection grid is deliberately fixed. It is shared by every display
    // density, so a density switch never changes semantic support membership.
    result.selection.cellSizeMeters = kWaterSeepageSupportCellSizeMeters;
    result.selection.reachLimitMeters = std::clamp(
        SeepageFiniteOr(node.selectionReachLimitMeters, 2.34375F),
        result.selection.cellSizeMeters,
        1000.0F);
    result.selection.widthLimitMeters = std::clamp(
        SeepageFiniteOr(node.selectionWidthLimitMeters, 1.215F),
        result.selection.cellSizeMeters,
        1000.0F);

    const auto sourceRole = SeepageSupportSourceRole(node, targetSceneRole);
    if (!sourceRole.has_value()) {
        result.errorMessage = "The Seepage node does not target a supported authored terrain role.";
        return result;
    }
    result.selection.sourceRole = sourceRole.value();
    if (surfaceCache.flowSurfaceSurfels.empty() ||
        !std::isfinite(surfaceCache.resolutionMeters) ||
        surfaceCache.resolutionMeters <= 0.0F ||
        !IsValidPoint(ToGlm(node.position))) {
        result.errorMessage = "The shared Water surface cache has no usable connected surfels.";
        return result;
    }

    const std::size_t maximumCells = std::clamp<std::size_t>(
        options.maximumSupportCells,
        1U,
        kWaterSeepageMaximumSupportCellsPerNode);
    const std::size_t maximumVisited = std::clamp<std::size_t>(
        options.maximumVisitedSurfels,
        1U,
        kWaterSeepageMaximumSupportCellsPerNode);
    const float sourceResolution = surfaceCache.resolutionMeters;
    const std::uint32_t roleMask = sourceRole.value() == WaterSurfaceRole::Sand
                                       ? kWaterSurfaceSandRoleMask
                                       : kWaterSurfaceRockRoleMask;
    // The immutable surfel vector is ordered by X/Y/Z/role. A flood probes 26
    // neighbours at every accepted cell; repeating a full-vector lower_bound
    // for each probe dominated full-site startup. Cache the much smaller
    // contiguous X slices touched by this node, then search only Y/Z/role
    // within a slice. This adds O(number of visited X cells) metadata and
    // returns the exact same surfel pointer as the former global search.
    std::unordered_map<
        std::int32_t,
        std::pair<std::size_t, std::size_t>>
        sourceXRanges;
    sourceXRanges.reserve(512U);
    const auto findSupportSurfel =
        [&](const SeepageSupportSourceKey& key)
            -> const WaterSurfaceSurfel* {
            const auto [cellX, cellY, cellZ, role] = key;
            auto range = sourceXRanges.find(cellX);
            if (range == sourceXRanges.end()) {
                const auto begin = std::lower_bound(
                    surfaceCache.flowSurfaceSurfels.begin(),
                    surfaceCache.flowSurfaceSurfels.end(),
                    cellX,
                    [](const WaterSurfaceSurfel& surfel,
                       std::int32_t candidateX) {
                        return surfel.cellX < candidateX;
                    });
                const auto end = std::upper_bound(
                    begin,
                    surfaceCache.flowSurfaceSurfels.end(),
                    cellX,
                    [](std::int32_t candidateX,
                       const WaterSurfaceSurfel& surfel) {
                        return candidateX < surfel.cellX;
                    });
                range = sourceXRanges.emplace(
                    cellX,
                    std::pair{
                        static_cast<std::size_t>(
                            begin -
                            surfaceCache.flowSurfaceSurfels.begin()),
                        static_cast<std::size_t>(
                            end -
                            surfaceCache.flowSurfaceSurfels.begin()),
                    }).first;
            }
            const auto sliceBegin =
                surfaceCache.flowSurfaceSurfels.begin() +
                static_cast<std::ptrdiff_t>(range->second.first);
            const auto sliceEnd =
                surfaceCache.flowSurfaceSurfels.begin() +
                static_cast<std::ptrdiff_t>(range->second.second);
            const auto candidateKey =
                std::tuple{cellY, cellZ, role};
            const auto found = std::lower_bound(
                sliceBegin,
                sliceEnd,
                candidateKey,
                [](const WaterSurfaceSurfel& surfel,
                   const auto& candidate) {
                    return std::tie(
                               surfel.cellY,
                               surfel.cellZ,
                               surfel.role) <
                           candidate;
                });
            if (found == sliceEnd ||
                std::tie(
                    found->cellY,
                    found->cellZ,
                    found->role) != candidateKey) {
                return nullptr;
            }
            return &*found;
        };
    const float startRadius = std::clamp(
        std::max(sourceResolution * 2.25F, 0.05F),
        sourceResolution,
        0.25F);
    const auto start = QueryWaterSurfaceCache(
        surfaceCache,
        node.position,
        startRadius,
        node.surfaceNormal,
        roleMask);
    if (!start.hit || start.surfel.role != sourceRole.value()) {
        result.errorMessage = "No matching shared-cache surface cell was found beneath the Seepage node.";
        result.surfaceUnavailable = true;
        return result;
    }

    const glm::vec3 nodePosition = ToGlm(node.position);
    const glm::vec3 nodeNormal = SafeSeepageNormal(node.surfaceNormal);
    const auto deterministicTangent = [](const glm::vec3& normal) {
        const glm::vec3 helper =
            std::abs(normal.x) <= std::abs(normal.y) &&
                    std::abs(normal.x) <= std::abs(normal.z)
                ? glm::vec3{1.0F, 0.0F, 0.0F}
                : (std::abs(normal.y) <= std::abs(normal.z)
                       ? glm::vec3{0.0F, 1.0F, 0.0F}
                       : glm::vec3{0.0F, 0.0F, 1.0F});
        return glm::normalize(glm::cross(normal, helper));
    };
    const auto projectedTangent = [&](const glm::vec3& direction,
                                      const glm::vec3& normal) {
        glm::vec3 tangent =
            direction - normal * glm::dot(direction, normal);
        return IsValidPoint(tangent) &&
                       glm::dot(tangent, tangent) > kNormalEpsilon
                   ? glm::normalize(tangent)
                   : deterministicTangent(normal);
    };
    const glm::vec3 startNormal = [&] {
        glm::vec3 normal = SafeSeepageNormal(start.surfel.normal);
        if (glm::dot(normal, nodeNormal) < 0.0F) {
            normal = -normal;
        }
        return normal;
    }();
    glm::vec3 seedDownTangent =
        kGravity - startNormal * glm::dot(kGravity, startNormal);
    if (!IsValidPoint(seedDownTangent) ||
        glm::dot(seedDownTangent, seedDownTangent) <= kNormalEpsilon) {
        seedDownTangent = projectedTangent(
            ToGlm(node.downAxis),
            startNormal);
    } else {
        seedDownTangent = glm::normalize(seedDownTangent);
    }

    struct TangentStepMetrics {
        glm::vec3 localNormal{0.0F, 0.0F, 1.0F};
        glm::vec3 downTangent{0.0F, 0.0F, -1.0F};
        float tangentLength = 0.0F;
        float down = 0.0F;
        float up = 0.0F;
        float cross = 0.0F;
        float steepBlend = 0.0F;
    };
    const auto tangentStepMetrics =
        [&](const glm::vec3& step,
            const glm::vec3& currentNormal,
            float currentConfidence,
            glm::vec3 neighbourNormal,
            float neighbourConfidence,
            const glm::vec3& previousDownTangent) {
            if (glm::dot(currentNormal, neighbourNormal) < 0.0F) {
                neighbourNormal = -neighbourNormal;
            }
            const float currentWeight = std::max(
                0.05F,
                Clamp01(currentConfidence));
            const float neighbourWeight = std::max(
                0.05F,
                Clamp01(neighbourConfidence));
            glm::vec3 localNormal =
                currentNormal * currentWeight +
                neighbourNormal * neighbourWeight;
            localNormal =
                IsValidPoint(localNormal) &&
                        glm::dot(localNormal, localNormal) > kNormalEpsilon
                    ? glm::normalize(localNormal)
                    : currentNormal;
            TangentStepMetrics metrics;
            metrics.localNormal = localNormal;
            const glm::vec3 tangentStep =
                step - localNormal * glm::dot(step, localNormal);
            metrics.tangentLength = glm::length(tangentStep);
            glm::vec3 projectedGravity =
                kGravity - localNormal * glm::dot(kGravity, localNormal);
            const float steepness = std::clamp(
                glm::length(projectedGravity),
                0.0F,
                1.0F);
            metrics.steepBlend = SmoothStep(
                kWaterSeepageSteepnessBlendStart,
                kWaterSeepageSteepnessBlendEnd,
                steepness);
            glm::vec3 transported =
                previousDownTangent -
                localNormal *
                    glm::dot(previousDownTangent, localNormal);
            if (!IsValidPoint(transported) ||
                glm::dot(transported, transported) <=
                    kNormalEpsilon) {
                transported = deterministicTangent(localNormal);
            } else {
                transported = glm::normalize(transported);
            }
            if (steepness > kNormalEpsilon) {
                const glm::vec3 gravityTangent =
                    projectedGravity / steepness;
                if (glm::dot(transported, gravityTangent) < 0.0F) {
                    transported = -transported;
                }
                metrics.downTangent = glm::mix(
                    transported,
                    gravityTangent,
                    metrics.steepBlend);
                metrics.downTangent =
                    glm::dot(
                        metrics.downTangent,
                        metrics.downTangent) > kNormalEpsilon
                        ? glm::normalize(metrics.downTangent)
                        : gravityTangent;
                if (glm::dot(
                        metrics.downTangent,
                        projectedGravity) < 0.0F) {
                    metrics.downTangent =
                        -metrics.downTangent;
                }
            } else {
                metrics.downTangent = transported;
            }
            const float signedFlow =
                glm::dot(tangentStep, metrics.downTangent);
            metrics.down = std::max(0.0F, signedFlow);
            metrics.up = std::max(0.0F, -signedFlow);
            metrics.cross = std::sqrt(std::max(
                0.0F,
                metrics.tangentLength * metrics.tangentLength -
                    signedFlow * signedFlow));
            return metrics;
        };

    // Two independent labels share the same connected surface but never each
    // other's route. Downstream blends flat isotropic creep into strongly
    // anisotropic wall flow; upstream is a narrow, separately tapered wick.
    const auto downstreamStepCost = [](const TangentStepMetrics& metrics) {
        const float flatCost =
            kWaterSeepageContourCostFactor * metrics.tangentLength;
        const float steepCost =
            kWaterSeepageDescentCostFactor * metrics.down +
            kWaterSeepageSteepContourCostFactor * metrics.cross +
            kWaterSeepageAscentCostFactor * metrics.up;
        return std::lerp(flatCost, steepCost, metrics.steepBlend);
    };
    const auto upstreamStepCost = [](const TangentStepMetrics& metrics) {
        return kWaterSeepageUpstreamCostFactor * metrics.up +
               kWaterSeepageSteepContourCostFactor * metrics.cross;
    };
    const float depthTolerance = std::clamp(
        SeepageFiniteOr(node.depthToleranceMeters, 0.15F),
        sourceResolution * 0.50F,
        2.0F);
    const float reachLimit = result.selection.reachLimitMeters;
    const float halfWidthLimit = result.selection.widthLimitMeters * 0.5F;
    // The selection includes enough topology for the live front and lateral
    // feathers at maximum authored limits. Cross-contour travel has its own
    // hard cap, so a cheap descending detour cannot later return far outside
    // the requested width.
    const float costLimit = reachLimit;
    const float sourcePlaneTolerance =
        result.selection.cellSizeMeters * 2.0F;
    const float sourceTopologyFeather =
        result.selection.cellSizeMeters * 2.0F;
    const float sourcePatchRadius =
        halfWidthLimit + sourceTopologyFeather;
    const float sourcePatchRadiusSquared =
        sourcePatchRadius * sourcePatchRadius;
    const float continuityDistance = std::clamp(
        depthTolerance,
        sourceResolution * 1.75F,
        sourceResolution * 6.0F);
    const float continuityDistanceSquared = continuityDistance * continuityDistance;
    // Every valid route step is priced by at least the cheapest authored
    // travel regime. The world-radius guard is deliberately more generous:
    // it includes the source-width exception and one continuity hop, but
    // prevents a noisy/thick substrate from walking through near-normal
    // neighbours indefinitely while accumulating almost no tangent cost.
    constexpr float kMinimumTravelCostFactor =
        std::min(
            kWaterSeepageDescentCostFactor,
            kWaterSeepageUpstreamCostFactor);
    const float maximumGeometricRadius =
        costLimit / kMinimumTravelCostFactor +
        sourcePatchRadius +
        continuityDistance;
    const float maximumGeometricRadiusSquared =
        maximumGeometricRadius * maximumGeometricRadius;

    std::priority_queue<
        PendingSeepageSupportSurfel,
        std::vector<PendingSeepageSupportSurfel>,
        PendingSeepageSupportSurfelGreater>
        pending;
    struct SeepageSupportState {
        float queuedCost = 0.0F;
        bool visited = false;
    };
    std::unordered_map<
        SeepageSupportStateKey,
        SeepageSupportState,
        SeepageSupportStateKeyHash>
        states;
    struct EmittedSeepageSupportCell {
        WaterSeepageSupportCell cell;
        std::uint8_t priority = 0U;
    };
    std::unordered_map<
        SeepageSupportCellKey,
        EmittedSeepageSupportCell,
        SeepageSupportCellKeyHash>
        emitted;
    // Avoid one allocation per tree node while keeping the initial bucket
    // arrays small enough for two concurrent full-site node builds.
    states.reserve(std::min<std::size_t>(maximumVisited, 65'536U));
    emitted.reserve(std::min<std::size_t>(maximumCells, 65'536U));
    std::size_t visitedCount = 0U;
    std::vector<const WaterSurfaceSurfel*> acceptedSubstrateSurfels;
    struct AcceptedSubstrateMetrics {
        float costMeters = 0.0F;
        float flowRunMeters = 0.0F;
        float crossContourMeters = 0.0F;
        bool upstream = false;
        glm::vec3 downTangent{0.0F, 0.0F, -1.0F};
    };
    std::vector<AcceptedSubstrateMetrics> acceptedSubstrateMetrics;
    const bool targetIsVegetation =
        NormalizeSeepageRole(targetSceneRole) == "veg";
    const auto emitSourceCell = [&](std::int32_t sourceX,
                                    std::int32_t sourceY,
                                    std::int32_t sourceZ,
                                    float costMeters,
                                    float flowRunMeters,
                                    float crossContourMeters,
                                    bool upstream,
                                    const invisible_places::io::Float3& surfaceNormal,
                                    float confidence) {
        const auto childMinimum = [&](std::int32_t sourceCoordinate) {
            return static_cast<std::int32_t>(std::floor(
                static_cast<double>(sourceCoordinate) * sourceResolution /
                result.selection.cellSizeMeters));
        };
        const auto childMaximum = [&](std::int32_t sourceCoordinate) {
            const double sourceMinimum =
                static_cast<double>(sourceCoordinate) * sourceResolution;
            return static_cast<std::int32_t>(std::ceil(
                       (sourceMinimum + sourceResolution) /
                       result.selection.cellSizeMeters) -
                   1.0);
        };
        const auto minX = childMinimum(sourceX);
        const auto minY = childMinimum(sourceY);
        const auto minZ = childMinimum(sourceZ);
        const auto maxX = childMaximum(sourceX);
        const auto maxY = childMaximum(sourceY);
        const auto maxZ = childMaximum(sourceZ);
        for (std::int64_t z = minZ; z <= static_cast<std::int64_t>(maxZ); ++z) {
            for (std::int64_t y = minY; y <= static_cast<std::int64_t>(maxY); ++y) {
                for (std::int64_t x = minX; x <= static_cast<std::int64_t>(maxX); ++x) {
                    const auto key = SeepageSupportCellKey{
                        static_cast<std::int32_t>(x),
                        static_cast<std::int32_t>(y),
                        static_cast<std::int32_t>(z),
                    };
                    const float cellCentreZ =
                        (static_cast<float>(z) + 0.5F) *
                        result.selection.cellSizeMeters;
                    const bool preferredUpstream =
                        cellCentreZ > nodePosition.z;
                    const std::uint8_t priority =
                        upstream == preferredUpstream ? 2U : 1U;
                    const WaterSeepageSupportCell cell{
                        .x = static_cast<std::int32_t>(x),
                        .y = static_cast<std::int32_t>(y),
                        .z = static_cast<std::int32_t>(z),
                        .downwardDistanceMeters = costMeters,
                        .flowRunMeters = flowRunMeters,
                        .crossContourMeters = crossContourMeters,
                        .surfaceNormal = surfaceNormal,
                        .confidence = Clamp01(confidence),
                        .upstream = upstream,
                    };
                    const auto [it, inserted] = emitted.emplace(
                        key,
                        EmittedSeepageSupportCell{
                            .cell = cell,
                            .priority = priority,
                        });
                    if (!inserted &&
                        (priority > it->second.priority ||
                         (priority == it->second.priority &&
                          (cell.downwardDistanceMeters <
                               it->second.cell.downwardDistanceMeters - 1.0e-6F ||
                           (std::abs(
                                cell.downwardDistanceMeters -
                                it->second.cell.downwardDistanceMeters) <= 1.0e-6F &&
                            cell.confidence > it->second.cell.confidence))))) {
                        it->second.cell = cell;
                        it->second.priority = priority;
                    }
                    if (emitted.size() > maximumCells) {
                        return false;
                    }
                }
            }
        }
        return true;
    };
    const auto startKey = SeepageSupportSourceKey{
        start.surfel.cellX,
        start.surfel.cellY,
        start.surfel.cellZ,
        sourceRole.value(),
    };
    const auto* startSurfel = findSupportSurfel(startKey);
    if (startSurfel == nullptr) {
        result.errorMessage = "The shared-cache Seepage seed cell was not addressable.";
        return result;
    }
    for (const bool upstream : {false, true}) {
        pending.push({
            .surfel = startSurfel,
            .upstream = upstream,
            .costMeters = 0.0F,
            .flowRunMeters = 0.0F,
            .crossContourMeters = 0.0F,
            .downTangent = seedDownTangent,
        });
        states.emplace(
            SeepageSupportStateKey{
                start.surfel.cellX,
                start.surfel.cellY,
                start.surfel.cellZ,
                sourceRole.value(),
                upstream,
            },
            SeepageSupportState{
                .queuedCost = 0.0F,
                .visited = false,
            });
    }

    while (!pending.empty()) {
        if (options.stopToken != nullptr && options.stopToken->stop_requested()) {
            result.cancelled = true;
            result.errorMessage = "Connected Seepage support selection was cancelled.";
            return result;
        }
        if (visitedCount >= maximumVisited) {
            result.diagnostics.cellLimitExceeded = true;
            result.errorMessage =
                "Connected Seepage support exceeded its bounded surfel budget; "
                "reduce Selection Reach/Width limits.";
            return result;
        }
        const auto current = pending.top();
        pending.pop();
        const auto& surfel = *current.surfel;
        const auto currentKey = SeepageSupportStateKey{
            surfel.cellX,
            surfel.cellY,
            surfel.cellZ,
            surfel.role,
            current.upstream,
        };
        const auto currentState = states.find(currentKey);
        if (currentState == states.end() ||
            currentState->second.visited) {
            continue;
        }
        currentState->second.visited = true;
        ++visitedCount;
        ++result.diagnostics.visitedSurfelCount;

        const glm::vec3 centroid = ToGlm(surfel.centroid);
        bool accepted = true;
        const glm::vec3 nodeDelta = centroid - nodePosition;
        const float gravitationalElevation =
            centroid.z - nodePosition.z;
        if (glm::dot(nodeDelta, nodeDelta) >
            maximumGeometricRadiusSquared) {
            ++result.diagnostics.rejectedReachCount;
            accepted = false;
        } else if ((!current.upstream &&
             gravitationalElevation > sourcePlaneTolerance) ||
            (current.upstream &&
             gravitationalElevation < -sourcePlaneTolerance)) {
            ++result.diagnostics.rejectedAboveNodeCount;
            accepted = false;
        } else if (current.crossContourMeters >
                   halfWidthLimit + sourceTopologyFeather) {
            ++result.diagnostics.rejectedWidthCount;
            accepted = false;
        } else if (
            current.costMeters > costLimit &&
            (std::hypot(
                 current.flowRunMeters,
                 current.crossContourMeters) >
                 sourcePatchRadius ||
             glm::dot(nodeDelta, nodeDelta) >
                 sourcePatchRadiusSquared)) {
            ++result.diagnostics.rejectedReachCount;
            accepted = false;
        } else if (surfel.confidence < 0.10F || surfel.normalCoherence < 0.10F) {
            ++result.diagnostics.rejectedContinuityCount;
            accepted = false;
        }

        if (accepted) {
            ++result.diagnostics.acceptedSurfelCount;
            const bool preferredUpstream =
                gravitationalElevation > 0.0F;
            if (current.upstream == preferredUpstream) {
                acceptedSubstrateSurfels.push_back(&surfel);
                acceptedSubstrateMetrics.push_back({
                    .costMeters = current.costMeters,
                    .flowRunMeters = current.flowRunMeters,
                    .crossContourMeters =
                        current.crossContourMeters,
                    .upstream = current.upstream,
                    .downTangent = current.downTangent,
                });
            }
            if (!targetIsVegetation &&
                !emitSourceCell(
                    surfel.cellX,
                    surfel.cellY,
                    surfel.cellZ,
                    current.costMeters,
                    current.flowRunMeters,
                    current.crossContourMeters,
                    current.upstream,
                    surfel.normal,
                    surfel.confidence *
                        (0.45F + 0.55F * surfel.normalCoherence))) {
                result.diagnostics.cellLimitExceeded = true;
                result.errorMessage =
                    "Connected Seepage support exceeded its bounded 10 mm cell budget; "
                    "reduce the Selection Reach limit.";
                return result;
            }
        }

        if (!accepted) {
            continue;
        }
        const glm::vec3 currentNormal = SafeSeepageNormal(surfel.normal);
        for (std::int32_t dz = -1; dz <= 1; ++dz) {
            for (std::int32_t dy = -1; dy <= 1; ++dy) {
                for (std::int32_t dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const auto sourceKey = SeepageSupportSourceKey{
                        surfel.cellX + dx,
                        surfel.cellY + dy,
                        surfel.cellZ + dz,
                        sourceRole.value(),
                    };
                    const auto stateKey = SeepageSupportStateKey{
                        surfel.cellX + dx,
                        surfel.cellY + dy,
                        surfel.cellZ + dz,
                        sourceRole.value(),
                        current.upstream,
                    };
                    const auto neighbourState = states.find(stateKey);
                    if (neighbourState != states.end() &&
                        neighbourState->second.visited) {
                        continue;
                    }
                    const auto* neighbour =
                        findSupportSurfel(sourceKey);
                    if (neighbour == nullptr) {
                        continue;
                    }
                    const glm::vec3 step = ToGlm(neighbour->centroid) - centroid;
                    const float distanceSquared = glm::dot(step, step);
                    const glm::vec3 neighbourNormal = SafeSeepageNormal(neighbour->normal);
                    if (!std::isfinite(distanceSquared) ||
                        distanceSquared > continuityDistanceSquared) {
                        ++result.diagnostics.rejectedContinuityCount;
                        continue;
                    }
                    const float stepLength = std::sqrt(distanceSquared);
                    const auto metrics = tangentStepMetrics(
                        step,
                        currentNormal,
                        surfel.confidence,
                        neighbourNormal,
                        neighbour->confidence,
                        current.downTangent);
                    if (!std::isfinite(metrics.tangentLength) ||
                        metrics.tangentLength <= 1.0e-6F) {
                        ++result.diagnostics.rejectedContinuityCount;
                        continue;
                    }
                    if (std::abs(
                            glm::dot(
                                step,
                                metrics.localNormal)) >
                        depthTolerance) {
                        ++result.diagnostics.rejectedContinuityCount;
                        continue;
                    }
                    if (current.upstream &&
                        metrics.down > sourceResolution * 0.25F) {
                        // The upstream label is a one-way wick from the node
                        // toward higher substrate. A downhill step has zero
                        // `up` and can have zero `cross`, so admitting it here
                        // creates an unpriced lane on flat or gently stepped
                        // terrain. Reject it at the source as well as farther
                        // uphill; the independent downstream label owns that
                        // side of the connected surface.
                        ++result.diagnostics.rejectedAboveNodeCount;
                        continue;
                    }
                    const float directionalEdgeCost =
                        current.upstream
                            ? upstreamStepCost(metrics)
                            : downstreamStepCost(metrics);
                    // Tangent decomposition shapes seepage on a surface, but
                    // it cannot make motion through the surface free. This
                    // floor is identical to ordinary steep descent when the
                    // step lies in the substrate plane and only adds cost to
                    // near-normal hops caused by roughness or layer thickness.
                    const float edgeCost = std::max(
                        directionalEdgeCost,
                        kMinimumTravelCostFactor * stepLength);
                    const float nextCost =
                        current.costMeters + edgeCost;
                    if (neighbourState != states.end() &&
                        neighbourState->second.queuedCost <=
                            nextCost + 1.0e-6F) {
                        continue;
                    }
                    pending.push({
                        .surfel = neighbour,
                        .upstream = current.upstream,
                        .costMeters = nextCost,
                        .flowRunMeters =
                            current.flowRunMeters +
                            (current.upstream
                                 ? metrics.up
                                 : metrics.down + metrics.up),
                        .crossContourMeters =
                            current.crossContourMeters +
                            metrics.cross,
                        .downTangent = metrics.downTangent,
                    });
                    if (neighbourState == states.end()) {
                        states.emplace(
                            stateKey,
                            SeepageSupportState{
                                .queuedCost = nextCost,
                                .visited = false,
                            });
                    } else {
                        neighbourState->second.queuedCost = nextCost;
                    }
                }
            }
        }
    }

    if (targetIsVegetation && !acceptedSubstrateSurfels.empty()) {
        // VEG uses the connected ROCK sheet as its semantic substrate, then
        // projects that settled downstream metric onto nearby authored VEG
        // occupancy. This keeps leaf points addressable in their own 3-D
        // cells without allowing vegetation to bridge disconnected rock.
        SupportGraph substrateGraph;
        substrateGraph.cellSize = std::max(0.05F, sourceResolution * 4.0F);
        substrateGraph.points.reserve(acceptedSubstrateSurfels.size());
        glm::vec3 substrateMinimum{
            std::numeric_limits<float>::max()};
        glm::vec3 substrateMaximum{
            std::numeric_limits<float>::lowest()};
        for (const auto* substrate : acceptedSubstrateSurfels) {
            const auto index = static_cast<std::uint32_t>(substrateGraph.points.size());
            substrateGraph.points.push_back({
                .sourceIndex = index,
                .position = ToGlm(substrate->centroid),
                .normal = SafeSeepageNormal(substrate->normal),
                .confidence = Clamp01(
                    substrate->confidence *
                    (0.45F + 0.55F * substrate->normalCoherence)),
                .hasNormal = true,
            });
            substrateMinimum = glm::min(
                substrateMinimum,
                substrateGraph.points.back().position);
            substrateMaximum = glm::max(
                substrateMaximum,
                substrateGraph.points.back().position);
            substrateGraph.grid[
                MakeGridKey(substrateGraph.points.back().position, substrateGraph.cellSize)]
                .push_back(index);
        }

        const float associationDistance = std::clamp(
            sourceResolution * 8.0F,
            0.12F,
            0.30F);
        const float associationDistanceSquared =
            associationDistance * associationDistance;
        // Descent-priced cost lets substrate reach costLimit / 0.75 metres
        // of geometric distance, so the scan radius must cover that.
        const float selectionRadius =
            costLimit / kWaterSeepageDescentCostFactor +
            halfWidthLimit + associationDistance;
        // A VEG voxel can only be emitted when it lies within
        // associationDistance of the already accepted substrate. Restrict the
        // sorted X scan to that exact expanded support envelope and reject Y/Z
        // outside it before the 27-cell graph query. On full-site caches this
        // avoids walking a metres-wide vegetation slab for a narrow seepage
        // route, without changing any association or depth threshold.
        const glm::vec3 associationPadding{
            associationDistance + 1.0e-5F};
        const glm::vec3 vegetationMinimum =
            substrateMinimum - associationPadding;
        const glm::vec3 vegetationMaximum =
            substrateMaximum + associationPadding;
        const auto minimumCellX = SeepageCellCoordinate(
            vegetationMinimum.x,
            sourceResolution);
        const auto maximumCellX = SeepageCellCoordinate(
            vegetationMaximum.x,
            sourceResolution);
        auto vegetation = std::lower_bound(
            surfaceCache.vegetationVoxels.begin(),
            surfaceCache.vegetationVoxels.end(),
            minimumCellX,
            [](const RainVegetationVoxel& voxel, std::int32_t x) {
                return voxel.cellX < x;
            });
        std::size_t vegetationScanCount = 0U;
        for (; vegetation != surfaceCache.vegetationVoxels.end() &&
               vegetation->cellX <= maximumCellX;
             ++vegetation) {
            if ((++vegetationScanCount & 0x3ffU) == 0U &&
                options.stopToken != nullptr &&
                options.stopToken->stop_requested()) {
                result.cancelled = true;
                result.errorMessage =
                    "Connected Seepage VEG association was cancelled.";
                return result;
            }
            const glm::vec3 vegetationPosition{
                (static_cast<float>(vegetation->cellX) + 0.5F) * sourceResolution,
                (static_cast<float>(vegetation->cellY) + 0.5F) * sourceResolution,
                (static_cast<float>(vegetation->cellZ) + 0.5F) * sourceResolution,
            };
            if (vegetationPosition.y < vegetationMinimum.y ||
                vegetationPosition.y > vegetationMaximum.y ||
                vegetationPosition.z < vegetationMinimum.z ||
                vegetationPosition.z > vegetationMaximum.z) {
                continue;
            }
            const glm::vec3 vegetationDelta = vegetationPosition - nodePosition;
            if (glm::dot(vegetationDelta, vegetationDelta) >
                selectionRadius * selectionRadius) {
                continue;
            }
            const auto nearby = NearbySupportIndices(
                substrateGraph,
                vegetationPosition,
                associationDistance);
            std::optional<std::uint32_t> closestIndex;
            float closestDistanceSquared = associationDistanceSquared;
            for (const auto candidateIndex : nearby) {
                if (candidateIndex >= substrateGraph.points.size()) {
                    continue;
                }
                const auto delta = substrateGraph.points[candidateIndex].position -
                                   vegetationPosition;
                const float distanceSquared = glm::dot(delta, delta);
                if (distanceSquared <= closestDistanceSquared) {
                    closestDistanceSquared = distanceSquared;
                    closestIndex = candidateIndex;
                }
            }
            if (!closestIndex.has_value()) {
                continue;
            }
            const auto& substrate = substrateGraph.points[*closestIndex];
            // Vegetation hugging its wet substrate (moss, low growth) shares
            // the seep; canopy hovering above it stays dry.
            const float riseAboveSubstrate =
                vegetationPosition.z - substrate.position.z;
            if (riseAboveSubstrate > kWaterSeepageVegetationRiseMeters) {
                continue;
            }
            const auto& substrateMetrics =
                acceptedSubstrateMetrics[*closestIndex];
            glm::vec3 substrateNormal = substrate.normal;
            if (glm::dot(substrateNormal, nodeNormal) < 0.0F) {
                substrateNormal = -substrateNormal;
            }
            const glm::vec3 substrateDelta =
                vegetationPosition - substrate.position;
            const float normalOffset =
                glm::dot(substrateDelta, substrateNormal);
            // Surface Depth is a rejection bound, not a finite-price bridge:
            // foliage floating outward from the rock must never become wet
            // merely because a large live budget can pay for the gap.
            if (std::abs(normalOffset) > depthTolerance) {
                continue;
            }
            const glm::vec3 tangentDelta =
                substrateDelta - substrateNormal * normalOffset;
            const auto associationMetrics = tangentStepMetrics(
                tangentDelta,
                substrateNormal,
                substrate.confidence,
                substrateNormal,
                substrate.confidence,
                substrateMetrics.downTangent);
            const float associationCost =
                substrateMetrics.upstream
                    ? upstreamStepCost(associationMetrics)
                    : downstreamStepCost(associationMetrics);
            const float flowRun =
                substrateMetrics.flowRunMeters +
                (substrateMetrics.upstream
                     ? associationMetrics.up
                     : associationMetrics.down +
                           associationMetrics.up);
            const float crossContour =
                substrateMetrics.crossContourMeters +
                associationMetrics.cross;
            const float cost =
                substrateMetrics.costMeters + associationCost;
            if (crossContour >
                    halfWidthLimit + sourceTopologyFeather ||
                (cost > costLimit &&
                 std::hypot(flowRun, crossContour) >
                     halfWidthLimit +
                         sourceTopologyFeather)) {
                continue;
            }
            const float vegetationConfidence = std::clamp(
                static_cast<float>(vegetation->sampleCount) / 8.0F,
                0.0F,
                1.0F);
            if (!emitSourceCell(
                    vegetation->cellX,
                    vegetation->cellY,
                    vegetation->cellZ,
                    cost,
                    flowRun,
                    crossContour,
                    substrateMetrics.upstream,
                    FromGlm(substrate.normal),
                    substrate.confidence * (0.50F + 0.50F * vegetationConfidence))) {
                result.diagnostics.cellLimitExceeded = true;
                result.errorMessage =
                    "Connected Seepage VEG support exceeded its bounded 10 mm cell budget; "
                    "reduce the Selection Reach limit.";
                return result;
            }
        }
    }

    if (emitted.empty()) {
        result.errorMessage = "No connected shared-cache cells were accepted beneath the Seepage node.";
        result.surfaceUnavailable = true;
        return result;
    }
    result.selection.cells.reserve(emitted.size());
    for (const auto& [_, emittedCell] : emitted) {
        result.selection.cells.push_back(emittedCell.cell);
    }
    // Upstream routes often have to wind around surface roughness before
    // reaching the highest resident cell. Their accumulated cross-contour
    // price is still required while selecting bounded support, but using that
    // absolute price as live width can reject every cell at the real tip.
    // Remove only the cheapest upstream price in each 10 mm flow-run station:
    // one connected centre route remains addressable while other branches
    // retain their local excess and must still pass the narrow live mask.
    std::unordered_map<std::int64_t, float> upstreamStationBaselines;
    upstreamStationBaselines.reserve(
        result.selection.cells.size() / 8U + 1U);
    const float inverseStationSize =
        1.0F / std::max(
                   1.0e-5F,
                   result.selection.cellSizeMeters);
    const auto upstreamStation = [inverseStationSize](
                                     const WaterSeepageSupportCell& cell) {
        return static_cast<std::int64_t>(std::llround(
            std::max(0.0F, cell.flowRunMeters) *
            inverseStationSize));
    };
    for (const auto& cell : result.selection.cells) {
        if (!cell.upstream) {
            continue;
        }
        const auto station = upstreamStation(cell);
        const float cross = std::max(0.0F, cell.crossContourMeters);
        const auto [baseline, inserted] =
            upstreamStationBaselines.emplace(station, cross);
        if (!inserted) {
            baseline->second = std::min(baseline->second, cross);
        }
    }
    for (auto& cell : result.selection.cells) {
        if (!cell.upstream) {
            continue;
        }
        const auto baseline =
            upstreamStationBaselines.find(upstreamStation(cell));
        if (baseline != upstreamStationBaselines.end()) {
            cell.crossContourMeters = std::max(
                0.0F,
                cell.crossContourMeters - baseline->second);
        }
    }
    std::sort(
        result.selection.cells.begin(),
        result.selection.cells.end(),
        [](const WaterSeepageSupportCell& left,
           const WaterSeepageSupportCell& right) {
            return std::tie(left.x, left.y, left.z) <
                   std::tie(right.x, right.y, right.z);
        });
    const float halfCell = result.selection.cellSizeMeters * 0.5F;
    for (const auto& cell : result.selection.cells) {
        const invisible_places::io::Float3 centre{
            (static_cast<float>(cell.x) + 0.5F) * result.selection.cellSizeMeters,
            (static_cast<float>(cell.y) + 0.5F) * result.selection.cellSizeMeters,
            (static_cast<float>(cell.z) + 0.5F) * result.selection.cellSizeMeters,
        };
        result.selection.bounds.Expand({centre.x - halfCell, centre.y - halfCell, centre.z - halfCell});
        result.selection.bounds.Expand({centre.x + halfCell, centre.y + halfCell, centre.z + halfCell});
    }
    result.diagnostics.emittedCellCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        result.selection.cells.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    result.selection.fingerprint = WaterSeepageSupportSelectionFingerprint(
        result.selection,
        surfaceCache,
        node);
    result.success = true;
    return result;
}

std::vector<WaterSeepageSupportBuildResult>
BuildWaterSeepageSupportSelections(
    std::span<const WaterSeepageNode> nodes,
    std::string_view targetSceneRole,
    const WaterSurfaceCache& surfaceCache,
    const WaterSeepageSupportBuildOptions& options,
    std::size_t maximumParallelBuilds) {
    std::vector<WaterSeepageSupportBuildResult> results(nodes.size());
    if (nodes.empty()) {
        return results;
    }

    // Results use fixed authored indices, so scheduling cannot affect their
    // order, fingerprints, or later atomic publication. Three simultaneous
    // floods provide useful full-site overlap without allowing 18 authored
    // nodes to allocate their bounded Dijkstra maps at once.
    const std::size_t workerCount = std::clamp<std::size_t>(
        maximumParallelBuilds,
        1U,
        std::min<std::size_t>(3U, nodes.size()));
    std::atomic_size_t nextNodeIndex{0U};
    const auto buildNext = [&]() {
        while (options.stopToken == nullptr ||
               !options.stopToken->stop_requested()) {
            const std::size_t nodeIndex =
                nextNodeIndex.fetch_add(1U, std::memory_order_relaxed);
            if (nodeIndex >= nodes.size()) {
                return;
            }
            try {
                results[nodeIndex] = BuildWaterSeepageSupportSelection(
                    nodes[nodeIndex],
                    targetSceneRole,
                    surfaceCache,
                    options);
            } catch (...) {
                // Allocation and container failures must remain ordinary
                // candidate failures. Escaping a std::jthread entry point
                // calls std::terminate and can take down the whole desktop
                // while a large shared surface cache is being traversed.
                auto& failed = results[nodeIndex];
                failed.selection.nodeId = nodes[nodeIndex].id;
                try {
                    failed.errorMessage =
                        "Connected Seepage support generation failed unexpectedly.";
                } catch (...) {
                    // Preserve the no-throw worker boundary even when the
                    // original failure was exhausted memory. Polling also
                    // treats an empty candidate error as a generic failure.
                }
            }
        }
    };
    if (workerCount == 1U) {
        buildNext();
        return results;
    }

    std::vector<std::jthread> workers;
    workers.reserve(workerCount);
    for (std::size_t workerIndex = 0U;
         workerIndex < workerCount;
         ++workerIndex) {
        workers.emplace_back(buildNext);
    }
    // Join before evaluating the return expression; moving `results` while a
    // worker still owns one of its elements would otherwise be a data race.
    workers.clear();
    return results;
}

float EvaluateWaterSeepageSupportCellMask(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageSupportCell& cell) {
    // Exact renderer membership math, excluding only point-normal agreement
    // because a structure-overlay cell has no rendered point normal. Quantize
    // through the compact reference representation so threshold cells match
    // the GPU instead of drifting by a half-float rounding step.
    const auto runCross = UnpackWaterSeepageSupportRunCrossMetrics(
        PackWaterSeepageSupportRunCrossMetrics(
            cell.flowRunMeters,
            cell.crossContourMeters));
    const auto liveMask = EvaluateConnectedSeepageLiveMask(
        node,
        cell.downwardDistanceMeters,
        runCross.flowRunMeters,
        runCross.crossContourMeters,
        SafeSeepageNormal(cell.surfaceNormal),
        cell.upstream,
        kWaterSeepageSupportCellSizeMeters);
    const float packedConfidence =
        UnpackWaterSeepageSupportReferenceMetadata(
            PackWaterSeepageSupportReferenceMetadata(
                cell.surfaceNormal,
                WaterSurfaceRole::None,
                cell.confidence,
                cell.upstream
                    ? kWaterSeepageSupportUpstreamFlag
                    : 0U))
            .confidence;
    const float confidenceMask =
        std::lerp(0.65F, 1.0F, packedConfidence);
    return Clamp01(liveMask.mask * confidenceMask);
}

bool CommitWaterSeepageSupportSelection(
    const WaterSeepageSupportBuildResult& candidate,
    WaterSeepageSupportSelection* settledSelection) {
    if (settledSelection == nullptr || !candidate.success || candidate.cancelled ||
        candidate.diagnostics.cellLimitExceeded || candidate.selection.cells.empty() ||
        candidate.selection.fingerprint.empty()) {
        return false;
    }
    *settledSelection = candidate.selection;
    return true;
}

std::vector<WaterSeepageSurfaceGuide> BuildWaterSeepageSurfaceGuides(
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSceneSupportLayer> supportLayers,
    std::size_t sampleLimit) {
    std::vector<WaterSeepageSurfaceGuide> guides;
    guides.reserve(nodes.size());
    if (nodes.empty()) {
        return guides;
    }

    const std::size_t safeSampleLimit = std::clamp<std::size_t>(sampleLimit, 64U, 450'000U);
    auto rockSupport = BuildSeepageGuideSupportGraph(
        supportLayers,
        "ROCK",
        safeSampleLimit);
    std::unordered_map<std::string, SeepageGuideSupportGraph> roleSupport;
    std::optional<SeepageGuideSupportGraph> allRoleSupport;

    auto supportForRole = [&](std::string_view role) -> const SeepageGuideSupportGraph* {
        const auto normalized = NormalizeSeepageRole(role);
        if (normalized == "rock" && !rockSupport.graph.points.empty()) {
            return &rockSupport;
        }
        const auto existing = roleSupport.find(normalized);
        if (existing != roleSupport.end()) {
            return &existing->second;
        }
        auto [inserted, _] = roleSupport.emplace(
            normalized,
            BuildSeepageGuideSupportGraph(
                supportLayers,
                normalized,
                safeSampleLimit));
        return &inserted->second;
    };

    for (const auto& node : nodes) {
        const SeepageGuideSupportGraph* selectedSupport = nullptr;
        const bool rockPreferred = std::any_of(
            node.targetSceneRoles.begin(),
            node.targetSceneRoles.end(),
            [](const std::string& role) {
                const auto normalized = NormalizeSeepageRole(role);
                return normalized == "rock" || normalized == "veg";
            });
        if (rockPreferred && SeepageGuideGraphSupportsNode(rockSupport, node)) {
            selectedSupport = &rockSupport;
        }
        if (selectedSupport == nullptr) {
            for (const auto& role : node.targetSceneRoles) {
                const auto* candidate = supportForRole(role);
                if (candidate != nullptr && SeepageGuideGraphSupportsNode(*candidate, node)) {
                    selectedSupport = candidate;
                    break;
                }
            }
        }
        if (selectedSupport == nullptr) {
            if (!allRoleSupport.has_value()) {
                allRoleSupport = BuildSeepageGuideSupportGraph(
                    supportLayers,
                    {},
                    safeSampleLimit,
                    true);
            }
            if (SeepageGuideGraphSupportsNode(*allRoleSupport, node)) {
                selectedSupport = &*allRoleSupport;
            }
        }
        guides.push_back(
            selectedSupport == nullptr
                ? WaterSeepageSurfaceGuide{
                      .nodeId = node.id,
                      .requestedReachMeters = std::clamp(
                          SeepageFiniteOr(node.reachMeters, 1.25F),
                          0.001F,
                          1000.0F) * kSeepageMaximumReachScale,
                  }
                : TraceWaterSeepageSurfaceGuide(node, *selectedSupport));
    }
    return guides;
}

std::vector<WaterSeepageSurfaceGuide> BuildWaterSeepageSurfaceGuides(
    std::span<const WaterSeepageNode> nodes,
    const WaterSurfaceCache& surfaceCache) {
    std::vector<WaterSeepageSurfaceGuide> guides;
    guides.reserve(nodes.size());
    for (const auto& node : nodes) {
        guides.push_back(TraceWaterSeepageSurfaceCacheGuide(node, surfaceCache));
    }
    return guides;
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
        const float smoothing = Clamp01(sourceSettings.path.smoothing);
        SmoothWaterPath(&path, smoothing, std::max(0.001F, branch.length));
        if (smoothing > 1.0e-4F && path.size() >= 3U) {
            path = BuildSplineViewSamples(path, 0.0F);
            RecomputePathDistances(&path, std::max(0.001F, branch.length));
        }
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

namespace {

void IncludeTrailSample(WaterTrailOverlay* overlay, WaterTrailSample sample) {
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

template <typename IncludePath>
void ForEachAnchorPath(
    std::span<const WaterOverlayPoint> anchorPoints,
    IncludePath&& includePath) {
    std::vector<WaterOverlayPoint> currentPath;
    float currentFlowId = -1.0F;
    std::size_t pathIndex = 0U;
    const auto flushPath = [&]() {
        if (currentPath.size() >= 2U) {
            includePath(
                std::span<const WaterOverlayPoint>{currentPath},
                pathIndex++);
        }
        currentPath.clear();
    };
    for (const auto& point : anchorPoints) {
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
}

std::vector<std::vector<WaterOverlayPoint>> GroupAnchorPaths(
    std::span<const WaterOverlayPoint> anchorPoints) {
    std::vector<std::vector<WaterOverlayPoint>> paths;
    ForEachAnchorPath(
        anchorPoints,
        [&](std::span<const WaterOverlayPoint> path, std::size_t) {
            paths.emplace_back(path.begin(), path.end());
        });
    return paths;
}

std::vector<std::vector<WaterOverlayPoint>> GroupAnchorPaths(
    const WaterOverlay& pathAnchors) {
    return GroupAnchorPaths(pathAnchors.points);
}

struct PreparedFlowPath {
    const std::vector<WaterOverlayPoint>* anchors = nullptr;
    std::vector<float> cumulativeDistances;
    float lengthMeters = 0.0F;
};

PreparedFlowPath PrepareFlowPath(const std::vector<WaterOverlayPoint>& path) {
    PreparedFlowPath prepared;
    prepared.anchors = &path;
    prepared.cumulativeDistances.resize(path.size(), 0.0F);
    for (std::size_t index = 1U; index < path.size(); ++index) {
        prepared.lengthMeters +=
            glm::length(ToGlm(path[index].position) - ToGlm(path[index - 1U].position));
        prepared.cumulativeDistances[index] = prepared.lengthMeters;
    }
    return prepared;
}

WaterOverlayPoint InterpolatePreparedPathByArcLength(
    const PreparedFlowPath& prepared,
    float distanceMeters) {
    if (prepared.anchors == nullptr || prepared.anchors->empty()) {
        return {};
    }
    const auto& path = *prepared.anchors;
    if (path.size() == 1U || distanceMeters <= 0.0F) {
        return path.front();
    }
    if (distanceMeters >= prepared.lengthMeters) {
        return path.back();
    }

    const auto distanceIt = std::lower_bound(
        prepared.cumulativeDistances.begin() + 1,
        prepared.cumulativeDistances.end(),
        distanceMeters);
    if (distanceIt == prepared.cumulativeDistances.end()) {
        return path.back();
    }
    const auto index = static_cast<std::size_t>(
        std::distance(prepared.cumulativeDistances.begin(), distanceIt));
    const float previousDistance = prepared.cumulativeDistances[index - 1U];
    const float segmentLength = *distanceIt - previousDistance;
    if (segmentLength <= 1.0e-6F) {
        return path[index];
    }
    const float t = std::clamp(
        (distanceMeters - previousDistance) / segmentLength,
        0.0F,
        1.0F);
    auto blended = BlendPathAnchor(path[index - 1U], path[index], t);
    // Spline the position through the neighbouring anchors (with reflected
    // endpoint phantoms) instead of lerping: cached emitter-path anchors are
    // coarse enough that a lerp puts a visible corner at every anchor, and
    // trails then run down chains of straight segments on curvy paths. Scalar
    // fields stay linearly blended. GPU parity: FlowEvaluateAuthored in
    // water_flow_gpu_common.glsl splines both input kinds identically.
    const glm::vec3 p1 = ToGlm(path[index - 1U].position);
    const glm::vec3 p2 = ToGlm(path[index].position);
    const glm::vec3 p0 =
        index >= 2U ? ToGlm(path[index - 2U].position) : p1 + (p1 - p2);
    const glm::vec3 p3 = index + 1U < path.size()
                             ? ToGlm(path[index + 1U].position)
                             : p2 + (p2 - p1);
    blended.position = FromGlm(InterpolateCentripetalCatmullRom(p0, p1, p2, p3, t));
    return blended;
}

glm::vec3 TangentAtPreparedPathDistance(
    const PreparedFlowPath& prepared,
    float distanceMeters,
    float probeMeters) {
    const float safeProbe = std::max(0.005F, probeMeters);
    const float beforeDistance = std::max(0.0F, distanceMeters - safeProbe);
    const float afterDistance = distanceMeters + safeProbe;
    const glm::vec3 before =
        ToGlm(InterpolatePreparedPathByArcLength(prepared, beforeDistance).position);
    const glm::vec3 after =
        ToGlm(InterpolatePreparedPathByArcLength(prepared, afterDistance).position);
    const glm::vec3 tangent = after - before;
    return glm::dot(tangent, tangent) > kNormalEpsilon
               ? glm::normalize(tangent)
               : glm::vec3{1.0F, 0.0F, 0.0F};
}

std::uint8_t TrailColorByte(float value) {
    return FloatToByte(std::clamp(value, 0.0F, 1.0F));
}

float Fract01(float value) {
    return value - std::floor(value);
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
    float trailLooseness) {
    const float attractionRelief = 1.0F - std::clamp(pathAttraction, 0.0F, 1.0F);
    return std::clamp(
        std::clamp(laneCrossing, 0.0F, 1.0F) * 0.38F +
            std::clamp(trailLooseness, 0.0F, 1.0F) * 0.30F +
            attractionRelief * 0.24F +
            std::clamp(turbulence, 0.0F, 1.0F) * 0.18F,
        0.0F,
        1.0F);
}

float LaneAnalysisSpan(
    const LaneAnalysisSample& analysis,
    float fallbackSpan,
    float trailWidthMeters,
    float analysisInfluence) {
    if (!analysis.available) {
        return fallbackSpan;
    }
    const float minimumSpan = std::max(trailWidthMeters * 2.0F, 0.001F);
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

float FlowEndpointFade(float pathDistance, float pathLength, float fadeScaleMeters) {
    if (pathLength <= 1.0e-5F) {
        return 0.0F;
    }
    const float endpointDistance = std::max(
        0.0F,
        std::min(pathDistance, pathLength - pathDistance));
    const float fadeDistance = std::min(
        pathLength * 0.25F,
        std::max(0.02F, fadeScaleMeters));
    return SmoothStep(0.0F, fadeDistance, endpointDistance);
}

float TwoOctaveFlowTurbulence(
    float worldArcDistanceMeters,
    float scaleMeters,
    float primarySeed,
    float secondarySeed) {
    constexpr float kTau = 6.28318530718F;
    const float coordinate = worldArcDistanceMeters / std::max(0.005F, scaleMeters);
    const float primary = std::sin(kTau * (coordinate + primarySeed));
    const float secondary = std::sin(kTau * (coordinate * 2.07F + secondarySeed));
    return primary * 0.68F + secondary * 0.32F;
}

std::vector<WaterOverlayPoint> BuildSurfaceGuidedFlowPath(
    const PreparedFlowPath& authoredPath,
    float sampleSpacingMeters,
    float laneSpanMeters,
    float surfaceFollow,
    float downhillPull,
    float terrainWidthResponse,
    const WaterFlowTrailBuildOptions& options) {
    const auto* cache = options.surfaceCache;
    if (!options.useSurfaceGuide || cache == nullptr || cache->flowSurfaceSurfels.empty() ||
        authoredPath.anchors == nullptr || authoredPath.anchors->size() < 2U ||
        authoredPath.lengthMeters <= 1.0e-5F || surfaceFollow <= 1.0e-5F) {
        return {};
    }

    const auto cancelled = [&options]() {
        return options.stopToken != nullptr && options.stopToken->stop_requested();
    };
    const float resolution = std::clamp(cache->resolutionMeters, 0.001F, 1.0F);
    const float spacing = std::max(0.001F, sampleSpacingMeters);
    const std::uint32_t pointCount = std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(std::ceil(authoredPath.lengthMeters / spacing)) + 1U);

    std::vector<WaterOverlayPoint> guided;
    guided.reserve(pointCount);
    glm::vec3 previousSurfaceNormal{0.0F, 0.0F, 0.0F};
    bool hasPreviousSurfaceNormal = false;
    bool foundSupport = false;
    for (std::uint32_t index = 0U; index < pointCount; ++index) {
        if ((index & 255U) == 0U && cancelled()) {
            return {};
        }
        const float distance = std::min(
            authoredPath.lengthMeters,
            static_cast<float>(index) * spacing);
        auto anchor = InterpolatePreparedPathByArcLength(authoredPath, distance);
        const float authoredLaneSpan =
            anchor.laneSpanMeters >= 0.0F
                ? std::max(0.0F, anchor.laneSpanMeters)
                : std::max(0.0F, laneSpanMeters);
        const float localLaneHalfWidth = authoredLaneSpan * 0.5F;
        // Authored nodes are surface-snapped, so only query the immediately
        // local sheet. The node's resolved cover controls this corridor while
        // the cap prevents a wide lane from causing a cubic neighbourhood
        // scan in the CPU reference implementation.
        const float localLaneSupport =
            std::min(localLaneHalfWidth, resolution * 2.0F);
        const float searchRadius = std::clamp(
            std::max(resolution * 2.5F, localLaneSupport),
            resolution,
            std::max(resolution, 0.10F));
        const glm::vec3 authoredPosition = ToGlm(anchor.position);
        const glm::vec3 authoredNormal = SafeOverlayNormal(ToGlm(anchor.normal));
        const glm::vec3 referenceNormal =
            hasPreviousSurfaceNormal ? previousSurfaceNormal : authoredNormal;
        const auto query = QueryWaterSurfaceCache(
            *cache,
            anchor.position,
            searchRadius,
            FromGlm(referenceNormal));
        if (!query.hit) {
            guided.push_back(anchor);
            continue;
        }

        foundSupport = true;
        glm::vec3 surfaceNormal = SafeOverlayNormal(ToGlm(query.surfel.normal));
        if (glm::dot(surfaceNormal, referenceNormal) < 0.0F) {
            surfaceNormal = -surfaceNormal;
        }
        previousSurfaceNormal = surfaceNormal;
        hasPreviousSurfaceNormal = true;

        glm::vec3 authoredTangent = TangentAtPreparedPathDistance(
            authoredPath,
            distance,
            spacing * 2.0F);
        if (glm::dot(authoredTangent, authoredTangent) <= kNormalEpsilon) {
            authoredTangent = {1.0F, 0.0F, 0.0F};
        } else {
            authoredTangent = glm::normalize(authoredTangent);
        }

        glm::vec3 displacement = ToGlm(query.surfel.centroid) - authoredPosition;
        // Surface guidance may move across the authored corridor but never back
        // along it, preserving the first-to-last route direction.
        displacement -= authoredTangent * glm::dot(displacement, authoredTangent);
        glm::vec3 projectedGravity = kGravity - surfaceNormal * glm::dot(kGravity, surfaceNormal);
        projectedGravity -= authoredTangent * glm::dot(projectedGravity, authoredTangent);
        if (localLaneHalfWidth > 1.0e-6F &&
            glm::dot(projectedGravity, projectedGravity) > kNormalEpsilon) {
            displacement += glm::normalize(projectedGravity) * localLaneHalfWidth * 0.5F *
                            std::clamp(downhillPull, 0.0F, 1.0F);
        }
        if (localLaneHalfWidth <= 1.0e-6F) {
            displacement = {0.0F, 0.0F, 0.0F};
        } else {
            const float displacementLength = glm::length(displacement);
            if (displacementLength > localLaneHalfWidth) {
                displacement *= localLaneHalfWidth / displacementLength;
            }
        }

        const float roughness = Clamp01(query.surfel.roughness);
        const float coherence = Clamp01(query.surfel.normalCoherence);
        const float confidence = Clamp01(query.surfel.confidence);
        const float support = std::clamp(
            confidence * (0.40F + coherence * 0.60F) * (1.0F - roughness * 0.35F),
            0.0F,
            1.0F);
        const float endpointFade = FlowEndpointFade(
            distance,
            authoredPath.lengthMeters,
            std::max(resolution * 2.0F, spacing * 2.0F));
        const float follow = std::clamp(surfaceFollow, 0.0F, 1.0F) * support * endpointFade;
        anchor.position = FromGlm(authoredPosition + displacement * follow);
        anchor.normal = FromGlm(SafeOverlayNormal(glm::mix(authoredNormal, surfaceNormal, follow)));
        anchor.confidence = std::clamp(
            anchor.confidence * (1.0F + (support - 1.0F) * follow),
            0.0F,
            1.0F);
        anchor.surfaceSteepness = std::max(
            anchor.surfaceSteepness,
            Clamp01(1.0F - std::abs(surfaceNormal.z)) * follow);

        // Well-supported, coherent sheets retain the requested lane cover;
        // rough or weak support bunches lanes without exceeding that cover.
        const float terrainSpanScale = std::clamp(
            0.50F + confidence * 0.25F + coherence * 0.30F - roughness * 0.20F,
            0.45F,
            1.0F);
        const float widthInfluence =
            std::clamp(terrainWidthResponse, 0.0F, 1.0F) * follow;
        anchor.width = 1.0F + (terrainSpanScale - 1.0F) * widthInfluence;
        anchor.laneSpanMeters = authoredLaneSpan * anchor.width;
        guided.push_back(anchor);
    }

    return foundSupport ? guided : std::vector<WaterOverlayPoint>{};
}

WaterTrailOverlay BuildTrailOverlayFromPaths(
    const std::vector<std::vector<WaterOverlayPoint>>& paths,
    std::uint32_t requestedTrailCount,
    float trailLengthMeters,
    float pointSpacingMeters,
    float trailWidthMeters,
    float trailStreakLengthMeters,
    bool startFadeEnabled,
    float startFadeFullDistanceMeters,
    float startFadeRandomBeginDistanceMeters,
    bool endFadeEnabled,
    float endFadeFullDistanceMeters,
    float endFadeRandomBeginDistanceMeters,
    float surfaceOffsetMeters,
    float laneSpreadMeters,
    std::uint32_t requestedLaneCount,
    float turbulence,
    float surfaceFollow,
    float downhillPull,
    float terrainWidthResponse,
    float turbulenceScaleMeters,
    float laneCrossing,
    float pathAttraction,
    float trailSmoothness,
    float trailLooseness,
    float speedMetersPerSecond,
    std::uint32_t seed,
    float featureType,
    const WaterPathAnalysisCache* analysisCache,
    const WaterFlowTrailBuildOptions& buildOptions = {}) {
    WaterTrailOverlay overlay;
    if (paths.empty() || requestedTrailCount == 0U) {
        return overlay;
    }
    const auto cancelled = [&buildOptions]() {
        return buildOptions.stopToken != nullptr && buildOptions.stopToken->stop_requested();
    };

    const float safeTrailWidth = std::max(0.0005F, trailWidthMeters);
    const float endpointFadeFlags =
        (startFadeEnabled ? 1.0F : 0.0F) + (endFadeEnabled ? 2.0F : 0.0F);
    const float safeStartFadeFullDistance = std::clamp(
        std::isfinite(startFadeFullDistanceMeters)
            ? startFadeFullDistanceMeters
            : 0.25F,
        0.0F,
        50.0F);
    const float safeStartFadeRandomBeginDistance = std::clamp(
        std::isfinite(startFadeRandomBeginDistanceMeters)
            ? startFadeRandomBeginDistanceMeters
            : 0.10F,
        0.0F,
        50.0F);
    const float safeEndFadeFullDistance = std::clamp(
        std::isfinite(endFadeFullDistanceMeters)
            ? endFadeFullDistanceMeters
            : 0.25F,
        0.0F,
        50.0F);
    const float safeEndFadeRandomBeginDistance = std::clamp(
        std::isfinite(endFadeRandomBeginDistanceMeters)
            ? endFadeRandomBeginDistanceMeters
            : 0.10F,
        0.0F,
        50.0F);
    const float laneSpan = std::max(0.0F, laneSpreadMeters);
    const float lanePitch = std::max(safeTrailWidth * 0.5F, 0.00025F);
    const float safeSurfaceFollow =
        std::clamp(std::isfinite(surfaceFollow) ? surfaceFollow : 0.85F, 0.0F, 1.0F);
    const float safeDownhillPull =
        std::clamp(std::isfinite(downhillPull) ? downhillPull : 0.35F, 0.0F, 1.0F);
    const float safeTerrainWidthResponse = std::clamp(
        std::isfinite(terrainWidthResponse) ? terrainWidthResponse : 0.65F,
        0.0F,
        1.0F);
    const float safeTurbulenceScaleMeters = std::clamp(
        std::isfinite(turbulenceScaleMeters) ? turbulenceScaleMeters : 0.18F,
        0.005F,
        100.0F);
    const float signedSurfaceOffsetMeters =
        std::isfinite(surfaceOffsetMeters) ? surfaceOffsetMeters : 0.0F;
    const float analysisGuideInfluence =
        LaneAnalysisGuideInfluence(turbulence, laneCrossing, pathAttraction, trailLooseness);

    std::vector<PreparedFlowPath> preparedPaths;
    preparedPaths.reserve(paths.size());
    std::vector<float> pathLengths;
    pathLengths.reserve(paths.size());
    std::vector<const WaterPathBranchAnalysis*> pathAnalyses;
    pathAnalyses.reserve(paths.size());
    std::vector<float> pathWeights;
    pathWeights.reserve(paths.size());
    float totalLength = 0.0F;
    float totalWeight = 0.0F;
    for (const auto& path : paths) {
        if (cancelled()) {
            return {};
        }
        auto preparedPath = PrepareFlowPath(path);
        const float length = preparedPath.lengthMeters;
        preparedPaths.push_back(std::move(preparedPath));
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

    const float safeLength = std::clamp(trailLengthMeters, 0.02F, 100.0F);
    const float safeSpacing = std::clamp(pointSpacingMeters, 0.001F, 5.0F);
    const std::uint32_t samplesPerTrail = std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(std::ceil(safeLength / safeSpacing)) + 1U);
    const auto potentialLaneCount =
        requestedLaneCount > 0U
            ? std::max<std::uint32_t>(1U, requestedLaneCount)
            : static_cast<std::uint32_t>(std::max<float>(
                  1.0F,
                  std::ceil(laneSpan / lanePitch)));
    const auto trailAllocations =
        AllocateExactTrailCountsForPaths(pathLengths, pathWeights, requestedTrailCount);
    const float laneCrossingAmount = std::clamp(laneCrossing, 0.0F, 1.0F);
    const auto laneCenter = [](std::uint32_t laneIndex, std::uint32_t laneCount, float span) {
        if (laneCount <= 1U || span <= 1.0e-6F) {
            return 0.0F;
        }
        const auto clampedIndex = std::min(laneIndex, laneCount - 1U);
        return (((static_cast<float>(clampedIndex) + 0.5F) / static_cast<float>(laneCount)) - 0.5F) * span;
    };
    const auto centeredLaneIndex = [](
        std::uint32_t trailOrdinal,
        std::uint32_t laneCount,
        bool positiveSideFirst) {
        if (laneCount <= 1U) {
            return 0U;
        }
        const std::uint32_t slot = trailOrdinal % laneCount;
        const std::uint32_t centerLow = (laneCount - 1U) / 2U;
        const std::uint32_t canonical =
            slot == 0U
                ? centerLow
                : (slot % 2U == 1U
                       ? centerLow + ((slot + 1U) / 2U)
                       : centerLow - (slot / 2U));
        return positiveSideFirst ? canonical : (laneCount - 1U) - canonical;
    };
    std::uint32_t trailId = 1U;

    for (std::size_t pathIndex = 0; pathIndex < paths.size(); ++pathIndex) {
        if (cancelled()) {
            return {};
        }
        const auto& path = paths[pathIndex];
        const auto& authoredPreparedPath = preparedPaths[pathIndex];
        const float authoredPathLength = pathLengths[pathIndex];
        if (path.size() < 2U || authoredPathLength <= 1.0e-5F) {
            continue;
        }
        const std::uint32_t trailsForPath =
            pathIndex < trailAllocations.size() ? trailAllocations[pathIndex] : 0U;
        if (trailsForPath == 0U) {
            continue;
        }
        auto surfaceGuidedPath = BuildSurfaceGuidedFlowPath(
            authoredPreparedPath,
            safeSpacing,
            laneSpan,
            safeSurfaceFollow,
            safeDownhillPull,
            safeTerrainWidthResponse,
            buildOptions);
        auto surfaceGuidedPreparedPath = surfaceGuidedPath.empty()
                                             ? PreparedFlowPath{}
                                             : PrepareFlowPath(surfaceGuidedPath);
        const PreparedFlowPath& preparedPath =
            surfaceGuidedPath.empty() ? authoredPreparedPath : surfaceGuidedPreparedPath;
        const float pathLength = preparedPath.lengthMeters;
        if (pathLength <= 1.0e-5F) {
            continue;
        }
        const auto routeStartIndex = static_cast<std::uint32_t>(std::min<std::size_t>(
            overlay.samples.size(),
            static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
        const auto routePointCount = std::max<std::uint32_t>(
            2U,
            static_cast<std::uint32_t>(std::ceil(pathLength / safeSpacing)) + 1U);
        const auto* analysisBranch = pathIndex < pathAnalyses.size() ? pathAnalyses[pathIndex] : nullptr;
        const auto branchId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(path.front().flowId + 0.5F)));
        const auto sourceId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(path.front().emitterId + 0.5F)));

        for (std::uint32_t routeIndex = 0; routeIndex < routePointCount; ++routeIndex) {
            if ((routeIndex & 255U) == 0U && cancelled()) {
                return {};
            }
            const float routeDistance = std::min(pathLength, static_cast<float>(routeIndex) * safeSpacing);
            WaterOverlayPoint anchor = InterpolatePreparedPathByArcLength(preparedPath, routeDistance);
            const glm::vec3 normal = SafeOverlayNormal(ToGlm(anchor.normal));
            glm::vec3 tangent =
                TangentAtPreparedPathDistance(preparedPath, routeDistance, safeSpacing * 2.0F);
            tangent -= normal * glm::dot(tangent, normal);
            if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
                tangent =
                    TangentAtPreparedPathDistance(preparedPath, routeDistance, safeSpacing * 2.0F);
            }
            tangent = glm::normalize(tangent);
            const auto localAnalysis = SampleLaneAnalysis(analysisBranch, routeDistance);
            const float authoredLocalSpan =
                anchor.laneSpanMeters >= 0.0F
                    ? std::max(0.0F, anchor.laneSpanMeters)
                    : laneSpan;
            float localSpan =
                LaneAnalysisSpan(
                    localAnalysis,
                    authoredLocalSpan,
                    safeTrailWidth,
                    analysisGuideInfluence);
            const float localPitch = LaneAnalysisPitch(localAnalysis, lanePitch, localSpan, potentialLaneCount);
            const float localSpeed = std::max(
                0.01F,
                localAnalysis.available
                    ? speedMetersPerSecond * std::clamp(localAnalysis.speed, 0.10F, 8.0F)
                    : speedMetersPerSecond);

            WaterTrailSample routeSample;
            routeSample.position =
                FromGlm(ToGlm(anchor.position) + normal * signedSurfaceOffsetMeters);
            routeSample.normal = FromGlm(normal);
            routeSample.tangent = FromGlm(tangent);
            routeSample.red = 0U;
            routeSample.green = 0U;
            routeSample.blue = 0U;
            routeSample.trailId = 0.0F;
            routeSample.sourceId = static_cast<float>(sourceId);
            routeSample.pathId = static_cast<float>(pathIndex + 1U);
            routeSample.branchId = static_cast<float>(branchId);
            routeSample.trailDistance = routeDistance;
            routeSample.trailLength = safeLength;
            routeSample.trailSpeed = localSpeed;
            routeSample.trailWidth = std::max(0.0005F, trailWidthMeters);
            routeSample.trailStreakLength = std::max(
                routeSample.trailWidth * 2.0F,
                std::max(trailStreakLengthMeters, safeSpacing * 2.5F));
            routeSample.trailConfidence = std::clamp(anchor.confidence, 0.0F, 1.0F);
            routeSample.wetness = std::clamp(0.35F + anchor.accumulation * 0.65F, 0.0F, 1.0F);
            routeSample.featureType = featureType;
            routeSample.trailRole = 0.0F;
            routeSample.routeStartIndex = static_cast<float>(routeStartIndex);
            routeSample.routePointCount = static_cast<float>(routePointCount);
            routeSample.routeLength = pathLength;
            routeSample.trailStartPhase =
                pathLength > 1.0e-5F ? std::clamp(routeDistance / pathLength, 0.0F, 1.0F) : 0.0F;
            routeSample.trailLaneIndex = 0.0F;
            routeSample.trailLaneCount = static_cast<float>(potentialLaneCount);
            routeSample.trailLanePitch = localPitch;
            routeSample.trailLaneSpan = localSpan;
            routeSample.trailLaneCrossing = laneCrossingAmount;
            routeSample.trailCrossSeed = RegionHash01(seed + branchId, routeIndex, 7029U);
            routeSample.endpointFadeFlags = endpointFadeFlags;
            routeSample.startFadeFullDistanceMeters = safeStartFadeFullDistance;
            routeSample.startFadeRandomBeginDistanceMeters =
                safeStartFadeRandomBeginDistance;
            routeSample.endFadeFullDistanceMeters = safeEndFadeFullDistance;
            routeSample.endFadeRandomBeginDistanceMeters =
                safeEndFadeRandomBeginDistance;
            IncludeTrailSample(&overlay, routeSample);
        }

        for (std::uint32_t laneIndex = 0; laneIndex < trailsForPath && trailId <= requestedTrailCount; ++laneIndex, ++trailId) {
            if (cancelled()) {
                return {};
            }
            const float trailSeed = RegionHash01(seed + branchId, trailId, 7001U);
            const float laneSeed = RegionHash01(seed + branchId, trailId, 7003U);
            const bool positiveSideFirst =
                RegionHash01(seed + branchId, pathIndex + 1U, 7005U) >= 0.5F;
            const std::uint32_t baseLaneIndex = centeredLaneIndex(
                laneIndex,
                potentialLaneCount,
                positiveSideFirst);
            const float baseLaneCenter = laneCenter(baseLaneIndex, potentialLaneCount, laneSpan);
            const float baseLaneUnit = laneCenter(baseLaneIndex, potentialLaneCount, 1.0F);
            const float laneCellWidth =
                potentialLaneCount > 0U ? laneSpan / static_cast<float>(potentialLaneCount) : 0.0F;
            const float laneJitterAmplitude =
                potentialLaneCount <= 1U || laneSpan <= 1.0e-6F
                    ? 0.0F
                    : std::min(lanePitch, laneCellWidth) * 0.18F;
            const float laneJitter =
                (RegionHash01(seed + branchId, trailId, 7011U) - 0.5F) * 2.0F * laneJitterAmplitude;
            const float laneJitterUnit =
                (RegionHash01(seed + branchId, trailId, 7011U) - 0.5F) *
                (potentialLaneCount > 1U ? 0.36F / static_cast<float>(potentialLaneCount) : 0.0F);
            const float laneOffset = baseLaneCenter + laneJitter;
            const float trailCrossSeed = RegionHash01(seed + branchId, trailId, 7027U);
            const float maxStart = std::max(0.0F, pathLength - safeLength);
            const float startJitter = RegionHash01(seed + branchId, trailId, 7009U);
            const float startSlot =
                (static_cast<float>(laneIndex) + startJitter) /
                static_cast<float>(std::max<std::uint32_t>(1U, trailsForPath));
            const float startDistance = maxStart * std::clamp(startSlot, 0.0F, 1.0F);
            const float speed = std::max(0.01F, speedMetersPerSecond * (0.72F + trailSeed * 0.58F));
            for (std::uint32_t sampleIndex = 0; sampleIndex < samplesPerTrail; ++sampleIndex) {
                if ((sampleIndex & 255U) == 0U && cancelled()) {
                    return {};
                }
                const float localDistance = std::min(
                    safeLength,
                    static_cast<float>(sampleIndex) * safeSpacing);
                const float pathDistance = std::min(pathLength, startDistance + localDistance);
                WaterOverlayPoint anchor = InterpolatePreparedPathByArcLength(preparedPath, pathDistance);
                glm::vec3 position = ToGlm(anchor.position);
                const glm::vec3 normal = SafeOverlayNormal(ToGlm(anchor.normal));
                glm::vec3 tangent =
                    TangentAtPreparedPathDistance(preparedPath, pathDistance, safeSpacing * 2.0F);
                tangent -= normal * glm::dot(tangent, normal);
                if (glm::dot(tangent, tangent) <= kNormalEpsilon) {
                    tangent =
                        TangentAtPreparedPathDistance(preparedPath, pathDistance, safeSpacing * 2.0F);
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
                const float authoredLocalSpan =
                    anchor.laneSpanMeters >= 0.0F
                        ? std::max(0.0F, anchor.laneSpanMeters)
                        : laneSpan;
                float localSpan =
                    LaneAnalysisSpan(
                        localAnalysis,
                        authoredLocalSpan,
                        safeTrailWidth,
                        analysisGuideInfluence);
                const float localPitch = LaneAnalysisPitch(localAnalysis, lanePitch, localSpan, potentialLaneCount);
                const float localTurbulence =
                    localAnalysis.available
                        ? std::clamp(
                              turbulence + localAnalysis.turbulence * 0.70F * analysisGuideInfluence,
                              0.0F,
                              5.0F)
                        : std::max(0.0F, turbulence);
                const float motionLooseness =
                    std::clamp(trailLooseness +
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
                const float smoothness = std::clamp(trailSmoothness, 0.0F, 1.0F);
                const float authoredLaneOffset =
                    laneSpan > 1.0e-6F
                        ? laneOffset * (localSpan / laneSpan)
                        : (baseLaneUnit + laneJitterUnit) * localSpan;
                const float dynamicBaseOffset =
                    localAnalysis.available
                        ? (baseLaneUnit + laneJitterUnit) * localSpan * (1.0F - attraction * 0.18F)
                        : authoredLaneOffset;
                const float turbulenceDrive =
                    std::sqrt(std::max(0.0F, localTurbulence)) +
                    localAnalysis.ripplePotential * 0.65F * analysisGuideInfluence;
                const float wobbleAmplitude = std::min(
                    localSpan * 0.22F,
                    localSpan * (0.035F + motionLooseness * 0.060F) * turbulenceDrive);
                const float turbulenceNoise = TwoOctaveFlowTurbulence(
                    pathDistance,
                    safeTurbulenceScaleMeters,
                    trailSeed,
                    laneSeed);
                const float endpointFade = FlowEndpointFade(
                    pathDistance,
                    pathLength,
                    std::max(safeTurbulenceScaleMeters * 0.50F, safeSpacing * 2.0F));
                const float wobble = turbulenceNoise * wobbleAmplitude *
                                     (1.0F - smoothness * 0.45F) * endpointFade;
                const float curl =
                    localAnalysis.available
                        ? std::cos(
                              pathDistance * (2.1F + localAnalysis.eddyPotential * 4.0F) +
                              trailCrossSeed * 6.28318530718F) *
                              localSpan * 0.12F * localAnalysis.eddyPotential *
                              (0.35F + localTurbulence) * analysisGuideInfluence * endpointFade
                        : 0.0F;
                const float crossing =
                    localAnalysis.available
                        ? std::sin(
                              pointAge * 6.28318530718F +
                              trailCrossSeed * 6.28318530718F) *
                              localSpan * 0.10F * laneCrossingAmount *
                              std::max({localAnalysis.confluence, localAnalysis.ripplePotential, localTurbulence}) *
                              analysisGuideInfluence * endpointFade
                        : 0.0F;
                const float lateralLimit =
                    localAnalysis.available
                        ? localSpan * 0.5F *
                              (1.0F + laneCrossingAmount * 0.18F *
                                           std::max({localAnalysis.confluence, localAnalysis.ripplePotential, localTurbulence}) *
                                           analysisGuideInfluence)
                        : localSpan * 0.5F;
                const float lateralOffset =
                    std::clamp(
                        // Keep the authored lane cover at both endpoints.
                        // Endpoint easing is reserved for the non-authored
                        // wobble/curl terms above, so trails start and finish
                        // across the lane instead of converging to a point.
                        dynamicBaseOffset + wobble + curl + crossing,
                        -lateralLimit,
                        lateralLimit);
                position += lateral * lateralOffset;
                position += normal * signedSurfaceOffsetMeters;

                WaterTrailSample sample;
                sample.position = FromGlm(position);
                sample.normal = FromGlm(normal);
                sample.tangent = FromGlm(tangent);
                sample.red = featureType >= 3.0F ? TrailColorByte(0.06F + trailSeed * 0.10F)
                                                  : TrailColorByte(0.04F + trailSeed * 0.12F);
                sample.green = featureType >= 3.0F ? TrailColorByte(0.52F + pointAge * 0.26F)
                                                    : TrailColorByte(0.68F + pointAge * 0.20F);
                sample.blue = TrailColorByte(0.95F + trailSeed * 0.05F);
                sample.trailId = static_cast<float>(trailId);
                sample.sourceId = static_cast<float>(sourceId);
                sample.pathId = static_cast<float>(pathIndex + 1U);
                sample.branchId = static_cast<float>(branchId);
                sample.trailSeed = trailSeed;
                sample.pointSeed = RegionHash01(seed + branchId, trailId + sampleIndex, 7013U);
                sample.trailDistance = localDistance;
                sample.trailLength = safeLength;
                sample.pointAge = pointAge;
                sample.trailAge = RegionHash01(seed + branchId, trailId, 7019U);
                const float localSpeed =
                    localAnalysis.available
                        ? std::max(0.01F, speed * std::clamp(localAnalysis.speed, 0.10F, 8.0F))
                        : speed;
                const float localWidthFactor =
                    localAnalysis.available
                        ? std::clamp(
                              0.78F +
                                  Clamp01(localSpan / std::max(0.001F, laneSpan + safeTrailWidth)) * 0.28F +
                                  localTurbulence * 0.16F,
                              0.55F,
                              1.85F)
                        : 1.0F;
                sample.trailSpeed = localSpeed;
                sample.trailWidth =
                    std::max(0.0005F, trailWidthMeters * (0.80F + trailSeed * 0.42F) * localWidthFactor);
                sample.trailStreakLength = std::max(
                    sample.trailWidth * 2.0F,
                    std::max(trailStreakLengthMeters, safeSpacing * 2.5F));
                sample.trailConfidence = std::clamp(anchor.confidence * (0.72F + 0.28F * trailSeed), 0.0F, 1.0F);
                sample.wetness = std::clamp(0.35F + anchor.accumulation * 0.65F, 0.0F, 1.0F);
                sample.featureType = featureType;
                sample.trailRole = 1.0F;
                sample.routeStartIndex = static_cast<float>(routeStartIndex);
                sample.routePointCount = static_cast<float>(routePointCount);
                sample.routeLength = pathLength;
                sample.trailStartPhase =
                    pathLength > 1.0e-5F ? Fract01(startDistance / pathLength) : 0.0F;
                sample.trailLateralOffset = lateralOffset;
                sample.trailLaneIndex = static_cast<float>(baseLaneIndex);
                sample.trailLaneCount = static_cast<float>(potentialLaneCount);
                sample.trailLanePitch = localPitch;
                sample.trailLaneSpan = localSpan;
                sample.trailLaneCrossing = laneCrossingAmount;
                sample.trailCrossSeed = trailCrossSeed;
                sample.endpointFadeFlags = endpointFadeFlags;
                sample.startFadeFullDistanceMeters = safeStartFadeFullDistance;
                sample.startFadeRandomBeginDistanceMeters =
                    safeStartFadeRandomBeginDistance;
                sample.endFadeFullDistanceMeters = safeEndFadeFullDistance;
                sample.endFadeRandomBeginDistanceMeters =
                    safeEndFadeRandomBeginDistance;
                IncludeTrailSample(&overlay, sample);
            }
        }
    }

    return overlay;
}

}  // namespace

WaterFlowGpuCompactSourceInput BuildWaterFlowGpuSampledSourceInput(
    std::span<const WaterOverlayPoint> anchors,
    const WaterFlowTrailSettings& settings,
    const WaterPathAnalysisCache* analysis) {
    WaterFlowGpuCompactSourceInput source;
    source.points.reserve(anchors.size());
    const float analysisGuideInfluence = LaneAnalysisGuideInfluence(
        settings.turbulence,
        settings.laneCrossing,
        settings.pathAttraction,
        settings.trailLooseness);
    const float laneSpan = std::max(0.0F, settings.laneSpreadMeters);
    bool overflowed = false;

    ForEachAnchorPath(anchors, [&](std::span<const WaterOverlayPoint> path, std::size_t pathIndex) {
        if (overflowed) {
            return;
        }
        auto compact = BuildWaterFlowGpuSampledInput(path);
        if (!compact.Valid()) {
            return;
        }
        if (source.points.size() > std::numeric_limits<std::uint32_t>::max() ||
            compact.points.size() >
                std::numeric_limits<std::uint32_t>::max() - source.points.size()) {
            overflowed = true;
            return;
        }

        const std::uint32_t branchId = path.empty()
                                           ? 0U
                                           : static_cast<std::uint32_t>(std::max(
                                                 0.0F,
                                                 std::floor(path.front().flowId + 0.5F)));
        const auto* analysisBranch = FindPathAnalysisBranch(analysis, branchId);
        WaterFlowGpuCompactBranch branch;
        branch.inputStart = static_cast<std::uint32_t>(source.points.size());
        branch.inputCount = static_cast<std::uint32_t>(compact.points.size());
        branch.branchId = branchId;
        branch.pathId = static_cast<std::uint32_t>(pathIndex + 1U);
        branch.routeLengthMeters = compact.routeLengthMeters;
        branch.allocationWeight =
            compact.routeLengthMeters *
            LaneAnalysisWeight(analysisBranch, laneSpan, analysisGuideInfluence);
        source.branches.push_back(branch);
        source.points.insert(
            source.points.end(),
            compact.points.begin(),
            compact.points.end());
    });
    if (overflowed) {
        return {};
    }
    return source.Valid() ? source : WaterFlowGpuCompactSourceInput{};
}

WaterFlowGpuCompactSourceInput BuildWaterFlowGpuManualSplineSourceInput(
    std::span<const invisible_places::io::Float3> controlPoints,
    std::uint32_t branchId,
    std::uint32_t pathId) {
    return BuildWaterFlowGpuManualSplineSourceInput(
        controlPoints,
        {},
        branchId,
        pathId);
}

WaterFlowGpuCompactSourceInput BuildWaterFlowGpuManualSplineSourceInput(
    std::span<const invisible_places::io::Float3> controlPoints,
    std::span<const WaterManualFlowPathLaneWidth> laneWidths,
    std::uint32_t branchId,
    std::uint32_t pathId) {
    WaterFlowGpuCompactSourceInput source;
    auto compact = BuildWaterFlowGpuManualSplineInput(controlPoints, laneWidths);
    if (!compact.Valid() || compact.points.size() > std::numeric_limits<std::uint32_t>::max()) {
        return source;
    }
    source.points = std::move(compact.points);
    source.branches.push_back({
        .inputStart = 0U,
        .inputCount = static_cast<std::uint32_t>(source.points.size()),
        .branchId = branchId,
        .pathId = std::max(1U, pathId),
        .routeLengthMeters = compact.routeLengthMeters,
        .allocationWeight = compact.routeLengthMeters,
    });
    return source;
}

float ResolveWaterManualFlowPathLaneWidth(
    const WaterManualFlowPathLaneWidth& laneWidth,
    float globalLaneSpanMeters) {
    const float global = std::clamp(
        std::isfinite(globalLaneSpanMeters) ? globalLaneSpanMeters : 0.12F,
        0.0F,
        100.0F);
    const float value = std::clamp(
        std::isfinite(laneWidth.value) ? laneWidth.value : 1.0F,
        0.0F,
        100.0F);
    switch (laneWidth.mode) {
        case WaterManualFlowPathLaneWidthMode::Absolute:
            return value;
        case WaterManualFlowPathLaneWidthMode::Relative:
            return global * value;
        case WaterManualFlowPathLaneWidthMode::Inherit:
        default:
            return global;
    }
}

WaterManualFlowPathLaneWidth ApplyWaterManualFlowPathLaneWidthHandleDrag(
    const WaterManualFlowPathLaneWidth& laneWidth,
    float resolvedWidthMeters,
    float globalLaneSpanMeters) {
    const float resolvedWidth = std::clamp(
        std::isfinite(resolvedWidthMeters) ? resolvedWidthMeters : 0.0F,
        0.0F,
        100.0F);
    const float globalLaneSpan = std::clamp(
        std::isfinite(globalLaneSpanMeters) ? globalLaneSpanMeters : 0.0F,
        0.0F,
        100.0F);
    WaterManualFlowPathLaneWidth updated = laneWidth;
    switch (laneWidth.mode) {
        case WaterManualFlowPathLaneWidthMode::Absolute:
            updated.value = resolvedWidth;
            break;
        case WaterManualFlowPathLaneWidthMode::Relative:
            // A multiplier cannot express a visible width while its global
            // basis is zero. Preserve both its mode and last useful value so
            // it becomes meaningful again when the global width is restored.
            updated.value = globalLaneSpan > 1.0e-5F
                                ? std::clamp(
                                      resolvedWidth / globalLaneSpan,
                                      0.0F,
                                      100.0F)
                                : std::clamp(
                                      std::isfinite(laneWidth.value)
                                          ? laneWidth.value
                                          : 1.0F,
                                      0.0F,
                                      100.0F);
            break;
        case WaterManualFlowPathLaneWidthMode::Inherit:
        default:
            if (globalLaneSpan > 1.0e-5F) {
                updated.mode = WaterManualFlowPathLaneWidthMode::Relative;
                updated.value = std::clamp(
                    resolvedWidth / globalLaneSpan,
                    0.0F,
                    100.0F);
            } else {
                updated.mode = WaterManualFlowPathLaneWidthMode::Absolute;
                updated.value = resolvedWidth;
            }
            break;
    }
    return updated;
}

WaterOverlay BuildManualFlowPathAnchors(
    const WaterManualFlowPathSource& source,
    float sampleSpacingMeters,
    float globalLaneSpanMeters) {
    constexpr float kDuplicateDistance = 1.0e-5F;
    constexpr std::size_t kMaxSampleCount = 32768U;
    std::vector<glm::vec3> controls;
    std::vector<float> controlLaneSpans;
    controls.reserve(source.controlPoints.size());
    controlLaneSpans.reserve(source.controlPoints.size());
    for (std::size_t controlIndex = 0U;
         controlIndex < source.controlPoints.size();
         ++controlIndex) {
        const auto& controlPoint = source.controlPoints[controlIndex];
        const glm::vec3 point = ToGlm(controlPoint);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        if (controls.empty() || glm::length(point - controls.back()) > kDuplicateDistance) {
            controls.push_back(point);
            const WaterManualFlowPathLaneWidth laneWidth =
                controlIndex < source.controlPointLaneWidths.size()
                    ? source.controlPointLaneWidths[controlIndex]
                    : WaterManualFlowPathLaneWidth{};
            controlLaneSpans.push_back(
                ResolveWaterManualFlowPathLaneWidth(
                    laneWidth,
                    globalLaneSpanMeters));
        }
    }
    if (controls.size() < 2U) {
        return {};
    }

    const float spacing = std::clamp(
        std::isfinite(sampleSpacingMeters) ? sampleSpacingMeters : 0.025F,
        0.001F,
        25.0F);
    std::vector<glm::vec3> positions;
    std::vector<float> laneSpans;
    positions.reserve(std::min<std::size_t>(kMaxSampleCount, controls.size() * 16U));
    laneSpans.reserve(std::min<std::size_t>(kMaxSampleCount, controls.size() * 16U));
    positions.push_back(controls.front());
    laneSpans.push_back(controlLaneSpans.front());
    for (std::size_t segmentIndex = 0U;
         segmentIndex + 1U < controls.size() && positions.size() < kMaxSampleCount;
         ++segmentIndex) {
        const glm::vec3 p1 = controls[segmentIndex];
        const glm::vec3 p2 = controls[segmentIndex + 1U];
        const glm::vec3 p0 = segmentIndex > 0U
                                 ? controls[segmentIndex - 1U]
                                 : p1 + (p1 - p2);
        const glm::vec3 p3 = segmentIndex + 2U < controls.size()
                                 ? controls[segmentIndex + 2U]
                                 : p2 + (p2 - p1);
        const float chordLength = glm::length(p2 - p1);
        const std::uint32_t subdivisions = std::clamp<std::uint32_t>(
            static_cast<std::uint32_t>(std::ceil(chordLength / spacing)),
            controls.size() > 2U ? 4U : 1U,
            256U);
        for (std::uint32_t step = 1U;
             step <= subdivisions && positions.size() < kMaxSampleCount;
             ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(subdivisions);
            const glm::vec3 position = controls.size() > 2U
                                           ? InterpolateCentripetalCatmullRom(p0, p1, p2, p3, t)
                                           : glm::mix(p1, p2, t);
            if (glm::length(position - positions.back()) > kDuplicateDistance) {
                positions.push_back(position);
                // Width is an exact node attribute with a bounded C1 blend.
                // Smoothstep prevents the negative/overshooting widths a
                // scalar Catmull interpolation could create while matching
                // the route's control-node boundaries without a visible kink.
                const float eased = t * t * (3.0F - 2.0F * t);
                laneSpans.push_back(std::lerp(
                    controlLaneSpans[segmentIndex],
                    controlLaneSpans[segmentIndex + 1U],
                    eased));
            }
        }
    }
    if (positions.size() < 2U) {
        return {};
    }

    std::vector<glm::vec3> tangents(positions.size(), glm::vec3{1.0F, 0.0F, 0.0F});
    for (std::size_t index = 0U; index < positions.size(); ++index) {
        const glm::vec3 previous = positions[index > 0U ? index - 1U : index];
        const glm::vec3 next = positions[index + 1U < positions.size() ? index + 1U : index];
        const glm::vec3 difference = next - previous;
        if (glm::dot(difference, difference) > kNormalEpsilon) {
            tangents[index] = glm::normalize(difference);
        } else if (index > 0U) {
            tangents[index] = tangents[index - 1U];
        }
    }

    std::vector<glm::vec3> normals(positions.size(), glm::vec3{0.0F, 0.0F, 1.0F});
    const auto perpendicularNormal = [](const glm::vec3& tangent, const glm::vec3& preferred) {
        glm::vec3 normal = preferred - tangent * glm::dot(preferred, tangent);
        if (glm::dot(normal, normal) <= kNormalEpsilon) {
            const glm::vec3 fallback = std::abs(tangent.x) < 0.75F
                                           ? glm::vec3{1.0F, 0.0F, 0.0F}
                                           : glm::vec3{0.0F, 1.0F, 0.0F};
            normal = fallback - tangent * glm::dot(fallback, tangent);
        }
        return glm::normalize(normal);
    };
    normals.front() = perpendicularNormal(tangents.front(), glm::vec3{0.0F, 0.0F, 1.0F});
    for (std::size_t index = 1U; index < normals.size(); ++index) {
        normals[index] = perpendicularNormal(tangents[index], normals[index - 1U]);
        if (glm::dot(normals[index], normals[index - 1U]) < 0.0F) {
            normals[index] = -normals[index];
        }
    }

    WaterOverlay overlay;
    float distance = 0.0F;
    for (std::size_t index = 0U; index < positions.size(); ++index) {
        if (index > 0U) {
            distance += glm::length(positions[index] - positions[index - 1U]);
        }
        WaterOverlayPoint anchor;
        anchor.position = FromGlm(positions[index]);
        anchor.normal = FromGlm(normals[index]);
        anchor.red = 58U;
        anchor.green = 221U;
        anchor.blue = 255U;
        anchor.flowId = static_cast<float>(source.id);
        anchor.emitterId = static_cast<float>(source.id);
        anchor.pathDistance = distance;
        anchor.phase = std::fmod((distance * 0.37F) + (static_cast<float>(source.id) * 0.173F), 1.0F);
        anchor.speed = 1.0F;
        anchor.width = 1.0F;
        anchor.laneSpanMeters =
            index < laneSpans.size()
                ? laneSpans[index]
                : std::clamp(globalLaneSpanMeters, 0.0F, 100.0F);
        anchor.confidence = 1.0F;
        anchor.accumulation = 0.0F;
        anchor.pooling = 0.0F;
        anchor.particleRole = 0.0F;
        IncludeOverlayPoint(&overlay, anchor);
    }
    const float totalLength = std::max(distance, 1.0e-5F);
    for (auto& anchor : overlay.points) {
        anchor.accumulation = Clamp01(anchor.pathDistance / totalLength);
    }
    return overlay;
}

WaterTrailOverlay BuildAnimatedWaterTrailOverlay(
    const std::vector<WaterAnimatedTrailPath>& paths,
    const WaterAnimatedTrailBuildSettings& settings) {
    std::vector<std::vector<WaterOverlayPoint>> groupedPaths;
    groupedPaths.reserve(paths.size());
    for (const auto& path : paths) {
        if (path.anchors.size() >= 2U) {
            groupedPaths.push_back(path.anchors);
        }
    }
    return BuildTrailOverlayFromPaths(
        groupedPaths,
        settings.trailCountTotal,
        settings.trailLengthMeters,
        settings.trailPointSpacingMeters,
        settings.trailWidthMeters,
        settings.trailStreakLengthMeters,
        settings.startFadeEnabled,
        settings.startFadeFullDistanceMeters,
        settings.startFadeRandomBeginDistanceMeters,
        settings.endFadeEnabled,
        settings.endFadeFullDistanceMeters,
        settings.endFadeRandomBeginDistanceMeters,
        settings.surfaceOffsetMeters,
        settings.laneSpreadMeters,
        settings.laneCount,
        settings.turbulence,
        settings.surfaceFollow,
        settings.downhillPull,
        settings.terrainWidthResponse,
        settings.turbulenceScaleMeters,
        settings.laneCrossing,
        settings.pathAttraction,
        settings.trailSmoothness,
        settings.trailLooseness,
        settings.speedMetersPerSecond,
        settings.seed,
        settings.featureType,
        nullptr);
}

WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings) {
    return BuildFlowTrailOverlayFromPathAnchors(pathAnchors, settings, nullptr);
}

WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings,
    const WaterPathAnalysisCache* analysis) {
    return BuildFlowTrailOverlayFromPathAnchors(pathAnchors, settings, analysis, {});
}

WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings,
    const WaterPathAnalysisCache* analysis,
    const WaterFlowTrailBuildOptions& options) {
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
    return BuildTrailOverlayFromPaths(
        flowPaths,
        settings.trailCountTotal,
        settings.trailLengthMeters,
        settings.trailPointSpacingMeters,
        settings.trailWidthMeters,
        settings.trailStreakLengthMeters,
        settings.startFadeEnabled,
        settings.startFadeFullDistanceMeters,
        settings.startFadeRandomBeginDistanceMeters,
        settings.endFadeEnabled,
        settings.endFadeFullDistanceMeters,
        settings.endFadeRandomBeginDistanceMeters,
        settings.surfaceOffsetMeters,
        settings.laneSpreadMeters,
        settings.laneCount,
        settings.turbulence,
        settings.surfaceFollow,
        settings.downhillPull,
        settings.terrainWidthResponse,
        settings.turbulenceScaleMeters,
        settings.laneCrossing,
        settings.pathAttraction,
        settings.trailSmoothness,
        settings.trailLooseness,
        settings.speedMetersPerSecond,
        settings.seed,
        0.0F,
        analysis,
        options);
}

MeshSurfaceCache BuildMeshSurfaceCache(
    const invisible_places::io::LoadedTriangleMesh& mesh,
    const WaterDynamicMeshFlowSettings& settings) {
    const auto startedAt = std::chrono::steady_clock::now();
    MeshSurfaceCache cache;
    cache.meshPath = mesh.sourcePath;
    cache.settings = settings;
    cache.settings.attractors.clear();
    cache.bounds = mesh.bounds;
    cache.sourceVertexCount = static_cast<std::uint64_t>(mesh.vertices.size());
    cache.sourceTriangleCount = static_cast<std::uint64_t>(mesh.triangles.size());
    cache.meshSignature = mesh.sourcePath.generic_string() +
                          "|v=" + std::to_string(cache.sourceVertexCount) +
                          "|t=" + std::to_string(cache.sourceTriangleCount);
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        return cache;
    }

    struct Accumulator {
        glm::vec3 positionSum{0.0F, 0.0F, 0.0F};
        glm::vec3 normalSum{0.0F, 0.0F, 0.0F};
        double localXSum = 0.0;
        double localYSum = 0.0;
        double zSum = 0.0;
        double localXXSum = 0.0;
        double localXYSum = 0.0;
        double localYYSum = 0.0;
        double localXZSum = 0.0;
        double localYZSum = 0.0;
        float minZ = std::numeric_limits<float>::max();
        float maxZ = -std::numeric_limits<float>::max();
        int cellX = 0;
        int cellY = 0;
        std::uint32_t count = 0;
    };

    const float cellSize = std::clamp(settings.cacheCellSizeMeters, 0.005F, 5.0F);
    const float ambiguityHeight = std::max(0.0F, settings.ambiguityHeightMeters);
    std::unordered_map<std::uint64_t, Accumulator> accumulators;
    accumulators.reserve(std::min<std::size_t>(mesh.triangles.size(), 1'000'000U));
    const bool sparseLargeMeshMode = mesh.triangles.size() > 2'000'000U;

    auto addSample = [&](const glm::vec3& position, const glm::vec3& normal) {
        if (!IsValidPoint(position)) {
            return;
        }
        const auto key = MakeXyGridKey(position, cellSize);
        auto& accumulator = accumulators[EncodeTrailSurfaceGridKey(key.x, key.y)];
        if (accumulator.count == 0U) {
            accumulator.cellX = key.x;
            accumulator.cellY = key.y;
        }
        const double cellCenterX = (static_cast<double>(key.x) + 0.5) * static_cast<double>(cellSize);
        const double cellCenterY = (static_cast<double>(key.y) + 0.5) * static_cast<double>(cellSize);
        const double localX = static_cast<double>(position.x) - cellCenterX;
        const double localY = static_cast<double>(position.y) - cellCenterY;
        const double z = static_cast<double>(position.z);
        accumulator.positionSum += position;
        accumulator.normalSum += SafeOverlayNormal(normal);
        accumulator.localXSum += localX;
        accumulator.localYSum += localY;
        accumulator.zSum += z;
        accumulator.localXXSum += localX * localX;
        accumulator.localXYSum += localX * localY;
        accumulator.localYYSum += localY * localY;
        accumulator.localXZSum += localX * z;
        accumulator.localYZSum += localY * z;
        accumulator.minZ = std::min(accumulator.minZ, position.z);
        accumulator.maxZ = std::max(accumulator.maxZ, position.z);
        ++accumulator.count;
    };

    auto barycentric2 = [](const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) {
        const glm::vec2 v0 = b - a;
        const glm::vec2 v1 = c - a;
        const glm::vec2 v2 = p - a;
        const float d00 = glm::dot(v0, v0);
        const float d01 = glm::dot(v0, v1);
        const float d11 = glm::dot(v1, v1);
        const float d20 = glm::dot(v2, v0);
        const float d21 = glm::dot(v2, v1);
        const float denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) <= 1.0e-12F) {
            return glm::vec3{-1.0F, -1.0F, -1.0F};
        }
        const float v = (d11 * d20 - d01 * d21) / denom;
        const float w = (d00 * d21 - d01 * d20) / denom;
        const float u = 1.0F - v - w;
        return glm::vec3{u, v, w};
    };

    auto addSparseTriangleSamples = [&](
                                        const glm::vec3& a,
                                        const glm::vec3& b,
                                        const glm::vec3& c,
                                        const glm::vec3& normal,
                                        const glm::vec2& minXy,
                                        const glm::vec2& maxXy,
                                        std::uint32_t sampleBudget) {
        const glm::vec3 centroid = (a + b + c) / 3.0F;
        sampleBudget = std::max<std::uint32_t>(1U, sampleBudget);
        const float boundsArea = std::max(
            (maxXy.x - minXy.x) * (maxXy.y - minXy.y),
            cellSize * cellSize);
        const float budgetSpacing = std::sqrt(boundsArea / static_cast<float>(sampleBudget));
        const float sparseSpacing = std::clamp(
            std::max(cellSize, budgetSpacing),
            cellSize,
            std::max(cellSize * 16.0F, settings.projectionSearchRadiusMeters * 0.50F));
        const int minSampleX = static_cast<int>(std::floor(minXy.x / sparseSpacing));
        const int maxSampleX = static_cast<int>(std::floor(maxXy.x / sparseSpacing));
        const int minSampleY = static_cast<int>(std::floor(minXy.y / sparseSpacing));
        const int maxSampleY = static_cast<int>(std::floor(maxXy.y / sparseSpacing));
        const int sampleSpanX = std::max(0, maxSampleX - minSampleX + 1);
        const int sampleSpanY = std::max(0, maxSampleY - minSampleY + 1);
        const auto sampleSpan = static_cast<std::uint64_t>(sampleSpanX) * static_cast<std::uint64_t>(sampleSpanY);
        if (sampleSpan == 0U) {
            addSample(centroid, normal);
            return;
        }

        bool sampledInterior = false;
        if (sampleSpan <= static_cast<std::uint64_t>(sampleBudget)) {
            for (int sampleY = minSampleY; sampleY <= maxSampleY; ++sampleY) {
                for (int sampleX = minSampleX; sampleX <= maxSampleX; ++sampleX) {
                    const glm::vec2 p{
                        (static_cast<float>(sampleX) + 0.5F) * sparseSpacing,
                        (static_cast<float>(sampleY) + 0.5F) * sparseSpacing,
                    };
                    const glm::vec3 weights = barycentric2(p, {a.x, a.y}, {b.x, b.y}, {c.x, c.y});
                    if (weights.x < -1.0e-4F || weights.y < -1.0e-4F || weights.z < -1.0e-4F) {
                        continue;
                    }
                    addSample(a * weights.x + b * weights.y + c * weights.z, normal);
                    sampledInterior = true;
                }
            }
        } else {
            const auto latticeSide = static_cast<int>(std::clamp<std::uint32_t>(
                static_cast<std::uint32_t>(std::ceil(std::sqrt(static_cast<float>(sampleBudget) * 2.0F))),
                2U,
                24U));
            std::uint32_t emitted = 0U;
            for (int y = 0; y < latticeSide && emitted < sampleBudget; ++y) {
                for (int x = 0; x < latticeSide - y && emitted < sampleBudget; ++x) {
                    const float u = (static_cast<float>(x) + 0.5F) / static_cast<float>(latticeSide + 1);
                    const float v = (static_cast<float>(y) + 0.5F) / static_cast<float>(latticeSide + 1);
                    if (u + v >= 1.0F) {
                        continue;
                    }
                    addSample(a * (1.0F - u - v) + b * u + c * v, normal);
                    sampledInterior = true;
                    ++emitted;
                }
            }
        }

        if (!sampledInterior) {
            addSample(centroid, normal);
        }
    };

    for (const auto& triangle : mesh.triangles) {
        if (triangle.indices[0] >= mesh.vertices.size() ||
            triangle.indices[1] >= mesh.vertices.size() ||
            triangle.indices[2] >= mesh.vertices.size()) {
            continue;
        }
        const glm::vec3 a = ToGlm(mesh.vertices[triangle.indices[0]]);
        const glm::vec3 b = ToGlm(mesh.vertices[triangle.indices[1]]);
        const glm::vec3 c = ToGlm(mesh.vertices[triangle.indices[2]]);
        glm::vec3 normal = glm::cross(b - a, c - a);
        const float normalLengthSquared = glm::dot(normal, normal);
        if (!std::isfinite(normalLengthSquared) ||
            normalLengthSquared <= kMeshTriangleNormalEpsilon) {
            continue;
        }
        normal = glm::normalize(normal);
        if (normal.z < 0.0F) {
            normal = -normal;
        }

        const glm::vec3 centroid = (a + b + c) / 3.0F;
        const glm::vec2 minXy{
            std::min({a.x, b.x, c.x}),
            std::min({a.y, b.y, c.y}),
        };
        const glm::vec2 maxXy{
            std::max({a.x, b.x, c.x}),
            std::max({a.y, b.y, c.y}),
        };
        const int minCellX = static_cast<int>(std::floor(minXy.x / cellSize));
        const int maxCellX = static_cast<int>(std::floor(maxXy.x / cellSize));
        const int minCellY = static_cast<int>(std::floor(minXy.y / cellSize));
        const int maxCellY = static_cast<int>(std::floor(maxXy.y / cellSize));
        const int cellSpan = std::max(0, maxCellX - minCellX + 1) *
                             std::max(0, maxCellY - minCellY + 1);
        if (sparseLargeMeshMode) {
            const float edgeXy =
                std::max({glm::length(glm::vec2{b.x - a.x, b.y - a.y}),
                          glm::length(glm::vec2{c.x - b.x, c.y - b.y}),
                          glm::length(glm::vec2{a.x - c.x, a.y - c.y})});
            if (cellSpan <= 0 ||
                edgeXy <= std::max(cellSize * 3.0F, settings.projectionSearchRadiusMeters * 0.18F)) {
                addSample(centroid, normal);
            } else {
                const auto sampleBudget = static_cast<std::uint32_t>(std::clamp(
                    static_cast<int>(std::ceil(static_cast<float>(cellSpan) * 0.35F)),
                    48,
                    192));
                addSparseTriangleSamples(a, b, c, normal, minXy, maxXy, sampleBudget);
            }
            continue;
        }
        if (cellSpan <= 0) {
            addSample(centroid, normal);
            continue;
        }
        if (cellSpan > 256) {
            addSparseTriangleSamples(a, b, c, normal, minXy, maxXy, 512U);
            continue;
        }
        bool sampledInterior = false;
        for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
            for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
                const glm::vec2 p{
                    (static_cast<float>(cellX) + 0.5F) * cellSize,
                    (static_cast<float>(cellY) + 0.5F) * cellSize,
                };
                const glm::vec3 weights = barycentric2(p, {a.x, a.y}, {b.x, b.y}, {c.x, c.y});
                if (weights.x < -1.0e-4F || weights.y < -1.0e-4F || weights.z < -1.0e-4F) {
                    continue;
                }
                const glm::vec3 position = a * weights.x + b * weights.y + c * weights.z;
                addSample(position, normal);
                sampledInterior = true;
            }
        }
        if (!sampledInterior) {
            addSample(centroid, normal);
        }
    }

    cache.cells.reserve(accumulators.size());
    cache.cellLookup.reserve(accumulators.size());
    for (const auto& [key, accumulator] : accumulators) {
        if (accumulator.count == 0U) {
            continue;
        }
        MeshSurfaceCacheCell cell;
        const float count = static_cast<float>(accumulator.count);
        cell.cellX = accumulator.cellX;
        cell.cellY = accumulator.cellY;
        cell.position = FromGlm(accumulator.positionSum / count);
        glm::vec3 cellNormal = SafeOverlayNormal(accumulator.normalSum);
        if (accumulator.count >= 3U) {
            const double sampleCount = static_cast<double>(accumulator.count);
            const double centeredXX =
                accumulator.localXXSum - (accumulator.localXSum * accumulator.localXSum) / sampleCount;
            const double centeredXY =
                accumulator.localXYSum - (accumulator.localXSum * accumulator.localYSum) / sampleCount;
            const double centeredYY =
                accumulator.localYYSum - (accumulator.localYSum * accumulator.localYSum) / sampleCount;
            const double centeredXZ =
                accumulator.localXZSum - (accumulator.localXSum * accumulator.zSum) / sampleCount;
            const double centeredYZ =
                accumulator.localYZSum - (accumulator.localYSum * accumulator.zSum) / sampleCount;
            const double determinant = centeredXX * centeredYY - centeredXY * centeredXY;
            const double determinantEpsilon =
                std::max(1.0e-14, std::pow(static_cast<double>(cellSize), 4.0) * 1.0e-6);
            if (determinant > determinantEpsilon) {
                const double slopeX = (centeredXZ * centeredYY - centeredYZ * centeredXY) / determinant;
                const double slopeY = (centeredYZ * centeredXX - centeredXZ * centeredXY) / determinant;
                if (std::isfinite(slopeX) && std::isfinite(slopeY)) {
                    glm::vec3 fittedNormal{
                        static_cast<float>(-slopeX),
                        static_cast<float>(-slopeY),
                        1.0F,
                    };
                    if (glm::dot(fittedNormal, fittedNormal) > kNormalEpsilon) {
                        fittedNormal = glm::normalize(fittedNormal);
                        if (fittedNormal.z < 0.0F) {
                            fittedNormal = -fittedNormal;
                        }
                        if (fittedNormal.z > 0.05F) {
                            cellNormal = fittedNormal;
                        }
                    }
                }
            }
        }
        cell.normal = FromGlm(SafeOverlayNormal(cellNormal));
        const glm::vec3 normal = SafeOverlayNormal(ToGlm(cell.normal));
        glm::vec3 downhill{normal.x * std::max(0.0F, normal.z), normal.y * std::max(0.0F, normal.z), 0.0F};
        if (glm::dot(downhill, downhill) <= kNormalEpsilon) {
            downhill = {1.0F, 0.0F, 0.0F};
        } else {
            downhill = glm::normalize(downhill);
        }
        cell.downhill = FromGlm(downhill);
        cell.minZ = accumulator.minZ;
        cell.maxZ = accumulator.maxZ;
        cell.sampleCount = accumulator.count;
        cell.confidence = Clamp01(std::sqrt(count) * 0.25F);
        cell.ambiguous = (cell.maxZ - cell.minZ) > ambiguityHeight;
        const auto cellIndex = static_cast<std::uint32_t>(
            std::min<std::size_t>(cache.cells.size(), std::numeric_limits<std::uint32_t>::max()));
        cache.cellLookup[key] = cellIndex;
        cache.cells.push_back(cell);
    }
    const int downhillNeighbourRadius = std::clamp(
        static_cast<int>(std::ceil(std::max(settings.projectionSearchRadiusMeters, cellSize) / cellSize)) * 4,
        3,
        16);
    for (auto& cell : cache.cells) {
        const glm::vec3 cellPosition = ToGlm(cell.position);
        glm::vec3 bestLowerDirection{0.0F, 0.0F, 0.0F};
        float bestLowerScore = 0.0F;
        for (int dy = -downhillNeighbourRadius; dy <= downhillNeighbourRadius; ++dy) {
            for (int dx = -downhillNeighbourRadius; dx <= downhillNeighbourRadius; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const auto neighbourIt =
                    cache.cellLookup.find(EncodeTrailSurfaceGridKey(cell.cellX + dx, cell.cellY + dy));
                if (neighbourIt == cache.cellLookup.end() || neighbourIt->second >= cache.cells.size()) {
                    continue;
                }
                const auto& neighbour = cache.cells[neighbourIt->second];
                const glm::vec3 neighbourPosition = ToGlm(neighbour.position);
                glm::vec3 delta = neighbourPosition - cellPosition;
                delta.z = 0.0F;
                const float distance = glm::length(delta);
                if (distance <= 1.0e-5F) {
                    continue;
                }
                const float drop = cellPosition.z - neighbourPosition.z;
                if (drop <= std::max(0.001F, cellSize * 0.015F)) {
                    continue;
                }
                const float score = drop / distance;
                if (score > bestLowerScore) {
                    bestLowerScore = score;
                    bestLowerDirection = delta / distance;
                }
            }
        }
        if (bestLowerScore > 0.0F) {
            const glm::vec3 normalDownhill = ToGlm(cell.downhill);
            glm::vec3 blended = bestLowerDirection * 0.78F + normalDownhill * 0.22F;
            if (glm::dot(blended, blended) > kNormalEpsilon) {
                cell.downhill = FromGlm(glm::normalize(blended));
            } else {
                cell.downhill = FromGlm(bestLowerDirection);
            }
        }
    }
    cache.buildMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - startedAt).count();
    return cache;
}

glm::vec3 MeshSurfaceCellProjectedPosition(
    const MeshSurfaceCacheCell& cell,
    const glm::vec3& query,
    float cellSize,
    float searchRadius) {
    const glm::vec3 anchor = ToGlm(cell.position);
    const glm::vec3 normal = SafeOverlayNormal(ToGlm(cell.normal));
    float projectedZ = anchor.z;
    if (std::abs(normal.z) > 0.05F) {
        const glm::vec2 offset{query.x - anchor.x, query.y - anchor.y};
        const float planeZ = anchor.z - ((normal.x * offset.x) + (normal.y * offset.y)) / normal.z;
        const float maxPlaneDelta = std::max({cellSize * 8.0F, searchRadius, 0.02F});
        if (std::isfinite(planeZ) && std::abs(planeZ - anchor.z) <= maxPlaneDelta) {
            projectedZ = planeZ;
        }
    }
    return {query.x, query.y, projectedZ};
}

MeshSurfaceProjection ProjectToMeshSurface(
    const MeshSurfaceCache& cache,
    const invisible_places::io::Float3& position) {
    MeshSurfaceProjection projection;
    if (cache.cells.empty()) {
        return projection;
    }
    const float cellSize = std::clamp(cache.settings.cacheCellSizeMeters, 0.005F, 5.0F);
    const float searchRadius = std::max(cache.settings.projectionSearchRadiusMeters, cellSize);
    const int cellRadius = std::clamp(
        static_cast<int>(std::ceil(searchRadius / cellSize)),
        1,
        24);
    const glm::vec3 query = ToGlm(position);
    const auto baseKey = MakeXyGridKey(query, cellSize);
    const MeshSurfaceCacheCell* bestCell = nullptr;
    glm::vec3 bestProjectedPosition{0.0F, 0.0F, 0.0F};
    const float maxSearchDistance = searchRadius + cellSize * 0.75F;
    const float maxSearchDistanceSquared = maxSearchDistance * maxSearchDistance;
    float bestScore = std::numeric_limits<float>::max();
    for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
        for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
            const int cellX = baseKey.x + dx;
            const int cellY = baseKey.y + dy;
            const auto key = EncodeTrailSurfaceGridKey(cellX, cellY);
            const auto cellIt = cache.cellLookup.find(key);
            if (cellIt == cache.cellLookup.end() || cellIt->second >= cache.cells.size()) {
                continue;
            }
            const auto& cell = cache.cells[cellIt->second];
            const glm::vec2 cellCenter{
                (static_cast<float>(cellX) + 0.5F) * cellSize,
                (static_cast<float>(cellY) + 0.5F) * cellSize,
            };
            const glm::vec2 centerDelta = cellCenter - glm::vec2{query.x, query.y};
            const float centerDistanceSquared = glm::dot(centerDelta, centerDelta);
            if (centerDistanceSquared > maxSearchDistanceSquared) {
                continue;
            }
            const glm::vec3 projectedPosition =
                MeshSurfaceCellProjectedPosition(cell, query, cellSize, searchRadius);
            const float verticalDelta = projectedPosition.z - query.z;
            const float verticalPenalty = std::min(
                verticalDelta * verticalDelta,
                maxSearchDistanceSquared) * 0.35F;
            const float ringPenalty =
                static_cast<float>(std::abs(dx) + std::abs(dy)) * cellSize * cellSize * 0.08F;
            const float score = centerDistanceSquared + verticalPenalty + ringPenalty;
            if (score < bestScore) {
                bestScore = score;
                bestCell = &cell;
                bestProjectedPosition = projectedPosition;
            }
        }
    }
    if (bestCell == nullptr) {
        return projection;
    }
    projection.position = FromGlm(bestProjectedPosition);
    projection.normal = bestCell->normal;
    projection.downhill = bestCell->downhill;
    projection.confidence = bestCell->confidence;
    projection.ambiguous = bestCell->ambiguous;
    projection.hit = true;
    return projection;
}

MeshSurfaceProjection ProjectRayToMeshSurface(
    const MeshSurfaceCache& cache,
    const invisible_places::io::Float3& rayOrigin,
    const invisible_places::io::Float3& rayDirection) {
    MeshSurfaceProjection projection;
    if (cache.cells.empty()) {
        return projection;
    }

    const glm::vec3 origin = ToGlm(rayOrigin);
    glm::vec3 direction = ToGlm(rayDirection);
    if (!IsValidPoint(origin) ||
        !std::isfinite(direction.x) ||
        !std::isfinite(direction.y) ||
        !std::isfinite(direction.z) ||
        glm::dot(direction, direction) <= kNormalEpsilon ||
        std::abs(direction.z) <= 1.0e-5F) {
        return projection;
    }
    direction = glm::normalize(direction);

    const float cellSize = std::clamp(cache.settings.cacheCellSizeMeters, 0.005F, 5.0F);
    const float searchRadius = std::max(cache.settings.projectionSearchRadiusMeters, cellSize);
    glm::vec3 minimum{
        cache.bounds.minimum.x,
        cache.bounds.minimum.y,
        cache.bounds.minimum.z,
    };
    glm::vec3 maximum{
        cache.bounds.maximum.x,
        cache.bounds.maximum.y,
        cache.bounds.maximum.z,
    };
    if (!cache.bounds.valid) {
        minimum = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        maximum = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
        for (const auto& cell : cache.cells) {
            const glm::vec3 position = ToGlm(cell.position);
            minimum = glm::min(minimum, position);
            maximum = glm::max(maximum, position);
        }
    }
    minimum -= glm::vec3{searchRadius, searchRadius, searchRadius};
    maximum += glm::vec3{searchRadius, searchRadius, searchRadius};

    float tMin = 0.0F;
    float tMax = std::numeric_limits<float>::max();
    auto includeSlab = [&](float slabMinimum, float slabMaximum, float rayValue, float rayDelta) {
        if (std::abs(rayDelta) <= 1.0e-6F) {
            return rayValue >= slabMinimum && rayValue <= slabMaximum;
        }
        float nearT = (slabMinimum - rayValue) / rayDelta;
        float farT = (slabMaximum - rayValue) / rayDelta;
        if (nearT > farT) {
            std::swap(nearT, farT);
        }
        tMin = std::max(tMin, nearT);
        tMax = std::min(tMax, farT);
        return tMin <= tMax;
    };
    if (!includeSlab(minimum.x, maximum.x, origin.x, direction.x) ||
        !includeSlab(minimum.y, maximum.y, origin.y, direction.y) ||
        !includeSlab(minimum.z, maximum.z, origin.z, direction.z) ||
        !std::isfinite(tMin) ||
        !std::isfinite(tMax) ||
        tMax <= 0.0F) {
        return projection;
    }
    tMin = std::max(0.0F, tMin);

    float bestT = tMin;
    float bestResidual = std::numeric_limits<float>::max();
    MeshSurfaceProjection bestProjection;
    constexpr int kRaySampleCount = 32;
    for (int sampleIndex = 0; sampleIndex < kRaySampleCount; ++sampleIndex) {
        const float alpha = kRaySampleCount > 1
                                ? static_cast<float>(sampleIndex) / static_cast<float>(kRaySampleCount - 1)
                                : 0.0F;
        const float t = tMin + (tMax - tMin) * alpha;
        const glm::vec3 query = origin + direction * t;
        auto candidate = ProjectToMeshSurface(cache, FromGlm(query));
        if (!candidate.hit) {
            continue;
        }
        const float residual = std::abs(query.z - candidate.position.z);
        if (residual < bestResidual) {
            bestResidual = residual;
            bestT = t;
            bestProjection = candidate;
        }
    }
    if (!bestProjection.hit) {
        return projection;
    }

    const float residualLimit = std::max({searchRadius * 2.0F, cellSize * 8.0F, 0.05F});
    for (int iteration = 0; iteration < 8; ++iteration) {
        const glm::vec3 query = origin + direction * bestT;
        auto candidate = ProjectToMeshSurface(cache, FromGlm(query));
        if (!candidate.hit) {
            break;
        }
        bestProjection = candidate;
        bestResidual = std::abs(query.z - candidate.position.z);
        const float nextT = (candidate.position.z - origin.z) / direction.z;
        if (!std::isfinite(nextT) || nextT < 0.0F) {
            break;
        }
        const float clampedNextT = std::clamp(nextT, tMin, tMax);
        if (std::abs(clampedNextT - bestT) <= 1.0e-4F) {
            bestT = clampedNextT;
            break;
        }
        bestT = clampedNextT;
    }

    const glm::vec3 finalQuery = origin + direction * bestT;
    auto finalProjection = ProjectToMeshSurface(cache, FromGlm(finalQuery));
    if (!finalProjection.hit) {
        finalProjection = bestProjection;
    } else {
        bestResidual = std::abs(finalQuery.z - finalProjection.position.z);
    }
    if (bestResidual > residualLimit) {
        return projection;
    }
    return finalProjection;
}

glm::vec3 EvaluateDynamicMeshMotionPosition(
    const std::vector<WaterDynamicMeshMotionKeyframe>& keyframes,
    const invisible_places::io::Float3& fallbackPosition,
    float timeSeconds) {
    if (keyframes.empty()) {
        return ToGlm(fallbackPosition);
    }
    const WaterDynamicMeshMotionKeyframe* previous = nullptr;
    const WaterDynamicMeshMotionKeyframe* next = nullptr;
    for (const auto& keyframe : keyframes) {
        if (keyframe.timeSeconds <= timeSeconds &&
            (previous == nullptr || keyframe.timeSeconds >= previous->timeSeconds)) {
            previous = &keyframe;
        }
        if (keyframe.timeSeconds >= timeSeconds &&
            (next == nullptr || keyframe.timeSeconds <= next->timeSeconds)) {
            next = &keyframe;
        }
    }
    if (previous == nullptr) {
        return ToGlm(next != nullptr ? next->position : keyframes.front().position);
    }
    if (next == nullptr) {
        return ToGlm(previous->position);
    }
    const float duration = next->timeSeconds - previous->timeSeconds;
    if (std::abs(duration) <= 1.0e-5F) {
        return ToGlm(next->position);
    }
    const float alpha = std::clamp((timeSeconds - previous->timeSeconds) / duration, 0.0F, 1.0F);
    return glm::mix(ToGlm(previous->position), ToGlm(next->position), alpha);
}

const WaterDynamicMeshEmitterMotion* DynamicMeshEmitterMotionForId(
    const WaterDynamicMeshFlowSettings& settings,
    std::uint32_t emitterId) {
    const auto motionIt = std::find_if(
        settings.emitterMotions.begin(),
        settings.emitterMotions.end(),
        [emitterId](const WaterDynamicMeshEmitterMotion& motion) {
            return motion.enabled && motion.emitterId == emitterId && !motion.keyframes.empty();
        });
    return motionIt == settings.emitterMotions.end() ? nullptr : &*motionIt;
}

WaterTrailOverlay BuildDynamicMeshWaterTrailOverlay(
    const MeshSurfaceCache& cache,
    const std::vector<WaterEmitter>& emitters,
    const WaterDynamicMeshFlowSettings& settings,
    WaterTrailBuildQuality quality,
    WaterDynamicMeshFlowDiagnostics* diagnostics) {
    const auto startedAt = std::chrono::steady_clock::now();
    WaterDynamicMeshFlowDiagnostics localDiagnostics;
    localDiagnostics.cacheBuildMilliseconds = cache.buildMilliseconds;
    localDiagnostics.sourceVertexCount = cache.sourceVertexCount;
    localDiagnostics.sourceTriangleCount = cache.sourceTriangleCount;
    localDiagnostics.cacheCellCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        cache.cells.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    if (diagnostics != nullptr) {
        *diagnostics = localDiagnostics;
    }
    if (!settings.enabled || cache.cells.empty()) {
        return {};
    }

    std::vector<const WaterEmitter*> activeEmitters;
    activeEmitters.reserve(emitters.size());
    for (const auto& emitter : emitters) {
        if (emitter.status != WaterEmitterStatus::Disabled && emitter.strength > 0.0F) {
            activeEmitters.push_back(&emitter);
        }
    }
    if (activeEmitters.empty()) {
        if (diagnostics != nullptr) {
            *diagnostics = localDiagnostics;
        }
        return {};
    }

    const std::uint32_t particleLimit = std::max<std::uint32_t>(
        1U,
        quality == WaterTrailBuildQuality::Preview
            ? settings.previewParticleLimit
            : settings.finalParticleLimit);
    const std::uint32_t maxSteps = std::clamp<std::uint32_t>(
        static_cast<std::uint32_t>(
            std::ceil(std::max(0.02F, settings.trailLengthMeters) /
                      std::max(0.002F, settings.stepMeters))) +
            1U,
        2U,
        quality == WaterTrailBuildQuality::Preview ? 80U : 220U);
    const float stepMeters = std::clamp(settings.stepMeters, 0.002F, 2.0F);
    const float safeInertia = std::clamp(settings.inertia, 0.0F, 0.98F);
    const float animationDurationSeconds = std::max(0.0F, settings.animationDurationSeconds);
    const float sourceVelocityWeight = std::max(0.0F, settings.sourceVelocityWeight);
    const float branchingStrength = std::max(0.0F, settings.branchingStrength);
    const float eddyStrength = std::max(0.0F, settings.eddyStrength);
    const float topologyResponse = std::max(0.0F, settings.topologyResponse);

    std::vector<WaterAnimatedTrailPath> paths;
    paths.reserve(particleLimit);
    const std::vector<WaterDynamicMeshMotionKeyframe> emptyKeyframes;
    for (std::uint32_t particleIndex = 0; particleIndex < particleLimit; ++particleIndex) {
        const auto* emitter = activeEmitters[particleIndex % activeEmitters.size()];
        const auto* emitterMotion = DynamicMeshEmitterMotionForId(settings, emitter->id);
        const float angle = RegionHash01(settings.seed + emitter->id, particleIndex, 12011U) * 6.28318530718F;
        const float radius01 = std::sqrt(RegionHash01(settings.seed + emitter->id, particleIndex, 12017U));
        const float radius = radius01 * std::max(0.001F, emitter->radius);
        glm::vec3 start = EvaluateDynamicMeshMotionPosition(
            emitterMotion != nullptr ? emitterMotion->keyframes : emptyKeyframes,
            emitter->position,
            0.0F) + glm::vec3{
            std::cos(angle) * radius,
            std::sin(angle) * radius,
            0.0F,
        };
        auto projected = ProjectToMeshSurface(cache, FromGlm(start));
        if (!projected.hit) {
            ++localDiagnostics.projectionMissCount;
            continue;
        }
        if (projected.ambiguous) {
            ++localDiagnostics.ambiguousHitCount;
        }

        WaterAnimatedTrailPath path;
        path.motionMode = WaterAnimatedTrailMotionMode::VectorField;
        path.sourceId = emitter->id;
        path.anchors.reserve(maxSteps);
        glm::vec3 position = ToGlm(projected.position);
        glm::vec3 normal = SafeOverlayNormal(ToGlm(projected.normal));
        glm::vec3 velocity = ToGlm(projected.downhill);
        if (glm::dot(velocity, velocity) <= kNormalEpsilon) {
            velocity = {1.0F, 0.0F, 0.0F};
        }
        velocity = glm::normalize(velocity);
        float travelled = 0.0F;

        for (std::uint32_t stepIndex = 0; stepIndex < maxSteps; ++stepIndex) {
            WaterOverlayPoint point;
            point.position = FromGlm(position);
            point.normal = FromGlm(normal);
            point.flowId = static_cast<float>(particleIndex + 1U);
            point.emitterId = static_cast<float>(emitter->id);
            point.pathDistance = travelled;
            point.speed = std::max(0.01F, settings.speedMetersPerSecond * std::max(0.05F, emitter->speed));
            point.width = std::max(0.0005F, settings.trailWidthMeters);
            point.confidence = std::clamp(projected.confidence * emitter->confidence, 0.0F, 1.0F);
            point.accumulation = std::clamp(emitter->strength, 0.0F, 1.0F);
            path.anchors.push_back(point);
            if (stepIndex + 1U >= maxSteps) {
                break;
            }

            const float normalizedStep = maxSteps > 1U
                                             ? static_cast<float>(stepIndex) / static_cast<float>(maxSteps - 1U)
                                             : 0.0F;
            const float normalizedNextStep = maxSteps > 1U
                                                 ? static_cast<float>(stepIndex + 1U) /
                                                       static_cast<float>(maxSteps - 1U)
                                                 : normalizedStep;
            const float motionTime = normalizedStep * animationDurationSeconds;
            const float nextMotionTime = normalizedNextStep * animationDurationSeconds;
            glm::vec3 force = ToGlm(projected.downhill) * std::max(0.0F, settings.downhillWeight);
            if (emitterMotion != nullptr && sourceVelocityWeight > 0.0F) {
                glm::vec3 sourceVelocity =
                    EvaluateDynamicMeshMotionPosition(emitterMotion->keyframes, emitter->position, nextMotionTime) -
                    EvaluateDynamicMeshMotionPosition(emitterMotion->keyframes, emitter->position, motionTime);
                sourceVelocity.z = 0.0F;
                if (glm::dot(sourceVelocity, sourceVelocity) > kNormalEpsilon) {
                    force += glm::normalize(sourceVelocity) * sourceVelocityWeight;
                }
            }
            for (const auto& attractor : settings.attractors) {
                if (!attractor.enabled || attractor.strength <= 0.0F || attractor.radiusMeters <= 1.0e-5F) {
                    continue;
                }
                glm::vec3 toAttractor =
                    EvaluateDynamicMeshMotionPosition(attractor.keyframes, attractor.position, motionTime) -
                    position;
                toAttractor.z = 0.0F;
                const float distance = glm::length(toAttractor);
                if (distance <= 1.0e-5F || distance > attractor.radiusMeters) {
                    continue;
                }
                const float falloff = 1.0F - SmoothStep(0.0F, attractor.radiusMeters, distance);
                force += glm::normalize(toAttractor) *
                         (settings.attractorWeight * attractor.strength * falloff);
            }
            const float curlPhase =
                travelled * 2.7F +
                RegionHash01(settings.seed + emitter->id, particleIndex, 12031U) * 6.28318530718F;
            glm::vec3 lateral = glm::cross(normal, velocity);
            if (glm::dot(lateral, lateral) > kNormalEpsilon) {
                lateral = glm::normalize(lateral);
                const float topologyBranchGate =
                    projected.ambiguous
                        ? 1.0F
                        : std::clamp(0.25F + (1.0F - projected.confidence) * topologyResponse, 0.0F, 1.0F);
                const float branchSign =
                    RegionHash01(settings.seed + emitter->id, particleIndex, stepIndex + 12043U) < 0.5F
                        ? -1.0F
                        : 1.0F;
                force += lateral *
                         (std::sin(curlPhase) * std::max(0.0F, settings.curlStrength) +
                          branchSign * branchingStrength * topologyBranchGate * 0.55F);
                const float eddyPhase =
                    travelled * 5.1F +
                    RegionHash01(settings.seed + emitter->id, particleIndex, 12047U) * 6.28318530718F;
                force -= velocity * (std::max(0.0F, std::sin(eddyPhase)) * eddyStrength * topologyBranchGate);
            }
            force.z = 0.0F;
            if (glm::dot(force, force) <= kNormalEpsilon) {
                force = velocity;
            }
            glm::vec3 nextVelocity = velocity * safeInertia + glm::normalize(force) * (1.0F - safeInertia);
            nextVelocity.z = 0.0F;
            if (glm::dot(nextVelocity, nextVelocity) <= kNormalEpsilon) {
                nextVelocity = velocity;
            }
            nextVelocity = glm::normalize(nextVelocity);
            const glm::vec3 candidate = position + nextVelocity * stepMeters;
            auto nextProjection = ProjectToMeshSurface(cache, FromGlm(candidate));
            if (!nextProjection.hit) {
                ++localDiagnostics.projectionMissCount;
                break;
            }
            if (nextProjection.ambiguous) {
                ++localDiagnostics.ambiguousHitCount;
            }
            const glm::vec3 nextPosition = ToGlm(nextProjection.position);
            travelled += glm::length(nextPosition - position);
            position = nextPosition;
            normal = SafeOverlayNormal(ToGlm(nextProjection.normal));
            velocity = nextVelocity;
            projected = nextProjection;
        }
        if (path.anchors.size() >= 2U) {
            paths.push_back(std::move(path));
        }
    }

    localDiagnostics.emittedPathCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        paths.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    auto overlay = BuildAnimatedWaterTrailOverlay(
        paths,
        {
            .trailCountTotal = static_cast<std::uint32_t>(std::max<std::size_t>(1U, paths.size())),
            .trailLengthMeters = settings.trailLengthMeters,
            .trailPointSpacingMeters = settings.stepMeters,
            .trailWidthMeters = settings.trailWidthMeters,
            .trailStreakLengthMeters = settings.trailStreakLengthMeters,
            .surfaceOffsetMeters = settings.surfaceOffsetMeters,
            .laneSpreadMeters = std::max(settings.trailWidthMeters * 8.0F, settings.stepMeters * 0.75F),
            .turbulence = settings.curlStrength,
            .laneCrossing = std::clamp(settings.attractorWeight * 0.22F, 0.0F, 1.0F),
            .speedMetersPerSecond = settings.speedMetersPerSecond,
            .seed = settings.seed,
            .featureType = kWaterTrailFeatureTypeDynamicMesh,
        });
    localDiagnostics.emittedSampleCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        overlay.samples.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    localDiagnostics.solveMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - startedAt).count();
    if (diagnostics != nullptr) {
        *diagnostics = localDiagnostics;
    }
    return overlay;
}

invisible_places::io::LoadedPointCloud BuildWaterTrailOverlayPointCloud(
    const WaterTrailOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName) {
    return BuildWaterTrailOverlayPointCloud(overlay, sourcePath, layerName, nullptr);
}

invisible_places::io::LoadedPointCloud BuildWaterTrailOverlayPointCloud(
    const WaterTrailOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName,
    const std::stop_token* stopToken) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = sourcePath;
    cloud.layerName = std::string{layerName};
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    cloud.positions.reserve(overlay.samples.size());
    cloud.normals.reserve(overlay.samples.size());
    cloud.packedColors.reserve(overlay.samples.size());
    cloud.bounds = overlay.bounds;
    std::size_t packedSampleIndex = 0U;
    for (const auto& sample : overlay.samples) {
        if ((packedSampleIndex++ & 4095U) == 0U &&
            stopToken != nullptr && stopToken->stop_requested()) {
            return {};
        }
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

    struct TrailScalarField {
        std::string_view name;
        float (*value)(const WaterTrailSample&);
    };
    const TrailScalarField fields[] = {
        {"trail_role", [](const WaterTrailSample& sample) { return sample.trailRole; }},
        {"trail_id", [](const WaterTrailSample& sample) { return sample.trailId; }},
        {"source_id", [](const WaterTrailSample& sample) { return sample.sourceId; }},
        {"path_id", [](const WaterTrailSample& sample) { return sample.pathId; }},
        {"branch_id", [](const WaterTrailSample& sample) { return sample.branchId; }},
        {"trail_seed", [](const WaterTrailSample& sample) { return sample.trailSeed; }},
        {"point_seed", [](const WaterTrailSample& sample) { return sample.pointSeed; }},
        {"trail_distance", [](const WaterTrailSample& sample) { return sample.trailDistance; }},
        {"trail_length", [](const WaterTrailSample& sample) { return sample.trailLength; }},
        {"route_start_index", [](const WaterTrailSample& sample) { return sample.routeStartIndex; }},
        {"route_point_count", [](const WaterTrailSample& sample) { return sample.routePointCount; }},
        {"route_length", [](const WaterTrailSample& sample) { return sample.routeLength; }},
        {"trail_start_phase", [](const WaterTrailSample& sample) { return sample.trailStartPhase; }},
        {"trail_lateral_offset", [](const WaterTrailSample& sample) { return sample.trailLateralOffset; }},
        {"point_age", [](const WaterTrailSample& sample) { return sample.pointAge; }},
        {"trail_age", [](const WaterTrailSample& sample) { return sample.trailAge; }},
        {"trail_speed", [](const WaterTrailSample& sample) { return sample.trailSpeed; }},
        {"trail_width", [](const WaterTrailSample& sample) { return sample.trailWidth; }},
        {"trail_streak_length", [](const WaterTrailSample& sample) { return sample.trailStreakLength; }},
        {"trail_confidence", [](const WaterTrailSample& sample) { return sample.trailConfidence; }},
        {"wetness", [](const WaterTrailSample& sample) { return sample.wetness; }},
        {"feature_type", [](const WaterTrailSample& sample) { return sample.featureType; }},
        {"tangent_x", [](const WaterTrailSample& sample) { return sample.tangent.x; }},
        {"tangent_y", [](const WaterTrailSample& sample) { return sample.tangent.y; }},
        {"tangent_z", [](const WaterTrailSample& sample) { return sample.tangent.z; }},
        {"trail_lane_index", [](const WaterTrailSample& sample) { return sample.trailLaneIndex; }},
        {"trail_lane_count", [](const WaterTrailSample& sample) { return sample.trailLaneCount; }},
        {"trail_lane_pitch", [](const WaterTrailSample& sample) { return sample.trailLanePitch; }},
        {"trail_lane_span", [](const WaterTrailSample& sample) { return sample.trailLaneSpan; }},
        {"trail_lane_crossing", [](const WaterTrailSample& sample) { return sample.trailLaneCrossing; }},
        {"trail_cross_seed", [](const WaterTrailSample& sample) { return sample.trailCrossSeed; }},
        {"endpoint_fade_flags", [](const WaterTrailSample& sample) { return sample.endpointFadeFlags; }},
        {"start_fade_full_distance", [](const WaterTrailSample& sample) { return sample.startFadeFullDistanceMeters; }},
        {"start_fade_random_begin_distance", [](const WaterTrailSample& sample) { return sample.startFadeRandomBeginDistanceMeters; }},
        {"end_fade_full_distance", [](const WaterTrailSample& sample) { return sample.endFadeFullDistanceMeters; }},
        {"end_fade_random_begin_distance", [](const WaterTrailSample& sample) { return sample.endFadeRandomBeginDistanceMeters; }},
    };

    cloud.scalarFields.reserve(std::size(fields));
    cloud.scalarFieldValues.reserve(overlay.samples.size() * std::size(fields));
    for (const auto& field : fields) {
        if (stopToken != nullptr && stopToken->stop_requested()) {
            return {};
        }
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::string{field.name};
        std::size_t fieldSampleIndex = 0U;
        for (const auto& sample : overlay.samples) {
            if ((fieldSampleIndex++ & 4095U) == 0U &&
                stopToken != nullptr && stopToken->stop_requested()) {
                return {};
            }
            const float value = field.value(sample);
            cloud.scalarFieldValues.push_back(value);
            stats.Include(value);
        }
        cloud.scalarFields.push_back(stats);
    }
    return cloud;
}

std::vector<invisible_places::io::ScalarFieldStats> WaterTrailOverlayScalarFieldsForPointCount(
    std::uint64_t pointCount,
    bool includeEndpointFadeFields) {
    constexpr std::string_view names[] = {
        "trail_role",
        "trail_id",
        "source_id",
        "path_id",
        "branch_id",
        "trail_seed",
        "point_seed",
        "trail_distance",
        "trail_length",
        "route_start_index",
        "route_point_count",
        "route_length",
        "trail_start_phase",
        "trail_lateral_offset",
        "point_age",
        "trail_age",
        "trail_speed",
        "trail_width",
        "trail_streak_length",
        "trail_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z",
        "trail_lane_index",
        "trail_lane_count",
        "trail_lane_pitch",
        "trail_lane_span",
        "trail_lane_crossing",
        "trail_cross_seed",
        "endpoint_fade_flags",
        "start_fade_full_distance",
        "start_fade_random_begin_distance",
        "end_fade_full_distance",
        "end_fade_random_begin_distance",
    };

    std::vector<invisible_places::io::ScalarFieldStats> fields;
    const std::size_t fieldCount = includeEndpointFadeFields
                                       ? std::size(names)
                                       : 31U;
    fields.reserve(fieldCount);
    for (const auto name : std::span{names, fieldCount}) {
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::string{name};
        stats.minimum = 0.0F;
        stats.maximum = 1.0F;
        stats.count = static_cast<std::uint64_t>(std::max<std::uint64_t>(1U, pointCount));
        stats.valid = true;
        if (name == "route_start_index" ||
            name == "trail_id" ||
            name == "source_id" ||
            name == "path_id" ||
            name == "branch_id") {
            stats.maximum = static_cast<float>(std::max<std::uint64_t>(1U, pointCount));
        } else if (name == "trail_distance" || name == "trail_length" ||
                   name == "route_length" ||
                   name == "start_fade_full_distance" ||
                   name == "start_fade_random_begin_distance" ||
                   name == "end_fade_full_distance" ||
                   name == "end_fade_random_begin_distance") {
            stats.maximum = 100.0F;
        } else if (name == "endpoint_fade_flags") {
            stats.maximum = 3.0F;
        } else if (name == "tangent_x" || name == "tangent_y" || name == "tangent_z" ||
                   name == "trail_lateral_offset") {
            stats.minimum = -1.0F;
            stats.maximum = 1.0F;
        } else if (name == "feature_type") {
            stats.maximum = kWaterTrailFeatureTypeDynamicMesh;
        }
        fields.push_back(stats);
    }
    return fields;
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
