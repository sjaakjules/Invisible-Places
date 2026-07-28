#include "water/WaterFlow.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
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
    return geometry;
}

WaterTrailGeometrySettings WaterTrailGeometryFromFlowTrailSettings(
    const WaterFlowTrailSettings& settings) {
    WaterTrailGeometrySettings geometry;
    geometry.trailLengthMeters = settings.trailLengthMeters;
    geometry.pointSpacingMeters = settings.trailPointSpacingMeters;
    geometry.widthMeters = settings.trailWidthMeters;
    geometry.streakLengthMeters = settings.trailStreakLengthMeters;
    return geometry;
}

WaterFlowTrailSettings ApplyWaterTrailGeometryToFlowTrailSettings(
    WaterFlowTrailSettings settings,
    const WaterTrailGeometrySettings& geometry) {
    settings.trailLengthMeters = geometry.trailLengthMeters;
    settings.trailPointSpacingMeters = geometry.pointSpacingMeters;
    settings.trailWidthMeters = geometry.widthMeters;
    settings.trailStreakLengthMeters = geometry.streakLengthMeters;
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
    constexpr float kDuplicateDistance = 1.0e-5F;
    constexpr std::uint32_t kArcLengthSubdivisions = 16U;
    constexpr std::uint32_t kSubdivisionsPerCheckpoint = 4U;
    WaterFlowGpuCompactInput input;
    input.points.reserve(controlPoints.size());
    for (const auto& controlPoint : controlPoints) {
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
           left.pointSpacingMeters == right.pointSpacingMeters;
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
    const float patchSize = std::max(0.005F, look.tricklePatchSizeMeters);
    const float trickleWidth = std::max(0.001F, look.trickleWidthMeters);
    // The wetting front travels the same strength- and slope-shaped run the
    // fan mask uses, so the pattern never re-declares its own extent (the
    // node Reach — already spread/rain scaled — is the single area source).
    const float trickleLength = std::max(0.005F, fan.effectiveReach);
    const float frontSoftness = std::max(0.001F, look.trickleFrontSoftness);
    const float downDistance = std::max(0.0F, fan.downDistance);
    const float time = std::max(0.0F, timeSeconds);
    const glm::vec3 advectedPosition =
        position - fan.downTangent * (time * look.downhillDriftMetersPerSecond);
    const std::uint32_t proceduralSeed = node.seed ^ (node.id * 0x9e3779b9U);

    glm::vec3 patchCoordinate = node.noiseRotation * (advectedPosition / patchSize);
    patchCoordinate += glm::vec3{0.019F, -0.031F, 0.023F} * (time * look.evolution);
    const auto patchNoise = SeepageFractalNoise3(
        patchCoordinate,
        proceduralSeed + 6151U,
        node.resolvedQuality);
    const float patchFeather = std::lerp(0.26F, 0.055F, Clamp01(look.contrast));
    const float patchThreshold = 0.78F - density * 0.46F;
    const float patch = SmoothStep(
        patchThreshold - patchFeather,
        patchThreshold + patchFeather,
        patchNoise.value);

    const float frontVariation =
        (patchNoise.value - 0.5F) * frontSoftness * (0.35F + look.breakup * 1.15F);
    const float frontDistance = trickleLength * progress + frontVariation;
    // No separate tail mask: the fan mask's end feather already fades the run
    // out at the effective reach.
    const float arrivalMask = Clamp01(1.0F - SmoothStep(
        frontDistance,
        frontDistance + frontSoftness,
        downDistance));
    if (arrivalMask <= 1.0e-6F) {
        return {};
    }

    glm::vec3 fingerCoordinate{
        fan.signedLateralDistance / trickleWidth,
        downDistance / std::max(patchSize * 1.75F, trickleWidth),
        time * look.evolution * 0.37F,
    };
    fingerCoordinate += patchNoise.gradient * (look.curl * 0.18F);
    const auto fingerNoise = SeepageFractalNoise3(
        fingerCoordinate,
        proceduralSeed + 12289U,
        node.resolvedQuality);
    const float ridge = 1.0F - std::abs(fingerNoise.value * 2.0F - 1.0F);
    const float fingerThreshold = 0.46F + Clamp01(look.breakup) * 0.22F;
    const float fingers = SmoothStep(fingerThreshold, 0.94F, ridge);
    const float seededPatch = Clamp01(
        patch * (0.58F + fingers * 0.42F) + fingers * density * 0.24F);
    const glm::vec3 baseNormal = ResolveSeepagePatternNormal(pointNormal, fan);
    const glm::vec3 environmentDirection = SeepageEnvironmentDirection(look);
    const glm::vec3 worldGradient = glm::transpose(node.noiseRotation) * patchNoise.gradient;
    const float sparseGate = SmoothStep(
        0.90F - look.glintDensity * 0.58F,
        0.98F,
        fingerNoise.value);
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
    const float strengthMask = fan.mask * arrivalMask;
    return {
        .damp = Clamp01(
            strengthMask *
            (wetness * (0.58F + seededPatch * 0.38F) +
             fingers * (0.08F + rainGain * 0.12F))),
        .variation = Clamp01(
            strengthMask * fingers * (0.16F + seededPatch * 0.34F)),
        .glint = Clamp01(
            strengthMask * seededPatch * look.glisten * reflection),
    };
}

WaterSeepageLookSettings SelectSeepageTransitionLook(
    const WaterSeepageRuntimeNode& node,
    const glm::vec3& position) {
    if (!node.transitionLook.has_value() || node.transitionAmount <= 1.0e-6F) {
        return node.look;
    }
    if (node.transitionAmount >= 1.0F - 1.0e-6F) {
        return node.transitionLook.value();
    }
    const float cellSize = std::max(0.005F, std::min(
        node.look.featureSizeMeters,
        node.transitionLook->featureSizeMeters) * 0.18F);
    const glm::ivec3 coordinate = glm::ivec3(glm::floor(position / cellSize));
    const float selector = static_cast<float>(
        SeepageHash3(coordinate, node.seed ^ node.id) & 0x00ffffffU) /
        static_cast<float>(0x01000000U);
    return selector < node.transitionAmount ? node.transitionLook.value() : node.look;
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
    node->look = scenarioState.has_value()
                     ? SanitizeSeepageLook(scenarioState->seepageLook)
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
    const float globalSpread = scenarioState.has_value()
                                   ? Clamp01(SeepageFiniteOr(
                                         scenarioState->seepageSpread,
                                         0.0F))
                                   : 0.0F;
    node->effectiveActivity = nodeState.activity;
    node->localSpread = nodeState.localSpread;
    node->wettingProgress = nodeState.wettingProgress;
    node->strength = node->authoredStrength * scenarioLevel * node->effectiveActivity;
    node->scenarioSpread = Clamp01(
        1.0F - (1.0F - globalSpread) * (1.0F - node->localSpread));
    node->rainVisualStrength = scenarioState.has_value()
                                   ? Clamp01(SeepageFiniteOr(
                                         scenarioState->rainLevel,
                                         0.0F))
                                   : (rainSettings.enabled
                                          ? WaterRainPresetVisualStrength(
                                                rainSettings.intensityPreset)
                                          : 0.0F);
    const float rainGain = SeepageRainGain(*node);
    const float reachScale =
        (1.0F + 0.50F * node->scenarioSpread) * (1.0F + 0.25F * rainGain);
    const float widthScale =
        (1.0F + 0.35F * node->scenarioSpread) * (1.0F + 0.20F * rainGain);
    node->reachMeters = std::clamp(
        node->authoredReachMeters * nodeState.reachScale * reachScale,
        0.0F,
        std::max(0.0F, node->selectionReachLimitMeters));
    node->widthMeters = std::clamp(
        node->authoredWidthMeters * nodeState.widthScale * widthScale,
        0.0F,
        std::max(0.0F, node->selectionWidthLimitMeters));
    node->prominence = std::clamp(
        node->authoredProminence * nodeState.prominence,
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
    if (scenarioState.has_value() && scenarioState->transitionLook.has_value()) {
        node->transitionLook = SanitizeSeepageLook(
            scenarioState->transitionLook.value());
        node->transitionLook->quality = ResolveWaterSeepageQuality(
            node->transitionLook->quality,
            effectivePointInvocations);
        node->transitionAmount = Clamp01(SeepageFiniteOr(
            scenarioState->transitionAmount,
            0.0F));
    }
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
    return "water-dynamic-mesh-flow-settings-v2-" + SeepageFingerprintString(hash);
}

WaterSeepageLookSettings DefaultWaterSeepageLookSettings() {
    return {};
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
        if (left.interpolation == WaterScenarioInterpolation::Smooth) {
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
        if (left.interpolation == WaterScenarioInterpolation::Smooth) {
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
        if (left.interpolation == WaterScenarioInterpolation::Smooth) {
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
            if (std::abs(leftLevel - rightLevel) <= kLevelTolerance) {
                continue;
            }
            anyChange = true;
            // Exact only when this run has its own keys at both endpoints, so
            // the compiled segment coincides with one authored run segment.
            const auto* leftKey = runKeyAt(run, position);
            const auto* rightKey = runKeyAt(run, nextPosition);
            if (leftKey == nullptr || rightKey == nullptr) {
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
                !std::isfinite(cell.lateralDistanceMeters)) {
                continue;
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
                    .lateralDistanceMeters = reference.cell.lateralDistanceMeters,
                    .packedNormalRoleConfidenceFlags =
                        PackWaterSeepageSupportReferenceMetadata(
                            reference.cell.surfaceNormal,
                            reference.sourceRole,
                            reference.cell.confidence,
                            1U),
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

namespace {

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

    auto look = SanitizeSeepageLook(SelectSeepageTransitionLook(node, ToGlm(position)));
    if (look.quality == WaterSeepageQuality::Auto) {
        look.quality = node.resolvedQuality;
    }
    if (look.pattern == WaterSeepagePattern::WettingTrickle &&
        node.wettingProgress <= 1.0e-6F) {
        return contribution;
    }
    SeepagePatternSignals signals;
    switch (look.pattern) {
        case WaterSeepagePattern::WetRockSheen:
            signals = EvaluateWetRockSeepageSignals(
                node,
                look,
                fan,
                ToGlm(position),
                pointNormal,
                timeSeconds,
                viewContext);
            break;
        case WaterSeepagePattern::ChaoticBloom:
            signals = EvaluateChaoticBloomSeepageSignals(
                node,
                look,
                fan,
                ToGlm(position),
                pointNormal,
                timeSeconds,
                viewContext);
            break;
        case WaterSeepagePattern::WettingTrickle:
            signals = EvaluateWettingTrickleSeepageSignals(
                node,
                look,
                fan,
                ToGlm(position),
                pointNormal,
                timeSeconds,
                viewContext);
            break;
    }
    contribution.mask = fan.mask;
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
    sample.downDistance = reference.downwardDistanceMeters;
    sample.lateralDistance = reference.lateralDistanceMeters;
    sample.signedLateralDistance = glm::dot(ToGlm(position) - node.position, node.lateralAxis);
    sample.surfaceNormal = SafeSeepageNormal(metadata.surfaceNormal);
    sample.downTangent = node.downAxis;
    // Least-resistance membership: the cached per-cell flood cost is
    // compared against the live strength-driven budget, and the geodesic
    // path distance keeps a full-strength source patch of the authored
    // Width around the node. Mirrored in SeepageConnectedSupportMask in
    // shaders/pointcloud_sparse_ripple.glsl.
    const float cost = std::max(0.0F, reference.downwardDistanceMeters);
    const float pathDistance = std::max(0.0F, reference.lateralDistanceMeters);
    const float budget = std::max(0.0F, node.budgetMeters);
    sample.effectiveReach = budget;
    sample.effectiveHalfWidth = std::max(node.widthMeters * 0.5F, budget * 0.5F);
    if (budget <= 1.0e-5F) {
        return sample;
    }
    const float feather = std::max(
        std::max(1.0e-5F, node.edgeFeatherMeters),
        budget * 0.15F);
    const float sourceRadius = node.widthMeters * 0.5F;
    const float budgetMask = 1.0F - SmoothStep(
        std::max(0.0F, budget - feather),
        budget,
        cost);
    const float sourceMask = 1.0F - SmoothStep(
        sourceRadius,
        sourceRadius + feather,
        pathDistance);
    const float coreMask = Clamp01(std::max(budgetMask, sourceMask));
    if (coreMask <= 1.0e-6F) {
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
    sample.mask = Clamp01(coreMask * normalMask * confidenceMask);
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
        SeepageFingerprintU32(&nodeHash, 3U);
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
    SeepageFingerprintU32(&hash, 3U);
    SeepageFingerprintText(&hash, normalizedTargetRole);
    SeepageFingerprintU32(
        &hash,
        static_cast<std::uint32_t>(nodeFingerprints.size()));
    for (const auto& fingerprint : nodeFingerprints) {
        SeepageFingerprintText(&hash, fingerprint);
    }
    return "water-seepage-authored-topology-v3-" + SeepageFingerprintString(hash);
}

bool WaterSeepageGridHasActiveViewportEffect(
    const WaterSeepageSpatialGrid& grid) {
    constexpr float activeThreshold = 1.0e-6F;
    return std::any_of(
        grid.nodes.begin(),
        grid.nodes.end(),
        [](const WaterSeepageRuntimeNode& node) {
            if (node.enabledFactor <= activeThreshold ||
                node.reachMeters <= activeThreshold ||
                node.widthMeters <= activeThreshold ||
                node.prominence <= activeThreshold ||
                node.strength <= activeThreshold ||
                node.effectiveActivity <= activeThreshold) {
                return false;
            }
            const auto lookCanContribute = [&](const WaterSeepageLookSettings& look) {
                return look.response.intensity > activeThreshold &&
                       (look.pattern != WaterSeepagePattern::WettingTrickle ||
                        node.wettingProgress > activeThreshold);
            };
            if (!node.transitionLook.has_value() ||
                node.transitionAmount <= activeThreshold) {
                return lookCanContribute(node.look);
            }
            if (node.transitionAmount >= 1.0F - activeThreshold) {
                return lookCanContribute(node.transitionLook.value());
            }
            return lookCanContribute(node.look) ||
                   lookCanContribute(node.transitionLook.value());
        });
}

std::string WaterSeepageTopologyFingerprint(const WaterSeepageSpatialGrid& grid) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 3U);
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
        SeepageFingerprintFloat(&hash, reference.lateralDistanceMeters);
        SeepageFingerprintU32(&hash, reference.packedNormalRoleConfidenceFlags);
    }
    return "water-seepage-topology-v3-" + SeepageFingerprintString(hash);
}

std::string WaterSeepageParamsFingerprint(const WaterSeepageSpatialGrid& grid) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 5U);
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
    }
    return "water-seepage-params-v5-" + SeepageFingerprintString(hash);
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
using SeepageSupportCellKey = std::tuple<std::int32_t, std::int32_t, std::int32_t>;

const WaterSurfaceSurfel* FindSeepageSupportSurfel(
    const WaterSurfaceCache& cache,
    const SeepageSupportSourceKey& key) {
    const auto found = std::lower_bound(
        cache.flowSurfaceSurfels.begin(),
        cache.flowSurfaceSurfels.end(),
        key,
        [](const WaterSurfaceSurfel& surfel, const SeepageSupportSourceKey& candidate) {
            return std::tie(surfel.cellX, surfel.cellY, surfel.cellZ, surfel.role) < candidate;
        });
    if (found == cache.flowSurfaceSurfels.end() ||
        std::tie(found->cellX, found->cellY, found->cellZ, found->role) != key) {
        return nullptr;
    }
    return &*found;
}

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
    // Accumulated least-resistance cost and geodesic distance from the node.
    float costMeters = 0.0F;
    float pathMeters = 0.0F;
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
                   left.surfel->role) >
               std::tie(
                   right.surfel->cellX,
                   right.surfel->cellY,
                   right.surfel->cellZ,
                   right.surfel->role);
    }
};

std::string WaterSeepageSupportSelectionFingerprint(
    const WaterSeepageSupportSelection& selection,
    const WaterSurfaceCache& cache,
    const WaterSeepageNode& node) {
    std::uint64_t hash = 1469598103934665603ULL;
    SeepageFingerprintU32(&hash, 1U);
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
        SeepageFingerprintFloat(&hash, cell.lateralDistanceMeters);
        SeepageFingerprintFloat(&hash, cell.surfaceNormal.x);
        SeepageFingerprintFloat(&hash, cell.surfaceNormal.y);
        SeepageFingerprintFloat(&hash, cell.surfaceNormal.z);
        SeepageFingerprintFloat(&hash, cell.confidence);
    }
    // v2: least-resistance flood cost/path metrics replaced the
    // centreline corridor.
    // v3: contour step cost scales with surface steepness (stored per-cell
    // costs change), so cached support selections must rebuild.
    return "water-seepage-support-v3-" + SeepageFingerprintString(hash);
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
        return result;
    }

    const glm::vec3 nodePosition = ToGlm(node.position);
    const glm::vec3 nodeNormal = SafeSeepageNormal(node.surfaceNormal);
    glm::vec3 downAxis = ToGlm(DeriveWaterSeepageDownAxis(
        node.surfaceNormal,
        node.downAxis));
    if (!IsValidPoint(downAxis) || glm::dot(downAxis, downAxis) <= kNormalEpsilon) {
        downAxis = kGravity;
    } else {
        downAxis = glm::normalize(downAxis);
    }
    glm::vec3 lateralAxis = glm::cross(nodeNormal, downAxis);
    if (!IsValidPoint(lateralAxis) || glm::dot(lateralAxis, lateralAxis) <= kNormalEpsilon) {
        const glm::vec3 helper = std::abs(nodeNormal.x) < 0.8F
                                     ? glm::vec3{1.0F, 0.0F, 0.0F}
                                     : glm::vec3{0.0F, 1.0F, 0.0F};
        lateralAxis = glm::cross(nodeNormal, helper);
    }
    lateralAxis = glm::normalize(lateralAxis);

    // Least-resistance flood: a Dijkstra expansion over connected surfels
    // where each step costs its length scaled by how slow seeping wetness
    // moves in that direction — steep descent is cheap, contouring is
    // expensive, and climbing is very expensive. Water therefore splits into
    // every available downhill path (left AND right of a saddle), runs
    // further down steeper routes, spreads evenly but shortly across flat
    // ground, and wicks a short way up connected structure above the node.
    // The stored per-cell cost is compared against the live strength-driven
    // budget at render time, so the area reshapes without rebuilding.
    const auto stepCostFactor = [](const glm::vec3& step,
                                   float stepLength,
                                   const glm::vec3& surfaceNormal) {
        const float drop = -step.z / std::max(stepLength, 1.0e-6F);
        // Contouring gets more expensive the steeper the local surface, and
        // the descent discount sharpens with it: on a near-vertical face
        // only a genuinely steep step is descent-priced (shallow diagonals
        // no longer smear the fan sideways at the cheap rate), so seeping
        // wetness runs a long way down before it slowly widens. Gentle
        // terrain keeps the forgiving threshold and its even, short spread.
        const float steepness =
            1.0F - std::clamp(std::abs(surfaceNormal.z), 0.0F, 1.0F);
        const float descentSharpness = std::lerp(2.0F, 1.0F, steepness);
        const float blend =
            std::clamp(drop * descentSharpness, -1.0F, 1.0F);
        const float contourFactor =
            kWaterSeepageContourCostFactor *
            std::lerp(
                1.0F,
                kWaterSeepageSteepContourMultiplier,
                steepness);
        return blend >= 0.0F
                   ? std::lerp(
                         contourFactor,
                         kWaterSeepageDescentCostFactor,
                         blend)
                   : std::lerp(
                         contourFactor,
                         kWaterSeepageAscentCostFactor,
                         -blend);
    };
    const float depthTolerance = std::clamp(
        SeepageFiniteOr(node.depthToleranceMeters, 0.15F),
        sourceResolution * 0.50F,
        2.0F);
    const float edgeAllowance = std::clamp(
        SeepageFiniteOr(node.edgeFeatherMeters, 0.10F),
        0.0F,
        std::max(result.selection.reachLimitMeters,
                 result.selection.widthLimitMeters));
    const float reachLimit = result.selection.reachLimitMeters;
    const float halfWidthLimit = result.selection.widthLimitMeters * 0.5F;
    // The flood stops at the cost limit: the selection reach limit expressed
    // in cost-metres plus the feather allowance priced at the steepest
    // contour rate, so the live budget always has selected cells to feather
    // across even on a near-vertical face.
    const float costLimit =
        reachLimit +
        edgeAllowance * kWaterSeepageContourCostFactor *
            kWaterSeepageSteepContourMultiplier;
    const float continuityDistance = std::clamp(
        depthTolerance,
        sourceResolution * 1.75F,
        sourceResolution * 6.0F);
    const float continuityDistanceSquared = continuityDistance * continuityDistance;

    std::priority_queue<
        PendingSeepageSupportSurfel,
        std::vector<PendingSeepageSupportSurfel>,
        PendingSeepageSupportSurfelGreater>
        pending;
    std::map<SeepageSupportSourceKey, float> queuedCost;
    std::set<SeepageSupportSourceKey> visited;
    std::map<SeepageSupportCellKey, WaterSeepageSupportCell> emitted;
    std::vector<const WaterSurfaceSurfel*> acceptedSubstrateSurfels;
    std::vector<std::pair<float, float>> acceptedSubstrateCostPath;
    const bool targetIsVegetation =
        NormalizeSeepageRole(targetSceneRole) == "veg";
    const auto emitSourceCell = [&](std::int32_t sourceX,
                                    std::int32_t sourceY,
                                    std::int32_t sourceZ,
                                    float downwardDistance,
                                    float lateralDistance,
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
                    const WaterSeepageSupportCell cell{
                        .x = static_cast<std::int32_t>(x),
                        .y = static_cast<std::int32_t>(y),
                        .z = static_cast<std::int32_t>(z),
                        .downwardDistanceMeters = downwardDistance,
                        .lateralDistanceMeters = lateralDistance,
                        .surfaceNormal = surfaceNormal,
                        .confidence = Clamp01(confidence),
                    };
                    const auto [it, inserted] = emitted.emplace(key, cell);
                    if (!inserted && cell.confidence > it->second.confidence) {
                        it->second = cell;
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
    const auto* startSurfel = FindSeepageSupportSurfel(surfaceCache, startKey);
    if (startSurfel == nullptr) {
        result.errorMessage = "The shared-cache Seepage seed cell was not addressable.";
        return result;
    }
    pending.push({.surfel = startSurfel, .costMeters = 0.0F, .pathMeters = 0.0F});
    queuedCost.emplace(startKey, 0.0F);

    while (!pending.empty()) {
        if (options.stopToken != nullptr && options.stopToken->stop_requested()) {
            result.cancelled = true;
            result.errorMessage = "Connected Seepage support selection was cancelled.";
            return result;
        }
        if (visited.size() >= maximumVisited) {
            result.diagnostics.cellLimitExceeded = true;
            result.errorMessage =
                "Connected Seepage support exceeded its bounded surfel budget; "
                "reduce Selection Reach/Width limits.";
            return result;
        }
        const auto current = pending.top();
        pending.pop();
        const auto& surfel = *current.surfel;
        const auto currentKey = SeepageSupportSourceKey{
            surfel.cellX,
            surfel.cellY,
            surfel.cellZ,
            surfel.role,
        };
        if (!visited.insert(currentKey).second) {
            continue;
        }
        ++result.diagnostics.visitedSurfelCount;

        const glm::vec3 centroid = ToGlm(surfel.centroid);
        bool accepted = true;
        // Cells are kept while the flood cost stays inside the reach limit
        // OR the geodesic path stays inside the Source Width patch bound
        // (the width limit), so the always-wet patch is never clipped by
        // ascent pricing right above the node.
        if (current.costMeters > costLimit &&
            current.pathMeters > halfWidthLimit + edgeAllowance) {
            ++result.diagnostics.rejectedReachCount;
            accepted = false;
        } else if (surfel.confidence < 0.10F || surfel.normalCoherence < 0.10F) {
            ++result.diagnostics.rejectedContinuityCount;
            accepted = false;
        }

        if (accepted) {
            ++result.diagnostics.acceptedSurfelCount;
            acceptedSubstrateSurfels.push_back(&surfel);
            acceptedSubstrateCostPath.emplace_back(
                current.costMeters,
                current.pathMeters);
            if (!targetIsVegetation &&
                !emitSourceCell(
                    surfel.cellX,
                    surfel.cellY,
                    surfel.cellZ,
                    current.costMeters,
                    current.pathMeters,
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
                    const auto key = SeepageSupportSourceKey{
                        surfel.cellX + dx,
                        surfel.cellY + dy,
                        surfel.cellZ + dz,
                        sourceRole.value(),
                    };
                    if (visited.contains(key)) {
                        continue;
                    }
                    const auto* neighbour = FindSeepageSupportSurfel(surfaceCache, key);
                    if (neighbour == nullptr) {
                        continue;
                    }
                    const glm::vec3 step = ToGlm(neighbour->centroid) - centroid;
                    const float distanceSquared = glm::dot(step, step);
                    const glm::vec3 neighbourNormal = SafeSeepageNormal(neighbour->normal);
                    if (!std::isfinite(distanceSquared) ||
                        distanceSquared > continuityDistanceSquared ||
                        std::abs(glm::dot(currentNormal, neighbourNormal)) < 0.20F) {
                        ++result.diagnostics.rejectedContinuityCount;
                        continue;
                    }
                    const float stepLength = std::sqrt(distanceSquared);
                    const float nextCost =
                        current.costMeters +
                        stepLength *
                            stepCostFactor(step, stepLength, neighbourNormal);
                    if (const auto existing = queuedCost.find(key);
                        existing != queuedCost.end() &&
                        existing->second <= nextCost + 0.005F) {
                        continue;
                    }
                    pending.push({
                        .surfel = neighbour,
                        .costMeters = nextCost,
                        .pathMeters = current.pathMeters + stepLength,
                    });
                    queuedCost.insert_or_assign(key, nextCost);
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
        const auto minimumCellX = SeepageCellCoordinate(
            nodePosition.x - selectionRadius,
            sourceResolution);
        const auto maximumCellX = SeepageCellCoordinate(
            nodePosition.x + selectionRadius,
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
            const auto& [substrateCost, substratePath] =
                acceptedSubstrateCostPath[*closestIndex];
            const float climbPenalty =
                std::max(0.0F, riseAboveSubstrate) *
                kWaterSeepageAscentCostFactor;
            const float vegetationConfidence = std::clamp(
                static_cast<float>(vegetation->sampleCount) / 8.0F,
                0.0F,
                1.0F);
            if (!emitSourceCell(
                    vegetation->cellX,
                    vegetation->cellY,
                    vegetation->cellZ,
                    substrateCost + climbPenalty,
                    substratePath + std::max(0.0F, riseAboveSubstrate),
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
        return result;
    }
    result.selection.cells.reserve(emitted.size());
    const float halfCell = result.selection.cellSizeMeters * 0.5F;
    for (const auto& [_, cell] : emitted) {
        result.selection.cells.push_back(cell);
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

float EvaluateWaterSeepageSupportCellMask(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageSupportCell& cell) {
    // Same budget/source-patch math as EvaluateConnectedSeepageSupportMask
    // (minus the per-point normal-agreement term the overlay cannot know).
    const float cost = std::max(0.0F, cell.downwardDistanceMeters);
    const float pathDistance = std::max(0.0F, cell.lateralDistanceMeters);
    const float budget = std::max(0.0F, node.budgetMeters);
    if (budget <= 1.0e-5F) {
        return 0.0F;
    }
    const float feather = std::max(
        std::max(1.0e-5F, node.edgeFeatherMeters),
        budget * 0.15F);
    const float sourceRadius = node.widthMeters * 0.5F;
    const float budgetMask = 1.0F - SmoothStep(
        std::max(0.0F, budget - feather),
        budget,
        cost);
    const float sourceMask = 1.0F - SmoothStep(
        sourceRadius,
        sourceRadius + feather,
        pathDistance);
    const float coreMask = Clamp01(std::max(budgetMask, sourceMask));
    const float confidenceMask = std::lerp(0.65F, 1.0F, Clamp01(cell.confidence));
    return Clamp01(coreMask * confidenceMask);
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

std::vector<invisible_places::io::Float3> BuildWaterRegionBoundary(
    const std::vector<invisible_places::io::Float3>& vertices);

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

std::uint8_t TrailColorByte(float value) {
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

int RippleCellCoordinate(float value) {
    // Ripple hashes use signed 32-bit cell coordinates to match their shader
    // counterparts. Leave enough headroom for every neighbouring-cell probe;
    // extreme authored coordinates are bounded deterministically instead of
    // overflowing during float-to-int conversion or neighbour arithmetic.
    constexpr int kNeighbourHeadroom = 8;
    if (!std::isfinite(value)) {
        return 0;
    }
    const double floored = std::floor(static_cast<double>(value));
    const double minimum = static_cast<double>(std::numeric_limits<int>::min() + kNeighbourHeadroom);
    const double maximum = static_cast<double>(std::numeric_limits<int>::max() - kNeighbourHeadroom);
    return static_cast<int>(std::clamp(floored, minimum, maximum));
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
    const int cellX = RippleCellCoordinate(uv.x / safeCellSize);
    const int cellY = RippleCellCoordinate(uv.y / safeCellSize);
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

    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
    const int cellX = RippleCellCoordinate(p.x);
    const int cellY = RippleCellCoordinate(p.y);
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
    const int cellX = RippleCellCoordinate(uv.x / safeCellSize);
    const int cellY = RippleCellCoordinate(uv.y / safeCellSize);
    return RuntimeRippleCellHash(cellX, cellY, seed, salt);
}

float RuntimeRippleSmoothBlockNoise(const glm::vec2& uv, float cellSize, float seed, float salt) {
    const float safeCellSize = std::max(0.001F, cellSize);
    const glm::vec2 p = uv / safeCellSize;
    const int cellX = RippleCellCoordinate(p.x);
    const int cellY = RippleCellCoordinate(p.y);
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

    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
        const float speedNoise = std::lerp(0.91F, 1.09F, RuntimeRippleHash(slotSeed * 0.097F + 23.0F));
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
            const float returnFade = 1.0F - SmoothStep(0.45F, 1.0F, returnProgress);
            const float value =
                heldFoam * remainingMask * (0.50F + density01 * 0.30F) * breakup * returnFade;
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
    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
    const int baseAX = RippleCellCoordinate(pA.x);
    const int baseAY = RippleCellCoordinate(pA.y);
    const int baseBX = RippleCellCoordinate(pB.x);
    const int baseBY = RippleCellCoordinate(pB.y);
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
    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
    glm::vec2 currentUv = uv;
    currentUv.x += normalBias * wavelength * (0.10F + warp * 0.06F);
    currentUv.y += std::sin(uv.x / std::max(wavelength * 1.25F, 0.010F) + seed * 1.17F + t * 0.13F) *
                  wavelength * (0.055F + warp * 0.060F + normalBias * 0.035F);
    currentUv.y += std::sin(uv.x / std::max(wavelength * 0.47F, 0.006F) - seed * 0.73F - t * 0.19F) *
                  wavelength * turbulence * 0.055F;
    const glm::vec2 p{currentUv.x / cellXSize, currentUv.y / cellYSize};
    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
            const glm::vec2 local = currentUv - origin;
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
                currentUv + glm::vec2{t * 0.018F + pulseSeed, -t * 0.011F},
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
        currentUv + glm::vec2{t * 0.014F, -t * 0.009F},
        std::max(wavelength * 0.42F, 0.006F),
        seed,
        239.0F);
    const float fallbackPulse =
        RippleWavePeak(
            currentUv.x / std::max(wavelength * 1.9F, 0.012F) - t * 0.22F + fallbackNoise,
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
    const int baseX = RippleCellCoordinate(p.x);
    const int baseY = RippleCellCoordinate(p.y);
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
    const float laneHalfWidth = std::max(0.0F, laneSpanMeters) * 0.5F;
    // Authored nodes are surface-snapped, so only query the immediately local
    // sheet. Keeping this independent of a very wide lane cover prevents a
    // large cubic neighbourhood scan in the CPU reference implementation.
    const float localLaneSupport = std::min(laneHalfWidth, resolution * 2.0F);
    const float searchRadius = std::clamp(
        std::max(resolution * 2.5F, localLaneSupport),
        resolution,
        std::max(resolution, 0.10F));
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
        anchor.width = 1.0F;
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
        if (laneHalfWidth > 1.0e-6F && glm::dot(projectedGravity, projectedGravity) > kNormalEpsilon) {
            displacement += glm::normalize(projectedGravity) * laneHalfWidth * 0.5F *
                            std::clamp(downhillPull, 0.0F, 1.0F);
        }
        if (laneHalfWidth <= 1.0e-6F) {
            displacement = {0.0F, 0.0F, 0.0F};
        } else {
            const float displacementLength = glm::length(displacement);
            if (displacementLength > laneHalfWidth) {
                displacement *= laneHalfWidth / displacementLength;
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
            float localSpan =
                LaneAnalysisSpan(localAnalysis, laneSpan, safeTrailWidth, analysisGuideInfluence);
            if (!surfaceGuidedPath.empty()) {
                localSpan = std::min(localSpan, laneSpan) *
                            std::clamp(anchor.width, 0.45F, 1.0F);
            }
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
                float localSpan =
                    LaneAnalysisSpan(localAnalysis, laneSpan, safeTrailWidth, analysisGuideInfluence);
                if (!surfaceGuidedPath.empty()) {
                    localSpan = std::min(localSpan, laneSpan) *
                                std::clamp(anchor.width, 0.45F, 1.0F);
                }
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
                const float dynamicBaseOffset =
                    localAnalysis.available
                        ? (baseLaneUnit + laneJitterUnit) * localSpan * (1.0F - attraction * 0.18F)
                        : (!surfaceGuidedPath.empty()
                               ? (baseLaneUnit + laneJitterUnit) * localSpan
                               : laneOffset);
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
                        dynamicBaseOffset * endpointFade + wobble + curl + crossing,
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
    WaterFlowGpuCompactSourceInput source;
    auto compact = BuildWaterFlowGpuManualSplineInput(controlPoints);
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

WaterOverlay BuildManualFlowPathAnchors(
    const WaterManualFlowPathSource& source,
    float sampleSpacingMeters) {
    constexpr float kDuplicateDistance = 1.0e-5F;
    constexpr std::size_t kMaxSampleCount = 32768U;
    std::vector<glm::vec3> controls;
    controls.reserve(source.controlPoints.size());
    for (const auto& controlPoint : source.controlPoints) {
        const glm::vec3 point = ToGlm(controlPoint);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        if (controls.empty() || glm::length(point - controls.back()) > kDuplicateDistance) {
            controls.push_back(point);
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
    positions.reserve(std::min<std::size_t>(kMaxSampleCount, controls.size() * 16U));
    positions.push_back(controls.front());
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
    overlay.fieldDiagnostics.inputNodeCount = localDiagnostics.cacheCellCount;
    overlay.fieldDiagnostics.emittedPathCount = localDiagnostics.emittedPathCount;
    overlay.fieldDiagnostics.emittedSampleCount = localDiagnostics.emittedSampleCount;
    overlay.fieldDiagnostics.lowConfidenceTerminationCount = localDiagnostics.projectionMissCount;
    overlay.fieldDiagnostics.lowConfidenceFadeCount = localDiagnostics.ambiguousHitCount;
    if (diagnostics != nullptr) {
        *diagnostics = localDiagnostics;
    }
    return overlay;
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

void FilterVisibleTrailSamplesToRegionBoundary(
    WaterTrailOverlay* overlay,
    const std::vector<invisible_places::io::Float3>& boundary) {
    if (overlay == nullptr || overlay->samples.empty() || boundary.size() < 3U) {
        return;
    }

    WaterTrailOverlay filtered;
    filtered.fieldDiagnostics = overlay->fieldDiagnostics;
    filtered.samples.reserve(overlay->samples.size());
    for (const auto& sample : overlay->samples) {
        if (sample.trailRole >= 0.5F && !PointInPolygonXy(ToGlm(sample.position), boundary)) {
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

WaterTrailOverlay BuildFieldTrailOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldTrailSettings& settings) {
    if (!settings.enabled || fieldCache.nodes.size() < 2U) {
        return {};
    }
    std::vector<std::vector<WaterOverlayPoint>> paths;
    std::vector<WaterOverlayPoint> path;
    path.reserve(fieldCache.nodes.size());
    float distance = 0.0F;
    WaterFieldTrailDiagnostics diagnostics;
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
            .trailCountTotal = settings.trailCount,
            .trailLengthMeters = settings.trailLengthMeters,
            .trailPointSpacingMeters = settings.trailPointSpacingMeters,
            .trailWidthMeters = settings.trailWidthMeters,
            .trailStreakLengthMeters = settings.trailStreakLengthMeters,
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
    FilterVisibleTrailSamplesToRegionBoundary(&overlay, fieldCache.regionBoundary);
    return overlay;
}

WaterTrailOverlay BuildFieldTrailOverlay(
    const WaterFieldCache& fieldCache,
    const WaterFieldTrailSettings& settings,
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
        std::max(0.003F, settings.trailPointSpacingMeters) * 2.0F);
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
        return BuildFieldTrailOverlay(fieldCache, settings);
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
            std::max(settings.trailPointSpacingMeters * 4.0F, fieldCache.settings.fieldResolutionMeters * 4.0F));
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
        return BuildFieldTrailOverlay(fieldCache, settings);
    }

    WaterFieldTrailDiagnostics diagnostics;
    diagnostics.inputNodeCount = static_cast<std::uint32_t>(std::min<std::size_t>(
        fieldCache.nodes.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    const float bridgeStart = std::max(0.0F, fieldCache.settings.fieldResolutionMeters * 1.25F);
    const float maxBridgeDistance = std::max(
        fieldCache.settings.maxBridgeDistanceMeters,
        fieldCache.settings.fieldResolutionMeters * (1.25F + fieldCache.settings.bridgeAggression * 4.0F));
    const float confidenceThreshold = std::clamp(fieldCache.settings.surfaceConfidenceThreshold, 0.0F, 1.0F);
    const std::uint32_t requestedPathCount = std::clamp<std::uint32_t>(
        std::max<std::uint32_t>(1U, settings.trailCount / 4U),
        1U,
        96U);
    const std::uint32_t maxSteps = std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(
            std::ceil(std::max(0.02F, settings.trailLengthMeters) /
                      std::max(0.001F, settings.trailPointSpacingMeters))) +
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
            .trailCountTotal = settings.trailCount,
            .trailLengthMeters = settings.trailLengthMeters,
            .trailPointSpacingMeters = settings.trailPointSpacingMeters,
            .trailWidthMeters = settings.trailWidthMeters,
            .trailStreakLengthMeters = settings.trailStreakLengthMeters,
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
    FilterVisibleTrailSamplesToRegionBoundary(&overlay, fieldCache.regionBoundary);
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

WaterRippleRuntimeParams BuildWaterRippleRuntimeParams(const WaterEffectLayer& layer) {
    WaterRegionSelection selection;
    selection.layerId = layer.id;
    selection.featureType = layer.featureType;
    selection.targetLayerSourcePath = layer.targetLayerSourcePath;
    selection.targetLayerKey = layer.targetLayerSourcePath.generic_string();
    selection.boundary = BuildWaterRegionBoundary(layer.vertices);
    selection.hull = layer.hull.empty() ? BuildWaterRegionHull(layer.vertices) : layer.hull;
    for (const auto& vertex : selection.boundary) {
        selection.bounds.Expand(vertex);
    }
    return BuildWaterRippleRuntimeParams(layer, selection);
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
    std::uint64_t pointCount) {
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
    };

    std::vector<invisible_places::io::ScalarFieldStats> fields;
    fields.reserve(std::size(names));
    for (const auto name : names) {
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
        } else if (name == "trail_distance" || name == "trail_length" || name == "route_length") {
            stats.maximum = 100.0F;
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
