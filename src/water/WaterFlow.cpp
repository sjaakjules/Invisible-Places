#include "water/WaterFlow.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
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

bool SamePathGenerationSettings(
    const WaterPathGenerationSettings& left,
    const WaterPathGenerationSettings& right) {
    return left.legacyScaleMode == right.legacyScaleMode &&
           left.supportVoxelSize == right.supportVoxelSize &&
           left.maxBridgeDistance == right.maxBridgeDistance &&
           left.smoothing == right.smoothing &&
           left.pathLength == right.pathLength &&
           left.pathSampleSpacing == right.pathSampleSpacing &&
           left.maxSteps == right.maxSteps &&
           left.supportSampleLimit == right.supportSampleLimit;
}

glm::vec3 ToGlm(const invisible_places::io::Float3& point) {
    return {point.x, point.y, point.z};
}

invisible_places::io::Float3 FromGlm(const glm::vec3& point) {
    return {point.x, point.y, point.z};
}

float Clamp01(float value) {
    return std::clamp(value, 0.0F, 1.0F);
}

float PositiveOr(float value, float fallback) {
    return std::isfinite(value) && value > 0.0F ? value : fallback;
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

std::optional<std::uint32_t> ChooseDownhillNeighbour(
    const SupportGraph& graph,
    std::uint32_t currentIndex,
    const WaterBakeSettings& settings,
    const std::vector<std::uint32_t>& visited) {
    if (currentIndex >= graph.points.size()) {
        return std::nullopt;
    }

    const auto& current = graph.points[currentIndex];
    const auto direction = FlowDirection(current);
    const float searchRadius = std::max(settings.maxBridgeDistance, settings.supportVoxelSize * 2.0F);
    const auto candidates = NearbySupportIndices(graph, current.position, searchRadius);

    std::optional<std::uint32_t> bestIndex;
    float bestScore = -std::numeric_limits<float>::max();
    for (const auto candidateIndex : candidates) {
        if (candidateIndex == currentIndex || candidateIndex >= graph.points.size()) {
            continue;
        }
        if (std::find(visited.begin(), visited.end(), candidateIndex) != visited.end()) {
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
        const float bridgePenalty = distance / std::max(0.01F, searchRadius);
        const float normalCoherence = current.hasNormal && candidate.hasNormal
                                          ? Clamp01((glm::dot(current.normal, candidate.normal) + 1.0F) * 0.5F)
                                          : 0.55F;
        const float score =
            (downhillScore * 3.2F) +
            (alignment * 1.9F) +
            (candidate.confidence * 1.6F) +
            (normalCoherence * 0.55F) -
            (bridgePenalty * 0.85F) -
            uphillPenalty;
        if (score > bestScore && (zDrop > -settings.maxBridgeDistance * 0.08F || score > 1.6F)) {
            bestScore = score;
            bestIndex = candidateIndex;
        }
    }

    return bestIndex;
}

void IncludeOverlayPoint(WaterOverlay* overlay, WaterOverlayPoint point) {
    if (overlay == nullptr) {
        return;
    }
    overlay->bounds.Expand(point.position);
    overlay->points.push_back(point);
}

float Hash01(std::uint32_t value);
WaterOverlayPoint BlendPathAnchor(
    const WaterOverlayPoint& left,
    const WaterOverlayPoint& right,
    float amount);

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
    const std::vector<WaterOverlayPoint>& anchors) {
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
        const std::uint32_t subdivisions = std::clamp<std::uint32_t>(
            static_cast<std::uint32_t>(std::ceil(segmentLength / std::max(0.03F, segmentLength * 0.25F))),
            3U,
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
    point.pathDistance = left.pathDistance + ((right.pathDistance - left.pathDistance) * t);
    point.width = left.width + ((right.width - left.width) * t);
    point.confidence = left.confidence + ((right.confidence - left.confidence) * t);
    point.accumulation = left.accumulation + ((right.accumulation - left.accumulation) * t);
    point.pooling = left.pooling + ((right.pooling - left.pooling) * t);
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

void IncludeWaterPathWithParticles(
    WaterOverlay* overlay,
    std::vector<WaterOverlayPoint> path,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings) {
    if (overlay == nullptr || path.empty()) {
        return;
    }

    path = ResampleSplineAnchors(path, trailShapeSettings.splineAnchorSpacing);
    if (path.empty()) {
        return;
    }

    IncludeWaterPathViewAnchors(overlay, BuildSplineViewSamples(path));

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
        IncludeOverlayPoint(overlay, anchor);
    }

    if (pathPointCount < 2U) {
        return;
    }

    const float density = std::clamp(animationTrailSettings.particleDensity, 0.05F, 10.0F);
    const float spacing = std::max(0.001F, PositiveOr(trailShapeSettings.splineAnchorSpacing, 0.5F));
    constexpr std::uint32_t maxParticleCount = 8192U;
    const std::uint32_t particleCount = std::clamp<std::uint32_t>(
        static_cast<std::uint32_t>(std::ceil(std::max(pathLength, spacing) / spacing * density)),
        1U,
        maxParticleCount);
    const float jitter = std::clamp(trailShapeSettings.particleJitter, 0.0F, 3.0F);
    const float speed = std::clamp(animationTrailSettings.particleSpeed, 0.05F, 8.0F);

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
        WaterOverlayPoint particle = InterpolatePathAnchor(path, phase);
        particle.phase = phase;
        particle.speed = std::max(0.02F, particle.speed * speed * (0.82F + seed * 0.36F));
        particle.width = jitter;
        particle.particleRole = 1.0F;
        particle.pathStartIndex = static_cast<float>(pathStartIndex);
        particle.pathPointCount = static_cast<float>(pathPointCount);
        particle.jitterSeed = seed;
        ApplyParticleBlue(&particle, seed, animationTrailSettings.colorVariation);
        IncludeOverlayPoint(overlay, particle);
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
            settings.supportVoxelSize = 1.25F;
            settings.maxBridgeDistance = 4.0F;
            settings.smoothing = 0.65F;
            settings.pathLength = 180.0F;
            settings.pathSampleSpacing = 1.2F;
            settings.maxSteps = 260;
            settings.supportSampleLimit = 160000;
            break;
        case WaterScaleMode::Detail:
            settings.supportVoxelSize = 0.012F;
            settings.maxBridgeDistance = 0.055F;
            settings.smoothing = 0.28F;
            settings.pathLength = 3.5F;
            settings.pathSampleSpacing = 0.012F;
            settings.maxSteps = 220;
            settings.supportSampleLimit = 220000;
            break;
        case WaterScaleMode::Mid:
            break;
    }
    return settings;
}

WaterSourceSettings DefaultWaterSourceSettings(WaterScaleMode mode) {
    WaterSourceSettings settings;
    settings.path = DefaultWaterPathGenerationSettings(mode);
    return settings;
}

WaterAnimationTrailSettings DefaultWaterAnimationTrailSettings() {
    return {};
}

WaterVisualSettings DefaultWaterVisualSettings() {
    return {};
}

WaterSettingsBundle DefaultWaterSettingsBundle(WaterScaleMode mode) {
    WaterSettingsBundle settings;
    settings.path = DefaultWaterPathGenerationSettings(mode);
    return settings;
}

WaterBakeSettings DefaultWaterBakeSettings(WaterScaleMode mode) {
    return DefaultWaterPathGenerationSettings(mode);
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

WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterPathGenerationSettings& settings) {
    WaterOverlay overlay;
    if (cloud.positions.empty() || emitters.empty()) {
        return overlay;
    }

    auto graph = BuildSupportGraph(cloud, settings);
    if (graph.points.empty()) {
        return overlay;
    }

    const std::uint32_t maxSteps = settings.maxSteps;
    const float pathLength = settings.pathLength;
    const float pathSampleSpacing = std::max(0.005F, settings.pathSampleSpacing);
    const float surfaceLift = std::max(0.003F, settings.supportVoxelSize * 0.18F);

    std::uint32_t flowId = 0;
    for (const auto& emitter : emitters) {
        if (emitter.status == WaterEmitterStatus::Disabled) {
            continue;
        }
        const auto startIndex = NearestSupportIndex(
            graph,
            ToGlm(emitter.position),
            std::max(settings.maxBridgeDistance * 4.0F, emitter.radius * 2.0F));
        if (!startIndex.has_value() || startIndex.value() >= graph.points.size()) {
            continue;
        }

        ++flowId;
        std::vector<std::uint32_t> visited;
        visited.reserve(maxSteps);
        std::uint32_t currentIndex = startIndex.value();
        float distanceAlongPath = 0.0F;
        float confidence = Clamp01(emitter.confidence);
        std::vector<WaterOverlayPoint> pathPoints;
        pathPoints.reserve(maxSteps * 2U);

        for (std::uint32_t step = 0; step < maxSteps && distanceAlongPath < pathLength; ++step) {
            if (currentIndex >= graph.points.size()) {
                break;
            }
            visited.push_back(currentIndex);
            const auto& current = graph.points[currentIndex];
            const auto nextIndex = ChooseDownhillNeighbour(graph, currentIndex, settings, visited);
            if (!nextIndex.has_value() || nextIndex.value() >= graph.points.size()) {
                const glm::vec3 lift = current.hasNormal ? current.normal * surfaceLift : glm::vec3{0.0F, 0.0F, surfaceLift};
                pathPoints.push_back(
                    MakeOverlayPoint(
                        current.position + lift,
                        emitter,
                        flowId,
                        distanceAlongPath,
                        pathLength,
                        confidence * current.confidence,
                        0.85F,
                        std::max(pathSampleSpacing, emitter.radius * 0.55F)));
                break;
            }

            const auto& next = graph.points[nextIndex.value()];
            const glm::vec3 segment = next.position - current.position;
            const float segmentLength = SafeLength(segment);
            if (segmentLength <= 1.0e-5F) {
                break;
            }

            const float drop = current.position.z - next.position.z;
            const float pooling = Clamp01(1.0F - (std::abs(drop) / std::max(0.01F, settings.maxBridgeDistance * 0.35F)));
            const std::uint32_t segmentSamples = std::max<std::uint32_t>(
                1U,
                static_cast<std::uint32_t>(std::ceil(segmentLength / pathSampleSpacing)));
            for (std::uint32_t sample = 0; sample <= segmentSamples; ++sample) {
                const float t = static_cast<float>(sample) / static_cast<float>(segmentSamples);
                const glm::vec3 position = current.position + segment * t;
                const glm::vec3 normal = current.hasNormal ? current.normal : (next.hasNormal ? next.normal : glm::vec3{0.0F, 0.0F, 1.0F});
                const glm::vec3 lift = normal * surfaceLift;
                const float sampleDistance = distanceAlongPath + segmentLength * t;
                const float width =
                    std::max(pathSampleSpacing, emitter.radius * (0.35F + emitter.strength * 0.28F)) *
                    (0.7F + Clamp01(sampleDistance / std::max(0.001F, pathLength)) * 0.55F + pooling * 0.35F);
                pathPoints.push_back(
                    MakeOverlayPoint(
                        position + lift,
                        emitter,
                        flowId,
                        sampleDistance,
                        pathLength,
                        confidence * ((current.confidence * (1.0F - t)) + (next.confidence * t)),
                        pooling,
                        width));
            }

            confidence *= 0.985F;
            if (segmentLength > settings.maxBridgeDistance * 0.85F) {
                confidence *= 0.82F;
            }
            distanceAlongPath += segmentLength;
            currentIndex = nextIndex.value();
        }

        SmoothWaterPath(&pathPoints, settings.smoothing, pathLength);
        IncludeWaterPathAnchorsOnly(&overlay, std::move(pathPoints));
    }

    return overlay;
}

WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings) {
    struct EmitterGroup {
        WaterSourceSettings settings{};
        std::vector<WaterEmitter> emitters;
    };

    std::vector<EmitterGroup> groups;
    for (const auto& emitter : emitters) {
        if (emitter.status == WaterEmitterStatus::Disabled) {
            continue;
        }
        const auto& settings = ResolveWaterSourceSettings(emitter, emitters, defaultSettings);
        auto groupIt = std::find_if(
            groups.begin(),
            groups.end(),
            [&](const EmitterGroup& group) {
                return SamePathGenerationSettings(group.settings.path, settings.path);
            });
        if (groupIt == groups.end()) {
            EmitterGroup group;
            group.settings = settings;
            group.emitters.push_back(emitter);
            groups.push_back(std::move(group));
        } else {
            groupIt->emitters.push_back(emitter);
        }
    }

    WaterOverlay overlay;
    float flowIdOffset = 0.0F;
    for (const auto& group : groups) {
        auto groupAnchors = GenerateWaterPathAnchors(cloud, group.emitters, group.settings.path);
        float maxGroupFlowId = 0.0F;
        for (auto& point : groupAnchors.points) {
            maxGroupFlowId = std::max(maxGroupFlowId, point.flowId);
            point.flowId += flowIdOffset;
            IncludeOverlayPoint(&overlay, point);
        }
        flowIdOffset += maxGroupFlowId + 1.0F;
    }
    return overlay;
}

WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings) {
    WaterOverlay overlay;
    std::vector<WaterOverlayPoint> currentPath;
    float currentFlowId = -1.0F;
    for (const auto& point : pathAnchors.points) {
        if (point.particleRole >= 0.5F) {
            continue;
        }
        if (!currentPath.empty() && std::abs(point.flowId - currentFlowId) > 1.0e-4F) {
            IncludeWaterPathWithParticles(&overlay, std::move(currentPath), trailShapeSettings, animationTrailSettings);
            currentPath.clear();
        }
        currentFlowId = point.flowId;
        currentPath.push_back(point);
    }
    if (!currentPath.empty()) {
        IncludeWaterPathWithParticles(&overlay, std::move(currentPath), trailShapeSettings, animationTrailSettings);
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
            animationTrailSettings);
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
        animationTrailSettings);
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
    output << "end_header\n";

    for (const auto& point : overlay.points) {
        WriteFloat(output, point.position.x);
        WriteFloat(output, point.position.y);
        WriteFloat(output, point.position.z);
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
    }

    if (!output.good()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed while writing water overlay PLY.";
        }
        return false;
    }

    return true;
}

}  // namespace invisible_places::water
