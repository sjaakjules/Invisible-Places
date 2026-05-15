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

glm::vec3 ToGlm(const invisible_places::io::Float3& point) {
    return {point.x, point.y, point.z};
}

invisible_places::io::Float3 FromGlm(const glm::vec3& point) {
    return {point.x, point.y, point.z};
}

float Clamp01(float value) {
    return std::clamp(value, 0.0F, 1.0F);
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

WaterBakeSettings DefaultWaterBakeSettings(WaterScaleMode mode) {
    WaterBakeSettings settings;
    settings.scaleMode = mode;
    switch (mode) {
        case WaterScaleMode::Aerial:
            settings.supportVoxelSize = 1.25F;
            settings.maxBridgeDistance = 4.0F;
            settings.smoothing = 0.65F;
            settings.pathLength = 180.0F;
            settings.pathDensity = 1.2F;
            settings.maxSteps = 260;
            settings.supportSampleLimit = 160000;
            break;
        case WaterScaleMode::Detail:
            settings.supportVoxelSize = 0.012F;
            settings.maxBridgeDistance = 0.055F;
            settings.smoothing = 0.28F;
            settings.pathLength = 3.5F;
            settings.pathDensity = 0.012F;
            settings.maxSteps = 220;
            settings.supportSampleLimit = 220000;
            break;
        case WaterScaleMode::Mid:
            break;
    }
    return settings;
}

std::vector<WaterEmitter> SuggestWaterEmitters(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& existingEmitters,
    const WaterBakeSettings& settings,
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
        if (settings.scaleMode != WaterScaleMode::Aerial && verticalFace < 0.28F) {
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
        emitter.speed = settings.scaleMode == WaterScaleMode::Aerial ? 0.45F : 1.0F;
        emitter.scope = settings.scaleMode;
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
    const WaterBakeSettings& settings) {
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

WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterBakeSettings& settings,
    bool previewOnly) {
    WaterOverlay overlay;
    if (cloud.positions.empty() || emitters.empty()) {
        return overlay;
    }

    auto graph = BuildSupportGraph(cloud, settings);
    if (graph.points.empty()) {
        return overlay;
    }

    const std::uint32_t maxSteps = previewOnly ? std::min<std::uint32_t>(settings.maxSteps, 52U) : settings.maxSteps;
    const float pathLength = previewOnly ? std::min(settings.pathLength, settings.maxBridgeDistance * 32.0F)
                                         : settings.pathLength;
    const float pathDensity = std::max(0.005F, settings.pathDensity);
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

        for (std::uint32_t step = 0; step < maxSteps && distanceAlongPath < pathLength; ++step) {
            if (currentIndex >= graph.points.size()) {
                break;
            }
            visited.push_back(currentIndex);
            const auto& current = graph.points[currentIndex];
            const auto nextIndex = ChooseDownhillNeighbour(graph, currentIndex, settings, visited);
            if (!nextIndex.has_value() || nextIndex.value() >= graph.points.size()) {
                const glm::vec3 lift = current.hasNormal ? current.normal * surfaceLift : glm::vec3{0.0F, 0.0F, surfaceLift};
                IncludeOverlayPoint(
                    &overlay,
                    MakeOverlayPoint(
                        current.position + lift,
                        emitter,
                        flowId,
                        distanceAlongPath,
                        pathLength,
                        confidence * current.confidence,
                        0.85F,
                        std::max(pathDensity, emitter.radius * 0.55F)));
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
                static_cast<std::uint32_t>(std::ceil(segmentLength / pathDensity)));
            for (std::uint32_t sample = 0; sample <= segmentSamples; ++sample) {
                const float t = static_cast<float>(sample) / static_cast<float>(segmentSamples);
                const glm::vec3 position = current.position + segment * t;
                const glm::vec3 normal = current.hasNormal ? current.normal : (next.hasNormal ? next.normal : glm::vec3{0.0F, 0.0F, 1.0F});
                const glm::vec3 lift = normal * surfaceLift;
                const float sampleDistance = distanceAlongPath + segmentLength * t;
                const float width =
                    std::max(pathDensity, emitter.radius * (0.35F + emitter.strength * 0.28F)) *
                    (0.7F + Clamp01(sampleDistance / std::max(0.001F, pathLength)) * 0.55F + pooling * 0.35F);
                IncludeOverlayPoint(
                    &overlay,
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
