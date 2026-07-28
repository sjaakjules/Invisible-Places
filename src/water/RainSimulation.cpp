#include "water/RainSimulation.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <unordered_map>

namespace invisible_places::water {

namespace {

constexpr std::uint32_t kWaterSurfaceWriterSchemaVersion =
    kWaterSurfaceCacheSchemaVersion;
constexpr std::string_view kWaterSurfaceCacheMagic = "IPWSC004";
constexpr std::string_view kPreviousWaterSurfaceCacheMagic = "IPWSC003";
constexpr std::string_view kLegacyRainCacheMagic = "IPWSC002";
constexpr std::string_view kWaterSurfaceIdentityTrailerMagic = "WSCID004";
constexpr std::string_view kPreviousWaterSurfaceIdentityTrailerMagic = "WSCID003";
constexpr std::string_view kLegacyWaterSurfaceIdentityTrailerMagic = "WSCID002";
constexpr std::string_view kPreviousWaterSurfaceCacheAlgorithmId =
    "water-surface-10mm-normal-average-v1";
constexpr std::uint64_t kWaterSurfacePayloadIoChunkBytes = 64ULL * 1024ULL * 1024ULL;
constexpr std::uint32_t kMaximumHashProbeCount = 32U;
constexpr float kPi = 3.14159265358979323846F;

struct Cell2 {
    std::int32_t x = 0;
    std::int32_t y = 0;

    bool operator==(const Cell2&) const = default;
};

struct Cell3 {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;

    bool operator==(const Cell3&) const = default;
};

std::uint32_t HashBits(std::uint32_t value) {
    value ^= value >> 16U;
    value *= 0x7FEB352DU;
    value ^= value >> 15U;
    value *= 0x846CA68BU;
    return value ^ (value >> 16U);
}

std::uint32_t HashCell(std::int32_t x, std::int32_t y, std::int32_t z = 0) {
    auto hash = HashBits(std::bit_cast<std::uint32_t>(x));
    hash ^= std::rotl(HashBits(std::bit_cast<std::uint32_t>(y)), 11);
    hash ^= std::rotl(HashBits(std::bit_cast<std::uint32_t>(z)), 22);
    return HashBits(hash);
}

struct Cell2Hash {
    std::size_t operator()(const Cell2& cell) const {
        return static_cast<std::size_t>(HashCell(cell.x, cell.y));
    }
};

struct Cell3Hash {
    std::size_t operator()(const Cell3& cell) const {
        return static_cast<std::size_t>(HashCell(cell.x, cell.y, cell.z));
    }
};

struct FlowCellKey {
    Cell3 cell{};
    WaterSurfaceRole role = WaterSurfaceRole::None;

    bool operator==(const FlowCellKey&) const = default;
};

struct FlowCellKeyHash {
    std::size_t operator()(const FlowCellKey& key) const {
        return static_cast<std::size_t>(HashBits(
            HashCell(key.cell.x, key.cell.y, key.cell.z) ^
            HashBits(static_cast<std::uint32_t>(key.role))));
    }
};

io::Float3 Add(const io::Float3& left, const io::Float3& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

io::Float3 Subtract(const io::Float3& left, const io::Float3& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

io::Float3 Scale(const io::Float3& value, float scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

float Dot(const io::Float3& left, const io::Float3& right) {
    return (left.x * right.x) + (left.y * right.y) + (left.z * right.z);
}

io::Float3 Cross(const io::Float3& left, const io::Float3& right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

float Length(const io::Float3& value) {
    return std::sqrt(std::max(0.0F, Dot(value, value)));
}

io::Float3 Normalize(const io::Float3& value, const io::Float3& fallback = {0.0F, 0.0F, 1.0F}) {
    const float length = Length(value);
    if (!std::isfinite(length) || length <= 1.0e-7F) {
        return fallback;
    }
    return Scale(value, 1.0F / length);
}

io::Float3 CanonicalHemisphereNormal(const io::Float3& value) {
    const auto normal = Normalize(value);
    constexpr float orientationEpsilon = 1.0e-6F;
    // Point-cloud normals describe an unoriented surface plane. Prefer the
    // upward hemisphere for terrain; exactly vertical planes use Y then X as
    // deterministic tie-breakers. This keeps consistently upward slopes
    // continuous while making arbitrary per-point flips order-independent.
    const float orientation = std::abs(normal.z) > orientationEpsilon
                                  ? normal.z
                                  : (std::abs(normal.y) > orientationEpsilon
                                         ? normal.y
                                         : normal.x);
    return orientation < 0.0F ? Scale(normal, -1.0F) : normal;
}

io::Float3 DownhillFromNormal(const io::Float3& value) {
    const auto normal = CanonicalHemisphereNormal(value);
    constexpr io::Float3 gravity{0.0F, 0.0F, -1.0F};
    const auto tangentGravity =
        Subtract(gravity, Scale(normal, Dot(gravity, normal)));
    const float length = Length(tangentGravity);
    if (!std::isfinite(length) || length <= 1.0e-7F) {
        return {};
    }
    return Scale(tangentGravity, 1.0F / length);
}

io::Float3 Lerp(const io::Float3& left, const io::Float3& right, float amount) {
    return Add(left, Scale(Subtract(right, left), amount));
}

std::int32_t CellCoordinate(float value, float resolution) {
    return static_cast<std::int32_t>(std::floor(value / resolution));
}

Cell2 PositionCell2(const io::Float3& position, float resolution) {
    return {CellCoordinate(position.x, resolution), CellCoordinate(position.y, resolution)};
}

Cell3 PositionCell3(const io::Float3& position, float resolution) {
    return {
        CellCoordinate(position.x, resolution),
        CellCoordinate(position.y, resolution),
        CellCoordinate(position.z, resolution),
    };
}

io::Float3 TransformPosition(const io::Float3& point, const WaterSurfaceSource& source) {
    if (!source.hasTransform) {
        return point;
    }
    const auto& matrix = source.localToWorld;
    const double x = matrix.At(0, 0) * point.x + matrix.At(0, 1) * point.y +
                     matrix.At(0, 2) * point.z + matrix.At(0, 3);
    const double y = matrix.At(1, 0) * point.x + matrix.At(1, 1) * point.y +
                     matrix.At(1, 2) * point.z + matrix.At(1, 3);
    const double z = matrix.At(2, 0) * point.x + matrix.At(2, 1) * point.y +
                     matrix.At(2, 2) * point.z + matrix.At(2, 3);
    const double w = matrix.At(3, 0) * point.x + matrix.At(3, 1) * point.y +
                     matrix.At(3, 2) * point.z + matrix.At(3, 3);
    const double inverseW = std::abs(w) > 1.0e-12 ? 1.0 / w : 1.0;
    return {
        static_cast<float>(x * inverseW),
        static_cast<float>(y * inverseW),
        static_cast<float>(z * inverseW),
    };
}

io::Float3 TransformNormal(const io::Float3& normal, const WaterSurfaceSource& source) {
    if (!source.hasTransform) {
        return Normalize(normal);
    }
    const auto& matrix = source.localToWorld;
    return Normalize({
        static_cast<float>(matrix.At(0, 0) * normal.x + matrix.At(0, 1) * normal.y +
                           matrix.At(0, 2) * normal.z),
        static_cast<float>(matrix.At(1, 0) * normal.x + matrix.At(1, 1) * normal.y +
                           matrix.At(1, 2) * normal.z),
        static_cast<float>(matrix.At(2, 0) * normal.x + matrix.At(2, 1) * normal.y +
                           matrix.At(2, 2) * normal.z),
    });
}

struct SurfaceAccumulatorRole {
    float height = -std::numeric_limits<float>::infinity();
    io::Float3 normalSum{};
    std::uint32_t count = 0U;
};

struct SurfaceAccumulator {
    SurfaceAccumulatorRole rock;
    SurfaceAccumulatorRole sand;
};

struct VegetationAccumulator {
    io::Float3 normalSum{};
    std::uint32_t count = 0U;
};

struct FlowSurfaceAccumulator {
    io::Float3 positionSum{};
    io::Float3 normalSum{};
    float maximumHeight = -std::numeric_limits<float>::infinity();
    double roughnessSum = 0.0;
    std::uint32_t count = 0U;
    std::uint32_t roughnessCount = 0U;
};

struct GroundSurfaceAccumulator {
    io::Float3 normalSum{};
    io::Float3 downhillSum{};
    float maximumHeight = -std::numeric_limits<float>::infinity();
    std::uint32_t count = 0U;
    std::int32_t cellZ = std::numeric_limits<std::int32_t>::min();
};

class CollisionAccumulator {
public:
    explicit CollisionAccumulator(float resolution) : resolution_(std::max(0.001F, resolution)) {}

    void AddSample(const WaterSurfaceSample& sample) {
        if (sample.role == WaterSurfaceRole::None) {
            return;
        }
        bounds_.Expand(sample.position);
        const auto normal = CanonicalHemisphereNormal(sample.normal);
        if (sample.role == WaterSurfaceRole::Ground) {
            ++groundPointCount_;
            const auto sampleCell =
                PositionCell3(sample.position, resolution_);
            auto& target = groundSurfaces_[{sampleCell.x, sampleCell.y}];
            if (sampleCell.z > target.cellZ) {
                target = {};
                target.cellZ = sampleCell.z;
            } else if (sampleCell.z < target.cellZ) {
                return;
            }
            target.normalSum = Add(target.normalSum, normal);
            target.downhillSum = Add(target.downhillSum, DownhillFromNormal(normal));
            target.maximumHeight = std::max(target.maximumHeight, sample.position.z);
            ++target.count;
            return;
        }

        ++pointCount_;
        if (sample.role == WaterSurfaceRole::Vegetation) {
            auto& target = vegetation_[PositionCell3(sample.position, resolution_)];
            target.normalSum = Add(target.normalSum, normal);
            ++target.count;
            return;
        }

        auto& flowTarget = flowSurfaces_[{
            .cell = PositionCell3(sample.position, resolution_),
            .role = sample.role,
        }];
        flowTarget.positionSum = Add(flowTarget.positionSum, sample.position);
        flowTarget.normalSum = Add(flowTarget.normalSum, normal);
        flowTarget.maximumHeight = std::max(flowTarget.maximumHeight, sample.position.z);
        if (sample.hasRoughness && std::isfinite(sample.roughness)) {
            flowTarget.roughnessSum += std::max(0.0F, sample.roughness);
            ++flowTarget.roughnessCount;
        }
        ++flowTarget.count;
    }

    WaterSurfaceCache Finish() {
        WaterSurfaceCache cache;
        cache.schemaVersion = kWaterSurfaceWriterSchemaVersion;
        cache.resolutionMeters = resolution_;
        cache.bounds = bounds_;
        cache.sourcePointCount = pointCount_;
        cache.groundSourcePointCount = groundPointCount_;
        // Rain consumes one top sheet per XY cell and terrain role. Derive it
        // from the highest occupied 3D surfel so its normal is the same
        // deterministic 10 mm average used by Flow/Seepage, independent of PLY
        // point order. This also avoids a second hash lookup for every source
        // point during the one-time cold build.
        {
            std::unordered_map<Cell2, SurfaceAccumulator, Cell2Hash> surfaces;
            surfaces.reserve(flowSurfaces_.size());
            for (const auto& [key, accumulator] : flowSurfaces_) {
                auto& cell = surfaces[{key.cell.x, key.cell.y}];
                auto& target = key.role == WaterSurfaceRole::Sand ? cell.sand : cell.rock;
                if (accumulator.maximumHeight > target.height) {
                    target.height = accumulator.maximumHeight;
                    target.normalSum = accumulator.normalSum;
                    target.count = accumulator.count;
                }
            }

            cache.surfaceCells.reserve(surfaces.size());
            for (const auto& [key, accumulator] : surfaces) {
                RainSurfaceCell cell;
                cell.cellX = key.x;
                cell.cellY = key.y;
                cell.rockHeight = accumulator.rock.height;
                cell.sandHeight = accumulator.sand.height;
                cell.rockNormal = Normalize(accumulator.rock.normalSum);
                cell.sandNormal = Normalize(accumulator.sand.normalSum);
                cell.rockSampleCount = accumulator.rock.count;
                cell.sandSampleCount = accumulator.sand.count;
                cell.rockConfidence = std::clamp(
                    static_cast<float>(accumulator.rock.count) / 8.0F,
                    0.0F,
                    1.0F);
                cell.sandConfidence = std::clamp(
                    static_cast<float>(accumulator.sand.count) / 8.0F,
                    0.0F,
                    1.0F);
                cache.surfaceCells.push_back(cell);
            }
        }
        std::sort(cache.surfaceCells.begin(), cache.surfaceCells.end(), [](const auto& left, const auto& right) {
            return std::tie(left.cellX, left.cellY) < std::tie(right.cellX, right.cellY);
        });

        cache.vegetationVoxels.reserve(vegetation_.size());
        for (const auto& [key, accumulator] : vegetation_) {
            cache.vegetationVoxels.push_back({
                .cellX = key.x,
                .cellY = key.y,
                .cellZ = key.z,
                .normal = Normalize(accumulator.normalSum),
                .sampleCount = accumulator.count,
            });
        }
        std::sort(
            cache.vegetationVoxels.begin(),
            cache.vegetationVoxels.end(),
            [](const auto& left, const auto& right) {
                return std::tie(left.cellX, left.cellY, left.cellZ) <
                       std::tie(right.cellX, right.cellY, right.cellZ);
            });

        cache.flowSurfaceSurfels.reserve(flowSurfaces_.size());
        for (const auto& [key, accumulator] : flowSurfaces_) {
            if (accumulator.count == 0U) {
                continue;
            }
            const float inverseCount = 1.0F / static_cast<float>(accumulator.count);
            const float coherence = std::clamp(Length(accumulator.normalSum) * inverseCount, 0.0F, 1.0F);
            const float normalVariance = 1.0F - coherence;
            const float roughness = accumulator.roughnessCount != 0U
                ? static_cast<float>(accumulator.roughnessSum /
                                     static_cast<double>(accumulator.roughnessCount))
                : normalVariance;
            cache.flowSurfaceSurfels.push_back({
                .cellX = key.cell.x,
                .cellY = key.cell.y,
                .cellZ = key.cell.z,
                .role = key.role,
                .centroid = Scale(accumulator.positionSum, inverseCount),
                .normal = Normalize(accumulator.normalSum),
                .confidence = std::clamp(static_cast<float>(accumulator.count) / 8.0F, 0.0F, 1.0F),
                .normalCoherence = coherence,
                .roughness = roughness,
                .normalVariance = normalVariance,
                .sampleCount = accumulator.count,
            });
        }
        std::sort(
            cache.flowSurfaceSurfels.begin(),
            cache.flowSurfaceSurfels.end(),
            [](const auto& left, const auto& right) {
                return std::tie(left.cellX, left.cellY, left.cellZ, left.role) <
                       std::tie(right.cellX, right.cellY, right.cellZ, right.role);
            });

        // Ground begins as the highest occupied 10 mm Z cell for each XY
        // coordinate. It is deliberately classified only after authored
        // ROCK/SAND and VEG aggregates are complete.
        std::unordered_map<Cell2, const RainSurfaceCell*, Cell2Hash> terrainByXy;
        terrainByXy.reserve(cache.surfaceCells.size());
        for (const auto& terrain : cache.surfaceCells) {
            terrainByXy[{terrain.cellX, terrain.cellY}] = &terrain;
        }

        std::vector<WaterGroundCell> groundCandidates;
        groundCandidates.reserve(groundSurfaces_.size());
        constexpr float kTerrainContactToleranceMeters = 0.020F;
        for (const auto& [cell, accumulator] : groundSurfaces_) {
            if (accumulator.count == 0U ||
                !std::isfinite(accumulator.maximumHeight)) {
                continue;
            }
            std::uint32_t flags = 0U;
            if (const auto terrainIt = terrainByXy.find(cell);
                terrainIt != terrainByXy.end()) {
                const auto* terrain = terrainIt->second;
                const float terrainHeight = std::max(
                    terrain->rockHeight,
                    terrain->sandHeight);
                if (std::isfinite(terrainHeight)) {
                    if (accumulator.maximumHeight <
                        terrainHeight - kTerrainContactToleranceMeters) {
                        continue;
                    }
                    if (accumulator.maximumHeight <=
                        terrainHeight + kTerrainContactToleranceMeters) {
                        flags |= kWaterGroundTerminalContactFlag;
                    } else {
                        flags |= kWaterGroundUpperFlag;
                    }
                }
            }

            // Near-surface VEG retains the legacy component-seed semantics:
            // this is the only vegetation test allowed to change Ground
            // membership. Elevated canopy association happens after component
            // retention so it can feed Rain/Mesh Flow without admitting
            // otherwise disconnected sampled surfaces.
            const auto groundCellZ =
                CellCoordinate(accumulator.maximumHeight, resolution_);
            for (std::int32_t zOffset = -2; zOffset <= 2; ++zOffset) {
                if (vegetation_.contains({
                        cell.x,
                        cell.y,
                        groundCellZ + zOffset,
                    })) {
                    flags |= kWaterGroundVegetationSupportedFlag;
                    break;
                }
            }

            groundCandidates.push_back({
                .cellX = cell.x,
                .cellY = cell.y,
                .height = accumulator.maximumHeight,
                .normal = Normalize(accumulator.normalSum),
                .downhill = Normalize(accumulator.downhillSum, {}),
                .confidence = std::clamp(
                    static_cast<float>(accumulator.count) / 8.0F,
                    0.0F,
                    1.0F),
                .sampleCount = accumulator.count,
                .flags = flags,
            });
        }
        std::sort(
            groundCandidates.begin(),
            groundCandidates.end(),
            [](const auto& left, const auto& right) {
                return std::tie(left.cellX, left.cellY) <
                       std::tie(right.cellX, right.cellY);
            });

        static constexpr std::array<Cell2, 8> kGroundNeighbours{{
            {0, 1},
            {1, 1},
            {1, 0},
            {1, -1},
            {0, -1},
            {-1, -1},
            {-1, 0},
            {-1, 1},
        }};
        std::unordered_map<Cell2, std::size_t, Cell2Hash> candidateByCell;
        candidateByCell.reserve(groundCandidates.size());
        for (std::size_t index = 0U; index < groundCandidates.size(); ++index) {
            candidateByCell[{
                groundCandidates[index].cellX,
                groundCandidates[index].cellY,
            }] = index;
        }

        std::vector<bool> visited(groundCandidates.size(), false);
        std::vector<bool> retained(groundCandidates.size(), false);
        std::uint32_t nextComponentId = 1U;
        for (std::size_t seedIndex = 0U;
             seedIndex < groundCandidates.size();
             ++seedIndex) {
            if (visited[seedIndex]) {
                continue;
            }
            std::vector<std::size_t> component;
            std::vector<std::size_t> pending{seedIndex};
            visited[seedIndex] = true;
            bool containsSeed = false;
            while (!pending.empty()) {
                const auto index = pending.back();
                pending.pop_back();
                component.push_back(index);
                const auto& cell = groundCandidates[index];
                containsSeed = containsSeed ||
                    (cell.flags & (kWaterGroundUpperFlag |
                                   kWaterGroundVegetationSupportedFlag)) != 0U;
                for (const auto& offset : kGroundNeighbours) {
                    const auto neighbourIt = candidateByCell.find({
                        cell.cellX + offset.x,
                        cell.cellY + offset.y,
                    });
                    if (neighbourIt == candidateByCell.end() ||
                        visited[neighbourIt->second]) {
                        continue;
                    }
                    visited[neighbourIt->second] = true;
                    pending.push_back(neighbourIt->second);
                }
            }
            if (!containsSeed) {
                continue;
            }
            const auto componentId = nextComponentId++;
            for (const auto index : component) {
                retained[index] = true;
                groundCandidates[index].componentId = componentId;
            }
        }

        cache.groundCells.reserve(groundCandidates.size());
        for (std::size_t index = 0U; index < groundCandidates.size(); ++index) {
            if (retained[index]) {
                cache.groundCells.push_back(groundCandidates[index]);
            }
        }
        std::unordered_map<Cell2, std::size_t, Cell2Hash> retainedByCell;
        retainedByCell.reserve(cache.groundCells.size());
        for (std::size_t index = 0U; index < cache.groundCells.size(); ++index) {
            retainedByCell[{
                cache.groundCells[index].cellX,
                cache.groundCells[index].cellY,
            }] = index;
        }

        // Elevated VEG is a post-retention flag association. It therefore
        // enriches the already approved Ground topology without allowing a
        // canopy point to retain a new disconnected component. Exact-column
        // ownership and a finite vertical reach keep the join deterministic
        // and prevent lateral or vertically unrelated acquisitions linking.
        for (const auto& [vegetationCell, vegetation] : vegetation_) {
            const auto retainedIt = retainedByCell.find({
                vegetationCell.x,
                vegetationCell.y,
            });
            if (retainedIt == retainedByCell.end() ||
                vegetation.count == 0U) {
                continue;
            }
            auto& ground = cache.groundCells[retainedIt->second];
            const float vegetationMinimumHeight =
                static_cast<float>(vegetationCell.z) * resolution_;
            const float vegetationMaximumHeight =
                vegetationMinimumHeight + resolution_;
            if (vegetationMaximumHeight <
                ground.height - kTerrainContactToleranceMeters) {
                continue;
            }
            const float verticalDistance = std::max(
                0.0F,
                vegetationMinimumHeight - ground.height);
            if (verticalDistance <=
                kWaterGroundVegetationAssociationMaximumHeightMeters) {
                ground.flags |= kWaterGroundVegetationSupportedFlag;
            }
        }

        for (auto& cell : cache.groundCells) {
            for (std::size_t neighbourIndex = 0U;
                 neighbourIndex < kGroundNeighbours.size();
                 ++neighbourIndex) {
                const auto& offset = kGroundNeighbours[neighbourIndex];
                const auto neighbourIt = retainedByCell.find({
                    cell.cellX + offset.x,
                    cell.cellY + offset.y,
                });
                if (neighbourIt != retainedByCell.end() &&
                    cache.groundCells[neighbourIt->second].componentId ==
                        cell.componentId) {
                    cell.connectivityMask |= 1U << neighbourIndex;
                }
            }
        }

        // A 3x3 Sobel derivative of the normalized downhill field yields a
        // bounded, dimensionless negative divergence. Missing neighbours copy
        // the centre vector so sparse component boundaries do not create false
        // convergence attractors.
        for (auto& cell : cache.groundCells) {
            const auto sampleDownhill = [&](std::int32_t xOffset,
                                            std::int32_t yOffset) {
                const auto sampleIt = retainedByCell.find({
                    cell.cellX + xOffset,
                    cell.cellY + yOffset,
                });
                if (sampleIt == retainedByCell.end() ||
                    cache.groundCells[sampleIt->second].componentId !=
                        cell.componentId) {
                    return cell.downhill;
                }
                return cache.groundCells[sampleIt->second].downhill;
            };
            const auto northWest = sampleDownhill(-1, 1);
            const auto north = sampleDownhill(0, 1);
            const auto northEast = sampleDownhill(1, 1);
            const auto west = sampleDownhill(-1, 0);
            const auto east = sampleDownhill(1, 0);
            const auto southWest = sampleDownhill(-1, -1);
            const auto south = sampleDownhill(0, -1);
            const auto southEast = sampleDownhill(1, -1);
            const float derivativeX =
                (northEast.x + 2.0F * east.x + southEast.x -
                 northWest.x - 2.0F * west.x - southWest.x) /
                8.0F;
            const float derivativeY =
                (northWest.y + 2.0F * north.y + northEast.y -
                 southWest.y - 2.0F * south.y - southEast.y) /
                8.0F;
            cell.convergence = std::clamp(
                -0.5F * (derivativeX + derivativeY),
                0.0F,
                1.0F);
        }
        return cache;
    }

private:
    float resolution_ = kWaterSurfaceResolutionMeters;
    std::unordered_map<Cell3, VegetationAccumulator, Cell3Hash> vegetation_;
    std::unordered_map<FlowCellKey, FlowSurfaceAccumulator, FlowCellKeyHash> flowSurfaces_;
    std::unordered_map<Cell2, GroundSurfaceAccumulator, Cell2Hash> groundSurfaces_;
    io::Bounds3f bounds_;
    std::uint64_t pointCount_ = 0U;
    std::uint64_t groundPointCount_ = 0U;
};

WaterSurfaceRole CollisionRoleForSceneRole(scene::ScenePointCloudRole role) {
    switch (role) {
        case scene::ScenePointCloudRole::Rock:
            return WaterSurfaceRole::Rock;
        case scene::ScenePointCloudRole::Sand:
            return WaterSurfaceRole::Sand;
        case scene::ScenePointCloudRole::Vegetation:
            return WaterSurfaceRole::Vegetation;
    }
    return WaterSurfaceRole::None;
}

template <typename T>
bool WritePod(std::ofstream& output, const T& value) {
    output.write(reinterpret_cast<const char*>(&value), sizeof(T));
    return output.good();
}

template <typename T>
bool ReadPod(std::ifstream& input, T* value) {
    input.read(reinterpret_cast<char*>(value), sizeof(T));
    return input.good();
}

bool WriteString(std::ofstream& output, std::string_view value) {
    const auto size = static_cast<std::uint32_t>(value.size());
    return WritePod(output, size) &&
           (output.write(value.data(), static_cast<std::streamsize>(value.size())), output.good());
}

bool ReadString(std::ifstream& input, std::string* value, std::uint32_t maximumLength = 1U << 20U) {
    std::uint32_t size = 0U;
    if (!ReadPod(input, &size) || size > maximumLength) {
        return false;
    }
    value->resize(size);
    input.read(value->data(), static_cast<std::streamsize>(size));
    return input.good();
}

void HashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
    constexpr std::uint64_t prime = 1099511628211ULL;
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t index = 0; index < size; ++index) {
        *hash ^= bytes[index];
        *hash *= prime;
    }
}

void HashString(std::uint64_t* hash, std::string_view value) {
    HashBytes(hash, value.data(), value.size());
}

class WaterSurfaceIdentityDigestBuilder {
public:
    template <typename T>
    void AddPod(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>);
        AddBytes(&value, sizeof(value));
    }

    void AddString(std::string_view value) {
        AddPod(static_cast<std::uint64_t>(value.size()));
        AddBytes(value.data(), value.size());
    }

    void AddBytes(const void* data, std::size_t size) {
        static constexpr std::array<std::uint64_t, 4> primes{
            0x00000100000001B3ULL,
            0x9E3779B185EBCA87ULL,
            0xC2B2AE3D27D4EB4FULL,
            0x165667B19E3779F9ULL,
        };
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        for (std::size_t index = 0U; index < size; ++index) {
            const std::uint64_t byte = bytes[index];
            for (std::size_t lane = 0U; lane < state_.size(); ++lane) {
                state_[lane] ^= byte + (0x9DU * lane);
                state_[lane] *= primes[lane];
                state_[lane] = std::rotl(state_[lane], static_cast<int>(7U + lane * 6U));
            }
        }
        byteCount_ += size;
    }

    [[nodiscard]] std::array<std::uint64_t, 4> Finish() const {
        auto result = state_;
        for (std::size_t lane = 0U; lane < result.size(); ++lane) {
            auto value = result[lane] ^
                         (static_cast<std::uint64_t>(byteCount_) +
                          0x9E3779B97F4A7C15ULL * (lane + 1U));
            value ^= value >> 30U;
            value *= 0xBF58476D1CE4E5B9ULL;
            value ^= value >> 27U;
            value *= 0x94D049BB133111EBULL;
            result[lane] = value ^ (value >> 31U);
        }
        return result;
    }

private:
    std::array<std::uint64_t, 4> state_{
        0xCBF29CE484222325ULL,
        0x84222325CBF29CE4ULL,
        0x6A09E667F3BCC909ULL,
        0xBB67AE8584CAA73BULL,
    };
    std::size_t byteCount_ = 0U;
};

class WaterSurfacePayloadChecksumBuilder {
public:
    template <typename T>
    void AddPod(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>);
        AddBytes(&value, sizeof(value));
    }

    void AddBytes(const void* data, std::size_t size) {
        if (data == nullptr || size == 0U) {
            return;
        }
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        hashedByteCount_ += static_cast<std::uint64_t>(size);
        if (tailSize_ != 0U) {
            const auto copied = std::min(size, tail_.size() - tailSize_);
            std::memcpy(tail_.data() + tailSize_, bytes, copied);
            tailSize_ += copied;
            bytes += copied;
            size -= copied;
            if (tailSize_ == tail_.size()) {
                std::uint64_t word = 0U;
                std::memcpy(&word, tail_.data(), sizeof(word));
                MixWord(word);
                tailSize_ = 0U;
            }
        }
        while (size >= sizeof(std::uint64_t)) {
            std::uint64_t word = 0U;
            std::memcpy(&word, bytes, sizeof(word));
            MixWord(word);
            bytes += sizeof(word);
            size -= sizeof(word);
        }
        if (size != 0U) {
            std::memcpy(tail_.data(), bytes, size);
            tailSize_ = size;
        }
    }

    [[nodiscard]] WaterSurfaceCachePayloadChecksum Finish() const {
        auto first = first_;
        auto second = second_;
        if (tailSize_ != 0U) {
            std::uint64_t tailWord = 0U;
            std::memcpy(&tailWord, tail_.data(), tailSize_);
            tailWord ^= static_cast<std::uint64_t>(tailSize_) << 56U;
            MixWordInto(&first, &second, tailWord, wordCount_);
        }
        const auto finalise = [this](std::uint64_t value, std::uint64_t salt) {
            value ^= hashedByteCount_ + salt;
            value ^= value >> 30U;
            value *= 0xBF58476D1CE4E5B9ULL;
            value ^= value >> 27U;
            value *= 0x94D049BB133111EBULL;
            return value ^ (value >> 31U);
        };
        return {
            .words = {
                finalise(first, 0x9E3779B97F4A7C15ULL),
                finalise(second, 0xD6E8FEB86659FD93ULL),
            },
            .hashedByteCount = static_cast<std::uint64_t>(hashedByteCount_),
        };
    }

private:
    static void MixWordInto(
        std::uint64_t* first,
        std::uint64_t* second,
        std::uint64_t word,
        std::uint64_t wordIndex) {
        const auto mixed = word + 0x9E3779B97F4A7C15ULL * (wordIndex + 1U);
        *first = std::rotl(*first ^ mixed, 27) * 0x3C79AC492BA7B653ULL;
        *second = std::rotl(*second + (mixed ^ (*first >> 17U)), 31) *
                  0x1C69B3F74AC4AE35ULL;
    }

    void MixWord(std::uint64_t word) {
        MixWordInto(&first_, &second_, word, wordCount_);
        ++wordCount_;
    }

    std::uint64_t first_ = 0x243F6A8885A308D3ULL;
    std::uint64_t second_ = 0x13198A2E03707344ULL;
    std::uint64_t wordCount_ = 0U;
    std::uint64_t hashedByteCount_ = 0U;
    std::array<std::uint8_t, sizeof(std::uint64_t)> tail_{};
    std::size_t tailSize_ = 0U;
};

template <typename T>
void AddWaterSurfacePayloadArray(
    WaterSurfacePayloadChecksumBuilder* checksum,
    std::uint64_t tag,
    const std::vector<T>& values) {
    checksum->AddPod(tag);
    checksum->AddPod(static_cast<std::uint64_t>(values.size()));
    if (!values.empty()) {
        checksum->AddBytes(values.data(), values.size() * sizeof(T));
    }
}

WaterSurfaceCachePayloadChecksum CalculateWaterSurfacePayloadChecksumForSchema(
    const WaterSurfaceCache& cache,
    const WaterSurfaceGpuData& gpuData,
    std::uint32_t schemaVersion) {
    WaterSurfacePayloadChecksumBuilder checksum;
    AddWaterSurfacePayloadArray(&checksum, 1U, cache.surfaceCells);
    AddWaterSurfacePayloadArray(&checksum, 2U, cache.vegetationVoxels);
    AddWaterSurfacePayloadArray(&checksum, 3U, cache.flowSurfaceSurfels);
    const bool hasGroundSchema =
        schemaVersion >= kWaterSurfaceCacheSchemaVersion;
    if (hasGroundSchema) {
        AddWaterSurfacePayloadArray(&checksum, 7U, cache.groundCells);
    }
    AddWaterSurfacePayloadArray(&checksum, 4U, gpuData.surfaceTable);
    AddWaterSurfacePayloadArray(&checksum, 5U, gpuData.vegetationTable);
    AddWaterSurfacePayloadArray(&checksum, 6U, gpuData.flowSurfaceTable);
    if (hasGroundSchema) {
        AddWaterSurfacePayloadArray(&checksum, 8U, gpuData.groundTable);
    }
    return checksum.Finish();
}

WaterSurfaceCachePayloadChecksum CalculateWaterSurfacePayloadChecksum(
    const WaterSurfaceCache& cache,
    const WaterSurfaceGpuData& gpuData) {
    return CalculateWaterSurfacePayloadChecksumForSchema(
        cache,
        gpuData,
        kWaterSurfaceCacheSchemaVersion);
}

WaterSurfaceCacheIdentity MakeWaterSurfaceCacheIdentity(
    const WaterSurfaceCache& cache,
    const WaterSurfaceGpuData& gpuData,
    std::uint32_t schemaVersion = kWaterSurfaceCacheSchemaVersion,
    const WaterSurfaceCachePayloadChecksum* checksumOverride = nullptr) {
    WaterSurfaceIdentityDigestBuilder digest;
    const bool hasGroundSchema =
        schemaVersion >= kWaterSurfaceCacheSchemaVersion;
    digest.AddString(
        hasGroundSchema
            ? "InvisiblePlaces.WaterSurfaceCache.Identity.v4"
            : "InvisiblePlaces.WaterSurfaceCache.Identity.v3");
    digest.AddPod(schemaVersion);
    digest.AddPod(cache.resolutionMeters);
    digest.AddString(cache.signature);
    digest.AddPod(cache.bounds.minimum.x);
    digest.AddPod(cache.bounds.minimum.y);
    digest.AddPod(cache.bounds.minimum.z);
    digest.AddPod(cache.bounds.maximum.x);
    digest.AddPod(cache.bounds.maximum.y);
    digest.AddPod(cache.bounds.maximum.z);
    digest.AddPod(static_cast<std::uint8_t>(cache.bounds.valid ? 1U : 0U));
    digest.AddPod(cache.sourcePointCount);
    if (hasGroundSchema) {
        digest.AddPod(cache.groundSourcePointCount);
    }
    digest.AddPod(cache.revision);

    digest.AddPod(static_cast<std::uint64_t>(cache.sources.size()));
    for (const auto& source : cache.sources) {
        digest.AddString(
            schemaVersion <= kWaterSurfaceCacheLegacySchemaVersion
                ? source.sourcePath.generic_string()
                : source.sourcePath.filename().generic_string());
        digest.AddPod(static_cast<std::uint32_t>(source.role));
        digest.AddPod(source.spacingMicrometres);
        digest.AddPod(source.fileSize);
        digest.AddPod(source.modificationTicks);
        digest.AddPod(static_cast<std::uint8_t>(source.isFallback ? 1U : 0U));
    }

    digest.AddPod(static_cast<std::uint64_t>(cache.surfaceCells.size()));
    digest.AddPod(static_cast<std::uint64_t>(cache.vegetationVoxels.size()));
    digest.AddPod(static_cast<std::uint64_t>(cache.flowSurfaceSurfels.size()));
    if (hasGroundSchema) {
        digest.AddPod(static_cast<std::uint64_t>(cache.groundCells.size()));
    }
    digest.AddPod(gpuData.surfaceMask);
    digest.AddPod(gpuData.vegetationMask);
    digest.AddPod(gpuData.flowSurfaceMask);
    if (hasGroundSchema) {
        digest.AddPod(gpuData.groundMask);
    }
    digest.AddPod(gpuData.maximumProbeCount);
    digest.AddPod(gpuData.flowMaximumProbeCount);
    if (hasGroundSchema) {
        digest.AddPod(gpuData.groundMaximumProbeCount);
    }
    digest.AddPod(gpuData.sourceRevision);
    const auto& payloadChecksum = checksumOverride != nullptr
        ? *checksumOverride
        : gpuData.payloadChecksum;
    digest.AddPod(payloadChecksum.words[0]);
    digest.AddPod(payloadChecksum.words[1]);
    digest.AddPod(payloadChecksum.hashedByteCount);

    return {
        .sourceSignature = cache.signature,
        .contentDigest = digest.Finish(),
    };
}

WaterSurfaceCacheIdentity MakeLegacyWaterSurfaceCacheIdentity(
    const WaterSurfaceCache& cache,
    const WaterSurfaceGpuData& gpuData) {
    WaterSurfaceIdentityDigestBuilder digest;
    digest.AddString("InvisiblePlaces.WaterSurfaceCache.Identity.v2");
    digest.AddPod(cache.schemaVersion);
    digest.AddPod(cache.resolutionMeters);
    digest.AddString(cache.signature);
    digest.AddPod(cache.bounds.minimum.x);
    digest.AddPod(cache.bounds.minimum.y);
    digest.AddPod(cache.bounds.minimum.z);
    digest.AddPod(cache.bounds.maximum.x);
    digest.AddPod(cache.bounds.maximum.y);
    digest.AddPod(cache.bounds.maximum.z);
    digest.AddPod(static_cast<std::uint8_t>(cache.bounds.valid ? 1U : 0U));
    digest.AddPod(cache.sourcePointCount);
    digest.AddPod(cache.revision);

    digest.AddPod(static_cast<std::uint64_t>(cache.sources.size()));
    for (const auto& source : cache.sources) {
        digest.AddString(source.sourcePath.generic_string());
        digest.AddPod(static_cast<std::uint32_t>(source.role));
        digest.AddPod(source.spacingMicrometres);
        digest.AddPod(source.fileSize);
        digest.AddPod(source.modificationTicks);
        digest.AddPod(static_cast<std::uint8_t>(source.isFallback ? 1U : 0U));
    }

    digest.AddPod(static_cast<std::uint64_t>(cache.surfaceCells.size()));
    digest.AddPod(static_cast<std::uint64_t>(cache.vegetationVoxels.size()));
    digest.AddPod(static_cast<std::uint64_t>(cache.flowSurfaceSurfels.size()));
    digest.AddPod(gpuData.surfaceMask);
    digest.AddPod(gpuData.vegetationMask);
    digest.AddPod(gpuData.flowSurfaceMask);
    digest.AddPod(gpuData.maximumProbeCount);
    digest.AddPod(gpuData.flowMaximumProbeCount);
    digest.AddPod(gpuData.sourceRevision);

    const auto addTable = [&digest](const auto& table) {
        digest.AddPod(static_cast<std::uint64_t>(table.size()));
        if (!table.empty()) {
            digest.AddBytes(table.data(), table.size() * sizeof(table.front()));
        }
    };
    addTable(gpuData.surfaceTable);
    addTable(gpuData.vegetationTable);
    addTable(gpuData.flowSurfaceTable);
    return {
        .sourceSignature = cache.signature,
        .contentDigest = digest.Finish(),
    };
}

std::optional<RainSurfaceCell> FindSurfaceCell(
    const WaterSurfaceCache& cache,
    std::int32_t cellX,
    std::int32_t cellY) {
    const auto found = std::lower_bound(
        cache.surfaceCells.begin(),
        cache.surfaceCells.end(),
        std::pair{cellX, cellY},
        [](const RainSurfaceCell& cell, const std::pair<std::int32_t, std::int32_t>& key) {
            return std::tie(cell.cellX, cell.cellY) < std::tie(key.first, key.second);
        });
    if (found == cache.surfaceCells.end() || found->cellX != cellX || found->cellY != cellY) {
        return std::nullopt;
    }
    return *found;
}

std::optional<RainVegetationVoxel> FindVegetationVoxel(
    const WaterSurfaceCache& cache,
    std::int32_t cellX,
    std::int32_t cellY,
    std::int32_t cellZ) {
    const auto found = std::lower_bound(
        cache.vegetationVoxels.begin(),
        cache.vegetationVoxels.end(),
        std::tuple{cellX, cellY, cellZ},
        [](const RainVegetationVoxel& cell, const std::tuple<std::int32_t, std::int32_t, std::int32_t>& key) {
            return std::tie(cell.cellX, cell.cellY, cell.cellZ) < key;
        });
    if (found == cache.vegetationVoxels.end() || found->cellX != cellX || found->cellY != cellY ||
        found->cellZ != cellZ) {
        return std::nullopt;
    }
    return *found;
}

const WaterSurfaceSurfel* FindFlowSurfaceSurfel(
    const WaterSurfaceCache& cache,
    std::int32_t cellX,
    std::int32_t cellY,
    std::int32_t cellZ,
    WaterSurfaceRole role) {
    const auto key = std::tuple{cellX, cellY, cellZ, role};
    const auto found = std::lower_bound(
        cache.flowSurfaceSurfels.begin(),
        cache.flowSurfaceSurfels.end(),
        key,
        [](const WaterSurfaceSurfel& surfel, const auto& candidateKey) {
            return std::tie(surfel.cellX, surfel.cellY, surfel.cellZ, surfel.role) <
                   candidateKey;
        });
    if (found == cache.flowSurfaceSurfels.end() || found->cellX != cellX ||
        found->cellY != cellY || found->cellZ != cellZ || found->role != role) {
        return nullptr;
    }
    return &(*found);
}

std::uint32_t PackNormal(const io::Float3& input) {
    const auto normal = Normalize(input);
    const auto pack = [](float value) {
        const auto normalized = std::clamp(value * 0.5F + 0.5F, 0.0F, 1.0F);
        return static_cast<std::uint32_t>(std::lround(normalized * 1023.0F));
    };
    return pack(normal.x) | (pack(normal.y) << 10U) | (pack(normal.z) << 20U);
}

std::uint32_t PackUnit3(const io::Float3& input) {
    const auto pack = [](float value) {
        return static_cast<std::uint32_t>(
            std::lround(std::clamp(value, 0.0F, 1.0F) * 1023.0F));
    };
    return pack(input.x) | (pack(input.y) << 10U) | (pack(input.z) << 20U);
}

std::uint32_t PackUnorm2x16(float first, float second) {
    const auto pack = [](float value) {
        return static_cast<std::uint32_t>(
            std::lround(std::clamp(value, 0.0F, 1.0F) * 65535.0F));
    };
    return pack(first) | (pack(second) << 16U);
}

std::uint16_t FloatToHalf(float value) {
    const auto bits = std::bit_cast<std::uint32_t>(value);
    const auto sign = static_cast<std::uint16_t>((bits >> 16U) & 0x8000U);
    const auto exponent = static_cast<std::int32_t>((bits >> 23U) & 0xFFU) - 127 + 15;
    const auto mantissa = bits & 0x007FFFFFU;
    if (exponent <= 0) {
        if (exponent < -10) {
            return sign;
        }
        const auto normalizedMantissa = mantissa | 0x00800000U;
        const auto shift = static_cast<std::uint32_t>(14 - exponent);
        return static_cast<std::uint16_t>(
            sign | ((normalizedMantissa + (1U << (shift - 1U))) >> shift));
    }
    if (exponent >= 31) {
        return static_cast<std::uint16_t>(sign | 0x7C00U);
    }
    auto roundedMantissa = mantissa + 0x00001000U;
    auto roundedExponent = exponent;
    if ((roundedMantissa & 0x00800000U) != 0U) {
        roundedMantissa = 0U;
        ++roundedExponent;
        if (roundedExponent >= 31) {
            return static_cast<std::uint16_t>(sign | 0x7C00U);
        }
    }
    return static_cast<std::uint16_t>(
        sign | (static_cast<std::uint16_t>(roundedExponent) << 10U) |
        (roundedMantissa >> 13U));
}

std::uint32_t PackHalf2x16(float first, float second) {
    return static_cast<std::uint32_t>(FloatToHalf(first)) |
           (static_cast<std::uint32_t>(FloatToHalf(second)) << 16U);
}

std::uint32_t RequiredHashCapacity(std::size_t count) {
    const auto required = std::max<std::size_t>(2U, static_cast<std::size_t>(std::ceil(count / 0.65)));
    return static_cast<std::uint32_t>(std::bit_ceil(required));
}

std::uint32_t RequiredFlowHashCapacity(std::size_t count) {
    const auto required = std::max<std::size_t>(2U, static_cast<std::size_t>(std::ceil(count / 0.80)));
    return static_cast<std::uint32_t>(std::bit_ceil(required));
}

float Random01(std::uint32_t value) {
    return static_cast<float>(HashBits(value) & 0x00FFFFFFU) / static_cast<float>(0x01000000U);
}

float SmoothStep(float edge0, float edge1, float value) {
    if (edge0 == edge1) {
        return value < edge0 ? 0.0F : 1.0F;
    }
    const float t = std::clamp((value - edge0) / (edge1 - edge0), 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

bool EventActive(const RainImpactEvent& event, float timeSeconds) {
    const float age = timeSeconds - event.birthTimeSeconds;
    return event.role != WaterSurfaceRole::None && age >= 0.0F && age <= event.lifetimeSeconds;
}

io::Float3 RainImpactRandomDirection2D(std::uint32_t seed) {
    const io::Float3 direction{
        Random01(seed + 0x68BC21EBU) * 2.0F - 1.0F,
        Random01(seed + 0x02E5BE93U) * 2.0F - 1.0F,
        0.0F,
    };
    return Normalize(direction, {1.0F, 0.0F, 0.0F});
}

float EvaluateVegetationSprinkle(
    const RainImpactEvent& event,
    const io::Float3& point,
    const io::Float3& pointNormal,
    float age,
    const RainVegetationImpactSettings& settings) {
    if (point.z > event.position.z + 0.01F) {
        return 0.0F;
    }

    constexpr std::uint32_t kStreamCount = 4U;
    constexpr float kSparkleCellMeters = 0.005F;
    const float downwardSpeed = std::clamp(
        settings.propagationMetersPerSecond,
        0.05F,
        6.0F);
    const float hopSpacing = std::clamp(settings.hopSpacingMeters, 0.010F, 0.30F);
    const float streamSpread = std::clamp(settings.streamSpread, 0.0F, 2.0F);
    const float twinkle = std::clamp(settings.twinkle, 0.0F, 4.0F);
    const float authoredWidth = std::clamp(
        settings.streamWidthMeters,
        0.0010F,
        std::max(0.0010F, event.radiusMeters * 0.20F));
    // Account for crown widening and the smooth feather when clamping every
    // hop centre. No visible VEG support can leave the event broad phase.
    const float maximumPathOffset = std::max(
        0.0F,
        event.radiusMeters - authoredWidth * 4.10F);
    const float verticalDistance = std::max(0.0F, event.position.z - point.z);
    const float maximumDepth = std::max(0.18F, event.lifetimeSeconds * downwardSpeed);
    if (verticalDistance > maximumDepth) {
        return 0.0F;
    }

    const float bandCoordinate = verticalDistance / hopSpacing;
    const auto bandIndex = static_cast<std::uint32_t>(std::floor(bandCoordinate));
    const float bandMix = SmoothStep(0.0F, 1.0F, bandCoordinate - std::floor(bandCoordinate));
    const auto pointHash = HashCell(
        CellCoordinate(point.x, kSparkleCellMeters),
        CellCoordinate(point.y, kSparkleCellMeters),
        CellCoordinate(point.z, kSparkleCellMeters));

    // Assign each point to one path. More visible crown branches therefore do
    // not add another per-point path loop.
    const auto streamIndex = HashBits(event.seed ^ pointHash ^ 0x27D4EB2DU) % kStreamCount;
    const auto streamSeed = event.seed ^ HashBits(0x9E3779B9U * (streamIndex + 1U));

    const auto eventNormal = Normalize(event.normal);
    const auto basisReference = std::abs(eventNormal.z) < 0.90F
        ? io::Float3{0.0F, 0.0F, 1.0F}
        : io::Float3{1.0F, 0.0F, 0.0F};
    const auto tangentX = Normalize(Cross(basisReference, eventNormal), {1.0F, 0.0F, 0.0F});
    const auto tangentY = Normalize(Cross(eventNormal, tangentX), {0.0F, 1.0F, 0.0F});
    const auto branchDirection = RainImpactRandomDirection2D(streamSeed);
    const auto anchorOffset = [&](std::uint32_t index) {
        const float anchorDepth = static_cast<float>(index) * hopSpacing;
        const float anchorDepthFraction = std::clamp(anchorDepth / maximumDepth, 0.0F, 1.0F);
        const float branchScale = event.radiusMeters * streamSpread *
                                  (0.06F + 0.76F * anchorDepthFraction) *
                                  (0.65F + 0.35F * Random01(streamSeed + 5U));
        const auto wander = RainImpactRandomDirection2D(
            streamSeed ^ HashBits(index + 0xA511E9B3U));
        const float wanderScale = event.radiusMeters * 0.16F *
                                  SmoothStep(0.0F, hopSpacing * 2.0F, anchorDepth);
        io::Float3 result{
            branchDirection.x * branchScale + wander.x * wanderScale,
            branchDirection.y * branchScale + wander.y * wanderScale,
            0.0F,
        };
        const float offsetLength = std::hypot(result.x, result.y);
        if (offsetLength > maximumPathOffset && offsetLength > 1.0e-7F) {
            result.x *= maximumPathOffset / offsetLength;
            result.y *= maximumPathOffset / offsetLength;
        }
        return result;
    };
    const auto anchorA = anchorOffset(bandIndex);
    const auto anchorB = anchorOffset(bandIndex + 1U);
    auto pathOffset = Lerp(anchorA, anchorB, bandMix);
    const float hopSide = (streamIndex & 1U) == 0U ? 1.0F : -1.0F;
    const float hopArc = std::sin(kPi * std::clamp(bandCoordinate - std::floor(bandCoordinate), 0.0F, 1.0F)) *
                         event.radiusMeters * 0.055F * hopSide;
    pathOffset.x += -branchDirection.y * hopArc;
    pathOffset.y += branchDirection.x * hopArc;
    const float pathOffsetLength = std::hypot(pathOffset.x, pathOffset.y);
    if (pathOffsetLength > maximumPathOffset && pathOffsetLength > 1.0e-7F) {
        pathOffset.x *= maximumPathOffset / pathOffsetLength;
        pathOffset.y *= maximumPathOffset / pathOffsetLength;
    }
    const auto pathCentre = Add(
        Add(
            {event.position.x, event.position.y, event.position.z - verticalDistance},
            Scale(tangentX, pathOffset.x)),
        Scale(tangentY, pathOffset.y));
    const auto pointOffset = Subtract(point, pathCentre);
    const auto tangentPointOffset = Subtract(
        pointOffset,
        Scale(eventNormal, Dot(pointOffset, eventNormal)));
    const float pathDistance = Length(tangentPointOffset);
    const float crownWidth = std::lerp(
        1.75F,
        1.0F,
        SmoothStep(0.0F, hopSpacing * 2.0F, verticalDistance));
    const float streamWidth = authoredWidth * crownWidth *
                              (0.85F + 0.30F * Random01(streamSeed + 7U));
    const float path = 1.0F - SmoothStep(streamWidth, streamWidth * 2.0F, pathDistance);
    if (path <= 0.0F) {
        return 0.0F;
    }

    const auto sparkleSeed = event.seed ^ pointHash ^ HashBits(0x85EBCA6BU * (streamIndex + 1U));
    const float selectedPoint = SmoothStep(0.52F, 0.92F, Random01(sparkleSeed));
    if (selectedPoint <= 0.0F) {
        return 0.0F;
    }
    const float timeJitter = (Random01(sparkleSeed + 0xC2B2AE35U) - 0.5F) * 0.10F;
    const float streamDelay = streamIndex * 0.026F + Random01(streamSeed + 19U) * 0.035F;
    const float localAge = age - verticalDistance / downwardSpeed -
                           streamDelay - timeJitter;
    const float pulse = SmoothStep(0.0F, 0.030F, localAge) *
                        (1.0F - SmoothStep(0.12F, 0.31F, localAge));
    const float delayedGlint = SmoothStep(0.15F, 0.20F, localAge) *
                               (1.0F - SmoothStep(0.28F, 0.47F, localAge)) * 0.62F;
    const auto normalizedPointNormal = Normalize(pointNormal);
    const float leafFacing = 0.72F + 0.28F * std::abs(Dot(normalizedPointNormal, eventNormal));
    return path * selectedPoint * std::max(pulse, delayedGlint) * twinkle * leafFacing;
}

float RainFrontWave(const RainRuntimeSettings& settings, const io::Float3& position, float timeSeconds) {
    const float scale = std::max(0.2F, settings.weatherFrontScaleMeters);
    const float phase =
        (position.x + position.y) / scale - timeSeconds * settings.weatherFrontSpeedMetersPerSecond / scale;
    return 0.5F + 0.5F * std::sin(phase * 2.0F * kPi);
}

float RainFrontSpawnProbability(
    const RainRuntimeSettings& settings,
    const io::Float3& position,
    float timeSeconds) {
    const float wave = RainFrontWave(settings, position, timeSeconds);
    return std::lerp(1.0F, 0.12F + wave * 0.88F, std::clamp(settings.weatherFrontStrength, 0.0F, 1.0F));
}

std::string WaterSurfaceCacheSignatureForSchema(
    std::span<const WaterSurfaceSource> sources,
    float resolutionMeters,
    std::uint32_t schemaVersion) {
    std::uint64_t hash = 1469598103934665603ULL;
    HashBytes(&hash, &schemaVersion, sizeof(schemaVersion));
    if (schemaVersion > kWaterSurfaceCacheLegacySchemaVersion) {
        // Cache schema describes the persisted ABI; the algorithm id describes
        // aggregation semantics. Hash both so an algorithm-only revision never
        // reuses a structurally compatible but semantically stale sidecar.
        HashString(
            &hash,
            schemaVersion == kWaterSurfaceCachePreviousSchemaVersion
                ? kPreviousWaterSurfaceCacheAlgorithmId
                : kWaterSurfaceCacheAlgorithmId);
    }
    HashBytes(&hash, &resolutionMeters, sizeof(resolutionMeters));
    std::filesystem::path commonSourceParent;
    if (schemaVersion > kWaterSurfaceCacheLegacySchemaVersion && !sources.empty()) {
        commonSourceParent = sources.front().sourcePath.lexically_normal().parent_path();
        const auto isWithin = [](const std::filesystem::path& path,
                                 const std::filesystem::path& parent) {
            if (parent.empty()) {
                return false;
            }
            const auto relative = path.lexically_normal().lexically_relative(parent);
            if (relative.empty() || relative.is_absolute()) {
                return false;
            }
            const auto first = relative.begin();
            return first == relative.end() || *first != "..";
        };
        for (const auto& source : sources) {
            while (!commonSourceParent.empty() &&
                   !isWithin(source.sourcePath, commonSourceParent)) {
                const auto parent = commonSourceParent.parent_path();
                if (parent == commonSourceParent) {
                    commonSourceParent.clear();
                    break;
                }
                commonSourceParent = parent;
            }
        }
    }
    for (const auto& source : sources) {
        std::error_code error;
        if (schemaVersion <= kWaterSurfaceCacheLegacySchemaVersion) {
            const auto normalizedPath =
                std::filesystem::weakly_canonical(source.sourcePath, error).generic_string();
            HashString(
                &hash,
                error ? source.sourcePath.lexically_normal().generic_string() : normalizedPath);
        } else {
            // Preserve nested source paths without coupling the signature to an
            // installation's absolute scene location.
            auto relativePath = source.sourcePath.lexically_normal().lexically_relative(
                commonSourceParent);
            if (relativePath.empty() || relativePath.is_absolute() ||
                (!relativePath.empty() && *relativePath.begin() == "..")) {
                relativePath = source.sourcePath.filename();
            }
            HashString(&hash, relativePath.generic_string());
        }
        const auto role = static_cast<std::uint32_t>(source.role);
        HashBytes(&hash, &role, sizeof(role));
        HashBytes(&hash, &source.spacingMicrometres, sizeof(source.spacingMicrometres));
        HashBytes(&hash, &source.hasTransform, sizeof(source.hasTransform));
        HashBytes(
            &hash,
            source.localToWorld.values.data(),
            source.localToWorld.values.size() * sizeof(double));
        const auto size = std::filesystem::file_size(source.sourcePath, error);
        const auto safeSize = error ? 0U : size;
        HashBytes(&hash, &safeSize, sizeof(safeSize));
        error.clear();
        const auto writeTime = std::filesystem::last_write_time(source.sourcePath, error);
        const auto ticks =
            error ? 0LL : static_cast<long long>(writeTime.time_since_epoch().count());
        HashBytes(&hash, &ticks, sizeof(ticks));
    }
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(16) << hash;
    return stream.str();
}

template <typename T>
bool ReadWaterSurfacePayloadArray(
    std::ifstream& input,
    std::vector<T>* values,
    std::uint64_t tag,
    WaterSurfacePayloadChecksumBuilder* checksum) {
    if (values == nullptr || checksum == nullptr) {
        return false;
    }
    checksum->AddPod(tag);
    checksum->AddPod(static_cast<std::uint64_t>(values->size()));
    const std::size_t maximumChunkElements = std::max<std::size_t>(
        1U,
        static_cast<std::size_t>(kWaterSurfacePayloadIoChunkBytes / sizeof(T)));
    std::size_t offset = 0U;
    while (offset < values->size()) {
        const auto chunkElements = std::min(maximumChunkElements, values->size() - offset);
        const auto chunkBytes = chunkElements * sizeof(T);
        input.read(
            reinterpret_cast<char*>(values->data() + offset),
            static_cast<std::streamsize>(chunkBytes));
        if (!input.good()) {
            return false;
        }
        checksum->AddBytes(values->data() + offset, chunkBytes);
        offset += chunkElements;
    }
    return true;
}

std::uint64_t SaturatingAdd(std::uint64_t left, std::uint64_t right) {
    return right > std::numeric_limits<std::uint64_t>::max() - left
        ? std::numeric_limits<std::uint64_t>::max()
        : left + right;
}

std::uint64_t SaturatingMultiply(std::uint64_t left, std::uint64_t right) {
    if (left == 0U || right == 0U) {
        return 0U;
    }
    return left > std::numeric_limits<std::uint64_t>::max() / right
        ? std::numeric_limits<std::uint64_t>::max()
        : left * right;
}

std::uint64_t WaterSurfaceCachePersistenceBytes(
    const WaterSurfaceCache& cache,
    std::uint64_t gpuSurfaceCount,
    std::uint64_t gpuVegetationCount,
    std::uint64_t gpuFlowSurfaceCount,
    std::uint64_t gpuGroundCount) {
    std::uint64_t bytes = 0U;
    const auto add = [&bytes](std::uint64_t amount) {
        bytes = SaturatingAdd(bytes, amount);
    };
    add(kWaterSurfaceCacheMagic.size());
    add(sizeof(std::uint32_t) + sizeof(float));
    add(sizeof(std::uint32_t) + cache.signature.size());
    add(sizeof(cache.bounds) + sizeof(cache.sourcePointCount) +
        sizeof(cache.groundSourcePointCount) + sizeof(cache.revision));
    add(sizeof(std::uint32_t));
    for (const auto& source : cache.sources) {
        add(sizeof(std::uint32_t) + source.sourcePath.generic_string().size());
        add(sizeof(std::uint32_t) * 2U + sizeof(std::uint64_t) + sizeof(std::int64_t) +
            sizeof(bool));
    }
    add(4U * sizeof(std::uint64_t));
    add(SaturatingMultiply(cache.surfaceCells.size(), sizeof(RainSurfaceCell)));
    add(SaturatingMultiply(cache.vegetationVoxels.size(), sizeof(RainVegetationVoxel)));
    add(SaturatingMultiply(cache.flowSurfaceSurfels.size(), sizeof(WaterSurfaceSurfel)));
    add(SaturatingMultiply(cache.groundCells.size(), sizeof(WaterGroundCell)));
    add(4U * sizeof(std::uint64_t));
    add(7U * sizeof(std::uint32_t) + sizeof(std::uint64_t));
    add(SaturatingMultiply(gpuSurfaceCount, sizeof(RainGpuSurfaceSlot)));
    add(SaturatingMultiply(gpuVegetationCount, sizeof(RainGpuVegetationSlot)));
    add(SaturatingMultiply(gpuFlowSurfaceCount, sizeof(WaterGpuSurfaceSurfelSlot)));
    add(SaturatingMultiply(gpuGroundCount, sizeof(WaterGpuGroundSlot)));
    add(kWaterSurfaceIdentityTrailerMagic.size());
    add(sizeof(std::uint32_t) + cache.signature.size());
    add(4U * sizeof(std::uint64_t));
    add(3U * sizeof(std::uint64_t));
    return bytes;
}

}  // namespace

RainRuntimeSettings DefaultRainRuntimeSettings() {
    return {};
}

RainImpactHeightBand SanitizeRainImpactHeightBand(RainImpactHeightBand band) {
    if (!std::isfinite(band.minZ)) {
        band.minZ = -kRainImpactBandUnbounded;
    }
    if (!std::isfinite(band.maxZ)) {
        band.maxZ = kRainImpactBandUnbounded;
    }
    band.minZ = std::clamp(band.minZ, -kRainImpactBandUnbounded, kRainImpactBandUnbounded);
    band.maxZ = std::clamp(band.maxZ, band.minZ, kRainImpactBandUnbounded);
    band.fadeMeters = std::isfinite(band.fadeMeters)
                          ? std::clamp(band.fadeMeters, 0.01F, 5.0F)
                          : 0.30F;
    return band;
}

float RainImpactBandWeight(const RainImpactHeightBand& band, float pointZ) {
    // Mirrors RainImpactBandWeight in shaders/pointcloud_rain_impact.glsl.
    const float bandFade = std::max(1.0e-3F, band.fadeMeters);
    return std::clamp((pointZ - (band.minZ - bandFade)) / bandFade, 0.0F, 1.0F) *
           std::clamp(((band.maxZ + bandFade) - pointZ) / bandFade, 0.0F, 1.0F);
}

std::uint32_t RainImpactEffectMask(const RainRuntimeSettings& settings) {
    if (!settings.enabled || !settings.impactEffectsEnabled) {
        return 0U;
    }
    std::uint32_t mask = 0U;
    if (settings.sandEffectsEnabled) {
        mask |= kRainImpactEffectRingsBit;
    }
    if (settings.rockEffectsEnabled) {
        mask |= kRainImpactEffectWetnessBit;
    }
    if (settings.vegetationEffectsEnabled) {
        mask |= kRainImpactEffectDropletsBit;
    }
    return mask;
}

RainParticleVisualShape EvaluateRainParticleVisualShape(
    float authoredWidthMeters,
    float authoredLengthMeters,
    float surfaceProximity,
    const RainNearSurfaceSettings& settings) {
    const float proximity = std::clamp(surfaceProximity, 0.0F, 1.0F);
    const float squishAmount =
        std::clamp(settings.squish, 0.0F, 1.0F) * proximity;
    const float approachSpeed = std::lerp(
        1.0F,
        std::clamp(settings.minimumSpeedFactor, 0.05F, 1.0F),
        proximity);
    // Motion blur contracts with the slowed drop. Squaring the remaining
    // unsquished span lets the default 0.65 setting become a readable oval at
    // the surface while retaining the authored streak away from it.
    const float unsquishedSpan = std::max(0.05F, 1.0F - 0.85F * squishAmount);
    RainParticleVisualShape shape;
    shape.widthMeters =
        std::max(0.0001F, authoredWidthMeters) * (1.0F + squishAmount);
    shape.lengthMeters = std::max(
        shape.widthMeters,
        std::max(0.001F, authoredLengthMeters) * approachSpeed *
            unsquishedSpan * unsquishedSpan);
    shape.ellipseBlend = SmoothStep(0.0F, 1.0F, squishAmount);
    return shape;
}

RainIntensityMultipliers RainIntensityValues(RainIntensityPreset preset) {
    switch (preset) {
        case RainIntensityPreset::LightMist:
            return {
                .density = 0.30F,
                .speed = 0.58F,
                .width = 0.58F,
                .length = 0.52F,
                .opacity = 0.50F,
                .emission = 0.42F,
                .effectEnergy = 0.30F,
                .windResponse = 1.55F,
            };
        case RainIntensityPreset::HeavyDownpour:
            return {
                .density = 1.0F,
                .speed = 1.22F,
                .width = 1.32F,
                .length = 1.48F,
                .opacity = 1.18F,
                .emission = 1.30F,
                .effectEnergy = 1.45F,
                .windResponse = 0.32F,
            };
        case RainIntensityPreset::Rain:
            return {};
    }
    return {};
}

WaterRainVisualSettings RainVisualPreset(std::string_view name) {
    if (name == "Rain Mist" || name == "Rain Mist_preset") {
        return {
            .colour = {0.72F, 0.83F, 0.88F},
            .widthMeters = 0.0021F,
            .streakLengthMeters = 0.075F,
            .softness = 0.72F,
            .opacity = 0.34F,
            .emission = 0.08F,
            .minimumScreenPixels = 0.45F,
            .maximumScreenPixels = 2.2F,
        };
    }
    if (name == "Rain Downpour" || name == "Rain Downpour_preset") {
        return {
            .colour = {0.62F, 0.78F, 0.91F},
            .widthMeters = 0.0042F,
            .streakLengthMeters = 0.25F,
            .softness = 0.28F,
            .opacity = 0.72F,
            .emission = 0.24F,
            .minimumScreenPixels = 0.9F,
            .maximumScreenPixels = 5.5F,
        };
    }
    return {};
}

std::array<std::string_view, 3> RainVisualPresetNames() {
    return {"Rain Mist", "Rain Fine Lines", "Rain Downpour"};
}

float RainImpactGridWorldSpan(const RainRuntimeSettings& settings) {
    return std::clamp(std::max(settings.spawnRadiusMeters * 2.4F, 16.0F), 16.0F, 256.0F);
}

std::vector<WaterSurfaceSource> SelectWaterSurfaceSources(
    const scene::ScenePointCloudGroup& group,
    std::uint32_t preferredSpacingMicrometres) {
    std::vector<WaterSurfaceSource> selected;
    const auto exactBundle = group.FindCompleteDisplayBundle(preferredSpacingMicrometres);
    const auto nearestBundle = std::min_element(
        group.completeDisplayBundles.begin(),
        group.completeDisplayBundles.end(),
        [preferredSpacingMicrometres](const auto& left, const auto& right) {
            const auto spacingDistance = [preferredSpacingMicrometres](std::uint32_t spacing) {
                return spacing > preferredSpacingMicrometres
                           ? spacing - preferredSpacingMicrometres
                           : preferredSpacingMicrometres - spacing;
            };
            const auto leftDistance = spacingDistance(left.spacingMicrometres);
            const auto rightDistance = spacingDistance(right.spacingMicrometres);
            if (leftDistance != rightDistance) {
                return leftDistance < rightDistance;
            }
            // Prefer the coarser complete bundle on an exact tie to bound the
            // one-time fallback scan without ever mixing role spacings.
            return left.spacingMicrometres > right.spacingMicrometres;
        });
    const auto* bundle = exactBundle != nullptr
                             ? exactBundle
                             : (nearestBundle == group.completeDisplayBundles.end()
                                    ? nullptr
                                    : &*nearestBundle);
    if (bundle == nullptr) {
        return selected;
    }
    const bool isFallback = bundle->spacingMicrometres != preferredSpacingMicrometres;
    selected.reserve(scene::kScenePointCloudRoleCount);
    for (std::size_t roleIndex = 0; roleIndex < scene::kScenePointCloudRoleCount; ++roleIndex) {
        const auto role = static_cast<scene::ScenePointCloudRole>(roleIndex);
        const auto& variant = bundle->Find(role);
        selected.push_back({
            .sourcePath = variant.sourcePath,
            .role = CollisionRoleForSceneRole(role),
            .spacingMicrometres = variant.spacingMicrometres,
            .isFallback = isFallback,
        });
    }
    return selected;
}

std::string WaterSurfaceCacheSignature(
    std::span<const WaterSurfaceSource> sources,
    float resolutionMeters) {
    return WaterSurfaceCacheSignatureForSchema(
        sources,
        resolutionMeters,
        kWaterSurfaceCacheSchemaVersion);
}

static std::filesystem::path LegacyRainCachePath(
    const std::filesystem::path& cacheRoot,
    std::string_view signature) {
    return cacheRoot / "cache" / "rain" / (std::string{signature} + ".raincache");
}

WaterSurfaceBuildResult BuildWaterSurfaceCache(
    std::span<const WaterSurfaceSource> sources,
    const std::filesystem::path& cacheRoot,
    const std::atomic_bool* cancelRequested) {
    WaterSurfaceBuildResult result;
    if (sources.empty()) {
        result.errorMessage = "Water surface cache has no role sources.";
        return result;
    }

    const auto signature = WaterSurfaceCacheSignature(sources);
    const auto cachePath = cacheRoot.empty()
        ? std::filesystem::path{}
        : WaterSurfaceCachePath(cacheRoot, signature);
    if (!cachePath.empty()) {
        const auto legacySignature = WaterSurfaceCacheSignatureForSchema(
            sources,
            kWaterSurfaceResolutionMeters,
            kWaterSurfaceCacheLegacySchemaVersion);
        const std::array<std::pair<std::filesystem::path, std::string>, 3> candidates{{
            {cachePath, signature},
            {LegacyRainCachePath(cacheRoot, signature), signature},
            {LegacyRainCachePath(cacheRoot, legacySignature), legacySignature},
        }};
        for (const auto& [candidatePath, candidateSignature] : candidates) {
            std::error_code candidateError;
            if (!std::filesystem::is_regular_file(candidatePath, candidateError) ||
                candidateError ||
                !LoadWaterSurfaceCache(
                    candidatePath,
                    candidateSignature,
                    &result.cache,
                    &result.errorMessage,
                    &result.diagnostics)) {
                result.errorMessage.clear();
                continue;
            }

            // Older files remain directly readable for recovery, but the
            // algorithm-id revision deliberately makes them stale. Re-scan the
            // authored sources instead of migrating semantically incomplete
            // payloads (notably schema 3, which has no sampled Ground stream).
            if (result.cache.schemaVersion != kWaterSurfaceCacheSchemaVersion ||
                result.cache.signature != signature) {
                result.cache = {};
                continue;
            }
            if (candidatePath != cachePath) {
                std::string migrationError;
                if (!SaveWaterSurfaceCache(result.cache, cachePath, &migrationError)) {
                    result.warnings.push_back(
                        "Water surface cache migration could not be persisted: " +
                        migrationError);
                } else {
                    result.persistedPath = cachePath;
                }
            } else {
                result.persistedPath = candidatePath;
            }
            result.loadedFromDisk = true;
            result.success = true;
            return result;
        }
    }

    const auto startedAt = std::chrono::steady_clock::now();
    CollisionAccumulator accumulator{kWaterSurfaceResolutionMeters};
    result.cache.sources.reserve(sources.size());
    for (const auto& source : sources) {
        if (cancelRequested != nullptr && cancelRequested->load(std::memory_order_relaxed)) {
            result.cancelled = true;
            return result;
        }
        if (source.role == WaterSurfaceRole::None) {
            continue;
        }
        if (source.isFallback) {
            result.warnings.push_back(
                "Water surface cache is using fallback " + std::to_string(source.spacingMicrometres / 1000U) +
                " mm data for " + source.sourcePath.filename().string() + ".");
        }

        std::error_code sizeError;
        const auto size = std::filesystem::file_size(source.sourcePath, sizeError);
        std::error_code timeError;
        const auto writeTime = std::filesystem::last_write_time(source.sourcePath, timeError);
        result.cache.sources.push_back({
            .sourcePath = source.sourcePath,
            .role = source.role,
            .spacingMicrometres = source.spacingMicrometres,
            .fileSize = sizeError ? 0U : size,
            .modificationTicks = timeError
                ? 0LL
                : static_cast<std::int64_t>(writeTime.time_since_epoch().count()),
            .isFallback = source.isFallback,
        });

        ++result.diagnostics.sourceScanCount;
        const auto streamResult = io::StreamPointCloudPositionsNormals(
            source.sourcePath,
            [&](const io::PointCloudPositionNormalSample& sample, std::uint64_t) {
                if (cancelRequested != nullptr && cancelRequested->load(std::memory_order_relaxed)) {
                    return false;
                }
                accumulator.AddSample({
                    .position = TransformPosition(sample.position, source),
                    .normal = sample.hasNormal ? TransformNormal(sample.normal, source) : io::Float3{0.0F, 0.0F, 1.0F},
                    .role = source.role,
                    .roughness = sample.roughness,
                    .hasRoughness = sample.hasRoughness,
                });
                return true;
            });
        if (streamResult.cancelled) {
            result.cancelled = true;
            return result;
        }
        if (!streamResult.success) {
            result.errorMessage = source.sourcePath.filename().string() + ": " + streamResult.errorMessage;
            return result;
        }
    }

    auto cache = accumulator.Finish();
    cache.signature = signature;
    cache.sources = std::move(result.cache.sources);
    cache.revision = HashBits(static_cast<std::uint32_t>(std::hash<std::string>{}(signature)));
    cache.gpuData = BuildWaterSurfaceGpuData(cache);
    ++result.diagnostics.gpuTableBuildCount;
    cache.cacheIdentity = cache.gpuData.sourceIdentity;
    cache.buildMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - startedAt).count();
    result.cache = std::move(cache);
    if (result.cache.surfaceCells.empty() && result.cache.vegetationVoxels.empty() &&
        result.cache.groundCells.empty()) {
        result.errorMessage = "Water surface cache did not contain any occupied cells.";
        return result;
    }
    if (!cachePath.empty()) {
        std::string saveError;
        if (!SaveWaterSurfaceCache(result.cache, cachePath, &saveError)) {
            result.warnings.push_back("Water surface cache could not be persisted: " + saveError);
        } else {
            result.persistedPath = cachePath;
        }
    }
    result.success = true;
    return result;
}

WaterSurfaceCache BuildWaterSurfaceCacheFromSamples(
    std::span<const WaterSurfaceSample> samples,
    float resolutionMeters) {
    CollisionAccumulator accumulator{resolutionMeters};
    for (const auto& sample : samples) {
        accumulator.AddSample(sample);
    }
    auto cache = accumulator.Finish();
    cache.signature = "memory";
    cache.revision = 1U;
    cache.gpuData = BuildWaterSurfaceGpuData(cache);
    cache.cacheIdentity = cache.gpuData.sourceIdentity;
    return cache;
}

bool SaveWaterSurfaceCache(
    const WaterSurfaceCache& cache,
    const std::filesystem::path& filePath,
    std::string* errorMessage) {
    if (!WaterSurfaceCacheFitsPersistenceLimit(cache)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache exceeds the 5 GiB persistence limit.";
        }
        return false;
    }
    std::error_code error;
    if (!filePath.parent_path().empty()) {
        std::filesystem::create_directories(filePath.parent_path(), error);
    }
    if (error) {
        if (errorMessage != nullptr) {
            *errorMessage = error.message();
        }
        return false;
    }
    const auto sourceCount = static_cast<std::uint32_t>(cache.sources.size());
    const auto surfaceCount = static_cast<std::uint64_t>(cache.surfaceCells.size());
    const auto vegetationCount = static_cast<std::uint64_t>(cache.vegetationVoxels.size());
    const auto flowSurfaceCount = static_cast<std::uint64_t>(cache.flowSurfaceSurfels.size());
    const auto groundCount = static_cast<std::uint64_t>(cache.groundCells.size());
    WaterSurfaceGpuData generatedGpuData;
    const WaterSurfaceGpuData* gpuData = &cache.gpuData;
    if (cache.gpuData.sourceRevision != cache.revision || cache.gpuData.surfaceTable.empty() ||
        cache.gpuData.vegetationTable.empty() || cache.gpuData.flowSurfaceTable.empty() ||
        cache.gpuData.groundTable.empty()) {
        try {
            generatedGpuData = BuildWaterSurfaceGpuData(cache);
        } catch (const std::exception& buildError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Water surface GPU-table preparation failed: " +
                                std::string{buildError.what()};
            }
            return false;
        }
        gpuData = &generatedGpuData;
    }
    const auto gpuSurfaceCount = static_cast<std::uint64_t>(gpuData->surfaceTable.size());
    const auto gpuVegetationCount = static_cast<std::uint64_t>(gpuData->vegetationTable.size());
    const auto gpuFlowSurfaceCount = static_cast<std::uint64_t>(gpuData->flowSurfaceTable.size());
    const auto gpuGroundCount = static_cast<std::uint64_t>(gpuData->groundTable.size());
    auto payloadChecksum = gpuData->payloadChecksum;
    if (!payloadChecksum.Valid()) {
        payloadChecksum = CalculateWaterSurfacePayloadChecksum(cache, *gpuData);
    }
    const bool canReuseIdentity =
        cache.schemaVersion == kWaterSurfaceCacheSchemaVersion &&
        cache.cacheIdentity.Valid() &&
        cache.cacheIdentity.sourceSignature == cache.signature &&
        gpuData->sourceIdentity == cache.cacheIdentity &&
        gpuData->payloadChecksum == payloadChecksum;
    const auto cacheIdentity = canReuseIdentity
        ? cache.cacheIdentity
        : MakeWaterSurfaceCacheIdentity(
              cache,
              *gpuData,
              kWaterSurfaceCacheSchemaVersion,
              &payloadChecksum);
    const auto persistenceBytes = WaterSurfaceCachePersistenceBytes(
        cache,
        gpuSurfaceCount,
        gpuVegetationCount,
        gpuFlowSurfaceCount,
        gpuGroundCount);
    if (!WaterSurfaceCachePersistenceSizeAllowed(persistenceBytes)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache exceeds the 5 GiB persistence limit.";
        }
        return false;
    }

    const auto temporaryPath = filePath.string() + ".tmp";
    std::ofstream output{temporaryPath, std::ios::binary | std::ios::trunc};
    if (!output.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Unable to open temporary cache file.";
        }
        return false;
    }
    output.write(
        kWaterSurfaceCacheMagic.data(),
        static_cast<std::streamsize>(kWaterSurfaceCacheMagic.size()));
    bool ok = WritePod(output, kWaterSurfaceCacheSchemaVersion) &&
              WritePod(output, cache.resolutionMeters) &&
              WriteString(output, cache.signature) && WritePod(output, cache.bounds) &&
              WritePod(output, cache.sourcePointCount) &&
              WritePod(output, cache.groundSourcePointCount) &&
              WritePod(output, cache.revision) &&
              WritePod(output, sourceCount);
    for (const auto& source : cache.sources) {
        const auto role = static_cast<std::uint32_t>(source.role);
        ok = ok && WriteString(output, source.sourcePath.generic_string()) && WritePod(output, role) &&
             WritePod(output, source.spacingMicrometres) && WritePod(output, source.fileSize) &&
             WritePod(output, source.modificationTicks) && WritePod(output, source.isFallback);
    }
    ok = ok && WritePod(output, surfaceCount) && WritePod(output, vegetationCount) &&
         WritePod(output, flowSurfaceCount) && WritePod(output, groundCount);
    if (ok && surfaceCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(cache.surfaceCells.data()),
            static_cast<std::streamsize>(surfaceCount * sizeof(RainSurfaceCell)));
        ok = output.good();
    }
    if (ok && vegetationCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(cache.vegetationVoxels.data()),
            static_cast<std::streamsize>(vegetationCount * sizeof(RainVegetationVoxel)));
        ok = output.good();
    }
    if (ok && flowSurfaceCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(cache.flowSurfaceSurfels.data()),
            static_cast<std::streamsize>(flowSurfaceCount * sizeof(WaterSurfaceSurfel)));
        ok = output.good();
    }
    if (ok && groundCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(cache.groundCells.data()),
            static_cast<std::streamsize>(groundCount * sizeof(WaterGroundCell)));
        ok = output.good();
    }
    ok = ok && WritePod(output, gpuSurfaceCount) && WritePod(output, gpuVegetationCount) &&
         WritePod(output, gpuFlowSurfaceCount) && WritePod(output, gpuGroundCount) &&
         WritePod(output, gpuData->surfaceMask) &&
         WritePod(output, gpuData->vegetationMask) &&
         WritePod(output, gpuData->flowSurfaceMask) &&
         WritePod(output, gpuData->groundMask) &&
         WritePod(output, gpuData->maximumProbeCount) &&
         WritePod(output, gpuData->flowMaximumProbeCount) &&
         WritePod(output, gpuData->groundMaximumProbeCount) &&
         WritePod(output, gpuData->sourceRevision);
    if (ok && gpuSurfaceCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(gpuData->surfaceTable.data()),
            static_cast<std::streamsize>(gpuSurfaceCount * sizeof(RainGpuSurfaceSlot)));
        ok = output.good();
    }
    if (ok && gpuVegetationCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(gpuData->vegetationTable.data()),
            static_cast<std::streamsize>(gpuVegetationCount * sizeof(RainGpuVegetationSlot)));
        ok = output.good();
    }
    if (ok && gpuFlowSurfaceCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(gpuData->flowSurfaceTable.data()),
            static_cast<std::streamsize>(gpuFlowSurfaceCount * sizeof(WaterGpuSurfaceSurfelSlot)));
        ok = output.good();
    }
    if (ok && gpuGroundCount != 0U) {
        output.write(
            reinterpret_cast<const char*>(gpuData->groundTable.data()),
            static_cast<std::streamsize>(gpuGroundCount * sizeof(WaterGpuGroundSlot)));
        ok = output.good();
    }
    if (ok) {
        output.write(
            kWaterSurfaceIdentityTrailerMagic.data(),
            static_cast<std::streamsize>(kWaterSurfaceIdentityTrailerMagic.size()));
        ok = output.good() && WriteString(output, cacheIdentity.sourceSignature);
        for (const auto word : cacheIdentity.contentDigest) {
            ok = ok && WritePod(output, word);
        }
        for (const auto word : payloadChecksum.words) {
            ok = ok && WritePod(output, word);
        }
        ok = ok && WritePod(output, payloadChecksum.hashedByteCount);
    }
    if (ok) {
        output.flush();
        ok = output.good();
    }
    output.close();
    ok = ok && !output.fail();
    if (!ok) {
        std::filesystem::remove(temporaryPath, error);
        if (errorMessage != nullptr) {
            *errorMessage = "Failed while writing water surface cache.";
        }
        return false;
    }
    std::filesystem::rename(temporaryPath, filePath, error);
    if (error) {
        const auto renameError = error;
        std::error_code cleanupError;
        std::filesystem::remove(temporaryPath, cleanupError);
        if (errorMessage != nullptr) {
            *errorMessage = renameError.message();
        }
        return false;
    }
    return true;
}

bool LoadWaterSurfaceCache(
    const std::filesystem::path& filePath,
    std::string_view expectedSignature,
    WaterSurfaceCache* cache,
    std::string* errorMessage,
    WaterSurfaceBuildDiagnostics* diagnostics) {
    if (cache == nullptr) {
        return false;
    }
    std::error_code fileError;
    const auto fileBytes = std::filesystem::file_size(filePath, fileError);
    std::error_code writeTimeError;
    const auto fileWriteTime = std::filesystem::last_write_time(filePath, writeTimeError);
    if (fileError || writeTimeError ||
        !WaterSurfaceCachePersistenceSizeAllowed(fileBytes)) {
        if (errorMessage != nullptr) {
            *errorMessage = fileError || writeTimeError
                                ? "Unable to inspect the water surface cache file."
                                : "Water surface cache exceeds the 5 GiB persistence limit.";
        }
        return false;
    }
    std::ifstream input{filePath, std::ios::binary};
    if (!input.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Unable to open water surface cache.";
        }
        return false;
    }
    const auto unreadFileBytes = [&]() -> std::optional<std::uint64_t> {
        const auto position = input.tellg();
        if (position == std::ifstream::pos_type{-1}) {
            return std::nullopt;
        }
        const auto consumed = static_cast<std::streamoff>(position);
        if (consumed < 0 || static_cast<std::uint64_t>(consumed) > fileBytes) {
            return std::nullopt;
        }
        return static_cast<std::uint64_t>(fileBytes) -
               static_cast<std::uint64_t>(consumed);
    };
    std::array<char, kWaterSurfaceCacheMagic.size()> magic{};
    input.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    WaterSurfaceCache loaded;
    const auto magicView = std::string_view{magic.data(), magic.size()};
    const bool legacySchema = magicView == kLegacyRainCacheMagic;
    const bool previousSchema = magicView == kPreviousWaterSurfaceCacheMagic;
    const bool currentSchema = magicView == kWaterSurfaceCacheMagic;
    const bool checksummedSchema = previousSchema || currentSchema;
    if (!input.good() ||
        (!legacySchema && !previousSchema && !currentSchema) ||
        !ReadPod(input, &loaded.schemaVersion) ||
        (legacySchema && loaded.schemaVersion != kWaterSurfaceCacheLegacySchemaVersion) ||
        (previousSchema &&
         loaded.schemaVersion != kWaterSurfaceCachePreviousSchemaVersion) ||
        (currentSchema && loaded.schemaVersion != kWaterSurfaceCacheSchemaVersion) ||
        !ReadPod(input, &loaded.resolutionMeters) || !ReadString(input, &loaded.signature) ||
        loaded.signature != expectedSignature || !ReadPod(input, &loaded.bounds) ||
        !ReadPod(input, &loaded.sourcePointCount) ||
        (currentSchema && !ReadPod(input, &loaded.groundSourcePointCount)) ||
        !ReadPod(input, &loaded.revision)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache header is invalid or stale.";
        }
        return false;
    }
    std::uint32_t sourceCount = 0U;
    constexpr std::uint32_t maximumCurrentSourceCount =
        static_cast<std::uint32_t>(scene::kScenePointCloudRoleCount) + 1U;
    const auto maximumSourceCount = currentSchema
        ? maximumCurrentSourceCount
        : static_cast<std::uint32_t>(scene::kScenePointCloudRoleCount);
    if (!ReadPod(input, &sourceCount) || sourceCount > maximumSourceCount) {
        return false;
    }
    loaded.sources.reserve(sourceCount);
    for (std::uint32_t index = 0; index < sourceCount; ++index) {
        WaterSurfaceSourceMetadata source;
        std::string path;
        std::uint32_t role = 0U;
        if (!ReadString(input, &path) || !ReadPod(input, &role) ||
            role > static_cast<std::uint32_t>(
                       currentSchema
                           ? WaterSurfaceRole::Ground
                           : WaterSurfaceRole::Vegetation) ||
            !ReadPod(input, &source.spacingMicrometres) || !ReadPod(input, &source.fileSize) ||
            !ReadPod(input, &source.modificationTicks) || !ReadPod(input, &source.isFallback)) {
            return false;
        }
        source.sourcePath = path;
        source.role = static_cast<WaterSurfaceRole>(role);
        loaded.sources.push_back(std::move(source));
    }
    std::uint64_t surfaceCount = 0U;
    std::uint64_t vegetationCount = 0U;
    std::uint64_t flowSurfaceCount = 0U;
    std::uint64_t groundCount = 0U;
    constexpr std::uint64_t maximumCacheCells = 200'000'000ULL;
    if (!ReadPod(input, &surfaceCount) || !ReadPod(input, &vegetationCount) ||
        !ReadPod(input, &flowSurfaceCount) ||
        (currentSchema && !ReadPod(input, &groundCount)) ||
        surfaceCount > maximumCacheCells ||
        vegetationCount > maximumCacheCells ||
        flowSurfaceCount > maximumCacheCells ||
        groundCount > maximumCacheCells) {
        return false;
    }
    const auto cpuPayloadBytes = SaturatingAdd(
        SaturatingMultiply(surfaceCount, sizeof(RainSurfaceCell)),
        SaturatingAdd(
            SaturatingMultiply(vegetationCount, sizeof(RainVegetationVoxel)),
            SaturatingAdd(
                SaturatingMultiply(
                    flowSurfaceCount,
                    sizeof(WaterSurfaceSurfel)),
                SaturatingMultiply(
                    groundCount,
                    sizeof(WaterGroundCell)))));
    const auto unreadBeforeCpuPayload = unreadFileBytes();
    if (!unreadBeforeCpuPayload.has_value() ||
        cpuPayloadBytes > unreadBeforeCpuPayload.value() ||
        !WaterSurfaceCachePersistenceSizeAllowed(cpuPayloadBytes)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache CPU payload is invalid.";
        }
        return false;
    }
    try {
        loaded.surfaceCells.resize(static_cast<std::size_t>(surfaceCount));
        loaded.vegetationVoxels.resize(static_cast<std::size_t>(vegetationCount));
        loaded.flowSurfaceSurfels.resize(static_cast<std::size_t>(flowSurfaceCount));
        loaded.groundCells.resize(static_cast<std::size_t>(groundCount));
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = error.what();
        }
        return false;
    }
    WaterSurfacePayloadChecksumBuilder payloadChecksumBuilder;
    if (!ReadWaterSurfacePayloadArray(
            input,
            &loaded.surfaceCells,
            1U,
            &payloadChecksumBuilder) ||
        !ReadWaterSurfacePayloadArray(
            input,
            &loaded.vegetationVoxels,
            2U,
            &payloadChecksumBuilder) ||
        !ReadWaterSurfacePayloadArray(
            input,
            &loaded.flowSurfaceSurfels,
            3U,
            &payloadChecksumBuilder) ||
        (currentSchema &&
         !ReadWaterSurfacePayloadArray(
             input,
             &loaded.groundCells,
             7U,
             &payloadChecksumBuilder))) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache CPU payload is truncated.";
        }
        return false;
    }
    std::uint64_t gpuSurfaceCount = 0U;
    std::uint64_t gpuVegetationCount = 0U;
    std::uint64_t gpuFlowSurfaceCount = 0U;
    std::uint64_t gpuGroundCount = 0U;
    if (!input.good() || !ReadPod(input, &gpuSurfaceCount) ||
        !ReadPod(input, &gpuVegetationCount) ||
        !ReadPod(input, &gpuFlowSurfaceCount) ||
        (currentSchema && !ReadPod(input, &gpuGroundCount)) ||
        gpuSurfaceCount > maximumCacheCells ||
        gpuVegetationCount > maximumCacheCells ||
        gpuFlowSurfaceCount > maximumCacheCells ||
        gpuGroundCount > maximumCacheCells ||
        !ReadPod(input, &loaded.gpuData.surfaceMask) ||
        !ReadPod(input, &loaded.gpuData.vegetationMask) ||
        !ReadPod(input, &loaded.gpuData.flowSurfaceMask) ||
        (currentSchema && !ReadPod(input, &loaded.gpuData.groundMask)) ||
        !ReadPod(input, &loaded.gpuData.maximumProbeCount) ||
        !ReadPod(input, &loaded.gpuData.flowMaximumProbeCount) ||
        (currentSchema &&
         !ReadPod(input, &loaded.gpuData.groundMaximumProbeCount)) ||
        !ReadPod(input, &loaded.gpuData.sourceRevision)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache payload is truncated.";
        }
        return false;
    }
    const auto validTableSize = [](std::uint64_t count, std::uint32_t mask) {
        return count >= 2U && std::has_single_bit(count) && mask == count - 1U;
    };
    if (!validTableSize(gpuSurfaceCount, loaded.gpuData.surfaceMask) ||
        !validTableSize(gpuVegetationCount, loaded.gpuData.vegetationMask) ||
        !validTableSize(gpuFlowSurfaceCount, loaded.gpuData.flowSurfaceMask) ||
        (currentSchema &&
         !validTableSize(gpuGroundCount, loaded.gpuData.groundMask)) ||
        loaded.gpuData.maximumProbeCount > kMaximumHashProbeCount ||
        loaded.gpuData.flowMaximumProbeCount > kMaximumHashProbeCount ||
        loaded.gpuData.groundMaximumProbeCount > kMaximumHashProbeCount ||
        loaded.gpuData.sourceRevision != loaded.revision) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache GPU table header is invalid.";
        }
        return false;
    }
    const auto gpuPayloadBytes = SaturatingAdd(
        SaturatingMultiply(gpuSurfaceCount, sizeof(RainGpuSurfaceSlot)),
        SaturatingAdd(
            SaturatingMultiply(gpuVegetationCount, sizeof(RainGpuVegetationSlot)),
            SaturatingAdd(
                SaturatingMultiply(
                    gpuFlowSurfaceCount,
                    sizeof(WaterGpuSurfaceSurfelSlot)),
                SaturatingMultiply(
                    gpuGroundCount,
                    sizeof(WaterGpuGroundSlot)))));
    const auto unreadBeforeGpuPayload = unreadFileBytes();
    if (!unreadBeforeGpuPayload.has_value() ||
        gpuPayloadBytes > unreadBeforeGpuPayload.value() ||
        !WaterSurfaceCachePersistenceSizeAllowed(gpuPayloadBytes)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache GPU payload is invalid.";
        }
        return false;
    }
    bool gpuPayloadRead = true;
    if (checksummedSchema) {
        std::vector<std::byte> scratch;
        WaterSurfaceGpuStreamChecksumBuilder gpuStreamChecksumBuilder;
        try {
            scratch.resize(static_cast<std::size_t>(std::min<std::uint64_t>(
                kWaterSurfacePayloadIoChunkBytes,
                std::max<std::uint64_t>(1U, gpuPayloadBytes))));
        } catch (const std::exception& error) {
            if (errorMessage != nullptr) {
                *errorMessage = error.what();
            }
            return false;
        }
        loaded.gpuData.persistedTables.filePath = filePath;
        loaded.gpuData.persistedTables.fileSize =
            static_cast<std::uint64_t>(fileBytes);
        loaded.gpuData.persistedTables.modificationTicks =
            static_cast<std::int64_t>(fileWriteTime.time_since_epoch().count());
        loaded.gpuData.persistedTables.surfaceCount = gpuSurfaceCount;
        loaded.gpuData.persistedTables.vegetationCount = gpuVegetationCount;
        loaded.gpuData.persistedTables.flowSurfaceCount = gpuFlowSurfaceCount;
        loaded.gpuData.persistedTables.groundCount = gpuGroundCount;
        const auto readPersistedSection = [&](std::uint64_t count,
                                              std::uint64_t elementBytes,
                                              std::uint64_t tag,
                                              std::uint64_t* fileOffset) {
            const auto position = input.tellg();
            if (fileOffset == nullptr || position == std::ifstream::pos_type{-1}) {
                return false;
            }
            *fileOffset = static_cast<std::uint64_t>(
                static_cast<std::streamoff>(position));
            payloadChecksumBuilder.AddPod(tag);
            payloadChecksumBuilder.AddPod(count);
            gpuStreamChecksumBuilder.AddPod(tag);
            gpuStreamChecksumBuilder.AddPod(count);
            std::uint64_t remaining = SaturatingMultiply(count, elementBytes);
            while (remaining > 0U) {
                const auto chunkBytes = static_cast<std::size_t>(std::min<std::uint64_t>(
                    remaining,
                    scratch.size()));
                input.read(
                    reinterpret_cast<char*>(scratch.data()),
                    static_cast<std::streamsize>(chunkBytes));
                if (!input.good()) {
                    return false;
                }
                payloadChecksumBuilder.AddBytes(scratch.data(), chunkBytes);
                gpuStreamChecksumBuilder.AddBytes(scratch.data(), chunkBytes);
                remaining -= chunkBytes;
            }
            return true;
        };
        gpuPayloadRead =
            readPersistedSection(
                gpuSurfaceCount,
                sizeof(RainGpuSurfaceSlot),
                4U,
                &loaded.gpuData.persistedTables.surfaceOffset) &&
            readPersistedSection(
                gpuVegetationCount,
                sizeof(RainGpuVegetationSlot),
                5U,
                &loaded.gpuData.persistedTables.vegetationOffset) &&
            readPersistedSection(
                gpuFlowSurfaceCount,
                sizeof(WaterGpuSurfaceSurfelSlot),
                6U,
                &loaded.gpuData.persistedTables.flowSurfaceOffset) &&
            (!currentSchema ||
             readPersistedSection(
                 gpuGroundCount,
                 sizeof(WaterGpuGroundSlot),
                 8U,
                 &loaded.gpuData.persistedTables.groundOffset));
        loaded.gpuData.persistedTables.streamChecksum =
            gpuStreamChecksumBuilder.Finish();
    } else {
        try {
            loaded.gpuData.surfaceTable.resize(static_cast<std::size_t>(gpuSurfaceCount));
            loaded.gpuData.vegetationTable.resize(static_cast<std::size_t>(gpuVegetationCount));
            loaded.gpuData.flowSurfaceTable.resize(static_cast<std::size_t>(gpuFlowSurfaceCount));
        } catch (const std::exception& error) {
            if (errorMessage != nullptr) {
                *errorMessage = error.what();
            }
            return false;
        }
        gpuPayloadRead =
            ReadWaterSurfacePayloadArray(
                input,
                &loaded.gpuData.surfaceTable,
                4U,
                &payloadChecksumBuilder) &&
            ReadWaterSurfacePayloadArray(
                input,
                &loaded.gpuData.vegetationTable,
                5U,
                &payloadChecksumBuilder) &&
            ReadWaterSurfacePayloadArray(
                input,
                &loaded.gpuData.flowSurfaceTable,
                6U,
                &payloadChecksumBuilder);
    }
    if (!gpuPayloadRead) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water surface cache GPU table payload is truncated.";
        }
        return false;
    }
    loaded.gpuData.payloadChecksum = payloadChecksumBuilder.Finish();

    WaterSurfaceCacheIdentity derivedIdentity;
    if (checksummedSchema) {
        if (input.peek() == std::char_traits<char>::eof()) {
            if (errorMessage != nullptr) {
                *errorMessage = "Water surface cache checksum trailer is missing.";
            }
            return false;
        }
        std::array<char, kWaterSurfaceIdentityTrailerMagic.size()> trailerMagic{};
        WaterSurfaceCacheIdentity persistedIdentity;
        WaterSurfaceCachePayloadChecksum persistedChecksum;
        input.read(trailerMagic.data(), static_cast<std::streamsize>(trailerMagic.size()));
        const auto expectedTrailerMagic = currentSchema
            ? kWaterSurfaceIdentityTrailerMagic
            : kPreviousWaterSurfaceIdentityTrailerMagic;
        bool identityOk = input.good() &&
                          std::string_view{trailerMagic.data(), trailerMagic.size()} ==
                              expectedTrailerMagic &&
                          ReadString(input, &persistedIdentity.sourceSignature);
        for (auto& word : persistedIdentity.contentDigest) {
            identityOk = identityOk && ReadPod(input, &word);
        }
        for (auto& word : persistedChecksum.words) {
            identityOk = identityOk && ReadPod(input, &word);
        }
        identityOk = identityOk && ReadPod(input, &persistedChecksum.hashedByteCount);
        derivedIdentity = MakeWaterSurfaceCacheIdentity(
            loaded,
            loaded.gpuData,
            loaded.schemaVersion,
            &loaded.gpuData.payloadChecksum);
        if (!identityOk || persistedChecksum != loaded.gpuData.payloadChecksum ||
            persistedIdentity != derivedIdentity ||
            input.peek() != std::char_traits<char>::eof()) {
            if (errorMessage != nullptr) {
                *errorMessage = "Water surface cache checksum or identity trailer is invalid.";
            }
            return false;
        }
    } else {
        // Schema 2 used an expensive four-lane byte digest and allowed files
        // without a trailer. Pay that cost only for the one-time legacy read;
        // BuildWaterSurfaceCache migrates the result to the current schema
        // afterwards.
        if (diagnostics != nullptr) {
            ++diagnostics->fullPayloadHashPassCount;
        }
        derivedIdentity = MakeLegacyWaterSurfaceCacheIdentity(loaded, loaded.gpuData);
        if (input.peek() != std::char_traits<char>::eof()) {
            std::array<char, kLegacyWaterSurfaceIdentityTrailerMagic.size()> trailerMagic{};
            WaterSurfaceCacheIdentity persistedIdentity;
            input.read(trailerMagic.data(), static_cast<std::streamsize>(trailerMagic.size()));
            bool identityOk = input.good() &&
                              std::string_view{trailerMagic.data(), trailerMagic.size()} ==
                                  kLegacyWaterSurfaceIdentityTrailerMagic &&
                              ReadString(input, &persistedIdentity.sourceSignature);
            for (auto& word : persistedIdentity.contentDigest) {
                identityOk = identityOk && ReadPod(input, &word);
            }
            if (!identityOk || persistedIdentity != derivedIdentity ||
                input.peek() != std::char_traits<char>::eof()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "Legacy water surface cache identity trailer is invalid.";
                }
                return false;
            }
        }
    }
    loaded.cacheIdentity = derivedIdentity;
    loaded.gpuData.sourceIdentity = derivedIdentity;
    if (checksummedSchema) {
        // The renderer will reopen this path later and stream the tables
        // directly. Capture a stable snapshot now so an atomic replacement or
        // in-place edit between validation and upload cannot be mistaken for
        // the payload that produced the identity above.
        std::error_code finalSizeError;
        const auto finalFileBytes = std::filesystem::file_size(filePath, finalSizeError);
        std::error_code finalWriteTimeError;
        const auto finalWriteTime =
            std::filesystem::last_write_time(filePath, finalWriteTimeError);
        if (finalSizeError || finalWriteTimeError || finalFileBytes != fileBytes ||
            finalWriteTime != fileWriteTime) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    "Water surface cache changed while it was being validated.";
            }
            return false;
        }
    }
    *cache = std::move(loaded);
    return true;
}

RainCollisionHit TraceRainCollision(
    const WaterSurfaceCache& cache,
    const io::Float3& segmentStart,
    const io::Float3& segmentEnd) {
    if (cache.resolutionMeters <= 0.0F ||
        (cache.surfaceCells.empty() && cache.vegetationVoxels.empty())) {
        return {};
    }

    constexpr float epsilon = 1.0e-6F;
    constexpr std::uint32_t maximumTraversalCells = 16'384U;
    const float resolution = cache.resolutionMeters;
    const auto delta = Subtract(segmentEnd, segmentStart);
    auto cell = PositionCell3(segmentStart, resolution);
    const auto endCell = PositionCell3(segmentEnd, resolution);
    const auto axisStep = [&](float value) -> std::int32_t {
        return value > epsilon ? 1 : (value < -epsilon ? -1 : 0);
    };
    const Cell3 step{axisStep(delta.x), axisStep(delta.y), axisStep(delta.z)};
    const float infinity = std::numeric_limits<float>::infinity();
    const auto firstBoundaryTime = [&](float start, float direction, std::int32_t coordinate, std::int32_t directionStep) {
        if (directionStep == 0) {
            return infinity;
        }
        const float boundary = static_cast<float>(coordinate + (directionStep > 0 ? 1 : 0)) * resolution;
        return std::max(0.0F, (boundary - start) / direction);
    };
    io::Float3 maximumTime{
        firstBoundaryTime(segmentStart.x, delta.x, cell.x, step.x),
        firstBoundaryTime(segmentStart.y, delta.y, cell.y, step.y),
        firstBoundaryTime(segmentStart.z, delta.z, cell.z, step.z),
    };
    const io::Float3 timeDelta{
        step.x == 0 ? infinity : resolution / std::abs(delta.x),
        step.y == 0 ? infinity : resolution / std::abs(delta.y),
        step.z == 0 ? infinity : resolution / std::abs(delta.z),
    };

    float entryTime = 0.0F;
    for (std::uint32_t traversal = 0; traversal < maximumTraversalCells; ++traversal) {
        const float exitTime = std::min({1.0F, maximumTime.x, maximumTime.y, maximumTime.z});
        if (const auto vegetation = FindVegetationVoxel(cache, cell.x, cell.y, cell.z);
            vegetation.has_value()) {
            return {
                .position = Lerp(segmentStart, segmentEnd, entryTime),
                .normal = vegetation->normal,
                .role = WaterSurfaceRole::Vegetation,
                .segmentTime = entryTime,
                .hit = true,
            };
        }

        const auto surface = FindSurfaceCell(cache, cell.x, cell.y);
        if (surface.has_value() && delta.z < -1.0e-7F) {
            RainCollisionHit bestSurface;
            const auto testSurface = [&](float height, const io::Float3& normal, WaterSurfaceRole role) {
                if (!std::isfinite(height)) {
                    return;
                }
                const float hitTime = (height - segmentStart.z) / delta.z;
                if (hitTime < entryTime - epsilon || hitTime > exitTime + epsilon ||
                    hitTime < 0.0F || hitTime > 1.0F) {
                    return;
                }
                const auto hitPosition = Lerp(segmentStart, segmentEnd, hitTime);
                const auto hitCell = PositionCell2(hitPosition, resolution);
                if (hitCell.x != cell.x || hitCell.y != cell.y ||
                    (bestSurface.hit && hitTime >= bestSurface.segmentTime)) {
                    return;
                }
                bestSurface = {
                    .position = {hitPosition.x, hitPosition.y, height},
                    .normal = normal,
                    .role = role,
                    .segmentTime = hitTime,
                    .hit = true,
                };
            };
            testSurface(surface->rockHeight, surface->rockNormal, WaterSurfaceRole::Rock);
            testSurface(surface->sandHeight, surface->sandNormal, WaterSurfaceRole::Sand);
            if (bestSurface.hit) {
                return bestSurface;
            }
        }

        if (exitTime >= 1.0F - epsilon || cell == endCell) {
            break;
        }
        bool advanced = false;
        if (maximumTime.x <= exitTime + epsilon) {
            cell.x += step.x;
            maximumTime.x += timeDelta.x;
            advanced = true;
        }
        if (maximumTime.y <= exitTime + epsilon) {
            cell.y += step.y;
            maximumTime.y += timeDelta.y;
            advanced = true;
        }
        if (maximumTime.z <= exitTime + epsilon) {
            cell.z += step.z;
            maximumTime.z += timeDelta.z;
            advanced = true;
        }
        if (!advanced) {
            break;
        }
        entryTime = std::clamp(exitTime, 0.0F, 1.0F);
    }
    return {};
}

WaterSurfaceGpuData BuildWaterSurfaceGpuData(const WaterSurfaceCache& cache) {
    const bool requiresGroundTable =
        cache.schemaVersion >= kWaterSurfaceCacheSchemaVersion;
    if (cache.gpuData.sourceRevision == cache.revision &&
        !cache.gpuData.surfaceTable.empty() && !cache.gpuData.vegetationTable.empty() &&
        !cache.gpuData.flowSurfaceTable.empty() &&
        (!requiresGroundTable || !cache.gpuData.groundTable.empty())) {
        auto gpu = cache.gpuData;
        if (!gpu.payloadChecksum.Valid()) {
            gpu.payloadChecksum = CalculateWaterSurfacePayloadChecksumForSchema(
                cache,
                gpu,
                cache.schemaVersion);
        }
        if (!gpu.sourceIdentity.Valid() ||
            gpu.sourceIdentity.sourceSignature != cache.signature) {
            gpu.sourceIdentity = cache.schemaVersion == kWaterSurfaceCacheLegacySchemaVersion
                ? MakeLegacyWaterSurfaceCacheIdentity(cache, gpu)
                : MakeWaterSurfaceCacheIdentity(
                      cache,
                      gpu,
                      cache.schemaVersion);
        }
        return gpu;
    }
    WaterSurfaceGpuData gpu;
    gpu.sourceRevision = cache.revision;
    auto surfaceCapacity = RequiredHashCapacity(cache.surfaceCells.size());
    auto vegetationCapacity = RequiredHashCapacity(cache.vegetationVoxels.size());
    auto flowSurfaceCapacity = RequiredFlowHashCapacity(cache.flowSurfaceSurfels.size());
    auto groundCapacity = RequiredFlowHashCapacity(cache.groundCells.size());
    for (;;) {
        gpu.surfaceTable.assign(surfaceCapacity, {});
        bool success = true;
        std::uint32_t maximumProbe = 0U;
        for (const auto& cell : cache.surfaceCells) {
            const auto base = HashCell(cell.cellX, cell.cellY) & (surfaceCapacity - 1U);
            bool inserted = false;
            for (std::uint32_t probe = 0; probe < kMaximumHashProbeCount; ++probe) {
                auto& slot = gpu.surfaceTable[(base + probe) & (surfaceCapacity - 1U)];
                if (slot.cellX == std::numeric_limits<std::int32_t>::min()) {
                    slot = {
                        .cellX = cell.cellX,
                        .cellY = cell.cellY,
                        .rockHeight = cell.rockHeight,
                        .sandHeight = cell.sandHeight,
                        .packedRockNormal = PackNormal(cell.rockNormal),
                        .packedSandNormal = PackNormal(cell.sandNormal),
                        .rockConfidence = cell.rockConfidence,
                        .sandConfidence = cell.sandConfidence,
                    };
                    maximumProbe = std::max(maximumProbe, probe + 1U);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                success = false;
                break;
            }
        }
        if (success) {
            gpu.surfaceMask = surfaceCapacity - 1U;
            gpu.maximumProbeCount = maximumProbe;
            break;
        }
        surfaceCapacity *= 2U;
    }
    for (;;) {
        gpu.vegetationTable.assign(vegetationCapacity, {});
        bool success = true;
        std::uint32_t maximumProbe = 0U;
        for (const auto& cell : cache.vegetationVoxels) {
            const auto base = HashCell(cell.cellX, cell.cellY, cell.cellZ) & (vegetationCapacity - 1U);
            bool inserted = false;
            for (std::uint32_t probe = 0; probe < kMaximumHashProbeCount; ++probe) {
                auto& slot = gpu.vegetationTable[(base + probe) & (vegetationCapacity - 1U)];
                if (slot.cellX == std::numeric_limits<std::int32_t>::min()) {
                    slot = {
                        .cellX = cell.cellX,
                        .cellY = cell.cellY,
                        .cellZ = cell.cellZ,
                        .packedNormal = PackNormal(cell.normal),
                    };
                    maximumProbe = std::max(maximumProbe, probe + 1U);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                success = false;
                break;
            }
        }
        if (success) {
            gpu.vegetationMask = vegetationCapacity - 1U;
            gpu.maximumProbeCount = std::max(gpu.maximumProbeCount, maximumProbe);
            break;
        }
        vegetationCapacity *= 2U;
    }
    for (;;) {
        gpu.flowSurfaceTable.assign(flowSurfaceCapacity, {});
        bool success = true;
        std::uint32_t maximumProbe = 0U;
        for (const auto& surfel : cache.flowSurfaceSurfels) {
            const auto role = static_cast<std::uint32_t>(surfel.role);
            const io::Float3 centroidWithinCell{
                surfel.centroid.x / cache.resolutionMeters - static_cast<float>(surfel.cellX),
                surfel.centroid.y / cache.resolutionMeters - static_cast<float>(surfel.cellY),
                surfel.centroid.z / cache.resolutionMeters - static_cast<float>(surfel.cellZ),
            };
            const auto base = HashBits(
                                  HashCell(surfel.cellX, surfel.cellY, surfel.cellZ) ^
                                  HashBits(role)) &
                              (flowSurfaceCapacity - 1U);
            bool inserted = false;
            for (std::uint32_t probe = 0; probe < kMaximumHashProbeCount; ++probe) {
                auto& slot = gpu.flowSurfaceTable[(base + probe) & (flowSurfaceCapacity - 1U)];
                if (slot.cellX == std::numeric_limits<std::int32_t>::min()) {
                    slot = {
                        .cellX = surfel.cellX,
                        .cellY = surfel.cellY,
                        .cellZ = surfel.cellZ,
                        .roleAndSampleCount =
                            role | (std::min(surfel.sampleCount, 0x00FFFFFFU) << 8U),
                        .packedCentroid = PackUnit3(centroidWithinCell),
                        .packedNormal = PackNormal(surfel.normal),
                        .packedConfidenceCoherence =
                            PackUnorm2x16(surfel.confidence, surfel.normalCoherence),
                        .packedRoughnessVariance =
                            PackHalf2x16(surfel.roughness, surfel.normalVariance),
                    };
                    maximumProbe = std::max(maximumProbe, probe + 1U);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                success = false;
                break;
            }
        }
        if (success) {
            gpu.flowSurfaceMask = flowSurfaceCapacity - 1U;
            gpu.flowMaximumProbeCount = maximumProbe;
            gpu.maximumProbeCount = std::max(gpu.maximumProbeCount, maximumProbe);
            break;
        }
        flowSurfaceCapacity *= 2U;
    }
    for (;;) {
        gpu.groundTable.assign(groundCapacity, {});
        bool success = true;
        std::uint32_t maximumProbe = 0U;
        for (const auto& cell : cache.groundCells) {
            const auto base =
                HashCell(cell.cellX, cell.cellY) & (groundCapacity - 1U);
            bool inserted = false;
            for (std::uint32_t probe = 0U;
                 probe < kMaximumHashProbeCount;
                 ++probe) {
                auto& slot =
                    gpu.groundTable[(base + probe) & (groundCapacity - 1U)];
                if (slot.cellX == std::numeric_limits<std::int32_t>::min()) {
                    slot = {
                        .cellX = cell.cellX,
                        .cellY = cell.cellY,
                        .height = cell.height,
                        .packedNormal = PackNormal(cell.normal),
                        .packedDownhill = PackNormal(cell.downhill),
                        .packedConvergenceConfidence =
                            PackUnorm2x16(cell.convergence, cell.confidence),
                        .flagsAndSampleCount =
                            (cell.flags & 0xFFU) |
                            (std::min(cell.sampleCount, 0x00FFFFFFU) << 8U),
                        .componentAndConnectivity =
                            (cell.connectivityMask & 0xFFU) |
                            (std::min(cell.componentId, 0x00FFFFFFU) << 8U),
                    };
                    maximumProbe = std::max(maximumProbe, probe + 1U);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                success = false;
                break;
            }
        }
        if (success) {
            gpu.groundMask = groundCapacity - 1U;
            gpu.groundMaximumProbeCount = maximumProbe;
            gpu.maximumProbeCount =
                std::max(gpu.maximumProbeCount, maximumProbe);
            break;
        }
        groundCapacity *= 2U;
    }
    gpu.payloadChecksum = CalculateWaterSurfacePayloadChecksum(cache, gpu);
    gpu.sourceIdentity = MakeWaterSurfaceCacheIdentity(cache, gpu);
    return gpu;
}

std::filesystem::path WaterSurfaceCachePath(
    const std::filesystem::path& cacheRoot,
    std::string_view signature) {
    return cacheRoot / ".invisible_places" / "cache" / "water" /
           (std::string{signature} + ".surfacecache");
}

std::filesystem::path WaterSurfaceSceneCacheDirectory(
    const std::filesystem::path& sceneDirectory) {
    return sceneDirectory / ".invisible_places" / "cache" / "water";
}

std::filesystem::path WaterSurfaceSceneCachePath(
    const std::filesystem::path& sceneDirectory,
    std::string_view signature) {
    return WaterSurfaceSceneCacheDirectory(sceneDirectory) /
           (std::string{signature} + ".surfacecache");
}

std::uint64_t WaterSurfaceCacheEstimatedPersistenceBytes(
    const WaterSurfaceCache& cache) {
    const bool hasGpuTables =
        cache.gpuData.sourceRevision == cache.revision &&
        !cache.gpuData.surfaceTable.empty() &&
        !cache.gpuData.vegetationTable.empty() &&
        !cache.gpuData.flowSurfaceTable.empty() &&
        !cache.gpuData.groundTable.empty();
    const auto surfaceCount = hasGpuTables
        ? static_cast<std::uint64_t>(cache.gpuData.surfaceTable.size())
        : static_cast<std::uint64_t>(RequiredHashCapacity(cache.surfaceCells.size()));
    const auto vegetationCount = hasGpuTables
        ? static_cast<std::uint64_t>(cache.gpuData.vegetationTable.size())
        : static_cast<std::uint64_t>(RequiredHashCapacity(cache.vegetationVoxels.size()));
    const auto flowSurfaceCount = hasGpuTables
        ? static_cast<std::uint64_t>(cache.gpuData.flowSurfaceTable.size())
        : static_cast<std::uint64_t>(RequiredFlowHashCapacity(cache.flowSurfaceSurfels.size()));
    const auto groundCount = hasGpuTables
        ? static_cast<std::uint64_t>(cache.gpuData.groundTable.size())
        : static_cast<std::uint64_t>(
              RequiredFlowHashCapacity(cache.groundCells.size()));
    return WaterSurfaceCachePersistenceBytes(
        cache,
        surfaceCount,
        vegetationCount,
        flowSurfaceCount,
        groundCount);
}

bool WaterSurfaceCacheFitsPersistenceLimit(
    const WaterSurfaceCache& cache,
    std::uint64_t maximumBytes) {
    return WaterSurfaceCachePersistenceSizeAllowed(
        WaterSurfaceCacheEstimatedPersistenceBytes(cache),
        maximumBytes);
}

bool WaterSurfaceCachePersistenceSizeAllowed(
    std::uintmax_t byteCount,
    std::uint64_t maximumBytes) {
    return byteCount <= static_cast<std::uintmax_t>(maximumBytes);
}

WaterSurfaceCacheIdentity BuildWaterSurfaceCacheIdentity(
    const WaterSurfaceCache& cache) {
    if (cache.cacheIdentity.Valid() &&
        cache.cacheIdentity.sourceSignature == cache.signature &&
        cache.gpuData.sourceRevision == cache.revision &&
        cache.gpuData.sourceIdentity == cache.cacheIdentity) {
        return cache.cacheIdentity;
    }
    if (cache.gpuData.sourceRevision == cache.revision &&
        !cache.gpuData.surfaceTable.empty() &&
        !cache.gpuData.vegetationTable.empty() &&
        !cache.gpuData.flowSurfaceTable.empty() &&
        (cache.schemaVersion < kWaterSurfaceCacheSchemaVersion ||
         !cache.gpuData.groundTable.empty())) {
        if (cache.schemaVersion == kWaterSurfaceCacheLegacySchemaVersion) {
            return MakeLegacyWaterSurfaceCacheIdentity(cache, cache.gpuData);
        }
        auto checksum = cache.gpuData.payloadChecksum;
        if (!checksum.Valid()) {
            checksum = CalculateWaterSurfacePayloadChecksumForSchema(
                cache,
                cache.gpuData,
                cache.schemaVersion);
        }
        return MakeWaterSurfaceCacheIdentity(
            cache,
            cache.gpuData,
            cache.schemaVersion,
            &checksum);
    }
    const auto gpuData = BuildWaterSurfaceGpuData(cache);
    return gpuData.sourceIdentity;
}

WaterSurfaceQueryResult QueryWaterSurfaceCache(
    const WaterSurfaceCache& cache,
    const io::Float3& position,
    float maximumDistanceMeters,
    const io::Float3& referenceNormal,
    std::uint32_t roleMask) {
    WaterSurfaceQueryResult result;
    if (cache.flowSurfaceSurfels.empty() || !std::isfinite(maximumDistanceMeters) ||
        maximumDistanceMeters <= 0.0F || cache.resolutionMeters <= 0.0F) {
        return result;
    }

    const float resolution = cache.resolutionMeters;
    const auto minimumCell = PositionCell3(
        Subtract(position, {maximumDistanceMeters, maximumDistanceMeters, maximumDistanceMeters}),
        resolution);
    const auto maximumCell = PositionCell3(
        Add(position, {maximumDistanceMeters, maximumDistanceMeters, maximumDistanceMeters}),
        resolution);
    const float referenceLength = Length(referenceNormal);
    const bool hasReference = std::isfinite(referenceLength) && referenceLength > 1.0e-6F;
    const auto orientedReference =
        hasReference ? Scale(referenceNormal, 1.0F / referenceLength) : io::Float3{};
    const float maximumDistanceSquared = maximumDistanceMeters * maximumDistanceMeters;
    const float resolutionSquared = resolution * resolution;
    constexpr std::array<WaterSurfaceRole, 2> roles{
        WaterSurfaceRole::Rock,
        WaterSurfaceRole::Sand,
    };

    for (std::int32_t cellZ = minimumCell.z; cellZ <= maximumCell.z; ++cellZ) {
        for (std::int32_t cellY = minimumCell.y; cellY <= maximumCell.y; ++cellY) {
            for (std::int32_t cellX = minimumCell.x; cellX <= maximumCell.x; ++cellX) {
                for (const auto role : roles) {
                    const auto roleBit = 1U << static_cast<std::uint32_t>(role);
                    if ((roleMask & roleBit) == 0U) {
                        continue;
                    }
                    const auto* candidate =
                        FindFlowSurfaceSurfel(cache, cellX, cellY, cellZ, role);
                    if (candidate == nullptr) {
                        continue;
                    }
                    const auto delta = Subtract(candidate->centroid, position);
                    const float distanceSquared = Dot(delta, delta);
                    if (!std::isfinite(distanceSquared) ||
                        distanceSquared > maximumDistanceSquared) {
                        continue;
                    }

                    auto candidateNormal = Normalize(candidate->normal);
                    float planeResidual = 0.0F;
                    float normalDisagreement = 0.0F;
                    if (hasReference) {
                        float agreement = Dot(candidateNormal, orientedReference);
                        if (agreement < 0.0F) {
                            candidateNormal = Scale(candidateNormal, -1.0F);
                            agreement = -agreement;
                        }
                        planeResidual = std::abs(Dot(delta, orientedReference));
                        normalDisagreement = 1.0F - std::clamp(agreement, 0.0F, 1.0F);
                    }
                    const float confidencePenalty =
                        (1.0F - std::clamp(candidate->confidence, 0.0F, 1.0F)) *
                        resolutionSquared * 0.25F;
                    const float score = distanceSquared + planeResidual * planeResidual * 3.0F +
                                        normalDisagreement * resolutionSquared * 2.0F +
                                        confidencePenalty;
                    if (result.hit && score >= result.score) {
                        continue;
                    }
                    result.surfel = *candidate;
                    result.surfel.normal = candidateNormal;
                    result.distanceMeters = std::sqrt(distanceSquared);
                    result.score = score;
                    result.hit = true;
                }
            }
        }
    }
    return result;
}

WaterGroundQueryResult QueryWaterGroundCache(
    const WaterSurfaceCache& cache,
    const io::Float3& position,
    float maximumDistanceMeters) {
    WaterGroundQueryResult result;
    if (cache.groundCells.empty() ||
        !std::isfinite(maximumDistanceMeters) ||
        maximumDistanceMeters <= 0.0F ||
        !std::isfinite(cache.resolutionMeters) ||
        cache.resolutionMeters <= 0.0F) {
        return result;
    }

    const auto minimumCell = PositionCell2(
        Subtract(
            position,
            {maximumDistanceMeters, maximumDistanceMeters, 0.0F}),
        cache.resolutionMeters);
    const auto maximumCell = PositionCell2(
        Add(
            position,
            {maximumDistanceMeters, maximumDistanceMeters, 0.0F}),
        cache.resolutionMeters);
    const float maximumDistanceSquared =
        maximumDistanceMeters * maximumDistanceMeters;
    for (std::int32_t cellX = minimumCell.x;
         cellX <= maximumCell.x;
         ++cellX) {
        const auto first = std::lower_bound(
            cache.groundCells.begin(),
            cache.groundCells.end(),
            std::pair{cellX, minimumCell.y},
            [](const WaterGroundCell& cell,
               const std::pair<std::int32_t, std::int32_t>& coordinate) {
                return std::tie(cell.cellX, cell.cellY) <
                       std::tie(coordinate.first, coordinate.second);
            });
        for (auto candidate = first;
             candidate != cache.groundCells.end() &&
             candidate->cellX == cellX &&
             candidate->cellY <= maximumCell.y;
             ++candidate) {
            const io::Float3 candidatePosition{
                (static_cast<float>(candidate->cellX) + 0.5F) *
                    cache.resolutionMeters,
                (static_cast<float>(candidate->cellY) + 0.5F) *
                    cache.resolutionMeters,
                candidate->height,
            };
            const auto delta = Subtract(candidatePosition, position);
            const float distanceSquared = Dot(delta, delta);
            if (!std::isfinite(distanceSquared) ||
                distanceSquared > maximumDistanceSquared ||
                (result.hit &&
                 distanceSquared >=
                     result.distanceMeters * result.distanceMeters)) {
                continue;
            }
            result.cell = *candidate;
            result.distanceMeters = std::sqrt(distanceSquared);
            result.hit = true;
        }
    }
    return result;
}

RainSimulator::RainSimulator(std::uint32_t particleCapacity)
    : particles_(std::max(1U, particleCapacity)), events_(kRainImpactEventCapacity) {}

void RainSimulator::Reset(std::uint32_t seed) {
    seed_ = seed;
    eventWriteIndex_ = 0U;
    previousTimeSeconds_ = -std::numeric_limits<float>::infinity();
    std::fill(particles_.begin(), particles_.end(), RainParticle{});
    std::fill(events_.begin(), events_.end(), RainImpactEvent{});
}

void RainSimulator::SpawnParticle(std::uint32_t index, const RainSimulationFrame& frame) {
    auto& particle = particles_[index];
    ++particle.generation;
    const std::uint32_t base = frame.settings.seed ^ HashBits(index + particle.generation * 0x9E3779B9U);
    const float radius = std::max(0.1F, frame.settings.spawnRadiusMeters);
    const float angle = Random01(base + 1U) * 2.0F * kPi;
    const float radial = std::sqrt(Random01(base + 2U)) * radius;
    particle.position = {
        frame.spawnCentre.x + std::cos(angle) * radial,
        frame.spawnCentre.y + std::sin(angle) * radial,
        frame.spawnCentre.z + frame.settings.spawnHeightMeters + Random01(base + 3U) * 0.75F,
    };
    particle.previousPosition = particle.position;
    particle.randomState = base;
    particle.ageSeconds = 0.0F;
    particle.visibility = 1.0F;
    particle.surfaceNormal = {0.0F, 0.0F, 1.0F};
    particle.surfaceProximity = 0.0F;
    particle.active = Random01(base + 4U) <=
                      RainFrontSpawnProbability(frame.settings, particle.position, frame.timeSeconds);
}

void RainSimulator::EmitImpact(
    const RainCollisionHit& hit,
    const RainSimulationFrame& frame,
    const RainParticle& particle,
    RainSimulationDiagnostics* diagnostics) {
    const auto multipliers = RainIntensityValues(frame.settings.intensityPreset);
    const float dropWidth = frame.visual.widthMeters * frame.settings.dropletSizeScale * multipliers.width;
    const float impactSpeed = std::max(0.1F, Length(particle.velocity));
    float roleScale = 1.0F;
    float radius = 0.04F;
    float lifetime = 1.0F;
    bool enabled = true;
    switch (hit.role) {
        case WaterSurfaceRole::Sand:
            enabled = frame.settings.sandEffectsEnabled;
            roleScale = frame.settings.sandEffectScale;
            radius = std::clamp(0.020F + dropWidth * 6.0F + impactSpeed * 0.0015F, 0.025F, 0.11F);
            lifetime = 0.48F + std::min(0.35F, impactSpeed * 0.018F);
            ++diagnostics->sandEvents;
            break;
        case WaterSurfaceRole::Rock: {
            enabled = frame.settings.rockEffectsEnabled;
            roleScale = frame.settings.rockEffectScale;
            radius = std::clamp(0.028F + dropWidth * 10.0F + impactSpeed * 0.0020F, 0.035F, 0.16F);
            // Steep rock sheds water further: boost the event radius by
            // surface steepness so the Wetness footprint can run downhill.
            // The impact model reconstructs this boost from the event normal
            // (GPU mirror: EmitImpact in shaders/rain_simulation.comp).
            const auto unitNormal = Normalize(hit.normal);
            const float steepness =
                std::clamp(1.0F - std::abs(unitNormal.z), 0.0F, 1.0F);
            radius *= 1.0F + steepness * std::clamp(
                frame.settings.rockImpact.downhillStretch,
                0.0F,
                1.0F);
            lifetime = 3.8F + std::min(2.2F, impactSpeed * 0.12F);
            ++diagnostics->rockEvents;
            break;
        }
        case WaterSurfaceRole::Vegetation:
            enabled = frame.settings.vegetationEffectsEnabled;
            roleScale = frame.settings.vegetationEffectScale;
            radius = std::clamp(0.016F + dropWidth * 4.0F, 0.018F, 0.065F);
            lifetime = 1.1F + std::min(0.7F, impactSpeed * 0.045F);
            ++diagnostics->vegetationEvents;
            break;
        case WaterSurfaceRole::Ground:
        case WaterSurfaceRole::None:
            return;
    }
    if (!enabled) {
        return;
    }
    const float energy = std::clamp(
        (dropWidth / 0.003F) * (impactSpeed / 8.0F) * multipliers.effectEnergy * roleScale,
        0.05F,
        2.5F);
    events_[eventWriteIndex_ % events_.size()] = {
        .position = hit.position,
        .birthTimeSeconds = frame.timeSeconds,
        .normal = Normalize(hit.normal),
        .radiusMeters = radius,
        .role = hit.role,
        .lifetimeSeconds = lifetime,
        .energy = energy,
        .seed = particle.randomState,
    };
    eventWriteIndex_ = (eventWriteIndex_ + 1U) % static_cast<std::uint32_t>(events_.size());
    ++diagnostics->emittedEvents;
}

RainSimulationDiagnostics RainSimulator::Advance(
    const RainSimulationFrame& frame,
    const WaterSurfaceCache& surfaceCache) {
    RainSimulationDiagnostics diagnostics;
    if (frame.settings.seed != seed_ || frame.timeSeconds < previousTimeSeconds_) {
        Reset(frame.settings.seed);
    }
    previousTimeSeconds_ = frame.timeSeconds;
    if (!frame.settings.enabled) {
        return diagnostics;
    }

    const auto multipliers = RainIntensityValues(frame.settings.intensityPreset);
    const float amount = std::clamp(frame.settings.rainLevel, 0.0F, 1.0F);
    const float density = std::clamp(frame.settings.density * multipliers.density, 0.0F, 1.0F);
    const auto targetCount = std::min<std::uint32_t>(
        static_cast<std::uint32_t>(particles_.size()),
        static_cast<std::uint32_t>(std::lround(frame.settings.activeParticleCount * amount * density)));
    const float deltaSeconds = std::clamp(frame.deltaSeconds, 0.0F, 0.25F);
    const auto windDirection = Normalize(
        {frame.settings.windDirectionX, frame.settings.windDirectionY, 0.0F},
        {1.0F, 0.0F, 0.0F});

    for (std::uint32_t index = 0; index < particles_.size(); ++index) {
        auto& particle = particles_[index];
        if (index >= targetCount) {
            particle.active = false;
            continue;
        }
        if (!particle.active) {
            SpawnParticle(index, frame);
            ++diagnostics.respawnCount;
            if (!particle.active) {
                continue;
            }
        }

        const float frontWave = RainFrontWave(frame.settings, particle.position, frame.timeSeconds);
        const float front = 1.0F + (frontWave * 2.0F - 1.0F) *
                                       frame.settings.weatherFrontStrength * 0.35F;
        const float gustPhase =
            (particle.position.x * 0.73F + particle.position.y * 0.41F) /
                std::max(0.2F, frame.settings.gustScaleMeters) +
            frame.timeSeconds * frame.settings.gustSpeedMetersPerSecond /
                std::max(0.2F, frame.settings.gustScaleMeters);
        const float gustFrontScale = std::lerp(0.65F, 1.35F, frontWave);
        const float gust = 1.0F + std::sin(gustPhase * 2.0F * kPi + Random01(particle.randomState) * kPi) *
                                      frame.settings.gustStrength * gustFrontScale;
        const float turbulentX =
            std::sin(frame.timeSeconds * 2.7F + Random01(particle.randomState + 11U) * 17.0F) *
            frame.settings.turbulence * multipliers.windResponse;
        const float turbulentY =
            std::cos(frame.timeSeconds * 2.2F + Random01(particle.randomState + 19U) * 19.0F) *
            frame.settings.turbulence * multipliers.windResponse;
        const float windSpeed = frame.settings.windSpeedMetersPerSecond * multipliers.windResponse * gust;
        particle.velocity = {
            windDirection.x * windSpeed + turbulentX,
            windDirection.y * windSpeed + turbulentY,
            -frame.settings.fallSpeedMetersPerSecond * multipliers.speed * std::max(0.35F, front),
        };
        particle.previousPosition = particle.position;
        const float particleSpeed = Length(particle.velocity);
        const float stepDistance = particleSpeed * deltaSeconds;
        const float approachDistance = std::clamp(
            frame.settings.nearSurface.approachDistanceMeters,
            0.0F,
            1.0F);
        const auto travelDirection = Normalize(particle.velocity, {0.0F, 0.0F, -1.0F});
        const float probeDistance = std::max(stepDistance, approachDistance);
        const auto probeEnd = Add(
            particle.previousPosition,
            Scale(travelDirection, probeDistance));
        const auto hit = probeDistance > 1.0e-7F
            ? TraceRainCollision(surfaceCache, particle.previousPosition, probeEnd)
            : RainCollisionHit{};
        float speedFactor = 1.0F;
        float hitDistance = std::numeric_limits<float>::infinity();
        if (hit.hit) {
            hitDistance = std::clamp(hit.segmentTime, 0.0F, 1.0F) * probeDistance;
            particle.surfaceNormal = Normalize(hit.normal);
            particle.surfaceProximity = approachDistance > 1.0e-6F
                ? 1.0F - SmoothStep(0.0F, approachDistance, hitDistance)
                : 0.0F;
            speedFactor = std::lerp(
                1.0F,
                std::clamp(frame.settings.nearSurface.minimumSpeedFactor, 0.05F, 1.0F),
                particle.surfaceProximity);
        } else {
            particle.surfaceNormal = {0.0F, 0.0F, 1.0F};
            particle.surfaceProximity = 0.0F;
        }
        const float travelledDistance = stepDistance * speedFactor;
        particle.position = Add(
            particle.previousPosition,
            Scale(travelDirection, travelledDistance));
        particle.ageSeconds += deltaSeconds;
        particle.visibility = std::clamp(front, 0.25F, 1.5F);
        ++diagnostics.activeParticles;

        // The extended query replaces the old second, actual-step query. A
        // future hit remains cached on the particle until slowed travel reaches
        // it, preserving one bounded collision walk per particle and frame.
        if (hit.hit && hitDistance <= travelledDistance + 1.0e-6F) {
            particle.position = hit.position;
            ++diagnostics.collisionCount;
            if (frame.settings.impactEffectsEnabled) {
                EmitImpact(hit, frame, particle, &diagnostics);
            }
            SpawnParticle(index, frame);
            ++diagnostics.respawnCount;
            continue;
        }

        const auto cameraOffset = Subtract(particle.position, frame.cameraPosition);
        const float cameraDistanceSquared = Dot(cameraOffset, cameraOffset);
        const float deathDistance = std::max(1.0F, frame.settings.cameraDeathDistanceMeters);
        const bool outsideCache = surfaceCache.bounds.valid &&
            (particle.position.z < surfaceCache.bounds.minimum.z - 1.0F ||
             particle.position.x < surfaceCache.bounds.minimum.x - deathDistance ||
             particle.position.x > surfaceCache.bounds.maximum.x + deathDistance ||
             particle.position.y < surfaceCache.bounds.minimum.y - deathDistance ||
             particle.position.y > surfaceCache.bounds.maximum.y + deathDistance);
        if (outsideCache || cameraDistanceSquared > deathDistance * deathDistance) {
            ++diagnostics.escapedParticles;
            SpawnParticle(index, frame);
            ++diagnostics.respawnCount;
        }
    }
    return diagnostics;
}

RainImpactGrid BuildRainImpactGrid(
    std::span<const RainImpactEvent> events,
    const io::Float3& cameraPosition,
    float timeSeconds,
    float worldSpanMeters,
    const RainRockImpactSettings& rockImpact,
    const RainVegetationImpactSettings& vegetationImpact) {
    RainImpactGrid grid;
    grid.dimension = kRainImpactGridDimension;
    grid.cellSizeMeters = std::max(0.01F, worldSpanMeters / static_cast<float>(grid.dimension));
    grid.origin = {
        cameraPosition.x - worldSpanMeters * 0.5F,
        cameraPosition.y - worldSpanMeters * 0.5F,
        0.0F,
    };
    grid.cells.resize(static_cast<std::size_t>(grid.dimension) * grid.dimension);
    grid.events.assign(events.begin(), events.end());
    grid.rockImpact = rockImpact;
    grid.vegetationImpact = vegetationImpact;

    const auto rockPriorityKey = [&](const RainImpactEvent& event,
                                     std::uint32_t eventIndex,
                                     std::int32_t cellX,
                                     std::int32_t cellY) {
        const float cellMinimumX =
            grid.origin.x + static_cast<float>(cellX) * grid.cellSizeMeters;
        const float cellMinimumY =
            grid.origin.y + static_cast<float>(cellY) * grid.cellSizeMeters;
        const float cellMaximumX = cellMinimumX + grid.cellSizeMeters;
        const float cellMaximumY = cellMinimumY + grid.cellSizeMeters;
        const float distanceX = std::max(
            {cellMinimumX - event.position.x,
             0.0F,
             event.position.x - cellMaximumX});
        const float distanceY = std::max(
            {cellMinimumY - event.position.y,
             0.0F,
             event.position.y - cellMaximumY});
        // Compare physical distance, not distance relative to each event's
        // radius. Otherwise a farther large drop can displace a nearer small
        // drop even though both overlap this cell. Generated ROCK impacts are
        // capped at 160 mm and the steepness spawn boost can double that;
        // 500 mm covers the corner of the boosted square broad phase while
        // preserving sub-millimetre priority resolution (GPU mirror:
        // BinEvent in shaders/rain_simulation.comp).
        constexpr float kRockReservoirDistanceRangeMeters = 0.50F;
        const float normalizedDistance = std::clamp(
            std::hypot(distanceX, distanceY) /
                kRockReservoirDistanceRangeMeters,
            0.0F,
            1.0F);
        const float age = timeSeconds - event.birthTimeSeconds;
        // Age is likewise compared in seconds. Normalizing by each event's
        // lifetime could rank an older long-lived event ahead of a younger
        // short-lived one.
        constexpr float kRockReservoirAgeRangeSeconds = 6.0F;
        const float agePriority = std::clamp(
            age / kRockReservoirAgeRangeSeconds,
            0.0F,
            1.0F);
        const float inverseEnergy =
            1.0F - std::clamp(event.energy / 2.5F, 0.0F, 1.0F);

        // Preserve the requested ordering within one 16-bit priority:
        // distance to the cell AABB, then age, then inverse energy. The event
        // index in the low 16 bits is the final deterministic tie-breaker.
        const auto distanceKey = static_cast<std::uint32_t>(
            std::floor(normalizedDistance * 1'023.0F + 0.5F));
        const auto ageKey = static_cast<std::uint32_t>(
            std::floor(agePriority * 7.0F + 0.5F));
        const auto energyKey = static_cast<std::uint32_t>(
            std::floor(inverseEnergy * 7.0F + 0.5F));
        const auto priority = std::min(
            (distanceKey << 6U) | (ageKey << 3U) | energyKey,
            65'534U);
        return (priority << 16U) | (eventIndex & 0xFFFFU);
    };
    const auto insertFreeSlot = [&](auto& references,
                                    std::uint32_t* count,
                                    std::uint32_t* occupiedMask,
                                    std::uint32_t eventIndex) {
        if (*count >= references.size()) {
            ++grid.overflowCount;
            return false;
        }
        const auto lane = *count;
        references[lane] = eventIndex;
        *occupiedMask |= 1U << lane;
        ++(*count);
        return true;
    };
    const auto insertRock = [&](RainImpactGridCell* cell,
                                std::uint32_t eventIndex,
                                std::int32_t cellX,
                                std::int32_t cellY) {
        std::uint32_t carriedIndex = eventIndex;
        auto carriedKey = rockPriorityKey(
            grid.events[carriedIndex],
            carriedIndex,
            cellX,
            cellY);
        for (std::uint32_t lane = 0U; lane < cell->rockCount; ++lane) {
            const auto retainedIndex = cell->rock[lane];
            const auto retainedKey = rockPriorityKey(
                grid.events[retainedIndex],
                retainedIndex,
                cellX,
                cellY);
            if (carriedKey >= retainedKey) {
                continue;
            }
            std::swap(carriedIndex, cell->rock[lane]);
            carriedKey = retainedKey;
        }

        if (cell->rockCount < cell->rock.size()) {
            cell->rock[cell->rockCount] = carriedIndex;
            ++cell->rockCount;
            cell->rockMask =
                (1U << static_cast<std::uint32_t>(cell->rockCount)) - 1U;
            return;
        }

        // A full reservoir always discards exactly one event: either this
        // candidate or the lowest-priority event displaced by it.
        ++grid.overflowCount;
    };

    for (std::uint32_t eventIndex = 0; eventIndex < grid.events.size(); ++eventIndex) {
        const auto& event = grid.events[eventIndex];
        if (!EventActive(event, timeSeconds)) {
            continue;
        }
        const auto minX = static_cast<std::int32_t>(std::floor(
            (event.position.x - event.radiusMeters - grid.origin.x) / grid.cellSizeMeters));
        const auto maxX = static_cast<std::int32_t>(std::floor(
            (event.position.x + event.radiusMeters - grid.origin.x) / grid.cellSizeMeters));
        const auto minY = static_cast<std::int32_t>(std::floor(
            (event.position.y - event.radiusMeters - grid.origin.y) / grid.cellSizeMeters));
        const auto maxY = static_cast<std::int32_t>(std::floor(
            (event.position.y + event.radiusMeters - grid.origin.y) / grid.cellSizeMeters));
        for (std::int32_t y = std::max(0, minY); y <= std::min(static_cast<std::int32_t>(grid.dimension) - 1, maxY); ++y) {
            for (std::int32_t x = std::max(0, minX); x <= std::min(static_cast<std::int32_t>(grid.dimension) - 1, maxX); ++x) {
                auto& cell = grid.cells[static_cast<std::size_t>(y) * grid.dimension + static_cast<std::size_t>(x)];
                bool inserted = false;
                switch (event.role) {
                    case WaterSurfaceRole::Sand:
                        inserted = insertFreeSlot(
                            cell.sand,
                            &cell.sandCount,
                            &cell.sandMask,
                            eventIndex);
                        break;
                    case WaterSurfaceRole::Rock:
                        insertRock(
                            &cell,
                            eventIndex,
                            x,
                            y);
                        inserted = true;
                        break;
                    case WaterSurfaceRole::Vegetation:
                        inserted = insertFreeSlot(
                            cell.vegetation,
                            &cell.vegetationCount,
                            &cell.vegetationMask,
                            eventIndex);
                        break;
                    case WaterSurfaceRole::Ground:
                    case WaterSurfaceRole::None:
                        break;
                }
                (void)inserted;
            }
        }
    }
    return grid;
}

float EvaluateRockRainImpactValue(
    const RainImpactEvent& event,
    const io::Float3& point,
    const io::Float3& pointNormal,
    float timeSeconds,
    const RainRockImpactSettings& settings) {
    const float age = timeSeconds - event.birthTimeSeconds;
    if (event.role != WaterSurfaceRole::Rock || age < 0.0F ||
        age > event.lifetimeSeconds) {
        return 0.0F;
    }

    io::Float3 eventNormal;
    if (Dot(event.normal, event.normal) > 1.0e-8F) {
        eventNormal = Normalize(event.normal);
    } else if (Dot(pointNormal, pointNormal) > 1.0e-8F) {
        eventNormal = Normalize(pointNormal);
    } else {
        eventNormal = {0.0F, 0.0F, 1.0F};
    }

    const float safeLifetime = std::max(0.001F, event.lifetimeSeconds);
    const float life = std::clamp(age / safeLifetime, 0.0F, 1.0F);
    const float effectiveRadius = std::max(
        0.001F,
        event.radiusMeters * std::sqrt(2.0F / 3.0F));
    const float growthSeconds =
        std::clamp(event.lifetimeSeconds * 0.18F, 0.55F, 0.95F) *
        (3.20F / std::clamp(settings.spreadSpeed, 0.10F, 6.0F));
    const float growth = SmoothStep(0.0F, growthSeconds, age);

    const io::Float3 gravity{0.0F, 0.0F, -1.0F};
    const auto projectedGravity = Subtract(
        gravity,
        Scale(eventNormal, Dot(gravity, eventNormal)));
    const float projectedGravityLengthSquared =
        Dot(projectedGravity, projectedGravity);
    const auto downhill = projectedGravityLengthSquared > 1.0e-8F
        ? Scale(projectedGravity, 1.0F / std::sqrt(projectedGravityLengthSquared))
        : io::Float3{};
    const float driftStartLife = std::clamp(
        growthSeconds / safeLifetime,
        0.0F,
        0.9F);
    const float drift = driftStartLife < 0.9F
        ? SmoothStep(driftStartLife, 0.9F, life)
        : 0.0F;
    const auto impactCentre = Add(
        event.position,
        Scale(downhill, effectiveRadius * 0.2F * drift));

    const auto offset = Subtract(point, impactCentre);
    const float normalDistance = std::abs(Dot(offset, eventNormal));
    const auto tangentOffset = Subtract(
        offset,
        Scale(eventNormal, Dot(offset, eventNormal)));
    const auto basisReference = std::abs(eventNormal.z) < 0.90F
        ? io::Float3{0.0F, 0.0F, 1.0F}
        : io::Float3{1.0F, 0.0F, 0.0F};
    const auto tangentX = Normalize(Cross(basisReference, eventNormal), {1.0F, 0.0F, 0.0F});
    const auto tangentY = Normalize(Cross(eventNormal, tangentX), {0.0F, 1.0F, 0.0F});
    const float angle = std::atan2(Dot(tangentOffset, tangentY), Dot(tangentOffset, tangentX));
    const float breakupPhase = Random01(event.seed ^ 0xD1B54A35U) * 2.0F * kPi;
    const float breakupNoise = std::clamp(
        0.5F + 0.31F * std::sin(angle * 5.0F + breakupPhase) +
            0.19F * std::sin(angle * 9.0F - breakupPhase * 1.7F),
        0.0F,
        1.0F);
    // Break the edge only inward. The original event radius therefore remains
    // a conservative broad phase even at maximum configured breakup.
    const float irregularRadiusScale = 1.0F -
        0.35F * std::clamp(settings.edgeBreakup, 0.0F, 1.0F) * breakupNoise;
    // Steep impacts spawn with a boosted radius (see EmitImpact in both
    // simulators, same steepness formula). The footprint puts that extra
    // radius into the downhill run only: lateral and uphill extents
    // normalize back to the unboosted radius and tighten further with
    // stretch, so wetness reads as rivulets running down walls while flat
    // impacts keep their even spread. Every extent stays inside the boosted
    // event radius, preserving the broad-phase grid invariant.
    const float steepness = std::clamp(1.0F - std::abs(eventNormal.z), 0.0F, 1.0F);
    const float stretch =
        std::clamp(settings.downhillStretch, 0.0F, 2.0F) * steepness;
    const float spawnBoost =
        1.0F + steepness * std::clamp(settings.downhillStretch, 0.0F, 1.0F);
    const float downAmount = Dot(tangentOffset, downhill);
    const auto lateralOffset = Subtract(tangentOffset, Scale(downhill, downAmount));
    const float lateralFactor = spawnBoost * (1.0F + 0.25F * stretch);
    const float uphillFactor = spawnBoost * (1.0F + 0.80F * stretch);
    const float shapedDown = std::max(downAmount, 0.0F);
    const float shapedUp = std::max(-downAmount, 0.0F) * uphillFactor;
    const float shapedTangentSquared =
        shapedDown * shapedDown + shapedUp * shapedUp +
        Dot(lateralOffset, lateralOffset) * lateralFactor * lateralFactor;
    const float normalizedDistance = (
        std::sqrt(
            shapedTangentSquared +
            normalDistance * normalDistance * 4.0F) /
        effectiveRadius) / std::max(0.65F, irregularRadiusScale);
    // Retain the original broad-phase cells: once downhill drift begins, the
    // late feather plus the 20% centre travel still fits inside eventRadius.
    const float edgeWidth = 0.02F + (1.0F - growth) * 0.14F;
    const float lowerPointWeight = SmoothStep(
        -0.45F,
        0.45F,
        (event.position.z - point.z) / effectiveRadius);
    const float heightBias = std::clamp(settings.heightBias * (0.20F / 0.75F), 0.0F, 0.55F);
    const float heightGain = std::lerp(1.0F - heightBias, 1.0F + heightBias, lowerPointWeight);
    const float fadeDifference = std::clamp(heightBias * 0.75F, 0.0F, 0.30F);
    const float fadeStart = std::clamp(
        std::lerp(0.55F - fadeDifference, 0.55F + fadeDifference, lowerPointWeight) *
            std::clamp(settings.persistence / 1.35F, 0.10F, 2.25F),
        0.05F,
        0.95F);
    const float footprint = 1.0F - SmoothStep(
        std::max(0.0F, growth - edgeWidth),
        growth + edgeWidth,
        normalizedDistance);
    // Match the shared GLSL evaluator: a smooth radial rolloff reads as a wet
    // gradient rather than a uniformly coloured stamped region.
    const float radialFade =
        1.0F - SmoothStep(0.0F, 1.0F, normalizedDistance);
    const float centreFalloffAmount =
        std::sqrt(std::clamp(settings.centreFalloff, 0.0F, 1.0F));
    const float centreWeight = std::lerp(
        1.0F,
        radialFade,
        centreFalloffAmount);
    return footprint *
           (1.0F - SmoothStep(fadeStart, 1.0F, life)) *
           heightGain * centreWeight;
}

float EvaluateSandRainImpactValue(
    const RainImpactEvent& event,
    const io::Float3& point,
    float timeSeconds,
    float sandWaterMask) {
    const float age = timeSeconds - event.birthTimeSeconds;
    if (event.role != WaterSurfaceRole::Sand || age < 0.0F ||
        age > event.lifetimeSeconds) {
        return 0.0F;
    }

    const float safeLifetime = std::max(0.001F, event.lifetimeSeconds);
    const float life = std::clamp(age / safeLifetime, 0.0F, 1.0F);
    const float planarDistance =
        std::hypot(point.x - event.position.x, point.y - event.position.y);

    const float wetRadius = std::max(0.001F, event.radiusMeters);
    const float wetNormalizedDistance = planarDistance / wetRadius;
    const float ringRadius = wetRadius * (0.12F + 0.88F * life);
    const float ringThickness = std::max(0.003F, wetRadius * 0.14F);
    const float ringDistance = std::abs(planarDistance - ringRadius);
    const float wetRadialFade =
        1.0F - SmoothStep(0.0F, 1.0F, wetNormalizedDistance);
    const float wetValue =
        std::exp(
            -(ringDistance * ringDistance) /
            (ringThickness * ringThickness)) *
        (1.0F - SmoothStep(0.72F, 1.0F, life)) *
        std::sqrt(std::max(0.0F, wetRadialFade));

    const float dryRadius =
        std::clamp(event.radiusMeters * 0.22F, 0.005F, 0.025F);
    const float dryNormalizedDistance = planarDistance / dryRadius;
    const float dryGrowthSeconds = std::min(0.14F, safeLifetime * 0.28F);
    const float dryGrowth = SmoothStep(0.0F, dryGrowthSeconds, age);
    const float dryEdgeWidth = 0.10F + (1.0F - dryGrowth) * 0.18F;
    const float dryFootprint = 1.0F - SmoothStep(
        std::max(0.0F, dryGrowth - dryEdgeWidth),
        dryGrowth + dryEdgeWidth,
        dryNormalizedDistance);
    const float dryRadialFade =
        1.0F - SmoothStep(0.0F, 1.0F, dryNormalizedDistance);
    const float lowerPointWeight = SmoothStep(
        -0.35F,
        0.35F,
        (event.position.z - point.z) / dryRadius);
    const float dryHeightGain =
        std::lerp(0.78F, 1.16F, lowerPointWeight);
    const float dryValue =
        dryFootprint *
        dryRadialFade *
        dryHeightGain *
        (1.0F - SmoothStep(0.55F, 1.0F, life));

    return std::lerp(
        dryValue,
        wetValue,
        std::clamp(sandWaterMask, 0.0F, 1.0F));
}

RainImpactEffect EvaluateRainImpact(
    const RainImpactGrid& grid,
    std::uint32_t effectMask,
    const io::Float3& point,
    const io::Float3& normal,
    float timeSeconds,
    float sandWaterMask,
    const RainImpactHeightBand& ringsBand,
    const RainImpactHeightBand& wetnessBand,
    const RainImpactHeightBand& dropletsBand) {
    RainImpactEffect effect;
    if (effectMask == 0U || grid.cells.empty()) {
        return effect;
    }
    // Band weights first: a disabled or out-of-band model skips its entire
    // reference walk, matching ResolveRainImpactComposite's early-outs.
    const float ringsWeight =
        (effectMask & kRainImpactEffectRingsBit) != 0U
            ? RainImpactBandWeight(SanitizeRainImpactHeightBand(ringsBand), point.z)
            : 0.0F;
    const float wetnessWeight =
        (effectMask & kRainImpactEffectWetnessBit) != 0U
            ? RainImpactBandWeight(SanitizeRainImpactHeightBand(wetnessBand), point.z)
            : 0.0F;
    const float dropletsWeight =
        (effectMask & kRainImpactEffectDropletsBit) != 0U
            ? RainImpactBandWeight(SanitizeRainImpactHeightBand(dropletsBand), point.z)
            : 0.0F;
    if (ringsWeight <= 0.0F && wetnessWeight <= 0.0F && dropletsWeight <= 0.0F) {
        return effect;
    }
    const auto cellX = static_cast<std::int32_t>(std::floor((point.x - grid.origin.x) / grid.cellSizeMeters));
    const auto cellY = static_cast<std::int32_t>(std::floor((point.y - grid.origin.y) / grid.cellSizeMeters));
    if (cellX < 0 || cellY < 0 || cellX >= static_cast<std::int32_t>(grid.dimension) ||
        cellY >= static_cast<std::int32_t>(grid.dimension)) {
        return effect;
    }
    const auto& cell = grid.cells[static_cast<std::size_t>(cellY) * grid.dimension + static_cast<std::size_t>(cellX)];

    float ringsValue = 0.0F;
    if (ringsWeight > 0.0F && cell.sandMask != 0U) {
        for (std::uint32_t index = 0; index < cell.sand.size(); ++index) {
            if ((cell.sandMask & (1U << index)) == 0U) {
                continue;
            }
            const auto& event = grid.events[cell.sand[index]];
            if (event.role != WaterSurfaceRole::Sand ||
                !EventActive(event, timeSeconds)) {
                continue;
            }
            const float value =
                EvaluateSandRainImpactValue(event, point, timeSeconds, sandWaterMask) *
                event.energy * ringsWeight;
            ringsValue = std::max(ringsValue, value);
        }
    }
    float rockPeak = 0.0F;
    float rockRemaining = 1.0F;
    if (wetnessWeight > 0.0F && cell.rockMask != 0U) {
        for (std::uint32_t index = 0; index < cell.rock.size(); ++index) {
            if ((cell.rockMask & (1U << index)) == 0U) {
                continue;
            }
            const auto& event = grid.events[cell.rock[index]];
            if (event.role != WaterSurfaceRole::Rock ||
                !EventActive(event, timeSeconds)) {
                continue;
            }
            const float value =
                EvaluateRockRainImpactValue(event, point, normal, timeSeconds, grid.rockImpact) *
                event.energy * wetnessWeight;
            // Peak-preserving soft union: one impact remains bit-for-bit
            // equivalent to its narrow-phase value, while every additional
            // retained impact can only strengthen the wet response.
            rockPeak = std::max(rockPeak, value);
            rockRemaining *= 1.0F - std::clamp(value, 0.0F, 1.0F);
        }
    }
    const float wetnessValue = std::max(rockPeak, 1.0F - rockRemaining);
    float dropletsValue = 0.0F;
    if (dropletsWeight > 0.0F && cell.vegetationMask != 0U) {
        for (std::uint32_t index = 0; index < cell.vegetation.size(); ++index) {
            if ((cell.vegetationMask & (1U << index)) == 0U) {
                continue;
            }
            const auto& event = grid.events[cell.vegetation[index]];
            if (event.role != WaterSurfaceRole::Vegetation ||
                !EventActive(event, timeSeconds)) {
                continue;
            }
            const float age = timeSeconds - event.birthTimeSeconds;
            const float value =
                EvaluateVegetationSprinkle(event, point, normal, age, grid.vegetationImpact) *
                event.energy * dropletsWeight;
            dropletsValue = std::max(dropletsValue, value);
        }
    }
    // Cross-effect composition, identical to ResolveRainImpactComposite in
    // shaders/pointcloud_rain_impact.glsl: the three models coexist. Adds
    // sum, size multipliers multiply, and both tint weights sum per target
    // colour (consumers clamp the tint application at 0.72), so a point
    // inside overlapping bands shows Rings AND Wetness AND Droplets. A model
    // whose value is zero contributes exactly nothing, keeping every
    // single-effect response identical to its standalone evaluation.
    effect.opacity =
        ringsValue * 0.18F + wetnessValue * 0.18F + dropletsValue * 0.14F;
    effect.emission =
        ringsValue * 0.11F + wetnessValue * 0.11F + dropletsValue * 0.48F;
    effect.sizeScale =
        (1.0F + ringsValue * 0.16F) *
        (1.0F + wetnessValue * 0.16F) *
        (1.0F + dropletsValue * 0.12F);
    effect.colourBlend = ringsValue * 0.20F + wetnessValue * 0.42F;
    effect.dropletBlend = dropletsValue * 0.18F;
    return effect;
}

}  // namespace invisible_places::water
