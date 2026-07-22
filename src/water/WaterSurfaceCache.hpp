#pragma once

#include "io/PointCloudData.hpp"
#include "io/TransformMatrix.hpp"
#include "scene/PointCloudVariants.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::water {

inline constexpr float kWaterSurfaceResolutionMeters = 0.020F;
inline constexpr std::uint32_t kWaterSurfaceCacheSchemaVersion = 3U;
inline constexpr std::uint32_t kWaterSurfaceCacheLegacySchemaVersion = 2U;
inline constexpr std::uint64_t kWaterSurfaceCacheMaximumPersistenceBytes =
    5ULL * 1024ULL * 1024ULL * 1024ULL;

enum class WaterSurfaceRole : std::uint32_t {
    None = 0U,
    Rock = 1U,
    Sand = 2U,
    Vegetation = 3U,
};

enum class WaterSurfaceCacheRuntimeStatus : std::uint32_t {
    Missing = 0U,
    Valid = 1U,
    Stale = 2U,
    Building = 3U,
    Failed = 4U,
};

inline constexpr std::uint32_t kWaterSurfaceRockRoleMask =
    1U << static_cast<std::uint32_t>(WaterSurfaceRole::Rock);
inline constexpr std::uint32_t kWaterSurfaceSandRoleMask =
    1U << static_cast<std::uint32_t>(WaterSurfaceRole::Sand);
inline constexpr std::uint32_t kWaterSurfaceDefaultRoleMask =
    kWaterSurfaceRockRoleMask | kWaterSurfaceSandRoleMask;

struct WaterSurfaceSource {
    std::filesystem::path sourcePath;
    WaterSurfaceRole role = WaterSurfaceRole::None;
    io::Matrix4d localToWorld{};
    std::uint32_t spacingMicrometres = 0U;
    bool hasTransform = false;
    bool isFallback = false;
};

struct WaterSurfaceSourceMetadata {
    std::filesystem::path sourcePath;
    WaterSurfaceRole role = WaterSurfaceRole::None;
    std::uint32_t spacingMicrometres = 0U;
    std::uint64_t fileSize = 0U;
    std::int64_t modificationTicks = 0;
    bool isFallback = false;
};

struct WaterSurfaceSample {
    io::Float3 position{};
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    WaterSurfaceRole role = WaterSurfaceRole::None;
    float roughness = 0.0F;
    bool hasRoughness = false;
};

// Rain uses the top-most ROCK/SAND heights, while Flow and Seepage consume the
// orientation-independent surfels stored alongside them.
struct RainSurfaceCell {
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    float rockHeight = -std::numeric_limits<float>::infinity();
    float sandHeight = -std::numeric_limits<float>::infinity();
    io::Float3 rockNormal{0.0F, 0.0F, 1.0F};
    io::Float3 sandNormal{0.0F, 0.0F, 1.0F};
    float rockConfidence = 0.0F;
    float sandConfidence = 0.0F;
    std::uint32_t rockSampleCount = 0U;
    std::uint32_t sandSampleCount = 0U;
};

struct RainVegetationVoxel {
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    std::int32_t cellZ = 0;
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    std::uint32_t sampleCount = 0U;
};

struct WaterSurfaceSurfel {
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    std::int32_t cellZ = 0;
    WaterSurfaceRole role = WaterSurfaceRole::None;
    io::Float3 centroid{};
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    float confidence = 0.0F;
    float normalCoherence = 0.0F;
    float roughness = 0.0F;
    float normalVariance = 0.0F;
    std::uint32_t sampleCount = 0U;
};

struct RainGpuSurfaceSlot {
    std::int32_t cellX = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellY = std::numeric_limits<std::int32_t>::min();
    float rockHeight = -std::numeric_limits<float>::infinity();
    float sandHeight = -std::numeric_limits<float>::infinity();
    std::uint32_t packedRockNormal = 0U;
    std::uint32_t packedSandNormal = 0U;
    float rockConfidence = 0.0F;
    float sandConfidence = 0.0F;
};

static_assert(sizeof(RainGpuSurfaceSlot) == 32U);

struct RainGpuVegetationSlot {
    std::int32_t cellX = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellY = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellZ = std::numeric_limits<std::int32_t>::min();
    std::uint32_t packedNormal = 0U;
};

static_assert(sizeof(RainGpuVegetationSlot) == 16U);

struct alignas(16) WaterGpuSurfaceSurfelSlot {
    std::int32_t cellX = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellY = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellZ = std::numeric_limits<std::int32_t>::min();
    std::uint32_t roleAndSampleCount = 0U;
    std::uint32_t packedCentroid = 0U;
    std::uint32_t packedNormal = 0U;
    std::uint32_t packedConfidenceCoherence = 0U;
    std::uint32_t packedRoughnessVariance = 0U;
};

static_assert(sizeof(WaterGpuSurfaceSurfelSlot) == 32U);

struct WaterSurfaceCacheIdentity {
    std::string sourceSignature;
    std::array<std::uint64_t, 4> contentDigest{};

    [[nodiscard]] bool Valid() const {
        return std::any_of(
            contentDigest.begin(),
            contentDigest.end(),
            [](std::uint64_t word) { return word != 0U; });
    }

    bool operator==(const WaterSurfaceCacheIdentity&) const = default;
};

struct WaterSurfaceCachePayloadChecksum {
    std::array<std::uint64_t, 2> words{};
    std::uint64_t hashedByteCount = 0U;

    [[nodiscard]] bool Valid() const {
        return hashedByteCount != 0U &&
               std::any_of(words.begin(), words.end(), [](std::uint64_t word) {
                   return word != 0U;
               });
    }

    bool operator==(const WaterSurfaceCachePayloadChecksum&) const = default;
};

struct WaterSurfaceGpuData {
    std::vector<RainGpuSurfaceSlot> surfaceTable;
    std::vector<RainGpuVegetationSlot> vegetationTable;
    std::vector<WaterGpuSurfaceSurfelSlot> flowSurfaceTable;
    std::uint32_t surfaceMask = 0U;
    std::uint32_t vegetationMask = 0U;
    std::uint32_t flowSurfaceMask = 0U;
    std::uint32_t maximumProbeCount = 0U;
    std::uint32_t flowMaximumProbeCount = 0U;
    std::uint64_t sourceRevision = 0U;
    WaterSurfaceCacheIdentity sourceIdentity;
    WaterSurfaceCachePayloadChecksum payloadChecksum;
};

struct WaterSurfaceCache {
    std::uint32_t schemaVersion = kWaterSurfaceCacheSchemaVersion;
    float resolutionMeters = kWaterSurfaceResolutionMeters;
    std::string signature;
    std::vector<WaterSurfaceSourceMetadata> sources;
    std::vector<RainSurfaceCell> surfaceCells;
    std::vector<RainVegetationVoxel> vegetationVoxels;
    std::vector<WaterSurfaceSurfel> flowSurfaceSurfels;
    WaterSurfaceGpuData gpuData;
    io::Bounds3f bounds;
    std::uint64_t sourcePointCount = 0U;
    double buildMilliseconds = 0.0;
    std::uint64_t revision = 0U;
    WaterSurfaceCacheIdentity cacheIdentity;
};

struct WaterSurfaceBuildDiagnostics {
    std::uint32_t sourceScanCount = 0U;
    std::uint32_t gpuTableBuildCount = 0U;
    std::uint32_t fullPayloadHashPassCount = 0U;
};

struct WaterSurfaceBuildResult {
    WaterSurfaceCache cache;
    WaterSurfaceBuildDiagnostics diagnostics;
    std::vector<std::string> warnings;
    std::string errorMessage;
    std::filesystem::path persistedPath;
    bool loadedFromDisk = false;
    bool success = false;
    bool cancelled = false;
};

struct WaterSurfaceQueryResult {
    WaterSurfaceSurfel surfel{};
    float distanceMeters = std::numeric_limits<float>::infinity();
    float score = std::numeric_limits<float>::infinity();
    bool hit = false;
};

[[nodiscard]] std::vector<WaterSurfaceSource> SelectWaterSurfaceSources(
    const scene::ScenePointCloudGroup& group,
    std::uint32_t preferredSpacingMicrometres = 5'000U);
[[nodiscard]] std::string WaterSurfaceCacheSignature(
    std::span<const WaterSurfaceSource> sources,
    float resolutionMeters = kWaterSurfaceResolutionMeters);
[[nodiscard]] std::filesystem::path WaterSurfaceCachePath(
    const std::filesystem::path& cacheRoot,
    std::string_view signature);
[[nodiscard]] std::filesystem::path WaterSurfaceSceneCacheDirectory(
    const std::filesystem::path& sceneDirectory);
[[nodiscard]] std::filesystem::path WaterSurfaceSceneCachePath(
    const std::filesystem::path& sceneDirectory,
    std::string_view signature);
[[nodiscard]] std::uint64_t WaterSurfaceCacheEstimatedPersistenceBytes(
    const WaterSurfaceCache& cache);
[[nodiscard]] bool WaterSurfaceCacheFitsPersistenceLimit(
    const WaterSurfaceCache& cache,
    std::uint64_t maximumBytes = kWaterSurfaceCacheMaximumPersistenceBytes);
[[nodiscard]] bool WaterSurfaceCachePersistenceSizeAllowed(
    std::uintmax_t byteCount,
    std::uint64_t maximumBytes = kWaterSurfaceCacheMaximumPersistenceBytes);
[[nodiscard]] WaterSurfaceBuildResult BuildWaterSurfaceCache(
    std::span<const WaterSurfaceSource> sources,
    const std::filesystem::path& cacheRoot = {},
    const std::atomic_bool* cancelRequested = nullptr);
[[nodiscard]] WaterSurfaceCache BuildWaterSurfaceCacheFromSamples(
    std::span<const WaterSurfaceSample> samples,
    float resolutionMeters = kWaterSurfaceResolutionMeters);
[[nodiscard]] bool SaveWaterSurfaceCache(
    const WaterSurfaceCache& cache,
    const std::filesystem::path& filePath,
    std::string* errorMessage = nullptr);
[[nodiscard]] bool LoadWaterSurfaceCache(
    const std::filesystem::path& filePath,
    std::string_view expectedSignature,
    WaterSurfaceCache* cache,
    std::string* errorMessage = nullptr,
    WaterSurfaceBuildDiagnostics* diagnostics = nullptr);
[[nodiscard]] WaterSurfaceGpuData BuildWaterSurfaceGpuData(
    const WaterSurfaceCache& cache);
[[nodiscard]] WaterSurfaceCacheIdentity BuildWaterSurfaceCacheIdentity(
    const WaterSurfaceCache& cache);
[[nodiscard]] WaterSurfaceQueryResult QueryWaterSurfaceCache(
    const WaterSurfaceCache& cache,
    const io::Float3& position,
    float maximumDistanceMeters,
    const io::Float3& referenceNormal = {},
    std::uint32_t roleMask = kWaterSurfaceDefaultRoleMask);

}  // namespace invisible_places::water
