#pragma once

#include "io/PointCloudData.hpp"
#include "io/TransformMatrix.hpp"
#include "scene/PointCloudVariants.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace invisible_places::water {

inline constexpr float kWaterSurfaceResolutionMeters = 0.010F;
inline constexpr std::uint32_t kWaterSurfaceNormalSourceSpacingMicrometres = 2'000U;
inline constexpr std::uint32_t kWaterGroundSourceSpacingMicrometres = 5'000U;
inline constexpr std::string_view kWaterSurfaceCacheAlgorithmId =
    "water-surface-10mm-normal-average-ground-v2";
inline constexpr std::uint32_t kWaterSurfaceCacheSchemaVersion = 4U;
inline constexpr std::uint32_t kWaterSurfaceCachePreviousSchemaVersion = 3U;
inline constexpr std::uint32_t kWaterSurfaceCacheLegacySchemaVersion = 2U;
inline constexpr std::uint64_t kWaterSurfaceCacheMaximumPersistenceBytes =
    5ULL * 1024ULL * 1024ULL * 1024ULL;

enum class WaterSurfaceRole : std::uint32_t {
    None = 0U,
    Rock = 1U,
    Sand = 2U,
    Vegetation = 3U,
    Ground = 4U,
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
inline constexpr std::uint32_t kWaterSurfaceGroundRoleMask =
    1U << static_cast<std::uint32_t>(WaterSurfaceRole::Ground);

inline constexpr std::uint32_t kWaterGroundTerminalContactFlag = 1U << 0U;
inline constexpr std::uint32_t kWaterGroundUpperFlag = 1U << 1U;
inline constexpr std::uint32_t kWaterGroundVegetationSupportedFlag = 1U << 2U;

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

// Highest occupied sampled-Ground cell for one 10 mm XY coordinate. Ground is
// kept separate from authored ROCK/SAND/VEG classification: it supplies a
// connected flow surface without becoming Rain or Seepage terrain.
struct WaterGroundCell {
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    float height = 0.0F;
    io::Float3 normal{0.0F, 0.0F, 1.0F};
    io::Float3 downhill{};
    float convergence = 0.0F;
    float confidence = 0.0F;
    std::uint32_t sampleCount = 0U;
    std::uint32_t flags = 0U;
    std::uint32_t componentId = 0U;
    std::uint32_t connectivityMask = 0U;
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

// Mesh Flow consumes this fixed 32-byte hash-table ABI. The final word packs
// the 8-connected neighbour mask in its low byte and a 24-bit component id in
// the remaining bits.
struct alignas(16) WaterGpuGroundSlot {
    std::int32_t cellX = std::numeric_limits<std::int32_t>::min();
    std::int32_t cellY = std::numeric_limits<std::int32_t>::min();
    float height = 0.0F;
    std::uint32_t packedNormal = 0U;
    std::uint32_t packedDownhill = 0U;
    std::uint32_t packedConvergenceConfidence = 0U;
    std::uint32_t flagsAndSampleCount = 0U;
    std::uint32_t componentAndConnectivity = 0U;
};

static_assert(sizeof(WaterGpuGroundSlot) == 32U);

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

// A chunk-boundary-independent checksum used while schema-3/4 GPU table bytes
// move directly from the sidecar into the mapped staging allocation. Keeping
// this tiny builder shared by the loader and renderer lets the renderer reject
// a sidecar that changed after validation without retaining a second table
// copy or performing another disk pass.
class WaterSurfaceGpuStreamChecksumBuilder {
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
            .hashedByteCount = hashedByteCount_,
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

// A schema-3/4 warm load keeps the queryable CPU aggregates resident but leaves
// the already-built GPU tables in the validated sidecar. The renderer reads
// these contiguous sections directly through its reusable 64 MiB staging
// allocation, avoiding another cache-sized CPU table allocation.
struct WaterSurfacePersistedGpuTables {
    std::filesystem::path filePath;
    std::uint64_t fileSize = 0U;
    std::int64_t modificationTicks = 0;
    std::uint64_t surfaceOffset = 0U;
    std::uint64_t vegetationOffset = 0U;
    std::uint64_t flowSurfaceOffset = 0U;
    std::uint64_t groundOffset = 0U;
    std::uint64_t surfaceCount = 0U;
    std::uint64_t vegetationCount = 0U;
    std::uint64_t flowSurfaceCount = 0U;
    std::uint64_t groundCount = 0U;
    WaterSurfaceCachePayloadChecksum streamChecksum;

    [[nodiscard]] bool Valid() const {
        return !filePath.empty() && fileSize > 0U && surfaceCount > 0U &&
               vegetationCount > 0U && flowSurfaceCount > 0U &&
               streamChecksum.Valid();
    }

    [[nodiscard]] bool GroundValid() const {
        return Valid() && groundOffset > flowSurfaceOffset && groundCount > 0U;
    }
};

struct WaterSurfaceGpuData {
    std::vector<RainGpuSurfaceSlot> surfaceTable;
    std::vector<RainGpuVegetationSlot> vegetationTable;
    std::vector<WaterGpuSurfaceSurfelSlot> flowSurfaceTable;
    std::vector<WaterGpuGroundSlot> groundTable;
    std::uint32_t surfaceMask = 0U;
    std::uint32_t vegetationMask = 0U;
    std::uint32_t flowSurfaceMask = 0U;
    std::uint32_t groundMask = 0U;
    std::uint32_t maximumProbeCount = 0U;
    std::uint32_t flowMaximumProbeCount = 0U;
    std::uint32_t groundMaximumProbeCount = 0U;
    std::uint64_t sourceRevision = 0U;
    WaterSurfaceCacheIdentity sourceIdentity;
    WaterSurfaceCachePayloadChecksum payloadChecksum;
    WaterSurfacePersistedGpuTables persistedTables;
};

struct WaterSurfaceCache {
    std::uint32_t schemaVersion = kWaterSurfaceCacheSchemaVersion;
    float resolutionMeters = kWaterSurfaceResolutionMeters;
    std::string signature;
    std::vector<WaterSurfaceSourceMetadata> sources;
    std::vector<RainSurfaceCell> surfaceCells;
    std::vector<RainVegetationVoxel> vegetationVoxels;
    std::vector<WaterSurfaceSurfel> flowSurfaceSurfels;
    std::vector<WaterGroundCell> groundCells;
    WaterSurfaceGpuData gpuData;
    io::Bounds3f bounds;
    std::uint64_t sourcePointCount = 0U;
    std::uint64_t groundSourcePointCount = 0U;
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

struct WaterGroundQueryResult {
    WaterGroundCell cell{};
    float distanceMeters = std::numeric_limits<float>::infinity();
    bool hit = false;
};

[[nodiscard]] std::vector<WaterSurfaceSource> SelectWaterSurfaceSources(
    const scene::ScenePointCloudGroup& group,
    std::uint32_t preferredSpacingMicrometres =
        kWaterSurfaceNormalSourceSpacingMicrometres);
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
[[nodiscard]] WaterGroundQueryResult QueryWaterGroundCache(
    const WaterSurfaceCache& cache,
    const io::Float3& position,
    float maximumDistanceMeters);

}  // namespace invisible_places::water
