#pragma once

#include "io/PointCloudData.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::io {

inline constexpr std::uint32_t kAdaptiveHqCacheSchemaVersion = 2U;
inline constexpr std::string_view kAdaptiveHqCacheAlgorithmId =
    "adaptive-hq-morton-column-blocks-v2";
inline constexpr std::uint64_t kAdaptiveHqCacheTargetBlockBytes =
    4U * 1024U * 1024U;
inline constexpr std::uint64_t kAdaptiveHqCacheMinimumBlockBytes =
    1U * 1024U * 1024U;
inline constexpr std::uint64_t kAdaptiveHqCacheMaximumBlockBytes =
    8U * 1024U * 1024U;
// Disk blocks stay large enough for efficient sequential reads. Once a block
// is decoded, this smaller logical subdivision lets a camera request accept
// contiguous Morton ranges by bounds instead of testing every point. The
// subdivision is session-only, so existing schema-v2 Site3 caches remain
// valid and do not need to be rebuilt.
inline constexpr std::uint32_t kAdaptiveHqMicroBlockPointCount = 128U;

struct AdaptiveHqSourceIdentity {
    std::filesystem::path path;
    std::uintmax_t byteSize = 0U;
    std::int64_t writeTimeTicks = 0;
    std::string schemaFingerprint;
    std::string contentFingerprint;
    std::uint64_t pointCount = 0U;
    std::uint32_t recordSize = 0U;

    [[nodiscard]] bool operator==(
        const AdaptiveHqSourceIdentity&) const = default;
};

struct AdaptiveHqSourceIdentityResult {
    AdaptiveHqSourceIdentity identity;
    std::string errorMessage;
    bool success = false;
};

// Size/timestamp are complemented by an ordered property-schema digest and a
// bounded content fingerprint sampled across the file. The latter detects a
// same-size source replacement without hashing several GiB on every 500 ms
// aHQ resolve.
[[nodiscard]] AdaptiveHqSourceIdentityResult
InspectAdaptiveHqSource(
    const std::filesystem::path& sourcePath);

struct AdaptiveHqCacheBlock {
    std::uint32_t blockIndex = 0U;
    std::uint64_t firstPoint = 0U;
    std::uint64_t pointCount = 0U;
    std::uint64_t fileOffsetBytes = 0U;
    std::uint64_t byteSize = 0U;
    // Scalar values are stored field-major inside the same spatial block:
    // field 0's pointCount float32 values, then field 1, and so on. This
    // makes one selected field a single compact seek without touching any
    // unused property bytes.
    std::uint64_t scalarFileOffsetBytes = 0U;
    std::uint64_t scalarByteSize = 0U;
    Bounds3f bounds{};
};

struct AdaptiveHqCacheScalarField {
    std::string name;
    std::uint32_t sourceIndex = 0U;
    ScalarFieldStats stats;
};

struct AdaptiveHqCacheIndex {
    std::filesystem::path cacheRoot;
    std::filesystem::path indexPath;
    // Raw block-columnar geometry: positions, packed RGBA, optional normals,
    // then retained source ids for each block. `dataPath` is deliberately
    // retained as the public name used by schema-v1 callers and diagnostics.
    std::filesystem::path dataPath;
    std::filesystem::path scalarDataPath;
    std::string cacheFingerprint;
    AdaptiveHqSourceIdentity source;
    std::uint64_t dataOffsetBytes = 0U;
    std::uint64_t dataByteSize = 0U;
    std::uint64_t scalarDataByteSize = 0U;
    // Runtime geometry bytes per point (not an interleaved on-disk stride;
    // each block is structure-of-arrays).
    std::uint32_t cachedRecordSize = 0U;
    bool hasSourceRgb = false;
    bool hasNormals = false;
    std::vector<AdaptiveHqCacheScalarField> scalarFields;
    std::vector<AdaptiveHqCacheBlock> blocks;
};

using AdaptiveHqCacheProgress = std::function<void(float)>;

struct AdaptiveHqCacheOpenResult {
    AdaptiveHqCacheIndex index;
    std::string errorMessage;
    bool success = false;
    bool cancelled = false;
    bool built = false;
};

// Reuses a validated local cache or builds it transactionally. `cacheRoot`
// is supplied by the app as Saved/.invisible_places/cache/adaptive_hq; no
// source-adjacent or shared/OneDrive path is ever considered.
[[nodiscard]] AdaptiveHqCacheOpenResult OpenOrBuildAdaptiveHqCache(
    const std::filesystem::path& cacheRoot,
    const AdaptiveHqSourceIdentity& expectedSource,
    std::stop_token stopToken = {},
    const AdaptiveHqCacheProgress& progress = {});

using AdaptiveHqBoundsPredicate =
    std::function<bool(const Bounds3f&)>;

[[nodiscard]] std::vector<std::uint32_t>
SelectAdaptiveHqCacheBlocks(
    const AdaptiveHqCacheIndex& index,
    const AdaptiveHqBoundsPredicate& includeBounds);

[[nodiscard]] std::vector<PointCloudSourceRange>
AdaptiveHqCacheBlockRanges(
    const AdaptiveHqCacheIndex& index,
    std::span<const std::uint32_t> blockIndices);

struct AdaptiveHqResidentBlock {
    std::uint32_t blockIndex = 0U;
    std::shared_ptr<const PointCloudSubsetLoadResult> points;
    struct MicroBlock {
        std::uint32_t firstPoint = 0U;
        std::uint32_t pointCount = 0U;
        Bounds3f bounds{};
    };
    std::shared_ptr<const std::vector<MicroBlock>> microBlocks;
    // Monotonic request serial used by the app's small LRU fringe cache.
    std::uint64_t lastUsedSerial = 0U;
    // Squared distance from the latest request camera to the parent cache
    // block. Navigation retention uses this before age; animation/scrubbing
    // uses age before distance so a repeatedly visited path remains warm.
    float requestDistanceSquared = std::numeric_limits<float>::infinity();
};

[[nodiscard]] std::vector<AdaptiveHqResidentBlock::MicroBlock>
BuildAdaptiveHqMicroBlocks(
    std::span<const Float3> positions,
    std::uint32_t targetPointCount = kAdaptiveHqMicroBlockPointCount);

[[nodiscard]] std::uint64_t AdaptiveHqFieldFilterFingerprint(
    const PointCloudScalarFieldFilter& filter);

// Reads one complete 1--8 MiB geometry block plus only the selected scalar
// columns. No point predicate or decimation is applied here so the decoded
// result can be reused by overlapping camera guards.
[[nodiscard]] PointCloudSubsetLoadResult LoadAdaptiveHqCacheBlock(
    const AdaptiveHqCacheIndex& index,
    std::uint32_t blockIndex,
    const PointCloudScalarFieldFilter& fieldFilter,
    std::stop_token stopToken = {},
    const PointCloudLoadProgress& progress = {});

// Assembles active resident blocks, then applies the same deterministic grid
// decimation as fixed HQ. When micro-block bounds are supplied, intersecting
// contiguous Morton ranges are accepted conservatively and the expensive
// per-point frustum predicate is skipped; without them the exact predicate is
// retained as a compatibility fallback. Restoring original source order is
// available for callers that need monotonic source indices; live aHQ keeps
// stable Morton order and avoids an O(n log n) sort of millions of points.
// Every selected disk block must be resident.
[[nodiscard]] PointCloudSubsetLoadResult AssembleAdaptiveHqCacheSubset(
    const AdaptiveHqCacheIndex& index,
    std::span<const std::uint32_t> activeBlockIndices,
    std::span<const AdaptiveHqResidentBlock> residentBlocks,
    const PointCloudSubsetPredicate& includePoint,
    const PointCloudGridDecimation& gridDecimation = {},
    std::stop_token stopToken = {},
    bool restoreSourceOrder = true,
    const AdaptiveHqBoundsPredicate& includeMicroBlockBounds = {});

}  // namespace invisible_places::io
