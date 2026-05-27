#pragma once

#include "io/PointCloudData.hpp"
#include "renderer/pointcloud/PointCloudLodHierarchy.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace invisible_places::renderer::pointcloud {

inline constexpr std::uint32_t kPointCloudIpcloudCacheFormatVersion = 2U;
inline constexpr std::uint32_t kPointCloudIpcloudAttributeSchemaVersion = 1U;
inline constexpr std::uint32_t kPointCloudIpcloudBuildSettingsVersion = 1U;
inline constexpr std::uint32_t kPointCloudIpcloudRawChunkFormatVersion = 1U;
inline constexpr std::uint32_t kPointCloudIpcloudDefaultPreviewPointCount = 65'536U;
inline constexpr std::uint64_t kPointCloudIpcloudDefaultCpuResidencyBytes = 512ULL * 1024ULL * 1024ULL;

enum class PointCloudIpcloudCacheState : std::uint32_t {
    Missing,
    Hit,
    Stale,
    Partial,
    Corrupt,
};

struct PointCloudIpcloudSourceInfo {
    std::filesystem::path sourcePath;
    std::uint64_t normalizedPathHash = 0;
    std::uint64_t sourceSizeBytes = 0;
    std::int64_t sourceWriteTimeNs = 0;
    std::uint64_t headerHash = 0;
    std::uint64_t sampledContentHash = 0;
    std::uint64_t pointCount = 0;
    std::uint32_t scalarFieldCount = 0;
    bool hasRgb = false;
    bool hasNormals = false;
    std::uint64_t dataOffsetBytes = 0;
    std::uint32_t recordSizeBytes = 0;
};

struct PointCloudIpcloudBuildStatus {
    bool parseHeaderComplete = false;
    bool upperHierarchyComplete = false;
    bool nodePagesComplete = false;
    bool representativeRangesComplete = false;
    bool rawChunkRangesComplete = false;
    bool scalarStatsComplete = false;
    bool complete = false;
    bool failed = false;
    bool interrupted = false;
    std::uint64_t rawChunksCompleted = 0;
    std::uint64_t rawChunkCount = 0;
    std::string phase;
    std::string message;
};

struct PointCloudIpcloudInspection {
    PointCloudIpcloudCacheState state = PointCloudIpcloudCacheState::Missing;
    std::filesystem::path bundlePath;
    std::filesystem::path temporaryBundlePath;
    PointCloudIpcloudBuildStatus buildStatus{};
    std::string status;
    std::string reason;
    std::uint64_t previewPointCount = 0;
    std::uint64_t representedSourceCount = 0;
};

struct PointCloudIpcloudPreview {
    bool loaded = false;
    bool fromPersistentBundle = false;
    invisible_places::io::LoadedPointCloud cloud{};
    PointCloudLodHierarchy hierarchy{};
    std::vector<std::uint32_t> sourcePointIndices;
    std::uint64_t representedSourceCount = 0;
    double loadMilliseconds = 0.0;
    std::string status;
    std::string reason;
};

struct PointCloudIpcloudRawChunkInfo {
    std::uint32_t chunkIndex = 0;
    invisible_places::io::Bounds3f bounds{};
    std::uint32_t pointCount = 0;
    std::uint32_t scalarFieldCount = 0;
    bool hasRgb = false;
    bool hasNormals = false;
    std::uint64_t encodedBytes = 0;
    std::uint64_t decodedCpuBytes = 0;
};

struct PointCloudIpcloudRawChunk {
    PointCloudIpcloudRawChunkInfo info{};
    std::vector<std::uint32_t> sourcePointIndices;
    std::vector<invisible_places::io::Float3> positions;
    std::vector<invisible_places::io::Float3> normals;
    std::vector<std::uint32_t> packedColors;
    std::vector<float> scalarFieldValues;
};

struct PointCloudIpcloudNodeRawChunkRange {
    std::uint32_t firstChunkIndex = 0;
    std::uint32_t chunkCount = 0;
};

struct PointCloudIpcloudRawChunkCatalog {
    std::vector<PointCloudIpcloudRawChunkInfo> chunks;
    std::vector<PointCloudIpcloudNodeRawChunkRange> nodeRanges;
    std::vector<std::uint32_t> nodeChunkIndices;
    std::uint64_t totalEncodedBytes = 0;
    std::uint64_t totalDecodedCpuBytes = 0;
};

struct PointCloudIpcloudResidencyDiagnostics {
    std::uint32_t visibleChunkRequestCount = 0;
    std::uint32_t residentChunkCount = 0;
    std::uint32_t missingChunkCount = 0;
    std::uint64_t cpuResidentBytes = 0;
    std::uint64_t gpuResidentBytes = 0;
    std::uint64_t uploadBytesThisFrame = 0;
    std::uint64_t uploadBudgetBytes = 0;
    std::uint32_t uploadQueueLength = 0;
    float chunkHitRate = 0.0F;
    std::uint32_t evictionCount = 0;
    std::string evictionReason;
    std::string fallbackReason;
};

struct PointCloudIpcloudResidentSet {
    bool loaded = false;
    invisible_places::io::LoadedPointCloud cloud{};
    std::vector<std::uint32_t> sourcePointIndices;
    std::vector<PointCloudDrawItemGpu> remappedDrawItems;
    PointCloudIpcloudResidencyDiagnostics diagnostics{};
};

struct PointCloudIpcloudSaveResult {
    bool saved = false;
    std::filesystem::path bundlePath;
    std::string status;
    std::string errorMessage;
};

struct PointCloudIpcloudResumeResult {
    bool resumable = false;
    bool restartRequired = false;
    PointCloudIpcloudBuildStatus buildStatus{};
    std::string status;
    std::string reason;
};

[[nodiscard]] const char* PointCloudIpcloudCacheStateName(PointCloudIpcloudCacheState state);

std::optional<PointCloudIpcloudSourceInfo> MakePointCloudIpcloudSourceInfo(
    const std::filesystem::path& sourcePath,
    std::string* errorMessage = nullptr);

std::filesystem::path BuildPointCloudIpcloudBundlePath(
    const std::filesystem::path& cacheDirectory,
    const PointCloudIpcloudSourceInfo& sourceInfo);

std::filesystem::path BuildPointCloudIpcloudBundlePath(
    const std::filesystem::path& cacheDirectory,
    const std::filesystem::path& sourcePath);

PointCloudIpcloudInspection InspectPointCloudIpcloudBundle(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig = {});

PointCloudIpcloudPreview LoadPointCloudIpcloudPreview(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig = {});

PointCloudIpcloudPreview LoadPointCloudIpcloudSourcePreview(
    const std::filesystem::path& sourcePath,
    std::uint32_t targetRepresentativeCount = kPointCloudIpcloudDefaultPreviewPointCount);

PointCloudLodCacheLoadResult LoadPointCloudIpcloudHierarchy(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig = {});

PointCloudIpcloudRawChunkCatalog LoadPointCloudIpcloudRawChunkCatalog(
    const std::filesystem::path& bundlePath);

PointCloudIpcloudRawChunk LoadPointCloudIpcloudRawChunk(
    const std::filesystem::path& bundlePath,
    std::uint32_t chunkIndex,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields);

PointCloudIpcloudResidentSet BuildPointCloudIpcloudResidentSet(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudRawChunkCatalog& catalog,
    const std::vector<std::uint32_t>& frontierNodeIndices,
    const std::vector<PointCloudDrawItemGpu>& drawItems,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    std::uint64_t uploadBudgetBytes,
    std::uint64_t cpuResidencyBudgetBytes = kPointCloudIpcloudDefaultCpuResidencyBytes);

PointCloudIpcloudSaveResult SavePointCloudIpcloudBundle(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig,
    const invisible_places::io::LoadedPointCloud& cloud,
    const PointCloudLodHierarchy& hierarchy);

PointCloudIpcloudResumeResult ResumePointCloudIpcloudBuild(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig = {});

}  // namespace invisible_places::renderer::pointcloud
