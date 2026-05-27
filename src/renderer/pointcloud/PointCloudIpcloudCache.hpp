#pragma once

#include "io/PointCloudData.hpp"
#include "renderer/pointcloud/PointCloudLodHierarchy.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace invisible_places::renderer::pointcloud {

inline constexpr std::uint32_t kPointCloudIpcloudCacheFormatVersion = 1U;
inline constexpr std::uint32_t kPointCloudIpcloudAttributeSchemaVersion = 1U;
inline constexpr std::uint32_t kPointCloudIpcloudBuildSettingsVersion = 1U;
inline constexpr std::uint32_t kPointCloudIpcloudDefaultPreviewPointCount = 65'536U;

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
