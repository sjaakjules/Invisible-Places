#pragma once

#include "io/PointCloudData.hpp"
#include "output/RenderPreset.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <stop_token>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace invisible_places::renderer::pointcloud {

inline constexpr std::uint32_t kPointCloudLodRepresentativeClassCount = 8U;
inline constexpr std::uint32_t kPointCloudLodInvalidScalarFieldSlot = 0xffffffffU;

enum PointCloudLodRepresentativeClass : std::uint32_t {
    PointCloudLodRepresentativeClassSpatialCoverage = 1U << 0U,
    PointCloudLodRepresentativeClassColorContrast = 1U << 1U,
    PointCloudLodRepresentativeClassNormalEdge = 1U << 2U,
    PointCloudLodRepresentativeClassScalarMin = 1U << 3U,
    PointCloudLodRepresentativeClassScalarMax = 1U << 4U,
    PointCloudLodRepresentativeClassScalarThreshold = 1U << 5U,
    PointCloudLodRepresentativeClassEmissiveAccent = 1U << 6U,
    PointCloudLodRepresentativeClassBlueNoiseFill = 1U << 7U,
};

enum class PointCloudLodRendererCostProfile : std::uint32_t {
    FastBasicSquare,
    BeautyScreenSprite,
    BeautyWorldScreenSprite,
    BeautyWorldSurfel
};

struct PointCloudLodScalarStats {
    float minimum = 0.0F;
    float maximum = 0.0F;
    float mean = 0.0F;
    float variance = 0.0F;
    std::uint32_t count = 0;
    std::uint32_t flags = 0;
};

struct PointCloudLodRepresentative {
    std::uint32_t sourcePointIndex = 0;
    std::uint32_t representedSourceCount = 0;
    std::uint32_t representativeClassFlags = PointCloudLodRepresentativeClassSpatialCoverage;
    std::uint32_t scalarFieldSlot = kPointCloudLodInvalidScalarFieldSlot;
    invisible_places::io::Float3 position{};
    float sourceSpacingMeters = 0.0F;
    float importance = 0.0F;
    float lodRank = 0.0F;
};

struct PointCloudLodNode {
    invisible_places::io::Bounds3f bounds{};
    std::array<std::uint32_t, 8> childIndices{};
    std::uint32_t childCount = 0;
    std::uint32_t firstRepresentative = 0;
    std::uint32_t representativeCount = 0;
    std::uint32_t representedSourceCount = 0;
    std::uint32_t depth = 0;
    float spacingMeters = 0.0F;
    float densityPointsPerM3 = 0.0F;
    float colorVariance = 0.0F;
    float colorContrast = 0.0F;
    float normalVariance = 0.0F;
    float scalarVarianceHint = 0.0F;
    float scalarRangeHint = 0.0F;
    float emissiveImportanceHint = 0.0F;
    std::uint32_t featureFlags = 0;
    std::uint32_t reserved0 = 0;

    [[nodiscard]] bool IsLeaf() const { return childCount == 0; }
};

struct alignas(16) PointCloudDrawItemGpu {
    std::uint32_t sourcePointIndex = 0;
    std::uint32_t representedSourceCount = 0;
    std::uint32_t reserved0 = 0;
    std::uint32_t reserved1 = 0;
    float footprintAreaPixels = 1.0F;
    float opacityCompensation = 1.0F;
    float emissionCompensation = 1.0F;
    float renderAreaPixels = 1.0F;
};

struct PointCloudLodHierarchy {
    std::vector<PointCloudLodNode> nodes;
    std::vector<PointCloudLodRepresentative> representatives;
    std::vector<PointCloudLodScalarStats> scalarFieldStats;
    std::vector<PointCloudLodScalarStats> nodeScalarStats;
    std::uint32_t sourcePointCount = 0;
    std::uint32_t scalarFieldCount = 0;

    [[nodiscard]] bool Empty() const { return nodes.empty() || sourcePointCount == 0; }
};

struct PointCloudLodBuildConfig {
    std::uint32_t maxLeafSourcePoints = 2048;
    std::uint32_t maxDepth = 12;
    std::uint32_t maxInternalRepresentatives = 32;
};

struct PointCloudLodBuildProgress {
    std::uint64_t sourcePointCount = 0;
    std::uint64_t processedSourceReferences = 0;
    std::uint64_t estimatedTotalSourceReferences = 0;
    std::uint64_t nodesBuilt = 0;
    std::uint64_t representativesBuilt = 0;
    std::uint32_t currentDepth = 0;
    std::uint32_t maxDepthReached = 0;
    bool finished = false;
};

using PointCloudLodBuildProgressCallback = std::function<void(const PointCloudLodBuildProgress&)>;

struct PointCloudLodTraversalParams {
    glm::mat4 viewProjection{1.0F};
    glm::vec3 cameraPosition{0.0F, 0.0F, 0.0F};
    std::uint32_t viewportWidth = 1;
    std::uint32_t viewportHeight = 1;
    PointCloudStyleState style{};
    invisible_places::output::PointCloudExportDensityMode densityMode =
        invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality;
    PointCloudLodRendererCostProfile rendererCostProfile = PointCloudLodRendererCostProfile::FastBasicSquare;
    std::uint32_t maxDrawItems = 0;
    std::uint32_t maxRepresentatives = 0;
    float maxEstimatedFragments = 0.0F;
    float maxEstimatedBlendedFragments = 0.0F;
    float maxRepresentativeDiameterPixels = 0.0F;
    float targetProjectedSpacingPixels = 0.0F;
    std::vector<std::uint32_t> previousFrontierNodeIndices;
    float hysteresisPromoteScale = 1.12F;
    float hysteresisDemoteScale = 0.78F;
};

struct PointCloudLodTraversalDiagnostics {
    std::uint32_t visibleFrontierNodeCount = 0;
    std::vector<std::uint32_t> frontierNodeIndices;
    std::uint32_t previousFrontierNodeCount = 0;
    std::uint32_t promotedNodeCount = 0;
    std::uint32_t demotedNodeCount = 0;
    std::uint32_t hysteresisKeptNodeCount = 0;
    std::int64_t representativeDelta = 0;
    float hysteresisPromoteScale = 1.0F;
    float hysteresisDemoteScale = 1.0F;
    std::uint64_t visibleFrontierRepresentedSourceCount = 0;
    std::uint32_t emittedRepresentativeCount = 0;
    std::uint64_t emittedRepresentedSourceCount = 0;
    std::uint64_t culledRepresentedSourceCount = 0;
    std::array<std::uint32_t, kPointCloudLodRepresentativeClassCount> emittedClassCounts{};
    std::uint32_t colorFeatureRefinedNodeCount = 0;
    std::uint32_t scalarFeatureRefinedNodeCount = 0;
    std::uint32_t normalFeatureRefinedNodeCount = 0;
    std::uint32_t emissiveFeatureRefinedNodeCount = 0;
    bool representativeBudgetReached = false;
    bool fragmentBudgetReached = false;
    bool blendedFragmentBudgetReached = false;
    PointCloudLodRendererCostProfile rendererCostProfile = PointCloudLodRendererCostProfile::FastBasicSquare;
    float minRadiusScale = 1.0F;
    float maxRadiusScale = 1.0F;
    float minOpacityCoverageScale = 1.0F;
    float maxOpacityCoverageScale = 1.0F;
    float minEmissionCoverageScale = 1.0F;
    float maxEmissionCoverageScale = 1.0F;
    float estimatedVertexCost = 0.0F;
    float estimatedFragmentCost = 0.0F;
    float estimatedBlendedFragmentCost = 0.0F;
    bool opacityCompensationClamped = false;
    bool emissionCompensationClamped = false;
    bool performanceCompensationClamped = false;
};

struct PointCloudLodCacheSource {
    std::filesystem::path sourcePath;
    std::uint64_t normalizedPathHash = 0;
    std::uint64_t sourceSizeBytes = 0;
    std::int64_t sourceWriteTimeNs = 0;
    std::uint32_t pointCount = 0;
    invisible_places::io::Bounds3f bounds{};
};

struct PointCloudLodCacheLoadResult {
    bool loaded = false;
    bool stale = false;
    PointCloudLodHierarchy hierarchy{};
    std::string message;
};

PointCloudLodHierarchy BuildPointCloudLodHierarchy(
    const invisible_places::io::LoadedPointCloud& cloud,
    const PointCloudLodBuildConfig& config = {},
    const PointCloudLodBuildProgressCallback& progressCallback = {});

std::vector<PointCloudDrawItemGpu> TraversePointCloudLodHierarchy(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodTraversalParams& params,
    std::stop_token stopToken = {},
    PointCloudLodTraversalDiagnostics* diagnostics = nullptr);

std::vector<PointCloudDrawItemGpu> BuildCoarsePointCloudLodFallbackDrawItems(
    const PointCloudLodHierarchy& hierarchy,
    const PointCloudLodTraversalParams& params,
    std::uint32_t targetRepresentatives);

std::uint64_t HashPointCloudLodCachePath(const std::filesystem::path& sourcePath);

PointCloudLodCacheSource MakePointCloudLodCacheSource(
    const std::filesystem::path& sourcePath,
    const invisible_places::io::LoadedPointCloud& cloud);

std::filesystem::path BuildPointCloudLodCachePath(
    const std::filesystem::path& cacheDirectory,
    const std::filesystem::path& sourcePath);

PointCloudLodCacheLoadResult LoadPointCloudLodHierarchyCache(
    const std::filesystem::path& cachePath,
    const PointCloudLodCacheSource& source,
    const PointCloudLodBuildConfig& config = {});

bool SavePointCloudLodHierarchyCache(
    const std::filesystem::path& cachePath,
    const PointCloudLodCacheSource& source,
    const PointCloudLodBuildConfig& config,
    const PointCloudLodHierarchy& hierarchy,
    std::string* errorMessage = nullptr);

[[nodiscard]] const char* PointCloudLodRendererCostProfileName(PointCloudLodRendererCostProfile profile);

}  // namespace invisible_places::renderer::pointcloud
