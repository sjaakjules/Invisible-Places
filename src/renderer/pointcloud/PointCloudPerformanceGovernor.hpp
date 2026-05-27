#pragma once

#include "renderer/pointcloud/PointCloudLodHierarchy.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace invisible_places::renderer::pointcloud {

inline constexpr std::size_t kPointCloudLodRendererCostProfileCount = 4U;

[[nodiscard]] std::size_t PointCloudLodRendererCostProfileIndex(PointCloudLodRendererCostProfile profile);

struct PointCloudPerformanceGovernor {
    bool ewmaInitialized = false;
    bool timestampSupported = false;
    bool lastTimingValid = false;
    double gpuPointPassEwmaMs = 0.0;
    double gpuCompositeEwmaMs = 0.0;
    float budgetScale = 1.0F;
    std::uint64_t validSampleCount = 0;
    std::uint64_t invalidSampleCount = 0;
};

struct PointCloudPerformanceGovernorSample {
    PointCloudLodRendererCostProfile profile = PointCloudLodRendererCostProfile::FastBasicSquare;
    bool sceneRendered = false;
    bool timestampSupported = false;
    bool timingValid = false;
    double gpuPointPassMs = 0.0;
    double gpuCompositeMs = 0.0;
    double targetPointPassMs = 10.0;
    double targetCompositeMs = 4.0;
    std::uint64_t baseUploadBytesPerFrame = 64ULL * 1024ULL * 1024ULL;
    float minimumBudgetScale = 0.35F;
    float maximumBudgetScale = 1.15F;
    bool representativeBudgetReached = false;
    bool fragmentBudgetReached = false;
    bool blendedFragmentBudgetReached = false;
};

struct PointCloudPerformanceGovernorOutput {
    bool timestampSupported = false;
    bool timingValid = false;
    bool sceneRendered = false;
    bool performanceLimited = false;
    bool visuallyLossless = true;
    bool overTarget = false;
    bool underTarget = false;
    float budgetScale = 1.0F;
    float targetSpacingScale = 1.0F;
    double gpuPointPassMs = 0.0;
    double gpuPointPassEwmaMs = 0.0;
    double gpuCompositeMs = 0.0;
    double gpuCompositeEwmaMs = 0.0;
    double targetPointPassMs = 10.0;
    double targetCompositeMs = 4.0;
    std::uint64_t maxUploadBytesPerFrame = 64ULL * 1024ULL * 1024ULL;
    std::string timestampState = "unavailable";
    std::string status = "visually lossless";
    std::string activeLimit = "visual budget";
};

[[nodiscard]] double DefaultPointCloudGovernorTargetPointPassMs(
    PointCloudLodRendererCostProfile profile,
    bool interactive);

[[nodiscard]] double DefaultPointCloudGovernorTargetCompositeMs(bool interactive);

[[nodiscard]] std::uint64_t DefaultPointCloudGovernorUploadBudgetBytes(
    PointCloudLodRendererCostProfile profile);

PointCloudPerformanceGovernorOutput UpdatePointCloudPerformanceGovernor(
    PointCloudPerformanceGovernor* governor,
    const PointCloudPerformanceGovernorSample& sample);

[[nodiscard]] PointCloudPerformanceGovernorOutput CurrentPointCloudPerformanceGovernorOutput(
    const PointCloudPerformanceGovernor& governor,
    const PointCloudPerformanceGovernorSample& sample);

}  // namespace invisible_places::renderer::pointcloud
