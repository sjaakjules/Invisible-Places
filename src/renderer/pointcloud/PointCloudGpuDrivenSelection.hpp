#pragma once

#include "renderer/pointcloud/PointCloudLodHierarchy.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace invisible_places::renderer::pointcloud {

struct PointCloudGpuDrivenSelectionCapabilities {
    bool computeQueueSupported = false;
    bool storageBuffersSupported = false;
    bool indirectDrawSupported = false;
    bool indirectCountSupported = false;
    bool portabilitySubsetActive = false;
    bool drawIndirectFirstInstance = false;
    bool multiDrawIndirect = false;
    std::uint32_t maxStorageBuffersPerShaderStage = 0;
    std::uint64_t maxStorageBufferRange = 0;
    std::uint32_t maxDrawIndirectCount = 0;
    std::string deviceName;
    std::string limitations;
};

struct PointCloudGpuDrivenSelectionInputs {
    std::uint32_t hierarchyNodeCount = 0;
    std::uint32_t representativeCount = 0;
    std::uint32_t selectedDrawItemCount = 0;
    double cpuSelectionMs = 0.0;
    double gpuSelectionMs = 0.0;
    double gpuCompactionMs = 0.0;
    bool parityChecked = false;
    bool parityPassed = false;
    bool allowGpuDrivenSelection = true;
    bool allowIndirectDraw = true;
};

struct PointCloudGpuDrivenSelectionDecision {
    std::string selectionPath = "cpu";
    std::string fallbackReason = "GPU-driven selection has not been evaluated";
    std::string parityStatus = "not checked";
    bool computeSelectionSupported = false;
    bool computeSelectionBeneficial = false;
    bool indirectDrawSupported = false;
    bool indirectCountSupported = false;
    bool indirectDrawRecommended = false;
    bool indirectDrawUsed = false;
    std::uint32_t indirectDrawCount = 0;
    std::uint64_t gpuSelectedRepresentativeCount = 0;
    double computeSelectionMs = 0.0;
    double compactionMs = 0.0;
};

struct PointCloudGpuSelectionParityResult {
    bool passed = false;
    std::uint32_t cpuSelectedCount = 0;
    std::uint32_t gpuSelectedCount = 0;
    std::int64_t selectedCountDelta = 0;
    double sourceOverlapRatio = 0.0;
    std::uint64_t cpuSelectionHash = 0;
    std::uint64_t gpuSelectionHash = 0;
    std::string status = "not checked";
};

[[nodiscard]] PointCloudGpuDrivenSelectionDecision EvaluatePointCloudGpuDrivenSelection(
    const PointCloudGpuDrivenSelectionCapabilities& capabilities,
    const PointCloudGpuDrivenSelectionInputs& inputs);

[[nodiscard]] PointCloudGpuSelectionParityResult ComparePointCloudGpuSelectionParity(
    const std::vector<PointCloudDrawItemGpu>& cpuSelection,
    const std::vector<PointCloudDrawItemGpu>& gpuSelection,
    double minimumSourceOverlapRatio = 0.999);

[[nodiscard]] std::uint64_t HashPointCloudDrawItemsForGpuSelection(
    const std::vector<PointCloudDrawItemGpu>& drawItems);

}  // namespace invisible_places::renderer::pointcloud
