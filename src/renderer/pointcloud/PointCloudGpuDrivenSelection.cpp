#include "renderer/pointcloud/PointCloudGpuDrivenSelection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace invisible_places::renderer::pointcloud {

namespace {

constexpr std::uint32_t kMinimumGpuSelectionCandidateReps = 65536U;
constexpr std::uint32_t kMinimumIndirectDrawReps = 4096U;
constexpr double kRequiredGpuSpeedup = 0.95;
constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void HashCombine(std::uint64_t* hash, std::uint64_t value) {
    if (hash == nullptr) {
        return;
    }
    for (int byte = 0; byte < 8; ++byte) {
        *hash ^= (value >> (byte * 8)) & 0xffULL;
        *hash *= kFnvPrime;
    }
}

bool ValidMeasuredMs(double milliseconds) {
    return std::isfinite(milliseconds) && milliseconds > 0.0;
}

std::string JoinReason(std::string first, const std::string& second) {
    if (first.empty()) {
        return second;
    }
    if (second.empty()) {
        return first;
    }
    first += "; ";
    first += second;
    return first;
}

}  // namespace

std::uint64_t HashPointCloudDrawItemsForGpuSelection(
    const std::vector<PointCloudDrawItemGpu>& drawItems) {
    std::uint64_t hash = kFnvOffset;
    HashCombine(&hash, drawItems.size());
    for (const auto& drawItem : drawItems) {
        HashCombine(&hash, drawItem.sourcePointIndex);
        HashCombine(&hash, drawItem.representedSourceCount);
        HashCombine(&hash, drawItem.reserved0);
        HashCombine(&hash, drawItem.reserved1);
    }
    return hash;
}

PointCloudGpuSelectionParityResult ComparePointCloudGpuSelectionParity(
    const std::vector<PointCloudDrawItemGpu>& cpuSelection,
    const std::vector<PointCloudDrawItemGpu>& gpuSelection,
    double minimumSourceOverlapRatio) {
    PointCloudGpuSelectionParityResult result;
    result.cpuSelectedCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(cpuSelection.size(), std::numeric_limits<std::uint32_t>::max()));
    result.gpuSelectedCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(gpuSelection.size(), std::numeric_limits<std::uint32_t>::max()));
    result.selectedCountDelta =
        static_cast<std::int64_t>(result.gpuSelectedCount) -
        static_cast<std::int64_t>(result.cpuSelectedCount);
    result.cpuSelectionHash = HashPointCloudDrawItemsForGpuSelection(cpuSelection);
    result.gpuSelectionHash = HashPointCloudDrawItemsForGpuSelection(gpuSelection);

    if (cpuSelection.empty() && gpuSelection.empty()) {
        result.passed = true;
        result.sourceOverlapRatio = 1.0;
        result.status = "passed: both selections empty";
        return result;
    }

    std::unordered_set<std::uint32_t> cpuSourceIds;
    cpuSourceIds.reserve(cpuSelection.size());
    for (const auto& drawItem : cpuSelection) {
        cpuSourceIds.insert(drawItem.sourcePointIndex);
    }

    std::uint32_t overlap = 0;
    for (const auto& drawItem : gpuSelection) {
        if (cpuSourceIds.contains(drawItem.sourcePointIndex)) {
            ++overlap;
        }
    }

    const auto denominator = std::max(cpuSelection.size(), gpuSelection.size());
    result.sourceOverlapRatio =
        denominator > 0U ? static_cast<double>(overlap) / static_cast<double>(denominator) : 1.0;
    result.passed =
        result.selectedCountDelta == 0 &&
        result.cpuSelectionHash == result.gpuSelectionHash &&
        result.sourceOverlapRatio >= minimumSourceOverlapRatio;

    std::ostringstream status;
    status << (result.passed ? "passed" : "failed")
           << ": CPU " << result.cpuSelectedCount
           << ", GPU " << result.gpuSelectedCount
           << ", overlap " << result.sourceOverlapRatio;
    result.status = status.str();
    return result;
}

PointCloudGpuDrivenSelectionDecision EvaluatePointCloudGpuDrivenSelection(
    const PointCloudGpuDrivenSelectionCapabilities& capabilities,
    const PointCloudGpuDrivenSelectionInputs& inputs) {
    PointCloudGpuDrivenSelectionDecision decision;
    decision.indirectDrawSupported = capabilities.indirectDrawSupported;
    decision.indirectCountSupported = capabilities.indirectCountSupported;
    decision.computeSelectionMs = inputs.gpuSelectionMs;
    decision.compactionMs = inputs.gpuCompactionMs;
    decision.gpuSelectedRepresentativeCount = inputs.selectedDrawItemCount;
    decision.indirectDrawCount = inputs.selectedDrawItemCount > 0U ? 1U : 0U;

    std::string fallbackReason;
    decision.computeSelectionSupported =
        capabilities.computeQueueSupported &&
        capabilities.storageBuffersSupported &&
        capabilities.maxStorageBuffersPerShaderStage >= 4U &&
        capabilities.maxStorageBufferRange >= sizeof(PointCloudDrawItemGpu);
    if (!capabilities.computeQueueSupported) {
        fallbackReason = JoinReason(fallbackReason, "graphics queue does not expose compute");
    }
    if (!capabilities.storageBuffersSupported || capabilities.maxStorageBuffersPerShaderStage < 4U) {
        fallbackReason = JoinReason(fallbackReason, "storage-buffer limits are too small for hierarchy pages");
    }
    if (capabilities.maxStorageBufferRange < sizeof(PointCloudDrawItemGpu)) {
        fallbackReason = JoinReason(fallbackReason, "storage-buffer range is below draw-item ABI size");
    }
    if (!inputs.allowGpuDrivenSelection) {
        fallbackReason = JoinReason(fallbackReason, "GPU-driven selection disabled by policy");
    }
    if (inputs.representativeCount == 0U && inputs.selectedDrawItemCount == 0U) {
        fallbackReason = JoinReason(fallbackReason, "no hierarchy data available for GPU selection");
    }
    if (inputs.selectedDrawItemCount < kMinimumGpuSelectionCandidateReps) {
        fallbackReason = JoinReason(fallbackReason, "selection is below the GPU-selection work threshold");
    }
    if (!inputs.parityChecked) {
        decision.parityStatus = "not checked";
        fallbackReason = JoinReason(fallbackReason, "CPU/GPU selection parity has not been checked");
    } else if (!inputs.parityPassed) {
        decision.parityStatus = "failed";
        fallbackReason = JoinReason(fallbackReason, "CPU/GPU selection parity failed");
    } else {
        decision.parityStatus = "passed";
    }

    if (ValidMeasuredMs(inputs.cpuSelectionMs) && ValidMeasuredMs(inputs.gpuSelectionMs)) {
        decision.computeSelectionBeneficial = inputs.gpuSelectionMs <= inputs.cpuSelectionMs * kRequiredGpuSpeedup;
        if (!decision.computeSelectionBeneficial) {
            fallbackReason = JoinReason(fallbackReason, "GPU selection is not measurably faster than CPU");
        }
    } else {
        decision.computeSelectionBeneficial = false;
        fallbackReason = JoinReason(fallbackReason, "GPU selection timing is not available");
    }

    decision.indirectDrawRecommended =
        inputs.allowIndirectDraw &&
        capabilities.indirectDrawSupported &&
        inputs.selectedDrawItemCount >= kMinimumIndirectDrawReps;
    if (!inputs.allowIndirectDraw) {
        fallbackReason = JoinReason(fallbackReason, "indirect draw disabled by policy");
    } else if (!capabilities.indirectDrawSupported) {
        fallbackReason = JoinReason(fallbackReason, "indirect draw unsupported");
    }

    if (decision.computeSelectionSupported &&
        decision.computeSelectionBeneficial &&
        inputs.allowGpuDrivenSelection &&
        inputs.parityChecked &&
        inputs.parityPassed &&
        inputs.selectedDrawItemCount >= kMinimumGpuSelectionCandidateReps &&
        (inputs.hierarchyNodeCount > 0U || inputs.representativeCount > 0U)) {
        decision.selectionPath = decision.indirectDrawRecommended ? "gpu-driven+indirect" : "gpu-driven";
        decision.fallbackReason.clear();
        return decision;
    }

    decision.selectionPath = decision.indirectDrawRecommended ? "cpu-selection+indirect" : "cpu";
    decision.fallbackReason = fallbackReason.empty() ? "CPU fallback retained by policy" : fallbackReason;
    return decision;
}

}  // namespace invisible_places::renderer::pointcloud
