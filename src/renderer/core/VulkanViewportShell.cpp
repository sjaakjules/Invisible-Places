#include "renderer/core/VulkanViewportShell.hpp"

#include "InvisiblePlacesBuildConfig.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <Imath/half.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/matrix.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::renderer::core {

namespace {

#ifndef VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME
#define VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME "VK_KHR_portability_subset"
#endif

struct QueueFamilySelection {
    std::optional<std::uint32_t> graphicsFamily;
    std::optional<std::uint32_t> presentFamily;

    [[nodiscard]] bool IsComplete() const {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapchainSupport {
    VkSurfaceCapabilitiesKHR capabilities{};
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

enum GpuTimestampPass : std::uint32_t {
    kGpuTimestampFastBasicPointPass = 0U,
    kGpuTimestampBeautyDepthPass = 1U,
    kGpuTimestampBeautyPointPass = 2U,
    kGpuTimestampCompositePass = 3U,
    kGpuTimestampPostProcessPass = 4U,
    kGpuTimestampGpuDrawItemCompactionPass = 5U,
    kGpuTimestampGpuFeatureClassProbePass = 6U,
    kGpuTimestampGpuRankProbePass = 7U,
    kGpuTimestampGpuDepthProbePass = 8U,
    kGpuTimestampGpuProjectedAreaProbePass = 9U,
    kGpuTimestampGpuRenderAreaProbePass = 10U,
    kGpuTimestampGpuRepresentedCountProbePass = 11U,
    kGpuTimestampGpuCoverageCompensationProbePass = 12U,
    kGpuTimestampGpuClampFlagsProbePass = 13U,
    kGpuTimestampGpuDrivenIndirectCommandPass = 14U,
    kGpuTimestampPassCount = 15U,
};

constexpr std::uint32_t kGpuTimestampQueriesPerPass = 2U;
constexpr std::uint32_t kGpuTimestampQueryCount = kGpuTimestampPassCount * kGpuTimestampQueriesPerPass;

struct TimestampQueryResult {
    std::uint64_t value = 0;
    std::uint64_t available = 0;
};

struct alignas(16) FrameUniforms {
    glm::mat4 viewProjection{1.0F};
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::vec4 cameraPosition{0.0F, 0.0F, 1.0F, 0.0F};
    glm::vec4 depthParameters{0.0F, 0.05F, 1000.0F, 0.0F};
    glm::vec4 viewportParameters{1.0F, 1.0F, 2.0F, 2.0F};
    glm::vec4 depthOfFieldParameters{0.0F, 1.0F, 8.0F, 24.0F};
    glm::mat4 inverseViewProjection{1.0F};
};

double MillisecondsBetween(
    std::chrono::steady_clock::time_point start,
    std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

struct alignas(16) PointCloudBindingGpu {
    glm::vec4 constantValue{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 range{0.0F, 1.0F, 0.0F, 1.0F};
    glm::vec4 extra{1.0F, 0.0F, 0.0F, 0.0F};
    glm::uvec4 control{0U, 0U, 0U, 0U};
};

struct alignas(16) PointCloudStyleGpu {
    glm::vec4 solidColor{0.93F, 0.88F, 0.72F, 1.0F};
    glm::uvec4 globalControl{0U, 0U, 0U, 1U};
    glm::uvec4 pointMeta{0U, 0U, 0U, 0U};
    glm::uvec4 renderControl{0U, 1U, 0U, 0U};
    glm::vec4 renderParams0{1.0F, 0.55F, 4.0F, 1.6F};
    glm::vec4 renderParams1{80.0F, 0.0005F, 0.16F, 0.08F};
    glm::vec4 renderParams2{1.0F, 64.0F, 1.0F, 64.0F};
    glm::vec4 renderParams3{0.5F, 1.0F, 64.0F, 0.0F};
    PointCloudBindingGpu pointSize{};
    PointCloudBindingGpu opacity{};
    PointCloudBindingGpu emissive{};
    PointCloudBindingGpu xray{};
    PointCloudBindingGpu depthFade{};
    PointCloudBindingGpu colormapPosition{};
    PointCloudBindingGpu surfelDiameter{};
    glm::vec4 colorize{0.95F, 0.68F, 0.28F, 0.0F};
    glm::uvec4 stylisationControl{0U, 0U, 0U, 0U};
    glm::vec4 stylisationParams0{1.0F, 5.0F, 0.35F, 0.35F};
    glm::vec4 stylisationParams1{0.45F, 2.2F, 0.35F, 0.0F};
    glm::vec4 stylisationParams2{0.25F, 0.0F, 0.0F, 0.0F};
    glm::vec4 surfaceMotionParams{0.0F, 1.5F, 0.35F, 0.58F};
    glm::vec4 surfaceMotionStats{0.0F, 1.0F, 1.0F, 0.25F};
    glm::uvec4 causticControl{0U, 0U, 0U, 0U};
    glm::vec4 causticParams0{0.0F, 0.20F, 0.55F, 0.015F};
    glm::vec4 causticParams1{0.045F, 1.15F, 0.08F, 0.0F};
    glm::vec4 causticParams2{0.006F, 0.005F, 0.0F, 0.0F};
    glm::vec4 causticTint{0.62F, 0.88F, 1.0F, 1.0F};
    glm::uvec4 waterEffectControl{0U, 0U, 0U, 0U};
    glm::uvec4 waterEffectSlots0{0U, 0U, 0U, 0U};
    glm::uvec4 waterEffectSlots1{0U, 0U, 0U, 0U};
    glm::vec4 gradientStartColor{0.05F, 0.28F, 0.95F, 1.0F};
    glm::vec4 gradientEndColor{0.96F, 0.94F, 0.58F, 1.0F};
};

struct alignas(16) GaussianSplatPushConstants {
    glm::mat4 localToWorld{1.0F};
    glm::vec4 layerTint{0.94F, 0.82F, 0.60F, 1.0F};
    glm::vec4 style{1.0F, 1.0F, 1.0F, 1.0F};
    glm::uvec4 control{0U, 0U, 1U, 0U};
    glm::vec4 extra{1.5F, 0.0F, 0.0F, 0.0F};
};

struct alignas(16) HighQualityGaussianLayerStyle {
    glm::mat4 localToWorld{1.0F};
    glm::vec4 layerTint{0.94F, 0.82F, 0.60F, 1.0F};
    glm::vec4 style{1.0F, 1.0F, 1.0F, 1.0F};
    glm::uvec4 control{0U, 0U, 1U, 0U};
};

struct alignas(16) HighQualityGaussianPushConstants {
    glm::vec4 extra{1.5F, 0.0F, 0.0F, 0.0F};
};

struct alignas(16) PostProcessPushConstants {
    glm::vec4 edl{0.0F, 24.0F, 0.35F, 1.0F};
};

struct alignas(16) GpuDrivenIndirectCommandPushConstants {
    glm::uvec4 command{0U, 1U, 0U, 0U};
};

struct alignas(16) GpuDrawItemCompactionPushConstants {
    glm::uvec4 control{0U, 0U, 0U, 0U};
    glm::uvec4 metadataWindow{0U, 0xffU, 0U, 0U};
    glm::uvec4 areaWindow{0U, 0U, 0U, 0U};
    glm::uvec4 representedSourceWindow{0U, 0U, 0U, 0U};
    glm::uvec4 profileWindow{0U, 0U, 0U, 0U};
    glm::uvec4 compensationWindow{0U, 0U, 0U, 0U};
};

std::string NormalizeScalarFieldName(std::string_view name) {
    std::string normalized;
    normalized.reserve(name.size());
    for (const char character : name) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) == 0) {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(byte)));
    }
    return normalized;
}

std::optional<std::uint32_t> FindScalarFieldSlotByNormalizedName(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    std::initializer_list<std::string_view> exactNames,
    std::string_view containsName) {
    std::optional<std::uint32_t> containsMatch;
    for (std::size_t index = 0; index < scalarFields.size(); ++index) {
        const auto normalized = NormalizeScalarFieldName(scalarFields[index].name);
        for (const auto exactName : exactNames) {
            if (normalized == exactName) {
                return static_cast<std::uint32_t>(index);
            }
        }
        if (!containsMatch.has_value() && normalized.find(containsName) != std::string::npos) {
            containsMatch = static_cast<std::uint32_t>(index);
        }
    }
    return containsMatch;
}

std::optional<std::uint32_t> FindRoughnessScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindScalarFieldSlotByNormalizedName(
        scalarFields,
        {"roughness", "scalarroughness"},
        "roughness");
}

std::optional<std::uint32_t> FindGroundIdScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindScalarFieldSlotByNormalizedName(
        scalarFields,
        {"groundid", "scalargroundid"},
        "groundid");
}

std::optional<std::uint32_t> FindExactScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    std::string_view name) {
    const auto normalizedName = NormalizeScalarFieldName(name);
    for (std::size_t index = 0; index < scalarFields.size(); ++index) {
        if (NormalizeScalarFieldName(scalarFields[index].name) == normalizedName) {
            return static_cast<std::uint32_t>(index);
        }
    }
    return std::nullopt;
}

constexpr std::uint32_t kSurfelVerticesPerPoint = 6U;
constexpr std::uint32_t kMaxSurfelEncodedPointCount =
    std::numeric_limits<std::uint32_t>::max() / kSurfelVerticesPerPoint;
constexpr std::uint32_t kMinimumAdaptiveIndirectDrawPoints = 4096U;
constexpr std::uint32_t kGpuIndirectCommandModeFromPushConstants = 0U;
constexpr std::uint32_t kGpuIndirectCommandModeFromCompactionStats = 1U;
constexpr std::uint32_t kDrawItemMetadataRankMask = 0x7ffU;
constexpr std::uint32_t kGpuDiagnosticSemanticSelectionClassMask =
    renderer::pointcloud::PointCloudLodRepresentativeClassSpatialCoverage |
    renderer::pointcloud::PointCloudLodRepresentativeClassColorContrast |
    renderer::pointcloud::PointCloudLodRepresentativeClassNormalEdge |
    renderer::pointcloud::PointCloudLodRepresentativeClassScalarMin |
    renderer::pointcloud::PointCloudLodRepresentativeClassScalarMax |
    renderer::pointcloud::PointCloudLodRepresentativeClassScalarThreshold |
    renderer::pointcloud::PointCloudLodRepresentativeClassEmissiveAccent |
    renderer::pointcloud::PointCloudLodRepresentativeClassBlueNoiseFill;
constexpr std::uint32_t kGpuDiagnosticSemanticSelectionProfileMask = 0U;
constexpr std::uint32_t kGpuDiagnosticRankSelectionLimit = kDrawItemMetadataRankMask;
constexpr std::uint32_t kDrawItemMetadataDepthShift = 24U;
constexpr std::uint32_t kDrawItemMetadataDepthMask = 0xffU;
constexpr std::uint32_t kDrawItemMetadataProfileShift = 22U;
constexpr std::uint32_t kDrawItemMetadataProfileMask = 0x3U;
constexpr std::uint32_t kGpuDiagnosticMinSelectionDepth = 0U;
constexpr std::uint32_t kGpuDiagnosticMaxSelectionDepth = kDrawItemMetadataDepthMask;
constexpr std::uint32_t kDrawItemMetadataFlagsShift = 11U;
constexpr std::uint32_t kDrawItemMetadataFlagsMask = 0x7U;
constexpr std::uint32_t kGpuDiagnosticRequiredSelectionFlags = 0U;
constexpr std::uint32_t kGpuDiagnosticRejectedSelectionFlags = 0U;
constexpr float kGpuDiagnosticMinSelectionFootprintAreaPixels = std::numeric_limits<float>::lowest();
constexpr float kGpuDiagnosticMaxSelectionFootprintAreaPixels = std::numeric_limits<float>::max();
constexpr float kGpuDiagnosticMinSelectionRenderAreaPixels = std::numeric_limits<float>::lowest();
constexpr float kGpuDiagnosticMaxSelectionRenderAreaPixels = std::numeric_limits<float>::max();
constexpr float kGpuDiagnosticMinSelectionOpacityCompensation = std::numeric_limits<float>::lowest();
constexpr float kGpuDiagnosticMaxSelectionOpacityCompensation = std::numeric_limits<float>::max();
constexpr float kGpuDiagnosticMinSelectionEmissionCompensation = std::numeric_limits<float>::lowest();
constexpr float kGpuDiagnosticMaxSelectionEmissionCompensation = std::numeric_limits<float>::max();
constexpr std::uint32_t kGpuDiagnosticMinSelectionRepresentedSourceCount = 0U;
constexpr std::uint32_t kGpuDiagnosticMaxSelectionRepresentedSourceCount =
    std::numeric_limits<std::uint32_t>::max();
constexpr bool kGpuDiagnosticCompactionOutputWriteEnabled = true;
constexpr bool kGpuDiagnosticCompactionOrderedOutputEnabled = true;
constexpr std::uint32_t kGpuDiagnosticCompactionOutputCapacityLimit = 3'145'728U;
constexpr double kGpuDiagnosticCompactionTimingEpsilonMs = 0.02;
constexpr std::uint32_t kGpuDiagnosticCompactionSlowFrameThreshold = 1U;
constexpr std::uint32_t kGpuDiagnosticCompactionRetryCooldownFrames = 120U;
constexpr float kGpuDiagnosticSelectionFrustumGuardBand = 1.05F;
constexpr bool kGpuDiagnosticSelectionFrustumGuardEnabled = false;
constexpr std::string_view kGpuDiagnosticSelectionFrustumFallbackReason =
    "GPU geometry-frustum predicate is disabled because the previous MoltenVK/sample measurement was slower than metadata-only full-range compaction";
constexpr std::uint32_t kGpuDiagnosticFeatureClassProbeMask =
    renderer::pointcloud::PointCloudLodRepresentativeClassColorContrast |
    renderer::pointcloud::PointCloudLodRepresentativeClassNormalEdge |
    renderer::pointcloud::PointCloudLodRepresentativeClassScalarMin |
    renderer::pointcloud::PointCloudLodRepresentativeClassScalarMax |
    renderer::pointcloud::PointCloudLodRepresentativeClassScalarThreshold |
    renderer::pointcloud::PointCloudLodRepresentativeClassEmissiveAccent;
constexpr std::uint32_t kGpuDiagnosticRankProbeLimit = 0xffU;
constexpr std::uint32_t kGpuDiagnosticDepthProbeMin = 7U;
constexpr std::uint32_t kGpuDiagnosticDepthProbeMax = kDrawItemMetadataDepthMask;
constexpr float kGpuDiagnosticProjectedAreaProbeMinFootprintAreaPixels = 4.0F;
constexpr float kGpuDiagnosticProjectedAreaProbeMaxFootprintAreaPixels =
    std::numeric_limits<float>::max();
constexpr float kGpuDiagnosticProjectedAreaProbeMinRenderAreaPixels =
    std::numeric_limits<float>::lowest();
constexpr float kGpuDiagnosticProjectedAreaProbeMaxRenderAreaPixels =
    std::numeric_limits<float>::max();
constexpr float kGpuDiagnosticRenderAreaProbeMinFootprintAreaPixels =
    std::numeric_limits<float>::lowest();
constexpr float kGpuDiagnosticRenderAreaProbeMaxFootprintAreaPixels =
    std::numeric_limits<float>::max();
constexpr float kGpuDiagnosticRenderAreaProbeMinRenderAreaPixels = 4.0F;
constexpr float kGpuDiagnosticRenderAreaProbeMaxRenderAreaPixels =
    std::numeric_limits<float>::max();
constexpr std::uint32_t kGpuDiagnosticRepresentedCountProbeMin = 2U;
constexpr std::uint32_t kGpuDiagnosticRepresentedCountProbeMax =
    std::numeric_limits<std::uint32_t>::max();
constexpr float kGpuDiagnosticCoverageCompensationProbeMinOpacity = 1.25F;
constexpr float kGpuDiagnosticCoverageCompensationProbeMaxOpacity =
    std::numeric_limits<float>::max();
constexpr float kGpuDiagnosticCoverageCompensationProbeMinEmission = 1.25F;
constexpr float kGpuDiagnosticCoverageCompensationProbeMaxEmission =
    std::numeric_limits<float>::max();
constexpr std::uint32_t kGpuDiagnosticClampFlagsProbeRequiredFlags = 0x6U;
constexpr std::uint32_t kGpuDiagnosticClampFlagsProbeRejectedFlags = 0U;
constexpr std::uint32_t kDrawItemMetadataClassShift = 14U;
constexpr std::uint32_t kDrawItemMetadataClassMask = 0xffU;

std::uint32_t GpuDiagnosticSelectionLimit(std::uint32_t drawPointCount) {
    return drawPointCount;
}

std::uint32_t GpuDiagnosticCompactionOutputCapacity(std::uint32_t drawPointCount) {
    if (!kGpuDiagnosticCompactionOutputWriteEnabled) {
        return 1U;
    }
    return std::max<std::uint32_t>(
        1U,
        std::min(GpuDiagnosticSelectionLimit(drawPointCount), kGpuDiagnosticCompactionOutputCapacityLimit));
}

std::uint32_t GpuDiagnosticRendererProfileSelectionMask(
    renderer::pointcloud::PointCloudLodRendererCostProfile profile) {
    const auto profileIndex = static_cast<std::uint32_t>(profile);
    if (profileIndex > kDrawItemMetadataProfileMask) {
        return kGpuDiagnosticSemanticSelectionProfileMask;
    }
    return 1U << profileIndex;
}

std::uint32_t DrawItemRepresentativeClassFlags(
    const renderer::pointcloud::PointCloudDrawItemGpu& item) {
    return (item.reserved1 >> kDrawItemMetadataClassShift) & kDrawItemMetadataClassMask;
}

std::uint32_t DrawItemRendererCostProfileBit(
    const renderer::pointcloud::PointCloudDrawItemGpu& item) {
    const auto profileIndex =
        (item.reserved1 >> kDrawItemMetadataProfileShift) & kDrawItemMetadataProfileMask;
    return 1U << profileIndex;
}

std::uint32_t DrawItemRepresentativePackedRank(
    const renderer::pointcloud::PointCloudDrawItemGpu& item) {
    return item.reserved1 & kDrawItemMetadataRankMask;
}

std::uint32_t DrawItemRepresentativePackedDepth(
    const renderer::pointcloud::PointCloudDrawItemGpu& item) {
    return (item.reserved1 >> kDrawItemMetadataDepthShift) & kDrawItemMetadataDepthMask;
}

std::uint32_t DrawItemRepresentativePackedFlags(
    const renderer::pointcloud::PointCloudDrawItemGpu& item) {
    return (item.reserved1 >> kDrawItemMetadataFlagsShift) & kDrawItemMetadataFlagsMask;
}

std::uint32_t FloatBits(float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

bool DrawItemWithinProjectedAreaWindow(
    const renderer::pointcloud::PointCloudDrawItemGpu& item,
    float selectionMinFootprintAreaPixels,
    float selectionMaxFootprintAreaPixels,
    float selectionMinRenderAreaPixels,
    float selectionMaxRenderAreaPixels) {
    return item.footprintAreaPixels >= selectionMinFootprintAreaPixels &&
           item.footprintAreaPixels <= selectionMaxFootprintAreaPixels &&
           item.renderAreaPixels >= selectionMinRenderAreaPixels &&
           item.renderAreaPixels <= selectionMaxRenderAreaPixels;
}

bool DrawItemWithinRepresentedSourceWindow(
    const renderer::pointcloud::PointCloudDrawItemGpu& item,
    std::uint32_t selectionMinRepresentedSourceCount,
    std::uint32_t selectionMaxRepresentedSourceCount) {
    return item.representedSourceCount >= selectionMinRepresentedSourceCount &&
           item.representedSourceCount <= selectionMaxRepresentedSourceCount;
}

bool DrawItemWithinCompensationWindow(
    const renderer::pointcloud::PointCloudDrawItemGpu& item,
    float selectionMinOpacityCompensation,
    float selectionMaxOpacityCompensation,
    float selectionMinEmissionCompensation,
    float selectionMaxEmissionCompensation) {
    return item.opacityCompensation >= selectionMinOpacityCompensation &&
           item.opacityCompensation <= selectionMaxOpacityCompensation &&
           item.emissionCompensation >= selectionMinEmissionCompensation &&
           item.emissionCompensation <= selectionMaxEmissionCompensation;
}

bool DrawItemWithinFrustumGuard(
    const renderer::pointcloud::PointCloudDrawItemGpu& item,
    const std::vector<invisible_places::io::Float3>& positions,
    const glm::mat4& viewProjection,
    float frustumGuardBand) {
    if (item.sourcePointIndex >= positions.size()) {
        return false;
    }
    const auto& position = positions[item.sourcePointIndex];
    const glm::vec4 clipPosition =
        viewProjection * glm::vec4{position.x, position.y, position.z, 1.0F};
    if (clipPosition.w <= 0.0F) {
        return false;
    }
    const auto guardedW = clipPosition.w * std::max(1.0F, frustumGuardBand);
    return clipPosition.x >= -guardedW &&
           clipPosition.x <= guardedW &&
           clipPosition.y >= -guardedW &&
           clipPosition.y <= guardedW &&
           clipPosition.z >= -guardedW &&
           clipPosition.z <= guardedW;
}

std::uint32_t IndirectVertexCountFromCompactedItems(
    std::uint32_t compactedItemCount,
    bool worldSurfels) {
    const auto multiplier = worldSurfels ? kSurfelVerticesPerPoint : 1U;
    return std::min(compactedItemCount, std::numeric_limits<std::uint32_t>::max() / multiplier) * multiplier;
}

bool MatricesApproximatelyEqual(const glm::mat4& left, const glm::mat4& right, float epsilon = 1.0e-6F) {
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            if (std::abs(left[column][row] - right[column][row]) > epsilon) {
                return false;
            }
        }
    }

    return true;
}

constexpr std::array<const char*, 1> kRequiredDeviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
};

[[nodiscard]] bool HasExtension(
    const std::vector<VkExtensionProperties>& extensions,
    std::string_view name) {
    return std::any_of(extensions.begin(), extensions.end(), [name](const VkExtensionProperties& extension) {
        return name == extension.extensionName;
    });
}

std::vector<VkExtensionProperties> EnumerateDeviceExtensions(VkPhysicalDevice device) {
    std::uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data());
    return extensions;
}

QueueFamilySelection FindQueueFamilies(VkPhysicalDevice device, VkSurfaceKHR surface) {
    QueueFamilySelection selection;

    std::uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

    for (std::uint32_t index = 0; index < queueFamilyCount; ++index) {
        const bool supportsGraphics = (queueFamilies[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0;
        if (supportsGraphics && !selection.graphicsFamily.has_value()) {
            selection.graphicsFamily = index;
        }

        VkBool32 presentSupported = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, index, surface, &presentSupported);
        if (presentSupported == VK_TRUE) {
            selection.presentFamily = index;
        }

        if (selection.IsComplete()) {
            break;
        }
    }

    return selection;
}

SwapchainSupport QuerySwapchainSupport(VkPhysicalDevice device, VkSurfaceKHR surface) {
    SwapchainSupport support;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &support.capabilities);

    std::uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);
    support.formats.resize(formatCount);
    if (formatCount > 0) {
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, support.formats.data());
    }

    std::uint32_t presentModeCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);
    support.presentModes.resize(presentModeCount);
    if (presentModeCount > 0) {
        vkGetPhysicalDeviceSurfacePresentModesKHR(
            device, surface, &presentModeCount, support.presentModes.data());
    }

    return support;
}

bool IsDeviceSuitable(VkPhysicalDevice device, VkSurfaceKHR surface, bool* portabilitySubsetEnabled) {
    const auto queueFamilies = FindQueueFamilies(device, surface);
    if (!queueFamilies.IsComplete()) {
        return false;
    }

    const auto extensions = EnumerateDeviceExtensions(device);
    for (const char* requiredExtension : kRequiredDeviceExtensions) {
        if (!HasExtension(extensions, requiredExtension)) {
            return false;
        }
    }

    const auto swapchainSupport = QuerySwapchainSupport(device, surface);
    if (swapchainSupport.formats.empty() || swapchainSupport.presentModes.empty()) {
        return false;
    }

    VkPhysicalDeviceFeatures features{};
    vkGetPhysicalDeviceFeatures(device, &features);
    if (features.largePoints == VK_FALSE) {
        return false;
    }

    if (portabilitySubsetEnabled != nullptr) {
        *portabilitySubsetEnabled = HasExtension(extensions, VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
    }

    return true;
}

VkSurfaceFormatKHR ChooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& formats) {
    for (const auto& format : formats) {
        if (format.format == VK_FORMAT_B8G8R8A8_SRGB &&
            format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return format;
        }
    }

    return formats.front();
}

VkPresentModeKHR ChoosePresentMode(const std::vector<VkPresentModeKHR>& presentModes) {
    for (const auto& mode : presentModes) {
        if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
            return mode;
        }
    }

    return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D ChooseExtent(GLFWwindow* window, const VkSurfaceCapabilitiesKHR& capabilities) {
    if (capabilities.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);

    VkExtent2D extent{
        static_cast<std::uint32_t>(width),
        static_cast<std::uint32_t>(height),
    };

    extent.width = std::clamp(
        extent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
    extent.height = std::clamp(
        extent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
    return extent;
}

void Check(VkResult result, std::string_view context) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(
            std::string{context} + " failed with VkResult " + std::to_string(static_cast<int>(result)));
    }
}

void CheckImGuiResult(VkResult result) {
    Check(result, "ImGui Vulkan backend");
}

void ApplyImGuiStyle() {
    ImGui::StyleColorsLight();
    auto& style = ImGui::GetStyle();
    style.WindowRounding = 18.0F;
    style.FrameRounding = 10.0F;
    style.GrabRounding = 10.0F;
    style.PopupRounding = 12.0F;
    style.ScrollbarRounding = 10.0F;
    style.WindowPadding = ImVec2{16.0F, 14.0F};
    style.FramePadding = ImVec2{10.0F, 7.0F};
    style.ItemSpacing = ImVec2{10.0F, 8.0F};

    auto& colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4{0.95F, 0.94F, 0.90F, 0.93F};
    colors[ImGuiCol_ChildBg] = ImVec4{0.97F, 0.96F, 0.93F, 0.86F};
    colors[ImGuiCol_Border] = ImVec4{0.63F, 0.58F, 0.47F, 0.35F};
    colors[ImGuiCol_FrameBg] = ImVec4{0.90F, 0.88F, 0.82F, 0.92F};
    colors[ImGuiCol_FrameBgHovered] = ImVec4{0.83F, 0.80F, 0.72F, 0.94F};
    colors[ImGuiCol_FrameBgActive] = ImVec4{0.78F, 0.74F, 0.63F, 0.96F};
    colors[ImGuiCol_TitleBg] = ImVec4{0.89F, 0.86F, 0.76F, 0.95F};
    colors[ImGuiCol_TitleBgActive] = ImVec4{0.85F, 0.80F, 0.64F, 0.98F};
    colors[ImGuiCol_Button] = ImVec4{0.71F, 0.63F, 0.44F, 0.86F};
    colors[ImGuiCol_ButtonHovered] = ImVec4{0.76F, 0.67F, 0.46F, 0.92F};
    colors[ImGuiCol_ButtonActive] = ImVec4{0.62F, 0.54F, 0.37F, 0.95F};
    colors[ImGuiCol_Header] = ImVec4{0.78F, 0.71F, 0.53F, 0.75F};
    colors[ImGuiCol_HeaderHovered] = ImVec4{0.81F, 0.74F, 0.55F, 0.87F};
    colors[ImGuiCol_HeaderActive] = ImVec4{0.70F, 0.62F, 0.46F, 0.94F};
    colors[ImGuiCol_SliderGrab] = ImVec4{0.53F, 0.44F, 0.24F, 0.92F};
    colors[ImGuiCol_SliderGrabActive] = ImVec4{0.45F, 0.37F, 0.19F, 0.98F};
}

const invisible_places::io::ScalarFieldStats* ResolveBindingScalarFieldStats(
    const invisible_places::style::RenderParameterBinding& binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    if (binding.fieldMap.fieldSlot < 0 ||
        static_cast<std::size_t>(binding.fieldMap.fieldSlot) >= scalarFields.size()) {
        return nullptr;
    }

    return &scalarFields[static_cast<std::size_t>(binding.fieldMap.fieldSlot)];
}

PointCloudBindingGpu MakePointCloudBindingGpu(
    const invisible_places::style::RenderParameterBinding& binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    float inactiveDefault) {
    const auto* fieldStats = ResolveBindingScalarFieldStats(binding, scalarFields);
    PointCloudBindingGpu gpuBinding;
    gpuBinding.constantValue = glm::vec4{
        binding.active ? binding.constantValue[0] : inactiveDefault,
        binding.constantValue[1],
        binding.constantValue[2],
        binding.constantValue[3],
    };
    gpuBinding.range = glm::vec4{
        invisible_places::style::ResolveBindingInputMinimum(binding, fieldStats),
        invisible_places::style::ResolveBindingInputMaximum(binding, fieldStats),
        binding.fieldMap.outputMin,
        binding.fieldMap.outputMax,
    };
    gpuBinding.extra = glm::vec4{binding.fieldMap.gamma, 0.0F, 0.0F, 0.0F};
    gpuBinding.control = glm::uvec4{
        binding.active ? static_cast<std::uint32_t>(binding.mode) : 0U,
        binding.active && binding.fieldMap.fieldSlot >= 0
            ? static_cast<std::uint32_t>(binding.fieldMap.fieldSlot)
            : 0xFFFFFFFFU,
        binding.active ? binding.fieldMap.flags : 0U,
        binding.active ? 1U : 0U,
    };
    return gpuBinding;
}

std::uint32_t InactivePointBindingCount(const renderer::pointcloud::PointCloudStyleState& style) {
    std::uint32_t count = 0;
    count += style.pointSize.active ? 0U : 1U;
    count += style.surfelDiameter.active ? 0U : 1U;
    count += style.opacity.active ? 0U : 1U;
    count += style.emissiveStrength.active ? 0U : 1U;
    count += style.xrayStrength.active ? 0U : 1U;
    count += style.depthFade.active ? 0U : 1U;
    count += style.colormapPosition.active ? 0U : 1U;
    return count;
}

void UpdateImGuiPlatformWindowsIfNeeded() {
    if (ImGui::GetCurrentContext() == nullptr ||
        (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) == 0) {
        return;
    }

    GLFWwindow* backupContext = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(backupContext);
}

void ScalePointCloudBindingGpu(PointCloudBindingGpu* binding, float scale) {
    if (binding == nullptr) {
        return;
    }

    const float safeScale = std::max(0.001F, scale);
    binding->constantValue.x *= safeScale;
    binding->range.z *= safeScale;
    binding->range.w *= safeScale;
}

std::uint16_t FloatToHalfBits(float value) {
    return Imath::half{std::max(0.0F, value)}.bits();
}

VkDescriptorPoolSize MakePoolSize(VkDescriptorType type, std::uint32_t descriptorCount) {
    return VkDescriptorPoolSize{type, descriptorCount};
}

VkPipelineColorBlendAttachmentState MakeAlphaBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkPipelineColorBlendAttachmentState MakeAdditiveBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkPipelineColorBlendAttachmentState MakeRevealageBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ZERO;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkPipelineColorBlendAttachmentState MakePremultipliedAlphaBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkShaderModule CreateShaderModule(VkDevice device, const std::vector<char>& code, const char* label) {
    VkShaderModuleCreateInfo moduleInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    moduleInfo.codeSize = code.size();
    moduleInfo.pCode = reinterpret_cast<const std::uint32_t*>(code.data());

    VkShaderModule module = VK_NULL_HANDLE;
    Check(vkCreateShaderModule(device, &moduleInfo, nullptr, &module), label);
    return module;
}

bool FormatSupportsOptimalFeatures(
    VkPhysicalDevice physicalDevice,
    VkFormat format,
    VkFormatFeatureFlags requiredFeatures) {
    VkFormatProperties properties{};
    vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &properties);
    return (properties.optimalTilingFeatures & requiredFeatures) == requiredFeatures;
}

}  // namespace

VulkanViewportShell::VulkanViewportShell(GLFWwindow* window) : window_(window) {
    if (window_ == nullptr) {
        throw std::runtime_error{"Vulkan viewport requires a valid GLFW window."};
    }

    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, &VulkanViewportShell::FramebufferResizeCallback);

    CreateInstance();
    CreateSurface();
    PickPhysicalDevice();
    CreateLogicalDevice();
    CreateSwapchain();
    CreateImageViews();
    CreateRenderPass();
    CreatePresentRenderPass();
    CreatePointDescriptorSetLayout();
    CreateGaussianSplatDescriptorSetLayout();
    CreateHighQualityGaussianSplatDescriptorSetLayout();
    CreateCompositeDescriptorSetLayout();
    CreatePostProcessDescriptorSetLayout();
    CreateGpuDrivenSelectionDescriptorSetLayout();
    CreateGpuCompactionDescriptorSetLayout();
    CreateDescriptorPools();
    CreatePostProcessSampler();
    CreateUniformResources();
    CreateSceneColorResources();
    CreateDepthResources();
    CreateAccumulationResources();
    CreateLinearDepthResources();
    CreatePointPipelines();
    CreateGaussianSplatPipeline();
    CreateHighQualityGaussianSplatPipeline();
    CreateCompositePipeline();
    CreatePostProcessPipeline();
    CreateGpuDrivenSelectionPipeline();
    CreateGpuCompactionPipeline();
    CreateFramebuffers();
    CreatePresentFramebuffers();
    CreateCommandPool();
    CreateCommandBuffers();
    CreateSyncObjects();
    CreateOrUpdateCompositeDescriptorSet();
    CreateOrUpdatePostProcessDescriptorSets();
    CreateImGuiResources();
    UploadImGuiFonts();
}

VulkanViewportShell::~VulkanViewportShell() {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    for (auto& resources : pointCloudResources_) {
        CleanupPointCloudResources(&resources);
    }
    pointCloudResources_.clear();

    for (auto& resources : gaussianSplatResources_) {
        CleanupGaussianSplatResources(&resources);
    }
    gaussianSplatResources_.clear();
    CleanupHighQualityGaussianScene();
    CleanupExrExportResources();

    CleanupSwapchain();

    for (auto& frame : frameResources_) {
        DestroyBuffer(&frame.uniformBuffer);
        if (frame.imageAvailableSemaphore != VK_NULL_HANDLE) {
            vkDestroySemaphore(device_, frame.imageAvailableSemaphore, nullptr);
            frame.imageAvailableSemaphore = VK_NULL_HANDLE;
        }
        if (frame.renderFinishedSemaphore != VK_NULL_HANDLE) {
            vkDestroySemaphore(device_, frame.renderFinishedSemaphore, nullptr);
            frame.renderFinishedSemaphore = VK_NULL_HANDLE;
        }
        if (frame.fence != VK_NULL_HANDLE) {
            vkDestroyFence(device_, frame.fence, nullptr);
            frame.fence = VK_NULL_HANDLE;
        }
        if (frame.timestampQueryPool != VK_NULL_HANDLE) {
            vkDestroyQueryPool(device_, frame.timestampQueryPool, nullptr);
            frame.timestampQueryPool = VK_NULL_HANDLE;
        }
    }

    if (commandPool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, commandPool_, nullptr);
        commandPool_ = VK_NULL_HANDLE;
    }

    if (pointDepthPrepassPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointDepthPrepassPipeline_, nullptr);
    }
    if (pointAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointAccumulationPipeline_, nullptr);
    }
    if (pointConstantSimpleAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointConstantSimpleAccumulationPipeline_, nullptr);
    }
    if (pointOpaqueHardDiscPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointOpaqueHardDiscPipeline_, nullptr);
    }
    if (pointFastBasicPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointFastBasicPipeline_, nullptr);
    }
    if (surfelDepthPrepassPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelDepthPrepassPipeline_, nullptr);
    }
    if (surfelAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelAccumulationPipeline_, nullptr);
    }
    if (surfelConstantSimpleAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelConstantSimpleAccumulationPipeline_, nullptr);
    }
    if (surfelOpaqueHardDiscPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelOpaqueHardDiscPipeline_, nullptr);
    }
    if (gaussianSplatPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, gaussianSplatPipeline_, nullptr);
    }
    if (highQualityGaussianSplatPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, highQualityGaussianSplatPipeline_, nullptr);
    }
    if (compositePipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, compositePipeline_, nullptr);
    }
    if (postProcessPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, postProcessPipeline_, nullptr);
    }
    if (gpuDrivenIndirectCommandPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, gpuDrivenIndirectCommandPipeline_, nullptr);
    }
    if (gpuDrawItemCompactionPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, gpuDrawItemCompactionPipeline_, nullptr);
    }
    if (pointPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, pointPipelineLayout_, nullptr);
    }
    if (gaussianSplatPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, gaussianSplatPipelineLayout_, nullptr);
    }
    if (highQualityGaussianSplatPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, highQualityGaussianSplatPipelineLayout_, nullptr);
    }
    if (compositePipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, compositePipelineLayout_, nullptr);
    }
    if (postProcessPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, postProcessPipelineLayout_, nullptr);
    }
    if (gpuDrivenSelectionPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, gpuDrivenSelectionPipelineLayout_, nullptr);
    }
    if (gpuCompactionPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, gpuCompactionPipelineLayout_, nullptr);
    }
    if (postProcessSampler_ != VK_NULL_HANDLE) {
        vkDestroySampler(device_, postProcessSampler_, nullptr);
    }
    if (descriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
    }
    if (gaussianSplatDescriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, gaussianSplatDescriptorPool_, nullptr);
    }
    if (imguiDescriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, imguiDescriptorPool_, nullptr);
    }
    if (pointDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, pointDescriptorSetLayout_, nullptr);
    }
    if (gaussianSplatDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, gaussianSplatDescriptorSetLayout_, nullptr);
    }
    if (highQualityGaussianSplatDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, highQualityGaussianSplatDescriptorSetLayout_, nullptr);
    }
    if (compositeDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, compositeDescriptorSetLayout_, nullptr);
    }
    if (postProcessDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, postProcessDescriptorSetLayout_, nullptr);
    }
    if (gpuDrivenSelectionDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, gpuDrivenSelectionDescriptorSetLayout_, nullptr);
    }
    if (gpuCompactionDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, gpuCompactionDescriptorSetLayout_, nullptr);
    }
    if (renderPass_ != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, renderPass_, nullptr);
        renderPass_ = VK_NULL_HANDLE;
    }
    if (presentRenderPass_ != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, presentRenderPass_, nullptr);
        presentRenderPass_ = VK_NULL_HANDLE;
    }

    if (device_ != VK_NULL_HANDLE) {
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    if (surface_ != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(instance_, surface_, nullptr);
        surface_ = VK_NULL_HANDLE;
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
    }
}

void VulkanViewportShell::BeginUiFrame() {
    if (ImGui::GetCurrentContext() == nullptr || uiFrameBegun_) {
        return;
    }

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    auto& io = ImGui::GetIO();
    if ((io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0 &&
        ImGui::GetPlatformIO().Monitors.Size == 0) {
        io.ConfigFlags &= ~ImGuiConfigFlags_ViewportsEnable;
    }
    ImGui::NewFrame();
    uiFrameBegun_ = true;
}

bool VulkanViewportShell::SceneImageNeedsRender(std::uint32_t imageIndex) const {
    if (!liveSceneRenderingEnabled_) {
        return false;
    }
    if (!sceneCachingEnabled_) {
        return true;
    }
    return imageIndex >= sceneImageRevisions_.size() ||
           sceneImageRevisions_[imageIndex] != sceneRevision_;
}

bool VulkanViewportShell::AnySceneImageNeedsRender() const {
    if (!liveSceneRenderingEnabled_) {
        return false;
    }
    if (!sceneCachingEnabled_) {
        return true;
    }
    if (sceneImageRevisions_.size() != swapchainImages_.size()) {
        return true;
    }
    return std::any_of(
        sceneImageRevisions_.begin(),
        sceneImageRevisions_.end(),
        [this](std::uint64_t imageRevision) { return imageRevision != sceneRevision_; });
}

void VulkanViewportShell::DrawFrame() {
    const bool collectDiagnostics = diagnosticsEnabled_;
    const auto frameStart = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};

    if (uiFrameBegun_) {
        ImGui::Render();
        uiFrameBegun_ = false;
    }
    const auto uiEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                          : std::chrono::steady_clock::time_point{};

    auto& frame = frameResources_[currentFrameIndex_];
    vkWaitForFences(device_, 1, &frame.fence, VK_TRUE, UINT64_MAX);
    if (collectDiagnostics) {
        ReadPreviousGpuTimestampResults(&frame, currentFrameIndex_);
        ReadPreviousGpuCompactionResults(currentFrameIndex_);
    }
    const auto fenceEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                             : std::chrono::steady_clock::time_point{};
    std::uint32_t drawItemBufferReallocations = 0;
    const auto drawItemUploadStart = collectDiagnostics ? std::chrono::steady_clock::now()
                                                        : std::chrono::steady_clock::time_point{};
    const bool sceneNeedsPrepare = AnySceneImageNeedsRender();
    if (sceneNeedsPrepare) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            auto* resources = FindPointCloudResources(layer.layerId);
            if (resources == nullptr) {
                continue;
            }
            const bool reallocated = UpdatePointCloudDrawItemBuffer(
                resources,
                currentFrameIndex_,
                layer.useAdaptiveDrawItems ? layer.adaptiveDrawItems.get() : nullptr,
                layer.useAdaptiveDrawItems ? layer.adaptiveLodRevision : 0ULL);
            drawItemBufferReallocations += reallocated ? 1U : 0U;
        }
    }
    const auto drawItemUploadEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                                      : std::chrono::steady_clock::time_point{};
    if (sceneNeedsPrepare) {
        RefreshHighQualityGaussianScene(currentFrameIndex_);
        UpdateUniformBuffer(currentFrameIndex_);
    }
    const auto prepareEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};

    std::uint32_t imageIndex = 0;
    const VkResult acquireResult =
        vkAcquireNextImageKHR(
            device_,
            swapchain_,
            UINT64_MAX,
            frame.imageAvailableSemaphore,
            VK_NULL_HANDLE,
            &imageIndex);
    const auto acquireEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};

    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
        UpdateImGuiPlatformWindowsIfNeeded();
        RecreateSwapchain();
        return;
    }
    if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
        Check(acquireResult, "vkAcquireNextImageKHR");
    }

    if (imageIndex < swapchainImagesInFlight_.size() && swapchainImagesInFlight_[imageIndex] != VK_NULL_HANDLE) {
        vkWaitForFences(device_, 1, &swapchainImagesInFlight_[imageIndex], VK_TRUE, UINT64_MAX);
    }
    const auto imageWaitEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                                 : std::chrono::steady_clock::time_point{};
    if (imageIndex < swapchainImagesInFlight_.size()) {
        swapchainImagesInFlight_[imageIndex] = frame.fence;
    }

    Check(vkResetFences(device_, 1, &frame.fence), "vkResetFences");
    Check(vkResetCommandBuffer(frame.commandBuffer, 0), "vkResetCommandBuffer");
    RecordCommandBuffer(frame.commandBuffer, imageIndex, currentFrameIndex_);
    const auto recordEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                              : std::chrono::steady_clock::time_point{};

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = &frame.imageAvailableSemaphore;
    submitInfo.pWaitDstStageMask = &waitStage;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &frame.commandBuffer;
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = &frame.renderFinishedSemaphore;

    Check(vkQueueSubmit(graphicsQueue_, 1, &submitInfo, frame.fence), "vkQueueSubmit");
    const auto submitEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                              : std::chrono::steady_clock::time_point{};

    VkPresentInfoKHR presentInfo{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = &frame.renderFinishedSemaphore;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain_;
    presentInfo.pImageIndices = &imageIndex;

    const VkResult presentResult = vkQueuePresentKHR(presentQueue_, &presentInfo);
    const auto presentEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR || framebufferResized_) {
        framebufferResized_ = false;
        UpdateImGuiPlatformWindowsIfNeeded();
        RecreateSwapchain();
        return;
    }

    Check(presentResult, "vkQueuePresentKHR");
    UpdateImGuiPlatformWindowsIfNeeded();
    const auto frameEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                             : std::chrono::steady_clock::time_point{};
    if (collectDiagnostics) {
        constexpr double kFrameAverageWindowMs = 500.0;
        diagnostics_.framesInFlight = static_cast<std::uint32_t>(kFramesInFlight);
        diagnostics_.swapchainImageCount = static_cast<std::uint32_t>(swapchainImages_.size());
        diagnostics_.currentFrameIndex = static_cast<std::uint32_t>(currentFrameIndex_);
        diagnostics_.frameAverageWindowSeconds = kFrameAverageWindowMs / 1000.0;
        diagnostics_.frameUiRenderMs = MillisecondsBetween(frameStart, uiEnd);
        diagnostics_.frameFenceWaitMs = MillisecondsBetween(uiEnd, fenceEnd);
        diagnostics_.pointDrawItemUploadMs = MillisecondsBetween(drawItemUploadStart, drawItemUploadEnd);
        diagnostics_.pointDrawItemWaitMs = 0.0;
        diagnostics_.pointDrawItemBufferReallocations = drawItemBufferReallocations;
        diagnostics_.framePrepareMs = MillisecondsBetween(fenceEnd, prepareEnd);
        diagnostics_.frameAcquireMs = MillisecondsBetween(prepareEnd, acquireEnd);
        diagnostics_.frameImageWaitMs = MillisecondsBetween(acquireEnd, imageWaitEnd);
        diagnostics_.frameCommandBufferMs = MillisecondsBetween(imageWaitEnd, recordEnd);
        diagnostics_.frameSubmitMs = MillisecondsBetween(recordEnd, submitEnd);
        diagnostics_.framePresentMs = MillisecondsBetween(submitEnd, presentEnd);
        diagnostics_.framePlatformWindowsMs = MillisecondsBetween(presentEnd, frameEnd);
        diagnostics_.frameRenderMs = MillisecondsBetween(frameStart, frameEnd);
        diagnostics_.frameFps =
            diagnostics_.frameRenderMs > 0.0 ? 1000.0 / diagnostics_.frameRenderMs : 0.0;
        if (!diagnosticsTimingInitialized_) {
            diagnostics_.averageFrameRenderMs = diagnostics_.frameRenderMs;
            diagnostics_.averageFrameFps = diagnostics_.frameFps;
            diagnostics_.minFrameRenderMs = diagnostics_.frameRenderMs;
            diagnostics_.maxFrameRenderMs = diagnostics_.frameRenderMs;
            diagnosticsTimingInitialized_ = true;
        } else {
            diagnostics_.minFrameRenderMs =
                std::min(diagnostics_.minFrameRenderMs, diagnostics_.frameRenderMs);
            diagnostics_.maxFrameRenderMs =
                std::max(diagnostics_.maxFrameRenderMs, diagnostics_.frameRenderMs);
        }
        diagnosticsFpsWindowMs_ += diagnostics_.frameRenderMs;
        ++diagnosticsFpsWindowFrames_;
        if (diagnosticsFpsWindowMs_ >= kFrameAverageWindowMs) {
            diagnostics_.averageFrameRenderMs =
                diagnosticsFpsWindowMs_ / static_cast<double>(diagnosticsFpsWindowFrames_);
            diagnostics_.averageFrameFps =
                diagnosticsFpsWindowMs_ > 0.0
                    ? (1000.0 * static_cast<double>(diagnosticsFpsWindowFrames_)) / diagnosticsFpsWindowMs_
                    : 0.0;
            diagnosticsFpsWindowMs_ = 0.0;
            diagnosticsFpsWindowFrames_ = 0;
        }
    }
    currentFrameIndex_ = (currentFrameIndex_ + 1U) % kFramesInFlight;
}

void VulkanViewportShell::WaitIdle() const {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }
}

void VulkanViewportShell::SetDiagnosticsEnabled(bool enabled) {
    if (diagnosticsEnabled_ == enabled) {
        return;
    }
    diagnosticsEnabled_ = enabled;
    if (enabled) {
        for (auto& cpuReferenceMsByProfile : gpuCompactionCpuReferenceMsByFrame_) {
            cpuReferenceMsByProfile.fill(0.0);
        }
        gpuCompactionPerformanceGates_.fill({});
        diagnosticsTimingInitialized_ = false;
        diagnosticsFpsWindowMs_ = 0.0;
        diagnosticsFpsWindowFrames_ = 0;
        diagnostics_.frameRenderMs = 0.0;
        diagnostics_.averageFrameRenderMs = 0.0;
        diagnostics_.minFrameRenderMs = 0.0;
        diagnostics_.maxFrameRenderMs = 0.0;
        diagnostics_.frameFps = 0.0;
        diagnostics_.averageFrameFps = 0.0;
        diagnostics_.frameAverageWindowSeconds = 0.5;
        diagnostics_.frameUiRenderMs = 0.0;
        diagnostics_.frameFenceWaitMs = 0.0;
        diagnostics_.frameAcquireMs = 0.0;
        diagnostics_.frameImageWaitMs = 0.0;
        diagnostics_.framePrepareMs = 0.0;
        diagnostics_.frameCommandBufferMs = 0.0;
        diagnostics_.frameSubmitMs = 0.0;
        diagnostics_.framePresentMs = 0.0;
        diagnostics_.framePlatformWindowsMs = 0.0;
        diagnostics_.gpuTimestampSupported = gpuTimestampsSupported_;
        diagnostics_.gpuTimestampTimingValid = false;
        diagnostics_.gpuTimestampState =
            gpuTimestampsSupported_ ? "supported; waiting for first scene timing" : "unavailable";
        diagnostics_.gpuFastBasicPointPassMs = 0.0;
        diagnostics_.gpuBeautyDepthPassMs = 0.0;
        diagnostics_.gpuBeautyPointPassMs = 0.0;
        diagnostics_.gpuCompositePassMs = 0.0;
        diagnostics_.gpuPostProcessPassMs = 0.0;
        diagnostics_.adaptiveSelectionExecutionPath = "cpu";
        diagnostics_.adaptiveSelectionFallbackReason = "GPU-driven selection has not been evaluated";
        diagnostics_.adaptiveSelectionParityStatus = "not checked";
        diagnostics_.adaptiveGpuSelectionSupported = false;
        diagnostics_.adaptiveGpuSelectionBeneficial = false;
        diagnostics_.adaptiveGpuCompactionSupported =
            gpuDrivenSelectionCapabilities_.computeQueueSupported &&
            gpuDrawItemCompactionPipeline_ != VK_NULL_HANDLE;
        diagnostics_.adaptiveGpuCompactionUsed = false;
        diagnostics_.adaptiveGpuCompactionParityStatus = "not checked";
        diagnostics_.adaptiveGpuCompactionDispatches = 0;
        diagnostics_.adaptiveGpuCompactionCpuCount = 0;
        diagnostics_.adaptiveGpuCompactionGpuCount = 0;
        diagnostics_.adaptiveGpuCompactionInputDrawItems = 0;
        diagnostics_.adaptiveGpuCompactionDispatchedDrawItems = 0;
        diagnostics_.adaptiveGpuCompactionSelectionLimit = 0;
        diagnostics_.adaptiveGpuCompactionSelectionProfileMask = 0;
        diagnostics_.adaptiveGpuCompactionSelectionClassMask = 0;
        diagnostics_.adaptiveGpuCompactionSelectionRankLimit = 0;
        diagnostics_.adaptiveGpuCompactionSelectionMinDepth = 0;
        diagnostics_.adaptiveGpuCompactionSelectionMaxDepth = 0;
        diagnostics_.adaptiveGpuCompactionSelectionRequiredFlags = 0;
        diagnostics_.adaptiveGpuCompactionSelectionRejectedFlags = 0;
        diagnostics_.adaptiveGpuCompactionSelectionMinFootprintAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMaxFootprintAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMinRenderAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMaxRenderAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMinOpacityCompensation = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMaxOpacityCompensation = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMinEmissionCompensation = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMaxEmissionCompensation = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionMinRepresentedSourceCount = 0;
        diagnostics_.adaptiveGpuCompactionSelectionMaxRepresentedSourceCount = 0;
        diagnostics_.adaptiveGpuCompactionSelectionPositionCount = 0;
        diagnostics_.adaptiveGpuCompactionSelectionFrustumGuardBand = 0.0F;
        diagnostics_.adaptiveGpuCompactionSelectionFrustumEnabled = false;
        diagnostics_.adaptiveGpuCompactionSelectionFrustumFallbackReason.clear();
        diagnostics_.adaptiveGpuCompactionOutputWriteEnabled = false;
        diagnostics_.adaptiveGpuCompactionOutputWriteFallbackReason.clear();
        diagnostics_.adaptiveGpuCompactionOutputCapacity = 0;
        diagnostics_.adaptiveGpuCompactionCopiedDrawItems = 0;
        diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuCompactionOutputProbeCpuCount = 0;
        diagnostics_.adaptiveGpuCompactionOutputProbeGpuCount = 0;
        diagnostics_.adaptiveGpuCompactionOutputProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuCompactionOutputProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuCompactionOutputProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuCompactionOutputProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeUsed = false;
        diagnostics_.adaptiveGpuFeatureClassProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuFeatureClassProbeDispatches = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeMask = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeCpuCount = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeGpuCount = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuFeatureClassProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuFeatureClassProbeMs = 0.0;
        diagnostics_.adaptiveGpuRankProbeUsed = false;
        diagnostics_.adaptiveGpuRankProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuRankProbeDispatches = 0;
        diagnostics_.adaptiveGpuRankProbeLimit = 0;
        diagnostics_.adaptiveGpuRankProbeCpuCount = 0;
        diagnostics_.adaptiveGpuRankProbeGpuCount = 0;
        diagnostics_.adaptiveGpuRankProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuRankProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuRankProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuRankProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuRankProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuRankProbeMs = 0.0;
        diagnostics_.adaptiveGpuDepthProbeUsed = false;
        diagnostics_.adaptiveGpuDepthProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuDepthProbeDispatches = 0;
        diagnostics_.adaptiveGpuDepthProbeMinDepth = 0;
        diagnostics_.adaptiveGpuDepthProbeMaxDepth = 0;
        diagnostics_.adaptiveGpuDepthProbeCpuCount = 0;
        diagnostics_.adaptiveGpuDepthProbeGpuCount = 0;
        diagnostics_.adaptiveGpuDepthProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuDepthProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuDepthProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuDepthProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuDepthProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuDepthProbeMs = 0.0;
        diagnostics_.adaptiveGpuProjectedAreaProbeUsed = false;
        diagnostics_.adaptiveGpuProjectedAreaProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuProjectedAreaProbeDispatches = 0;
        diagnostics_.adaptiveGpuProjectedAreaProbeMinFootprintAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuProjectedAreaProbeMaxFootprintAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuProjectedAreaProbeMinRenderAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuProjectedAreaProbeMaxRenderAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuCount = 0;
        diagnostics_.adaptiveGpuProjectedAreaProbeGpuCount = 0;
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuProjectedAreaProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuProjectedAreaProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuProjectedAreaProbeMs = 0.0;
        diagnostics_.adaptiveGpuRenderAreaProbeUsed = false;
        diagnostics_.adaptiveGpuRenderAreaProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuRenderAreaProbeDispatches = 0;
        diagnostics_.adaptiveGpuRenderAreaProbeMinFootprintAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuRenderAreaProbeMaxFootprintAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuRenderAreaProbeMinRenderAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuRenderAreaProbeMaxRenderAreaPixels = 0.0F;
        diagnostics_.adaptiveGpuRenderAreaProbeCpuCount = 0;
        diagnostics_.adaptiveGpuRenderAreaProbeGpuCount = 0;
        diagnostics_.adaptiveGpuRenderAreaProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuRenderAreaProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuRenderAreaProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuRenderAreaProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuRenderAreaProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuRenderAreaProbeMs = 0.0;
        diagnostics_.adaptiveGpuRepresentedCountProbeUsed = false;
        diagnostics_.adaptiveGpuRepresentedCountProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuRepresentedCountProbeDispatches = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeMinRepresentedSourceCount = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeMaxRepresentedSourceCount = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuCount = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeGpuCount = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuRepresentedCountProbeMs = 0.0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeUsed = false;
        diagnostics_.adaptiveGpuCoverageCompensationProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuCoverageCompensationProbeDispatches = 0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeMinOpacityCompensation = 0.0F;
        diagnostics_.adaptiveGpuCoverageCompensationProbeMaxOpacityCompensation = 0.0F;
        diagnostics_.adaptiveGpuCoverageCompensationProbeMinEmissionCompensation = 0.0F;
        diagnostics_.adaptiveGpuCoverageCompensationProbeMaxEmissionCompensation = 0.0F;
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuCount = 0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeGpuCount = 0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeMs = 0.0;
        diagnostics_.adaptiveGpuClampFlagsProbeUsed = false;
        diagnostics_.adaptiveGpuClampFlagsProbeParityStatus = "not checked";
        diagnostics_.adaptiveGpuClampFlagsProbeDispatches = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeRequiredFlags = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeRejectedFlags = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeCpuCount = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeGpuCount = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeCpuChecksum = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeGpuChecksum = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuClampFlagsProbeCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuClampFlagsProbeMs = 0.0;
        diagnostics_.adaptiveGpuCompactionSubmissionEligible = false;
        diagnostics_.adaptiveGpuCompactionSubmissionUsed = false;
        diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason.clear();
        diagnostics_.adaptiveGpuCompactionSubmissionCandidateVertices = 0;
        diagnostics_.adaptiveGpuCompactionSubmissionReferenceVertices = 0;
        diagnostics_.adaptiveGpuCompactionCpuChecksum = 0;
        diagnostics_.adaptiveGpuCompactionGpuChecksum = 0;
        diagnostics_.adaptiveGpuCompactionCpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuCompactionGpuSourceFingerprint = 0;
        diagnostics_.adaptiveGpuCompactionCpuClassCounts = {};
        diagnostics_.adaptiveGpuCompactionGpuClassCounts = {};
        diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason.clear();
        diagnostics_.adaptiveGpuCompactionPerformanceSlowFrames = 0;
        diagnostics_.adaptiveGpuCompactionPerformanceRetryFrames = 0;
        diagnostics_.adaptiveIndirectDrawSupported = gpuDrivenSelectionCapabilities_.indirectDrawSupported;
        diagnostics_.adaptiveIndirectCountSupported = gpuDrivenSelectionCapabilities_.indirectCountSupported;
        diagnostics_.adaptiveIndirectDrawRecommended = false;
        diagnostics_.adaptiveIndirectDrawUsed = false;
        diagnostics_.adaptiveGpuIndirectCommandSupported =
            gpuDrivenSelectionCapabilities_.computeQueueSupported &&
            gpuDrivenIndirectCommandPipeline_ != VK_NULL_HANDLE;
        diagnostics_.adaptiveGpuIndirectCommandUsed = false;
        diagnostics_.adaptiveGpuCompactionIndirectCommandSupported =
            gpuDrivenSelectionCapabilities_.computeQueueSupported &&
            gpuDrivenIndirectCommandPipeline_ != VK_NULL_HANDLE &&
            gpuDrawItemCompactionPipeline_ != VK_NULL_HANDLE;
        diagnostics_.adaptiveGpuCompactionIndirectCommandUsed = false;
        diagnostics_.adaptiveGpuCompactionIndirectCommandParityStatus = "not checked";
        diagnostics_.adaptiveGpuCompactionIndirectCommandDispatches = 0;
        diagnostics_.adaptiveGpuCompactionIndirectCommandCpuVertices = 0;
        diagnostics_.adaptiveGpuCompactionIndirectCommandGpuVertices = 0;
        diagnostics_.adaptiveGpuIndirectCommandDispatches = 0;
        diagnostics_.adaptiveIndirectDrawCalls = 0;
        diagnostics_.adaptiveIndirectDrawCount = 0;
        diagnostics_.adaptiveIndirectSubmittedVertices = 0;
        diagnostics_.adaptiveGpuSelectedRepresentativeCount = 0;
        diagnostics_.adaptiveGpuSelectionMs = 0.0;
        diagnostics_.adaptiveGpuCompactionCpuReferenceMs = 0.0;
        diagnostics_.adaptiveGpuCompactionMs = 0.0;
        diagnostics_.adaptiveGpuFeatureClassProbeMs = 0.0;
        diagnostics_.adaptiveGpuRankProbeMs = 0.0;
        diagnostics_.adaptiveGpuDepthProbeMs = 0.0;
        diagnostics_.adaptiveGpuProjectedAreaProbeMs = 0.0;
        diagnostics_.adaptiveGpuRenderAreaProbeMs = 0.0;
        diagnostics_.adaptiveGpuRepresentedCountProbeMs = 0.0;
        diagnostics_.adaptiveGpuCoverageCompensationProbeMs = 0.0;
        diagnostics_.adaptiveGpuClampFlagsProbeMs = 0.0;
        diagnostics_.adaptiveGpuIndirectCommandMs = 0.0;
    }
}

void VulkanViewportShell::SetSceneCachingEnabled(bool enabled) {
    if (sceneCachingEnabled_ == enabled) {
        return;
    }
    sceneCachingEnabled_ = enabled;
    if (!sceneCachingEnabled_) {
        ++sceneRevision_;
    }
}

void VulkanViewportShell::UpdateRenderState(const SceneRenderState& state) {
    renderState_ = state;
    ++sceneRevision_;

    std::uint64_t pointCount = 0;
    std::uint64_t adaptiveRepresentativeCount = 0;
    std::uint64_t adaptiveRepresentedSourceCount = 0;
    std::uint64_t adaptiveVisibleRepresentedSourceCount = 0;
    std::uint64_t adaptiveEmittedRepresentedSourceCount = 0;
    std::uint64_t adaptiveCulledRepresentedSourceCount = 0;
    std::uint32_t adaptiveVisibleFrontierNodeCount = 0;
    std::int64_t adaptiveRepresentativeDelta = 0;
    std::uint32_t adaptivePromotedNodeCount = 0;
    std::uint32_t adaptiveDemotedNodeCount = 0;
    std::uint32_t adaptiveHysteresisKeptNodeCount = 0;
    std::array<std::uint32_t, renderer::pointcloud::kPointCloudLodRepresentativeClassCount> adaptiveEmittedClassCounts{};
    std::uint32_t adaptiveColorFeatureRefinedNodeCount = 0;
    std::uint32_t adaptiveScalarFeatureRefinedNodeCount = 0;
    std::uint32_t adaptiveNormalFeatureRefinedNodeCount = 0;
    std::uint32_t adaptiveEmissiveFeatureRefinedNodeCount = 0;
    float adaptiveHysteresisPromoteScale = 1.0F;
    float adaptiveHysteresisDemoteScale = 1.0F;
    std::uint32_t adaptiveActiveTransitionCount = 0;
    double adaptiveTransitionAgeMsSum = 0.0;
    double adaptiveMaxTransitionAgeMs = 0.0;
    bool adaptiveIdleRefinementPending = false;
    double adaptiveEstimatedFragments = 0.0;
    double adaptiveFragmentBudget = 0.0;
    std::uint32_t adaptiveRepresentativeBudget = 0;
    bool adaptiveRepresentativeBudgetReached = false;
    bool adaptiveFragmentBudgetReached = false;
    bool adaptiveBlendedFragmentBudgetReached = false;
    renderer::pointcloud::PointCloudLodRendererCostProfile adaptiveRendererCostProfile =
        renderer::pointcloud::PointCloudLodRendererCostProfile::FastBasicSquare;
    bool adaptiveHasCostProfile = false;
    float adaptiveMinRadiusScale = 1.0F;
    float adaptiveMaxRadiusScale = 1.0F;
    float adaptiveMinOpacityCoverageScale = 1.0F;
    float adaptiveMaxOpacityCoverageScale = 1.0F;
    float adaptiveMinEmissionCoverageScale = 1.0F;
    float adaptiveMaxEmissionCoverageScale = 1.0F;
    double adaptiveEstimatedVertexCost = 0.0;
    double adaptiveEstimatedBlendedFragments = 0.0;
    double adaptiveBlendedFragmentBudget = 0.0;
    bool adaptiveTileBudgetEnabled = false;
    std::uint32_t adaptiveTileSizePixels = 0;
    std::uint32_t adaptiveTileCount = 0;
    double adaptiveTileFragmentBudget = 0.0;
    double adaptiveTileBlendedFragmentBudget = 0.0;
    double adaptiveMaxTileEstimatedFragments = 0.0;
    double adaptiveMaxTileEstimatedBlendedFragments = 0.0;
    std::uint32_t adaptiveOverBudgetTileCount = 0;
    double adaptiveOverBudgetTileScreenPercent = 0.0;
    std::uint32_t adaptiveTileLimitedRepresentativeCount = 0;
    std::uint32_t adaptiveTileLimitedNodeCount = 0;
    std::uint32_t adaptiveTilePreservedRepresentativeCount = 0;
    bool adaptiveOcclusionCullingEnabled = false;
    std::string adaptiveOcclusionCullingState = "disabled";
    std::string adaptiveOcclusionCullingDisabledReason = "not requested";
    std::uint32_t adaptiveOcclusionRejectedNodeCount = 0;
    std::uint64_t adaptiveOcclusionRejectedRepresentedSourceCount = 0;
    bool adaptiveOpacityCompensationClamped = false;
    bool adaptiveEmissionCompensationClamped = false;
    bool adaptivePerformanceCompensationClamped = false;
    double adaptiveGovernorBudgetScale = 1.0;
    double adaptiveGovernorPointPassMs = 0.0;
    double adaptiveGovernorPointPassEwmaMs = 0.0;
    double adaptiveGovernorCompositeMs = 0.0;
    double adaptiveGovernorCompositeEwmaMs = 0.0;
    double adaptiveGovernorTargetPointPassMs = 0.0;
    std::uint64_t adaptiveGovernorUploadBudgetBytes = 0;
    std::string adaptiveGovernorTimestampState;
    std::string adaptiveGovernorStatus;
    std::string adaptiveGovernorActiveLimit;
    double adaptiveTraversalMs = 0.0;
    bool adaptiveReusedPrevious = false;
    bool adaptiveRuntimeCacheHit = false;
    bool adaptiveAsyncPending = false;
    double adaptiveAsyncPendingAgeMs = 0.0;
    std::uint64_t adaptiveDisplayedCacheAgeFrames = 0;
    std::uint64_t adaptiveStaleTraversalDiscardedCount = 0;
    std::uint64_t adaptiveDrawItemBytes = 0;
    std::string adaptivePersistentCacheStatus;
    std::string adaptiveRuntimeStatus;
    std::string adaptiveRequestedDensity;
    std::string adaptiveDisplayedDensity;
    std::string adaptiveFallbackState;
    double pointSizeSum = 0.0;
    std::uint64_t pointSizeWeight = 0;
    for (const auto& layer : renderState_.pointCloudLayers) {
        pointCount += layer.drawPointCount;
        if (layer.useAdaptiveDrawItems) {
            adaptiveRepresentativeCount += layer.drawPointCount;
            adaptiveRepresentedSourceCount += layer.adaptiveRepresentedSourceCount;
            adaptiveVisibleRepresentedSourceCount += layer.adaptiveVisibleRepresentedSourceCount;
            adaptiveEmittedRepresentedSourceCount += layer.adaptiveEmittedRepresentedSourceCount;
            adaptiveCulledRepresentedSourceCount += layer.adaptiveCulledRepresentedSourceCount;
            adaptiveVisibleFrontierNodeCount += layer.adaptiveVisibleFrontierNodeCount;
            adaptiveRepresentativeDelta += layer.adaptiveRepresentativeDelta;
            adaptivePromotedNodeCount += layer.adaptivePromotedNodeCount;
            adaptiveDemotedNodeCount += layer.adaptiveDemotedNodeCount;
            adaptiveHysteresisKeptNodeCount += layer.adaptiveHysteresisKeptNodeCount;
            for (std::size_t classIndex = 0; classIndex < adaptiveEmittedClassCounts.size(); ++classIndex) {
                adaptiveEmittedClassCounts[classIndex] += layer.adaptiveEmittedClassCounts[classIndex];
            }
            adaptiveColorFeatureRefinedNodeCount += layer.adaptiveColorFeatureRefinedNodeCount;
            adaptiveScalarFeatureRefinedNodeCount += layer.adaptiveScalarFeatureRefinedNodeCount;
            adaptiveNormalFeatureRefinedNodeCount += layer.adaptiveNormalFeatureRefinedNodeCount;
            adaptiveEmissiveFeatureRefinedNodeCount += layer.adaptiveEmissiveFeatureRefinedNodeCount;
            adaptiveHysteresisPromoteScale = std::max(
                adaptiveHysteresisPromoteScale,
                layer.adaptiveHysteresisPromoteScale);
            adaptiveHysteresisDemoteScale = std::min(
                adaptiveHysteresisDemoteScale,
                layer.adaptiveHysteresisDemoteScale);
            adaptiveActiveTransitionCount += layer.adaptiveActiveTransitionCount;
            adaptiveTransitionAgeMsSum +=
                layer.adaptiveAverageTransitionAgeMs * static_cast<double>(layer.adaptiveActiveTransitionCount);
            adaptiveMaxTransitionAgeMs = std::max(adaptiveMaxTransitionAgeMs, layer.adaptiveMaxTransitionAgeMs);
            adaptiveIdleRefinementPending =
                adaptiveIdleRefinementPending || layer.adaptiveIdleRefinementPending;
            adaptiveDrawItemBytes += layer.adaptiveDrawItemBytes;
        }
        adaptiveEstimatedFragments += layer.adaptiveEstimatedFragments;
        adaptiveFragmentBudget += layer.adaptiveFragmentBudget;
        adaptiveBlendedFragmentBudget += layer.adaptiveBlendedFragmentBudget;
        adaptiveRepresentativeBudget += layer.adaptiveRepresentativeBudget;
        adaptiveRepresentativeBudgetReached =
            adaptiveRepresentativeBudgetReached || layer.adaptiveRepresentativeBudgetReached;
        adaptiveFragmentBudgetReached = adaptiveFragmentBudgetReached || layer.adaptiveFragmentBudgetReached;
        adaptiveBlendedFragmentBudgetReached =
            adaptiveBlendedFragmentBudgetReached || layer.adaptiveBlendedFragmentBudgetReached;
        if (layer.useAdaptiveDrawItems) {
            if (!adaptiveHasCostProfile) {
                adaptiveRendererCostProfile = layer.adaptiveRendererCostProfile;
                adaptiveMinRadiusScale = layer.adaptiveMinRadiusScale;
                adaptiveMaxRadiusScale = layer.adaptiveMaxRadiusScale;
                adaptiveMinOpacityCoverageScale = layer.adaptiveMinOpacityCoverageScale;
                adaptiveMaxOpacityCoverageScale = layer.adaptiveMaxOpacityCoverageScale;
                adaptiveMinEmissionCoverageScale = layer.adaptiveMinEmissionCoverageScale;
                adaptiveMaxEmissionCoverageScale = layer.adaptiveMaxEmissionCoverageScale;
                adaptiveHasCostProfile = true;
            } else {
                adaptiveMinRadiusScale = std::min(adaptiveMinRadiusScale, layer.adaptiveMinRadiusScale);
                adaptiveMaxRadiusScale = std::max(adaptiveMaxRadiusScale, layer.adaptiveMaxRadiusScale);
                adaptiveMinOpacityCoverageScale =
                    std::min(adaptiveMinOpacityCoverageScale, layer.adaptiveMinOpacityCoverageScale);
                adaptiveMaxOpacityCoverageScale =
                    std::max(adaptiveMaxOpacityCoverageScale, layer.adaptiveMaxOpacityCoverageScale);
                adaptiveMinEmissionCoverageScale =
                    std::min(adaptiveMinEmissionCoverageScale, layer.adaptiveMinEmissionCoverageScale);
                adaptiveMaxEmissionCoverageScale =
                    std::max(adaptiveMaxEmissionCoverageScale, layer.adaptiveMaxEmissionCoverageScale);
            }
            adaptiveEstimatedVertexCost += layer.adaptiveEstimatedVertexCost;
            adaptiveEstimatedBlendedFragments += layer.adaptiveEstimatedBlendedFragments;
            adaptiveTileBudgetEnabled = adaptiveTileBudgetEnabled || layer.adaptiveTileBudgetEnabled;
            adaptiveTileSizePixels = std::max(adaptiveTileSizePixels, layer.adaptiveTileSizePixels);
            adaptiveTileCount = std::max(adaptiveTileCount, layer.adaptiveTileCount);
            adaptiveTileFragmentBudget =
                std::max(adaptiveTileFragmentBudget, static_cast<double>(layer.adaptiveTileFragmentBudget));
            adaptiveTileBlendedFragmentBudget =
                std::max(adaptiveTileBlendedFragmentBudget, static_cast<double>(layer.adaptiveTileBlendedFragmentBudget));
            adaptiveMaxTileEstimatedFragments =
                std::max(adaptiveMaxTileEstimatedFragments, static_cast<double>(layer.adaptiveMaxTileEstimatedFragments));
            adaptiveMaxTileEstimatedBlendedFragments = std::max(
                adaptiveMaxTileEstimatedBlendedFragments,
                static_cast<double>(layer.adaptiveMaxTileEstimatedBlendedFragments));
            adaptiveOverBudgetTileCount += layer.adaptiveOverBudgetTileCount;
            adaptiveOverBudgetTileScreenPercent = std::max(
                adaptiveOverBudgetTileScreenPercent,
                static_cast<double>(layer.adaptiveOverBudgetTileScreenPercent));
            adaptiveTileLimitedRepresentativeCount += layer.adaptiveTileLimitedRepresentativeCount;
            adaptiveTileLimitedNodeCount += layer.adaptiveTileLimitedNodeCount;
            adaptiveTilePreservedRepresentativeCount += layer.adaptiveTilePreservedRepresentativeCount;
            adaptiveOcclusionCullingEnabled =
                adaptiveOcclusionCullingEnabled || layer.adaptiveOcclusionCullingEnabled;
            if (layer.adaptiveOcclusionCullingState == "active" ||
                (layer.adaptiveOcclusionCullingState == "uncertain" &&
                 adaptiveOcclusionCullingState != "active") ||
                (adaptiveOcclusionCullingState == "disabled" &&
                 !layer.adaptiveOcclusionCullingState.empty())) {
                adaptiveOcclusionCullingState = layer.adaptiveOcclusionCullingState;
                adaptiveOcclusionCullingDisabledReason = layer.adaptiveOcclusionCullingDisabledReason;
            }
            adaptiveOcclusionRejectedNodeCount += layer.adaptiveOcclusionRejectedNodeCount;
            adaptiveOcclusionRejectedRepresentedSourceCount += layer.adaptiveOcclusionRejectedRepresentedSourceCount;
            adaptiveGovernorBudgetScale = std::min(
                adaptiveGovernorBudgetScale,
                static_cast<double>(layer.adaptiveGovernorBudgetScale));
            adaptiveGovernorPointPassMs =
                std::max(adaptiveGovernorPointPassMs, layer.adaptiveGovernorPointPassMs);
            adaptiveGovernorPointPassEwmaMs =
                std::max(adaptiveGovernorPointPassEwmaMs, layer.adaptiveGovernorPointPassEwmaMs);
            adaptiveGovernorCompositeMs =
                std::max(adaptiveGovernorCompositeMs, layer.adaptiveGovernorCompositeMs);
            adaptiveGovernorCompositeEwmaMs =
                std::max(adaptiveGovernorCompositeEwmaMs, layer.adaptiveGovernorCompositeEwmaMs);
            adaptiveGovernorTargetPointPassMs =
                std::max(adaptiveGovernorTargetPointPassMs, layer.adaptiveGovernorTargetPointPassMs);
            adaptiveGovernorUploadBudgetBytes =
                std::max(adaptiveGovernorUploadBudgetBytes, layer.adaptiveGovernorUploadBudgetBytes);
            if (!layer.adaptiveGovernorTimestampState.empty()) {
                adaptiveGovernorTimestampState = layer.adaptiveGovernorTimestampState;
            }
            if (!layer.adaptiveGovernorStatus.empty()) {
                adaptiveGovernorStatus = layer.adaptiveGovernorStatus;
            }
            if (!layer.adaptiveGovernorActiveLimit.empty()) {
                adaptiveGovernorActiveLimit = layer.adaptiveGovernorActiveLimit;
            }
            adaptiveOpacityCompensationClamped =
                adaptiveOpacityCompensationClamped || layer.adaptiveOpacityCompensationClamped;
            adaptiveEmissionCompensationClamped =
                adaptiveEmissionCompensationClamped || layer.adaptiveEmissionCompensationClamped;
            adaptivePerformanceCompensationClamped =
                adaptivePerformanceCompensationClamped || layer.adaptivePerformanceCompensationClamped;
        }
        adaptiveTraversalMs += layer.adaptiveLodTraversalMs;
        adaptiveReusedPrevious = adaptiveReusedPrevious || layer.adaptiveLodReusedPrevious;
        adaptiveRuntimeCacheHit = adaptiveRuntimeCacheHit || layer.adaptiveLodRuntimeCacheHit;
        adaptiveAsyncPending = adaptiveAsyncPending || layer.adaptiveLodAsyncPending;
        adaptiveAsyncPendingAgeMs = std::max(adaptiveAsyncPendingAgeMs, layer.adaptiveLodAsyncPendingAgeMs);
        adaptiveDisplayedCacheAgeFrames = std::max(
            adaptiveDisplayedCacheAgeFrames,
            layer.adaptiveLodDisplayedCacheAgeFrames);
        adaptiveStaleTraversalDiscardedCount = std::max(
            adaptiveStaleTraversalDiscardedCount,
            layer.adaptiveLodStaleTraversalDiscardedCount);
        if (!layer.adaptiveLodPersistentCacheStatus.empty()) {
            adaptivePersistentCacheStatus = layer.adaptiveLodPersistentCacheStatus;
        }
        if (!layer.adaptiveLodRuntimeStatus.empty()) {
            adaptiveRuntimeStatus = layer.adaptiveLodRuntimeStatus;
        }
        if (!layer.adaptiveLodRequestedDensity.empty()) {
            adaptiveRequestedDensity = layer.adaptiveLodRequestedDensity;
        }
        if (!layer.adaptiveLodDisplayedDensity.empty()) {
            adaptiveDisplayedDensity = layer.adaptiveLodDisplayedDensity;
        }
        if (!layer.adaptiveLodFallbackState.empty()) {
            adaptiveFallbackState = layer.adaptiveLodFallbackState;
        }
        pointSizeSum +=
            static_cast<double>(layer.style.pointSize.constantValue[0] * renderState_.pointSizeScale) *
            static_cast<double>(std::max<std::uint32_t>(1U, layer.drawPointCount));
        pointSizeWeight += std::max<std::uint32_t>(1U, layer.drawPointCount);
    }

    diagnostics_.pointCount = pointCount;
    diagnostics_.adaptiveRepresentativeCount = adaptiveRepresentativeCount;
    diagnostics_.adaptiveRepresentedSourceCount = adaptiveRepresentedSourceCount;
    diagnostics_.adaptiveVisibleRepresentedSourceCount = adaptiveVisibleRepresentedSourceCount;
    diagnostics_.adaptiveEmittedRepresentedSourceCount = adaptiveEmittedRepresentedSourceCount;
    diagnostics_.adaptiveCulledRepresentedSourceCount = adaptiveCulledRepresentedSourceCount;
    diagnostics_.adaptiveVisibleFrontierNodeCount = adaptiveVisibleFrontierNodeCount;
    diagnostics_.adaptiveRepresentativeDelta = adaptiveRepresentativeDelta;
    diagnostics_.adaptivePromotedNodeCount = adaptivePromotedNodeCount;
    diagnostics_.adaptiveDemotedNodeCount = adaptiveDemotedNodeCount;
    diagnostics_.adaptiveHysteresisKeptNodeCount = adaptiveHysteresisKeptNodeCount;
    diagnostics_.adaptiveEmittedClassCounts = adaptiveEmittedClassCounts;
    diagnostics_.adaptiveColorFeatureRefinedNodeCount = adaptiveColorFeatureRefinedNodeCount;
    diagnostics_.adaptiveScalarFeatureRefinedNodeCount = adaptiveScalarFeatureRefinedNodeCount;
    diagnostics_.adaptiveNormalFeatureRefinedNodeCount = adaptiveNormalFeatureRefinedNodeCount;
    diagnostics_.adaptiveEmissiveFeatureRefinedNodeCount = adaptiveEmissiveFeatureRefinedNodeCount;
    diagnostics_.adaptiveHysteresisPromoteScale = adaptiveHysteresisPromoteScale;
    diagnostics_.adaptiveHysteresisDemoteScale = adaptiveHysteresisDemoteScale;
    diagnostics_.adaptiveActiveTransitionCount = adaptiveActiveTransitionCount;
    diagnostics_.adaptiveAverageTransitionAgeMs =
        adaptiveActiveTransitionCount > 0U
            ? adaptiveTransitionAgeMsSum / static_cast<double>(adaptiveActiveTransitionCount)
            : 0.0;
    diagnostics_.adaptiveMaxTransitionAgeMs = adaptiveMaxTransitionAgeMs;
    diagnostics_.adaptiveIdleRefinementPending = adaptiveIdleRefinementPending;
    diagnostics_.adaptiveEstimatedFragments = adaptiveEstimatedFragments;
    diagnostics_.adaptiveFragmentBudget = adaptiveFragmentBudget;
    diagnostics_.adaptiveRepresentativeBudget = adaptiveRepresentativeBudget;
    diagnostics_.adaptiveRepresentativeBudgetReached = adaptiveRepresentativeBudgetReached;
    diagnostics_.adaptiveFragmentBudgetReached = adaptiveFragmentBudgetReached;
    diagnostics_.adaptiveBlendedFragmentBudgetReached = adaptiveBlendedFragmentBudgetReached;
    diagnostics_.adaptiveRendererCostProfile = adaptiveRendererCostProfile;
    diagnostics_.adaptiveMinRadiusScale = adaptiveMinRadiusScale;
    diagnostics_.adaptiveMaxRadiusScale = adaptiveMaxRadiusScale;
    diagnostics_.adaptiveMinOpacityCoverageScale = adaptiveMinOpacityCoverageScale;
    diagnostics_.adaptiveMaxOpacityCoverageScale = adaptiveMaxOpacityCoverageScale;
    diagnostics_.adaptiveMinEmissionCoverageScale = adaptiveMinEmissionCoverageScale;
    diagnostics_.adaptiveMaxEmissionCoverageScale = adaptiveMaxEmissionCoverageScale;
    diagnostics_.adaptiveEstimatedVertexCost = adaptiveEstimatedVertexCost;
    diagnostics_.adaptiveEstimatedBlendedFragments = adaptiveEstimatedBlendedFragments;
    diagnostics_.adaptiveBlendedFragmentBudget = adaptiveBlendedFragmentBudget;
    diagnostics_.adaptiveTileBudgetEnabled = adaptiveTileBudgetEnabled;
    diagnostics_.adaptiveTileSizePixels = adaptiveTileSizePixels;
    diagnostics_.adaptiveTileCount = adaptiveTileCount;
    diagnostics_.adaptiveTileFragmentBudget = adaptiveTileFragmentBudget;
    diagnostics_.adaptiveTileBlendedFragmentBudget = adaptiveTileBlendedFragmentBudget;
    diagnostics_.adaptiveMaxTileEstimatedFragments = adaptiveMaxTileEstimatedFragments;
    diagnostics_.adaptiveMaxTileEstimatedBlendedFragments = adaptiveMaxTileEstimatedBlendedFragments;
    diagnostics_.adaptiveOverBudgetTileCount = adaptiveOverBudgetTileCount;
    diagnostics_.adaptiveOverBudgetTileScreenPercent = adaptiveOverBudgetTileScreenPercent;
    diagnostics_.adaptiveTileLimitedRepresentativeCount = adaptiveTileLimitedRepresentativeCount;
    diagnostics_.adaptiveTileLimitedNodeCount = adaptiveTileLimitedNodeCount;
    diagnostics_.adaptiveTilePreservedRepresentativeCount = adaptiveTilePreservedRepresentativeCount;
    diagnostics_.adaptiveOcclusionCullingEnabled = adaptiveOcclusionCullingEnabled;
    diagnostics_.adaptiveOcclusionCullingState = std::move(adaptiveOcclusionCullingState);
    diagnostics_.adaptiveOcclusionCullingDisabledReason = std::move(adaptiveOcclusionCullingDisabledReason);
    diagnostics_.adaptiveOcclusionRejectedNodeCount = adaptiveOcclusionRejectedNodeCount;
    diagnostics_.adaptiveOcclusionRejectedRepresentedSourceCount = adaptiveOcclusionRejectedRepresentedSourceCount;
    diagnostics_.adaptiveOpacityCompensationClamped = adaptiveOpacityCompensationClamped;
    diagnostics_.adaptiveEmissionCompensationClamped = adaptiveEmissionCompensationClamped;
    diagnostics_.adaptivePerformanceCompensationClamped = adaptivePerformanceCompensationClamped;
    diagnostics_.adaptiveGovernorBudgetScale = adaptiveGovernorBudgetScale;
    diagnostics_.adaptiveGovernorPointPassMs = adaptiveGovernorPointPassMs;
    diagnostics_.adaptiveGovernorPointPassEwmaMs = adaptiveGovernorPointPassEwmaMs;
    diagnostics_.adaptiveGovernorCompositeMs = adaptiveGovernorCompositeMs;
    diagnostics_.adaptiveGovernorCompositeEwmaMs = adaptiveGovernorCompositeEwmaMs;
    diagnostics_.adaptiveGovernorTargetPointPassMs = adaptiveGovernorTargetPointPassMs;
    diagnostics_.adaptiveGovernorUploadBudgetBytes = adaptiveGovernorUploadBudgetBytes;
    diagnostics_.adaptiveGovernorTimestampState = std::move(adaptiveGovernorTimestampState);
    diagnostics_.adaptiveGovernorStatus = std::move(adaptiveGovernorStatus);
    diagnostics_.adaptiveGovernorActiveLimit = std::move(adaptiveGovernorActiveLimit);
    diagnostics_.adaptiveLodTraversalMs = adaptiveTraversalMs;
    diagnostics_.adaptiveLodReusedPrevious = adaptiveReusedPrevious;
    diagnostics_.adaptiveLodRuntimeCacheHit = adaptiveRuntimeCacheHit;
    diagnostics_.adaptiveLodAsyncPending = adaptiveAsyncPending;
    diagnostics_.adaptiveLodAsyncPendingAgeMs = adaptiveAsyncPendingAgeMs;
    diagnostics_.adaptiveLodDisplayedCacheAgeFrames = adaptiveDisplayedCacheAgeFrames;
    diagnostics_.adaptiveLodStaleTraversalDiscardedCount = adaptiveStaleTraversalDiscardedCount;
    diagnostics_.adaptiveGpuIdleWaitCount = 0;
    diagnostics_.adaptiveDrawItemBytes = adaptiveDrawItemBytes;
    const auto gpuSelectionDecision = renderer::pointcloud::EvaluatePointCloudGpuDrivenSelection(
        gpuDrivenSelectionCapabilities_,
        {.hierarchyNodeCount = adaptiveVisibleFrontierNodeCount,
         .representativeCount = static_cast<std::uint32_t>(
             std::min<std::uint64_t>(
                 adaptiveRepresentativeCount,
                 std::numeric_limits<std::uint32_t>::max())),
         .selectedDrawItemCount = static_cast<std::uint32_t>(
             std::min<std::uint64_t>(
                 adaptiveRepresentativeCount,
                 std::numeric_limits<std::uint32_t>::max())),
         .cpuSelectionMs = adaptiveTraversalMs,
         .gpuSelectionMs = 0.0,
         .gpuCompactionMs = 0.0,
         .parityChecked = false,
         .parityPassed = false,
         .allowGpuDrivenSelection = true,
         .allowIndirectDraw = true});
    diagnostics_.adaptiveSelectionExecutionPath = gpuSelectionDecision.selectionPath;
    diagnostics_.adaptiveSelectionFallbackReason = gpuSelectionDecision.fallbackReason;
    diagnostics_.adaptiveSelectionParityStatus = gpuSelectionDecision.parityStatus;
    diagnostics_.adaptiveGpuSelectionSupported = gpuSelectionDecision.computeSelectionSupported;
    diagnostics_.adaptiveGpuSelectionBeneficial = gpuSelectionDecision.computeSelectionBeneficial;
    diagnostics_.adaptiveGpuCompactionSupported =
        gpuDrivenSelectionCapabilities_.computeQueueSupported &&
        gpuDrawItemCompactionPipeline_ != VK_NULL_HANDLE;
    diagnostics_.adaptiveGpuCompactionUsed = false;
    diagnostics_.adaptiveGpuCompactionParityStatus = "not checked";
    diagnostics_.adaptiveGpuCompactionDispatches = 0;
    diagnostics_.adaptiveGpuCompactionCpuCount = 0;
    diagnostics_.adaptiveGpuCompactionGpuCount = 0;
    diagnostics_.adaptiveGpuCompactionInputDrawItems = 0;
    diagnostics_.adaptiveGpuCompactionDispatchedDrawItems = 0;
    diagnostics_.adaptiveGpuCompactionSelectionLimit = 0;
    diagnostics_.adaptiveGpuCompactionSelectionProfileMask = 0;
    diagnostics_.adaptiveGpuCompactionSelectionClassMask = 0;
    diagnostics_.adaptiveGpuCompactionSelectionRankLimit = 0;
    diagnostics_.adaptiveGpuCompactionSelectionMinDepth = 0;
    diagnostics_.adaptiveGpuCompactionSelectionMaxDepth = 0;
    diagnostics_.adaptiveGpuCompactionSelectionRequiredFlags = 0;
    diagnostics_.adaptiveGpuCompactionSelectionRejectedFlags = 0;
    diagnostics_.adaptiveGpuCompactionSelectionMinFootprintAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMaxFootprintAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMinRenderAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMaxRenderAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMinOpacityCompensation = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMaxOpacityCompensation = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMinEmissionCompensation = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMaxEmissionCompensation = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionMinRepresentedSourceCount = 0;
    diagnostics_.adaptiveGpuCompactionSelectionMaxRepresentedSourceCount = 0;
    diagnostics_.adaptiveGpuCompactionSelectionPositionCount = 0;
    diagnostics_.adaptiveGpuCompactionSelectionFrustumGuardBand = 0.0F;
    diagnostics_.adaptiveGpuCompactionSelectionFrustumEnabled = false;
    diagnostics_.adaptiveGpuCompactionSelectionFrustumFallbackReason.clear();
    diagnostics_.adaptiveGpuCompactionOutputWriteEnabled = false;
    diagnostics_.adaptiveGpuCompactionOutputWriteFallbackReason.clear();
    diagnostics_.adaptiveGpuCompactionOutputCapacity = 0;
    diagnostics_.adaptiveGpuCompactionCopiedDrawItems = 0;
    diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuCompactionOutputProbeCpuCount = 0;
    diagnostics_.adaptiveGpuCompactionOutputProbeGpuCount = 0;
    diagnostics_.adaptiveGpuCompactionOutputProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuCompactionOutputProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuCompactionOutputProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuCompactionOutputProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeUsed = false;
    diagnostics_.adaptiveGpuFeatureClassProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuFeatureClassProbeDispatches = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeMask = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeCpuCount = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeGpuCount = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuFeatureClassProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuFeatureClassProbeMs = 0.0;
    diagnostics_.adaptiveGpuRankProbeUsed = false;
    diagnostics_.adaptiveGpuRankProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuRankProbeDispatches = 0;
    diagnostics_.adaptiveGpuRankProbeLimit = 0;
    diagnostics_.adaptiveGpuRankProbeCpuCount = 0;
    diagnostics_.adaptiveGpuRankProbeGpuCount = 0;
    diagnostics_.adaptiveGpuRankProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuRankProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuRankProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuRankProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuRankProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuRankProbeMs = 0.0;
    diagnostics_.adaptiveGpuDepthProbeUsed = false;
    diagnostics_.adaptiveGpuDepthProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuDepthProbeDispatches = 0;
    diagnostics_.adaptiveGpuDepthProbeMinDepth = 0;
    diagnostics_.adaptiveGpuDepthProbeMaxDepth = 0;
    diagnostics_.adaptiveGpuDepthProbeCpuCount = 0;
    diagnostics_.adaptiveGpuDepthProbeGpuCount = 0;
    diagnostics_.adaptiveGpuDepthProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuDepthProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuDepthProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuDepthProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuDepthProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuDepthProbeMs = 0.0;
    diagnostics_.adaptiveGpuProjectedAreaProbeUsed = false;
    diagnostics_.adaptiveGpuProjectedAreaProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuProjectedAreaProbeDispatches = 0;
    diagnostics_.adaptiveGpuProjectedAreaProbeMinFootprintAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuProjectedAreaProbeMaxFootprintAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuProjectedAreaProbeMinRenderAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuProjectedAreaProbeMaxRenderAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuProjectedAreaProbeCpuCount = 0;
    diagnostics_.adaptiveGpuProjectedAreaProbeGpuCount = 0;
    diagnostics_.adaptiveGpuProjectedAreaProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuProjectedAreaProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuProjectedAreaProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuProjectedAreaProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuProjectedAreaProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuProjectedAreaProbeMs = 0.0;
    diagnostics_.adaptiveGpuRenderAreaProbeUsed = false;
    diagnostics_.adaptiveGpuRenderAreaProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuRenderAreaProbeDispatches = 0;
    diagnostics_.adaptiveGpuRenderAreaProbeMinFootprintAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuRenderAreaProbeMaxFootprintAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuRenderAreaProbeMinRenderAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuRenderAreaProbeMaxRenderAreaPixels = 0.0F;
    diagnostics_.adaptiveGpuRenderAreaProbeCpuCount = 0;
    diagnostics_.adaptiveGpuRenderAreaProbeGpuCount = 0;
    diagnostics_.adaptiveGpuRenderAreaProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuRenderAreaProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuRenderAreaProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuRenderAreaProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuRenderAreaProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuRenderAreaProbeMs = 0.0;
    diagnostics_.adaptiveGpuRepresentedCountProbeUsed = false;
    diagnostics_.adaptiveGpuRepresentedCountProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuRepresentedCountProbeDispatches = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeMinRepresentedSourceCount = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeMaxRepresentedSourceCount = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeCpuCount = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeGpuCount = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuRepresentedCountProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuRepresentedCountProbeMs = 0.0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeUsed = false;
    diagnostics_.adaptiveGpuCoverageCompensationProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuCoverageCompensationProbeDispatches = 0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeMinOpacityCompensation = 0.0F;
    diagnostics_.adaptiveGpuCoverageCompensationProbeMaxOpacityCompensation = 0.0F;
    diagnostics_.adaptiveGpuCoverageCompensationProbeMinEmissionCompensation = 0.0F;
    diagnostics_.adaptiveGpuCoverageCompensationProbeMaxEmissionCompensation = 0.0F;
    diagnostics_.adaptiveGpuCoverageCompensationProbeCpuCount = 0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeGpuCount = 0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeMs = 0.0;
    diagnostics_.adaptiveGpuClampFlagsProbeUsed = false;
    diagnostics_.adaptiveGpuClampFlagsProbeParityStatus = "not checked";
    diagnostics_.adaptiveGpuClampFlagsProbeDispatches = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeRequiredFlags = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeRejectedFlags = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeCpuCount = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeGpuCount = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeCpuChecksum = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeGpuChecksum = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuClampFlagsProbeCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuClampFlagsProbeMs = 0.0;
    diagnostics_.adaptiveGpuCompactionSubmissionEligible = false;
    diagnostics_.adaptiveGpuCompactionSubmissionUsed = false;
    diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason.clear();
    diagnostics_.adaptiveGpuCompactionSubmissionCandidateVertices = 0;
    diagnostics_.adaptiveGpuCompactionSubmissionReferenceVertices = 0;
    diagnostics_.adaptiveGpuCompactionCpuChecksum = 0;
    diagnostics_.adaptiveGpuCompactionGpuChecksum = 0;
    diagnostics_.adaptiveGpuCompactionCpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuCompactionGpuSourceFingerprint = 0;
    diagnostics_.adaptiveGpuCompactionCpuClassCounts = {};
    diagnostics_.adaptiveGpuCompactionGpuClassCounts = {};
    diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason.clear();
    diagnostics_.adaptiveGpuCompactionPerformanceSlowFrames = 0;
    diagnostics_.adaptiveGpuCompactionPerformanceRetryFrames = 0;
    diagnostics_.adaptiveIndirectDrawSupported = gpuSelectionDecision.indirectDrawSupported;
    diagnostics_.adaptiveIndirectCountSupported = gpuSelectionDecision.indirectCountSupported;
    diagnostics_.adaptiveIndirectDrawRecommended = gpuSelectionDecision.indirectDrawRecommended;
    diagnostics_.adaptiveIndirectDrawUsed = false;
    diagnostics_.adaptiveGpuIndirectCommandSupported =
        gpuDrivenSelectionCapabilities_.computeQueueSupported &&
        gpuDrivenIndirectCommandPipeline_ != VK_NULL_HANDLE;
    diagnostics_.adaptiveGpuIndirectCommandUsed = false;
    diagnostics_.adaptiveGpuIndirectCommandDispatches = 0;
    diagnostics_.adaptiveGpuCompactionIndirectCommandSupported =
        gpuDrivenSelectionCapabilities_.computeQueueSupported &&
        gpuDrivenIndirectCommandPipeline_ != VK_NULL_HANDLE &&
        gpuDrawItemCompactionPipeline_ != VK_NULL_HANDLE;
    diagnostics_.adaptiveGpuCompactionIndirectCommandUsed = false;
    diagnostics_.adaptiveGpuCompactionIndirectCommandParityStatus = "not checked";
    diagnostics_.adaptiveGpuCompactionIndirectCommandDispatches = 0;
    diagnostics_.adaptiveGpuCompactionIndirectCommandCpuVertices = 0;
    diagnostics_.adaptiveGpuCompactionIndirectCommandGpuVertices = 0;
    diagnostics_.adaptiveIndirectDrawCalls = 0;
    diagnostics_.adaptiveIndirectDrawCount = gpuSelectionDecision.indirectDrawCount;
    diagnostics_.adaptiveIndirectSubmittedVertices = 0;
    diagnostics_.adaptiveGpuSelectedRepresentativeCount = gpuSelectionDecision.gpuSelectedRepresentativeCount;
    diagnostics_.adaptiveGpuSelectionMs = gpuSelectionDecision.computeSelectionMs;
    diagnostics_.adaptiveGpuCompactionCpuReferenceMs = 0.0;
    diagnostics_.adaptiveGpuCompactionMs = gpuSelectionDecision.compactionMs;
    diagnostics_.adaptiveGpuFeatureClassProbeMs = 0.0;
    diagnostics_.adaptiveGpuRankProbeMs = 0.0;
    diagnostics_.adaptiveGpuDepthProbeMs = 0.0;
    diagnostics_.adaptiveGpuProjectedAreaProbeMs = 0.0;
    diagnostics_.adaptiveGpuRenderAreaProbeMs = 0.0;
    diagnostics_.adaptiveGpuRepresentedCountProbeMs = 0.0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeMs = 0.0;
    diagnostics_.adaptiveGpuClampFlagsProbeMs = 0.0;
    diagnostics_.adaptiveGpuIndirectCommandMs = 0.0;
    diagnostics_.adaptiveLodPersistentCacheStatus = std::move(adaptivePersistentCacheStatus);
    diagnostics_.adaptiveLodRuntimeStatus = std::move(adaptiveRuntimeStatus);
    diagnostics_.adaptiveLodRequestedDensity = std::move(adaptiveRequestedDensity);
    diagnostics_.adaptiveLodDisplayedDensity = std::move(adaptiveDisplayedDensity);
    diagnostics_.adaptiveLodFallbackState = std::move(adaptiveFallbackState);
    if (pointCount == 0) {
        diagnostics_.pointSubmittedCount = 0;
        diagnostics_.pointPassSubmittedCount = 0;
    }
    diagnostics_.averagePointSizePx =
        pointSizeWeight > 0 ? static_cast<float>(pointSizeSum / static_cast<double>(pointSizeWeight)) : 0.0F;
    diagnostics_.accumulationWidth = swapchainWidth_;
    diagnostics_.accumulationHeight = swapchainHeight_;
    diagnostics_.pointRenderModes =
        pointCount == 0 ? ""
                        : (renderer::pointcloud::PointCloudRendererModeUsesFastBasic(
                                   renderState_.pointCloudRendererMode)
                               ? (renderer::pointcloud::PointCloudRendererModeUsesFullSource(
                                      renderState_.pointCloudRendererMode)
                                      ? "fast-basic-source"
                                      : "fast-basic-square")
                               : (renderer::pointcloud::PointCloudRendererModeUsesFullSource(
                                      renderState_.pointCloudRendererMode)
                                      ? "beauty-full-source"
                                      : (renderer::pointcloud::PointCloudRendererModeUsesPaintedStyle(
                                             renderState_.pointCloudRendererMode)
                                             ? "painted-adaptive"
                                             : "beauty-adaptive")));

    std::ostringstream summary;
    summary << "Renderer: " << diagnostics_.rendererName << " | " << swapchainWidth_ << "x"
            << swapchainHeight_ << " | mixed scene Vulkan viewport";
    if (pointCount > 0) {
        summary << " | points: " << pointCount << " | point px avg: " << diagnostics_.averagePointSizePx
                << " | accumulation: " << diagnostics_.accumulationWidth << "x" << diagnostics_.accumulationHeight
                << " | point material: " << diagnostics_.pointRenderModes;
    }
    diagnostics_.summary = summary.str();
}

void VulkanViewportShell::UploadPointCloud(
    std::size_t layerId,
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<std::uint32_t>& sampledIndices) {
    if (cloud.PointCount() == 0) {
        throw std::runtime_error{"Cannot upload an empty point cloud."};
    }
    if (cloud.PointCount() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Point cloud exceeds the current 32-bit draw-count limit."};
    }

    WaitIdle();

    auto* existingResources = FindPointCloudResources(layerId);
    if (existingResources == nullptr) {
        pointCloudResources_.push_back(ActivePointCloudResources{});
        existingResources = &pointCloudResources_.back();
    } else {
        CleanupPointCloudResources(existingResources);
    }

    auto& resources = *existingResources;
    resources.layerId = layerId;
    resources.pointCount = static_cast<std::uint32_t>(cloud.PointCount());
    resources.activePointCount = resources.pointCount;
    resources.scalarFieldCount = static_cast<std::uint32_t>(cloud.ScalarFieldCount());
    resources.hasSourceRgb = cloud.hasSourceRgb;
    resources.hasNormals = cloud.hasNormals && cloud.normals.size() == cloud.positions.size();
    resources.cpuPositions = cloud.positions;

    resources.positionBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(cloud.positions.size() * sizeof(invisible_places::io::Float3)),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    UploadBufferData(
        resources.positionBuffer,
        cloud.positions.data(),
        resources.positionBuffer.size);

    std::vector<glm::vec4> storagePositions;
    storagePositions.reserve(cloud.positions.size());
    for (const auto& position : cloud.positions) {
        storagePositions.emplace_back(position.x, position.y, position.z, 1.0F);
    }
    resources.positionStorageBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(storagePositions.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(
        resources.positionStorageBuffer,
        storagePositions.data(),
        resources.positionStorageBuffer.size);

    resources.colorBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(cloud.packedColors.size() * sizeof(std::uint32_t)),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(
        resources.colorBuffer,
        cloud.packedColors.data(),
        resources.colorBuffer.size);

    std::vector<glm::vec4> storageNormals;
    if (resources.hasNormals) {
        storageNormals.reserve(cloud.normals.size());
        for (const auto& normal : cloud.normals) {
            storageNormals.emplace_back(normal.x, normal.y, normal.z, 0.0F);
        }
    } else {
        storageNormals.emplace_back(0.0F, 0.0F, 0.0F, 0.0F);
    }
    resources.normalBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(storageNormals.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(
        resources.normalBuffer,
        storageNormals.data(),
        resources.normalBuffer.size);

    if (!cloud.scalarFieldValues.empty()) {
        resources.scalarFieldBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(cloud.scalarFieldValues.size() * sizeof(float)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.scalarFieldBuffer,
            cloud.scalarFieldValues.data(),
            resources.scalarFieldBuffer.size);
    } else {
        const float fallbackScalar = 0.0F;
        resources.scalarFieldBuffer = CreateHostVisibleBuffer(
            sizeof(float),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources.scalarFieldBuffer, &fallbackScalar, sizeof(float));
    }

    for (auto& styleBuffer : resources.styleBuffers) {
        styleBuffer = CreateHostVisibleBuffer(
            sizeof(PointCloudStyleGpu),
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    }
    resources.exrStyleBuffer = CreateHostVisibleBuffer(
        sizeof(PointCloudStyleGpu),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    const renderer::pointcloud::PointCloudDrawItemGpu fallbackDrawItem{};
    const VkDrawIndirectCommand fallbackIndirectDraw{0U, 1U, 0U, 0U};
    const GpuDrawItemCompactionStats fallbackCompactionStats{};
    for (std::size_t frameIndex = 0; frameIndex < resources.drawItemBuffers.size(); ++frameIndex) {
        resources.drawItemBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackDrawItem),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources.drawItemBuffers[frameIndex], &fallbackDrawItem, sizeof(fallbackDrawItem));
        resources.gpuCompactedDrawItemBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackDrawItem),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuCompactedDrawItemBuffers[frameIndex],
            &fallbackDrawItem,
            sizeof(fallbackDrawItem));
        resources.gpuCompactionStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuCompactionStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuFeatureClassProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuFeatureClassProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuRankProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuRankProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuDepthProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuDepthProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuProjectedAreaProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuProjectedAreaProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuRenderAreaProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuRenderAreaProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuRepresentedCountProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuRepresentedCountProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuCoverageCompensationProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuCoverageCompensationProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.gpuClampFlagsProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackCompactionStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuClampFlagsProbeStatsBuffers[frameIndex],
            &fallbackCompactionStats,
            sizeof(fallbackCompactionStats));
        resources.indirectDrawCommandBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackIndirectDraw),
            VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.indirectDrawCommandBuffers[frameIndex],
            &fallbackIndirectDraw,
            sizeof(fallbackIndirectDraw));
        resources.gpuCompactionIndirectCommandBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackIndirectDraw),
            VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources.gpuCompactionIndirectCommandBuffers[frameIndex],
            &fallbackIndirectDraw,
            sizeof(fallbackIndirectDraw));
        UpdateGpuDrivenIndirectDescriptorSet(&resources, frameIndex);
        UpdateGpuCompactionIndirectDescriptorSet(&resources, frameIndex);
        UpdateGpuCompactionDescriptorSet(&resources, frameIndex);
        UpdateGpuFeatureClassProbeDescriptorSet(&resources, frameIndex);
        UpdateGpuRankProbeDescriptorSet(&resources, frameIndex);
        UpdateGpuDepthProbeDescriptorSet(&resources, frameIndex);
        UpdateGpuProjectedAreaProbeDescriptorSet(&resources, frameIndex);
        UpdateGpuRenderAreaProbeDescriptorSet(&resources, frameIndex);
        UpdateGpuRepresentedCountProbeDescriptorSet(&resources, frameIndex);
        UpdateGpuCoverageCompensationProbeDescriptorSet(&resources, frameIndex);
        UpdateGpuClampFlagsProbeDescriptorSet(&resources, frameIndex);
        resources.drawItemCapacities[frameIndex] = 1U;
        resources.gpuCompactedDrawItemCapacities[frameIndex] = 1U;
    }
    resources.exrDrawItemBuffer = CreateHostVisibleBuffer(
        sizeof(fallbackDrawItem),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.exrDrawItemBuffer, &fallbackDrawItem, sizeof(fallbackDrawItem));
    resources.exrDrawItemCapacity = 1U;
    UpdatePointCloudDescriptorSets(&resources);

    UpdatePointBudget(layerId, sampledIndices);
}

void VulkanViewportShell::UploadPointCloudResidentSubset(
    std::size_t layerId,
    const invisible_places::io::LoadedPointCloud& cloud) {
    if (cloud.PointCount() == 0) {
        return;
    }
    if (cloud.PointCount() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Resident point-cloud subset exceeds the current 32-bit draw-count limit."};
    }

    auto* resources = FindPointCloudResources(layerId);
    const bool hasNormals = cloud.hasNormals && cloud.normals.size() == cloud.positions.size();
    const auto positionBytes =
        static_cast<VkDeviceSize>(cloud.positions.size() * sizeof(invisible_places::io::Float3));
    const auto positionStorageBytes =
        static_cast<VkDeviceSize>(cloud.positions.size() * sizeof(glm::vec4));
    const auto colorBytes =
        static_cast<VkDeviceSize>(std::max<std::size_t>(1U, cloud.packedColors.size()) * sizeof(std::uint32_t));
    const auto normalBytes =
        static_cast<VkDeviceSize>(std::max<std::size_t>(1U, hasNormals ? cloud.normals.size() : 1U) * sizeof(glm::vec4));
    const auto scalarBytes =
        static_cast<VkDeviceSize>(std::max<std::size_t>(1U, cloud.scalarFieldValues.size()) * sizeof(float));

    const bool needsFullUpload =
        resources == nullptr ||
        resources->positionBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE ||
        resources->colorBuffer.buffer == VK_NULL_HANDLE ||
        resources->normalBuffer.buffer == VK_NULL_HANDLE ||
        resources->scalarFieldBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionBuffer.size < positionBytes ||
        resources->positionStorageBuffer.size < positionStorageBytes ||
        resources->colorBuffer.size < colorBytes ||
        resources->normalBuffer.size < normalBytes ||
        resources->scalarFieldBuffer.size < scalarBytes ||
        resources->scalarFieldCount != static_cast<std::uint32_t>(cloud.ScalarFieldCount());
    if (needsFullUpload) {
        UploadPointCloud(layerId, cloud, {});
        return;
    }

    resources->pointCount = static_cast<std::uint32_t>(cloud.PointCount());
    resources->activePointCount = resources->pointCount;
    resources->scalarFieldCount = static_cast<std::uint32_t>(cloud.ScalarFieldCount());
    resources->hasSourceRgb = cloud.hasSourceRgb;
    resources->hasNormals = hasNormals;
    resources->cpuPositions = cloud.positions;
    resources->usingSampledIndices = false;

    UploadBufferData(resources->positionBuffer, cloud.positions.data(), positionBytes);

    std::vector<glm::vec4> storagePositions;
    storagePositions.reserve(cloud.positions.size());
    for (const auto& position : cloud.positions) {
        storagePositions.emplace_back(position.x, position.y, position.z, 1.0F);
    }
    UploadBufferData(resources->positionStorageBuffer, storagePositions.data(), positionStorageBytes);

    if (!cloud.packedColors.empty()) {
        UploadBufferData(resources->colorBuffer, cloud.packedColors.data(), colorBytes);
    } else {
        const std::uint32_t fallbackColor = 0xffffffffU;
        UploadBufferData(resources->colorBuffer, &fallbackColor, sizeof(fallbackColor));
    }

    std::vector<glm::vec4> storageNormals;
    if (hasNormals) {
        storageNormals.reserve(cloud.normals.size());
        for (const auto& normal : cloud.normals) {
            storageNormals.emplace_back(normal.x, normal.y, normal.z, 0.0F);
        }
    } else {
        storageNormals.emplace_back(0.0F, 0.0F, 0.0F, 0.0F);
    }
    UploadBufferData(
        resources->normalBuffer,
        storageNormals.data(),
        static_cast<VkDeviceSize>(storageNormals.size() * sizeof(glm::vec4)));

    if (!cloud.scalarFieldValues.empty()) {
        UploadBufferData(resources->scalarFieldBuffer, cloud.scalarFieldValues.data(), scalarBytes);
    } else {
        const float fallbackScalar = 0.0F;
        UploadBufferData(resources->scalarFieldBuffer, &fallbackScalar, sizeof(fallbackScalar));
    }
}

void VulkanViewportShell::UpdatePointBudget(
    std::size_t layerId,
    const std::vector<std::uint32_t>& sampledIndices) {
    WaitIdle();

    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        return;
    }

    DestroyBuffer(&resources->sampledIndexBuffer);
    DestroyBuffer(&resources->sampledSurfelIndexBuffer);
    resources->usingSampledIndices = false;
    resources->activePointCount = resources->pointCount;

    if (resources->pointCount == 0 ||
        sampledIndices.empty() ||
        sampledIndices.size() >= resources->pointCount) {
        return;
    }

    resources->sampledIndexBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(sampledIndices.size() * sizeof(std::uint32_t)),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    UploadBufferData(
        resources->sampledIndexBuffer,
        sampledIndices.data(),
        resources->sampledIndexBuffer.size);
    resources->usingSampledIndices = true;
    resources->activePointCount = static_cast<std::uint32_t>(sampledIndices.size());

    const auto surfelIndices =
        invisible_places::renderer::pointcloud::GenerateSurfelEncodedSampleIndices(sampledIndices);
    if (!surfelIndices.empty()) {
        resources->sampledSurfelIndexBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(surfelIndices.size() * sizeof(std::uint32_t)),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
        UploadBufferData(
            resources->sampledSurfelIndexBuffer,
            surfelIndices.data(),
            resources->sampledSurfelIndexBuffer.size);
    }
}

void VulkanViewportShell::RemovePointCloud(std::size_t layerId) {
    WaitIdle();

    auto resourcesIt = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) { return resources.layerId == layerId; });
    if (resourcesIt == pointCloudResources_.end()) {
        return;
    }

    CleanupPointCloudResources(&(*resourcesIt));
    pointCloudResources_.erase(resourcesIt);
}

void VulkanViewportShell::ClearPointClouds() {
    WaitIdle();
    for (auto& resources : pointCloudResources_) {
        CleanupPointCloudResources(&resources);
    }
    pointCloudResources_.clear();
}

void VulkanViewportShell::UploadGaussianSplats(
    std::size_t layerId,
    const invisible_places::io::LoadedGaussianSplat& splats) {
    if (splats.SplatCount() == 0) {
        throw std::runtime_error{"Cannot upload an empty Gaussian splat layer."};
    }
    if (splats.SplatCount() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Gaussian splat layer exceeds the current 32-bit draw-count limit."};
    }

    WaitIdle();

    auto* existingResources = FindGaussianSplatResources(layerId);
    const auto nextRevision = existingResources != nullptr ? (existingResources->revision + 1U) : 1U;
    if (existingResources == nullptr) {
        gaussianSplatResources_.push_back(ActiveGaussianSplatResources{});
        existingResources = &gaussianSplatResources_.back();
    } else {
        CleanupGaussianSplatResources(existingResources);
    }

    auto& resources = *existingResources;
    resources.layerId = layerId;
    resources.splatCount = static_cast<std::uint32_t>(splats.SplatCount());
    resources.cpuCenters = splats.centers;
    resources.cpuScales = splats.scales;
    resources.cpuRotations = splats.rotations;
    resources.cpuOpacities = splats.opacities;
    resources.cpuShCoefficients = splats.shCoefficients;
    resources.revision = nextRevision;

    std::vector<glm::vec4> centers;
    std::vector<glm::vec4> scales;
    std::vector<glm::vec4> rotations;
    centers.reserve(splats.SplatCount());
    scales.reserve(splats.SplatCount());
    rotations.reserve(splats.SplatCount());

    for (std::size_t index = 0; index < splats.SplatCount(); ++index) {
        const auto& center = splats.centers[index];
        const auto& scale = splats.scales[index];
        const auto& rotation = splats.rotations[index];
        centers.emplace_back(center.x, center.y, center.z, 1.0F);
        scales.emplace_back(scale[0], scale[1], scale[2], 0.0F);
        rotations.emplace_back(rotation[0], rotation[1], rotation[2], rotation[3]);
    }

    resources.centerBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(centers.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.centerBuffer, centers.data(), resources.centerBuffer.size);

    resources.scaleBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(scales.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.scaleBuffer, scales.data(), resources.scaleBuffer.size);

    resources.rotationBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(rotations.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.rotationBuffer, rotations.data(), resources.rotationBuffer.size);

    resources.opacityBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(splats.opacities.size() * sizeof(float)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.opacityBuffer, splats.opacities.data(), resources.opacityBuffer.size);

    resources.shBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(splats.shCoefficients.size() * sizeof(float)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.shBuffer, splats.shCoefficients.data(), resources.shBuffer.size);

    UpdateGaussianSplatDescriptorSets(&resources);
    highQualityGaussianSceneDirty_ = true;
}

void VulkanViewportShell::RemoveGaussianSplats(std::size_t layerId) {
    WaitIdle();

    auto resourcesIt = std::find_if(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [layerId](const ActiveGaussianSplatResources& resources) { return resources.layerId == layerId; });
    if (resourcesIt == gaussianSplatResources_.end()) {
        return;
    }

    CleanupGaussianSplatResources(&(*resourcesIt));
    gaussianSplatResources_.erase(resourcesIt);
    highQualityGaussianSceneDirty_ = true;
}

void VulkanViewportShell::ClearGaussianSplats() {
    WaitIdle();
    for (auto& resources : gaussianSplatResources_) {
        CleanupGaussianSplatResources(&resources);
    }
    gaussianSplatResources_.clear();
    highQualityGaussianSceneDirty_ = true;
}

bool VulkanViewportShell::UiWantsMouseCapture() const {
    return ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse;
}

bool VulkanViewportShell::UiWantsKeyboardCapture() const {
    return ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureKeyboard;
}

bool VulkanViewportShell::HasPointClouds() const {
    return std::any_of(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [](const ActivePointCloudResources& resources) { return resources.activePointCount > 0; });
}

bool VulkanViewportShell::HasGaussianSplats() const {
    return std::any_of(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [](const ActiveGaussianSplatResources& resources) { return resources.splatCount > 0; });
}

invisible_places::output::HalfRgbaExrImage VulkanViewportShell::RenderPointCloudExrFrame(
    const PointCloudExrFrameRequest& request) {
    if (request.width == 0 || request.height == 0) {
        throw std::runtime_error{"GPU EXR export requires a non-zero frame size."};
    }
    if (request.renderState.pointCloudLayers.empty()) {
        throw std::runtime_error{"GPU EXR export requires at least one visible point-cloud layer."};
    }

    if (exrExportResources_.framebuffer == VK_NULL_HANDLE ||
        exrExportResources_.width != request.width ||
        exrExportResources_.height != request.height) {
        CreateExrExportResources(request.width, request.height);
    }

    std::array<VkFence, kFramesInFlight> frameFences{};
    std::size_t frameFenceCount = 0;
    for (const auto& frame : frameResources_) {
        if (frame.fence != VK_NULL_HANDLE) {
            frameFences[frameFenceCount++] = frame.fence;
        }
    }
    if (frameFenceCount > 0) {
        vkWaitForFences(
            device_,
            static_cast<std::uint32_t>(frameFenceCount),
            frameFences.data(),
            VK_TRUE,
            UINT64_MAX);
    }

    const auto previousRenderState = renderState_;
    auto applyDrawItemBuffers = [&](const SceneRenderState& state) {
        std::uint32_t reallocations = 0;
        for (const auto& layer : state.pointCloudLayers) {
            auto* resources = FindPointCloudResources(layer.layerId);
            if (resources == nullptr) {
                continue;
            }
            const bool reallocated = UpdatePointCloudExrDrawItemBuffer(
                resources,
                layer.useAdaptiveDrawItems ? layer.adaptiveDrawItems.get() : nullptr,
                layer.useAdaptiveDrawItems ? layer.adaptiveLodRevision : 0ULL);
            reallocations += reallocated ? 1U : 0U;
        }
        diagnostics_.pointDrawItemBufferReallocations = reallocations;
    };
    auto restoreRenderState = [&]() {
        renderState_ = previousRenderState;
        applyDrawItemBuffers(previousRenderState);
    };

    try {
        renderState_ = request.renderState;
        applyDrawItemBuffers(request.renderState);
        for (auto& resources : pointCloudResources_) {
            UpdatePointCloudExrDescriptorSet(&resources, exrExportResources_.depthImage.view);
        }

        Check(vkWaitForFences(device_, 1, &exrExportResources_.fence, VK_TRUE, UINT64_MAX), "vkWaitForFences(exr)");
        Check(vkResetFences(device_, 1, &exrExportResources_.fence), "vkResetFences(exr)");
        Check(
            vkResetCommandBuffer(exrExportResources_.commandBuffer, 0),
            "vkResetCommandBuffer(exr)");
        UploadFrameUniforms(0U, request.width, request.height);
        RecordExrExportCommandBuffer(request);

        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &exrExportResources_.commandBuffer;
        Check(vkQueueSubmit(graphicsQueue_, 1, &submitInfo, exrExportResources_.fence), "vkQueueSubmit(exr)");
        Check(vkWaitForFences(device_, 1, &exrExportResources_.fence, VK_TRUE, UINT64_MAX), "vkWaitForFences(exr complete)");

        invisible_places::output::HalfRgbaExrImage image;
        image.width = request.width;
        image.height = request.height;
        const auto pixelCount =
            static_cast<std::size_t>(request.width) * static_cast<std::size_t>(request.height);
        image.rgbaHalf.resize(pixelCount * 4U);
        image.normalHalf.resize(pixelCount * 3U);
        image.albedoHalf.resize(pixelCount * 3U);
        image.depth.resize(pixelCount);

        void* mappedColor = exrExportResources_.colorReadbackBuffer.mapped;
        bool unmapColor = false;
        if (mappedColor == nullptr) {
            Check(
                vkMapMemory(
                    device_,
                    exrExportResources_.colorReadbackBuffer.memory,
                    0,
                    exrExportResources_.colorReadbackBuffer.size,
                    0,
                    &mappedColor),
                "vkMapMemory(exr color)");
            unmapColor = true;
        }
        std::memcpy(
            image.rgbaHalf.data(),
            mappedColor,
            image.rgbaHalf.size() * sizeof(std::uint16_t));
        if (unmapColor) {
            vkUnmapMemory(device_, exrExportResources_.colorReadbackBuffer.memory);
        }

        void* mappedDepth = exrExportResources_.depthReadbackBuffer.mapped;
        bool unmapDepth = false;
        if (mappedDepth == nullptr) {
            Check(
                vkMapMemory(
                    device_,
                    exrExportResources_.depthReadbackBuffer.memory,
                    0,
                    exrExportResources_.depthReadbackBuffer.size,
                    0,
                    &mappedDepth),
                "vkMapMemory(exr depth)");
            unmapDepth = true;
        }
        std::memcpy(
            image.depth.data(),
            mappedDepth,
            image.depth.size() * sizeof(float));
        if (unmapDepth) {
            vkUnmapMemory(device_, exrExportResources_.depthReadbackBuffer.memory);
        }

        auto copyRgbHalfReadback = [&](const BufferAllocation& buffer,
                                       std::vector<std::uint16_t>* destination,
                                       const char* mapLabel) {
            if (destination == nullptr || destination->size() != pixelCount * 3U) {
                return;
            }

            void* mapped = buffer.mapped;
            bool unmap = false;
            if (mapped == nullptr) {
                Check(
                    vkMapMemory(device_, buffer.memory, 0, buffer.size, 0, &mapped),
                    mapLabel);
                unmap = true;
            }

            const auto* source = static_cast<const std::uint16_t*>(mapped);
            for (std::size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
                const std::size_t sourceOffset = pixelIndex * 4U;
                const std::size_t destinationOffset = pixelIndex * 3U;
                (*destination)[destinationOffset + 0U] = source[sourceOffset + 0U];
                (*destination)[destinationOffset + 1U] = source[sourceOffset + 1U];
                (*destination)[destinationOffset + 2U] = source[sourceOffset + 2U];
            }
            if (unmap) {
                vkUnmapMemory(device_, buffer.memory);
            }
        };
        copyRgbHalfReadback(exrExportResources_.normalReadbackBuffer, &image.normalHalf, "vkMapMemory(exr normal)");
        copyRgbHalfReadback(exrExportResources_.albedoReadbackBuffer, &image.albedoHalf, "vkMapMemory(exr albedo)");

        restoreRenderState();
        return image;
    } catch (...) {
        restoreRenderState();
        throw;
    }
}

void VulkanViewportShell::CreateInstance() {
    std::uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
    if (glfwExtensions == nullptr) {
        throw std::runtime_error{"GLFW did not provide required Vulkan instance extensions."};
    }

    std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
#if defined(__APPLE__)
    if (std::find(extensions.begin(), extensions.end(), VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME) ==
        extensions.end()) {
        extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    }
#endif

    VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    appInfo.pApplicationName = "Invisible Places";
    appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    appInfo.pEngineName = "InvisiblePlacesScenePreview";
    appInfo.engineVersion = VK_MAKE_VERSION(0, 1, 0);
    appInfo.apiVersion = VK_API_VERSION_1_2;

    VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    createInfo.ppEnabledExtensionNames = extensions.data();
#if defined(__APPLE__)
    createInfo.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif

    Check(vkCreateInstance(&createInfo, nullptr, &instance_), "vkCreateInstance");
}

void VulkanViewportShell::CreateSurface() {
    Check(glfwCreateWindowSurface(instance_, window_, nullptr, &surface_), "glfwCreateWindowSurface");
}

void VulkanViewportShell::PickPhysicalDevice() {
    std::uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance_, &deviceCount, nullptr);
    if (deviceCount == 0) {
        throw std::runtime_error{"No Vulkan physical devices were found."};
    }

    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(instance_, &deviceCount, devices.data());

    for (const auto device : devices) {
        bool portabilitySubsetEnabled = false;
        if (!IsDeviceSuitable(device, surface_, &portabilitySubsetEnabled)) {
            continue;
        }

        physicalDevice_ = device;
        enablePortabilitySubset_ = portabilitySubsetEnabled;

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(device, &properties);
        VkPhysicalDeviceFeatures features{};
        vkGetPhysicalDeviceFeatures(device, &features);
        const auto extensions = EnumerateDeviceExtensions(device);
        diagnostics_.rendererName = properties.deviceName;
        diagnostics_.driverName = "Vulkan";
        gpuTimestampPeriodNs_ = properties.limits.timestampPeriod;
        pointSizeRangeMin_ = std::max(1.0F, properties.limits.pointSizeRange[0]);
        pointSizeRangeMax_ = std::max(pointSizeRangeMin_, properties.limits.pointSizeRange[1]);
        gpuDrivenSelectionCapabilities_.deviceName = properties.deviceName;
        gpuDrivenSelectionCapabilities_.portabilitySubsetActive = portabilitySubsetEnabled;
        gpuDrivenSelectionCapabilities_.storageBuffersSupported =
            properties.limits.maxPerStageDescriptorStorageBuffers >= 4U &&
            properties.limits.maxStorageBufferRange >= sizeof(renderer::pointcloud::PointCloudDrawItemGpu);
        gpuDrivenSelectionCapabilities_.maxStorageBuffersPerShaderStage =
            properties.limits.maxPerStageDescriptorStorageBuffers;
        gpuDrivenSelectionCapabilities_.maxStorageBufferRange = properties.limits.maxStorageBufferRange;
        gpuDrivenSelectionCapabilities_.indirectDrawSupported = properties.limits.maxDrawIndirectCount > 0U;
        gpuDrivenSelectionCapabilities_.maxDrawIndirectCount = properties.limits.maxDrawIndirectCount;
        gpuDrivenSelectionCapabilities_.drawIndirectFirstInstance =
            features.drawIndirectFirstInstance == VK_TRUE;
        gpuDrivenSelectionCapabilities_.multiDrawIndirect = features.multiDrawIndirect == VK_TRUE;
#ifdef VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME
        gpuDrivenSelectionCapabilities_.indirectCountSupported =
            properties.apiVersion >= VK_API_VERSION_1_2 ||
            HasExtension(extensions, VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME);
#else
        gpuDrivenSelectionCapabilities_.indirectCountSupported =
            properties.apiVersion >= VK_API_VERSION_1_2;
#endif
        break;
    }

    if (physicalDevice_ == VK_NULL_HANDLE) {
        throw std::runtime_error{"No suitable Vulkan device was found for presentation."};
    }

    const auto selection = FindQueueFamilies(physicalDevice_, surface_);
    graphicsQueueFamily_ = selection.graphicsFamily.value();
    presentQueueFamily_ = selection.presentFamily.value();
    std::uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    if (queueFamilyCount > 0) {
        vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &queueFamilyCount, queueFamilies.data());
    }
    graphicsQueueTimestampValidBits_ =
        graphicsQueueFamily_ < queueFamilies.size() ? queueFamilies[graphicsQueueFamily_].timestampValidBits : 0U;
    gpuDrivenSelectionCapabilities_.computeQueueSupported =
        graphicsQueueFamily_ < queueFamilies.size() &&
        (queueFamilies[graphicsQueueFamily_].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0;
    if (enablePortabilitySubset_ && gpuDrivenSelectionCapabilities_.limitations.empty()) {
        gpuDrivenSelectionCapabilities_.limitations = "MoltenVK portability subset active; GPU selection remains guarded";
    }
    gpuTimestampsSupported_ = gpuTimestampPeriodNs_ > 0.0F && graphicsQueueTimestampValidBits_ > 0U;
    diagnostics_.gpuTimestampSupported = gpuTimestampsSupported_;
    diagnostics_.gpuTimestampState =
        gpuTimestampsSupported_ ? "supported; waiting for first scene timing" : "unavailable";
}

void VulkanViewportShell::CreateLogicalDevice() {
    const std::set<std::uint32_t> uniqueFamilies = {graphicsQueueFamily_, presentQueueFamily_};

    float queuePriority = 1.0F;
    std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
    for (const auto family : uniqueFamilies) {
        VkDeviceQueueCreateInfo queueCreateInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueCreateInfo.queueFamilyIndex = family;
        queueCreateInfo.queueCount = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;
        queueCreateInfos.push_back(queueCreateInfo);
    }

    std::vector<const char*> deviceExtensions(kRequiredDeviceExtensions.begin(), kRequiredDeviceExtensions.end());
    if (enablePortabilitySubset_) {
        deviceExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
    }

    VkPhysicalDeviceFeatures features{};
    features.largePoints = VK_TRUE;
    features.shaderInt64 = VK_TRUE;

    VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    createInfo.queueCreateInfoCount = static_cast<std::uint32_t>(queueCreateInfos.size());
    createInfo.pQueueCreateInfos = queueCreateInfos.data();
    createInfo.pEnabledFeatures = &features;
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    Check(vkCreateDevice(physicalDevice_, &createInfo, nullptr, &device_), "vkCreateDevice");

    vkGetDeviceQueue(device_, graphicsQueueFamily_, 0, &graphicsQueue_);
    vkGetDeviceQueue(device_, presentQueueFamily_, 0, &presentQueue_);
}

void VulkanViewportShell::CreateSwapchain() {
    const auto support = QuerySwapchainSupport(physicalDevice_, surface_);
    const auto surfaceFormat = ChooseSurfaceFormat(support.formats);
    const auto presentMode = ChoosePresentMode(support.presentModes);
    const auto extent = ChooseExtent(window_, support.capabilities);

    std::uint32_t imageCount = support.capabilities.minImageCount + 1;
    if (support.capabilities.maxImageCount > 0 && imageCount > support.capabilities.maxImageCount) {
        imageCount = support.capabilities.maxImageCount;
    }

    const std::uint32_t queueFamilyIndices[] = {graphicsQueueFamily_, presentQueueFamily_};

    VkSwapchainCreateInfoKHR createInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    createInfo.surface = surface_;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = extent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    if (graphicsQueueFamily_ != presentQueueFamily_) {
        createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        createInfo.queueFamilyIndexCount = 2;
        createInfo.pQueueFamilyIndices = queueFamilyIndices;
    } else {
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    createInfo.preTransform = support.capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = presentMode;
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = VK_NULL_HANDLE;

    Check(vkCreateSwapchainKHR(device_, &createInfo, nullptr, &swapchain_), "vkCreateSwapchainKHR");

    std::uint32_t actualImageCount = 0;
    vkGetSwapchainImagesKHR(device_, swapchain_, &actualImageCount, nullptr);
    swapchainImages_.resize(actualImageCount);
    vkGetSwapchainImagesKHR(device_, swapchain_, &actualImageCount, swapchainImages_.data());

    swapchainImageFormat_ = surfaceFormat.format;
    swapchainWidth_ = extent.width;
    swapchainHeight_ = extent.height;
    swapchainImagesInFlight_.assign(swapchainImages_.size(), VK_NULL_HANDLE);

    std::ostringstream summary;
    summary << "Renderer: " << diagnostics_.rendererName << " | " << swapchainWidth_ << "x"
            << swapchainHeight_ << " | mixed scene Vulkan viewport";
    diagnostics_.summary = summary.str();
    diagnostics_.width = swapchainWidth_;
    diagnostics_.height = swapchainHeight_;
}

void VulkanViewportShell::CreateImageViews() {
    imageViews_.clear();
    imageViews_.reserve(swapchainImages_.size());

    for (const auto image : swapchainImages_) {
        VkImageViewCreateInfo createInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        createInfo.image = image;
        createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        createInfo.format = swapchainImageFormat_;
        createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        createInfo.subresourceRange.baseMipLevel = 0;
        createInfo.subresourceRange.levelCount = 1;
        createInfo.subresourceRange.baseArrayLayer = 0;
        createInfo.subresourceRange.layerCount = 1;

        VkImageView imageView = VK_NULL_HANDLE;
        Check(vkCreateImageView(device_, &createInfo, nullptr, &imageView), "vkCreateImageView");
        imageViews_.push_back(imageView);
    }
}

void VulkanViewportShell::CreateRenderPass() {
    accumulationFormat_ = SelectAccumulationFormat();
    revealageFormat_ = SelectRevealageFormat();
    linearDepthFormat_ = VK_FORMAT_R32_SFLOAT;

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = swapchainImageFormat_;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = SelectDepthFormat();
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription accumulationAttachment{};
    accumulationAttachment.format = accumulationFormat_;
    accumulationAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    accumulationAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    accumulationAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    accumulationAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    accumulationAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription revealageAttachment{};
    revealageAttachment.format = revealageFormat_;
    revealageAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    revealageAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    revealageAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    revealageAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    revealageAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription emissiveAttachment{};
    emissiveAttachment.format = accumulationFormat_;
    emissiveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    emissiveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    emissiveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    emissiveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    emissiveAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription linearDepthAttachment{};
    linearDepthAttachment.format = linearDepthFormat_;
    linearDepthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    linearDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    linearDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    linearDepthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    linearDepthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    linearDepthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    linearDepthAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentReference subpass0ColorRef{5, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

    VkAttachmentReference depthAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthReadOnlyAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass0{};
    subpass0.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass0.colorAttachmentCount = 1;
    subpass0.pColorAttachments = &subpass0ColorRef;
    subpass0.pDepthStencilAttachment = &depthAttachmentRef;

    VkAttachmentReference subpass1ColorRefs[3]{};
    subpass1ColorRefs[0] = {2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    subpass1ColorRefs[1] = {3, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    subpass1ColorRefs[2] = {4, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass1InputRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass1{};
    subpass1.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass1.colorAttachmentCount = 3;
    subpass1.pColorAttachments = subpass1ColorRefs;
    subpass1.inputAttachmentCount = 1;
    subpass1.pInputAttachments = &subpass1InputRef;
    subpass1.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    VkAttachmentReference subpass2ColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass2InputRefs[3]{};
    subpass2InputRefs[0] = {2, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    subpass2InputRefs[1] = {3, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    subpass2InputRefs[2] = {4, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass2{};
    subpass2.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass2.colorAttachmentCount = 1;
    subpass2.pColorAttachments = &subpass2ColorRef;
    subpass2.inputAttachmentCount = 3;
    subpass2.pInputAttachments = subpass2InputRefs;

    VkAttachmentReference subpass3ColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass3{};
    subpass3.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass3.colorAttachmentCount = 1;
    subpass3.pColorAttachments = &subpass3ColorRef;
    subpass3.pDepthStencilAttachment = &depthAttachmentRef;

    std::array<VkSubpassDependency, 8> dependencies{};
    dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[0].dstSubpass = 0;
    dependencies[0].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    dependencies[1].srcSubpass = 0;
    dependencies[1].dstSubpass = 1;
    dependencies[1].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[1].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[2].srcSubpass = 0;
    dependencies[2].dstSubpass = 3;
    dependencies[2].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[2].dstStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[2].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[2].dstAccessMask =
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    dependencies[3].srcSubpass = 1;
    dependencies[3].dstSubpass = 2;
    dependencies[3].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[3].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[4].srcSubpass = 2;
    dependencies[4].dstSubpass = 3;
    dependencies[4].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[4].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[5].srcSubpass = 3;
    dependencies[5].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[5].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[5].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[5].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[5].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    dependencies[6].srcSubpass = 0;
    dependencies[6].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[6].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[6].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[6].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[6].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    dependencies[7].srcSubpass = 1;
    dependencies[7].dstSubpass = 3;
    dependencies[7].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[7].dstStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[7].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
    dependencies[7].dstAccessMask =
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    const std::array<VkAttachmentDescription, 6> attachments = {
        colorAttachment,
        depthAttachment,
        accumulationAttachment,
        revealageAttachment,
        emissiveAttachment,
        linearDepthAttachment,
    };
    const std::array<VkSubpassDescription, 4> subpasses = {subpass0, subpass1, subpass2, subpass3};

    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = static_cast<std::uint32_t>(subpasses.size());
    renderPassInfo.pSubpasses = subpasses.data();
    renderPassInfo.dependencyCount = static_cast<std::uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();

    Check(vkCreateRenderPass(device_, &renderPassInfo, nullptr, &renderPass_), "vkCreateRenderPass");
}

void VulkanViewportShell::CreatePresentRenderPass() {
    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = swapchainImageFormat_;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;

    std::array<VkSubpassDependency, 2> dependencies{};
    dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[0].dstSubpass = 0;
    dependencies[0].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[1].srcSubpass = 0;
    dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = 1;
    renderPassInfo.pAttachments = &colorAttachment;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = static_cast<std::uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();

    Check(vkCreateRenderPass(device_, &renderPassInfo, nullptr, &presentRenderPass_), "vkCreateRenderPass(present)");
}

void VulkanViewportShell::CreatePointDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 8> bindings{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[2].binding = 2;
    bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[2].descriptorCount = 1;
    bindings[2].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[3].binding = 3;
    bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    bindings[3].descriptorCount = 1;
    bindings[3].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    for (std::uint32_t bindingIndex = 4; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[bindingIndex].descriptorCount = 1;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &pointDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(point)");
}

void VulkanViewportShell::CreateGaussianSplatDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 7> bindings{};
    bindings[0] = {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};
    for (std::uint32_t index = 1; index < 6; ++index) {
        bindings[index].binding = index;
        bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[index].descriptorCount = 1;
        bindings[index].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    bindings[6] = {6, VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &gaussianSplatDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(gsplat)");
}

void VulkanViewportShell::CreateHighQualityGaussianSplatDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 9> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorCount = 1;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[bindingIndex].descriptorType =
            bindingIndex == 0 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(
            device_,
            &layoutInfo,
            nullptr,
            &highQualityGaussianSplatDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(gsplat_hq)");
}

void VulkanViewportShell::CreateCompositeDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex] = {
            bindingIndex,
            VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT,
            1,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr};
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &compositeDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(composite)");
}

void VulkanViewportShell::CreatePostProcessDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 2> bindings{};
    bindings[0] = {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};
    bindings[1] = {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &postProcessDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(postprocess)");
}

void VulkanViewportShell::CreateGpuDrivenSelectionDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 2> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[bindingIndex].descriptorCount = 1;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &gpuDrivenSelectionDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(gpu driven pointcloud)");
}

void VulkanViewportShell::CreateGpuCompactionDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[bindingIndex].descriptorCount = 1;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &gpuCompactionDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(pointcloud gpu compaction)");
}

void VulkanViewportShell::CreateDescriptorPools() {
    const std::array<VkDescriptorPoolSize, 4> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1024),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4096),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1024),
        MakePoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1024),
    };

    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 1024;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    Check(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_), "vkCreateDescriptorPool");

    const std::array<VkDescriptorPoolSize, 3> gsplatPoolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1024),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4096),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1024),
    };

    VkDescriptorPoolCreateInfo gsplatPoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    gsplatPoolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    gsplatPoolInfo.maxSets = 1024;
    gsplatPoolInfo.poolSizeCount = static_cast<std::uint32_t>(gsplatPoolSizes.size());
    gsplatPoolInfo.pPoolSizes = gsplatPoolSizes.data();
    Check(
        vkCreateDescriptorPool(device_, &gsplatPoolInfo, nullptr, &gaussianSplatDescriptorPool_),
        "vkCreateDescriptorPool(gsplat)");
}

void VulkanViewportShell::CreatePostProcessSampler() {
    VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    samplerInfo.magFilter = VK_FILTER_NEAREST;
    samplerInfo.minFilter = VK_FILTER_NEAREST;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.minLod = 0.0F;
    samplerInfo.maxLod = 0.0F;
    samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;

    Check(vkCreateSampler(device_, &samplerInfo, nullptr, &postProcessSampler_), "vkCreateSampler(postprocess)");
}

void VulkanViewportShell::CreateUniformResources() {
    for (auto& frame : frameResources_) {
        DestroyBuffer(&frame.uniformBuffer);
        frame.uniformBuffer = CreateHostVisibleBuffer(sizeof(FrameUniforms), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    }
}

void VulkanViewportShell::CreatePointPipelines() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.vert.spv").string());
    const auto depthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_export_depth.frag.spv").string());
    const auto accumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_accumulation.frag.spv").string());
    const auto constantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_constant_simple.vert.spv").string());
    const auto constantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_constant_simple_accumulation.frag.spv").string());
    const auto opaqueHardDiscFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_opaque_hard_disc.frag.spv").string());
    const auto fastBasicVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.vert.spv").string());
    const auto fastBasicFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.frag.spv").string());
    const auto surfelVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel.vert.spv").string());
    const auto surfelDepthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_export_depth.frag.spv").string());
    const auto surfelAccumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_accumulation.frag.spv").string());
    const auto surfelConstantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_constant_simple.vert.spv").string());
    const auto surfelConstantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_constant_simple_accumulation.frag.spv").string());
    const auto surfelOpaqueHardDiscFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_opaque_hard_disc.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(point vertex)");
    const auto depthFragmentModule =
        CreateShaderModule(device_, depthFragmentShaderCode, "vkCreateShaderModule(point depth fragment)");
    const auto accumulationFragmentModule =
        CreateShaderModule(device_, accumulationFragmentShaderCode, "vkCreateShaderModule(point accumulation fragment)");
    const auto constantSimpleVertexModule =
        CreateShaderModule(device_, constantSimpleVertexShaderCode, "vkCreateShaderModule(point simple vertex)");
    const auto constantSimpleFragmentModule =
        CreateShaderModule(device_, constantSimpleFragmentShaderCode, "vkCreateShaderModule(point simple accumulation fragment)");
    const auto opaqueHardDiscFragmentModule =
        CreateShaderModule(device_, opaqueHardDiscFragmentShaderCode, "vkCreateShaderModule(point opaque hard disc fragment)");
    const auto fastBasicVertexModule =
        CreateShaderModule(device_, fastBasicVertexShaderCode, "vkCreateShaderModule(point fast basic vertex)");
    const auto fastBasicFragmentModule =
        CreateShaderModule(device_, fastBasicFragmentShaderCode, "vkCreateShaderModule(point fast basic fragment)");
    const auto surfelVertexModule =
        CreateShaderModule(device_, surfelVertexShaderCode, "vkCreateShaderModule(surfel vertex)");
    const auto surfelDepthFragmentModule =
        CreateShaderModule(device_, surfelDepthFragmentShaderCode, "vkCreateShaderModule(surfel depth fragment)");
    const auto surfelAccumulationFragmentModule =
        CreateShaderModule(device_, surfelAccumulationFragmentShaderCode, "vkCreateShaderModule(surfel accumulation fragment)");
    const auto surfelConstantSimpleVertexModule =
        CreateShaderModule(device_, surfelConstantSimpleVertexShaderCode, "vkCreateShaderModule(surfel simple vertex)");
    const auto surfelConstantSimpleFragmentModule =
        CreateShaderModule(
            device_,
            surfelConstantSimpleFragmentShaderCode,
            "vkCreateShaderModule(surfel simple accumulation fragment)");
    const auto surfelOpaqueHardDiscFragmentModule =
        CreateShaderModule(
            device_,
            surfelOpaqueHardDiscFragmentShaderCode,
            "vkCreateShaderModule(surfel opaque hard disc fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelVertexStage.module = surfelVertexModule;
    surfelVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo constantSimpleVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    constantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    constantSimpleVertexStage.module = constantSimpleVertexModule;
    constantSimpleVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fastBasicVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fastBasicVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    fastBasicVertexStage.module = fastBasicVertexModule;
    fastBasicVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelConstantSimpleVertexStage{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelConstantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelConstantSimpleVertexStage.module = surfelConstantSimpleVertexModule;
    surfelConstantSimpleVertexStage.pName = "main";

    const std::array<VkVertexInputBindingDescription, 2> bindingDescriptions = {
        VkVertexInputBindingDescription{0, sizeof(invisible_places::io::Float3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(std::uint32_t), VK_VERTEX_INPUT_RATE_VERTEX},
    };

    const std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R8G8B8A8_UNORM, 0},
    };

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInputInfo.vertexBindingDescriptionCount = static_cast<std::uint32_t>(bindingDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

    VkPipelineVertexInputStateCreateInfo surfelVertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo surfelInputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    surfelInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &pointDescriptorSetLayout_;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pointPipelineLayout_), "vkCreatePipelineLayout(point)");

    auto createPointPipeline =
        [&](const VkPipelineShaderStageCreateInfo& selectedVertexStage,
            const VkPipelineVertexInputStateCreateInfo& selectedVertexInputInfo,
            const VkPipelineInputAssemblyStateCreateInfo& selectedInputAssembly,
            VkShaderModule fragmentModule,
            std::uint32_t subpass,
            const std::vector<VkPipelineColorBlendAttachmentState>& blendAttachments,
            bool depthTest,
            bool depthWrite,
            VkCompareOp depthCompare,
            const char* label,
            VkPipeline* pipeline) {
            VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
            fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            fragmentStage.module = fragmentModule;
            fragmentStage.pName = "main";
            const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {
                selectedVertexStage,
                fragmentStage};

            VkPipelineDepthStencilStateCreateInfo depthStencil{
                VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
            depthStencil.depthTestEnable = depthTest ? VK_TRUE : VK_FALSE;
            depthStencil.depthWriteEnable = depthWrite ? VK_TRUE : VK_FALSE;
            depthStencil.depthCompareOp = depthCompare;

            VkPipelineColorBlendStateCreateInfo colorBlending{
                VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
            colorBlending.attachmentCount = static_cast<std::uint32_t>(blendAttachments.size());
            colorBlending.pAttachments = blendAttachments.data();

            VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
            pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
            pipelineInfo.pStages = shaderStages.data();
            pipelineInfo.pVertexInputState = &selectedVertexInputInfo;
            pipelineInfo.pInputAssemblyState = &selectedInputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlending;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = pointPipelineLayout_;
            pipelineInfo.renderPass = renderPass_;
            pipelineInfo.subpass = subpass;

            Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, pipeline), label);
        };

    VkPipelineColorBlendAttachmentState linearDepthBlend{};
    linearDepthBlend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
    linearDepthBlend.blendEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState opaqueColorBlend{};
    opaqueColorBlend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    opaqueColorBlend.blendEnable = VK_FALSE;

    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        depthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(point depth prepass)",
        &pointDepthPrepassPipeline_);

    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        accumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(point accumulation)",
        &pointAccumulationPipeline_);

    createPointPipeline(
        constantSimpleVertexStage,
        vertexInputInfo,
        inputAssembly,
        constantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(point simple accumulation)",
        &pointConstantSimpleAccumulationPipeline_);

    createPointPipeline(
        constantSimpleVertexStage,
        vertexInputInfo,
        inputAssembly,
        opaqueHardDiscFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{opaqueColorBlend},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(point opaque hard disc)",
        &pointOpaqueHardDiscPipeline_);

    createPointPipeline(
        fastBasicVertexStage,
        vertexInputInfo,
        inputAssembly,
        fastBasicFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{opaqueColorBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(point fast basic)",
        &pointFastBasicPipeline_);

    createPointPipeline(
        surfelVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelDepthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(surfel depth prepass)",
        &surfelDepthPrepassPipeline_);

    createPointPipeline(
        surfelVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelAccumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(surfel accumulation)",
        &surfelAccumulationPipeline_);

    createPointPipeline(
        surfelConstantSimpleVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelConstantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(surfel simple accumulation)",
        &surfelConstantSimpleAccumulationPipeline_);

    createPointPipeline(
        surfelConstantSimpleVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelOpaqueHardDiscFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{opaqueColorBlend},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(surfel opaque hard disc)",
        &surfelOpaqueHardDiscPipeline_);

    vkDestroyShaderModule(device_, surfelOpaqueHardDiscFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelConstantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelConstantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, surfelAccumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelDepthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelVertexModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicVertexModule, nullptr);
    vkDestroyShaderModule(device_, opaqueHardDiscFragmentModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, accumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, depthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateExrExportResources(std::uint32_t width, std::uint32_t height) {
    if (width == 0 || height == 0) {
        throw std::runtime_error{"GPU EXR export requires a non-zero frame size."};
    }

    constexpr VkFormat kExportColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    constexpr VkFormat kExportLinearDepthFormat = VK_FORMAT_R32_SFLOAT;
    if (!FormatSupportsOptimalFeatures(
            physicalDevice_,
            kExportColorFormat,
            VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT | VK_FORMAT_FEATURE_TRANSFER_SRC_BIT)) {
        throw std::runtime_error{"GPU EXR export requires RGBA16F color attachment readback support."};
    }
    if (!FormatSupportsOptimalFeatures(
            physicalDevice_,
            kExportLinearDepthFormat,
            VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT | VK_FORMAT_FEATURE_TRANSFER_SRC_BIT)) {
        throw std::runtime_error{"GPU EXR export requires R32F depth attachment readback support."};
    }

    WaitIdle();
    CleanupExrExportResources();

    auto& resources = exrExportResources_;
    resources.width = width;
    resources.height = height;

    CreateExrExportRenderPass(&resources);
    CreateExrExportPipelines(&resources);

    resources.colorImage = CreateAttachmentImage(
        width,
        height,
        kExportColorFormat,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.depthImage = CreateAttachmentImage(
        width,
        height,
        depthFormat_,
        VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_DEPTH_BIT);
    resources.accumulationImage = CreateAttachmentImage(
        width,
        height,
        accumulationFormat_,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.revealageImage = CreateAttachmentImage(
        width,
        height,
        revealageFormat_,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.emissiveImage = CreateAttachmentImage(
        width,
        height,
        accumulationFormat_,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.normalAccumulationImage = CreateAttachmentImage(
        width,
        height,
        accumulationFormat_,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.albedoAccumulationImage = CreateAttachmentImage(
        width,
        height,
        accumulationFormat_,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.linearDepthImage = CreateAttachmentImage(
        width,
        height,
        kExportLinearDepthFormat,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.normalImage = CreateAttachmentImage(
        width,
        height,
        kExportColorFormat,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    resources.albedoImage = CreateAttachmentImage(
        width,
        height,
        kExportColorFormat,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);

    const std::array<VkImageView, 10> attachments = {
        resources.colorImage.view,
        resources.depthImage.view,
        resources.accumulationImage.view,
        resources.revealageImage.view,
        resources.emissiveImage.view,
        resources.linearDepthImage.view,
        resources.normalAccumulationImage.view,
        resources.albedoAccumulationImage.view,
        resources.normalImage.view,
        resources.albedoImage.view,
    };

    VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
    framebufferInfo.renderPass = resources.renderPass;
    framebufferInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
    framebufferInfo.pAttachments = attachments.data();
    framebufferInfo.width = width;
    framebufferInfo.height = height;
    framebufferInfo.layers = 1;
    Check(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &resources.framebuffer), "vkCreateFramebuffer(exr)");

    resources.colorReadbackBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) * 4U * sizeof(std::uint16_t),
        VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    resources.depthReadbackBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) * sizeof(float),
        VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    resources.normalReadbackBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) * 4U * sizeof(std::uint16_t),
        VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    resources.albedoReadbackBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) * 4U * sizeof(std::uint16_t),
        VK_BUFFER_USAGE_TRANSFER_DST_BIT);

    VkCommandBufferAllocateInfo commandBufferInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    commandBufferInfo.commandPool = commandPool_;
    commandBufferInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    commandBufferInfo.commandBufferCount = 1;
    Check(
        vkAllocateCommandBuffers(device_, &commandBufferInfo, &resources.commandBuffer),
        "vkAllocateCommandBuffers(exr)");

    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    Check(vkCreateFence(device_, &fenceInfo, nullptr, &resources.fence), "vkCreateFence(exr)");

    CreateOrUpdateCompositeDescriptorSet(
        &resources.compositeDescriptorSet,
        resources.accumulationImage.view,
        resources.revealageImage.view,
        resources.emissiveImage.view,
        resources.normalAccumulationImage.view,
        resources.albedoAccumulationImage.view);
}

void VulkanViewportShell::CreateExrExportRenderPass(ExrExportResources* resources) {
    if (resources == nullptr) {
        return;
    }

    constexpr VkFormat kExportColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    constexpr VkFormat kExportLinearDepthFormat = VK_FORMAT_R32_SFLOAT;

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = kExportColorFormat;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = depthFormat_;
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription accumulationAttachment{};
    accumulationAttachment.format = accumulationFormat_;
    accumulationAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    accumulationAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    accumulationAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    accumulationAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    accumulationAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription revealageAttachment{};
    revealageAttachment.format = revealageFormat_;
    revealageAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    revealageAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    revealageAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    revealageAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    revealageAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription emissiveAttachment{};
    emissiveAttachment.format = accumulationFormat_;
    emissiveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    emissiveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    emissiveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    emissiveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    emissiveAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription aovAccumulationAttachment = emissiveAttachment;

    VkAttachmentDescription linearDepthAttachment{};
    linearDepthAttachment.format = kExportLinearDepthFormat;
    linearDepthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    linearDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    linearDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    linearDepthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    linearDepthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    linearDepthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    linearDepthAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

    VkAttachmentDescription aovOutputAttachment = colorAttachment;

    VkAttachmentReference linearDepthColorRef{5, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference finalColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference normalOutputRef{8, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference albedoOutputRef{9, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthReadOnlyAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription depthSubpass{};
    depthSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    depthSubpass.colorAttachmentCount = 1;
    depthSubpass.pColorAttachments = &linearDepthColorRef;
    depthSubpass.pDepthStencilAttachment = &depthAttachmentRef;

    VkAttachmentReference accumulationColorRefs[5]{};
    accumulationColorRefs[0] = {2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[1] = {3, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[2] = {4, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[3] = {6, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[4] = {7, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthInputRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription accumulationSubpass{};
    accumulationSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    accumulationSubpass.colorAttachmentCount = 5;
    accumulationSubpass.pColorAttachments = accumulationColorRefs;
    accumulationSubpass.inputAttachmentCount = 1;
    accumulationSubpass.pInputAttachments = &depthInputRef;
    accumulationSubpass.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    VkAttachmentReference compositeInputRefs[5]{};
    compositeInputRefs[0] = {2, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[1] = {3, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[2] = {4, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[3] = {6, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[4] = {7, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};

    VkAttachmentReference compositeColorRefs[3]{};
    compositeColorRefs[0] = finalColorRef;
    compositeColorRefs[1] = normalOutputRef;
    compositeColorRefs[2] = albedoOutputRef;

    VkSubpassDescription compositeSubpass{};
    compositeSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    compositeSubpass.colorAttachmentCount = 3;
    compositeSubpass.pColorAttachments = compositeColorRefs;
    compositeSubpass.inputAttachmentCount = 5;
    compositeSubpass.pInputAttachments = compositeInputRefs;

    VkAttachmentReference fastBasicColorRefs[3]{};
    fastBasicColorRefs[0] = finalColorRef;
    fastBasicColorRefs[1] = normalOutputRef;
    fastBasicColorRefs[2] = albedoOutputRef;

    VkSubpassDescription fastBasicSubpass{};
    fastBasicSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    fastBasicSubpass.colorAttachmentCount = 3;
    fastBasicSubpass.pColorAttachments = fastBasicColorRefs;
    fastBasicSubpass.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    std::array<VkSubpassDependency, 7> dependencies{};
    dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[0].dstSubpass = 0;
    dependencies[0].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    dependencies[1].srcSubpass = 0;
    dependencies[1].dstSubpass = 1;
    dependencies[1].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[1].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[2].srcSubpass = 1;
    dependencies[2].dstSubpass = 2;
    dependencies[2].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[2].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[2].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[2].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[3].srcSubpass = 0;
    dependencies[3].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[3].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependencies[3].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[3].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    dependencies[4].srcSubpass = 2;
    dependencies[4].dstSubpass = 3;
    dependencies[4].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[4].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[5].srcSubpass = 0;
    dependencies[5].dstSubpass = 3;
    dependencies[5].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[5].dstStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[5].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[5].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[6].srcSubpass = 3;
    dependencies[6].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[6].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[6].dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependencies[6].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[6].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    const std::array<VkAttachmentDescription, 10> attachments = {
        colorAttachment,
        depthAttachment,
        accumulationAttachment,
        revealageAttachment,
        emissiveAttachment,
        linearDepthAttachment,
        aovAccumulationAttachment,
        aovAccumulationAttachment,
        aovOutputAttachment,
        aovOutputAttachment,
    };
    const std::array<VkSubpassDescription, 4> subpasses = {
        depthSubpass,
        accumulationSubpass,
        compositeSubpass,
        fastBasicSubpass,
    };

    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = static_cast<std::uint32_t>(subpasses.size());
    renderPassInfo.pSubpasses = subpasses.data();
    renderPassInfo.dependencyCount = static_cast<std::uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();
    Check(vkCreateRenderPass(device_, &renderPassInfo, nullptr, &resources->renderPass), "vkCreateRenderPass(exr)");
}

void VulkanViewportShell::CreateExrExportPipelines(ExrExportResources* resources) {
    if (resources == nullptr || resources->renderPass == VK_NULL_HANDLE) {
        return;
    }

    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.vert.spv").string());
    const auto accumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_exr_accumulation.frag.spv").string());
    const auto constantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_constant_simple.vert.spv").string());
    const auto constantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_exr_constant_simple_accumulation.frag.spv").string());
    const auto fastBasicVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.vert.spv").string());
    const auto fastBasicFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.frag.spv").string());
    const auto fastBasicDepthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic_depth.frag.spv").string());
    const auto depthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_export_depth.frag.spv").string());
    const auto surfelVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel.vert.spv").string());
    const auto surfelAccumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_exr_accumulation.frag.spv").string());
    const auto surfelConstantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_constant_simple.vert.spv").string());
    const auto surfelConstantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_exr_constant_simple_accumulation.frag.spv").string());
    const auto surfelDepthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_export_depth.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(exr point vertex)");
    const auto accumulationFragmentModule =
        CreateShaderModule(device_, accumulationFragmentShaderCode, "vkCreateShaderModule(exr point accumulation fragment)");
    const auto constantSimpleVertexModule =
        CreateShaderModule(device_, constantSimpleVertexShaderCode, "vkCreateShaderModule(exr point simple vertex)");
    const auto constantSimpleFragmentModule =
        CreateShaderModule(
            device_,
            constantSimpleFragmentShaderCode,
            "vkCreateShaderModule(exr point simple accumulation fragment)");
    const auto fastBasicVertexModule =
        CreateShaderModule(device_, fastBasicVertexShaderCode, "vkCreateShaderModule(exr point fast basic vertex)");
    const auto fastBasicFragmentModule =
        CreateShaderModule(device_, fastBasicFragmentShaderCode, "vkCreateShaderModule(exr point fast basic fragment)");
    const auto fastBasicDepthFragmentModule =
        CreateShaderModule(
            device_,
            fastBasicDepthFragmentShaderCode,
            "vkCreateShaderModule(exr point fast basic depth fragment)");
    const auto depthFragmentModule =
        CreateShaderModule(device_, depthFragmentShaderCode, "vkCreateShaderModule(exr point depth fragment)");
    const auto surfelVertexModule =
        CreateShaderModule(device_, surfelVertexShaderCode, "vkCreateShaderModule(exr surfel vertex)");
    const auto surfelAccumulationFragmentModule =
        CreateShaderModule(device_, surfelAccumulationFragmentShaderCode, "vkCreateShaderModule(exr surfel accumulation fragment)");
    const auto surfelConstantSimpleVertexModule =
        CreateShaderModule(device_, surfelConstantSimpleVertexShaderCode, "vkCreateShaderModule(exr surfel simple vertex)");
    const auto surfelConstantSimpleFragmentModule =
        CreateShaderModule(
            device_,
            surfelConstantSimpleFragmentShaderCode,
            "vkCreateShaderModule(exr surfel simple accumulation fragment)");
    const auto surfelDepthFragmentModule =
        CreateShaderModule(device_, surfelDepthFragmentShaderCode, "vkCreateShaderModule(exr surfel depth fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelVertexStage.module = surfelVertexModule;
    surfelVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo constantSimpleVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    constantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    constantSimpleVertexStage.module = constantSimpleVertexModule;
    constantSimpleVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fastBasicVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fastBasicVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    fastBasicVertexStage.module = fastBasicVertexModule;
    fastBasicVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelConstantSimpleVertexStage{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelConstantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelConstantSimpleVertexStage.module = surfelConstantSimpleVertexModule;
    surfelConstantSimpleVertexStage.pName = "main";

    const std::array<VkVertexInputBindingDescription, 2> bindingDescriptions = {
        VkVertexInputBindingDescription{0, sizeof(invisible_places::io::Float3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(std::uint32_t), VK_VERTEX_INPUT_RATE_VERTEX},
    };
    const std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R8G8B8A8_UNORM, 0},
    };

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInputInfo.vertexBindingDescriptionCount = static_cast<std::uint32_t>(bindingDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

    VkPipelineVertexInputStateCreateInfo surfelVertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo surfelInputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    surfelInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    auto createPointPipeline =
        [&](const VkPipelineShaderStageCreateInfo& selectedVertexStage,
            const VkPipelineVertexInputStateCreateInfo& selectedVertexInputInfo,
            const VkPipelineInputAssemblyStateCreateInfo& selectedInputAssembly,
            VkShaderModule fragmentModule,
            std::uint32_t subpass,
            const std::vector<VkPipelineColorBlendAttachmentState>& blendAttachments,
            bool depthTest,
            bool depthWrite,
            VkCompareOp depthCompare,
            const char* label,
            VkPipeline* pipeline) {
            VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
            fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            fragmentStage.module = fragmentModule;
            fragmentStage.pName = "main";
            const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {
                selectedVertexStage,
                fragmentStage};

            VkPipelineDepthStencilStateCreateInfo depthStencil{
                VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
            depthStencil.depthTestEnable = depthTest ? VK_TRUE : VK_FALSE;
            depthStencil.depthWriteEnable = depthWrite ? VK_TRUE : VK_FALSE;
            depthStencil.depthCompareOp = depthCompare;

            VkPipelineColorBlendStateCreateInfo colorBlending{
                VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
            colorBlending.attachmentCount = static_cast<std::uint32_t>(blendAttachments.size());
            colorBlending.pAttachments = blendAttachments.data();

            VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
            pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
            pipelineInfo.pStages = shaderStages.data();
            pipelineInfo.pVertexInputState = &selectedVertexInputInfo;
            pipelineInfo.pInputAssemblyState = &selectedInputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlending;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = pointPipelineLayout_;
            pipelineInfo.renderPass = resources->renderPass;
            pipelineInfo.subpass = subpass;
            Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, pipeline), label);
        };

    VkPipelineColorBlendAttachmentState linearDepthBlend{};
    linearDepthBlend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
    linearDepthBlend.blendEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState opaqueColorBlend{};
    opaqueColorBlend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    opaqueColorBlend.blendEnable = VK_FALSE;

    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        depthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(exr point depth)",
        &resources->pointDepthPipeline);
    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        accumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr point accumulation)",
        &resources->pointAccumulationPipeline);

    createPointPipeline(
        constantSimpleVertexStage,
        vertexInputInfo,
        inputAssembly,
        constantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr point simple accumulation)",
        &resources->pointConstantSimpleAccumulationPipeline);

    createPointPipeline(
        fastBasicVertexStage,
        vertexInputInfo,
        inputAssembly,
        fastBasicDepthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(exr point fast basic depth)",
        &resources->pointFastBasicDepthPipeline);

    createPointPipeline(
        fastBasicVertexStage,
        vertexInputInfo,
        inputAssembly,
        fastBasicFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{
            opaqueColorBlend,
            VkPipelineColorBlendAttachmentState{},
            VkPipelineColorBlendAttachmentState{}},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(exr point fast basic)",
        &resources->pointFastBasicPipeline);

    createPointPipeline(
        surfelVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelDepthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(exr surfel depth)",
        &resources->surfelDepthPipeline);
    createPointPipeline(
        surfelVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelAccumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr surfel accumulation)",
        &resources->surfelAccumulationPipeline);

    createPointPipeline(
        surfelConstantSimpleVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelConstantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr surfel simple accumulation)",
        &resources->surfelConstantSimpleAccumulationPipeline);

    vkDestroyShaderModule(device_, surfelConstantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelConstantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, surfelDepthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelAccumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelVertexModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicDepthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicVertexModule, nullptr);
    vkDestroyShaderModule(device_, depthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, accumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);

    const auto compositeVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.vert.spv").string());
    const auto compositeFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_exr_composite.frag.spv").string());
    const auto compositeVertexModule =
        CreateShaderModule(device_, compositeVertexShaderCode, "vkCreateShaderModule(exr composite vertex)");
    const auto compositeFragmentModule =
        CreateShaderModule(device_, compositeFragmentShaderCode, "vkCreateShaderModule(exr composite fragment)");

    VkPipelineShaderStageCreateInfo compositeVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    compositeVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    compositeVertexStage.module = compositeVertexModule;
    compositeVertexStage.pName = "main";
    VkPipelineShaderStageCreateInfo compositeFragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    compositeFragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    compositeFragmentStage.module = compositeFragmentModule;
    compositeFragmentStage.pName = "main";
    const std::array<VkPipelineShaderStageCreateInfo, 2> compositeStages = {
        compositeVertexStage,
        compositeFragmentStage,
    };

    VkPipelineVertexInputStateCreateInfo compositeVertexInput{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo compositeInputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    compositeInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineDepthStencilStateCreateInfo compositeDepthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    compositeDepthStencil.depthTestEnable = VK_FALSE;
    compositeDepthStencil.depthWriteEnable = VK_FALSE;
    const std::array<VkPipelineColorBlendAttachmentState, 3> compositeBlendAttachments = {
        MakeAlphaBlendAttachment(),
        VkPipelineColorBlendAttachmentState{
            VK_FALSE,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT},
        VkPipelineColorBlendAttachmentState{
            VK_FALSE,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT},
    };
    VkPipelineColorBlendStateCreateInfo compositeColorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    compositeColorBlending.attachmentCount = static_cast<std::uint32_t>(compositeBlendAttachments.size());
    compositeColorBlending.pAttachments = compositeBlendAttachments.data();

    VkGraphicsPipelineCreateInfo compositePipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    compositePipelineInfo.stageCount = static_cast<std::uint32_t>(compositeStages.size());
    compositePipelineInfo.pStages = compositeStages.data();
    compositePipelineInfo.pVertexInputState = &compositeVertexInput;
    compositePipelineInfo.pInputAssemblyState = &compositeInputAssembly;
    compositePipelineInfo.pViewportState = &viewportState;
    compositePipelineInfo.pRasterizationState = &rasterizer;
    compositePipelineInfo.pMultisampleState = &multisampling;
    compositePipelineInfo.pDepthStencilState = &compositeDepthStencil;
    compositePipelineInfo.pColorBlendState = &compositeColorBlending;
    compositePipelineInfo.pDynamicState = &dynamicState;
    compositePipelineInfo.layout = compositePipelineLayout_;
    compositePipelineInfo.renderPass = resources->renderPass;
    compositePipelineInfo.subpass = 2;
    Check(
        vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &compositePipelineInfo,
            nullptr,
            &resources->compositePipeline),
        "vkCreateGraphicsPipelines(exr composite)");

    vkDestroyShaderModule(device_, compositeFragmentModule, nullptr);
    vkDestroyShaderModule(device_, compositeVertexModule, nullptr);
}

void VulkanViewportShell::CreateGaussianSplatPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_accumulation.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_accumulation.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(gsplat vertex)");
    const auto fragmentModule = CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(gsplat fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_FALSE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

    const std::array<VkPipelineColorBlendAttachmentState, 3> colorBlendAttachments = {
        MakeAdditiveBlendAttachment(),
        MakeRevealageBlendAttachment(),
        MakeAdditiveBlendAttachment(),
    };
    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = static_cast<std::uint32_t>(colorBlendAttachments.size());
    colorBlending.pAttachments = colorBlendAttachments.data();

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.size = sizeof(GaussianSplatPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &gaussianSplatDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &gaussianSplatPipelineLayout_), "vkCreatePipelineLayout(gsplat)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = gaussianSplatPipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 1;

    Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &gaussianSplatPipeline_), "vkCreateGraphicsPipelines(gsplat)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateHighQualityGaussianSplatPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile(
            (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_high_quality.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile(
            (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_high_quality.frag.spv").string());

    const auto vertexModule =
        CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(gsplat_hq vertex)");
    const auto fragmentModule =
        CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(gsplat_hq fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_FALSE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

    const auto colorBlendAttachment = MakePremultipliedAlphaBlendAttachment();
    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.size = sizeof(HighQualityGaussianPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &highQualityGaussianSplatDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(
        vkCreatePipelineLayout(
            device_,
            &pipelineLayoutInfo,
            nullptr,
            &highQualityGaussianSplatPipelineLayout_),
        "vkCreatePipelineLayout(gsplat_hq)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = highQualityGaussianSplatPipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 3;

    Check(
        vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &pipelineInfo,
            nullptr,
            &highQualityGaussianSplatPipeline_),
        "vkCreateGraphicsPipelines(gsplat_hq)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateCompositePipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(composite vertex)");
    const auto fragmentModule = CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(composite fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_FALSE;
    depthStencil.depthWriteEnable = VK_FALSE;

    const auto colorBlendAttachment = MakeAlphaBlendAttachment();
    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &compositeDescriptorSetLayout_;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &compositePipelineLayout_), "vkCreatePipelineLayout(composite)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = compositePipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 2;

    Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &compositePipeline_), "vkCreateGraphicsPipelines(composite)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreatePostProcessPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "edl_postprocess.frag.spv").string());

    const auto vertexModule =
        CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(postprocess vertex)");
    const auto fragmentModule =
        CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(postprocess fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_FALSE;
    depthStencil.depthWriteEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_FALSE;

    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(PostProcessPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &postProcessDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(
        vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &postProcessPipelineLayout_),
        "vkCreatePipelineLayout(postprocess)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = postProcessPipelineLayout_;
    pipelineInfo.renderPass = presentRenderPass_;
    pipelineInfo.subpass = 0;

    Check(
        vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &postProcessPipeline_),
        "vkCreateGraphicsPipelines(postprocess)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateGpuDrivenSelectionPipeline() {
    if (!gpuDrivenSelectionCapabilities_.computeQueueSupported ||
        gpuDrivenSelectionDescriptorSetLayout_ == VK_NULL_HANDLE) {
        return;
    }

    const auto computeShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_indirect_command.comp.spv").string());
    const auto computeModule =
        CreateShaderModule(device_, computeShaderCode, "vkCreateShaderModule(pointcloud indirect command compute)");

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(GpuDrivenIndirectCommandPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &gpuDrivenSelectionDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(
        vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &gpuDrivenSelectionPipelineLayout_),
        "vkCreatePipelineLayout(pointcloud gpu driven selection)");

    VkPipelineShaderStageCreateInfo computeStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    computeStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    computeStage.module = computeModule;
    computeStage.pName = "main";

    VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipelineInfo.stage = computeStage;
    pipelineInfo.layout = gpuDrivenSelectionPipelineLayout_;

    Check(
        vkCreateComputePipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &pipelineInfo,
            nullptr,
            &gpuDrivenIndirectCommandPipeline_),
        "vkCreateComputePipelines(pointcloud indirect command)");

    vkDestroyShaderModule(device_, computeModule, nullptr);
}

void VulkanViewportShell::CreateGpuCompactionPipeline() {
    if (!gpuDrivenSelectionCapabilities_.computeQueueSupported ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE) {
        return;
    }

    const auto compactionShaderName = kGpuDiagnosticSelectionFrustumGuardEnabled
                                          ? "pointcloud_draw_item_compact.comp.spv"
                                          : "pointcloud_draw_item_compact_metadata.comp.spv";
    const auto computeShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / compactionShaderName).string());
    const auto computeModule =
        CreateShaderModule(device_, computeShaderCode, "vkCreateShaderModule(pointcloud draw item compaction compute)");

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(GpuDrawItemCompactionPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(
        vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &gpuCompactionPipelineLayout_),
        "vkCreatePipelineLayout(pointcloud gpu compaction)");

    VkPipelineShaderStageCreateInfo computeStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    computeStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    computeStage.module = computeModule;
    computeStage.pName = "main";

    VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipelineInfo.stage = computeStage;
    pipelineInfo.layout = gpuCompactionPipelineLayout_;

    Check(
        vkCreateComputePipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &pipelineInfo,
            nullptr,
            &gpuDrawItemCompactionPipeline_),
        "vkCreateComputePipelines(pointcloud draw item compaction)");

    vkDestroyShaderModule(device_, computeModule, nullptr);
}

void VulkanViewportShell::CreateFramebuffers() {
    framebuffers_.clear();
    framebuffers_.reserve(sceneColorImages_.size());

    for (std::size_t imageIndex = 0; imageIndex < sceneColorImages_.size(); ++imageIndex) {
        const std::array<VkImageView, 6> attachments = {
            sceneColorImages_[imageIndex].view,
            depthImages_[imageIndex].view,
            accumulationImages_[imageIndex].view,
            revealageImages_[imageIndex].view,
            emissiveImages_[imageIndex].view,
            linearDepthImages_[imageIndex].view,
        };

        VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebufferInfo.renderPass = renderPass_;
        framebufferInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = swapchainWidth_;
        framebufferInfo.height = swapchainHeight_;
        framebufferInfo.layers = 1;

        VkFramebuffer framebuffer = VK_NULL_HANDLE;
        Check(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &framebuffer), "vkCreateFramebuffer");
        framebuffers_.push_back(framebuffer);
    }
}

void VulkanViewportShell::CreatePresentFramebuffers() {
    presentFramebuffers_.clear();
    presentFramebuffers_.reserve(imageViews_.size());

    for (const auto imageView : imageViews_) {
        VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebufferInfo.renderPass = presentRenderPass_;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = &imageView;
        framebufferInfo.width = swapchainWidth_;
        framebufferInfo.height = swapchainHeight_;
        framebufferInfo.layers = 1;

        VkFramebuffer framebuffer = VK_NULL_HANDLE;
        Check(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &framebuffer), "vkCreateFramebuffer(present)");
        presentFramebuffers_.push_back(framebuffer);
    }
}

void VulkanViewportShell::CreateSceneColorResources() {
    for (auto& image : sceneColorImages_) {
        DestroyImage(&image);
    }
    sceneColorImages_.clear();
    sceneImageRevisions_.clear();
    sceneColorImages_.reserve(swapchainImages_.size());
    sceneImageRevisions_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        sceneColorImages_.push_back(CreateAttachmentImage(
            swapchainImageFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
        sceneImageRevisions_.push_back(0);
    }
}

void VulkanViewportShell::CreateDepthResources() {
    for (auto& image : depthImages_) {
        DestroyImage(&image);
    }
    depthImages_.clear();
    depthFormat_ = SelectDepthFormat();
    depthImages_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        depthImages_.push_back(CreateAttachmentImage(
            depthFormat_,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT));
    }
}

void VulkanViewportShell::CreateAccumulationResources() {
    for (auto& image : accumulationImages_) {
        DestroyImage(&image);
    }
    for (auto& image : revealageImages_) {
        DestroyImage(&image);
    }
    for (auto& image : emissiveImages_) {
        DestroyImage(&image);
    }
    accumulationImages_.clear();
    revealageImages_.clear();
    emissiveImages_.clear();

    accumulationImages_.reserve(swapchainImages_.size());
    revealageImages_.reserve(swapchainImages_.size());
    emissiveImages_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        accumulationImages_.push_back(CreateAttachmentImage(
            accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
        revealageImages_.push_back(CreateAttachmentImage(
            revealageFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
        emissiveImages_.push_back(CreateAttachmentImage(
            accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
    }
}

void VulkanViewportShell::CreateLinearDepthResources() {
    for (auto& image : linearDepthImages_) {
        DestroyImage(&image);
    }
    linearDepthImages_.clear();
    linearDepthImages_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        linearDepthImages_.push_back(CreateAttachmentImage(
            linearDepthFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
    }
}

void VulkanViewportShell::CreateCommandPool() {
    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = graphicsQueueFamily_;

    Check(vkCreateCommandPool(device_, &poolInfo, nullptr, &commandPool_), "vkCreateCommandPool");
}

void VulkanViewportShell::CreateCommandBuffers() {
    for (auto& frame : frameResources_) {
        if (frame.commandBuffer != VK_NULL_HANDLE) {
            vkFreeCommandBuffers(device_, commandPool_, 1, &frame.commandBuffer);
            frame.commandBuffer = VK_NULL_HANDLE;
        }
    }

    std::array<VkCommandBuffer, kFramesInFlight> commandBuffers{};

    VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocInfo.commandPool = commandPool_;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = static_cast<std::uint32_t>(commandBuffers.size());

    Check(vkAllocateCommandBuffers(device_, &allocInfo, commandBuffers.data()), "vkAllocateCommandBuffers");
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        frameResources_[frameIndex].commandBuffer = commandBuffers[frameIndex];
    }
}

void VulkanViewportShell::CreateSyncObjects() {
    VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (auto& frame : frameResources_) {
        Check(
            vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &frame.imageAvailableSemaphore),
            "vkCreateSemaphore(imageAvailable)");
        Check(
            vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &frame.renderFinishedSemaphore),
            "vkCreateSemaphore(renderFinished)");
        Check(vkCreateFence(device_, &fenceInfo, nullptr, &frame.fence), "vkCreateFence");
        if (gpuTimestampsSupported_) {
            VkQueryPoolCreateInfo queryPoolInfo{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
            queryPoolInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
            queryPoolInfo.queryCount = kGpuTimestampQueryCount;
            const VkResult queryResult =
                vkCreateQueryPool(device_, &queryPoolInfo, nullptr, &frame.timestampQueryPool);
            if (queryResult != VK_SUCCESS) {
                frame.timestampQueryPool = VK_NULL_HANDLE;
                gpuTimestampsSupported_ = false;
                diagnostics_.gpuTimestampSupported = false;
                diagnostics_.gpuTimestampState = "query pool unavailable";
            }
        }
    }
}

void VulkanViewportShell::ResetGpuCompactionPerformanceFrame(std::size_t frameIndex) {
    if (frameIndex >= gpuCompactionCpuReferenceMsByFrame_.size()) {
        return;
    }
    gpuCompactionCpuReferenceMsByFrame_[frameIndex].fill(0.0);
}

void VulkanViewportShell::DecayGpuCompactionPerformanceCooldowns() {
    for (auto& gate : gpuCompactionPerformanceGates_) {
        if (gate.retryCooldownFrames > 0U) {
            --gate.retryCooldownFrames;
        }
    }
}

std::size_t VulkanViewportShell::GpuCompactionPerformanceProfileIndex(
    renderer::pointcloud::PointCloudLodRendererCostProfile profile) {
    return std::min<std::size_t>(
        static_cast<std::size_t>(profile),
        kGpuCompactionPerformanceProfileCount - 1U);
}

std::string VulkanViewportShell::GpuCompactionPerformanceFallbackReason(
    renderer::pointcloud::PointCloudLodRendererCostProfile profile) const {
    const auto profileIndex = GpuCompactionPerformanceProfileIndex(profile);
    const auto& gate = gpuCompactionPerformanceGates_[profileIndex];
    if (gate.retryCooldownFrames == 0U) {
        return {};
    }
    const auto slowSamplePhrase =
        kGpuDiagnosticCompactionSlowFrameThreshold == 1U
            ? std::string{"a slower "}
            : (std::to_string(kGpuDiagnosticCompactionSlowFrameThreshold) + " consecutive slower ");
    return std::string{"GPU full-range compaction was slower than the CPU reference after "} +
           slowSamplePhrase + renderer::pointcloud::PointCloudLodRendererCostProfileName(profile) +
           " sample (last CPU " + std::to_string(gate.lastCpuReferenceMs) +
           " ms, GPU " + std::to_string(gate.lastGpuMs) +
           " ms); compare pass suspended until the retry window reopens";
}

void VulkanViewportShell::UpdateGpuCompactionPerformanceGate(std::size_t frameIndex, double gpuMs) {
    if (frameIndex >= gpuCompactionCpuReferenceMsByFrame_.size() || gpuMs <= 0.0) {
        return;
    }
    for (std::size_t profileIndex = 0; profileIndex < kGpuCompactionPerformanceProfileCount; ++profileIndex) {
        const double cpuReferenceMs = gpuCompactionCpuReferenceMsByFrame_[frameIndex][profileIndex];
        if (cpuReferenceMs <= 0.0) {
            continue;
        }
        auto& gate = gpuCompactionPerformanceGates_[profileIndex];
        gate.lastCpuReferenceMs = cpuReferenceMs;
        gate.lastGpuMs = gpuMs;
        if (gpuMs > cpuReferenceMs + kGpuDiagnosticCompactionTimingEpsilonMs) {
            ++gate.consecutiveSlowerFrames;
            if (gate.consecutiveSlowerFrames >= kGpuDiagnosticCompactionSlowFrameThreshold) {
                gate.retryCooldownFrames = kGpuDiagnosticCompactionRetryCooldownFrames;
                gate.consecutiveSlowerFrames = 0U;
            }
        } else {
            gate.consecutiveSlowerFrames = 0U;
            gate.retryCooldownFrames = 0U;
        }
    }
}

void VulkanViewportShell::ReadPreviousGpuTimestampResults(FrameResources* frame, std::size_t frameIndex) {
    diagnostics_.gpuTimestampSupported = gpuTimestampsSupported_;
    diagnostics_.gpuTimestampTimingValid = false;
    diagnostics_.gpuFastBasicPointPassMs = 0.0;
    diagnostics_.gpuBeautyDepthPassMs = 0.0;
    diagnostics_.gpuBeautyPointPassMs = 0.0;
    diagnostics_.gpuCompositePassMs = 0.0;
    diagnostics_.gpuPostProcessPassMs = 0.0;
    diagnostics_.adaptiveGpuCompactionMs = 0.0;
    diagnostics_.adaptiveGpuFeatureClassProbeMs = 0.0;
    diagnostics_.adaptiveGpuRankProbeMs = 0.0;
    diagnostics_.adaptiveGpuDepthProbeMs = 0.0;
    diagnostics_.adaptiveGpuProjectedAreaProbeMs = 0.0;
    diagnostics_.adaptiveGpuRenderAreaProbeMs = 0.0;
    diagnostics_.adaptiveGpuRepresentedCountProbeMs = 0.0;
    diagnostics_.adaptiveGpuCoverageCompensationProbeMs = 0.0;
    diagnostics_.adaptiveGpuClampFlagsProbeMs = 0.0;
    diagnostics_.adaptiveGpuIndirectCommandMs = 0.0;
    if (!gpuTimestampsSupported_ || frame == nullptr || frame->timestampQueryPool == VK_NULL_HANDLE) {
        diagnostics_.gpuTimestampState = "unavailable";
        return;
    }
    if (!frame->timestampQueriesArmed) {
        diagnostics_.gpuTimestampState = "warming up";
        return;
    }

    std::array<TimestampQueryResult, kGpuTimestampQueryCount> results{};
    const VkResult result = vkGetQueryPoolResults(
        device_,
        frame->timestampQueryPool,
        0,
        kGpuTimestampQueryCount,
        sizeof(results),
        results.data(),
        sizeof(TimestampQueryResult),
        VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WITH_AVAILABILITY_BIT);
    if (result != VK_SUCCESS && result != VK_NOT_READY) {
        diagnostics_.gpuTimestampState = "readback failed";
        return;
    }

    const auto passMilliseconds = [&](GpuTimestampPass pass) {
        const auto passIndex = static_cast<std::uint32_t>(pass);
        if (passIndex >= frame->timestampPassWritten.size() || !frame->timestampPassWritten[passIndex]) {
            return 0.0;
        }
        const auto beginIndex = passIndex * kGpuTimestampQueriesPerPass;
        const auto endIndex = beginIndex + 1U;
        if (results[beginIndex].available == 0U || results[endIndex].available == 0U ||
            results[endIndex].value < results[beginIndex].value) {
            return 0.0;
        }
        return static_cast<double>(results[endIndex].value - results[beginIndex].value) *
               static_cast<double>(gpuTimestampPeriodNs_) / 1'000'000.0;
    };

    diagnostics_.gpuFastBasicPointPassMs = passMilliseconds(kGpuTimestampFastBasicPointPass);
    diagnostics_.gpuBeautyDepthPassMs = passMilliseconds(kGpuTimestampBeautyDepthPass);
    diagnostics_.gpuBeautyPointPassMs = passMilliseconds(kGpuTimestampBeautyPointPass);
    diagnostics_.gpuCompositePassMs = passMilliseconds(kGpuTimestampCompositePass);
    diagnostics_.gpuPostProcessPassMs = passMilliseconds(kGpuTimestampPostProcessPass);
    diagnostics_.adaptiveGpuCompactionMs = passMilliseconds(kGpuTimestampGpuDrawItemCompactionPass);
    diagnostics_.adaptiveGpuFeatureClassProbeMs = passMilliseconds(kGpuTimestampGpuFeatureClassProbePass);
    diagnostics_.adaptiveGpuRankProbeMs = passMilliseconds(kGpuTimestampGpuRankProbePass);
    diagnostics_.adaptiveGpuDepthProbeMs = passMilliseconds(kGpuTimestampGpuDepthProbePass);
    diagnostics_.adaptiveGpuProjectedAreaProbeMs = passMilliseconds(kGpuTimestampGpuProjectedAreaProbePass);
    diagnostics_.adaptiveGpuRenderAreaProbeMs = passMilliseconds(kGpuTimestampGpuRenderAreaProbePass);
    diagnostics_.adaptiveGpuRepresentedCountProbeMs = passMilliseconds(kGpuTimestampGpuRepresentedCountProbePass);
    diagnostics_.adaptiveGpuCoverageCompensationProbeMs =
        passMilliseconds(kGpuTimestampGpuCoverageCompensationProbePass);
    diagnostics_.adaptiveGpuClampFlagsProbeMs = passMilliseconds(kGpuTimestampGpuClampFlagsProbePass);
    diagnostics_.adaptiveGpuIndirectCommandMs = passMilliseconds(kGpuTimestampGpuDrivenIndirectCommandPass);
    UpdateGpuCompactionPerformanceGate(frameIndex, diagnostics_.adaptiveGpuCompactionMs);
    diagnostics_.gpuTimestampTimingValid =
        diagnostics_.gpuFastBasicPointPassMs > 0.0 ||
        diagnostics_.gpuBeautyDepthPassMs > 0.0 ||
        diagnostics_.gpuBeautyPointPassMs > 0.0 ||
        diagnostics_.gpuCompositePassMs > 0.0 ||
        diagnostics_.gpuPostProcessPassMs > 0.0 ||
        diagnostics_.adaptiveGpuCompactionMs > 0.0 ||
        diagnostics_.adaptiveGpuFeatureClassProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuRankProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuDepthProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuProjectedAreaProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuRenderAreaProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuRepresentedCountProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuCoverageCompensationProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuClampFlagsProbeMs > 0.0 ||
        diagnostics_.adaptiveGpuIndirectCommandMs > 0.0;
    diagnostics_.gpuTimestampState =
        diagnostics_.gpuTimestampTimingValid ? "valid previous frame" : "waiting for written queries";
}

void VulkanViewportShell::ReadPreviousGpuCompactionResults(std::size_t frameIndex) {
    if (frameIndex >= kFramesInFlight) {
        return;
    }

    bool checked = false;
    bool passed = true;
    std::uint32_t cpuCount = 0;
    std::uint32_t gpuCount = 0;
    std::uint32_t cpuChecksum = 0;
    std::uint32_t gpuChecksum = 0;
    std::uint32_t cpuSourceFingerprint = 0;
    std::uint32_t gpuSourceFingerprint = 0;
    std::array<std::uint32_t, renderer::pointcloud::kPointCloudLodRepresentativeClassCount>
        cpuClassCounts{};
    std::array<std::uint32_t, renderer::pointcloud::kPointCloudLodRepresentativeClassCount>
        gpuClassCounts{};
    bool featureProbeChecked = false;
    bool featureProbePassed = true;
    std::uint32_t featureProbeCpuCount = 0;
    std::uint32_t featureProbeGpuCount = 0;
    std::uint32_t featureProbeCpuChecksum = 0;
    std::uint32_t featureProbeGpuChecksum = 0;
    std::uint32_t featureProbeCpuSourceFingerprint = 0;
    std::uint32_t featureProbeGpuSourceFingerprint = 0;
    bool rankProbeChecked = false;
    bool rankProbePassed = true;
    std::uint32_t rankProbeCpuCount = 0;
    std::uint32_t rankProbeGpuCount = 0;
    std::uint32_t rankProbeCpuChecksum = 0;
    std::uint32_t rankProbeGpuChecksum = 0;
    std::uint32_t rankProbeCpuSourceFingerprint = 0;
    std::uint32_t rankProbeGpuSourceFingerprint = 0;
    bool depthProbeChecked = false;
    bool depthProbePassed = true;
    std::uint32_t depthProbeCpuCount = 0;
    std::uint32_t depthProbeGpuCount = 0;
    std::uint32_t depthProbeCpuChecksum = 0;
    std::uint32_t depthProbeGpuChecksum = 0;
    std::uint32_t depthProbeCpuSourceFingerprint = 0;
    std::uint32_t depthProbeGpuSourceFingerprint = 0;
    bool projectedAreaProbeChecked = false;
    bool projectedAreaProbePassed = true;
    std::uint32_t projectedAreaProbeCpuCount = 0;
    std::uint32_t projectedAreaProbeGpuCount = 0;
    std::uint32_t projectedAreaProbeCpuChecksum = 0;
    std::uint32_t projectedAreaProbeGpuChecksum = 0;
    std::uint32_t projectedAreaProbeCpuSourceFingerprint = 0;
    std::uint32_t projectedAreaProbeGpuSourceFingerprint = 0;
    bool renderAreaProbeChecked = false;
    bool renderAreaProbePassed = true;
    std::uint32_t renderAreaProbeCpuCount = 0;
    std::uint32_t renderAreaProbeGpuCount = 0;
    std::uint32_t renderAreaProbeCpuChecksum = 0;
    std::uint32_t renderAreaProbeGpuChecksum = 0;
    std::uint32_t renderAreaProbeCpuSourceFingerprint = 0;
    std::uint32_t renderAreaProbeGpuSourceFingerprint = 0;
    bool representedCountProbeChecked = false;
    bool representedCountProbePassed = true;
    std::uint32_t representedCountProbeCpuCount = 0;
    std::uint32_t representedCountProbeGpuCount = 0;
    std::uint32_t representedCountProbeCpuChecksum = 0;
    std::uint32_t representedCountProbeGpuChecksum = 0;
    std::uint32_t representedCountProbeCpuSourceFingerprint = 0;
    std::uint32_t representedCountProbeGpuSourceFingerprint = 0;
    bool coverageCompensationProbeChecked = false;
    bool coverageCompensationProbePassed = true;
    std::uint32_t coverageCompensationProbeCpuCount = 0;
    std::uint32_t coverageCompensationProbeGpuCount = 0;
    std::uint32_t coverageCompensationProbeCpuChecksum = 0;
    std::uint32_t coverageCompensationProbeGpuChecksum = 0;
    std::uint32_t coverageCompensationProbeCpuSourceFingerprint = 0;
    std::uint32_t coverageCompensationProbeGpuSourceFingerprint = 0;
    bool clampFlagsProbeChecked = false;
    bool clampFlagsProbePassed = true;
    std::uint32_t clampFlagsProbeCpuCount = 0;
    std::uint32_t clampFlagsProbeGpuCount = 0;
    std::uint32_t clampFlagsProbeCpuChecksum = 0;
    std::uint32_t clampFlagsProbeGpuChecksum = 0;
    std::uint32_t clampFlagsProbeCpuSourceFingerprint = 0;
    std::uint32_t clampFlagsProbeGpuSourceFingerprint = 0;
    const auto foldSourceFingerprint = [](const GpuDrawItemCompactionStats& stats) {
        auto mixedSum = stats.sourceIndexSum;
        mixedSum ^= mixedSum >> 16U;
        mixedSum *= 0x7feb352dU;
        mixedSum ^= mixedSum >> 15U;
        mixedSum *= 0x846ca68bU;
        mixedSum ^= mixedSum >> 16U;
        return stats.sourceIndexXor ^ mixedSum;
    };
    bool outputProbeChecked = false;
    bool outputProbePassed = true;
    std::uint32_t outputProbeCpuCount = 0;
    std::uint32_t outputProbeGpuCount = 0;
    std::uint32_t outputProbeCpuChecksum = 0;
    std::uint32_t outputProbeGpuChecksum = 0;
    std::uint32_t outputProbeCpuSourceFingerprint = 0;
    std::uint32_t outputProbeGpuSourceFingerprint = 0;
    const auto foldOutputProbeFingerprint = [](const GpuDrawItemOutputProbeStats& stats) {
        auto mixedSum = stats.sourceIndexSum;
        mixedSum ^= mixedSum >> 16U;
        mixedSum *= 0x7feb352dU;
        mixedSum ^= mixedSum >> 15U;
        mixedSum *= 0x846ca68bU;
        mixedSum ^= mixedSum >> 16U;
        return stats.sourceIndexXor ^ stats.metadataXor ^ stats.orderChecksum ^ mixedSum;
    };
    bool indirectChecked = false;
    bool indirectPassed = true;
    std::uint32_t indirectCpuVertices = 0;
    std::uint32_t indirectGpuVertices = 0;
    for (auto& resources : pointCloudResources_) {
        if (!resources.gpuCompactionResultPending[frameIndex] ||
            resources.gpuCompactionStatsBuffers[frameIndex].mapped == nullptr) {
            const bool outputProbePendingAndMapped =
                resources.gpuCompactionOutputProbeResultPending[frameIndex] &&
                resources.gpuCompactedDrawItemBuffers[frameIndex].mapped != nullptr;
            const bool indirectPendingAndMapped =
                resources.gpuCompactionIndirectCommandResultPending[frameIndex] &&
                resources.gpuCompactionIndirectCommandBuffers[frameIndex].mapped != nullptr;
            const bool featureProbePendingAndMapped =
                resources.gpuFeatureClassProbeResultPending[frameIndex] &&
                resources.gpuFeatureClassProbeStatsBuffers[frameIndex].mapped != nullptr;
            const bool rankProbePendingAndMapped =
                resources.gpuRankProbeResultPending[frameIndex] &&
                resources.gpuRankProbeStatsBuffers[frameIndex].mapped != nullptr;
            const bool depthProbePendingAndMapped =
                resources.gpuDepthProbeResultPending[frameIndex] &&
                resources.gpuDepthProbeStatsBuffers[frameIndex].mapped != nullptr;
            const bool projectedAreaProbePendingAndMapped =
                resources.gpuProjectedAreaProbeResultPending[frameIndex] &&
                resources.gpuProjectedAreaProbeStatsBuffers[frameIndex].mapped != nullptr;
            const bool renderAreaProbePendingAndMapped =
                resources.gpuRenderAreaProbeResultPending[frameIndex] &&
                resources.gpuRenderAreaProbeStatsBuffers[frameIndex].mapped != nullptr;
            const bool representedCountProbePendingAndMapped =
                resources.gpuRepresentedCountProbeResultPending[frameIndex] &&
                resources.gpuRepresentedCountProbeStatsBuffers[frameIndex].mapped != nullptr;
            const bool coverageCompensationProbePendingAndMapped =
                resources.gpuCoverageCompensationProbeResultPending[frameIndex] &&
                resources.gpuCoverageCompensationProbeStatsBuffers[frameIndex].mapped != nullptr;
            const bool clampFlagsProbePendingAndMapped =
                resources.gpuClampFlagsProbeResultPending[frameIndex] &&
                resources.gpuClampFlagsProbeStatsBuffers[frameIndex].mapped != nullptr;
            if (!outputProbePendingAndMapped &&
                !indirectPendingAndMapped &&
                !featureProbePendingAndMapped &&
                !rankProbePendingAndMapped &&
                !depthProbePendingAndMapped &&
                !projectedAreaProbePendingAndMapped &&
                !renderAreaProbePendingAndMapped &&
                !representedCountProbePendingAndMapped &&
                !coverageCompensationProbePendingAndMapped &&
                !clampFlagsProbePendingAndMapped) {
                continue;
            }
        }

        if (resources.gpuCompactionResultPending[frameIndex] &&
            resources.gpuCompactionStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuCompactionStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuCompactionExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            checked = true;
            passed = passed && layerPassed;
            cpuCount += expected.count;
            gpuCount += actual.count;
            cpuChecksum ^= expected.combinedChecksum;
            gpuChecksum ^= actual.combinedChecksum;
            cpuSourceFingerprint ^= foldSourceFingerprint(expected);
            gpuSourceFingerprint ^= foldSourceFingerprint(actual);
            for (std::size_t classIndex = 0; classIndex < cpuClassCounts.size(); ++classIndex) {
                cpuClassCounts[classIndex] += expected.classCounts[classIndex];
                gpuClassCounts[classIndex] += actual.classCounts[classIndex];
            }
            resources.gpuCompactionResultPending[frameIndex] = false;
        }

        if (resources.gpuFeatureClassProbeResultPending[frameIndex] &&
            resources.gpuFeatureClassProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuFeatureClassProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuFeatureClassProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            featureProbeChecked = true;
            featureProbePassed = featureProbePassed && layerPassed;
            featureProbeCpuCount += expected.count;
            featureProbeGpuCount += actual.count;
            featureProbeCpuChecksum ^= expected.combinedChecksum;
            featureProbeGpuChecksum ^= actual.combinedChecksum;
            featureProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            featureProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuFeatureClassProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuRankProbeResultPending[frameIndex] &&
            resources.gpuRankProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuRankProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuRankProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            rankProbeChecked = true;
            rankProbePassed = rankProbePassed && layerPassed;
            rankProbeCpuCount += expected.count;
            rankProbeGpuCount += actual.count;
            rankProbeCpuChecksum ^= expected.combinedChecksum;
            rankProbeGpuChecksum ^= actual.combinedChecksum;
            rankProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            rankProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuRankProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuDepthProbeResultPending[frameIndex] &&
            resources.gpuDepthProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuDepthProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuDepthProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            depthProbeChecked = true;
            depthProbePassed = depthProbePassed && layerPassed;
            depthProbeCpuCount += expected.count;
            depthProbeGpuCount += actual.count;
            depthProbeCpuChecksum ^= expected.combinedChecksum;
            depthProbeGpuChecksum ^= actual.combinedChecksum;
            depthProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            depthProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuDepthProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuProjectedAreaProbeResultPending[frameIndex] &&
            resources.gpuProjectedAreaProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuProjectedAreaProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuProjectedAreaProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            projectedAreaProbeChecked = true;
            projectedAreaProbePassed = projectedAreaProbePassed && layerPassed;
            projectedAreaProbeCpuCount += expected.count;
            projectedAreaProbeGpuCount += actual.count;
            projectedAreaProbeCpuChecksum ^= expected.combinedChecksum;
            projectedAreaProbeGpuChecksum ^= actual.combinedChecksum;
            projectedAreaProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            projectedAreaProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuProjectedAreaProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuRenderAreaProbeResultPending[frameIndex] &&
            resources.gpuRenderAreaProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuRenderAreaProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuRenderAreaProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            renderAreaProbeChecked = true;
            renderAreaProbePassed = renderAreaProbePassed && layerPassed;
            renderAreaProbeCpuCount += expected.count;
            renderAreaProbeGpuCount += actual.count;
            renderAreaProbeCpuChecksum ^= expected.combinedChecksum;
            renderAreaProbeGpuChecksum ^= actual.combinedChecksum;
            renderAreaProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            renderAreaProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuRenderAreaProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuRepresentedCountProbeResultPending[frameIndex] &&
            resources.gpuRepresentedCountProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuRepresentedCountProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuRepresentedCountProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            representedCountProbeChecked = true;
            representedCountProbePassed = representedCountProbePassed && layerPassed;
            representedCountProbeCpuCount += expected.count;
            representedCountProbeGpuCount += actual.count;
            representedCountProbeCpuChecksum ^= expected.combinedChecksum;
            representedCountProbeGpuChecksum ^= actual.combinedChecksum;
            representedCountProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            representedCountProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuRepresentedCountProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuCoverageCompensationProbeResultPending[frameIndex] &&
            resources.gpuCoverageCompensationProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuCoverageCompensationProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuCoverageCompensationProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            coverageCompensationProbeChecked = true;
            coverageCompensationProbePassed = coverageCompensationProbePassed && layerPassed;
            coverageCompensationProbeCpuCount += expected.count;
            coverageCompensationProbeGpuCount += actual.count;
            coverageCompensationProbeCpuChecksum ^= expected.combinedChecksum;
            coverageCompensationProbeGpuChecksum ^= actual.combinedChecksum;
            coverageCompensationProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            coverageCompensationProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuCoverageCompensationProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuClampFlagsProbeResultPending[frameIndex] &&
            resources.gpuClampFlagsProbeStatsBuffers[frameIndex].mapped != nullptr) {
            GpuDrawItemCompactionStats actual{};
            std::memcpy(
                &actual,
                resources.gpuClampFlagsProbeStatsBuffers[frameIndex].mapped,
                sizeof(actual));
            const auto expected = resources.gpuClampFlagsProbeExpectedStats[frameIndex];
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.drawIndexXor == expected.drawIndexXor &&
                actual.combinedChecksum == expected.combinedChecksum &&
                actual.classCounts == expected.classCounts;

            clampFlagsProbeChecked = true;
            clampFlagsProbePassed = clampFlagsProbePassed && layerPassed;
            clampFlagsProbeCpuCount += expected.count;
            clampFlagsProbeGpuCount += actual.count;
            clampFlagsProbeCpuChecksum ^= expected.combinedChecksum;
            clampFlagsProbeGpuChecksum ^= actual.combinedChecksum;
            clampFlagsProbeCpuSourceFingerprint ^= foldSourceFingerprint(expected);
            clampFlagsProbeGpuSourceFingerprint ^= foldSourceFingerprint(actual);
            resources.gpuClampFlagsProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuCompactionOutputProbeResultPending[frameIndex] &&
            resources.gpuCompactedDrawItemBuffers[frameIndex].mapped != nullptr) {
            const auto expected = resources.gpuCompactionExpectedOutputProbeStats[frameIndex];
            const auto actual = ComputeGpuCompactionOutputProbeStatsFromBuffer(
                static_cast<const renderer::pointcloud::PointCloudDrawItemGpu*>(
                    resources.gpuCompactedDrawItemBuffers[frameIndex].mapped),
                expected.count);
            const bool layerPassed =
                actual.count == expected.count &&
                actual.sourceIndexXor == expected.sourceIndexXor &&
                actual.representedCountXor == expected.representedCountXor &&
                actual.footprintXor == expected.footprintXor &&
                actual.metadataXor == expected.metadataXor &&
                actual.sourceIndexSum == expected.sourceIndexSum &&
                actual.representedCountSum == expected.representedCountSum &&
                actual.identityChecksum == expected.identityChecksum &&
                actual.orderChecksum == expected.orderChecksum;

            outputProbeChecked = true;
            outputProbePassed = outputProbePassed && layerPassed;
            outputProbeCpuCount += expected.count;
            outputProbeGpuCount += actual.count;
            outputProbeCpuChecksum ^= expected.identityChecksum ^ expected.orderChecksum;
            outputProbeGpuChecksum ^= actual.identityChecksum ^ actual.orderChecksum;
            outputProbeCpuSourceFingerprint ^= foldOutputProbeFingerprint(expected);
            outputProbeGpuSourceFingerprint ^= foldOutputProbeFingerprint(actual);
            resources.gpuCompactionOutputProbeResultPending[frameIndex] = false;
        }

        if (resources.gpuCompactionIndirectCommandResultPending[frameIndex] &&
            resources.gpuCompactionIndirectCommandBuffers[frameIndex].mapped != nullptr) {
            VkDrawIndirectCommand actualCommand{};
            std::memcpy(
                &actualCommand,
                resources.gpuCompactionIndirectCommandBuffers[frameIndex].mapped,
                sizeof(actualCommand));
            const auto expectedCommand = resources.gpuCompactionExpectedIndirectCommands[frameIndex];
            const bool commandPassed =
                actualCommand.vertexCount == expectedCommand.vertexCount &&
                actualCommand.instanceCount == expectedCommand.instanceCount &&
                actualCommand.firstVertex == expectedCommand.firstVertex &&
                actualCommand.firstInstance == expectedCommand.firstInstance;
            indirectChecked = true;
            indirectPassed = indirectPassed && commandPassed;
            indirectCpuVertices += expectedCommand.vertexCount;
            indirectGpuVertices += actualCommand.vertexCount;
            resources.gpuCompactionIndirectCommandResultPending[frameIndex] = false;
        }
    }

    if (checked) {
        diagnostics_.adaptiveGpuCompactionCpuCount = cpuCount;
        diagnostics_.adaptiveGpuCompactionGpuCount = gpuCount;
        diagnostics_.adaptiveGpuCompactionCpuChecksum = cpuChecksum;
        diagnostics_.adaptiveGpuCompactionGpuChecksum = gpuChecksum;
        diagnostics_.adaptiveGpuCompactionCpuSourceFingerprint = cpuSourceFingerprint;
        diagnostics_.adaptiveGpuCompactionGpuSourceFingerprint = gpuSourceFingerprint;
        diagnostics_.adaptiveGpuCompactionCpuClassCounts = cpuClassCounts;
        diagnostics_.adaptiveGpuCompactionGpuClassCounts = gpuClassCounts;
        diagnostics_.adaptiveGpuCompactionParityStatus =
            passed ? "passed previous-frame full-range count/source-fingerprint/checksum/class-counts"
                   : "mismatch in previous-frame full-range count/source-fingerprint/checksum/class-counts";
    }

    if (featureProbeChecked) {
        diagnostics_.adaptiveGpuFeatureClassProbeCpuCount = featureProbeCpuCount;
        diagnostics_.adaptiveGpuFeatureClassProbeGpuCount = featureProbeGpuCount;
        diagnostics_.adaptiveGpuFeatureClassProbeCpuChecksum = featureProbeCpuChecksum;
        diagnostics_.adaptiveGpuFeatureClassProbeGpuChecksum = featureProbeGpuChecksum;
        diagnostics_.adaptiveGpuFeatureClassProbeCpuSourceFingerprint = featureProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuFeatureClassProbeGpuSourceFingerprint = featureProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuFeatureClassProbeParityStatus =
            featureProbePassed
                ? "passed previous-frame protected feature-class count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame protected feature-class count/source-fingerprint/checksum/class-counts";
    }

    if (rankProbeChecked) {
        diagnostics_.adaptiveGpuRankProbeCpuCount = rankProbeCpuCount;
        diagnostics_.adaptiveGpuRankProbeGpuCount = rankProbeGpuCount;
        diagnostics_.adaptiveGpuRankProbeCpuChecksum = rankProbeCpuChecksum;
        diagnostics_.adaptiveGpuRankProbeGpuChecksum = rankProbeGpuChecksum;
        diagnostics_.adaptiveGpuRankProbeCpuSourceFingerprint = rankProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuRankProbeGpuSourceFingerprint = rankProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuRankProbeParityStatus =
            rankProbePassed
                ? "passed previous-frame stable-rank prefix count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame stable-rank prefix count/source-fingerprint/checksum/class-counts";
    }

    if (depthProbeChecked) {
        diagnostics_.adaptiveGpuDepthProbeCpuCount = depthProbeCpuCount;
        diagnostics_.adaptiveGpuDepthProbeGpuCount = depthProbeGpuCount;
        diagnostics_.adaptiveGpuDepthProbeCpuChecksum = depthProbeCpuChecksum;
        diagnostics_.adaptiveGpuDepthProbeGpuChecksum = depthProbeGpuChecksum;
        diagnostics_.adaptiveGpuDepthProbeCpuSourceFingerprint = depthProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuDepthProbeGpuSourceFingerprint = depthProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuDepthProbeParityStatus =
            depthProbePassed
                ? "passed previous-frame hierarchy-depth window count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame hierarchy-depth window count/source-fingerprint/checksum/class-counts";
    }

    if (projectedAreaProbeChecked) {
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuCount = projectedAreaProbeCpuCount;
        diagnostics_.adaptiveGpuProjectedAreaProbeGpuCount = projectedAreaProbeGpuCount;
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuChecksum = projectedAreaProbeCpuChecksum;
        diagnostics_.adaptiveGpuProjectedAreaProbeGpuChecksum = projectedAreaProbeGpuChecksum;
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuSourceFingerprint =
            projectedAreaProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuProjectedAreaProbeGpuSourceFingerprint =
            projectedAreaProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuProjectedAreaProbeParityStatus =
            projectedAreaProbePassed
                ? "passed previous-frame projected-area window count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame projected-area window count/source-fingerprint/checksum/class-counts";
    }

    if (renderAreaProbeChecked) {
        diagnostics_.adaptiveGpuRenderAreaProbeCpuCount = renderAreaProbeCpuCount;
        diagnostics_.adaptiveGpuRenderAreaProbeGpuCount = renderAreaProbeGpuCount;
        diagnostics_.adaptiveGpuRenderAreaProbeCpuChecksum = renderAreaProbeCpuChecksum;
        diagnostics_.adaptiveGpuRenderAreaProbeGpuChecksum = renderAreaProbeGpuChecksum;
        diagnostics_.adaptiveGpuRenderAreaProbeCpuSourceFingerprint =
            renderAreaProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuRenderAreaProbeGpuSourceFingerprint =
            renderAreaProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuRenderAreaProbeParityStatus =
            renderAreaProbePassed
                ? "passed previous-frame render-area window count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame render-area window count/source-fingerprint/checksum/class-counts";
    }

    if (representedCountProbeChecked) {
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuCount = representedCountProbeCpuCount;
        diagnostics_.adaptiveGpuRepresentedCountProbeGpuCount = representedCountProbeGpuCount;
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuChecksum = representedCountProbeCpuChecksum;
        diagnostics_.adaptiveGpuRepresentedCountProbeGpuChecksum = representedCountProbeGpuChecksum;
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuSourceFingerprint =
            representedCountProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuRepresentedCountProbeGpuSourceFingerprint =
            representedCountProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuRepresentedCountProbeParityStatus =
            representedCountProbePassed
                ? "passed previous-frame represented-count window count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame represented-count window count/source-fingerprint/checksum/class-counts";
    }

    if (coverageCompensationProbeChecked) {
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuCount = coverageCompensationProbeCpuCount;
        diagnostics_.adaptiveGpuCoverageCompensationProbeGpuCount = coverageCompensationProbeGpuCount;
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuChecksum = coverageCompensationProbeCpuChecksum;
        diagnostics_.adaptiveGpuCoverageCompensationProbeGpuChecksum = coverageCompensationProbeGpuChecksum;
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuSourceFingerprint =
            coverageCompensationProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuCoverageCompensationProbeGpuSourceFingerprint =
            coverageCompensationProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuCoverageCompensationProbeParityStatus =
            coverageCompensationProbePassed
                ? "passed previous-frame coverage-compensation window count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame coverage-compensation window count/source-fingerprint/checksum/class-counts";
    }

    if (clampFlagsProbeChecked) {
        diagnostics_.adaptiveGpuClampFlagsProbeCpuCount = clampFlagsProbeCpuCount;
        diagnostics_.adaptiveGpuClampFlagsProbeGpuCount = clampFlagsProbeGpuCount;
        diagnostics_.adaptiveGpuClampFlagsProbeCpuChecksum = clampFlagsProbeCpuChecksum;
        diagnostics_.adaptiveGpuClampFlagsProbeGpuChecksum = clampFlagsProbeGpuChecksum;
        diagnostics_.adaptiveGpuClampFlagsProbeCpuSourceFingerprint =
            clampFlagsProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuClampFlagsProbeGpuSourceFingerprint =
            clampFlagsProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuClampFlagsProbeParityStatus =
            clampFlagsProbePassed
                ? "passed previous-frame clamp-flags count/source-fingerprint/checksum/class-counts"
                : "mismatch in previous-frame clamp-flags count/source-fingerprint/checksum/class-counts";
    }

    if (outputProbeChecked) {
        diagnostics_.adaptiveGpuCompactionOutputProbeCpuCount = outputProbeCpuCount;
        diagnostics_.adaptiveGpuCompactionOutputProbeGpuCount = outputProbeGpuCount;
        diagnostics_.adaptiveGpuCompactionOutputProbeCpuChecksum = outputProbeCpuChecksum;
        diagnostics_.adaptiveGpuCompactionOutputProbeGpuChecksum = outputProbeGpuChecksum;
        diagnostics_.adaptiveGpuCompactionOutputProbeCpuSourceFingerprint = outputProbeCpuSourceFingerprint;
        diagnostics_.adaptiveGpuCompactionOutputProbeGpuSourceFingerprint = outputProbeGpuSourceFingerprint;
        diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus =
            outputProbePassed ? "passed previous-frame ordered compacted output identity"
                              : "mismatch in previous-frame ordered compacted output identity";
    }

    if (indirectChecked) {
        diagnostics_.adaptiveGpuCompactionIndirectCommandCpuVertices = indirectCpuVertices;
        diagnostics_.adaptiveGpuCompactionIndirectCommandGpuVertices = indirectGpuVertices;
        diagnostics_.adaptiveGpuCompactionIndirectCommandParityStatus =
            indirectPassed ? "passed previous-frame compacted indirect vertex count"
                           : "mismatch in previous-frame compacted indirect vertex count";
    }
}

void VulkanViewportShell::ResetGpuTimestampQueries(VkCommandBuffer commandBuffer, FrameResources* frame) {
    if (!gpuTimestampsSupported_ || frame == nullptr || frame->timestampQueryPool == VK_NULL_HANDLE) {
        return;
    }
    vkCmdResetQueryPool(commandBuffer, frame->timestampQueryPool, 0, kGpuTimestampQueryCount);
    frame->timestampQueriesArmed = true;
    frame->timestampPassWritten.fill(false);
}

void VulkanViewportShell::WriteGpuTimestamp(
    VkCommandBuffer commandBuffer,
    FrameResources* frame,
    std::uint32_t passIndex,
    bool end) {
    if (!gpuTimestampsSupported_ ||
        frame == nullptr ||
        frame->timestampQueryPool == VK_NULL_HANDLE ||
        passIndex >= kGpuTimestampPassCount) {
        return;
    }
    const auto queryIndex = passIndex * kGpuTimestampQueriesPerPass + (end ? 1U : 0U);
    vkCmdWriteTimestamp(
        commandBuffer,
        end ? VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        frame->timestampQueryPool,
        queryIndex);
    if (end && passIndex < frame->timestampPassWritten.size()) {
        frame->timestampPassWritten[passIndex] = true;
    }
}

void VulkanViewportShell::CreateImGuiResources() {
    const std::array<VkDescriptorPoolSize, 11> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_SAMPLER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000),
    };

    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 1000 * static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    Check(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &imguiDescriptorPool_), "vkCreateDescriptorPool(imgui)");

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ApplyImGuiStyle();

    auto& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.ConfigViewportsNoDecoration = false;

    ImGui_ImplGlfw_InitForVulkan(window_, true);

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.Instance = instance_;
    initInfo.PhysicalDevice = physicalDevice_;
    initInfo.Device = device_;
    initInfo.QueueFamily = graphicsQueueFamily_;
    initInfo.Queue = graphicsQueue_;
    initInfo.DescriptorPool = imguiDescriptorPool_;
    initInfo.MinImageCount = std::max<std::uint32_t>(2U, static_cast<std::uint32_t>(swapchainImages_.size()));
    initInfo.ImageCount = static_cast<std::uint32_t>(swapchainImages_.size());
    initInfo.PipelineInfoMain.RenderPass = presentRenderPass_;
    initInfo.PipelineInfoMain.Subpass = 0;
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    initInfo.CheckVkResultFn = &CheckImGuiResult;

    if (!ImGui_ImplVulkan_Init(&initInfo)) {
        throw std::runtime_error{"ImGui Vulkan backend initialization failed."};
    }
}

void VulkanViewportShell::UploadImGuiFonts() {}

void VulkanViewportShell::UpdatePointCloudDescriptorSets(ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        resources->descriptorSets[frameIndex].resize(depthImages_.size(), VK_NULL_HANDLE);
        resources->gpuCompactedDescriptorSets[frameIndex].resize(depthImages_.size(), VK_NULL_HANDLE);
        for (std::uint32_t imageIndex = 0; imageIndex < depthImages_.size(); ++imageIndex) {
            UpdatePointCloudDescriptorSet(resources, frameIndex, imageIndex, depthImages_[imageIndex].view);
            UpdatePointCloudCompactedDescriptorSet(
                resources,
                frameIndex,
                imageIndex,
                depthImages_[imageIndex].view);
        }
    }
}

bool VulkanViewportShell::UpdatePointCloudDrawItemBuffer(
    ActivePointCloudResources* resources,
    std::size_t frameIndex,
    const std::vector<renderer::pointcloud::PointCloudDrawItemGpu>* drawItems,
    std::uint64_t revision) {
    if (resources == nullptr || frameIndex >= kFramesInFlight) {
        return false;
    }

    const auto drawItemCount = drawItems == nullptr ? 0U : static_cast<std::uint32_t>(
        std::min<std::size_t>(drawItems->size(), std::numeric_limits<std::uint32_t>::max()));
    const auto requiredCapacity = std::max<std::uint32_t>(1U, drawItemCount);
    const auto requiredGpuCompactedDrawItemCapacity =
        GpuDiagnosticCompactionOutputCapacity(drawItemCount);
    if (resources->drawItemSignatures[frameIndex] == revision &&
        resources->drawItemCounts[frameIndex] == drawItemCount &&
        resources->drawItemBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuCompactedDrawItemCapacities[frameIndex] >= requiredGpuCompactedDrawItemCapacity &&
        resources->gpuCompactionStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuFeatureClassProbeStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuRankProbeStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuDepthProbeStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuProjectedAreaProbeStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuRepresentedCountProbeStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuClampFlagsProbeStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
        resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer != VK_NULL_HANDLE) {
        resources->drawItemSignature = revision;
        resources->drawItemCount = drawItemCount;
        return false;
    }

    bool reallocated = false;
    if (resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->drawItemCapacities[frameIndex] < requiredCapacity) {
        DestroyBuffer(&resources->drawItemBuffers[frameIndex]);
        const auto newCapacity = std::max(requiredCapacity, resources->drawItemCapacities[frameIndex] * 2U);
        resources->drawItemBuffers[frameIndex] = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(newCapacity * sizeof(renderer::pointcloud::PointCloudDrawItemGpu)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        resources->drawItemCapacities[frameIndex] = newCapacity;
        reallocated = true;
    }
    if (resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemCapacities[frameIndex] < requiredGpuCompactedDrawItemCapacity) {
        DestroyBuffer(&resources->gpuCompactedDrawItemBuffers[frameIndex]);
        const auto newCapacity =
            std::max(
                requiredGpuCompactedDrawItemCapacity,
                resources->gpuCompactedDrawItemCapacities[frameIndex] * 2U);
        resources->gpuCompactedDrawItemBuffers[frameIndex] = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(newCapacity * sizeof(renderer::pointcloud::PointCloudDrawItemGpu)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        resources->gpuCompactedDrawItemCapacities[frameIndex] = newCapacity;
        reallocated = true;
    }
    if (resources->gpuCompactionStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuCompactionStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuCompactionStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuFeatureClassProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuFeatureClassProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuFeatureClassProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuRankProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuRankProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuRankProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuDepthProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuDepthProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuDepthProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuProjectedAreaProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuProjectedAreaProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuProjectedAreaProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuRenderAreaProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuRenderAreaProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuRenderAreaProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuRepresentedCountProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuRepresentedCountProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuRepresentedCountProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuClampFlagsProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const GpuDrawItemCompactionStats fallbackStats{};
        resources->gpuClampFlagsProbeStatsBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackStats),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuClampFlagsProbeStatsBuffers[frameIndex],
            &fallbackStats,
            sizeof(fallbackStats));
        reallocated = true;
    }
    if (resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        const VkDrawIndirectCommand fallbackIndirectDraw{0U, 1U, 0U, 0U};
        resources->gpuCompactionIndirectCommandBuffers[frameIndex] = CreateHostVisibleBuffer(
            sizeof(fallbackIndirectDraw),
            VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->gpuCompactionIndirectCommandBuffers[frameIndex],
            &fallbackIndirectDraw,
            sizeof(fallbackIndirectDraw));
        reallocated = true;
    }

    if (drawItemCount == 0) {
        const renderer::pointcloud::PointCloudDrawItemGpu fallback{};
        UploadBufferData(resources->drawItemBuffers[frameIndex], &fallback, sizeof(fallback));
    } else {
        UploadBufferData(
            resources->drawItemBuffers[frameIndex],
            drawItems->data(),
            static_cast<VkDeviceSize>(drawItemCount * sizeof(renderer::pointcloud::PointCloudDrawItemGpu)));
    }

    resources->drawItemSignatures[frameIndex] = revision;
    resources->drawItemCounts[frameIndex] = drawItemCount;
    resources->drawItemSignature = revision;
    resources->drawItemCount = drawItemCount;
    if (reallocated) {
        for (std::uint32_t imageIndex = 0; imageIndex < depthImages_.size(); ++imageIndex) {
            UpdatePointCloudDescriptorSet(
                resources,
                frameIndex,
                imageIndex,
                depthImages_[imageIndex].view);
            UpdatePointCloudCompactedDescriptorSet(
                resources,
                frameIndex,
                imageIndex,
                depthImages_[imageIndex].view);
        }
        UpdateGpuDrivenIndirectDescriptorSet(resources, frameIndex);
        UpdateGpuCompactionIndirectDescriptorSet(resources, frameIndex);
        UpdateGpuCompactionDescriptorSet(resources, frameIndex);
        UpdateGpuFeatureClassProbeDescriptorSet(resources, frameIndex);
        UpdateGpuRankProbeDescriptorSet(resources, frameIndex);
        UpdateGpuDepthProbeDescriptorSet(resources, frameIndex);
        UpdateGpuProjectedAreaProbeDescriptorSet(resources, frameIndex);
        UpdateGpuRenderAreaProbeDescriptorSet(resources, frameIndex);
        UpdateGpuRepresentedCountProbeDescriptorSet(resources, frameIndex);
        UpdateGpuCoverageCompensationProbeDescriptorSet(resources, frameIndex);
        UpdateGpuClampFlagsProbeDescriptorSet(resources, frameIndex);
    }
    return reallocated;
}

bool VulkanViewportShell::UpdatePointCloudExrDrawItemBuffer(
    ActivePointCloudResources* resources,
    const std::vector<renderer::pointcloud::PointCloudDrawItemGpu>* drawItems,
    std::uint64_t revision) {
    if (resources == nullptr) {
        return false;
    }

    const auto drawItemCount = drawItems == nullptr ? 0U : static_cast<std::uint32_t>(
        std::min<std::size_t>(drawItems->size(), std::numeric_limits<std::uint32_t>::max()));
    if (resources->exrDrawItemSignature == revision &&
        resources->exrDrawItemCount == drawItemCount &&
        resources->exrDrawItemBuffer.buffer != VK_NULL_HANDLE) {
        resources->drawItemSignature = revision;
        resources->drawItemCount = drawItemCount;
        return false;
    }

    bool reallocated = false;
    const auto requiredCapacity = std::max<std::uint32_t>(1U, drawItemCount);
    if (resources->exrDrawItemBuffer.buffer == VK_NULL_HANDLE ||
        resources->exrDrawItemCapacity < requiredCapacity) {
        DestroyBuffer(&resources->exrDrawItemBuffer);
        const auto newCapacity = std::max(requiredCapacity, resources->exrDrawItemCapacity * 2U);
        resources->exrDrawItemBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(newCapacity * sizeof(renderer::pointcloud::PointCloudDrawItemGpu)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        resources->exrDrawItemCapacity = newCapacity;
        reallocated = true;
    }

    if (drawItemCount == 0) {
        const renderer::pointcloud::PointCloudDrawItemGpu fallback{};
        UploadBufferData(resources->exrDrawItemBuffer, &fallback, sizeof(fallback));
    } else {
        UploadBufferData(
            resources->exrDrawItemBuffer,
            drawItems->data(),
            static_cast<VkDeviceSize>(drawItemCount * sizeof(renderer::pointcloud::PointCloudDrawItemGpu)));
    }

    resources->exrDrawItemSignature = revision;
    resources->exrDrawItemCount = drawItemCount;
    resources->drawItemSignature = revision;
    resources->drawItemCount = drawItemCount;
    if (reallocated && exrExportResources_.depthImage.view != VK_NULL_HANDLE) {
        UpdatePointCloudExrDescriptorSet(resources, exrExportResources_.depthImage.view);
    }
    return reallocated;
}

void VulkanViewportShell::UpdatePointCloudDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    VkImageView sceneDepthView) {
    if (resources == nullptr || frameIndex >= kFramesInFlight || imageIndex >= depthImages_.size()) {
        return;
    }
    if (imageIndex >= resources->descriptorSets[frameIndex].size()) {
        return;
    }
    UpdatePointCloudDescriptorSetForDrawItems(
        resources,
        frameIndex,
        imageIndex,
        sceneDepthView,
        resources->drawItemBuffers[frameIndex],
        &resources->descriptorSets[frameIndex][imageIndex],
        "vkAllocateDescriptorSets(point)");
}

void VulkanViewportShell::UpdatePointCloudCompactedDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    VkImageView sceneDepthView) {
    if (resources == nullptr || frameIndex >= kFramesInFlight || imageIndex >= depthImages_.size()) {
        return;
    }
    if (imageIndex >= resources->gpuCompactedDescriptorSets[frameIndex].size()) {
        return;
    }
    UpdatePointCloudDescriptorSetForDrawItems(
        resources,
        frameIndex,
        imageIndex,
        sceneDepthView,
        resources->gpuCompactedDrawItemBuffers[frameIndex],
        &resources->gpuCompactedDescriptorSets[frameIndex][imageIndex],
        "vkAllocateDescriptorSets(point compacted)");
}

void VulkanViewportShell::UpdatePointCloudDescriptorSetForDrawItems(
    ActivePointCloudResources* resources,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    VkImageView sceneDepthView,
    const BufferAllocation& drawItemBuffer,
    VkDescriptorSet* targetDescriptorSet,
    std::string_view allocationContext) {
    if (resources == nullptr || frameIndex >= kFramesInFlight || imageIndex >= depthImages_.size()) {
        return;
    }
    if (targetDescriptorSet == nullptr) {
        return;
    }

    auto& descriptorSet = *targetDescriptorSet;
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &pointDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            allocationContext);
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo scalarInfo{};
    scalarInfo.buffer = resources->scalarFieldBuffer.buffer;
    scalarInfo.offset = 0;
    scalarInfo.range = resources->scalarFieldBuffer.size;

    VkDescriptorBufferInfo styleInfo{};
    styleInfo.buffer = resources->styleBuffers[frameIndex].buffer;
    styleInfo.offset = 0;
    styleInfo.range = sizeof(PointCloudStyleGpu);

    VkDescriptorBufferInfo positionStorageInfo{};
    positionStorageInfo.buffer = resources->positionStorageBuffer.buffer;
    positionStorageInfo.offset = 0;
    positionStorageInfo.range = resources->positionStorageBuffer.size;

    VkDescriptorBufferInfo colorStorageInfo{};
    colorStorageInfo.buffer = resources->colorBuffer.buffer;
    colorStorageInfo.offset = 0;
    colorStorageInfo.range = resources->colorBuffer.size;

    VkDescriptorBufferInfo normalInfo{};
    normalInfo.buffer = resources->normalBuffer.buffer;
    normalInfo.offset = 0;
    normalInfo.range = resources->normalBuffer.size;

    VkDescriptorBufferInfo drawItemInfo{};
    drawItemInfo.buffer = drawItemBuffer.buffer;
    drawItemInfo.offset = 0;
    drawItemInfo.range = drawItemBuffer.size;

    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = sceneDepthView;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 8> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &scalarInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &styleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[3].descriptorCount = 1;
    writes[3].pImageInfo = &sceneDepthInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &positionStorageInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &colorStorageInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &normalInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = descriptorSet;
    writes[7].dstBinding = 7;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &drawItemInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdatePointCloudExrDescriptorSet(
    ActivePointCloudResources* resources,
    VkImageView sceneDepthView) {
    if (resources == nullptr) {
        return;
    }

    if (resources->exrDescriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &pointDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &resources->exrDescriptorSet),
            "vkAllocateDescriptorSets(point exr)");
    }

    VkDescriptorBufferInfo uniformInfo{frameResources_[0].uniformBuffer.buffer, 0, sizeof(FrameUniforms)};
    VkDescriptorBufferInfo scalarInfo{resources->scalarFieldBuffer.buffer, 0, resources->scalarFieldBuffer.size};
    VkDescriptorBufferInfo styleInfo{resources->exrStyleBuffer.buffer, 0, sizeof(PointCloudStyleGpu)};
    VkDescriptorBufferInfo positionStorageInfo{
        resources->positionStorageBuffer.buffer,
        0,
        resources->positionStorageBuffer.size};
    VkDescriptorBufferInfo colorStorageInfo{resources->colorBuffer.buffer, 0, resources->colorBuffer.size};
    VkDescriptorBufferInfo normalInfo{resources->normalBuffer.buffer, 0, resources->normalBuffer.size};
    VkDescriptorBufferInfo drawItemInfo{
        resources->exrDrawItemBuffer.buffer,
        0,
        resources->exrDrawItemBuffer.size};
    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = sceneDepthView;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 8> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = resources->exrDescriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = resources->exrDescriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &scalarInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = resources->exrDescriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &styleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = resources->exrDescriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[3].descriptorCount = 1;
    writes[3].pImageInfo = &sceneDepthInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = resources->exrDescriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &positionStorageInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = resources->exrDescriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &colorStorageInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = resources->exrDescriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &normalInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = resources->exrDescriptorSet;
    writes[7].dstBinding = 7;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &drawItemInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuDrivenIndirectDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuDrivenSelectionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->indirectDrawCommandBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactionStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuIndirectDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuDrivenSelectionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu indirect)");
    }

    VkDescriptorBufferInfo commandInfo{};
    commandInfo.buffer = resources->indirectDrawCommandBuffers[frameIndex].buffer;
    commandInfo.offset = 0;
    commandInfo.range = sizeof(VkDrawIndirectCommand);

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuCompactionStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    std::array<VkWriteDescriptorSet, 2> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &commandInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &statsInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuCompactionIndirectDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuDrivenSelectionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactionStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuCompactionIndirectDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuDrivenSelectionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu compaction indirect)");
    }

    VkDescriptorBufferInfo commandInfo{};
    commandInfo.buffer = resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer;
    commandInfo.offset = 0;
    commandInfo.range = sizeof(VkDrawIndirectCommand);

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuCompactionStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    std::array<VkWriteDescriptorSet, 2> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &commandInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &statsInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuCompactionDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactionStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuCompactionDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu compaction)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuCompactionStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuFeatureClassProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuFeatureClassProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuFeatureClassProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu feature class probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuFeatureClassProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuRankProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuRankProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuRankProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu rank probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuRankProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuDepthProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuDepthProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuDepthProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu depth probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuDepthProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuProjectedAreaProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuProjectedAreaProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuProjectedAreaProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu projected area probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuProjectedAreaProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuRenderAreaProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuRenderAreaProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuRenderAreaProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu render area probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuRenderAreaProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuRepresentedCountProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuRepresentedCountProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuRepresentedCountProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu represented count probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuRepresentedCountProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuCoverageCompensationProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuCoverageCompensationProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu coverage compensation probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGpuClampFlagsProbeDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t frameIndex) {
    if (resources == nullptr ||
        frameIndex >= kFramesInFlight ||
        gpuCompactionDescriptorSetLayout_ == VK_NULL_HANDLE ||
        resources->drawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuCompactedDrawItemBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        resources->gpuClampFlagsProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    auto& descriptorSet = resources->gpuClampFlagsProbeDescriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gpuCompactionDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point gpu clamp flags probe)");
    }

    VkDescriptorBufferInfo inputInfo{};
    inputInfo.buffer = resources->drawItemBuffers[frameIndex].buffer;
    inputInfo.offset = 0;
    inputInfo.range = resources->drawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo outputInfo{};
    outputInfo.buffer = resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
    outputInfo.offset = 0;
    outputInfo.range = resources->gpuCompactedDrawItemBuffers[frameIndex].size;

    VkDescriptorBufferInfo statsInfo{};
    statsInfo.buffer = resources->gpuClampFlagsProbeStatsBuffers[frameIndex].buffer;
    statsInfo.offset = 0;
    statsInfo.range = sizeof(GpuDrawItemCompactionStats);

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo positionInfo{};
    positionInfo.buffer = resources->positionStorageBuffer.buffer;
    positionInfo.offset = 0;
    positionInfo.range = resources->positionStorageBuffer.size;

    std::array<VkWriteDescriptorSet, 5> writes{};
    for (auto& write : writes) {
        write = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.descriptorCount = 1;
    }
    writes[0].dstBinding = 0;
    writes[0].pBufferInfo = &inputInfo;
    writes[1].dstBinding = 1;
    writes[1].pBufferInfo = &outputInfo;
    writes[2].dstBinding = 2;
    writes[2].pBufferInfo = &statsInfo;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[3].pBufferInfo = &uniformInfo;
    writes[4].dstBinding = 4;
    writes[4].pBufferInfo = &positionInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

VulkanViewportShell::GpuDrawItemCompactionStats VulkanViewportShell::ComputeGpuCompactionStats(
    const std::vector<renderer::pointcloud::PointCloudDrawItemGpu>& drawItems,
    const std::vector<invisible_places::io::Float3>& positions,
    std::uint32_t drawItemCount,
    std::uint32_t selectionClassMask,
    std::uint32_t selectionProfileMask,
    std::uint32_t selectionRankLimit,
    std::uint32_t selectionMinDepth,
    std::uint32_t selectionMaxDepth,
    std::uint32_t selectionRequiredFlags,
    std::uint32_t selectionRejectedFlags,
    float selectionMinFootprintAreaPixels,
    float selectionMaxFootprintAreaPixels,
    float selectionMinRenderAreaPixels,
    float selectionMaxRenderAreaPixels,
    float selectionMinOpacityCompensation,
    float selectionMaxOpacityCompensation,
    float selectionMinEmissionCompensation,
    float selectionMaxEmissionCompensation,
    std::uint32_t selectionMinRepresentedSourceCount,
    std::uint32_t selectionMaxRepresentedSourceCount,
    const glm::mat4& selectionViewProjection,
    float selectionFrustumGuardBand,
    GpuDrawItemOutputProbeStats* outputProbeStats,
    std::uint32_t outputProbeCapacity) const {
    const auto mixWord = [](std::uint32_t value) {
        value ^= value >> 16U;
        value *= 0x7feb352dU;
        value ^= value >> 15U;
        value *= 0x846ca68bU;
        value ^= value >> 16U;
        return value;
    };

    GpuDrawItemCompactionStats stats{};
    const auto candidateCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(
            std::min<std::size_t>(drawItems.size(), drawItemCount),
            std::numeric_limits<std::uint32_t>::max()));
    for (std::uint32_t drawIndex = 0; drawIndex < candidateCount; ++drawIndex) {
        const auto& item = drawItems[drawIndex];
        if (selectionClassMask != 0U &&
            (DrawItemRepresentativeClassFlags(item) & selectionClassMask) == 0U) {
            continue;
        }
        const auto classFlags = DrawItemRepresentativeClassFlags(item);
        if (selectionProfileMask != 0U &&
            (DrawItemRendererCostProfileBit(item) & selectionProfileMask) == 0U) {
            continue;
        }
        if (DrawItemRepresentativePackedRank(item) > selectionRankLimit) {
            continue;
        }
        const auto packedDepth = DrawItemRepresentativePackedDepth(item);
        if (packedDepth < selectionMinDepth || packedDepth > selectionMaxDepth) {
            continue;
        }
        const auto packedFlags = DrawItemRepresentativePackedFlags(item);
        if ((packedFlags & selectionRequiredFlags) != selectionRequiredFlags) {
            continue;
        }
        if ((packedFlags & selectionRejectedFlags) != 0U) {
            continue;
        }
        if (!DrawItemWithinProjectedAreaWindow(
                item,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels)) {
            continue;
        }
        if (!DrawItemWithinCompensationWindow(
                item,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation)) {
            continue;
        }
        if (!DrawItemWithinRepresentedSourceWindow(
                item,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount)) {
            continue;
        }
        if (selectionFrustumGuardBand > 0.0F &&
            !DrawItemWithinFrustumGuard(
                item,
                positions,
                selectionViewProjection,
                selectionFrustumGuardBand)) {
            continue;
        }
        if (outputProbeStats != nullptr && outputProbeStats->count < outputProbeCapacity) {
            AccumulateGpuCompactionOutputProbeStats(item, outputProbeStats->count, outputProbeStats);
        }
        std::uint32_t footprintBits = 0;
        std::memcpy(&footprintBits, &item.footprintAreaPixels, sizeof(footprintBits));
        const auto mixed =
            mixWord(item.sourcePointIndex ^ (drawIndex * 0x9e3779b9U)) ^
            mixWord(item.representedSourceCount + 0x85ebca6bU) ^
            mixWord(footprintBits + 0xc2b2ae35U);
        ++stats.count;
        stats.sourceIndexXor ^= item.sourcePointIndex;
        stats.representedCountXor ^= item.representedSourceCount;
        stats.footprintXor ^= footprintBits;
        stats.sourceIndexSum += item.sourcePointIndex;
        stats.representedCountSum += item.representedSourceCount;
        stats.drawIndexXor ^= drawIndex;
        stats.combinedChecksum ^= mixed;
        for (std::size_t classIndex = 0; classIndex < stats.classCounts.size(); ++classIndex) {
            const auto classBit = 1U << static_cast<std::uint32_t>(classIndex);
            if ((classFlags & classBit) != 0U) {
                ++stats.classCounts[classIndex];
            }
        }
    }
    return stats;
}

void VulkanViewportShell::AccumulateGpuCompactionOutputProbeStats(
    const renderer::pointcloud::PointCloudDrawItemGpu& item,
    std::uint32_t outputIndex,
    GpuDrawItemOutputProbeStats* stats) {
    if (stats == nullptr) {
        return;
    }
    const auto mixWord = [](std::uint32_t value) {
        value ^= value >> 16U;
        value *= 0x7feb352dU;
        value ^= value >> 15U;
        value *= 0x846ca68bU;
        value ^= value >> 16U;
        return value;
    };

    std::uint32_t footprintBits = 0;
    std::memcpy(&footprintBits, &item.footprintAreaPixels, sizeof(footprintBits));
    ++stats->count;
    stats->sourceIndexXor ^= item.sourcePointIndex;
    stats->representedCountXor ^= item.representedSourceCount;
    stats->footprintXor ^= footprintBits;
    stats->metadataXor ^= item.reserved1;
    stats->sourceIndexSum += item.sourcePointIndex;
    stats->representedCountSum += item.representedSourceCount;
    stats->identityChecksum ^=
        mixWord(item.sourcePointIndex + 0x9e3779b9U) ^
        mixWord(item.representedSourceCount + 0x85ebca6bU) ^
        mixWord(footprintBits + 0xc2b2ae35U) ^
        mixWord(item.reserved1 + 0x27d4eb2dU);
    stats->orderChecksum ^=
        mixWord((outputIndex * 0x9e3779b9U) ^ item.sourcePointIndex) ^
        mixWord(outputIndex + (item.representedSourceCount * 0x85ebca6bU)) ^
        mixWord((outputIndex ^ 0x27d4eb2dU) + footprintBits) ^
        mixWord((outputIndex * 0xc2b2ae35U) ^ item.reserved1);
}

VulkanViewportShell::GpuDrawItemOutputProbeStats
VulkanViewportShell::ComputeGpuCompactionOutputProbeStatsFromBuffer(
    const renderer::pointcloud::PointCloudDrawItemGpu* drawItems,
    std::uint32_t drawItemCount) const {
    GpuDrawItemOutputProbeStats stats{};
    if (drawItems == nullptr) {
        return stats;
    }
    for (std::uint32_t index = 0; index < drawItemCount; ++index) {
        AccumulateGpuCompactionOutputProbeStats(drawItems[index], index, &stats);
    }
    return stats;
}

void VulkanViewportShell::CreateOrUpdateCompositeDescriptorSet() {
    compositeDescriptorSets_.resize(accumulationImages_.size(), VK_NULL_HANDLE);
    for (std::uint32_t imageIndex = 0; imageIndex < accumulationImages_.size(); ++imageIndex) {
        CreateOrUpdateCompositeDescriptorSet(
            &compositeDescriptorSets_[imageIndex],
            accumulationImages_[imageIndex].view,
            revealageImages_[imageIndex].view,
            emissiveImages_[imageIndex].view);
    }
}

void VulkanViewportShell::CreateOrUpdateCompositeDescriptorSet(
    VkDescriptorSet* descriptorSet,
    VkImageView accumulationView,
    VkImageView revealageView,
    VkImageView emissiveView) {
    CreateOrUpdateCompositeDescriptorSet(
        descriptorSet,
        accumulationView,
        revealageView,
        emissiveView,
        VK_NULL_HANDLE,
        VK_NULL_HANDLE);
}

void VulkanViewportShell::CreateOrUpdateCompositeDescriptorSet(
    VkDescriptorSet* descriptorSet,
    VkImageView accumulationView,
    VkImageView revealageView,
    VkImageView emissiveView,
    VkImageView normalAccumulationView,
    VkImageView albedoAccumulationView) {
    if (descriptorSet == nullptr) {
        return;
    }

    if (*descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &compositeDescriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &allocInfo, descriptorSet), "vkAllocateDescriptorSets(composite)");
    }

    VkDescriptorImageInfo accumulationInfo{};
    accumulationInfo.imageView = accumulationView;
    accumulationInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo revealageInfo{};
    revealageInfo.imageView = revealageView;
    revealageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo emissiveInfo{};
    emissiveInfo.imageView = emissiveView;
    emissiveInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo normalAccumulationInfo{};
    normalAccumulationInfo.imageView = normalAccumulationView;
    normalAccumulationInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo albedoAccumulationInfo{};
    albedoAccumulationInfo.imageView = albedoAccumulationView;
    albedoAccumulationInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 5> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = *descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[0].descriptorCount = 1;
    writes[0].pImageInfo = &accumulationInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = *descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[1].descriptorCount = 1;
    writes[1].pImageInfo = &revealageInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = *descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo = &emissiveInfo;

    std::uint32_t writeCount = 3U;
    if (normalAccumulationView != VK_NULL_HANDLE && albedoAccumulationView != VK_NULL_HANDLE) {
        writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[3].dstSet = *descriptorSet;
        writes[3].dstBinding = 3;
        writes[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
        writes[3].descriptorCount = 1;
        writes[3].pImageInfo = &normalAccumulationInfo;

        writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[4].dstSet = *descriptorSet;
        writes[4].dstBinding = 4;
        writes[4].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
        writes[4].descriptorCount = 1;
        writes[4].pImageInfo = &albedoAccumulationInfo;
        writeCount = 5U;
    }

    vkUpdateDescriptorSets(device_, writeCount, writes.data(), 0, nullptr);
}

void VulkanViewportShell::CreateOrUpdatePostProcessDescriptorSets() {
    postProcessDescriptorSets_.resize(sceneColorImages_.size(), VK_NULL_HANDLE);
    for (std::uint32_t imageIndex = 0; imageIndex < sceneColorImages_.size(); ++imageIndex) {
        auto& descriptorSet = postProcessDescriptorSets_[imageIndex];
        if (descriptorSet == VK_NULL_HANDLE) {
            VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            allocInfo.descriptorPool = descriptorPool_;
            allocInfo.descriptorSetCount = 1;
            allocInfo.pSetLayouts = &postProcessDescriptorSetLayout_;
            Check(
                vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
                "vkAllocateDescriptorSets(postprocess)");
        }

        VkDescriptorImageInfo sceneColorInfo{};
        sceneColorInfo.sampler = postProcessSampler_;
        sceneColorInfo.imageView = sceneColorImages_[imageIndex].view;
        sceneColorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo linearDepthInfo{};
        linearDepthInfo.sampler = postProcessSampler_;
        linearDepthInfo.imageView = linearDepthImages_[imageIndex].view;
        linearDepthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        std::array<VkWriteDescriptorSet, 2> writes{};
        writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[0].dstSet = descriptorSet;
        writes[0].dstBinding = 0;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[0].descriptorCount = 1;
        writes[0].pImageInfo = &sceneColorInfo;

        writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[1].dstSet = descriptorSet;
        writes[1].dstBinding = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].descriptorCount = 1;
        writes[1].pImageInfo = &linearDepthInfo;

        vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

void VulkanViewportShell::UpdateGaussianSplatDescriptorSets(ActiveGaussianSplatResources* resources) {
    if (resources == nullptr) {
        return;
    }
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        resources->descriptorSets[frameIndex].resize(depthImages_.size(), VK_NULL_HANDLE);
        for (std::uint32_t imageIndex = 0; imageIndex < depthImages_.size(); ++imageIndex) {
            UpdateGaussianSplatDescriptorSet(resources, frameIndex, imageIndex);
        }
    }
}

void VulkanViewportShell::UpdateGaussianSplatDescriptorSet(
    ActiveGaussianSplatResources* resources,
    std::size_t frameIndex,
    std::uint32_t imageIndex) {
    if (resources == nullptr || frameIndex >= kFramesInFlight || imageIndex >= depthImages_.size()) {
        return;
    }

    auto& descriptorSet = resources->descriptorSets[frameIndex][imageIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = gaussianSplatDescriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gaussianSplatDescriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet), "vkAllocateDescriptorSets(gsplat)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo centerInfo{resources->centerBuffer.buffer, 0, resources->centerBuffer.size};
    VkDescriptorBufferInfo scaleInfo{resources->scaleBuffer.buffer, 0, resources->scaleBuffer.size};
    VkDescriptorBufferInfo rotationInfo{resources->rotationBuffer.buffer, 0, resources->rotationBuffer.size};
    VkDescriptorBufferInfo opacityInfo{resources->opacityBuffer.buffer, 0, resources->opacityBuffer.size};
    VkDescriptorBufferInfo shInfo{resources->shBuffer.buffer, 0, resources->shBuffer.size};
    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = depthImages_[imageIndex].view;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 7> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &centerInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &scaleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = 1;
    writes[3].pBufferInfo = &rotationInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &opacityInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &shInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[6].descriptorCount = 1;
    writes[6].pImageInfo = &sceneDepthInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateHighQualityGaussianDescriptorSet(std::size_t frameIndex) {
    auto& scene = highQualityGaussianScene_;
    if (scene.splatCount == 0 || scene.layerCount == 0 || frameIndex >= kFramesInFlight) {
        return;
    }

    auto& descriptorSet = scene.descriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = gaussianSplatDescriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &highQualityGaussianSplatDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(gsplat_hq)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo centerInfo{scene.centerBuffer.buffer, 0, scene.centerBuffer.size};
    VkDescriptorBufferInfo scaleInfo{scene.scaleBuffer.buffer, 0, scene.scaleBuffer.size};
    VkDescriptorBufferInfo rotationInfo{scene.rotationBuffer.buffer, 0, scene.rotationBuffer.size};
    VkDescriptorBufferInfo opacityInfo{scene.opacityBuffer.buffer, 0, scene.opacityBuffer.size};
    VkDescriptorBufferInfo shInfo{scene.shBuffer.buffer, 0, scene.shBuffer.size};
    VkDescriptorBufferInfo layerStyleIndexInfo{
        scene.layerStyleIndexBuffer.buffer,
        0,
        scene.layerStyleIndexBuffer.size};
    VkDescriptorBufferInfo layerStyleInfo{
        scene.layerStyleBuffers[frameIndex].buffer,
        0,
        scene.layerStyleBuffers[frameIndex].size};
    VkDescriptorBufferInfo sortedIndexInfo{
        scene.sortedIndexBuffers[frameIndex].buffer,
        0,
        scene.sortedIndexBuffers[frameIndex].size};

    std::array<VkWriteDescriptorSet, 9> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &centerInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &scaleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = 1;
    writes[3].pBufferInfo = &rotationInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &opacityInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &shInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &layerStyleIndexInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = descriptorSet;
    writes[7].dstBinding = 7;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &layerStyleInfo;

    writes[8] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[8].dstSet = descriptorSet;
    writes[8].dstBinding = 8;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[8].descriptorCount = 1;
    writes[8].pBufferInfo = &sortedIndexInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::RefreshHighQualityGaussianScene(std::size_t frameIndex) {
    struct ActiveHighLayer {
        const SceneRenderState::GaussianSplatLayerState* renderLayer = nullptr;
        const ActiveGaussianSplatResources* resources = nullptr;
    };

    std::vector<ActiveHighLayer> activeHighLayers;
    activeHighLayers.reserve(renderState_.gaussianSplatLayers.size());

    for (const auto& layer : renderState_.gaussianSplatLayers) {
        if (layer.style.qualityMode != renderer::gsplat::GaussianSplatQualityMode::High) {
            continue;
        }

        const auto* resources = FindGaussianSplatResources(layer.layerId);
        if (resources == nullptr || resources->splatCount == 0) {
            continue;
        }

        ActiveHighLayer activeLayer;
        activeLayer.renderLayer = &layer;
        activeLayer.resources = resources;
        activeHighLayers.push_back(activeLayer);
    }

    if (activeHighLayers.empty()) {
        const bool hasDescriptor = std::any_of(
            highQualityGaussianScene_.descriptorSets.begin(),
            highQualityGaussianScene_.descriptorSets.end(),
            [](VkDescriptorSet descriptorSet) { return descriptorSet != VK_NULL_HANDLE; });
        if (highQualityGaussianScene_.splatCount > 0 || hasDescriptor) {
            CleanupHighQualityGaussianScene();
        }
        highQualityGaussianSceneDirty_ = false;
        return;
    }

    std::vector<renderer::gsplat::HighQualityGaussianLayerInput> hqLayerInputs;
    hqLayerInputs.reserve(activeHighLayers.size());
    for (const auto& activeLayer : activeHighLayers) {
        renderer::gsplat::HighQualityGaussianLayerInput input;
        input.layerId = activeLayer.renderLayer->layerId;
        input.revision = activeLayer.resources->revision;
        input.splatCount = activeLayer.resources->splatCount;
        input.localToWorld = activeLayer.renderLayer->localToWorld;
        input.transformEnabled = activeLayer.renderLayer->style.transformEnabled;
        hqLayerInputs.push_back(input);
    }

    const auto signatures = renderer::gsplat::BuildHighQualityGaussianLayerSignatures(hqLayerInputs);

    const bool needsRebuild =
        highQualityGaussianSceneDirty_ ||
        highQualityGaussianScene_.splatCount == 0 ||
        !renderer::gsplat::HighQualityGaussianLayerSignaturesMatch(
            highQualityGaussianScene_.layerSignatures,
            signatures);

    if (needsRebuild) {
        CleanupHighQualityGaussianScene();

        std::vector<glm::vec4> mergedCenters;
        std::vector<glm::vec4> mergedScales;
        std::vector<glm::vec4> mergedRotations;
        std::vector<float> mergedOpacities;
        std::vector<float> mergedShCoefficients;
        std::vector<std::uint32_t> mergedLayerStyleIndices;

        highQualityGaussianScene_.layerRanges =
            renderer::gsplat::BuildHighQualityGaussianLayerRanges(hqLayerInputs);

        for (const auto& activeLayer : activeHighLayers) {
            const auto& resources = *activeLayer.resources;
            const auto splatCount = resources.splatCount;

            mergedCenters.reserve(mergedCenters.size() + splatCount);
            for (const auto& center : resources.cpuCenters) {
                mergedCenters.emplace_back(center.x, center.y, center.z, 1.0F);
                highQualityGaussianScene_.mergedLocalCenters.push_back(center);
            }

            mergedScales.reserve(mergedScales.size() + splatCount);
            for (const auto& scale : resources.cpuScales) {
                mergedScales.emplace_back(scale[0], scale[1], scale[2], 0.0F);
            }

            mergedRotations.reserve(mergedRotations.size() + splatCount);
            for (const auto& rotation : resources.cpuRotations) {
                mergedRotations.emplace_back(rotation[0], rotation[1], rotation[2], rotation[3]);
            }

            mergedOpacities.insert(
                mergedOpacities.end(),
                resources.cpuOpacities.begin(),
                resources.cpuOpacities.end());
            mergedShCoefficients.insert(
                mergedShCoefficients.end(),
                resources.cpuShCoefficients.begin(),
                resources.cpuShCoefficients.end());
        }

        highQualityGaussianScene_.layerSignatures = signatures;
        highQualityGaussianScene_.splatCount =
            highQualityGaussianScene_.layerRanges.empty()
                ? 0U
                : highQualityGaussianScene_.layerRanges.back().mergedStart +
                      highQualityGaussianScene_.layerRanges.back().splatCount;
        highQualityGaussianScene_.layerCount = static_cast<std::uint32_t>(highQualityGaussianScene_.layerRanges.size());

        mergedLayerStyleIndices.clear();
        for (const auto& layerRange : highQualityGaussianScene_.layerRanges) {
            mergedLayerStyleIndices.insert(
                mergedLayerStyleIndices.end(),
                layerRange.splatCount,
                layerRange.styleIndex);
        }

        highQualityGaussianScene_.centerBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedCenters.size() * sizeof(glm::vec4)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.centerBuffer,
            mergedCenters.data(),
            highQualityGaussianScene_.centerBuffer.size);

        highQualityGaussianScene_.scaleBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedScales.size() * sizeof(glm::vec4)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.scaleBuffer,
            mergedScales.data(),
            highQualityGaussianScene_.scaleBuffer.size);

        highQualityGaussianScene_.rotationBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedRotations.size() * sizeof(glm::vec4)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.rotationBuffer,
            mergedRotations.data(),
            highQualityGaussianScene_.rotationBuffer.size);

        highQualityGaussianScene_.opacityBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedOpacities.size() * sizeof(float)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.opacityBuffer,
            mergedOpacities.data(),
            highQualityGaussianScene_.opacityBuffer.size);

        highQualityGaussianScene_.shBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedShCoefficients.size() * sizeof(float)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.shBuffer,
            mergedShCoefficients.data(),
            highQualityGaussianScene_.shBuffer.size);

        highQualityGaussianScene_.layerStyleIndexBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedLayerStyleIndices.size() * sizeof(std::uint32_t)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.layerStyleIndexBuffer,
            mergedLayerStyleIndices.data(),
            highQualityGaussianScene_.layerStyleIndexBuffer.size);

        for (std::size_t resourceFrameIndex = 0; resourceFrameIndex < kFramesInFlight; ++resourceFrameIndex) {
            highQualityGaussianScene_.layerStyleBuffers[resourceFrameIndex] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(
                    std::max<std::uint32_t>(1U, highQualityGaussianScene_.layerCount) *
                    sizeof(HighQualityGaussianLayerStyle)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            highQualityGaussianScene_.sortedIndexBuffers[resourceFrameIndex] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(
                    std::max<std::uint32_t>(1U, highQualityGaussianScene_.splatCount) * sizeof(std::uint32_t)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UpdateHighQualityGaussianDescriptorSet(resourceFrameIndex);
        }
        highQualityGaussianSceneDirty_ = false;
    }

    if (highQualityGaussianScene_.splatCount == 0 || highQualityGaussianScene_.layerCount == 0) {
        return;
    }

    std::vector<HighQualityGaussianLayerStyle> layerStyles(
        highQualityGaussianScene_.layerCount);
    auto findActiveLayer = [&](std::size_t layerId) -> const ActiveHighLayer* {
        const auto it = std::find_if(
            activeHighLayers.begin(),
            activeHighLayers.end(),
            [layerId](const ActiveHighLayer& activeLayer) {
                return activeLayer.renderLayer->layerId == layerId;
            });
        return it != activeHighLayers.end() ? &(*it) : nullptr;
    };

    for (const auto& layerRange : highQualityGaussianScene_.layerRanges) {
        const auto* activeLayer = findActiveLayer(layerRange.layerId);
        if (activeLayer == nullptr) {
            continue;
        }

        const auto& renderLayer = *activeLayer->renderLayer;
        auto& layerStyle = layerStyles[layerRange.styleIndex];
        layerStyle.localToWorld = renderLayer.localToWorld;
        layerStyle.layerTint = glm::vec4{
            renderLayer.style.layerTint[0],
            renderLayer.style.layerTint[1],
            renderLayer.style.layerTint[2],
            renderLayer.style.layerTint[3]};
        layerStyle.style = glm::vec4{
            renderLayer.style.opacityMultiplier,
            renderLayer.style.scaleMultiplier,
            renderLayer.style.exposure,
            renderLayer.style.saturation};
        layerStyle.control = glm::uvec4{
            static_cast<std::uint32_t>(renderLayer.style.colorMode),
            static_cast<std::uint32_t>(renderLayer.style.debugMode),
            renderLayer.style.transformEnabled ? 1U : 0U,
            0U};

    }

    UploadBufferData(
        highQualityGaussianScene_.layerStyleBuffers[frameIndex],
        layerStyles.data(),
        static_cast<VkDeviceSize>(layerStyles.size() * sizeof(HighQualityGaussianLayerStyle)));

    const bool needsResort =
        !highQualityGaussianScene_.hasSortedView ||
        !MatricesApproximatelyEqual(highQualityGaussianScene_.lastSortedView, renderState_.view);
    if (needsResort) {
        highQualityGaussianScene_.sortedIndices = renderer::gsplat::SortHighQualityGaussianIndices(
            highQualityGaussianScene_.mergedLocalCenters,
            hqLayerInputs,
            highQualityGaussianScene_.layerRanges,
            renderState_.view);
        highQualityGaussianScene_.lastSortedView = renderState_.view;
        highQualityGaussianScene_.hasSortedView = true;
    }
    UploadBufferData(
        highQualityGaussianScene_.sortedIndexBuffers[frameIndex],
        highQualityGaussianScene_.sortedIndices.data(),
        static_cast<VkDeviceSize>(highQualityGaussianScene_.sortedIndices.size() * sizeof(std::uint32_t)));
}

void VulkanViewportShell::CleanupSwapchain() {
    if (commandPool_ != VK_NULL_HANDLE) {
        for (auto& frame : frameResources_) {
            if (frame.commandBuffer != VK_NULL_HANDLE) {
                vkFreeCommandBuffers(device_, commandPool_, 1, &frame.commandBuffer);
                frame.commandBuffer = VK_NULL_HANDLE;
            }
        }
    }

    for (auto& descriptorSet : compositeDescriptorSets_) {
        if (descriptorSet != VK_NULL_HANDLE && descriptorPool_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
        }
    }
    compositeDescriptorSets_.clear();
    for (auto& descriptorSet : postProcessDescriptorSets_) {
        if (descriptorSet != VK_NULL_HANDLE && descriptorPool_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
        }
    }
    postProcessDescriptorSets_.clear();

    for (auto& resources : pointCloudResources_) {
        for (auto& descriptorSets : resources.descriptorSets) {
            for (auto& descriptorSet : descriptorSets) {
                if (descriptorSet != VK_NULL_HANDLE && descriptorPool_ != VK_NULL_HANDLE) {
                    vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
                    descriptorSet = VK_NULL_HANDLE;
                }
            }
            descriptorSets.clear();
        }
        for (auto& descriptorSets : resources.gpuCompactedDescriptorSets) {
            for (auto& descriptorSet : descriptorSets) {
                if (descriptorSet != VK_NULL_HANDLE && descriptorPool_ != VK_NULL_HANDLE) {
                    vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
                    descriptorSet = VK_NULL_HANDLE;
                }
            }
            descriptorSets.clear();
        }
    }
    for (auto& resources : gaussianSplatResources_) {
        for (auto& descriptorSets : resources.descriptorSets) {
            for (auto& descriptorSet : descriptorSets) {
                if (descriptorSet != VK_NULL_HANDLE && gaussianSplatDescriptorPool_ != VK_NULL_HANDLE) {
                    vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &descriptorSet);
                    descriptorSet = VK_NULL_HANDLE;
                }
            }
            descriptorSets.clear();
        }
    }

    for (const auto framebuffer : framebuffers_) {
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
    }
    framebuffers_.clear();
    for (const auto framebuffer : presentFramebuffers_) {
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
    }
    presentFramebuffers_.clear();

    for (auto& image : sceneColorImages_) {
        DestroyImage(&image);
    }
    for (auto& image : depthImages_) {
        DestroyImage(&image);
    }
    for (auto& image : accumulationImages_) {
        DestroyImage(&image);
    }
    for (auto& image : revealageImages_) {
        DestroyImage(&image);
    }
    for (auto& image : emissiveImages_) {
        DestroyImage(&image);
    }
    for (auto& image : linearDepthImages_) {
        DestroyImage(&image);
    }
    sceneColorImages_.clear();
    depthImages_.clear();
    accumulationImages_.clear();
    revealageImages_.clear();
    emissiveImages_.clear();
    linearDepthImages_.clear();

    for (const auto imageView : imageViews_) {
        vkDestroyImageView(device_, imageView, nullptr);
    }
    imageViews_.clear();

    if (swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device_, swapchain_, nullptr);
        swapchain_ = VK_NULL_HANDLE;
    }

    swapchainImages_.clear();
    swapchainImagesInFlight_.clear();
}

void VulkanViewportShell::CleanupPointCloudResources(ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }

    DestroyBuffer(&resources->positionBuffer);
    DestroyBuffer(&resources->positionStorageBuffer);
    DestroyBuffer(&resources->colorBuffer);
    DestroyBuffer(&resources->normalBuffer);
    DestroyBuffer(&resources->scalarFieldBuffer);
    for (auto& styleBuffer : resources->styleBuffers) {
        DestroyBuffer(&styleBuffer);
    }
    DestroyBuffer(&resources->exrStyleBuffer);
    DestroyBuffer(&resources->sampledIndexBuffer);
    DestroyBuffer(&resources->sampledSurfelIndexBuffer);
    for (auto& drawItemBuffer : resources->drawItemBuffers) {
        DestroyBuffer(&drawItemBuffer);
    }
    for (auto& compactedDrawItemBuffer : resources->gpuCompactedDrawItemBuffers) {
        DestroyBuffer(&compactedDrawItemBuffer);
    }
    for (auto& statsBuffer : resources->gpuCompactionStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuFeatureClassProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuRankProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuDepthProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuProjectedAreaProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuRenderAreaProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuRepresentedCountProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuCoverageCompensationProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& statsBuffer : resources->gpuClampFlagsProbeStatsBuffers) {
        DestroyBuffer(&statsBuffer);
    }
    for (auto& indirectDrawCommandBuffer : resources->indirectDrawCommandBuffers) {
        DestroyBuffer(&indirectDrawCommandBuffer);
    }
    for (auto& indirectDrawCommandBuffer : resources->gpuCompactionIndirectCommandBuffers) {
        DestroyBuffer(&indirectDrawCommandBuffer);
    }
    DestroyBuffer(&resources->exrDrawItemBuffer);
    for (auto& descriptorSets : resources->descriptorSets) {
        for (auto& descriptorSet : descriptorSets) {
            if (descriptorSet != VK_NULL_HANDLE &&
                descriptorPool_ != VK_NULL_HANDLE &&
                device_ != VK_NULL_HANDLE) {
                vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            }
        }
        descriptorSets.clear();
    }
    for (auto& descriptorSets : resources->gpuCompactedDescriptorSets) {
        for (auto& descriptorSet : descriptorSets) {
            if (descriptorSet != VK_NULL_HANDLE &&
                descriptorPool_ != VK_NULL_HANDLE &&
                device_ != VK_NULL_HANDLE) {
                vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            }
        }
        descriptorSets.clear();
    }
    for (auto& descriptorSet : resources->gpuIndirectDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuCompactionIndirectDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuCompactionDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuFeatureClassProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuRankProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuDepthProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuProjectedAreaProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuRenderAreaProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuRepresentedCountProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuCoverageCompensationProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    for (auto& descriptorSet : resources->gpuClampFlagsProbeDescriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            descriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet);
            descriptorSet = VK_NULL_HANDLE;
        }
    }
    if (resources->exrDescriptorSet != VK_NULL_HANDLE &&
        descriptorPool_ != VK_NULL_HANDLE &&
        device_ != VK_NULL_HANDLE) {
        vkFreeDescriptorSets(device_, descriptorPool_, 1, &resources->exrDescriptorSet);
    }
    *resources = ActivePointCloudResources{};
}

void VulkanViewportShell::CleanupGaussianSplatResources(ActiveGaussianSplatResources* resources) {
    if (resources == nullptr) {
        return;
    }

    DestroyBuffer(&resources->centerBuffer);
    DestroyBuffer(&resources->scaleBuffer);
    DestroyBuffer(&resources->rotationBuffer);
    DestroyBuffer(&resources->opacityBuffer);
    DestroyBuffer(&resources->shBuffer);

    for (auto& descriptorSets : resources->descriptorSets) {
        for (auto& descriptorSet : descriptorSets) {
            if (descriptorSet != VK_NULL_HANDLE &&
                gaussianSplatDescriptorPool_ != VK_NULL_HANDLE &&
                device_ != VK_NULL_HANDLE) {
                vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &descriptorSet);
            }
        }
        descriptorSets.clear();
    }

    *resources = ActiveGaussianSplatResources{};
}

void VulkanViewportShell::CleanupHighQualityGaussianScene() {
    DestroyBuffer(&highQualityGaussianScene_.centerBuffer);
    DestroyBuffer(&highQualityGaussianScene_.scaleBuffer);
    DestroyBuffer(&highQualityGaussianScene_.rotationBuffer);
    DestroyBuffer(&highQualityGaussianScene_.opacityBuffer);
    DestroyBuffer(&highQualityGaussianScene_.shBuffer);
    DestroyBuffer(&highQualityGaussianScene_.layerStyleIndexBuffer);
    for (auto& buffer : highQualityGaussianScene_.layerStyleBuffers) {
        DestroyBuffer(&buffer);
    }
    for (auto& buffer : highQualityGaussianScene_.sortedIndexBuffers) {
        DestroyBuffer(&buffer);
    }

    for (auto& descriptorSet : highQualityGaussianScene_.descriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            gaussianSplatDescriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &descriptorSet);
        }
    }

    highQualityGaussianScene_ = HighQualityGaussianSceneResources{};
}

void VulkanViewportShell::CleanupExrExportResources() {
    auto& resources = exrExportResources_;

    if (resources.commandBuffer != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(device_, commandPool_, 1, &resources.commandBuffer);
    }
    if (resources.fence != VK_NULL_HANDLE) {
        vkDestroyFence(device_, resources.fence, nullptr);
    }
    if (resources.framebuffer != VK_NULL_HANDLE) {
        vkDestroyFramebuffer(device_, resources.framebuffer, nullptr);
    }
    if (resources.pointDepthPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.pointDepthPipeline, nullptr);
    }
    if (resources.pointAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.pointAccumulationPipeline, nullptr);
    }
    if (resources.pointConstantSimpleAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.pointConstantSimpleAccumulationPipeline, nullptr);
    }
    if (resources.pointFastBasicDepthPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.pointFastBasicDepthPipeline, nullptr);
    }
    if (resources.pointFastBasicPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.pointFastBasicPipeline, nullptr);
    }
    if (resources.surfelDepthPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.surfelDepthPipeline, nullptr);
    }
    if (resources.surfelAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.surfelAccumulationPipeline, nullptr);
    }
    if (resources.surfelConstantSimpleAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.surfelConstantSimpleAccumulationPipeline, nullptr);
    }
    if (resources.compositePipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources.compositePipeline, nullptr);
    }
    if (resources.compositeDescriptorSet != VK_NULL_HANDLE &&
        descriptorPool_ != VK_NULL_HANDLE &&
        device_ != VK_NULL_HANDLE) {
        vkFreeDescriptorSets(device_, descriptorPool_, 1, &resources.compositeDescriptorSet);
    }
    if (resources.renderPass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, resources.renderPass, nullptr);
    }

    DestroyImage(&resources.colorImage);
    DestroyImage(&resources.depthImage);
    DestroyImage(&resources.accumulationImage);
    DestroyImage(&resources.revealageImage);
    DestroyImage(&resources.emissiveImage);
    DestroyImage(&resources.normalAccumulationImage);
    DestroyImage(&resources.albedoAccumulationImage);
    DestroyImage(&resources.linearDepthImage);
    DestroyImage(&resources.normalImage);
    DestroyImage(&resources.albedoImage);
    DestroyBuffer(&resources.colorReadbackBuffer);
    DestroyBuffer(&resources.depthReadbackBuffer);
    DestroyBuffer(&resources.normalReadbackBuffer);
    DestroyBuffer(&resources.albedoReadbackBuffer);

    resources = ExrExportResources{};
}

void VulkanViewportShell::RecreateSwapchain() {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    if (width <= 0 || height <= 0) {
        return;
    }

    vkDeviceWaitIdle(device_);

    CleanupSwapchain();
    CreateSwapchain();
    CreateImageViews();
    CreateSceneColorResources();
    CreateDepthResources();
    CreateAccumulationResources();
    CreateLinearDepthResources();
    CreateFramebuffers();
    CreatePresentFramebuffers();
    CreateCommandBuffers();
    CreateOrUpdateCompositeDescriptorSet();
    CreateOrUpdatePostProcessDescriptorSets();
    for (auto& resources : pointCloudResources_) {
        UpdatePointCloudDescriptorSets(&resources);
    }
    for (auto& resources : gaussianSplatResources_) {
        UpdateGaussianSplatDescriptorSets(&resources);
    }

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_SetMinImageCount(static_cast<int>(std::max<std::size_t>(2, swapchainImages_.size())));
    }
}

bool VulkanViewportShell::ResolvePointCloudDrawPlan(
    const SceneRenderState::PointCloudLayerState& layer,
    bool forceFullSource,
    PointCloudDrawPlan* plan) {
    if (plan == nullptr) {
        return false;
    }
    *plan = {};

    auto* resources = FindPointCloudResources(layer.layerId);
    if (resources == nullptr ||
        resources->activePointCount == 0 ||
        resources->descriptorSets[0].empty()) {
        return false;
    }

    std::uint32_t drawPointCount = 0;
    const bool worldSurfels =
        layer.style.geometryMode != renderer::pointcloud::PointCloudGeometryMode::ScreenSprites;
    const bool adaptiveDrawItemsReady =
        !forceFullSource &&
        layer.useAdaptiveDrawItems &&
        layer.drawPointCount > 0 &&
        resources->drawItemCount == layer.drawPointCount &&
        resources->drawItemSignature == layer.adaptiveLodRevision;
    if (adaptiveDrawItemsReady) {
        drawPointCount = resources->drawItemCount;
        if (worldSurfels) {
            drawPointCount = std::min(drawPointCount, kMaxSurfelEncodedPointCount);
        }
        if (drawPointCount == 0) {
            return false;
        }

        plan->resources = resources;
        plan->drawPointCount = drawPointCount;
        plan->worldSurfels = worldSurfels;
        plan->drawItemReady = true;
        return true;
    }

    if (!forceFullSource && layer.requiresAdaptiveDrawItems) {
        return false;
    }

    if (forceFullSource) {
        drawPointCount = resources->pointCount;
    } else {
        drawPointCount = layer.drawPointCount > 0
                             ? std::min(layer.drawPointCount, resources->activePointCount)
                             : resources->activePointCount;
    }
    if (drawPointCount == 0) {
        return false;
    }

    if (worldSurfels) {
        drawPointCount = std::min(drawPointCount, kMaxSurfelEncodedPointCount);
        if (drawPointCount == 0) {
            return false;
        }
    }

    const bool sampledBudgetReady =
        !forceFullSource &&
        resources->usingSampledIndices &&
        resources->sampledIndexBuffer.buffer != VK_NULL_HANDLE &&
        (!worldSurfels || resources->sampledSurfelIndexBuffer.buffer != VK_NULL_HANDLE);
    if (!forceFullSource &&
        !sampledBudgetReady &&
        drawPointCount < resources->activePointCount) {
        drawPointCount = resources->activePointCount;
    }
    if (worldSurfels) {
        drawPointCount = std::min(drawPointCount, kMaxSurfelEncodedPointCount);
        if (drawPointCount == 0) {
            return false;
        }
    }

    plan->resources = resources;
    plan->drawPointCount = drawPointCount;
    plan->worldSurfels = worldSurfels;
    plan->sampledBudgetReady = sampledBudgetReady;
    return true;
}

bool VulkanViewportShell::UploadPointCloudLayerStyle(
    const SceneRenderState::PointCloudLayerState& layer,
    const PointCloudDrawPlan& plan,
    std::size_t frameIndex,
    bool exrStyle) {
    auto* resources = plan.resources;
    if (resources == nullptr || plan.drawPointCount == 0 || frameIndex >= kFramesInFlight) {
        return false;
    }

    PointCloudStyleGpu styleGpu;
    styleGpu.solidColor = glm::vec4{
        layer.style.solidColor[0],
        layer.style.solidColor[1],
        layer.style.solidColor[2],
        layer.style.solidColor[3],
    };
    styleGpu.gradientStartColor = glm::vec4{
        layer.style.gradientStartColor[0],
        layer.style.gradientStartColor[1],
        layer.style.gradientStartColor[2],
        1.0F,
    };
    styleGpu.gradientEndColor = glm::vec4{
        layer.style.gradientEndColor[0],
        layer.style.gradientEndColor[1],
        layer.style.gradientEndColor[2],
        1.0F,
    };
    styleGpu.colorize = glm::vec4{
        layer.style.colorizeColor[0],
        layer.style.colorizeColor[1],
        layer.style.colorizeColor[2],
        std::clamp(layer.style.colorizeAmount, 0.0F, 1.0F),
    };
    styleGpu.globalControl = glm::uvec4{
        static_cast<std::uint32_t>(layer.style.colorMode),
        static_cast<std::uint32_t>(layer.style.colormap),
        resources->scalarFieldCount,
        layer.hasSourceRgb ? 1U : 0U,
    };
    styleGpu.pointMeta = glm::uvec4{
        resources->pointCount,
        plan.drawPointCount,
        resources->hasNormals ? 1U : 0U,
        layer.style.waterStreamOverlay ? 3U : (layer.style.flowAnimation ? (layer.style.waterPathView ? 2U : 1U) : 0U),
    };
    const bool forceDepthContribution =
        renderState_.eyeDomeLightingEnabled ||
        renderer::pointcloud::ResolvePointCloudMaterialVariant(layer.style) ==
            renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc;
    const auto effectiveDepthContribution =
        forceDepthContribution &&
                layer.style.depthContribution == renderer::pointcloud::PointCloudDepthContribution::None
            ? renderer::pointcloud::PointCloudDepthContribution::Always
            : layer.style.depthContribution;
    const std::uint32_t solidCenterFlag = layer.style.solidCenters ? 1U : 0U;
    const std::uint32_t drawItemFlag = plan.drawItemReady ? 2U : 0U;
    styleGpu.renderControl = glm::uvec4{
        static_cast<std::uint32_t>(effectiveDepthContribution),
        static_cast<std::uint32_t>(layer.style.falloffProfile),
        static_cast<std::uint32_t>(layer.style.geometryMode),
        solidCenterFlag | drawItemFlag,
    };
    styleGpu.renderParams0 = glm::vec4{
        layer.style.exposure,
        layer.style.innerRadius,
        layer.style.gaussianSharpness,
        layer.style.featherPower,
    };
    styleGpu.renderParams1 = glm::vec4{
        layer.style.depthFalloff,
        layer.style.depthBias,
        layer.style.frontAlpha,
        layer.style.hiddenAlpha,
    };
    styleGpu.renderParams2 = glm::vec4{
        layer.style.densityScale,
        layer.style.densityClamp,
        std::clamp(layer.style.waterStreakAspect, 1.0F, 32.0F),
        renderer::pointcloud::PointCloudStyleUsesWorldSizedScreenSprites(layer.style) ? 1.0F : 0.0F,
    };
    styleGpu.renderParams3 = glm::vec4{
        layer.style.depthAlphaThreshold,
        pointSizeRangeMin_,
        pointSizeRangeMax_,
        std::max(0.0F, renderState_.flowTimeSeconds),
    };
    styleGpu.stylisationControl = glm::uvec4{
        static_cast<std::uint32_t>(layer.style.stylisationMode),
        static_cast<std::uint32_t>(layer.style.nprPreset),
        0U,
        0U,
    };
    styleGpu.stylisationParams0 = glm::vec4{
        std::clamp(layer.style.stylisationStrength, 0.0F, 1.0F),
        std::clamp(layer.style.stylisationColorLevels, 2.0F, 16.0F),
        std::clamp(layer.style.stylisationInkStrength, 0.0F, 1.0F),
        std::clamp(layer.style.stylisationPaperGrain, 0.0F, 1.0F),
    };
    styleGpu.stylisationParams1 = glm::vec4{
        std::clamp(layer.style.stylisationPigmentBleed, 0.0F, 1.0F),
        std::clamp(layer.style.brushAspect, 0.25F, 6.0F),
        std::clamp(layer.style.strokeJitter, 0.0F, 1.0F),
        std::clamp(layer.style.hatchStrength, 0.0F, 1.0F),
    };
    styleGpu.stylisationParams2 = glm::vec4{
        std::clamp(layer.style.strokeOpacityVariance, 0.0F, 1.0F),
        std::clamp(layer.style.pigmentVariation, 0.0F, 1.0F),
        std::clamp(layer.style.pigmentAnimationSpeed, 0.0F, 4.0F),
        std::clamp(layer.style.granulationAngleStrength, 0.0F, 1.0F),
    };
    if (layer.style.roughnessMotionStrength > 1.0e-5F) {
        const auto roughnessSlot = FindRoughnessScalarFieldSlot(layer.scalarFields);
        if (roughnessSlot.has_value() && roughnessSlot.value() < layer.scalarFields.size()) {
            const auto& roughnessStats = layer.scalarFields[roughnessSlot.value()];
            const float roughnessRange =
                std::max(1.0e-6F, roughnessStats.maximum - roughnessStats.minimum);
            styleGpu.stylisationControl.z = roughnessSlot.value() + 1U;
            if (const auto groundSlot = FindGroundIdScalarFieldSlot(layer.scalarFields);
                groundSlot.has_value() && groundSlot.value() < layer.scalarFields.size()) {
                styleGpu.stylisationControl.w = groundSlot.value() + 1U;
            }
            styleGpu.surfaceMotionParams = glm::vec4{
                std::clamp(layer.style.roughnessMotionStrength, 0.0F, 1.0F),
                std::clamp(layer.style.roughnessMotionScale, 0.01F, 50.0F),
                std::clamp(layer.style.roughnessMotionSpeed, 0.0F, 8.0F),
                std::clamp(layer.style.roughnessMotionThreshold, 0.0F, 1.0F),
            };
            styleGpu.surfaceMotionStats = glm::vec4{
                roughnessStats.minimum,
                1.0F / roughnessRange,
                std::clamp(layer.style.roughnessMotionGroundId, 0.0F, 1.0F),
                0.25F,
            };
        }
    }
    if (renderer::pointcloud::PointCloudStyleHasActiveCaustics(layer.style)) {
        styleGpu.causticControl = glm::uvec4{
            1U,
            static_cast<std::uint32_t>(layer.style.causticMaskFieldSlot + 1),
            static_cast<std::uint32_t>(layer.style.causticEdgeFieldSlot + 1),
            static_cast<std::uint32_t>(layer.style.causticSeedFieldSlot + 1),
        };
        styleGpu.causticParams0 = glm::vec4{
            std::clamp(layer.style.causticIntensity, 0.0F, 5.0F),
            std::clamp(layer.style.causticCellSizeMeters, 0.005F, 5.0F),
            std::clamp(layer.style.causticSpeed, 0.0F, 10.0F),
            std::clamp(layer.style.causticLineWidthMeters, 0.0005F, 0.50F),
        };
        styleGpu.causticParams1 = glm::vec4{
            std::clamp(layer.style.causticWarpAmplitudeMeters, 0.0F, 2.0F),
            std::clamp(layer.style.causticEmissionBoost, 0.0F, 8.0F),
            std::clamp(layer.style.causticOpacityBoost, 0.0F, 2.0F),
            std::clamp(layer.style.causticPointSizeBoost, 0.0F, 4.0F),
        };
        styleGpu.causticParams2 = glm::vec4{
            std::clamp(layer.style.causticFeatherMeters, 0.0005F, 0.50F),
            std::clamp(layer.style.causticSurfacePointSpacingMeters, 0.0005F, 0.10F),
            std::clamp(layer.style.causticPreviewTintAmount, 0.0F, 1.0F),
            std::clamp(layer.style.causticPreviewTintRegionId, 0.0F, 16777216.0F),
        };
        styleGpu.causticTint = glm::vec4{
            std::clamp(layer.style.causticTint[0], 0.0F, 4.0F),
            std::clamp(layer.style.causticTint[1], 0.0F, 4.0F),
            std::clamp(layer.style.causticTint[2], 0.0F, 4.0F),
            1.0F,
        };
    }
    const auto waterEffectEmissionAddSlot = FindExactScalarFieldSlot(layer.scalarFields, "water_effect_emission_add");
    const auto waterEffectOpacityAddSlot = FindExactScalarFieldSlot(layer.scalarFields, "water_effect_opacity_add");
    const auto waterEffectOpacityMultiplySlot =
        FindExactScalarFieldSlot(layer.scalarFields, "water_effect_opacity_multiply");
    const auto waterEffectPointSizeAddSlot =
        FindExactScalarFieldSlot(layer.scalarFields, "water_effect_point_size_add");
    const auto waterEffectPointSizeMultiplySlot =
        FindExactScalarFieldSlot(layer.scalarFields, "water_effect_point_size_multiply");
    const auto waterEffectColourRedSlot = FindExactScalarFieldSlot(layer.scalarFields, "water_effect_colour_red");
    const auto waterEffectColourGreenSlot = FindExactScalarFieldSlot(layer.scalarFields, "water_effect_colour_green");
    const auto waterEffectColourBlueSlot = FindExactScalarFieldSlot(layer.scalarFields, "water_effect_colour_blue");
    const auto waterEffectColourMixSlot = FindExactScalarFieldSlot(layer.scalarFields, "water_effect_colour_mix");
    if (waterEffectEmissionAddSlot.has_value() &&
        waterEffectOpacityAddSlot.has_value() &&
        waterEffectOpacityMultiplySlot.has_value() &&
        waterEffectPointSizeAddSlot.has_value() &&
        waterEffectPointSizeMultiplySlot.has_value() &&
        waterEffectColourRedSlot.has_value() &&
        waterEffectColourGreenSlot.has_value() &&
        waterEffectColourBlueSlot.has_value() &&
        waterEffectColourMixSlot.has_value()) {
        styleGpu.waterEffectControl = glm::uvec4{
            1U,
            waterEffectEmissionAddSlot.value() + 1U,
            waterEffectOpacityAddSlot.value() + 1U,
            waterEffectOpacityMultiplySlot.value() + 1U,
        };
        styleGpu.waterEffectSlots0 = glm::uvec4{
            waterEffectPointSizeAddSlot.value() + 1U,
            waterEffectPointSizeMultiplySlot.value() + 1U,
            waterEffectColourMixSlot.value() + 1U,
            waterEffectColourRedSlot.value() + 1U,
        };
        styleGpu.waterEffectSlots1 = glm::uvec4{
            waterEffectColourGreenSlot.value() + 1U,
            waterEffectColourBlueSlot.value() + 1U,
            0U,
            0U,
        };
    }
    styleGpu.pointSize = MakePointCloudBindingGpu(
        layer.style.pointSize,
        layer.scalarFields,
        renderer::pointcloud::kInactivePointSizeDefault);
    ScalePointCloudBindingGpu(&styleGpu.pointSize, renderState_.pointSizeScale);
    styleGpu.opacity = MakePointCloudBindingGpu(
        layer.style.opacity,
        layer.scalarFields,
        renderer::pointcloud::kInactiveOpacityDefault);
    styleGpu.emissive = MakePointCloudBindingGpu(
        layer.style.emissiveStrength,
        layer.scalarFields,
        renderer::pointcloud::kInactiveEmissionDefault);
    styleGpu.xray = MakePointCloudBindingGpu(
        layer.style.xrayStrength,
        layer.scalarFields,
        renderer::pointcloud::kInactiveXrayDefault);
    styleGpu.depthFade = MakePointCloudBindingGpu(
        layer.style.depthFade,
        layer.scalarFields,
        renderer::pointcloud::kInactiveDepthFadeDefault);
    styleGpu.colormapPosition = MakePointCloudBindingGpu(
        layer.style.colormapPosition,
        layer.scalarFields,
        renderer::pointcloud::kInactiveColormapPositionDefault);
    styleGpu.surfelDiameter = MakePointCloudBindingGpu(
        layer.style.surfelDiameter,
        layer.scalarFields,
        renderer::pointcloud::kInactiveSurfelDiameterDefault);

    UploadBufferData(
        exrStyle ? resources->exrStyleBuffer : resources->styleBuffers[frameIndex],
        &styleGpu,
        sizeof(styleGpu));
    return true;
}

bool VulkanViewportShell::PointCloudPlanUsesGpuCompaction(
    const PointCloudDrawPlan& plan,
    std::size_t frameIndex,
    bool exrStyle) const {
    if (exrStyle ||
        frameIndex >= kFramesInFlight ||
        !gpuDrivenSelectionCapabilities_.computeQueueSupported ||
        gpuDrawItemCompactionPipeline_ == VK_NULL_HANDLE ||
        gpuCompactionPipelineLayout_ == VK_NULL_HANDLE ||
        !plan.drawItemReady ||
        plan.drawPointCount < kMinimumAdaptiveIndirectDrawPoints ||
        plan.resources == nullptr) {
        return false;
    }

    const auto requiredCompactedOutputCapacity =
        GpuDiagnosticCompactionOutputCapacity(plan.drawPointCount);
    return plan.resources->drawItemBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
           plan.resources->gpuCompactedDrawItemBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
           plan.resources->gpuCompactedDrawItemCapacities[frameIndex] >= requiredCompactedOutputCapacity &&
           plan.resources->gpuCompactionStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
           plan.resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
           plan.resources->positionStorageBuffer.buffer != VK_NULL_HANDLE &&
           plan.resources->gpuCompactionIndirectDescriptorSets[frameIndex] != VK_NULL_HANDLE &&
           plan.resources->gpuCompactionDescriptorSets[frameIndex] != VK_NULL_HANDLE;
}

bool VulkanViewportShell::PointCloudPlanUsesGpuCompactionSubmission(
    const PointCloudDrawPlan& plan,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    bool exrStyle) const {
    if (exrStyle ||
        frameIndex >= kFramesInFlight ||
        !gpuDrivenSelectionCapabilities_.indirectDrawSupported ||
        !plan.drawItemReady ||
        plan.drawPointCount < kMinimumAdaptiveIndirectDrawPoints ||
        plan.resources == nullptr ||
        !plan.resources->gpuCompactionSubmissionEligible[frameIndex] ||
        plan.resources->gpuCompactionSubmissionVertexCounts[frameIndex] == 0U ||
        imageIndex >= plan.resources->gpuCompactedDescriptorSets[frameIndex].size()) {
        return false;
    }

    return plan.resources->gpuCompactedDescriptorSets[frameIndex][imageIndex] != VK_NULL_HANDLE &&
           plan.resources->gpuCompactedDrawItemBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
           plan.resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer != VK_NULL_HANDLE;
}

void VulkanViewportShell::ResetGpuCompactionSubmissionFrame(std::size_t frameIndex) {
    if (frameIndex >= kFramesInFlight) {
        return;
    }
    for (auto& resources : pointCloudResources_) {
        resources.gpuCompactionSubmissionEligible[frameIndex] = false;
        resources.gpuCompactionSubmissionVertexCounts[frameIndex] = 0;
    }
}

bool VulkanViewportShell::RecordGpuDrawItemCompactionForScene(
    VkCommandBuffer commandBuffer,
    std::size_t frameIndex,
    bool forceFullSource) {
    if (frameIndex < kFramesInFlight) {
        ResetGpuCompactionPerformanceFrame(frameIndex);
        DecayGpuCompactionPerformanceCooldowns();
    }
    if (commandBuffer == VK_NULL_HANDLE ||
        frameIndex >= kFramesInFlight ||
        gpuDrawItemCompactionPipeline_ == VK_NULL_HANDLE ||
        gpuCompactionPipelineLayout_ == VK_NULL_HANDLE) {
        return false;
    }

    bool recordedAny = false;
    bool compactionPipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        const auto performanceSlowFrames =
            performanceGate.retryCooldownFrames > 0U
                ? kGpuDiagnosticCompactionSlowFrameThreshold
                : performanceGate.consecutiveSlowerFrames;
        diagnostics_.adaptiveGpuCompactionPerformanceSlowFrames = std::max(
            diagnostics_.adaptiveGpuCompactionPerformanceSlowFrames,
            performanceSlowFrames);
        diagnostics_.adaptiveGpuCompactionPerformanceRetryFrames = std::max(
            diagnostics_.adaptiveGpuCompactionPerformanceRetryFrames,
            performanceGate.retryCooldownFrames);
        if (performanceGate.retryCooldownFrames > 0U) {
            diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason =
                GpuCompactionPerformanceFallbackReason(layer.adaptiveRendererCostProfile);
            continue;
        }

        if (!recordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuDrawItemCompactionPass,
                false);
            recordedAny = true;
        }
        if (!compactionPipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            compactionPipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels = kGpuDiagnosticMinSelectionFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels = kGpuDiagnosticMaxSelectionFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels = kGpuDiagnosticMinSelectionRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels = kGpuDiagnosticMaxSelectionRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr bool selectionWriteOutput = kGpuDiagnosticCompactionOutputWriteEnabled;
        const auto selectionMaxOutputCount =
            GpuDiagnosticCompactionOutputCapacity(plan.drawPointCount);
        constexpr bool selectionFrustumEnabled = kGpuDiagnosticSelectionFrustumGuardEnabled;
        const auto selectionPositionCount = selectionFrustumEnabled
                                                ? static_cast<std::uint32_t>(
                                                      std::min<std::size_t>(
                                                          plan.resources->cpuPositions.size(),
                                                          std::numeric_limits<std::uint32_t>::max()))
                                                : 0U;
        constexpr float selectionFrustumGuardBand =
            selectionFrustumEnabled ? kGpuDiagnosticSelectionFrustumGuardBand : 0.0F;
        GpuDrawItemOutputProbeStats expectedOutputProbeStats{};
        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                &expectedOutputProbeStats,
                selectionMaxOutputCount);
        const double cpuReferenceMs = MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());
        diagnostics_.adaptiveGpuCompactionCpuReferenceMs += cpuReferenceMs;
        gpuCompactionCpuReferenceMsByFrame_[frameIndex][performanceProfileIndex] += cpuReferenceMs;
        const bool selectionOrderedOutput =
            selectionWriteOutput &&
            kGpuDiagnosticCompactionOrderedOutputEnabled &&
            expectedStats.count == selectionLimit &&
            expectedStats.count <= selectionMaxOutputCount;
        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuCompactionStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuCompactionExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuCompactionResultPending[frameIndex] = true;
        plan.resources->gpuCompactionExpectedOutputProbeStats[frameIndex] = expectedOutputProbeStats;
        plan.resources->gpuCompactionOutputProbeResultPending[frameIndex] =
            selectionOrderedOutput;
        const auto compactedSubmissionCandidateVertices =
            IndirectVertexCountFromCompactedItems(expectedStats.count, plan.worldSurfels);
        const auto compactedSubmissionReferenceVertices =
            IndirectVertexCountFromCompactedItems(plan.drawPointCount, plan.worldSurfels);
        const bool compactedSubmissionOutputFits =
            selectionWriteOutput && expectedStats.count <= selectionMaxOutputCount;
        const bool compactedSubmissionSemanticEquivalent =
            compactedSubmissionCandidateVertices == compactedSubmissionReferenceVertices;
        const bool compactedSubmissionOrderedOutput = selectionOrderedOutput;
        const bool compactedSubmissionOutputProbePassed =
            diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus.rfind("passed", 0) == 0;
        const bool compactedSubmissionEligible =
            compactedSubmissionOutputFits &&
            compactedSubmissionSemanticEquivalent &&
            compactedSubmissionOrderedOutput &&
            compactedSubmissionOutputProbePassed &&
            diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason.empty();
        diagnostics_.adaptiveGpuCompactionSubmissionEligible =
            diagnostics_.adaptiveGpuCompactionSubmissionEligible || compactedSubmissionEligible;
        diagnostics_.adaptiveGpuCompactionSubmissionCandidateVertices += compactedSubmissionCandidateVertices;
        diagnostics_.adaptiveGpuCompactionSubmissionReferenceVertices += compactedSubmissionReferenceVertices;
        plan.resources->gpuCompactionSubmissionEligible[frameIndex] = compactedSubmissionEligible;
        plan.resources->gpuCompactionSubmissionVertexCounts[frameIndex] =
            compactedSubmissionEligible ? compactedSubmissionCandidateVertices : 0U;
        if (!selectionWriteOutput) {
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason =
                "compacted output not submitted: diagnostic output writes are disabled";
        } else if (!compactedSubmissionOutputFits) {
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason =
                "compacted output not submitted: selected count exceeds diagnostic output capacity";
        } else if (!compactedSubmissionSemanticEquivalent) {
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason =
                "compacted output not submitted: GPU diagnostic predicate selected " +
                std::to_string(compactedSubmissionCandidateVertices) +
                " vertices from " +
                std::to_string(compactedSubmissionReferenceVertices) +
                " CPU-submitted vertices; submitted compacted draws require semantic-equivalent selection or explicit visual acceptance";
        } else if (!compactedSubmissionOrderedOutput) {
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason =
                "compacted output not submitted: ordered diagnostic output requires the full CPU-selected draw-item range to fit and be selected";
        } else if (!compactedSubmissionOutputProbePassed) {
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason =
                "compacted output not submitted: waiting for previous-frame ordered compacted output identity parity";
        } else if (!diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason.empty()) {
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason =
                "compacted output not submitted: " +
                diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason;
        } else {
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason =
                "compacted output pending graphics submission after parity gates";
        }

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                selectionPositionCount,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                selectionWriteOutput ? 1U : 0U,
                selectionMaxOutputCount,
                selectionOrderedOutput ? 1U : 0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuCompactionDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        if (selectionWriteOutput && selectionMaxOutputCount > 0U) {
            VkBufferMemoryBarrier outputProbeBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
            outputProbeBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            outputProbeBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT | VK_ACCESS_SHADER_READ_BIT;
            outputProbeBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            outputProbeBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            outputProbeBarrier.buffer = plan.resources->gpuCompactedDrawItemBuffers[frameIndex].buffer;
            outputProbeBarrier.offset = 0;
            outputProbeBarrier.size = static_cast<VkDeviceSize>(
                selectionMaxOutputCount * sizeof(renderer::pointcloud::PointCloudDrawItemGpu));
            vkCmdPipelineBarrier(
                commandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_HOST_BIT | VK_PIPELINE_STAGE_VERTEX_SHADER_BIT,
                0,
                0,
                nullptr,
                1,
                &outputProbeBarrier,
                0,
                nullptr);
        }

        if (gpuDrivenIndirectCommandPipeline_ != VK_NULL_HANDLE &&
            gpuDrivenSelectionPipelineLayout_ != VK_NULL_HANDLE &&
            plan.resources->gpuCompactionIndirectDescriptorSets[frameIndex] != VK_NULL_HANDLE) {
            VkBufferMemoryBarrier statsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
            statsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            statsBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            statsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            statsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            statsBarrier.buffer = plan.resources->gpuCompactionStatsBuffers[frameIndex].buffer;
            statsBarrier.offset = 0;
            statsBarrier.size = sizeof(GpuDrawItemCompactionStats);
            vkCmdPipelineBarrier(
                commandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                0,
                0,
                nullptr,
                1,
                &statsBarrier,
                0,
                nullptr);

            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrivenIndirectCommandPipeline_);
            compactionPipelineBound = false;
            const auto vertexMultiplier = plan.worldSurfels ? kSurfelVerticesPerPoint : 1U;
            const GpuDrivenIndirectCommandPushConstants indirectPushConstants{
                glm::uvec4{vertexMultiplier, 1U, 0U, kGpuIndirectCommandModeFromCompactionStats}};
            VkDescriptorSet indirectDescriptorSet =
                plan.resources->gpuCompactionIndirectDescriptorSets[frameIndex];
            vkCmdBindDescriptorSets(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrivenSelectionPipelineLayout_,
                0,
                1,
                &indirectDescriptorSet,
                0,
                nullptr);
            vkCmdPushConstants(
                commandBuffer,
                gpuDrivenSelectionPipelineLayout_,
                VK_SHADER_STAGE_COMPUTE_BIT,
                0,
                sizeof(GpuDrivenIndirectCommandPushConstants),
                &indirectPushConstants);
            vkCmdDispatch(commandBuffer, 1, 1, 1);

            VkBufferMemoryBarrier commandBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
            commandBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            commandBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT | VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
            commandBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            commandBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            commandBarrier.buffer = plan.resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer;
            commandBarrier.offset = 0;
            commandBarrier.size = sizeof(VkDrawIndirectCommand);
            vkCmdPipelineBarrier(
                commandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_HOST_BIT | VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
                0,
                0,
                nullptr,
                1,
                &commandBarrier,
                0,
                nullptr);

            const VkDrawIndirectCommand expectedIndirectCommand{
                IndirectVertexCountFromCompactedItems(expectedStats.count, plan.worldSurfels),
                1U,
                0U,
                0U,
            };
            plan.resources->gpuCompactionExpectedIndirectCommands[frameIndex] = expectedIndirectCommand;
            plan.resources->gpuCompactionIndirectCommandResultPending[frameIndex] = true;
            diagnostics_.adaptiveGpuCompactionIndirectCommandUsed = true;
            diagnostics_.adaptiveGpuCompactionIndirectCommandDispatches += 1U;
        }

        diagnostics_.adaptiveGpuCompactionUsed = true;
        diagnostics_.adaptiveGpuCompactionDispatches += 1U;
        diagnostics_.adaptiveGpuCompactionInputDrawItems += plan.drawPointCount;
        diagnostics_.adaptiveGpuCompactionDispatchedDrawItems += dispatchItemCount;
        diagnostics_.adaptiveGpuCompactionSelectionLimit += selectionLimit;
        diagnostics_.adaptiveGpuCompactionSelectionProfileMask |= selectionProfileMask;
        diagnostics_.adaptiveGpuCompactionSelectionClassMask |= selectionClassMask;
        diagnostics_.adaptiveGpuCompactionSelectionRankLimit = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionRankLimit,
            selectionRankLimit);
        diagnostics_.adaptiveGpuCompactionSelectionMinDepth = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMinDepth,
            selectionMinDepth);
        diagnostics_.adaptiveGpuCompactionSelectionMaxDepth = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMaxDepth,
            selectionMaxDepth);
        diagnostics_.adaptiveGpuCompactionSelectionRequiredFlags |= selectionRequiredFlags;
        diagnostics_.adaptiveGpuCompactionSelectionRejectedFlags |= selectionRejectedFlags;
        diagnostics_.adaptiveGpuCompactionSelectionMinFootprintAreaPixels = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMinFootprintAreaPixels,
            selectionMinFootprintAreaPixels);
        diagnostics_.adaptiveGpuCompactionSelectionMaxFootprintAreaPixels = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMaxFootprintAreaPixels,
            selectionMaxFootprintAreaPixels);
        diagnostics_.adaptiveGpuCompactionSelectionMinRenderAreaPixels = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMinRenderAreaPixels,
            selectionMinRenderAreaPixels);
        diagnostics_.adaptiveGpuCompactionSelectionMaxRenderAreaPixels = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMaxRenderAreaPixels,
            selectionMaxRenderAreaPixels);
        diagnostics_.adaptiveGpuCompactionSelectionMinOpacityCompensation = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMinOpacityCompensation,
            selectionMinOpacityCompensation);
        diagnostics_.adaptiveGpuCompactionSelectionMaxOpacityCompensation = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMaxOpacityCompensation,
            selectionMaxOpacityCompensation);
        diagnostics_.adaptiveGpuCompactionSelectionMinEmissionCompensation = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMinEmissionCompensation,
            selectionMinEmissionCompensation);
        diagnostics_.adaptiveGpuCompactionSelectionMaxEmissionCompensation = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMaxEmissionCompensation,
            selectionMaxEmissionCompensation);
        diagnostics_.adaptiveGpuCompactionSelectionMinRepresentedSourceCount = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMinRepresentedSourceCount,
            selectionMinRepresentedSourceCount);
        diagnostics_.adaptiveGpuCompactionSelectionMaxRepresentedSourceCount = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionMaxRepresentedSourceCount,
            selectionMaxRepresentedSourceCount);
        diagnostics_.adaptiveGpuCompactionSelectionPositionCount = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionPositionCount,
            selectionPositionCount);
        diagnostics_.adaptiveGpuCompactionSelectionFrustumGuardBand = std::max(
            diagnostics_.adaptiveGpuCompactionSelectionFrustumGuardBand,
            selectionFrustumGuardBand);
        diagnostics_.adaptiveGpuCompactionSelectionFrustumEnabled =
            diagnostics_.adaptiveGpuCompactionSelectionFrustumEnabled || selectionFrustumEnabled;
        if (!selectionFrustumEnabled) {
            diagnostics_.adaptiveGpuCompactionSelectionFrustumFallbackReason =
                std::string(kGpuDiagnosticSelectionFrustumFallbackReason);
        }
        diagnostics_.adaptiveGpuCompactionOutputWriteEnabled =
            diagnostics_.adaptiveGpuCompactionOutputWriteEnabled || selectionWriteOutput;
        if (selectionWriteOutput) {
            diagnostics_.adaptiveGpuCompactionOutputWriteFallbackReason.clear();
            diagnostics_.adaptiveGpuCompactionOutputCapacity += selectionMaxOutputCount;
            diagnostics_.adaptiveGpuCompactionCopiedDrawItems +=
                std::min(expectedStats.count, selectionMaxOutputCount);
            if (expectedStats.count > selectionMaxOutputCount) {
                diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus =
                    "not checked: selected count exceeds diagnostic output capacity";
                diagnostics_.adaptiveGpuCompactionOutputProbeCpuCount = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeGpuCount = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeCpuChecksum = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeGpuChecksum = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeCpuSourceFingerprint = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeGpuSourceFingerprint = 0;
            } else if (!selectionOrderedOutput) {
                diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus =
                    "not checked: ordered compacted output requires semantic-equivalent full-range selection";
                diagnostics_.adaptiveGpuCompactionOutputProbeCpuCount = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeGpuCount = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeCpuChecksum = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeGpuChecksum = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeCpuSourceFingerprint = 0;
                diagnostics_.adaptiveGpuCompactionOutputProbeGpuSourceFingerprint = 0;
            }
        } else {
            diagnostics_.adaptiveGpuCompactionOutputWriteFallbackReason =
                "compacted draw-item output writes are disabled by policy";
            diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus =
                "not checked: compacted output writes disabled";
        }
    }

    if (recordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuDrawItemCompactionPass,
            true);
        diagnostics_.adaptiveSelectionExecutionPath =
            "cpu-selection+gpu-full-range-selection-compare+gpu-generated-indirect";
        diagnostics_.adaptiveSelectionFallbackReason =
            "GPU compute selection remains on CPU fallback until full selection parity/timing are proven; "
            "CPU-selected representative draw items are full-range semantic-equivalent filtered, workgroup-aggregated, count-compacted, class-counted, checksummed, optionally copied into an ordered diagnostic output buffer, and converted to a diagnostic indirect command by compute for comparison; "
            "the geometry-frustum predicate remains disabled after slower MoltenVK timing";
        if (!diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason.empty()) {
            diagnostics_.adaptiveSelectionFallbackReason +=
                "; GPU full-range compaction performance fallback: " +
                diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason;
        }
        if (diagnostics_.adaptiveGpuCompactionParityStatus == "not checked") {
            diagnostics_.adaptiveGpuCompactionParityStatus = "waiting for previous-frame full-range GPU checksum";
        }
        if (diagnostics_.adaptiveGpuCompactionIndirectCommandUsed &&
            diagnostics_.adaptiveGpuCompactionIndirectCommandParityStatus == "not checked") {
            diagnostics_.adaptiveGpuCompactionIndirectCommandParityStatus =
                "waiting for previous-frame compacted indirect command";
        }
        if (diagnostics_.adaptiveGpuCompactionOutputWriteEnabled &&
            diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuCompactionOutputProbeParityStatus =
                "waiting for previous-frame ordered compacted output probe";
        }
    }

    bool featureProbeRecordedAny = false;
    bool featureProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuFeatureClassProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuFeatureClassProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!featureProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuFeatureClassProbePass,
                false);
            featureProbeRecordedAny = true;
        }
        if (!featureProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            featureProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticFeatureClassProbeMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels = kGpuDiagnosticMinSelectionFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels = kGpuDiagnosticMaxSelectionFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels = kGpuDiagnosticMinSelectionRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels = kGpuDiagnosticMaxSelectionRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuFeatureClassProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuFeatureClassProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuFeatureClassProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuFeatureClassProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuFeatureClassProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuFeatureClassProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuFeatureClassProbeUsed = true;
        diagnostics_.adaptiveGpuFeatureClassProbeDispatches += 1U;
        diagnostics_.adaptiveGpuFeatureClassProbeMask |= selectionClassMask;
    }

    if (featureProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuFeatureClassProbePass,
            true);
        if (diagnostics_.adaptiveGpuFeatureClassProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuFeatureClassProbeParityStatus =
                "waiting for previous-frame protected feature-class GPU checksum";
        }
    }

    bool rankProbeRecordedAny = false;
    bool rankProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuRankProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuRankProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!rankProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuRankProbePass,
                false);
            rankProbeRecordedAny = true;
        }
        if (!rankProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            rankProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankProbeLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels = kGpuDiagnosticMinSelectionFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels = kGpuDiagnosticMaxSelectionFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels = kGpuDiagnosticMinSelectionRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels = kGpuDiagnosticMaxSelectionRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuRankProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuRankProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuRankProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuRankProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuRankProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuRankProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuRankProbeUsed = true;
        diagnostics_.adaptiveGpuRankProbeDispatches += 1U;
        diagnostics_.adaptiveGpuRankProbeLimit = std::max(
            diagnostics_.adaptiveGpuRankProbeLimit,
            selectionRankLimit);
    }

    if (rankProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuRankProbePass,
            true);
        if (diagnostics_.adaptiveGpuRankProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuRankProbeParityStatus =
                "waiting for previous-frame stable-rank prefix GPU checksum";
        }
    }

    bool depthProbeRecordedAny = false;
    bool depthProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuDepthProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuDepthProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!depthProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuDepthProbePass,
                false);
            depthProbeRecordedAny = true;
        }
        if (!depthProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            depthProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticDepthProbeMin;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticDepthProbeMax;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels = kGpuDiagnosticMinSelectionFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels = kGpuDiagnosticMaxSelectionFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels = kGpuDiagnosticMinSelectionRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels = kGpuDiagnosticMaxSelectionRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuDepthProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuDepthProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuDepthProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuDepthProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuDepthProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuDepthProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuDepthProbeUsed = true;
        diagnostics_.adaptiveGpuDepthProbeDispatches += 1U;
        diagnostics_.adaptiveGpuDepthProbeMinDepth = std::max(
            diagnostics_.adaptiveGpuDepthProbeMinDepth,
            selectionMinDepth);
        diagnostics_.adaptiveGpuDepthProbeMaxDepth = std::max(
            diagnostics_.adaptiveGpuDepthProbeMaxDepth,
            selectionMaxDepth);
    }

    if (depthProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuDepthProbePass,
            true);
        if (diagnostics_.adaptiveGpuDepthProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuDepthProbeParityStatus =
                "waiting for previous-frame hierarchy-depth GPU checksum";
        }
    }

    bool projectedAreaProbeRecordedAny = false;
    bool projectedAreaProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuProjectedAreaProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuProjectedAreaProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!projectedAreaProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuProjectedAreaProbePass,
                false);
            projectedAreaProbeRecordedAny = true;
        }
        if (!projectedAreaProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            projectedAreaProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels =
            kGpuDiagnosticProjectedAreaProbeMinFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels =
            kGpuDiagnosticProjectedAreaProbeMaxFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels =
            kGpuDiagnosticProjectedAreaProbeMinRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels =
            kGpuDiagnosticProjectedAreaProbeMaxRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuProjectedAreaProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuProjectedAreaProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuProjectedAreaProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuProjectedAreaProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuProjectedAreaProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuProjectedAreaProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuProjectedAreaProbeUsed = true;
        diagnostics_.adaptiveGpuProjectedAreaProbeDispatches += 1U;
        diagnostics_.adaptiveGpuProjectedAreaProbeMinFootprintAreaPixels = std::max(
            diagnostics_.adaptiveGpuProjectedAreaProbeMinFootprintAreaPixels,
            selectionMinFootprintAreaPixels);
        diagnostics_.adaptiveGpuProjectedAreaProbeMaxFootprintAreaPixels = std::max(
            diagnostics_.adaptiveGpuProjectedAreaProbeMaxFootprintAreaPixels,
            selectionMaxFootprintAreaPixels);
        diagnostics_.adaptiveGpuProjectedAreaProbeMinRenderAreaPixels = std::max(
            diagnostics_.adaptiveGpuProjectedAreaProbeMinRenderAreaPixels,
            selectionMinRenderAreaPixels);
        diagnostics_.adaptiveGpuProjectedAreaProbeMaxRenderAreaPixels = std::max(
            diagnostics_.adaptiveGpuProjectedAreaProbeMaxRenderAreaPixels,
            selectionMaxRenderAreaPixels);
    }

    if (projectedAreaProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuProjectedAreaProbePass,
            true);
        if (diagnostics_.adaptiveGpuProjectedAreaProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuProjectedAreaProbeParityStatus =
                "waiting for previous-frame projected-area GPU checksum";
        }
    }

    bool renderAreaProbeRecordedAny = false;
    bool renderAreaProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuRenderAreaProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuRenderAreaProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!renderAreaProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuRenderAreaProbePass,
                false);
            renderAreaProbeRecordedAny = true;
        }
        if (!renderAreaProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            renderAreaProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels =
            kGpuDiagnosticRenderAreaProbeMinFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels =
            kGpuDiagnosticRenderAreaProbeMaxFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels =
            kGpuDiagnosticRenderAreaProbeMinRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels =
            kGpuDiagnosticRenderAreaProbeMaxRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuRenderAreaProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuRenderAreaProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuRenderAreaProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuRenderAreaProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuRenderAreaProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuRenderAreaProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuRenderAreaProbeUsed = true;
        diagnostics_.adaptiveGpuRenderAreaProbeDispatches += 1U;
        diagnostics_.adaptiveGpuRenderAreaProbeMinFootprintAreaPixels = std::max(
            diagnostics_.adaptiveGpuRenderAreaProbeMinFootprintAreaPixels,
            selectionMinFootprintAreaPixels);
        diagnostics_.adaptiveGpuRenderAreaProbeMaxFootprintAreaPixels = std::max(
            diagnostics_.adaptiveGpuRenderAreaProbeMaxFootprintAreaPixels,
            selectionMaxFootprintAreaPixels);
        diagnostics_.adaptiveGpuRenderAreaProbeMinRenderAreaPixels = std::max(
            diagnostics_.adaptiveGpuRenderAreaProbeMinRenderAreaPixels,
            selectionMinRenderAreaPixels);
        diagnostics_.adaptiveGpuRenderAreaProbeMaxRenderAreaPixels = std::max(
            diagnostics_.adaptiveGpuRenderAreaProbeMaxRenderAreaPixels,
            selectionMaxRenderAreaPixels);
    }

    if (renderAreaProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuRenderAreaProbePass,
            true);
        if (diagnostics_.adaptiveGpuRenderAreaProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuRenderAreaProbeParityStatus =
                "waiting for previous-frame render-area GPU checksum";
        }
    }

    bool representedCountProbeRecordedAny = false;
    bool representedCountProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuRepresentedCountProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuRepresentedCountProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!representedCountProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuRepresentedCountProbePass,
                false);
            representedCountProbeRecordedAny = true;
        }
        if (!representedCountProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            representedCountProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels = kGpuDiagnosticMinSelectionFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels = kGpuDiagnosticMaxSelectionFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels = kGpuDiagnosticMinSelectionRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels = kGpuDiagnosticMaxSelectionRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticRepresentedCountProbeMin;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticRepresentedCountProbeMax;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuRepresentedCountProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuRepresentedCountProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuRepresentedCountProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuRepresentedCountProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuRepresentedCountProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuRepresentedCountProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuRepresentedCountProbeUsed = true;
        diagnostics_.adaptiveGpuRepresentedCountProbeDispatches += 1U;
        diagnostics_.adaptiveGpuRepresentedCountProbeMinRepresentedSourceCount = std::max(
            diagnostics_.adaptiveGpuRepresentedCountProbeMinRepresentedSourceCount,
            selectionMinRepresentedSourceCount);
        diagnostics_.adaptiveGpuRepresentedCountProbeMaxRepresentedSourceCount = std::max(
            diagnostics_.adaptiveGpuRepresentedCountProbeMaxRepresentedSourceCount,
            selectionMaxRepresentedSourceCount);
    }

    if (representedCountProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuRepresentedCountProbePass,
            true);
        if (diagnostics_.adaptiveGpuRepresentedCountProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuRepresentedCountProbeParityStatus =
                "waiting for previous-frame represented-count GPU checksum";
        }
    }

    bool coverageCompensationProbeRecordedAny = false;
    bool coverageCompensationProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuCoverageCompensationProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!coverageCompensationProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuCoverageCompensationProbePass,
                false);
            coverageCompensationProbeRecordedAny = true;
        }
        if (!coverageCompensationProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            coverageCompensationProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags = kGpuDiagnosticRequiredSelectionFlags;
        constexpr std::uint32_t selectionRejectedFlags = kGpuDiagnosticRejectedSelectionFlags;
        constexpr float selectionMinFootprintAreaPixels = kGpuDiagnosticMinSelectionFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels = kGpuDiagnosticMaxSelectionFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels = kGpuDiagnosticMinSelectionRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels = kGpuDiagnosticMaxSelectionRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation =
            kGpuDiagnosticCoverageCompensationProbeMinOpacity;
        constexpr float selectionMaxOpacityCompensation =
            kGpuDiagnosticCoverageCompensationProbeMaxOpacity;
        constexpr float selectionMinEmissionCompensation =
            kGpuDiagnosticCoverageCompensationProbeMinEmission;
        constexpr float selectionMaxEmissionCompensation =
            kGpuDiagnosticCoverageCompensationProbeMaxEmission;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuCoverageCompensationProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuCoverageCompensationProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuCoverageCompensationProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuCoverageCompensationProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuCoverageCompensationProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuCoverageCompensationProbeUsed = true;
        diagnostics_.adaptiveGpuCoverageCompensationProbeDispatches += 1U;
        diagnostics_.adaptiveGpuCoverageCompensationProbeMinOpacityCompensation = std::max(
            diagnostics_.adaptiveGpuCoverageCompensationProbeMinOpacityCompensation,
            selectionMinOpacityCompensation);
        diagnostics_.adaptiveGpuCoverageCompensationProbeMaxOpacityCompensation = std::max(
            diagnostics_.adaptiveGpuCoverageCompensationProbeMaxOpacityCompensation,
            selectionMaxOpacityCompensation);
        diagnostics_.adaptiveGpuCoverageCompensationProbeMinEmissionCompensation = std::max(
            diagnostics_.adaptiveGpuCoverageCompensationProbeMinEmissionCompensation,
            selectionMinEmissionCompensation);
        diagnostics_.adaptiveGpuCoverageCompensationProbeMaxEmissionCompensation = std::max(
            diagnostics_.adaptiveGpuCoverageCompensationProbeMaxEmissionCompensation,
            selectionMaxEmissionCompensation);
    }

    if (coverageCompensationProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuCoverageCompensationProbePass,
            true);
        if (diagnostics_.adaptiveGpuCoverageCompensationProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuCoverageCompensationProbeParityStatus =
                "waiting for previous-frame coverage-compensation GPU checksum";
        }
    }

    bool clampFlagsProbeRecordedAny = false;
    bool clampFlagsProbePipelineBound = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuCompaction(plan, frameIndex, false) ||
            layer.adaptiveDrawItems == nullptr ||
            layer.adaptiveDrawItems->empty()) {
            continue;
        }

        const auto performanceProfileIndex =
            GpuCompactionPerformanceProfileIndex(layer.adaptiveRendererCostProfile);
        const auto& performanceGate = gpuCompactionPerformanceGates_[performanceProfileIndex];
        if (performanceGate.retryCooldownFrames > 0U ||
            plan.resources->gpuClampFlagsProbeStatsBuffers[frameIndex].buffer == VK_NULL_HANDLE ||
            plan.resources->gpuClampFlagsProbeDescriptorSets[frameIndex] == VK_NULL_HANDLE) {
            continue;
        }

        if (!clampFlagsProbeRecordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuClampFlagsProbePass,
                false);
            clampFlagsProbeRecordedAny = true;
        }
        if (!clampFlagsProbePipelineBound) {
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrawItemCompactionPipeline_);
            clampFlagsProbePipelineBound = true;
        }

        const auto selectionLimit = GpuDiagnosticSelectionLimit(plan.drawPointCount);
        constexpr std::uint32_t selectionClassMask = kGpuDiagnosticSemanticSelectionClassMask;
        const std::uint32_t selectionProfileMask =
            GpuDiagnosticRendererProfileSelectionMask(layer.adaptiveRendererCostProfile);
        constexpr std::uint32_t selectionRankLimit = kGpuDiagnosticRankSelectionLimit;
        constexpr std::uint32_t selectionMinDepth = kGpuDiagnosticMinSelectionDepth;
        constexpr std::uint32_t selectionMaxDepth = kGpuDiagnosticMaxSelectionDepth;
        constexpr std::uint32_t selectionRequiredFlags =
            kGpuDiagnosticClampFlagsProbeRequiredFlags;
        constexpr std::uint32_t selectionRejectedFlags =
            kGpuDiagnosticClampFlagsProbeRejectedFlags;
        constexpr float selectionMinFootprintAreaPixels = kGpuDiagnosticMinSelectionFootprintAreaPixels;
        constexpr float selectionMaxFootprintAreaPixels = kGpuDiagnosticMaxSelectionFootprintAreaPixels;
        constexpr float selectionMinRenderAreaPixels = kGpuDiagnosticMinSelectionRenderAreaPixels;
        constexpr float selectionMaxRenderAreaPixels = kGpuDiagnosticMaxSelectionRenderAreaPixels;
        constexpr float selectionMinOpacityCompensation = kGpuDiagnosticMinSelectionOpacityCompensation;
        constexpr float selectionMaxOpacityCompensation = kGpuDiagnosticMaxSelectionOpacityCompensation;
        constexpr float selectionMinEmissionCompensation = kGpuDiagnosticMinSelectionEmissionCompensation;
        constexpr float selectionMaxEmissionCompensation = kGpuDiagnosticMaxSelectionEmissionCompensation;
        constexpr std::uint32_t selectionMinRepresentedSourceCount =
            kGpuDiagnosticMinSelectionRepresentedSourceCount;
        constexpr std::uint32_t selectionMaxRepresentedSourceCount =
            kGpuDiagnosticMaxSelectionRepresentedSourceCount;
        constexpr float selectionFrustumGuardBand = 0.0F;

        const auto cpuReferenceStart = std::chrono::steady_clock::now();
        const auto expectedStats =
            ComputeGpuCompactionStats(
                *layer.adaptiveDrawItems,
                plan.resources->cpuPositions,
                selectionLimit,
                selectionClassMask,
                selectionProfileMask,
                selectionRankLimit,
                selectionMinDepth,
                selectionMaxDepth,
                selectionRequiredFlags,
                selectionRejectedFlags,
                selectionMinFootprintAreaPixels,
                selectionMaxFootprintAreaPixels,
                selectionMinRenderAreaPixels,
                selectionMaxRenderAreaPixels,
                selectionMinOpacityCompensation,
                selectionMaxOpacityCompensation,
                selectionMinEmissionCompensation,
                selectionMaxEmissionCompensation,
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                renderState_.viewProjection,
                selectionFrustumGuardBand,
                nullptr,
                0U);
        diagnostics_.adaptiveGpuClampFlagsProbeCpuReferenceMs +=
            MillisecondsBetween(cpuReferenceStart, std::chrono::steady_clock::now());

        const GpuDrawItemCompactionStats resetStats{};
        UploadBufferData(
            plan.resources->gpuClampFlagsProbeStatsBuffers[frameIndex],
            &resetStats,
            sizeof(resetStats));
        plan.resources->gpuClampFlagsProbeExpectedStats[frameIndex] = expectedStats;
        plan.resources->gpuClampFlagsProbeResultPending[frameIndex] = true;

        const GpuDrawItemCompactionPushConstants pushConstants{
            glm::uvec4{plan.drawPointCount, selectionLimit, selectionClassMask, selectionRankLimit},
            glm::uvec4{selectionMinDepth, selectionMaxDepth, selectionRequiredFlags, selectionRejectedFlags},
            glm::uvec4{
                FloatBits(selectionMinFootprintAreaPixels),
                FloatBits(selectionMaxFootprintAreaPixels),
                FloatBits(selectionMinRenderAreaPixels),
                FloatBits(selectionMaxRenderAreaPixels)},
            glm::uvec4{
                selectionMinRepresentedSourceCount,
                selectionMaxRepresentedSourceCount,
                0U,
                FloatBits(selectionFrustumGuardBand)},
            glm::uvec4{
                selectionProfileMask,
                0U,
                1U,
                0U},
            glm::uvec4{
                FloatBits(selectionMinOpacityCompensation),
                FloatBits(selectionMaxOpacityCompensation),
                FloatBits(selectionMinEmissionCompensation),
                FloatBits(selectionMaxEmissionCompensation)}};
        VkDescriptorSet descriptorSet = plan.resources->gpuClampFlagsProbeDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuCompactionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuCompactionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrawItemCompactionPushConstants),
            &pushConstants);
        const auto dispatchItemCount = std::min(plan.drawPointCount, selectionLimit);
        vkCmdDispatch(commandBuffer, (dispatchItemCount + 63U) / 64U, 1, 1);

        VkBufferMemoryBarrier probeStatsBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        probeStatsBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        probeStatsBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        probeStatsBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        probeStatsBarrier.buffer = plan.resources->gpuClampFlagsProbeStatsBuffers[frameIndex].buffer;
        probeStatsBarrier.offset = 0;
        probeStatsBarrier.size = sizeof(GpuDrawItemCompactionStats);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0,
            nullptr,
            1,
            &probeStatsBarrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuClampFlagsProbeUsed = true;
        diagnostics_.adaptiveGpuClampFlagsProbeDispatches += 1U;
        diagnostics_.adaptiveGpuClampFlagsProbeRequiredFlags |= selectionRequiredFlags;
        diagnostics_.adaptiveGpuClampFlagsProbeRejectedFlags |= selectionRejectedFlags;
    }

    if (clampFlagsProbeRecordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuClampFlagsProbePass,
            true);
        if (diagnostics_.adaptiveGpuClampFlagsProbeParityStatus == "not checked") {
            diagnostics_.adaptiveGpuClampFlagsProbeParityStatus =
                "waiting for previous-frame clamp-flags GPU checksum";
        }
    }

    return recordedAny ||
           featureProbeRecordedAny ||
           rankProbeRecordedAny ||
           depthProbeRecordedAny ||
           projectedAreaProbeRecordedAny ||
           renderAreaProbeRecordedAny ||
           representedCountProbeRecordedAny ||
           coverageCompensationProbeRecordedAny ||
           clampFlagsProbeRecordedAny;
}

bool VulkanViewportShell::PointCloudPlanUsesGpuIndirectCommand(
    const PointCloudDrawPlan& plan,
    std::size_t frameIndex,
    bool exrStyle) const {
    if (exrStyle ||
        frameIndex >= kFramesInFlight ||
        !gpuDrivenSelectionCapabilities_.computeQueueSupported ||
        !gpuDrivenSelectionCapabilities_.indirectDrawSupported ||
        gpuDrivenIndirectCommandPipeline_ == VK_NULL_HANDLE ||
        gpuDrivenSelectionPipelineLayout_ == VK_NULL_HANDLE ||
        !plan.drawItemReady ||
        plan.drawPointCount < kMinimumAdaptiveIndirectDrawPoints ||
        plan.resources == nullptr) {
        return false;
    }

    return plan.resources->indirectDrawCommandBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
           plan.resources->gpuCompactionStatsBuffers[frameIndex].buffer != VK_NULL_HANDLE &&
           plan.resources->gpuIndirectDescriptorSets[frameIndex] != VK_NULL_HANDLE;
}

bool VulkanViewportShell::RecordGpuDrivenIndirectCommandsForScene(
    VkCommandBuffer commandBuffer,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    bool forceFullSource) {
    if (commandBuffer == VK_NULL_HANDLE ||
        frameIndex >= kFramesInFlight ||
        gpuDrivenIndirectCommandPipeline_ == VK_NULL_HANDLE ||
        gpuDrivenSelectionPipelineLayout_ == VK_NULL_HANDLE) {
        return false;
    }

    bool recordedAny = false;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan) ||
            !PointCloudPlanUsesGpuIndirectCommand(plan, frameIndex, false)) {
            continue;
        }
        if (PointCloudPlanUsesGpuCompactionSubmission(plan, frameIndex, imageIndex, false)) {
            continue;
        }

        if (!recordedAny) {
            WriteGpuTimestamp(
                commandBuffer,
                &frameResources_[frameIndex],
                kGpuTimestampGpuDrivenIndirectCommandPass,
                false);
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_COMPUTE,
                gpuDrivenIndirectCommandPipeline_);
            recordedAny = true;
        }

        const std::uint32_t vertexCount =
            plan.worldSurfels ? plan.drawPointCount * kSurfelVerticesPerPoint : plan.drawPointCount;
        const GpuDrivenIndirectCommandPushConstants pushConstants{
            glm::uvec4{vertexCount, 1U, 0U, kGpuIndirectCommandModeFromPushConstants}};
        VkDescriptorSet descriptorSet = plan.resources->gpuIndirectDescriptorSets[frameIndex];
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gpuDrivenSelectionPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            gpuDrivenSelectionPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0,
            sizeof(GpuDrivenIndirectCommandPushConstants),
            &pushConstants);
        vkCmdDispatch(commandBuffer, 1, 1, 1);

        VkBufferMemoryBarrier barrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.buffer = plan.resources->indirectDrawCommandBuffers[frameIndex].buffer;
        barrier.offset = 0;
        barrier.size = sizeof(VkDrawIndirectCommand);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
            0,
            0,
            nullptr,
            1,
            &barrier,
            0,
            nullptr);

        diagnostics_.adaptiveGpuIndirectCommandUsed = true;
        diagnostics_.adaptiveGpuIndirectCommandDispatches += 1U;
    }

    if (recordedAny) {
        WriteGpuTimestamp(
            commandBuffer,
            &frameResources_[frameIndex],
            kGpuTimestampGpuDrivenIndirectCommandPass,
            true);
        diagnostics_.adaptiveSelectionExecutionPath =
            diagnostics_.adaptiveGpuCompactionUsed
                ? "cpu-selection+gpu-full-range-selection-compare+gpu-generated-indirect"
                : "cpu-selection+gpu-generated-indirect";
        diagnostics_.adaptiveSelectionFallbackReason =
            diagnostics_.adaptiveGpuCompactionUsed
                ? "GPU compute selection remains on CPU fallback until full selection parity/timing are proven; "
                  "CPU-selected representative draw items are full-range semantic-equivalent filtered, workgroup-aggregated, count-compacted, class-counted, checksummed, optionally copied into an ordered output buffer, and converted to a compacted indirect command by compute; the geometry-frustum predicate remains disabled after slower MoltenVK timing; CPU-count indirect command generation remains the fallback when compacted submission gates fail"
                : "GPU compute selection remains on CPU fallback until parity/timing are proven; "
                  "indirect draw commands are generated by a compute dispatch";
        if (!diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason.empty()) {
            diagnostics_.adaptiveSelectionFallbackReason +=
                "; GPU full-range compaction performance fallback: " +
                diagnostics_.adaptiveGpuCompactionPerformanceFallbackReason;
        }
        diagnostics_.adaptiveSelectionParityStatus =
            "CPU-selected draw count drives GPU-generated indirect command";
    }

    return recordedAny;
}

bool VulkanViewportShell::RecordPointCloudLayerDraw(
    VkCommandBuffer commandBuffer,
    const SceneRenderState::PointCloudLayerState& layer,
    bool forceFullSource,
    VkPipeline spritePipeline,
    VkPipeline surfelPipeline,
    bool uploadStyle,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    bool exrStyle,
    std::uint32_t* recordedDrawPointCount) {
    if (recordedDrawPointCount != nullptr) {
        *recordedDrawPointCount = 0;
    }
    PointCloudDrawPlan plan;
    if (!ResolvePointCloudDrawPlan(layer, forceFullSource, &plan)) {
        return false;
    }
    auto* resources = plan.resources;
    if (plan.worldSurfels) {
        if (surfelPipeline == VK_NULL_HANDLE) {
            return false;
        }
    } else if (spritePipeline == VK_NULL_HANDLE) {
        return false;
    }

    if (uploadStyle) {
        if (!UploadPointCloudLayerStyle(layer, plan, frameIndex, exrStyle)) {
            return false;
        }
    }

    vkCmdBindPipeline(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        plan.worldSurfels ? surfelPipeline : spritePipeline);

    const bool useIndirectDraw =
        gpuDrivenSelectionCapabilities_.indirectDrawSupported &&
        plan.drawItemReady &&
        plan.drawPointCount >= kMinimumAdaptiveIndirectDrawPoints &&
        frameIndex < kFramesInFlight &&
        resources->indirectDrawCommandBuffers[frameIndex].buffer != VK_NULL_HANDLE;
    const bool useGpuGeneratedIndirectCommand =
        useIndirectDraw && PointCloudPlanUsesGpuIndirectCommand(plan, frameIndex, exrStyle);
    const bool useGpuCompactedSubmission =
        useIndirectDraw && PointCloudPlanUsesGpuCompactionSubmission(plan, frameIndex, imageIndex, exrStyle);

    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
    if (exrStyle) {
        descriptorSet = resources->exrDescriptorSet;
    } else if (useGpuCompactedSubmission &&
               frameIndex < kFramesInFlight &&
               imageIndex < resources->gpuCompactedDescriptorSets[frameIndex].size()) {
        descriptorSet = resources->gpuCompactedDescriptorSets[frameIndex][imageIndex];
    } else if (frameIndex < kFramesInFlight &&
               imageIndex < resources->descriptorSets[frameIndex].size()) {
        descriptorSet = resources->descriptorSets[frameIndex][imageIndex];
    }
    if (descriptorSet == VK_NULL_HANDLE) {
        return false;
    }

    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pointPipelineLayout_,
        0,
        1,
        &descriptorSet,
        0,
        nullptr);

    if (recordedDrawPointCount != nullptr) {
        *recordedDrawPointCount = plan.drawPointCount;
    }

    if (plan.worldSurfels) {
        const std::uint32_t surfelVertexCount = plan.drawPointCount * kSurfelVerticesPerPoint;
        if (useIndirectDraw) {
            if (!useGpuCompactedSubmission && !useGpuGeneratedIndirectCommand) {
                const VkDrawIndirectCommand command{
                    surfelVertexCount,
                    1U,
                    0U,
                    0U,
                };
                UploadBufferData(
                    resources->indirectDrawCommandBuffers[frameIndex],
                    &command,
                    sizeof(command));
            }
            const VkBuffer indirectCommandBuffer = useGpuCompactedSubmission
                                                       ? resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer
                                                       : resources->indirectDrawCommandBuffers[frameIndex].buffer;
            vkCmdDrawIndirect(
                commandBuffer,
                indirectCommandBuffer,
                0,
                1,
                sizeof(VkDrawIndirectCommand));
            const std::uint32_t submittedVertexCount =
                useGpuCompactedSubmission
                    ? resources->gpuCompactionSubmissionVertexCounts[frameIndex]
                    : surfelVertexCount;
            diagnostics_.adaptiveIndirectDrawUsed = true;
            diagnostics_.adaptiveIndirectDrawCalls += 1U;
            diagnostics_.adaptiveIndirectSubmittedVertices += submittedVertexCount;
            if (useGpuCompactedSubmission) {
                diagnostics_.adaptiveGpuCompactionSubmissionUsed = true;
                diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason.clear();
                diagnostics_.adaptiveSelectionExecutionPath =
                    "cpu-selection+gpu-full-range-selection-compare+gpu-compacted-indirect-submit";
                diagnostics_.adaptiveSelectionFallbackReason =
                    "CPU selection remains authoritative; GPU full-range semantic-equivalent compaction output and compacted indirect command are submitted after previous-frame ordered output parity and current-frame semantic gates passed";
                diagnostics_.adaptiveSelectionParityStatus =
                    "GPU-compacted draw-item output submitted after CPU/GPU parity";
            }
            return true;
        }
        if (plan.sampledBudgetReady && !plan.drawItemReady) {
            vkCmdBindIndexBuffer(
                commandBuffer,
                resources->sampledSurfelIndexBuffer.buffer,
                0,
                VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(commandBuffer, surfelVertexCount, 1, 0, 0, 0);
        } else {
            vkCmdDraw(commandBuffer, surfelVertexCount, 1, 0, 0);
        }
        return true;
    }

    const std::array<VkBuffer, 2> vertexBuffers = {
        resources->positionBuffer.buffer,
        resources->colorBuffer.buffer,
    };
    constexpr std::array<VkDeviceSize, 2> offsets = {0, 0};
    vkCmdBindVertexBuffers(
        commandBuffer,
        0,
        static_cast<std::uint32_t>(vertexBuffers.size()),
        vertexBuffers.data(),
        offsets.data());

    if (plan.sampledBudgetReady && !plan.drawItemReady) {
        vkCmdBindIndexBuffer(
            commandBuffer,
            resources->sampledIndexBuffer.buffer,
            0,
            VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(commandBuffer, plan.drawPointCount, 1, 0, 0, 0);
    } else if (useIndirectDraw) {
        const VkDrawIndirectCommand command{
            plan.drawPointCount,
            1U,
            0U,
            0U,
        };
        if (!useGpuCompactedSubmission && !useGpuGeneratedIndirectCommand) {
            UploadBufferData(
                resources->indirectDrawCommandBuffers[frameIndex],
                &command,
                sizeof(command));
        }
        const VkBuffer indirectCommandBuffer = useGpuCompactedSubmission
                                                   ? resources->gpuCompactionIndirectCommandBuffers[frameIndex].buffer
                                                   : resources->indirectDrawCommandBuffers[frameIndex].buffer;
        vkCmdDrawIndirect(
            commandBuffer,
            indirectCommandBuffer,
            0,
            1,
            sizeof(VkDrawIndirectCommand));
        const std::uint32_t submittedVertexCount =
            useGpuCompactedSubmission
                ? resources->gpuCompactionSubmissionVertexCounts[frameIndex]
                : plan.drawPointCount;
        diagnostics_.adaptiveIndirectDrawUsed = true;
        diagnostics_.adaptiveIndirectDrawCalls += 1U;
        diagnostics_.adaptiveIndirectSubmittedVertices += submittedVertexCount;
        if (useGpuCompactedSubmission) {
            diagnostics_.adaptiveGpuCompactionSubmissionUsed = true;
            diagnostics_.adaptiveGpuCompactionSubmissionFallbackReason.clear();
            diagnostics_.adaptiveSelectionExecutionPath =
                "cpu-selection+gpu-full-range-selection-compare+gpu-compacted-indirect-submit";
            diagnostics_.adaptiveSelectionFallbackReason =
                "CPU selection remains authoritative; GPU full-range semantic-equivalent compaction output and compacted indirect command are submitted after previous-frame ordered output parity and current-frame semantic gates passed";
            diagnostics_.adaptiveSelectionParityStatus =
                "GPU-compacted draw-item output submitted after CPU/GPU parity";
        }
    } else {
        vkCmdDraw(commandBuffer, plan.drawPointCount, 1, 0, 0);
    }
    return true;
}

void VulkanViewportShell::RecordExrExportCommandBuffer(const PointCloudExrFrameRequest& request) {
    auto& resources = exrExportResources_;
    if (resources.commandBuffer == VK_NULL_HANDLE ||
        resources.renderPass == VK_NULL_HANDLE ||
        resources.framebuffer == VK_NULL_HANDLE) {
        throw std::runtime_error{"GPU EXR export resources are not initialized."};
    }

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    Check(vkBeginCommandBuffer(resources.commandBuffer, &beginInfo), "vkBeginCommandBuffer(exr)");

    std::array<VkClearValue, 10> clearValues{};
    clearValues[0].color = {{
        request.renderState.backgroundColor.r,
        request.renderState.backgroundColor.g,
        request.renderState.backgroundColor.b,
        request.renderState.backgroundColor.a,
    }};
    clearValues[1].depthStencil = {1.0F, 0};
    clearValues[2].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[3].color = {{1.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[4].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[5].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[6].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[7].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[8].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[9].color = {{0.0F, 0.0F, 0.0F, 0.0F}};

    const bool fastBasicPointRenderer = renderer::pointcloud::PointCloudRendererModeUsesFastBasic(
        request.renderState.pointCloudRendererMode);
    const bool forceFullSource =
        invisible_places::output::PointCloudExportDensityModeUsesFullSource(request.pointCloudDensityMode) ||
        renderer::pointcloud::PointCloudRendererModeUsesFullSource(request.renderState.pointCloudRendererMode);
    for (const auto& layer : request.renderState.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (ResolvePointCloudDrawPlan(layer, forceFullSource, &plan)) {
            static_cast<void>(UploadPointCloudLayerStyle(layer, plan, 0U, true));
        }
    }

    VkRenderPassBeginInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    renderPassInfo.renderPass = resources.renderPass;
    renderPassInfo.framebuffer = resources.framebuffer;
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = {request.width, request.height};
    renderPassInfo.clearValueCount = static_cast<std::uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    vkCmdBeginRenderPass(resources.commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    const VkViewport viewport{
        0.0F,
        0.0F,
        static_cast<float>(request.width),
        static_cast<float>(request.height),
        0.0F,
        1.0F,
    };
    const VkRect2D scissor{{0, 0}, {request.width, request.height}};
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    const bool sceneHasActiveXray = !fastBasicPointRenderer && std::any_of(
        request.renderState.pointCloudLayers.begin(),
        request.renderState.pointCloudLayers.end(),
        [](const SceneRenderState::PointCloudLayerState& layer) {
            return renderer::pointcloud::PointCloudStyleHasActiveXray(layer.style);
        });
    for (const auto& layer : request.renderState.pointCloudLayers) {
        if (fastBasicPointRenderer) {
            static_cast<void>(RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSource,
                resources.pointFastBasicDepthPipeline,
                VK_NULL_HANDLE,
                false,
                0U,
                0U,
                true));
            continue;
        }
        if (renderer::pointcloud::PointCloudStyleUsesDepthPrepass(layer.style, sceneHasActiveXray) ||
            request.renderState.eyeDomeLightingEnabled) {
            static_cast<void>(RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSource,
                resources.pointDepthPipeline,
                resources.surfelDepthPipeline,
                false,
                0U,
                0U,
                true));
        }
    }

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    if (!fastBasicPointRenderer) {
    for (const auto& layer : request.renderState.pointCloudLayers) {
        VkPipeline spritePipeline = resources.pointAccumulationPipeline;
        VkPipeline surfelPipeline = resources.surfelAccumulationPipeline;
        if (renderer::pointcloud::ResolvePointCloudMaterialVariant(layer.style) ==
            renderer::pointcloud::PointCloudMaterialVariant::ConstantSimple) {
            spritePipeline = resources.pointConstantSimpleAccumulationPipeline;
            surfelPipeline = resources.surfelConstantSimpleAccumulationPipeline;
        }
        static_cast<void>(RecordPointCloudLayerDraw(
            resources.commandBuffer,
            layer,
            forceFullSource,
            spritePipeline,
            surfelPipeline,
            false,
            0U,
            0U,
            true));
    }
    }

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    if (resources.compositeDescriptorSet != VK_NULL_HANDLE) {
        vkCmdBindPipeline(resources.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, resources.compositePipeline);
        vkCmdBindDescriptorSets(
            resources.commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            compositePipelineLayout_,
            0,
            1,
            &resources.compositeDescriptorSet,
            0,
            nullptr);
        vkCmdDraw(resources.commandBuffer, 3, 1, 0, 0);
    }

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    if (fastBasicPointRenderer) {
        for (const auto& layer : request.renderState.pointCloudLayers) {
            static_cast<void>(RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSource,
                resources.pointFastBasicPipeline,
                VK_NULL_HANDLE,
                false,
                0U,
                0U,
                true));
        }
    }

    vkCmdEndRenderPass(resources.commandBuffer);

    VkBufferImageCopy colorCopyRegion{};
    colorCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    colorCopyRegion.imageSubresource.mipLevel = 0;
    colorCopyRegion.imageSubresource.baseArrayLayer = 0;
    colorCopyRegion.imageSubresource.layerCount = 1;
    colorCopyRegion.imageExtent = {request.width, request.height, 1};
    vkCmdCopyImageToBuffer(
        resources.commandBuffer,
        resources.colorImage.image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        resources.colorReadbackBuffer.buffer,
        1,
        &colorCopyRegion);

    VkBufferImageCopy depthCopyRegion{};
    depthCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    depthCopyRegion.imageSubresource.mipLevel = 0;
    depthCopyRegion.imageSubresource.baseArrayLayer = 0;
    depthCopyRegion.imageSubresource.layerCount = 1;
    depthCopyRegion.imageExtent = {request.width, request.height, 1};
    vkCmdCopyImageToBuffer(
        resources.commandBuffer,
        resources.linearDepthImage.image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        resources.depthReadbackBuffer.buffer,
        1,
        &depthCopyRegion);

    VkBufferImageCopy normalCopyRegion{};
    normalCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    normalCopyRegion.imageSubresource.mipLevel = 0;
    normalCopyRegion.imageSubresource.baseArrayLayer = 0;
    normalCopyRegion.imageSubresource.layerCount = 1;
    normalCopyRegion.imageExtent = {request.width, request.height, 1};
    vkCmdCopyImageToBuffer(
        resources.commandBuffer,
        resources.normalImage.image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        resources.normalReadbackBuffer.buffer,
        1,
        &normalCopyRegion);

    VkBufferImageCopy albedoCopyRegion{};
    albedoCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    albedoCopyRegion.imageSubresource.mipLevel = 0;
    albedoCopyRegion.imageSubresource.baseArrayLayer = 0;
    albedoCopyRegion.imageSubresource.layerCount = 1;
    albedoCopyRegion.imageExtent = {request.width, request.height, 1};
    vkCmdCopyImageToBuffer(
        resources.commandBuffer,
        resources.albedoImage.image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        resources.albedoReadbackBuffer.buffer,
        1,
        &albedoCopyRegion);

    Check(vkEndCommandBuffer(resources.commandBuffer), "vkEndCommandBuffer(exr)");
}

void VulkanViewportShell::RecordCommandBuffer(
    VkCommandBuffer commandBuffer,
    std::uint32_t imageIndex,
    std::size_t frameIndex) {
    const bool collectDiagnostics = diagnosticsEnabled_;
    const auto recordStart = collectDiagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    std::uint32_t pointDrawCalls = 0;
    std::uint32_t pointDepthLayerCount = 0;
    std::uint32_t pointAccumulationLayerCount = 0;
    std::uint32_t pointStyleUploadCount = 0;
    std::uint32_t pointSkippedInactiveBindings = 0;
    std::uint32_t pointOpaqueHardDiscDrawCalls = 0;
    std::uint32_t pointConstantSimpleDrawCalls = 0;
    std::uint32_t pointUnifiedDrawCalls = 0;
    std::uint32_t pointFastBasicDrawCalls = 0;
    std::uint64_t pointFastBasicDrawnPoints = 0;
    std::uint64_t pointSubmittedCount = diagnostics_.pointSubmittedCount;
    std::uint64_t pointPassSubmittedCount = diagnostics_.pointPassSubmittedCount;
    std::uint32_t pointDepthPrepassSkippedNoXray = 0;

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    Check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer");
    ResetGpuTimestampQueries(commandBuffer, &frameResources_[frameIndex]);

    const bool drawLiveScene = SceneImageNeedsRender(imageIndex);
    if (drawLiveScene) {
        pointSubmittedCount = 0;
        pointPassSubmittedCount = 0;
    }
    const bool fastBasicPointRenderer = renderer::pointcloud::PointCloudRendererModeUsesFastBasic(
        renderState_.pointCloudRendererMode);
    const bool forceFullSourcePointRenderer = renderer::pointcloud::PointCloudRendererModeUsesFullSource(
        renderState_.pointCloudRendererMode);
    const VkViewport viewport{
        0.0F,
        0.0F,
        static_cast<float>(swapchainWidth_),
        static_cast<float>(swapchainHeight_),
        0.0F,
        1.0F,
    };
    const VkRect2D scissor{{0, 0}, {swapchainWidth_, swapchainHeight_}};

    if (drawLiveScene) {
        ResetGpuCompactionSubmissionFrame(frameIndex);
        for (const auto& layer : renderState_.pointCloudLayers) {
            PointCloudDrawPlan plan;
            if (ResolvePointCloudDrawPlan(layer, forceFullSourcePointRenderer, &plan) &&
                UploadPointCloudLayerStyle(layer, plan, frameIndex, false)) {
                if (collectDiagnostics) {
                    ++pointStyleUploadCount;
                    pointSkippedInactiveBindings += InactivePointBindingCount(layer.style);
                }
            }
        }
        if (collectDiagnostics) {
            static_cast<void>(RecordGpuDrawItemCompactionForScene(
                commandBuffer,
                frameIndex,
                forceFullSourcePointRenderer));
        }
        static_cast<void>(RecordGpuDrivenIndirectCommandsForScene(
            commandBuffer,
            frameIndex,
            imageIndex,
            forceFullSourcePointRenderer));
    }

    if (drawLiveScene) {
    std::array<VkClearValue, 6> clearValues{};
    clearValues[0].color = {
        {
            renderState_.backgroundColor.r,
            renderState_.backgroundColor.g,
            renderState_.backgroundColor.b,
            renderState_.backgroundColor.a,
        }};
    clearValues[1].depthStencil = {1.0F, 0};
    clearValues[2].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[3].color = {{1.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[4].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[5].color = {{0.0F, 0.0F, 0.0F, 0.0F}};

    VkRenderPassBeginInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    renderPassInfo.renderPass = renderPass_;
    renderPassInfo.framebuffer = framebuffers_[imageIndex];
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = {swapchainWidth_, swapchainHeight_};
    renderPassInfo.clearValueCount = static_cast<std::uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    const bool sceneHasActiveXray =
        drawLiveScene &&
        std::any_of(
            renderState_.pointCloudLayers.begin(),
            renderState_.pointCloudLayers.end(),
            [](const SceneRenderState::PointCloudLayerState& layer) {
                return renderer::pointcloud::PointCloudStyleHasActiveXray(layer.style);
            });
    if (drawLiveScene && !fastBasicPointRenderer && !renderState_.pointCloudLayers.empty()) {
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampBeautyDepthPass, false);
        for (const auto& layer : renderState_.pointCloudLayers) {
            const auto materialVariant = renderer::pointcloud::ResolvePointCloudMaterialVariant(layer.style);
            const bool opaqueHardDisc =
                materialVariant == renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc;
            if (collectDiagnostics &&
                !sceneHasActiveXray &&
                renderer::pointcloud::PointCloudStyleUsesDepthPrepass(layer.style) &&
                !opaqueHardDisc) {
                ++pointDepthPrepassSkippedNoXray;
            }
            if (opaqueHardDisc ||
                renderer::pointcloud::PointCloudStyleUsesDepthPrepass(layer.style, sceneHasActiveXray) ||
                renderState_.eyeDomeLightingEnabled) {
                if (collectDiagnostics) {
                    ++pointDepthLayerCount;
                }
                std::uint32_t recordedDrawPointCount = 0;
                if (RecordPointCloudLayerDraw(
                    commandBuffer,
                    layer,
                    forceFullSourcePointRenderer,
                    pointDepthPrepassPipeline_,
                    surfelDepthPrepassPipeline_,
                    false,
                    frameIndex,
                    imageIndex,
                    false,
                    &recordedDrawPointCount)) {
                    if (collectDiagnostics) {
                        ++pointDrawCalls;
                        pointPassSubmittedCount += recordedDrawPointCount;
                    }
                }
            }
        }
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampBeautyDepthPass, true);
    }

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (drawLiveScene && !fastBasicPointRenderer && !renderState_.pointCloudLayers.empty()) {
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampBeautyPointPass, false);
        for (const auto& layer : renderState_.pointCloudLayers) {
            const auto materialVariant = renderer::pointcloud::ResolvePointCloudMaterialVariant(layer.style);
            if (materialVariant == renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc) {
                continue;
            }
            VkPipeline spritePipeline = pointAccumulationPipeline_;
            VkPipeline surfelPipeline = surfelAccumulationPipeline_;
            if (materialVariant == renderer::pointcloud::PointCloudMaterialVariant::ConstantSimple) {
                spritePipeline = pointConstantSimpleAccumulationPipeline_;
                surfelPipeline = surfelConstantSimpleAccumulationPipeline_;
            }
            if (collectDiagnostics) {
                ++pointAccumulationLayerCount;
            }
            std::uint32_t recordedDrawPointCount = 0;
            if (RecordPointCloudLayerDraw(
                commandBuffer,
                layer,
                forceFullSourcePointRenderer,
                spritePipeline,
                surfelPipeline,
                false,
                frameIndex,
                imageIndex,
                false,
                &recordedDrawPointCount)) {
                if (collectDiagnostics) {
                    ++pointDrawCalls;
                    pointSubmittedCount += recordedDrawPointCount;
                    pointPassSubmittedCount += recordedDrawPointCount;
                    if (materialVariant == renderer::pointcloud::PointCloudMaterialVariant::ConstantSimple) {
                        ++pointConstantSimpleDrawCalls;
                    } else {
                        ++pointUnifiedDrawCalls;
                    }
                }
            }
        }
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampBeautyPointPass, true);
    }

    if (drawLiveScene && !renderState_.gaussianSplatLayers.empty()) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, gaussianSplatPipeline_);

        for (const auto& layer : renderState_.gaussianSplatLayers) {
            if (layer.style.qualityMode == renderer::gsplat::GaussianSplatQualityMode::High) {
                continue;
            }

            const auto* resources = FindGaussianSplatResources(layer.layerId);
            if (resources == nullptr ||
                resources->splatCount == 0 ||
                frameIndex >= kFramesInFlight ||
                imageIndex >= resources->descriptorSets[frameIndex].size()) {
                continue;
            }
            VkDescriptorSet descriptorSet = resources->descriptorSets[frameIndex][imageIndex];
            if (descriptorSet == VK_NULL_HANDLE) {
                continue;
            }

            std::uint32_t weightedQualityMode = 0U;
            if (layer.style.qualityMode == renderer::gsplat::GaussianSplatQualityMode::Fast) {
                weightedQualityMode = 1U;
            } else if (layer.style.qualityMode == renderer::gsplat::GaussianSplatQualityMode::SurfaceGuided) {
                weightedQualityMode = 2U;
            }

            GaussianSplatPushConstants pushConstants;
            pushConstants.localToWorld = layer.localToWorld;
            pushConstants.layerTint = glm::vec4{
                layer.style.layerTint[0],
                layer.style.layerTint[1],
                layer.style.layerTint[2],
                layer.style.layerTint[3],
            };
            pushConstants.style = glm::vec4{
                layer.style.opacityMultiplier,
                layer.style.scaleMultiplier,
                layer.style.exposure,
                layer.style.saturation,
            };
            pushConstants.control = glm::uvec4{
                static_cast<std::uint32_t>(layer.style.colorMode),
                static_cast<std::uint32_t>(layer.style.debugMode),
                layer.style.transformEnabled ? 1U : 0U,
                weightedQualityMode,
            };
            pushConstants.extra = glm::vec4{
                renderState_.gaussianSplatFootprintBoost,
                0.0F,
                0.0F,
                0.0F,
            };

            vkCmdBindDescriptorSets(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                gaussianSplatPipelineLayout_,
                0,
                1,
                &descriptorSet,
                0,
                nullptr);
            vkCmdPushConstants(
                commandBuffer,
                gaussianSplatPipelineLayout_,
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                0,
                sizeof(GaussianSplatPushConstants),
                &pushConstants);
            vkCmdDraw(commandBuffer, 6, resources->splatCount, 0, 0);
        }
    }

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (drawLiveScene &&
        imageIndex < compositeDescriptorSets_.size() &&
        compositeDescriptorSets_[imageIndex] != VK_NULL_HANDLE) {
        VkDescriptorSet descriptorSet = compositeDescriptorSets_[imageIndex];
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampCompositePass, false);
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            compositePipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampCompositePass, true);
    }

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (drawLiveScene && fastBasicPointRenderer && !renderState_.pointCloudLayers.empty()) {
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampFastBasicPointPass, false);
        for (const auto& layer : renderState_.pointCloudLayers) {
            std::uint32_t recordedDrawPointCount = 0;
            if (RecordPointCloudLayerDraw(
                commandBuffer,
                layer,
                false,
                pointFastBasicPipeline_,
                VK_NULL_HANDLE,
                false,
                frameIndex,
                imageIndex,
                false,
                &recordedDrawPointCount)) {
                if (collectDiagnostics) {
                    ++pointDrawCalls;
                    ++pointFastBasicDrawCalls;
                    pointFastBasicDrawnPoints += recordedDrawPointCount;
                    pointSubmittedCount += recordedDrawPointCount;
                    pointPassSubmittedCount += recordedDrawPointCount;
                }
            }
        }
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampFastBasicPointPass, true);
    } else if (drawLiveScene && !renderState_.pointCloudLayers.empty()) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            const auto materialVariant = renderer::pointcloud::ResolvePointCloudMaterialVariant(layer.style);
            if (materialVariant != renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc) {
                continue;
            }
            std::uint32_t recordedDrawPointCount = 0;
            if (RecordPointCloudLayerDraw(
                commandBuffer,
                layer,
                forceFullSourcePointRenderer,
                pointOpaqueHardDiscPipeline_,
                surfelOpaqueHardDiscPipeline_,
                false,
                frameIndex,
                imageIndex,
                false,
                &recordedDrawPointCount)) {
                if (collectDiagnostics) {
                    ++pointDrawCalls;
                    ++pointOpaqueHardDiscDrawCalls;
                    pointSubmittedCount += recordedDrawPointCount;
                    pointPassSubmittedCount += recordedDrawPointCount;
                }
            }
        }
    }

    if (drawLiveScene &&
        frameIndex < kFramesInFlight &&
        highQualityGaussianScene_.descriptorSets[frameIndex] != VK_NULL_HANDLE &&
        highQualityGaussianScene_.splatCount > 0) {
        VkDescriptorSet descriptorSet = highQualityGaussianScene_.descriptorSets[frameIndex];
        HighQualityGaussianPushConstants pushConstants;
        pushConstants.extra = glm::vec4{
            renderState_.gaussianSplatFootprintBoost,
            0.0F,
            0.0F,
            0.0F,
        };

        vkCmdBindPipeline(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            highQualityGaussianSplatPipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            highQualityGaussianSplatPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            highQualityGaussianSplatPipelineLayout_,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0,
            sizeof(HighQualityGaussianPushConstants),
            &pushConstants);
        vkCmdDraw(commandBuffer, 6, highQualityGaussianScene_.splatCount, 0, 0);
    }

    vkCmdEndRenderPass(commandBuffer);
        if (imageIndex < sceneImageRevisions_.size()) {
            sceneImageRevisions_[imageIndex] = sceneRevision_;
        }
    }

    const bool presentScene =
        liveSceneRenderingEnabled_ &&
        imageIndex < sceneImageRevisions_.size() &&
        sceneImageRevisions_[imageIndex] == sceneRevision_;

    std::array<VkClearValue, 1> presentClearValues{};
    presentClearValues[0].color = {
        {
            renderState_.backgroundColor.r,
            renderState_.backgroundColor.g,
            renderState_.backgroundColor.b,
            renderState_.backgroundColor.a,
        }};

    VkRenderPassBeginInfo presentPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    presentPassInfo.renderPass = presentRenderPass_;
    presentPassInfo.framebuffer = presentFramebuffers_[imageIndex];
    presentPassInfo.renderArea.offset = {0, 0};
    presentPassInfo.renderArea.extent = {swapchainWidth_, swapchainHeight_};
    presentPassInfo.clearValueCount = static_cast<std::uint32_t>(presentClearValues.size());
    presentPassInfo.pClearValues = presentClearValues.data();

    vkCmdBeginRenderPass(commandBuffer, &presentPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (presentScene &&
        imageIndex < postProcessDescriptorSets_.size() &&
        postProcessDescriptorSets_[imageIndex] != VK_NULL_HANDLE) {
        VkDescriptorSet descriptorSet = postProcessDescriptorSets_[imageIndex];
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampPostProcessPass, false);
        PostProcessPushConstants pushConstants;
        pushConstants.edl = glm::vec4{
            renderState_.eyeDomeLightingEnabled ? 1.0F : 0.0F,
            24.0F,
            0.35F,
            std::clamp(renderState_.eyeDomeLightingThickness, 1.0F, 24.0F),
        };
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, postProcessPipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            postProcessPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            postProcessPipelineLayout_,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            0,
            sizeof(PostProcessPushConstants),
            &pushConstants);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
        WriteGpuTimestamp(commandBuffer, &frameResources_[frameIndex], kGpuTimestampPostProcessPass, true);
    }

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
    }

    vkCmdEndRenderPass(commandBuffer);
    Check(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer");

    const auto recordEnd = collectDiagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    diagnostics_.pointDrawCalls = pointDrawCalls;
    diagnostics_.pointDepthLayerCount = pointDepthLayerCount;
    diagnostics_.pointAccumulationLayerCount = pointAccumulationLayerCount;
    diagnostics_.pointStyleUploadCount = pointStyleUploadCount;
    diagnostics_.pointSkippedInactiveBindings = pointSkippedInactiveBindings;
    diagnostics_.pointOpaqueHardDiscDrawCalls = pointOpaqueHardDiscDrawCalls;
    diagnostics_.pointConstantSimpleDrawCalls = pointConstantSimpleDrawCalls;
    diagnostics_.pointUnifiedDrawCalls = pointUnifiedDrawCalls;
    diagnostics_.pointFastBasicDrawCalls = pointFastBasicDrawCalls;
    diagnostics_.pointFastBasicDrawnPoints = pointFastBasicDrawnPoints;
    diagnostics_.pointSubmittedCount = pointSubmittedCount;
    diagnostics_.pointPassSubmittedCount = pointPassSubmittedCount;
    diagnostics_.pointDepthPrepassSkippedNoXray = pointDepthPrepassSkippedNoXray;
    diagnostics_.sceneRenderedThisFrame = drawLiveScene;
    diagnostics_.sceneCacheActive = sceneCachingEnabled_;
    diagnostics_.pointCommandRecordMs =
        collectDiagnostics ? std::chrono::duration<double, std::milli>(recordEnd - recordStart).count() : 0.0;
}

void VulkanViewportShell::UpdateUniformBuffer(std::size_t frameIndex) {
    UploadFrameUniforms(frameIndex, swapchainWidth_, swapchainHeight_);
}

void VulkanViewportShell::UploadFrameUniforms(
    std::size_t frameIndex,
    std::uint32_t width,
    std::uint32_t height) {
    if (frameIndex >= kFramesInFlight ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    FrameUniforms uniforms;
    uniforms.viewProjection = renderState_.viewProjection;
    uniforms.view = renderState_.view;
    uniforms.projection = renderState_.projection;
    uniforms.cameraPosition = glm::vec4{renderState_.cameraPosition, 0.0F};
    uniforms.depthParameters = glm::vec4{
        std::max(0.0F, renderState_.flowTimeSeconds),
        renderState_.nearPlane,
        renderState_.farPlane,
        std::max(0.001F, renderState_.pointSizeScale),
    };
    const float viewportWidth = std::max(1.0F, static_cast<float>(width));
    const float viewportHeight = std::max(1.0F, static_cast<float>(height));
    uniforms.viewportParameters = glm::vec4{
        viewportWidth,
        viewportHeight,
        2.0F / viewportWidth,
        2.0F / viewportHeight,
    };
    uniforms.depthOfFieldParameters = glm::vec4{
        renderState_.hasDepthOfField ? 1.0F : 0.0F,
        std::max(0.001F, renderState_.focusDistance),
        std::max(0.1F, renderState_.apertureFStops),
        std::max(0.0F, renderState_.depthOfFieldMaxBlurPixels),
    };
    uniforms.inverseViewProjection = glm::inverse(renderState_.viewProjection);

    UploadBufferData(frameResources_[frameIndex].uniformBuffer, &uniforms, sizeof(uniforms));
}

VulkanViewportShell::BufferAllocation VulkanViewportShell::CreateHostVisibleBuffer(
    VkDeviceSize size,
    VkBufferUsageFlags usage) const {
    if (size == 0) {
        size = 4;
    }

    BufferAllocation allocation;
    allocation.size = size;

    VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    Check(vkCreateBuffer(device_, &bufferInfo, nullptr, &allocation.buffer), "vkCreateBuffer");

    VkMemoryRequirements memoryRequirements{};
    vkGetBufferMemoryRequirements(device_, allocation.buffer, &memoryRequirements);

    VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocInfo.allocationSize = memoryRequirements.size;
    allocInfo.memoryTypeIndex = FindMemoryType(
        memoryRequirements.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    Check(vkAllocateMemory(device_, &allocInfo, nullptr, &allocation.memory), "vkAllocateMemory");
    Check(vkBindBufferMemory(device_, allocation.buffer, allocation.memory, 0), "vkBindBufferMemory");
    Check(
        vkMapMemory(device_, allocation.memory, 0, allocation.size, 0, &allocation.mapped),
        "vkMapMemory(persistent buffer)");
    return allocation;
}

void VulkanViewportShell::UploadBufferData(
    const BufferAllocation& buffer,
    const void* data,
    VkDeviceSize size) const {
    if (buffer.memory == VK_NULL_HANDLE || data == nullptr || size == 0) {
        return;
    }

    if (buffer.mapped != nullptr) {
        std::memcpy(buffer.mapped, data, static_cast<std::size_t>(size));
        return;
    }

    void* mapped = nullptr;
    Check(vkMapMemory(device_, buffer.memory, 0, size, 0, &mapped), "vkMapMemory");
    std::memcpy(mapped, data, static_cast<std::size_t>(size));
    vkUnmapMemory(device_, buffer.memory);
}

void VulkanViewportShell::DestroyBuffer(BufferAllocation* buffer) {
    if (buffer == nullptr) {
        return;
    }

    if (buffer->memory != VK_NULL_HANDLE) {
        if (buffer->mapped != nullptr) {
            vkUnmapMemory(device_, buffer->memory);
            buffer->mapped = nullptr;
        }
    }
    if (buffer->buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device_, buffer->buffer, nullptr);
    }
    if (buffer->memory != VK_NULL_HANDLE) {
        vkFreeMemory(device_, buffer->memory, nullptr);
    }

    *buffer = BufferAllocation{};
}

void VulkanViewportShell::DestroyImage(ImageAllocation* image) {
    if (image == nullptr) {
        return;
    }

    if (image->view != VK_NULL_HANDLE) {
        vkDestroyImageView(device_, image->view, nullptr);
    }
    if (image->image != VK_NULL_HANDLE) {
        vkDestroyImage(device_, image->image, nullptr);
    }
    if (image->memory != VK_NULL_HANDLE) {
        vkFreeMemory(device_, image->memory, nullptr);
    }

    *image = ImageAllocation{};
}

std::uint32_t VulkanViewportShell::FindMemoryType(
    std::uint32_t typeFilter,
    VkMemoryPropertyFlags requiredFlags,
    VkMemoryPropertyFlags preferredFlags) const {
    VkPhysicalDeviceMemoryProperties memoryProperties{};
    vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memoryProperties);

    std::optional<std::uint32_t> fallback;
    for (std::uint32_t index = 0; index < memoryProperties.memoryTypeCount; ++index) {
        const bool supported = (typeFilter & (1U << index)) != 0;
        const auto propertyFlags = memoryProperties.memoryTypes[index].propertyFlags;
        if (!supported || (propertyFlags & requiredFlags) != requiredFlags) {
            continue;
        }

        if ((propertyFlags & preferredFlags) == preferredFlags) {
            return index;
        }

        if (!fallback.has_value()) {
            fallback = index;
        }
    }

    if (!fallback.has_value()) {
        throw std::runtime_error{"Failed to find a compatible Vulkan memory type."};
    }

    return fallback.value();
}

VkFormat VulkanViewportShell::SelectDepthFormat() const {
    constexpr std::array<VkFormat, 3> candidates = {
        VK_FORMAT_D32_SFLOAT,
        VK_FORMAT_D32_SFLOAT_S8_UINT,
        VK_FORMAT_D24_UNORM_S8_UINT,
    };

    for (const auto format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physicalDevice_, format, &properties);
        if ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0) {
            return format;
        }
    }

    throw std::runtime_error{"No compatible Vulkan depth format is available."};
}

VkFormat VulkanViewportShell::SelectAccumulationFormat() const {
    constexpr std::array<VkFormat, 2> candidates = {
        VK_FORMAT_R16G16B16A16_SFLOAT,
        VK_FORMAT_R32G32B32A32_SFLOAT,
    };

    for (const auto format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physicalDevice_, format, &properties);
        if ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT) != 0) {
            return format;
        }
    }

    throw std::runtime_error{"No compatible accumulation color format is available."};
}

VkFormat VulkanViewportShell::SelectRevealageFormat() const {
    constexpr std::array<VkFormat, 2> candidates = {
        VK_FORMAT_R16_SFLOAT,
        VK_FORMAT_R32_SFLOAT,
    };

    for (const auto format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physicalDevice_, format, &properties);
        if ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT) != 0) {
            return format;
        }
    }

    throw std::runtime_error{"No compatible revealage color format is available."};
}

std::vector<char> VulkanViewportShell::ReadBinaryFile(const std::string& filePath) const {
    std::ifstream input{filePath, std::ios::ate | std::ios::binary};
    if (!input.is_open()) {
        throw std::runtime_error{"Failed to open shader file: " + filePath};
    }

    const auto fileSize = static_cast<std::size_t>(input.tellg());
    std::vector<char> bytes(fileSize);
    input.seekg(0, std::ios::beg);
    input.read(bytes.data(), static_cast<std::streamsize>(fileSize));
    if (!input.good() && !input.eof()) {
        throw std::runtime_error{"Failed to read shader file: " + filePath};
    }

    return bytes;
}

VulkanViewportShell::ImageAllocation VulkanViewportShell::CreateAttachmentImage(
    VkFormat format,
    VkImageUsageFlags usage,
    VkImageAspectFlags aspectFlags) const {
    return CreateAttachmentImage(swapchainWidth_, swapchainHeight_, format, usage, aspectFlags);
}

VulkanViewportShell::ImageAllocation VulkanViewportShell::CreateAttachmentImage(
    std::uint32_t width,
    std::uint32_t height,
    VkFormat format,
    VkImageUsageFlags usage,
    VkImageAspectFlags aspectFlags) const {
    ImageAllocation image;
    image.format = format;

    VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = width;
    imageInfo.extent.height = height;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = format;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = usage;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    Check(vkCreateImage(device_, &imageInfo, nullptr, &image.image), "vkCreateImage(attachment)");

    VkMemoryRequirements memoryRequirements{};
    vkGetImageMemoryRequirements(device_, image.image, &memoryRequirements);

    VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocInfo.allocationSize = memoryRequirements.size;
    allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, 0, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    Check(vkAllocateMemory(device_, &allocInfo, nullptr, &image.memory), "vkAllocateMemory(attachment)");
    Check(vkBindImageMemory(device_, image.image, image.memory, 0), "vkBindImageMemory(attachment)");

    VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    viewInfo.image = image.image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = format;
    viewInfo.subresourceRange.aspectMask = aspectFlags;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    Check(vkCreateImageView(device_, &viewInfo, nullptr, &image.view), "vkCreateImageView(attachment)");
    return image;
}

VulkanViewportShell::ActivePointCloudResources* VulkanViewportShell::FindPointCloudResources(std::size_t layerId) {
    auto resourcesIt = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != pointCloudResources_.end() ? &(*resourcesIt) : nullptr;
}

const VulkanViewportShell::ActivePointCloudResources* VulkanViewportShell::FindPointCloudResources(
    std::size_t layerId) const {
    auto resourcesIt = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != pointCloudResources_.end() ? &(*resourcesIt) : nullptr;
}

VulkanViewportShell::ActiveGaussianSplatResources* VulkanViewportShell::FindGaussianSplatResources(
    std::size_t layerId) {
    auto resourcesIt = std::find_if(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [layerId](const ActiveGaussianSplatResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != gaussianSplatResources_.end() ? &(*resourcesIt) : nullptr;
}

const VulkanViewportShell::ActiveGaussianSplatResources* VulkanViewportShell::FindGaussianSplatResources(
    std::size_t layerId) const {
    auto resourcesIt = std::find_if(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [layerId](const ActiveGaussianSplatResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != gaussianSplatResources_.end() ? &(*resourcesIt) : nullptr;
}

void VulkanViewportShell::FramebufferResizeCallback(GLFWwindow* window, int /*width*/, int /*height*/) {
    auto* shell = static_cast<VulkanViewportShell*>(glfwGetWindowUserPointer(window));
    if (shell != nullptr) {
        shell->framebufferResized_ = true;
    }
}

}  // namespace invisible_places::renderer::core
