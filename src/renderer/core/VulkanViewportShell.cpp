#include "renderer/core/VulkanViewportShell.hpp"

#include "InvisiblePlacesBuildConfig.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <glm/mat4x4.hpp>
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

struct alignas(16) FrameUniforms {
    glm::mat4 viewProjection{1.0F};
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::vec4 cameraPosition{0.0F, 0.0F, 1.0F, 0.0F};
    glm::vec4 depthParameters{0.0F, 0.05F, 1000.0F, 0.0F};
    glm::vec4 viewportParameters{1.0F, 1.0F, 2.0F, 2.0F};
};

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
    glm::uvec4 padding{0U, 0U, 0U, 0U};
    PointCloudBindingGpu pointSize{};
    PointCloudBindingGpu opacity{};
    PointCloudBindingGpu emissive{};
    PointCloudBindingGpu xray{};
    PointCloudBindingGpu depthFade{};
    PointCloudBindingGpu colormapPosition{};
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
        if ((queueFamilies[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
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

PointCloudBindingGpu MakePointCloudBindingGpu(const invisible_places::style::RenderParameterBinding& binding) {
    PointCloudBindingGpu gpuBinding;
    gpuBinding.constantValue = glm::vec4{
        binding.constantValue[0],
        binding.constantValue[1],
        binding.constantValue[2],
        binding.constantValue[3],
    };
    gpuBinding.range = glm::vec4{
        binding.fieldMap.inputMin,
        binding.fieldMap.inputMax,
        binding.fieldMap.outputMin,
        binding.fieldMap.outputMax,
    };
    gpuBinding.extra = glm::vec4{binding.fieldMap.gamma, 0.0F, 0.0F, 0.0F};
    gpuBinding.control = glm::uvec4{
        static_cast<std::uint32_t>(binding.mode),
        binding.fieldMap.fieldSlot >= 0 ? static_cast<std::uint32_t>(binding.fieldMap.fieldSlot) : 0xFFFFFFFFU,
        binding.fieldMap.flags,
        0U,
    };
    return gpuBinding;
}

VkDescriptorPoolSize MakePoolSize(VkDescriptorType type, std::uint32_t descriptorCount) {
    return VkDescriptorPoolSize{type, descriptorCount};
}

VkPipelineColorBlendAttachmentState MakeDisabledBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.blendEnable = VK_FALSE;
    attachment.colorWriteMask = 0;
    return attachment;
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
    CreatePointDescriptorSetLayout();
    CreateGaussianSplatDescriptorSetLayout();
    CreateHighQualityGaussianSplatDescriptorSetLayout();
    CreateCompositeDescriptorSetLayout();
    CreateDescriptorPools();
    CreateUniformResources();
    CreateDepthResources();
    CreateAccumulationResources();
    CreatePointPipeline();
    CreateGaussianSplatPipeline();
    CreateHighQualityGaussianSplatPipeline();
    CreateCompositePipeline();
    CreateFramebuffers();
    CreateCommandPool();
    CreateCommandBuffers();
    CreateSyncObjects();
    CreateOrUpdateCompositeDescriptorSet();
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

    DestroyBuffer(&uniformBuffer_);

    if (imageAvailableSemaphore_ != VK_NULL_HANDLE) {
        vkDestroySemaphore(device_, imageAvailableSemaphore_, nullptr);
    }
    if (renderFinishedSemaphore_ != VK_NULL_HANDLE) {
        vkDestroySemaphore(device_, renderFinishedSemaphore_, nullptr);
    }
    if (inFlightFence_ != VK_NULL_HANDLE) {
        vkDestroyFence(device_, inFlightFence_, nullptr);
    }

    if (commandPool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, commandPool_, nullptr);
    }

    if (pointPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointPipeline_, nullptr);
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

    CleanupSwapchain();

    if (renderPass_ != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, renderPass_, nullptr);
    }

    if (device_ != VK_NULL_HANDLE) {
        vkDestroyDevice(device_, nullptr);
    }
    if (surface_ != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(instance_, surface_, nullptr);
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
    }
}

void VulkanViewportShell::BeginUiFrame() {
    if (ImGui::GetCurrentContext() == nullptr || uiFrameBegun_) {
        return;
    }

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    uiFrameBegun_ = true;
}

void VulkanViewportShell::DrawFrame() {
    if (uiFrameBegun_) {
        ImGui::Render();
        uiFrameBegun_ = false;
    }

    vkWaitForFences(device_, 1, &inFlightFence_, VK_TRUE, UINT64_MAX);
    RefreshHighQualityGaussianScene();
    UpdateUniformBuffer();

    std::uint32_t imageIndex = 0;
    const VkResult acquireResult =
        vkAcquireNextImageKHR(device_, swapchain_, UINT64_MAX, imageAvailableSemaphore_, VK_NULL_HANDLE, &imageIndex);

    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
        RecreateSwapchain();
        return;
    }
    if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
        Check(acquireResult, "vkAcquireNextImageKHR");
    }

    Check(vkResetFences(device_, 1, &inFlightFence_), "vkResetFences");
    Check(vkResetCommandBuffer(commandBuffers_[imageIndex], 0), "vkResetCommandBuffer");
    RecordCommandBuffer(commandBuffers_[imageIndex], imageIndex);

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = &imageAvailableSemaphore_;
    submitInfo.pWaitDstStageMask = &waitStage;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffers_[imageIndex];
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = &renderFinishedSemaphore_;

    Check(vkQueueSubmit(graphicsQueue_, 1, &submitInfo, inFlightFence_), "vkQueueSubmit");

    VkPresentInfoKHR presentInfo{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = &renderFinishedSemaphore_;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain_;
    presentInfo.pImageIndices = &imageIndex;

    const VkResult presentResult = vkQueuePresentKHR(presentQueue_, &presentInfo);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR || framebufferResized_) {
        framebufferResized_ = false;
        RecreateSwapchain();
        return;
    }

    Check(presentResult, "vkQueuePresentKHR");
}

void VulkanViewportShell::WaitIdle() const {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }
}

void VulkanViewportShell::UpdateRenderState(const SceneRenderState& state) {
    renderState_ = state;
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

    resources.positionBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(cloud.positions.size() * sizeof(invisible_places::io::Float3)),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    UploadBufferData(
        resources.positionBuffer,
        cloud.positions.data(),
        resources.positionBuffer.size);

    resources.colorBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(cloud.packedColors.size() * sizeof(std::uint32_t)),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    UploadBufferData(
        resources.colorBuffer,
        cloud.packedColors.data(),
        resources.colorBuffer.size);

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

    resources.styleBuffer = CreateHostVisibleBuffer(
        sizeof(PointCloudStyleGpu),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    UpdatePointCloudDescriptorSet(&resources);

    UpdatePointBudget(layerId, sampledIndices);
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

    UpdateGaussianSplatDescriptorSet(&resources);
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
        diagnostics_.rendererName = properties.deviceName;
        diagnostics_.driverName = "Vulkan";
        break;
    }

    if (physicalDevice_ == VK_NULL_HANDLE) {
        throw std::runtime_error{"No suitable Vulkan device was found for presentation."};
    }

    const auto selection = FindQueueFamilies(physicalDevice_, surface_);
    graphicsQueueFamily_ = selection.graphicsFamily.value();
    presentQueueFamily_ = selection.presentFamily.value();
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

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = swapchainImageFormat_;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

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

    VkAttachmentReference subpass0ColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

    VkAttachmentReference depthAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthReadOnlyAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass0{};
    subpass0.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass0.colorAttachmentCount = 1;
    subpass0.pColorAttachments = &subpass0ColorRef;
    subpass0.pDepthStencilAttachment = &depthAttachmentRef;

    VkAttachmentReference subpass1ColorRefs[2]{};
    subpass1ColorRefs[0] = {2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    subpass1ColorRefs[1] = {3, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass1InputRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass1{};
    subpass1.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass1.colorAttachmentCount = 2;
    subpass1.pColorAttachments = subpass1ColorRefs;
    subpass1.inputAttachmentCount = 1;
    subpass1.pInputAttachments = &subpass1InputRef;
    subpass1.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    VkAttachmentReference subpass2ColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass2InputRefs[2]{};
    subpass2InputRefs[0] = {2, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    subpass2InputRefs[1] = {3, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass2{};
    subpass2.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass2.colorAttachmentCount = 1;
    subpass2.pColorAttachments = &subpass2ColorRef;
    subpass2.inputAttachmentCount = 2;
    subpass2.pInputAttachments = subpass2InputRefs;

    VkAttachmentReference subpass3ColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass3{};
    subpass3.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass3.colorAttachmentCount = 1;
    subpass3.pColorAttachments = &subpass3ColorRef;
    subpass3.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    std::array<VkSubpassDependency, 6> dependencies{};
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
    dependencies[2].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[3].srcSubpass = 1;
    dependencies[3].dstSubpass = 2;
    dependencies[3].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[3].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

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
    dependencies[5].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    dependencies[5].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    const std::array<VkAttachmentDescription, 4> attachments = {
        colorAttachment,
        depthAttachment,
        accumulationAttachment,
        revealageAttachment,
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

void VulkanViewportShell::CreatePointDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 3> bindings{};
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
    std::array<VkDescriptorSetLayoutBinding, 2> bindings{};
    bindings[0] = {0, VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};
    bindings[1] = {1, VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &compositeDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(composite)");
}

void VulkanViewportShell::CreateDescriptorPools() {
    const std::array<VkDescriptorPoolSize, 3> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 130),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 64),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 2),
    };

    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 66;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    Check(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_), "vkCreateDescriptorPool");

    const std::array<VkDescriptorPoolSize, 3> gsplatPoolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 65),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, (64 * 5) + 8),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 65),
    };

    VkDescriptorPoolCreateInfo gsplatPoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    gsplatPoolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    gsplatPoolInfo.maxSets = 65;
    gsplatPoolInfo.poolSizeCount = static_cast<std::uint32_t>(gsplatPoolSizes.size());
    gsplatPoolInfo.pPoolSizes = gsplatPoolSizes.data();
    Check(
        vkCreateDescriptorPool(device_, &gsplatPoolInfo, nullptr, &gaussianSplatDescriptorPool_),
        "vkCreateDescriptorPool(gsplat)");
}

void VulkanViewportShell::CreateUniformResources() {
    uniformBuffer_ = CreateHostVisibleBuffer(sizeof(FrameUniforms), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
}

void VulkanViewportShell::CreatePointPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(point vertex)");
    const auto fragmentModule = CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(point fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

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
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;

    const std::array<VkPipelineColorBlendAttachmentState, 3> colorBlendAttachments = {
        MakeAlphaBlendAttachment(),
        MakeDisabledBlendAttachment(),
        MakeDisabledBlendAttachment(),
    };
    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = static_cast<std::uint32_t>(colorBlendAttachments.size());
    colorBlending.pAttachments = colorBlendAttachments.data();

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &pointDescriptorSetLayout_;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pointPipelineLayout_), "vkCreatePipelineLayout(point)");

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
    pipelineInfo.layout = pointPipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 2;

    Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pointPipeline_), "vkCreateGraphicsPipelines(point)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
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

    const std::array<VkPipelineColorBlendAttachmentState, 2> colorBlendAttachments = {
        MakeAdditiveBlendAttachment(),
        MakeRevealageBlendAttachment(),
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
    pipelineInfo.subpass = 0;

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
    pipelineInfo.subpass = 1;

    Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &compositePipeline_), "vkCreateGraphicsPipelines(composite)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateFramebuffers() {
    framebuffers_.clear();
    framebuffers_.reserve(imageViews_.size());

    for (const auto& imageView : imageViews_) {
        const std::array<VkImageView, 4> attachments = {
            imageView,
            depthImage_.view,
            accumulationImage_.view,
            revealageImage_.view,
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

void VulkanViewportShell::CreateDepthResources() {
    DestroyImage(&depthImage_);
    depthFormat_ = SelectDepthFormat();
    depthImage_ = CreateAttachmentImage(
        depthFormat_,
        VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_DEPTH_BIT);
}

void VulkanViewportShell::CreateAccumulationResources() {
    DestroyImage(&accumulationImage_);
    DestroyImage(&revealageImage_);

    accumulationImage_ = CreateAttachmentImage(
        accumulationFormat_,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
    revealageImage_ = CreateAttachmentImage(
        revealageFormat_,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);
}

void VulkanViewportShell::CreateCommandPool() {
    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = graphicsQueueFamily_;

    Check(vkCreateCommandPool(device_, &poolInfo, nullptr, &commandPool_), "vkCreateCommandPool");
}

void VulkanViewportShell::CreateCommandBuffers() {
    if (!commandBuffers_.empty()) {
        vkFreeCommandBuffers(
            device_,
            commandPool_,
            static_cast<std::uint32_t>(commandBuffers_.size()),
            commandBuffers_.data());
        commandBuffers_.clear();
    }

    commandBuffers_.resize(framebuffers_.size());

    VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocInfo.commandPool = commandPool_;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = static_cast<std::uint32_t>(commandBuffers_.size());

    Check(vkAllocateCommandBuffers(device_, &allocInfo, commandBuffers_.data()), "vkAllocateCommandBuffers");
}

void VulkanViewportShell::CreateSyncObjects() {
    VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    Check(vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &imageAvailableSemaphore_), "vkCreateSemaphore(imageAvailable)");
    Check(vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &renderFinishedSemaphore_), "vkCreateSemaphore(renderFinished)");
    Check(vkCreateFence(device_, &fenceInfo, nullptr, &inFlightFence_), "vkCreateFence");
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
    initInfo.PipelineInfoMain.RenderPass = renderPass_;
    initInfo.PipelineInfoMain.Subpass = 3;
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    initInfo.CheckVkResultFn = &CheckImGuiResult;

    if (!ImGui_ImplVulkan_Init(&initInfo)) {
        throw std::runtime_error{"ImGui Vulkan backend initialization failed."};
    }
}

void VulkanViewportShell::UploadImGuiFonts() {}

void VulkanViewportShell::UpdatePointCloudDescriptorSet(ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }

    if (resources->descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &pointDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &resources->descriptorSet),
            "vkAllocateDescriptorSets(point)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = uniformBuffer_.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo scalarInfo{};
    scalarInfo.buffer = resources->scalarFieldBuffer.buffer;
    scalarInfo.offset = 0;
    scalarInfo.range = resources->scalarFieldBuffer.size;

    VkDescriptorBufferInfo styleInfo{};
    styleInfo.buffer = resources->styleBuffer.buffer;
    styleInfo.offset = 0;
    styleInfo.range = sizeof(PointCloudStyleGpu);

    std::array<VkWriteDescriptorSet, 3> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = resources->descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = resources->descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &scalarInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = resources->descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &styleInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::CreateOrUpdateCompositeDescriptorSet() {
    if (compositeDescriptorSet_ == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &compositeDescriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &allocInfo, &compositeDescriptorSet_), "vkAllocateDescriptorSets(composite)");
    }

    VkDescriptorImageInfo accumulationInfo{};
    accumulationInfo.imageView = accumulationImage_.view;
    accumulationInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo revealageInfo{};
    revealageInfo.imageView = revealageImage_.view;
    revealageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 2> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = compositeDescriptorSet_;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[0].descriptorCount = 1;
    writes[0].pImageInfo = &accumulationInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = compositeDescriptorSet_;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[1].descriptorCount = 1;
    writes[1].pImageInfo = &revealageInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateGaussianSplatDescriptorSet(ActiveGaussianSplatResources* resources) {
    if (resources == nullptr) {
        return;
    }

    if (resources->descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = gaussianSplatDescriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gaussianSplatDescriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &allocInfo, &resources->descriptorSet), "vkAllocateDescriptorSets(gsplat)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = uniformBuffer_.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo centerInfo{resources->centerBuffer.buffer, 0, resources->centerBuffer.size};
    VkDescriptorBufferInfo scaleInfo{resources->scaleBuffer.buffer, 0, resources->scaleBuffer.size};
    VkDescriptorBufferInfo rotationInfo{resources->rotationBuffer.buffer, 0, resources->rotationBuffer.size};
    VkDescriptorBufferInfo opacityInfo{resources->opacityBuffer.buffer, 0, resources->opacityBuffer.size};
    VkDescriptorBufferInfo shInfo{resources->shBuffer.buffer, 0, resources->shBuffer.size};
    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = depthImage_.view;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 7> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = resources->descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = resources->descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &centerInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = resources->descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &scaleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = resources->descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = 1;
    writes[3].pBufferInfo = &rotationInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = resources->descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &opacityInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = resources->descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &shInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = resources->descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[6].descriptorCount = 1;
    writes[6].pImageInfo = &sceneDepthInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateHighQualityGaussianDescriptorSet() {
    auto& scene = highQualityGaussianScene_;
    if (scene.splatCount == 0 || scene.layerCount == 0) {
        return;
    }

    if (scene.descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = gaussianSplatDescriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &highQualityGaussianSplatDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &scene.descriptorSet),
            "vkAllocateDescriptorSets(gsplat_hq)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = uniformBuffer_.buffer;
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
    VkDescriptorBufferInfo layerStyleInfo{scene.layerStyleBuffer.buffer, 0, scene.layerStyleBuffer.size};
    VkDescriptorBufferInfo sortedIndexInfo{scene.sortedIndexBuffer.buffer, 0, scene.sortedIndexBuffer.size};

    std::array<VkWriteDescriptorSet, 9> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = scene.descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = scene.descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &centerInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = scene.descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &scaleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = scene.descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = 1;
    writes[3].pBufferInfo = &rotationInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = scene.descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &opacityInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = scene.descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &shInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = scene.descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &layerStyleIndexInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = scene.descriptorSet;
    writes[7].dstBinding = 7;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &layerStyleInfo;

    writes[8] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[8].dstSet = scene.descriptorSet;
    writes[8].dstBinding = 8;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[8].descriptorCount = 1;
    writes[8].pBufferInfo = &sortedIndexInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::RefreshHighQualityGaussianScene() {
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
        if (highQualityGaussianScene_.splatCount > 0 || highQualityGaussianScene_.descriptorSet != VK_NULL_HANDLE) {
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

        highQualityGaussianScene_.layerStyleBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(
                std::max<std::uint32_t>(1U, highQualityGaussianScene_.layerCount) *
                sizeof(HighQualityGaussianLayerStyle)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        highQualityGaussianScene_.sortedIndexBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(
                std::max<std::uint32_t>(1U, highQualityGaussianScene_.splatCount) * sizeof(std::uint32_t)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);

        UpdateHighQualityGaussianDescriptorSet();
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
        highQualityGaussianScene_.layerStyleBuffer,
        layerStyles.data(),
        static_cast<VkDeviceSize>(layerStyles.size() * sizeof(HighQualityGaussianLayerStyle)));

    const bool needsResort =
        !highQualityGaussianScene_.hasSortedView ||
        !MatricesApproximatelyEqual(highQualityGaussianScene_.lastSortedView, renderState_.view);
    if (needsResort) {
        const auto sortedIndices = renderer::gsplat::SortHighQualityGaussianIndices(
            highQualityGaussianScene_.mergedLocalCenters,
            hqLayerInputs,
            highQualityGaussianScene_.layerRanges,
            renderState_.view);

        UploadBufferData(
            highQualityGaussianScene_.sortedIndexBuffer,
            sortedIndices.data(),
            static_cast<VkDeviceSize>(sortedIndices.size() * sizeof(std::uint32_t)));
        highQualityGaussianScene_.lastSortedView = renderState_.view;
        highQualityGaussianScene_.hasSortedView = true;
    }
}

void VulkanViewportShell::CleanupSwapchain() {
    for (const auto framebuffer : framebuffers_) {
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
    }
    framebuffers_.clear();

    DestroyImage(&depthImage_);
    DestroyImage(&accumulationImage_);
    DestroyImage(&revealageImage_);

    if (!commandBuffers_.empty() && commandPool_ != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(
            device_,
            commandPool_,
            static_cast<std::uint32_t>(commandBuffers_.size()),
            commandBuffers_.data());
        commandBuffers_.clear();
    }

    for (const auto imageView : imageViews_) {
        vkDestroyImageView(device_, imageView, nullptr);
    }
    imageViews_.clear();

    if (swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device_, swapchain_, nullptr);
        swapchain_ = VK_NULL_HANDLE;
    }

    swapchainImages_.clear();
}

void VulkanViewportShell::CleanupPointCloudResources(ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }

    DestroyBuffer(&resources->positionBuffer);
    DestroyBuffer(&resources->colorBuffer);
    DestroyBuffer(&resources->scalarFieldBuffer);
    DestroyBuffer(&resources->styleBuffer);
    DestroyBuffer(&resources->sampledIndexBuffer);
    if (resources->descriptorSet != VK_NULL_HANDLE &&
        descriptorPool_ != VK_NULL_HANDLE &&
        device_ != VK_NULL_HANDLE) {
        vkFreeDescriptorSets(device_, descriptorPool_, 1, &resources->descriptorSet);
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

    if (resources->descriptorSet != VK_NULL_HANDLE &&
        gaussianSplatDescriptorPool_ != VK_NULL_HANDLE &&
        device_ != VK_NULL_HANDLE) {
        vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &resources->descriptorSet);
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
    DestroyBuffer(&highQualityGaussianScene_.layerStyleBuffer);
    DestroyBuffer(&highQualityGaussianScene_.sortedIndexBuffer);

    if (highQualityGaussianScene_.descriptorSet != VK_NULL_HANDLE &&
        gaussianSplatDescriptorPool_ != VK_NULL_HANDLE &&
        device_ != VK_NULL_HANDLE) {
        vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &highQualityGaussianScene_.descriptorSet);
    }

    highQualityGaussianScene_ = HighQualityGaussianSceneResources{};
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
    CreateDepthResources();
    CreateAccumulationResources();
    CreateFramebuffers();
    CreateCommandBuffers();
    CreateOrUpdateCompositeDescriptorSet();
    for (auto& resources : gaussianSplatResources_) {
        if (resources.descriptorSet != VK_NULL_HANDLE) {
            UpdateGaussianSplatDescriptorSet(&resources);
        }
    }

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_SetMinImageCount(static_cast<int>(std::max<std::size_t>(2, swapchainImages_.size())));
    }
}

void VulkanViewportShell::RecordCommandBuffer(VkCommandBuffer commandBuffer, std::uint32_t imageIndex) {
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    Check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer");

    std::array<VkClearValue, 4> clearValues{};
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

    VkRenderPassBeginInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    renderPassInfo.renderPass = renderPass_;
    renderPassInfo.framebuffer = framebuffers_[imageIndex];
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = {swapchainWidth_, swapchainHeight_};
    renderPassInfo.clearValueCount = static_cast<std::uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    const VkViewport viewport{
        0.0F,
        0.0F,
        static_cast<float>(swapchainWidth_),
        static_cast<float>(swapchainHeight_),
        0.0F,
        1.0F,
    };
    const VkRect2D scissor{{0, 0}, {swapchainWidth_, swapchainHeight_}};
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (!renderState_.pointCloudLayers.empty()) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pointPipeline_);

        for (const auto& layer : renderState_.pointCloudLayers) {
            const auto* resources = FindPointCloudResources(layer.layerId);
            if (resources == nullptr ||
                resources->activePointCount == 0 ||
                resources->descriptorSet == VK_NULL_HANDLE) {
                continue;
            }

            PointCloudStyleGpu styleGpu;
            styleGpu.solidColor = glm::vec4{
                layer.style.solidColor[0],
                layer.style.solidColor[1],
                layer.style.solidColor[2],
                layer.style.solidColor[3],
            };
            styleGpu.globalControl = glm::uvec4{
                static_cast<std::uint32_t>(layer.style.colorMode),
                static_cast<std::uint32_t>(layer.style.colormap),
                resources->scalarFieldCount,
                layer.hasSourceRgb ? 1U : 0U,
            };
            styleGpu.pointMeta = glm::uvec4{
                resources->pointCount,
                resources->activePointCount,
                0U,
                0U,
            };
            styleGpu.pointSize = MakePointCloudBindingGpu(layer.style.pointSize);
            styleGpu.opacity = MakePointCloudBindingGpu(layer.style.opacity);
            styleGpu.emissive = MakePointCloudBindingGpu(layer.style.emissiveStrength);
            styleGpu.xray = MakePointCloudBindingGpu(layer.style.xrayStrength);
            styleGpu.depthFade = MakePointCloudBindingGpu(layer.style.depthFade);
            styleGpu.colormapPosition = MakePointCloudBindingGpu(layer.style.colormapPosition);

            UploadBufferData(resources->styleBuffer, &styleGpu, sizeof(styleGpu));

            vkCmdBindDescriptorSets(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                pointPipelineLayout_,
                0,
                1,
                &resources->descriptorSet,
                0,
                nullptr);

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

            if (resources->usingSampledIndices && resources->sampledIndexBuffer.buffer != VK_NULL_HANDLE) {
                vkCmdBindIndexBuffer(
                    commandBuffer,
                    resources->sampledIndexBuffer.buffer,
                    0,
                    VK_INDEX_TYPE_UINT32);
                vkCmdDrawIndexed(commandBuffer, resources->activePointCount, 1, 0, 0, 0);
            } else {
                vkCmdDraw(commandBuffer, resources->activePointCount, 1, 0, 0);
            }
        }
    }

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (!renderState_.gaussianSplatLayers.empty()) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, gaussianSplatPipeline_);

        for (const auto& layer : renderState_.gaussianSplatLayers) {
            if (layer.style.qualityMode == renderer::gsplat::GaussianSplatQualityMode::High) {
                continue;
            }

            const auto* resources = FindGaussianSplatResources(layer.layerId);
            if (resources == nullptr || resources->splatCount == 0 || resources->descriptorSet == VK_NULL_HANDLE) {
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
                &resources->descriptorSet,
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

    if (compositeDescriptorSet_ != VK_NULL_HANDLE) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            compositePipelineLayout_,
            0,
            1,
            &compositeDescriptorSet_,
            0,
            nullptr);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
    }

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (highQualityGaussianScene_.descriptorSet != VK_NULL_HANDLE &&
        highQualityGaussianScene_.splatCount > 0) {
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
            &highQualityGaussianScene_.descriptorSet,
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

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
    }

    vkCmdEndRenderPass(commandBuffer);
    Check(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer");
}

void VulkanViewportShell::UpdateUniformBuffer() {
    if (uniformBuffer_.buffer == VK_NULL_HANDLE) {
        return;
    }

    FrameUniforms uniforms;
    uniforms.viewProjection = renderState_.viewProjection;
    uniforms.view = renderState_.view;
    uniforms.projection = renderState_.projection;
    uniforms.cameraPosition = glm::vec4{renderState_.cameraPosition, 0.0F};
    uniforms.depthParameters = glm::vec4{
        0.0F,
        renderState_.nearPlane,
        renderState_.farPlane,
        0.0F,
    };
    const float viewportWidth = std::max(1.0F, static_cast<float>(swapchainWidth_));
    const float viewportHeight = std::max(1.0F, static_cast<float>(swapchainHeight_));
    uniforms.viewportParameters = glm::vec4{
        viewportWidth,
        viewportHeight,
        2.0F / viewportWidth,
        2.0F / viewportHeight,
    };

    UploadBufferData(uniformBuffer_, &uniforms, sizeof(uniforms));
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
    return allocation;
}

void VulkanViewportShell::UploadBufferData(
    const BufferAllocation& buffer,
    const void* data,
    VkDeviceSize size) const {
    if (buffer.memory == VK_NULL_HANDLE || data == nullptr || size == 0) {
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
    ImageAllocation image;
    image.format = format;

    VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = swapchainWidth_;
    imageInfo.extent.height = swapchainHeight_;
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
