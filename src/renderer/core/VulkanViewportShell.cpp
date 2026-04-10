#include "renderer/core/VulkanViewportShell.hpp"

#include "InvisiblePlacesBuildConfig.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <array>
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
    glm::vec4 cameraPosition{0.0F, 0.0F, 1.0F, 0.0F};
    glm::vec4 depthParameters{0.0F, 0.05F, 1000.0F, 0.0F};
};

struct alignas(16) PointCloudPushConstants {
    glm::vec4 solidColor{0.93F, 0.88F, 0.72F, 1.0F};
    glm::vec4 scalarRange{0.0F, 1.0F, 0.0F, 0.0F};
    glm::vec4 shading{2.0F, 1.0F, 0.0F, 0.0F};
    glm::uvec4 control{0U, 0U, 1U, 1U};
    glm::vec4 extra{0.0F, 0.0F, 0.0F, 0.0F};
};

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

VkDescriptorPoolSize MakePoolSize(VkDescriptorType type, std::uint32_t descriptorCount) {
    return VkDescriptorPoolSize{type, descriptorCount};
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
    CreateDescriptorSetLayout();
    CreateDescriptorPool();
    CreateUniformResources();
    CreatePointPipeline();
    CreateDepthResources();
    CreateFramebuffers();
    CreateCommandPool();
    CreateCommandBuffers();
    CreateSyncObjects();
    CreateImGuiResources();
    CreateOrUpdateDescriptorSet();
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
    if (pipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
    }
    if (descriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
    }
    if (imguiDescriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, imguiDescriptorPool_, nullptr);
    }
    if (descriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, descriptorSetLayout_, nullptr);
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

    UpdateUniformBuffer();

    vkWaitForFences(device_, 1, &inFlightFence_, VK_TRUE, UINT64_MAX);

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

void VulkanViewportShell::UpdateRenderState(const PointCloudRenderState& state) {
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

    resources.scalarFieldBuffers.reserve(cloud.scalarFields.size());
    for (std::size_t fieldIndex = 0; fieldIndex < cloud.scalarFields.size(); ++fieldIndex) {
        resources.scalarFieldBuffers.push_back(BuildScalarFieldBuffer(cloud, fieldIndex));
    }

    if (resources.scalarFieldBuffers.empty()) {
        std::vector<float> fallbackScalars(cloud.PointCount(), 0.0F);
        resources.fallbackScalarFieldBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(fallbackScalars.size() * sizeof(float)),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        UploadBufferData(
            resources.fallbackScalarFieldBuffer,
            fallbackScalars.data(),
            resources.fallbackScalarFieldBuffer.size);
    }

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
    appInfo.pEngineName = "InvisiblePlacesPointPreview";
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
            << swapchainHeight_ << " | point-cloud Vulkan viewport";
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

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthAttachmentRef{};
    depthAttachmentRef.attachment = 1;
    depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorAttachmentRef;
    subpass.pDepthStencilAttachment = &depthAttachmentRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    const std::array<VkAttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};
    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    Check(vkCreateRenderPass(device_, &renderPassInfo, nullptr, &renderPass_), "vkCreateRenderPass");
}

void VulkanViewportShell::CreateDescriptorSetLayout() {
    VkDescriptorSetLayoutBinding uniformBinding{};
    uniformBinding.binding = 0;
    uniformBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uniformBinding.descriptorCount = 1;
    uniformBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &uniformBinding;

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &descriptorSetLayout_),
        "vkCreateDescriptorSetLayout");
}

void VulkanViewportShell::CreateDescriptorPool() {
    const VkDescriptorPoolSize poolSize = MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1);

    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    poolInfo.maxSets = 1;

    Check(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_), "vkCreateDescriptorPool");
}

void VulkanViewportShell::CreateUniformResources() {
    uniformBuffer_ = CreateHostVisibleBuffer(sizeof(FrameUniforms), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
}

void VulkanViewportShell::CreatePointPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.frag.spv").string());

    VkShaderModuleCreateInfo vertexModuleInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    vertexModuleInfo.codeSize = vertexShaderCode.size();
    vertexModuleInfo.pCode = reinterpret_cast<const std::uint32_t*>(vertexShaderCode.data());

    VkShaderModuleCreateInfo fragmentModuleInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    fragmentModuleInfo.codeSize = fragmentShaderCode.size();
    fragmentModuleInfo.pCode = reinterpret_cast<const std::uint32_t*>(fragmentShaderCode.data());

    VkShaderModule vertexModule = VK_NULL_HANDLE;
    VkShaderModule fragmentModule = VK_NULL_HANDLE;
    Check(vkCreateShaderModule(device_, &vertexModuleInfo, nullptr, &vertexModule), "vkCreateShaderModule(vertex)");
    Check(vkCreateShaderModule(device_, &fragmentModuleInfo, nullptr, &fragmentModule), "vkCreateShaderModule(fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    const std::array<VkVertexInputBindingDescription, 3> bindingDescriptions = {
        VkVertexInputBindingDescription{0, sizeof(invisible_places::io::Float3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(std::uint32_t), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{2, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},
    };

    const std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions = {
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R8G8B8A8_UNORM, 0},
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32_SFLOAT, 0},
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
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;
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
    depthStencil.depthBoundsTestEnable = VK_FALSE;
    depthStencil.stencilTestEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_TRUE;
    colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
    colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.logicOpEnable = VK_FALSE;
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(PointCloudPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pipelineLayout_), "vkCreatePipelineLayout");

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
    pipelineInfo.layout = pipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 0;

    Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pointPipeline_), "vkCreateGraphicsPipelines");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateFramebuffers() {
    framebuffers_.clear();
    framebuffers_.reserve(imageViews_.size());

    for (const auto& imageView : imageViews_) {
        const std::array<VkImageView, 2> attachments = {imageView, depthImage_.view};

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

    VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = swapchainWidth_;
    imageInfo.extent.height = swapchainHeight_;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = depthFormat_;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    Check(vkCreateImage(device_, &imageInfo, nullptr, &depthImage_.image), "vkCreateImage(depth)");

    VkMemoryRequirements memoryRequirements{};
    vkGetImageMemoryRequirements(device_, depthImage_.image, &memoryRequirements);

    VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocInfo.allocationSize = memoryRequirements.size;
    allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, 0, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    Check(vkAllocateMemory(device_, &allocInfo, nullptr, &depthImage_.memory), "vkAllocateMemory(depth)");
    Check(vkBindImageMemory(device_, depthImage_.image, depthImage_.memory, 0), "vkBindImageMemory(depth)");

    VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    viewInfo.image = depthImage_.image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = depthFormat_;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    Check(vkCreateImageView(device_, &viewInfo, nullptr, &depthImage_.view), "vkCreateImageView(depth)");
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
    initInfo.PipelineInfoMain.Subpass = 0;
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    initInfo.CheckVkResultFn = &CheckImGuiResult;

    if (!ImGui_ImplVulkan_Init(&initInfo)) {
        throw std::runtime_error{"ImGui Vulkan backend initialization failed."};
    }
}

void VulkanViewportShell::UploadImGuiFonts() {}

void VulkanViewportShell::CreateOrUpdateDescriptorSet() {
    if (descriptorSet_ == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &descriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet_), "vkAllocateDescriptorSets");
    }

    VkDescriptorBufferInfo bufferInfo{};
    bufferInfo.buffer = uniformBuffer_.buffer;
    bufferInfo.offset = 0;
    bufferInfo.range = sizeof(FrameUniforms);

    VkWriteDescriptorSet descriptorWrite{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    descriptorWrite.dstSet = descriptorSet_;
    descriptorWrite.dstBinding = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfo;

    vkUpdateDescriptorSets(device_, 1, &descriptorWrite, 0, nullptr);
}

void VulkanViewportShell::CleanupSwapchain() {
    for (const auto framebuffer : framebuffers_) {
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
    }
    framebuffers_.clear();

    DestroyImage(&depthImage_);

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
    for (auto& buffer : resources->scalarFieldBuffers) {
        DestroyBuffer(&buffer);
    }
    resources->scalarFieldBuffers.clear();
    DestroyBuffer(&resources->fallbackScalarFieldBuffer);
    DestroyBuffer(&resources->sampledIndexBuffer);
    *resources = ActivePointCloudResources{};
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
    CreateFramebuffers();
    CreateCommandBuffers();

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_SetMinImageCount(static_cast<int>(std::max<std::size_t>(2, swapchainImages_.size())));
    }
}

void VulkanViewportShell::RecordCommandBuffer(VkCommandBuffer commandBuffer, std::uint32_t imageIndex) {
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    Check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer");

    std::array<VkClearValue, 2> clearValues{};
    clearValues[0].color = {{0.93F, 0.91F, 0.85F, 1.0F}};
    clearValues[1].depthStencil = {1.0F, 0};

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

    if (HasPointClouds()) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pointPipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            pipelineLayout_,
            0,
            1,
            &descriptorSet_,
            0,
            nullptr);

        for (const auto& layer : renderState_.layers) {
            const auto* resources = FindPointCloudResources(layer.layerId);
            if (resources == nullptr || resources->activePointCount == 0) {
                continue;
            }

            PointCloudPushConstants pushConstants;
            pushConstants.solidColor = glm::vec4{
                layer.style.solidColor[0],
                layer.style.solidColor[1],
                layer.style.solidColor[2],
                layer.style.solidColor[3],
            };
            const auto scalarMinimum = layer.style.scalarRange.minimum;
            const auto scalarMaximum =
                std::max(layer.style.scalarRange.maximum, layer.style.scalarRange.minimum + 0.0001F);
            pushConstants.scalarRange = glm::vec4{scalarMinimum, scalarMaximum, 0.0F, 0.0F};
            pushConstants.shading = glm::vec4{
                layer.style.pointSize,
                layer.style.opacity,
                layer.style.emissiveStrength,
                layer.style.xrayStrength,
            };
            pushConstants.control = glm::uvec4{
                static_cast<std::uint32_t>(layer.style.colorMode),
                static_cast<std::uint32_t>(layer.style.colormap),
                layer.style.scalarRange.clamp ? 1U : 0U,
                layer.hasSourceRgb ? 1U : 0U,
            };
            pushConstants.extra = glm::vec4{layer.style.depthFade, 0.0F, 0.0F, 0.0F};

            vkCmdPushConstants(
                commandBuffer,
                pipelineLayout_,
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                0,
                sizeof(PointCloudPushConstants),
                &pushConstants);

            const auto selectedField = static_cast<std::size_t>(layer.style.selectedScalarField);
            const VkBuffer scalarBuffer =
                selectedField < resources->scalarFieldBuffers.size()
                    ? resources->scalarFieldBuffers[selectedField].buffer
                    : resources->fallbackScalarFieldBuffer.buffer;

            const std::array<VkBuffer, 3> vertexBuffers = {
                resources->positionBuffer.buffer,
                resources->colorBuffer.buffer,
                scalarBuffer,
            };
            constexpr std::array<VkDeviceSize, 3> offsets = {0, 0, 0};
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
    uniforms.cameraPosition = glm::vec4{renderState_.cameraPosition, 0.0F};
    uniforms.depthParameters = glm::vec4{
        0.0F,
        renderState_.nearPlane,
        renderState_.farPlane,
        0.0F,
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

VulkanViewportShell::BufferAllocation VulkanViewportShell::BuildScalarFieldBuffer(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::size_t fieldIndex) const {
    const auto pointCount = cloud.PointCount();
    const auto offset = fieldIndex * pointCount;
    const auto* values = cloud.scalarFieldValues.data() + offset;

    auto buffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(pointCount * sizeof(float)),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    UploadBufferData(buffer, values, buffer.size);
    return buffer;
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

void VulkanViewportShell::FramebufferResizeCallback(GLFWwindow* window, int /*width*/, int /*height*/) {
    auto* shell = static_cast<VulkanViewportShell*>(glfwGetWindowUserPointer(window));
    if (shell != nullptr) {
        shell->framebufferResized_ = true;
    }
}

}  // namespace invisible_places::renderer::core
