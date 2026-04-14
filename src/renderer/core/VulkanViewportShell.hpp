#pragma once

#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/gsplat/HighQualityGaussianScene.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <vulkan/vulkan.h>

struct GLFWwindow;

namespace invisible_places::renderer::core {

struct ViewportDiagnostics {
    std::string rendererName;
    std::string driverName;
    std::string summary;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

struct SceneRenderState {
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::mat4 viewProjection{1.0F};
    glm::vec3 cameraPosition{0.0F, 0.0F, 1.0F};
    glm::vec4 backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    float nearPlane = 0.05F;
    float farPlane = 1000.0F;
    float gaussianSplatFootprintBoost = 1.5F;

    struct PointCloudLayerState {
        std::size_t layerId = 0;
        renderer::pointcloud::PointCloudStyleState style{};
        bool hasSourceRgb = true;
    };

    struct GaussianSplatLayerState {
        std::size_t layerId = 0;
        renderer::gsplat::GaussianSplatStyleState style{};
        glm::mat4 localToWorld{1.0F};
    };

    std::vector<PointCloudLayerState> pointCloudLayers;
    std::vector<GaussianSplatLayerState> gaussianSplatLayers;
};

class VulkanViewportShell {
  public:
    explicit VulkanViewportShell(GLFWwindow* window);
    ~VulkanViewportShell();

    VulkanViewportShell(const VulkanViewportShell&) = delete;
    VulkanViewportShell& operator=(const VulkanViewportShell&) = delete;

    VulkanViewportShell(VulkanViewportShell&&) = delete;
    VulkanViewportShell& operator=(VulkanViewportShell&&) = delete;

    void BeginUiFrame();
    void DrawFrame();
    void WaitIdle() const;
    void UpdateRenderState(const SceneRenderState& state);
    void UploadPointCloud(
        std::size_t layerId,
        const invisible_places::io::LoadedPointCloud& cloud,
        const std::vector<std::uint32_t>& sampledIndices);
    void UpdatePointBudget(std::size_t layerId, const std::vector<std::uint32_t>& sampledIndices);
    void RemovePointCloud(std::size_t layerId);
    void ClearPointClouds();
    void UploadGaussianSplats(std::size_t layerId, const invisible_places::io::LoadedGaussianSplat& splats);
    void RemoveGaussianSplats(std::size_t layerId);
    void ClearGaussianSplats();

    [[nodiscard]] bool UiWantsMouseCapture() const;
    [[nodiscard]] bool UiWantsKeyboardCapture() const;
    [[nodiscard]] bool HasPointClouds() const;
    [[nodiscard]] bool HasGaussianSplats() const;
    [[nodiscard]] std::uint32_t Width() const { return swapchainWidth_; }
    [[nodiscard]] std::uint32_t Height() const { return swapchainHeight_; }

    [[nodiscard]] const ViewportDiagnostics& Diagnostics() const { return diagnostics_; }

  private:
    struct BufferAllocation {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkDeviceSize size = 0;
    };

    struct ImageAllocation {
        VkImage image = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        VkFormat format = VK_FORMAT_UNDEFINED;
    };

    struct ActivePointCloudResources {
        std::size_t layerId = 0;
        BufferAllocation positionBuffer{};
        BufferAllocation colorBuffer{};
        BufferAllocation scalarFieldBuffer{};
        BufferAllocation styleBuffer{};
        VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
        BufferAllocation sampledIndexBuffer{};
        std::uint32_t pointCount = 0;
        std::uint32_t activePointCount = 0;
        std::uint32_t scalarFieldCount = 0;
        bool usingSampledIndices = false;
        bool hasSourceRgb = false;
    };

    struct ActiveGaussianSplatResources {
        std::size_t layerId = 0;
        BufferAllocation centerBuffer{};
        BufferAllocation scaleBuffer{};
        BufferAllocation rotationBuffer{};
        BufferAllocation opacityBuffer{};
        BufferAllocation shBuffer{};
        std::vector<invisible_places::io::Float3> cpuCenters;
        std::vector<std::array<float, 3>> cpuScales;
        std::vector<std::array<float, 4>> cpuRotations;
        std::vector<float> cpuOpacities;
        std::vector<float> cpuShCoefficients;
        VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
        std::uint32_t splatCount = 0;
        std::uint64_t revision = 0;
    };

    struct HighQualityGaussianSceneResources {
        BufferAllocation centerBuffer{};
        BufferAllocation scaleBuffer{};
        BufferAllocation rotationBuffer{};
        BufferAllocation opacityBuffer{};
        BufferAllocation shBuffer{};
        BufferAllocation layerStyleIndexBuffer{};
        BufferAllocation layerStyleBuffer{};
        BufferAllocation sortedIndexBuffer{};
        std::vector<renderer::gsplat::HighQualityGaussianLayerSignature> layerSignatures;
        std::vector<renderer::gsplat::HighQualityGaussianLayerRange> layerRanges;
        std::vector<invisible_places::io::Float3> mergedLocalCenters;
        glm::mat4 lastSortedView{1.0F};
        bool hasSortedView = false;
        VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
        std::uint32_t splatCount = 0;
        std::uint32_t layerCount = 0;
    };

    void CreateInstance();
    void CreateSurface();
    void PickPhysicalDevice();
    void CreateLogicalDevice();
    void CreateSwapchain();
    void CreateImageViews();
    void CreateRenderPass();
    void CreatePointDescriptorSetLayout();
    void CreateGaussianSplatDescriptorSetLayout();
    void CreateHighQualityGaussianSplatDescriptorSetLayout();
    void CreateCompositeDescriptorSetLayout();
    void CreateDescriptorPools();
    void CreateUniformResources();
    void CreatePointPipeline();
    void CreateGaussianSplatPipeline();
    void CreateHighQualityGaussianSplatPipeline();
    void CreateCompositePipeline();
    void CreateFramebuffers();
    void CreateDepthResources();
    void CreateAccumulationResources();
    void CreateCommandPool();
    void CreateCommandBuffers();
    void CreateSyncObjects();
    void CreateImGuiResources();
    void UploadImGuiFonts();
    void UpdatePointCloudDescriptorSet(ActivePointCloudResources* resources);
    void CreateOrUpdateCompositeDescriptorSet();
    void UpdateGaussianSplatDescriptorSet(ActiveGaussianSplatResources* resources);
    void UpdateHighQualityGaussianDescriptorSet();
    void RefreshHighQualityGaussianScene();
    void CleanupSwapchain();
    void CleanupPointCloudResources(ActivePointCloudResources* resources);
    void CleanupGaussianSplatResources(ActiveGaussianSplatResources* resources);
    void CleanupHighQualityGaussianScene();
    void RecreateSwapchain();
    void RecordCommandBuffer(VkCommandBuffer commandBuffer, std::uint32_t imageIndex);
    void UpdateUniformBuffer();

    [[nodiscard]] BufferAllocation CreateHostVisibleBuffer(VkDeviceSize size, VkBufferUsageFlags usage) const;
    void UploadBufferData(const BufferAllocation& buffer, const void* data, VkDeviceSize size) const;
    void DestroyBuffer(BufferAllocation* buffer);
    void DestroyImage(ImageAllocation* image);
    [[nodiscard]] std::uint32_t FindMemoryType(
        std::uint32_t typeFilter,
        VkMemoryPropertyFlags requiredFlags,
        VkMemoryPropertyFlags preferredFlags) const;
    [[nodiscard]] VkFormat SelectDepthFormat() const;
    [[nodiscard]] VkFormat SelectAccumulationFormat() const;
    [[nodiscard]] VkFormat SelectRevealageFormat() const;
    [[nodiscard]] std::vector<char> ReadBinaryFile(const std::string& filePath) const;
    [[nodiscard]] ImageAllocation CreateAttachmentImage(
        VkFormat format,
        VkImageUsageFlags usage,
        VkImageAspectFlags aspectFlags) const;
    [[nodiscard]] ActivePointCloudResources* FindPointCloudResources(std::size_t layerId);
    [[nodiscard]] const ActivePointCloudResources* FindPointCloudResources(std::size_t layerId) const;
    [[nodiscard]] ActiveGaussianSplatResources* FindGaussianSplatResources(std::size_t layerId);
    [[nodiscard]] const ActiveGaussianSplatResources* FindGaussianSplatResources(std::size_t layerId) const;

    static void FramebufferResizeCallback(GLFWwindow* window, int width, int height);

    GLFWwindow* window_ = nullptr;

    VkInstance instance_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue graphicsQueue_ = VK_NULL_HANDLE;
    VkQueue presentQueue_ = VK_NULL_HANDLE;
    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    VkPipelineLayout pointPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout gaussianSplatPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout highQualityGaussianSplatPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout compositePipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pointPipeline_ = VK_NULL_HANDLE;
    VkPipeline gaussianSplatPipeline_ = VK_NULL_HANDLE;
    VkPipeline highQualityGaussianSplatPipeline_ = VK_NULL_HANDLE;
    VkPipeline compositePipeline_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout pointDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout gaussianSplatDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout highQualityGaussianSplatDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout compositeDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorPool gaussianSplatDescriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorPool imguiDescriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorSet compositeDescriptorSet_ = VK_NULL_HANDLE;
    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    VkSemaphore imageAvailableSemaphore_ = VK_NULL_HANDLE;
    VkSemaphore renderFinishedSemaphore_ = VK_NULL_HANDLE;
    VkFence inFlightFence_ = VK_NULL_HANDLE;
    VkFormat depthFormat_ = VK_FORMAT_UNDEFINED;
    VkFormat accumulationFormat_ = VK_FORMAT_UNDEFINED;
    VkFormat revealageFormat_ = VK_FORMAT_UNDEFINED;

    std::vector<VkImage> swapchainImages_;
    std::vector<VkImageView> imageViews_;
    std::vector<VkFramebuffer> framebuffers_;
    std::vector<VkCommandBuffer> commandBuffers_;

    std::uint32_t graphicsQueueFamily_ = 0;
    std::uint32_t presentQueueFamily_ = 0;
    VkFormat swapchainImageFormat_ = VK_FORMAT_UNDEFINED;
    std::uint32_t swapchainWidth_ = 0;
    std::uint32_t swapchainHeight_ = 0;
    bool framebufferResized_ = false;
    bool enablePortabilitySubset_ = false;
    bool uiFrameBegun_ = false;

    BufferAllocation uniformBuffer_{};
    ImageAllocation depthImage_{};
    ImageAllocation accumulationImage_{};
    ImageAllocation revealageImage_{};
    std::vector<ActivePointCloudResources> pointCloudResources_;
    std::vector<ActiveGaussianSplatResources> gaussianSplatResources_;
    HighQualityGaussianSceneResources highQualityGaussianScene_{};
    bool highQualityGaussianSceneDirty_ = true;
    SceneRenderState renderState_{};
    ViewportDiagnostics diagnostics_{};
};

}  // namespace invisible_places::renderer::core
