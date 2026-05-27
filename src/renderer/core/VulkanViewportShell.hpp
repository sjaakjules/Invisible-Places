#pragma once

#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
#include "output/ExrWriter.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/gsplat/HighQualityGaussianScene.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <array>
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
    std::uint32_t accumulationWidth = 0;
    std::uint32_t accumulationHeight = 0;
    std::uint64_t pointCount = 0;
    float averagePointSizePx = 0.0F;
    std::string pointRenderModes;
    std::uint32_t pointDrawCalls = 0;
    std::uint64_t pointSubmittedCount = 0;
    std::uint64_t pointPassSubmittedCount = 0;
    std::uint32_t pointDepthLayerCount = 0;
    std::uint32_t pointAccumulationLayerCount = 0;
    std::uint32_t pointStyleUploadCount = 0;
    std::uint32_t pointSkippedInactiveBindings = 0;
    std::uint32_t pointOpaqueHardDiscDrawCalls = 0;
    std::uint32_t pointConstantSimpleDrawCalls = 0;
    std::uint32_t pointUnifiedDrawCalls = 0;
    std::uint32_t pointFastBasicDrawCalls = 0;
    std::uint64_t pointFastBasicDrawnPoints = 0;
    std::uint32_t pointDepthPrepassSkippedNoXray = 0;
    bool sceneRenderedThisFrame = false;
    bool sceneCacheActive = false;
    double pointCommandRecordMs = 0.0;
    std::uint32_t framesInFlight = 0;
    std::uint32_t swapchainImageCount = 0;
    std::uint32_t currentFrameIndex = 0;
    double frameRenderMs = 0.0;
    double averageFrameRenderMs = 0.0;
    double minFrameRenderMs = 0.0;
    double maxFrameRenderMs = 0.0;
    double frameFps = 0.0;
    double averageFrameFps = 0.0;
    double frameAverageWindowSeconds = 0.5;
    double frameUiRenderMs = 0.0;
    double frameFenceWaitMs = 0.0;
    double frameAcquireMs = 0.0;
    double frameImageWaitMs = 0.0;
    double framePrepareMs = 0.0;
    double frameCommandBufferMs = 0.0;
    double frameSubmitMs = 0.0;
    double framePresentMs = 0.0;
    double framePlatformWindowsMs = 0.0;
};

struct SceneRenderState {
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::mat4 viewProjection{1.0F};
    glm::vec3 cameraPosition{0.0F, 0.0F, 1.0F};
    glm::vec4 backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    bool eyeDomeLightingEnabled = false;
    float eyeDomeLightingThickness = 1.0F;
    float nearPlane = 0.05F;
    float farPlane = 1000.0F;
    bool hasDepthOfField = false;
    float focusDistance = 1.0F;
    float apertureFStops = 8.0F;
    float depthOfFieldMaxBlurPixels = 24.0F;
    float gaussianSplatFootprintBoost = 1.5F;
    float pointSizeScale = 1.0F;
    float flowTimeSeconds = 0.0F;
    renderer::pointcloud::PointCloudRendererMode pointCloudRendererMode =
        renderer::pointcloud::PointCloudRendererMode::Beauty;

    struct PointCloudLayerState {
        std::size_t layerId = 0;
        renderer::pointcloud::PointCloudStyleState style{};
        std::vector<invisible_places::io::ScalarFieldStats> scalarFields;
        bool hasSourceRgb = true;
        std::uint32_t drawPointCount = 0;
    };

    struct GaussianSplatLayerState {
        std::size_t layerId = 0;
        renderer::gsplat::GaussianSplatStyleState style{};
        glm::mat4 localToWorld{1.0F};
    };

    std::vector<PointCloudLayerState> pointCloudLayers;
    std::vector<GaussianSplatLayerState> gaussianSplatLayers;
};

struct PointCloudExrFrameRequest {
    SceneRenderState renderState{};
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    bool previewDensity = true;
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
    void UpdateInteractivePointSampleBuffer(
        std::size_t layerId,
        const std::vector<std::uint32_t>& sampledIndices);
    void RemovePointCloud(std::size_t layerId);
    void ClearPointClouds();
    void UploadGaussianSplats(std::size_t layerId, const invisible_places::io::LoadedGaussianSplat& splats);
    void RemoveGaussianSplats(std::size_t layerId);
    void ClearGaussianSplats();
    [[nodiscard]] invisible_places::output::HalfRgbaExrImage RenderPointCloudExrFrame(
        const PointCloudExrFrameRequest& request);

    [[nodiscard]] bool UiWantsMouseCapture() const;
    [[nodiscard]] bool UiWantsKeyboardCapture() const;
    [[nodiscard]] bool HasPointClouds() const;
    [[nodiscard]] bool HasGaussianSplats() const;
    [[nodiscard]] std::uint32_t Width() const { return swapchainWidth_; }
    [[nodiscard]] std::uint32_t Height() const { return swapchainHeight_; }

    [[nodiscard]] const ViewportDiagnostics& Diagnostics() const { return diagnostics_; }
    void SetDiagnosticsEnabled(bool enabled);
    [[nodiscard]] bool DiagnosticsEnabled() const { return diagnosticsEnabled_; }
    void SetLiveSceneRenderingEnabled(bool enabled) { liveSceneRenderingEnabled_ = enabled; }
    [[nodiscard]] bool LiveSceneRenderingEnabled() const { return liveSceneRenderingEnabled_; }
    void SetSceneCachingEnabled(bool enabled);
    [[nodiscard]] bool SceneCachingEnabled() const { return sceneCachingEnabled_; }

  private:
    static constexpr std::size_t kFramesInFlight = 2U;

    struct BufferAllocation {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkDeviceSize size = 0;
        void* mapped = nullptr;
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
        BufferAllocation positionStorageBuffer{};
        BufferAllocation colorBuffer{};
        BufferAllocation normalBuffer{};
        BufferAllocation scalarFieldBuffer{};
        std::array<BufferAllocation, kFramesInFlight> styleBuffers{};
        BufferAllocation exrStyleBuffer{};
        std::array<std::vector<VkDescriptorSet>, kFramesInFlight> descriptorSets{};
        VkDescriptorSet exrDescriptorSet = VK_NULL_HANDLE;
        BufferAllocation sampledIndexBuffer{};
        BufferAllocation sampledSurfelIndexBuffer{};
        BufferAllocation interactiveSampledIndexBuffer{};
        BufferAllocation interactiveSurfelIndexBuffer{};
        std::uint32_t pointCount = 0;
        std::uint32_t activePointCount = 0;
        std::uint32_t interactiveSampledIndexCount = 0;
        std::uint32_t scalarFieldCount = 0;
        bool usingSampledIndices = false;
        bool hasSourceRgb = false;
        bool hasNormals = false;
        std::vector<invisible_places::io::Float3> cpuPositions;
    };

    struct PointCloudDrawPlan {
        ActivePointCloudResources* resources = nullptr;
        std::uint32_t drawPointCount = 0;
        bool worldSurfels = false;
        bool sampledBudgetReady = false;
        bool interactiveSampleReady = false;
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
        std::array<std::vector<VkDescriptorSet>, kFramesInFlight> descriptorSets{};
        std::uint32_t splatCount = 0;
        std::uint64_t revision = 0;
    };

    struct FrameResources {
        BufferAllocation uniformBuffer{};
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        VkSemaphore imageAvailableSemaphore = VK_NULL_HANDLE;
        VkSemaphore renderFinishedSemaphore = VK_NULL_HANDLE;
        VkFence fence = VK_NULL_HANDLE;
    };

    struct HighQualityGaussianSceneResources {
        BufferAllocation centerBuffer{};
        BufferAllocation scaleBuffer{};
        BufferAllocation rotationBuffer{};
        BufferAllocation opacityBuffer{};
        BufferAllocation shBuffer{};
        BufferAllocation layerStyleIndexBuffer{};
        std::array<BufferAllocation, kFramesInFlight> layerStyleBuffers{};
        std::array<BufferAllocation, kFramesInFlight> sortedIndexBuffers{};
        std::vector<renderer::gsplat::HighQualityGaussianLayerSignature> layerSignatures;
        std::vector<renderer::gsplat::HighQualityGaussianLayerRange> layerRanges;
        std::vector<invisible_places::io::Float3> mergedLocalCenters;
        std::vector<std::uint32_t> sortedIndices;
        glm::mat4 lastSortedView{1.0F};
        bool hasSortedView = false;
        std::array<VkDescriptorSet, kFramesInFlight> descriptorSets{};
        std::uint32_t splatCount = 0;
        std::uint32_t layerCount = 0;
    };

    struct ExrExportResources {
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        VkRenderPass renderPass = VK_NULL_HANDLE;
        VkFramebuffer framebuffer = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        VkFence fence = VK_NULL_HANDLE;
        VkDescriptorSet compositeDescriptorSet = VK_NULL_HANDLE;
        VkPipeline pointDepthPipeline = VK_NULL_HANDLE;
        VkPipeline pointAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline pointConstantSimpleAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline pointFastBasicDepthPipeline = VK_NULL_HANDLE;
        VkPipeline pointFastBasicPipeline = VK_NULL_HANDLE;
        VkPipeline surfelDepthPipeline = VK_NULL_HANDLE;
        VkPipeline surfelAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline surfelConstantSimpleAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline compositePipeline = VK_NULL_HANDLE;
        ImageAllocation colorImage{};
        ImageAllocation depthImage{};
        ImageAllocation accumulationImage{};
        ImageAllocation revealageImage{};
        ImageAllocation emissiveImage{};
        ImageAllocation normalAccumulationImage{};
        ImageAllocation albedoAccumulationImage{};
        ImageAllocation linearDepthImage{};
        ImageAllocation normalImage{};
        ImageAllocation albedoImage{};
        BufferAllocation colorReadbackBuffer{};
        BufferAllocation depthReadbackBuffer{};
        BufferAllocation normalReadbackBuffer{};
        BufferAllocation albedoReadbackBuffer{};
    };

    void CreateInstance();
    void CreateSurface();
    void PickPhysicalDevice();
    void CreateLogicalDevice();
    void CreateSwapchain();
    void CreateImageViews();
    void CreateRenderPass();
    void CreatePresentRenderPass();
    void CreatePointDescriptorSetLayout();
    void CreateGaussianSplatDescriptorSetLayout();
    void CreateHighQualityGaussianSplatDescriptorSetLayout();
    void CreateCompositeDescriptorSetLayout();
    void CreatePostProcessDescriptorSetLayout();
    void CreateDescriptorPools();
    void CreatePostProcessSampler();
    void CreateUniformResources();
    void CreatePointPipelines();
    void CreateGaussianSplatPipeline();
    void CreateHighQualityGaussianSplatPipeline();
    void CreateCompositePipeline();
    void CreatePostProcessPipeline();
    void CreateExrExportResources(std::uint32_t width, std::uint32_t height);
    void CreateExrExportRenderPass(ExrExportResources* resources);
    void CreateExrExportPipelines(ExrExportResources* resources);
    void CreateFramebuffers();
    void CreatePresentFramebuffers();
    void CreateDepthResources();
    void CreateAccumulationResources();
    void CreateSceneColorResources();
    void CreateLinearDepthResources();
    void CreateCommandPool();
    void CreateCommandBuffers();
    void CreateSyncObjects();
    void CreateImGuiResources();
    void UploadImGuiFonts();
    void UpdatePointCloudDescriptorSets(ActivePointCloudResources* resources);
    void UpdatePointCloudDescriptorSet(
        ActivePointCloudResources* resources,
        std::size_t frameIndex,
        std::uint32_t imageIndex,
        VkImageView sceneDepthView);
    void UpdatePointCloudExrDescriptorSet(ActivePointCloudResources* resources, VkImageView sceneDepthView);
    void CreateOrUpdateCompositeDescriptorSet();
    void CreateOrUpdateCompositeDescriptorSet(
        VkDescriptorSet* descriptorSet,
        VkImageView accumulationView,
        VkImageView revealageView,
        VkImageView emissiveView);
    void CreateOrUpdateCompositeDescriptorSet(
        VkDescriptorSet* descriptorSet,
        VkImageView accumulationView,
        VkImageView revealageView,
        VkImageView emissiveView,
        VkImageView normalAccumulationView,
        VkImageView albedoAccumulationView);
    void CreateOrUpdatePostProcessDescriptorSets();
    void UpdateGaussianSplatDescriptorSets(ActiveGaussianSplatResources* resources);
    void UpdateGaussianSplatDescriptorSet(
        ActiveGaussianSplatResources* resources,
        std::size_t frameIndex,
        std::uint32_t imageIndex);
    void UpdateHighQualityGaussianDescriptorSet(std::size_t frameIndex);
    void RefreshHighQualityGaussianScene(std::size_t frameIndex);
    void CleanupSwapchain();
    void CleanupPointCloudResources(ActivePointCloudResources* resources);
    void CleanupGaussianSplatResources(ActiveGaussianSplatResources* resources);
    void CleanupHighQualityGaussianScene();
    void CleanupExrExportResources();
    void RecreateSwapchain();
    void RecordCommandBuffer(VkCommandBuffer commandBuffer, std::uint32_t imageIndex, std::size_t frameIndex);
    void RecordExrExportCommandBuffer(const PointCloudExrFrameRequest& request);
    [[nodiscard]] bool SceneImageNeedsRender(std::uint32_t imageIndex) const;
    [[nodiscard]] bool AnySceneImageNeedsRender() const;
    [[nodiscard]] bool ResolvePointCloudDrawPlan(
        const SceneRenderState::PointCloudLayerState& layer,
        bool forceFullSource,
        PointCloudDrawPlan* plan);
    [[nodiscard]] bool UploadPointCloudLayerStyle(
        const SceneRenderState::PointCloudLayerState& layer,
        const PointCloudDrawPlan& plan,
        std::size_t frameIndex,
        bool exrStyle);
    [[nodiscard]] bool RecordPointCloudLayerDraw(
        VkCommandBuffer commandBuffer,
        const SceneRenderState::PointCloudLayerState& layer,
        bool forceFullSource,
        VkPipeline spritePipeline,
        VkPipeline surfelPipeline,
        bool uploadStyle,
        std::size_t frameIndex,
        std::uint32_t imageIndex,
        bool exrStyle,
        std::uint32_t* recordedDrawPointCount = nullptr);
    void UpdateUniformBuffer(std::size_t frameIndex);
    void UploadFrameUniforms(std::size_t frameIndex, std::uint32_t width, std::uint32_t height);

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
    [[nodiscard]] ImageAllocation CreateAttachmentImage(
        std::uint32_t width,
        std::uint32_t height,
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
    VkRenderPass presentRenderPass_ = VK_NULL_HANDLE;
    VkPipelineLayout pointPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout gaussianSplatPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout highQualityGaussianSplatPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout compositePipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout postProcessPipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pointDepthPrepassPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointConstantSimpleAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointOpaqueHardDiscPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointFastBasicPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelDepthPrepassPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelConstantSimpleAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelOpaqueHardDiscPipeline_ = VK_NULL_HANDLE;
    VkPipeline gaussianSplatPipeline_ = VK_NULL_HANDLE;
    VkPipeline highQualityGaussianSplatPipeline_ = VK_NULL_HANDLE;
    VkPipeline compositePipeline_ = VK_NULL_HANDLE;
    VkPipeline postProcessPipeline_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout pointDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout gaussianSplatDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout highQualityGaussianSplatDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout compositeDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout postProcessDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorPool gaussianSplatDescriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorPool imguiDescriptorPool_ = VK_NULL_HANDLE;
    std::vector<VkDescriptorSet> compositeDescriptorSets_;
    std::vector<VkDescriptorSet> postProcessDescriptorSets_;
    VkSampler postProcessSampler_ = VK_NULL_HANDLE;
    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    VkFormat depthFormat_ = VK_FORMAT_UNDEFINED;
    VkFormat accumulationFormat_ = VK_FORMAT_UNDEFINED;
    VkFormat revealageFormat_ = VK_FORMAT_UNDEFINED;
    VkFormat linearDepthFormat_ = VK_FORMAT_R32_SFLOAT;

    std::vector<VkImage> swapchainImages_;
    std::vector<VkImageView> imageViews_;
    std::vector<VkFramebuffer> framebuffers_;
    std::vector<VkFramebuffer> presentFramebuffers_;
    std::vector<VkFence> swapchainImagesInFlight_;

    std::uint32_t graphicsQueueFamily_ = 0;
    std::uint32_t presentQueueFamily_ = 0;
    VkFormat swapchainImageFormat_ = VK_FORMAT_UNDEFINED;
    std::uint32_t swapchainWidth_ = 0;
    std::uint32_t swapchainHeight_ = 0;
    bool framebufferResized_ = false;
    bool enablePortabilitySubset_ = false;
    bool uiFrameBegun_ = false;

    std::array<FrameResources, kFramesInFlight> frameResources_{};
    std::size_t currentFrameIndex_ = 0;
    std::vector<ImageAllocation> depthImages_;
    std::vector<ImageAllocation> sceneColorImages_;
    std::vector<ImageAllocation> accumulationImages_;
    std::vector<ImageAllocation> revealageImages_;
    std::vector<ImageAllocation> emissiveImages_;
    std::vector<ImageAllocation> linearDepthImages_;
    std::vector<ActivePointCloudResources> pointCloudResources_;
    std::vector<ActiveGaussianSplatResources> gaussianSplatResources_;
    HighQualityGaussianSceneResources highQualityGaussianScene_{};
    ExrExportResources exrExportResources_{};
    bool highQualityGaussianSceneDirty_ = true;
    SceneRenderState renderState_{};
    ViewportDiagnostics diagnostics_{};
    bool diagnosticsEnabled_ = false;
    bool liveSceneRenderingEnabled_ = true;
    bool sceneCachingEnabled_ = false;
    std::uint64_t sceneRevision_ = 1;
    std::vector<std::uint64_t> sceneImageRevisions_;
    bool diagnosticsTimingInitialized_ = false;
    double diagnosticsFpsWindowMs_ = 0.0;
    std::uint32_t diagnosticsFpsWindowFrames_ = 0;
    float pointSizeRangeMin_ = 1.0F;
    float pointSizeRangeMax_ = 64.0F;
};

}  // namespace invisible_places::renderer::core
