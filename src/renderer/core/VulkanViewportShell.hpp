#pragma once

#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
#include "output/ExrWriter.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/gsplat/HighQualityGaussianScene.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "water/WaterSurfaceCache.hpp"
#include "water/WaterFlow.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <imgui.h>
#include <vulkan/vulkan.h>

struct GLFWwindow;

namespace invisible_places::renderer::core {

inline constexpr std::uint64_t kWaterSurfaceUploadStagingLimitBytes =
    64ULL * 1024ULL * 1024ULL;

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
    std::uint32_t rainActiveParticleCount = 0U;
    std::uint32_t rainImpactOverflowCount = 0U;
    std::uint32_t rainEventsEmittedThisFrame = 0U;
    std::uint32_t rainSandImpactsThisFrame = 0U;
    std::uint32_t rainRockImpactsThisFrame = 0U;
    std::uint32_t rainVegetationImpactsThisFrame = 0U;
    std::uint64_t rainCollisionCacheRevision = 0U;
    std::uint64_t rainCollisionUploadRevision = 0U;
    std::uint64_t waterSurfaceCacheRevision = 0U;
    std::uint64_t waterSurfaceUploadRevision = 0U;
    std::uint64_t waterSurfaceGpuBytes = 0U;
    std::uint64_t waterSurfacePeakStagingBytes = 0U;
    std::uint32_t waterSurfaceSurfelCellCount = 0U;
    std::uint32_t waterSurfaceTableCapacity = 0U;
    std::uint32_t waterSurfaceMaximumProbeCount = 0U;
    std::uint32_t waterSurfacePreprocessDispatchCount = 0U;
    bool waterSurfacePreprocessPending = false;
    std::uint32_t rainParticleCapacity = 0U;
    std::uint32_t rainEventCapacity = 0U;
    std::uint64_t pointDescriptorGenerationsAllocated = 0U;
    std::uint64_t pointDescriptorGenerationsRetired = 0U;
};

struct WaterEffectFramePublicationDiagnostics {
    std::uint64_t requestedGeneration = 0U;
    std::array<std::uint64_t, 2> liveFrameGenerations{};
    std::uint64_t exrGeneration = 0U;
    bool liveBuffersDistinct = false;
    bool exrBufferDistinct = false;
    // Seepage topology is published as an immutable descriptor generation.
    // Keeping the per-swapchain-image values visible makes startup and density
    // switch smoke tests able to prove that every submitted descriptor refers
    // to the same settled topology.
    std::uint64_t topologyGeneration = 0U;
    std::array<std::vector<std::uint64_t>, 2> liveDescriptorGenerations{};
    std::uint64_t exrDescriptorGeneration = 0U;
    std::uint32_t descriptorGenerationMismatchCount = 0U;
    bool exrParamsPublicationRequired = false;
    bool descriptorGenerationReady = false;
    // Retained for source compatibility with older diagnostics consumers.
    // Activation is generation-based now, so this is always zero.
    std::uint32_t activationFramesRemaining = 0U;
};

struct LiveSceneReadback {
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;
    std::uint32_t submittedFrameSlot = 0U;
    std::uint32_t swapchainImageIndex = 0U;
    std::vector<std::uint8_t> colorRgba8;
    std::vector<float> linearDepth;
};

struct SceneRenderState {
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::mat4 viewProjection{1.0F};
    glm::vec3 cameraPosition{0.0F, 0.0F, 1.0F};
    glm::vec4 backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    bool proResAlphaPreviewEnabled = false;
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
    invisible_places::water::RainRuntimeSettings rainSettings{};
    invisible_places::water::WaterRainVisualSettings rainVisual{};
    glm::vec3 rainSpawnCentre{0.0F, 0.0F, 0.0F};
    renderer::pointcloud::PointCloudRendererMode pointCloudRendererMode =
        renderer::pointcloud::PointCloudRendererMode::Beauty;

    struct PointCloudLayerState {
        std::size_t layerId = 0;
        renderer::pointcloud::PointCloudStyleState style{};
        std::vector<invisible_places::io::ScalarFieldStats> scalarFields;
        bool hasSourceRgb = true;
        std::uint32_t drawPointCount = 0;
        renderer::pointcloud::PointCloudDensityCompensation densityCompensation{};
        invisible_places::water::WaterSurfaceRole rainCollisionRole =
            invisible_places::water::WaterSurfaceRole::None;
    };

    struct GaussianSplatLayerState {
        std::size_t layerId = 0;
        renderer::gsplat::GaussianSplatStyleState style{};
        glm::mat4 localToWorld{1.0F};
    };

    std::vector<PointCloudLayerState> pointCloudLayers;
    std::vector<GaussianSplatLayerState> gaussianSplatLayers;
};

enum class PointCloudExrReadbackMask : std::uint32_t {
    None = 0U,
    Color = 1U << 0U,
    Depth = 1U << 1U,
    Normal = 1U << 2U,
    Albedo = 1U << 3U,
    All = (1U << 0U) | (1U << 1U) | (1U << 2U) | (1U << 3U),
};

inline PointCloudExrReadbackMask operator|(
    PointCloudExrReadbackMask lhs,
    PointCloudExrReadbackMask rhs) {
    using Underlying = std::underlying_type_t<PointCloudExrReadbackMask>;
    return static_cast<PointCloudExrReadbackMask>(
        static_cast<Underlying>(lhs) | static_cast<Underlying>(rhs));
}

inline PointCloudExrReadbackMask operator&(
    PointCloudExrReadbackMask lhs,
    PointCloudExrReadbackMask rhs) {
    using Underlying = std::underlying_type_t<PointCloudExrReadbackMask>;
    return static_cast<PointCloudExrReadbackMask>(
        static_cast<Underlying>(lhs) & static_cast<Underlying>(rhs));
}

inline bool HasPointCloudExrReadback(
    PointCloudExrReadbackMask mask,
    PointCloudExrReadbackMask flag) {
    return (mask & flag) != PointCloudExrReadbackMask::None;
}

enum class PointCloudExrFrameStatus {
    Idle,
    Running,
    Ready,
};

struct PointCloudExrFrameRequest {
    SceneRenderState renderState{};
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    bool previewDensity = true;
    PointCloudExrReadbackMask readbackMask = PointCloudExrReadbackMask::All;
};

struct PointHighlightStyle {
    std::array<float, 4> color{0.45F, 0.88F, 1.0F, 0.86F};
    bool pulseAlpha = true;
};

struct DynamicMeshFlowGpuUploadResult {
    std::uint32_t pointCount = 0;
    std::uint32_t particleCount = 0;
    std::uint32_t routeAnchorCount = 0;
    std::uint32_t visibleSampleCount = 0;
    double solveMilliseconds = 0.0;
    double staticUploadMilliseconds = 0.0;
    double liveUploadMilliseconds = 0.0;
    double dispatchMilliseconds = 0.0;
    bool reusedStaticBuffers = false;
    bool asynchronousDispatch = false;
};

// One live Mesh Flow update. Capacity/history are settled allocation controls;
// every other member is copied into the parameter ring and may change each
// frame without replacing descriptors or simulation storage.
struct DynamicMeshFlowGpuFrameRequest {
    invisible_places::water::WaterDynamicMeshFlowSettings settings{};
    float activity = 1.0F;
    float moisture = 0.0F;
    float timeSeconds = 0.0F;
    float deltaSeconds = 1.0F / 30.0F;
    bool resetSimulation = false;
};

struct DynamicMeshFlowGpuUpdateResult {
    std::uint32_t pointCount = 0U;
    std::uint32_t particleCapacity = 0U;
    std::uint32_t historyLength = 0U;
    std::uint32_t activeParticles = 0U;
    std::uint32_t contactEvents = 0U;
    std::uint32_t entryCandidateCount = 0U;
    std::uint32_t rainSeedCount = 0U;
    std::uint32_t usedRainSeedParticleCount = 0U;
    std::uint32_t routeSampleCount = 0U;
    float routeWithinGroundBoundsFraction = 0.0F;
    float routeWithinSurfaceToleranceFraction = 0.0F;
    float maximumRenderedSegmentMeters = 0.0F;
    std::uint32_t longSegmentCount = 0U;
    std::uint32_t unexplainedVerticalJumpCount = 0U;
    float medianTangentDownhillAlignment = 0.0F;
    std::uint64_t allocationRevision = 0U;
    std::uint64_t parameterRevision = 0U;
    std::uint64_t descriptorGeneration = 0U;
    std::uint64_t sharedGroundUploadRevision = 0U;
    std::uint32_t dispatchCount = 0U;
    double allocationMilliseconds = 0.0;
    double parameterUploadMilliseconds = 0.0;
    double dispatchMilliseconds = 0.0;
    bool allocatedThisUpdate = false;
    bool parametersOnly = false;
    bool asynchronousDispatch = false;
    bool sharedGroundReady = false;
};

struct DynamicMeshFlowContactGpuView {
    VkBuffer eventBuffer = VK_NULL_HANDLE;
    VkDeviceSize eventOffset = 0U;
    VkDeviceSize eventRange = 0U;
    VkBuffer gridBuffer = VK_NULL_HANDLE;
    VkDeviceSize gridOffset = 0U;
    VkDeviceSize gridRange = 0U;
    std::uint32_t eventCapacity = 0U;
    std::uint32_t gridMask = 0U;
    float gridCellSizeMeters = 0.10F;
    std::uint64_t allocationRevision = 0U;
    std::uint64_t descriptorGeneration = 0U;
    bool valid = false;
};

// Immutable view of the shared, orientation-independent ROCK/SAND surface
// table. The buffer contains WaterGpuSurfaceSurfelSlot entries in the hash
// layout produced by BuildWaterSurfaceGpuData(). It remains owned by the
// viewport and is valid until the next changed surface-cache upload or clear.
struct WaterSurfaceFlowGpuView {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceSize offset = 0U;
    VkDeviceSize range = 0U;
    std::uint32_t tableMask = 0U;
    std::uint32_t maximumProbeCount = 0U;
    std::uint32_t occupiedCellCount = 0U;
    float resolutionMeters = invisible_places::water::kWaterSurfaceResolutionMeters;
    std::uint64_t cacheRevision = 0U;
    std::uint64_t uploadRevision = 0U;
    const invisible_places::water::WaterSurfaceCacheIdentity* cacheIdentity = nullptr;
    bool valid = false;
    bool preprocessingComplete = false;
};

// Immutable view of the schema-4 Ground hash table shared by Mesh Flow. The
// viewport owns the buffer; callers must treat an upload-revision change as a
// settled topology change.
struct WaterGroundFlowGpuView {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceSize offset = 0U;
    VkDeviceSize range = 0U;
    // Cache-owned, vegetation-supported cells in the highest +X band of each
    // connected Ground component. Mesh Flow samples this immutable table
    // directly; ordinary authored Flow sources never enter the simulation.
    VkBuffer entryCandidateBuffer = VK_NULL_HANDLE;
    VkDeviceSize entryCandidateOffset = 0U;
    VkDeviceSize entryCandidateRange = 0U;
    std::uint32_t entryCandidateCount = 0U;
    std::uint32_t tableMask = 0U;
    std::uint32_t maximumProbeCount = 0U;
    std::uint32_t occupiedCellCount = 0U;
    float resolutionMeters = invisible_places::water::kWaterSurfaceResolutionMeters;
    std::uint64_t cacheRevision = 0U;
    std::uint64_t uploadRevision = 0U;
    const invisible_places::water::WaterSurfaceCacheIdentity* cacheIdentity = nullptr;
    invisible_places::io::Bounds3f bounds{};
    bool valid = false;
};

struct WaterFlowGpuSourceRequest {
    std::uint32_t sourceId = 0U;
    std::uint64_t sourceRevision = 0U;
    invisible_places::water::WaterFlowGpuInputKind inputKind =
        invisible_places::water::WaterFlowGpuInputKind::SampledAnchors;
    std::span<const invisible_places::water::WaterOverlayPoint> sampledAnchors{};
    std::span<const invisible_places::io::Float3> controlPoints{};
    // Preferred source-local input. Keeping this immutable object shared lets
    // queued revisions replace one another without copying dense generated
    // branches. The legacy spans remain valid compatibility inputs.
    std::shared_ptr<const invisible_places::water::WaterFlowGpuCompactSourceInput>
        compactSourceInput;
    invisible_places::water::WaterFlowTrailSettings settings{};
    bool useSurfaceGuide = false;
};

struct WaterFlowGpuSourceDiagnostics {
    std::uint32_t sourceId = 0U;
    std::uint64_t requestedRevision = 0U;
    std::uint64_t completedRevision = 0U;
    std::uint64_t surfaceUploadRevision = 0U;
    std::uint64_t bytesTransferred = 0U;
    std::uint64_t computeDescriptorGeneration = 0U;
    std::uint64_t pointDescriptorGeneration = 0U;
    std::uint64_t residentBytes = 0U;
    std::uint32_t computeDispatchCount = 0U;
    std::uint32_t pointCapacity = 0U;
    std::uint32_t activePointCount = 0U;
    bool pending = false;
    bool usingSurfaceGuide = false;
};

struct WaterFlowGpuSourceUploadResult {
    invisible_places::water::WaterFlowGpuOutputLayout layout{};
    WaterFlowGpuSourceDiagnostics diagnostics{};
    bool accepted = false;
    bool reusedOutputCapacity = false;
    bool asynchronousDispatch = false;
};

class VulkanViewportShell {
  public:
    struct ImGuiPreviewImageTexture {
        ImTextureID textureId = ImTextureID_Invalid;
        std::uint32_t width = 0;
        std::uint32_t height = 0;

        [[nodiscard]] bool Valid() const {
            return textureId != ImTextureID_Invalid && width > 0 && height > 0;
        }
    };

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
    // A density transaction can bracket several remove/upload/attachment
    // operations so executable point command buffers are settled and reset
    // exactly once. Calls may be nested; only the outer pair owns the settle.
    void BeginPointCloudMutationBatch();
    void EndPointCloudMutationBatch();
    void SetSmokeFailNextPointCloudUploadAfterPartialAllocation(bool enabled = true);
    void SetSmokeFailNextWaterFlowUploadAfterPartialAllocation(bool enabled = true);
    [[nodiscard]] bool HasPointCloudResources(std::size_t layerId) const;
    // Geometry residency includes active/pending/retired position, colour,
    // normal, scalar, and sampled-index buffers. Compact effect/style and
    // descriptor allocations are intentionally excluded from this bound.
    [[nodiscard]] std::uint64_t PointCloudLayerResidentBytes(std::size_t layerId) const;
    [[nodiscard]] std::uint64_t PointCloudResidentBytes() const;
    [[nodiscard]] std::uint64_t PointCloudPeakResidentBytes() const;
    [[nodiscard]] std::uint64_t LastPointCloudMutationBatchPeakResidentBytes() const;
    [[nodiscard]] std::uint32_t LastPointCloudMutationBatchWaitCount() const;
    void UploadPointCloudScalarFields(
        std::size_t layerId,
        const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
        const std::vector<float>& scalarFieldValues);
    void UploadPointCloudScalarFieldValues(
        std::size_t layerId,
        std::size_t scalarFieldIndex,
        std::span<const float> values);
    void UploadSparseWaterRippleMembership(
        std::size_t layerId,
        const std::vector<invisible_places::water::WaterRippleRuntimeMembership>& memberships,
        const std::vector<invisible_places::water::WaterRippleRuntimeParams>& params);
    [[nodiscard]] DynamicMeshFlowGpuUploadResult UploadDynamicMeshFlowPreviewPointCloud(
        std::size_t layerId,
        const invisible_places::water::MeshSurfaceCache& cache,
        const std::vector<invisible_places::water::WaterEmitter>& emitters,
        const invisible_places::water::WaterDynamicMeshFlowSettings& settings,
        invisible_places::water::WaterTrailBuildQuality quality);
    [[nodiscard]] DynamicMeshFlowGpuUpdateResult UpdateDynamicMeshFlowGpuSimulation(
        std::size_t layerId,
        const DynamicMeshFlowGpuFrameRequest& request);
    [[nodiscard]] DynamicMeshFlowContactGpuView DynamicMeshFlowContactView(
        std::size_t layerId) const;
    void ClearDynamicMeshFlowGpuSimulation(std::size_t layerId);
    [[nodiscard]] WaterFlowGpuSourceUploadResult UploadWaterFlowGpuSource(
        std::size_t layerId,
        const WaterFlowGpuSourceRequest& request);
    [[nodiscard]] WaterFlowGpuSourceDiagnostics WaterFlowGpuSourceState(
        std::size_t layerId) const;
    void RemoveWaterFlowGpuSource(std::size_t layerId);
    void UpdateSparseWaterRippleParams(
        std::size_t layerId,
        const std::vector<invisible_places::water::WaterRippleRuntimeParams>& params);
    void UploadWaterSeepageTopology(
        std::size_t layerId,
        const invisible_places::water::WaterSeepageSpatialGrid& grid);
    void UpdateWaterSeepageParams(
        std::size_t layerId,
        const invisible_places::water::WaterSeepageSpatialGrid& grid);
    [[nodiscard]] std::size_t SparseWaterRippleEffectCount(std::size_t layerId) const;
    [[nodiscard]] std::size_t SparseWaterRippleRegionCount(std::size_t layerId) const;
    [[nodiscard]] std::uint64_t SparseWaterRippleMembershipUploadRevision(std::size_t layerId) const;
    [[nodiscard]] std::uint64_t SparseWaterRippleParamsUploadRevision(std::size_t layerId) const;
    [[nodiscard]] WaterEffectFramePublicationDiagnostics SparseWaterRippleParamsPublicationState(
        std::size_t layerId) const;
    [[nodiscard]] std::size_t WaterSeepageNodeCount(std::size_t layerId) const;
    [[nodiscard]] std::size_t WaterSeepageOccupiedCellCount(std::size_t layerId) const;
    [[nodiscard]] std::size_t WaterSeepageNodeReferenceCount(std::size_t layerId) const;
    [[nodiscard]] std::uint64_t WaterSeepageTopologyUploadRevision(std::size_t layerId) const;
    [[nodiscard]] std::uint64_t WaterSeepageParamsUploadRevision(std::size_t layerId) const;
    [[nodiscard]] WaterEffectFramePublicationDiagnostics WaterSeepageParamsPublicationState(
        std::size_t layerId) const;
    void UpdatePointBudget(std::size_t layerId, const std::vector<std::uint32_t>& sampledIndices);
    void UpdateInteractivePointSampleBuffer(
        std::size_t layerId,
        const std::vector<std::uint32_t>& sampledIndices,
        bool includeSurfelIndices = true);
    void UploadPointHighlightIndices(
        std::size_t layerId,
        std::uint64_t key,
        const std::vector<std::uint32_t>& indices,
        const PointHighlightStyle& style = {});
    void ClearPointHighlightIndices(std::size_t layerId, std::uint64_t key);
    void ClearPointHighlights(std::size_t layerId);
    void RemovePointCloud(std::size_t layerId);
    void ClearPointClouds();
    void UploadWaterSurfaceCache(const invisible_places::water::WaterSurfaceCache& cache);
    void ClearWaterSurfaceCache();
    [[nodiscard]] WaterSurfaceFlowGpuView WaterSurfaceFlowView() const;
    [[nodiscard]] WaterGroundFlowGpuView WaterGroundFlowView() const;
    [[nodiscard]] bool WaterSurfaceUploadPending() const;
    [[nodiscard]] std::uint64_t WaterSurfaceUploadRevision() const;
    void UploadGaussianSplats(std::size_t layerId, const invisible_places::io::LoadedGaussianSplat& splats);
    void RemoveGaussianSplats(std::size_t layerId);
    void ClearGaussianSplats();
    [[nodiscard]] invisible_places::output::HalfRgbaExrImage RenderPointCloudExrFrame(
        const PointCloudExrFrameRequest& request);
    // Synchronous smoke/diagnostic readback of the most recently presented
    // live scene attachments. This deliberately does not use the independent
    // EXR descriptor/render path.
    [[nodiscard]] LiveSceneReadback ReadLiveSceneFrame();
    [[nodiscard]] bool BeginPointCloudExrFrame(const PointCloudExrFrameRequest& request);
    [[nodiscard]] PointCloudExrFrameStatus PollPointCloudExrFrame();
    [[nodiscard]] invisible_places::output::HalfRgbaExrImage CompletePointCloudExrFrame();
    void CancelPointCloudExrFrame();
    // Diagnostics for the most recently recorded EXR frame: how many
    // point-cloud layers actually recorded draw commands, and their summed
    // draw point count. Lets export paths verify the base cloud was drawn.
    [[nodiscard]] std::uint32_t ExrLastRecordedLayerCount() const {
        return exrLastRecordedLayerCount_;
    }
    [[nodiscard]] std::uint64_t ExrLastRecordedPointCount() const {
        return exrLastRecordedPointCount_;
    }
    [[nodiscard]] ImGuiPreviewImageTexture UploadImGuiPreviewImageTexture(
        std::uint32_t width,
        std::uint32_t height,
        const std::vector<std::uint8_t>& rgba);
    void ClearImGuiPreviewImageTexture();

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
    // Smoke diagnostics can arm the existing live depth prepass even when the
    // selected visual renderer does not otherwise need it. Normal viewport
    // rendering pays no additional point-cloud pass.
    void SetLiveSceneReadbackCaptureEnabled(bool enabled);
    [[nodiscard]] bool LiveSceneReadbackCaptureEnabled() const {
        return liveSceneReadbackCaptureEnabled_;
    }
    void SetSceneCachingEnabled(bool enabled);
    [[nodiscard]] bool SceneCachingEnabled() const { return sceneCachingEnabled_; }

  private:
    static constexpr std::size_t kFramesInFlight = 2U;
    static constexpr std::size_t kDynamicMeshFlowLiveSlots = 4U;

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
        struct PointHighlightResources {
            std::uint64_t key = 0;
            BufferAllocation indexBuffer{};
            BufferAllocation surfelIndexBuffer{};
            std::array<BufferAllocation, kFramesInFlight> styleBuffers{};
            std::array<std::vector<VkDescriptorSet>, kFramesInFlight> descriptorSets{};
            std::array<std::vector<std::uint64_t>, kFramesInFlight>
                seepageDescriptorGenerations{};
            std::uint32_t indexCount = 0;
            PointHighlightStyle style{};
        };

        struct WaterFlowRetiredOutputResources {
            BufferAllocation positionBuffer{};
            BufferAllocation positionStorageBuffer{};
            BufferAllocation colorBuffer{};
            BufferAllocation normalBuffer{};
            BufferAllocation scalarFieldBuffer{};
            std::uint32_t pointCapacity = 0U;
            std::uint32_t outstandingFrameMask = 0U;
            bool outstandingExr = false;
        };

        std::size_t layerId = 0;
        // Base, highlight, and EXR point descriptors are one immutable
        // generation owned by this pool. Shape changes replace the complete
        // pool after all command buffers referencing the prior generation are
        // retired.
        VkDescriptorPool pointDescriptorPool = VK_NULL_HANDLE;
        std::uint64_t pointDescriptorGeneration = 0U;
        BufferAllocation positionBuffer{};
        BufferAllocation positionStorageBuffer{};
        BufferAllocation colorBuffer{};
        BufferAllocation normalBuffer{};
        BufferAllocation scalarFieldBuffer{};
        BufferAllocation sparseRippleRangeBuffer{};
        BufferAllocation sparseRippleMembershipBuffer{};
        // Animated Ripple and Seepage state is published only after the target
        // frame fence signals. Dedicated EXR snapshots keep asynchronous export
        // reads isolated from live viewport updates.
        std::array<BufferAllocation, kFramesInFlight> sparseRippleParamsBuffers{};
        BufferAllocation sparseRippleExrParamsBuffer{};
        BufferAllocation seepageNodeBuffer{};
        std::array<BufferAllocation, kFramesInFlight> seepageParamsBuffers{};
        BufferAllocation seepageExrParamsBuffer{};
        BufferAllocation seepageHashCellBuffer{};
        BufferAllocation seepageNodeReferenceBuffer{};
        // Mesh Flow owns only its fixed-capacity simulation/output storage.
        // The schema-4 Ground table is borrowed immutably from RainGpuResources
        // and is kept alive by its upload revision until these descriptors
        // retire.
        BufferAllocation dynamicMeshFlowParticleBuffer{};
        BufferAllocation dynamicMeshFlowContactEventBuffer{};
        BufferAllocation dynamicMeshFlowContactGridBuffer{};
        BufferAllocation dynamicMeshFlowCellBuffer{};
        BufferAllocation dynamicMeshFlowGridBuffer{};
        std::array<BufferAllocation, kDynamicMeshFlowLiveSlots> dynamicMeshFlowUniformBuffers{};
        std::array<BufferAllocation, kDynamicMeshFlowLiveSlots> dynamicMeshFlowEmitterBuffers{};
        std::array<BufferAllocation, kDynamicMeshFlowLiveSlots> dynamicMeshFlowAttractorBuffers{};
        std::array<BufferAllocation, kDynamicMeshFlowLiveSlots> dynamicMeshFlowCounterBuffers{};
        std::array<BufferAllocation, kFramesInFlight> styleBuffers{};
        BufferAllocation exrStyleBuffer{};
        std::array<std::vector<VkDescriptorSet>, kFramesInFlight> descriptorSets{};
        std::array<std::vector<std::uint64_t>, kFramesInFlight>
            seepageDescriptorGenerations{};
        std::array<VkDescriptorSet, kDynamicMeshFlowLiveSlots> dynamicMeshFlowDescriptorSets{};
        std::array<VkFence, kDynamicMeshFlowLiveSlots> dynamicMeshFlowDispatchFences{};
        std::array<VkCommandBuffer, kDynamicMeshFlowLiveSlots> dynamicMeshFlowCommandBuffers{};
        VkDescriptorPool dynamicMeshFlowDescriptorPool = VK_NULL_HANDLE;
        BufferAllocation waterFlowSourceUniformBuffer{};
        BufferAllocation waterFlowSourceInputBuffer{};
        BufferAllocation waterFlowSourceBranchBuffer{};
        BufferAllocation waterFlowPendingPositionBuffer{};
        BufferAllocation waterFlowPendingPositionStorageBuffer{};
        BufferAllocation waterFlowPendingColorBuffer{};
        BufferAllocation waterFlowPendingNormalBuffer{};
        BufferAllocation waterFlowPendingScalarFieldBuffer{};
        // The source-compute descriptor is an immutable one-set generation.
        // Its pool is retired only after the dispatch fence has signalled and
        // the command buffer retaining MoltenVK's resolved bindings is freed.
        VkDescriptorPool waterFlowSourceDescriptorPool = VK_NULL_HANDLE;
        VkDescriptorSet waterFlowSourceDescriptorSet = VK_NULL_HANDLE;
        VkFence waterFlowSourceDispatchFence = VK_NULL_HANDLE;
        VkCommandBuffer waterFlowSourceCommandBuffer = VK_NULL_HANDLE;
        VkDescriptorSet exrDescriptorSet = VK_NULL_HANDLE;
        std::uint64_t seepageExrDescriptorGeneration = 0U;
        BufferAllocation sampledIndexBuffer{};
        BufferAllocation sampledSurfelIndexBuffer{};
        BufferAllocation interactiveSampledIndexBuffer{};
        BufferAllocation interactiveSurfelIndexBuffer{};
        std::uint32_t pointCount = 0;
        // GPU Flow keeps a geometrically settled result independent from the
        // current point-budget draw subset. Unlike activePointCount, this does
        // not shrink when sampled indices are installed.
        std::uint32_t waterFlowSettledPointCount = 0;
        std::uint32_t activePointCount = 0;
        std::uint32_t interactiveSampledIndexCount = 0;
        std::uint32_t scalarFieldCount = 0;
        std::uint32_t sparseRippleMembershipCount = 0;
        std::uint32_t sparseRippleParamCount = 0;
        std::uint64_t sparseRippleMembershipUploadRevision = 0;
        std::uint64_t sparseRippleParamsUploadRevision = 0;
        std::uint64_t sparseRippleParamsGeneration = 0;
        std::array<std::uint64_t, kFramesInFlight> sparseRippleParamsFrameGenerations{};
        std::uint64_t sparseRippleParamsExrGeneration = 0;
        std::vector<std::byte> pendingSparseRippleParams;
        std::uint32_t seepageNodeCount = 0;
        std::uint32_t seepageHashCellCapacity = 0;
        std::uint32_t seepageOccupiedCellCount = 0;
        std::uint32_t seepageNodeReferenceCount = 0;
        std::uint32_t seepageHashProbeLimit = 1;
        bool seepageUsesConnectedSupport = false;
        float seepageCellSizeMeters = 0.50F;
        invisible_places::io::Bounds3f seepageUnionBounds{};
        std::uint64_t seepageTopologyUploadRevision = 0;
        std::uint64_t seepageParamsUploadRevision = 0;
        std::uint64_t seepageParamsGeneration = 0;
        std::array<std::uint64_t, kFramesInFlight> seepageParamsFrameGenerations{};
        std::uint64_t seepageParamsExrGeneration = 0;
        std::uint64_t seepageTopologyGeneration = 0U;
        std::vector<std::byte> pendingSeepageParams;
        std::vector<std::uint32_t> seepageTopologyNodeIds;
        const invisible_places::water::MeshSurfaceCache* dynamicMeshFlowCacheIdentity = nullptr;
        std::uint32_t dynamicMeshFlowParticleCount = 0;
        std::uint32_t dynamicMeshFlowRouteAnchorCount = 0;
        std::uint32_t dynamicMeshFlowVisibleSampleCount = 0;
        std::uint32_t dynamicMeshFlowGridWidth = 0;
        std::uint32_t dynamicMeshFlowGridHeight = 0;
        std::uint32_t dynamicMeshFlowEmitterCapacity = 0;
        std::uint32_t dynamicMeshFlowAttractorCapacity = 0;
        std::size_t dynamicMeshFlowCellCount = 0;
        int dynamicMeshFlowMinCellX = 0;
        int dynamicMeshFlowMinCellY = 0;
        float dynamicMeshFlowCellSize = 0.0F;
        std::size_t dynamicMeshFlowNextLiveSlot = 0;
        invisible_places::water::WaterSurfaceCacheIdentity dynamicMeshFlowGroundIdentity{};
        std::uint64_t dynamicMeshFlowGroundUploadRevision = 0U;
        std::uint64_t dynamicMeshFlowAllocationRevision = 0U;
        std::uint64_t dynamicMeshFlowParameterRevision = 0U;
        std::uint64_t dynamicMeshFlowDescriptorGeneration = 0U;
        std::uint32_t dynamicMeshFlowDispatchCount = 0U;
        std::uint32_t dynamicMeshFlowResetEpoch = 1U;
        std::uint32_t dynamicMeshFlowLastActiveParticleCount = 0U;
        std::uint32_t dynamicMeshFlowLastContactEventCount = 0U;
        std::uint32_t dynamicMeshFlowLastRainSeedCount = 0U;
        std::uint32_t dynamicMeshFlowLastRainSeedParticleCount = 0U;
        std::uint32_t dynamicMeshFlowLastRouteSampleCount = 0U;
        std::uint32_t dynamicMeshFlowLastRouteWithinBoundsCount = 0U;
        std::uint32_t dynamicMeshFlowLastRouteNearSurfaceCount = 0U;
        std::uint32_t dynamicMeshFlowLastLongSegmentCount = 0U;
        std::uint32_t dynamicMeshFlowLastVerticalJumpCount = 0U;
        float dynamicMeshFlowLastMaximumSegmentMeters = 0.0F;
        std::array<std::uint32_t, 8>
            dynamicMeshFlowLastAlignmentHistogram{};
        std::uint32_t dynamicMeshFlowEventCapacity = 0U;
        std::uint32_t dynamicMeshFlowContactGridMask = 0U;
        float dynamicMeshFlowContactGridCellSizeMeters = 0.10F;
        std::uint32_t waterFlowSourceId = 0U;
        std::uint32_t waterFlowSourceInputCapacity = 0U;
        std::uint32_t waterFlowSourceBranchCapacity = 0U;
        std::uint32_t waterFlowSourceDispatchCount = 0U;
        std::uint64_t waterFlowSourceRequestedRevision = 0U;
        std::uint64_t waterFlowSourceCompletedRevision = 0U;
        std::uint64_t waterFlowSourceSurfaceUploadRevision = 0U;
        std::uint64_t waterFlowSourceBytesTransferred = 0U;
        std::uint64_t waterFlowSourceDescriptorGeneration = 0U;
        invisible_places::water::WaterFlowGpuOutputLayout waterFlowPendingLayout{};
        std::uint32_t waterFlowSparePointCapacity = 0U;
        std::uint64_t waterFlowPendingRevision = 0U;
        std::uint32_t waterFlowPendingSourceId = 0U;
        invisible_places::water::WaterFlowGpuInputKind waterFlowQueuedInputKind =
            invisible_places::water::WaterFlowGpuInputKind::SampledAnchors;
        std::shared_ptr<const invisible_places::water::WaterFlowGpuCompactSourceInput>
            waterFlowQueuedCompactSourceInput;
        invisible_places::water::WaterFlowTrailSettings waterFlowQueuedSettings{};
        std::uint64_t waterFlowQueuedRevision = 0U;
        std::uint32_t waterFlowQueuedSourceId = 0U;
        bool waterFlowSourceActive = false;
        bool waterFlowSourceDispatchPending = false;
        bool waterFlowSourceQueued = false;
        bool waterFlowSourceUseSurfaceGuide = false;
        bool waterFlowQueuedUseSurfaceGuide = false;
        bool usingSampledIndices = false;
        bool hasSourceRgb = false;
        bool hasNormals = false;
        std::vector<std::uint32_t> activeSparseRipplePointIndices;
        std::vector<PointHighlightResources> highlights;
        std::vector<WaterFlowRetiredOutputResources> waterFlowRetiredOutputs;
        std::uint32_t waterFlowDeleteOutstandingFrameMask = 0U;
        bool waterFlowDeletePending = false;
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
        // The EXR composite descriptor binds this generation's attachments,
        // so it is retired with the attachments instead of accumulating in
        // the long-lived viewport descriptor pool.
        VkDescriptorPool compositeDescriptorPool = VK_NULL_HANDLE;
        VkDescriptorSet compositeDescriptorSet = VK_NULL_HANDLE;
        VkPipeline pointDepthPipeline = VK_NULL_HANDLE;
        VkPipeline pointAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline pointConstantSimpleAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline pointFastBasicDepthPipeline = VK_NULL_HANDLE;
        VkPipeline pointFastBasicPipeline = VK_NULL_HANDLE;
        VkPipeline surfelDepthPipeline = VK_NULL_HANDLE;
        VkPipeline surfelAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline surfelConstantSimpleAccumulationPipeline = VK_NULL_HANDLE;
        VkPipeline rainPipeline = VK_NULL_HANDLE;
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
        BufferAllocation uniformBuffer{};
    };

    struct PendingPointDescriptorGeneration {
        VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
        std::array<std::vector<VkDescriptorSet>, kFramesInFlight> descriptorSets{};
        std::array<std::vector<std::uint64_t>, kFramesInFlight> seepageDescriptorGenerations{};
        std::vector<std::array<std::vector<VkDescriptorSet>, kFramesInFlight>> highlightDescriptorSets;
        std::vector<std::array<std::vector<std::uint64_t>, kFramesInFlight>> highlightSeepageDescriptorGenerations;
        VkDescriptorSet exrDescriptorSet = VK_NULL_HANDLE;
        std::uint64_t seepageExrDescriptorGeneration = 0U;
    };

    struct WaterSurfacePendingGpuUpload {
        // Resident tables are swapped into Rain and Flow descriptors only
        // after preprocessing signals. Upload staging is bounded and released
        // before this pending job is retained.
        BufferAllocation surfaceTableBuffer{};
        BufferAllocation vegetationTableBuffer{};
        BufferAllocation flowSurfaceInputBuffer{};
        BufferAllocation flowSurfaceTableBuffer{};
        BufferAllocation groundTableBuffer{};
        BufferAllocation groundEntryCandidateBuffer{};
        BufferAllocation preprocessUniformBuffer{};
        std::array<VkDescriptorSet, kFramesInFlight> rainDescriptorSets{};
        VkDescriptorPool rainDescriptorPool = VK_NULL_HANDLE;
        VkDescriptorSet preprocessDescriptorSet = VK_NULL_HANDLE;
        VkDescriptorPool preprocessDescriptorPool = VK_NULL_HANDLE;
        VkCommandBuffer preprocessCommandBuffer = VK_NULL_HANDLE;
        VkFence preprocessFence = VK_NULL_HANDLE;
        invisible_places::io::Bounds3f bounds{};
        std::uint32_t surfaceMask = 0U;
        std::uint32_t vegetationMask = 0U;
        std::uint32_t maximumProbeCount = 1U;
        std::uint32_t flowSurfaceMask = 0U;
        std::uint32_t flowMaximumProbeCount = 1U;
        std::uint32_t flowSurfaceCellCount = 0U;
        std::uint32_t flowSurfaceTableCapacity = 0U;
        std::uint32_t groundMask = 0U;
        std::uint32_t groundMaximumProbeCount = 1U;
        std::uint32_t groundCellCount = 0U;
        std::uint32_t groundEntryCandidateCount = 0U;
        float resolutionMeters = invisible_places::water::kWaterSurfaceResolutionMeters;
        std::uint64_t cacheRevision = 0U;
        invisible_places::water::WaterSurfaceCacheIdentity cacheIdentity{};
        std::uint64_t uploadRevision = 0U;
        std::uint64_t tableBytes = 0U;
        std::uint64_t peakStagingBytes = 0U;
    };

    struct WaterSurfaceRetiredGpuResources {
        BufferAllocation surfaceTableBuffer{};
        BufferAllocation vegetationTableBuffer{};
        BufferAllocation flowSurfaceTableBuffer{};
        BufferAllocation groundTableBuffer{};
        BufferAllocation groundEntryCandidateBuffer{};
        std::array<VkDescriptorSet, kFramesInFlight> rainDescriptorSets{};
        VkDescriptorPool rainDescriptorPool = VK_NULL_HANDLE;
        std::uint32_t outstandingFrameMask = 0U;
        bool outstandingExr = false;
        std::uint64_t uploadRevision = 0U;
    };

    struct RainGpuResources {
        BufferAllocation surfaceTableBuffer{};
        BufferAllocation vegetationTableBuffer{};
        BufferAllocation flowSurfaceTableBuffer{};
        BufferAllocation groundTableBuffer{};
        BufferAllocation groundEntryCandidateBuffer{};
        // Header plus a fixed 1,024-entry GPU-only ring. Rain writes VEG
        // impacts; Mesh Flow consumes the preceding completed submission.
        BufferAllocation dynamicMeshFlowRainSeedBuffer{};
        BufferAllocation dynamicMeshFlowDummyContactEventBuffer{};
        BufferAllocation dynamicMeshFlowDummyContactGridBuffer{};
        BufferAllocation particleBuffer{};
        BufferAllocation eventBuffer{};
        BufferAllocation counterBuffer{};
        std::array<BufferAllocation, kFramesInFlight> counterReadbackBuffers{};
        BufferAllocation impactCountBuffer{};
        BufferAllocation impactReferenceBuffer{};
        std::array<BufferAllocation, kFramesInFlight> uniformBuffers{};
        BufferAllocation exrUniformBuffer{};
        std::array<VkDescriptorSet, kFramesInFlight> descriptorSets{};
        VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
        VkDescriptorSet exrDescriptorSet = VK_NULL_HANDLE;
        VkDescriptorPool exrDescriptorPool = VK_NULL_HANDLE;
        WaterSurfacePendingGpuUpload pendingSurfaceUpload{};
        std::vector<WaterSurfacePendingGpuUpload> abandonedSurfaceUploads;
        std::vector<WaterSurfaceRetiredGpuResources> retiredSurfaceResources;
        invisible_places::io::Bounds3f collisionBounds{};
        std::uint32_t surfaceMask = 0U;
        std::uint32_t vegetationMask = 0U;
        std::uint32_t maximumProbeCount = 1U;
        std::uint32_t flowSurfaceMask = 0U;
        std::uint32_t flowMaximumProbeCount = 1U;
        std::uint32_t flowSurfaceCellCount = 0U;
        std::uint32_t flowSurfaceTableCapacity = 0U;
        std::uint32_t groundMask = 0U;
        std::uint32_t groundMaximumProbeCount = 1U;
        std::uint32_t groundCellCount = 0U;
        std::uint32_t groundEntryCandidateCount = 0U;
        float surfaceResolutionMeters = invisible_places::water::kWaterSurfaceResolutionMeters;
        std::uint32_t resetEpoch = 1U;
        std::uint64_t collisionCacheRevision = 0U;
        invisible_places::water::WaterSurfaceCacheIdentity collisionCacheIdentity{};
        std::uint64_t collisionUploadRevision = 0U;
        std::uint64_t flowSurfaceResidentUploadRevision = 0U;
        std::uint64_t groundResidentUploadRevision = 0U;
        std::uint64_t residentTableBytes = 0U;
        std::uint64_t peakStagingBytes = 0U;
        std::uint32_t preprocessDispatchCount = 0U;
        std::uint32_t lastSeed = 0U;
        float previousTimeSeconds = -std::numeric_limits<float>::infinity();
        float frameDeltaSeconds = 1.0F / 30.0F;
        bool previousRainEnabled = false;
        bool collisionReady = false;
        bool flowSurfaceReady = false;
    };

    struct ImGuiPreviewImageTextureResources {
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        ImageAllocation image{};
        VkSampler sampler = VK_NULL_HANDLE;
        VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
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
    void CreateDynamicMeshFlowDescriptorSetLayout();
    void CreateWaterFlowSourceDescriptorSetLayout();
    void CreateRainDescriptorSetLayout();
    void CreateWaterSurfacePreprocessDescriptorSetLayout();
    void CreateGaussianSplatDescriptorSetLayout();
    void CreateHighQualityGaussianSplatDescriptorSetLayout();
    void CreateCompositeDescriptorSetLayout();
    void CreatePostProcessDescriptorSetLayout();
    void CreateDescriptorPools();
    void CreatePostProcessSampler();
    void CreateUniformResources();
    void CreateRainResources();
    void CreatePointPipelines();
    void CreateDynamicMeshFlowComputePipeline();
    void CreateWaterFlowSourceComputePipelines();
    void CreateRainPipelines();
    void CreateWaterSurfacePreprocessPipeline();
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
    void UpdateRainDescriptorSets();
    void UpdateRainDescriptorSets(
        const std::array<VkDescriptorSet, kFramesInFlight>& descriptorSets,
        const BufferAllocation& surfaceTableBuffer,
        const BufferAllocation& vegetationTableBuffer);
    void UpdateRainExrDescriptorSet(
        const BufferAllocation& surfaceTableBuffer,
        const BufferAllocation& vegetationTableBuffer);
    void UpdateWaterSurfacePreprocessDescriptorSet(WaterSurfacePendingGpuUpload* pending);
    bool DispatchWaterSurfacePreprocess(WaterSurfacePendingGpuUpload* pending);
    void PollWaterSurfacePreprocess();
    void CleanupWaterSurfacePendingUpload(WaterSurfacePendingGpuUpload* pending);
    void CleanupWaterSurfaceRetiredResources(WaterSurfaceRetiredGpuResources* retired);
    void UpdateRainRuntimeTiming(const SceneRenderState& state);
    void UploadRainUniforms(std::size_t frameIndex, std::uint32_t width, std::uint32_t height);
    void UploadRainUniformsToBuffer(const BufferAllocation& target, std::uint32_t width, std::uint32_t height);
    void RecordRainCompute(VkCommandBuffer commandBuffer, std::size_t frameIndex);
    void RecordRainComputeWithDescriptor(VkCommandBuffer commandBuffer, VkDescriptorSet descriptorSet,
                                         const BufferAllocation* counterReadback);
    void RecordRainDraw(VkCommandBuffer commandBuffer, std::size_t frameIndex, VkPipeline pipeline);
    void RecordRainDrawWithDescriptor(VkCommandBuffer commandBuffer, VkDescriptorSet descriptorSet,
                                      VkPipeline pipeline);
    void CleanupRainResources();
    void FlushSparseWaterRippleParamsForFrame(std::size_t frameIndex);
    void FlushSparseWaterRippleParamsForExr();
    void FlushWaterSeepageParamsForFrame(std::size_t frameIndex);
    void FlushWaterSeepageParamsForExr();
    void UpdatePointCloudDescriptorSets(ActivePointCloudResources* resources,
                                        VkDescriptorPool allocationPool = VK_NULL_HANDLE);
    void ReplacePointCloudDescriptorSets(ActivePointCloudResources* resources);
    [[nodiscard]] PendingPointDescriptorGeneration BuildPointDescriptorGeneration(ActivePointCloudResources* resources,
                                                                                  VkImageView exrDepthView,
                                                                                  VkBuffer exrFrameUniformBuffer);
    void InstallPointDescriptorGeneration(ActivePointCloudResources* resources,
                                          PendingPointDescriptorGeneration* generation);
    void DestroyPointDescriptorGeneration(PendingPointDescriptorGeneration* generation);
    [[nodiscard]] VkDescriptorPool CreatePointDescriptorPool(std::size_t highlightCount, bool includeExrDescriptor);
    void DestroyPointDescriptorPool(VkDescriptorPool* descriptorPool);
    void ResetPointCloudReferencingCommandBuffers();
    void SettlePointCloudMutation();
    [[nodiscard]] std::uint64_t PointCloudResourceResidentBytes(const ActivePointCloudResources& resources) const;
    void TrackPointCloudResidentPeak(std::uint64_t unpublishedBytes = 0U);
    void RetirePointCloudRenderDescriptors(ActivePointCloudResources* resources);
    void DestroyPointCloudResources(ActivePointCloudResources* resources);
    void RetirePointCloudDescriptorSets(std::array<std::vector<VkDescriptorSet>, kFramesInFlight>* descriptorSets);
    void UpdateDynamicMeshFlowDescriptorSet(ActivePointCloudResources* resources,
                                            std::size_t liveSlot,
                                            const WaterGroundFlowGpuView& groundView);
    [[nodiscard]] VkDescriptorPool CreateDynamicMeshFlowDescriptorPool();
    void UpdateWaterFlowSourceDescriptorSet(ActivePointCloudResources* resources,
                                            const WaterSurfaceFlowGpuView& surfaceView);
    void CreateWaterFlowDummyPointResources(ActivePointCloudResources* resources);
    void RetireWaterFlowSourceComputeGeneration(ActivePointCloudResources* resources);
    [[nodiscard]] VkDescriptorPool CreateWaterFlowSourceDescriptorPool();
    void DispatchWaterFlowSourceCompute(ActivePointCloudResources* resources,
                                        const invisible_places::water::WaterFlowGpuOutputLayout& layout);
    void PollWaterFlowSourceDispatches();
    void PrepareDynamicMeshFlowDispatchSlot(ActivePointCloudResources* resources, std::size_t liveSlot);
    void DispatchDynamicMeshFlowCompute(ActivePointCloudResources* resources, std::uint32_t particleCount,
                                        std::size_t liveSlot);
    [[nodiscard]] DynamicMeshFlowContactGpuView
    PointDescriptorDynamicMeshFlowContactView(
        const ActivePointCloudResources* targetResources) const;
    void UpdatePointCloudDescriptorSet(ActivePointCloudResources* resources, std::size_t frameIndex,
                                       std::uint32_t imageIndex, VkImageView sceneDepthView,
                                       VkDescriptorPool allocationPool = VK_NULL_HANDLE);
    void UpdatePointHighlightDescriptorSets(ActivePointCloudResources* resources,
                                            ActivePointCloudResources::PointHighlightResources* highlight,
                                            VkDescriptorPool allocationPool = VK_NULL_HANDLE);
    void UpdatePointHighlightDescriptorSet(ActivePointCloudResources* resources,
                                           ActivePointCloudResources::PointHighlightResources* highlight,
                                           std::size_t frameIndex, std::uint32_t imageIndex, VkImageView sceneDepthView,
                                           VkDescriptorPool allocationPool = VK_NULL_HANDLE);
    void UpdatePointCloudExrDescriptorSet(ActivePointCloudResources* resources, VkImageView sceneDepthView,
                                          VkBuffer exrFrameUniformBuffer,
                                          VkDescriptorPool allocationPool = VK_NULL_HANDLE);
    void CreateOrUpdateCompositeDescriptorSet();
    void CreateOrUpdateCompositeDescriptorSet(VkDescriptorSet* descriptorSet, VkImageView accumulationView,
                                              VkImageView revealageView, VkImageView emissiveView);
    void CreateOrUpdateCompositeDescriptorSet(VkDescriptorSet* descriptorSet, VkImageView accumulationView,
                                              VkImageView revealageView, VkImageView emissiveView,
                                              VkImageView normalAccumulationView, VkImageView albedoAccumulationView);
    void CreateOrUpdatePostProcessDescriptorSets();
    void UpdateGaussianSplatDescriptorSets(ActiveGaussianSplatResources* resources);
    void UpdateGaussianSplatDescriptorSet(ActiveGaussianSplatResources* resources, std::size_t frameIndex,
                                          std::uint32_t imageIndex);
    void UpdateHighQualityGaussianDescriptorSet(std::size_t frameIndex);
    void RefreshHighQualityGaussianScene(std::size_t frameIndex);
    void CleanupSwapchain();
    void CleanupPointCloudResources(ActivePointCloudResources* resources);
    void CleanupPointHighlightResources(ActivePointCloudResources::PointHighlightResources* highlight);
    void CleanupGaussianSplatResources(ActiveGaussianSplatResources* resources);
    void CleanupHighQualityGaussianScene();
    void DestroyExrExportResources(ExrExportResources* resources);
    void CleanupExrExportResources();
    void RecreateSwapchain();
    void RecordCommandBuffer(VkCommandBuffer commandBuffer, std::uint32_t imageIndex, std::size_t frameIndex);
    void RecordExrExportCommandBuffer(const PointCloudExrFrameRequest& request);
    [[nodiscard]] invisible_places::output::HalfRgbaExrImage
    ReadCompletedExrExportFrame(PointCloudExrReadbackMask readbackMask, std::uint32_t width, std::uint32_t height);
    [[nodiscard]] bool SceneImageNeedsRender(std::uint32_t imageIndex) const;
    [[nodiscard]] bool AnySceneImageNeedsRender() const;
    [[nodiscard]] bool ResolvePointCloudDrawPlan(
        const SceneRenderState::PointCloudLayerState& layer,
        bool forceFullSource,
        bool requireLiveDescriptors,
        PointCloudDrawPlan* plan);
    [[nodiscard]] bool UploadPointCloudLayerStyle(
        const SceneRenderState::PointCloudLayerState& layer,
        const PointCloudDrawPlan& plan,
        std::size_t frameIndex,
        std::uint32_t imageIndex,
        bool exrStyle,
        const BufferAllocation* styleBufferOverride = nullptr,
        const bool* seepageDescriptorReadyOverride = nullptr);
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
    [[nodiscard]] bool RecordPointCloudHighlightDraw(
        VkCommandBuffer commandBuffer,
        const SceneRenderState::PointCloudLayerState& layer,
        const ActivePointCloudResources::PointHighlightResources& highlight,
        VkPipeline spritePipeline,
        VkPipeline surfelPipeline,
        std::size_t frameIndex,
        std::uint32_t imageIndex,
        std::uint32_t* recordedDrawPointCount = nullptr);
    void UpdateUniformBuffer(std::size_t frameIndex);
    void UploadFrameUniforms(std::size_t frameIndex, std::uint32_t width, std::uint32_t height);
    void UploadFrameUniformsToBuffer(
        const BufferAllocation& buffer,
        std::uint32_t width,
        std::uint32_t height);

    [[nodiscard]] BufferAllocation CreateHostVisibleBuffer(VkDeviceSize size, VkBufferUsageFlags usage) const;
    [[nodiscard]] BufferAllocation CreateDeviceLocalBuffer(
        VkDeviceSize size,
        VkBufferUsageFlags usage) const;
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
    VkPipelineLayout dynamicMeshFlowPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout waterFlowSourcePipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout rainPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout waterSurfacePreprocessPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout gaussianSplatPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout highQualityGaussianSplatPipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout compositePipelineLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout postProcessPipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pointDepthPrepassPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointConstantSimpleAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointOpaqueHardDiscPipeline_ = VK_NULL_HANDLE;
    VkPipeline pointFastBasicPipeline_ = VK_NULL_HANDLE;
    VkPipeline dynamicMeshFlowComputePipeline_ = VK_NULL_HANDLE;
    VkPipeline waterFlowRouteComputePipeline_ = VK_NULL_HANDLE;
    VkPipeline waterFlowTrailComputePipeline_ = VK_NULL_HANDLE;
    VkPipeline rainComputePipeline_ = VK_NULL_HANDLE;
    VkPipeline waterSurfacePreprocessPipeline_ = VK_NULL_HANDLE;
    VkPipeline rainPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelDepthPrepassPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelConstantSimpleAccumulationPipeline_ = VK_NULL_HANDLE;
    VkPipeline surfelOpaqueHardDiscPipeline_ = VK_NULL_HANDLE;
    VkPipeline gaussianSplatPipeline_ = VK_NULL_HANDLE;
    VkPipeline highQualityGaussianSplatPipeline_ = VK_NULL_HANDLE;
    VkPipeline compositePipeline_ = VK_NULL_HANDLE;
    VkPipeline postProcessPipeline_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout pointDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout dynamicMeshFlowDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout waterFlowSourceDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout rainDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout waterSurfacePreprocessDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout gaussianSplatDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout highQualityGaussianSplatDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout compositeDescriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout postProcessDescriptorSetLayout_ = VK_NULL_HANDLE;
    std::optional<DynamicMeshFlowContactGpuView>
        pointDescriptorDynamicMeshFlowContactOverride_;
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
    RainGpuResources rainResources_{};
    BufferAllocation liveSceneReadbackBuffer_{};
    std::uint32_t lastSubmittedFrameSlot_ = 0U;
    std::uint32_t lastSubmittedImageIndex_ = 0U;
    bool lastSubmittedSceneImageValid_ = false;
    bool exrExportFrameInFlight_ = false;
    std::uint32_t exrLastRecordedLayerCount_ = 0U;
    std::uint64_t exrLastRecordedPointCount_ = 0U;
    std::uint32_t exrExportInFlightWidth_ = 0;
    std::uint32_t exrExportInFlightHeight_ = 0;
    PointCloudExrReadbackMask exrExportInFlightReadbackMask_ = PointCloudExrReadbackMask::All;
    ImGuiPreviewImageTextureResources imguiPreviewImageTexture_{};
    bool highQualityGaussianSceneDirty_ = true;
    SceneRenderState renderState_{};
    ViewportDiagnostics diagnostics_{};
    bool diagnosticsEnabled_ = false;
    bool liveSceneRenderingEnabled_ = true;
    bool liveSceneReadbackCaptureEnabled_ = false;
    bool sceneCachingEnabled_ = false;
    std::uint32_t pointCloudMutationBatchDepth_ = 0U;
    std::uint32_t pointCloudMutationBatchWaitCount_ = 0U;
    std::uint32_t lastPointCloudMutationBatchWaitCount_ = 0U;
    std::uint64_t pointCloudResidentPeakBytes_ = 0U;
    std::uint64_t pointCloudMutationBatchPeakBytes_ = 0U;
    std::uint64_t lastPointCloudMutationBatchPeakBytes_ = 0U;
    bool smokeFailNextPointCloudUploadAfterPartialAllocation_ = false;
    bool smokeFailNextWaterFlowUploadAfterPartialAllocation_ = false;
    std::uint64_t sceneRevision_ = 1;
    std::vector<std::uint64_t> sceneImageRevisions_;
    bool diagnosticsTimingInitialized_ = false;
    double diagnosticsFpsWindowMs_ = 0.0;
    std::uint32_t diagnosticsFpsWindowFrames_ = 0;
    float pointSizeRangeMin_ = 1.0F;
    float pointSizeRangeMax_ = 64.0F;
};

}  // namespace invisible_places::renderer::core
