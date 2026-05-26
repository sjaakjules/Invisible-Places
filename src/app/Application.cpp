#include "app/Application.hpp"

#include "app/PointVisualSelection.hpp"
#include "camera/AnimationPath.hpp"
#include "camera/CameraPath.hpp"
#include "camera/CameraShot.hpp"
#include "InvisiblePlacesBuildConfig.hpp"
#include "camera/OrbitCamera.hpp"
#include "io/AssetDiscovery.hpp"
#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
#include "io/PlyHeader.hpp"
#include "output/ExrWriter.hpp"
#include "output/EyeDomeLighting.hpp"
#include "output/HoudiniCameraExport.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "output/RenderPreset.hpp"
#include "output/VideoWriter.hpp"
#include "platform/VulkanRuntimeConfig.hpp"
#include "platform/Window.hpp"
#include "platform/WindowTitle.hpp"
#include "renderer/core/VulkanViewportShell.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/pointcloud/Colormap.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "scene/SceneCatalog.hpp"
#include "serialization/ProjectDocument.hpp"
#include "style/RenderParameterBinding.hpp"
#include "ui/SidePanelState.hpp"
#include "water/WaterFlow.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cfloat>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stop_token>
#include <string_view>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#if defined(__APPLE__)
#include <mach/mach.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

#include <glm/mat4x4.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::app {

namespace {

using LayerKind = invisible_places::scene::LayerKind;
using PointBudgetState = invisible_places::renderer::pointcloud::PointBudgetState;
using PointCloudStyleState = invisible_places::renderer::pointcloud::PointCloudStyleState;
using PointCloudColorMode = invisible_places::renderer::pointcloud::PointCloudColorMode;
using PointCloudColormapId = invisible_places::renderer::pointcloud::PointCloudColormapId;
using PointCloudDepthContribution = invisible_places::renderer::pointcloud::PointCloudDepthContribution;
using PointCloudFalloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
using PointCloudGeometryMode = invisible_places::renderer::pointcloud::PointCloudGeometryMode;
using PointCloudNprPreset = invisible_places::renderer::pointcloud::PointCloudNprPreset;
using PointCloudPreviewLodMode = invisible_places::renderer::pointcloud::PointCloudPreviewLodMode;
using PointCloudRaycastPrimitiveMode = invisible_places::renderer::pointcloud::PointCloudRaycastPrimitiveMode;
using PointCloudRendererMode = invisible_places::renderer::pointcloud::PointCloudRendererMode;
using PointCloudStylisationMode = invisible_places::renderer::pointcloud::PointCloudStylisationMode;
using GaussianSplatStyleState = invisible_places::renderer::gsplat::GaussianSplatStyleState;
using GaussianSplatColorMode = invisible_places::renderer::gsplat::GaussianSplatColorMode;
using GaussianSplatDebugMode = invisible_places::renderer::gsplat::GaussianSplatDebugMode;
using GaussianSplatQualityMode = invisible_places::renderer::gsplat::GaussianSplatQualityMode;
using AnimationExportSettings = invisible_places::camera::AnimationExportSettings;
using AnimationPath = invisible_places::camera::AnimationPath;
using CameraShot = invisible_places::camera::CameraShot;
using RenderParameterBinding = invisible_places::style::RenderParameterBinding;
using ParameterSourceMode = invisible_places::style::ParameterSourceMode;
using FieldMapFlags = invisible_places::style::FieldMapFlags;
using ProjectDocument = invisible_places::serialization::ProjectDocument;
using ProjectLayerDocument = invisible_places::serialization::ProjectLayerDocument;
using PointCloudStylePresetDocument = invisible_places::serialization::PointCloudStylePresetDocument;
using RenderJobSettings = invisible_places::output::RenderJobSettings;
using WaterBakeSettings = invisible_places::water::WaterBakeSettings;
using WaterAnimationTrailSettings = invisible_places::water::WaterAnimationTrailSettings;
using WaterBasinRegion = invisible_places::water::WaterBasinRegion;
using WaterCausticLookSettings = invisible_places::water::WaterCausticLookSettings;
using WaterCausticPreviewTintMode = invisible_places::water::WaterCausticPreviewTintMode;
using WaterCausticRegion = invisible_places::water::WaterCausticRegion;
using WaterEmitter = invisible_places::water::WaterEmitter;
using WaterEmitterOrigin = invisible_places::water::WaterEmitterOrigin;
using WaterEmitterStatus = invisible_places::water::WaterEmitterStatus;
using WaterOverlay = invisible_places::water::WaterOverlay;
using WaterOverlayPoint = invisible_places::water::WaterOverlayPoint;
using WaterParticleTrailSettings = invisible_places::water::WaterParticleTrailSettings;
using WaterParticleTrailShapeSettings = invisible_places::water::WaterParticleTrailShapeSettings;
using WaterParticleVisualSettings = invisible_places::water::WaterParticleVisualSettings;
using WaterPathBranch = invisible_places::water::WaterPathBranch;
using WaterPathBranchRole = invisible_places::water::WaterPathBranchRole;
using WaterPathCache = invisible_places::water::WaterPathCache;
using WaterPathGenerationSettings = invisible_places::water::WaterPathGenerationSettings;
using WaterRenderSettings = invisible_places::water::WaterRenderSettings;
using WaterRunoffMode = invisible_places::water::WaterRunoffMode;
using WaterRunoffRegion = invisible_places::water::WaterRunoffRegion;
using WaterScaleMode = invisible_places::water::WaterScaleMode;
using WaterSettingsBundle = invisible_places::water::WaterSettingsBundle;
using WaterSourceSettingsAssignment = invisible_places::water::WaterSourceSettingsAssignment;
using WaterSourceSettings = invisible_places::water::WaterSourceSettings;
using WaterTrailBuildDiagnostics = invisible_places::water::WaterTrailBuildDiagnostics;
using WaterTrailBuildQuality = invisible_places::water::WaterTrailBuildQuality;
using TrailSurfaceIndex = invisible_places::water::TrailSurfaceIndex;
using LayerLoadResult = std::variant<
    invisible_places::io::PointCloudLoadResult,
    invisible_places::io::GaussianSplatLoadResult>;

constexpr float kPi = 3.14159265358979323846F;
constexpr std::size_t kMaxPivotSamples = 65536;
constexpr std::uint64_t kDefaultInteractivePointCap = 10'000'000ULL;
constexpr std::uint64_t kPointCloudPreviewLodTarget = 10'000'000ULL;
constexpr auto kPerformanceInteractionHold = std::chrono::milliseconds{300};
constexpr std::string_view kDefaultPointVisualName = invisible_places::app::point_visual::kDefaultName;
constexpr std::string_view kPresetPointVisualSuffix = invisible_places::app::point_visual::kPresetSuffix;
constexpr std::string_view kEditedPointVisualSuffix = invisible_places::app::point_visual::kEditedSuffix;

enum class PendingLoadPhase {
    CpuLoading,
    UploadPending
};

enum class GsplatTransformConvention {
    AsEncoded,
    SwapYZ,
    SwapYZFlipX
};

struct ProjectSettings {
    std::array<float, 4> backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    bool eyeDomeLightingEnabled = false;
    float eyeDomeLightingThickness = 1.0F;
    bool showStatusOverlay = true;
    bool constantUpdateView = false;
    bool liveVisualEffects = false;
    bool autoLowerGsplatQualityWhileNavigating = true;
    PointCloudPreviewLodMode pointCloudPreviewLodMode = PointCloudPreviewLodMode::AutoCameraLod;
    std::uint64_t interactivePointCap = kDefaultInteractivePointCap;
    PointCloudRendererMode pointCloudRendererMode = PointCloudRendererMode::Beauty;
    float gaussianSplatFootprintBoost = 1.5F;
    GsplatTransformConvention gsplatTransformConvention = GsplatTransformConvention::AsEncoded;
};

struct PersistenceState {
    std::string projectFilePath;
    std::string pointStylePresetPath;
    std::string animationDirectoryPath;
    std::vector<std::size_t> queuedLoadIndices;
};

enum class AssociationFilterMode {
    Visible,
    Any,
    Unassociated,
    Layer
};

struct CameraPanelState {
    std::string draftShotName = "Shot_001";
    std::optional<std::size_t> selectedShotIndex;
    std::optional<std::size_t> renamingShotIndex;
    std::optional<std::size_t> pendingLinkedShotDeleteIndex;
    std::string shotRenameBuffer;
    bool focusShotRename = false;
    std::optional<std::size_t> blendFromIndex;
    std::optional<std::size_t> blendToIndex;
    std::vector<std::size_t> pathShotIndices;
    std::optional<std::size_t> selectedPathItemIndex;
    float blendAmount = 0.0F;
    std::uint32_t pathDurationFrames = 180;
    bool liveBlend = true;
    AssociationFilterMode associationFilterMode = AssociationFilterMode::Visible;
    std::filesystem::path associationFilterLayerPath;
    std::vector<std::string> multiEditAllowedCameraIds;
    invisible_places::output::AnimationExportMode stillExportMode =
        invisible_places::output::AnimationExportMode::FastPreviewMp4;
};

struct CameraPlaybackState {
    bool active = false;
    std::vector<std::size_t> pathShotIndices;
    std::uint32_t durationFrames = 180;
    float durationSeconds = 3.0F;
    std::chrono::steady_clock::time_point startedAt{};
};

enum class AnimationEditTarget {
    Camera,
    Focus
};

struct AnimationViewportDragState {
    bool active = false;
    AnimationEditTarget target = AnimationEditTarget::Camera;
    std::size_t keyIndex = 0;
    int axis = 0;
    glm::vec3 startWorldPoint{0.0F, 0.0F, 0.0F};
    ImVec2 startMouse{};
    glm::vec3 axisWorld{1.0F, 0.0F, 0.0F};
    ImVec2 axisScreenDirection{1.0F, 0.0F};
    float pixelsPerWorldUnit = 1.0F;
};

struct QueuedQuickMp4Export {
    AnimationPath animationPath{};
    RenderJobSettings settings{};
    std::filesystem::path animationFilePath;
    std::string visualName;
    std::size_t visualSessionIndex = 0;
    PointCloudStyleState visualStyle{};
    std::filesystem::path videoOutputPath;
};

struct AnimationPanelState {
    std::optional<AnimationPath> currentPath;
    std::string currentFilePath;
    std::string draftAnimationName = "Animation 1";
    std::vector<std::filesystem::path> availableFiles;
    std::vector<std::vector<std::filesystem::path>> availableFileAssociatedLayerPaths;
    std::vector<std::optional<AnimationPath>> availableFileLoadedPaths;
    std::vector<bool> availableFileDirtyFlags;
    bool animationRegistryInitialized = false;
    std::vector<std::filesystem::path> selectedExportFiles;
    std::deque<QueuedQuickMp4Export> quickMp4Queue;
    std::size_t quickMp4QueueTotal = 0;
    std::size_t quickMp4QueueCompleted = 0;
    std::size_t quickMp4QueueSkipped = 0;
    std::optional<std::size_t> selectedFileIndex;
    std::optional<std::size_t> renamingFileIndex;
    std::string fileRenameBuffer;
    bool focusFileRename = false;
    std::optional<std::size_t> selectedKeyIndex;
    AnimationEditTarget editTarget = AnimationEditTarget::Camera;
    invisible_places::output::AnimationExportMode exportMode =
        invisible_places::output::AnimationExportMode::FastPreviewMp4;
    float scrubAmount = 0.0F;
    bool liveApply = true;
    bool previewDepthOfField = false;
    bool dirty = false;
    bool showSplines = true;
    bool exportPreviewDensity = true;
    bool exportSizeInitialized = false;
    AssociationFilterMode associationFilterMode = AssociationFilterMode::Visible;
    std::filesystem::path associationFilterLayerPath;
    std::filesystem::path lastHoudiniCameraScriptPath;
    std::filesystem::path lastHoudiniCameraExportDirectory;
    bool showHoudiniCameraExportNotice = false;
    AnimationViewportDragState drag{};
};

struct AnimationPlaybackState {
    bool active = false;
    AnimationPath path{};
    float durationSeconds = 3.0F;
    std::chrono::steady_clock::time_point startedAt{};
};

struct AnimationExportFramePayload {
    std::uint32_t outputFrameIndex = 0;
    invisible_places::output::HalfRgbaExrImage image{};
    invisible_places::output::HalfRgbaExrImage previewImage{};
};

struct AnimationExportOutputOptions {
    bool writeExrStack = false;
    bool writePreviewMp4 = false;
    bool previewMp4Optional = false;
    std::uint32_t mp4SupersampleScale = 1;
    std::filesystem::path previewVideoPath;
    std::string previewVideoWarning;
};

struct AnimationExportWriterState {
    std::mutex mutex;
    std::condition_variable condition;
    std::deque<AnimationExportFramePayload> pendingFrames;
    bool acceptingFrames = true;
    bool finishRequested = false;
    bool cancelRequested = false;
    bool completed = false;
    std::uint32_t writtenFrames = 0;
    std::filesystem::path lastOutputPath;
    std::string statusMessage;
    std::string errorMessage;
};

struct StillCameraPreparationState;

struct ExportLogState {
    std::filesystem::path path;
    std::chrono::system_clock::time_point startedWallTime{};
    std::chrono::steady_clock::time_point startedAt{};
    std::uint64_t startResidentMemoryBytes = 0;
    std::uint64_t peakResidentMemoryBytes = 0;
    std::uint64_t endResidentMemoryBytes = 0;
    std::chrono::steady_clock::duration gpuCaptureTotal{};
    std::chrono::steady_clock::duration gpuCaptureMin{};
    std::chrono::steady_clock::duration gpuCaptureMax{};
    std::uint32_t capturedFrames = 0;
    std::size_t peakQueuedFrames = 0;
};

struct OfflineRenderJobState {
    bool active = false;
    bool cancelRequested = false;
    invisible_places::output::AnimationExportMode mode =
        invisible_places::output::AnimationExportMode::FastPreviewMp4;
    RenderJobSettings settings{};
    std::vector<invisible_places::camera::CameraState> frames;
    std::vector<invisible_places::output::OfflineRenderTile> tiles;
    std::uint32_t currentFrame = 0;
    std::uint32_t currentTile = 0;
    invisible_places::output::ExrImage image{};
    std::chrono::steady_clock::time_point startedAt{};
    std::filesystem::path lastOutputPath;
    std::filesystem::path videoOutputPath;
    bool writeExrStack = false;
    bool writePreviewMp4 = false;
    bool optionalPreviewMp4 = false;
    std::uint32_t mp4SupersampleScale = 1;
    std::string previewVideoWarning;
    std::string errorMessage;
    std::uint32_t setupViewportWidth = 1;
    std::uint32_t setupViewportHeight = 1;
    std::uint32_t writtenFrameCount = 0;
    std::size_t pendingFrameCount = 0;
    bool previewDensity = true;
    PointCloudRendererMode pointCloudRendererMode = PointCloudRendererMode::Beauty;
    bool writerFinishRequested = false;
    bool quickMp4BatchJob = false;
    bool stillCameraJob = false;
    std::size_t quickMp4BatchIndex = 0;
    std::size_t quickMp4BatchTotal = 0;
    std::string animationName;
    std::filesystem::path animationFilePath;
    std::string exportVisualName;
    ExportLogState exportLog{};
    glm::vec4 exportBackgroundColor{0.0F, 0.0F, 0.0F, 0.0F};
    bool exportEyeDomeLightingEnabled = false;
    float exportEyeDomeLightingThickness = 1.0F;
    float exportGaussianSplatFootprintBoost = 1.5F;
    std::vector<invisible_places::renderer::core::SceneRenderState::PointCloudLayerState> exportPointCloudLayers;
    bool preparingExport = false;
    std::shared_ptr<StillCameraPreparationState> preparationState;
    std::shared_ptr<AnimationExportWriterState> writerState;
    std::shared_ptr<struct OfflineRenderProgressState> progressState;
    std::jthread worker;
};

struct StillCameraPreviewLodPreparationRequest {
    std::size_t sessionIndex = 0;
    std::string displayName;
    std::uint32_t requestedCount = 0;
    std::shared_ptr<const invisible_places::io::LoadedPointCloud> cloud;
};

struct StillCameraPreviewLodPreparationResult {
    std::size_t sessionIndex = 0;
    std::uint32_t requestedCount = 0;
    std::vector<std::uint32_t> sampledIndices;
};

struct StillCameraPreparationState {
    std::mutex mutex;
    bool cancelRequested = false;
    bool cancelled = false;
    bool completed = false;
    std::size_t totalRequests = 0;
    std::size_t completedRequests = 0;
    std::size_t currentRequestIndex = 0;
    std::string currentLayerName;
    std::vector<StillCameraPreviewLodPreparationResult> results;
    std::string errorMessage;
};

struct OfflineRenderProgressState {
    std::mutex mutex;
    bool cancelRequested = false;
    bool completed = false;
    std::uint32_t currentFrame = 0;
    std::uint32_t currentTile = 0;
    std::filesystem::path lastOutputPath;
    invisible_places::output::OfflinePointRenderDiagnostics lastDiagnostics{};
    std::string statusMessage;
    std::string errorMessage;
};

struct PreviewLayerSession;
struct PreviewRuntimeState;

void EnsureCameraShotSelections(CameraPanelState* panelState, std::size_t shotCount);
void EnsureRuntimeCameraShotIds(PreviewRuntimeState* runtimeState);
std::string NextCameraShotName(const PreviewRuntimeState& runtimeState);
void RequestAnimationExportWriterCancellation(OfflineRenderJobState* job);
void FinishOfflineRenderJob(
    PreviewRuntimeState* runtimeState,
    const std::string& statusMessage,
    const std::string& errorMessage = {});
std::filesystem::path AnimationDirectory(const PreviewRuntimeState& runtimeState);
std::filesystem::path HoudiniCameraExportDirectory(const PreviewRuntimeState& runtimeState);
const std::vector<std::filesystem::path>& AnimationRegistryAssociationPaths(
    const AnimationPanelState& panelState,
    std::size_t fileIndex);
bool AddAnimationFileToRegistry(
    AnimationPanelState* panelState,
    const std::filesystem::path& filePath,
    std::vector<std::filesystem::path> associatedLayerPaths);
void RefreshAnimationFileList(
    AnimationPanelState* panelState,
    const std::filesystem::path& animationDirectory);
PointCloudStyleState ApplyWaterCausticRenderStyle(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session,
    PointCloudStyleState style,
    bool includeEditorPreview = true);
std::string NormalizeMotionScalarFieldName(std::string_view name);
bool RefreshWaterCausticMaskForSession(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    std::size_t sessionIndex);

struct OfflinePointLayerSnapshot {
    std::shared_ptr<const invisible_places::io::LoadedPointCloud> cloud;
    PointCloudStyleState style{};
    bool hasSourceRgb = false;
    bool fastBasic = false;
    std::uint64_t drawPointCount = 0;
    glm::mat4 localToWorld{1.0F};
};

using SavedPointVisualState = invisible_places::app::point_visual::VisualState;

struct SavedWaterAnimationTrailProfileState {
    std::string name = "Unnamed";
    WaterAnimationTrailSettings settings{};
};

struct PreviewLayerSession {
    LayerKind kind = LayerKind::PointCloud;
    std::filesystem::path sourcePath;
    std::filesystem::path transformPath;
    std::string displayName;
    bool loaded = false;
    bool visible = false;
    bool hasSourceRgb = false;
    bool hasNormals = false;
    bool hasFocusPoint = false;
    std::uint64_t totalPrimitives = 0;
    invisible_places::io::Bounds3f localBounds{};
    invisible_places::io::Bounds3f bounds{};
    invisible_places::io::Float3 localFocusPoint{};
    invisible_places::io::Float3 focusPoint{};
    invisible_places::io::Matrix4d localToWorld{};
    bool hasLocalFocusPoint = false;
    std::vector<invisible_places::io::ScalarFieldStats> scalarFields;
    std::vector<invisible_places::io::Float3> pivotSamples;
    std::shared_ptr<invisible_places::io::LoadedPointCloud> offlinePointCloud;
    std::vector<std::uint32_t> previewLodSampledIndices;
    std::uint32_t previewLodRequestedDrawCount = 0;
    std::uint32_t previewLodSampledDrawCount = 0;
    PointBudgetState pointBudget{};
    PointCloudStyleState pointStyle{};
    std::vector<SavedPointVisualState> pointVisuals;
    std::string selectedPointVisualName = std::string{kDefaultPointVisualName};
    std::string pointVisualNameBuffer = std::string{kDefaultPointVisualName};
    GaussianSplatStyleState gsplatStyle{};
};

struct BackgroundLayerLoadState {
    std::mutex mutex;
    std::optional<LayerLoadResult> result;
};

struct PendingLayerLoad {
    std::size_t sessionIndex = 0;
    PendingLoadPhase phase = PendingLoadPhase::CpuLoading;
    std::shared_ptr<BackgroundLayerLoadState> backgroundState;
    std::jthread backgroundThread;
    std::optional<LayerLoadResult> completedResult;
    bool showUploadOverlayFrame = false;
    std::chrono::steady_clock::time_point startedAt = std::chrono::steady_clock::now();
};

struct CameraInteractionState {
    bool navigationActive = false;
    bool renderViewportMouseActive = false;
    std::chrono::steady_clock::time_point lastNavigationInputAt{};
};

struct PerformanceInteractionState {
    bool active = false;
    std::chrono::steady_clock::time_point lastUiInteractionAt{};
};

struct PivotOverlayState {
    bool visible = true;
    invisible_places::io::Float3 pivot{};
    std::chrono::steady_clock::time_point lastSetAt{};
};

enum class WaterAnimationTrailProfileSource {
    Auto,
    AnimationSaved,
    AnimationTemp,
    ProjectDefault,
    ProjectTempDefault
};

enum class WaterOverlayViewMode {
    Trail,
    Path
};

enum class WaterRegionFeature {
    None,
    Basin,
    Runoff,
    Caustic
};

struct WaterRegionVertexRef {
    WaterRegionFeature feature = WaterRegionFeature::None;
    std::size_t regionIndex = 0;
    std::size_t vertexIndex = 0;
};

struct WaterRegionSnapState {
    bool active = false;
    invisible_places::io::Float3 point{};
    std::optional<WaterRegionVertexRef> vertex;
    bool surface = false;
};

struct WaterRegionDragState {
    bool active = false;
    WaterRegionVertexRef vertex{};
    invisible_places::io::Float3 originalPoint{};
    WaterRegionSnapState snap{};
};

struct WaterRegionEditorState {
    std::optional<WaterRegionVertexRef> hoveredVertex;
    WaterRegionDragState drag{};
    bool consumedViewportInputThisFrame = false;
};

struct WaterPathDebugPolyline {
    std::uint32_t branchId = 0;
    bool trailLane = false;
    std::vector<invisible_places::io::Float3> points;
};

struct WaterWorkflowState {
    std::vector<WaterEmitter> emitters;
    std::vector<WaterBasinRegion> basinRegions;
    std::vector<WaterRunoffRegion> runoffRegions;
    std::vector<WaterCausticRegion> causticRegions;
    WaterSourceSettings defaultSourceSettings = invisible_places::water::DefaultWaterSourceSettings(WaterScaleMode::Mid);
    std::optional<WaterSourceSettings> tempDefaultSourceSettings;
    WaterAnimationTrailSettings defaultAnimationTrailSettings =
        invisible_places::water::DefaultWaterAnimationTrailSettings();
    std::optional<WaterAnimationTrailSettings> tempDefaultAnimationTrailSettings;
    std::vector<SavedWaterAnimationTrailProfileState> animationTrailProfiles;
    std::string selectedAnimationTrailProfileName = "Default";
    std::string animationTrailProfileNameBuffer = "Default";
    std::optional<WaterAnimationTrailSettings> editedAnimationTrailProfileSettings;
    WaterCausticLookSettings defaultCausticLookSettings =
        invisible_places::water::DefaultWaterCausticLookSettings();
    std::optional<WaterCausticLookSettings> tempDefaultCausticLookSettings;
    PointCloudStyleState defaultPointVisualStyle{};
    std::optional<PointCloudStyleState> tempDefaultPointVisualStyle;
    std::vector<SavedPointVisualState> pointVisuals;
    std::string selectedPointVisualName = "Water Flow_preset";
    std::string pointVisualNameBuffer = "Water Flow";
    WaterAnimationTrailProfileSource activeAnimationTrailProfileSource =
        WaterAnimationTrailProfileSource::Auto;
    WaterOverlayViewMode overlayViewMode = WaterOverlayViewMode::Trail;
    WaterOverlay pathAnchors{};
    WaterOverlay flowOverlay{};
    std::uint64_t flowOverlayRevision = 0;
    std::uint64_t pathDebugCacheRevision = 0;
    std::vector<WaterPathDebugPolyline> pathDebugPolylines;
    WaterPathCache pathCache{};
    std::shared_ptr<const TrailSurfaceIndex> trailSurfaceIndex;
    std::string trailSurfaceIndexSupportSignature;
    std::filesystem::path trailSurfaceIndexSupportPath;
    WaterTrailBuildDiagnostics lastTrailBuildDiagnostics{};
    bool pathCacheLoaded = false;
    bool pathCacheStale = false;
    bool placementArmed = false;
    bool pathDirty = false;
    std::unordered_set<std::uint32_t> dirtyEmitterIds;
    std::optional<std::uint32_t> hoveredPathBranchId;
    std::optional<std::uint32_t> selectedPathBranchId;
    std::vector<std::vector<std::uint32_t>> pathEditUndoHiddenBranchIds;
    std::unordered_map<std::uint32_t, double> causticPreviewTintPulseUntilSeconds;
    WaterRegionFeature activeRegionFeature = WaterRegionFeature::None;
    WaterRegionEditorState regionEditor{};
    std::optional<std::size_t> selectedEmitterIndex;
    std::optional<std::size_t> movingEmitterIndex;
    std::optional<std::size_t> selectedBasinRegionIndex;
    std::optional<std::size_t> selectedRunoffRegionIndex;
    std::optional<std::size_t> selectedCausticRegionIndex;
    std::optional<std::size_t> activeSupportSessionIndex;
    std::filesystem::path lastOverlayPath;
    std::filesystem::path lastBasinOverlayPath;
    std::filesystem::path lastRunoffOverlayPath;
    std::uint32_t nextEmitterId = 1;
    std::uint32_t nextBasinRegionId = 1;
    std::uint32_t nextRunoffRegionId = 1;
    std::uint32_t nextCausticRegionId = 1;
    std::uint32_t maxAutoSuggestions = 8;
    bool basinRegionPlacementArmed = false;
    bool runoffRegionPlacementArmed = false;
    bool causticRegionPlacementArmed = false;
};

struct PreviewRuntimeState {
    std::vector<PreviewLayerSession> sessions;
    std::optional<std::size_t> selectedSessionIndex;
    std::optional<PendingLayerLoad> pendingLoad;
    invisible_places::camera::OrbitCamera camera;
    bool preserveProjectCameraOnNextLayerActivation = false;
    CameraInteractionState cameraInteraction{};
    PerformanceInteractionState performanceInteraction{};
    PivotOverlayState pivotOverlay{};
    WaterWorkflowState water{};
    CameraPanelState cameraPanel{};
    CameraPlaybackState cameraPlayback{};
    AnimationPanelState animationPanel{};
    AnimationPlaybackState animationPlayback{};
    std::vector<CameraShot> cameraShots;
    RenderJobSettings renderSettings{};
    OfflineRenderJobState offlineRenderJob{};
    invisible_places::ui::SidePanelState sidePanel{};
    ProjectSettings projectSettings{};
    PersistenceState persistence{};
    bool showDiagnosticsPanel = false;
    bool pauseLiveViewportDuringExport = true;
    bool previewRenderStateSignatureValid = false;
    std::uint64_t previewRenderStateSignature = 0;
    std::chrono::steady_clock::time_point startedAt = std::chrono::steady_clock::now();
    std::string statusMessage;
    std::string errorMessage;
};

std::filesystem::path ResolveDataDirectory(const std::filesystem::path& requested) {
    if (!requested.empty()) {
        return requested;
    }

    if (const char* envValue = std::getenv("INVISIBLE_PLACES_DATA_DIR"); envValue != nullptr) {
        return std::filesystem::path{envValue};
    }

    return Application::DefaultDataDirectory();
}

std::string FormatPointCount(std::uint64_t value) {
    std::string digits = std::to_string(value);
    for (std::ptrdiff_t index = static_cast<std::ptrdiff_t>(digits.size()) - 3; index > 0; index -= 3) {
        digits.insert(static_cast<std::size_t>(index), ",");
    }
    return digits;
}

std::string FormatFixed(double value, int precision) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

std::string NormalizePathKey(const std::filesystem::path& path) {
    return path.lexically_normal().generic_string();
}

std::filesystem::path DefaultProjectFilePath(const std::filesystem::path& dataRoot) {
    return dataRoot.parent_path() / "Saved" / "invisible_places_project.json";
}

std::filesystem::path DefaultPointStylePresetPath(const std::filesystem::path& dataRoot) {
    return dataRoot.parent_path() / "Saved" / "pointcloud_style_preset.json";
}

std::filesystem::path DefaultRenderOutputDirectory(const std::filesystem::path& dataRoot) {
    return dataRoot.parent_path() / "Saved" / "renders" / "Invisible Places";
}

std::filesystem::path DefaultAnimationDirectory(const std::filesystem::path& dataRoot) {
    return dataRoot.parent_path() / "Saved" / "animations";
}

std::string DescribeBudget(const PointBudgetState& budget) {
    std::ostringstream output;
    output << FormatPointCount(budget.activePoints) << " / " << FormatPointCount(budget.totalPoints)
           << " points";
    return output.str();
}

invisible_places::renderer::pointcloud::PointCloudPreviewLodDecision ResolvePointCloudPreviewLodDecision(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    return invisible_places::renderer::pointcloud::ResolvePointCloudPreviewLod(
        session.pointBudget,
        runtimeState.projectSettings.pointCloudPreviewLodMode,
        runtimeState.cameraInteraction.navigationActive,
        runtimeState.cameraPlayback.active || runtimeState.animationPlayback.active,
        kPointCloudPreviewLodTarget);
}

bool FastBasicPointRendererActive(const ProjectSettings& settings) {
    return settings.pointCloudRendererMode == PointCloudRendererMode::FastBasic;
}

std::uint64_t EffectivePointDrawCount(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    if (FastBasicPointRendererActive(runtimeState.projectSettings)) {
        return session.pointBudget.activePoints;
    }

    const auto decision = ResolvePointCloudPreviewLodDecision(runtimeState, session);
    if (!decision.usesPreviewLod) {
        return decision.drawPointCount;
    }

    if (session.previewLodSampledDrawCount == 0) {
        return session.pointBudget.activePoints;
    }

    return std::min<std::uint64_t>(
        session.previewLodSampledDrawCount,
        decision.drawPointCount);
}

bool PointCloudPreviewLodApplied(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    return EffectivePointDrawCount(runtimeState, session) < session.pointBudget.activePoints;
}

std::string DescribePointCloudPreviewDraw(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    std::ostringstream output;
    output << FormatPointCount(EffectivePointDrawCount(runtimeState, session)) << " / "
           << FormatPointCount(session.pointBudget.activePoints) << " points";
    return output.str();
}

PointBudgetState MakePreviewPointBudgetState(
    const PreviewLayerSession& session,
    std::uint64_t requestedPoints) {
    if (session.offlinePointCloud != nullptr) {
        return invisible_places::renderer::pointcloud::MakePointBudgetState(
            *session.offlinePointCloud,
            requestedPoints);
    }

    return invisible_places::renderer::pointcloud::MakePointBudgetState(
        session.totalPrimitives,
        requestedPoints);
}

void ClearPreviewLodSampleCache(PreviewLayerSession* session) {
    if (session == nullptr) {
        return;
    }

    session->previewLodSampledIndices.clear();
    session->previewLodRequestedDrawCount = 0;
    session->previewLodSampledDrawCount = 0;
}

std::uint32_t RequestedPreviewLodSampleCount(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    if (FastBasicPointRendererActive(runtimeState.projectSettings)) {
        return 0;
    }

    const std::uint64_t targetPoints = kPointCloudPreviewLodTarget;
    if (session.kind != LayerKind::PointCloud ||
        session.pointBudget.UsesSampledIndices() ||
        session.pointBudget.activePoints == 0 ||
        targetPoints >= session.pointBudget.activePoints) {
        return 0;
    }

    return static_cast<std::uint32_t>(std::min<std::uint64_t>(
        std::max<std::uint64_t>(1ULL, targetPoints),
        std::numeric_limits<std::uint32_t>::max()));
}

void PreparePreviewLodSampleCache(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    std::size_t sessionIndex) {
    if (runtimeState == nullptr || viewport == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return;
    }

    auto& session = runtimeState->sessions[sessionIndex];
    if (!session.loaded ||
        session.kind != LayerKind::PointCloud ||
        session.offlinePointCloud == nullptr) {
        ClearPreviewLodSampleCache(&session);
        viewport->UpdateInteractivePointSampleBuffer(sessionIndex, session.previewLodSampledIndices);
        return;
    }

    const auto requestedCount = RequestedPreviewLodSampleCount(*runtimeState, session);
    if (requestedCount == 0) {
        ClearPreviewLodSampleCache(&session);
        viewport->UpdateInteractivePointSampleBuffer(sessionIndex, session.previewLodSampledIndices);
        return;
    }

    if (session.previewLodRequestedDrawCount == requestedCount &&
        session.previewLodSampledDrawCount > 0) {
        return;
    }

    session.previewLodSampledIndices =
        invisible_places::renderer::pointcloud::GenerateSpatialSampleIndices(
            session.offlinePointCloud->positions,
            session.offlinePointCloud->bounds,
            requestedCount);
    session.previewLodRequestedDrawCount = requestedCount;
    session.previewLodSampledDrawCount =
        static_cast<std::uint32_t>(session.previewLodSampledIndices.size());
    viewport->UpdateInteractivePointSampleBuffer(sessionIndex, session.previewLodSampledIndices);
}

void PreparePreviewLodSampleCaches(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return;
    }

    for (std::size_t sessionIndex = 0; sessionIndex < runtimeState->sessions.size(); ++sessionIndex) {
        PreparePreviewLodSampleCache(runtimeState, viewport, sessionIndex);
    }
}

std::vector<StillCameraPreviewLodPreparationRequest> BuildStillCameraPreviewLodPreparationRequests(
    const PreviewRuntimeState& runtimeState) {
    std::vector<StillCameraPreviewLodPreparationRequest> requests;
    for (std::size_t sessionIndex = 0; sessionIndex < runtimeState.sessions.size(); ++sessionIndex) {
        const auto& session = runtimeState.sessions[sessionIndex];
        if (!session.loaded ||
            !session.visible ||
            session.kind != LayerKind::PointCloud ||
            session.offlinePointCloud == nullptr) {
            continue;
        }

        const auto requestedCount = RequestedPreviewLodSampleCount(runtimeState, session);
        if (requestedCount == 0 ||
            (session.previewLodRequestedDrawCount == requestedCount &&
             session.previewLodSampledDrawCount > 0)) {
            continue;
        }

        requests.push_back(
            {.sessionIndex = sessionIndex,
             .displayName = session.displayName,
             .requestedCount = requestedCount,
             .cloud = session.offlinePointCloud});
    }
    return requests;
}

void RunStillCameraPreviewLodPreparationWorker(
    std::stop_token stopToken,
    std::vector<StillCameraPreviewLodPreparationRequest> requests,
    std::shared_ptr<StillCameraPreparationState> preparationState) {
    if (preparationState == nullptr) {
        return;
    }

    {
        std::scoped_lock lock(preparationState->mutex);
        preparationState->totalRequests = requests.size();
        preparationState->completedRequests = 0;
        preparationState->currentRequestIndex = 0;
        preparationState->currentLayerName.clear();
    }

    try {
        for (std::size_t requestIndex = 0; requestIndex < requests.size(); ++requestIndex) {
            const auto& request = requests[requestIndex];
            {
                std::scoped_lock lock(preparationState->mutex);
                if (preparationState->cancelRequested || stopToken.stop_requested()) {
                    preparationState->cancelled = true;
                    preparationState->completed = true;
                    return;
                }
                preparationState->currentRequestIndex = requestIndex;
                preparationState->currentLayerName = request.displayName;
            }

            std::vector<std::uint32_t> sampledIndices;
            if (request.cloud != nullptr) {
                sampledIndices =
                    invisible_places::renderer::pointcloud::GenerateSpatialSampleIndices(
                        request.cloud->positions,
                        request.cloud->bounds,
                        request.requestedCount);
            }

            {
                std::scoped_lock lock(preparationState->mutex);
                if (preparationState->cancelRequested || stopToken.stop_requested()) {
                    preparationState->cancelled = true;
                    preparationState->completed = true;
                    return;
                }
                preparationState->results.push_back(
                    {.sessionIndex = request.sessionIndex,
                     .requestedCount = request.requestedCount,
                     .sampledIndices = std::move(sampledIndices)});
                preparationState->completedRequests = requestIndex + 1U;
            }
        }
    } catch (const std::exception& error) {
        std::scoped_lock lock(preparationState->mutex);
        preparationState->errorMessage = "Still-camera EXR preparation failed: " + std::string{error.what()};
        preparationState->completed = true;
        return;
    }

    std::scoped_lock lock(preparationState->mutex);
    preparationState->completed = true;
}

const char* LayerKindLabel(LayerKind kind) {
    return kind == LayerKind::PointCloud ? "point-cloud" : "gSplat";
}

const char* LayerKindBadge(LayerKind kind) {
    return kind == LayerKind::PointCloud ? "[PC]" : "[GS]";
}

const char* PointCloudColorModeLabel(PointCloudColorMode mode) {
    switch (mode) {
        case PointCloudColorMode::SourceRgb:
            return "Source RGB";
        case PointCloudColorMode::SolidColor:
            return "Solid Color";
        case PointCloudColorMode::ScalarColormap:
            return "Scalar Colormap";
    }

    return "Unknown";
}

const char* PointCloudColormapLabel(PointCloudColormapId colormap) {
    switch (colormap) {
        case PointCloudColormapId::Viridis:
            return "Viridis";
        case PointCloudColormapId::Plasma:
            return "Plasma";
        case PointCloudColormapId::Inferno:
            return "Inferno";
        case PointCloudColormapId::Magma:
            return "Magma";
        case PointCloudColormapId::Cividis:
            return "Cividis";
        case PointCloudColormapId::Turbo:
            return "Turbo";
        case PointCloudColormapId::Topographic:
            return "Topo";
        case PointCloudColormapId::LandSurface:
            return "Land Surface";
        case PointCloudColormapId::ExponentialFire:
            return "Exp Fire";
        case PointCloudColormapId::ExponentialIce:
            return "Exp Ice";
        case PointCloudColormapId::HighContrast:
            return "High Contrast";
        case PointCloudColormapId::CustomGradient:
            return "Custom Gradient";
    }

    return "Viridis";
}

const char* PointCloudPreviewLodModeLabel(PointCloudPreviewLodMode mode) {
    switch (mode) {
        case PointCloudPreviewLodMode::FullResolution:
            return "Full Resolution";
        case PointCloudPreviewLodMode::AutoCameraLod:
            return "Auto Camera LOD";
        case PointCloudPreviewLodMode::ForceLod:
            return "Force LOD";
    }

    return "Unknown";
}

const char* PointCloudRendererModeLabel(PointCloudRendererMode mode) {
    switch (mode) {
        case PointCloudRendererMode::Beauty:
            return "Beauty";
        case PointCloudRendererMode::FastBasic:
            return "Fast Basic";
        case PointCloudRendererMode::Raytraced:
            return "Raytraced";
    }

    return "Beauty";
}

const char* PointCloudRaycastPrimitiveModeLabel(PointCloudRaycastPrimitiveMode mode) {
    switch (mode) {
        case PointCloudRaycastPrimitiveMode::StyleSurfels:
            return "Style Surfels";
        case PointCloudRaycastPrimitiveMode::SoftDensitySpheres:
            return "Soft Density Spheres";
    }

    return "Style Surfels";
}

const char* GaussianSplatColorModeLabel(GaussianSplatColorMode mode) {
    switch (mode) {
        case GaussianSplatColorMode::FullSh:
            return "Full SH";
        case GaussianSplatColorMode::DcOnly:
            return "DC Only";
    }

    return "Unknown";
}

const char* GaussianSplatDebugModeLabel(GaussianSplatDebugMode mode) {
    switch (mode) {
        case GaussianSplatDebugMode::Final:
            return "Final";
        case GaussianSplatDebugMode::Opacity:
            return "Opacity";
        case GaussianSplatDebugMode::Scale:
            return "Scale";
        case GaussianSplatDebugMode::Depth:
            return "Depth";
        case GaussianSplatDebugMode::LayerTint:
            return "Layer Tint";
    }

    return "Unknown";
}

const char* GaussianSplatQualityModeLabel(GaussianSplatQualityMode mode) {
    switch (mode) {
        case GaussianSplatQualityMode::Fast:
            return "Fast";
        case GaussianSplatQualityMode::Medium:
            return "Medium";
        case GaussianSplatQualityMode::SurfaceGuided:
            return "Surface Guided";
        case GaussianSplatQualityMode::High:
            return "High";
    }

    return "Unknown";
}

const char* GaussianSplatQualityModeShortLabel(GaussianSplatQualityMode mode) {
    switch (mode) {
        case GaussianSplatQualityMode::Fast:
            return "FAST";
        case GaussianSplatQualityMode::Medium:
            return "MED";
        case GaussianSplatQualityMode::SurfaceGuided:
            return "SURF";
        case GaussianSplatQualityMode::High:
            return "HQ";
    }

    return "?";
}

const char* GsplatTransformConventionLabel(GsplatTransformConvention convention) {
    switch (convention) {
        case GsplatTransformConvention::AsEncoded:
            return "As Encoded";
        case GsplatTransformConvention::SwapYZ:
            return "Swap Y/Z";
        case GsplatTransformConvention::SwapYZFlipX:
            return "Swap Y/Z + Flip X";
    }

    return "Unknown";
}

void HashCombine(std::uint64_t* seed, std::uint64_t value) {
    if (seed == nullptr) {
        return;
    }
    *seed ^= value + 0x9e3779b97f4a7c15ULL + (*seed << 6U) + (*seed >> 2U);
}

void HashString(std::uint64_t* seed, std::string_view value) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const char character : value) {
        hash ^= static_cast<unsigned char>(character);
        hash *= 1099511628211ULL;
    }
    HashCombine(seed, hash);
}

void HashBool(std::uint64_t* seed, bool value) {
    HashCombine(seed, value ? 1ULL : 0ULL);
}

void HashFloat(std::uint64_t* seed, float value) {
    HashCombine(seed, static_cast<std::uint64_t>(std::bit_cast<std::uint32_t>(value)));
}

void HashVec3(std::uint64_t* seed, const glm::vec3& value) {
    HashFloat(seed, value.x);
    HashFloat(seed, value.y);
    HashFloat(seed, value.z);
}

void HashVec4(std::uint64_t* seed, const glm::vec4& value) {
    HashFloat(seed, value.x);
    HashFloat(seed, value.y);
    HashFloat(seed, value.z);
    HashFloat(seed, value.w);
}

void HashMat4(std::uint64_t* seed, const glm::mat4& value) {
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            HashFloat(seed, value[column][row]);
        }
    }
}

void HashBinding(std::uint64_t* seed, const RenderParameterBinding& binding) {
    HashBool(seed, binding.active);
    HashCombine(seed, static_cast<std::uint64_t>(binding.mode));
    for (const float value : binding.constantValue) {
        HashFloat(seed, value);
    }
    HashCombine(seed, static_cast<std::uint64_t>(binding.fieldMap.fieldSlot));
    HashString(seed, binding.fieldMap.fieldName);
    HashFloat(seed, binding.fieldMap.inputMin);
    HashFloat(seed, binding.fieldMap.inputMax);
    HashFloat(seed, binding.fieldMap.outputMin);
    HashFloat(seed, binding.fieldMap.outputMax);
    HashFloat(seed, binding.fieldMap.gamma);
    HashCombine(seed, binding.fieldMap.flags);
}

void HashPointStyle(std::uint64_t* seed, const PointCloudStyleState& style) {
    HashCombine(seed, static_cast<std::uint64_t>(style.geometryMode));
    HashCombine(seed, static_cast<std::uint64_t>(style.depthContribution));
    HashCombine(seed, static_cast<std::uint64_t>(style.falloffProfile));
    HashCombine(seed, static_cast<std::uint64_t>(style.stylisationMode));
    HashCombine(seed, static_cast<std::uint64_t>(style.nprPreset));
    HashCombine(seed, static_cast<std::uint64_t>(style.colorMode));
    HashCombine(seed, static_cast<std::uint64_t>(style.colormap));
    for (const float value : style.solidColor) {
        HashFloat(seed, value);
    }
    for (const float value : style.gradientStartColor) {
        HashFloat(seed, value);
    }
    for (const float value : style.gradientEndColor) {
        HashFloat(seed, value);
    }
    for (const float value : style.colorizeColor) {
        HashFloat(seed, value);
    }
    HashFloat(seed, style.colorizeAmount);
    HashFloat(seed, style.stylisationStrength);
    HashFloat(seed, style.stylisationColorLevels);
    HashFloat(seed, style.stylisationInkStrength);
    HashFloat(seed, style.stylisationPaperGrain);
    HashFloat(seed, style.stylisationPigmentBleed);
    HashFloat(seed, style.brushAspect);
    HashFloat(seed, style.strokeJitter);
    HashFloat(seed, style.hatchStrength);
    HashFloat(seed, style.strokeOpacityVariance);
    HashFloat(seed, style.pigmentVariation);
    HashFloat(seed, style.pigmentAnimationSpeed);
    HashFloat(seed, style.granulationAngleStrength);
    HashFloat(seed, style.roughnessMotionStrength);
    HashFloat(seed, style.roughnessMotionScale);
    HashFloat(seed, style.roughnessMotionSpeed);
    HashFloat(seed, style.roughnessMotionThreshold);
    HashFloat(seed, style.roughnessMotionGroundId);
    HashBool(seed, style.causticAnimation);
    HashFloat(seed, style.causticIntensity);
    HashFloat(seed, style.causticScale);
    HashFloat(seed, style.causticSpeed);
    HashFloat(seed, style.causticLineSharpness);
    HashFloat(seed, style.causticWarp);
    HashFloat(seed, style.causticCellSizeMeters);
    HashFloat(seed, style.causticLineWidthMeters);
    HashFloat(seed, style.causticFeatherMeters);
    HashFloat(seed, style.causticSurfacePointSpacingMeters);
    HashFloat(seed, style.causticWarpAmplitudeMeters);
    for (const float value : style.causticTint) {
        HashFloat(seed, value);
    }
    HashFloat(seed, style.causticEmissionBoost);
    HashFloat(seed, style.causticOpacityBoost);
    HashFloat(seed, style.causticPointSizeBoost);
    HashFloat(seed, style.causticPreviewTintAmount);
    HashFloat(seed, style.causticPreviewTintRegionId);
    HashCombine(seed, static_cast<std::uint64_t>(style.causticMaskFieldSlot + 1));
    HashCombine(seed, static_cast<std::uint64_t>(style.causticEdgeFieldSlot + 1));
    HashCombine(seed, static_cast<std::uint64_t>(style.causticSeedFieldSlot + 1));
    HashFloat(seed, style.exposure);
    HashFloat(seed, style.innerRadius);
    HashFloat(seed, style.gaussianSharpness);
    HashFloat(seed, style.featherPower);
    HashFloat(seed, style.depthFalloff);
    HashFloat(seed, style.depthBias);
    HashFloat(seed, style.frontAlpha);
    HashFloat(seed, style.hiddenAlpha);
    HashFloat(seed, style.densityScale);
    HashFloat(seed, style.densityClamp);
    HashFloat(seed, style.waterStreakAspect);
    HashFloat(seed, style.depthAlphaThreshold);
    HashBool(seed, style.solidCenters);
    HashBool(seed, style.flowAnimation);
    HashBool(seed, style.waterPathView);
    HashBinding(seed, style.pointSize);
    HashBinding(seed, style.surfelDiameter);
    HashBinding(seed, style.opacity);
    HashBinding(seed, style.emissiveStrength);
    HashBinding(seed, style.xrayStrength);
    HashBinding(seed, style.depthFade);
    HashBinding(seed, style.colormapPosition);
}

void HashScalarFields(
    std::uint64_t* seed,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    HashCombine(seed, scalarFields.size());
    for (const auto& field : scalarFields) {
        HashString(seed, field.name);
        HashFloat(seed, field.minimum);
        HashFloat(seed, field.maximum);
        HashCombine(seed, field.count);
        HashBool(seed, field.valid);
    }
}

void HashGsplatStyle(std::uint64_t* seed, const GaussianSplatStyleState& style) {
    HashBool(seed, style.transformEnabled);
    HashCombine(seed, static_cast<std::uint64_t>(style.colorMode));
    HashCombine(seed, static_cast<std::uint64_t>(style.debugMode));
    HashCombine(seed, static_cast<std::uint64_t>(style.qualityMode));
    HashFloat(seed, style.opacityMultiplier);
    HashFloat(seed, style.scaleMultiplier);
    HashFloat(seed, style.exposure);
    HashFloat(seed, style.saturation);
    for (const float value : style.layerTint) {
        HashFloat(seed, value);
    }
}

std::uint64_t RenderStateSignature(
    const invisible_places::renderer::core::SceneRenderState& renderState) {
    std::uint64_t seed = 1469598103934665603ULL;
    HashMat4(&seed, renderState.view);
    HashMat4(&seed, renderState.projection);
    HashMat4(&seed, renderState.viewProjection);
    HashVec3(&seed, renderState.cameraPosition);
    HashVec4(&seed, renderState.backgroundColor);
    HashBool(&seed, renderState.eyeDomeLightingEnabled);
    HashFloat(&seed, renderState.eyeDomeLightingThickness);
    HashFloat(&seed, renderState.nearPlane);
    HashFloat(&seed, renderState.farPlane);
    HashBool(&seed, renderState.hasDepthOfField);
    HashFloat(&seed, renderState.focusDistance);
    HashFloat(&seed, renderState.apertureFStops);
    HashFloat(&seed, renderState.depthOfFieldMaxBlurPixels);
    HashFloat(&seed, renderState.gaussianSplatFootprintBoost);
    HashFloat(&seed, renderState.pointSizeScale);
    HashFloat(&seed, renderState.flowTimeSeconds);
    HashCombine(&seed, static_cast<std::uint64_t>(renderState.pointCloudRendererMode));
    HashCombine(&seed, renderState.pointCloudLayers.size());
    for (const auto& layer : renderState.pointCloudLayers) {
        HashCombine(&seed, layer.layerId);
        HashPointStyle(&seed, layer.style);
        HashScalarFields(&seed, layer.scalarFields);
        HashBool(&seed, layer.hasSourceRgb);
        HashCombine(&seed, layer.drawPointCount);
    }
    HashCombine(&seed, renderState.gaussianSplatLayers.size());
    for (const auto& layer : renderState.gaussianSplatLayers) {
        HashCombine(&seed, layer.layerId);
        HashGsplatStyle(&seed, layer.style);
        HashMat4(&seed, layer.localToWorld);
    }
    return seed;
}

glm::mat4 ToGlmMatrix(const invisible_places::io::Matrix4d& matrix) {
    glm::mat4 converted{1.0F};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            converted[column][row] = static_cast<float>(matrix.At(row, column));
        }
    }
    return converted;
}

glm::mat4 MakeGsplatConventionMatrix(GsplatTransformConvention convention) {
    switch (convention) {
        case GsplatTransformConvention::AsEncoded:
            return glm::mat4{1.0F};
        case GsplatTransformConvention::SwapYZ:
            return glm::mat4{
                glm::vec4{1.0F, 0.0F, 0.0F, 0.0F},
                glm::vec4{0.0F, 0.0F, 1.0F, 0.0F},
                glm::vec4{0.0F, 1.0F, 0.0F, 0.0F},
                glm::vec4{0.0F, 0.0F, 0.0F, 1.0F}};
        case GsplatTransformConvention::SwapYZFlipX:
            return glm::mat4{
                glm::vec4{-1.0F, 0.0F, 0.0F, 0.0F},
                glm::vec4{0.0F, 0.0F, 1.0F, 0.0F},
                glm::vec4{0.0F, 1.0F, 0.0F, 0.0F},
                glm::vec4{0.0F, 0.0F, 0.0F, 1.0F}};
    }

    return glm::mat4{1.0F};
}

glm::mat4 EffectiveGsplatLocalToWorld(const ProjectSettings& settings, const PreviewLayerSession& session) {
    return ToGlmMatrix(session.localToWorld) * MakeGsplatConventionMatrix(settings.gsplatTransformConvention);
}

float MatrixDeterminant3x3(const glm::mat4& matrix) {
    const float m00 = matrix[0][0];
    const float m01 = matrix[1][0];
    const float m02 = matrix[2][0];
    const float m10 = matrix[0][1];
    const float m11 = matrix[1][1];
    const float m12 = matrix[2][1];
    const float m20 = matrix[0][2];
    const float m21 = matrix[1][2];
    const float m22 = matrix[2][2];

    return (m00 * ((m11 * m22) - (m12 * m21))) -
           (m01 * ((m10 * m22) - (m12 * m20))) +
           (m02 * ((m10 * m21) - (m11 * m20)));
}

invisible_places::io::Float3 TransformPoint(
    const glm::mat4& matrix,
    const invisible_places::io::Float3& point) {
    const glm::vec4 transformed =
        matrix * glm::vec4{point.x, point.y, point.z, 1.0F};
    const float inverseW = std::abs(transformed.w) > 1.0e-6F ? (1.0F / transformed.w) : 1.0F;
    return {
        transformed.x * inverseW,
        transformed.y * inverseW,
        transformed.z * inverseW,
    };
}

invisible_places::io::Bounds3f TransformBounds(
    const invisible_places::io::Bounds3f& bounds,
    const glm::mat4& matrix) {
    invisible_places::io::Bounds3f transformedBounds;
    if (!bounds.valid) {
        return transformedBounds;
    }

    const std::array<invisible_places::io::Float3, 8> corners = {
        invisible_places::io::Float3{bounds.minimum.x, bounds.minimum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.minimum.x, bounds.minimum.y, bounds.maximum.z},
        invisible_places::io::Float3{bounds.minimum.x, bounds.maximum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.minimum.x, bounds.maximum.y, bounds.maximum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.minimum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.minimum.y, bounds.maximum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.maximum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.maximum.y, bounds.maximum.z},
    };

    for (const auto& corner : corners) {
        transformedBounds.Expand(TransformPoint(matrix, corner));
    }

    return transformedBounds;
}

struct EffectiveLayerFrame {
    invisible_places::io::Bounds3f bounds{};
    invisible_places::io::Float3 focusPoint{};
    bool hasFocusPoint = false;
};

EffectiveLayerFrame ComputeEffectiveLayerFrame(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    EffectiveLayerFrame frame;

    if (session.kind != LayerKind::GaussianSplat || !session.localBounds.valid) {
        frame.bounds = session.bounds;
        frame.focusPoint = session.focusPoint;
        frame.hasFocusPoint = session.hasFocusPoint;
        return frame;
    }

    const auto effectiveMatrix = EffectiveGsplatLocalToWorld(runtimeState.projectSettings, session);
    frame.bounds = TransformBounds(session.localBounds, effectiveMatrix);
    frame.focusPoint = session.hasLocalFocusPoint
                           ? TransformPoint(effectiveMatrix, session.localFocusPoint)
                           : session.focusPoint;
    frame.hasFocusPoint = frame.bounds.valid;
    return frame;
}

GaussianSplatQualityMode EffectiveGaussianSplatQualityMode(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    return invisible_places::renderer::gsplat::ResolveEffectiveGaussianSplatQualityMode(
        session.gsplatStyle.qualityMode,
        runtimeState.projectSettings.autoLowerGsplatQualityWhileNavigating,
        runtimeState.performanceInteraction.active);
}

bool BoundsIntersectsViewFrustum(
    const invisible_places::io::Bounds3f& bounds,
    const glm::mat4& viewProjection) {
    if (!bounds.valid) {
        return true;
    }

    const std::array<invisible_places::io::Float3, 8> corners = {
        invisible_places::io::Float3{bounds.minimum.x, bounds.minimum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.minimum.x, bounds.minimum.y, bounds.maximum.z},
        invisible_places::io::Float3{bounds.minimum.x, bounds.maximum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.minimum.x, bounds.maximum.y, bounds.maximum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.minimum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.minimum.y, bounds.maximum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.maximum.y, bounds.minimum.z},
        invisible_places::io::Float3{bounds.maximum.x, bounds.maximum.y, bounds.maximum.z},
    };

    bool allOutsideLeft = true;
    bool allOutsideRight = true;
    bool allOutsideBottom = true;
    bool allOutsideTop = true;
    bool allOutsideNear = true;
    bool allOutsideFar = true;

    for (const auto& corner : corners) {
        const glm::vec4 clip =
            viewProjection * glm::vec4{corner.x, corner.y, corner.z, 1.0F};
        allOutsideLeft &= clip.x < -clip.w;
        allOutsideRight &= clip.x > clip.w;
        allOutsideBottom &= clip.y < -clip.w;
        allOutsideTop &= clip.y > clip.w;
        allOutsideNear &= clip.z < -clip.w;
        allOutsideFar &= clip.z > clip.w;
    }

    return !(allOutsideLeft || allOutsideRight || allOutsideBottom ||
             allOutsideTop || allOutsideNear || allOutsideFar);
}

std::string MakeVerticalLabel(std::string_view label) {
    std::string verticalLabel;
    verticalLabel.reserve((label.size() * 2U) + 1U);

    for (std::size_t index = 0; index < label.size(); ++index) {
        verticalLabel.push_back(label[index]);
        if (index + 1U < label.size()) {
            verticalLabel.push_back('\n');
        }
    }

    return verticalLabel;
}

const invisible_places::io::ScalarFieldStats* ScalarFieldStatsBySlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    std::int32_t fieldSlot) {
    if (fieldSlot < 0 || static_cast<std::size_t>(fieldSlot) >= scalarFields.size()) {
        return nullptr;
    }

    return &scalarFields[static_cast<std::size_t>(fieldSlot)];
}

void EnsureFieldMappedBindingDefaults(
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    float outputMin,
    float outputMax) {
    if (binding == nullptr || scalarFields.empty()) {
        return;
    }

    invisible_places::style::SyncBindingFieldReference(binding, scalarFields);
    if (binding->fieldMap.fieldSlot >= 0 &&
        static_cast<std::size_t>(binding->fieldMap.fieldSlot) < scalarFields.size()) {
        return;
    }

    invisible_places::style::ConfigureFieldMapFromStats(
        binding,
        0,
        scalarFields[0].name,
        outputMin,
        outputMax,
        &scalarFields[0]);
}

void SanitizePointCloudStyle(PreviewLayerSession* session) {
    if (session == nullptr) {
        return;
    }

    if (!session->hasSourceRgb && session->pointStyle.colorMode == PointCloudColorMode::SourceRgb) {
        session->pointStyle.colorMode = session->scalarFields.empty()
                                            ? PointCloudColorMode::SolidColor
                                            : PointCloudColorMode::ScalarColormap;
    }

    if (session->scalarFields.empty() &&
        session->pointStyle.colorMode == PointCloudColorMode::ScalarColormap) {
        session->pointStyle.colorMode =
            session->hasSourceRgb ? PointCloudColorMode::SourceRgb : PointCloudColorMode::SolidColor;
    }

    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.pointSize, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.surfelDiameter, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.opacity, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.emissiveStrength, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.xrayStrength, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.depthFade, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.colormapPosition, session->scalarFields);

    session->pointStyle.exposure = std::max(0.0F, session->pointStyle.exposure);
    session->pointStyle.innerRadius = std::clamp(session->pointStyle.innerRadius, 0.0F, 0.99F);
    session->pointStyle.gaussianSharpness = std::max(0.001F, session->pointStyle.gaussianSharpness);
    session->pointStyle.featherPower = std::max(0.001F, session->pointStyle.featherPower);
    session->pointStyle.depthFalloff = std::max(0.0F, session->pointStyle.depthFalloff);
    session->pointStyle.depthBias = std::max(0.0F, session->pointStyle.depthBias);
    session->pointStyle.frontAlpha = std::clamp(session->pointStyle.frontAlpha, 0.0F, 1.0F);
    session->pointStyle.hiddenAlpha = std::clamp(session->pointStyle.hiddenAlpha, 0.0F, 1.0F);
    session->pointStyle.densityScale = std::max(0.0F, session->pointStyle.densityScale);
    session->pointStyle.densityClamp = std::max(0.0F, session->pointStyle.densityClamp);
    session->pointStyle.waterStreakAspect = std::clamp(session->pointStyle.waterStreakAspect, 1.0F, 32.0F);
    session->pointStyle.depthAlphaThreshold = std::clamp(session->pointStyle.depthAlphaThreshold, 0.0F, 1.0F);
    session->pointStyle.colorizeAmount = std::clamp(session->pointStyle.colorizeAmount, 0.0F, 1.0F);
    session->pointStyle.stylisationStrength =
        std::clamp(session->pointStyle.stylisationStrength, 0.0F, 1.0F);
    session->pointStyle.stylisationColorLevels =
        std::clamp(session->pointStyle.stylisationColorLevels, 2.0F, 16.0F);
    session->pointStyle.stylisationInkStrength =
        std::clamp(session->pointStyle.stylisationInkStrength, 0.0F, 1.0F);
    session->pointStyle.stylisationPaperGrain =
        std::clamp(session->pointStyle.stylisationPaperGrain, 0.0F, 1.0F);
    session->pointStyle.stylisationPigmentBleed =
        std::clamp(session->pointStyle.stylisationPigmentBleed, 0.0F, 1.0F);
    session->pointStyle.brushAspect =
        std::clamp(session->pointStyle.brushAspect, 0.25F, 6.0F);
    session->pointStyle.strokeJitter =
        std::clamp(session->pointStyle.strokeJitter, 0.0F, 1.0F);
    session->pointStyle.hatchStrength =
        std::clamp(session->pointStyle.hatchStrength, 0.0F, 1.0F);
    session->pointStyle.strokeOpacityVariance =
        std::clamp(session->pointStyle.strokeOpacityVariance, 0.0F, 1.0F);
    session->pointStyle.roughnessMotionStrength =
        std::clamp(session->pointStyle.roughnessMotionStrength, 0.0F, 1.0F);
    session->pointStyle.roughnessMotionScale =
        std::clamp(session->pointStyle.roughnessMotionScale, 0.01F, 50.0F);
    session->pointStyle.roughnessMotionSpeed =
        std::clamp(session->pointStyle.roughnessMotionSpeed, 0.0F, 8.0F);
    session->pointStyle.roughnessMotionThreshold =
        std::clamp(session->pointStyle.roughnessMotionThreshold, 0.0F, 1.0F);
    session->pointStyle.roughnessMotionGroundId =
        std::clamp(session->pointStyle.roughnessMotionGroundId, 0.0F, 1.0F);
    session->pointStyle.causticIntensity = std::clamp(session->pointStyle.causticIntensity, 0.0F, 5.0F);
    session->pointStyle.causticScale = std::clamp(session->pointStyle.causticScale, 0.01F, 80.0F);
    session->pointStyle.causticSpeed = std::clamp(session->pointStyle.causticSpeed, 0.0F, 10.0F);
    session->pointStyle.causticLineSharpness =
        std::clamp(session->pointStyle.causticLineSharpness, 0.0F, 1.0F);
    session->pointStyle.causticWarp = std::clamp(session->pointStyle.causticWarp, 0.0F, 3.0F);
    session->pointStyle.causticCellSizeMeters =
        std::clamp(session->pointStyle.causticCellSizeMeters, 0.005F, 5.0F);
    session->pointStyle.causticLineWidthMeters =
        std::clamp(session->pointStyle.causticLineWidthMeters, 0.0005F, 0.50F);
    session->pointStyle.causticFeatherMeters =
        std::clamp(session->pointStyle.causticFeatherMeters, 0.0005F, 0.50F);
    session->pointStyle.causticSurfacePointSpacingMeters =
        std::clamp(session->pointStyle.causticSurfacePointSpacingMeters, 0.0005F, 0.10F);
    session->pointStyle.causticWarpAmplitudeMeters =
        std::clamp(session->pointStyle.causticWarpAmplitudeMeters, 0.0F, 2.0F);
    for (auto& channel : session->pointStyle.causticTint) {
        channel = std::clamp(channel, 0.0F, 4.0F);
    }
    session->pointStyle.causticEmissionBoost =
        std::clamp(session->pointStyle.causticEmissionBoost, 0.0F, 8.0F);
    session->pointStyle.causticOpacityBoost =
        std::clamp(session->pointStyle.causticOpacityBoost, 0.0F, 2.0F);
    session->pointStyle.causticPointSizeBoost =
        std::clamp(session->pointStyle.causticPointSizeBoost, 0.0F, 4.0F);
    session->pointStyle.causticPreviewTintAmount =
        std::clamp(session->pointStyle.causticPreviewTintAmount, 0.0F, 1.0F);
    session->pointStyle.causticPreviewTintRegionId =
        std::clamp(session->pointStyle.causticPreviewTintRegionId, 0.0F, 16777216.0F);
    for (auto& channel : session->pointStyle.colorizeColor) {
        channel = std::clamp(channel, 0.0F, 1.0F);
    }
    for (auto& channel : session->pointStyle.gradientStartColor) {
        channel = std::clamp(channel, 0.0F, 1.0F);
    }
    for (auto& channel : session->pointStyle.gradientEndColor) {
        channel = std::clamp(channel, 0.0F, 1.0F);
    }

    if (session->pointStyle.colorMode == PointCloudColorMode::ScalarColormap) {
        EnsureFieldMappedBindingDefaults(
            &session->pointStyle.colormapPosition,
            session->scalarFields,
            0.0F,
            1.0F);
        session->pointStyle.colormapPosition.constantValue[0] =
            std::clamp(session->pointStyle.colormapPosition.constantValue[0], 0.0F, 1.0F);
        session->pointStyle.colormapPosition.fieldMap.outputMin =
            std::clamp(session->pointStyle.colormapPosition.fieldMap.outputMin, 0.0F, 1.0F);
        session->pointStyle.colormapPosition.fieldMap.outputMax =
            std::clamp(session->pointStyle.colormapPosition.fieldMap.outputMax, 0.0F, 1.0F);
    }
}

bool InputTextStringWithFlags(
    const char* label,
    std::string* value,
    ImGuiInputTextFlags flags) {
    if (value == nullptr) {
        return false;
    }

    constexpr std::size_t kMinimumInputBufferSize = 4096U;
    const auto bufferSize = std::max(kMinimumInputBufferSize, value->size() + 1024U + 1U);
    std::vector<char> buffer(bufferSize, '\0');
    const auto copySize = std::min(value->size(), buffer.size() - 1U);
    std::copy_n(value->data(), copySize, buffer.data());

    const bool submitted = ImGui::InputText(
        label,
        buffer.data(),
        buffer.size(),
        flags & ~ImGuiInputTextFlags_CallbackResize);
    if (submitted || ImGui::IsItemEdited() || ImGui::IsItemDeactivatedAfterEdit()) {
        *value = buffer.data();
    }
    return submitted;
}

bool InputTextString(const char* label, std::string* value) {
    return InputTextStringWithFlags(label, value, ImGuiInputTextFlags_None);
}

std::string TrimText(std::string_view value) {
    std::size_t begin = 0;
    while (begin < value.size() &&
           std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
        ++begin;
    }

    std::size_t end = value.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1U])) != 0) {
        --end;
    }

    return std::string{value.substr(begin, end - begin)};
}

bool EndsWith(std::string_view value, std::string_view suffix) {
    return value.size() >= suffix.size() &&
           value.substr(value.size() - suffix.size()) == suffix;
}

std::string NormalizePointVisualName(std::string_view name) {
    return invisible_places::app::point_visual::NormalizeName(name);
}

bool IsEditedPointVisualName(std::string_view name) {
    return invisible_places::app::point_visual::IsEditedName(name);
}

bool IsPresetPointVisualName(std::string_view name) {
    return invisible_places::app::point_visual::IsPresetName(name);
}

std::string BasePointVisualName(std::string_view name) {
    return invisible_places::app::point_visual::BaseName(name);
}

std::string PresetPointVisualName(std::string_view baseName) {
    return invisible_places::app::point_visual::PresetName(baseName);
}

std::string EditedPointVisualName(std::string_view baseName) {
    return invisible_places::app::point_visual::EditedName(baseName);
}

std::string NormalizeWaterPointVisualName(std::string_view name) {
    const auto normalized = NormalizePointVisualName(name);
    constexpr std::string_view legacyBuiltIns[] = {
        "Water Flow",
        "White Needle Glow",
        "White Gold Surfels",
        "Soft Mist Lines",
        "Blue Silver Threads",
    };
    for (const auto legacyName : legacyBuiltIns) {
        if (normalized == legacyName) {
            return PresetPointVisualName(legacyName);
        }
    }
    return normalized;
}

bool PointStylesEqualForSelection(
    const PointCloudStyleState& left,
    const PointCloudStyleState& right) {
    std::uint64_t leftSeed = 0U;
    std::uint64_t rightSeed = 0U;
    HashPointStyle(&leftSeed, left);
    HashPointStyle(&rightSeed, right);
    return leftSeed == rightSeed;
}

bool IsGeneratedWaterOverlaySession(const PreviewLayerSession& session);
bool IsGeneratedWaterFlowOverlaySession(const PreviewLayerSession& session);
bool IsProtectedWaterPointVisualName(std::string_view name);
std::optional<PointCloudStyleState> MakeProtectedWaterPointVisualStyle(
    const PreviewRuntimeState& runtimeState,
    std::string_view name);
PointCloudStyleState MakeWaterTrailExportStyle(PointCloudStyleState style);
void SaveWaterPointVisualStyle(PreviewRuntimeState* runtimeState, const PointCloudStyleState& style);
void ApplyWaterPointVisualStyleToGeneratedSessions(PreviewRuntimeState* runtimeState);
enum class WaterOverlayRefreshPersistence {
    InMemoryOnly,
    SavePathCache
};
bool RefreshWaterOverlayFromAnchors(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    WaterOverlayRefreshPersistence persistence = WaterOverlayRefreshPersistence::InMemoryOnly,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Preview);
bool TryLoadWaterPathCacheForSupport(
    PreviewRuntimeState* runtimeState,
    const PreviewLayerSession& sourceSession);

std::optional<std::size_t> FindPointVisualIndex(
    const PreviewLayerSession& session,
    std::string_view name) {
    return invisible_places::app::point_visual::FindIndex(session.pointVisuals, name);
}

void UpsertPointVisual(
    PreviewLayerSession* session,
    std::string_view name,
    const PointCloudStyleState& style) {
    if (session == nullptr) {
        return;
    }

    invisible_places::app::point_visual::Upsert(&session->pointVisuals, name, style);
}

bool IsWaterProjectCustomVisualName(std::string_view name) {
    const auto normalized = NormalizePointVisualName(name);
    return !normalized.empty() &&
           !IsPresetPointVisualName(normalized) &&
           !IsEditedPointVisualName(normalized);
}

std::optional<std::size_t> FindWaterProjectVisualIndex(
    const std::vector<SavedPointVisualState>& visuals,
    std::string_view name) {
    return invisible_places::app::point_visual::FindIndex(visuals, name);
}

void AppendWaterPointVisualIfMissing(
    std::vector<SavedPointVisualState>* visuals,
    std::string_view name,
    const PointCloudStyleState& style) {
    if (visuals == nullptr || !IsWaterProjectCustomVisualName(name)) {
        return;
    }

    const auto normalized = NormalizePointVisualName(name);
    if (FindWaterProjectVisualIndex(*visuals, normalized).has_value()) {
        return;
    }

    visuals->push_back({.name = normalized, .style = MakeWaterTrailExportStyle(style)});
}

void UpsertWaterProjectVisual(
    PreviewRuntimeState* runtimeState,
    std::string_view name,
    const PointCloudStyleState& style) {
    if (runtimeState == nullptr) {
        return;
    }

    const auto normalized = NormalizePointVisualName(name);
    if (!IsWaterProjectCustomVisualName(normalized)) {
        return;
    }
    invisible_places::app::point_visual::Upsert(
        &runtimeState->water.pointVisuals,
        normalized,
        MakeWaterTrailExportStyle(style));
    runtimeState->water.selectedPointVisualName = normalized;
    runtimeState->water.pointVisualNameBuffer = BasePointVisualName(normalized);
}

void ImportLegacyWaterPointVisualStyle(
    PreviewRuntimeState* runtimeState,
    const PointCloudStyleState& style,
    std::string_view customName = "Water Flow") {
    if (runtimeState == nullptr) {
        return;
    }

    const auto normalized = NormalizePointVisualName(customName);
    if (!FindWaterProjectVisualIndex(runtimeState->water.pointVisuals, normalized).has_value()) {
        UpsertWaterProjectVisual(runtimeState, normalized, style);
    }
    if (runtimeState->water.selectedPointVisualName.empty() ||
        runtimeState->water.selectedPointVisualName == "Water Flow_preset") {
        runtimeState->water.selectedPointVisualName = normalized;
        runtimeState->water.pointVisualNameBuffer = BasePointVisualName(normalized);
    }
}

void SyncWaterPointVisualSelectionFromSession(
    PreviewRuntimeState* runtimeState,
    const PreviewLayerSession& session) {
    if (runtimeState == nullptr || !IsGeneratedWaterFlowOverlaySession(session)) {
        return;
    }

    runtimeState->water.selectedPointVisualName = NormalizePointVisualName(session.selectedPointVisualName);
    runtimeState->water.pointVisualNameBuffer = BasePointVisualName(runtimeState->water.selectedPointVisualName);
}

void RemovePointVisual(PreviewLayerSession* session, std::string_view name) {
    if (session == nullptr) {
        return;
    }

    invisible_places::app::point_visual::Remove(&session->pointVisuals, name);
}

void SyncPointVisualNameBuffer(PreviewLayerSession* session) {
    if (session == nullptr) {
        return;
    }
    invisible_places::app::point_visual::SyncNameBuffer(
        &session->pointVisualNameBuffer,
        session->selectedPointVisualName);
}

void EnsurePointVisuals(PreviewLayerSession* session) {
    if (session == nullptr || session->kind != LayerKind::PointCloud) {
        return;
    }

    invisible_places::app::point_visual::Ensure(
        &session->pointVisuals,
        &session->selectedPointVisualName,
        &session->pointVisualNameBuffer,
        session->pointStyle);
}

void SelectPointVisual(PreviewLayerSession* session, std::string_view name) {
    if (session == nullptr) {
        return;
    }

    const bool selected = invisible_places::app::point_visual::Select(
        &session->pointVisuals,
        &session->selectedPointVisualName,
        &session->pointVisualNameBuffer,
        &session->pointStyle,
        name,
        session->pointStyle);
    (void)selected;
}

void MarkPointVisualEdited(PreviewLayerSession* session) {
    if (session == nullptr || session->kind != LayerKind::PointCloud) {
        return;
    }

    EnsurePointVisuals(session);
    const auto selectedName = NormalizePointVisualName(session->selectedPointVisualName);
    if (IsEditedPointVisualName(selectedName)) {
        UpsertPointVisual(session, selectedName, session->pointStyle);
        session->selectedPointVisualName = selectedName;
        return;
    }

    const auto editedName = EditedPointVisualName(selectedName);
    UpsertPointVisual(session, editedName, session->pointStyle);
    session->selectedPointVisualName = editedName;
    SyncPointVisualNameBuffer(session);
}

void SaveCurrentPointVisual(PreviewRuntimeState* runtimeState, PreviewLayerSession* session) {
    if (runtimeState == nullptr || session == nullptr || session->kind != LayerKind::PointCloud) {
        return;
    }

    EnsurePointVisuals(session);
    const auto selectedName = NormalizePointVisualName(session->selectedPointVisualName);
    const auto targetName = BasePointVisualName(
        session->pointVisualNameBuffer.empty()
            ? BasePointVisualName(selectedName)
            : session->pointVisualNameBuffer);
    if (IsGeneratedWaterFlowOverlaySession(*session) &&
        (IsPresetPointVisualName(targetName) || IsEditedPointVisualName(targetName))) {
        runtimeState->errorMessage =
            "Choose a new custom water visual name; preset and edited names cannot be overwritten.";
        runtimeState->statusMessage.clear();
        return;
    }

    UpsertPointVisual(session, targetName, session->pointStyle);
    if (IsGeneratedWaterFlowOverlaySession(*session)) {
        UpsertWaterProjectVisual(runtimeState, targetName, session->pointStyle);
    }
    if (IsEditedPointVisualName(selectedName)) {
        RemovePointVisual(session, selectedName);
        if (BasePointVisualName(selectedName) == std::string{kDefaultPointVisualName} &&
            targetName != std::string{kDefaultPointVisualName} &&
            session->pointVisuals.size() > 1U) {
            RemovePointVisual(session, kDefaultPointVisualName);
        }
    } else if (
        selectedName == std::string{kDefaultPointVisualName} &&
        targetName != selectedName &&
        session->pointVisuals.size() > 1U) {
        RemovePointVisual(session, selectedName);
    }

    session->selectedPointVisualName = targetName;
    SyncPointVisualNameBuffer(session);
    if (IsGeneratedWaterFlowOverlaySession(*session)) {
        runtimeState->water.selectedPointVisualName = targetName;
        runtimeState->water.pointVisualNameBuffer = BasePointVisualName(targetName);
        ApplyWaterPointVisualStyleToGeneratedSessions(runtimeState);
    }
    runtimeState->statusMessage = "Saved visual " + targetName + ".";
    runtimeState->errorMessage.clear();
}

bool IsExportablePointVisualName(std::string_view name) {
    const auto normalized = NormalizePointVisualName(name);
    return !normalized.empty() && !IsEditedPointVisualName(normalized);
}

bool AnimationPathHasExportVisual(const AnimationPath& path, std::string_view name) {
    const auto normalized = NormalizePointVisualName(name);
    return std::any_of(
        path.exportVisualNames.begin(),
        path.exportVisualNames.end(),
        [&normalized](const std::string& visualName) {
            return NormalizePointVisualName(visualName) == normalized;
        });
}

void SetAnimationPathExportVisual(AnimationPath* path, std::string_view name, bool enabled) {
    if (path == nullptr) {
        return;
    }

    const auto normalized = NormalizePointVisualName(name);
    if (!IsExportablePointVisualName(normalized)) {
        return;
    }

    auto existing = std::find_if(
        path->exportVisualNames.begin(),
        path->exportVisualNames.end(),
        [&normalized](const std::string& visualName) {
            return NormalizePointVisualName(visualName) == normalized;
        });
    if (enabled) {
        if (existing == path->exportVisualNames.end()) {
            path->exportVisualNames.push_back(normalized);
        }
        return;
    }

    if (existing != path->exportVisualNames.end()) {
        path->exportVisualNames.erase(existing);
    }
}

void RemoveUnexportableVisualNames(AnimationPath* path) {
    if (path == nullptr) {
        return;
    }

    for (auto& visualName : path->exportVisualNames) {
        visualName = NormalizePointVisualName(visualName);
    }
    path->exportVisualNames.erase(
        std::remove_if(
            path->exportVisualNames.begin(),
            path->exportVisualNames.end(),
            [](const std::string& visualName) {
                return !IsExportablePointVisualName(visualName);
            }),
        path->exportVisualNames.end());
    std::sort(path->exportVisualNames.begin(), path->exportVisualNames.end());
    path->exportVisualNames.erase(
        std::unique(path->exportVisualNames.begin(), path->exportVisualNames.end()),
        path->exportVisualNames.end());
}

const SavedPointVisualState* FindExportablePointVisual(
    const PreviewLayerSession& session,
    std::string_view name) {
    const auto normalized = NormalizePointVisualName(name);
    if (!IsExportablePointVisualName(normalized)) {
        return nullptr;
    }

    const auto visualIt = std::find_if(
        session.pointVisuals.begin(),
        session.pointVisuals.end(),
        [&normalized](const SavedPointVisualState& visual) {
            return NormalizePointVisualName(visual.name) == normalized &&
                   IsExportablePointVisualName(visual.name);
        });
    return visualIt == session.pointVisuals.end() ? nullptr : &*visualIt;
}

bool DrawSectionHeader(const char* label) {
    static std::unordered_map<ImGuiID, bool> collapsedSections;

    const ImGuiStyle& style = ImGui::GetStyle();
    const ImGuiID sectionId = ImGui::GetID(label);
    bool& collapsed = collapsedSections[sectionId];
    const bool open = !collapsed;
    const float textHeight = ImGui::GetTextLineHeight();
    const float rowHeight = textHeight + style.FramePadding.y * 2.0F;
    const float toggleWidth = 22.0F;
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    const ImU32 lineColor = ImGui::GetColorU32(ImGuiCol_Separator);
    const ImU32 textColor = ImGui::GetColorU32(ImGuiCol_Text);

    ImGui::PushID(label);
    ImGui::InvisibleButton("##section_toggle", ImVec2{toggleWidth, rowHeight});
    if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
        collapsed = !collapsed;
    }

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const float lineY = cursor.y + rowHeight * 0.5F;
    const float lineStartX = cursor.x + 3.0F;
    const float lineEndX = cursor.x + toggleWidth - 3.0F;
    drawList->AddLine(ImVec2{lineStartX, lineY}, ImVec2{lineEndX, lineY}, lineColor, 2.0F);
    if (collapsed) {
        const float centerX = cursor.x + toggleWidth * 0.5F;
        drawList->AddLine(
            ImVec2{centerX, cursor.y + 4.0F},
            ImVec2{centerX, cursor.y + rowHeight - 4.0F},
            lineColor,
            2.0F);
    }

    ImGui::SameLine(0.0F, style.ItemSpacing.x);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(textColor), "%s", label);

    const ImVec2 afterText = ImGui::GetItemRectMax();
    const float availableEndX = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    const float continuationStartX = afterText.x + style.ItemSpacing.x;
    if (availableEndX > continuationStartX) {
        drawList->AddLine(
            ImVec2{continuationStartX, lineY},
            ImVec2{availableEndX, lineY},
            lineColor,
            2.0F);
    }

    ImGui::PopID();
    return open;
}

bool BeginPanelSection(const char* label) {
    const bool open = DrawSectionHeader(label);
    if (open) {
        ImGui::PushID(label);
        ImGui::Spacing();
    }
    return open;
}

void EndPanelSection() {
    ImGui::Spacing();
    ImGui::PopID();
}

bool DrawRightAlignedCombo(
    const char* label,
    int* currentItem,
    const char* const* items,
    int itemCount,
    float comboWidth = 210.0F) {
    if (currentItem == nullptr || items == nullptr || itemCount <= 0) {
        return false;
    }

    ImGui::PushID(label);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine();
    const float cursorX = ImGui::GetCursorPosX();
    const float rightX = ImGui::GetWindowContentRegionMax().x;
    ImGui::SetCursorPosX(std::max(cursorX, rightX - comboWidth));
    ImGui::SetNextItemWidth(std::min(comboWidth, ImGui::GetContentRegionAvail().x));
    const bool changed = ImGui::Combo("##combo", currentItem, items, itemCount);
    ImGui::PopID();
    return changed;
}

struct RangedFloatControlConfig {
    float visualMin = 0.0F;
    float visualMax = 1.0F;
    const char* format = "%.3f";
    float speed = 0.0F;
    std::optional<float> hardMin = std::nullopt;
    std::optional<float> hardMax = std::nullopt;
};

bool IsValidRangedFloatValue(float value, const RangedFloatControlConfig& config) {
    if (!std::isfinite(value)) {
        return false;
    }
    if (config.hardMin.has_value() && value < config.hardMin.value()) {
        return false;
    }
    if (config.hardMax.has_value() && value > config.hardMax.value()) {
        return false;
    }
    return true;
}

bool TryAssignRangedFloatValue(float* value, float candidate, const RangedFloatControlConfig& config) {
    if (value == nullptr || !IsValidRangedFloatValue(candidate, config)) {
        return false;
    }

    if (*value == candidate) {
        return false;
    }

    *value = candidate;
    return true;
}

bool DrawRangedFloatControl(const char* label, float* value, const RangedFloatControlConfig& config) {
    if (value == nullptr) {
        return false;
    }

    const float visualMin = std::min(config.visualMin, config.visualMax);
    const float visualMax = std::max(config.visualMin, config.visualMax);
    const float visualRange = std::max(visualMax - visualMin, 1.0e-6F);
    const float dragSpeed = config.speed > 0.0F ? config.speed : visualRange / 200.0F;
    static ImGuiID editingControlId = 0;
    static ImGuiID focusEditingControlId = 0;
    static ImGuiID activeDragId = 0;
    static float activeDragStartValue = 0.0F;
    static ImVec2 activeDragStartMouse{};
    bool changed = false;

    ImGui::PushID(label);
    const ImGuiID controlId = ImGui::GetID("##range");
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine();

    const ImGuiStyle& style = ImGui::GetStyle();
    const float barWidth = std::max(110.0F, ImGui::GetContentRegionAvail().x);
    const float frameHeight = ImGui::GetFrameHeight();

    if (editingControlId == controlId) {
        float inputValue = *value;
        ImGui::SetNextItemWidth(barWidth);
        if (focusEditingControlId == controlId) {
            ImGui::SetKeyboardFocusHere();
            focusEditingControlId = 0;
        }
        if (ImGui::InputFloat("##value", &inputValue, 0.0F, 0.0F, config.format)) {
            // TODO: surface an "out of range" popup once transient validation UI is structured.
            changed |= TryAssignRangedFloatValue(value, inputValue, config);
        }
        if (ImGui::IsItemDeactivated()) {
            editingControlId = 0;
        }
        ImGui::PopID();
        return changed;
    }

    const ImVec2 barMin = ImGui::GetCursorScreenPos();
    const ImVec2 barMax{barMin.x + barWidth, barMin.y + frameHeight};
    ImGui::InvisibleButton("##range", ImVec2{barWidth, frameHeight});
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        editingControlId = controlId;
        focusEditingControlId = controlId;
    }
    if (ImGui::IsItemActivated()) {
        activeDragId = controlId;
        activeDragStartValue = std::clamp(*value, visualMin, visualMax);
        activeDragStartMouse = ImGui::GetIO().MousePos;
    }
    if (activeDragId == controlId && ImGui::IsItemActive()) {
        const float deltaX = ImGui::GetIO().MousePos.x - activeDragStartMouse.x;
        if (std::abs(deltaX) > 0.0F) {
            const float candidate = std::clamp(activeDragStartValue + deltaX * dragSpeed, visualMin, visualMax);
            changed |= TryAssignRangedFloatValue(value, candidate, config);
        }
    }
    if (activeDragId == controlId && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        activeDragId = 0;
    }

    const float fillT = std::clamp((*value - visualMin) / visualRange, 0.0F, 1.0F);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImU32 frameColor = ImGui::GetColorU32(ImGuiCol_FrameBg);
    const ImU32 fillColor = ImGui::GetColorU32(ImGuiCol_SliderGrabActive);
    const ImU32 borderColor = ImGui::GetColorU32(ImGuiCol_Border);
    const ImU32 textColor = ImGui::GetColorU32(ImGuiCol_Text);
    drawList->AddRectFilled(barMin, barMax, frameColor, style.FrameRounding);
    drawList->AddRectFilled(
        barMin,
        ImVec2{barMin.x + barWidth * fillT, barMax.y},
        fillColor,
        style.FrameRounding);
    drawList->AddRect(barMin, barMax, borderColor, style.FrameRounding);

    char valueText[64]{};
    std::snprintf(valueText, sizeof(valueText), config.format, *value);
    const ImVec2 textSize = ImGui::CalcTextSize(valueText);
    drawList->AddText(
        ImVec2{barMin.x + (barWidth - textSize.x) * 0.5F, barMin.y + (frameHeight - textSize.y) * 0.5F},
        textColor,
        valueText);

    ImGui::PopID();
    return changed;
}

struct ScalarBindingWidgetConfig {
    float constantMin = 0.0F;
    float constantMax = 1.0F;
    float defaultOutputMin = 0.0F;
    float defaultOutputMax = 1.0F;
    float defaultConstant = 0.0F;
    const char* format = "%.3f";
    float displayScale = 1.0F;
    std::optional<float> hardMin = std::nullopt;
    std::optional<float> hardMax = std::nullopt;
};

const char* FieldMapModeLabel(ParameterSourceMode mode) {
    return mode == ParameterSourceMode::FieldMapped ? "Field-Mapped" : "Constant";
}

std::string BindingFieldLabel(
    const RenderParameterBinding& binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    const auto* fieldStats = ScalarFieldStatsBySlot(scalarFields, binding.fieldMap.fieldSlot);
    if (fieldStats != nullptr) {
        return fieldStats->name;
    }
    if (!binding.fieldMap.fieldName.empty()) {
        return binding.fieldMap.fieldName + " (missing)";
    }
    return "Select Field";
}

bool DrawScalarBindingBody(
    const char* id,
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    const ScalarBindingWidgetConfig& config) {
    if (binding == nullptr) {
        return false;
    }

    bool changed = false;
    ImGui::PushID(id);

    bool active = binding->active;
    if (ImGui::Checkbox("Active", &active)) {
        binding->active = active;
        changed = true;
    }
    if (!binding->active) {
        ImGui::BeginDisabled();
    }

    int modeIndex = static_cast<int>(binding->mode);
    const char* modeLabels[] = {"Constant", "Field-Mapped"};
    if (DrawRightAlignedCombo("Mode", &modeIndex, modeLabels, IM_ARRAYSIZE(modeLabels))) {
        binding->mode = static_cast<ParameterSourceMode>(modeIndex);
        changed = true;
        if (binding->mode == ParameterSourceMode::FieldMapped) {
            EnsureFieldMappedBindingDefaults(
                binding,
                scalarFields,
                config.defaultOutputMin,
                config.defaultOutputMax);
        }
    }

    if (binding->mode == ParameterSourceMode::Constant || scalarFields.empty()) {
        const float displayScale = std::max(1.0e-6F, config.displayScale);
        float constantValue = invisible_places::style::ScalarConstant(*binding) * displayScale;
        const std::optional<float> hardMin =
            config.hardMin.has_value() ? std::optional<float>{config.hardMin.value() * displayScale} : std::nullopt;
        const std::optional<float> hardMax =
            config.hardMax.has_value() ? std::optional<float>{config.hardMax.value() * displayScale} : std::nullopt;
        const bool valueChanged = DrawRangedFloatControl(
            "Value",
            &constantValue,
            {.visualMin = config.constantMin * displayScale,
             .visualMax = config.constantMax * displayScale,
             .format = config.format,
             .hardMin = hardMin,
             .hardMax = hardMax});
        if (valueChanged) {
            invisible_places::style::SetScalarConstant(binding, constantValue / displayScale);
            changed = true;
        }
        if (binding->mode == ParameterSourceMode::FieldMapped && scalarFields.empty()) {
            ImGui::TextDisabled("No scalar fields are available for this layer.");
        }
        if (!binding->active) {
            ImGui::EndDisabled();
        }
        ImGui::PopID();
        return changed;
    }

    EnsureFieldMappedBindingDefaults(binding, scalarFields, config.defaultOutputMin, config.defaultOutputMax);

    if (ImGui::BeginCombo("Field", BindingFieldLabel(*binding, scalarFields).c_str())) {
        for (std::size_t fieldIndex = 0; fieldIndex < scalarFields.size(); ++fieldIndex) {
            const bool selected = binding->fieldMap.fieldSlot == static_cast<std::int32_t>(fieldIndex);
            if (ImGui::Selectable(scalarFields[fieldIndex].name.c_str(), selected)) {
                invisible_places::style::ConfigureFieldMapFromStats(
                    binding,
                    static_cast<std::int32_t>(fieldIndex),
                    scalarFields[fieldIndex].name,
                    binding->fieldMap.outputMin,
                    binding->fieldMap.outputMax,
                    &scalarFields[fieldIndex]);
                changed = true;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    const auto* fieldStats = ScalarFieldStatsBySlot(scalarFields, binding->fieldMap.fieldSlot);
    if (fieldStats != nullptr && fieldStats->valid) {
        ImGui::Text("Discovered: %.5g to %.5g", fieldStats->minimum, fieldStats->maximum);
    }

    bool useLayerStats = invisible_places::style::HasFieldMapFlag(
        binding->fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats);
    if (ImGui::Checkbox("Use Layer Stats", &useLayerStats)) {
        invisible_places::style::SetFieldMapFlag(
            &binding->fieldMap,
            invisible_places::style::FieldMapFlagUseLayerStats,
            useLayerStats);
        changed = true;
    }

    if (!useLayerStats) {
        changed |= ImGui::InputFloat("Input Min", &binding->fieldMap.inputMin, 0.0F, 0.0F, "%.5g");
        changed |= ImGui::InputFloat("Input Max", &binding->fieldMap.inputMax, 0.0F, 0.0F, "%.5g");
    } else if (fieldStats != nullptr && fieldStats->valid) {
        binding->fieldMap.inputMin = fieldStats->minimum;
        binding->fieldMap.inputMax = fieldStats->maximum;
    }

    const float displayScale = std::max(1.0e-6F, config.displayScale);
    float outputMin = binding->fieldMap.outputMin * displayScale;
    float outputMax = binding->fieldMap.outputMax * displayScale;
    const std::optional<float> hardMin =
        config.hardMin.has_value() ? std::optional<float>{config.hardMin.value() * displayScale} : std::nullopt;
    const std::optional<float> hardMax =
        config.hardMax.has_value() ? std::optional<float>{config.hardMax.value() * displayScale} : std::nullopt;
    if (DrawRangedFloatControl(
            "Output Min",
            &outputMin,
            {.visualMin = config.constantMin * displayScale,
             .visualMax = config.constantMax * displayScale,
             .format = config.format,
             .hardMin = hardMin,
             .hardMax = hardMax})) {
        binding->fieldMap.outputMin = outputMin / displayScale;
        changed = true;
    }
    if (DrawRangedFloatControl(
            "Output Max",
            &outputMax,
            {.visualMin = config.constantMin * displayScale,
             .visualMax = config.constantMax * displayScale,
             .format = config.format,
             .hardMin = hardMin,
             .hardMax = hardMax})) {
        binding->fieldMap.outputMax = outputMax / displayScale;
        changed = true;
    }
    changed |= DrawRangedFloatControl(
        "Gamma",
        &binding->fieldMap.gamma,
        {.visualMin = 0.05F, .visualMax = 4.0F, .format = "%.2f", .hardMin = 0.0001F});

    bool clamp = invisible_places::style::HasFieldMapFlag(
        binding->fieldMap,
        invisible_places::style::FieldMapFlagClamp);
    if (ImGui::Checkbox("Clamp", &clamp)) {
        invisible_places::style::SetFieldMapFlag(
            &binding->fieldMap,
            invisible_places::style::FieldMapFlagClamp,
            clamp);
        changed = true;
    }

    bool invert = invisible_places::style::HasFieldMapFlag(
        binding->fieldMap,
        invisible_places::style::FieldMapFlagInvert);
    if (ImGui::Checkbox("Invert", &invert)) {
        invisible_places::style::SetFieldMapFlag(
            &binding->fieldMap,
            invisible_places::style::FieldMapFlagInvert,
            invert);
        changed = true;
    }

    if (ImGui::Button("Reset To Constant")) {
        binding->mode = ParameterSourceMode::Constant;
        invisible_places::style::SetScalarConstant(binding, config.defaultConstant);
        changed = true;
    }

    if (!binding->active) {
        ImGui::EndDisabled();
    }
    ImGui::PopID();
    return changed;
}

bool DrawScalarBindingWidget(
    const char* label,
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    const ScalarBindingWidgetConfig& config) {
    if (!BeginPanelSection(label)) {
        return false;
    }

    const bool changed = DrawScalarBindingBody(label, binding, scalarFields, config);
    EndPanelSection();
    return changed;
}

std::vector<PreviewLayerSession> BuildSessions(const invisible_places::io::AssetCatalog& assetCatalog) {
    std::vector<PreviewLayerSession> sessions;
    sessions.reserve(assetCatalog.pointClouds.size() + assetCatalog.gaussianSplats.size());

    for (const auto& asset : assetCatalog.pointClouds) {
        PreviewLayerSession session;
        session.kind = LayerKind::PointCloud;
        session.sourcePath = asset.filePath;
        session.displayName = asset.filePath.stem().string();
        session.hasSourceRgb = asset.header.HasColorRgb();
        session.totalPrimitives = asset.header.vertexCount;
        session.pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            asset.header.vertexCount,
            asset.header.vertexCount);
        session.pointStyle.colorMode =
            session.hasSourceRgb ? PointCloudColorMode::SourceRgb : PointCloudColorMode::SolidColor;
        EnsurePointVisuals(&session);
        sessions.push_back(std::move(session));
    }

    for (const auto& asset : assetCatalog.gaussianSplats) {
        PreviewLayerSession session;
        session.kind = LayerKind::GaussianSplat;
        session.sourcePath = asset.filePath;
        session.transformPath = asset.transformPath;
        session.displayName = asset.filePath.stem().string();
        session.totalPrimitives = asset.header.vertexCount;
        session.localToWorld = asset.localToWorld;
        sessions.push_back(std::move(session));
    }

    return sessions;
}

std::size_t ChooseStartupCloudIndex(const std::vector<PreviewLayerSession>& sessions) {
    constexpr std::string_view preferredFile = "Site2 -5mm.ply";
    for (std::size_t index = 0; index < sessions.size(); ++index) {
        if (sessions[index].kind != LayerKind::PointCloud) {
            continue;
        }
        if (sessions[index].sourcePath.filename() == preferredFile) {
            return index;
        }
    }

    for (std::size_t index = 0; index < sessions.size(); ++index) {
        if (sessions[index].kind == LayerKind::PointCloud) {
            return index;
        }
    }

    return 0;
}

bool HasAnyPointClouds(const PreviewRuntimeState& runtimeState) {
    return std::any_of(
        runtimeState.sessions.begin(),
        runtimeState.sessions.end(),
        [](const PreviewLayerSession& session) { return session.kind == LayerKind::PointCloud; });
}

bool IsBusyLoading(const PreviewRuntimeState& runtimeState) {
    return runtimeState.pendingLoad.has_value();
}

std::size_t LoadedLayerCount(const PreviewRuntimeState& runtimeState) {
    return static_cast<std::size_t>(std::count_if(
        runtimeState.sessions.begin(),
        runtimeState.sessions.end(),
        [](const PreviewLayerSession& session) { return session.loaded; }));
}

std::size_t VisibleLayerCount(const PreviewRuntimeState& runtimeState) {
    return static_cast<std::size_t>(std::count_if(
        runtimeState.sessions.begin(),
        runtimeState.sessions.end(),
        [](const PreviewLayerSession& session) { return session.loaded && session.visible; }));
}

float CurrentAspectRatio(const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    const auto width = std::max<std::uint32_t>(1U, viewport.Width());
    const auto height = std::max<std::uint32_t>(1U, viewport.Height());
    return static_cast<float>(width) / static_cast<float>(height);
}

ImVec2 CurrentUiViewportSize(const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (ImGui::GetCurrentContext() != nullptr) {
        if (const ImGuiViewport* mainViewport = ImGui::GetMainViewport(); mainViewport != nullptr) {
            if (mainViewport->Size.x > 1.0F && mainViewport->Size.y > 1.0F) {
                return mainViewport->Size;
            }
        }
    }

    return ImVec2{
        static_cast<float>(std::max<std::uint32_t>(1U, viewport.Width())),
        static_cast<float>(std::max<std::uint32_t>(1U, viewport.Height())),
    };
}

ImVec2 CurrentFramebufferViewportSize(const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    return ImVec2{
        static_cast<float>(std::max<std::uint32_t>(1U, viewport.Width())),
        static_cast<float>(std::max<std::uint32_t>(1U, viewport.Height())),
    };
}

ImVec2 CurrentUiViewportOrigin() {
    if (ImGui::GetCurrentContext() != nullptr) {
        if (const ImGuiViewport* mainViewport = ImGui::GetMainViewport(); mainViewport != nullptr) {
            return mainViewport->Pos;
        }
    }

    return ImVec2{0.0F, 0.0F};
}

ImVec2 CurrentUiViewportCenter(const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    const auto origin = CurrentUiViewportOrigin();
    const auto size = CurrentUiViewportSize(viewport);
    return ImVec2{origin.x + (size.x * 0.5F), origin.y + (size.y * 0.5F)};
}

ImVec2 ToRenderViewportLocal(
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    const auto origin = CurrentUiViewportOrigin();
    const auto size = CurrentUiViewportSize(viewport);
    const ImVec2 local{screenPoint.x - origin.x, screenPoint.y - origin.y};
    return ImVec2{
        std::clamp(local.x, 0.0F, std::max(1.0F, size.x)),
        std::clamp(local.y, 0.0F, std::max(1.0F, size.y)),
    };
}

bool IsInsideRenderViewport(
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    const auto origin = CurrentUiViewportOrigin();
    const auto size = CurrentUiViewportSize(viewport);
    return screenPoint.x >= origin.x && screenPoint.x <= origin.x + size.x &&
           screenPoint.y >= origin.y && screenPoint.y <= origin.y + size.y;
}

bool IsMouseOverRenderViewport(const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (ImGui::GetCurrentContext() == nullptr) {
        return false;
    }

    const auto& io = ImGui::GetIO();
    if (!ImGui::IsMousePosValid(&io.MousePos) || !IsInsideRenderViewport(viewport, io.MousePos)) {
        return false;
    }

    const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
    return mainViewport == nullptr || io.MouseHoveredViewport == 0 || io.MouseHoveredViewport == mainViewport->ID;
}

bool IsRenderViewportFocused() {
    if (ImGui::GetCurrentContext() == nullptr) {
        return false;
    }

    const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
    return mainViewport != nullptr && (mainViewport->Flags & ImGuiViewportFlags_IsFocused) != 0;
}

glm::vec3 ToGlm(const invisible_places::io::Float3& value) {
    return {value.x, value.y, value.z};
}

invisible_places::io::Float3 FromGlm(const glm::vec3& value) {
    return {value.x, value.y, value.z};
}

bool IsGeneratedWaterOverlaySession(const PreviewLayerSession& session) {
    if (session.kind != LayerKind::PointCloud) {
        return false;
    }

    const auto stem = session.sourcePath.stem().string();
    const auto waterVisualName = NormalizeWaterPointVisualName(session.selectedPointVisualName);
    return session.pointStyle.flowAnimation ||
           waterVisualName == "Water Flow_preset" ||
           waterVisualName == "Basin Haze" ||
           waterVisualName == "Runoff" ||
           stem.ends_with("-WaterPreview") ||
           stem.ends_with("-WaterFlow") ||
           stem.ends_with("-BasinHaze") ||
           stem.ends_with("-Runoff");
}

bool IsGeneratedWaterFlowOverlaySession(const PreviewLayerSession& session) {
    if (session.kind != LayerKind::PointCloud) {
        return false;
    }
    const auto stem = session.sourcePath.stem().string();
    return NormalizeWaterPointVisualName(session.selectedPointVisualName) == "Water Flow_preset" ||
           stem.ends_with("-WaterPreview") ||
           stem.ends_with("-WaterFlow");
}

std::optional<std::size_t> FindSessionIndexBySourcePath(
    const PreviewRuntimeState& runtimeState,
    const std::filesystem::path& sourcePath);

bool IsAssociableLidarSession(const PreviewLayerSession& session) {
    if (session.kind != LayerKind::PointCloud) {
        return false;
    }

    const auto stem = session.sourcePath.stem().string();
    const auto waterVisualName = NormalizeWaterPointVisualName(session.selectedPointVisualName);
    return waterVisualName != "Water Flow_preset" &&
           waterVisualName != "Basin Haze" &&
           waterVisualName != "Runoff" &&
           !stem.ends_with("-WaterPreview") &&
           !stem.ends_with("-WaterFlow") &&
           !stem.ends_with("-BasinHaze") &&
           !stem.ends_with("-Runoff");
}

bool IsVisibleAssociableLidarSession(const PreviewLayerSession& session) {
    return IsAssociableLidarSession(session) && session.loaded && session.visible;
}

void NormalizeAssociatedLayerPaths(std::vector<std::filesystem::path>* paths) {
    if (paths == nullptr) {
        return;
    }

    paths->erase(
        std::remove_if(
            paths->begin(),
            paths->end(),
            [](const std::filesystem::path& path) { return path.empty(); }),
        paths->end());
    for (auto& path : *paths) {
        path = path.lexically_normal();
    }
    std::sort(
        paths->begin(),
        paths->end(),
        [](const std::filesystem::path& left, const std::filesystem::path& right) {
            return NormalizePathKey(left) < NormalizePathKey(right);
        });
    paths->erase(
        std::unique(
            paths->begin(),
            paths->end(),
            [](const std::filesystem::path& left, const std::filesystem::path& right) {
                return NormalizePathKey(left) == NormalizePathKey(right);
            }),
        paths->end());
}

bool AssociatedLayerPathsContain(
    const std::vector<std::filesystem::path>& paths,
    const std::filesystem::path& targetPath) {
    const auto targetKey = NormalizePathKey(targetPath);
    return std::any_of(
        paths.begin(),
        paths.end(),
        [&targetKey](const std::filesystem::path& path) {
            return NormalizePathKey(path) == targetKey;
        });
}

std::vector<std::filesystem::path> VisibleAssociatedLidarLayerPaths(
    const PreviewRuntimeState& runtimeState) {
    std::vector<std::filesystem::path> paths;
    for (const auto& session : runtimeState.sessions) {
        if (IsVisibleAssociableLidarSession(session)) {
            paths.push_back(session.sourcePath);
        }
    }
    NormalizeAssociatedLayerPaths(&paths);
    return paths;
}

bool AssociatedLayerPathsIntersect(
    const std::vector<std::filesystem::path>& left,
    const std::vector<std::filesystem::path>& right) {
    return std::any_of(
        left.begin(),
        left.end(),
        [&right](const std::filesystem::path& path) {
            return AssociatedLayerPathsContain(right, path);
        });
}

bool AssociationMatchesFilter(
    const PreviewRuntimeState& runtimeState,
    const std::vector<std::filesystem::path>& associatedLayerPaths,
    AssociationFilterMode filterMode,
    const std::filesystem::path& filterLayerPath) {
    switch (filterMode) {
        case AssociationFilterMode::Any:
            return true;
        case AssociationFilterMode::Unassociated:
            return associatedLayerPaths.empty();
        case AssociationFilterMode::Layer:
            return !filterLayerPath.empty() &&
                   AssociatedLayerPathsContain(associatedLayerPaths, filterLayerPath);
        case AssociationFilterMode::Visible:
        default: {
            const auto visibleLayerPaths = VisibleAssociatedLidarLayerPaths(runtimeState);
            if (visibleLayerPaths.empty()) {
                return associatedLayerPaths.empty();
            }
            return AssociatedLayerPathsIntersect(associatedLayerPaths, visibleLayerPaths);
        }
    }
}

std::string FormatAssociationSummary(
    const PreviewRuntimeState& runtimeState,
    const std::vector<std::filesystem::path>& associatedLayerPaths) {
    if (associatedLayerPaths.empty()) {
        return "Unassociated";
    }

    std::vector<std::string> names;
    names.reserve(associatedLayerPaths.size());
    for (const auto& path : associatedLayerPaths) {
        auto name = path.stem().string();
        if (const auto index = FindSessionIndexBySourcePath(runtimeState, path);
            index.has_value() && index.value() < runtimeState.sessions.size()) {
            name = runtimeState.sessions[index.value()].displayName;
        }
        names.push_back(name.empty() ? path.filename().string() : name);
    }
    std::sort(names.begin(), names.end());

    std::string summary;
    for (std::size_t index = 0; index < names.size(); ++index) {
        if (index > 0) {
            summary += ", ";
        }
        summary += names[index];
    }
    return summary;
}

std::string AssociationFilterSummary(
    const PreviewRuntimeState& runtimeState,
    AssociationFilterMode filterMode,
    const std::filesystem::path& filterLayerPath) {
    switch (filterMode) {
        case AssociationFilterMode::Any:
            return "Any";
        case AssociationFilterMode::Unassociated:
            return "Unassociated";
        case AssociationFilterMode::Layer:
            if (const auto index = FindSessionIndexBySourcePath(runtimeState, filterLayerPath);
                index.has_value() && index.value() < runtimeState.sessions.size()) {
                return runtimeState.sessions[index.value()].displayName;
            }
            return filterLayerPath.empty() ? std::string{"Layer"} : filterLayerPath.stem().string();
        case AssociationFilterMode::Visible:
        default:
            return "Visible";
    }
}

bool DrawAssociationFilterControl(
    const char* label,
    const PreviewRuntimeState& runtimeState,
    AssociationFilterMode* filterMode,
    std::filesystem::path* filterLayerPath) {
    if (filterMode == nullptr || filterLayerPath == nullptr) {
        return false;
    }

    bool changed = false;
    const auto preview = AssociationFilterSummary(runtimeState, *filterMode, *filterLayerPath);
    if (ImGui::BeginCombo(label, preview.c_str())) {
        const auto drawOption = [&](AssociationFilterMode mode, const char* optionLabel) {
            const bool selected = *filterMode == mode;
            if (ImGui::Selectable(optionLabel, selected)) {
                *filterMode = mode;
                if (mode != AssociationFilterMode::Layer) {
                    filterLayerPath->clear();
                }
                changed = true;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        };

        drawOption(AssociationFilterMode::Visible, "Visible");
        drawOption(AssociationFilterMode::Any, "Any");
        drawOption(AssociationFilterMode::Unassociated, "Unassociated");
        ImGui::Separator();
        for (std::size_t index = 0; index < runtimeState.sessions.size(); ++index) {
            const auto& session = runtimeState.sessions[index];
            if (!IsAssociableLidarSession(session)) {
                continue;
            }
            const bool selected =
                *filterMode == AssociationFilterMode::Layer &&
                AssociatedLayerPathsContain(std::vector<std::filesystem::path>{*filterLayerPath}, session.sourcePath);
            ImGui::PushID(static_cast<int>(index));
            if (ImGui::Selectable(session.displayName.c_str(), selected)) {
                *filterMode = AssociationFilterMode::Layer;
                *filterLayerPath = session.sourcePath.lexically_normal();
                changed = true;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    return changed;
}

bool DrawLayerAssociationControls(
    const char* label,
    PreviewRuntimeState* runtimeState,
    std::vector<std::filesystem::path>* associatedLayerPaths) {
    if (runtimeState == nullptr || associatedLayerPaths == nullptr) {
        return false;
    }

    NormalizeAssociatedLayerPaths(associatedLayerPaths);
    bool changed = false;
    ImGui::TextDisabled("%s: %s", label, FormatAssociationSummary(*runtimeState, *associatedLayerPaths).c_str());
    if (ImGui::Button("Make Unassociated")) {
        if (!associatedLayerPaths->empty()) {
            associatedLayerPaths->clear();
            changed = true;
        }
    }

    std::size_t lidarCount = 0;
    for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
        const auto& session = runtimeState->sessions[index];
        if (!IsAssociableLidarSession(session)) {
            continue;
        }
        ++lidarCount;
        bool associated = AssociatedLayerPathsContain(*associatedLayerPaths, session.sourcePath);
        ImGui::PushID(static_cast<int>(index));
        if (ImGui::Checkbox(session.displayName.c_str(), &associated)) {
            if (associated) {
                associatedLayerPaths->push_back(session.sourcePath);
            } else {
                const auto sourceKey = NormalizePathKey(session.sourcePath);
                associatedLayerPaths->erase(
                    std::remove_if(
                        associatedLayerPaths->begin(),
                        associatedLayerPaths->end(),
                        [&sourceKey](const std::filesystem::path& path) {
                            return NormalizePathKey(path) == sourceKey;
                        }),
                    associatedLayerPaths->end());
            }
            NormalizeAssociatedLayerPaths(associatedLayerPaths);
            changed = true;
        }
        ImGui::PopID();
    }

    if (lidarCount == 0) {
        ImGui::TextDisabled("No LiDAR files are available for association.");
    }
    return changed;
}

invisible_places::io::Float3 BoundsCenter(const invisible_places::io::Bounds3f& bounds) {
    return {
        0.5F * (bounds.minimum.x + bounds.maximum.x),
        0.5F * (bounds.minimum.y + bounds.maximum.y),
        0.5F * (bounds.minimum.z + bounds.maximum.z),
    };
}

std::vector<invisible_places::io::Float3> BuildPivotSamples(
    const std::vector<invisible_places::io::Float3>& points,
    const invisible_places::io::Bounds3f& bounds) {
    (void)bounds;
    if (points.empty()) {
        return {};
    }

    if (points.size() <= kMaxPivotSamples) {
        return points;
    }

    std::vector<invisible_places::io::Float3> samples;
    samples.reserve(kMaxPivotSamples);
    const auto stride = std::max<std::size_t>(1U, (points.size() + kMaxPivotSamples - 1U) / kMaxPivotSamples);
    for (std::size_t index = 0; index < points.size() && samples.size() < kMaxPivotSamples; index += stride) {
        samples.push_back(points[index]);
    }
    return samples;
}

std::optional<invisible_places::io::Float3> FallbackPivot(const PreviewRuntimeState& runtimeState) {
    if (runtimeState.selectedSessionIndex.has_value() &&
        runtimeState.selectedSessionIndex.value() < runtimeState.sessions.size()) {
        const auto& selectedSession = runtimeState.sessions[runtimeState.selectedSessionIndex.value()];
        if (selectedSession.loaded && !IsGeneratedWaterOverlaySession(selectedSession) && selectedSession.hasFocusPoint) {
            return selectedSession.focusPoint;
        }
        if (selectedSession.loaded && !IsGeneratedWaterOverlaySession(selectedSession) && selectedSession.bounds.valid) {
            return BoundsCenter(selectedSession.bounds);
        }
    }

    for (const auto& session : runtimeState.sessions) {
        if (!session.loaded || !session.visible || IsGeneratedWaterOverlaySession(session)) {
            continue;
        }
        if (session.hasFocusPoint) {
            return session.focusPoint;
        }
        if (session.bounds.valid) {
            return BoundsCenter(session.bounds);
        }
    }

    return std::nullopt;
}

struct ScreenRay {
    glm::vec3 origin{0.0F, 0.0F, 0.0F};
    glm::vec3 direction{0.0F, 0.0F, -1.0F};
};

struct ProjectedPoint {
    ImVec2 screen{};
    float depth = 0.0F;
};

struct PivotCandidate {
    glm::vec3 point{0.0F, 0.0F, 0.0F};
    float rayDistance = 0.0F;
    float alongRay = 0.0F;
    float depth = 0.0F;
    float screenDistance = 0.0F;
    float pickRadiusWorld = 0.0F;
};

struct ResolvedPivot {
    invisible_places::io::Float3 point{};
    bool matchedSurface = false;
    std::size_t sampleCount = 0;
};

std::optional<ProjectedPoint> ProjectWorldPoint(
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    const glm::vec3& worldPoint) {
    const auto viewportSize = CurrentUiViewportSize(viewport);
    const auto viewportOrigin = CurrentUiViewportOrigin();
    const float viewportWidth = std::max(1.0F, viewportSize.x);
    const float viewportHeight = std::max(1.0F, viewportSize.y);
    const glm::vec4 clip = matrices.viewProjection * glm::vec4{worldPoint, 1.0F};
    if (clip.w <= 1.0e-6F) {
        return std::nullopt;
    }

    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    if (ndc.x < -1.0F || ndc.x > 1.0F || ndc.y < -1.0F || ndc.y > 1.0F ||
        ndc.z < -1.0F || ndc.z > 1.0F) {
        return std::nullopt;
    }

    return ProjectedPoint{
        .screen =
            ImVec2{
                viewportOrigin.x + ((ndc.x * 0.5F + 0.5F) * viewportWidth),
                viewportOrigin.y + ((ndc.y * 0.5F + 0.5F) * viewportHeight),
            },
        .depth = ndc.z,
    };
}

std::optional<ScreenRay> MakeScreenRay(
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    const auto viewportSize = CurrentUiViewportSize(viewport);
    const float viewportWidth = std::max(1.0F, viewportSize.x);
    const float viewportHeight = std::max(1.0F, viewportSize.y);
    auto localPoint = ToRenderViewportLocal(viewport, screenPoint);
    if (!IsInsideRenderViewport(viewport, screenPoint)) {
        localPoint = ImVec2{viewportWidth * 0.5F, viewportHeight * 0.5F};
    }

    const float ndcX = (localPoint.x / viewportWidth) * 2.0F - 1.0F;
    const float ndcY = (localPoint.y / viewportHeight) * 2.0F - 1.0F;
    const glm::mat4 inverseViewProjection = glm::inverse(matrices.viewProjection);
    glm::vec4 nearWorld = inverseViewProjection * glm::vec4{ndcX, ndcY, -1.0F, 1.0F};
    glm::vec4 farWorld = inverseViewProjection * glm::vec4{ndcX, ndcY, 1.0F, 1.0F};
    if (std::abs(nearWorld.w) <= 1.0e-6F || std::abs(farWorld.w) <= 1.0e-6F) {
        return std::nullopt;
    }

    nearWorld /= nearWorld.w;
    farWorld /= farWorld.w;
    const glm::vec3 direction = glm::vec3{farWorld} - glm::vec3{nearWorld};
    if (glm::length(direction) <= 1.0e-6F) {
        return std::nullopt;
    }

    return ScreenRay{
        .origin = matrices.position,
        .direction = glm::normalize(direction),
    };
}

float MedianValue(std::vector<float> values) {
    if (values.empty()) {
        return 0.0F;
    }

    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
    std::nth_element(values.begin(), middle, values.end());
    if ((values.size() % 2U) != 0U) {
        return *middle;
    }

    const auto lower = std::max_element(values.begin(), middle);
    return 0.5F * (*lower + *middle);
}

glm::vec3 MedianPoint(const std::vector<PivotCandidate>& candidates) {
    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> zs;
    xs.reserve(candidates.size());
    ys.reserve(candidates.size());
    zs.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        xs.push_back(candidate.point.x);
        ys.push_back(candidate.point.y);
        zs.push_back(candidate.point.z);
    }

    return {MedianValue(std::move(xs)), MedianValue(std::move(ys)), MedianValue(std::move(zs))};
}

std::vector<PivotCandidate> SelectPivotDepthCluster(std::vector<PivotCandidate> candidates) {
    if (candidates.size() <= 3U) {
        return candidates;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const PivotCandidate& left, const PivotCandidate& right) {
            if (std::abs(left.alongRay - right.alongRay) > 1.0e-5F) {
                return left.alongRay < right.alongRay;
            }
            if (std::abs(left.screenDistance - right.screenDistance) > 0.5F) {
                return left.screenDistance < right.screenDistance;
            }
            return left.rayDistance < right.rayDistance;
        });

    std::vector<double> screenPrefix(candidates.size() + 1U, 0.0);
    std::vector<double> rayPrefix(candidates.size() + 1U, 0.0);
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        screenPrefix[index + 1U] = screenPrefix[index] + candidates[index].screenDistance;
        rayPrefix[index + 1U] = rayPrefix[index] + candidates[index].rayDistance;
    }

    std::size_t bestStart = 0;
    std::size_t bestEnd = 1;
    double bestMeanScreen = std::numeric_limits<double>::max();
    double bestMeanRay = std::numeric_limits<double>::max();
    for (std::size_t start = 0; start < candidates.size(); ++start) {
        const float depthWindow = std::max(
            candidates[start].pickRadiusWorld * 6.0F,
            std::max(0.001F, candidates[start].alongRay * 0.01F));
        auto end = start + 1U;
        while (end < candidates.size() &&
               candidates[end].alongRay <= candidates[start].alongRay + depthWindow) {
            ++end;
        }

        const auto count = end - start;
        const double meanScreen = (screenPrefix[end] - screenPrefix[start]) / static_cast<double>(count);
        const double meanRay = (rayPrefix[end] - rayPrefix[start]) / static_cast<double>(count);
        const auto bestCount = bestEnd - bestStart;
        if (count > bestCount ||
            (count == bestCount &&
             (meanScreen < bestMeanScreen ||
              (std::abs(meanScreen - bestMeanScreen) <= 0.01 && meanRay < bestMeanRay)))) {
            bestStart = start;
            bestEnd = end;
            bestMeanScreen = meanScreen;
            bestMeanRay = meanRay;
        }
    }

    return {candidates.begin() + static_cast<std::ptrdiff_t>(bestStart),
            candidates.begin() + static_cast<std::ptrdiff_t>(bestEnd)};
}

glm::vec3 BulkCenter(const std::vector<PivotCandidate>& candidates) {
    if (candidates.empty()) {
        return {0.0F, 0.0F, 0.0F};
    }

    if (candidates.size() == 1U) {
        return candidates.front().point;
    }

    const auto median = MedianPoint(candidates);
    std::vector<std::pair<float, std::size_t>> distances;
    distances.reserve(candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        distances.emplace_back(glm::length(candidates[index].point - median), index);
    }
    std::sort(
        distances.begin(),
        distances.end(),
        [](const auto& left, const auto& right) {
            if (std::abs(left.first - right.first) > 1.0e-6F) {
                return left.first < right.first;
            }
            return left.second < right.second;
        });

    const std::size_t keepCount = std::max<std::size_t>(
        1U,
        static_cast<std::size_t>(std::ceil(static_cast<float>(distances.size()) * 0.8F)));
    glm::vec3 sum{0.0F, 0.0F, 0.0F};
    for (std::size_t index = 0; index < keepCount; ++index) {
        sum += candidates[distances[index].second].point;
    }

    const auto center = sum / static_cast<float>(keepCount);
    const auto nearest = std::min_element(
        candidates.begin(),
        candidates.end(),
        [&center](const PivotCandidate& left, const PivotCandidate& right) {
            const float leftDistance = glm::length(left.point - center);
            const float rightDistance = glm::length(right.point - center);
            if (std::abs(leftDistance - rightDistance) > 1.0e-6F) {
                return leftDistance < rightDistance;
            }
            return left.screenDistance < right.screenDistance;
        });
    return nearest != candidates.end() ? nearest->point : center;
}

std::optional<ResolvedPivot> ResolveSurfacePivot(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (!IsInsideRenderViewport(viewport, screenPoint)) {
        screenPoint = CurrentUiViewportCenter(viewport);
    }
    const auto viewportSize = CurrentUiViewportSize(viewport);
    const float viewportHeight = std::max(1.0F, viewportSize.y);

    const auto matrices = runtimeState.camera.Matrices(CurrentAspectRatio(viewport));
    const auto screenRay = MakeScreenRay(matrices, viewport, screenPoint);
    if (!screenRay.has_value()) {
        if (const auto fallback = FallbackPivot(runtimeState); fallback.has_value()) {
            return ResolvedPivot{.point = fallback.value(), .matchedSurface = false};
        }
        return std::nullopt;
    }

    std::vector<PivotCandidate> candidates;
    candidates.reserve(256);
    constexpr float pickRadiusPixels = 48.0F;
    const float tanHalfFov = std::tan(runtimeState.camera.FovDegrees() * kPi / 360.0F);

    for (const auto& session : runtimeState.sessions) {
        if (!session.loaded ||
            !session.visible ||
            session.pivotSamples.empty() ||
            IsGeneratedWaterOverlaySession(session)) {
            continue;
        }

        const glm::mat4 localToWorld = session.kind == LayerKind::GaussianSplat
                                           ? EffectiveGsplatLocalToWorld(runtimeState.projectSettings, session)
                                           : glm::mat4{1.0F};

        for (const auto& sample : session.pivotSamples) {
            const glm::vec4 worldPosition =
                localToWorld * glm::vec4{sample.x, sample.y, sample.z, 1.0F};
            if (std::abs(worldPosition.w) <= 1.0e-6F) {
                continue;
            }

            const glm::vec3 worldPoint = glm::vec3{worldPosition} / worldPosition.w;
            const auto projected = ProjectWorldPoint(matrices, viewport, worldPoint);
            if (!projected.has_value()) {
                continue;
            }

            const glm::vec3 pointFromRayOrigin = worldPoint - screenRay->origin;
            const float alongRay = glm::dot(pointFromRayOrigin, screenRay->direction);
            if (alongRay <= runtimeState.camera.NearPlane()) {
                continue;
            }

            const float dx = projected->screen.x - screenPoint.x;
            const float dy = projected->screen.y - screenPoint.y;
            const float screenDistance = std::sqrt((dx * dx) + (dy * dy));
            if (screenDistance > pickRadiusPixels) {
                continue;
            }

            const glm::vec3 closestPointOnRay = screenRay->origin + (screenRay->direction * alongRay);
            const float rayDistance = glm::length(worldPoint - closestPointOnRay);
            const float worldUnitsPerPixel =
                (2.0F * std::max(0.001F, alongRay) * tanHalfFov) / viewportHeight;
            const float pickRadiusWorld = std::max(0.0001F, worldUnitsPerPixel * pickRadiusPixels);
            if (rayDistance > pickRadiusWorld) {
                continue;
            }

            candidates.push_back(
                {.point = worldPoint,
                 .rayDistance = rayDistance,
                 .alongRay = alongRay,
                 .depth = projected->depth,
                 .screenDistance = screenDistance,
                 .pickRadiusWorld = pickRadiusWorld});
        }
    }

    if (candidates.empty()) {
        if (const auto fallback = FallbackPivot(runtimeState); fallback.has_value()) {
            return ResolvedPivot{.point = fallback.value(), .matchedSurface = false};
        }
        return std::nullopt;
    }

    auto cluster = SelectPivotDepthCluster(std::move(candidates));
    return ResolvedPivot{
        .point = FromGlm(BulkCenter(cluster)),
        .matchedSurface = true,
        .sampleCount = cluster.size()};
}

bool SetCameraPivotFromScreenPoint(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (runtimeState == nullptr) {
        return false;
    }

    const auto pivot = ResolveSurfacePivot(*runtimeState, viewport, screenPoint);
    if (!pivot.has_value()) {
        runtimeState->statusMessage = "No visible surface pivot was available.";
        return false;
    }

    runtimeState->camera.SetOrbitCenterPreservingView(ToGlm(pivot->point));
    runtimeState->pivotOverlay.pivot = pivot->point;
    runtimeState->pivotOverlay.lastSetAt = std::chrono::steady_clock::now();
    runtimeState->cameraPlayback.active = false;
    if (pivot->matchedSurface) {
        runtimeState->statusMessage =
            "Camera pivot set to nearby surface bulk from " +
            FormatPointCount(static_cast<std::uint64_t>(pivot->sampleCount)) + " samples.";
    } else {
        runtimeState->statusMessage = "Camera pivot set to fallback focus point.";
    }
    runtimeState->errorMessage.clear();
    return true;
}

void SyncPivotOverlayToCamera(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    runtimeState->pivotOverlay.visible = true;
    runtimeState->pivotOverlay.pivot = FromGlm(runtimeState->camera.OrbitCenter());
    runtimeState->pivotOverlay.lastSetAt = std::chrono::steady_clock::now();
}

PreviewLayerSession* SelectedSession(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->selectedSessionIndex.has_value()) {
        return nullptr;
    }

    return &runtimeState->sessions[runtimeState->selectedSessionIndex.value()];
}

const PreviewLayerSession* SelectedSession(const PreviewRuntimeState& runtimeState) {
    if (!runtimeState.selectedSessionIndex.has_value()) {
        return nullptr;
    }

    return &runtimeState.sessions[runtimeState.selectedSessionIndex.value()];
}

PreviewLayerSession* SelectedLoadedSession(PreviewRuntimeState* runtimeState) {
    auto* session = SelectedSession(runtimeState);
    return session != nullptr && session->loaded ? session : nullptr;
}

const PreviewLayerSession* SelectedLoadedSession(const PreviewRuntimeState& runtimeState) {
    const auto* session = SelectedSession(runtimeState);
    return session != nullptr && session->loaded ? session : nullptr;
}

PreviewLayerSession* SelectedLoadedSessionOfKind(PreviewRuntimeState* runtimeState, LayerKind kind) {
    auto* session = SelectedLoadedSession(runtimeState);
    return session != nullptr && session->kind == kind ? session : nullptr;
}

const PreviewLayerSession* SelectedLoadedSessionOfKind(const PreviewRuntimeState& runtimeState, LayerKind kind) {
    const auto* session = SelectedLoadedSession(runtimeState);
    return session != nullptr && session->kind == kind ? session : nullptr;
}

bool IsVisibleLoadedPointCloudSession(const PreviewLayerSession& session) {
    return session.kind == LayerKind::PointCloud && session.loaded && session.visible;
}

bool IsLoadedPointCloudSession(const PreviewLayerSession& session) {
    return session.kind == LayerKind::PointCloud && session.loaded;
}

bool IsVisibleLoadedWaterSupportSession(const PreviewLayerSession& session) {
    return IsVisibleLoadedPointCloudSession(session) &&
           session.offlinePointCloud != nullptr &&
           !IsGeneratedWaterOverlaySession(session);
}

std::optional<std::size_t> ResolveLoadedPointCloudLookdevIndex(const PreviewRuntimeState& runtimeState) {
    if (runtimeState.selectedSessionIndex.has_value() &&
        runtimeState.selectedSessionIndex.value() < runtimeState.sessions.size()) {
        const auto selectedIndex = runtimeState.selectedSessionIndex.value();
        if (IsLoadedPointCloudSession(runtimeState.sessions[selectedIndex])) {
            return selectedIndex;
        }
    }

    for (std::size_t index = 0; index < runtimeState.sessions.size(); ++index) {
        if (IsLoadedPointCloudSession(runtimeState.sessions[index])) {
            return index;
        }
    }

    return std::nullopt;
}

std::optional<std::size_t> ResolveVisiblePointCloudLookdevIndex(const PreviewRuntimeState& runtimeState) {
    if (runtimeState.selectedSessionIndex.has_value() &&
        runtimeState.selectedSessionIndex.value() < runtimeState.sessions.size()) {
        const auto selectedIndex = runtimeState.selectedSessionIndex.value();
        if (IsVisibleLoadedPointCloudSession(runtimeState.sessions[selectedIndex])) {
            return selectedIndex;
        }
    }

    for (std::size_t index = 0; index < runtimeState.sessions.size(); ++index) {
        if (IsVisibleLoadedPointCloudSession(runtimeState.sessions[index])) {
            return index;
        }
    }

    return std::nullopt;
}

std::optional<LayerLoadResult> TakeCompletedBackgroundResult(
    const std::shared_ptr<BackgroundLayerLoadState>& backgroundState) {
    if (backgroundState == nullptr) {
        return std::nullopt;
    }

    std::scoped_lock lock(backgroundState->mutex);
    if (!backgroundState->result.has_value()) {
        return std::nullopt;
    }

    auto completed = std::move(backgroundState->result.value());
    backgroundState->result.reset();
    return completed;
}

bool LoadResultSucceeded(const LayerLoadResult& result) {
    return std::visit([](const auto& loadResult) { return loadResult.success; }, result);
}

std::string LoadResultErrorMessage(const LayerLoadResult& result) {
    return std::visit([](const auto& loadResult) { return loadResult.errorMessage; }, result);
}

void FocusSessionLayer(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    std::size_t sessionIndex,
    float distanceScale = 1.0F) {
    if (runtimeState == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return;
    }

    const auto& session = runtimeState->sessions[sessionIndex];
    if (!session.loaded) {
        return;
    }

    const auto effectiveFrame = ComputeEffectiveLayerFrame(*runtimeState, session);
    if (!effectiveFrame.bounds.valid) {
        return;
    }

    runtimeState->camera.FrameBounds(
        effectiveFrame.bounds,
        effectiveFrame.hasFocusPoint ? effectiveFrame.focusPoint : effectiveFrame.bounds.minimum,
        CurrentAspectRatio(viewport),
        distanceScale);
    SyncPivotOverlayToCamera(runtimeState);
}

bool ActivateLoadedPointCloud(
    std::size_t sessionIndex,
    invisible_places::io::LoadedPointCloud cloud,
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return false;
    }

    const bool hadVisibleLayersBefore = VisibleLayerCount(*runtimeState) > 0;
    auto& session = runtimeState->sessions[sessionIndex];
    session.hasSourceRgb = cloud.hasSourceRgb;
    session.hasNormals = cloud.hasNormals;
    session.totalPrimitives = cloud.PointCount();
    session.scalarFields = cloud.scalarFields;
    session.bounds = cloud.bounds;
    session.focusPoint = cloud.focusPoint;
    session.hasFocusPoint = cloud.hasFocusPoint;
    session.pivotSamples = BuildPivotSamples(cloud.positions, cloud.bounds);
    session.loaded = true;
    session.visible = true;

    if (session.pointBudget.totalPoints != session.totalPrimitives) {
        session.pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            cloud,
            session.totalPrimitives);
    } else {
        session.pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            cloud,
            session.pointBudget.activePoints == 0 ? session.totalPrimitives : session.pointBudget.activePoints);
    }
    ClearPreviewLodSampleCache(&session);

    SanitizePointCloudStyle(&session);
    if (session.kind == LayerKind::PointCloud) {
        EnsurePointVisuals(&session);
        UpsertPointVisual(&session, session.selectedPointVisualName, session.pointStyle);
    }

    try {
        viewport->UploadPointCloud(sessionIndex, cloud, session.pointBudget.sampledIndices);
    } catch (const std::exception& error) {
        session.loaded = false;
        session.visible = false;
        session.offlinePointCloud.reset();
        runtimeState->errorMessage = "GPU upload failed: " + std::string{error.what()};
        std::cerr << runtimeState->errorMessage << std::endl;
        return false;
    }

    session.offlinePointCloud =
        std::make_shared<invisible_places::io::LoadedPointCloud>(std::move(cloud));
    if (!IsGeneratedWaterOverlaySession(session)) {
        TryLoadWaterPathCacheForSupport(runtimeState, session);
    }
    const auto sourceKey = NormalizePathKey(session.sourcePath);
    const bool hasCausticRegions = std::any_of(
        runtimeState->water.causticRegions.begin(),
        runtimeState->water.causticRegions.end(),
        [&](const WaterCausticRegion& region) {
            return NormalizePathKey(region.targetLayerSourcePath) == sourceKey &&
                   region.enabled &&
                   region.vertices.size() >= 3U;
        });
    if (hasCausticRegions) {
        RefreshWaterCausticMaskForSession(runtimeState, viewport, sessionIndex);
    }
    runtimeState->selectedSessionIndex = sessionIndex;
    if (!hadVisibleLayersBefore && runtimeState->preserveProjectCameraOnNextLayerActivation) {
        runtimeState->preserveProjectCameraOnNextLayerActivation = false;
        SyncPivotOverlayToCamera(runtimeState);
    } else if (!hadVisibleLayersBefore) {
        constexpr float kStartupFocusDistanceScale = 0.46F;
        FocusSessionLayer(runtimeState, *viewport, sessionIndex, kStartupFocusDistanceScale);
    }

    runtimeState->statusMessage = "Loaded point cloud " + session.displayName + " with " +
                                  FormatPointCount(session.totalPrimitives) + " points.";
    runtimeState->errorMessage.clear();
    std::cout << runtimeState->statusMessage << std::endl;
    return true;
}

bool ActivateLoadedGaussianSplats(
    std::size_t sessionIndex,
    invisible_places::io::LoadedGaussianSplat splats,
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return false;
    }

    const bool hadVisibleLayersBefore = VisibleLayerCount(*runtimeState) > 0;
    auto& session = runtimeState->sessions[sessionIndex];
    session.totalPrimitives = splats.SplatCount();
    session.localBounds = splats.localBounds;
    session.localToWorld = splats.localToWorld;
    session.bounds = splats.bounds;
    session.localFocusPoint = splats.localFocusPoint;
    session.focusPoint = splats.focusPoint;
    session.hasLocalFocusPoint = splats.hasLocalFocusPoint;
    session.hasFocusPoint = splats.hasFocusPoint;
    session.pivotSamples = BuildPivotSamples(splats.centers, splats.localBounds);
    session.loaded = true;
    session.visible = true;

    try {
        viewport->UploadGaussianSplats(sessionIndex, splats);
    } catch (const std::exception& error) {
        session.loaded = false;
        session.visible = false;
        runtimeState->errorMessage = "GPU upload failed: " + std::string{error.what()};
        std::cerr << runtimeState->errorMessage << std::endl;
        return false;
    }

    runtimeState->selectedSessionIndex = sessionIndex;
    if (!hadVisibleLayersBefore && runtimeState->preserveProjectCameraOnNextLayerActivation) {
        runtimeState->preserveProjectCameraOnNextLayerActivation = false;
        SyncPivotOverlayToCamera(runtimeState);
    } else if (!hadVisibleLayersBefore) {
        constexpr float kStartupFocusDistanceScale = 0.46F;
        FocusSessionLayer(runtimeState, *viewport, sessionIndex, kStartupFocusDistanceScale);
    }

    runtimeState->statusMessage = "Loaded gSplat " + session.displayName + " with " +
                                  FormatPointCount(session.totalPrimitives) + " splats.";
    runtimeState->errorMessage.clear();
    std::cout << runtimeState->statusMessage << std::endl;
    return true;
}

void BeginLayerLoad(std::size_t sessionIndex, PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return;
    }

    runtimeState->selectedSessionIndex = sessionIndex;
    auto& session = runtimeState->sessions[sessionIndex];

    if (runtimeState->pendingLoad.has_value()) {
        runtimeState->statusMessage = "Please wait for the current layer to finish loading.";
        return;
    }

    if (session.loaded) {
        runtimeState->statusMessage = session.displayName + " is already loaded.";
        runtimeState->errorMessage.clear();
        return;
    }

    const auto layerKind = session.kind;
    const auto filePath = session.sourcePath;
    const auto transformPath = session.transformPath;
    runtimeState->statusMessage = "Loading " + session.displayName + " in the background...";
    runtimeState->errorMessage.clear();
    std::cout << "Loading " << LayerKindLabel(layerKind) << ": " << filePath.filename().string() << std::endl;

    auto backgroundState = std::make_shared<BackgroundLayerLoadState>();
    std::jthread backgroundThread{
        [backgroundState, layerKind, filePath, transformPath](std::stop_token stopToken) {
            if (stopToken.stop_requested()) {
                return;
            }

            LayerLoadResult result = layerKind == LayerKind::PointCloud
                                         ? LayerLoadResult{invisible_places::io::LoadPointCloud(filePath)}
                                         : LayerLoadResult{
                                               invisible_places::io::LoadGaussianSplat(filePath, transformPath)};
            if (stopToken.stop_requested()) {
                return;
            }

            std::scoped_lock lock(backgroundState->mutex);
            backgroundState->result = std::move(result);
        }};

    runtimeState->pendingLoad = PendingLayerLoad{
        .sessionIndex = sessionIndex,
        .phase = PendingLoadPhase::CpuLoading,
        .backgroundState = std::move(backgroundState),
        .backgroundThread = std::move(backgroundThread),
        .completedResult = std::nullopt,
        .showUploadOverlayFrame = false,
        .startedAt = std::chrono::steady_clock::now(),
    };
}

void PollPendingLayerLoad(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr || !runtimeState->pendingLoad.has_value()) {
        return;
    }

    auto& pendingLoad = runtimeState->pendingLoad.value();

    if (pendingLoad.phase == PendingLoadPhase::CpuLoading) {
        const auto completedResult = TakeCompletedBackgroundResult(pendingLoad.backgroundState);
        if (!completedResult.has_value()) {
            return;
        }

        if (!LoadResultSucceeded(completedResult.value())) {
            runtimeState->statusMessage.clear();
            runtimeState->errorMessage = "Load failed: " + LoadResultErrorMessage(completedResult.value());
            std::cerr << runtimeState->errorMessage << std::endl;
            runtimeState->pendingLoad.reset();
            return;
        }

        pendingLoad.completedResult = std::move(completedResult);
        pendingLoad.phase = PendingLoadPhase::UploadPending;
        pendingLoad.showUploadOverlayFrame = true;
        runtimeState->statusMessage =
            "Uploading " + runtimeState->sessions[pendingLoad.sessionIndex].displayName + " to the GPU...";
        return;
    }

    if (pendingLoad.showUploadOverlayFrame) {
        pendingLoad.showUploadOverlayFrame = false;
        return;
    }

    if (!pendingLoad.completedResult.has_value()) {
        return;
    }

    auto completedLoad = std::move(pendingLoad.completedResult.value());
    pendingLoad.completedResult.reset();
    const auto completedSessionIndex = pendingLoad.sessionIndex;

    std::visit(
        [&](auto&& loadResult) {
            using LoadType = std::decay_t<decltype(loadResult)>;
            if (!loadResult.success) {
                runtimeState->statusMessage.clear();
                runtimeState->errorMessage = "Load failed: " + loadResult.errorMessage;
                std::cerr << runtimeState->errorMessage << std::endl;
                return;
            }

            if constexpr (std::is_same_v<LoadType, invisible_places::io::PointCloudLoadResult>) {
                ActivateLoadedPointCloud(
                    pendingLoad.sessionIndex,
                    std::move(loadResult.cloud),
                    runtimeState,
                    viewport);
            } else {
                ActivateLoadedGaussianSplats(
                    pendingLoad.sessionIndex,
                    std::move(loadResult.splats),
                    runtimeState,
                    viewport);
            }
        },
        std::move(completedLoad));

    runtimeState->pendingLoad.reset();
    if (runtimeState->water.pathCacheLoaded &&
        !runtimeState->water.pathCacheStale &&
        runtimeState->water.activeSupportSessionIndex.has_value() &&
        runtimeState->water.activeSupportSessionIndex.value() == completedSessionIndex) {
        RefreshWaterOverlayFromAnchors(
            runtimeState,
            viewport,
            WaterOverlayRefreshPersistence::InMemoryOnly);
    }
}

void UnloadLayerByIndex(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    std::size_t sessionIndex) {
    if (runtimeState == nullptr || viewport == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return;
    }

    auto& session = runtimeState->sessions[sessionIndex];
    if (!session.loaded) {
        return;
    }

    if (session.kind == LayerKind::PointCloud) {
        viewport->RemovePointCloud(sessionIndex);
    } else {
        viewport->RemoveGaussianSplats(sessionIndex);
    }

    session.loaded = false;
    session.visible = false;
    session.pivotSamples.clear();
    session.offlinePointCloud.reset();
    ClearPreviewLodSampleCache(&session);
}

bool UnloadGeneratedWaterOverlays(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return false;
    }

    bool unloadedAny = false;
    for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
        if (IsGeneratedWaterOverlaySession(runtimeState->sessions[index]) &&
            runtimeState->sessions[index].loaded) {
            UnloadLayerByIndex(runtimeState, viewport, index);
            unloadedAny = true;
        }
    }
    return unloadedAny;
}

std::optional<std::size_t> FindSessionIndexBySourcePath(
    const PreviewRuntimeState& runtimeState,
    const std::filesystem::path& sourcePath);

bool SaveAnimationPathToFile(
    PreviewRuntimeState* runtimeState,
    const AnimationPath& path,
    const std::filesystem::path& outputPath);

const AnimationPath* RegistryAnimationPath(
    const PreviewRuntimeState& runtimeState,
    std::size_t fileIndex);

std::uint32_t NextWaterEmitterId(const PreviewRuntimeState& runtimeState) {
    std::uint32_t nextId = std::max<std::uint32_t>(1U, runtimeState.water.nextEmitterId);
    for (const auto& emitter : runtimeState.water.emitters) {
        nextId = std::max<std::uint32_t>(nextId, emitter.id + 1U);
    }
    return nextId;
}

std::uint32_t NextWaterBasinRegionId(const PreviewRuntimeState& runtimeState) {
    std::uint32_t nextId = std::max<std::uint32_t>(1U, runtimeState.water.nextBasinRegionId);
    for (const auto& region : runtimeState.water.basinRegions) {
        nextId = std::max<std::uint32_t>(nextId, region.id + 1U);
    }
    return nextId;
}

std::uint32_t NextWaterRunoffRegionId(const PreviewRuntimeState& runtimeState) {
    std::uint32_t nextId = std::max<std::uint32_t>(1U, runtimeState.water.nextRunoffRegionId);
    for (const auto& region : runtimeState.water.runoffRegions) {
        nextId = std::max<std::uint32_t>(nextId, region.id + 1U);
    }
    return nextId;
}

std::uint32_t NextWaterCausticRegionId(const PreviewRuntimeState& runtimeState) {
    std::uint32_t nextId = std::max<std::uint32_t>(1U, runtimeState.water.nextCausticRegionId);
    for (const auto& region : runtimeState.water.causticRegions) {
        nextId = std::max<std::uint32_t>(nextId, region.id + 1U);
    }
    return nextId;
}

bool HasPreviewableWaterEmitters(const WaterWorkflowState& water) {
    return std::any_of(
        water.emitters.begin(),
        water.emitters.end(),
        [](const WaterEmitter& emitter) { return emitter.status != WaterEmitterStatus::Disabled; });
}

AnimationPath* CurrentAnimationPath(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->animationPanel.currentPath.has_value()) {
        return nullptr;
    }
    return &runtimeState->animationPanel.currentPath.value();
}

const AnimationPath* CurrentAnimationPath(const PreviewRuntimeState& runtimeState) {
    return runtimeState.animationPanel.currentPath.has_value()
               ? &runtimeState.animationPanel.currentPath.value()
               : nullptr;
}

WaterEmitter* SelectedWaterEmitter(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr ||
        !runtimeState->water.selectedEmitterIndex.has_value() ||
        runtimeState->water.selectedEmitterIndex.value() >= runtimeState->water.emitters.size()) {
        return nullptr;
    }
    return &runtimeState->water.emitters[runtimeState->water.selectedEmitterIndex.value()];
}

const WaterEmitter* SelectedWaterEmitter(const PreviewRuntimeState& runtimeState) {
    if (!runtimeState.water.selectedEmitterIndex.has_value() ||
        runtimeState.water.selectedEmitterIndex.value() >= runtimeState.water.emitters.size()) {
        return nullptr;
    }
    return &runtimeState.water.emitters[runtimeState.water.selectedEmitterIndex.value()];
}

void MarkWaterPathDirty(PreviewRuntimeState* runtimeState, std::optional<std::uint32_t> emitterId = std::nullopt) {
    if (runtimeState == nullptr) {
        return;
    }
    runtimeState->water.pathDirty = true;
    runtimeState->water.pathCacheStale = true;
    runtimeState->water.selectedPathBranchId.reset();
    runtimeState->water.hoveredPathBranchId.reset();
    if (emitterId.has_value()) {
        runtimeState->water.dirtyEmitterIds.insert(emitterId.value());
    } else {
        runtimeState->water.dirtyEmitterIds.clear();
        for (const auto& emitter : runtimeState->water.emitters) {
            runtimeState->water.dirtyEmitterIds.insert(emitter.id);
        }
    }
}

void QueueWaterPreview(PreviewRuntimeState* runtimeState) {
    if (runtimeState != nullptr) {
        MarkWaterPathDirty(runtimeState);
        runtimeState->statusMessage = "Water path bake required.";
        runtimeState->errorMessage.clear();
    }
}

const WaterSourceSettings& ActiveDefaultWaterSourceSettings(const PreviewRuntimeState& runtimeState) {
    return runtimeState.water.tempDefaultSourceSettings.has_value()
               ? runtimeState.water.tempDefaultSourceSettings.value()
               : runtimeState.water.defaultSourceSettings;
}

std::string WaterCustomSettingsName(std::uint32_t emitterId, bool edited = false) {
    std::ostringstream label;
    label << "Custom " << std::setw(2) << std::setfill('0') << emitterId;
    if (edited) {
        label << "_edited";
    }
    return label.str();
}

const WaterEmitter* WaterEmitterById(
    const std::vector<WaterEmitter>& emitters,
    std::uint32_t emitterId) {
    const auto emitterIt = std::find_if(
        emitters.begin(),
        emitters.end(),
        [emitterId](const WaterEmitter& emitter) { return emitter.id == emitterId; });
    return emitterIt == emitters.end() ? nullptr : &*emitterIt;
}

bool WaterEmitterHasSavedCustomSettings(
    const std::vector<WaterEmitter>& emitters,
    std::uint32_t emitterId) {
    const auto* emitter = WaterEmitterById(emitters, emitterId);
    return emitter != nullptr && emitter->sourceSettings.has_value();
}

std::string WaterSourceSettingsAssignmentLabel(
    const WaterEmitter& emitter,
    const std::vector<WaterEmitter>& emitters) {
    if (emitter.tempSourceSettings.has_value()) {
        return WaterCustomSettingsName(emitter.id, true);
    }
    if (emitter.sourceSettingsAssignment == WaterSourceSettingsAssignment::Custom &&
        emitter.sourceSettings.has_value()) {
        return WaterCustomSettingsName(emitter.id);
    }
    if (emitter.sourceSettingsAssignment == WaterSourceSettingsAssignment::LinkedEmitter &&
        emitter.linkedSourceSettingsEmitterId.has_value() &&
        WaterEmitterHasSavedCustomSettings(emitters, emitter.linkedSourceSettingsEmitterId.value())) {
        return WaterCustomSettingsName(emitter.linkedSourceSettingsEmitterId.value());
    }
    return "Default";
}

bool ValidateWaterSourceSettingLinks(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return false;
    }
    bool changed = false;
    for (auto& emitter : runtimeState->water.emitters) {
        if (emitter.sourceSettingsAssignment != WaterSourceSettingsAssignment::LinkedEmitter) {
            emitter.linkedSourceSettingsEmitterId.reset();
            continue;
        }
        const bool validLink =
            emitter.linkedSourceSettingsEmitterId.has_value() &&
            emitter.linkedSourceSettingsEmitterId.value() != emitter.id &&
            WaterEmitterHasSavedCustomSettings(
                runtimeState->water.emitters,
                emitter.linkedSourceSettingsEmitterId.value());
        if (!validLink) {
            emitter.sourceSettingsAssignment = WaterSourceSettingsAssignment::Default;
            emitter.linkedSourceSettingsEmitterId.reset();
            emitter.tempSourceSettings.reset();
            changed = true;
        }
    }
    if (changed) {
        MarkWaterPathDirty(runtimeState);
        runtimeState->statusMessage = "Some linked water source settings were missing and fell back to Default.";
        runtimeState->errorMessage.clear();
    }
    return changed;
}

const WaterSourceSettings& ViewedWaterSourceSettings(const PreviewRuntimeState& runtimeState) {
    if (const auto* selectedEmitter = SelectedWaterEmitter(runtimeState); selectedEmitter != nullptr) {
        return invisible_places::water::ResolveWaterSourceSettings(
            *selectedEmitter,
            runtimeState.water.emitters,
            ActiveDefaultWaterSourceSettings(runtimeState));
    }
    return ActiveDefaultWaterSourceSettings(runtimeState);
}

bool WaterSourceRefreshInputsEqual(
    const WaterSourceSettings& left,
    const WaterSourceSettings& right) {
    return left.path.smoothing == right.path.smoothing &&
           left.trailShape.particleJitter == right.trailShape.particleJitter &&
           left.trailShape.splineAnchorSpacing == right.trailShape.splineAnchorSpacing &&
           left.trailShape.trailLaneCount == right.trailShape.trailLaneCount &&
           left.trailShape.trailLooseness == right.trailShape.trailLooseness &&
           left.trailShape.trailSmoothness == right.trailShape.trailSmoothness;
}

void ApplyWaterSourceSettingsTransition(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    const WaterSourceSettings& before,
    const WaterSourceSettings& after,
    std::optional<std::uint32_t> emitterId = std::nullopt) {
    if (runtimeState == nullptr) {
        return;
    }
    if (!invisible_places::water::WaterSourceBakeInputsEqual(before, after)) {
        MarkWaterPathDirty(runtimeState, emitterId);
        return;
    }
    if (!WaterSourceRefreshInputsEqual(before, after) && viewport != nullptr) {
        RefreshWaterOverlayFromAnchors(runtimeState, viewport);
    }
}

WaterSourceSettings* EnsureEditableWaterSourceSettings(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return nullptr;
    }
    if (auto* emitter = SelectedWaterEmitter(runtimeState); emitter != nullptr) {
        if (!emitter->tempSourceSettings.has_value()) {
            emitter->tempSourceSettings = ViewedWaterSourceSettings(*runtimeState);
        }
        emitter->sourceSettingsAssignment = WaterSourceSettingsAssignment::Custom;
        emitter->linkedSourceSettingsEmitterId.reset();
        return &emitter->tempSourceSettings.value();
    }
    if (!runtimeState->water.tempDefaultSourceSettings.has_value()) {
        runtimeState->water.tempDefaultSourceSettings = ViewedWaterSourceSettings(*runtimeState);
    }
    return &runtimeState->water.tempDefaultSourceSettings.value();
}

void SaveEditableWaterSourceSettings(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr) {
        return;
    }
    if (auto* emitter = SelectedWaterEmitter(runtimeState); emitter != nullptr) {
        if (!emitter->tempSourceSettings.has_value()) {
            runtimeState->statusMessage = "No source temp water settings to save.";
            runtimeState->errorMessage.clear();
            return;
        }
        const auto before = invisible_places::water::ResolveWaterSourceSettings(
            *emitter,
            runtimeState->water.emitters,
            ActiveDefaultWaterSourceSettings(*runtimeState));
        emitter->sourceSettings = emitter->tempSourceSettings.value();
        emitter->tempSourceSettings.reset();
        emitter->sourceSettingsAssignment = WaterSourceSettingsAssignment::Custom;
        emitter->linkedSourceSettingsEmitterId.reset();
        const auto after = invisible_places::water::ResolveWaterSourceSettings(
            *emitter,
            runtimeState->water.emitters,
            ActiveDefaultWaterSourceSettings(*runtimeState));
        ApplyWaterSourceSettingsTransition(runtimeState, viewport, before, after, emitter->id);
        runtimeState->statusMessage =
            "Saved water source settings as " + WaterCustomSettingsName(emitter->id) + ".";
        runtimeState->errorMessage.clear();
        return;
    }
    if (!runtimeState->water.tempDefaultSourceSettings.has_value()) {
        runtimeState->statusMessage = "No default source temp settings to save.";
        runtimeState->errorMessage.clear();
        return;
    }
    const auto before = ActiveDefaultWaterSourceSettings(*runtimeState);
    runtimeState->water.defaultSourceSettings = runtimeState->water.tempDefaultSourceSettings.value();
    runtimeState->water.tempDefaultSourceSettings.reset();
    const auto after = ActiveDefaultWaterSourceSettings(*runtimeState);
    ApplyWaterSourceSettingsTransition(runtimeState, viewport, before, after);
    runtimeState->statusMessage = "Saved edited water source defaults.";
    runtimeState->errorMessage.clear();
}

void DiscardEditableWaterSourceSettings(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr) {
        return;
    }
    if (auto* emitter = SelectedWaterEmitter(runtimeState); emitter != nullptr) {
        const auto before = invisible_places::water::ResolveWaterSourceSettings(
            *emitter,
            runtimeState->water.emitters,
            ActiveDefaultWaterSourceSettings(*runtimeState));
        emitter->tempSourceSettings.reset();
        emitter->linkedSourceSettingsEmitterId.reset();
        emitter->sourceSettingsAssignment = emitter->sourceSettings.has_value()
                                                ? WaterSourceSettingsAssignment::Custom
                                                : WaterSourceSettingsAssignment::Default;
        const auto after = invisible_places::water::ResolveWaterSourceSettings(
            *emitter,
            runtimeState->water.emitters,
            ActiveDefaultWaterSourceSettings(*runtimeState));
        ApplyWaterSourceSettingsTransition(runtimeState, viewport, before, after, emitter->id);
        runtimeState->statusMessage = "Discarded edited water source settings.";
        runtimeState->errorMessage.clear();
        return;
    }
    const auto before = ActiveDefaultWaterSourceSettings(*runtimeState);
    runtimeState->water.tempDefaultSourceSettings.reset();
    const auto after = ActiveDefaultWaterSourceSettings(*runtimeState);
    ApplyWaterSourceSettingsTransition(runtimeState, viewport, before, after);
    runtimeState->statusMessage = "Discarded edited water source defaults.";
    runtimeState->errorMessage.clear();
}

enum class WaterTrailPlaybackQuickPreset {
    FineWhiteThreads,
    LongWhiteVeil,
    SoftDistantDrift
};

constexpr std::string_view kDefaultWaterTrailProfileName = "Default";
constexpr std::string_view kAnimationWaterTrailProfileName = "Animation";

const char* WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset preset) {
    switch (preset) {
        case WaterTrailPlaybackQuickPreset::FineWhiteThreads:
            return "Fine White Threads";
        case WaterTrailPlaybackQuickPreset::LongWhiteVeil:
            return "Long White Veil";
        case WaterTrailPlaybackQuickPreset::SoftDistantDrift:
            return "Soft Distant Drift";
    }
    return "Water Trail";
}

WaterAnimationTrailSettings MakeWaterTrailPlaybackQuickPreset(WaterTrailPlaybackQuickPreset preset) {
    WaterAnimationTrailSettings settings = invisible_places::water::DefaultWaterAnimationTrailSettings();
    switch (preset) {
        case WaterTrailPlaybackQuickPreset::FineWhiteThreads:
            settings.particleDensity = 2.4F;
            settings.particleSpeed = 1.15F;
            settings.colorVariation = 0.18F;
            settings.trailLengthMeters = 0.95F;
            settings.trailSampleSpacingMeters = 0.018F;
            break;
        case WaterTrailPlaybackQuickPreset::LongWhiteVeil:
            settings.particleDensity = 1.25F;
            settings.particleSpeed = 0.82F;
            settings.colorVariation = 0.12F;
            settings.trailLengthMeters = 2.20F;
            settings.trailSampleSpacingMeters = 0.035F;
            break;
        case WaterTrailPlaybackQuickPreset::SoftDistantDrift:
            settings.particleDensity = 0.58F;
            settings.particleSpeed = 0.48F;
            settings.colorVariation = 0.28F;
            settings.trailLengthMeters = 3.80F;
            settings.trailSampleSpacingMeters = 0.085F;
            break;
    }
    return settings;
}

std::string NormalizeWaterAnimationTrailProfileName(std::string_view name) {
    const auto trimmed = TrimText(name);
    return trimmed.empty() ? std::string{kDefaultWaterTrailProfileName} : trimmed;
}

bool IsEditedWaterAnimationTrailProfileName(std::string_view name) {
    return EndsWith(name, kEditedPointVisualSuffix);
}

std::string BaseWaterAnimationTrailProfileName(std::string_view name) {
    if (!IsEditedWaterAnimationTrailProfileName(name)) {
        return NormalizeWaterAnimationTrailProfileName(name);
    }
    return NormalizeWaterAnimationTrailProfileName(name.substr(0, name.size() - kEditedPointVisualSuffix.size()));
}

std::string EditedWaterAnimationTrailProfileName(std::string_view baseName) {
    return BaseWaterAnimationTrailProfileName(baseName) + std::string{kEditedPointVisualSuffix};
}

bool IsProtectedWaterAnimationTrailProfileName(std::string_view name) {
    const auto normalized = NormalizeWaterAnimationTrailProfileName(name);
    return normalized == kDefaultWaterTrailProfileName ||
           normalized == WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::FineWhiteThreads) ||
           normalized == WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::LongWhiteVeil) ||
           normalized == WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::SoftDistantDrift);
}

std::optional<WaterAnimationTrailSettings> BuiltInWaterAnimationTrailProfileSettings(
    const PreviewRuntimeState& runtimeState,
    std::string_view name) {
    const auto normalized = NormalizeWaterAnimationTrailProfileName(name);
    if (normalized == kDefaultWaterTrailProfileName) {
        return runtimeState.water.defaultAnimationTrailSettings;
    }
    if (normalized == WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::FineWhiteThreads)) {
        return MakeWaterTrailPlaybackQuickPreset(WaterTrailPlaybackQuickPreset::FineWhiteThreads);
    }
    if (normalized == WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::LongWhiteVeil)) {
        return MakeWaterTrailPlaybackQuickPreset(WaterTrailPlaybackQuickPreset::LongWhiteVeil);
    }
    if (normalized == WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::SoftDistantDrift)) {
        return MakeWaterTrailPlaybackQuickPreset(WaterTrailPlaybackQuickPreset::SoftDistantDrift);
    }
    return std::nullopt;
}

std::optional<std::size_t> FindWaterAnimationTrailProfileIndex(
    const WaterWorkflowState& water,
    std::string_view name) {
    const auto normalized = NormalizeWaterAnimationTrailProfileName(name);
    for (std::size_t index = 0; index < water.animationTrailProfiles.size(); ++index) {
        if (NormalizeWaterAnimationTrailProfileName(water.animationTrailProfiles[index].name) == normalized) {
            return index;
        }
    }
    return std::nullopt;
}

std::optional<WaterAnimationTrailSettings> CustomWaterAnimationTrailProfileSettings(
    const WaterWorkflowState& water,
    std::string_view name) {
    const auto index = FindWaterAnimationTrailProfileIndex(water, name);
    return index.has_value() ? std::optional<WaterAnimationTrailSettings>{water.animationTrailProfiles[index.value()].settings}
                             : std::nullopt;
}

void UpsertWaterAnimationTrailProfile(
    WaterWorkflowState* water,
    std::string_view name,
    const WaterAnimationTrailSettings& settings) {
    if (water == nullptr) {
        return;
    }
    const auto normalized = NormalizeWaterAnimationTrailProfileName(name);
    if (const auto index = FindWaterAnimationTrailProfileIndex(*water, normalized); index.has_value()) {
        water->animationTrailProfiles[index.value()].name = normalized;
        water->animationTrailProfiles[index.value()].settings = settings;
        return;
    }
    water->animationTrailProfiles.push_back({.name = normalized, .settings = settings});
}

void EnsureWaterAnimationTrailProfiles(WaterWorkflowState* water) {
    if (water == nullptr) {
        return;
    }

    std::vector<SavedWaterAnimationTrailProfileState> profiles;
    profiles.reserve(water->animationTrailProfiles.size());
    for (const auto& profile : water->animationTrailProfiles) {
        const auto normalized = NormalizeWaterAnimationTrailProfileName(profile.name);
        const auto duplicate = std::any_of(
            profiles.begin(),
            profiles.end(),
            [&normalized](const SavedWaterAnimationTrailProfileState& existing) {
                return NormalizeWaterAnimationTrailProfileName(existing.name) == normalized;
            });
        if (IsProtectedWaterAnimationTrailProfileName(normalized) ||
            IsEditedWaterAnimationTrailProfileName(normalized) ||
            duplicate) {
            continue;
        }
        profiles.push_back({.name = normalized, .settings = profile.settings});
    }
    water->animationTrailProfiles = std::move(profiles);
    water->selectedAnimationTrailProfileName =
        NormalizeWaterAnimationTrailProfileName(water->selectedAnimationTrailProfileName);
    if (water->animationTrailProfileNameBuffer.empty()) {
        water->animationTrailProfileNameBuffer =
            BaseWaterAnimationTrailProfileName(water->selectedAnimationTrailProfileName);
    }
}

std::optional<WaterAnimationTrailSettings> WaterAnimationTrailProfileSettingsByName(
    const PreviewRuntimeState& runtimeState,
    std::string_view name) {
    const auto normalized = NormalizeWaterAnimationTrailProfileName(name);
    if (runtimeState.water.editedAnimationTrailProfileSettings.has_value() &&
        normalized == runtimeState.water.selectedAnimationTrailProfileName) {
        return runtimeState.water.editedAnimationTrailProfileSettings.value();
    }
    if (const auto builtIn = BuiltInWaterAnimationTrailProfileSettings(runtimeState, normalized); builtIn.has_value()) {
        return builtIn.value();
    }
    return CustomWaterAnimationTrailProfileSettings(runtimeState.water, normalized);
}

bool WaterAnimationTrailSettingsApproximatelyEqual(
    const WaterAnimationTrailSettings& left,
    const WaterAnimationTrailSettings& right) {
    constexpr float epsilon = 1.0e-4F;
    return std::abs(left.particleDensity - right.particleDensity) <= epsilon &&
           std::abs(left.particleSpeed - right.particleSpeed) <= epsilon &&
           std::abs(left.colorVariation - right.colorVariation) <= epsilon &&
           std::abs(left.trailLengthMeters - right.trailLengthMeters) <= epsilon &&
           std::abs(left.trailSampleSpacingMeters - right.trailSampleSpacingMeters) <= epsilon;
}

std::optional<std::string> FindWaterAnimationTrailProfileNameForSettings(
    const PreviewRuntimeState& runtimeState,
    const WaterAnimationTrailSettings& settings) {
    constexpr WaterTrailPlaybackQuickPreset presets[] = {
        WaterTrailPlaybackQuickPreset::FineWhiteThreads,
        WaterTrailPlaybackQuickPreset::LongWhiteVeil,
        WaterTrailPlaybackQuickPreset::SoftDistantDrift,
    };
    if (WaterAnimationTrailSettingsApproximatelyEqual(
            runtimeState.water.defaultAnimationTrailSettings,
            settings)) {
        return std::string{kDefaultWaterTrailProfileName};
    }
    for (const auto preset : presets) {
        const auto presetSettings = MakeWaterTrailPlaybackQuickPreset(preset);
        if (WaterAnimationTrailSettingsApproximatelyEqual(presetSettings, settings)) {
            return std::string{WaterTrailPlaybackQuickPresetName(preset)};
        }
    }
    for (const auto& profile : runtimeState.water.animationTrailProfiles) {
        if (WaterAnimationTrailSettingsApproximatelyEqual(profile.settings, settings)) {
            return NormalizeWaterAnimationTrailProfileName(profile.name);
        }
    }
    return std::nullopt;
}

const WaterAnimationTrailSettings& ViewedWaterAnimationTrailSettings(const PreviewRuntimeState& runtimeState) {
    if (runtimeState.water.editedAnimationTrailProfileSettings.has_value()) {
        return runtimeState.water.editedAnimationTrailProfileSettings.value();
    }
    if (const auto profileSettings =
            WaterAnimationTrailProfileSettingsByName(runtimeState, runtimeState.water.selectedAnimationTrailProfileName);
        profileSettings.has_value()) {
        static thread_local WaterAnimationTrailSettings cachedSettings;
        cachedSettings = profileSettings.value();
        return cachedSettings;
    }
    if (const auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        if (animationPath->tempWaterAnimationTrailSettings.has_value()) {
            return animationPath->tempWaterAnimationTrailSettings.value();
        }
        if (animationPath->waterAnimationTrailSettings.has_value()) {
            return animationPath->waterAnimationTrailSettings.value();
        }
    }
    if (runtimeState.water.tempDefaultAnimationTrailSettings.has_value()) {
        return runtimeState.water.tempDefaultAnimationTrailSettings.value();
    }
    return runtimeState.water.defaultAnimationTrailSettings;
}

void SelectWaterAnimationTrailProfile(PreviewRuntimeState* runtimeState, std::string_view name) {
    if (runtimeState == nullptr) {
        return;
    }
    const auto normalized = NormalizeWaterAnimationTrailProfileName(name);
    runtimeState->water.selectedAnimationTrailProfileName = normalized;
    runtimeState->water.animationTrailProfileNameBuffer = BaseWaterAnimationTrailProfileName(normalized);
    runtimeState->water.editedAnimationTrailProfileSettings.reset();
    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        animationPath->tempWaterAnimationTrailSettings.reset();
        runtimeState->animationPanel.dirty = true;
    }
    runtimeState->statusMessage = "Selected water trail profile " + normalized + ".";
    runtimeState->errorMessage.clear();
}

WaterAnimationTrailSettings* EnsureEditableWaterAnimationTrailSettings(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return nullptr;
    }
    if (!runtimeState->water.editedAnimationTrailProfileSettings.has_value()) {
        runtimeState->water.editedAnimationTrailProfileSettings = ViewedWaterAnimationTrailSettings(*runtimeState);
        runtimeState->water.selectedAnimationTrailProfileName =
            EditedWaterAnimationTrailProfileName(runtimeState->water.selectedAnimationTrailProfileName);
        runtimeState->water.animationTrailProfileNameBuffer =
            BaseWaterAnimationTrailProfileName(runtimeState->water.selectedAnimationTrailProfileName);
    }
    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        animationPath->tempWaterAnimationTrailSettings = runtimeState->water.editedAnimationTrailProfileSettings.value();
        runtimeState->animationPanel.dirty = true;
    } else {
        runtimeState->water.tempDefaultAnimationTrailSettings =
            runtimeState->water.editedAnimationTrailProfileSettings.value();
    }
    return &runtimeState->water.editedAnimationTrailProfileSettings.value();
}

void SaveCurrentWaterAnimationTrailProfile(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    const auto targetName = NormalizeWaterAnimationTrailProfileName(
        runtimeState->water.animationTrailProfileNameBuffer.empty()
            ? BaseWaterAnimationTrailProfileName(runtimeState->water.selectedAnimationTrailProfileName)
            : runtimeState->water.animationTrailProfileNameBuffer);
    if (IsProtectedWaterAnimationTrailProfileName(targetName) ||
        IsEditedWaterAnimationTrailProfileName(targetName)) {
        runtimeState->errorMessage =
            "Choose a new custom trail profile name; built-in and edited names cannot be overwritten.";
        runtimeState->statusMessage.clear();
        return;
    }

    const auto settings = ViewedWaterAnimationTrailSettings(*runtimeState);
    UpsertWaterAnimationTrailProfile(&runtimeState->water, targetName, settings);
    runtimeState->water.selectedAnimationTrailProfileName = targetName;
    runtimeState->water.animationTrailProfileNameBuffer = targetName;
    runtimeState->water.editedAnimationTrailProfileSettings.reset();
    runtimeState->water.tempDefaultAnimationTrailSettings.reset();
    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        animationPath->waterAnimationTrailSettings = settings;
        animationPath->tempWaterAnimationTrailSettings.reset();
        runtimeState->animationPanel.dirty = true;
    }
    runtimeState->statusMessage = "Saved water trail profile " + targetName + ".";
    runtimeState->errorMessage.clear();
}

void DiscardEditableWaterAnimationTrailSettings(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    if (!runtimeState->water.editedAnimationTrailProfileSettings.has_value()) {
        runtimeState->statusMessage = "No edited water trail profile to discard.";
        runtimeState->errorMessage.clear();
        return;
    }
    runtimeState->water.selectedAnimationTrailProfileName =
        BaseWaterAnimationTrailProfileName(runtimeState->water.selectedAnimationTrailProfileName);
    runtimeState->water.animationTrailProfileNameBuffer =
        runtimeState->water.selectedAnimationTrailProfileName;
    runtimeState->water.editedAnimationTrailProfileSettings.reset();
    runtimeState->water.tempDefaultAnimationTrailSettings.reset();
    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        animationPath->tempWaterAnimationTrailSettings.reset();
        runtimeState->animationPanel.dirty = true;
    }
    runtimeState->statusMessage = "Discarded edited water trail profile.";
    runtimeState->errorMessage.clear();
}

void SyncWaterAnimationTrailProfileFromCurrentAnimation(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    EnsureWaterAnimationTrailProfiles(&runtimeState->water);
    const auto* animationPath = CurrentAnimationPath(*runtimeState);
    if (animationPath == nullptr) {
        if (runtimeState->water.tempDefaultAnimationTrailSettings.has_value()) {
            runtimeState->water.editedAnimationTrailProfileSettings =
                runtimeState->water.tempDefaultAnimationTrailSettings.value();
            runtimeState->water.selectedAnimationTrailProfileName =
                EditedWaterAnimationTrailProfileName(kDefaultWaterTrailProfileName);
        } else {
            runtimeState->water.editedAnimationTrailProfileSettings.reset();
            runtimeState->water.selectedAnimationTrailProfileName = std::string{kDefaultWaterTrailProfileName};
        }
        runtimeState->water.animationTrailProfileNameBuffer =
            BaseWaterAnimationTrailProfileName(runtimeState->water.selectedAnimationTrailProfileName);
        return;
    }

    const auto settings = animationPath->tempWaterAnimationTrailSettings
                              .value_or(animationPath->waterAnimationTrailSettings
                                            .value_or(runtimeState->water.defaultAnimationTrailSettings));
    if (const auto match = FindWaterAnimationTrailProfileNameForSettings(*runtimeState, settings);
        match.has_value()) {
        runtimeState->water.selectedAnimationTrailProfileName = match.value();
        runtimeState->water.editedAnimationTrailProfileSettings.reset();
    } else {
        runtimeState->water.selectedAnimationTrailProfileName =
            EditedWaterAnimationTrailProfileName(kAnimationWaterTrailProfileName);
        runtimeState->water.editedAnimationTrailProfileSettings = settings;
    }
    runtimeState->water.animationTrailProfileNameBuffer =
        BaseWaterAnimationTrailProfileName(runtimeState->water.selectedAnimationTrailProfileName);
}

const WaterCausticLookSettings& ViewedWaterCausticLookSettings(const PreviewRuntimeState& runtimeState) {
    if (const auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        if (animationPath->tempWaterCausticLookSettings.has_value()) {
            return animationPath->tempWaterCausticLookSettings.value();
        }
        if (animationPath->waterCausticLookSettings.has_value()) {
            return animationPath->waterCausticLookSettings.value();
        }
    }
    if (runtimeState.water.tempDefaultCausticLookSettings.has_value()) {
        return runtimeState.water.tempDefaultCausticLookSettings.value();
    }
    return runtimeState.water.defaultCausticLookSettings;
}

WaterCausticLookSettings* EnsureEditableWaterCausticLookSettings(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return nullptr;
    }
    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        if (!animationPath->tempWaterCausticLookSettings.has_value()) {
            animationPath->tempWaterCausticLookSettings = ViewedWaterCausticLookSettings(*runtimeState);
        }
        runtimeState->animationPanel.dirty = true;
        return &animationPath->tempWaterCausticLookSettings.value();
    }
    if (!runtimeState->water.tempDefaultCausticLookSettings.has_value()) {
        runtimeState->water.tempDefaultCausticLookSettings = ViewedWaterCausticLookSettings(*runtimeState);
    }
    return &runtimeState->water.tempDefaultCausticLookSettings.value();
}

void SaveEditableWaterCausticLookSettings(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        if (!animationPath->tempWaterCausticLookSettings.has_value()) {
            runtimeState->statusMessage = "No animation caustic look temp settings to save.";
            runtimeState->errorMessage.clear();
            return;
        }
        animationPath->waterCausticLookSettings = animationPath->tempWaterCausticLookSettings.value();
        animationPath->tempWaterCausticLookSettings.reset();
        runtimeState->animationPanel.dirty = true;
        if (!runtimeState->animationPanel.currentFilePath.empty()) {
            SaveAnimationPathToFile(
                runtimeState,
                *animationPath,
                runtimeState->animationPanel.currentFilePath);
        } else {
            runtimeState->statusMessage = "Saved caustic look to current animation.";
            runtimeState->errorMessage.clear();
        }
        return;
    }
    if (!runtimeState->water.tempDefaultCausticLookSettings.has_value()) {
        runtimeState->statusMessage = "No project caustic look temp default to save.";
        runtimeState->errorMessage.clear();
        return;
    }
    runtimeState->water.defaultCausticLookSettings = runtimeState->water.tempDefaultCausticLookSettings.value();
    runtimeState->water.tempDefaultCausticLookSettings.reset();
    runtimeState->statusMessage = "Saved caustic look to project default.";
    runtimeState->errorMessage.clear();
}

void DiscardEditableWaterCausticLookSettings(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
        animationPath->tempWaterCausticLookSettings.reset();
        runtimeState->animationPanel.dirty = true;
        runtimeState->statusMessage = "Discarded animation caustic look temp settings.";
        runtimeState->errorMessage.clear();
        return;
    }
    runtimeState->water.tempDefaultCausticLookSettings.reset();
    runtimeState->statusMessage = "Discarded project caustic look temp default.";
    runtimeState->errorMessage.clear();
}

PointCloudStyleState ViewedWaterPointVisualStyle(const PreviewRuntimeState& runtimeState) {
    const auto selectedName = NormalizePointVisualName(runtimeState.water.selectedPointVisualName);
    if (const auto protectedStyle = MakeProtectedWaterPointVisualStyle(runtimeState, selectedName);
        protectedStyle.has_value()) {
        return MakeWaterTrailExportStyle(protectedStyle.value());
    }

    if (const auto customIndex = FindWaterProjectVisualIndex(runtimeState.water.pointVisuals, selectedName);
        customIndex.has_value()) {
        return MakeWaterTrailExportStyle(runtimeState.water.pointVisuals[customIndex.value()].style);
    }

    if (runtimeState.water.tempDefaultPointVisualStyle.has_value()) {
        return MakeWaterTrailExportStyle(runtimeState.water.tempDefaultPointVisualStyle.value());
    }
    return MakeWaterTrailExportStyle(runtimeState.water.defaultPointVisualStyle);
}

PointCloudStyleState MakeWaterTrailExportStyle(PointCloudStyleState style) {
    style.flowAnimation = true;
    style.waterPathView = false;
    return style;
}

void SaveWaterPointVisualStyle(PreviewRuntimeState* runtimeState, const PointCloudStyleState& style) {
    if (runtimeState == nullptr) {
        return;
    }
    auto savedStyle = MakeWaterTrailExportStyle(style);
    std::string targetName = NormalizePointVisualName(runtimeState->water.selectedPointVisualName);
    if (targetName.empty() || IsPresetPointVisualName(targetName) || IsEditedPointVisualName(targetName)) {
        targetName = BasePointVisualName(targetName.empty() ? "Water Flow_preset" : targetName);
    }
    UpsertWaterProjectVisual(runtimeState, targetName, savedStyle);
    runtimeState->water.tempDefaultPointVisualStyle.reset();
    ApplyWaterPointVisualStyleToGeneratedSessions(runtimeState);
}

void ApplyWaterPointVisualStyleToGeneratedSessions(PreviewRuntimeState* runtimeState);

void UnloadCurrentAnimationForWaterEditing(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    runtimeState->animationPlayback.active = false;
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPanel.currentPath.reset();
    runtimeState->animationPanel.currentFilePath.clear();
    runtimeState->animationPanel.draftAnimationName.clear();
    runtimeState->animationPanel.selectedKeyIndex.reset();
    runtimeState->animationPanel.scrubAmount = 0.0F;
    runtimeState->animationPanel.dirty = false;
    SyncWaterAnimationTrailProfileFromCurrentAnimation(runtimeState);
    runtimeState->water.pathAnchors = {};
    runtimeState->water.pathDirty = true;
    ApplyWaterPointVisualStyleToGeneratedSessions(runtimeState);
    runtimeState->statusMessage = "Unloaded animation; editing project water defaults.";
    runtimeState->errorMessage.clear();
}

std::optional<std::size_t> ResolveWaterSupportSessionIndex(const PreviewRuntimeState& runtimeState) {
    if (runtimeState.water.activeSupportSessionIndex.has_value() &&
        runtimeState.water.activeSupportSessionIndex.value() < runtimeState.sessions.size()) {
        const auto index = runtimeState.water.activeSupportSessionIndex.value();
        if (IsVisibleLoadedWaterSupportSession(runtimeState.sessions[index])) {
            return index;
        }
    }

    if (runtimeState.selectedSessionIndex.has_value() &&
        runtimeState.selectedSessionIndex.value() < runtimeState.sessions.size()) {
        const auto selectedIndex = runtimeState.selectedSessionIndex.value();
        if (IsVisibleLoadedWaterSupportSession(runtimeState.sessions[selectedIndex])) {
            return selectedIndex;
        }
    }

    for (std::size_t index = 0; index < runtimeState.sessions.size(); ++index) {
        if (IsVisibleLoadedWaterSupportSession(runtimeState.sessions[index])) {
            return index;
        }
    }

    return std::nullopt;
}

void ConfigureWaterFieldBinding(
    RenderParameterBinding* binding,
    std::int32_t fieldSlot,
    const std::string& fieldName,
    float outputMin,
    float outputMax) {
    if (binding == nullptr) {
        return;
    }

    binding->active = true;
    binding->mode = ParameterSourceMode::FieldMapped;
    binding->fieldMap.fieldSlot = fieldSlot;
    binding->fieldMap.fieldName = fieldName;
    binding->fieldMap.inputMin = 0.0F;
    binding->fieldMap.inputMax = 1.0F;
    binding->fieldMap.outputMin = outputMin;
    binding->fieldMap.outputMax = outputMax;
    binding->fieldMap.gamma = 1.0F;
    binding->fieldMap.flags =
        invisible_places::style::FieldMapFlagClamp |
        invisible_places::style::FieldMapFlagUseLayerStats;
}

const char* WaterOverlayViewModeName(WaterOverlayViewMode mode) {
    switch (mode) {
        case WaterOverlayViewMode::Trail:
            return "Trail View";
        case WaterOverlayViewMode::Path:
            return "Path View";
    }
    return "Trail View";
}

PointCloudStyleState MakeWaterOverlayStyle(WaterOverlayViewMode viewMode) {
    PointCloudStyleState style;
    style.geometryMode = PointCloudGeometryMode::ScreenSprites;
    style.depthContribution = PointCloudDepthContribution::None;
    style.falloffProfile = PointCloudFalloffProfile::Gaussian;
    style.colorMode = PointCloudColorMode::SourceRgb;
    style.solidColor = {0.04F, 0.74F, 1.0F, 1.0F};
    style.colorizeColor = {0.05F, 0.82F, 1.0F};
    style.colorizeAmount = 0.0F;
    style.exposure = 1.8F;
    style.gaussianSharpness = 1.65F;
    style.densityScale = 1.0F;
    style.densityClamp = 8.0F;
    style.solidCenters = true;
    style.flowAnimation = true;
    style.waterPathView = viewMode == WaterOverlayViewMode::Path;
    invisible_places::style::SetScalarConstant(
        &style.pointSize,
        style.waterPathView ? 12.0F : 14.0F);
    ConfigureWaterFieldBinding(
        &style.opacity,
        6,
        "confidence",
        0.0F,
        style.waterPathView ? 1.0F : 0.24F);
    invisible_places::style::SetFieldMapFlag(
        &style.opacity.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    ConfigureWaterFieldBinding(
        &style.emissiveStrength,
        7,
        "accumulation",
        style.waterPathView ? 0.65F : 0.0F,
        style.waterPathView ? 1.40F : 0.35F);
    invisible_places::style::SetFieldMapFlag(
        &style.emissiveStrength.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);
    invisible_places::style::SetScalarConstant(&style.colormapPosition, 0.5F);
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.012F);
    return style;
}

PointCloudStyleState MakeWaterOverlayStyle(
    const WaterParticleVisualSettings& legacyVisualSettings,
    WaterOverlayViewMode viewMode) {
    auto style = MakeWaterOverlayStyle(viewMode);
    if (!style.waterPathView) {
        invisible_places::style::SetScalarConstant(
            &style.pointSize,
            std::clamp(legacyVisualSettings.particleSizePixels, 1.0F, 96.0F));
        style.opacity.fieldMap.outputMax =
            std::clamp(legacyVisualSettings.particleOpacity, 0.0F, 1.0F);
        style.emissiveStrength.fieldMap.outputMax =
            std::clamp(legacyVisualSettings.glow, 0.0F, 4.0F);
    }
    return style;
}

PointCloudStyleState MakeWaterOverlayStyle(const WaterParticleVisualSettings& visualSettings) {
    return MakeWaterOverlayStyle(visualSettings, WaterOverlayViewMode::Trail);
}

PointCloudStyleState MakeDefaultWaterPointVisualStyle() {
    return MakeWaterOverlayStyle(WaterOverlayViewMode::Trail);
}

enum class WaterOverlayVisualPreset {
    WhiteNeedleGlow,
    WhiteGoldSurfels,
    SoftMistLines,
    BlueSilverThreads
};

const char* WaterOverlayVisualPresetName(WaterOverlayVisualPreset preset) {
    switch (preset) {
        case WaterOverlayVisualPreset::WhiteNeedleGlow:
            return "White Needle Glow_preset";
        case WaterOverlayVisualPreset::WhiteGoldSurfels:
            return "White Gold Surfels_preset";
        case WaterOverlayVisualPreset::SoftMistLines:
            return "Soft Mist Lines_preset";
        case WaterOverlayVisualPreset::BlueSilverThreads:
            return "Blue Silver Threads_preset";
    }
    return "Water Visual_preset";
}

PointCloudStyleState MakeWaterOverlayVisualPreset(WaterOverlayVisualPreset preset) {
    auto style = MakeWaterOverlayStyle(WaterOverlayViewMode::Trail);
    style.flowAnimation = true;
    style.waterPathView = false;
    style.colorMode = PointCloudColorMode::SolidColor;
    style.falloffProfile = PointCloudFalloffProfile::Gaussian;
    style.depthContribution = PointCloudDepthContribution::None;
    style.solidCenters = false;
    style.densityScale = 1.0F;
    style.densityClamp = 10.0F;
    style.waterStreakAspect = 1.0F;
    ConfigureWaterFieldBinding(&style.opacity, 13, "trail_age", 0.28F, 0.035F);
    invisible_places::style::SetFieldMapFlag(
        &style.opacity.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    ConfigureWaterFieldBinding(&style.emissiveStrength, 7, "accumulation", 0.55F, 1.65F);
    invisible_places::style::SetFieldMapFlag(
        &style.emissiveStrength.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);

    switch (preset) {
        case WaterOverlayVisualPreset::WhiteNeedleGlow:
            style.geometryMode = PointCloudGeometryMode::ScreenSprites;
            style.solidColor = {0.96F, 0.98F, 1.0F, 1.0F};
            style.colorizeColor = {0.92F, 0.98F, 1.0F};
            style.exposure = 2.15F;
            style.gaussianSharpness = 2.35F;
            invisible_places::style::SetScalarConstant(&style.pointSize, 5.5F);
            style.opacity.fieldMap.outputMin = 0.32F;
            style.opacity.fieldMap.outputMax = 0.045F;
            style.emissiveStrength.fieldMap.outputMin = 0.85F;
            style.emissiveStrength.fieldMap.outputMax = 2.20F;
            break;
        case WaterOverlayVisualPreset::WhiteGoldSurfels:
            style.geometryMode = PointCloudGeometryMode::WorldSurfels;
            style.solidColor = {1.0F, 0.91F, 0.62F, 1.0F};
            style.colorizeColor = {1.0F, 0.82F, 0.34F};
            style.exposure = 2.35F;
            style.gaussianSharpness = 2.0F;
            style.waterStreakAspect = 9.0F;
            invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.012F);
            style.opacity.fieldMap.outputMin = 0.38F;
            style.opacity.fieldMap.outputMax = 0.028F;
            style.emissiveStrength.fieldMap.outputMin = 1.10F;
            style.emissiveStrength.fieldMap.outputMax = 2.75F;
            break;
        case WaterOverlayVisualPreset::SoftMistLines:
            style.geometryMode = PointCloudGeometryMode::CameraFacingWorldSprites;
            style.solidColor = {0.82F, 0.91F, 1.0F, 1.0F};
            style.colorizeColor = {0.74F, 0.88F, 1.0F};
            style.exposure = 1.45F;
            style.gaussianSharpness = 0.72F;
            style.densityScale = 0.72F;
            style.waterStreakAspect = 4.0F;
            invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.040F);
            style.opacity.fieldMap.outputMin = 0.11F;
            style.opacity.fieldMap.outputMax = 0.010F;
            style.emissiveStrength.fieldMap.outputMin = 0.18F;
            style.emissiveStrength.fieldMap.outputMax = 0.62F;
            break;
        case WaterOverlayVisualPreset::BlueSilverThreads:
            style.geometryMode = PointCloudGeometryMode::ScreenSprites;
            style.colorMode = PointCloudColorMode::SourceRgb;
            style.solidColor = {0.78F, 0.92F, 1.0F, 1.0F};
            style.colorizeColor = {0.70F, 0.88F, 1.0F};
            style.exposure = 1.85F;
            style.gaussianSharpness = 1.75F;
            invisible_places::style::SetScalarConstant(&style.pointSize, 8.0F);
            style.opacity.fieldMap.outputMin = 0.24F;
            style.opacity.fieldMap.outputMax = 0.030F;
            style.emissiveStrength.fieldMap.outputMin = 0.55F;
            style.emissiveStrength.fieldMap.outputMax = 1.45F;
            break;
    }
    return MakeWaterTrailExportStyle(style);
}

bool IsProtectedWaterPointVisualName(std::string_view name) {
    const auto normalized = NormalizePointVisualName(name);
    return normalized == "Water Flow_preset" ||
           normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::WhiteNeedleGlow) ||
           normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::WhiteGoldSurfels) ||
           normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::SoftMistLines) ||
           normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::BlueSilverThreads);
}

std::optional<PointCloudStyleState> MakeProtectedWaterPointVisualStyle(
    const PreviewRuntimeState& runtimeState,
    std::string_view name) {
    const auto normalized = NormalizePointVisualName(name);
    if (normalized == "Water Flow_preset") {
        return MakeWaterTrailExportStyle(runtimeState.water.defaultPointVisualStyle);
    }
    if (normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::WhiteNeedleGlow)) {
        return MakeWaterOverlayVisualPreset(WaterOverlayVisualPreset::WhiteNeedleGlow);
    }
    if (normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::WhiteGoldSurfels)) {
        return MakeWaterOverlayVisualPreset(WaterOverlayVisualPreset::WhiteGoldSurfels);
    }
    if (normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::SoftMistLines)) {
        return MakeWaterOverlayVisualPreset(WaterOverlayVisualPreset::SoftMistLines);
    }
    if (normalized == WaterOverlayVisualPresetName(WaterOverlayVisualPreset::BlueSilverThreads)) {
        return MakeWaterOverlayVisualPreset(WaterOverlayVisualPreset::BlueSilverThreads);
    }
    return std::nullopt;
}

void SeedWaterFlowBuiltInVisuals(PreviewRuntimeState* runtimeState, PreviewLayerSession* session) {
    if (runtimeState == nullptr || session == nullptr || session->kind != LayerKind::PointCloud) {
        return;
    }

    constexpr std::string_view protectedNames[] = {
        "Water Flow_preset",
        "White Needle Glow_preset",
        "White Gold Surfels_preset",
        "Soft Mist Lines_preset",
        "Blue Silver Threads_preset",
    };
    std::vector<SavedPointVisualState> seeded;
    seeded.reserve(std::size(protectedNames) + runtimeState->water.pointVisuals.size() + session->pointVisuals.size());
    for (const auto protectedName : protectedNames) {
        if (const auto style = MakeProtectedWaterPointVisualStyle(*runtimeState, protectedName);
            style.has_value()) {
            seeded.push_back({.name = std::string{protectedName}, .style = style.value()});
        }
    }

    auto appendCustomVisual = [&](std::string_view name, const PointCloudStyleState& style) {
        const auto normalized = NormalizePointVisualName(name);
        if (!IsWaterProjectCustomVisualName(normalized)) {
            return;
        }
        if (std::any_of(
                seeded.begin(),
                seeded.end(),
                [&normalized](const SavedPointVisualState& existing) {
                    return NormalizePointVisualName(existing.name) == normalized;
                })) {
            return;
        }
        seeded.push_back({.name = normalized, .style = MakeWaterTrailExportStyle(style)});
    };

    for (const auto& visual : runtimeState->water.pointVisuals) {
        appendCustomVisual(visual.name, visual.style);
    }

    for (const auto& visual : session->pointVisuals) {
        const auto normalized = NormalizePointVisualName(visual.name);
        const auto legacyPresetName = NormalizeWaterPointVisualName(normalized);
        if (legacyPresetName != normalized) {
            const auto protectedStyle = MakeProtectedWaterPointVisualStyle(*runtimeState, legacyPresetName);
            if (!protectedStyle.has_value() ||
                PointStylesEqualForSelection(visual.style, protectedStyle.value())) {
                continue;
            }
        }
        if (!IsWaterProjectCustomVisualName(normalized)) {
            continue;
        }
        appendCustomVisual(normalized, visual.style);
        AppendWaterPointVisualIfMissing(&runtimeState->water.pointVisuals, normalized, visual.style);
    }
    session->pointVisuals = std::move(seeded);
    const auto projectSelectedName =
        runtimeState->water.selectedPointVisualName.empty()
            ? std::string{"Water Flow_preset"}
            : NormalizePointVisualName(runtimeState->water.selectedPointVisualName);
    if (FindPointVisualIndex(*session, projectSelectedName).has_value()) {
        session->selectedPointVisualName = projectSelectedName;
    } else {
        const auto rawSelectedName = NormalizePointVisualName(session->selectedPointVisualName);
        const auto legacySelectedName = NormalizeWaterPointVisualName(rawSelectedName);
        session->selectedPointVisualName =
            legacySelectedName != rawSelectedName &&
                    FindPointVisualIndex(*session, rawSelectedName).has_value()
                ? rawSelectedName
                : legacySelectedName;
    }
    if (!FindPointVisualIndex(*session, session->selectedPointVisualName).has_value()) {
        session->selectedPointVisualName =
            FindPointVisualIndex(*session, projectSelectedName).has_value()
                ? projectSelectedName
                : std::string{"Water Flow_preset"};
    }
    const std::string selectedVisualName = session->selectedPointVisualName;
    SelectPointVisual(session, selectedVisualName);
    runtimeState->water.selectedPointVisualName = session->selectedPointVisualName;
    runtimeState->water.pointVisualNameBuffer = BasePointVisualName(session->selectedPointVisualName);
    session->pointStyle.flowAnimation = true;
    session->pointStyle.waterPathView = runtimeState->water.overlayViewMode == WaterOverlayViewMode::Path;
}

void ApplyWaterOverlayVisualPreset(PreviewRuntimeState* runtimeState, PreviewLayerSession* session, WaterOverlayVisualPreset preset) {
    if (runtimeState == nullptr || session == nullptr || session->kind != LayerKind::PointCloud) {
        return;
    }

    const std::string presetName = WaterOverlayVisualPresetName(preset);
    const auto style = MakeWaterOverlayVisualPreset(preset);
    EnsurePointVisuals(session);
    UpsertPointVisual(session, presetName, style);
    session->selectedPointVisualName = presetName;
    session->pointVisualNameBuffer = BasePointVisualName(presetName);
    session->pointStyle = style;
    runtimeState->water.selectedPointVisualName = presetName;
    runtimeState->water.pointVisualNameBuffer = BasePointVisualName(presetName);
    runtimeState->statusMessage = "Selected water visual " + presetName + ".";
    runtimeState->errorMessage.clear();
}

PointCloudStyleState MakeWaterOverlayDisplayStyle(const PreviewRuntimeState& runtimeState) {
    auto style = ViewedWaterPointVisualStyle(runtimeState);
    style.flowAnimation = true;
    style.waterPathView = runtimeState.water.overlayViewMode == WaterOverlayViewMode::Path;
    return style;
}

PointCloudStyleState MakeBasinHazeOverlayStyle() {
    auto style = MakeWaterOverlayStyle(WaterOverlayViewMode::Trail);
    style.geometryMode = PointCloudGeometryMode::CameraFacingWorldSprites;
    style.solidColor = {0.68F, 0.90F, 1.0F, 1.0F};
    style.colorizeColor = {0.72F, 0.92F, 1.0F};
    style.exposure = 1.15F;
    style.gaussianSharpness = 0.80F;
    style.densityScale = 0.70F;
    style.densityClamp = 10.0F;
    style.solidCenters = false;
    invisible_places::style::SetScalarConstant(&style.pointSize, 1.0F);
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.045F);
    ConfigureWaterFieldBinding(&style.opacity, 13, "trail_age", 0.12F, 0.02F);
    invisible_places::style::SetFieldMapFlag(
        &style.opacity.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.04F);
    style.waterPathView = false;
    return style;
}

PointCloudStyleState MakeRunoffOverlayStyle() {
    auto style = MakeWaterOverlayStyle(WaterOverlayViewMode::Trail);
    invisible_places::style::SetScalarConstant(&style.pointSize, 7.0F);
    ConfigureWaterFieldBinding(&style.opacity, 13, "trail_age", 0.32F, 0.03F);
    invisible_places::style::SetFieldMapFlag(
        &style.opacity.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    ConfigureWaterFieldBinding(&style.emissiveStrength, 7, "accumulation", 0.05F, 0.52F);
    invisible_places::style::SetFieldMapFlag(
        &style.emissiveStrength.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    style.waterPathView = false;
    return style;
}

void ApplyWaterOverlayDisplayStyle(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    for (auto& session : runtimeState->sessions) {
        if (!IsGeneratedWaterFlowOverlaySession(session)) {
            continue;
        }
        session.pointStyle.flowAnimation = true;
        session.pointStyle.waterPathView = runtimeState->water.overlayViewMode == WaterOverlayViewMode::Path;
    }
}

void ApplyWaterPointVisualStyleToGeneratedSessions(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    for (auto& session : runtimeState->sessions) {
        if (!IsGeneratedWaterFlowOverlaySession(session)) {
            continue;
        }
        SeedWaterFlowBuiltInVisuals(runtimeState, &session);
    }
}

PointCloudStyleState MakeEffectiveFastBasicStyle(
    const PointCloudStyleState& sourceStyle,
    bool hasSourceRgb,
    bool waterOverlay) {
    auto style = invisible_places::renderer::pointcloud::MakeFastBasicPointCloudStyle(sourceStyle, hasSourceRgb);
    if (waterOverlay && sourceStyle.flowAnimation) {
        style.flowAnimation = true;
        style.waterPathView = sourceStyle.waterPathView;
        if (style.waterPathView) {
            style.pointSize = sourceStyle.pointSize;
        }
    }
    return style;
}

std::size_t AddOrRefreshWaterOverlaySession(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    const std::filesystem::path& overlayPath,
    std::string_view visualName = "Water Flow_preset",
    std::optional<PointCloudStyleState> defaultStyle = std::nullopt) {
    const auto headerResult = invisible_places::io::ParsePlyHeader(overlayPath);
    if (!headerResult.success) {
        if (runtimeState != nullptr) {
            runtimeState->errorMessage = "Water overlay header parse failed: " + headerResult.errorMessage;
        }
        return std::numeric_limits<std::size_t>::max();
    }

    const auto existingIndex = runtimeState != nullptr
                                   ? FindSessionIndexBySourcePath(*runtimeState, overlayPath)
                                   : std::nullopt;
    const auto sessionIndex = existingIndex.value_or(runtimeState->sessions.size());
    if (!existingIndex.has_value()) {
        PreviewLayerSession session;
        session.kind = LayerKind::PointCloud;
        session.sourcePath = overlayPath;
        session.displayName = overlayPath.stem().string();
        runtimeState->sessions.push_back(std::move(session));
    } else if (runtimeState->sessions[sessionIndex].loaded) {
        UnloadLayerByIndex(runtimeState, viewport, sessionIndex);
    }

    auto& session = runtimeState->sessions[sessionIndex];
    session.kind = LayerKind::PointCloud;
    session.sourcePath = overlayPath;
    session.displayName = overlayPath.stem().string();
    session.hasSourceRgb = headerResult.header.HasColorRgb();
    session.totalPrimitives = headerResult.header.vertexCount;
    session.pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
        headerResult.header.vertexCount,
        headerResult.header.vertexCount);
    if (!existingIndex.has_value() || session.pointVisuals.empty()) {
        session.pointStyle = defaultStyle.value_or(MakeWaterOverlayDisplayStyle(*runtimeState));
        session.selectedPointVisualName = NormalizeWaterPointVisualName(visualName);
        session.pointVisualNameBuffer = BasePointVisualName(session.selectedPointVisualName);
        session.pointVisuals.clear();
        session.pointVisuals.push_back({.name = session.selectedPointVisualName, .style = session.pointStyle});
    } else {
        session.pointStyle.flowAnimation = true;
        session.pointStyle.waterPathView =
            NormalizeWaterPointVisualName(visualName) == "Water Flow_preset" &&
            runtimeState->water.overlayViewMode == WaterOverlayViewMode::Path;
        EnsurePointVisuals(&session);
    }
    if (NormalizeWaterPointVisualName(visualName) == "Water Flow_preset") {
        SeedWaterFlowBuiltInVisuals(runtimeState, &session);
    }
    BeginLayerLoad(sessionIndex, runtimeState);
    return sessionIndex;
}

void StoreLiveWaterFlowOverlay(
    PreviewRuntimeState* runtimeState,
    WaterOverlay overlay) {
    if (runtimeState == nullptr) {
        return;
    }
    runtimeState->water.flowOverlay = std::move(overlay);
    ++runtimeState->water.flowOverlayRevision;
    runtimeState->water.pathDebugCacheRevision = 0;
    runtimeState->water.pathDebugPolylines.clear();
}

std::size_t AddOrRefreshWaterFlowOverlaySession(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    const std::filesystem::path& overlayPath,
    WaterOverlay overlay) {
    if (runtimeState == nullptr || viewport == nullptr || overlay.points.empty()) {
        return std::numeric_limits<std::size_t>::max();
    }

    StoreLiveWaterFlowOverlay(runtimeState, std::move(overlay));
    const auto existingIndex = FindSessionIndexBySourcePath(*runtimeState, overlayPath);
    const auto sessionIndex = existingIndex.value_or(runtimeState->sessions.size());
    if (!existingIndex.has_value()) {
        PreviewLayerSession session;
        session.kind = LayerKind::PointCloud;
        session.sourcePath = overlayPath;
        session.displayName = overlayPath.stem().string();
        runtimeState->sessions.push_back(std::move(session));
    }

    auto& session = runtimeState->sessions[sessionIndex];
    session.kind = LayerKind::PointCloud;
    session.sourcePath = overlayPath;
    session.displayName = overlayPath.stem().string();
    if (!existingIndex.has_value() || session.pointVisuals.empty()) {
        session.pointStyle = MakeWaterOverlayDisplayStyle(*runtimeState);
        session.selectedPointVisualName = "Water Flow_preset";
        session.pointVisualNameBuffer = "Water Flow";
        session.pointVisuals.clear();
        session.pointVisuals.push_back({.name = "Water Flow_preset", .style = session.pointStyle});
    } else {
        session.pointStyle.flowAnimation = true;
        session.pointStyle.waterPathView = runtimeState->water.overlayViewMode == WaterOverlayViewMode::Path;
        EnsurePointVisuals(&session);
    }
    SeedWaterFlowBuiltInVisuals(runtimeState, &session);

    auto cloud = invisible_places::water::BuildWaterOverlayPointCloud(
        runtimeState->water.flowOverlay,
        overlayPath,
        session.displayName);
    if (!ActivateLoadedPointCloud(sessionIndex, std::move(cloud), runtimeState, viewport)) {
        return std::numeric_limits<std::size_t>::max();
    }
    return sessionIndex;
}

std::filesystem::path BuildWaterOverlayPath(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& sourceSession) {
    const std::filesystem::path projectPath{runtimeState.persistence.projectFilePath};
    const auto waterDirectory = projectPath.empty()
                                    ? std::filesystem::path{"Saved"} / "water"
                                    : projectPath.parent_path() / "water";
    const auto suffix = "-WaterFlow.ply";
    return waterDirectory / (sourceSession.sourcePath.stem().string() + suffix);
}

std::filesystem::path BuildWaterPathCachePath(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& sourceSession) {
    const std::filesystem::path projectPath{runtimeState.persistence.projectFilePath};
    const auto waterDirectory = projectPath.empty()
                                    ? std::filesystem::path{"Saved"} / "water"
                                    : projectPath.parent_path() / "water";
    return waterDirectory / (sourceSession.sourcePath.stem().string() + "-WaterPathCache.json");
}

std::string WaterSupportSignature(const PreviewLayerSession& sourceSession) {
    std::ostringstream signature;
    signature << sourceSession.sourcePath.generic_string()
              << "|points=" << sourceSession.totalPrimitives
              << "|normals=" << (sourceSession.hasNormals ? 1 : 0);
    if (sourceSession.bounds.valid) {
        signature << "|min="
                  << sourceSession.bounds.minimum.x << ","
                  << sourceSession.bounds.minimum.y << ","
                  << sourceSession.bounds.minimum.z
                  << "|max="
                  << sourceSession.bounds.maximum.x << ","
                  << sourceSession.bounds.maximum.y << ","
                  << sourceSession.bounds.maximum.z;
    }
    return signature.str();
}

const TrailSurfaceIndex* EnsureTrailSurfaceIndexForSupport(
    PreviewRuntimeState* runtimeState,
    const PreviewLayerSession& sourceSession,
    WaterTrailBuildDiagnostics* diagnostics) {
    if (runtimeState == nullptr || sourceSession.offlinePointCloud == nullptr) {
        return nullptr;
    }

    const auto signature = WaterSupportSignature(sourceSession);
    if (runtimeState->water.trailSurfaceIndex != nullptr &&
        runtimeState->water.trailSurfaceIndexSupportSignature == signature &&
        NormalizePathKey(runtimeState->water.trailSurfaceIndexSupportPath) ==
            NormalizePathKey(sourceSession.sourcePath)) {
        if (diagnostics != nullptr) {
            diagnostics->surfaceSampleCount = invisible_places::water::TrailSurfaceIndexSampleCount(
                *runtimeState->water.trailSurfaceIndex);
        }
        return runtimeState->water.trailSurfaceIndex.get();
    }

    runtimeState->water.trailSurfaceIndex =
        invisible_places::water::BuildTrailSurfaceIndex(sourceSession.offlinePointCloud.get());
    runtimeState->water.trailSurfaceIndexSupportSignature = signature;
    runtimeState->water.trailSurfaceIndexSupportPath = sourceSession.sourcePath;
    if (diagnostics != nullptr && runtimeState->water.trailSurfaceIndex != nullptr) {
        diagnostics->surfaceIndexBuildMs = invisible_places::water::TrailSurfaceIndexBuildMilliseconds(
            *runtimeState->water.trailSurfaceIndex);
        diagnostics->surfaceSampleCount = invisible_places::water::TrailSurfaceIndexSampleCount(
            *runtimeState->water.trailSurfaceIndex);
    }
    return runtimeState->water.trailSurfaceIndex == nullptr ? nullptr : runtimeState->water.trailSurfaceIndex.get();
}

void AppendWaterPathBakeSettingsFingerprint(
    std::ostringstream* stream,
    const WaterPathGenerationSettings& settings) {
    if (stream == nullptr) {
        return;
    }
    (*stream) << "|auto=" << (settings.autoTune ? 1 : 0)
              << "|voxel=" << settings.supportVoxelSize
              << "|bridge=" << settings.maxBridgeDistance
              << "|length=" << settings.pathLength
              << "|sample=" << settings.pathSampleSpacing
              << "|branch=" << settings.branching
              << "|coverage=" << settings.coverage
              << "|gap=" << settings.gapTolerance
              << "|steps=" << settings.maxSteps
              << "|samples=" << settings.supportSampleLimit;
}

std::string WaterEmitterSettingsFingerprint(const PreviewRuntimeState& runtimeState) {
    std::ostringstream fingerprint;
    const auto& defaultSettings = ActiveDefaultWaterSourceSettings(runtimeState);
    fingerprint << "default";
    AppendWaterPathBakeSettingsFingerprint(&fingerprint, defaultSettings.path);
    for (const auto& emitter : runtimeState.water.emitters) {
        fingerprint << "|emitter=" << emitter.id
                    << "," << static_cast<int>(emitter.status)
                    << "," << emitter.position.x
                    << "," << emitter.position.y
                    << "," << emitter.position.z
                    << "," << emitter.radius
                    << "," << emitter.strength
                    << "," << emitter.speed
                    << "," << static_cast<int>(emitter.sourceSettingsAssignment);
        const auto& settings = invisible_places::water::ResolveWaterSourceSettings(
            emitter,
            runtimeState.water.emitters,
            defaultSettings);
        AppendWaterPathBakeSettingsFingerprint(&fingerprint, settings.path);
    }
    return fingerprint.str();
}

bool WaterPathCacheMatchesSupportAndSettings(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& sourceSession,
    const WaterPathCache& cache) {
    return NormalizePathKey(cache.supportLayerPath) == NormalizePathKey(sourceSession.sourcePath) &&
           cache.supportSignature == WaterSupportSignature(sourceSession) &&
           cache.emitterSettingsFingerprint == WaterEmitterSettingsFingerprint(runtimeState);
}

bool SaveWaterPathCacheForSupport(
    PreviewRuntimeState* runtimeState,
    const PreviewLayerSession& sourceSession) {
    if (runtimeState == nullptr || runtimeState->water.pathCache.branches.empty()) {
        return false;
    }
    auto cache = runtimeState->water.pathCache;
    cache.supportLayerPath = sourceSession.sourcePath;
    cache.supportSignature = WaterSupportSignature(sourceSession);
    cache.emitterSettingsFingerprint = WaterEmitterSettingsFingerprint(*runtimeState);
    cache.stale = runtimeState->water.pathCacheStale;
    const auto outputPath = BuildWaterPathCachePath(*runtimeState, sourceSession);
    std::string errorMessage;
    if (!invisible_places::serialization::SaveWaterPathCacheDocument(cache, outputPath, &errorMessage)) {
        runtimeState->errorMessage = errorMessage;
        return false;
    }
    runtimeState->water.pathCache = std::move(cache);
    runtimeState->water.pathCacheLoaded = true;
    return true;
}

bool ApplyWaterPathCacheForSupport(
    PreviewRuntimeState* runtimeState,
    const PreviewLayerSession& sourceSession,
    WaterPathCache cache) {
    if (runtimeState == nullptr || cache.branches.empty()) {
        return false;
    }

    const bool matchesSupportAndSettings = WaterPathCacheMatchesSupportAndSettings(
        *runtimeState,
        sourceSession,
        cache);
    cache.stale = cache.stale || !matchesSupportAndSettings;
    runtimeState->water.pathCache = std::move(cache);
    runtimeState->water.pathCacheLoaded = true;
    runtimeState->water.pathCacheStale = runtimeState->water.pathCache.stale;
    runtimeState->water.pathDirty = runtimeState->water.pathCacheStale;
    const auto supportKey = NormalizePathKey(sourceSession.sourcePath);
    for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
        if (NormalizePathKey(runtimeState->sessions[index].sourcePath) == supportKey) {
            runtimeState->water.activeSupportSessionIndex = index;
            break;
        }
    }
    if (!runtimeState->water.pathCacheStale) {
        runtimeState->water.dirtyEmitterIds.clear();
    }
    runtimeState->water.pathAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
        runtimeState->water.pathCache,
        runtimeState->water.emitters,
        ActiveDefaultWaterSourceSettings(*runtimeState));
    return !runtimeState->water.pathCacheStale;
}

bool TryLoadWaterPathCacheForSupport(
    PreviewRuntimeState* runtimeState,
    const PreviewLayerSession& sourceSession) {
    if (runtimeState == nullptr) {
        return false;
    }
    if (runtimeState->water.pathCacheLoaded &&
        !runtimeState->water.pathCache.branches.empty() &&
        WaterPathCacheMatchesSupportAndSettings(*runtimeState, sourceSession, runtimeState->water.pathCache)) {
        auto cache = runtimeState->water.pathCache;
        cache.stale = false;
        return ApplyWaterPathCacheForSupport(runtimeState, sourceSession, std::move(cache));
    }

    const auto inputPath = BuildWaterPathCachePath(*runtimeState, sourceSession);
    std::string errorMessage;
    auto cache = invisible_places::serialization::LoadWaterPathCacheDocument(inputPath, &errorMessage);
    if (!cache.has_value()) {
        return false;
    }
    return ApplyWaterPathCacheForSupport(runtimeState, sourceSession, std::move(cache.value()));
}

std::optional<WaterPathCache> CurrentWaterPathCacheForDocument(const PreviewRuntimeState& runtimeState) {
    if (!runtimeState.water.pathCacheLoaded || runtimeState.water.pathCache.branches.empty()) {
        return std::nullopt;
    }

    auto cache = runtimeState.water.pathCache;
    cache.stale = runtimeState.water.pathCacheStale || cache.stale;
    const auto supportIndex = ResolveWaterSupportSessionIndex(runtimeState);
    if (supportIndex.has_value() && supportIndex.value() < runtimeState.sessions.size()) {
        const auto& sourceSession = runtimeState.sessions[supportIndex.value()];
        cache.supportLayerPath = sourceSession.sourcePath;
        if (!cache.stale) {
            cache.supportSignature = WaterSupportSignature(sourceSession);
            cache.emitterSettingsFingerprint = WaterEmitterSettingsFingerprint(runtimeState);
        }
    }
    return cache;
}

std::filesystem::path BuildWaterFeatureOverlayPath(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& sourceSession,
    const char* suffix) {
    const std::filesystem::path projectPath{runtimeState.persistence.projectFilePath};
    const auto waterDirectory = projectPath.empty()
                                    ? std::filesystem::path{"Saved"} / "water"
                                    : projectPath.parent_path() / "water";
    return waterDirectory / (sourceSession.sourcePath.stem().string() + suffix);
}

std::filesystem::path WaterSourcesPath(const PreviewRuntimeState& runtimeState) {
    const std::filesystem::path projectPath{runtimeState.persistence.projectFilePath};
    return (projectPath.empty() ? std::filesystem::path{"Saved"} : projectPath.parent_path()) /
           "water_sources.json";
}

void SaveWaterSources(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    invisible_places::serialization::WaterSourcesDocument document;
    document.emitters = runtimeState->water.emitters;
    document.basinRegions = runtimeState->water.basinRegions;
    document.runoffRegions = runtimeState->water.runoffRegions;
    document.causticRegions = runtimeState->water.causticRegions;
    document.sourceSettings = runtimeState->water.defaultSourceSettings;
    document.tempSourceSettings = runtimeState->water.tempDefaultSourceSettings;
    document.causticLookSettings = runtimeState->water.defaultCausticLookSettings;
    document.tempCausticLookSettings = runtimeState->water.tempDefaultCausticLookSettings;
    document.pathCache = CurrentWaterPathCacheForDocument(*runtimeState);
    std::string errorMessage;
    const auto outputPath = WaterSourcesPath(*runtimeState);
    if (invisible_places::serialization::SaveWaterSourcesDocument(document, outputPath, &errorMessage)) {
        runtimeState->statusMessage = "Saved water sources to " + outputPath.string() + ".";
        runtimeState->errorMessage.clear();
    } else {
        runtimeState->errorMessage = errorMessage;
        runtimeState->statusMessage.clear();
    }
}

void LoadWaterSources(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr) {
        return;
    }
    std::string errorMessage;
    const auto inputPath = WaterSourcesPath(*runtimeState);
    const auto document = invisible_places::serialization::LoadWaterSourcesDocument(inputPath, &errorMessage);
    if (!document.has_value()) {
        runtimeState->errorMessage = errorMessage;
        runtimeState->statusMessage.clear();
        return;
    }

    runtimeState->water.emitters = document->emitters;
    runtimeState->water.basinRegions = document->basinRegions;
    runtimeState->water.runoffRegions = document->runoffRegions;
    runtimeState->water.causticRegions = document->causticRegions;
    runtimeState->water.defaultSourceSettings = document->sourceSettings;
    runtimeState->water.tempDefaultSourceSettings = document->tempSourceSettings;
    runtimeState->water.defaultCausticLookSettings = document->causticLookSettings;
    runtimeState->water.tempDefaultCausticLookSettings = document->tempCausticLookSettings;
    runtimeState->water.nextEmitterId = NextWaterEmitterId(*runtimeState);
    runtimeState->water.nextBasinRegionId = NextWaterBasinRegionId(*runtimeState);
    runtimeState->water.nextRunoffRegionId = NextWaterRunoffRegionId(*runtimeState);
    runtimeState->water.nextCausticRegionId = NextWaterCausticRegionId(*runtimeState);
    runtimeState->water.selectedEmitterIndex.reset();
    runtimeState->water.selectedBasinRegionIndex.reset();
    runtimeState->water.selectedRunoffRegionIndex.reset();
    runtimeState->water.selectedCausticRegionIndex.reset();
    runtimeState->water.placementArmed = false;
    runtimeState->water.basinRegionPlacementArmed = false;
    runtimeState->water.runoffRegionPlacementArmed = false;
    runtimeState->water.causticRegionPlacementArmed = false;
    runtimeState->water.movingEmitterIndex.reset();
    if (document->pathCache.has_value() && !document->pathCache->branches.empty()) {
        runtimeState->water.pathCache = document->pathCache.value();
        runtimeState->water.pathCacheLoaded = true;
        runtimeState->water.pathCacheStale = runtimeState->water.pathCache.stale;
        runtimeState->water.pathDirty = runtimeState->water.pathCacheStale;
        runtimeState->water.dirtyEmitterIds.clear();
        runtimeState->water.pathAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
            runtimeState->water.pathCache,
            runtimeState->water.emitters,
            ActiveDefaultWaterSourceSettings(*runtimeState));
        if (const auto supportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
            supportIndex.has_value() && supportIndex.value() < runtimeState->sessions.size() &&
            TryLoadWaterPathCacheForSupport(runtimeState, runtimeState->sessions[supportIndex.value()]) &&
            viewport != nullptr) {
            RefreshWaterOverlayFromAnchors(
                runtimeState,
                viewport,
                WaterOverlayRefreshPersistence::InMemoryOnly);
        }
    } else {
        runtimeState->water.pathCache = {};
        runtimeState->water.pathCacheLoaded = false;
        runtimeState->water.pathCacheStale = false;
        MarkWaterPathDirty(runtimeState);
    }
    runtimeState->statusMessage = "Loaded water sources from " + inputPath.string() + ".";
    runtimeState->errorMessage.clear();
    ValidateWaterSourceSettingLinks(runtimeState);
}

void SelectWaterEmitterInViewport(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell&,
    std::size_t emitterIndex) {
    if (runtimeState == nullptr || emitterIndex >= runtimeState->water.emitters.size()) {
        return;
    }

    const auto& emitter = runtimeState->water.emitters[emitterIndex];
    runtimeState->water.selectedEmitterIndex = emitterIndex;
    runtimeState->pivotOverlay.visible = true;
    runtimeState->pivotOverlay.pivot = emitter.position;
    runtimeState->pivotOverlay.lastSetAt = std::chrono::steady_clock::now();
    runtimeState->cameraPlayback.active = false;
    runtimeState->statusMessage = "Selected water source " + emitter.name + " in the viewport.";
    runtimeState->errorMessage.clear();
}

void DeselectWaterEmitter(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    runtimeState->water.selectedEmitterIndex.reset();
    runtimeState->water.movingEmitterIndex.reset();
    runtimeState->water.placementArmed = false;
    runtimeState->pivotOverlay.visible = false;
    runtimeState->cameraPlayback.active = false;
    runtimeState->statusMessage = runtimeState->water.tempDefaultSourceSettings.has_value()
                                      ? "Editing Default_edited water source settings."
                                      : "Editing Default water source settings.";
    runtimeState->errorMessage.clear();
}

bool BakeWaterOverlayForActiveLayer(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return false;
    }
    if (runtimeState->pendingLoad.has_value()) {
        runtimeState->errorMessage = "Please wait for the current layer load before generating water flow.";
        runtimeState->statusMessage.clear();
        return false;
    }
    const auto supportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
    if (!supportIndex.has_value() || supportIndex.value() >= runtimeState->sessions.size()) {
        runtimeState->errorMessage = "Load and show a point-cloud layer before baking water flow.";
        runtimeState->statusMessage.clear();
        return false;
    }
    runtimeState->water.activeSupportSessionIndex = supportIndex.value();

    auto& sourceSession = runtimeState->sessions[supportIndex.value()];
    if (sourceSession.offlinePointCloud == nullptr) {
        runtimeState->errorMessage = "The selected water support layer is not available on CPU.";
        runtimeState->statusMessage.clear();
        return false;
    }

    runtimeState->errorMessage.clear();
    const auto& defaultSourceSettings = ActiveDefaultWaterSourceSettings(*runtimeState);
    if (TryLoadWaterPathCacheForSupport(runtimeState, sourceSession) &&
        !runtimeState->water.pathCacheStale) {
        runtimeState->water.pathEditUndoHiddenBranchIds.clear();
        runtimeState->water.selectedPathBranchId.reset();
        runtimeState->water.hoveredPathBranchId.reset();
        if (!RefreshWaterOverlayFromAnchors(
                runtimeState,
                viewport,
                WaterOverlayRefreshPersistence::SavePathCache,
                WaterTrailBuildQuality::Final)) {
            return false;
        }
        runtimeState->water.pathDirty = false;
        runtimeState->water.dirtyEmitterIds.clear();
        runtimeState->statusMessage =
            "Reused cached water main paths with " +
            FormatPointCount(runtimeState->water.pathAnchors.points.size()) + " path anchors and " +
            FormatPointCount(runtimeState->water.pathCache.branches.size()) + " branches.";
        runtimeState->errorMessage.clear();
        return true;
    }

    runtimeState->statusMessage = "Baking water main paths...";
    runtimeState->water.pathCache = invisible_places::water::GenerateWaterPathCache(
        *sourceSession.offlinePointCloud,
        runtimeState->water.emitters,
        defaultSourceSettings);
    runtimeState->water.pathCache.supportLayerPath = sourceSession.sourcePath;
    runtimeState->water.pathCache.supportSignature = WaterSupportSignature(sourceSession);
    runtimeState->water.pathCache.emitterSettingsFingerprint = WaterEmitterSettingsFingerprint(*runtimeState);
    runtimeState->water.pathCache.stale = false;
    runtimeState->water.pathCacheLoaded = true;
    runtimeState->water.pathCacheStale = false;
    runtimeState->water.pathEditUndoHiddenBranchIds.clear();
    runtimeState->water.selectedPathBranchId.reset();
    runtimeState->water.hoveredPathBranchId.reset();
    runtimeState->water.pathAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
        runtimeState->water.pathCache,
        runtimeState->water.emitters,
        defaultSourceSettings);
    WaterTrailBuildDiagnostics diagnostics;
    const auto* surfaceIndexForTrails =
        EnsureTrailSurfaceIndexForSupport(runtimeState, sourceSession, &diagnostics);
    const auto overlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        runtimeState->water.pathAnchors,
        runtimeState->water.emitters,
        defaultSourceSettings,
        ViewedWaterAnimationTrailSettings(*runtimeState),
        surfaceIndexForTrails,
        WaterTrailBuildQuality::Final,
        &diagnostics);
    runtimeState->water.lastTrailBuildDiagnostics = diagnostics;
    const auto outputPath = BuildWaterOverlayPath(*runtimeState, sourceSession);
    std::string errorMessage;
    if (!invisible_places::water::WriteWaterOverlayPly(overlay, outputPath, &errorMessage)) {
        runtimeState->errorMessage = errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }

    runtimeState->water.lastOverlayPath = outputPath;
    SaveWaterPathCacheForSupport(runtimeState, sourceSession);
    AddOrRefreshWaterFlowOverlaySession(runtimeState, viewport, outputPath, overlay);
    runtimeState->water.pathDirty = false;
    runtimeState->water.dirtyEmitterIds.clear();
    runtimeState->statusMessage =
        "Baked water main paths " + outputPath.filename().string() +
        " with " + FormatPointCount(runtimeState->water.pathAnchors.points.size()) + " path anchors and " +
        FormatPointCount(overlay.points.size()) + " live trail points across " +
        FormatPointCount(runtimeState->water.pathCache.branches.size()) + " branches.";
    runtimeState->errorMessage.clear();
    return true;
}

bool RefreshWaterOverlayFromAnchors(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    WaterOverlayRefreshPersistence persistence,
    WaterTrailBuildQuality quality) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return false;
    }
    if (runtimeState->pendingLoad.has_value()) {
        return false;
    }
    const auto supportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
    if (!supportIndex.has_value() || supportIndex.value() >= runtimeState->sessions.size()) {
        runtimeState->water.pathDirty = true;
        runtimeState->statusMessage = "Water path bake required.";
        runtimeState->errorMessage.clear();
        return false;
    }
    const auto& defaultSourceSettings = ActiveDefaultWaterSourceSettings(*runtimeState);
    if (runtimeState->water.pathCacheLoaded && !runtimeState->water.pathCache.branches.empty()) {
        runtimeState->water.pathAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
            runtimeState->water.pathCache,
            runtimeState->water.emitters,
            defaultSourceSettings);
    }
    if (runtimeState->water.pathAnchors.points.empty()) {
        runtimeState->water.pathDirty = true;
        runtimeState->statusMessage = "Water path bake required.";
        runtimeState->errorMessage.clear();
        return false;
    }
    WaterTrailBuildDiagnostics diagnostics;
    const auto* surfaceIndexForTrails = EnsureTrailSurfaceIndexForSupport(
        runtimeState,
        runtimeState->sessions[supportIndex.value()],
        &diagnostics);
    const auto overlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        runtimeState->water.pathAnchors,
        runtimeState->water.emitters,
        defaultSourceSettings,
        ViewedWaterAnimationTrailSettings(*runtimeState),
        surfaceIndexForTrails,
        quality,
        &diagnostics);
    runtimeState->water.lastTrailBuildDiagnostics = diagnostics;
    const auto outputPath = BuildWaterOverlayPath(*runtimeState, runtimeState->sessions[supportIndex.value()]);
    runtimeState->water.lastOverlayPath = outputPath;
    if (persistence == WaterOverlayRefreshPersistence::SavePathCache && runtimeState->water.pathCacheLoaded) {
        SaveWaterPathCacheForSupport(runtimeState, runtimeState->sessions[supportIndex.value()]);
    }
    AddOrRefreshWaterFlowOverlaySession(runtimeState, viewport, outputPath, overlay);
    runtimeState->statusMessage = "Water trail preview refreshed.";
    runtimeState->errorMessage.clear();
    return true;
}

bool BakeBasinHazeOverlayForActiveLayer(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return false;
    }
    if (!runtimeState->water.selectedBasinRegionIndex.has_value() ||
        runtimeState->water.selectedBasinRegionIndex.value() >= runtimeState->water.basinRegions.size()) {
        runtimeState->errorMessage = "Select a basin region before baking basin steam.";
        runtimeState->statusMessage.clear();
        return false;
    }
    const auto basinIndex = runtimeState->water.selectedBasinRegionIndex.value();
    WaterBasinRegion selectedRegion = runtimeState->water.basinRegions[basinIndex];
    invisible_places::water::RefreshWaterBasinRegionDerivedValues(&selectedRegion);
    if (selectedRegion.hull.size() < 3U) {
        runtimeState->errorMessage = "Close the selected basin region before baking basin steam.";
        runtimeState->statusMessage.clear();
        return false;
    }
    const auto supportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
    if (!supportIndex.has_value() || supportIndex.value() >= runtimeState->sessions.size()) {
        runtimeState->errorMessage = "Load and show a point-cloud layer before baking basin haze.";
        runtimeState->statusMessage.clear();
        return false;
    }
    auto& sourceSession = runtimeState->sessions[supportIndex.value()];
    if (sourceSession.offlinePointCloud == nullptr) {
        runtimeState->errorMessage = "The selected basin support layer is not available on CPU.";
        runtimeState->statusMessage.clear();
        return false;
    }
    const auto overlay = invisible_places::water::GenerateBasinHazeOverlay(
        *sourceSession.offlinePointCloud,
        std::vector<WaterBasinRegion>{selectedRegion});
    const auto outputPath = BuildWaterFeatureOverlayPath(*runtimeState, sourceSession, "-BasinHaze.ply");
    std::string errorMessage;
    if (!invisible_places::water::WriteWaterOverlayPly(overlay, outputPath, &errorMessage)) {
        runtimeState->errorMessage = errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }
    runtimeState->water.lastBasinOverlayPath = outputPath;
    AddOrRefreshWaterOverlaySession(runtimeState, viewport, outputPath, "Basin Haze", MakeBasinHazeOverlayStyle());
    runtimeState->statusMessage =
        "Baked basin steam " + outputPath.filename().string() +
        " with " + FormatPointCount(overlay.points.size()) + " points.";
    runtimeState->errorMessage.clear();
    return true;
}

bool BakeRunoffOverlayForActiveLayer(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return false;
    }
    const auto supportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
    if (!supportIndex.has_value() || supportIndex.value() >= runtimeState->sessions.size()) {
        runtimeState->errorMessage = "Load and show a point-cloud layer before baking runoff.";
        runtimeState->statusMessage.clear();
        return false;
    }
    auto& sourceSession = runtimeState->sessions[supportIndex.value()];
    if (sourceSession.offlinePointCloud == nullptr) {
        runtimeState->errorMessage = "The selected runoff support layer is not available on CPU.";
        runtimeState->statusMessage.clear();
        return false;
    }
    const auto overlay = invisible_places::water::GenerateRunoffOverlay(
        *sourceSession.offlinePointCloud,
        runtimeState->water.runoffRegions,
        ViewedWaterAnimationTrailSettings(*runtimeState));
    const auto outputPath = BuildWaterFeatureOverlayPath(*runtimeState, sourceSession, "-Runoff.ply");
    std::string errorMessage;
    if (!invisible_places::water::WriteWaterOverlayPly(overlay, outputPath, &errorMessage)) {
        runtimeState->errorMessage = errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }
    runtimeState->water.lastRunoffOverlayPath = outputPath;
    AddOrRefreshWaterOverlaySession(runtimeState, viewport, outputPath, "Runoff", MakeRunoffOverlayStyle());
    runtimeState->statusMessage =
        "Baked runoff " + outputPath.filename().string() +
        " with " + FormatPointCount(overlay.points.size()) + " points.";
    runtimeState->errorMessage.clear();
    return true;
}

std::optional<std::size_t> FindScalarFieldByName(
    const std::vector<invisible_places::io::ScalarFieldStats>& fields,
    std::string_view name) {
    const auto target = NormalizeMotionScalarFieldName(name);
    for (std::size_t index = 0; index < fields.size(); ++index) {
        if (NormalizeMotionScalarFieldName(fields[index].name) == target) {
            return index;
        }
    }
    return std::nullopt;
}

void UpsertGeneratedScalarField(
    invisible_places::io::LoadedPointCloud* cloud,
    std::string_view name,
    const std::vector<float>& values) {
    if (cloud == nullptr || values.size() != cloud->PointCount()) {
        return;
    }
    const auto pointCount = cloud->PointCount();
    const auto expectedValueCount = cloud->scalarFields.size() * pointCount;
    if (cloud->scalarFieldValues.size() != expectedValueCount) {
        cloud->scalarFieldValues.resize(expectedValueCount, 0.0F);
    }

    auto slot = FindScalarFieldByName(cloud->scalarFields, name);
    if (!slot.has_value()) {
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::string{name};
        cloud->scalarFields.push_back(stats);
        cloud->scalarFieldValues.resize(cloud->scalarFields.size() * pointCount, 0.0F);
        slot = cloud->scalarFields.size() - 1U;
    }

    auto& stats = cloud->scalarFields[slot.value()];
    stats = {};
    stats.name = std::string{name};
    for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
        const float value = std::isfinite(values[pointIndex]) ? values[pointIndex] : 0.0F;
        cloud->scalarFieldValues[cloud->ScalarFieldValueIndex(slot.value(), pointIndex)] = value;
        stats.Include(value);
    }
}

bool RefreshWaterCausticMaskForSession(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    std::size_t sessionIndex) {
    if (runtimeState == nullptr || viewport == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return false;
    }
    auto& session = runtimeState->sessions[sessionIndex];
    if (!session.loaded || session.kind != LayerKind::PointCloud || session.offlinePointCloud == nullptr) {
        runtimeState->errorMessage = "The caustic target point cloud is not loaded.";
        runtimeState->statusMessage.clear();
        return false;
    }

    const auto targetKey = NormalizePathKey(session.sourcePath);
    std::vector<WaterCausticRegion> targetRegions;
    for (const auto& region : runtimeState->water.causticRegions) {
        if (NormalizePathKey(region.targetLayerSourcePath) == targetKey) {
            targetRegions.push_back(region);
        }
    }
    if (targetRegions.empty()) {
        runtimeState->errorMessage = "No caustic regions target " + session.displayName + ".";
        runtimeState->statusMessage.clear();
        return false;
    }

    auto mask = invisible_places::water::GenerateCausticMask(*session.offlinePointCloud, targetRegions);
    UpsertGeneratedScalarField(session.offlinePointCloud.get(), "caustic_mask", mask.mask);
    UpsertGeneratedScalarField(session.offlinePointCloud.get(), "caustic_edge", mask.edge);
    UpsertGeneratedScalarField(session.offlinePointCloud.get(), "caustic_region_id", mask.regionId);
    UpsertGeneratedScalarField(session.offlinePointCloud.get(), "caustic_plane_distance", mask.planeDistance);
    UpsertGeneratedScalarField(session.offlinePointCloud.get(), "caustic_seed", mask.seed);
    session.scalarFields = session.offlinePointCloud->scalarFields;
    SanitizePointCloudStyle(&session);
    try {
        viewport->UploadPointCloud(sessionIndex, *session.offlinePointCloud, session.pointBudget.sampledIndices);
    } catch (const std::exception& error) {
        runtimeState->errorMessage = "GPU upload failed while refreshing caustic mask: " + std::string{error.what()};
        runtimeState->statusMessage.clear();
        return false;
    }

    for (auto& region : runtimeState->water.causticRegions) {
        if (NormalizePathKey(region.targetLayerSourcePath) == targetKey) {
            region.maskDirty = false;
            region.maskStale = false;
        }
    }
    if (runtimeState->water.selectedCausticRegionIndex.has_value() &&
        runtimeState->water.selectedCausticRegionIndex.value() < runtimeState->water.causticRegions.size()) {
        const auto& selectedRegion =
            runtimeState->water.causticRegions[runtimeState->water.selectedCausticRegionIndex.value()];
        if (NormalizePathKey(selectedRegion.targetLayerSourcePath) == targetKey &&
            selectedRegion.previewTintMode == WaterCausticPreviewTintMode::PulseAfterRefresh) {
            runtimeState->water.causticPreviewTintPulseUntilSeconds[selectedRegion.id] =
                ImGui::GetTime() + 4.0;
        }
    }
    runtimeState->statusMessage =
        "Refreshed caustic selection mask for " + session.displayName + " with " +
        FormatPointCount(mask.affectedPointCount) + " affected points.";
    runtimeState->errorMessage.clear();
    return true;
}

bool RefreshSelectedWaterCausticMask(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr) {
        return false;
    }
    if (!runtimeState->water.selectedCausticRegionIndex.has_value() ||
        runtimeState->water.selectedCausticRegionIndex.value() >= runtimeState->water.causticRegions.size()) {
        runtimeState->errorMessage = "Select a caustic region before refreshing its mask.";
        runtimeState->statusMessage.clear();
        return false;
    }
    const auto& region = runtimeState->water.causticRegions[runtimeState->water.selectedCausticRegionIndex.value()];
    const auto targetIndex = FindSessionIndexBySourcePath(*runtimeState, region.targetLayerSourcePath);
    if (!targetIndex.has_value()) {
        runtimeState->errorMessage = "The caustic target layer is not loaded.";
        runtimeState->statusMessage.clear();
        return false;
    }
    return RefreshWaterCausticMaskForSession(runtimeState, viewport, targetIndex.value());
}

bool PlaceWaterEmitterAtScreenPoint(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (runtimeState == nullptr) {
        return false;
    }

    const auto pivot = ResolveSurfacePivot(*runtimeState, viewport, screenPoint);
    if (!pivot.has_value()) {
        runtimeState->errorMessage = "No visible point-cloud surface was available for water source placement.";
        runtimeState->statusMessage.clear();
        return false;
    }

    WaterEmitter emitter;
    emitter.id = NextWaterEmitterId(*runtimeState);
    emitter.name = "Source " + std::to_string(emitter.id);
    emitter.position = pivot->point;
    const auto& pathSettings = ActiveDefaultWaterSourceSettings(*runtimeState).path;
    emitter.radius = std::max(0.05F, pathSettings.maxBridgeDistance * 0.75F);
    emitter.strength = 1.0F;
    emitter.speed = 1.0F;
    emitter.origin = WaterEmitterOrigin::Manual;
    emitter.status = WaterEmitterStatus::Accepted;
    emitter.confidence = pivot->matchedSurface ? 1.0F : 0.55F;
    runtimeState->water.nextEmitterId = emitter.id + 1U;
    runtimeState->water.emitters.push_back(std::move(emitter));
    runtimeState->water.selectedEmitterIndex = runtimeState->water.emitters.size() - 1U;
    runtimeState->water.placementArmed = false;
    runtimeState->water.movingEmitterIndex.reset();
    QueueWaterPreview(runtimeState);
    runtimeState->pivotOverlay.visible = true;
    runtimeState->pivotOverlay.pivot = pivot->point;
    runtimeState->pivotOverlay.lastSetAt = std::chrono::steady_clock::now();
    runtimeState->statusMessage = "Placed water source from viewport; path bake required.";
    runtimeState->errorMessage.clear();
    return true;
}

bool AddWaterBasinVertexAtScreenPoint(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (runtimeState == nullptr) {
        return false;
    }
    const auto pivot = ResolveSurfacePivot(*runtimeState, viewport, screenPoint);
    if (!pivot.has_value()) {
        runtimeState->errorMessage = "No visible point-cloud surface was available for basin region placement.";
        runtimeState->statusMessage.clear();
        return false;
    }
    if (!runtimeState->water.selectedBasinRegionIndex.has_value() ||
        runtimeState->water.selectedBasinRegionIndex.value() >= runtimeState->water.basinRegions.size()) {
        WaterBasinRegion region;
        region.id = NextWaterBasinRegionId(*runtimeState);
        region.name = "Basin " + std::to_string(region.id);
        runtimeState->water.nextBasinRegionId = region.id + 1U;
        runtimeState->water.basinRegions.push_back(std::move(region));
        runtimeState->water.selectedBasinRegionIndex = runtimeState->water.basinRegions.size() - 1U;
    }
    auto& region = runtimeState->water.basinRegions[runtimeState->water.selectedBasinRegionIndex.value()];
    region.vertices.push_back(pivot->point);
    invisible_places::water::RefreshWaterBasinRegionDerivedValues(&region);
    runtimeState->statusMessage =
        "Added basin vertex " + std::to_string(region.vertices.size()) + " to " + region.name + ".";
    runtimeState->errorMessage.clear();
    return true;
}

bool AddWaterRunoffVertexAtScreenPoint(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (runtimeState == nullptr) {
        return false;
    }
    const auto pivot = ResolveSurfacePivot(*runtimeState, viewport, screenPoint);
    if (!pivot.has_value()) {
        runtimeState->errorMessage = "No visible point-cloud surface was available for runoff region placement.";
        runtimeState->statusMessage.clear();
        return false;
    }
    if (!runtimeState->water.selectedRunoffRegionIndex.has_value() ||
        runtimeState->water.selectedRunoffRegionIndex.value() >= runtimeState->water.runoffRegions.size()) {
        WaterRunoffRegion region;
        region.id = NextWaterRunoffRegionId(*runtimeState);
        region.name = "Runoff " + std::to_string(region.id);
        runtimeState->water.nextRunoffRegionId = region.id + 1U;
        runtimeState->water.runoffRegions.push_back(std::move(region));
        runtimeState->water.selectedRunoffRegionIndex = runtimeState->water.runoffRegions.size() - 1U;
    }
    auto& region = runtimeState->water.runoffRegions[runtimeState->water.selectedRunoffRegionIndex.value()];
    region.vertices.push_back(pivot->point);
    invisible_places::water::RefreshWaterRunoffRegionDerivedValues(&region);
    runtimeState->statusMessage =
        "Added runoff vertex " + std::to_string(region.vertices.size()) + " to " + region.name + ".";
    runtimeState->errorMessage.clear();
    return true;
}

std::optional<std::filesystem::path> SelectedCausticTargetLayerPath(const PreviewRuntimeState& runtimeState) {
    if (runtimeState.selectedSessionIndex.has_value() &&
        runtimeState.selectedSessionIndex.value() < runtimeState.sessions.size()) {
        const auto& session = runtimeState.sessions[runtimeState.selectedSessionIndex.value()];
        if (session.loaded && IsAssociableLidarSession(session)) {
            return session.sourcePath;
        }
    }
    const auto supportIndex = ResolveWaterSupportSessionIndex(runtimeState);
    if (supportIndex.has_value() && supportIndex.value() < runtimeState.sessions.size()) {
        const auto& session = runtimeState.sessions[supportIndex.value()];
        if (session.loaded && IsAssociableLidarSession(session)) {
            return session.sourcePath;
        }
    }
    return std::nullopt;
}

bool AddWaterCausticVertexAtScreenPoint(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (runtimeState == nullptr) {
        return false;
    }
    const auto pivot = ResolveSurfacePivot(*runtimeState, viewport, screenPoint);
    if (!pivot.has_value()) {
        runtimeState->errorMessage = "No visible point-cloud surface was available for caustic region placement.";
        runtimeState->statusMessage.clear();
        return false;
    }
    if (!runtimeState->water.selectedCausticRegionIndex.has_value() ||
        runtimeState->water.selectedCausticRegionIndex.value() >= runtimeState->water.causticRegions.size()) {
        const auto targetPath = SelectedCausticTargetLayerPath(*runtimeState);
        if (!targetPath.has_value()) {
            runtimeState->errorMessage = "Select a loaded LiDAR layer before creating caustics.";
            runtimeState->statusMessage.clear();
            return false;
        }
        WaterCausticRegion region;
        region.id = NextWaterCausticRegionId(*runtimeState);
        region.name = "Caustics " + std::to_string(region.id);
        region.targetLayerSourcePath = targetPath.value();
        runtimeState->water.nextCausticRegionId = region.id + 1U;
        runtimeState->water.causticRegions.push_back(std::move(region));
        runtimeState->water.selectedCausticRegionIndex = runtimeState->water.causticRegions.size() - 1U;
    }
    auto& region = runtimeState->water.causticRegions[runtimeState->water.selectedCausticRegionIndex.value()];
    region.vertices.push_back(pivot->point);
    invisible_places::water::RefreshWaterCausticRegionDerivedValues(&region);
    runtimeState->statusMessage =
        "Added caustic vertex " + std::to_string(region.vertices.size()) + " to " + region.name + ".";
    runtimeState->errorMessage.clear();
    return true;
}

bool MoveWaterEmitterAtScreenPoint(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (runtimeState == nullptr || !runtimeState->water.movingEmitterIndex.has_value()) {
        return false;
    }

    const auto emitterIndex = runtimeState->water.movingEmitterIndex.value();
    if (emitterIndex >= runtimeState->water.emitters.size()) {
        runtimeState->water.movingEmitterIndex.reset();
        runtimeState->errorMessage = "The selected water source is no longer available to move.";
        runtimeState->statusMessage.clear();
        return false;
    }

    const auto pivot = ResolveSurfacePivot(*runtimeState, viewport, screenPoint);
    if (!pivot.has_value()) {
        runtimeState->errorMessage = "No visible point-cloud surface was available for moving the water source.";
        runtimeState->statusMessage.clear();
        return false;
    }

    auto& emitter = runtimeState->water.emitters[emitterIndex];
    emitter.position = pivot->point;
    emitter.confidence = std::max(emitter.confidence, pivot->matchedSurface ? 0.85F : 0.55F);
    runtimeState->water.selectedEmitterIndex = emitterIndex;
    runtimeState->water.movingEmitterIndex.reset();
    runtimeState->water.placementArmed = false;
    MarkWaterPathDirty(runtimeState, emitter.id);
    runtimeState->pivotOverlay.visible = true;
    runtimeState->pivotOverlay.pivot = pivot->point;
    runtimeState->pivotOverlay.lastSetAt = std::chrono::steady_clock::now();
    runtimeState->statusMessage = "Moved water source " + emitter.name + "; path bake required.";
    runtimeState->errorMessage.clear();
    return true;
}

void SuggestWaterEmittersForActiveLayer(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    const auto supportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
    if (!supportIndex.has_value() || supportIndex.value() >= runtimeState->sessions.size()) {
        runtimeState->errorMessage = "Load and show a point-cloud layer before suggesting water sources.";
        runtimeState->statusMessage.clear();
        return;
    }
    const auto& session = runtimeState->sessions[supportIndex.value()];
    if (session.offlinePointCloud == nullptr) {
        runtimeState->errorMessage = "The selected water support layer is not available on CPU.";
        runtimeState->statusMessage.clear();
        return;
    }

    const auto firstId = NextWaterEmitterId(*runtimeState);
    auto suggestions = invisible_places::water::SuggestWaterEmitters(
        *session.offlinePointCloud,
        runtimeState->water.emitters,
        ActiveDefaultWaterSourceSettings(*runtimeState).path,
        firstId,
        runtimeState->water.maxAutoSuggestions);
    if (suggestions.empty()) {
        runtimeState->statusMessage = "No conservative water source candidates were found.";
        runtimeState->errorMessage.clear();
        return;
    }

    runtimeState->water.nextEmitterId = firstId + static_cast<std::uint32_t>(suggestions.size());
    runtimeState->water.emitters.insert(
        runtimeState->water.emitters.end(),
        std::make_move_iterator(suggestions.begin()),
        std::make_move_iterator(suggestions.end()));
    QueueWaterPreview(runtimeState);
    runtimeState->statusMessage =
        "Suggested " + FormatPointCount(runtimeState->water.nextEmitterId - firstId) +
        " conservative water sources; path bake required.";
    runtimeState->errorMessage.clear();
}

void PropagateWaterEmittersToActiveLayer(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    const auto supportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
    if (!supportIndex.has_value() || supportIndex.value() >= runtimeState->sessions.size()) {
        runtimeState->errorMessage = "Load and show a target point-cloud layer before propagating emitters.";
        runtimeState->statusMessage.clear();
        return;
    }
    const auto& targetSession = runtimeState->sessions[supportIndex.value()];
    if (targetSession.offlinePointCloud == nullptr) {
        runtimeState->errorMessage = "The target layer is not available on CPU.";
        runtimeState->statusMessage.clear();
        return;
    }

    std::vector<WaterEmitter> propagated;
    for (const auto& emitter : runtimeState->water.emitters) {
        if (emitter.status == WaterEmitterStatus::Disabled ||
            emitter.origin == WaterEmitterOrigin::Propagated) {
            continue;
        }
        const auto& pathSettings =
            invisible_places::water::ResolveWaterSourceSettings(
                emitter,
                runtimeState->water.emitters,
                ActiveDefaultWaterSourceSettings(*runtimeState)).path;
        const auto snapped = invisible_places::water::SnapEmitterToCloud(
            *targetSession.offlinePointCloud,
            emitter.position,
            pathSettings);
        if (!snapped.has_value()) {
            continue;
        }

        WaterEmitter clone = emitter;
        clone.id = NextWaterEmitterId(*runtimeState) + static_cast<std::uint32_t>(propagated.size());
        clone.name = emitter.name + " snapped";
        clone.position = snapped.value();
        clone.origin = WaterEmitterOrigin::Propagated;
        clone.status = WaterEmitterStatus::Accepted;
        clone.parentId = emitter.id;
        clone.radius = std::max(pathSettings.supportVoxelSize * 3.0F, emitter.radius * 0.45F);
        clone.confidence = std::min(0.95F, emitter.confidence);
        propagated.push_back(std::move(clone));
    }

    if (propagated.empty()) {
        runtimeState->statusMessage = "No emitters were propagated to the active scale.";
        runtimeState->errorMessage.clear();
        return;
    }

    runtimeState->water.nextEmitterId =
        NextWaterEmitterId(*runtimeState) + static_cast<std::uint32_t>(propagated.size());
    runtimeState->water.emitters.insert(
        runtimeState->water.emitters.end(),
        std::make_move_iterator(propagated.begin()),
        std::make_move_iterator(propagated.end()));
    QueueWaterPreview(runtimeState);
    runtimeState->statusMessage = "Snapped accepted emitters to the active support layer; path bake required.";
    runtimeState->errorMessage.clear();
}

ProjectDocument BuildProjectDocument(const PreviewRuntimeState& runtimeState) {
    ProjectDocument document;
    document.projectName = "Invisible Places";
    document.backgroundColor = runtimeState.projectSettings.backgroundColor;
    document.eyeDomeLightingEnabled = runtimeState.projectSettings.eyeDomeLightingEnabled;
    document.eyeDomeLightingThickness = runtimeState.projectSettings.eyeDomeLightingThickness;
    document.constantUpdateView = runtimeState.projectSettings.constantUpdateView;
    document.liveVisualEffects = runtimeState.projectSettings.liveVisualEffects;
    document.sidePanelPinned = runtimeState.sidePanel.pinned;
    document.autoLowerGsplatQualityWhileNavigating =
        runtimeState.projectSettings.autoLowerGsplatQualityWhileNavigating;
    document.pointCloudPreviewLodMode = runtimeState.projectSettings.pointCloudPreviewLodMode;
    document.interactivePointCap = runtimeState.projectSettings.interactivePointCap;
    document.pointCloudRendererMode = runtimeState.projectSettings.pointCloudRendererMode;
    document.cameraState = runtimeState.camera.CaptureState();
    document.cameraShots = runtimeState.cameraShots;
    document.cameraPathShotIndices = runtimeState.cameraPanel.pathShotIndices;
    document.cameraPathDurationFrames = runtimeState.cameraPanel.pathDurationFrames;
    document.lastAnimationPath = runtimeState.animationPanel.currentFilePath;
    document.hasSavedAnimationRegistry = true;
    for (std::size_t index = 0; index < runtimeState.animationPanel.availableFiles.size(); ++index) {
        auto associatedLayerPaths =
            AnimationRegistryAssociationPaths(runtimeState.animationPanel, index);
        NormalizeAssociatedLayerPaths(&associatedLayerPaths);
        document.savedAnimations.push_back(
            {.filePath = runtimeState.animationPanel.availableFiles[index],
             .associatedLayerPaths = std::move(associatedLayerPaths)});
    }
    document.renderJobSettings = runtimeState.renderSettings;
    document.waterEmitters = runtimeState.water.emitters;
    document.waterBasinRegions = runtimeState.water.basinRegions;
    document.waterRunoffRegions = runtimeState.water.runoffRegions;
    document.waterCausticRegions = runtimeState.water.causticRegions;
    document.waterSourceSettings = runtimeState.water.defaultSourceSettings;
    document.tempWaterSourceSettings = runtimeState.water.tempDefaultSourceSettings;
    document.waterAnimationTrailSettings = runtimeState.water.defaultAnimationTrailSettings;
    document.tempWaterAnimationTrailSettings = runtimeState.water.tempDefaultAnimationTrailSettings;
    document.waterAnimationTrailProfiles.clear();
    document.waterAnimationTrailProfiles.reserve(runtimeState.water.animationTrailProfiles.size());
    for (const auto& profile : runtimeState.water.animationTrailProfiles) {
        document.waterAnimationTrailProfiles.push_back({
            .name = NormalizeWaterAnimationTrailProfileName(profile.name),
            .settings = profile.settings,
        });
    }
    document.waterCausticLookSettings = runtimeState.water.defaultCausticLookSettings;
    document.tempWaterCausticLookSettings = runtimeState.water.tempDefaultCausticLookSettings;
    document.waterPointVisualStyle = runtimeState.water.defaultPointVisualStyle;
    document.tempWaterPointVisualStyle = runtimeState.water.tempDefaultPointVisualStyle;
    auto appendWaterProjectVisual = [&document](std::string_view name, const PointCloudStyleState& style) {
        const auto normalized = NormalizePointVisualName(name);
        if (!IsWaterProjectCustomVisualName(normalized)) {
            return;
        }
        const auto existing = std::find_if(
            document.waterPointVisuals.begin(),
            document.waterPointVisuals.end(),
            [&normalized](const ProjectLayerDocument::PointVisual& visual) {
                return NormalizePointVisualName(visual.name) == normalized;
            });
        if (existing != document.waterPointVisuals.end()) {
            return;
        }

        ProjectLayerDocument::PointVisual visual;
        visual.name = normalized;
        visual.style = MakeWaterTrailExportStyle(style);
        document.waterPointVisuals.push_back(std::move(visual));
    };
    for (const auto& visual : runtimeState.water.pointVisuals) {
        appendWaterProjectVisual(visual.name, visual.style);
    }
    document.selectedWaterPointVisualName =
        runtimeState.water.selectedPointVisualName.empty()
            ? std::string{"Water Flow_preset"}
            : NormalizePointVisualName(runtimeState.water.selectedPointVisualName);
    if (document.selectedWaterPointVisualName.empty() ||
        document.selectedWaterPointVisualName == std::string{kDefaultPointVisualName}) {
        document.selectedWaterPointVisualName = "Water Flow_preset";
    } else if (IsEditedPointVisualName(document.selectedWaterPointVisualName)) {
        document.selectedWaterPointVisualName = BasePointVisualName(document.selectedWaterPointVisualName);
    }
    document.waterSettings.path = runtimeState.water.defaultSourceSettings.path;
    document.waterSettings.trail.particleJitter =
        runtimeState.water.defaultSourceSettings.trailShape.particleJitter;
    document.waterSettings.trail.splineAnchorSpacing =
        runtimeState.water.defaultSourceSettings.trailShape.splineAnchorSpacing;
    document.waterSettings.trail.particleDensity =
        runtimeState.water.defaultAnimationTrailSettings.particleDensity;
    document.waterSettings.trail.particleSpeed =
        runtimeState.water.defaultAnimationTrailSettings.particleSpeed;
    document.waterSettings.visual.colorVariation =
        runtimeState.water.defaultAnimationTrailSettings.colorVariation;
    if (runtimeState.water.tempDefaultSourceSettings.has_value() ||
        runtimeState.water.tempDefaultAnimationTrailSettings.has_value()) {
        auto tempBundle = document.waterSettings;
        if (runtimeState.water.tempDefaultSourceSettings.has_value()) {
            tempBundle.path = runtimeState.water.tempDefaultSourceSettings->path;
            tempBundle.trail.particleJitter =
                runtimeState.water.tempDefaultSourceSettings->trailShape.particleJitter;
            tempBundle.trail.splineAnchorSpacing =
                runtimeState.water.tempDefaultSourceSettings->trailShape.splineAnchorSpacing;
        }
        if (runtimeState.water.tempDefaultAnimationTrailSettings.has_value()) {
            tempBundle.trail.particleDensity =
                runtimeState.water.tempDefaultAnimationTrailSettings->particleDensity;
            tempBundle.trail.particleSpeed =
                runtimeState.water.tempDefaultAnimationTrailSettings->particleSpeed;
            tempBundle.visual.colorVariation =
                runtimeState.water.tempDefaultAnimationTrailSettings->colorVariation;
        }
        document.tempWaterSettings = tempBundle;
    }
    document.waterBakeSettings = runtimeState.water.defaultSourceSettings.path;
    document.waterRenderSettings = document.waterSettings;
    document.waterPathCache = CurrentWaterPathCacheForDocument(runtimeState);

    if (runtimeState.selectedSessionIndex.has_value()) {
        document.selectedLayerPath =
            runtimeState.sessions[runtimeState.selectedSessionIndex.value()].sourcePath;
    }

    document.layers.reserve(runtimeState.sessions.size());
    for (const auto& session : runtimeState.sessions) {
        if (IsGeneratedWaterFlowOverlaySession(session)) {
            for (const auto& visual : session.pointVisuals) {
                const auto normalized = NormalizePointVisualName(visual.name);
                const auto legacyPresetName = NormalizeWaterPointVisualName(normalized);
                if (legacyPresetName != normalized) {
                    const auto protectedStyle = MakeProtectedWaterPointVisualStyle(runtimeState, legacyPresetName);
                    if (!protectedStyle.has_value() ||
                        PointStylesEqualForSelection(visual.style, protectedStyle.value())) {
                        continue;
                    }
                }
                appendWaterProjectVisual(normalized, visual.style);
            }
        }

        ProjectLayerDocument layerDocument;
        layerDocument.kind = session.kind == LayerKind::GaussianSplat
                                 ? invisible_places::serialization::SerializedLayerKind::GaussianSplat
                                 : invisible_places::serialization::SerializedLayerKind::PointCloud;
        layerDocument.sourcePath = session.sourcePath;
        layerDocument.loaded = session.loaded;
        layerDocument.visible = session.visible;
        if (session.kind == LayerKind::PointCloud) {
            layerDocument.pointBudgetActivePoints = session.pointBudget.activePoints;
            layerDocument.pointStyle = session.pointStyle;
            layerDocument.selectedPointVisualName =
                session.selectedPointVisualName.empty()
                    ? std::string{kDefaultPointVisualName}
                    : session.selectedPointVisualName;
            if (session.pointVisuals.empty()) {
                ProjectLayerDocument::PointVisual visual;
                visual.name = std::string{kDefaultPointVisualName};
                visual.style = session.pointStyle;
                layerDocument.pointVisuals.push_back(std::move(visual));
            } else {
                layerDocument.pointVisuals.reserve(session.pointVisuals.size());
                for (const auto& visual : session.pointVisuals) {
                    ProjectLayerDocument::PointVisual visualDocument;
                    visualDocument.name = NormalizePointVisualName(visual.name);
                    visualDocument.style = visual.style;
                    layerDocument.pointVisuals.push_back(std::move(visualDocument));
                }
            }
        }
        document.layers.push_back(std::move(layerDocument));
    }

    return document;
}

std::optional<std::size_t> FindSessionIndexBySourcePath(
    const PreviewRuntimeState& runtimeState,
    const std::filesystem::path& sourcePath) {
    const auto targetKey = NormalizePathKey(sourcePath);
    for (std::size_t index = 0; index < runtimeState.sessions.size(); ++index) {
        if (NormalizePathKey(runtimeState.sessions[index].sourcePath) == targetKey) {
            return index;
        }
    }

    return std::nullopt;
}

void StartQueuedLayerLoadIfIdle(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr ||
        runtimeState->pendingLoad.has_value() ||
        runtimeState->persistence.queuedLoadIndices.empty()) {
        return;
    }

    const auto nextSessionIndex = runtimeState->persistence.queuedLoadIndices.front();
    runtimeState->persistence.queuedLoadIndices.erase(runtimeState->persistence.queuedLoadIndices.begin());
    BeginLayerLoad(nextSessionIndex, runtimeState);
}

void StopBackgroundWorkForShutdown(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    runtimeState->persistence.queuedLoadIndices.clear();
    if (runtimeState->offlineRenderJob.active) {
        std::cout << "Requesting animation export shutdown..." << std::endl;
        runtimeState->offlineRenderJob.cancelRequested = true;
        RequestAnimationExportWriterCancellation(&runtimeState->offlineRenderJob);
        if (runtimeState->offlineRenderJob.worker.joinable()) {
            runtimeState->offlineRenderJob.worker = std::jthread{};
        }
    }

    if (!runtimeState->pendingLoad.has_value()) {
        return;
    }

    std::cout << "Waiting for background layer load to finish before shutdown..." << std::endl;
    runtimeState->pendingLoad->backgroundThread.request_stop();
    runtimeState->pendingLoad.reset();
}

bool ApplyProjectDocumentToRuntime(
    const ProjectDocument& document,
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return false;
    }

    if (runtimeState->pendingLoad.has_value()) {
        runtimeState->statusMessage = "Please wait for the current layer to finish loading before loading a project.";
        return false;
    }

    runtimeState->projectSettings.backgroundColor = document.backgroundColor;
    runtimeState->projectSettings.eyeDomeLightingEnabled = document.eyeDomeLightingEnabled;
    runtimeState->projectSettings.eyeDomeLightingThickness =
        std::clamp(document.eyeDomeLightingThickness, 1.0F, 24.0F);
    runtimeState->projectSettings.constantUpdateView = document.constantUpdateView;
    runtimeState->projectSettings.liveVisualEffects = document.liveVisualEffects;
    runtimeState->sidePanel.pinned = document.sidePanelPinned;
    runtimeState->projectSettings.autoLowerGsplatQualityWhileNavigating =
        document.autoLowerGsplatQualityWhileNavigating;
    runtimeState->projectSettings.pointCloudPreviewLodMode = document.pointCloudPreviewLodMode;
    runtimeState->projectSettings.interactivePointCap = document.interactivePointCap;
    runtimeState->projectSettings.pointCloudRendererMode = document.pointCloudRendererMode;
    auto renderSettings = document.renderJobSettings;
    if (renderSettings.outputDirectory.empty() && !runtimeState->renderSettings.outputDirectory.empty()) {
        renderSettings.outputDirectory = runtimeState->renderSettings.outputDirectory;
    }
    runtimeState->renderSettings = renderSettings;
    runtimeState->water.emitters = document.waterEmitters;
    runtimeState->water.basinRegions = document.waterBasinRegions;
    runtimeState->water.runoffRegions = document.waterRunoffRegions;
    runtimeState->water.causticRegions = document.waterCausticRegions;
    runtimeState->water.defaultSourceSettings = document.waterSourceSettings;
    runtimeState->water.tempDefaultSourceSettings = document.tempWaterSourceSettings;
    runtimeState->water.defaultAnimationTrailSettings = document.waterAnimationTrailSettings;
    runtimeState->water.tempDefaultAnimationTrailSettings = document.tempWaterAnimationTrailSettings;
    runtimeState->water.animationTrailProfiles.clear();
    runtimeState->water.animationTrailProfiles.reserve(document.waterAnimationTrailProfiles.size());
    for (const auto& profile : document.waterAnimationTrailProfiles) {
        runtimeState->water.animationTrailProfiles.push_back({
            .name = NormalizeWaterAnimationTrailProfileName(profile.name),
            .settings = profile.settings,
        });
    }
    EnsureWaterAnimationTrailProfiles(&runtimeState->water);
    runtimeState->water.defaultCausticLookSettings = document.waterCausticLookSettings;
    runtimeState->water.tempDefaultCausticLookSettings = document.tempWaterCausticLookSettings;
    runtimeState->water.defaultPointVisualStyle = document.waterPointVisualStyle;
    runtimeState->water.tempDefaultPointVisualStyle = document.tempWaterPointVisualStyle;
    runtimeState->water.pointVisuals.clear();
    for (const auto& visualDocument : document.waterPointVisuals) {
        const auto normalized = NormalizePointVisualName(visualDocument.name);
        if (!IsWaterProjectCustomVisualName(normalized)) {
            continue;
        }
        AppendWaterPointVisualIfMissing(
            &runtimeState->water.pointVisuals,
            normalized,
            visualDocument.style);
    }
    bool importedLegacyWaterPointVisual = false;
    if (document.schemaVersion < 23U && document.waterPointVisuals.empty()) {
        if (document.tempWaterPointVisualStyle.has_value()) {
            ImportLegacyWaterPointVisualStyle(
                runtimeState,
                document.tempWaterPointVisualStyle.value());
        } else {
            ImportLegacyWaterPointVisualStyle(runtimeState, document.waterPointVisualStyle);
        }
        importedLegacyWaterPointVisual = true;
    }
    runtimeState->water.selectedPointVisualName =
        document.selectedWaterPointVisualName.empty()
            ? std::string{"Water Flow_preset"}
            : NormalizePointVisualName(document.selectedWaterPointVisualName);
    if (importedLegacyWaterPointVisual &&
        runtimeState->water.selectedPointVisualName == "Water Flow_preset" &&
        FindWaterProjectVisualIndex(runtimeState->water.pointVisuals, "Water Flow").has_value()) {
        runtimeState->water.selectedPointVisualName = "Water Flow";
    }
    if (document.schemaVersion < 23U) {
        const auto rawSelected = runtimeState->water.selectedPointVisualName;
        const auto legacySelected = NormalizeWaterPointVisualName(rawSelected);
        if (legacySelected != rawSelected &&
            !FindWaterProjectVisualIndex(runtimeState->water.pointVisuals, rawSelected).has_value()) {
            runtimeState->water.selectedPointVisualName = legacySelected;
        }
    }
    const bool selectedIsPreset =
        MakeProtectedWaterPointVisualStyle(*runtimeState, runtimeState->water.selectedPointVisualName).has_value();
    if (!selectedIsPreset &&
        !FindWaterProjectVisualIndex(
             runtimeState->water.pointVisuals,
             runtimeState->water.selectedPointVisualName)
             .has_value()) {
        runtimeState->water.selectedPointVisualName = "Water Flow_preset";
    }
    runtimeState->water.pointVisualNameBuffer = BasePointVisualName(runtimeState->water.selectedPointVisualName);
    runtimeState->water.nextEmitterId = NextWaterEmitterId(*runtimeState);
    runtimeState->water.nextBasinRegionId = NextWaterBasinRegionId(*runtimeState);
    runtimeState->water.nextRunoffRegionId = NextWaterRunoffRegionId(*runtimeState);
    runtimeState->water.nextCausticRegionId = NextWaterCausticRegionId(*runtimeState);
    runtimeState->water.selectedEmitterIndex.reset();
    runtimeState->water.selectedBasinRegionIndex.reset();
    runtimeState->water.selectedRunoffRegionIndex.reset();
    runtimeState->water.selectedCausticRegionIndex.reset();
    runtimeState->water.placementArmed = false;
    runtimeState->water.basinRegionPlacementArmed = false;
    runtimeState->water.runoffRegionPlacementArmed = false;
    runtimeState->water.causticRegionPlacementArmed = false;
    runtimeState->water.movingEmitterIndex.reset();
    SyncWaterAnimationTrailProfileFromCurrentAnimation(runtimeState);
    if (document.waterPathCache.has_value() && !document.waterPathCache->branches.empty()) {
        runtimeState->water.pathCache = document.waterPathCache.value();
        runtimeState->water.pathCacheLoaded = true;
        runtimeState->water.pathCacheStale = runtimeState->water.pathCache.stale;
        runtimeState->water.pathDirty = runtimeState->water.pathCacheStale;
        runtimeState->water.dirtyEmitterIds.clear();
        runtimeState->water.pathAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
            runtimeState->water.pathCache,
            runtimeState->water.emitters,
            ActiveDefaultWaterSourceSettings(*runtimeState));
    } else {
        runtimeState->water.pathCache = {};
        runtimeState->water.pathCacheLoaded = false;
        runtimeState->water.pathCacheStale = false;
        MarkWaterPathDirty(runtimeState);
    }
    ValidateWaterSourceSettingLinks(runtimeState);
    runtimeState->cameraShots = document.cameraShots;
    EnsureRuntimeCameraShotIds(runtimeState);
    runtimeState->cameraPanel.draftShotName = NextCameraShotName(*runtimeState);
    runtimeState->cameraPanel.selectedShotIndex.reset();
    runtimeState->cameraPanel.renamingShotIndex.reset();
    runtimeState->cameraPanel.pendingLinkedShotDeleteIndex.reset();
    runtimeState->cameraPanel.multiEditAllowedCameraIds.clear();
    runtimeState->cameraPanel.shotRenameBuffer.clear();
    runtimeState->cameraPanel.focusShotRename = false;
    runtimeState->cameraPanel.blendFromIndex.reset();
    runtimeState->cameraPanel.blendToIndex.reset();
    runtimeState->cameraPanel.pathShotIndices = document.cameraPathShotIndices;
    runtimeState->cameraPanel.selectedPathItemIndex.reset();
    runtimeState->cameraPanel.pathDurationFrames =
        std::max<std::uint32_t>(1U, document.cameraPathDurationFrames);
    runtimeState->animationPanel.currentFilePath = document.lastAnimationPath.string();
    runtimeState->animationPanel.availableFiles.clear();
    runtimeState->animationPanel.availableFileAssociatedLayerPaths.clear();
    runtimeState->animationPanel.availableFileLoadedPaths.clear();
    runtimeState->animationPanel.availableFileDirtyFlags.clear();
    runtimeState->animationPanel.selectedExportFiles.clear();
    if (document.hasSavedAnimationRegistry) {
        for (const auto& animation : document.savedAnimations) {
            AddAnimationFileToRegistry(
                &runtimeState->animationPanel,
                animation.filePath,
                animation.associatedLayerPaths);
        }
        runtimeState->animationPanel.animationRegistryInitialized = true;
    } else {
        runtimeState->animationPanel.animationRegistryInitialized = false;
        RefreshAnimationFileList(&runtimeState->animationPanel, AnimationDirectory(*runtimeState));
    }
    runtimeState->animationPanel.renamingFileIndex.reset();
    runtimeState->animationPanel.fileRenameBuffer.clear();
    runtimeState->animationPanel.focusFileRename = false;
    if (runtimeState->cameraPanel.pathShotIndices.empty() &&
        document.schemaVersion < 9U &&
        !runtimeState->cameraShots.empty()) {
        runtimeState->cameraPanel.pathShotIndices.reserve(runtimeState->cameraShots.size());
        for (std::size_t index = 0; index < runtimeState->cameraShots.size(); ++index) {
            runtimeState->cameraPanel.pathShotIndices.push_back(index);
        }
        std::uint32_t legacyDurationFrames = 0U;
        for (std::size_t index = 1; index < runtimeState->cameraShots.size(); ++index) {
            legacyDurationFrames += std::max<std::uint32_t>(1U, runtimeState->cameraShots[index].durationFrames);
        }
        runtimeState->cameraPanel.pathDurationFrames = std::max<std::uint32_t>(1U, legacyDurationFrames);
    }
    runtimeState->cameraPlayback.active = false;
    if (!runtimeState->cameraShots.empty()) {
        runtimeState->cameraPanel.selectedShotIndex = 0;
        runtimeState->cameraPanel.blendFromIndex = 0;
        runtimeState->cameraPanel.blendToIndex = runtimeState->cameraShots.size() > 1 ? 1U : 0U;
    }
    EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());
    runtimeState->persistence.queuedLoadIndices.clear();
    const bool hasProjectCamera = document.cameraState.has_value();
    bool requestedLoadedLayer = false;

    for (std::size_t sessionIndex = 0; sessionIndex < runtimeState->sessions.size(); ++sessionIndex) {
        auto& session = runtimeState->sessions[sessionIndex];
        const auto layerIt = std::find_if(
            document.layers.begin(),
            document.layers.end(),
            [&session](const ProjectLayerDocument& layerDocument) {
                return NormalizePathKey(layerDocument.sourcePath) == NormalizePathKey(session.sourcePath);
            });

        if (layerIt == document.layers.end()) {
            UnloadLayerByIndex(runtimeState, viewport, sessionIndex);
            continue;
        }

        if (session.kind == LayerKind::PointCloud) {
            session.pointVisuals.clear();
            for (const auto& visualDocument : layerIt->pointVisuals) {
                session.pointVisuals.push_back({
                    .name = NormalizePointVisualName(visualDocument.name),
                    .style = visualDocument.style,
                });
            }
            session.selectedPointVisualName = NormalizePointVisualName(layerIt->selectedPointVisualName);

            if (!session.pointVisuals.empty()) {
                EnsurePointVisuals(&session);
                if (const auto visualIndex = FindPointVisualIndex(session, session.selectedPointVisualName);
                    visualIndex.has_value()) {
                    session.pointStyle = session.pointVisuals[visualIndex.value()].style;
                }
                SyncPointVisualNameBuffer(&session);
            } else if (layerIt->pointStyle.has_value()) {
                session.pointStyle = layerIt->pointStyle.value();
                session.pointVisuals.push_back(
                    {.name = std::string{kDefaultPointVisualName}, .style = session.pointStyle});
                session.selectedPointVisualName = std::string{kDefaultPointVisualName};
                SyncPointVisualNameBuffer(&session);
            } else {
                EnsurePointVisuals(&session);
            }
            if (IsGeneratedWaterFlowOverlaySession(session)) {
                SeedWaterFlowBuiltInVisuals(runtimeState, &session);
            }
        }

        if (session.kind == LayerKind::PointCloud && layerIt->pointBudgetActivePoints > 0) {
            session.pointBudget = MakePreviewPointBudgetState(session, layerIt->pointBudgetActivePoints);
            ClearPreviewLodSampleCache(&session);
            if (session.loaded) {
                viewport->UpdatePointBudget(sessionIndex, session.pointBudget.sampledIndices);
            }
        }

        SanitizePointCloudStyle(&session);
        if (session.kind == LayerKind::PointCloud) {
            EnsurePointVisuals(&session);
            UpsertPointVisual(&session, session.selectedPointVisualName, session.pointStyle);
        }

        requestedLoadedLayer |= layerIt->loaded;
        if (!layerIt->loaded) {
            UnloadLayerByIndex(runtimeState, viewport, sessionIndex);
            continue;
        }

        session.visible = layerIt->visible;
        if (!session.loaded) {
            runtimeState->persistence.queuedLoadIndices.push_back(sessionIndex);
        }
    }

    runtimeState->selectedSessionIndex.reset();
    if (!document.selectedLayerPath.empty()) {
        runtimeState->selectedSessionIndex = FindSessionIndexBySourcePath(*runtimeState, document.selectedLayerPath);
    }

    if (runtimeState->selectedSessionIndex.has_value()) {
        const auto selectedIndex = runtimeState->selectedSessionIndex.value();
        if (runtimeState->sessions[selectedIndex].loaded && !hasProjectCamera) {
            FocusSessionLayer(runtimeState, *viewport, selectedIndex);
        }
    }

    if (hasProjectCamera) {
        runtimeState->camera.ApplyState(document.cameraState.value());
        SyncPivotOverlayToCamera(runtimeState);
    }
    for (const auto& session : runtimeState->sessions) {
        if (IsVisibleLoadedWaterSupportSession(session) &&
            TryLoadWaterPathCacheForSupport(runtimeState, session)) {
            RefreshWaterOverlayFromAnchors(
                runtimeState,
                viewport,
                WaterOverlayRefreshPersistence::InMemoryOnly);
            break;
        }
    }
    runtimeState->preserveProjectCameraOnNextLayerActivation =
        hasProjectCamera && requestedLoadedLayer && VisibleLayerCount(*runtimeState) == 0;

    runtimeState->statusMessage =
        "Loaded project with " + FormatPointCount(document.layers.size()) + " layer settings.";
    runtimeState->errorMessage.clear();
    return true;
}

void UnloadSelectedLayer(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr || !runtimeState->selectedSessionIndex.has_value()) {
        return;
    }

    auto& session = runtimeState->sessions[runtimeState->selectedSessionIndex.value()];
    if (!session.loaded) {
        return;
    }

    UnloadLayerByIndex(runtimeState, viewport, runtimeState->selectedSessionIndex.value());
    runtimeState->statusMessage = "Unloaded " + session.displayName + ".";
    runtimeState->errorMessage.clear();
}

void EnsureCameraShotSelections(CameraPanelState* panelState, std::size_t shotCount) {
    if (panelState == nullptr) {
        return;
    }

    const auto validIndex = [shotCount](const std::optional<std::size_t>& index) {
        return index.has_value() && index.value() < shotCount;
    };

    if (!validIndex(panelState->selectedShotIndex)) {
        panelState->selectedShotIndex = shotCount > 0 ? std::optional<std::size_t>{0} : std::nullopt;
    }
    if (!validIndex(panelState->renamingShotIndex)) {
        panelState->renamingShotIndex.reset();
        panelState->shotRenameBuffer.clear();
        panelState->focusShotRename = false;
    }
    if (!validIndex(panelState->blendFromIndex)) {
        panelState->blendFromIndex = shotCount > 0 ? std::optional<std::size_t>{0} : std::nullopt;
    }
    if (!validIndex(panelState->blendToIndex)) {
        panelState->blendToIndex =
            shotCount > 1 ? std::optional<std::size_t>{1} : panelState->blendFromIndex;
    }

    panelState->pathShotIndices.erase(
        std::remove_if(
            panelState->pathShotIndices.begin(),
            panelState->pathShotIndices.end(),
            [shotCount](std::size_t index) {
                return index >= shotCount;
            }),
        panelState->pathShotIndices.end());

    if (panelState->selectedPathItemIndex.has_value() &&
        panelState->selectedPathItemIndex.value() >= panelState->pathShotIndices.size()) {
        panelState->selectedPathItemIndex = panelState->pathShotIndices.empty()
                                                ? std::nullopt
                                                : std::optional<std::size_t>{panelState->pathShotIndices.size() - 1U};
    }

    const auto minimumDurationFrames = panelState->pathShotIndices.size() > 1U
                                           ? static_cast<std::uint32_t>(panelState->pathShotIndices.size() - 1U)
                                           : 1U;
    panelState->pathDurationFrames = std::max(panelState->pathDurationFrames, minimumDurationFrames);
}

void RemoveCameraShotFromPath(CameraPanelState* panelState, std::size_t deletedShotIndex) {
    if (panelState == nullptr) {
        return;
    }

    std::vector<std::size_t> adjustedPath;
    adjustedPath.reserve(panelState->pathShotIndices.size());
    for (const auto shotIndex : panelState->pathShotIndices) {
        if (shotIndex == deletedShotIndex) {
            continue;
        }
        adjustedPath.push_back(shotIndex > deletedShotIndex ? shotIndex - 1U : shotIndex);
    }
    panelState->pathShotIndices = std::move(adjustedPath);
}

void MoveCameraPathItem(std::vector<std::size_t>* pathShotIndices, std::size_t fromIndex, std::size_t toIndex) {
    if (pathShotIndices == nullptr ||
        fromIndex >= pathShotIndices->size() ||
        toIndex >= pathShotIndices->size() ||
        fromIndex == toIndex) {
        return;
    }

    const auto shotIndex = (*pathShotIndices)[fromIndex];
    pathShotIndices->erase(pathShotIndices->begin() + static_cast<std::ptrdiff_t>(fromIndex));
    const auto insertIndex = std::min(toIndex, pathShotIndices->size());
    pathShotIndices->insert(pathShotIndices->begin() + static_cast<std::ptrdiff_t>(insertIndex), shotIndex);
}

std::vector<CameraShot> BuildOrderedCameraPathShots(
    const std::vector<CameraShot>& savedShots,
    const std::vector<std::size_t>& pathShotIndices,
    std::uint32_t totalDurationFrames) {
    std::vector<CameraShot> orderedShots;
    orderedShots.reserve(pathShotIndices.size());
    for (const auto shotIndex : pathShotIndices) {
        if (shotIndex < savedShots.size()) {
            orderedShots.push_back(savedShots[shotIndex]);
        }
    }
    return invisible_places::camera::BuildWeightedCameraPathShots(orderedShots, totalDurationFrames);
}

std::vector<CameraShot> BuildPanelCameraPathShots(const PreviewRuntimeState& runtimeState) {
    return BuildOrderedCameraPathShots(
        runtimeState.cameraShots,
        runtimeState.cameraPanel.pathShotIndices,
        runtimeState.cameraPanel.pathDurationFrames);
}

std::filesystem::path AnimationDirectory(const PreviewRuntimeState& runtimeState) {
    return runtimeState.persistence.animationDirectoryPath.empty()
               ? std::filesystem::path{"Saved/animations"}
               : std::filesystem::path{runtimeState.persistence.animationDirectoryPath};
}

std::filesystem::path HoudiniCameraExportDirectory(const PreviewRuntimeState& runtimeState) {
    const auto animationDirectory = AnimationDirectory(runtimeState).lexically_normal();
    const auto savedDirectory = animationDirectory.parent_path();
    const auto projectRoot = savedDirectory.parent_path();
    if (!projectRoot.empty()) {
        return projectRoot.parent_path() / "Invisible Places Houdini" / "camera_exports";
    }
    return std::filesystem::path{"../Invisible Places Houdini/camera_exports"};
}

std::string SanitizeAnimationFileStem(std::string name) {
    if (name.empty()) {
        name = "Animation";
    }

    std::string stem;
    stem.reserve(name.size());
    bool previousWasSeparator = false;
    for (const auto character : name) {
        const auto unsignedCharacter = static_cast<unsigned char>(character);
        if (std::isalnum(unsignedCharacter)) {
            stem.push_back(character);
            previousWasSeparator = false;
        } else if (!previousWasSeparator) {
            stem.push_back('_');
            previousWasSeparator = true;
        }
    }

    while (!stem.empty() && stem.back() == '_') {
        stem.pop_back();
    }
    return stem.empty() ? std::string{"Animation"} : stem;
}

std::string AnimationDisplayNameFromPath(const std::filesystem::path& path) {
    auto filename = path.filename().string();
    if (EndsWith(filename, ".ipanim.json")) {
        filename.resize(filename.size() - std::string_view{".ipanim.json"}.size());
        return filename;
    }
    return path.stem().string();
}

std::string NormalizeAnimationNameFromInput(std::string name) {
    name = TrimText(name);
    if (EndsWith(name, ".ipanim.json")) {
        name.resize(name.size() - std::string_view{".ipanim.json"}.size());
    } else if (EndsWith(name, ".ipanim")) {
        name.resize(name.size() - std::string_view{".ipanim"}.size());
    } else if (EndsWith(name, ".json")) {
        name.resize(name.size() - std::string_view{".json"}.size());
    }
    name = TrimText(name);
    return name.empty() ? std::string{"Animation"} : name;
}

std::string AnimationNameFromFilePath(const std::filesystem::path& path) {
    return NormalizeAnimationNameFromInput(AnimationDisplayNameFromPath(path));
}

std::filesystem::path AnimationFilePathForName(
    const PreviewRuntimeState& runtimeState,
    const std::string& animationName) {
    return AnimationDirectory(runtimeState) / (SanitizeAnimationFileStem(animationName) + ".ipanim.json");
}

std::filesystem::path UniqueAnimationFilePathForName(
    const PreviewRuntimeState& runtimeState,
    const std::string& animationName) {
    const auto directory = AnimationDirectory(runtimeState);
    const auto stem = SanitizeAnimationFileStem(animationName);
    auto candidate = directory / (stem + ".ipanim.json");
    std::error_code existsError;
    if (!std::filesystem::exists(candidate, existsError)) {
        return candidate;
    }

    for (std::uint32_t suffix = 2U; suffix < 10000U; ++suffix) {
        candidate = directory / (stem + "_" + std::to_string(suffix) + ".ipanim.json");
        existsError.clear();
        if (!std::filesystem::exists(candidate, existsError)) {
            return candidate;
        }
    }
    return directory / (stem + "_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
                        ".ipanim.json");
}

std::filesystem::path HoudiniCameraScriptPathForAnimation(
    const PreviewRuntimeState& runtimeState,
    const std::string& animationName) {
    return HoudiniCameraExportDirectory(runtimeState) /
           (SanitizeAnimationFileStem(animationName) + "_houdini_camera.py");
}

std::filesystem::path HoudiniCameraImportScriptPath(const PreviewRuntimeState& runtimeState) {
    return HoudiniCameraExportDirectory(runtimeState) / "import_houdini_camera.py";
}

RenderJobSettings RenderSettingsFromAnimationExportSettings(const AnimationExportSettings& exportSettings) {
    RenderJobSettings settings;
    settings.outputDirectory = exportSettings.outputDirectory;
    settings.width = std::max<std::uint32_t>(1U, exportSettings.width);
    settings.height = std::max<std::uint32_t>(1U, exportSettings.height);
    settings.framesPerSecond = std::max<std::uint32_t>(1U, exportSettings.framesPerSecond);
    settings.stillCameraDurationSeconds = std::clamp(exportSettings.stillCameraDurationSeconds, 0.001F, 3600.0F);
    settings.startFrame = exportSettings.startFrame;
    settings.endFrame = exportSettings.endFrame;
    return settings;
}

AnimationExportSettings AnimationExportSettingsFromRenderSettings(const RenderJobSettings& settings) {
    return AnimationExportSettings{
        .outputDirectory = settings.outputDirectory,
        .width = std::max<std::uint32_t>(1U, settings.width),
        .height = std::max<std::uint32_t>(1U, settings.height),
        .framesPerSecond = std::max<std::uint32_t>(1U, settings.framesPerSecond),
        .stillCameraDurationSeconds = std::clamp(settings.stillCameraDurationSeconds, 0.001F, 3600.0F),
        .startFrame = settings.startFrame,
        .endFrame = settings.endFrame,
    };
}

void MarkCurrentAnimationExportSettingsDirty(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->animationPanel.currentPath.has_value()) {
        return;
    }

    runtimeState->animationPanel.currentPath->exportSettings =
        AnimationExportSettingsFromRenderSettings(runtimeState->renderSettings);
    runtimeState->animationPanel.dirty = true;
}

bool PathsLexicallyEqual(const std::filesystem::path& left, const std::filesystem::path& right) {
    return left.lexically_normal() == right.lexically_normal();
}

void EnsureAnimationAssociationStorage(AnimationPanelState* panelState) {
    if (panelState == nullptr) {
        return;
    }
    panelState->availableFileAssociatedLayerPaths.resize(panelState->availableFiles.size());
    panelState->availableFileLoadedPaths.resize(panelState->availableFiles.size());
    panelState->availableFileDirtyFlags.resize(panelState->availableFiles.size(), false);
}

std::optional<std::size_t> FindAnimationRegistryIndex(
    const AnimationPanelState& panelState,
    const std::filesystem::path& path) {
    const auto targetKey = NormalizePathKey(path);
    for (std::size_t index = 0; index < panelState.availableFiles.size(); ++index) {
        if (NormalizePathKey(panelState.availableFiles[index]) == targetKey) {
            return index;
        }
    }
    return std::nullopt;
}

std::vector<std::filesystem::path>* AnimationRegistryAssociationPaths(
    AnimationPanelState* panelState,
    std::size_t fileIndex) {
    if (panelState == nullptr || fileIndex >= panelState->availableFiles.size()) {
        return nullptr;
    }
    EnsureAnimationAssociationStorage(panelState);
    return &panelState->availableFileAssociatedLayerPaths[fileIndex];
}

const std::vector<std::filesystem::path>& AnimationRegistryAssociationPaths(
    const AnimationPanelState& panelState,
    std::size_t fileIndex) {
    static const std::vector<std::filesystem::path> kEmpty;
    if (fileIndex >= panelState.availableFileAssociatedLayerPaths.size()) {
        return kEmpty;
    }
    return panelState.availableFileAssociatedLayerPaths[fileIndex];
}

bool SetAnimationRegistryAssociations(
    AnimationPanelState* panelState,
    const std::filesystem::path& path,
    std::vector<std::filesystem::path> associatedLayerPaths) {
    if (panelState == nullptr) {
        return false;
    }
    const auto fileIndex = FindAnimationRegistryIndex(*panelState, path);
    if (!fileIndex.has_value()) {
        return false;
    }
    NormalizeAssociatedLayerPaths(&associatedLayerPaths);
    EnsureAnimationAssociationStorage(panelState);
    panelState->availableFileAssociatedLayerPaths[fileIndex.value()] = std::move(associatedLayerPaths);
    return true;
}

void SortAnimationRegistry(AnimationPanelState* panelState) {
    if (panelState == nullptr) {
        return;
    }
    EnsureAnimationAssociationStorage(panelState);
    std::vector<std::size_t> indices(panelState->availableFiles.size());
    std::iota(indices.begin(), indices.end(), static_cast<std::size_t>(0U));
    std::sort(
        indices.begin(),
        indices.end(),
        [&](std::size_t left, std::size_t right) {
            return NormalizePathKey(panelState->availableFiles[left]) <
                   NormalizePathKey(panelState->availableFiles[right]);
        });

    std::vector<std::filesystem::path> sortedFiles;
    std::vector<std::vector<std::filesystem::path>> sortedAssociations;
    std::vector<std::optional<AnimationPath>> sortedLoadedPaths;
    std::vector<bool> sortedDirtyFlags;
    sortedFiles.reserve(panelState->availableFiles.size());
    sortedAssociations.reserve(panelState->availableFiles.size());
    sortedLoadedPaths.reserve(panelState->availableFiles.size());
    sortedDirtyFlags.reserve(panelState->availableFiles.size());
    for (const auto index : indices) {
        if (!sortedFiles.empty() &&
            NormalizePathKey(sortedFiles.back()) == NormalizePathKey(panelState->availableFiles[index])) {
            continue;
        }
        auto associations = panelState->availableFileAssociatedLayerPaths[index];
        NormalizeAssociatedLayerPaths(&associations);
        sortedFiles.push_back(panelState->availableFiles[index].lexically_normal());
        sortedAssociations.push_back(std::move(associations));
        sortedLoadedPaths.push_back(panelState->availableFileLoadedPaths[index]);
        sortedDirtyFlags.push_back(panelState->availableFileDirtyFlags[index]);
    }
    panelState->availableFiles = std::move(sortedFiles);
    panelState->availableFileAssociatedLayerPaths = std::move(sortedAssociations);
    panelState->availableFileLoadedPaths = std::move(sortedLoadedPaths);
    panelState->availableFileDirtyFlags = std::move(sortedDirtyFlags);
}

bool AddAnimationFileToRegistry(
    AnimationPanelState* panelState,
    const std::filesystem::path& filePath,
    std::vector<std::filesystem::path> associatedLayerPaths) {
    if (panelState == nullptr || filePath.empty()) {
        return false;
    }
    if (FindAnimationRegistryIndex(*panelState, filePath).has_value()) {
        return false;
    }
    NormalizeAssociatedLayerPaths(&associatedLayerPaths);
    std::optional<AnimationPath> loadedPath;
    std::string loadError;
    auto maybeLoadedPath = invisible_places::serialization::LoadAnimationPath(filePath, &loadError);
    if (maybeLoadedPath.has_value()) {
        loadedPath = std::move(maybeLoadedPath.value());
        loadedPath->associatedLayerPaths = associatedLayerPaths;
    }
    panelState->availableFiles.push_back(filePath.lexically_normal());
    panelState->availableFileAssociatedLayerPaths.push_back(std::move(associatedLayerPaths));
    panelState->availableFileLoadedPaths.push_back(std::move(loadedPath));
    panelState->availableFileDirtyFlags.push_back(false);
    SortAnimationRegistry(panelState);
    return true;
}

std::vector<std::filesystem::path> LoadAnimationFileAssociations(
    const std::filesystem::path& filePath) {
    std::string errorMessage;
    auto animationPath = invisible_places::serialization::LoadAnimationPath(filePath, &errorMessage);
    if (!animationPath.has_value()) {
        return {};
    }
    auto associations = animationPath->associatedLayerPaths;
    NormalizeAssociatedLayerPaths(&associations);
    return associations;
}

std::size_t ImportAnimationFilesFromDirectory(
    AnimationPanelState* panelState,
    const std::filesystem::path& animationDirectory) {
    if (panelState == nullptr) {
        return 0U;
    }

    std::error_code createError;
    std::filesystem::create_directories(animationDirectory, createError);
    if (createError) {
        return 0U;
    }

    std::vector<std::filesystem::path> files;
    std::error_code iterateError;
    for (const auto& entry : std::filesystem::directory_iterator{animationDirectory, iterateError}) {
        if (iterateError) {
            break;
        }
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto path = entry.path();
        if (path.filename().string().ends_with(".ipanim.json")) {
            files.push_back(path);
        }
    }
    std::sort(files.begin(), files.end());

    std::size_t importedCount = 0;
    for (const auto& filePath : files) {
        if (AddAnimationFileToRegistry(
                panelState,
                filePath,
                LoadAnimationFileAssociations(filePath))) {
            ++importedCount;
        }
    }
    return importedCount;
}

void RemoveAnimationFileFromRegistry(AnimationPanelState* panelState, std::size_t fileIndex) {
    if (panelState == nullptr || fileIndex >= panelState->availableFiles.size()) {
        return;
    }

    const auto removedPath = panelState->availableFiles[fileIndex];
    panelState->availableFiles.erase(panelState->availableFiles.begin() + static_cast<std::ptrdiff_t>(fileIndex));
    if (fileIndex < panelState->availableFileAssociatedLayerPaths.size()) {
        panelState->availableFileAssociatedLayerPaths.erase(
            panelState->availableFileAssociatedLayerPaths.begin() + static_cast<std::ptrdiff_t>(fileIndex));
    }
    if (fileIndex < panelState->availableFileLoadedPaths.size()) {
        panelState->availableFileLoadedPaths.erase(
            panelState->availableFileLoadedPaths.begin() + static_cast<std::ptrdiff_t>(fileIndex));
    }
    if (fileIndex < panelState->availableFileDirtyFlags.size()) {
        panelState->availableFileDirtyFlags.erase(
            panelState->availableFileDirtyFlags.begin() + static_cast<std::ptrdiff_t>(fileIndex));
    }
    panelState->selectedExportFiles.erase(
        std::remove_if(
            panelState->selectedExportFiles.begin(),
            panelState->selectedExportFiles.end(),
            [&removedPath](const std::filesystem::path& selectedPath) {
                return PathsLexicallyEqual(selectedPath, removedPath);
            }),
        panelState->selectedExportFiles.end());
    if (panelState->selectedFileIndex.has_value()) {
        if (panelState->availableFiles.empty()) {
            panelState->selectedFileIndex.reset();
        } else if (panelState->selectedFileIndex.value() >= panelState->availableFiles.size()) {
            panelState->selectedFileIndex = panelState->availableFiles.size() - 1U;
        }
    }
}

bool AnimationFileSelectedForExport(
    const AnimationPanelState& panelState,
    const std::filesystem::path& path) {
    return std::any_of(
        panelState.selectedExportFiles.begin(),
        panelState.selectedExportFiles.end(),
        [&path](const std::filesystem::path& selectedPath) {
            return PathsLexicallyEqual(selectedPath, path);
        });
}

void SetAnimationFileSelectedForExport(
    AnimationPanelState* panelState,
    const std::filesystem::path& path,
    bool selected) {
    if (panelState == nullptr) {
        return;
    }

    const auto existing = std::find_if(
        panelState->selectedExportFiles.begin(),
        panelState->selectedExportFiles.end(),
        [&path](const std::filesystem::path& selectedPath) {
            return PathsLexicallyEqual(selectedPath, path);
        });
    if (selected) {
        if (existing == panelState->selectedExportFiles.end()) {
            panelState->selectedExportFiles.push_back(path);
        }
        return;
    }

    if (existing != panelState->selectedExportFiles.end()) {
        panelState->selectedExportFiles.erase(existing);
    }
}

void RefreshAnimationFileList(AnimationPanelState* panelState, const std::filesystem::path& animationDirectory) {
    if (panelState == nullptr) {
        return;
    }

    std::error_code createError;
    std::filesystem::create_directories(animationDirectory, createError);
    if (createError) {
        return;
    }

    EnsureAnimationAssociationStorage(panelState);
    if (!panelState->animationRegistryInitialized) {
        panelState->availableFiles.clear();
        panelState->availableFileAssociatedLayerPaths.clear();
        ImportAnimationFilesFromDirectory(panelState, animationDirectory);
        panelState->animationRegistryInitialized = true;
    }
    SortAnimationRegistry(panelState);
    panelState->selectedExportFiles.erase(
        std::remove_if(
            panelState->selectedExportFiles.begin(),
            panelState->selectedExportFiles.end(),
            [&](const std::filesystem::path& selectedPath) {
                return std::none_of(
                    panelState->availableFiles.begin(),
                    panelState->availableFiles.end(),
                    [&selectedPath](const std::filesystem::path& availablePath) {
                        return PathsLexicallyEqual(selectedPath, availablePath);
                    });
            }),
        panelState->selectedExportFiles.end());
    if (panelState->selectedFileIndex.has_value() &&
        panelState->selectedFileIndex.value() >= panelState->availableFiles.size()) {
        panelState->selectedFileIndex = panelState->availableFiles.empty()
                                            ? std::nullopt
                                            : std::optional<std::size_t>{panelState->availableFiles.size() - 1U};
    }
    if (panelState->renamingFileIndex.has_value() &&
        panelState->renamingFileIndex.value() >= panelState->availableFiles.size()) {
        panelState->renamingFileIndex.reset();
        panelState->fileRenameBuffer.clear();
        panelState->focusFileRename = false;
    }
}

std::string FormatThreeDigitOrdinal(std::size_t ordinal) {
    std::ostringstream output;
    output << std::setw(3) << std::setfill('0') << ordinal;
    return output.str();
}

std::string LinkedCameraShotName(const std::string& animationName, std::size_t keyIndex) {
    return "c_" + SanitizeAnimationFileStem(animationName) + "_" + FormatThreeDigitOrdinal(keyIndex + 1U);
}

std::string MakeUniqueCameraShotId(const PreviewRuntimeState& runtimeState) {
    std::unordered_set<std::string> usedIds;
    for (const auto& shot : runtimeState.cameraShots) {
        if (!shot.id.empty()) {
            usedIds.insert(shot.id);
        }
    }

    for (std::size_t index = runtimeState.cameraShots.size() + 1U;; ++index) {
        auto candidate = "camera_" + std::to_string(index);
        if (!usedIds.contains(candidate)) {
            return candidate;
        }
    }
}

void EnsureRuntimeCameraShotIds(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    std::unordered_set<std::string> usedIds;
    for (const auto& shot : runtimeState->cameraShots) {
        if (!shot.id.empty()) {
            usedIds.insert(shot.id);
        }
    }

    for (std::size_t index = 0; index < runtimeState->cameraShots.size(); ++index) {
        auto& shot = runtimeState->cameraShots[index];
        if (!shot.id.empty()) {
            continue;
        }
        auto candidate = "camera_" + std::to_string(index + 1U);
        std::size_t suffix = index + 1U;
        while (usedIds.contains(candidate)) {
            candidate = "camera_" + std::to_string(++suffix);
        }
        shot.id = candidate;
        usedIds.insert(candidate);
    }
}

std::optional<std::size_t> FindCameraShotIndexById(
    const PreviewRuntimeState& runtimeState,
    const std::string& cameraId) {
    if (cameraId.empty()) {
        return std::nullopt;
    }

    for (std::size_t index = 0; index < runtimeState.cameraShots.size(); ++index) {
        if (runtimeState.cameraShots[index].id == cameraId) {
            return index;
        }
    }
    return std::nullopt;
}

bool IsCurrentAnimationRegistryIndex(const AnimationPanelState& panel, std::size_t fileIndex) {
    return fileIndex < panel.availableFiles.size() &&
           !panel.currentFilePath.empty() &&
           PathsLexicallyEqual(panel.availableFiles[fileIndex], std::filesystem::path{panel.currentFilePath});
}

AnimationPath* MutableRegistryAnimationPath(PreviewRuntimeState* runtimeState, std::size_t fileIndex) {
    if (runtimeState == nullptr || fileIndex >= runtimeState->animationPanel.availableFiles.size()) {
        return nullptr;
    }
    auto& panel = runtimeState->animationPanel;
    if (IsCurrentAnimationRegistryIndex(panel, fileIndex) && panel.currentPath.has_value()) {
        return &panel.currentPath.value();
    }
    if (fileIndex >= panel.availableFileLoadedPaths.size() ||
        !panel.availableFileLoadedPaths[fileIndex].has_value()) {
        return nullptr;
    }
    return &panel.availableFileLoadedPaths[fileIndex].value();
}

const AnimationPath* RegistryAnimationPath(const PreviewRuntimeState& runtimeState, std::size_t fileIndex) {
    if (fileIndex >= runtimeState.animationPanel.availableFiles.size()) {
        return nullptr;
    }
    const auto& panel = runtimeState.animationPanel;
    if (IsCurrentAnimationRegistryIndex(panel, fileIndex) && panel.currentPath.has_value()) {
        return &panel.currentPath.value();
    }
    if (fileIndex >= panel.availableFileLoadedPaths.size() ||
        !panel.availableFileLoadedPaths[fileIndex].has_value()) {
        return nullptr;
    }
    return &panel.availableFileLoadedPaths[fileIndex].value();
}

void MarkRegistryAnimationDirty(PreviewRuntimeState* runtimeState, std::size_t fileIndex) {
    if (runtimeState == nullptr || fileIndex >= runtimeState->animationPanel.availableFiles.size()) {
        return;
    }
    auto& panel = runtimeState->animationPanel;
    if (IsCurrentAnimationRegistryIndex(panel, fileIndex)) {
        panel.dirty = true;
    }
    EnsureAnimationAssociationStorage(&panel);
    if (fileIndex < panel.availableFileDirtyFlags.size()) {
        panel.availableFileDirtyFlags[fileIndex] = true;
    }
}

void SyncCurrentAnimationToRegistry(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr ||
        runtimeState->animationPanel.currentFilePath.empty() ||
        !runtimeState->animationPanel.currentPath.has_value()) {
        return;
    }
    auto& panel = runtimeState->animationPanel;
    const auto registryIndex = FindAnimationRegistryIndex(panel, std::filesystem::path{panel.currentFilePath});
    if (!registryIndex.has_value()) {
        return;
    }
    EnsureAnimationAssociationStorage(&panel);
    panel.availableFileLoadedPaths[registryIndex.value()] = panel.currentPath.value();
    panel.availableFileAssociatedLayerPaths[registryIndex.value()] = panel.currentPath->associatedLayerPaths;
    panel.availableFileDirtyFlags[registryIndex.value()] = panel.dirty;
}

std::array<float, 3> FocusPointFromCameraState(const invisible_places::camera::CameraState& state) {
    return state.target;
}

void CopyCameraShotToAnimationKeySnapshot(const CameraShot& shot, invisible_places::camera::AnimationPathKey* key) {
    if (key == nullptr) {
        return;
    }
    key->cameraPosition = shot.state.position;
    key->focusPoint = FocusPointFromCameraState(shot.state);
    key->fovDegrees = shot.state.fovDegrees;
    key->nearPlane = shot.state.nearPlane;
    key->farPlane = shot.state.farPlane;
    key->sourceShotName = shot.name;
    key->linkedCameraId = shot.id;
    key->linkedCameraName = shot.name;
}

void CopyAnimationKeyToCameraShot(
    const invisible_places::camera::AnimationPathKey& key,
    CameraShot* shot) {
    if (shot == nullptr) {
        return;
    }

    const auto previousDepthOfFieldEnabled = shot->state.hasDepthOfField;
    const auto previousApertureFStops = shot->state.apertureFStops;
    const auto previousMaxBlurPixels = shot->state.depthOfFieldMaxBlurPixels;
    AnimationPath singleKeyPath;
    singleKeyPath.keys.push_back(key);
    auto evaluated = invisible_places::camera::EvaluateAnimationPath(singleKeyPath, 0.0F).camera;
    evaluated.hasDepthOfField = previousDepthOfFieldEnabled;
    evaluated.apertureFStops = previousApertureFStops;
    evaluated.depthOfFieldMaxBlurPixels = previousMaxBlurPixels;
    shot->state = evaluated;
}

struct AnimationCameraLink {
    std::size_t fileIndex = 0;
    std::size_t keyIndex = 0;
    std::filesystem::path filePath;
    std::string animationName;
    std::string keyId;
};

std::vector<AnimationCameraLink> FindCameraAnimationLinks(
    const PreviewRuntimeState& runtimeState,
    const std::string& cameraId) {
    std::vector<AnimationCameraLink> links;
    if (cameraId.empty()) {
        return links;
    }

    for (std::size_t fileIndex = 0; fileIndex < runtimeState.animationPanel.availableFiles.size(); ++fileIndex) {
        const auto* path = RegistryAnimationPath(runtimeState, fileIndex);
        if (path == nullptr) {
            continue;
        }
        for (std::size_t keyIndex = 0; keyIndex < path->keys.size(); ++keyIndex) {
            if (path->keys[keyIndex].linkedCameraId == cameraId) {
                links.push_back({
                    .fileIndex = fileIndex,
                    .keyIndex = keyIndex,
                    .filePath = runtimeState.animationPanel.availableFiles[fileIndex],
                    .animationName = path->name,
                    .keyId = path->keys[keyIndex].id,
                });
            }
        }
    }
    return links;
}

bool IsMultiEditAllowedForCamera(const CameraPanelState& panel, const std::string& cameraId) {
    return std::find(
               panel.multiEditAllowedCameraIds.begin(),
               panel.multiEditAllowedCameraIds.end(),
               cameraId) != panel.multiEditAllowedCameraIds.end();
}

void SetMultiEditAllowedForCamera(CameraPanelState* panel, const std::string& cameraId, bool allowed) {
    if (panel == nullptr || cameraId.empty()) {
        return;
    }
    auto& allowedIds = panel->multiEditAllowedCameraIds;
    const auto existing = std::find(allowedIds.begin(), allowedIds.end(), cameraId);
    if (allowed && existing == allowedIds.end()) {
        allowedIds.push_back(cameraId);
    } else if (!allowed && existing != allowedIds.end()) {
        allowedIds.erase(existing);
    }
}

bool CanEditLinkedCamera(
    PreviewRuntimeState* runtimeState,
    const std::string& cameraId,
    const char* editLabel) {
    if (runtimeState == nullptr || cameraId.empty()) {
        return true;
    }

    const auto links = FindCameraAnimationLinks(*runtimeState, cameraId);
    if (links.size() <= 1U || IsMultiEditAllowedForCamera(runtimeState->cameraPanel, cameraId)) {
        return true;
    }

    runtimeState->statusMessage =
        "Linked to multiple animations. Enable Allow Multi-Animation Editing before changing them all.";
    runtimeState->errorMessage = editLabel != nullptr ? editLabel : std::string{};
    return false;
}

void PropagateCameraShotToLinkedAnimationKeys(
    PreviewRuntimeState* runtimeState,
    const CameraShot& shot) {
    if (runtimeState == nullptr || shot.id.empty()) {
        return;
    }

    for (std::size_t fileIndex = 0; fileIndex < runtimeState->animationPanel.availableFiles.size(); ++fileIndex) {
        auto* path = MutableRegistryAnimationPath(runtimeState, fileIndex);
        if (path == nullptr) {
            continue;
        }
        bool changed = false;
        for (auto& key : path->keys) {
            if (key.linkedCameraId != shot.id) {
                continue;
            }
            CopyCameraShotToAnimationKeySnapshot(shot, &key);
            changed = true;
        }
        if (changed) {
            MarkRegistryAnimationDirty(runtimeState, fileIndex);
        }
    }
}

void UnlinkCameraFromAnimations(PreviewRuntimeState* runtimeState, const std::string& cameraId) {
    if (runtimeState == nullptr || cameraId.empty()) {
        return;
    }

    for (std::size_t fileIndex = 0; fileIndex < runtimeState->animationPanel.availableFiles.size(); ++fileIndex) {
        auto* path = MutableRegistryAnimationPath(runtimeState, fileIndex);
        if (path == nullptr) {
            continue;
        }
        bool changed = false;
        for (auto& key : path->keys) {
            if (key.linkedCameraId != cameraId) {
                continue;
            }
            key.linkedCameraId.clear();
            key.linkedCameraName.clear();
            changed = true;
        }
        if (changed) {
            MarkRegistryAnimationDirty(runtimeState, fileIndex);
        }
    }
}

void UnlinkSingleAnimationKey(PreviewRuntimeState* runtimeState, const AnimationCameraLink& link) {
    if (runtimeState == nullptr) {
        return;
    }
    auto* path = MutableRegistryAnimationPath(runtimeState, link.fileIndex);
    if (path == nullptr || link.keyIndex >= path->keys.size()) {
        return;
    }
    auto& key = path->keys[link.keyIndex];
    key.linkedCameraId.clear();
    key.linkedCameraName.clear();
    MarkRegistryAnimationDirty(runtimeState, link.fileIndex);
    runtimeState->statusMessage = "Unlinked camera from " + path->name + ".";
    runtimeState->errorMessage.clear();
}

bool MoveLinkedAnimationKeyPoint(
    PreviewRuntimeState* runtimeState,
    AnimationPath* path,
    std::size_t keyIndex,
    AnimationEditTarget target,
    glm::vec3 point) {
    if (runtimeState == nullptr || path == nullptr || keyIndex >= path->keys.size()) {
        return false;
    }

    auto& key = path->keys[keyIndex];
    if (!key.linkedCameraId.empty() &&
        !CanEditLinkedCamera(runtimeState, key.linkedCameraId, "Linked key edit blocked.")) {
        return false;
    }

    if (target == AnimationEditTarget::Camera) {
        invisible_places::camera::MoveAnimationCameraKey(path, keyIndex, {point.x, point.y, point.z});
    } else {
        invisible_places::camera::MoveAnimationFocusKey(path, keyIndex, {point.x, point.y, point.z});
    }
    if (key.linkedCameraId.empty()) {
        return true;
    }

    const auto shotIndex = FindCameraShotIndexById(*runtimeState, key.linkedCameraId);
    if (!shotIndex.has_value() || shotIndex.value() >= runtimeState->cameraShots.size()) {
        return true;
    }

    auto& shot = runtimeState->cameraShots[shotIndex.value()];
    CopyAnimationKeyToCameraShot(key, &shot);
    shot.name = key.linkedCameraName.empty() ? shot.name : key.linkedCameraName;
    PropagateCameraShotToLinkedAnimationKeys(runtimeState, shot);
    return true;
}

void SyncAnimationSnapshotsFromLinkedCameras(PreviewRuntimeState* runtimeState, AnimationPath* path) {
    if (runtimeState == nullptr || path == nullptr) {
        return;
    }

    for (auto& key : path->keys) {
        const auto shotIndex = FindCameraShotIndexById(*runtimeState, key.linkedCameraId);
        if (!shotIndex.has_value() || shotIndex.value() >= runtimeState->cameraShots.size()) {
            continue;
        }
        CopyCameraShotToAnimationKeySnapshot(runtimeState->cameraShots[shotIndex.value()], &key);
    }
}

bool UntangleCameraAnimationLinks(PreviewRuntimeState* runtimeState, std::size_t shotIndex) {
    if (runtimeState == nullptr || shotIndex >= runtimeState->cameraShots.size()) {
        return false;
    }

    EnsureRuntimeCameraShotIds(runtimeState);
    const auto sourceCameraId = runtimeState->cameraShots[shotIndex].id;
    const auto links = FindCameraAnimationLinks(*runtimeState, sourceCameraId);
    if (links.size() <= 1U) {
        runtimeState->statusMessage = "This camera is not linked to multiple animation keys.";
        runtimeState->errorMessage.clear();
        return false;
    }

    std::size_t keepFileIndex = links.front().fileIndex;
    if (!runtimeState->animationPanel.currentFilePath.empty()) {
        for (const auto& link : links) {
            if (PathsLexicallyEqual(link.filePath, std::filesystem::path{runtimeState->animationPanel.currentFilePath})) {
                keepFileIndex = link.fileIndex;
                break;
            }
        }
    }

    struct DuplicateCameraLinkTarget {
        std::string id;
        std::string name;
    };
    std::unordered_map<std::size_t, DuplicateCameraLinkTarget> duplicateCamerasByFile;
    for (const auto& link : links) {
        if (link.fileIndex == keepFileIndex) {
            continue;
        }

        auto [duplicateIt, inserted] = duplicateCamerasByFile.emplace(link.fileIndex, DuplicateCameraLinkTarget{});
        if (inserted) {
            auto duplicate = runtimeState->cameraShots[shotIndex];
            duplicate.id = MakeUniqueCameraShotId(*runtimeState);
            duplicate.name = LinkedCameraShotName(link.animationName, link.keyIndex);
            runtimeState->cameraShots.push_back(std::move(duplicate));
            duplicateIt->second = {
                .id = runtimeState->cameraShots.back().id,
                .name = runtimeState->cameraShots.back().name,
            };
        }

        auto* path = MutableRegistryAnimationPath(runtimeState, link.fileIndex);
        if (path == nullptr) {
            continue;
        }
        bool changed = false;
        for (auto& key : path->keys) {
            if (key.linkedCameraId == sourceCameraId) {
                key.linkedCameraId = duplicateIt->second.id;
                key.linkedCameraName = duplicateIt->second.name;
                changed = true;
            }
        }
        if (changed) {
            MarkRegistryAnimationDirty(runtimeState, link.fileIndex);
        }
    }

    SetMultiEditAllowedForCamera(&runtimeState->cameraPanel, sourceCameraId, false);
    EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());
    runtimeState->statusMessage = "Untangled linked camera into separate animation cameras.";
    runtimeState->errorMessage.clear();
    return true;
}

float AnimationDurationSeconds(const AnimationPath& path) {
    return invisible_places::camera::AnimationPathDurationSeconds(path);
}

void SetCameraDepthOfFieldEnabled(PreviewRuntimeState* runtimeState, bool enabled) {
    if (runtimeState == nullptr) {
        return;
    }

    auto cameraState = runtimeState->camera.CaptureState();
    if (cameraState.hasDepthOfField == enabled) {
        return;
    }

    cameraState.hasDepthOfField = enabled;
    runtimeState->camera.ApplyState(cameraState);
}

void ApplyAnimationEvaluation(
    PreviewRuntimeState* runtimeState,
    const AnimationPath& path,
    float amount,
    bool allowDepthOfField) {
    if (runtimeState == nullptr || path.keys.size() < 2U) {
        return;
    }

    auto evaluation = invisible_places::camera::EvaluateAnimationPath(
        path,
        AnimationDurationSeconds(path) * std::clamp(amount, 0.0F, 1.0F));
    evaluation.camera.hasDepthOfField = evaluation.camera.hasDepthOfField && allowDepthOfField;
    runtimeState->camera.ApplyState(evaluation.camera);
    runtimeState->pivotOverlay.visible = true;
    runtimeState->pivotOverlay.pivot = FromGlm(glm::vec3{
        evaluation.focusPoint[0],
        evaluation.focusPoint[1],
        evaluation.focusPoint[2]});
    runtimeState->pivotOverlay.lastSetAt = std::chrono::steady_clock::now();
}

bool SaveAnimationPathToFile(
    PreviewRuntimeState* runtimeState,
    const AnimationPath& path,
    const std::filesystem::path& outputPath) {
    if (runtimeState == nullptr) {
        return false;
    }

    const bool savingCurrentPath =
        runtimeState->animationPanel.currentPath.has_value() &&
        &path == &runtimeState->animationPanel.currentPath.value();
    auto pathToSave = path;
    if (savingCurrentPath) {
        pathToSave.waterAnimationTrailSettings = ViewedWaterAnimationTrailSettings(*runtimeState);
        pathToSave.tempWaterAnimationTrailSettings.reset();
        runtimeState->animationPanel.currentPath = pathToSave;
    }

    std::string errorMessage;
    if (!invisible_places::serialization::SaveAnimationPath(pathToSave, outputPath, &errorMessage)) {
        runtimeState->errorMessage = errorMessage.empty() ? "Failed to save animation path." : errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }

    AddAnimationFileToRegistry(
        &runtimeState->animationPanel,
        outputPath,
        pathToSave.associatedLayerPaths);
    SetAnimationRegistryAssociations(
        &runtimeState->animationPanel,
        outputPath,
        pathToSave.associatedLayerPaths);
    runtimeState->animationPanel.animationRegistryInitialized = true;
    if (!savingCurrentPath) {
        runtimeState->animationPanel.currentPath = pathToSave;
    }
    runtimeState->animationPanel.currentFilePath = outputPath.string();
    runtimeState->animationPanel.draftAnimationName = pathToSave.name;
    runtimeState->animationPanel.dirty = false;
    if (const auto registryIndex = FindAnimationRegistryIndex(runtimeState->animationPanel, outputPath);
        registryIndex.has_value()) {
        EnsureAnimationAssociationStorage(&runtimeState->animationPanel);
        runtimeState->animationPanel.availableFileLoadedPaths[registryIndex.value()] = pathToSave;
        runtimeState->animationPanel.availableFileDirtyFlags[registryIndex.value()] = false;
    }
    RefreshAnimationFileList(&runtimeState->animationPanel, AnimationDirectory(*runtimeState));
    runtimeState->statusMessage = "Saved animation path: " + outputPath.filename().string() + ".";
    runtimeState->errorMessage.clear();
    return true;
}

bool ExportCurrentAnimationCameraToHoudini(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->animationPanel.currentPath.has_value()) {
        return false;
    }

    const auto& animationPath = runtimeState->animationPanel.currentPath.value();
    const auto outputPath = HoudiniCameraScriptPathForAnimation(*runtimeState, animationPath.name);
    std::string errorMessage;
    if (!invisible_places::output::WriteHoudiniCameraScript(
            animationPath,
            runtimeState->renderSettings,
            outputPath,
            &errorMessage)) {
        runtimeState->errorMessage =
            errorMessage.empty() ? "Failed to export Houdini camera script." : errorMessage;
        runtimeState->statusMessage.clear();
        runtimeState->animationPanel.showHoudiniCameraExportNotice = false;
        return false;
    }

    runtimeState->animationPanel.lastHoudiniCameraScriptPath = outputPath;
    runtimeState->animationPanel.lastHoudiniCameraExportDirectory = outputPath.parent_path();
    runtimeState->animationPanel.showHoudiniCameraExportNotice = true;
    runtimeState->statusMessage = "Exported Houdini camera script. File: " + outputPath.string() + ".";
    runtimeState->errorMessage.clear();
    return true;
}

bool WriteHoudiniCameraImporterScript(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return false;
    }

    const auto outputPath = HoudiniCameraImportScriptPath(*runtimeState);
    std::string errorMessage;
    if (!invisible_places::output::WriteHoudiniCameraImportScript(outputPath, &errorMessage)) {
        runtimeState->errorMessage =
            errorMessage.empty() ? "Failed to write Houdini camera importer script." : errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }

    runtimeState->statusMessage =
        "Wrote Houdini camera importer: " + outputPath.string() +
        ". Run with hython and then click Import Houdini Camera.";
    runtimeState->errorMessage.clear();
    return true;
}

std::optional<std::filesystem::path> LatestHoudiniCameraImportJsonPath(
    const PreviewRuntimeState& runtimeState) {
    const auto directory = HoudiniCameraExportDirectory(runtimeState);
    std::error_code iterateError;
    std::optional<std::filesystem::path> latestPath;
    std::filesystem::file_time_type latestWriteTime{};
    for (const auto& entry : std::filesystem::directory_iterator{directory, iterateError}) {
        if (iterateError) {
            break;
        }
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto path = entry.path();
        const auto filename = path.filename().string();
        if (!filename.ends_with("_invisible_places_camera.json")) {
            continue;
        }
        std::error_code timeError;
        const auto writeTime = std::filesystem::last_write_time(path, timeError);
        if (timeError) {
            continue;
        }
        if (!latestPath.has_value() || writeTime > latestWriteTime) {
            latestPath = path;
            latestWriteTime = writeTime;
        }
    }
    return latestPath;
}

bool ImportLatestHoudiniCameraAnimation(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return false;
    }

    const auto inputPath = LatestHoudiniCameraImportJsonPath(*runtimeState);
    if (!inputPath.has_value()) {
        runtimeState->errorMessage =
            "No Houdini camera JSON files were found in " +
            HoudiniCameraExportDirectory(*runtimeState).string() +
            ". Run import_houdini_camera.py with hython first.";
        runtimeState->statusMessage.clear();
        return false;
    }

    std::string errorMessage;
    auto importedPath = invisible_places::output::LoadHoudiniCameraAnimationPath(inputPath.value(), &errorMessage);
    if (!importedPath.has_value()) {
        runtimeState->errorMessage =
            errorMessage.empty() ? "Failed to import Houdini camera JSON." : errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }

    importedPath->associatedLayerPaths = VisibleAssociatedLidarLayerPaths(*runtimeState);
    NormalizeAssociatedLayerPaths(&importedPath->associatedLayerPaths);
    const auto outputPath = AnimationFilePathForName(*runtimeState, importedPath->name);
    if (!SaveAnimationPathToFile(runtimeState, importedPath.value(), outputPath)) {
        return false;
    }
    runtimeState->renderSettings = RenderSettingsFromAnimationExportSettings(importedPath->exportSettings);
    ApplyAnimationEvaluation(runtimeState, runtimeState->animationPanel.currentPath.value(), 0.0F, false);
    runtimeState->statusMessage =
        "Imported Houdini camera from " + inputPath->filename().string() +
        " as " + outputPath.filename().string() + ".";
    runtimeState->errorMessage.clear();
    return true;
}

bool LoadAnimationPathFromFile(PreviewRuntimeState* runtimeState, const std::filesystem::path& inputPath) {
    if (runtimeState == nullptr) {
        return false;
    }

    std::string errorMessage;
    const auto path = invisible_places::serialization::LoadAnimationPath(inputPath, &errorMessage);
    if (!path.has_value()) {
        runtimeState->errorMessage = errorMessage.empty() ? "Failed to load animation path." : errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }

    auto loadedPath = path.value();
    if (const auto registryIndex = FindAnimationRegistryIndex(runtimeState->animationPanel, inputPath);
        registryIndex.has_value()) {
        loadedPath.associatedLayerPaths =
            AnimationRegistryAssociationPaths(runtimeState->animationPanel, registryIndex.value());
    }
    NormalizeAssociatedLayerPaths(&loadedPath.associatedLayerPaths);
    SyncAnimationSnapshotsFromLinkedCameras(runtimeState, &loadedPath);
    loadedPath.name = AnimationNameFromFilePath(inputPath);

    runtimeState->animationPanel.currentPath = std::move(loadedPath);
    RemoveUnexportableVisualNames(&runtimeState->animationPanel.currentPath.value());
    if (runtimeState->animationPanel.currentPath->tempWaterPointVisualStyle.has_value()) {
        ImportLegacyWaterPointVisualStyle(
            runtimeState,
            runtimeState->animationPanel.currentPath->tempWaterPointVisualStyle.value());
    } else if (runtimeState->animationPanel.currentPath->waterPointVisualStyle.has_value()) {
        ImportLegacyWaterPointVisualStyle(
            runtimeState,
            runtimeState->animationPanel.currentPath->waterPointVisualStyle.value());
    }
    runtimeState->animationPanel.currentFilePath = inputPath.string();
    runtimeState->animationPanel.draftAnimationName = runtimeState->animationPanel.currentPath->name;
    runtimeState->renderSettings =
        RenderSettingsFromAnimationExportSettings(runtimeState->animationPanel.currentPath->exportSettings);
    runtimeState->animationPanel.selectedKeyIndex =
        runtimeState->animationPanel.currentPath->keys.empty() ? std::nullopt : std::optional<std::size_t>{0};
    runtimeState->animationPanel.scrubAmount = 0.0F;
    runtimeState->animationPanel.previewDepthOfField = false;
    runtimeState->animationPanel.dirty = false;
    SyncWaterAnimationTrailProfileFromCurrentAnimation(runtimeState);
    ApplyWaterPointVisualStyleToGeneratedSessions(runtimeState);
    MarkWaterPathDirty(runtimeState);
    if (const auto registryIndex = FindAnimationRegistryIndex(runtimeState->animationPanel, inputPath);
        registryIndex.has_value()) {
        EnsureAnimationAssociationStorage(&runtimeState->animationPanel);
        runtimeState->animationPanel.availableFileLoadedPaths[registryIndex.value()] =
            runtimeState->animationPanel.currentPath.value();
        runtimeState->animationPanel.availableFileDirtyFlags[registryIndex.value()] = false;
    }
    runtimeState->animationPlayback.active = false;
    runtimeState->cameraPlayback.active = false;
    ApplyAnimationEvaluation(runtimeState, runtimeState->animationPanel.currentPath.value(), 0.0F, false);
    runtimeState->statusMessage = "Loaded animation path: " + inputPath.filename().string() + ".";
    runtimeState->errorMessage.clear();
    return true;
}

bool PathsReferToSameFile(const std::filesystem::path& left, const std::filesystem::path& right) {
    if (left.empty() || right.empty()) {
        return false;
    }

    std::error_code equivalentError;
    if (std::filesystem::equivalent(left, right, equivalentError)) {
        return true;
    }

    return left.lexically_normal() == right.lexically_normal();
}

std::optional<std::size_t> FindAnimationFileIndex(
    const std::vector<std::filesystem::path>& files,
    const std::filesystem::path& path) {
    const auto normalized = path.lexically_normal();
    for (std::size_t index = 0; index < files.size(); ++index) {
        if (files[index].lexically_normal() == normalized) {
            return index;
        }
    }
    return std::nullopt;
}

void BeginAnimationFileRename(AnimationPanelState* panel, std::size_t fileIndex) {
    if (panel == nullptr || fileIndex >= panel->availableFiles.size()) {
        return;
    }

    panel->selectedFileIndex = fileIndex;
    panel->renamingFileIndex = fileIndex;
    panel->fileRenameBuffer = AnimationDisplayNameFromPath(panel->availableFiles[fileIndex]);
    panel->focusFileRename = true;
}

bool CommitAnimationFileRename(PreviewRuntimeState* runtimeState, std::size_t fileIndex) {
    if (runtimeState == nullptr) {
        return false;
    }

    auto& panel = runtimeState->animationPanel;
    if (fileIndex >= panel.availableFiles.size()) {
        panel.renamingFileIndex.reset();
        panel.fileRenameBuffer.clear();
        panel.focusFileRename = false;
        return false;
    }

    const auto oldPath = panel.availableFiles[fileIndex];
    const auto cleanName = NormalizeAnimationNameFromInput(panel.fileRenameBuffer);
    const auto newPath = oldPath.parent_path() / (SanitizeAnimationFileStem(cleanName) + ".ipanim.json");
    panel.renamingFileIndex.reset();
    panel.fileRenameBuffer.clear();
    panel.focusFileRename = false;

    if (newPath.lexically_normal() == oldPath.lexically_normal()) {
        return false;
    }

    std::error_code existsError;
    if (std::filesystem::exists(newPath, existsError)) {
        runtimeState->errorMessage = "An animation named " + newPath.filename().string() + " already exists.";
        runtimeState->statusMessage.clear();
        return false;
    }

    std::error_code renameError;
    std::filesystem::rename(oldPath, newPath, renameError);
    if (renameError) {
        runtimeState->errorMessage = "Failed to rename animation: " + renameError.message();
        runtimeState->statusMessage.clear();
        return false;
    }

    panel.availableFiles[fileIndex] = newPath.lexically_normal();
    const bool renamedCurrent =
        !panel.currentFilePath.empty() &&
        PathsReferToSameFile(std::filesystem::path{panel.currentFilePath}, oldPath);
    const bool renamedExportSelection = AnimationFileSelectedForExport(panel, oldPath);
    if (renamedExportSelection) {
        SetAnimationFileSelectedForExport(&panel, oldPath, false);
        SetAnimationFileSelectedForExport(&panel, newPath, true);
    }
    std::string saveError;
    bool savedInternalName = true;
    if (renamedCurrent && panel.currentPath.has_value()) {
        panel.currentPath->name = cleanName;
        panel.currentPath->associatedLayerPaths = AnimationRegistryAssociationPaths(panel, fileIndex);
        panel.currentFilePath = newPath.string();
        panel.draftAnimationName = cleanName;
        if (invisible_places::serialization::SaveAnimationPath(panel.currentPath.value(), newPath, &saveError)) {
            panel.dirty = false;
        } else {
            savedInternalName = false;
        }
    } else {
        auto renamedPath = invisible_places::serialization::LoadAnimationPath(newPath, &saveError);
        if (renamedPath.has_value()) {
            renamedPath->name = cleanName;
            renamedPath->associatedLayerPaths = AnimationRegistryAssociationPaths(panel, fileIndex);
            savedInternalName =
                invisible_places::serialization::SaveAnimationPath(renamedPath.value(), newPath, &saveError);
        } else {
            savedInternalName = false;
        }
    }

    RefreshAnimationFileList(&panel, AnimationDirectory(*runtimeState));
    panel.selectedFileIndex = FindAnimationFileIndex(panel.availableFiles, newPath);
    runtimeState->statusMessage = "Renamed animation to " + newPath.filename().string() + ".";
    runtimeState->errorMessage =
        savedInternalName
            ? std::string{}
            : "Renamed the file, but could not update the animation name inside it: " + saveError;
    return true;
}

void SaveCurrentCameraPathAsAnimation(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    EnsureRuntimeCameraShotIds(runtimeState);
    const auto animationName = runtimeState->animationPanel.draftAnimationName.empty()
                                   ? std::string{"Animation"}
                                   : runtimeState->animationPanel.draftAnimationName;
    for (std::size_t pathItemIndex = 0; pathItemIndex < runtimeState->cameraPanel.pathShotIndices.size(); ++pathItemIndex) {
        const auto shotIndex = runtimeState->cameraPanel.pathShotIndices[pathItemIndex];
        if (shotIndex >= runtimeState->cameraShots.size()) {
            continue;
        }
        auto& shot = runtimeState->cameraShots[shotIndex];
        if (FindCameraAnimationLinks(*runtimeState, shot.id).empty()) {
            shot.name = LinkedCameraShotName(animationName, pathItemIndex);
        }
    }

    const auto pathShots = BuildPanelCameraPathShots(*runtimeState);
    if (pathShots.size() < 2U) {
        runtimeState->errorMessage = "Add at least two camera path entries before saving an animation.";
        runtimeState->statusMessage.clear();
        return;
    }

    auto animationPath = invisible_places::camera::BuildAnimationPathFromCameraShots(
        animationName,
        pathShots,
        runtimeState->cameraPanel.pathDurationFrames,
        runtimeState->animationPanel.currentPath.has_value()
            ? runtimeState->animationPanel.currentPath->apertureFStops
            : 8.0F);
    animationPath.associatedLayerPaths = VisibleAssociatedLidarLayerPaths(*runtimeState);
    animationPath.exportSettings = AnimationExportSettingsFromRenderSettings(runtimeState->renderSettings);
    const auto outputPath = UniqueAnimationFilePathForName(*runtimeState, animationPath.name);
    animationPath.name = AnimationNameFromFilePath(outputPath);
    SaveAnimationPathToFile(runtimeState, animationPath, outputPath);
}

void ApplyAnimationScrub(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->animationPanel.currentPath.has_value()) {
        return;
    }

    ApplyAnimationEvaluation(
        runtimeState,
        runtimeState->animationPanel.currentPath.value(),
        runtimeState->animationPanel.scrubAmount,
        runtimeState->animationPanel.previewDepthOfField);
    runtimeState->animationPlayback.active = false;
    runtimeState->cameraPlayback.active = false;
}

void StartAnimationPlayback(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->animationPanel.currentPath.has_value()) {
        return;
    }

    const auto& path = runtimeState->animationPanel.currentPath.value();
    if (path.keys.size() < 2U) {
        runtimeState->errorMessage = "Load an animation with at least two keys before playback.";
        runtimeState->statusMessage.clear();
        return;
    }

    runtimeState->animationPlayback = {
        .active = true,
        .path = path,
        .durationSeconds = AnimationDurationSeconds(path),
        .startedAt = std::chrono::steady_clock::now(),
    };
    runtimeState->cameraPlayback.active = false;
    runtimeState->statusMessage = "Playing animation path.";
    runtimeState->errorMessage.clear();
}

void StopAnimationPlayback(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    runtimeState->animationPlayback.active = false;
    if (!runtimeState->animationPanel.previewDepthOfField) {
        SetCameraDepthOfFieldEnabled(runtimeState, false);
    }
}

void UpdateAnimationPlayback(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->animationPlayback.active) {
        return;
    }

    const auto& playback = runtimeState->animationPlayback;
    const float elapsedSeconds =
        std::chrono::duration<float>(std::chrono::steady_clock::now() - playback.startedAt).count();
    const float t = std::clamp(elapsedSeconds / std::max(0.001F, playback.durationSeconds), 0.0F, 1.0F);
    runtimeState->animationPanel.scrubAmount = t;
    ApplyAnimationEvaluation(runtimeState, playback.path, t, true);

    if (t >= 1.0F) {
        StopAnimationPlayback(runtimeState);
        runtimeState->statusMessage = "Animation playback complete.";
    }
}

std::string NextCameraShotName(const PreviewRuntimeState& runtimeState) {
    std::unordered_set<std::string> usedNames;
    for (const auto& shot : runtimeState.cameraShots) {
        usedNames.insert(shot.name);
    }
    for (std::size_t ordinal = runtimeState.cameraShots.size() + 1U;; ++ordinal) {
        auto candidate = "Shot_" + FormatThreeDigitOrdinal(ordinal);
        if (!usedNames.contains(candidate)) {
            return candidate;
        }
    }
}

void SaveCurrentCameraShot(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    CameraShot shot;
    shot.id = MakeUniqueCameraShotId(*runtimeState);
    shot.name = runtimeState->cameraPanel.draftShotName.empty()
                    ? NextCameraShotName(*runtimeState)
                    : runtimeState->cameraPanel.draftShotName;
    shot.state = runtimeState->camera.CaptureState();
    shot.associatedLayerPaths = VisibleAssociatedLidarLayerPaths(*runtimeState);
    runtimeState->cameraShots.push_back(std::move(shot));

    const auto savedIndex = runtimeState->cameraShots.size() - 1U;
    runtimeState->cameraPanel.selectedShotIndex = savedIndex;
    runtimeState->cameraPanel.pathShotIndices.push_back(savedIndex);
    runtimeState->cameraPanel.selectedPathItemIndex = runtimeState->cameraPanel.pathShotIndices.size() - 1U;
    if (!runtimeState->cameraPanel.blendFromIndex.has_value()) {
        runtimeState->cameraPanel.blendFromIndex = savedIndex;
    } else {
        runtimeState->cameraPanel.blendToIndex = savedIndex;
    }
    runtimeState->cameraPanel.draftShotName = NextCameraShotName(*runtimeState);
    runtimeState->statusMessage = "Saved camera position.";
    runtimeState->errorMessage.clear();
}

void ApplyCameraShot(PreviewRuntimeState* runtimeState, std::size_t shotIndex) {
    if (runtimeState == nullptr || shotIndex >= runtimeState->cameraShots.size()) {
        return;
    }

    runtimeState->camera.ApplyState(runtimeState->cameraShots[shotIndex].state);
    runtimeState->cameraPlayback.active = false;
    runtimeState->cameraPanel.selectedShotIndex = shotIndex;
    runtimeState->statusMessage = "Loaded camera shot " + runtimeState->cameraShots[shotIndex].name + ".";
    runtimeState->errorMessage.clear();
}

bool UpdateCameraShotFromCurrentView(PreviewRuntimeState* runtimeState, std::size_t shotIndex) {
    if (runtimeState == nullptr || shotIndex >= runtimeState->cameraShots.size()) {
        return false;
    }

    EnsureRuntimeCameraShotIds(runtimeState);
    auto& shot = runtimeState->cameraShots[shotIndex];
    if (!CanEditLinkedCamera(runtimeState, shot.id, "Camera update blocked.")) {
        return false;
    }

    shot.state = runtimeState->camera.CaptureState();
    PropagateCameraShotToLinkedAnimationKeys(runtimeState, shot);
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPlayback.active = false;
    runtimeState->statusMessage = "Updated camera " + shot.name + " from the current view.";
    runtimeState->errorMessage.clear();
    return true;
}

void DeleteCameraShot(PreviewRuntimeState* runtimeState, std::size_t shotIndex) {
    if (runtimeState == nullptr || shotIndex >= runtimeState->cameraShots.size()) {
        return;
    }

    const auto deletedName = runtimeState->cameraShots[shotIndex].name;
    const auto deletedId = runtimeState->cameraShots[shotIndex].id;
    UnlinkCameraFromAnimations(runtimeState, deletedId);
    SetMultiEditAllowedForCamera(&runtimeState->cameraPanel, deletedId, false);
    RemoveCameraShotFromPath(&runtimeState->cameraPanel, shotIndex);
    runtimeState->cameraShots.erase(runtimeState->cameraShots.begin() + static_cast<std::ptrdiff_t>(shotIndex));
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPlayback.active = false;
    runtimeState->cameraPanel.pendingLinkedShotDeleteIndex.reset();
    EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());
    runtimeState->cameraPanel.draftShotName = NextCameraShotName(*runtimeState);
    runtimeState->statusMessage = "Deleted camera " + deletedName + ".";
    runtimeState->errorMessage.clear();
}

void BeginCameraShotRename(PreviewRuntimeState* runtimeState, std::size_t shotIndex) {
    if (runtimeState == nullptr || shotIndex >= runtimeState->cameraShots.size()) {
        return;
    }

    runtimeState->cameraPanel.selectedShotIndex = shotIndex;
    runtimeState->cameraPanel.renamingShotIndex = shotIndex;
    runtimeState->cameraPanel.shotRenameBuffer = runtimeState->cameraShots[shotIndex].name;
    runtimeState->cameraPanel.focusShotRename = true;
}

void CommitCameraShotRename(PreviewRuntimeState* runtimeState, std::size_t shotIndex) {
    if (runtimeState == nullptr || shotIndex >= runtimeState->cameraShots.size()) {
        return;
    }

    auto name = TrimText(runtimeState->cameraPanel.shotRenameBuffer);
    if (name.empty()) {
        name = "Shot_" + FormatThreeDigitOrdinal(shotIndex + 1U);
    }

    if (!CanEditLinkedCamera(runtimeState, runtimeState->cameraShots[shotIndex].id, "Camera rename blocked.")) {
        runtimeState->cameraPanel.renamingShotIndex.reset();
        runtimeState->cameraPanel.shotRenameBuffer.clear();
        runtimeState->cameraPanel.focusShotRename = false;
        return;
    }

    runtimeState->cameraShots[shotIndex].name = name;
    PropagateCameraShotToLinkedAnimationKeys(runtimeState, runtimeState->cameraShots[shotIndex]);
    runtimeState->cameraPanel.renamingShotIndex.reset();
    runtimeState->cameraPanel.shotRenameBuffer.clear();
    runtimeState->cameraPanel.focusShotRename = false;
    runtimeState->cameraPlayback.active = false;
    runtimeState->statusMessage = "Renamed camera shot to " + name + ".";
    runtimeState->errorMessage.clear();
}

void ApplyCameraBlend(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    const auto pathShots = BuildPanelCameraPathShots(*runtimeState);
    if (pathShots.size() < 2U) {
        return;
    }

    const auto timing = invisible_places::camera::BuildCameraPathTiming(
        pathShots,
        0,
        pathShots.size() - 1U);
    if (!timing.IsValid()) {
        return;
    }

    const float timeSeconds =
        timing.DurationSeconds() * std::clamp(runtimeState->cameraPanel.blendAmount, 0.0F, 1.0F);
    runtimeState->camera.ApplyState(invisible_places::camera::EvaluateCameraPath(
        pathShots,
        timing,
        timeSeconds));
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPlayback.active = false;
}

void StartCameraPlayback(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    const auto pathShots = BuildPanelCameraPathShots(*runtimeState);
    if (pathShots.size() < 2U) {
        runtimeState->errorMessage = "Add at least two camera path entries before playback.";
        runtimeState->statusMessage.clear();
        return;
    }
    const auto timing = invisible_places::camera::BuildCameraPathTiming(pathShots, 0, pathShots.size() - 1U);
    if (!timing.IsValid()) {
        runtimeState->errorMessage = "Add at least two camera path entries before playback.";
        runtimeState->statusMessage.clear();
        return;
    }

    runtimeState->cameraPlayback = {
        .active = true,
        .pathShotIndices = runtimeState->cameraPanel.pathShotIndices,
        .durationFrames = runtimeState->cameraPanel.pathDurationFrames,
        .durationSeconds = timing.DurationSeconds(),
        .startedAt = std::chrono::steady_clock::now(),
    };
    runtimeState->animationPlayback.active = false;
    runtimeState->statusMessage =
        "Playing smooth camera path through " + std::to_string(pathShots.size()) + " entries.";
    runtimeState->errorMessage.clear();
}

void UpdateCameraShotPlayback(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->cameraPlayback.active) {
        return;
    }

    const auto& playback = runtimeState->cameraPlayback;
    const auto pathShots = BuildOrderedCameraPathShots(
        runtimeState->cameraShots,
        playback.pathShotIndices,
        playback.durationFrames);
    if (pathShots.size() < 2U) {
        runtimeState->cameraPlayback.active = false;
        return;
    }

    const auto timing = invisible_places::camera::BuildCameraPathTiming(pathShots, 0, pathShots.size() - 1U);
    if (!timing.IsValid()) {
        runtimeState->cameraPlayback.active = false;
        return;
    }

    const float elapsedSeconds =
        std::chrono::duration<float>(std::chrono::steady_clock::now() - playback.startedAt).count();
    const float t = std::clamp(elapsedSeconds / std::max(0.001F, playback.durationSeconds), 0.0F, 1.0F);
    runtimeState->cameraPanel.blendAmount = t;
    runtimeState->camera.ApplyState(invisible_places::camera::EvaluateCameraPath(
        pathShots,
        timing,
        timing.DurationSeconds() * t));

    if (t >= 1.0F) {
        runtimeState->cameraPlayback.active = false;
        runtimeState->statusMessage = "Camera path playback complete.";
    }
}

std::vector<invisible_places::camera::CameraState> BuildCurrentCameraRenderSequence(
    const PreviewRuntimeState& runtimeState,
    const RenderJobSettings& settings) {
    const auto pathShots = BuildPanelCameraPathShots(runtimeState);
    if (pathShots.size() < 2U) {
        return {};
    }

    auto pathSettings = settings;
    pathSettings.fromShotIndex = 0;
    pathSettings.toShotIndex = pathShots.size() - 1U;
    return invisible_places::output::BuildCameraRenderSequence(pathShots, pathSettings);
}

std::vector<invisible_places::camera::CameraState> BuildCurrentAnimationRenderSequence(
    const PreviewRuntimeState& runtimeState,
    const RenderJobSettings& settings) {
    if (!runtimeState.animationPanel.currentPath.has_value()) {
        return {};
    }

    return invisible_places::output::BuildAnimationRenderSequence(
        runtimeState.animationPanel.currentPath.value(),
        settings);
}

std::vector<invisible_places::camera::CameraState> BuildStillCameraRenderSequence(
    const PreviewRuntimeState& runtimeState,
    const RenderJobSettings& settings) {
    return invisible_places::output::BuildStillCameraRenderSequence(
        runtimeState.camera.CaptureState(),
        settings);
}

std::vector<OfflinePointLayerSnapshot> BuildOfflinePointLayerSnapshots(
    const PreviewRuntimeState& runtimeState) {
    std::vector<OfflinePointLayerSnapshot> layers;
    for (const auto& session : runtimeState.sessions) {
        if (!session.loaded ||
            !session.visible ||
            session.kind != LayerKind::PointCloud ||
            session.offlinePointCloud == nullptr) {
            continue;
        }

        auto style = IsGeneratedWaterOverlaySession(session)
                         ? MakeWaterTrailExportStyle(session.pointStyle)
                         : session.pointStyle;
        style = ApplyWaterCausticRenderStyle(runtimeState, session, style, false);
        layers.push_back(
            {.cloud = session.offlinePointCloud,
             .style = FastBasicPointRendererActive(runtimeState.projectSettings)
                          ? MakeEffectiveFastBasicStyle(
                                style,
                                session.hasSourceRgb,
                                IsGeneratedWaterOverlaySession(session))
                          : style,
             .hasSourceRgb = session.hasSourceRgb,
             .fastBasic = FastBasicPointRendererActive(runtimeState.projectSettings),
             .drawPointCount = FastBasicPointRendererActive(runtimeState.projectSettings)
                                   ? session.pointBudget.activePoints
                                   : session.totalPrimitives,
             .localToWorld = glm::mat4{1.0F}});
    }
    return layers;
}

std::string NormalizeMotionScalarFieldName(std::string_view name) {
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

std::optional<std::size_t> FindMotionScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    std::initializer_list<std::string_view> exactNames,
    std::string_view containsName) {
    std::optional<std::size_t> containsMatch;
    for (std::size_t index = 0; index < scalarFields.size(); ++index) {
        const auto normalized = NormalizeMotionScalarFieldName(scalarFields[index].name);
        for (const auto exactName : exactNames) {
            if (normalized == exactName) {
                return index;
            }
        }
        if (!containsMatch.has_value() && normalized.find(containsName) != std::string::npos) {
            containsMatch = index;
        }
    }
    return containsMatch;
}

std::optional<std::size_t> FindRoughnessMotionScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindMotionScalarFieldSlot(
        scalarFields,
        {"roughness", "scalarroughness"},
        "roughness");
}

std::optional<std::size_t> FindGroundIdMotionScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindMotionScalarFieldSlot(
        scalarFields,
        {"groundid", "scalargroundid"},
        "groundid");
}

std::optional<std::size_t> FindCausticMaskScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindMotionScalarFieldSlot(
        scalarFields,
        {"causticmask", "scalarcausticmask"},
        "causticmask");
}

std::optional<std::size_t> FindCausticEdgeScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindMotionScalarFieldSlot(
        scalarFields,
        {"causticedge", "scalarcausticedge"},
        "causticedge");
}

std::optional<std::size_t> FindCausticRegionScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindMotionScalarFieldSlot(
        scalarFields,
        {"causticregionid", "scalarcausticregionid"},
        "causticregionid");
}

std::optional<std::size_t> FindCausticSeedScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindMotionScalarFieldSlot(
        scalarFields,
        {"causticseed", "scalarcausticseed"},
        "causticseed");
}

bool HasCausticRegionForSession(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    if (session.kind != LayerKind::PointCloud || IsGeneratedWaterOverlaySession(session)) {
        return false;
    }
    const auto sourceKey = NormalizePathKey(session.sourcePath);
    return std::any_of(
        runtimeState.water.causticRegions.begin(),
        runtimeState.water.causticRegions.end(),
        [&](const WaterCausticRegion& region) {
            return region.enabled &&
                   region.vertices.size() >= 3U &&
                   NormalizePathKey(region.targetLayerSourcePath) == sourceKey;
        });
}

struct CausticPreviewTintState {
    float amount = 0.0F;
    float regionId = 0.0F;
};

CausticPreviewTintState ResolveCausticPreviewTintState(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
    if (runtimeState.water.activeRegionFeature != WaterRegionFeature::Caustic ||
        !runtimeState.water.selectedCausticRegionIndex.has_value() ||
        runtimeState.water.selectedCausticRegionIndex.value() >= runtimeState.water.causticRegions.size()) {
        return {};
    }

    const auto& region = runtimeState.water.causticRegions[runtimeState.water.selectedCausticRegionIndex.value()];
    if (!region.enabled ||
        NormalizePathKey(region.targetLayerSourcePath) != NormalizePathKey(session.sourcePath)) {
        return {};
    }

    constexpr float kCausticPreviewTintAmount = 0.30F;
    constexpr double kCausticPreviewPulseSeconds = 4.0;
    switch (region.previewTintMode) {
        case WaterCausticPreviewTintMode::Always:
            return {.amount = kCausticPreviewTintAmount, .regionId = static_cast<float>(region.id)};
        case WaterCausticPreviewTintMode::PulseAfterRefresh: {
            const auto pulseIt = runtimeState.water.causticPreviewTintPulseUntilSeconds.find(region.id);
            if (pulseIt == runtimeState.water.causticPreviewTintPulseUntilSeconds.end()) {
                return {};
            }
            const double remaining = pulseIt->second - ImGui::GetTime();
            if (remaining <= 0.0) {
                return {};
            }
            return {
                .amount = kCausticPreviewTintAmount *
                          std::clamp(static_cast<float>(remaining / kCausticPreviewPulseSeconds), 0.0F, 1.0F),
                .regionId = static_cast<float>(region.id)};
        }
        case WaterCausticPreviewTintMode::Off:
            return {};
    }
    return {};
}

PointCloudStyleState ApplyWaterCausticRenderStyle(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session,
    PointCloudStyleState style,
    bool includeEditorPreview) {
    const auto& look = ViewedWaterCausticLookSettings(runtimeState);
    const auto maskSlot = FindCausticMaskScalarFieldSlot(session.scalarFields);
    const auto edgeSlot = FindCausticEdgeScalarFieldSlot(session.scalarFields);
    const auto regionSlot = FindCausticRegionScalarFieldSlot(session.scalarFields);
    const auto seedSlot = FindCausticSeedScalarFieldSlot(session.scalarFields);
    const auto regionOrSeedSlot = regionSlot.has_value() ? regionSlot : seedSlot;
    const auto previewTint = includeEditorPreview
                                 ? ResolveCausticPreviewTintState(runtimeState, session)
                                 : CausticPreviewTintState{};
    const bool active =
        HasCausticRegionForSession(runtimeState, session) &&
        maskSlot.has_value() &&
        edgeSlot.has_value() &&
        regionOrSeedSlot.has_value() &&
        ((look.enabled && look.intensity > 1.0e-5F) ||
         previewTint.amount > 1.0e-5F);
    style.causticAnimation = active;
    style.causticIntensity = active && look.enabled ? look.intensity : 0.0F;
    style.causticScale = look.scale;
    style.causticSpeed = look.speed;
    style.causticLineSharpness = look.lineSharpness;
    style.causticWarp = look.warp;
    style.causticCellSizeMeters = look.cellSizeMeters;
    style.causticLineWidthMeters = look.lineWidthMeters;
    style.causticFeatherMeters = look.featherMeters;
    style.causticSurfacePointSpacingMeters = look.surfacePointSpacingMeters;
    style.causticWarpAmplitudeMeters = look.warpAmplitudeMeters;
    style.causticTint = {look.tintRed, look.tintGreen, look.tintBlue};
    style.causticEmissionBoost = look.emissionBoost;
    style.causticOpacityBoost = look.opacityBoost;
    style.causticPointSizeBoost = look.pointSizeBoost;
    style.causticPreviewTintAmount = active ? previewTint.amount : 0.0F;
    style.causticPreviewTintRegionId = active ? previewTint.regionId : 0.0F;
    style.causticMaskFieldSlot = active ? static_cast<std::int32_t>(maskSlot.value()) : -1;
    style.causticEdgeFieldSlot = active ? static_cast<std::int32_t>(edgeSlot.value()) : -1;
    style.causticSeedFieldSlot = active ? static_cast<std::int32_t>(regionOrSeedSlot.value()) : -1;
    return style;
}

std::vector<invisible_places::output::OfflinePointLayer> BuildOfflinePointLayers(
    const std::vector<OfflinePointLayerSnapshot>& snapshots) {
    std::vector<invisible_places::output::OfflinePointLayer> layers;
    layers.reserve(snapshots.size());
    for (const auto& snapshot : snapshots) {
        if (snapshot.cloud == nullptr || snapshot.cloud->positions.empty()) {
            continue;
        }

        invisible_places::output::OfflinePointLayer layer{
            .cloud = snapshot.cloud.get(),
            .style = snapshot.style,
            .hasSourceRgb = snapshot.hasSourceRgb,
            .fastBasic = snapshot.fastBasic,
            .drawPointCount = snapshot.drawPointCount,
            .localToWorld = snapshot.localToWorld};
        if (invisible_places::renderer::pointcloud::PointCloudStyleHasActiveRoughnessMotion(layer.style)) {
            if (const auto roughnessSlot = FindRoughnessMotionScalarFieldSlot(snapshot.cloud->scalarFields);
                roughnessSlot.has_value() && roughnessSlot.value() < snapshot.cloud->scalarFields.size()) {
                const auto& roughnessStats = snapshot.cloud->scalarFields[roughnessSlot.value()];
                layer.roughnessMotionFieldSlot = roughnessSlot.value();
                layer.roughnessMotionMinimum = roughnessStats.minimum;
                layer.roughnessMotionInvRange =
                    1.0F / std::max(1.0e-6F, roughnessStats.maximum - roughnessStats.minimum);
                if (const auto groundSlot = FindGroundIdMotionScalarFieldSlot(snapshot.cloud->scalarFields);
                    groundSlot.has_value() && groundSlot.value() < snapshot.cloud->scalarFields.size()) {
                    layer.groundIdMotionFieldSlot = groundSlot.value();
                }
            }
        }
        layers.push_back(layer);
    }
    return layers;
}

bool HasOfflinePointLayers(const PreviewRuntimeState& runtimeState) {
    return std::any_of(
        runtimeState.sessions.begin(),
        runtimeState.sessions.end(),
        [](const PreviewLayerSession& session) {
            return session.loaded &&
                   session.visible &&
                   session.kind == LayerKind::PointCloud &&
                   session.offlinePointCloud != nullptr &&
                   !session.offlinePointCloud->positions.empty();
        });
}

std::uint64_t EffectiveAnimationExportPointDrawCount(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session,
    bool previewDensity,
    PointCloudRendererMode rendererMode) {
    if (rendererMode == PointCloudRendererMode::FastBasic) {
        return session.pointBudget.activePoints;
    }

    if (!previewDensity) {
        return session.totalPrimitives;
    }

    const auto decision = invisible_places::renderer::pointcloud::ResolvePointCloudPreviewLod(
        session.pointBudget,
        runtimeState.projectSettings.pointCloudPreviewLodMode,
        false,
        true,
        kPointCloudPreviewLodTarget);
    if (!decision.usesPreviewLod) {
        return decision.drawPointCount;
    }

    if (session.previewLodSampledDrawCount == 0) {
        return session.pointBudget.activePoints;
    }

    return std::min<std::uint64_t>(
        session.previewLodSampledDrawCount,
        decision.drawPointCount);
}

struct PointVisualExportOverride {
    std::size_t sessionIndex = 0;
    PointCloudStyleState style{};
};

std::vector<invisible_places::renderer::core::SceneRenderState::PointCloudLayerState>
BuildAnimationExportPointCloudLayerSnapshot(
    const PreviewRuntimeState& runtimeState,
    bool previewDensity,
    const std::optional<PointVisualExportOverride>& visualOverride,
    PointCloudRendererMode rendererMode) {
    std::vector<invisible_places::renderer::core::SceneRenderState::PointCloudLayerState> layers;
    for (std::size_t sessionIndex = 0; sessionIndex < runtimeState.sessions.size(); ++sessionIndex) {
        const auto& session = runtimeState.sessions[sessionIndex];
        if (!session.loaded ||
            !session.visible ||
            session.kind != LayerKind::PointCloud ||
            session.totalPrimitives == 0) {
            continue;
        }

        const auto drawPointCount = EffectiveAnimationExportPointDrawCount(
            runtimeState,
            session,
            previewDensity,
            rendererMode);
        auto exportStyle =
            visualOverride.has_value() && visualOverride->sessionIndex == sessionIndex
                ? visualOverride->style
                : session.pointStyle;
        if (IsGeneratedWaterOverlaySession(session)) {
            exportStyle = MakeWaterTrailExportStyle(exportStyle);
        }
        exportStyle = ApplyWaterCausticRenderStyle(runtimeState, session, exportStyle, false);
        const auto effectiveStyle =
            rendererMode == PointCloudRendererMode::FastBasic
                ? MakeEffectiveFastBasicStyle(
                      exportStyle,
                      session.hasSourceRgb,
                      IsGeneratedWaterOverlaySession(session))
                : exportStyle;
        layers.push_back(
            {.layerId = sessionIndex,
             .style = effectiveStyle,
             .scalarFields = session.scalarFields,
             .hasSourceRgb = session.hasSourceRgb,
             .drawPointCount = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                 drawPointCount,
                 std::numeric_limits<std::uint32_t>::max()))});
    }
    return layers;
}

invisible_places::renderer::core::SceneRenderState BuildPointCloudExrRenderState(
    const OfflineRenderJobState& job,
    const invisible_places::camera::CameraState& cameraState,
    std::uint32_t width,
    std::uint32_t height) {
    invisible_places::renderer::core::SceneRenderState renderState;
    invisible_places::camera::OrbitCamera camera;
    camera.ApplyState(cameraState);
    const float aspectRatio =
        static_cast<float>(std::max<std::uint32_t>(1U, width)) /
        static_cast<float>(std::max<std::uint32_t>(1U, height));
    const auto matrices = camera.Matrices(aspectRatio);

    renderState.view = matrices.view;
    renderState.projection = matrices.projection;
    renderState.viewProjection = matrices.viewProjection;
    renderState.cameraPosition = matrices.position;
    renderState.backgroundColor = job.exportBackgroundColor;
    renderState.pointCloudRendererMode = job.pointCloudRendererMode;
    renderState.eyeDomeLightingEnabled =
        job.exportEyeDomeLightingEnabled && job.pointCloudRendererMode != PointCloudRendererMode::FastBasic;
    const float screenPixelScale = invisible_places::output::ComputePointSizePixelScale(
        width,
        height,
        job.setupViewportWidth,
        job.setupViewportHeight);
    renderState.eyeDomeLightingThickness =
        std::max(0.0F, job.exportEyeDomeLightingThickness) * screenPixelScale;
    renderState.nearPlane = camera.NearPlane();
    renderState.farPlane = camera.FarPlane();
    renderState.hasDepthOfField =
        cameraState.hasDepthOfField && job.pointCloudRendererMode != PointCloudRendererMode::FastBasic;
    renderState.focusDistance = cameraState.focusDistance;
    renderState.apertureFStops = cameraState.apertureFStops;
    renderState.depthOfFieldMaxBlurPixels =
        std::max(0.0F, cameraState.depthOfFieldMaxBlurPixels) * screenPixelScale;
    renderState.gaussianSplatFootprintBoost = job.exportGaussianSplatFootprintBoost;
    renderState.flowTimeSeconds =
        job.pointCloudRendererMode == PointCloudRendererMode::FastBasic
            ? 0.0F
            : static_cast<float>(job.currentFrame) /
                  static_cast<float>(std::max<std::uint32_t>(1U, job.settings.framesPerSecond));
    renderState.pointSizeScale = screenPixelScale;
    renderState.pointCloudLayers = job.exportPointCloudLayers;

    return renderState;
}

std::string FormatElapsedTime(std::chrono::steady_clock::duration elapsed) {
    const auto totalSeconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    const auto minutes = totalSeconds / 60;
    const auto seconds = totalSeconds % 60;
    std::ostringstream output;
    output << minutes << "m " << seconds << "s";
    return output.str();
}

std::uint64_t CurrentResidentMemoryBytes() {
#if defined(__APPLE__)
    mach_task_basic_info_data_t info{};
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    const kern_return_t result = task_info(
        mach_task_self(),
        MACH_TASK_BASIC_INFO,
        reinterpret_cast<task_info_t>(&info),
        &count);
    return result == KERN_SUCCESS ? static_cast<std::uint64_t>(info.resident_size) : 0U;
#elif defined(__linux__)
    std::ifstream statm{"/proc/self/statm"};
    long totalPages = 0;
    long residentPages = 0;
    statm >> totalPages >> residentPages;
    const long pageSize = sysconf(_SC_PAGESIZE);
    if (residentPages <= 0 || pageSize <= 0) {
        return 0U;
    }
    return static_cast<std::uint64_t>(residentPages) * static_cast<std::uint64_t>(pageSize);
#else
    return 0U;
#endif
}

std::string FormatByteCount(std::uint64_t bytes) {
    if (bytes == 0U) {
        return "Unavailable";
    }

    constexpr const char* kUnits[] = {"B", "KiB", "MiB", "GiB", "TiB"};
    double value = static_cast<double>(bytes);
    std::size_t unitIndex = 0;
    while (value >= 1024.0 && unitIndex + 1U < std::size(kUnits)) {
        value /= 1024.0;
        ++unitIndex;
    }

    std::ostringstream output;
    output << std::fixed << std::setprecision(unitIndex == 0U ? 0 : 1) << value << ' ' << kUnits[unitIndex];
    return output.str();
}

std::string FormatDurationForLog(std::chrono::steady_clock::duration duration) {
    const double seconds = std::chrono::duration<double>(duration).count();
    std::ostringstream output;
    output << std::fixed << std::setprecision(seconds < 10.0 ? 3 : 2) << seconds << " s";
    return output.str();
}

std::string FormatLocalTime(
    std::chrono::system_clock::time_point timePoint,
    const char* format) {
    const std::time_t time = std::chrono::system_clock::to_time_t(timePoint);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &time);
#else
    localtime_r(&time, &localTime);
#endif

    std::ostringstream output;
    output << std::put_time(&localTime, format);
    return output.str();
}

bool AnimationExportWritesMp4(invisible_places::output::AnimationExportMode mode) {
    return mode == invisible_places::output::AnimationExportMode::FastPreviewMp4 ||
           mode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4;
}

bool AnimationExportWritesExr(invisible_places::output::AnimationExportMode mode) {
    return mode != invisible_places::output::AnimationExportMode::FastPreviewMp4;
}

bool AnimationExportIsRaycast(invisible_places::output::AnimationExportMode mode) {
    return mode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4;
}

std::uint32_t Mp4SupersampleScaleForSettings(const RenderJobSettings& settings) {
    constexpr std::uint32_t kScale = 2U;
    if (settings.width == 0 || settings.height == 0) {
        return 1U;
    }
    return kScale;
}

std::uint32_t ScaledRenderDimension(std::uint32_t dimension, std::uint32_t scale) {
    const auto safeDimension = std::max<std::uint32_t>(1U, dimension);
    const auto safeScale = std::max<std::uint32_t>(1U, scale);
    if (safeDimension > std::numeric_limits<std::uint32_t>::max() / safeScale) {
        return safeDimension;
    }
    return std::max<std::uint32_t>(1U, safeDimension * safeScale);
}

AnimationExportOutputOptions MakeAnimationExportOutputOptions(
    invisible_places::output::AnimationExportMode mode,
    const RenderJobSettings& settings,
    std::filesystem::path videoOutputPath,
    bool previewMp4Optional = false,
    std::string previewVideoWarning = {}) {
    AnimationExportOutputOptions options;
    options.writeExrStack = AnimationExportWritesExr(mode);
    options.writePreviewMp4 = AnimationExportWritesMp4(mode);
    options.previewMp4Optional = previewMp4Optional;
    options.mp4SupersampleScale = options.writePreviewMp4 ? Mp4SupersampleScaleForSettings(settings) : 1U;
    options.previewVideoPath = std::move(videoOutputPath);
    options.previewVideoWarning = std::move(previewVideoWarning);
    return options;
}

const char* AnimationExportModeLabel(invisible_places::output::AnimationExportMode mode) {
    switch (mode) {
        case invisible_places::output::AnimationExportMode::FastPreviewMp4:
            return "Quick MP4";
        case invisible_places::output::AnimationExportMode::HqPreviewDensityExr:
            return "HQ Preview-Density EXR";
        case invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4:
            return "Beauty Raycast EXR + MP4";
    }

    return "Animation Export";
}

const char* AnimationExportCaptureLabel(invisible_places::output::AnimationExportMode mode) {
    switch (mode) {
        case invisible_places::output::AnimationExportMode::FastPreviewMp4:
            return "MP4";
        case invisible_places::output::AnimationExportMode::HqPreviewDensityExr:
            return "HQ EXR";
        case invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4:
            return "Beauty Raycast";
    }

    return "Animation";
}

const char* AnimationExportOverlayLabel(invisible_places::output::AnimationExportMode mode) {
    switch (mode) {
        case invisible_places::output::AnimationExportMode::FastPreviewMp4:
            return "Encoding Fast Preview MP4";
        case invisible_places::output::AnimationExportMode::HqPreviewDensityExr:
            return "Rendering HQ Preview-Density EXR";
        case invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4:
            return "Rendering Beauty Raycast";
    }

    return "Animation Export";
}

const char* StillCameraExportOverlayLabel(invisible_places::output::AnimationExportMode mode) {
    switch (mode) {
        case invisible_places::output::AnimationExportMode::FastPreviewMp4:
            return "Exporting Still Camera MP4";
        case invisible_places::output::AnimationExportMode::HqPreviewDensityExr:
            return "Exporting Still Camera EXR Stack";
        case invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4:
            return "Exporting Still Camera Raycast";
    }

    return "Still Camera Export";
}

const char* OfflineRenderJobOverlayLabel(const OfflineRenderJobState& job) {
    return job.stillCameraJob ? StillCameraExportOverlayLabel(job.mode) : AnimationExportOverlayLabel(job.mode);
}

const char* ExportLogPrefix(invisible_places::output::AnimationExportMode mode) {
    switch (mode) {
        case invisible_places::output::AnimationExportMode::FastPreviewMp4:
            return "ExportLog_MP4_";
        case invisible_places::output::AnimationExportMode::HqPreviewDensityExr:
            return "ExportLog_EXR_";
        case invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4:
            return "ExportLog_Raycast_";
    }

    return "ExportLog_";
}

const char* ExportLogTypeLabel(invisible_places::output::AnimationExportMode mode) {
    return AnimationExportModeLabel(mode);
}

std::filesystem::path BuildUniqueExportLogPath(
    const std::filesystem::path& outputDirectory,
    invisible_places::output::AnimationExportMode mode,
    std::chrono::system_clock::time_point startedAt) {
    const auto directory = outputDirectory.empty() ? std::filesystem::path{"."} : outputDirectory;
    const auto baseStem = std::string{ExportLogPrefix(mode)} + FormatLocalTime(startedAt, "%y%m%d-%H%M%S");
    auto candidate = directory / (baseStem + ".txt");
    std::error_code existsError;
    if (!std::filesystem::exists(candidate, existsError)) {
        return candidate;
    }

    for (std::uint32_t suffix = 1U; suffix < 10000U; ++suffix) {
        candidate = directory / (baseStem + "_" + std::to_string(suffix) + ".txt");
        existsError.clear();
        if (!std::filesystem::exists(candidate, existsError)) {
            return candidate;
        }
    }
    return candidate;
}

ExportLogState MakeExportLogState(
    const std::filesystem::path& outputDirectory,
    invisible_places::output::AnimationExportMode mode) {
    ExportLogState log;
    log.startedWallTime = std::chrono::system_clock::now();
    log.startedAt = std::chrono::steady_clock::now();
    log.path = BuildUniqueExportLogPath(outputDirectory, mode, log.startedWallTime);
    log.startResidentMemoryBytes = CurrentResidentMemoryBytes();
    log.peakResidentMemoryBytes = log.startResidentMemoryBytes;
    return log;
}

void SampleExportLogMemory(OfflineRenderJobState* job) {
    if (job == nullptr || job->exportLog.path.empty()) {
        return;
    }

    const auto memoryBytes = CurrentResidentMemoryBytes();
    if (memoryBytes > job->exportLog.peakResidentMemoryBytes) {
        job->exportLog.peakResidentMemoryBytes = memoryBytes;
    }
}

void RecordExportGpuCaptureDuration(
    OfflineRenderJobState* job,
    std::chrono::steady_clock::duration duration) {
    if (job == nullptr || job->exportLog.path.empty()) {
        return;
    }

    auto& log = job->exportLog;
    log.gpuCaptureTotal += duration;
    if (log.capturedFrames == 0U || duration < log.gpuCaptureMin) {
        log.gpuCaptureMin = duration;
    }
    if (log.capturedFrames == 0U || duration > log.gpuCaptureMax) {
        log.gpuCaptureMax = duration;
    }
    ++log.capturedFrames;
}

std::uint64_t WrittenOutputByteCount(const OfflineRenderJobState& job) {
    std::uint64_t bytes = 0;
    std::error_code error;
    if (job.writePreviewMp4) {
        if (!job.videoOutputPath.empty() && std::filesystem::exists(job.videoOutputPath, error)) {
            error.clear();
            const auto fileBytes = std::filesystem::file_size(job.videoOutputPath, error);
            if (!error) {
                bytes += fileBytes;
            }
        }
    }

    if (job.writeExrStack) {
        for (std::uint32_t frameIndex = 0; frameIndex < job.writtenFrameCount; ++frameIndex) {
            const auto outputPath =
                invisible_places::output::RenderFramePath(job.settings, job.settings.startFrame + frameIndex);
            error.clear();
            if (!std::filesystem::exists(outputPath, error)) {
                continue;
            }
            error.clear();
            const auto fileBytes = std::filesystem::file_size(outputPath, error);
            if (!error) {
                bytes += fileBytes;
            }
        }
    }
    return bytes;
}

std::string WriteExportLog(
    const OfflineRenderJobState& job,
    const std::string& statusMessage,
    const std::string& errorMessage) {
    if (job.exportLog.path.empty()) {
        return {};
    }

    std::error_code createError;
    const auto parentDirectory = job.exportLog.path.parent_path();
    if (!parentDirectory.empty()) {
        std::filesystem::create_directories(parentDirectory, createError);
        if (createError) {
            return createError.message();
        }
    }

    std::ofstream log{job.exportLog.path};
    if (!log) {
        return "Failed to open " + job.exportLog.path.string();
    }

    const auto finishedWallTime = std::chrono::system_clock::now();
    const auto duration = std::chrono::steady_clock::now() - job.exportLog.startedAt;
    const auto totalDrawPoints = std::accumulate(
        job.exportPointCloudLayers.begin(),
        job.exportPointCloudLayers.end(),
        std::uint64_t{0},
        [](std::uint64_t sum, const auto& layer) {
            return sum + static_cast<std::uint64_t>(layer.drawPointCount);
        });
    const auto readbackBytes =
        static_cast<std::uint64_t>(job.settings.width) *
        static_cast<std::uint64_t>(job.settings.height) * 4U * sizeof(std::uint16_t);
    const auto mp4ReadbackBytes =
        static_cast<std::uint64_t>(ScaledRenderDimension(job.settings.width, job.mp4SupersampleScale)) *
        static_cast<std::uint64_t>(ScaledRenderDimension(job.settings.height, job.mp4SupersampleScale)) *
        4U * sizeof(std::uint16_t);
    const auto mp4RawFrameBytes =
        static_cast<std::uint64_t>(job.settings.width) *
        static_cast<std::uint64_t>(job.settings.height) * 4U;
    const auto outputBytes = WrittenOutputByteCount(job);
    const auto averageCaptureDuration =
        job.exportLog.capturedFrames > 0U
            ? job.exportLog.gpuCaptureTotal / job.exportLog.capturedFrames
            : std::chrono::steady_clock::duration{};

    log << "Invisible Places Export Log\n";
    log << "Type: " << ExportLogTypeLabel(job.mode) << '\n';
    log << "Started: " << FormatLocalTime(job.exportLog.startedWallTime, "%Y-%m-%d %H:%M:%S") << '\n';
    log << "Finished: " << FormatLocalTime(finishedWallTime, "%Y-%m-%d %H:%M:%S") << '\n';
    log << "Duration: " << FormatDurationForLog(duration) << '\n';
    const bool cancelled =
        job.cancelRequested ||
        statusMessage.find("cancelled") != std::string::npos ||
        statusMessage.find("Cancelling") != std::string::npos;
    log << "Status: " << (!errorMessage.empty() ? "Failed" : (cancelled ? "Cancelled" : "Complete")) << '\n';
    if (!statusMessage.empty()) {
        log << "Status message: " << statusMessage << '\n';
    }
    if (!errorMessage.empty()) {
        log << "Error: " << errorMessage << '\n';
    }
    log << '\n';

    log << "Animation\n";
    log << "Name: " << (job.animationName.empty() ? "(unnamed)" : job.animationName) << '\n';
    if (!job.animationFilePath.empty()) {
        log << "File: " << job.animationFilePath.string() << '\n';
    }
    if (!job.exportVisualName.empty()) {
        log << "Visual: " << job.exportVisualName << '\n';
    }
    if (job.quickMp4BatchJob) {
        log << "Batch item: " << job.quickMp4BatchIndex << " / " << job.quickMp4BatchTotal << '\n';
    }
    log << '\n';

    log << "Output\n";
    if (job.writePreviewMp4) {
        log << "MP4: " << job.videoOutputPath.string() << '\n';
    } else if (!job.previewVideoWarning.empty()) {
        log << "MP4: " << job.previewVideoWarning << '\n';
    }
    if (job.writeExrStack) {
        log << "EXR folder: " << job.settings.outputDirectory << '\n';
        if (!job.lastOutputPath.empty()) {
            log << "Last EXR: " << job.lastOutputPath.string() << '\n';
        }
    }
    log << "Written bytes: " << FormatByteCount(outputBytes) << '\n';
    log << '\n';

    log << "Settings\n";
    log << "Resolution: " << job.settings.width << " x " << job.settings.height << '\n';
    log << "FPS: " << job.settings.framesPerSecond << '\n';
    log << "Still camera duration: " << job.settings.stillCameraDurationSeconds << " seconds\n";
    log << "Start frame: " << job.settings.startFrame << '\n';
    log << "End frame setting: " << job.settings.endFrame << '\n';
    log << "Total frames planned: " << job.frames.size() << '\n';
    log << "Frames captured: " << job.currentFrame << '\n';
    log << "Frames written: " << job.writtenFrameCount << '\n';
    log << "Preview density: " << (job.previewDensity ? "yes" : "no") << '\n';
    log << "Point renderer: " << PointCloudRendererModeLabel(job.pointCloudRendererMode) << '\n';
    log << "Export renderer: "
        << (AnimationExportIsRaycast(job.mode) ? "Beauty Raycast" : "Beauty Raster") << '\n';
    if (job.exportEyeDomeLightingEnabled) {
        log << "Eye-Dome Lighting thickness: " << job.exportEyeDomeLightingThickness << " px\n";
    }
    if (AnimationExportIsRaycast(job.mode)) {
        log << "Raycast primitive: " << PointCloudRaycastPrimitiveModeLabel(job.settings.raycastPrimitiveMode) << '\n';
        log << "Raycast samples per pixel: " << job.settings.raycastSamplesPerPixel << '\n';
        if (job.settings.raycastMaxDepth > 0.0F) {
            log << "Raycast max depth: " << job.settings.raycastMaxDepth << '\n';
        } else {
            log << "Raycast max depth: camera far plane\n";
        }
    }
    if (job.writePreviewMp4) {
        log << "MP4 sparse point smoothing: yes\n";
        log << "MP4 supersample scale: " << std::max<std::uint32_t>(1U, job.mp4SupersampleScale) << "x\n";
    }
    log << "Eye-Dome Lighting: " << (job.exportEyeDomeLightingEnabled ? "yes" : "no") << '\n';
    log << "Visible LiDAR export layers: " << job.exportPointCloudLayers.size() << '\n';
    log << "Total export draw points: " << totalDrawPoints << '\n';
    log << "Setup viewport: " << job.setupViewportWidth << " x " << job.setupViewportHeight << '\n';
    log << '\n';

    log << "Timing\n";
    log << "GPU capture total: " << FormatDurationForLog(job.exportLog.gpuCaptureTotal) << '\n';
    log << "GPU capture average: " << FormatDurationForLog(averageCaptureDuration) << '\n';
    log << "GPU capture min: " << FormatDurationForLog(job.exportLog.gpuCaptureMin) << '\n';
    log << "GPU capture max: " << FormatDurationForLog(job.exportLog.gpuCaptureMax) << '\n';
    log << "Peak writer queue frames: " << job.exportLog.peakQueuedFrames << '\n';
    log << '\n';

    log << "Memory\n";
    log << "Process resident at start: " << FormatByteCount(job.exportLog.startResidentMemoryBytes) << '\n';
    log << "Process resident at finish: " << FormatByteCount(job.exportLog.endResidentMemoryBytes) << '\n';
    log << "Peak sampled process resident: " << FormatByteCount(job.exportLog.peakResidentMemoryBytes) << '\n';
    log << "GPU readback frame buffer: " << FormatByteCount(readbackBytes) << '\n';
    if (job.writePreviewMp4) {
        log << "MP4 supersampled GPU readback frame buffer: " << FormatByteCount(mp4ReadbackBytes) << '\n';
        log << "MP4 raw RGBA frame bytes: " << FormatByteCount(mp4RawFrameBytes) << '\n';
    }

    return {};
}

void UpdateOfflineRenderProgress(
    const std::shared_ptr<OfflineRenderProgressState>& progress,
    std::uint32_t frame,
    std::uint32_t tile,
    const std::filesystem::path& lastOutputPath = {},
    const invisible_places::output::OfflinePointRenderDiagnostics* diagnostics = nullptr) {
    if (progress == nullptr) {
        return;
    }

    std::scoped_lock lock(progress->mutex);
    progress->currentFrame = frame;
    progress->currentTile = tile;
    if (!lastOutputPath.empty()) {
        progress->lastOutputPath = lastOutputPath;
    }
    if (diagnostics != nullptr) {
        progress->lastDiagnostics = *diagnostics;
    }
}

bool OfflineRenderCancellationRequested(
    const std::shared_ptr<OfflineRenderProgressState>& progress,
    const std::stop_token& stopToken) {
    if (stopToken.stop_requested()) {
        return true;
    }
    if (progress == nullptr) {
        return false;
    }

    std::scoped_lock lock(progress->mutex);
    return progress->cancelRequested;
}

void CompleteOfflineRenderProgress(
    const std::shared_ptr<OfflineRenderProgressState>& progress,
    const std::string& statusMessage,
    const std::string& errorMessage = {}) {
    if (progress == nullptr) {
        return;
    }

    std::scoped_lock lock(progress->mutex);
    progress->completed = true;
    progress->statusMessage = statusMessage;
    progress->errorMessage = errorMessage;
}

void RunOfflineRenderWorker(
    std::stop_token stopToken,
    RenderJobSettings settings,
    std::vector<invisible_places::camera::CameraState> frames,
    std::vector<invisible_places::output::OfflineRenderTile> tiles,
    std::vector<OfflinePointLayerSnapshot> layerSnapshots,
    std::shared_ptr<OfflineRenderProgressState> progress) {
    try {
        const auto offlineLayers = BuildOfflinePointLayers(layerSnapshots);
        if (offlineLayers.empty()) {
            CompleteOfflineRenderProgress(progress, {}, "No visible loaded LiDAR layers are available for rendering.");
            return;
        }

        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, settings.width, settings.height);
        invisible_places::output::OfflinePointRenderScratch scratch;

        for (std::uint32_t frameIndex = 0; frameIndex < frames.size(); ++frameIndex) {
            for (std::uint32_t tileIndex = 0; tileIndex < tiles.size(); ++tileIndex) {
                if (OfflineRenderCancellationRequested(progress, stopToken)) {
                    CompleteOfflineRenderProgress(progress, "EXR render cancelled.");
                    return;
                }

                UpdateOfflineRenderProgress(progress, frameIndex, tileIndex);
                invisible_places::output::OfflinePointRenderDiagnostics diagnostics;
                const float stylisationTimeSeconds =
                    static_cast<float>(frameIndex) / static_cast<float>(std::max<std::uint32_t>(1U, settings.framesPerSecond));
                invisible_places::output::RenderPointCloudTile(
                    offlineLayers,
                    frames[frameIndex],
                    tiles[tileIndex],
                    &image,
                    &diagnostics,
                    &scratch,
                    stylisationTimeSeconds);
                UpdateOfflineRenderProgress(progress, frameIndex, tileIndex + 1U, {}, &diagnostics);
            }

            const auto outputFrameIndex = settings.startFrame + frameIndex;
            const auto outputPath = invisible_places::output::RenderFramePath(settings, outputFrameIndex);
            std::string errorMessage;
            if (!invisible_places::output::WriteExrImage(image, outputPath, &errorMessage)) {
                CompleteOfflineRenderProgress(progress, {}, errorMessage);
                return;
            }
            UpdateOfflineRenderProgress(progress, frameIndex + 1U, 0U, outputPath);

            if (OfflineRenderCancellationRequested(progress, stopToken)) {
                CompleteOfflineRenderProgress(progress, "EXR render cancelled after " + outputPath.string() + ".");
                return;
            }

            if (frameIndex + 1U < frames.size()) {
                invisible_places::output::InitializeExrImage(&image, settings.width, settings.height);
            }
        }
    } catch (const std::exception& error) {
        CompleteOfflineRenderProgress(progress, {}, "EXR render failed: " + std::string{error.what()});
        return;
    }

    CompleteOfflineRenderProgress(progress, "EXR stack complete: " + settings.outputDirectory + ".");
}

void CompleteAnimationExportWriter(
    const std::shared_ptr<AnimationExportWriterState>& writerState,
    const std::string& statusMessage,
    const std::string& errorMessage = {}) {
    if (writerState == nullptr) {
        return;
    }

    {
        std::scoped_lock lock(writerState->mutex);
        writerState->acceptingFrames = false;
        writerState->completed = true;
        writerState->statusMessage = statusMessage;
        writerState->errorMessage = errorMessage;
    }
    writerState->condition.notify_all();
}

void RequestAnimationExportWriterCancellation(OfflineRenderJobState* job) {
    if (job == nullptr || job->writerState == nullptr) {
        return;
    }

    {
        std::scoped_lock lock(job->writerState->mutex);
        job->writerState->cancelRequested = true;
        job->writerState->acceptingFrames = false;
    }
    job->writerState->condition.notify_all();
    if (job->worker.joinable()) {
        job->worker.request_stop();
    }
}

void SignalAnimationExportWriterFinish(OfflineRenderJobState* job) {
    if (job == nullptr || job->writerState == nullptr || job->writerFinishRequested) {
        return;
    }

    {
        std::scoped_lock lock(job->writerState->mutex);
        job->writerState->finishRequested = true;
        job->writerState->acceptingFrames = false;
    }
    job->writerFinishRequested = true;
    job->writerState->condition.notify_all();
}

bool AnimationExportWriterCanAcceptFrame(const OfflineRenderJobState& job) {
    constexpr std::size_t kMaxQueuedExportFrames = 2U;
    if (job.writerState == nullptr) {
        return false;
    }

    std::scoped_lock lock(job.writerState->mutex);
    return job.writerState->acceptingFrames &&
           !job.writerState->cancelRequested &&
           !job.writerState->completed &&
           job.writerState->errorMessage.empty() &&
           job.writerState->pendingFrames.size() < kMaxQueuedExportFrames;
}

bool QueueAnimationExportFrame(
    OfflineRenderJobState* job,
    std::uint32_t outputFrameIndex,
    invisible_places::output::HalfRgbaExrImage image,
    invisible_places::output::HalfRgbaExrImage previewImage = {}) {
    if (job == nullptr || job->writerState == nullptr) {
        return false;
    }

    {
        std::scoped_lock lock(job->writerState->mutex);
        if (!job->writerState->acceptingFrames ||
            job->writerState->cancelRequested ||
            job->writerState->completed ||
            !job->writerState->errorMessage.empty()) {
            return false;
        }

        job->writerState->pendingFrames.push_back(
            {.outputFrameIndex = outputFrameIndex,
             .image = std::move(image),
             .previewImage = std::move(previewImage)});
    }
    job->writerState->condition.notify_one();
    return true;
}

void RefreshAnimationExportWriterProgress(OfflineRenderJobState* job) {
    if (job == nullptr || job->writerState == nullptr) {
        return;
    }

    std::scoped_lock lock(job->writerState->mutex);
    job->writtenFrameCount = job->writerState->writtenFrames;
    job->pendingFrameCount = job->writerState->pendingFrames.size();
    job->exportLog.peakQueuedFrames = std::max(job->exportLog.peakQueuedFrames, job->pendingFrameCount);
    if (!job->writerState->lastOutputPath.empty()) {
        job->lastOutputPath = job->writerState->lastOutputPath;
    }
    SampleExportLogMemory(job);
}

std::string CloseAnimationExportVideoPipe(FILE** videoPipe, const std::filesystem::path& outputPath) {
    if (videoPipe == nullptr || *videoPipe == nullptr) {
        return {};
    }

    const int closeStatus = ::pclose(*videoPipe);
    *videoPipe = nullptr;
    if (closeStatus != 0) {
        return "ffmpeg failed while writing " + outputPath.string() +
               " (status " + std::to_string(closeStatus) + ").";
    }
    return {};
}

void RunAnimationExportWriter(
    std::stop_token stopToken,
    invisible_places::output::AnimationExportMode mode,
    RenderJobSettings settings,
    AnimationExportOutputOptions outputOptions,
    std::uint32_t totalFrames,
    std::shared_ptr<AnimationExportWriterState> writerState) {
    FILE* videoPipe = nullptr;
    std::string previewVideoWarning = std::move(outputOptions.previewVideoWarning);
    auto appendPreviewVideoWarning = [&](const std::string& warning) {
        if (warning.empty()) {
            return;
        }
        if (!previewVideoWarning.empty()) {
            previewVideoWarning += " ";
        }
        previewVideoWarning += warning;
    };
    auto closePreviewPipe = [&]() {
        const auto closeError =
            CloseAnimationExportVideoPipe(&videoPipe, outputOptions.previewVideoPath);
        return closeError;
    };
    try {
        bool writesMp4 = outputOptions.writePreviewMp4 && !outputOptions.previewVideoPath.empty();
        const bool writesExr = outputOptions.writeExrStack;
        if (writesMp4) {
            const auto command = invisible_places::output::BuildFfmpegRawRgbaCommand(
                invisible_places::output::DefaultFfmpegExecutablePath(),
                settings.width,
                settings.height,
                settings.framesPerSecond,
                outputOptions.previewVideoPath);
            videoPipe = ::popen(command.c_str(), "w");
            if (videoPipe == nullptr) {
                if (outputOptions.previewMp4Optional) {
                    writesMp4 = false;
                    appendPreviewVideoWarning("Preview MP4 skipped: failed to start ffmpeg.");
                } else {
                    CompleteAnimationExportWriter(
                        writerState,
                        {},
                        "Failed to start ffmpeg for " + std::string{AnimationExportModeLabel(mode)} + " export.");
                    return;
                }
            }
        }

        if (!writesExr && !writesMp4) {
            if (outputOptions.previewMp4Optional && !previewVideoWarning.empty()) {
                CompleteAnimationExportWriter(
                    writerState,
                    previewVideoWarning,
                    {});
                return;
            }
            CompleteAnimationExportWriter(writerState, {}, "Animation export has no enabled outputs.");
            return;
        }

        auto disableOptionalPreviewVideo = [&](const std::string& warning) {
            if (!writesMp4) {
                return true;
            }
            if (!outputOptions.previewMp4Optional) {
                return false;
            }
            appendPreviewVideoWarning(warning);
            const auto closeError = closePreviewPipe();
            if (!closeError.empty()) {
                appendPreviewVideoWarning(closeError);
            }
            writesMp4 = false;
            return true;
        };

        auto makeCompletionStatus = [&]() {
            std::string status;
            if (writesExr && writesMp4) {
                status = mode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4
                             ? "Beauty Raycast EXR + MP4 complete: " + settings.outputDirectory +
                                   " and " + outputOptions.previewVideoPath.string() + "."
                             : "EXR stack + preview MP4 complete: " + settings.outputDirectory +
                                   " and " + outputOptions.previewVideoPath.string() + ".";
            } else if (writesExr) {
                status = "EXR stack complete: " + settings.outputDirectory + ".";
            } else {
                status = "Fast Preview MP4 complete: " + outputOptions.previewVideoPath.string() + ".";
            }
            if (!previewVideoWarning.empty()) {
                status += " " + previewVideoWarning;
            }
            return status;
        };

        while (true) {
            AnimationExportFramePayload payload;
            bool hasPayload = false;
            {
                std::unique_lock lock(writerState->mutex);
                writerState->condition.wait(lock, [&]() {
                    return stopToken.stop_requested() ||
                           writerState->cancelRequested ||
                           !writerState->pendingFrames.empty() ||
                           writerState->finishRequested;
                });

                if (stopToken.stop_requested() || writerState->cancelRequested) {
                    writerState->pendingFrames.clear();
                    lock.unlock();
                    const auto closeError = closePreviewPipe();
                    CompleteAnimationExportWriter(
                        writerState,
                        closeError.empty() ? "Animation export cancelled." : std::string{},
                        closeError);
                    return;
                }

                if (!writerState->pendingFrames.empty()) {
                    payload = std::move(writerState->pendingFrames.front());
                    writerState->pendingFrames.pop_front();
                    hasPayload = true;
                } else if (writerState->finishRequested) {
                    break;
                }
            }

            if (!hasPayload) {
                continue;
            }

            std::filesystem::path outputPath;
            if (writesExr) {
                outputPath = invisible_places::output::RenderFramePath(settings, payload.outputFrameIndex);
                std::string errorMessage;
                if (!invisible_places::output::WriteExrImage(payload.image, outputPath, &errorMessage)) {
                    CompleteAnimationExportWriter(writerState, {}, errorMessage);
                    return;
                }
            }

            if (writesMp4) {
                const auto& mp4Image =
                    payload.previewImage.rgbaHalf.empty() ? payload.image : payload.previewImage;
                const auto frameBytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(
                    mp4Image,
                    settings.width,
                    settings.height);
                if (frameBytes.empty()) {
                    if (disableOptionalPreviewVideo("Preview MP4 skipped: GPU readback did not produce a valid frame.")) {
                        if (outputPath.empty()) {
                            outputPath = outputOptions.previewVideoPath;
                        }
                    } else {
                        CompleteAnimationExportWriter(
                            writerState,
                            {},
                            "GPU readback did not produce a valid MP4 frame.");
                        return;
                    }
                }
                if (writesMp4) {
                    const auto written = std::fwrite(frameBytes.data(), 1U, frameBytes.size(), videoPipe);
                    if (written != frameBytes.size()) {
                        if (disableOptionalPreviewVideo("Preview MP4 skipped: failed to write raw frame data to ffmpeg.")) {
                            if (outputPath.empty()) {
                                outputPath = outputOptions.previewVideoPath;
                            }
                        } else {
                            CompleteAnimationExportWriter(
                                writerState,
                                {},
                                "Failed to write raw frame data to ffmpeg.");
                            return;
                        }
                    }
                }
                if (outputPath.empty()) {
                    outputPath = outputOptions.previewVideoPath;
                }
            }

            {
                std::scoped_lock lock(writerState->mutex);
                ++writerState->writtenFrames;
                writerState->lastOutputPath = outputPath;
                writerState->statusMessage =
                    "Saved frame " + std::to_string(writerState->writtenFrames) +
                    " / " + std::to_string(totalFrames) + ".";
            }
        }

        const auto closeError = closePreviewPipe();
        if (!closeError.empty()) {
            if (outputOptions.previewMp4Optional) {
                appendPreviewVideoWarning(closeError);
            } else {
                CompleteAnimationExportWriter(writerState, {}, closeError);
                return;
            }
        }

        CompleteAnimationExportWriter(writerState, makeCompletionStatus());
    } catch (const std::exception& error) {
        const auto closeError = closePreviewPipe();
        CompleteAnimationExportWriter(
            writerState,
            {},
            closeError.empty()
                ? "Animation export writer failed: " + std::string{error.what()}
                : closeError);
    }
}

void NormalizeAnimationRenderSettings(RenderJobSettings* settings) {
    if (settings == nullptr) {
        return;
    }

    settings->width = std::max<std::uint32_t>(1U, settings->width);
    settings->height = std::max<std::uint32_t>(1U, settings->height);
    settings->framesPerSecond = std::max<std::uint32_t>(1U, settings->framesPerSecond);
    settings->stillCameraDurationSeconds = std::clamp(settings->stillCameraDurationSeconds, 0.001F, 3600.0F);
    settings->tileSize = std::max<std::uint32_t>(1U, settings->tileSize);
    settings->raycastSamplesPerPixel = std::clamp(settings->raycastSamplesPerPixel, 1U, 16U);
    settings->raycastMaxDepth = std::max(0.0F, settings->raycastMaxDepth);
    if (settings->outputDirectory.empty()) {
        settings->outputDirectory = DefaultRenderOutputDirectory(std::filesystem::path{}).string();
    }
}

bool StartQuickMp4ExportJob(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    QueuedQuickMp4Export request) {
    if (runtimeState == nullptr || runtimeState->offlineRenderJob.active) {
        return false;
    }

    auto settings = request.settings;
    NormalizeAnimationRenderSettings(&settings);

    if (!HasOfflinePointLayers(*runtimeState)) {
        runtimeState->errorMessage = "Load and show at least one LiDAR layer before exporting an animation.";
        runtimeState->statusMessage.clear();
        return false;
    }

    if (request.animationPath.keys.size() < 2U) {
        runtimeState->errorMessage = "Animation " + request.animationPath.name +
                                     " has fewer than two keys and cannot be exported.";
        runtimeState->statusMessage.clear();
        return false;
    }

    if (request.visualSessionIndex >= runtimeState->sessions.size() ||
        !IsVisibleLoadedPointCloudSession(runtimeState->sessions[request.visualSessionIndex])) {
        runtimeState->errorMessage = "The LiDAR layer used for the selected export visual is no longer visible.";
        runtimeState->statusMessage.clear();
        return false;
    }

    const auto ffmpegPath = invisible_places::output::DefaultFfmpegExecutablePath();
    if (!invisible_places::output::FfmpegExecutableAvailable(ffmpegPath)) {
        runtimeState->errorMessage =
            "Fast MP4 export requires ffmpeg at " + ffmpegPath.string() + ".";
        runtimeState->statusMessage.clear();
        return false;
    }

    std::error_code createError;
    if (!request.videoOutputPath.parent_path().empty()) {
        std::filesystem::create_directories(request.videoOutputPath.parent_path(), createError);
        if (createError) {
            runtimeState->errorMessage = "Failed to create MP4 output directory: " + createError.message();
            runtimeState->statusMessage.clear();
            return false;
        }
    }

    auto frames = invisible_places::output::BuildAnimationRenderSequence(request.animationPath, settings);
    if (frames.empty()) {
        runtimeState->errorMessage = "Animation " + request.animationPath.name +
                                     " did not produce any output frames.";
        runtimeState->statusMessage.clear();
        return false;
    }

    auto visualStyle = request.visualStyle;
    if (IsGeneratedWaterOverlaySession(runtimeState->sessions[request.visualSessionIndex]) &&
        !visualStyle.flowAnimation) {
        visualStyle = MakeWaterTrailExportStyle(visualStyle);
    }

    const PointVisualExportOverride visualOverride{
        .sessionIndex = request.visualSessionIndex,
        .style = visualStyle,
    };
    auto exportPointCloudLayers = BuildAnimationExportPointCloudLayerSnapshot(
        *runtimeState,
        false,
        visualOverride,
        runtimeState->projectSettings.pointCloudRendererMode);
    if (exportPointCloudLayers.empty()) {
        runtimeState->errorMessage = "No visible loaded LiDAR layers are available for animation export.";
        runtimeState->statusMessage.clear();
        return false;
    }

    const auto setupSize = viewport != nullptr ? CurrentFramebufferViewportSize(*viewport) : ImVec2{1.0F, 1.0F};
    auto writerState = std::make_shared<AnimationExportWriterState>();
    const auto outputOptions = MakeAnimationExportOutputOptions(
        invisible_places::output::AnimationExportMode::FastPreviewMp4,
        settings,
        request.videoOutputPath);
    runtimeState->offlineRenderJob = {
        .active = true,
        .cancelRequested = false,
        .mode = invisible_places::output::AnimationExportMode::FastPreviewMp4,
        .settings = settings,
        .frames = std::move(frames),
        .tiles = {},
        .currentFrame = 0,
        .currentTile = 0,
        .startedAt = std::chrono::steady_clock::now(),
        .videoOutputPath = request.videoOutputPath,
        .writeExrStack = outputOptions.writeExrStack,
        .writePreviewMp4 = outputOptions.writePreviewMp4,
        .optionalPreviewMp4 = outputOptions.previewMp4Optional,
        .mp4SupersampleScale = outputOptions.mp4SupersampleScale,
        .previewVideoWarning = outputOptions.previewVideoWarning,
        .setupViewportWidth = static_cast<std::uint32_t>(std::max(1.0F, setupSize.x)),
        .setupViewportHeight = static_cast<std::uint32_t>(std::max(1.0F, setupSize.y)),
        .previewDensity = false,
        .pointCloudRendererMode = runtimeState->projectSettings.pointCloudRendererMode,
        .quickMp4BatchJob = true,
        .quickMp4BatchIndex = runtimeState->animationPanel.quickMp4QueueCompleted + 1U,
        .quickMp4BatchTotal = runtimeState->animationPanel.quickMp4QueueTotal,
        .animationName = request.animationPath.name,
        .animationFilePath = request.animationFilePath,
        .exportVisualName = request.visualName,
        .exportLog = MakeExportLogState(
            request.videoOutputPath.parent_path(),
            invisible_places::output::AnimationExportMode::FastPreviewMp4),
        .exportBackgroundColor = glm::vec4{
            runtimeState->projectSettings.backgroundColor[0],
            runtimeState->projectSettings.backgroundColor[1],
            runtimeState->projectSettings.backgroundColor[2],
            0.0F,
        },
        .exportEyeDomeLightingEnabled = runtimeState->projectSettings.eyeDomeLightingEnabled,
        .exportEyeDomeLightingThickness = runtimeState->projectSettings.eyeDomeLightingThickness,
        .exportGaussianSplatFootprintBoost = runtimeState->projectSettings.gaussianSplatFootprintBoost,
        .exportPointCloudLayers = std::move(exportPointCloudLayers),
        .writerState = writerState,
    };
    runtimeState->offlineRenderJob.worker = std::jthread{
        RunAnimationExportWriter,
        runtimeState->offlineRenderJob.mode,
        runtimeState->offlineRenderJob.settings,
        outputOptions,
        static_cast<std::uint32_t>(runtimeState->offlineRenderJob.frames.size()),
        writerState};
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPlayback.active = false;
    runtimeState->statusMessage =
        "Encoding Quick MP4 " +
        std::to_string(runtimeState->animationPanel.quickMp4QueueCompleted + 1U) +
        " / " + std::to_string(runtimeState->animationPanel.quickMp4QueueTotal) +
        ": " + request.animationPath.name + " / " + request.visualName + ".";
    runtimeState->errorMessage.clear();
    return true;
}

bool StartNextQueuedQuickMp4Export(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || runtimeState->offlineRenderJob.active) {
        return false;
    }

    auto& panel = runtimeState->animationPanel;
    if (panel.quickMp4Queue.empty()) {
        if (panel.quickMp4QueueTotal > 0) {
            runtimeState->statusMessage =
                "Quick MP4 batch complete: " +
                std::to_string(panel.quickMp4QueueCompleted) + " exported";
            if (panel.quickMp4QueueSkipped > 0) {
                runtimeState->statusMessage +=
                    ", " + std::to_string(panel.quickMp4QueueSkipped) + " skipped";
            }
            runtimeState->statusMessage += ".";
            runtimeState->errorMessage.clear();
        }
        return false;
    }

    auto request = std::move(panel.quickMp4Queue.front());
    panel.quickMp4Queue.pop_front();
    if (!StartQuickMp4ExportJob(runtimeState, viewport, std::move(request))) {
        panel.quickMp4Queue.clear();
        panel.quickMp4QueueTotal = 0;
        panel.quickMp4QueueCompleted = 0;
        panel.quickMp4QueueSkipped = 0;
        return false;
    }
    return true;
}

void StartSelectedQuickMp4Batch(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || runtimeState->offlineRenderJob.active) {
        return;
    }

    auto& panel = runtimeState->animationPanel;
    panel.quickMp4Queue.clear();
    panel.quickMp4QueueTotal = 0;
    panel.quickMp4QueueCompleted = 0;
    panel.quickMp4QueueSkipped = 0;

    std::vector<std::filesystem::path> selectedFiles;
    selectedFiles.reserve(panel.availableFiles.size());
    for (const auto& filePath : panel.availableFiles) {
        if (AnimationFileSelectedForExport(panel, filePath)) {
            selectedFiles.push_back(filePath);
        }
    }

    if (selectedFiles.empty()) {
        runtimeState->errorMessage = "Select at least one saved animation for Quick MP4 export.";
        runtimeState->statusMessage.clear();
        return;
    }

    const auto visualSessionIndex = ResolveVisiblePointCloudLookdevIndex(*runtimeState);
    if (!visualSessionIndex.has_value()) {
        runtimeState->errorMessage = "Load and show a LiDAR layer with saved visuals before exporting Quick MP4.";
        runtimeState->statusMessage.clear();
        return;
    }

    auto& visualSession = runtimeState->sessions[visualSessionIndex.value()];
    EnsurePointVisuals(&visualSession);
    if (!HasOfflinePointLayers(*runtimeState)) {
        runtimeState->errorMessage = "Load and show at least one LiDAR layer before exporting an animation.";
        runtimeState->statusMessage.clear();
        return;
    }

    const auto ffmpegPath = invisible_places::output::DefaultFfmpegExecutablePath();
    if (!invisible_places::output::FfmpegExecutableAvailable(ffmpegPath)) {
        runtimeState->errorMessage =
            "Fast MP4 export requires ffmpeg at " + ffmpegPath.string() + ".";
        runtimeState->statusMessage.clear();
        return;
    }

    std::vector<std::filesystem::path> reservedOutputPaths;
    std::string loadErrorMessage;
    for (const auto& animationFilePath : selectedFiles) {
        std::optional<AnimationPath> loadedAnimation;
        if (panel.currentPath.has_value() &&
            !panel.currentFilePath.empty() &&
            PathsLexicallyEqual(std::filesystem::path{panel.currentFilePath}, animationFilePath)) {
            loadedAnimation = panel.currentPath.value();
        } else {
            loadedAnimation =
                invisible_places::serialization::LoadAnimationPath(animationFilePath, &loadErrorMessage);
        }

        if (!loadedAnimation.has_value()) {
            ++panel.quickMp4QueueSkipped;
            continue;
        }

        auto animationPath = loadedAnimation.value();
        RemoveUnexportableVisualNames(&animationPath);
        if (animationPath.keys.size() < 2U || animationPath.exportVisualNames.empty()) {
            ++panel.quickMp4QueueSkipped;
            continue;
        }

        auto settings = RenderSettingsFromAnimationExportSettings(animationPath.exportSettings);
        NormalizeAnimationRenderSettings(&settings);
        for (const auto& visualName : animationPath.exportVisualNames) {
            const auto* visual = FindExportablePointVisual(visualSession, visualName);
            if (visual == nullptr) {
                ++panel.quickMp4QueueSkipped;
                continue;
            }

            const auto outputPath = invisible_places::output::BuildUniqueQuickMp4OutputPath(
                settings.outputDirectory,
                animationPath.name,
                visual->name,
                reservedOutputPaths);
            reservedOutputPaths.push_back(outputPath);
            panel.quickMp4Queue.push_back(
                {.animationPath = animationPath,
                 .settings = settings,
                 .animationFilePath = animationFilePath,
                 .visualName = visual->name,
                 .visualSessionIndex = visualSessionIndex.value(),
                 .visualStyle = visual->style,
                 .videoOutputPath = outputPath});
        }
    }

    panel.quickMp4QueueTotal = panel.quickMp4Queue.size();
    if (panel.quickMp4Queue.empty()) {
        runtimeState->statusMessage =
            "No Quick MP4 exports were queued. Select at least one saved visual on each animation.";
        if (panel.quickMp4QueueSkipped > 0) {
            runtimeState->statusMessage +=
                " Skipped " + std::to_string(panel.quickMp4QueueSkipped) + " animation/visual entries.";
        }
        runtimeState->errorMessage.clear();
        return;
    }

    if (!StartNextQueuedQuickMp4Export(runtimeState, viewport)) {
        panel.quickMp4Queue.clear();
        panel.quickMp4QueueTotal = 0;
    }
}

void StartStillCameraExportCapture(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || !runtimeState->offlineRenderJob.active) {
        return;
    }

    auto& job = runtimeState->offlineRenderJob;
    if (job.worker.joinable() && job.preparingExport) {
        job.worker = std::jthread{};
    }

    auto exportPointCloudLayers = BuildAnimationExportPointCloudLayerSnapshot(
        *runtimeState,
        job.previewDensity,
        std::nullopt,
        job.pointCloudRendererMode);
    if (exportPointCloudLayers.empty()) {
        FinishOfflineRenderJob(runtimeState, {}, "No visible loaded LiDAR layers are available for still-camera export.");
        return;
    }

    job.exportPointCloudLayers = std::move(exportPointCloudLayers);
    job.preparingExport = false;
    job.preparationState.reset();
    auto writerState = std::make_shared<AnimationExportWriterState>();
    job.writerState = writerState;
    const AnimationExportOutputOptions outputOptions{
        .writeExrStack = job.writeExrStack,
        .writePreviewMp4 = job.writePreviewMp4,
        .previewMp4Optional = job.optionalPreviewMp4,
        .mp4SupersampleScale = job.mp4SupersampleScale,
        .previewVideoPath = job.videoOutputPath,
        .previewVideoWarning = job.previewVideoWarning,
    };
    job.worker = std::jthread{
        RunAnimationExportWriter,
        job.mode,
        job.settings,
        outputOptions,
        static_cast<std::uint32_t>(job.frames.size()),
        writerState};
    runtimeState->statusMessage =
        "Rendering still camera for " +
        FormatFixed(job.settings.stillCameraDurationSeconds, 2) +
        " seconds...";
    runtimeState->errorMessage.clear();
}

void ProcessStillCameraPreparationStep(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr || !runtimeState->offlineRenderJob.active) {
        return;
    }

    auto& job = runtimeState->offlineRenderJob;
    auto preparationState = job.preparationState;
    if (preparationState == nullptr) {
        StartStillCameraExportCapture(runtimeState, viewport);
        return;
    }

    if (job.cancelRequested) {
        {
            std::scoped_lock lock(preparationState->mutex);
            preparationState->cancelRequested = true;
        }
        if (job.worker.joinable()) {
            job.worker.request_stop();
        }
        runtimeState->statusMessage = "Cancelling still-camera export preparation...";
    }

    bool completed = false;
    bool cancelled = false;
    std::size_t completedRequests = 0;
    std::size_t totalRequests = 0;
    std::string currentLayerName;
    std::string errorMessage;
    std::vector<StillCameraPreviewLodPreparationResult> results;
    {
        std::scoped_lock lock(preparationState->mutex);
        completed = preparationState->completed;
        cancelled = preparationState->cancelled;
        completedRequests = preparationState->completedRequests;
        totalRequests = preparationState->totalRequests;
        currentLayerName = preparationState->currentLayerName;
        errorMessage = preparationState->errorMessage;
        if (completed) {
            results = std::move(preparationState->results);
        }
    }

    if (!completed) {
        runtimeState->statusMessage =
            "Preparing still-camera EXR samples " +
            std::to_string(std::min(completedRequests + 1U, totalRequests)) +
            " / " + std::to_string(std::max<std::size_t>(1U, totalRequests)) +
            (currentLayerName.empty() ? std::string{} : ": " + currentLayerName) + ".";
        return;
    }

    if (!errorMessage.empty()) {
        FinishOfflineRenderJob(runtimeState, {}, errorMessage);
        return;
    }
    if (cancelled || job.cancelRequested) {
        FinishOfflineRenderJob(runtimeState, "Still-camera export cancelled.");
        return;
    }

    for (auto& result : results) {
        if (result.sessionIndex >= runtimeState->sessions.size()) {
            continue;
        }
        auto& session = runtimeState->sessions[result.sessionIndex];
        if (!session.loaded || session.kind != LayerKind::PointCloud) {
            continue;
        }
        session.previewLodRequestedDrawCount = result.requestedCount;
        session.previewLodSampledDrawCount = static_cast<std::uint32_t>(result.sampledIndices.size());
        session.previewLodSampledIndices = std::move(result.sampledIndices);
        viewport->UpdateInteractivePointSampleBuffer(result.sessionIndex, session.previewLodSampledIndices);
    }

    StartStillCameraExportCapture(runtimeState, viewport);
}

void StartStillCameraExportJob(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || runtimeState->offlineRenderJob.active) {
        return;
    }

    auto& settings = runtimeState->renderSettings;
    NormalizeAnimationRenderSettings(&settings);

    if (!HasOfflinePointLayers(*runtimeState)) {
        runtimeState->errorMessage = "Load and show at least one LiDAR layer before exporting a still-camera render.";
        runtimeState->statusMessage.clear();
        return;
    }

    const auto mode = runtimeState->cameraPanel.stillExportMode;
    const bool raycastExport =
        mode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4;
    if (raycastExport && viewport != nullptr && !viewport->SupportsRaycastExport()) {
        runtimeState->errorMessage = "Beauty Raycast export requires Vulkan compute support on the active GPU.";
        runtimeState->statusMessage.clear();
        return;
    }

    std::filesystem::path videoOutputPath;
    AnimationExportOutputOptions outputOptions = MakeAnimationExportOutputOptions(mode, settings, videoOutputPath);
    const bool exrStackPreviewMp4 =
        mode == invisible_places::output::AnimationExportMode::HqPreviewDensityExr;
    if (AnimationExportWritesMp4(mode) || exrStackPreviewMp4) {
        const auto ffmpegPath = invisible_places::output::DefaultFfmpegExecutablePath();
        const bool ffmpegAvailable = invisible_places::output::FfmpegExecutableAvailable(ffmpegPath);
        if (!ffmpegAvailable && !exrStackPreviewMp4) {
            runtimeState->errorMessage =
                std::string{AnimationExportModeLabel(mode)} +
                " still-camera export requires ffmpeg at " +
                ffmpegPath.string() + ".";
            runtimeState->statusMessage.clear();
            return;
        }
        if (ffmpegAvailable) {
            videoOutputPath = invisible_places::output::BuildUniqueQuickMp4OutputPath(
                settings.outputDirectory,
                "StillCamera",
                exrStackPreviewMp4 ? "EXRStackPreview" : (raycastExport ? "BeautyRaycast" : "CurrentView"));
        }
        outputOptions = MakeAnimationExportOutputOptions(mode, settings, videoOutputPath, exrStackPreviewMp4);
        if (exrStackPreviewMp4) {
            outputOptions.writeExrStack = true;
            outputOptions.writePreviewMp4 = ffmpegAvailable;
            outputOptions.previewMp4Optional = true;
            outputOptions.mp4SupersampleScale =
                ffmpegAvailable ? Mp4SupersampleScaleForSettings(settings) : 1U;
            if (!ffmpegAvailable) {
                outputOptions.previewVideoWarning =
                    "Preview MP4 skipped: ffmpeg not found at " + ffmpegPath.string() + ".";
            }
        }
    }

    const bool exportUsesPreviewDensity = false;

    auto frames = BuildStillCameraRenderSequence(*runtimeState, settings);
    if (frames.empty()) {
        runtimeState->errorMessage = "Still-camera export did not produce any output frames.";
        runtimeState->statusMessage.clear();
        return;
    }

    std::vector<StillCameraPreviewLodPreparationRequest> preparationRequests;
    if (viewport != nullptr && exportUsesPreviewDensity) {
        preparationRequests = BuildStillCameraPreviewLodPreparationRequests(*runtimeState);
    }

    const auto setupSize = viewport != nullptr ? CurrentFramebufferViewportSize(*viewport) : ImVec2{1.0F, 1.0F};
    auto preparationState = !preparationRequests.empty()
                                ? std::make_shared<StillCameraPreparationState>()
                                : std::shared_ptr<StillCameraPreparationState>{};
    runtimeState->offlineRenderJob = {
        .active = true,
        .cancelRequested = false,
        .mode = mode,
        .settings = settings,
        .frames = std::move(frames),
        .tiles = {},
        .currentFrame = 0,
        .currentTile = 0,
        .startedAt = std::chrono::steady_clock::now(),
        .videoOutputPath = videoOutputPath,
        .writeExrStack = outputOptions.writeExrStack,
        .writePreviewMp4 = outputOptions.writePreviewMp4,
        .optionalPreviewMp4 = outputOptions.previewMp4Optional,
        .mp4SupersampleScale = outputOptions.mp4SupersampleScale,
        .previewVideoWarning = outputOptions.previewVideoWarning,
        .setupViewportWidth = static_cast<std::uint32_t>(std::max(1.0F, setupSize.x)),
        .setupViewportHeight = static_cast<std::uint32_t>(std::max(1.0F, setupSize.y)),
        .previewDensity = exportUsesPreviewDensity,
        .pointCloudRendererMode =
            raycastExport ? PointCloudRendererMode::Beauty : runtimeState->projectSettings.pointCloudRendererMode,
        .stillCameraJob = true,
        .animationName = "Still Camera",
        .exportVisualName = "Current View",
        .exportLog = MakeExportLogState(settings.outputDirectory, mode),
        .exportBackgroundColor = glm::vec4{
            runtimeState->projectSettings.backgroundColor[0],
            runtimeState->projectSettings.backgroundColor[1],
            runtimeState->projectSettings.backgroundColor[2],
            0.0F,
        },
        .exportEyeDomeLightingEnabled = runtimeState->projectSettings.eyeDomeLightingEnabled,
        .exportEyeDomeLightingThickness = runtimeState->projectSettings.eyeDomeLightingThickness,
        .exportGaussianSplatFootprintBoost = runtimeState->projectSettings.gaussianSplatFootprintBoost,
        .exportPointCloudLayers = {},
        .preparingExport = !preparationRequests.empty(),
        .preparationState = preparationState,
    };
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPlayback.active = false;
    runtimeState->errorMessage.clear();
    if (!preparationRequests.empty()) {
        runtimeState->offlineRenderJob.worker = std::jthread{
            RunStillCameraPreviewLodPreparationWorker,
            std::move(preparationRequests),
            preparationState};
        runtimeState->statusMessage = "Preparing still-camera EXR samples...";
    } else {
        StartStillCameraExportCapture(runtimeState, viewport);
    }
}

void StartAnimationExportJob(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || runtimeState->offlineRenderJob.active) {
        return;
    }

    if (runtimeState->animationPanel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        runtimeState->errorMessage = "Use Export Selected Quick MP4 for MP4 animation exports.";
        runtimeState->statusMessage.clear();
        return;
    }

    auto& settings = runtimeState->renderSettings;
    NormalizeAnimationRenderSettings(&settings);

    if (!HasOfflinePointLayers(*runtimeState)) {
        runtimeState->errorMessage = "Load and show at least one LiDAR layer before exporting an animation.";
        runtimeState->statusMessage.clear();
        return;
    }

    if (!runtimeState->animationPanel.currentPath.has_value() ||
        runtimeState->animationPanel.currentPath->keys.size() < 2U) {
        runtimeState->errorMessage = "Load or save an animation path with at least two keys before exporting.";
        runtimeState->statusMessage.clear();
        return;
    }

    const bool raycastExport =
        runtimeState->animationPanel.exportMode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4;
    if (raycastExport && viewport != nullptr && !viewport->SupportsRaycastExport()) {
        runtimeState->errorMessage = "Beauty Raycast export requires Vulkan compute support on the active GPU.";
        runtimeState->statusMessage.clear();
        return;
    }
    std::filesystem::path videoOutputPath;
    if (raycastExport) {
        if (!invisible_places::output::FfmpegExecutableAvailable(
                invisible_places::output::DefaultFfmpegExecutablePath())) {
            runtimeState->errorMessage =
                "Beauty Raycast MP4 review export requires ffmpeg at " +
                invisible_places::output::DefaultFfmpegExecutablePath().string() + ".";
            runtimeState->statusMessage.clear();
            return;
        }
        const auto animationName = runtimeState->animationPanel.currentPath.has_value()
                                       ? runtimeState->animationPanel.currentPath->name
                                       : std::string{"Animation"};
        videoOutputPath = invisible_places::output::BuildUniqueQuickMp4OutputPath(
            settings.outputDirectory,
            animationName,
            "BeautyRaycast");
    }

    const bool exportUsesPreviewDensity = !raycastExport && runtimeState->animationPanel.exportPreviewDensity;
    if (viewport != nullptr && exportUsesPreviewDensity) {
        PreparePreviewLodSampleCaches(runtimeState, viewport);
    }

    auto frames = invisible_places::output::BuildAnimationRenderSequence(
        runtimeState->animationPanel.currentPath.value(),
        settings);
    if (frames.empty()) {
        runtimeState->errorMessage = "Animation path did not produce any output frames.";
        runtimeState->statusMessage.clear();
        return;
    }

    auto exportPointCloudLayers = BuildAnimationExportPointCloudLayerSnapshot(
        *runtimeState,
        exportUsesPreviewDensity,
        std::nullopt,
        raycastExport ? PointCloudRendererMode::Beauty : runtimeState->projectSettings.pointCloudRendererMode);
    if (exportPointCloudLayers.empty()) {
        runtimeState->errorMessage = "No visible loaded LiDAR layers are available for animation export.";
        runtimeState->statusMessage.clear();
        return;
    }

    const auto setupSize = viewport != nullptr ? CurrentFramebufferViewportSize(*viewport) : ImVec2{1.0F, 1.0F};
    auto writerState = std::make_shared<AnimationExportWriterState>();
    const auto outputOptions =
        MakeAnimationExportOutputOptions(runtimeState->animationPanel.exportMode, settings, videoOutputPath);
    runtimeState->offlineRenderJob = {
        .active = true,
        .cancelRequested = false,
        .mode = runtimeState->animationPanel.exportMode,
        .settings = settings,
        .frames = std::move(frames),
        .tiles = {},
        .currentFrame = 0,
        .currentTile = 0,
        .startedAt = std::chrono::steady_clock::now(),
        .videoOutputPath = videoOutputPath,
        .writeExrStack = outputOptions.writeExrStack,
        .writePreviewMp4 = outputOptions.writePreviewMp4,
        .optionalPreviewMp4 = outputOptions.previewMp4Optional,
        .mp4SupersampleScale = outputOptions.mp4SupersampleScale,
        .previewVideoWarning = outputOptions.previewVideoWarning,
        .setupViewportWidth = static_cast<std::uint32_t>(std::max(1.0F, setupSize.x)),
        .setupViewportHeight = static_cast<std::uint32_t>(std::max(1.0F, setupSize.y)),
        .previewDensity = exportUsesPreviewDensity,
        .pointCloudRendererMode =
            raycastExport ? PointCloudRendererMode::Beauty : runtimeState->projectSettings.pointCloudRendererMode,
        .animationName = runtimeState->animationPanel.currentPath->name,
        .animationFilePath = runtimeState->animationPanel.currentFilePath.empty()
                                  ? std::filesystem::path{}
                                  : std::filesystem::path{runtimeState->animationPanel.currentFilePath},
        .exportLog = MakeExportLogState(
            settings.outputDirectory,
            runtimeState->animationPanel.exportMode),
        .exportBackgroundColor = glm::vec4{
            runtimeState->projectSettings.backgroundColor[0],
            runtimeState->projectSettings.backgroundColor[1],
            runtimeState->projectSettings.backgroundColor[2],
            0.0F,
        },
        .exportEyeDomeLightingEnabled = runtimeState->projectSettings.eyeDomeLightingEnabled,
        .exportEyeDomeLightingThickness = runtimeState->projectSettings.eyeDomeLightingThickness,
        .exportGaussianSplatFootprintBoost = runtimeState->projectSettings.gaussianSplatFootprintBoost,
        .exportPointCloudLayers = std::move(exportPointCloudLayers),
        .writerState = writerState,
    };
    runtimeState->offlineRenderJob.worker = std::jthread{
        RunAnimationExportWriter,
        runtimeState->offlineRenderJob.mode,
        runtimeState->offlineRenderJob.settings,
        outputOptions,
        static_cast<std::uint32_t>(runtimeState->offlineRenderJob.frames.size()),
        writerState};
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPlayback.active = false;
    runtimeState->statusMessage = raycastExport
                                      ? "Rendering Beauty Raycast EXR + MP4 on GPU..."
                                      : "Rendering HQ preview-density EXR stack on GPU...";
    runtimeState->errorMessage.clear();
}

void FinishOfflineRenderJob(
    PreviewRuntimeState* runtimeState,
    const std::string& statusMessage,
    const std::string& errorMessage) {
    if (runtimeState == nullptr) {
        return;
    }

    auto finalStatusMessage = statusMessage;
    auto finalErrorMessage = errorMessage;
    auto& job = runtimeState->offlineRenderJob;
    const bool clearQuickMp4Queue = job.quickMp4BatchJob && !finalErrorMessage.empty();
    RefreshAnimationExportWriterProgress(&job);
    if (job.worker.joinable()) {
        if (job.writerState != nullptr && !job.writerState->completed) {
            RequestAnimationExportWriterCancellation(&job);
        }
        job.worker = std::jthread{};
    }
    job.exportLog.endResidentMemoryBytes = CurrentResidentMemoryBytes();
    if (job.exportLog.endResidentMemoryBytes > job.exportLog.peakResidentMemoryBytes) {
        job.exportLog.peakResidentMemoryBytes = job.exportLog.endResidentMemoryBytes;
    }

    const auto logPath = job.exportLog.path;
    const auto logError = WriteExportLog(job, finalStatusMessage, finalErrorMessage);
    if (!logPath.empty()) {
        if (logError.empty()) {
            if (!finalErrorMessage.empty()) {
                finalErrorMessage += " Export log: " + logPath.string() + ".";
            } else if (!finalStatusMessage.empty()) {
                finalStatusMessage += " Log: " + logPath.string() + ".";
            } else {
                finalStatusMessage = "Export log: " + logPath.string() + ".";
            }
        } else if (!finalErrorMessage.empty()) {
            finalErrorMessage += " Export log failed: " + logError + ".";
        } else if (!finalStatusMessage.empty()) {
            finalStatusMessage += " Export log failed: " + logError + ".";
        } else {
            finalStatusMessage = "Export log failed: " + logError + ".";
        }
    }

    job.active = false;
    job.cancelRequested = false;
    job.writerFinishRequested = false;
    job.preparingExport = false;
    job.preparationState.reset();
    job.writerState.reset();
    if (clearQuickMp4Queue) {
        runtimeState->animationPanel.quickMp4Queue.clear();
        runtimeState->animationPanel.quickMp4QueueTotal = 0;
        runtimeState->animationPanel.quickMp4QueueCompleted = 0;
        runtimeState->animationPanel.quickMp4QueueSkipped = 0;
    }
    runtimeState->statusMessage = finalStatusMessage;
    runtimeState->errorMessage = finalErrorMessage;
}

void RequestOfflineRenderCancellation(OfflineRenderJobState* job) {
    if (job == nullptr) {
        return;
    }

    job->cancelRequested = true;
    if (job->preparationState != nullptr) {
        std::scoped_lock lock(job->preparationState->mutex);
        job->preparationState->cancelRequested = true;
    }
    if (job->preparingExport && job->worker.joinable()) {
        job->worker.request_stop();
    }
}

void ProcessOfflineRenderJobStep(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || !runtimeState->offlineRenderJob.active) {
        return;
    }
    if (viewport == nullptr) {
        FinishOfflineRenderJob(runtimeState, {}, "GPU EXR export requires an active Vulkan viewport.");
        return;
    }

    auto& job = runtimeState->offlineRenderJob;
    if (job.preparingExport) {
        ProcessStillCameraPreparationStep(runtimeState, viewport);
        return;
    }

    RefreshAnimationExportWriterProgress(&job);
    if (job.writerState != nullptr) {
        bool writerCompleted = false;
        std::string writerStatus;
        std::string writerError;
        {
            std::scoped_lock lock(job.writerState->mutex);
            writerCompleted = job.writerState->completed;
            writerStatus = job.writerState->statusMessage;
            writerError = job.writerState->errorMessage;
        }
        if (writerCompleted) {
            const bool wasQuickMp4BatchJob = job.quickMp4BatchJob;
            const bool wasCancelled = job.cancelRequested;
            FinishOfflineRenderJob(runtimeState, writerStatus, writerError);
            if (wasQuickMp4BatchJob) {
                if (wasCancelled || !writerError.empty()) {
                    runtimeState->animationPanel.quickMp4Queue.clear();
                    runtimeState->animationPanel.quickMp4QueueTotal = 0;
                    runtimeState->animationPanel.quickMp4QueueCompleted = 0;
                    runtimeState->animationPanel.quickMp4QueueSkipped = 0;
                    return;
                }
                ++runtimeState->animationPanel.quickMp4QueueCompleted;
                StartNextQueuedQuickMp4Export(runtimeState, viewport);
            }
            return;
        }
    }

    if (job.cancelRequested) {
        RequestAnimationExportWriterCancellation(&job);
        runtimeState->statusMessage = "Cancelling animation export...";
        return;
    }

    if (job.currentFrame >= job.frames.size()) {
        SignalAnimationExportWriterFinish(&job);
        runtimeState->statusMessage = "Finishing animation export writer...";
        return;
    }

    if (!AnimationExportWriterCanAcceptFrame(job)) {
        runtimeState->statusMessage =
            "Animation export writer is saving queued frames (" +
            std::to_string(job.writtenFrameCount) + " / " +
            std::to_string(job.frames.size()) + " written).";
        return;
    }

    const auto elapsed = FormatElapsedTime(std::chrono::steady_clock::now() - job.startedAt);
    runtimeState->statusMessage =
        ("Capturing " + std::string{AnimationExportCaptureLabel(job.mode)} + " frame ") +
        std::to_string(job.currentFrame + 1U) +
        " / " + std::to_string(job.frames.size()) + " on GPU (" + elapsed + ").";

    try {
        const auto renderFrame = [&](std::uint32_t width, std::uint32_t height) {
            const auto renderState = BuildPointCloudExrRenderState(
                job,
                job.frames[job.currentFrame],
                width,
                height);
            if (renderState.pointCloudLayers.empty()) {
                throw std::runtime_error{"No visible loaded LiDAR layers are available for rendering."};
            }

            invisible_places::output::HalfRgbaExrImage renderedImage;
            if (AnimationExportIsRaycast(job.mode)) {
                const invisible_places::renderer::core::PointCloudRaycastFrameRequest request{
                    .renderState = renderState,
                    .width = width,
                    .height = height,
                    .primitiveMode = job.settings.raycastPrimitiveMode,
                    .samplesPerPixel = job.settings.raycastSamplesPerPixel,
                    .maxDepth = job.settings.raycastMaxDepth,
                };
                renderedImage = viewport->RenderPointCloudRaycastFrame(request);
            } else {
                const invisible_places::renderer::core::PointCloudExrFrameRequest request{
                    .renderState = renderState,
                    .width = width,
                    .height = height,
                    .previewDensity = job.previewDensity,
                };
                renderedImage = viewport->RenderPointCloudExrFrame(request);
            }

            if (job.exportEyeDomeLightingEnabled &&
                job.pointCloudRendererMode != PointCloudRendererMode::FastBasic) {
                invisible_places::output::ApplyEyeDomeLighting(
                    &renderedImage,
                    invisible_places::output::EyeDomeLightingSettings{
                        .enabled = true,
                        .outlineThicknessPixels = renderState.eyeDomeLightingThickness});
            }
            return renderedImage;
        };

        const auto captureStart = std::chrono::steady_clock::now();
        const auto mp4Scale = job.writePreviewMp4 ? std::max<std::uint32_t>(1U, job.mp4SupersampleScale) : 1U;
        const auto mp4Width = ScaledRenderDimension(job.settings.width, mp4Scale);
        const auto mp4Height = ScaledRenderDimension(job.settings.height, mp4Scale);
        invisible_places::output::HalfRgbaExrImage image;
        invisible_places::output::HalfRgbaExrImage previewImage;
        if (job.writeExrStack) {
            image = renderFrame(job.settings.width, job.settings.height);
            if (job.writePreviewMp4 && mp4Scale > 1U) {
                previewImage = renderFrame(mp4Width, mp4Height);
            }
        } else {
            image = renderFrame(mp4Width, mp4Height);
        }

        RecordExportGpuCaptureDuration(&job, std::chrono::steady_clock::now() - captureStart);
        SampleExportLogMemory(&job);
        const auto outputFrameIndex = job.settings.startFrame + job.currentFrame;
        if (!QueueAnimationExportFrame(&job, outputFrameIndex, std::move(image), std::move(previewImage))) {
            FinishOfflineRenderJob(runtimeState, {}, "Animation export writer stopped accepting frames.");
            return;
        }
        ++job.currentFrame;
    } catch (const std::exception& error) {
        FinishOfflineRenderJob(runtimeState, {}, "GPU animation export failed: " + std::string{error.what()});
        return;
    }

    if (job.cancelRequested) {
        RequestAnimationExportWriterCancellation(&job);
        runtimeState->statusMessage = "Cancelling animation export...";
        return;
    }

    if (job.currentFrame >= job.frames.size()) {
        SignalAnimationExportWriterFinish(&job);
        return;
    }
}

void DrawAnimationExportSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr) {
        return;
    }

    if (!BeginPanelSection("Export")) {
        return;
    }
    auto& panel = runtimeState->animationPanel;
    auto& settings = runtimeState->renderSettings;
    if (!panel.exportSizeInitialized && viewport != nullptr) {
        const auto viewportSize = CurrentUiViewportSize(*viewport);
        settings.width = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.x));
        settings.height = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.y));
        settings.framesPerSecond = 30U;
        panel.exportSizeInitialized = true;
    }

    const char* exportModeLabels[] = {
        "Fast Preview MP4",
        "HQ Preview-Density EXR",
        "Beauty Raycast EXR + MP4",
    };
    int exportModeIndex = 0;
    switch (panel.exportMode) {
        case invisible_places::output::AnimationExportMode::FastPreviewMp4:
            exportModeIndex = 0;
            break;
        case invisible_places::output::AnimationExportMode::HqPreviewDensityExr:
            exportModeIndex = 1;
            break;
        case invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4:
            exportModeIndex = 2;
            break;
    }
    if (ImGui::Combo("Export Mode", &exportModeIndex, exportModeLabels, IM_ARRAYSIZE(exportModeLabels))) {
        const invisible_places::output::AnimationExportMode exportModes[] = {
            invisible_places::output::AnimationExportMode::FastPreviewMp4,
            invisible_places::output::AnimationExportMode::HqPreviewDensityExr,
            invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4,
        };
        panel.exportMode = exportModes[std::clamp(exportModeIndex, 0, 2)];
    }
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        ImGui::TextDisabled("Fast MP4: full-cloud beauty export, sparse-point smoothing, no AOVs.");
    } else if (panel.exportMode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4) {
        ImGui::TextDisabled("Beauty Raycast: full-source compute EXR plus an MP4 review, using Beauty visual settings.");
    } else {
        ImGui::TextDisabled("HQ EXR: preview-density AOV export; optimized for visual parity, not full-source density.");
    }

    bool settingsChanged = false;
    settingsChanged |= InputTextString("Output Folder", &settings.outputDirectory);
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        ImGui::TextDisabled("Quick MP4 names are generated as Animation_Visual.mp4.");
    } else if (panel.exportMode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4) {
        ImGui::TextDisabled("Raycast writes frame_#### EXRs and an Animation_BeautyRaycast MP4 review.");
    }

    int width = static_cast<int>(settings.width);
    int height = static_cast<int>(settings.height);
    int fps = static_cast<int>(settings.framesPerSecond);
    int startFrame = static_cast<int>(settings.startFrame);
    int endFrame = static_cast<int>(settings.endFrame);

    if (ImGui::InputInt("Width", &width)) {
        settings.width = static_cast<std::uint32_t>(std::max(1, width));
        settingsChanged = true;
    }
    if (ImGui::InputInt("Height", &height)) {
        settings.height = static_cast<std::uint32_t>(std::max(1, height));
        settingsChanged = true;
    }
    if (ImGui::InputInt("Frame Rate", &fps)) {
        settings.framesPerSecond = static_cast<std::uint32_t>(std::max(1, fps));
        settingsChanged = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Changes output samples per second; animation duration and world-space speed stay fixed.");
    }
    if (ImGui::InputInt("Start Frame", &startFrame)) {
        settings.startFrame = static_cast<std::uint32_t>(std::max(0, startFrame));
        settingsChanged = true;
    }
    if (ImGui::InputInt("End Frame", &endFrame)) {
        settings.endFrame = static_cast<std::uint32_t>(std::max(0, endFrame));
        settingsChanged = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Use 0 to render through the final interpolated frame.");
    }
    if (viewport != nullptr && ImGui::Button("Use Viewport Size")) {
        const auto viewportSize = CurrentUiViewportSize(*viewport);
        settings.width = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.x));
        settings.height = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.y));
        settingsChanged = true;
    }
    const bool raycastMode =
        panel.exportMode == invisible_places::output::AnimationExportMode::BeautyRaycastExrMp4;
    const bool raycastSupported = viewport == nullptr || viewport->SupportsRaycastExport();
    if (raycastMode) {
        const char* primitiveLabels[] = {"Style Surfels", "Soft Density Spheres"};
        int primitiveIndex =
            settings.raycastPrimitiveMode == PointCloudRaycastPrimitiveMode::SoftDensitySpheres ? 1 : 0;
        if (ImGui::Combo("Raycast Primitive", &primitiveIndex, primitiveLabels, IM_ARRAYSIZE(primitiveLabels))) {
            settings.raycastPrimitiveMode = primitiveIndex == 1
                                                ? PointCloudRaycastPrimitiveMode::SoftDensitySpheres
                                                : PointCloudRaycastPrimitiveMode::StyleSurfels;
            settingsChanged = true;
        }
        int samplesPerPixel = static_cast<int>(settings.raycastSamplesPerPixel);
        if (ImGui::InputInt("Samples Per Pixel", &samplesPerPixel)) {
            settings.raycastSamplesPerPixel = static_cast<std::uint32_t>(std::clamp(samplesPerPixel, 1, 16));
            settingsChanged = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Raycast export samples per output pixel. Higher values reduce shimmer and cost more.");
        }
        float maxDepth = settings.raycastMaxDepth;
        if (ImGui::InputFloat("Max Raycast Depth", &maxDepth)) {
            settings.raycastMaxDepth = std::max(0.0F, maxDepth);
            settingsChanged = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Use 0 to use the current camera far plane.");
        }
        if (!raycastSupported) {
            ImGui::TextDisabled("Beauty Raycast is unavailable: this Vulkan graphics queue has no compute support.");
        }
    }
    if (settingsChanged) {
        MarkCurrentAnimationExportSettingsDirty(runtimeState);
    }
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        ImGui::TextDisabled("Point density: full source clouds; MP4 smoothing fills tiny gaps during encode.");
    } else if (raycastMode) {
        ImGui::TextDisabled("Point density: full source clouds; BVH raycast coverage is export-only.");
    } else {
        ImGui::SameLine();
        ImGui::Checkbox("Preview Density", &panel.exportPreviewDensity);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Uses the same draw counts and interactive sample buffers as animation playback.");
        }
    }

    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4 &&
        panel.currentPath.has_value()) {
        ImGui::Spacing();
        ImGui::TextUnformatted("Export Visuals");
        const auto visualIndex = ResolveVisiblePointCloudLookdevIndex(*runtimeState);
        if (!visualIndex.has_value()) {
            ImGui::TextDisabled("Load and show a LiDAR layer to choose saved visuals.");
        } else {
            auto& session = runtimeState->sessions[visualIndex.value()];
            EnsurePointVisuals(&session);
            ImGui::TextDisabled("Source: %s", session.displayName.c_str());
            std::size_t exportableVisualCount = 0;
            for (const auto& visual : session.pointVisuals) {
                if (!IsExportablePointVisualName(visual.name)) {
                    continue;
                }
                ++exportableVisualCount;
                bool checked = AnimationPathHasExportVisual(panel.currentPath.value(), visual.name);
                if (ImGui::Checkbox(visual.name.c_str(), &checked)) {
                    SetAnimationPathExportVisual(&panel.currentPath.value(), visual.name, checked);
                    panel.dirty = true;
                }
            }
            if (exportableVisualCount == 0) {
                ImGui::TextDisabled("No saved visuals are available for Quick MP4 export.");
            }
        }
    }

    if (panel.currentPath.has_value()) {
        ImGui::TextDisabled(
            "Export uses the active Animation path (%zu keys).",
            panel.currentPath->keys.size());
    } else {
        ImGui::TextDisabled("Load or save an animation path to export.");
    }

    const auto previewFrames = BuildCurrentAnimationRenderSequence(*runtimeState, settings);
    ImGui::TextDisabled(
        "%s: %zu frames.",
        panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4
            ? "MP4"
            : (raycastMode ? "Raycast EXR + MP4" : "EXR stack"),
        previewFrames.size());
    if (AnimationExportWritesMp4(panel.exportMode) &&
        !invisible_places::output::FfmpegExecutableAvailable(invisible_places::output::DefaultFfmpegExecutablePath())) {
        ImGui::TextDisabled("ffmpeg not found at %s.", invisible_places::output::DefaultFfmpegExecutablePath().string().c_str());
    }

    auto& job = runtimeState->offlineRenderJob;
    if (job.active) {
        RefreshAnimationExportWriterProgress(&job);
        const float frameProgress = job.frames.empty()
                                        ? 0.0F
                                        : static_cast<float>(job.writtenFrameCount) /
                                              static_cast<float>(job.frames.size());
        ImGui::ProgressBar(frameProgress, ImVec2{-FLT_MIN, 0.0F});
        ImGui::Text(
            "Saved %u / %zu, captured %u, queued %zu",
            std::min<std::uint32_t>(job.writtenFrameCount, static_cast<std::uint32_t>(job.frames.size())),
            job.frames.size(),
            std::min<std::uint32_t>(job.currentFrame, static_cast<std::uint32_t>(job.frames.size())),
            job.pendingFrameCount);
        ImGui::TextDisabled(
            runtimeState->pauseLiveViewportDuringExport
                ? "Live 3D view is paused; export rendering continues."
                : "Export runs in its own window; the viewport remains editable.");
        if (job.quickMp4BatchJob) {
            ImGui::TextDisabled(
                "Quick MP4 %zu / %zu: %s",
                panel.quickMp4QueueCompleted + 1U,
                panel.quickMp4QueueTotal,
                job.exportVisualName.c_str());
        }
        if (!job.lastOutputPath.empty()) {
            ImGui::TextWrapped("Last: %s", job.lastOutputPath.string().c_str());
        }
        if (!job.exportLog.path.empty()) {
            ImGui::TextWrapped("Log: %s", job.exportLog.path.string().c_str());
        }
        if (job.writePreviewMp4 && job.mp4SupersampleScale > 1U) {
            ImGui::Text(
                "MP4 supersample: %ux -> %u x %u",
                job.mp4SupersampleScale,
                job.settings.width,
                job.settings.height);
        }
        ImGui::Checkbox("Pause 3D View", &runtimeState->pauseLiveViewportDuringExport);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Skips live scene rendering while this export is active.");
        }
        if (ImGui::Button(job.cancelRequested ? "Cancelling..." : "Cancel Export")) {
            RequestOfflineRenderCancellation(&job);
        }
        EndPanelSection();
        return;
    }

    const bool ffmpegAvailable =
        invisible_places::output::FfmpegExecutableAvailable(invisible_places::output::DefaultFfmpegExecutablePath());
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        ImGui::TextDisabled("Use Export Selected Quick MP4 above for saved animations. Current-view still export is in Camera.");
        EndPanelSection();
        return;
    }

    const bool exportAvailable = !raycastMode || (raycastSupported && ffmpegAvailable);
    if (!exportAvailable) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button(raycastMode ? "Export Beauty Raycast" : "Export HQ EXR Stack")) {
        StartAnimationExportJob(runtimeState, viewport);
    }
    if (!exportAvailable) {
        ImGui::EndDisabled();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            raycastMode
                ? "Writes Beauty Raycast EXRs plus an MP4 review using full-source point clouds."
                : "Writes beauty.RGB, alpha.A, and depth.Z EXRs using preview-density point draws.");
    }
    EndPanelSection();
}

void DrawOfflineRenderOverlay(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->offlineRenderJob.active) {
        return;
    }

    auto& job = runtimeState->offlineRenderJob;
    RefreshAnimationExportWriterProgress(&job);
    const auto& io = ImGui::GetIO();
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize;
    ImGui::SetNextWindowPos(
        ImVec2{std::max(20.0F, io.DisplaySize.x - 420.0F), 24.0F},
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2{340.0F, 0.0F}, ImVec2{520.0F, FLT_MAX});
    ImGui::Begin("Export Progress", nullptr, flags);

    ImGui::TextUnformatted(OfflineRenderJobOverlayLabel(job));
    if (job.quickMp4BatchJob) {
        ImGui::Text(
            "Quick MP4: %zu / %zu",
            runtimeState->animationPanel.quickMp4QueueCompleted + 1U,
            runtimeState->animationPanel.quickMp4QueueTotal);
        ImGui::Text("Visual: %s", job.exportVisualName.c_str());
    }
    if (job.preparingExport && job.preparationState != nullptr) {
        std::size_t completedRequests = 0;
        std::size_t totalRequests = 0;
        std::string currentLayerName;
        {
            std::scoped_lock lock(job.preparationState->mutex);
            completedRequests = job.preparationState->completedRequests;
            totalRequests = job.preparationState->totalRequests;
            currentLayerName = job.preparationState->currentLayerName;
        }
        const float prepareProgress = totalRequests == 0
                                          ? 0.0F
                                          : static_cast<float>(completedRequests) /
                                                static_cast<float>(totalRequests);
        ImGui::ProgressBar(prepareProgress, ImVec2{360.0F, 0.0F});
        ImGui::Text("Preparing samples: %zu / %zu", completedRequests, totalRequests);
        if (!currentLayerName.empty()) {
            ImGui::TextWrapped("Layer: %s", currentLayerName.c_str());
        }
    } else {
        const float frameProgress =
            job.frames.empty()
                ? 0.0F
                : static_cast<float>(job.writtenFrameCount) /
                      static_cast<float>(job.frames.size());
        ImGui::ProgressBar(frameProgress, ImVec2{360.0F, 0.0F});
        ImGui::Text(
            "Captured: %u / %zu",
            std::min<std::uint32_t>(job.currentFrame, static_cast<std::uint32_t>(job.frames.size())),
            job.frames.size());
        ImGui::Text(
            "Saved: %u / %zu",
            std::min<std::uint32_t>(job.writtenFrameCount, static_cast<std::uint32_t>(job.frames.size())),
            job.frames.size());
        ImGui::Text("Queued: %zu", job.pendingFrameCount);
    }
        ImGui::TextUnformatted(
            AnimationExportIsRaycast(job.mode)
                ? "Renderer: Beauty Raycast full source"
                : (job.previewDensity ? "Renderer: GPU preview density" : "Renderer: GPU full source"));
    if (job.writePreviewMp4 && job.mp4SupersampleScale > 1U) {
        ImGui::Text(
            "MP4 supersample: %ux -> %u x %u",
            job.mp4SupersampleScale,
            job.settings.width,
            job.settings.height);
    }
    ImGui::Text(
        "Elapsed: %s",
        FormatElapsedTime(std::chrono::steady_clock::now() - job.startedAt).c_str());
    ImGui::TextWrapped(
        "Output: %s",
        job.mode == invisible_places::output::AnimationExportMode::FastPreviewMp4
            ? job.videoOutputPath.string().c_str()
            : job.settings.outputDirectory.c_str());
    if (job.writePreviewMp4 && !job.videoOutputPath.empty()) {
        ImGui::TextWrapped("MP4: %s", job.videoOutputPath.string().c_str());
    } else if (!job.previewVideoWarning.empty()) {
        ImGui::TextWrapped("%s", job.previewVideoWarning.c_str());
    }
    if (!job.lastOutputPath.empty()) {
        ImGui::TextWrapped("Last: %s", job.lastOutputPath.string().c_str());
    }
    if (ImGui::Button(job.cancelRequested ? "Cancelling..." : "Cancel Export")) {
        RequestOfflineRenderCancellation(&job);
    }
    ImGui::End();
}

void DrawStatusOverlay(const PreviewRuntimeState& runtimeState) {
    if (!runtimeState.projectSettings.showStatusOverlay) {
        return;
    }

    ImGui::SetNextWindowPos(ImVec2{20.0F, 20.0F}, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.88F);
    constexpr auto flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                           ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav |
                           ImGuiWindowFlags_NoInputs;
    ImGui::Begin("PreviewStatusOverlay", nullptr, flags);

    if (!runtimeState.errorMessage.empty()) {
        ImGui::TextColored(ImVec4{0.74F, 0.18F, 0.14F, 1.0F}, "%s", runtimeState.errorMessage.c_str());
    } else if (!runtimeState.statusMessage.empty()) {
        ImGui::Text("%s", runtimeState.statusMessage.c_str());
    }

    ImGui::Separator();
    ImGui::Text("Loaded layers: %s", FormatPointCount(LoadedLayerCount(runtimeState)).c_str());
    ImGui::Text("Visible layers: %s", FormatPointCount(VisibleLayerCount(runtimeState)).c_str());

    if (const auto* session = SelectedLoadedSession(runtimeState); session != nullptr) {
        ImGui::Text("Selected: %s", session->displayName.c_str());
        ImGui::Text("Kind: %s", LayerKindLabel(session->kind));
        if (session->kind == LayerKind::PointCloud) {
            ImGui::Text("Budget: %s", DescribeBudget(session->pointBudget).c_str());
            ImGui::Text(
                "%s: %s",
                PointCloudPreviewLodApplied(runtimeState, *session) ? "Preview LOD" : "Preview",
                DescribePointCloudPreviewDraw(runtimeState, *session).c_str());
            ImGui::Text(
                "LOD Mode: %s",
                PointCloudPreviewLodModeLabel(runtimeState.projectSettings.pointCloudPreviewLodMode));
            ImGui::Text("Mode: %s", PointCloudColorModeLabel(session->pointStyle.colorMode));
        } else {
            ImGui::Text("Mode: %s", GaussianSplatColorModeLabel(session->gsplatStyle.colorMode));
            ImGui::Text("Debug: %s", GaussianSplatDebugModeLabel(session->gsplatStyle.debugMode));
            const auto effectiveQuality = EffectiveGaussianSplatQualityMode(runtimeState, *session);
            ImGui::Text("Quality: %s", GaussianSplatQualityModeLabel(session->gsplatStyle.qualityMode));
            if (effectiveQuality != session->gsplatStyle.qualityMode) {
                ImGui::Text(
                    "Rendering: %s while interacting",
                    GaussianSplatQualityModeLabel(effectiveQuality));
            }
            ImGui::Text(
                "Transform: %s",
                GsplatTransformConventionLabel(runtimeState.projectSettings.gsplatTransformConvention));
        }
        ImGui::Text("LMB orbit  Double-click pivot  RMB/MMB pan  Wheel dolly  F focus  P set pivot  V marker");
    }

    ImGui::End();
}

void DrawPivotOverlay(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (VisibleLayerCount(runtimeState) == 0) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool hasRecentSet =
        runtimeState.pivotOverlay.lastSetAt.time_since_epoch().count() != 0 &&
        (now - runtimeState.pivotOverlay.lastSetAt) < std::chrono::milliseconds{1800};
    if (!runtimeState.pivotOverlay.visible &&
        !runtimeState.cameraInteraction.navigationActive &&
        !hasRecentSet) {
        return;
    }

    const auto matrices = runtimeState.camera.Matrices(CurrentAspectRatio(viewport));
    const auto projected = ProjectWorldPoint(matrices, viewport, runtimeState.camera.OrbitCenter());
    if (!projected.has_value()) {
        return;
    }

    const float age = runtimeState.pivotOverlay.lastSetAt.time_since_epoch().count() != 0
                          ? std::chrono::duration<float>(now - runtimeState.pivotOverlay.lastSetAt).count()
                          : static_cast<float>(ImGui::GetTime());
    const float pulse = 1.0F + (0.12F * std::sin((age * 5.6F) + static_cast<float>(ImGui::GetTime() * 0.5)));
    const ImVec2 center = projected->screen;
    constexpr float baseRadius = 9.0F;
    const float radius = baseRadius * pulse;
    constexpr float crosshairLength = 17.0F;
    constexpr float innerGap = 5.5F;
    ImDrawList* drawList = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
    const ImU32 shadowColor = IM_COL32(0, 0, 0, 170);
    const ImU32 ringColor = IM_COL32(255, 178, 54, 238);
    const ImU32 centerColor = IM_COL32(255, 242, 212, 255);

    drawList->AddCircle(center, radius + 1.5F, shadowColor, 40, 4.0F);
    drawList->AddCircle(center, radius, ringColor, 40, 2.2F);
    drawList->AddCircleFilled(center, 2.8F, centerColor, 16);
    drawList->AddLine(
        ImVec2{center.x - crosshairLength, center.y},
        ImVec2{center.x - innerGap, center.y},
        shadowColor,
        4.0F);
    drawList->AddLine(
        ImVec2{center.x + innerGap, center.y},
        ImVec2{center.x + crosshairLength, center.y},
        shadowColor,
        4.0F);
    drawList->AddLine(
        ImVec2{center.x, center.y - crosshairLength},
        ImVec2{center.x, center.y - innerGap},
        shadowColor,
        4.0F);
    drawList->AddLine(
        ImVec2{center.x, center.y + innerGap},
        ImVec2{center.x, center.y + crosshairLength},
        shadowColor,
        4.0F);
    drawList->AddLine(
        ImVec2{center.x - crosshairLength, center.y},
        ImVec2{center.x - innerGap, center.y},
        ringColor,
        1.6F);
    drawList->AddLine(
        ImVec2{center.x + innerGap, center.y},
        ImVec2{center.x + crosshairLength, center.y},
        ringColor,
        1.6F);
    drawList->AddLine(
        ImVec2{center.x, center.y - crosshairLength},
        ImVec2{center.x, center.y - innerGap},
        ringColor,
        1.6F);
    drawList->AddLine(
        ImVec2{center.x, center.y + innerGap},
        ImVec2{center.x, center.y + crosshairLength},
        ringColor,
        1.6F);
}

float ScreenDistance(ImVec2 left, ImVec2 right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    return std::sqrt((dx * dx) + (dy * dy));
}

float DistanceToScreenSegment(ImVec2 point, ImVec2 start, ImVec2 end) {
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float lengthSquared = (dx * dx) + (dy * dy);
    if (lengthSquared <= 1.0e-6F) {
        return ScreenDistance(point, start);
    }

    const float t = std::clamp(
        (((point.x - start.x) * dx) + ((point.y - start.y) * dy)) / lengthSquared,
        0.0F,
        1.0F);
    return ScreenDistance(point, ImVec2{start.x + (dx * t), start.y + (dy * t)});
}

bool SameWaterRegionVertexRef(const WaterRegionVertexRef& left, const WaterRegionVertexRef& right) {
    return left.feature == right.feature &&
           left.regionIndex == right.regionIndex &&
           left.vertexIndex == right.vertexIndex;
}

const char* WaterRegionFeatureLabel(WaterRegionFeature feature) {
    switch (feature) {
        case WaterRegionFeature::Basin:
            return "basin";
        case WaterRegionFeature::Runoff:
            return "runoff";
        case WaterRegionFeature::Caustic:
            return "caustic";
        case WaterRegionFeature::None:
            break;
    }
    return "region";
}

std::size_t WaterRegionCount(const WaterWorkflowState& water, WaterRegionFeature feature) {
    switch (feature) {
        case WaterRegionFeature::Basin:
            return water.basinRegions.size();
        case WaterRegionFeature::Runoff:
            return water.runoffRegions.size();
        case WaterRegionFeature::Caustic:
            return water.causticRegions.size();
        case WaterRegionFeature::None:
            break;
    }
    return 0U;
}

bool WaterRegionPlacementArmed(const WaterWorkflowState& water, WaterRegionFeature feature) {
    switch (feature) {
        case WaterRegionFeature::Basin:
            return water.basinRegionPlacementArmed;
        case WaterRegionFeature::Runoff:
            return water.runoffRegionPlacementArmed;
        case WaterRegionFeature::Caustic:
            return water.causticRegionPlacementArmed;
        case WaterRegionFeature::None:
            break;
    }
    return false;
}

std::optional<std::size_t> SelectedWaterRegionIndex(
    const WaterWorkflowState& water,
    WaterRegionFeature feature) {
    switch (feature) {
        case WaterRegionFeature::Basin:
            return water.selectedBasinRegionIndex;
        case WaterRegionFeature::Runoff:
            return water.selectedRunoffRegionIndex;
        case WaterRegionFeature::Caustic:
            return water.selectedCausticRegionIndex;
        case WaterRegionFeature::None:
            break;
    }
    return std::nullopt;
}

void SelectWaterRegion(
    PreviewRuntimeState* runtimeState,
    WaterRegionFeature feature,
    std::size_t regionIndex) {
    if (runtimeState == nullptr || regionIndex >= WaterRegionCount(runtimeState->water, feature)) {
        return;
    }
    switch (feature) {
        case WaterRegionFeature::Basin:
            runtimeState->water.selectedBasinRegionIndex = regionIndex;
            break;
        case WaterRegionFeature::Runoff:
            runtimeState->water.selectedRunoffRegionIndex = regionIndex;
            break;
        case WaterRegionFeature::Caustic:
            runtimeState->water.selectedCausticRegionIndex = regionIndex;
            break;
        case WaterRegionFeature::None:
            break;
    }
}

std::vector<invisible_places::io::Float3>* WaterRegionVertices(
    PreviewRuntimeState* runtimeState,
    WaterRegionFeature feature,
    std::size_t regionIndex) {
    if (runtimeState == nullptr) {
        return nullptr;
    }
    switch (feature) {
        case WaterRegionFeature::Basin:
            return regionIndex < runtimeState->water.basinRegions.size()
                       ? &runtimeState->water.basinRegions[regionIndex].vertices
                       : nullptr;
        case WaterRegionFeature::Runoff:
            return regionIndex < runtimeState->water.runoffRegions.size()
                       ? &runtimeState->water.runoffRegions[regionIndex].vertices
                       : nullptr;
        case WaterRegionFeature::Caustic:
            return regionIndex < runtimeState->water.causticRegions.size()
                       ? &runtimeState->water.causticRegions[regionIndex].vertices
                       : nullptr;
        case WaterRegionFeature::None:
            break;
    }
    return nullptr;
}

const std::vector<invisible_places::io::Float3>* WaterRegionVertices(
    const PreviewRuntimeState& runtimeState,
    WaterRegionFeature feature,
    std::size_t regionIndex) {
    switch (feature) {
        case WaterRegionFeature::Basin:
            return regionIndex < runtimeState.water.basinRegions.size()
                       ? &runtimeState.water.basinRegions[regionIndex].vertices
                       : nullptr;
        case WaterRegionFeature::Runoff:
            return regionIndex < runtimeState.water.runoffRegions.size()
                       ? &runtimeState.water.runoffRegions[regionIndex].vertices
                       : nullptr;
        case WaterRegionFeature::Caustic:
            return regionIndex < runtimeState.water.causticRegions.size()
                       ? &runtimeState.water.causticRegions[regionIndex].vertices
                       : nullptr;
        case WaterRegionFeature::None:
            break;
    }
    return nullptr;
}

const std::vector<invisible_places::io::Float3>* WaterRegionHull(
    const PreviewRuntimeState& runtimeState,
    WaterRegionFeature feature,
    std::size_t regionIndex) {
    switch (feature) {
        case WaterRegionFeature::Basin:
            return regionIndex < runtimeState.water.basinRegions.size()
                       ? &runtimeState.water.basinRegions[regionIndex].hull
                       : nullptr;
        case WaterRegionFeature::Runoff:
            return regionIndex < runtimeState.water.runoffRegions.size()
                       ? &runtimeState.water.runoffRegions[regionIndex].hull
                       : nullptr;
        case WaterRegionFeature::Caustic:
            return regionIndex < runtimeState.water.causticRegions.size()
                       ? &runtimeState.water.causticRegions[regionIndex].vertices
                       : nullptr;
        case WaterRegionFeature::None:
            break;
    }
    return nullptr;
}

std::string WaterRegionName(
    const PreviewRuntimeState& runtimeState,
    WaterRegionFeature feature,
    std::size_t regionIndex) {
    switch (feature) {
        case WaterRegionFeature::Basin:
            return regionIndex < runtimeState.water.basinRegions.size()
                       ? runtimeState.water.basinRegions[regionIndex].name
                       : "Basin";
        case WaterRegionFeature::Runoff:
            return regionIndex < runtimeState.water.runoffRegions.size()
                       ? runtimeState.water.runoffRegions[regionIndex].name
                       : "Runoff";
        case WaterRegionFeature::Caustic:
            return regionIndex < runtimeState.water.causticRegions.size()
                       ? runtimeState.water.causticRegions[regionIndex].name
                       : "Caustics";
        case WaterRegionFeature::None:
            break;
    }
    return "Region";
}

bool WaterRegionMaskStale(
    const PreviewRuntimeState& runtimeState,
    WaterRegionFeature feature,
    std::size_t regionIndex) {
    return feature == WaterRegionFeature::Caustic &&
           regionIndex < runtimeState.water.causticRegions.size() &&
           (runtimeState.water.causticRegions[regionIndex].maskDirty ||
            runtimeState.water.causticRegions[regionIndex].maskStale);
}

bool WaterRegionVertexRefValid(
    const PreviewRuntimeState& runtimeState,
    const WaterRegionVertexRef& ref) {
    const auto* vertices = WaterRegionVertices(runtimeState, ref.feature, ref.regionIndex);
    return vertices != nullptr && ref.vertexIndex < vertices->size();
}

std::optional<invisible_places::io::Float3> WaterRegionVertexPoint(
    const PreviewRuntimeState& runtimeState,
    const WaterRegionVertexRef& ref) {
    const auto* vertices = WaterRegionVertices(runtimeState, ref.feature, ref.regionIndex);
    if (vertices == nullptr || ref.vertexIndex >= vertices->size()) {
        return std::nullopt;
    }
    return vertices->at(ref.vertexIndex);
}

void RefreshWaterRegionDerivedValues(
    PreviewRuntimeState* runtimeState,
    WaterRegionFeature feature,
    std::size_t regionIndex) {
    if (runtimeState == nullptr) {
        return;
    }
    switch (feature) {
        case WaterRegionFeature::Basin:
            if (regionIndex < runtimeState->water.basinRegions.size()) {
                invisible_places::water::RefreshWaterBasinRegionDerivedValues(
                    &runtimeState->water.basinRegions[regionIndex]);
            }
            break;
        case WaterRegionFeature::Runoff:
            if (regionIndex < runtimeState->water.runoffRegions.size()) {
                invisible_places::water::RefreshWaterRunoffRegionDerivedValues(
                    &runtimeState->water.runoffRegions[regionIndex]);
            }
            break;
        case WaterRegionFeature::Caustic:
            if (regionIndex < runtimeState->water.causticRegions.size()) {
                invisible_places::water::RefreshWaterCausticRegionDerivedValues(
                    &runtimeState->water.causticRegions[regionIndex]);
            }
            break;
        case WaterRegionFeature::None:
            break;
    }
}

bool SetWaterRegionVertexPoint(
    PreviewRuntimeState* runtimeState,
    const WaterRegionVertexRef& ref,
    invisible_places::io::Float3 point) {
    auto* vertices = WaterRegionVertices(runtimeState, ref.feature, ref.regionIndex);
    if (vertices == nullptr || ref.vertexIndex >= vertices->size()) {
        return false;
    }

    (*vertices)[ref.vertexIndex] = point;
    RefreshWaterRegionDerivedValues(runtimeState, ref.feature, ref.regionIndex);
    return true;
}

bool RemoveWaterRegionVertex(
    PreviewRuntimeState* runtimeState,
    const WaterRegionVertexRef& ref) {
    auto* vertices = WaterRegionVertices(runtimeState, ref.feature, ref.regionIndex);
    if (vertices == nullptr || ref.vertexIndex >= vertices->size()) {
        return false;
    }

    vertices->erase(vertices->begin() + static_cast<std::ptrdiff_t>(ref.vertexIndex));
    RefreshWaterRegionDerivedValues(runtimeState, ref.feature, ref.regionIndex);
    return true;
}

WaterRegionSnapState FindWaterRegionVertexSnapCandidate(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    WaterRegionFeature feature,
    ImVec2 screenPoint,
    const std::optional<WaterRegionVertexRef>& exclude) {
    WaterRegionSnapState snap{};
    float bestDistance = 13.0F;
    const auto matrices = runtimeState.camera.Matrices(CurrentAspectRatio(viewport));
    const std::size_t regionCount = WaterRegionCount(runtimeState.water, feature);
    for (std::size_t regionIndex = 0; regionIndex < regionCount; ++regionIndex) {
        const auto* vertices = WaterRegionVertices(runtimeState, feature, regionIndex);
        if (vertices == nullptr) {
            continue;
        }
        for (std::size_t vertexIndex = 0; vertexIndex < vertices->size(); ++vertexIndex) {
            const WaterRegionVertexRef ref{.feature = feature, .regionIndex = regionIndex, .vertexIndex = vertexIndex};
            if (exclude.has_value() && SameWaterRegionVertexRef(exclude.value(), ref)) {
                continue;
            }
            const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(vertices->at(vertexIndex)));
            if (!projected.has_value()) {
                continue;
            }
            const float distance = ScreenDistance(screenPoint, projected->screen);
            if (distance < bestDistance) {
                bestDistance = distance;
                snap.active = true;
                snap.point = vertices->at(vertexIndex);
                snap.vertex = ref;
                snap.surface = false;
            }
        }
    }
    return snap;
}

WaterRegionSnapState FindWaterRegionSurfaceSnapCandidate(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    WaterRegionSnapState snap{};
    const auto pivot = ResolveSurfacePivot(runtimeState, viewport, screenPoint);
    if (pivot.has_value() && pivot->matchedSurface) {
        snap.active = true;
        snap.point = pivot->point;
        snap.surface = true;
    }
    return snap;
}

WaterRegionSnapState ResolveWaterRegionDragSnap(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    WaterRegionFeature feature,
    ImVec2 screenPoint,
    const WaterRegionVertexRef& draggedVertex) {
    auto snap = FindWaterRegionVertexSnapCandidate(runtimeState, viewport, feature, screenPoint, draggedVertex);
    if (snap.active) {
        return snap;
    }
    return FindWaterRegionSurfaceSnapCandidate(runtimeState, viewport, screenPoint);
}

struct WaterRegionOverlayPalette {
    ImU32 fill = IM_COL32(0, 0, 0, 0);
    ImU32 line = IM_COL32(0, 0, 0, 0);
    ImU32 rawLine = IM_COL32(0, 0, 0, 0);
    ImU32 handle = IM_COL32(0, 0, 0, 0);
    ImU32 handleRing = IM_COL32(0, 0, 0, 0);
    float lineWidth = 1.6F;
    float handleRadius = 4.0F;
};

WaterRegionOverlayPalette WaterRegionPalette(
    WaterRegionFeature feature,
    bool selected,
    bool stale) {
    const int alpha = selected ? 235 : 115;
    const int fillAlpha = selected ? 38 : 14;
    WaterRegionOverlayPalette palette;
    switch (feature) {
        case WaterRegionFeature::Basin:
            palette.line = IM_COL32(64, 218, 255, alpha);
            palette.fill = IM_COL32(64, 218, 255, fillAlpha);
            palette.handle = IM_COL32(214, 251, 255, selected ? 255 : 165);
            break;
        case WaterRegionFeature::Runoff:
            palette.line = IM_COL32(80, 236, 162, alpha);
            palette.fill = IM_COL32(80, 236, 162, fillAlpha);
            palette.handle = IM_COL32(218, 255, 232, selected ? 255 : 165);
            break;
        case WaterRegionFeature::Caustic:
            if (stale) {
                palette.line = IM_COL32(255, 190, 66, alpha);
                palette.fill = IM_COL32(255, 190, 66, selected ? 42 : 18);
                palette.handle = IM_COL32(255, 236, 192, selected ? 255 : 175);
            } else {
                palette.line = IM_COL32(108, 170, 255, alpha);
                palette.fill = IM_COL32(108, 170, 255, fillAlpha);
                palette.handle = IM_COL32(226, 240, 255, selected ? 255 : 165);
            }
            break;
        case WaterRegionFeature::None:
            break;
    }
    palette.rawLine = selected ? IM_COL32(255, 255, 255, 165) : IM_COL32(255, 255, 255, 62);
    palette.handleRing = selected ? IM_COL32(0, 0, 0, 210) : IM_COL32(0, 0, 0, 135);
    palette.lineWidth = selected ? 3.0F : 1.6F;
    palette.handleRadius = selected ? 5.6F : 4.0F;
    return palette;
}

void DrawWaterRegionClosedPolyline(
    ImDrawList* drawList,
    const std::vector<ImVec2>& points,
    ImU32 color,
    float width) {
    if (drawList == nullptr || points.size() < 2U) {
        return;
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto& start = points[index];
        const auto& end = points[(index + 1U) % points.size()];
        drawList->AddLine(start, end, color, width);
    }
}

void DrawWaterRegionOpenPolyline(
    ImDrawList* drawList,
    const std::vector<ImVec2>& points,
    ImU32 color,
    float width) {
    if (drawList == nullptr || points.size() < 2U) {
        return;
    }
    for (std::size_t index = 1; index < points.size(); ++index) {
        drawList->AddLine(points[index - 1U], points[index], color, width);
    }
}

std::vector<ImVec2> ProjectWaterRegionPoints(
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    const std::vector<invisible_places::io::Float3>& points) {
    std::vector<ImVec2> projectedPoints;
    projectedPoints.reserve(points.size());
    for (const auto& point : points) {
        const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(point));
        if (!projected.has_value()) {
            return {};
        }
        projectedPoints.push_back(projected->screen);
    }
    return projectedPoints;
}

std::optional<WaterRegionVertexRef> PickWaterRegionVertexAtScreenPoint(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    WaterRegionFeature feature,
    ImVec2 screenPoint) {
    const auto snap = FindWaterRegionVertexSnapCandidate(runtimeState, viewport, feature, screenPoint, std::nullopt);
    return snap.vertex;
}

void DrawWaterRegionOverlay(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr) {
        return;
    }

    auto& water = runtimeState->water;
    auto& editor = water.regionEditor;
    editor.consumedViewportInputThisFrame = false;
    const WaterRegionFeature feature = water.activeRegionFeature;
    const bool overlayActive = feature != WaterRegionFeature::None && VisibleLayerCount(*runtimeState) > 0;
    if (!overlayActive) {
        editor.hoveredVertex.reset();
        editor.drag = {};
        return;
    }

    if (editor.drag.active &&
        (editor.drag.vertex.feature != feature || !WaterRegionVertexRefValid(*runtimeState, editor.drag.vertex))) {
        editor.drag = {};
    }

    const auto& io = ImGui::GetIO();
    const bool renderViewportHovered = IsMouseOverRenderViewport(viewport);
    const bool canInteractWithViewport =
        !viewport.UiWantsMouseCapture() &&
        renderViewportHovered &&
        !water.placementArmed &&
        !water.movingEmitterIndex.has_value();

    if (editor.drag.active) {
        editor.consumedViewportInputThisFrame = true;
        if (!io.MouseDown[0]) {
            const auto snapVertex = editor.drag.snap.vertex;
            const auto draggedVertex = editor.drag.vertex;
            if (snapVertex.has_value() &&
                snapVertex->feature == draggedVertex.feature &&
                snapVertex->regionIndex == draggedVertex.regionIndex &&
                snapVertex->vertexIndex != draggedVertex.vertexIndex) {
                const std::string regionName =
                    WaterRegionName(*runtimeState, draggedVertex.feature, draggedVertex.regionIndex);
                if (RemoveWaterRegionVertex(runtimeState, draggedVertex)) {
                    runtimeState->statusMessage =
                        "Merged overlapping " + std::string{WaterRegionFeatureLabel(draggedVertex.feature)} +
                        " vertex in " + regionName + ".";
                    runtimeState->errorMessage.clear();
                }
            }
            editor.drag = {};
        } else {
            editor.drag.snap =
                ResolveWaterRegionDragSnap(*runtimeState, viewport, feature, io.MousePos, editor.drag.vertex);
            if (editor.drag.snap.active) {
                SetWaterRegionVertexPoint(runtimeState, editor.drag.vertex, editor.drag.snap.point);
            }
        }
    }

    if (!editor.drag.active && canInteractWithViewport) {
        editor.hoveredVertex = PickWaterRegionVertexAtScreenPoint(*runtimeState, viewport, feature, io.MousePos);
        if (editor.hoveredVertex.has_value()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        }
        if (editor.hoveredVertex.has_value() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            const auto point = WaterRegionVertexPoint(*runtimeState, editor.hoveredVertex.value());
            if (point.has_value()) {
                editor.drag.active = true;
                editor.drag.vertex = editor.hoveredVertex.value();
                editor.drag.originalPoint = point.value();
                editor.drag.snap = {};
                editor.consumedViewportInputThisFrame = true;
                SelectWaterRegion(runtimeState, feature, editor.drag.vertex.regionIndex);
                runtimeState->cameraInteraction.navigationActive = false;
                runtimeState->statusMessage =
                    "Dragging " + std::string{WaterRegionFeatureLabel(feature)} + " vertex.";
                runtimeState->errorMessage.clear();
            }
        }
    } else if (editor.drag.active) {
        editor.hoveredVertex = editor.drag.vertex;
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    } else {
        editor.hoveredVertex.reset();
    }

    const auto matrices = runtimeState->camera.Matrices(CurrentAspectRatio(viewport));
    ImDrawList* drawList = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
    const std::size_t regionCount = WaterRegionCount(water, feature);
    const auto selectedIndex = SelectedWaterRegionIndex(water, feature);
    for (std::size_t regionIndex = 0; regionIndex < regionCount; ++regionIndex) {
        const auto* vertices = WaterRegionVertices(*runtimeState, feature, regionIndex);
        const auto* hull = WaterRegionHull(*runtimeState, feature, regionIndex);
        if (vertices == nullptr || hull == nullptr || vertices->empty()) {
            continue;
        }

        const bool selected = selectedIndex.has_value() && selectedIndex.value() == regionIndex;
        const bool stale = WaterRegionMaskStale(*runtimeState, feature, regionIndex);
        const auto palette = WaterRegionPalette(feature, selected, stale);
        const auto projectedHull = ProjectWaterRegionPoints(matrices, viewport, *hull);
        if (projectedHull.size() >= 3U && projectedHull.size() == hull->size()) {
            if (feature != WaterRegionFeature::Caustic) {
                drawList->AddConvexPolyFilled(
                    projectedHull.data(),
                    static_cast<int>(projectedHull.size()),
                    palette.fill);
            }
            DrawWaterRegionClosedPolyline(
                drawList,
                projectedHull,
                IM_COL32(0, 0, 0, selected ? 210 : 120),
                palette.lineWidth + 2.4F);
            DrawWaterRegionClosedPolyline(drawList, projectedHull, palette.line, palette.lineWidth);
        }

        std::vector<ImVec2> projectedVertices;
        projectedVertices.reserve(vertices->size());
        for (const auto& vertex : *vertices) {
            const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(vertex));
            if (projected.has_value()) {
                projectedVertices.push_back(projected->screen);
            } else {
                projectedVertices.push_back(ImVec2{-FLT_MAX, -FLT_MAX});
            }
        }
        std::vector<ImVec2> contiguousVisible;
        contiguousVisible.reserve(projectedVertices.size());
        for (const auto& point : projectedVertices) {
            if (point.x <= -FLT_MAX * 0.5F) {
                DrawWaterRegionOpenPolyline(drawList, contiguousVisible, IM_COL32(0, 0, 0, 120), 3.6F);
                DrawWaterRegionOpenPolyline(drawList, contiguousVisible, palette.rawLine, 1.5F);
                contiguousVisible.clear();
            } else {
                contiguousVisible.push_back(point);
            }
        }
        DrawWaterRegionOpenPolyline(drawList, contiguousVisible, IM_COL32(0, 0, 0, 120), 3.6F);
        DrawWaterRegionOpenPolyline(drawList, contiguousVisible, palette.rawLine, 1.5F);

        for (std::size_t vertexIndex = 0; vertexIndex < projectedVertices.size(); ++vertexIndex) {
            const auto screen = projectedVertices[vertexIndex];
            if (screen.x <= -FLT_MAX * 0.5F) {
                continue;
            }
            const WaterRegionVertexRef ref{.feature = feature, .regionIndex = regionIndex, .vertexIndex = vertexIndex};
            const bool hovered =
                editor.hoveredVertex.has_value() && SameWaterRegionVertexRef(editor.hoveredVertex.value(), ref);
            const bool dragged =
                editor.drag.active && SameWaterRegionVertexRef(editor.drag.vertex, ref);
            const bool snapTarget =
                editor.drag.active &&
                editor.drag.snap.vertex.has_value() &&
                SameWaterRegionVertexRef(editor.drag.snap.vertex.value(), ref);
            const float radius = palette.handleRadius + (hovered ? 2.0F : 0.0F) + (dragged ? 1.5F : 0.0F);
            drawList->AddCircleFilled(screen, radius + 2.6F, IM_COL32(0, 0, 0, selected ? 175 : 120), 24);
            drawList->AddCircleFilled(screen, radius, palette.handle, 24);
            drawList->AddCircle(screen, radius + 1.8F, palette.handleRing, 24, selected ? 2.0F : 1.4F);
            if (snapTarget) {
                drawList->AddCircle(screen, radius + 6.0F, IM_COL32(255, 255, 255, 245), 28, 2.2F);
            } else if (stale && selected) {
                drawList->AddCircle(screen, radius + 4.2F, IM_COL32(255, 190, 66, 230), 24, 1.7F);
            }
        }

        if (selected && !projectedVertices.empty()) {
            const auto labelAnchor = std::find_if(
                projectedVertices.begin(),
                projectedVertices.end(),
                [](const ImVec2& point) { return point.x > -FLT_MAX * 0.5F; });
            if (labelAnchor != projectedVertices.end()) {
                const std::string label =
                    WaterRegionName(*runtimeState, feature, regionIndex) +
                    (stale ? "  mask stale" : "");
                const ImVec2 labelPosition{labelAnchor->x + 12.0F, labelAnchor->y - 16.0F};
                drawList->AddText(
                    ImVec2{labelPosition.x + 1.0F, labelPosition.y + 1.0F},
                    IM_COL32(0, 0, 0, 210),
                    label.c_str());
                drawList->AddText(labelPosition, stale ? IM_COL32(255, 232, 180, 255) : palette.handle, label.c_str());
            }
        }
    }

    if (!editor.drag.active &&
        WaterRegionPlacementArmed(water, feature) &&
        canInteractWithViewport &&
        !editor.hoveredVertex.has_value()) {
        const auto candidate = ResolveSurfacePivot(*runtimeState, viewport, io.MousePos);
        if (candidate.has_value() && candidate->matchedSurface) {
            const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(candidate->point));
            if (projected.has_value()) {
                const auto palette = WaterRegionPalette(feature, true, false);
                drawList->AddCircleFilled(projected->screen, 8.0F, IM_COL32(0, 0, 0, 100), 28);
                drawList->AddCircle(projected->screen, 9.5F, palette.line, 28, 2.0F);
                drawList->AddCircleFilled(projected->screen, 3.2F, palette.handle, 18);
            }
        }
    }
}

ImU32 WaterEmitterMarkerColor(const WaterEmitter& emitter, bool selected) {
    if (selected) {
        return IM_COL32(255, 255, 255, 255);
    }
    if (emitter.status == WaterEmitterStatus::Disabled) {
        return IM_COL32(126, 145, 156, 135);
    }
    if (emitter.origin == WaterEmitterOrigin::AutoSuggested || emitter.status == WaterEmitterStatus::Candidate) {
        return IM_COL32(255, 198, 82, 235);
    }
    if (emitter.origin == WaterEmitterOrigin::Propagated) {
        return IM_COL32(104, 236, 184, 240);
    }
    return IM_COL32(58, 221, 255, 245);
}

void DrawWaterEmitterOverlay(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr || runtimeState->water.emitters.empty() || VisibleLayerCount(*runtimeState) == 0) {
        return;
    }

    const auto matrices = runtimeState->camera.Matrices(CurrentAspectRatio(viewport));
    ImDrawList* drawList = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
    const auto& io = ImGui::GetIO();
    const bool canPick =
        !viewport.UiWantsMouseCapture() &&
        IsMouseOverRenderViewport(viewport) &&
        runtimeState->water.overlayViewMode != WaterOverlayViewMode::Path &&
        !runtimeState->water.placementArmed &&
        !runtimeState->water.basinRegionPlacementArmed &&
        !runtimeState->water.runoffRegionPlacementArmed &&
        !runtimeState->water.causticRegionPlacementArmed &&
        !runtimeState->water.movingEmitterIndex.has_value();

    std::optional<std::size_t> nearestEmitterIndex;
    float nearestEmitterDistance = 13.0F;

    for (std::size_t index = 0; index < runtimeState->water.emitters.size(); ++index) {
        const auto& emitter = runtimeState->water.emitters[index];
        const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(emitter.position));
        if (!projected.has_value()) {
            continue;
        }

        const bool selected =
            runtimeState->water.selectedEmitterIndex.has_value() &&
            runtimeState->water.selectedEmitterIndex.value() == index;
        const bool moving =
            runtimeState->water.movingEmitterIndex.has_value() &&
            runtimeState->water.movingEmitterIndex.value() == index;
        const ImU32 markerColor = WaterEmitterMarkerColor(emitter, selected);
        const ImVec2 center = projected->screen;
        const float radius = selected ? 8.0F : 5.5F;
        const float pulse = moving ? static_cast<float>(0.75 + (0.25 * std::sin(ImGui::GetTime() * 8.0))) : 1.0F;

        drawList->AddCircleFilled(center, (radius + 3.5F) * pulse, IM_COL32(0, 0, 0, 145), 28);
        drawList->AddCircleFilled(center, radius * pulse, markerColor, 28);
        drawList->AddCircle(center, (radius + 2.5F) * pulse, IM_COL32(2, 24, 31, 210), 28, 2.2F);
        drawList->AddLine(
            ImVec2{center.x - radius - 4.0F, center.y},
            ImVec2{center.x - 2.5F, center.y},
            IM_COL32(255, 255, 255, selected ? 245 : 185),
            1.3F);
        drawList->AddLine(
            ImVec2{center.x + 2.5F, center.y},
            ImVec2{center.x + radius + 4.0F, center.y},
            IM_COL32(255, 255, 255, selected ? 245 : 185),
            1.3F);
        drawList->AddLine(
            ImVec2{center.x, center.y - radius - 4.0F},
            ImVec2{center.x, center.y - 2.5F},
            IM_COL32(255, 255, 255, selected ? 245 : 185),
            1.3F);
        drawList->AddLine(
            ImVec2{center.x, center.y + 2.5F},
            ImVec2{center.x, center.y + radius + 4.0F},
            IM_COL32(255, 255, 255, selected ? 245 : 185),
            1.3F);

        if (selected) {
            const std::string label = moving ? emitter.name + "  click new surface point" : emitter.name;
            const ImVec2 labelPosition{center.x + 12.0F, center.y - 14.0F};
            drawList->AddText(ImVec2{labelPosition.x + 1.0F, labelPosition.y + 1.0F}, IM_COL32(0, 0, 0, 210), label.c_str());
            drawList->AddText(labelPosition, IM_COL32(220, 251, 255, 255), label.c_str());
        }

        if (canPick) {
            const float distance = ScreenDistance(io.MousePos, center);
            if (distance < nearestEmitterDistance) {
                nearestEmitterDistance = distance;
                nearestEmitterIndex = index;
            }
        }
    }

    if (canPick && nearestEmitterIndex.has_value() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        SelectWaterEmitterInViewport(runtimeState, viewport, nearestEmitterIndex.value());
        runtimeState->cameraInteraction.navigationActive = false;
    }
}

bool WaterPathBranchIsHidden(const WaterPathCache& cache, std::uint32_t branchId) {
    return std::find(cache.hiddenBranchIds.begin(), cache.hiddenBranchIds.end(), branchId) !=
           cache.hiddenBranchIds.end();
}

ImU32 WaterPathBranchDebugColor(const WaterPathBranch& branch) {
    if (branch.confidence < 0.45F || branch.gapCount >= 2U) {
        return IM_COL32(255, 190, 54, 255);
    }
    if (branch.role == WaterPathBranchRole::Spread) {
        return IM_COL32(31, 236, 204, 255);
    }
    if (branch.role == WaterPathBranchRole::Secondary) {
        return IM_COL32(20, 202, 184, 245);
    }
    return IM_COL32(0, 218, 255, 255);
}

std::vector<WaterOverlayPoint> WaterPathDisplayAnchorsForBranch(
    const PreviewRuntimeState& runtimeState,
    const WaterPathBranch& branch) {
    std::vector<WaterOverlayPoint> anchors;
    if (!runtimeState.water.pathAnchors.points.empty()) {
        const float flowId = static_cast<float>(branch.id);
        for (const auto& point : runtimeState.water.pathAnchors.points) {
            if (point.particleRole < 0.5F && std::abs(point.flowId - flowId) < 0.25F) {
                anchors.push_back(point);
            }
        }
    }
    if (anchors.size() >= 2U) {
        return anchors;
    }
    return branch.rawAnchors;
}

std::vector<invisible_places::io::Float3> DecimateWaterPathPolyline(
    const std::vector<invisible_places::io::Float3>& points,
    std::size_t maxPointCount) {
    if (points.size() <= maxPointCount || maxPointCount < 2U) {
        return points;
    }

    std::vector<invisible_places::io::Float3> decimated;
    decimated.reserve(maxPointCount);
    const double scale = static_cast<double>(points.size() - 1U) / static_cast<double>(maxPointCount - 1U);
    std::size_t previousIndex = std::numeric_limits<std::size_t>::max();
    for (std::size_t index = 0; index < maxPointCount; ++index) {
        const auto sourceIndex = std::min<std::size_t>(
            points.size() - 1U,
            static_cast<std::size_t>(std::round(static_cast<double>(index) * scale)));
        if (sourceIndex != previousIndex) {
            decimated.push_back(points[sourceIndex]);
            previousIndex = sourceIndex;
        }
    }
    if (decimated.back().x != points.back().x ||
        decimated.back().y != points.back().y ||
        decimated.back().z != points.back().z) {
        decimated.back() = points.back();
    }
    return decimated;
}

void EnsureWaterPathDebugCache(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }
    auto& water = runtimeState->water;
    if (water.pathDebugCacheRevision == water.flowOverlayRevision &&
        !water.pathDebugPolylines.empty()) {
        return;
    }

    water.pathDebugPolylines.clear();
    constexpr std::size_t kMaxMainSamplesPerBranch = 1024U;
    constexpr std::size_t kMaxLaneSamples = 384U;
    constexpr std::size_t kMaxTotalSamples = 50000U;

    auto addPolyline =
        [&](std::uint32_t branchId, bool trailLane, const std::vector<invisible_places::io::Float3>& points) {
            if (points.size() < 2U || WaterPathBranchIsHidden(water.pathCache, branchId)) {
                return;
            }
            water.pathDebugPolylines.push_back({
                .branchId = branchId,
                .trailLane = trailLane,
                .points = DecimateWaterPathPolyline(
                    points,
                    trailLane ? kMaxLaneSamples : kMaxMainSamplesPerBranch),
            });
        };

    struct PolylineKey {
        std::uint32_t branchId = 0;
        bool trailLane = false;
        std::uint32_t laneId = 0;

        bool operator==(const PolylineKey& other) const {
            return branchId == other.branchId && trailLane == other.trailLane && laneId == other.laneId;
        }
    };

    std::optional<PolylineKey> currentKey;
    std::vector<invisible_places::io::Float3> currentPoints;
    const auto flushCurrent = [&]() {
        if (currentKey.has_value()) {
            addPolyline(currentKey->branchId, currentKey->trailLane, currentPoints);
        }
        currentKey.reset();
        currentPoints.clear();
    };

    for (const auto& point : water.flowOverlay.points) {
        const bool mainGuide = point.particleRole >= 1.5F && point.particleRole < 2.5F;
        const bool trailLane = point.particleRole >= 2.5F && point.particleRole < 3.5F;
        if (!mainGuide && !trailLane) {
            flushCurrent();
            continue;
        }
        const PolylineKey key{
            .branchId = static_cast<std::uint32_t>(std::max(0.0F, std::floor(point.flowId + 0.5F))),
            .trailLane = trailLane,
            .laneId = trailLane
                          ? static_cast<std::uint32_t>(std::max(0.0F, std::floor(point.trailLaneId + 0.5F)))
                          : 0U,
        };
        if (currentKey.has_value() && !(currentKey.value() == key)) {
            flushCurrent();
        }
        currentKey = key;
        currentPoints.push_back(point.position);
    }
    flushCurrent();

    if (water.pathDebugPolylines.empty() && water.pathCacheLoaded) {
        for (const auto& branch : water.pathCache.branches) {
            if (WaterPathBranchIsHidden(water.pathCache, branch.id)) {
                continue;
            }
            const auto anchors = WaterPathDisplayAnchorsForBranch(*runtimeState, branch);
            std::vector<invisible_places::io::Float3> points;
            points.reserve(anchors.size());
            for (const auto& anchor : anchors) {
                points.push_back(anchor.position);
            }
            addPolyline(branch.id, false, points);
        }
    }

    std::size_t totalPointCount = 0;
    for (const auto& polyline : water.pathDebugPolylines) {
        totalPointCount += polyline.points.size();
    }
    if (totalPointCount > kMaxTotalSamples) {
        const double scale = static_cast<double>(kMaxTotalSamples) / static_cast<double>(totalPointCount);
        for (auto& polyline : water.pathDebugPolylines) {
            const auto scaledLimit = std::max<std::size_t>(
                2U,
                static_cast<std::size_t>(std::floor(static_cast<double>(polyline.points.size()) * scale)));
            polyline.points = DecimateWaterPathPolyline(polyline.points, scaledLimit);
        }
    }

    water.pathDebugCacheRevision = water.flowOverlayRevision;
}

std::optional<std::uint32_t> PickWaterPathBranchAtScreenPoint(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImVec2 screenPoint) {
    if (runtimeState == nullptr ||
        !runtimeState->water.pathCacheLoaded ||
        runtimeState->water.pathCache.branches.empty()) {
        return std::nullopt;
    }

    EnsureWaterPathDebugCache(runtimeState);
    const auto matrices = runtimeState->camera.Matrices(CurrentAspectRatio(viewport));
    std::optional<std::uint32_t> bestBranchId;
    float bestDistance = 13.5F;
    for (const auto& polyline : runtimeState->water.pathDebugPolylines) {
        if (polyline.trailLane || polyline.points.size() < 2U) {
            continue;
        }
        std::optional<ImVec2> previous;
        for (const auto& point : polyline.points) {
            const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(point));
            if (!projected.has_value()) {
                previous.reset();
                continue;
            }
            if (previous.has_value()) {
                const float distance = DistanceToScreenSegment(screenPoint, previous.value(), projected->screen);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestBranchId = polyline.branchId;
                }
            }
            previous = projected->screen;
        }
    }
    return bestBranchId;
}

bool HideSelectedWaterPathBranch(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr ||
        viewport == nullptr ||
        !runtimeState->water.selectedPathBranchId.has_value() ||
        !runtimeState->water.pathCacheLoaded) {
        return false;
    }
    const auto branchId = runtimeState->water.selectedPathBranchId.value();
    if (WaterPathBranchIsHidden(runtimeState->water.pathCache, branchId)) {
        return false;
    }

    runtimeState->water.pathEditUndoHiddenBranchIds.push_back(runtimeState->water.pathCache.hiddenBranchIds);
    runtimeState->water.pathCache.hiddenBranchIds.push_back(branchId);
    std::sort(
        runtimeState->water.pathCache.hiddenBranchIds.begin(),
        runtimeState->water.pathCache.hiddenBranchIds.end());
    runtimeState->water.pathCache.hiddenBranchIds.erase(
        std::unique(
            runtimeState->water.pathCache.hiddenBranchIds.begin(),
            runtimeState->water.pathCache.hiddenBranchIds.end()),
        runtimeState->water.pathCache.hiddenBranchIds.end());
    runtimeState->water.selectedPathBranchId.reset();
    runtimeState->water.hoveredPathBranchId.reset();
    RefreshWaterOverlayFromAnchors(runtimeState, viewport, WaterOverlayRefreshPersistence::SavePathCache);
    runtimeState->statusMessage = "Hidden selected water path branch.";
    runtimeState->errorMessage.clear();
    return true;
}

bool UndoWaterPathBranchEdit(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr ||
        viewport == nullptr ||
        runtimeState->water.pathEditUndoHiddenBranchIds.empty() ||
        !runtimeState->water.pathCacheLoaded) {
        return false;
    }
    runtimeState->water.pathCache.hiddenBranchIds =
        std::move(runtimeState->water.pathEditUndoHiddenBranchIds.back());
    runtimeState->water.pathEditUndoHiddenBranchIds.pop_back();
    runtimeState->water.selectedPathBranchId.reset();
    runtimeState->water.hoveredPathBranchId.reset();
    RefreshWaterOverlayFromAnchors(runtimeState, viewport, WaterOverlayRefreshPersistence::SavePathCache);
    runtimeState->statusMessage = "Restored last water path edit.";
    runtimeState->errorMessage.clear();
    return true;
}

bool HandleWaterPathViewInput(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr ||
        viewport == nullptr ||
        runtimeState->water.overlayViewMode != WaterOverlayViewMode::Path ||
        !runtimeState->water.pathCacheLoaded ||
        runtimeState->water.pathCache.branches.empty()) {
        return false;
    }

    const auto& io = ImGui::GetIO();
    const bool mouseCanPick =
        !viewport->UiWantsMouseCapture() &&
        IsMouseOverRenderViewport(*viewport) &&
        !runtimeState->water.placementArmed &&
        !runtimeState->water.basinRegionPlacementArmed &&
        !runtimeState->water.runoffRegionPlacementArmed &&
        !runtimeState->water.causticRegionPlacementArmed &&
        !runtimeState->water.movingEmitterIndex.has_value();
    runtimeState->water.hoveredPathBranchId =
        mouseCanPick ? PickWaterPathBranchAtScreenPoint(runtimeState, *viewport, io.MousePos) : std::nullopt;

    if (mouseCanPick && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        runtimeState->water.selectedPathBranchId = runtimeState->water.hoveredPathBranchId;
        if (runtimeState->water.selectedPathBranchId.has_value()) {
            runtimeState->statusMessage = "Selected water path branch " +
                                          std::to_string(runtimeState->water.selectedPathBranchId.value()) + ".";
            runtimeState->errorMessage.clear();
            return true;
        }
    }

    const bool keyboardCanEdit = !viewport->UiWantsKeyboardCapture() && IsRenderViewportFocused();
    if (!keyboardCanEdit) {
        return false;
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) {
        return UndoWaterPathBranchEdit(runtimeState, viewport);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
        return HideSelectedWaterPathBranch(runtimeState, viewport);
    }
    return false;
}

void DrawWaterPathDebugOverlay(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr ||
        runtimeState->water.overlayViewMode != WaterOverlayViewMode::Path ||
        !runtimeState->water.pathCacheLoaded ||
        runtimeState->water.pathCache.branches.empty() ||
        VisibleLayerCount(*runtimeState) == 0) {
        return;
    }

    EnsureWaterPathDebugCache(runtimeState);
    const auto matrices = runtimeState->camera.Matrices(CurrentAspectRatio(viewport));
    ImDrawList* drawList = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
    std::unordered_map<std::uint32_t, const WaterPathBranch*> branchById;
    branchById.reserve(runtimeState->water.pathCache.branches.size());
    for (const auto& branch : runtimeState->water.pathCache.branches) {
        branchById[branch.id] = &branch;
    }

    for (const auto& polyline : runtimeState->water.pathDebugPolylines) {
        if (polyline.points.size() < 2U) {
            continue;
        }
        const auto branchIt = branchById.find(polyline.branchId);
        if (branchIt == branchById.end()) {
            continue;
        }
        const auto& branch = *branchIt->second;
        const bool hovered =
            runtimeState->water.hoveredPathBranchId.has_value() &&
            runtimeState->water.hoveredPathBranchId.value() == branch.id;
        const bool selected =
            runtimeState->water.selectedPathBranchId.has_value() &&
            runtimeState->water.selectedPathBranchId.value() == branch.id;
        ImU32 color = WaterPathBranchDebugColor(branch);
        float width = branch.role == WaterPathBranchRole::Main ? 3.4F : 2.6F;
        if (polyline.trailLane) {
            color = IM_COL32(
                (color >> IM_COL32_R_SHIFT) & 0xFF,
                (color >> IM_COL32_G_SHIFT) & 0xFF,
                (color >> IM_COL32_B_SHIFT) & 0xFF,
                selected ? 105 : 62);
            width = branch.role == WaterPathBranchRole::Main ? 1.25F : 1.0F;
        }
        if (!polyline.trailLane && hovered) {
            color = IM_COL32(160, 255, 255, 255);
            width += 1.4F;
        }
        if (!polyline.trailLane && selected) {
            color = IM_COL32(255, 255, 255, 255);
            width += 2.2F;
        }

        std::optional<ImVec2> firstPoint;
        const auto strokeCurrentPath = [&](ImU32 strokeColor, float strokeWidth) {
            if (drawList->_Path.Size >= 2) {
                drawList->PathStroke(strokeColor, 0, strokeWidth);
            } else {
                drawList->PathClear();
            }
        };
        drawList->PathClear();
        for (const auto& point : polyline.points) {
            const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(point));
            if (!projected.has_value()) {
                strokeCurrentPath(
                    IM_COL32(0, 0, 0, polyline.trailLane ? 70 : (selected ? 230 : 180)),
                    width + (polyline.trailLane ? 1.3F : (selected ? 5.0F : 3.2F)));
                drawList->PathClear();
                continue;
            }
            if (!firstPoint.has_value()) {
                firstPoint = projected->screen;
            }
            drawList->PathLineTo(projected->screen);
        }
        strokeCurrentPath(
            IM_COL32(0, 0, 0, polyline.trailLane ? 70 : (selected ? 230 : 180)),
            width + (polyline.trailLane ? 1.3F : (selected ? 5.0F : 3.2F)));
        drawList->PathClear();
        for (const auto& point : polyline.points) {
            const auto projected = ProjectWorldPoint(matrices, viewport, ToGlm(point));
            if (!projected.has_value()) {
                strokeCurrentPath(color, width);
                drawList->PathClear();
                continue;
            }
            drawList->PathLineTo(projected->screen);
        }
        strokeCurrentPath(color, width);
        if (!polyline.trailLane && firstPoint.has_value()) {
            drawList->AddCircleFilled(firstPoint.value(), selected ? 5.4F : 3.8F, IM_COL32(0, 0, 0, 210), 20);
            drawList->AddCircleFilled(firstPoint.value(), selected ? 3.6F : 2.5F, color, 20);
        }
    }
}

glm::vec3 AnimationKeyPointWorld(
    const AnimationPath& path,
    std::size_t keyIndex,
    AnimationEditTarget target) {
    if (keyIndex >= path.keys.size()) {
        return {};
    }
    const auto& key = path.keys[keyIndex];
    const auto& point = target == AnimationEditTarget::Camera ? key.cameraPosition : key.focusPoint;
    return {point[0], point[1], point[2]};
}

void MoveAnimationKeyPoint(
    AnimationPath* path,
    std::size_t keyIndex,
    AnimationEditTarget target,
    glm::vec3 point) {
    if (target == AnimationEditTarget::Camera) {
        invisible_places::camera::MoveAnimationCameraKey(path, keyIndex, {point.x, point.y, point.z});
    } else {
        invisible_places::camera::MoveAnimationFocusKey(path, keyIndex, {point.x, point.y, point.z});
    }
}

void DrawAnimationCurveOverlay(
    const AnimationPath& path,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    ImU32 color,
    bool drawCameraCurve) {
    if (path.keys.size() < 2U) {
        return;
    }

    std::optional<ImVec2> previous;
    constexpr std::uint32_t kSampleCount = 80;
    for (std::uint32_t sampleIndex = 0; sampleIndex <= kSampleCount; ++sampleIndex) {
        const float amount = static_cast<float>(sampleIndex) / static_cast<float>(kSampleCount);
        const auto evaluation = invisible_places::camera::EvaluateAnimationPath(
            path,
            AnimationDurationSeconds(path) * amount);
        const auto point = drawCameraCurve
                               ? glm::vec3{
                                     evaluation.camera.position[0],
                                     evaluation.camera.position[1],
                                     evaluation.camera.position[2]}
                               : glm::vec3{
                                     evaluation.focusPoint[0],
                                     evaluation.focusPoint[1],
                                     evaluation.focusPoint[2]};
        const auto projected = ProjectWorldPoint(matrices, viewport, point);
        if (projected.has_value() && previous.has_value()) {
            ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())->AddLine(
                previous.value(),
                projected->screen,
                color,
                2.0F);
        }
        previous = projected.has_value() ? std::optional<ImVec2>{projected->screen} : std::nullopt;
    }
}

void DrawAnimationViewportOverlay(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr ||
        !runtimeState->animationPanel.showSplines ||
        !runtimeState->animationPanel.currentPath.has_value()) {
        return;
    }

    auto& panel = runtimeState->animationPanel;
    auto& path = panel.currentPath.value();
    if (path.keys.empty()) {
        return;
    }

    const auto matrices = runtimeState->camera.Matrices(CurrentAspectRatio(viewport));
    ImDrawList* drawList = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
    const ImU32 cameraColor = IM_COL32(80, 160, 255, 230);
    const ImU32 focusColor = IM_COL32(88, 220, 150, 230);
    const ImU32 selectedColor = IM_COL32(255, 232, 128, 255);
    DrawAnimationCurveOverlay(path, matrices, viewport, cameraColor, true);
    DrawAnimationCurveOverlay(path, matrices, viewport, focusColor, false);

    const auto& io = ImGui::GetIO();
    if (panel.drag.active) {
        if (!io.MouseDown[0]) {
            panel.drag.active = false;
        } else {
            const float mouseDeltaAlongAxis =
                ((io.MousePos.x - panel.drag.startMouse.x) * panel.drag.axisScreenDirection.x) +
                ((io.MousePos.y - panel.drag.startMouse.y) * panel.drag.axisScreenDirection.y);
            const float worldDelta = mouseDeltaAlongAxis / std::max(1.0F, panel.drag.pixelsPerWorldUnit);
            if (MoveLinkedAnimationKeyPoint(
                    runtimeState,
                    &path,
                    panel.drag.keyIndex,
                    panel.drag.target,
                    panel.drag.startWorldPoint + (panel.drag.axisWorld * worldDelta))) {
                panel.selectedKeyIndex = panel.drag.keyIndex;
                panel.editTarget = panel.drag.target;
                panel.dirty = true;
            } else {
                panel.drag.active = false;
            }
        }
    }

    const bool canInteractWithRenderViewport =
        panel.drag.active || (!viewport.UiWantsMouseCapture() && IsMouseOverRenderViewport(viewport));

    constexpr float kPickRadius = 10.0F;
    bool selectedKeyThisFrame = false;
    float bestDistance = kPickRadius;
    std::optional<std::size_t> bestKeyIndex;
    std::optional<AnimationEditTarget> bestTarget;

    for (std::size_t keyIndex = 0; keyIndex < path.keys.size(); ++keyIndex) {
        for (const auto target : {AnimationEditTarget::Camera, AnimationEditTarget::Focus}) {
            const auto worldPoint = AnimationKeyPointWorld(path, keyIndex, target);
            const auto projected = ProjectWorldPoint(matrices, viewport, worldPoint);
            if (!projected.has_value()) {
                continue;
            }

            const bool selected = panel.selectedKeyIndex.has_value() &&
                                  panel.selectedKeyIndex.value() == keyIndex &&
                                  panel.editTarget == target;
            const bool multiLinkedCamera =
                target == AnimationEditTarget::Camera &&
                !path.keys[keyIndex].linkedCameraId.empty() &&
                FindCameraAnimationLinks(*runtimeState, path.keys[keyIndex].linkedCameraId).size() > 1U;
            const ImU32 color =
                selected ? selectedColor
                         : (multiLinkedCamera ? IM_COL32(235, 66, 76, 245)
                                              : (target == AnimationEditTarget::Camera ? cameraColor : focusColor));
            drawList->AddCircleFilled(projected->screen, selected ? 6.0F : 4.5F, color, 18);
            drawList->AddCircle(projected->screen, selected ? 8.0F : 6.0F, IM_COL32(0, 0, 0, 170), 18, 2.0F);
            if (multiLinkedCamera) {
                drawList->AddCircle(projected->screen, selected ? 10.0F : 8.0F, IM_COL32(255, 210, 210, 235), 18, 1.5F);
            }

            if (canInteractWithRenderViewport) {
                const float distance = ScreenDistance(io.MousePos, projected->screen);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestKeyIndex = keyIndex;
                    bestTarget = target;
                }
            }
        }
    }

    if (canInteractWithRenderViewport &&
        !panel.drag.active &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        bestKeyIndex.has_value() &&
        bestTarget.has_value()) {
        panel.selectedKeyIndex = bestKeyIndex.value();
        panel.editTarget = bestTarget.value();
        selectedKeyThisFrame = true;
    }

    if (!panel.selectedKeyIndex.has_value() || panel.selectedKeyIndex.value() >= path.keys.size()) {
        return;
    }

    const auto selectedPoint = AnimationKeyPointWorld(path, panel.selectedKeyIndex.value(), panel.editTarget);
    const auto projectedSelected = ProjectWorldPoint(matrices, viewport, selectedPoint);
    if (!projectedSelected.has_value()) {
        return;
    }

    const float cameraDistance = glm::length(selectedPoint - matrices.position);
    const float axisLength = std::max(0.25F, cameraDistance * 0.14F);
    const std::array<glm::vec3, 3> axes{
        glm::vec3{1.0F, 0.0F, 0.0F},
        glm::vec3{0.0F, 1.0F, 0.0F},
        glm::vec3{0.0F, 0.0F, 1.0F},
    };
    const std::array<ImU32, 3> axisColors{
        IM_COL32(255, 88, 88, 245),
        IM_COL32(88, 235, 120, 245),
        IM_COL32(90, 150, 255, 245),
    };

    for (std::size_t axisIndex = 0; axisIndex < axes.size(); ++axisIndex) {
        const auto axisEnd = selectedPoint + (axes[axisIndex] * axisLength);
        const auto projectedEnd = ProjectWorldPoint(matrices, viewport, axisEnd);
        if (!projectedEnd.has_value()) {
            continue;
        }

        drawList->AddLine(projectedSelected->screen, projectedEnd->screen, IM_COL32(0, 0, 0, 185), 5.0F);
        drawList->AddLine(projectedSelected->screen, projectedEnd->screen, axisColors[axisIndex], 3.0F);
        drawList->AddCircleFilled(projectedEnd->screen, 5.0F, axisColors[axisIndex], 14);

        const float handleDistance = canInteractWithRenderViewport
                                         ? DistanceToScreenSegment(io.MousePos, projectedSelected->screen, projectedEnd->screen)
                                         : std::numeric_limits<float>::max();
        if (canInteractWithRenderViewport &&
            !panel.drag.active &&
            !selectedKeyThisFrame &&
            ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            handleDistance < 8.0F) {
            const ImVec2 screenAxis{
                projectedEnd->screen.x - projectedSelected->screen.x,
                projectedEnd->screen.y - projectedSelected->screen.y,
            };
            const float screenAxisLength = std::max(1.0F, std::sqrt((screenAxis.x * screenAxis.x) + (screenAxis.y * screenAxis.y)));
            panel.drag = {
                .active = true,
                .target = panel.editTarget,
                .keyIndex = panel.selectedKeyIndex.value(),
                .axis = static_cast<int>(axisIndex),
                .startWorldPoint = selectedPoint,
                .startMouse = io.MousePos,
                .axisWorld = axes[axisIndex],
                .axisScreenDirection = ImVec2{screenAxis.x / screenAxisLength, screenAxis.y / screenAxisLength},
                .pixelsPerWorldUnit = screenAxisLength / axisLength,
            };
        }
    }
}

void DrawSpinnerArc(float radius, float thickness, ImU32 color) {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 position = ImGui::GetCursorScreenPos();
    const ImVec2 center = ImVec2{position.x + radius, position.y + radius};
    constexpr int segmentCount = 40;
    const float startOffset = static_cast<float>(ImGui::GetTime() * 3.2);
    const float arcLength = kPi * 1.55F;

    drawList->PathClear();
    for (int segmentIndex = 0; segmentIndex <= segmentCount; ++segmentIndex) {
        const float t = static_cast<float>(segmentIndex) / static_cast<float>(segmentCount);
        const float angle = startOffset + (t * arcLength);
        drawList->PathLineTo(
            ImVec2{center.x + (std::cos(angle) * radius), center.y + (std::sin(angle) * radius)});
    }

    drawList->PathStroke(color, false, thickness);
    ImGui::Dummy(ImVec2{radius * 2.0F, radius * 2.0F});
}

void DrawLoadingOverlay(const PreviewRuntimeState& runtimeState) {
    if (!runtimeState.pendingLoad.has_value()) {
        return;
    }

    const auto& io = ImGui::GetIO();
    const auto& pendingLoad = runtimeState.pendingLoad.value();
    const auto& targetSession = runtimeState.sessions[pendingLoad.sessionIndex];
    const float elapsedSeconds = std::chrono::duration<float>(
                                     std::chrono::steady_clock::now() - pendingLoad.startedAt)
                                     .count();
    const bool firstVisibleLayerLoad = VisibleLayerCount(runtimeState) == 0;
    const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
    const ImVec2 viewportPosition = mainViewport != nullptr ? mainViewport->Pos : ImVec2{0.0F, 0.0F};
    const ImVec2 viewportSize = mainViewport != nullptr ? mainViewport->Size : io.DisplaySize;

    if (firstVisibleLayerLoad) {
        if (mainViewport != nullptr) {
            ImGui::SetNextWindowViewport(mainViewport->ID);
        }
        ImGui::SetNextWindowPos(viewportPosition, ImGuiCond_Always);
        ImGui::SetNextWindowSize(viewportSize, ImGuiCond_Always);
        constexpr auto fullscreenFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;
        ImGui::Begin("InitialLayerLoadingOverlay", nullptr, fullscreenFlags);
        const auto& backgroundColor = runtimeState.projectSettings.backgroundColor;
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImGui::GetWindowPos(),
            ImVec2{ImGui::GetWindowPos().x + viewportSize.x, ImGui::GetWindowPos().y + viewportSize.y},
            ImGui::ColorConvertFloat4ToU32(ImVec4{
                backgroundColor[0],
                backgroundColor[1],
                backgroundColor[2],
                0.82F,
            }));

        const ImVec2 overlaySize = ImVec2{390.0F, 214.0F};
        const ImVec2 overlayPosition = ImVec2{
            viewportPosition.x + ((viewportSize.x - overlaySize.x) * 0.5F),
            viewportPosition.y + ((viewportSize.y - overlaySize.y) * 0.5F)};
        ImGui::SetCursorScreenPos(overlayPosition);
        ImGui::BeginChild(
            "LoadingCard",
            overlaySize,
            true,
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::SetCursorPos(ImVec2{(overlaySize.x - 44.0F) * 0.5F, 24.0F});
        DrawSpinnerArc(22.0F, 5.0F, IM_COL32(110, 86, 34, 255));
        ImGui::SetCursorPosX(24.0F);
        ImGui::SetCursorPosY(86.0F);
        ImGui::PushTextWrapPos(overlaySize.x - 24.0F);
        ImGui::Text("%s", targetSession.displayName.c_str());
        ImGui::Text("%s", LayerKindLabel(targetSession.kind));
        ImGui::TextWrapped(
            "%s",
            pendingLoad.phase == PendingLoadPhase::CpuLoading
                ? "The window is ready. Loading the first layer from disk now."
                : "The layer is loaded. Uploading buffers to the GPU now.");
        ImGui::Spacing();
        ImGui::Text("Elapsed: %.1f s", elapsedSeconds);
        ImGui::TextDisabled("The preview will appear automatically.");
        ImGui::PopTextWrapPos();
        ImGui::EndChild();
        ImGui::End();
        return;
    }

    if (mainViewport != nullptr) {
        ImGui::SetNextWindowViewport(mainViewport->ID);
    }
    ImGui::SetNextWindowPos(ImVec2{viewportPosition.x + 24.0F, viewportPosition.y + 100.0F}, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.90F);
    constexpr auto cardFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                               ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;
    ImGui::Begin("BackgroundLayerLoadingCard", nullptr, cardFlags);
    DrawSpinnerArc(14.0F, 4.0F, IM_COL32(110, 86, 34, 255));
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Text("%s", targetSession.displayName.c_str());
    ImGui::Text("%s", LayerKindLabel(targetSession.kind));
    ImGui::TextUnformatted(
        pendingLoad.phase == PendingLoadPhase::CpuLoading ? "Loading in the background..." : "Uploading to GPU...");
    ImGui::TextDisabled("Existing loaded layers stay visible. %.1f s", elapsedSeconds);
    ImGui::EndGroup();
    ImGui::End();
}

void DrawLayerSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport,
    LayerKind layerKind,
    const char* heading) {
    if (!BeginPanelSection(heading)) {
        return;
    }

    const bool hasAnyMatchingLayer = std::any_of(
        runtimeState->sessions.begin(),
        runtimeState->sessions.end(),
        [layerKind](const PreviewLayerSession& session) { return session.kind == layerKind; });
    if (!hasAnyMatchingLayer) {
        ImGui::Text("No %s layers were discovered.", LayerKindLabel(layerKind));
        EndPanelSection();
        return;
    }

    for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
        auto& session = runtimeState->sessions[index];
        if (session.kind != layerKind) {
            continue;
        }

        const bool isSelected = runtimeState->selectedSessionIndex.has_value() &&
                                runtimeState->selectedSessionIndex.value() == index;
        const bool isPendingLoad = runtimeState->pendingLoad.has_value() &&
                                   runtimeState->pendingLoad->sessionIndex == index;

        ImGui::PushID(static_cast<int>(index));
        if (ImGui::Selectable(session.displayName.c_str(), isSelected)) {
            runtimeState->selectedSessionIndex = index;
        }
        const bool hovered = ImGui::IsItemHovered();
        const bool doubleClicked = hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
        if (doubleClicked) {
            runtimeState->selectedSessionIndex = index;
            if (session.loaded) {
                FocusSessionLayer(runtimeState, *viewport, index);
            } else {
                BeginLayerLoad(index, runtimeState);
            }
        }

        ImGui::SameLine();
        ImGui::TextDisabled("%s", LayerKindBadge(session.kind));
        ImGui::SameLine();
        if (isPendingLoad) {
            ImGui::TextColored(ImVec4{0.55F, 0.42F, 0.16F, 1.0F}, "loading...");
        } else if (session.loaded) {
            const ImVec4 loadedColor = session.visible
                                           ? ImVec4{0.16F, 0.46F, 0.22F, 1.0F}
                                           : ImVec4{0.40F, 0.40F, 0.40F, 1.0F};
            ImGui::TextColored(loadedColor, "%s", session.visible ? "loaded" : "hidden");
            if (session.kind == LayerKind::GaussianSplat) {
                ImGui::SameLine();
                const auto requestedQuality = session.gsplatStyle.qualityMode;
                const auto effectiveQuality = EffectiveGaussianSplatQualityMode(*runtimeState, session);
                ImVec4 qualityColor = ImVec4{0.48F, 0.48F, 0.48F, 1.0F};
                if (requestedQuality == GaussianSplatQualityMode::Medium) {
                    qualityColor = ImVec4{0.26F, 0.46F, 0.78F, 1.0F};
                } else if (requestedQuality == GaussianSplatQualityMode::SurfaceGuided) {
                    qualityColor = ImVec4{0.22F, 0.58F, 0.50F, 1.0F};
                } else if (requestedQuality == GaussianSplatQualityMode::High) {
                    qualityColor = ImVec4{0.78F, 0.56F, 0.18F, 1.0F};
                }
                ImGui::TextColored(
                    qualityColor,
                    "[%s]",
                    GaussianSplatQualityModeShortLabel(requestedQuality));
                if (effectiveQuality != requestedQuality) {
                    ImGui::SameLine();
                    ImGui::TextDisabled("-> %s", GaussianSplatQualityModeShortLabel(effectiveQuality));
                }
            }
        } else {
            ImGui::TextDisabled("%s", FormatPointCount(session.totalPrimitives).c_str());
        }
        ImGui::PopID();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Single click selects. Double click loads or focuses.");

    if (auto* session = SelectedSession(runtimeState); session != nullptr) {
        if (session->kind != layerKind) {
            ImGui::Spacing();
            ImGui::Text("Select a %s layer to edit this panel.", LayerKindLabel(layerKind));
            EndPanelSection();
            return;
        }

        ImGui::Spacing();
        ImGui::Text("Selected: %s", session->displayName.c_str());
        ImGui::Text("Kind: %s", LayerKindLabel(session->kind));
        ImGui::Text("Total primitives: %s", FormatPointCount(session->totalPrimitives).c_str());

        if (session->loaded) {
            bool visible = session->visible;
            if (ImGui::Checkbox("Visible", &visible)) {
                session->visible = visible;
            }

            if (session->kind == LayerKind::PointCloud) {
                std::uint64_t requestedBudget = session->pointBudget.activePoints;
                if (ImGui::InputScalar("Budget Points", ImGuiDataType_U64, &requestedBudget)) {
                    session->pointBudget = MakePreviewPointBudgetState(*session, requestedBudget);
                    ClearPreviewLodSampleCache(session);
                    viewport->UpdatePointBudget(
                        runtimeState->selectedSessionIndex.value(),
                        session->pointBudget.sampledIndices);
                    PreparePreviewLodSampleCache(
                        runtimeState,
                        viewport,
                        runtimeState->selectedSessionIndex.value());
                }

                float requestedFraction = session->pointBudget.activeFraction;
                if (DrawRangedFloatControl(
                        "Budget Fraction",
                        &requestedFraction,
                        {.visualMin = 0.0F,
                         .visualMax = 1.0F,
                         .format = "%.3f",
                         .hardMin = 0.0F,
                         .hardMax = 1.0F})) {
                    const auto requestedPoints = static_cast<std::uint64_t>(
                        requestedFraction >= 1.0F ? session->totalPrimitives
                                                  : static_cast<double>(session->totalPrimitives) * requestedFraction);
                    session->pointBudget = MakePreviewPointBudgetState(*session, requestedPoints);
                    ClearPreviewLodSampleCache(session);
                    viewport->UpdatePointBudget(
                        runtimeState->selectedSessionIndex.value(),
                        session->pointBudget.sampledIndices);
                    PreparePreviewLodSampleCache(
                        runtimeState,
                        viewport,
                        runtimeState->selectedSessionIndex.value());
                }

                ImGui::Text("Budget: %s", DescribeBudget(session->pointBudget).c_str());
                ImGui::TextDisabled(
                    "%s: %s",
                    PointCloudPreviewLodApplied(*runtimeState, *session) ? "Preview LOD" : "Preview draw",
                    DescribePointCloudPreviewDraw(*runtimeState, *session).c_str());
            }

            if (ImGui::Button("Unload Selected Layer")) {
                UnloadSelectedLayer(runtimeState, viewport);
            }
        } else {
            if (ImGui::Button("Load Selected Layer")) {
                BeginLayerLoad(runtimeState->selectedSessionIndex.value(), runtimeState);
            }
            if (IsBusyLoading(*runtimeState)) {
                ImGui::TextDisabled("Another layer is loading right now.");
            }
        }
    }
    EndPanelSection();
}

ScalarBindingWidgetConfig PointSizeBindingConfig(const PreviewLayerSession& session) {
    if (session.pointStyle.geometryMode != PointCloudGeometryMode::ScreenSprites) {
        return {.constantMin = 0.0001F,
                .constantMax = 0.1F,
                .defaultOutputMin = 0.0001F,
                .defaultOutputMax = 0.1F,
                .defaultConstant = 0.005F,
                .format = "%.3f",
                .displayScale = 1000.0F,
                .hardMin = 0.0F};
    }

    return {.constantMin = 1.0F,
            .constantMax = 16.0F,
            .defaultOutputMin = 1.0F,
            .defaultOutputMax = 16.0F,
            .defaultConstant = 1.0F,
            .format = "%.2f",
            .hardMin = 0.0F};
}

RenderParameterBinding* PointSizeBinding(PreviewLayerSession* session) {
    if (session == nullptr) {
        return nullptr;
    }
    return session->pointStyle.geometryMode != PointCloudGeometryMode::ScreenSprites
               ? &session->pointStyle.surfelDiameter
               : &session->pointStyle.pointSize;
}

PointCloudFalloffProfile EffectivePointFalloffProfile(const PointCloudStyleState& style) {
    return style.falloffProfile;
}

bool DrawPointCloudPointSettingsSection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("Point Settings")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;
    int geometryModeIndex = static_cast<int>(style.geometryMode);
    const char* geometryModes[] = {"Screen Sprites", "World Surfels", "Camera-Facing World Sprites"};
    if (DrawRightAlignedCombo("Geometry", &geometryModeIndex, geometryModes, IM_ARRAYSIZE(geometryModes))) {
        style.geometryMode = static_cast<PointCloudGeometryMode>(geometryModeIndex);
        changed = true;
    }
    if (style.geometryMode == PointCloudGeometryMode::WorldSurfels && !session->hasNormals) {
        ImGui::TextDisabled("No normals were loaded; surfels face the camera.");
    }

    ImGui::Spacing();
    ImGui::TextUnformatted(
        style.geometryMode == PointCloudGeometryMode::ScreenSprites ? "Point Size (px)" : "Point Size (mm)");
    changed |= DrawScalarBindingBody(
        "Point Size",
        PointSizeBinding(session),
        session->scalarFields,
        PointSizeBindingConfig(*session));
    if (style.flowAnimation && style.geometryMode != PointCloudGeometryMode::ScreenSprites) {
        changed |= ImGui::SliderFloat("Streak Aspect", &style.waterStreakAspect, 1.0F, 32.0F, "%.1f");
    }
    EndPanelSection();
    return changed;
}

std::array<float, 3> SampleStyleColormap(
    const PointCloudStyleState& style,
    PointCloudColormapId colormap,
    float value) {
    if (colormap == PointCloudColormapId::CustomGradient) {
        return invisible_places::renderer::pointcloud::SampleGradient(
            style.gradientStartColor,
            style.gradientEndColor,
            value);
    }
    return invisible_places::renderer::pointcloud::SampleColormap(colormap, value);
}

bool DrawColormapSwatch(
    const char* id,
    const PointCloudStyleState& style,
    PointCloudColormapId colormap,
    bool selected) {
    const ImVec2 size{96.0F, 14.0F};
    ImGui::PushID(id);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton("swatch", size);
    auto* drawList = ImGui::GetWindowDrawList();
    constexpr int kSegmentCount = 16;
    for (int segment = 0; segment < kSegmentCount; ++segment) {
        const float leftT = static_cast<float>(segment) / static_cast<float>(kSegmentCount);
        const float rightT = static_cast<float>(segment + 1) / static_cast<float>(kSegmentCount);
        const auto left = SampleStyleColormap(style, colormap, leftT);
        const auto right = SampleStyleColormap(style, colormap, rightT);
        const ImU32 leftColor = ImGui::ColorConvertFloat4ToU32(ImVec4{left[0], left[1], left[2], 1.0F});
        const ImU32 rightColor = ImGui::ColorConvertFloat4ToU32(ImVec4{right[0], right[1], right[2], 1.0F});
        const float x0 = origin.x + size.x * leftT;
        const float x1 = origin.x + size.x * rightT;
        drawList->AddRectFilledMultiColor(
            ImVec2{x0, origin.y},
            ImVec2{x1, origin.y + size.y},
            leftColor,
            rightColor,
            rightColor,
            leftColor);
    }
    const ImU32 borderColor =
        ImGui::GetColorU32(selected ? ImGuiCol_Text : ImGuiCol_Border);
    drawList->AddRect(origin, ImVec2{origin.x + size.x, origin.y + size.y}, borderColor);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", PointCloudColormapLabel(colormap));
    }
    ImGui::PopID();
    return clicked;
}

bool DrawPointCloudColourSection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("Colour")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;
    int colorModeIndex = static_cast<int>(style.colorMode);
    const char* colorModes[] = {"Source RGB", "Solid Color", "Scalar Colormap"};
    if (DrawRightAlignedCombo("Colour Source", &colorModeIndex, colorModes, IM_ARRAYSIZE(colorModes))) {
        style.colorMode = static_cast<PointCloudColorMode>(colorModeIndex);
        changed = true;
        if (style.colorMode == PointCloudColorMode::ScalarColormap) {
            EnsureFieldMappedBindingDefaults(&style.colormapPosition, session->scalarFields, 0.0F, 1.0F);
        }
    }
    if (!session->hasSourceRgb && style.colorMode == PointCloudColorMode::SourceRgb) {
        ImGui::TextDisabled("Source RGB is not available on this file.");
    }

    if (style.colorMode == PointCloudColorMode::SolidColor) {
        changed |= ImGui::ColorEdit4("Solid Color", style.solidColor.data());
    }

    if (style.colorMode == PointCloudColorMode::ScalarColormap && !session->scalarFields.empty()) {
        int colormapIndex = static_cast<int>(style.colormap);
        const char* colormaps[] = {
            "Viridis",
            "Plasma",
            "Inferno",
            "Magma",
            "Cividis",
            "Turbo",
            "Topo",
            "Land Surface",
            "Exp Fire",
            "Exp Ice",
            "High Contrast",
            "Custom Gradient"};
        if (DrawRightAlignedCombo("Colormap", &colormapIndex, colormaps, IM_ARRAYSIZE(colormaps))) {
            style.colormap = static_cast<PointCloudColormapId>(colormapIndex);
            changed = true;
        }
        constexpr std::array<PointCloudColormapId, 12U> kColormapSwatches{{
            PointCloudColormapId::Viridis,
            PointCloudColormapId::Plasma,
            PointCloudColormapId::Inferno,
            PointCloudColormapId::Magma,
            PointCloudColormapId::Cividis,
            PointCloudColormapId::Turbo,
            PointCloudColormapId::Topographic,
            PointCloudColormapId::LandSurface,
            PointCloudColormapId::ExponentialFire,
            PointCloudColormapId::ExponentialIce,
            PointCloudColormapId::HighContrast,
            PointCloudColormapId::CustomGradient,
        }};
        ImGui::Spacing();
        for (std::size_t index = 0; index < kColormapSwatches.size(); ++index) {
            const auto swatch = kColormapSwatches[index];
            if (index % 2U != 0U) {
                ImGui::SameLine();
            }
            if (DrawColormapSwatch(
                    PointCloudColormapLabel(swatch),
                    style,
                    swatch,
                    style.colormap == swatch)) {
                style.colormap = swatch;
                changed = true;
            }
        }
        if (style.colormap == PointCloudColormapId::CustomGradient) {
            ImGui::Spacing();
            changed |= ImGui::ColorEdit3("Gradient Start", style.gradientStartColor.data());
            changed |= ImGui::ColorEdit3("Gradient End", style.gradientEndColor.data());
        }
        ImGui::Spacing();
        ImGui::TextUnformatted("Colormap Position");
        changed |= DrawScalarBindingBody(
            "Colormap Position",
            &style.colormapPosition,
            session->scalarFields,
            {.constantMin = 0.0F,
             .constantMax = 1.0F,
             .defaultOutputMin = 0.0F,
             .defaultOutputMax = 1.0F,
             .defaultConstant = 0.5F,
             .format = "%.3f",
             .hardMin = 0.0F,
             .hardMax = 1.0F});
    } else if (style.colorMode == PointCloudColorMode::ScalarColormap) {
        ImGui::TextDisabled("No scalar fields were discovered for this cloud.");
    }

    ImGui::Spacing();
    changed |= ImGui::ColorEdit3("Colourise Colour", style.colorizeColor.data());
    changed |= DrawRangedFloatControl(
        "Colourise Amount",
        &style.colorizeAmount,
        {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.3f", .hardMin = 0.0F, .hardMax = 1.0F});

    EndPanelSection();
    return changed;
}

enum class PointCloudStylisationQuickPreset {
    Watercolor,
    LivingWash,
    CartoonInk,
    BrushDabs,
    PencilHatch,
    GrainyPigment
};

void ApplyPointCloudStylisationQuickPreset(
    PointCloudStyleState* style,
    PointCloudStylisationQuickPreset preset) {
    if (style == nullptr) {
        return;
    }

    switch (preset) {
        case PointCloudStylisationQuickPreset::Watercolor:
            style->stylisationMode = PointCloudStylisationMode::NprStylisation;
            style->nprPreset = PointCloudNprPreset::Watercolor;
            style->stylisationStrength = 1.0F;
            style->stylisationColorLevels = 7.0F;
            style->stylisationInkStrength = 0.08F;
            style->stylisationPaperGrain = 0.48F;
            style->stylisationPigmentBleed = 0.78F;
            style->brushAspect = 2.2F;
            style->strokeJitter = 0.25F;
            style->hatchStrength = 0.0F;
            style->strokeOpacityVariance = 0.18F;
            style->pigmentVariation = 0.22F;
            style->pigmentAnimationSpeed = 0.55F;
            style->granulationAngleStrength = 0.40F;
            break;
        case PointCloudStylisationQuickPreset::LivingWash:
            style->stylisationMode = PointCloudStylisationMode::BrushParticles;
            style->nprPreset = PointCloudNprPreset::Watercolor;
            style->stylisationStrength = 1.0F;
            style->stylisationColorLevels = 7.0F;
            style->stylisationInkStrength = 0.10F;
            style->stylisationPaperGrain = 0.65F;
            style->stylisationPigmentBleed = 0.72F;
            style->brushAspect = 1.8F;
            style->strokeJitter = 0.42F;
            style->hatchStrength = 0.05F;
            style->strokeOpacityVariance = 0.35F;
            style->pigmentVariation = 0.62F;
            style->pigmentAnimationSpeed = 1.0F;
            style->granulationAngleStrength = 0.75F;
            break;
        case PointCloudStylisationQuickPreset::CartoonInk:
            style->stylisationMode = PointCloudStylisationMode::NprStylisation;
            style->nprPreset = PointCloudNprPreset::Cartoon;
            style->stylisationStrength = 1.0F;
            style->stylisationColorLevels = 4.0F;
            style->stylisationInkStrength = 0.70F;
            style->stylisationPaperGrain = 0.10F;
            style->stylisationPigmentBleed = 0.20F;
            style->brushAspect = 2.2F;
            style->strokeJitter = 0.15F;
            style->hatchStrength = 0.0F;
            style->strokeOpacityVariance = 0.10F;
            style->pigmentVariation = 0.0F;
            style->pigmentAnimationSpeed = 0.0F;
            style->granulationAngleStrength = 0.0F;
            break;
        case PointCloudStylisationQuickPreset::BrushDabs:
            style->stylisationMode = PointCloudStylisationMode::BrushParticles;
            style->nprPreset = PointCloudNprPreset::Watercolor;
            style->stylisationStrength = 1.0F;
            style->stylisationColorLevels = 6.0F;
            style->stylisationInkStrength = 0.15F;
            style->stylisationPaperGrain = 0.42F;
            style->stylisationPigmentBleed = 0.66F;
            style->brushAspect = 2.4F;
            style->strokeJitter = 0.45F;
            style->hatchStrength = 0.0F;
            style->strokeOpacityVariance = 0.32F;
            style->pigmentVariation = 0.34F;
            style->pigmentAnimationSpeed = 0.70F;
            style->granulationAngleStrength = 0.52F;
            break;
        case PointCloudStylisationQuickPreset::PencilHatch:
            style->stylisationMode = PointCloudStylisationMode::BrushParticles;
            style->nprPreset = PointCloudNprPreset::Cartoon;
            style->stylisationStrength = 0.85F;
            style->stylisationColorLevels = 5.0F;
            style->stylisationInkStrength = 0.58F;
            style->stylisationPaperGrain = 0.24F;
            style->stylisationPigmentBleed = 0.22F;
            style->brushAspect = 3.8F;
            style->strokeJitter = 0.28F;
            style->hatchStrength = 0.75F;
            style->strokeOpacityVariance = 0.16F;
            style->pigmentVariation = 0.08F;
            style->pigmentAnimationSpeed = 0.15F;
            style->granulationAngleStrength = 0.25F;
            break;
        case PointCloudStylisationQuickPreset::GrainyPigment:
            style->stylisationMode = PointCloudStylisationMode::BrushParticles;
            style->nprPreset = PointCloudNprPreset::Watercolor;
            style->stylisationStrength = 0.92F;
            style->stylisationColorLevels = 8.0F;
            style->stylisationInkStrength = 0.10F;
            style->stylisationPaperGrain = 0.86F;
            style->stylisationPigmentBleed = 0.52F;
            style->brushAspect = 1.35F;
            style->strokeJitter = 0.50F;
            style->hatchStrength = 0.12F;
            style->strokeOpacityVariance = 0.55F;
            style->pigmentVariation = 0.72F;
            style->pigmentAnimationSpeed = 0.90F;
            style->granulationAngleStrength = 0.86F;
            break;
    }
}

bool DrawPointCloudStylisationSection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("Stylisation Filters")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;

    bool enabled = style.stylisationMode != PointCloudStylisationMode::Off;
    if (ImGui::Checkbox("Enable Stylisation", &enabled)) {
        if (enabled && style.stylisationMode == PointCloudStylisationMode::Off) {
            ApplyPointCloudStylisationQuickPreset(&style, PointCloudStylisationQuickPreset::Watercolor);
        } else if (!enabled) {
            style.stylisationMode = PointCloudStylisationMode::Off;
        }
        changed = true;
    }

    if (enabled) {
        ImGui::Spacing();
        if (ImGui::Button("Watercolor")) {
            ApplyPointCloudStylisationQuickPreset(&style, PointCloudStylisationQuickPreset::Watercolor);
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Living Wash")) {
            ApplyPointCloudStylisationQuickPreset(&style, PointCloudStylisationQuickPreset::LivingWash);
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cartoon Ink")) {
            ApplyPointCloudStylisationQuickPreset(&style, PointCloudStylisationQuickPreset::CartoonInk);
            changed = true;
        }
        if (ImGui::Button("Brush Dabs")) {
            ApplyPointCloudStylisationQuickPreset(&style, PointCloudStylisationQuickPreset::BrushDabs);
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Pencil Hatch")) {
            ApplyPointCloudStylisationQuickPreset(&style, PointCloudStylisationQuickPreset::PencilHatch);
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Grainy Pigment")) {
            ApplyPointCloudStylisationQuickPreset(&style, PointCloudStylisationQuickPreset::GrainyPigment);
            changed = true;
        }

        ImGui::Spacing();
        int modeIndex = static_cast<int>(style.stylisationMode);
        const char* modes[] = {"Off", "NPR Stylisation", "Brush Particles"};
        if (DrawRightAlignedCombo("Filter", &modeIndex, modes, IM_ARRAYSIZE(modes))) {
            style.stylisationMode = static_cast<PointCloudStylisationMode>(modeIndex);
            changed = true;
        }

        int presetIndex = static_cast<int>(style.nprPreset);
        const char* presets[] = {"Watercolor", "Cartoon"};
        if (style.stylisationMode != PointCloudStylisationMode::Off &&
            DrawRightAlignedCombo("Style", &presetIndex, presets, IM_ARRAYSIZE(presets))) {
            style.nprPreset = static_cast<PointCloudNprPreset>(presetIndex);
            changed = true;
        }

        if (style.stylisationMode != PointCloudStylisationMode::Off) {
            changed |= DrawRangedFloatControl(
                "Strength",
                &style.stylisationStrength,
                {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});

            if (style.stylisationMode == PointCloudStylisationMode::NprStylisation ||
                style.nprPreset == PointCloudNprPreset::Cartoon) {
                changed |= DrawRangedFloatControl(
                    "Colour Levels",
                    &style.stylisationColorLevels,
                    {.visualMin = 2.0F, .visualMax = 12.0F, .format = "%.0f", .hardMin = 2.0F, .hardMax = 16.0F});
                changed |= DrawRangedFloatControl(
                    "Ink Strength",
                    &style.stylisationInkStrength,
                    {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
            }

            changed |= DrawRangedFloatControl(
                "Paper Grain",
                &style.stylisationPaperGrain,
                {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
            changed |= DrawRangedFloatControl(
                "Pigment Bleed",
                &style.stylisationPigmentBleed,
                {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
            if (style.nprPreset == PointCloudNprPreset::Watercolor) {
                changed |= DrawRangedFloatControl(
                    "Pigment Variation",
                    &style.pigmentVariation,
                    {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
                changed |= DrawRangedFloatControl(
                    "Pigment Drift",
                    &style.pigmentAnimationSpeed,
                    {.visualMin = 0.0F, .visualMax = 2.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 4.0F});
                changed |= DrawRangedFloatControl(
                    "Angle Granulation",
                    &style.granulationAngleStrength,
                    {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
            }
            changed |= DrawRangedFloatControl(
                "Hatch Strength",
                &style.hatchStrength,
                {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});

            if (style.stylisationMode == PointCloudStylisationMode::BrushParticles) {
                changed |= DrawRangedFloatControl(
                    "Brush Aspect",
                    &style.brushAspect,
                    {.visualMin = 0.25F, .visualMax = 6.0F, .format = "%.2f", .hardMin = 0.25F, .hardMax = 6.0F});
                changed |= DrawRangedFloatControl(
                    "Stroke Jitter",
                    &style.strokeJitter,
                    {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
                changed |= DrawRangedFloatControl(
                    "Opacity Variance",
                    &style.strokeOpacityVariance,
                    {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
            }
        }
    }

    EndPanelSection();
    return changed;
}

bool DrawPointCloudSurfaceMotionSection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("Surface Motion")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;
    bool enabled = style.roughnessMotionStrength > 1.0e-5F;
    if (ImGui::Checkbox("Roughness Motion", &enabled)) {
        style.roughnessMotionStrength = enabled ? 0.012F : 0.0F;
        changed = true;
    }

    if (enabled) {
        changed |= DrawRangedFloatControl(
            "Strength",
            &style.roughnessMotionStrength,
            {.visualMin = 0.0F,
             .visualMax = 0.08F,
             .format = "%.3f m",
             .hardMin = 0.0F,
             .hardMax = 1.0F});
        changed |= DrawRangedFloatControl(
            "Noise Scale",
            &style.roughnessMotionScale,
            {.visualMin = 0.05F,
             .visualMax = 12.0F,
             .format = "%.2f",
             .hardMin = 0.01F,
             .hardMax = 50.0F});
        changed |= DrawRangedFloatControl(
            "Speed",
            &style.roughnessMotionSpeed,
            {.visualMin = 0.0F,
             .visualMax = 2.0F,
             .format = "%.2f",
             .hardMin = 0.0F,
             .hardMax = 8.0F});
        changed |= DrawRangedFloatControl(
            "Roughness Threshold",
            &style.roughnessMotionThreshold,
            {.visualMin = 0.0F,
             .visualMax = 1.0F,
             .format = "%.2f",
             .hardMin = 0.0F,
             .hardMax = 1.0F});
        changed |= DrawRangedFloatControl(
            "Ground ID Target",
            &style.roughnessMotionGroundId,
            {.visualMin = 0.0F,
             .visualMax = 1.0F,
             .format = "%.0f",
             .hardMin = 0.0F,
             .hardMax = 1.0F});
    }

    EndPanelSection();
    return changed;
}

bool DrawPointCloudFalloffSection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("Point Falloff")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;
    int falloffIndex = static_cast<int>(style.falloffProfile);
    const char* falloffProfiles[] = {"Hard Disc", "Soft Disc", "Gaussian", "Rim"};
    if (DrawRightAlignedCombo("Profile", &falloffIndex, falloffProfiles, IM_ARRAYSIZE(falloffProfiles))) {
        style.falloffProfile = static_cast<PointCloudFalloffProfile>(falloffIndex);
        changed = true;
    }

    const auto effectiveFalloff = EffectivePointFalloffProfile(style);
    changed |= ImGui::Checkbox("Solid Centres", &style.solidCenters);
    if (effectiveFalloff == PointCloudFalloffProfile::SoftDisc) {
        changed |= DrawRangedFloatControl(
            "Inner Radius",
            &style.innerRadius,
            {.visualMin = 0.0F, .visualMax = 0.99F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 0.99F});
    } else if (effectiveFalloff == PointCloudFalloffProfile::Gaussian) {
        changed |= DrawRangedFloatControl(
            "Gaussian Sharpness",
            &style.gaussianSharpness,
            {.visualMin = 0.1F, .visualMax = 16.0F, .format = "%.2f", .hardMin = 0.001F});
    } else if (effectiveFalloff == PointCloudFalloffProfile::Rim) {
        changed |= DrawRangedFloatControl(
            "Feather Power",
            &style.featherPower,
            {.visualMin = 0.1F, .visualMax = 8.0F, .format = "%.2f", .hardMin = 0.001F});
    }

    EndPanelSection();
    return changed;
}

bool DrawVisualBindingSection(
    const char* sectionLabel,
    const char* id,
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    const ScalarBindingWidgetConfig& config) {
    if (!BeginPanelSection(sectionLabel)) {
        return false;
    }
    const bool changed = DrawScalarBindingBody(id, binding, scalarFields, config);
    EndPanelSection();
    return changed;
}

bool DrawPointCloudEmissionSection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("Emission")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;
    changed |= DrawScalarBindingBody(
        "Emission",
        &style.emissiveStrength,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 2.5F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 2.5F,
         .defaultConstant = 0.0F,
         .format = "%.2f",
         .hardMin = 0.0F});
    ImGui::Spacing();
    changed |= DrawRangedFloatControl(
        "Exposure",
        &style.exposure,
        {.visualMin = 0.0F, .visualMax = 8.0F, .format = "%.2f", .hardMin = 0.0F});

    EndPanelSection();
    return changed;
}

bool DrawPointCloudXraySection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("X-Ray")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;
    changed |= DrawScalarBindingBody(
        "X-Ray",
        &style.xrayStrength,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 1.0F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 1.0F,
         .defaultConstant = 0.0F,
         .format = "%.2f",
         .hardMin = 0.0F,
         .hardMax = 1.0F});
    changed |= DrawRangedFloatControl(
        "Depth Falloff",
        &style.depthFalloff,
        {.visualMin = 0.0F, .visualMax = 400.0F, .format = "%.1f", .hardMin = 0.0F});
    changed |= DrawRangedFloatControl(
        "Depth Bias",
        &style.depthBias,
        {.visualMin = 0.0F, .visualMax = 0.01F, .format = "%.5f", .hardMin = 0.0F});
    changed |= DrawRangedFloatControl(
        "Front Alpha",
        &style.frontAlpha,
        {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
    changed |= DrawRangedFloatControl(
        "Hidden Alpha",
        &style.hiddenAlpha,
        {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});

    EndPanelSection();
    return changed;
}

bool DrawPointCloudDepthFadeSection(PreviewLayerSession* session) {
    if (session == nullptr) {
        return false;
    }

    return DrawVisualBindingSection(
        "Depth Fade",
        "Depth Fade",
        &session->pointStyle.depthFade,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 1.0F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 1.0F,
         .defaultConstant = 0.0F,
         .format = "%.2f",
         .hardMin = 0.0F,
         .hardMax = 1.0F});
}

const char* PointCloudDepthContributionLabel(PointCloudDepthContribution contribution) {
    switch (contribution) {
        case PointCloudDepthContribution::None:
            return "None";
        case PointCloudDepthContribution::AlphaThreshold:
            return "Alpha Threshold";
        case PointCloudDepthContribution::Always:
            return "Always";
    }

    return "Alpha Threshold";
}

bool DrawPointCloudCompositingSection(PreviewLayerSession* session) {
    if (session == nullptr || !BeginPanelSection("Compositing")) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;
    changed |= DrawRangedFloatControl(
        "Density Scale",
        &style.densityScale,
        {.visualMin = 0.0F, .visualMax = 8.0F, .format = "%.2f", .hardMin = 0.0F});
    changed |= DrawRangedFloatControl(
        "Density Clamp",
        &style.densityClamp,
        {.visualMin = 0.0F, .visualMax = 512.0F, .format = "%.1f", .hardMin = 0.0F});

    int depthContributionIndex = static_cast<int>(style.depthContribution);
    const char* depthContributionModes[] = {
        PointCloudDepthContributionLabel(PointCloudDepthContribution::None),
        PointCloudDepthContributionLabel(PointCloudDepthContribution::AlphaThreshold),
        PointCloudDepthContributionLabel(PointCloudDepthContribution::Always),
    };
    if (DrawRightAlignedCombo(
            "Depth Contribution",
            &depthContributionIndex,
            depthContributionModes,
            IM_ARRAYSIZE(depthContributionModes))) {
        style.depthContribution = static_cast<PointCloudDepthContribution>(depthContributionIndex);
        changed = true;
    }
    if (style.depthContribution == PointCloudDepthContribution::AlphaThreshold) {
        changed |= DrawRangedFloatControl(
            "Depth Alpha Threshold",
            &style.depthAlphaThreshold,
            {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.2f", .hardMin = 0.0F, .hardMax = 1.0F});
    }

    EndPanelSection();
    return changed;
}

bool DrawPointCloudStyleSection(PreviewLayerSession* session) {
    if (session == nullptr) {
        return false;
    }

    auto& style = session->pointStyle;
    bool changed = false;

    changed |= DrawPointCloudPointSettingsSection(session);
    changed |= DrawPointCloudStylisationSection(session);
    changed |= DrawPointCloudSurfaceMotionSection(session);
    changed |= DrawPointCloudFalloffSection(session);
    changed |= DrawPointCloudColourSection(session);
    changed |= DrawVisualBindingSection(
        "Opacity",
        "Opacity",
        &style.opacity,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 1.0F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 1.0F,
         .defaultConstant = 1.0F,
         .format = "%.2f",
         .hardMin = 0.0F,
         .hardMax = 1.0F});
    changed |= DrawPointCloudEmissionSection(session);
    changed |= DrawPointCloudXraySection(session);
    changed |= DrawPointCloudDepthFadeSection(session);
    changed |= DrawPointCloudCompositingSection(session);

    if (changed) {
        SanitizePointCloudStyle(session);
    }
    return changed;
}

void DrawGaussianSplatStyleSection(PreviewLayerSession* session) {
    auto& style = session->gsplatStyle;

    ImGui::TextDisabled(
        "Layer controls multiply the per-splat scale and opacity stored in the PLY. "
        "Surface Guided uses the point-cloud surface depth to suppress floaters and back clutter.");

    int qualityModeIndex = static_cast<int>(style.qualityMode);
    const char* qualityModes[] = {"Fast", "Medium", "Surface Guided", "High"};
    if (ImGui::Combo("Quality", &qualityModeIndex, qualityModes, IM_ARRAYSIZE(qualityModes))) {
        style.qualityMode = static_cast<GaussianSplatQualityMode>(qualityModeIndex);
    }

    int colorModeIndex = static_cast<int>(style.colorMode);
    const char* colorModes[] = {"Full SH", "DC Only"};
    if (ImGui::Combo("Color Mode", &colorModeIndex, colorModes, IM_ARRAYSIZE(colorModes))) {
        style.colorMode = static_cast<GaussianSplatColorMode>(colorModeIndex);
    }

    int debugModeIndex = static_cast<int>(style.debugMode);
    const char* debugModes[] = {"Final", "Opacity", "Scale", "Depth", "Layer Tint"};
    if (ImGui::Combo("Debug View", &debugModeIndex, debugModes, IM_ARRAYSIZE(debugModes))) {
        style.debugMode = static_cast<GaussianSplatDebugMode>(debugModeIndex);
    }

    ImGui::Checkbox("Apply Transform", &style.transformEnabled);
    ImGui::ColorEdit4("Layer Tint", style.layerTint.data());
    DrawRangedFloatControl(
        "Opacity Multiplier",
        &style.opacityMultiplier,
        {.visualMin = 0.0F, .visualMax = 2.0F, .format = "%.2f", .hardMin = 0.0F});
    DrawRangedFloatControl(
        "Scale Multiplier",
        &style.scaleMultiplier,
        {.visualMin = 0.1F, .visualMax = 4.0F, .format = "%.2f", .hardMin = 0.001F});
    DrawRangedFloatControl(
        "Exposure",
        &style.exposure,
        {.visualMin = 0.0F, .visualMax = 4.0F, .format = "%.2f", .hardMin = 0.0F});
    DrawRangedFloatControl(
        "Saturation",
        &style.saturation,
        {.visualMin = 0.0F, .visualMax = 2.0F, .format = "%.2f", .hardMin = 0.0F});
}

void DrawStyleSection(PreviewRuntimeState* runtimeState) {
    if (!BeginPanelSection("Style")) {
        return;
    }

    auto* session = SelectedLoadedSession(runtimeState);
    if (session == nullptr) {
        ImGui::TextUnformatted("Select a loaded layer to edit lookdev.");
        EndPanelSection();
        return;
    }

    if (session->kind == LayerKind::PointCloud) {
        if (DrawPointCloudStyleSection(session)) {
            MarkPointVisualEdited(session);
            SyncWaterPointVisualSelectionFromSession(runtimeState, *session);
        }
    } else {
        const auto effectiveQuality = EffectiveGaussianSplatQualityMode(*runtimeState, *session);
        if (effectiveQuality != session->gsplatStyle.qualityMode) {
            ImGui::TextDisabled(
                "Rendering as %s while interacting.",
                GaussianSplatQualityModeLabel(effectiveQuality));
        }
        DrawGaussianSplatStyleSection(session);
    }
    EndPanelSection();
}

void DrawStillCameraExportSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr) {
        return;
    }

    if (!BeginPanelSection("Still Camera Export")) {
        return;
    }

    auto& panel = runtimeState->cameraPanel;
    auto& settings = runtimeState->renderSettings;
    if (!runtimeState->animationPanel.exportSizeInitialized) {
        const auto viewportSize = CurrentUiViewportSize(viewport);
        settings.width = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.x));
        settings.height = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.y));
        settings.framesPerSecond = 30U;
        runtimeState->animationPanel.exportSizeInitialized = true;
    }

    const char* stillExportLabels[] = {"Preview MP4", "EXR Stack"};
    int stillExportIndex =
        panel.stillExportMode == invisible_places::output::AnimationExportMode::HqPreviewDensityExr ? 1 : 0;
    if (ImGui::Combo("Format", &stillExportIndex, stillExportLabels, IM_ARRAYSIZE(stillExportLabels))) {
        panel.stillExportMode = stillExportIndex == 1
                                    ? invisible_places::output::AnimationExportMode::HqPreviewDensityExr
                                    : invisible_places::output::AnimationExportMode::FastPreviewMp4;
    }

    bool settingsChanged = false;
    settingsChanged |= InputTextString("Output Folder", &settings.outputDirectory);

    int width = static_cast<int>(settings.width);
    int height = static_cast<int>(settings.height);
    int fps = static_cast<int>(settings.framesPerSecond);
    float stillDurationSeconds = settings.stillCameraDurationSeconds;
    if (ImGui::InputInt("Width", &width)) {
        settings.width = static_cast<std::uint32_t>(std::max(1, width));
        settingsChanged = true;
    }
    if (ImGui::InputInt("Height", &height)) {
        settings.height = static_cast<std::uint32_t>(std::max(1, height));
        settingsChanged = true;
    }
    if (ImGui::InputInt("Frame Rate", &fps)) {
        settings.framesPerSecond = static_cast<std::uint32_t>(std::max(1, fps));
        settingsChanged = true;
    }
    if (ImGui::InputFloat("Duration", &stillDurationSeconds, 0.1F, 1.0F, "%.2f s")) {
        settings.stillCameraDurationSeconds = std::clamp(stillDurationSeconds, 0.001F, 3600.0F);
        settingsChanged = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("The camera stays fixed while animated water and visual effects advance.");
    }
    if (ImGui::Button("Use Viewport Size")) {
        const auto viewportSize = CurrentUiViewportSize(viewport);
        settings.width = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.x));
        settings.height = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.y));
        settingsChanged = true;
    }
    if (settingsChanged) {
        NormalizeAnimationRenderSettings(&settings);
        MarkCurrentAnimationExportSettingsDirty(runtimeState);
    }

    const auto stillFrameCount = std::max<std::uint32_t>(
        1U,
        static_cast<std::uint32_t>(
            std::ceil(
                std::max(0.001F, settings.stillCameraDurationSeconds) *
                static_cast<float>(std::max<std::uint32_t>(1U, settings.framesPerSecond)))));
    ImGui::TextDisabled(
        "%s: %u frames from the current view.",
        panel.stillExportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4
            ? "Preview MP4"
            : "EXR stack",
        stillFrameCount);
    if (panel.stillExportMode == invisible_places::output::AnimationExportMode::HqPreviewDensityExr) {
        ImGui::TextDisabled("EXR stack uses full-source point density to avoid preview sampling artifacts.");
    }

    auto& job = runtimeState->offlineRenderJob;
    if (job.active) {
        RefreshAnimationExportWriterProgress(&job);
        if (job.stillCameraJob) {
            if (job.preparingExport && job.preparationState != nullptr) {
                std::size_t completedRequests = 0;
                std::size_t totalRequests = 0;
                std::string currentLayerName;
                {
                    std::scoped_lock lock(job.preparationState->mutex);
                    completedRequests = job.preparationState->completedRequests;
                    totalRequests = job.preparationState->totalRequests;
                    currentLayerName = job.preparationState->currentLayerName;
                }
                const float prepareProgress = totalRequests == 0
                                                  ? 0.0F
                                                  : static_cast<float>(completedRequests) /
                                                        static_cast<float>(totalRequests);
                ImGui::ProgressBar(prepareProgress, ImVec2{-FLT_MIN, 0.0F});
                ImGui::Text(
                    "Preparing samples %zu / %zu",
                    completedRequests,
                    totalRequests);
                if (!currentLayerName.empty()) {
                    ImGui::TextWrapped("Layer: %s", currentLayerName.c_str());
                }
            } else {
                const float frameProgress = job.frames.empty()
                                                ? 0.0F
                                                : static_cast<float>(job.writtenFrameCount) /
                                                      static_cast<float>(job.frames.size());
                ImGui::ProgressBar(frameProgress, ImVec2{-FLT_MIN, 0.0F});
                ImGui::Text(
                    "Captured %u / %zu, saved %u, queued %zu",
                    std::min<std::uint32_t>(job.currentFrame, static_cast<std::uint32_t>(job.frames.size())),
                    job.frames.size(),
                    std::min<std::uint32_t>(job.writtenFrameCount, static_cast<std::uint32_t>(job.frames.size())),
                    job.pendingFrameCount);
            }
            ImGui::Text("Elapsed: %s", FormatElapsedTime(std::chrono::steady_clock::now() - job.startedAt).c_str());
            if (!job.lastOutputPath.empty()) {
                ImGui::TextWrapped("Last: %s", job.lastOutputPath.string().c_str());
            }
        } else {
            ImGui::TextDisabled("Another export is already running.");
        }
        if (ImGui::Button(job.cancelRequested ? "Cancelling..." : "Cancel Export")) {
            RequestOfflineRenderCancellation(&job);
        }
        EndPanelSection();
        return;
    }

    const bool ffmpegAvailable =
        invisible_places::output::FfmpegExecutableAvailable(invisible_places::output::DefaultFfmpegExecutablePath());
    const bool exportAvailable =
        panel.stillExportMode != invisible_places::output::AnimationExportMode::FastPreviewMp4 ||
        ffmpegAvailable;
    if (!exportAvailable) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("Export Still Camera")) {
        StartStillCameraExportJob(runtimeState, &viewport);
    }
    if (!exportAvailable) {
        ImGui::EndDisabled();
        ImGui::TextDisabled(
            "Preview MP4 requires ffmpeg at %s.",
            invisible_places::output::DefaultFfmpegExecutablePath().string().c_str());
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Exports the current camera without moving it.");
    }

    EndPanelSection();
}

void DrawCameraSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell& viewport) {
    EnsureRuntimeCameraShotIds(runtimeState);
    SyncCurrentAnimationToRegistry(runtimeState);
    auto* session = SelectedLoadedSession(runtimeState);
    if (BeginPanelSection("Camera")) {
        const auto target = runtimeState->camera.Target();
        const auto pivot = runtimeState->camera.OrbitCenter();
        ImGui::Text("Target: %.3f  %.3f  %.3f", target.x, target.y, target.z);
        ImGui::Text("Pivot: %.3f  %.3f  %.3f", pivot.x, pivot.y, pivot.z);
        bool showPivotMarker = runtimeState->pivotOverlay.visible;
        if (ImGui::Checkbox("Show Pivot Marker", &showPivotMarker)) {
            runtimeState->pivotOverlay.visible = showPivotMarker;
            runtimeState->pivotOverlay.pivot = FromGlm(pivot);
            runtimeState->pivotOverlay.lastSetAt =
                showPivotMarker ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        }
        ImGui::Text("Distance: %.3f", runtimeState->camera.Distance());
        ImGui::Text("FOV: %.1f", runtimeState->camera.FovDegrees());
        ImGui::Text(
            "Near/Far: %.4f / %.1f",
            runtimeState->camera.NearPlane(),
            runtimeState->camera.FarPlane());

        if (session == nullptr || !runtimeState->camera.HasFramedBounds()) {
            ImGui::TextUnformatted("Select a loaded layer to frame the camera.");
        } else {
            if (ImGui::Button("Focus Selected Layer")) {
                FocusSessionLayer(runtimeState, viewport, runtimeState->selectedSessionIndex.value());
            }
            ImGui::SameLine();
            if (ImGui::Button("Pivot Center")) {
                SetCameraPivotFromScreenPoint(
                    runtimeState,
                    viewport,
                    CurrentUiViewportCenter(viewport));
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Infer an orbit pivot from visible samples near the screen center.");
            }

            const auto effectiveFrame = ComputeEffectiveLayerFrame(*runtimeState, *session);
            ImGui::Text("Bounds valid: %s", effectiveFrame.bounds.valid ? "yes" : "no");
            if (session->kind == LayerKind::GaussianSplat) {
                ImGui::Text(
                    "Convention: %s",
                    GsplatTransformConventionLabel(runtimeState->projectSettings.gsplatTransformConvention));
            }
        }
        EndPanelSection();
    }

    DrawStillCameraExportSection(runtimeState, viewport);

    if (BeginPanelSection("Shots")) {
    EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());

    InputTextString("Camera Name", &runtimeState->cameraPanel.draftShotName);

    if (ImGui::Button("Save Camera Position")) {
        SaveCurrentCameraShot(runtimeState);
    }
    DrawAssociationFilterControl(
        "Show Cameras",
        *runtimeState,
        &runtimeState->cameraPanel.associationFilterMode,
        &runtimeState->cameraPanel.associationFilterLayerPath);

    if (runtimeState->cameraShots.empty()) {
        ImGui::TextUnformatted("No camera shots saved yet.");
        EndPanelSection();
        return;
    }

    ImGui::Spacing();
    if (ImGui::BeginListBox("Saved Shots", ImVec2{-FLT_MIN, 128.0F})) {
        std::size_t visibleShotCount = 0;
        for (std::size_t index = 0; index < runtimeState->cameraShots.size(); ++index) {
            if (!AssociationMatchesFilter(
                    *runtimeState,
                    runtimeState->cameraShots[index].associatedLayerPaths,
                    runtimeState->cameraPanel.associationFilterMode,
                    runtimeState->cameraPanel.associationFilterLayerPath)) {
                continue;
            }
            ++visibleShotCount;
            const bool selected = runtimeState->cameraPanel.selectedShotIndex.has_value() &&
                                  runtimeState->cameraPanel.selectedShotIndex.value() == index;
            ImGui::PushID(static_cast<int>(index));
            if (runtimeState->cameraPanel.renamingShotIndex.has_value() &&
                runtimeState->cameraPanel.renamingShotIndex.value() == index) {
                if (runtimeState->cameraPanel.focusShotRename) {
                    ImGui::SetKeyboardFocusHere();
                    runtimeState->cameraPanel.focusShotRename = false;
                }
                ImGui::SetNextItemWidth(-FLT_MIN);
                const bool submitted = InputTextStringWithFlags(
                    "##shotRename",
                    &runtimeState->cameraPanel.shotRenameBuffer,
                    ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
                const bool commit = submitted || ImGui::IsItemDeactivatedAfterEdit();
                if (commit) {
                    CommitCameraShotRename(runtimeState, index);
                } else if (ImGui::IsItemDeactivated()) {
                    runtimeState->cameraPanel.renamingShotIndex.reset();
                    runtimeState->cameraPanel.shotRenameBuffer.clear();
                    runtimeState->cameraPanel.focusShotRename = false;
                }
            } else {
                const auto label =
                    runtimeState->cameraShots[index].name + "  [" +
                    FormatAssociationSummary(
                        *runtimeState,
                        runtimeState->cameraShots[index].associatedLayerPaths) +
                    "]";
                if (ImGui::Selectable(label.c_str(), selected)) {
                    runtimeState->cameraPanel.selectedShotIndex = index;
                }
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    BeginCameraShotRename(runtimeState, index);
                }
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        if (visibleShotCount == 0) {
            ImGui::TextDisabled("No camera shots match this filter.");
        }
        ImGui::EndListBox();
    }

    if (runtimeState->cameraPanel.selectedShotIndex.has_value()) {
        const auto selectedIndex = runtimeState->cameraPanel.selectedShotIndex.value();
        if (selectedIndex < runtimeState->cameraShots.size()) {
            if (ImGui::Button("Load Camera")) {
                ApplyCameraShot(runtimeState, selectedIndex);
            }
            ImGui::SameLine();
            if (ImGui::Button("Update From View")) {
                UpdateCameraShotFromCurrentView(runtimeState, selectedIndex);
            }
            ImGui::SameLine();
            if (ImGui::Button("Add To Path")) {
                runtimeState->cameraPanel.pathShotIndices.push_back(selectedIndex);
                runtimeState->cameraPanel.selectedPathItemIndex =
                    runtimeState->cameraPanel.pathShotIndices.size() - 1U;
                runtimeState->cameraPlayback.active = false;
                runtimeState->statusMessage =
                    "Added " + runtimeState->cameraShots[selectedIndex].name + " to the camera path.";
                runtimeState->errorMessage.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("Delete Shot")) {
                const auto links = FindCameraAnimationLinks(*runtimeState, runtimeState->cameraShots[selectedIndex].id);
                if (links.empty() ||
                    (runtimeState->cameraPanel.pendingLinkedShotDeleteIndex.has_value() &&
                     runtimeState->cameraPanel.pendingLinkedShotDeleteIndex.value() == selectedIndex)) {
                    DeleteCameraShot(runtimeState, selectedIndex);
                } else {
                    runtimeState->cameraPanel.pendingLinkedShotDeleteIndex = selectedIndex;
                    runtimeState->statusMessage =
                        "This camera is linked to animations. Click Delete Shot again to unlink keys and delete it.";
                    runtimeState->errorMessage.clear();
                }
            }
            if (selectedIndex < runtimeState->cameraShots.size() && DrawLayerAssociationControls(
                    "Shot Files",
                    runtimeState,
                    &runtimeState->cameraShots[selectedIndex].associatedLayerPaths)) {
                runtimeState->cameraPlayback.active = false;
                runtimeState->statusMessage =
                    "Updated camera shot associations for " +
                    runtimeState->cameraShots[selectedIndex].name + ".";
                runtimeState->errorMessage.clear();
            }
            if (selectedIndex < runtimeState->cameraShots.size()) {
                const auto links = FindCameraAnimationLinks(*runtimeState, runtimeState->cameraShots[selectedIndex].id);
                if (!links.empty()) {
                    ImGui::Spacing();
                    ImGui::TextDisabled("Linked Animations: %zu", links.size());
                    if (links.size() > 1U) {
                        bool allowMultiEdit = IsMultiEditAllowedForCamera(
                            runtimeState->cameraPanel,
                            runtimeState->cameraShots[selectedIndex].id);
                        if (ImGui::Checkbox("Allow Multi-Animation Editing", &allowMultiEdit)) {
                            SetMultiEditAllowedForCamera(
                                &runtimeState->cameraPanel,
                                runtimeState->cameraShots[selectedIndex].id,
                                allowMultiEdit);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Untangle")) {
                            UntangleCameraAnimationLinks(runtimeState, selectedIndex);
                        }
                    }
                    for (std::size_t linkIndex = 0; linkIndex < links.size(); ++linkIndex) {
                        const auto& link = links[linkIndex];
                        ImGui::PushID(static_cast<int>(linkIndex));
                        const auto animationLabel =
                            link.animationName + "  key " + std::to_string(link.keyIndex + 1U);
                        ImGui::TextUnformatted(animationLabel.c_str());
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Unlink")) {
                            UnlinkSingleAnimationKey(runtimeState, link);
                        }
                        ImGui::PopID();
                    }
                }
            }
        }
    }
    EndPanelSection();
    }

    if (!BeginPanelSection("Camera Path")) {
        return;
    }
    int pathDurationFrames = static_cast<int>(runtimeState->cameraPanel.pathDurationFrames);
    if (ImGui::InputInt("Full Duration Frames", &pathDurationFrames)) {
        runtimeState->cameraPanel.pathDurationFrames =
            static_cast<std::uint32_t>(std::max(1, pathDurationFrames));
        runtimeState->cameraPlayback.active = false;
        EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());
    }
    ImGui::TextDisabled(
        "Full path duration: %.2f seconds at the 30 fps path timebase.",
        static_cast<float>(runtimeState->cameraPanel.pathDurationFrames) / 30.0F);

    if (ImGui::Button("Clear Path")) {
        runtimeState->cameraPanel.pathShotIndices.clear();
        runtimeState->cameraPanel.selectedPathItemIndex.reset();
        runtimeState->cameraPlayback.active = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Save Path as Animation")) {
        SaveCurrentCameraPathAsAnimation(runtimeState);
    }

    if (ImGui::BeginListBox("Path Order", ImVec2{-FLT_MIN, 160.0F})) {
        for (std::size_t pathItemIndex = 0; pathItemIndex < runtimeState->cameraPanel.pathShotIndices.size(); ++pathItemIndex) {
            const auto shotIndex = runtimeState->cameraPanel.pathShotIndices[pathItemIndex];
            const bool validShot = shotIndex < runtimeState->cameraShots.size();
            const auto label = std::to_string(pathItemIndex + 1U) +
                               "  " +
                               (validShot ? runtimeState->cameraShots[shotIndex].name : std::string{"Missing Shot"});
            const bool selected = runtimeState->cameraPanel.selectedPathItemIndex.has_value() &&
                                  runtimeState->cameraPanel.selectedPathItemIndex.value() == pathItemIndex;

            ImGui::PushID(static_cast<int>(pathItemIndex));
            if (ImGui::Selectable(label.c_str(), selected)) {
                runtimeState->cameraPanel.selectedPathItemIndex = pathItemIndex;
            }
            if (ImGui::BeginDragDropSource()) {
                const auto payloadIndex = pathItemIndex;
                ImGui::SetDragDropPayload("CAMERA_PATH_ITEM", &payloadIndex, sizeof(payloadIndex));
                ImGui::TextUnformatted(label.c_str());
                ImGui::EndDragDropSource();
            }
            if (ImGui::BeginDragDropTarget()) {
                if (const auto* payload = ImGui::AcceptDragDropPayload("CAMERA_PATH_ITEM");
                    payload != nullptr && payload->DataSize == sizeof(std::size_t)) {
                    const auto sourceIndex = *static_cast<const std::size_t*>(payload->Data);
                    MoveCameraPathItem(
                        &runtimeState->cameraPanel.pathShotIndices,
                        sourceIndex,
                        pathItemIndex);
                    runtimeState->cameraPanel.selectedPathItemIndex =
                        std::min(pathItemIndex, runtimeState->cameraPanel.pathShotIndices.size() - 1U);
                    runtimeState->cameraPlayback.active = false;
                }
                ImGui::EndDragDropTarget();
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }

    if (runtimeState->cameraPanel.selectedPathItemIndex.has_value()) {
        const auto pathItemIndex = runtimeState->cameraPanel.selectedPathItemIndex.value();
        if (pathItemIndex < runtimeState->cameraPanel.pathShotIndices.size()) {
            const auto shotIndex = runtimeState->cameraPanel.pathShotIndices[pathItemIndex];
            if (shotIndex < runtimeState->cameraShots.size()) {
                if (ImGui::Button("Load Path Entry")) {
                    ApplyCameraShot(runtimeState, shotIndex);
                    runtimeState->cameraPanel.selectedPathItemIndex = pathItemIndex;
                }
                ImGui::SameLine();
            }
            if (ImGui::Button("Remove Path Entry")) {
                runtimeState->cameraPanel.pathShotIndices.erase(
                    runtimeState->cameraPanel.pathShotIndices.begin() + static_cast<std::ptrdiff_t>(pathItemIndex));
                if (runtimeState->cameraPanel.pathShotIndices.empty()) {
                    runtimeState->cameraPanel.selectedPathItemIndex.reset();
                } else {
                    runtimeState->cameraPanel.selectedPathItemIndex =
                        std::min(pathItemIndex, runtimeState->cameraPanel.pathShotIndices.size() - 1U);
                }
                runtimeState->cameraPlayback.active = false;
                EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());
            }
        }
    }

    const bool pathReady = runtimeState->cameraPanel.pathShotIndices.size() >= 2U;
    if (!pathReady) {
        ImGui::TextDisabled("Add at least two path entries. Duplicates are allowed.");
    }

    const bool blendChanged =
        DrawRangedFloatControl(
            "Path Position",
            &runtimeState->cameraPanel.blendAmount,
            {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.3f", .hardMin = 0.0F, .hardMax = 1.0F});
    ImGui::Checkbox("Live Apply", &runtimeState->cameraPanel.liveBlend);
    if (runtimeState->cameraPanel.liveBlend && blendChanged) {
        ApplyCameraBlend(runtimeState);
    }
    if (ImGui::Button("Apply Path Position")) {
        ApplyCameraBlend(runtimeState);
    }
    ImGui::SameLine();
    if (ImGui::Button(runtimeState->cameraPlayback.active ? "Stop Playback" : "Play")) {
        if (runtimeState->cameraPlayback.active) {
            runtimeState->cameraPlayback.active = false;
        } else {
            StartCameraPlayback(runtimeState);
        }
    }

    EndPanelSection();
}

void DrawAnimationSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr) {
        return;
    }

    auto& panel = runtimeState->animationPanel;
    EnsureRuntimeCameraShotIds(runtimeState);
    SyncCurrentAnimationToRegistry(runtimeState);
    if (BeginPanelSection("Animation")) {
    if (InputTextString("Animation Name", &panel.draftAnimationName) && panel.currentPath.has_value()) {
        panel.currentPath->name = panel.draftAnimationName;
        panel.dirty = true;
    }
    if (ImGui::Button("Refresh")) {
        RefreshAnimationFileList(&panel, AnimationDirectory(*runtimeState));
    }
    ImGui::SameLine();
    if (ImGui::Button("Import Files")) {
        const auto importedCount = ImportAnimationFilesFromDirectory(&panel, AnimationDirectory(*runtimeState));
        panel.animationRegistryInitialized = true;
        RefreshAnimationFileList(&panel, AnimationDirectory(*runtimeState));
        runtimeState->statusMessage =
            importedCount == 0U
                ? "No new animation files were found to import."
                : "Imported " + std::to_string(importedCount) + " animation file(s).";
        runtimeState->errorMessage.clear();
    }
    ImGui::SameLine();
    if (ImGui::Button("Write Houdini Importer")) {
        WriteHoudiniCameraImporterScript(runtimeState);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Writes a hython script that extracts a HIP camera into camera_exports.");
    }
    ImGui::SameLine();
    if (ImGui::Button("Import Houdini Camera")) {
        ImportLatestHoudiniCameraAnimation(runtimeState);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Loads the newest *_invisible_places_camera.json from the Houdini camera_exports folder.");
    }
    ImGui::SameLine();
    ImGui::Checkbox("Show Splines", &panel.showSplines);
    DrawAssociationFilterControl(
        "Show Animations",
        *runtimeState,
        &panel.associationFilterMode,
        &panel.associationFilterLayerPath);

    if (panel.availableFiles.empty()) {
        ImGui::TextDisabled("No project animation paths are registered.");
    } else if (ImGui::BeginListBox("Saved Animations", ImVec2{-FLT_MIN, 128.0F})) {
        bool animationListChanged = false;
        std::size_t visibleAnimationCount = 0;
        for (std::size_t index = 0; index < panel.availableFiles.size(); ++index) {
            const auto& associatedLayerPaths = AnimationRegistryAssociationPaths(panel, index);
            if (!AssociationMatchesFilter(
                    *runtimeState,
                    associatedLayerPaths,
                    panel.associationFilterMode,
                    panel.associationFilterLayerPath)) {
                continue;
            }
            ++visibleAnimationCount;
            const bool selected = panel.selectedFileIndex.has_value() && panel.selectedFileIndex.value() == index;
            ImGui::PushID(static_cast<int>(index));
            bool selectedForExport = AnimationFileSelectedForExport(panel, panel.availableFiles[index]);
            if (ImGui::Checkbox("##quickMp4Export", &selectedForExport)) {
                SetAnimationFileSelectedForExport(&panel, panel.availableFiles[index], selectedForExport);
            }
            ImGui::SameLine();
            if (panel.renamingFileIndex.has_value() && panel.renamingFileIndex.value() == index) {
                if (panel.focusFileRename) {
                    ImGui::SetKeyboardFocusHere();
                    panel.focusFileRename = false;
                }
                ImGui::SetNextItemWidth(-FLT_MIN);
                const bool submitted = InputTextStringWithFlags(
                    "##animationRename",
                    &panel.fileRenameBuffer,
                    ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
                const bool commit = submitted || ImGui::IsItemDeactivatedAfterEdit();
                if (commit) {
                    animationListChanged = CommitAnimationFileRename(runtimeState, index);
                    ImGui::PopID();
                    break;
                }
                if (ImGui::IsItemDeactivated()) {
                    panel.renamingFileIndex.reset();
                    panel.fileRenameBuffer.clear();
                    panel.focusFileRename = false;
                }
            } else {
                const auto modified =
                    index < panel.availableFileDirtyFlags.size() && panel.availableFileDirtyFlags[index]
                        ? std::string{" *"}
                        : std::string{};
                const auto displayName =
                    AnimationDisplayNameFromPath(panel.availableFiles[index]) + modified + "  [" +
                    FormatAssociationSummary(*runtimeState, associatedLayerPaths) + "]";
                const bool selectedForBatchExport = selectedForExport;
                if (ImGui::Selectable(displayName.c_str(), selectedForBatchExport)) {
                    panel.selectedFileIndex = index;
                    SetAnimationFileSelectedForExport(
                        &panel,
                        panel.availableFiles[index],
                        !selectedForBatchExport);
                }
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    BeginAnimationFileRename(&panel, index);
                }
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
            if (animationListChanged) {
                break;
            }
        }
        if (visibleAnimationCount == 0) {
            ImGui::TextDisabled("No animations match this filter.");
        }
        ImGui::EndListBox();
    }

    const auto selectedQuickMp4Count = panel.selectedExportFiles.size();
    const bool exportButtonDisabled = runtimeState->offlineRenderJob.active;
    if (exportButtonDisabled) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("Export Selected Quick MP4")) {
        StartSelectedQuickMp4Batch(runtimeState, &viewport);
    }
    if (exportButtonDisabled) {
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu selected", selectedQuickMp4Count);

    if (panel.selectedFileIndex.has_value() && panel.selectedFileIndex.value() < panel.availableFiles.size()) {
        if (ImGui::Button("Load")) {
            LoadAnimationPathFromFile(runtimeState, panel.availableFiles[panel.selectedFileIndex.value()]);
        }
        ImGui::SameLine();
        if (ImGui::Button("Remove From Project")) {
            const auto removedPath = panel.availableFiles[panel.selectedFileIndex.value()];
            if (!panel.currentFilePath.empty() &&
                PathsLexicallyEqual(std::filesystem::path{panel.currentFilePath}, removedPath)) {
                panel.currentFilePath.clear();
                panel.currentPath.reset();
                panel.selectedKeyIndex.reset();
            }
            RemoveAnimationFileFromRegistry(&panel, panel.selectedFileIndex.value());
            RefreshAnimationFileList(&panel, AnimationDirectory(*runtimeState));
            runtimeState->statusMessage =
                "Removed animation from project: " + removedPath.filename().string() + ".";
            runtimeState->errorMessage.clear();
        }
    }
    EndPanelSection();
    }

    if (!panel.currentPath.has_value()) {
        ImGui::TextDisabled("Load an animation or save the Camera path as an animation.");
        return;
    }

    auto& animationPath = panel.currentPath.value();
    animationPath.name = panel.draftAnimationName.empty() ? animationPath.name : panel.draftAnimationName;

    if (BeginPanelSection("Current Path")) {
    ImGui::Text("File: %s", panel.currentFilePath.empty() ? "unsaved" : panel.currentFilePath.c_str());
    ImGui::Text("Keys: %zu", animationPath.keys.size());
    if (DrawLayerAssociationControls("Animation Files", runtimeState, &animationPath.associatedLayerPaths)) {
        if (!panel.currentFilePath.empty()) {
            SetAnimationRegistryAssociations(
                &panel,
                std::filesystem::path{panel.currentFilePath},
                animationPath.associatedLayerPaths);
        }
        panel.dirty = true;
    }

    int durationFrames = static_cast<int>(animationPath.durationFrames);
    const auto setDurationFrames = [&](std::uint32_t requestedFrames) {
        const auto minimumFrames = animationPath.keys.size() > 1U
                                       ? static_cast<std::uint32_t>(animationPath.keys.size() - 1U)
                                       : 1U;
        const auto clampedFrames = std::max<std::uint32_t>(minimumFrames, requestedFrames);
        if (animationPath.durationFrames != clampedFrames) {
            animationPath.durationFrames = clampedFrames;
            panel.dirty = true;
            runtimeState->animationPlayback.active = false;
        }
    };
    if (ImGui::InputInt("Full Duration Frames", &durationFrames)) {
        setDurationFrames(static_cast<std::uint32_t>(std::max(1, durationFrames)));
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Stored as 30 fps source frames; export frame rate changes samples, not world speed.");
    }
    float durationSeconds = AnimationDurationSeconds(animationPath);
    if (ImGui::InputFloat("Duration Seconds", &durationSeconds, 0.1F, 1.0F, "%.3f")) {
        const auto requestedFrames =
            static_cast<std::uint32_t>(std::max(1.0F, std::round(std::max(0.001F, durationSeconds) * 30.0F)));
        setDurationFrames(requestedFrames);
    }

    auto motionStats = invisible_places::camera::MeasureAnimationPathMotion(
        animationPath,
        panel.scrubAmount);
    ImGui::TextDisabled(
        "World distance: camera %.3f | target %.3f",
        motionStats.cameraDistance,
        motionStats.targetDistance);

    float cameraAverageSpeed = motionStats.averageCameraSpeed;
    const bool cameraSpeedAvailable = motionStats.cameraDistance > 1.0e-5F;
    if (!cameraSpeedAvailable) {
        ImGui::BeginDisabled();
    }
    if (ImGui::InputFloat("Camera Avg Speed", &cameraAverageSpeed, 0.05F, 0.5F, "%.3f")) {
        if (cameraAverageSpeed > 1.0e-5F) {
            setDurationFrames(invisible_places::camera::AnimationDurationFramesForAverageSpeed(
                animationPath,
                invisible_places::camera::AnimationPathMotionTarget::Camera,
                cameraAverageSpeed));
            motionStats = invisible_places::camera::MeasureAnimationPathMotion(
                animationPath,
                panel.scrubAmount);
        }
    }
    if (!cameraSpeedAvailable) {
        ImGui::EndDisabled();
    }

    float targetAverageSpeed = motionStats.averageTargetSpeed;
    const bool targetSpeedAvailable = motionStats.targetDistance > 1.0e-5F;
    if (!targetSpeedAvailable) {
        ImGui::BeginDisabled();
    }
    if (ImGui::InputFloat("Target Avg Speed", &targetAverageSpeed, 0.05F, 0.5F, "%.3f")) {
        if (targetAverageSpeed > 1.0e-5F) {
            setDurationFrames(invisible_places::camera::AnimationDurationFramesForAverageSpeed(
                animationPath,
                invisible_places::camera::AnimationPathMotionTarget::Target,
                targetAverageSpeed));
            motionStats = invisible_places::camera::MeasureAnimationPathMotion(
                animationPath,
                panel.scrubAmount);
        }
    }
    if (!targetSpeedAvailable) {
        ImGui::EndDisabled();
    }
    ImGui::TextDisabled(
        "Export samples at %u fps: %.3f seconds.",
        std::max<std::uint32_t>(1U, runtimeState->renderSettings.framesPerSecond),
        motionStats.durationSeconds);
    bool depthOfFieldEnabled = animationPath.depthOfFieldEnabled;
    bool depthOfFieldChanged = false;
    bool depthOfFieldPreviewChanged = false;
    if (ImGui::Checkbox("Depth of Field", &depthOfFieldEnabled)) {
        animationPath.depthOfFieldEnabled = depthOfFieldEnabled;
        panel.previewDepthOfField = depthOfFieldEnabled;
        panel.dirty = true;
        depthOfFieldChanged = true;
        depthOfFieldPreviewChanged = true;
    }
    if (!animationPath.depthOfFieldEnabled) {
        panel.previewDepthOfField = false;
    }
    bool previewDepthOfField = panel.previewDepthOfField && animationPath.depthOfFieldEnabled;
    if (!animationPath.depthOfFieldEnabled) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Checkbox("Preview DoF", &previewDepthOfField)) {
        panel.previewDepthOfField = previewDepthOfField;
        depthOfFieldPreviewChanged = true;
    }
    if (!animationPath.depthOfFieldEnabled) {
        ImGui::EndDisabled();
    }
    float apertureFStops = animationPath.apertureFStops;
    if (ImGui::InputFloat("Aperture f-stop", &apertureFStops, 0.1F, 1.0F, "%.2f")) {
        animationPath.apertureFStops = std::max(0.1F, apertureFStops);
        panel.dirty = true;
        depthOfFieldChanged = true;
    }
    float maxBlurPixels = animationPath.depthOfFieldMaxBlurPixels;
    if (DrawRangedFloatControl(
            "Max Blur",
            &maxBlurPixels,
            {.visualMin = 0.0F, .visualMax = 96.0F, .format = "%.1f px", .hardMin = 0.0F})) {
        animationPath.depthOfFieldMaxBlurPixels = std::max(0.0F, maxBlurPixels);
        panel.dirty = true;
        depthOfFieldChanged = true;
    }

    const auto evaluation = invisible_places::camera::EvaluateAnimationPath(
        animationPath,
        AnimationDurationSeconds(animationPath) * std::clamp(panel.scrubAmount, 0.0F, 1.0F));
    ImGui::TextDisabled("Focus distance: %.3f", evaluation.focusDistance);
    if (depthOfFieldPreviewChanged || (depthOfFieldChanged && panel.liveApply)) {
        ApplyAnimationScrub(runtimeState);
    }

    const bool scrubChanged = DrawRangedFloatControl(
        "Animation Position",
        &panel.scrubAmount,
        {.visualMin = 0.0F, .visualMax = 1.0F, .format = "%.3f", .hardMin = 0.0F, .hardMax = 1.0F});
    if (scrubChanged) {
        motionStats = invisible_places::camera::MeasureAnimationPathMotion(
            animationPath,
            panel.scrubAmount);
    }
    ImGui::TextDisabled(
        "At position: camera %.3f/s | target %.3f/s",
        motionStats.currentCameraSpeed,
        motionStats.currentTargetSpeed);
    ImGui::Checkbox("Live Apply", &panel.liveApply);
    if (panel.liveApply && scrubChanged) {
        ApplyAnimationScrub(runtimeState);
    }
    if (ImGui::Button("Apply Position")) {
        ApplyAnimationScrub(runtimeState);
    }
    ImGui::SameLine();
    if (ImGui::Button(runtimeState->animationPlayback.active ? "Stop Playback" : "Play")) {
        if (runtimeState->animationPlayback.active) {
            StopAnimationPlayback(runtimeState);
        } else {
            StartAnimationPlayback(runtimeState);
        }
    }

    if (ImGui::Button("Save")) {
        const auto outputPath = panel.currentFilePath.empty()
                                    ? UniqueAnimationFilePathForName(*runtimeState, animationPath.name)
                                    : std::filesystem::path{panel.currentFilePath};
        auto pathToSave = animationPath;
        if (panel.currentFilePath.empty()) {
            pathToSave.name = AnimationNameFromFilePath(outputPath);
        }
        SaveAnimationPathToFile(runtimeState, pathToSave, outputPath);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save As")) {
        const auto outputPath = UniqueAnimationFilePathForName(*runtimeState, animationPath.name);
        auto pathToSave = animationPath;
        pathToSave.name = AnimationNameFromFilePath(outputPath);
        SaveAnimationPathToFile(runtimeState, pathToSave, outputPath);
    }
    ImGui::SameLine();
    if (ImGui::Button("Export Houdini Camera")) {
        ExportCurrentAnimationCameraToHoudini(runtimeState);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Writes a Houdini Python camera script without launching Houdini.");
    }
    const bool houdiniNoticeVisible =
        panel.showHoudiniCameraExportNotice && !panel.lastHoudiniCameraScriptPath.empty();
    if (houdiniNoticeVisible) {
        const auto scriptPath = panel.lastHoudiniCameraScriptPath.string();
        const auto folderPath = panel.lastHoudiniCameraExportDirectory.empty()
                                    ? panel.lastHoudiniCameraScriptPath.parent_path().string()
                                    : panel.lastHoudiniCameraExportDirectory.string();
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextColored(ImVec4{0.16F, 0.48F, 0.22F, 1.0F}, "Houdini camera export ready");
        ImGui::TextWrapped("Script: %s", scriptPath.c_str());
        ImGui::TextWrapped("Folder: %s", folderPath.c_str());
        if (ImGui::SmallButton("Copy Script Path")) {
            ImGui::SetClipboardText(scriptPath.c_str());
            runtimeState->statusMessage = "Copied Houdini camera script path.";
            runtimeState->errorMessage.clear();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Copy Folder Path")) {
            ImGui::SetClipboardText(folderPath.c_str());
            runtimeState->statusMessage = "Copied Houdini camera export folder path.";
            runtimeState->errorMessage.clear();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Hide")) {
            panel.showHoudiniCameraExportNotice = false;
        }
    }
    if (panel.dirty) {
        if (!houdiniNoticeVisible) {
            ImGui::SameLine();
        }
        ImGui::TextDisabled("Modified");
    }
    EndPanelSection();
    }

    DrawAnimationExportSection(runtimeState, &viewport);

    if (animationPath.keys.empty()) {
        return;
    }

    if (!BeginPanelSection("Keys")) {
        return;
    }
    const char* editTargetLabels[] = {"Camera", "Focus"};
    int editTargetIndex = panel.editTarget == AnimationEditTarget::Camera ? 0 : 1;
    if (ImGui::Combo("Edit Target", &editTargetIndex, editTargetLabels, 2)) {
        panel.editTarget = editTargetIndex == 0 ? AnimationEditTarget::Camera : AnimationEditTarget::Focus;
    }

    if (ImGui::BeginListBox("Animation Keys", ImVec2{-FLT_MIN, 110.0F})) {
        for (std::size_t index = 0; index < animationPath.keys.size(); ++index) {
            const auto& listKey = animationPath.keys[index];
            const auto keyCameraLabel =
                !listKey.linkedCameraName.empty()
                    ? listKey.linkedCameraName
                    : (!listKey.linkedCameraId.empty() ? std::string{"Linked Camera"} : std::string{"Unlinked"});
            const auto label = std::to_string(index + 1U) + "  " + keyCameraLabel;
            const bool selected = panel.selectedKeyIndex.has_value() && panel.selectedKeyIndex.value() == index;
            if (ImGui::Selectable(label.c_str(), selected)) {
                panel.selectedKeyIndex = index;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndListBox();
    }

    if (!panel.selectedKeyIndex.has_value() || panel.selectedKeyIndex.value() >= animationPath.keys.size()) {
        EndPanelSection();
        return;
    }

    auto& key = animationPath.keys[panel.selectedKeyIndex.value()];
    if (!key.linkedCameraId.empty()) {
        const auto links = FindCameraAnimationLinks(*runtimeState, key.linkedCameraId);
        const auto linkedName = key.linkedCameraName.empty() ? std::string{"Linked Camera"} : key.linkedCameraName;
        ImGui::TextDisabled("Camera: %s", linkedName.c_str());
        if (links.size() > 1U) {
            bool allowMultiEdit = IsMultiEditAllowedForCamera(runtimeState->cameraPanel, key.linkedCameraId);
            if (ImGui::Checkbox("Allow Multi-Animation Editing", &allowMultiEdit)) {
                SetMultiEditAllowedForCamera(&runtimeState->cameraPanel, key.linkedCameraId, allowMultiEdit);
            }
        }
    } else {
        ImGui::TextDisabled("Camera: Unlinked");
    }
    float cameraPosition[3] = {key.cameraPosition[0], key.cameraPosition[1], key.cameraPosition[2]};
    if (ImGui::InputFloat3("Camera Key", cameraPosition, "%.3f")) {
        if (MoveLinkedAnimationKeyPoint(
                runtimeState,
                &animationPath,
                panel.selectedKeyIndex.value(),
                AnimationEditTarget::Camera,
                {cameraPosition[0], cameraPosition[1], cameraPosition[2]})) {
            panel.dirty = true;
        }
    }
    float focusPoint[3] = {key.focusPoint[0], key.focusPoint[1], key.focusPoint[2]};
    if (ImGui::InputFloat3("Focus Key", focusPoint, "%.3f")) {
        if (MoveLinkedAnimationKeyPoint(
                runtimeState,
                &animationPath,
                panel.selectedKeyIndex.value(),
                AnimationEditTarget::Focus,
                {focusPoint[0], focusPoint[1], focusPoint[2]})) {
            panel.dirty = true;
        }
    }
    EndPanelSection();
}

void DrawSettingsSection(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (!BeginPanelSection("gSplat Settings")) {
        return;
    }

    auto& settings = runtimeState->projectSettings;
    ImGui::Checkbox(
        "Auto Lower gSplat Quality While Interacting",
        &settings.autoLowerGsplatQualityWhileNavigating);
    DrawRangedFloatControl(
        "gSplat Footprint Boost",
        &settings.gaussianSplatFootprintBoost,
        {.visualMin = 0.35F, .visualMax = 3.0F, .format = "%.2f", .hardMin = 0.001F});

    int conventionIndex = static_cast<int>(settings.gsplatTransformConvention);
    const char* transformConventions[] = {"As Encoded", "Swap Y/Z", "Swap Y/Z + Flip X"};
    if (ImGui::Combo(
            "gSplat Transform",
            &conventionIndex,
            transformConventions,
            IM_ARRAYSIZE(transformConventions))) {
        settings.gsplatTransformConvention = static_cast<GsplatTransformConvention>(conventionIndex);
        runtimeState->statusMessage =
            "Updated gSplat transform convention to " +
            std::string{GsplatTransformConventionLabel(settings.gsplatTransformConvention)} + ".";
        runtimeState->errorMessage.clear();

        if (runtimeState->selectedSessionIndex.has_value()) {
            const auto selectedIndex = runtimeState->selectedSessionIndex.value();
            if (runtimeState->sessions[selectedIndex].kind == LayerKind::GaussianSplat &&
                runtimeState->sessions[selectedIndex].loaded) {
                FocusSessionLayer(runtimeState, viewport, selectedIndex);
            }
        }
    }

    ImGui::TextWrapped(
        "Polycam gSplat exports in this dataset advertise Z-up in the PLY header. "
        "CloudCompare preserves coordinates as stored, so the default stays As Encoded.");
    ImGui::TextDisabled(
        "Per-splat opacity and scale are already decoded from the file. "
        "The layer sliders and footprint boost multiply those stored values.");

    if (const auto* session = SelectedLoadedSession(runtimeState);
        session != nullptr && session->kind == LayerKind::GaussianSplat) {
        const auto rawMatrix = ToGlmMatrix(session->localToWorld);
        const auto effectiveMatrix = EffectiveGsplatLocalToWorld(settings, *session);

        ImGui::Spacing();
        ImGui::Text("Selected gSplat Matrix");
        ImGui::Text(
            "Raw determinant: %.4f",
            MatrixDeterminant3x3(rawMatrix));
        ImGui::Text(
            "Effective determinant: %.4f",
            MatrixDeterminant3x3(effectiveMatrix));
        ImGui::Text(
            "Translation: %.3f  %.3f  %.3f",
            rawMatrix[3][0],
            rawMatrix[3][1],
            rawMatrix[3][2]);
        ImGui::Text(
            "Row 0: %.3f  %.3f  %.3f  %.3f",
            session->localToWorld.At(0, 0),
            session->localToWorld.At(0, 1),
            session->localToWorld.At(0, 2),
            session->localToWorld.At(0, 3));
        ImGui::Text(
            "Row 1: %.3f  %.3f  %.3f  %.3f",
            session->localToWorld.At(1, 0),
            session->localToWorld.At(1, 1),
            session->localToWorld.At(1, 2),
            session->localToWorld.At(1, 3));
        ImGui::Text(
            "Row 2: %.3f  %.3f  %.3f  %.3f",
            session->localToWorld.At(2, 0),
            session->localToWorld.At(2, 1),
            session->localToWorld.At(2, 2),
            session->localToWorld.At(2, 3));
        ImGui::TextDisabled("A negative determinant indicates a handedness flip.");
    }
    EndPanelSection();
}

void DrawProjectSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (!BeginPanelSection("Project")) {
        return;
    }

    InputTextString("Project File", &runtimeState->persistence.projectFilePath);
    if (ImGui::Button("Save Project")) {
        const auto document = BuildProjectDocument(*runtimeState);
        std::string errorMessage;
        if (invisible_places::serialization::SaveProjectDocument(
                document,
                runtimeState->persistence.projectFilePath,
                &errorMessage)) {
            runtimeState->statusMessage = "Saved project to " + runtimeState->persistence.projectFilePath + ".";
            runtimeState->errorMessage.clear();
        } else {
            runtimeState->errorMessage = errorMessage;
            runtimeState->statusMessage.clear();
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Load Project")) {
        std::string errorMessage;
        const auto document = invisible_places::serialization::LoadProjectDocument(
            runtimeState->persistence.projectFilePath,
            &errorMessage);
        if (!document.has_value()) {
            runtimeState->errorMessage = errorMessage;
            runtimeState->statusMessage.clear();
        } else if (!ApplyProjectDocumentToRuntime(document.value(), runtimeState, viewport)) {
            if (runtimeState->errorMessage.empty() && runtimeState->statusMessage.empty()) {
                runtimeState->statusMessage = "Project load was cancelled.";
            }
        }
    }

    if (runtimeState->pendingLoad.has_value()) {
        ImGui::TextDisabled("Project load waits for the current background layer load to finish.");
    }
    EndPanelSection();
}

void DrawPresetSection(PreviewRuntimeState* runtimeState, PreviewLayerSession* session) {
    if (!BeginPanelSection("Presets")) {
        return;
    }

    InputTextString("Point Style Preset", &runtimeState->persistence.pointStylePresetPath);

    if (session == nullptr || session->kind != LayerKind::PointCloud) {
        ImGui::TextUnformatted("Select a point cloud layer to save or load a point style preset.");
        EndPanelSection();
        return;
    }

    if (ImGui::Button("Save Point Style")) {
        PointCloudStylePresetDocument presetDocument;
        presetDocument.presetName = session->displayName + " Style";
        presetDocument.style = session->pointStyle;

        std::string errorMessage;
        if (invisible_places::serialization::SavePointCloudStylePreset(
                presetDocument,
                runtimeState->persistence.pointStylePresetPath,
                &errorMessage)) {
            runtimeState->statusMessage =
                "Saved point style preset to " + runtimeState->persistence.pointStylePresetPath + ".";
            runtimeState->errorMessage.clear();
        } else {
            runtimeState->errorMessage = errorMessage;
            runtimeState->statusMessage.clear();
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Load Point Style")) {
        std::string errorMessage;
        const auto presetDocument = invisible_places::serialization::LoadPointCloudStylePreset(
            runtimeState->persistence.pointStylePresetPath,
            &errorMessage);
        if (!presetDocument.has_value()) {
            runtimeState->errorMessage = errorMessage;
            runtimeState->statusMessage.clear();
        } else {
            session->pointStyle = presetDocument->style;
            SanitizePointCloudStyle(session);
            MarkPointVisualEdited(session);
            SyncWaterPointVisualSelectionFromSession(runtimeState, *session);
            runtimeState->statusMessage =
                "Loaded point style preset from " + runtimeState->persistence.pointStylePresetPath + ".";
            runtimeState->errorMessage.clear();
        }
    }
    EndPanelSection();
}

PreviewLayerSession* DrawLoadedPointCloudLookdevSelector(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return nullptr;
    }

    auto visualIndex = ResolveLoadedPointCloudLookdevIndex(*runtimeState);
    if (!visualIndex.has_value()) {
        return nullptr;
    }

    const auto currentIndex = visualIndex.value();
    const std::string currentLabel =
        runtimeState->sessions[currentIndex].displayName +
        (runtimeState->sessions[currentIndex].visible ? "" : " [hidden]");
    if (ImGui::BeginCombo("Lidar Cloud", currentLabel.c_str())) {
        for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
            auto& session = runtimeState->sessions[index];
            if (!IsLoadedPointCloudSession(session)) {
                continue;
            }

            const bool selected = visualIndex.has_value() && visualIndex.value() == index;
            const std::string label = session.displayName + (session.visible ? "" : " [hidden]");
            if (ImGui::Selectable(label.c_str(), selected)) {
                runtimeState->selectedSessionIndex = index;
                visualIndex = index;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    return visualIndex.has_value() ? &runtimeState->sessions[visualIndex.value()] : nullptr;
}

void DrawSavedPointVisualSelector(PreviewRuntimeState* runtimeState, PreviewLayerSession* session) {
    if (runtimeState == nullptr || session == nullptr || session->kind != LayerKind::PointCloud) {
        return;
    }

    EnsurePointVisuals(session);
    const auto selectedName = NormalizePointVisualName(session->selectedPointVisualName);
    if (ImGui::BeginCombo("Saved Visuals", selectedName.c_str())) {
        for (const auto& visual : session->pointVisuals) {
            const std::string visualName = visual.name;
            const bool selected = visualName == selectedName;
            if (ImGui::Selectable(visualName.c_str(), selected)) {
                SelectPointVisual(session, visualName);
                SyncWaterPointVisualSelectionFromSession(runtimeState, *session);
                runtimeState->statusMessage = "Loaded visual " + visualName + ".";
                runtimeState->errorMessage.clear();
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    InputTextString("Visual Name", &session->pointVisualNameBuffer);
    if (ImGui::Button("Save Visual")) {
        SaveCurrentPointVisual(runtimeState, session);
    }
    if (IsEditedPointVisualName(session->selectedPointVisualName)) {
        ImGui::SameLine();
        ImGui::TextDisabled("Editing %s", BasePointVisualName(session->selectedPointVisualName).c_str());
    } else if (
        IsGeneratedWaterFlowOverlaySession(*session) &&
        IsProtectedWaterPointVisualName(session->selectedPointVisualName)) {
        ImGui::SameLine();
        ImGui::TextDisabled("Preset");
    }
}

void DrawLidarPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    DrawLayerSection(runtimeState, viewport, LayerKind::PointCloud, "Lidar Layers");
}

void DrawVisualsPanel(PreviewRuntimeState* runtimeState) {
    const auto visualIndex = runtimeState != nullptr ? ResolveLoadedPointCloudLookdevIndex(*runtimeState) : std::nullopt;
    PreviewLayerSession* session =
        visualIndex.has_value() ? &runtimeState->sessions[visualIndex.value()] : nullptr;
    if (BeginPanelSection("Cloud Visuals")) {
        session = DrawLoadedPointCloudLookdevSelector(runtimeState);
        DrawSavedPointVisualSelector(runtimeState, session);
        EndPanelSection();
    }
    if (session == nullptr) {
        ImGui::TextUnformatted("Load a lidar layer to edit its visual settings.");
        return;
    }

    if (DrawPointCloudStyleSection(session)) {
        MarkPointVisualEdited(session);
        SyncWaterPointVisualSelectionFromSession(runtimeState, *session);
    }
    DrawPresetSection(runtimeState, session);
}

void DrawGsplatPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    DrawLayerSection(runtimeState, viewport, LayerKind::GaussianSplat, "gSplat Layers");
    ImGui::Spacing();
    if (BeginPanelSection("gSplat Visuals")) {
        if (auto* session = SelectedLoadedSessionOfKind(runtimeState, LayerKind::GaussianSplat); session != nullptr) {
        const auto effectiveQuality = EffectiveGaussianSplatQualityMode(*runtimeState, *session);
        if (effectiveQuality != session->gsplatStyle.qualityMode) {
            ImGui::TextDisabled(
                "Rendering as %s while navigating.",
                GaussianSplatQualityModeLabel(effectiveQuality));
        }
        DrawGaussianSplatStyleSection(session);
        } else {
        ImGui::TextUnformatted("Select and load a gSplat layer to edit its visual settings.");
        }
        EndPanelSection();
    }

    DrawSettingsSection(runtimeState, *viewport);
}

const char* WaterRunoffModeLabel(WaterRunoffMode mode) {
    switch (mode) {
        case WaterRunoffMode::LightRain:
            return "Light Rain";
        case WaterRunoffMode::Dew:
            return "Dew";
    }
    return "Dew";
}

const char* WaterCausticPreviewTintModeLabel(WaterCausticPreviewTintMode mode) {
    switch (mode) {
        case WaterCausticPreviewTintMode::PulseAfterRefresh:
            return "Pulse";
        case WaterCausticPreviewTintMode::Always:
            return "Always";
        case WaterCausticPreviewTintMode::Off:
            return "Off";
    }
    return "Pulse";
}

void DrawWaterBasinPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return;
    }
    auto& water = runtimeState->water;
    if (BeginPanelSection("Basin Regions")) {
        if (ImGui::Button("New Basin Region")) {
            WaterBasinRegion region;
            region.id = NextWaterBasinRegionId(*runtimeState);
            region.name = "Basin " + std::to_string(region.id);
            water.nextBasinRegionId = region.id + 1U;
            water.basinRegions.push_back(std::move(region));
            water.selectedBasinRegionIndex = water.basinRegions.size() - 1U;
            water.basinRegionPlacementArmed = true;
            water.runoffRegionPlacementArmed = false;
            water.causticRegionPlacementArmed = false;
            water.placementArmed = false;
            water.movingEmitterIndex.reset();
            runtimeState->statusMessage = "Click LiDAR points to add basin polygon vertices.";
            runtimeState->errorMessage.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button(water.basinRegionPlacementArmed ? "Stop Adding Vertices" : "Add Vertices")) {
            water.basinRegionPlacementArmed = !water.basinRegionPlacementArmed;
            water.runoffRegionPlacementArmed = false;
            water.causticRegionPlacementArmed = false;
            runtimeState->statusMessage = water.basinRegionPlacementArmed
                                              ? "Click LiDAR points to add basin polygon vertices."
                                              : "Basin vertex placement stopped.";
            runtimeState->errorMessage.clear();
        }

        if (water.basinRegions.empty()) {
            ImGui::TextUnformatted("No basin regions yet.");
        } else {
            const char* currentLabel =
                water.selectedBasinRegionIndex.has_value() &&
                        water.selectedBasinRegionIndex.value() < water.basinRegions.size()
                    ? water.basinRegions[water.selectedBasinRegionIndex.value()].name.c_str()
                    : "Select basin";
            if (ImGui::BeginCombo("Region", currentLabel)) {
                for (std::size_t index = 0; index < water.basinRegions.size(); ++index) {
                    const bool selected =
                        water.selectedBasinRegionIndex.has_value() &&
                        water.selectedBasinRegionIndex.value() == index;
                    if (ImGui::Selectable(water.basinRegions[index].name.c_str(), selected)) {
                        water.selectedBasinRegionIndex = index;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            if (water.selectedBasinRegionIndex.has_value() &&
                water.selectedBasinRegionIndex.value() < water.basinRegions.size()) {
                auto& region = water.basinRegions[water.selectedBasinRegionIndex.value()];
                InputTextString("Name", &region.name);
                ImGui::TextDisabled(
                    "Vertices: %s, hull: %s",
                    FormatPointCount(region.vertices.size()).c_str(),
                    FormatPointCount(region.hull.size()).c_str());
                if (ImGui::Button("Close Basin Region")) {
                    invisible_places::water::RefreshWaterBasinRegionDerivedValues(&region);
                    water.basinRegionPlacementArmed = false;
                    runtimeState->statusMessage =
                        region.hull.size() >= 3U ? "Basin region closed." : "Add at least three basin vertices.";
                    runtimeState->errorMessage.clear();
                }
                ImGui::SameLine();
                if (ImGui::Button("Remove Last Vertex") && !region.vertices.empty()) {
                    region.vertices.pop_back();
                    invisible_places::water::RefreshWaterBasinRegionDerivedValues(&region);
                }
                bool changed = false;
                changed |= ImGui::SliderFloat("Height Above", &region.heightAbove, 0.0F, 5.0F, "%.2f m");
                changed |= ImGui::SliderFloat("Depth Below", &region.depthBelow, 0.0F, 5.0F, "%.2f m");
                changed |= ImGui::SliderFloat("Density", &region.density, 0.01F, 6.0F, "%.2f", ImGuiSliderFlags_Logarithmic);
                if (region.hull.size() >= 2U) {
                    bool hasOutlet = region.outletEdgeIndex.has_value();
                    if (ImGui::Checkbox("Outlet Edge", &hasOutlet)) {
                        region.outletEdgeIndex = hasOutlet ? std::optional<std::uint32_t>{0U} : std::nullopt;
                    }
                    if (region.outletEdgeIndex.has_value()) {
                        int outletIndex = static_cast<int>(region.outletEdgeIndex.value());
                        if (ImGui::SliderInt(
                                "Outlet Edge Index",
                                &outletIndex,
                                0,
                                static_cast<int>(region.hull.size() - 1U))) {
                            region.outletEdgeIndex = static_cast<std::uint32_t>(outletIndex);
                        }
                        ImGui::Checkbox("Outlet Blocked", &region.outletBlocked);
                    }
                }
                if (changed) {
                    invisible_places::water::RefreshWaterBasinRegionDerivedValues(&region);
                }
                if (ImGui::Button("Bake Basin Haze")) {
                    BakeBasinHazeOverlayForActiveLayer(runtimeState, viewport);
                }
                ImGui::SameLine();
                if (ImGui::Button("Delete Basin")) {
                    water.basinRegions.erase(
                        water.basinRegions.begin() +
                        static_cast<std::ptrdiff_t>(water.selectedBasinRegionIndex.value()));
                    water.selectedBasinRegionIndex.reset();
                    water.basinRegionPlacementArmed = false;
                }
            }
        }
        if (!water.lastBasinOverlayPath.empty()) {
            ImGui::TextDisabled("Last overlay: %s", water.lastBasinOverlayPath.filename().string().c_str());
        }
        EndPanelSection();
    }
}

void DrawWaterRunoffPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return;
    }
    auto& water = runtimeState->water;
    if (BeginPanelSection("Runoff Regions")) {
        if (ImGui::Button("New Runoff Region")) {
            WaterRunoffRegion region;
            region.id = NextWaterRunoffRegionId(*runtimeState);
            region.name = "Runoff " + std::to_string(region.id);
            water.nextRunoffRegionId = region.id + 1U;
            water.runoffRegions.push_back(std::move(region));
            water.selectedRunoffRegionIndex = water.runoffRegions.size() - 1U;
            water.runoffRegionPlacementArmed = true;
            water.basinRegionPlacementArmed = false;
            water.causticRegionPlacementArmed = false;
            water.placementArmed = false;
            water.movingEmitterIndex.reset();
            runtimeState->statusMessage = "Click LiDAR points to add runoff polygon vertices.";
            runtimeState->errorMessage.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button(water.runoffRegionPlacementArmed ? "Stop Adding Vertices" : "Add Vertices")) {
            water.runoffRegionPlacementArmed = !water.runoffRegionPlacementArmed;
            water.basinRegionPlacementArmed = false;
            water.causticRegionPlacementArmed = false;
            runtimeState->statusMessage = water.runoffRegionPlacementArmed
                                              ? "Click LiDAR points to add runoff polygon vertices."
                                              : "Runoff vertex placement stopped.";
            runtimeState->errorMessage.clear();
        }

        if (water.runoffRegions.empty()) {
            ImGui::TextUnformatted("No runoff regions yet.");
        } else {
            const char* currentLabel =
                water.selectedRunoffRegionIndex.has_value() &&
                        water.selectedRunoffRegionIndex.value() < water.runoffRegions.size()
                    ? water.runoffRegions[water.selectedRunoffRegionIndex.value()].name.c_str()
                    : "Select runoff";
            if (ImGui::BeginCombo("Region", currentLabel)) {
                for (std::size_t index = 0; index < water.runoffRegions.size(); ++index) {
                    const bool selected =
                        water.selectedRunoffRegionIndex.has_value() &&
                        water.selectedRunoffRegionIndex.value() == index;
                    if (ImGui::Selectable(water.runoffRegions[index].name.c_str(), selected)) {
                        water.selectedRunoffRegionIndex = index;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            if (water.selectedRunoffRegionIndex.has_value() &&
                water.selectedRunoffRegionIndex.value() < water.runoffRegions.size()) {
                auto& region = water.runoffRegions[water.selectedRunoffRegionIndex.value()];
                InputTextString("Name", &region.name);
                ImGui::TextDisabled(
                    "Vertices: %s, hull: %s",
                    FormatPointCount(region.vertices.size()).c_str(),
                    FormatPointCount(region.hull.size()).c_str());
                int modeIndex = region.mode == WaterRunoffMode::LightRain ? 1 : 0;
                const char* modes[] = {"Dew", "Light Rain"};
                if (ImGui::Combo("Mode", &modeIndex, modes, IM_ARRAYSIZE(modes))) {
                    region.mode = modeIndex == 1 ? WaterRunoffMode::LightRain : WaterRunoffMode::Dew;
                }
                if (ImGui::Button("Close Runoff Region")) {
                    invisible_places::water::RefreshWaterRunoffRegionDerivedValues(&region);
                    water.runoffRegionPlacementArmed = false;
                    runtimeState->statusMessage =
                        region.hull.size() >= 3U ? "Runoff region closed." : "Add at least three runoff vertices.";
                    runtimeState->errorMessage.clear();
                }
                ImGui::SameLine();
                if (ImGui::Button("Remove Last Vertex") && !region.vertices.empty()) {
                    region.vertices.pop_back();
                    invisible_places::water::RefreshWaterRunoffRegionDerivedValues(&region);
                }
                bool changed = false;
                changed |= ImGui::SliderFloat("Ground Voxel", &region.groundVoxelSize, 0.03F, 4.0F, "%.2f m", ImGuiSliderFlags_Logarithmic);
                changed |= ImGui::SliderFloat("High Point Fraction", &region.highPointFraction, 0.01F, 0.95F, "%.2f");
                changed |= ImGui::SliderFloat("Density", &region.density, 0.01F, 6.0F, "%.2f", ImGuiSliderFlags_Logarithmic);
                changed |= ImGui::SliderFloat("Path Length", &region.pathLength, 0.5F, 120.0F, "%.1f m", ImGuiSliderFlags_Logarithmic);
                changed |= ImGui::SliderFloat("Max Steps", &region.maxSteps, 4.0F, 250.0F, "%.0f");
                if (changed) {
                    invisible_places::water::RefreshWaterRunoffRegionDerivedValues(&region);
                }
                if (ImGui::Button("Bake Runoff")) {
                    BakeRunoffOverlayForActiveLayer(runtimeState, viewport);
                }
                ImGui::SameLine();
                if (ImGui::Button("Delete Runoff")) {
                    water.runoffRegions.erase(
                        water.runoffRegions.begin() +
                        static_cast<std::ptrdiff_t>(water.selectedRunoffRegionIndex.value()));
                    water.selectedRunoffRegionIndex.reset();
                    water.runoffRegionPlacementArmed = false;
                }
            }
        }
        if (!water.lastRunoffOverlayPath.empty()) {
            ImGui::TextDisabled("Last overlay: %s", water.lastRunoffOverlayPath.filename().string().c_str());
        }
        EndPanelSection();
    }
}

void DrawWaterCausticsPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return;
    }
    auto& water = runtimeState->water;
    if (BeginPanelSection("Caustic Regions")) {
        if (ImGui::Button("New Caustic Area")) {
            const auto targetPath = SelectedCausticTargetLayerPath(*runtimeState);
            if (!targetPath.has_value()) {
                runtimeState->errorMessage = "Select a loaded LiDAR layer before creating caustics.";
                runtimeState->statusMessage.clear();
            } else {
                WaterCausticRegion region;
                region.id = NextWaterCausticRegionId(*runtimeState);
                region.name = "Caustics " + std::to_string(region.id);
                region.targetLayerSourcePath = targetPath.value();
                water.nextCausticRegionId = region.id + 1U;
                water.causticRegions.push_back(std::move(region));
                water.selectedCausticRegionIndex = water.causticRegions.size() - 1U;
                water.causticRegionPlacementArmed = true;
                water.basinRegionPlacementArmed = false;
                water.runoffRegionPlacementArmed = false;
                water.placementArmed = false;
                water.movingEmitterIndex.reset();
                runtimeState->statusMessage = "Click LiDAR points to add caustic boundary vertices.";
                runtimeState->errorMessage.clear();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(water.causticRegionPlacementArmed ? "Stop Adding Vertices" : "Add Vertices")) {
            water.causticRegionPlacementArmed = !water.causticRegionPlacementArmed;
            water.basinRegionPlacementArmed = false;
            water.runoffRegionPlacementArmed = false;
            water.placementArmed = false;
            runtimeState->statusMessage = water.causticRegionPlacementArmed
                                              ? "Click LiDAR points to add caustic boundary vertices."
                                              : "Caustic vertex placement stopped.";
            runtimeState->errorMessage.clear();
        }

        if (water.causticRegions.empty()) {
            ImGui::TextUnformatted("No caustic regions yet.");
        } else {
            const char* currentLabel =
                water.selectedCausticRegionIndex.has_value() &&
                        water.selectedCausticRegionIndex.value() < water.causticRegions.size()
                    ? water.causticRegions[water.selectedCausticRegionIndex.value()].name.c_str()
                    : "Select caustics";
            if (ImGui::BeginCombo("Region", currentLabel)) {
                for (std::size_t index = 0; index < water.causticRegions.size(); ++index) {
                    const bool selected =
                        water.selectedCausticRegionIndex.has_value() &&
                        water.selectedCausticRegionIndex.value() == index;
                    if (ImGui::Selectable(water.causticRegions[index].name.c_str(), selected)) {
                        water.selectedCausticRegionIndex = index;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            if (water.selectedCausticRegionIndex.has_value() &&
                water.selectedCausticRegionIndex.value() < water.causticRegions.size()) {
                auto& region = water.causticRegions[water.selectedCausticRegionIndex.value()];
                InputTextString("Name", &region.name);
                const auto targetIndex = FindSessionIndexBySourcePath(*runtimeState, region.targetLayerSourcePath);
                const std::string targetLabel =
                    targetIndex.has_value() && targetIndex.value() < runtimeState->sessions.size()
                        ? runtimeState->sessions[targetIndex.value()].displayName
                        : region.targetLayerSourcePath.filename().string();
                ImGui::TextDisabled("Target: %s", targetLabel.empty() ? "Missing layer" : targetLabel.c_str());
                ImGui::TextDisabled(
                    "Boundary vertices: %s%s",
                    FormatPointCount(region.vertices.size()).c_str(),
                    (region.maskDirty || region.maskStale) ? "  | mask stale" : "");

                bool enabled = region.enabled;
                if (ImGui::Checkbox("Enabled", &enabled)) {
                    region.enabled = enabled;
                    region.maskDirty = true;
                    region.maskStale = true;
                }
                if (ImGui::Button("Close Caustic Area")) {
                    invisible_places::water::RefreshWaterCausticRegionDerivedValues(&region);
                    water.causticRegionPlacementArmed = false;
                    runtimeState->statusMessage =
                        region.vertices.size() >= 3U ? "Caustic boundary closed; refresh selection mask to apply."
                                                     : "Add at least three caustic boundary vertices.";
                    runtimeState->errorMessage.clear();
                }
                ImGui::SameLine();
                if (ImGui::Button("Remove Last Vertex") && !region.vertices.empty()) {
                    region.vertices.pop_back();
                    invisible_places::water::RefreshWaterCausticRegionDerivedValues(&region);
                }
                bool changed = false;
                changed |= ImGui::SliderFloat("Edge Blend", &region.edgeBlendWidth, 0.01F, 5.0F, "%.2f m");
                const std::array<WaterCausticPreviewTintMode, 3> tintModes{
                    WaterCausticPreviewTintMode::PulseAfterRefresh,
                    WaterCausticPreviewTintMode::Always,
                    WaterCausticPreviewTintMode::Off};
                int tintModeIndex = 0;
                for (std::size_t modeIndex = 0; modeIndex < tintModes.size(); ++modeIndex) {
                    if (region.previewTintMode == tintModes[modeIndex]) {
                        tintModeIndex = static_cast<int>(modeIndex);
                        break;
                    }
                }
                if (ImGui::BeginCombo(
                        "Mask Tint",
                        WaterCausticPreviewTintModeLabel(region.previewTintMode))) {
                    for (std::size_t modeIndex = 0; modeIndex < tintModes.size(); ++modeIndex) {
                        const bool selected = tintModeIndex == static_cast<int>(modeIndex);
                        if (ImGui::Selectable(
                                WaterCausticPreviewTintModeLabel(tintModes[modeIndex]),
                                selected)) {
                            region.previewTintMode = tintModes[modeIndex];
                            tintModeIndex = static_cast<int>(modeIndex);
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                if (changed) {
                    invisible_places::water::RefreshWaterCausticRegionDerivedValues(&region);
                }
                if (ImGui::Button("Refresh Mask")) {
                    RefreshSelectedWaterCausticMask(runtimeState, viewport);
                }
                ImGui::SameLine();
                if (ImGui::Button("Delete Caustics")) {
                    water.causticPreviewTintPulseUntilSeconds.erase(region.id);
                    water.causticRegions.erase(
                        water.causticRegions.begin() +
                        static_cast<std::ptrdiff_t>(water.selectedCausticRegionIndex.value()));
                    water.selectedCausticRegionIndex.reset();
                    water.causticRegionPlacementArmed = false;
                }
            }
        }
        EndPanelSection();
    }

    if (BeginPanelSection("Caustic Look")) {
        auto look = ViewedWaterCausticLookSettings(*runtimeState);
        const auto* animationPath = CurrentAnimationPath(*runtimeState);
        const char* profileLabel = animationPath != nullptr
                                       ? (animationPath->tempWaterCausticLookSettings.has_value()
                                              ? "Animation Temp"
                                              : (animationPath->waterCausticLookSettings.has_value()
                                                     ? "Animation Saved"
                                                     : "Project Default"))
                                       : (water.tempDefaultCausticLookSettings.has_value()
                                              ? "Project Temp Default"
                                              : "Project Default");
        ImGui::TextDisabled("Editing: %s", profileLabel);

        bool changed = false;
        changed |= ImGui::Checkbox("Enable Caustics", &look.enabled);
        changed |= ImGui::SliderFloat("Intensity", &look.intensity, 0.0F, 2.5F, "%.2f");
        changed |= ImGui::SliderFloat("Speed", &look.speed, 0.0F, 3.0F, "%.2f");
        changed |= ImGui::SliderFloat(
            "Cell Size",
            &look.cellSizeMeters,
            0.04F,
            0.80F,
            "%.3f m",
            ImGuiSliderFlags_Logarithmic);
        changed |= ImGui::SliderFloat(
            "Line Width",
            &look.lineWidthMeters,
            0.003F,
            0.050F,
            "%.3f m",
            ImGuiSliderFlags_Logarithmic);
        changed |= ImGui::SliderFloat(
            "Feather",
            &look.featherMeters,
            0.001F,
            0.030F,
            "%.3f m",
            ImGuiSliderFlags_Logarithmic);
        changed |= ImGui::SliderFloat(
            "Point Spacing",
            &look.surfacePointSpacingMeters,
            0.001F,
            0.020F,
            "%.3f m",
            ImGuiSliderFlags_Logarithmic);
        changed |= ImGui::SliderFloat(
            "Warp Amplitude",
            &look.warpAmplitudeMeters,
            0.0F,
            0.20F,
            "%.3f m");
        float tint[3] = {look.tintRed, look.tintGreen, look.tintBlue};
        if (ImGui::ColorEdit3("Tint", tint)) {
            look.tintRed = tint[0];
            look.tintGreen = tint[1];
            look.tintBlue = tint[2];
            changed = true;
        }
        changed |= ImGui::SliderFloat("Emission Boost", &look.emissionBoost, 0.0F, 4.0F, "%.2f");
        changed |= ImGui::SliderFloat("Opacity Boost", &look.opacityBoost, 0.0F, 1.0F, "%.2f");
        changed |= ImGui::SliderFloat("Point Size Boost", &look.pointSizeBoost, 0.0F, 2.0F, "%.2f");
        if (changed) {
            look.scale = 1.0F / std::max(0.001F, look.cellSizeMeters);
            const float widthInCells = look.lineWidthMeters / std::max(0.001F, look.cellSizeMeters);
            look.lineSharpness = std::clamp((0.16F - widthInCells) / (0.16F - 0.025F), 0.0F, 1.0F);
            look.warp = std::clamp(
                (look.warpAmplitudeMeters * 2.0F) / std::max(0.001F, look.cellSizeMeters),
                0.0F,
                3.0F);
            if (auto* editable = EnsureEditableWaterCausticLookSettings(runtimeState); editable != nullptr) {
                *editable = look;
            }
        }
        if (ImGui::Button(animationPath != nullptr ? "Save Look To Animation" : "Save Look To Default")) {
            SaveEditableWaterCausticLookSettings(runtimeState);
        }
        ImGui::SameLine();
        if (ImGui::Button(animationPath != nullptr ? "Discard Animation Edit" : "Discard Default Edit")) {
            DiscardEditableWaterCausticLookSettings(runtimeState);
        }
        EndPanelSection();
    }
}

void DrawWaterPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return;
    }

    auto& water = runtimeState->water;
    if (!ImGui::BeginTabBar("WaterFeatureTabs")) {
        return;
    }
    if (ImGui::BeginTabItem("Flow")) {
    if (BeginPanelSection("Water Sources")) {
        auto activeSupportIndex = ResolveWaterSupportSessionIndex(*runtimeState);
        const char* activeLabel =
            activeSupportIndex.has_value()
                ? runtimeState->sessions[activeSupportIndex.value()].displayName.c_str()
                : "No visible LiDAR layer";
        if (ImGui::BeginCombo("Support Layer", activeLabel)) {
            for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
                const auto& session = runtimeState->sessions[index];
                if (!IsVisibleLoadedWaterSupportSession(session)) {
                    continue;
                }
                const bool selected = activeSupportIndex.has_value() && activeSupportIndex.value() == index;
                if (ImGui::Selectable(session.displayName.c_str(), selected)) {
                    water.activeSupportSessionIndex = index;
                    QueueWaterPreview(runtimeState);
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        int waterViewModeIndex = water.overlayViewMode == WaterOverlayViewMode::Path ? 1 : 0;
        const char* waterViewModes[] = {"Trail View", "Path View"};
        if (ImGui::Combo("Viewport Water View", &waterViewModeIndex, waterViewModes, IM_ARRAYSIZE(waterViewModes))) {
            water.overlayViewMode =
                waterViewModeIndex == 1 ? WaterOverlayViewMode::Path : WaterOverlayViewMode::Trail;
            ApplyWaterOverlayDisplayStyle(runtimeState);
            runtimeState->statusMessage =
                std::string{"Water viewport set to "} + WaterOverlayViewModeName(water.overlayViewMode) + ".";
            runtimeState->errorMessage.clear();
        }

        if (BeginPanelSection("Source Settings")) {
            auto* selectedEmitter = SelectedWaterEmitter(runtimeState);
            if (selectedEmitter != nullptr) {
                ImGui::TextDisabled("Selected source: %s", selectedEmitter->name.c_str());
            }
            if (selectedEmitter != nullptr) {
                const auto currentSettingsLabel =
                    WaterSourceSettingsAssignmentLabel(*selectedEmitter, water.emitters);
                if (ImGui::BeginCombo("Path/Trail Settings", currentSettingsLabel.c_str())) {
                    const bool defaultSelected =
                        !selectedEmitter->tempSourceSettings.has_value() &&
                        selectedEmitter->sourceSettingsAssignment == WaterSourceSettingsAssignment::Default;
                    if (ImGui::Selectable("Default", defaultSelected)) {
                        const auto before = invisible_places::water::ResolveWaterSourceSettings(
                            *selectedEmitter,
                            water.emitters,
                            ActiveDefaultWaterSourceSettings(*runtimeState));
                        selectedEmitter->sourceSettingsAssignment = WaterSourceSettingsAssignment::Default;
                        selectedEmitter->linkedSourceSettingsEmitterId.reset();
                        selectedEmitter->tempSourceSettings.reset();
                        const auto after = invisible_places::water::ResolveWaterSourceSettings(
                            *selectedEmitter,
                            water.emitters,
                            ActiveDefaultWaterSourceSettings(*runtimeState));
                        ApplyWaterSourceSettingsTransition(
                            runtimeState,
                            viewport,
                            before,
                            after,
                            selectedEmitter->id);
                        runtimeState->statusMessage =
                            selectedEmitter->name + " now uses Default water source settings.";
                        runtimeState->errorMessage.clear();
                    }
                    if (defaultSelected) {
                        ImGui::SetItemDefaultFocus();
                    }

                    if (selectedEmitter->sourceSettings.has_value()) {
                        const auto label = WaterCustomSettingsName(selectedEmitter->id);
                        const bool selected =
                            !selectedEmitter->tempSourceSettings.has_value() &&
                            selectedEmitter->sourceSettingsAssignment == WaterSourceSettingsAssignment::Custom;
                        if (ImGui::Selectable(label.c_str(), selected)) {
                            const auto before = invisible_places::water::ResolveWaterSourceSettings(
                                *selectedEmitter,
                                water.emitters,
                                ActiveDefaultWaterSourceSettings(*runtimeState));
                            selectedEmitter->sourceSettingsAssignment = WaterSourceSettingsAssignment::Custom;
                            selectedEmitter->linkedSourceSettingsEmitterId.reset();
                            selectedEmitter->tempSourceSettings.reset();
                            const auto after = invisible_places::water::ResolveWaterSourceSettings(
                                *selectedEmitter,
                                water.emitters,
                                ActiveDefaultWaterSourceSettings(*runtimeState));
                            ApplyWaterSourceSettingsTransition(
                                runtimeState,
                                viewport,
                                before,
                                after,
                                selectedEmitter->id);
                            runtimeState->statusMessage =
                                selectedEmitter->name + " now uses " + label + ".";
                            runtimeState->errorMessage.clear();
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    if (selectedEmitter->tempSourceSettings.has_value()) {
                        const auto label = WaterCustomSettingsName(selectedEmitter->id, true);
                        ImGui::Selectable(label.c_str(), true);
                        ImGui::SetItemDefaultFocus();
                    }

                    bool addedLinkedSources = false;
                    for (const auto& otherEmitter : water.emitters) {
                        if (otherEmitter.id == selectedEmitter->id ||
                            !otherEmitter.sourceSettings.has_value()) {
                            continue;
                        }
                        if (!addedLinkedSources) {
                            ImGui::Separator();
                            addedLinkedSources = true;
                        }
                        const auto label = WaterCustomSettingsName(otherEmitter.id);
                        const bool selected =
                            !selectedEmitter->tempSourceSettings.has_value() &&
                            selectedEmitter->sourceSettingsAssignment == WaterSourceSettingsAssignment::LinkedEmitter &&
                            selectedEmitter->linkedSourceSettingsEmitterId.has_value() &&
                            selectedEmitter->linkedSourceSettingsEmitterId.value() == otherEmitter.id;
                        if (ImGui::Selectable(label.c_str(), selected)) {
                            const auto before = invisible_places::water::ResolveWaterSourceSettings(
                                *selectedEmitter,
                                water.emitters,
                                ActiveDefaultWaterSourceSettings(*runtimeState));
                            selectedEmitter->sourceSettingsAssignment = WaterSourceSettingsAssignment::LinkedEmitter;
                            selectedEmitter->linkedSourceSettingsEmitterId = otherEmitter.id;
                            selectedEmitter->tempSourceSettings.reset();
                            const auto after = invisible_places::water::ResolveWaterSourceSettings(
                                *selectedEmitter,
                                water.emitters,
                                ActiveDefaultWaterSourceSettings(*runtimeState));
                            ApplyWaterSourceSettingsTransition(
                                runtimeState,
                                viewport,
                                before,
                                after,
                                selectedEmitter->id);
                            runtimeState->statusMessage =
                                selectedEmitter->name + " now uses " + label + ".";
                            runtimeState->errorMessage.clear();
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::TextDisabled("Using: %s", WaterSourceSettingsAssignmentLabel(*selectedEmitter, water.emitters).c_str());
            } else {
                ImGui::TextDisabled(
                    water.tempDefaultSourceSettings.has_value()
                        ? "No source selected; editing Default_edited."
                        : "No source selected; editing Default.");
            }

            const bool hasSelectedSource = selectedEmitter != nullptr;
            if (!hasSelectedSource) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Deselect Source")) {
                DeselectWaterEmitter(runtimeState);
                selectedEmitter = nullptr;
            }
            if (!hasSelectedSource) {
                ImGui::EndDisabled();
            }

            const bool hasSourceTemp =
                selectedEmitter != nullptr
                    ? selectedEmitter->tempSourceSettings.has_value()
                    : water.tempDefaultSourceSettings.has_value();
            if (!hasSourceTemp) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button(selectedEmitter != nullptr ? "Save Edited Settings" : "Save Edited Default")) {
                SaveEditableWaterSourceSettings(runtimeState, viewport);
            }
            if (!hasSourceTemp) {
                ImGui::EndDisabled();
            }
            ImGui::SameLine();
            if (!hasSourceTemp) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button(selectedEmitter != nullptr ? "Discard Edited Settings" : "Discard Edited Default")) {
                DiscardEditableWaterSourceSettings(runtimeState, viewport);
            }
            if (!hasSourceTemp) {
                ImGui::EndDisabled();
            }
            EndPanelSection();
        }

        auto visibleSourceSettings = ViewedWaterSourceSettings(*runtimeState);
        if (BeginPanelSection("Path Generation")) {
            auto pathSettings = visibleSourceSettings.path;
            const auto previousPathSettings = pathSettings;
            bool pathChanged = false;
            pathChanged |= ImGui::Checkbox("Auto Tune", &pathSettings.autoTune);
            pathChanged |= ImGui::SliderFloat("Branching Flats", &pathSettings.branching, 0.0F, 1.0F, "%.2f");
            pathChanged |= ImGui::SliderFloat("Dense Coverage", &pathSettings.coverage, 0.0F, 1.0F, "%.2f");
            pathChanged |= ImGui::SliderFloat("Gap Tolerance", &pathSettings.gapTolerance, 0.0F, 1.0F, "%.2f");
            pathChanged |= ImGui::SliderFloat(
                "Path Reach",
                &pathSettings.pathLength,
                0.5F,
                250.0F,
                "%.1f m",
                ImGuiSliderFlags_Logarithmic);
            pathChanged |= ImGui::SliderFloat("Smoothing", &pathSettings.smoothing, 0.0F, 1.0F, "%.2f");
            if (ImGui::TreeNode("Advanced Path Controls")) {
                pathChanged |= ImGui::SliderFloat(
                    "Support Voxel",
                    &pathSettings.supportVoxelSize,
                    0.001F,
                    4.0F,
                    "%.3f m",
                    ImGuiSliderFlags_Logarithmic);
                pathChanged |= ImGui::SliderFloat(
                    "Bridge Upper Limit",
                    &pathSettings.maxBridgeDistance,
                    0.002F,
                    8.0F,
                    "%.3f m",
                    ImGuiSliderFlags_Logarithmic);
                pathChanged |= ImGui::SliderFloat(
                    "Path Sample Spacing",
                    &pathSettings.pathSampleSpacing,
                    0.001F,
                    2.0F,
                    "%.3f m",
                    ImGuiSliderFlags_Logarithmic);
                int maxSteps = static_cast<int>(pathSettings.maxSteps);
                if (ImGui::SliderInt("Max Steps", &maxSteps, 16, 20000)) {
                    pathSettings.maxSteps = static_cast<std::uint32_t>(std::max(16, maxSteps));
                    pathChanged = true;
                }
                int sampleLimit = static_cast<int>(pathSettings.supportSampleLimit);
                if (ImGui::SliderInt("Support Samples", &sampleLimit, 512, 500000)) {
                    pathSettings.supportSampleLimit = static_cast<std::uint32_t>(std::max(512, sampleLimit));
                    pathChanged = true;
                }
                ImGui::TreePop();
            }
            pathSettings.branching = std::clamp(pathSettings.branching, 0.0F, 1.0F);
            pathSettings.coverage = std::clamp(pathSettings.coverage, 0.0F, 1.0F);
            pathSettings.gapTolerance = std::clamp(pathSettings.gapTolerance, 0.0F, 1.0F);
            pathSettings.supportVoxelSize = std::clamp(pathSettings.supportVoxelSize, 0.001F, 10.0F);
            pathSettings.maxBridgeDistance = std::clamp(pathSettings.maxBridgeDistance, 0.001F, 20.0F);
            pathSettings.smoothing = std::clamp(pathSettings.smoothing, 0.0F, 1.0F);
            pathSettings.pathLength = std::clamp(pathSettings.pathLength, 0.1F, 500.0F);
            pathSettings.pathSampleSpacing = std::clamp(pathSettings.pathSampleSpacing, 0.001F, 10.0F);
            if (pathChanged) {
                auto beforeSettings = visibleSourceSettings;
                auto afterSettings = visibleSourceSettings;
                afterSettings.path = pathSettings;
                const bool bakeInputsChanged =
                    !invisible_places::water::WaterSourceBakeInputsEqual(beforeSettings, afterSettings);
                const auto* selectedEmitter = SelectedWaterEmitter(*runtimeState);
                if (auto* editableSettings = EnsureEditableWaterSourceSettings(runtimeState); editableSettings != nullptr) {
                    editableSettings->path = pathSettings;
                    if (bakeInputsChanged) {
                        if (selectedEmitter != nullptr) {
                            MarkWaterPathDirty(runtimeState, selectedEmitter->id);
                            runtimeState->statusMessage = "Selected water source path bake required.";
                        } else {
                            MarkWaterPathDirty(runtimeState);
                            runtimeState->statusMessage = "Water path bake required.";
                        }
                    } else if (previousPathSettings.smoothing != pathSettings.smoothing) {
                        RefreshWaterOverlayFromAnchors(runtimeState, viewport);
                    }
                    runtimeState->errorMessage.clear();
                }
            }
            if (water.pathCacheLoaded && !water.pathCache.diagnostics.summary.empty()) {
                ImGui::TextDisabled("%s", water.pathCache.diagnostics.summary.c_str());
                ImGui::TextDisabled(
                    "Branches: %u  hidden: %zu  confidence: %.2f  tuned bridge: %.3f m",
                    water.pathCache.diagnostics.branchCount,
                    water.pathCache.hiddenBranchIds.size(),
                    water.pathCache.diagnostics.averageConfidence,
                    water.pathCache.diagnostics.maxBridgeDistance);
            }
            const auto* selectedEmitter = SelectedWaterEmitter(*runtimeState);
            const bool selectedSourceDirty =
                selectedEmitter != nullptr &&
                water.dirtyEmitterIds.contains(selectedEmitter->id);
            if (selectedSourceDirty) {
                ImGui::TextDisabled("Selected source path bake required.");
            } else if (water.pathDirty) {
                ImGui::TextDisabled("Path bake required.");
            } else if (!water.pathAnchors.points.empty()) {
                ImGui::TextDisabled("Baked path anchors: %s", FormatPointCount(water.pathAnchors.points.size()).c_str());
            }
            if (water.lastTrailBuildDiagnostics.routedPathCount > 0U ||
                water.lastTrailBuildDiagnostics.surfaceSampleCount > 0U) {
                const auto& trailDiagnostics = water.lastTrailBuildDiagnostics;
                ImGui::TextDisabled(
                    "Trail: surface %.1f ms  route %.1f ms  lanes %.1f ms  particles %.1f ms",
                    trailDiagnostics.surfaceIndexBuildMs,
                    trailDiagnostics.routeMs,
                    trailDiagnostics.laneMs,
                    trailDiagnostics.particleMs);
                ImGui::TextDisabled(
                    "Trail samples: %s  lanes: %u  particles: %u",
                    FormatPointCount(trailDiagnostics.surfaceSampleCount).c_str(),
                    trailDiagnostics.emittedLaneCount,
                    trailDiagnostics.emittedParticleCount);
            }
            if (ImGui::Button("Bake Path")) {
                BakeWaterOverlayForActiveLayer(runtimeState, viewport);
            }
            EndPanelSection();
        }

        visibleSourceSettings = ViewedWaterSourceSettings(*runtimeState);
        if (BeginPanelSection("Trail Shape")) {
            auto trailShapeSettings = visibleSourceSettings.trailShape;
            bool trailShapeChanged = false;
            trailShapeChanged |= ImGui::SliderFloat(
                "Trail Looseness",
                &trailShapeSettings.trailLooseness,
                0.0F,
                1.0F,
                "%.2f");
            trailShapeChanged |= ImGui::SliderFloat(
                "Trail Smoothness",
                &trailShapeSettings.trailSmoothness,
                0.0F,
                1.0F,
                "%.2f");
            trailShapeChanged |= ImGui::SliderFloat(
                "Trail Spread",
                &trailShapeSettings.particleJitter,
                0.0F,
                2.0F,
                "%.2f");
            trailShapeChanged |= ImGui::SliderFloat(
                "Spline Anchor Spacing",
                &trailShapeSettings.splineAnchorSpacing,
                0.01F,
                10.0F,
                "%.2f m",
                ImGuiSliderFlags_Logarithmic);
            int trailLaneCount = static_cast<int>(trailShapeSettings.trailLaneCount);
            if (ImGui::SliderInt("Trail Lanes", &trailLaneCount, 0, 24)) {
                trailShapeSettings.trailLaneCount =
                    static_cast<std::uint32_t>(std::clamp(trailLaneCount, 0, 24));
                trailShapeChanged = true;
            }
            trailShapeSettings.trailLooseness = std::clamp(trailShapeSettings.trailLooseness, 0.0F, 1.0F);
            trailShapeSettings.trailSmoothness = std::clamp(trailShapeSettings.trailSmoothness, 0.0F, 1.0F);
            trailShapeSettings.particleJitter = std::clamp(trailShapeSettings.particleJitter, 0.0F, 3.0F);
            trailShapeSettings.splineAnchorSpacing =
                std::clamp(trailShapeSettings.splineAnchorSpacing, 0.01F, 25.0F);
            trailShapeSettings.trailLaneCount =
                std::clamp<std::uint32_t>(trailShapeSettings.trailLaneCount, 0U, 32U);
            trailShapeSettings.trailTurbulence =
                std::clamp(trailShapeSettings.trailLooseness * 1.5F, 0.0F, 3.0F);
            trailShapeSettings.trailMomentum =
                std::clamp(0.18F + trailShapeSettings.trailLooseness * 0.72F, 0.0F, 0.98F);
            trailShapeSettings.normalTurbulenceResponse =
                std::clamp(trailShapeSettings.trailLooseness * 1.4F, 0.0F, 3.0F);
            if (trailShapeChanged) {
                if (auto* editableSettings = EnsureEditableWaterSourceSettings(runtimeState); editableSettings != nullptr) {
                    editableSettings->trailShape = trailShapeSettings;
                    RefreshWaterOverlayFromAnchors(runtimeState, viewport);
                }
            }
            EndPanelSection();
        }

        if (BeginPanelSection("Animation Trail Playback")) {
            EnsureWaterAnimationTrailProfiles(&water);
            const auto selectedTrailProfile =
                NormalizeWaterAnimationTrailProfileName(water.selectedAnimationTrailProfileName);
            if (ImGui::BeginCombo("Trail Profile", selectedTrailProfile.c_str())) {
                const auto addTrailProfile = [&](std::string_view name) {
                    const auto normalized = NormalizeWaterAnimationTrailProfileName(name);
                    const bool selected = normalized == selectedTrailProfile;
                    if (ImGui::Selectable(normalized.c_str(), selected)) {
                        SelectWaterAnimationTrailProfile(runtimeState, normalized);
                        RefreshWaterOverlayFromAnchors(runtimeState, viewport);
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                };
                addTrailProfile(kDefaultWaterTrailProfileName);
                addTrailProfile(WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::FineWhiteThreads));
                addTrailProfile(WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::LongWhiteVeil));
                addTrailProfile(WaterTrailPlaybackQuickPresetName(WaterTrailPlaybackQuickPreset::SoftDistantDrift));
                if (!water.animationTrailProfiles.empty()) {
                    ImGui::Separator();
                }
                for (const auto& profile : water.animationTrailProfiles) {
                    addTrailProfile(profile.name);
                }
                if (water.editedAnimationTrailProfileSettings.has_value()) {
                    ImGui::Separator();
                    addTrailProfile(water.selectedAnimationTrailProfileName);
                }
                ImGui::EndCombo();
            }

            InputTextString("Trail Profile Name", &water.animationTrailProfileNameBuffer);
            if (ImGui::Button("Save Trail Profile")) {
                SaveCurrentWaterAnimationTrailProfile(runtimeState);
                RefreshWaterOverlayFromAnchors(runtimeState, viewport);
            }
            if (water.editedAnimationTrailProfileSettings.has_value()) {
                ImGui::SameLine();
                if (ImGui::Button("Discard Trail Edit")) {
                    DiscardEditableWaterAnimationTrailSettings(runtimeState);
                    RefreshWaterOverlayFromAnchors(runtimeState, viewport);
                }
            }
            if (CurrentAnimationPath(*runtimeState) != nullptr) {
                ImGui::SameLine();
                if (ImGui::Button("Unload Animation")) {
                    UnloadCurrentAnimationForWaterEditing(runtimeState);
                    SyncWaterAnimationTrailProfileFromCurrentAnimation(runtimeState);
                    RefreshWaterOverlayFromAnchors(runtimeState, viewport);
                }
            }
            if (IsProtectedWaterAnimationTrailProfileName(selectedTrailProfile)) {
                ImGui::TextDisabled("Built-in profile. Edits are saved under a new custom name.");
            } else if (IsEditedWaterAnimationTrailProfileName(selectedTrailProfile)) {
                ImGui::TextDisabled(
                    "Editing %s",
                    BaseWaterAnimationTrailProfileName(selectedTrailProfile).c_str());
            }

            auto trailSettings = ViewedWaterAnimationTrailSettings(*runtimeState);
            bool trailChanged = false;
            trailChanged |= ImGui::SliderFloat(
                "Particle Density",
                &trailSettings.particleDensity,
                0.05F,
                6.0F,
                "%.2f",
                ImGuiSliderFlags_Logarithmic);
            trailChanged |= ImGui::SliderFloat(
                "Particle Speed",
                &trailSettings.particleSpeed,
                0.05F,
                4.0F,
                "%.2f",
                ImGuiSliderFlags_Logarithmic);
            trailChanged |= ImGui::SliderFloat(
                "Trail Length",
                &trailSettings.trailLengthMeters,
                0.0F,
                8.0F,
                "%.2f m");
            trailChanged |= ImGui::SliderFloat(
                "Trail Point Spacing",
                &trailSettings.trailSampleSpacingMeters,
                0.0F,
                0.25F,
                "%.3f m");
            trailChanged |= ImGui::SliderFloat(
                "Colour Variation",
                &trailSettings.colorVariation,
                0.0F,
                1.0F,
                "%.2f");
            trailSettings.particleDensity = std::clamp(trailSettings.particleDensity, 0.05F, 10.0F);
            trailSettings.particleSpeed = std::clamp(trailSettings.particleSpeed, 0.05F, 8.0F);
            trailSettings.trailLengthMeters = std::clamp(trailSettings.trailLengthMeters, 0.0F, 25.0F);
            trailSettings.trailSampleSpacingMeters =
                std::clamp(trailSettings.trailSampleSpacingMeters, 0.0F, 25.0F);
            trailSettings.colorVariation = std::clamp(trailSettings.colorVariation, 0.0F, 1.0F);
            if (trailChanged) {
                if (auto* editableSettings = EnsureEditableWaterAnimationTrailSettings(runtimeState);
                    editableSettings != nullptr) {
                    *editableSettings = trailSettings;
                    if (auto* animationPath = CurrentAnimationPath(runtimeState); animationPath != nullptr) {
                        animationPath->tempWaterAnimationTrailSettings = trailSettings;
                        runtimeState->animationPanel.dirty = true;
                    } else {
                        water.tempDefaultAnimationTrailSettings = trailSettings;
                    }
                    RefreshWaterOverlayFromAnchors(runtimeState, viewport);
                }
            }
            EndPanelSection();
        }

        if (ImGui::Button(water.placementArmed ? "Click Viewport..." : "Place Emitter")) {
            water.placementArmed = !water.placementArmed;
            if (water.placementArmed) {
                water.movingEmitterIndex.reset();
                water.basinRegionPlacementArmed = false;
                water.runoffRegionPlacementArmed = false;
                water.causticRegionPlacementArmed = false;
            }
            runtimeState->statusMessage = water.placementArmed
                                              ? "Click the point-cloud viewport to place a water source."
                                              : "Water source placement cancelled.";
            runtimeState->errorMessage.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Suggest Emitters")) {
            SuggestWaterEmittersForActiveLayer(runtimeState);
        }

        if (ImGui::Button("Save Sources")) {
            SaveWaterSources(runtimeState);
        }
        ImGui::SameLine();
        if (ImGui::Button("Load Sources")) {
            LoadWaterSources(runtimeState, viewport);
        }

        if (!water.lastOverlayPath.empty()) {
            ImGui::TextDisabled("Last overlay: %s", water.lastOverlayPath.filename().string().c_str());
        }
        ImGui::TextDisabled("Sources file: %s", WaterSourcesPath(*runtimeState).filename().string().c_str());
        EndPanelSection();
    }

    if (BeginPanelSection("Emitter List")) {

    if (water.emitters.empty()) {
        ImGui::TextUnformatted("No water emitters yet.");
        EndPanelSection();
    } else {

    if (water.selectedEmitterIndex.has_value()) {
        if (ImGui::Button("Deselect Source")) {
            DeselectWaterEmitter(runtimeState);
        }
        ImGui::Separator();
    }

    std::optional<std::size_t> deleteIndex;
    for (std::size_t index = 0; index < water.emitters.size(); ++index) {
        auto& emitter = water.emitters[index];
        ImGui::PushID(static_cast<int>(index));
        const bool selected = water.selectedEmitterIndex.has_value() && water.selectedEmitterIndex.value() == index;
        const std::string label =
            emitter.name + "  [" + WaterSourceSettingsAssignmentLabel(emitter, water.emitters) + "]";
        if (ImGui::Selectable(label.c_str(), selected)) {
            SelectWaterEmitterInViewport(runtimeState, *viewport, index);
        }
        if (selected) {
            InputTextString("Name", &emitter.name);
            bool emitterChanged = false;
            float position[3] = {emitter.position.x, emitter.position.y, emitter.position.z};
            if (ImGui::InputFloat3("Position", position, "%.3f")) {
                emitter.position = {position[0], position[1], position[2]};
                emitterChanged = true;
            }
            const bool movingSelected =
                water.movingEmitterIndex.has_value() && water.movingEmitterIndex.value() == index;
            if (ImGui::Button(movingSelected ? "Cancel Move" : "Move In View")) {
                if (movingSelected) {
                    water.movingEmitterIndex.reset();
                    runtimeState->statusMessage = "Water source move cancelled.";
                } else {
                    water.movingEmitterIndex = index;
                    water.placementArmed = false;
                    water.basinRegionPlacementArmed = false;
                    water.runoffRegionPlacementArmed = false;
                    water.causticRegionPlacementArmed = false;
                    SelectWaterEmitterInViewport(runtimeState, *viewport, index);
                    runtimeState->statusMessage = "Click the point-cloud viewport to move " + emitter.name + ".";
                }
                runtimeState->errorMessage.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("Delete")) {
                deleteIndex = index;
            }
            if (emitterChanged) {
                QueueWaterPreview(runtimeState);
            }
        }
        ImGui::Separator();
        ImGui::PopID();
    }

    if (deleteIndex.has_value()) {
        water.emitters.erase(water.emitters.begin() + static_cast<std::ptrdiff_t>(deleteIndex.value()));
        if (water.selectedEmitterIndex.has_value()) {
            if (water.emitters.empty()) {
                water.selectedEmitterIndex.reset();
            } else if (water.selectedEmitterIndex.value() == deleteIndex.value()) {
                water.selectedEmitterIndex = std::min(deleteIndex.value(), water.emitters.size() - 1U);
            } else if (water.selectedEmitterIndex.value() > deleteIndex.value()) {
                water.selectedEmitterIndex = water.selectedEmitterIndex.value() - 1U;
            }
        }
        if (water.movingEmitterIndex.has_value()) {
            if (water.emitters.empty() || water.movingEmitterIndex.value() == deleteIndex.value()) {
                water.movingEmitterIndex.reset();
            } else if (water.movingEmitterIndex.value() > deleteIndex.value()) {
                water.movingEmitterIndex = water.movingEmitterIndex.value() - 1U;
            }
        }
        if (water.selectedEmitterIndex.has_value()) {
            if (water.selectedEmitterIndex.value() >= water.emitters.size()) {
                water.selectedEmitterIndex.reset();
            } else {
                SelectWaterEmitterInViewport(runtimeState, *viewport, water.selectedEmitterIndex.value());
            }
        }
        QueueWaterPreview(runtimeState);
        ValidateWaterSourceSettingLinks(runtimeState);
    }

    EndPanelSection();
    }
    }
    ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem("Basin Haze")) {
        water.activeRegionFeature = WaterRegionFeature::Basin;
        DrawWaterBasinPanel(runtimeState, viewport);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Runoff")) {
        water.activeRegionFeature = WaterRegionFeature::Runoff;
        DrawWaterRunoffPanel(runtimeState, viewport);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Caustics")) {
        water.activeRegionFeature = WaterRegionFeature::Caustic;
        DrawWaterCausticsPanel(runtimeState, viewport);
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
}

void DrawProjectPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (BeginPanelSection("Project Settings")) {
    auto& settings = runtimeState->projectSettings;
    ImGui::ColorEdit3("Background Color", settings.backgroundColor.data());
    ImGui::Checkbox("Eye-Dome Lighting", &settings.eyeDomeLightingEnabled);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Darkens point-cloud depth discontinuities in the viewport and animation exports.");
    }
    if (settings.eyeDomeLightingEnabled) {
        ImGui::SliderFloat("EDL Thickness", &settings.eyeDomeLightingThickness, 1.0F, 24.0F, "%.0f px");
        settings.eyeDomeLightingThickness = std::clamp(settings.eyeDomeLightingThickness, 1.0F, 24.0F);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Expands the eye-dome depth sampling radius for thicker, cartoon-like outlines.");
        }
    }
    ImGui::Checkbox("Show Status Overlay", &settings.showStatusOverlay);
    ImGui::Checkbox("Constant Update View", &settings.constantUpdateView);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Keeps re-rendering the 3D preview even when camera and visual settings are unchanged.");
    }
    ImGui::Checkbox("Live Visual Effects", &settings.liveVisualEffects);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Allows time-driven water and stylisation effects to update in preview.");
    }

    int pointRendererModeIndex =
        settings.pointCloudRendererMode == PointCloudRendererMode::FastBasic ? 1 : 0;
    const char* pointRendererModes[] = {"Beauty", "Fast Basic", "Raytraced (later)"};
    if (ImGui::Combo(
            "Point Renderer",
            &pointRendererModeIndex,
            pointRendererModes,
            IM_ARRAYSIZE(pointRendererModes))) {
        if (pointRendererModeIndex == 0) {
            settings.pointCloudRendererMode = PointCloudRendererMode::Beauty;
        } else if (pointRendererModeIndex == 1) {
            settings.pointCloudRendererMode = PointCloudRendererMode::FastBasic;
        } else {
            settings.pointCloudRendererMode = PointCloudRendererMode::Beauty;
            runtimeState->statusMessage = "Raytraced point rendering is reserved for a later implementation.";
            runtimeState->errorMessage.clear();
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Fast Basic renders opaque 1 px square points, with source RGB, solid colour, scalar colormaps, and colourise.");
    }

    int pointLodModeIndex = static_cast<int>(settings.pointCloudPreviewLodMode);
    const char* pointLodModes[] = {"Full Resolution", "Auto Camera LOD", "Force LOD"};
    if (ImGui::Combo(
            "Point Preview LOD",
            &pointLodModeIndex,
            pointLodModes,
            IM_ARRAYSIZE(pointLodModes))) {
        settings.pointCloudPreviewLodMode = static_cast<PointCloudPreviewLodMode>(pointLodModeIndex);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Auto mode uses the cached 10M point LOD only while the camera is moving.");
    }
    ImGui::TextDisabled("Point LOD Target: %s", FormatPointCount(kPointCloudPreviewLodTarget).c_str());
    EndPanelSection();
    }
    DrawProjectSection(runtimeState, viewport);
}

ImVec2 DefaultControlsWindowPosition(ImVec2 controlsSize) {
    const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
    if (mainViewport == nullptr) {
        return ImVec2{60.0F, 60.0F};
    }

    ImVec2 position{mainViewport->Pos.x + mainViewport->Size.x + 24.0F, mainViewport->Pos.y + 36.0F};
    const ImVec2 mainCenter{
        mainViewport->Pos.x + (mainViewport->Size.x * 0.5F),
        mainViewport->Pos.y + (mainViewport->Size.y * 0.5F),
    };

    const auto& platformIo = ImGui::GetPlatformIO();
    for (const auto& monitor : platformIo.Monitors) {
        const bool containsMainCenter =
            mainCenter.x >= monitor.WorkPos.x &&
            mainCenter.x <= monitor.WorkPos.x + monitor.WorkSize.x &&
            mainCenter.y >= monitor.WorkPos.y &&
            mainCenter.y <= monitor.WorkPos.y + monitor.WorkSize.y;
        if (!containsMainCenter) {
            continue;
        }

        const float workRight = monitor.WorkPos.x + monitor.WorkSize.x;
        const float workBottom = monitor.WorkPos.y + monitor.WorkSize.y;
        if (position.x + controlsSize.x > workRight) {
            position.x = std::max(monitor.WorkPos.x + 24.0F, workRight - controlsSize.x - 24.0F);
        }
        if (position.y + controlsSize.y > workBottom) {
            position.y = std::max(monitor.WorkPos.y + 24.0F, workBottom - controlsSize.y - 24.0F);
        }
        break;
    }

    return position;
}

void DrawRenderInfoSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr) {
        return;
    }

    if (BeginPanelSection("Render Info")) {
        if (!runtimeState->errorMessage.empty()) {
            ImGui::TextColored(ImVec4{0.74F, 0.18F, 0.14F, 1.0F}, "%s", runtimeState->errorMessage.c_str());
        } else if (!runtimeState->statusMessage.empty()) {
            ImGui::TextWrapped("%s", runtimeState->statusMessage.c_str());
        } else {
            ImGui::TextDisabled("Ready.");
        }

        const auto& diagnostics = viewport.Diagnostics();
        const auto viewportSize = CurrentUiViewportSize(viewport);
        ImGui::Text(
            "Render window: %.0f x %.0f UI, %u x %u framebuffer",
            viewportSize.x,
            viewportSize.y,
            viewport.Width(),
            viewport.Height());
        if (!diagnostics.rendererName.empty()) {
            ImGui::Text("GPU: %s", diagnostics.rendererName.c_str());
        }
        if (diagnostics.pointCount > 0) {
            ImGui::Text("Layer draw target: %s", FormatPointCount(diagnostics.pointCount).c_str());
            ImGui::Text(
                "Last render submitted: %s",
                FormatPointCount(diagnostics.pointSubmittedCount).c_str());
            if (diagnostics.pointPassSubmittedCount > diagnostics.pointSubmittedCount) {
                ImGui::Text(
                    "Pass submissions: %s",
                    FormatPointCount(diagnostics.pointPassSubmittedCount).c_str());
            }
            ImGui::Text("Average point size: %.2f px", diagnostics.averagePointSizePx);
            ImGui::Text("Point renderer: %s", PointCloudRendererModeLabel(runtimeState->projectSettings.pointCloudRendererMode));
            if (!diagnostics.pointRenderModes.empty()) {
                ImGui::TextWrapped("Point modes: %s", diagnostics.pointRenderModes.c_str());
            }
            ImGui::TextDisabled("Off-screen points are currently clipped by the GPU after submission.");
        }
        ImGui::Separator();
        ImGui::Checkbox("Diagnostics", &runtimeState->showDiagnosticsPanel);
        ImGui::Checkbox("Pause 3D View During Export", &runtimeState->pauseLiveViewportDuringExport);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Keeps the controls responsive while skipping live scene rendering during MP4/EXR exports.");
        }
        if (runtimeState->offlineRenderJob.active && runtimeState->pauseLiveViewportDuringExport) {
            ImGui::TextDisabled("3D view paused while export renders.");
        }
        EndPanelSection();
    }

    if (BeginPanelSection("Layers")) {
        ImGui::Text("Loaded layers: %s", FormatPointCount(LoadedLayerCount(*runtimeState)).c_str());
        ImGui::Text("Visible layers: %s", FormatPointCount(VisibleLayerCount(*runtimeState)).c_str());
        if (const auto* session = SelectedLoadedSession(*runtimeState); session != nullptr) {
            ImGui::Text("Selected: %s", session->displayName.c_str());
            ImGui::Text("Kind: %s", LayerKindLabel(session->kind));
            if (session->kind == LayerKind::PointCloud) {
                ImGui::Text("Budget: %s", DescribeBudget(session->pointBudget).c_str());
                ImGui::Text(
                    "%s: %s",
                    PointCloudPreviewLodApplied(*runtimeState, *session) ? "Preview LOD" : "Preview",
                    DescribePointCloudPreviewDraw(*runtimeState, *session).c_str());
                ImGui::Text(
                    "LOD Mode: %s",
                    PointCloudPreviewLodModeLabel(runtimeState->projectSettings.pointCloudPreviewLodMode));
            } else {
                const auto effectiveQuality = EffectiveGaussianSplatQualityMode(*runtimeState, *session);
                ImGui::Text("Quality: %s", GaussianSplatQualityModeLabel(session->gsplatStyle.qualityMode));
                if (effectiveQuality != session->gsplatStyle.qualityMode) {
                    ImGui::Text("Rendering: %s while interacting", GaussianSplatQualityModeLabel(effectiveQuality));
                }
                ImGui::Text("Debug: %s", GaussianSplatDebugModeLabel(session->gsplatStyle.debugMode));
            }
        }
        EndPanelSection();
    }

    if (runtimeState->pendingLoad.has_value()) {
        const auto& pendingLoad = runtimeState->pendingLoad.value();
        if (!BeginPanelSection("Layer Load")) {
            return;
        }
        if (pendingLoad.sessionIndex < runtimeState->sessions.size()) {
            const auto& session = runtimeState->sessions[pendingLoad.sessionIndex];
            ImGui::Text("%s", session.displayName.c_str());
            ImGui::Text("%s", LayerKindLabel(session.kind));
        }
        const auto elapsed = std::chrono::steady_clock::now() - pendingLoad.startedAt;
        ImGui::Text(
            "%s for %s",
            pendingLoad.phase == PendingLoadPhase::CpuLoading ? "Reading source data" : "Uploading GPU buffers",
            FormatElapsedTime(elapsed).c_str());
        EndPanelSection();
    }

    auto& job = runtimeState->offlineRenderJob;
    if (job.active) {
        RefreshAnimationExportWriterProgress(&job);
        const char* sectionLabel = OfflineRenderJobOverlayLabel(job);
        if (!BeginPanelSection(sectionLabel)) {
            return;
        }
        if (job.preparingExport && job.preparationState != nullptr) {
            std::size_t completedRequests = 0;
            std::size_t totalRequests = 0;
            std::string currentLayerName;
            {
                std::scoped_lock lock(job.preparationState->mutex);
                completedRequests = job.preparationState->completedRequests;
                totalRequests = job.preparationState->totalRequests;
                currentLayerName = job.preparationState->currentLayerName;
            }
            const float prepareProgress = totalRequests == 0
                                              ? 0.0F
                                              : static_cast<float>(completedRequests) /
                                                    static_cast<float>(totalRequests);
            ImGui::ProgressBar(prepareProgress, ImVec2{-FLT_MIN, 0.0F});
            ImGui::Text("Preparing samples %zu / %zu", completedRequests, totalRequests);
            if (!currentLayerName.empty()) {
                ImGui::TextWrapped("Layer: %s", currentLayerName.c_str());
            }
        } else {
            const float frameProgress =
                job.frames.empty()
                    ? 0.0F
                    : static_cast<float>(job.writtenFrameCount) / static_cast<float>(job.frames.size());
            ImGui::ProgressBar(frameProgress, ImVec2{-FLT_MIN, 0.0F});
            ImGui::Text(
                "Captured %u / %zu, saved %u, queued %zu",
                std::min<std::uint32_t>(job.currentFrame, static_cast<std::uint32_t>(job.frames.size())),
                job.frames.size(),
                std::min<std::uint32_t>(job.writtenFrameCount, static_cast<std::uint32_t>(job.frames.size())),
                job.pendingFrameCount);
        }
        ImGui::Text("Elapsed: %s", FormatElapsedTime(std::chrono::steady_clock::now() - job.startedAt).c_str());
        if (!job.lastOutputPath.empty()) {
            ImGui::TextWrapped("Last: %s", job.lastOutputPath.string().c_str());
        }
        if (!job.exportLog.path.empty()) {
            ImGui::TextWrapped("Log: %s", job.exportLog.path.string().c_str());
        }
        ImGui::Checkbox("Pause 3D View", &runtimeState->pauseLiveViewportDuringExport);
        if (ImGui::Button(job.cancelRequested ? "Cancelling..." : "Cancel Export")) {
            RequestOfflineRenderCancellation(&job);
        }
        EndPanelSection();
    }
}

void DrawDiagnosticsWindow(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr || !runtimeState->showDiagnosticsPanel) {
        return;
    }

    bool open = true;
    ImGui::SetNextWindowSize(ImVec2{460.0F, 520.0F}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Renderer Diagnostics", &open, ImGuiWindowFlags_NoCollapse)) {
        const auto& diagnostics = viewport.Diagnostics();
        const auto viewportSize = CurrentUiViewportSize(viewport);

        if (BeginPanelSection("Renderer")) {
            ImGui::Text(
                "GPU: %s",
                diagnostics.rendererName.empty() ? "Unknown" : diagnostics.rendererName.c_str());
            if (!diagnostics.driverName.empty()) {
                ImGui::Text("Driver: %s", diagnostics.driverName.c_str());
            }
            ImGui::Text(
                "Render window: %.0f x %.0f UI, %u x %u framebuffer",
                viewportSize.x,
                viewportSize.y,
                viewport.Width(),
                viewport.Height());
            ImGui::Text(
                "Accumulation: %u x %u",
                diagnostics.accumulationWidth,
                diagnostics.accumulationHeight);
            if (diagnostics.framesInFlight > 0U) {
                ImGui::Text(
                    "Frames: %u in flight, %u swapchain images, current %u",
                    diagnostics.framesInFlight,
                    diagnostics.swapchainImageCount,
                    diagnostics.currentFrameIndex);
            }
            EndPanelSection();
        }

        if (BeginPanelSection("Frame Timing")) {
            ImGui::Text(
                "Render frame: %.3f ms (%.1f FPS)",
                diagnostics.frameRenderMs,
                diagnostics.frameFps);
            ImGui::Text(
                "%.1fs average: %.3f ms (%.1f FPS)",
                diagnostics.frameAverageWindowSeconds,
                diagnostics.averageFrameRenderMs,
                diagnostics.averageFrameFps);
            ImGui::Text(
                "Min / max while open: %.3f / %.3f ms",
                diagnostics.minFrameRenderMs,
                diagnostics.maxFrameRenderMs);
            ImGui::Separator();
            ImGui::Text("UI render: %.3f ms", diagnostics.frameUiRenderMs);
            ImGui::Text("Fence wait: %.3f ms", diagnostics.frameFenceWaitMs);
            ImGui::Text("Prepare uniforms/resources: %.3f ms", diagnostics.framePrepareMs);
            ImGui::Text("Acquire image: %.3f ms", diagnostics.frameAcquireMs);
            ImGui::Text("Acquired image wait: %.3f ms", diagnostics.frameImageWaitMs);
            ImGui::Text("Command buffer: %.3f ms", diagnostics.frameCommandBufferMs);
            ImGui::Text("Queue submit: %.3f ms", diagnostics.frameSubmitMs);
            ImGui::Text("Present: %.3f ms", diagnostics.framePresentMs);
            ImGui::Text("Platform windows: %.3f ms", diagnostics.framePlatformWindowsMs);
            EndPanelSection();
        }

        if (BeginPanelSection("Point Clouds")) {
            ImGui::Text("Layer draw target: %s", FormatPointCount(diagnostics.pointCount).c_str());
            ImGui::Text(
                "Last render submitted: %s",
                FormatPointCount(diagnostics.pointSubmittedCount).c_str());
            ImGui::Text(
                "Pass submissions: %s",
                FormatPointCount(diagnostics.pointPassSubmittedCount).c_str());
            ImGui::Text("Average point size: %.2f px", diagnostics.averagePointSizePx);
            if (!diagnostics.pointRenderModes.empty()) {
                ImGui::TextWrapped("Point modes: %s", diagnostics.pointRenderModes.c_str());
            }
            ImGui::Text(
                "Point draws: %u (%u depth, %u material)",
                diagnostics.pointDrawCalls,
                diagnostics.pointDepthLayerCount,
                diagnostics.pointAccumulationLayerCount);
            ImGui::Text(
                "Material variants: %u opaque, %u simple, %u unified",
                diagnostics.pointOpaqueHardDiscDrawCalls,
                diagnostics.pointConstantSimpleDrawCalls,
                diagnostics.pointUnifiedDrawCalls);
            if (diagnostics.pointFastBasicDrawCalls > 0) {
                ImGui::Text(
                    "Fast Basic: %u draws, %s points",
                    diagnostics.pointFastBasicDrawCalls,
                    FormatPointCount(diagnostics.pointFastBasicDrawnPoints).c_str());
            }
            ImGui::Text(
                "Depth prepass skipped (no X-Ray): %u",
                diagnostics.pointDepthPrepassSkippedNoXray);
            ImGui::Text("Style uploads: %u", diagnostics.pointStyleUploadCount);
            ImGui::Text("Inactive bindings skipped: %u", diagnostics.pointSkippedInactiveBindings);
            ImGui::Text("Command record: %.3f ms", diagnostics.pointCommandRecordMs);
            ImGui::Text(
                "Preview cache: %s",
                diagnostics.sceneCacheActive
                    ? (diagnostics.sceneRenderedThisFrame ? "refreshing" : "cached")
                    : "off");
            EndPanelSection();
        }
    }
    ImGui::End();

    runtimeState->showDiagnosticsPanel = open;
}

void DrawControlsWindow(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr) {
        return;
    }

    auto& sidePanel = runtimeState->sidePanel;
    sidePanel.revealAmount = 1.0F;
    sidePanel.mode = invisible_places::ui::SidePanelMode::Expanded;
    runtimeState->water.activeRegionFeature = WaterRegionFeature::None;

    const ImVec2 controlsSize{540.0F, 780.0F};
    ImGuiWindowClass controlsWindowClass;
    controlsWindowClass.ClassId = 0x49504354U;
    controlsWindowClass.ViewportFlagsOverrideSet = ImGuiViewportFlags_NoAutoMerge;
    controlsWindowClass.ViewportFlagsOverrideClear = ImGuiViewportFlags_NoDecoration;
    ImGui::SetNextWindowClass(&controlsWindowClass);
    ImGui::SetNextWindowPos(DefaultControlsWindowPosition(controlsSize), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(controlsSize, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2{460.0F, 420.0F}, ImVec2{780.0F, FLT_MAX});

    const bool popupOpen =
        ImGui::IsPopupOpen(static_cast<const char*>(nullptr), ImGuiPopupFlags_AnyPopupId);
    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    ImGui::Begin("Invisible Places Controls", nullptr, flags);

    const auto& diagnostics = viewport->Diagnostics();
    const double renderFps =
        diagnostics.averageFrameFps > 0.0 ? diagnostics.averageFrameFps : diagnostics.frameFps;
    const char* previewStatus =
        diagnostics.sceneCacheActive && !diagnostics.sceneRenderedThisFrame ? "cached" : "rendering";
    const std::string fpsLabel =
        "Render FPS (" + FormatFixed(diagnostics.frameAverageWindowSeconds, 1) + "s): " +
        FormatFixed(renderFps, 1) + "  " + previewStatus;
    const float fpsLabelWidth = ImGui::CalcTextSize(fpsLabel.c_str()).x;
    const float fpsCursorX =
        std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - fpsLabelWidth);
    ImGui::SetCursorPosX(fpsCursorX);
    ImGui::TextDisabled("%s", fpsLabel.c_str());

    sidePanel.hovered = ImGui::IsWindowHovered(
        ImGuiHoveredFlags_AllowWhenBlockedByActiveItem | ImGuiHoveredFlags_RootAndChildWindows);
    sidePanel.interacting =
        popupOpen || ImGui::IsAnyItemActive() ||
        ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    constexpr ImGuiWindowFlags tabScrollFlags = ImGuiWindowFlags_AlwaysVerticalScrollbar;
    if (ImGui::BeginTabBar("ScenePanelTabs")) {
        if (ImGui::BeginTabItem("Info")) {
            if (ImGui::BeginChild("InfoTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawRenderInfoSection(runtimeState, *viewport);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Lidar")) {
            if (ImGui::BeginChild("LidarTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawLidarPanel(runtimeState, viewport);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Visuals")) {
            if (ImGui::BeginChild("VisualsTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawVisualsPanel(runtimeState);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("gSplat")) {
            if (ImGui::BeginChild("GsplatTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawGsplatPanel(runtimeState, viewport);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Water")) {
            if (ImGui::BeginChild("WaterTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawWaterPanel(runtimeState, viewport);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Camera")) {
            if (ImGui::BeginChild("CameraTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawCameraSection(runtimeState, *viewport);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Animation")) {
            if (ImGui::BeginChild("AnimationTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawAnimationSection(runtimeState, *viewport);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Project")) {
            if (ImGui::BeginChild("ProjectTabScroll", ImVec2{0.0F, 0.0F}, false, tabScrollFlags)) {
                DrawProjectPanel(runtimeState, viewport);
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    sidePanel.interacting =
        popupOpen || ImGui::IsAnyItemActive() ||
        ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    ImGui::End();
}

void UpdateCameraFromInput(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (VisibleLayerCount(*runtimeState) == 0) {
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }

    const auto& io = ImGui::GetIO();
    bool navigatedThisFrame = false;
    if (runtimeState->animationPanel.drag.active) {
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    if (runtimeState->water.regionEditor.consumedViewportInputThisFrame ||
        runtimeState->water.regionEditor.drag.active) {
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    const bool mouseButtonDown = io.MouseDown[0] || io.MouseDown[1] || io.MouseDown[2];
    const bool renderViewportHovered = IsMouseOverRenderViewport(viewport);
    if (!mouseButtonDown) {
        runtimeState->cameraInteraction.renderViewportMouseActive = false;
    } else if (renderViewportHovered && !viewport.UiWantsMouseCapture()) {
        runtimeState->cameraInteraction.renderViewportMouseActive = true;
    }

    const bool mouseCanNavigate =
        !viewport.UiWantsMouseCapture() &&
        (renderViewportHovered || runtimeState->cameraInteraction.renderViewportMouseActive);
    if (runtimeState->water.movingEmitterIndex.has_value() &&
        renderViewportHovered &&
        !viewport.UiWantsMouseCapture() &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        MoveWaterEmitterAtScreenPoint(runtimeState, viewport, io.MousePos);
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    if (runtimeState->water.placementArmed &&
        renderViewportHovered &&
        !viewport.UiWantsMouseCapture() &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        PlaceWaterEmitterAtScreenPoint(runtimeState, viewport, io.MousePos);
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    if (runtimeState->water.basinRegionPlacementArmed &&
        renderViewportHovered &&
        !viewport.UiWantsMouseCapture() &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        AddWaterBasinVertexAtScreenPoint(runtimeState, viewport, io.MousePos);
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    if (runtimeState->water.runoffRegionPlacementArmed &&
        renderViewportHovered &&
        !viewport.UiWantsMouseCapture() &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        AddWaterRunoffVertexAtScreenPoint(runtimeState, viewport, io.MousePos);
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    if (runtimeState->water.causticRegionPlacementArmed &&
        renderViewportHovered &&
        !viewport.UiWantsMouseCapture() &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        AddWaterCausticVertexAtScreenPoint(runtimeState, viewport, io.MousePos);
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    if (HandleWaterPathViewInput(runtimeState, &viewport)) {
        runtimeState->cameraInteraction.navigationActive = false;
        return;
    }
    if (mouseCanNavigate) {
        if (renderViewportHovered && io.MouseWheel != 0.0F) {
            runtimeState->camera.Dolly(io.MouseWheel);
            navigatedThisFrame = true;
        }

        const bool doubleClickedPivot =
            !io.KeyShift && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
        const bool mouseMoved = io.MouseDelta.x != 0.0F || io.MouseDelta.y != 0.0F;
        const bool isPanning = io.MouseDown[1] || io.MouseDown[2] || (io.KeyShift && io.MouseDown[0]);
        if (doubleClickedPivot) {
            SetCameraPivotFromScreenPoint(runtimeState, viewport, io.MousePos);
            navigatedThisFrame = true;
        } else if (isPanning && mouseMoved) {
            const auto viewportSize = CurrentUiViewportSize(viewport);
            runtimeState->camera.Pan(io.MouseDelta.x, io.MouseDelta.y, viewportSize.x, viewportSize.y);
            navigatedThisFrame = true;
        } else if (io.MouseDown[0] && mouseMoved) {
            runtimeState->camera.Orbit(io.MouseDelta.x, io.MouseDelta.y);
            navigatedThisFrame = true;
        }
    }

    const bool keyboardCanNavigate = !viewport.UiWantsKeyboardCapture() && IsRenderViewportFocused();
    if (keyboardCanNavigate && ImGui::IsKeyPressed(ImGuiKey_F)) {
        if (runtimeState->selectedSessionIndex.has_value()) {
            FocusSessionLayer(runtimeState, viewport, runtimeState->selectedSessionIndex.value());
            navigatedThisFrame = true;
        }
    }

    if (keyboardCanNavigate && ImGui::IsKeyPressed(ImGuiKey_P)) {
        const ImVec2 mousePosition = io.MousePos;
        SetCameraPivotFromScreenPoint(runtimeState, viewport, mousePosition);
        navigatedThisFrame = true;
    }

    if (keyboardCanNavigate && ImGui::IsKeyPressed(ImGuiKey_V)) {
        runtimeState->pivotOverlay.visible = !runtimeState->pivotOverlay.visible;
        runtimeState->pivotOverlay.pivot = FromGlm(runtimeState->camera.OrbitCenter());
        runtimeState->pivotOverlay.lastSetAt = runtimeState->pivotOverlay.visible
                                                   ? std::chrono::steady_clock::now()
                                                   : std::chrono::steady_clock::time_point{};
        runtimeState->statusMessage = runtimeState->pivotOverlay.visible
                                          ? "Pivot marker pinned on."
                                          : "Pivot marker hidden until camera navigation.";
    }

    const auto now = std::chrono::steady_clock::now();
    if (navigatedThisFrame) {
        runtimeState->cameraInteraction.lastNavigationInputAt = now;
        if (runtimeState->cameraPlayback.active) {
            runtimeState->cameraPlayback.active = false;
        }
        if (runtimeState->animationPlayback.active) {
            runtimeState->animationPlayback.active = false;
        }
    }
    runtimeState->cameraInteraction.navigationActive =
        navigatedThisFrame ||
        (runtimeState->cameraInteraction.lastNavigationInputAt.time_since_epoch().count() != 0 &&
         (now - runtimeState->cameraInteraction.lastNavigationInputAt) <
             std::chrono::milliseconds{180});
}

void UpdatePerformanceInteractionState(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (runtimeState == nullptr) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool uiInteractionActive =
        runtimeState->sidePanel.hovered ||
        runtimeState->sidePanel.interacting ||
        ImGui::IsAnyItemActive() ||
        viewport.UiWantsMouseCapture() ||
        viewport.UiWantsKeyboardCapture();
    if (uiInteractionActive) {
        runtimeState->performanceInteraction.lastUiInteractionAt = now;
    }

    const bool uiInteractionRecent =
        runtimeState->performanceInteraction.lastUiInteractionAt.time_since_epoch().count() != 0 &&
        (now - runtimeState->performanceInteraction.lastUiInteractionAt) < kPerformanceInteractionHold;
    runtimeState->performanceInteraction.active =
        runtimeState->cameraInteraction.navigationActive || uiInteractionRecent;
}

void PrunePreviewLodSampleCaches(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    for (auto& session : runtimeState->sessions) {
        if (!session.loaded ||
            session.kind != LayerKind::PointCloud ||
            session.offlinePointCloud == nullptr) {
            ClearPreviewLodSampleCache(&session);
            continue;
        }
    }
}

invisible_places::renderer::core::SceneRenderState BuildRenderState(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    float flowTimeSeconds) {
    invisible_places::renderer::core::SceneRenderState renderState;
    const auto aspectRatio = CurrentAspectRatio(viewport);
    const auto matrices = runtimeState.camera.Matrices(aspectRatio);

    renderState.view = matrices.view;
    renderState.projection = matrices.projection;
    renderState.viewProjection = matrices.viewProjection;
    renderState.cameraPosition = matrices.position;
    renderState.backgroundColor = glm::vec4{
        runtimeState.projectSettings.backgroundColor[0],
        runtimeState.projectSettings.backgroundColor[1],
        runtimeState.projectSettings.backgroundColor[2],
        runtimeState.projectSettings.backgroundColor[3],
    };
    renderState.pointCloudRendererMode = runtimeState.projectSettings.pointCloudRendererMode;
    renderState.eyeDomeLightingEnabled =
        runtimeState.projectSettings.eyeDomeLightingEnabled &&
        !FastBasicPointRendererActive(runtimeState.projectSettings);
    renderState.eyeDomeLightingThickness = runtimeState.projectSettings.eyeDomeLightingThickness;
    renderState.nearPlane = runtimeState.camera.NearPlane();
    renderState.farPlane = runtimeState.camera.FarPlane();
    const auto cameraState = runtimeState.camera.CaptureState();
    renderState.hasDepthOfField =
        cameraState.hasDepthOfField && !FastBasicPointRendererActive(runtimeState.projectSettings);
    renderState.focusDistance = cameraState.focusDistance;
    renderState.apertureFStops = cameraState.apertureFStops;
    renderState.depthOfFieldMaxBlurPixels = cameraState.depthOfFieldMaxBlurPixels;
    renderState.gaussianSplatFootprintBoost = runtimeState.projectSettings.gaussianSplatFootprintBoost;
    renderState.flowTimeSeconds =
        FastBasicPointRendererActive(runtimeState.projectSettings) ? 0.0F : flowTimeSeconds;

    for (std::size_t sessionIndex = 0; sessionIndex < runtimeState.sessions.size(); ++sessionIndex) {
        const auto& session = runtimeState.sessions[sessionIndex];
        if (!session.loaded || !session.visible) {
            continue;
        }

        if (session.kind == LayerKind::PointCloud) {
            auto drawPointCount = std::min<std::uint64_t>(
                EffectivePointDrawCount(runtimeState, session),
                std::numeric_limits<std::uint32_t>::max());
            const auto previewLodDrawCount = static_cast<std::uint32_t>(drawPointCount);
            const bool previewLodSampleReady =
                session.previewLodSampledDrawCount == previewLodDrawCount &&
                session.previewLodSampledIndices.size() >= previewLodDrawCount;
            if (!session.pointBudget.UsesSampledIndices() &&
                drawPointCount < session.pointBudget.activePoints &&
                !previewLodSampleReady) {
                drawPointCount = std::min<std::uint64_t>(
                    session.pointBudget.activePoints,
                    std::numeric_limits<std::uint32_t>::max());
            }
            renderState.pointCloudLayers.push_back(
                {.layerId = sessionIndex,
                 .style = [&]() {
                     auto style = ApplyWaterCausticRenderStyle(runtimeState, session, session.pointStyle);
                     return FastBasicPointRendererActive(runtimeState.projectSettings)
                                ? MakeEffectiveFastBasicStyle(
                                      style,
                                      session.hasSourceRgb,
                                      IsGeneratedWaterOverlaySession(session))
                                : style;
                 }(),
                 .scalarFields = session.scalarFields,
                 .hasSourceRgb = session.hasSourceRgb,
                 .drawPointCount = static_cast<std::uint32_t>(drawPointCount)});
        } else {
            const auto effectiveFrame = ComputeEffectiveLayerFrame(runtimeState, session);
            if (effectiveFrame.bounds.valid &&
                !BoundsIntersectsViewFrustum(effectiveFrame.bounds, renderState.viewProjection)) {
                continue;
            }

            auto effectiveStyle = session.gsplatStyle;
            effectiveStyle.qualityMode = EffectiveGaussianSplatQualityMode(runtimeState, session);
            renderState.gaussianSplatLayers.push_back(
                {.layerId = sessionIndex,
                 .style = effectiveStyle,
                 .localToWorld = EffectiveGsplatLocalToWorld(runtimeState.projectSettings, session)});
        }
    }

    return renderState;
}

}  // namespace

Application::Application(std::filesystem::path dataRoot)
    : dataRoot_(ResolveDataDirectory(dataRoot)) {}

std::filesystem::path Application::DefaultDataDirectory() {
    return std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
}

int Application::Run() const {
    const auto runtimeConfig = platform::PrepareVulkanRuntime();
    const auto assetCatalog = io::DiscoverAssets(dataRoot_);
    const auto sceneCatalog = scene::SceneCatalog::FromDiscoveredAssets(assetCatalog);

    std::cout << "Invisible Places preview/export app" << std::endl;
    std::cout << "Data root: " << dataRoot_.string() << std::endl << std::endl;
    std::cout << platform::DescribeVulkanRuntime(runtimeConfig) << std::endl << std::endl;
    std::cout << assetCatalog.Summary();
    std::cout << std::endl;
    std::cout << sceneCatalog.Summary() << std::flush;

    if (!assetCatalog.issues.empty()) {
        std::cerr << "\nDiscovery issues\n";
        for (const auto& issue : assetCatalog.issues) {
            std::cerr << "- " << issue.filePath.string() << ": " << issue.message << "\n";
        }
        return 2;
    }

    std::cout << std::endl
              << "Opening preview window. Press Escape or close the window to exit." << std::endl;

    const auto windowTitle = platform::MakeBootstrapWindowTitle(assetCatalog);
    platform::Window window{
        {.width = 1440, .height = 900, .title = windowTitle},
    };

    std::optional<renderer::core::VulkanViewportShell> viewport;
    try {
        viewport.emplace(window.NativeHandle());
        std::cout << viewport->Diagnostics().summary << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "Vulkan viewport initialization failed: " << error.what() << "\n";
        window.ShowBootstrapContent(
            {.headline = "Vulkan viewport failed to start.",
             .summary =
                 "The window is alive, but the renderer shell could not initialize. "
                 "The details below are the next thing to fix.",
             .detailLines =
                 {
                     error.what(),
                     "Point-cloud layers discovered: " + std::to_string(assetCatalog.pointClouds.size()),
                     "Gaussian splat layers discovered: " + std::to_string(assetCatalog.gaussianSplats.size()),
                 },
             .footer = "Close the window or press Escape to exit."});
    }

    PreviewRuntimeState runtimeState;
    runtimeState.water.defaultPointVisualStyle = MakeDefaultWaterPointVisualStyle();
    runtimeState.sessions = BuildSessions(assetCatalog);
    runtimeState.sidePanel.pinned = false;
    runtimeState.sidePanel.panelWidth = 410.0F;
    runtimeState.persistence.projectFilePath = DefaultProjectFilePath(dataRoot_).string();
    runtimeState.persistence.pointStylePresetPath = DefaultPointStylePresetPath(dataRoot_).string();
    runtimeState.persistence.animationDirectoryPath = DefaultAnimationDirectory(dataRoot_).string();
    runtimeState.renderSettings.outputDirectory = DefaultRenderOutputDirectory(dataRoot_).string();
    RefreshAnimationFileList(&runtimeState.animationPanel, AnimationDirectory(runtimeState));

    if (viewport.has_value()) {
        bool loadedStartupProject = false;
        const std::filesystem::path startupProjectPath{runtimeState.persistence.projectFilePath};
        std::error_code startupProjectError;
        if (std::filesystem::is_regular_file(startupProjectPath, startupProjectError)) {
            std::string projectErrorMessage;
            const auto startupProject =
                invisible_places::serialization::LoadProjectDocument(startupProjectPath, &projectErrorMessage);
            if (startupProject.has_value() &&
                ApplyProjectDocumentToRuntime(startupProject.value(), &runtimeState, &viewport.value())) {
                loadedStartupProject = true;
                runtimeState.statusMessage =
                    "Loaded last project from " + startupProjectPath.string() + ".";
            } else {
                runtimeState.errorMessage =
                    "Could not load last project: " + projectErrorMessage + ". Loading the startup cloud instead.";
                std::cerr << runtimeState.errorMessage << std::endl;
            }
        }

        if (loadedStartupProject) {
            StartQueuedLayerLoadIfIdle(&runtimeState);
        } else if (HasAnyPointClouds(runtimeState)) {
            BeginLayerLoad(ChooseStartupCloudIndex(runtimeState.sessions), &runtimeState);
        } else if (!runtimeState.sessions.empty()) {
            runtimeState.statusMessage = "No startup point cloud is available. Use the Layers panel to load a gSplat.";
        } else {
            runtimeState.errorMessage = "No point clouds or gSplats were discovered in the Data directory.";
        }
    } else if (runtimeState.sessions.empty()) {
        runtimeState.errorMessage = "No point clouds or gSplats were discovered in the Data directory.";
    }

    while (!window.ShouldClose()) {
        window.PollEvents();
        if (window.ShouldClose()) {
            break;
        }

        if (viewport.has_value()) {
            PollPendingLayerLoad(&runtimeState, &viewport.value());
            if (!runtimeState.offlineRenderJob.active) {
                StartQueuedLayerLoadIfIdle(&runtimeState);
            }
            viewport->BeginUiFrame();
            if (runtimeState.offlineRenderJob.active) {
                ProcessOfflineRenderJobStep(&runtimeState, &viewport.value());
            }
            const bool pauseLiveViewport =
                runtimeState.offlineRenderJob.active && runtimeState.pauseLiveViewportDuringExport;

            DrawControlsWindow(&runtimeState, &viewport.value());
            DrawDiagnosticsWindow(&runtimeState, viewport.value());
            DrawLoadingOverlay(runtimeState);
            DrawOfflineRenderOverlay(&runtimeState);
            if (!pauseLiveViewport) {
                DrawAnimationViewportOverlay(&runtimeState, viewport.value());
                DrawWaterRegionOverlay(&runtimeState, viewport.value());
                UpdateCameraFromInput(&runtimeState, viewport.value());
                UpdateAnimationPlayback(&runtimeState);
                UpdateCameraShotPlayback(&runtimeState);
                UpdatePerformanceInteractionState(&runtimeState, viewport.value());
            }
            PrunePreviewLodSampleCaches(&runtimeState);
            if (!pauseLiveViewport) {
                DrawPivotOverlay(runtimeState, viewport.value());
                DrawWaterPathDebugOverlay(&runtimeState, viewport.value());
                DrawWaterEmitterOverlay(&runtimeState, viewport.value());
            }
            viewport->SetDiagnosticsEnabled(true);
            viewport->SetSceneCachingEnabled(!runtimeState.projectSettings.constantUpdateView);
            viewport->SetLiveSceneRenderingEnabled(!pauseLiveViewport);
            if (!pauseLiveViewport) {
                const float previewFlowTimeSeconds =
                    runtimeState.projectSettings.liveVisualEffects
                        ? std::chrono::duration<float>(
                              std::chrono::steady_clock::now() - runtimeState.startedAt)
                              .count()
                        : 0.0F;
                const auto renderState = BuildRenderState(
                    runtimeState,
                    viewport.value(),
                    previewFlowTimeSeconds);
                const bool previewLiveEffectsAffectScene =
                    runtimeState.projectSettings.liveVisualEffects &&
                    !FastBasicPointRendererActive(runtimeState.projectSettings);
                const auto renderStateSignature = RenderStateSignature(renderState);
                const bool renderStateChanged =
                    !runtimeState.previewRenderStateSignatureValid ||
                    runtimeState.previewRenderStateSignature != renderStateSignature;
                if (renderStateChanged ||
                    runtimeState.projectSettings.constantUpdateView ||
                    previewLiveEffectsAffectScene) {
                    viewport->UpdateRenderState(renderState);
                    runtimeState.previewRenderStateSignature = renderStateSignature;
                    runtimeState.previewRenderStateSignatureValid = true;
                }
            } else {
                runtimeState.previewRenderStateSignatureValid = false;
            }
            viewport->DrawFrame();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds{16});
        }
    }

    StopBackgroundWorkForShutdown(&runtimeState);
    if (viewport.has_value()) {
        viewport->WaitIdle();
    }

    return 0;
}

}  // namespace invisible_places::app
