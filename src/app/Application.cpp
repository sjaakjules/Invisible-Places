#include "app/Application.hpp"

#include "camera/AnimationPath.hpp"
#include "camera/CameraPath.hpp"
#include "camera/CameraShot.hpp"
#include "InvisiblePlacesBuildConfig.hpp"
#include "camera/OrbitCamera.hpp"
#include "io/AssetDiscovery.hpp"
#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
#include "output/ExrWriter.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "output/RenderPreset.hpp"
#include "output/VideoWriter.hpp"
#include "platform/VulkanRuntimeConfig.hpp"
#include "platform/Window.hpp"
#include "platform/WindowTitle.hpp"
#include "renderer/core/VulkanViewportShell.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "scene/SceneCatalog.hpp"
#include "serialization/ProjectDocument.hpp"
#include "style/RenderParameterBinding.hpp"
#include "ui/SidePanelState.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
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
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stop_token>
#include <string_view>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

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
using PointCloudPreviewLodMode = invisible_places::renderer::pointcloud::PointCloudPreviewLodMode;
using GaussianSplatStyleState = invisible_places::renderer::gsplat::GaussianSplatStyleState;
using GaussianSplatColorMode = invisible_places::renderer::gsplat::GaussianSplatColorMode;
using GaussianSplatDebugMode = invisible_places::renderer::gsplat::GaussianSplatDebugMode;
using GaussianSplatQualityMode = invisible_places::renderer::gsplat::GaussianSplatQualityMode;
using AnimationPath = invisible_places::camera::AnimationPath;
using CameraShot = invisible_places::camera::CameraShot;
using RenderParameterBinding = invisible_places::style::RenderParameterBinding;
using ParameterSourceMode = invisible_places::style::ParameterSourceMode;
using FieldMapFlags = invisible_places::style::FieldMapFlags;
using ProjectDocument = invisible_places::serialization::ProjectDocument;
using ProjectLayerDocument = invisible_places::serialization::ProjectLayerDocument;
using PointCloudStylePresetDocument = invisible_places::serialization::PointCloudStylePresetDocument;
using RenderJobSettings = invisible_places::output::RenderJobSettings;
using LayerLoadResult = std::variant<
    invisible_places::io::PointCloudLoadResult,
    invisible_places::io::GaussianSplatLoadResult>;

constexpr float kPi = 3.14159265358979323846F;
constexpr std::size_t kMaxPivotSamples = 65536;
constexpr std::uint64_t kDefaultInteractivePointCap = 10'000'000ULL;
constexpr std::uint64_t kPointCloudPreviewLodTarget = 10'000'000ULL;
constexpr auto kPerformanceInteractionHold = std::chrono::milliseconds{300};

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
    bool showStatusOverlay = true;
    bool autoLowerGsplatQualityWhileNavigating = true;
    PointCloudPreviewLodMode pointCloudPreviewLodMode = PointCloudPreviewLodMode::AutoCameraLod;
    std::uint64_t interactivePointCap = kDefaultInteractivePointCap;
    float gaussianSplatFootprintBoost = 1.5F;
    GsplatTransformConvention gsplatTransformConvention = GsplatTransformConvention::AsEncoded;
};

struct PersistenceState {
    std::string projectFilePath;
    std::string pointStylePresetPath;
    std::string animationDirectoryPath;
    std::vector<std::size_t> queuedLoadIndices;
};

struct CameraPanelState {
    std::string draftShotName = "Shot 1";
    std::optional<std::size_t> selectedShotIndex;
    std::optional<std::size_t> blendFromIndex;
    std::optional<std::size_t> blendToIndex;
    std::vector<std::size_t> pathShotIndices;
    std::optional<std::size_t> selectedPathItemIndex;
    float blendAmount = 0.0F;
    std::uint32_t defaultDurationFrames = 90;
    std::uint32_t pathDurationFrames = 180;
    bool liveBlend = true;
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

struct AnimationPanelState {
    std::optional<AnimationPath> currentPath;
    std::string currentFilePath;
    std::string draftAnimationName = "Animation 1";
    std::string mp4OutputPath;
    std::vector<std::filesystem::path> availableFiles;
    std::optional<std::size_t> selectedFileIndex;
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
    std::string errorMessage;
    std::uint32_t setupViewportWidth = 1;
    std::uint32_t setupViewportHeight = 1;
    std::uint32_t writtenFrameCount = 0;
    std::size_t pendingFrameCount = 0;
    bool previewDensity = true;
    bool writerFinishRequested = false;
    glm::vec4 exportBackgroundColor{0.0F, 0.0F, 0.0F, 0.0F};
    float exportGaussianSplatFootprintBoost = 1.5F;
    std::vector<invisible_places::renderer::core::SceneRenderState::PointCloudLayerState> exportPointCloudLayers;
    std::shared_ptr<AnimationExportWriterState> writerState;
    std::shared_ptr<struct OfflineRenderProgressState> progressState;
    std::jthread worker;
};

struct OfflineRenderProgressState {
    std::mutex mutex;
    bool cancelRequested = false;
    bool completed = false;
    std::uint32_t currentFrame = 0;
    std::uint32_t currentTile = 0;
    std::filesystem::path lastOutputPath;
    std::string statusMessage;
    std::string errorMessage;
};

void EnsureCameraShotSelections(CameraPanelState* panelState, std::size_t shotCount);
void RequestAnimationExportWriterCancellation(OfflineRenderJobState* job);

struct OfflinePointLayerSnapshot {
    std::shared_ptr<const invisible_places::io::LoadedPointCloud> cloud;
    PointCloudStyleState style{};
    bool hasSourceRgb = false;
    glm::mat4 localToWorld{1.0F};
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

struct PreviewRuntimeState {
    std::vector<PreviewLayerSession> sessions;
    std::optional<std::size_t> selectedSessionIndex;
    std::optional<PendingLayerLoad> pendingLoad;
    invisible_places::camera::OrbitCamera camera;
    bool preserveProjectCameraOnNextLayerActivation = false;
    CameraInteractionState cameraInteraction{};
    PerformanceInteractionState performanceInteraction{};
    PivotOverlayState pivotOverlay{};
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

std::uint64_t EffectivePointDrawCount(
    const PreviewRuntimeState& runtimeState,
    const PreviewLayerSession& session) {
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
    const PreviewLayerSession& session) {
    if (session.kind != LayerKind::PointCloud ||
        session.pointBudget.UsesSampledIndices() ||
        session.pointBudget.activePoints == 0 ||
        kPointCloudPreviewLodTarget >= session.pointBudget.activePoints) {
        return 0;
    }

    return static_cast<std::uint32_t>(std::min<std::uint64_t>(
        kPointCloudPreviewLodTarget,
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

    const auto requestedCount = RequestedPreviewLodSampleCount(session);
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
        invisible_places::renderer::pointcloud::GenerateDeterministicSampleIndices(
            session.pointBudget.activePoints,
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
    session->pointStyle.depthAlphaThreshold = std::clamp(session->pointStyle.depthAlphaThreshold, 0.0F, 1.0F);

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

int ResizeInputTextCallback(ImGuiInputTextCallbackData* data) {
    if (data == nullptr || data->EventFlag != ImGuiInputTextFlags_CallbackResize) {
        return 0;
    }

    auto* text = static_cast<std::string*>(data->UserData);
    if (text == nullptr) {
        return 0;
    }

    text->resize(static_cast<std::size_t>(data->BufTextLen));
    data->Buf = text->data();
    return 0;
}

bool InputTextString(const char* label, std::string* value) {
    if (value == nullptr) {
        return false;
    }

    if (value->capacity() < 255U) {
        value->reserve(255U);
    }

    return ImGui::InputText(
        label,
        value->data(),
        value->capacity() + 1U,
        ImGuiInputTextFlags_CallbackResize,
        ResizeInputTextCallback,
        value);
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
        ImGui::Spacing();
    }
    return open;
}

void EndPanelSection() {
    ImGui::Spacing();
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
        if (selectedSession.loaded && selectedSession.hasFocusPoint) {
            return selectedSession.focusPoint;
        }
        if (selectedSession.loaded && selectedSession.bounds.valid) {
            return BoundsCenter(selectedSession.bounds);
        }
    }

    for (const auto& session : runtimeState.sessions) {
        if (!session.loaded || !session.visible) {
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
        if (!session.loaded || !session.visible || session.pivotSamples.empty()) {
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
    std::size_t sessionIndex) {
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
        CurrentAspectRatio(viewport));
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
    runtimeState->selectedSessionIndex = sessionIndex;
    if (!hadVisibleLayersBefore && runtimeState->preserveProjectCameraOnNextLayerActivation) {
        runtimeState->preserveProjectCameraOnNextLayerActivation = false;
        SyncPivotOverlayToCamera(runtimeState);
    } else if (!hadVisibleLayersBefore) {
        FocusSessionLayer(runtimeState, *viewport, sessionIndex);
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
        FocusSessionLayer(runtimeState, *viewport, sessionIndex);
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

ProjectDocument BuildProjectDocument(const PreviewRuntimeState& runtimeState) {
    ProjectDocument document;
    document.projectName = "Invisible Places";
    document.backgroundColor = runtimeState.projectSettings.backgroundColor;
    document.sidePanelPinned = runtimeState.sidePanel.pinned;
    document.autoLowerGsplatQualityWhileNavigating =
        runtimeState.projectSettings.autoLowerGsplatQualityWhileNavigating;
    document.pointCloudPreviewLodMode = runtimeState.projectSettings.pointCloudPreviewLodMode;
    document.interactivePointCap = runtimeState.projectSettings.interactivePointCap;
    document.cameraState = runtimeState.camera.CaptureState();
    document.cameraShots = runtimeState.cameraShots;
    document.cameraPathShotIndices = runtimeState.cameraPanel.pathShotIndices;
    document.cameraPathDurationFrames = runtimeState.cameraPanel.pathDurationFrames;
    document.lastAnimationPath = runtimeState.animationPanel.currentFilePath;
    document.renderJobSettings = runtimeState.renderSettings;

    if (runtimeState.selectedSessionIndex.has_value()) {
        document.selectedLayerPath =
            runtimeState.sessions[runtimeState.selectedSessionIndex.value()].sourcePath;
    }

    document.layers.reserve(runtimeState.sessions.size());
    for (const auto& session : runtimeState.sessions) {
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
    runtimeState->sidePanel.pinned = document.sidePanelPinned;
    runtimeState->projectSettings.autoLowerGsplatQualityWhileNavigating =
        document.autoLowerGsplatQualityWhileNavigating;
    runtimeState->projectSettings.pointCloudPreviewLodMode = document.pointCloudPreviewLodMode;
    runtimeState->projectSettings.interactivePointCap = document.interactivePointCap;
    auto renderSettings = document.renderJobSettings;
    if (renderSettings.outputDirectory.empty() && !runtimeState->renderSettings.outputDirectory.empty()) {
        renderSettings.outputDirectory = runtimeState->renderSettings.outputDirectory;
    }
    runtimeState->renderSettings = renderSettings;
    runtimeState->cameraShots = document.cameraShots;
    runtimeState->cameraPanel.selectedShotIndex.reset();
    runtimeState->cameraPanel.blendFromIndex.reset();
    runtimeState->cameraPanel.blendToIndex.reset();
    runtimeState->cameraPanel.pathShotIndices = document.cameraPathShotIndices;
    runtimeState->cameraPanel.selectedPathItemIndex.reset();
    runtimeState->cameraPanel.pathDurationFrames =
        std::max<std::uint32_t>(1U, document.cameraPathDurationFrames);
    runtimeState->animationPanel.currentFilePath = document.lastAnimationPath.string();
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

        if (session.kind == LayerKind::PointCloud && layerIt->pointStyle.has_value()) {
            session.pointStyle = layerIt->pointStyle.value();
        }

        if (session.kind == LayerKind::PointCloud && layerIt->pointBudgetActivePoints > 0) {
            session.pointBudget = MakePreviewPointBudgetState(session, layerIt->pointBudgetActivePoints);
            ClearPreviewLodSampleCache(&session);
            if (session.loaded) {
                viewport->UpdatePointBudget(sessionIndex, session.pointBudget.sampledIndices);
            }
        }

        SanitizePointCloudStyle(&session);

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

std::filesystem::path AnimationFilePathForName(
    const PreviewRuntimeState& runtimeState,
    const std::string& animationName) {
    return AnimationDirectory(runtimeState) / (SanitizeAnimationFileStem(animationName) + ".ipanim.json");
}

std::filesystem::path DefaultAnimationMp4Path(const PreviewRuntimeState& runtimeState) {
    const auto animationName =
        runtimeState.animationPanel.currentPath.has_value()
            ? runtimeState.animationPanel.currentPath->name
            : runtimeState.animationPanel.draftAnimationName;
    const auto outputDirectory = runtimeState.renderSettings.outputDirectory.empty()
                                     ? DefaultRenderOutputDirectory(std::filesystem::path{})
                                     : std::filesystem::path{runtimeState.renderSettings.outputDirectory};
    return outputDirectory / (SanitizeAnimationFileStem(animationName) + ".mp4");
}

bool IsMp4Extension(const std::filesystem::path& path) {
    auto extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return extension == ".mp4";
}

std::filesystem::path AnimationMp4OutputPath(const PreviewRuntimeState& runtimeState) {
    const auto outputDirectory = runtimeState.renderSettings.outputDirectory.empty()
                                     ? DefaultRenderOutputDirectory(std::filesystem::path{})
                                     : std::filesystem::path{runtimeState.renderSettings.outputDirectory};
    auto outputPath = DefaultAnimationMp4Path(runtimeState);
    if (!runtimeState.animationPanel.mp4OutputPath.empty()) {
        const auto requestedPath = std::filesystem::path{runtimeState.animationPanel.mp4OutputPath};
        outputPath = requestedPath.is_absolute() ? requestedPath : outputDirectory / requestedPath;
    }

    if (!IsMp4Extension(outputPath)) {
        if (outputPath.has_extension()) {
            outputPath.replace_extension(".mp4");
        } else {
            outputPath += ".mp4";
        }
    }
    return outputPath;
}

void RefreshAnimationFileList(AnimationPanelState* panelState, const std::filesystem::path& animationDirectory) {
    if (panelState == nullptr) {
        return;
    }

    panelState->availableFiles.clear();
    std::error_code createError;
    std::filesystem::create_directories(animationDirectory, createError);
    if (createError) {
        return;
    }

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
            panelState->availableFiles.push_back(path);
        }
    }
    std::sort(panelState->availableFiles.begin(), panelState->availableFiles.end());
    if (panelState->selectedFileIndex.has_value() &&
        panelState->selectedFileIndex.value() >= panelState->availableFiles.size()) {
        panelState->selectedFileIndex = panelState->availableFiles.empty()
                                            ? std::nullopt
                                            : std::optional<std::size_t>{panelState->availableFiles.size() - 1U};
    }
}

float AnimationDurationSeconds(const AnimationPath& path) {
    const auto minimumFrames = path.keys.size() > 1U
                                   ? static_cast<std::uint32_t>(path.keys.size() - 1U)
                                   : 1U;
    return static_cast<float>(std::max(path.durationFrames, minimumFrames)) / 30.0F;
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

    std::string errorMessage;
    if (!invisible_places::serialization::SaveAnimationPath(path, outputPath, &errorMessage)) {
        runtimeState->errorMessage = errorMessage.empty() ? "Failed to save animation path." : errorMessage;
        runtimeState->statusMessage.clear();
        return false;
    }

    const bool savingCurrentPath =
        runtimeState->animationPanel.currentPath.has_value() &&
        &path == &runtimeState->animationPanel.currentPath.value();
    if (!savingCurrentPath) {
        runtimeState->animationPanel.currentPath = path;
    }
    runtimeState->animationPanel.currentFilePath = outputPath.string();
    runtimeState->animationPanel.draftAnimationName = path.name;
    runtimeState->animationPanel.dirty = false;
    RefreshAnimationFileList(&runtimeState->animationPanel, AnimationDirectory(*runtimeState));
    runtimeState->statusMessage = "Saved animation path: " + outputPath.filename().string() + ".";
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

    runtimeState->animationPanel.currentPath = path.value();
    runtimeState->animationPanel.currentFilePath = inputPath.string();
    runtimeState->animationPanel.draftAnimationName = path->name;
    runtimeState->animationPanel.selectedKeyIndex = path->keys.empty() ? std::nullopt : std::optional<std::size_t>{0};
    runtimeState->animationPanel.scrubAmount = 0.0F;
    runtimeState->animationPanel.previewDepthOfField = false;
    runtimeState->animationPanel.dirty = false;
    runtimeState->animationPlayback.active = false;
    runtimeState->cameraPlayback.active = false;
    ApplyAnimationEvaluation(runtimeState, runtimeState->animationPanel.currentPath.value(), 0.0F, false);
    runtimeState->statusMessage = "Loaded animation path: " + inputPath.filename().string() + ".";
    runtimeState->errorMessage.clear();
    return true;
}

void SaveCurrentCameraPathAsAnimation(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    const auto pathShots = BuildPanelCameraPathShots(*runtimeState);
    if (pathShots.size() < 2U) {
        runtimeState->errorMessage = "Add at least two camera path entries before saving an animation.";
        runtimeState->statusMessage.clear();
        return;
    }

    auto animationPath = invisible_places::camera::BuildAnimationPathFromCameraShots(
        runtimeState->animationPanel.draftAnimationName,
        pathShots,
        runtimeState->cameraPanel.pathDurationFrames,
        runtimeState->animationPanel.currentPath.has_value()
            ? runtimeState->animationPanel.currentPath->apertureFStops
            : 8.0F);
    const auto outputPath = AnimationFilePathForName(*runtimeState, animationPath.name);
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
    return "Shot " + std::to_string(runtimeState.cameraShots.size() + 1U);
}

void SaveCurrentCameraShot(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return;
    }

    CameraShot shot;
    shot.name = runtimeState->cameraPanel.draftShotName.empty()
                    ? NextCameraShotName(*runtimeState)
                    : runtimeState->cameraPanel.draftShotName;
    shot.durationFrames = std::max<std::uint32_t>(1U, runtimeState->cameraPanel.defaultDurationFrames);
    shot.state = runtimeState->camera.CaptureState();
    const auto savedDurationFrames = shot.durationFrames;
    runtimeState->cameraShots.push_back(std::move(shot));

    const auto savedIndex = runtimeState->cameraShots.size() - 1U;
    const bool appendAddsSegment = !runtimeState->cameraPanel.pathShotIndices.empty();
    runtimeState->cameraPanel.selectedShotIndex = savedIndex;
    runtimeState->cameraPanel.pathShotIndices.push_back(savedIndex);
    runtimeState->cameraPanel.selectedPathItemIndex = runtimeState->cameraPanel.pathShotIndices.size() - 1U;
    if (appendAddsSegment) {
        runtimeState->cameraPanel.pathDurationFrames = static_cast<std::uint32_t>(std::min<std::uint64_t>(
            std::numeric_limits<std::uint32_t>::max(),
            static_cast<std::uint64_t>(runtimeState->cameraPanel.pathDurationFrames) + savedDurationFrames));
    }
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

        layers.push_back(
            {.cloud = session.offlinePointCloud,
             .style = session.pointStyle,
             .hasSourceRgb = session.hasSourceRgb,
             .localToWorld = glm::mat4{1.0F}});
    }
    return layers;
}

std::vector<invisible_places::output::OfflinePointLayer> BuildOfflinePointLayers(
    const std::vector<OfflinePointLayerSnapshot>& snapshots) {
    std::vector<invisible_places::output::OfflinePointLayer> layers;
    layers.reserve(snapshots.size());
    for (const auto& snapshot : snapshots) {
        if (snapshot.cloud == nullptr || snapshot.cloud->positions.empty()) {
            continue;
        }

        layers.push_back(
            {.cloud = snapshot.cloud.get(),
             .style = snapshot.style,
             .hasSourceRgb = snapshot.hasSourceRgb,
             .localToWorld = snapshot.localToWorld});
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
    bool previewDensity) {
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

std::vector<invisible_places::renderer::core::SceneRenderState::PointCloudLayerState>
BuildAnimationExportPointCloudLayerSnapshot(
    const PreviewRuntimeState& runtimeState,
    bool previewDensity) {
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
            previewDensity);
        layers.push_back(
            {.layerId = sessionIndex,
             .style = session.pointStyle,
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
    renderState.nearPlane = camera.NearPlane();
    renderState.farPlane = camera.FarPlane();
    renderState.hasDepthOfField = cameraState.hasDepthOfField;
    renderState.focusDistance = cameraState.focusDistance;
    renderState.apertureFStops = cameraState.apertureFStops;
    renderState.depthOfFieldMaxBlurPixels = cameraState.depthOfFieldMaxBlurPixels;
    renderState.gaussianSplatFootprintBoost = job.exportGaussianSplatFootprintBoost;
    renderState.pointSizeScale = invisible_places::output::ComputePointSizePixelScale(
        width,
        height,
        job.setupViewportWidth,
        job.setupViewportHeight);
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

void UpdateOfflineRenderProgress(
    const std::shared_ptr<OfflineRenderProgressState>& progress,
    std::uint32_t frame,
    std::uint32_t tile,
    const std::filesystem::path& lastOutputPath = {}) {
    if (progress == nullptr) {
        return;
    }

    std::scoped_lock lock(progress->mutex);
    progress->currentFrame = frame;
    progress->currentTile = tile;
    if (!lastOutputPath.empty()) {
        progress->lastOutputPath = lastOutputPath;
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

        for (std::uint32_t frameIndex = 0; frameIndex < frames.size(); ++frameIndex) {
            for (std::uint32_t tileIndex = 0; tileIndex < tiles.size(); ++tileIndex) {
                if (OfflineRenderCancellationRequested(progress, stopToken)) {
                    CompleteOfflineRenderProgress(progress, "EXR render cancelled.");
                    return;
                }

                UpdateOfflineRenderProgress(progress, frameIndex, tileIndex);
                invisible_places::output::RenderPointCloudTile(
                    offlineLayers,
                    frames[frameIndex],
                    tiles[tileIndex],
                    &image);
                UpdateOfflineRenderProgress(progress, frameIndex, tileIndex + 1U);
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
    invisible_places::output::HalfRgbaExrImage image) {
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
             .image = std::move(image)});
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
    if (!job->writerState->lastOutputPath.empty()) {
        job->lastOutputPath = job->writerState->lastOutputPath;
    }
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
    std::filesystem::path videoOutputPath,
    std::uint32_t totalFrames,
    std::shared_ptr<AnimationExportWriterState> writerState) {
    FILE* videoPipe = nullptr;
    try {
        if (mode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
            const auto command = invisible_places::output::BuildFfmpegRawRgbaCommand(
                invisible_places::output::DefaultFfmpegExecutablePath(),
                settings.width,
                settings.height,
                settings.framesPerSecond,
                videoOutputPath);
            videoPipe = ::popen(command.c_str(), "w");
            if (videoPipe == nullptr) {
                CompleteAnimationExportWriter(
                    writerState,
                    {},
                    "Failed to start ffmpeg for Fast Preview MP4 export.");
                return;
            }
        }

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
                    const auto closeError = CloseAnimationExportVideoPipe(&videoPipe, videoOutputPath);
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
            if (mode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
                const auto frameBytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(payload.image);
                if (frameBytes.empty()) {
                    CompleteAnimationExportWriter(
                        writerState,
                        {},
                        "GPU readback did not produce a valid MP4 frame.");
                    return;
                }
                const auto written = std::fwrite(frameBytes.data(), 1U, frameBytes.size(), videoPipe);
                if (written != frameBytes.size()) {
                    CompleteAnimationExportWriter(
                        writerState,
                        {},
                        "Failed to write raw frame data to ffmpeg.");
                    return;
                }
                outputPath = videoOutputPath;
            } else {
                outputPath = invisible_places::output::RenderFramePath(settings, payload.outputFrameIndex);
                std::string errorMessage;
                if (!invisible_places::output::WriteExrImage(payload.image, outputPath, &errorMessage)) {
                    CompleteAnimationExportWriter(writerState, {}, errorMessage);
                    return;
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

        const auto closeError = CloseAnimationExportVideoPipe(&videoPipe, videoOutputPath);
        if (!closeError.empty()) {
            CompleteAnimationExportWriter(writerState, {}, closeError);
            return;
        }

        CompleteAnimationExportWriter(
            writerState,
            mode == invisible_places::output::AnimationExportMode::FastPreviewMp4
                ? "Fast Preview MP4 complete: " + videoOutputPath.string() + "."
                : "HQ preview-density EXR stack complete: " + settings.outputDirectory + ".");
    } catch (const std::exception& error) {
        const auto closeError = CloseAnimationExportVideoPipe(&videoPipe, videoOutputPath);
        CompleteAnimationExportWriter(
            writerState,
            {},
            closeError.empty()
                ? "Animation export writer failed: " + std::string{error.what()}
                : closeError);
    }
}

void StartAnimationExportJob(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || runtimeState->offlineRenderJob.active) {
        return;
    }

    auto& settings = runtimeState->renderSettings;
    settings.width = std::max<std::uint32_t>(1U, settings.width);
    settings.height = std::max<std::uint32_t>(1U, settings.height);
    settings.framesPerSecond = std::max<std::uint32_t>(1U, settings.framesPerSecond);
    settings.tileSize = std::max<std::uint32_t>(1U, settings.tileSize);

    if (settings.outputDirectory.empty()) {
        settings.outputDirectory = DefaultRenderOutputDirectory(std::filesystem::path{}).string();
    }

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

    const bool exportUsesPreviewDensity =
        runtimeState->animationPanel.exportMode != invisible_places::output::AnimationExportMode::FastPreviewMp4 &&
        runtimeState->animationPanel.exportPreviewDensity;
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

    std::filesystem::path videoOutputPath;
    if (runtimeState->animationPanel.exportMode ==
        invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        const auto ffmpegPath = invisible_places::output::DefaultFfmpegExecutablePath();
        if (!invisible_places::output::FfmpegExecutableAvailable(ffmpegPath)) {
            runtimeState->errorMessage =
                "Fast MP4 export requires ffmpeg at " + ffmpegPath.string() + ".";
            runtimeState->statusMessage.clear();
            return;
        }

        videoOutputPath = AnimationMp4OutputPath(*runtimeState);
        std::error_code createError;
        if (!videoOutputPath.parent_path().empty()) {
            std::filesystem::create_directories(videoOutputPath.parent_path(), createError);
            if (createError) {
                runtimeState->errorMessage = "Failed to create MP4 output directory: " + createError.message();
                runtimeState->statusMessage.clear();
                return;
            }
        }

    }

    auto exportPointCloudLayers = BuildAnimationExportPointCloudLayerSnapshot(
        *runtimeState,
        exportUsesPreviewDensity);
    if (exportPointCloudLayers.empty()) {
        runtimeState->errorMessage = "No visible loaded LiDAR layers are available for animation export.";
        runtimeState->statusMessage.clear();
        return;
    }

    const auto setupSize = viewport != nullptr ? CurrentUiViewportSize(*viewport) : ImVec2{1.0F, 1.0F};
    auto writerState = std::make_shared<AnimationExportWriterState>();
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
        .setupViewportWidth = static_cast<std::uint32_t>(std::max(1.0F, setupSize.x)),
        .setupViewportHeight = static_cast<std::uint32_t>(std::max(1.0F, setupSize.y)),
        .previewDensity = exportUsesPreviewDensity,
        .exportBackgroundColor = glm::vec4{
            runtimeState->projectSettings.backgroundColor[0],
            runtimeState->projectSettings.backgroundColor[1],
            runtimeState->projectSettings.backgroundColor[2],
            0.0F,
        },
        .exportGaussianSplatFootprintBoost = runtimeState->projectSettings.gaussianSplatFootprintBoost,
        .exportPointCloudLayers = std::move(exportPointCloudLayers),
        .writerState = writerState,
    };
    runtimeState->offlineRenderJob.worker = std::jthread{
        RunAnimationExportWriter,
        runtimeState->offlineRenderJob.mode,
        runtimeState->offlineRenderJob.settings,
        runtimeState->offlineRenderJob.videoOutputPath,
        static_cast<std::uint32_t>(runtimeState->offlineRenderJob.frames.size()),
        writerState};
    runtimeState->cameraPlayback.active = false;
    runtimeState->animationPlayback.active = false;
    runtimeState->statusMessage =
        runtimeState->animationPanel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4
            ? "Encoding full-cloud Fast MP4..."
            : "Rendering HQ preview-density EXR stack on GPU...";
    runtimeState->errorMessage.clear();
}

void FinishOfflineRenderJob(
    PreviewRuntimeState* runtimeState,
    const std::string& statusMessage,
    const std::string& errorMessage = {}) {
    if (runtimeState == nullptr) {
        return;
    }

    auto finalStatusMessage = statusMessage;
    auto finalErrorMessage = errorMessage;
    auto& job = runtimeState->offlineRenderJob;
    if (job.worker.joinable()) {
        if (job.writerState != nullptr && !job.writerState->completed) {
            RequestAnimationExportWriterCancellation(&job);
        }
        job.worker = std::jthread{};
    }

    job.active = false;
    job.cancelRequested = false;
    job.writerFinishRequested = false;
    job.writerState.reset();
    runtimeState->statusMessage = finalStatusMessage;
    runtimeState->errorMessage = finalErrorMessage;
}

void RequestOfflineRenderCancellation(OfflineRenderJobState* job) {
    if (job == nullptr) {
        return;
    }

    job->cancelRequested = true;
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
            FinishOfflineRenderJob(runtimeState, writerStatus, writerError);
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
        (job.mode == invisible_places::output::AnimationExportMode::FastPreviewMp4
             ? "Capturing MP4 frame "
             : "Capturing HQ EXR frame ") +
        std::to_string(job.currentFrame + 1U) +
        " / " + std::to_string(job.frames.size()) + " on GPU (" + elapsed + ").";

    try {
        const auto renderState = BuildPointCloudExrRenderState(
            job,
            job.frames[job.currentFrame],
            job.settings.width,
            job.settings.height);
        if (renderState.pointCloudLayers.empty()) {
            FinishOfflineRenderJob(runtimeState, {}, "No visible loaded LiDAR layers are available for rendering.");
            return;
        }

        const invisible_places::renderer::core::PointCloudExrFrameRequest request{
            .renderState = renderState,
            .width = job.settings.width,
            .height = job.settings.height,
            .previewDensity = job.previewDensity,
        };
        auto image = viewport->RenderPointCloudExrFrame(request);
        const auto outputFrameIndex = job.settings.startFrame + job.currentFrame;
        if (!QueueAnimationExportFrame(&job, outputFrameIndex, std::move(image))) {
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
    };
    int exportModeIndex =
        panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4 ? 0 : 1;
    if (ImGui::Combo("Export Mode", &exportModeIndex, exportModeLabels, IM_ARRAYSIZE(exportModeLabels))) {
        panel.exportMode = exportModeIndex == 0
                               ? invisible_places::output::AnimationExportMode::FastPreviewMp4
                               : invisible_places::output::AnimationExportMode::HqPreviewDensityExr;
    }
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        ImGui::TextDisabled("Fast MP4: full-cloud beauty export, no AOVs.");
    } else {
        ImGui::TextDisabled("HQ EXR: preview-density AOV export; optimized for visual parity, not full-source density.");
    }

    InputTextString("Output Folder", &settings.outputDirectory);
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        InputTextString("MP4 Name/File", &panel.mp4OutputPath);
        if (panel.mp4OutputPath.empty()) {
            ImGui::TextDisabled("Default MP4: %s", AnimationMp4OutputPath(*runtimeState).string().c_str());
        } else {
            ImGui::TextDisabled("MP4 output: %s", AnimationMp4OutputPath(*runtimeState).string().c_str());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("A bare name saves inside Output Folder. Absolute paths save exactly there.");
        }
    }

    int width = static_cast<int>(settings.width);
    int height = static_cast<int>(settings.height);
    int fps = static_cast<int>(settings.framesPerSecond);
    int startFrame = static_cast<int>(settings.startFrame);
    int endFrame = static_cast<int>(settings.endFrame);

    if (ImGui::InputInt("Width", &width)) {
        settings.width = static_cast<std::uint32_t>(std::max(1, width));
    }
    if (ImGui::InputInt("Height", &height)) {
        settings.height = static_cast<std::uint32_t>(std::max(1, height));
    }
    if (ImGui::InputInt("Frame Rate", &fps)) {
        settings.framesPerSecond = static_cast<std::uint32_t>(std::max(1, fps));
    }
    if (ImGui::InputInt("Start Frame", &startFrame)) {
        settings.startFrame = static_cast<std::uint32_t>(std::max(0, startFrame));
    }
    if (ImGui::InputInt("End Frame", &endFrame)) {
        settings.endFrame = static_cast<std::uint32_t>(std::max(0, endFrame));
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Use 0 to render through the final interpolated frame.");
    }
    if (viewport != nullptr && ImGui::Button("Use Viewport Size")) {
        const auto viewportSize = CurrentUiViewportSize(*viewport);
        settings.width = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.x));
        settings.height = static_cast<std::uint32_t>(std::max(1.0F, viewportSize.y));
    }
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4) {
        ImGui::TextDisabled("Point density: full source clouds.");
    } else {
        ImGui::SameLine();
        ImGui::Checkbox("Preview Density", &panel.exportPreviewDensity);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Uses the same draw counts and interactive sample buffers as animation playback.");
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
            : "EXR stack",
        previewFrames.size());
    if (panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4 &&
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
            "Export runs in its own window; the viewport remains editable.");
        if (!job.lastOutputPath.empty()) {
            ImGui::TextWrapped("Last: %s", job.lastOutputPath.string().c_str());
        }
        if (ImGui::Button(job.cancelRequested ? "Cancelling..." : "Cancel Export")) {
            RequestOfflineRenderCancellation(&job);
        }
        EndPanelSection();
        return;
    }

    const char* exportButtonLabel =
        panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4
            ? "Export Fast MP4"
            : "Export HQ EXR Stack";
    if (ImGui::Button(exportButtonLabel)) {
        StartAnimationExportJob(runtimeState, viewport);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            panel.exportMode == invisible_places::output::AnimationExportMode::FastPreviewMp4
                ? "Captures the active animation as a full-cloud H.264 MP4."
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
    ImGui::Begin("Animation Export", nullptr, flags);

    const float frameProgress =
        job.frames.empty()
            ? 0.0F
            : static_cast<float>(job.writtenFrameCount) /
                  static_cast<float>(job.frames.size());
    ImGui::TextUnformatted(
        job.mode == invisible_places::output::AnimationExportMode::FastPreviewMp4
            ? "Encoding Fast Preview MP4"
            : "Rendering HQ Preview-Density EXR");
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
    ImGui::TextUnformatted(job.previewDensity ? "Renderer: GPU preview density" : "Renderer: GPU full source");
    ImGui::Text(
        "Elapsed: %s",
        FormatElapsedTime(std::chrono::steady_clock::now() - job.startedAt).c_str());
    ImGui::TextWrapped(
        "Output: %s",
        job.mode == invisible_places::output::AnimationExportMode::FastPreviewMp4
            ? job.videoOutputPath.string().c_str()
            : job.settings.outputDirectory.c_str());
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
                           ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;
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
            MoveAnimationKeyPoint(
                &path,
                panel.drag.keyIndex,
                panel.drag.target,
                panel.drag.startWorldPoint + (panel.drag.axisWorld * worldDelta));
            panel.selectedKeyIndex = panel.drag.keyIndex;
            panel.editTarget = panel.drag.target;
            panel.dirty = true;
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
            const ImU32 color = selected ? selectedColor : (target == AnimationEditTarget::Camera ? cameraColor : focusColor);
            drawList->AddCircleFilled(projected->screen, selected ? 6.0F : 4.5F, color, 18);
            drawList->AddCircle(projected->screen, selected ? 8.0F : 6.0F, IM_COL32(0, 0, 0, 170), 18, 2.0F);

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
    if (session.pointStyle.geometryMode == PointCloudGeometryMode::WorldSurfels) {
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
            .defaultConstant = 2.0F,
            .format = "%.2f",
            .hardMin = 0.0F};
}

RenderParameterBinding* PointSizeBinding(PreviewLayerSession* session) {
    if (session == nullptr) {
        return nullptr;
    }
    return session->pointStyle.geometryMode == PointCloudGeometryMode::WorldSurfels
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
    const char* geometryModes[] = {"Screen Sprites", "World Surfels"};
    if (DrawRightAlignedCombo("Geometry", &geometryModeIndex, geometryModes, IM_ARRAYSIZE(geometryModes))) {
        style.geometryMode = static_cast<PointCloudGeometryMode>(geometryModeIndex);
        changed = true;
    }
    if (style.geometryMode == PointCloudGeometryMode::WorldSurfels && !session->hasNormals) {
        ImGui::TextDisabled("No normals were loaded; surfels face the camera.");
    }

    ImGui::Spacing();
    ImGui::TextUnformatted(
        style.geometryMode == PointCloudGeometryMode::WorldSurfels ? "Point Size (mm)" : "Point Size (px)");
    changed |= DrawScalarBindingBody(
        "Point Size",
        PointSizeBinding(session),
        session->scalarFields,
        PointSizeBindingConfig(*session));
    EndPanelSection();
    return changed;
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
        const char* colormaps[] = {"Viridis", "Plasma", "Inferno", "Magma", "Cividis", "Turbo"};
        if (DrawRightAlignedCombo("Colormap", &colormapIndex, colormaps, IM_ARRAYSIZE(colormaps))) {
            style.colormap = static_cast<PointCloudColormapId>(colormapIndex);
            changed = true;
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

void DrawPointCloudStyleSection(PreviewLayerSession* session) {
    auto& style = session->pointStyle;
    bool changed = false;

    changed |= DrawPointCloudPointSettingsSection(session);
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
        DrawPointCloudStyleSection(session);
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

void DrawCameraSection(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
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

    if (BeginPanelSection("Shots")) {
    EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());

    InputTextString("Shot Name", &runtimeState->cameraPanel.draftShotName);
    int defaultDurationFrames = static_cast<int>(runtimeState->cameraPanel.defaultDurationFrames);
    if (ImGui::InputInt("Duration Frames", &defaultDurationFrames)) {
        runtimeState->cameraPanel.defaultDurationFrames =
            static_cast<std::uint32_t>(std::max(1, defaultDurationFrames));
    }
    ImGui::TextDisabled("Playback uses the project default 30 fps timebase.");

    if (ImGui::Button("Save Camera Position")) {
        SaveCurrentCameraShot(runtimeState);
    }

    if (runtimeState->cameraShots.empty()) {
        ImGui::TextUnformatted("No camera shots saved yet.");
        EndPanelSection();
        return;
    }

    ImGui::Spacing();
    if (ImGui::BeginListBox("Saved Shots", ImVec2{-FLT_MIN, 128.0F})) {
        for (std::size_t index = 0; index < runtimeState->cameraShots.size(); ++index) {
            const bool selected = runtimeState->cameraPanel.selectedShotIndex.has_value() &&
                                  runtimeState->cameraPanel.selectedShotIndex.value() == index;
            if (ImGui::Selectable(runtimeState->cameraShots[index].name.c_str(), selected)) {
                runtimeState->cameraPanel.selectedShotIndex = index;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
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
                RemoveCameraShotFromPath(&runtimeState->cameraPanel, selectedIndex);
                runtimeState->cameraShots.erase(runtimeState->cameraShots.begin() + static_cast<std::ptrdiff_t>(selectedIndex));
                runtimeState->cameraPlayback.active = false;
                EnsureCameraShotSelections(&runtimeState->cameraPanel, runtimeState->cameraShots.size());
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
    if (BeginPanelSection("Animation")) {
    if (InputTextString("Animation Name", &panel.draftAnimationName) && panel.currentPath.has_value()) {
        panel.currentPath->name = panel.draftAnimationName;
        panel.dirty = true;
    }
    if (ImGui::Button("Refresh")) {
        RefreshAnimationFileList(&panel, AnimationDirectory(*runtimeState));
    }
    ImGui::SameLine();
    ImGui::Checkbox("Show Splines", &panel.showSplines);

    if (panel.availableFiles.empty()) {
        ImGui::TextDisabled("No saved animation paths found.");
    } else if (ImGui::BeginListBox("Saved Animations", ImVec2{-FLT_MIN, 128.0F})) {
        for (std::size_t index = 0; index < panel.availableFiles.size(); ++index) {
            const bool selected = panel.selectedFileIndex.has_value() && panel.selectedFileIndex.value() == index;
            if (ImGui::Selectable(panel.availableFiles[index].stem().string().c_str(), selected)) {
                panel.selectedFileIndex = index;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndListBox();
    }

    if (panel.selectedFileIndex.has_value() && panel.selectedFileIndex.value() < panel.availableFiles.size()) {
        if (ImGui::Button("Load")) {
            LoadAnimationPathFromFile(runtimeState, panel.availableFiles[panel.selectedFileIndex.value()]);
        }
        ImGui::SameLine();
        if (ImGui::Button("Delete File")) {
            std::error_code removeError;
            const auto deletedPath = panel.availableFiles[panel.selectedFileIndex.value()];
            std::filesystem::remove(deletedPath, removeError);
            if (removeError) {
                runtimeState->errorMessage = "Failed to delete animation: " + removeError.message();
                runtimeState->statusMessage.clear();
            } else {
                runtimeState->statusMessage = "Deleted animation: " + deletedPath.filename().string() + ".";
                runtimeState->errorMessage.clear();
                if (panel.currentFilePath == deletedPath.string()) {
                    panel.currentFilePath.clear();
                    panel.currentPath.reset();
                    panel.selectedKeyIndex.reset();
                }
                RefreshAnimationFileList(&panel, AnimationDirectory(*runtimeState));
            }
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

    int durationFrames = static_cast<int>(animationPath.durationFrames);
    if (ImGui::InputInt("Full Duration Frames", &durationFrames)) {
        animationPath.durationFrames = static_cast<std::uint32_t>(std::max(1, durationFrames));
        panel.dirty = true;
    }
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
                                    ? AnimationFilePathForName(*runtimeState, animationPath.name)
                                    : std::filesystem::path{panel.currentFilePath};
        SaveAnimationPathToFile(runtimeState, animationPath, outputPath);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save As")) {
        SaveAnimationPathToFile(
            runtimeState,
            animationPath,
            AnimationFilePathForName(*runtimeState, animationPath.name));
    }
    if (panel.dirty) {
        ImGui::SameLine();
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
            const auto label = std::to_string(index + 1U) + "  " +
                               (animationPath.keys[index].sourceShotName.empty()
                                    ? std::string{"Key"}
                                    : animationPath.keys[index].sourceShotName);
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
    float cameraPosition[3] = {key.cameraPosition[0], key.cameraPosition[1], key.cameraPosition[2]};
    if (ImGui::InputFloat3("Camera Key", cameraPosition, "%.3f")) {
        invisible_places::camera::MoveAnimationCameraKey(
            &animationPath,
            panel.selectedKeyIndex.value(),
            {cameraPosition[0], cameraPosition[1], cameraPosition[2]});
        panel.dirty = true;
    }
    float focusPoint[3] = {key.focusPoint[0], key.focusPoint[1], key.focusPoint[2]};
    if (ImGui::InputFloat3("Focus Key", focusPoint, "%.3f")) {
        invisible_places::camera::MoveAnimationFocusKey(
            &animationPath,
            panel.selectedKeyIndex.value(),
            {focusPoint[0], focusPoint[1], focusPoint[2]});
        panel.dirty = true;
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
            runtimeState->statusMessage =
                "Loaded point style preset from " + runtimeState->persistence.pointStylePresetPath + ".";
            runtimeState->errorMessage.clear();
        }
    }
    EndPanelSection();
}

PreviewLayerSession* DrawVisiblePointCloudLookdevSelector(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr) {
        return nullptr;
    }

    auto visualIndex = ResolveVisiblePointCloudLookdevIndex(*runtimeState);
    if (!visualIndex.has_value()) {
        return nullptr;
    }

    const auto currentIndex = visualIndex.value();
    if (ImGui::BeginCombo("Visible Cloud", runtimeState->sessions[currentIndex].displayName.c_str())) {
        for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
            auto& session = runtimeState->sessions[index];
            if (!IsVisibleLoadedPointCloudSession(session)) {
                continue;
            }

            const bool selected = visualIndex.has_value() && visualIndex.value() == index;
            if (ImGui::Selectable(session.displayName.c_str(), selected)) {
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

void DrawLidarPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    DrawLayerSection(runtimeState, viewport, LayerKind::PointCloud, "Lidar Layers");
}

void DrawVisualsPanel(PreviewRuntimeState* runtimeState) {
    const auto visualIndex = runtimeState != nullptr ? ResolveVisiblePointCloudLookdevIndex(*runtimeState) : std::nullopt;
    PreviewLayerSession* session =
        visualIndex.has_value() ? &runtimeState->sessions[visualIndex.value()] : nullptr;
    if (BeginPanelSection("Cloud Visuals")) {
        session = DrawVisiblePointCloudLookdevSelector(runtimeState);
        EndPanelSection();
    }
    if (session == nullptr) {
        ImGui::TextUnformatted("Load and show a lidar layer to edit its visual settings.");
        return;
    }

    DrawPointCloudStyleSection(session);
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

void DrawProjectPanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (BeginPanelSection("Project Settings")) {
    auto& settings = runtimeState->projectSettings;
    ImGui::ColorEdit3("Background Color", settings.backgroundColor.data());
    ImGui::Checkbox("Show Status Overlay", &settings.showStatusOverlay);

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
            ImGui::Text("Points: %s", FormatPointCount(diagnostics.pointCount).c_str());
            ImGui::Text("Average point size: %.2f px", diagnostics.averagePointSizePx);
            if (!diagnostics.pointRenderModes.empty()) {
                ImGui::TextWrapped("Point modes: %s", diagnostics.pointRenderModes.c_str());
            }
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
        const float frameProgress =
            job.frames.empty()
                ? 0.0F
                : static_cast<float>(job.writtenFrameCount) / static_cast<float>(job.frames.size());
        const char* sectionLabel = job.mode == invisible_places::output::AnimationExportMode::FastPreviewMp4
                                       ? "Encoding Fast Preview MP4"
                                       : "Rendering HQ Preview-Density EXR";
        if (!BeginPanelSection(sectionLabel)) {
            return;
        }
        ImGui::ProgressBar(frameProgress, ImVec2{-FLT_MIN, 0.0F});
        ImGui::Text(
            "Captured %u / %zu, saved %u, queued %zu",
            std::min<std::uint32_t>(job.currentFrame, static_cast<std::uint32_t>(job.frames.size())),
            job.frames.size(),
            std::min<std::uint32_t>(job.writtenFrameCount, static_cast<std::uint32_t>(job.frames.size())),
            job.pendingFrameCount);
        ImGui::Text("Elapsed: %s", FormatElapsedTime(std::chrono::steady_clock::now() - job.startedAt).c_str());
        if (!job.lastOutputPath.empty()) {
            ImGui::TextWrapped("Last: %s", job.lastOutputPath.string().c_str());
        }
        if (ImGui::Button(job.cancelRequested ? "Cancelling..." : "Cancel Export")) {
            RequestOfflineRenderCancellation(&job);
        }
        EndPanelSection();
    }
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
    constexpr auto flags = ImGuiWindowFlags_NoCollapse;
    ImGui::Begin("Invisible Places Controls", nullptr, flags);

    sidePanel.hovered = ImGui::IsWindowHovered(
        ImGuiHoveredFlags_AllowWhenBlockedByActiveItem | ImGuiHoveredFlags_RootAndChildWindows);
    sidePanel.interacting =
        popupOpen || ImGui::IsAnyItemActive() ||
        ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    if (ImGui::BeginTabBar("ScenePanelTabs")) {
        if (ImGui::BeginTabItem("Info")) {
            DrawRenderInfoSection(runtimeState, *viewport);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Lidar")) {
            DrawLidarPanel(runtimeState, viewport);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Visuals")) {
            DrawVisualsPanel(runtimeState);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("gSplat")) {
            DrawGsplatPanel(runtimeState, viewport);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Camera")) {
            DrawCameraSection(runtimeState, *viewport);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Animation")) {
            DrawAnimationSection(runtimeState, *viewport);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Project")) {
            DrawProjectPanel(runtimeState, viewport);
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
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
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
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
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
    renderState.nearPlane = runtimeState.camera.NearPlane();
    renderState.farPlane = runtimeState.camera.FarPlane();
    const auto cameraState = runtimeState.camera.CaptureState();
    renderState.hasDepthOfField = cameraState.hasDepthOfField;
    renderState.focusDistance = cameraState.focusDistance;
    renderState.apertureFStops = cameraState.apertureFStops;
    renderState.depthOfFieldMaxBlurPixels = cameraState.depthOfFieldMaxBlurPixels;
    renderState.gaussianSplatFootprintBoost = runtimeState.projectSettings.gaussianSplatFootprintBoost;

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
                 .style = session.pointStyle,
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

            DrawControlsWindow(&runtimeState, &viewport.value());
            DrawLoadingOverlay(runtimeState);
            DrawAnimationViewportOverlay(&runtimeState, viewport.value());
            UpdateCameraFromInput(&runtimeState, viewport.value());
            UpdateAnimationPlayback(&runtimeState);
            UpdateCameraShotPlayback(&runtimeState);
            UpdatePerformanceInteractionState(&runtimeState, viewport.value());
            PrunePreviewLodSampleCaches(&runtimeState);
            DrawPivotOverlay(runtimeState, viewport.value());
            viewport->UpdateRenderState(BuildRenderState(runtimeState, viewport.value()));
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
