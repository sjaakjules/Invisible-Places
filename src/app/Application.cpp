#include "app/Application.hpp"

#include "InvisiblePlacesBuildConfig.hpp"
#include "camera/OrbitCamera.hpp"
#include "io/AssetDiscovery.hpp"
#include "io/PointCloudData.hpp"
#include "platform/VulkanRuntimeConfig.hpp"
#include "platform/Window.hpp"
#include "platform/WindowTitle.hpp"
#include "renderer/core/VulkanViewportShell.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "scene/SceneCatalog.hpp"
#include "ui/SidePanelState.hpp"

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string_view>
#include <thread>
#include <vector>

namespace invisible_places::app {

namespace {

using PreviewSession = invisible_places::renderer::pointcloud::PointCloudSessionState;
using PointBudgetState = invisible_places::renderer::pointcloud::PointBudgetState;
using PointCloudStyleState = invisible_places::renderer::pointcloud::PointCloudStyleState;
using PointCloudColorMode = invisible_places::renderer::pointcloud::PointCloudColorMode;
using PointCloudColormapId = invisible_places::renderer::pointcloud::PointCloudColormapId;

constexpr float kPi = 3.14159265358979323846F;

enum class PendingLoadPhase {
    CpuLoading,
    UploadPending
};

struct BackgroundPointCloudLoadState {
    std::mutex mutex;
    std::optional<invisible_places::io::PointCloudLoadResult> result;
};

struct PendingPointCloudLoad {
    std::size_t sessionIndex = 0;
    PendingLoadPhase phase = PendingLoadPhase::CpuLoading;
    std::shared_ptr<BackgroundPointCloudLoadState> backgroundState;
    std::optional<invisible_places::io::PointCloudLoadResult> completedResult;
    bool showUploadOverlayFrame = false;
    std::chrono::steady_clock::time_point startedAt = std::chrono::steady_clock::now();
};

struct PreviewRuntimeState {
    std::vector<PreviewSession> sessions;
    std::optional<std::size_t> selectedSessionIndex;
    std::optional<PendingPointCloudLoad> pendingLoad;
    invisible_places::camera::OrbitCamera camera;
    invisible_places::ui::SidePanelState sidePanel{};
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

std::string DescribeBudget(const PointBudgetState& budget) {
    std::ostringstream output;
    output << FormatPointCount(budget.activePoints) << " / " << FormatPointCount(budget.totalPoints)
           << " points";
    return output.str();
}

const char* ColorModeLabel(PointCloudColorMode mode) {
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

void ApplyAutoScalarRange(
    PointCloudStyleState* style,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    if (style == nullptr || scalarFields.empty()) {
        return;
    }

    const auto fieldIndex = std::min<std::size_t>(style->selectedScalarField, scalarFields.size() - 1U);
    style->selectedScalarField = static_cast<std::uint32_t>(fieldIndex);

    if (style->scalarRange.useAutoRange && scalarFields[fieldIndex].valid) {
        style->scalarRange.minimum = scalarFields[fieldIndex].minimum;
        style->scalarRange.maximum = scalarFields[fieldIndex].maximum;
    }
}

void SanitizeStyle(PreviewSession* session) {
    if (session == nullptr) {
        return;
    }

    if (!session->hasSourceRgb && session->style.colorMode == PointCloudColorMode::SourceRgb) {
        session->style.colorMode = session->scalarFields.empty()
                                       ? PointCloudColorMode::SolidColor
                                       : PointCloudColorMode::ScalarColormap;
    }

    if (session->scalarFields.empty() && session->style.colorMode == PointCloudColorMode::ScalarColormap) {
        session->style.colorMode =
            session->hasSourceRgb ? PointCloudColorMode::SourceRgb : PointCloudColorMode::SolidColor;
    }

    ApplyAutoScalarRange(&session->style, session->scalarFields);
}

std::vector<PreviewSession> BuildSessions(const invisible_places::io::AssetCatalog& assetCatalog) {
    std::vector<PreviewSession> sessions;
    sessions.reserve(assetCatalog.pointClouds.size());

    for (const auto& asset : assetCatalog.pointClouds) {
        PreviewSession session;
        session.sourcePath = asset.filePath;
        session.displayName = asset.filePath.stem().string();
        session.loaded = false;
        session.active = false;
        session.hasSourceRgb = asset.header.HasColorRgb();
        session.totalPoints = asset.header.vertexCount;
        session.budget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            asset.header.vertexCount,
            asset.header.vertexCount);
        session.style.colorMode =
            session.hasSourceRgb ? PointCloudColorMode::SourceRgb : PointCloudColorMode::SolidColor;
        sessions.push_back(std::move(session));
    }

    return sessions;
}

std::size_t ChooseStartupCloudIndex(const std::vector<PreviewSession>& sessions) {
    constexpr std::string_view preferredFile = "Site2 -5mm.ply";
    for (std::size_t index = 0; index < sessions.size(); ++index) {
        if (sessions[index].sourcePath.filename() == preferredFile) {
            return index;
        }
    }

    return 0;
}

bool IsBusyLoading(const PreviewRuntimeState& runtimeState) {
    return runtimeState.pendingLoad.has_value();
}

std::size_t LoadedCloudCount(const PreviewRuntimeState& runtimeState) {
    return static_cast<std::size_t>(std::count_if(
        runtimeState.sessions.begin(),
        runtimeState.sessions.end(),
        [](const PreviewSession& session) { return session.loaded && session.active; }));
}

float CurrentAspectRatio(const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    const auto width = std::max<std::uint32_t>(1U, viewport.Width());
    const auto height = std::max<std::uint32_t>(1U, viewport.Height());
    return static_cast<float>(width) / static_cast<float>(height);
}

PreviewSession* SelectedSession(PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || !runtimeState->selectedSessionIndex.has_value()) {
        return nullptr;
    }

    return &runtimeState->sessions[runtimeState->selectedSessionIndex.value()];
}

const PreviewSession* SelectedSession(const PreviewRuntimeState& runtimeState) {
    if (!runtimeState.selectedSessionIndex.has_value()) {
        return nullptr;
    }

    return &runtimeState.sessions[runtimeState.selectedSessionIndex.value()];
}

PreviewSession* SelectedLoadedSession(PreviewRuntimeState* runtimeState) {
    auto* session = SelectedSession(runtimeState);
    return session != nullptr && session->loaded && session->active ? session : nullptr;
}

const PreviewSession* SelectedLoadedSession(const PreviewRuntimeState& runtimeState) {
    const auto* session = SelectedSession(runtimeState);
    return session != nullptr && session->loaded && session->active ? session : nullptr;
}

std::optional<invisible_places::io::PointCloudLoadResult> TakeCompletedBackgroundResult(
    const std::shared_ptr<BackgroundPointCloudLoadState>& backgroundState) {
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

void FocusSessionCloud(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport,
    std::size_t sessionIndex) {
    if (runtimeState == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return;
    }

    const auto& session = runtimeState->sessions[sessionIndex];
    if (!session.loaded || !session.active || !session.bounds.valid) {
        return;
    }

    runtimeState->camera.FrameBounds(
        session.bounds,
        session.hasFocusPoint ? session.focusPoint : session.bounds.minimum,
        CurrentAspectRatio(viewport));
}

bool ActivateLoadedPointCloud(
    std::size_t sessionIndex,
    invisible_places::io::LoadedPointCloud cloud,
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return false;
    }

    const bool hadLoadedCloudsBefore = LoadedCloudCount(*runtimeState) > 0;
    auto& session = runtimeState->sessions[sessionIndex];
    session.hasSourceRgb = cloud.hasSourceRgb;
    session.totalPoints = cloud.PointCount();
    session.scalarFields = cloud.scalarFields;
    session.bounds = cloud.bounds;
    session.focusPoint = cloud.focusPoint;
    session.hasFocusPoint = cloud.hasFocusPoint;
    session.loaded = true;
    session.active = true;

    if (session.budget.totalPoints != session.totalPoints) {
        session.budget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            session.totalPoints,
            session.totalPoints);
    } else {
        session.budget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            session.totalPoints,
            session.budget.activePoints == 0 ? session.totalPoints : session.budget.activePoints);
    }

    SanitizeStyle(&session);

    try {
        viewport->UploadPointCloud(sessionIndex, cloud, session.budget.sampledIndices);
    } catch (const std::exception& error) {
        session.loaded = false;
        session.active = false;
        runtimeState->errorMessage = "GPU upload failed: " + std::string{error.what()};
        std::cerr << runtimeState->errorMessage << std::endl;
        return false;
    }

    runtimeState->selectedSessionIndex = sessionIndex;
    if (!hadLoadedCloudsBefore) {
        FocusSessionCloud(runtimeState, *viewport, sessionIndex);
    }

    runtimeState->statusMessage = "Loaded " + session.displayName + " with " +
                                  FormatPointCount(session.totalPoints) + " points.";
    runtimeState->errorMessage.clear();
    std::cout << runtimeState->statusMessage << std::endl;
    return true;
}

void BeginPointCloudLoad(std::size_t sessionIndex, PreviewRuntimeState* runtimeState) {
    if (runtimeState == nullptr || sessionIndex >= runtimeState->sessions.size()) {
        return;
    }

    runtimeState->selectedSessionIndex = sessionIndex;
    auto& session = runtimeState->sessions[sessionIndex];

    if (runtimeState->pendingLoad.has_value()) {
        runtimeState->statusMessage = "Please wait for the current cloud to finish loading.";
        return;
    }

    if (session.loaded && session.active) {
        runtimeState->statusMessage = session.displayName + " is already loaded.";
        runtimeState->errorMessage.clear();
        return;
    }

    const auto filePath = session.sourcePath;
    runtimeState->statusMessage = "Loading " + session.displayName + " in the background...";
    runtimeState->errorMessage.clear();
    std::cout << "Loading point cloud: " << filePath.filename().string() << std::endl;

    auto backgroundState = std::make_shared<BackgroundPointCloudLoadState>();
    std::thread(
        [backgroundState, filePath]() {
            auto result = invisible_places::io::LoadPointCloud(filePath);
            std::scoped_lock lock(backgroundState->mutex);
            backgroundState->result = std::move(result);
        })
        .detach();

    runtimeState->pendingLoad = PendingPointCloudLoad{
        .sessionIndex = sessionIndex,
        .phase = PendingLoadPhase::CpuLoading,
        .backgroundState = std::move(backgroundState),
        .completedResult = std::nullopt,
        .showUploadOverlayFrame = false,
        .startedAt = std::chrono::steady_clock::now(),
    };
}

void PollPendingPointCloudLoad(
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

        if (!completedResult->success) {
            runtimeState->statusMessage.clear();
            runtimeState->errorMessage = "Load failed: " + completedResult->errorMessage;
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

    if (completedLoad.success) {
        ActivateLoadedPointCloud(
            pendingLoad.sessionIndex,
            std::move(completedLoad.cloud),
            runtimeState,
            viewport);
    } else {
        runtimeState->statusMessage.clear();
        runtimeState->errorMessage = "Load failed: " + completedLoad.errorMessage;
        std::cerr << runtimeState->errorMessage << std::endl;
    }

    runtimeState->pendingLoad.reset();
}

void UnloadSelectedPointCloud(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    if (runtimeState == nullptr || viewport == nullptr || !runtimeState->selectedSessionIndex.has_value()) {
        return;
    }

    auto& session = runtimeState->sessions[runtimeState->selectedSessionIndex.value()];
    if (!session.loaded) {
        return;
    }

    viewport->RemovePointCloud(runtimeState->selectedSessionIndex.value());
    session.loaded = false;
    session.active = false;
    runtimeState->statusMessage = "Unloaded " + session.displayName + ".";
    runtimeState->errorMessage.clear();
}

void DrawStatusOverlay(const PreviewRuntimeState& runtimeState) {
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
    ImGui::Text("Loaded clouds: %s", FormatPointCount(LoadedCloudCount(runtimeState)).c_str());

    if (const auto* session = SelectedLoadedSession(runtimeState); session != nullptr) {
        ImGui::Text("Selected: %s", session->displayName.c_str());
        ImGui::Text("Budget: %s", DescribeBudget(session->budget).c_str());
        ImGui::Text("Mode: %s", ColorModeLabel(session->style.colorMode));
        ImGui::Text("LMB orbit  RMB/MMB pan  Wheel dolly  F focus");
    }

    ImGui::End();
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
    const bool firstCloudLoad = LoadedCloudCount(runtimeState) == 0;

    if (firstCloudLoad) {
        ImGui::SetNextWindowPos(ImVec2{0.0F, 0.0F}, ImGuiCond_Always);
        ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
        constexpr auto fullscreenFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;
        ImGui::Begin("InitialCloudLoadingOverlay", nullptr, fullscreenFlags);
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImGui::GetWindowPos(),
            ImVec2{ImGui::GetWindowPos().x + io.DisplaySize.x, ImGui::GetWindowPos().y + io.DisplaySize.y},
            IM_COL32(236, 231, 218, 206));

        const ImVec2 overlaySize = ImVec2{380.0F, 208.0F};
        const ImVec2 overlayPosition =
            ImVec2{(io.DisplaySize.x - overlaySize.x) * 0.5F, (io.DisplaySize.y - overlaySize.y) * 0.5F};
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
        ImGui::TextWrapped(
            "%s",
            pendingLoad.phase == PendingLoadPhase::CpuLoading
                ? "The window is ready. Loading the first point cloud from disk now."
                : "The cloud is loaded. Uploading point and scalar buffers to the GPU now.");
        ImGui::Spacing();
        ImGui::Text("Elapsed: %.1f s", elapsedSeconds);
        ImGui::TextDisabled("The preview will appear automatically.");
        ImGui::PopTextWrapPos();
        ImGui::EndChild();
        ImGui::End();
        return;
    }

    ImGui::SetNextWindowPos(ImVec2{24.0F, 100.0F}, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.90F);
    constexpr auto cardFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                               ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;
    ImGui::Begin("BackgroundCloudLoadingCard", nullptr, cardFlags);
    DrawSpinnerArc(14.0F, 4.0F, IM_COL32(110, 86, 34, 255));
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Text("%s", targetSession.displayName.c_str());
    ImGui::TextUnformatted(
        pendingLoad.phase == PendingLoadPhase::CpuLoading ? "Loading in the background..." : "Uploading to GPU...");
    ImGui::TextDisabled("Existing loaded clouds stay visible. %.1f s", elapsedSeconds);
    ImGui::EndGroup();
    ImGui::End();
}

void DrawFileSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    ImGui::SeparatorText("Files");

    if (runtimeState->sessions.empty()) {
        ImGui::TextUnformatted("No point clouds were discovered.");
        return;
    }

    for (std::size_t index = 0; index < runtimeState->sessions.size(); ++index) {
        auto& session = runtimeState->sessions[index];
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
            if (session.loaded && session.active) {
                FocusSessionCloud(runtimeState, *viewport, index);
            } else {
                BeginPointCloudLoad(index, runtimeState);
            }
        }

        ImGui::SameLine();
        if (isPendingLoad) {
            ImGui::TextColored(ImVec4{0.55F, 0.42F, 0.16F, 1.0F}, "loading...");
        } else if (session.loaded && session.active) {
            ImGui::TextColored(ImVec4{0.16F, 0.46F, 0.22F, 1.0F}, "loaded");
        } else {
            ImGui::TextDisabled("%s", FormatPointCount(session.totalPoints).c_str());
        }
        ImGui::PopID();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Single click selects. Double click loads or focuses.");

    if (auto* session = SelectedSession(runtimeState); session != nullptr) {
        ImGui::Spacing();
        ImGui::Text("Selected: %s", session->displayName.c_str());
        ImGui::Text("Total points: %s", FormatPointCount(session->totalPoints).c_str());

        if (session->loaded && session->active) {
            std::uint64_t requestedBudget = session->budget.activePoints;
            if (ImGui::InputScalar("Budget Points", ImGuiDataType_U64, &requestedBudget)) {
                session->budget = invisible_places::renderer::pointcloud::MakePointBudgetState(
                    session->totalPoints,
                    requestedBudget);
                viewport->UpdatePointBudget(runtimeState->selectedSessionIndex.value(), session->budget.sampledIndices);
            }

            float requestedFraction = session->budget.activeFraction;
            if (ImGui::SliderFloat("Budget Fraction", &requestedFraction, 0.0F, 1.0F, "%.3f")) {
                const auto requestedPoints = static_cast<std::uint64_t>(
                    requestedFraction >= 1.0F ? session->totalPoints
                                              : static_cast<double>(session->totalPoints) * requestedFraction);
                session->budget = invisible_places::renderer::pointcloud::MakePointBudgetState(
                    session->totalPoints,
                    requestedPoints);
                viewport->UpdatePointBudget(runtimeState->selectedSessionIndex.value(), session->budget.sampledIndices);
            }

            ImGui::Text("Drawn: %s", DescribeBudget(session->budget).c_str());
            if (ImGui::Button("Unload Selected Cloud")) {
                UnloadSelectedPointCloud(runtimeState, viewport);
            }
        } else {
            if (ImGui::Button("Load Selected Cloud")) {
                BeginPointCloudLoad(runtimeState->selectedSessionIndex.value(), runtimeState);
            }
            if (IsBusyLoading(*runtimeState)) {
                ImGui::TextDisabled("Another cloud is loading right now.");
            }
        }
    }
}

void DrawStyleSection(PreviewRuntimeState* runtimeState) {
    ImGui::SeparatorText("Style");

    auto* session = SelectedLoadedSession(runtimeState);
    if (session == nullptr) {
        ImGui::TextUnformatted("Select a loaded point cloud to edit lookdev.");
        return;
    }

    auto& style = session->style;

    int colorModeIndex = static_cast<int>(style.colorMode);
    const char* colorModes[] = {"Source RGB", "Solid Color", "Scalar Colormap"};
    if (ImGui::Combo("Color Source", &colorModeIndex, colorModes, IM_ARRAYSIZE(colorModes))) {
        style.colorMode = static_cast<PointCloudColorMode>(colorModeIndex);
        SanitizeStyle(session);
    }
    if (!session->hasSourceRgb && style.colorMode == PointCloudColorMode::SourceRgb) {
        ImGui::TextDisabled("Source RGB is not available on this file.");
    }

    if (style.colorMode == PointCloudColorMode::SolidColor) {
        ImGui::ColorEdit4("Solid Color", style.solidColor.data());
    }

    if (!session->scalarFields.empty() && style.colorMode == PointCloudColorMode::ScalarColormap) {
        if (ImGui::BeginCombo(
                "Scalar Field",
                session
                    ->scalarFields[std::min<std::size_t>(style.selectedScalarField, session->scalarFields.size() - 1)]
                    .name.c_str())) {
            for (std::size_t fieldIndex = 0; fieldIndex < session->scalarFields.size(); ++fieldIndex) {
                const bool selected = style.selectedScalarField == fieldIndex;
                if (ImGui::Selectable(session->scalarFields[fieldIndex].name.c_str(), selected)) {
                    style.selectedScalarField = static_cast<std::uint32_t>(fieldIndex);
                    ApplyAutoScalarRange(&style, session->scalarFields);
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        int colormapIndex = static_cast<int>(style.colormap);
        const char* colormaps[] = {"Viridis", "Plasma", "Inferno", "Magma", "Cividis", "Turbo"};
        if (ImGui::Combo("Colormap", &colormapIndex, colormaps, IM_ARRAYSIZE(colormaps))) {
            style.colormap = static_cast<PointCloudColormapId>(colormapIndex);
        }

        if (ImGui::Checkbox("Auto Scalar Range", &style.scalarRange.useAutoRange)) {
            ApplyAutoScalarRange(&style, session->scalarFields);
        }
        ImGui::Checkbox("Clamp Range", &style.scalarRange.clamp);

        if (!style.scalarRange.useAutoRange) {
            ImGui::InputFloat("Range Min", &style.scalarRange.minimum, 0.0F, 0.0F, "%.5g");
            ImGui::InputFloat("Range Max", &style.scalarRange.maximum, 0.0F, 0.0F, "%.5g");
        } else {
            ImGui::Text("Range: %.5g to %.5g", style.scalarRange.minimum, style.scalarRange.maximum);
        }
    } else if (style.colorMode == PointCloudColorMode::ScalarColormap) {
        ImGui::TextDisabled("No scalar fields were discovered for this cloud.");
    }

    ImGui::SliderFloat("Point Size", &style.pointSize, 1.0F, 16.0F, "%.2f");
    ImGui::SliderFloat("Opacity", &style.opacity, 0.05F, 1.0F, "%.2f");
    ImGui::SliderFloat("Emissive", &style.emissiveStrength, 0.0F, 2.5F, "%.2f");
    ImGui::SliderFloat("X-Ray", &style.xrayStrength, 0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Depth Fade", &style.depthFade, 0.0F, 1.0F, "%.2f");
}

void DrawCameraSection(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    ImGui::SeparatorText("Camera");

    auto* session = SelectedLoadedSession(runtimeState);
    if (session == nullptr || !runtimeState->camera.HasFramedBounds()) {
        ImGui::TextUnformatted("Select a loaded cloud to frame the camera.");
        return;
    }

    if (ImGui::Button("Focus Selected Cloud")) {
        FocusSessionCloud(runtimeState, viewport, runtimeState->selectedSessionIndex.value());
    }

    const auto target = runtimeState->camera.Target();
    ImGui::Text("Target: %.3f  %.3f  %.3f", target.x, target.y, target.z);
    ImGui::Text("Distance: %.3f", runtimeState->camera.Distance());
    ImGui::Text("FOV: %.1f", runtimeState->camera.FovDegrees());
    ImGui::Text(
        "Near/Far: %.4f / %.1f",
        runtimeState->camera.NearPlane(),
        runtimeState->camera.FarPlane());
    ImGui::Text("Cloud bounds valid: %s", session->bounds.valid ? "yes" : "no");
}

void DrawSidePanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    auto& sidePanel = runtimeState->sidePanel;
    const auto& io = ImGui::GetIO();

    const bool nearRightEdge = io.MousePos.x >= (io.DisplaySize.x - sidePanel.handleWidth - 6.0F);
    const bool shouldReveal = sidePanel.pinned || nearRightEdge || sidePanel.hovered;
    const float targetReveal = shouldReveal ? 1.0F : 0.0F;
    sidePanel.revealAmount += (targetReveal - sidePanel.revealAmount) * 0.18F;
    sidePanel.revealAmount = std::clamp(sidePanel.revealAmount, 0.0F, 1.0F);
    const float visibleWidth =
        sidePanel.handleWidth + ((sidePanel.panelWidth - sidePanel.handleWidth) * sidePanel.revealAmount);

    sidePanel.mode = sidePanel.pinned
                         ? invisible_places::ui::SidePanelMode::Pinned
                         : sidePanel.revealAmount > 0.02F ? invisible_places::ui::SidePanelMode::RevealOnHover
                                                          : invisible_places::ui::SidePanelMode::Collapsed;

    ImGui::SetNextWindowPos(ImVec2{io.DisplaySize.x - visibleWidth, 0.0F}, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2{visibleWidth, io.DisplaySize.y}, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.90F);

    constexpr auto flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("PointCloudLookdevPanel", nullptr, flags);

    sidePanel.hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.48F, 0.40F, 0.22F, 0.92F});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.54F, 0.45F, 0.24F, 0.96F});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.42F, 0.34F, 0.18F, 0.98F});

    if (ImGui::Button(sidePanel.pinned ? "Unpin" : "Pin", ImVec2{sidePanel.handleWidth - 2.0F, 34.0F})) {
        sidePanel.pinned = !sidePanel.pinned;
    }
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        sidePanel.pinned = !sidePanel.pinned;
    }

    ImGui::PopStyleColor(3);
    ImGui::SameLine();
    ImGui::BeginGroup();
    DrawFileSection(runtimeState, viewport);
    DrawStyleSection(runtimeState);
    DrawCameraSection(runtimeState, *viewport);
    ImGui::EndGroup();
    ImGui::End();
}

void UpdateCameraFromInput(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    if (LoadedCloudCount(*runtimeState) == 0) {
        return;
    }

    const auto& io = ImGui::GetIO();
    if (!viewport.UiWantsMouseCapture()) {
        if (io.MouseWheel != 0.0F) {
            runtimeState->camera.Dolly(io.MouseWheel);
        }

        const bool isPanning = io.MouseDown[1] || io.MouseDown[2] || (io.KeyShift && io.MouseDown[0]);
        if (isPanning) {
            runtimeState->camera.Pan(io.MouseDelta.x, io.MouseDelta.y, io.DisplaySize.x, io.DisplaySize.y);
        } else if (io.MouseDown[0]) {
            runtimeState->camera.Orbit(io.MouseDelta.x, io.MouseDelta.y);
        }
    }

    if (!viewport.UiWantsKeyboardCapture() && ImGui::IsKeyPressed(ImGuiKey_F)) {
        if (runtimeState->selectedSessionIndex.has_value()) {
            FocusSessionCloud(runtimeState, viewport, runtimeState->selectedSessionIndex.value());
        }
    }
}

invisible_places::renderer::core::PointCloudRenderState BuildRenderState(
    const PreviewRuntimeState& runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    invisible_places::renderer::core::PointCloudRenderState renderState;
    const auto aspectRatio = CurrentAspectRatio(viewport);
    const auto matrices = runtimeState.camera.Matrices(aspectRatio);

    renderState.view = matrices.view;
    renderState.projection = matrices.projection;
    renderState.viewProjection = matrices.viewProjection;
    renderState.cameraPosition = matrices.position;
    renderState.nearPlane = runtimeState.camera.NearPlane();
    renderState.farPlane = runtimeState.camera.FarPlane();

    for (std::size_t sessionIndex = 0; sessionIndex < runtimeState.sessions.size(); ++sessionIndex) {
        const auto& session = runtimeState.sessions[sessionIndex];
        if (!session.loaded || !session.active) {
            continue;
        }

        renderState.layers.push_back(
            {.layerId = sessionIndex, .style = session.style, .hasSourceRgb = session.hasSourceRgb});
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

    std::cout << "Invisible Places bootstrap" << std::endl;
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
              << "Opening bootstrap window. Press Escape or close the window to exit." << std::endl;

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
    runtimeState.sidePanel.panelWidth = 390.0F;

    if (viewport.has_value() && !runtimeState.sessions.empty()) {
        BeginPointCloudLoad(ChooseStartupCloudIndex(runtimeState.sessions), &runtimeState);
    } else if (runtimeState.sessions.empty()) {
        runtimeState.errorMessage = "No point clouds were discovered in the Data directory.";
    }

    while (!window.ShouldClose()) {
        window.PollEvents();

        if (viewport.has_value()) {
            PollPendingPointCloudLoad(&runtimeState, &viewport.value());
            viewport->BeginUiFrame();
            DrawStatusOverlay(runtimeState);
            DrawSidePanel(&runtimeState, &viewport.value());
            DrawLoadingOverlay(runtimeState);
            UpdateCameraFromInput(&runtimeState, viewport.value());
            viewport->UpdateRenderState(BuildRenderState(runtimeState, viewport.value()));
            viewport->DrawFrame();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds{16});
        }
    }

    if (viewport.has_value()) {
        viewport->WaitIdle();
    }

    return 0;
}

}  // namespace invisible_places::app
