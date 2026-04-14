#include "app/Application.hpp"

#include "InvisiblePlacesBuildConfig.hpp"
#include "camera/OrbitCamera.hpp"
#include "io/AssetDiscovery.hpp"
#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
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
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string_view>
#include <thread>
#include <variant>
#include <vector>

#include <glm/mat4x4.hpp>

namespace invisible_places::app {

namespace {

using LayerKind = invisible_places::scene::LayerKind;
using PointBudgetState = invisible_places::renderer::pointcloud::PointBudgetState;
using PointCloudStyleState = invisible_places::renderer::pointcloud::PointCloudStyleState;
using PointCloudColorMode = invisible_places::renderer::pointcloud::PointCloudColorMode;
using PointCloudColormapId = invisible_places::renderer::pointcloud::PointCloudColormapId;
using GaussianSplatStyleState = invisible_places::renderer::gsplat::GaussianSplatStyleState;
using GaussianSplatColorMode = invisible_places::renderer::gsplat::GaussianSplatColorMode;
using GaussianSplatDebugMode = invisible_places::renderer::gsplat::GaussianSplatDebugMode;
using GaussianSplatQualityMode = invisible_places::renderer::gsplat::GaussianSplatQualityMode;
using RenderParameterBinding = invisible_places::style::RenderParameterBinding;
using ParameterSourceMode = invisible_places::style::ParameterSourceMode;
using FieldMapFlags = invisible_places::style::FieldMapFlags;
using ProjectDocument = invisible_places::serialization::ProjectDocument;
using ProjectLayerDocument = invisible_places::serialization::ProjectLayerDocument;
using PointCloudStylePresetDocument = invisible_places::serialization::PointCloudStylePresetDocument;
using LayerLoadResult = std::variant<
    invisible_places::io::PointCloudLoadResult,
    invisible_places::io::GaussianSplatLoadResult>;

constexpr float kPi = 3.14159265358979323846F;

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
    float gaussianSplatFootprintBoost = 1.5F;
    GsplatTransformConvention gsplatTransformConvention = GsplatTransformConvention::AsEncoded;
};

struct PersistenceState {
    std::string projectFilePath;
    std::string pointStylePresetPath;
    std::vector<std::size_t> queuedLoadIndices;
};

struct PreviewLayerSession {
    LayerKind kind = LayerKind::PointCloud;
    std::filesystem::path sourcePath;
    std::filesystem::path transformPath;
    std::string displayName;
    bool loaded = false;
    bool visible = false;
    bool hasSourceRgb = false;
    bool hasFocusPoint = false;
    std::uint64_t totalPrimitives = 0;
    invisible_places::io::Bounds3f localBounds{};
    invisible_places::io::Bounds3f bounds{};
    invisible_places::io::Float3 localFocusPoint{};
    invisible_places::io::Float3 focusPoint{};
    invisible_places::io::Matrix4d localToWorld{};
    bool hasLocalFocusPoint = false;
    std::vector<invisible_places::io::ScalarFieldStats> scalarFields;
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
    std::optional<LayerLoadResult> completedResult;
    bool showUploadOverlayFrame = false;
    std::chrono::steady_clock::time_point startedAt = std::chrono::steady_clock::now();
};

struct CameraInteractionState {
    bool navigationActive = false;
    std::chrono::steady_clock::time_point lastNavigationInputAt{};
};

struct PreviewRuntimeState {
    std::vector<PreviewLayerSession> sessions;
    std::optional<std::size_t> selectedSessionIndex;
    std::optional<PendingLayerLoad> pendingLoad;
    invisible_places::camera::OrbitCamera camera;
    CameraInteractionState cameraInteraction{};
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

std::string DescribeBudget(const PointBudgetState& budget) {
    std::ostringstream output;
    output << FormatPointCount(budget.activePoints) << " / " << FormatPointCount(budget.totalPoints)
           << " points";
    return output.str();
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
        runtimeState.cameraInteraction.navigationActive);
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
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.opacity, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.emissiveStrength, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.xrayStrength, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.depthFade, session->scalarFields);
    invisible_places::style::SyncBindingFieldReference(&session->pointStyle.colormapPosition, session->scalarFields);

    if (session->pointStyle.colorMode == PointCloudColorMode::ScalarColormap) {
        EnsureFieldMappedBindingDefaults(
            &session->pointStyle.colormapPosition,
            session->scalarFields,
            0.0F,
            1.0F);
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

struct ScalarBindingWidgetConfig {
    float constantMin = 0.0F;
    float constantMax = 1.0F;
    float defaultOutputMin = 0.0F;
    float defaultOutputMax = 1.0F;
    float defaultConstant = 0.0F;
    const char* format = "%.3f";
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

bool DrawScalarBindingWidget(
    const char* label,
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    const ScalarBindingWidgetConfig& config) {
    if (binding == nullptr) {
        return false;
    }

    bool changed = false;
    ImGui::PushID(label);
    ImGui::SeparatorText(label);

    int modeIndex = static_cast<int>(binding->mode);
    const char* modeLabels[] = {"Constant", "Field-Mapped"};
    if (ImGui::Combo("Mode", &modeIndex, modeLabels, IM_ARRAYSIZE(modeLabels))) {
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
        float constantValue = invisible_places::style::ScalarConstant(*binding);
        if (ImGui::SliderFloat("Value", &constantValue, config.constantMin, config.constantMax, config.format)) {
            invisible_places::style::SetScalarConstant(binding, constantValue);
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

    changed |= ImGui::InputFloat("Output Min", &binding->fieldMap.outputMin, 0.0F, 0.0F, config.format);
    changed |= ImGui::InputFloat("Output Max", &binding->fieldMap.outputMax, 0.0F, 0.0F, config.format);
    changed |= ImGui::SliderFloat("Gamma", &binding->fieldMap.gamma, 0.05F, 4.0F, "%.2f");

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
    session.totalPrimitives = cloud.PointCount();
    session.scalarFields = cloud.scalarFields;
    session.bounds = cloud.bounds;
    session.focusPoint = cloud.focusPoint;
    session.hasFocusPoint = cloud.hasFocusPoint;
    session.loaded = true;
    session.visible = true;

    if (session.pointBudget.totalPoints != session.totalPrimitives) {
        session.pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            session.totalPrimitives,
            session.totalPrimitives);
    } else {
        session.pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
            session.totalPrimitives,
            session.pointBudget.activePoints == 0 ? session.totalPrimitives : session.pointBudget.activePoints);
    }

    SanitizePointCloudStyle(&session);

    try {
        viewport->UploadPointCloud(sessionIndex, cloud, session.pointBudget.sampledIndices);
    } catch (const std::exception& error) {
        session.loaded = false;
        session.visible = false;
        runtimeState->errorMessage = "GPU upload failed: " + std::string{error.what()};
        std::cerr << runtimeState->errorMessage << std::endl;
        return false;
    }

    runtimeState->selectedSessionIndex = sessionIndex;
    if (!hadVisibleLayersBefore) {
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
    if (!hadVisibleLayersBefore) {
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
    std::thread(
        [backgroundState, layerKind, filePath, transformPath]() {
            LayerLoadResult result = layerKind == LayerKind::PointCloud
                                         ? LayerLoadResult{invisible_places::io::LoadPointCloud(filePath)}
                                         : LayerLoadResult{
                                               invisible_places::io::LoadGaussianSplat(filePath, transformPath)};
            std::scoped_lock lock(backgroundState->mutex);
            backgroundState->result = std::move(result);
        })
        .detach();

    runtimeState->pendingLoad = PendingLayerLoad{
        .sessionIndex = sessionIndex,
        .phase = PendingLoadPhase::CpuLoading,
        .backgroundState = std::move(backgroundState),
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
}

ProjectDocument BuildProjectDocument(const PreviewRuntimeState& runtimeState) {
    ProjectDocument document;
    document.projectName = "Invisible Places";
    document.backgroundColor = runtimeState.projectSettings.backgroundColor;
    document.sidePanelPinned = runtimeState.sidePanel.pinned;
    document.autoLowerGsplatQualityWhileNavigating =
        runtimeState.projectSettings.autoLowerGsplatQualityWhileNavigating;

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
    runtimeState->persistence.queuedLoadIndices.clear();

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
            session.pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
                session.totalPrimitives == 0 ? layerIt->pointBudgetActivePoints : session.totalPrimitives,
                layerIt->pointBudgetActivePoints);
            if (session.loaded) {
                viewport->UpdatePointBudget(sessionIndex, session.pointBudget.sampledIndices);
            }
        }

        SanitizePointCloudStyle(&session);

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
        if (runtimeState->sessions[selectedIndex].loaded) {
            FocusSessionLayer(runtimeState, *viewport, selectedIndex);
        }
    }

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
            ImGui::Text("Mode: %s", PointCloudColorModeLabel(session->pointStyle.colorMode));
        } else {
            ImGui::Text("Mode: %s", GaussianSplatColorModeLabel(session->gsplatStyle.colorMode));
            ImGui::Text("Debug: %s", GaussianSplatDebugModeLabel(session->gsplatStyle.debugMode));
            const auto effectiveQuality = EffectiveGaussianSplatQualityMode(runtimeState, *session);
            ImGui::Text("Quality: %s", GaussianSplatQualityModeLabel(session->gsplatStyle.qualityMode));
            if (effectiveQuality != session->gsplatStyle.qualityMode) {
                ImGui::Text(
                    "Rendering: %s while navigating",
                    GaussianSplatQualityModeLabel(effectiveQuality));
            }
            ImGui::Text(
                "Transform: %s",
                GsplatTransformConventionLabel(runtimeState.projectSettings.gsplatTransformConvention));
        }
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
    const bool firstVisibleLayerLoad = VisibleLayerCount(runtimeState) == 0;

    if (firstVisibleLayerLoad) {
        ImGui::SetNextWindowPos(ImVec2{0.0F, 0.0F}, ImGuiCond_Always);
        ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
        constexpr auto fullscreenFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;
        ImGui::Begin("InitialLayerLoadingOverlay", nullptr, fullscreenFlags);
        const auto& backgroundColor = runtimeState.projectSettings.backgroundColor;
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImGui::GetWindowPos(),
            ImVec2{ImGui::GetWindowPos().x + io.DisplaySize.x, ImGui::GetWindowPos().y + io.DisplaySize.y},
            ImGui::ColorConvertFloat4ToU32(ImVec4{
                backgroundColor[0],
                backgroundColor[1],
                backgroundColor[2],
                0.82F,
            }));

        const ImVec2 overlaySize = ImVec2{390.0F, 214.0F};
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

    ImGui::SetNextWindowPos(ImVec2{24.0F, 100.0F}, ImGuiCond_Always);
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
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    ImGui::SeparatorText("Layers");

    if (runtimeState->sessions.empty()) {
        ImGui::TextUnformatted("No point clouds or gSplats were discovered.");
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
                    session->pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
                        session->totalPrimitives,
                        requestedBudget);
                    viewport->UpdatePointBudget(
                        runtimeState->selectedSessionIndex.value(),
                        session->pointBudget.sampledIndices);
                }

                float requestedFraction = session->pointBudget.activeFraction;
                if (ImGui::SliderFloat("Budget Fraction", &requestedFraction, 0.0F, 1.0F, "%.3f")) {
                    const auto requestedPoints = static_cast<std::uint64_t>(
                        requestedFraction >= 1.0F ? session->totalPrimitives
                                                  : static_cast<double>(session->totalPrimitives) * requestedFraction);
                    session->pointBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
                        session->totalPrimitives,
                        requestedPoints);
                    viewport->UpdatePointBudget(
                        runtimeState->selectedSessionIndex.value(),
                        session->pointBudget.sampledIndices);
                }

                ImGui::Text("Drawn: %s", DescribeBudget(session->pointBudget).c_str());
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
}

void DrawPointCloudStyleSection(PreviewLayerSession* session) {
    auto& style = session->pointStyle;
    bool changed = false;

    int colorModeIndex = static_cast<int>(style.colorMode);
    const char* colorModes[] = {"Source RGB", "Solid Color", "Scalar Colormap"};
    if (ImGui::Combo("Color Source", &colorModeIndex, colorModes, IM_ARRAYSIZE(colorModes))) {
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
        if (ImGui::Combo("Colormap", &colormapIndex, colormaps, IM_ARRAYSIZE(colormaps))) {
            style.colormap = static_cast<PointCloudColormapId>(colormapIndex);
            changed = true;
        }
        changed |= DrawScalarBindingWidget(
            "Colormap Position",
            &style.colormapPosition,
            session->scalarFields,
            {.constantMin = 0.0F,
             .constantMax = 1.0F,
             .defaultOutputMin = 0.0F,
             .defaultOutputMax = 1.0F,
             .defaultConstant = 0.5F,
             .format = "%.3f"});
    } else if (style.colorMode == PointCloudColorMode::ScalarColormap) {
        ImGui::TextDisabled("No scalar fields were discovered for this cloud.");
    }

    changed |= DrawScalarBindingWidget(
        "Point Size",
        &style.pointSize,
        session->scalarFields,
        {.constantMin = 1.0F,
         .constantMax = 16.0F,
         .defaultOutputMin = 1.0F,
         .defaultOutputMax = 16.0F,
         .defaultConstant = 2.0F,
         .format = "%.2f"});
    changed |= DrawScalarBindingWidget(
        "Opacity",
        &style.opacity,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 1.0F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 1.0F,
         .defaultConstant = 1.0F,
         .format = "%.2f"});
    changed |= DrawScalarBindingWidget(
        "Emissive",
        &style.emissiveStrength,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 2.5F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 2.5F,
         .defaultConstant = 0.0F,
         .format = "%.2f"});
    changed |= DrawScalarBindingWidget(
        "X-Ray",
        &style.xrayStrength,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 1.0F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 1.0F,
         .defaultConstant = 0.0F,
         .format = "%.2f"});
    changed |= DrawScalarBindingWidget(
        "Depth Fade",
        &style.depthFade,
        session->scalarFields,
        {.constantMin = 0.0F,
         .constantMax = 1.0F,
         .defaultOutputMin = 0.0F,
         .defaultOutputMax = 1.0F,
         .defaultConstant = 0.0F,
         .format = "%.2f"});

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
    ImGui::SliderFloat("Opacity Multiplier", &style.opacityMultiplier, 0.0F, 2.0F, "%.2f");
    ImGui::SliderFloat("Scale Multiplier", &style.scaleMultiplier, 0.1F, 4.0F, "%.2f");
    ImGui::SliderFloat("Exposure", &style.exposure, 0.0F, 4.0F, "%.2f");
    ImGui::SliderFloat("Saturation", &style.saturation, 0.0F, 2.0F, "%.2f");
}

void DrawStyleSection(PreviewRuntimeState* runtimeState) {
    ImGui::SeparatorText("Style");

    auto* session = SelectedLoadedSession(runtimeState);
    if (session == nullptr) {
        ImGui::TextUnformatted("Select a loaded layer to edit lookdev.");
        return;
    }

    if (session->kind == LayerKind::PointCloud) {
        DrawPointCloudStyleSection(session);
    } else {
        const auto effectiveQuality = EffectiveGaussianSplatQualityMode(*runtimeState, *session);
        if (effectiveQuality != session->gsplatStyle.qualityMode) {
            ImGui::TextDisabled(
                "Rendering as %s while navigating.",
                GaussianSplatQualityModeLabel(effectiveQuality));
        }
        DrawGaussianSplatStyleSection(session);
    }
}

void DrawCameraSection(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    ImGui::SeparatorText("Camera");

    auto* session = SelectedLoadedSession(runtimeState);
    if (session == nullptr || !runtimeState->camera.HasFramedBounds()) {
        ImGui::TextUnformatted("Select a loaded layer to frame the camera.");
        return;
    }

    if (ImGui::Button("Focus Selected Layer")) {
        FocusSessionLayer(runtimeState, viewport, runtimeState->selectedSessionIndex.value());
    }

    const auto effectiveFrame = ComputeEffectiveLayerFrame(*runtimeState, *session);
    const auto target = runtimeState->camera.Target();
    ImGui::Text("Target: %.3f  %.3f  %.3f", target.x, target.y, target.z);
    ImGui::Text("Distance: %.3f", runtimeState->camera.Distance());
    ImGui::Text("FOV: %.1f", runtimeState->camera.FovDegrees());
    ImGui::Text(
        "Near/Far: %.4f / %.1f",
        runtimeState->camera.NearPlane(),
        runtimeState->camera.FarPlane());
    ImGui::Text("Bounds valid: %s", effectiveFrame.bounds.valid ? "yes" : "no");
    if (session->kind == LayerKind::GaussianSplat) {
        ImGui::Text(
            "Convention: %s",
            GsplatTransformConventionLabel(runtimeState->projectSettings.gsplatTransformConvention));
    }
}

void DrawSettingsSection(
    PreviewRuntimeState* runtimeState,
    const invisible_places::renderer::core::VulkanViewportShell& viewport) {
    ImGui::SeparatorText("Settings");

    auto& settings = runtimeState->projectSettings;
    ImGui::ColorEdit3("Background Color", settings.backgroundColor.data());
    ImGui::Checkbox("Show Status Overlay", &settings.showStatusOverlay);
    ImGui::Checkbox(
        "Auto Lower gSplat Quality While Navigating",
        &settings.autoLowerGsplatQualityWhileNavigating);
    ImGui::SliderFloat(
        "gSplat Footprint Boost",
        &settings.gaussianSplatFootprintBoost,
        0.35F,
        3.0F,
        "%.2f");

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
}

void DrawProjectSection(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    ImGui::SeparatorText("Project");

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
}

void DrawPresetSection(PreviewRuntimeState* runtimeState) {
    ImGui::SeparatorText("Presets");

    InputTextString("Point Style Preset", &runtimeState->persistence.pointStylePresetPath);

    auto* session = SelectedSession(runtimeState);
    if (session == nullptr || session->kind != LayerKind::PointCloud) {
        ImGui::TextUnformatted("Select a point cloud layer to save or load a point style preset.");
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
}

void DrawSidePanel(
    PreviewRuntimeState* runtimeState,
    invisible_places::renderer::core::VulkanViewportShell* viewport) {
    auto& sidePanel = runtimeState->sidePanel;
    const auto& io = ImGui::GetIO();

    const bool nearRightEdge = io.MousePos.x >= (io.DisplaySize.x - sidePanel.handleWidth - 6.0F);
    const bool popupOpen =
        ImGui::IsPopupOpen(static_cast<const char*>(nullptr), ImGuiPopupFlags_AnyPopupId);
    const bool shouldReveal =
        sidePanel.pinned || nearRightEdge || sidePanel.hovered || sidePanel.interacting || popupOpen;
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
    ImGui::SetNextWindowBgAlpha(0.94F);

    constexpr auto flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("SceneLookdevPanel", nullptr, flags);

    sidePanel.hovered = ImGui::IsWindowHovered(
        ImGuiHoveredFlags_AllowWhenBlockedByActiveItem | ImGuiHoveredFlags_RootAndChildWindows);
    sidePanel.interacting =
        popupOpen || ImGui::IsAnyItemActive() ||
        ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    const auto windowPosition = ImGui::GetWindowPos();
    const bool handleHovered =
        sidePanel.hovered && io.MousePos.x <= (windowPosition.x + sidePanel.handleWidth + 2.0F);
    if (handleHovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        sidePanel.pinned = !sidePanel.pinned;
    }

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.48F, 0.40F, 0.22F, 0.92F});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.54F, 0.45F, 0.24F, 0.96F});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.42F, 0.34F, 0.18F, 0.98F});

    if (ImGui::Button(sidePanel.pinned ? "<" : ">", ImVec2{sidePanel.handleWidth - 6.0F, 28.0F})) {
        sidePanel.pinned = !sidePanel.pinned;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(sidePanel.pinned ? "Unpin panel" : "Pin panel");
    }

    ImGui::Spacing();
    const auto verticalLabel = MakeVerticalLabel("SCENE");
    ImGui::SetCursorPosX(std::max(2.0F, (sidePanel.handleWidth - 10.0F) * 0.5F));
    ImGui::TextUnformatted(verticalLabel.c_str());

    ImGui::PopStyleColor(3);

    if (visibleWidth > (sidePanel.handleWidth + 20.0F)) {
        ImGui::SameLine();
        ImGui::BeginGroup();
        DrawLayerSection(runtimeState, viewport);
        DrawStyleSection(runtimeState);
        DrawCameraSection(runtimeState, *viewport);
        DrawSettingsSection(runtimeState, *viewport);
        DrawProjectSection(runtimeState, viewport);
        DrawPresetSection(runtimeState);
        ImGui::EndGroup();
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
    if (!viewport.UiWantsMouseCapture()) {
        if (io.MouseWheel != 0.0F) {
            runtimeState->camera.Dolly(io.MouseWheel);
            navigatedThisFrame = true;
        }

        const bool isPanning = io.MouseDown[1] || io.MouseDown[2] || (io.KeyShift && io.MouseDown[0]);
        if (isPanning && (io.MouseDelta.x != 0.0F || io.MouseDelta.y != 0.0F)) {
            runtimeState->camera.Pan(io.MouseDelta.x, io.MouseDelta.y, io.DisplaySize.x, io.DisplaySize.y);
            navigatedThisFrame = true;
        } else if (io.MouseDown[0] && (io.MouseDelta.x != 0.0F || io.MouseDelta.y != 0.0F)) {
            runtimeState->camera.Orbit(io.MouseDelta.x, io.MouseDelta.y);
            navigatedThisFrame = true;
        }
    }

    if (!viewport.UiWantsKeyboardCapture() && ImGui::IsKeyPressed(ImGuiKey_F)) {
        if (runtimeState->selectedSessionIndex.has_value()) {
            FocusSessionLayer(runtimeState, viewport, runtimeState->selectedSessionIndex.value());
            navigatedThisFrame = true;
        }
    }

    const auto now = std::chrono::steady_clock::now();
    if (navigatedThisFrame) {
        runtimeState->cameraInteraction.lastNavigationInputAt = now;
    }
    runtimeState->cameraInteraction.navigationActive =
        navigatedThisFrame ||
        (runtimeState->cameraInteraction.lastNavigationInputAt.time_since_epoch().count() != 0 &&
         (now - runtimeState->cameraInteraction.lastNavigationInputAt) <
             std::chrono::milliseconds{180});
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
    renderState.gaussianSplatFootprintBoost = runtimeState.projectSettings.gaussianSplatFootprintBoost;

    for (std::size_t sessionIndex = 0; sessionIndex < runtimeState.sessions.size(); ++sessionIndex) {
        const auto& session = runtimeState.sessions[sessionIndex];
        if (!session.loaded || !session.visible) {
            continue;
        }

        if (session.kind == LayerKind::PointCloud) {
            renderState.pointCloudLayers.push_back(
                {.layerId = sessionIndex, .style = session.pointStyle, .hasSourceRgb = session.hasSourceRgb});
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
    runtimeState.sidePanel.panelWidth = 410.0F;
    runtimeState.persistence.projectFilePath = DefaultProjectFilePath(dataRoot_).string();
    runtimeState.persistence.pointStylePresetPath = DefaultPointStylePresetPath(dataRoot_).string();

    if (viewport.has_value()) {
        if (HasAnyPointClouds(runtimeState)) {
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

        if (viewport.has_value()) {
            PollPendingLayerLoad(&runtimeState, &viewport.value());
            StartQueuedLayerLoadIfIdle(&runtimeState);
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
