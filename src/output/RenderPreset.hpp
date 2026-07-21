#pragma once

#include "camera/AnimationPath.hpp"
#include "camera/CameraShot.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::output {

struct RenderPreset {
    std::uint32_t width = 1920;
    std::uint32_t height = 1080;
    bool tiledRendering = false;
    bool exportDepth = true;
    bool exportAlpha = true;
};

struct RenderJobSettings {
    std::string outputDirectory;
    std::uint32_t width = 1920;
    std::uint32_t height = 1080;
    std::uint32_t framesPerSecond = 30;
    float stillCameraDurationSeconds = 5.0F;
    std::uint32_t tileSize = 512;
    std::uint32_t startFrame = 0;
    std::uint32_t endFrame = 0;
    std::size_t fromShotIndex = 0;
    std::size_t toShotIndex = 1;
    std::uint32_t supersampleScale = 1;
    bool spatialAntialiasing = true;
    bool temporalSupersampling = false;
    std::uint32_t temporalSampleCount = 1;
    bool motionBlur = false;
    std::uint32_t motionBlurSampleCount = 4;
    float motionBlurShutterAngleDegrees = 180.0F;
};

enum class AnimationExportMode {
    FastPreviewMp4,
    HevcAlphaMp4,
    PngStack,
    HqPreviewDensityExr,
    ProRes422Mov,
    ProRes422HqMov,
    ProRes422VideoToolboxMov,
    ProRes422HqVideoToolboxMov,
    ProRes4444Mov,
    ProRes4444XqMov,
    ProRes4444VideoToolboxMov,
    ProRes4444XqVideoToolboxMov,
};

struct ExportPreset {
    std::string name;
    AnimationExportMode mode = AnimationExportMode::FastPreviewMp4;
    RenderJobSettings settings{};
};

constexpr std::string_view kFastPreviewMp4PresetName = "Fast Preview MP4_preset";
constexpr std::string_view kHevcAlphaMp4PresetName = "H.265 Alpha MP4_preset";
constexpr std::string_view kPngStackPresetName = "PNG Stack_preset";
constexpr std::string_view kProRes422PresetName = "ProRes 422_preset";
constexpr std::string_view kProRes422HqPresetName = "ProRes 422 HQ_preset";
constexpr std::string_view kProRes422VideoToolboxPresetName = "ProRes 422 VideoToolbox_preset";
constexpr std::string_view kProRes422HqVideoToolboxPresetName = "ProRes 422 HQ VideoToolbox_preset";
constexpr std::string_view kProRes4444PresetName = "ProRes 4444_preset";
constexpr std::string_view kProRes4444XqPresetName = "ProRes 4444 XQ_preset";
constexpr std::string_view kProRes4444VideoToolboxPresetName = "ProRes 4444 VideoToolbox_preset";
constexpr std::string_view kProRes4444XqVideoToolboxPresetName = "ProRes 4444 XQ VideoToolbox_preset";

[[nodiscard]] ExportPreset MakeFastPreviewMp4ExportPreset();
[[nodiscard]] ExportPreset MakeHevcAlphaMp4ExportPreset();
[[nodiscard]] ExportPreset MakePngStackExportPreset();
[[nodiscard]] ExportPreset MakeProRes422ExportPreset();
[[nodiscard]] ExportPreset MakeProRes422HqExportPreset();
[[nodiscard]] ExportPreset MakeProRes422VideoToolboxExportPreset();
[[nodiscard]] ExportPreset MakeProRes422HqVideoToolboxExportPreset();
[[nodiscard]] ExportPreset MakeProRes4444ExportPreset();
[[nodiscard]] ExportPreset MakeProRes4444XqExportPreset();
[[nodiscard]] ExportPreset MakeProRes4444VideoToolboxExportPreset();
[[nodiscard]] ExportPreset MakeProRes4444XqVideoToolboxExportPreset();
[[nodiscard]] std::vector<ExportPreset> BuiltInExportPresets();
[[nodiscard]] bool IsBuiltInExportPresetName(std::string_view name);
[[nodiscard]] bool IsEditedExportPresetName(std::string_view name);
[[nodiscard]] std::string BaseExportPresetName(std::string_view name);
[[nodiscard]] std::string EditedExportPresetName(std::string_view name);

std::vector<invisible_places::camera::CameraState> BuildCameraRenderSequence(
    const std::vector<invisible_places::camera::CameraShot>& shots,
    const RenderJobSettings& settings);

std::vector<invisible_places::camera::CameraState> BuildAnimationRenderSequence(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings);

[[nodiscard]] std::size_t AnimationRenderSequenceFrameCount(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings);

std::vector<invisible_places::camera::CameraState> BuildStillCameraRenderSequence(
    const invisible_places::camera::CameraState& cameraState,
    const RenderJobSettings& settings);

std::vector<float> BuildExportFrameSampleOffsetsFrames(const RenderJobSettings& settings);

float ComputePointSizePixelScale(
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    std::uint32_t setupViewportWidth,
    std::uint32_t setupViewportHeight);

std::filesystem::path RenderFramePath(
    const RenderJobSettings& settings,
    std::uint32_t frameIndex);

}  // namespace invisible_places::output
