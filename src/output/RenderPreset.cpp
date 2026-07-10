#include "output/RenderPreset.hpp"

#include "camera/CameraPath.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace invisible_places::output {

namespace {

std::uint32_t ScaleThirtyFpsFramesToOutputFps(
    std::uint32_t sourceDurationFrames,
    std::uint32_t framesPerSecond) {
    return std::max<std::uint32_t>(
        1U,
        static_cast<std::uint32_t>(
            ((static_cast<std::uint64_t>(std::max<std::uint32_t>(1U, sourceDurationFrames)) *
              std::max<std::uint32_t>(1U, framesPerSecond)) +
             15ULL) /
            30ULL));
}

std::vector<invisible_places::camera::CameraState> SliceFrameRange(
    const std::vector<invisible_places::camera::CameraState>& frames,
    const RenderJobSettings& settings) {
    if (frames.empty()) {
        return {};
    }

    const std::uint32_t startFrame = std::min<std::uint32_t>(
        settings.startFrame,
        static_cast<std::uint32_t>(frames.size() - 1U));
    const std::uint32_t endFrame = settings.endFrame == 0
                                       ? static_cast<std::uint32_t>(frames.size() - 1U)
                                       : std::min<std::uint32_t>(
                                             settings.endFrame,
                                             static_cast<std::uint32_t>(frames.size() - 1U));
    if (startFrame > endFrame) {
        return {};
    }

    return std::vector<invisible_places::camera::CameraState>{
        frames.begin() + static_cast<std::ptrdiff_t>(startFrame),
        frames.begin() + static_cast<std::ptrdiff_t>(endFrame + 1U)};
}

struct AnimationRenderFrameRange {
    std::uint32_t durationFrames = 0;
    std::uint32_t startFrame = 0;
    std::uint32_t endFrame = 0;
    bool valid = false;
};

std::uint32_t AnimationRenderSequenceDurationFrames(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings) {
    if (path.keys.empty()) {
        return 0;
    }

    const auto minimumFrameSpans = path.keys.size() > 1U
                                       ? static_cast<std::uint32_t>(path.keys.size() - 1U)
                                       : 1U;
    const float durationSeconds = invisible_places::camera::AnimationPathDurationSeconds(path);
    const auto roundedDurationFrames = static_cast<std::uint32_t>(
        std::max(
            1.0,
            std::round(
                static_cast<double>(durationSeconds) *
                static_cast<double>(std::max<std::uint32_t>(1U, settings.framesPerSecond)))));
    return std::max<std::uint32_t>(minimumFrameSpans, roundedDurationFrames);
}

AnimationRenderFrameRange BuildAnimationRenderFrameRange(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings) {
    const auto durationFrames = AnimationRenderSequenceDurationFrames(path, settings);
    if (durationFrames == 0) {
        return {};
    }

    const auto maxFrame = durationFrames;
    const auto startFrame = std::min<std::uint32_t>(settings.startFrame, maxFrame);
    const auto endFrame = settings.endFrame == 0
                              ? maxFrame
                              : std::min<std::uint32_t>(settings.endFrame, maxFrame);
    if (startFrame > endFrame) {
        return {.durationFrames = durationFrames};
    }
    return {
        .durationFrames = durationFrames,
        .startFrame = startFrame,
        .endFrame = endFrame,
        .valid = true,
    };
}

std::string NormalizeExportPresetName(std::string_view name) {
    std::string normalized;
    normalized.reserve(name.size());
    bool previousWasWhitespace = false;
    for (const char character : name) {
        const auto unsignedCharacter = static_cast<unsigned char>(character);
        if (std::isspace(unsignedCharacter) != 0) {
            if (!normalized.empty() && !previousWasWhitespace) {
                normalized.push_back(' ');
            }
            previousWasWhitespace = true;
        } else {
            normalized.push_back(character);
            previousWasWhitespace = false;
        }
    }
    while (!normalized.empty() && normalized.back() == ' ') {
        normalized.pop_back();
    }
    return normalized.empty() ? std::string{kFastPreviewMp4PresetName} : normalized;
}

}  // namespace

ExportPreset MakeFastPreviewMp4ExportPreset() {
    ExportPreset preset;
    preset.name = std::string{kFastPreviewMp4PresetName};
    preset.mode = AnimationExportMode::FastPreviewMp4;
    preset.settings.width = 1920;
    preset.settings.height = 1080;
    preset.settings.framesPerSecond = 30;
    preset.settings.supersampleScale = 2;
    preset.settings.spatialAntialiasing = true;
    preset.settings.temporalSupersampling = false;
    preset.settings.temporalSampleCount = 1;
    preset.settings.motionBlur = false;
    preset.settings.motionBlurSampleCount = 4;
    preset.settings.motionBlurShutterAngleDegrees = 180.0F;
    return preset;
}

ExportPreset MakeProRes4444ExportPreset() {
    ExportPreset preset;
    preset.name = std::string{kProRes4444PresetName};
    preset.mode = AnimationExportMode::ProRes4444Mov;
    preset.settings.width = 3840;
    preset.settings.height = 2160;
    preset.settings.framesPerSecond = 30;
    preset.settings.supersampleScale = 2;
    preset.settings.spatialAntialiasing = true;
    preset.settings.temporalSupersampling = true;
    preset.settings.temporalSampleCount = 4;
    preset.settings.motionBlur = false;
    preset.settings.motionBlurSampleCount = 4;
    preset.settings.motionBlurShutterAngleDegrees = 180.0F;
    return preset;
}

ExportPreset MakeProRes422ExportPreset() {
    auto preset = MakeProRes4444ExportPreset();
    preset.name = std::string{kProRes422PresetName};
    preset.mode = AnimationExportMode::ProRes422Mov;
    return preset;
}

ExportPreset MakeProRes422HqExportPreset() {
    auto preset = MakeProRes422ExportPreset();
    preset.name = std::string{kProRes422HqPresetName};
    preset.mode = AnimationExportMode::ProRes422HqMov;
    return preset;
}

ExportPreset MakeProRes422VideoToolboxExportPreset() {
    auto preset = MakeProRes422ExportPreset();
    preset.name = std::string{kProRes422VideoToolboxPresetName};
    preset.mode = AnimationExportMode::ProRes422VideoToolboxMov;
    return preset;
}

ExportPreset MakeProRes422HqVideoToolboxExportPreset() {
    auto preset = MakeProRes422HqExportPreset();
    preset.name = std::string{kProRes422HqVideoToolboxPresetName};
    preset.mode = AnimationExportMode::ProRes422HqVideoToolboxMov;
    return preset;
}

ExportPreset MakeProRes4444XqExportPreset() {
    auto preset = MakeProRes4444ExportPreset();
    preset.name = std::string{kProRes4444XqPresetName};
    preset.mode = AnimationExportMode::ProRes4444XqMov;
    return preset;
}

ExportPreset MakeProRes4444VideoToolboxExportPreset() {
    auto preset = MakeProRes4444ExportPreset();
    preset.name = std::string{kProRes4444VideoToolboxPresetName};
    preset.mode = AnimationExportMode::ProRes4444VideoToolboxMov;
    return preset;
}

ExportPreset MakeProRes4444XqVideoToolboxExportPreset() {
    auto preset = MakeProRes4444ExportPreset();
    preset.name = std::string{kProRes4444XqVideoToolboxPresetName};
    preset.mode = AnimationExportMode::ProRes4444XqVideoToolboxMov;
    return preset;
}

std::vector<ExportPreset> BuiltInExportPresets() {
    return {
        MakeFastPreviewMp4ExportPreset(),
        MakeProRes422ExportPreset(),
        MakeProRes422HqExportPreset(),
        MakeProRes422VideoToolboxExportPreset(),
        MakeProRes422HqVideoToolboxExportPreset(),
        MakeProRes4444ExportPreset(),
        MakeProRes4444XqExportPreset(),
        MakeProRes4444VideoToolboxExportPreset(),
        MakeProRes4444XqVideoToolboxExportPreset()};
}

bool IsBuiltInExportPresetName(std::string_view name) {
    const auto normalized = NormalizeExportPresetName(name);
    return normalized == kFastPreviewMp4PresetName ||
           normalized == kProRes422PresetName ||
           normalized == kProRes422HqPresetName ||
           normalized == kProRes422VideoToolboxPresetName ||
           normalized == kProRes422HqVideoToolboxPresetName ||
           normalized == kProRes4444PresetName ||
           normalized == kProRes4444XqPresetName ||
           normalized == kProRes4444VideoToolboxPresetName ||
           normalized == kProRes4444XqVideoToolboxPresetName;
}

bool IsEditedExportPresetName(std::string_view name) {
    constexpr std::string_view suffix = "_edited";
    const auto normalized = NormalizeExportPresetName(name);
    return normalized.size() > suffix.size() &&
           std::string_view{normalized}.substr(normalized.size() - suffix.size()) == suffix;
}

std::string BaseExportPresetName(std::string_view name) {
    constexpr std::string_view suffix = "_edited";
    auto normalized = NormalizeExportPresetName(name);
    if (normalized.size() > suffix.size() &&
        std::string_view{normalized}.substr(normalized.size() - suffix.size()) == suffix) {
        normalized.erase(normalized.size() - suffix.size());
    }
    return normalized;
}

std::string EditedExportPresetName(std::string_view name) {
    return BaseExportPresetName(name) + "_edited";
}

std::vector<invisible_places::camera::CameraState> BuildCameraRenderSequence(
    const std::vector<invisible_places::camera::CameraShot>& shots,
    const RenderJobSettings& settings) {
    if (shots.size() < 2) {
        return {};
    }

    const std::size_t fromIndex = std::min(settings.fromShotIndex, shots.size() - 1U);
    const std::size_t toIndex = std::min(settings.toShotIndex, shots.size() - 1U);
    if (fromIndex >= toIndex) {
        return {};
    }

    const auto timing = invisible_places::camera::BuildCameraPathTiming(shots, fromIndex, toIndex);
    if (!timing.IsValid()) {
        return {};
    }

    std::vector<invisible_places::camera::CameraState> frames;
    for (std::size_t shotIndex = fromIndex; shotIndex < toIndex; ++shotIndex) {
        const auto& toShot = shots[shotIndex + 1U];
        const auto sourceDurationFrames = std::max<std::uint32_t>(1U, toShot.durationFrames);
        const auto durationFrames = ScaleThirtyFpsFramesToOutputFps(sourceDurationFrames, settings.framesPerSecond);
        const auto timingIndex = shotIndex - fromIndex;
        const float segmentStartSeconds = timing.knotSeconds[timingIndex];
        const float segmentEndSeconds = timing.knotSeconds[timingIndex + 1U];
        for (std::uint32_t frameIndex = 0; frameIndex < durationFrames; ++frameIndex) {
            const float t = static_cast<float>(frameIndex) / static_cast<float>(durationFrames);
            const float timeSeconds =
                segmentStartSeconds + ((segmentEndSeconds - segmentStartSeconds) * t);
            frames.push_back(invisible_places::camera::EvaluateCameraPath(shots, timing, timeSeconds));
        }
    }
    frames.push_back(invisible_places::camera::EvaluateCameraPath(shots, timing, timing.DurationSeconds()));

    return SliceFrameRange(frames, settings);
}

std::vector<invisible_places::camera::CameraState> BuildAnimationRenderSequence(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings) {
    const auto frameRange = BuildAnimationRenderFrameRange(path, settings);
    if (!frameRange.valid) {
        return {};
    }

    const float durationSeconds = invisible_places::camera::AnimationPathDurationSeconds(path);
    const auto preparedPath = invisible_places::camera::PrepareAnimationPathEvaluation(path);

    std::vector<invisible_places::camera::CameraState> frames;
    frames.reserve(
        static_cast<std::size_t>(frameRange.endFrame - frameRange.startFrame) + 1U);
    for (std::uint32_t frameIndex = frameRange.startFrame;
         frameIndex <= frameRange.endFrame;
         ++frameIndex) {
        const float t = static_cast<float>(frameIndex) /
                        static_cast<float>(std::max<std::uint32_t>(1U, frameRange.durationFrames));
        frames.push_back(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                preparedPath,
                durationSeconds * t)
                .camera);
    }
    return frames;
}

std::size_t AnimationRenderSequenceFrameCount(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& settings) {
    const auto frameRange = BuildAnimationRenderFrameRange(path, settings);
    return frameRange.valid
               ? static_cast<std::size_t>(frameRange.endFrame - frameRange.startFrame) + 1U
               : 0U;
}

std::vector<invisible_places::camera::CameraState> BuildStillCameraRenderSequence(
    const invisible_places::camera::CameraState& cameraState,
    const RenderJobSettings& settings) {
    const auto frameCount = std::max<std::uint32_t>(
        1U,
        static_cast<std::uint32_t>(
            std::ceil(
                std::max(0.001F, settings.stillCameraDurationSeconds) *
                static_cast<float>(std::max<std::uint32_t>(1U, settings.framesPerSecond)))));
    return std::vector<invisible_places::camera::CameraState>(frameCount, cameraState);
}

std::vector<float> BuildExportFrameSampleOffsetsFrames(const RenderJobSettings& settings) {
    std::uint32_t sampleCount = 1U;
    float shutterWidthFrames = 0.0F;

    if (settings.temporalSupersampling) {
        sampleCount = std::max(sampleCount, std::max<std::uint32_t>(1U, settings.temporalSampleCount));
        shutterWidthFrames = std::max(shutterWidthFrames, 1.0F);
    }
    if (settings.motionBlur) {
        sampleCount = std::max(sampleCount, std::max<std::uint32_t>(1U, settings.motionBlurSampleCount));
        shutterWidthFrames = std::max(
            shutterWidthFrames,
            std::clamp(settings.motionBlurShutterAngleDegrees, 0.0F, 360.0F) / 360.0F);
    }

    if (sampleCount <= 1U || shutterWidthFrames <= 1.0e-6F) {
        return {0.0F};
    }

    std::vector<float> offsets;
    offsets.reserve(sampleCount);
    for (std::uint32_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
        const float centered =
            ((static_cast<float>(sampleIndex) + 0.5F) / static_cast<float>(sampleCount)) - 0.5F;
        offsets.push_back(centered * shutterWidthFrames);
    }
    return offsets;
}

float ComputePointSizePixelScale(
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    std::uint32_t setupViewportWidth,
    std::uint32_t setupViewportHeight) {
    if (outputWidth == 0 ||
        outputHeight == 0 ||
        setupViewportWidth == 0 ||
        setupViewportHeight == 0) {
        return 1.0F;
    }

    const float widthScale = static_cast<float>(outputWidth) / static_cast<float>(setupViewportWidth);
    const float heightScale = static_cast<float>(outputHeight) / static_cast<float>(setupViewportHeight);
    return std::max(0.001F, std::sqrt(std::max(0.0F, widthScale * heightScale)));
}

std::filesystem::path RenderFramePath(
    const RenderJobSettings& settings,
    std::uint32_t frameIndex) {
    std::ostringstream filename;
    filename << "frame_" << std::setw(6) << std::setfill('0') << (frameIndex + 1U) << ".exr";
    return std::filesystem::path{settings.outputDirectory} / filename.str();
}

}  // namespace invisible_places::output
