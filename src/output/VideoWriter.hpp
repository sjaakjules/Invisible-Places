#pragma once

#include "output/ExrWriter.hpp"
#include "output/RenderPreset.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::output {

constexpr std::uint32_t kTestMp4OutputFramesPerSecond = 30U;

std::filesystem::path DefaultFfmpegExecutablePath();

bool FfmpegExecutableAvailable(const std::filesystem::path& executablePath);

struct HevcAlphaMp4OutputPaths {
    std::filesystem::path colorPath;
    std::filesystem::path alphaMattePath;
};

struct ProResAlphaMatteOutputPaths {
    std::filesystem::path colorPath;
    std::filesystem::path alphaMattePath;
};

struct Mp4SparsePointSmoothingSettings {
    // Kept temporarily so existing app-side call sites remain source-compatible.
    bool enabled = false;
};

const char* AnimationExportModeFilenameToken(AnimationExportMode mode);
std::string AnimationExportModeFilenameToken(
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    bool externalAlphaMatte);
std::string AnimationExportSettingsFilenameToken(
    const RenderJobSettings& settings,
    AnimationExportMode mode);
std::string BuildAnimationExportFilenameStem(
    std::string_view animationName,
    AnimationExportMode mode,
    const RenderJobSettings& settings,
    std::string_view visualName = {});
std::string BuildAnimationExportFilenameStem(
    std::string_view animationName,
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    bool externalAlphaMatte,
    const RenderJobSettings& settings,
    std::string_view visualName = {});
std::filesystem::path BuildUniqueAnimationExportMediaOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    AnimationExportMode mode,
    const RenderJobSettings& settings,
    std::string_view extension,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueAnimationExportMediaOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    bool externalAlphaMatte,
    const RenderJobSettings& settings,
    std::string_view extension,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});

std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName);
std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths);
std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueHevcAlphaMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
HevcAlphaMp4OutputPaths BuildUniqueHevcAlphaMp4OutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
HevcAlphaMp4OutputPaths BuildUniqueMp4AlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniquePngStackOutputDirectory(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {},
    AnimationExportMode mode = AnimationExportMode::PngStack);
std::filesystem::path PngStackFramePath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::uint32_t frameIndex);
std::filesystem::path PngStackFramePattern(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName);
std::filesystem::path BuildUniqueVideoOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    std::string_view formatSuffix,
    std::string_view extension,
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes422OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes422HqOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
ProResAlphaMatteOutputPaths BuildUniqueProRes422AlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
ProResAlphaMatteOutputPaths BuildUniqueProRes422HqAlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
ProResAlphaMatteOutputPaths BuildUniqueProResAlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    AnimationExportMode mode,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes422VideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes422HqVideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444XqOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444XqOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444VideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444VideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444XqVideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths = {});
std::filesystem::path BuildUniqueProRes4444XqVideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName = {},
    const std::vector<std::filesystem::path>& reservedPaths = {});

std::string BuildFfmpegRawRgbaCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegTestMp4Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t sourceFramesPerSecond,
    std::uint32_t sourceFrameCount,
    const std::filesystem::path& outputPath,
    bool useVideoToolbox);
std::string BuildFfmpegHevcAlphaMp4Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegHevcColorMp4Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegMp4ColorCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox);
std::string BuildFfmpegHevcAlphaMatteMp4Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegMp4AlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox);
std::string BuildFfmpegMp4ColorAndAlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& colorOutputPath,
    const std::filesystem::path& alphaMatteOutputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox);
std::string BuildFfmpegPngStackCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPattern);
std::string BuildFfmpegProRes422Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegProRes422Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox);
std::string BuildFfmpegProRes422ColorAndAlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& colorOutputPath,
    const std::filesystem::path& alphaMatteOutputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox);
std::string BuildFfmpegProRes422HqCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegProRes422VideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegProRes422HqVideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegProRes4444Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegProRes4444Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox);
std::string BuildFfmpegProRes4444ColorAndAlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& colorOutputPath,
    const std::filesystem::path& alphaMatteOutputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox);
std::string BuildFfmpegProRes4444XqCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegProRes4444VideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);
std::string BuildFfmpegProRes4444XqVideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);

invisible_places::output::HalfRgbaExrImage AverageHalfRgbaFrames(
    const std::vector<HalfRgbaExrImage>& images);

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(
    const HalfRgbaExrImage& image,
    const Mp4SparsePointSmoothingSettings& smoothing = {});
std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Mp4SparsePointSmoothingSettings& smoothing = {},
    bool spatialAntialiasing = true);
std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba16(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    bool spatialAntialiasing = true);
std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgb16OpaqueBlack(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    bool spatialAntialiasing = true);

}  // namespace invisible_places::output
