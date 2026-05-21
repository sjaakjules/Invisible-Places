#pragma once

#include "output/ExrWriter.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::output {

std::filesystem::path DefaultFfmpegExecutablePath();

bool FfmpegExecutableAvailable(const std::filesystem::path& executablePath);

struct Mp4SparsePointSmoothingSettings {
    bool enabled = true;
    std::uint32_t radiusPixels = 1;
    float alphaThreshold = 0.01F;
    float gapFillStrength = 0.72F;
    float coveredPixelBlendStrength = 0.08F;
    float depthAbsoluteTolerance = 0.05F;
    float depthRelativeTolerance = 0.015F;
};

std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName);
std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths);

std::string BuildFfmpegRawRgbaCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(
    const HalfRgbaExrImage& image,
    const Mp4SparsePointSmoothingSettings& smoothing = {});
std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Mp4SparsePointSmoothingSettings& smoothing = {});

}  // namespace invisible_places::output
