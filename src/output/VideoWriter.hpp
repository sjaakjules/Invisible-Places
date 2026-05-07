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

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(const HalfRgbaExrImage& image);

}  // namespace invisible_places::output
