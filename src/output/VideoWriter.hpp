#pragma once

#include "output/ExrWriter.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::output {

std::filesystem::path DefaultFfmpegExecutablePath();

bool FfmpegExecutableAvailable(const std::filesystem::path& executablePath);

std::string BuildFfmpegRawRgbaCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath);

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(const HalfRgbaExrImage& image);

}  // namespace invisible_places::output
