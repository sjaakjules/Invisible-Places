#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::output {

struct ExrImage {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::vector<float> beautyR;
    std::vector<float> beautyG;
    std::vector<float> beautyB;
    std::vector<float> alpha;
    std::vector<float> depth;
};

struct HalfRgbaExrImage {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::vector<std::uint16_t> rgbaHalf;
    std::vector<float> depth;
};

bool WriteExrImage(
    const ExrImage& image,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);

bool WriteExrImage(
    const HalfRgbaExrImage& image,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);

}  // namespace invisible_places::output
