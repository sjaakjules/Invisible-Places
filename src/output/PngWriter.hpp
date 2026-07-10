#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::output {

[[nodiscard]] bool WritePngRgba8(
    const std::filesystem::path& outputPath,
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t>& rgba,
    std::string* errorMessage = nullptr);

}  // namespace invisible_places::output
