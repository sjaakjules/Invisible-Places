#include "output/VideoWriter.hpp"

#include <Imath/half.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string_view>

namespace invisible_places::output {

namespace {

std::string ShellQuote(std::string_view value) {
    std::string quoted;
    quoted.reserve(value.size() + 2U);
    quoted.push_back('\'');
    for (const char character : value) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted.push_back(character);
        }
    }
    quoted.push_back('\'');
    return quoted;
}

float HalfBitsToFloat(std::uint16_t bits) {
    return static_cast<float>(Imath::half{Imath::half::FromBits, bits});
}

float LinearToSrgb(float value) {
    value = std::clamp(value, 0.0F, 1.0F);
    if (value <= 0.0031308F) {
        return value * 12.92F;
    }
    return (1.055F * std::pow(value, 1.0F / 2.4F)) - 0.055F;
}

std::uint8_t UnitFloatToByte(float value) {
    if (!std::isfinite(value)) {
        value = 0.0F;
    }
    return static_cast<std::uint8_t>(std::clamp(std::lround(value * 255.0F), 0L, 255L));
}

}  // namespace

std::filesystem::path DefaultFfmpegExecutablePath() {
    return std::filesystem::path{"/opt/homebrew/bin/ffmpeg"};
}

bool FfmpegExecutableAvailable(const std::filesystem::path& executablePath) {
    std::error_code statusError;
    const auto status = std::filesystem::status(executablePath, statusError);
    return !statusError &&
           std::filesystem::exists(status) &&
           (std::filesystem::is_regular_file(status) || std::filesystem::is_symlink(status));
}

std::string BuildFfmpegRawRgbaCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt rgba"
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -an"
            << " -c:v libx264"
            << " -preset veryfast"
            << " -crf 18"
            << " -pix_fmt yuv420p "
            << ShellQuote(outputPath.string());
    return command.str();
}

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(const HalfRgbaExrImage& image) {
    const auto pixelCount = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    if (image.width == 0 || image.height == 0 || image.rgbaHalf.size() != pixelCount * 4U) {
        return {};
    }

    std::vector<std::uint8_t> bytes;
    bytes.resize(pixelCount * 4U);
    for (std::size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
        const std::size_t sourceOffset = pixelIndex * 4U;
        const std::size_t destinationOffset = pixelIndex * 4U;
        bytes[destinationOffset + 0U] =
            UnitFloatToByte(LinearToSrgb(HalfBitsToFloat(image.rgbaHalf[sourceOffset + 0U])));
        bytes[destinationOffset + 1U] =
            UnitFloatToByte(LinearToSrgb(HalfBitsToFloat(image.rgbaHalf[sourceOffset + 1U])));
        bytes[destinationOffset + 2U] =
            UnitFloatToByte(LinearToSrgb(HalfBitsToFloat(image.rgbaHalf[sourceOffset + 2U])));
        bytes[destinationOffset + 3U] =
            UnitFloatToByte(std::clamp(HalfBitsToFloat(image.rgbaHalf[sourceOffset + 3U]), 0.0F, 1.0F));
    }
    return bytes;
}

}  // namespace invisible_places::output
