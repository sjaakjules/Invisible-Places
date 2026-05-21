#include "output/VideoWriter.hpp"

#include <Imath/half.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string_view>

namespace invisible_places::output {

namespace {

struct LinearRgbaPixel {
    float r = 0.0F;
    float g = 0.0F;
    float b = 0.0F;
    float a = 0.0F;
    float depth = 0.0F;
};

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

std::string SanitizeFileStem(std::string_view name, std::string_view fallback) {
    std::string stem;
    stem.reserve(name.size());
    bool previousWasSeparator = false;
    for (const char character : name) {
        const auto unsignedCharacter = static_cast<unsigned char>(character);
        if (std::isalnum(unsignedCharacter) != 0) {
            stem.push_back(character);
            previousWasSeparator = false;
        } else if (!previousWasSeparator) {
            stem.push_back('_');
            previousWasSeparator = true;
        }
    }

    while (!stem.empty() && stem.back() == '_') {
        stem.pop_back();
    }
    return stem.empty() ? std::string{fallback} : stem;
}

bool Covered(const LinearRgbaPixel& pixel, float alphaThreshold) {
    return pixel.a > alphaThreshold;
}

bool ValidDepth(float depth) {
    return std::isfinite(depth) && depth > 0.0F;
}

float LerpFloat(float left, float right, float amount) {
    return left + ((right - left) * amount);
}

bool DepthCompatible(
    const LinearRgbaPixel& center,
    const LinearRgbaPixel& sample,
    const Mp4SparsePointSmoothingSettings& settings) {
    if (!ValidDepth(center.depth) || !ValidDepth(sample.depth)) {
        return true;
    }

    const float tolerance = std::max(
        std::max(0.0F, settings.depthAbsoluteTolerance),
        std::abs(center.depth) * std::max(0.0F, settings.depthRelativeTolerance));
    return std::abs(center.depth - sample.depth) <= tolerance;
}

void ApplySparsePointSmoothing(
    std::vector<LinearRgbaPixel>* pixels,
    std::uint32_t width,
    std::uint32_t height,
    const Mp4SparsePointSmoothingSettings& settings) {
    if (pixels == nullptr ||
        !settings.enabled ||
        settings.radiusPixels == 0 ||
        width == 0 ||
        height == 0 ||
        pixels->size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
        return;
    }

    const auto source = *pixels;
    const auto radius = static_cast<int>(std::min<std::uint32_t>(settings.radiusPixels, 4U));
    const float alphaThreshold = std::max(0.0F, settings.alphaThreshold);
    const float gapFillStrength = std::clamp(settings.gapFillStrength, 0.0F, 1.0F);
    const float coveredBlendStrength = std::clamp(settings.coveredPixelBlendStrength, 0.0F, 1.0F);

    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const auto pixelIndex = static_cast<std::size_t>(y) * width + x;
            const auto& center = source[pixelIndex];
            const bool centerCovered = Covered(center, alphaThreshold);

            float weightSum = 0.0F;
            float redSum = 0.0F;
            float greenSum = 0.0F;
            float blueSum = 0.0F;
            std::uint32_t coveredSamples = 0;

            for (int offsetY = -radius; offsetY <= radius; ++offsetY) {
                const int sampleY = static_cast<int>(y) + offsetY;
                if (sampleY < 0 || sampleY >= static_cast<int>(height)) {
                    continue;
                }

                for (int offsetX = -radius; offsetX <= radius; ++offsetX) {
                    const int sampleX = static_cast<int>(x) + offsetX;
                    if (sampleX < 0 || sampleX >= static_cast<int>(width)) {
                        continue;
                    }

                    const auto sampleIndex =
                        static_cast<std::size_t>(sampleY) * width + static_cast<std::uint32_t>(sampleX);
                    const auto& sample = source[sampleIndex];
                    if (!Covered(sample, alphaThreshold)) {
                        continue;
                    }
                    if (centerCovered && !DepthCompatible(center, sample, settings)) {
                        continue;
                    }

                    const float distance = std::sqrt(
                        static_cast<float>((offsetX * offsetX) + (offsetY * offsetY)));
                    const float weight = std::max(0.0F, sample.a) / (1.0F + distance);
                    weightSum += weight;
                    redSum += sample.r * weight;
                    greenSum += sample.g * weight;
                    blueSum += sample.b * weight;
                    ++coveredSamples;
                }
            }

            if (weightSum <= 1.0e-6F) {
                continue;
            }

            const float averageRed = redSum / weightSum;
            const float averageGreen = greenSum / weightSum;
            const float averageBlue = blueSum / weightSum;
            float blendStrength = 0.0F;
            if (!centerCovered) {
                blendStrength = gapFillStrength;
            } else if (coveredSamples > 1U) {
                blendStrength = coveredBlendStrength;
            }

            if (blendStrength <= 0.0F) {
                continue;
            }

            auto& destination = (*pixels)[pixelIndex];
            destination.r = LerpFloat(destination.r, averageRed, blendStrength);
            destination.g = LerpFloat(destination.g, averageGreen, blendStrength);
            destination.b = LerpFloat(destination.b, averageBlue, blendStrength);
        }
    }
}

std::vector<LinearRgbaPixel> DownsampleLinearRgba(
    const std::vector<LinearRgbaPixel>& source,
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight) {
    const auto sourcePixelCount =
        static_cast<std::size_t>(sourceWidth) * static_cast<std::size_t>(sourceHeight);
    if (sourceWidth == 0 ||
        sourceHeight == 0 ||
        outputWidth == 0 ||
        outputHeight == 0 ||
        source.size() != sourcePixelCount) {
        return {};
    }
    if (sourceWidth == outputWidth && sourceHeight == outputHeight) {
        return source;
    }

    std::vector<LinearRgbaPixel> output(
        static_cast<std::size_t>(outputWidth) * static_cast<std::size_t>(outputHeight));
    const double scaleX = static_cast<double>(sourceWidth) / static_cast<double>(outputWidth);
    const double scaleY = static_cast<double>(sourceHeight) / static_cast<double>(outputHeight);

    for (std::uint32_t y = 0; y < outputHeight; ++y) {
        const double sourceY0 = static_cast<double>(y) * scaleY;
        const double sourceY1 = static_cast<double>(y + 1U) * scaleY;
        const auto firstY = static_cast<std::uint32_t>(
            std::clamp(std::floor(sourceY0), 0.0, static_cast<double>(sourceHeight - 1U)));
        const auto lastY = static_cast<std::uint32_t>(
            std::clamp(std::ceil(sourceY1), 1.0, static_cast<double>(sourceHeight)));

        for (std::uint32_t x = 0; x < outputWidth; ++x) {
            const double sourceX0 = static_cast<double>(x) * scaleX;
            const double sourceX1 = static_cast<double>(x + 1U) * scaleX;
            const auto firstX = static_cast<std::uint32_t>(
                std::clamp(std::floor(sourceX0), 0.0, static_cast<double>(sourceWidth - 1U)));
            const auto lastX = static_cast<std::uint32_t>(
                std::clamp(std::ceil(sourceX1), 1.0, static_cast<double>(sourceWidth)));

            float premultipliedRed = 0.0F;
            float premultipliedGreen = 0.0F;
            float premultipliedBlue = 0.0F;
            float alphaSum = 0.0F;
            float depthSum = 0.0F;
            float depthWeight = 0.0F;
            float sampleCount = 0.0F;

            for (std::uint32_t sampleY = firstY; sampleY < lastY; ++sampleY) {
                for (std::uint32_t sampleX = firstX; sampleX < lastX; ++sampleX) {
                    const auto& sample =
                        source[static_cast<std::size_t>(sampleY) * sourceWidth + sampleX];
                    premultipliedRed += sample.r * sample.a;
                    premultipliedGreen += sample.g * sample.a;
                    premultipliedBlue += sample.b * sample.a;
                    alphaSum += sample.a;
                    if (ValidDepth(sample.depth)) {
                        depthSum += sample.depth * std::max(sample.a, 0.001F);
                        depthWeight += std::max(sample.a, 0.001F);
                    }
                    sampleCount += 1.0F;
                }
            }

            auto& destination = output[static_cast<std::size_t>(y) * outputWidth + x];
            if (sampleCount <= 1.0e-6F) {
                continue;
            }

            destination.a = std::clamp(alphaSum / sampleCount, 0.0F, 1.0F);
            if (alphaSum > 1.0e-6F) {
                destination.r = premultipliedRed / alphaSum;
                destination.g = premultipliedGreen / alphaSum;
                destination.b = premultipliedBlue / alphaSum;
            }
            destination.depth = depthWeight > 1.0e-6F ? depthSum / depthWeight : 0.0F;
        }
    }

    return output;
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

std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName) {
    return BuildUniqueQuickMp4OutputPath(outputDirectory, animationName, visualName, {});
}

std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    const auto baseStem =
        SanitizeFileStem(animationName, "Animation") + "_" + SanitizeFileStem(visualName, "Visual");
    auto candidate = outputDirectory / (baseStem + ".mp4");
    const auto reserved = [&reservedPaths](const std::filesystem::path& path) {
        const auto normalized = path.lexically_normal();
        return std::any_of(
            reservedPaths.begin(),
            reservedPaths.end(),
            [&normalized](const std::filesystem::path& reservedPath) {
                return reservedPath.lexically_normal() == normalized;
            });
    };
    for (std::uint32_t suffix = 1; std::filesystem::exists(candidate) || reserved(candidate); ++suffix) {
        candidate = outputDirectory / (baseStem + "_" + std::to_string(suffix) + ".mp4");
    }
    return candidate;
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

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(
    const HalfRgbaExrImage& image,
    const Mp4SparsePointSmoothingSettings& smoothing) {
    return ConvertHalfRgbaToSrgbRgba8(image, image.width, image.height, smoothing);
}

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Mp4SparsePointSmoothingSettings& smoothing) {
    const auto pixelCount = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    if (image.width == 0 ||
        image.height == 0 ||
        outputWidth == 0 ||
        outputHeight == 0 ||
        image.rgbaHalf.size() != pixelCount * 4U) {
        return {};
    }

    std::vector<LinearRgbaPixel> pixels(pixelCount);
    const bool hasDepth = image.depth.size() == pixelCount;
    for (std::size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
        const std::size_t sourceOffset = pixelIndex * 4U;
        pixels[pixelIndex] = {
            .r = HalfBitsToFloat(image.rgbaHalf[sourceOffset + 0U]),
            .g = HalfBitsToFloat(image.rgbaHalf[sourceOffset + 1U]),
            .b = HalfBitsToFloat(image.rgbaHalf[sourceOffset + 2U]),
            .a = std::clamp(HalfBitsToFloat(image.rgbaHalf[sourceOffset + 3U]), 0.0F, 1.0F),
            .depth = hasDepth ? image.depth[pixelIndex] : 0.0F,
        };
    }

    ApplySparsePointSmoothing(&pixels, image.width, image.height, smoothing);
    pixels = DownsampleLinearRgba(pixels, image.width, image.height, outputWidth, outputHeight);
    if (pixels.empty()) {
        return {};
    }

    std::vector<std::uint8_t> bytes;
    const auto outputPixelCount =
        static_cast<std::size_t>(outputWidth) * static_cast<std::size_t>(outputHeight);
    bytes.resize(outputPixelCount * 4U);
    for (std::size_t pixelIndex = 0; pixelIndex < outputPixelCount; ++pixelIndex) {
        const std::size_t destinationOffset = pixelIndex * 4U;
        bytes[destinationOffset + 0U] = UnitFloatToByte(LinearToSrgb(pixels[pixelIndex].r));
        bytes[destinationOffset + 1U] = UnitFloatToByte(LinearToSrgb(pixels[pixelIndex].g));
        bytes[destinationOffset + 2U] = UnitFloatToByte(LinearToSrgb(pixels[pixelIndex].b));
        bytes[destinationOffset + 3U] = UnitFloatToByte(pixels[pixelIndex].a);
    }
    return bytes;
}

}  // namespace invisible_places::output
