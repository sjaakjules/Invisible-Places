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

std::uint16_t UnitFloatToWord(float value) {
    if (!std::isfinite(value)) {
        value = 0.0F;
    }
    return static_cast<std::uint16_t>(std::clamp(std::lround(value * 65535.0F), 0L, 65535L));
}

std::uint16_t FloatToHalfBits(float value) {
    if (!std::isfinite(value)) {
        value = 0.0F;
    }
    return Imath::half{value}.bits();
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

std::vector<LinearRgbaPixel> DownsampleLinearRgbaNearest(
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
    for (std::uint32_t y = 0; y < outputHeight; ++y) {
        const auto sourceY = std::min<std::uint32_t>(
            sourceHeight - 1U,
            static_cast<std::uint32_t>(
                (static_cast<std::uint64_t>(y) * sourceHeight) / outputHeight));
        for (std::uint32_t x = 0; x < outputWidth; ++x) {
            const auto sourceX = std::min<std::uint32_t>(
                sourceWidth - 1U,
                static_cast<std::uint32_t>(
                    (static_cast<std::uint64_t>(x) * sourceWidth) / outputWidth));
            output[static_cast<std::size_t>(y) * outputWidth + x] =
                source[static_cast<std::size_t>(sourceY) * sourceWidth + sourceX];
        }
    }
    return output;
}

std::vector<LinearRgbaPixel> ResampleLinearRgba(
    const std::vector<LinearRgbaPixel>& source,
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    bool spatialAntialiasing) {
    return spatialAntialiasing
               ? DownsampleLinearRgba(source, sourceWidth, sourceHeight, outputWidth, outputHeight)
               : DownsampleLinearRgbaNearest(source, sourceWidth, sourceHeight, outputWidth, outputHeight);
}

std::vector<LinearRgbaPixel> HalfRgbaToLinearPixels(const HalfRgbaExrImage& image) {
    const auto pixelCount = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    if (image.width == 0 ||
        image.height == 0 ||
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
    return pixels;
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

const char* AnimationExportModeFilenameToken(AnimationExportMode mode) {
    switch (mode) {
        case AnimationExportMode::FastPreviewMp4:
            return "fast";
        case AnimationExportMode::HqPreviewDensityExr:
            return "HQ";
        case AnimationExportMode::ProRes422Mov:
            return "ProRes422";
        case AnimationExportMode::ProRes422HqMov:
            return "ProRes422HQ";
        case AnimationExportMode::ProRes422VideoToolboxMov:
            return "ProRes422M1";
        case AnimationExportMode::ProRes422HqVideoToolboxMov:
            return "ProRes422HQM1";
        case AnimationExportMode::ProRes4444Mov:
            return "ProRes";
        case AnimationExportMode::ProRes4444XqMov:
            return "ProResXQ";
        case AnimationExportMode::ProRes4444VideoToolboxMov:
            return "ProResM1";
        case AnimationExportMode::ProRes4444XqVideoToolboxMov:
            return "ProResXQM1";
    }
    return "export";
}

std::string AnimationExportSettingsFilenameToken(
    const RenderJobSettings& settings,
    AnimationExportMode mode) {
    std::ostringstream token;
    token << std::max<std::uint32_t>(1U, settings.width) << "x"
          << std::max<std::uint32_t>(1U, settings.height) << "_"
          << std::max<std::uint32_t>(1U, settings.framesPerSecond) << "fps";

    if (mode != AnimationExportMode::HqPreviewDensityExr) {
        token << "_SS" << std::max<std::uint32_t>(1U, settings.supersampleScale) << "x";
    }

    token << (settings.spatialAntialiasing ? "_AA" : "_NoAA");

    if (settings.temporalSupersampling) {
        token << "_TS" << std::max<std::uint32_t>(1U, settings.temporalSampleCount);
    }

    if (settings.motionBlur) {
        token << "_MB" << std::max<std::uint32_t>(1U, settings.motionBlurSampleCount)
              << "_" << static_cast<long long>(std::llround(settings.motionBlurShutterAngleDegrees))
              << "deg";
    }

    return token.str();
}

std::string BuildAnimationExportFilenameStem(
    std::string_view animationName,
    AnimationExportMode mode,
    const RenderJobSettings& settings,
    std::string_view visualName) {
    auto stem = SanitizeFileStem(animationName, "Animation");
    stem += "_";
    stem += AnimationExportModeFilenameToken(mode);
    stem += "_";
    stem += AnimationExportSettingsFilenameToken(settings, mode);

    const auto safeVisualName = SanitizeFileStem(visualName, "");
    if (!safeVisualName.empty()) {
        stem += "_";
        stem += safeVisualName;
    }
    return stem;
}

std::filesystem::path BuildUniqueAnimationExportMediaOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    AnimationExportMode mode,
    const RenderJobSettings& settings,
    std::string_view extension,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    const auto fullStem = BuildAnimationExportFilenameStem(animationName, mode, settings, visualName);
    std::string safeExtension{extension};
    if (safeExtension.empty()) {
        safeExtension = ".mov";
    }
    if (safeExtension.front() != '.') {
        safeExtension.insert(safeExtension.begin(), '.');
    }

    auto candidate = outputDirectory / (fullStem + safeExtension);
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
        candidate = outputDirectory / (fullStem + "_" + std::to_string(suffix) + safeExtension);
    }
    return candidate;
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
    return BuildUniqueVideoOutputPath(outputDirectory, animationName, visualName, {}, ".mp4", reservedPaths);
}

std::filesystem::path BuildUniqueQuickMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::FastPreviewMp4,
        settings,
        ".mp4",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueVideoOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    std::string_view formatSuffix,
    std::string_view extension,
    const std::vector<std::filesystem::path>& reservedPaths) {
    const auto baseStem =
        SanitizeFileStem(animationName, "Animation") + "_" + SanitizeFileStem(visualName, "Visual");
    const auto safeSuffix = SanitizeFileStem(formatSuffix, "");
    std::string fullStem = baseStem;
    if (!safeSuffix.empty()) {
        fullStem += "_" + safeSuffix;
    }
    std::string safeExtension{extension};
    if (safeExtension.empty()) {
        safeExtension = ".mov";
    }
    if (safeExtension.front() != '.') {
        safeExtension.insert(safeExtension.begin(), '.');
    }

    auto candidate = outputDirectory / (fullStem + safeExtension);
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
        candidate = outputDirectory / (fullStem + "_" + std::to_string(suffix) + safeExtension);
    }
    return candidate;
}

std::filesystem::path BuildUniqueProRes422OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes422Mov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes422HqOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes422HqMov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes422VideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes422VideoToolboxMov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes422HqVideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes422HqVideoToolboxMov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueVideoOutputPath(
        outputDirectory,
        animationName,
        visualName,
        "ProRes4444",
        ".mov",
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes4444Mov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444XqOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueVideoOutputPath(
        outputDirectory,
        animationName,
        visualName,
        "ProRes4444XQ",
        ".mov",
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444XqOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes4444XqMov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444VideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueVideoOutputPath(
        outputDirectory,
        animationName,
        visualName,
        "ProRes4444VT",
        ".mov",
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444VideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes4444VideoToolboxMov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444XqVideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueVideoOutputPath(
        outputDirectory,
        animationName,
        visualName,
        "ProRes4444XQVT",
        ".mov",
        reservedPaths);
}

std::filesystem::path BuildUniqueProRes4444XqVideoToolboxOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueAnimationExportMediaOutputPath(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes4444XqVideoToolboxMov,
        settings,
        ".mov",
        visualName,
        reservedPaths);
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

std::string BuildFfmpegProRes422Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    std::uint32_t profile,
    bool videoToolbox) {
    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt rgb48le"
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -an";
    if (videoToolbox) {
        command << " -vf format=p210le"
                << " -c:v prores_videotoolbox"
                << " -profile:v " << std::clamp<std::uint32_t>(profile, 2U, 3U)
                << " -pix_fmt p210le"
                << " -allow_sw 1";
    } else {
        command << " -c:v prores_ks"
                << " -profile:v " << std::clamp<std::uint32_t>(profile, 2U, 3U)
                << " -pix_fmt yuv422p10le";
    }
    command << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(outputPath.string());
    return command.str();
}

std::string BuildFfmpegProRes422Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProRes422Command(executablePath, width, height, framesPerSecond, outputPath, 2U, false);
}

std::string BuildFfmpegProRes422HqCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProRes422Command(executablePath, width, height, framesPerSecond, outputPath, 3U, false);
}

std::string BuildFfmpegProRes422VideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProRes422Command(executablePath, width, height, framesPerSecond, outputPath, 2U, true);
}

std::string BuildFfmpegProRes422HqVideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProRes422Command(executablePath, width, height, framesPerSecond, outputPath, 3U, true);
}

std::string BuildFfmpegProResCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    std::uint32_t profile,
    bool videoToolbox) {
    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt rgba64le"
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -an";
    if (videoToolbox) {
        command << " -vf format=ayuv64le"
                << " -c:v prores_videotoolbox"
                << " -profile:v " << std::clamp<std::uint32_t>(profile, 4U, 5U)
                << " -pix_fmt ayuv64le"
                << " -allow_sw 1";
    } else {
        command << " -c:v prores_ks"
                << " -profile:v " << std::clamp<std::uint32_t>(profile, 4U, 5U)
                << " -pix_fmt yuva444p10le"
                << " -alpha_bits 16";
    }
    command << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(outputPath.string());
    return command.str();
}

std::string BuildFfmpegProRes4444Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProResCommand(executablePath, width, height, framesPerSecond, outputPath, 4U, false);
}

std::string BuildFfmpegProRes4444XqCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProResCommand(executablePath, width, height, framesPerSecond, outputPath, 5U, false);
}

std::string BuildFfmpegProRes4444VideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProResCommand(executablePath, width, height, framesPerSecond, outputPath, 4U, true);
}

std::string BuildFfmpegProRes4444XqVideoToolboxCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegProResCommand(executablePath, width, height, framesPerSecond, outputPath, 5U, true);
}

HalfRgbaExrImage AverageHalfRgbaFrames(const std::vector<HalfRgbaExrImage>& images) {
    if (images.empty()) {
        return {};
    }
    const auto width = images.front().width;
    const auto height = images.front().height;
    const auto pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (width == 0 || height == 0 || images.front().rgbaHalf.size() != pixelCount * 4U) {
        return {};
    }
    if (images.size() == 1U) {
        return images.front();
    }

    std::vector<float> premultipliedRed(pixelCount, 0.0F);
    std::vector<float> premultipliedGreen(pixelCount, 0.0F);
    std::vector<float> premultipliedBlue(pixelCount, 0.0F);
    std::vector<float> alpha(pixelCount, 0.0F);
    std::vector<float> depth(pixelCount, 0.0F);
    std::vector<float> depthWeight(pixelCount, 0.0F);
    std::uint32_t validImageCount = 0;

    for (const auto& image : images) {
        if (image.width != width ||
            image.height != height ||
            image.rgbaHalf.size() != pixelCount * 4U) {
            return {};
        }
        ++validImageCount;
        const bool hasDepth = image.depth.size() == pixelCount;
        for (std::size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
            const auto componentOffset = pixelIndex * 4U;
            const float sampleAlpha =
                std::clamp(HalfBitsToFloat(image.rgbaHalf[componentOffset + 3U]), 0.0F, 1.0F);
            premultipliedRed[pixelIndex] += HalfBitsToFloat(image.rgbaHalf[componentOffset + 0U]) * sampleAlpha;
            premultipliedGreen[pixelIndex] += HalfBitsToFloat(image.rgbaHalf[componentOffset + 1U]) * sampleAlpha;
            premultipliedBlue[pixelIndex] += HalfBitsToFloat(image.rgbaHalf[componentOffset + 2U]) * sampleAlpha;
            alpha[pixelIndex] += sampleAlpha;
            if (hasDepth && ValidDepth(image.depth[pixelIndex])) {
                const float weight = std::max(sampleAlpha, 0.001F);
                depth[pixelIndex] += image.depth[pixelIndex] * weight;
                depthWeight[pixelIndex] += weight;
            }
        }
    }

    if (validImageCount == 0U) {
        return {};
    }

    HalfRgbaExrImage averaged;
    averaged.width = width;
    averaged.height = height;
    averaged.rgbaHalf.resize(pixelCount * 4U);
    averaged.depth.resize(pixelCount, 0.0F);
    const float inverseCount = 1.0F / static_cast<float>(validImageCount);
    for (std::size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
        const float averagedAlpha = std::clamp(alpha[pixelIndex] * inverseCount, 0.0F, 1.0F);
        float red = 0.0F;
        float green = 0.0F;
        float blue = 0.0F;
        if (alpha[pixelIndex] > 1.0e-6F) {
            red = premultipliedRed[pixelIndex] / alpha[pixelIndex];
            green = premultipliedGreen[pixelIndex] / alpha[pixelIndex];
            blue = premultipliedBlue[pixelIndex] / alpha[pixelIndex];
        }
        const auto componentOffset = pixelIndex * 4U;
        averaged.rgbaHalf[componentOffset + 0U] = FloatToHalfBits(std::max(0.0F, red));
        averaged.rgbaHalf[componentOffset + 1U] = FloatToHalfBits(std::max(0.0F, green));
        averaged.rgbaHalf[componentOffset + 2U] = FloatToHalfBits(std::max(0.0F, blue));
        averaged.rgbaHalf[componentOffset + 3U] = FloatToHalfBits(averagedAlpha);
        averaged.depth[pixelIndex] =
            depthWeight[pixelIndex] > 1.0e-6F ? depth[pixelIndex] / depthWeight[pixelIndex] : 0.0F;
    }
    return averaged;
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
    const Mp4SparsePointSmoothingSettings& smoothing,
    bool spatialAntialiasing) {
    const auto pixelCount = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    if (image.width == 0 ||
        image.height == 0 ||
        outputWidth == 0 ||
        outputHeight == 0 ||
        image.rgbaHalf.size() != pixelCount * 4U) {
        return {};
    }

    std::vector<LinearRgbaPixel> pixels = HalfRgbaToLinearPixels(image);

    ApplySparsePointSmoothing(&pixels, image.width, image.height, smoothing);
    pixels = ResampleLinearRgba(
        pixels,
        image.width,
        image.height,
        outputWidth,
        outputHeight,
        spatialAntialiasing);
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

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba16(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    bool spatialAntialiasing) {
    if (image.width == 0 ||
        image.height == 0 ||
        outputWidth == 0 ||
        outputHeight == 0) {
        return {};
    }

    std::vector<LinearRgbaPixel> pixels = HalfRgbaToLinearPixels(image);
    pixels = ResampleLinearRgba(
        pixels,
        image.width,
        image.height,
        outputWidth,
        outputHeight,
        spatialAntialiasing);
    if (pixels.empty()) {
        return {};
    }

    std::vector<std::uint8_t> bytes;
    const auto outputPixelCount =
        static_cast<std::size_t>(outputWidth) * static_cast<std::size_t>(outputHeight);
    bytes.resize(outputPixelCount * 8U);
    auto writeWord = [&bytes](std::size_t offset, std::uint16_t value) {
        bytes[offset + 0U] = static_cast<std::uint8_t>(value & 0xFFU);
        bytes[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    };
    for (std::size_t pixelIndex = 0; pixelIndex < outputPixelCount; ++pixelIndex) {
        const std::size_t destinationOffset = pixelIndex * 8U;
        writeWord(destinationOffset + 0U, UnitFloatToWord(LinearToSrgb(pixels[pixelIndex].r)));
        writeWord(destinationOffset + 2U, UnitFloatToWord(LinearToSrgb(pixels[pixelIndex].g)));
        writeWord(destinationOffset + 4U, UnitFloatToWord(LinearToSrgb(pixels[pixelIndex].b)));
        writeWord(destinationOffset + 6U, UnitFloatToWord(pixels[pixelIndex].a));
    }
    return bytes;
}

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgb16OpaqueBlack(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    bool spatialAntialiasing) {
    if (image.width == 0 ||
        image.height == 0 ||
        outputWidth == 0 ||
        outputHeight == 0) {
        return {};
    }

    std::vector<LinearRgbaPixel> pixels = HalfRgbaToLinearPixels(image);
    pixels = ResampleLinearRgba(
        pixels,
        image.width,
        image.height,
        outputWidth,
        outputHeight,
        spatialAntialiasing);
    if (pixels.empty()) {
        return {};
    }

    std::vector<std::uint8_t> bytes;
    const auto outputPixelCount =
        static_cast<std::size_t>(outputWidth) * static_cast<std::size_t>(outputHeight);
    bytes.resize(outputPixelCount * 6U);
    auto writeWord = [&bytes](std::size_t offset, std::uint16_t value) {
        bytes[offset + 0U] = static_cast<std::uint8_t>(value & 0xFFU);
        bytes[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    };
    for (std::size_t pixelIndex = 0; pixelIndex < outputPixelCount; ++pixelIndex) {
        const auto alpha = std::clamp(pixels[pixelIndex].a, 0.0F, 1.0F);
        const std::size_t destinationOffset = pixelIndex * 6U;
        writeWord(destinationOffset + 0U, UnitFloatToWord(LinearToSrgb(pixels[pixelIndex].r * alpha)));
        writeWord(destinationOffset + 2U, UnitFloatToWord(LinearToSrgb(pixels[pixelIndex].g * alpha)));
        writeWord(destinationOffset + 4U, UnitFloatToWord(LinearToSrgb(pixels[pixelIndex].b * alpha)));
    }
    return bytes;
}

}  // namespace invisible_places::output
