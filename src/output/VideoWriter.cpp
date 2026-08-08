#include "output/VideoWriter.hpp"

#include <Imath/half.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string_view>
#include <thread>
#include <type_traits>

#if defined(__APPLE__)
#include <Accelerate/Accelerate.h>
#include <dispatch/dispatch.h>
#include <sys/sysctl.h>
#endif

namespace invisible_places::output {

namespace {

struct LinearRgbaPixel {
    float r = 0.0F;
    float g = 0.0F;
    float b = 0.0F;
    float a = 0.0F;
    float depth = 0.0F;
};

std::size_t PreferredImageWorkerCount(std::uint32_t rowCount) {
    if (rowCount == 0U) {
        return 0U;
    }

    static const std::size_t availableWorkers = []() {
#if defined(__APPLE__)
        std::uint32_t performanceLogicalCores = 0U;
        std::size_t valueSize = sizeof(performanceLogicalCores);
        if (::sysctlbyname(
                "hw.perflevel0.logicalcpu",
                &performanceLogicalCores,
                &valueSize,
                nullptr,
                0) == 0 &&
            performanceLogicalCores > 0U) {
            return static_cast<std::size_t>(performanceLogicalCores);
        }
#endif
        return std::max<std::size_t>(1U, std::thread::hardware_concurrency());
    }();
    return std::min<std::size_t>(availableWorkers, rowCount);
}

template <typename Function>
void ParallelForRows(std::uint32_t rowCount, Function&& function) {
    const auto taskCount = PreferredImageWorkerCount(rowCount);
    if (taskCount <= 1U || rowCount < 32U) {
        function(0U, rowCount);
        return;
    }

    using Callable = std::remove_reference_t<Function>;
    struct Context {
        Callable* callable = nullptr;
        std::uint32_t rows = 0U;
        std::size_t tasks = 0U;
    };
    Context context{.callable = &function, .rows = rowCount, .tasks = taskCount};
    auto runTask = [](void* rawContext, std::size_t taskIndex) {
        auto* taskContext = static_cast<Context*>(rawContext);
        const auto begin = static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(taskIndex) * taskContext->rows) / taskContext->tasks);
        const auto end = static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(taskIndex + 1U) * taskContext->rows) / taskContext->tasks);
        (*taskContext->callable)(begin, end);
    };

#if defined(__APPLE__)
    // User-initiated GCD work favors performance cores without pinning threads.
    ::dispatch_apply_f(
        taskCount,
        ::dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0),
        &context,
        runTask);
#else
    std::vector<std::jthread> workers;
    workers.reserve(taskCount);
    for (std::size_t taskIndex = 0; taskIndex < taskCount; ++taskIndex) {
        workers.emplace_back([&context, runTask, taskIndex]() {
            runTask(&context, taskIndex);
        });
    }
#endif
}

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

struct SimpleAlphaMatteOutputPaths {
    std::filesystem::path colorPath;
    std::filesystem::path alphaMattePath;
};

SimpleAlphaMatteOutputPaths BuildUniqueSimpleAlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view extension,
    const std::vector<std::filesystem::path>& reservedPaths) {
    const auto directory =
        outputDirectory.empty() ? std::filesystem::path{"."} : outputDirectory;
    const auto baseStem = SanitizeFileStem(animationName, "Animation");
    const auto reserved = [&reservedPaths](const std::filesystem::path& path) {
        const auto normalized = path.lexically_normal();
        return std::any_of(
            reservedPaths.begin(),
            reservedPaths.end(),
            [&normalized](const std::filesystem::path& reservedPath) {
                return reservedPath.lexically_normal() == normalized;
            });
    };
    const auto makePaths = [&](std::uint32_t index) {
        const auto suffix =
            index > 1U ? "_" + std::to_string(index) : std::string{};
        return SimpleAlphaMatteOutputPaths{
            .colorPath = directory /
                         (baseStem + "_Colour" + suffix + std::string{extension}),
            .alphaMattePath = directory /
                              (baseStem + "_Alpha" + suffix + std::string{extension}),
        };
    };

    auto candidate = makePaths(1U);
    for (std::uint32_t index = 2U;
         std::filesystem::exists(candidate.colorPath) ||
         std::filesystem::exists(candidate.alphaMattePath) ||
         reserved(candidate.colorPath) || reserved(candidate.alphaMattePath);
         ++index) {
        candidate = makePaths(index);
    }
    return candidate;
}

bool IsProRes422Mode(AnimationExportMode mode) {
    return mode == AnimationExportMode::ProRes422Mov ||
           mode == AnimationExportMode::ProRes422HqMov ||
           mode == AnimationExportMode::ProRes422AlphaMatteMov ||
           mode == AnimationExportMode::ProRes422HqAlphaMatteMov ||
           mode == AnimationExportMode::ProRes422VideoToolboxMov ||
           mode == AnimationExportMode::ProRes422HqVideoToolboxMov;
}

bool IsProRes4444Mode(AnimationExportMode mode) {
    return mode == AnimationExportMode::ProRes4444Mov ||
           mode == AnimationExportMode::ProRes4444XqMov ||
           mode == AnimationExportMode::ProRes4444VideoToolboxMov ||
           mode == AnimationExportMode::ProRes4444XqVideoToolboxMov;
}

AnimationExportMode CompactAnimationExportMode(AnimationExportMode mode) {
    if (mode == AnimationExportMode::HevcAlphaMp4) {
        return AnimationExportMode::FastPreviewMp4;
    }
    if (IsProRes422Mode(mode)) {
        return AnimationExportMode::ProRes422Mov;
    }
    if (IsProRes4444Mode(mode)) {
        return AnimationExportMode::ProRes4444Mov;
    }
    return mode;
}

AnimationExportQuality LegacyQualityForMode(AnimationExportMode mode) {
    switch (mode) {
        case AnimationExportMode::HevcAlphaMp4:
        case AnimationExportMode::ProRes422HqMov:
        case AnimationExportMode::ProRes422HqAlphaMatteMov:
        case AnimationExportMode::ProRes422HqVideoToolboxMov:
            return AnimationExportQuality::Hq;
        case AnimationExportMode::ProRes4444XqMov:
        case AnimationExportMode::ProRes4444XqVideoToolboxMov:
            return AnimationExportQuality::Xq;
        case AnimationExportMode::FastPreviewMp4:
        case AnimationExportMode::TestMp4:
        case AnimationExportMode::PngStack:
        case AnimationExportMode::FastPngStack:
        case AnimationExportMode::HqPreviewDensityExr:
        case AnimationExportMode::ProRes422Mov:
        case AnimationExportMode::ProRes422AlphaMatteMov:
        case AnimationExportMode::ProRes422VideoToolboxMov:
        case AnimationExportMode::ProRes4444Mov:
        case AnimationExportMode::ProRes4444VideoToolboxMov:
            return AnimationExportQuality::Normal;
    }
    return AnimationExportQuality::Normal;
}

bool LegacyUsesVideoToolbox(AnimationExportMode mode) {
    return mode == AnimationExportMode::FastPreviewMp4 ||
           mode == AnimationExportMode::TestMp4 ||
           mode == AnimationExportMode::HevcAlphaMp4 ||
           mode == AnimationExportMode::ProRes422VideoToolboxMov ||
           mode == AnimationExportMode::ProRes422HqVideoToolboxMov ||
           mode == AnimationExportMode::ProRes4444VideoToolboxMov ||
           mode == AnimationExportMode::ProRes4444XqVideoToolboxMov;
}

bool LegacyWritesExternalAlphaMatte(AnimationExportMode mode) {
    return mode == AnimationExportMode::TestMp4 ||
           mode == AnimationExportMode::HevcAlphaMp4 ||
           mode == AnimationExportMode::ProRes422AlphaMatteMov ||
           mode == AnimationExportMode::ProRes422HqAlphaMatteMov;
}

bool ValidDepth(float depth) {
    return std::isfinite(depth) && depth > 0.0F;
}

std::vector<LinearRgbaPixel> DownsampleLinearRgba(const std::vector<LinearRgbaPixel>& source,
                                                  std::uint32_t sourceWidth,
                                                  std::uint32_t sourceHeight,
                                                  std::uint32_t outputWidth,
                                                  std::uint32_t outputHeight) {
    const auto sourcePixelCount = static_cast<std::size_t>(sourceWidth) * static_cast<std::size_t>(sourceHeight);
    if (sourceWidth == 0 || sourceHeight == 0 || outputWidth == 0 || outputHeight == 0 ||
        source.size() != sourcePixelCount) {
        return {};
    }
    if (sourceWidth == outputWidth && sourceHeight == outputHeight) {
        return source;
    }

    std::vector<LinearRgbaPixel> output(static_cast<std::size_t>(outputWidth) * static_cast<std::size_t>(outputHeight));
    const double scaleX = static_cast<double>(sourceWidth) / static_cast<double>(outputWidth);
    const double scaleY = static_cast<double>(sourceHeight) / static_cast<double>(outputHeight);

    ParallelForRows(outputHeight, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            const double sourceY0 = static_cast<double>(y) * scaleY;
            const double sourceY1 = static_cast<double>(y + 1U) * scaleY;
            const auto firstY = static_cast<std::uint32_t>(
                std::clamp(std::floor(sourceY0), 0.0, static_cast<double>(sourceHeight - 1U)));
            const auto lastY =
                static_cast<std::uint32_t>(std::clamp(std::ceil(sourceY1), 1.0, static_cast<double>(sourceHeight)));

            for (std::uint32_t x = 0; x < outputWidth; ++x) {
                const double sourceX0 = static_cast<double>(x) * scaleX;
                const double sourceX1 = static_cast<double>(x + 1U) * scaleX;
                const auto firstX = static_cast<std::uint32_t>(
                    std::clamp(std::floor(sourceX0), 0.0, static_cast<double>(sourceWidth - 1U)));
                const auto lastX =
                    static_cast<std::uint32_t>(std::clamp(std::ceil(sourceX1), 1.0, static_cast<double>(sourceWidth)));

                float premultipliedRed = 0.0F;
                float premultipliedGreen = 0.0F;
                float premultipliedBlue = 0.0F;
                float alphaSum = 0.0F;
                float depthSum = 0.0F;
                float depthWeight = 0.0F;
                float sampleCount = 0.0F;

                for (std::uint32_t sampleY = firstY; sampleY < lastY; ++sampleY) {
                    for (std::uint32_t sampleX = firstX; sampleX < lastX; ++sampleX) {
                        const auto& sample = source[static_cast<std::size_t>(sampleY) * sourceWidth + sampleX];
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
    });

    return output;
}

std::vector<LinearRgbaPixel> DownsampleLinearRgbaNearest(const std::vector<LinearRgbaPixel>& source,
                                                         std::uint32_t sourceWidth,
                                                         std::uint32_t sourceHeight,
                                                         std::uint32_t outputWidth,
                                                         std::uint32_t outputHeight) {
    const auto sourcePixelCount = static_cast<std::size_t>(sourceWidth) * static_cast<std::size_t>(sourceHeight);
    if (sourceWidth == 0 || sourceHeight == 0 || outputWidth == 0 || outputHeight == 0 ||
        source.size() != sourcePixelCount) {
        return {};
    }
    if (sourceWidth == outputWidth && sourceHeight == outputHeight) {
        return source;
    }

    std::vector<LinearRgbaPixel> output(static_cast<std::size_t>(outputWidth) * static_cast<std::size_t>(outputHeight));
    ParallelForRows(outputHeight, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            const auto sourceY = std::min<std::uint32_t>(
                sourceHeight - 1U,
                static_cast<std::uint32_t>((static_cast<std::uint64_t>(y) * sourceHeight) / outputHeight));
            for (std::uint32_t x = 0; x < outputWidth; ++x) {
                const auto sourceX = std::min<std::uint32_t>(
                    sourceWidth - 1U,
                    static_cast<std::uint32_t>((static_cast<std::uint64_t>(x) * sourceWidth) / outputWidth));
                output[static_cast<std::size_t>(y) * outputWidth + x] =
                    source[static_cast<std::size_t>(sourceY) * sourceWidth + sourceX];
            }
        }
    });
    return output;
}

std::vector<LinearRgbaPixel> ResampleLinearRgba(const std::vector<LinearRgbaPixel>& source,
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
    if (image.width == 0 || image.height == 0 || image.rgbaHalf.size() != pixelCount * 4U) {
        return {};
    }

    std::vector<LinearRgbaPixel> pixels(pixelCount);
    const bool hasDepth = image.depth.size() == pixelCount;
    ParallelForRows(image.height, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            const auto rowOffset = static_cast<std::size_t>(y) * image.width;
            for (std::uint32_t x = 0; x < image.width; ++x) {
                const auto pixelIndex = rowOffset + x;
                const std::size_t sourceOffset = pixelIndex * 4U;
                pixels[pixelIndex] = {
                    .r = HalfBitsToFloat(image.rgbaHalf[sourceOffset + 0U]),
                    .g = HalfBitsToFloat(image.rgbaHalf[sourceOffset + 1U]),
                    .b = HalfBitsToFloat(image.rgbaHalf[sourceOffset + 2U]),
                    .a = std::clamp(HalfBitsToFloat(image.rgbaHalf[sourceOffset + 3U]), 0.0F, 1.0F),
                    .depth = hasDepth ? image.depth[pixelIndex] : 0.0F,
                };
            }
        }
    });
    return pixels;
}

void WriteLittleEndianWord(std::uint8_t* destination, std::uint16_t value) {
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

struct Rgba8OutputWriter {
    static constexpr std::size_t kBytesPerPixel = 4U;

    void operator()(std::uint8_t* destination, float red, float green, float blue, float alpha) const {
        destination[0] = UnitFloatToByte(LinearToSrgb(red));
        destination[1] = UnitFloatToByte(LinearToSrgb(green));
        destination[2] = UnitFloatToByte(LinearToSrgb(blue));
        destination[3] = UnitFloatToByte(alpha);
    }
};

struct Rgba16OutputWriter {
    static constexpr std::size_t kBytesPerPixel = 8U;

    void operator()(std::uint8_t* destination, float red, float green, float blue, float alpha) const {
        WriteLittleEndianWord(destination + 0U, UnitFloatToWord(LinearToSrgb(red)));
        WriteLittleEndianWord(destination + 2U, UnitFloatToWord(LinearToSrgb(green)));
        WriteLittleEndianWord(destination + 4U, UnitFloatToWord(LinearToSrgb(blue)));
        WriteLittleEndianWord(destination + 6U, UnitFloatToWord(alpha));
    }
};

struct OpaqueBlackRgb16OutputWriter {
    static constexpr std::size_t kBytesPerPixel = 6U;

    void operator()(std::uint8_t* destination, float red, float green, float blue, float alpha) const {
        // Match After Effects' default display-referred luma-matte workflow:
        // encode the straight RGB first, then apply the matte over black.
        // Applying alpha in linear light before this transfer makes partially
        // transparent point splats substantially brighter than the AE result.
        WriteLittleEndianWord(destination + 0U, UnitFloatToWord(LinearToSrgb(red) * alpha));
        WriteLittleEndianWord(destination + 2U, UnitFloatToWord(LinearToSrgb(green) * alpha));
        WriteLittleEndianWord(destination + 4U, UnitFloatToWord(LinearToSrgb(blue) * alpha));
    }
};

// The rgba-layout twins of OpaqueBlackRgb16OutputWriter (same
// display-referred matte-over-black, see its note) for encoders whose raw
// input stays rgba/rgba64le: the matte is pre-applied and alpha forced
// opaque so the existing ffmpeg commands need no format change.
struct OpaqueBlackRgba8OutputWriter {
    static constexpr std::size_t kBytesPerPixel = 4U;

    void operator()(std::uint8_t* destination, float red, float green, float blue, float alpha) const {
        destination[0] = UnitFloatToByte(LinearToSrgb(red) * alpha);
        destination[1] = UnitFloatToByte(LinearToSrgb(green) * alpha);
        destination[2] = UnitFloatToByte(LinearToSrgb(blue) * alpha);
        destination[3] = 0xFFU;
    }
};

struct OpaqueBlackRgba16OutputWriter {
    static constexpr std::size_t kBytesPerPixel = 8U;

    void operator()(std::uint8_t* destination, float red, float green, float blue, float alpha) const {
        WriteLittleEndianWord(destination + 0U, UnitFloatToWord(LinearToSrgb(red) * alpha));
        WriteLittleEndianWord(destination + 2U, UnitFloatToWord(LinearToSrgb(green) * alpha));
        WriteLittleEndianWord(destination + 4U, UnitFloatToWord(LinearToSrgb(blue) * alpha));
        WriteLittleEndianWord(destination + 6U, 0xFFFFU);
    }
};

bool IsExactTwoTimesResolve(
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight) {
    return static_cast<std::uint64_t>(outputWidth) * 2U == sourceWidth &&
           static_cast<std::uint64_t>(outputHeight) * 2U == sourceHeight;
}

template <typename Writer>
std::vector<std::uint8_t> ResolveLinearRgbaTwoTimes(
    const std::vector<LinearRgbaPixel>& source,
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Writer& writer) {
    if (!IsExactTwoTimesResolve(sourceWidth, sourceHeight, outputWidth, outputHeight) ||
        source.size() != static_cast<std::size_t>(sourceWidth) * sourceHeight) {
        return {};
    }

    std::vector<std::uint8_t> bytes(
        static_cast<std::size_t>(outputWidth) * outputHeight * Writer::kBytesPerPixel);
    ParallelForRows(outputHeight, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            for (std::uint32_t x = 0; x < outputWidth; ++x) {
                float premultipliedRed = 0.0F;
                float premultipliedGreen = 0.0F;
                float premultipliedBlue = 0.0F;
                float alphaSum = 0.0F;
                for (std::uint32_t offsetY = 0; offsetY < 2U; ++offsetY) {
                    for (std::uint32_t offsetX = 0; offsetX < 2U; ++offsetX) {
                        const auto& sample = source[
                            static_cast<std::size_t>((y * 2U) + offsetY) * sourceWidth +
                            ((x * 2U) + offsetX)];
                        premultipliedRed += sample.r * sample.a;
                        premultipliedGreen += sample.g * sample.a;
                        premultipliedBlue += sample.b * sample.a;
                        alphaSum += sample.a;
                    }
                }
                const float red = alphaSum > 1.0e-6F ? premultipliedRed / alphaSum : 0.0F;
                const float green = alphaSum > 1.0e-6F ? premultipliedGreen / alphaSum : 0.0F;
                const float blue = alphaSum > 1.0e-6F ? premultipliedBlue / alphaSum : 0.0F;
                const float alpha = std::clamp(alphaSum * 0.25F, 0.0F, 1.0F);
                const auto outputOffset =
                    (static_cast<std::size_t>(y) * outputWidth + x) * Writer::kBytesPerPixel;
                writer(bytes.data() + outputOffset, red, green, blue, alpha);
            }
        }
    });
    return bytes;
}

template <typename Writer>
std::vector<std::uint8_t> ResolveHalfRgbaTwoTimes(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Writer& writer) {
    const auto sourcePixelCount = static_cast<std::size_t>(image.width) * image.height;
    if (!IsExactTwoTimesResolve(image.width, image.height, outputWidth, outputHeight) ||
        image.rgbaHalf.size() != sourcePixelCount * 4U) {
        return {};
    }

    // Fuse alpha-aware 2x box filtering, unpremultiplication, transfer, and packing.
    std::vector<std::uint8_t> bytes(
        static_cast<std::size_t>(outputWidth) * outputHeight * Writer::kBytesPerPixel);
    ParallelForRows(outputHeight, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            for (std::uint32_t x = 0; x < outputWidth; ++x) {
                float premultipliedRed = 0.0F;
                float premultipliedGreen = 0.0F;
                float premultipliedBlue = 0.0F;
                float alphaSum = 0.0F;
                for (std::uint32_t offsetY = 0; offsetY < 2U; ++offsetY) {
                    for (std::uint32_t offsetX = 0; offsetX < 2U; ++offsetX) {
                        const auto sourcePixelIndex =
                            static_cast<std::size_t>((y * 2U) + offsetY) * image.width +
                            ((x * 2U) + offsetX);
                        const auto sourceOffset = sourcePixelIndex * 4U;
                        const float alpha = std::clamp(
                            HalfBitsToFloat(image.rgbaHalf[sourceOffset + 3U]), 0.0F, 1.0F);
                        premultipliedRed += HalfBitsToFloat(image.rgbaHalf[sourceOffset + 0U]) * alpha;
                        premultipliedGreen += HalfBitsToFloat(image.rgbaHalf[sourceOffset + 1U]) * alpha;
                        premultipliedBlue += HalfBitsToFloat(image.rgbaHalf[sourceOffset + 2U]) * alpha;
                        alphaSum += alpha;
                    }
                }
                const float red = alphaSum > 1.0e-6F ? premultipliedRed / alphaSum : 0.0F;
                const float green = alphaSum > 1.0e-6F ? premultipliedGreen / alphaSum : 0.0F;
                const float blue = alphaSum > 1.0e-6F ? premultipliedBlue / alphaSum : 0.0F;
                const float alpha = std::clamp(alphaSum * 0.25F, 0.0F, 1.0F);
                const auto outputOffset =
                    (static_cast<std::size_t>(y) * outputWidth + x) * Writer::kBytesPerPixel;
                writer(bytes.data() + outputOffset, red, green, blue, alpha);
            }
        }
    });
    return bytes;
}

template <typename Writer>
std::vector<std::uint8_t> WriteLinearRgbaPixels(
    const std::vector<LinearRgbaPixel>& pixels,
    std::uint32_t width,
    std::uint32_t height,
    const Writer& writer) {
    if (pixels.size() != static_cast<std::size_t>(width) * height) {
        return {};
    }
    std::vector<std::uint8_t> bytes(pixels.size() * Writer::kBytesPerPixel);
    ParallelForRows(height, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            for (std::uint32_t x = 0; x < width; ++x) {
                const auto pixelIndex = static_cast<std::size_t>(y) * width + x;
                const auto destinationOffset = pixelIndex * Writer::kBytesPerPixel;
                const auto& pixel = pixels[pixelIndex];
                writer(bytes.data() + destinationOffset, pixel.r, pixel.g, pixel.b, pixel.a);
            }
        }
    });
    return bytes;
}

#if defined(__APPLE__)
template <typename Writer>
std::optional<std::vector<std::uint8_t>> ResolveHalfRgbaWithVImage(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Writer& writer) {
    const auto sourcePixelCount = static_cast<std::size_t>(image.width) * image.height;
    if (image.rgbaHalf.size() != sourcePixelCount * 4U) {
        return std::nullopt;
    }

    // vImage handles uncommon scale ratios; premultiplication prevents transparent RGB fringes.
    auto premultipliedHalf = image.rgbaHalf;
    ParallelForRows(image.height, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            const auto rowOffset = static_cast<std::size_t>(y) * image.width;
            for (std::uint32_t x = 0; x < image.width; ++x) {
                const auto alphaOffset = ((rowOffset + x) * 4U) + 3U;
                premultipliedHalf[alphaOffset] = FloatToHalfBits(std::clamp(
                    HalfBitsToFloat(premultipliedHalf[alphaOffset]), 0.0F, 1.0F));
            }
        }
    });
    vImage_Buffer sourceBuffer{
        .data = premultipliedHalf.data(),
        .height = image.height,
        .width = image.width,
        .rowBytes = static_cast<std::size_t>(image.width) * 4U * sizeof(std::uint16_t),
    };
    if (vImagePremultiplyData_RGBA16F(&sourceBuffer, &sourceBuffer, kvImageNoFlags) != kvImageNoError) {
        return std::nullopt;
    }

    std::vector<std::uint16_t> scaledHalf(
        static_cast<std::size_t>(outputWidth) * outputHeight * 4U);
    vImage_Buffer outputBuffer{
        .data = scaledHalf.data(),
        .height = outputHeight,
        .width = outputWidth,
        .rowBytes = static_cast<std::size_t>(outputWidth) * 4U * sizeof(std::uint16_t),
    };
    if (vImageScale_ARGB16F(
            &sourceBuffer,
            &outputBuffer,
            nullptr,
            kvImageHighQualityResampling) != kvImageNoError) {
        return std::nullopt;
    }

    std::vector<std::uint8_t> bytes(
        static_cast<std::size_t>(outputWidth) * outputHeight * Writer::kBytesPerPixel);
    ParallelForRows(outputHeight, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            for (std::uint32_t x = 0; x < outputWidth; ++x) {
                const auto pixelIndex = static_cast<std::size_t>(y) * outputWidth + x;
                const auto sourceOffset = pixelIndex * 4U;
                const float alpha = std::clamp(
                    HalfBitsToFloat(scaledHalf[sourceOffset + 3U]), 0.0F, 1.0F);
                const float inverseAlpha = alpha > 1.0e-6F ? 1.0F / alpha : 0.0F;
                writer(
                    bytes.data() + (pixelIndex * Writer::kBytesPerPixel),
                    HalfBitsToFloat(scaledHalf[sourceOffset + 0U]) * inverseAlpha,
                    HalfBitsToFloat(scaledHalf[sourceOffset + 1U]) * inverseAlpha,
                    HalfBitsToFloat(scaledHalf[sourceOffset + 2U]) * inverseAlpha,
                    alpha);
            }
        }
    });
    return bytes;
}

template <typename Writer>
std::optional<std::vector<std::uint8_t>> ResolveLinearRgbaWithVImage(
    const std::vector<LinearRgbaPixel>& source,
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Writer& writer) {
    if (source.size() != static_cast<std::size_t>(sourceWidth) * sourceHeight) {
        return std::nullopt;
    }

    std::vector<float> premultiplied(source.size() * 4U);
    ParallelForRows(sourceHeight, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            for (std::uint32_t x = 0; x < sourceWidth; ++x) {
                const auto pixelIndex = static_cast<std::size_t>(y) * sourceWidth + x;
                const auto offset = pixelIndex * 4U;
                const float alpha = std::clamp(source[pixelIndex].a, 0.0F, 1.0F);
                premultiplied[offset + 0U] = source[pixelIndex].r * alpha;
                premultiplied[offset + 1U] = source[pixelIndex].g * alpha;
                premultiplied[offset + 2U] = source[pixelIndex].b * alpha;
                premultiplied[offset + 3U] = alpha;
            }
        }
    });
    vImage_Buffer sourceBuffer{
        .data = premultiplied.data(),
        .height = sourceHeight,
        .width = sourceWidth,
        .rowBytes = static_cast<std::size_t>(sourceWidth) * 4U * sizeof(float),
    };
    std::vector<float> scaled(static_cast<std::size_t>(outputWidth) * outputHeight * 4U);
    vImage_Buffer outputBuffer{
        .data = scaled.data(),
        .height = outputHeight,
        .width = outputWidth,
        .rowBytes = static_cast<std::size_t>(outputWidth) * 4U * sizeof(float),
    };
    if (vImageScale_ARGBFFFF(
            &sourceBuffer,
            &outputBuffer,
            nullptr,
            kvImageHighQualityResampling) != kvImageNoError) {
        return std::nullopt;
    }

    std::vector<std::uint8_t> bytes(
        static_cast<std::size_t>(outputWidth) * outputHeight * Writer::kBytesPerPixel);
    ParallelForRows(outputHeight, [&](std::uint32_t beginY, std::uint32_t endY) {
        for (std::uint32_t y = beginY; y < endY; ++y) {
            for (std::uint32_t x = 0; x < outputWidth; ++x) {
                const auto pixelIndex = static_cast<std::size_t>(y) * outputWidth + x;
                const auto offset = pixelIndex * 4U;
                const float alpha = std::clamp(scaled[offset + 3U], 0.0F, 1.0F);
                const float inverseAlpha = alpha > 1.0e-6F ? 1.0F / alpha : 0.0F;
                writer(
                    bytes.data() + (pixelIndex * Writer::kBytesPerPixel),
                    scaled[offset + 0U] * inverseAlpha,
                    scaled[offset + 1U] * inverseAlpha,
                    scaled[offset + 2U] * inverseAlpha,
                    alpha);
            }
        }
    });
    return bytes;
}
#endif

template <typename Writer>
std::vector<std::uint8_t> ResolveLinearRgba(
    const std::vector<LinearRgbaPixel>& source,
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    bool spatialAntialiasing,
    const Writer& writer) {
    if (sourceWidth == outputWidth && sourceHeight == outputHeight) {
        return WriteLinearRgbaPixels(source, outputWidth, outputHeight, writer);
    }
    if (spatialAntialiasing &&
        IsExactTwoTimesResolve(sourceWidth, sourceHeight, outputWidth, outputHeight)) {
        return ResolveLinearRgbaTwoTimes(
            source, sourceWidth, sourceHeight, outputWidth, outputHeight, writer);
    }
#if defined(__APPLE__)
    if (spatialAntialiasing) {
        if (auto resolved = ResolveLinearRgbaWithVImage(
                source,
                sourceWidth,
                sourceHeight,
                outputWidth,
                outputHeight,
                writer);
            resolved.has_value()) {
            return std::move(resolved.value());
        }
    }
#endif
    return WriteLinearRgbaPixels(
        ResampleLinearRgba(
            source,
            sourceWidth,
            sourceHeight,
            outputWidth,
            outputHeight,
            spatialAntialiasing),
        outputWidth,
        outputHeight,
        writer);
}

template <typename Writer>
std::vector<std::uint8_t> ResolveHalfRgba(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    bool spatialAntialiasing,
    const Writer& writer) {
    if (spatialAntialiasing &&
        IsExactTwoTimesResolve(image.width, image.height, outputWidth, outputHeight)) {
        return ResolveHalfRgbaTwoTimes(image, outputWidth, outputHeight, writer);
    }
#if defined(__APPLE__)
    if (spatialAntialiasing &&
        (image.width != outputWidth || image.height != outputHeight)) {
        if (auto resolved = ResolveHalfRgbaWithVImage(image, outputWidth, outputHeight, writer);
            resolved.has_value()) {
            return std::move(resolved.value());
        }
    }
#endif
    return ResolveLinearRgba(
        HalfRgbaToLinearPixels(image),
        image.width,
        image.height,
        outputWidth,
        outputHeight,
        spatialAntialiasing,
        writer);
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
            return "MP4";
        case AnimationExportMode::TestMp4:
            return "TestMP4";
        case AnimationExportMode::HevcAlphaMp4:
            return "MP4";
        case AnimationExportMode::PngStack:
            return "PNG";
        case AnimationExportMode::FastPngStack:
            return "FastPNG";
        case AnimationExportMode::HqPreviewDensityExr:
            return "HQ";
        case AnimationExportMode::ProRes422Mov:
            return "ProRes422";
        case AnimationExportMode::ProRes422HqMov:
            return "ProRes422HQ";
        case AnimationExportMode::ProRes422AlphaMatteMov:
            return "ProRes422Alpha";
        case AnimationExportMode::ProRes422HqAlphaMatteMov:
            return "ProRes422HQAlpha";
        case AnimationExportMode::ProRes422VideoToolboxMov:
            return "ProRes422M1";
        case AnimationExportMode::ProRes422HqVideoToolboxMov:
            return "ProRes422HQM1";
        case AnimationExportMode::ProRes4444Mov:
            return "ProRes4444";
        case AnimationExportMode::ProRes4444XqMov:
            return "ProRes4444";
        case AnimationExportMode::ProRes4444VideoToolboxMov:
            return "ProRes4444";
        case AnimationExportMode::ProRes4444XqVideoToolboxMov:
            return "ProRes4444";
    }
    return "export";
}

const char* AnimationExportQualityFilenameToken(AnimationExportQuality quality) {
    switch (quality) {
        case AnimationExportQuality::Normal:
            return "Normal";
        case AnimationExportQuality::Hq:
            return "HQ";
        case AnimationExportQuality::Xq:
            return "XQ";
    }
    return "Normal";
}

std::string AnimationExportModeFilenameToken(
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    bool externalAlphaMatte) {
    const auto compactMode = CompactAnimationExportMode(mode);
    std::string token{AnimationExportModeFilenameToken(compactMode)};
    if (compactMode == AnimationExportMode::TestMp4) {
        token += "_30fps";
        token += useVideoToolbox ? "_VT" : "_CPU";
        if (externalAlphaMatte) {
            token += "_AlphaBlack";
        }
    } else if (compactMode == AnimationExportMode::FastPreviewMp4 ||
        compactMode == AnimationExportMode::ProRes422Mov ||
        compactMode == AnimationExportMode::ProRes4444Mov) {
        token += "_";
        token += AnimationExportQualityFilenameToken(quality);
        token += useVideoToolbox ? "_VT" : "_CPU";
        if (externalAlphaMatte) {
            token += "_Alpha";
        }
    }
    return token;
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
    return BuildAnimationExportFilenameStem(
        animationName,
        CompactAnimationExportMode(mode),
        LegacyQualityForMode(mode),
        LegacyUsesVideoToolbox(mode),
        LegacyWritesExternalAlphaMatte(mode),
        settings,
        visualName);
}

std::string BuildAnimationExportFilenameStem(
    std::string_view animationName,
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    bool externalAlphaMatte,
    const RenderJobSettings& settings,
    std::string_view visualName) {
    (void)quality;
    (void)useVideoToolbox;
    (void)externalAlphaMatte;
    (void)settings;
    (void)visualName;
    auto stem = SanitizeFileStem(animationName, "Animation");
    // Test renders sit beside their final deliverables in the same output
    // directory; the marker keeps them distinguishable at a glance.
    if (CompactAnimationExportMode(mode) == AnimationExportMode::TestMp4) {
        stem += "_test";
    }
    return stem;
}

std::string SanitizeExportOutputNameSuffix(std::string_view suffix) {
    auto sanitized = SanitizeFileStem(suffix, "");
    while (!sanitized.empty() && sanitized.front() == '_') {
        sanitized.erase(sanitized.begin());
    }
    return sanitized;
}

std::filesystem::path AppendExportOutputNameSuffix(
    const std::filesystem::path& path,
    std::string_view sanitizedSuffix) {
    if (path.empty() || sanitizedSuffix.empty()) {
        return path;
    }
    const auto directory = path.parent_path();
    const auto stem = path.stem().string();
    const auto extension = path.extension().string();
    auto candidate =
        directory / (stem + "_" + std::string{sanitizedSuffix} + extension);
    // The un-suffixed name was already unique, so suffixed batch siblings
    // stay unique against each other; only survivors of earlier suffixed
    // runs on disk need the counter.
    for (std::uint32_t counter = 2U; std::filesystem::exists(candidate);
         ++counter) {
        candidate = directory / (stem + "_" + std::string{sanitizedSuffix} +
                                 "_" + std::to_string(counter) + extension);
    }
    return candidate;
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

std::filesystem::path BuildUniqueAnimationExportMediaOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    bool externalAlphaMatte,
    const RenderJobSettings& settings,
    std::string_view extension,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    const auto fullStem = BuildAnimationExportFilenameStem(
        animationName,
        mode,
        quality,
        useVideoToolbox,
        externalAlphaMatte,
        settings,
        visualName);
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

std::filesystem::path BuildUniqueHevcAlphaMp4OutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueHevcAlphaMp4OutputPaths(
        outputDirectory,
        animationName,
        settings,
        visualName,
        reservedPaths).colorPath;
}

HevcAlphaMp4OutputPaths BuildUniqueHevcAlphaMp4OutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueMp4AlphaMatteOutputPaths(
        outputDirectory,
        animationName,
        settings,
        AnimationExportQuality::Hq,
        true,
        visualName,
        reservedPaths);
}

HevcAlphaMp4OutputPaths BuildUniqueMp4AlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    AnimationExportQuality quality,
    bool useVideoToolbox,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    (void)settings;
    (void)quality;
    (void)useVideoToolbox;
    (void)visualName;
    const auto paths = BuildUniqueSimpleAlphaMatteOutputPaths(
        outputDirectory,
        animationName,
        ".mp4",
        reservedPaths);
    return {
        .colorPath = paths.colorPath,
        .alphaMattePath = paths.alphaMattePath,
    };
}

ProResAlphaMatteOutputPaths BuildUniqueProResAlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    AnimationExportMode mode,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    (void)mode;
    (void)settings;
    (void)visualName;
    const auto paths = BuildUniqueSimpleAlphaMatteOutputPaths(
        outputDirectory,
        animationName,
        ".mov",
        reservedPaths);
    return {
        .colorPath = paths.colorPath,
        .alphaMattePath = paths.alphaMattePath,
    };
}

std::filesystem::path BuildUniquePngStackOutputDirectory(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths,
    AnimationExportMode mode) {
    const auto directory = outputDirectory.empty() ? std::filesystem::path{"."} : outputDirectory;
    const auto fullStem = BuildAnimationExportFilenameStem(
        animationName,
        mode,
        settings,
        visualName);
    auto candidate = directory / fullStem;
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
        candidate = directory / (fullStem + "_" + std::to_string(suffix));
    }
    return candidate;
}

std::filesystem::path PngStackFramePath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::uint32_t frameIndex) {
    const auto safeAnimationName = SanitizeFileStem(animationName, "Animation");
    std::ostringstream filename;
    filename << safeAnimationName << "_" << std::setw(4) << std::setfill('0') << (frameIndex + 1U) << ".png";
    return outputDirectory / filename.str();
}

std::filesystem::path PngStackFramePattern(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName) {
    const auto safeAnimationName = SanitizeFileStem(animationName, "Animation");
    return outputDirectory / (safeAnimationName + "_%04d.png");
}

std::filesystem::path BuildUniqueVideoOutputPath(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    std::string_view visualName,
    std::string_view formatSuffix,
    std::string_view extension,
    const std::vector<std::filesystem::path>& reservedPaths) {
    (void)visualName;
    (void)formatSuffix;
    const auto fullStem = SanitizeFileStem(animationName, "Animation");
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

ProResAlphaMatteOutputPaths BuildUniqueProRes422AlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueProResAlphaMatteOutputPaths(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes422AlphaMatteMov,
        settings,
        visualName,
        reservedPaths);
}

ProResAlphaMatteOutputPaths BuildUniqueProRes422HqAlphaMatteOutputPaths(
    const std::filesystem::path& outputDirectory,
    std::string_view animationName,
    const RenderJobSettings& settings,
    std::string_view visualName,
    const std::vector<std::filesystem::path>& reservedPaths) {
    return BuildUniqueProResAlphaMatteOutputPaths(
        outputDirectory,
        animationName,
        AnimationExportMode::ProRes422HqAlphaMatteMov,
        settings,
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

std::string BuildFfmpegTestMp4Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t sourceFramesPerSecond,
    std::uint32_t sourceFrameCount,
    const std::filesystem::path& outputPath,
    bool useVideoToolbox) {
    const auto sourceFps = std::max<std::uint32_t>(1U, sourceFramesPerSecond);
    const auto sourceFrames = std::max<std::uint32_t>(1U, sourceFrameCount);
    const auto outputFrameCount = std::max<std::uint64_t>(
        1ULL,
        ((static_cast<std::uint64_t>(sourceFrames) * kTestMp4OutputFramesPerSecond) +
         static_cast<std::uint64_t>(sourceFps) - 1ULL) /
            static_cast<std::uint64_t>(sourceFps));

    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt rgb48le"
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x"
            << std::max<std::uint32_t>(1U, height)
            << " -r " << sourceFps
            << " -i -"
            << " -an"
            << " -vf format=yuv444p,tpad=stop_mode=clone:stop=2,"
               "minterpolate=fps=" << kTestMp4OutputFramesPerSecond
            << ":mi_mode=mci:mc_mode=aobmc:me_mode=bilat:me=epzs:mb_size=8:"
               "search_param=32:vsbmc=1:scd=fdiff:scd_threshold=10,format=yuv420p";
    if (useVideoToolbox) {
        command << " -c:v hevc_videotoolbox"
                << " -b:v 25000k"
                << " -maxrate 40000k"
                << " -bufsize 50000k"
                << " -tag:v hvc1"
                << " -pix_fmt yuv420p"
                << " -allow_sw 1"
                << " -power_efficient 0"
                << " -spatial_aq 1";
    } else {
        command << " -c:v libx265"
                << " -preset medium"
                << " -b:v 25000k"
                << " -maxrate 40000k"
                << " -bufsize 50000k"
                << " -tag:v hvc1"
                << " -pix_fmt yuv420p"
                << " -x265-params log-level=error";
    }
    command << " -fps_mode cfr"
            << " -frames:v " << outputFrameCount
            << " -movflags +faststart"
            << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(outputPath.string());
    return command.str();
}

std::string BuildFfmpegHevcAlphaMp4Command(
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
            << " -pix_fmt rgba64le"
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -an"
            << " -vf format=ayuv"
            << " -c:v hevc_videotoolbox"
            << " -alpha_quality 1.0"
            << " -tag:v hvc1"
            << " -pix_fmt ayuv"
            << " -allow_sw 1"
            << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(outputPath.string());
    return command.str();
}

std::string BuildFfmpegHevcColorMp4Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegMp4ColorCommand(
        executablePath,
        width,
        height,
        framesPerSecond,
        outputPath,
        AnimationExportQuality::Hq,
        true);
}

std::string BuildFfmpegMp4ColorCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    const bool hq = quality == AnimationExportQuality::Hq || quality == AnimationExportQuality::Xq;
    const auto videoToolboxPixelFormat = hq ? "p210le" : "yuv420p";
    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt " << (hq ? "rgba64le" : "rgba")
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -an";
    if (useVideoToolbox) {
        command << " -vf format=" << videoToolboxPixelFormat
                << " -c:v hevc_videotoolbox";
        if (hq) {
            command << " -profile:v main42210"
                    << " -b:v 300000k"
                    << " -maxrate 450000k";
        } else {
            command << " -b:v 80000k"
                    << " -maxrate 120000k";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << videoToolboxPixelFormat
                << " -allow_sw 1"
                << " -power_efficient 0"
                << " -spatial_aq 1";
        if (hq) {
            command << " -prio_speed 0";
        }
    } else {
        command << " -vf format=" << (hq ? "yuv420p10le" : "yuv420p")
                << " -c:v libx265"
                << " -preset " << (hq ? "slow" : "medium")
                << " -crf " << (hq ? "14" : "18");
        if (hq) {
            command << " -profile:v main10";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << (hq ? "yuv420p10le" : "yuv420p")
                << " -x265-params log-level=error";
    }
    command << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(outputPath.string());
    return command.str();
}

std::string BuildFfmpegHevcAlphaMatteMp4Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath) {
    return BuildFfmpegMp4AlphaMatteCommand(
        executablePath,
        width,
        height,
        framesPerSecond,
        outputPath,
        AnimationExportQuality::Hq,
        true);
}

std::string BuildFfmpegMp4AlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    const bool hq = quality == AnimationExportQuality::Hq || quality == AnimationExportQuality::Xq;
    const auto videoToolboxPixelFormat = hq ? "p210le" : "yuv420p";
    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt " << (hq ? "gray16le" : "gray")
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -an"
            << " -vf scale=in_range=full:out_range=full,format="
            << (useVideoToolbox ? videoToolboxPixelFormat : (hq ? "yuv420p10le" : "yuv420p"));
    if (useVideoToolbox) {
        command << " -c:v hevc_videotoolbox";
        if (hq) {
            command << " -profile:v main42210"
                    << " -b:v 90000k"
                    << " -maxrate 135000k";
        } else {
            command << " -b:v 30000k"
                    << " -maxrate 45000k";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << videoToolboxPixelFormat
                << " -allow_sw 1"
                << " -power_efficient 0"
                << " -spatial_aq 1";
        if (hq) {
            command << " -prio_speed 0";
        }
    } else {
        command << " -c:v libx265"
                << " -preset " << (hq ? "slow" : "medium")
                << " -crf " << (hq ? "12" : "16");
        if (hq) {
            command << " -profile:v main10";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << (hq ? "yuv420p10le" : "yuv420p")
                << " -x265-params log-level=error";
    }
    command << " -color_range pc"
            << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(outputPath.string());
    return command.str();
}

std::string BuildFfmpegMp4ColorAndAlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& colorOutputPath,
    const std::filesystem::path& alphaMatteOutputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    const bool hq = quality == AnimationExportQuality::Hq || quality == AnimationExportQuality::Xq;
    const auto inputPixelFormat = hq ? "rgba64le" : "rgba";
    const auto colorPixelFormat = hq ? (useVideoToolbox ? "p210le" : "yuv420p10le") : "yuv420p";
    const auto mattePixelFormat = colorPixelFormat;
    const std::string filter =
        std::string{"[0:v]split=2[color_src][alpha_src];"} +
        "[color_src]format=" + colorPixelFormat + "[color_out];" +
        "[alpha_src]alphaextract,scale=in_range=full:out_range=full,format=" +
        mattePixelFormat + "[alpha_out]";

    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt " << inputPixelFormat
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -filter_complex " << ShellQuote(filter)
            << " -map " << ShellQuote("[color_out]")
            << " -an";
    if (useVideoToolbox) {
        command << " -c:v hevc_videotoolbox";
        if (hq) {
            command << " -profile:v main42210"
                    << " -b:v 300000k"
                    << " -maxrate 450000k";
        } else {
            command << " -b:v 80000k"
                    << " -maxrate 120000k";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << colorPixelFormat
                << " -allow_sw 1"
                << " -power_efficient 0"
                << " -spatial_aq 1";
        if (hq) {
            command << " -prio_speed 0";
        }
    } else {
        command << " -c:v libx265"
                << " -preset " << (hq ? "slow" : "medium")
                << " -crf " << (hq ? "14" : "18");
        if (hq) {
            command << " -profile:v main10";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << colorPixelFormat
                << " -x265-params log-level=error";
    }
    command << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(colorOutputPath.string())
            << " -map " << ShellQuote("[alpha_out]")
            << " -an";
    if (useVideoToolbox) {
        command << " -c:v hevc_videotoolbox";
        if (hq) {
            command << " -profile:v main42210"
                    << " -b:v 90000k"
                    << " -maxrate 135000k";
        } else {
            command << " -b:v 30000k"
                    << " -maxrate 45000k";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << mattePixelFormat
                << " -allow_sw 1"
                << " -power_efficient 0"
                << " -spatial_aq 1";
        if (hq) {
            command << " -prio_speed 0";
        }
    } else {
        command << " -c:v libx265"
                << " -preset " << (hq ? "slow" : "medium")
                << " -crf " << (hq ? "12" : "16");
        if (hq) {
            command << " -profile:v main10";
        }
        command << " -tag:v hvc1"
                << " -pix_fmt " << mattePixelFormat
                << " -x265-params log-level=error";
    }
    command << " -color_range pc"
            << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(alphaMatteOutputPath.string());
    return command.str();
}

std::string BuildFfmpegPngStackCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPattern) {
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
            << " -threads 0"
            << " -c:v png"
            << " -pred mixed"
            << " -compression_level 3"
            << " -start_number 1"
            << " -f image2 "
            << ShellQuote(outputPattern.string());
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

std::string BuildFfmpegProRes422Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    const auto profile = quality == AnimationExportQuality::Hq || quality == AnimationExportQuality::Xq
                             ? 3U
                             : 2U;
    return BuildFfmpegProRes422Command(
        executablePath,
        width,
        height,
        framesPerSecond,
        outputPath,
        profile,
        useVideoToolbox);
}

std::string BuildFfmpegProRes422ColorAndAlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& colorOutputPath,
    const std::filesystem::path& alphaMatteOutputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    const auto profile = quality == AnimationExportQuality::Hq || quality == AnimationExportQuality::Xq
                             ? 3U
                             : 2U;
    const auto colorPixelFormat = useVideoToolbox ? "p210le" : "rgb48le";
    const auto outputPixelFormat = useVideoToolbox ? "p210le" : "yuv422p10le";
    const std::string filter =
        std::string{"[0:v]split=2[color_src][alpha_src];"} +
        "[color_src]format=" + colorPixelFormat + "[color_out];" +
        "[alpha_src]alphaextract,scale=in_range=full:out_range=full,format=" +
        outputPixelFormat + "[alpha_out]";

    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt rgba64le"
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -filter_complex " << ShellQuote(filter)
            << " -map " << ShellQuote("[color_out]")
            << " -an";
    if (useVideoToolbox) {
        command << " -c:v prores_videotoolbox"
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
            << ShellQuote(colorOutputPath.string())
            << " -map " << ShellQuote("[alpha_out]")
            << " -an";
    if (useVideoToolbox) {
        command << " -c:v prores_videotoolbox"
                << " -profile:v " << std::clamp<std::uint32_t>(profile, 2U, 3U)
                << " -pix_fmt p210le"
                << " -allow_sw 1";
    } else {
        command << " -c:v prores_ks"
                << " -profile:v " << std::clamp<std::uint32_t>(profile, 2U, 3U)
                << " -pix_fmt yuv422p10le";
    }
    command << " -color_range pc"
            << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(alphaMatteOutputPath.string());
    return command.str();
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

std::string BuildFfmpegProRes4444Command(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& outputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    const auto profile = quality == AnimationExportQuality::Xq ? 5U : 4U;
    return BuildFfmpegProResCommand(
        executablePath,
        width,
        height,
        framesPerSecond,
        outputPath,
        profile,
        useVideoToolbox);
}

std::string BuildFfmpegProRes4444ColorAndAlphaMatteCommand(
    const std::filesystem::path& executablePath,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t framesPerSecond,
    const std::filesystem::path& colorOutputPath,
    const std::filesystem::path& alphaMatteOutputPath,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    const auto colorProfile = quality == AnimationExportQuality::Xq ? 5U : 4U;
    const auto matteProfile = quality == AnimationExportQuality::Xq ? 3U : 2U;
    const auto colorPixelFormat = useVideoToolbox ? "ayuv64le" : "yuva444p10le";
    const auto mattePixelFormat = useVideoToolbox ? "p210le" : "yuv422p10le";
    const std::string filter =
        std::string{"[0:v]split=2[color_src][alpha_src];"} +
        "[color_src]format=" + colorPixelFormat + "[color_out];" +
        "[alpha_src]alphaextract,scale=in_range=full:out_range=full,format=" +
        mattePixelFormat + "[alpha_out]";

    std::ostringstream command;
    command << ShellQuote(executablePath.string())
            << " -y"
            << " -loglevel error"
            << " -f rawvideo"
            << " -pix_fmt rgba64le"
            << " -s:v " << std::max<std::uint32_t>(1U, width) << "x" << std::max<std::uint32_t>(1U, height)
            << " -r " << std::max<std::uint32_t>(1U, framesPerSecond)
            << " -i -"
            << " -filter_complex " << ShellQuote(filter)
            << " -map " << ShellQuote("[color_out]")
            << " -an";
    if (useVideoToolbox) {
        command << " -c:v prores_videotoolbox"
                << " -profile:v " << std::clamp<std::uint32_t>(colorProfile, 4U, 5U)
                << " -pix_fmt ayuv64le"
                << " -allow_sw 1";
    } else {
        command << " -c:v prores_ks"
                << " -profile:v " << std::clamp<std::uint32_t>(colorProfile, 4U, 5U)
                << " -pix_fmt yuva444p10le"
                << " -alpha_bits 16";
    }
    command << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(colorOutputPath.string())
            << " -map " << ShellQuote("[alpha_out]")
            << " -an";
    if (useVideoToolbox) {
        command << " -c:v prores_videotoolbox"
                << " -profile:v " << std::clamp<std::uint32_t>(matteProfile, 2U, 3U)
                << " -pix_fmt p210le"
                << " -allow_sw 1";
    } else {
        command << " -c:v prores_ks"
                << " -profile:v " << std::clamp<std::uint32_t>(matteProfile, 2U, 3U)
                << " -pix_fmt yuv422p10le";
    }
    command << " -color_range pc"
            << " -color_primaries bt709"
            << " -color_trc iec61966-2-1"
            << " -colorspace bt709 "
            << ShellQuote(alphaMatteOutputPath.string());
    return command.str();
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
    const Mp4SparsePointSmoothingSettings&) {
    return ConvertHalfRgbaToSrgbRgba8(image, image.width, image.height);
}

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8(
    const HalfRgbaExrImage& image,
    std::uint32_t outputWidth,
    std::uint32_t outputHeight,
    const Mp4SparsePointSmoothingSettings&,
    bool spatialAntialiasing) {
    const auto pixelCount = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    if (image.width == 0 ||
        image.height == 0 ||
        outputWidth == 0 ||
        outputHeight == 0 ||
        image.rgbaHalf.size() != pixelCount * 4U) {
        return {};
    }

    return ResolveHalfRgba(
        image,
        outputWidth,
        outputHeight,
        spatialAntialiasing,
        Rgba8OutputWriter{});
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

    return ResolveHalfRgba(
        image,
        outputWidth,
        outputHeight,
        spatialAntialiasing,
        Rgba16OutputWriter{});
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

    return ResolveHalfRgba(
        image,
        outputWidth,
        outputHeight,
        spatialAntialiasing,
        OpaqueBlackRgb16OutputWriter{});
}

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba8OpaqueBlack(
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

    return ResolveHalfRgba(
        image,
        outputWidth,
        outputHeight,
        spatialAntialiasing,
        OpaqueBlackRgba8OutputWriter{});
}

std::vector<std::uint8_t> ConvertHalfRgbaToSrgbRgba16OpaqueBlack(
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

    return ResolveHalfRgba(
        image,
        outputWidth,
        outputHeight,
        spatialAntialiasing,
        OpaqueBlackRgba16OutputWriter{});
}

}  // namespace invisible_places::output
