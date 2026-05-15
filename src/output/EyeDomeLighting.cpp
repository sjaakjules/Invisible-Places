#include "output/EyeDomeLighting.hpp"

#include <Imath/half.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace invisible_places::output {

namespace {

float HalfBitsToFloat(std::uint16_t bits) {
    return static_cast<float>(Imath::half{Imath::half::FromBits, bits});
}

std::uint16_t FloatToHalfBits(float value) {
    return Imath::half{std::max(0.0F, value)}.bits();
}

bool ValidDepth(float depth) {
    return std::isfinite(depth) && depth > 0.0F;
}

float LogDepth(float depth) {
    return std::log2(std::max(depth, 1.0e-6F));
}

}  // namespace

float ComputeEyeDomeLightingShade(
    const float* linearDepth,
    unsigned int width,
    unsigned int height,
    unsigned int x,
    unsigned int y,
    const EyeDomeLightingSettings& settings) {
    if (!settings.enabled ||
        linearDepth == nullptr ||
        width == 0U ||
        height == 0U ||
        x >= width ||
        y >= height) {
        return 1.0F;
    }

    const auto centerIndex =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
    const float centerDepth = linearDepth[centerIndex];
    if (!ValidDepth(centerDepth)) {
        return 1.0F;
    }

    constexpr std::array<std::array<int, 2>, 8> kOffsets{{
        {{-1, 0}},
        {{1, 0}},
        {{0, -1}},
        {{0, 1}},
        {{-1, -1}},
        {{1, -1}},
        {{-1, 1}},
        {{1, 1}},
    }};

    const float centerLogDepth = LogDepth(centerDepth);
    float response = 0.0F;
    unsigned int sampleCount = 0U;
    for (const auto& offset : kOffsets) {
        const int nx = static_cast<int>(x) + offset[0];
        const int ny = static_cast<int>(y) + offset[1];
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
            continue;
        }

        const auto neighborIndex =
            static_cast<std::size_t>(ny) * static_cast<std::size_t>(width) + static_cast<std::size_t>(nx);
        const float neighborDepth = linearDepth[neighborIndex];
        if (!ValidDepth(neighborDepth)) {
            continue;
        }

        response += std::max(0.0F, LogDepth(neighborDepth) - centerLogDepth);
        ++sampleCount;
    }

    if (sampleCount == 0U || response <= 1.0e-6F) {
        return 1.0F;
    }

    const float shade = std::exp(-std::max(0.0F, settings.strength) * response / static_cast<float>(sampleCount));
    return std::clamp(shade, std::clamp(settings.minShade, 0.0F, 1.0F), 1.0F);
}

void ApplyEyeDomeLighting(HalfRgbaExrImage* image, const EyeDomeLightingSettings& settings) {
    if (image == nullptr || !settings.enabled) {
        return;
    }

    const auto pixelCount = static_cast<std::size_t>(image->width) * static_cast<std::size_t>(image->height);
    if (image->width == 0U ||
        image->height == 0U ||
        image->rgbaHalf.size() != pixelCount * 4U ||
        image->depth.size() != pixelCount) {
        return;
    }

    for (unsigned int y = 0U; y < image->height; ++y) {
        for (unsigned int x = 0U; x < image->width; ++x) {
            const auto pixelIndex =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) + static_cast<std::size_t>(x);
            const float shade =
                ComputeEyeDomeLightingShade(image->depth.data(), image->width, image->height, x, y, settings);
            if (shade >= 0.9999F) {
                continue;
            }

            const auto componentOffset = pixelIndex * 4U;
            image->rgbaHalf[componentOffset + 0U] =
                FloatToHalfBits(HalfBitsToFloat(image->rgbaHalf[componentOffset + 0U]) * shade);
            image->rgbaHalf[componentOffset + 1U] =
                FloatToHalfBits(HalfBitsToFloat(image->rgbaHalf[componentOffset + 1U]) * shade);
            image->rgbaHalf[componentOffset + 2U] =
                FloatToHalfBits(HalfBitsToFloat(image->rgbaHalf[componentOffset + 2U]) * shade);
        }
    }
}

}  // namespace invisible_places::output
