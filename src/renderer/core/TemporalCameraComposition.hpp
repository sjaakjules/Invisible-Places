#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace invisible_places::renderer::core {

enum class TemporalCameraCompositionMode : std::uint32_t {
    ReprojectedBlend = 1U,
    Split = 2U,
    AlphaOver = 3U,
};

constexpr bool CurrentSourceIsSelectedTop(
    bool currentIsOrderedFirst,
    bool orderedFirstIsSelectedTop) {
    return currentIsOrderedFirst == orderedFirstIsSelectedTop;
}

constexpr std::array<float, 4> CompositeStraightAlphaOver(
    const std::array<float, 4>& top,
    const std::array<float, 4>& bottom,
    float topOpacity = 1.0F,
    float bottomOpacity = 1.0F) {
    const float topAlpha = std::clamp(
        top[3] * std::clamp(topOpacity, 0.0F, 1.0F),
        0.0F,
        1.0F);
    const float bottomAlpha = std::clamp(
        bottom[3] * std::clamp(bottomOpacity, 0.0F, 1.0F),
        0.0F,
        1.0F);
    const float outputAlpha =
        topAlpha + (bottomAlpha * (1.0F - topAlpha));
    if (outputAlpha <= 1.0e-6F) {
        return {0.0F, 0.0F, 0.0F, 0.0F};
    }

    std::array<float, 4> output{};
    for (std::size_t channel = 0U; channel < 3U; ++channel) {
        const float premultiplied =
            (top[channel] * topAlpha) +
            (bottom[channel] * bottomAlpha * (1.0F - topAlpha));
        output[channel] = premultiplied / outputAlpha;
    }
    output[3] = outputAlpha;
    return output;
}

}  // namespace invisible_places::renderer::core
