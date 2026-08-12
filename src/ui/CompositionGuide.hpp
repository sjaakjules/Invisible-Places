#pragma once

#include <array>
#include <cstddef>

namespace invisible_places::ui {

struct CompositionGuideState {
    bool enabled = false;
    bool showHalfway = true;
    bool showThirds = true;
    std::array<float, 3> colour{1.0F, 1.0F, 1.0F};
    float opacity = 0.55F;
    float lineThickness = 1.25F;
};

struct CompositionGuideLine {
    float normalizedPosition = 0.0F;
    bool halfway = false;
};

struct CompositionGuideLines {
    std::array<CompositionGuideLine, 3> values{};
    std::size_t count = 0U;
};

constexpr CompositionGuideLines BuildCompositionGuideLines(
    bool showHalfway,
    bool showThirds) {
    CompositionGuideLines lines;
    if (showThirds) {
        lines.values[lines.count++] = {
            .normalizedPosition = 1.0F / 3.0F,
            .halfway = false,
        };
    }
    if (showHalfway) {
        lines.values[lines.count++] = {
            .normalizedPosition = 0.5F,
            .halfway = true,
        };
    }
    if (showThirds) {
        lines.values[lines.count++] = {
            .normalizedPosition = 2.0F / 3.0F,
            .halfway = false,
        };
    }
    return lines;
}

}  // namespace invisible_places::ui
