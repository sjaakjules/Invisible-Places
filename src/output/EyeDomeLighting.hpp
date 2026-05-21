#pragma once

#include "output/ExrWriter.hpp"

namespace invisible_places::output {

struct EyeDomeLightingSettings {
    bool enabled = false;
    float strength = 24.0F;
    float minShade = 0.35F;
    float outlineThicknessPixels = 1.0F;
};

[[nodiscard]] float ComputeEyeDomeLightingShade(
    const float* linearDepth,
    unsigned int width,
    unsigned int height,
    unsigned int x,
    unsigned int y,
    const EyeDomeLightingSettings& settings);

void ApplyEyeDomeLighting(
    HalfRgbaExrImage* image,
    const EyeDomeLightingSettings& settings);

}  // namespace invisible_places::output
