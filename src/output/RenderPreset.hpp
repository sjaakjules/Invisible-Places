#pragma once

#include <cstdint>

namespace invisible_places::output {

struct RenderPreset {
    std::uint32_t width = 1920;
    std::uint32_t height = 1080;
    bool tiledRendering = false;
    bool exportDepth = true;
    bool exportAlpha = true;
};

}  // namespace invisible_places::output

