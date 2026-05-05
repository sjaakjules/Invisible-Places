#pragma once

#include "style/RenderParameterBinding.hpp"

namespace invisible_places::motion {

// Data-only placeholder for the next motion slice; UI, shader evaluation,
// project serialization, and export integration are still outstanding.
struct MotionProfile {
    bool enabled = false;
    invisible_places::style::RenderParameterBinding amplitude;
    invisible_places::style::RenderParameterBinding frequency;
};

}  // namespace invisible_places::motion
