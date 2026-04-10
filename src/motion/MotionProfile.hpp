#pragma once

#include "style/RenderParameterBinding.hpp"

namespace invisible_places::motion {

struct MotionProfile {
    bool enabled = false;
    invisible_places::style::RenderParameterBinding amplitude;
    invisible_places::style::RenderParameterBinding frequency;
};

}  // namespace invisible_places::motion

