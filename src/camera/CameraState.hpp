#pragma once

#include <array>

namespace invisible_places::camera {

struct CameraState {
    std::array<float, 3> position{0.0F, 0.0F, 0.0F};
    std::array<float, 4> orientation{0.0F, 0.0F, 0.0F, 1.0F};
    std::array<float, 3> target{0.0F, 0.0F, 0.0F};
    float fovDegrees = 60.0F;
    float nearPlane = 0.01F;
    float farPlane = 1000.0F;
};

}  // namespace invisible_places::camera

