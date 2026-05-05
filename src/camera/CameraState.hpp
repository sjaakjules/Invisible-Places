#pragma once

#include <array>

namespace invisible_places::camera {

struct CameraState {
    std::array<float, 3> position{0.0F, 0.0F, 0.0F};
    std::array<float, 4> orientation{0.0F, 0.0F, 0.0F, 1.0F};
    std::array<float, 3> target{0.0F, 0.0F, 0.0F};
    std::array<float, 3> orbitCenter{0.0F, 0.0F, 0.0F};
    bool hasOrbitCenter = false;
    bool hasDepthOfField = false;
    float fovDegrees = 60.0F;
    float nearPlane = 0.01F;
    float farPlane = 1000.0F;
    float focusDistance = 1.0F;
    float apertureFStops = 8.0F;
    float depthOfFieldMaxBlurPixels = 24.0F;
};

}  // namespace invisible_places::camera
