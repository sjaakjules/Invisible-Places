#pragma once

#include "camera/CameraState.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

#include <glm/gtc/quaternion.hpp>

namespace invisible_places::camera {

struct CameraShot {
    std::string name;
    CameraState state;
    std::uint32_t durationFrames = 90;
};

inline float LerpFloat(float left, float right, float amount) {
    return left + ((right - left) * amount);
}

inline glm::quat QuaternionFromCameraState(const CameraState& state) {
    return glm::normalize(glm::quat{
        state.orientation[3],
        state.orientation[0],
        state.orientation[1],
        state.orientation[2],
    });
}

inline void WriteQuaternionToCameraState(const glm::quat& orientation, CameraState* state) {
    if (state == nullptr) {
        return;
    }

    const auto normalized = glm::normalize(orientation);
    state->orientation = {
        normalized.x,
        normalized.y,
        normalized.z,
        normalized.w,
    };
}

inline CameraState InterpolateCameraStates(
    const CameraState& from,
    const CameraState& to,
    float amount) {
    const float t = std::clamp(amount, 0.0F, 1.0F);
    CameraState result;

    for (std::size_t index = 0; index < result.position.size(); ++index) {
        result.position[index] = LerpFloat(from.position[index], to.position[index], t);
        result.target[index] = LerpFloat(from.target[index], to.target[index], t);
    }

    auto fromOrientation = QuaternionFromCameraState(from);
    auto toOrientation = QuaternionFromCameraState(to);
    if (glm::dot(fromOrientation, toOrientation) < 0.0F) {
        toOrientation = -toOrientation;
    }
    WriteQuaternionToCameraState(glm::slerp(fromOrientation, toOrientation, t), &result);

    result.fovDegrees = LerpFloat(from.fovDegrees, to.fovDegrees, t);
    result.nearPlane = LerpFloat(from.nearPlane, to.nearPlane, t);
    result.farPlane = LerpFloat(from.farPlane, to.farPlane, t);
    return result;
}

}  // namespace invisible_places::camera
