#pragma once

#include "camera/CameraShot.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace invisible_places::camera {

struct CameraPathTiming {
    std::size_t fromIndex = 0;
    std::size_t toIndex = 0;
    std::vector<float> knotSeconds;

    [[nodiscard]] bool IsValid() const {
        return toIndex > fromIndex &&
               knotSeconds.size() == ((toIndex - fromIndex) + 1U) &&
               !knotSeconds.empty();
    }

    [[nodiscard]] float DurationSeconds() const {
        return knotSeconds.empty() ? 0.0F : knotSeconds.back();
    }
};

CameraPathTiming BuildCameraPathTiming(
    const std::vector<CameraShot>& shots,
    std::size_t fromIndex,
    std::size_t toIndex);

std::vector<CameraShot> BuildWeightedCameraPathShots(
    const std::vector<CameraShot>& orderedShots,
    std::uint32_t totalDurationFrames);

CameraState EvaluateCameraPath(
    const std::vector<CameraShot>& shots,
    const CameraPathTiming& timing,
    float timeSeconds);

}  // namespace invisible_places::camera
