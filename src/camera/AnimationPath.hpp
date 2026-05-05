#pragma once

#include "camera/CameraShot.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace invisible_places::camera {

struct AnimationPathKey {
    std::array<float, 3> cameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 3> focusPoint{0.0F, 0.0F, 0.0F};
    float fovDegrees = 60.0F;
    float nearPlane = 0.01F;
    float farPlane = 1000.0F;
    std::uint32_t durationFrames = 90;
    std::string sourceShotName;
};

struct AnimationPath {
    std::string name = "Animation";
    std::uint32_t durationFrames = 180;
    std::vector<AnimationPathKey> keys;
    bool depthOfFieldEnabled = false;
    float apertureFStops = 8.0F;
    float depthOfFieldMaxBlurPixels = 24.0F;
};

struct AnimationPathEvaluation {
    CameraState camera;
    std::array<float, 3> focusPoint{0.0F, 0.0F, 0.0F};
    float focusDistance = 1.0F;
};

AnimationPath BuildAnimationPathFromCameraShots(
    const std::string& name,
    const std::vector<CameraShot>& orderedShots,
    std::uint32_t durationFrames,
    float apertureFStops = 8.0F);

AnimationPathEvaluation EvaluateAnimationPath(
    const AnimationPath& path,
    float timeSeconds);

void MoveAnimationCameraKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& cameraPosition);

void MoveAnimationFocusKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& focusPoint);

}  // namespace invisible_places::camera
