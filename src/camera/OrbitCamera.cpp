#include "camera/OrbitCamera.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>

namespace invisible_places::camera {

namespace {

constexpr float kPitchLimit = 1.45F;

glm::vec3 ToGlm(const invisible_places::io::Float3& value) {
    return {value.x, value.y, value.z};
}

}  // namespace

void OrbitCamera::FrameBounds(const invisible_places::io::Bounds3f& bounds, float aspectRatio) {
    if (!bounds.valid) {
        return;
    }

    framedBounds_ = bounds;
    framedFocusPoint_ = 0.5F * (ToGlm(bounds.minimum) + ToGlm(bounds.maximum));
    ApplyFramedBounds(aspectRatio);
}

void OrbitCamera::FrameBounds(
    const invisible_places::io::Bounds3f& bounds,
    const invisible_places::io::Float3& focusPoint,
    float aspectRatio) {
    if (!bounds.valid) {
        return;
    }

    framedBounds_ = bounds;
    framedFocusPoint_ = ToGlm(focusPoint);
    ApplyFramedBounds(aspectRatio);
}

void OrbitCamera::ResetToFramedBounds(float aspectRatio) {
    ApplyFramedBounds(aspectRatio);
}

void OrbitCamera::Orbit(float deltaX, float deltaY) {
    yawRadians_ -= deltaX * 0.0105F;
    pitchRadians_ = std::clamp(pitchRadians_ - deltaY * 0.0105F, -kPitchLimit, kPitchLimit);
}

void OrbitCamera::Pan(float deltaX, float deltaY, float viewportWidth, float viewportHeight) {
    const auto safeViewportWidth = std::max(1.0F, viewportWidth);
    const auto safeViewportHeight = std::max(1.0F, viewportHeight);
    const auto panScale = std::max(0.001F, distance_ * 0.0014F);
    target_ += (-Right() * (deltaX / safeViewportWidth) * distance_ * panScale * 1200.0F);
    target_ += (Up() * (deltaY / safeViewportHeight) * distance_ * panScale * 1200.0F);
}

void OrbitCamera::Dolly(float wheelDelta) {
    if (wheelDelta == 0.0F) {
        return;
    }

    const auto zoomFactor = std::pow(0.82F, wheelDelta);
    distance_ = std::max(minimumDistance_, distance_ * zoomFactor);
    UpdateClippingPlanes();
}

OrbitCameraMatrices OrbitCamera::Matrices(float aspectRatio) const {
    OrbitCameraMatrices matrices;
    matrices.position = Position();
    matrices.view = glm::lookAtRH(matrices.position, target_, WorldUp());
    matrices.projection = glm::perspective(glm::radians(fovDegrees_), EffectiveAspectRatio(aspectRatio), nearPlane_, farPlane_);
    matrices.projection[1][1] *= -1.0F;
    matrices.viewProjection = matrices.projection * matrices.view;
    return matrices;
}

glm::vec3 OrbitCamera::WorldUp() const {
    return {0.0F, 0.0F, 1.0F};
}

glm::vec3 OrbitCamera::Position() const {
    const auto cosPitch = std::cos(pitchRadians_);
    const auto offset = glm::vec3{
        std::cos(yawRadians_) * cosPitch,
        std::sin(yawRadians_) * cosPitch,
        std::sin(pitchRadians_),
    };
    return target_ - (glm::normalize(offset) * distance_);
}

glm::vec3 OrbitCamera::Forward() const {
    return glm::normalize(target_ - Position());
}

glm::vec3 OrbitCamera::Right() const {
    return glm::normalize(glm::cross(Forward(), WorldUp()));
}

glm::vec3 OrbitCamera::Up() const {
    return glm::normalize(glm::cross(Right(), Forward()));
}

float OrbitCamera::EffectiveAspectRatio(float aspectRatio) const {
    return std::max(0.1F, aspectRatio);
}

void OrbitCamera::ApplyFramedBounds(float aspectRatio) {
    if (!framedBounds_.valid) {
        return;
    }

    const auto minimum = ToGlm(framedBounds_.minimum);
    const auto maximum = ToGlm(framedBounds_.maximum);
    target_ = framedFocusPoint_;

    const auto extent = maximum - minimum;
    framedRadius_ = std::max(0.5F * glm::length(extent), 0.25F);
    const auto verticalDistance = framedRadius_ / std::tan(glm::radians(fovDegrees_) * 0.5F);
    const auto horizontalFov = 2.0F * std::atan(std::tan(glm::radians(fovDegrees_) * 0.5F) * EffectiveAspectRatio(aspectRatio));
    const auto horizontalDistance = framedRadius_ / std::tan(horizontalFov * 0.5F);
    distance_ = std::max(verticalDistance, horizontalDistance) * 1.35F;
    minimumDistance_ = std::max(0.0005F, framedRadius_ * 0.00005F);
    UpdateClippingPlanes();
}

void OrbitCamera::UpdateClippingPlanes() {
    const auto safeRadius = std::max(framedRadius_, 0.25F);
    nearPlane_ = std::max(0.0001F, std::min(safeRadius * 0.0005F, distance_ * 0.2F));
    farPlane_ = std::max(nearPlane_ + 1.0F, (safeRadius * 60.0F) + (distance_ * 4.0F));
}

}  // namespace invisible_places::camera
