#include "camera/OrbitCamera.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>
#include <glm/trigonometric.hpp>

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
    if (deltaX == 0.0F && deltaY == 0.0F) {
        return;
    }

    const auto position = Position();
    const auto viewTarget = target_;
    const auto pivot = orbitCenter_;
    const float yawDelta = -deltaX * 0.0105F;
    const float pitchDelta = -deltaY * 0.0105F;

    glm::vec3 pitchAxis = glm::cross(Forward(), WorldUp());
    if (glm::length(pitchAxis) <= 1.0e-5F) {
        pitchAxis = {1.0F, 0.0F, 0.0F};
    } else {
        pitchAxis = glm::normalize(pitchAxis);
    }

    glm::mat4 rotation{1.0F};
    rotation = glm::rotate(rotation, yawDelta, WorldUp());
    rotation = glm::rotate(rotation, pitchDelta, pitchAxis);

    const auto rotateAroundPivot = [&](const glm::vec3& point) {
        return pivot + glm::vec3{rotation * glm::vec4{point - pivot, 1.0F}};
    };

    ApplyPositionTarget(rotateAroundPivot(position), rotateAroundPivot(viewTarget));
    orbitCenter_ = pivot;
}

void OrbitCamera::Pan(float deltaX, float deltaY, float viewportWidth, float viewportHeight) {
    const auto safeViewportWidth = std::max(1.0F, viewportWidth);
    const auto safeViewportHeight = std::max(1.0F, viewportHeight);
    const float verticalWorldSpan = 2.0F * distance_ * std::tan(glm::radians(fovDegrees_) * 0.5F);
    const float horizontalWorldSpan = verticalWorldSpan * (safeViewportWidth / safeViewportHeight);
    const float worldUnitsPerPixelX = horizontalWorldSpan / safeViewportWidth;
    const float worldUnitsPerPixelY = verticalWorldSpan / safeViewportHeight;
    constexpr float kPanDamping = 0.55F;
    const glm::vec3 panDelta =
        (-Right() * deltaX * worldUnitsPerPixelX * kPanDamping) +
        (Up() * deltaY * worldUnitsPerPixelY * kPanDamping);
    position_ += panDelta;
    target_ += panDelta;
    distance_ = std::max(minimumDistance_, glm::length(orbitCenter_ - position_));
    UpdateClippingPlanes();
}

void OrbitCamera::Dolly(float wheelDelta) {
    if (wheelDelta == 0.0F) {
        return;
    }

    const auto zoomFactor = std::pow(0.82F, wheelDelta);
    auto pivotOffset = position_ - orbitCenter_;
    if (glm::length(pivotOffset) <= 1.0e-6F) {
        pivotOffset = -Forward() * std::max(distance_, minimumDistance_);
    }

    const float requestedDistance = glm::length(pivotOffset) * zoomFactor;
    const float nextDistance = std::max(minimumDistance_, requestedDistance);
    const auto nextPosition = orbitCenter_ + (glm::normalize(pivotOffset) * nextDistance);
    const auto translation = nextPosition - position_;
    position_ = nextPosition;
    target_ += translation;
    distance_ = nextDistance;
    UpdateClippingPlanes();
}

void OrbitCamera::SetTargetPreservingPosition(const glm::vec3& target) {
    ApplyPositionTarget(Position(), target);
    orbitCenter_ = target_;
    distance_ = glm::length(orbitCenter_ - position_);
}

void OrbitCamera::SetOrbitCenterPreservingView(const glm::vec3& center) {
    orbitCenter_ = center;
    distance_ = glm::length(orbitCenter_ - position_);
    UpdateClippingPlanes();
}

void OrbitCamera::ApplyState(const CameraState& state) {
    glm::vec3 position{
        state.position[0],
        state.position[1],
        state.position[2],
    };
    glm::vec3 target{
        state.target[0],
        state.target[1],
        state.target[2],
    };
    glm::vec3 orbitCenter = state.hasOrbitCenter
                                ? glm::vec3{
                                      state.orbitCenter[0],
                                      state.orbitCenter[1],
                                      state.orbitCenter[2],
                                  }
                                : target;

    const glm::quat storedOrientation{
        state.orientation[3],
        state.orientation[0],
        state.orientation[1],
        state.orientation[2],
    };
    const float orientationLengthSquared =
        (storedOrientation.w * storedOrientation.w) +
        (storedOrientation.x * storedOrientation.x) +
        (storedOrientation.y * storedOrientation.y) +
        (storedOrientation.z * storedOrientation.z);

    glm::vec3 forward{0.0F, 0.0F, -1.0F};
    const glm::vec3 targetOffset = target - position;
    const float targetDistance = glm::length(targetOffset);
    if (orientationLengthSquared > 1.0e-8F) {
        forward = glm::normalize(storedOrientation) * glm::vec3{0.0F, 0.0F, -1.0F};
        if (!state.hasOrbitCenter && targetDistance > 1.0e-5F) {
            const float alignment = glm::dot(glm::normalize(targetOffset), glm::normalize(forward));
            if (alignment < 0.999F) {
                orbitCenter = target;
            }
        }
        if (targetDistance > 1.0e-5F) {
            target = position + (glm::normalize(forward) * targetDistance);
        }
    } else if (targetDistance > 1.0e-5F) {
        forward = glm::normalize(targetOffset);
    }

    if (glm::length(target - position) <= 1.0e-5F) {
        const float orbitDistance = glm::length(orbitCenter - position);
        const float viewDistance = orbitDistance > 1.0e-5F
                                       ? orbitDistance
                                       : std::max(distance_, minimumDistance_);
        target = position + (glm::normalize(forward) * viewDistance);
    }

    orbitCenter_ = orbitCenter;
    fovDegrees_ = std::clamp(state.fovDegrees, 1.0F, 160.0F);
    ApplyPositionTarget(position, target);
    nearPlane_ = std::max(0.0001F, state.nearPlane);
    farPlane_ = std::max(nearPlane_ + 1.0F, state.farPlane);
}

CameraState OrbitCamera::CaptureState() const {
    const auto position = Position();
    const auto view = glm::lookAtRH(position, target_, WorldUp());
    const auto cameraToWorld = glm::inverse(glm::mat3{view});
    const auto orientation = glm::normalize(glm::quat_cast(cameraToWorld));

    CameraState state;
    state.position = {position.x, position.y, position.z};
    state.orientation = {orientation.x, orientation.y, orientation.z, orientation.w};
    state.target = {target_.x, target_.y, target_.z};
    state.orbitCenter = {orbitCenter_.x, orbitCenter_.y, orbitCenter_.z};
    state.hasOrbitCenter = true;
    state.fovDegrees = fovDegrees_;
    state.nearPlane = nearPlane_;
    state.farPlane = farPlane_;
    return state;
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

void OrbitCamera::ApplyPositionTarget(glm::vec3 position, glm::vec3 target) {
    const glm::vec3 toTarget = target - position;
    const float requestedDistance = glm::length(toTarget);
    if (requestedDistance <= 1.0e-6F) {
        return;
    }

    const glm::vec3 direction = glm::normalize(toTarget);
    position_ = position;
    target_ = target;
    distance_ = std::max(minimumDistance_, glm::length(orbitCenter_ - position_));
    yawRadians_ = std::atan2(direction.y, direction.x);
    pitchRadians_ = std::clamp(std::asin(std::clamp(direction.z, -1.0F, 1.0F)), -kPitchLimit, kPitchLimit);
    UpdateClippingPlanes();
}

glm::vec3 OrbitCamera::WorldUp() const {
    return {0.0F, 0.0F, 1.0F};
}

glm::vec3 OrbitCamera::Position() const {
    return position_;
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
    orbitCenter_ = target_;

    const auto extent = maximum - minimum;
    framedRadius_ = std::max(0.5F * glm::length(extent), 0.25F);
    const auto verticalDistance = framedRadius_ / std::tan(glm::radians(fovDegrees_) * 0.5F);
    const auto horizontalFov = 2.0F * std::atan(std::tan(glm::radians(fovDegrees_) * 0.5F) * EffectiveAspectRatio(aspectRatio));
    const auto horizontalDistance = framedRadius_ / std::tan(horizontalFov * 0.5F);
    distance_ = std::max(verticalDistance, horizontalDistance) * 1.35F;
    const auto cosPitch = std::cos(pitchRadians_);
    const auto offset = glm::vec3{
        std::cos(yawRadians_) * cosPitch,
        std::sin(yawRadians_) * cosPitch,
        std::sin(pitchRadians_),
    };
    position_ = target_ - (glm::normalize(offset) * distance_);
    minimumDistance_ = std::max(0.0005F, framedRadius_ * 0.00005F);
    UpdateClippingPlanes();
}

void OrbitCamera::UpdateClippingPlanes() {
    const auto safeRadius = std::max(framedRadius_, 0.25F);
    nearPlane_ = std::max(0.0001F, std::min(safeRadius * 0.0005F, distance_ * 0.2F));
    farPlane_ = std::max(nearPlane_ + 1.0F, (safeRadius * 60.0F) + (distance_ * 4.0F));
}

}  // namespace invisible_places::camera
