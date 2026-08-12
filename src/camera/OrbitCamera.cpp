#include "camera/OrbitCamera.hpp"

#include <algorithm>
#include <cmath>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>
#include <glm/trigonometric.hpp>

namespace invisible_places::camera {

namespace {

constexpr float kPitchLimit = 1.45F;
constexpr float kPi = 3.14159265358979323846F;

glm::vec3 ToGlm(const invisible_places::io::Float3& value) {
    return {value.x, value.y, value.z};
}

glm::vec3 TrackballVector(
    float x,
    float y,
    float viewportWidth,
    float viewportHeight,
    float pivotScreenX,
    float pivotScreenY) {
    const float safeWidth = std::max(1.0F, viewportWidth);
    const float safeHeight = std::max(1.0F, viewportHeight);
    const float centerX = std::clamp(pivotScreenX, safeWidth * 0.25F, safeWidth * 0.75F);
    const float centerY = std::clamp(pivotScreenY, safeHeight * 0.25F, safeHeight * 0.75F);
    glm::vec3 result{
        std::clamp((x - centerX) / (safeWidth * 0.5F), -1.0F, 1.0F),
        std::clamp((centerY - y) / (safeHeight * 0.5F), -1.0F, 1.0F),
        0.0F,
    };
    const float radiusSquared = (result.x * result.x) + (result.y * result.y);
    if (radiusSquared > 1.0F) {
        const float inverseRadius = 1.0F / std::sqrt(radiusSquared);
        result.x *= inverseRadius;
        result.y *= inverseRadius;
    } else {
        result.z = std::sqrt(std::max(0.0F, 1.0F - radiusSquared));
    }
    return result;
}

glm::quat ShortestArcRotation(const glm::vec3& from, const glm::vec3& to) {
    const glm::vec3 normalizedFrom = glm::normalize(from);
    const glm::vec3 normalizedTo = glm::normalize(to);
    const float cosine = std::clamp(glm::dot(normalizedFrom, normalizedTo), -1.0F, 1.0F);
    if (cosine >= 1.0F - 1.0e-6F) {
        return {1.0F, 0.0F, 0.0F, 0.0F};
    }
    if (cosine <= -1.0F + 1.0e-6F) {
        glm::vec3 axis = glm::cross(normalizedFrom, glm::vec3{1.0F, 0.0F, 0.0F});
        if (glm::length(axis) <= 1.0e-5F) {
            axis = glm::cross(normalizedFrom, glm::vec3{0.0F, 1.0F, 0.0F});
        }
        return glm::angleAxis(kPi, glm::normalize(axis));
    }

    const glm::vec3 axis = glm::cross(normalizedFrom, normalizedTo);
    const float scale = std::sqrt((1.0F + cosine) * 2.0F);
    const float inverseScale = 1.0F / scale;
    return glm::normalize(glm::quat{
        scale * 0.5F,
        axis.x * inverseScale,
        axis.y * inverseScale,
        axis.z * inverseScale,
    });
}

}  // namespace

std::optional<ClippedNdcSegment> ProjectWorldSegmentToNdc(
    const glm::mat4& viewProjection,
    const glm::vec3& worldStart,
    const glm::vec3& worldEnd) {
    const glm::vec4 clipStart =
        viewProjection * glm::vec4{worldStart, 1.0F};
    const glm::vec4 clipEnd =
        viewProjection * glm::vec4{worldEnd, 1.0F};
    const auto finiteClipPoint = [](const glm::vec4& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z) && std::isfinite(point.w);
    };
    if (!finiteClipPoint(clipStart) || !finiteClipPoint(clipEnd)) {
        return std::nullopt;
    }

    float firstT = 0.0F;
    float lastT = 1.0F;
    const auto clipToHalfSpace = [&](float startDistance,
                                     float endDistance) {
        if (startDistance < 0.0F && endDistance < 0.0F) {
            return false;
        }
        if (startDistance >= 0.0F && endDistance >= 0.0F) {
            return true;
        }

        const float denominator = startDistance - endDistance;
        if (!std::isfinite(denominator)) {
            return false;
        }
        const float intersectionT = startDistance / denominator;
        if (startDistance < 0.0F) {
            firstT = std::max(firstT, intersectionT);
        } else {
            lastT = std::min(lastT, intersectionT);
        }
        return firstT <= lastT;
    };

    // GLM's projection matrices in this application use the OpenGL clip
    // volume: -w <= x,y,z <= w. The positive-w half-space avoids attempting
    // a perspective divide at the camera origin.
    constexpr float kMinimumClipW = 1.0e-6F;
    constexpr float kClippedW = 2.0F * kMinimumClipW;
    if (!clipToHalfSpace(
            clipStart.w - kClippedW,
            clipEnd.w - kClippedW) ||
        !clipToHalfSpace(
            clipStart.x + clipStart.w,
            clipEnd.x + clipEnd.w) ||
        !clipToHalfSpace(
            clipStart.w - clipStart.x,
            clipEnd.w - clipEnd.x) ||
        !clipToHalfSpace(
            clipStart.y + clipStart.w,
            clipEnd.y + clipEnd.w) ||
        !clipToHalfSpace(
            clipStart.w - clipStart.y,
            clipEnd.w - clipEnd.y) ||
        !clipToHalfSpace(
            clipStart.z + clipStart.w,
            clipEnd.z + clipEnd.w) ||
        !clipToHalfSpace(
            clipStart.w - clipStart.z,
            clipEnd.w - clipEnd.z)) {
        return std::nullopt;
    }

    const glm::vec4 clipDelta = clipEnd - clipStart;
    const glm::vec4 clippedStart = clipStart + clipDelta * firstT;
    const glm::vec4 clippedEnd = clipStart + clipDelta * lastT;
    if (clippedStart.w <= kMinimumClipW || clippedEnd.w <= kMinimumClipW) {
        return std::nullopt;
    }

    const auto toNdc = [](const glm::vec4& clipPoint) {
        const glm::vec3 ndc = glm::vec3{clipPoint} / clipPoint.w;
        return glm::clamp(
            ndc,
            glm::vec3{-1.0F},
            glm::vec3{1.0F});
    };
    const glm::vec3 ndcStart = toNdc(clippedStart);
    const glm::vec3 ndcEnd = toNdc(clippedEnd);
    if (!std::isfinite(ndcStart.x) || !std::isfinite(ndcStart.y) ||
        !std::isfinite(ndcStart.z) || !std::isfinite(ndcEnd.x) ||
        !std::isfinite(ndcEnd.y) || !std::isfinite(ndcEnd.z)) {
        return std::nullopt;
    }
    return ClippedNdcSegment{.start = ndcStart, .end = ndcEnd};
}

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
    FrameBounds(bounds, focusPoint, aspectRatio, 1.0F);
}

void OrbitCamera::FrameBounds(
    const invisible_places::io::Bounds3f& bounds,
    const invisible_places::io::Float3& focusPoint,
    float aspectRatio,
    float distanceScale) {
    if (!bounds.valid) {
        return;
    }

    framedBounds_ = bounds;
    framedFocusPoint_ = ToGlm(focusPoint);
    ApplyFramedBounds(aspectRatio, distanceScale);
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

void OrbitCamera::OrbitTrackball(
    float previousX,
    float previousY,
    float currentX,
    float currentY,
    float viewportWidth,
    float viewportHeight,
    float pivotScreenX,
    float pivotScreenY) {
    if ((previousX == currentX && previousY == currentY) ||
        viewportWidth <= 0.0F || viewportHeight <= 0.0F) {
        return;
    }

    const glm::vec3 previous = TrackballVector(
        previousX,
        previousY,
        viewportWidth,
        viewportHeight,
        pivotScreenX,
        pivotScreenY);
    const glm::vec3 current = TrackballVector(
        currentX,
        currentY,
        viewportWidth,
        viewportHeight,
        pivotScreenX,
        pivotScreenY);
    const glm::quat viewSpaceRotation = ShortestArcRotation(previous, current);
    const glm::quat localCameraRotation = glm::conjugate(viewSpaceRotation);
    const glm::quat oldOrientation = Orientation();
    const glm::quat worldRotation =
        glm::normalize(oldOrientation * localCameraRotation * glm::conjugate(oldOrientation));
    const auto rotateAroundPivot = [&](const glm::vec3& point) {
        return orbitCenter_ + (worldRotation * (point - orbitCenter_));
    };

    position_ = rotateAroundPivot(position_);
    target_ = rotateAroundPivot(target_);
    explicitOrientation_ = glm::normalize(oldOrientation * localCameraRotation);
    hasExplicitOrientation_ = true;
    distance_ = std::max(minimumDistance_, glm::length(orbitCenter_ - position_));
    const auto forwardForAngles = Forward();
    yawRadians_ = std::atan2(forwardForAngles.y, forwardForAngles.x);
    pitchRadians_ = std::clamp(
        std::asin(std::clamp(forwardForAngles.z, -1.0F, 1.0F)),
        -kPitchLimit,
        kPitchLimit);
    UpdateClippingPlanes();
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

    auto pivotOffset = position_ - orbitCenter_;
    if (glm::length(pivotOffset) <= 1.0e-6F) {
        pivotOffset = -Forward() * std::max(distance_, minimumDistance_);
    }

    const float currentDistance = std::max(minimumDistance_, glm::length(pivotOffset));
    const float clampedWheelDelta = std::clamp(wheelDelta, -8.0F, 8.0F);
    constexpr float kZoomStep = 0.075F;
    const float distanceScale = std::exp(-clampedWheelDelta * kZoomStep);
    const float nextDistance = std::max(minimumDistance_, currentDistance * distanceScale);
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

void OrbitCamera::SetViewDirectionAroundOrbitCenter(
    const glm::vec3& viewDirection,
    const glm::vec3& screenUpDirection) {
    if (glm::length(viewDirection) <= 1.0e-6F) {
        return;
    }

    const glm::vec3 forward = glm::normalize(viewDirection);
    glm::vec3 cameraUp =
        screenUpDirection -
        (forward * glm::dot(screenUpDirection, forward));
    if (glm::length(cameraUp) <= 1.0e-6F) {
        const glm::vec3 fallbackUp =
            std::abs(forward.z) < 0.9F
                ? glm::vec3{0.0F, 0.0F, 1.0F}
                : glm::vec3{1.0F, 0.0F, 0.0F};
        cameraUp = fallbackUp -
                   (forward * glm::dot(fallbackUp, forward));
    }
    cameraUp = glm::normalize(cameraUp);
    const glm::vec3 cameraRight =
        glm::normalize(glm::cross(forward, cameraUp));
    cameraUp = glm::normalize(glm::cross(cameraRight, forward));

    const float orbitDistance = std::max(
        minimumDistance_,
        glm::length(position_ - orbitCenter_));
    position_ = orbitCenter_ - (forward * orbitDistance);
    target_ = orbitCenter_;
    distance_ = orbitDistance;

    const glm::mat3 cameraToWorld{
        cameraRight,
        cameraUp,
        -forward,
    };
    explicitOrientation_ =
        glm::normalize(glm::quat_cast(cameraToWorld));
    hasExplicitOrientation_ = true;
    yawRadians_ = std::atan2(forward.y, forward.x);
    pitchRadians_ = std::clamp(
        std::asin(std::clamp(forward.z, -1.0F, 1.0F)),
        -kPitchLimit,
        kPitchLimit);
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

    fovDegrees_ = std::clamp(state.fovDegrees, 1.0F, 160.0F);
    position_ = position;
    target_ = target;
    orbitCenter_ = orbitCenter;
    distance_ = std::max(minimumDistance_, glm::length(orbitCenter_ - position_));
    const auto forwardForAngles = glm::normalize(target_ - position_);
    yawRadians_ = std::atan2(forwardForAngles.y, forwardForAngles.x);
    pitchRadians_ = std::clamp(
        std::asin(std::clamp(forwardForAngles.z, -1.0F, 1.0F)),
        -kPitchLimit,
        kPitchLimit);
    hasExplicitOrientation_ = orientationLengthSquared > 1.0e-8F;
    if (hasExplicitOrientation_) {
        explicitOrientation_ = glm::normalize(storedOrientation);
    }
    nearPlane_ = std::max(0.0001F, state.nearPlane);
    farPlane_ = std::max(nearPlane_ + 1.0F, state.farPlane);
    hasDepthOfField_ = state.hasDepthOfField;
    focusDistance_ = std::max(0.001F, state.focusDistance);
    apertureFStops_ = std::max(0.1F, state.apertureFStops);
    depthOfFieldMaxBlurPixels_ = std::max(0.0F, state.depthOfFieldMaxBlurPixels);
}

CameraState OrbitCamera::CaptureState() const {
    const auto position = Position();
    const auto orientation = Orientation();

    CameraState state;
    state.position = {position.x, position.y, position.z};
    state.orientation = {orientation.x, orientation.y, orientation.z, orientation.w};
    state.target = {target_.x, target_.y, target_.z};
    state.orbitCenter = {orbitCenter_.x, orbitCenter_.y, orbitCenter_.z};
    state.hasOrbitCenter = true;
    state.hasDepthOfField = hasDepthOfField_;
    state.fovDegrees = fovDegrees_;
    state.nearPlane = nearPlane_;
    state.farPlane = farPlane_;
    state.focusDistance = focusDistance_;
    state.apertureFStops = apertureFStops_;
    state.depthOfFieldMaxBlurPixels = depthOfFieldMaxBlurPixels_;
    return state;
}

OrbitCameraMatrices OrbitCamera::Matrices(float aspectRatio) const {
    OrbitCameraMatrices matrices;
    matrices.position = Position();
    if (hasExplicitOrientation_) {
        const auto rotation = glm::mat4_cast(glm::conjugate(glm::normalize(explicitOrientation_)));
        const auto translation = glm::translate(glm::mat4{1.0F}, -matrices.position);
        matrices.view = rotation * translation;
    } else {
        matrices.view = glm::lookAtRH(matrices.position, target_, WorldUp());
    }
    const float effectiveAspectRatio = EffectiveAspectRatio(aspectRatio);
    if (parallelProjection_) {
        const float verticalHalfExtent = std::max(
            minimumDistance_,
            distance_ * std::tan(glm::radians(fovDegrees_) * 0.5F));
        const float horizontalHalfExtent =
            verticalHalfExtent * effectiveAspectRatio;
        matrices.projection = glm::ortho(
            -horizontalHalfExtent,
            horizontalHalfExtent,
            -verticalHalfExtent,
            verticalHalfExtent,
            nearPlane_,
            farPlane_);
    } else {
        matrices.projection = glm::perspective(
            glm::radians(fovDegrees_),
            effectiveAspectRatio,
            nearPlane_,
            farPlane_);
    }
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
    hasExplicitOrientation_ = false;
    UpdateClippingPlanes();
}

glm::vec3 OrbitCamera::WorldUp() const {
    return {0.0F, 0.0F, 1.0F};
}

glm::vec3 OrbitCamera::Position() const {
    return position_;
}

glm::vec3 OrbitCamera::Forward() const {
    if (hasExplicitOrientation_) {
        return glm::normalize(explicitOrientation_) * glm::vec3{0.0F, 0.0F, -1.0F};
    }
    return glm::normalize(target_ - Position());
}

glm::vec3 OrbitCamera::Right() const {
    if (hasExplicitOrientation_) {
        return glm::normalize(explicitOrientation_) * glm::vec3{1.0F, 0.0F, 0.0F};
    }
    return glm::normalize(glm::cross(Forward(), WorldUp()));
}

glm::vec3 OrbitCamera::Up() const {
    if (hasExplicitOrientation_) {
        return glm::normalize(explicitOrientation_) * glm::vec3{0.0F, 1.0F, 0.0F};
    }
    return glm::normalize(glm::cross(Right(), Forward()));
}

glm::quat OrbitCamera::Orientation() const {
    if (hasExplicitOrientation_) {
        return glm::normalize(explicitOrientation_);
    }
    const auto view = glm::lookAtRH(position_, target_, WorldUp());
    const auto cameraToWorld = glm::inverse(glm::mat3{view});
    return glm::normalize(glm::quat_cast(cameraToWorld));
}

float OrbitCamera::EffectiveAspectRatio(float aspectRatio) const {
    return std::max(0.1F, aspectRatio);
}

void OrbitCamera::ApplyFramedBounds(float aspectRatio, float distanceScale) {
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
    distance_ = std::max(verticalDistance, horizontalDistance) * 1.35F *
                std::clamp(distanceScale, 0.05F, 4.0F);
    const auto cosPitch = std::cos(pitchRadians_);
    const auto offset = glm::vec3{
        std::cos(yawRadians_) * cosPitch,
        std::sin(yawRadians_) * cosPitch,
        std::sin(pitchRadians_),
    };
    position_ = target_ - (glm::normalize(offset) * distance_);
    hasExplicitOrientation_ = false;
    minimumDistance_ = std::max(0.0005F, framedRadius_ * 0.00005F);
    UpdateClippingPlanes();
}

void OrbitCamera::UpdateClippingPlanes() {
    const auto safeRadius = std::max(framedRadius_, 0.25F);
    nearPlane_ = std::max(0.0001F, std::min(safeRadius * 0.0005F, distance_ * 0.2F));
    farPlane_ = std::max(nearPlane_ + 1.0F, (safeRadius * 60.0F) + (distance_ * 4.0F));
}

}  // namespace invisible_places::camera
