#pragma once

#include "camera/CameraState.hpp"
#include "io/PointCloudData.hpp"

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace invisible_places::camera {

struct OrbitCameraMatrices {
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::mat4 viewProjection{1.0F};
    glm::vec3 position{0.0F, 0.0F, 1.0F};
};

class OrbitCamera {
  public:
    void FrameBounds(const invisible_places::io::Bounds3f& bounds, float aspectRatio);
    void FrameBounds(
        const invisible_places::io::Bounds3f& bounds,
        const invisible_places::io::Float3& focusPoint,
        float aspectRatio);
    void ResetToFramedBounds(float aspectRatio);
    void Orbit(float deltaX, float deltaY);
    void Pan(float deltaX, float deltaY, float viewportWidth, float viewportHeight);
    void Dolly(float wheelDelta);
    void SetTargetPreservingPosition(const glm::vec3& target);
    void SetOrbitCenterPreservingView(const glm::vec3& center);
    void ApplyState(const CameraState& state);

    [[nodiscard]] CameraState CaptureState() const;
    [[nodiscard]] OrbitCameraMatrices Matrices(float aspectRatio) const;
    [[nodiscard]] glm::vec3 Target() const { return target_; }
    [[nodiscard]] glm::vec3 OrbitCenter() const { return orbitCenter_; }
    [[nodiscard]] float Distance() const { return distance_; }
    [[nodiscard]] float FovDegrees() const { return fovDegrees_; }
    [[nodiscard]] float NearPlane() const { return nearPlane_; }
    [[nodiscard]] float FarPlane() const { return farPlane_; }
    [[nodiscard]] bool HasDepthOfField() const { return hasDepthOfField_; }
    [[nodiscard]] float FocusDistance() const { return focusDistance_; }
    [[nodiscard]] float ApertureFStops() const { return apertureFStops_; }
    [[nodiscard]] float DepthOfFieldMaxBlurPixels() const { return depthOfFieldMaxBlurPixels_; }
    [[nodiscard]] bool HasFramedBounds() const { return framedBounds_.valid; }

  private:
    void ApplyPositionTarget(glm::vec3 position, glm::vec3 target);
    [[nodiscard]] glm::vec3 WorldUp() const;
    [[nodiscard]] glm::vec3 Position() const;
    [[nodiscard]] glm::vec3 Forward() const;
    [[nodiscard]] glm::vec3 Right() const;
    [[nodiscard]] glm::vec3 Up() const;
    [[nodiscard]] float EffectiveAspectRatio(float aspectRatio) const;
    void UpdateClippingPlanes();
    void ApplyFramedBounds(float aspectRatio);

    invisible_places::io::Bounds3f framedBounds_{};
    glm::vec3 framedFocusPoint_{0.0F, 0.0F, 0.0F};
    glm::vec3 position_{-3.085F, -2.338F, 2.613F};
    glm::vec3 target_{0.0F, 0.0F, 0.0F};
    glm::vec3 orbitCenter_{0.0F, 0.0F, 0.0F};
    float framedRadius_ = 1.0F;
    float minimumDistance_ = 0.0005F;
    float distance_ = 5.0F;
    float yawRadians_ = 0.65F;
    float pitchRadians_ = -0.55F;
    float fovDegrees_ = 55.0F;
    float nearPlane_ = 0.05F;
    float farPlane_ = 1000.0F;
    bool hasDepthOfField_ = false;
    float focusDistance_ = 1.0F;
    float apertureFStops_ = 8.0F;
    float depthOfFieldMaxBlurPixels_ = 24.0F;
};

}  // namespace invisible_places::camera
