#pragma once

#include "io/PointCloudData.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace invisible_places::app::manual_flow_path {

struct Ray {
    glm::vec3 origin{0.0F, 0.0F, 0.0F};
    glm::vec3 direction{0.0F, 0.0F, -1.0F};
};

inline std::optional<glm::vec3> IntersectRayPlane(
    const Ray& ray,
    const glm::vec3& planePoint,
    const glm::vec3& planeNormal) {
    const float denominator = glm::dot(ray.direction, planeNormal);
    if (std::abs(denominator) <= 1.0e-5F) {
        return std::nullopt;
    }
    const float distance = glm::dot(planePoint - ray.origin, planeNormal) / denominator;
    return ray.origin + ray.direction * distance;
}

inline std::optional<float> ClosestRayAxisParameter(
    const Ray& ray,
    const glm::vec3& axisOrigin,
    const glm::vec3& axis) {
    const float rayLength = glm::length(ray.direction);
    const float axisLength = glm::length(axis);
    if (rayLength <= 1.0e-6F || axisLength <= 1.0e-6F) {
        return std::nullopt;
    }
    const glm::vec3 rayDirection = ray.direction / rayLength;
    const glm::vec3 axisDirection = axis / axisLength;
    const float parallelAmount = glm::dot(rayDirection, axisDirection);
    const float denominator = 1.0F - parallelAmount * parallelAmount;
    if (denominator <= 1.0e-4F) {
        return std::nullopt;
    }
    const glm::vec3 offset = ray.origin - axisOrigin;
    return (glm::dot(axisDirection, offset) -
            parallelAmount * glm::dot(rayDirection, offset)) /
           denominator;
}

inline glm::vec3 ProjectedAxisDragPoint(
    const glm::vec3& startPoint,
    const glm::vec3& axis,
    const glm::vec2& startMouse,
    const glm::vec2& currentMouse,
    const glm::vec2& axisScreenDirection,
    float pixelsPerWorldUnit) {
    const glm::vec2 mouseDelta = currentMouse - startMouse;
    const float projectedPixels = glm::dot(mouseDelta, axisScreenDirection);
    return startPoint + axis * (projectedPixels / std::max(1.0F, pixelsPerWorldUnit));
}

inline std::optional<std::size_t> InsertControlPoint(
    std::vector<invisible_places::io::Float3>* controlPoints,
    std::size_t segmentIndex,
    const invisible_places::io::Float3& point) {
    if (controlPoints == nullptr ||
        controlPoints->size() < 2U ||
        segmentIndex + 1U >= controlPoints->size()) {
        return std::nullopt;
    }
    const std::size_t insertIndex = segmentIndex + 1U;
    controlPoints->insert(
        controlPoints->begin() + static_cast<std::ptrdiff_t>(insertIndex),
        point);
    return insertIndex;
}

}  // namespace invisible_places::app::manual_flow_path
