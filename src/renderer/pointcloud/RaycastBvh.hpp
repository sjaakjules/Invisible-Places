#pragma once

#include "io/PointCloudData.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace invisible_places::renderer::pointcloud {

struct RaycastBvhBuildPoint {
    invisible_places::io::Float3 center{};
    float radius = 0.0F;
    std::uint32_t pointIndex = 0;
};

struct alignas(16) RaycastBvhNodeGpu {
    std::array<float, 4> boundsMin{0.0F, 0.0F, 0.0F, 0.0F};
    std::array<float, 4> boundsMax{0.0F, 0.0F, 0.0F, 0.0F};
    std::array<std::uint32_t, 4> meta{0U, 0U, 0U, 0U};
};

struct LinearRaycastBvh {
    std::vector<RaycastBvhNodeGpu> nodes;
    std::vector<std::uint32_t> pointIndices;

    [[nodiscard]] bool Empty() const { return nodes.empty() || pointIndices.empty(); }
};

[[nodiscard]] LinearRaycastBvh BuildLinearRaycastBvh(
    const std::vector<RaycastBvhBuildPoint>& points,
    std::uint32_t maxLeafSize = 8U);

}  // namespace invisible_places::renderer::pointcloud
