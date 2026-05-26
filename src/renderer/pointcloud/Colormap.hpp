#pragma once

#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <array>

namespace invisible_places::renderer::pointcloud {

[[nodiscard]] std::array<float, 3> SampleColormap(PointCloudColormapId colormap, float value);
[[nodiscard]] std::array<float, 3> SampleGradient(
    const std::array<float, 3>& startColor,
    const std::array<float, 3>& endColor,
    float value);

}  // namespace invisible_places::renderer::pointcloud
