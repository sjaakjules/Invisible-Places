#pragma once

#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <array>

namespace invisible_places::renderer::pointcloud {

[[nodiscard]] std::array<float, 3> SampleColormap(PointCloudColormapId colormap, float value);

}  // namespace invisible_places::renderer::pointcloud
