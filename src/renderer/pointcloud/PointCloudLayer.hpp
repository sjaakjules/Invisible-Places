#pragma once

#include "style/RenderParameterBinding.hpp"

#include <string>
#include <vector>

namespace invisible_places::renderer::pointcloud {

struct PointCloudLayer {
    std::string layerName;
    bool visible = true;
    bool useSourceRgbByDefault = true;
    std::vector<std::string> availableScalarFields;
    invisible_places::style::RenderParameterBinding pointSize;
    invisible_places::style::RenderParameterBinding opacity;
};

}  // namespace invisible_places::renderer::pointcloud

