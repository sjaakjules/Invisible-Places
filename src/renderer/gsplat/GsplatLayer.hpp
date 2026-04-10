#pragma once

#include "io/TransformMatrix.hpp"

#include <filesystem>
#include <string>

namespace invisible_places::renderer::gsplat {

struct GsplatLayer {
    std::string layerName;
    std::filesystem::path assetPath;
    invisible_places::io::Matrix4d localToWorld;
    bool visible = true;
};

}  // namespace invisible_places::renderer::gsplat

