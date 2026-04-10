#include "scene/SceneCatalog.hpp"

#include <sstream>

namespace invisible_places::scene {

SceneCatalog SceneCatalog::FromDiscoveredAssets(const invisible_places::io::AssetCatalog& assetCatalog) {
    SceneCatalog catalog;

    for (const auto& pointCloud : assetCatalog.pointClouds) {
        catalog.layers_.push_back(
            {.kind = LayerKind::PointCloud,
             .name = pointCloud.filePath.stem().string(),
             .sourcePath = pointCloud.filePath,
             .primitiveCount = pointCloud.header.vertexCount});
    }

    for (const auto& gsplat : assetCatalog.gaussianSplats) {
        catalog.layers_.push_back(
            {.kind = LayerKind::GaussianSplat,
             .name = gsplat.filePath.stem().string(),
             .sourcePath = gsplat.filePath,
             .auxiliaryPath = gsplat.transformPath,
             .primitiveCount = gsplat.header.vertexCount});
    }

    return catalog;
}

std::string SceneCatalog::Summary() const {
    std::ostringstream output;
    output << "Scene catalog summary\n";
    output << "- layers: " << layers_.size() << "\n";

    if (!layers_.empty()) {
        output << "- layer order\n";
        for (const auto& layer : layers_) {
            output << "  * " << (layer.kind == LayerKind::PointCloud ? "point-cloud" : "gsplat") << ": "
                   << layer.name << " (" << layer.primitiveCount << " primitives)\n";
        }
    }

    return output.str();
}

}  // namespace invisible_places::scene

