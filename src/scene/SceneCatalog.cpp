#include "scene/SceneCatalog.hpp"

#include <algorithm>
#include <sstream>

namespace invisible_places::scene {

SceneCatalog SceneCatalog::FromDiscoveredAssets(const invisible_places::io::AssetCatalog& assetCatalog) {
    SceneCatalog catalog;
    catalog.pointCloudGroups_ = BuildScenePointCloudGroups(assetCatalog);

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

const ScenePointCloudGroup* SceneCatalog::FindPointCloudGroup(std::string_view name) const {
    const ScenePointCloudGroup* match = nullptr;
    for (const auto& group : pointCloudGroups_) {
        if (group.name != name) {
            continue;
        }
        if (match != nullptr) {
            return nullptr;
        }
        match = &group;
    }
    return match;
}

const ScenePointCloudGroup* SceneCatalog::FindPointCloudGroupBySourceFolder(
    const std::filesystem::path& sourceFolder) const {
    const auto normalizedFolder = sourceFolder.lexically_normal().generic_string();
    const auto match = std::find_if(
        pointCloudGroups_.begin(),
        pointCloudGroups_.end(),
        [&](const ScenePointCloudGroup& group) {
            return group.sourceFolder.lexically_normal().generic_string() == normalizedFolder;
        });
    return match == pointCloudGroups_.end() ? nullptr : &*match;
}

std::string SceneCatalog::Summary() const {
    std::ostringstream output;
    output << "Scene catalog summary\n";
    output << "- layers: " << layers_.size() << "\n";
    output << "- point-cloud scene groups: " << pointCloudGroups_.size() << "\n";

    if (!pointCloudGroups_.empty()) {
        output << "- scene density bundles\n";
        for (const auto& group : pointCloudGroups_) {
            output << "  * " << group.name << ": ";
            if (group.completeDisplayBundles.empty()) {
                output << "Mixed (no complete density bundle)";
            } else {
                for (std::size_t index = 0; index < group.completeDisplayBundles.size(); ++index) {
                    if (index > 0U) {
                        output << ", ";
                    }
                    const auto spacing = group.completeDisplayBundles[index].spacingMicrometres;
                    output << (spacing / 1'000U);
                    const auto fractionalMicrometres = spacing % 1'000U;
                    if (fractionalMicrometres != 0U) {
                        output << ".";
                        if (fractionalMicrometres < 100U) {
                            output << "0";
                        }
                        if (fractionalMicrometres < 10U) {
                            output << "0";
                        }
                        output << fractionalMicrometres;
                    }
                    output << " mm";
                }
            }
            output << "\n";
        }
    }

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
