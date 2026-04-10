#pragma once

#include "io/PlyHeader.hpp"
#include "io/TransformMatrix.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::io {

struct DiscoveryIssue {
    std::filesystem::path filePath;
    std::string message;
};

struct PointCloudAsset {
    std::filesystem::path filePath;
    PlyHeader header;
};

struct GaussianSplatAsset {
    std::filesystem::path filePath;
    std::filesystem::path transformPath;
    PlyHeader header;
    Matrix4d localToWorld;
};

struct AssetCatalog {
    std::filesystem::path dataRoot;
    std::vector<PointCloudAsset> pointClouds;
    std::vector<GaussianSplatAsset> gaussianSplats;
    std::vector<DiscoveryIssue> issues;

    [[nodiscard]] std::string Summary() const;
};

AssetCatalog DiscoverAssets(const std::filesystem::path& dataRoot);

}  // namespace invisible_places::io

