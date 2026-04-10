#pragma once

#include "io/AssetDiscovery.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::scene {

enum class LayerKind {
    PointCloud,
    GaussianSplat
};

struct LayerSummary {
    LayerKind kind = LayerKind::PointCloud;
    std::string name;
    std::filesystem::path sourcePath;
    std::filesystem::path auxiliaryPath;
    std::uint64_t primitiveCount = 0;
};

class SceneCatalog {
  public:
    static SceneCatalog FromDiscoveredAssets(const invisible_places::io::AssetCatalog& assetCatalog);

    [[nodiscard]] const std::vector<LayerSummary>& Layers() const { return layers_; }
    [[nodiscard]] std::string Summary() const;

  private:
    std::vector<LayerSummary> layers_;
};

}  // namespace invisible_places::scene

