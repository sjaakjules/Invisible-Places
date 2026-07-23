#pragma once

#include "scene/PointCloudVariants.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
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

// Discovers the one qualifying 5 mm MESH-sampled Ground point cloud in a
// scene-local directory. Ambiguity is an error: callers must never bind the
// lexically first candidate to an otherwise unrelated active scene.
[[nodiscard]] std::optional<std::filesystem::path> FindSampledGroundSurfaceInDirectory(
    const std::filesystem::path& directory,
    std::string* errorMessage = nullptr);

class SceneCatalog {
  public:
    static SceneCatalog FromDiscoveredAssets(const invisible_places::io::AssetCatalog& assetCatalog);

    [[nodiscard]] const std::vector<LayerSummary>& Layers() const { return layers_; }
    [[nodiscard]] const std::vector<ScenePointCloudGroup>& PointCloudGroups() const {
        return pointCloudGroups_;
    }
    // Scene names are expected to be unique for application-authored data. If
    // two different folders share a name, the name-only lookup deliberately
    // returns null so callers cannot silently bind the wrong density set.
    [[nodiscard]] const ScenePointCloudGroup* FindPointCloudGroup(std::string_view name) const;
    [[nodiscard]] const ScenePointCloudGroup* FindPointCloudGroupBySourceFolder(
        const std::filesystem::path& sourceFolder) const;
    [[nodiscard]] std::string Summary() const;

  private:
    std::vector<LayerSummary> layers_;
    std::vector<ScenePointCloudGroup> pointCloudGroups_;
};

}  // namespace invisible_places::scene
