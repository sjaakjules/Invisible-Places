#include "scene/SceneCatalog.hpp"

#include "io/PlyHeader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace invisible_places::scene {

namespace {

std::string LowercaseAsciiCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::string NormalizePathKey(const std::filesystem::path& path) {
    return path.lexically_normal().generic_string();
}

}  // namespace

std::optional<std::filesystem::path> FindSampledGroundSurfaceInDirectory(
    const std::filesystem::path& directory,
    std::string* errorMessage) {
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    if (directory.empty()) {
        return std::nullopt;
    }
    std::error_code directoryError;
    if (!std::filesystem::is_directory(directory, directoryError) || directoryError) {
        return std::nullopt;
    }

    std::vector<std::filesystem::path> candidates;
    for (std::filesystem::directory_iterator iterator{
             directory,
             std::filesystem::directory_options::skip_permission_denied,
             directoryError};
         !directoryError && iterator != std::filesystem::directory_iterator{};
         iterator.increment(directoryError)) {
        std::error_code entryError;
        if (!iterator->is_regular_file(entryError) || entryError ||
            LowercaseAsciiCopy(iterator->path().extension().string()) != ".ply") {
            continue;
        }
        std::string compactStem;
        for (const char character : LowercaseAsciiCopy(iterator->path().stem().string())) {
            if (std::isalnum(static_cast<unsigned char>(character)) != 0) {
                compactStem.push_back(character);
            }
        }
        if (compactStem.find("meshsampled") == std::string::npos ||
            std::fabs(
                invisible_places::io::InferPointSpacingMetersFromName(
                    iterator->path().stem().string()) -
                0.005F) >
                1.0e-6F) {
            continue;
        }
        const auto header = invisible_places::io::ParsePlyHeader(iterator->path());
        if (!header.success || header.header.faceCount != 0U ||
            !header.header.LooksLikePointCloud() ||
            !header.header.HasProperty("nx") ||
            !header.header.HasProperty("ny") ||
            !header.header.HasProperty("nz")) {
            continue;
        }
        const auto scalarFields = header.header.ScalarFieldNames();
        const bool hasDip = std::any_of(
            scalarFields.begin(),
            scalarFields.end(),
            [](const std::string& name) {
                return LowercaseAsciiCopy(name).find("dip") != std::string::npos;
            });
        const bool hasDirection = std::any_of(
            scalarFields.begin(),
            scalarFields.end(),
            [](const std::string& name) {
                return LowercaseAsciiCopy(name).find("direction") != std::string::npos;
            });
        if (hasDip && hasDirection) {
            candidates.push_back(iterator->path());
        }
    }
    if (candidates.empty()) {
        return std::nullopt;
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right) {
        return NormalizePathKey(left) < NormalizePathKey(right);
    });
    if (candidates.size() != 1U) {
        if (errorMessage != nullptr) {
            *errorMessage =
                "Multiple qualifying 5 mm MESH-sampled Ground clouds were "
                "found in " +
                directory.generic_string() +
                "; select one scene-local source instead of relying on "
                "discovery order.";
        }
        return std::nullopt;
    }
    return candidates.front();
}

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
