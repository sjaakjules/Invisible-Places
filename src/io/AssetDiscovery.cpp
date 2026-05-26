#include "io/AssetDiscovery.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <sstream>
#include <string_view>

namespace invisible_places::io {

namespace {

bool StartsWith(std::string_view value, std::string_view prefix) {
    return value.substr(0, prefix.size()) == prefix;
}

std::string LowercaseCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::vector<std::filesystem::path> SortedDataFiles(const std::filesystem::path& root) {
    std::vector<std::filesystem::path> files;
    if (!std::filesystem::exists(root)) {
        return files;
    }

    for (const auto& entry : std::filesystem::recursive_directory_iterator{
             root,
             std::filesystem::directory_options::skip_permission_denied}) {
        if (!entry.is_regular_file()) {
            continue;
        }

        files.push_back(entry.path());
    }

    std::sort(files.begin(), files.end());
    return files;
}

}  // namespace

std::string AssetCatalog::Summary() const {
    std::ostringstream output;
    output << "Asset discovery summary\n";
    output << "- point clouds: " << pointClouds.size() << "\n";
    output << "- gaussian splats: " << gaussianSplats.size() << "\n";
    output << "- discovery issues: " << issues.size() << "\n";

    if (!pointClouds.empty()) {
        output << "- point-cloud files\n";
        for (const auto& asset : pointClouds) {
            output << "  * " << asset.filePath.filename().string() << " (" << asset.header.vertexCount
                   << " vertices";

            const auto scalarFields = asset.header.ScalarFieldNames();
            if (!scalarFields.empty()) {
                output << ", scalar fields: ";
                for (std::size_t index = 0; index < scalarFields.size(); ++index) {
                    if (index > 0) {
                        output << ", ";
                    }
                    output << scalarFields[index];
                }
            }

            output << ")\n";
        }
    }

    if (!gaussianSplats.empty()) {
        output << "- gaussian splat files\n";
        for (const auto& asset : gaussianSplats) {
            output << "  * " << asset.filePath.filename().string() << " (" << asset.header.vertexCount
                   << " gaussians, transform: " << asset.transformPath.filename().string() << ")\n";
        }
    }

    return output.str();
}

AssetCatalog DiscoverAssets(const std::filesystem::path& dataRoot) {
    AssetCatalog catalog;
    catalog.dataRoot = dataRoot;

    if (!std::filesystem::exists(dataRoot)) {
        catalog.issues.push_back(
            {.filePath = dataRoot, .message = "Data directory does not exist."});
        return catalog;
    }

    std::map<std::filesystem::path, std::map<std::string, std::filesystem::path>> transformsByDirectoryAndStem;
    const auto files = SortedDataFiles(dataRoot);

    for (const auto& filePath : files) {
        if (filePath.extension() == ".txt") {
            transformsByDirectoryAndStem[filePath.parent_path()][LowercaseCopy(filePath.stem().string())] =
                filePath;
        }
    }

    for (const auto& filePath : files) {
        if (filePath.extension() != ".ply") {
            continue;
        }

        const auto headerResult = ParsePlyHeader(filePath);
        if (!headerResult.success) {
            catalog.issues.push_back(
                {.filePath = filePath, .message = "PLY header parse failed: " + headerResult.errorMessage});
            continue;
        }

        const bool filenameHintsGsplat = StartsWith(filePath.filename().string(), "gSplat-");
        const bool looksLikeGsplat = filenameHintsGsplat || headerResult.header.LooksLikeGaussianSplat();

        if (looksLikeGsplat) {
            const auto directoryIt = transformsByDirectoryAndStem.find(filePath.parent_path());
            if (directoryIt == transformsByDirectoryAndStem.end()) {
                catalog.issues.push_back(
                    {.filePath = filePath, .message = "Missing same-stem transform matrix .txt file."});
                continue;
            }

            const auto transformIt = directoryIt->second.find(LowercaseCopy(filePath.stem().string()));
            if (transformIt == directoryIt->second.end()) {
                catalog.issues.push_back(
                    {.filePath = filePath, .message = "Missing same-stem transform matrix .txt file."});
                continue;
            }

            const auto matrixResult = ParseMatrix4x4(transformIt->second);
            if (!matrixResult.success) {
                catalog.issues.push_back(
                    {.filePath = transformIt->second, .message = matrixResult.errorMessage});
                continue;
            }

            catalog.gaussianSplats.push_back(
                {.filePath = filePath,
                 .transformPath = transformIt->second,
                 .header = headerResult.header,
                 .localToWorld = matrixResult.matrix});
            continue;
        }

        if (headerResult.header.LooksLikePointCloud()) {
            catalog.pointClouds.push_back({.filePath = filePath, .header = headerResult.header});
            continue;
        }

        catalog.issues.push_back(
            {.filePath = filePath, .message = "PLY did not match current point-cloud or gSplat heuristics."});
    }

    return catalog;
}

}  // namespace invisible_places::io
