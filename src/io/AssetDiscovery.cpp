#include "io/AssetDiscovery.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
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

bool TokenBoundary(char character) {
    return character == '\0' ||
           character == '-' ||
           character == '_' ||
           character == ' ' ||
           character == '.';
}

bool ContainsRoleToken(std::string_view lowerName, std::string_view token) {
    std::size_t cursor = 0;
    while (cursor < lowerName.size()) {
        const auto position = lowerName.find(token, cursor);
        if (position == std::string_view::npos) {
            return false;
        }
        const char before = position == 0U ? '\0' : lowerName[position - 1U];
        const char after = position + token.size() >= lowerName.size()
                               ? '\0'
                               : lowerName[position + token.size()];
        if (TokenBoundary(before) && TokenBoundary(after)) {
            return true;
        }
        cursor = position + 1U;
    }
    return false;
}

std::string SceneRoleFromStem(const std::filesystem::path& filePath) {
    return InferPointCloudSceneRoleFromName(filePath.stem().string());
}

float SceneSpacingFromStem(const std::filesystem::path& filePath) {
    return InferPointSpacingMetersFromName(filePath.stem().string());
}

bool ParentIsDiscoveryRoot(const std::filesystem::path& dataRoot, const std::filesystem::path& filePath) {
    std::error_code error;
    return std::filesystem::equivalent(filePath.parent_path(), dataRoot, error) && !error;
}

std::vector<std::filesystem::path> SortedDataFiles(const std::filesystem::path& root) {
    std::vector<std::filesystem::path> files;
    if (!std::filesystem::exists(root)) {
        return files;
    }

    auto iterator = std::filesystem::recursive_directory_iterator{
        root,
        std::filesystem::directory_options::skip_permission_denied};
    const auto end = std::filesystem::recursive_directory_iterator{};
    for (; iterator != end; ++iterator) {
        const auto& entry = *iterator;
        if (entry.is_directory() &&
            std::filesystem::exists(entry.path() / ".invisible_places-ignore")) {
            iterator.disable_recursion_pending();
            continue;
        }
        if (!entry.is_regular_file()) {
            continue;
        }

        files.push_back(entry.path());
    }

    std::sort(files.begin(), files.end());
    return files;
}

}  // namespace

std::string InferPointCloudSceneRoleFromName(std::string_view name) {
    std::string lowerName{name};
    lowerName = LowercaseCopy(std::move(lowerName));
    const std::string_view lowerView{lowerName};
    if (ContainsRoleToken(lowerView, "rock")) {
        return "ROCK";
    }
    if (ContainsRoleToken(lowerView, "sand")) {
        return "SAND";
    }
    if (ContainsRoleToken(lowerView, "veg") || ContainsRoleToken(lowerView, "vegetation")) {
        return "VEG";
    }
    return {};
}

float InferPointSpacingMetersFromName(std::string_view name) {
    std::string lowerName{name};
    lowerName = LowercaseCopy(std::move(lowerName));
    const char* data = lowerName.data();
    const char* const end = data + lowerName.size();
    for (const char* cursor = data; cursor < end; ++cursor) {
        if (!std::isdigit(static_cast<unsigned char>(*cursor)) && *cursor != '.') {
            continue;
        }
        const char* numberStart = cursor;
        bool seenDecimal = false;
        while (cursor < end &&
               (std::isdigit(static_cast<unsigned char>(*cursor)) ||
                (!seenDecimal && *cursor == '.'))) {
            seenDecimal = seenDecimal || *cursor == '.';
            ++cursor;
        }
        if (cursor + 1 < end && cursor[0] == 'm' && cursor[1] == 'm') {
            float millimetres = 0.0F;
            const auto result = std::from_chars(numberStart, cursor, millimetres);
            if (result.ec == std::errc{} && millimetres > 0.0F) {
                return millimetres * 0.001F;
            }
        }
        if (cursor == numberStart) {
            ++cursor;
        }
    }
    return 0.0F;
}

bool IsPrimaryPointCloudSceneRole(std::string_view role) {
    return role == "ROCK" || role == "rock" || role == "Rock";
}

bool IsWaterFillPointCloudName(std::string_view name) {
    std::string lowerName{name};
    lowerName = LowercaseCopy(std::move(lowerName));
    return ContainsRoleToken(lowerName, "water");
}

bool IsArchivedWaterFillPointCloudName(std::string_view name) {
    std::string lowerName{name};
    lowerName = LowercaseCopy(std::move(lowerName));

    constexpr std::array<std::string_view, 6> archiveTokens{
        "old", "backup", "bak", "archive", "previous", "copy"};
    const bool canonicalWater = ContainsRoleToken(lowerName, "water");
    for (const auto token : archiveTokens) {
        if (canonicalWater) {
            std::size_t tokenCursor = 0U;
            while (tokenCursor < lowerName.size()) {
                const auto position = lowerName.find(token, tokenCursor);
                if (position == std::string::npos) {
                    break;
                }
                const char before = position == 0U
                                        ? '\0'
                                        : lowerName[position - 1U];
                const auto afterIndex = position + token.size();
                const char after = afterIndex >= lowerName.size()
                                       ? '\0'
                                       : lowerName[afterIndex];
                // Recovery generations commonly use old01/backup2. A
                // numeric suffix is part of the archive token, not a new
                // semantic word.
                if (TokenBoundary(before) &&
                    (TokenBoundary(after) ||
                     std::isdigit(static_cast<unsigned char>(after)) != 0)) {
                    return true;
                }
                tokenCursor = position + 1U;
            }
        }

        // Legacy recovery files sometimes concatenate the marker directly
        // to WATER (for example WATERold01). Accept that one compound while
        // still rejecting unrelated words such as waterfall-old.
        const std::string compound = std::string{"water"} + std::string{token};
        std::size_t compoundCursor = 0U;
        while (compoundCursor < lowerName.size()) {
            const auto position = lowerName.find(compound, compoundCursor);
            if (position == std::string::npos) {
                break;
            }
            const char before = position == 0U
                                    ? '\0'
                                    : lowerName[position - 1U];
            const auto afterIndex = position + compound.size();
            const char after = afterIndex >= lowerName.size()
                                   ? '\0'
                                   : lowerName[afterIndex];
            if (TokenBoundary(before) &&
                (TokenBoundary(after) ||
                 std::isdigit(static_cast<unsigned char>(after)) != 0)) {
                return true;
            }
            compoundCursor = position + 1U;
        }
    }
    return false;
}

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
            if (!asset.sceneGroupName.empty()) {
                output << ", scene: " << asset.sceneGroupName;
                if (!asset.sceneRole.empty()) {
                    output << "/" << asset.sceneRole;
                }
                if (asset.inferredPointSpacingMeters > 0.0F) {
                    output << ", spacing: " << (asset.inferredPointSpacingMeters * 1000.0F) << " mm";
                }
            }

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

        // Recovery copies intentionally live beside the installed WATER
        // variants. Keeping them out of the catalog prevents an old copy
        // from being exposed as a loadable standalone layer or export source.
        if (IsArchivedWaterFillPointCloudName(filePath.stem().string())) {
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
            PointCloudAsset asset{.filePath = filePath, .header = headerResult.header};
            asset.sceneRole = SceneRoleFromStem(filePath);
            asset.inferredPointSpacingMeters = SceneSpacingFromStem(filePath);
            asset.scenePrimaryRole = IsPrimaryPointCloudSceneRole(asset.sceneRole);
            if (!asset.sceneRole.empty() && !ParentIsDiscoveryRoot(dataRoot, filePath)) {
                asset.sceneGroupName = filePath.parent_path().filename().string();
            }
            catalog.pointClouds.push_back(std::move(asset));
            continue;
        }

        catalog.issues.push_back(
            {.filePath = filePath, .message = "PLY did not match current point-cloud or gSplat heuristics."});
    }

    return catalog;
}

}  // namespace invisible_places::io
