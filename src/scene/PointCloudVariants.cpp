#include "scene/PointCloudVariants.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <iterator>
#include <map>
#include <set>
#include <tuple>
#include <utility>

namespace invisible_places::scene {

namespace {

bool EqualsAsciiCaseInsensitive(std::string_view left, std::string_view right) {
    if (left.size() != right.size()) {
        return false;
    }
    for (std::size_t index = 0; index < left.size(); ++index) {
        const auto leftCharacter = static_cast<unsigned char>(left[index]);
        const auto rightCharacter = static_cast<unsigned char>(right[index]);
        if (std::toupper(leftCharacter) != std::toupper(rightCharacter)) {
            return false;
        }
    }
    return true;
}

std::string_view TrimAsciiWhitespace(std::string_view value) {
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())) != 0) {
        value.remove_prefix(1U);
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
        value.remove_suffix(1U);
    }
    return value;
}

std::uint64_t SaturatingAdd(std::uint64_t left, std::uint64_t right) {
    if (right > std::numeric_limits<std::uint64_t>::max() - left) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return left + right;
}

std::string PathSortKey(const std::filesystem::path& path) {
    return path.lexically_normal().generic_string();
}

const PointCloudVariant* SelectAnalysisSource(
    const std::vector<PointCloudVariant>& variants,
    PointSpacingMicrometres targetSpacingMicrometres) {
    const auto first = std::lower_bound(
        variants.begin(),
        variants.end(),
        targetSpacingMicrometres,
        [](const PointCloudVariant& variant, PointSpacingMicrometres spacing) {
            return variant.spacingMicrometres < spacing;
        });
    if (first == variants.end() || first->spacingMicrometres != targetSpacingMicrometres) {
        return nullptr;
    }
    const auto after = std::find_if(
        std::next(first),
        variants.end(),
        [targetSpacingMicrometres](const PointCloudVariant& variant) {
            return variant.spacingMicrometres != targetSpacingMicrometres;
        });
    return std::distance(first, after) == 1 ? &*first : nullptr;
}

struct GroupAccumulator {
    std::filesystem::path sourceFolder;
    std::set<std::string> names;
    std::array<std::vector<PointCloudVariant>, kScenePointCloudRoleCount> variantsByRole;
};

SceneDisplaySelection CompleteSelectionFromBundle(const SceneDensityBundle& bundle) {
    SceneDisplaySelection selection;
    selection.kind = SceneDisplaySelectionKind::CompleteBundle;
    selection.spacingMicrometres = bundle.spacingMicrometres;
    selection.totalPointCount = bundle.totalPointCount;
    for (std::size_t roleIndex = 0; roleIndex < kScenePointCloudRoleCount; ++roleIndex) {
        selection.byRole[roleIndex] = bundle.byRole[roleIndex];
    }
    return selection;
}

SceneDisplaySelection MixedSelectionFromAnalysis(const SceneAnalysisSources& analysisSources) {
    SceneDisplaySelection selection;
    selection.kind = SceneDisplaySelectionKind::Mixed;
    selection.byRole = analysisSources.byRole;
    selection.totalPointCount = analysisSources.TotalPointCount();
    return selection;
}

}  // namespace

std::optional<ScenePointCloudRole> ParseScenePointCloudRole(std::string_view role) {
    role = TrimAsciiWhitespace(role);
    if (EqualsAsciiCaseInsensitive(role, "ROCK")) {
        return ScenePointCloudRole::Rock;
    }
    if (EqualsAsciiCaseInsensitive(role, "SAND")) {
        return ScenePointCloudRole::Sand;
    }
    if (EqualsAsciiCaseInsensitive(role, "VEG") ||
        EqualsAsciiCaseInsensitive(role, "VEGETATION")) {
        return ScenePointCloudRole::Vegetation;
    }
    return std::nullopt;
}

std::string_view ScenePointCloudRoleName(ScenePointCloudRole role) {
    switch (role) {
        case ScenePointCloudRole::Rock:
            return "ROCK";
        case ScenePointCloudRole::Sand:
            return "SAND";
        case ScenePointCloudRole::Vegetation:
            return "VEG";
    }
    return {};
}

std::size_t ScenePointCloudRoleIndex(ScenePointCloudRole role) {
    switch (role) {
        case ScenePointCloudRole::Rock:
            return 0U;
        case ScenePointCloudRole::Sand:
            return 1U;
        case ScenePointCloudRole::Vegetation:
            return 2U;
    }
    return kScenePointCloudRoleCount;
}

PointSpacingMicrometres CanonicalAnalysisSpacingMicrometres(ScenePointCloudRole role) {
    switch (role) {
        case ScenePointCloudRole::Rock:
            return kRockAnalysisSpacingMicrometres;
        case ScenePointCloudRole::Sand:
            return kSandAnalysisSpacingMicrometres;
        case ScenePointCloudRole::Vegetation:
            return kVegetationAnalysisSpacingMicrometres;
    }
    return 0U;
}

std::optional<PointSpacingMicrometres> QuantizePointSpacingMicrometres(double spacingMeters) {
    constexpr double kMicrometresPerMetre = 1'000'000.0;
    if (!std::isfinite(spacingMeters) || spacingMeters <= 0.0) {
        return std::nullopt;
    }
    const double spacingMicrometres = spacingMeters * kMicrometresPerMetre;
    if (!std::isfinite(spacingMicrometres) || spacingMicrometres < 0.5 ||
        spacingMicrometres > static_cast<double>(std::numeric_limits<PointSpacingMicrometres>::max()) +
                                0.499999) {
        return std::nullopt;
    }
    return static_cast<PointSpacingMicrometres>(std::llround(spacingMicrometres));
}

const PointCloudVariant* SceneAnalysisSources::Find(ScenePointCloudRole role) const {
    const auto roleIndex = ScenePointCloudRoleIndex(role);
    if (roleIndex >= byRole.size() || !byRole[roleIndex].has_value()) {
        return nullptr;
    }
    return &byRole[roleIndex].value();
}

bool SceneAnalysisSources::Complete() const {
    return std::all_of(byRole.begin(), byRole.end(), [](const auto& source) {
        return source.has_value();
    });
}

std::uint64_t SceneAnalysisSources::TotalPointCount() const {
    std::uint64_t totalPointCount = 0U;
    for (const auto& source : byRole) {
        if (source.has_value()) {
            totalPointCount = SaturatingAdd(totalPointCount, source->pointCount);
        }
    }
    return totalPointCount;
}

const PointCloudVariant& SceneDensityBundle::Find(ScenePointCloudRole role) const {
    return byRole.at(ScenePointCloudRoleIndex(role));
}

const PointCloudVariant* SceneDisplaySelection::Find(ScenePointCloudRole role) const {
    const auto roleIndex = ScenePointCloudRoleIndex(role);
    if (roleIndex >= byRole.size() || !byRole[roleIndex].has_value()) {
        return nullptr;
    }
    return &byRole[roleIndex].value();
}

bool SceneDisplaySelection::Complete() const {
    return std::all_of(byRole.begin(), byRole.end(), [](const auto& source) {
        return source.has_value();
    });
}

const std::vector<PointCloudVariant>& ScenePointCloudGroup::Variants(ScenePointCloudRole role) const {
    return variantsByRole.at(ScenePointCloudRoleIndex(role));
}

const PointCloudVariant* ScenePointCloudGroup::AnalysisSource(ScenePointCloudRole role) const {
    return analysisSources.Find(role);
}

const SceneDensityBundle* ScenePointCloudGroup::FindCompleteDisplayBundle(
    PointSpacingMicrometres spacingMicrometres) const {
    const auto bundle = std::lower_bound(
        completeDisplayBundles.begin(),
        completeDisplayBundles.end(),
        spacingMicrometres,
        [](const SceneDensityBundle& candidate, PointSpacingMicrometres spacing) {
            return candidate.spacingMicrometres < spacing;
        });
    if (bundle == completeDisplayBundles.end() ||
        bundle->spacingMicrometres != spacingMicrometres) {
        return nullptr;
    }
    return &*bundle;
}

std::vector<ScenePointCloudGroup> BuildScenePointCloudGroups(
    std::span<const invisible_places::io::PointCloudAsset> assets) {
    std::map<std::string, GroupAccumulator> groupsByFolder;

    for (const auto& asset : assets) {
        if (asset.sceneGroupName.empty() || asset.filePath.empty()) {
            continue;
        }
        const auto role = ParseScenePointCloudRole(asset.sceneRole);
        const auto spacing = QuantizePointSpacingMicrometres(asset.inferredPointSpacingMeters);
        if (!role.has_value() || !spacing.has_value()) {
            continue;
        }

        const auto sourceFolder = asset.filePath.parent_path().lexically_normal();
        auto& accumulator = groupsByFolder[PathSortKey(sourceFolder)];
        accumulator.sourceFolder = sourceFolder;
        accumulator.names.insert(asset.sceneGroupName);
        accumulator.variantsByRole[ScenePointCloudRoleIndex(role.value())].push_back(
            {.role = role.value(),
             .spacingMicrometres = spacing.value(),
             .sourcePath = asset.filePath,
             .pointCount = asset.header.vertexCount});
    }

    std::vector<ScenePointCloudGroup> groups;
    groups.reserve(groupsByFolder.size());
    for (auto& [folderKey, accumulator] : groupsByFolder) {
        (void)folderKey;
        ScenePointCloudGroup group;
        group.sourceFolder = std::move(accumulator.sourceFolder);
        if (!accumulator.names.empty()) {
            group.name = *accumulator.names.begin();
        }
        group.variantsByRole = std::move(accumulator.variantsByRole);

        std::map<
            PointSpacingMicrometres,
            std::array<std::vector<const PointCloudVariant*>, kScenePointCloudRoleCount>>
            variantsBySpacing;
        for (std::size_t roleIndex = 0; roleIndex < kScenePointCloudRoleCount; ++roleIndex) {
            auto& roleVariants = group.variantsByRole[roleIndex];
            std::sort(
                roleVariants.begin(),
                roleVariants.end(),
                [](const PointCloudVariant& left, const PointCloudVariant& right) {
                    return std::tuple{
                               left.spacingMicrometres,
                               PathSortKey(left.sourcePath),
                               left.pointCount} <
                           std::tuple{
                               right.spacingMicrometres,
                               PathSortKey(right.sourcePath),
                               right.pointCount};
                });

            const auto role = static_cast<ScenePointCloudRole>(roleIndex);
            if (const auto* analysisSource = SelectAnalysisSource(
                    roleVariants,
                    CanonicalAnalysisSpacingMicrometres(role));
                analysisSource != nullptr) {
                group.analysisSources.byRole[roleIndex] = *analysisSource;
            }
            for (const auto& variant : roleVariants) {
                variantsBySpacing[variant.spacingMicrometres][roleIndex].push_back(&variant);
            }
        }

        for (const auto& [spacing, roleVariants] : variantsBySpacing) {
            const bool exactlyOneAssetPerRole =
                std::all_of(roleVariants.begin(), roleVariants.end(), [](const auto& variants) {
                    return variants.size() == 1U;
                });
            if (!exactlyOneAssetPerRole) {
                continue;
            }

            SceneDensityBundle bundle;
            bundle.spacingMicrometres = spacing;
            for (std::size_t roleIndex = 0; roleIndex < kScenePointCloudRoleCount; ++roleIndex) {
                bundle.byRole[roleIndex] = *roleVariants[roleIndex].front();
                bundle.totalPointCount =
                    SaturatingAdd(bundle.totalPointCount, bundle.byRole[roleIndex].pointCount);
            }
            group.completeDisplayBundles.push_back(std::move(bundle));
        }

        if (!group.completeDisplayBundles.empty()) {
            group.defaultDisplay = CompleteSelectionFromBundle(group.completeDisplayBundles.front());
        } else {
            group.defaultDisplay = MixedSelectionFromAnalysis(group.analysisSources);
        }
        groups.push_back(std::move(group));
    }

    return groups;
}

}  // namespace invisible_places::scene
