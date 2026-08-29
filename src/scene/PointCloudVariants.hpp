#pragma once

#include "io/AssetDiscovery.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::scene {

using PointSpacingMicrometres = std::uint32_t;

enum class ScenePointCloudRole : std::uint8_t {
    Rock = 0,
    Sand,
    Vegetation,
};

inline constexpr std::size_t kScenePointCloudRoleCount = 3U;
inline constexpr PointSpacingMicrometres kRockAnalysisSpacingMicrometres = 1'000U;
inline constexpr PointSpacingMicrometres kSandAnalysisSpacingMicrometres = 1'000U;
inline constexpr PointSpacingMicrometres kVegetationAnalysisSpacingMicrometres = 1'000U;

[[nodiscard]] std::optional<ScenePointCloudRole> ParseScenePointCloudRole(std::string_view role);
[[nodiscard]] std::string_view ScenePointCloudRoleName(ScenePointCloudRole role);
[[nodiscard]] std::size_t ScenePointCloudRoleIndex(ScenePointCloudRole role);
[[nodiscard]] PointSpacingMicrometres CanonicalAnalysisSpacingMicrometres(
    ScenePointCloudRole role);

// Asset discovery stores spacing as floating-point metres. Quantizing once at the
// catalog boundary keeps density equality and ordering exact everywhere else.
[[nodiscard]] std::optional<PointSpacingMicrometres> QuantizePointSpacingMicrometres(
    double spacingMeters);

struct PointCloudVariant {
    ScenePointCloudRole role = ScenePointCloudRole::Rock;
    PointSpacingMicrometres spacingMicrometres = 0U;
    std::filesystem::path sourcePath;
    std::uint64_t pointCount = 0U;
};

// Density-aware auxiliary clouds (currently the standing-water fill) are not
// primary ROCK/SAND/VEG roles, but still follow the active scene density. The
// source index is supplied by the caller so this policy stays independent of
// the app runtime's session type.
struct AuxiliaryDensityVariantCandidate {
    PointSpacingMicrometres spacingMicrometres = 0U;
    std::size_t sourceIndex = 0U;
};

// Prefer an exact density. If it is absent, choose the nearest available
// density and prefer the finer source on an equal-distance tie. A zero target
// selects the finest candidate and is used as the visual reference density.
[[nodiscard]] std::optional<std::size_t> SelectAuxiliaryDensityVariant(
    std::span<const AuxiliaryDensityVariantCandidate> candidates,
    PointSpacingMicrometres targetSpacingMicrometres);

struct SceneAnalysisSources {
    std::array<std::optional<PointCloudVariant>, kScenePointCloudRoleCount> byRole;

    [[nodiscard]] const PointCloudVariant* Find(ScenePointCloudRole role) const;
    [[nodiscard]] bool Complete() const;
    [[nodiscard]] std::uint64_t TotalPointCount() const;
};

struct SceneDensityBundle {
    PointSpacingMicrometres spacingMicrometres = 0U;
    std::array<PointCloudVariant, kScenePointCloudRoleCount> byRole;
    std::uint64_t totalPointCount = 0U;

    [[nodiscard]] const PointCloudVariant& Find(ScenePointCloudRole role) const;
};

enum class SceneDisplaySelectionKind : std::uint8_t {
    CompleteBundle,
    Mixed,
};

struct SceneDisplaySelection {
    SceneDisplaySelectionKind kind = SceneDisplaySelectionKind::Mixed;
    std::optional<PointSpacingMicrometres> spacingMicrometres;
    std::array<std::optional<PointCloudVariant>, kScenePointCloudRoleCount> byRole;
    std::uint64_t totalPointCount = 0U;

    [[nodiscard]] const PointCloudVariant* Find(ScenePointCloudRole role) const;
    [[nodiscard]] bool Complete() const;
    [[nodiscard]] bool Switchable() const {
        return kind == SceneDisplaySelectionKind::CompleteBundle;
    }
};

struct ScenePointCloudGroup {
    std::string name;
    std::filesystem::path sourceFolder;
    std::array<std::vector<PointCloudVariant>, kScenePointCloudRoleCount> variantsByRole;
    SceneAnalysisSources analysisSources;
    std::vector<SceneDensityBundle> completeDisplayBundles;
    SceneDisplaySelection defaultDisplay;

    [[nodiscard]] const std::vector<PointCloudVariant>& Variants(ScenePointCloudRole role) const;
    [[nodiscard]] const PointCloudVariant* AnalysisSource(ScenePointCloudRole role) const;
    [[nodiscard]] const SceneDensityBundle* FindCompleteDisplayBundle(
        PointSpacingMicrometres spacingMicrometres) const;
};

// Only assets assigned to a named scene group participate. Standalone point
// clouds remain independent layers and are intentionally absent from this model.
[[nodiscard]] std::vector<ScenePointCloudGroup> BuildScenePointCloudGroups(
    std::span<const invisible_places::io::PointCloudAsset> assets);

[[nodiscard]] inline std::vector<ScenePointCloudGroup> BuildScenePointCloudGroups(
    const invisible_places::io::AssetCatalog& catalog) {
    return BuildScenePointCloudGroups(catalog.pointClouds);
}

}  // namespace invisible_places::scene
