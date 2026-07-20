#include "InvisiblePlacesBuildConfig.hpp"
#include "io/AssetDiscovery.hpp"
#include "scene/PointCloudVariants.hpp"
#include "scene/SceneCatalog.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using invisible_places::io::PointCloudAsset;
using invisible_places::scene::PointSpacingMicrometres;
using invisible_places::scene::ScenePointCloudRole;

PointCloudAsset MakeVariant(
    const std::filesystem::path& folder,
    std::string_view sceneName,
    std::string_view role,
    PointSpacingMicrometres spacingMicrometres,
    std::uint64_t pointCount,
    std::string_view suffix = {}) {
    const auto spacingMillimetres = static_cast<double>(spacingMicrometres) / 1'000.0;
    auto filename = std::string{role} + "-" + std::to_string(spacingMillimetres) + "mm";
    if (!suffix.empty()) {
        filename += "-";
        filename += suffix;
    }
    filename += ".ply";

    PointCloudAsset asset;
    asset.filePath = folder / filename;
    asset.header.vertexCount = pointCount;
    asset.sceneGroupName = sceneName;
    asset.sceneRole = role;
    asset.inferredPointSpacingMeters = static_cast<float>(
        static_cast<double>(spacingMicrometres) / 1'000'000.0);
    asset.scenePrimaryRole = role == "ROCK";
    return asset;
}

void AddCompleteDensity(
    std::vector<PointCloudAsset>* assets,
    const std::filesystem::path& folder,
    std::string_view sceneName,
    PointSpacingMicrometres spacingMicrometres,
    std::uint64_t basePointCount) {
    assets->push_back(MakeVariant(
        folder,
        sceneName,
        "ROCK",
        spacingMicrometres,
        basePointCount + 1U));
    assets->push_back(MakeVariant(
        folder,
        sceneName,
        "SAND",
        spacingMicrometres,
        basePointCount + 2U));
    assets->push_back(MakeVariant(
        folder,
        sceneName,
        "VEG",
        spacingMicrometres,
        basePointCount + 3U));
}

}  // namespace

TEST_CASE("Point-cloud density spacing is quantized to integer micrometres", "[scene][density]") {
    using invisible_places::scene::ParseScenePointCloudRole;
    using invisible_places::scene::QuantizePointSpacingMicrometres;

    CHECK(QuantizePointSpacingMicrometres(0.001).value() == 1'000U);
    CHECK(QuantizePointSpacingMicrometres(0.002).value() == 2'000U);
    CHECK(QuantizePointSpacingMicrometres(0.005).value() == 5'000U);
    CHECK(QuantizePointSpacingMicrometres(0.001000499).value() == 1'000U);
    CHECK(QuantizePointSpacingMicrometres(0.001000501).value() == 1'001U);
    CHECK_FALSE(QuantizePointSpacingMicrometres(0.0).has_value());
    CHECK_FALSE(QuantizePointSpacingMicrometres(-0.001).has_value());
    CHECK_FALSE(QuantizePointSpacingMicrometres(std::numeric_limits<double>::infinity()).has_value());
    CHECK_FALSE(QuantizePointSpacingMicrometres(std::numeric_limits<double>::quiet_NaN()).has_value());

    CHECK(ParseScenePointCloudRole(" rock ").value() == ScenePointCloudRole::Rock);
    CHECK(ParseScenePointCloudRole("Sand").value() == ScenePointCloudRole::Sand);
    CHECK(ParseScenePointCloudRole("vegetation").value() == ScenePointCloudRole::Vegetation);
    CHECK_FALSE(ParseScenePointCloudRole("MESH").has_value());
}

TEST_CASE("Only unambiguous complete scene densities are selectable", "[scene][density]") {
    const std::filesystem::path folder{"Data/SyntheticScene"};
    std::vector<PointCloudAsset> assets;
    AddCompleteDensity(&assets, folder, "SyntheticScene", 1'000U, 100U);

    assets.push_back(MakeVariant(folder, "SyntheticScene", "ROCK", 2'000U, 50U));
    assets.push_back(MakeVariant(folder, "SyntheticScene", "SAND", 2'000U, 60U));

    AddCompleteDensity(&assets, folder, "SyntheticScene", 3'000U, 30U);
    assets.push_back(MakeVariant(
        folder,
        "SyntheticScene",
        "ROCK",
        3'000U,
        31U,
        "duplicate"));

    auto ignoredStandalone = MakeVariant(folder, {}, "ROCK", 4'000U, 20U);
    assets.push_back(std::move(ignoredStandalone));
    auto ignoredMesh = MakeVariant(folder, "SyntheticScene", "MESH", 1'000U, 20U);
    assets.push_back(std::move(ignoredMesh));

    const auto groups = invisible_places::scene::BuildScenePointCloudGroups(assets);
    REQUIRE(groups.size() == 1U);
    const auto& group = groups.front();
    REQUIRE(group.completeDisplayBundles.size() == 1U);
    CHECK(group.completeDisplayBundles.front().spacingMicrometres == 1'000U);
    CHECK(group.FindCompleteDisplayBundle(1'000U) != nullptr);
    CHECK(group.FindCompleteDisplayBundle(2'000U) == nullptr);
    CHECK(group.FindCompleteDisplayBundle(3'000U) == nullptr);
    CHECK(group.Variants(ScenePointCloudRole::Rock).size() == 4U);
}

TEST_CASE("Canonical analysis sources require one exact role spacing", "[scene][density]") {
    const std::filesystem::path folder{"Data/FallbackScene"};
    std::vector<PointCloudAsset> assets{
        MakeVariant(folder, "FallbackScene", "ROCK", 1'500U, 15U, "z"),
        MakeVariant(folder, "FallbackScene", "ROCK", 500U, 5U, "z"),
        MakeVariant(folder, "FallbackScene", "SAND", 2'000U, 20U, "z"),
        MakeVariant(folder, "FallbackScene", "VEG", 1'000U, 11U, "z"),
        MakeVariant(folder, "FallbackScene", "VEG", 1'000U, 12U, "a"),
    };

    const auto groups = invisible_places::scene::BuildScenePointCloudGroups(assets);
    REQUIRE(groups.size() == 1U);
    const auto& group = groups.front();
    REQUIRE(group.completeDisplayBundles.empty());
    CHECK_FALSE(group.analysisSources.Complete());

    const auto* rock = group.AnalysisSource(ScenePointCloudRole::Rock);
    const auto* sand = group.AnalysisSource(ScenePointCloudRole::Sand);
    const auto* vegetation = group.AnalysisSource(ScenePointCloudRole::Vegetation);
    CHECK(rock == nullptr);
    REQUIRE(sand != nullptr);
    CHECK(vegetation == nullptr);
    CHECK(sand->spacingMicrometres == 2'000U);

    CHECK(group.defaultDisplay.kind == invisible_places::scene::SceneDisplaySelectionKind::Mixed);
    CHECK_FALSE(group.defaultDisplay.spacingMicrometres.has_value());
    CHECK_FALSE(group.defaultDisplay.Complete());
    CHECK_FALSE(group.defaultDisplay.Switchable());
    CHECK(group.defaultDisplay.totalPointCount == 20U);

    std::reverse(assets.begin(), assets.end());
    const auto reversedGroups = invisible_places::scene::BuildScenePointCloudGroups(assets);
    REQUIRE(reversedGroups.size() == 1U);
    CHECK(reversedGroups.front().AnalysisSource(ScenePointCloudRole::Rock) == nullptr);
    CHECK(reversedGroups.front().AnalysisSource(ScenePointCloudRole::Vegetation) == nullptr);
    REQUIRE(reversedGroups.front().AnalysisSource(ScenePointCloudRole::Sand) != nullptr);
    CHECK(reversedGroups.front().AnalysisSource(ScenePointCloudRole::Sand)->spacingMicrometres == 2'000U);
}

TEST_CASE("The finest complete density is the default display bundle", "[scene][density]") {
    const std::filesystem::path folder{"Data/CompleteScene"};
    std::vector<PointCloudAsset> assets;
    AddCompleteDensity(&assets, folder, "CompleteScene", 5'000U, 10U);
    AddCompleteDensity(&assets, folder, "CompleteScene", 2'000U, 100U);

    const auto groups = invisible_places::scene::BuildScenePointCloudGroups(assets);
    REQUIRE(groups.size() == 1U);
    const auto& group = groups.front();
    REQUIRE(group.completeDisplayBundles.size() == 2U);
    CHECK(group.completeDisplayBundles[0].spacingMicrometres == 2'000U);
    CHECK(group.completeDisplayBundles[1].spacingMicrometres == 5'000U);
    CHECK(group.defaultDisplay.kind == invisible_places::scene::SceneDisplaySelectionKind::CompleteBundle);
    REQUIRE(group.defaultDisplay.spacingMicrometres.has_value());
    CHECK(group.defaultDisplay.spacingMicrometres.value() == 2'000U);
    CHECK(group.defaultDisplay.Switchable());
    CHECK(group.defaultDisplay.totalPointCount == 306U);
    REQUIRE(group.defaultDisplay.Find(ScenePointCloudRole::Sand) != nullptr);
    CHECK(group.defaultDisplay.Find(ScenePointCloudRole::Sand)->spacingMicrometres == 2'000U);
}

TEST_CASE("Scene groups are keyed by folder rather than display name", "[scene][density]") {
    std::vector<PointCloudAsset> assets;
    AddCompleteDensity(&assets, "Data/A/Repeated", "Repeated", 1'000U, 100U);
    AddCompleteDensity(&assets, "Data/B/Repeated", "Repeated", 2'000U, 50U);
    invisible_places::io::AssetCatalog assetCatalog;
    assetCatalog.pointClouds = assets;

    const auto catalog = invisible_places::scene::SceneCatalog::FromDiscoveredAssets(assetCatalog);
    REQUIRE(catalog.PointCloudGroups().size() == 2U);
    CHECK(catalog.FindPointCloudGroup("Repeated") == nullptr);
    REQUIRE(catalog.FindPointCloudGroupBySourceFolder("Data/A/Repeated") != nullptr);
    CHECK(
        catalog.FindPointCloudGroupBySourceFolder("Data/A/Repeated")
            ->completeDisplayBundles.front()
            .spacingMicrometres == 1'000U);
}

TEST_CASE("Scene1 exposes 1, 2, 3, and 5 millimetre display bundles", "[scene][density][data]") {
    const std::filesystem::path dataRoot{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
    if (!std::filesystem::exists(dataRoot / "Scene1")) {
        SKIP("Scene1 fixture is not present in the local Data directory.");
    }

    const auto assetCatalog = invisible_places::io::DiscoverAssets(dataRoot);
    const auto sceneCatalog = invisible_places::scene::SceneCatalog::FromDiscoveredAssets(assetCatalog);
    const auto* scene1 = sceneCatalog.FindPointCloudGroup("Scene1");
    REQUIRE(scene1 != nullptr);
    REQUIRE(scene1->completeDisplayBundles.size() == 4U);
    const std::array<PointSpacingMicrometres, 4U> expectedSpacings{
        1'000U,
        2'000U,
        3'000U,
        5'000U,
    };
    for (std::size_t index = 0; index < expectedSpacings.size(); ++index) {
        CHECK(scene1->completeDisplayBundles[index].spacingMicrometres == expectedSpacings[index]);
        CHECK(scene1->completeDisplayBundles[index].totalPointCount > 0U);
    }

    REQUIRE(scene1->AnalysisSource(ScenePointCloudRole::Rock) != nullptr);
    REQUIRE(scene1->AnalysisSource(ScenePointCloudRole::Sand) != nullptr);
    REQUIRE(scene1->AnalysisSource(ScenePointCloudRole::Vegetation) != nullptr);
    CHECK(scene1->AnalysisSource(ScenePointCloudRole::Rock)->spacingMicrometres == 1'000U);
    CHECK(scene1->AnalysisSource(ScenePointCloudRole::Sand)->spacingMicrometres == 2'000U);
    CHECK(scene1->AnalysisSource(ScenePointCloudRole::Vegetation)->spacingMicrometres == 1'000U);
    REQUIRE(scene1->defaultDisplay.spacingMicrometres.has_value());
    CHECK(scene1->defaultDisplay.spacingMicrometres.value() == 1'000U);
}
