#include "InvisiblePlacesBuildConfig.hpp"
#include "io/AssetDiscovery.hpp"
#include "renderer/core/FrameTiming.hpp"
#include "scene/PointCloudVariants.hpp"
#include "scene/SceneCatalog.hpp"

#include <catch2/catch_approx.hpp>
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

TEST_CASE("Swapchain image ownership is tied to a submission generation", "[renderer][sync]") {
    using invisible_places::renderer::core::ClassifySwapchainImageOwner;
    using invisible_places::renderer::core::ClearCompletedImageOwners;
    using invisible_places::renderer::core::SwapchainImageOwner;
    using invisible_places::renderer::core::SwapchainImageOwnerState;

    std::vector<SwapchainImageOwner> imageOwners(3U);

    // F0/image0 -> F1/image1.
    imageOwners[0] = {.frameSlot = 0U, .submissionSerial = 1U};
    imageOwners[1] = {.frameSlot = 1U, .submissionSerial = 2U};
    CHECK(
        ClassifySwapchainImageOwner(imageOwners[0], 1U, 0U) ==
        SwapchainImageOwnerState::Active);
    CHECK(
        ClassifySwapchainImageOwner(imageOwners[1], 2U, 0U) ==
        SwapchainImageOwnerState::Active);

    // Completing F0/serial1 retires only image0's exact generation.
    CHECK(ClearCompletedImageOwners(&imageOwners, 0U, 1U) == 1U);
    CHECK_FALSE(imageOwners[0].Valid());
    CHECK(imageOwners[1] == SwapchainImageOwner{1U, 2U});

    // Reuse F0 for image2. Reacquiring image0 must remain unowned rather than
    // aliasing the fence now used by F0/serial3.
    imageOwners[2] = {.frameSlot = 0U, .submissionSerial = 3U};
    CHECK(
        ClassifySwapchainImageOwner(imageOwners[0], 3U, 1U) ==
        SwapchainImageOwnerState::Unowned);
    CHECK(
        ClassifySwapchainImageOwner(imageOwners[2], 3U, 1U) ==
        SwapchainImageOwnerState::Active);

    const SwapchainImageOwner staleOwner{.frameSlot = 0U, .submissionSerial = 1U};
    CHECK(
        ClassifySwapchainImageOwner(staleOwner, 3U, 0U) ==
        SwapchainImageOwnerState::Stale);
    CHECK(
        ClassifySwapchainImageOwner(staleOwner, 3U, 1U) ==
        SwapchainImageOwnerState::Completed);
    CHECK(ClearCompletedImageOwners(nullptr, 0U, 1U) == 0U);
    CHECK(ClearCompletedImageOwners(&imageOwners, 0U, 0U) == 0U);
}

TEST_CASE("GPU timestamp deltas respect valid-bit wrapping", "[renderer][timing]") {
    using invisible_places::renderer::core::RollingPhaseAverages;
    using invisible_places::renderer::core::TimestampDeltaMilliseconds;
    using invisible_places::renderer::core::TimestampMask;
    using invisible_places::renderer::core::TimestampQueryResult;
    using invisible_places::renderer::core::TimestampResultsAvailable;
    using invisible_places::renderer::core::TimestampTickDelta;

    CHECK(TimestampMask(0U) == 0U);
    CHECK(TimestampMask(4U) == 0x0fU);
    CHECK(TimestampMask(64U) == std::numeric_limits<std::uint64_t>::max());
    CHECK(TimestampTickDelta(100U, 140U, 64U) == 40U);
    CHECK(TimestampTickDelta(250U, 5U, 8U) == 11U);
    CHECK(TimestampTickDelta(250U, 5U, 0U) == 0U);
    CHECK(TimestampDeltaMilliseconds(0U, 500'000U, 64U, 2.0) ==
          Catch::Approx(1.0));
    CHECK(TimestampDeltaMilliseconds(0U, 500'000U, 64U, 0.0) == 0.0);
    CHECK(TimestampDeltaMilliseconds(0U, 500'000U, 64U, -1.0) == 0.0);

    const std::array<TimestampQueryResult, 2U> availableResults{{
        {.value = 10U, .available = 1U},
        {.value = 20U, .available = 1U},
    }};
    auto unavailableResults = availableResults;
    unavailableResults[1].available = 0U;
    CHECK(TimestampResultsAvailable(availableResults));
    CHECK_FALSE(TimestampResultsAvailable(unavailableResults));
    CHECK_FALSE(TimestampResultsAvailable({}));

    RollingPhaseAverages<3U> averages;
    averages.AddFrame(
        {2.0, 50.0, 6.0},
        {true, false, true},
        200.0);
    CHECK_FALSE(averages.PublishIfReady());
    CHECK(averages.PendingSampleCount(0U) == 1U);
    CHECK(averages.PendingSampleCount(1U) == 0U);
    averages.AddFrame(
        {4.0, 70.0, 10.0},
        {true, false, true},
        300.0);
    REQUIRE(averages.PublishIfReady());
    CHECK(averages.PublishedAverage(0U) == Catch::Approx(3.0));
    CHECK_FALSE(averages.PublishedActive(1U));
    CHECK(averages.PublishedAverage(2U) == Catch::Approx(8.0));
    averages.Reset();
    CHECK(averages.PendingSampleCount(0U) == 0U);
    CHECK_FALSE(averages.PublishedActive(0U));
}

TEST_CASE("Scene3 exposes 1, 2, 3, and 5 millimetre display bundles", "[scene][density][data]") {
    const std::filesystem::path dataRoot{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
    if (!std::filesystem::exists(dataRoot / "Scene3")) {
        SKIP("Scene3 fixture is not present in the local Data directory.");
    }

    const auto assetCatalog = invisible_places::io::DiscoverAssets(dataRoot);
    const auto sceneCatalog = invisible_places::scene::SceneCatalog::FromDiscoveredAssets(assetCatalog);
    const auto* scene3 = sceneCatalog.FindPointCloudGroup("Scene3");
    REQUIRE(scene3 != nullptr);
    REQUIRE(scene3->completeDisplayBundles.size() == 4U);
    const std::array<PointSpacingMicrometres, 4U> expectedSpacings{
        1'000U,
        2'000U,
        3'000U,
        5'000U,
    };
    for (std::size_t index = 0; index < expectedSpacings.size(); ++index) {
        CHECK(scene3->completeDisplayBundles[index].spacingMicrometres == expectedSpacings[index]);
        CHECK(scene3->completeDisplayBundles[index].totalPointCount > 0U);
    }

    REQUIRE(scene3->AnalysisSource(ScenePointCloudRole::Rock) != nullptr);
    REQUIRE(scene3->AnalysisSource(ScenePointCloudRole::Sand) != nullptr);
    REQUIRE(scene3->AnalysisSource(ScenePointCloudRole::Vegetation) != nullptr);
    CHECK(scene3->AnalysisSource(ScenePointCloudRole::Rock)->spacingMicrometres == 1'000U);
    CHECK(scene3->AnalysisSource(ScenePointCloudRole::Sand)->spacingMicrometres == 2'000U);
    CHECK(scene3->AnalysisSource(ScenePointCloudRole::Vegetation)->spacingMicrometres == 1'000U);
    REQUIRE(scene3->defaultDisplay.spacingMicrometres.has_value());
    CHECK(scene3->defaultDisplay.spacingMicrometres.value() == 1'000U);
}
