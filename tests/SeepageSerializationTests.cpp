#include "serialization/ProjectDocument.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

using invisible_places::serialization::ProjectDocument;
using invisible_places::serialization::WaterSourcesDocument;
using invisible_places::water::WaterEffectBlendMode;
using invisible_places::water::WaterSeepageLookProfile;
using invisible_places::water::WaterSeepageLookSettings;
using invisible_places::water::WaterSeepageNode;
using invisible_places::water::WaterSeepageQuality;

WaterSeepageLookSettings MakeLook(
    WaterSeepageQuality quality,
    float offset,
    WaterEffectBlendMode blendMode) {
    WaterSeepageLookSettings look;
    look.quality = quality;
    look.baseWetness = 0.11F + offset;
    look.density = 0.22F + offset;
    look.glisten = 0.33F + offset;
    look.patternScale = 1.44F + offset;
    look.wavelengthMeters = 0.055F + offset;
    look.speed = 0.16F + offset;
    look.warp = 0.27F + offset;
    look.turbulence = 0.38F + offset;
    look.phase = 0.49F + offset;
    look.rainResponse = 0.60F + offset;
    look.blendMode = blendMode;
    look.response.intensity = 0.71F + offset;
    look.response.emissionAdd = 0.82F + offset;
    look.response.opacityAdd = 0.093F + offset;
    look.response.opacityMultiply = 1.14F + offset;
    look.response.pointSizeAdd = 0.025F + offset;
    look.response.pointSizeMultiply = 1.36F + offset;
    look.response.hueShift = 0.047F + offset;
    look.response.colouriseRed = 0.18F + offset;
    look.response.colouriseGreen = 0.29F + offset;
    look.response.colouriseBlue = 0.40F + offset;
    look.response.colouriseAmount = 0.51F + offset;
    look.response.gaussianSharpnessBias = 0.062F + offset;
    return look;
}

void CheckLook(
    const WaterSeepageLookSettings& actual,
    const WaterSeepageLookSettings& expected) {
    CHECK(actual.quality == expected.quality);
    CHECK(actual.baseWetness == Catch::Approx(expected.baseWetness));
    CHECK(actual.density == Catch::Approx(expected.density));
    CHECK(actual.glisten == Catch::Approx(expected.glisten));
    CHECK(actual.patternScale == Catch::Approx(expected.patternScale));
    CHECK(actual.wavelengthMeters == Catch::Approx(expected.wavelengthMeters));
    CHECK(actual.speed == Catch::Approx(expected.speed));
    CHECK(actual.warp == Catch::Approx(expected.warp));
    CHECK(actual.turbulence == Catch::Approx(expected.turbulence));
    CHECK(actual.phase == Catch::Approx(expected.phase));
    CHECK(actual.rainResponse == Catch::Approx(expected.rainResponse));
    CHECK(actual.blendMode == expected.blendMode);
    CHECK(actual.response.intensity == Catch::Approx(expected.response.intensity));
    CHECK(actual.response.emissionAdd == Catch::Approx(expected.response.emissionAdd));
    CHECK(actual.response.opacityAdd == Catch::Approx(expected.response.opacityAdd));
    CHECK(actual.response.opacityMultiply == Catch::Approx(expected.response.opacityMultiply));
    CHECK(actual.response.pointSizeAdd == Catch::Approx(expected.response.pointSizeAdd));
    CHECK(actual.response.pointSizeMultiply == Catch::Approx(expected.response.pointSizeMultiply));
    CHECK(actual.response.hueShift == Catch::Approx(expected.response.hueShift));
    CHECK(actual.response.colouriseRed == Catch::Approx(expected.response.colouriseRed));
    CHECK(actual.response.colouriseGreen == Catch::Approx(expected.response.colouriseGreen));
    CHECK(actual.response.colouriseBlue == Catch::Approx(expected.response.colouriseBlue));
    CHECK(actual.response.colouriseAmount == Catch::Approx(expected.response.colouriseAmount));
    CHECK(
        actual.response.gaussianSharpnessBias ==
        Catch::Approx(expected.response.gaussianSharpnessBias));
}

WaterSeepageNode MakeNode() {
    WaterSeepageNode node;
    node.id = 47U;
    node.name = "Cliff vegetation seep";
    node.position = {12.5F, -8.25F, 3.75F};
    node.surfaceNormal = {0.15F, 0.97F, 0.19F};
    node.downAxis = {0.08F, -0.12F, -0.99F};
    node.reachMeters = 2.4F;
    node.startWidthMeters = 0.18F;
    node.endWidthMeters = 1.35F;
    node.edgeFeatherMeters = 0.21F;
    node.depthToleranceMeters = 0.32F;
    node.normalAlignment = 0.67F;
    node.strength = 1.45F;
    node.seed = 987U;
    node.enabledInViewport = false;
    node.enabledInExport = true;
    node.targetSceneRoles = {"ROCK", "VEG"};
    node.lookProfileName = "Rain-darkened cliff";
    node.lookOverride = MakeLook(WaterSeepageQuality::Low, 0.01F, WaterEffectBlendMode::Screen);
    node.tempLookOverride = MakeLook(WaterSeepageQuality::High, 0.02F, WaterEffectBlendMode::Override);
    return node;
}

void CheckNode(const WaterSeepageNode& actual, const WaterSeepageNode& expected) {
    CHECK(actual.id == expected.id);
    CHECK(actual.name == expected.name);
    CHECK(actual.position.x == Catch::Approx(expected.position.x));
    CHECK(actual.position.y == Catch::Approx(expected.position.y));
    CHECK(actual.position.z == Catch::Approx(expected.position.z));
    CHECK(actual.surfaceNormal.x == Catch::Approx(expected.surfaceNormal.x));
    CHECK(actual.surfaceNormal.y == Catch::Approx(expected.surfaceNormal.y));
    CHECK(actual.surfaceNormal.z == Catch::Approx(expected.surfaceNormal.z));
    CHECK(actual.downAxis.x == Catch::Approx(expected.downAxis.x));
    CHECK(actual.downAxis.y == Catch::Approx(expected.downAxis.y));
    CHECK(actual.downAxis.z == Catch::Approx(expected.downAxis.z));
    CHECK(actual.reachMeters == Catch::Approx(expected.reachMeters));
    CHECK(actual.startWidthMeters == Catch::Approx(expected.startWidthMeters));
    CHECK(actual.endWidthMeters == Catch::Approx(expected.endWidthMeters));
    CHECK(actual.edgeFeatherMeters == Catch::Approx(expected.edgeFeatherMeters));
    CHECK(actual.depthToleranceMeters == Catch::Approx(expected.depthToleranceMeters));
    CHECK(actual.normalAlignment == Catch::Approx(expected.normalAlignment));
    CHECK(actual.strength == Catch::Approx(expected.strength));
    CHECK(actual.seed == expected.seed);
    CHECK(actual.enabledInViewport == expected.enabledInViewport);
    CHECK(actual.enabledInExport == expected.enabledInExport);
    CHECK(actual.targetSceneRoles == expected.targetSceneRoles);
    CHECK(actual.lookProfileName == expected.lookProfileName);
    REQUIRE(actual.lookOverride.has_value());
    REQUIRE(expected.lookOverride.has_value());
    CheckLook(actual.lookOverride.value(), expected.lookOverride.value());
    REQUIRE(actual.tempLookOverride.has_value());
    REQUIRE(expected.tempLookOverride.has_value());
    CheckLook(actual.tempLookOverride.value(), expected.tempLookOverride.value());
}

std::string ReadTextFile(const std::filesystem::path& path) {
    std::ifstream input{path};
    return {
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};
}

}  // namespace

TEST_CASE("Project documents round-trip Seepage nodes and shared looks", "[water][seepage][serialization]") {
    ProjectDocument document;
    document.projectName = "Seepage serialization";
    document.waterSeepageDefaultLook =
        MakeLook(WaterSeepageQuality::Auto, 0.0F, WaterEffectBlendMode::Max);
    const WaterSeepageLookProfile profile{
        .name = "Rain-darkened cliff",
        .settings = MakeLook(
            WaterSeepageQuality::Balanced,
            0.03F,
            WaterEffectBlendMode::Multiply),
    };
    document.waterSeepageLookProfiles.push_back(profile);
    const auto node = MakeNode();
    document.waterSeepageNodes.push_back(node);

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_seepage_project.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, outputPath, &errorMessage));

    const auto savedJson = ReadTextFile(outputPath);
    CHECK(savedJson.find("\"schema_version\": 34") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_nodes\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_default_look\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_look_profiles\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"auto\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"low\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"balanced\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"high\"") != std::string::npos);
    CHECK(savedJson.find("\"depth_tolerance_meters\"") != std::string::npos);

    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == 34U);
    CheckLook(loaded->waterSeepageDefaultLook, document.waterSeepageDefaultLook);
    REQUIRE(loaded->waterSeepageLookProfiles.size() == 1U);
    CHECK(loaded->waterSeepageLookProfiles.front().name == profile.name);
    CheckLook(loaded->waterSeepageLookProfiles.front().settings, profile.settings);
    REQUIRE(loaded->waterSeepageNodes.size() == 1U);
    CheckNode(loaded->waterSeepageNodes.front(), node);
    REQUIRE(loaded->waterSceneStates.size() == 1U);
    REQUIRE(loaded->waterSceneStates.front().seepageNodes.size() == 1U);
    CheckNode(loaded->waterSceneStates.front().seepageNodes.front(), node);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Water source documents round-trip Seepage state", "[water][seepage][serialization]") {
    WaterSourcesDocument document;
    document.seepageDefaultLook =
        MakeLook(WaterSeepageQuality::High, 0.04F, WaterEffectBlendMode::Add);
    const WaterSeepageLookProfile profile{
        .name = "Subtle moss damp",
        .settings = MakeLook(WaterSeepageQuality::Low, 0.05F, WaterEffectBlendMode::Screen),
    };
    document.seepageLookProfiles.push_back(profile);
    const auto node = MakeNode();
    document.seepageNodes.push_back(node);

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_seepage_sources.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveWaterSourcesDocument(
        document,
        outputPath,
        &errorMessage));

    const auto savedJson = ReadTextFile(outputPath);
    CHECK(savedJson.find("\"schema_version\": 11") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_nodes\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_default_look\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_look_profiles\"") != std::string::npos);

    const auto loaded =
        invisible_places::serialization::LoadWaterSourcesDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == 11U);
    CheckLook(loaded->seepageDefaultLook, document.seepageDefaultLook);
    REQUIRE(loaded->seepageLookProfiles.size() == 1U);
    CHECK(loaded->seepageLookProfiles.front().name == profile.name);
    CheckLook(loaded->seepageLookProfiles.front().settings, profile.settings);
    REQUIRE(loaded->seepageNodes.size() == 1U);
    CheckNode(loaded->seepageNodes.front(), node);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy documents default missing Seepage state", "[water][seepage][serialization][legacy]") {
    const auto projectPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_no_seepage_project.json";
    {
        std::ofstream output{projectPath};
        output << R"({"schema_version":33,"project_name":"Legacy without seepage","layers":[]})";
    }
    std::string errorMessage;
    const auto loadedProject =
        invisible_places::serialization::LoadProjectDocument(projectPath, &errorMessage);
    REQUIRE(loadedProject.has_value());
    CHECK(loadedProject->schemaVersion == 34U);
    CHECK(loadedProject->waterSeepageNodes.empty());
    CHECK(loadedProject->waterSeepageLookProfiles.empty());
    CheckLook(loadedProject->waterSeepageDefaultLook, WaterSeepageLookSettings{});

    const auto sourcesPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_no_seepage_sources.json";
    {
        std::ofstream output{sourcesPath};
        output << R"({"schema_version":10})";
    }
    const auto loadedSources =
        invisible_places::serialization::LoadWaterSourcesDocument(sourcesPath, &errorMessage);
    REQUIRE(loadedSources.has_value());
    CHECK(loadedSources->schemaVersion == 10U);
    CHECK(loadedSources->seepageNodes.empty());
    CHECK(loadedSources->seepageLookProfiles.empty());
    CheckLook(loadedSources->seepageDefaultLook, WaterSeepageLookSettings{});

    std::filesystem::remove(projectPath);
    std::filesystem::remove(sourcesPath);
}
