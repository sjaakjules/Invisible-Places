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
using invisible_places::water::WaterSeepagePattern;
using invisible_places::water::WaterSeepageQuality;

WaterSeepageLookSettings MakeLook(
    WaterSeepageQuality quality,
    float offset,
    WaterEffectBlendMode blendMode) {
    WaterSeepageLookSettings look;
    look.quality = quality;
    look.pattern = quality == WaterSeepageQuality::Auto
                       ? WaterSeepagePattern::ChaoticBloom
                       : (quality == WaterSeepageQuality::Balanced
                              ? WaterSeepagePattern::WetRockSheen
                              : WaterSeepagePattern::WettingTrickle);
    look.baseWetness = 0.11F + offset;
    look.density = 0.22F + offset;
    look.glisten = 0.33F + offset;
    look.rainResponse = 0.60F + offset;
    look.featureSizeMeters = 0.19F + offset;
    look.contrast = 0.41F + offset;
    look.evolution = 0.052F + offset;
    look.roughness = 0.46F + offset;
    look.angleResponse = 0.57F + offset;
    look.microNormalStrength = 0.18F + offset;
    look.glintDensity = 0.31F + offset;
    look.environmentAzimuthDegrees = 225.0F + offset;
    look.environmentElevationDegrees = 55.0F + offset;
    look.curl = 0.42F + offset;
    look.breakup = 0.53F + offset;
    look.downhillDriftMetersPerSecond = 0.026F + offset;
    look.tricklePatchSizeMeters = 0.081F + offset;
    look.trickleWidthMeters = 0.019F + offset;
    look.trickleFrontSoftness = 0.11F + offset;
    look.pulseSpacingMeters = 0.18F + offset;
    look.pulseWidthMeters = 0.055F + offset;
    look.pulseSpeedMetersPerSecond = 0.12F + offset;
    look.pulseIrregularity = 0.38F + offset;
    look.pulseWaveCount = 7.0F + offset;
    look.pulseSpeedVariation = 0.55F + offset;
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
    CHECK(actual.pattern == expected.pattern);
    CHECK(actual.baseWetness == Catch::Approx(expected.baseWetness));
    CHECK(actual.density == Catch::Approx(expected.density));
    CHECK(actual.glisten == Catch::Approx(expected.glisten));
    CHECK(actual.rainResponse == Catch::Approx(expected.rainResponse));
    CHECK(actual.featureSizeMeters == Catch::Approx(expected.featureSizeMeters));
    CHECK(actual.contrast == Catch::Approx(expected.contrast));
    CHECK(actual.evolution == Catch::Approx(expected.evolution));
    CHECK(actual.roughness == Catch::Approx(expected.roughness));
    CHECK(actual.angleResponse == Catch::Approx(expected.angleResponse));
    CHECK(actual.microNormalStrength == Catch::Approx(expected.microNormalStrength));
    CHECK(actual.glintDensity == Catch::Approx(expected.glintDensity));
    CHECK(actual.environmentAzimuthDegrees == Catch::Approx(expected.environmentAzimuthDegrees));
    CHECK(actual.environmentElevationDegrees == Catch::Approx(expected.environmentElevationDegrees));
    CHECK(actual.curl == Catch::Approx(expected.curl));
    CHECK(actual.breakup == Catch::Approx(expected.breakup));
    CHECK(
        actual.downhillDriftMetersPerSecond ==
        Catch::Approx(expected.downhillDriftMetersPerSecond));
    CHECK(
        actual.tricklePatchSizeMeters ==
        Catch::Approx(expected.tricklePatchSizeMeters));
    CHECK(
        actual.trickleWidthMeters ==
        Catch::Approx(expected.trickleWidthMeters));
    CHECK(
        actual.trickleFrontSoftness ==
        Catch::Approx(expected.trickleFrontSoftness));
    CHECK(
        actual.pulseSpacingMeters ==
        Catch::Approx(expected.pulseSpacingMeters));
    CHECK(
        actual.pulseWidthMeters ==
        Catch::Approx(expected.pulseWidthMeters));
    CHECK(
        actual.pulseSpeedMetersPerSecond ==
        Catch::Approx(expected.pulseSpeedMetersPerSecond));
    CHECK(
        actual.pulseIrregularity ==
        Catch::Approx(expected.pulseIrregularity));
    CHECK(
        actual.pulseWaveCount ==
        Catch::Approx(expected.pulseWaveCount));
    CHECK(
        actual.pulseSpeedVariation ==
        Catch::Approx(expected.pulseSpeedVariation));
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
    node.widthMeters = 1.35F;
    node.startWidthMeters = node.widthMeters;
    node.endWidthMeters = node.widthMeters;
    node.prominence = 1.27F;
    node.selectionReachLimitMeters = 4.8F;
    node.selectionWidthLimitMeters = 2.4F;
    node.edgeFeatherMeters = 0.21F;
    node.depthToleranceMeters = 0.32F;
    node.normalAlignment = 0.67F;
    node.strength = 1.45F;
    node.seed = 987U;
    node.enabledInViewport = false;
    node.enabledInExport = true;
    node.targetSceneRoles = {"ROCK", "VEG"};
    node.lookProfileName = "Rain-darkened cliff";
    node.responseProfileName = "Strong response";
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
    CHECK(actual.widthMeters == Catch::Approx(expected.widthMeters));
    CHECK(actual.startWidthMeters == Catch::Approx(expected.widthMeters));
    CHECK(actual.endWidthMeters == Catch::Approx(expected.widthMeters));
    CHECK(actual.prominence == Catch::Approx(expected.prominence));
    CHECK(
        actual.selectionReachLimitMeters ==
        Catch::Approx(expected.selectionReachLimitMeters));
    CHECK(
        actual.selectionWidthLimitMeters ==
        Catch::Approx(expected.selectionWidthLimitMeters));
    CHECK(actual.edgeFeatherMeters == Catch::Approx(expected.edgeFeatherMeters));
    CHECK(actual.depthToleranceMeters == Catch::Approx(expected.depthToleranceMeters));
    CHECK(actual.normalAlignment == Catch::Approx(expected.normalAlignment));
    CHECK(actual.strength == Catch::Approx(expected.strength));
    CHECK(actual.seed == expected.seed);
    CHECK(actual.enabledInViewport == expected.enabledInViewport);
    CHECK(actual.enabledInExport == expected.enabledInExport);
    CHECK(actual.targetSceneRoles == expected.targetSceneRoles);
    CHECK(actual.lookProfileName == expected.lookProfileName);
    CHECK(actual.responseProfileName == expected.responseProfileName);
    // Legacy per-node overrides are migration-only inputs and never persist.
    CHECK_FALSE(actual.lookOverride.has_value());
    CHECK_FALSE(actual.tempLookOverride.has_value());
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
    document.waterSeepageLookProfiles.push_back({
        .name = "Trickle low",
        .settings = MakeLook(
            WaterSeepageQuality::Low,
            0.01F,
            WaterEffectBlendMode::Screen),
    });
    document.waterSeepageLookProfiles.push_back({
        .name = "Trickle high",
        .settings = MakeLook(
            WaterSeepageQuality::High,
            0.02F,
            WaterEffectBlendMode::Override),
    });
    auto contourPulses = MakeLook(
        WaterSeepageQuality::High,
        0.03F,
        WaterEffectBlendMode::Max);
    contourPulses.pattern = WaterSeepagePattern::ContourPulses;
    document.waterSeepageLookProfiles.push_back({
        .name = "Contour pulses",
        .settings = contourPulses,
    });
    const invisible_places::water::WaterSeepageResponseProfile responseProfile{
        .name = "Strong response",
        .response = MakeLook(
            WaterSeepageQuality::Auto,
            0.06F,
            WaterEffectBlendMode::Add).response,
        .blendMode = WaterEffectBlendMode::Add,
    };
    document.waterSeepageResponseProfiles.push_back(responseProfile);
    document.waterScenarios = invisible_places::water::DefaultWaterScenarioDefinitions();
    document.waterScenarios.front().state.seepageRainDelaySeconds = 8.0F;
    document.waterScenarios.front().state.seepageRainRiseSeconds = 14.0F;
    document.waterScenarios.front().state.seepageRainRecessionSeconds = 35.0F;
    document.selectedWaterScenarioId = "pre-colonisation-wet";
    const auto node = MakeNode();
    document.waterSeepageNodes.push_back(node);

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_seepage_project.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, outputPath, &errorMessage));

    const auto savedJson = ReadTextFile(outputPath);
    CHECK(savedJson.find(
              "\"schema_version\": " +
              std::to_string(invisible_places::serialization::kProjectDocumentSchemaVersion)) !=
          std::string::npos);
    CHECK(savedJson.find("\"water_seepage_nodes\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_default_look\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_look_profiles\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_response_profiles\"") != std::string::npos);
    CHECK(savedJson.find("\"response_profile_name\"") != std::string::npos);
    CHECK(savedJson.find("\"water_scenarios\"") != std::string::npos);
    CHECK(savedJson.find("\"selected_water_scenario\": \"pre-colonisation-wet\"") != std::string::npos);
    CHECK(savedJson.find("\"pattern\": \"wet_rock_sheen\"") != std::string::npos);
    CHECK(savedJson.find("\"pattern\": \"chaotic_bloom\"") != std::string::npos);
    CHECK(savedJson.find("\"pattern\": \"wetting_trickle\"") != std::string::npos);
    CHECK(savedJson.find("\"pattern\": \"contour_pulses\"") != std::string::npos);
    CHECK(savedJson.find("\"pulse_spacing_meters\"") != std::string::npos);
    CHECK(savedJson.find("\"pulse_speed_meters_per_second\"") != std::string::npos);
    CHECK(savedJson.find("\"pulse_wave_count\"") != std::string::npos);
    CHECK(savedJson.find("\"pulse_speed_variation\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"auto\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"low\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"balanced\"") != std::string::npos);
    CHECK(savedJson.find("\"quality\": \"high\"") != std::string::npos);
    CHECK(savedJson.find("\"width_meters\"") != std::string::npos);
    CHECK(savedJson.find("\"prominence\"") != std::string::npos);
    CHECK(savedJson.find("\"selection_reach_limit_meters\"") != std::string::npos);
    CHECK(savedJson.find("\"selection_width_limit_meters\"") != std::string::npos);
    CHECK(savedJson.find("\"start_width_meters\"") == std::string::npos);
    CHECK(savedJson.find("\"end_width_meters\"") == std::string::npos);
    CHECK(savedJson.find("\"depth_tolerance_meters\"") != std::string::npos);

    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == invisible_places::serialization::kProjectDocumentSchemaVersion);
    CheckLook(loaded->waterSeepageDefaultLook, document.waterSeepageDefaultLook);
    REQUIRE(loaded->waterSeepageLookProfiles.size() == 4U);
    CHECK(loaded->waterSeepageLookProfiles.front().name == profile.name);
    CheckLook(loaded->waterSeepageLookProfiles.front().settings, profile.settings);
    REQUIRE(loaded->waterSeepageResponseProfiles.size() == 1U);
    CHECK(loaded->waterSeepageResponseProfiles.front().name == responseProfile.name);
    CHECK(
        loaded->waterSeepageResponseProfiles.front().response.emissionAdd ==
        Catch::Approx(responseProfile.response.emissionAdd));
    CHECK(
        loaded->waterSeepageResponseProfiles.front().blendMode ==
        WaterEffectBlendMode::Add);
    REQUIRE(loaded->waterScenarios.size() == 2U);
    CHECK(loaded->selectedWaterScenarioId == "pre-colonisation-wet");
    CHECK(
        loaded->waterScenarios.front().state.seepageLook.pattern ==
        WaterSeepagePattern::ChaoticBloom);
    CHECK(
        loaded->waterScenarios.front().state.seepageRainDelaySeconds ==
        Catch::Approx(8.0F));
    CHECK(
        loaded->waterScenarios.front().state.seepageRainRiseSeconds ==
        Catch::Approx(14.0F));
    CHECK(
        loaded->waterScenarios.front().state.seepageRainRecessionSeconds ==
        Catch::Approx(35.0F));
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
    CHECK(savedJson.find(
              "\"schema_version\": " +
              std::to_string(invisible_places::serialization::kWaterSourcesDocumentSchemaVersion)) !=
          std::string::npos);
    CHECK(savedJson.find("\"water_seepage_nodes\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_default_look\"") != std::string::npos);
    CHECK(savedJson.find("\"water_seepage_look_profiles\"") != std::string::npos);

    const auto loaded =
        invisible_places::serialization::LoadWaterSourcesDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == invisible_places::serialization::kWaterSourcesDocumentSchemaVersion);
    CheckLook(loaded->seepageDefaultLook, document.seepageDefaultLook);
    REQUIRE(loaded->seepageLookProfiles.size() == 1U);
    CHECK(loaded->seepageLookProfiles.front().name == profile.name);
    CheckLook(loaded->seepageLookProfiles.front().settings, profile.settings);
    REQUIRE(loaded->seepageNodes.size() == 1U);
    CheckNode(loaded->seepageNodes.front(), node);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy per-node look overrides still parse for migration", "[water][seepage][serialization][legacy]") {
    // Overrides are never written any more, but old documents carry them and
    // the app-side migration materializes them as named profiles — so the
    // parser must keep reading them, and re-saving must drop the keys.
    const auto sourcesPath = std::filesystem::temp_directory_path() /
                             "invisible_places_seepage_legacy_override_sources.json";
    {
        std::ofstream output{sourcesPath};
        output << R"({
            "schema_version": 16,
            "water_seepage_nodes": [{
                "id": 7,
                "name": "Legacy node",
                "look_profile_name": "Wet",
                "look_override": {"base_wetness": 0.71},
                "temp_look_override": {"base_wetness": 0.83}
            }]
        })";
    }
    std::string errorMessage;
    const auto loaded =
        invisible_places::serialization::LoadWaterSourcesDocument(sourcesPath, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->seepageNodes.size() == 1U);
    const auto& node = loaded->seepageNodes.front();
    CHECK(node.lookProfileName == "Wet");
    CHECK(node.responseProfileName.empty());
    REQUIRE(node.lookOverride.has_value());
    CHECK(node.lookOverride->baseWetness == Catch::Approx(0.71F));
    REQUIRE(node.tempLookOverride.has_value());
    CHECK(node.tempLookOverride->baseWetness == Catch::Approx(0.83F));

    REQUIRE(invisible_places::serialization::SaveWaterSourcesDocument(
        loaded.value(),
        sourcesPath,
        &errorMessage));
    const auto rewrittenJson = ReadTextFile(sourcesPath);
    CHECK(rewrittenJson.find("look_override") == std::string::npos);
    CHECK(rewrittenJson.find("temp_look_override") == std::string::npos);
    CHECK(rewrittenJson.find("response_profile_name") != std::string::npos);
    std::filesystem::remove(sourcesPath);
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
    CHECK(loadedProject->schemaVersion == invisible_places::serialization::kProjectDocumentSchemaVersion);
    CHECK(loadedProject->waterSeepageNodes.empty());
    CHECK(loadedProject->waterSeepageLookProfiles.empty());
    REQUIRE(loadedProject->waterScenarios.size() == 2U);
    CHECK(loadedProject->selectedWaterScenarioId.empty());
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

TEST_CASE("Seepage looks without a pattern load as Chaotic Bloom", "[water][seepage][serialization][migration]") {
    const auto sourcesPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_seepage_pattern.json";
    {
        std::ofstream output{sourcesPath};
        output << R"({
  "schema_version": 12,
  "water_seepage_default_look": {
    "base_wetness": 0.41,
    "density": 0.52
  }
})";
    }
    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadWaterSourcesDocument(
        sourcesPath,
        &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->seepageDefaultLook.pattern == WaterSeepagePattern::ChaoticBloom);
    CHECK(loaded->seepageDefaultLook.baseWetness == Catch::Approx(0.41F));
    CHECK(loaded->seepageDefaultLook.density == Catch::Approx(0.52F));
    CHECK(loaded->seepageDefaultLook.tricklePatchSizeMeters == Catch::Approx(0.08F));
    CHECK(loaded->seepageDefaultLook.trickleWidthMeters == Catch::Approx(0.018F));
    CHECK(loaded->seepageDefaultLook.trickleFrontSoftness == Catch::Approx(0.10F));
    std::filesystem::remove(sourcesPath);
}

TEST_CASE(
    "Previous Seepage schemas load trickle and Rain timing defaults",
    "[water][seepage][serialization][migration]") {
    const auto projectPath =
        std::filesystem::temp_directory_path() /
        "invisible_places_seepage_project_v40.json";
    {
        std::ofstream output{projectPath};
        output << R"({
  "schema_version": 40,
  "water_seepage_default_look": {"pattern": "chaotic_bloom"},
  "water_scenarios": [{
    "id": "prior-scenario",
    "name": "Prior scenario",
    "state": {"rain_level": 0.5}
  }]
})";
    }

    std::string errorMessage;
    const auto loadedProject = invisible_places::serialization::LoadProjectDocument(
        projectPath,
        &errorMessage);
    REQUIRE(loadedProject.has_value());
    CHECK(
        loadedProject->schemaVersion ==
        invisible_places::serialization::kProjectDocumentSchemaVersion);
    CHECK(
        loadedProject->waterSeepageDefaultLook.pattern ==
        WaterSeepagePattern::ChaoticBloom);
    CHECK(
        loadedProject->waterSeepageDefaultLook.tricklePatchSizeMeters ==
        Catch::Approx(0.08F));
    REQUIRE(loadedProject->waterScenarios.size() == 1U);
    CHECK(
        loadedProject->waterScenarios.front().state.seepageRainDelaySeconds ==
        Catch::Approx(0.0F));
    CHECK(
        loadedProject->waterScenarios.front().state.seepageRainRiseSeconds ==
        Catch::Approx(0.0F));
    CHECK(
        loadedProject->waterScenarios.front().state.seepageRainRecessionSeconds ==
        Catch::Approx(0.0F));

    const auto sourcesPath =
        std::filesystem::temp_directory_path() /
        "invisible_places_seepage_sources_v16.json";
    {
        std::ofstream output{sourcesPath};
        output << R"({
  "schema_version": 16,
  "water_seepage_default_look": {"pattern": "chaotic_bloom"}
})";
    }
    const auto loadedSources =
        invisible_places::serialization::LoadWaterSourcesDocument(
            sourcesPath,
            &errorMessage);
    REQUIRE(loadedSources.has_value());
    CHECK(loadedSources->schemaVersion == 16U);
    CHECK(
        loadedSources->seepageDefaultLook.pattern ==
        WaterSeepagePattern::ChaoticBloom);
    CHECK(
        loadedSources->seepageDefaultLook.trickleWidthMeters ==
        Catch::Approx(0.018F));
    CHECK(
        loadedSources->seepageDefaultLook.trickleFrontSoftness ==
        Catch::Approx(0.10F));

    std::filesystem::remove(projectPath);
    std::filesystem::remove(sourcesPath);
}

TEST_CASE(
    "Schema-17 Seepage fans migrate to live dimensions and connected-selection limits",
    "[water][seepage][serialization][migration]") {
    const auto sourcesPath =
        std::filesystem::temp_directory_path() /
        "invisible_places_seepage_sources_v17_fan.json";
    {
        std::ofstream output{sourcesPath};
        output << R"({
  "schema_version": 17,
  "water_seepage_nodes": [{
    "id": 73,
    "reach_meters": 2.0,
    "start_width_meters": 0.2,
    "end_width_meters": 1.5
  }]
})";
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadWaterSourcesDocument(
        sourcesPath,
        &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->seepageNodes.size() == 1U);
    const auto& node = loaded->seepageNodes.front();
    CHECK(node.widthMeters == Catch::Approx(1.5F));
    CHECK(node.startWidthMeters == Catch::Approx(1.5F));
    CHECK(node.endWidthMeters == Catch::Approx(1.5F));
    CHECK(node.prominence == Catch::Approx(1.0F));
    CHECK(node.selectionReachLimitMeters == Catch::Approx(3.75F));
    CHECK(node.selectionWidthLimitMeters == Catch::Approx(2.43F));

    const auto migratedPath =
        std::filesystem::temp_directory_path() /
        "invisible_places_seepage_sources_v18_connected.json";
    REQUIRE(invisible_places::serialization::SaveWaterSourcesDocument(
        loaded.value(), migratedPath, &errorMessage));
    const auto migratedJson = ReadTextFile(migratedPath);
    CHECK(
        migratedJson.find(
            "\"schema_version\": " +
            std::to_string(
                invisible_places::serialization::
                    kWaterSourcesDocumentSchemaVersion)) !=
        std::string::npos);
    CHECK(migratedJson.find("\"width_meters\": 1.5") != std::string::npos);
    CHECK(migratedJson.find("\"selection_reach_limit_meters\": 3.75") !=
          std::string::npos);
    CHECK(migratedJson.find("\"start_width_meters\"") == std::string::npos);
    CHECK(migratedJson.find("\"end_width_meters\"") == std::string::npos);

    std::filesystem::remove(sourcesPath);
    std::filesystem::remove(migratedPath);
}

TEST_CASE("Animation paths round-trip normalized Seepage scenario tracks", "[water][seepage][serialization][animation]") {
    invisible_places::camera::AnimationPath path;
    path.name = "Historical water comparison";
    path.selectedWaterScenarioId = "pre-colonisation-wet";
    const auto definitions = invisible_places::water::DefaultWaterScenarioDefinitions();
    invisible_places::water::WaterScenarioTrack track;
    track.scenarioId = definitions.front().id;
    track.scenarioName = definitions.front().name;
    track.fallbackScenario = definitions.front();
    auto startState = definitions.front().state;
    auto endState = definitions.back().state;
    startState.seepageRainDelaySeconds = 4.0F;
    startState.seepageRainRiseSeconds = 12.0F;
    startState.seepageRainRecessionSeconds = 30.0F;
    endState.seepageLook.pattern = WaterSeepagePattern::WetRockSheen;
    track.keys = {
        {
            .id = "water_start",
            .position = 0.0F,
            .state = startState,
            .interpolation = invisible_places::water::WaterScenarioInterpolation::Smooth,
        },
        {
            .id = "water_end",
            .position = 1.0F,
            .state = endState,
            .interpolation = invisible_places::water::WaterScenarioInterpolation::Hold,
        },
    };
    invisible_places::water::WaterSeepageNodeTrack nodeTrack;
    nodeTrack.nodeId = 47U;
    nodeTrack.keys = {
        {
            .id = "node_wet",
            .position = 0.80F,
            .state = {
                .activity = 0.90F,
                .localSpread = 0.65F,
                .wettingProgress = 1.0F,
                .reachScale = 1.30F,
                .widthScale = 1.45F,
                .prominence = 1.60F,
            },
            .interpolation = invisible_places::water::WaterScenarioInterpolation::Hold,
        },
        {
            .id = "node_dry",
            .position = 0.20F,
            .state = {
                .activity = 0.15F,
                .localSpread = 0.05F,
                .wettingProgress = 0.10F,
                .reachScale = 0.35F,
                .widthScale = 0.25F,
                .prominence = 0.40F,
            },
            .interpolation = invisible_places::water::WaterScenarioInterpolation::Linear,
        },
    };
    track.seepageNodeTracks.push_back(nodeTrack);
    path.waterScenarioTracks.push_back(track);

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_seepage_animation.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveAnimationPath(path, outputPath, &errorMessage));
    const auto savedJson = ReadTextFile(outputPath);
    CHECK(
        savedJson.find(
            "\"schema_version\": " +
            std::to_string(
                invisible_places::serialization::kAnimationDocumentSchemaVersion)) !=
        std::string::npos);
    CHECK(savedJson.find("\"water_scenario_tracks\"") != std::string::npos);
    CHECK(savedJson.find("\"position\": 1.0") != std::string::npos);
    CHECK(savedJson.find("\"interpolation\": \"hold\"") != std::string::npos);
    CHECK(savedJson.find("\"fallback_scenario\"") != std::string::npos);
    CHECK(savedJson.find("\"seepage_node_tracks\"") != std::string::npos);
    CHECK(savedJson.find("\"reach_scale\"") != std::string::npos);
    CHECK(savedJson.find("\"width_scale\"") != std::string::npos);
    CHECK(savedJson.find("\"prominence\"") != std::string::npos);
    CHECK(savedJson.find("\"seepage_rain_delay_seconds\"") != std::string::npos);

    const auto loaded = invisible_places::serialization::LoadAnimationPath(
        outputPath,
        &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->selectedWaterScenarioId == "pre-colonisation-wet");
    REQUIRE(loaded->waterScenarioTracks.size() == 1U);
    const auto& loadedTrack = loaded->waterScenarioTracks.front();
    CHECK(loadedTrack.scenarioName == "Past/Future");
    REQUIRE(loadedTrack.keys.size() == 2U);
    CHECK(loadedTrack.keys.front().position == Catch::Approx(0.0F));
    CHECK(loadedTrack.keys.back().position == Catch::Approx(1.0F));
    CHECK(
        loadedTrack.keys.back().interpolation ==
        invisible_places::water::WaterScenarioInterpolation::Hold);
    CHECK(
        loadedTrack.keys.back().state.seepageLook.pattern ==
        WaterSeepagePattern::WetRockSheen);
    CHECK(loadedTrack.fallbackScenario.id == "pre-colonisation-wet");
    CHECK(
        loadedTrack.keys.front().state.seepageRainDelaySeconds ==
        Catch::Approx(4.0F));
    CHECK(
        loadedTrack.keys.front().state.seepageRainRiseSeconds ==
        Catch::Approx(12.0F));
    CHECK(
        loadedTrack.keys.front().state.seepageRainRecessionSeconds ==
        Catch::Approx(30.0F));
    REQUIRE(loadedTrack.seepageNodeTracks.size() == 1U);
    const auto& loadedNodeTrack = loadedTrack.seepageNodeTracks.front();
    CHECK(loadedNodeTrack.nodeId == 47U);
    REQUIRE(loadedNodeTrack.keys.size() == 2U);
    CHECK(loadedNodeTrack.keys.front().id == "node_dry");
    CHECK(loadedNodeTrack.keys.front().position == Catch::Approx(0.20F));
    CHECK(loadedNodeTrack.keys.front().state.activity == Catch::Approx(0.15F));
    CHECK(loadedNodeTrack.keys.front().state.localSpread == Catch::Approx(0.05F));
    CHECK(loadedNodeTrack.keys.front().state.wettingProgress == Catch::Approx(0.10F));
    CHECK(loadedNodeTrack.keys.front().state.reachScale == Catch::Approx(0.35F));
    CHECK(loadedNodeTrack.keys.front().state.widthScale == Catch::Approx(0.25F));
    CHECK(loadedNodeTrack.keys.front().state.prominence == Catch::Approx(0.40F));
    CHECK(
        loadedNodeTrack.keys.front().interpolation ==
        invisible_places::water::WaterScenarioInterpolation::Linear);
    CHECK(loadedNodeTrack.keys.back().id == "node_wet");
    CHECK(loadedNodeTrack.keys.back().state.activity == Catch::Approx(0.90F));
    CHECK(loadedNodeTrack.keys.back().state.localSpread == Catch::Approx(0.65F));
    CHECK(loadedNodeTrack.keys.back().state.wettingProgress == Catch::Approx(1.0F));
    CHECK(loadedNodeTrack.keys.back().state.reachScale == Catch::Approx(1.30F));
    CHECK(loadedNodeTrack.keys.back().state.widthScale == Catch::Approx(1.45F));
    CHECK(loadedNodeTrack.keys.back().state.prominence == Catch::Approx(1.60F));
    std::filesystem::remove(outputPath);
}

TEST_CASE(
    "Schema-ten Seepage animation keeps Local Spread and defaults connected factors",
    "[water][seepage][serialization][animation][legacy]") {
    const auto inputPath =
        std::filesystem::temp_directory_path() /
        "invisible_places_legacy_seepage_animation_v10.json";
    {
        std::ofstream output{inputPath};
        output << R"({
  "schema_version": 10,
  "water_scenario_tracks": [{
    "scenario_id": "legacy",
    "seepage_node_tracks": [{
      "node_id": 47,
      "keys": [{
        "id": "legacy_spread",
        "position": 0.5,
        "state": {
          "activity": 0.6,
          "local_spread": 0.7,
          "wetting_progress": 0.8
        }
      }]
    }]
  }]
})";
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadAnimationPath(
        inputPath,
        &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterScenarioTracks.size() == 1U);
    REQUIRE(loaded->waterScenarioTracks.front().seepageNodeTracks.size() == 1U);
    const auto& state = loaded->waterScenarioTracks.front()
                            .seepageNodeTracks.front()
                            .keys.front()
                            .state;
    CHECK(state.activity == Catch::Approx(0.6F));
    CHECK(state.localSpread == Catch::Approx(0.7F));
    CHECK(state.wettingProgress == Catch::Approx(0.8F));
    CHECK(state.reachScale == Catch::Approx(1.0F));
    CHECK(state.widthScale == Catch::Approx(1.0F));
    CHECK(state.prominence == Catch::Approx(1.0F));
    std::filesystem::remove(inputPath);
}

TEST_CASE(
    "Schema-nine animations default missing Seepage node tracks and Rain timing",
    "[water][seepage][serialization][animation][legacy]") {
    const auto inputPath =
        std::filesystem::temp_directory_path() /
        "invisible_places_legacy_seepage_animation_v9.json";
    {
        std::ofstream output{inputPath};
        output << R"({
  "schema_version": 9,
  "name": "Legacy static seepage",
  "water_scenario_tracks": [{
    "scenario_id": "legacy",
    "fallback_scenario": {
      "id": "legacy",
      "state": {"rain_level": 0.6}
    },
    "keys": [{
      "id": "legacy_key",
      "position": 0.5,
      "state": {"rain_level": 0.8}
    }]
  }]
})";
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadAnimationPath(
        inputPath,
        &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterScenarioTracks.size() == 1U);
    const auto& track = loaded->waterScenarioTracks.front();
    CHECK(track.seepageNodeTracks.empty());
    CHECK(
        track.fallbackScenario.state.seepageRainDelaySeconds ==
        Catch::Approx(0.0F));
    CHECK(
        track.fallbackScenario.state.seepageRainRiseSeconds ==
        Catch::Approx(0.0F));
    CHECK(
        track.fallbackScenario.state.seepageRainRecessionSeconds ==
        Catch::Approx(0.0F));
    REQUIRE(track.keys.size() == 1U);
    CHECK(track.keys.front().state.seepageRainDelaySeconds == Catch::Approx(0.0F));
    CHECK(track.keys.front().state.seepageRainRiseSeconds == Catch::Approx(0.0F));
    CHECK(track.keys.front().state.seepageRainRecessionSeconds == Catch::Approx(0.0F));
    std::filesystem::remove(inputPath);
}
