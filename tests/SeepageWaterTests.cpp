#include "water/WaterFlow.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

using invisible_places::water::WaterRainIntensityPreset;
using invisible_places::water::WaterRainSettings;
using invisible_places::water::WaterSeepageLookProfile;
using invisible_places::water::WaterSeepageLookSettings;
using invisible_places::water::WaterSeepageNode;
using invisible_places::water::WaterSeepagePattern;
using invisible_places::water::WaterSeepageQuality;
using invisible_places::water::WaterSeepageSurfaceGuide;
using invisible_places::water::WaterSeepageSpatialGrid;

WaterSeepageNode MakeSeepageNode(std::uint32_t id = 1U) {
    WaterSeepageNode node;
    node.id = id;
    node.name = "Seepage " + std::to_string(id);
    node.position = {0.0F, 0.0F, 0.0F};
    node.surfaceNormal = {0.0F, 1.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.seed = 100U + id;
    return node;
}

WaterSeepageSurfaceGuide MakeCurvedSeepageGuide(std::uint32_t nodeId = 1U) {
    WaterSeepageSurfaceGuide guide;
    guide.nodeId = nodeId;
    guide.sampleCount = 4U;
    guide.samples[0] = {{0.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, 0.0F, 1.0F};
    guide.samples[1] = {{0.0F, 0.0F, -0.40F}, {0.0F, 1.0F, 0.0F}, 0.40F, 1.0F};
    guide.samples[2] = {{0.25F, 0.0F, -0.70F}, {0.0F, 1.0F, 0.0F}, 0.7905F, 0.95F};
    guide.samples[3] = {{0.55F, 0.0F, -0.90F}, {0.0F, 1.0F, 0.0F}, 1.1511F, 0.90F};
    guide.requestedReachMeters = 1.5625F;
    guide.achievedReachMeters = guide.samples[3].station;
    guide.valid = true;
    guide.complete = false;
    return guide;
}

WaterSeepageSpatialGrid BuildGrid(
    const std::vector<WaterSeepageNode>& nodes,
    std::string_view role = "ROCK",
    bool forExport = false,
    WaterRainSettings rain = {},
    std::uint64_t effectiveInvocations = 12'000'000ULL,
    const WaterSeepageLookSettings& defaultLook = {},
    std::span<const WaterSeepageSurfaceGuide> guides = {},
    const std::optional<invisible_places::water::WaterScenarioState>& scenario = std::nullopt,
    std::span<const invisible_places::water::WaterSeepageNodeAnimationStateEntry> nodeStates = {},
    std::span<const invisible_places::water::WaterSeepageSupportSelection> support = {}) {
    return invisible_places::water::BuildWaterSeepageSpatialGrid(
        nodes,
        std::span<const WaterSeepageLookProfile>{},
        defaultLook,
        role,
        forExport,
        rain,
        effectiveInvocations,
        guides,
        scenario,
        nodeStates,
        support);
}

bool IsPowerOfTwo(std::size_t value) {
    return value > 0U && (value & (value - 1U)) == 0U;
}

}  // namespace

TEST_CASE("Seepage defaults describe a subtle damp fan", "[water][seepage][defaults]") {
    using Catch::Approx;
    using invisible_places::water::DefaultWaterSeepageLookSettings;
    using invisible_places::water::WaterEffectBlendMode;

    const auto look = DefaultWaterSeepageLookSettings();
    CHECK(look.quality == WaterSeepageQuality::Auto);
    CHECK(look.pattern == WaterSeepagePattern::ChaoticBloom);
    CHECK(look.baseWetness == Approx(0.35F));
    CHECK(look.density == Approx(0.45F));
    CHECK(look.glisten == Approx(0.55F));
    CHECK(look.rainResponse == Approx(0.50F));
    CHECK(look.tricklePatchSizeMeters == Approx(0.08F));
    CHECK(look.trickleWidthMeters == Approx(0.018F));
    CHECK(look.trickleFrontSoftness == Approx(0.10F));
    CHECK(look.pulseSpacingMeters == Approx(0.18F));
    CHECK(look.pulseWidthMeters == Approx(0.055F));
    CHECK(look.pulseSpeedMetersPerSecond == Approx(0.12F));
    CHECK(look.pulseIrregularity == Approx(0.38F));
    CHECK(look.pulseWaveCount == Approx(7.0F));
    CHECK(look.pulseSpeedVariation == Approx(0.55F));
    CHECK(look.response.intensity == Approx(0.85F));
    CHECK(look.response.emissionAdd == Approx(0.35F));
    CHECK(look.response.opacityAdd == Approx(0.04F));
    CHECK(look.response.opacityMultiply == Approx(1.12F));
    CHECK(look.response.pointSizeAdd == Approx(0.0F));
    CHECK(look.response.pointSizeMultiply == Approx(1.08F));
    CHECK(look.response.colouriseAmount == Approx(0.22F));
    CHECK(look.response.colouriseRed == Approx(0.28F));
    CHECK(look.response.colouriseGreen == Approx(0.42F));
    CHECK(look.response.colouriseBlue == Approx(0.46F));
    CHECK(look.blendMode == WaterEffectBlendMode::Max);

    const WaterSeepageNode node;
    CHECK(node.reachMeters == Approx(1.25F));
    CHECK(node.widthMeters == Approx(0.10F));
    CHECK(node.prominence == Approx(1.0F));
    CHECK(node.selectionReachLimitMeters == Approx(2.34375F));
    CHECK(node.selectionWidthLimitMeters == Approx(1.215F));
    CHECK(node.startWidthMeters == Approx(0.12F));
    CHECK(node.endWidthMeters == Approx(0.75F));
    CHECK(node.edgeFeatherMeters == Approx(0.10F));
    CHECK(node.depthToleranceMeters == Approx(0.15F));
    CHECK(node.normalAlignment == Approx(0.20F));
    CHECK(node.strength == Approx(1.0F));
    REQUIRE(node.targetSceneRoles.size() == 2U);
    CHECK(node.targetSceneRoles[0] == "ROCK");
    CHECK(node.targetSceneRoles[1] == "VEG");
}

TEST_CASE("Scalar keyed profile values update look and response fields safely",
          "[water][seepage][timing][profiles]") {
    using Catch::Approx;
    using invisible_places::water::
        ApplyWaterSeepageLookTimingValue;

    WaterSeepageLookSettings look;
    REQUIRE(
        ApplyWaterSeepageLookTimingValue(
            &look,
            "look.pulse_speed",
            0.42F));
    REQUIRE(
        ApplyWaterSeepageLookTimingValue(
            &look,
            "response.emission_add",
            1.75F));
    REQUIRE(
        ApplyWaterSeepageLookTimingValue(
            &look,
            "response.colourise_blue",
            0.21F));
    CHECK(
        look.pulseSpeedMetersPerSecond ==
        Approx(0.42F));
    CHECK(look.response.emissionAdd == Approx(1.75F));
    CHECK(look.response.colouriseBlue == Approx(0.21F));

    REQUIRE(
        ApplyWaterSeepageLookTimingValue(
            &look,
            "look.pulse_wave_count",
            99.0F));
    CHECK(look.pulseWaveCount == Approx(12.0F));
    CHECK_FALSE(
        ApplyWaterSeepageLookTimingValue(
            &look,
            "look.not_a_setting",
            1.0F));
    CHECK_FALSE(
        ApplyWaterSeepageLookTimingValue(
            nullptr,
            "look.density",
            0.5F));
}

TEST_CASE("Scalar keyed Seepage looks preserve named and local authored profile bases",
          "[water][seepage][timing][profiles][authored-base]") {
    using invisible_places::water::ApplyWaterSeepageLookTimingValue;
    using invisible_places::water::ResolveWaterSeepageLook;
    using invisible_places::water::ResolveWaterSeepageTimingLookBase;
    using invisible_places::water::WaterEffectBlendMode;
    using invisible_places::water::WaterScenarioState;
    using invisible_places::water::WaterSeepageResponseProfile;

    WaterSeepageLookSettings namedSettings;
    namedSettings.pattern = WaterSeepagePattern::WetRockSheen;
    namedSettings.baseWetness = 0.62F;
    namedSettings.glisten = 0.81F;
    namedSettings.featureSizeMeters = 0.27F;
    WaterSeepageLookSettings localSettings = namedSettings;
    localSettings.baseWetness = 0.74F;
    localSettings.glisten = 0.93F;
    localSettings.featureSizeMeters = 0.19F;
    const std::vector<WaterSeepageLookProfile> lookProfiles{
        {.name = "Wet Rock", .settings = namedSettings},
        {.name = "Wet Rock_02", .settings = localSettings},
    };
    const std::vector<WaterSeepageResponseProfile> responseProfiles{
        {.name = "Strong",
         .response = {.intensity = 1.25F,
                      .emissionAdd = 0.42F,
                      .colouriseRed = 0.17F},
         .blendMode = WaterEffectBlendMode::Screen},
        {.name = "Strong_02",
         .response = {.intensity = 1.55F,
                      .emissionAdd = 0.63F,
                      .colouriseRed = 0.29F},
         .blendMode = WaterEffectBlendMode::Add},
    };

    const auto checkAuthoredBase =
        [&](std::string lookName,
            std::string responseName,
            float expectedWetness,
            float expectedGlisten,
            float expectedFeatureSize,
            float expectedEmission,
            float expectedRed,
            WaterEffectBlendMode expectedBlend) {
            auto node = MakeSeepageNode();
            node.lookProfileName = std::move(lookName);
            node.responseProfileName = std::move(responseName);
            const auto resolved = ResolveWaterSeepageLook(
                node,
                lookProfiles,
                responseProfiles,
                {});
            auto keyed = ResolveWaterSeepageTimingLookBase(
                resolved,
                std::nullopt);
            REQUIRE(
                ApplyWaterSeepageLookTimingValue(
                    &keyed,
                    "look.density",
                    0.37F));
            CHECK(keyed.density == Catch::Approx(0.37F));
            CHECK(keyed.pattern == WaterSeepagePattern::WetRockSheen);
            CHECK(keyed.baseWetness == Catch::Approx(expectedWetness));
            CHECK(keyed.glisten == Catch::Approx(expectedGlisten));
            CHECK(
                keyed.featureSizeMeters ==
                Catch::Approx(expectedFeatureSize));
            CHECK(
                keyed.response.emissionAdd ==
                Catch::Approx(expectedEmission));
            CHECK(
                keyed.response.colouriseRed ==
                Catch::Approx(expectedRed));
            CHECK(keyed.blendMode == expectedBlend);
        };

    checkAuthoredBase(
        "Wet Rock",
        "Strong",
        0.62F,
        0.81F,
        0.27F,
        0.42F,
        0.17F,
        WaterEffectBlendMode::Screen);
    checkAuthoredBase(
        "Wet Rock_02",
        "Strong_02",
        0.74F,
        0.93F,
        0.19F,
        0.63F,
        0.29F,
        WaterEffectBlendMode::Add);

    WaterScenarioState scenario;
    scenario.seepageLook.pattern = WaterSeepagePattern::WettingTrickle;
    scenario.seepageLook.baseWetness = 0.19F;
    scenario.seepageLook.glisten = 0.28F;
    scenario.seepageLook.response.emissionAdd = 0.31F;
    const auto scenarioBase = ResolveWaterSeepageTimingLookBase(
        localSettings,
        scenario);
    CHECK(
        scenarioBase.pattern ==
        WaterSeepagePattern::WettingTrickle);
    CHECK(scenarioBase.baseWetness == Catch::Approx(0.19F));
    CHECK(scenarioBase.glisten == Catch::Approx(0.28F));
    CHECK(
        scenarioBase.response.emissionAdd ==
        Catch::Approx(0.31F));
}

TEST_CASE("Historical Seepage scenarios share visual language but differ in moisture", "[water][seepage][scenario]") {
    using Catch::Approx;
    const auto scenarios = invisible_places::water::DefaultWaterScenarioDefinitions();
    REQUIRE(scenarios.size() == 2U);
    const auto& historical = scenarios[0];
    const auto& contemporary = scenarios[1];
    CHECK(historical.id == "pre-colonisation-wet");
    CHECK(contemporary.id == "contemporary-managed");
    CHECK(historical.state.seepageLook.pattern == WaterSeepagePattern::ChaoticBloom);
    CHECK(contemporary.state.seepageLook.pattern == WaterSeepagePattern::ChaoticBloom);
    CHECK(historical.state.seepageLook.featureSizeMeters == Approx(0.20F));
    CHECK(contemporary.state.seepageLook.featureSizeMeters == Approx(0.20F));
    CHECK(historical.state.seepageLook.environmentAzimuthDegrees == Approx(225.0F));
    CHECK(contemporary.state.seepageLook.environmentElevationDegrees == Approx(55.0F));
    CHECK(historical.state.seepageLevel == Approx(1.0F));
    CHECK(contemporary.state.seepageLevel == Approx(0.50F));
    CHECK(historical.state.seepageSpread == Approx(0.60F));
    CHECK(contemporary.state.seepageSpread == Approx(0.10F));
    CHECK(historical.state.seepageRainDelaySeconds == Approx(4.0F));
    CHECK(historical.state.seepageRainRiseSeconds == Approx(12.0F));
    CHECK(historical.state.seepageRainRecessionSeconds == Approx(60.0F));
    CHECK(contemporary.state.seepageRainDelaySeconds == Approx(1.0F));
    CHECK(contemporary.state.seepageRainRiseSeconds == Approx(4.0F));
    CHECK(contemporary.state.seepageRainRecessionSeconds == Approx(15.0F));
    CHECK(historical.state.seepageLook.rainResponse > contemporary.state.seepageLook.rainResponse);
}

TEST_CASE("Seepage resolves quality from effective point invocations", "[water][seepage][quality]") {
    using invisible_places::water::ResolveWaterSeepageQuality;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;

    CHECK(ResolveWaterSeepageQuality(WaterSeepageQuality::Auto, 10'000'000ULL) ==
          WaterSeepageQuality::High);
    CHECK(ResolveWaterSeepageQuality(WaterSeepageQuality::Auto, 10'000'001ULL) ==
          WaterSeepageQuality::Balanced);
    CHECK(ResolveWaterSeepageQuality(WaterSeepageQuality::Auto, 50'000'000ULL) ==
          WaterSeepageQuality::Balanced);
    CHECK(ResolveWaterSeepageQuality(WaterSeepageQuality::Auto, 50'000'001ULL) ==
          WaterSeepageQuality::Low);
    CHECK(ResolveWaterSeepageQuality(WaterSeepageQuality::High, 500'000'000ULL) ==
          WaterSeepageQuality::High);

    const std::vector<WaterSeepageNode> nodes{MakeSeepageNode()};
    const auto highGrid = BuildGrid(nodes, "ROCK", false, {}, 1'000'000ULL);
    const auto lowGrid = BuildGrid(nodes, "ROCK", false, {}, 100'000'000ULL);
    REQUIRE(highGrid.nodes.size() == 1U);
    REQUIRE(lowGrid.nodes.size() == 1U);
    CHECK(highGrid.nodes.front().resolvedQuality == WaterSeepageQuality::High);
    CHECK(lowGrid.nodes.front().resolvedQuality == WaterSeepageQuality::Low);
    CHECK(WaterSeepageTopologyFingerprint(highGrid) == WaterSeepageTopologyFingerprint(lowGrid));
    CHECK(WaterSeepageParamsFingerprint(highGrid) != WaterSeepageParamsFingerprint(lowGrid));
}

TEST_CASE("Authored Seepage topology identity is role-local and excludes live parameters", "[water][seepage][fingerprint]") {
    using invisible_places::water::WaterSeepageAuthoredTopologyFingerprint;

    auto rockNode = MakeSeepageNode(1U);
    rockNode.targetSceneRoles = {"ROCK", "vegetation"};
    auto sandNode = MakeSeepageNode(2U);
    sandNode.position = {2.0F, 0.0F, 0.0F};
    sandNode.targetSceneRoles = {"SAND"};
    const std::vector<WaterSeepageNode> authored{rockNode, sandNode};
    const auto rockFingerprint = WaterSeepageAuthoredTopologyFingerprint(authored, "ROCK");
    const auto vegetationFingerprint =
        WaterSeepageAuthoredTopologyFingerprint(authored, "VEG");
    const auto sandFingerprint = WaterSeepageAuthoredTopologyFingerprint(authored, "SAND");

    auto reordered = authored;
    std::reverse(reordered.begin(), reordered.end());
    reordered.back().targetSceneRoles = {"VEG", "rock", "ROCK"};
    CHECK(WaterSeepageAuthoredTopologyFingerprint(reordered, "rock") == rockFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(reordered, "vegetation") ==
          vegetationFingerprint);

    auto liveParameterEdit = authored;
    liveParameterEdit.front().name = "Renamed";
    liveParameterEdit.front().enabledInViewport = false;
    liveParameterEdit.front().enabledInExport = false;
    liveParameterEdit.front().strength = 0.0F;
    liveParameterEdit.front().reachMeters = 0.25F;
    liveParameterEdit.front().widthMeters = 0.30F;
    liveParameterEdit.front().prominence = 2.0F;
    liveParameterEdit.front().normalAlignment = 0.95F;
    liveParameterEdit.front().seed += 1000U;
    liveParameterEdit.front().lookProfileName = "Alternate";
    liveParameterEdit.front().lookOverride = WaterSeepageLookSettings{};
    liveParameterEdit.front().lookOverride->density = 0.91F;
    CHECK(WaterSeepageAuthoredTopologyFingerprint(liveParameterEdit, "ROCK") ==
          rockFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(liveParameterEdit, "SAND") ==
          sandFingerprint);

    auto sandGeometryEdit = authored;
    sandGeometryEdit.back().selectionReachLimitMeters += 0.25F;
    CHECK(WaterSeepageAuthoredTopologyFingerprint(sandGeometryEdit, "ROCK") ==
          rockFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(sandGeometryEdit, "SAND") !=
          sandFingerprint);

    auto rockGeometryEdit = authored;
    rockGeometryEdit.front().position.z -= 0.10F;
    CHECK(WaterSeepageAuthoredTopologyFingerprint(rockGeometryEdit, "ROCK") !=
          rockFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(rockGeometryEdit, "SAND") ==
          sandFingerprint);

    auto roleEdit = authored;
    roleEdit.front().targetSceneRoles = {"VEG"};
    CHECK(WaterSeepageAuthoredTopologyFingerprint(roleEdit, "ROCK") != rockFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(roleEdit, "VEG") ==
          vegetationFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(roleEdit, "SAND") == sandFingerprint);

    auto addedUnrelatedRole = authored;
    addedUnrelatedRole.front().targetSceneRoles.push_back("SAND");
    CHECK(WaterSeepageAuthoredTopologyFingerprint(addedUnrelatedRole, "ROCK") ==
          rockFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(addedUnrelatedRole, "VEG") ==
          vegetationFingerprint);
    CHECK(WaterSeepageAuthoredTopologyFingerprint(addedUnrelatedRole, "SAND") !=
          sandFingerprint);
}

TEST_CASE("Seepage looks pair independent settings and response profiles", "[water][seepage][profiles]") {
    using Catch::Approx;
    using invisible_places::water::ResolveWaterSeepageLook;
    using invisible_places::water::WaterSeepageResponseProfile;

    WaterSeepageLookProfile savedSettings;
    savedSettings.name = "Wet Rock";
    savedSettings.settings.baseWetness = 0.62F;
    // The response stored inside a settings profile is ignored at resolve
    // time; only response profiles (or the default look) supply it.
    savedSettings.settings.response.emissionAdd = 9.0F;
    std::vector<WaterSeepageLookProfile> profiles{savedSettings};

    WaterSeepageResponseProfile savedResponse;
    savedResponse.name = "Strong";
    savedResponse.response.emissionAdd = 2.5F;
    savedResponse.blendMode = invisible_places::water::WaterEffectBlendMode::Add;
    std::vector<WaterSeepageResponseProfile> responseProfiles{savedResponse};

    auto node = MakeSeepageNode();
    node.lookProfileName = savedSettings.name;
    node.responseProfileName = savedResponse.name;
    const auto resolved = ResolveWaterSeepageLook(node, profiles, responseProfiles, {});
    CHECK(resolved.baseWetness == Approx(0.62F));
    CHECK(resolved.response.emissionAdd == Approx(2.5F));
    CHECK(resolved.blendMode == invisible_places::water::WaterEffectBlendMode::Add);

    // Switching the settings profile keeps the chosen response and vice
    // versa — the two halves are fully independent.
    auto settingsOnlyNode = node;
    settingsOnlyNode.lookProfileName = "Default";
    const auto settingsSwitched =
        ResolveWaterSeepageLook(settingsOnlyNode, profiles, responseProfiles, {});
    CHECK(settingsSwitched.baseWetness == Approx(WaterSeepageLookSettings{}.baseWetness));
    CHECK(settingsSwitched.response.emissionAdd == Approx(2.5F));

    auto responseOnlyNode = node;
    responseOnlyNode.responseProfileName = "Default";
    WaterSeepageLookSettings fallback;
    fallback.response.emissionAdd = 0.15F;
    const auto responseSwitched =
        ResolveWaterSeepageLook(responseOnlyNode, profiles, responseProfiles, fallback);
    CHECK(responseSwitched.baseWetness == Approx(0.62F));
    CHECK(responseSwitched.response.emissionAdd == Approx(0.15F));

    // Whitespace-bearing stored names still resolve (trimmed matching).
    auto paddedNode = node;
    paddedNode.lookProfileName = "  Wet Rock ";
    CHECK(ResolveWaterSeepageLook(paddedNode, profiles, responseProfiles, {}).baseWetness ==
          Approx(0.62F));

    auto missingProfileNode = MakeSeepageNode(2U);
    missingProfileNode.lookProfileName = "Missing Profile";
    missingProfileNode.responseProfileName = "Missing Response";
    WaterSeepageLookSettings missingFallback;
    missingFallback.baseWetness = 0.61F;
    missingFallback.response.emissionAdd = 0.35F;
    const auto missing = ResolveWaterSeepageLook(
        missingProfileNode,
        profiles,
        responseProfiles,
        missingFallback);
    CHECK(missing.baseWetness == Approx(0.61F));
    CHECK(missing.response.emissionAdd == Approx(0.35F));
}

TEST_CASE("Seepage local look names are stable and padded", "[water][seepage][naming]") {
    using invisible_places::water::WaterSeepageLocalLookName;

    CHECK(WaterSeepageLocalLookName(" saved_name ", 2U) == "saved_name_02");
    CHECK(WaterSeepageLocalLookName("Default", 3U) == "Default_03");
    CHECK(WaterSeepageLocalLookName("saved_name", 123U) == "saved_name_123");
    CHECK(WaterSeepageLocalLookName("  ", 4U) == "Default_04");
}

TEST_CASE("Seepage edited shadow profiles leave the saved profile untouched", "[water][seepage][profiles]") {
    using Catch::Approx;
    using invisible_places::water::ResolveWaterSeepageLook;

    WaterSeepageLookProfile saved;
    saved.name = "saved_name";
    saved.settings.baseWetness = 0.42F;
    std::vector<WaterSeepageLookProfile> profiles{saved};
    auto node = MakeSeepageNode(2U);
    node.lookProfileName = saved.name;

    // Editing upserts a "<base>_edited" entry and points the node at it —
    // the saved profile is never mutated, and the user can flip between the
    // two names freely.
    auto edited = ResolveWaterSeepageLook(node, profiles, {}, {});
    edited.baseWetness = 0.78F;
    profiles.push_back({.name = "saved_name_edited", .settings = edited});
    node.lookProfileName = "saved_name_edited";
    CHECK(ResolveWaterSeepageLook(node, profiles, {}, {}).baseWetness == Approx(0.78F));
    CHECK(profiles.front().settings.baseWetness == Approx(0.42F));

    node.lookProfileName = saved.name;
    CHECK(ResolveWaterSeepageLook(node, profiles, {}, {}).baseWetness == Approx(0.42F));

    // Saving copies the edited settings over the base and removes the shadow.
    profiles.front().settings = profiles.back().settings;
    profiles.pop_back();
    CHECK(ResolveWaterSeepageLook(node, profiles, {}, {}).baseWetness == Approx(0.78F));
}

TEST_CASE("Seepage derives surface-tangent gravity direction", "[water][seepage][direction]") {
    using Catch::Approx;
    using invisible_places::water::DeriveWaterSeepageDownAxis;

    const auto cliffDown = DeriveWaterSeepageDownAxis({1.0F, 0.0F, 0.0F});
    CHECK(cliffDown.x == Approx(0.0F).margin(1.0e-5F));
    CHECK(cliffDown.y == Approx(0.0F).margin(1.0e-5F));
    CHECK(cliffDown.z == Approx(-1.0F).margin(1.0e-5F));

    const auto flatFallback = DeriveWaterSeepageDownAxis(
        {0.0F, 0.0F, 1.0F},
        {1.0F, 0.0F, 0.0F});
    CHECK(flatFallback.x == Approx(1.0F).margin(1.0e-5F));
    CHECK(flatFallback.y == Approx(0.0F).margin(1.0e-5F));
    CHECK(flatFallback.z == Approx(0.0F).margin(1.0e-5F));
}

TEST_CASE("Seepage guide bends the affected fan along supported surface stations", "[water][seepage][guide]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;

    const auto guide = MakeCurvedSeepageGuide();
    const auto guidedGrid = BuildGrid(
        {MakeSeepageNode()},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        std::span<const WaterSeepageSurfaceGuide>{&guide, 1U});
    REQUIRE(guidedGrid.nodes.size() == 1U);
    CHECK(guidedGrid.nodes.front().guideValid);
    CHECK(guidedGrid.nodes.front().guideSampleCount == 4U);

    const auto aroundBend = EvaluateWaterSeepageGridContribution(
        guidedGrid,
        {0.48F, 0.0F, -0.85F},
        {0.0F, 1.0F, 0.0F},
        0.15F);
    CHECK(aroundBend.mask > 0.85F);
    CHECK(aroundBend.scale > 0.0F);

    // The envelope widens with travelled distance, so the off-centreline
    // probe sits further out than it did under the fixed-width fan.
    const auto straightBelow = EvaluateWaterSeepageGridContribution(
        guidedGrid,
        {-0.45F, 0.0F, -0.85F},
        {0.0F, 1.0F, 0.0F},
        0.15F);
    CHECK(straightBelow.scale == 0.0F);

    const auto missingNormal = EvaluateWaterSeepageGridContribution(
        guidedGrid,
        {0.48F, 0.0F, -0.85F},
        {0.0F, 0.0F, 0.0F},
        0.15F);
    CHECK(missingNormal.mask > 0.85F);
}

TEST_CASE("Seepage guide rejects points above its node and beyond incomplete support", "[water][seepage][guide]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;

    auto guide = MakeCurvedSeepageGuide();
    guide.sampleCount = 3U;
    guide.achievedReachMeters = guide.samples[2].station;
    const auto grid = BuildGrid(
        {MakeSeepageNode()},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        std::span<const WaterSeepageSurfaceGuide>{&guide, 1U});

    const auto aboveHead = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, 0.15F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(aboveHead.scale == 0.0F);

    const auto beyondSupportedTail = EvaluateWaterSeepageGridContribution(
        grid,
        {0.48F, 0.0F, -0.90F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(beyondSupportedTail.scale == 0.0F);
}

TEST_CASE("Seepage guide geometry advances topology but not parameter fingerprints", "[water][seepage][guide][revisions]") {
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;

    const auto planarGrid = BuildGrid({MakeSeepageNode()});
    auto guide = MakeCurvedSeepageGuide();
    const auto guidedGrid = BuildGrid(
        {MakeSeepageNode()},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        std::span<const WaterSeepageSurfaceGuide>{&guide, 1U});
    CHECK(WaterSeepageTopologyFingerprint(planarGrid) !=
          WaterSeepageTopologyFingerprint(guidedGrid));
    CHECK(WaterSeepageParamsFingerprint(planarGrid) ==
          WaterSeepageParamsFingerprint(guidedGrid));

    guide.samples[2].position.x += 0.03F;
    guide.samples[2].confidence -= 0.05F;
    const auto changedGuideGrid = BuildGrid(
        {MakeSeepageNode()},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        std::span<const WaterSeepageSurfaceGuide>{&guide, 1U});
    CHECK(WaterSeepageTopologyFingerprint(guidedGrid) !=
          WaterSeepageTopologyFingerprint(changedGuideGrid));
    CHECK(WaterSeepageParamsFingerprint(guidedGrid) ==
          WaterSeepageParamsFingerprint(changedGuideGrid));
}

TEST_CASE("Seepage surface-guide builder follows curved ROCK support before VEG", "[water][seepage][guide][surface]") {
    invisible_places::io::LoadedPointCloud rock;
    rock.hasNormals = true;
    invisible_places::io::LoadedPointCloud veg;
    veg.hasNormals = true;
    for (int step = 0; step <= 12; ++step) {
        const float amount = static_cast<float>(step);
        const float rockX = 0.006F * amount * amount;
        const float vegX = -0.006F * amount * amount;
        const float z = -0.075F * amount;
        const glm::vec3 tangent = glm::normalize(glm::vec3{
            0.006F * (2.0F * amount + 1.0F),
            0.0F,
            -0.075F});
        const glm::vec3 normal = glm::normalize(glm::cross(tangent, glm::vec3{0.0F, 1.0F, 0.0F}));
        for (const float y : {-0.02F, 0.0F, 0.02F}) {
            rock.positions.push_back({rockX, y, z});
            rock.normals.push_back({normal.x, normal.y, normal.z});
            rock.bounds.Expand(rock.positions.back());
            veg.positions.push_back({vegX, y, z});
            veg.normals.push_back({-normal.x, normal.y, normal.z});
            veg.bounds.Expand(veg.positions.back());
        }
    }

    const std::array<invisible_places::water::WaterSceneSupportLayer, 2> layers{{
        {.cloud = &veg, .role = "VEG", .pointSpacingMeters = 0.02F, .samplingMultiplier = 1.0F},
        {.cloud = &rock, .role = "ROCK", .pointSpacingMeters = 0.02F, .samplingMultiplier = 1.0F},
    }};
    auto node = MakeSeepageNode();
    node.surfaceNormal = rock.normals.front();
    node.reachMeters = 0.62F;
    const auto guides = invisible_places::water::BuildWaterSeepageSurfaceGuides(
        std::span<const WaterSeepageNode>{&node, 1U},
        layers,
        1024U);
    REQUIRE(guides.size() == 1U);
    const auto& guide = guides.front();
    REQUIRE(guide.valid);
    CHECK(guide.nodeId == node.id);
    CHECK(guide.sampleCount >= 2U);
    CHECK(guide.sampleCount <= invisible_places::water::kWaterSeepageMaximumGuideSamples);
    CHECK(guide.requestedReachMeters == Catch::Approx(node.reachMeters * 1.25F * 1.50F));
    CHECK(guide.achievedReachMeters > 0.25F);
    CHECK(guide.samples[guide.sampleCount - 1U].position.x > 0.05F);
    for (std::size_t index = 1U; index < guide.sampleCount; ++index) {
        CHECK(guide.samples[index].station > guide.samples[index - 1U].station);
        CHECK(guide.samples[index].position.z <= guide.samples[index - 1U].position.z + 0.0041F);
        const glm::vec3 previousNormal{
            guide.samples[index - 1U].normal.x,
            guide.samples[index - 1U].normal.y,
            guide.samples[index - 1U].normal.z};
        const glm::vec3 normal{
            guide.samples[index].normal.x,
            guide.samples[index].normal.y,
            guide.samples[index].normal.z};
        CHECK(glm::dot(previousNormal, normal) >= 0.0F);
    }
}

TEST_CASE("Seepage surface-guide builder falls back to an enabled node role", "[water][seepage][guide][roles]") {
    invisible_places::io::LoadedPointCloud vegetation;
    vegetation.hasNormals = true;
    for (int step = 0; step <= 10; ++step) {
        vegetation.positions.push_back({0.0F, 0.0F, -0.06F * static_cast<float>(step)});
        vegetation.normals.push_back({1.0F, 0.0F, 0.0F});
        vegetation.bounds.Expand(vegetation.positions.back());
    }
    const std::array<invisible_places::water::WaterSceneSupportLayer, 1> layers{{
        {.cloud = &vegetation, .role = "VEG", .pointSpacingMeters = 0.02F, .samplingMultiplier = 1.0F},
    }};
    auto node = MakeSeepageNode();
    node.targetSceneRoles = {"VEG"};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.reachMeters = 0.40F;
    const auto guides = invisible_places::water::BuildWaterSeepageSurfaceGuides(
        std::span<const WaterSeepageNode>{&node, 1U},
        layers,
        256U);
    REQUIRE(guides.size() == 1U);
    CHECK(guides.front().valid);
    CHECK(guides.front().achievedReachMeters > 0.20F);
}

TEST_CASE("Seepage surface-guide builder never uses named SAND without opt-in", "[water][seepage][guide][roles]") {
    invisible_places::io::LoadedPointCloud sand;
    sand.hasNormals = true;
    for (int step = 0; step <= 10; ++step) {
        sand.positions.push_back({0.0F, 0.0F, -0.06F * static_cast<float>(step)});
        sand.normals.push_back({1.0F, 0.0F, 0.0F});
        sand.bounds.Expand(sand.positions.back());
    }
    const std::array<invisible_places::water::WaterSceneSupportLayer, 1> layers{{
        {.cloud = &sand, .role = "SAND", .pointSpacingMeters = 0.02F, .samplingMultiplier = 1.0F},
    }};
    auto node = MakeSeepageNode();
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.reachMeters = 0.40F;
    const auto excluded = invisible_places::water::BuildWaterSeepageSurfaceGuides(
        std::span<const WaterSeepageNode>{&node, 1U},
        layers,
        256U);
    REQUIRE(excluded.size() == 1U);
    CHECK_FALSE(excluded.front().valid);

    node.targetSceneRoles.push_back("SAND");
    const auto included = invisible_places::water::BuildWaterSeepageSurfaceGuides(
        std::span<const WaterSeepageNode>{&node, 1U},
        layers,
        256U);
    REQUIRE(included.size() == 1U);
    CHECK(included.front().valid);
}

TEST_CASE("Seepage fan affects only supported points below its node", "[water][seepage][fan]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;

    const auto grid = BuildGrid({MakeSeepageNode()});
    REQUIRE(grid.nodes.size() == 1U);

    const auto inside = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(inside.mask > 0.90F);
    CHECK(inside.damp > 0.0F);
    CHECK(inside.scale > 0.0F);

    const auto above = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, 0.25F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(above.scale == 0.0F);

    const auto outsideFan = EvaluateWaterSeepageGridContribution(
        grid,
        {0.80F, 0.0F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(outsideFan.scale == 0.0F);

    const auto behindSurface = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.40F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(behindSurface.scale == 0.0F);
}

TEST_CASE("Seepage fan widens downstream and feathers its sides and tail", "[water][seepage][fan]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;

    // The band starts at the authored width at the node (half-width 0.375 m
    // here) and spreads outward with travelled distance. The node is a
    // vertical wall, so the run is reach 1.25 x 1.15 = ~1.44 m with a
    // scaled end feather. (This exercises the cache-less guided/planar
    // fallback; connected support uses the least-resistance flood.)
    auto fanNode = MakeSeepageNode();
    fanNode.widthMeters = 0.75F;
    const auto grid = BuildGrid({fanNode});
    const auto insideHead = EvaluateWaterSeepageGridContribution(
        grid,
        {0.20F, 0.0F, -0.10F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto outsideHead = EvaluateWaterSeepageGridContribution(
        grid,
        {0.55F, 0.0F, -0.10F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto sameLateralDownstream = EvaluateWaterSeepageGridContribution(
        grid,
        {0.55F, 0.0F, -1.10F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(insideHead.mask > 0.90F);
    CHECK(outsideHead.mask == 0.0F);
    CHECK(sameLateralDownstream.mask > 0.0F);

    const auto center = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -0.625F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto featheredSide = EvaluateWaterSeepageGridContribution(
        grid,
        {0.49F, 0.0F, -0.625F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto featheredTail = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -1.45F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(center.mask > 0.99F);
    CHECK(featheredSide.mask > 0.0F);
    CHECK(featheredSide.mask < center.mask);
    CHECK(featheredTail.mask > 0.0F);
    CHECK(featheredTail.mask < center.mask);
}

TEST_CASE("Seepage can require matching cliff normals", "[water][seepage][fan][normals]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;

    auto node = MakeSeepageNode();
    node.normalAlignment = 1.0F;
    const auto grid = BuildGrid({node});
    const auto aligned = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto perpendicular = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -0.55F},
        {1.0F, 0.0F, 0.0F},
        0.0F);
    CHECK(aligned.mask > 0.99F);
    CHECK(perpendicular.mask == 0.0F);
    CHECK(perpendicular.scale == 0.0F);
}

TEST_CASE("Seepage grid filters roles and viewport export enablement", "[water][seepage][roles]") {
    using invisible_places::water::ApplyWaterSeepageRuntimeParameters;
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::WaterSeepageGridHasActiveViewportEffect;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;

    auto node = MakeSeepageNode();
    CHECK(BuildGrid({node}, "ROCK").nodes.size() == 1U);
    CHECK(BuildGrid({node}, "VEG").nodes.size() == 1U);
    CHECK(BuildGrid({node}, "vegetation").nodes.size() == 1U);
    CHECK(BuildGrid({node}, "SAND").nodes.empty());
    CHECK(BuildGrid({node}, "").nodes.size() == 1U);

    auto noRoles = node;
    noRoles.targetSceneRoles.clear();
    CHECK(BuildGrid({noRoles}, "ROCK").nodes.empty());
    CHECK(BuildGrid({noRoles}, "").nodes.empty());

    auto sandOptIn = node;
    sandOptIn.targetSceneRoles.push_back("SAND");
    CHECK(BuildGrid({sandOptIn}, "SAND").nodes.size() == 1U);

    node.enabledInViewport = false;
    node.enabledInExport = true;
    auto viewportGrid = BuildGrid({node}, "ROCK", false);
    REQUIRE(viewportGrid.nodes.size() == 1U);
    CHECK(viewportGrid.nodes.front().enabledFactor == Catch::Approx(0.0F));
    CHECK_FALSE(WaterSeepageGridHasActiveViewportEffect(viewportGrid));
    CHECK(EvaluateWaterSeepageGridContribution(
              viewportGrid,
              {0.0F, 0.0F, -0.55F},
              {0.0F, 1.0F, 0.0F},
              0.0F)
              .scale == 0.0F);
    CHECK(BuildGrid({node}, "ROCK", true).nodes.size() == 1U);

    const auto topologyBefore = WaterSeepageTopologyFingerprint(viewportGrid);
    const auto paramsBefore = WaterSeepageParamsFingerprint(viewportGrid);
    node.enabledInViewport = true;
    const std::vector<WaterSeepageNode> enabledNodes{node};
    ApplyWaterSeepageRuntimeParameters(
        &viewportGrid,
        enabledNodes,
        {},
        {},
        std::nullopt,
        {},
        12'000'000ULL);
    CHECK(viewportGrid.nodes.front().enabledFactor == Catch::Approx(1.0F));
    CHECK(WaterSeepageGridHasActiveViewportEffect(viewportGrid));
    CHECK(WaterSeepageTopologyFingerprint(viewportGrid) == topologyBefore);
    CHECK(WaterSeepageParamsFingerprint(viewportGrid) != paramsBefore);
    CHECK(EvaluateWaterSeepageGridContribution(
              viewportGrid,
              {0.0F, 0.0F, -0.55F},
              {0.0F, 1.0F, 0.0F},
              0.0F)
              .scale > 0.0F);
}

TEST_CASE("Inactive Seepage runtime parameters settle without requesting live redraw", "[water][seepage][redraw]") {
    using invisible_places::water::WaterScenarioState;
    using invisible_places::water::WaterSeepageGridHasActiveViewportEffect;
    using invisible_places::water::WaterSeepageNodeAnimationStateEntry;

    const auto node = MakeSeepageNode();
    CHECK(WaterSeepageGridHasActiveViewportEffect(BuildGrid({node})));

    auto zeroStrength = node;
    zeroStrength.strength = 0.0F;
    CHECK_FALSE(WaterSeepageGridHasActiveViewportEffect(BuildGrid({zeroStrength})));

    WaterScenarioState inactiveScenario;
    inactiveScenario.seepageLevel = 0.0F;
    CHECK_FALSE(WaterSeepageGridHasActiveViewportEffect(BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        {},
        inactiveScenario)));

    const std::array<WaterSeepageNodeAnimationStateEntry, 1U> inactiveAnimation{{
        {
            .nodeId = node.id,
            .state = {
                .activity = 0.0F,
                .localSpread = 0.0F,
                .wettingProgress = 1.0F,
            },
        },
    }};
    CHECK_FALSE(WaterSeepageGridHasActiveViewportEffect(BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        {},
        std::nullopt,
        inactiveAnimation)));

    WaterSeepageLookSettings noResponse;
    noResponse.response.intensity = 0.0F;
    CHECK_FALSE(WaterSeepageGridHasActiveViewportEffect(BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        noResponse)));

    WaterScenarioState fullyInactiveTransition;
    fullyInactiveTransition.transitionLook = noResponse;
    fullyInactiveTransition.transitionAmount = 1.0F;
    CHECK_FALSE(WaterSeepageGridHasActiveViewportEffect(BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        {},
        fullyInactiveTransition)));
    fullyInactiveTransition.transitionAmount = 0.5F;
    CHECK(WaterSeepageGridHasActiveViewportEffect(BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        {},
        {},
        fullyInactiveTransition)));

    WaterSeepageLookSettings trickle;
    trickle.pattern = WaterSeepagePattern::WettingTrickle;
    const std::array<WaterSeepageNodeAnimationStateEntry, 1U> dryTrickleAnimation{{
        {
            .nodeId = node.id,
            .state = {
                .activity = 1.0F,
                .localSpread = 0.0F,
                .wettingProgress = 0.0F,
            },
        },
    }};
    CHECK_FALSE(WaterSeepageGridHasActiveViewportEffect(BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        trickle,
        {},
        std::nullopt,
        dryTrickleAnimation)));
}

TEST_CASE("Rain presets strengthen seepage without changing topology", "[water][seepage][rain]") {
    using Catch::Approx;
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::WaterRainPresetVisualStrength;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;

    CHECK(WaterRainPresetVisualStrength(WaterRainIntensityPreset::LightMist) == Approx(0.30F));
    CHECK(WaterRainPresetVisualStrength(WaterRainIntensityPreset::Rain) == Approx(0.68F));
    CHECK(WaterRainPresetVisualStrength(WaterRainIntensityPreset::HeavyDownpour) == Approx(1.0F));

    const std::vector<WaterSeepageNode> nodes{MakeSeepageNode()};
    WaterRainSettings dryRain;
    dryRain.enabled = false;
    WaterRainSettings heavyRain;
    heavyRain.enabled = true;
    heavyRain.intensityPreset = WaterRainIntensityPreset::HeavyDownpour;
    const auto dryGrid = BuildGrid(nodes, "ROCK", false, dryRain);
    const auto wetGrid = BuildGrid(nodes, "ROCK", false, heavyRain);
    REQUIRE(dryGrid.nodes.size() == 1U);
    REQUIRE(wetGrid.nodes.size() == 1U);
    CHECK(dryGrid.nodes.front().rainVisualStrength == Approx(0.0F));
    CHECK(wetGrid.nodes.front().rainVisualStrength == Approx(1.0F));
    CHECK(WaterSeepageTopologyFingerprint(dryGrid) == WaterSeepageTopologyFingerprint(wetGrid));
    CHECK(WaterSeepageParamsFingerprint(dryGrid) != WaterSeepageParamsFingerprint(wetGrid));

    const auto dry = EvaluateWaterSeepageGridContribution(
        dryGrid,
        {0.0F, 0.0F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.23F);
    const auto wet = EvaluateWaterSeepageGridContribution(
        wetGrid,
        {0.0F, 0.0F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.23F);
    CHECK(wet.damp > dry.damp);
    CHECK(wet.scale >= dry.scale);

    // The dry wall run ends near 1.44 m (+0.14 m feather); heavy rain scales
    // reach by 1.25x, carrying the run past 1.60 m.
    const auto dryBeyondBaseReach = EvaluateWaterSeepageGridContribution(
        dryGrid,
        {0.0F, 0.0F, -1.60F},
        {0.0F, 1.0F, 0.0F},
        0.23F);
    const auto wetExpandedReach = EvaluateWaterSeepageGridContribution(
        wetGrid,
        {0.0F, 0.0F, -1.60F},
        {0.0F, 1.0F, 0.0F},
        0.23F);
    CHECK(dryBeyondBaseReach.scale == 0.0F);
    CHECK(wetExpandedReach.scale > 0.0F);
}

TEST_CASE("Seepage grid remains compact and reports bounded cell overflow", "[water][seepage][grid]") {
    std::vector<WaterSeepageNode> nodes;
    for (std::uint32_t id = 1U; id <= 10U; ++id) {
        auto node = MakeSeepageNode(id);
        node.strength = static_cast<float>(id) * 0.10F;
        nodes.push_back(node);
    }
    const auto grid = BuildGrid(nodes, "ROCK", false, {}, 100'000'000ULL);
    REQUIRE(grid.nodes.size() == 10U);
    REQUIRE_FALSE(grid.hashCells.empty());
    CHECK(IsPowerOfTwo(grid.hashCells.size()));
    CHECK(grid.hashCells.size() < 1024U);
    CHECK(grid.diagnostics.overflowCellCount > 0U);
    CHECK(grid.diagnostics.droppedNodeReferenceCount > 0U);
    CHECK(grid.nodeReferences.size() <=
          static_cast<std::size_t>(grid.diagnostics.occupiedCellCount) *
              WaterSeepageSpatialGrid::kMaxReferencesPerCell);
    for (const auto& cell : grid.hashCells) {
        CHECK(cell.referenceCount <= WaterSeepageSpatialGrid::kMaxReferencesPerCell);
    }

    const auto fullCell = std::find_if(
        grid.hashCells.begin(),
        grid.hashCells.end(),
        [](const auto& cell) {
            return cell.occupied &&
                   cell.referenceCount == WaterSeepageSpatialGrid::kMaxReferencesPerCell;
        });
    REQUIRE(fullCell != grid.hashCells.end());
    std::vector<std::uint32_t> retainedIds;
    for (std::uint32_t index = 0U; index < fullCell->referenceCount; ++index) {
        const auto reference = grid.nodeReferences[fullCell->referenceOffset + index];
        REQUIRE(reference < grid.nodes.size());
        retainedIds.push_back(grid.nodes[reference].id);
    }
    CHECK(std::find(retainedIds.begin(), retainedIds.end(), 1U) != retainedIds.end());
    CHECK(std::find(retainedIds.begin(), retainedIds.end(), 2U) != retainedIds.end());
    CHECK(std::find(retainedIds.begin(), retainedIds.end(), 9U) == retainedIds.end());
    CHECK(std::find(retainedIds.begin(), retainedIds.end(), 10U) == retainedIds.end());

    auto parameterEditedNodes = nodes;
    for (std::size_t index = 0U; index < parameterEditedNodes.size(); ++index) {
        parameterEditedNodes[index].strength =
            static_cast<float>(parameterEditedNodes.size() - index) * 0.73F;
        parameterEditedNodes[index].enabledInViewport = index % 2U == 0U;
        parameterEditedNodes[index].seed += static_cast<std::uint32_t>(1000U + index);
    }
    const auto parameterEditedGrid = BuildGrid(
        parameterEditedNodes,
        "ROCK",
        false,
        {},
        100'000'000ULL);
    CHECK(parameterEditedGrid.nodeReferences == grid.nodeReferences);
    CHECK(
        invisible_places::water::WaterSeepageTopologyFingerprint(grid) ==
        invisible_places::water::WaterSeepageTopologyFingerprint(parameterEditedGrid));

    std::reverse(nodes.begin(), nodes.end());
    const auto reversedGrid = BuildGrid(nodes, "ROCK", false, {}, 100'000'000ULL);
    CHECK(invisible_places::water::WaterSeepageTopologyFingerprint(grid) ==
          invisible_places::water::WaterSeepageTopologyFingerprint(reversedGrid));
}

TEST_CASE("Overlapping seepage nodes follow their selected blend mode", "[water][seepage][blend]") {
    using Catch::Approx;
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::EvaluateWaterSeepageRuntimeContribution;
    using invisible_places::water::WaterEffectBlendMode;

    auto first = MakeSeepageNode(1U);
    auto second = MakeSeepageNode(2U);
    first.strength = 0.35F;
    second.strength = 0.45F;
    const invisible_places::io::Float3 position{0.0F, 0.0F, -0.55F};
    const invisible_places::io::Float3 normal{0.0F, 1.0F, 0.0F};

    const auto maxGrid = BuildGrid({first, second});
    REQUIRE(maxGrid.nodes.size() == 2U);
    const auto firstMax = EvaluateWaterSeepageRuntimeContribution(
        maxGrid.nodes[0], position, normal, 0.17F);
    const auto secondMax = EvaluateWaterSeepageRuntimeContribution(
        maxGrid.nodes[1], position, normal, 0.17F);
    const auto maxComposite = EvaluateWaterSeepageGridContribution(maxGrid, position, normal, 0.17F);
    CHECK(maxComposite.scale == Approx(std::max(firstMax.scale, secondMax.scale)).margin(1.0e-6F));

    WaterSeepageLookSettings additiveLook;
    additiveLook.blendMode = WaterEffectBlendMode::Add;
    const auto addGrid = BuildGrid(
        {first, second},
        "ROCK",
        false,
        {},
        12'000'000ULL,
        additiveLook);
    REQUIRE(addGrid.nodes.size() == 2U);
    const auto firstAdd = EvaluateWaterSeepageRuntimeContribution(
        addGrid.nodes[0], position, normal, 0.17F);
    const auto secondAdd = EvaluateWaterSeepageRuntimeContribution(
        addGrid.nodes[1], position, normal, 0.17F);
    const auto addComposite = EvaluateWaterSeepageGridContribution(addGrid, position, normal, 0.17F);
    CHECK(addComposite.scale == Approx(std::min(1.0F, firstAdd.scale + secondAdd.scale)).margin(1.0e-6F));
    CHECK(addComposite.scale >= maxComposite.scale);
}

TEST_CASE("Seepage auxiliary memory stays compact for 256 nodes", "[water][seepage][performance]") {
    std::vector<WaterSeepageNode> nodes;
    nodes.reserve(256U);
    for (std::uint32_t id = 1U; id <= 256U; ++id) {
        auto node = MakeSeepageNode(id);
        node.position.x = static_cast<float>((id - 1U) % 32U) * 1.5F;
        node.position.y = static_cast<float>((id - 1U) / 32U) * 1.5F;
        nodes.push_back(node);
    }
    const auto grid = BuildGrid(nodes, "ROCK", false, {}, 100'000'000ULL);
    REQUIRE(grid.nodes.size() == 256U);
    const std::size_t auxiliaryBytes =
        grid.nodes.size() * sizeof(invisible_places::water::WaterSeepageRuntimeNode) +
        grid.hashCells.size() * sizeof(invisible_places::water::WaterSeepageSpatialHashCell) +
        grid.nodeReferences.size() * sizeof(std::uint32_t);
    CHECK(auxiliaryBytes < 4U * 1024U * 1024U);
}

TEST_CASE("All Seepage patterns are deterministic and organic modes respond in world space", "[water][seepage][patterns]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::WaterSeepageViewContext;

    const invisible_places::io::Float3 position{0.09F, 0.0F, -0.63F};
    const invisible_places::io::Float3 normal{0.0F, 1.0F, 0.0F};
    const WaterSeepageViewContext frontView{
        .cameraPosition = {0.4F, -2.0F, 0.8F},
        .hasCameraPosition = true,
    };
    const WaterSeepageViewContext grazingView{
        .cameraPosition = {2.0F, -0.15F, 0.1F},
        .hasCameraPosition = true,
    };

    for (const auto pattern : {
             WaterSeepagePattern::WetRockSheen,
             WaterSeepagePattern::ChaoticBloom,
             WaterSeepagePattern::WettingTrickle,
             WaterSeepagePattern::ContourPulses}) {
        WaterSeepageLookSettings look;
        look.pattern = pattern;
        look.quality = WaterSeepageQuality::High;
        look.baseWetness = 0.65F;
        look.density = 0.80F;
        look.glisten = 1.0F;
        look.glintDensity = 1.0F;
        look.angleResponse = 1.0F;
        look.microNormalStrength = 0.75F;
        look.evolution = 0.18F;
        look.downhillDriftMetersPerSecond = 0.08F;
        const auto grid = BuildGrid(
            {MakeSeepageNode()},
            "ROCK",
            false,
            {},
            1'000'000ULL,
            look);
        const auto first = EvaluateWaterSeepageGridContribution(
            grid, position, normal, 1.37F, frontView);
        const auto repeated = EvaluateWaterSeepageGridContribution(
            grid, position, normal, 1.37F, frontView);
        CHECK(first.scale == Catch::Approx(repeated.scale).margin(1.0e-7F));
        CHECK(first.glint == Catch::Approx(repeated.glint).margin(1.0e-7F));
        CHECK(first.scale > 0.0F);

        const auto grazing = EvaluateWaterSeepageGridContribution(
            grid, position, normal, 1.37F, grazingView);
        CHECK(std::isfinite(grazing.scale));
        CHECK(std::abs(first.glint - grazing.glint) > 1.0e-6F);
        const auto missingNormal = EvaluateWaterSeepageGridContribution(
            grid,
            position,
            {},
            1.37F,
            grazingView);
        CHECK(std::isfinite(missingNormal.scale));
        CHECK(missingNormal.scale > 0.0F);
    }
}

TEST_CASE("Chaotic Bloom evolves without changing Seepage topology", "[water][seepage][patterns][chaotic]") {
    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::ChaoticBloom;
    look.quality = WaterSeepageQuality::High;
    look.density = 0.82F;
    look.evolution = 0.23F;
    look.downhillDriftMetersPerSecond = 0.12F;
    look.curl = 0.85F;
    const auto grid = BuildGrid(
        {MakeSeepageNode()},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        look);
    const auto start = invisible_places::water::EvaluateWaterSeepageGridContribution(
        grid,
        {0.11F, 0.0F, -0.72F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto later = invisible_places::water::EvaluateWaterSeepageGridContribution(
        grid,
        {0.11F, 0.0F, -0.72F},
        {0.0F, 1.0F, 0.0F},
        3.17F);
    CHECK(std::abs(start.damp - later.damp) +
              std::abs(start.ripple - later.ripple) +
              std::abs(start.glint - later.glint) >
          1.0e-5F);
}

TEST_CASE("Chaotic Bloom lobes advect downhill along the surface guide", "[water][seepage][patterns][chaotic][direction]") {
    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::ChaoticBloom;
    look.quality = WaterSeepageQuality::High;
    look.baseWetness = 0.75F;
    look.density = 0.80F;
    look.glisten = 0.0F;
    look.evolution = 0.0F;
    look.downhillDriftMetersPerSecond = 0.10F;

    auto node = MakeSeepageNode();
    node.reachMeters = 2.0F;
    node.startWidthMeters = 1.0F;
    node.endWidthMeters = 1.0F;
    const auto grid = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        look);
    const auto upstream = invisible_places::water::EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -0.40F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto downstreamLater = invisible_places::water::EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -0.50F},
        {0.0F, 1.0F, 0.0F},
        1.0F);
    CHECK(downstreamLater.damp == Catch::Approx(upstream.damp).margin(1.0e-5F));
    CHECK(downstreamLater.ripple == Catch::Approx(upstream.ripple).margin(1.0e-5F));
}

TEST_CASE("Contour Pulses build strong regions from independently drifting wave overlap", "[water][seepage][patterns][contour-pulses]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::PrepareWaterSeepagePulseFields;

    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::ContourPulses;
    look.quality = WaterSeepageQuality::High;
    look.baseWetness = 0.70F;
    look.density = 0.85F;
    look.glisten = 0.0F;
    look.evolution = 0.0F;
    look.pulseSpacingMeters = 0.50F;
    look.pulseWidthMeters = 0.025F;
    look.pulseSpeedMetersPerSecond = 0.10F;
    look.pulseIrregularity = 0.60F;
    look.pulseWaveCount = 1.0F;
    look.pulseSpeedVariation = 0.0F;

    auto node = MakeSeepageNode();
    node.reachMeters = 2.0F;
    node.widthMeters = 0.60F;
    auto singleWaveGrid = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        look);
    const invisible_places::io::Float3 normal{0.0F, 1.0F, 0.0F};
    look.pulseWaveCount = 12.0F;
    look.pulseSpeedVariation = 0.90F;
    auto variedWaveGrid = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        look);
    look.pulseSpeedVariation = 0.0F;
    auto uniformSpeedGrid = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        look);

    float maximumSingleWave = 0.0F;
    float maximumVariedWaves = 0.0F;
    float variedWaveSum = 0.0F;
    float singleWaveSum = 0.0F;
    float maximumSpeedVariationDifference = 0.0F;
    float maximumFrontDifference = 0.0F;
    for (int timeIndex = 0; timeIndex < 36; ++timeIndex) {
        const float sampleTime =
            static_cast<float>(timeIndex) * 0.31F;
        PrepareWaterSeepagePulseFields(
            &singleWaveGrid,
            sampleTime);
        PrepareWaterSeepagePulseFields(
            &variedWaveGrid,
            sampleTime);
        PrepareWaterSeepagePulseFields(
            &uniformSpeedGrid,
            sampleTime);
        for (int distanceIndex = 0;
             distanceIndex < 24;
             ++distanceIndex) {
            const float downstream =
                static_cast<float>(distanceIndex) * 0.06F;
            const invisible_places::io::Float3 point{
                0.0F,
                0.0F,
                -downstream,
            };
            const auto single =
                EvaluateWaterSeepageGridContribution(
                    singleWaveGrid,
                    point,
                    normal,
                    sampleTime);
            const auto varied =
                EvaluateWaterSeepageGridContribution(
                    variedWaveGrid,
                    point,
                    normal,
                    sampleTime);
            const auto uniform =
                EvaluateWaterSeepageGridContribution(
                    uniformSpeedGrid,
                    point,
                    normal,
                    sampleTime);
            CHECK(std::isfinite(single.ripple));
            CHECK(std::isfinite(varied.ripple));
            maximumSingleWave =
                std::max(maximumSingleWave, single.ripple);
            maximumVariedWaves =
                std::max(maximumVariedWaves, varied.ripple);
            singleWaveSum += single.ripple;
            variedWaveSum += varied.ripple;
            maximumSpeedVariationDifference = std::max(
                maximumSpeedVariationDifference,
                std::abs(varied.ripple - uniform.ripple));
        }
        const auto centre =
            EvaluateWaterSeepageGridContribution(
                variedWaveGrid,
                {0.0F, 0.0F, -0.35F},
                normal,
                sampleTime);
        const auto side =
            EvaluateWaterSeepageGridContribution(
                variedWaveGrid,
                {0.16F, 0.0F, -0.35F},
                normal,
                sampleTime);
        maximumFrontDifference = std::max(
            maximumFrontDifference,
            std::abs(centre.ripple - side.ripple));
    }

    CHECK(maximumSingleWave < 0.30F);
    CHECK(maximumVariedWaves > maximumSingleWave + 0.08F);
    CHECK(variedWaveSum > singleWaveSum * 2.0F);
    CHECK(maximumSpeedVariationDifference > 1.0e-3F);
    CHECK(maximumFrontDifference > 1.0e-4F);
}

TEST_CASE("Contour Pulses carry visible transient wetness with a dry base",
          "[water][seepage][patterns][contour-pulses][regression]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::PrepareWaterSeepagePulseFields;

    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::ContourPulses;
    look.quality = WaterSeepageQuality::High;
    look.baseWetness = 0.0F;
    look.density = 1.0F;
    look.glisten = 0.0F;
    look.pulseWaveCount = 7.0F;
    look.response.intensity = 1.0F;

    auto node = MakeSeepageNode();
    node.prominence = 3.0F;
    node.reachMeters = 2.0F;
    auto grid = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        look);
    REQUIRE(grid.nodes.size() == 1U);
    REQUIRE(grid.nodes.front().pulseField.sampleCount == 128U);

    float maximumDamp = 0.0F;
    float maximumScale = 0.0F;
    for (int timeIndex = 0; timeIndex < 24; ++timeIndex) {
        const float time = static_cast<float>(timeIndex) * 0.23F;
        PrepareWaterSeepagePulseFields(&grid, time);
        for (int distanceIndex = 1; distanceIndex < 24; ++distanceIndex) {
            const float downstream =
                static_cast<float>(distanceIndex) * 0.05F;
            const auto contribution =
                EvaluateWaterSeepageGridContribution(
                    grid,
                    {0.0F, 0.0F, -downstream},
                    {0.0F, 1.0F, 0.0F},
                    time);
            maximumDamp = std::max(
                maximumDamp,
                contribution.damp);
            maximumScale = std::max(
                maximumScale,
                contribution.scale);
        }
    }

    CHECK(maximumDamp > 0.08F);
    CHECK(maximumScale > 0.08F);
}

TEST_CASE(
    "Contour Pulse animation scrubs deterministically and matches frozen rendering",
    "[water][seepage][patterns][contour-pulses][runtime]") {
    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::ContourPulses;
    look.quality = WaterSeepageQuality::High;
    look.pulseWaveCount = 7.5F;
    look.pulseSpeedMetersPerSecond = 0.12F;
    auto grid = BuildGrid(
        {MakeSeepageNode()},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        look);
    REQUIRE(grid.nodes.size() == 1U);
    CHECK(grid.nodes.front().pulseField.sampleCount == 128U);

    const auto topologyBefore =
        invisible_places::water::WaterSeepageTopologyFingerprint(grid);
    const auto paramsBefore =
        invisible_places::water::WaterSeepageParamsFingerprint(grid);
    const auto supportReferenceCount = grid.supportReferences.size();
    const auto coarseReferenceCount = grid.nodeReferences.size();
    auto frozenGrid = grid;

    invisible_places::water::PrepareWaterSeepagePulseFields(
        &grid,
        0.83F);
    invisible_places::water::PrepareWaterSeepagePulseFields(
        &frozenGrid,
        0.83F);
    const auto preparationAtTime =
        grid.nodes.front().pulseFieldPreparationFingerprint;
    CHECK(preparationAtTime != 0U);
    CHECK(
        invisible_places::water::WaterSeepageTopologyFingerprint(grid) ==
        topologyBefore);
    CHECK(
        invisible_places::water::WaterSeepageParamsFingerprint(grid) !=
        paramsBefore);
    CHECK(grid.supportReferences.size() == supportReferenceCount);
    CHECK(grid.nodeReferences.size() == coarseReferenceCount);
    CHECK(
        invisible_places::water::WaterSeepageParamsFingerprint(grid) ==
        invisible_places::water::WaterSeepageParamsFingerprint(frozenGrid));
    CHECK(
        grid.nodes.front().pulseFieldPreparationFingerprint ==
        frozenGrid.nodes.front().pulseFieldPreparationFingerprint);
    CHECK(
        grid.nodes.front().pulseField.samples ==
        frozenGrid.nodes.front().pulseField.samples);

    const auto paramsAtTime =
        invisible_places::water::WaterSeepageParamsFingerprint(grid);
    const auto samplesAtTime = grid.nodes.front().pulseField.samples;
    invisible_places::water::PrepareWaterSeepagePulseFields(
        &grid,
        0.83F);
    CHECK(
        invisible_places::water::WaterSeepageParamsFingerprint(grid) ==
        paramsAtTime);
    CHECK(
        grid.nodes.front().pulseFieldPreparationFingerprint ==
        preparationAtTime);
    CHECK(grid.nodes.front().pulseField.samples == samplesAtTime);

    grid.nodes.front().look.pulseWaveCount = 8.25F;
    invisible_places::water::PrepareWaterSeepagePulseFields(
        &grid,
        0.83F);
    CHECK(
        grid.nodes.front().pulseFieldPreparationFingerprint !=
        preparationAtTime);
    CHECK(grid.nodes.front().pulseField.samples != samplesAtTime);
}

TEST_CASE("Water scenario tracks interpolate normalized snapshots and wrap reflection angles", "[water][seepage][scenario][animation]") {
    using Catch::Approx;
    using invisible_places::water::AddOrUpdateWaterScenarioKey;
    using invisible_places::water::EffectiveWaterFlowActivity;
    using invisible_places::water::EvaluateWaterScenarioTrack;
    using invisible_places::water::WaterScenarioInterpolation;
    using invisible_places::water::WaterScenarioKey;
    using invisible_places::water::WaterScenarioTrack;

    const auto definition = invisible_places::water::DefaultWaterScenarioDefinitions().front();
    WaterScenarioTrack track;
    track.scenarioId = definition.id;
    track.fallbackScenario = definition;
    auto startState = definition.state;
    startState.seepageLevel = 0.20F;
    startState.rainLevel = 0.10F;
    startState.flowLevel = 0.10F;
    startState.meshFlowLevel = 0.20F;
    startState.meshFlowRainGain = 0.40F;
    startState.meshFlowPersistenceScale = 0.50F;
    startState.meshFlowRainRiseSeconds = 2.0F;
    startState.meshFlowRainRecessionSeconds = 12.0F;
    startState.seepageRainDelaySeconds = 0.0F;
    startState.seepageRainRiseSeconds = 0.0F;
    startState.seepageRainRecessionSeconds = 2.0F;
    startState.seepageLook.environmentAzimuthDegrees = 350.0F;
    startState.seepageLook.tricklePatchSizeMeters = 0.05F;
    startState.seepageLook.trickleWidthMeters = 0.01F;
    startState.seepageLook.trickleFrontSoftness = 0.02F;
    auto endState = definition.state;
    endState.seepageLevel = 0.80F;
    endState.rainLevel = 0.90F;
    endState.flowLevel = 0.90F;
    endState.meshFlowLevel = 0.80F;
    endState.meshFlowRainGain = 1.20F;
    endState.meshFlowPersistenceScale = 1.50F;
    endState.meshFlowRainRiseSeconds = 8.0F;
    endState.meshFlowRainRecessionSeconds = 72.0F;
    endState.seepageRainDelaySeconds = 2.0F;
    endState.seepageRainRiseSeconds = 4.0F;
    endState.seepageRainRecessionSeconds = 6.0F;
    endState.seepageLook.environmentAzimuthDegrees = 10.0F;
    endState.seepageLook.tricklePatchSizeMeters = 0.15F;
    endState.seepageLook.trickleWidthMeters = 0.03F;
    endState.seepageLook.trickleFrontSoftness = 0.10F;
    AddOrUpdateWaterScenarioKey(
        &track,
        WaterScenarioKey{
            .id = "start",
            .position = 0.0F,
            .state = startState,
            .interpolation = WaterScenarioInterpolation::Linear,
        });
    AddOrUpdateWaterScenarioKey(
        &track,
        WaterScenarioKey{
            .id = "end",
            .position = 1.0F,
            .state = endState,
        });
    const auto middle = EvaluateWaterScenarioTrack(track, definition, 0.50F);
    CHECK(middle.seepageLevel == Approx(0.50F));
    CHECK(middle.rainLevel == Approx(0.50F));
    CHECK(middle.flowLevel == Approx(0.50F));
    CHECK(middle.meshFlowLevel == Approx(0.50F));
    CHECK(middle.meshFlowRainGain == Approx(0.80F));
    CHECK(middle.meshFlowPersistenceScale == Approx(1.0F));
    CHECK(middle.meshFlowRainRiseSeconds == Approx(5.0F));
    CHECK(middle.meshFlowRainRecessionSeconds == Approx(42.0F));
    CHECK(middle.seepageRainDelaySeconds == Approx(1.0F));
    CHECK(middle.seepageRainRiseSeconds == Approx(2.0F));
    CHECK(middle.seepageRainRecessionSeconds == Approx(4.0F));
    CHECK(middle.seepageLook.tricklePatchSizeMeters == Approx(0.10F));
    CHECK(middle.seepageLook.trickleWidthMeters == Approx(0.02F));
    CHECK(middle.seepageLook.trickleFrontSoftness == Approx(0.06F));
    CHECK(EffectiveWaterFlowActivity(middle, 0.80F, 0.75F) == Approx(0.55F));
    CHECK(std::abs(middle.seepageLook.environmentAzimuthDegrees) < 0.01F);
    CHECK(EvaluateWaterScenarioTrack(track, definition, -1.0F).seepageLevel == Approx(0.20F));
    CHECK(EvaluateWaterScenarioTrack(track, definition, 2.0F).seepageLevel == Approx(0.80F));

    track.keys.front().interpolation = WaterScenarioInterpolation::Hold;
    CHECK(EvaluateWaterScenarioTrack(track, definition, 0.75F).seepageLevel == Approx(0.20F));
    CHECK(EvaluateWaterScenarioTrack(track, definition, 0.75F).flowLevel == Approx(0.10F));
    CHECK(EvaluateWaterScenarioTrack(track, definition, 0.75F).meshFlowLevel ==
          Approx(0.20F));

    auto replacement = track.keys.front();
    replacement.id.clear();
    replacement.position = 0.00005F;
    replacement.state.seepageLevel = 0.33F;
    AddOrUpdateWaterScenarioKey(&track, replacement);
    REQUIRE(track.keys.size() == 2U);
    CHECK(track.keys.front().id == "start");
    CHECK(track.keys.front().state.seepageLevel == Approx(0.33F));

    track.keys.front().state.seepageLook.pattern = WaterSeepagePattern::WetRockSheen;
    track.keys.back().state.seepageLook.pattern = WaterSeepagePattern::ChaoticBloom;
    track.keys.front().interpolation = WaterScenarioInterpolation::Linear;
    const auto transition = EvaluateWaterScenarioTrack(track, definition, 0.5F);
    REQUIRE(transition.transitionLook.has_value());
    CHECK(transition.seepageLook.pattern == WaterSeepagePattern::WetRockSheen);
    CHECK(transition.transitionLook->pattern == WaterSeepagePattern::ChaoticBloom);
    CHECK(transition.transitionAmount == Approx(0.5F).margin(0.0001F));
}

TEST_CASE("Mesh Flow Rain envelopes preserve scenario rise and recession",
          "[water][mesh-flow][scenario][animation]") {
    using Catch::Approx;
    using invisible_places::water::BuildWaterMeshFlowRainEnvelope;
    using invisible_places::water::EffectiveWaterDynamicMeshFlowLevel;
    using invisible_places::water::EffectiveWaterDynamicMeshPersistenceSeconds;
    using invisible_places::water::EvaluateWaterMeshFlowRainEnvelope;
    using invisible_places::water::WaterDynamicMeshFlowScenarioFingerprint;
    using invisible_places::water::WaterScenarioInterpolation;
    using invisible_places::water::WaterScenarioKey;
    using invisible_places::water::WaterScenarioTrack;

    const auto definitions = invisible_places::water::DefaultWaterScenarioDefinitions();
    REQUIRE(definitions.size() == 2U);
    const auto& historical = definitions[0];
    const auto& contemporary = definitions[1];
    CHECK(historical.state.meshFlowLevel == Approx(0.45F));
    CHECK(historical.state.meshFlowRainGain == Approx(1.0F));
    CHECK(historical.state.meshFlowRainRiseSeconds == Approx(8.0F));
    CHECK(historical.state.meshFlowRainRecessionSeconds == Approx(75.0F));
    CHECK(contemporary.state.meshFlowLevel == Approx(0.18F));
    CHECK(contemporary.state.meshFlowRainGain == Approx(0.30F));
    CHECK(contemporary.state.meshFlowRainRiseSeconds == Approx(3.0F));
    CHECK(contemporary.state.meshFlowRainRecessionSeconds == Approx(18.0F));

    const auto makePulseTrack = [](const invisible_places::water::WaterScenarioDefinition& definition) {
        WaterScenarioTrack track;
        track.scenarioId = definition.id;
        track.fallbackScenario = definition;
        auto dry = definition.state;
        dry.rainLevel = 0.0F;
        auto wet = definition.state;
        wet.rainLevel = 1.0F;
        track.keys = {
            WaterScenarioKey{
                .id = "dry-start",
                .position = 0.0F,
                .state = dry,
                .interpolation = WaterScenarioInterpolation::Hold,
            },
            WaterScenarioKey{
                .id = "rain-on",
                .position = 0.10F,
                .state = wet,
                .interpolation = WaterScenarioInterpolation::Hold,
            },
            WaterScenarioKey{
                .id = "rain-off",
                .position = 0.50F,
                .state = dry,
                .interpolation = WaterScenarioInterpolation::Hold,
            },
            WaterScenarioKey{
                .id = "dry-end",
                .position = 1.0F,
                .state = dry,
                .interpolation = WaterScenarioInterpolation::Hold,
            },
        };
        return track;
    };

    const auto historicalTrack = makePulseTrack(historical);
    const auto contemporaryTrack = makePulseTrack(contemporary);
    const auto historicalEnvelope = BuildWaterMeshFlowRainEnvelope(
        historicalTrack,
        historical,
        120.0F,
        20.0F);
    const auto historicalRepeat = BuildWaterMeshFlowRainEnvelope(
        historicalTrack,
        historical,
        120.0F,
        20.0F);
    const auto contemporaryEnvelope = BuildWaterMeshFlowRainEnvelope(
        contemporaryTrack,
        contemporary,
        120.0F,
        20.0F);
    CHECK(historicalEnvelope.fingerprint == historicalRepeat.fingerprint);
    CHECK(historicalEnvelope.samples == historicalRepeat.samples);
    CHECK(
        WaterDynamicMeshFlowScenarioFingerprint(historicalTrack, historical) !=
        WaterDynamicMeshFlowScenarioFingerprint(contemporaryTrack, contemporary));

    // The contemporary scenario rises more quickly, while the historical
    // scenario retains substantially more moisture after Rain ends.
    CHECK(EvaluateWaterMeshFlowRainEnvelope(contemporaryEnvelope, 20.0F) >
          EvaluateWaterMeshFlowRainEnvelope(historicalEnvelope, 20.0F));
    CHECK(EvaluateWaterMeshFlowRainEnvelope(historicalEnvelope, 90.0F) >
          EvaluateWaterMeshFlowRainEnvelope(contemporaryEnvelope, 90.0F));

    const float historicalMoisture =
        EvaluateWaterMeshFlowRainEnvelope(historicalEnvelope, 30.0F);
    CHECK(EffectiveWaterDynamicMeshFlowLevel(
              historical.state,
              historicalMoisture) >= historical.state.meshFlowLevel);
    CHECK(EffectiveWaterDynamicMeshPersistenceSeconds(
              2.5F,
              historical.state) == Approx(2.5F));
    auto longerPersistence = historical.state;
    longerPersistence.meshFlowPersistenceScale = 2.0F;
    CHECK(EffectiveWaterDynamicMeshPersistenceSeconds(
              2.5F,
              longerPersistence) == Approx(5.0F));
}

TEST_CASE("Mesh Flow export sampling uses deterministic fixed ticks and bounded seek pre-roll",
          "[water][mesh-flow][animation][export]") {
    using Catch::Approx;
    using invisible_places::water::BuildWaterMeshFlowSampleTimeline;
    using invisible_places::water::WaterMeshFlowSampleTick;

    constexpr float stepSeconds = 1.0F / 30.0F;
    const auto direct = BuildWaterMeshFlowSampleTimeline(
        std::nullopt,
        60.0F,
        24U,
        stepSeconds);
    REQUIRE(direct.size() == 25U);
    CHECK(direct.front().resetSimulation);
    CHECK(direct.front().deltaSeconds == Approx(0.0F));
    CHECK(direct.front().timeSeconds == Approx(59.2F).margin(1.0e-4F));
    CHECK_FALSE(direct.back().resetSimulation);
    CHECK(direct.back().deltaSeconds == Approx(stepSeconds));
    CHECK(direct.back().timeSeconds == Approx(60.0F).margin(1.0e-4F));

    const auto targetTick = WaterMeshFlowSampleTick(60.0F, stepSeconds);
    CHECK(targetTick == 1'800U);
    CHECK(BuildWaterMeshFlowSampleTimeline(
              targetTick,
              60.0F,
              24U,
              stepSeconds)
              .empty());

    const auto forward = BuildWaterMeshFlowSampleTimeline(
        targetTick,
        60.1F,
        24U,
        stepSeconds);
    REQUIRE(forward.size() == 3U);
    CHECK(std::none_of(
        forward.begin(),
        forward.end(),
        [](const auto& step) {
            return step.resetSimulation;
        }));
    CHECK(forward.back().timeSeconds == Approx(60.1F).margin(1.0e-4F));

    const auto backward = BuildWaterMeshFlowSampleTimeline(
        WaterMeshFlowSampleTick(60.1F, stepSeconds),
        12.0F,
        24U,
        stepSeconds);
    REQUIRE(backward.size() == 25U);
    CHECK(backward.front().resetSimulation);
    CHECK(backward.back().timeSeconds == Approx(12.0F).margin(1.0e-4F));

    // Asking for intermediate temporal samples cannot change the fixed tick
    // sequence needed to reach the final sample.
    const auto whole = BuildWaterMeshFlowSampleTimeline(
        WaterMeshFlowSampleTick(2.0F, stepSeconds),
        2.2F,
        24U,
        stepSeconds);
    const auto firstHalf = BuildWaterMeshFlowSampleTimeline(
        WaterMeshFlowSampleTick(2.0F, stepSeconds),
        2.1F,
        24U,
        stepSeconds);
    const auto secondHalf = BuildWaterMeshFlowSampleTimeline(
        WaterMeshFlowSampleTick(2.1F, stepSeconds),
        2.2F,
        24U,
        stepSeconds);
    std::vector<float> splitTimes;
    for (const auto& step : firstHalf) {
        splitTimes.push_back(step.timeSeconds);
    }
    for (const auto& step : secondHalf) {
        splitTimes.push_back(step.timeSeconds);
    }
    REQUIRE(splitTimes.size() == whole.size());
    for (std::size_t index = 0U;
         index < whole.size();
         ++index) {
        CHECK(splitTimes[index] ==
              Approx(whole[index].timeSeconds).margin(1.0e-6F));
    }
}

TEST_CASE("Mesh Flow fixed-capacity settings sanitize and fingerprint live controls",
          "[water][mesh-flow][settings]") {
    using Catch::Approx;
    using invisible_places::water::DefaultWaterDynamicMeshFlowSettings;
    using invisible_places::water::SanitizeWaterDynamicMeshFlowSettings;
    using invisible_places::water::WaterDynamicMeshFlowSettingsFingerprint;

    const auto defaults = DefaultWaterDynamicMeshFlowSettings();
    CHECK(defaults.particleCapacity == 4096U);
    CHECK(defaults.historyLength == 24U);
    CHECK(defaults.sourceBandWidthMeters == Approx(0.75F));
    CHECK(defaults.sourceBandFraction == Approx(0.04F));
    CHECK(defaults.edgeCoverage == Approx(0.0F));
    CHECK(defaults.surfaceSurge == Approx(0.6F));
    CHECK(defaults.rainDistributedSourceFraction == Approx(0.55F));
    CHECK(defaults.speedMetersPerSecond == Approx(0.26F));
    CHECK(defaults.surfaceOffsetMeters == Approx(0.003F));
    CHECK(defaults.inertia == Approx(0.88F));
    CHECK(defaults.trailOpacityDry == Approx(0.025F));
    CHECK(defaults.trailOpacityWet == Approx(0.14F));
    CHECK(defaults.trailEmissionDry == Approx(0.04F));
    CHECK(defaults.trailEmissionWet == Approx(0.45F));
    CHECK(defaults.trailExposure == Approx(1.25F));
    CHECK(defaults.rockResponse.persistenceSeconds == Approx(2.5F));
    CHECK(defaults.vegetationResponse.twinkle == Approx(1.4F));

    auto invalid = defaults;
    invalid.particleCapacity = 0U;
    invalid.historyLength = 1U;
    invalid.sourceBandWidthMeters = -1.0F;
    invalid.edgeCoverage = 2.0F;
    invalid.surfaceSurge = -1.0F;
    invalid.rockResponse.radiusMeters = 5.0F;
    invalid.rockResponse.colourise.x = -2.0F;
    invalid.vegetationResponse.twinkle = 9.0F;
    invalid.vegetationResponse.streamDepthMeters = 8.0F;
    const auto sanitized = SanitizeWaterDynamicMeshFlowSettings(invalid);
    CHECK(sanitized.particleCapacity == 4096U);
    CHECK(sanitized.historyLength == 24U);
    CHECK(sanitized.sourceBandWidthMeters == Approx(0.75F));
    CHECK(sanitized.edgeCoverage == Approx(1.0F));
    CHECK(sanitized.surfaceSurge == Approx(0.0F));
    CHECK(sanitized.rockResponse.radiusMeters == Approx(0.75F));
    CHECK(sanitized.rockResponse.colourise.x == Approx(0.0F));
    CHECK(sanitized.vegetationResponse.twinkle == Approx(4.0F));
    CHECK(sanitized.vegetationResponse.streamDepthMeters == Approx(2.0F));

    const auto defaultFingerprint =
        WaterDynamicMeshFlowSettingsFingerprint(defaults);
    CHECK(defaultFingerprint ==
          WaterDynamicMeshFlowSettingsFingerprint(defaults));
    auto changed = defaults;
    changed.sharedWindStrength += 0.01F;
    CHECK(defaultFingerprint !=
          WaterDynamicMeshFlowSettingsFingerprint(changed));
    changed = defaults;
    changed.rockResponse.emissionAdd += 0.01F;
    CHECK(defaultFingerprint !=
          WaterDynamicMeshFlowSettingsFingerprint(changed));
    changed = defaults;
    changed.trailOpacityWet += 0.01F;
    CHECK(defaultFingerprint !=
          WaterDynamicMeshFlowSettingsFingerprint(changed));
    changed = defaults;
    changed.edgeCoverage = 0.5F;
    CHECK(defaultFingerprint !=
          WaterDynamicMeshFlowSettingsFingerprint(changed));
    changed = defaults;
    changed.surfaceSurge = 0.2F;
    CHECK(defaultFingerprint !=
          WaterDynamicMeshFlowSettingsFingerprint(changed));

    auto legacySources = defaults;
    legacySources.automaticSources = false;
    legacySources.particleCapacity = 8192U;
    legacySources.historyLength = 48U;
    legacySources.attractors.push_back({
        .id = 17U,
        .name = "Legacy Mesh attractor",
        .position = {12.0F, 8.0F, 4.0F},
        .radiusMeters = 2.0F,
        .strength = 3.0F,
        .enabled = true,
    });
    legacySources.emitterMotions.push_back({
        .emitterId = 91U,
        .name = "Legacy ordinary-Flow emitter motion",
        .enabled = true,
        .keyframes = {{
            .timeSeconds = 1.0F,
            .position = {40.0F, 30.0F, 20.0F},
        }},
    });
    const auto automatic = SanitizeWaterDynamicMeshFlowSettings(legacySources);
    CHECK(automatic.automaticSources);
    CHECK(automatic.attractors.empty());
    CHECK(automatic.emitterMotions.empty());
    CHECK(automatic.particleCapacity == 4096U);
    CHECK(automatic.historyLength == 24U);
    CHECK(defaultFingerprint ==
          WaterDynamicMeshFlowSettingsFingerprint(legacySources));
}

TEST_CASE("Mesh Flow light and heavy visual regimes preserve rills without topology work",
          "[water][mesh-flow][visual][gpu]") {
    using Catch::Approx;
    using invisible_places::water::DefaultWaterDynamicMeshFlowSettings;
    using invisible_places::water::EvaluateWaterDynamicMeshFlowVisualWeights;

    const auto settings = DefaultWaterDynamicMeshFlowSettings();
    const auto dryOpen = EvaluateWaterDynamicMeshFlowVisualWeights(
        settings,
        0.0F,
        0.0F,
        1.0F);
    const auto dryRill = EvaluateWaterDynamicMeshFlowVisualWeights(
        settings,
        1.0F,
        0.0F,
        1.0F);
    const auto heavyOpen = EvaluateWaterDynamicMeshFlowVisualWeights(
        settings,
        0.0F,
        1.0F,
        1.0F);
    const auto heavyRill = EvaluateWaterDynamicMeshFlowVisualWeights(
        settings,
        1.0F,
        1.0F,
        1.0F);

    // Light flow is overwhelmingly selected from convergent/concave cells.
    CHECK(dryRill.automaticSpawnAcceptance >
          dryOpen.automaticSpawnAcceptance * 10.0F);
    CHECK(dryRill.trailProminence >
          dryOpen.trailProminence * 7.0F);

    // Heavy flow admits distributed understory trickles while preserving a
    // clearly stronger, wider, longer rill signal in convergent cells.
    CHECK(heavyOpen.automaticSpawnAcceptance == Approx(1.0F));
    CHECK(heavyRill.automaticSpawnAcceptance == Approx(1.0F));
    // The flat +X entry band keeps a stable sub-pixel coverage floor; the
    // fixed dry/wet opacity and emission controls provide the intensity
    // separation without a second visibility suppression.
    CHECK(heavyOpen.trailProminence ==
          Approx(dryOpen.trailProminence));
    CHECK(heavyRill.trailProminence >
          heavyOpen.trailProminence * 2.0F);
    CHECK(heavyRill.trailWidthScale > heavyOpen.trailWidthScale * 1.5F);
    CHECK(heavyRill.trailStreakScale > heavyOpen.trailStreakScale * 1.3F);

    // Rill noise is suppressed rather than removed, keeping paths coherent
    // without turning them into regular splines.
    CHECK(heavyRill.directionalNoiseScale < dryRill.directionalNoiseScale);
    CHECK(heavyRill.directionalNoiseScale > 0.20F);
    CHECK(heavyOpen.directionalNoiseScale == Approx(1.0F));

    const auto weakCacheRill = EvaluateWaterDynamicMeshFlowVisualWeights(
        settings,
        1.0F,
        1.0F,
        0.0F);
    CHECK(weakCacheRill.trailProminence < heavyRill.trailProminence);
    CHECK(weakCacheRill.trailProminence > 0.45F);

    // Edge Coverage overrides the dry concentration: full coverage accepts
    // spawn candidates along the whole rim even in a dry scene, and half
    // coverage sits strictly between the focused and open extremes.
    auto fullCoverage = settings;
    fullCoverage.edgeCoverage = 1.0F;
    const auto dryOpenFullCoverage =
        EvaluateWaterDynamicMeshFlowVisualWeights(
            fullCoverage,
            0.0F,
            0.0F,
            1.0F);
    CHECK(dryOpenFullCoverage.automaticSpawnAcceptance == Approx(1.0F));
    auto halfCoverage = settings;
    halfCoverage.edgeCoverage = 0.5F;
    const auto dryOpenHalfCoverage =
        EvaluateWaterDynamicMeshFlowVisualWeights(
            halfCoverage,
            0.0F,
            0.0F,
            1.0F);
    CHECK(dryOpenHalfCoverage.automaticSpawnAcceptance >
          dryOpen.automaticSpawnAcceptance);
    CHECK(dryOpenHalfCoverage.automaticSpawnAcceptance < 1.0F);
    // Coverage reshapes spawn placement only; the visual regime weights
    // stay untouched so the calibrated light/heavy envelope holds.
    CHECK(dryOpenFullCoverage.trailProminence ==
          Approx(dryOpen.trailProminence));
    CHECK(dryOpenFullCoverage.directionalNoiseScale ==
          Approx(dryOpen.directionalNoiseScale));
}

TEST_CASE("Mesh Flow entries follow the curved positive-X rim with geodesic distances",
          "[water][mesh-flow][ground][entry]") {
    using Catch::Approx;
    using invisible_places::water::BuildWaterDynamicMeshFlowGroundEntries;
    using invisible_places::water::WaterGroundCell;
    using invisible_places::water::WaterSurfaceCache;
    using invisible_places::water::kWaterGroundTerminalContactFlag;
    using invisible_places::water::kWaterGroundVegetationSupportedFlag;

    WaterSurfaceCache cache;
    cache.resolutionMeters = 0.010F;
    const auto cell =
        [](std::int32_t x,
           std::int32_t y,
           std::uint32_t component,
           std::uint32_t flags) {
            WaterGroundCell result;
            result.cellX = x;
            result.cellY = y;
            result.componentId = component;
            result.flags = flags;
            result.connectivityMask = 1U;
            return result;
        };

    // One connected surface whose +X rim bends: a wide arm (rows 0..4
    // reaching x = 140) and a narrow arm (rows 5..9 stopping at x = 40).
    // The retired per-component maximum-X band measured the narrow arm's
    // rim as 1.0 m behind the component edge and dropped it entirely.
    for (std::int32_t y = 0; y <= 4; ++y) {
        for (std::int32_t x = 0; x <= 140; ++x) {
            cache.groundCells.push_back(
                cell(x, y, 1U, kWaterGroundVegetationSupportedFlag));
        }
    }
    for (std::int32_t y = 5; y <= 9; ++y) {
        for (std::int32_t x = 0; x <= 40; ++x) {
            cache.groundCells.push_back(
                cell(x, y, 1U, kWaterGroundVegetationSupportedFlag));
        }
    }
    // A qualifying bench separated from every free rim by terminal rock
    // skin (a contact row above it and a contact column on its +X side).
    // Distance must cross the skin at a cost premium instead of stranding
    // the bench without sources.
    for (std::int32_t x = 0; x <= 40; ++x) {
        cache.groundCells.push_back(
            cell(x, 10, 1U, kWaterGroundTerminalContactFlag));
    }
    for (std::int32_t y = 11; y <= 13; ++y) {
        cache.groundCells.push_back(
            cell(41, y, 1U, kWaterGroundTerminalContactFlag));
        for (std::int32_t x = 0; x <= 40; ++x) {
            cache.groundCells.push_back(
                cell(x, y, 1U, kWaterGroundVegetationSupportedFlag));
        }
    }
    // A second component whose entire +X boundary is terminal rock skin:
    // the rim fallback must seed its highest qualifying column instead.
    for (std::int32_t x = 200; x <= 210; ++x) {
        cache.groundCells.push_back(cell(
            x,
            50,
            2U,
            x == 210
                ? (kWaterGroundVegetationSupportedFlag |
                   kWaterGroundTerminalContactFlag)
                : kWaterGroundVegetationSupportedFlag));
    }
    // A vegetation-supported but disconnected column is never an automatic
    // route source even though it forms its own rim.
    cache.groundCells.push_back(
        cell(300, 0, 3U, kWaterGroundVegetationSupportedFlag));
    cache.groundCells.back().connectivityMask = 0U;
    std::sort(
        cache.groundCells.begin(),
        cache.groundCells.end(),
        [](const WaterGroundCell& left, const WaterGroundCell& right) {
            return std::tie(left.cellX, left.cellY) <
                   std::tie(right.cellX, right.cellY);
        });

    const auto entries = BuildWaterDynamicMeshFlowGroundEntries(cache);
    // Every vegetation-supported, non-terminal, connected cell qualifies:
    // 5x141 + 5x41 + 3x41 bench + 10 (terminal cells excluded,
    // disconnected excluded).
    REQUIRE(entries.size() == 1043U);

    const auto findEntry = [&](std::int32_t x, std::int32_t y)
        -> const invisible_places::water::WaterDynamicMeshFlowGroundEntry* {
        for (const auto& entry : entries) {
            if (entry.cellX == x && entry.cellY == y) {
                return &entry;
            }
        }
        return nullptr;
    };

    // Both stretches of the curved rim carry distance-zero entries.
    const auto* wideRim = findEntry(140, 2);
    REQUIRE(wideRim != nullptr);
    CHECK(wideRim->edgeDistanceMeters == Approx(0.0F));
    CHECK(wideRim->edgeDistanceFraction == Approx(0.0F));
    const auto* narrowRim = findEntry(40, 8);
    REQUIRE(narrowRim != nullptr);
    CHECK(narrowRim->edgeDistanceMeters == Approx(0.0F));
    // (40, 5) touches the wide arm diagonally at (41, 4), so it is one
    // orthogonal step behind the rim rather than on it.
    const auto* nearRim = findEntry(40, 5);
    REQUIRE(nearRim != nullptr);
    CHECK(nearRim->edgeDistanceMeters == Approx(0.010F));

    // Geodesic distance reaches the NEAREST rim: (0, 2) is 1.40 m from the
    // wide rim but only 4 diagonal + 36 orthogonal steps from the narrow
    // rim cell (40, 6).
    const auto* interior = findEntry(0, 2);
    REQUIRE(interior != nullptr);
    CHECK(interior->edgeDistanceMeters ==
          Approx((4.0F * 14.0F + 36.0F * 10.0F) / 1000.0F));
    const auto* wideInterior = findEntry(100, 2);
    REQUIRE(wideInterior != nullptr);
    CHECK(wideInterior->edgeDistanceMeters == Approx(0.40F));

    // The skin-ringed bench gets sources through the terminal row at three
    // times the orthogonal step cost: rim (40,9) -> terminal (40,10) costs
    // 30, then -> bench (40,11) costs 10.
    const auto* benchEdge = findEntry(40, 11);
    REQUIRE(benchEdge != nullptr);
    CHECK(benchEdge->edgeDistanceMeters == Approx(0.040F));
    const auto* benchInterior = findEntry(20, 12);
    REQUIRE(benchInterior != nullptr);
    CHECK(benchInterior->edgeDistanceMeters > 0.0F);
    CHECK(findEntry(40, 10) == nullptr);

    // The terminal-skin component seeds from its highest qualifying column.
    const auto* fallbackSeed = findEntry(209, 50);
    REQUIRE(fallbackSeed != nullptr);
    CHECK(fallbackSeed->edgeDistanceMeters == Approx(0.0F));
    const auto* fallbackBack = findEntry(200, 50);
    REQUIRE(fallbackBack != nullptr);
    CHECK(fallbackBack->edgeDistanceMeters == Approx(0.09F));
    CHECK(fallbackBack->edgeDistanceFraction == Approx(1.0F));
    CHECK(findEntry(210, 50) == nullptr);
    CHECK(findEntry(300, 0) == nullptr);

    // Entries stay ordered by 0.10 m distance band so the GPU sampler's
    // index bias means "near the rim", spread along the whole edge.
    for (std::size_t index = 1U; index < entries.size(); ++index) {
        const auto band = [](float meters) {
            return static_cast<std::uint32_t>(
                       std::lround(meters * 1000.0F)) /
                   100U;
        };
        CHECK(band(entries[index - 1U].edgeDistanceMeters) <=
              band(entries[index].edgeDistanceMeters));
    }

    // Fractions normalise against the component's farthest qualifying cell.
    for (const auto& entry : entries) {
        CHECK(entry.edgeDistanceFraction >= 0.0F);
        CHECK(entry.edgeDistanceFraction <= 1.0F);
    }
}

TEST_CASE("Water Flow activity combines keyed level and Rain response deterministically", "[water][flow][scenario][animation]") {
    using Catch::Approx;
    using invisible_places::water::EffectiveWaterFlowActivity;
    using invisible_places::water::WaterScenarioState;

    WaterScenarioState state;
    CHECK(state.flowLevel == Approx(1.0F));
    CHECK(EffectiveWaterFlowActivity(state, 0.70F, 1.0F) == Approx(0.70F));

    state.flowLevel = 0.20F;
    state.rainLevel = 0.50F;
    CHECK(EffectiveWaterFlowActivity(state, 0.80F, 0.75F) == Approx(0.40F));

    state.flowLevel = 0.0F;
    state.rainLevel = 1.0F;
    CHECK(EffectiveWaterFlowActivity(state, 0.35F, 1.0F) == Approx(0.35F));
    CHECK(EffectiveWaterFlowActivity(state, 0.35F, 0.0F) == Approx(0.0F));

    state.flowLevel = 2.0F;
    state.rainLevel = -1.0F;
    CHECK(EffectiveWaterFlowActivity(state, 2.0F, -1.0F) == Approx(1.0F));

    CHECK(EffectiveWaterFlowActivity(state, 1.0F, 1.0F, false, true) == Approx(0.0F));
    CHECK(EffectiveWaterFlowActivity(state, 1.0F, 1.0F, true, false) == Approx(0.0F));
    CHECK(EffectiveWaterFlowActivity(state, 1.0F, 1.0F, true, true) == Approx(1.0F));

    const invisible_places::water::WaterEmitter emitter;
    const invisible_places::water::WaterManualFlowPathSource manual;
    CHECK(emitter.showTrail);
    CHECK(manual.showTrail);
}

TEST_CASE("Applying scenario keys changes only compact Seepage parameters", "[water][seepage][scenario][topology]") {
    using invisible_places::water::ApplyWaterSeepageScenarioParameters;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;

    auto grid = BuildGrid({MakeSeepageNode()});
    const auto topologyBefore = WaterSeepageTopologyFingerprint(grid);
    const auto paramsBefore = WaterSeepageParamsFingerprint(grid);
    auto scenario = invisible_places::water::DefaultWaterScenarioDefinitions().front().state;
    scenario.rainLevel = 0.72F;
    ApplyWaterSeepageScenarioParameters(&grid, scenario, {}, 100'000'000ULL);
    CHECK(WaterSeepageTopologyFingerprint(grid) == topologyBefore);
    CHECK(WaterSeepageParamsFingerprint(grid) != paramsBefore);
    REQUIRE(grid.nodes.size() == 1U);
    CHECK(grid.nodes.front().resolvedQuality == WaterSeepageQuality::Low);
    CHECK(grid.nodes.front().scenarioSpread == Catch::Approx(0.60F));
    CHECK(grid.nodes.front().rainVisualStrength == Catch::Approx(0.72F));
}

TEST_CASE("Keyed wetting progress reveals Contour Pulses and Wetting Trickle consistently",
          "[water][seepage][patterns][wetting-progress]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::PrepareWaterSeepagePulseFields;
    using invisible_places::water::WaterSeepageNodeAnimationStateEntry;

    const auto nodeState = [](float progress) {
        return std::array<WaterSeepageNodeAnimationStateEntry, 1U>{{
            {
                .nodeId = 1U,
                .state = {
                    .activity = 1.0F,
                    .localSpread = 0.0F,
                    .wettingProgress = progress,
                },
            },
        }};
    };
    constexpr std::array patterns{
        WaterSeepagePattern::ContourPulses,
        WaterSeepagePattern::WettingTrickle,
    };
    for (const auto pattern : patterns) {
        INFO("pattern " << static_cast<std::uint32_t>(pattern));
        WaterSeepageLookSettings look;
        look.pattern = pattern;
        look.quality = WaterSeepageQuality::High;
        look.baseWetness = 1.0F;
        look.density = 1.0F;
        look.glisten = 0.0F;
        look.tricklePatchSizeMeters = 0.12F;
        look.trickleWidthMeters = 0.03F;
        look.trickleFrontSoftness = 0.02F;

        const auto dryState = nodeState(0.0F);
        const auto partialState = nodeState(0.50F);
        const auto fullState = nodeState(1.0F);
        auto dry = BuildGrid(
            {MakeSeepageNode()},
            "ROCK",
            false,
            {},
            1'000'000ULL,
            look,
            {},
            std::nullopt,
            dryState);
        auto partial = BuildGrid(
            {MakeSeepageNode()},
            "ROCK",
            false,
            {},
            1'000'000ULL,
            look,
            {},
            std::nullopt,
            partialState);
        auto full = BuildGrid(
            {MakeSeepageNode()},
            "ROCK",
            false,
            {},
            1'000'000ULL,
            look,
            {},
            std::nullopt,
            fullState);
        PrepareWaterSeepagePulseFields(&dry, 1.25F);
        PrepareWaterSeepagePulseFields(&partial, 1.25F);
        PrepareWaterSeepagePulseFields(&full, 1.25F);

        constexpr invisible_places::io::Float3 normal{
            0.0F,
            1.0F,
            0.0F,
        };
        const invisible_places::io::Float3 nearPoint{
            0.0F,
            0.0F,
            -0.08F,
        };
        const invisible_places::io::Float3 farPoint{
            0.0F,
            0.0F,
            -0.95F,
        };
        const auto dryNear = EvaluateWaterSeepageGridContribution(
            dry,
            nearPoint,
            normal,
            1.25F);
        const auto partialNear = EvaluateWaterSeepageGridContribution(
            partial,
            nearPoint,
            normal,
            1.25F);
        const auto partialFar = EvaluateWaterSeepageGridContribution(
            partial,
            farPoint,
            normal,
            1.25F);
        const auto fullFar = EvaluateWaterSeepageGridContribution(
            full,
            farPoint,
            normal,
            1.25F);
        CHECK(dryNear.mask == Catch::Approx(0.0F));
        CHECK(dryNear.scale == Catch::Approx(0.0F));
        CHECK(partialNear.mask > 0.0F);
        CHECK(partialNear.scale > 0.0F);
        CHECK(partialFar.mask == Catch::Approx(0.0F).margin(1.0e-6F));
        CHECK(partialFar.scale == Catch::Approx(0.0F).margin(1.0e-6F));
        CHECK(fullFar.mask > 0.0F);

        float fullFarMaximumScale = 0.0F;
        for (std::int32_t lateralIndex = -20;
             lateralIndex <= 20;
             ++lateralIndex) {
            const invisible_places::io::Float3 point{
                static_cast<float>(lateralIndex) * 0.005F,
                0.0F,
                farPoint.z,
            };
            fullFarMaximumScale = std::max(
                fullFarMaximumScale,
                EvaluateWaterSeepageGridContribution(
                    full,
                    point,
                    normal,
                    1.25F)
                    .scale);
        }
        CHECK(fullFarMaximumScale > 0.0F);
    }
}

TEST_CASE("Wetting Trickle reveals short deterministic fingers behind a keyed front", "[water][seepage][patterns][trickle]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::WaterSeepageNodeAnimationStateEntry;

    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::WettingTrickle;
    look.quality = WaterSeepageQuality::High;
    look.baseWetness = 0.75F;
    look.density = 0.80F;
    look.glisten = 0.0F;
    look.tricklePatchSizeMeters = 0.08F;
    look.trickleWidthMeters = 0.018F;
    look.trickleFrontSoftness = 0.025F;
    look.evolution = 0.04F;

    const std::vector<WaterSeepageNodeAnimationStateEntry> dryState{{
        .nodeId = 1U,
        .state = {.activity = 1.0F, .localSpread = 0.0F, .wettingProgress = 0.0F},
    }};
    const std::vector<WaterSeepageNodeAnimationStateEntry> halfState{{
        .nodeId = 1U,
        .state = {.activity = 1.0F, .localSpread = 0.0F, .wettingProgress = 0.50F},
    }};
    const std::vector<WaterSeepageNodeAnimationStateEntry> fullState{{
        .nodeId = 1U,
        .state = {.activity = 1.0F, .localSpread = 0.0F, .wettingProgress = 1.0F},
    }};
    const auto dryGrid = BuildGrid(
        {MakeSeepageNode()}, "ROCK", false, {}, 1'000'000ULL, look, {}, std::nullopt, dryState);
    const auto halfGrid = BuildGrid(
        {MakeSeepageNode()}, "ROCK", false, {}, 1'000'000ULL, look, {}, std::nullopt, halfState);
    const auto fullGrid = BuildGrid(
        {MakeSeepageNode()}, "ROCK", false, {}, 1'000'000ULL, look, {}, std::nullopt, fullState);

    // The wetting front travels the strength- and slope-shaped envelope run.
    // MakeSeepageNode is a vertical wall (steep = 1), so the run is
    // reach 1.25 x strength 1 x 1.15 = ~1.44 m; a half-keyed front sits near
    // 0.72 m. The far point lies beyond the half front but inside the run.
    const invisible_places::io::Float3 nearPoint{0.0F, 0.0F, -0.08F};
    const invisible_places::io::Float3 farPoint{0.0F, 0.0F, -1.10F};
    const invisible_places::io::Float3 normal{0.0F, 1.0F, 0.0F};
    CHECK(EvaluateWaterSeepageGridContribution(dryGrid, nearPoint, normal, 2.0F).scale ==
          Catch::Approx(0.0F));
    CHECK(EvaluateWaterSeepageGridContribution(halfGrid, nearPoint, normal, 2.0F).scale > 0.0F);
    CHECK(EvaluateWaterSeepageGridContribution(halfGrid, farPoint, normal, 2.0F).scale ==
          Catch::Approx(0.0F).margin(1.0e-6F));
    const auto full = EvaluateWaterSeepageGridContribution(fullGrid, farPoint, normal, 2.0F);
    const auto repeated = EvaluateWaterSeepageGridContribution(fullGrid, farPoint, normal, 2.0F);
    CHECK(full.scale > 0.0F);
    CHECK(full.scale == Catch::Approx(repeated.scale).margin(1.0e-7F));
    CHECK(full.ripple == Catch::Approx(repeated.ripple).margin(1.0e-7F));
}

TEST_CASE("Node strength shapes the seepage area while prominence only scales intensity", "[water][seepage][envelope]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;

    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::WetRockSheen;
    look.baseWetness = 0.75F;
    look.density = 0.90F;

    // MakeSeepageNode is a vertical wall (steep = 1): the envelope run is
    // reach x strength x 1.15.
    const auto contributionAt = [&](float strength,
                                    float prominence,
                                    const invisible_places::io::Float3& point) {
        auto node = MakeSeepageNode();
        node.strength = strength;
        node.prominence = prominence;
        const auto grid = BuildGrid({node}, "ROCK", false, {}, 1'000'000ULL, look);
        return EvaluateWaterSeepageGridContribution(
            grid, point, {0.0F, 1.0F, 0.0F}, 1.0F);
    };

    SECTION("strength lengthens the run") {
        // Run at strength 0.5 ends near 0.72 m; at 1.5 it passes 2 m.
        const invisible_places::io::Float3 probe{0.0F, 0.0F, -0.95F};
        CHECK(contributionAt(0.5F, 1.0F, probe).mask == Catch::Approx(0.0F));
        CHECK(contributionAt(1.5F, 1.0F, probe).mask > 0.0F);
    }

    SECTION("prominence never changes the area, only the applied intensity") {
        const invisible_places::io::Float3 inside{0.0F, 0.0F, -0.40F};
        const invisible_places::io::Float3 outside{0.0F, 0.0F, -1.90F};
        const auto low = contributionAt(1.0F, 0.5F, inside);
        const auto high = contributionAt(1.0F, 2.0F, inside);
        CHECK(low.mask == Catch::Approx(high.mask).margin(1.0e-6F));
        CHECK(high.scale > low.scale);
        CHECK(contributionAt(1.0F, 0.5F, outside).mask ==
              contributionAt(1.0F, 2.0F, outside).mask);
    }

    SECTION("the half-width spreads outward with travelled distance") {
        auto node = MakeSeepageNode();
        node.widthMeters = 0.10F;
        node.edgeFeatherMeters = 0.02F;
        const auto grid = BuildGrid({node}, "ROCK", false, {}, 1'000'000ULL, look);
        // Lateral axis of the wall node is world X. Near the node the band is
        // ~0.05 m half-width; 1.2 m down it has spread past 0.12 m.
        const auto near = EvaluateWaterSeepageGridContribution(
            grid, {0.12F, 0.0F, -0.10F}, {0.0F, 1.0F, 0.0F}, 1.0F);
        const auto far = EvaluateWaterSeepageGridContribution(
            grid, {0.12F, 0.0F, -1.20F}, {0.0F, 1.0F, 0.0F}, 1.0F);
        CHECK(near.mask == Catch::Approx(0.0F));
        CHECK(far.mask > 0.0F);
    }
}

TEST_CASE("Seepage runs travel further on steep guide surfaces than flat ones", "[water][seepage][envelope]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::WaterSeepageSurfaceGuide;

    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::WetRockSheen;
    look.baseWetness = 0.75F;
    look.density = 0.90F;

    const auto makeGuide = [](const invisible_places::io::Float3& normal) {
        WaterSeepageSurfaceGuide guide;
        guide.nodeId = 1U;
        guide.sampleCount = 4U;
        guide.samples[0] = {{0.0F, 0.0F, 0.0F}, normal, 0.0F, 1.0F};
        guide.samples[1] = {{0.0F, 0.0F, -0.40F}, normal, 0.40F, 1.0F};
        guide.samples[2] = {{0.0F, 0.0F, -0.80F}, normal, 0.80F, 1.0F};
        guide.samples[3] = {{0.0F, 0.0F, -1.20F}, normal, 1.20F, 1.0F};
        guide.requestedReachMeters = 1.25F;
        guide.achievedReachMeters = 1.20F;
        guide.valid = true;
        return guide;
    };

    const auto maskAt = [&](const invisible_places::io::Float3& surfaceNormal) {
        const std::vector<WaterSeepageSurfaceGuide> guides{makeGuide(surfaceNormal)};
        const auto grid = BuildGrid(
            {MakeSeepageNode()}, "ROCK", false, {}, 1'000'000ULL, look, guides);
        // Station 0.9: inside the steep run (~1.2 m, guide-bounded) but past
        // the flat run (1.25 x 0.45 = ~0.56 m).
        return EvaluateWaterSeepageGridContribution(
                   grid, {0.0F, 0.0F, -0.90F}, surfaceNormal, 1.0F)
            .mask;
    };

    CHECK(maskAt({0.0F, 1.0F, 0.0F}) > 0.0F);
    CHECK(maskAt({0.0F, 0.0F, 1.0F}) == Catch::Approx(0.0F).margin(1.0e-6F));
}

TEST_CASE("Per-node Seepage keys compose activity spread and wetting without topology changes", "[water][seepage][scenario][node-animation]") {
    using Catch::Approx;
    using invisible_places::water::AddOrUpdateWaterSeepageNodeKey;
    using invisible_places::water::ApplyWaterSeepageRuntimeParameters;
    using invisible_places::water::ApplyWaterSeepageScenarioParameters;
    using invisible_places::water::EvaluateWaterSeepageNodeAnimationTrack;
    using invisible_places::water::EvaluateWaterSeepageNodeAnimationTracks;
    using invisible_places::water::WaterScenarioInterpolation;
    using invisible_places::water::WaterScenarioTrack;
    using invisible_places::water::WaterSeepageNodeKey;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;

    WaterScenarioTrack track;
    AddOrUpdateWaterSeepageNodeKey(
        &track,
        1U,
        WaterSeepageNodeKey{
            .id = "dry",
            .position = 0.0F,
            .state = {.activity = 0.0F, .localSpread = 0.0F, .wettingProgress = 0.0F},
            .interpolation = WaterScenarioInterpolation::Linear,
        });
    AddOrUpdateWaterSeepageNodeKey(
        &track,
        1U,
        WaterSeepageNodeKey{
            .id = "wet",
            .position = 1.0F,
            .state = {.activity = 1.0F, .localSpread = 1.0F, .wettingProgress = 1.0F},
        });
    const auto middle = EvaluateWaterSeepageNodeAnimationTrack(track, 1U, 0.5F);
    CHECK(middle.activity == Approx(0.5F));
    CHECK(middle.localSpread == Approx(0.5F));
    CHECK(middle.wettingProgress == Approx(0.5F));
    CHECK(EvaluateWaterSeepageNodeAnimationTrack(track, 999U, 0.5F).activity == Approx(1.0F));

    auto replacement = track.seepageNodeTracks.front().keys.front();
    replacement.id.clear();
    replacement.position = 0.00005F;
    replacement.state.activity = 0.2F;
    AddOrUpdateWaterSeepageNodeKey(&track, 1U, replacement);
    REQUIRE(track.seepageNodeTracks.front().keys.size() == 2U);
    CHECK(track.seepageNodeTracks.front().keys.front().id == "dry");

    const auto nodeStates = EvaluateWaterSeepageNodeAnimationTracks(track, 0.5F);
    REQUIRE(nodeStates.size() == 1U);
    auto scenario = invisible_places::water::DefaultWaterScenarioDefinitions().front().state;
    scenario.seepageLevel = 0.80F;
    scenario.seepageSpread = 0.25F;
    auto grid = BuildGrid(
        {MakeSeepageNode()}, "ROCK", false, {}, 1'000'000ULL, {}, {}, scenario, nodeStates);
    REQUIRE(grid.nodes.size() == 1U);
    CHECK(grid.nodes.front().strength == Approx(0.48F).margin(5.0e-5F));
    CHECK(grid.nodes.front().scenarioSpread == Approx(0.625F).margin(5.0e-5F));
    CHECK(grid.nodes.front().wettingProgress == Approx(0.5F).margin(5.0e-5F));

    const auto topology = WaterSeepageTopologyFingerprint(grid);
    const auto params = WaterSeepageParamsFingerprint(grid);
    const std::vector<invisible_places::water::WaterSeepageNodeAnimationStateEntry> changed{{
        .nodeId = 1U,
        .state = {.activity = 0.25F, .localSpread = 0.20F, .wettingProgress = 0.90F},
    }};
    ApplyWaterSeepageScenarioParameters(
        &grid,
        scenario,
        {},
        1'000'000ULL,
        changed);
    CHECK(WaterSeepageTopologyFingerprint(grid) == topology);
    CHECK(WaterSeepageParamsFingerprint(grid) != params);
    CHECK(grid.nodes.front().strength == Approx(0.20F));
    CHECK(grid.nodes.front().scenarioSpread == Approx(0.40F));
    CHECK(grid.nodes.front().wettingProgress == Approx(0.90F));

    auto editedNode = MakeSeepageNode();
    editedNode.seed = 991U;
    editedNode.strength = 1.75F;
    editedNode.normalAlignment = 0.82F;
    editedNode.lookProfileName = "Trickle";
    const std::vector<WaterSeepageLookProfile> trickleProfiles{{
        .name = "Trickle",
        .settings = [] {
            WaterSeepageLookSettings settings;
            settings.pattern = WaterSeepagePattern::WettingTrickle;
            settings.tricklePatchSizeMeters = 0.12F;
            return settings;
        }(),
    }};
    ApplyWaterSeepageRuntimeParameters(
        &grid,
        std::span<const WaterSeepageNode>{&editedNode, 1U},
        trickleProfiles,
        {},
        scenario,
        {},
        1'000'000ULL,
        changed);
    CHECK(WaterSeepageTopologyFingerprint(grid) == topology);
    CHECK(grid.nodes.front().seed == 991U);
    CHECK(grid.nodes.front().normalAlignment == Approx(0.82F));
    CHECK(grid.nodes.front().authoredStrength == Approx(1.75F));
    CHECK(grid.nodes.front().authoredLook.pattern == WaterSeepagePattern::WettingTrickle);
}

TEST_CASE("Seepage Rain envelopes are immediate by default and deterministic when delayed", "[water][seepage][rain][animation]") {
    using Catch::Approx;
    using invisible_places::water::AddOrUpdateWaterScenarioKey;
    using invisible_places::water::BuildWaterSeepageRainEnvelope;
    using invisible_places::water::EvaluateWaterScenarioTrack;
    using invisible_places::water::EvaluateWaterSeepageRainEnvelope;
    using invisible_places::water::WaterScenarioInterpolation;
    using invisible_places::water::WaterScenarioKey;
    using invisible_places::water::WaterScenarioTrack;

    auto definition = invisible_places::water::DefaultWaterScenarioDefinitions().front();
    definition.state.seepageRainDelaySeconds = 0.0F;
    definition.state.seepageRainRiseSeconds = 0.0F;
    definition.state.seepageRainRecessionSeconds = 0.0F;
    WaterScenarioTrack track;
    auto dry = definition.state;
    dry.rainLevel = 0.0F;
    auto wet = definition.state;
    wet.rainLevel = 1.0F;
    AddOrUpdateWaterScenarioKey(
        &track,
        WaterScenarioKey{
            .id = "dry",
            .position = 0.0F,
            .state = dry,
            .interpolation = WaterScenarioInterpolation::Linear,
        });
    AddOrUpdateWaterScenarioKey(
        &track,
        WaterScenarioKey{.id = "wet", .position = 1.0F, .state = wet});

    const auto immediate = BuildWaterSeepageRainEnvelope(track, definition, 10.0F, 10.0F);
    REQUIRE(immediate.samples.size() == 101U);
    CHECK(EvaluateWaterSeepageRainEnvelope(immediate, 2.5F) == Approx(0.25F).margin(1.0e-5F));
    CHECK(EvaluateWaterSeepageRainEnvelope(immediate, 5.0F) == Approx(0.50F).margin(1.0e-5F));
    CHECK(immediate.fingerprint ==
          invisible_places::water::WaterSeepageRainEnvelopeFingerprint(
              track, definition, 10.0F));

    for (auto& key : track.keys) {
        key.state.seepageRainDelaySeconds = 2.0F;
        key.state.seepageRainRiseSeconds = 1.0F;
        key.state.seepageRainRecessionSeconds = 3.0F;
    }
    const auto delayed = BuildWaterSeepageRainEnvelope(track, definition, 10.0F, 20.0F);
    const float delayedMiddle = EvaluateWaterSeepageRainEnvelope(delayed, 5.0F);
    CHECK(delayedMiddle > 0.0F);
    CHECK(delayedMiddle < 0.50F);
    CHECK(delayed.fingerprint != immediate.fingerprint);

    const auto bounded = BuildWaterSeepageRainEnvelope(
        track, definition, 120.0F, 120.0F, 16U);
    REQUIRE(bounded.samples.size() == 16U);
    CHECK(bounded.durationSeconds == Approx(120.0F));
    CHECK(std::isfinite(EvaluateWaterSeepageRainEnvelope(bounded, 999.0F)));

    track.keys.front().state.seepageRainDelaySeconds = -2.0F;
    track.keys.front().state.seepageRainRiseSeconds =
        std::numeric_limits<float>::quiet_NaN();
    const auto sanitized = EvaluateWaterScenarioTrack(track, definition, 0.0F);
    CHECK(sanitized.seepageRainDelaySeconds == Approx(0.0F));
    CHECK(sanitized.seepageRainRiseSeconds == Approx(0.0F));
}

TEST_CASE("Seepage surface-cache guides follow a vertical ROCK sheet", "[water][seepage][surface-cache][guide]") {
    using invisible_places::water::BuildWaterSeepageSurfaceGuides;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> samples;
    for (std::uint32_t station = 0U; station < 32U; ++station) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    0.001F + static_cast<float>(sample) * 0.0002F,
                    0.002F + static_cast<float>(sample) * 0.0010F,
                    -0.001F - static_cast<float>(station) * 0.020F,
                },
                .normal = {1.0F, 0.0F, 0.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.020F);
    auto node = MakeSeepageNode();
    node.position = {0.001F, 0.005F, -0.001F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.reachMeters = 0.30F;
    const auto guides = BuildWaterSeepageSurfaceGuides(
        std::span<const WaterSeepageNode>{&node, 1U},
        cache);
    REQUIRE(guides.size() == 1U);
    REQUIRE(guides.front().valid);
    CHECK(guides.front().sampleCount >= 2U);
    CHECK(guides.front().sampleCount <=
          invisible_places::water::kWaterSeepageMaximumGuideSamples);
    CHECK(guides.front().achievedReachMeters > 0.25F);
    CHECK(guides.front().samples[guides.front().sampleCount - 1U].position.z < -0.20F);
    for (std::uint32_t index = 1U; index < guides.front().sampleCount; ++index) {
        CHECK(guides.front().samples[index].station >
              guides.front().samples[index - 1U].station);
        CHECK(guides.front().samples[index].position.z <=
              guides.front().samples[index - 1U].position.z + 0.008F);
    }
}

TEST_CASE("Connected Seepage support is deterministic bounded and transactional",
          "[water][seepage][surface-cache][connected]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::CommitWaterSeepageSupportSelection;
    using invisible_places::water::WaterSeepageSupportBuildOptions;
    using invisible_places::water::WaterSeepageSupportSelection;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> samples;
    const auto addCell = [&](float x, float y, float z, WaterSurfaceRole role) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    x + static_cast<float>(sample) * 0.0001F,
                    y + static_cast<float>(sample) * 0.0001F,
                    z,
                },
                .normal = {0.0F, 1.0F, 0.0F},
                .role = role,
            });
        }
    };
    for (std::uint32_t station = 0U; station < 7U; ++station) {
        addCell(0.005F, 0.005F, -0.005F - station * 0.020F, WaterSurfaceRole::Rock);
    }
    // Within the authored envelope but separated by empty cache cells: it
    // must not bridge into the connected selection.
    addCell(0.085F, 0.005F, -0.045F, WaterSurfaceRole::Rock);
    addCell(0.005F, 0.005F, 0.025F, WaterSurfaceRole::Rock);
    for (std::uint32_t station = 0U; station < 4U; ++station) {
        addCell(0.005F, 0.005F, -0.005F - station * 0.020F, WaterSurfaceRole::Sand);
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.020F);

    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.selectionReachLimitMeters = 0.13F;
    node.selectionWidthLimitMeters = 0.30F;
    const auto first = BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(first.success);
    REQUIRE_FALSE(first.selection.cells.empty());
    CHECK(first.selection.cellSizeMeters == Catch::Approx(0.010F));
    CHECK(first.selection.sourceRole == WaterSurfaceRole::Rock);
    CHECK(first.selection.cells.size() <=
          invisible_places::water::kWaterSeepageMaximumSupportCellsPerNode);
    CHECK(std::none_of(
        first.selection.cells.begin(),
        first.selection.cells.end(),
        [](const auto& cell) { return cell.x >= 8; }));

    auto reversed = samples;
    std::reverse(reversed.begin(), reversed.end());
    const auto reversedCache = BuildWaterSurfaceCacheFromSamples(reversed, 0.020F);
    const auto repeated = BuildWaterSeepageSupportSelection(node, "rock", cache);
    REQUIRE(repeated.success);
    CHECK(repeated.selection.fingerprint == first.selection.fingerprint);
    const auto reordered = BuildWaterSeepageSupportSelection(node, "rock", reversedCache);
    REQUIRE(reordered.success);
    REQUIRE(reordered.selection.cells.size() == first.selection.cells.size());
    REQUIRE(repeated.selection.cells.size() == first.selection.cells.size());
    for (std::size_t index = 0U; index < first.selection.cells.size(); ++index) {
        CHECK(repeated.selection.cells[index].x == first.selection.cells[index].x);
        CHECK(repeated.selection.cells[index].y == first.selection.cells[index].y);
        CHECK(repeated.selection.cells[index].z == first.selection.cells[index].z);
        CHECK(reordered.selection.cells[index].x == first.selection.cells[index].x);
        CHECK(reordered.selection.cells[index].y == first.selection.cells[index].y);
        CHECK(reordered.selection.cells[index].z == first.selection.cells[index].z);
    }

    WaterSeepageSupportSelection settled = first.selection;
    const auto settledFingerprint = settled.fingerprint;
    const auto capped = BuildWaterSeepageSupportSelection(
        node,
        "ROCK",
        cache,
        WaterSeepageSupportBuildOptions{
            .maximumSupportCells = 4U,
            .maximumVisitedSurfels = 100U,
        });
    CHECK_FALSE(capped.success);
    CHECK(capped.diagnostics.cellLimitExceeded);
    CHECK_FALSE(CommitWaterSeepageSupportSelection(capped, &settled));
    CHECK(settled.fingerprint == settledFingerprint);

    std::vector<WaterSeepageNode> overlappingNodes;
    std::vector<WaterSeepageSupportSelection> overlappingSelections;
    for (std::uint32_t id = 1U; id <= 10U; ++id) {
        auto overlappingNode = node;
        overlappingNode.id = id;
        overlappingNodes.push_back(overlappingNode);
        auto overlappingSelection = first.selection;
        overlappingSelection.nodeId = id;
        overlappingSelection.fingerprint += "-" + std::to_string(id);
        overlappingSelections.push_back(std::move(overlappingSelection));
    }
    const auto overlappingGrid = BuildGrid(
        overlappingNodes,
        "ROCK",
        false,
        {},
        1'000'000ULL,
        {},
        {},
        std::nullopt,
        {},
        overlappingSelections);
    CHECK(overlappingGrid.diagnostics.supportOverflowCellCount > 0U);
    CHECK(overlappingGrid.diagnostics.droppedSupportReferenceCount > 0U);
    for (const auto& cell : overlappingGrid.supportHashCells) {
        CHECK(cell.referenceCount <= WaterSeepageSpatialGrid::kMaxReferencesPerCell);
    }

    auto sandNode = node;
    CHECK_FALSE(BuildWaterSeepageSupportSelection(sandNode, "SAND", cache).success);
    CHECK_FALSE(
        BuildWaterSeepageSupportSelection(sandNode, "SAND", cache)
            .surfaceUnavailable);
    sandNode.targetSceneRoles.push_back("SAND");
    const auto sand = BuildWaterSeepageSupportSelection(sandNode, "SAND", cache);
    REQUIRE(sand.success);
    CHECK(sand.selection.sourceRole == WaterSurfaceRole::Sand);

    auto drySandNode = sandNode;
    drySandNode.position = {0.125F, 0.005F, -0.005F};
    const auto drySand =
        BuildWaterSeepageSupportSelection(
            drySandNode,
            "SAND",
            cache);
    CHECK_FALSE(drySand.success);
    CHECK(drySand.surfaceUnavailable);
    CHECK_FALSE(drySand.cancelled);
    CHECK_FALSE(drySand.diagnostics.cellLimitExceeded);
}

TEST_CASE("Connected Seepage node batches preserve authored result order",
          "[water][seepage][surface-cache][connected][parallel]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSeepageSupportSelections;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> samples;
    for (std::int32_t z = 0; z > -20; --z) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    0.005F,
                    0.005F + static_cast<float>(sample) * 0.0001F,
                    -0.005F + static_cast<float>(z) * 0.010F,
                },
                .normal = {1.0F, 0.0F, 0.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    std::array<WaterSeepageNode, 4U> nodes;
    for (std::size_t index = 0U; index < nodes.size(); ++index) {
        nodes[index] = MakeSeepageNode();
        nodes[index].id = static_cast<std::uint32_t>(40U + index);
        nodes[index].name = "Parallel " + std::to_string(index);
        nodes[index].position = {
            0.005F,
            0.005F,
            -0.005F - static_cast<float>(index) * 0.010F,
        };
        nodes[index].surfaceNormal = {1.0F, 0.0F, 0.0F};
        nodes[index].downAxis = {0.0F, 0.0F, -1.0F};
        nodes[index].selectionReachLimitMeters = 0.12F;
        nodes[index].selectionWidthLimitMeters = 0.08F;
    }

    const auto parallel = BuildWaterSeepageSupportSelections(
        nodes,
        "ROCK",
        cache,
        {},
        3U);
    REQUIRE(parallel.size() == nodes.size());
    for (std::size_t index = 0U; index < nodes.size(); ++index) {
        const auto serial =
            BuildWaterSeepageSupportSelection(nodes[index], "ROCK", cache);
        REQUIRE(parallel[index].success);
        REQUIRE(serial.success);
        CHECK(parallel[index].selection.nodeId == nodes[index].id);
        CHECK(parallel[index].selection.fingerprint ==
              serial.selection.fingerprint);
        REQUIRE(parallel[index].selection.cells.size() ==
                serial.selection.cells.size());
        for (std::size_t cellIndex = 0U;
             cellIndex < serial.selection.cells.size();
             ++cellIndex) {
            const auto& parallelCell =
                parallel[index].selection.cells[cellIndex];
            const auto& serialCell =
                serial.selection.cells[cellIndex];
            CHECK(parallelCell.x == serialCell.x);
            CHECK(parallelCell.y == serialCell.y);
            CHECK(parallelCell.z == serialCell.z);
            CHECK(parallelCell.downwardDistanceMeters ==
                  Catch::Approx(serialCell.downwardDistanceMeters));
            CHECK(parallelCell.flowRunMeters ==
                  Catch::Approx(serialCell.flowRunMeters));
            CHECK(parallelCell.crossContourMeters ==
                  Catch::Approx(serialCell.crossContourMeters));
            CHECK(parallelCell.upstream == serialCell.upstream);
        }
    }
}

TEST_CASE("VEG Seepage maps connected ROCK metrics onto VEG occupancy cells",
          "[water][seepage][surface-cache][connected][vegetation]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> samples;
    for (std::uint32_t station = 0U; station < 6U; ++station) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {0.005F, 0.005F, -0.005F - station * 0.020F},
                .normal = {0.0F, 1.0F, 0.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    }
    // The first vegetation layer hugs the ROCK substrate; the second remains
    // within the tangential association search but is beyond Surface Depth.
    for (const float vegetationY : {0.065F, 0.105F}) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {0.005F, vegetationY, -0.045F},
                .normal = {0.0F, 1.0F, 0.0F},
                .role = WaterSurfaceRole::Vegetation,
            });
        }
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.020F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.selectionReachLimitMeters = 0.12F;
    node.selectionWidthLimitMeters = 0.20F;
    node.depthToleranceMeters = 0.08F;

    const auto selection = BuildWaterSeepageSupportSelection(node, "VEG", cache);
    REQUIRE(selection.success);
    CHECK(selection.selection.sourceRole == WaterSurfaceRole::Rock);
    REQUIRE_FALSE(selection.selection.cells.empty());
    CHECK(std::all_of(
        selection.selection.cells.begin(),
        selection.selection.cells.end(),
        [](const auto& cell) {
            return cell.y >= 6 && cell.y < 10;
        }));
    CHECK(std::none_of(
        selection.selection.cells.begin(),
        selection.selection.cells.end(),
        [](const auto& cell) {
            return cell.y >= 10;
        }));
    CHECK(std::any_of(
        selection.selection.cells.begin(),
        selection.selection.cells.end(),
        [](const auto& cell) {
            return cell.downwardDistanceMeters > 0.02F;
        }));
}

TEST_CASE("Connected Seepage support follows a turning maximum-downhill centreline",
          "[water][seepage][surface-cache][connected][centreline]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> samples;
    const std::array path{
        invisible_places::io::Float3{0.005F, 0.005F, -0.005F},
        invisible_places::io::Float3{0.005F, 0.005F, -0.015F},
        invisible_places::io::Float3{0.005F, 0.005F, -0.025F},
        invisible_places::io::Float3{0.015F, 0.005F, -0.035F},
        invisible_places::io::Float3{0.025F, 0.005F, -0.045F},
        invisible_places::io::Float3{0.035F, 0.005F, -0.055F},
        invisible_places::io::Float3{0.045F, 0.005F, -0.065F},
    };
    for (const auto& position : path) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    position.x + static_cast<float>(sample) * 0.00001F,
                    position.y,
                    position.z,
                },
                .normal = {0.0F, 1.0F, 0.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = path.front();
    node.reachMeters = 0.08F;
    node.widthMeters = 0.03F;
    node.selectionReachLimitMeters = 0.50F;
    node.selectionWidthLimitMeters = 0.08F;
    node.edgeFeatherMeters = 0.005F;

    const auto support = BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(support.success);
    const auto turnedCell = std::find_if(
        support.selection.cells.begin(),
        support.selection.cells.end(),
        [](const auto& cell) { return cell.x >= 4 && cell.z <= -7; });
    REQUIRE(turnedCell != support.selection.cells.end());
    // The least-resistance flood retains independent flow-run and
    // cross-contour metrics: the turning path advances downhill while its
    // smaller lateral component is charged at the steep contour rate.
    CHECK(turnedCell->downwardDistanceMeters > 0.03F);
    CHECK(turnedCell->flowRunMeters > 0.05F);
    CHECK(turnedCell->crossContourMeters > 0.02F);
    CHECK(turnedCell->crossContourMeters <
          turnedCell->flowRunMeters);
}

TEST_CASE("Seepage on a vertical face runs far down before it creeps sideways",
          "[water][seepage][surface-cache][connected][least-resistance][steep]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    // A vertical rock wall in the Y-Z plane (normals point along +X, so
    // surface steepness is 1). The node sits top-centre; cells straight
    // below must stay descent-priced while equally distant contour cells
    // cost the steep contour rate — tall and narrow, as slow wetness on a
    // cliff face behaves.
    std::vector<WaterSurfaceSample> samples;
    for (std::int32_t y = 0; y <= 60; ++y) {
        for (std::int32_t z = 0; z >= -60; --z) {
            for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
                samples.push_back({
                    .position = {
                        0.005F + static_cast<float>(sample) * 0.00001F,
                        0.005F + 0.010F * static_cast<float>(y),
                        -0.005F + 0.010F * static_cast<float>(z),
                    },
                    .normal = {1.0F, 0.0F, 0.0F},
                    .role = WaterSurfaceRole::Rock,
                });
            }
        }
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.305F, -0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.widthMeters = 0.03F;
    node.selectionReachLimitMeters = 1.0F;
    node.selectionWidthLimitMeters = 0.70F;
    node.edgeFeatherMeters = 0.005F;

    const auto support = BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(support.success);
    const auto findCell = [&](const auto& predicate)
        -> const invisible_places::water::WaterSeepageSupportCell* {
        for (const auto& cell : support.selection.cells) {
            if (predicate(cell)) {
                return &cell;
            }
        }
        return nullptr;
    };
    // 0.30 m straight down: descent-priced cost, well under the travelled
    // distance.
    const auto* below = findCell([](const auto& cell) {
        return cell.z <= -29 && cell.z >= -32 &&
               cell.y == 30;
    });
    REQUIRE(below != nullptr);
    CHECK(below->downwardDistanceMeters < 0.30F);
    // 0.30 m along the contour: priced at the steep contour rate
    // (kWaterSeepageSteepContourCostFactor),
    // several times the descent cost for the same travelled distance.
    const auto* beside = findCell([](const auto& cell) {
        return cell.y >= 59 && cell.y <= 61 && cell.z >= -2;
    });
    REQUIRE(beside != nullptr);
    CHECK(beside->downwardDistanceMeters > 1.0F);
    CHECK(beside->downwardDistanceMeters >
          below->downwardDistanceMeters * 4.0F);
}

TEST_CASE("Seepage flood splits into both downhill routes of a saddle",
          "[water][seepage][surface-cache][connected][least-resistance]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    // A ridge point with two descending gullies (+X and -X) and a level
    // contour shelf (+Y). Slow wetness should run down BOTH gullies and
    // barely creep along the level shelf.
    std::vector<WaterSurfaceSample> samples;
    const auto addColumn = [&](
                               float x,
                               float y,
                               float z,
                               const invisible_places::io::Float3& normal,
                               std::uint32_t sampleCount = 8U) {
        for (std::uint32_t sample = 0U;
             sample < sampleCount;
             ++sample) {
            samples.push_back({
                .position = {x, y, z},
                .normal = normal,
                .role = WaterSurfaceRole::Rock,
            });
        }
    };
    constexpr float kInvSqrtTwo = 0.70710678F;
    addColumn(
        0.005F,
        0.005F,
        -0.005F,
        {0.0F, 0.0F, 1.0F},
        1U);
    for (std::uint32_t station = 1U; station <= 8U; ++station) {
        const float along = static_cast<float>(station) * 0.010F;
        addColumn(
            0.005F + along,
            0.005F,
            -0.005F - along,
            {kInvSqrtTwo, 0.0F, kInvSqrtTwo});
        addColumn(
            0.005F - along,
            0.005F,
            -0.005F - along,
            {-kInvSqrtTwo, 0.0F, kInvSqrtTwo});
        addColumn(
            0.005F,
            0.005F + along,
            -0.005F,
            {0.0F, 0.0F, 1.0F});
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    // Generous cost limit so even the contour-priced shelf end is selected;
    // the live budget does the real gating at render time.
    node.selectionReachLimitMeters = 0.20F;
    node.selectionWidthLimitMeters = 0.30F;
    node.edgeFeatherMeters = 0.005F;

    const auto support = BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(support.success);
    const auto costAt = [&](std::int32_t x, std::int32_t y) {
        float best = std::numeric_limits<float>::max();
        for (const auto& cell : support.selection.cells) {
            if (cell.x == x && cell.y == y) {
                best = std::min(best, cell.downwardDistanceMeters);
            }
        }
        return best;
    };
    // Both gully ends were reached, at similar (cheap, descent-priced) cost.
    const float rightGully = costAt(8, 0);
    const float leftGully = costAt(-8, 0);
    REQUIRE(rightGully < 1.0F);
    REQUIRE(leftGully < 1.0F);
    CHECK(rightGully == Catch::Approx(leftGully).margin(0.01F));
    // The level shelf at the same travelled distance costs contour rates —
    // markedly more than either descent.
    const float shelf = costAt(0, 8);
    REQUIRE(shelf < 1.0F);
    CHECK(shelf > rightGully * 1.5F);
}

TEST_CASE("Connected Seepage live masks grow monotonically through the authored strength sweep",
          "[water][seepage][surface-cache][connected][mask][strength-sweep]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::EvaluateWaterSeepageSupportCellMask;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    // A 1.8 m vertical sheet covers the eight strengths captured from the
    // reference node. Its modest cross extent makes a sideways leak obvious
    // while keeping this deterministic CPU fixture quick.
    std::vector<WaterSurfaceSample> samples;
    for (std::int32_t y = -15; y <= 15; ++y) {
        for (std::int32_t z = 0; z >= -180; --z) {
            for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
                samples.push_back({
                    .position = {
                        0.005F +
                            static_cast<float>(sample) * 0.00001F,
                        0.005F + static_cast<float>(y) * 0.010F,
                        -0.005F + static_cast<float>(z) * 0.010F,
                    },
                    .normal = {1.0F, 0.0F, 0.0F},
                    .role = WaterSurfaceRole::Rock,
                });
            }
        }
    }
    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.widthMeters = 0.06F;
    node.selectionReachLimitMeters = 1.80F;
    node.selectionWidthLimitMeters = 0.30F;
    node.edgeFeatherMeters = 0.14F;
    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    REQUIRE_FALSE(built.selection.cells.empty());
    const std::array strengths{
        0.02F,
        0.12F,
        0.20F,
        0.32F,
        0.43F,
        0.63F,
        0.93F,
        1.13F,
    };

    std::size_t previousActiveCount = 0U;
    float previousMaximumRun = 0.0F;
    for (std::size_t strengthIndex = 0U;
         strengthIndex < strengths.size();
         ++strengthIndex) {
        node.strength = strengths[strengthIndex];
        const std::array selections{built.selection};
        const auto grid = BuildGrid(
            {node},
            "ROCK",
            false,
            {},
            1'000'000ULL,
            {},
            {},
            std::nullopt,
            {},
            selections);
        REQUIRE(grid.nodes.size() == 1U);
        const auto& runtime = grid.nodes.front();
        std::size_t activeCount = 0U;
        float maximumRun = 0.0F;
        float maximumCross = 0.0F;
        float maximumParityDifference = 0.0F;
        for (const auto& cell : built.selection.cells) {
            const float overlayMask =
                EvaluateWaterSeepageSupportCellMask(runtime, cell);
            const invisible_places::io::Float3 point{
                (static_cast<float>(cell.x) + 0.5F) *
                    built.selection.cellSizeMeters,
                (static_cast<float>(cell.y) + 0.5F) *
                    built.selection.cellSizeMeters,
                (static_cast<float>(cell.z) + 0.5F) *
                    built.selection.cellSizeMeters,
            };
            const float rendererMask =
                EvaluateWaterSeepageGridContribution(
                    grid,
                    point,
                    cell.surfaceNormal,
                    0.31F)
                    .mask;
            maximumParityDifference = std::max(
                maximumParityDifference,
                std::abs(overlayMask - rendererMask));
            if (overlayMask <= 1.0e-4F) {
                continue;
            }
            ++activeCount;
            if (!cell.upstream) {
                maximumRun = std::max(
                    maximumRun,
                    cell.flowRunMeters);
                maximumCross = std::max(
                    maximumCross,
                    cell.crossContourMeters);
            }
        }
        INFO("strength index " << strengthIndex);
        CHECK(maximumParityDifference < 2.0e-5F);
        CHECK(activeCount >= previousActiveCount);
        CHECK(maximumRun + 1.0e-5F >= previousMaximumRun);
        if (strengthIndex == 0U) {
            // The 0.14 m artistic feather must not turn the source into a
            // broad disk; source membership is capped at two 10 mm cells.
            CHECK(maximumCross < 0.061F);
        }
        if (strengths[strengthIndex] >= 0.93F) {
            CHECK(maximumRun > maximumCross * 5.0F);
        }
        previousActiveCount = activeCount;
        previousMaximumRun = maximumRun;
    }
    CHECK(previousMaximumRun > 1.45F);
}

TEST_CASE("Connected steep support converts cost budget to physical wetting-front reach",
          "[water][seepage][surface-cache][connected][wetting-progress]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::WaterSeepageNodeAnimationStateEntry;
    using invisible_places::water::WaterSeepageSupportCell;
    using invisible_places::water::WaterSeepageSupportSelection;
    using invisible_places::water::WaterSurfaceRole;

    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.widthMeters = 0.04F;
    node.strength = 0.40F;
    node.edgeFeatherMeters = 0.005F;
    node.selectionReachLimitMeters = 1.0F;
    node.selectionWidthLimitMeters = 0.08F;

    // At this wall cell, 0.59 m of pure descent costs 0.4425 cost-metres.
    // Strength 0.4 supplies a 0.6 cost-metre budget. After reserving the
    // upstream-first phase, its remapped downhill budget is about 0.545 m,
    // whose physical pure-descent reach is about 0.727 m (not 0.545 m).
    WaterSeepageSupportSelection support;
    support.nodeId = node.id;
    support.targetSceneRole = "ROCK";
    support.sourceRole = WaterSurfaceRole::Rock;
    support.cellSizeMeters = 0.010F;
    support.reachLimitMeters = node.selectionReachLimitMeters;
    support.widthLimitMeters = node.selectionWidthLimitMeters;
    support.fingerprint = "steep-physical-reach";
    support.cells.push_back(WaterSeepageSupportCell{
        .x = 0,
        .y = 0,
        .z = -60,
        .downwardDistanceMeters = 0.59F * 0.75F,
        .flowRunMeters = 0.59F,
        .crossContourMeters = 0.0F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
    });
    // Supply the paired upstream topology used by the production mask. Its
    // immutable extent reserves the short high-wick lead before downhill.
    support.cells.push_back(WaterSeepageSupportCell{
        .x = 0,
        .y = 0,
        .z = 75,
        .downwardDistanceMeters = 0.75F,
        .flowRunMeters = 0.75F,
        .crossContourMeters = 0.0F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = true,
    });
    support.bounds.Expand({0.0F, 0.0F, -0.60F});
    support.bounds.Expand({0.01F, 0.01F, 0.76F});
    const std::array supportSelections{support};

    WaterSeepageLookSettings look;
    look.pattern = WaterSeepagePattern::ContourPulses;
    look.baseWetness = 1.0F;
    look.density = 1.0F;
    look.glisten = 0.0F;
    look.trickleFrontSoftness = 0.005F;
    const auto buildAtProgress = [&](float progress) {
        const std::array<WaterSeepageNodeAnimationStateEntry, 1U> state{{
            {
                .nodeId = node.id,
                .state = {
                    .activity = 1.0F,
                    .localSpread = 0.0F,
                    .wettingProgress = progress,
                },
            },
        }};
        return BuildGrid(
            {node},
            "ROCK",
            false,
            {},
            1'000'000ULL,
            look,
            {},
            std::nullopt,
            state,
            supportSelections);
    };
    const auto beforeFront = buildAtProgress(0.78F);
    const auto afterFront = buildAtProgress(0.84F);
    const auto full = buildAtProgress(1.0F);
    const invisible_places::io::Float3 point{
        0.005F,
        0.005F,
        -0.595F,
    };
    const invisible_places::io::Float3 normal{1.0F, 0.0F, 0.0F};
    const auto before = EvaluateWaterSeepageGridContribution(
        beforeFront,
        point,
        normal,
        0.0F);
    const auto after = EvaluateWaterSeepageGridContribution(
        afterFront,
        point,
        normal,
        0.0F);
    const auto fullyRevealed = EvaluateWaterSeepageGridContribution(
        full,
        point,
        normal,
        0.0F);
    CHECK(before.mask == Catch::Approx(0.0F).margin(1.0e-6F));
    CHECK(after.mask > 0.95F);
    CHECK(after.mask == Catch::Approx(fullyRevealed.mask).margin(2.0e-4F));
}

TEST_CASE("Connected Seepage reveals its highest wick back to the node before releasing downhill",
          "[water][seepage][surface-cache][connected][mask][upstream-first]") {
    using invisible_places::water::EvaluateWaterSeepageSupportCellMask;
    using invisible_places::water::WaterSeepageSupportCell;

    auto node = MakeSeepageNode();
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.widthMeters = 0.10F;
    node.selectionReachLimitMeters = 1.80F;
    node.selectionWidthLimitMeters = 0.20F;
    node.edgeFeatherMeters = 0.010F;

    const WaterSeepageSupportCell highestUpstream{
        .downwardDistanceMeters = 1.35F,
        .flowRunMeters = 1.35F,
        .crossContourMeters = 0.0F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = true,
    };
    const WaterSeepageSupportCell middleUpstream{
        .downwardDistanceMeters = 1.10F,
        .flowRunMeters = 1.10F,
        .crossContourMeters = 0.0F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = true,
    };
    const WaterSeepageSupportCell nodewardUpstream{
        .downwardDistanceMeters = 0.010F,
        .flowRunMeters = 0.010F,
        .crossContourMeters = 0.0F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = true,
    };
    const WaterSeepageSupportCell sourceDownstream{
        .downwardDistanceMeters = 0.0075F,
        .flowRunMeters = 0.010F,
        .crossContourMeters = 0.0F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = false,
    };
    const WaterSeepageSupportCell firstRouteDownstream{
        .downwardDistanceMeters = 0.075F,
        .flowRunMeters = 0.10F,
        .crossContourMeters = 0.0F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = false,
    };

    const auto maskAtStrength =
        [&](float strength,
            const WaterSeepageSupportCell& cell) {
            node.strength = strength;
            const auto grid = BuildGrid(
                {node},
                "ROCK",
                false,
                {},
                1'000'000ULL);
            REQUIRE(grid.nodes.size() == 1U);
            auto runtime = grid.nodes.front();
            // The production grid derives this immutable value from the
            // connected selection. Supply the synthetic route's true tip.
            runtime.maximumUpstreamRunMeters =
                highestUpstream.flowRunMeters;
            return EvaluateWaterSeepageSupportCellMask(
                runtime,
                cell);
        };

    // The old node-outward reveal produced the opposite ordering here.
    CHECK(maskAtStrength(0.02F, highestUpstream) > 0.90F);
    CHECK(maskAtStrength(0.02F, middleUpstream) < 1.0e-4F);
    CHECK(maskAtStrength(0.02F, nodewardUpstream) < 1.0e-4F);
    CHECK(maskAtStrength(0.02F, sourceDownstream) < 1.0e-4F);

    CHECK(maskAtStrength(0.11F, highestUpstream) > 0.90F);
    CHECK(maskAtStrength(0.11F, middleUpstream) > 0.90F);
    CHECK(maskAtStrength(0.11F, nodewardUpstream) < 1.0e-4F);
    CHECK(maskAtStrength(0.11F, sourceDownstream) < 1.0e-4F);

    CHECK(maskAtStrength(0.15F, nodewardUpstream) > 0.90F);
    CHECK(maskAtStrength(0.15F, sourceDownstream) > 0.90F);
    CHECK(maskAtStrength(0.15F, firstRouteDownstream) < 1.0e-4F);
    CHECK(maskAtStrength(0.21F, firstRouteDownstream) > 0.90F);
}

TEST_CASE("Connected Seepage recentres a winding upstream route without widening its branches",
          "[water][seepage][surface-cache][connected][mask][upstream-centreline]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::EvaluateWaterSeepageSupportCellMask;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    // Climb a vertical face while the only connected route winds sideways.
    // The route therefore accumulates far more contour travel than the live
    // one-cell tip width, despite still being the correct centre branch.
    std::vector<WaterSurfaceSample> samples;
    const auto addCell = [&](std::int32_t y, std::int32_t z) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    0.005F +
                        static_cast<float>(sample) * 0.00001F,
                    0.005F + static_cast<float>(y) * 0.010F,
                    0.005F + static_cast<float>(z) * 0.010F,
                },
                .normal = {1.0F, 0.0F, 0.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    };
    for (std::int32_t z = 0; z <= 25; ++z) {
        addCell(std::min(z / 2, 10), z);
    }
    // Add a same-station branch at the highest point. Its first cell shares
    // the station centreline, but the distant end must retain branch excess
    // after the station baseline is removed.
    for (std::int32_t y = 11; y <= 18; ++y) {
        addCell(y, 25);
    }

    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, 0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.widthMeters = 0.10F;
    node.strength = 0.02F;
    node.selectionReachLimitMeters = 2.34375F;
    node.selectionWidthLimitMeters = 0.50F;
    node.edgeFeatherMeters = 0.005F;

    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    const std::array selections{built.selection};
    const auto grid = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        {},
        {},
        std::nullopt,
        {},
        selections);
    REQUIRE(grid.nodes.size() == 1U);

    const auto findCell = [&](std::int32_t y, std::int32_t z) {
        return std::find_if(
            built.selection.cells.begin(),
            built.selection.cells.end(),
            [=](const auto& cell) {
                return cell.x == 0 && cell.y == y &&
                       cell.z == z && cell.upstream;
            });
    };
    const auto highSpine = findCell(10, 25);
    const auto highBranch = findCell(18, 25);
    const auto lowerSpine = findCell(5, 10);
    REQUIRE(highSpine != built.selection.cells.end());
    REQUIRE(highBranch != built.selection.cells.end());
    REQUIRE(lowerSpine != built.selection.cells.end());

    CHECK(highSpine->crossContourMeters < 0.001F);
    CHECK(highBranch->crossContourMeters > 0.065F);
    CHECK(EvaluateWaterSeepageSupportCellMask(
              grid.nodes.front(),
              *highSpine) >
          0.90F);
    CHECK(EvaluateWaterSeepageSupportCellMask(
              grid.nodes.front(),
              *highBranch) <
          0.01F);
    CHECK(EvaluateWaterSeepageSupportCellMask(
              grid.nodes.front(),
              *lowerSpine) <
          0.01F);
}

TEST_CASE("Connected Seepage fills the top-down upstream taper laterally through strength one",
          "[water][seepage][surface-cache][connected][mask][upstream-width]") {
    using invisible_places::water::EvaluateWaterSeepageSupportCellMask;
    using invisible_places::water::WaterSeepageSupportCell;

    auto node = MakeSeepageNode();
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.widthMeters = 0.10F;
    node.selectionReachLimitMeters = 1.80F;
    node.selectionWidthLimitMeters = 0.20F;
    node.edgeFeatherMeters = 0.005F;

    const WaterSeepageSupportCell middleOuter{
        .downwardDistanceMeters = 0.50F,
        .flowRunMeters = 0.50F,
        .crossContourMeters = 0.025F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = true,
    };
    const WaterSeepageSupportCell nodeEdge{
        .downwardDistanceMeters = 0.010F,
        .flowRunMeters = 0.010F,
        .crossContourMeters = 0.045F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
        .upstream = true,
    };

    const auto maskAtStrength =
        [&](float strength,
            const WaterSeepageSupportCell& cell) {
            node.strength = strength;
            const auto grid = BuildGrid(
                {node},
                "ROCK",
                false,
                {},
                1'000'000ULL);
            REQUIRE(grid.nodes.size() == 1U);
            auto runtime = grid.nodes.front();
            runtime.maximumUpstreamRunMeters = 1.0F;
            return EvaluateWaterSeepageSupportCellMask(runtime, cell);
        };

    // When the descending front first reaches the node, the source already
    // meets Source Width but the older upper route remains a narrow centre.
    CHECK(maskAtStrength(0.20F, nodeEdge) > 0.90F);
    const float earlyMiddle = maskAtStrength(0.20F, middleOuter);
    const float laterMiddle = maskAtStrength(0.60F, middleOuter);
    const float fullMiddle = maskAtStrength(1.0F, middleOuter);
    CHECK(earlyMiddle < 0.10F);
    CHECK(laterMiddle > earlyMiddle);
    CHECK(fullMiddle > laterMiddle);
    CHECK(fullMiddle > 0.90F);
}

TEST_CASE("Connected Seepage keeps a separately tapered upstream wick",
          "[water][seepage][surface-cache][connected][mask][upstream]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::EvaluateWaterSeepageSupportCellMask;
    using invisible_places::water::UnpackWaterSeepageSupportReferenceMetadata;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;
    using invisible_places::water::kWaterSeepageSupportUpstreamFlag;

    std::vector<WaterSurfaceSample> samples;
    for (std::int32_t y = -3; y <= 3; ++y) {
        for (std::int32_t z = -45; z <= 35; ++z) {
            for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
                samples.push_back({
                    .position = {
                        0.005F +
                            static_cast<float>(sample) * 0.00001F,
                        0.005F + static_cast<float>(y) * 0.010F,
                        -0.005F + static_cast<float>(z) * 0.010F,
                    },
                    .normal = {1.0F, 0.0F, 0.0F},
                    .role = WaterSurfaceRole::Rock,
                });
            }
        }
    }
    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.widthMeters = 0.04F;
    node.strength = 0.20F;
    node.selectionReachLimitMeters = 0.30F;
    node.selectionWidthLimitMeters = 0.08F;
    node.edgeFeatherMeters = 0.005F;
    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    const std::array selections{built.selection};
    const auto grid = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        {},
        {},
        std::nullopt,
        {},
        selections);
    REQUIRE(grid.nodes.size() == 1U);
    float selectedMaximumUpstreamRun = 0.0F;
    for (const auto& cell : built.selection.cells) {
        if (cell.upstream) {
            selectedMaximumUpstreamRun = std::max(
                selectedMaximumUpstreamRun,
                cell.flowRunMeters);
        }
    }
    CHECK(grid.nodes.front().maximumUpstreamRunMeters ==
          Catch::Approx(selectedMaximumUpstreamRun).margin(1.0e-6F));
    CHECK(std::any_of(
        grid.supportReferences.begin(),
        grid.supportReferences.end(),
        [](const auto& reference) {
            return (UnpackWaterSeepageSupportReferenceMetadata(
                        reference.packedNormalRoleConfidenceFlags)
                        .flags &
                    kWaterSeepageSupportUpstreamFlag) != 0U;
        }));
    float maximumDownstreamRun = 0.0F;
    float maximumUpstreamRun = 0.0F;
    float farUpstreamCross = 0.0F;
    for (const auto& cell : built.selection.cells) {
        if (EvaluateWaterSeepageSupportCellMask(
                grid.nodes.front(),
                cell) <= 1.0e-4F) {
            continue;
        }
        if (cell.upstream) {
            maximumUpstreamRun = std::max(
                maximumUpstreamRun,
                cell.flowRunMeters);
            if (cell.flowRunMeters >
                node.selectionReachLimitMeters * 0.75F) {
                farUpstreamCross = std::max(
                    farUpstreamCross,
                    cell.crossContourMeters);
            }
        } else {
            maximumDownstreamRun = std::max(
                maximumDownstreamRun,
                cell.flowRunMeters);
        }
    }
    REQUIRE(maximumDownstreamRun > 0.35F);
    REQUIRE(maximumUpstreamRun > 0.26F);
    CHECK(maximumUpstreamRun / maximumDownstreamRun ==
          Catch::Approx(0.75F).margin(0.08F));
    CHECK(farUpstreamCross < 0.016F);
}

TEST_CASE("Connected Seepage upstream routing cannot enter an unpriced downhill lane",
          "[water][seepage][surface-cache][connected][upstream][bounded]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    // On a level sheet projected gravity is degenerate, so the authored
    // down-axis supplies the transported tangent. Before downhill rejection
    // was applied at the source, the upstream label could walk every +X cell
    // for zero cost (`up == cross == 0`) regardless of Selection Reach.
    std::vector<WaterSurfaceSample> samples;
    for (std::int32_t x = -20; x <= 300; ++x) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    0.005F + static_cast<float>(x) * 0.010F,
                    0.005F + static_cast<float>(sample) * 0.00001F,
                    0.005F,
                },
                .normal = {0.0F, 0.0F, 1.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, 0.005F};
    node.surfaceNormal = {0.0F, 0.0F, 1.0F};
    node.downAxis = {1.0F, 0.0F, 0.0F};
    node.selectionReachLimitMeters = 0.10F;
    node.selectionWidthLimitMeters = 0.06F;
    node.depthToleranceMeters = 0.02F;

    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    CHECK_FALSE(built.diagnostics.cellLimitExceeded);
    CHECK(built.diagnostics.visitedSurfelCount < 100U);
    CHECK(std::none_of(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [](const auto& cell) {
            return cell.upstream && cell.x > 1;
        }));
}

TEST_CASE("Connected Seepage bounds near-normal substrate tunnelling",
          "[water][seepage][surface-cache][connected][bounded][normal-step]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    // Model a thick/noisy ROCK layer whose adjacent cells advance almost
    // entirely through the surface normal. The tiny downward tangent used to
    // cost almost nothing and also remained inside the flow/cross source
    // exception, so a full-site cache could be searched far beyond the
    // authored reach even though this is not travel along the rock surface.
    std::vector<WaterSurfaceSample> samples;
    for (std::int32_t x = 0; x <= 500; ++x) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    0.005F + static_cast<float>(x) * 0.010F,
                    0.005F + static_cast<float>(sample) * 0.00001F,
                    0.005F - static_cast<float>(x) * 0.0001F,
                },
                .normal = {1.0F, 0.0F, 0.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, 0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.selectionReachLimitMeters = 0.10F;
    node.selectionWidthLimitMeters = 0.06F;
    node.depthToleranceMeters = 0.02F;

    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    CHECK_FALSE(built.diagnostics.cellLimitExceeded);
    CHECK(built.diagnostics.rejectedReachCount > 0U);
    CHECK(built.diagnostics.visitedSurfelCount < 100U);
    CHECK(std::none_of(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [](const auto& cell) {
            return cell.x > 20;
        }));
}

TEST_CASE("Connected Seepage decomposes diagonal slope travel into run and true contour width",
          "[water][seepage][surface-cache][connected][diagonal]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    constexpr float kInvSqrtTwo = 0.70710678F;
    std::vector<WaterSurfaceSample> samples;
    for (std::int32_t cross = -10; cross <= 10; ++cross) {
        for (std::int32_t run = 0; run <= 16; ++run) {
            for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
                samples.push_back({
                    .position = {
                        0.005F + static_cast<float>(run) * 0.010F,
                        0.005F + static_cast<float>(cross) * 0.010F,
                        -0.005F - static_cast<float>(run) * 0.010F,
                    },
                    .normal = {
                        kInvSqrtTwo,
                        0.0F,
                        kInvSqrtTwo,
                    },
                    .role = WaterSurfaceRole::Rock,
                });
            }
        }
    }
    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.surfaceNormal = {
        kInvSqrtTwo,
        0.0F,
        kInvSqrtTwo,
    };
    node.downAxis = {
        kInvSqrtTwo,
        0.0F,
        -kInvSqrtTwo,
    };
    node.selectionReachLimitMeters = 0.30F;
    node.selectionWidthLimitMeters = 0.12F;
    node.edgeFeatherMeters = 0.005F;
    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);

    const auto diagonal = std::max_element(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [](const auto& left, const auto& right) {
            const float leftScore =
                left.upstream || left.crossContourMeters > 0.002F
                    ? -1.0F
                    : left.flowRunMeters;
            const float rightScore =
                right.upstream || right.crossContourMeters > 0.002F
                    ? -1.0F
                    : right.flowRunMeters;
            return leftScore < rightScore;
        });
    REQUIRE(diagonal != built.selection.cells.end());
    CHECK_FALSE(diagonal->upstream);
    CHECK(diagonal->flowRunMeters > 0.18F);
    CHECK(diagonal->crossContourMeters < 0.002F);
    CHECK(diagonal->downwardDistanceMeters <
          diagonal->flowRunMeters * 1.10F);
    CHECK(std::none_of(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [&](const auto& cell) {
            return cell.crossContourMeters >
                   node.selectionWidthLimitMeters * 0.5F + 0.021F;
        }));
}

TEST_CASE("Connected Seepage follows a ledge tangent and widens only where the surface flattens",
          "[water][seepage][surface-cache][connected][ledge]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::EvaluateWaterSeepageSupportCellMask;
    using invisible_places::water::WaterSeepageSupportCell;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> samples;
    const auto addCell = [&](float x,
                             float y,
                             float z,
                             const invisible_places::io::Float3& normal) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {
                    x + static_cast<float>(sample) * 0.00001F,
                    y,
                    z,
                },
                .normal = normal,
                .role = WaterSurfaceRole::Rock,
            });
        }
    };
    for (std::int32_t cross = -8; cross <= 8; ++cross) {
        const float y =
            0.005F + static_cast<float>(cross) * 0.010F;
        for (std::int32_t down = 0; down <= 9; ++down) {
            addCell(
                0.005F,
                y,
                -0.005F - static_cast<float>(down) * 0.010F,
                {1.0F, 0.0F, 0.0F});
        }
        for (std::int32_t out = 1; out <= 10; ++out) {
            addCell(
                0.005F + static_cast<float>(out) * 0.010F,
                y,
                -0.105F,
                {0.0F, 0.0F, 1.0F});
        }
    }
    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.widthMeters = 0.04F;
    node.selectionReachLimitMeters = 0.45F;
    node.selectionWidthLimitMeters = 0.16F;
    node.edgeFeatherMeters = 0.005F;
    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    const auto ledgeEnd = std::find_if(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [](const auto& cell) {
            return !cell.upstream && cell.x >= 9 &&
                   cell.z <= -10 &&
                   cell.crossContourMeters < 0.012F;
        });
    REQUIRE(ledgeEnd != built.selection.cells.end());
    CHECK(ledgeEnd->flowRunMeters > 0.16F);
    CHECK(ledgeEnd->surfaceNormal.z > 0.9F);

    const std::array selections{built.selection};
    const auto runtime = BuildGrid(
        {node},
        "ROCK",
        false,
        {},
        1'000'000ULL,
        {},
        {},
        std::nullopt,
        {},
        selections)
                             .nodes.front();
    const WaterSeepageSupportCell wallCell{
        .downwardDistanceMeters = 0.20F,
        .flowRunMeters = 0.20F,
        .crossContourMeters = 0.075F,
        .surfaceNormal = {1.0F, 0.0F, 0.0F},
        .confidence = 1.0F,
    };
    auto flatCell = wallCell;
    flatCell.surfaceNormal = {0.0F, 0.0F, 1.0F};
    CHECK(EvaluateWaterSeepageSupportCellMask(
              runtime,
              flatCell) >
          EvaluateWaterSeepageSupportCellMask(
              runtime,
              wallCell) +
              0.20F);
}

TEST_CASE("Connected Seepage charges confidence-weighted tangents and rejects normal-layer jumps",
          "[water][seepage][surface-cache][connected][normal-frame]") {
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> samples;
    for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
        samples.push_back({
            .position = {
                0.005F + static_cast<float>(sample) * 0.00001F,
                0.005F,
                -0.005F,
            },
            .normal = {1.0F, 0.0F, 0.0F},
            .role = WaterSurfaceRole::Rock,
        });
    }
    // Low-confidence normal noise must not rotate the local tangent enough to
    // reject this genuinely downhill neighbour.
    samples.push_back({
        .position = {0.005F, 0.005F, -0.015F},
        .normal = {0.21F, 0.0F, 0.9777F},
        .role = WaterSurfaceRole::Rock,
    });
    // An adjacent parallel layer is one cache cell away but lies entirely
    // normal to the substrate, outside Surface Depth.
    for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
        samples.push_back({
            .position = {
                0.015F,
                0.005F + static_cast<float>(sample) * 0.00001F,
                -0.015F,
            },
            .normal = {1.0F, 0.0F, 0.0F},
            .role = WaterSurfaceRole::Rock,
        });
    }
    const auto cache =
        BuildWaterSurfaceCacheFromSamples(samples, 0.010F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.surfaceNormal = {1.0F, 0.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.depthToleranceMeters = 0.005F;
    node.selectionReachLimitMeters = 0.10F;
    node.selectionWidthLimitMeters = 0.10F;
    node.edgeFeatherMeters = 0.005F;
    const auto built =
        BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    CHECK(std::any_of(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [](const auto& cell) {
            return cell.z <= -2 &&
                   cell.flowRunMeters > 0.005F;
        }));
    CHECK(std::none_of(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [](const auto& cell) {
            return cell.x >= 1;
        }));
}

TEST_CASE("Connected Seepage support keeps live dimensions parameter-only",
          "[water][seepage][surface-cache][connected][parameters]") {
    using invisible_places::water::ApplyWaterSeepageScenarioParameters;
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::PackWaterSeepageSupportReferenceMetadata;
    using invisible_places::water::PackWaterSeepageSupportRunCrossMetrics;
    using invisible_places::water::UnpackWaterSeepageSupportReferenceMetadata;
    using invisible_places::water::UnpackWaterSeepageSupportRunCrossMetrics;
    using invisible_places::water::WaterSeepageNodeAnimationStateEntry;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;
    using invisible_places::water::kWaterSeepageSupportConnectedFlag;
    using invisible_places::water::kWaterSeepageSupportUpstreamFlag;

    const auto packed = PackWaterSeepageSupportReferenceMetadata(
        {0.0F, 1.0F, 0.0F},
        WaterSurfaceRole::Rock,
        0.73F,
        kWaterSeepageSupportConnectedFlag |
            kWaterSeepageSupportUpstreamFlag);
    const auto unpacked = UnpackWaterSeepageSupportReferenceMetadata(packed);
    CHECK(unpacked.sourceRole == WaterSurfaceRole::Rock);
    CHECK(unpacked.confidence == Catch::Approx(0.73F).margin(0.005F));
    CHECK(unpacked.surfaceNormal.y == Catch::Approx(1.0F).margin(0.005F));
    CHECK(unpacked.flags ==
          (kWaterSeepageSupportConnectedFlag |
           kWaterSeepageSupportUpstreamFlag));
    const auto packedRunCross =
        PackWaterSeepageSupportRunCrossMetrics(1.234F, 0.067F);
    const auto unpackedRunCross =
        UnpackWaterSeepageSupportRunCrossMetrics(packedRunCross);
    CHECK(unpackedRunCross.flowRunMeters ==
          Catch::Approx(1.234F).margin(0.001F));
    CHECK(unpackedRunCross.crossContourMeters ==
          Catch::Approx(0.067F).margin(0.0001F));
    const auto clampedRunCross =
        UnpackWaterSeepageSupportRunCrossMetrics(
            PackWaterSeepageSupportRunCrossMetrics(
                -1.0F,
                std::numeric_limits<float>::quiet_NaN()));
    CHECK(clampedRunCross.flowRunMeters == 0.0F);
    CHECK(clampedRunCross.crossContourMeters == 0.0F);
    STATIC_REQUIRE(sizeof(invisible_places::water::WaterSeepageSupportReference) == 16U);

    std::vector<WaterSurfaceSample> samples;
    for (std::uint32_t station = 0U; station < 7U; ++station) {
        for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
            samples.push_back({
                .position = {0.005F, 0.005F, -0.005F - station * 0.020F},
                .normal = {0.0F, 1.0F, 0.0F},
                .role = WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.020F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.reachMeters = 0.11F;
    node.widthMeters = 0.12F;
    node.selectionReachLimitMeters = 0.13F;
    node.selectionWidthLimitMeters = 0.20F;
    const auto built = BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(built.success);
    const std::array selections{built.selection};
    auto grid = BuildGrid(
        {node}, "ROCK", false, {}, 1'000'000ULL, {}, {}, std::nullopt, {}, selections);
    REQUIRE(grid.nodes.size() == 1U);
    REQUIRE_FALSE(grid.supportHashCells.empty());
    REQUIRE_FALSE(grid.supportReferences.empty());
    CHECK(grid.nodes.front().usesConnectedSupport);
    CHECK(grid.diagnostics.supportReferenceCount == grid.supportReferences.size());
    for (const auto& cell : grid.supportHashCells) {
        CHECK(cell.referenceCount <= WaterSeepageSpatialGrid::kMaxReferencesPerCell);
    }

    const auto selectedCell = std::find_if(
        built.selection.cells.begin(),
        built.selection.cells.end(),
        [](const auto& cell) {
            return cell.downwardDistanceMeters > 0.03F &&
                   cell.downwardDistanceMeters < 0.08F;
        });
    REQUIRE(selectedCell != built.selection.cells.end());
    const invisible_places::io::Float3 selectedPoint{
        (selectedCell->x + 0.5F) * built.selection.cellSizeMeters,
        (selectedCell->y + 0.5F) * built.selection.cellSizeMeters,
        (selectedCell->z + 0.5F) * built.selection.cellSizeMeters,
    };
    CHECK(EvaluateWaterSeepageGridContribution(
              grid, selectedPoint, {0.0F, 1.0F, 0.0F}, 0.31F)
              .scale > 0.0F);
    auto unsupportedPoint = selectedPoint;
    unsupportedPoint.y += 0.04F;
    CHECK(EvaluateWaterSeepageGridContribution(
              grid, unsupportedPoint, {0.0F, 1.0F, 0.0F}, 0.31F)
              .scale == Catch::Approx(0.0F));

    const auto topology = WaterSeepageTopologyFingerprint(grid);
    const auto params = WaterSeepageParamsFingerprint(grid);
    const std::array<WaterSeepageNodeAnimationStateEntry, 1U> animation{{{
        .nodeId = node.id,
        .state = {
            .activity = 1.0F,
            .localSpread = 0.0F,
            .wettingProgress = 1.0F,
            .reachScale = 0.50F,
            .widthScale = 0.50F,
            .prominence = 0.40F,
        },
    }}};
    ApplyWaterSeepageScenarioParameters(
        &grid, std::nullopt, {}, 1'000'000ULL, animation);
    CHECK(WaterSeepageTopologyFingerprint(grid) == topology);
    CHECK(WaterSeepageParamsFingerprint(grid) != params);
    CHECK(grid.nodes.front().reachMeters == Catch::Approx(0.055F));
    CHECK(grid.nodes.front().widthMeters == Catch::Approx(0.060F));
    CHECK(grid.nodes.front().prominence == Catch::Approx(0.40F));
}
