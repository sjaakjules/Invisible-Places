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
    CHECK(node.widthMeters == Approx(0.75F));
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
    // for the default node) and spreads outward with travelled distance. The
    // default node is a vertical wall, so the run is reach 1.25 x 1.15 =
    // ~1.44 m with a scaled end feather.
    const auto grid = BuildGrid({MakeSeepageNode()});
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
             WaterSeepagePattern::WettingTrickle}) {
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
    invalid.rockResponse.radiusMeters = 5.0F;
    invalid.rockResponse.colourise.x = -2.0F;
    invalid.vegetationResponse.twinkle = 9.0F;
    invalid.vegetationResponse.streamDepthMeters = 8.0F;
    const auto sanitized = SanitizeWaterDynamicMeshFlowSettings(invalid);
    CHECK(sanitized.particleCapacity == 4096U);
    CHECK(sanitized.historyLength == 24U);
    CHECK(sanitized.sourceBandWidthMeters == Approx(0.75F));
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
}

TEST_CASE("Mesh Flow dry entries follow the vegetation-supported positive-X edge",
          "[water][mesh-flow][ground][entry]") {
    using Catch::Approx;
    using invisible_places::water::BuildWaterDynamicMeshFlowGroundEntries;
    using invisible_places::water::WaterGroundCell;
    using invisible_places::water::WaterSurfaceCache;
    using invisible_places::water::kWaterGroundVegetationSupportedFlag;

    WaterSurfaceCache cache;
    cache.resolutionMeters = 0.010F;
    const auto cell =
        [](std::int32_t x,
           std::int32_t y,
           std::uint32_t component,
           bool vegetationSupported) {
            WaterGroundCell result;
            result.cellX = x;
            result.cellY = y;
            result.componentId = component;
            result.flags = vegetationSupported
                               ? kWaterGroundVegetationSupportedFlag
                               : 0U;
            result.connectivityMask = 1U;
            return result;
        };
    cache.groundCells = {
        cell(0, 0, 1U, false),
        cell(50, 0, 1U, true),
        cell(60, 0, 1U, true),
        // Bare sampled Ground continues 1.40 m beyond the vegetation. The
        // former component-wide +X anchor incorrectly discarded both valid
        // vegetation-supported entries.
        cell(200, 0, 1U, false),
        cell(220, 0, 2U, false),
        // A vegetation-supported but disconnected column is not a valid
        // automatic route source even when it is the local +X extreme.
        cell(300, 0, 3U, true),
    };
    cache.groundCells.back().connectivityMask = 0U;

    const auto entries = BuildWaterDynamicMeshFlowGroundEntries(cache);
    REQUIRE(entries.size() == 2U);
    CHECK(entries[0].cellX == 60);
    CHECK(entries[0].edgeDistanceMeters == Approx(0.0F));
    CHECK(entries[1].cellX == 50);
    CHECK(entries[1].edgeDistanceMeters == Approx(0.10F));
    CHECK(entries[1].edgeDistanceFraction ==
          Approx(10.0F / 201.0F));
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
    sandNode.targetSceneRoles.push_back("SAND");
    const auto sand = BuildWaterSeepageSupportSelection(sandNode, "SAND", cache);
    REQUIRE(sand.success);
    CHECK(sand.selection.sourceRole == WaterSurfaceRole::Sand);
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
    for (std::uint32_t sample = 0U; sample < 8U; ++sample) {
        samples.push_back({
            .position = {0.005F, 0.105F, -0.045F},
            .normal = {0.0F, 1.0F, 0.0F},
            .role = WaterSurfaceRole::Vegetation,
        });
    }
    const auto cache = BuildWaterSurfaceCacheFromSamples(samples, 0.020F);
    auto node = MakeSeepageNode();
    node.position = {0.005F, 0.005F, -0.005F};
    node.selectionReachLimitMeters = 0.12F;
    node.selectionWidthLimitMeters = 0.20F;

    const auto selection = BuildWaterSeepageSupportSelection(node, "VEG", cache);
    REQUIRE(selection.success);
    CHECK(selection.selection.sourceRole == WaterSurfaceRole::Rock);
    REQUIRE_FALSE(selection.selection.cells.empty());
    CHECK(std::all_of(
        selection.selection.cells.begin(),
        selection.selection.cells.end(),
        [](const auto& cell) { return cell.y >= 10; }));
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
    node.selectionReachLimitMeters = 0.10F;
    node.selectionWidthLimitMeters = 0.03F;
    node.edgeFeatherMeters = 0.005F;

    const auto support = BuildWaterSeepageSupportSelection(node, "ROCK", cache);
    REQUIRE(support.success);
    const auto turnedCell = std::find_if(
        support.selection.cells.begin(),
        support.selection.cells.end(),
        [](const auto& cell) { return cell.x >= 4 && cell.z <= -7; });
    REQUIRE(turnedCell != support.selection.cells.end());
    CHECK(turnedCell->lateralDistanceMeters < 0.015F);
    CHECK(turnedCell->downwardDistanceMeters > 0.05F);
}

TEST_CASE("Connected Seepage support keeps live dimensions parameter-only",
          "[water][seepage][surface-cache][connected][parameters]") {
    using invisible_places::water::ApplyWaterSeepageScenarioParameters;
    using invisible_places::water::BuildWaterSeepageSupportSelection;
    using invisible_places::water::BuildWaterSurfaceCacheFromSamples;
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::PackWaterSeepageSupportReferenceMetadata;
    using invisible_places::water::UnpackWaterSeepageSupportReferenceMetadata;
    using invisible_places::water::WaterSeepageNodeAnimationStateEntry;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    const auto packed = PackWaterSeepageSupportReferenceMetadata(
        {0.0F, 1.0F, 0.0F},
        WaterSurfaceRole::Rock,
        0.73F,
        1U);
    const auto unpacked = UnpackWaterSeepageSupportReferenceMetadata(packed);
    CHECK(unpacked.sourceRole == WaterSurfaceRole::Rock);
    CHECK(unpacked.confidence == Catch::Approx(0.73F).margin(0.005F));
    CHECK(unpacked.surfaceNormal.y == Catch::Approx(1.0F).margin(0.005F));
    CHECK(unpacked.flags == 1U);
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
