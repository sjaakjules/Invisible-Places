#include "water/WaterFlow.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using invisible_places::water::WaterRainIntensityPreset;
using invisible_places::water::WaterRainSettings;
using invisible_places::water::WaterSeepageLookProfile;
using invisible_places::water::WaterSeepageLookSettings;
using invisible_places::water::WaterSeepageNode;
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
    std::span<const WaterSeepageSurfaceGuide> guides = {}) {
    return invisible_places::water::BuildWaterSeepageSpatialGrid(
        nodes,
        std::span<const WaterSeepageLookProfile>{},
        defaultLook,
        role,
        forExport,
        rain,
        effectiveInvocations,
        guides);
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
    CHECK(look.baseWetness == Approx(0.35F));
    CHECK(look.density == Approx(0.45F));
    CHECK(look.glisten == Approx(0.55F));
    CHECK(look.wavelengthMeters == Approx(0.16F));
    CHECK(look.patternScale == Approx(1.0F));
    CHECK(look.speed == Approx(0.18F));
    CHECK(look.warp == Approx(0.40F));
    CHECK(look.turbulence == Approx(0.22F));
    CHECK(look.phase == Approx(0.0F));
    CHECK(look.rainResponse == Approx(0.50F));
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

TEST_CASE("Seepage look resolution honors profile and local edit precedence", "[water][seepage][profiles]") {
    using Catch::Approx;
    using invisible_places::water::ResolveWaterSeepageLook;

    WaterSeepageLookProfile saved;
    saved.name = "Wet Rock";
    saved.settings.baseWetness = 0.62F;
    std::vector<WaterSeepageLookProfile> profiles{saved};

    auto node = MakeSeepageNode();
    node.lookProfileName = saved.name;
    CHECK(ResolveWaterSeepageLook(node, profiles, {}).baseWetness == Approx(0.62F));

    WaterSeepageLookSettings local;
    local.baseWetness = 0.73F;
    node.lookOverride = local;
    CHECK(ResolveWaterSeepageLook(node, profiles, {}).baseWetness == Approx(0.73F));

    WaterSeepageLookSettings edited = local;
    edited.baseWetness = 0.84F;
    node.tempLookOverride = edited;
    CHECK(ResolveWaterSeepageLook(node, profiles, {}).baseWetness == Approx(0.84F));

    auto missingProfileNode = MakeSeepageNode(2U);
    missingProfileNode.lookProfileName = "Missing Profile";
    WaterSeepageLookSettings fallback;
    fallback.baseWetness = 0.61F;
    CHECK(ResolveWaterSeepageLook(missingProfileNode, profiles, fallback).baseWetness == Approx(0.61F));
}

TEST_CASE("Seepage local look names are stable and padded", "[water][seepage][naming]") {
    using invisible_places::water::WaterSeepageLocalLookName;

    CHECK(WaterSeepageLocalLookName(" saved_name ", 2U) == "saved_name_02");
    CHECK(WaterSeepageLocalLookName("Default", 3U) == "Default_03");
    CHECK(WaterSeepageLocalLookName("saved_name", 123U) == "saved_name_123");
    CHECK(WaterSeepageLocalLookName("  ", 4U) == "Default_04");
}

TEST_CASE("Seepage node look edits save discard and revert without mutating profiles", "[water][seepage][profiles]") {
    using Catch::Approx;
    using invisible_places::water::ResolveWaterSeepageLook;
    using invisible_places::water::WaterSeepageLocalLookName;

    WaterSeepageLookProfile saved;
    saved.name = "saved_name";
    saved.settings.baseWetness = 0.42F;
    std::vector<WaterSeepageLookProfile> profiles{saved};
    auto node = MakeSeepageNode(2U);
    node.lookProfileName = saved.name;

    auto edited = ResolveWaterSeepageLook(node, profiles, {});
    edited.baseWetness = 0.78F;
    node.tempLookOverride = edited;
    CHECK(WaterSeepageLocalLookName(node.lookProfileName, node.id) == "saved_name_02");
    CHECK(profiles.front().settings.baseWetness == Approx(0.42F));

    node.lookOverride = node.tempLookOverride;
    node.tempLookOverride.reset();
    CHECK(ResolveWaterSeepageLook(node, profiles, {}).baseWetness == Approx(0.78F));
    CHECK(profiles.front().settings.baseWetness == Approx(0.42F));

    auto discarded = ResolveWaterSeepageLook(node, profiles, {});
    discarded.baseWetness = 0.91F;
    node.tempLookOverride = discarded;
    node.tempLookOverride.reset();
    CHECK(ResolveWaterSeepageLook(node, profiles, {}).baseWetness == Approx(0.78F));

    node.lookOverride.reset();
    CHECK(ResolveWaterSeepageLook(node, profiles, {}).baseWetness == Approx(0.42F));
    profiles.front().settings.baseWetness = 0.55F;
    CHECK(ResolveWaterSeepageLook(node, profiles, {}).baseWetness == Approx(0.55F));
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

    const auto straightBelow = EvaluateWaterSeepageGridContribution(
        guidedGrid,
        {-0.20F, 0.0F, -0.85F},
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
    CHECK(guide.requestedReachMeters == Catch::Approx(node.reachMeters * 1.25F));
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

    const auto grid = BuildGrid({MakeSeepageNode()});
    const auto outsideNarrowHead = EvaluateWaterSeepageGridContribution(
        grid,
        {0.20F, 0.0F, -0.10F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto insideWideTail = EvaluateWaterSeepageGridContribution(
        grid,
        {0.20F, 0.0F, -1.00F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    CHECK(outsideNarrowHead.mask == 0.0F);
    CHECK(insideWideTail.mask > 0.90F);

    const auto center = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -0.625F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto featheredSide = EvaluateWaterSeepageGridContribution(
        grid,
        {0.2675F, 0.0F, -0.625F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto featheredTail = EvaluateWaterSeepageGridContribution(
        grid,
        {0.0F, 0.0F, -1.28F},
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
    CHECK(BuildGrid({node}, "ROCK", false).nodes.empty());
    CHECK(BuildGrid({node}, "ROCK", true).nodes.size() == 1U);
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

    const auto dryBeyondBaseReach = EvaluateWaterSeepageGridContribution(
        dryGrid,
        {0.0F, 0.0F, -1.36F},
        {0.0F, 1.0F, 0.0F},
        0.23F);
    const auto wetExpandedReach = EvaluateWaterSeepageGridContribution(
        wetGrid,
        {0.0F, 0.0F, -1.36F},
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
    CHECK(std::find(retainedIds.begin(), retainedIds.end(), 1U) == retainedIds.end());
    CHECK(std::find(retainedIds.begin(), retainedIds.end(), 2U) == retainedIds.end());
    CHECK(std::find(retainedIds.begin(), retainedIds.end(), 10U) != retainedIds.end());

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
    first.lookOverride = additiveLook;
    second.lookOverride = additiveLook;
    const auto addGrid = BuildGrid({first, second});
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

TEST_CASE("Seepage animation changes ripple and glint while topology stays stable", "[water][seepage][time]") {
    using invisible_places::water::EvaluateWaterSeepageGridContribution;
    using invisible_places::water::WaterSeepageParamsFingerprint;
    using invisible_places::water::WaterSeepageTopologyFingerprint;

    WaterSeepageLookSettings animated;
    animated.quality = WaterSeepageQuality::High;
    animated.density = 1.0F;
    animated.glisten = 1.0F;
    animated.speed = 1.0F;
    animated.turbulence = 1.0F;
    const auto firstGrid = BuildGrid({MakeSeepageNode()}, "ROCK", false, {}, 1'000'000ULL, animated);
    const auto atStart = EvaluateWaterSeepageGridContribution(
        firstGrid,
        {0.0F, 0.0F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.0F);
    const auto later = EvaluateWaterSeepageGridContribution(
        firstGrid,
        {0.0F, 0.0F, -0.55F},
        {0.0F, 1.0F, 0.0F},
        0.137F);
    CHECK(std::abs(atStart.ripple - later.ripple) + std::abs(atStart.glint - later.glint) > 1.0e-4F);

    auto brighter = animated;
    brighter.response.emissionAdd += 0.25F;
    const auto secondGrid = BuildGrid({MakeSeepageNode()}, "ROCK", false, {}, 1'000'000ULL, brighter);
    CHECK(WaterSeepageTopologyFingerprint(firstGrid) == WaterSeepageTopologyFingerprint(secondGrid));
    CHECK(WaterSeepageParamsFingerprint(firstGrid) != WaterSeepageParamsFingerprint(secondGrid));
}
