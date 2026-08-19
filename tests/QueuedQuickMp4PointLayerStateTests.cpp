#include "app/QueuedQuickMp4PointLayerState.hpp"

#include "serialization/ProjectDocumentJson.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <vector>

namespace {

using Catch::Approx;
using invisible_places::app::QuickMp4PointCloudLayerState;
using invisible_places::renderer::pointcloud::PointCloudColorMode;
using invisible_places::renderer::pointcloud::PointCloudColormapId;
using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
using invisible_places::renderer::pointcloud::PointCloudGeometryMode;
using invisible_places::renderer::pointcloud::PointCloudStyleState;

PointCloudStyleState MakeQueuedVisualStyle(float marker, bool alternate) {
    PointCloudStyleState style;
    style.geometryMode = alternate
                             ? PointCloudGeometryMode::WorldSurfels
                             : PointCloudGeometryMode::CameraFacingWorldSprites;
    style.falloffProfile = alternate
                               ? PointCloudFalloffProfile::Gaussian
                               : PointCloudFalloffProfile::Rim;
    style.colorMode = PointCloudColorMode::ScalarColormap;
    style.colormap = alternate ? PointCloudColormapId::Magma
                               : PointCloudColormapId::Cividis;
    style.solidColor = {marker, marker + 0.1F, marker + 0.2F, 0.65F};
    style.gradientStartColor = {marker, 0.12F, 0.23F};
    style.gradientEndColor = {0.78F, marker, 0.89F};
    style.colorizeColor = {0.31F, 0.42F, marker};
    style.colorizeAmount = marker;
    style.exposure = 0.75F + marker;
    style.innerRadius = 0.20F + marker;
    style.flowAnimation = alternate;
    style.waterPathView = !alternate;
    style.waterTrailOverlay = alternate;
    style.causticAnimation = !alternate;
    style.causticIntensity = marker * 2.0F;
    style.pointSize.constantValue[0] = 1.0F + marker;
    style.opacity.constantValue[0] = 0.50F + marker;
    style.emissiveStrength.constantValue[0] = 0.25F + marker;

    // These effective runtime fields are intentionally absent from saved
    // Visual JSON, but they are still part of the complete export style.
    style.waterFlowActivity = 0.30F + marker;
    style.waterFlowSpeedScale = 1.20F + marker;
    style.rainImpactEffects = alternate;
    return style;
}

QuickMp4PointCloudLayerState MakeLayer(
    std::size_t layerId,
    PointCloudStyleState style,
    std::uint32_t drawPointCount) {
    QuickMp4PointCloudLayerState layer;
    layer.layerId = layerId;
    layer.style = std::move(style);
    layer.drawPointCount = drawPointCount;
    return layer;
}

}  // namespace

TEST_CASE(
    "Queued animation jobs retain complete scene Visual styles through shared layer restore",
    "[animation][export][point-visual][queue]") {
    constexpr std::size_t kSandLayerId = 7U;
    constexpr std::size_t kRockLayerId = 8U;
    constexpr std::size_t kVegetationLayerId = 9U;
    constexpr std::size_t kOtherLayerId = 11U;
    const std::vector<std::size_t> visualLayerIds{
        kSandLayerId,
        kRockLayerId,
        kVegetationLayerId,
    };

    const std::vector<PointCloudStyleState> visualStylesA{
        MakeQueuedVisualStyle(0.11F, false),
        MakeQueuedVisualStyle(0.12F, true),
        MakeQueuedVisualStyle(0.13F, false),
    };
    const std::vector<PointCloudStyleState> visualStylesB{
        MakeQueuedVisualStyle(0.37F, true),
        MakeQueuedVisualStyle(0.38F, false),
        MakeQueuedVisualStyle(0.39F, true),
    };
    std::vector<nlohmann::json> visualStylesAJson;
    std::vector<nlohmann::json> visualStylesBJson;
    for (const auto& style : visualStylesA) {
        visualStylesAJson.push_back(
            invisible_places::serialization::PointCloudStyleToJson(style));
    }
    for (const auto& style : visualStylesB) {
        visualStylesBJson.push_back(
            invisible_places::serialization::PointCloudStyleToJson(style));
    }

    std::vector<QuickMp4PointCloudLayerState> animationA{
        MakeLayer(kSandLayerId, visualStylesA[0], 101U),
        MakeLayer(kRockLayerId, visualStylesA[1], 102U),
        MakeLayer(kVegetationLayerId, visualStylesA[2], 103U),
        MakeLayer(kOtherLayerId, MakeQueuedVisualStyle(0.21F, false), 51U),
    };
    std::vector<QuickMp4PointCloudLayerState> animationB{
        MakeLayer(kSandLayerId, visualStylesB[0], 201U),
        MakeLayer(kRockLayerId, visualStylesB[1], 202U),
        MakeLayer(kVegetationLayerId, visualStylesB[2], 203U),
        MakeLayer(kOtherLayerId, MakeQueuedVisualStyle(0.48F, true), 62U),
    };

    // Simulate the shared/live style changing after both per-animation Visuals
    // were queued. Source metadata still comes from this common frozen layer
    // inventory when each job starts.
    const auto laterLiveVisual = MakeQueuedVisualStyle(0.73F, false);
    const auto frozenOtherVisual = MakeQueuedVisualStyle(0.64F, true);
    std::vector<QuickMp4PointCloudLayerState> sharedBase{
        MakeLayer(kSandLayerId, laterLiveVisual, 997U),
        MakeLayer(kRockLayerId, MakeQueuedVisualStyle(0.74F, true), 998U),
        MakeLayer(
            kVegetationLayerId,
            MakeQueuedVisualStyle(0.75F, false),
            999U),
        MakeLayer(kOtherLayerId, frozenOtherVisual, 888U),
    };
    sharedBase[0].hasNormals = true;
    sharedBase[0].timingColouriseEligible = true;
    sharedBase[0].densityCompensation = {
        .footprintScale = 5.0F,
        .coverageCorrection = 0.20F,
    };
    sharedBase[3].hasSourceRgb = false;

    REQUIRE(invisible_places::app::RestoreQueuedQuickMp4PointLayerState(
        &animationA,
        sharedBase,
        visualLayerIds));
    REQUIRE(invisible_places::app::RestoreQueuedQuickMp4PointLayerState(
        &animationB,
        sharedBase,
        visualLayerIds));

    REQUIRE(animationA.size() == 4U);
    REQUIRE(animationB.size() == 4U);
    for (std::size_t index = 0U; index < visualLayerIds.size(); ++index) {
        CHECK(invisible_places::serialization::PointCloudStyleToJson(
                  animationA[index].style) == visualStylesAJson[index]);
        CHECK(invisible_places::serialization::PointCloudStyleToJson(
                  animationB[index].style) == visualStylesBJson[index]);
    }
    CHECK(invisible_places::serialization::PointCloudStyleToJson(
              animationA[0].style) !=
          invisible_places::serialization::PointCloudStyleToJson(
              laterLiveVisual));

    // Runtime-only effective style fields survive too.
    CHECK(animationA[0].style.waterFlowActivity ==
          Approx(visualStylesA[0].waterFlowActivity));
    CHECK(animationA[0].style.waterFlowSpeedScale ==
          Approx(visualStylesA[0].waterFlowSpeedScale));
    CHECK(animationA[0].style.rainImpactEffects ==
          visualStylesA[0].rainImpactEffects);
    CHECK(animationB[2].style.waterFlowActivity ==
          Approx(visualStylesB[2].waterFlowActivity));
    CHECK(animationB[2].style.waterFlowSpeedScale ==
          Approx(visualStylesB[2].waterFlowSpeedScale));
    CHECK(animationB[2].style.rainImpactEffects ==
          visualStylesB[2].rainImpactEffects);

    // Animation-specific frustum counts remain independent, while immutable
    // source facts are restored from the shared queue-time snapshot.
    CHECK(animationA[0].drawPointCount == 101U);
    CHECK(animationA[1].drawPointCount == 102U);
    CHECK(animationA[2].drawPointCount == 103U);
    CHECK(animationB[0].drawPointCount == 201U);
    CHECK(animationB[1].drawPointCount == 202U);
    CHECK(animationB[2].drawPointCount == 203U);
    CHECK(animationA[0].hasNormals);
    CHECK(animationB[0].timingColouriseEligible);
    CHECK(animationA[0].densityCompensation.footprintScale == Approx(5.0F));
    CHECK(animationB[0].densityCompensation.coverageCorrection ==
          Approx(0.20F));

    // Only the designated scene's Visual layers keep job-specific styles. An
    // unrelated layer retains the frozen style but keeps its per-job count.
    CHECK(invisible_places::serialization::PointCloudStyleToJson(
              animationA[3].style) ==
          invisible_places::serialization::PointCloudStyleToJson(
              frozenOtherVisual));
    CHECK(invisible_places::serialization::PointCloudStyleToJson(
              animationB[3].style) ==
          invisible_places::serialization::PointCloudStyleToJson(
              frozenOtherVisual));
    CHECK(animationA[3].drawPointCount == 51U);
    CHECK(animationB[3].drawPointCount == 62U);
    CHECK_FALSE(animationA[3].hasSourceRgb);
    CHECK_FALSE(animationB[3].hasSourceRgb);
}

TEST_CASE(
    "Queued animation jobs reject a missing queue-time LiDAR layer",
    "[animation][export][point-visual][queue]") {
    constexpr std::size_t kSandLayerId = 7U;
    constexpr std::size_t kRockLayerId = 8U;
    constexpr std::size_t kVegetationLayerId = 9U;
    constexpr std::size_t kLateLayerId = 99U;

    const std::vector<QuickMp4PointCloudLayerState> frozenBase{
        MakeLayer(
            kSandLayerId,
            MakeQueuedVisualStyle(0.11F, false),
            100U),
        MakeLayer(
            kRockLayerId,
            MakeQueuedVisualStyle(0.12F, true),
            200U),
        MakeLayer(
            kVegetationLayerId,
            MakeQueuedVisualStyle(0.13F, false),
            300U),
    };
    std::vector<QuickMp4PointCloudLayerState> current{
        MakeLayer(
            kSandLayerId,
            MakeQueuedVisualStyle(0.41F, true),
            101U),
        MakeLayer(
            kRockLayerId,
            MakeQueuedVisualStyle(0.42F, false),
            201U),
        // The frozen vegetation layer was unloaded; a newly loaded layer
        // cannot stand in for it even when the total count happens to match.
        MakeLayer(
            kLateLayerId,
            MakeQueuedVisualStyle(0.99F, true),
            999U),
    };
    const std::vector<std::size_t> visualLayerIds{
        kSandLayerId,
        kRockLayerId,
        kVegetationLayerId,
    };

    CHECK_FALSE(
        invisible_places::app::RestoreQueuedQuickMp4PointLayerState(
            &current,
            frozenBase,
            visualLayerIds));

    // New layers are pruned, but the helper refuses to invent the missing
    // frozen layer or partially restore a renderable inventory.
    REQUIRE(current.size() == 2U);
    CHECK(current[0].layerId == kSandLayerId);
    CHECK(current[1].layerId == kRockLayerId);
    CHECK(current[0].drawPointCount == 101U);
    CHECK(current[1].drawPointCount == 201U);
}
