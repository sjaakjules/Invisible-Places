#include "camera/CameraState.hpp"
#include "io/PointCloudData.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <glm/mat4x4.hpp>

namespace {

invisible_places::output::ExrImage RenderSinglePoint(
    float opacity,
    float emissive,
    invisible_places::renderer::pointcloud::PointCloudDensityCompensation densityCompensation,
    invisible_places::style::RenderParameterBinding* opacityBinding = nullptr,
    bool fastBasic = false) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFFFFFFFFU};
    cloud.hasSourceRgb = true;

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {0.8F, 0.6F, 0.4F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.solidCenters = true;
    invisible_places::style::SetScalarConstant(&style.pointSize, 2.0F);
    invisible_places::style::SetScalarConstant(&style.opacity, opacity);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, emissive);
    if (opacityBinding != nullptr) {
        style.opacity = *opacityBinding;
    }

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .fastBasic = fastBasic,
        .localToWorld = glm::mat4{1.0F},
        .densityCompensation = densityCompensation,
    };

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, 0.0F, 5.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 32U, 32U);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0U, 0U, 32U, 32U},
        &image);
    return image;
}

invisible_places::output::ExrImage RenderSingleWaterTrail(
    float activity,
    float trailSeed,
    bool fastBasic = false) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFFFFFFFFU};
    cloud.hasSourceRgb = true;
    cloud.bounds.Expand(cloud.positions.front());
    constexpr std::size_t kWaterTrailFieldCount = 31U;
    for (std::size_t fieldIndex = 0; fieldIndex < kWaterTrailFieldCount; ++fieldIndex) {
        cloud.scalarFields.push_back({
            .name = "stream_" + std::to_string(fieldIndex),
            .minimum = 0.0F,
            .maximum = 1.0F,
            .count = 1U,
            .valid = true,
        });
        cloud.scalarFieldValues.push_back(0.0F);
    }
    auto setField = [&cloud](std::size_t fieldSlot, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, 0U)] = value;
    };
    setField(0U, 1.0F);   // trail_role
    setField(5U, trailSeed);
    setField(11U, 1.0F);  // route_length
    setField(17U, 0.30F); // trail_width
    setField(18U, 0.40F); // trail_streak_length
    setField(22U, 1.0F);  // tangent_x

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterTrailOverlay = true;
    style.waterFlowActivity = activity;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.30F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .generatedWaterOverlay = true,
        .hasSourceRgb = true,
        .fastBasic = fastBasic,
        .localToWorld = glm::mat4{1.0F},
    };
    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, 0.0F, 5.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 32U, 32U);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0U, 0U, 32U, 32U},
        &image);
    return image;
}

invisible_places::output::ExrImage RenderMovingWaterTrail(
    float speedScale,
    bool fastBasic = false) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {
        {-1.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
    };
    cloud.normals.assign(cloud.positions.size(), {0.0F, 0.0F, 1.0F});
    cloud.packedColors.assign(cloud.positions.size(), 0xFFFFFFFFU);
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    for (const auto& position : cloud.positions) {
        cloud.bounds.Expand(position);
    }
    constexpr std::size_t kScalarFieldCount = 31U;
    for (std::size_t fieldIndex = 0; fieldIndex < kScalarFieldCount; ++fieldIndex) {
        cloud.scalarFields.push_back({
            .name = "stream_" + std::to_string(fieldIndex),
            .minimum = 0.0F,
            .maximum = 2.0F,
            .count = cloud.PointCount(),
            .valid = true,
        });
    }
    cloud.scalarFieldValues.assign(kScalarFieldCount * cloud.PointCount(), 0.0F);
    auto setTrailField = [&cloud](std::size_t fieldSlot, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, 2U)] = value;
    };
    setTrailField(0U, 1.0F);   // trail_role
    setTrailField(5U, 0.0F);   // trail_seed
    setTrailField(9U, 0.0F);   // route_start
    setTrailField(10U, 2.0F);  // route_count
    setTrailField(11U, 2.0F);  // route_length
    setTrailField(16U, 1.0F);  // trail_speed
    setTrailField(17U, 0.10F); // trail_width
    setTrailField(18U, 0.40F); // trail_streak_length
    setTrailField(22U, 1.0F);  // tangent_x

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterTrailOverlay = true;
    style.waterFlowActivity = 1.0F;
    style.waterFlowSpeedScale = speedScale;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.10F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .generatedWaterOverlay = true,
        .hasSourceRgb = true,
        .fastBasic = fastBasic,
        .localToWorld = glm::mat4{1.0F},
    };
    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, 0.0F, 5.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 64U, 64U);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0U, 0U, 64U, 64U},
        &image,
        nullptr,
        nullptr,
        0.5F);
    return image;
}

std::size_t CoveredPixelCount(const invisible_places::output::ExrImage& image) {
    return static_cast<std::size_t>(std::count_if(
        image.alpha.begin(),
        image.alpha.end(),
        [](float alpha) { return alpha > 1.0e-5F; }));
}

float Maximum(const std::vector<float>& values) {
    return values.empty() ? 0.0F : *std::max_element(values.begin(), values.end());
}

float AlphaCentroidX(const invisible_places::output::ExrImage& image) {
    double weightedX = 0.0;
    double totalAlpha = 0.0;
    for (std::uint32_t y = 0U; y < image.height; ++y) {
        for (std::uint32_t x = 0U; x < image.width; ++x) {
            const float alpha = image.alpha[static_cast<std::size_t>(y) * image.width + x];
            weightedX += static_cast<double>(x) * alpha;
            totalAlpha += alpha;
        }
    }
    return totalAlpha > 0.0 ? static_cast<float>(weightedX / totalAlpha) : -1.0F;
}

}  // namespace

TEST_CASE("Density compensation resolves spacing and count-aware coverage", "[pointcloud][density]") {
    using invisible_places::renderer::pointcloud::ResolvePointCloudDensityCompensation;

    const auto oneMillimeter = ResolvePointCloudDensityCompensation(0.001F, 1000U, 0.001F, 1000U);
    CHECK(oneMillimeter.footprintScale == Catch::Approx(1.0F));
    CHECK(oneMillimeter.coverageCorrection == Catch::Approx(1.0F));

    const auto idealFiveMillimeter = ResolvePointCloudDensityCompensation(0.005F, 40U, 0.001F, 1000U);
    CHECK(idealFiveMillimeter.footprintScale == Catch::Approx(5.0F));
    CHECK(idealFiveMillimeter.coverageCorrection == Catch::Approx(1.0F));

    const auto underCoveredFiveMillimeter =
        ResolvePointCloudDensityCompensation(0.005F, 20U, 0.001F, 1000U);
    CHECK(underCoveredFiveMillimeter.footprintScale == Catch::Approx(5.0F * std::sqrt(2.0F)));
    CHECK(underCoveredFiveMillimeter.coverageCorrection == Catch::Approx(1.0F));

    // An over-covered bundle keeps its nominal pitch, so the footprint never
    // shrinks below the spacing ratio; the residual becomes per-fragment alpha.
    const auto overCoveredFiveMillimeter =
        ResolvePointCloudDensityCompensation(0.005F, 80U, 0.001F, 1000U);
    CHECK(overCoveredFiveMillimeter.footprintScale == Catch::Approx(5.0F));
    CHECK(overCoveredFiveMillimeter.coverageCorrection == Catch::Approx(0.5F));

    // Scene3 ROCK measured counts: 2,317,741 of 39,451,487 at 5 mm.
    const auto scene3Rock =
        ResolvePointCloudDensityCompensation(0.005F, 2'317'741U, 0.001F, 39'451'487U);
    CHECK(scene3Rock.footprintScale == Catch::Approx(5.0F));
    CHECK(scene3Rock.coverageCorrection == Catch::Approx(0.6809F).epsilon(1.0e-3));

    // A sparse pseudo-canonical reference (Site1 VEG: 22,065,445 at 5 mm vs
    // 28,404,413 at "1 mm") must not drive per-fragment alpha to extremes:
    // the coverage correction floors at 1/4 so roles stay cohesive live.
    const auto site1Veg =
        ResolvePointCloudDensityCompensation(0.005F, 22'065'445U, 0.001F, 28'404'413U);
    CHECK(site1Veg.footprintScale == Catch::Approx(5.0F));
    CHECK(site1Veg.coverageCorrection ==
          Catch::Approx(invisible_places::renderer::pointcloud::
                            kPointCloudCoverageCorrectionFloor));

    const auto clamped = ResolvePointCloudDensityCompensation(0.005F, 1U, 0.001F, 1'000'000U);
    CHECK(clamped.footprintScale == Catch::Approx(20.0F));
    CHECK(clamped.coverageCorrection == Catch::Approx(1.0F));

    const auto missingCounts = ResolvePointCloudDensityCompensation(0.005F, 0U, 0.001F, 1000U);
    CHECK(missingCounts.footprintScale == Catch::Approx(5.0F));
    CHECK(missingCounts.coverageCorrection == Catch::Approx(1.0F));

    // Standalone spaced clouds (no scene bundle, e.g. Site1-WATER-5mm) have
    // no sibling reference at all: spacing-only compensation must still cover
    // the authored pitch under the ideal-lattice assumption.
    const auto standaloneFiveMillimeter = ResolvePointCloudDensityCompensation(0.005F, 0U, 0.0F, 0U);
    CHECK(standaloneFiveMillimeter.footprintScale == Catch::Approx(5.0F));
    CHECK(standaloneFiveMillimeter.coverageCorrection == Catch::Approx(1.0F));

    // Site1 WATER uses its finest 2 mm sibling as the family reference. The
    // 2 mm export therefore keeps a 2x footprint at identity alpha/emission,
    // while the 5 mm live variant keeps a 5x footprint and attenuates its
    // measured over-coverage so the two densities integrate consistently.
    const auto site1WaterTwoMillimeter =
        ResolvePointCloudDensityCompensation(
            0.002F,
            35'528'278U,
            0.002F,
            35'528'278U);
    const auto site1WaterFiveMillimeter =
        ResolvePointCloudDensityCompensation(
            0.005F,
            6'456'640U,
            0.002F,
            35'528'278U);
    CHECK(site1WaterTwoMillimeter.footprintScale == Catch::Approx(2.0F));
    CHECK(site1WaterTwoMillimeter.coverageCorrection == Catch::Approx(1.0F));
    CHECK(site1WaterFiveMillimeter.footprintScale == Catch::Approx(5.0F));
    CHECK(site1WaterFiveMillimeter.coverageCorrection ==
          Catch::Approx(0.8806F).epsilon(1.0e-3));
}

TEST_CASE(
    "Adaptive HQ derives a projected-spacing transition and safe keep weights",
    "[pointcloud][density][adaptive-hq]") {
    using namespace invisible_places::renderer::pointcloud;

    const auto transition = ResolvePointCloudAdaptiveDensityTransition(
        2.0F,
        1000.0F,
        0.005F,
        1.0F);
    REQUIRE(transition.Valid());
    CHECK(transition.switchDepthMeters == Catch::Approx(5.0F));
    CHECK(transition.startDepthMeters == Catch::Approx(3.25F));
    CHECK(transition.endDepthMeters == Catch::Approx(6.75F));
    CHECK(transition.preparedFineDepthMeters >
          transition.endDepthMeters);

    CHECK(PointCloudAdaptiveDensityCoarseWeight(3.0F, transition) ==
          Catch::Approx(0.0F));
    CHECK(PointCloudAdaptiveDensityCoarseWeight(5.0F, transition) ==
          Catch::Approx(0.5F));
    CHECK(PointCloudAdaptiveDensityCoarseWeight(7.0F, transition) ==
          Catch::Approx(1.0F));
    CHECK(PointCloudAdaptiveDensityKeepProbability(
              PointCloudAdaptiveDensityRole::Fine,
              5.0F,
              transition) == Catch::Approx(0.25F));
    CHECK(PointCloudAdaptiveDensityKeepProbability(
              PointCloudAdaptiveDensityRole::Coarse,
              5.0F,
              transition) == Catch::Approx(0.5F));
    CHECK(PointCloudAdaptiveDensityKeepProbability(
              PointCloudAdaptiveDensityRole::Disabled,
              5.0F,
              transition) == Catch::Approx(1.0F));
    // An outgoing fine layer's steady state is fully handed off: the
    // complementary handoff share it draws mid-crossfade is a GPU-only
    // runtime blend, so the CPU model reports zero.
    CHECK(PointCloudAdaptiveDensityKeepProbability(
              PointCloudAdaptiveDensityRole::FineOutgoing,
              5.0F,
              transition) == Catch::Approx(0.0F));
    CHECK(PointCloudAdaptiveDensityKeepProbability(
              PointCloudAdaptiveDensityRole::FineOutgoing,
              3.0F,
              transition) == Catch::Approx(0.0F));

    const auto invalid = ResolvePointCloudAdaptiveDensityTransition(
        0.0F,
        1000.0F);
    CHECK_FALSE(invalid.Valid());
    CHECK(PointCloudAdaptiveDensityKeepProbability(
              PointCloudAdaptiveDensityRole::Fine,
              5.0F,
              invalid) == Catch::Approx(1.0F));
}

TEST_CASE("Density compensation preserves reference coverage without shrinking the footprint", "[pointcloud][density]") {
    using invisible_places::renderer::pointcloud::ResolvePointCloudDensityCompensation;

    const auto checkArea = [](float displaySpacing,
                              std::uint64_t displayCount,
                              float referenceSpacing,
                              std::uint64_t referenceCount) {
        const auto compensation = ResolvePointCloudDensityCompensation(
            displaySpacing,
            displayCount,
            referenceSpacing,
            referenceCount);
        // Effective covered area = count x footprint^2 x per-fragment alpha
        // correction. Under-covered sources grow the footprint; over-covered
        // sources keep the nominal footprint and attenuate alpha instead.
        const double displayArea =
            static_cast<double>(displayCount) * compensation.footprintScale *
            compensation.footprintScale * compensation.coverageCorrection;
        const double referenceFootprint =
            static_cast<double>(referenceSpacing) / 0.001;
        const double referenceArea =
            static_cast<double>(referenceCount) * referenceFootprint * referenceFootprint;
        CHECK(displayArea == Catch::Approx(referenceArea).epsilon(1.0e-5));
        const float nominalFootprint = displaySpacing / 0.001F;
        CHECK(compensation.footprintScale >= Catch::Approx(nominalFootprint));
        CHECK(compensation.coverageCorrection <= Catch::Approx(1.0F));
        CHECK((compensation.footprintScale == Catch::Approx(nominalFootprint) ||
               compensation.coverageCorrection == Catch::Approx(1.0F)));
    };

    checkArea(0.001F, 1000U, 0.001F, 1000U);
    checkArea(0.005F, 40U, 0.001F, 1000U);
    checkArea(0.005F, 20U, 0.001F, 1000U);
    checkArea(0.005F, 80U, 0.001F, 1000U);
    checkArea(0.005F, 160U, 0.002F, 1000U);
}

TEST_CASE(
    "Density compensation keeps antialias support unscaled before depth of field",
    "[pointcloud][density][footprint]") {
    using invisible_places::renderer::pointcloud::PointCloudDensityCompensation;
    using invisible_places::renderer::pointcloud::ClampPointCloudResolvedSurfelDiameter;
    using invisible_places::renderer::pointcloud::ResolvePointCloudDensityAdjustedFootprint;

    CHECK(
        ResolvePointCloudDensityAdjustedFootprint(
            3.0F,
            1.0F,
            2.0F,
            PointCloudDensityCompensation{1.0F, 1.0F}) ==
        Catch::Approx(6.0F));
    // Only the authored kernel scales with density; the antialias margin is
    // the same fixed screen-space pad for every display bundle so the live
    // coarse view and the fine-bundle export stay size-consistent at every
    // camera distance.
    CHECK(
        ResolvePointCloudDensityAdjustedFootprint(
            3.0F,
            1.0F,
            2.0F,
            PointCloudDensityCompensation{4.0F, 1.0F}) ==
        Catch::Approx(15.0F));
    // Coverage correction changes fragment opacity, not geometry.
    CHECK(
        ResolvePointCloudDensityAdjustedFootprint(
            3.0F,
            1.0F,
            2.0F,
            PointCloudDensityCompensation{4.0F, 8.0F}) ==
        Catch::Approx(15.0F));
    // Signed water size additions stay composed until the same final
    // geometry clamp used by the GPU.
    CHECK(
        ResolvePointCloudDensityAdjustedFootprint(
            -1.0F,
            1.0F,
            0.0F,
            PointCloudDensityCompensation{5.0F, 1.0F}) ==
        Catch::Approx(-4.0F));
    CHECK(
        ClampPointCloudResolvedSurfelDiameter(
            ResolvePointCloudDensityAdjustedFootprint(
                20.0F,
                1.0F,
                2.0F,
                PointCloudDensityCompensation{5.0F, 1.0F}),
            64.0F) ==
        Catch::Approx(64.0F));
    CHECK(
        ClampPointCloudResolvedSurfelDiameter(-3.0F, 64.0F) ==
        Catch::Approx(0.0F));
}

TEST_CASE("Water Flow activity scales are deterministic and monotonic", "[pointcloud][water][activity]") {
    using invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales;

    const auto off = ResolveWaterFlowActivityScales(0.0F, 0.0F);
    CHECK(off.activity == 0.0F);
    CHECK(off.trailVisibility == 0.0F);
    CHECK(off.appearance == Catch::Approx(0.30F));
    CHECK(off.width == Catch::Approx(0.65F));
    CHECK(off.speed == Catch::Approx(1.0F));
    CHECK(off.visibleLength == Catch::Approx(0.55F));
    CHECK(off.lateralMotion == Catch::Approx(0.15F));

    const auto full = ResolveWaterFlowActivityScales(1.0F, 1.0F);
    CHECK(full.trailVisibility == 1.0F);
    CHECK(full.appearance == 1.0F);
    CHECK(full.width == 1.0F);
    CHECK(full.speed == 1.0F);
    CHECK(full.visibleLength == 1.0F);
    CHECK(full.lateralMotion == Catch::Approx(0.15F));

    const auto earlyTrail = ResolveWaterFlowActivityScales(0.25F, 0.10F);
    const auto lateTrail = ResolveWaterFlowActivityScales(0.25F, 0.80F);
    CHECK(earlyTrail.trailVisibility == Catch::Approx(0.25F));
    CHECK(lateTrail.trailVisibility == Catch::Approx(0.25F));
    CHECK(earlyTrail.speed == lateTrail.speed);
    CHECK(earlyTrail.lateralMotion == lateTrail.lateralMotion);

    float previousCoverage = 0.0F;
    for (std::uint32_t step = 0U; step <= 1000U; ++step) {
        const float activity = static_cast<float>(step) / 1000.0F;
        for (const float seed : {0.0F, 0.10F, 0.50F, 0.80F, 1.0F}) {
            const auto scales = ResolveWaterFlowActivityScales(activity, seed);
            CHECK(scales.trailVisibility == Catch::Approx(activity));
            CHECK(scales.speed == Catch::Approx(1.0F));
            CHECK(scales.lateralMotion == Catch::Approx(0.15F));
        }
        const auto scales = ResolveWaterFlowActivityScales(activity, 0.5F);
        const float coverage = scales.trailVisibility * scales.appearance;
        CHECK(coverage >= previousCoverage);
        if (step > 0U) {
            CHECK(coverage - previousCoverage < 0.002F);
        }
        previousCoverage = coverage;
    }
}

TEST_CASE("Water Flow runtime speed scale is safe and survives Fast Basic styling", "[pointcloud][water][speed]") {
    using invisible_places::renderer::pointcloud::MakeFastBasicPointCloudStyle;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::SanitizeWaterFlowSpeedScale;

    CHECK(SanitizeWaterFlowSpeedScale(0.0F) == 0.0F);
    CHECK(SanitizeWaterFlowSpeedScale(2.5F) == Catch::Approx(2.5F));
    CHECK(SanitizeWaterFlowSpeedScale(-1.0F) == 0.0F);
    CHECK(SanitizeWaterFlowSpeedScale(std::numeric_limits<float>::quiet_NaN()) == 1.0F);

    PointCloudStyleState style;
    CHECK(style.waterFlowSpeedScale == 1.0F);
    style.waterFlowSpeedScale = 1.75F;
    CHECK(MakeFastBasicPointCloudStyle(style, true).waterFlowSpeedScale == Catch::Approx(1.75F));
}

TEST_CASE(
    "Fast Basic keeps only generated Flow trails on Beauty",
    "[pointcloud][renderer][fast-basic][water]") {
    using invisible_places::renderer::pointcloud::PointCloudRendererMode;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::ResolvePointCloudLayerRendererMode;

    PointCloudStyleState style;
    CHECK(
        ResolvePointCloudLayerRendererMode(
            PointCloudRendererMode::FastBasic,
            false,
            style) == PointCloudRendererMode::FastBasic);

    style.waterTrailOverlay = true;
    CHECK(
        ResolvePointCloudLayerRendererMode(
            PointCloudRendererMode::FastBasic,
            false,
            style) == PointCloudRendererMode::FastBasic);
    CHECK(
        ResolvePointCloudLayerRendererMode(
            PointCloudRendererMode::FastBasic,
            true,
            style) == PointCloudRendererMode::Beauty);
    CHECK(
        ResolvePointCloudLayerRendererMode(
            PointCloudRendererMode::Beauty,
            true,
            style) == PointCloudRendererMode::Beauty);

    style.waterTrailOverlay = false;
    style.flowAnimation = true;
    CHECK(
        ResolvePointCloudLayerRendererMode(
            PointCloudRendererMode::FastBasic,
            true,
            style) == PointCloudRendererMode::FastBasic);
}

TEST_CASE(
    "Offline mixed renderer keeps Fast Basic and Beauty layers on separate paths",
    "[output][offline][pointcloud][fast-basic][mixed]") {
    invisible_places::io::LoadedPointCloud fastCloud;
    fastCloud.positions = {{-0.35F, 0.0F, 0.0F}};
    fastCloud.packedColors = {0xFFFFFFFFU};
    fastCloud.hasSourceRgb = true;

    invisible_places::io::LoadedPointCloud beautyCloud;
    beautyCloud.positions = {{0.35F, 0.0F, 0.0F}};
    beautyCloud.packedColors = {0xFFFFFFFFU};
    beautyCloud.hasSourceRgb = true;

    invisible_places::renderer::pointcloud::PointCloudStyleState fastStyle;
    fastStyle.colorMode =
        invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    fastStyle.solidColor = {1.0F, 0.0F, 0.0F, 1.0F};
    fastStyle.falloffProfile =
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    invisible_places::style::SetScalarConstant(&fastStyle.pointSize, 5.0F);
    invisible_places::style::SetScalarConstant(&fastStyle.opacity, 0.1F);

    auto beautyStyle = fastStyle;
    beautyStyle.solidColor = {0.0F, 0.0F, 1.0F, 1.0F};
    invisible_places::style::SetScalarConstant(&beautyStyle.opacity, 0.25F);

    const std::vector<invisible_places::output::OfflinePointLayer> layers{
        {.cloud = &fastCloud,
         .style = fastStyle,
         .hasSourceRgb = true,
         .fastBasic = true,
         .localToWorld = glm::mat4{1.0F}},
        {.cloud = &beautyCloud,
         .style = beautyStyle,
         .hasSourceRgb = true,
         .fastBasic = false,
         .localToWorld = glm::mat4{1.0F}},
    };

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, 0.0F, 5.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 32U, 32U);
    invisible_places::output::OfflinePointRenderDiagnostics diagnostics;
    invisible_places::output::RenderPointCloudTile(
        layers,
        cameraState,
        invisible_places::output::OfflineRenderTile{0U, 0U, 32U, 32U},
        &image,
        &diagnostics);

    float fastAlpha = 0.0F;
    float beautyAlpha = 0.0F;
    for (std::size_t index = 0U; index < image.alpha.size(); ++index) {
        if (image.beautyR[index] > image.beautyB[index]) {
            fastAlpha = std::max(fastAlpha, image.alpha[index]);
        } else if (image.beautyB[index] > image.beautyR[index]) {
            beautyAlpha = std::max(beautyAlpha, image.alpha[index]);
        }
    }

    CHECK(fastAlpha == Catch::Approx(1.0F));
    CHECK(beautyAlpha == Catch::Approx(0.25F).margin(1.0e-4F));
    CHECK(diagnostics.depthPassLayers == 1U);
    CHECK(diagnostics.accumulationPassLayers == 2U);
}

TEST_CASE("Offline Water Flow speed scale changes travel without scalar edits", "[output][offline][water][speed]") {
    const auto frozen = RenderMovingWaterTrail(0.0F);
    const auto moving = RenderMovingWaterTrail(1.0F);
    CHECK(AlphaCentroidX(frozen) >= 0.0F);
    CHECK(AlphaCentroidX(moving) > AlphaCentroidX(frozen) + 1.0F);

    const auto fastFrozen = RenderMovingWaterTrail(0.0F, true);
    const auto fastMoving = RenderMovingWaterTrail(1.0F, true);
    CHECK(AlphaCentroidX(fastFrozen) >= 0.0F);
    CHECK(AlphaCentroidX(fastMoving) > AlphaCentroidX(fastFrozen) + 1.0F);
}

TEST_CASE("Offline Water Flow activity hides and reveals stable trails", "[output][offline][water][activity]") {
    const auto off = RenderSingleWaterTrail(0.0F, 0.1F);
    const auto late = RenderSingleWaterTrail(0.25F, 0.8F);
    const auto early = RenderSingleWaterTrail(0.25F, 0.1F);
    const auto full = RenderSingleWaterTrail(1.0F, 0.8F);
    CHECK(Maximum(off.alpha) == 0.0F);
    CHECK(Maximum(late.alpha) > 0.0F);
    CHECK(Maximum(early.alpha) > 0.0F);
    // The seed may still move a trail within its stable micro-motion pattern,
    // but it no longer decides whether that trail exists at this activity.
    CHECK(Maximum(late.alpha) > Maximum(early.alpha) * 0.5F);
    CHECK(Maximum(early.alpha) > Maximum(late.alpha) * 0.5F);
    CHECK(Maximum(full.alpha) > Maximum(early.alpha));

    const auto fastOff = RenderSingleWaterTrail(0.0F, 0.1F, true);
    const auto fastPartialEarly = RenderSingleWaterTrail(0.25F, 0.1F, true);
    const auto fastPartialLate = RenderSingleWaterTrail(0.25F, 0.8F, true);
    const auto fastFull = RenderSingleWaterTrail(1.0F, 0.1F, true);
    CHECK(Maximum(fastOff.alpha) == 0.0F);
    CHECK(Maximum(fastPartialEarly.alpha) > 0.0F);
    CHECK(Maximum(fastPartialLate.alpha) > 0.0F);
    CHECK(Maximum(fastFull.alpha) > 0.0F);
}

TEST_CASE("Density compensation forces the unified Beauty material", "[pointcloud][density][material]") {
    using invisible_places::renderer::pointcloud::PointCloudDensityCompensation;
    using invisible_places::renderer::pointcloud::PointCloudMaterialVariant;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::ResolvePointCloudDensityCompensation;
    using invisible_places::renderer::pointcloud::ResolvePointCloudMaterialVariant;

    const PointCloudStyleState style;
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::OpaqueHardDisc);
    CHECK(
        ResolvePointCloudMaterialVariant(style, PointCloudDensityCompensation{5.0F, 1.0F}) ==
        PointCloudMaterialVariant::Unified);
    CHECK(
        ResolvePointCloudMaterialVariant(style, PointCloudDensityCompensation{5.0F, 1.01F}) ==
        PointCloudMaterialVariant::Unified);
    CHECK(
        ResolvePointCloudMaterialVariant(
            style,
            ResolvePointCloudDensityCompensation(0.005F, 80U, 0.001F, 1000U)) ==
        PointCloudMaterialVariant::Unified);
}

TEST_CASE("Preview depth culling follows the authored opaque style", "[pointcloud][preview][performance]") {
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::PointCloudStyleSupportsPreviewDepthCulling;

    PointCloudStyleState style;
    CHECK(PointCloudStyleSupportsPreviewDepthCulling(style));

    invisible_places::style::SetScalarConstant(&style.opacity, 0.5F);
    CHECK_FALSE(PointCloudStyleSupportsPreviewDepthCulling(style));

    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.25F);
    CHECK_FALSE(PointCloudStyleSupportsPreviewDepthCulling(style));
}

TEST_CASE("Paused shoreline settings do not require animated redraws", "[pointcloud][shoreline][cache]") {
    using invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm;
    using invisible_places::renderer::pointcloud::PointCloudShorelineWaveSettings;
    using invisible_places::renderer::pointcloud::PointCloudShorelineWaveSettingsHasActiveMotion;

    PointCloudShorelineWaveSettings settings;
    CHECK_FALSE(PointCloudShorelineWaveSettingsHasActiveMotion(settings));

    settings.enabled = true;
    CHECK(PointCloudShorelineWaveSettingsHasActiveMotion(settings));
    settings.foamFronts.speed = 0.0F;
    CHECK_FALSE(PointCloudShorelineWaveSettingsHasActiveMotion(settings));

    settings.algorithm = PointCloudShorelineWaveAlgorithm::HeightFoam;
    CHECK(PointCloudShorelineWaveSettingsHasActiveMotion(settings));
    settings.heightFoam.speed = 0.0F;
    CHECK_FALSE(PointCloudShorelineWaveSettingsHasActiveMotion(settings));
}

TEST_CASE("Offline density compensation scales footprint before alpha and emission", "[output][offline][density]") {
    using invisible_places::renderer::pointcloud::PointCloudDensityCompensation;

    const auto fine = RenderSinglePoint(0.2F, 0.0F, PointCloudDensityCompensation{1.0F, 1.0F});
    const auto sparse = RenderSinglePoint(0.2F, 0.0F, PointCloudDensityCompensation{5.0F, 1.0F});
    CHECK(CoveredPixelCount(sparse) > CoveredPixelCount(fine));

    const auto authoredOpacity =
        RenderSinglePoint(0.2F, 2.0F, PointCloudDensityCompensation{1.0F, 1.0F});
    const auto compensatedOpacity =
        RenderSinglePoint(0.1F, 2.0F, PointCloudDensityCompensation{1.0F, 2.0F});
    CHECK(Maximum(compensatedOpacity.alpha) == Catch::Approx(Maximum(authoredOpacity.alpha)).margin(1.0e-5F));
    CHECK(Maximum(compensatedOpacity.beautyR) == Catch::Approx(Maximum(authoredOpacity.beautyR)).margin(1.0e-5F));
}

TEST_CASE("Fast Basic applies footprint scale while remaining opaque", "[output][offline][density][fast-basic]") {
    using invisible_places::renderer::pointcloud::ResolvePointCloudDensityCompensation;

    const auto fine =
        RenderSinglePoint(
            0.0F,
            0.0F,
            ResolvePointCloudDensityCompensation(0.001F, 1000U, 0.001F, 1000U),
            nullptr,
            true);
    const auto sparse =
        RenderSinglePoint(
            0.0F,
            8.0F,
            ResolvePointCloudDensityCompensation(0.005F, 20U, 0.001F, 1000U),
            nullptr,
            true);
    CHECK(CoveredPixelCount(sparse) > CoveredPixelCount(fine));
    CHECK(Maximum(fine.alpha) == Catch::Approx(1.0F));
    CHECK(Maximum(sparse.alpha) == Catch::Approx(1.0F));
    CHECK(Maximum(fine.beautyR) == Catch::Approx(Maximum(sparse.beautyR)));
}

TEST_CASE("Missing offline scalar fields retain the authored constant fallback", "[output][offline][field-map]") {
    invisible_places::style::RenderParameterBinding missingField;
    invisible_places::style::SetScalarConstant(&missingField, 0.3F);
    invisible_places::style::ConfigureFieldMapFromStats(
        &missingField,
        7,
        "Interest",
        0.8F,
        1.0F,
        nullptr);

    const auto constant = RenderSinglePoint(0.3F, 0.0F, {});
    const auto missing = RenderSinglePoint(1.0F, 0.0F, {}, &missingField);
    CHECK(Maximum(missing.alpha) == Catch::Approx(Maximum(constant.alpha)).margin(1.0e-5F));
    CHECK(Maximum(missing.beautyR) == Catch::Approx(Maximum(constant.beautyR)).margin(1.0e-5F));
}

TEST_CASE(
    "Offline timing colourise uses scalar bounds without changing point opacity",
    "[output][offline][timing-colourise]") {
    using invisible_places::renderer::pointcloud::PointCloudColorMode;
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::ResolvedTimingColouriseStack;
    using invisible_places::renderer::pointcloud::TimingColouriseSource;

    auto render = [](float scalarValue, bool fastBasic) {
        invisible_places::io::LoadedPointCloud cloud;
        cloud.positions = {{0.0F, 0.0F, 0.0F}};
        cloud.scalarFields = {{
            .name = "Interest",
            .minimum = 0.0F,
            .maximum = 2.0F,
            .count = 1U,
            .valid = true,
        }};
        cloud.scalarFieldValues = {scalarValue};

        PointCloudStyleState style;
        style.colorMode = PointCloudColorMode::SolidColor;
        style.solidColor = {0.20F, 0.40F, 0.80F, 1.0F};
        style.falloffProfile = PointCloudFalloffProfile::HardDisc;
        style.solidCenters = true;
        invisible_places::style::SetScalarConstant(&style.pointSize, 4.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);

        ResolvedTimingColouriseStack stack;
        REQUIRE(stack.effects.size() == 8U);
        stack.effectCount = static_cast<std::uint32_t>(
            stack.effects.size());
        for (auto& transparentEffect : stack.effects) {
            transparentEffect.enabled = true;
            transparentEffect.source = TimingColouriseSource::ScalarField;
            transparentEffect.scalarFieldSlot = 0;
            transparentEffect.lowerBound = 0.5F;
            transparentEffect.upperBound = 1.5F;
            transparentEffect.edgeFadeLowerFraction = 0.25F;
            transparentEffect.edgeFadeUpperFraction = 0.25F;
            transparentEffect.rgbaLut.fill({0.0F, 1.0F, 0.0F, 0.0F});
        }
        // Put the visible layer in the eighth slot so this fixture proves the
        // complete renderer/offline capacity is evaluated.
        auto& effect = stack.effects.back();
        effect.enabled = true;
        effect.source = TimingColouriseSource::ScalarField;
        effect.scalarFieldSlot = 0;
        effect.lowerBound = 0.5F;
        effect.upperBound = 1.5F;
        effect.edgeFadeLowerFraction = 0.25F;
        effect.edgeFadeUpperFraction = 0.25F;
        effect.rgbaLut.fill({1.0F, 0.0F, 0.0F, 1.0F});

        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .timingColourise = stack,
            .hasSourceRgb = false,
            .fastBasic = fastBasic,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::camera::CameraState cameraState;
        cameraState.position = {0.0F, 0.0F, 5.0F};
        cameraState.target = {0.0F, 0.0F, 0.0F};
        cameraState.nearPlane = 0.1F;
        cameraState.farPlane = 20.0F;

        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 9U, 9U);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0U, 0U, 9U, 9U},
            &image);
        return image;
    };

    for (const bool fastBasic : {false, true}) {
        const auto outside = render(0.25F, fastBasic);
        const auto above = render(1.75F, fastBasic);
        const auto notANumber =
            render(std::numeric_limits<float>::quiet_NaN(), fastBasic);
        const auto positiveInfinity =
            render(std::numeric_limits<float>::infinity(), fastBasic);
        const auto negativeInfinity =
            render(-std::numeric_limits<float>::infinity(), fastBasic);
        const auto edge = render(0.5F, fastBasic);
        const auto inside = render(1.0F, fastBasic);
        const std::size_t center = 4U * 9U + 4U;
        REQUIRE(outside.alpha[center] > 0.0F);
        for (const auto* rejected : {
                 &above,
                 &notANumber,
                 &positiveInfinity,
                 &negativeInfinity,
             }) {
            REQUIRE(rejected->alpha[center] > 0.0F);
            CHECK(
                rejected->beautyR[center] ==
                Catch::Approx(outside.beautyR[center]).margin(1.0e-5F));
            CHECK(
                rejected->beautyG[center] ==
                Catch::Approx(outside.beautyG[center]).margin(1.0e-5F));
            CHECK(
                rejected->beautyB[center] ==
                Catch::Approx(outside.beautyB[center]).margin(1.0e-5F));
            CHECK(
                rejected->alpha[center] ==
                Catch::Approx(outside.alpha[center]).margin(1.0e-5F));
        }
        CHECK(edge.beautyB[center] == Catch::Approx(outside.beautyB[center]).margin(1.0e-5F));
        CHECK(inside.beautyR[center] > inside.beautyB[center]);
        CHECK(inside.alpha[center] == Catch::Approx(outside.alpha[center]).margin(1.0e-5F));
    }
}

TEST_CASE(
    "Offline timing emissive supports masked glow and darkening",
    "[output][offline][timing-colourise][emissive]") {
    using invisible_places::renderer::pointcloud::PointCloudColorMode;
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::ResolvedTimingColouriseStack;
    using invisible_places::renderer::pointcloud::TimingColouriseOutput;
    using invisible_places::renderer::pointcloud::TimingColouriseSource;

    auto render = [](
                      float fieldValue,
                      float emissiveLevel,
                      bool fastBasic,
                      bool enabled = true,
                      bool useNormalZ = false,
                      float edgeFadeFraction = 0.25F,
                      float secondEmissiveLevel = 0.0F) {
        invisible_places::io::LoadedPointCloud cloud;
        cloud.positions = {{0.0F, 0.0F, 0.0F}};
        cloud.normals = {{0.0F, 0.0F, fieldValue}};
        cloud.hasNormals = true;
        cloud.scalarFields = {{
            .name = "Interest",
            .minimum = 0.0F,
            .maximum = 2.0F,
            .count = 1U,
            .valid = true,
        }};
        cloud.scalarFieldValues = {fieldValue};

        PointCloudStyleState style;
        style.colorMode = PointCloudColorMode::SolidColor;
        style.solidColor = {0.20F, 0.40F, 0.80F, 1.0F};
        style.falloffProfile = PointCloudFalloffProfile::HardDisc;
        style.solidCenters = true;
        invisible_places::style::SetScalarConstant(&style.pointSize, 4.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
        invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);

        ResolvedTimingColouriseStack stack;
        REQUIRE(stack.effects.size() == 8U);
        stack.effectCount = static_cast<std::uint32_t>(stack.effects.size());
        auto configureEmissive = [&](std::size_t effectIndex, float level) {
            auto& effect = stack.effects[effectIndex];
            effect.enabled = enabled;
            effect.output = TimingColouriseOutput::Emissive;
            effect.source = useNormalZ
                                ? TimingColouriseSource::NormalZ
                                : TimingColouriseSource::ScalarField;
            effect.scalarFieldSlot = useNormalZ ? -1 : 0;
            effect.lowerBound = 0.5F;
            effect.upperBound = 1.5F;
            effect.edgeFadeLowerFraction = edgeFadeFraction;
            effect.edgeFadeUpperFraction = edgeFadeFraction;
            effect.emissiveLevel = level;
            // Emissive slots carry their falloff-shaped level in LUT
            // channel r; a flat profile reproduces the plain level.
            effect.rgbaLut.fill({level, 0.0F, 0.0F, 0.0F});
        };
        // Exercise the complete shared capacity by placing the first visible
        // emissive effect in the eighth slot.
        configureEmissive(stack.effects.size() - 1U, emissiveLevel);
        if (secondEmissiveLevel > 0.0F) {
            configureEmissive(stack.effects.size() - 2U, secondEmissiveLevel);
        }

        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .timingColourise = stack,
            .hasSourceRgb = false,
            .fastBasic = fastBasic,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::camera::CameraState cameraState;
        cameraState.position = {0.0F, 0.0F, 5.0F};
        cameraState.target = {0.0F, 0.0F, 0.0F};
        cameraState.nearPlane = 0.1F;
        cameraState.farPlane = 20.0F;

        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 9U, 9U);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0U, 0U, 9U, 9U},
            &image);
        return image;
    };

    constexpr std::size_t kCenter = 4U * 9U + 4U;
    for (const bool fastBasic : {false, true}) {
        const auto disabled = render(1.0F, 0.5F, fastBasic, false);
        const auto zero = render(1.0F, 0.0F, fastBasic);
        const auto outside = render(0.25F, 0.5F, fastBasic);
        REQUIRE(disabled.alpha[kCenter] > 0.0F);
        CHECK(zero.beautyR[kCenter] == Catch::Approx(disabled.beautyR[kCenter]).margin(1.0e-6F));
        CHECK(zero.beautyG[kCenter] == Catch::Approx(disabled.beautyG[kCenter]).margin(1.0e-6F));
        CHECK(zero.beautyB[kCenter] == Catch::Approx(disabled.beautyB[kCenter]).margin(1.0e-6F));
        CHECK(outside.beautyR[kCenter] == Catch::Approx(disabled.beautyR[kCenter]).margin(1.0e-6F));
        CHECK(outside.beautyG[kCenter] == Catch::Approx(disabled.beautyG[kCenter]).margin(1.0e-6F));
        CHECK(outside.beautyB[kCenter] == Catch::Approx(disabled.beautyB[kCenter]).margin(1.0e-6F));
        CHECK(zero.alpha[kCenter] == Catch::Approx(disabled.alpha[kCenter]).margin(1.0e-6F));
        CHECK(outside.alpha[kCenter] == Catch::Approx(disabled.alpha[kCenter]).margin(1.0e-6F));
    }

    const auto beautyOff = render(1.0F, 0.0F, false);
    const auto beautyInside = render(1.0F, 0.5F, false);
    const auto beautyStacked = render(1.0F, 0.5F, false, true, false, 0.25F, 0.5F);
    const auto beautyNormal = render(1.0F, 0.5F, false, true, true);
    const auto beautyDarkened = render(1.0F, -0.5F, false);
    const auto beautyHalfDarkened = render(0.625F, -0.5F, false);
    const auto beautyFullyDark = render(1.0F, -1.0F, false);
    CHECK(beautyInside.beautyR[kCenter] > beautyOff.beautyR[kCenter]);
    CHECK(beautyInside.beautyG[kCenter] > beautyOff.beautyG[kCenter]);
    CHECK(beautyInside.beautyB[kCenter] > beautyOff.beautyB[kCenter]);
    CHECK(beautyStacked.beautyR[kCenter] > beautyInside.beautyR[kCenter]);
    CHECK(beautyNormal.beautyR[kCenter] == Catch::Approx(beautyInside.beautyR[kCenter]).margin(1.0e-5F));
    CHECK(
        beautyDarkened.beautyR[kCenter] / beautyOff.beautyR[kCenter] ==
        Catch::Approx(0.5F).margin(1.0e-5F));
    CHECK(
        beautyHalfDarkened.beautyR[kCenter] / beautyOff.beautyR[kCenter] ==
        Catch::Approx(0.75F).margin(1.0e-5F));
    CHECK(beautyFullyDark.beautyR[kCenter] == Catch::Approx(0.0F).margin(1.0e-6F));
    CHECK(beautyDarkened.alpha[kCenter] == Catch::Approx(beautyOff.alpha[kCenter]).margin(1.0e-6F));

    const auto fastOff = render(1.0F, 0.0F, true);
    const auto fastInside = render(1.0F, 0.5F, true);
    const auto fastHalfFade = render(0.625F, 0.5F, true);
    const auto fastOutwardHalfFade =
        render(0.375F, 0.5F, true, true, false, -0.25F);
    const auto fastLongOutwardHalfFade =
        render(-0.5F, 0.5F, true, true, false, -2.0F);
    const auto fastCapped = render(1.0F, 2.0F, true);
    const auto fastDarkened = render(1.0F, -0.5F, true);
    const auto fastHalfDarkened = render(0.625F, -0.5F, true);
    const auto fastFullyDark = render(1.0F, -1.0F, true);
    REQUIRE(fastOff.beautyR[kCenter] > 0.0F);
    CHECK(
        fastInside.beautyR[kCenter] / fastOff.beautyR[kCenter] ==
        Catch::Approx(1.175F).margin(1.0e-5F));
    CHECK(
        fastHalfFade.beautyR[kCenter] / fastOff.beautyR[kCenter] ==
        Catch::Approx(1.0875F).margin(1.0e-5F));
    CHECK(
        fastOutwardHalfFade.beautyR[kCenter] /
            fastOff.beautyR[kCenter] ==
        Catch::Approx(1.0875F).margin(1.0e-5F));
    CHECK(
        fastLongOutwardHalfFade.beautyR[kCenter] /
            fastOff.beautyR[kCenter] ==
        Catch::Approx(1.0875F).margin(1.0e-5F));
    CHECK(
        fastCapped.beautyR[kCenter] / fastOff.beautyR[kCenter] ==
        Catch::Approx(1.35F).margin(1.0e-5F));
    CHECK(
        fastDarkened.beautyR[kCenter] / fastOff.beautyR[kCenter] ==
        Catch::Approx(0.5F).margin(1.0e-5F));
    CHECK(
        fastHalfDarkened.beautyR[kCenter] / fastOff.beautyR[kCenter] ==
        Catch::Approx(0.75F).margin(1.0e-5F));
    CHECK(fastFullyDark.beautyR[kCenter] == Catch::Approx(0.0F).margin(1.0e-6F));
    CHECK(fastInside.alpha[kCenter] == Catch::Approx(fastOff.alpha[kCenter]).margin(1.0e-6F));
    CHECK(fastDarkened.alpha[kCenter] == Catch::Approx(fastOff.alpha[kCenter]).margin(1.0e-6F));
}

TEST_CASE(
    "Offline timing dual-aspect features occupy one resolved slot per aspect",
    "[output][offline][timing-colourise][emissive][capacity]") {
    using invisible_places::renderer::pointcloud::PointCloudColorMode;
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::ResolvedTimingColouriseStack;
    using invisible_places::renderer::pointcloud::TimingColouriseOutput;
    using invisible_places::renderer::pointcloud::TimingColouriseSource;
    using invisible_places::renderer::pointcloud::kTimingColouriseMaxEffects;

    // A Visual Feature with both aspects enabled resolves into two stack
    // slots (its colourise slot before its emissive slot, sharing bounds).
    // Four active dual-aspect features therefore fill the entire renderer
    // capacity; the app-side resolver drops a fifth active dual-aspect
    // feature whole because its two-slot cost exceeds the remaining space.
    STATIC_CHECK(4U * 2U == kTimingColouriseMaxEffects);
    STATIC_CHECK(5U * 2U > kTimingColouriseMaxEffects);

    auto render = [](bool tintVisible, float emissiveLevel, bool fastBasic) {
        invisible_places::io::LoadedPointCloud cloud;
        cloud.positions = {{0.0F, 0.0F, 0.0F}};
        cloud.scalarFields = {{
            .name = "Interest",
            .minimum = 0.0F,
            .maximum = 2.0F,
            .count = 1U,
            .valid = true,
        }};
        cloud.scalarFieldValues = {1.0F};

        PointCloudStyleState style;
        style.colorMode = PointCloudColorMode::SolidColor;
        style.solidColor = {0.20F, 0.40F, 0.80F, 1.0F};
        style.falloffProfile = PointCloudFalloffProfile::HardDisc;
        style.solidCenters = true;
        invisible_places::style::SetScalarConstant(&style.pointSize, 4.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
        invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);

        ResolvedTimingColouriseStack stack;
        REQUIRE(stack.effects.size() == 8U);
        stack.effectCount = static_cast<std::uint32_t>(stack.effects.size());
        // Four dual-aspect features: slots 0/2/4/6 carry the colourise
        // aspect and slots 1/3/5/7 the paired emissive aspect with the
        // same bounds. Only the topmost pair is visible.
        for (std::size_t feature = 0U; feature < 4U; ++feature) {
            const bool topmost = feature == 3U;
            auto& colourise = stack.effects[feature * 2U];
            colourise.enabled = true;
            colourise.output = TimingColouriseOutput::Colourise;
            colourise.source = TimingColouriseSource::ScalarField;
            colourise.scalarFieldSlot = 0;
            colourise.lowerBound = 0.5F;
            colourise.upperBound = 1.5F;
            colourise.edgeFadeLowerFraction = 0.25F;
            colourise.edgeFadeUpperFraction = 0.25F;
            colourise.rgbaLut.fill(
                {1.0F, 0.0F, 0.0F, topmost && tintVisible ? 1.0F : 0.0F});
            auto& emissive = stack.effects[feature * 2U + 1U];
            emissive.enabled = true;
            emissive.output = TimingColouriseOutput::Emissive;
            emissive.source = TimingColouriseSource::ScalarField;
            emissive.scalarFieldSlot = 0;
            emissive.lowerBound = 0.5F;
            emissive.upperBound = 1.5F;
            emissive.edgeFadeLowerFraction = 0.25F;
            emissive.edgeFadeUpperFraction = 0.25F;
            emissive.emissiveLevel = topmost ? emissiveLevel : 0.0F;
            emissive.rgbaLut.fill(
                {topmost ? emissiveLevel : 0.0F, 0.0F, 0.0F, 0.0F});
        }

        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .timingColourise = stack,
            .hasSourceRgb = false,
            .fastBasic = fastBasic,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::camera::CameraState cameraState;
        cameraState.position = {0.0F, 0.0F, 5.0F};
        cameraState.target = {0.0F, 0.0F, 0.0F};
        cameraState.nearPlane = 0.1F;
        cameraState.farPlane = 20.0F;

        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 9U, 9U);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0U, 0U, 9U, 9U},
            &image);
        return image;
    };

    constexpr std::size_t kCenter = 4U * 9U + 4U;
    for (const bool fastBasic : {false, true}) {
        const auto base = render(false, 0.0F, fastBasic);
        const auto tinted = render(true, 0.0F, fastBasic);
        const auto dual = render(true, 0.5F, fastBasic);
        REQUIRE(base.alpha[kCenter] > 0.0F);
        // The colourise slot tints the point toward the LUT colour.
        CHECK(tinted.beautyR[kCenter] > base.beautyR[kCenter]);
        CHECK(tinted.beautyR[kCenter] > tinted.beautyB[kCenter]);
        // The paired emissive slot lifts the tinted result further.
        CHECK(dual.beautyR[kCenter] > tinted.beautyR[kCenter]);
        // Neither aspect ever changes point opacity.
        CHECK(tinted.alpha[kCenter] ==
              Catch::Approx(base.alpha[kCenter]).margin(1.0e-6F));
        CHECK(dual.alpha[kCenter] ==
              Catch::Approx(base.alpha[kCenter]).margin(1.0e-6F));
    }
}

TEST_CASE(
    "Soft-edge depth prepass rejects deep points but preserves its tolerance band",
    "[output][offline][pointcloud][depth-prepass]") {
    auto render = [](bool depthPrepassEnabled,
                     float toleranceMeters,
                     float depthWeightStrength) {
        invisible_places::io::LoadedPointCloud cloud;
        cloud.positions = {
            {0.0F, 0.0F, 0.0F},
            {0.0F, 0.0F, -5.0F},
        };
        cloud.packedColors = {
            0xFF0000FFU,
            0xFFFF0000U,
        };
        cloud.hasSourceRgb = true;

        invisible_places::renderer::pointcloud::PointCloudStyleState style;
        style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
        style.falloffProfile =
            invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian;
        style.gaussianSharpness = 4.0F;
        style.depthPrepassEnabled = depthPrepassEnabled;
        style.depthPrepassAlphaThreshold = 0.35F;
        style.depthPrepassToleranceMeters = toleranceMeters;
        style.depthWeightStrength = depthWeightStrength;
        invisible_places::style::SetScalarConstant(&style.pointSize, 9.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 0.8F);

        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .hasSourceRgb = true,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::camera::CameraState cameraState;
        cameraState.position = {0.0F, 0.0F, 5.0F};
        cameraState.target = {0.0F, 0.0F, 0.0F};
        cameraState.nearPlane = 0.1F;
        cameraState.farPlane = 1000.0F;

        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 17U, 17U);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0U, 0U, 17U, 17U},
            &image);
        return image;
    };

    constexpr std::size_t kCenter = 8U * 17U + 8U;
    const auto original = render(false, 0.0F, 1.0F);
    const auto culled = render(true, 0.05F, 1.0F);
    const auto tolerated = render(true, 5.01F, 1.0F);
    const auto strongerWeight = render(false, 0.0F, 8.0F);

    REQUIRE(original.beautyR[kCenter] > 0.0F);
    REQUIRE(original.beautyB[kCenter] > 0.0F);
    CHECK(culled.beautyR[kCenter] > 0.0F);
    CHECK(culled.beautyB[kCenter] == Catch::Approx(0.0F).margin(1.0e-6F));
    CHECK(tolerated.beautyR[kCenter] ==
          Catch::Approx(original.beautyR[kCenter]).margin(1.0e-5F));
    CHECK(tolerated.beautyB[kCenter] ==
          Catch::Approx(original.beautyB[kCenter]).margin(1.0e-5F));

    const float originalFrontToBack =
        original.beautyR[kCenter] / original.beautyB[kCenter];
    const float strongerFrontToBack =
        strongerWeight.beautyR[kCenter] / strongerWeight.beautyB[kCenter];
    CHECK(strongerFrontToBack > originalFrontToBack * 1.5F);
}
