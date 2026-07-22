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
    for (std::size_t fieldIndex = 0; fieldIndex < 31U; ++fieldIndex) {
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

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
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
    CHECK(underCoveredFiveMillimeter.footprintScale == Catch::Approx(5.0F));
    CHECK(underCoveredFiveMillimeter.coverageCorrection == Catch::Approx(2.0F));

    const auto clamped = ResolvePointCloudDensityCompensation(0.005F, 1U, 0.001F, 1'000'000U);
    CHECK(clamped.coverageCorrection == Catch::Approx(16.0F));

    const auto missingCounts = ResolvePointCloudDensityCompensation(0.005F, 0U, 0.001F, 1000U);
    CHECK(missingCounts.footprintScale == Catch::Approx(5.0F));
    CHECK(missingCounts.coverageCorrection == Catch::Approx(1.0F));
}

TEST_CASE("Water Flow activity scales are deterministic and monotonic", "[pointcloud][water][activity]") {
    using invisible_places::renderer::pointcloud::ResolveWaterFlowActivityScales;

    const auto off = ResolveWaterFlowActivityScales(0.0F, 0.0F);
    CHECK(off.activity == 0.0F);
    CHECK(off.trailVisibility == 0.0F);
    CHECK(off.appearance == Catch::Approx(0.30F));
    CHECK(off.width == Catch::Approx(0.65F));
    CHECK(off.speed == Catch::Approx(0.60F));
    CHECK(off.visibleLength == Catch::Approx(0.55F));
    CHECK(off.lateralMotion == 0.0F);

    const auto full = ResolveWaterFlowActivityScales(1.0F, 1.0F);
    CHECK(full.trailVisibility == 1.0F);
    CHECK(full.appearance == 1.0F);
    CHECK(full.width == 1.0F);
    CHECK(full.speed == 1.0F);
    CHECK(full.visibleLength == 1.0F);
    CHECK(full.lateralMotion == Catch::Approx(0.15F));

    const auto earlyTrail = ResolveWaterFlowActivityScales(0.25F, 0.10F);
    const auto lateTrail = ResolveWaterFlowActivityScales(0.25F, 0.80F);
    CHECK(earlyTrail.trailVisibility == 1.0F);
    CHECK(lateTrail.trailVisibility == 0.0F);
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
    CHECK(Maximum(late.alpha) == 0.0F);
    CHECK(Maximum(early.alpha) > 0.0F);
    CHECK(Maximum(full.alpha) > Maximum(early.alpha));

    const auto fastOff = RenderSingleWaterTrail(0.0F, 0.1F, true);
    const auto fastFull = RenderSingleWaterTrail(1.0F, 0.1F, true);
    CHECK(Maximum(fastOff.alpha) == 0.0F);
    CHECK(Maximum(fastFull.alpha) > 0.0F);
}

TEST_CASE("Coverage correction forces the unified transparent material", "[pointcloud][density][material]") {
    using invisible_places::renderer::pointcloud::PointCloudDensityCompensation;
    using invisible_places::renderer::pointcloud::PointCloudMaterialVariant;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::ResolvePointCloudMaterialVariant;

    const PointCloudStyleState style;
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::OpaqueHardDisc);
    CHECK(
        ResolvePointCloudMaterialVariant(style, PointCloudDensityCompensation{5.0F, 1.0F}) ==
        PointCloudMaterialVariant::OpaqueHardDisc);
    CHECK(
        ResolvePointCloudMaterialVariant(style, PointCloudDensityCompensation{5.0F, 1.01F}) ==
        PointCloudMaterialVariant::Unified);
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
    using invisible_places::renderer::pointcloud::PointCloudDensityCompensation;

    const auto fine =
        RenderSinglePoint(0.0F, 0.0F, PointCloudDensityCompensation{1.0F, 16.0F}, nullptr, true);
    const auto sparse =
        RenderSinglePoint(0.0F, 8.0F, PointCloudDensityCompensation{5.0F, 1.0F / 16.0F}, nullptr, true);
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
