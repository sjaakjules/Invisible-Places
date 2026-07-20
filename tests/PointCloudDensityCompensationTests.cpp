#include "camera/CameraState.hpp"
#include "io/PointCloudData.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <cmath>
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

std::size_t CoveredPixelCount(const invisible_places::output::ExrImage& image) {
    return static_cast<std::size_t>(std::count_if(
        image.alpha.begin(),
        image.alpha.end(),
        [](float alpha) { return alpha > 1.0e-5F; }));
}

float Maximum(const std::vector<float>& values) {
    return values.empty() ? 0.0F : *std::max_element(values.begin(), values.end());
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
