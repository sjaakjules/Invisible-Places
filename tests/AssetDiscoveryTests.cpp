#include "InvisiblePlacesBuildConfig.hpp"
#include "io/AssetDiscovery.hpp"
#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
#include "io/PlyHeader.hpp"
#include "io/TransformMatrix.hpp"
#include "platform/WindowTitle.hpp"
#include "platform/VulkanRuntimeConfig.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/gsplat/HighQualityGaussianScene.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "serialization/ProjectDocument.hpp"
#include "style/RenderParameterBinding.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

std::filesystem::path DataRoot() {
    return std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
}

template <typename T>
void AppendBinary(std::vector<std::byte>* bytes, const T& value) {
    const auto* begin = reinterpret_cast<const std::byte*>(&value);
    bytes->insert(bytes->end(), begin, begin + sizeof(T));
}

std::filesystem::path WriteTinyBinaryPointCloudFixture() {
    const auto fixturePath = std::filesystem::temp_directory_path() / "invisible_places_point_fixture.ply";
    std::ofstream output{fixturePath, std::ios::binary | std::ios::trunc};
    if (!output.is_open()) {
        throw std::runtime_error{"Failed to create temporary PLY fixture."};
    }

    output << "ply\n";
    output << "format binary_little_endian 1.0\n";
    output << "element vertex 3\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "property uchar red\n";
    output << "property uchar green\n";
    output << "property uchar blue\n";
    output << "property float scalar_Temperature\n";
    output << "property double scalar_Density\n";
    output << "end_header\n";

    std::vector<std::byte> bytes;
    bytes.reserve(3 * (sizeof(float) * 4 + sizeof(double) + (sizeof(std::uint8_t) * 3)));

    const struct {
        float x;
        float y;
        float z;
        std::uint8_t r;
        std::uint8_t g;
        std::uint8_t b;
        float temperature;
        double density;
    } points[] = {
        {0.0F, 1.0F, 2.0F, 255, 10, 20, 1.5F, 10.0},
        {-1.0F, 0.5F, 4.0F, 12, 34, 56, 2.5F, 20.0},
        {3.0F, -2.0F, 1.0F, 90, 120, 150, -4.0F, 5.0},
    };

    for (const auto& point : points) {
        AppendBinary(&bytes, point.x);
        AppendBinary(&bytes, point.y);
        AppendBinary(&bytes, point.z);
        AppendBinary(&bytes, point.r);
        AppendBinary(&bytes, point.g);
        AppendBinary(&bytes, point.b);
        AppendBinary(&bytes, point.temperature);
        AppendBinary(&bytes, point.density);
    }

    output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    output.close();
    return fixturePath;
}

std::pair<std::filesystem::path, std::filesystem::path> WriteTinyGaussianSplatFixture() {
    const auto plyPath = std::filesystem::temp_directory_path() / "invisible_places_gsplat_fixture.ply";
    const auto matrixPath = std::filesystem::temp_directory_path() / "invisible_places_gsplat_fixture.txt";

    std::ofstream plyOutput{plyPath, std::ios::binary | std::ios::trunc};
    if (!plyOutput.is_open()) {
        throw std::runtime_error{"Failed to create temporary Gaussian splat PLY fixture."};
    }

    plyOutput << "ply\n";
    plyOutput << "format binary_little_endian 1.0\n";
    plyOutput << "element vertex 3\n";
    plyOutput << "property float x\n";
    plyOutput << "property float y\n";
    plyOutput << "property float z\n";
    plyOutput << "property float nx\n";
    plyOutput << "property float ny\n";
    plyOutput << "property float nz\n";
    plyOutput << "property float f_dc_0\n";
    plyOutput << "property float f_dc_1\n";
    plyOutput << "property float f_dc_2\n";
    for (int index = 0; index < 45; ++index) {
        plyOutput << "property float f_rest_" << index << "\n";
    }
    plyOutput << "property float opacity\n";
    plyOutput << "property float scale_0\n";
    plyOutput << "property float scale_1\n";
    plyOutput << "property float scale_2\n";
    plyOutput << "property float rot_0\n";
    plyOutput << "property float rot_1\n";
    plyOutput << "property float rot_2\n";
    plyOutput << "property float rot_3\n";
    plyOutput << "end_header\n";

    std::vector<std::byte> bytes;
    bytes.reserve(3 * (6 + 3 + 45 + 1 + 3 + 4) * sizeof(float));

    struct TinySplat {
        float x;
        float y;
        float z;
        float nx;
        float ny;
        float nz;
        float dc[3];
        float rest[45];
        float opacity;
        float scale[3];
        float rotation[4];
    };

    TinySplat splats[3]{};
    splats[0].x = 1.0F;
    splats[0].y = 2.0F;
    splats[0].z = 3.0F;
    splats[0].dc[0] = 0.10F;
    splats[0].dc[1] = 0.20F;
    splats[0].dc[2] = 0.30F;
    splats[0].rest[0] = 0.40F;
    splats[0].rest[1] = 0.50F;
    splats[0].rest[2] = 0.60F;
    splats[0].opacity = 0.0F;
    splats[0].scale[0] = std::log(2.0F);
    splats[0].scale[1] = std::log(3.0F);
    splats[0].scale[2] = std::log(4.0F);
    splats[0].rotation[0] = 2.0F;

    splats[1].x = -1.0F;
    splats[1].y = 0.0F;
    splats[1].z = 1.0F;
    splats[1].dc[0] = -0.25F;
    splats[1].dc[1] = 0.10F;
    splats[1].dc[2] = 0.50F;
    splats[1].opacity = 2.0F;
    splats[1].scale[0] = 0.0F;
    splats[1].scale[1] = std::log(0.5F);
    splats[1].scale[2] = std::log(2.0F);
    splats[1].rotation[1] = 1.0F;

    splats[2].x = 0.0F;
    splats[2].y = 1.0F;
    splats[2].z = 2.0F;
    splats[2].dc[0] = 0.05F;
    splats[2].dc[1] = 0.15F;
    splats[2].dc[2] = 0.25F;
    splats[2].opacity = -2.0F;
    splats[2].scale[0] = std::log(1.5F);
    splats[2].scale[1] = std::log(1.0F);
    splats[2].scale[2] = std::log(0.75F);
    splats[2].rotation[2] = 1.0F;

    for (const auto& splat : splats) {
        AppendBinary(&bytes, splat.x);
        AppendBinary(&bytes, splat.y);
        AppendBinary(&bytes, splat.z);
        AppendBinary(&bytes, splat.nx);
        AppendBinary(&bytes, splat.ny);
        AppendBinary(&bytes, splat.nz);
        for (const auto value : splat.dc) {
            AppendBinary(&bytes, value);
        }
        for (const auto value : splat.rest) {
            AppendBinary(&bytes, value);
        }
        AppendBinary(&bytes, splat.opacity);
        for (const auto value : splat.scale) {
            AppendBinary(&bytes, value);
        }
        for (const auto value : splat.rotation) {
            AppendBinary(&bytes, value);
        }
    }

    plyOutput.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    plyOutput.close();

    std::ofstream matrixOutput{matrixPath, std::ios::trunc};
    if (!matrixOutput.is_open()) {
        throw std::runtime_error{"Failed to create temporary Gaussian splat matrix fixture."};
    }
    matrixOutput << "2 0 0 10\n";
    matrixOutput << "0 3 0 -5\n";
    matrixOutput << "0 0 4 2\n";
    matrixOutput << "0 0 0 1\n";
    matrixOutput.close();

    return {plyPath, matrixPath};
}

}  // namespace

TEST_CASE("CloudCompare point clouds expose RGB and scalar fields", "[ply][pointcloud]") {
    const auto result = invisible_places::io::ParsePlyHeader(DataRoot() / "Site2 -5mm.ply");

    REQUIRE(result.success);
    CHECK(result.header.LooksLikePointCloud());
    CHECK(result.header.HasColorRgb());

    const auto scalarFields = result.header.ScalarFieldNames();
    CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Height") != scalarFields.end());
    CHECK(std::find(scalarFields.begin(), scalarFields.end(), "GroundOffset") != scalarFields.end());
}

TEST_CASE("Gaussian splat assets expose the expected transform pairing", "[ply][gsplat]") {
    const auto headerResult = invisible_places::io::ParsePlyHeader(DataRoot() / "gSplat-Site3-1.ply");
    const auto matrixResult = invisible_places::io::ParseMatrix4x4(DataRoot() / "gSplat-Site3-1.txt");

    REQUIRE(headerResult.success);
    REQUIRE(matrixResult.success);

    CHECK(headerResult.header.LooksLikeGaussianSplat());
    CHECK(headerResult.header.vertexCount > 1000000);
    CHECK(matrixResult.matrix.At(3, 3) == Catch::Approx(1.0));
}

TEST_CASE("Data discovery finds both point clouds and gaussian splats", "[discovery]") {
    const auto catalog = invisible_places::io::DiscoverAssets(DataRoot());

    CHECK(catalog.pointClouds.size() >= 5);
    CHECK(catalog.gaussianSplats.size() >= 8);
    CHECK(catalog.issues.empty());
}

TEST_CASE("Bootstrap window title summarizes discovered layer counts", "[window][discovery]") {
    const auto catalog = invisible_places::io::DiscoverAssets(DataRoot());
    const auto title = invisible_places::platform::MakeBootstrapWindowTitle(catalog);

    CHECK(title == "Invisible Places | 6 point clouds | 10 gSplats");
}

TEST_CASE("Vulkan runtime description includes explicit ICD when present", "[vulkan][runtime]") {
    invisible_places::platform::VulkanRuntimeConfig config;
    config.injectedMoltenVkIcd = true;
    config.explicitIcdPath = "/tmp/MoltenVK_icd.json";

    const auto description = invisible_places::platform::DescribeVulkanRuntime(config);
    CHECK(description == "Vulkan runtime | ICD: /tmp/MoltenVK_icd.json (auto)");
}

TEST_CASE("Binary point cloud loader parses payload, colors, bounds, and scalar stats", "[ply][loader]") {
    const auto fixturePath = WriteTinyBinaryPointCloudFixture();
    const auto result = invisible_places::io::LoadPointCloud(fixturePath);

    REQUIRE(result.success);
    CHECK(result.cloud.PointCount() == 3);
    CHECK(result.cloud.ScalarFieldCount() == 2);
    CHECK(result.cloud.hasSourceRgb);

    CHECK(result.cloud.bounds.minimum.x == Catch::Approx(-1.0F));
    CHECK(result.cloud.bounds.minimum.y == Catch::Approx(-2.0F));
    CHECK(result.cloud.bounds.maximum.z == Catch::Approx(4.0F));
    CHECK(result.cloud.hasFocusPoint);
    CHECK(result.cloud.focusPoint.x == Catch::Approx(0.0F));
    CHECK(result.cloud.focusPoint.y == Catch::Approx(1.0F));
    CHECK(result.cloud.focusPoint.z == Catch::Approx(2.0F));

    CHECK(result.cloud.scalarFields[0].name == "Temperature");
    CHECK(result.cloud.scalarFields[0].minimum == Catch::Approx(-4.0F));
    CHECK(result.cloud.scalarFields[0].maximum == Catch::Approx(2.5F));
    CHECK(result.cloud.scalarFields[1].name == "Density");
    CHECK(result.cloud.scalarFields[1].minimum == Catch::Approx(5.0F));
    CHECK(result.cloud.scalarFields[1].maximum == Catch::Approx(20.0F));
    CHECK(result.cloud.scalarFieldValues[result.cloud.ScalarFieldValueIndex(0, 1)] == Catch::Approx(2.5F));
    CHECK(result.cloud.scalarFieldValues[result.cloud.ScalarFieldValueIndex(1, 2)] == Catch::Approx(5.0F));

    std::filesystem::remove(fixturePath);
}

TEST_CASE("Binary gaussian splat loader parses payload, transform, and decoded parameters", "[ply][gsplat][loader]") {
    const auto [plyPath, matrixPath] = WriteTinyGaussianSplatFixture();
    const auto result = invisible_places::io::LoadGaussianSplat(plyPath, matrixPath);

    REQUIRE(result.success);
    CHECK(result.splats.SplatCount() == 3);
    CHECK(result.splats.layerName == "invisible_places_gsplat_fixture");
    CHECK(result.splats.localBounds.valid);
    CHECK(result.splats.bounds.valid);
    CHECK(result.splats.hasLocalFocusPoint);
    CHECK(result.splats.hasFocusPoint);

    CHECK(result.splats.localBounds.minimum.x == Catch::Approx(-1.0F));
    CHECK(result.splats.localBounds.minimum.y == Catch::Approx(0.0F));
    CHECK(result.splats.localBounds.minimum.z == Catch::Approx(1.0F));
    CHECK(result.splats.localBounds.maximum.x == Catch::Approx(1.0F));
    CHECK(result.splats.localBounds.maximum.y == Catch::Approx(2.0F));
    CHECK(result.splats.localBounds.maximum.z == Catch::Approx(3.0F));
    CHECK(result.splats.localFocusPoint.x == Catch::Approx(0.0F));
    CHECK(result.splats.localFocusPoint.y == Catch::Approx(1.0F));
    CHECK(result.splats.localFocusPoint.z == Catch::Approx(2.0F));

    CHECK(result.splats.bounds.minimum.x == Catch::Approx(8.0F));
    CHECK(result.splats.bounds.minimum.y == Catch::Approx(-5.0F));
    CHECK(result.splats.bounds.minimum.z == Catch::Approx(6.0F));
    CHECK(result.splats.bounds.maximum.x == Catch::Approx(12.0F));
    CHECK(result.splats.bounds.maximum.y == Catch::Approx(1.0F));
    CHECK(result.splats.bounds.maximum.z == Catch::Approx(14.0F));
    CHECK(result.splats.focusPoint.x == Catch::Approx(10.0F));
    CHECK(result.splats.focusPoint.y == Catch::Approx(-2.0F));
    CHECK(result.splats.focusPoint.z == Catch::Approx(10.0F));

    CHECK(result.splats.scales[0][0] == Catch::Approx(2.0F));
    CHECK(result.splats.scales[0][1] == Catch::Approx(3.0F));
    CHECK(result.splats.scales[0][2] == Catch::Approx(4.0F));
    CHECK(result.splats.rotations[0][0] == Catch::Approx(1.0F));
    CHECK(result.splats.rotations[0][1] == Catch::Approx(0.0F));
    CHECK(result.splats.opacities[0] == Catch::Approx(0.5F));
    CHECK(result.splats.opacities[1] == Catch::Approx(1.0F / (1.0F + std::exp(-2.0F))));
    CHECK(result.splats.opacities[2] == Catch::Approx(1.0F / (1.0F + std::exp(2.0F))));

    const auto shOffset = result.splats.ShCoefficientOffset(0);
    CHECK(result.splats.shCoefficients[shOffset + 0] == Catch::Approx(0.10F));
    CHECK(result.splats.shCoefficients[shOffset + 1] == Catch::Approx(0.20F));
    CHECK(result.splats.shCoefficients[shOffset + 2] == Catch::Approx(0.30F));
    CHECK(result.splats.shCoefficients[shOffset + 3] == Catch::Approx(0.40F));
    CHECK(result.splats.shCoefficients[shOffset + 4] == Catch::Approx(0.50F));
    CHECK(result.splats.shCoefficients[shOffset + 5] == Catch::Approx(0.60F));

    std::filesystem::remove(plyPath);
    std::filesystem::remove(matrixPath);
}

TEST_CASE("Point budget sampling is deterministic and avoids first-N ordering", "[budget][sampling]") {
    const auto first = invisible_places::renderer::pointcloud::GenerateDeterministicSampleIndices(100, 10);
    const auto second = invisible_places::renderer::pointcloud::GenerateDeterministicSampleIndices(100, 10);
    const auto budget = invisible_places::renderer::pointcloud::MakePointBudgetState(100, 10);
    const auto full = invisible_places::renderer::pointcloud::MakePointBudgetState(100, 100);

    REQUIRE(first.size() == 10);
    CHECK(first == second);
    CHECK(first[0] != 0U);
    CHECK(budget.activePoints == 10);
    CHECK(budget.activeFraction == Catch::Approx(0.1F));
    CHECK(budget.UsesSampledIndices());
    CHECK(!full.UsesSampledIndices());
}

TEST_CASE("Scalar field binding evaluation matches the mapped style rules", "[style][binding]") {
    invisible_places::io::ScalarFieldStats stats;
    stats.name = "Height";
    stats.Include(0.0F);
    stats.Include(10.0F);

    invisible_places::style::RenderParameterBinding constantBinding;
    invisible_places::style::SetScalarConstant(&constantBinding, 2.5F);
    CHECK(invisible_places::style::EvaluateScalarBinding(constantBinding, 100.0F, &stats) == Catch::Approx(2.5F));

    invisible_places::style::RenderParameterBinding mappedBinding;
    invisible_places::style::ConfigureFieldMapFromStats(&mappedBinding, 0, "Height", 0.0F, 1.0F, &stats);
    mappedBinding.fieldMap.gamma = 2.0F;
    CHECK(invisible_places::style::EvaluateScalarBinding(mappedBinding, 5.0F, &stats) == Catch::Approx(0.25F));

    invisible_places::style::SetFieldMapFlag(
        &mappedBinding.fieldMap,
        invisible_places::style::FieldMapFlagInvert,
        true);
    CHECK(invisible_places::style::EvaluateScalarBinding(mappedBinding, 2.5F, &stats) == Catch::Approx(0.5625F));

    invisible_places::style::SetFieldMapFlag(
        &mappedBinding.fieldMap,
        invisible_places::style::FieldMapFlagClamp,
        false);
    invisible_places::style::SetFieldMapFlag(
        &mappedBinding.fieldMap,
        invisible_places::style::FieldMapFlagInvert,
        false);
    mappedBinding.fieldMap.gamma = 1.0F;
    mappedBinding.fieldMap.outputMin = -1.0F;
    mappedBinding.fieldMap.outputMax = 1.0F;
    CHECK(invisible_places::style::EvaluateScalarBinding(mappedBinding, 15.0F, &stats) == Catch::Approx(2.0F));
}

TEST_CASE("Project document round-trips binding-backed point-cloud styles", "[serialization][project]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_roundtrip.json";

    invisible_places::serialization::ProjectDocument document;
    document.projectName = "Roundtrip";
    document.selectedLayerPath = "Data/Site2 -5mm.ply";
    document.backgroundColor = {0.02F, 0.04F, 0.08F, 1.0F};
    document.sidePanelPinned = true;
    document.autoLowerGsplatQualityWhileNavigating = false;

    invisible_places::serialization::ProjectLayerDocument layer;
    layer.kind = invisible_places::serialization::SerializedLayerKind::PointCloud;
    layer.sourcePath = "Data/Site2 -5mm.ply";
    layer.loaded = true;
    layer.visible = true;
    layer.pointBudgetActivePoints = 2048;

    invisible_places::renderer::pointcloud::PointCloudStyleState pointStyle;
    pointStyle.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap;
    pointStyle.colormap = invisible_places::renderer::pointcloud::PointCloudColormapId::Turbo;
    invisible_places::style::ConfigureFieldMapFromStats(
        &pointStyle.pointSize,
        2,
        "Height",
        1.0F,
        8.0F,
        nullptr);
    invisible_places::style::SetFieldMapFlag(
        &pointStyle.pointSize.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    pointStyle.pointSize.fieldMap.inputMin = -2.0F;
    pointStyle.pointSize.fieldMap.inputMax = 5.0F;
    invisible_places::style::SetScalarConstant(&pointStyle.opacity, 0.55F);
    invisible_places::style::ConfigureFieldMapFromStats(
        &pointStyle.colormapPosition,
        1,
        "Intensity",
        0.0F,
        1.0F,
        nullptr);
    layer.pointStyle = pointStyle;
    document.layers.push_back(layer);

    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, outputPath, &errorMessage));

    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    REQUIRE(loadedDocument->layers.size() == 1);

    CHECK(loadedDocument->projectName == "Roundtrip");
    CHECK(loadedDocument->sidePanelPinned);
    CHECK(!loadedDocument->autoLowerGsplatQualityWhileNavigating);
    CHECK(loadedDocument->backgroundColor[2] == Catch::Approx(0.08F));
    CHECK(loadedDocument->selectedLayerPath == std::filesystem::path{"Data/Site2 -5mm.ply"});

    const auto& loadedLayer = loadedDocument->layers.front();
    REQUIRE(loadedLayer.pointStyle.has_value());
    CHECK(loadedLayer.loaded);
    CHECK(loadedLayer.visible);
    CHECK(loadedLayer.pointBudgetActivePoints == 2048);
    CHECK(
        loadedLayer.pointStyle->colorMode ==
        invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap);
    CHECK(
        loadedLayer.pointStyle->colormap ==
        invisible_places::renderer::pointcloud::PointCloudColormapId::Turbo);
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.fieldSlot == 2);
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.fieldName == "Height");
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.inputMin == Catch::Approx(-2.0F));
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.outputMax == Catch::Approx(8.0F));
    CHECK(invisible_places::style::ScalarConstant(loadedLayer.pointStyle->opacity) == Catch::Approx(0.55F));
    CHECK(loadedLayer.pointStyle->colormapPosition.fieldMap.fieldName == "Intensity");

    std::filesystem::remove(outputPath);
}

TEST_CASE("gSplat quality resolver steps down during navigation and restores afterward", "[gsplat][quality]") {
    using invisible_places::renderer::gsplat::GaussianSplatQualityMode;
    using invisible_places::renderer::gsplat::ResolveEffectiveGaussianSplatQualityMode;

    CHECK(
        ResolveEffectiveGaussianSplatQualityMode(
            GaussianSplatQualityMode::Fast,
            true,
            true) == GaussianSplatQualityMode::Fast);
    CHECK(
        ResolveEffectiveGaussianSplatQualityMode(
            GaussianSplatQualityMode::Medium,
            true,
            true) == GaussianSplatQualityMode::Fast);
    CHECK(
        ResolveEffectiveGaussianSplatQualityMode(
            GaussianSplatQualityMode::SurfaceGuided,
            true,
            true) == GaussianSplatQualityMode::Medium);
    CHECK(
        ResolveEffectiveGaussianSplatQualityMode(
            GaussianSplatQualityMode::High,
            true,
            true) == GaussianSplatQualityMode::SurfaceGuided);
    CHECK(
        ResolveEffectiveGaussianSplatQualityMode(
            GaussianSplatQualityMode::High,
            false,
            true) == GaussianSplatQualityMode::High);
    CHECK(
        ResolveEffectiveGaussianSplatQualityMode(
            GaussianSplatQualityMode::Medium,
            true,
            false) == GaussianSplatQualityMode::Medium);
}

TEST_CASE("High-quality gaussian scene helpers build stable signatures and ranges", "[gsplat][hq]") {
    using invisible_places::renderer::gsplat::BuildHighQualityGaussianLayerRanges;
    using invisible_places::renderer::gsplat::BuildHighQualityGaussianLayerSignatures;
    using invisible_places::renderer::gsplat::HighQualityGaussianLayerInput;
    using invisible_places::renderer::gsplat::HighQualityGaussianLayerSignaturesMatch;

    HighQualityGaussianLayerInput firstLayer;
    firstLayer.layerId = 7;
    firstLayer.revision = 2;
    firstLayer.splatCount = 3;

    HighQualityGaussianLayerInput secondLayer;
    secondLayer.layerId = 11;
    secondLayer.revision = 9;
    secondLayer.splatCount = 5;

    std::vector<HighQualityGaussianLayerInput> inputs = {firstLayer, secondLayer};

    const auto signatures = BuildHighQualityGaussianLayerSignatures(inputs);
    const auto ranges = BuildHighQualityGaussianLayerRanges(inputs);

    REQUIRE(signatures.size() == 2);
    CHECK(signatures[0].layerId == 7);
    CHECK(signatures[1].revision == 9);
    CHECK(HighQualityGaussianLayerSignaturesMatch(signatures, signatures));

    REQUIRE(ranges.size() == 2);
    CHECK(ranges[0].styleIndex == 0);
    CHECK(ranges[0].mergedStart == 0);
    CHECK(ranges[0].splatCount == 3);
    CHECK(ranges[1].styleIndex == 1);
    CHECK(ranges[1].mergedStart == 3);
    CHECK(ranges[1].splatCount == 5);

    inputs[1].revision = 10;
    const auto changedSignatures = BuildHighQualityGaussianLayerSignatures(inputs);
    CHECK(!HighQualityGaussianLayerSignaturesMatch(signatures, changedSignatures));
}

TEST_CASE("High-quality gaussian sort orders merged splats back-to-front", "[gsplat][hq][sorting]") {
    using invisible_places::renderer::gsplat::BuildHighQualityGaussianLayerRanges;
    using invisible_places::renderer::gsplat::HighQualityGaussianLayerInput;
    using invisible_places::renderer::gsplat::SortHighQualityGaussianIndices;

    HighQualityGaussianLayerInput firstLayer;
    firstLayer.layerId = 1;
    firstLayer.revision = 1;
    firstLayer.splatCount = 2;
    firstLayer.localToWorld = glm::mat4{1.0F};
    firstLayer.transformEnabled = true;

    HighQualityGaussianLayerInput secondLayer;
    secondLayer.layerId = 2;
    secondLayer.revision = 4;
    secondLayer.splatCount = 2;
    secondLayer.localToWorld = glm::translate(glm::mat4{1.0F}, glm::vec3{0.0F, 0.0F, -4.0F});
    secondLayer.transformEnabled = true;

    std::vector<HighQualityGaussianLayerInput> inputs = {firstLayer, secondLayer};
    const auto ranges = BuildHighQualityGaussianLayerRanges(inputs);
    const std::vector<invisible_places::io::Float3> mergedCenters = {
        {0.0F, 0.0F, -2.0F},
        {0.0F, 0.0F, -5.0F},
        {0.0F, 0.0F, -2.0F},
        {0.0F, 0.0F, -3.0F},
    };

    const auto sorted = SortHighQualityGaussianIndices(mergedCenters, inputs, ranges, glm::mat4{1.0F});
    const auto sortedAgain = SortHighQualityGaussianIndices(mergedCenters, inputs, ranges, glm::mat4{1.0F});

    REQUIRE(sorted.size() == mergedCenters.size());
    CHECK(sorted == sortedAgain);
    CHECK(sorted[0] == 3U);
    CHECK(sorted[1] == 2U);
    CHECK(sorted[2] == 1U);
    CHECK(sorted[3] == 0U);
}
