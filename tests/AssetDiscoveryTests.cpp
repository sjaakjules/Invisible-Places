#include "InvisiblePlacesBuildConfig.hpp"
#include "camera/AnimationPath.hpp"
#include "camera/CameraPath.hpp"
#include "camera/CameraShot.hpp"
#include "camera/OrbitCamera.hpp"
#include "io/AssetDiscovery.hpp"
#include "io/GaussianSplatData.hpp"
#include "io/PointCloudData.hpp"
#include "io/PlyHeader.hpp"
#include "io/TransformMatrix.hpp"
#include "output/ExrWriter.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "output/RenderPreset.hpp"
#include "output/VideoWriter.hpp"
#include "platform/WindowTitle.hpp"
#include "platform/VulkanRuntimeConfig.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/gsplat/HighQualityGaussianScene.hpp"
#include "renderer/pointcloud/Colormap.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "serialization/ProjectDocument.hpp"
#include "style/RenderParameterBinding.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

std::filesystem::path DataRoot() {
    return std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
}

void WriteLookAtOrientation(invisible_places::camera::CameraState* state) {
    if (state == nullptr) {
        return;
    }

    const glm::vec3 position{state->position[0], state->position[1], state->position[2]};
    const glm::vec3 target{state->target[0], state->target[1], state->target[2]};
    if (glm::length(target - position) <= 1.0e-5F) {
        return;
    }

    const auto view = glm::lookAtRH(position, target, glm::vec3{0.0F, 0.0F, 1.0F});
    const auto cameraToWorld = glm::inverse(glm::mat3{view});
    const auto orientation = glm::normalize(glm::quat_cast(cameraToWorld));
    state->orientation = {orientation.x, orientation.y, orientation.z, orientation.w};
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

std::filesystem::path WriteTinyBinaryPointCloudNormalFixture(bool longNames) {
    const auto fixturePath = std::filesystem::temp_directory_path() /
                             (longNames ? "invisible_places_point_normals_long_fixture.ply"
                                        : "invisible_places_point_normals_short_fixture.ply");
    std::ofstream output{fixturePath, std::ios::binary | std::ios::trunc};
    if (!output.is_open()) {
        throw std::runtime_error{"Failed to create temporary normal PLY fixture."};
    }

    output << "ply\n";
    output << "format binary_little_endian 1.0\n";
    output << "element vertex 3\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "property float " << (longNames ? "normal_x" : "nx") << "\n";
    output << "property float " << (longNames ? "normal_y" : "ny") << "\n";
    output << "property float " << (longNames ? "normal_z" : "nz") << "\n";
    output << "end_header\n";

    const struct {
        float x;
        float y;
        float z;
        float nx;
        float ny;
        float nz;
    } points[] = {
        {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 2.0F},
        {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F},
        {2.0F, 0.0F, 0.0F, 3.0F, 4.0F, 0.0F},
    };

    std::vector<std::byte> bytes;
    bytes.reserve(3 * 6 * sizeof(float));
    for (const auto& point : points) {
        AppendBinary(&bytes, point.x);
        AppendBinary(&bytes, point.y);
        AppendBinary(&bytes, point.z);
        AppendBinary(&bytes, point.nx);
        AppendBinary(&bytes, point.ny);
        AppendBinary(&bytes, point.nz);
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

    CHECK(
        title ==
        "Invisible Places | " + std::to_string(catalog.pointClouds.size()) +
            " point clouds | " + std::to_string(catalog.gaussianSplats.size()) + " gSplats");
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
    CHECK(!result.cloud.hasNormals);

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

TEST_CASE("Binary point cloud loader parses normal triplets", "[ply][loader][normals]") {
    for (const bool longNames : {true, false}) {
        const auto fixturePath = WriteTinyBinaryPointCloudNormalFixture(longNames);
        const auto result = invisible_places::io::LoadPointCloud(fixturePath);

        REQUIRE(result.success);
        REQUIRE(result.cloud.hasNormals);
        REQUIRE(result.cloud.normals.size() == 3);
        CHECK(result.cloud.normals[0].x == Catch::Approx(0.0F));
        CHECK(result.cloud.normals[0].y == Catch::Approx(0.0F));
        CHECK(result.cloud.normals[0].z == Catch::Approx(1.0F));
        CHECK(result.cloud.normals[1].x == Catch::Approx(0.0F));
        CHECK(result.cloud.normals[1].y == Catch::Approx(0.0F));
        CHECK(result.cloud.normals[1].z == Catch::Approx(0.0F));
        CHECK(result.cloud.normals[2].x == Catch::Approx(0.6F));
        CHECK(result.cloud.normals[2].y == Catch::Approx(0.8F));
        CHECK(result.cloud.normals[2].z == Catch::Approx(0.0F));

        std::filesystem::remove(fixturePath);
    }
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
    CHECK(std::is_sorted(first.begin(), first.end()));
    CHECK(first[0] != 0U);
    CHECK(budget.activePoints == 10);
    CHECK(budget.activeFraction == Catch::Approx(0.1F));
    CHECK(budget.UsesSampledIndices());
    CHECK(!full.UsesSampledIndices());
}

TEST_CASE("Surfel sampled indices encode six vertices per source point", "[budget][sampling][surfel]") {
    const std::vector<std::uint32_t> sampledPoints{2U, 7U};
    const auto encoded =
        invisible_places::renderer::pointcloud::GenerateSurfelEncodedSampleIndices(sampledPoints);

    REQUIRE(encoded.size() == 12);
    CHECK(encoded[0] == 12U);
    CHECK(encoded[5] == 17U);
    CHECK(encoded[6] == 42U);
    CHECK(encoded[11] == 47U);
}

TEST_CASE("Spatial point budget sampling preserves coverage across ordered clusters", "[budget][sampling]") {
    invisible_places::io::LoadedPointCloud cloud;
    const std::array<invisible_places::io::Float3, 4> clusterCenters = {
        invisible_places::io::Float3{0.0F, 0.0F, 0.0F},
        invisible_places::io::Float3{100.0F, 0.0F, 0.0F},
        invisible_places::io::Float3{0.0F, 100.0F, 0.0F},
        invisible_places::io::Float3{100.0F, 100.0F, 0.0F},
    };

    for (const auto& center : clusterCenters) {
        for (int offset = 0; offset < 6; ++offset) {
            invisible_places::io::Float3 point{
                center.x + static_cast<float>(offset) * 0.1F,
                center.y + static_cast<float>(offset) * 0.1F,
                center.z,
            };
            cloud.positions.push_back(point);
            cloud.bounds.Expand(point);
        }
    }

    const auto first = invisible_places::renderer::pointcloud::GenerateSpatialSampleIndices(
        cloud.positions,
        cloud.bounds,
        4);
    const auto second = invisible_places::renderer::pointcloud::GenerateSpatialSampleIndices(
        cloud.positions,
        cloud.bounds,
        4);

    REQUIRE(first.size() == 4);
    CHECK(first == second);
    CHECK(std::is_sorted(first.begin(), first.end()));
    CHECK(first != std::vector<std::uint32_t>{0U, 1U, 2U, 3U});

    std::array<bool, 4> coveredQuadrants = {false, false, false, false};
    for (const auto pointIndex : first) {
        REQUIRE(pointIndex < cloud.positions.size());
        const auto& point = cloud.positions[pointIndex];
        const auto quadrant =
            static_cast<std::size_t>((point.x >= 50.0F ? 1U : 0U) + (point.y >= 50.0F ? 2U : 0U));
        coveredQuadrants[quadrant] = true;
    }

    CHECK(std::all_of(coveredQuadrants.begin(), coveredQuadrants.end(), [](bool covered) { return covered; }));
}

TEST_CASE("Spatial point budget stratifies overfull octree candidates", "[budget][sampling]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSide = 102;
    for (int y = 0; y < gridSide; ++y) {
        for (int x = 0; x < gridSide; ++x) {
            invisible_places::io::Float3 point{
                static_cast<float>(x),
                static_cast<float>(y),
                0.0F,
            };
            cloud.positions.push_back(point);
            cloud.bounds.Expand(point);
        }
    }

    const auto first = invisible_places::renderer::pointcloud::GenerateSpatialSampleIndices(
        cloud.positions,
        cloud.bounds,
        10'000);
    const auto second = invisible_places::renderer::pointcloud::GenerateSpatialSampleIndices(
        cloud.positions,
        cloud.bounds,
        10'000);

    REQUIRE(first.size() == 10'000);
    CHECK(first == second);
    CHECK(std::is_sorted(first.begin(), first.end()));
    CHECK(std::adjacent_find(first.begin(), first.end()) == first.end());

    std::vector<std::uint32_t> firstN;
    firstN.reserve(first.size());
    for (std::uint32_t index = 0; index < first.size(); ++index) {
        firstN.push_back(index);
    }
    CHECK(first != firstN);
}

TEST_CASE("Spatial point budget keeps full-resolution draws unsampled", "[budget][sampling]") {
    invisible_places::io::LoadedPointCloud cloud;
    for (int index = 0; index < 8; ++index) {
        invisible_places::io::Float3 point{
            static_cast<float>(index),
            static_cast<float>(index % 2),
            0.0F,
        };
        cloud.positions.push_back(point);
        cloud.bounds.Expand(point);
    }

    const auto fullBudget = invisible_places::renderer::pointcloud::MakePointBudgetState(
        cloud,
        cloud.PointCount());
    CHECK(fullBudget.activePoints == cloud.PointCount());
    CHECK(!fullBudget.UsesSampledIndices());

    const auto capped = invisible_places::renderer::pointcloud::GenerateSpatialSampleIndices(
        cloud.positions,
        cloud.bounds,
        3);
    REQUIRE(capped.size() == 3);
    CHECK(fullBudget.activePoints == cloud.PointCount());
}

TEST_CASE("Orbit camera can move its pivot without changing the current view", "[camera][pivot]") {
    invisible_places::io::Bounds3f bounds;
    bounds.Expand({-1.0F, -1.0F, -1.0F});
    bounds.Expand({1.0F, 1.0F, 1.0F});

    invisible_places::camera::OrbitCamera camera;
    camera.FrameBounds(bounds, 1.0F);

    const auto beforeState = camera.CaptureState();
    const auto beforeMatrices = camera.Matrices(1.0F);
    const glm::vec3 pivot{3.0F, 2.0F, 0.5F};
    camera.SetOrbitCenterPreservingView(pivot);
    const auto afterState = camera.CaptureState();
    const auto afterMatrices = camera.Matrices(1.0F);
    const auto projectNdc = [](const invisible_places::camera::OrbitCameraMatrices& matrices, const glm::vec3& point) {
        const glm::vec4 clip = matrices.viewProjection * glm::vec4{point, 1.0F};
        return glm::vec3{clip} / clip.w;
    };

    for (std::size_t component = 0; component < 3; ++component) {
        CHECK(afterState.position[component] == Catch::Approx(beforeState.position[component]));
    }
    for (std::size_t component = 0; component < 4; ++component) {
        CHECK(afterState.orientation[component] == Catch::Approx(beforeState.orientation[component]));
    }
    CHECK(afterState.target[0] == Catch::Approx(beforeState.target[0]));
    CHECK(afterState.target[1] == Catch::Approx(beforeState.target[1]));
    CHECK(afterState.target[2] == Catch::Approx(beforeState.target[2]));
    REQUIRE(afterState.hasOrbitCenter);
    CHECK(afterState.orbitCenter[0] == Catch::Approx(pivot.x));
    CHECK(afterState.orbitCenter[1] == Catch::Approx(pivot.y));
    CHECK(afterState.orbitCenter[2] == Catch::Approx(pivot.z));
    CHECK(afterMatrices.position.x == Catch::Approx(beforeMatrices.position.x));
    CHECK(afterMatrices.position.y == Catch::Approx(beforeMatrices.position.y));
    CHECK(afterMatrices.position.z == Catch::Approx(beforeMatrices.position.z));
    CHECK(camera.OrbitCenter().x == Catch::Approx(pivot.x));
    CHECK(camera.OrbitCenter().y == Catch::Approx(pivot.y));
    CHECK(camera.OrbitCenter().z == Catch::Approx(pivot.z));

    const auto pivotNdcBeforePan = projectNdc(afterMatrices, pivot);
    camera.Pan(32.0F, -18.0F, 1024.0F, 768.0F);
    const auto pannedMatrices = camera.Matrices(1.0F);
    const auto pivotNdcAfterPan = projectNdc(pannedMatrices, pivot);
    CHECK(camera.OrbitCenter().x == Catch::Approx(pivot.x));
    CHECK(camera.OrbitCenter().y == Catch::Approx(pivot.y));
    CHECK(camera.OrbitCenter().z == Catch::Approx(pivot.z));
    CHECK(glm::length(glm::vec2{pivotNdcAfterPan - pivotNdcBeforePan}) > 0.001F);

    const auto pivotNdcBeforeDolly = projectNdc(pannedMatrices, pivot);
    const float radiusBeforeDolly = glm::length(pannedMatrices.position - pivot);
    camera.Dolly(1.0F);
    const auto dollyMatrices = camera.Matrices(1.0F);
    const auto pivotNdcAfterDolly = projectNdc(dollyMatrices, pivot);
    CHECK(pivotNdcAfterDolly.x == Catch::Approx(pivotNdcBeforeDolly.x).margin(0.0001F));
    CHECK(pivotNdcAfterDolly.y == Catch::Approx(pivotNdcBeforeDolly.y).margin(0.0001F));
    CHECK(glm::length(dollyMatrices.position - pivot) < radiusBeforeDolly);

    const float radiusBefore = glm::length(dollyMatrices.position - pivot);
    const auto pivotNdcBeforeOrbit = projectNdc(dollyMatrices, pivot);
    camera.Orbit(24.0F, 0.0F);
    const auto orbitedMatrices = camera.Matrices(1.0F);
    const auto pivotNdcAfterOrbit = projectNdc(orbitedMatrices, pivot);
    CHECK(pivotNdcAfterOrbit.x == Catch::Approx(pivotNdcBeforeOrbit.x).margin(0.0001F));
    CHECK(pivotNdcAfterOrbit.y == Catch::Approx(pivotNdcBeforeOrbit.y).margin(0.0001F));
    CHECK(glm::length(orbitedMatrices.position - pivot) == Catch::Approx(radiusBefore));
    CHECK(glm::length(orbitedMatrices.position - dollyMatrices.position) > 0.01F);
    CHECK(camera.OrbitCenter().x == Catch::Approx(pivot.x));
    CHECK(camera.OrbitCenter().y == Catch::Approx(pivot.y));
    CHECK(camera.OrbitCenter().z == Catch::Approx(pivot.z));

    const auto pivotNdcBeforePitchOrbit = projectNdc(orbitedMatrices, pivot);
    camera.Orbit(0.0F, 18.0F);
    const auto pitchOrbitedMatrices = camera.Matrices(1.0F);
    const auto pivotNdcAfterPitchOrbit = projectNdc(pitchOrbitedMatrices, pivot);
    CHECK(pivotNdcAfterPitchOrbit.x == Catch::Approx(pivotNdcBeforePitchOrbit.x).margin(0.0001F));
    CHECK(pivotNdcAfterPitchOrbit.y == Catch::Approx(pivotNdcBeforePitchOrbit.y).margin(0.0001F));
    CHECK(camera.OrbitCenter().x == Catch::Approx(pivot.x));
    CHECK(camera.OrbitCenter().y == Catch::Approx(pivot.y));
    CHECK(camera.OrbitCenter().z == Catch::Approx(pivot.z));
}

TEST_CASE("Point preview LOD resolver only applies automatic LOD to camera motion", "[budget][lod]") {
    using invisible_places::renderer::pointcloud::MakePointBudgetState;
    using invisible_places::renderer::pointcloud::PointCloudPreviewLodMode;
    using invisible_places::renderer::pointcloud::ResolvePointCloudPreviewLod;

    const auto largeBudget = MakePointBudgetState(42'000'000, 42'000'000);
    const auto panelOnly = ResolvePointCloudPreviewLod(
        largeBudget,
        PointCloudPreviewLodMode::AutoCameraLod,
        false,
        false,
        10'000'000);
    CHECK(panelOnly.drawPointCount == 42'000'000);
    CHECK(!panelOnly.usesPreviewLod);

    const auto cameraNavigation = ResolvePointCloudPreviewLod(
        largeBudget,
        PointCloudPreviewLodMode::AutoCameraLod,
        true,
        false,
        10'000'000);
    CHECK(cameraNavigation.drawPointCount == 10'000'000);
    CHECK(cameraNavigation.usesPreviewLod);

    const auto cameraPlayback = ResolvePointCloudPreviewLod(
        largeBudget,
        PointCloudPreviewLodMode::AutoCameraLod,
        false,
        true,
        10'000'000);
    CHECK(cameraPlayback.drawPointCount == 10'000'000);
    CHECK(cameraPlayback.usesPreviewLod);

    const auto fullOverride = ResolvePointCloudPreviewLod(
        largeBudget,
        PointCloudPreviewLodMode::FullResolution,
        true,
        true,
        10'000'000);
    CHECK(fullOverride.drawPointCount == 42'000'000);
    CHECK(!fullOverride.usesPreviewLod);

    const auto forceOverride = ResolvePointCloudPreviewLod(
        largeBudget,
        PointCloudPreviewLodMode::ForceLod,
        false,
        false,
        10'000'000);
    CHECK(forceOverride.drawPointCount == 10'000'000);
    CHECK(forceOverride.usesPreviewLod);

    const auto userCappedBudget = MakePointBudgetState(42'000'000, 4'000'000);
    const auto manualBudget = ResolvePointCloudPreviewLod(
        userCappedBudget,
        PointCloudPreviewLodMode::ForceLod,
        true,
        true,
        10'000'000);
    CHECK(manualBudget.drawPointCount == 4'000'000);
    CHECK(!manualBudget.usesPreviewLod);
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

TEST_CASE("Point-cloud colormaps sample the Matplotlib listed tables", "[style][colormap]") {
    using invisible_places::renderer::pointcloud::PointCloudColormapId;
    using invisible_places::renderer::pointcloud::SampleColormap;

    auto checkColor = [](std::array<float, 3> color, std::array<float, 3> expected) {
        constexpr float tolerance = 1.0F / 255.0F;
        CHECK(color[0] == Catch::Approx(expected[0]).margin(tolerance));
        CHECK(color[1] == Catch::Approx(expected[1]).margin(tolerance));
        CHECK(color[2] == Catch::Approx(expected[2]).margin(tolerance));
    };

    checkColor(SampleColormap(PointCloudColormapId::Viridis, 0.0F), {0.267004F, 0.004874F, 0.329415F});
    checkColor(SampleColormap(PointCloudColormapId::Viridis, 128.0F / 255.0F), {0.127568F, 0.566949F, 0.550556F});
    checkColor(SampleColormap(PointCloudColormapId::Viridis, 1.0F), {0.993248F, 0.906157F, 0.143936F});
    checkColor(SampleColormap(PointCloudColormapId::Plasma, 0.0F), {0.050383F, 0.029803F, 0.527975F});
    checkColor(SampleColormap(PointCloudColormapId::Inferno, 128.0F / 255.0F), {0.735683F, 0.215906F, 0.330245F});
    checkColor(SampleColormap(PointCloudColormapId::Magma, 128.0F / 255.0F), {0.716387F, 0.214982F, 0.475290F});
    checkColor(SampleColormap(PointCloudColormapId::Cividis, 1.0F), {0.995737F, 0.909344F, 0.217772F});
    checkColor(SampleColormap(PointCloudColormapId::Turbo, 128.0F / 255.0F), {0.643620F, 0.989990F, 0.233560F});
}

TEST_CASE("Project document round-trips binding-backed point-cloud styles", "[serialization][project]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_roundtrip.json";

    invisible_places::serialization::ProjectDocument document;
    document.projectName = "Roundtrip";
    document.selectedLayerPath = "Data/Site2 -5mm.ply";
    document.lastAnimationPath = "Saved/animations/Roundtrip.ipanim.json";
    document.backgroundColor = {0.02F, 0.04F, 0.08F, 1.0F};
    document.sidePanelPinned = true;
    document.autoLowerGsplatQualityWhileNavigating = false;
    document.pointCloudPreviewLodMode =
        invisible_places::renderer::pointcloud::PointCloudPreviewLodMode::ForceLod;
    document.interactivePointCap = 12'345'678;
    document.renderJobSettings.outputDirectory = "Saved/renders/Roundtrip";
    document.renderJobSettings.width = 3840;
    document.renderJobSettings.height = 2160;
    document.renderJobSettings.framesPerSecond = 24;
    document.renderJobSettings.tileSize = 256;
    document.renderJobSettings.startFrame = 10;
    document.renderJobSettings.endFrame = 42;
    document.renderJobSettings.fromShotIndex = 0;
    document.renderJobSettings.toShotIndex = 1;
    document.cameraPathShotIndices = {0, 0};
    document.cameraPathDurationFrames = 144;
    invisible_places::camera::CameraState currentCamera;
    currentCamera.position = {10.0F, 20.0F, 30.0F};
    currentCamera.target = {4.0F, 5.0F, 6.0F};
    currentCamera.orbitCenter = {7.0F, 8.0F, 9.0F};
    currentCamera.hasOrbitCenter = true;
    currentCamera.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    currentCamera.fovDegrees = 42.0F;
    currentCamera.nearPlane = 0.001F;
    currentCamera.farPlane = 250.0F;
    document.cameraState = currentCamera;

    invisible_places::camera::CameraShot shot;
    shot.name = "Entry";
    shot.durationFrames = 120;
    shot.state.position = {1.0F, 2.0F, 3.0F};
    shot.state.target = {4.0F, 5.0F, 6.0F};
    shot.state.orbitCenter = {7.0F, 8.0F, 9.0F};
    shot.state.hasOrbitCenter = true;
    shot.state.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    document.cameraShots.push_back(shot);

    invisible_places::serialization::ProjectLayerDocument layer;
    layer.kind = invisible_places::serialization::SerializedLayerKind::PointCloud;
    layer.sourcePath = "Data/Site2 -5mm.ply";
    layer.loaded = true;
    layer.visible = true;
    layer.pointBudgetActivePoints = 2048;

    invisible_places::renderer::pointcloud::PointCloudStyleState pointStyle;
    pointStyle.geometryMode = invisible_places::renderer::pointcloud::PointCloudGeometryMode::WorldSurfels;
    pointStyle.depthContribution =
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::Always;
    pointStyle.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian;
    pointStyle.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap;
    pointStyle.colormap = invisible_places::renderer::pointcloud::PointCloudColormapId::Turbo;
    pointStyle.colorizeColor = {0.2F, 0.6F, 1.0F};
    pointStyle.colorizeAmount = 0.35F;
    pointStyle.exposure = 2.25F;
    pointStyle.innerRadius = 0.35F;
    pointStyle.gaussianSharpness = 5.5F;
    pointStyle.featherPower = 2.25F;
    pointStyle.depthFalloff = 123.0F;
    pointStyle.depthBias = 0.003F;
    pointStyle.frontAlpha = 0.21F;
    pointStyle.hiddenAlpha = 0.09F;
    pointStyle.densityScale = 1.75F;
    pointStyle.densityClamp = 96.0F;
    pointStyle.depthAlphaThreshold = 0.42F;
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
    invisible_places::style::SetScalarConstant(&pointStyle.surfelDiameter, 0.0125F);
    invisible_places::style::SetScalarConstant(&pointStyle.opacity, 0.55F);
    pointStyle.opacity.active = false;
    invisible_places::style::ConfigureFieldMapFromStats(
        &pointStyle.colormapPosition,
        1,
        "Intensity",
        0.0F,
        1.0F,
        nullptr);
    pointStyle.colormapPosition.active = false;
    layer.pointStyle = pointStyle;
    document.layers.push_back(layer);

    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, outputPath, &errorMessage));
    {
        std::ifstream savedProject{outputPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedProject},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find("\"render_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"blend_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"depth_contribution\"") != std::string::npos);
        CHECK(savedJson.find("\"active\"") != std::string::npos);
    }

    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    REQUIRE(loadedDocument->layers.size() == 1);

    CHECK(loadedDocument->projectName == "Roundtrip");
    CHECK(loadedDocument->sidePanelPinned);
    CHECK(!loadedDocument->autoLowerGsplatQualityWhileNavigating);
    CHECK(
        loadedDocument->pointCloudPreviewLodMode ==
        invisible_places::renderer::pointcloud::PointCloudPreviewLodMode::ForceLod);
    CHECK(loadedDocument->interactivePointCap == 12'345'678);
    CHECK(loadedDocument->renderJobSettings.outputDirectory == "Saved/renders/Roundtrip");
    CHECK(loadedDocument->renderJobSettings.width == 3840);
    CHECK(loadedDocument->renderJobSettings.height == 2160);
    CHECK(loadedDocument->renderJobSettings.framesPerSecond == 24);
    CHECK(loadedDocument->renderJobSettings.tileSize == 256);
    CHECK(loadedDocument->renderJobSettings.startFrame == 10);
    CHECK(loadedDocument->renderJobSettings.endFrame == 42);
    CHECK(loadedDocument->renderJobSettings.toShotIndex == 1);
    CHECK(loadedDocument->cameraPathShotIndices == std::vector<std::size_t>{0, 0});
    CHECK(loadedDocument->cameraPathDurationFrames == 144);
    CHECK(loadedDocument->backgroundColor[2] == Catch::Approx(0.08F));
    CHECK(loadedDocument->selectedLayerPath == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loadedDocument->lastAnimationPath == std::filesystem::path{"Saved/animations/Roundtrip.ipanim.json"});
    REQUIRE(loadedDocument->cameraState.has_value());
    CHECK(loadedDocument->cameraState->position[0] == Catch::Approx(10.0F));
    CHECK(loadedDocument->cameraState->position[1] == Catch::Approx(20.0F));
    CHECK(loadedDocument->cameraState->position[2] == Catch::Approx(30.0F));
    REQUIRE(loadedDocument->cameraState->hasOrbitCenter);
    CHECK(loadedDocument->cameraState->orbitCenter[0] == Catch::Approx(7.0F));
    CHECK(loadedDocument->cameraState->fovDegrees == Catch::Approx(42.0F));
    REQUIRE(loadedDocument->cameraShots.size() == 1);
    CHECK(loadedDocument->cameraShots[0].name == "Entry");
    CHECK(loadedDocument->cameraShots[0].durationFrames == 120);
    CHECK(loadedDocument->cameraShots[0].state.position[2] == Catch::Approx(3.0F));
    REQUIRE(loadedDocument->cameraShots[0].state.hasOrbitCenter);
    CHECK(loadedDocument->cameraShots[0].state.orbitCenter[0] == Catch::Approx(7.0F));
    CHECK(loadedDocument->cameraShots[0].state.orbitCenter[1] == Catch::Approx(8.0F));
    CHECK(loadedDocument->cameraShots[0].state.orbitCenter[2] == Catch::Approx(9.0F));

    const auto& loadedLayer = loadedDocument->layers.front();
    REQUIRE(loadedLayer.pointStyle.has_value());
    CHECK(loadedLayer.loaded);
    CHECK(loadedLayer.visible);
    CHECK(loadedLayer.pointBudgetActivePoints == 2048);
    CHECK(
        loadedLayer.pointStyle->colorMode ==
        invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap);
    CHECK(
        loadedLayer.pointStyle->geometryMode ==
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::WorldSurfels);
    CHECK(
        loadedLayer.pointStyle->depthContribution ==
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::Always);
    CHECK(
        loadedLayer.pointStyle->falloffProfile ==
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian);
    CHECK(
        loadedLayer.pointStyle->colormap ==
        invisible_places::renderer::pointcloud::PointCloudColormapId::Turbo);
    CHECK(loadedLayer.pointStyle->colorizeColor[0] == Catch::Approx(0.2F));
    CHECK(loadedLayer.pointStyle->colorizeColor[1] == Catch::Approx(0.6F));
    CHECK(loadedLayer.pointStyle->colorizeColor[2] == Catch::Approx(1.0F));
    CHECK(loadedLayer.pointStyle->colorizeAmount == Catch::Approx(0.35F));
    CHECK(loadedLayer.pointStyle->exposure == Catch::Approx(2.25F));
    CHECK(loadedLayer.pointStyle->innerRadius == Catch::Approx(0.35F));
    CHECK(loadedLayer.pointStyle->gaussianSharpness == Catch::Approx(5.5F));
    CHECK(loadedLayer.pointStyle->featherPower == Catch::Approx(2.25F));
    CHECK(loadedLayer.pointStyle->depthFalloff == Catch::Approx(123.0F));
    CHECK(loadedLayer.pointStyle->depthBias == Catch::Approx(0.003F));
    CHECK(loadedLayer.pointStyle->frontAlpha == Catch::Approx(0.21F));
    CHECK(loadedLayer.pointStyle->hiddenAlpha == Catch::Approx(0.09F));
    CHECK(loadedLayer.pointStyle->densityScale == Catch::Approx(1.75F));
    CHECK(loadedLayer.pointStyle->densityClamp == Catch::Approx(96.0F));
    CHECK(loadedLayer.pointStyle->depthAlphaThreshold == Catch::Approx(0.42F));
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.fieldSlot == 2);
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.fieldName == "Height");
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.inputMin == Catch::Approx(-2.0F));
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.outputMax == Catch::Approx(8.0F));
    CHECK(invisible_places::style::ScalarConstant(loadedLayer.pointStyle->surfelDiameter) == Catch::Approx(0.0125F));
    CHECK(invisible_places::style::ScalarConstant(loadedLayer.pointStyle->opacity) == Catch::Approx(0.55F));
    CHECK(!loadedLayer.pointStyle->opacity.active);
    CHECK(loadedLayer.pointStyle->colormapPosition.fieldMap.fieldName == "Intensity");
    CHECK(!loadedLayer.pointStyle->colormapPosition.active);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Point cloud style parsing defaults missing surfel fields to sprite mode", "[serialization]") {
    const auto presetPath = std::filesystem::temp_directory_path() / "invisible_places_legacy_point_style.json";
    std::ofstream output{presetPath, std::ios::trunc};
    output << R"({
  "schema_version": 1,
  "preset_name": "Legacy",
  "point_style": {
    "render_mode": "solid",
    "point_size": {"mode": "constant", "constant_value": [3.0, 0.0, 0.0, 0.0]}
  }
})";
    output.close();

    std::string errorMessage;
    const auto preset = invisible_places::serialization::LoadPointCloudStylePreset(presetPath, &errorMessage);
    REQUIRE(preset.has_value());
    CHECK(
        preset->style.geometryMode ==
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::ScreenSprites);
    CHECK(
        preset->style.depthContribution ==
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::AlphaThreshold);
    CHECK(preset->style.pointSize.active);
    CHECK(preset->style.opacity.active);
    CHECK(preset->style.emissiveStrength.active);
    CHECK(invisible_places::style::ScalarConstant(preset->style.surfelDiameter) == Catch::Approx(0.005F));

    std::filesystem::remove(presetPath);
}

TEST_CASE("Legacy point render modes migrate to unified material style", "[serialization][point-style]") {
    const auto presetPath = std::filesystem::temp_directory_path() / "invisible_places_legacy_render_mode.json";

    auto loadLegacyMode = [&](const std::string& modeName) {
        std::ofstream output{presetPath, std::ios::trunc};
        output << R"({
  "schema_version": 1,
  "preset_name": "Legacy",
  "point_style": {
    "render_mode": ")" << modeName << R"(",
    "density_scale": 2.5,
    "density_clamp": 72.0
  }
})";
        output.close();

        std::string errorMessage;
        const auto preset = invisible_places::serialization::LoadPointCloudStylePreset(presetPath, &errorMessage);
        REQUIRE(preset.has_value());
        return preset->style;
    };

    const auto solid = loadLegacyMode("solid");
    CHECK(
        solid.depthContribution ==
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::AlphaThreshold);

    const auto emissiveHard = loadLegacyMode("emissive_hard");
    CHECK(
        emissiveHard.falloffProfile ==
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc);
    CHECK(invisible_places::style::ScalarConstant(emissiveHard.emissiveStrength) == Catch::Approx(1.0F));

    const auto emissiveFeathered = loadLegacyMode("emissive_feathered");
    CHECK(
        emissiveFeathered.falloffProfile ==
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian);
    CHECK(invisible_places::style::ScalarConstant(emissiveFeathered.emissiveStrength) == Catch::Approx(1.0F));

    const auto xray = loadLegacyMode("depth_xray");
    CHECK(
        xray.depthContribution ==
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::Always);
    CHECK(invisible_places::style::ScalarConstant(xray.xrayStrength) == Catch::Approx(1.0F));

    const auto weighted = loadLegacyMode("weighted_transparent");
    CHECK(weighted.densityScale == Catch::Approx(2.5F));
    CHECK(weighted.densityClamp == Catch::Approx(72.0F));

    const auto density = loadLegacyMode("compute_density");
    CHECK(density.densityScale == Catch::Approx(2.5F));
    CHECK(density.densityClamp == Catch::Approx(72.0F));

    const auto gaussianSprite = loadLegacyMode("gaussian_point_sprite");
    CHECK(
        gaussianSprite.falloffProfile ==
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian);

    std::filesystem::remove(presetPath);
}

TEST_CASE("Point depth contribution policy is shared by preview and export selection", "[point-style]") {
    invisible_places::renderer::pointcloud::PointCloudStyleState style;

    style.depthContribution = invisible_places::renderer::pointcloud::PointCloudDepthContribution::None;
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style));
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style, true));
    CHECK(!invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(style, 1.0F));

    style.depthContribution =
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::AlphaThreshold;
    style.depthAlphaThreshold = 0.5F;
    CHECK(invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style));
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style, false));
    CHECK(invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style, true));
    CHECK(!invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(style, 0.49F));
    CHECK(invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(style, 0.5F));

    style.depthContribution = invisible_places::renderer::pointcloud::PointCloudDepthContribution::Always;
    CHECK(invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style));
    CHECK(invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(style, 0.01F));
}

TEST_CASE("New point styles default to no depth prepass", "[point-style]") {
    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    CHECK(
        style.depthContribution ==
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::None);
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style));
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(style, true));
}

TEST_CASE("Point X-Ray gates scene depth prepass selection", "[point-style]") {
    invisible_places::renderer::pointcloud::PointCloudStyleState occluder;
    occluder.depthContribution =
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::AlphaThreshold;

    invisible_places::renderer::pointcloud::PointCloudStyleState xray;
    invisible_places::style::SetScalarConstant(&xray.xrayStrength, 0.0F);
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleHasActiveXray(xray));
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(occluder, false));

    invisible_places::style::SetScalarConstant(&xray.xrayStrength, 0.35F);
    CHECK(invisible_places::renderer::pointcloud::PointCloudStyleHasActiveXray(xray));
    CHECK(invisible_places::renderer::pointcloud::PointCloudStyleUsesDepthPrepass(occluder, true));

    xray.xrayStrength.active = false;
    CHECK(!invisible_places::renderer::pointcloud::PointCloudStyleHasActiveXray(xray));
}

TEST_CASE("Point material variant resolver selects simple and unified paths", "[point-style]") {
    using invisible_places::renderer::pointcloud::PointCloudColorMode;
    using invisible_places::renderer::pointcloud::PointCloudMaterialVariant;
    using invisible_places::renderer::pointcloud::ResolvePointCloudMaterialVariant;
    using invisible_places::style::ParameterSourceMode;

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.colorMode = PointCloudColorMode::SourceRgb;
    invisible_places::style::SetScalarConstant(&style.opacity, 0.65F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 1.25F);
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::ConstantSimple);

    auto fieldOpacity = style;
    fieldOpacity.opacity.mode = ParameterSourceMode::FieldMapped;
    CHECK(ResolvePointCloudMaterialVariant(fieldOpacity) == PointCloudMaterialVariant::Unified);

    auto fieldEmission = style;
    fieldEmission.emissiveStrength.mode = ParameterSourceMode::FieldMapped;
    CHECK(ResolvePointCloudMaterialVariant(fieldEmission) == PointCloudMaterialVariant::Unified);

    auto fieldColormapPosition = style;
    fieldColormapPosition.colormapPosition.mode = ParameterSourceMode::FieldMapped;
    CHECK(ResolvePointCloudMaterialVariant(fieldColormapPosition) == PointCloudMaterialVariant::Unified);

    auto xray = style;
    invisible_places::style::SetScalarConstant(&xray.xrayStrength, 0.1F);
    CHECK(ResolvePointCloudMaterialVariant(xray) == PointCloudMaterialVariant::Unified);

    auto depthFade = style;
    invisible_places::style::SetScalarConstant(&depthFade.depthFade, 0.5F);
    CHECK(ResolvePointCloudMaterialVariant(depthFade) == PointCloudMaterialVariant::Unified);

    auto colormap = style;
    colormap.colorMode = PointCloudColorMode::ScalarColormap;
    CHECK(ResolvePointCloudMaterialVariant(colormap) == PointCloudMaterialVariant::Unified);
}

TEST_CASE("Camera shot interpolation stores quaternion slerp and linear camera values", "[camera][shots]") {
    invisible_places::camera::CameraState start;
    start.position = {0.0F, 0.0F, 0.0F};
    start.target = {0.0F, 0.0F, -1.0F};
    start.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    start.fovDegrees = 50.0F;

    invisible_places::camera::CameraState end;
    end.position = {10.0F, 0.0F, 0.0F};
    end.target = {10.0F, 0.0F, -1.0F};
    const auto endOrientation =
        glm::angleAxis(glm::half_pi<float>(), glm::vec3{0.0F, 0.0F, 1.0F});
    end.orientation = {
        endOrientation.x,
        endOrientation.y,
        endOrientation.z,
        endOrientation.w,
    };
    end.fovDegrees = 70.0F;

    const auto midpoint = invisible_places::camera::InterpolateCameraStates(start, end, 0.5F);
    const auto midpointOrientation = invisible_places::camera::QuaternionFromCameraState(midpoint);

    CHECK(midpoint.position[0] == Catch::Approx(5.0F));
    CHECK(midpoint.target[0] == Catch::Approx(5.0F));
    CHECK(midpoint.fovDegrees == Catch::Approx(60.0F));
    CHECK(midpointOrientation.z == Catch::Approx(std::sin(glm::quarter_pi<float>() * 0.5F)));
    CHECK(midpointOrientation.w == Catch::Approx(std::cos(glm::quarter_pi<float>() * 0.5F)));
}

TEST_CASE("Camera path splines pass through middle camera waypoints", "[camera][path]") {
    auto writeOrientation = [](invisible_places::camera::CameraShot* shot, const glm::quat& orientation) {
        shot->state.orientation = {orientation.x, orientation.y, orientation.z, orientation.w};
    };

    invisible_places::camera::CameraShot firstShot;
    firstShot.state.position = {0.0F, 0.0F, 0.0F};
    firstShot.state.target = {0.0F, 0.0F, -2.0F};
    writeOrientation(&firstShot, glm::quat{1.0F, 0.0F, 0.0F, 0.0F});

    invisible_places::camera::CameraShot middleShot;
    middleShot.durationFrames = 30;
    middleShot.state.position = {1.0F, 2.0F, 0.5F};
    middleShot.state.target = {2.0F, 2.5F, -2.0F};
    middleShot.state.fovDegrees = 48.0F;
    const auto middleOrientation = glm::angleAxis(glm::quarter_pi<float>(), glm::vec3{0.0F, 0.0F, 1.0F});
    writeOrientation(&middleShot, middleOrientation);

    invisible_places::camera::CameraShot lastShot;
    lastShot.durationFrames = 30;
    lastShot.state.position = {4.0F, 0.0F, 1.0F};
    lastShot.state.target = {4.0F, 0.0F, -2.0F};
    writeOrientation(&lastShot, glm::angleAxis(glm::half_pi<float>(), glm::vec3{0.0F, 0.0F, 1.0F}));

    const std::vector<invisible_places::camera::CameraShot> shots = {firstShot, middleShot, lastShot};
    const auto timing = invisible_places::camera::BuildCameraPathTiming(shots, 0, 2);
    REQUIRE(timing.IsValid());
    REQUIRE(timing.knotSeconds.size() == 3);

    const auto evaluated = invisible_places::camera::EvaluateCameraPath(shots, timing, timing.knotSeconds[1]);
    const auto evaluatedOrientation = invisible_places::camera::QuaternionFromCameraState(evaluated);

    CHECK(evaluated.position[0] == Catch::Approx(middleShot.state.position[0]));
    CHECK(evaluated.position[1] == Catch::Approx(middleShot.state.position[1]));
    CHECK(evaluated.position[2] == Catch::Approx(middleShot.state.position[2]));
    CHECK(evaluated.target[0] == Catch::Approx(middleShot.state.target[0]));
    CHECK(evaluated.target[1] == Catch::Approx(middleShot.state.target[1]));
    CHECK(evaluated.target[2] == Catch::Approx(middleShot.state.target[2]));
    CHECK(evaluated.fovDegrees == Catch::Approx(middleShot.state.fovDegrees));
    CHECK(std::abs(glm::dot(evaluatedOrientation, middleOrientation)) == Catch::Approx(1.0F).margin(0.0001F));
}

TEST_CASE("Camera path keeps velocity and acceleration smooth through middle positions", "[camera][path]") {
    const auto makeShot = [](std::array<float, 3> position, std::uint32_t durationFrames) {
        invisible_places::camera::CameraShot shot;
        shot.durationFrames = durationFrames;
        shot.state.position = position;
        shot.state.target = {position[0], position[1], position[2] - 3.0F};
        return shot;
    };
    const std::vector<invisible_places::camera::CameraShot> shots = {
        makeShot({0.0F, 0.0F, 0.0F}, 30),
        makeShot({1.0F, 2.0F, 0.5F}, 30),
        makeShot({4.0F, -1.0F, 1.0F}, 45),
        makeShot({7.0F, 1.0F, 0.0F}, 30),
    };

    const auto timing = invisible_places::camera::BuildCameraPathTiming(shots, 0, 3);
    REQUIRE(timing.IsValid());

    const auto evaluatePosition = [&shots, &timing](float timeSeconds) {
        const auto state = invisible_places::camera::EvaluateCameraPath(shots, timing, timeSeconds);
        return glm::vec3{state.position[0], state.position[1], state.position[2]};
    };

    const float knotTime = timing.knotSeconds[1];
    constexpr float h = 0.005F;
    const auto velocityIn = (evaluatePosition(knotTime) - evaluatePosition(knotTime - h)) / h;
    const auto velocityOut = (evaluatePosition(knotTime + h) - evaluatePosition(knotTime)) / h;
    const auto accelerationIn =
        (evaluatePosition(knotTime) - (2.0F * evaluatePosition(knotTime - h)) + evaluatePosition(knotTime - (2.0F * h))) /
        (h * h);
    const auto accelerationOut =
        (evaluatePosition(knotTime + (2.0F * h)) - (2.0F * evaluatePosition(knotTime + h)) + evaluatePosition(knotTime)) /
        (h * h);

    CHECK(velocityOut.x == Catch::Approx(velocityIn.x).margin(0.06F));
    CHECK(velocityOut.y == Catch::Approx(velocityIn.y).margin(0.06F));
    CHECK(velocityOut.z == Catch::Approx(velocityIn.z).margin(0.06F));
    CHECK(accelerationOut.x == Catch::Approx(accelerationIn.x).margin(0.5F));
    CHECK(accelerationOut.y == Catch::Approx(accelerationIn.y).margin(0.5F));
    CHECK(accelerationOut.z == Catch::Approx(accelerationIn.z).margin(0.5F));
}

TEST_CASE("Camera path keeps flipped quaternion inputs sign-continuous", "[camera][path]") {
    auto writeOrientation = [](invisible_places::camera::CameraShot* shot, glm::quat orientation) {
        shot->state.orientation = {orientation.x, orientation.y, orientation.z, orientation.w};
    };
    auto makeShot = [&writeOrientation](glm::quat orientation, std::uint32_t durationFrames) {
        invisible_places::camera::CameraShot shot;
        shot.durationFrames = durationFrames;
        shot.state.target = {0.0F, 0.0F, -4.0F};
        writeOrientation(&shot, orientation);
        return shot;
    };

    const auto middleOrientation = glm::angleAxis(glm::quarter_pi<float>(), glm::vec3{0.0F, 0.0F, 1.0F});
    const std::vector<invisible_places::camera::CameraShot> shots = {
        makeShot(glm::quat{1.0F, 0.0F, 0.0F, 0.0F}, 30),
        makeShot(-middleOrientation, 30),
        makeShot(glm::angleAxis(glm::half_pi<float>(), glm::vec3{0.0F, 0.0F, 1.0F}), 30),
    };

    const auto timing = invisible_places::camera::BuildCameraPathTiming(shots, 0, 2);
    REQUIRE(timing.IsValid());

    auto previousOrientation = invisible_places::camera::QuaternionFromCameraState(
        invisible_places::camera::EvaluateCameraPath(shots, timing, 0.0F));
    for (std::uint32_t sampleIndex = 1; sampleIndex <= 60; ++sampleIndex) {
        const float timeSeconds =
            timing.DurationSeconds() * (static_cast<float>(sampleIndex) / 60.0F);
        const auto orientation = invisible_places::camera::QuaternionFromCameraState(
            invisible_places::camera::EvaluateCameraPath(shots, timing, timeSeconds));
        CHECK(glm::dot(previousOrientation, orientation) > 0.0F);
        previousOrientation = orientation;
    }

    const auto middleState =
        invisible_places::camera::EvaluateCameraPath(shots, timing, timing.knotSeconds[1]);
    const auto evaluatedMiddleOrientation = invisible_places::camera::QuaternionFromCameraState(middleState);
    CHECK(glm::dot(evaluatedMiddleOrientation, middleOrientation) == Catch::Approx(1.0F).margin(0.0001F));
}

TEST_CASE("Weighted camera paths keep duplicate entries and total duration", "[camera][path]") {
    auto makeShot = [](const char* name, std::array<float, 3> position) {
        invisible_places::camera::CameraShot shot;
        shot.name = name;
        shot.state.position = position;
        shot.state.target = {position[0], position[1], position[2] - 2.0F};
        return shot;
    };

    const auto firstShot = makeShot("A", {0.0F, 0.0F, 0.0F});
    const auto duplicateShot = firstShot;
    const auto lastShot = makeShot("B", {8.0F, 0.0F, 0.0F});
    const auto weightedShots = invisible_places::camera::BuildWeightedCameraPathShots(
        {firstShot, duplicateShot, lastShot},
        12U);

    REQUIRE(weightedShots.size() == 3);
    CHECK(weightedShots[1].durationFrames >= 1U);
    CHECK(weightedShots[2].durationFrames >= 1U);
    CHECK(weightedShots[1].durationFrames + weightedShots[2].durationFrames == 12U);

    const auto timing = invisible_places::camera::BuildCameraPathTiming(weightedShots, 0, 2);
    REQUIRE(timing.IsValid());
    const auto middleState =
        invisible_places::camera::EvaluateCameraPath(weightedShots, timing, timing.knotSeconds[1]);
    CHECK(middleState.position[0] == Catch::Approx(duplicateShot.state.position[0]));
    CHECK(middleState.position[1] == Catch::Approx(duplicateShot.state.position[1]));
    CHECK(middleState.position[2] == Catch::Approx(duplicateShot.state.position[2]));
}

TEST_CASE("Weighted camera paths distribute duration by movement when possible", "[camera][path]") {
    auto makeShot = [](std::array<float, 3> position, glm::quat orientation) {
        invisible_places::camera::CameraShot shot;
        shot.state.position = position;
        shot.state.target = {position[0], position[1], position[2] - 3.0F};
        invisible_places::camera::WriteQuaternionToCameraState(orientation, &shot.state);
        return shot;
    };

    const auto weightedShots = invisible_places::camera::BuildWeightedCameraPathShots(
        {
            makeShot({0.0F, 0.0F, 0.0F}, glm::quat{1.0F, 0.0F, 0.0F, 0.0F}),
            makeShot({1.0F, 0.0F, 0.0F}, glm::quat{1.0F, 0.0F, 0.0F, 0.0F}),
            makeShot(
                {9.0F, 0.0F, 0.0F},
                glm::angleAxis(glm::half_pi<float>(), glm::vec3{0.0F, 0.0F, 1.0F})),
        },
        18U);

    REQUIRE(weightedShots.size() == 3);
    CHECK(weightedShots[1].durationFrames + weightedShots[2].durationFrames == 18U);
    CHECK(weightedShots[2].durationFrames > weightedShots[1].durationFrames);
}

TEST_CASE("Animation path evaluation passes through camera and focus keys", "[camera][animation]") {
    invisible_places::camera::AnimationPath path;
    path.name = "Pass Through";
    path.durationFrames = 60;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 1.0F, 0.0F}, .durationFrames = 30},
        {.cameraPosition = {1.0F, 2.0F, 0.5F}, .focusPoint = {2.0F, 2.5F, 0.5F}, .durationFrames = 30},
        {.cameraPosition = {4.0F, 0.0F, 1.0F}, .focusPoint = {4.5F, 0.5F, 1.0F}, .durationFrames = 30},
    };

    const auto evaluation = invisible_places::camera::EvaluateAnimationPath(path, 1.0F);
    CHECK(evaluation.camera.position[0] == Catch::Approx(path.keys[1].cameraPosition[0]));
    CHECK(evaluation.camera.position[1] == Catch::Approx(path.keys[1].cameraPosition[1]));
    CHECK(evaluation.camera.position[2] == Catch::Approx(path.keys[1].cameraPosition[2]));
    CHECK(evaluation.focusPoint[0] == Catch::Approx(path.keys[1].focusPoint[0]));
    CHECK(evaluation.focusPoint[1] == Catch::Approx(path.keys[1].focusPoint[1]));
    CHECK(evaluation.focusPoint[2] == Catch::Approx(path.keys[1].focusPoint[2]));
}

TEST_CASE("Animation path keeps camera and focus derivatives smooth through middle keys", "[camera][animation]") {
    invisible_places::camera::AnimationPath path;
    path.name = "Smooth";
    path.durationFrames = 90;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 1.0F, 0.0F}, .durationFrames = 30},
        {.cameraPosition = {1.0F, 2.0F, 0.5F}, .focusPoint = {2.0F, 2.5F, 0.5F}, .durationFrames = 30},
        {.cameraPosition = {4.0F, -1.0F, 1.0F}, .focusPoint = {4.5F, -0.5F, 1.5F}, .durationFrames = 30},
        {.cameraPosition = {7.0F, 1.0F, 0.0F}, .focusPoint = {7.0F, 1.0F, 1.0F}, .durationFrames = 30},
    };

    const auto evaluatePoint = [&path](float timeSeconds, bool focus) {
        const auto evaluation = invisible_places::camera::EvaluateAnimationPath(path, timeSeconds);
        if (focus) {
            return glm::vec3{evaluation.focusPoint[0], evaluation.focusPoint[1], evaluation.focusPoint[2]};
        }
        return glm::vec3{
            evaluation.camera.position[0],
            evaluation.camera.position[1],
            evaluation.camera.position[2]};
    };

    constexpr float knotTime = 1.0F;
    constexpr float h = 0.005F;
    for (const bool focus : {false, true}) {
        const auto velocityIn = (evaluatePoint(knotTime, focus) - evaluatePoint(knotTime - h, focus)) / h;
        const auto velocityOut = (evaluatePoint(knotTime + h, focus) - evaluatePoint(knotTime, focus)) / h;
        const auto accelerationIn =
            (evaluatePoint(knotTime, focus) -
             (2.0F * evaluatePoint(knotTime - h, focus)) +
             evaluatePoint(knotTime - (2.0F * h), focus)) /
            (h * h);
        const auto accelerationOut =
            (evaluatePoint(knotTime + (2.0F * h), focus) -
             (2.0F * evaluatePoint(knotTime + h, focus)) +
             evaluatePoint(knotTime, focus)) /
            (h * h);

        CHECK(velocityOut.x == Catch::Approx(velocityIn.x).margin(0.08F));
        CHECK(velocityOut.y == Catch::Approx(velocityIn.y).margin(0.08F));
        CHECK(velocityOut.z == Catch::Approx(velocityIn.z).margin(0.08F));
        CHECK(accelerationOut.x == Catch::Approx(accelerationIn.x).margin(0.8F));
        CHECK(accelerationOut.y == Catch::Approx(accelerationIn.y).margin(0.8F));
        CHECK(accelerationOut.z == Catch::Approx(accelerationIn.z).margin(0.8F));
    }
}

TEST_CASE("Animation path depth of field is opt-in by default", "[camera][animation]") {
    const invisible_places::camera::AnimationPath path;
    CHECK_FALSE(path.depthOfFieldEnabled);
}

TEST_CASE("Animation path looks at the focal spline and stores focus distance", "[camera][animation]") {
    invisible_places::camera::AnimationPath path;
    path.name = "Focus";
    path.durationFrames = 30;
    path.depthOfFieldEnabled = true;
    path.apertureFStops = 4.0F;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 3.0F, 4.0F}, .durationFrames = 30},
        {.cameraPosition = {1.0F, 0.0F, 0.0F}, .focusPoint = {1.0F, 3.0F, 4.0F}, .durationFrames = 30},
    };

    const auto evaluation = invisible_places::camera::EvaluateAnimationPath(path, 0.0F);
    const auto orientation = invisible_places::camera::QuaternionFromCameraState(evaluation.camera);
    const glm::vec3 forward = orientation * glm::vec3{0.0F, 0.0F, -1.0F};
    const glm::vec3 expectedForward = glm::normalize(glm::vec3{0.0F, 3.0F, 4.0F});

    CHECK(glm::dot(forward, expectedForward) == Catch::Approx(1.0F).margin(0.0001F));
    CHECK(evaluation.focusDistance == Catch::Approx(5.0F));
    CHECK(evaluation.camera.hasDepthOfField);
    CHECK(evaluation.camera.focusDistance == Catch::Approx(5.0F));
    CHECK(evaluation.camera.apertureFStops == Catch::Approx(4.0F));
    CHECK(evaluation.camera.depthOfFieldMaxBlurPixels == Catch::Approx(24.0F));
}

TEST_CASE("Animation path serialization round-trips standalone files", "[serialization][animation]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_roundtrip.ipanim.json";

    invisible_places::camera::AnimationPath path;
    path.name = "Roundtrip Animation";
    path.durationFrames = 72;
    path.depthOfFieldEnabled = false;
    path.apertureFStops = 2.8F;
    path.depthOfFieldMaxBlurPixels = 36.0F;
    path.keys = {
        {
            .cameraPosition = {1.0F, 2.0F, 3.0F},
            .focusPoint = {4.0F, 5.0F, 6.0F},
            .fovDegrees = 42.0F,
            .nearPlane = 0.02F,
            .farPlane = 900.0F,
            .durationFrames = 24,
            .sourceShotName = "Entry",
        },
        {
            .cameraPosition = {7.0F, 8.0F, 9.0F},
            .focusPoint = {10.0F, 11.0F, 12.0F},
            .fovDegrees = 55.0F,
            .nearPlane = 0.04F,
            .farPlane = 1200.0F,
            .durationFrames = 48,
            .sourceShotName = "Exit",
        },
    };

    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveAnimationPath(path, outputPath, &errorMessage));
    const auto loadedPath = invisible_places::serialization::LoadAnimationPath(outputPath, &errorMessage);
    REQUIRE(loadedPath.has_value());
    REQUIRE(loadedPath->keys.size() == 2);
    CHECK(loadedPath->name == "Roundtrip Animation");
    CHECK(loadedPath->durationFrames == 72);
    CHECK_FALSE(loadedPath->depthOfFieldEnabled);
    CHECK(loadedPath->apertureFStops == Catch::Approx(2.8F));
    CHECK(loadedPath->depthOfFieldMaxBlurPixels == Catch::Approx(36.0F));
    CHECK(loadedPath->keys[0].cameraPosition[2] == Catch::Approx(3.0F));
    CHECK(loadedPath->keys[0].focusPoint[1] == Catch::Approx(5.0F));
    CHECK(loadedPath->keys[0].fovDegrees == Catch::Approx(42.0F));
    CHECK(loadedPath->keys[0].nearPlane == Catch::Approx(0.02F));
    CHECK(loadedPath->keys[0].farPlane == Catch::Approx(900.0F));
    CHECK(loadedPath->keys[0].sourceShotName == "Entry");

    std::filesystem::remove(outputPath);
}

TEST_CASE("Animation path loading reports invalid files without producing a path", "[serialization][animation]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_invalid.ipanim.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << "{\"keys\": [";
    }

    std::string errorMessage;
    const auto loadedPath = invisible_places::serialization::LoadAnimationPath(outputPath, &errorMessage);
    CHECK(!loadedPath.has_value());
    CHECK(!errorMessage.empty());
    std::filesystem::remove(outputPath);
}

TEST_CASE("Animation key edits do not mutate source camera shots", "[camera][animation]") {
    invisible_places::camera::CameraShot shot;
    shot.name = "Original";
    shot.state.position = {1.0F, 2.0F, 3.0F};
    shot.state.target = {4.0F, 5.0F, 6.0F};
    shot.state.orbitCenter = {7.0F, 8.0F, 9.0F};
    shot.state.hasOrbitCenter = true;

    const std::vector<invisible_places::camera::CameraShot> sourceShots = {shot, shot};
    auto animationPath = invisible_places::camera::BuildAnimationPathFromCameraShots(
        "Editable",
        sourceShots,
        30U);

    invisible_places::camera::MoveAnimationCameraKey(&animationPath, 0, {10.0F, 11.0F, 12.0F});
    invisible_places::camera::MoveAnimationFocusKey(&animationPath, 0, {13.0F, 14.0F, 15.0F});

    CHECK(animationPath.keys[0].cameraPosition[0] == Catch::Approx(10.0F));
    CHECK(animationPath.keys[0].focusPoint[0] == Catch::Approx(13.0F));
    CHECK(sourceShots[0].state.position[0] == Catch::Approx(1.0F));
    CHECK(sourceShots[0].state.orbitCenter[0] == Catch::Approx(7.0F));
}

TEST_CASE("Orbit camera applies shot quaternion as view direction", "[camera][shots]") {
    invisible_places::camera::CameraState state;
    state.position = {0.0F, 0.0F, 0.0F};
    state.target = {0.0F, 0.0F, -5.0F};
    state.orbitCenter = {2.0F, 1.0F, 0.0F};
    state.hasOrbitCenter = true;
    const auto orientation = glm::angleAxis(glm::half_pi<float>(), glm::vec3{0.0F, 1.0F, 0.0F});
    state.orientation = {orientation.x, orientation.y, orientation.z, orientation.w};

    invisible_places::camera::OrbitCamera camera;
    camera.ApplyState(state);
    const auto matrices = camera.Matrices(1.0F);
    const glm::vec3 expectedForwardPoint = orientation * glm::vec3{0.0F, 0.0F, -3.0F};
    const glm::vec4 viewPosition = matrices.view * glm::vec4{expectedForwardPoint, 1.0F};

    CHECK(viewPosition.x == Catch::Approx(0.0F).margin(0.0001F));
    CHECK(viewPosition.y == Catch::Approx(0.0F).margin(0.0001F));
    CHECK(viewPosition.z < 0.0F);
    CHECK(camera.OrbitCenter().x == Catch::Approx(state.orbitCenter[0]));
    CHECK(camera.OrbitCenter().y == Catch::Approx(state.orbitCenter[1]));
    CHECK(camera.OrbitCenter().z == Catch::Approx(state.orbitCenter[2]));
}

TEST_CASE("Orbit camera keeps legacy target-only shots usable", "[camera][shots]") {
    invisible_places::camera::CameraState state;
    state.position = {0.0F, 0.0F, 0.0F};
    state.target = {0.0F, 0.0F, -5.0F};
    state.orientation = {0.0F, 0.0F, 0.0F, 1.0F};

    invisible_places::camera::OrbitCamera camera;
    camera.ApplyState(state);

    CHECK(camera.Target().x == Catch::Approx(state.target[0]));
    CHECK(camera.Target().y == Catch::Approx(state.target[1]));
    CHECK(camera.Target().z == Catch::Approx(state.target[2]));
    CHECK(camera.OrbitCenter().x == Catch::Approx(state.target[0]));
    CHECK(camera.OrbitCenter().y == Catch::Approx(state.target[1]));
    CHECK(camera.OrbitCenter().z == Catch::Approx(state.target[2]));
}

TEST_CASE("Render camera sequence expands shots and frame ranges deterministically", "[output][camera]") {
    invisible_places::camera::CameraShot firstShot;
    firstShot.name = "A";
    firstShot.state.position = {0.0F, 0.0F, 0.0F};
    firstShot.state.target = {0.0F, 0.0F, -1.0F};

    invisible_places::camera::CameraShot secondShot;
    secondShot.name = "B";
    secondShot.durationFrames = 3;
    secondShot.state.position = {3.0F, 0.0F, 0.0F};
    secondShot.state.target = {3.0F, 0.0F, -1.0F};

    invisible_places::output::RenderJobSettings settings;
    settings.outputDirectory = "Saved/renders/Test";
    settings.framesPerSecond = 30;

    const std::vector<invisible_places::camera::CameraShot> shots = {firstShot, secondShot};
    const auto frames = invisible_places::output::BuildCameraRenderSequence(shots, settings);
    REQUIRE(frames.size() == 4);
    CHECK(frames[0].position[0] == Catch::Approx(0.0F));
    CHECK(frames[1].position[0] == Catch::Approx(1.0F));
    CHECK(frames[2].position[0] == Catch::Approx(2.0F));
    CHECK(frames[3].position[0] == Catch::Approx(3.0F));

    invisible_places::camera::CameraShot thirdShot;
    thirdShot.name = "C";
    thirdShot.durationFrames = 3;
    thirdShot.state.position = {6.0F, 1.0F, 0.0F};
    thirdShot.state.target = {6.0F, 1.0F, -1.0F};

    settings.toShotIndex = 2;
    const std::vector<invisible_places::camera::CameraShot> pathShots = {firstShot, secondShot, thirdShot};
    const auto pathFrames = invisible_places::output::BuildCameraRenderSequence(pathShots, settings);
    REQUIRE(pathFrames.size() == 7);
    CHECK(pathFrames[0].position[0] == Catch::Approx(firstShot.state.position[0]));
    CHECK(pathFrames[3].position[0] == Catch::Approx(secondShot.state.position[0]));
    CHECK(pathFrames[3].position[1] == Catch::Approx(secondShot.state.position[1]));
    CHECK(pathFrames[6].position[0] == Catch::Approx(thirdShot.state.position[0]));
    CHECK(pathFrames[6].position[1] == Catch::Approx(thirdShot.state.position[1]));

    settings.startFrame = 1;
    settings.endFrame = 2;
    settings.toShotIndex = 1;
    const auto rangedFrames = invisible_places::output::BuildCameraRenderSequence(shots, settings);
    REQUIRE(rangedFrames.size() == 2);
    CHECK(rangedFrames[0].position[0] == Catch::Approx(1.0F));
    CHECK(rangedFrames[1].position[0] == Catch::Approx(2.0F));
    CHECK(
        invisible_places::output::RenderFramePath(settings, 41).generic_string() ==
        "Saved/renders/Test/frame_000042.exr");
}

TEST_CASE("Animation render sequence evaluates animation paths directly", "[output][animation]") {
    invisible_places::camera::AnimationPath path;
    path.name = "Export Path";
    path.durationFrames = 60;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 0.0F, -1.0F}, .durationFrames = 30},
        {.cameraPosition = {2.0F, 0.0F, 0.0F}, .focusPoint = {2.0F, 0.0F, -1.0F}, .durationFrames = 30},
        {.cameraPosition = {6.0F, 3.0F, 0.0F}, .focusPoint = {6.0F, 3.0F, -1.0F}, .durationFrames = 30},
    };

    invisible_places::output::RenderJobSettings settings;
    settings.framesPerSecond = 30;
    const auto frames = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(frames.size() == 61);

    const auto middleEvaluation = invisible_places::camera::EvaluateAnimationPath(path, 1.0F);
    CHECK(frames[30].position[0] == Catch::Approx(middleEvaluation.camera.position[0]));
    CHECK(frames[30].position[1] == Catch::Approx(middleEvaluation.camera.position[1]));
    CHECK(frames[30].target[0] == Catch::Approx(middleEvaluation.camera.target[0]));
    CHECK(frames.back().position[0] == Catch::Approx(path.keys.back().cameraPosition[0]));
    CHECK(frames.back().position[1] == Catch::Approx(path.keys.back().cameraPosition[1]));

    settings.startFrame = 10;
    settings.endFrame = 12;
    const auto rangedFrames = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(rangedFrames.size() == 3);
    CHECK(rangedFrames.front().position[0] == Catch::Approx(frames[10].position[0]));
    CHECK(rangedFrames.back().position[0] == Catch::Approx(frames[12].position[0]));
}

TEST_CASE("Preview-density export point-size scale follows output viewport ratio", "[output][animation]") {
    CHECK(invisible_places::output::ComputePointSizePixelScale(1920, 1080, 1920, 1080) == Catch::Approx(1.0F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(3840, 2160, 1920, 1080) == Catch::Approx(2.0F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(960, 540, 1920, 1080) == Catch::Approx(0.5F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(1920, 1080, 0, 1080) == Catch::Approx(1.0F));
}

TEST_CASE("Fast preview MP4 ffmpeg command uses raw RGBA video input", "[output][video]") {
    const auto command = invisible_places::output::BuildFfmpegRawRgbaCommand(
        "/opt/homebrew/bin/ffmpeg",
        1920,
        1080,
        24,
        "/tmp/Invisible Places/it'll render.mp4");

    CHECK(command.find("'/opt/homebrew/bin/ffmpeg'") != std::string::npos);
    CHECK(command.find("-f rawvideo") != std::string::npos);
    CHECK(command.find("-pix_fmt rgba") != std::string::npos);
    CHECK(command.find("-s:v 1920x1080") != std::string::npos);
    CHECK(command.find("-r 24") != std::string::npos);
    CHECK(command.find("-i -") != std::string::npos);
    CHECK(command.find("-c:v libx264") != std::string::npos);
    CHECK(command.find("-pix_fmt yuv420p") != std::string::npos);
    CHECK(command.find("'/tmp/Invisible Places/it'\\''ll render.mp4'") != std::string::npos);
}

TEST_CASE("Fast preview MP4 converts half-float beauty frames to display RGBA8", "[output][video]") {
    invisible_places::output::HalfRgbaExrImage image;
    image.width = 1;
    image.height = 1;
    image.rgbaHalf = {
        0x3C00U,
        0x3800U,
        0x0000U,
        0x3C00U,
    };

    const auto bytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(image);
    REQUIRE(bytes.size() == 4);
    CHECK(bytes[0] == 255U);
    CHECK(bytes[1] == 188U);
    CHECK(bytes[2] == 0U);
    CHECK(bytes[3] == 255U);
}

TEST_CASE("EXR writer emits multichannel scanline files", "[output][exr]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_tiny.exr";

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 2, 1);
    image.beautyR[0] = 1.0F;
    image.beautyG[0] = 0.5F;
    image.beautyB[0] = 0.25F;
    image.alpha[0] = 1.0F;
    image.depth[0] = 7.0F;

    std::string errorMessage;
    REQUIRE(invisible_places::output::WriteExrImage(image, outputPath, &errorMessage));

    std::ifstream input{outputPath, std::ios::binary};
    REQUIRE(input.is_open());
    const std::vector<char> bytes{
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};
    REQUIRE(bytes.size() > 128);
    CHECK(static_cast<unsigned char>(bytes[0]) == 0x76U);
    CHECK(static_cast<unsigned char>(bytes[1]) == 0x2FU);
    CHECK(static_cast<unsigned char>(bytes[2]) == 0x31U);
    CHECK(static_cast<unsigned char>(bytes[3]) == 0x01U);
    const std::string header{bytes.begin(), bytes.end()};
    CHECK(header.find("beauty.R") != std::string::npos);
    CHECK(header.find("depth.Z") != std::string::npos);

    std::filesystem::remove(outputPath);
}

TEST_CASE("EXR writer accepts GPU half RGBA readback buffers", "[output][exr]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_gpu_half.exr";

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 1;
    image.height = 1;
    image.rgbaHalf = {
        0x3C00U,
        0x3800U,
        0x3400U,
        0x3C00U,
    };
    image.depth = {5.0F};

    std::string errorMessage;
    REQUIRE(invisible_places::output::WriteExrImage(image, outputPath, &errorMessage));

    std::ifstream input{outputPath, std::ios::binary};
    REQUIRE(input.is_open());
    const std::vector<char> bytes{
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};
    REQUIRE(bytes.size() > 128);
    const std::string header{bytes.begin(), bytes.end()};
    CHECK(header.find("beauty.R") != std::string::npos);
    CHECK(header.find("beauty.G") != std::string::npos);
    CHECK(header.find("beauty.B") != std::string::npos);
    CHECK(header.find("alpha.A") != std::string::npos);
    CHECK(header.find("depth.Z") != std::string::npos);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Offline point tiles match untiled output for deterministic scenes", "[output][offline]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {
        {0.0F, 0.0F, 0.0F},
        {0.2F, 0.0F, 0.0F},
        {-0.2F, 0.0F, 0.0F},
    };
    cloud.packedColors = {0xFF0000FFU, 0xFF00FF00U, 0xFFFF0000U};
    cloud.hasSourceRgb = true;

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
    invisible_places::style::SetScalarConstant(&style.pointSize, 1.0F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 2.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    invisible_places::output::ExrImage tiled;
    invisible_places::output::ExrImage untiled;
    invisible_places::output::InitializeExrImage(&tiled, 16, 16);
    invisible_places::output::InitializeExrImage(&untiled, 16, 16);

    for (const auto& tile : invisible_places::output::BuildOfflineRenderTiles(16, 16, 5)) {
        invisible_places::output::RenderPointCloudTile({layer}, cameraState, tile, &tiled);
    }
    for (const auto& tile : invisible_places::output::BuildOfflineRenderTiles(16, 16, 16)) {
        invisible_places::output::RenderPointCloudTile({layer}, cameraState, tile, &untiled);
    }

    REQUIRE(tiled.beautyR.size() == untiled.beautyR.size());
    for (std::size_t index = 0; index < tiled.beautyR.size(); ++index) {
        CHECK(tiled.beautyR[index] == Catch::Approx(untiled.beautyR[index]));
        CHECK(tiled.beautyG[index] == Catch::Approx(untiled.beautyG[index]));
        CHECK(tiled.beautyB[index] == Catch::Approx(untiled.beautyB[index]));
        CHECK(tiled.alpha[index] == Catch::Approx(untiled.alpha[index]));
        if (std::isfinite(tiled.depth[index]) || std::isfinite(untiled.depth[index])) {
            CHECK(tiled.depth[index] == Catch::Approx(untiled.depth[index]));
        }
    }
}

TEST_CASE("Offline world surfels use world diameter instead of pixel point size", "[output][offline][surfel]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFFFFFFFFU};
    cloud.hasSourceRgb = true;

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 0.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderWithPointSize = [&](float pointSize) {
        invisible_places::renderer::pointcloud::PointCloudStyleState style;
        style.geometryMode = invisible_places::renderer::pointcloud::PointCloudGeometryMode::WorldSurfels;
        style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
        invisible_places::style::SetScalarConstant(&style.pointSize, pointSize);
        invisible_places::style::SetScalarConstant(&style.surfelDiameter, 1.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);

        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .hasSourceRgb = true,
            .localToWorld = glm::mat4{1.0F},
        };

        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 32, 32);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 32, 32},
            &image);
        return image;
    };

    const auto smallPointSize = renderWithPointSize(1.0F);
    const auto largePointSize = renderWithPointSize(20.0F);
    REQUIRE(smallPointSize.alpha.size() == largePointSize.alpha.size());
    const auto coveredPixels =
        std::count_if(smallPointSize.alpha.begin(), smallPointSize.alpha.end(), [](float alpha) {
            return alpha > 0.0F;
        });
    CHECK(coveredPixels > 1);
    for (std::size_t index = 0; index < smallPointSize.alpha.size(); ++index) {
        CHECK(smallPointSize.alpha[index] == Catch::Approx(largePointSize.alpha[index]));
        if (std::isfinite(smallPointSize.depth[index]) || std::isfinite(largePointSize.depth[index])) {
            CHECK(smallPointSize.depth[index] == Catch::Approx(largePointSize.depth[index]));
        }
    }
}

TEST_CASE("Offline point renderer stacks opacity emission xray and falloff", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, -0.5F},
    };
    cloud.packedColors = {0xFF0000FFU, 0xFF00FF00U};
    cloud.hasSourceRgb = true;

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 2.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderWithStyle = [&](invisible_places::renderer::pointcloud::PointCloudStyleState style) {
        invisible_places::style::SetScalarConstant(&style.pointSize, 5.0F);
        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .hasSourceRgb = true,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 9, 9);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 9, 9},
            &image);
        return image;
    };

    invisible_places::renderer::pointcloud::PointCloudStyleState hard;
    hard.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    invisible_places::style::SetScalarConstant(&hard.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&hard.emissiveStrength, 2.0F);
    const auto hardImage = renderWithStyle(hard);
    const auto center = static_cast<std::size_t>(4) * 9U + 4U;
    CHECK(hardImage.beautyR[center] > 0.1F);
    CHECK(hardImage.alpha[center] > 0.1F);

    invisible_places::renderer::pointcloud::PointCloudStyleState gaussian;
    gaussian.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian;
    gaussian.gaussianSharpness = 6.0F;
    invisible_places::style::SetScalarConstant(&gaussian.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&gaussian.emissiveStrength, 2.0F);
    const auto gaussianImage = renderWithStyle(gaussian);
    const auto edge = static_cast<std::size_t>(4) * 9U + 6U;
    CHECK(gaussianImage.beautyR[center] > gaussianImage.beautyR[edge]);

    invisible_places::renderer::pointcloud::PointCloudStyleState stacked;
    stacked.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::SoftDisc;
    stacked.depthContribution =
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::Always;
    stacked.frontAlpha = 0.25F;
    stacked.hiddenAlpha = 0.18F;
    stacked.depthFalloff = 30.0F;
    invisible_places::style::SetScalarConstant(&stacked.opacity, 0.5F);
    invisible_places::style::SetScalarConstant(&stacked.xrayStrength, 1.0F);
    invisible_places::style::SetScalarConstant(&stacked.emissiveStrength, 1.5F);
    const auto stackedImage = renderWithStyle(stacked);
    CHECK(stackedImage.alpha[center] > 0.1F);
    CHECK(stackedImage.beautyR[center] > 0.01F);
    CHECK(stackedImage.beautyG[center] > 0.01F);
}

TEST_CASE("Offline point depth fade reduces alpha without changing color ratio", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFF0000FFU};
    cloud.hasSourceRgb = true;

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 2.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderWithDepthFade = [&](float depthFade) {
        invisible_places::renderer::pointcloud::PointCloudStyleState style;
        style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
        style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
        invisible_places::style::SetScalarConstant(&style.pointSize, 5.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
        invisible_places::style::SetScalarConstant(&style.depthFade, depthFade);

        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .hasSourceRgb = true,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 9, 9);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 9, 9},
            &image);
        return image;
    };

    const auto noFade = renderWithDepthFade(0.0F);
    const auto farFade = renderWithDepthFade(1.0F);
    const auto center = static_cast<std::size_t>(4) * 9U + 4U;
    REQUIRE(noFade.alpha[center] > 0.0F);
    REQUIRE(farFade.alpha[center] > 0.0F);
    CHECK(farFade.alpha[center] < noFade.alpha[center]);
    CHECK((farFade.beautyR[center] / farFade.alpha[center]) ==
          Catch::Approx(noFade.beautyR[center] / noFade.alpha[center]).margin(0.02F));
}

TEST_CASE("Offline point colourise recolours while preserving lightness", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFF000080U};
    cloud.hasSourceRgb = true;

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 2.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderWithColourise = [&](float amount) {
        invisible_places::renderer::pointcloud::PointCloudStyleState style;
        style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
        style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
        style.colorizeColor = {0.0F, 0.0F, 1.0F};
        style.colorizeAmount = amount;
        invisible_places::style::SetScalarConstant(&style.pointSize, 5.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);

        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .hasSourceRgb = true,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 9, 9);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 9, 9},
            &image);
        return image;
    };

    const auto unchanged = renderWithColourise(0.0F);
    const auto colourised = renderWithColourise(1.0F);
    const auto center = static_cast<std::size_t>(4) * 9U + 4U;
    REQUIRE(unchanged.alpha[center] > 0.0F);
    REQUIRE(colourised.alpha[center] > 0.0F);
    CHECK(unchanged.beautyR[center] > unchanged.beautyB[center]);
    CHECK(colourised.beautyB[center] > colourised.beautyR[center]);
    CHECK((colourised.beautyB[center] / colourised.alpha[center]) ==
          Catch::Approx(unchanged.beautyR[center] / unchanged.alpha[center]).margin(0.03F));
}

TEST_CASE("Offline point renderer uses safe defaults for inactive material bindings", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFF0000FFU};
    cloud.hasSourceRgb = true;
    cloud.scalarFields = {{"Value", 0.0F, 1.0F, 1U, true}};
    cloud.scalarFieldValues = {1.0F};

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 2.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderWithStyle = [&](invisible_places::renderer::pointcloud::PointCloudStyleState style,
                               invisible_places::output::OfflinePointRenderDiagnostics* diagnostics = nullptr) {
        style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
        invisible_places::style::SetScalarConstant(&style.pointSize, 5.0F);
        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .hasSourceRgb = true,
            .localToWorld = glm::mat4{1.0F},
        };
        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 9, 9);
        invisible_places::output::OfflinePointRenderScratch scratch;
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 9, 9},
            &image,
            diagnostics,
            &scratch);
        return image;
    };

    invisible_places::renderer::pointcloud::PointCloudStyleState baseline;
    invisible_places::style::SetScalarConstant(&baseline.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&baseline.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&baseline.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&baseline.depthFade, 0.0F);
    const auto baselineImage = renderWithStyle(baseline);

    invisible_places::renderer::pointcloud::PointCloudStyleState inactive = baseline;
    invisible_places::style::SetScalarConstant(&inactive.opacity, 0.0F);
    invisible_places::style::SetScalarConstant(&inactive.emissiveStrength, 8.0F);
    invisible_places::style::SetScalarConstant(&inactive.xrayStrength, 1.0F);
    invisible_places::style::SetScalarConstant(&inactive.depthFade, 1.0F);
    inactive.opacity.active = false;
    inactive.emissiveStrength.active = false;
    inactive.xrayStrength.active = false;
    inactive.depthFade.active = false;
    invisible_places::output::OfflinePointRenderDiagnostics diagnostics;
    const auto inactiveImage = renderWithStyle(inactive, &diagnostics);

    const auto center = static_cast<std::size_t>(4) * 9U + 4U;
    CHECK(inactiveImage.alpha[center] == Catch::Approx(baselineImage.alpha[center]));
    CHECK(inactiveImage.beautyR[center] == Catch::Approx(baselineImage.beautyR[center]));
    CHECK(diagnostics.skippedInactiveBindings >= 4U);

    invisible_places::renderer::pointcloud::PointCloudStyleState colormapDefault;
    colormapDefault.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap;
    invisible_places::style::SetScalarConstant(&colormapDefault.colormapPosition, 0.5F);
    const auto defaultColormapImage = renderWithStyle(colormapDefault);

    invisible_places::renderer::pointcloud::PointCloudStyleState inactiveColormap = colormapDefault;
    invisible_places::style::ConfigureFieldMapFromStats(
        &inactiveColormap.colormapPosition,
        0,
        "Value",
        0.0F,
        1.0F,
        &cloud.scalarFields.front());
    inactiveColormap.colormapPosition.active = false;
    const auto inactiveColormapImage = renderWithStyle(inactiveColormap);
    CHECK(inactiveColormapImage.beautyR[center] == Catch::Approx(defaultColormapImage.beautyR[center]));
    CHECK(inactiveColormapImage.beautyG[center] == Catch::Approx(defaultColormapImage.beautyG[center]));
    CHECK(inactiveColormapImage.beautyB[center] == Catch::Approx(defaultColormapImage.beautyB[center]));
}

TEST_CASE("Offline point diagnostics skip depth pass for non-depth layers", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFFFFFFFFU};
    cloud.hasSourceRgb = true;

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.depthContribution = invisible_places::renderer::pointcloud::PointCloudDepthContribution::None;
    invisible_places::style::SetScalarConstant(&style.pointSize, 5.0F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 2.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };
    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 9, 9);
    invisible_places::output::OfflinePointRenderDiagnostics diagnostics;
    invisible_places::output::OfflinePointRenderScratch scratch;
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0, 0, 9, 9},
        &image,
        &diagnostics,
        &scratch);

    CHECK(diagnostics.depthPassLayers == 0U);
    CHECK(diagnostics.depthVisitedPoints == 0U);
    CHECK(diagnostics.accumulationPassLayers == 1U);
    CHECK(diagnostics.accumulationVisitedPoints == 1U);

    auto explicitDepthStyle = style;
    explicitDepthStyle.depthContribution =
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::Always;
    const invisible_places::output::OfflinePointLayer explicitDepthLayer{
        .cloud = &cloud,
        .style = explicitDepthStyle,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };
    invisible_places::output::InitializeExrImage(&image, 9, 9);
    invisible_places::output::RenderPointCloudTile(
        {explicitDepthLayer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0, 0, 9, 9},
        &image,
        &diagnostics,
        &scratch);

    CHECK(diagnostics.depthPassLayers == 0U);
    CHECK(diagnostics.depthVisitedPoints == 0U);

    invisible_places::style::SetScalarConstant(&explicitDepthStyle.xrayStrength, 0.25F);
    const invisible_places::output::OfflinePointLayer xrayDepthLayer{
        .cloud = &cloud,
        .style = explicitDepthStyle,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };
    invisible_places::output::InitializeExrImage(&image, 9, 9);
    invisible_places::output::RenderPointCloudTile(
        {xrayDepthLayer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0, 0, 9, 9},
        &image,
        &diagnostics,
        &scratch);

    CHECK(diagnostics.depthPassLayers == 1U);
    CHECK(diagnostics.depthVisitedPoints == 1U);
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
