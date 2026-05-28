#include "InvisiblePlacesBuildConfig.hpp"
#include "app/PointVisualSelection.hpp"
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
#include "output/EyeDomeLighting.hpp"
#include "output/HoudiniCameraExport.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "output/RenderPreset.hpp"
#include "output/VideoWriter.hpp"
#include "platform/Window.hpp"
#include "platform/WindowTitle.hpp"
#include "platform/VulkanRuntimeConfig.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/gsplat/HighQualityGaussianScene.hpp"
#include "renderer/pointcloud/Colormap.hpp"
#include "renderer/pointcloud/PointCloudIpcloudCache.hpp"
#include "renderer/pointcloud/PointCloudGpuDrivenSelection.hpp"
#include "renderer/pointcloud/PointCloudLodHierarchy.hpp"
#include "renderer/pointcloud/PointCloudPerformanceGovernor.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "serialization/ProjectDocument.hpp"
#include "style/RenderParameterBinding.hpp"
#include "water/WaterFlow.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Imath/half.h>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

std::filesystem::path DataRoot() {
    return std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
}

std::filesystem::path RepoRoot() {
    const std::filesystem::path dataRootParent = DataRoot().parent_path();
    if (std::filesystem::exists(dataRootParent / "src/app/Application.cpp")) {
        return dataRootParent;
    }

    for (std::filesystem::path candidate = std::filesystem::current_path();
         !candidate.empty();
         candidate = candidate.parent_path()) {
        if (std::filesystem::exists(candidate / "src/app/Application.cpp")) {
            return candidate;
        }
        if (candidate == candidate.root_path()) {
            break;
        }
    }

    throw std::runtime_error{"Could not locate repository root for source-level regression tests."};
}

std::string ReadRepoTextFile(const std::filesystem::path& relativePath) {
    std::ifstream input{RepoRoot() / relativePath};
    if (!input) {
        throw std::runtime_error{"Could not read repo file: " + relativePath.generic_string()};
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string SourceSection(
    const std::string& source,
    std::string_view beginNeedle,
    std::string_view endNeedle) {
    const auto begin = source.find(beginNeedle);
    REQUIRE(begin != std::string::npos);
    const auto end = source.find(endNeedle, begin + beginNeedle.size());
    REQUIRE(end != std::string::npos);
    return source.substr(begin, end - begin);
}

std::filesystem::path FindDataFileByName(std::string_view filename) {
    const auto root = DataRoot();
    const auto directPath = root / std::string{filename};
    if (std::filesystem::exists(directPath)) {
        return directPath;
    }

    if (!std::filesystem::exists(root)) {
        return {};
    }

    for (const auto& entry : std::filesystem::recursive_directory_iterator{
             root,
             std::filesystem::directory_options::skip_permission_denied}) {
        if (entry.is_regular_file() && entry.path().filename().string() == filename) {
            return entry.path();
        }
    }

    return {};
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

invisible_places::io::LoadedPointCloud MakeRippleFixtureCloud() {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.hasNormals = true;
    cloud.hasSourceRgb = true;
    constexpr std::uint32_t gridSize = 17U;
    cloud.positions.reserve(gridSize * gridSize);
    cloud.normals.reserve(gridSize * gridSize);
    cloud.packedColors.reserve(gridSize * gridSize);
    for (std::uint32_t y = 0; y < gridSize; ++y) {
        for (std::uint32_t x = 0; x < gridSize; ++x) {
            const float px = static_cast<float>(x) / static_cast<float>(gridSize - 1U);
            const float py = static_cast<float>(y) / static_cast<float>(gridSize - 1U);
            cloud.positions.push_back({px, py, 0.0F});
            const bool sloped = px > 0.50F;
            cloud.normals.push_back(sloped ? invisible_places::io::Float3{0.72F, 0.0F, 0.69F}
                                           : invisible_places::io::Float3{0.0F, 0.0F, 1.0F});
            cloud.packedColors.push_back(0xffffffffU);
            cloud.bounds.Expand(cloud.positions.back());
        }
    }
    cloud.focusPoint = {0.5F, 0.5F, 0.0F};
    cloud.hasFocusPoint = true;
    cloud.sourcePath = "Data/RippleFixture.ply";
    cloud.layerName = "Ripple fixture";
    return cloud;
}

invisible_places::water::WaterEffectLayer MakeRippleTestLayer(
    invisible_places::water::WaterRippleOverlayType type,
    std::uint32_t id = 1U) {
    invisible_places::water::WaterEffectLayer layer;
    layer.id = id;
    layer.name = "test ripple";
    layer.featureType = invisible_places::water::WaterEffectFeatureType::Ripple;
    layer.rippleOverlayType = type;
    layer.vertices = {
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        {1.0F, 1.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
    };
    layer.hull = invisible_places::water::BuildWaterRegionHull(layer.vertices);
    layer.edgeBlendWidth = 0.08F;
    layer.regionStrength = 1.0F;
    layer.patternScale = 1.0F;
    layer.speed = 0.55F;
    layer.wavelengthMeters = 0.16F;
    layer.warp = 0.45F;
    layer.turbulence = 0.0F;
    layer.phase = 0.125F;
    layer.directionX = 1.0F;
    layer.directionY = 0.0F;
    layer.directionZ = 0.0F;
    layer.seed = 77U;
    layer.maxAffectedPoints = 4096U;
    layer.response.intensity = 1.0F;
    layer.response.emissionAdd = 1.0F;
    layer.response.opacityAdd = 0.25F;
    return layer;
}

std::optional<float> RippleValueAt(
    const invisible_places::water::WaterEffectOverlay& overlay,
    float x,
    float y) {
    for (const auto& point : overlay.points) {
        if (std::abs(point.position.x - x) <= 1.0e-5F &&
            std::abs(point.position.y - y) <= 1.0e-5F) {
            return point.value;
        }
    }
    return std::nullopt;
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
    const std::filesystem::path candidates[] = {
        DataRoot() / "Site2 -5mm.ply",
        DataRoot() / "Site3-Sample-Terrestrial.ply",
    };
    const auto candidateIt = std::find_if(
        std::begin(candidates),
        std::end(candidates),
        [](const std::filesystem::path& path) {
            return std::filesystem::exists(path);
        });
    if (candidateIt == std::end(candidates)) {
        SKIP("No CloudCompare point-cloud fixture is available in the local Data directory.");
    }

    const auto result = invisible_places::io::ParsePlyHeader(*candidateIt);

    REQUIRE(result.success);
    CHECK(result.header.LooksLikePointCloud());
    CHECK(result.header.HasColorRgb());

    const auto scalarFields = result.header.ScalarFieldNames();
    CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Height") != scalarFields.end());
    CHECK(std::find(scalarFields.begin(), scalarFields.end(), "GroundOffset") != scalarFields.end());
}

TEST_CASE("Gaussian splat assets expose the expected transform pairing", "[ply][gsplat]") {
    const auto plyPath = FindDataFileByName("gSplat-Site3-1.ply");
    const auto matrixPath = FindDataFileByName("gSplat-Site3-1.txt");
    if (plyPath.empty() || matrixPath.empty()) {
        SKIP("No Gaussian splat fixture is available in the local Data directory.");
    }

    const auto headerResult = invisible_places::io::ParsePlyHeader(plyPath);
    const auto matrixResult = invisible_places::io::ParseMatrix4x4(matrixPath);

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

TEST_CASE("Initial window size clamps only displays larger than 1080p", "[window]") {
    using invisible_places::platform::ResolveInitialWindowSizeForScreen;
    using invisible_places::platform::WindowSize;

    const WindowSize usual{.width = 1440, .height = 900};

    const auto large = ResolveInitialWindowSizeForScreen(2560, 1440, usual);
    CHECK(large.width == 1920);
    CHECK(large.height == 1080);

    const auto exact1080p = ResolveInitialWindowSizeForScreen(1920, 1080, usual);
    CHECK(exact1080p.width == 1440);
    CHECK(exact1080p.height == 900);

    const auto wideButNotTall = ResolveInitialWindowSizeForScreen(2560, 1080, usual);
    CHECK(wideButNotTall.width == 1440);
    CHECK(wideButNotTall.height == 900);
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

TEST_CASE("Orbit camera keeps repeated zoom-out wheel steps controlled", "[camera][zoom]") {
    invisible_places::io::Bounds3f bounds;
    bounds.Expand({-1.0F, -1.0F, -1.0F});
    bounds.Expand({1.0F, 1.0F, 1.0F});

    invisible_places::camera::OrbitCamera camera;
    camera.FrameBounds(bounds, 1.0F);

    std::vector<float> zoomOutSteps;
    for (int index = 0; index < 10; ++index) {
        const float beforeDistance = camera.Distance();
        camera.Dolly(-1.0F);
        const float afterDistance = camera.Distance();
        zoomOutSteps.push_back(afterDistance - beforeDistance);
    }

    REQUIRE(zoomOutSteps.size() == 10);
    CHECK(zoomOutSteps.front() > 0.0F);
    CHECK(zoomOutSteps.back() < zoomOutSteps.front() * 2.0F);

    const float zoomedOutDistance = camera.Distance();
    camera.Dolly(1.0F);
    CHECK(camera.Distance() < zoomedOutDistance);
}

TEST_CASE("Manual point budget sampling is an explicit draw cap", "[budget]") {
    using invisible_places::renderer::pointcloud::ClampPointBudget;
    using invisible_places::renderer::pointcloud::MakePointBudgetState;

    const auto fullSourceBudget = MakePointBudgetState(42'000, 42'000);
    CHECK(fullSourceBudget.totalPoints == 42'000);
    CHECK(fullSourceBudget.activePoints == 42'000);
    CHECK(!fullSourceBudget.UsesSampledIndices());
    CHECK(fullSourceBudget.sampledIndices.empty());

    const auto userCappedBudget = MakePointBudgetState(42'000, 4'000);
    CHECK(userCappedBudget.totalPoints == 42'000);
    CHECK(userCappedBudget.activePoints == 4'000);
    CHECK(userCappedBudget.UsesSampledIndices());
    CHECK(userCappedBudget.sampledIndices.size() == 4'000);
    CHECK(ClampPointBudget(42'000, 99'000) == 42'000);
}

TEST_CASE("Point-cloud LOD hierarchy preserves source indices for full-source traversal", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    for (int z = 0; z < 2; ++z) {
        for (int y = 0; y < 2; ++y) {
            for (int x = 0; x < 2; ++x) {
                invisible_places::io::Float3 point{
                    static_cast<float>(x) - 0.5F,
                    static_cast<float>(y) - 0.5F,
                    static_cast<float>(z) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 4U});
    REQUIRE(!hierarchy.Empty());
    CHECK(hierarchy.sourcePointCount == cloud.PointCount());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::FullSource;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    const auto drawItems = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);

    REQUIRE(drawItems.size() == cloud.PointCount());
    std::set<std::uint32_t> sourceIndices;
    std::uint32_t representedSourceCount = 0;
    for (const auto& drawItem : drawItems) {
        sourceIndices.insert(drawItem.sourcePointIndex);
        representedSourceCount += drawItem.representedSourceCount;
        CHECK(drawItem.representedSourceCount == 1U);
    }
    CHECK(sourceIndices.size() == cloud.PointCount());
    CHECK(representedSourceCount == cloud.PointCount());
}

TEST_CASE("Point-cloud adaptive LOD traversal uses area-normalized compensation", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    for (int z = 0; z < 2; ++z) {
        for (int y = 0; y < 2; ++y) {
            for (int x = 0; x < 2; ++x) {
                invisible_places::io::Float3 point{
                    static_cast<float>(x) - 0.5F,
                    static_cast<float>(y) - 0.5F,
                    static_cast<float>(z) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 4U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::FastAdaptivePreview;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxDrawItems = 1U;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 7.0F);
    const auto drawItems = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);

    REQUIRE(drawItems.size() == 1);
    CHECK(drawItems[0].sourcePointIndex < cloud.PointCount());
    CHECK(drawItems[0].representedSourceCount == cloud.PointCount());
    CHECK(drawItems[0].footprintAreaPixels > 7.0F * 7.0F);
    CHECK(std::sqrt(drawItems[0].footprintAreaPixels) <= Catch::Approx(48.0F));
    CHECK(drawItems[0].renderAreaPixels == Catch::Approx(7.0F * 7.0F));
    CHECK(drawItems[0].opacityCompensation <= 1.5F);
    CHECK(drawItems[0].emissionCompensation <= 1.0F);
    CHECK(drawItems[0].opacityCompensation != Catch::Approx(std::sqrt(static_cast<float>(cloud.PointCount()))));
}

TEST_CASE("Point-cloud adaptive LOD visible size only grows for overlapping source density", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 10;
    constexpr float extent = 0.15F;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    ((static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F) * extent};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxDrawItems = 1U;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 8.0F);

    const auto drawItems = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    REQUIRE(drawItems.size() == 1U);
    CHECK(drawItems.front().representedSourceCount == cloud.PointCount());
    CHECK(drawItems.front().footprintAreaPixels > 8.0F * 8.0F);
    CHECK(drawItems.front().renderAreaPixels > 8.0F * 8.0F);
    CHECK(std::sqrt(drawItems.front().renderAreaPixels) <= Catch::Approx(8.0F * 1.35F));
    CHECK(drawItems.front().renderAreaPixels < drawItems.front().footprintAreaPixels);
}

TEST_CASE("Beauty adaptive LOD opacity uses optical-depth compensation", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 6;
    constexpr float extent = 0.02F;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    ((static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F) * extent};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxDrawItems = 1U;
    params.maxRepresentativeDiameterPixels = 256.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 3.0F);
    invisible_places::style::SetScalarConstant(&params.style.opacity, 0.25F);

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(drawItems.size() == 1U);
    const auto& drawItem = drawItems.front();
    const float representedArea =
        3.0F * 3.0F * static_cast<float>(drawItem.representedSourceCount);
    const float linearAreaScale = representedArea / std::max(1.0F, drawItem.footprintAreaPixels);

    CHECK(linearAreaScale > 8.0F);
    CHECK(drawItem.opacityCompensation <= 4.05F);
    CHECK(drawItem.opacityCompensation < linearAreaScale * 0.5F);
    CHECK(drawItem.emissionCompensation == Catch::Approx(1.0F));
    CHECK(diagnostics.rendererCostProfile == pc::PointCloudLodRendererCostProfile::BeautyScreenSprite);
    CHECK(diagnostics.maxOpacityCoverageScale <= 4.05F);
    CHECK(diagnostics.estimatedBlendedFragmentCost > 0.0F);
}

TEST_CASE("Beauty adaptive LOD footprint grows render radius in the right direction", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 10;
    constexpr float extent = 0.15F;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    ((static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F) * extent};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxDrawItems = 1U;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 8.0F);

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(drawItems.size() == 1U);
    CHECK(drawItems.front().renderAreaPixels > 8.0F * 8.0F);
    CHECK(drawItems.front().renderAreaPixels < drawItems.front().footprintAreaPixels);
    CHECK(diagnostics.maxRadiusScale > 1.0F);
    CHECK(diagnostics.performanceCompensationClamped == false);
}

TEST_CASE("Beauty adaptive LOD preserves ordinary emission and caps emissive accents", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    cloud.scalarFields.push_back({.name = "Interest"});
    constexpr int gridSize = 4;
    constexpr float extent = 0.02F;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                const auto pointIndex = static_cast<std::uint32_t>(cloud.positions.size());
                invisible_places::io::Float3 point{
                    ((static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F) * extent,
                    ((static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F) * extent};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
                const float interest = pointIndex == 42U ? 10.0F : 0.25F;
                cloud.scalarFieldValues.push_back(interest);
                cloud.scalarFields.front().Include(interest);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U, .maxInternalRepresentatives = 12U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 12U;
    params.maxEstimatedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    params.style.colorMode = pc::PointCloudColorMode::ScalarColormap;
    params.style.emissiveStrength.mode = invisible_places::style::ParameterSourceMode::FieldMapped;
    params.style.emissiveStrength.fieldMap.fieldSlot = 0;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 5.0F);
    invisible_places::style::SetScalarConstant(&params.style.opacity, 0.45F);

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(!drawItems.empty());
    REQUIRE(diagnostics.emittedClassCounts[6] > 0U);

    bool sawAccent = false;
    bool sawOrdinaryAverage = false;
    for (const auto& drawItem : drawItems) {
        if (drawItem.sourcePointIndex == 42U) {
            sawAccent = true;
            CHECK(drawItem.emissionCompensation > 1.0F);
            CHECK(drawItem.emissionCompensation <= 2.5F);
        } else {
            sawOrdinaryAverage = sawOrdinaryAverage || drawItem.emissionCompensation == Catch::Approx(1.0F);
        }
    }

    CHECK(sawAccent);
    CHECK(sawOrdinaryAverage);
    CHECK(diagnostics.maxEmissionCoverageScale <= 2.5F);
}

TEST_CASE("Beauty adaptive LOD estimates sprite and surfel renderer costs separately", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 5;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams spriteParams;
    spriteParams.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    spriteParams.densityMode = invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality;
    spriteParams.viewportWidth = 1024;
    spriteParams.viewportHeight = 1024;
    spriteParams.maxDrawItems = 8U;
    spriteParams.maxEstimatedFragments = 1.0e9F;
    invisible_places::style::SetScalarConstant(&spriteParams.style.pointSize, 6.0F);

    pc::PointCloudLodTraversalDiagnostics spriteDiagnostics;
    const auto spriteItems =
        pc::TraversePointCloudLodHierarchy(hierarchy, spriteParams, {}, &spriteDiagnostics);
    REQUIRE(!spriteItems.empty());

    auto surfelParams = spriteParams;
    surfelParams.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyWorldSurfel;
    surfelParams.style.geometryMode = pc::PointCloudGeometryMode::WorldSurfels;
    invisible_places::style::SetScalarConstant(&surfelParams.style.surfelDiameter, 0.01F);

    pc::PointCloudLodTraversalDiagnostics surfelDiagnostics;
    const auto surfelItems =
        pc::TraversePointCloudLodHierarchy(hierarchy, surfelParams, {}, &surfelDiagnostics);
    REQUIRE(!surfelItems.empty());

    CHECK(spriteDiagnostics.rendererCostProfile == pc::PointCloudLodRendererCostProfile::BeautyScreenSprite);
    CHECK(surfelDiagnostics.rendererCostProfile == pc::PointCloudLodRendererCostProfile::BeautyWorldSurfel);
    CHECK(spriteDiagnostics.estimatedVertexCost == Catch::Approx(static_cast<float>(spriteItems.size())));
    CHECK(surfelDiagnostics.estimatedVertexCost == Catch::Approx(static_cast<float>(surfelItems.size()) * 6.0F));
    CHECK(surfelDiagnostics.estimatedVertexCost > spriteDiagnostics.estimatedVertexCost);
    CHECK(spriteDiagnostics.estimatedBlendedFragmentCost > 0.0F);
    CHECK(surfelDiagnostics.estimatedBlendedFragmentCost > 0.0F);
}

TEST_CASE("Beauty adaptive LOD preserves source coverage under fragment pressure", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.targetProjectedSpacingPixels = 8.0F;
    params.maxRepresentativeDiameterPixels = 64.0F;
    params.maxRepresentatives = 4096U;
    params.maxEstimatedFragments = 64.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 32.0F);
    invisible_places::style::SetScalarConstant(&params.style.opacity, 0.25F);

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(drawItems.size() > 1U);
    CHECK(diagnostics.fragmentBudgetReached);
    CHECK(diagnostics.emittedRepresentedSourceCount == cloud.PointCount());
    CHECK(diagnostics.visibleFrontierRepresentedSourceCount == cloud.PointCount());
}

TEST_CASE("Point-cloud LOD internal nodes store distributed representatives", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 4;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U, .maxInternalRepresentatives = 16U});
    REQUIRE(!hierarchy.Empty());
    REQUIRE(!hierarchy.nodes.front().IsLeaf());
    const auto& root = hierarchy.nodes.front();
    CHECK(root.representativeCount > 1U);
    CHECK(root.representativeCount <= 16U);

    std::set<std::uint32_t> representativeSourceIndices;
    std::uint32_t representedSourceCount = 0;
    for (std::uint32_t offset = 0; offset < root.representativeCount; ++offset) {
        const auto representativeIndex = root.firstRepresentative + offset;
        REQUIRE(representativeIndex < hierarchy.representatives.size());
        const auto& representative = hierarchy.representatives[representativeIndex];
        representativeSourceIndices.insert(representative.sourcePointIndex);
        representedSourceCount += representative.representedSourceCount;
    }
    CHECK(representativeSourceIndices.size() == root.representativeCount);
    CHECK(representedSourceCount == cloud.PointCount());
}

TEST_CASE("Point-cloud LOD stores visual stats and emits feature representatives", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    auto packColor = [](std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
        return static_cast<std::uint32_t>(red) |
               (static_cast<std::uint32_t>(green) << 8U) |
               (static_cast<std::uint32_t>(blue) << 16U) |
               (0xffU << 24U);
    };

    invisible_places::io::LoadedPointCloud cloud;
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    cloud.scalarFields.push_back({.name = "Interest"});

    constexpr int gridSize = 4;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                const auto pointIndex = static_cast<std::uint32_t>(cloud.positions.size());
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
                cloud.packedColors.push_back(packColor(96, 96, 96));
                cloud.normals.push_back({0.0F, 0.0F, 1.0F});
                float interest = 0.25F;
                if (pointIndex == 5U) {
                    cloud.packedColors.back() = packColor(255, 16, 16);
                }
                if (pointIndex == 17U) {
                    cloud.normals.back() = {1.0F, 0.0F, 0.0F};
                }
                if (pointIndex == 42U) {
                    interest = 10.0F;
                    cloud.packedColors.back() = packColor(255, 240, 80);
                }
                cloud.scalarFieldValues.push_back(interest);
                cloud.scalarFields.front().Include(interest);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U, .maxInternalRepresentatives = 12U});
    REQUIRE(!hierarchy.Empty());
    REQUIRE(hierarchy.scalarFieldCount == 1U);
    REQUIRE(hierarchy.nodeScalarStats.size() == hierarchy.nodes.size());
    const auto& root = hierarchy.nodes.front();
    CHECK(root.spacingMeters > 0.0F);
    CHECK(root.densityPointsPerM3 > 0.0F);
    CHECK(root.colorContrast > 0.0F);
    CHECK(root.normalVariance > 0.0F);
    CHECK(root.scalarRangeHint > 0.0F);
    CHECK(root.emissiveImportanceHint > 0.0F);
    CHECK((root.featureFlags & pc::PointCloudLodRepresentativeClassColorContrast) != 0U);
    CHECK((root.featureFlags & pc::PointCloudLodRepresentativeClassNormalEdge) != 0U);
    CHECK((root.featureFlags & pc::PointCloudLodRepresentativeClassEmissiveAccent) != 0U);

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 12U;
    params.maxEstimatedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    params.style.colorMode = pc::PointCloudColorMode::ScalarColormap;
    params.style.colormapPosition.mode = invisible_places::style::ParameterSourceMode::FieldMapped;
    params.style.colormapPosition.fieldMap.fieldSlot = 0;
    params.style.emissiveStrength.mode = invisible_places::style::ParameterSourceMode::FieldMapped;
    params.style.emissiveStrength.fieldMap.fieldSlot = 0;

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(!drawItems.empty());
    CHECK(diagnostics.emittedClassCounts[1] > 0U);
    CHECK(diagnostics.emittedClassCounts[2] > 0U);
    CHECK(diagnostics.emittedClassCounts[4] > 0U);
    CHECK(diagnostics.emittedClassCounts[6] > 0U);

    std::unordered_set<std::uint32_t> emittedSourceIndices;
    for (const auto& drawItem : drawItems) {
        emittedSourceIndices.insert(drawItem.sourcePointIndex);
    }
    CHECK(emittedSourceIndices.contains(42U));
    CHECK(std::all_of(
        drawItems.begin(),
        drawItems.end(),
        [&](const pc::PointCloudDrawItemGpu& drawItem) {
            return drawItem.sourcePointIndex < cloud.PointCount();
        }));
}

TEST_CASE("Point-cloud adaptive LOD does not collapse large projected nodes to one tiny point", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 16U;
    params.maxEstimatedFragments = 0.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 7.0F);

    const auto drawItems = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    REQUIRE(drawItems.size() > 1U);
    CHECK(drawItems.size() <= params.maxRepresentatives);
    const auto representedSourceCount = std::accumulate(
        drawItems.begin(),
        drawItems.end(),
        std::uint64_t{0},
        [](std::uint64_t total, const invisible_places::renderer::pointcloud::PointCloudDrawItemGpu& drawItem) {
            return total + drawItem.representedSourceCount;
        });
    CHECK(representedSourceCount == cloud.PointCount());

    bool foundAggregatedRepresentative = false;
    for (const auto& drawItem : drawItems) {
        foundAggregatedRepresentative = foundAggregatedRepresentative || drawItem.representedSourceCount > 1U;
        CHECK(drawItem.footprintAreaPixels > 0.0F);
        CHECK(std::sqrt(drawItem.footprintAreaPixels) <= Catch::Approx(32.0F));
        CHECK(drawItem.renderAreaPixels == Catch::Approx(7.0F * 7.0F));
        CHECK(drawItem.opacityCompensation <= 4.0F);
        CHECK(drawItem.emissionCompensation <= 1.25F);
        if (drawItem.representedSourceCount > 1U) {
            CHECK(drawItem.footprintAreaPixels > 7.0F * 7.0F);
            CHECK(drawItem.renderAreaPixels <= drawItem.footprintAreaPixels);
        }
    }
    CHECK(foundAggregatedRepresentative);
}

TEST_CASE("Point-cloud LOD hierarchy cache round-trips and rejects stale metadata", "[pointcloud][lod][cache]") {
    namespace pc = invisible_places::renderer::pointcloud;

    auto cloud = MakeRippleFixtureCloud();
    const auto tempRoot =
        std::filesystem::temp_directory_path() /
        ("InvisiblePlacesLodCacheTest-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(tempRoot);
    const auto sourcePath = tempRoot / "fixture.ply";
    {
        std::ofstream source{sourcePath};
        source << "ply\nformat ascii 1.0\ncomment cache fixture\nend_header\n";
    }
    cloud.sourcePath = sourcePath;

    const pc::PointCloudLodBuildConfig config{
        .maxLeafSourcePoints = 8U,
        .maxDepth = 5U,
        .maxInternalRepresentatives = 12U};
    const auto hierarchy = pc::BuildPointCloudLodHierarchy(cloud, config);
    REQUIRE(!hierarchy.Empty());

    const auto source = pc::MakePointCloudLodCacheSource(sourcePath, cloud);
    const auto cachePath = pc::BuildPointCloudLodCachePath(tempRoot / "lod", sourcePath);
    CHECK(cachePath.filename().string().find("PointCloudLodCache-v4.bin") != std::string::npos);
    const auto relativeSourcePath = std::filesystem::relative(sourcePath);
    const auto relativeSource = pc::MakePointCloudLodCacheSource(relativeSourcePath, cloud);
    const auto relativeCachePath = pc::BuildPointCloudLodCachePath(tempRoot / "lod", relativeSourcePath);
    CHECK(relativeSource.normalizedPathHash == source.normalizedPathHash);
    CHECK(relativeSource.sourceSizeBytes == source.sourceSizeBytes);
    CHECK(relativeSource.sourceWriteTimeNs == source.sourceWriteTimeNs);
    CHECK(relativeCachePath.filename() == cachePath.filename());
    std::string errorMessage;
    REQUIRE(pc::SavePointCloudLodHierarchyCache(cachePath, source, config, hierarchy, &errorMessage));

    auto loaded = pc::LoadPointCloudLodHierarchyCache(cachePath, source, config);
    REQUIRE(loaded.loaded);
    CHECK(!loaded.stale);
    CHECK(loaded.hierarchy.sourcePointCount == hierarchy.sourcePointCount);
    CHECK(loaded.hierarchy.nodes.size() == hierarchy.nodes.size());
    CHECK(loaded.hierarchy.representatives.size() == hierarchy.representatives.size());
    REQUIRE(!loaded.hierarchy.nodes.empty());
    CHECK(loaded.hierarchy.nodes.front().representedSourceCount == hierarchy.nodes.front().representedSourceCount);

    auto wrongPointCount = source;
    ++wrongPointCount.pointCount;
    auto stale = pc::LoadPointCloudLodHierarchyCache(cachePath, wrongPointCount, config);
    CHECK(!stale.loaded);
    CHECK(stale.stale);

    auto wrongConfig = config;
    ++wrongConfig.maxDepth;
    stale = pc::LoadPointCloudLodHierarchyCache(cachePath, source, wrongConfig);
    CHECK(!stale.loaded);
    CHECK(stale.stale);

    wrongConfig = config;
    ++wrongConfig.maxInternalRepresentatives;
    stale = pc::LoadPointCloudLodHierarchyCache(cachePath, source, wrongConfig);
    CHECK(!stale.loaded);
    CHECK(stale.stale);

    {
        std::fstream cache{cachePath, std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(cache);
        const std::uint32_t wrongVersion = 3U;
        cache.seekp(static_cast<std::streamoff>(sizeof(std::uint64_t)));
        cache.write(reinterpret_cast<const char*>(&wrongVersion), sizeof(wrongVersion));
    }
    stale = pc::LoadPointCloudLodHierarchyCache(cachePath, source, config);
    CHECK(!stale.loaded);
    CHECK(stale.stale);
    CHECK(stale.message.find("version mismatch") != std::string::npos);

    std::filesystem::remove_all(tempRoot);
}

TEST_CASE("Point-cloud .ipcloud bundle publishes atomically and validates manifest", "[pointcloud][lod][cache]") {
    namespace pc = invisible_places::renderer::pointcloud;

    const auto sourcePath = WriteTinyBinaryPointCloudFixture();
    auto loadResult = invisible_places::io::LoadPointCloud(sourcePath);
    REQUIRE(loadResult.success);
    auto sourceInfo = pc::MakePointCloudIpcloudSourceInfo(sourcePath);
    REQUIRE(sourceInfo.has_value());

    const pc::PointCloudLodBuildConfig config{
        .maxLeafSourcePoints = 2U,
        .maxDepth = 4U,
        .maxInternalRepresentatives = 8U};
    const auto hierarchy = pc::BuildPointCloudLodHierarchy(loadResult.cloud, config);
    REQUIRE(!hierarchy.Empty());

    const auto tempRoot =
        std::filesystem::temp_directory_path() /
        ("InvisiblePlacesIpcloudTest-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto bundlePath = pc::BuildPointCloudIpcloudBundlePath(tempRoot / "PointCloudCache", sourceInfo.value());
    auto partialPreview = pc::LoadPointCloudIpcloudSourcePreview(sourcePath, 2U);
    REQUIRE(partialPreview.loaded);
    auto partialHierarchy = pc::BuildPointCloudLodHierarchy(partialPreview.cloud, config);
    REQUIRE(!partialHierarchy.Empty());
    const auto partialBundlePath = bundlePath.parent_path() / "partial-preview-bundle";
    const auto partialSaveResult =
        pc::SavePointCloudIpcloudBundle(
            partialBundlePath,
            sourceInfo.value(),
            config,
            partialPreview.cloud,
            partialHierarchy);
    CHECK(!partialSaveResult.saved);
    CHECK(partialSaveResult.errorMessage.find("source point count mismatch") != std::string::npos);
    CHECK(!std::filesystem::exists(partialBundlePath));

    const auto saveResult =
        pc::SavePointCloudIpcloudBundle(bundlePath, sourceInfo.value(), config, loadResult.cloud, hierarchy);
    REQUIRE(saveResult.saved);
    CHECK(std::filesystem::is_directory(bundlePath));
    CHECK(!std::filesystem::exists(std::filesystem::path{bundlePath.string() + ".tmp"}));
    CHECK(std::filesystem::is_regular_file(bundlePath / "manifest.json"));
    std::ifstream manifestInput{bundlePath / "manifest.json"};
    const std::string manifestText{
        std::istreambuf_iterator<char>{manifestInput},
        std::istreambuf_iterator<char>{}};
    CHECK(manifestText.find("\"source_point_count\"") != std::string::npos);
    CHECK(manifestText.find(std::to_string(sourceInfo->pointCount)) != std::string::npos);
    CHECK(std::filesystem::is_regular_file(bundlePath / "attribute_schema.bin"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "hierarchy.bin"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "node_pages.bin"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "node_raw_chunks.bin"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "node_stats.bin"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "lod_representatives.bin"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "scalar_stats.bin"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "build_status.json"));
    CHECK(std::filesystem::is_regular_file(bundlePath / "build_log.txt"));
    CHECK(std::filesystem::is_directory(bundlePath / "raw_chunks"));

    auto inspection = pc::InspectPointCloudIpcloudBundle(bundlePath, sourceInfo.value(), config);
    CHECK(inspection.state == pc::PointCloudIpcloudCacheState::Hit);
    CHECK(inspection.previewPointCount > 0U);
    CHECK(inspection.representedSourceCount == sourceInfo->pointCount);

    auto preview = pc::LoadPointCloudIpcloudPreview(bundlePath, sourceInfo.value(), config);
    REQUIRE(preview.loaded);
    CHECK(preview.fromPersistentBundle);
    CHECK(preview.cloud.PointCount() == inspection.previewPointCount);
    CHECK(preview.representedSourceCount == sourceInfo->pointCount);
    REQUIRE(!preview.hierarchy.Empty());
    for (const auto& drawItem : pc::TraversePointCloudLodHierarchy(preview.hierarchy, {})) {
        CHECK(drawItem.sourcePointIndex < preview.cloud.PointCount());
    }

    auto hierarchyLoad = pc::LoadPointCloudIpcloudHierarchy(bundlePath, sourceInfo.value(), config);
    REQUIRE(hierarchyLoad.loaded);
    CHECK(hierarchyLoad.hierarchy.sourcePointCount == loadResult.cloud.PointCount());
    auto catalog = pc::LoadPointCloudIpcloudRawChunkCatalog(bundlePath);
    REQUIRE(!catalog.chunks.empty());
    REQUIRE(catalog.nodeRanges.size() == hierarchyLoad.hierarchy.nodes.size());
    CHECK(catalog.totalEncodedBytes > 0U);
    CHECK(catalog.totalDecodedCpuBytes > 0U);
    auto rawChunk = pc::LoadPointCloudIpcloudRawChunk(
        bundlePath,
        catalog.chunks.front().chunkIndex,
        loadResult.cloud.scalarFields);
    REQUIRE(!rawChunk.positions.empty());
    REQUIRE(rawChunk.positions.size() == rawChunk.sourcePointIndices.size());
    CHECK(rawChunk.info.bounds.valid);
    CHECK(rawChunk.info.scalarFieldCount == loadResult.cloud.ScalarFieldCount());
    for (const auto sourceIndex : rawChunk.sourcePointIndices) {
        CHECK(sourceIndex < loadResult.cloud.PointCount());
    }

    pc::PointCloudLodTraversalParams streamingParams;
    streamingParams.viewportWidth = 128;
    streamingParams.viewportHeight = 128;
    streamingParams.maxRepresentatives = 16U;
    streamingParams.maxDrawItems = 16U;
    pc::PointCloudLodTraversalDiagnostics streamingDiagnostics;
    auto streamingDrawItems = pc::TraversePointCloudLodHierarchy(
        hierarchyLoad.hierarchy,
        streamingParams,
        {},
        &streamingDiagnostics);
    REQUIRE(!streamingDrawItems.empty());
    auto resident = pc::BuildPointCloudIpcloudResidentSet(
        bundlePath,
        catalog,
        streamingDiagnostics.frontierNodeIndices,
        streamingDrawItems,
        loadResult.cloud.scalarFields,
        64ULL * 1024ULL * 1024ULL);
    REQUIRE(resident.loaded);
    CHECK(!resident.remappedDrawItems.empty());
    CHECK(resident.diagnostics.residentChunkCount > 0U);
    CHECK(resident.diagnostics.uploadBytesThisFrame <= resident.diagnostics.uploadBudgetBytes);
    for (const auto& drawItem : resident.remappedDrawItems) {
        CHECK(drawItem.sourcePointIndex < resident.cloud.PointCount());
    }

    const auto replaceResult =
        pc::SavePointCloudIpcloudBundle(bundlePath, sourceInfo.value(), config, loadResult.cloud, hierarchy);
    REQUIRE(replaceResult.saved);
    CHECK(!std::filesystem::exists(std::filesystem::path{bundlePath.string() + ".tmp"}));
    CHECK(!std::filesystem::exists(std::filesystem::path{bundlePath.string() + ".old"}));

    auto wrongConfig = config;
    ++wrongConfig.maxDepth;
    inspection = pc::InspectPointCloudIpcloudBundle(bundlePath, sourceInfo.value(), wrongConfig);
    CHECK(inspection.state == pc::PointCloudIpcloudCacheState::Stale);
    CHECK(inspection.reason.find("build settings") != std::string::npos);

    {
        std::ifstream manifestInput{bundlePath / "manifest.json"};
        std::string manifest{
            std::istreambuf_iterator<char>{manifestInput},
            std::istreambuf_iterator<char>{}};
        const auto versionPos = manifest.find("\"cache_format_version\": 2");
        REQUIRE(versionPos != std::string::npos);
        manifest.replace(versionPos, std::string{"\"cache_format_version\": 2"}.size(), "\"cache_format_version\": 1");
        std::ofstream manifestOutput{bundlePath / "manifest.json", std::ios::trunc};
        manifestOutput << manifest;
        manifestOutput.close();
        std::filesystem::remove(bundlePath / "node_raw_chunks.bin");
        inspection = pc::InspectPointCloudIpcloudBundle(bundlePath, sourceInfo.value(), config);
        CHECK(inspection.state == pc::PointCloudIpcloudCacheState::Stale);
        CHECK(inspection.reason.find("format version") != std::string::npos);
    }

    const auto restoredResult =
        pc::SavePointCloudIpcloudBundle(bundlePath, sourceInfo.value(), config, loadResult.cloud, hierarchy);
    REQUIRE(restoredResult.saved);

    std::filesystem::remove(bundlePath / "lod_representatives.bin");
    inspection = pc::InspectPointCloudIpcloudBundle(bundlePath, sourceInfo.value(), config);
    CHECK(inspection.state == pc::PointCloudIpcloudCacheState::Corrupt);
    CHECK(inspection.reason.find("lod_representatives.bin") != std::string::npos);

    std::filesystem::remove_all(tempRoot);
    std::filesystem::remove(sourcePath);
}

TEST_CASE("Point-cloud .ipcloud temporary bundle is partial and resumable", "[pointcloud][lod][cache]") {
    namespace pc = invisible_places::renderer::pointcloud;

    const auto sourcePath = WriteTinyBinaryPointCloudFixture();
    auto sourceInfo = pc::MakePointCloudIpcloudSourceInfo(sourcePath);
    REQUIRE(sourceInfo.has_value());
    const auto tempRoot =
        std::filesystem::temp_directory_path() /
        ("InvisiblePlacesIpcloudPartialTest-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const pc::PointCloudLodBuildConfig config{
        .maxLeafSourcePoints = 2U,
        .maxDepth = 4U,
        .maxInternalRepresentatives = 8U};
    const auto bundlePath = pc::BuildPointCloudIpcloudBundlePath(tempRoot / "PointCloudCache", sourceInfo.value());
    const auto temporaryPath = std::filesystem::path{bundlePath.string() + ".tmp"};
    std::filesystem::create_directories(temporaryPath);
    {
        std::ofstream status{temporaryPath / "build_status.json"};
        status << R"json({
  "parse_header_complete": true,
  "upper_hierarchy_complete": true,
  "node_pages_complete": false,
  "representative_ranges_complete": true,
  "raw_chunk_ranges_complete": false,
  "scalar_stats_complete": false,
  "complete": false,
  "failed": false,
  "interrupted": true,
  "raw_chunks_completed": 1,
  "raw_chunk_count": 3,
  "phase": "interrupted after representatives",
  "message": "test interruption"
})json";
    }

    const auto inspection = pc::InspectPointCloudIpcloudBundle(bundlePath, sourceInfo.value(), config);
    CHECK(inspection.state == pc::PointCloudIpcloudCacheState::Partial);
    CHECK(inspection.buildStatus.interrupted);
    CHECK(inspection.buildStatus.representativeRangesComplete);

    const auto resume = pc::ResumePointCloudIpcloudBuild(bundlePath, sourceInfo.value(), config);
    CHECK(resume.resumable);
    CHECK(!resume.restartRequired);

    std::filesystem::remove_all(tempRoot);
    std::filesystem::remove(sourcePath);
}

TEST_CASE("Point-cloud .ipcloud source preview remaps draw-item indices", "[pointcloud][lod][cache]") {
    namespace pc = invisible_places::renderer::pointcloud;

    const auto sourcePath = WriteTinyBinaryPointCloudFixture();
    auto preview = pc::LoadPointCloudIpcloudSourcePreview(sourcePath, 2U);
    REQUIRE(preview.loaded);
    REQUIRE(preview.cloud.PointCount() == 2U);
    REQUIRE(preview.sourcePointIndices.size() == preview.cloud.PointCount());
    CHECK(preview.representedSourceCount == 3U);
    REQUIRE(!preview.hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::FullSource;
    params.viewportWidth = 128;
    params.viewportHeight = 128;
    params.maxRepresentatives = 16U;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(preview.hierarchy, params);
    REQUIRE(!drawItems.empty());
    for (const auto& drawItem : drawItems) {
        CHECK(drawItem.sourcePointIndex < preview.cloud.PointCount());
    }

    std::filesystem::remove(sourcePath);
}

TEST_CASE("Point-cloud adaptive LOD traversal culls offscreen nodes and obeys budgets", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    for (int z = 0; z < 4; ++z) {
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 4; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / 3.0F) - 0.5F,
                    (static_cast<float>(y) / 3.0F) - 0.5F,
                    (static_cast<float>(z) / 3.0F) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 3U;
    params.viewProjection = glm::mat4{1.0F};
    const auto budgeted = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    REQUIRE(!budgeted.empty());
    CHECK(budgeted.size() <= 3U);

    params.viewProjection = glm::translate(glm::mat4{1.0F}, glm::vec3{4.0F, 0.0F, 0.0F});
    const auto offscreen = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    CHECK(offscreen.empty());
}

TEST_CASE("Point-cloud GPU selection parity compares compacted draw-item identity", "[pointcloud][lod][gpu]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 4;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U, .maxInternalRepresentatives = 16U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 512;
    params.viewportHeight = 512;
    params.maxRepresentatives = 16U;
    params.maxEstimatedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    const auto cpuSelection = pc::TraversePointCloudLodHierarchy(hierarchy, params);
    REQUIRE(!cpuSelection.empty());

    const auto exactParity = pc::ComparePointCloudGpuSelectionParity(cpuSelection, cpuSelection);
    CHECK(exactParity.passed);
    CHECK(exactParity.selectedCountDelta == 0);
    CHECK(exactParity.sourceOverlapRatio == Catch::Approx(1.0));
    CHECK(exactParity.cpuSelectionHash == exactParity.gpuSelectionHash);

    auto mismatchedGpuSelection = cpuSelection;
    mismatchedGpuSelection.pop_back();
    const auto mismatch = pc::ComparePointCloudGpuSelectionParity(cpuSelection, mismatchedGpuSelection);
    CHECK_FALSE(mismatch.passed);
    CHECK(mismatch.selectedCountDelta == -1);
}

TEST_CASE("GPU-driven selection policy keeps CPU fallback until parity and speed are proven", "[pointcloud][lod][gpu]") {
    namespace pc = invisible_places::renderer::pointcloud;

    pc::PointCloudGpuDrivenSelectionCapabilities capabilities;
    capabilities.computeQueueSupported = true;
    capabilities.storageBuffersSupported = true;
    capabilities.indirectDrawSupported = true;
    capabilities.indirectCountSupported = false;
    capabilities.maxStorageBuffersPerShaderStage = 8U;
    capabilities.maxStorageBufferRange = 1ULL << 30U;
    capabilities.maxDrawIndirectCount = 1U;

    pc::PointCloudGpuDrivenSelectionInputs inputs;
    inputs.hierarchyNodeCount = 4096U;
    inputs.representativeCount = 131072U;
    inputs.selectedDrawItemCount = 131072U;
    inputs.cpuSelectionMs = 4.0;

    const auto unproven = pc::EvaluatePointCloudGpuDrivenSelection(capabilities, inputs);
    CHECK(unproven.selectionPath == "cpu-selection+indirect");
    CHECK(unproven.indirectDrawRecommended);
    CHECK_FALSE(unproven.indirectCountSupported);
    CHECK(unproven.fallbackReason.find("parity") != std::string::npos);
    CHECK(unproven.fallbackReason.find("timing") != std::string::npos);

    inputs.parityChecked = true;
    inputs.parityPassed = true;
    inputs.gpuSelectionMs = 2.5;
    inputs.gpuCompactionMs = 0.2;
    const auto proven = pc::EvaluatePointCloudGpuDrivenSelection(capabilities, inputs);
    CHECK(proven.selectionPath == "gpu-driven+indirect");
    CHECK(proven.fallbackReason.empty());
    CHECK(proven.computeSelectionBeneficial);

    inputs.gpuSelectionMs = 4.2;
    const auto slower = pc::EvaluatePointCloudGpuDrivenSelection(capabilities, inputs);
    CHECK(slower.selectionPath == "cpu-selection+indirect");
    CHECK(slower.fallbackReason.find("not measurably faster") != std::string::npos);
}

TEST_CASE("GPU-driven selection policy reports feature fallback reasons", "[pointcloud][lod][gpu]") {
    namespace pc = invisible_places::renderer::pointcloud;

    pc::PointCloudGpuDrivenSelectionCapabilities capabilities;
    capabilities.computeQueueSupported = false;
    capabilities.storageBuffersSupported = false;
    capabilities.indirectDrawSupported = false;
    capabilities.maxStorageBuffersPerShaderStage = 1U;
    capabilities.maxStorageBufferRange = 16U;
    capabilities.maxDrawIndirectCount = 0U;

    const auto decision = pc::EvaluatePointCloudGpuDrivenSelection(
        capabilities,
        {.hierarchyNodeCount = 1024U,
         .representativeCount = 131072U,
         .selectedDrawItemCount = 131072U,
         .cpuSelectionMs = 6.0,
         .gpuSelectionMs = 2.0,
         .gpuCompactionMs = 0.4,
         .parityChecked = true,
         .parityPassed = true,
         .allowGpuDrivenSelection = true,
         .allowIndirectDraw = true});

    CHECK(decision.selectionPath == "cpu");
    CHECK_FALSE(decision.computeSelectionSupported);
    CHECK_FALSE(decision.indirectDrawRecommended);
    CHECK(decision.fallbackReason.find("compute") != std::string::npos);
    CHECK(decision.fallbackReason.find("storage-buffer") != std::string::npos);
    CHECK(decision.fallbackReason.find("indirect draw unsupported") != std::string::npos);
}

TEST_CASE("Point-cloud adaptive LOD traversal enforces estimated fragment budget", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 16U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 128U;
    params.maxEstimatedFragments = 1.0e9F;
    params.viewProjection = glm::mat4{1.0F};
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 8.0F);

    const auto unlimited = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    REQUIRE(unlimited.size() > 8U);
    const auto unlimitedFragments = std::accumulate(
        unlimited.begin(),
        unlimited.end(),
        0.0F,
        [](float total, const invisible_places::renderer::pointcloud::PointCloudDrawItemGpu& drawItem) {
            return total + drawItem.footprintAreaPixels;
        });
    REQUIRE(unlimitedFragments > 0.0F);

    params.maxEstimatedFragments = unlimited.front().footprintAreaPixels * 3.5F;
    const auto budgeted = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    REQUIRE(!budgeted.empty());
    CHECK(budgeted.size() < unlimited.size());
    const auto budgetedFragments = std::accumulate(
        budgeted.begin(),
        budgeted.end(),
        0.0F,
        [](float total, const invisible_places::renderer::pointcloud::PointCloudDrawItemGpu& drawItem) {
            return total + drawItem.footprintAreaPixels;
        });
    CHECK(budgetedFragments <= params.maxEstimatedFragments + 0.01F);
}

TEST_CASE("Beauty adaptive LOD traversal enforces estimated blended fragment budget", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 16U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 128U;
    params.maxEstimatedFragments = 1.0e9F;
    params.maxEstimatedBlendedFragments = 1.0e9F;
    params.viewProjection = glm::mat4{1.0F};
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 12.0F);

    pc::PointCloudLodTraversalDiagnostics unlimitedDiagnostics;
    const auto unlimited = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &unlimitedDiagnostics);
    REQUIRE(unlimited.size() > 8U);
    REQUIRE(unlimitedDiagnostics.estimatedBlendedFragmentCost > 0.0F);

    params.maxEstimatedBlendedFragments = unlimited.front().renderAreaPixels * 0.8F;
    pc::PointCloudLodTraversalDiagnostics budgetedDiagnostics;
    const auto budgeted = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &budgetedDiagnostics);
    REQUIRE(!budgeted.empty());
    CHECK(budgeted.size() < unlimited.size());
    CHECK(budgetedDiagnostics.blendedFragmentBudgetReached);
    CHECK_FALSE(budgetedDiagnostics.fragmentBudgetReached);
}

TEST_CASE("Beauty adaptive LOD tile pressure limits low-priority representatives", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 32U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.viewportWidth = 96;
    params.viewportHeight = 64;
    params.tileSizePixels = 32U;
    params.tileBudgetEnabled = true;
    params.maxRepresentatives = 32U;
    params.maxEstimatedFragments = 1.0e9F;
    params.maxEstimatedBlendedFragments = 1.0e9F;
    params.maxTileEstimatedFragments = 1.0e9F;
    params.maxTileEstimatedBlendedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    params.viewProjection = glm::mat4{1.0F};
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 24.0F);
    invisible_places::style::SetScalarConstant(&params.style.opacity, 0.35F);

    pc::PointCloudLodTraversalDiagnostics unlimitedDiagnostics;
    const auto unlimited = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &unlimitedDiagnostics);
    REQUIRE(unlimited.size() > 8U);
    CHECK(unlimitedDiagnostics.tileBudgetEnabled);
    CHECK(unlimitedDiagnostics.tileSizePixels == 32U);
    CHECK(unlimitedDiagnostics.tileCount == 6U);
    CHECK(unlimitedDiagnostics.maxTileEstimatedBlendedFragments > 0.0F);

    params.maxTileEstimatedBlendedFragments = 1.0F;
    pc::PointCloudLodTraversalDiagnostics budgetedDiagnostics;
    const auto budgeted = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &budgetedDiagnostics);
    REQUIRE(!budgeted.empty());
    CHECK(budgeted.size() < unlimited.size());
    CHECK(budgetedDiagnostics.tileLimitedRepresentativeCount > 0U);
    CHECK(budgetedDiagnostics.tileLimitedNodeCount > 0U);
    CHECK(budgetedDiagnostics.tilePreservedRepresentativeCount > 0U);
    CHECK(budgetedDiagnostics.blendedFragmentBudgetReached);
    CHECK_FALSE(budgetedDiagnostics.fragmentBudgetReached);
}

TEST_CASE("Beauty adaptive LOD tile pressure preserves protected feature representatives", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    auto packColor = [](std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
        return static_cast<std::uint32_t>(red) |
               (static_cast<std::uint32_t>(green) << 8U) |
               (static_cast<std::uint32_t>(blue) << 16U) |
               (0xffU << 24U);
    };

    invisible_places::io::LoadedPointCloud cloud;
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    cloud.scalarFields.push_back({.name = "Interest"});
    constexpr int gridSize = 4;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                const auto pointIndex = static_cast<std::uint32_t>(cloud.positions.size());
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
                cloud.packedColors.push_back(packColor(96, 96, 96));
                cloud.normals.push_back({0.0F, 0.0F, 1.0F});
                float interest = 0.25F;
                if (pointIndex == 5U) {
                    cloud.packedColors.back() = packColor(255, 16, 16);
                }
                if (pointIndex == 17U) {
                    cloud.normals.back() = {1.0F, 0.0F, 0.0F};
                }
                if (pointIndex == 42U) {
                    interest = 10.0F;
                    cloud.packedColors.back() = packColor(255, 240, 80);
                }
                cloud.scalarFieldValues.push_back(interest);
                cloud.scalarFields.front().Include(interest);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U, .maxInternalRepresentatives = 12U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.viewportWidth = 128;
    params.viewportHeight = 128;
    params.tileSizePixels = 32U;
    params.tileBudgetEnabled = true;
    params.maxRepresentatives = 12U;
    params.maxEstimatedFragments = 1.0e9F;
    params.maxEstimatedBlendedFragments = 1.0e9F;
    params.maxTileEstimatedFragments = 1.0e9F;
    params.maxTileEstimatedBlendedFragments = 1.0F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    params.style.colorMode = pc::PointCloudColorMode::ScalarColormap;
    params.style.emissiveStrength.mode = invisible_places::style::ParameterSourceMode::FieldMapped;
    params.style.emissiveStrength.fieldMap.fieldSlot = 0;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 20.0F);
    invisible_places::style::SetScalarConstant(&params.style.opacity, 0.45F);

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(!drawItems.empty());
    CHECK(diagnostics.tileLimitedRepresentativeCount > 0U);
    CHECK(diagnostics.tilePreservedRepresentativeCount > 0U);
    CHECK(diagnostics.emittedClassCounts[2] > 0U);
    CHECK(diagnostics.emittedClassCounts[4] > 0U);
    CHECK(diagnostics.emittedClassCounts[6] > 0U);
}

TEST_CASE("Tile pressure preserves one representative per visible frontier node", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 6;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 8U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::AdaptiveHighQuality;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.viewportWidth = 256;
    params.viewportHeight = 256;
    params.tileBudgetEnabled = true;
    params.tileSizePixels = 32U;
    params.maxRepresentatives = 4096U;
    params.maxEstimatedFragments = 1.0e9F;
    params.maxEstimatedBlendedFragments = 1.0e9F;
    params.maxTileEstimatedFragments = 1.0e9F;
    params.maxTileEstimatedBlendedFragments = 1.0F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 18.0F);

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(!drawItems.empty());
    REQUIRE(diagnostics.visibleFrontierNodeCount >= 1U);
    CHECK(drawItems.size() >= diagnostics.visibleFrontierNodeCount);
    CHECK(diagnostics.tileLimitedRepresentativeCount > 0U);
}

TEST_CASE("Fast Basic records tile diagnostics without applying tile pressure", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 6;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 24U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::FastBasicSquare;
    params.viewportWidth = 96;
    params.viewportHeight = 64;
    params.tileBudgetEnabled = false;
    params.tileSizePixels = 32U;
    params.maxRepresentatives = 24U;
    params.maxEstimatedFragments = 1.0e9F;
    params.maxTileEstimatedFragments = 1.0F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 18.0F);

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(drawItems.size() > 8U);
    CHECK_FALSE(diagnostics.tileBudgetEnabled);
    CHECK(diagnostics.tileSizePixels == 32U);
    CHECK(diagnostics.tileCount == 6U);
    CHECK(diagnostics.maxTileEstimatedFragments > 0.0F);
    CHECK(diagnostics.tileLimitedRepresentativeCount == 0U);
}

TEST_CASE("Full Source bypasses tile limits and conservative culling", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 4;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 6U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::FullSource;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.viewportWidth = 128;
    params.viewportHeight = 128;
    params.tileBudgetEnabled = true;
    params.tileSizePixels = 32U;
    params.maxTileEstimatedFragments = 1.0F;
    params.maxTileEstimatedBlendedFragments = 1.0F;
    params.occlusionCullingEnabled = true;

    pc::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &diagnostics);
    REQUIRE(drawItems.size() == cloud.PointCount());
    CHECK_FALSE(diagnostics.tileBudgetEnabled);
    CHECK(diagnostics.tileCount == 0U);
    CHECK(diagnostics.tileLimitedRepresentativeCount == 0U);
    CHECK_FALSE(diagnostics.occlusionCullingEnabled);
    CHECK(diagnostics.occlusionCullingState == "disabled");
    CHECK(diagnostics.occlusionRejectedNodeCount == 0U);
}

TEST_CASE("Conservative culling stays disabled without a safe depth proxy", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) {
            invisible_places::io::Float3 point{
                (static_cast<float>(x) / 2.0F) - 0.5F,
                (static_cast<float>(y) / 2.0F) - 0.5F,
                0.0F};
            cloud.positions.push_back(point);
            cloud.bounds.Expand(point);
        }
    }

    const auto hierarchy = pc::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 4U});
    REQUIRE(!hierarchy.Empty());

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.rendererCostProfile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    params.viewportWidth = 128;
    params.viewportHeight = 128;
    params.maxRepresentatives = 64U;
    params.maxEstimatedFragments = 1.0e9F;
    params.occlusionCullingEnabled = true;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 8.0F);

    pc::PointCloudLodTraversalDiagnostics translucentDiagnostics;
    const auto translucentItems =
        pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &translucentDiagnostics);
    REQUIRE(!translucentItems.empty());
    CHECK_FALSE(translucentDiagnostics.occlusionCullingEnabled);
    CHECK(translucentDiagnostics.occlusionCullingState == "disabled");
    CHECK(translucentDiagnostics.occlusionCullingDisabledReason.find("depth") != std::string::npos);
    CHECK(translucentDiagnostics.occlusionRejectedNodeCount == 0U);

    params.style.depthContribution = pc::PointCloudDepthContribution::Always;
    pc::PointCloudLodTraversalDiagnostics depthDiagnostics;
    const auto depthItems = pc::TraversePointCloudLodHierarchy(hierarchy, params, {}, &depthDiagnostics);
    REQUIRE(!depthItems.empty());
    CHECK_FALSE(depthDiagnostics.occlusionCullingEnabled);
    CHECK(depthDiagnostics.occlusionCullingState == "uncertain");
    CHECK(depthDiagnostics.occlusionRejectedNodeCount == 0U);
    CHECK(depthDiagnostics.occlusionRejectedRepresentedSourceCount == 0ULL);
}

TEST_CASE("Point-cloud adaptive LOD diagnostics distinguish visible emitted and budget pressure", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 16U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 4U;
    params.maxEstimatedFragments = 1.0e9F;
    params.viewProjection = glm::mat4{1.0F};
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 8.0F);

    invisible_places::renderer::pointcloud::PointCloudLodTraversalDiagnostics diagnostics;
    const auto drawItems = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params,
        {},
        &diagnostics);
    REQUIRE(!drawItems.empty());
    CHECK(drawItems.size() <= params.maxRepresentatives);
    CHECK(diagnostics.visibleFrontierNodeCount > 0U);
    CHECK(diagnostics.visibleFrontierRepresentedSourceCount >= diagnostics.emittedRepresentedSourceCount);
    CHECK(diagnostics.emittedRepresentativeCount == drawItems.size());
    CHECK(diagnostics.representativeBudgetReached);
}

TEST_CASE("Point-cloud coarse adaptive fallback stays spatially distributed", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::FastAdaptivePreview;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 7.0F);

    const auto fallback = invisible_places::renderer::pointcloud::BuildCoarsePointCloudLodFallbackDrawItems(
        hierarchy,
        params,
        32U);
    REQUIRE(fallback.size() > 1U);
    const auto representedSourceCount = std::accumulate(
        fallback.begin(),
        fallback.end(),
        std::uint64_t{0},
        [](std::uint64_t total, const invisible_places::renderer::pointcloud::PointCloudDrawItemGpu& drawItem) {
            return total + drawItem.representedSourceCount;
        });
    CHECK(representedSourceCount == cloud.PointCount());
    CHECK(std::any_of(
        fallback.begin(),
        fallback.end(),
        [](const invisible_places::renderer::pointcloud::PointCloudDrawItemGpu& drawItem) {
            return drawItem.representedSourceCount > 1U;
        }));
}

TEST_CASE("Point-cloud draw items use a std430-compatible GPU layout", "[pointcloud][lod]") {
    using invisible_places::renderer::pointcloud::PointCloudDrawItemGpu;

    CHECK(sizeof(PointCloudDrawItemGpu) == 32U);
    CHECK(alignof(PointCloudDrawItemGpu) == 16U);
    CHECK(offsetof(PointCloudDrawItemGpu, sourcePointIndex) == 0U);
    CHECK(offsetof(PointCloudDrawItemGpu, representedSourceCount) == 4U);
    CHECK(offsetof(PointCloudDrawItemGpu, reserved0) == 8U);
    CHECK(offsetof(PointCloudDrawItemGpu, reserved1) == 12U);
    CHECK(offsetof(PointCloudDrawItemGpu, footprintAreaPixels) == 16U);
    CHECK(offsetof(PointCloudDrawItemGpu, opacityCompensation) == 20U);
    CHECK(offsetof(PointCloudDrawItemGpu, emissionCompensation) == 24U);
    CHECK(offsetof(PointCloudDrawItemGpu, renderAreaPixels) == 28U);
    CHECK(static_cast<std::uint32_t>(
              invisible_places::renderer::pointcloud::PointCloudLodRendererCostProfile::BeautyWorldSurfel) == 3U);
}

TEST_CASE("Point-cloud adaptive LOD traversal emits deterministic stable-ranked draw items", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 32U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 24U;
    params.maxEstimatedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 7.0F);

    const auto first = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    const auto second = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    REQUIRE(first.size() == second.size());
    REQUIRE(first.size() > 8U);
    for (std::size_t index = 0; index < first.size(); ++index) {
        CHECK(first[index].sourcePointIndex == second[index].sourcePointIndex);
        CHECK(first[index].reserved0 == second[index].reserved0);
        CHECK(first[index].reserved1 == second[index].reserved1);
    }
}

TEST_CASE("Point-cloud adaptive LOD lower budgets select stable rank prefixes", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 32U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxEstimatedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 7.0F);

    params.maxRepresentatives = 24U;
    const auto highBudget = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);
    params.maxRepresentatives = 12U;
    const auto lowBudget = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params);

    REQUIRE(highBudget.size() >= lowBudget.size());
    REQUIRE(lowBudget.size() == 12U);
    std::unordered_set<std::uint32_t> highSeeds;
    for (const auto& item : highBudget) {
        highSeeds.insert(item.reserved0);
    }
    for (const auto& item : lowBudget) {
        CHECK(highSeeds.contains(item.reserved0));
    }
}

TEST_CASE("Point-cloud LOD cache-loaded traversal preserves stable rank ordering", "[pointcloud][lod][cache]") {
    namespace pc = invisible_places::renderer::pointcloud;

    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 6;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto tempRoot =
        std::filesystem::temp_directory_path() /
        ("InvisiblePlacesLodRankCacheTest-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(tempRoot);
    const auto sourcePath = tempRoot / "fixture.ply";
    {
        std::ofstream source{sourcePath};
        source << "ply\nformat ascii 1.0\ncomment stable rank fixture\nend_header\n";
    }
    cloud.sourcePath = sourcePath;

    const pc::PointCloudLodBuildConfig config{
        .maxLeafSourcePoints = 1U,
        .maxDepth = 8U,
        .maxInternalRepresentatives = 24U};
    const auto fresh = pc::BuildPointCloudLodHierarchy(cloud, config);
    REQUIRE(!fresh.Empty());
    const auto source = pc::MakePointCloudLodCacheSource(sourcePath, cloud);
    const auto cachePath = pc::BuildPointCloudLodCachePath(tempRoot / "lod", sourcePath);
    std::string errorMessage;
    REQUIRE(pc::SavePointCloudLodHierarchyCache(cachePath, source, config, fresh, &errorMessage));
    const auto loaded = pc::LoadPointCloudLodHierarchyCache(cachePath, source, config);
    REQUIRE(loaded.loaded);

    pc::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 12U;
    params.maxEstimatedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 10000.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 7.0F);

    const auto freshItems = pc::TraversePointCloudLodHierarchy(fresh, params);
    const auto loadedItems = pc::TraversePointCloudLodHierarchy(loaded.hierarchy, params);
    REQUIRE(freshItems.size() == loadedItems.size());
    for (std::size_t index = 0; index < freshItems.size(); ++index) {
        CHECK(freshItems[index].sourcePointIndex == loadedItems[index].sourcePointIndex);
        CHECK(freshItems[index].reserved0 == loadedItems[index].reserved0);
        CHECK(freshItems[index].reserved1 == loadedItems[index].reserved1);
    }

    std::filesystem::remove_all(tempRoot);
}

TEST_CASE("Point-cloud adaptive LOD hysteresis prevents threshold flip-flop", "[pointcloud][lod]") {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int gridSize = 8;
    for (int z = 0; z < gridSize; ++z) {
        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                invisible_places::io::Float3 point{
                    (static_cast<float>(x) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(y) / static_cast<float>(gridSize - 1)) - 0.5F,
                    (static_cast<float>(z) / static_cast<float>(gridSize - 1)) - 0.5F};
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    const auto hierarchy = invisible_places::renderer::pointcloud::BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 1U, .maxDepth = 8U, .maxInternalRepresentatives = 16U});
    REQUIRE(!hierarchy.Empty());

    invisible_places::renderer::pointcloud::PointCloudLodTraversalParams params;
    params.densityMode = invisible_places::output::PointCloudExportDensityMode::MatchViewportAdaptive;
    params.viewportWidth = 1024;
    params.viewportHeight = 1024;
    params.maxRepresentatives = 128U;
    params.maxEstimatedFragments = 1.0e9F;
    params.targetProjectedSpacingPixels = 15.0F;
    params.maxRepresentativeDiameterPixels = 10000.0F;
    params.hysteresisPromoteScale = 1.10F;
    params.hysteresisDemoteScale = 0.76F;
    invisible_places::style::SetScalarConstant(&params.style.pointSize, 7.0F);

    invisible_places::renderer::pointcloud::PointCloudLodTraversalDiagnostics firstDiagnostics;
    const auto first = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params,
        {},
        &firstDiagnostics);
    REQUIRE(!first.empty());
    REQUIRE(!firstDiagnostics.frontierNodeIndices.empty());

    params.previousFrontierNodeIndices = firstDiagnostics.frontierNodeIndices;
    params.targetProjectedSpacingPixels = 25.0F;
    invisible_places::renderer::pointcloud::PointCloudLodTraversalDiagnostics secondDiagnostics;
    const auto second = invisible_places::renderer::pointcloud::TraversePointCloudLodHierarchy(
        hierarchy,
        params,
        {},
        &secondDiagnostics);
    REQUIRE(!second.empty());
    INFO("first frontier " << firstDiagnostics.frontierNodeIndices.size()
                           << " second frontier " << secondDiagnostics.frontierNodeIndices.size()
                           << " promoted " << secondDiagnostics.promotedNodeCount
                           << " demoted " << secondDiagnostics.demotedNodeCount
                           << " kept " << secondDiagnostics.hysteresisKeptNodeCount);
    CHECK(secondDiagnostics.demotedNodeCount == 0U);
    CHECK(secondDiagnostics.hysteresisKeptNodeCount > 0U);
    CHECK(secondDiagnostics.frontierNodeIndices == firstDiagnostics.frontierNodeIndices);
}

TEST_CASE("Adaptive LOD signature uses traversal metadata instead of vector hashing", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto rendererHeader = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.hpp");
    const auto signatureSection = SourceSection(
        applicationSource,
        "std::uint64_t RenderStateSignature",
        "glm::mat4 ToGlmMatrix");

    CHECK(applicationSource.find("HashAdaptiveDrawItems") == std::string::npos);
    CHECK(signatureSection.find("adaptiveLodRevision") != std::string::npos);
    CHECK(signatureSection.find("adaptiveRepresentedSourceCount") != std::string::npos);
    CHECK(signatureSection.find("requiresAdaptiveDrawItems") != std::string::npos);
    CHECK(signatureSection.find("adaptiveLodRequestedDensity") != std::string::npos);
    CHECK(signatureSection.find("adaptiveLodDisplayedDensity") != std::string::npos);
    CHECK(signatureSection.find("adaptiveLodAsyncPending") != std::string::npos);
    CHECK(signatureSection.find("adaptiveDrawItems") == std::string::npos);
    CHECK(rendererHeader.find("bool requiresAdaptiveDrawItems") != std::string::npos);
}

TEST_CASE("Adaptive LOD traversal cache key covers render-affecting inputs", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto keySection = SourceSection(
        applicationSource,
        "std::uint64_t AdaptiveLodTraversalKey",
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems");
    const auto buildSection = SourceSection(
        applicationSource,
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems",
        "bool PopulateAdaptivePointCloudLayer");

    CHECK(keySection.find("HashQuantizedMat4(&seed, viewProjection") != std::string::npos);
    CHECK(keySection.find("HashQuantizedVec3(&seed, cameraPosition") != std::string::npos);
    CHECK(keySection.find("viewportWidth") != std::string::npos);
    CHECK(keySection.find("viewportHeight") != std::string::npos);
    CHECK(keySection.find("densityMode") != std::string::npos);
    CHECK(keySection.find("ManualAdaptiveDrawItemCap") != std::string::npos);
    CHECK(keySection.find("pointLodHierarchyGeneration") != std::string::npos);
    CHECK(keySection.find("HashPointStyle(&seed, style)") != std::string::npos);
    CHECK(buildSection.find("FindAdaptiveLodCacheEntry(session, traversalKey)") != std::string::npos);
    CHECK(buildSection.find("StoreAdaptiveLodCacheEntry") != std::string::npos);
    CHECK(buildSection.find("ScheduleAdaptiveLodTraversal") != std::string::npos);
}

TEST_CASE("Viewport adaptive LOD schedules traversal instead of running it synchronously", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto buildRenderStateSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::core::SceneRenderState BuildRenderState",
        "int Application::Run");
    const auto buildItemsSection = SourceSection(
        applicationSource,
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems",
        "bool PopulateAdaptivePointCloudLayer");

    CHECK(buildRenderStateSection.find("TraversePointCloudLodHierarchy") == std::string::npos);
    CHECK(buildRenderStateSection.find("SelectAdaptiveViewportDensityMode") != std::string::npos);
    CHECK(buildRenderStateSection.find("runtimeState.previewFrameCounter") != std::string::npos);
    CHECK(buildItemsSection.find("allowSynchronousTraversal") != std::string::npos);
    CHECK(buildItemsSection.find("ScheduleAdaptiveLodTraversal") != std::string::npos);
    CHECK(buildItemsSection.find("coarse fallback") != std::string::npos);
    CHECK(buildItemsSection.find("coarseFallback") != std::string::npos);
    CHECK(buildItemsSection.find("PointCloudExportDensityMode::FastAdaptivePreview") != std::string::npos);
}

TEST_CASE("Adaptive viewport path does not fall back to source draw count", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto rendererSource = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.cpp");
    const auto populateSection = SourceSection(
        applicationSource,
        "bool PopulateAdaptivePointCloudLayer",
        "AdaptiveLodPreviewSummary ComputeAdaptiveLodPreviewSummary");
    const auto buildRenderStateSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::core::SceneRenderState BuildRenderState",
        "int Application::Run");
    const auto resolveSection = SourceSection(
        rendererSource,
        "bool VulkanViewportShell::ResolvePointCloudDrawPlan",
        "bool VulkanViewportShell::RecordPointCloudLayerDraw");

    CHECK(populateSection.find("layer->requiresAdaptiveDrawItems = true") != std::string::npos);
    CHECK(populateSection.find("layer->drawPointCount = 0") != std::string::npos);
    CHECK(populateSection.find("layer->useAdaptiveDrawItems = false") != std::string::npos);
    CHECK(populateSection.find("adaptiveLodRuntimeStatus") != std::string::npos);
    CHECK(applicationSource.find("AdaptiveLodTransitionTooLargeForSmoothBlend") != std::string::npos);
    CHECK(applicationSource.find("large adaptive transition skipped") != std::string::npos);
    CHECK(applicationSource.find("512'000U") != std::string::npos);
    CHECK(resolveSection.find("layer.requiresAdaptiveDrawItems") != std::string::npos);
    CHECK(resolveSection.find("return false;") < resolveSection.find("resources->activePointCount"));
    CHECK(buildRenderStateSection.find("exactFullSourceReady") != std::string::npos);
    CHECK(buildRenderStateSection.find("PointCloudExactSourceResident(session)") != std::string::npos);
    CHECK(buildRenderStateSection.find("PopulateAdaptivePointCloudLayer") != std::string::npos);
}

TEST_CASE("Adaptive async traversal supersedes stale viewport requests", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto scheduleSection = SourceSection(
        applicationSource,
        "bool ScheduleAdaptiveLodTraversal",
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems");
    const auto pollSection = SourceSection(
        applicationSource,
        "void PollPointCloudLodWorkers",
        "ImVec2 CurrentUiViewportOrigin");
    const auto lodHeader = ReadRepoTextFile("src/renderer/pointcloud/PointCloudLodHierarchy.hpp");
    const auto rendererHeader = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.hpp");

    CHECK(lodHeader.find("std::stop_token stopToken = {}") != std::string::npos);
    CHECK(scheduleSection.find("latestRequestedGeneration") != std::string::npos);
    CHECK(scheduleSection.find("request_stop()") != std::string::npos);
    CHECK(scheduleSection.find("async traversal superseded by newer request") != std::string::npos);
    CHECK(scheduleSection.find("discarded stale async traversal before scheduling") != std::string::npos);
    CHECK(applicationSource.find("maxReuseAgeFrames") != std::string::npos);
    CHECK(applicationSource.find("std::numeric_limits<std::uint64_t>::max()") != std::string::npos);
    CHECK(applicationSource.find("skipEmergencyPreview") != std::string::npos);
    CHECK(applicationSource.find("async traversal in flight, supersede throttled") != std::string::npos);
    CHECK(applicationSource.find("MinimumReusableAdaptiveRepresentedSourceCount") != std::string::npos);
    CHECK(applicationSource.find("\"reused exact\"") != std::string::npos);
    CHECK(pollSection.find("discarded stale async traversal") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveLodStaleTraversalDiscardedCount") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveLodAsyncPendingAgeMs") != std::string::npos);
}

TEST_CASE("Point-cloud performance governor updates EWMA and clamps scale changes", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    pc::PointCloudPerformanceGovernor governor;
    pc::PointCloudPerformanceGovernorSample sample;
    sample.profile = pc::PointCloudLodRendererCostProfile::BeautyScreenSprite;
    sample.sceneRendered = true;
    sample.timestampSupported = true;
    sample.timingValid = true;
    sample.targetPointPassMs = 10.0;
    sample.gpuPointPassMs = 30.0;
    sample.gpuCompositeMs = 4.0;

    const auto first = pc::UpdatePointCloudPerformanceGovernor(&governor, sample);
    CHECK(first.gpuPointPassEwmaMs == Catch::Approx(30.0));
    CHECK(first.budgetScale == Catch::Approx(0.80F));
    CHECK(first.overTarget);

    sample.gpuPointPassMs = 10.0;
    const auto second = pc::UpdatePointCloudPerformanceGovernor(&governor, sample);
    CHECK(second.gpuPointPassEwmaMs == Catch::Approx(26.4));
    CHECK(second.budgetScale >= 0.64F);
    CHECK(second.budgetScale <= 0.80F);
}

TEST_CASE("Point-cloud performance governor uses conservative fixed fallback without timestamps", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    pc::PointCloudPerformanceGovernor governor;
    pc::PointCloudPerformanceGovernorSample sample;
    sample.sceneRendered = true;
    sample.timestampSupported = false;
    sample.timingValid = false;
    sample.gpuPointPassMs = 80.0;

    const auto output = pc::UpdatePointCloudPerformanceGovernor(&governor, sample);
    CHECK_FALSE(output.timestampSupported);
    CHECK(output.timestampState == "unavailable");
    CHECK(output.budgetScale == Catch::Approx(1.0F));
    CHECK(output.status.find("conservative fixed budget") != std::string::npos);
}

TEST_CASE("Point-cloud performance governor reports performance-limited at floor", "[pointcloud][lod]") {
    namespace pc = invisible_places::renderer::pointcloud;

    pc::PointCloudPerformanceGovernor governor;
    pc::PointCloudPerformanceGovernorSample sample;
    sample.sceneRendered = true;
    sample.timestampSupported = true;
    sample.timingValid = true;
    sample.targetPointPassMs = 10.0;
    sample.gpuPointPassMs = 100.0;
    sample.minimumBudgetScale = 0.35F;
    sample.fragmentBudgetReached = true;

    pc::PointCloudPerformanceGovernorOutput output;
    for (int i = 0; i < 10; ++i) {
        output = pc::UpdatePointCloudPerformanceGovernor(&governor, sample);
    }

    CHECK(output.budgetScale == Catch::Approx(0.35F));
    CHECK(output.performanceLimited);
    CHECK(output.status == "performance-limited");
    CHECK(output.activeLimit == "fragments");
}

TEST_CASE("Beauty Adaptive quality controller uses measured GPU timing governor", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto controllerSection = SourceSection(
        applicationSource,
        "PointCloudExportDensityMode SelectAdaptiveViewportDensityMode",
        "invisible_places::renderer::pointcloud::PointCloudLodTraversalParams MakeAdaptiveLodTraversalParams");
    const auto buildRenderStateSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::core::SceneRenderState BuildRenderState",
        "int Application::Run");

    CHECK(controllerSection.find("LastAdaptivePerformanceFps") == std::string::npos);
    CHECK(controllerSection.find("kAdaptiveDemoteFps") == std::string::npos);
    CHECK(controllerSection.find("kAdaptivePromoteFps") == std::string::npos);
    CHECK(controllerSection.find("UpdateAdaptivePerformanceGovernor") != std::string::npos);
    CHECK(controllerSection.find("overTarget") != std::string::npos);
    CHECK(controllerSection.find("underTarget") != std::string::npos);
    CHECK(controllerSection.find("DemoteAdaptiveQuality") != std::string::npos);
    CHECK(controllerSection.find("PromoteAdaptiveQuality") != std::string::npos);
    CHECK(controllerSection.find("adaptiveMovementDensityMode") != std::string::npos);
    CHECK(controllerSection.find("AdaptiveHighQuality") != std::string::npos);
    CHECK(buildRenderStateSection.find("SelectAdaptiveViewportDensityMode") != std::string::npos);
}

TEST_CASE("Viewport diagnostics expose Vulkan timestamp governor surfaces", "[pointcloud][lod][source]") {
    const auto rendererHeader = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.hpp");
    const auto rendererSource = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.cpp");
    const auto appSource = ReadRepoTextFile("src/app/Application.cpp");

    CHECK(rendererHeader.find("gpuTimestampSupported") != std::string::npos);
    CHECK(rendererHeader.find("gpuFastBasicPointPassMs") != std::string::npos);
    CHECK(rendererHeader.find("gpuBeautyPointPassMs") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGovernorBudgetScale") != std::string::npos);
    CHECK(rendererSource.find("vkCreateQueryPool") != std::string::npos);
    CHECK(rendererSource.find("vkGetQueryPoolResults") != std::string::npos);
    CHECK(rendererSource.find("VK_QUERY_RESULT_WITH_AVAILABILITY_BIT") != std::string::npos);
    CHECK(rendererSource.find("vkCmdWriteTimestamp") != std::string::npos);
    CHECK(rendererSource.find("ImGuiConfigFlags_ViewportsEnable") != std::string::npos);
    CHECK(rendererSource.find("ImGui::GetPlatformIO().Monitors.Size == 0") != std::string::npos);
    CHECK(appSource.find("Adaptive preview limited by GPU") != std::string::npos);
    CHECK(appSource.find("fast_basic_max_gpu_point_pass_ms") != std::string::npos);
}

TEST_CASE("GPU-driven draw item compaction and indirect command generation are guarded and diagnosed", "[pointcloud][lod][gpu][source]") {
    const auto cmakeSource = ReadRepoTextFile("CMakeLists.txt");
    const auto compactionShader = ReadRepoTextFile("shaders/pointcloud_draw_item_compact.comp");
    const auto metadataCompactionShader = ReadRepoTextFile("shaders/pointcloud_draw_item_compact_metadata.comp");
    const auto computeShader = ReadRepoTextFile("shaders/pointcloud_indirect_command.comp");
    const auto rendererHeader = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.hpp");
    const auto rendererSource = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.cpp");
    const auto appSource = ReadRepoTextFile("src/app/Application.cpp");

    CHECK(cmakeSource.find("shaders/pointcloud_draw_item_compact.comp") != std::string::npos);
    CHECK(cmakeSource.find("shaders/pointcloud_draw_item_compact_metadata.comp") != std::string::npos);
    CHECK(cmakeSource.find("shaders/pointcloud_indirect_command.comp") != std::string::npos);
    CHECK(compactionShader.find("layout(local_size_x = 64") != std::string::npos);
    CHECK(compactionShader.find("selectionLimit") != std::string::npos);
    CHECK(compactionShader.find("selectionClassMask") != std::string::npos);
    CHECK(compactionShader.find("selectionProfileMask") != std::string::npos);
    CHECK(compactionShader.find("selectionRankLimit") != std::string::npos);
    CHECK(compactionShader.find("selectionMinDepth") != std::string::npos);
    CHECK(compactionShader.find("selectionMaxDepth") != std::string::npos);
    CHECK(compactionShader.find("selectionRequiredFlags") != std::string::npos);
    CHECK(compactionShader.find("selectionRejectedFlags") != std::string::npos);
    CHECK(compactionShader.find("selectionMinFootprintAreaBits") != std::string::npos);
    CHECK(compactionShader.find("selectionMaxRenderAreaBits") != std::string::npos);
    CHECK(compactionShader.find("selectionMinOpacityCompensationBits") != std::string::npos);
    CHECK(compactionShader.find("selectionMaxEmissionCompensationBits") != std::string::npos);
    CHECK(compactionShader.find("selectionWriteOutput") != std::string::npos);
    CHECK(compactionShader.find("selectionMaxOutputCount") != std::string::npos);
    CHECK(compactionShader.find("const bool orderedOutput = pushConstants.reserved2 != 0u") != std::string::npos);
    CHECK(compactionShader.find("orderedOutput ? drawIndex : groupBase + localOutputIndex") != std::string::npos);
    CHECK(compactionShader.find("selectionMinRepresentedSourceCount") != std::string::npos);
    CHECK(compactionShader.find("selectionMaxRepresentedSourceCount") != std::string::npos);
    CHECK(compactionShader.find("selectionPositionCount") != std::string::npos);
    CHECK(compactionShader.find("selectionFrustumGuardBandBits") != std::string::npos);
    CHECK(compactionShader.find("layout(set = 0, binding = 3) uniform FrameUniforms") != std::string::npos);
    CHECK(compactionShader.find("layout(set = 0, binding = 4") != std::string::npos);
    CHECK(compactionShader.find("shared uint groupCount") != std::string::npos);
    CHECK(compactionShader.find("shared uint groupBase") != std::string::npos);
    CHECK(compactionShader.find("drawIndex < pushConstants.selectionLimit") != std::string::npos);
    CHECK(compactionShader.find("item.indices.w >> 14u") != std::string::npos);
    CHECK(compactionShader.find("classFlags & pushConstants.selectionClassMask") != std::string::npos);
    CHECK(compactionShader.find("item.indices.w >> 22u") != std::string::npos);
    CHECK(compactionShader.find("profileBits") != std::string::npos);
    CHECK(compactionShader.find("const uint packedRank = item.indices.w & 0x7ffu") != std::string::npos);
    CHECK(compactionShader.find("packedRank <= pushConstants.selectionRankLimit") != std::string::npos);
    CHECK(compactionShader.find("const uint packedDepth = (item.indices.w >> 24u) & 0xffu") != std::string::npos);
    CHECK(compactionShader.find("packedDepth >= pushConstants.selectionMinDepth") != std::string::npos);
    CHECK(compactionShader.find("const uint packedFlags = (item.indices.w >> 11u) & 0x7u") != std::string::npos);
    CHECK(compactionShader.find("packedFlags & pushConstants.selectionRequiredFlags") != std::string::npos);
    CHECK(compactionShader.find("packedFlags & pushConstants.selectionRejectedFlags") != std::string::npos);
    CHECK(compactionShader.find("const float footprintArea = item.params.x") != std::string::npos);
    CHECK(compactionShader.find("uintBitsToFloat(pushConstants.selectionMinFootprintAreaBits)") != std::string::npos);
    CHECK(compactionShader.find("renderArea <= maxRenderArea") != std::string::npos);
    CHECK(compactionShader.find("const float opacityCompensation = item.params.y") != std::string::npos);
    CHECK(compactionShader.find("const float emissionCompensation = item.params.z") != std::string::npos);
    CHECK(compactionShader.find("representedCount >= pushConstants.selectionMinRepresentedSourceCount") != std::string::npos);
    CHECK(compactionShader.find("if (frustumGuardBand > 0.0)") != std::string::npos);
    CHECK(compactionShader.find("sourceIndex < pushConstants.selectionPositionCount") != std::string::npos);
    CHECK(compactionShader.find("uniforms.viewProjection * pointPositions.positions[sourceIndex]") != std::string::npos);
    CHECK(compactionShader.find("clipPosition.w > 0.0") != std::string::npos);
    CHECK(compactionShader.find("outputDrawItems.drawItems") != std::string::npos);
    CHECK(compactionShader.find("pushConstants.selectionWriteOutput != 0u") != std::string::npos);
    CHECK(compactionShader.find("outputIndex < pushConstants.selectionMaxOutputCount") != std::string::npos);
    CHECK(compactionShader.find("localOutputIndex = atomicAdd(groupCount") != std::string::npos);
    CHECK(compactionShader.find("groupBase = atomicAdd(stats.count, groupCount)") != std::string::npos);
    CHECK(compactionShader.find("atomicXor(stats.sourceIndexXor") != std::string::npos);
    CHECK(compactionShader.find("atomicAdd(stats.sourceIndexSum") != std::string::npos);
    CHECK(compactionShader.find("sourceFingerprintSum") == std::string::npos);
    CHECK(compactionShader.find("floatBitsToUint(item.params.x)") != std::string::npos);
    CHECK(compactionShader.find("combinedChecksum") != std::string::npos);
    CHECK(metadataCompactionShader.find("layout(local_size_x = 64") != std::string::npos);
    CHECK(metadataCompactionShader.find("selectionFrustumGuardBandBits") != std::string::npos);
    CHECK(metadataCompactionShader.find("selectionProfileMask") != std::string::npos);
    CHECK(metadataCompactionShader.find("selectionMinOpacityCompensationBits") != std::string::npos);
    CHECK(metadataCompactionShader.find("selectionWriteOutput") != std::string::npos);
    CHECK(metadataCompactionShader.find("selectionMaxOutputCount") != std::string::npos);
    CHECK(metadataCompactionShader.find("const bool orderedOutput = pushConstants.reserved2 != 0u") != std::string::npos);
    CHECK(metadataCompactionShader.find("orderedOutput ? drawIndex : groupBase + localOutputIndex") != std::string::npos);
    CHECK(metadataCompactionShader.find("shared uint groupCount") != std::string::npos);
    CHECK(metadataCompactionShader.find("localOutputIndex = atomicAdd(groupCount") != std::string::npos);
    CHECK(metadataCompactionShader.find("groupBase = atomicAdd(stats.count, groupCount)") != std::string::npos);
    CHECK(metadataCompactionShader.find("outputIndex < pushConstants.selectionMaxOutputCount") != std::string::npos);
    CHECK(metadataCompactionShader.find("uniforms.viewProjection") == std::string::npos);
    CHECK(metadataCompactionShader.find("pointPositions.positions") == std::string::npos);
    CHECK(metadataCompactionShader.find("outputDrawItems.drawItems") != std::string::npos);
    CHECK(metadataCompactionShader.find("atomicXor(stats.sourceIndexXor") != std::string::npos);
    CHECK(metadataCompactionShader.find("uint classCounts[8]") != std::string::npos);
    CHECK(metadataCompactionShader.find("shared uint groupClassCounts[8]") != std::string::npos);
    CHECK(metadataCompactionShader.find("atomicAdd(groupClassCounts[classIndex]") != std::string::npos);
    CHECK(metadataCompactionShader.find("atomicAdd(stats.classCounts[classIndex]") != std::string::npos);
    CHECK(metadataCompactionShader.find("sourceFingerprintSum") == std::string::npos);
    CHECK(metadataCompactionShader.find("combinedChecksum") != std::string::npos);
    CHECK(computeShader.find("layout(local_size_x = 1") != std::string::npos);
    CHECK(computeShader.find("layout(push_constant)") != std::string::npos);
    CHECK(computeShader.find("layout(set = 0, binding = 1") != std::string::npos);
    CHECK(computeShader.find("stats.count") != std::string::npos);
    CHECK(computeShader.find("pushConstants.command.w == 1u") != std::string::npos);
    CHECK(computeShader.find("indirectCommand.vertexCount") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionUsed") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionInputDrawItems") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionDispatchedDrawItems") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionLimit") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionProfileMask") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionClassMask") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionRankLimit") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMinDepth") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMaxDepth") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionRequiredFlags") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionRejectedFlags") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMinFootprintAreaPixels") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMaxRenderAreaPixels") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMinOpacityCompensation") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMaxEmissionCompensation") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMinRepresentedSourceCount") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionMaxRepresentedSourceCount") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionFrustumGuardBand") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionPositionCount") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionFrustumEnabled") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSelectionFrustumFallbackReason") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionCpuReferenceMs") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionOutputWriteEnabled") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionOutputWriteFallbackReason") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionOutputCapacity") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionOutputProbeParityStatus") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionOutputProbeCpuChecksum") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSubmissionEligible") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSubmissionFallbackReason") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSubmissionCandidateVertices") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionSubmissionReferenceVertices") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionCpuSourceFingerprint") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionGpuSourceFingerprint") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionCpuClassCounts") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionGpuClassCounts") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionPerformanceFallbackReason") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionPerformanceSlowFrames") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionPerformanceRetryFrames") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuIndirectCommandMs") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuIndirectCommandUsed") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveGpuCompactionIndirectCommandUsed") != std::string::npos);
    CHECK(rendererSource.find("vkCreateComputePipelines") != std::string::npos);
    CHECK(rendererSource.find("VK_PIPELINE_BIND_POINT_COMPUTE") != std::string::npos);
    CHECK(rendererSource.find("RecordGpuDrawItemCompactionForScene") != std::string::npos);
    CHECK(rendererSource.find("ReadPreviousGpuCompactionResults") != std::string::npos);
    CHECK(rendererSource.find("kGpuTimestampGpuDrawItemCompactionPass") != std::string::npos);
    CHECK(rendererSource.find("vkCmdDispatch(commandBuffer, 1, 1, 1)") != std::string::npos);
    CHECK(rendererSource.find("VK_ACCESS_INDIRECT_COMMAND_READ_BIT") != std::string::npos);
    CHECK(rendererSource.find("VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT") != std::string::npos);
    CHECK(rendererSource.find("kGpuIndirectCommandModeFromCompactionStats") != std::string::npos);
    CHECK(rendererSource.find("gpuCompactionIndirectCommandBuffers") != std::string::npos);
    CHECK(rendererSource.find("gpuCompactionExpectedIndirectCommands") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticCompactionOutputWriteEnabled") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticCompactionOutputCapacityLimit") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticCompactionOrderedOutputEnabled") != std::string::npos);
    CHECK(rendererSource.find("3'145'728U") != std::string::npos);
    CHECK(rendererSource.find("GpuDiagnosticCompactionOutputCapacity") != std::string::npos);
    CHECK(rendererSource.find("GpuDiagnosticSelectionLimit(drawPointCount)") != std::string::npos);
    CHECK(rendererSource.find("return drawPointCount;") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticCompactionOutputWriteFallbackReason") != std::string::npos);
    CHECK(rendererSource.find("GpuDrawItemOutputProbeStats") != std::string::npos);
    CHECK(rendererSource.find("orderChecksum") != std::string::npos);
    CHECK(rendererSource.find("AccumulateGpuCompactionOutputProbeStats") != std::string::npos);
    CHECK(rendererSource.find("ComputeGpuCompactionOutputProbeStatsFromBuffer") != std::string::npos);
    CHECK(rendererSource.find("submitted compacted draws require semantic-equivalent selection") != std::string::npos);
    CHECK(rendererSource.find("VK_ACCESS_HOST_READ_BIT") != std::string::npos);
    CHECK(rendererSource.find("passed previous-frame ordered compacted output identity") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticCompactionSlowFrameThreshold") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticCompactionRetryCooldownFrames") != std::string::npos);
    CHECK(rendererSource.find("GpuCompactionPerformanceFallbackReason") != std::string::npos);
    CHECK(rendererSource.find("UpdateGpuCompactionPerformanceGate") != std::string::npos);
    CHECK(rendererSource.find("ResetGpuCompactionPerformanceFrame") != std::string::npos);
    CHECK(rendererSource.find("GPU full-range compaction performance fallback") != std::string::npos);
    CHECK(rendererSource.find("cpu-selection+gpu-generated-indirect") != std::string::npos);
    CHECK(rendererSource.find("GpuDiagnosticSelectionLimit") != std::string::npos);
    CHECK(rendererSource.find("dispatchItemCount") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticSemanticSelectionClassMask") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticSemanticSelectionProfileMask") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticRankSelectionLimit") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticMinSelectionDepth") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticRequiredSelectionFlags") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticMinSelectionFootprintAreaPixels") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticMinSelectionOpacityCompensation") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticMinSelectionRepresentedSourceCount") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticSelectionFrustumGuardBand") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticSelectionFrustumGuardEnabled") != std::string::npos);
    CHECK(rendererSource.find("kGpuDiagnosticSelectionFrustumFallbackReason") != std::string::npos);
    CHECK(rendererSource.find("pointcloud_draw_item_compact_metadata.comp.spv") != std::string::npos);
    CHECK(rendererSource.find("DrawItemWithinProjectedAreaWindow") != std::string::npos);
    CHECK(rendererSource.find("DrawItemWithinCompensationWindow") != std::string::npos);
    CHECK(rendererSource.find("DrawItemWithinRepresentedSourceWindow") != std::string::npos);
    CHECK(rendererSource.find("DrawItemWithinFrustumGuard") != std::string::npos);
    CHECK(rendererSource.find("cpuReferenceStart") != std::string::npos);
    CHECK(rendererSource.find("foldSourceFingerprint") != std::string::npos);
    CHECK(rendererSource.find("bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER") != std::string::npos);
    CHECK(rendererSource.find("FloatBits(selectionMinFootprintAreaPixels)") != std::string::npos);
    CHECK(rendererSource.find("DrawItemRepresentativeClassFlags") != std::string::npos);
    CHECK(rendererSource.find("classCounts == expected.classCounts") != std::string::npos);
    CHECK(rendererSource.find("passed previous-frame full-range count/source-fingerprint/checksum/class-counts") != std::string::npos);
    CHECK(rendererSource.find("selectionOrderedOutput ? 1U : 0U") != std::string::npos);
    CHECK(rendererSource.find("DrawItemRendererCostProfileBit") != std::string::npos);
    CHECK(rendererSource.find("DrawItemRepresentativePackedRank") != std::string::npos);
    CHECK(rendererSource.find("DrawItemRepresentativePackedDepth") != std::string::npos);
    CHECK(rendererSource.find("DrawItemRepresentativePackedFlags") != std::string::npos);
    CHECK(rendererSource.find("cpu-selection+gpu-full-range-selection-compare+gpu-generated-indirect") != std::string::npos);
    CHECK(rendererSource.find("workgroup-aggregated") != std::string::npos);
    CHECK(rendererSource.find("const bool useGpuGeneratedIndirectCommand") != std::string::npos);
    CHECK(rendererSource.find("if (!useGpuGeneratedIndirectCommand)") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_used") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_input_draw_items") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_dispatched_draw_items") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_profile_mask") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_class_mask") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_rank_limit") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_min_depth") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_min_footprint_area_pixels") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_output_write_enabled") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_output_write_fallback_reason") != std::string::npos);
    CHECK(appSource.find("gpu_compaction_output_capacity") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_output_capacity") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_output_probe_parity_status") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_output_probe_cpu_checksum") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_submission_eligible") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_submission_fallback_reason") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_submission_candidate_vertices") != std::string::npos);
    CHECK(appSource.find("gpu_compaction_submission_reference_vertices") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_max_render_area_pixels") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_min_opacity_compensation") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_max_emission_compensation") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_min_represented_source_count") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_frustum_guard_band") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_frustum_enabled") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_frustum_fallback_reason") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_cpu_reference_ms") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_cpu_source_fingerprint") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_gpu_source_fingerprint") != std::string::npos);
    CHECK(appSource.find("writeGpuCompactionClassCountMetrics") != std::string::npos);
    CHECK(appSource.find("_cpu_class_") != std::string::npos);
    CHECK(appSource.find("_gpu_class_") != std::string::npos);
    CHECK(appSource.find("blue_noise_fill") != std::string::npos);
    CHECK(appSource.find("gpu_compaction_performance_fallback_reason") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_performance_status") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_performance_fallback_reason") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_performance_retry_frames") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_selection_required_flags") != std::string::npos);
    CHECK(appSource.find("fast_basic_compaction_indirect_command_used") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_used") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_input_draw_items") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_dispatched_draw_items") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_profile_mask") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_class_mask") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_rank_limit") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_min_depth") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_min_footprint_area_pixels") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_max_render_area_pixels") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_min_opacity_compensation") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_max_emission_compensation") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_min_represented_source_count") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_frustum_guard_band") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_frustum_enabled") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_frustum_fallback_reason") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_output_write_enabled") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_output_write_fallback_reason") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_output_capacity") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_output_probe_parity_status") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_output_probe_cpu_checksum") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_submission_eligible") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_submission_fallback_reason") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_submission_candidate_vertices") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_cpu_reference_ms") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_cpu_source_fingerprint") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_gpu_source_fingerprint") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_performance_status") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_performance_fallback_reason") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_performance_retry_frames") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_selection_required_flags") != std::string::npos);
    CHECK(appSource.find("beauty_stress_compaction_indirect_command_used") != std::string::npos);
    CHECK(appSource.find("fast_basic_indirect_command_used") != std::string::npos);
    CHECK(appSource.find("beauty_stress_indirect_command_used") != std::string::npos);
}

TEST_CASE("Fast Basic adaptive viewport targets high quality and reserves preview for emergency demotion", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto controllerSection = SourceSection(
        applicationSource,
        "PointCloudExportDensityMode SelectAdaptiveViewportDensityMode",
        "invisible_places::renderer::pointcloud::PointCloudLodTraversalParams MakeAdaptiveLodTraversalParams");
    const auto buildRenderStateSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::core::SceneRenderState BuildRenderState",
        "int Application::Run");

    CHECK(controllerSection.find("fastBasicViewport") != std::string::npos);
    CHECK(controllerSection.find("PointCloudExportDensityMode::FastAdaptivePreview") != std::string::npos);
    CHECK(controllerSection.find("DemoteAdaptiveQuality") != std::string::npos);
    CHECK(controllerSection.find("fastBasicIdleCeiling") == std::string::npos);
    CHECK(controllerSection.find("next == highQuality") == std::string::npos);
    CHECK(controllerSection.find("std::chrono::milliseconds{120}") != std::string::npos);
    CHECK(controllerSection.find("std::chrono::milliseconds{300}") != std::string::npos);
    CHECK(controllerSection.find("runtimeState.cameraInteraction.navigationActive") != std::string::npos);
    CHECK(controllerSection.find("adaptiveInteractionActive") != std::string::npos);
    CHECK(controllerSection.find("session->adaptiveLodEmergencyDemotion") != std::string::npos);
    CHECK(controllerSection.find("session->adaptiveLodAsyncPending") == std::string::npos);
    CHECK(controllerSection.find("session->adaptiveLodFallbackState == \"coarse fallback\"") == std::string::npos);
    CHECK(controllerSection.find("setQuality(PointCloudExportDensityMode::FastAdaptivePreview);\n        return PointCloudExportDensityMode::FastAdaptivePreview;") == std::string::npos);
    CHECK(buildRenderStateSection.find("FastBasicPointRendererActive(runtimeState.projectSettings)") != std::string::npos);
}

TEST_CASE("Streaming ipcloud previews stay in bounded fast adaptive mode", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto helperSection = SourceSection(
        applicationSource,
        "bool PointCloudStreamingPreviewActive",
        "bool PointCloudNeedsExactSourceLoad");
    const auto controllerSection = SourceSection(
        applicationSource,
        "PointCloudExportDensityMode SelectAdaptiveViewportDensityMode",
        "invisible_places::renderer::pointcloud::PointCloudLodTraversalParams MakeAdaptiveLodTraversalParams");
    const auto traversalSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::pointcloud::PointCloudLodTraversalParams MakeAdaptiveLodTraversalParams",
        "std::uint64_t AdaptiveLodTraversalKey");

    CHECK(helperSection.find("pointCloudPreviewActive") != std::string::npos);
    CHECK(helperSection.find("pointIpcloudStreamingActive") != std::string::npos);
    CHECK(helperSection.find("!PointCloudExactSourceResident(session)") != std::string::npos);
    CHECK(applicationSource.find("kStreamingPreviewMaxAdaptiveDrawItems = 524'288U") != std::string::npos);
    CHECK(controllerSection.find("PointCloudStreamingPreviewActive(*session)") != std::string::npos);
    CHECK(controllerSection.find("const auto streamingQuality = AdaptiveViewportQualityMode(paintedAdaptive)") !=
          std::string::npos);
    CHECK(controllerSection.find("setQuality(streamingQuality)") != std::string::npos);
    CHECK(controllerSection.find("session->adaptiveLodEmergencyDemotion = false") != std::string::npos);
    CHECK(traversalSection.find("PointCloudStreamingPreviewActive(session)") != std::string::npos);
    CHECK(traversalSection.find("std::min(params.maxDrawItems, streamingCap)") != std::string::npos);
    CHECK(traversalSection.find("std::min(params.maxRepresentatives, streamingCap)") != std::string::npos);
    CHECK(traversalSection.find("BaseAdaptiveTargetSpacingPixels(AdaptiveViewportQualityMode(") !=
          std::string::npos);
    CHECK(traversalSection.find("!deterministicExport && PointCloudStreamingPreviewActive(session)") !=
          std::string::npos);
}

TEST_CASE("Adaptive ipcloud streaming restores after Source mode and reuses resident uploads", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto ipcloudCacheSource = ReadRepoTextFile("src/renderer/pointcloud/PointCloudIpcloudCache.cpp");
    const auto restoreSection = SourceSection(
        applicationSource,
        "bool RestorePointCloudIpcloudAdaptivePreview",
        "bool ActivateLoadedGaussianSplats");
    const auto populateSection = SourceSection(
        applicationSource,
        "bool PopulateAdaptivePointCloudLayer",
        "AdaptiveLodPreviewSummary ComputeAdaptiveLodPreviewSummary");
    const auto buildRenderStateSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::core::SceneRenderState BuildRenderState",
        "struct LodComparisonImageMetrics");
    const auto residentSetSection = SourceSection(
        ipcloudCacheSource,
        "PointCloudIpcloudResidentSet BuildPointCloudIpcloudResidentSet",
        "PointCloudIpcloudSaveResult SavePointCloudIpcloudBundle");

    CHECK(restoreSection.find("PointCloudExactSourceResident(session)") != std::string::npos);
    CHECK(restoreSection.find("InspectPointCloudIpcloudBundle") != std::string::npos);
    CHECK(restoreSection.find("PointCloudIpcloudCacheState::Hit") != std::string::npos);
    CHECK(restoreSection.find("LoadPointCloudIpcloudPreview") != std::string::npos);
    CHECK(restoreSection.find("ActivatePointCloudPreview") != std::string::npos);
    CHECK(restoreSection.find("restored warm .ipcloud adaptive streaming after Source mode") !=
          std::string::npos);
    CHECK(applicationSource.find("interactiveViewportRuntime = true") != std::string::npos);
    CHECK(buildRenderStateSection.find("runtimeState.interactiveViewportRuntime") != std::string::npos);
    CHECK(buildRenderStateSection.find("!fullSourcePointRenderer &&") != std::string::npos);
    CHECK(buildRenderStateSection.find("PointCloudExactSourceResident(session)") !=
          std::string::npos);
    CHECK(buildRenderStateSection.find("RestorePointCloudIpcloudAdaptivePreview") != std::string::npos);
    CHECK(populateSection.find("pointIpcloudResidentInputRevision == result.revision") != std::string::npos);
    CHECK(populateSection.find("pointIpcloudResidentUploadBudgetBytes == uploadBudgetBytes") != std::string::npos);
    CHECK(populateSection.find("session->pointIpcloudResidentDrawItems = adaptiveDrawItems") != std::string::npos);
    CHECK(populateSection.find("ClearPointCloudStreamingResidentCache(session)") != std::string::npos);
    CHECK(ipcloudCacheSource.find("CountPointCloudIpcloudRawChunkTargetHits") != std::string::npos);
    CHECK(residentSetSection.find("targetSourceIndices.insert(drawItem.sourcePointIndex)") != std::string::npos);
    CHECK(residentSetSection.find("std::stable_sort") != std::string::npos);
    CHECK(residentSetSection.find("leftHits > rightHits") != std::string::npos);
}

TEST_CASE("Fast Basic adaptive budgets use a dense renderer profile", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto budgetSection = SourceSection(
        applicationSource,
        "std::uint32_t AdaptiveViewportRepresentativeBudget",
        "PointCloudExportDensityMode AdaptiveHighQualityMode");
    const auto traversalSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::pointcloud::PointCloudLodTraversalParams MakeAdaptiveLodTraversalParams",
        "std::uint64_t AdaptiveLodTraversalKey");

    CHECK(budgetSection.find("fastBasicProfile ? pixelCount / 8ULL : pixelCount / 96ULL") != std::string::npos);
    CHECK(budgetSection.find("fastBasicProfile ? pixelCount / 2ULL : pixelCount / 8ULL") != std::string::npos);
    CHECK(budgetSection.find("fastBasicProfile ? pixelCount : pixelCount / 3ULL") != std::string::npos);
    CHECK(budgetSection.find("4'000'000ULL") != std::string::npos);
    CHECK(budgetSection.find("16'000'000ULL") != std::string::npos);
    CHECK(budgetSection.find("fastBasicProfile ? 32.0F : 8.0F") != std::string::npos);
    CHECK(budgetSection.find("fastBasicProfile ? 96.0F : 16.0F") != std::string::npos);
    CHECK(traversalSection.find("fastBasicProfile") != std::string::npos);
    CHECK(traversalSection.find("session.totalPrimitives") != std::string::npos);
}

TEST_CASE("Fast Basic Source mode serializes and uses full source through Fast Basic", "[pointcloud][lod][source]") {
    using invisible_places::renderer::pointcloud::PointCloudRendererMode;
    using invisible_places::renderer::pointcloud::PointCloudRendererModeUsesFastBasic;
    using invisible_places::renderer::pointcloud::PointCloudRendererModeUsesFullSource;

    CHECK(PointCloudRendererModeUsesFastBasic(PointCloudRendererMode::FastBasicSource));
    CHECK(PointCloudRendererModeUsesFullSource(PointCloudRendererMode::FastBasicSource));
    CHECK(PointCloudRendererModeUsesFastBasic(PointCloudRendererMode::FastBasic));
    CHECK_FALSE(PointCloudRendererModeUsesFullSource(PointCloudRendererMode::FastBasic));

    const auto appSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto projectSource = ReadRepoTextFile("src/serialization/ProjectDocument.cpp");
    const auto rendererSource = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.cpp");
    CHECK(appSource.find("Fast Basic Source") != std::string::npos);
    CHECK(projectSource.find("\"fast_basic_source\"") != std::string::npos);
    CHECK(appSource.find("exactFullSourceReady ? session.totalPrimitives : 0ULL") != std::string::npos);
    CHECK(appSource.find("PointCloudExactSourceResident") != std::string::npos);
    CHECK(appSource.find("PointCloudRendererModeUsesFullSource(rendererMode)") != std::string::npos);
    CHECK(rendererSource.find("\"fast-basic-source\"") != std::string::npos);
}

TEST_CASE("Fast Basic coarse fallback is bounded but richer than emergency preview", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto fallbackSection = SourceSection(
        applicationSource,
        "std::uint32_t CoarseAdaptiveFallbackRepresentativeTarget",
        "std::vector<PointCloudDrawItemGpu> BuildCoarseAdaptiveFallbackDrawItems");
    const auto buildItemsSection = SourceSection(
        applicationSource,
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems",
        "bool PopulateAdaptivePointCloudLayer");

    CHECK(fallbackSection.find("pixelCount / 24ULL") != std::string::npos);
    CHECK(fallbackSection.find("262'144ULL") != std::string::npos);
    CHECK(buildItemsSection.find("coarse fallback for ") != std::string::npos);
    CHECK(buildItemsSection.find("showing previous lower-tier set") != std::string::npos);
}

TEST_CASE("Fast Basic navigation reuses adaptive LOD instead of rebuilding fallback every camera key", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto reusableSection = SourceSection(
        applicationSource,
        "AdaptiveLodCacheState* FindReusableAdaptiveLodCacheEntry",
        "double CurrentAdaptiveLodPendingAgeMs");
    const auto buildItemsSection = SourceSection(
        applicationSource,
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems",
        "bool PopulateAdaptivePointCloudLayer");
    const auto pollSection = SourceSection(
        applicationSource,
        "void PollPointCloudLodWorkers",
        "ImVec2 CurrentUiViewportOrigin");

    CHECK(reusableSection.find("allowCoarseFallback") != std::string::npos);
    CHECK(reusableSection.find("bestCoarseFallback") != std::string::npos);
    CHECK(reusableSection.find("denseFastBasicNavigation ? 0ULL") != std::string::npos);
    CHECK(buildItemsSection.find("std::numeric_limits<std::uint64_t>::max()") != std::string::npos);
    CHECK(buildItemsSection.find("motion hold reused exact") != std::string::npos);
    CHECK(buildItemsSection.find("motion hold reused coarse fallback") != std::string::npos);
    CHECK(pollSection.find("kAdaptiveMotionApplyInterval") != std::string::npos);
    CHECK(pollSection.find("completionApplyThrottled") != std::string::npos);
    CHECK(pollSection.find("adaptiveLodMotionApplyThrottleCount") != std::string::npos);
}

TEST_CASE("Adaptive LOD blocks idle demotion below the displayed base", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto buildItemsSection = SourceSection(
        applicationSource,
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems",
        "bool PopulateAdaptivePointCloudLayer");
    const auto pollSection = SourceSection(
        applicationSource,
        "void PollPointCloudLodWorkers",
        "ImVec2 CurrentUiViewportOrigin");

    CHECK(applicationSource.find("AdaptiveLodCandidateWouldDemoteDisplayed") != std::string::npos);
    CHECK(buildItemsSection.find("holding displayed base, coarse fallback blocked") != std::string::npos);
    CHECK(buildItemsSection.find("idle no-flash refine") != std::string::npos);
    CHECK(pollSection.find("candidate demotion blocked; idle no-flash refine") != std::string::npos);
    CHECK(pollSection.find("adaptiveLodCandidateDemotionBlockedCount") != std::string::npos);
}

TEST_CASE("Adaptive LOD motion traversals complete as coverage patches", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto scheduleSection = SourceSection(
        applicationSource,
        "bool ScheduleAdaptiveLodTraversal",
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems");
    const auto pollSection = SourceSection(
        applicationSource,
        "void PollPointCloudLodWorkers",
        "ImVec2 CurrentUiViewportOrigin");

    CHECK(scheduleSection.find("supersede throttled for motion patch") != std::string::npos);
    CHECK(scheduleSection.find("return true;") < scheduleSection.find("request_stop()"));
    CHECK(pollSection.find("motion coverage patch applied") != std::string::npos);
    CHECK(pollSection.find("adaptiveLodMotionPatch") != std::string::npos);
    CHECK(pollSection.find("adaptiveLodCoverageGapSourceCount") != std::string::npos);
    CHECK(pollSection.find("candidateVisibleSourceCount > displayedVisibleSourceCount") != std::string::npos);
}

TEST_CASE("LOD comparison command writes full-source and adaptive image metrics", "[pointcloud][lod][source]") {
    const auto applicationHeader = ReadRepoTextFile("src/app/Application.hpp");
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto mainSource = ReadRepoTextFile("src/main.cpp");
    const auto comparisonSection = SourceSection(
        applicationSource,
        "int Application::RunLodComparison",
        "int Application::Run() const");

    CHECK(applicationHeader.find("RunLodComparison") != std::string::npos);
    CHECK(mainSource.find("--lod-compare") != std::string::npos);
    CHECK(comparisonSection.find("Site3-Sample-Terrestrial.ply") != std::string::npos);
    CHECK(comparisonSection.find("beauty_full_source.exr") != std::string::npos);
    CHECK(comparisonSection.find("beauty_adaptive_hq.exr") != std::string::npos);
    CHECK(comparisonSection.find("beauty_adaptive_hq_error.exr") != std::string::npos);
    CHECK(comparisonSection.find("BeautyLodComparisonCase") != std::string::npos);
    CHECK(comparisonSection.find("small_opaque_sprites") != std::string::npos);
    CHECK(comparisonSection.find("large_translucent_gaussian_sprites") != std::string::npos);
    CHECK(comparisonSection.find("emissive_scalar_sprites") != std::string::npos);
    CHECK(comparisonSection.find("world_sized_sprites") != std::string::npos);
    CHECK(comparisonSection.find("world_surfels") != std::string::npos);
    CHECK(comparisonSection.find("lod_compare_metrics.json") != std::string::npos);
    CHECK(comparisonSection.find("\\\"metrics_schema_version\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"difference_exr\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"density_policy\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"determinism\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_selection_hash\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"full_source_image_hash\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_image_hash\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"difference_image_hash\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"rgb_mae\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"alpha_rmse\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"process_peak_resident_memory_bytes\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_upload_budget_bytes\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_visible_chunks\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_fallback_reason\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"beauty_matrix\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"renderer_cost_profile\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_estimated_vertices\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_estimated_blended_fragments\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_tile_budget_enabled\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_tile_limited_representatives\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_occlusion_culling_state\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_radius_scale_min\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_opacity_scale_max\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_emission_scale_max\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_performance_clamped\\\"") != std::string::npos);
    CHECK(comparisonSection.find("Adaptive High Quality") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_fallback\\\": false") != std::string::npos);
    CHECK(comparisonSection.find("PointCloudRendererMode::FastBasic") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_max_submitted_points\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_max_estimated_fragments\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_max_visible_represented_source\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_class_colour_contrast\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_class_scalar_max\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_class_normal_edge\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"adaptive_class_emissive_accent\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_exact_class_colour_contrast\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_exact_class_scalar_max\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_exact_class_normal_edge\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_exact_class_emissive_accent\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_scalar_feature_refinements\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_exact_scalar_feature_refinements\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_zoom_out_updated\\\"") != std::string::npos);
    CHECK(comparisonSection.find("kFastBasicZoomOutStartFrame") != std::string::npos);
    CHECK(comparisonSection.find("\\\"fast_basic_submitted_full_source\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"beauty_stress_tile_limited_representatives\\\"") != std::string::npos);
    CHECK(comparisonSection.find("\\\"beauty_stress_occlusion_culling_state\\\"") != std::string::npos);
}

TEST_CASE("Adaptive export density modes use deterministic explicit policies", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto renderPresetHeader = ReadRepoTextFile("src/output/RenderPreset.hpp");
    const auto renderPresetSource = ReadRepoTextFile("src/output/RenderPreset.cpp");
    const auto traversalParamsSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::pointcloud::PointCloudLodTraversalParams MakeAdaptiveLodTraversalParams",
        "std::uint64_t AdaptiveLodTraversalKey");
    const auto traversalKeySection = SourceSection(
        applicationSource,
        "std::uint64_t AdaptiveLodTraversalKey",
        "AdaptiveLodCacheState* FindAdaptiveLodCacheEntry");
    const auto adaptiveBuildSection = SourceSection(
        applicationSource,
        "AdaptiveLodBuildResult BuildAdaptivePointCloudDrawItems",
        "void RememberDisplayedAdaptiveLodEntry");
    const auto exrRenderStateSection = SourceSection(
        applicationSource,
        "invisible_places::renderer::core::SceneRenderState BuildPointCloudExrRenderState",
        "std::uint64_t CurrentResidentMemoryBytes");
    const auto stillExportSection = SourceSection(
        applicationSource,
        "void StartStillCameraExportJob",
        "void StartAnimationExportJob");
    const auto animationExportSection = SourceSection(
        applicationSource,
        "void StartAnimationExportJob",
        "void FinishOfflineRenderJob");
    const auto exportLogSection = SourceSection(
        applicationSource,
        "std::string WriteExportLog",
        "void UpdateOfflineRenderProgress");

    CHECK(renderPresetHeader.find("PointCloudExportDensityModePolicy") != std::string::npos);
    CHECK(renderPresetSource.find("requiresViewportSnapshot = true") != std::string::npos);
    CHECK(renderPresetSource.find("deterministicSelection = true") != std::string::npos);
    CHECK(traversalParamsSection.find("deterministicExport") != std::string::npos);
    CHECK(traversalParamsSection.find("deterministic export fixed budget") != std::string::npos);
    CHECK(traversalParamsSection.find("!deterministicExport && manualCap") != std::string::npos);
    CHECK(traversalKeySection.find("HashBool(&seed, deterministicExport)") != std::string::npos);
    CHECK(adaptiveBuildSection.find("if (!deterministicExport)") != std::string::npos);
    CHECK(applicationSource.find("EffectiveExportPointCloudRendererMode") != std::string::npos);
    CHECK(applicationSource.find("Full Source export needs exact source data") != std::string::npos);
    CHECK(exrRenderStateSection.find("PointCloudExportDensityModeRequiresViewportSnapshot") != std::string::npos);
    CHECK(exrRenderStateSection.find("ApplyMatchViewportExportSnapshot") != std::string::npos);
    CHECK(exrRenderStateSection.find("true,\n                static_cast<std::uint64_t>(job.currentFrame)") !=
          std::string::npos);
    CHECK(stillExportSection.find("CaptureMatchViewportExportSnapshots") != std::string::npos);
    CHECK(animationExportSection.find("Animation EXR paths need Adaptive High Quality or Fast Adaptive Preview") !=
          std::string::npos);
    CHECK(exportLogSection.find("Point density policy") != std::string::npos);
    CHECK(exportLogSection.find("Selection signature") != std::string::npos);
    CHECK(exportLogSection.find("Adaptive streamed chunks visible/resident/missing max") != std::string::npos);
}

TEST_CASE("Adaptive LOD summary reads cached metadata without retraversal", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto summarySection = SourceSection(
        applicationSource,
        "AdaptiveLodPreviewSummary ComputeAdaptiveLodPreviewSummary",
        "ImVec2 CurrentUiViewportOrigin");

    CHECK(summarySection.find("adaptiveLodCache") != std::string::npos);
    CHECK(summarySection.find("TraversePointCloudLodHierarchy") == std::string::npos);
    CHECK(summarySection.find("BuildAdaptivePointCloudDrawItems") == std::string::npos);
}

TEST_CASE("Persistent and runtime LOD cache status is surfaced in diagnostics", "[pointcloud][lod][source]") {
    const auto applicationSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto rendererHeader = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.hpp");

    CHECK(applicationSource.find("PointCloudLodCacheDirectory") != std::string::npos);
    CHECK(applicationSource.find("LoadPointCloudLodHierarchyCache") != std::string::npos);
    CHECK(applicationSource.find("SavePointCloudLodHierarchyCache") != std::string::npos);
    CHECK(applicationSource.find("LOD cache ready") != std::string::npos);
    CHECK(applicationSource.find("LOD cache stale") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveLodPersistentCacheStatus") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveLodRequestedDensity") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveLodDisplayedDensity") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveVisibleRepresentedSourceCount") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveCulledRepresentedSourceCount") != std::string::npos);
    CHECK(rendererHeader.find("adaptiveLodDisplayedCacheAgeFrames") != std::string::npos);
    CHECK(applicationSource.find("Represented visible/emitted/culled") != std::string::npos);
}

TEST_CASE("Adaptive LOD shaders apply draw-item footprint", "[pointcloud][lod][shader]") {
    const std::array<std::filesystem::path, 5> drawItemShaders = {
        std::filesystem::path{"shaders/pointcloud_preview.vert"},
        std::filesystem::path{"shaders/pointcloud_fast_basic.vert"},
        std::filesystem::path{"shaders/pointcloud_constant_simple.vert"},
        std::filesystem::path{"shaders/pointcloud_surfel.vert"},
        std::filesystem::path{"shaders/pointcloud_surfel_constant_simple.vert"},
    };

    for (const auto& shaderPath : drawItemShaders) {
        const auto shaderSource = ReadRepoTextFile(shaderPath);
        CHECK(shaderSource.find("drawItems[drawIndex].params.w") != std::string::npos);
        CHECK(shaderSource.find("DrawItemFootprintDiameterPixels") != std::string::npos);
        CHECK(shaderSource.find("pointDrawItems.drawItems[drawIndex].indices.x") != std::string::npos);
        if (shaderPath.string().find("surfel") != std::string::npos) {
            CHECK(shaderSource.find("DrawItemFootprintWorldDiameter") != std::string::npos);
            CHECK(shaderSource.find("ScreenPointSizePixelsToWorldDiameter") != std::string::npos);
            CHECK(shaderSource.find("encodedVertexIndex / kSurfelVerticesPerPoint") != std::string::npos);
            CHECK(shaderSource.find("drawIndex * kSurfelVerticesPerPoint") != std::string::npos);
        } else {
            CHECK(shaderSource.find("gl_PointSize = clamp(") != std::string::npos);
            CHECK(shaderSource.find("DrawItemFootprintDiameterPixels(drawIndex)") != std::string::npos);
        }
    }
}

TEST_CASE("Adaptive draw-item uploads are grow-only and avoid device idle", "[pointcloud][lod][source]") {
    const auto rendererSource = ReadRepoTextFile("src/renderer/core/VulkanViewportShell.cpp");
    const auto updateSection = SourceSection(
        rendererSource,
        "bool VulkanViewportShell::UpdatePointCloudDrawItemBuffer",
        "bool VulkanViewportShell::UpdatePointCloudExrDrawItemBuffer");
    const auto descriptorRewrite = updateSection.find("UpdatePointCloudDescriptorSet");
    const auto reallocationGuard = updateSection.find("if (reallocated)");

    CHECK(updateSection.find("WaitIdle") == std::string::npos);
    CHECK(updateSection.find("vkDeviceWaitIdle") == std::string::npos);
    CHECK(updateSection.find("drawItemCapacities") != std::string::npos);
    REQUIRE(descriptorRewrite != std::string::npos);
    REQUIRE(reallocationGuard != std::string::npos);
    CHECK(reallocationGuard < descriptorRewrite);
}

TEST_CASE("Point renderer controls expose adaptive modes in the Visuals tab", "[ui][pointcloud]") {
    const auto source = ReadRepoTextFile("src/app/Application.cpp");
    const auto rendererSection = SourceSection(
        source,
        "void DrawPointRendererPanel",
        "void DrawWaterEffectStackVisualsSection");
    CHECK(rendererSection.find("\"Fast Basic\"") != std::string::npos);
    CHECK(rendererSection.find("\"Fast Basic Source\"") != std::string::npos);
    CHECK(rendererSection.find("\"Beauty Adaptive\"") != std::string::npos);
    CHECK(rendererSection.find("\"Beauty Full Source\"") != std::string::npos);
    CHECK(rendererSection.find("\"Painted Adaptive\"") != std::string::npos);
    CHECK(rendererSection.find("ImGui::Checkbox(\"Eye-Dome Lighting\"") != std::string::npos);
    CHECK(rendererSection.find("ImGui::Checkbox(\"Constant Update View\"") != std::string::npos);
    CHECK(rendererSection.find("ImGui::Checkbox(\"Live Visual Effects\"") != std::string::npos);

    const auto projectSection = SourceSection(
        source,
        "void DrawProjectPanel",
        "ImVec2 DefaultControlsWindowPosition");
    CHECK(projectSection.find("ImGui::Checkbox(\"Eye-Dome Lighting\"") == std::string::npos);
    CHECK(projectSection.find("ImGui::Checkbox(\"Constant Update View\"") == std::string::npos);
    CHECK(projectSection.find("ImGui::Checkbox(\"Live Visual Effects\"") == std::string::npos);
}

TEST_CASE("LiDAR panel exposes manual adaptive LOD cache rebuild controls", "[ui][pointcloud][lod]") {
    const auto source = ReadRepoTextFile("src/app/Application.cpp");
    CHECK(source.find("bool rebuildPointLodCacheOnLoad") != std::string::npos);
    CHECK(source.find("RemovePointCloudLodCacheFiles") != std::string::npos);
    CHECK(source.find("PointCloudExactSourceResident") != std::string::npos);
    CHECK(source.find("BeginPointCloudExactSourceLoad") != std::string::npos);
    CHECK(source.find("RebuildSelectedPointCloudLodCache") != std::string::npos);
    CHECK(source.find("session.rebuildPointLodCacheOnLoad") != std::string::npos);
    CHECK(source.find("\"Rebuild LOD Cache On Load\"") != std::string::npos);
    CHECK(source.find("\"Rebuild LOD Cache Now\"") != std::string::npos);
    CHECK(source.find("Waiting for exact source before LOD rebuild") != std::string::npos);
    CHECK(source.find("Source modes wait for exact source data") != std::string::npos);
    CHECK(source.find("session.pointLodCacheSource.sourceSizeBytes > 0ULL && session.rebuildPointLodCacheOnLoad") !=
          std::string::npos);
}

TEST_CASE("LiDAR loading and LOD cache builds expose progress and ETA", "[ui][pointcloud][lod]") {
    const auto appSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto pointCloudHeader = ReadRepoTextFile("src/io/PointCloudData.hpp");
    const auto lodHeader = ReadRepoTextFile("src/renderer/pointcloud/PointCloudLodHierarchy.hpp");
    const auto lodSource = ReadRepoTextFile("src/renderer/pointcloud/PointCloudLodHierarchy.cpp");

    CHECK(pointCloudHeader.find("struct PointCloudLoadProgress") != std::string::npos);
    CHECK(pointCloudHeader.find("PointCloudLoadProgressCallback") != std::string::npos);
    CHECK(appSource.find("pointCloudProgress") != std::string::npos);
    CHECK(appSource.find("DrawLoadProgressLines") != std::string::npos);
    CHECK(appSource.find("FormatEta") != std::string::npos);

    CHECK(lodHeader.find("struct PointCloudLodBuildProgress") != std::string::npos);
    CHECK(lodHeader.find("PointCloudLodBuildProgressCallback") != std::string::npos);
    CHECK(lodSource.find("PointCloudLodBuildProgressTracker") != std::string::npos);
    CHECK(appSource.find("DrawLodBuildProgressLines") != std::string::npos);
    CHECK(appSource.find("CurrentPointCloudLodBuildProgress") != std::string::npos);
    CHECK(appSource.find("BuildPointCloudLodHierarchy(\n                *cloud,\n                buildConfig,\n                progressCallback)") !=
          std::string::npos);
}

TEST_CASE(".ipcloud cache check CLI reports cold, warm, and resume diagnostics", "[pointcloud][lod][cache]") {
    const auto appSource = ReadRepoTextFile("src/app/Application.cpp");
    const auto mainSource = ReadRepoTextFile("src/main.cpp");

    CHECK(mainSource.find("--lod-cache-check") != std::string::npos);
    CHECK(appSource.find("int Application::RunLodCacheCheck") != std::string::npos);
    CHECK(appSource.find("lod_cache_metrics.json") != std::string::npos);
    CHECK(appSource.find("cold_cache_state") != std::string::npos);
    CHECK(appSource.find("warm_cache_state") != std::string::npos);
    CHECK(appSource.find("resume_resumable") != std::string::npos);
    CHECK(appSource.find("time_to_first_coarse_frame_ms") != std::string::npos);
    CHECK(appSource.find("LoadPointCloudIpcloudSourcePreview") != std::string::npos);
    CHECK(appSource.find("LoadPointCloudIpcloudPreview") != std::string::npos);
}

TEST_CASE("LOD HUD reports adaptive density fallback and hierarchy details", "[ui][pointcloud][lod]") {
    const auto source = ReadRepoTextFile("src/app/Application.cpp");
    const auto statusOverlay = SourceSection(
        source,
        "void DrawStatusOverlay",
        "void DrawPivotOverlay");
    CHECK(statusOverlay.find("\"LOD density: %s / %s\"") != std::string::npos);
    CHECK(statusOverlay.find("\"LOD reps: %s for %s source (%s)\"") != std::string::npos);
    CHECK(statusOverlay.find("coarseFallback ? \"fallback\" : \"exact\"") != std::string::npos);

    const auto lodDebugSection = SourceSection(
        source,
        "void DrawPointCloudLodDebugLines",
        "void DrawStatusOverlay");
    CHECK(lodDebugSection.find("\"Hierarchy: %s source, %s nodes, %s reps, gen %s\"") != std::string::npos);
    CHECK(lodDebugSection.find("\"Density requested/displayed: %s / %s\"") != std::string::npos);
    CHECK(lodDebugSection.find("\"Movement tier: %s\"") != std::string::npos);
    CHECK(lodDebugSection.find("\"Displayed: %s | key %llx | revision %s\"") != std::string::npos);
    CHECK(lodDebugSection.find("\"Budget: %s reps, %.0f fragments | estimated %.0f fragments\"") != std::string::npos);
    CHECK(lodDebugSection.find("\"Async pending age: %.1f ms | stale discarded: %s | motion apply holds: %s\"") !=
          std::string::npos);
    CHECK(lodDebugSection.find("\"Motion patches: %s applied, %s reps last, %s source gap | demotion blocks: %s\"") !=
          std::string::npos);
    CHECK(lodDebugSection.find("\"Displayed base/patch/candidate: %s / %s / %s | skipped patches: %s\"") !=
          std::string::npos);
    CHECK(lodDebugSection.find("\"Transitions: %s active, age %.1f ms, max delta/frame %s, idle refine: %s\"") !=
          std::string::npos);
    CHECK(lodDebugSection.find("\"Transition diagnostics: delta %+lld reps | promoted %s | demoted %s | hysteresis-kept %s\"") !=
          std::string::npos);
    CHECK(lodDebugSection.find("\"Feature reps: spatial %s | colour %s | normal %s | scalar min/max/thresh %s/%s/%s | accent %s\"") !=
          std::string::npos);
    CHECK(lodDebugSection.find("\"Feature refinements: colour %s | scalar %s | normal %s | accent %s\"") !=
          std::string::npos);
    CHECK(lodDebugSection.find("\"Hysteresis band: promote %.2fx | demote %.2fx\"") != std::string::npos);
    CHECK(lodDebugSection.find("\"coarse fallback\"") != std::string::npos);
    CHECK(lodDebugSection.find("\"exact traversal\"") != std::string::npos);
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

TEST_CASE("Point-cloud colormaps sample listed and procedural tables", "[style][colormap]") {
    using invisible_places::renderer::pointcloud::PointCloudColormapId;
    using invisible_places::renderer::pointcloud::SampleColormap;
    using invisible_places::renderer::pointcloud::SampleGradient;

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
    checkColor(SampleColormap(PointCloudColormapId::Topographic, 0.0F), {0.03F, 0.12F, 0.28F});
    checkColor(SampleColormap(PointCloudColormapId::Topographic, 1.0F), {0.96F, 0.95F, 0.90F});
    checkColor(SampleColormap(PointCloudColormapId::LandSurface, 1.0F), {0.86F, 0.82F, 0.72F});
    checkColor(SampleColormap(PointCloudColormapId::ExponentialFire, 0.0F), {0.0F, 0.0F, 0.0F});
    checkColor(SampleColormap(PointCloudColormapId::ExponentialFire, 1.0F), {1.0F, 1.0F, 0.92F});
    checkColor(SampleColormap(PointCloudColormapId::ExponentialIce, 1.0F), {0.96F, 1.0F, 1.0F});
    checkColor(SampleColormap(PointCloudColormapId::HighContrast, 0.5F), {0.0F, 0.82F, 0.95F});
    checkColor(SampleGradient({1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.0F), {1.0F, 0.0F, 0.0F});
    checkColor(SampleGradient({1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 1.0F), {0.0F, 0.0F, 1.0F});
    checkColor(SampleGradient({1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.25F), {0.75F, 0.0F, 0.25F});
}

TEST_CASE("Project document round-trips binding-backed point-cloud styles", "[serialization][project]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_roundtrip.json";

    invisible_places::serialization::ProjectDocument document;
    document.projectName = "Roundtrip";
    document.selectedLayerPath = "Data/Site2 -5mm.ply";
    document.lastAnimationPath = "Saved/animations/Roundtrip.ipanim.json";
    document.backgroundColor = {0.02F, 0.04F, 0.08F, 1.0F};
    document.eyeDomeLightingEnabled = true;
    document.eyeDomeLightingThickness = 4.0F;
    document.constantUpdateView = true;
    document.liveVisualEffects = true;
    document.sidePanelPinned = true;
    document.autoLowerGsplatQualityWhileNavigating = false;
    document.pointCloudRendererMode =
        invisible_places::renderer::pointcloud::PointCloudRendererMode::FastBasicSource;
    document.renderJobSettings.outputDirectory = "Saved/renders/Roundtrip";
    document.renderJobSettings.width = 3840;
    document.renderJobSettings.height = 2160;
    document.renderJobSettings.framesPerSecond = 24;
    document.renderJobSettings.stillCameraDurationSeconds = 8.5F;
    document.renderJobSettings.tileSize = 256;
    document.renderJobSettings.startFrame = 10;
    document.renderJobSettings.endFrame = 42;
    document.renderJobSettings.fromShotIndex = 0;
    document.renderJobSettings.toShotIndex = 1;
    document.waterSourceSettings = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Detail);
    document.waterSourceSettings.path.pathLength = 4.25F;
    document.waterSourceSettings.trailShape.particleJitter = 0.64F;
    document.waterSourceSettings.trailShape.splineAnchorSpacing = 0.37F;
    document.waterSourceSettings.trailShape.trailLaneCount = 11U;
    document.waterSourceSettings.trailShape.trailLooseness = 0.73F;
    document.waterSourceSettings.trailShape.trailSmoothness = 0.68F;
    document.waterSourceSettings.trailShape.trailTurbulence = 0.82F;
    document.waterSourceSettings.trailShape.trailMomentum = 0.44F;
    document.waterSourceSettings.trailShape.normalTurbulenceResponse = 1.15F;
    document.waterAnimationTrailSettings = invisible_places::water::DefaultWaterAnimationTrailSettings();
    document.waterAnimationTrailSettings.particleDensity = 1.75F;
    document.waterAnimationTrailSettings.particleSpeed = 1.8F;
    document.waterAnimationTrailSettings.colorVariation = 0.48F;
    document.waterAnimationTrailSettings.trailLengthMeters = 1.35F;
    document.waterAnimationTrailSettings.trailSampleSpacingMeters = 0.045F;
    invisible_places::serialization::WaterAnimationTrailProfileDocument trailProfile;
    trailProfile.name = "Custom Bright Ribbons";
    trailProfile.settings = document.waterAnimationTrailSettings;
    trailProfile.settings.particleDensity = 2.6F;
    trailProfile.settings.trailLengthMeters = 2.4F;
    trailProfile.settings.trailSampleSpacingMeters = 0.022F;
    document.waterAnimationTrailProfiles.push_back(trailProfile);
    invisible_places::renderer::pointcloud::PointCloudStyleState waterVisualStyle;
    invisible_places::style::SetScalarConstant(&waterVisualStyle.pointSize, 22.0F);
    invisible_places::style::SetScalarConstant(&waterVisualStyle.opacity, 0.42F);
    invisible_places::style::SetScalarConstant(&waterVisualStyle.emissiveStrength, 0.58F);
    waterVisualStyle.colorMode =
        invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap;
    waterVisualStyle.colormap =
        invisible_places::renderer::pointcloud::PointCloudColormapId::CustomGradient;
    waterVisualStyle.gradientStartColor = {0.10F, 0.20F, 0.30F};
    waterVisualStyle.gradientEndColor = {0.85F, 0.75F, 0.25F};
    document.waterPointVisuals.push_back({.name = "River Threads", .style = waterVisualStyle});
    document.selectedWaterPointVisualName = "River Threads";
    document.tempWaterSourceSettings = document.waterSourceSettings;
    document.tempWaterAnimationTrailSettings = document.waterAnimationTrailSettings;
    document.tempWaterAnimationTrailSettings->particleSpeed = 2.1F;
    document.tempWaterAnimationTrailSettings->trailLengthMeters = 1.85F;
    document.tempWaterAnimationTrailSettings->trailSampleSpacingMeters = 0.030F;
    document.waterSettings.path = document.waterSourceSettings.path;
    document.waterSettings.trail.particleDensity = document.waterAnimationTrailSettings.particleDensity;
    document.waterSettings.trail.particleSpeed = document.waterAnimationTrailSettings.particleSpeed;
    document.waterSettings.trail.particleJitter = document.waterSourceSettings.trailShape.particleJitter;
    document.waterSettings.trail.splineAnchorSpacing =
        document.waterSourceSettings.trailShape.splineAnchorSpacing;
    document.waterSettings.visual.colorVariation = document.waterAnimationTrailSettings.colorVariation;
    document.waterBakeSettings = document.waterSourceSettings.path;
    document.waterRenderSettings = document.waterSettings;
    invisible_places::water::WaterEmitter waterEmitter;
    waterEmitter.id = 17;
    waterEmitter.name = "Pool seep";
    waterEmitter.position = {1.0F, 2.0F, 3.0F};
    waterEmitter.radius = 0.12F;
    waterEmitter.strength = 1.4F;
    waterEmitter.speed = 0.75F;
    waterEmitter.scope = invisible_places::water::WaterScaleMode::Detail;
    waterEmitter.origin = invisible_places::water::WaterEmitterOrigin::Manual;
    waterEmitter.status = invisible_places::water::WaterEmitterStatus::Accepted;
    waterEmitter.confidence = 0.92F;
    document.waterEmitters.push_back(waterEmitter);
    invisible_places::water::WaterPathCache projectPathCache;
    projectPathCache.supportLayerPath = "Data/Site2 -5mm.ply";
    projectPathCache.supportSignature = "Data/Site2 -5mm.ply|points=2048";
    projectPathCache.emitterSettingsFingerprint = "roundtrip-fingerprint";
    projectPathCache.requestedSettings = document.waterSourceSettings.path;
    projectPathCache.tunedSettings = document.waterSourceSettings.path;
    invisible_places::water::WaterPathBranch projectBranch;
    projectBranch.id = 31U;
    projectBranch.emitterId = waterEmitter.id;
    projectBranch.role = invisible_places::water::WaterPathBranchRole::Main;
    projectBranch.length = 2.5F;
    projectBranch.rawAnchors.push_back({
        .position = {1.0F, 2.0F, 3.0F},
        .normal = {0.0F, 0.0F, 1.0F},
        .emitterId = static_cast<float>(waterEmitter.id),
        .pathDistance = 0.0F,
    });
    projectBranch.rawAnchors.push_back({
        .position = {1.5F, 2.2F, 2.6F},
        .normal = {0.0F, 0.0F, 1.0F},
        .emitterId = static_cast<float>(waterEmitter.id),
        .pathDistance = 2.5F,
    });
    projectPathCache.branches.push_back(projectBranch);
    projectPathCache.hiddenBranchIds.push_back(99U);
    document.waterPathCache = projectPathCache;
    document.waterCausticLookSettings = invisible_places::water::DefaultWaterCausticLookSettings();
    document.waterCausticLookSettings.enabled = true;
    document.waterCausticLookSettings.intensity = 1.25F;
    document.waterCausticLookSettings.scale = 5.5F;
    document.waterCausticLookSettings.speed = 0.9F;
    document.waterCausticLookSettings.cellSizeMeters = 0.18F;
    document.waterCausticLookSettings.lineWidthMeters = 0.012F;
    document.waterCausticLookSettings.featherMeters = 0.004F;
    document.waterCausticLookSettings.surfacePointSpacingMeters = 0.003F;
    document.waterCausticLookSettings.warpAmplitudeMeters = 0.035F;
    document.waterCausticLookSettings.tintBlue = 1.35F;
    document.tempWaterCausticLookSettings = document.waterCausticLookSettings;
    document.tempWaterCausticLookSettings->warp = 0.8F;
    document.tempWaterCausticLookSettings->warpAmplitudeMeters = 0.055F;
    const std::vector<invisible_places::io::Float3> causticLaceVertices{
        {0.0F, 0.0F, 0.2F},
        {1.0F, 0.0F, 0.25F},
        {0.0F, 1.0F, 0.15F}};
    constexpr float causticLaceEdgeBlend = 0.70F;
    invisible_places::water::WaterEffectLayer rippleLayer;
    rippleLayer.id = 8;
    rippleLayer.name = "Rock pool caustic lace";
    rippleLayer.featureType = invisible_places::water::WaterEffectFeatureType::Ripple;
    rippleLayer.rippleOverlayType = invisible_places::water::WaterRippleOverlayType::CausticLace;
    rippleLayer.targetLayerSourcePath = "Data/Site2 -5mm.ply";
    rippleLayer.vertices = causticLaceVertices;
    rippleLayer.hull = invisible_places::water::BuildWaterRegionHull(rippleLayer.vertices);
    rippleLayer.edgeBlendWidth = causticLaceEdgeBlend;
    rippleLayer.wavelengthMeters = 0.18F;
    rippleLayer.response.emissionAdd = 1.25F;
    document.waterRippleLayers.push_back(rippleLayer);
    invisible_places::water::WaterEffectLayer fieldLayer;
    fieldLayer.id = 12;
    fieldLayer.name = "Rock edge field";
    fieldLayer.featureType = invisible_places::water::WaterEffectFeatureType::FieldSurfaceMotion;
    fieldLayer.targetLayerSourcePath = "Data/Site2 -5mm.ply";
    fieldLayer.vertices = causticLaceVertices;
    fieldLayer.hull = invisible_places::water::BuildWaterRegionHull(fieldLayer.vertices);
    fieldLayer.edgeBlendWidth = 0.24F;
    fieldLayer.regionStrength = 0.82F;
    fieldLayer.directionX = 0.0F;
    fieldLayer.directionY = 1.0F;
    fieldLayer.directionZ = 0.0F;
    document.waterFieldLayers.push_back(fieldLayer);
    invisible_places::water::WaterEffectLayer noFlowLayer;
    noFlowLayer.id = 13;
    noFlowLayer.name = "No flow backside";
    noFlowLayer.featureType = invisible_places::water::WaterEffectFeatureType::FieldNoFlowRegion;
    noFlowLayer.targetLayerSourcePath = "Data/Site2 -5mm.ply";
    noFlowLayer.vertices = causticLaceVertices;
    noFlowLayer.hull = invisible_places::water::BuildWaterRegionHull(noFlowLayer.vertices);
    document.waterFieldLayers.push_back(noFlowLayer);
    document.waterFlowStreamSettings.streamCountTotal = 321U;
    document.waterFlowStreamSettings.streamWidthMeters = 0.014F;
    document.waterFieldSettings.corridorRadiusMeters = 0.42F;
    document.waterFieldSettings.outputMode = invisible_places::water::WaterFieldOutputMode::Both;
    document.waterFieldStreamSettings.streamlineCount = 654U;
    document.waterFieldStreamSettings.streamlineWidthMeters = 0.009F;
    document.cameraPathShotIndices = {0, 0};
    document.cameraPathDurationFrames = 144;
    document.hasSavedAnimationRegistry = true;
    document.savedAnimations.push_back(
        {.filePath = "Saved/animations/Roundtrip.ipanim.json",
         .associatedLayerPaths = {"Data/Site2 -5mm.ply"}});
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
    shot.id = "camera_entry";
    shot.name = "Entry";
    shot.durationFrames = 120;
    shot.state.position = {1.0F, 2.0F, 3.0F};
    shot.state.target = {4.0F, 5.0F, 6.0F};
    shot.state.orbitCenter = {7.0F, 8.0F, 9.0F};
    shot.state.hasOrbitCenter = true;
    shot.state.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    shot.associatedLayerPaths = {"Data/Site2 -5mm.ply"};
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
    pointStyle.colormap = invisible_places::renderer::pointcloud::PointCloudColormapId::HighContrast;
    pointStyle.colorizeColor = {0.2F, 0.6F, 1.0F};
    pointStyle.colorizeAmount = 0.35F;
    pointStyle.stylisationMode =
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::BrushParticles;
    pointStyle.nprPreset = invisible_places::renderer::pointcloud::PointCloudNprPreset::Cartoon;
    pointStyle.stylisationStrength = 0.8F;
    pointStyle.stylisationColorLevels = 4.0F;
    pointStyle.stylisationInkStrength = 0.55F;
    pointStyle.stylisationPaperGrain = 0.45F;
    pointStyle.stylisationPigmentBleed = 0.6F;
    pointStyle.brushAspect = 3.0F;
    pointStyle.strokeJitter = 0.25F;
    pointStyle.hatchStrength = 0.2F;
    pointStyle.strokeOpacityVariance = 0.4F;
    pointStyle.pigmentVariation = 0.65F;
    pointStyle.pigmentAnimationSpeed = 1.25F;
    pointStyle.granulationAngleStrength = 0.75F;
    pointStyle.roughnessMotionStrength = 0.018F;
    pointStyle.roughnessMotionScale = 2.4F;
    pointStyle.roughnessMotionSpeed = 0.6F;
    pointStyle.roughnessMotionThreshold = 0.62F;
    pointStyle.roughnessMotionGroundId = 0.0F;
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
    pointStyle.waterStreakAspect = 7.5F;
    pointStyle.depthAlphaThreshold = 0.42F;
    pointStyle.solidCenters = false;
    pointStyle.flowAnimation = true;
    pointStyle.waterStreamOverlay = true;
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
    auto editedStyle = pointStyle;
    editedStyle.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    editedStyle.solidColor = {0.1F, 0.2F, 0.3F, 1.0F};
    editedStyle.waterStreakAspect = 9.0F;
    layer.pointVisuals.push_back({.name = "Warm", .style = pointStyle});
    layer.pointVisuals.push_back({.name = "Warm_edited", .style = editedStyle});
    layer.selectedPointVisualName = "Warm_edited";
    document.layers.push_back(layer);

    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, outputPath, &errorMessage));
    {
        std::ifstream savedProject{outputPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedProject},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find("\"render_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"blend_mode\"") != std::string::npos);
        CHECK(savedJson.find("\"depth_contribution\"") != std::string::npos);
        CHECK(savedJson.find("\"active\"") != std::string::npos);
        CHECK(savedJson.find("\"solid_centers\"") != std::string::npos);
    CHECK(savedJson.find("\"stylisation_mode\"") != std::string::npos);
    CHECK(savedJson.find("\"pigment_animation_speed\"") != std::string::npos);
    CHECK(savedJson.find("\"brush_particles\"") != std::string::npos);
        CHECK(savedJson.find("\"npr_preset\"") != std::string::npos);
        CHECK(savedJson.find("\"water_emitters\"") != std::string::npos);
        CHECK(savedJson.find("\"water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_length_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_sample_spacing_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"water_animation_trail_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"Custom Bright Ribbons\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_streak_aspect\"") != std::string::npos);
        CHECK(savedJson.find("\"water_point_visuals\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_point_visual\"") != std::string::npos);
        CHECK(savedJson.find("\"custom_gradient\"") != std::string::npos);
        CHECK(savedJson.find("\"gradient_start_color\"") != std::string::npos);
        CHECK(savedJson.find("\"gradient_end_color\"") != std::string::npos);
        CHECK(savedJson.find("\"water_point_visual_style\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_point_visual_style\"") == std::string::npos);
        CHECK(savedJson.find("\"water_basin_regions\"") == std::string::npos);
        CHECK(savedJson.find("\"water_runoff_regions\"") == std::string::npos);
        CHECK(savedJson.find("\"water_caustic_regions\"") == std::string::npos);
        CHECK(savedJson.find("\"water_ripple_layers\"") != std::string::npos);
        CHECK(savedJson.find("\"water_field_layers\"") != std::string::npos);
        CHECK(savedJson.find("\"field_surface_motion\"") != std::string::npos);
        CHECK(savedJson.find("\"caustic_lace\"") != std::string::npos);
        CHECK(savedJson.find("\"water_flow_stream_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_field_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_field_stream_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_caustic_look_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_caustic_look_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"preview_tint_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"water_path_cache\"") != std::string::npos);
        CHECK(savedJson.find("\"water_stream_overlay\"") != std::string::npos);
        CHECK(savedJson.find("\"water_visual_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_visual_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"settings_assignment\"") != std::string::npos);
        CHECK(savedJson.find("\"water_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"path_generation\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_shape\"") != std::string::npos);
        CHECK(savedJson.find("\"visuals\"") == std::string::npos);
        CHECK(savedJson.find("\"water_bake_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"water_render_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"scale_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"scope\"") == std::string::npos);
        CHECK(savedJson.find("\"still_camera_duration_seconds\"") != std::string::npos);
        CHECK(savedJson.find("\"particle_speed\"") != std::string::npos);
        CHECK(savedJson.find("\"spline_anchor_spacing\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_lane_count\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_looseness\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_smoothness\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_turbulence\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_momentum\"") != std::string::npos);
        CHECK(savedJson.find("\"normal_turbulence_response\"") != std::string::npos);
        CHECK(savedJson.find("\"eye_dome_lighting_enabled\"") != std::string::npos);
        CHECK(savedJson.find("\"eye_dome_lighting_thickness\": 4.0") != std::string::npos);
        CHECK(savedJson.find("\"constant_update_view\"") != std::string::npos);
        CHECK(savedJson.find("\"live_visual_effects\"") != std::string::npos);
        CHECK(savedJson.find("\"point_cloud_renderer_mode\"") != std::string::npos);
        CHECK(savedJson.find("\"point_visuals\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_point_visual\"") != std::string::npos);
        CHECK(savedJson.find("\"associated_layer_paths\"") != std::string::npos);
        CHECK(savedJson.find("\"saved_animations\"") != std::string::npos);
        CHECK(savedJson.find("\"schema_version\": 24") != std::string::npos);
        CHECK(savedJson.find("\"id\": \"camera_entry\"") != std::string::npos);
        CHECK(savedJson.find("\"duration_frames\": 120") == std::string::npos);
    }

    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    REQUIRE(loadedDocument->layers.size() == 1);

    CHECK(loadedDocument->projectName == "Roundtrip");
    CHECK(loadedDocument->sidePanelPinned);
    CHECK(!loadedDocument->autoLowerGsplatQualityWhileNavigating);
    CHECK(
        loadedDocument->pointCloudRendererMode ==
        invisible_places::renderer::pointcloud::PointCloudRendererMode::FastBasicSource);
    CHECK(loadedDocument->renderJobSettings.outputDirectory == "Saved/renders/Roundtrip");
    CHECK(loadedDocument->renderJobSettings.width == 3840);
    CHECK(loadedDocument->renderJobSettings.height == 2160);
    CHECK(loadedDocument->renderJobSettings.framesPerSecond == 24);
    CHECK(loadedDocument->renderJobSettings.stillCameraDurationSeconds == Catch::Approx(8.5F));
    CHECK(loadedDocument->renderJobSettings.tileSize == 256);
    CHECK(loadedDocument->renderJobSettings.startFrame == 10);
    CHECK(loadedDocument->renderJobSettings.endFrame == 42);
    CHECK(loadedDocument->renderJobSettings.toShotIndex == 1);
    CHECK(loadedDocument->waterSourceSettings.path.pathLength == Catch::Approx(4.25F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.particleJitter == Catch::Approx(0.64F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.splineAnchorSpacing == Catch::Approx(0.37F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.trailLaneCount == 11U);
    CHECK(loadedDocument->waterSourceSettings.trailShape.trailLooseness == Catch::Approx(0.73F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.trailSmoothness == Catch::Approx(0.68F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.trailTurbulence == Catch::Approx(0.82F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.trailMomentum == Catch::Approx(0.44F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.normalTurbulenceResponse == Catch::Approx(1.15F));
    CHECK(loadedDocument->waterAnimationTrailSettings.particleDensity == Catch::Approx(1.75F));
    CHECK(loadedDocument->waterAnimationTrailSettings.particleSpeed == Catch::Approx(1.8F));
    CHECK(loadedDocument->waterAnimationTrailSettings.colorVariation == Catch::Approx(0.48F));
    CHECK(loadedDocument->waterAnimationTrailSettings.trailLengthMeters == Catch::Approx(1.35F));
    CHECK(loadedDocument->waterAnimationTrailSettings.trailSampleSpacingMeters == Catch::Approx(0.045F));
    REQUIRE(loadedDocument->waterAnimationTrailProfiles.size() == 1U);
    CHECK(loadedDocument->waterAnimationTrailProfiles[0].name == "Custom Bright Ribbons");
    CHECK(loadedDocument->waterAnimationTrailProfiles[0].settings.particleDensity == Catch::Approx(2.6F));
    CHECK(loadedDocument->waterAnimationTrailProfiles[0].settings.trailLengthMeters == Catch::Approx(2.4F));
    CHECK(loadedDocument->waterAnimationTrailProfiles[0].settings.trailSampleSpacingMeters == Catch::Approx(0.022F));
    CHECK(loadedDocument->selectedWaterPointVisualName == "River Threads");
    REQUIRE(loadedDocument->waterPointVisuals.size() == 1U);
    CHECK(loadedDocument->waterPointVisuals[0].name == "River Threads");
    CHECK(
        loadedDocument->waterPointVisuals[0].style.colormap ==
        invisible_places::renderer::pointcloud::PointCloudColormapId::CustomGradient);
    CHECK(invisible_places::style::ScalarConstant(loadedDocument->waterPointVisuals[0].style.pointSize) ==
          Catch::Approx(22.0F));
    CHECK(invisible_places::style::ScalarConstant(loadedDocument->waterPointVisuals[0].style.opacity) ==
          Catch::Approx(0.42F));
    CHECK(invisible_places::style::ScalarConstant(loadedDocument->waterPointVisuals[0].style.emissiveStrength) ==
          Catch::Approx(0.58F));
    CHECK(loadedDocument->waterPointVisuals[0].style.gradientStartColor[0] == Catch::Approx(0.10F));
    CHECK(loadedDocument->waterPointVisuals[0].style.gradientEndColor[2] == Catch::Approx(0.25F));
    REQUIRE(loadedDocument->tempWaterSourceSettings.has_value());
    CHECK(loadedDocument->tempWaterSourceSettings->trailShape.particleJitter == Catch::Approx(0.64F));
    CHECK(loadedDocument->tempWaterSourceSettings->trailShape.trailLaneCount == 11U);
    REQUIRE(loadedDocument->tempWaterAnimationTrailSettings.has_value());
    CHECK(loadedDocument->tempWaterAnimationTrailSettings->particleSpeed == Catch::Approx(2.1F));
    CHECK(loadedDocument->tempWaterAnimationTrailSettings->trailLengthMeters == Catch::Approx(1.85F));
    CHECK(loadedDocument->tempWaterAnimationTrailSettings->trailSampleSpacingMeters == Catch::Approx(0.030F));
    CHECK(loadedDocument->waterSettings.path.pathLength == Catch::Approx(4.25F));
    CHECK(loadedDocument->waterSettings.trail.particleDensity == Catch::Approx(1.75F));
    CHECK(loadedDocument->waterSettings.visual.colorVariation == Catch::Approx(0.48F));
    CHECK(loadedDocument->waterBakeSettings.pathLength == Catch::Approx(4.25F));
    CHECK(loadedDocument->waterRenderSettings.trail.particleSpeed == Catch::Approx(1.8F));
    REQUIRE(loadedDocument->waterEmitters.size() == 1);
    CHECK(loadedDocument->waterEmitters[0].id == 17U);
    CHECK(loadedDocument->waterEmitters[0].name == "Pool seep");
    CHECK(loadedDocument->waterEmitters[0].position.z == Catch::Approx(3.0F));
    CHECK(loadedDocument->waterEmitters[0].radius == Catch::Approx(0.12F));
    CHECK(loadedDocument->waterEmitters[0].strength == Catch::Approx(1.4F));
    CHECK(loadedDocument->waterEmitters[0].speed == Catch::Approx(0.75F));
    CHECK(loadedDocument->waterEmitters[0].scope == invisible_places::water::WaterScaleMode::Mid);
    CHECK(loadedDocument->waterEmitters[0].origin == invisible_places::water::WaterEmitterOrigin::Manual);
    CHECK(loadedDocument->waterEmitters[0].status == invisible_places::water::WaterEmitterStatus::Accepted);
    CHECK(loadedDocument->waterEmitters[0].confidence == Catch::Approx(0.92F));
    CHECK(
        loadedDocument->waterEmitters[0].sourceSettingsAssignment ==
        invisible_places::water::WaterSourceSettingsAssignment::Default);
    REQUIRE(loadedDocument->waterPathCache.has_value());
    CHECK(loadedDocument->waterPathCache->supportLayerPath == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loadedDocument->waterPathCache->supportSignature == "Data/Site2 -5mm.ply|points=2048");
    CHECK(loadedDocument->waterPathCache->emitterSettingsFingerprint == "roundtrip-fingerprint");
    REQUIRE(loadedDocument->waterPathCache->branches.size() == 1U);
    CHECK(loadedDocument->waterPathCache->branches[0].id == 31U);
    REQUIRE(loadedDocument->waterPathCache->branches[0].rawAnchors.size() == 2U);
    CHECK(loadedDocument->waterPathCache->branches[0].rawAnchors[1].pathDistance == Catch::Approx(2.5F));
    CHECK(loadedDocument->waterPathCache->hiddenBranchIds == std::vector<std::uint32_t>{99U});
    REQUIRE(loadedDocument->waterRippleLayers.size() == 1U);
    CHECK(loadedDocument->waterRippleLayers[0].name == "Rock pool caustic lace");
    CHECK(
        loadedDocument->waterRippleLayers[0].rippleOverlayType ==
        invisible_places::water::WaterRippleOverlayType::CausticLace);
    CHECK(loadedDocument->waterRippleLayers[0].edgeBlendWidth == Catch::Approx(0.70F));
    CHECK(loadedDocument->waterRippleLayers[0].wavelengthMeters == Catch::Approx(0.18F));
    REQUIRE(loadedDocument->waterFieldLayers.size() == 2U);
    CHECK(loadedDocument->waterFieldLayers[0].name == "Rock edge field");
    CHECK(
        loadedDocument->waterFieldLayers[0].featureType ==
        invisible_places::water::WaterEffectFeatureType::FieldSurfaceMotion);
    CHECK(loadedDocument->waterFieldLayers[0].edgeBlendWidth == Catch::Approx(0.24F));
    CHECK(loadedDocument->waterFieldLayers[0].regionStrength == Catch::Approx(0.82F));
    CHECK(loadedDocument->waterFieldLayers[0].directionY == Catch::Approx(1.0F));
    CHECK(loadedDocument->waterFieldLayers[1].name == "No flow backside");
    CHECK(
        loadedDocument->waterFieldLayers[1].featureType ==
        invisible_places::water::WaterEffectFeatureType::FieldNoFlowRegion);
    CHECK(loadedDocument->waterFlowStreamSettings.streamCountTotal == 321U);
    CHECK(loadedDocument->waterFlowStreamSettings.streamWidthMeters == Catch::Approx(0.014F));
    CHECK(loadedDocument->waterFieldSettings.corridorRadiusMeters == Catch::Approx(0.42F));
    CHECK(loadedDocument->waterFieldStreamSettings.streamlineCount == 654U);
    CHECK(loadedDocument->waterFieldStreamSettings.streamlineWidthMeters == Catch::Approx(0.009F));
    CHECK(loadedDocument->waterCausticLookSettings.enabled);
    CHECK(loadedDocument->waterCausticLookSettings.intensity == Catch::Approx(1.25F));
    CHECK(loadedDocument->waterCausticLookSettings.scale == Catch::Approx(5.5F));
    CHECK(loadedDocument->waterCausticLookSettings.cellSizeMeters == Catch::Approx(0.18F));
    CHECK(loadedDocument->waterCausticLookSettings.lineWidthMeters == Catch::Approx(0.012F));
    CHECK(loadedDocument->waterCausticLookSettings.featherMeters == Catch::Approx(0.004F));
    CHECK(loadedDocument->waterCausticLookSettings.surfacePointSpacingMeters == Catch::Approx(0.003F));
    CHECK(loadedDocument->waterCausticLookSettings.warpAmplitudeMeters == Catch::Approx(0.035F));
    CHECK(loadedDocument->waterCausticLookSettings.tintBlue == Catch::Approx(1.35F));
    REQUIRE(loadedDocument->tempWaterCausticLookSettings.has_value());
    CHECK(loadedDocument->tempWaterCausticLookSettings->warp == Catch::Approx(0.8F));
    CHECK(loadedDocument->tempWaterCausticLookSettings->warpAmplitudeMeters == Catch::Approx(0.055F));
    CHECK(loadedDocument->cameraPathShotIndices == std::vector<std::size_t>{0, 0});
    CHECK(loadedDocument->cameraPathDurationFrames == 144);
    REQUIRE(loadedDocument->hasSavedAnimationRegistry);
    REQUIRE(loadedDocument->savedAnimations.size() == 1);
    CHECK(loadedDocument->savedAnimations[0].filePath == std::filesystem::path{"Saved/animations/Roundtrip.ipanim.json"});
    REQUIRE(loadedDocument->savedAnimations[0].associatedLayerPaths.size() == 1);
    CHECK(loadedDocument->savedAnimations[0].associatedLayerPaths[0] == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loadedDocument->backgroundColor[2] == Catch::Approx(0.08F));
    CHECK(loadedDocument->eyeDomeLightingEnabled);
    CHECK(loadedDocument->eyeDomeLightingThickness == Catch::Approx(4.0F));
    CHECK(loadedDocument->constantUpdateView);
    CHECK(loadedDocument->liveVisualEffects);
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
    CHECK(loadedDocument->cameraShots[0].id == "camera_entry");
    CHECK(loadedDocument->cameraShots[0].name == "Entry");
    CHECK(loadedDocument->cameraShots[0].state.position[2] == Catch::Approx(3.0F));
    REQUIRE(loadedDocument->cameraShots[0].associatedLayerPaths.size() == 1);
    CHECK(loadedDocument->cameraShots[0].associatedLayerPaths[0] == std::filesystem::path{"Data/Site2 -5mm.ply"});
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
        invisible_places::renderer::pointcloud::PointCloudColormapId::HighContrast);
    CHECK(loadedLayer.pointStyle->colorizeColor[0] == Catch::Approx(0.2F));
    CHECK(loadedLayer.pointStyle->colorizeColor[1] == Catch::Approx(0.6F));
    CHECK(loadedLayer.pointStyle->colorizeColor[2] == Catch::Approx(1.0F));
    CHECK(loadedLayer.pointStyle->colorizeAmount == Catch::Approx(0.35F));
    CHECK(
        loadedLayer.pointStyle->stylisationMode ==
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::BrushParticles);
    CHECK(
        loadedLayer.pointStyle->nprPreset ==
        invisible_places::renderer::pointcloud::PointCloudNprPreset::Cartoon);
    CHECK(loadedLayer.pointStyle->stylisationStrength == Catch::Approx(0.8F));
    CHECK(loadedLayer.pointStyle->stylisationColorLevels == Catch::Approx(4.0F));
    CHECK(loadedLayer.pointStyle->stylisationInkStrength == Catch::Approx(0.55F));
    CHECK(loadedLayer.pointStyle->stylisationPaperGrain == Catch::Approx(0.45F));
    CHECK(loadedLayer.pointStyle->stylisationPigmentBleed == Catch::Approx(0.6F));
    CHECK(loadedLayer.pointStyle->brushAspect == Catch::Approx(3.0F));
    CHECK(loadedLayer.pointStyle->strokeJitter == Catch::Approx(0.25F));
    CHECK(loadedLayer.pointStyle->hatchStrength == Catch::Approx(0.2F));
    CHECK(loadedLayer.pointStyle->strokeOpacityVariance == Catch::Approx(0.4F));
    CHECK(loadedLayer.pointStyle->pigmentVariation == Catch::Approx(0.65F));
    CHECK(loadedLayer.pointStyle->pigmentAnimationSpeed == Catch::Approx(1.25F));
    CHECK(loadedLayer.pointStyle->granulationAngleStrength == Catch::Approx(0.75F));
    CHECK(loadedLayer.pointStyle->roughnessMotionStrength == Catch::Approx(0.018F));
    CHECK(loadedLayer.pointStyle->roughnessMotionScale == Catch::Approx(2.4F));
    CHECK(loadedLayer.pointStyle->roughnessMotionSpeed == Catch::Approx(0.6F));
    CHECK(loadedLayer.pointStyle->roughnessMotionThreshold == Catch::Approx(0.62F));
    CHECK(loadedLayer.pointStyle->roughnessMotionGroundId == Catch::Approx(0.0F));
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
    CHECK(loadedLayer.pointStyle->waterStreakAspect == Catch::Approx(7.5F));
    CHECK(loadedLayer.pointStyle->depthAlphaThreshold == Catch::Approx(0.42F));
    CHECK(!loadedLayer.pointStyle->solidCenters);
    CHECK(loadedLayer.pointStyle->flowAnimation);
    CHECK(loadedLayer.pointStyle->waterStreamOverlay);
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.fieldSlot == 2);
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.fieldName == "Height");
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.inputMin == Catch::Approx(-2.0F));
    CHECK(loadedLayer.pointStyle->pointSize.fieldMap.outputMax == Catch::Approx(8.0F));
    CHECK(invisible_places::style::ScalarConstant(loadedLayer.pointStyle->surfelDiameter) == Catch::Approx(0.0125F));
    CHECK(invisible_places::style::ScalarConstant(loadedLayer.pointStyle->opacity) == Catch::Approx(0.55F));
    CHECK(!loadedLayer.pointStyle->opacity.active);
    CHECK(loadedLayer.pointStyle->colormapPosition.fieldMap.fieldName == "Intensity");
    CHECK(!loadedLayer.pointStyle->colormapPosition.active);
    REQUIRE(loadedLayer.pointVisuals.size() == 2);
    CHECK(loadedLayer.selectedPointVisualName == "Warm_edited");
    CHECK(loadedLayer.pointVisuals[0].name == "Warm");
    CHECK(loadedLayer.pointVisuals[1].name == "Warm_edited");
    CHECK(
        loadedLayer.pointVisuals[1].style.colorMode ==
        invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor);
    CHECK(loadedLayer.pointVisuals[1].style.solidColor[2] == Catch::Approx(0.3F));
    CHECK(loadedLayer.pointVisuals[1].style.waterStreakAspect == Catch::Approx(9.0F));

    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy water settings migrate into split water profiles", "[serialization][project][water]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_legacy_water_settings.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << R"({
  "schema_version": 16,
  "project_name": "Legacy Water",
  "water_bake_settings": {
    "scale_mode": "aerial",
    "support_voxel_size": 1.7,
    "max_bridge_distance": 6.5,
    "smoothing": 0.7,
    "path_length": 120.0,
    "path_density": 0.44,
    "max_steps": 321,
    "support_sample_limit": 12345
  },
  "water_render_settings": {
    "particle_size_pixels": 19.0,
    "particle_opacity": 0.37,
    "particle_density": 2.2,
    "particle_jitter": 0.9,
    "particle_speed": 2.7,
    "spline_anchor_spacing": 1.1,
    "color_variation": 0.83,
    "glow": 0.71
  },
  "water_emitters": [
    {
      "id": 4,
      "name": "legacy source",
      "position": [1.0, 2.0, 3.0],
      "scope": "detail",
      "status": "accepted"
    }
  ],
  "layers": []
})";
    }

    std::string errorMessage;
    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    CHECK(loadedDocument->waterSettings.path.legacyScaleMode == invisible_places::water::WaterScaleMode::Aerial);
    CHECK(loadedDocument->waterSettings.path.supportVoxelSize == Catch::Approx(1.7F));
    CHECK(loadedDocument->waterSettings.path.maxBridgeDistance == Catch::Approx(6.5F));
    CHECK(loadedDocument->waterSettings.path.pathSampleSpacing == Catch::Approx(0.44F));
    CHECK(loadedDocument->waterSettings.path.maxSteps == 321U);
    CHECK(loadedDocument->waterSettings.path.supportSampleLimit == 12345U);
    CHECK(loadedDocument->waterSourceSettings.path.maxBridgeDistance == Catch::Approx(6.5F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.particleJitter == Catch::Approx(0.9F));
    CHECK(loadedDocument->waterSourceSettings.trailShape.splineAnchorSpacing == Catch::Approx(1.1F));
    CHECK(loadedDocument->waterAnimationTrailSettings.particleDensity == Catch::Approx(2.2F));
    CHECK(loadedDocument->waterAnimationTrailSettings.particleSpeed == Catch::Approx(2.7F));
    CHECK(loadedDocument->waterAnimationTrailSettings.colorVariation == Catch::Approx(0.83F));
    CHECK(loadedDocument->waterVisualSettings.particleSizePixels == Catch::Approx(19.0F));
    CHECK(invisible_places::style::ScalarConstant(loadedDocument->waterPointVisualStyle.pointSize) ==
          Catch::Approx(19.0F));
    CHECK(loadedDocument->waterSettings.visual.particleSizePixels == Catch::Approx(19.0F));
    CHECK(loadedDocument->waterSettings.visual.particleOpacity == Catch::Approx(0.37F));
    CHECK(loadedDocument->waterSettings.trail.particleDensity == Catch::Approx(2.2F));
    CHECK(loadedDocument->waterSettings.trail.particleJitter == Catch::Approx(0.9F));
    CHECK(loadedDocument->waterSettings.trail.particleSpeed == Catch::Approx(2.7F));
    CHECK(loadedDocument->waterSettings.trail.splineAnchorSpacing == Catch::Approx(1.1F));
    CHECK(loadedDocument->waterSettings.visual.colorVariation == Catch::Approx(0.83F));
    CHECK(loadedDocument->waterSettings.visual.glow == Catch::Approx(0.71F));
    REQUIRE(loadedDocument->waterEmitters.size() == 1U);
    CHECK(loadedDocument->waterEmitters.front().scope == invisible_places::water::WaterScaleMode::Detail);
    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy water region records load as v2 ripples while basin and runoff are ignored", "[serialization][water][v2]") {
    const auto inputPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_water_regions.json";
    {
        std::ofstream legacyProject{inputPath};
        legacyProject << R"json({
  "schema_version": 23,
  "project_name": "Legacy water",
  "water_basin_regions": [
    {"id": 1, "name": "legacy basin", "vertices": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]}
  ],
  "water_runoff_regions": [
    {"id": 2, "name": "legacy runoff", "vertices": [[2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 1.0, 0.0]]}
  ],
  "water_caustic_look_settings": {
    "enabled": true,
    "intensity": 1.35,
    "speed": 0.8,
    "cell_size_meters": 0.22,
    "line_width_meters": 0.011,
    "feather_meters": 0.004,
    "warp": 0.45,
    "emission_boost": 1.75,
    "opacity_boost": 0.18,
    "point_size_boost": 0.09,
    "tint": [0.55, 0.85, 1.2]
  },
  "water_caustic_regions": [
    {
      "id": 8,
      "name": "legacy caustics",
      "target_layer_source_path": "Data/Site2 -5mm.ply",
      "vertices": [[0.0, 0.0, 0.2], [1.0, 0.0, 0.25], [0.0, 1.0, 0.15]],
      "edge_blend_width": 0.66,
      "enabled": true,
      "preview_tint_mode": "always"
    }
  ]
})json";
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadProjectDocument(inputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterRippleLayers.size() == 1U);
    const auto& layer = loaded->waterRippleLayers.front();
    CHECK(layer.id == 8U);
    CHECK(layer.name == "legacy caustics");
    CHECK(layer.featureType == invisible_places::water::WaterEffectFeatureType::Ripple);
    CHECK(layer.rippleOverlayType == invisible_places::water::WaterRippleOverlayType::CausticLace);
    CHECK(layer.targetLayerSourcePath == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(layer.edgeBlendWidth == Catch::Approx(0.66F));
    CHECK(layer.wavelengthMeters == Catch::Approx(0.22F));
    CHECK(layer.response.emissionAdd == Catch::Approx(1.75F));
    CHECK(layer.response.opacityAdd == Catch::Approx(0.18F));
    CHECK(layer.response.pointSizeAdd == Catch::Approx(0.09F));
    CHECK(layer.response.colouriseBlue == Catch::Approx(1.2F));

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_migrated_water_regions.json";
    REQUIRE(invisible_places::serialization::SaveProjectDocument(loaded.value(), outputPath, &errorMessage));
    std::ifstream savedProject{outputPath};
    const std::string savedJson{
        std::istreambuf_iterator<char>{savedProject},
        std::istreambuf_iterator<char>{}};
    CHECK(savedJson.find("\"water_ripple_layers\"") != std::string::npos);
    CHECK(savedJson.find("\"water_basin_regions\"") == std::string::npos);
    CHECK(savedJson.find("\"water_runoff_regions\"") == std::string::npos);
    CHECK(savedJson.find("\"water_caustic_regions\"") == std::string::npos);
    std::filesystem::remove(inputPath);
    std::filesystem::remove(outputPath);
}

TEST_CASE("Project document defaults to Beauty Adaptive renderer mode for older projects", "[serialization][project]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_legacy_renderer_defaults.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << R"({
  "schema_version": 16,
  "project_name": "Legacy",
  "background_color": [0.0, 0.0, 0.0, 1.0],
  "layers": []
})";
    }

    std::string errorMessage;
    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    CHECK(
        loadedDocument->pointCloudRendererMode ==
        invisible_places::renderer::pointcloud::PointCloudRendererMode::BeautyAdaptive);
    CHECK(loadedDocument->eyeDomeLightingThickness == Catch::Approx(1.0F));
    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy Raytraced renderer mode loads as Beauty Adaptive and is not re-saved", "[serialization][project]") {
    const auto inputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_legacy_raytraced_mode.json";
    {
        std::ofstream output{inputPath, std::ios::trunc};
        output << R"({
  "schema_version": 24,
  "project_name": "Legacy raytraced",
  "point_cloud_renderer_mode": "raytraced",
  "layers": []
})";
    }

    std::string errorMessage;
    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(inputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    CHECK(
        loadedDocument->pointCloudRendererMode ==
        invisible_places::renderer::pointcloud::PointCloudRendererMode::BeautyAdaptive);

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_legacy_raytraced_mode_saved.json";
    REQUIRE(invisible_places::serialization::SaveProjectDocument(loadedDocument.value(), outputPath, &errorMessage));
    std::ifstream savedProject{outputPath};
    const std::string savedJson{
        std::istreambuf_iterator<char>{savedProject},
        std::istreambuf_iterator<char>{}};
    CHECK(savedJson.find("\"point_cloud_renderer_mode\": \"beauty_adaptive\"") != std::string::npos);
    CHECK(savedJson.find("\"point_cloud_renderer_mode\": \"raytraced\"") == std::string::npos);
    std::filesystem::remove(inputPath);
    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy project files load camera and animation associations as unregistered", "[serialization][project]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_project_associations.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << R"({
  "schema_version": 12,
  "project_name": "Legacy",
  "camera_shots": [
    {
      "name": "Legacy Shot",
      "duration_frames": 90,
      "camera": {
        "position": [0, 0, 1],
        "orientation": [0, 0, 0, 1],
        "target": [0, 0, 0],
        "fov_degrees": 60,
        "near_plane": 0.01,
        "far_plane": 1000
      }
    }
  ]
})";
    }

    std::string errorMessage;
    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    CHECK_FALSE(loadedDocument->hasSavedAnimationRegistry);
    REQUIRE(loadedDocument->cameraShots.size() == 1);
    CHECK(!loadedDocument->cameraShots[0].id.empty());
    CHECK(loadedDocument->cameraShots[0].durationFrames == 90);
    CHECK(loadedDocument->cameraShots[0].associatedLayerPaths.empty());

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
        preset->style.screenSpriteSizeMode ==
        invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode::Pixels);
    CHECK(
        preset->style.depthContribution ==
        invisible_places::renderer::pointcloud::PointCloudDepthContribution::AlphaThreshold);
    CHECK(preset->style.pointSize.active);
    CHECK(preset->style.opacity.active);
    CHECK(preset->style.emissiveStrength.active);
    CHECK(
        preset->style.stylisationMode ==
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::Off);
    CHECK(
        preset->style.nprPreset ==
        invisible_places::renderer::pointcloud::PointCloudNprPreset::Watercolor);
    CHECK(preset->style.stylisationStrength == Catch::Approx(1.0F));
    CHECK(preset->style.pigmentVariation == Catch::Approx(0.0F));
    CHECK(preset->style.pigmentAnimationSpeed == Catch::Approx(0.0F));
    CHECK(preset->style.granulationAngleStrength == Catch::Approx(0.0F));
    CHECK(invisible_places::style::ScalarConstant(preset->style.surfelDiameter) == Catch::Approx(0.005F));

    std::filesystem::remove(presetPath);
}

TEST_CASE("Point cloud style parses and round-trips world-sized screen sprites", "[serialization][point-style]") {
    const auto presetPath =
        std::filesystem::temp_directory_path() / "invisible_places_world_sized_screen_sprite.json";
    std::ofstream output{presetPath, std::ios::trunc};
    output << R"({
  "schema_version": 2,
  "preset_name": "World Sized Sprites",
  "point_style": {
    "geometry_mode": "screen_sprites",
    "screen_sprite_size_mode": "world_millimeters",
    "surfel_diameter": {"mode": "constant", "constant_value": [0.012, 0.0, 0.0, 0.0]}
  }
})";
    output.close();

    std::string errorMessage;
    const auto preset = invisible_places::serialization::LoadPointCloudStylePreset(presetPath, &errorMessage);
    REQUIRE(preset.has_value());
    CHECK(
        preset->style.screenSpriteSizeMode ==
        invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode::WorldMillimeters);
    CHECK(invisible_places::style::ScalarConstant(preset->style.surfelDiameter) == Catch::Approx(0.012F));

    const auto roundTripPath =
        std::filesystem::temp_directory_path() / "invisible_places_world_sized_screen_sprite_roundtrip.json";
    REQUIRE(invisible_places::serialization::SavePointCloudStylePreset(*preset, roundTripPath, &errorMessage));
    const auto roundTrip = invisible_places::serialization::LoadPointCloudStylePreset(roundTripPath, &errorMessage);
    REQUIRE(roundTrip.has_value());
    CHECK(
        roundTrip->style.screenSpriteSizeMode ==
        invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode::WorldMillimeters);
    CHECK(invisible_places::style::ScalarConstant(roundTrip->style.surfelDiameter) == Catch::Approx(0.012F));

    std::filesystem::remove(presetPath);
    std::filesystem::remove(roundTripPath);
}

TEST_CASE("Point cloud style presets round-trip stylisation controls", "[serialization][point-style]") {
    invisible_places::serialization::PointCloudStylePresetDocument document;
    document.presetName = "Ink Brushes";
    document.style.stylisationMode =
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::NprStylisation;
    document.style.nprPreset = invisible_places::renderer::pointcloud::PointCloudNprPreset::Cartoon;
    document.style.stylisationStrength = 0.7F;
    document.style.stylisationColorLevels = 3.0F;
    document.style.stylisationInkStrength = 0.9F;
    document.style.stylisationPaperGrain = 0.1F;
    document.style.stylisationPigmentBleed = 0.2F;
    document.style.brushAspect = 4.0F;
    document.style.strokeJitter = 0.3F;
    document.style.hatchStrength = 0.6F;
    document.style.strokeOpacityVariance = 0.5F;
    document.style.pigmentVariation = 0.45F;
    document.style.pigmentAnimationSpeed = 1.5F;
    document.style.granulationAngleStrength = 0.7F;
    document.style.roughnessMotionStrength = 0.025F;
    document.style.roughnessMotionScale = 3.25F;
    document.style.roughnessMotionSpeed = 0.8F;
    document.style.roughnessMotionThreshold = 0.66F;
    document.style.roughnessMotionGroundId = 1.0F;

    const auto presetPath =
        std::filesystem::temp_directory_path() / "invisible_places_stylisation_style.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SavePointCloudStylePreset(document, presetPath, &errorMessage));
    const auto loaded = invisible_places::serialization::LoadPointCloudStylePreset(presetPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->presetName == "Ink Brushes");
    CHECK(
        loaded->style.stylisationMode ==
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::NprStylisation);
    CHECK(
        loaded->style.nprPreset ==
        invisible_places::renderer::pointcloud::PointCloudNprPreset::Cartoon);
    CHECK(loaded->style.stylisationStrength == Catch::Approx(0.7F));
    CHECK(loaded->style.stylisationColorLevels == Catch::Approx(3.0F));
    CHECK(loaded->style.stylisationInkStrength == Catch::Approx(0.9F));
    CHECK(loaded->style.stylisationPaperGrain == Catch::Approx(0.1F));
    CHECK(loaded->style.stylisationPigmentBleed == Catch::Approx(0.2F));
    CHECK(loaded->style.brushAspect == Catch::Approx(4.0F));
    CHECK(loaded->style.strokeJitter == Catch::Approx(0.3F));
    CHECK(loaded->style.hatchStrength == Catch::Approx(0.6F));
    CHECK(loaded->style.strokeOpacityVariance == Catch::Approx(0.5F));
    CHECK(loaded->style.pigmentVariation == Catch::Approx(0.45F));
    CHECK(loaded->style.pigmentAnimationSpeed == Catch::Approx(1.5F));
    CHECK(loaded->style.granulationAngleStrength == Catch::Approx(0.7F));
    CHECK(loaded->style.roughnessMotionStrength == Catch::Approx(0.025F));
    CHECK(loaded->style.roughnessMotionScale == Catch::Approx(3.25F));
    CHECK(loaded->style.roughnessMotionSpeed == Catch::Approx(0.8F));
    CHECK(loaded->style.roughnessMotionThreshold == Catch::Approx(0.66F));
    CHECK(loaded->style.roughnessMotionGroundId == Catch::Approx(1.0F));

    std::filesystem::remove(presetPath);
}

TEST_CASE("Point cloud style parses camera-facing world sprite geometry", "[serialization]") {
    const auto presetPath = std::filesystem::temp_directory_path() / "invisible_places_camera_facing_world_sprite.json";
    std::ofstream output{presetPath, std::ios::trunc};
    output << R"({
  "schema_version": 2,
  "preset_name": "Camera Facing",
  "point_style": {
    "geometry_mode": "camera_facing_world_sprites",
    "solid_centers": true
  }
})";
    output.close();

    std::string errorMessage;
    const auto preset = invisible_places::serialization::LoadPointCloudStylePreset(presetPath, &errorMessage);
    REQUIRE(preset.has_value());
    CHECK(
        preset->style.geometryMode ==
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites);
    CHECK(preset->style.solidCenters);

    std::filesystem::remove(presetPath);
}

TEST_CASE("Point visual selection owns aliased names before normalizing storage", "[water][pointcloud]") {
    namespace point_visual = invisible_places::app::point_visual;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;

    CHECK(point_visual::NormalizeName(" Warm_Edited ") == "Warm_edited");
    CHECK(point_visual::PresetName("Water Flow") == "Water Flow_preset");
    CHECK(point_visual::BaseName("White Needle Glow_preset") == "White Needle Glow");
    CHECK(point_visual::EditedName("White Needle Glow_preset") == "White Needle Glow_edited");
    CHECK(point_visual::IsPresetName("Water Flow_preset"));
    CHECK(point_visual::IsEditedName("Water Flow_Edited"));

    PointCloudStyleState waterStyle;
    waterStyle.exposure = 2.0F;
    waterStyle.waterStreakAspect = 4.0F;
    PointCloudStyleState customStyle;
    customStyle.exposure = 3.0F;
    customStyle.waterStreakAspect = 12.0F;

    std::vector<point_visual::VisualState> visuals{
        {.name = " Water Flow ", .style = waterStyle},
        {.name = " Custom Ribbons ", .style = customStyle},
    };
    PointCloudStyleState activeStyle;
    activeStyle.exposure = 1.0F;
    std::string selectedName = " Custom Ribbons ";
    std::string nameBuffer;

    const std::string_view requestedFromSelectedName{selectedName};
    REQUIRE(point_visual::Select(
        &visuals,
        &selectedName,
        &nameBuffer,
        &activeStyle,
        requestedFromSelectedName,
        activeStyle));
    CHECK(selectedName == "Custom Ribbons");
    CHECK(nameBuffer == "Custom Ribbons");
    CHECK(activeStyle.exposure == Catch::Approx(3.0F));
    CHECK(activeStyle.waterStreakAspect == Catch::Approx(12.0F));
    CHECK(visuals[0].name == "Water Flow");
    CHECK(visuals[1].name == "Custom Ribbons");

    activeStyle = waterStyle;
    selectedName = "Water Flow";
    nameBuffer.clear();
    visuals[1].name = " Custom Ribbons ";
    const std::string_view requestedFromVisualName{visuals[1].name};
    REQUIRE(point_visual::Select(
        &visuals,
        &selectedName,
        &nameBuffer,
        &activeStyle,
        requestedFromVisualName,
        activeStyle));
    CHECK(selectedName == "Custom Ribbons");
    CHECK(nameBuffer == "Custom Ribbons");
    CHECK(activeStyle.exposure == Catch::Approx(3.0F));
    CHECK(activeStyle.waterStreakAspect == Catch::Approx(12.0F));

    visuals.push_back({.name = "Legacy_Edited", .style = customStyle});
    point_visual::Remove(&visuals, "Legacy_edited");
    CHECK_FALSE(point_visual::FindIndex(visuals, "Legacy_Edited").has_value());
}

TEST_CASE("Water flow overlay bakes loadable scalar-field PLY traces", "[water][pointcloud]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-support.ply";
    cloud.layerName = "synthetic-water-support";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    for (int index = 0; index < 24; ++index) {
        const invisible_places::io::Float3 position{
            0.0F,
            0.0F,
            1.0F - static_cast<float>(index) * 0.04F};
        cloud.positions.push_back(position);
        cloud.normals.push_back({1.0F, 0.0F, 0.0F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    }
    cloud.focusPoint = cloud.positions.front();
    cloud.hasFocusPoint = true;

    auto settings = invisible_places::water::DefaultWaterSettingsBundle(
        invisible_places::water::WaterScaleMode::Detail);
    settings.path.supportVoxelSize = 0.02F;
    settings.path.maxBridgeDistance = 0.09F;
    settings.path.pathLength = 0.7F;
    settings.path.pathSampleSpacing = 0.018F;
    settings.path.maxSteps = 48;
    settings.path.supportSampleLimit = 256;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 3;
    emitter.name = "test spring";
    emitter.position = cloud.positions.front();
    emitter.radius = 0.04F;
    emitter.speed = 1.2F;
    emitter.confidence = 0.95F;

    settings.trail.particleDensity = 1.35F;
    settings.trail.particleJitter = 0.5F;
    settings.trail.particleSpeed = 2.0F;
    settings.trail.splineAnchorSpacing = 0.04F;
    settings.visual.colorVariation = 0.8F;

    const auto pathAnchors = invisible_places::water::GenerateWaterPathAnchors(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings.path);
    REQUIRE(pathAnchors.points.size() > 8);
    for (const auto& point : pathAnchors.points) {
        CHECK(point.particleRole < 0.5F);
    }

    const auto overlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        settings.trail,
        settings.visual);
    REQUIRE(overlay.points.size() > 8);
    CHECK(overlay.points.front().emitterId == Catch::Approx(3.0F));
    CHECK(overlay.points.back().particleRole == Catch::Approx(1.0F));
    CHECK(overlay.points.back().confidence <= 1.0F);

    std::size_t anchorCount = 0;
    std::size_t mainGuideCount = 0;
    std::size_t trailLaneGuideCount = 0;
    std::size_t particleCount = 0;
    float previousAnchorDistance = -1.0F;
    bool sawDifferentParticleBlue = false;
    std::uint8_t firstParticleBlue = 0;
    bool hasFirstParticleBlue = false;
    for (const auto& point : overlay.points) {
        const glm::vec3 normal{point.normal.x, point.normal.y, point.normal.z};
        CHECK(std::isfinite(normal.x));
        CHECK(std::isfinite(normal.y));
        CHECK(std::isfinite(normal.z));
        CHECK(glm::length(normal) == Catch::Approx(1.0F).margin(0.0001F));
        CHECK(glm::dot(normal, glm::vec3{1.0F, 0.0F, 0.0F}) > 0.99F);
        if (point.particleRole < 0.5F) {
            ++anchorCount;
            if (previousAnchorDistance >= 0.0F) {
                CHECK(point.pathDistance - previousAnchorDistance <= settings.trail.splineAnchorSpacing + 0.001F);
            }
            previousAnchorDistance = point.pathDistance;
            continue;
        }
        if (point.particleRole >= 1.5F && point.particleRole < 2.5F) {
            ++mainGuideCount;
            CHECK(point.pathStartIndex >= 0.0F);
            CHECK(point.pathPointCount >= 2.0F);
            CHECK(point.pathStartIndex + point.pathPointCount <= static_cast<float>(overlay.points.size()));
            continue;
        }
        if (point.particleRole >= 2.5F && point.particleRole < 3.5F) {
            ++trailLaneGuideCount;
            CHECK(point.pathStartIndex >= 0.0F);
            CHECK(point.pathPointCount >= 2.0F);
            CHECK(point.pathStartIndex + point.pathPointCount <= static_cast<float>(overlay.points.size()));
            CHECK(point.trailLaneId >= 0.0F);
            CHECK(std::abs(point.trailLateralOffset) <= point.width * settings.trail.particleJitter * 0.46F + 0.001F);
            continue;
        }
        ++particleCount;
        CHECK(point.blue >= point.green);
        CHECK(point.blue > point.red);
        CHECK(point.speed > emitter.speed);
        CHECK(point.pathStartIndex >= 0.0F);
        CHECK(point.pathPointCount >= 2.0F);
        CHECK(point.pathStartIndex + point.pathPointCount <= static_cast<float>(overlay.points.size()));
        CHECK(point.trailAge >= 0.0F);
        CHECK(point.trailAge <= 1.0F);
        CHECK(point.trailLength >= 0.0F);
        CHECK(point.featureType == Catch::Approx(0.0F));
        if (!hasFirstParticleBlue) {
            firstParticleBlue = point.blue;
            hasFirstParticleBlue = true;
        } else if (point.blue != firstParticleBlue) {
            sawDifferentParticleBlue = true;
        }
    }
    CHECK(anchorCount > 8);
    CHECK(mainGuideCount > 8);
    CHECK(trailLaneGuideCount > mainGuideCount);
    CHECK(particleCount > 8);
    CHECK(sawDifferentParticleBlue);

    struct LaneMetrics {
        std::size_t count = 0;
        std::uint32_t maxLaneId = 0;
        double positionSignature = 0.0;
        double absoluteOffset = 0.0;
        double steepness = 0.0;
    };
    const auto laneMetricsForEmitter =
        [](const invisible_places::water::WaterOverlay& waterOverlay, std::uint32_t emitterId) {
            LaneMetrics metrics;
            for (const auto& point : waterOverlay.points) {
                if (point.particleRole < 2.5F || point.particleRole >= 3.5F) {
                    continue;
                }
                const auto pointEmitterId = static_cast<std::uint32_t>(
                    std::max(0.0F, std::floor(point.emitterId + 0.5F)));
                if (emitterId != 0U && pointEmitterId != emitterId) {
                    continue;
                }
                ++metrics.count;
                metrics.maxLaneId = std::max(
                    metrics.maxLaneId,
                    static_cast<std::uint32_t>(std::max(0.0F, std::floor(point.trailLaneId + 0.5F))));
                metrics.positionSignature +=
                    static_cast<double>(point.position.x) * 13.0 +
                    static_cast<double>(point.position.y) * 17.0 +
                    static_cast<double>(point.position.z) * 19.0;
                metrics.absoluteOffset += static_cast<double>(std::abs(point.trailLateralOffset));
                metrics.steepness += static_cast<double>(point.surfaceSteepness);
            }
            return metrics;
        };
    const auto laneSignatureChanged = [](LaneMetrics left, LaneMetrics right) {
        return std::abs(left.positionSignature - right.positionSignature) > 1.0e-4 ||
               std::abs(left.absoluteOffset - right.absoluteOffset) > 1.0e-4 ||
               std::abs(left.steepness - right.steepness) > 1.0e-4;
    };
    const auto countRoles = [](const invisible_places::water::WaterOverlay& waterOverlay, bool particles) {
        return std::count_if(
            waterOverlay.points.begin(),
            waterOverlay.points.end(),
            [particles](const invisible_places::water::WaterOverlayPoint& point) {
                return particles
                           ? (point.particleRole >= 0.5F && point.particleRole < 1.5F)
                           : point.particleRole < 0.5F;
            });
    };
    const auto countMovingParticleHeads = [](const invisible_places::water::WaterOverlay& waterOverlay) {
        return std::count_if(
            waterOverlay.points.begin(),
            waterOverlay.points.end(),
            [](const invisible_places::water::WaterOverlayPoint& point) {
                return point.particleRole >= 0.5F &&
                       point.particleRole < 1.5F &&
                       point.trailAge <= 1.0e-5F;
            });
    };

    invisible_places::water::WaterParticleTrailShapeSettings shapedTrail;
    shapedTrail.particleJitter = settings.trail.particleJitter;
    shapedTrail.splineAnchorSpacing = settings.trail.splineAnchorSpacing;
    shapedTrail.trailLaneCount = 4U;
    shapedTrail.trailLooseness = 0.15F;
    invisible_places::water::WaterAnimationTrailSettings shapedAnimation;
    shapedAnimation.particleDensity = settings.trail.particleDensity;
    shapedAnimation.particleSpeed = settings.trail.particleSpeed;
    shapedAnimation.colorVariation = settings.visual.colorVariation;
    shapedAnimation.trailLengthMeters = 0.4F;
    const auto shapedOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        shapedTrail,
        shapedAnimation);
    const auto shapedLaneMetrics = laneMetricsForEmitter(shapedOverlay, 0U);
    REQUIRE(shapedLaneMetrics.count > 0U);
    CHECK(shapedLaneMetrics.maxLaneId == shapedTrail.trailLaneCount - 1U);
    CHECK(std::any_of(shapedOverlay.points.begin(), shapedOverlay.points.end(), [&shapedOverlay](const auto& point) {
        if (point.particleRole < 0.5F || point.particleRole >= 1.5F) {
            return false;
        }
        const auto pathStart = static_cast<std::size_t>(
            std::max(0.0F, std::floor(point.pathStartIndex + 0.5F)));
        return pathStart < shapedOverlay.points.size() &&
               shapedOverlay.points[pathStart].particleRole >= 2.5F &&
               shapedOverlay.points[pathStart].particleRole < 3.5F;
    }));

    auto fineSampleAnimation = shapedAnimation;
    fineSampleAnimation.trailSampleSpacingMeters = shapedTrail.splineAnchorSpacing * 0.5F;
    const auto fineSampleOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        shapedTrail,
        fineSampleAnimation);
    CHECK(countRoles(fineSampleOverlay, false) > countRoles(shapedOverlay, false));
    CHECK(countMovingParticleHeads(fineSampleOverlay) == countMovingParticleHeads(shapedOverlay));
    CHECK(countRoles(fineSampleOverlay, true) > countRoles(shapedOverlay, true));

    auto wideSampleAnimation = shapedAnimation;
    wideSampleAnimation.trailSampleSpacingMeters = shapedTrail.splineAnchorSpacing * 3.0F;
    const auto wideSampleOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        shapedTrail,
        wideSampleAnimation);
    CHECK(countRoles(wideSampleOverlay, false) < countRoles(shapedOverlay, false));
    CHECK(countMovingParticleHeads(wideSampleOverlay) == countMovingParticleHeads(shapedOverlay));

    auto laneCountTrail = shapedTrail;
    laneCountTrail.trailLaneCount = 7U;
    const auto laneCountOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        laneCountTrail,
        shapedAnimation);
    const auto laneCountMetrics = laneMetricsForEmitter(laneCountOverlay, 0U);
    CHECK(laneCountMetrics.count > shapedLaneMetrics.count);
    CHECK(laneCountMetrics.maxLaneId == laneCountTrail.trailLaneCount - 1U);

    auto jitterTrail = shapedTrail;
    jitterTrail.particleJitter = 0.95F;
    const auto jitterOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        jitterTrail,
        shapedAnimation);
    CHECK(laneMetricsForEmitter(jitterOverlay, 0U).absoluteOffset >
          shapedLaneMetrics.absoluteOffset * 1.15);

    auto anchorSpacingTrail = shapedTrail;
    anchorSpacingTrail.splineAnchorSpacing = shapedTrail.splineAnchorSpacing * 0.5F;
    const auto anchorSpacingOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        anchorSpacingTrail,
        shapedAnimation);
    CHECK(laneMetricsForEmitter(anchorSpacingOverlay, 0U).count > shapedLaneMetrics.count);

    auto looseTrail = shapedTrail;
    looseTrail.trailLooseness = 0.95F;
    const auto looseOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        looseTrail,
        shapedAnimation);
    CHECK(laneSignatureChanged(laneMetricsForEmitter(looseOverlay, 0U), shapedLaneMetrics));

    auto smoothTrail = shapedTrail;
    smoothTrail.trailSmoothness = 0.98F;
    const auto smoothOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        smoothTrail,
        shapedAnimation);
    CHECK(laneSignatureChanged(laneMetricsForEmitter(smoothOverlay, 0U), shapedLaneMetrics));

    auto denserTrailSettings = settings.trail;
    denserTrailSettings.particleDensity = 3.0F;
    const auto denserOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        denserTrailSettings,
        settings.visual);
    CHECK(countRoles(denserOverlay, false) == countRoles(overlay, false));
    CHECK(countRoles(denserOverlay, true) > countRoles(overlay, true));

    invisible_places::water::WaterSourceSettings defaultSourceSettings;
    defaultSourceSettings.path = settings.path;
    defaultSourceSettings.trailShape.particleJitter = settings.trail.particleJitter;
    defaultSourceSettings.trailShape.splineAnchorSpacing = settings.trail.splineAnchorSpacing;
    defaultSourceSettings.trailShape.trailLaneCount = 3U;
    invisible_places::water::WaterAnimationTrailSettings animationTrailSettings;
    animationTrailSettings.particleDensity = settings.trail.particleDensity;
    animationTrailSettings.particleSpeed = settings.trail.particleSpeed;
    animationTrailSettings.colorVariation = settings.visual.colorVariation;
    auto customEmitter = emitter;
    customEmitter.id = 9;
    customEmitter.name = "custom spring";
    customEmitter.sourceSettingsAssignment = invisible_places::water::WaterSourceSettingsAssignment::Custom;
    customEmitter.sourceSettings = defaultSourceSettings;
    customEmitter.sourceSettings->path.pathLength = 0.28F;
    customEmitter.sourceSettings->trailShape.splineAnchorSpacing = 0.02F;
    customEmitter.sourceSettings->trailShape.trailLaneCount = 6U;
    const std::vector<invisible_places::water::WaterEmitter> perSourceEmitters{emitter, customEmitter};
    const auto perSourceAnchors = invisible_places::water::GenerateWaterPathAnchors(
        cloud,
        perSourceEmitters,
        defaultSourceSettings);
    const auto maxAnchorDistanceForEmitter =
        [](const invisible_places::water::WaterOverlay& waterOverlay, std::uint32_t emitterId) {
            float maxDistance = 0.0F;
            for (const auto& point : waterOverlay.points) {
                if (point.particleRole < 0.5F &&
                    static_cast<std::uint32_t>(point.emitterId + 0.5F) == emitterId) {
                    maxDistance = std::max(maxDistance, point.pathDistance);
                }
            }
            return maxDistance;
        };
    CHECK(maxAnchorDistanceForEmitter(perSourceAnchors, 9U) <
          maxAnchorDistanceForEmitter(perSourceAnchors, 3U));

    const auto perSourceOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        perSourceAnchors,
        perSourceEmitters,
        defaultSourceSettings,
        animationTrailSettings);
    const auto defaultEmitterLaneMetrics = laneMetricsForEmitter(perSourceOverlay, 3U);
    const auto customEmitterLaneMetrics = laneMetricsForEmitter(perSourceOverlay, 9U);
    REQUIRE(defaultEmitterLaneMetrics.count > 0U);
    REQUIRE(customEmitterLaneMetrics.count > 0U);
    CHECK(defaultEmitterLaneMetrics.maxLaneId == defaultSourceSettings.trailShape.trailLaneCount - 1U);
    CHECK(customEmitterLaneMetrics.maxLaneId == customEmitter.sourceSettings->trailShape.trailLaneCount - 1U);

    auto customLaneEmitters = perSourceEmitters;
    REQUIRE(customLaneEmitters[1].sourceSettings.has_value());
    customLaneEmitters[1].sourceSettings->trailShape.trailLaneCount = 8U;
    const auto customLaneOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        perSourceAnchors,
        customLaneEmitters,
        defaultSourceSettings,
        animationTrailSettings);
    const auto unchangedDefaultLaneMetrics = laneMetricsForEmitter(customLaneOverlay, 3U);
    const auto widerCustomLaneMetrics = laneMetricsForEmitter(customLaneOverlay, 9U);
    CHECK(unchangedDefaultLaneMetrics.count == defaultEmitterLaneMetrics.count);
    CHECK(unchangedDefaultLaneMetrics.maxLaneId == defaultEmitterLaneMetrics.maxLaneId);
    CHECK(unchangedDefaultLaneMetrics.positionSignature ==
          Catch::Approx(defaultEmitterLaneMetrics.positionSignature));
    CHECK(widerCustomLaneMetrics.count > customEmitterLaneMetrics.count);
    CHECK(widerCustomLaneMetrics.maxLaneId == 7U);

    const auto particleCountForEmitter =
        [](const invisible_places::water::WaterOverlay& waterOverlay, std::uint32_t emitterId) {
            std::size_t count = 0;
            for (const auto& point : waterOverlay.points) {
                if (point.particleRole >= 0.5F &&
                    point.particleRole < 1.5F &&
                    static_cast<std::uint32_t>(point.emitterId + 0.5F) == emitterId) {
                    ++count;
                }
            }
            return count;
        };
    CHECK(particleCountForEmitter(perSourceOverlay, 3U) > 0U);
    CHECK(particleCountForEmitter(perSourceOverlay, 9U) > 0U);

    auto denserCustomEmitters = perSourceEmitters;
    REQUIRE(denserCustomEmitters[1].sourceSettings.has_value());
    denserCustomEmitters[1].sourceSettings->trailShape.splineAnchorSpacing = 0.01F;
    const auto denserCustomOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        perSourceAnchors,
        denserCustomEmitters,
        defaultSourceSettings,
        animationTrailSettings);
    const auto countEmitterRole =
        [](const invisible_places::water::WaterOverlay& waterOverlay, std::uint32_t emitterId, bool particles) {
            return std::count_if(
                waterOverlay.points.begin(),
                waterOverlay.points.end(),
                [emitterId, particles](const invisible_places::water::WaterOverlayPoint& point) {
                    const bool isParticle = point.particleRole >= 0.5F && point.particleRole < 1.5F;
                    return isParticle == particles &&
                           (particles || point.particleRole < 0.5F) &&
                           static_cast<std::uint32_t>(point.emitterId + 0.5F) == emitterId;
                });
        };
    CHECK(countEmitterRole(denserCustomOverlay, 9U, true) >
          countEmitterRole(perSourceOverlay, 9U, true));

    auto visualOnlySettings = settings.visual;
    visualOnlySettings.glow = 1.25F;
    visualOnlySettings.particleOpacity = 0.12F;
    const auto visualOnlyOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        pathAnchors,
        settings.trail,
        visualOnlySettings);
    CHECK(countRoles(visualOnlyOverlay, false) == countRoles(overlay, false));
    CHECK(countRoles(visualOnlyOverlay, true) == countRoles(overlay, true));
    REQUIRE_FALSE(visualOnlyOverlay.points.empty());
    CHECK(visualOnlyOverlay.points.front().position.z == Catch::Approx(overlay.points.front().position.z));

    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_water_overlay_test.ply";
    std::string errorMessage;
    REQUIRE(invisible_places::water::WriteWaterOverlayPly(overlay, outputPath, &errorMessage));

    const auto header = invisible_places::io::ParsePlyHeader(outputPath);
    REQUIRE(header.success);
    CHECK(header.header.vertexCount == overlay.points.size());
    CHECK(header.header.HasProperty("scalar_phase"));
    CHECK(header.header.HasProperty("scalar_speed"));
    CHECK(header.header.HasProperty("scalar_pooling"));
    CHECK(header.header.HasProperty("scalar_particle_role"));
    CHECK(header.header.HasProperty("scalar_path_start_index"));
    CHECK(header.header.HasProperty("scalar_path_point_count"));
    CHECK(header.header.HasProperty("scalar_jitter_seed"));
    CHECK(header.header.HasProperty("scalar_trail_age"));
    CHECK(header.header.HasProperty("scalar_trail_length"));
    CHECK(header.header.HasProperty("scalar_feature_type"));
    CHECK(header.header.HasProperty("scalar_region_id"));
    CHECK(header.header.HasProperty("scalar_surface_steepness"));
    CHECK(header.header.HasProperty("scalar_trail_lane_id"));
    CHECK(header.header.HasProperty("scalar_trail_lateral_offset"));
    CHECK(header.header.HasProperty("normal_x"));
    CHECK(header.header.HasProperty("normal_y"));
    CHECK(header.header.HasProperty("normal_z"));

    const auto loaded = invisible_places::io::LoadPointCloud(outputPath);
    REQUIRE(loaded.success);
    REQUIRE(loaded.cloud.ScalarFieldCount() == 20);
    REQUIRE(loaded.cloud.hasNormals);
    REQUIRE(loaded.cloud.normals.size() == overlay.points.size());
    CHECK(loaded.cloud.normals.front().x == Catch::Approx(1.0F));
    CHECK(loaded.cloud.normals.front().y == Catch::Approx(0.0F));
    CHECK(loaded.cloud.normals.front().z == Catch::Approx(0.0F));
    CHECK(loaded.cloud.scalarFields[3].name == "phase");
    CHECK(loaded.cloud.scalarFields[4].name == "speed");
    CHECK(loaded.cloud.scalarFields[8].name == "pooling");
    CHECK(loaded.cloud.scalarFields[9].name == "particle_role");
    CHECK(loaded.cloud.scalarFields[10].name == "path_start_index");
    CHECK(loaded.cloud.scalarFields[11].name == "path_point_count");
    CHECK(loaded.cloud.scalarFields[12].name == "jitter_seed");
    CHECK(loaded.cloud.scalarFields[13].name == "trail_age");
    CHECK(loaded.cloud.scalarFields[14].name == "trail_length");
    CHECK(loaded.cloud.scalarFields[15].name == "feature_type");
    CHECK(loaded.cloud.scalarFields[16].name == "region_id");
    CHECK(loaded.cloud.scalarFields[17].name == "surface_steepness");
    CHECK(loaded.cloud.scalarFields[18].name == "trail_lane_id");
    CHECK(loaded.cloud.scalarFields[19].name == "trail_lateral_offset");

    const auto liveCloud = invisible_places::water::BuildWaterOverlayPointCloud(
        overlay,
        outputPath,
        "water overlay preview");
    REQUIRE(liveCloud.PointCount() == overlay.points.size());
    REQUIRE(liveCloud.ScalarFieldCount() == 20);
    CHECK(liveCloud.sourcePath == outputPath);
    CHECK(liveCloud.layerName == "water overlay preview");
    CHECK(liveCloud.hasSourceRgb);
    REQUIRE(liveCloud.hasNormals);
    REQUIRE(liveCloud.normals.size() == overlay.points.size());
    CHECK(liveCloud.normals.front().x == Catch::Approx(1.0F));
    CHECK(liveCloud.scalarFields[16].name == "region_id");
    CHECK(liveCloud.scalarFields[17].name == "surface_steepness");
    CHECK(liveCloud.scalarFields[18].name == "trail_lane_id");
    CHECK(liveCloud.scalarFields[19].name == "trail_lateral_offset");
    const auto firstLanePoint = std::find_if(
        overlay.points.begin(),
        overlay.points.end(),
        [](const invisible_places::water::WaterOverlayPoint& point) {
            return point.particleRole >= 2.5F && point.particleRole < 3.5F;
        });
    REQUIRE(firstLanePoint != overlay.points.end());
    const auto firstLaneIndex = static_cast<std::size_t>(std::distance(overlay.points.begin(), firstLanePoint));
    CHECK(liveCloud.scalarFieldValues[liveCloud.ScalarFieldValueIndex(17, firstLaneIndex)] ==
          Catch::Approx(firstLanePoint->surfaceSteepness));
    CHECK(liveCloud.scalarFieldValues[liveCloud.ScalarFieldValueIndex(18, firstLaneIndex)] ==
          Catch::Approx(firstLanePoint->trailLaneId));
    CHECK(liveCloud.scalarFieldValues[liveCloud.ScalarFieldValueIndex(19, firstLaneIndex)] ==
          Catch::Approx(firstLanePoint->trailLateralOffset));
    std::filesystem::remove(outputPath);
}

TEST_CASE("Water v2 streams expose deterministic scalar contracts", "[water][v2]") {
    invisible_places::water::WaterOverlay anchors;
    for (std::uint32_t index = 0; index < 6U; ++index) {
        invisible_places::water::WaterOverlayPoint point;
        point.position = {static_cast<float>(index) * 0.20F, 0.05F * static_cast<float>(index % 2U), 0.0F};
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = 10.0F;
        point.emitterId = 5.0F;
        point.pathDistance = static_cast<float>(index) * 0.20F;
        point.width = 0.08F;
        point.confidence = 0.75F;
        point.accumulation = 0.5F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::water::WaterFlowStreamSettings streamSettings;
    streamSettings.streamCountTotal = 8U;
    streamSettings.streamLengthMeters = 0.36F;
    streamSettings.streamPointSpacingMeters = 0.09F;
    streamSettings.streamWidthMeters = 0.012F;
    streamSettings.streamWorldLengthMeters = 0.050F;
    streamSettings.laneSpreadMeters = 0.04F;
    streamSettings.turbulence = 0.03F;
    streamSettings.seed = 123U;
    const auto flowA = invisible_places::water::BuildFlowStreamOverlayFromPathAnchors(anchors, streamSettings);
    const auto flowB = invisible_places::water::BuildFlowStreamOverlayFromPathAnchors(anchors, streamSettings);
    REQUIRE_FALSE(flowA.samples.empty());
    REQUIRE(flowA.samples.size() == flowB.samples.size());
    CHECK(flowA.samples.front().position.x == Catch::Approx(flowB.samples.front().position.x));
    CHECK(flowA.samples.front().tangent.x == Catch::Approx(flowB.samples.front().tangent.x));
    CHECK(flowA.samples.back().pointSeed == Catch::Approx(flowB.samples.back().pointSeed));

    const auto cloud = invisible_places::water::BuildWaterStreamOverlayPointCloud(
        flowA,
        "Saved/water/test-WaterFlowStreams.generated",
        "flow streams");
    const std::vector<std::string> expectedFields{
        "stream_id",
        "source_id",
        "path_id",
        "branch_id",
        "stream_seed",
        "point_seed",
        "stream_distance",
        "stream_length",
        "point_age",
        "stream_age",
        "stream_speed",
        "stream_width",
        "stream_world_length",
        "stream_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z"};
    REQUIRE(cloud.ScalarFieldCount() == expectedFields.size());
    for (std::size_t index = 0; index < expectedFields.size(); ++index) {
        CHECK(cloud.scalarFields[index].name == expectedFields[index]);
    }
    REQUIRE(cloud.PointCount() == flowA.samples.size());
    const float firstStreamWidth = cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(11, 0)];
    const float firstStreamWorldLength = cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(12, 0)];
    CHECK(firstStreamWidth >= streamSettings.streamWidthMeters * 0.80F);
    CHECK(firstStreamWidth <= streamSettings.streamWidthMeters * 1.22F);
    CHECK(firstStreamWorldLength >= std::max(streamSettings.streamWorldLengthMeters, streamSettings.streamPointSpacingMeters * 2.5F));
    CHECK(firstStreamWorldLength >= firstStreamWidth * 2.0F);

    invisible_places::water::WaterFieldSettings fieldSettings;
    fieldSettings.corridorRadiusMeters = 0.18F;
    fieldSettings.fieldResolutionMeters = 0.05F;
    fieldSettings.seed = 456U;
    const auto fieldCacheA = invisible_places::water::BuildFieldCacheFromPathAnchors(anchors, fieldSettings);
    const auto fieldCacheB = invisible_places::water::BuildFieldCacheFromPathAnchors(anchors, fieldSettings);
    REQUIRE_FALSE(fieldCacheA.nodes.empty());
    REQUIRE(fieldCacheA.nodes.size() == fieldCacheB.nodes.size());
    CHECK(fieldCacheA.nodes.front().position.x == Catch::Approx(fieldCacheB.nodes.front().position.x));
    CHECK(fieldCacheA.nodes.front().confidence == Catch::Approx(fieldCacheB.nodes.front().confidence));

    invisible_places::water::WaterFieldStreamSettings fieldStreamSettings;
    fieldStreamSettings.streamlineCount = 7U;
    fieldStreamSettings.streamlineLengthMeters = 0.30F;
    fieldStreamSettings.stepLengthMeters = 0.06F;
    fieldStreamSettings.streamlineWidthMeters = 0.008F;
    fieldStreamSettings.streamWorldLengthMeters = 0.042F;
    const auto fieldStreamA = invisible_places::water::BuildFieldStreamOverlay(fieldCacheA, fieldStreamSettings);
    const auto fieldStreamB = invisible_places::water::BuildFieldStreamOverlay(fieldCacheA, fieldStreamSettings);
    REQUIRE_FALSE(fieldStreamA.samples.empty());
    REQUIRE(fieldStreamA.samples.size() == fieldStreamB.samples.size());
    CHECK(fieldStreamA.samples.front().position.x == Catch::Approx(fieldStreamB.samples.front().position.x));
    CHECK(fieldStreamA.samples.front().featureType == Catch::Approx(3.0F));

    invisible_places::water::WaterEffectLayer fieldLayer;
    fieldLayer.featureType = invisible_places::water::WaterEffectFeatureType::FieldSurfaceMotion;
    fieldLayer.maxAffectedPoints = 128U;
    const auto surfaceMotion = invisible_places::water::GenerateFieldSurfaceEffectOverlay(fieldCacheA, fieldLayer);
    REQUIRE_FALSE(surfaceMotion.points.empty());
    CHECK(surfaceMotion.points.front().featureType == Catch::Approx(2.0F));
}

TEST_CASE("Water ripple overlay types generate distinct procedural effect fields", "[water][v2][ripples]") {
    const auto cloud = MakeRippleFixtureCloud();
    const std::array<invisible_places::water::WaterRippleOverlayType, 11> overlayTypes{
        invisible_places::water::WaterRippleOverlayType::CausticLace,
        invisible_places::water::WaterRippleOverlayType::LinearRipples,
        invisible_places::water::WaterRippleOverlayType::RadialRipples,
        invisible_places::water::WaterRippleOverlayType::RainRings,
        invisible_places::water::WaterRippleOverlayType::TideBands,
        invisible_places::water::WaterRippleOverlayType::WetSheen,
        invisible_places::water::WaterRippleOverlayType::CurrentThreads,
        invisible_places::water::WaterRippleOverlayType::DropletGlints,
        invisible_places::water::WaterRippleOverlayType::DripTrails,
        invisible_places::water::WaterRippleOverlayType::FoamSparkle,
        invisible_places::water::WaterRippleOverlayType::SaltMineralShimmer,
    };

    std::set<std::string> fingerprints;
    for (std::size_t typeIndex = 0; typeIndex < overlayTypes.size(); ++typeIndex) {
        const auto type = overlayTypes[typeIndex];
        INFO("Ripple overlay type index: " << typeIndex);
        const auto layer = MakeRippleTestLayer(type, static_cast<std::uint32_t>(typeIndex + 1U));
        const auto overlay = invisible_places::water::GenerateRippleEffectOverlay(cloud, {layer});
        REQUIRE_FALSE(overlay.points.empty());

        float sum = 0.0F;
        float weightedX = 0.0F;
        float weightedY = 0.0F;
        float maxValue = 0.0F;
        for (const auto& point : overlay.points) {
            sum += point.value;
            weightedX += point.value * point.position.x;
            weightedY += point.value * point.position.y;
            maxValue = std::max(maxValue, point.value);
        }
        CHECK(maxValue > 0.001F);
        CHECK(sum > 0.001F);

        std::ostringstream signature;
        signature << std::llround(sum * 10000.0F) << ":"
                  << std::llround(weightedX * 10000.0F) << ":"
                  << std::llround(weightedY * 10000.0F) << ":"
                  << std::llround(maxValue * 10000.0F);
        CHECK(fingerprints.insert(signature.str()).second);
    }
    CHECK(fingerprints.size() == overlayTypes.size());

    const auto linear = invisible_places::water::GenerateRippleEffectOverlay(
        cloud,
        {MakeRippleTestLayer(invisible_places::water::WaterRippleOverlayType::LinearRipples, 20U)});
    const auto linearTop = RippleValueAt(linear, 0.25F, 0.25F);
    const auto linearBottom = RippleValueAt(linear, 0.25F, 0.75F);
    REQUIRE(linearTop.has_value());
    REQUIRE(linearBottom.has_value());
    CHECK(linearTop.value() == Catch::Approx(linearBottom.value()).margin(1.0e-5F));

    const auto radial = invisible_places::water::GenerateRippleEffectOverlay(
        cloud,
        {MakeRippleTestLayer(invisible_places::water::WaterRippleOverlayType::RadialRipples, 21U)});
    const auto radialLeft = RippleValueAt(radial, 0.25F, 0.5F);
    const auto radialRight = RippleValueAt(radial, 0.75F, 0.5F);
    REQUIRE(radialLeft.has_value());
    REQUIRE(radialRight.has_value());
    CHECK(radialLeft.value() == Catch::Approx(radialRight.value()).margin(1.0e-5F));

    const auto wetSheen = invisible_places::water::GenerateRippleEffectOverlay(
        cloud,
        {MakeRippleTestLayer(invisible_places::water::WaterRippleOverlayType::WetSheen, 22U)});
    const auto flatWetSheen = RippleValueAt(wetSheen, 0.25F, 0.5F);
    const auto slopedWetSheen = RippleValueAt(wetSheen, 0.75F, 0.5F);
    REQUIRE(flatWetSheen.has_value());
    REQUIRE(slopedWetSheen.has_value());
    CHECK(slopedWetSheen.value() > flatWetSheen.value());
}

TEST_CASE("Ripple and Field effects compose onto base cloud visual evaluation", "[water][v2][effects][output]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.normals = {{0.0F, 0.0F, 1.0F}};
    cloud.packedColors = {0xFFFFFFFFU};
    cloud.hasNormals = true;
    cloud.hasSourceRgb = true;

    auto addScalarField = [&](std::string_view name, std::vector<float> values) -> std::size_t {
        REQUIRE(values.size() == cloud.positions.size());
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::string{name};
        for (const float value : values) {
            stats.Include(value);
        }
        const auto slot = cloud.scalarFields.size();
        cloud.scalarFields.push_back(stats);
        cloud.scalarFieldValues.insert(cloud.scalarFieldValues.end(), values.begin(), values.end());
        return slot;
    };

    const auto heightSlot = addScalarField("Height", {1.0F});
    const auto intensitySlot = addScalarField("Intensity", {0.40F});
    cloud.scalarFields[heightSlot].minimum = 0.0F;
    cloud.scalarFields[heightSlot].maximum = 1.0F;
    cloud.scalarFields[intensitySlot].minimum = 0.0F;
    cloud.scalarFields[intensitySlot].maximum = 1.0F;

    invisible_places::water::WaterEffectPoint ripple;
    ripple.sourcePointIndex = 0U;
    ripple.featureType = 1.0F;
    ripple.value = 1.0F;
    ripple.emissionHint = 0.35F;
    ripple.opacityHint = 0.20F;
    ripple.opacityMultiplyHint = 1.25F;
    ripple.sizeMultiplyHint = 1.0F;
    ripple.colourMixHint = 0.60F;
    ripple.red = 24U;
    ripple.green = 90U;
    ripple.blue = 255U;

    invisible_places::water::WaterEffectPoint field = ripple;
    field.featureType = 2.0F;
    field.emissionHint = 0.25F;
    field.opacityHint = 0.08F;
    field.opacityMultiplyHint = 1.10F;
    field.colourMixHint = 0.25F;
    field.red = 40U;
    field.green = 240U;
    field.blue = 190U;

    invisible_places::water::WaterEffectOverlay rippleOverlay;
    rippleOverlay.points.push_back(ripple);
    invisible_places::water::WaterEffectOverlay fieldOverlay;
    fieldOverlay.points.push_back(field);
    const auto composition = invisible_places::water::ComposeWaterEffectFields(
        cloud,
        {rippleOverlay, fieldOverlay});
    REQUIRE(composition.affectedPointCount == 1U);
    CHECK(composition.emissionAdd[0] == Catch::Approx(0.60F));
    CHECK(composition.opacityAdd[0] == Catch::Approx(0.28F));
    CHECK(composition.opacityMultiply[0] == Catch::Approx(1.375F));
    CHECK(composition.colourMix[0] == Catch::Approx(0.85F));

    const auto valueSlot = addScalarField("water_effect_value", composition.value);
    const auto emissionSlot = addScalarField("water_effect_emission_add", composition.emissionAdd);
    const auto opacityAddSlot = addScalarField("water_effect_opacity_add", composition.opacityAdd);
    const auto opacityMultiplySlot = addScalarField("water_effect_opacity_multiply", composition.opacityMultiply);
    const auto pointSizeAddSlot = addScalarField("water_effect_point_size_add", composition.pointSizeAdd);
    const auto pointSizeMultiplySlot = addScalarField("water_effect_point_size_multiply", composition.pointSizeMultiply);
    const auto colourRedSlot = addScalarField("water_effect_colour_red", composition.colourRed);
    const auto colourGreenSlot = addScalarField("water_effect_colour_green", composition.colourGreen);
    const auto colourBlueSlot = addScalarField("water_effect_colour_blue", composition.colourBlue);
    const auto colourMixSlot = addScalarField("water_effect_colour_mix", composition.colourMix);
    CHECK(valueSlot == 2U);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap;
    style.colormap = invisible_places::renderer::pointcloud::PointCloudColormapId::CustomGradient;
    style.gradientStartColor = {1.0F, 0.0F, 0.0F};
    style.gradientEndColor = {1.0F, 0.0F, 0.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    invisible_places::style::SetScalarConstant(&style.pointSize, 7.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::ConfigureFieldMapFromStats(
        &style.colormapPosition,
        static_cast<std::int32_t>(heightSlot),
        "Height",
        0.0F,
        1.0F,
        &cloud.scalarFields[heightSlot]);
    invisible_places::style::ConfigureFieldMapFromStats(
        &style.opacity,
        static_cast<std::int32_t>(intensitySlot),
        "Intensity",
        0.0F,
        1.0F,
        &cloud.scalarFields[intensitySlot]);

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 0.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto render = [&](invisible_places::output::OfflinePointLayer layer) {
        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 11, 11);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 11, 11},
            &image);
        return image;
    };

    const invisible_places::output::OfflinePointLayer baseLayer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };
    const invisible_places::output::OfflinePointLayer effectLayer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
        .waterEffectEmissionAddFieldSlot = emissionSlot,
        .waterEffectOpacityAddFieldSlot = opacityAddSlot,
        .waterEffectOpacityMultiplyFieldSlot = opacityMultiplySlot,
        .waterEffectPointSizeAddFieldSlot = pointSizeAddSlot,
        .waterEffectPointSizeMultiplyFieldSlot = pointSizeMultiplySlot,
        .waterEffectColourRedFieldSlot = colourRedSlot,
        .waterEffectColourGreenFieldSlot = colourGreenSlot,
        .waterEffectColourBlueFieldSlot = colourBlueSlot,
        .waterEffectColourMixFieldSlot = colourMixSlot,
    };

    const auto baseImage = render(baseLayer);
    const auto effectImage = render(effectLayer);
    const auto center = static_cast<std::size_t>(5) * 11U + 5U;
    REQUIRE(baseImage.alpha[center] > 0.0F);
    REQUIRE(effectImage.alpha[center] > 0.0F);
    CHECK(style.colormapPosition.fieldMap.fieldSlot == static_cast<std::int32_t>(heightSlot));
    CHECK(style.opacity.fieldMap.fieldSlot == static_cast<std::int32_t>(intensitySlot));
    CHECK(effectImage.alpha[center] > baseImage.alpha[center]);
    CHECK(baseImage.beautyR[center] > baseImage.beautyB[center]);
    CHECK(effectImage.beautyB[center] > effectImage.beautyR[center]);

    const auto exportPath =
        std::filesystem::temp_directory_path() / "invisible_places_water_effect_composition_export.exr";
    std::error_code removeError;
    std::filesystem::remove(exportPath, removeError);
    std::string errorMessage;
    REQUIRE(invisible_places::output::WriteExrImage(effectImage, exportPath, &errorMessage));
    CHECK(std::filesystem::exists(exportPath));
    CHECK(std::filesystem::file_size(exportPath) > 0U);
    std::filesystem::remove(exportPath, removeError);

    auto toHalfRgba = [](const invisible_places::output::ExrImage& image) {
        invisible_places::output::HalfRgbaExrImage halfImage;
        halfImage.width = image.width;
        halfImage.height = image.height;
        halfImage.rgbaHalf.resize(static_cast<std::size_t>(image.width) * image.height * 4U);
        for (std::size_t pixel = 0; pixel < static_cast<std::size_t>(image.width) * image.height; ++pixel) {
            halfImage.rgbaHalf[pixel * 4U + 0U] = Imath::half{std::clamp(image.beautyR[pixel], 0.0F, 1.0F)}.bits();
            halfImage.rgbaHalf[pixel * 4U + 1U] = Imath::half{std::clamp(image.beautyG[pixel], 0.0F, 1.0F)}.bits();
            halfImage.rgbaHalf[pixel * 4U + 2U] = Imath::half{std::clamp(image.beautyB[pixel], 0.0F, 1.0F)}.bits();
            halfImage.rgbaHalf[pixel * 4U + 3U] = Imath::half{std::clamp(image.alpha[pixel], 0.0F, 1.0F)}.bits();
        }
        return halfImage;
    };
    const auto baseMp4Frame = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(toHalfRgba(baseImage));
    const auto effectMp4Frame = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(toHalfRgba(effectImage));
    REQUIRE(baseMp4Frame.size() == effectMp4Frame.size());
    const auto centerByte = center * 4U;
    CHECK(effectMp4Frame[centerByte + 3U] >= baseMp4Frame[centerByte + 3U]);
    CHECK(baseMp4Frame[centerByte + 0U] > baseMp4Frame[centerByte + 2U]);
    CHECK(effectMp4Frame[centerByte + 2U] > effectMp4Frame[centerByte + 0U]);
}

TEST_CASE("Ripple effect generation preserves concave clicked region boundaries", "[water][v2][ripples]") {
    const auto cloud = MakeRippleFixtureCloud();
    auto layer = MakeRippleTestLayer(
        invisible_places::water::WaterRippleOverlayType::LinearRipples,
        41U);
    layer.vertices = {
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        {1.0F, 0.25F, 0.0F},
        {0.25F, 0.25F, 0.0F},
        {0.25F, 0.75F, 0.0F},
        {1.0F, 0.75F, 0.0F},
        {1.0F, 1.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
    };
    layer.hull = invisible_places::water::BuildWaterRegionHull(layer.vertices);

    const auto overlay = invisible_places::water::GenerateRippleEffectOverlay(cloud, {layer});
    REQUIRE_FALSE(overlay.points.empty());

    const auto leftBarPoint = RippleValueAt(overlay, 0.125F, 0.5F);
    const auto bottomArmPoint = RippleValueAt(overlay, 0.75F, 0.125F);
    const auto cutOutPoint = RippleValueAt(overlay, 0.75F, 0.5F);

    CHECK(leftBarPoint.has_value());
    CHECK(bottomArmPoint.has_value());
    CHECK_FALSE(cutOutPoint.has_value());
}

TEST_CASE("Field cache builds from concave selected regions", "[water][v2][field]") {
    const auto cloud = MakeRippleFixtureCloud();
    auto layer = MakeRippleTestLayer(
        invisible_places::water::WaterRippleOverlayType::LinearRipples,
        51U);
    layer.name = "field c region";
    layer.featureType = invisible_places::water::WaterEffectFeatureType::FieldSurfaceMotion;
    layer.vertices = {
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        {1.0F, 0.25F, 0.0F},
        {0.25F, 0.25F, 0.0F},
        {0.25F, 0.75F, 0.0F},
        {1.0F, 0.75F, 0.0F},
        {1.0F, 1.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
    };
    layer.hull = invisible_places::water::BuildWaterRegionHull(layer.vertices);
    layer.edgeBlendWidth = 0.05F;
    layer.maxAffectedPoints = 4096U;
    layer.blendMode = invisible_places::water::WaterEffectBlendMode::Screen;
    layer.response.intensity = 1.0F;
    layer.response.emissionAdd = 0.33F;
    layer.response.opacityAdd = 0.21F;
    layer.response.opacityMultiply = 0.62F;
    layer.response.pointSizeAdd = 0.07F;
    layer.response.pointSizeMultiply = 1.80F;
    layer.response.colouriseRed = 0.18F;
    layer.response.colouriseGreen = 0.72F;
    layer.response.colouriseBlue = 0.94F;
    layer.response.colouriseAmount = 0.44F;
    layer.speed = 1.23F;
    layer.directionX = 1.0F;
    layer.directionY = 0.0F;
    layer.directionZ = 0.0F;
    auto noFlowLayer = layer;
    noFlowLayer.id = 52U;
    noFlowLayer.name = "field no-flow pocket";
    noFlowLayer.featureType = invisible_places::water::WaterEffectFeatureType::FieldNoFlowRegion;
    noFlowLayer.vertices = {
        {0.00F, 0.375F, 0.0F},
        {0.25F, 0.375F, 0.0F},
        {0.25F, 0.625F, 0.0F},
        {0.00F, 0.625F, 0.0F},
    };
    noFlowLayer.hull = invisible_places::water::BuildWaterRegionHull(noFlowLayer.vertices);

    invisible_places::water::WaterFieldSettings settings;
    settings.fieldResolutionMeters = 0.0625F;
    settings.guideWeight = 1.0F;
    settings.downhillWeight = 0.0F;
    settings.turbulence = 0.0F;
    const auto cache = invisible_places::water::BuildFieldCacheFromRegions(cloud, {layer, noFlowLayer}, settings);
    REQUIRE_FALSE(cache.nodes.empty());
    REQUIRE(cache.regionBoundary.size() == layer.vertices.size());

    auto hasNodeAt = [&cache](float x, float y) {
        return std::any_of(
            cache.nodes.begin(),
            cache.nodes.end(),
            [x, y](const invisible_places::water::WaterFieldNode& node) {
                return std::abs(node.position.x - x) <= 1.0e-5F &&
                       std::abs(node.position.y - y) <= 1.0e-5F;
            });
    };
    CHECK(hasNodeAt(0.125F, 0.5F));
    CHECK(hasNodeAt(0.75F, 0.125F));
    CHECK_FALSE(hasNodeAt(0.75F, 0.5F));
    const auto noFlowNode = std::find_if(
        cache.nodes.begin(),
        cache.nodes.end(),
        [](const invisible_places::water::WaterFieldNode& node) {
            return std::abs(node.position.x - 0.125F) <= 1.0e-5F &&
                   std::abs(node.position.y - 0.5F) <= 1.0e-5F;
        });
    REQUIRE(noFlowNode != cache.nodes.end());
    CHECK(noFlowNode->flowBlocked);

    invisible_places::water::WaterFieldStreamSettings streamSettings;
    streamSettings.streamlineCount = 24U;
    streamSettings.streamlineLengthMeters = 0.20F;
    streamSettings.stepLengthMeters = 0.04F;
    streamSettings.streamlineWidthMeters = 0.008F;
    streamSettings.streamWorldLengthMeters = 0.040F;
    const auto streamOverlay = invisible_places::water::BuildFieldStreamOverlay(cache, streamSettings);
    REQUIRE_FALSE(streamOverlay.samples.empty());
    for (const auto& sample : streamOverlay.samples) {
        const bool inCutOut = sample.position.x > 0.25F &&
                              sample.position.y > 0.25F &&
                              sample.position.y < 0.75F;
        CHECK_FALSE(inCutOut);
    }

    const auto surfaceMotion = invisible_places::water::GenerateFieldSurfaceEffectOverlay(cache, layer);
    REQUIRE_FALSE(surfaceMotion.points.empty());
    const auto strongestPoint = std::max_element(
        surfaceMotion.points.begin(),
        surfaceMotion.points.end(),
        [](const invisible_places::water::WaterEffectPoint& left,
           const invisible_places::water::WaterEffectPoint& right) {
            return left.value < right.value;
        });
    REQUIRE(strongestPoint != surfaceMotion.points.end());
    CHECK(strongestPoint->blendMode == invisible_places::water::WaterEffectBlendMode::Screen);
    CHECK(strongestPoint->emissionHint == Catch::Approx(strongestPoint->value * 0.33F));
    CHECK(strongestPoint->opacityHint == Catch::Approx(strongestPoint->value * 0.21F));
    CHECK(strongestPoint->sizeHint == Catch::Approx(strongestPoint->value * 0.07F));
    CHECK(strongestPoint->colourMixHint == Catch::Approx(strongestPoint->value * 0.44F));
    CHECK(strongestPoint->speed == Catch::Approx(1.23F));
    CHECK(static_cast<float>(strongestPoint->red) / 255.0F == Catch::Approx(0.18F).margin(1.0F / 255.0F));
    CHECK(static_cast<float>(strongestPoint->green) / 255.0F == Catch::Approx(0.72F).margin(1.0F / 255.0F));
    CHECK(static_cast<float>(strongestPoint->blue) / 255.0F == Catch::Approx(0.94F).margin(1.0F / 255.0F));
    for (const auto& point : surfaceMotion.points) {
        const bool inCutOut = point.position.x > 0.25F &&
                              point.position.y > 0.25F &&
                              point.position.y < 0.75F;
        CHECK_FALSE(inCutOut);
        const bool inNoFlowPocket = std::abs(point.position.x - 0.125F) <= 1.0e-5F &&
                                    std::abs(point.position.y - 0.5F) <= 1.0e-5F;
        CHECK_FALSE(inNoFlowPocket);
    }
}

TEST_CASE("Field streamlines split rejected gaps and fade low-confidence support", "[water][v2][field]") {
    invisible_places::water::WaterFieldCache cache;
    cache.settings.enabled = true;
    cache.settings.fieldResolutionMeters = 0.05F;
    cache.settings.maxBridgeDistanceMeters = 0.12F;
    cache.settings.bridgeAggression = 0.0F;
    cache.settings.surfaceConfidenceThreshold = 0.50F;

    auto makeNode = [](float x, float confidence) {
        invisible_places::water::WaterFieldNode node;
        node.position = {x, 0.0F, 0.0F};
        node.normal = {0.0F, 0.0F, 1.0F};
        node.vector = {1.0F, 0.0F, 0.0F};
        node.wetness = 1.0F;
        node.confidence = confidence;
        node.surfaceConfidence = confidence;
        node.pathStation = x;
        return node;
    };
    cache.nodes = {
        makeNode(0.00F, 1.0F),
        makeNode(0.10F, 1.0F),
        makeNode(1.00F, 0.25F),
        makeNode(1.05F, 0.25F),
    };

    invisible_places::water::WaterFieldStreamSettings streamSettings;
    streamSettings.streamlineCount = 8U;
    streamSettings.streamlineLengthMeters = 0.05F;
    streamSettings.stepLengthMeters = 0.025F;
    streamSettings.streamlineWidthMeters = 0.006F;
    streamSettings.streamWorldLengthMeters = 0.030F;
    streamSettings.fadeOnLowConfidence = true;

    const auto streamOverlay = invisible_places::water::BuildFieldStreamOverlay(cache, streamSettings);
    REQUIRE_FALSE(streamOverlay.samples.empty());

    bool highConfidenceSeen = false;
    bool fadedConfidenceSeen = false;
    for (const auto& sample : streamOverlay.samples) {
        const bool inRejectedGap = sample.position.x > 0.20F && sample.position.x < 0.80F;
        CHECK_FALSE(inRejectedGap);
        if (sample.position.x < 0.20F) {
            highConfidenceSeen = highConfidenceSeen || sample.streamConfidence > 0.70F;
        }
        if (sample.position.x > 0.80F) {
            fadedConfidenceSeen = fadedConfidenceSeen || sample.streamConfidence < 0.35F;
        }
    }
    CHECK(highConfidenceSeen);
    CHECK(fadedConfidenceSeen);
    CHECK(streamOverlay.fieldDiagnostics.inputNodeCount == 4U);
    CHECK(streamOverlay.fieldDiagnostics.emittedPathCount == 2U);
    CHECK(streamOverlay.fieldDiagnostics.emittedSampleCount == streamOverlay.samples.size());
    CHECK(streamOverlay.fieldDiagnostics.acceptedBridgeCount >= 1U);
    CHECK(streamOverlay.fieldDiagnostics.rejectedGapCount == 1U);
    CHECK(streamOverlay.fieldDiagnostics.lowConfidenceFadeCount >= 2U);
    CHECK(streamOverlay.fieldDiagnostics.lowConfidenceTerminationCount == 0U);
    CHECK(streamOverlay.fieldDiagnostics.maxAcceptedBridgeMeters >= 0.09F);
    CHECK(streamOverlay.fieldDiagnostics.minRejectedGapMeters >= 0.80F);

    auto allowedBridgeCache = cache;
    allowedBridgeCache.nodes = {
        makeNode(0.00F, 1.0F),
        makeNode(0.10F, 1.0F),
        makeNode(0.30F, 1.0F),
        makeNode(0.35F, 1.0F),
    };
    allowedBridgeCache.nodes[1].bridgeAllowed = true;
    allowedBridgeCache.nodes[2].bridgeAllowed = true;
    const auto allowedOverlay = invisible_places::water::BuildFieldStreamOverlay(allowedBridgeCache, streamSettings);
    CHECK(allowedOverlay.fieldDiagnostics.rejectedGapCount == 0U);
    CHECK(allowedOverlay.fieldDiagnostics.manualBridgeAllowedCount == 1U);
    CHECK(allowedOverlay.fieldDiagnostics.acceptedBridgeCount >= 1U);

    auto blockedBridgeCache = cache;
    blockedBridgeCache.nodes = {
        makeNode(0.00F, 1.0F),
        makeNode(0.05F, 1.0F),
        makeNode(0.10F, 1.0F),
    };
    blockedBridgeCache.nodes[1].bridgeBlocked = true;
    const auto blockedOverlay = invisible_places::water::BuildFieldStreamOverlay(blockedBridgeCache, streamSettings);
    CHECK(blockedOverlay.fieldDiagnostics.manualBridgeBlockedCount >= 1U);
    CHECK(blockedOverlay.fieldDiagnostics.rejectedGapCount >= 1U);

    auto noFlowCache = cache;
    noFlowCache.nodes = {
        makeNode(0.00F, 1.0F),
        makeNode(0.05F, 1.0F),
        makeNode(0.10F, 1.0F),
    };
    noFlowCache.nodes[0].flowBlocked = true;
    const auto noFlowOverlay = invisible_places::water::BuildFieldStreamOverlay(noFlowCache, streamSettings);
    CHECK(noFlowOverlay.fieldDiagnostics.manualNoFlowBlockCount == 1U);
    REQUIRE_FALSE(noFlowOverlay.samples.empty());
    for (const auto& sample : noFlowOverlay.samples) {
        CHECK(sample.position.x >= 0.04F);
    }
}

TEST_CASE("Offline ripple effect overlays render from virtual effect fields", "[output][offline][water][v2][ripples]") {
    const auto sourceCloud = MakeRippleFixtureCloud();
    const auto overlay = invisible_places::water::GenerateRippleEffectOverlay(
        sourceCloud,
        {MakeRippleTestLayer(invisible_places::water::WaterRippleOverlayType::CausticLace, 31U)});
    REQUIRE_FALSE(overlay.points.empty());

    const auto cloud = invisible_places::water::BuildWaterEffectOverlayPointCloud(
        overlay,
        "Saved/water/test-Ripples.generated",
        "ripples");
    REQUIRE(cloud.ScalarFieldCount() >= 14U);
    CHECK(cloud.scalarFields[0].name == "ripple_mask");
    CHECK(cloud.scalarFields[2].name == "ripple_value");
    CHECK(cloud.scalarFields[10].name == "ripple_emission_hint");

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian;
    style.depthContribution = invisible_places::renderer::pointcloud::PointCloudDepthContribution::None;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.040F);
    invisible_places::style::ConfigureFieldMapFromStats(
        &style.opacity,
        2,
        "ripple_value",
        0.0F,
        0.65F,
        &cloud.scalarFields[2]);
    invisible_places::style::SetFieldMapFlag(
        &style.opacity.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    invisible_places::style::ConfigureFieldMapFromStats(
        &style.emissiveStrength,
        10,
        "ripple_emission_hint",
        0.0F,
        0.45F,
        &cloud.scalarFields[10]);
    invisible_places::style::SetFieldMapFlag(
        &style.emissiveStrength.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.5F, -3.0F, 1.2F};
    cameraState.target = {0.5F, 0.5F, 0.0F};
    cameraState.fovDegrees = 45.0F;
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 128, 128);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0, 0, 128, 128},
        &image);

    float alphaSum = 0.0F;
    float beautySum = 0.0F;
    for (std::size_t index = 0; index < image.alpha.size(); ++index) {
        alphaSum += image.alpha[index];
        beautySum += image.beautyR[index] + image.beautyG[index] + image.beautyB[index];
    }
    CHECK(alphaSum > 0.0F);
    CHECK(beautySum > 0.0F);
}

TEST_CASE("Water trail looseness simplifies flat knotted lane guides", "[water]") {
    auto makeAnchors = [](const std::vector<invisible_places::io::Float3>& positions) {
        invisible_places::water::WaterOverlay overlay;
        float distance = 0.0F;
        for (std::size_t index = 0; index < positions.size(); ++index) {
            if (index > 0U) {
                const glm::vec3 previous{positions[index - 1U].x, positions[index - 1U].y, positions[index - 1U].z};
                const glm::vec3 current{positions[index].x, positions[index].y, positions[index].z};
                distance += glm::length(current - previous);
            }
            invisible_places::water::WaterOverlayPoint point;
            point.position = positions[index];
            point.normal = {0.0F, 0.0F, 1.0F};
            point.flowId = 1.0F;
            point.emitterId = 1.0F;
            point.pathDistance = distance;
            point.speed = 1.0F;
            point.width = 0.24F;
            point.confidence = 1.0F;
            overlay.bounds.Expand(point.position);
            overlay.points.push_back(point);
        }
        return overlay;
    };
    const auto guideMetrics = [](const invisible_places::water::WaterOverlay& overlay) {
        struct Metrics {
            std::size_t count = 0U;
            double length = 0.0;
        } metrics;
        for (std::size_t index = 0; index < overlay.points.size(); ++index) {
            const auto& point = overlay.points[index];
            if (point.particleRole < 2.5F || point.particleRole >= 3.5F) {
                continue;
            }
            ++metrics.count;
            if (index == 0U) {
                continue;
            }
            const auto& previous = overlay.points[index - 1U];
            if (previous.particleRole >= 2.5F &&
                previous.particleRole < 3.5F &&
                previous.pathStartIndex == point.pathStartIndex) {
                const glm::vec3 a{previous.position.x, previous.position.y, previous.position.z};
                const glm::vec3 b{point.position.x, point.position.y, point.position.z};
                metrics.length += glm::length(b - a);
            }
        }
        return metrics;
    };

    const auto anchors = makeAnchors({
        {0.0F, 0.0F, 0.0F},
        {0.8F, 0.0F, 0.0F},
        {0.8F, 0.7F, 0.0F},
        {0.08F, 0.7F, 0.0F},
        {0.08F, 0.14F, 0.0F},
        {0.88F, 0.14F, 0.0F},
        {0.88F, -0.45F, 0.0F},
    });

    invisible_places::water::WaterParticleTrailShapeSettings tightShape;
    tightShape.particleJitter = 0.45F;
    tightShape.splineAnchorSpacing = 0.08F;
    tightShape.trailLaneCount = 3U;
    tightShape.trailLooseness = 0.0F;
    invisible_places::water::WaterAnimationTrailSettings animation;
    animation.particleDensity = 0.8F;
    animation.trailSampleSpacingMeters = 0.06F;

    auto looseShape = tightShape;
    looseShape.trailLooseness = 1.0F;
    const auto tightOverlay =
        invisible_places::water::BuildWaterOverlayFromPathAnchors(anchors, tightShape, animation);
    const auto looseOverlay =
        invisible_places::water::BuildWaterOverlayFromPathAnchors(anchors, looseShape, animation);
    const auto tightMetrics = guideMetrics(tightOverlay);
    const auto looseMetrics = guideMetrics(looseOverlay);
    REQUIRE(tightMetrics.count > 0U);
    REQUIRE(looseMetrics.count > 0U);
    CHECK(looseMetrics.count < tightMetrics.count);
    CHECK(looseMetrics.length < tightMetrics.length * 0.92);
}

TEST_CASE("Water trail looseness respects projected terrain ridges", "[water]") {
    invisible_places::water::WaterOverlay anchors;
    const std::vector<invisible_places::io::Float3> path{
        {-1.0F, 0.0F, 0.0F},
        {-1.0F, 0.8F, 0.0F},
        {1.0F, 0.8F, 0.0F},
        {1.0F, 0.0F, 0.0F},
    };
    float distance = 0.0F;
    for (std::size_t index = 0; index < path.size(); ++index) {
        if (index > 0U) {
            const glm::vec3 previous{path[index - 1U].x, path[index - 1U].y, path[index - 1U].z};
            const glm::vec3 current{path[index].x, path[index].y, path[index].z};
            distance += glm::length(current - previous);
        }
        invisible_places::water::WaterOverlayPoint point;
        point.position = path[index];
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = 1.0F;
        point.emitterId = 1.0F;
        point.pathDistance = distance;
        point.speed = 1.0F;
        point.width = 0.22F;
        point.confidence = 1.0F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::io::LoadedPointCloud support;
    support.hasNormals = true;
    for (int yi = -2; yi <= 10; ++yi) {
        for (int xi = -12; xi <= 12; ++xi) {
            const float x = static_cast<float>(xi) * 0.10F;
            const float y = static_cast<float>(yi) * 0.10F;
            const float ridge = (std::abs(x) < 0.18F && y < 0.24F) ? 0.65F : 0.0F;
            support.positions.push_back({x, y, ridge});
            support.normals.push_back({0.0F, 0.0F, 1.0F});
            support.bounds.Expand(support.positions.back());
        }
    }

    invisible_places::water::WaterParticleTrailShapeSettings shape;
    shape.particleJitter = 0.25F;
    shape.splineAnchorSpacing = 0.08F;
    shape.trailLaneCount = 3U;
    shape.trailLooseness = 1.0F;
    invisible_places::water::WaterAnimationTrailSettings animation;
    animation.particleDensity = 0.8F;
    animation.trailSampleSpacingMeters = 0.06F;

    const auto overlay =
        invisible_places::water::BuildWaterOverlayFromPathAnchors(anchors, shape, animation, &support);
    const auto highGuideIt = std::find_if(
        overlay.points.begin(),
        overlay.points.end(),
        [](const invisible_places::water::WaterOverlayPoint& point) {
            return point.particleRole >= 2.5F && point.particleRole < 3.5F && point.position.z > 0.25F;
        });
    CHECK(highGuideIt == overlay.points.end());
    bool crossedRidgeShortcut = false;
    for (std::size_t index = 1U; index < overlay.points.size(); ++index) {
        const auto& previous = overlay.points[index - 1U];
        const auto& point = overlay.points[index];
        if (previous.particleRole < 2.5F || previous.particleRole >= 3.5F ||
            point.particleRole < 2.5F || point.particleRole >= 3.5F ||
            previous.pathStartIndex != point.pathStartIndex) {
            continue;
        }
        if (previous.position.y < 0.24F &&
            point.position.y < 0.24F &&
            ((previous.position.x < 0.0F && point.position.x > 0.0F) ||
             (previous.position.x > 0.0F && point.position.x < 0.0F))) {
            crossedRidgeShortcut = true;
            break;
        }
    }
    CHECK_FALSE(crossedRidgeShortcut);
}

TEST_CASE("Water trail projection stays close to flat support surface", "[water]") {
    invisible_places::water::WaterOverlay anchors;
    const std::vector<invisible_places::io::Float3> path{
        {-1.0F, -0.2F, 0.0F},
        {-0.45F, 0.08F, 0.0F},
        {0.0F, -0.05F, 0.0F},
        {0.55F, 0.12F, 0.0F},
        {1.0F, -0.2F, 0.0F},
    };
    float distance = 0.0F;
    for (std::size_t index = 0; index < path.size(); ++index) {
        if (index > 0U) {
            const glm::vec3 previous{path[index - 1U].x, path[index - 1U].y, path[index - 1U].z};
            const glm::vec3 current{path[index].x, path[index].y, path[index].z};
            distance += glm::length(current - previous);
        }
        invisible_places::water::WaterOverlayPoint point;
        point.position = path[index];
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = 1.0F;
        point.emitterId = 1.0F;
        point.pathDistance = distance;
        point.speed = 1.0F;
        point.width = 0.28F;
        point.confidence = 1.0F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::io::LoadedPointCloud support;
    support.hasNormals = true;
    for (int yi = -2; yi <= 2; ++yi) {
        for (int xi = -2; xi <= 2; ++xi) {
            support.positions.push_back({
                static_cast<float>(xi),
                static_cast<float>(yi),
                0.0F});
            support.normals.push_back({0.0F, 0.0F, 1.0F});
            support.bounds.Expand(support.positions.back());
            support.positions.push_back({
                static_cast<float>(xi),
                static_cast<float>(yi),
                0.80F});
            support.normals.push_back({0.0F, 0.0F, 1.0F});
            support.bounds.Expand(support.positions.back());
        }
    }

    invisible_places::water::WaterParticleTrailShapeSettings shape;
    shape.particleJitter = 0.65F;
    shape.splineAnchorSpacing = 0.08F;
    shape.trailLaneCount = 3U;
    shape.trailLooseness = 0.85F;
    shape.trailSmoothness = 0.75F;
    invisible_places::water::WaterAnimationTrailSettings animation;
    animation.particleDensity = 1.0F;
    animation.trailLengthMeters = 0.18F;
    animation.trailSampleSpacingMeters = 0.05F;

    const auto overlay =
        invisible_places::water::BuildWaterOverlayFromPathAnchors(anchors, shape, animation, &support);
    std::size_t visibleTrailCount = 0U;
    float maxTrailHeight = 0.0F;
    float maxGuideHeightStep = 0.0F;
    for (std::size_t index = 0; index < overlay.points.size(); ++index) {
        const auto& point = overlay.points[index];
        if (point.particleRole >= 0.5F) {
            maxTrailHeight = std::max(maxTrailHeight, std::abs(point.position.z));
            ++visibleTrailCount;
        }
        if (index == 0U || point.particleRole < 2.5F || point.particleRole >= 3.5F) {
            continue;
        }
        const auto& previous = overlay.points[index - 1U];
        if (previous.particleRole >= 2.5F &&
            previous.particleRole < 3.5F &&
            previous.pathStartIndex == point.pathStartIndex) {
            maxGuideHeightStep = std::max(maxGuideHeightStep, std::abs(point.position.z - previous.position.z));
        }
    }
    REQUIRE(visibleTrailCount > 0U);
    CHECK(maxTrailHeight < 0.025F);
    CHECK(maxGuideHeightStep < 0.010F);
}

TEST_CASE("Water trail surface index is reusable for preview and final builds", "[water]") {
    invisible_places::water::WaterOverlay anchors;
    const std::vector<invisible_places::io::Float3> path{
        {-0.8F, 0.0F, 0.0F},
        {-0.4F, 0.32F, 0.0F},
        {0.0F, 0.20F, 0.0F},
        {0.4F, 0.44F, 0.0F},
        {0.8F, 0.0F, 0.0F},
    };
    float distance = 0.0F;
    for (std::size_t index = 0; index < path.size(); ++index) {
        if (index > 0U) {
            const glm::vec3 previous{path[index - 1U].x, path[index - 1U].y, path[index - 1U].z};
            const glm::vec3 current{path[index].x, path[index].y, path[index].z};
            distance += glm::length(current - previous);
        }
        invisible_places::water::WaterOverlayPoint point;
        point.position = path[index];
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = 11.0F;
        point.emitterId = 1.0F;
        point.pathDistance = distance;
        point.speed = 1.0F;
        point.width = 0.20F;
        point.confidence = 1.0F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::io::LoadedPointCloud support;
    support.hasNormals = true;
    for (int yi = -8; yi <= 8; ++yi) {
        for (int xi = -12; xi <= 12; ++xi) {
            const float x = static_cast<float>(xi) * 0.08F;
            const float y = static_cast<float>(yi) * 0.08F;
            const float z = std::sin(x * 2.0F) * 0.015F + std::cos(y * 1.7F) * 0.010F;
            support.positions.push_back({x, y, z});
            support.normals.push_back({0.0F, 0.0F, 1.0F});
            support.bounds.Expand(support.positions.back());
        }
    }

    const auto surfaceIndex = invisible_places::water::BuildTrailSurfaceIndex(support);
    REQUIRE(surfaceIndex != nullptr);
    REQUIRE(invisible_places::water::TrailSurfaceIndexSampleCount(*surfaceIndex) > 0U);

    invisible_places::water::WaterParticleTrailShapeSettings shape;
    shape.particleJitter = 0.32F;
    shape.splineAnchorSpacing = 0.06F;
    shape.trailLaneCount = 4U;
    shape.trailLooseness = 0.75F;
    shape.trailSmoothness = 0.85F;
    invisible_places::water::WaterAnimationTrailSettings animation;
    animation.particleDensity = 1.2F;
    animation.trailLengthMeters = 0.18F;
    animation.trailSampleSpacingMeters = 0.04F;

    invisible_places::water::WaterTrailBuildDiagnostics previewDiagnostics;
    const auto previewOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        anchors,
        shape,
        animation,
        surfaceIndex.get(),
        invisible_places::water::WaterTrailBuildQuality::Preview,
        &previewDiagnostics);
    invisible_places::water::WaterTrailBuildDiagnostics secondPreviewDiagnostics;
    const auto secondPreviewOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        anchors,
        shape,
        animation,
        surfaceIndex.get(),
        invisible_places::water::WaterTrailBuildQuality::Preview,
        &secondPreviewDiagnostics);
    invisible_places::water::WaterTrailBuildDiagnostics finalDiagnostics;
    const auto finalOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        anchors,
        shape,
        animation,
        surfaceIndex.get(),
        invisible_places::water::WaterTrailBuildQuality::Final,
        &finalDiagnostics);

    auto guideStats = [](const invisible_places::water::WaterOverlay& overlay) {
        struct Stats {
            std::size_t count = 0;
            std::uint32_t maxLane = 0;
            float length = 0.0F;
        } stats;
        for (std::size_t index = 0; index < overlay.points.size(); ++index) {
            const auto& point = overlay.points[index];
            if (point.particleRole < 2.5F || point.particleRole >= 3.5F) {
                continue;
            }
            ++stats.count;
            stats.maxLane = std::max(
                stats.maxLane,
                static_cast<std::uint32_t>(std::max(0.0F, std::floor(point.trailLaneId + 0.5F))));
            if (index > 0U) {
                const auto& previous = overlay.points[index - 1U];
                if (previous.pathStartIndex == point.pathStartIndex &&
                    previous.particleRole >= 2.5F &&
                    previous.particleRole < 3.5F) {
                    const glm::vec3 a{previous.position.x, previous.position.y, previous.position.z};
                    const glm::vec3 b{point.position.x, point.position.y, point.position.z};
                    stats.length += glm::length(b - a);
                }
            }
        }
        return stats;
    };

    const auto previewStats = guideStats(previewOverlay);
    const auto secondPreviewStats = guideStats(secondPreviewOverlay);
    const auto finalStats = guideStats(finalOverlay);
    REQUIRE(previewStats.count > 0U);
    CHECK(secondPreviewStats.count == previewStats.count);
    CHECK(previewStats.maxLane == shape.trailLaneCount - 1U);
    CHECK(finalStats.maxLane == shape.trailLaneCount - 1U);
    CHECK(finalStats.count >= previewStats.count);
    CHECK(finalStats.length > previewStats.length * 0.70F);
    CHECK(finalStats.length < previewStats.length * 1.45F);
    CHECK(previewDiagnostics.surfaceIndexBuildMs == Catch::Approx(0.0));
    CHECK(secondPreviewDiagnostics.surfaceIndexBuildMs == Catch::Approx(0.0));
    CHECK(secondPreviewDiagnostics.surfaceSampleCount ==
          invisible_places::water::TrailSurfaceIndexSampleCount(*surfaceIndex));
    CHECK(secondPreviewDiagnostics.routedPathCount == previewDiagnostics.routedPathCount);
}

TEST_CASE("Water path smoothing changes baked anchor geometry", "[water]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-zigzag.ply";
    cloud.layerName = "synthetic-water-zigzag";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    for (int index = 0; index < 10; ++index) {
        const float x = (index % 2 == 0) ? -0.04F : 0.04F;
        const invisible_places::io::Float3 position{
            x,
            0.0F,
            1.0F - static_cast<float>(index) * 0.08F};
        cloud.positions.push_back(position);
        cloud.normals.push_back({1.0F, 0.0F, 0.0F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    }
    cloud.focusPoint = cloud.positions.front();
    cloud.hasFocusPoint = true;

    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Aerial);
    settings.supportVoxelSize = 0.02F;
    settings.maxBridgeDistance = 0.13F;
    settings.pathLength = 0.8F;
    settings.pathSampleSpacing = 0.03F;
    settings.maxSteps = 32;
    settings.supportSampleLimit = 64;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 11;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.06F;
    emitter.confidence = 1.0F;

    auto rawSettings = settings;
    rawSettings.smoothing = 0.0F;
    auto smoothSettings = settings;
    smoothSettings.smoothing = 1.0F;

    const auto rawAnchors = invisible_places::water::GenerateWaterPathAnchors(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        rawSettings);
    const auto smoothAnchors = invisible_places::water::GenerateWaterPathAnchors(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        smoothSettings);
    REQUIRE(rawAnchors.points.size() > 6U);
    REQUIRE(smoothAnchors.points.size() == rawAnchors.points.size());

    const auto meanInteriorAbsX = [](const invisible_places::water::WaterOverlay& overlay) {
        float sum = 0.0F;
        std::size_t count = 0;
        for (std::size_t index = 1U; index + 1U < overlay.points.size(); ++index) {
            if (overlay.points[index].particleRole < 0.5F) {
                sum += std::abs(overlay.points[index].position.x);
                ++count;
            }
        }
        return count == 0U ? 0.0F : sum / static_cast<float>(count);
    };

    CHECK(meanInteriorAbsX(smoothAnchors) < meanInteriorAbsX(rawAnchors) * 0.85F);
    CHECK(smoothAnchors.points.front().pathDistance == Catch::Approx(0.0F));
    CHECK(smoothAnchors.points.back().pathDistance > smoothAnchors.points.front().pathDistance);
}

TEST_CASE("Water path smoothing refreshes cached branch anchors without rebaking", "[water]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-cache-zigzag.ply";
    cloud.layerName = "synthetic-water-cache-zigzag";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    for (int index = 0; index < 10; ++index) {
        const float x = (index % 2 == 0) ? -0.04F : 0.04F;
        const invisible_places::io::Float3 position{
            x,
            0.0F,
            1.0F - static_cast<float>(index) * 0.08F};
        cloud.positions.push_back(position);
        cloud.normals.push_back({1.0F, 0.0F, 0.0F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    }

    auto sourceSettings = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Detail);
    sourceSettings.path.autoTune = false;
    sourceSettings.path.supportVoxelSize = 0.02F;
    sourceSettings.path.maxBridgeDistance = 0.13F;
    sourceSettings.path.pathLength = 0.8F;
    sourceSettings.path.pathSampleSpacing = 0.03F;
    sourceSettings.path.maxSteps = 32;
    sourceSettings.path.supportSampleLimit = 64;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 29;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.04F;
    emitter.confidence = 1.0F;

    const auto cache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        sourceSettings);
    REQUIRE_FALSE(cache.branches.empty());

    auto rawSourceSettings = sourceSettings;
    rawSourceSettings.path.smoothing = 0.0F;
    auto smoothSourceSettings = sourceSettings;
    smoothSourceSettings.path.smoothing = 1.0F;

    const auto rawAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        rawSourceSettings);
    const auto smoothAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        smoothSourceSettings);
    REQUIRE(rawAnchors.points.size() > 8U);
    REQUIRE(smoothAnchors.points.size() == rawAnchors.points.size());

    const auto meanInteriorAbsX = [](const invisible_places::water::WaterOverlay& overlay) {
        float sum = 0.0F;
        std::size_t count = 0;
        for (std::size_t index = 1U; index + 1U < overlay.points.size(); ++index) {
            sum += std::abs(overlay.points[index].position.x);
            ++count;
        }
        return count == 0U ? 0.0F : sum / static_cast<float>(count);
    };

    CHECK(meanInteriorAbsX(smoothAnchors) < meanInteriorAbsX(rawAnchors) * 0.85F);
    CHECK(invisible_places::water::WaterSourceBakeInputsEqual(rawSourceSettings, smoothSourceSettings));
}

TEST_CASE("Water path bake inputs ignore refresh-only trail and smoothing settings", "[water]") {
    auto source = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Detail);
    source.path.autoTune = false;
    source.path.supportVoxelSize = 0.02F;
    source.path.maxBridgeDistance = 0.10F;
    source.path.pathLength = 0.55F;
    source.path.pathSampleSpacing = 0.025F;
    source.path.maxSteps = 32;
    source.path.supportSampleLimit = 128;
    auto refreshOnly = source;
    refreshOnly.path.smoothing = std::clamp(source.path.smoothing + 0.25F, 0.0F, 1.0F);
    refreshOnly.trailShape.particleJitter += 0.42F;
    refreshOnly.trailShape.splineAnchorSpacing *= 1.7F;
    refreshOnly.trailShape.trailLaneCount += 4U;
    refreshOnly.trailShape.trailLooseness = std::clamp(source.trailShape.trailLooseness + 0.35F, 0.0F, 1.0F);
    refreshOnly.trailShape.trailSmoothness = std::clamp(source.trailShape.trailSmoothness + 0.28F, 0.0F, 1.0F);
    refreshOnly.trailShape.trailTurbulence += 0.35F;
    refreshOnly.trailShape.trailMomentum = std::clamp(source.trailShape.trailMomentum + 0.25F, 0.0F, 0.98F);
    refreshOnly.trailShape.normalTurbulenceResponse += 0.55F;
    CHECK(invisible_places::water::WaterSourceBakeInputsEqual(source, refreshOnly));

    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-trail-refresh.ply";
    cloud.layerName = "synthetic-water-trail-refresh";
    cloud.hasNormals = true;
    for (int index = 0; index < 16; ++index) {
        const invisible_places::io::Float3 position{
            static_cast<float>(index % 3) * 0.01F,
            0.0F,
            1.0F - static_cast<float>(index) * 0.045F};
        cloud.positions.push_back(position);
        cloud.normals.push_back({0.7F, 0.0F, 0.71F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    }
    invisible_places::water::WaterEmitter emitter;
    emitter.id = 31;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.04F;
    emitter.confidence = 1.0F;

    const auto sourceCache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        source);
    const auto refreshOnlyCache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        refreshOnly);
    REQUIRE_FALSE(sourceCache.branches.empty());
    REQUIRE(sourceCache.branches.size() == refreshOnlyCache.branches.size());
    CHECK(sourceCache.requestedSettings.pathLength == Catch::Approx(refreshOnlyCache.requestedSettings.pathLength));
    CHECK(sourceCache.tunedSettings.maxBridgeDistance ==
          Catch::Approx(refreshOnlyCache.tunedSettings.maxBridgeDistance));
    for (std::size_t branchIndex = 0; branchIndex < sourceCache.branches.size(); ++branchIndex) {
        const auto& leftBranch = sourceCache.branches[branchIndex];
        const auto& rightBranch = refreshOnlyCache.branches[branchIndex];
        CHECK(leftBranch.id == rightBranch.id);
        CHECK(leftBranch.emitterId == rightBranch.emitterId);
        REQUIRE(leftBranch.rawAnchors.size() == rightBranch.rawAnchors.size());
        for (std::size_t anchorIndex = 0; anchorIndex < leftBranch.rawAnchors.size(); ++anchorIndex) {
            const auto& leftAnchor = leftBranch.rawAnchors[anchorIndex];
            const auto& rightAnchor = rightBranch.rawAnchors[anchorIndex];
            CHECK(leftAnchor.position.x == Catch::Approx(rightAnchor.position.x));
            CHECK(leftAnchor.position.y == Catch::Approx(rightAnchor.position.y));
            CHECK(leftAnchor.position.z == Catch::Approx(rightAnchor.position.z));
            CHECK(leftAnchor.pathDistance == Catch::Approx(rightAnchor.pathDistance));
            CHECK(leftAnchor.surfaceSteepness == Catch::Approx(rightAnchor.surfaceSteepness));
        }
    }

    auto bakeChanging = source;
    bakeChanging.path.pathLength += 0.35F;
    CHECK_FALSE(invisible_places::water::WaterSourceBakeInputsEqual(source, bakeChanging));

    bakeChanging = source;
    bakeChanging.path.branching = std::clamp(source.path.branching + 0.2F, 0.0F, 1.0F);
    CHECK_FALSE(invisible_places::water::WaterSourceBakeInputsEqual(source, bakeChanging));
}

TEST_CASE("Water gap tolerance controls how much bridge upper limit is used", "[water]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-gap-tolerance.ply";
    cloud.layerName = "synthetic-water-gap-tolerance";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    const auto appendPoint = [&](float y, float z) {
        const invisible_places::io::Float3 position{0.0F, y, z};
        cloud.positions.push_back(position);
        cloud.normals.push_back({0.0F, 0.45F, 0.89F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    };
    for (int index = 0; index < 4; ++index) {
        appendPoint(static_cast<float>(index) * 0.006F, 1.0F - static_cast<float>(index) * 0.004F);
    }
    for (int index = 0; index < 6; ++index) {
        appendPoint(0.055F + static_cast<float>(index) * 0.006F, 0.982F - static_cast<float>(index) * 0.004F);
    }

    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Detail);
    settings.autoTune = true;
    settings.supportVoxelSize = 0.006F;
    settings.maxBridgeDistance = 0.060F;
    settings.pathLength = 0.14F;
    settings.pathSampleSpacing = 0.006F;
    settings.maxSteps = 64;
    settings.supportSampleLimit = 128;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 31;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.025F;
    emitter.confidence = 1.0F;

    auto lowTolerance = settings;
    lowTolerance.gapTolerance = 0.0F;
    const auto lowCache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        lowTolerance);

    auto highTolerance = settings;
    highTolerance.gapTolerance = 1.0F;
    const auto highCache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        highTolerance);

    REQUIRE_FALSE(lowCache.branches.empty());
    REQUIRE_FALSE(highCache.branches.empty());
    CHECK(lowCache.tunedSettings.maxBridgeDistance < 0.025F);
    CHECK(highCache.tunedSettings.maxBridgeDistance > 0.050F);
    CHECK(highCache.branches.front().length > lowCache.branches.front().length + 0.025F);
    CHECK(highCache.branches.front().gapCount > lowCache.branches.front().gapCount);
}

TEST_CASE("Site3 terrestrial sample water sources produce cached paths", "[water][sample][.]") {
    const auto samplePath = DataRoot() / "Site3-Sample-Terrestrial.ply";
    if (!std::filesystem::exists(samplePath)) {
        return;
    }

    const auto loadResult = invisible_places::io::LoadPointCloud(samplePath);
    REQUIRE(loadResult.success);
    REQUIRE(loadResult.cloud.PointCount() > 0U);

    auto sourceSettings = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Detail);
    sourceSettings.path.pathLength = 4.0F;
    sourceSettings.path.maxBridgeDistance = 0.075F;
    sourceSettings.path.gapTolerance = 0.85F;
    sourceSettings.path.supportSampleLimit = 900000;

    std::vector<invisible_places::water::WaterEmitter> emitters;
    invisible_places::water::WaterEmitter canopyEdge;
    canopyEdge.id = 101;
    canopyEdge.name = "Site3 top edge";
    canopyEdge.position = {307.199F, 100.993F, 2.088F};
    canopyEdge.radius = 0.035F;
    canopyEdge.confidence = 1.0F;
    emitters.push_back(canopyEdge);

    invisible_places::water::WaterEmitter rockEdge;
    rockEdge.id = 102;
    rockEdge.name = "Site3 rock edge";
    rockEdge.position = {307.641F, 102.531F, 1.889F};
    rockEdge.radius = 0.035F;
    rockEdge.confidence = 1.0F;
    emitters.push_back(rockEdge);

    const auto cache = invisible_places::water::GenerateWaterPathCache(
        loadResult.cloud,
        emitters,
        sourceSettings);
    REQUIRE_FALSE(cache.branches.empty());
    CHECK(cache.tunedSettings.maxBridgeDistance <= sourceSettings.path.maxBridgeDistance);
    CHECK(cache.diagnostics.estimatedPointSpacing > 0.0F);
    CHECK(cache.diagnostics.pathSampleSpacing <= 0.008F);

    const auto anchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
        cache,
        emitters,
        sourceSettings);
    CHECK_FALSE(anchors.points.empty());
}

TEST_CASE("Water path cache branches across flat fan support", "[water]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-flat-fan.ply";
    cloud.layerName = "synthetic-water-flat-fan";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    for (int y = 0; y <= 7; ++y) {
        const invisible_places::io::Float3 position{
            0.0F,
            static_cast<float>(y) * 0.025F,
            1.0F - static_cast<float>(y) * 0.018F};
        cloud.positions.push_back(position);
        cloud.normals.push_back({1.0F, 0.0F, 0.0F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    }
    for (int y = 8; y <= 14; ++y) {
        for (int x = -3; x <= 3; ++x) {
            const invisible_places::io::Float3 position{
                static_cast<float>(x) * 0.022F,
                static_cast<float>(y) * 0.025F,
                0.86F + static_cast<float>((x + 3) % 2) * 0.001F};
            cloud.positions.push_back(position);
            cloud.normals.push_back({0.0F, 0.0F, 1.0F});
            cloud.packedColors.push_back(0xFFFFFFFFU);
            cloud.bounds.Expand(position);
        }
    }

    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Detail);
    settings.autoTune = false;
    settings.supportVoxelSize = 0.018F;
    settings.maxBridgeDistance = 0.065F;
    settings.pathLength = 0.45F;
    settings.pathSampleSpacing = 0.018F;
    settings.branching = 1.0F;
    settings.coverage = 1.0F;
    settings.gapTolerance = 0.8F;
    settings.maxSteps = 96;
    settings.supportSampleLimit = 4096;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 17;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.035F;
    emitter.confidence = 1.0F;

    const auto cache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings);
    REQUIRE(cache.branches.size() >= 3U);
    CHECK(cache.diagnostics.branchCount == cache.branches.size());
    CHECK(cache.diagnostics.averageConfidence > 0.0F);
    CHECK(std::any_of(cache.branches.begin(), cache.branches.end(), [](const auto& branch) {
        return branch.role == invisible_places::water::WaterPathBranchRole::Main;
    }));
    CHECK(std::any_of(cache.branches.begin(), cache.branches.end(), [](const auto& branch) {
        return branch.role == invisible_places::water::WaterPathBranchRole::Spread ||
               branch.role == invisible_places::water::WaterPathBranchRole::Secondary;
    }));

    invisible_places::water::WaterSourceSettings sourceSettings;
    sourceSettings.path = settings;
    const auto visibleAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        sourceSettings);
    REQUIRE_FALSE(visibleAnchors.points.empty());

    auto hiddenCache = cache;
    hiddenCache.hiddenBranchIds.push_back(cache.branches.front().id);
    const auto hiddenAnchors = invisible_places::water::BuildWaterPathAnchorsFromCache(
        hiddenCache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        sourceSettings);
    CHECK(hiddenAnchors.points.size() < visibleAnchors.points.size());
}

TEST_CASE("Water path cache tags bridge gaps and round-trips hidden branches", "[water][serialization]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-gap.ply";
    cloud.layerName = "synthetic-water-gap";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    const std::array<float, 7> gapHeights{1.0F, 0.972F, 0.944F, 0.820F, 0.792F, 0.764F, 0.736F};
    for (std::size_t index = 0; index < gapHeights.size(); ++index) {
        const float z = gapHeights[index];
        const invisible_places::io::Float3 position{0.0F, 0.0F, z};
        cloud.positions.push_back(position);
        cloud.normals.push_back({1.0F, 0.0F, 0.0F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    }

    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Detail);
    settings.autoTune = false;
    settings.supportVoxelSize = 0.015F;
    settings.maxBridgeDistance = 0.12F;
    settings.pathLength = 0.45F;
    settings.pathSampleSpacing = 0.015F;
    settings.maxSteps = 32;
    settings.supportSampleLimit = 128;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 5;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.03F;

    auto cache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings);
    CHECK(cache.schemaVersion == 2U);
    REQUIRE_FALSE(cache.branches.empty());
    CHECK(std::any_of(cache.branches.begin(), cache.branches.end(), [](const auto& branch) {
        return branch.gapCount > 0U || branch.confidence < 0.95F;
    }));
    cache.supportLayerPath = cloud.sourcePath;
    cache.supportSignature = "points=7";
    cache.emitterSettingsFingerprint = "emitter=5";
    cache.hiddenBranchIds.push_back(cache.branches.front().id);

    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_water_path_cache_test.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveWaterPathCacheDocument(cache, outputPath, &errorMessage));
    const auto loaded = invisible_places::serialization::LoadWaterPathCacheDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == 2U);
    CHECK(loaded->supportLayerPath == cache.supportLayerPath);
    CHECK(loaded->supportSignature == cache.supportSignature);
    CHECK(loaded->emitterSettingsFingerprint == cache.emitterSettingsFingerprint);
    REQUIRE(loaded->branches.size() == cache.branches.size());
    CHECK(loaded->hiddenBranchIds == cache.hiddenBranchIds);
    CHECK(loaded->branches.front().rawAnchors.size() == cache.branches.front().rawAnchors.size());
    std::filesystem::remove(outputPath);
}

TEST_CASE("Water emitter suggestions stay conservative and editable", "[water]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.hasNormals = true;
    for (int index = 0; index < 32; ++index) {
        const invisible_places::io::Float3 position{
            0.0F,
            static_cast<float>(index % 2) * 0.025F,
            2.0F - static_cast<float>(index) * 0.035F};
        cloud.positions.push_back(position);
        cloud.normals.push_back({1.0F, 0.0F, 0.0F});
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    }

    auto settings = invisible_places::water::DefaultWaterBakeSettings(
        invisible_places::water::WaterScaleMode::Mid);
    settings.supportVoxelSize = 0.04F;
    settings.maxBridgeDistance = 0.12F;
    settings.supportSampleLimit = 512;

    const auto suggestions = invisible_places::water::SuggestWaterEmitters(
        cloud,
        {},
        settings,
        10,
        3);
    REQUIRE(!suggestions.empty());
    CHECK(suggestions.size() <= 3);
    CHECK(suggestions.front().id == 10U);
    CHECK(suggestions.front().origin == invisible_places::water::WaterEmitterOrigin::AutoSuggested);
    CHECK(suggestions.front().status == invisible_places::water::WaterEmitterStatus::Candidate);
    CHECK(suggestions.front().confidence >= 0.62F);
}

TEST_CASE("Water source documents round-trip independently from projects", "[water][serialization]") {
    invisible_places::serialization::WaterSourcesDocument document;
    document.sourceSettings = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Aerial);
    document.sourceSettings.path.maxBridgeDistance = 6.5F;
    document.sourceSettings.trailShape.particleJitter = 0.72F;
    document.sourceSettings.trailShape.splineAnchorSpacing = 1.25F;
    document.sourceSettings.trailShape.trailLaneCount = 5U;
    document.sourceSettings.trailShape.trailTurbulence = 0.66F;
    document.sourceSettings.trailShape.trailMomentum = 0.72F;
    document.sourceSettings.trailShape.normalTurbulenceResponse = 1.25F;
    document.sourceSettings.trailShape.trailLooseness = 0.61F;
    document.sourceSettings.trailShape.trailSmoothness = 0.82F;
    document.tempSourceSettings = document.sourceSettings;
    document.tempSourceSettings->trailShape.particleJitter = 1.05F;
    document.tempSourceSettings->trailShape.trailLaneCount = 8U;
    document.settings.path = document.sourceSettings.path;
    document.settings.trail.particleJitter = document.sourceSettings.trailShape.particleJitter;
    document.settings.trail.splineAnchorSpacing = document.sourceSettings.trailShape.splineAnchorSpacing;
    document.bakeSettings = document.sourceSettings.path;
    document.renderSettings = document.settings;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 42;
    emitter.name = "cliff seep";
    emitter.position = {10.0F, 11.0F, 12.0F};
    emitter.scope = invisible_places::water::WaterScaleMode::Aerial;
    emitter.origin = invisible_places::water::WaterEmitterOrigin::AutoSuggested;
    emitter.status = invisible_places::water::WaterEmitterStatus::Candidate;
    emitter.parentId = 7U;
    emitter.sourceSettingsAssignment = invisible_places::water::WaterSourceSettingsAssignment::Custom;
    emitter.sourceSettings = document.sourceSettings;
    emitter.sourceSettings->trailShape.splineAnchorSpacing = 0.75F;
    emitter.sourceSettings->trailShape.trailTurbulence = 1.40F;
    emitter.tempSourceSettings = emitter.sourceSettings;
    emitter.tempSourceSettings->trailShape.particleJitter = 1.2F;
    emitter.tempSourceSettings->trailShape.trailMomentum = 0.31F;
    document.emitters.push_back(emitter);

    invisible_places::water::WaterEmitter linkedEmitter;
    linkedEmitter.id = 43;
    linkedEmitter.name = "linked seep";
    linkedEmitter.position = {20.0F, 21.0F, 22.0F};
    linkedEmitter.sourceSettingsAssignment = invisible_places::water::WaterSourceSettingsAssignment::LinkedEmitter;
    linkedEmitter.linkedSourceSettingsEmitterId = emitter.id;
    document.emitters.push_back(linkedEmitter);

    invisible_places::water::WaterEmitter defaultEmitter;
    defaultEmitter.id = 44;
    defaultEmitter.name = "default seep";
    defaultEmitter.position = {30.0F, 31.0F, 32.0F};
    document.emitters.push_back(defaultEmitter);
    invisible_places::water::WaterPathCache sourcePathCache;
    sourcePathCache.supportLayerPath = "Data/Site2 -5mm.ply";
    sourcePathCache.supportSignature = "Data/Site2 -5mm.ply|points=4096";
    sourcePathCache.emitterSettingsFingerprint = "source-fingerprint";
    sourcePathCache.requestedSettings = document.sourceSettings.path;
    sourcePathCache.tunedSettings = document.sourceSettings.path;
    invisible_places::water::WaterPathBranch sourceBranch;
    sourceBranch.id = 77U;
    sourceBranch.emitterId = emitter.id;
    sourceBranch.role = invisible_places::water::WaterPathBranchRole::Main;
    sourceBranch.length = 1.75F;
    sourceBranch.rawAnchors.push_back({
        .position = emitter.position,
        .normal = {0.0F, 0.0F, 1.0F},
        .emitterId = static_cast<float>(emitter.id),
        .pathDistance = 0.0F,
    });
    sourceBranch.rawAnchors.push_back({
        .position = {10.5F, 11.1F, 11.6F},
        .normal = {0.0F, 0.0F, 1.0F},
        .emitterId = static_cast<float>(emitter.id),
        .pathDistance = 1.75F,
    });
    sourcePathCache.branches.push_back(sourceBranch);
    document.pathCache = sourcePathCache;
    document.causticLookSettings = invisible_places::water::DefaultWaterCausticLookSettings();
    document.causticLookSettings.enabled = true;
    document.causticLookSettings.intensity = 1.4F;
    document.causticLookSettings.opacityBoost = 0.22F;
    document.causticLookSettings.cellSizeMeters = 0.16F;
    document.causticLookSettings.lineWidthMeters = 0.010F;
    document.causticLookSettings.featherMeters = 0.003F;
    document.causticLookSettings.surfacePointSpacingMeters = 0.002F;
    document.causticLookSettings.warpAmplitudeMeters = 0.025F;
    document.tempCausticLookSettings = document.causticLookSettings;
    document.tempCausticLookSettings->speed = 1.3F;
    document.tempCausticLookSettings->lineWidthMeters = 0.018F;
    const std::vector<invisible_places::io::Float3> sourceCausticLaceVertices{
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F}};
    invisible_places::water::WaterEffectLayer rippleLayer;
    rippleLayer.id = 9;
    rippleLayer.name = "sandstone caustic lace";
    rippleLayer.featureType = invisible_places::water::WaterEffectFeatureType::Ripple;
    rippleLayer.rippleOverlayType = invisible_places::water::WaterRippleOverlayType::CausticLace;
    rippleLayer.targetLayerSourcePath = "Data/Site2 -5mm.ply";
    rippleLayer.vertices = sourceCausticLaceVertices;
    rippleLayer.hull = invisible_places::water::BuildWaterRegionHull(rippleLayer.vertices);
    rippleLayer.edgeBlendWidth = 0.44F;
    rippleLayer.wavelengthMeters = 0.16F;
    document.rippleLayers.push_back(rippleLayer);
    invisible_places::water::WaterEffectLayer fieldLayer;
    fieldLayer.id = 10;
    fieldLayer.name = "sandstone field";
    fieldLayer.featureType = invisible_places::water::WaterEffectFeatureType::FieldSurfaceMotion;
    fieldLayer.targetLayerSourcePath = "Data/Site2 -5mm.ply";
    fieldLayer.vertices = sourceCausticLaceVertices;
    fieldLayer.hull = invisible_places::water::BuildWaterRegionHull(fieldLayer.vertices);
    fieldLayer.edgeBlendWidth = 0.33F;
    fieldLayer.regionStrength = 0.77F;
    document.fieldLayers.push_back(fieldLayer);
    document.flowStreamSettings.streamCountTotal = 222U;
    document.flowStreamSettings.streamWorldLengthMeters = 0.052F;
    document.fieldSettings.corridorRadiusMeters = 0.38F;
    document.fieldStreamSettings.streamlineCount = 333U;
    document.fieldStreamSettings.streamlineLengthMeters = 0.94F;

    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_water_sources.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveWaterSourcesDocument(document, outputPath, &errorMessage));
    {
        std::ifstream savedSources{outputPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedSources},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find("\"schema_version\": 6") != std::string::npos);
        CHECK(savedJson.find("\"water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_lane_count\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_smoothness\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_turbulence\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_momentum\"") != std::string::npos);
        CHECK(savedJson.find("\"normal_turbulence_response\"") != std::string::npos);
        CHECK(savedJson.find("\"settings_assignment\"") != std::string::npos);
        CHECK(savedJson.find("\"linked_settings_emitter_id\"") != std::string::npos);
        CHECK(savedJson.find("\"water_basin_regions\"") == std::string::npos);
        CHECK(savedJson.find("\"water_runoff_regions\"") == std::string::npos);
        CHECK(savedJson.find("\"water_caustic_regions\"") == std::string::npos);
        CHECK(savedJson.find("\"water_ripple_layers\"") != std::string::npos);
        CHECK(savedJson.find("\"water_field_layers\"") != std::string::npos);
        CHECK(savedJson.find("\"field_surface_motion\"") != std::string::npos);
        CHECK(savedJson.find("\"water_flow_stream_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_field_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_field_stream_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_caustic_look_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_caustic_look_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_path_cache\"") != std::string::npos);
        CHECK(savedJson.find("\"preview_tint_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"water_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"water_bake_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"water_render_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"scale_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"scope\"") == std::string::npos);
    }
    const auto loaded = invisible_places::serialization::LoadWaterSourcesDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->sourceSettings.path.maxBridgeDistance == Catch::Approx(6.5F));
    CHECK(loaded->sourceSettings.trailShape.particleJitter == Catch::Approx(0.72F));
    CHECK(loaded->sourceSettings.trailShape.splineAnchorSpacing == Catch::Approx(1.25F));
    CHECK(loaded->sourceSettings.trailShape.trailLaneCount == 5U);
    CHECK(loaded->sourceSettings.trailShape.trailTurbulence == Catch::Approx(0.66F));
    CHECK(loaded->sourceSettings.trailShape.trailMomentum == Catch::Approx(0.72F));
    CHECK(loaded->sourceSettings.trailShape.normalTurbulenceResponse == Catch::Approx(1.25F));
    CHECK(loaded->sourceSettings.trailShape.trailLooseness == Catch::Approx(0.61F));
    CHECK(loaded->sourceSettings.trailShape.trailSmoothness == Catch::Approx(0.82F));
    CHECK(loaded->settings.path.maxBridgeDistance == Catch::Approx(6.5F));
    CHECK(loaded->settings.trail.particleJitter == Catch::Approx(0.72F));
    REQUIRE(loaded->tempSourceSettings.has_value());
    CHECK(loaded->tempSourceSettings->trailShape.particleJitter == Catch::Approx(1.05F));
    CHECK(loaded->tempSourceSettings->trailShape.trailLaneCount == 8U);
    CHECK(loaded->bakeSettings.maxBridgeDistance == Catch::Approx(6.5F));
    CHECK(loaded->renderSettings.trail.splineAnchorSpacing == Catch::Approx(1.25F));
    REQUIRE(loaded->emitters.size() == 3);
    CHECK(loaded->emitters[0].id == 42U);
    CHECK(loaded->emitters[0].origin == invisible_places::water::WaterEmitterOrigin::AutoSuggested);
    CHECK(
        loaded->emitters[0].sourceSettingsAssignment ==
        invisible_places::water::WaterSourceSettingsAssignment::Custom);
    REQUIRE(loaded->emitters[0].parentId.has_value());
    CHECK(loaded->emitters[0].parentId.value() == 7U);
    REQUIRE(loaded->emitters[0].sourceSettings.has_value());
    CHECK(loaded->emitters[0].sourceSettings->trailShape.splineAnchorSpacing == Catch::Approx(0.75F));
    CHECK(loaded->emitters[0].sourceSettings->trailShape.trailTurbulence == Catch::Approx(1.40F));
    REQUIRE(loaded->emitters[0].tempSourceSettings.has_value());
    CHECK(loaded->emitters[0].tempSourceSettings->trailShape.particleJitter == Catch::Approx(1.2F));
    CHECK(loaded->emitters[0].tempSourceSettings->trailShape.trailMomentum == Catch::Approx(0.31F));
    CHECK(
        loaded->emitters[1].sourceSettingsAssignment ==
        invisible_places::water::WaterSourceSettingsAssignment::LinkedEmitter);
    REQUIRE(loaded->emitters[1].linkedSourceSettingsEmitterId.has_value());
    CHECK(loaded->emitters[1].linkedSourceSettingsEmitterId.value() == 42U);
    CHECK(
        loaded->emitters[2].sourceSettingsAssignment ==
        invisible_places::water::WaterSourceSettingsAssignment::Default);
    CHECK_FALSE(loaded->emitters[2].sourceSettings.has_value());
    REQUIRE(loaded->rippleLayers.size() == 1U);
    CHECK(loaded->rippleLayers[0].name == "sandstone caustic lace");
    CHECK(loaded->rippleLayers[0].edgeBlendWidth == Catch::Approx(0.44F));
    CHECK(loaded->rippleLayers[0].wavelengthMeters == Catch::Approx(0.16F));
    REQUIRE(loaded->fieldLayers.size() == 1U);
    CHECK(loaded->fieldLayers[0].name == "sandstone field");
    CHECK(
        loaded->fieldLayers[0].featureType ==
        invisible_places::water::WaterEffectFeatureType::FieldSurfaceMotion);
    CHECK(loaded->fieldLayers[0].edgeBlendWidth == Catch::Approx(0.33F));
    CHECK(loaded->fieldLayers[0].regionStrength == Catch::Approx(0.77F));
    CHECK(loaded->flowStreamSettings.streamCountTotal == 222U);
    CHECK(loaded->flowStreamSettings.streamWorldLengthMeters == Catch::Approx(0.052F));
    CHECK(loaded->fieldSettings.corridorRadiusMeters == Catch::Approx(0.38F));
    CHECK(loaded->fieldStreamSettings.streamlineCount == 333U);
    CHECK(loaded->fieldStreamSettings.streamlineLengthMeters == Catch::Approx(0.94F));
    CHECK(loaded->causticLookSettings.enabled);
    CHECK(loaded->causticLookSettings.intensity == Catch::Approx(1.4F));
    CHECK(loaded->causticLookSettings.opacityBoost == Catch::Approx(0.22F));
    CHECK(loaded->causticLookSettings.cellSizeMeters == Catch::Approx(0.16F));
    CHECK(loaded->causticLookSettings.lineWidthMeters == Catch::Approx(0.010F));
    CHECK(loaded->causticLookSettings.featherMeters == Catch::Approx(0.003F));
    CHECK(loaded->causticLookSettings.surfacePointSpacingMeters == Catch::Approx(0.002F));
    CHECK(loaded->causticLookSettings.warpAmplitudeMeters == Catch::Approx(0.025F));
    REQUIRE(loaded->tempCausticLookSettings.has_value());
    CHECK(loaded->tempCausticLookSettings->speed == Catch::Approx(1.3F));
    CHECK(loaded->tempCausticLookSettings->lineWidthMeters == Catch::Approx(0.018F));
    REQUIRE(loaded->pathCache.has_value());
    CHECK(loaded->pathCache->supportLayerPath == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loaded->pathCache->supportSignature == "Data/Site2 -5mm.ply|points=4096");
    CHECK(loaded->pathCache->emitterSettingsFingerprint == "source-fingerprint");
    REQUIRE(loaded->pathCache->branches.size() == 1U);
    CHECK(loaded->pathCache->branches[0].id == 77U);
    REQUIRE(loaded->pathCache->branches[0].rawAnchors.size() == 2U);
    CHECK(loaded->pathCache->branches[0].rawAnchors[1].pathDistance == Catch::Approx(1.75F));
    const auto& linkedSettings = invisible_places::water::ResolveWaterSourceSettings(
        loaded->emitters[1],
        loaded->emitters,
        loaded->sourceSettings);
    CHECK(linkedSettings.trailShape.splineAnchorSpacing == Catch::Approx(0.75F));
    auto missingLinkedEmitter = loaded->emitters[1];
    missingLinkedEmitter.linkedSourceSettingsEmitterId = 999U;
    const auto& fallbackSettings = invisible_places::water::ResolveWaterSourceSettings(
        missingLinkedEmitter,
        loaded->emitters,
        loaded->sourceSettings);
    CHECK(fallbackSettings.trailShape.splineAnchorSpacing == Catch::Approx(1.25F));
    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy water caustic look settings derive meter controls", "[water][serialization]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_legacy_caustics.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << R"({
  "schema_version": 3,
  "water_caustic_look_settings": {
    "enabled": true,
    "intensity": 1.0,
    "scale": 5.0,
    "line_sharpness": 0.5,
    "warp": 0.4
  }
})";
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadWaterSourcesDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->causticLookSettings.enabled);
    CHECK(loaded->causticLookSettings.cellSizeMeters == Catch::Approx(0.20F));
    CHECK(loaded->causticLookSettings.lineWidthMeters == Catch::Approx(0.0185F));
    CHECK(loaded->causticLookSettings.featherMeters == Catch::Approx(0.0074F));
    CHECK(loaded->causticLookSettings.warpAmplitudeMeters == Catch::Approx(0.04F));
    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy water trail shape derives looseness", "[water][serialization]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_legacy_trail_looseness.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << R"({
  "schema_version": 3,
  "water_source_settings": {
    "trail_shape": {
      "particle_jitter": 0.4,
      "spline_anchor_spacing": 0.2,
      "trail_lane_count": 5,
      "trail_turbulence": 0.9,
      "trail_momentum": 0.5,
      "normal_turbulence_response": 1.2
    }
  }
})";
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadWaterSourcesDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->sourceSettings.trailShape.trailLooseness == Catch::Approx(0.605F));
    CHECK(loaded->sourceSettings.trailShape.trailSmoothness == Catch::Approx(0.55F));
    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy water source documents migrate split settings", "[water][serialization]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_water_sources.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << R"({
  "schema_version": 1,
  "water_bake_settings": {
    "scale_mode": "detail",
    "support_voxel_size": 0.02,
    "max_bridge_distance": 0.08,
    "path_density": 0.015
  },
  "water_render_settings": {
    "particle_size_pixels": 21.0,
    "particle_opacity": 0.43,
    "particle_density": 1.7,
    "particle_jitter": 0.61,
    "particle_speed": 1.9,
    "spline_anchor_spacing": 0.22,
    "color_variation": 0.77,
    "glow": 0.53
  },
  "water_emitters": [
    {"id": 12, "name": "legacy seep", "position": [0.0, 1.0, 2.0], "scope": "aerial"},
    {
      "id": 13,
      "name": "legacy custom seep",
      "position": [0.0, 1.5, 2.0],
      "source_settings": {
        "point_trail": {
          "particle_jitter": 1.4
        }
      }
    }
  ]
})";
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadWaterSourcesDocument(outputPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->settings.path.legacyScaleMode == invisible_places::water::WaterScaleMode::Detail);
    CHECK(loaded->settings.path.supportVoxelSize == Catch::Approx(0.02F));
    CHECK(loaded->settings.path.pathSampleSpacing == Catch::Approx(0.015F));
    CHECK(loaded->settings.visual.particleSizePixels == Catch::Approx(21.0F));
    CHECK(loaded->settings.visual.particleOpacity == Catch::Approx(0.43F));
    CHECK(loaded->settings.trail.particleDensity == Catch::Approx(1.7F));
    CHECK(loaded->settings.trail.particleJitter == Catch::Approx(0.61F));
    CHECK(loaded->settings.trail.particleSpeed == Catch::Approx(1.9F));
    CHECK(loaded->settings.trail.splineAnchorSpacing == Catch::Approx(0.22F));
    CHECK(loaded->settings.visual.colorVariation == Catch::Approx(0.77F));
    CHECK(loaded->settings.visual.glow == Catch::Approx(0.53F));
    REQUIRE(loaded->emitters.size() == 2U);
    CHECK(loaded->emitters[0].scope == invisible_places::water::WaterScaleMode::Aerial);
    CHECK(
        loaded->emitters[0].sourceSettingsAssignment ==
        invisible_places::water::WaterSourceSettingsAssignment::Default);
    CHECK(
        loaded->emitters[1].sourceSettingsAssignment ==
        invisible_places::water::WaterSourceSettingsAssignment::Custom);
    REQUIRE(loaded->emitters[1].sourceSettings.has_value());
    CHECK(loaded->emitters[1].sourceSettings->trailShape.particleJitter == Catch::Approx(1.4F));
    std::filesystem::remove(outputPath);
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

    auto stylised = style;
    stylised.stylisationMode =
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::NprStylisation;
    stylised.stylisationStrength = 1.0F;
    CHECK(ResolvePointCloudMaterialVariant(stylised) == PointCloudMaterialVariant::Unified);

    auto zeroStrengthStylised = stylised;
    zeroStrengthStylised.stylisationStrength = 0.0F;
    CHECK(ResolvePointCloudMaterialVariant(zeroStrengthStylised) == PointCloudMaterialVariant::ConstantSimple);

    auto caustic = style;
    caustic.causticAnimation = true;
    caustic.causticIntensity = 0.75F;
    caustic.causticMaskFieldSlot = 0;
    caustic.causticEdgeFieldSlot = 1;
    caustic.causticSeedFieldSlot = 2;
    CHECK(invisible_places::renderer::pointcloud::PointCloudStyleHasActiveCaustics(caustic));
    CHECK(ResolvePointCloudMaterialVariant(caustic) == PointCloudMaterialVariant::Unified);

    auto previewTintOnly = caustic;
    previewTintOnly.causticIntensity = 0.0F;
    previewTintOnly.causticPreviewTintAmount = 0.3F;
    previewTintOnly.causticPreviewTintRegionId = 21.0F;
    CHECK(invisible_places::renderer::pointcloud::PointCloudStyleHasActiveCaustics(previewTintOnly));
    CHECK(ResolvePointCloudMaterialVariant(previewTintOnly) == PointCloudMaterialVariant::Unified);

    previewTintOnly.causticAnimation = false;
    CHECK_FALSE(invisible_places::renderer::pointcloud::PointCloudStyleHasActiveCaustics(previewTintOnly));
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

TEST_CASE("Animation path reports world-space speeds and retimes from average speed", "[camera][animation]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 60;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 1.0F, 0.0F}, .durationFrames = 30},
        {.cameraPosition = {10.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 5.0F, 0.0F}, .durationFrames = 30},
    };

    const auto stats = invisible_places::camera::MeasureAnimationPathMotion(path, 0.5F, 32U);
    CHECK(stats.durationSeconds == Catch::Approx(2.0F));
    CHECK(stats.cameraDistance == Catch::Approx(10.0F));
    CHECK(stats.targetDistance == Catch::Approx(4.0F));
    CHECK(stats.averageCameraSpeed == Catch::Approx(5.0F));
    CHECK(stats.averageTargetSpeed == Catch::Approx(2.0F));
    CHECK(stats.currentCameraSpeed == Catch::Approx(5.0F));
    CHECK(stats.currentTargetSpeed == Catch::Approx(2.0F));

    path.durationFrames = invisible_places::camera::AnimationDurationFramesForAverageSpeed(
        path,
        invisible_places::camera::AnimationPathMotionTarget::Camera,
        2.5F,
        32U);
    CHECK(path.durationFrames == 120U);
    const auto retimed = invisible_places::camera::MeasureAnimationPathMotion(path, 0.5F, 32U);
    CHECK(retimed.durationSeconds == Catch::Approx(4.0F));
    CHECK(retimed.averageCameraSpeed == Catch::Approx(2.5F));
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

TEST_CASE("Houdini camera calibration round-trips vertical FOV", "[houdini][camera]") {
    constexpr float kDegrees = 180.0F / glm::pi<float>();
    const auto calibration16x9 = invisible_places::output::BuildHoudiniCameraCalibration(
        55.0F,
        1920U,
        1080U);
    const auto calibration16x10 = invisible_places::output::BuildHoudiniCameraCalibration(
        55.0F,
        1440U,
        900U);
    const auto calibration1x1 = invisible_places::output::BuildHoudiniCameraCalibration(
        55.0F,
        1024U,
        1024U);

    const auto roundTripVerticalFov = [](const invisible_places::output::HoudiniCameraCalibration& calibration) {
        return 2.0F * std::atan((calibration.verticalApertureMm * 0.5F) / calibration.focalLengthMm);
    };

    CHECK(roundTripVerticalFov(calibration16x9) * kDegrees == Catch::Approx(55.0F).margin(0.0001F));
    CHECK(roundTripVerticalFov(calibration16x10) * kDegrees == Catch::Approx(55.0F).margin(0.0001F));
    CHECK(roundTripVerticalFov(calibration1x1) * kDegrees == Catch::Approx(55.0F).margin(0.0001F));
    CHECK(calibration16x10.focalLengthMm == Catch::Approx(24.86555F).margin(0.0001F));
    CHECK(calibration16x10.aspectRatio == Catch::Approx(1.6F).margin(0.0001F));
    CHECK(calibration16x10.pixelAspectRatio == Catch::Approx(1.0F));
}

TEST_CASE("Houdini horizontal FOV follows output aspect ratio", "[houdini][camera]") {
    const auto calibration16x9 = invisible_places::output::BuildHoudiniCameraCalibration(
        55.0F,
        1920U,
        1080U);
    const auto calibration16x10 = invisible_places::output::BuildHoudiniCameraCalibration(
        55.0F,
        1440U,
        900U);
    const auto calibration1x1 = invisible_places::output::BuildHoudiniCameraCalibration(
        55.0F,
        1024U,
        1024U);

    CHECK(calibration16x9.verticalFovDegrees == Catch::Approx(55.0F));
    CHECK(calibration16x10.verticalFovDegrees == Catch::Approx(55.0F));
    CHECK(calibration1x1.verticalFovDegrees == Catch::Approx(55.0F));
    CHECK(calibration16x9.horizontalFovDegrees > calibration16x10.horizontalFovDegrees);
    CHECK(calibration16x10.horizontalFovDegrees > calibration1x1.horizontalFovDegrees);
    CHECK(calibration1x1.horizontalFovDegrees == Catch::Approx(55.0F).margin(0.0001F));
}

TEST_CASE("Houdini camera script keeps raw samples and transform hook", "[houdini][camera]") {
    invisible_places::camera::AnimationPath path;
    path.name = "Raw Houdini";
    path.durationFrames = 1;
    path.depthOfFieldEnabled = true;
    path.apertureFStops = 5.6F;
    path.keys = {
        {
            .id = "key_a",
            .cameraPosition = {1.0F, 2.0F, 3.0F},
            .focusPoint = {4.0F, 5.0F, 6.0F},
            .fovDegrees = 55.0F,
            .nearPlane = 0.02F,
            .farPlane = 100.0F,
            .durationFrames = 1,
        },
        {
            .id = "key_b",
            .cameraPosition = {7.0F, 8.0F, 9.0F},
            .focusPoint = {10.0F, 11.0F, 12.0F},
            .fovDegrees = 55.0F,
            .nearPlane = 0.02F,
            .farPlane = 100.0F,
            .durationFrames = 1,
        },
    };

    invisible_places::output::RenderJobSettings settings;
    settings.width = 1440U;
    settings.height = 900U;
    settings.framesPerSecond = 30U;
    settings.startFrame = 0U;
    settings.endFrame = 0U;

    const auto expectedFrames = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(expectedFrames.size() == 2U);

    invisible_places::output::HoudiniCameraScriptSettings scriptSettings;
    scriptSettings.transformNode = "/obj/Points/To_Base";
    scriptSettings.cameraPrim = "/cameras/camera1";
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_houdini_camera_test.py";
    std::string errorMessage;
    REQUIRE(invisible_places::output::WriteHoudiniCameraScript(
        path,
        settings,
        outputPath,
        &errorMessage,
        scriptSettings));

    std::ifstream scriptFile{outputPath};
    const std::string scriptText{
        std::istreambuf_iterator<char>{scriptFile},
        std::istreambuf_iterator<char>{}};
    CHECK(scriptText.find("# sample_count: 2") != std::string::npos);
    CHECK(scriptText.find("# default_transform_node: /obj/Points/To_Base") != std::string::npos);
    CHECK(scriptText.find("# first_raw_position: [1.0,2.0,3.0]") != std::string::npos);
    CHECK(scriptText.find("# first_raw_target: [4.0,5.0,6.0]") != std::string::npos);
    CHECK(scriptText.find("# first_raw_position: [1.0,3.0,-2.0]") == std::string::npos);
    CHECK(scriptText.find("\\\"sample_count\\\": 2") != std::string::npos);
    CHECK(scriptText.find("\\\"vertical_fov_degrees\\\": 55.0") != std::string::npos);
    CHECK(scriptText.find("\\\"horizontal_fov_degrees\\\"") != std::string::npos);
    CHECK(scriptText.find("\\\"aspect_ratio\\\": 1.6") != std::string::npos);
    CHECK(scriptText.find("\\\"pixel_aspect_ratio\\\": 1.0") != std::string::npos);
    CHECK(scriptText.find("\\\"horizontal_aperture_mm\\\": 41.421") != std::string::npos);
    CHECK(scriptText.find("\\\"vertical_aperture_mm\\\": 25.888") != std::string::npos);
    CHECK(scriptText.find("\\\"focal_length_mm\\\": 24.865") != std::string::npos);
    CHECK(scriptText.find("\\\"focus_distance\\\"") != std::string::npos);
    CHECK(scriptText.find("\\\"aperture_f_stops\\\": 5.599") != std::string::npos);
    CHECK(scriptText.find("\\\"has_depth_of_field\\\": true") != std::string::npos);
    CHECK(scriptText.find("build_transform_sop_matrix") != std::string::npos);
    CHECK(scriptText.find("aperture control") != std::string::npos);
    CHECK(scriptText.find("horizontal aperture") != std::string::npos);
    CHECK(scriptText.find("reset_camera_windowing") != std::string::npos);
    CHECK(scriptText.find("animated lens") != std::string::npos);
    const auto compileCommand = "python3 -m py_compile \"" + outputPath.string() + "\"";
    CHECK(std::system(compileCommand.c_str()) == 0);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Houdini camera script disables depth of field explicitly", "[houdini][camera]") {
    invisible_places::camera::AnimationPath path;
    path.name = "No DoF";
    path.durationFrames = 1;
    path.depthOfFieldEnabled = false;
    path.apertureFStops = 2.0F;
    path.keys = {
        {
            .id = "key_a",
            .cameraPosition = {0.0F, 0.0F, 0.0F},
            .focusPoint = {0.0F, 0.0F, -10.0F},
            .fovDegrees = 42.0F,
            .nearPlane = 0.01F,
            .farPlane = 50.0F,
            .durationFrames = 1,
        },
    };

    invisible_places::output::RenderJobSettings settings;
    settings.width = 1920U;
    settings.height = 1080U;

    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_houdini_camera_nodof_test.py";
    std::string errorMessage;
    REQUIRE(invisible_places::output::WriteHoudiniCameraScript(
        path,
        settings,
        outputPath,
        &errorMessage));

    std::ifstream scriptFile{outputPath};
    const std::string scriptText{
        std::istreambuf_iterator<char>{scriptFile},
        std::istreambuf_iterator<char>{}};

    CHECK(scriptText.find("\\\"has_depth_of_field\\\": false") != std::string::npos);
    CHECK(scriptText.find("sample[\"aperture_f_stops\"] if sample[\"has_depth_of_field\"] else 0.0") !=
          std::string::npos);
    CHECK(scriptText.find("depth of field: {'on' if any(sample['has_depth_of_field']") != std::string::npos);
    const auto compileCommand = "python3 -m py_compile \"" + outputPath.string() + "\"";
    CHECK(std::system(compileCommand.c_str()) == 0);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Houdini camera import script exposes HIP extraction workflow", "[houdini][camera]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_houdini_camera_importer.py";
    std::string errorMessage;
    REQUIRE(invisible_places::output::WriteHoudiniCameraImportScript(outputPath, &errorMessage));

    std::ifstream scriptFile{outputPath};
    const std::string scriptText{
        std::istreambuf_iterator<char>{scriptFile},
        std::istreambuf_iterator<char>{}};

    CHECK(scriptText.find("parser.add_argument(\"--hip\"") != std::string::npos);
    CHECK(scriptText.find("DEFAULT_CAMERA_PRIM") != std::string::npos);
    CHECK(scriptText.find("DEFAULT_TRANSFORM_NODE") != std::string::npos);
    CHECK(scriptText.find("_invisible_places_camera.json") != std::string::npos);
    CHECK(scriptText.find("sample_camera") != std::string::npos);
    CHECK(scriptText.find("alignment_matrix") != std::string::npos);
    const auto compileCommand = "python3 -m py_compile \"" + outputPath.string() + "\"";
    CHECK(std::system(compileCommand.c_str()) == 0);

    std::filesystem::remove(outputPath);
}

TEST_CASE("Houdini camera JSON imports explicit orientation and lens settings", "[houdini][camera]") {
    const auto inputPath = std::filesystem::temp_directory_path() / "invisible_places_houdini_camera_import.json";
    {
        std::ofstream output{inputPath, std::ios::trunc};
        output << R"JSON({
  "schema_version": 1,
  "source": "Houdini",
  "name": "Houdini Rolled Camera",
  "width": 1920,
  "height": 1080,
  "fps": 24,
  "start_frame": 1001,
  "end_frame": 1002,
  "samples": [
    {
      "frame": 1001,
      "position": [1.0, 2.0, 3.0],
      "target": [1.0, 2.0, -7.0],
      "up": [1.0, 0.0, 0.0],
      "vertical_fov_degrees": 42.0,
      "horizontal_fov_degrees": 65.0,
      "aspect_ratio": 1.7777778,
      "pixel_aspect_ratio": 1.0,
      "horizontal_aperture_mm": 41.4214,
      "vertical_aperture_mm": 23.2995,
      "focal_length_mm": 30.3208,
      "near_plane": 0.02,
      "far_plane": 500.0,
      "focus_distance": 10.0,
      "aperture_f_stops": 2.8,
      "has_depth_of_field": true
    },
    {
      "frame": 1002,
      "position": [2.0, 2.0, 3.0],
      "target": [2.0, 2.0, -7.0],
      "up": [1.0, 0.0, 0.0],
      "vertical_fov_degrees": 44.0,
      "near_plane": 0.03,
      "far_plane": 600.0,
      "focus_distance": 10.0,
      "aperture_f_stops": 4.0,
      "has_depth_of_field": true
    }
  ]
})JSON";
    }

    std::string errorMessage;
    const auto importedPath = invisible_places::output::LoadHoudiniCameraAnimationPath(inputPath, &errorMessage);
    REQUIRE(importedPath.has_value());
    REQUIRE(importedPath->keys.size() == 2U);
    CHECK(importedPath->name == "Houdini Rolled Camera");
    CHECK(importedPath->exportSettings.width == 1920U);
    CHECK(importedPath->exportSettings.height == 1080U);
    CHECK(importedPath->exportSettings.framesPerSecond == 24U);
    CHECK(importedPath->exportSettings.startFrame == 0U);
    CHECK(importedPath->exportSettings.endFrame == 1U);
    CHECK(importedPath->depthOfFieldEnabled);
    CHECK(importedPath->keys[0].hasOrientation);
    CHECK(importedPath->keys[0].hasFocusDistance);
    CHECK(importedPath->keys[0].hasApertureFStops);
    CHECK(importedPath->keys[0].fovDegrees == Catch::Approx(42.0F));
    CHECK(importedPath->keys[1].fovDegrees == Catch::Approx(44.0F));

    const auto evaluated = invisible_places::camera::EvaluateAnimationPath(importedPath.value(), 0.0F);
    const auto orientation = invisible_places::camera::QuaternionFromCameraState(evaluated.camera);
    const auto up = orientation * glm::vec3{0.0F, 1.0F, 0.0F};
    CHECK(glm::dot(glm::normalize(up), glm::vec3{1.0F, 0.0F, 0.0F}) == Catch::Approx(1.0F).margin(0.0001F));
    CHECK(evaluated.camera.focusDistance == Catch::Approx(10.0F));
    CHECK(evaluated.camera.apertureFStops == Catch::Approx(2.8F));
    CHECK(evaluated.camera.nearPlane == Catch::Approx(0.02F));
    CHECK(evaluated.camera.farPlane == Catch::Approx(500.0F));

    const auto roundTripPath =
        std::filesystem::temp_directory_path() / "invisible_places_houdini_import_roundtrip.ipanim.json";
    REQUIRE(invisible_places::serialization::SaveAnimationPath(importedPath.value(), roundTripPath, &errorMessage));
    const auto loadedPath = invisible_places::serialization::LoadAnimationPath(roundTripPath, &errorMessage);
    REQUIRE(loadedPath.has_value());
    REQUIRE(loadedPath->keys.size() == 2U);
    CHECK(loadedPath->keys[0].hasOrientation);
    CHECK(loadedPath->keys[0].hasFocusDistance);
    CHECK(loadedPath->keys[0].hasApertureFStops);

    std::filesystem::remove(inputPath);
    std::filesystem::remove(roundTripPath);
}

TEST_CASE("Animation path serialization round-trips standalone files", "[serialization][animation]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_roundtrip.ipanim.json";

    invisible_places::camera::AnimationPath path;
    path.name = "Roundtrip Animation";
    path.durationFrames = 72;
    path.associatedLayerPaths = {"Data/Site2 -5mm.ply", "Data/Site3.ply"};
    path.depthOfFieldEnabled = false;
    path.apertureFStops = 2.8F;
    path.depthOfFieldMaxBlurPixels = 36.0F;
    path.exportSettings = {
        .outputDirectory = "Saved/renders/Roundtrip",
        .width = 1280,
        .height = 720,
        .framesPerSecond = 24,
        .stillCameraDurationSeconds = 6.25F,
        .startFrame = 5,
        .endFrame = 42,
    };
    path.exportVisualNames = {"Painty", "X-Ray RGB"};
    path.waterAnimationTrailSettings = invisible_places::water::DefaultWaterAnimationTrailSettings();
    path.waterAnimationTrailSettings->particleDensity = 2.25F;
    path.waterAnimationTrailSettings->particleSpeed = 1.4F;
    path.waterAnimationTrailSettings->colorVariation = 0.72F;
    path.waterAnimationTrailSettings->trailLengthMeters = 1.6F;
    path.waterAnimationTrailSettings->trailSampleSpacingMeters = 0.066F;
    path.tempWaterAnimationTrailSettings = path.waterAnimationTrailSettings;
    path.tempWaterAnimationTrailSettings->particleSpeed = 1.8F;
    path.tempWaterAnimationTrailSettings->trailLengthMeters = 2.1F;
    path.tempWaterAnimationTrailSettings->trailSampleSpacingMeters = 0.041F;
    path.waterPointVisualStyle = invisible_places::renderer::pointcloud::PointCloudStyleState{};
    invisible_places::style::SetScalarConstant(&path.waterPointVisualStyle->opacity, 0.31F);
    path.tempWaterPointVisualStyle = path.waterPointVisualStyle;
    invisible_places::style::SetScalarConstant(&path.tempWaterPointVisualStyle->opacity, 0.44F);
    path.waterCausticLookSettings = invisible_places::water::DefaultWaterCausticLookSettings();
    path.waterCausticLookSettings->enabled = true;
    path.waterCausticLookSettings->intensity = 1.6F;
    path.waterCausticLookSettings->scale = 8.0F;
    path.waterCausticLookSettings->cellSizeMeters = 0.14F;
    path.waterCausticLookSettings->lineWidthMeters = 0.009F;
    path.waterCausticLookSettings->featherMeters = 0.002F;
    path.waterCausticLookSettings->surfacePointSpacingMeters = 0.0015F;
    path.waterCausticLookSettings->warpAmplitudeMeters = 0.022F;
    path.waterCausticLookSettings->tintRed = 0.72F;
    path.tempWaterCausticLookSettings = path.waterCausticLookSettings;
    path.tempWaterCausticLookSettings->pointSizeBoost = 0.35F;
    path.tempWaterCausticLookSettings->featherMeters = 0.011F;
    path.keys = {
        {
            .id = "key_entry",
            .cameraPosition = {1.0F, 2.0F, 3.0F},
            .focusPoint = {4.0F, 5.0F, 6.0F},
            .fovDegrees = 42.0F,
            .nearPlane = 0.02F,
            .farPlane = 900.0F,
            .durationFrames = 24,
            .sourceShotName = "Entry",
            .linkedCameraId = "camera_entry",
            .linkedCameraName = "Entry",
        },
        {
            .id = "key_exit",
            .cameraPosition = {7.0F, 8.0F, 9.0F},
            .focusPoint = {10.0F, 11.0F, 12.0F},
            .fovDegrees = 55.0F,
            .nearPlane = 0.04F,
            .farPlane = 1200.0F,
            .durationFrames = 48,
            .sourceShotName = "Exit",
            .linkedCameraId = "camera_exit",
            .linkedCameraName = "Exit",
        },
    };

    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveAnimationPath(path, outputPath, &errorMessage));
    {
        std::ifstream savedAnimation{outputPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedAnimation},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find("\"schema_version\": 7") != std::string::npos);
        CHECK(savedJson.find("\"associated_layer_paths\"") != std::string::npos);
        CHECK(savedJson.find("\"still_camera_duration_seconds\"") != std::string::npos);
        CHECK(savedJson.find("\"linked_camera_id\": \"camera_entry\"") != std::string::npos);
        CHECK(savedJson.find("\"water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_length_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_sample_spacing_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_point_visual_style\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_point_visual_style\"") == std::string::npos);
        CHECK(savedJson.find("\"water_caustic_look_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_caustic_look_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_visual_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_visual_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"water_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_settings\"") == std::string::npos);
    }
    const auto loadedPath = invisible_places::serialization::LoadAnimationPath(outputPath, &errorMessage);
    REQUIRE(loadedPath.has_value());
    REQUIRE(loadedPath->keys.size() == 2);
    CHECK(loadedPath->name == "Roundtrip Animation");
    CHECK(loadedPath->durationFrames == 72);
    REQUIRE(loadedPath->associatedLayerPaths.size() == 2);
    CHECK(loadedPath->associatedLayerPaths[0] == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loadedPath->associatedLayerPaths[1] == std::filesystem::path{"Data/Site3.ply"});
    CHECK_FALSE(loadedPath->depthOfFieldEnabled);
    CHECK(loadedPath->apertureFStops == Catch::Approx(2.8F));
    CHECK(loadedPath->depthOfFieldMaxBlurPixels == Catch::Approx(36.0F));
    CHECK(loadedPath->exportSettings.outputDirectory == "Saved/renders/Roundtrip");
    CHECK(loadedPath->exportSettings.width == 1280);
    CHECK(loadedPath->exportSettings.height == 720);
    CHECK(loadedPath->exportSettings.framesPerSecond == 24);
    CHECK(loadedPath->exportSettings.stillCameraDurationSeconds == Catch::Approx(6.25F));
    CHECK(loadedPath->exportSettings.startFrame == 5);
    CHECK(loadedPath->exportSettings.endFrame == 42);
    REQUIRE(loadedPath->exportVisualNames.size() == 2);
    CHECK(loadedPath->exportVisualNames[0] == "Painty");
    CHECK(loadedPath->exportVisualNames[1] == "X-Ray RGB");
    REQUIRE(loadedPath->waterAnimationTrailSettings.has_value());
    CHECK(loadedPath->waterAnimationTrailSettings->particleDensity == Catch::Approx(2.25F));
    CHECK(loadedPath->waterAnimationTrailSettings->particleSpeed == Catch::Approx(1.4F));
    CHECK(loadedPath->waterAnimationTrailSettings->colorVariation == Catch::Approx(0.72F));
    CHECK(loadedPath->waterAnimationTrailSettings->trailLengthMeters == Catch::Approx(1.6F));
    CHECK(loadedPath->waterAnimationTrailSettings->trailSampleSpacingMeters == Catch::Approx(0.066F));
    REQUIRE(loadedPath->tempWaterAnimationTrailSettings.has_value());
    CHECK(loadedPath->tempWaterAnimationTrailSettings->particleSpeed == Catch::Approx(1.8F));
    CHECK(loadedPath->tempWaterAnimationTrailSettings->trailLengthMeters == Catch::Approx(2.1F));
    CHECK(loadedPath->tempWaterAnimationTrailSettings->trailSampleSpacingMeters == Catch::Approx(0.041F));
    CHECK_FALSE(loadedPath->waterPointVisualStyle.has_value());
    CHECK_FALSE(loadedPath->tempWaterPointVisualStyle.has_value());
    REQUIRE(loadedPath->waterCausticLookSettings.has_value());
    CHECK(loadedPath->waterCausticLookSettings->enabled);
    CHECK(loadedPath->waterCausticLookSettings->intensity == Catch::Approx(1.6F));
    CHECK(loadedPath->waterCausticLookSettings->scale == Catch::Approx(8.0F));
    CHECK(loadedPath->waterCausticLookSettings->cellSizeMeters == Catch::Approx(0.14F));
    CHECK(loadedPath->waterCausticLookSettings->lineWidthMeters == Catch::Approx(0.009F));
    CHECK(loadedPath->waterCausticLookSettings->featherMeters == Catch::Approx(0.002F));
    CHECK(loadedPath->waterCausticLookSettings->surfacePointSpacingMeters == Catch::Approx(0.0015F));
    CHECK(loadedPath->waterCausticLookSettings->warpAmplitudeMeters == Catch::Approx(0.022F));
    CHECK(loadedPath->waterCausticLookSettings->tintRed == Catch::Approx(0.72F));
    REQUIRE(loadedPath->tempWaterCausticLookSettings.has_value());
    CHECK(loadedPath->tempWaterCausticLookSettings->pointSizeBoost == Catch::Approx(0.35F));
    CHECK(loadedPath->tempWaterCausticLookSettings->featherMeters == Catch::Approx(0.011F));
    CHECK_FALSE(loadedPath->waterSettings.has_value());
    CHECK_FALSE(loadedPath->tempWaterSettings.has_value());
    CHECK(loadedPath->keys[0].cameraPosition[2] == Catch::Approx(3.0F));
    CHECK(loadedPath->keys[0].focusPoint[1] == Catch::Approx(5.0F));
    CHECK(loadedPath->keys[0].fovDegrees == Catch::Approx(42.0F));
    CHECK(loadedPath->keys[0].nearPlane == Catch::Approx(0.02F));
    CHECK(loadedPath->keys[0].farPlane == Catch::Approx(900.0F));
    CHECK(loadedPath->keys[0].sourceShotName == "Entry");
    CHECK(loadedPath->keys[0].id == "key_entry");
    CHECK(loadedPath->keys[0].linkedCameraId == "camera_entry");
    CHECK(loadedPath->keys[0].linkedCameraName == "Entry");

    {
        std::ofstream legacyOutput{outputPath, std::ios::trunc};
        legacyOutput << R"({
  "schema_version": 7,
  "name": "Legacy Water Visual Animation",
  "duration_frames": 12,
  "associated_layer_paths": [],
  "export_settings": {},
  "water_point_visual_style": {
    "opacity": {"active": true, "constant_value": [0.31, 0.0, 0.0, 0.0]}
  },
  "temp_water_point_visual_style": {
    "opacity": {"active": true, "constant_value": [0.44, 0.0, 0.0, 0.0]}
  },
  "keys": [
    {
      "id": "legacy_key",
      "camera_position": [0.0, 0.0, 2.0],
      "focus_point": [0.0, 0.0, 0.0]
    }
  ]
})";
    }
    const auto legacyLoadedPath =
        invisible_places::serialization::LoadAnimationPath(outputPath, &errorMessage);
    REQUIRE(legacyLoadedPath.has_value());
    REQUIRE(legacyLoadedPath->waterPointVisualStyle.has_value());
    CHECK(invisible_places::style::ScalarConstant(legacyLoadedPath->waterPointVisualStyle->opacity) ==
          Catch::Approx(0.31F));
    REQUIRE(legacyLoadedPath->tempWaterPointVisualStyle.has_value());
    CHECK(invisible_places::style::ScalarConstant(legacyLoadedPath->tempWaterPointVisualStyle->opacity) ==
          Catch::Approx(0.44F));

    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy animation path files load with default export metadata", "[serialization][animation]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_legacy.ipanim.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output
            << "{\n"
            << "  \"schema_version\": 1,\n"
            << "  \"name\": \"Legacy Animation\",\n"
            << "  \"duration_frames\": 30,\n"
            << "  \"keys\": [\n"
            << "    {\"camera_position\": [0, 0, 0], \"focus_point\": [0, 0, -1]},\n"
            << "    {\"camera_position\": [1, 0, 0], \"focus_point\": [1, 0, -1]}\n"
            << "  ]\n"
            << "}\n";
    }

    std::string errorMessage;
    const auto loadedPath = invisible_places::serialization::LoadAnimationPath(outputPath, &errorMessage);
    REQUIRE(loadedPath.has_value());
    CHECK(loadedPath->name == "Legacy Animation");
    CHECK(loadedPath->exportSettings.outputDirectory.empty());
    CHECK(loadedPath->exportSettings.width == 1920);
    CHECK(loadedPath->exportSettings.height == 1080);
    CHECK(loadedPath->exportSettings.framesPerSecond == 30);
    CHECK(loadedPath->exportSettings.startFrame == 0);
    CHECK(loadedPath->exportSettings.endFrame == 0);
    CHECK(loadedPath->exportVisualNames.empty());
    CHECK(loadedPath->associatedLayerPaths.empty());
    REQUIRE(loadedPath->keys.size() == 2);
    CHECK(!loadedPath->keys[0].id.empty());
    CHECK(loadedPath->keys[0].linkedCameraId.empty());
    CHECK(loadedPath->keys[0].linkedCameraName.empty());
    std::filesystem::remove(outputPath);
}

TEST_CASE("Pre-link animation path files load as unlinked snapshots", "[serialization][animation]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_legacy_v3.ipanim.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output
            << "{\n"
            << "  \"schema_version\": 3,\n"
            << "  \"name\": \"Legacy Associated Animation\",\n"
            << "  \"duration_frames\": 60,\n"
            << "  \"associated_layer_paths\": [\"Data/Site2 -5mm.ply\"],\n"
            << "  \"keys\": [\n"
            << "    {\"camera_position\": [0, 0, 0], \"focus_point\": [0, 0, -1], \"source_shot_name\": \"Entry\"},\n"
            << "    {\"camera_position\": [1, 0, 0], \"focus_point\": [1, 0, -1], \"source_shot_name\": \"Exit\"}\n"
            << "  ]\n"
            << "}\n";
    }

    std::string errorMessage;
    const auto loadedPath = invisible_places::serialization::LoadAnimationPath(outputPath, &errorMessage);
    REQUIRE(loadedPath.has_value());
    REQUIRE(loadedPath->associatedLayerPaths.size() == 1);
    CHECK(loadedPath->associatedLayerPaths[0] == std::filesystem::path{"Data/Site2 -5mm.ply"});
    REQUIRE(loadedPath->keys.size() == 2);
    CHECK(!loadedPath->keys[0].id.empty());
    CHECK(loadedPath->keys[0].sourceShotName == "Entry");
    CHECK(loadedPath->keys[0].linkedCameraId.empty());
    CHECK(loadedPath->keys[0].linkedCameraName.empty());
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

TEST_CASE("Camera shots convert to animation focus keys from view targets", "[camera][animation]") {
    auto makeShot = [](
                        std::string id,
                        std::array<float, 3> position,
                        std::array<float, 3> target,
                        std::array<float, 3> orbitCenter) {
        invisible_places::camera::CameraShot shot;
        shot.id = std::move(id);
        shot.name = shot.id;
        shot.state.position = position;
        shot.state.target = target;
        shot.state.orbitCenter = orbitCenter;
        shot.state.hasOrbitCenter = true;
        return shot;
    };

    const std::vector<invisible_places::camera::CameraShot> sourceShots = {
        makeShot("camera_a", {0.0F, 1.0F, 2.0F}, {3.0F, 4.0F, 5.0F}, {30.0F, 40.0F, 50.0F}),
        makeShot("camera_b", {6.0F, 7.0F, 8.0F}, {9.0F, 10.0F, 11.0F}, {90.0F, 100.0F, 110.0F}),
    };

    const auto animationPath = invisible_places::camera::BuildAnimationPathFromCameraShots(
        "Target Focus",
        sourceShots,
        60U);

    REQUIRE(animationPath.keys.size() == sourceShots.size());
    for (std::size_t index = 0; index < sourceShots.size(); ++index) {
        CHECK(animationPath.keys[index].focusPoint[0] == Catch::Approx(sourceShots[index].state.target[0]));
        CHECK(animationPath.keys[index].focusPoint[1] == Catch::Approx(sourceShots[index].state.target[1]));
        CHECK(animationPath.keys[index].focusPoint[2] == Catch::Approx(sourceShots[index].state.target[2]));
        CHECK(animationPath.keys[index].focusPoint[0] != Catch::Approx(sourceShots[index].state.orbitCenter[0]));
    }
}

TEST_CASE("Weighted camera path conversion does not mutate source camera shots", "[camera][animation]") {
    auto makeShot = [](
                        std::string name,
                        std::array<float, 3> position,
                        std::array<float, 3> target,
                        std::array<float, 3> orbitCenter,
                        std::uint32_t durationFrames) {
        invisible_places::camera::CameraShot shot;
        shot.id = name;
        shot.name = std::move(name);
        shot.durationFrames = durationFrames;
        shot.state.position = position;
        shot.state.target = target;
        shot.state.orbitCenter = orbitCenter;
        shot.state.hasOrbitCenter = true;
        return shot;
    };

    std::vector<invisible_places::camera::CameraShot> sourceShots = {
        makeShot("A", {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -4.0F}, {1.0F, 1.0F, 1.0F}, 12U),
        makeShot("B", {4.0F, 1.0F, 0.0F}, {4.0F, 1.0F, -4.0F}, {2.0F, 2.0F, 2.0F}, 24U),
        makeShot("C", {8.0F, 0.0F, 2.0F}, {8.0F, 0.0F, -2.0F}, {3.0F, 3.0F, 3.0F}, 36U),
    };
    const auto originalShots = sourceShots;

    const auto weightedShots = invisible_places::camera::BuildWeightedCameraPathShots(sourceShots, 90U);
    const auto animationPath = invisible_places::camera::BuildAnimationPathFromCameraShots(
        "Weighted",
        weightedShots,
        90U);

    REQUIRE(animationPath.keys.size() == sourceShots.size());
    REQUIRE(sourceShots.size() == originalShots.size());
    for (std::size_t index = 0; index < sourceShots.size(); ++index) {
        CHECK(sourceShots[index].state.position[0] == Catch::Approx(originalShots[index].state.position[0]));
        CHECK(sourceShots[index].state.position[1] == Catch::Approx(originalShots[index].state.position[1]));
        CHECK(sourceShots[index].state.position[2] == Catch::Approx(originalShots[index].state.position[2]));
        CHECK(sourceShots[index].state.target[0] == Catch::Approx(originalShots[index].state.target[0]));
        CHECK(sourceShots[index].state.target[1] == Catch::Approx(originalShots[index].state.target[1]));
        CHECK(sourceShots[index].state.target[2] == Catch::Approx(originalShots[index].state.target[2]));
        CHECK(sourceShots[index].state.orbitCenter[0] == Catch::Approx(originalShots[index].state.orbitCenter[0]));
        CHECK(sourceShots[index].state.orbitCenter[1] == Catch::Approx(originalShots[index].state.orbitCenter[1]));
        CHECK(sourceShots[index].state.orbitCenter[2] == Catch::Approx(originalShots[index].state.orbitCenter[2]));
        CHECK(sourceShots[index].state.hasOrbitCenter == originalShots[index].state.hasOrbitCenter);
    }
}

TEST_CASE("Animation key edits do not mutate source camera shots", "[camera][animation]") {
    invisible_places::camera::CameraShot shot;
    shot.id = "camera_original";
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
    REQUIRE(animationPath.keys.size() == 2);
    CHECK(animationPath.keys[0].linkedCameraId == "camera_original");
    CHECK(animationPath.keys[0].linkedCameraName == "Original");
    CHECK(!animationPath.keys[0].id.empty());

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

    settings.framesPerSecond = 60;
    const auto sixtyFpsFrames = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(sixtyFpsFrames.size() == 121);
    CHECK(sixtyFpsFrames[60].position[0] == Catch::Approx(middleEvaluation.camera.position[0]));
    CHECK(sixtyFpsFrames[60].position[1] == Catch::Approx(middleEvaluation.camera.position[1]));

    settings.framesPerSecond = 30;
    settings.startFrame = 10;
    settings.endFrame = 12;
    const auto rangedFrames = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(rangedFrames.size() == 3);
    CHECK(rangedFrames.front().position[0] == Catch::Approx(frames[10].position[0]));
    CHECK(rangedFrames.back().position[0] == Catch::Approx(frames[12].position[0]));
}

TEST_CASE("Still camera render sequence repeats one camera for duration", "[output][animation]") {
    invisible_places::camera::CameraState cameraState;
    cameraState.position = {4.0F, 5.0F, 6.0F};
    cameraState.target = {1.0F, 2.0F, 3.0F};

    invisible_places::output::RenderJobSettings settings;
    settings.framesPerSecond = 24;
    settings.stillCameraDurationSeconds = 2.5F;

    const auto frames = invisible_places::output::BuildStillCameraRenderSequence(cameraState, settings);
    REQUIRE(frames.size() == 60);
    CHECK(frames.front().position[0] == Catch::Approx(4.0F));
    CHECK(frames.back().position[2] == Catch::Approx(6.0F));
    CHECK(frames.back().target[1] == Catch::Approx(2.0F));
}

TEST_CASE("Adaptive export point-size scale follows output viewport ratio", "[output][animation]") {
    CHECK(invisible_places::output::ComputePointSizePixelScale(1920, 1080, 1920, 1080) == Catch::Approx(1.0F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(3840, 2160, 1920, 1080) == Catch::Approx(2.0F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(960, 540, 1920, 1080) == Catch::Approx(0.5F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(3840, 1080, 1920, 1080) == Catch::Approx(std::sqrt(2.0F)));
    CHECK(invisible_places::output::ComputePointSizePixelScale(1920, 1080, 0, 1080) == Catch::Approx(1.0F));
}

TEST_CASE("Point-cloud export density modes are explicit", "[output][animation][lod]") {
    using invisible_places::output::PointCloudExportDensityMode;
    using invisible_places::output::PointCloudExportDensityModeDescription;
    using invisible_places::output::PointCloudExportDensityModeIsPreview;
    using invisible_places::output::PointCloudExportDensityModeName;
    using invisible_places::output::PointCloudExportDensityModePolicyFor;
    using invisible_places::output::PointCloudExportDensityModeRequiresViewportSnapshot;
    using invisible_places::output::PointCloudExportDensityModeUsesDeterministicSelection;
    using invisible_places::output::PointCloudExportDensityModeUsesFullSource;

    CHECK(PointCloudExportDensityModeName(PointCloudExportDensityMode::FullSource) == std::string{"Full Source"});
    CHECK(
        PointCloudExportDensityModeName(PointCloudExportDensityMode::AdaptiveHighQuality) ==
        std::string{"Adaptive High Quality"});
    CHECK(
        PointCloudExportDensityModeName(PointCloudExportDensityMode::FastAdaptivePreview) ==
        std::string{"Fast Adaptive Preview"});
    CHECK(
        PointCloudExportDensityModeName(PointCloudExportDensityMode::MatchViewportAdaptive) ==
        std::string{"Match Viewport Adaptive"});
    CHECK(PointCloudExportDensityModeUsesFullSource(PointCloudExportDensityMode::FullSource));
    CHECK_FALSE(PointCloudExportDensityModeUsesFullSource(PointCloudExportDensityMode::AdaptiveHighQuality));
    CHECK_FALSE(PointCloudExportDensityModeUsesFullSource(PointCloudExportDensityMode::MatchViewportAdaptive));
    CHECK(PointCloudExportDensityModeIsPreview(PointCloudExportDensityMode::FastAdaptivePreview));
    CHECK_FALSE(PointCloudExportDensityModeIsPreview(PointCloudExportDensityMode::AdaptiveHighQuality));
    CHECK(PointCloudExportDensityModeRequiresViewportSnapshot(PointCloudExportDensityMode::MatchViewportAdaptive));
    CHECK(PointCloudExportDensityModeRequiresViewportSnapshot(PointCloudExportDensityMode::ArtisticAsPreview));
    CHECK_FALSE(PointCloudExportDensityModeRequiresViewportSnapshot(PointCloudExportDensityMode::AdaptiveHighQuality));
    CHECK(PointCloudExportDensityModeUsesDeterministicSelection(PointCloudExportDensityMode::AdaptiveHighQuality));
    CHECK(PointCloudExportDensityModeUsesDeterministicSelection(PointCloudExportDensityMode::FastAdaptivePreview));
    CHECK(PointCloudExportDensityModeDescription(PointCloudExportDensityMode::FullSource) == std::string{
        PointCloudExportDensityModePolicyFor(PointCloudExportDensityMode::FullSource).logDescription});
}

TEST_CASE("Quick MP4 output paths append visual names and collision suffixes", "[output][video]") {
    const auto outputDirectory = std::filesystem::temp_directory_path() / "invisible_places_quick_mp4_names";
    std::filesystem::create_directories(outputDirectory);
    const auto firstPath = outputDirectory / "Site_1_Painty.mp4";
    const auto secondPath = outputDirectory / "Site_1_Painty_1.mp4";
    const auto reservedPath = outputDirectory / "Site_1_Painty_2.mp4";

    {
        std::ofstream first{firstPath, std::ios::trunc};
        first << "existing";
    }
    {
        std::ofstream second{secondPath, std::ios::trunc};
        second << "existing";
    }

    const auto uniquePath = invisible_places::output::BuildUniqueQuickMp4OutputPath(
        outputDirectory,
        "Site 1",
        "Painty",
        {reservedPath});
    CHECK(uniquePath == outputDirectory / "Site_1_Painty_3.mp4");

    std::filesystem::remove(firstPath);
    std::filesystem::remove(secondPath);
    std::filesystem::remove(outputDirectory / "Site_1_Painty_3.mp4");
    std::filesystem::remove(outputDirectory);
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

TEST_CASE("Fast preview MP4 smoothing fills sparse transparent point gaps", "[output][video]") {
    const auto zero = Imath::half{0.0F}.bits();
    const auto one = Imath::half{1.0F}.bits();

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 3;
    image.height = 1;
    image.rgbaHalf = {
        one,
        zero,
        zero,
        one,
        zero,
        zero,
        zero,
        zero,
        one,
        zero,
        zero,
        one,
    };
    image.depth = {2.0F, 0.0F, 2.0F};

    invisible_places::output::Mp4SparsePointSmoothingSettings disabledSmoothing;
    disabledSmoothing.enabled = false;
    const auto unsmoothedBytes =
        invisible_places::output::ConvertHalfRgbaToSrgbRgba8(image, disabledSmoothing);
    REQUIRE(unsmoothedBytes.size() == 12U);
    CHECK(unsmoothedBytes[4] == 0U);
    CHECK(unsmoothedBytes[7] == 0U);

    const auto smoothedBytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(image);
    REQUIRE(smoothedBytes.size() == 12U);
    CHECK(smoothedBytes[4] > 200U);
    CHECK(smoothedBytes[5] == 0U);
    CHECK(smoothedBytes[6] == 0U);
    CHECK(smoothedBytes[7] == 0U);
}

TEST_CASE("Fast preview MP4 conversion downsamples supersampled half-float frames", "[output][video]") {
    const auto zero = Imath::half{0.0F}.bits();
    const auto one = Imath::half{1.0F}.bits();

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 2;
    image.height = 2;
    image.rgbaHalf = {
        one,
        zero,
        zero,
        one,
        zero,
        one,
        zero,
        one,
        zero,
        zero,
        one,
        one,
        one,
        one,
        one,
        one,
    };
    image.depth = {1.0F, 1.0F, 1.0F, 1.0F};

    invisible_places::output::Mp4SparsePointSmoothingSettings disabledSmoothing;
    disabledSmoothing.enabled = false;
    const auto bytes =
        invisible_places::output::ConvertHalfRgbaToSrgbRgba8(image, 1, 1, disabledSmoothing);
    REQUIRE(bytes.size() == 4U);
    CHECK(bytes[0] == 188U);
    CHECK(bytes[1] == 188U);
    CHECK(bytes[2] == 188U);
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
    CHECK(header.find("N.X") == std::string::npos);
    CHECK(header.find("albedo.R") == std::string::npos);

    std::filesystem::remove(outputPath);
}

TEST_CASE("EXR writer emits optional Houdini denoise AOV channels", "[output][exr]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_gpu_aovs.exr";

    const auto zero = Imath::half{0.0F}.bits();
    const auto one = Imath::half{1.0F}.bits();
    const auto negativeOne = Imath::half{-1.0F}.bits();
    const auto half = Imath::half{0.5F}.bits();

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 1;
    image.height = 1;
    image.rgbaHalf = {
        one,
        half,
        zero,
        one,
    };
    image.normalHalf = {
        negativeOne,
        zero,
        one,
    };
    image.albedoHalf = {
        zero,
        half,
        one,
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
    CHECK(header.find("N.X") != std::string::npos);
    CHECK(header.find("N.Y") != std::string::npos);
    CHECK(header.find("N.Z") != std::string::npos);
    CHECK(header.find("albedo.R") != std::string::npos);
    CHECK(header.find("albedo.G") != std::string::npos);
    CHECK(header.find("albedo.B") != std::string::npos);

    std::filesystem::remove(outputPath);
}

TEST_CASE("EXR writer rejects incorrectly sized optional AOV buffers", "[output][exr]") {
    invisible_places::output::HalfRgbaExrImage image;
    image.width = 1;
    image.height = 1;
    image.rgbaHalf = {
        Imath::half{1.0F}.bits(),
        Imath::half{1.0F}.bits(),
        Imath::half{1.0F}.bits(),
        Imath::half{1.0F}.bits(),
    };
    image.normalHalf = {
        Imath::half{0.0F}.bits(),
        Imath::half{1.0F}.bits(),
    };
    image.depth = {1.0F};

    std::string errorMessage;
    CHECK_FALSE(invisible_places::output::WriteExrImage(
        image,
        std::filesystem::temp_directory_path() / "invisible_places_bad_aovs.exr",
        &errorMessage));
    CHECK(errorMessage.find("buffers do not match") != std::string::npos);
}

TEST_CASE("Eye-dome lighting leaves flat depth unchanged", "[output][edl]") {
    const std::vector<float> depth(9, 4.0F);
    const auto shade = invisible_places::output::ComputeEyeDomeLightingShade(
        depth.data(),
        3,
        3,
        1,
        1,
        invisible_places::output::EyeDomeLightingSettings{.enabled = true});
    CHECK(shade == Catch::Approx(1.0F));
}

TEST_CASE("Eye-dome lighting darkens foreground depth edges", "[output][edl]") {
    const std::vector<float> depth{
        8.0F,
        8.0F,
        8.0F,
        8.0F,
        2.0F,
        8.0F,
        8.0F,
        8.0F,
        8.0F,
    };
    const auto shade = invisible_places::output::ComputeEyeDomeLightingShade(
        depth.data(),
        3,
        3,
        1,
        1,
        invisible_places::output::EyeDomeLightingSettings{.enabled = true});
    CHECK(shade < 1.0F);
    CHECK(shade >= 0.35F);
}

TEST_CASE("Eye-dome lighting supports fractional export thickness", "[output][edl]") {
    const std::vector<float> depth{
        8.0F,
        8.0F,
        8.0F,
        8.0F,
        2.0F,
        8.0F,
        8.0F,
        8.0F,
        8.0F,
    };
    const auto halfShade = invisible_places::output::ComputeEyeDomeLightingShade(
        depth.data(),
        3,
        3,
        1,
        1,
        invisible_places::output::EyeDomeLightingSettings{
            .enabled = true,
            .minShade = 0.0F,
            .outlineThicknessPixels = 0.5F});
    const auto fullShade = invisible_places::output::ComputeEyeDomeLightingShade(
        depth.data(),
        3,
        3,
        1,
        1,
        invisible_places::output::EyeDomeLightingSettings{
            .enabled = true,
            .minShade = 0.0F,
            .outlineThicknessPixels = 1.0F});
    CHECK(halfShade < 1.0F);
    CHECK(halfShade > fullShade);
}

TEST_CASE("Eye-dome lighting thickness expands the shaded outline radius", "[output][edl]") {
    const std::vector<float> depth{
        2.0F,
        2.0F,
        8.0F,
        2.0F,
        2.0F,
    };
    const auto thinShade = invisible_places::output::ComputeEyeDomeLightingShade(
        depth.data(),
        5,
        1,
        0,
        0,
        invisible_places::output::EyeDomeLightingSettings{.enabled = true});
    const auto thickShade = invisible_places::output::ComputeEyeDomeLightingShade(
        depth.data(),
        5,
        1,
        0,
        0,
        invisible_places::output::EyeDomeLightingSettings{.enabled = true, .outlineThicknessPixels = 2.0F});
    CHECK(thinShade == Catch::Approx(1.0F));
    CHECK(thickShade < 1.0F);
}

TEST_CASE("Eye-dome lighting ignores invalid background center depth", "[output][edl]") {
    const std::vector<float> depth{
        2.0F,
        2.0F,
        2.0F,
        2.0F,
        0.0F,
        2.0F,
        2.0F,
        2.0F,
        2.0F,
    };
    const auto shade = invisible_places::output::ComputeEyeDomeLightingShade(
        depth.data(),
        3,
        3,
        1,
        1,
        invisible_places::output::EyeDomeLightingSettings{.enabled = true});
    CHECK(shade == Catch::Approx(1.0F));
}

TEST_CASE("Eye-dome lighting preserves alpha and depth while shading beauty", "[output][edl]") {
    invisible_places::output::HalfRgbaExrImage image;
    image.width = 3;
    image.height = 3;
    image.rgbaHalf.resize(3U * 3U * 4U, Imath::half{1.0F}.bits());
    image.depth = {
        8.0F,
        8.0F,
        8.0F,
        8.0F,
        2.0F,
        8.0F,
        8.0F,
        8.0F,
        8.0F,
    };
    const auto originalDepth = image.depth;
    const auto originalAlpha = image.rgbaHalf[(4U * 4U) + 3U];
    const auto originalRed = image.rgbaHalf[4U * 4U];

    invisible_places::output::ApplyEyeDomeLighting(
        &image,
        invisible_places::output::EyeDomeLightingSettings{.enabled = true});

    CHECK(image.depth == originalDepth);
    CHECK(image.rgbaHalf[(4U * 4U) + 3U] == originalAlpha);
    CHECK(image.rgbaHalf[4U * 4U] < originalRed);
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

TEST_CASE("Offline point stylisation modes alter color while preserving image shape", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFF8F6734U};
    cloud.hasSourceRgb = true;

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 0.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderWithStyle = [&](invisible_places::renderer::pointcloud::PointCloudStyleState style,
                               float stylisationTimeSeconds = 0.0F) {
        style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
        style.solidColor = {0.35F, 0.62F, 0.91F, 1.0F};
        style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
        invisible_places::style::SetScalarConstant(&style.pointSize, 7.0F);
        invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
        invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
        invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
        const invisible_places::output::OfflinePointLayer layer{
            .cloud = &cloud,
            .style = style,
            .hasSourceRgb = true,
            .localToWorld = glm::mat4{1.0F},
        };

        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 11, 11);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 11, 11},
            &image,
            nullptr,
            nullptr,
            stylisationTimeSeconds);
        return image;
    };

    auto imageDifference = [](const invisible_places::output::ExrImage& left,
                              const invisible_places::output::ExrImage& right) {
        CHECK(left.beautyR.size() == right.beautyR.size());
        float difference = 0.0F;
        const auto pixelCount = std::min(left.beautyR.size(), right.beautyR.size());
        for (std::size_t index = 0; index < pixelCount; ++index) {
            difference += std::abs(left.beautyR[index] - right.beautyR[index]);
            difference += std::abs(left.beautyG[index] - right.beautyG[index]);
            difference += std::abs(left.beautyB[index] - right.beautyB[index]);
            difference += std::abs(left.alpha[index] - right.alpha[index]);
        }
        return difference;
    };

    invisible_places::renderer::pointcloud::PointCloudStyleState baseStyle;
    const auto baseImage = renderWithStyle(baseStyle);

    auto nprStyle = baseStyle;
    nprStyle.stylisationMode =
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::NprStylisation;
    nprStyle.nprPreset = invisible_places::renderer::pointcloud::PointCloudNprPreset::Cartoon;
    nprStyle.stylisationStrength = 1.0F;
    nprStyle.stylisationColorLevels = 2.0F;
    nprStyle.stylisationInkStrength = 0.75F;
    const auto nprImage = renderWithStyle(nprStyle);

    auto brushStyle = baseStyle;
    brushStyle.stylisationMode =
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::BrushParticles;
    brushStyle.nprPreset = invisible_places::renderer::pointcloud::PointCloudNprPreset::Watercolor;
    brushStyle.stylisationStrength = 1.0F;
    brushStyle.stylisationPaperGrain = 1.0F;
    brushStyle.stylisationPigmentBleed = 0.8F;
    brushStyle.brushAspect = 3.0F;
    brushStyle.strokeJitter = 0.35F;
    brushStyle.strokeOpacityVariance = 1.0F;
    const auto brushImage = renderWithStyle(brushStyle);

    auto animatedWatercolorStyle = brushStyle;
    animatedWatercolorStyle.stylisationPaperGrain = 1.0F;
    animatedWatercolorStyle.pigmentVariation = 1.0F;
    animatedWatercolorStyle.pigmentAnimationSpeed = 2.0F;
    animatedWatercolorStyle.granulationAngleStrength = 0.0F;
    const auto animatedFrameA = renderWithStyle(animatedWatercolorStyle, 0.0F);
    const auto animatedFrameB = renderWithStyle(animatedWatercolorStyle, 0.25F);

    REQUIRE(baseImage.width == nprImage.width);
    REQUIRE(baseImage.height == brushImage.height);
    CHECK(imageDifference(baseImage, nprImage) > 0.01F);
    CHECK(imageDifference(baseImage, brushImage) > 0.01F);
    CHECK(animatedFrameA.width == animatedFrameB.width);
    CHECK(animatedFrameA.height == animatedFrameB.height);
    CHECK(imageDifference(animatedFrameA, animatedFrameB) > 0.001F);
    for (const auto alpha : brushImage.alpha) {
        CHECK(std::isfinite(alpha));
        CHECK(alpha >= 0.0F);
        CHECK(alpha <= 1.0F);
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

TEST_CASE("Offline screen sprites can use world millimeter size by camera depth", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {
        {-1.2F, -1.0F, 0.0F},
        {1.2F, 3.0F, 0.0F},
    };
    cloud.packedColors = {0xFFFFFFFFU, 0xFFFFFFFFU};
    cloud.hasSourceRgb = true;

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 0.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.fovDegrees = 45.0F;
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode = invisible_places::renderer::pointcloud::PointCloudGeometryMode::ScreenSprites;
    style.screenSpriteSizeMode =
        invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode::WorldMillimeters;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.5F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 96, 96);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0, 0, 96, 96},
        &image);

    std::size_t nearCoveredPixels = 0;
    std::size_t farCoveredPixels = 0;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        for (std::uint32_t x = 0; x < image.width; ++x) {
            const auto index = static_cast<std::size_t>(y) * image.width + x;
            if (image.alpha[index] <= 0.0F) {
                continue;
            }
            if (x < image.width / 2U) {
                ++nearCoveredPixels;
            } else {
                ++farCoveredPixels;
            }
        }
    }

    CHECK(nearCoveredPixels > 0);
    CHECK(farCoveredPixels > 0);
    CHECK(nearCoveredPixels > farCoveredPixels * 2U);
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

TEST_CASE("Offline point solid centres can reach opaque alpha", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFFFFFFFFU};
    cloud.hasSourceRgb = true;

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 2.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderWithSolidCenters = [&](bool solidCenters) {
        invisible_places::renderer::pointcloud::PointCloudStyleState style;
        style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
        style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
        style.solidCenters = solidCenters;
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

    const auto feathered = renderWithSolidCenters(false);
    const auto solid = renderWithSolidCenters(true);
    const auto center = static_cast<std::size_t>(4) * 9U + 4U;
    CHECK(feathered.alpha[center] < 1.0F);
    CHECK(solid.alpha[center] == Catch::Approx(1.0F).margin(1.0e-5F));
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

    invisible_places::renderer::pointcloud::PointCloudStyleState customGradient = baseline;
    customGradient.colorMode =
        invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap;
    customGradient.colormap =
        invisible_places::renderer::pointcloud::PointCloudColormapId::CustomGradient;
    customGradient.gradientStartColor = {1.0F, 0.0F, 0.0F};
    customGradient.gradientEndColor = {0.0F, 0.0F, 1.0F};
    invisible_places::style::ConfigureFieldMapFromStats(
        &customGradient.colormapPosition,
        0,
        "Value",
        0.0F,
        1.0F,
        &cloud.scalarFields.front());
    const auto customGradientImage = renderWithStyle(customGradient);
    CHECK(customGradientImage.beautyR[center] < 0.05F);
    CHECK(customGradientImage.beautyG[center] < 0.05F);
    CHECK(customGradientImage.beautyB[center] > 0.95F);
}

TEST_CASE("Offline water streaks follow projected flow tangent", "[output][offline][water]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {
        {0.0F, 0.0F, -0.25F},
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.25F},
        {0.0F, 0.0F, 0.0F},
    };
    cloud.hasSourceRgb = false;
    constexpr std::size_t kFieldCount = 13U;
    cloud.scalarFields.reserve(kFieldCount);
    for (std::size_t fieldIndex = 0; fieldIndex < kFieldCount; ++fieldIndex) {
        cloud.scalarFields.push_back({
            .name = "Field" + std::to_string(fieldIndex),
            .minimum = 0.0F,
            .maximum = 1.0F,
            .count = cloud.positions.size(),
            .valid = true,
        });
    }
    cloud.scalarFieldValues.assign(kFieldCount * cloud.positions.size(), 0.0F);
    auto setField = [&cloud](std::size_t fieldSlot, std::size_t pointIndex, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, pointIndex)] = value;
    };
    setField(3U, 3U, 0.5F);
    setField(4U, 3U, 0.02F);
    setField(5U, 3U, 0.0F);
    setField(9U, 3U, 1.0F);
    setField(10U, 3U, 0.0F);
    setField(11U, 3U, 3.0F);
    setField(12U, 3U, 0.5F);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterStreakAspect = 8.0F;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.08F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = false,
        .localToWorld = glm::mat4{1.0F},
    };

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 0.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.fovDegrees = 45.0F;
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 96, 96);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0, 0, 96, 96},
        &image);

    std::uint32_t minX = image.width;
    std::uint32_t maxX = 0;
    std::uint32_t minY = image.height;
    std::uint32_t maxY = 0;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        for (std::uint32_t x = 0; x < image.width; ++x) {
            const auto index = static_cast<std::size_t>(y) * image.width + x;
            if (image.alpha[index] <= 0.01F) {
                continue;
            }
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
        }
    }

    REQUIRE(minX <= maxX);
    REQUIRE(minY <= maxY);
    const auto width = maxX - minX + 1U;
    const auto height = maxY - minY + 1U;
    CHECK(height > width * 3U);
}

TEST_CASE("Offline water stream overlays use stream tangent and world length", "[output][offline][water][v2]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.hasSourceRgb = false;
    cloud.bounds.Expand(cloud.positions.front());

    const std::vector<std::string> streamFields{
        "stream_id",
        "source_id",
        "path_id",
        "branch_id",
        "stream_seed",
        "point_seed",
        "stream_distance",
        "stream_length",
        "point_age",
        "stream_age",
        "stream_speed",
        "stream_width",
        "stream_world_length",
        "stream_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z"};
    cloud.scalarFields.reserve(streamFields.size());
    for (const auto& name : streamFields) {
        cloud.scalarFields.push_back({
            .name = name,
            .minimum = 0.0F,
            .maximum = 1.0F,
            .count = cloud.positions.size(),
            .valid = true,
        });
    }
    cloud.scalarFieldValues.assign(streamFields.size() * cloud.positions.size(), 0.0F);
    auto setField = [&cloud](std::size_t fieldSlot, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, 0)] = value;
    };
    setField(8U, 0.0F);
    setField(11U, 0.080F);
    setField(12U, 1.20F);
    setField(13U, 1.0F);
    setField(14U, 1.0F);
    setField(16U, 1.0F);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterStreamOverlay = true;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.01F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = false,
        .localToWorld = glm::mat4{1.0F},
    };

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 0.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.fovDegrees = 45.0F;
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 128, 128);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        cameraState,
        invisible_places::output::OfflineRenderTile{0, 0, 128, 128},
        &image);

    std::uint32_t minX = image.width;
    std::uint32_t maxX = 0;
    std::uint32_t minY = image.height;
    std::uint32_t maxY = 0;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        for (std::uint32_t x = 0; x < image.width; ++x) {
            const auto index = static_cast<std::size_t>(y) * image.width + x;
            if (image.alpha[index] <= 0.01F) {
                continue;
            }
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
        }
    }

    REQUIRE(minX <= maxX);
    REQUIRE(minY <= maxY);
    const auto width = maxX - minX + 1U;
    const auto height = maxY - minY + 1U;
    CHECK(width > height * 3U);
}

TEST_CASE("Offline water stream overlays animate through time playback", "[output][offline][water][v2]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.hasSourceRgb = false;
    cloud.bounds.Expand(cloud.positions.front());

    const std::vector<std::string> streamFields{
        "stream_id",
        "source_id",
        "path_id",
        "branch_id",
        "stream_seed",
        "point_seed",
        "stream_distance",
        "stream_length",
        "point_age",
        "stream_age",
        "stream_speed",
        "stream_width",
        "stream_world_length",
        "stream_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z"};
    cloud.scalarFields.reserve(streamFields.size());
    for (const auto& name : streamFields) {
        cloud.scalarFields.push_back({
            .name = name,
            .minimum = 0.0F,
            .maximum = 1.0F,
            .count = cloud.positions.size(),
            .valid = true,
        });
    }
    cloud.scalarFieldValues.assign(streamFields.size() * cloud.positions.size(), 0.0F);
    auto setField = [&cloud](std::size_t fieldSlot, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, 0)] = value;
    };
    setField(5U, 0.0F);
    setField(8U, 0.0F);
    setField(10U, 6.0F);
    setField(11U, 0.10F);
    setField(12U, 1.10F);
    setField(13U, 1.0F);
    setField(14U, 1.0F);
    setField(16U, 1.0F);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterStreamOverlay = true;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.01F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = false,
        .localToWorld = glm::mat4{1.0F},
    };

    invisible_places::camera::CameraState cameraState;
    cameraState.position = {0.0F, -5.0F, 0.0F};
    cameraState.target = {0.0F, 0.0F, 0.0F};
    cameraState.fovDegrees = 45.0F;
    cameraState.nearPlane = 0.1F;
    cameraState.farPlane = 20.0F;
    WriteLookAtOrientation(&cameraState);

    auto renderAlphaSum = [&](float timeSeconds) {
        invisible_places::output::ExrImage image;
        invisible_places::output::InitializeExrImage(&image, 128, 128);
        invisible_places::output::RenderPointCloudTile(
            {layer},
            cameraState,
            invisible_places::output::OfflineRenderTile{0, 0, 128, 128},
            &image,
            nullptr,
            nullptr,
            timeSeconds);
        return std::accumulate(image.alpha.begin(), image.alpha.end(), 0.0F);
    };

    const float frameAAlpha = renderAlphaSum(0.0F);
    const float frameBAlpha = renderAlphaSum(0.5F);
    CHECK(frameAAlpha > 0.0F);
    CHECK(frameBAlpha > 0.0F);
    CHECK(std::abs(frameAAlpha - frameBAlpha) > 0.01F);
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

TEST_CASE("Point-cloud defaults choose the fastest preview path", "[pointcloud][style]") {
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
    using invisible_places::renderer::pointcloud::PointCloudGeometryMode;
    using invisible_places::renderer::pointcloud::PointCloudMaterialVariant;
    using invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::PointCloudStylisationMode;
    using invisible_places::renderer::pointcloud::ResolvePointCloudMaterialVariant;
    using invisible_places::style::ScalarConstant;

    const PointCloudStyleState style;

    CHECK(style.geometryMode == PointCloudGeometryMode::ScreenSprites);
    CHECK(style.screenSpriteSizeMode == PointCloudScreenSpriteSizeMode::Pixels);
    CHECK(style.falloffProfile == PointCloudFalloffProfile::HardDisc);
    CHECK(style.stylisationMode == PointCloudStylisationMode::Off);
    CHECK(ScalarConstant(style.pointSize) == Catch::Approx(1.0F));
    CHECK(ScalarConstant(style.opacity) == Catch::Approx(1.0F));
    CHECK(ScalarConstant(style.emissiveStrength) == Catch::Approx(0.0F));
    CHECK(ScalarConstant(style.xrayStrength) == Catch::Approx(0.0F));
    CHECK(ScalarConstant(style.depthFade) == Catch::Approx(0.0F));
    CHECK(style.colorizeAmount == Catch::Approx(0.0F));
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::OpaqueHardDisc);

    auto movingStyle = style;
    movingStyle.roughnessMotionStrength = 0.02F;
    CHECK(ResolvePointCloudMaterialVariant(movingStyle) == PointCloudMaterialVariant::Unified);
}

TEST_CASE("World sprite diameter projects to depth-adaptive point pixels", "[pointcloud][style]") {
    using invisible_places::renderer::pointcloud::WorldDiameterToScreenPointSizePixels;

    const float nearPixels = WorldDiameterToScreenPointSizePixels(0.01F, 1.0F, 2.0F, 1000.0F);
    const float farPixels = WorldDiameterToScreenPointSizePixels(0.01F, 10.0F, 2.0F, 1000.0F);
    CHECK(nearPixels == Catch::Approx(10.0F));
    CHECK(farPixels == Catch::Approx(1.0F));
    CHECK(nearPixels > farPixels);
    CHECK(WorldDiameterToScreenPointSizePixels(0.01F, 1.0F, -2.0F, 1000.0F) == Catch::Approx(nearPixels));
    CHECK(WorldDiameterToScreenPointSizePixels(-0.01F, 1.0F, 2.0F, 1000.0F) == Catch::Approx(0.0F));
}

TEST_CASE("Fast Basic point-cloud style override keeps cheap colour controls", "[pointcloud][style]") {
    using invisible_places::renderer::pointcloud::MakeFastBasicPointCloudStyle;
    using invisible_places::renderer::pointcloud::PointCloudColorMode;
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
    using invisible_places::renderer::pointcloud::PointCloudGeometryMode;
    using invisible_places::renderer::pointcloud::PointCloudMaterialVariant;
    using invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::PointCloudStylisationMode;
    using invisible_places::renderer::pointcloud::ResolvePointCloudMaterialVariant;
    using invisible_places::style::ConfigureFieldMapFromStats;
    using invisible_places::io::ScalarFieldStats;
    using invisible_places::style::ScalarConstant;
    using invisible_places::style::SetScalarConstant;

    const ScalarFieldStats scalarField{.name = "Height", .minimum = 0.0F, .maximum = 10.0F, .count = 8U, .valid = true};
    PointCloudStyleState style;
    style.geometryMode = PointCloudGeometryMode::WorldSurfels;
    style.colorMode = PointCloudColorMode::ScalarColormap;
    style.stylisationMode = PointCloudStylisationMode::NprStylisation;
    style.colorizeAmount = 0.8F;
    ConfigureFieldMapFromStats(&style.colormapPosition, 0, scalarField.name, 0.0F, 1.0F, &scalarField);
    style.flowAnimation = true;
    SetScalarConstant(&style.pointSize, 12.0F);
    SetScalarConstant(&style.opacity, 0.35F);
    SetScalarConstant(&style.emissiveStrength, 2.0F);
    SetScalarConstant(&style.xrayStrength, 0.5F);
    SetScalarConstant(&style.depthFade, 0.5F);

    const auto fast = MakeFastBasicPointCloudStyle(style, true);

    CHECK(fast.geometryMode == PointCloudGeometryMode::ScreenSprites);
    CHECK(fast.colorMode == PointCloudColorMode::ScalarColormap);
    CHECK(fast.falloffProfile == PointCloudFalloffProfile::HardDisc);
    CHECK(fast.stylisationMode == PointCloudStylisationMode::Off);
    CHECK_FALSE(fast.flowAnimation);
    CHECK(ScalarConstant(fast.pointSize) == Catch::Approx(1.0F));
    CHECK(ScalarConstant(fast.opacity) == Catch::Approx(1.0F));
    CHECK(ScalarConstant(fast.emissiveStrength) == Catch::Approx(0.0F));
    CHECK(ScalarConstant(fast.xrayStrength) == Catch::Approx(0.0F));
    CHECK(ScalarConstant(fast.depthFade) == Catch::Approx(0.0F));
    CHECK(fast.colorizeAmount == Catch::Approx(0.8F));
    CHECK(fast.colormapPosition.active);
    CHECK(fast.colormapPosition.fieldMap.fieldName == "Height");
    CHECK(ResolvePointCloudMaterialVariant(fast) == PointCloudMaterialVariant::Unified);

    auto sourceRgbStyle = style;
    sourceRgbStyle.colorMode = PointCloudColorMode::SourceRgb;
    const auto fastWithRgb = MakeFastBasicPointCloudStyle(sourceRgbStyle, true);
    CHECK(fastWithRgb.colorMode == PointCloudColorMode::SourceRgb);

    const auto fastWithoutRgb = MakeFastBasicPointCloudStyle(sourceRgbStyle, false);
    CHECK(fastWithoutRgb.colorMode == PointCloudColorMode::SolidColor);
    CHECK(fastWithoutRgb.colorizeAmount == Catch::Approx(0.8F));

    PointCloudStyleState worldSizedScreenStyle;
    worldSizedScreenStyle.screenSpriteSizeMode = PointCloudScreenSpriteSizeMode::WorldMillimeters;
    SetScalarConstant(&worldSizedScreenStyle.surfelDiameter, 0.012F);
    const auto fastWorldSized = MakeFastBasicPointCloudStyle(worldSizedScreenStyle, true);
    CHECK(fastWorldSized.screenSpriteSizeMode == PointCloudScreenSpriteSizeMode::WorldMillimeters);
    CHECK(ScalarConstant(fastWorldSized.surfelDiameter) == Catch::Approx(0.012F));
}

TEST_CASE("Point-cloud material variant separates opaque, simple, and unified styles", "[pointcloud][style]") {
    using invisible_places::renderer::pointcloud::PointCloudColorMode;
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
    using invisible_places::renderer::pointcloud::PointCloudMaterialVariant;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::renderer::pointcloud::ResolvePointCloudMaterialVariant;
    using invisible_places::style::ConfigureFieldMapFromStats;
    using invisible_places::style::SetScalarConstant;

    PointCloudStyleState style;
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::OpaqueHardDisc);

    style.falloffProfile = PointCloudFalloffProfile::SoftDisc;
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::ConstantSimple);

    style.falloffProfile = PointCloudFalloffProfile::HardDisc;
    SetScalarConstant(&style.emissiveStrength, 0.5F);
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::ConstantSimple);

    SetScalarConstant(&style.emissiveStrength, 0.0F);
    style.colorMode = PointCloudColorMode::ScalarColormap;
    invisible_places::io::ScalarFieldStats scalarField;
    scalarField.name = "height";
    scalarField.minimum = 0.0F;
    scalarField.maximum = 1.0F;
    scalarField.count = 2;
    scalarField.valid = true;
    ConfigureFieldMapFromStats(&style.colormapPosition, 0, scalarField.name, 0.0F, 1.0F, &scalarField);
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::Unified);
}
