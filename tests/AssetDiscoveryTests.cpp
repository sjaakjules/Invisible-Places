#include "InvisiblePlacesBuildConfig.hpp"
#include "app/ManualFlowPathEditMath.hpp"
#include "app/PointVisualSelection.hpp"
#include "app/WaterPathDiagnostics.hpp"
#include "camera/AnimationPath.hpp"
#include "camera/CameraPath.hpp"
#include "camera/CameraShot.hpp"
#include "camera/OrbitCamera.hpp"
#include "io/AssetDiscovery.hpp"
#include "io/GaussianSplatData.hpp"
#include "io/MeshData.hpp"
#include "io/PointCloudData.hpp"
#include "io/PlyHeader.hpp"
#include "io/TransformMatrix.hpp"
#include "output/ExrWriter.hpp"
#include "output/EyeDomeLighting.hpp"
#include "output/HoudiniCameraExport.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "output/PngWriter.hpp"
#include "output/RenderPreset.hpp"
#include "output/VideoWriter.hpp"
#include "platform/Window.hpp"
#include "platform/WindowTitle.hpp"
#include "platform/VulkanRuntimeConfig.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/gsplat/HighQualityGaussianScene.hpp"
#include "renderer/core/VulkanViewportShell.hpp"
#include "renderer/pointcloud/Colormap.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "scene/PointCloudVariants.hpp"
#include "scene/SceneCatalog.hpp"
#include "serialization/ProjectDocument.hpp"
#include "serialization/ProjectDocumentJson.hpp"
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

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <stop_token>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

std::filesystem::path DataRoot() {
    return std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
}

template <typename T>
void WriteBinaryValue(std::ofstream* output, const T& value) {
    output->write(reinterpret_cast<const char*>(&value), static_cast<std::streamsize>(sizeof(T)));
}

void WriteSyntheticTriangleMeshPly(
    const std::filesystem::path& path,
    const std::vector<invisible_places::io::Float3>& vertices,
    const std::vector<std::array<std::uint32_t, 3>>& faces,
    const std::vector<invisible_places::io::Float3>& normals = {}) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    output << "ply\n";
    output << "format binary_little_endian 1.0\n";
    output << "comment synthetic mesh fixture\n";
    output << "element vertex " << vertices.size() << "\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    if (!normals.empty()) {
        output << "property float nx\n";
        output << "property float ny\n";
        output << "property float nz\n";
    }
    output << "element face " << faces.size() << "\n";
    output << "property list uchar int vertex_indices\n";
    output << "end_header\n";
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        WriteBinaryValue(&output, vertices[index].x);
        WriteBinaryValue(&output, vertices[index].y);
        WriteBinaryValue(&output, vertices[index].z);
        if (!normals.empty()) {
            WriteBinaryValue(&output, normals[index].x);
            WriteBinaryValue(&output, normals[index].y);
            WriteBinaryValue(&output, normals[index].z);
        }
    }
    for (const auto& face : faces) {
        const std::uint8_t count = 3U;
        WriteBinaryValue(&output, count);
        for (const auto index : face) {
            const auto signedIndex = static_cast<std::int32_t>(index);
            WriteBinaryValue(&output, signedIndex);
        }
    }
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

void WriteTinyPointCloudPly(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output{path};
    output << "ply\n"
           << "format ascii 1.0\n"
           << "element vertex 3\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "end_header\n"
           << "0 0 0 255 0 0\n"
           << "1 0 0 0 255 0\n"
           << "0 1 0 0 0 255\n";
}

void WriteTinySampledGroundPointCloudPly(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output{path};
    output << "ply\n"
           << "format ascii 1.0\n"
           << "element vertex 1\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property float nx\n"
           << "property float ny\n"
           << "property float nz\n"
           << "property float scalar_dip\n"
           << "property float scalar_dip_direction\n"
           << "end_header\n"
           << "0 0 0 0 0 1 12 180\n";
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

TEST_CASE("SampleScene validates local multi-cloud shoreline fixture", "[discovery][scene][sample]") {
    const auto sampleRoot = DataRoot() / "SampleScene";
    if (!std::filesystem::exists(sampleRoot)) {
        SKIP("SampleScene fixture is not present in the local Data directory.");
    }

    const auto catalog = invisible_places::io::DiscoverAssets(DataRoot());
    const std::set<std::string> requiredFilenames{
        "Site1-Mesh-SampleScene.ply",
        "Site1-MeshSampled-5mm-SampleScene.ply",
        "Site1-ROCK-1mm. SampleScene.ply",
        "Site1-ROCK-2mm. SampleScene.ply",
        "Site1-ROCK-3mm. SampleScene.ply",
        "Site1-ROCK-5mm. SampleScene.ply",
        "Site1-SAND-1mm. SampleScene.ply",
        "Site1-SAND-2mm. SampleScene.ply",
        "Site1-SAND-3mm. SampleScene.ply",
        "Site1-SAND-5mm. SampleScene.ply",
        "Site1-VEG-1mm. SampleScene.ply",
        "Site1-VEG-2mm. SampleScene.ply",
        "Site1-VEG-3mm. SampleScene.ply",
        "Site1-VEG-5mm. SampleScene.ply",
    };
    std::vector<invisible_places::io::PointCloudAsset> sampleAssets;
    std::copy_if(
        catalog.pointClouds.begin(),
        catalog.pointClouds.end(),
        std::back_inserter(sampleAssets),
        [&requiredFilenames](const auto& asset) {
            return asset.filePath.parent_path().filename() == "SampleScene" &&
                   requiredFilenames.contains(asset.filePath.filename().string());
        });

    REQUIRE(sampleAssets.size() == requiredFilenames.size());
    std::set<std::string> roles;
    std::set<std::string> filenames;
    bool foundMeshFixture = false;
    bool foundSampledGroundFixture = false;
    for (const auto& asset : sampleAssets) {
        filenames.insert(asset.filePath.filename().string());
        CHECK(asset.header.LooksLikePointCloud());
        CHECK(asset.header.HasColorRgb());
        CHECK(asset.header.HasProperty("nx"));
        CHECK(asset.header.HasProperty("ny"));
        CHECK(asset.header.HasProperty("nz"));

        if (asset.filePath.filename() == "Site1-Mesh-SampleScene.ply") {
            foundMeshFixture = true;
            CHECK(asset.sceneRole.empty());
            CHECK(asset.header.faceCount > 0U);
            CHECK(asset.header.vertexCount > 500'000ULL);
            continue;
        }
        if (asset.filePath.filename() ==
            "Site1-MeshSampled-5mm-SampleScene.ply") {
            foundSampledGroundFixture = true;
            CHECK(asset.sceneRole.empty());
            CHECK(asset.header.faceCount == 0U);
            CHECK(asset.header.vertexCount > 2'000'000ULL);
            const auto scalarFields = asset.header.ScalarFieldNames();
            CHECK(std::any_of(
                scalarFields.begin(),
                scalarFields.end(),
                [](const std::string& name) {
                    return name.find("Dip") != std::string::npos;
                }));
            CHECK(std::any_of(
                scalarFields.begin(),
                scalarFields.end(),
                [](const std::string& name) {
                    return name.find("direction") != std::string::npos;
                }));
            continue;
        }

        roles.insert(asset.sceneRole);
        CHECK(asset.sceneGroupName == "SampleScene");
        CHECK(asset.header.vertexCount > 200'000ULL);

        const auto scalarFields = asset.header.ScalarFieldNames();
        CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Intensity") != scalarFields.end());
        CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Ranges") != scalarFields.end());
        CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Composite") != scalarFields.end());
        CHECK(std::find(scalarFields.begin(), scalarFields.end(), "ScanID") != scalarFields.end());
        const auto inferredSpacingMillimetres =
            static_cast<std::uint32_t>(std::lround(asset.inferredPointSpacingMeters * 1000.0F));
        CHECK(inferredSpacingMillimetres > 0U);
        if (asset.sceneRole == "ROCK") {
            CHECK(asset.scenePrimaryRole);
            CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Interest") != scalarFields.end());
        } else if (asset.sceneRole == "SAND") {
            CHECK_FALSE(asset.scenePrimaryRole);
            CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Roughness") != scalarFields.end());
        } else if (asset.sceneRole == "VEG") {
            CHECK_FALSE(asset.scenePrimaryRole);
            CHECK(std::find(scalarFields.begin(), scalarFields.end(), "Roughness") != scalarFields.end());
        }
    }

    CHECK(roles == std::set<std::string>{"ROCK", "SAND", "VEG"});
    CHECK(foundMeshFixture);
    CHECK(foundSampledGroundFixture);
    CHECK(filenames == requiredFilenames);
    CHECK_FALSE(filenames.contains("Site3-ROCK-1mm.Sample.ply"));
    CHECK_FALSE(filenames.contains("Site3-SAND-2mm.Sample.ply"));
    CHECK_FALSE(filenames.contains("Site3-VEG-1mm.Sample.ply"));
    CHECK_FALSE(filenames.contains("Site3-Mesh-Sample.ply"));

    const auto sceneCatalog = invisible_places::scene::SceneCatalog::FromDiscoveredAssets(catalog);
    const auto* sampleScene = sceneCatalog.FindPointCloudGroup("SampleScene");
    REQUIRE(sampleScene != nullptr);
    REQUIRE(sampleScene->completeDisplayBundles.size() == 4U);
    constexpr std::array<invisible_places::scene::PointSpacingMicrometres, 4U> expectedSpacings{
        1'000U,
        2'000U,
        3'000U,
        5'000U,
    };
    for (const auto expectedSpacing : expectedSpacings) {
        CAPTURE(expectedSpacing);
        const auto* bundle = sampleScene->FindCompleteDisplayBundle(expectedSpacing);
        REQUIRE(bundle != nullptr);
        CHECK(bundle->totalPointCount > 0U);
    }
    using invisible_places::scene::ScenePointCloudRole;
    REQUIRE(sampleScene->AnalysisSource(ScenePointCloudRole::Rock) != nullptr);
    REQUIRE(sampleScene->AnalysisSource(ScenePointCloudRole::Sand) != nullptr);
    REQUIRE(sampleScene->AnalysisSource(ScenePointCloudRole::Vegetation) != nullptr);
    CHECK(sampleScene->AnalysisSource(ScenePointCloudRole::Rock)->sourcePath.filename() ==
          "Site1-ROCK-1mm. SampleScene.ply");
    CHECK(sampleScene->AnalysisSource(ScenePointCloudRole::Sand)->sourcePath.filename() ==
          "Site1-SAND-1mm. SampleScene.ply");
    CHECK(sampleScene->AnalysisSource(ScenePointCloudRole::Vegetation)->sourcePath.filename() ==
          "Site1-VEG-1mm. SampleScene.ply");
}

TEST_CASE("SampleScene authored water controls lie on 3 mm ROCK display support",
          "[discovery][scene][sample][water][fixture]") {
    const auto sampleRoot = DataRoot() / "SampleScene";
    const auto fixturePath = DataRoot().parent_path() / "tests" / "fixtures" /
                             "sample_scene_water_sources.json";
    if (!std::filesystem::exists(sampleRoot) || !std::filesystem::exists(fixturePath)) {
        SKIP("SampleScene water fixtures are not present in the local workspace.");
    }

    const auto assetCatalog = invisible_places::io::DiscoverAssets(DataRoot());
    const auto sceneCatalog =
        invisible_places::scene::SceneCatalog::FromDiscoveredAssets(assetCatalog);
    const auto* sampleScene = sceneCatalog.FindPointCloudGroup("SampleScene");
    REQUIRE(sampleScene != nullptr);
    const auto* displayBundle = sampleScene->FindCompleteDisplayBundle(3'000U);
    REQUIRE(displayBundle != nullptr);
    const auto& rock = displayBundle->Find(
        invisible_places::scene::ScenePointCloudRole::Rock);
    REQUIRE(rock.sourcePath.filename() ==
            "Site1-ROCK-3mm. SampleScene.ply");

    const auto rockResult = invisible_places::io::LoadPointCloud(rock.sourcePath);
    REQUIRE(rockResult.success);
    REQUIRE_FALSE(rockResult.cloud.positions.empty());

    std::string fixtureError;
    const auto fixture = invisible_places::serialization::LoadWaterSourcesDocument(
        fixturePath,
        &fixtureError);
    INFO(fixtureError);
    REQUIRE(fixture.has_value());
    REQUIRE(fixture->emitters.size() == 1U);
    REQUIRE(fixture->manualFlowPaths.size() == 1U);
    REQUIRE(fixture->seepageNodes.size() == 1U);

    const auto nearestSupportDistance = [&](const invisible_places::io::Float3& authored) {
        float nearestSquared = std::numeric_limits<float>::infinity();
        for (const auto& support : rockResult.cloud.positions) {
            const float dx = authored.x - support.x;
            const float dy = authored.y - support.y;
            const float dz = authored.z - support.z;
            nearestSquared = std::min(nearestSquared, dx * dx + dy * dy + dz * dz);
        }
        return std::sqrt(nearestSquared);
    };
    const auto requireSupported = [&](std::string_view label,
                                      const invisible_places::io::Float3& position) {
        const float distance = nearestSupportDistance(position);
        INFO(label << " is " << distance << " m from 3 mm ROCK support");
        CHECK(distance <= 0.005F);
    };

    const auto& emitter = fixture->emitters.front();
    CHECK(emitter.name == "SampleFlowPoint");
    requireSupported(emitter.name, emitter.position);

    const auto& manualPath = fixture->manualFlowPaths.front();
    CHECK(manualPath.name == "SampleFlowPath");
    REQUIRE(manualPath.controlPoints.size() == 11U);
    float pathLength = 0.0F;
    for (std::size_t index = 0U; index < manualPath.controlPoints.size(); ++index) {
        requireSupported(
            manualPath.name + " control " + std::to_string(index),
            manualPath.controlPoints[index]);
        if (index != 0U) {
            const auto& previous = manualPath.controlPoints[index - 1U];
            const auto& current = manualPath.controlPoints[index];
            const float dx = current.x - previous.x;
            const float dy = current.y - previous.y;
            const float dz = current.z - previous.z;
            pathLength += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    }
    CHECK(pathLength == Catch::Approx(1.9723F).margin(0.005F));
    CHECK(manualPath.controlPoints.front().z - manualPath.controlPoints.back().z >= 0.60F);

    const auto& seepage = fixture->seepageNodes.front();
    CHECK(seepage.name == "SampleSeepage");
    requireSupported(seepage.name, seepage.position);
}

TEST_CASE("Point-cloud scene roles and millimetre spacing are inferred from filenames", "[discovery][scene]") {
    CHECK(invisible_places::io::InferPointCloudSceneRoleFromName("Site3-ROCK-1mm.ply") == "ROCK");
    CHECK(invisible_places::io::InferPointCloudSceneRoleFromName("Site3-SAND-2mm.ply") == "SAND");
    CHECK(invisible_places::io::InferPointCloudSceneRoleFromName("Site3-VEG-1mm.ply") == "VEG");
    CHECK(invisible_places::io::InferPointCloudSceneRoleFromName("Site3-ROCK-1mm.Sample.ply") == "ROCK");
    CHECK(invisible_places::io::InferPointCloudSceneRoleFromName("Site3-Mid-1mm100M.ply").empty());
    CHECK(invisible_places::io::InferPointSpacingMetersFromName("Site3-ROCK-1mm.ply") == Catch::Approx(0.001F));
    CHECK(invisible_places::io::InferPointSpacingMetersFromName("Site3-ROCK-1mm.Sample.ply") == Catch::Approx(0.001F));
    CHECK(invisible_places::io::InferPointSpacingMetersFromName("Site3-SAND-2.5mm.ply") == Catch::Approx(0.0025F));
}

TEST_CASE("Discovery groups role-named sibling PLY files by folder", "[discovery][scene]") {
    const auto root = std::filesystem::temp_directory_path() / "invisible_places_scene_grouping_test";
    std::filesystem::remove_all(root);
    WriteTinyPointCloudPly(root / "ExhibitionScene" / "Site3-ROCK-1mm.ply");
    WriteTinyPointCloudPly(root / "ExhibitionScene" / "Site3-SAND-2mm.ply");
    WriteTinyPointCloudPly(root / "Standalone-1mm.ply");

    const auto catalog = invisible_places::io::DiscoverAssets(root);

    REQUIRE(catalog.issues.empty());
    REQUIRE(catalog.pointClouds.size() == 3U);
    const auto rockIt = std::find_if(
        catalog.pointClouds.begin(),
        catalog.pointClouds.end(),
        [](const auto& asset) { return asset.sceneRole == "ROCK"; });
    const auto sandIt = std::find_if(
        catalog.pointClouds.begin(),
        catalog.pointClouds.end(),
        [](const auto& asset) { return asset.sceneRole == "SAND"; });
    REQUIRE(rockIt != catalog.pointClouds.end());
    REQUIRE(sandIt != catalog.pointClouds.end());
    CHECK(rockIt->sceneGroupName == "ExhibitionScene");
    CHECK(rockIt->scenePrimaryRole);
    CHECK(rockIt->inferredPointSpacingMeters == Catch::Approx(0.001F));
    CHECK(sandIt->sceneGroupName == "ExhibitionScene");
    CHECK_FALSE(sandIt->scenePrimaryRole);
    CHECK(sandIt->inferredPointSpacingMeters == Catch::Approx(0.002F));

    const auto standaloneIt = std::find_if(
        catalog.pointClouds.begin(),
        catalog.pointClouds.end(),
        [](const auto& asset) { return asset.filePath.filename() == "Standalone-1mm.ply"; });
    REQUIRE(standaloneIt != catalog.pointClouds.end());
    CHECK(standaloneIt->sceneGroupName.empty());
    CHECK(standaloneIt->sceneRole.empty());
    std::filesystem::remove_all(root);
}

TEST_CASE("Asset discovery skips marked staging trees", "[discovery][scene]") {
    const auto root =
        std::filesystem::temp_directory_path() /
        "invisible_places_discovery_ignore_test";
    std::filesystem::remove_all(root);
    WriteTinyPointCloudPly(root / "VisibleScene" / "Visible-SAND-1mm.ply");
    WriteTinyPointCloudPly(
        root / "RefinementStaging" / "HiddenScene" / "Hidden-SAND-1mm.ply");
    {
        std::ofstream marker{
            root / "RefinementStaging" / ".invisible_places-ignore"};
        marker << "staging\n";
    }

    const auto catalog = invisible_places::io::DiscoverAssets(root);

    REQUIRE(catalog.issues.empty());
    REQUIRE(catalog.pointClouds.size() == 1U);
    CHECK(catalog.pointClouds.front().filePath.filename() ==
          "Visible-SAND-1mm.ply");
    std::filesystem::remove_all(root);
}

TEST_CASE(
    "Sampled Ground discovery rejects ambiguous scene-local candidates",
    "[discovery][scene][ground]") {
    const auto root =
        std::filesystem::temp_directory_path() /
        "invisible_places_ambiguous_sampled_ground_test";
    std::filesystem::remove_all(root);
    const auto lexicalFirst = root / "Site1-MESHSampled-A-5mm.ply";
    const auto lexicalSecond = root / "Site1-MESHSampled-B-5mm.ply";
    WriteTinySampledGroundPointCloudPly(lexicalFirst);

    std::string errorMessage;
    const auto unique =
        invisible_places::scene::FindSampledGroundSurfaceInDirectory(
            root,
            &errorMessage);
    REQUIRE(unique.has_value());
    CHECK(unique.value() == lexicalFirst);
    CHECK(errorMessage.empty());

    WriteTinySampledGroundPointCloudPly(lexicalSecond);
    const auto ambiguous =
        invisible_places::scene::FindSampledGroundSurfaceInDirectory(
            root,
            &errorMessage);
    CHECK_FALSE(ambiguous.has_value());
    CHECK(errorMessage.find("Multiple qualifying 5 mm MESH-sampled Ground clouds") !=
          std::string::npos);
    CHECK(errorMessage.find(root.generic_string()) != std::string::npos);

    std::filesystem::remove_all(root);
}

TEST_CASE("Project layers persist scene variant spacing and shoreline wave settings", "[project][scene][water]") {
    using invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;
    using invisible_places::serialization::ProjectDocument;
    using invisible_places::serialization::ProjectLayerDocument;

    ProjectDocument document;
    document.projectName = "scene-serialization-test";
    ProjectLayerDocument layer;
    layer.sourcePath = "Data/ExhibitionScene/Site3-SAND-2mm.ply";
    layer.sceneGroupName = "ExhibitionScene";
    layer.sceneRole = "SAND";
    layer.inferredPointSpacingMeters = 0.002F;
    layer.pointSpacingMeters = 0.0024F;
    layer.pointSpacingManualOverride = true;
    layer.selectedSceneVariantPath = "Data/ExhibitionScene/Site3-SAND-2mm.ply";
    layer.loaded = true;
    layer.visible = true;
    PointCloudStyleState style;
    style.shorelineWaveEnabled = true;
    style.shorelineBoundaryZ = 1.55F;
    style.shorelineHeightReachMeters = 0.45F;
    style.shorelineEdgeFadeMeters = 0.05F;
    style.shorelineWaveAlgorithm = PointCloudShorelineWaveAlgorithm::HeightFoam;
    auto& foam = style.shorelineHeightFoam;
    foam.runupZ = 1.80F;
    foam.breakZ = 1.40F;
    foam.offshoreReachMeters = 0.80F;
    foam.edgeFadeMeters = 0.07F;
    foam.directionX = 0.60F;
    foam.directionY = 0.80F;
    foam.patternScale = 1.30F;
    foam.wavelengthMeters = 0.18F;
    foam.speed = 0.72F;
    foam.warp = 0.42F;
    foam.turbulence = 0.19F;
    foam.density = 0.68F;
    foam.phase = 0.23F;
    foam.intensity = 1.35F;
    foam.offshoreFoamStrength = 0.44F;
    foam.incomingStrength = 1.20F;
    foam.returnStrength = 0.26F;
    foam.emissionAdd = 0.90F;
    foam.opacityAdd = 0.12F;
    foam.opacityMultiply = 1.40F;
    foam.pointSizeAdd = 2.0F;
    foam.pointSizeMultiply = 1.60F;
    foam.colourMix = 0.81F;
    foam.colour = {0.70F, 0.90F, 1.0F};
    foam.seed = 77U;
    document.pointVisuals.push_back({.name = "RGB-Ghost", .style = style});
    document.selectedPointVisualName = "RGB-Ghost";
    document.layers.push_back(layer);

    const auto path = std::filesystem::temp_directory_path() / "invisible_places_scene_project_roundtrip.json";
    std::string error;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, path, &error));
    auto loaded = invisible_places::serialization::LoadProjectDocument(path, &error);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->schemaVersion >= 26U);
    REQUIRE(loaded->layers.size() == 1U);
    const auto& loadedLayer = loaded->layers.front();
    CHECK(loadedLayer.sceneGroupName == "ExhibitionScene");
    CHECK(loadedLayer.sceneRole == "SAND");
    CHECK(loadedLayer.pointSpacingMeters == Catch::Approx(0.0024F));
    CHECK(loadedLayer.pointSpacingManualOverride);
    CHECK_FALSE(loadedLayer.pointStyle.has_value());
    CHECK(loadedLayer.pointVisuals.empty());
    REQUIRE(loaded->pointVisuals.size() == 1U);
    CHECK(loaded->selectedPointVisualName == "RGB-Ghost");
    CHECK(loaded->pointVisuals[0].style.shorelineWaveEnabled);
    CHECK(loaded->pointVisuals[0].style.shorelineBoundaryZ == Catch::Approx(1.55F));
    CHECK(loaded->pointVisuals[0].style.shorelineHeightReachMeters == Catch::Approx(0.45F));
    CHECK(loaded->pointVisuals[0].style.shorelineWaveAlgorithm == PointCloudShorelineWaveAlgorithm::HeightFoam);
    const auto& loadedFoam = loaded->pointVisuals[0].style.shorelineHeightFoam;
    CHECK(loadedFoam.runupZ == Catch::Approx(1.80F));
    CHECK(loadedFoam.breakZ == Catch::Approx(1.40F));
    CHECK(loadedFoam.offshoreReachMeters == Catch::Approx(0.80F));
    CHECK(loadedFoam.edgeFadeMeters == Catch::Approx(0.07F));
    CHECK(loadedFoam.directionX == Catch::Approx(0.60F));
    CHECK(loadedFoam.directionY == Catch::Approx(0.80F));
    CHECK(loadedFoam.patternScale == Catch::Approx(1.30F));
    CHECK(loadedFoam.wavelengthMeters == Catch::Approx(0.18F));
    CHECK(loadedFoam.speed == Catch::Approx(0.72F));
    CHECK(loadedFoam.warp == Catch::Approx(0.42F));
    CHECK(loadedFoam.turbulence == Catch::Approx(0.19F));
    CHECK(loadedFoam.density == Catch::Approx(0.68F));
    CHECK(loadedFoam.phase == Catch::Approx(0.23F));
    CHECK(loadedFoam.intensity == Catch::Approx(1.35F));
    CHECK(loadedFoam.offshoreFoamStrength == Catch::Approx(0.44F));
    CHECK(loadedFoam.incomingStrength == Catch::Approx(1.20F));
    CHECK(loadedFoam.returnStrength == Catch::Approx(0.26F));
    CHECK(loadedFoam.emissionAdd == Catch::Approx(0.90F));
    CHECK(loadedFoam.opacityAdd == Catch::Approx(0.12F));
    CHECK(loadedFoam.opacityMultiply == Catch::Approx(1.40F));
    CHECK(loadedFoam.pointSizeAdd == Catch::Approx(2.0F));
    CHECK(loadedFoam.pointSizeMultiply == Catch::Approx(1.60F));
    CHECK(loadedFoam.colourMix == Catch::Approx(0.81F));
    CHECK(loadedFoam.colour[0] == Catch::Approx(0.70F));
    CHECK(loadedFoam.colour[1] == Catch::Approx(0.90F));
    CHECK(loadedFoam.colour[2] == Catch::Approx(1.0F));
    CHECK(loadedFoam.seed == 77U);
    std::filesystem::remove(path);
}

TEST_CASE("Legacy layer point visuals migrate to project library and scene temps", "[project][serialization][visuals]") {
    const auto path = std::filesystem::temp_directory_path() / "invisible_places_legacy_visual_migration.json";
    {
        std::ofstream output{path, std::ios::trunc};
        output << R"({
  "schema_version": 29,
  "project_name": "legacy-visuals",
  "layers": [
    {
      "kind": "point_cloud",
      "source_path": "Data/ExhibitionScene/Site3-ROCK-1mm.ply",
      "scene_group": "ExhibitionScene",
      "scene_role": "ROCK",
      "loaded": true,
      "visible": true,
      "selected_point_visual": "RGB-Ghost_edited",
      "point_visuals": [
        {"name": "ghosted", "point_style": {"exposure": 0.75}},
        {"name": "Roughness", "point_style": {"roughness_motion_strength": 0.02, "roughness_motion_speed": 0.5}},
        {"name": "RGB-Ghost", "point_style": {"exposure": 1.0}},
        {"name": "RGB-Ghost_edited", "point_style": {"exposure": 2.25, "water_streak_aspect": 9.0}}
      ]
    },
    {
      "kind": "point_cloud",
      "source_path": "Data/ExhibitionScene/Site3-SAND-2mm.ply",
      "scene_group": "ExhibitionScene",
      "scene_role": "SAND",
      "loaded": true,
      "visible": true,
      "selected_point_visual": "RGB-Ghost_edited",
      "point_visuals": [
        {"name": "RGB-Ghost_edited", "point_style": {
          "exposure": 1.5,
          "shoreline_wave_enabled": true,
          "shoreline_boundary_z": 1.55,
          "shoreline_intensity": 1.4
        }}
      ]
    },
    {
      "kind": "point_cloud",
      "source_path": "Data/SampleScene/Site1-ROCK-1mm. SampleScene.ply",
      "scene_group": "SampleScene",
      "scene_role": "ROCK",
      "loaded": true,
      "visible": true,
      "selected_point_visual": "Unnamed_edited",
      "point_visuals": [
        {"name": "Unnamed", "point_style": {"exposure": 0.9}},
        {"name": "Unnamed_edited", "point_style": {
          "color_mode": "solid_color",
          "solid_color": [0.1, 0.2, 0.3, 1.0],
          "exposure": 1.7
        }}
      ]
    }
  ]
})";
    }

    std::string error;
    const auto document = invisible_places::serialization::LoadProjectDocument(path, &error);
    REQUIRE(document.has_value());
    CHECK(document->schemaVersion == invisible_places::serialization::ProjectDocument{}.schemaVersion);
    CHECK(document->selectedPointVisualName == "RGB-Ghost");

    auto findVisual = [&](std::string_view name) {
        return std::find_if(
            document->pointVisuals.begin(),
            document->pointVisuals.end(),
            [name](const auto& visual) { return visual.name == name; });
    };
    const auto rgbGhost = findVisual("RGB-Ghost");
    REQUIRE(rgbGhost != document->pointVisuals.end());
    CHECK(rgbGhost->style.exposure == Catch::Approx(2.25F));
    CHECK(rgbGhost->style.waterStreakAspect == Catch::Approx(9.0F));
    CHECK(rgbGhost->style.shorelineWaveEnabled);
    CHECK(
        rgbGhost->style.shorelineWaveAlgorithm ==
        invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm::FoamFronts);
    CHECK(rgbGhost->style.shorelineBoundaryZ == Catch::Approx(1.55F));
    CHECK(findVisual("ghosted") != document->pointVisuals.end());
    CHECK(findVisual("Roughness") != document->pointVisuals.end());
    CHECK(findVisual("Unnamed") != document->pointVisuals.end());

    REQUIRE(document->sceneVisualStates.size() == 1U);
    CHECK(document->sceneVisualStates[0].sceneGroupName == "SampleScene");
    CHECK(document->sceneVisualStates[0].visual.name == "Unnamed_SampleScene");
    CHECK(
        document->sceneVisualStates[0].visual.style.colorMode ==
        invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor);
    CHECK(document->sceneVisualStates[0].visual.style.solidColor[2] == Catch::Approx(0.3F));
    CHECK(document->sceneVisualStates[0].visual.style.exposure == Catch::Approx(1.7F));

    std::filesystem::remove(path);
}

TEST_CASE("Exhibition final project selects the complete Scene3 five millimetre display", "[project][scene][water]") {
    const auto projectPath = DataRoot().parent_path() / "Saved" / "ExhibitionFinal_project.json";
    if (!std::filesystem::exists(projectPath)) {
        SKIP("Exhibition final project is not present.");
    }

    std::string error;
    const auto document = invisible_places::serialization::LoadProjectDocument(projectPath, &error);
    REQUIRE(document.has_value());
    CHECK(document->schemaVersion >= 30U);

    auto findVisual = [&](std::string_view name) {
        return std::find_if(
            document->pointVisuals.begin(),
            document->pointVisuals.end(),
            [name](const auto& visual) { return visual.name == name; });
    };
    REQUIRE_FALSE(document->selectedPointVisualName.empty());
    CHECK(findVisual(document->selectedPointVisualName) != document->pointVisuals.end());
    CHECK(findVisual("ghosted") != document->pointVisuals.end());
    CHECK(findVisual("Roughness") != document->pointVisuals.end());
    const auto rgbGhost = findVisual("RGB-Ghost");
    REQUIRE(rgbGhost != document->pointVisuals.end());
    CHECK(rgbGhost->style.waterStreakAspect >= 1.0F);

    const auto rgbScene3State = std::find_if(
        document->sceneVisualStates.begin(),
        document->sceneVisualStates.end(),
        [](const auto& state) {
            return state.sceneGroupName == "Scene3" &&
                   state.visual.name == "RGB-Ghost_Scene3";
        });
    CHECK(rgbScene3State != document->sceneVisualStates.end());

    std::vector<std::string> roles;
    std::vector<std::string> filenames;
    for (const auto& layer : document->layers) {
        CHECK_FALSE(layer.pointStyle.has_value());
        CHECK(layer.pointVisuals.empty());
        CHECK(layer.selectedPointVisualName == "Unnamed");
        if (layer.sceneGroupName == "Scene3" && layer.loaded && layer.visible &&
            (layer.sceneRole == "ROCK" || layer.sceneRole == "SAND" || layer.sceneRole == "VEG")) {
            roles.push_back(layer.sceneRole);
            filenames.push_back(layer.sourcePath.filename().string());
        }
    }
    CHECK(roles.size() == 3U);
    CHECK(std::find(roles.begin(), roles.end(), "ROCK") != roles.end());
    CHECK(std::find(roles.begin(), roles.end(), "SAND") != roles.end());
    CHECK(std::find(roles.begin(), roles.end(), "VEG") != roles.end());
    CHECK(std::find(filenames.begin(), filenames.end(), "Site3-ROCK-5mm.ply") != filenames.end());
    CHECK(std::find(filenames.begin(), filenames.end(), "Site3-SAND-5mm.ply") != filenames.end());
    CHECK(std::find(filenames.begin(), filenames.end(), "Site3-VEG-5mm.ply") != filenames.end());
}

TEST_CASE("Combined water support samples all scene roles with role multipliers", "[water][scene]") {
    auto makeCloud = [](float xOffset) {
        invisible_places::io::LoadedPointCloud cloud;
        cloud.sourcePath = "role-cloud.ply";
        cloud.layerName = "role-cloud";
        cloud.hasNormals = true;
        cloud.hasSourceRgb = true;
        for (std::size_t index = 0; index < 120U; ++index) {
            const invisible_places::io::Float3 point{
                xOffset + static_cast<float>(index) * 0.01F,
                static_cast<float>(index % 17U) * 0.01F,
                1.5F,
            };
            cloud.positions.push_back(point);
            cloud.normals.push_back({0.0F, 0.0F, 1.0F});
            cloud.packedColors.push_back(0xFFFFFFFFU);
            cloud.bounds.Expand(point);
        }
        cloud.hasFocusPoint = true;
        cloud.focusPoint = {xOffset, 0.0F, 1.5F};
        return cloud;
    };

    const auto rock = makeCloud(0.0F);
    const auto sand = makeCloud(10.0F);
    const auto veg = makeCloud(20.0F);
    invisible_places::water::WaterPathGenerationSettings settings;
    settings.supportVoxelSize = 0.002F;
    settings.supportSampleLimit = 60U;
    const std::array<invisible_places::water::WaterSceneSupportLayer, 3> layers{
        invisible_places::water::WaterSceneSupportLayer{.cloud = &rock, .role = "ROCK", .pointSpacingMeters = 0.001F, .samplingMultiplier = 1.0F},
        invisible_places::water::WaterSceneSupportLayer{.cloud = &sand, .role = "SAND", .pointSpacingMeters = 0.002F, .samplingMultiplier = 2.0F},
        invisible_places::water::WaterSceneSupportLayer{.cloud = &veg, .role = "VEG", .pointSpacingMeters = 0.001F, .samplingMultiplier = 2.0F},
    };

    const auto combined = invisible_places::water::BuildCombinedWaterSupportCloud(layers, settings);

    REQUIRE(combined.positions.size() <= settings.supportSampleLimit);
    const auto rockSamples = static_cast<std::size_t>(std::count_if(
        combined.positions.begin(),
        combined.positions.end(),
        [](const auto& point) { return point.x < 5.0F; }));
    const auto sandSamples = static_cast<std::size_t>(std::count_if(
        combined.positions.begin(),
        combined.positions.end(),
        [](const auto& point) { return point.x > 5.0F && point.x < 15.0F; }));
    const auto vegSamples = static_cast<std::size_t>(std::count_if(
        combined.positions.begin(),
        combined.positions.end(),
        [](const auto& point) { return point.x > 15.0F; }));
    CHECK(rockSamples > 0U);
    CHECK(sandSamples > 0U);
    CHECK(vegSamples > 0U);
    CHECK(rockSamples >= sandSamples);
    CHECK(rockSamples >= vegSamples);
}

TEST_CASE("Combined water support reserves source-neighbour samples across scene roles", "[water][scene]") {
    auto makeCloud = [](float xOffset) {
        invisible_places::io::LoadedPointCloud cloud;
        cloud.sourcePath = "role-cloud.ply";
        cloud.layerName = "role-cloud";
        cloud.hasNormals = true;
        cloud.hasSourceRgb = true;
        for (std::size_t index = 0; index < 80U; ++index) {
            const invisible_places::io::Float3 point{
                xOffset + static_cast<float>(index % 20U) * 0.001F,
                static_cast<float>(index / 20U) * 0.001F,
                1.0F - static_cast<float>(index % 7U) * 0.001F,
            };
            cloud.positions.push_back(point);
            cloud.normals.push_back({0.0F, 0.0F, 1.0F});
            cloud.packedColors.push_back(0xFFFFFFFFU);
            cloud.bounds.Expand(point);
        }
        cloud.hasFocusPoint = true;
        cloud.focusPoint = {xOffset, 0.0F, 1.0F};
        return cloud;
    };

    const auto rock = makeCloud(0.00F);
    const auto sand = makeCloud(0.20F);
    const auto veg = makeCloud(0.40F);
    invisible_places::water::WaterPathGenerationSettings settings;
    settings.supportVoxelSize = 0.002F;
    settings.maxBridgeDistance = 0.050F;
    settings.pathSampleSpacing = 0.002F;
    settings.supportSampleLimit = 30U;
    const std::array<invisible_places::water::WaterSceneSupportLayer, 3> layers{
        invisible_places::water::WaterSceneSupportLayer{.cloud = &rock, .role = "ROCK", .pointSpacingMeters = 0.001F, .samplingMultiplier = 1.0F},
        invisible_places::water::WaterSceneSupportLayer{.cloud = &sand, .role = "SAND", .pointSpacingMeters = 0.001F, .samplingMultiplier = 2.0F},
        invisible_places::water::WaterSceneSupportLayer{.cloud = &veg, .role = "VEG", .pointSpacingMeters = 0.001F, .samplingMultiplier = 2.0F},
    };
    const std::array<invisible_places::io::Float3, 1> sources{{{0.20F, 0.0F, 1.0F}}};

    const auto combined = invisible_places::water::BuildCombinedWaterSupportCloud(layers, settings, sources);

    REQUIRE(combined.positions.size() <= settings.supportSampleLimit);
    REQUIRE(combined.positions.size() >= 18U);
    const auto priorityEnd = combined.positions.begin() + 18;
    const auto rockPrioritySamples = static_cast<std::size_t>(std::count_if(
        combined.positions.begin(),
        priorityEnd,
        [](const auto& point) { return point.x < 0.10F; }));
    const auto sandPrioritySamples = static_cast<std::size_t>(std::count_if(
        combined.positions.begin(),
        priorityEnd,
        [](const auto& point) { return point.x > 0.10F && point.x < 0.30F; }));
    const auto vegPrioritySamples = static_cast<std::size_t>(std::count_if(
        combined.positions.begin(),
        priorityEnd,
        [](const auto& point) { return point.x > 0.30F; }));
    CHECK(rockPrioritySamples > 0U);
    CHECK(sandPrioritySamples > 0U);
    CHECK(vegPrioritySamples > 0U);
}

TEST_CASE("Shoreline wave height mask fades around the sand-rock boundary", "[water][shoreline]") {
    using invisible_places::renderer::pointcloud::ShorelineWaveHeightMask;
    constexpr float boundaryZ = 1.55F;
    constexpr float reach = 0.45F;
    constexpr float fade = 0.05F;

    CHECK(ShorelineWaveHeightMask(boundaryZ, reach, fade, 1.61F) == Catch::Approx(0.0F));
    CHECK(ShorelineWaveHeightMask(boundaryZ, reach, fade, 1.55F) > 0.45F);
    CHECK(ShorelineWaveHeightMask(boundaryZ, reach, fade, 1.40F) > 0.95F);
    CHECK(ShorelineWaveHeightMask(boundaryZ, reach, fade, 1.08F) > 0.0F);
    CHECK(ShorelineWaveHeightMask(boundaryZ, reach, fade, 0.99F) == Catch::Approx(0.0F));
}

TEST_CASE("Height Foam shoreline keeps independent defaults and clamps break height", "[water][shoreline]") {
    using invisible_places::renderer::pointcloud::NormalizeHeightFoamBreakZ;
    using invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm;
    using invisible_places::renderer::pointcloud::PointCloudStyleHasActiveShorelineWaves;
    using invisible_places::renderer::pointcloud::PointCloudStyleHasShorelineWaveRegion;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;

    PointCloudStyleState style;
    CHECK(style.shorelineWaveAlgorithm == PointCloudShorelineWaveAlgorithm::FoamFronts);
    CHECK(style.shorelineHeightFoam.runupZ == Catch::Approx(1.55F));
    CHECK(style.shorelineHeightFoam.breakZ == Catch::Approx(1.30F));
    CHECK(style.shorelineHeightFoam.offshoreReachMeters == Catch::Approx(0.55F));
    CHECK(style.shorelineHeightFoam.offshoreFoamStrength == Catch::Approx(0.30F));
    CHECK(style.shorelineHeightFoam.incomingStrength == Catch::Approx(1.0F));
    CHECK(style.shorelineHeightFoam.returnStrength == Catch::Approx(0.30F));

    style.shorelineWaveEnabled = true;
    style.shorelineWaveAlgorithm = PointCloudShorelineWaveAlgorithm::HeightFoam;
    style.shorelineHeightFoam.runupZ = 2.0F;
    style.shorelineHeightFoam.breakZ = 1.5F;
    CHECK(style.shorelineBoundaryZ == Catch::Approx(1.55F));
    CHECK(PointCloudStyleHasActiveShorelineWaves(style));

    CHECK(NormalizeHeightFoamBreakZ(2.0F, 0.8F, 0.1F, 0.0F) == Catch::Approx(1.3F));
    CHECK(NormalizeHeightFoamBreakZ(2.0F, 0.8F, 0.1F, 3.0F) == Catch::Approx(1.9F));
    CHECK(NormalizeHeightFoamBreakZ(2.0F, 0.8F, 0.1F, 1.6F) == Catch::Approx(1.6F));

    style.shorelineHeightFoam.speed = 0.0F;
    CHECK_FALSE(PointCloudStyleHasActiveShorelineWaves(style));
    CHECK(PointCloudStyleHasShorelineWaveRegion(style));
    CHECK(style.shorelineSpeed == Catch::Approx(0.55F));

    style.shorelineWaveAlgorithm = PointCloudShorelineWaveAlgorithm::ContinuousBands;
    CHECK(PointCloudStyleHasActiveShorelineWaves(style));
    CHECK(PointCloudStyleHasShorelineWaveRegion(style));
    style.shorelineSpeed = 0.0F;
    CHECK_FALSE(PointCloudStyleHasActiveShorelineWaves(style));

    style.shorelineWaveEnabled = false;
    CHECK_FALSE(PointCloudStyleHasShorelineWaveRegion(style));
}

TEST_CASE("Shoreline profiles copy only water settings and Calm matches the authored preset",
          "[water][shoreline][profiles]") {
    using invisible_places::renderer::pointcloud::ApplyPointCloudShorelineWaveSettings;
    using invisible_places::renderer::pointcloud::CalmPointCloudShorelineWaveSettings;
    using invisible_places::renderer::pointcloud::ExtractPointCloudShorelineWaveSettings;
    using invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;

    auto calm = CalmPointCloudShorelineWaveSettings();
    CHECK(calm.enabled);
    CHECK(calm.algorithm == PointCloudShorelineWaveAlgorithm::FoamFronts);
    CHECK(calm.foamFronts.boundaryZ == Catch::Approx(1.595F));
    CHECK(calm.foamFronts.heightReachMeters == Catch::Approx(2.0F));
    CHECK(calm.foamFronts.edgeFadeMeters == Catch::Approx(0.117F));
    CHECK(
        std::atan2(calm.foamFronts.directionY, calm.foamFronts.directionX) *
            180.0F / 3.14159265358979323846F ==
        Catch::Approx(81.0F));
    CHECK(calm.foamFronts.wavelengthMeters == Catch::Approx(0.10F));
    CHECK(calm.foamFronts.patternScale == Catch::Approx(0.33F));
    CHECK(calm.foamFronts.speed == Catch::Approx(0.55F));
    CHECK(calm.foamFronts.warp == Catch::Approx(1.05F));
    CHECK(calm.foamFronts.turbulence == Catch::Approx(0.64F));
    CHECK(calm.foamFronts.density == Catch::Approx(1.0F));
    CHECK(calm.foamFronts.intensity == Catch::Approx(0.97F));
    CHECK(calm.foamFronts.emissionAdd == Catch::Approx(0.65F));
    CHECK(calm.foamFronts.opacityAdd == Catch::Approx(0.08F));
    CHECK(calm.foamFronts.opacityMultiply == Catch::Approx(1.25F));
    CHECK(calm.foamFronts.pointSizeAdd == Catch::Approx(0.0F));
    CHECK(calm.foamFronts.pointSizeMultiply == Catch::Approx(1.35F));
    CHECK(calm.foamFronts.colourMix == Catch::Approx(0.75F));
    CHECK(calm.foamFronts.colour[0] == Catch::Approx(158.0F / 255.0F));
    CHECK(calm.foamFronts.colour[1] == Catch::Approx(224.0F / 255.0F));
    CHECK(calm.foamFronts.colour[2] == Catch::Approx(1.0F));

    // Applying a Shoreline profile must not replace the rest of the selected
    // point visual. It also carries the inactive algorithm bank so changing
    // algorithms after selection recovers the profile's authored values.
    calm.heightFoam.runupZ = 2.17F;
    calm.heightFoam.offshoreFoamStrength = 0.43F;
    PointCloudStyleState style;
    style.exposure = 1.73F;
    style.solidColor = {0.11F, 0.22F, 0.33F, 0.44F};
    ApplyPointCloudShorelineWaveSettings(&style, calm);
    CHECK(style.exposure == Catch::Approx(1.73F));
    CHECK(style.solidColor ==
          std::array<float, 4>{0.11F, 0.22F, 0.33F, 0.44F});
    CHECK(style.shorelineBoundaryZ == Catch::Approx(1.595F));
    CHECK(style.shorelineHeightFoam.runupZ == Catch::Approx(2.17F));

    const auto extracted = ExtractPointCloudShorelineWaveSettings(style);
    CHECK(extracted.enabled);
    CHECK(extracted.foamFronts.warp == Catch::Approx(1.05F));
    CHECK(extracted.heightFoam.offshoreFoamStrength == Catch::Approx(0.43F));
}

TEST_CASE("Shoreline object profile names avoid bases and other object copies",
          "[water][shoreline][profiles][naming]") {
    using invisible_places::renderer::pointcloud::
        PointCloudShorelineWaveProfile;
    using invisible_places::renderer::pointcloud::
        UniquePointCloudShorelineObjectProfileName;

    SECTION("saved base names retain ownership of the preferred name") {
        const std::vector<PointCloudShorelineWaveProfile> profiles{
            {.name = "Calm_North"},
        };
        CHECK(
            UniquePointCloudShorelineObjectProfileName(
                profiles,
                " Calm ",
                " North ",
                7U) ==
            "Calm_North 2");
    }

    SECTION("copies owned by other objects receive deterministic suffixes") {
        const std::vector<PointCloudShorelineWaveProfile> profiles{
            {.name = "Calm_North"},
            {.name = "Calm_North 2",
             .objectOverride = true,
             .shorelineInstanceId = 8U,
             .baseProfileName = "Calm"},
        };
        CHECK(
            UniquePointCloudShorelineObjectProfileName(
                profiles,
                "Calm",
                "North",
                7U) ==
            "Calm_North 3");
    }

    SECTION("updating the same owner and base keeps its stable name") {
        const std::vector<PointCloudShorelineWaveProfile> profiles{
            {.name = "Calm"},
            {.name = "Calm_North",
             .objectOverride = true,
             .shorelineInstanceId = 7U,
             .baseProfileName = " Calm "},
        };
        CHECK(
            UniquePointCloudShorelineObjectProfileName(
                profiles,
                "Calm",
                "North",
                7U) ==
            "Calm_North");
    }

    SECTION("blank object labels use the deterministic Shoreline fallback") {
        const std::vector<PointCloudShorelineWaveProfile> profiles;
        CHECK(
            UniquePointCloudShorelineObjectProfileName(
                profiles,
                " Calm ",
                "   ",
                7U) ==
            "Calm_Shoreline");
        CHECK(
            UniquePointCloudShorelineObjectProfileName(
                profiles,
                "   ",
                "North",
                7U)
                .empty());
    }
}

TEST_CASE("Sand-cloud shoreline waves use dedicated foam helpers", "[water][shoreline][shader]") {
    const auto shaderPath = DataRoot().parent_path() / "shaders" / "pointcloud_sparse_ripple.glsl";
    std::ifstream input{shaderPath};
    REQUIRE(input.good());
    const std::string shader{
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};

    const auto helperPos = shader.find("float SandCloudShorelineWaveValue(");
    REQUIRE(helperPos != std::string::npos);
    const auto helperEnd = shader.find("\n}\n\nfloat ContinuousBandShorelineWaveValue", helperPos);
    REQUIRE(helperEnd != std::string::npos);
    const auto helperBody = shader.substr(helperPos, helperEnd - helperPos);
    CHECK(helperBody.find("const float t = phase;") != std::string::npos);
    CHECK(helperBody.find("const float t = -phase;") == std::string::npos);
    CHECK(helperBody.find("const float alongShore = uv.y;") != std::string::npos);
    CHECK(helperBody.find("finishOffset - max(0.0, shoreDistance)") != std::string::npos);
    CHECK(helperBody.find("const float incomingMask") != std::string::npos);
    CHECK(helperBody.find("const float incomingDeepSideMask") != std::string::npos);
    CHECK(helperBody.find("const float incomingStartMask") != std::string::npos);
    CHECK(helperBody.find("const float incomingBodyFoam") != std::string::npos);
    CHECK(helperBody.find("const float arrivingFoam") != std::string::npos);
    CHECK(helperBody.find("const float frontLocalCoordinate = front;") != std::string::npos);
    CHECK(helperBody.find("const vec2 waveFrontStreakUv") != std::string::npos);
    CHECK(helperBody.find("const float waveFrontStreakNoise") != std::string::npos);
    CHECK(helperBody.find("frontLocalCoordinate * (2.05 + turbulence01 * 0.45)") != std::string::npos);
    CHECK(helperBody.find("x * (2.05 + turbulence01 * 0.45)") == std::string::npos);
    CHECK(helperBody.find("const float incomingFollowMask") != std::string::npos);
    CHECK(helperBody.find("const float trailDistance = max(0.0, front);") != std::string::npos);
    CHECK(helperBody.find("const float returnFollowMask") != std::string::npos);
    CHECK(helperBody.find("const float edgeIntroduce = smoothstep(0.0, 0.18, returnProgress);") != std::string::npos);
    CHECK(helperBody.find("const float peakCarryFade") != std::string::npos);
    CHECK(helperBody.find("edgeValue * edgeIntroduce") != std::string::npos);
    // No-pause guarantee: Band Density runs two to four fronts on one
    // shared cycle rate with staggered offsets and bounded wobble; the old
    // per-seed slot cull and per-slot cycle speeds that let quiet windows
    // align are gone.
    CHECK(helperBody.find("const float frontCount = mix(2.0, 4.0, density01);") !=
          std::string::npos);
    CHECK(helperBody.find("const float activationRank") != std::string::npos);
    CHECK(helperBody.find("slot * 0.25 +") != std::string::npos);
    CHECK(helperBody.find("waveGate") == std::string::npos);
    CHECK(helperBody.find("speedNoise") == std::string::npos);

    const auto continuousBandsPos = shader.find("float ContinuousBandShorelineWaveValue(");
    REQUIRE(continuousBandsPos != std::string::npos);
    const auto continuousBandsEnd = shader.find(
        "\n}\n\nfloat HeightFoamShorelineWaveValue",
        continuousBandsPos);
    REQUIRE(continuousBandsEnd != std::string::npos);
    const auto continuousBandsBody = shader.substr(
        continuousBandsPos,
        continuousBandsEnd - continuousBandsPos);
    CHECK(continuousBandsBody.find("const float activeBandCount = mix(2.0, 4.0, density01);") !=
          std::string::npos);
    // The bay-beach jostle contract: overlapping fronts on smooth staggered
    // cycles, a vigour swell choosing crash-versus-fade, and a run-in wash
    // zone — with no lifecycle reset or inactive branch anywhere.
    CHECK(continuousBandsBody.find("const float runInDepth") != std::string::npos);
    CHECK(continuousBandsBody.find("slot * 2.399963") != std::string::npos);
    CHECK(continuousBandsBody.find("const float approach") != std::string::npos);
    CHECK(continuousBandsBody.find("const float vigor") != std::string::npos);
    CHECK(continuousBandsBody.find("const float crashDepth") != std::string::npos);
    CHECK(continuousBandsBody.find("const float waveCenter = mix(startDepth, crashDepth, approach);") !=
          std::string::npos);
    CHECK(continuousBandsBody.find("const float washFoam") != std::string::npos);
    CHECK(continuousBandsBody.find("const float frontLocalCoordinate = front;") !=
          std::string::npos);
    CHECK(continuousBandsBody.find("fract(") == std::string::npos);
    CHECK(continuousBandsBody.find("continue;") == std::string::npos);

    const auto heightFoamPos = shader.find("float HeightFoamShorelineWaveValue(");
    REQUIRE(heightFoamPos != std::string::npos);
    const auto heightFoamEnd = shader.find("\n}\n", heightFoamPos);
    REQUIRE(heightFoamEnd != std::string::npos);
    const auto heightFoamBody = shader.substr(heightFoamPos, heightFoamEnd - heightFoamPos);
    CHECK(heightFoamBody.find("const float persistentFoam") != std::string::npos);
    CHECK(heightFoamBody.find("const float seaFoamMask") != std::string::npos);
    CHECK(heightFoamBody.find("const float breakGain") != std::string::npos);
    CHECK(heightFoamBody.find("float wakeClear") != std::string::npos);
    CHECK(heightFoamBody.find("const float recoveredReservoir") != std::string::npos);
    CHECK(heightFoamBody.find("max(0.0, incomingStrength)") != std::string::npos);
    CHECK(heightFoamBody.find("clamp(returnStrength, 0.0, 1.0)") != std::string::npos);

    // The algorithm dispatch lives in the shared lane-parameterized
    // evaluator; the primary shoreline and every additional instance route
    // through it with identical semantics.
    const auto evaluateFromPos =
        shader.find("SparseRippleComposite EvaluateShorelineWaveContributionFrom(");
    REQUIRE(evaluateFromPos != std::string::npos);
    const auto evaluatePos = shader.find(
        "SparseRippleComposite EvaluateShorelineWaveContribution(vec3",
        evaluateFromPos);
    REQUIRE(evaluatePos != std::string::npos);
    const auto evaluateEnd = shader.find("\n}\n", evaluatePos);
    REQUIRE(evaluateEnd != std::string::npos);
    const auto evaluateFromBody =
        shader.substr(evaluateFromPos, evaluatePos - evaluateFromPos);
    CHECK(evaluateFromBody.find("SandCloudShorelineWaveValue(") != std::string::npos);
    CHECK(evaluateFromBody.find("ContinuousBandShorelineWaveValue(") != std::string::npos);
    CHECK(evaluateFromBody.find("HeightFoamShorelineWaveValue(") != std::string::npos);
    CHECK(evaluateFromBody.find("waveControl.z == 1u") != std::string::npos);
    CHECK(evaluateFromBody.find("waveControl.z == 2u") != std::string::npos);
    const auto evaluateBody = shader.substr(evaluatePos, evaluateEnd - evaluatePos);
    CHECK(evaluateBody.find("EvaluateShorelineWaveContributionFrom(") != std::string::npos);
    CHECK(evaluateBody.find("HasShorelineWaveEffect()") != std::string::npos);

    // Additional shoreline instances blend additively over the primary in
    // the composite resolver, each behind its own height-mask early-out.
    const auto resolvePos =
        shader.find("SparseRippleComposite ResolveSparseRippleComposite(");
    REQUIRE(resolvePos != std::string::npos);
    const auto resolveEnd = shader.find("\n}\n", resolvePos);
    const auto resolveBody = shader.substr(
        resolvePos,
        resolveEnd == std::string::npos ? shader.size() - resolvePos
                                        : resolveEnd - resolvePos);
    CHECK(resolveBody.find("additionalShorelineCount.x") != std::string::npos);
    CHECK(resolveBody.find("additionalShorelineControl[shorelineIndex]") !=
          std::string::npos);
    CHECK(resolveBody.find("BlendSparseRippleContribution(result, instanceContribution, 0u)") !=
          std::string::npos);
}

TEST_CASE("Shoreline shader layouts reserve Height Foam parameters consistently", "[water][shoreline][shader]") {
    const auto shaderRoot = DataRoot().parent_path() / "shaders";
    const std::array<std::filesystem::path, 8> shaderPaths{
        shaderRoot / "pointcloud_fast_basic.frag",
        shaderRoot / "pointcloud_fast_basic.vert",
        shaderRoot / "pointcloud_preview.vert",
        shaderRoot / "pointcloud_surfel.vert",
        shaderRoot / "pointcloud_accumulation.frag",
        shaderRoot / "pointcloud_exr_accumulation.frag",
        shaderRoot / "pointcloud_surfel_accumulation.frag",
        shaderRoot / "pointcloud_surfel_exr_accumulation.frag",
    };

    for (const auto& shaderPath : shaderPaths) {
        std::ifstream input{shaderPath};
        REQUIRE(input.good());
        const std::string shader{
            std::istreambuf_iterator<char>{input},
            std::istreambuf_iterator<char>{}};
        const auto params4 = shader.find("vec4 shorelineWaveParams4;");
        const auto params5 = shader.find("vec4 shorelineWaveParams5;");
        const auto tint = shader.find("vec4 shorelineWaveTint;");
        CAPTURE(shaderPath);
        REQUIRE(params4 != std::string::npos);
        REQUIRE(params5 != std::string::npos);
        REQUIRE(tint != std::string::npos);
        CHECK(params4 < params5);
        CHECK(params5 < tint);
    }
}

TEST_CASE(
    "Mesh Flow contact composition has a frame-local timing visibility gate",
    "[water][mesh-flow][shader][timing][visibility]") {
    using invisible_places::water::WaterFeatureTimingOverlay;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;

    invisible_places::renderer::core::SceneRenderState renderState;
    CHECK(renderState.meshFlowContactEffectsEnabled);

    WaterFeatureTimingOverlay overlay;
    overlay.onlyShowRunFeatures = true;
    renderState.meshFlowContactEffectsEnabled = overlay.Allows(
        WaterKeyedFeatureId{.kind = WaterKeyedFeatureKind::MeshFlow});
    CHECK_FALSE(renderState.meshFlowContactEffectsEnabled);

    overlay.assignedRunFeatures.push_back(
        WaterKeyedFeatureId{.kind = WaterKeyedFeatureKind::MeshFlow});
    renderState.meshFlowContactEffectsEnabled = overlay.Allows(
        WaterKeyedFeatureId{.kind = WaterKeyedFeatureKind::MeshFlow});
    CHECK(renderState.meshFlowContactEffectsEnabled);

    const auto shaderPath =
        DataRoot().parent_path() /
        "shaders/pointcloud_mesh_flow_contact.glsl";
    std::ifstream input{shaderPath};
    REQUIRE(input.good());
    const std::string shader{
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};
    const auto gate = shader.find(
        "styleData.additionalShorelineCount.y == 0u");
    const auto bufferLengthRead = shader.find(
        "meshFlowContactEvents.length()");
    REQUIRE(gate != std::string::npos);
    REQUIRE(bufferLengthRead != std::string::npos);
    CHECK(gate < bufferLengthRead);
}

TEST_CASE("Seepage cannot publish non-finite point material outputs", "[water][seepage][shader]") {
    const auto shaderRoot = DataRoot().parent_path() / "shaders";
    const auto readShader = [](const std::filesystem::path& path) {
        std::ifstream input{path};
        REQUIRE(input.good());
        return std::string{
            std::istreambuf_iterator<char>{input},
            std::istreambuf_iterator<char>{}};
    };

    const auto sparseRipple = readShader(shaderRoot / "pointcloud_sparse_ripple.glsl");
    CHECK(sparseRipple.find("SparseRippleComposite SanitizeSparseRippleComposite(") !=
          std::string::npos);
    CHECK(sparseRipple.find("return SanitizeSparseRippleComposite(result);") !=
          std::string::npos);
    CHECK(sparseRipple.find("RippleFiniteFloat(styleData.seepageGridParams.x)") !=
          std::string::npos);
    CHECK(sparseRipple.find("RippleFiniteVec3(styleData.seepageBoundsMin.xyz)") !=
          std::string::npos);
    CHECK(sparseRipple.find("const uint available = styleData.seepageControl.w - start;") !=
          std::string::npos);
    CHECK(sparseRipple.find(
              "seepageParamData.seepageParams[nodeIndex].geometry.x <= 1e-5") !=
          std::string::npos);
    CHECK(sparseRipple.find("float SampleAnimatedSeepagePulseField(") !=
          std::string::npos);
    CHECK(sparseRipple.find("fieldDistanceMeters - time * speed") !=
          std::string::npos);
    CHECK(sparseRipple.find("if (quality >= 2u)") != std::string::npos);
    CHECK(sparseRipple.find("if (quality >= 3u)") != std::string::npos);

    const auto preview = readShader(shaderRoot / "pointcloud_preview.vert");
    CHECK(preview.find("RippleFiniteFloat(resolvedPointSize)") != std::string::npos);
    CHECK(preview.find("RippleFiniteFloat(resolvedOpacity)") != std::string::npos);
    CHECK(preview.find("RippleFiniteFloat(resolvedEmissive)") != std::string::npos);

    const auto fastBasic = readShader(shaderRoot / "pointcloud_fast_basic.vert");
    CHECK(fastBasic.find("RippleFiniteFloat(resolvedPointSize)") != std::string::npos);
}

TEST_CASE("Shoreline wave defaults are visible when enabled", "[water][shoreline][pointcloud]") {
    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.shorelineWaveEnabled = true;

    CHECK(style.shorelineIntensity >= 1.0F);
    CHECK(style.shorelineEmissionAdd >= 0.5F);
    CHECK(style.shorelineOpacityAdd > 0.0F);
    CHECK(style.shorelineOpacityMultiply > 1.0F);
    CHECK(style.shorelinePointSizeMultiply > 1.0F);
    CHECK(style.shorelineColourMix >= 0.70F);
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

TEST_CASE("Project window lock overrides an animation window preference", "[window]") {
    using invisible_places::platform::ResolvePreferredWindowSize;
    using invisible_places::platform::WindowSize;

    const WindowSize current{.width = 1440, .height = 900};
    const WindowSize animation{.width = 1920, .height = 1080};
    const WindowSize project{.width = 1280, .height = 720};

    const auto animationPreferred = ResolvePreferredWindowSize(
        current,
        animation,
        project,
        false);
    CHECK(animationPreferred.width == 1920);
    CHECK(animationPreferred.height == 1080);

    const auto projectLocked = ResolvePreferredWindowSize(
        current,
        animation,
        project,
        true);
    CHECK(projectLocked.width == 1280);
    CHECK(projectLocked.height == 720);

    const auto unchanged = ResolvePreferredWindowSize(
        current,
        std::nullopt,
        project,
        false);
    CHECK(unchanged.width == 1440);
    CHECK(unchanged.height == 900);
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

TEST_CASE("Frustum union point mask keeps full-density visible points", "[budget][frustum][export]") {
    invisible_places::io::LoadedPointCloud cloud;
    const std::array<invisible_places::io::Float3, 5> points = {
        invisible_places::io::Float3{-4.0F, 0.0F, 0.0F},
        invisible_places::io::Float3{-0.25F, 0.0F, 0.0F},
        invisible_places::io::Float3{0.25F, 0.0F, 0.0F},
        invisible_places::io::Float3{4.0F, 0.0F, 0.0F},
        invisible_places::io::Float3{0.0F, 2.5F, 0.0F},
    };
    for (const auto& point : points) {
        cloud.positions.push_back(point);
        cloud.bounds.Expand(point);
    }

    const std::vector<glm::mat4> centerView{
        glm::ortho(-0.5F, 0.5F, -0.5F, 0.5F, -1.0F, 1.0F),
    };
    const auto centerMask = invisible_places::renderer::pointcloud::GenerateFrustumUnionPointIndices(
        cloud.positions,
        cloud.bounds,
        std::span<const glm::mat4>{centerView.data(), centerView.size()},
        32U);

    CHECK(std::is_sorted(centerMask.begin(), centerMask.end()));
    CHECK(std::find(centerMask.begin(), centerMask.end(), 1U) != centerMask.end());
    CHECK(std::find(centerMask.begin(), centerMask.end(), 2U) != centerMask.end());
    CHECK(std::find(centerMask.begin(), centerMask.end(), 0U) == centerMask.end());
    CHECK(std::find(centerMask.begin(), centerMask.end(), 3U) == centerMask.end());
    CHECK(std::find(centerMask.begin(), centerMask.end(), 4U) == centerMask.end());

    const std::vector<glm::mat4> pathViews{
        glm::ortho(-4.25F, -3.75F, -0.5F, 0.5F, -1.0F, 1.0F),
        glm::ortho(3.75F, 4.25F, -0.5F, 0.5F, -1.0F, 1.0F),
    };
    const auto pathMask = invisible_places::renderer::pointcloud::GenerateFrustumUnionPointIndices(
        cloud.positions,
        cloud.bounds,
        std::span<const glm::mat4>{pathViews.data(), pathViews.size()},
        32U);

    CHECK(std::find(pathMask.begin(), pathMask.end(), 0U) != pathMask.end());
    CHECK(std::find(pathMask.begin(), pathMask.end(), 3U) != pathMask.end());
    CHECK(std::find(pathMask.begin(), pathMask.end(), 1U) == pathMask.end());
    CHECK(std::find(pathMask.begin(), pathMask.end(), 2U) == pathMask.end());
}

TEST_CASE(
    "Frustum union point mask never drops points inside any frame frustum",
    "[budget][frustum][export]") {
    // A dense deterministic grid large enough to exercise the concurrent
    // cell-marking and point-classification ranges.
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int kSide = 40;
    for (int x = 0; x < kSide; ++x) {
        for (int y = 0; y < kSide; ++y) {
            for (int z = 0; z < 4; ++z) {
                const invisible_places::io::Float3 point{
                    static_cast<float>(x) * 0.5F - 10.0F,
                    static_cast<float>(y) * 0.5F - 10.0F,
                    static_cast<float>(z) * 0.5F - 1.0F,
                };
                cloud.positions.push_back(point);
                cloud.bounds.Expand(point);
            }
        }
    }

    // A small moving window sweeping across one corner of the grid.
    std::vector<glm::mat4> views;
    for (int step = 0; step < 24; ++step) {
        const float center = -9.0F + static_cast<float>(step) * 0.25F;
        views.push_back(glm::ortho(
            center - 0.75F,
            center + 0.75F,
            -1.0F,
            1.0F,
            -2.0F,
            2.0F));
    }

    const auto mask =
        invisible_places::renderer::pointcloud::GenerateFrustumUnionPointIndices(
            cloud.positions,
            cloud.bounds,
            std::span<const glm::mat4>{views.data(), views.size()},
            48U);
    REQUIRE_FALSE(mask.empty());
    CHECK(std::is_sorted(mask.begin(), mask.end()));
    CHECK(std::adjacent_find(mask.begin(), mask.end()) == mask.end());
    CHECK(mask.size() < cloud.positions.size());

    // Every point that any frame projects inside clip space must survive.
    const std::unordered_set<std::uint32_t> masked{mask.begin(), mask.end()};
    for (std::uint32_t index = 0; index < cloud.positions.size(); ++index) {
        const auto& point = cloud.positions[index];
        bool insideAnyView = false;
        for (const auto& view : views) {
            const glm::vec4 clip =
                view * glm::vec4{point.x, point.y, point.z, 1.0F};
            if (std::abs(clip.x) <= clip.w &&
                std::abs(clip.y) <= clip.w &&
                std::abs(clip.z) <= clip.w) {
                insideAnyView = true;
                break;
            }
        }
        if (insideAnyView) {
            INFO("point index " << index << " lies inside a frame frustum");
            CHECK(masked.contains(index));
        }
    }
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

TEST_CASE(
    "Canonical live views orbit around the focal point without changing distance",
    "[camera][views]") {
    invisible_places::camera::CameraState state;
    state.position = {-7.0F, 2.0F, 6.0F};
    state.target = {2.0F, -3.0F, 4.0F};
    state.orbitCenter = state.target;
    state.hasOrbitCenter = true;
    state.orientation = {0.0F, 0.0F, 0.0F, 0.0F};

    invisible_places::camera::OrbitCamera camera;
    camera.ApplyState(state);
    const glm::vec3 focalPoint = camera.OrbitCenter();
    const float focalDistance = camera.Distance();

    struct ViewPreset {
        const char* name;
        glm::vec3 forward;
        glm::vec3 requestedUp;
    };
    const std::array presets{
        ViewPreset{"Top", {0.0F, 0.0F, -1.0F}, {1.0F, 0.0F, 0.0F}},
        ViewPreset{"Front", {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
        ViewPreset{"Left", {0.0F, -1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
        ViewPreset{"Right", {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
        ViewPreset{
            "Isometric",
            glm::normalize(glm::vec3{1.0F, 0.0F, -1.0F}),
            {0.0F, 0.0F, 1.0F}},
    };

    for (const auto& preset : presets) {
        DYNAMIC_SECTION(preset.name) {
            camera.SetViewDirectionAroundOrbitCenter(
                preset.forward,
                preset.requestedUp);
            const auto cameraState = camera.CaptureState();
            const glm::quat orientation{
                cameraState.orientation[3],
                cameraState.orientation[0],
                cameraState.orientation[1],
                cameraState.orientation[2],
            };
            const glm::vec3 actualForward =
                orientation * glm::vec3{0.0F, 0.0F, -1.0F};
            const glm::vec3 actualUp =
                orientation * glm::vec3{0.0F, 1.0F, 0.0F};
            const glm::vec3 expectedForward =
                glm::normalize(preset.forward);
            const glm::vec3 expectedUp = glm::normalize(
                preset.requestedUp -
                (expectedForward *
                 glm::dot(preset.requestedUp, expectedForward)));
            const glm::vec3 cameraPosition{
                cameraState.position[0],
                cameraState.position[1],
                cameraState.position[2],
            };

            CHECK(glm::dot(actualForward, expectedForward) > 0.9999F);
            CHECK(glm::dot(actualUp, expectedUp) > 0.9999F);
            CHECK(
                glm::length(cameraPosition - focalPoint) ==
                Catch::Approx(focalDistance));
            CHECK(
                glm::length(
                    cameraPosition -
                    (focalPoint - (expectedForward * focalDistance))) <
                0.0001F);

            const auto matrices = camera.Matrices(1.6F);
            const glm::vec4 focalClip =
                matrices.viewProjection * glm::vec4{focalPoint, 1.0F};
            REQUIRE(std::abs(focalClip.w) > 1.0e-6F);
            CHECK(focalClip.x / focalClip.w == Catch::Approx(0.0F).margin(0.0001F));
            CHECK(focalClip.y / focalClip.w == Catch::Approx(0.0F).margin(0.0001F));
        }
    }
}

TEST_CASE(
    "Parallel projection matches perspective scale at the focal point",
    "[camera][projection]") {
    invisible_places::camera::CameraState state;
    state.position = {-8.0F, 1.0F, 3.0F};
    state.target = {2.0F, 1.0F, 3.0F};
    state.orbitCenter = state.target;
    state.hasOrbitCenter = true;
    state.orientation = {0.0F, 0.0F, 0.0F, 0.0F};
    state.fovDegrees = 55.0F;

    invisible_places::camera::OrbitCamera camera;
    camera.ApplyState(state);
    camera.SetViewDirectionAroundOrbitCenter(
        glm::vec3{1.0F, 0.0F, 0.0F},
        glm::vec3{0.0F, 0.0F, 1.0F});
    const auto beforeState = camera.CaptureState();
    const auto perspective = camera.Matrices(1.6F);
    const glm::vec3 focalPoint = camera.OrbitCenter();
    const glm::vec3 focalPlanePoint =
        focalPoint + glm::vec3{0.0F, 0.0F, 1.0F};
    const auto projectNdc = [](
                                const invisible_places::camera::OrbitCameraMatrices& matrices,
                                const glm::vec3& point) {
        const glm::vec4 clip =
            matrices.viewProjection * glm::vec4{point, 1.0F};
        return glm::vec3{clip} / clip.w;
    };
    const glm::vec3 perspectiveNdc =
        projectNdc(perspective, focalPlanePoint);

    CHECK_FALSE(camera.ParallelProjection());
    camera.SetParallelProjection(true);
    REQUIRE(camera.ParallelProjection());
    const auto parallel = camera.Matrices(1.6F);
    const auto afterState = camera.CaptureState();
    const glm::vec3 parallelNdc =
        projectNdc(parallel, focalPlanePoint);

    CHECK(parallelNdc.x == Catch::Approx(perspectiveNdc.x).margin(0.0001F));
    CHECK(parallelNdc.y == Catch::Approx(perspectiveNdc.y).margin(0.0001F));
    for (std::size_t component = 0U; component < 3U; ++component) {
        CHECK(
            afterState.position[component] ==
            Catch::Approx(beforeState.position[component]));
        CHECK(
            afterState.orbitCenter[component] ==
            Catch::Approx(beforeState.orbitCenter[component]));
    }

    camera.SetParallelProjection(false);
    CHECK_FALSE(camera.ParallelProjection());
}

TEST_CASE("CloudCompare trackball orbits freely and rolls at its rim", "[camera][orbit][trackball]") {
    invisible_places::io::Bounds3f bounds;
    bounds.Expand({-1.0F, -1.0F, -1.0F});
    bounds.Expand({1.0F, 1.0F, 1.0F});

    invisible_places::camera::OrbitCamera camera;
    camera.FrameBounds(bounds, 1.0F);
    const glm::vec3 pivot = camera.OrbitCenter();
    const auto beforeOrbit = camera.Matrices(1.0F);
    const float radiusBeforeOrbit = glm::length(beforeOrbit.position - pivot);

    camera.OrbitTrackball(
        400.0F,
        400.0F,
        500.0F,
        400.0F,
        800.0F,
        800.0F,
        400.0F,
        400.0F);
    const auto afterOrbit = camera.Matrices(1.0F);
    CHECK(glm::length(afterOrbit.position - pivot) == Catch::Approx(radiusBeforeOrbit));
    CHECK(glm::length(afterOrbit.position - beforeOrbit.position) > 0.01F);
    CHECK(camera.OrbitCenter().x == Catch::Approx(pivot.x));
    CHECK(camera.OrbitCenter().y == Catch::Approx(pivot.y));
    CHECK(camera.OrbitCenter().z == Catch::Approx(pivot.z));

    invisible_places::camera::OrbitCamera rollingCamera;
    rollingCamera.FrameBounds(bounds, 1.0F);
    const auto beforeRollState = rollingCamera.CaptureState();
    const glm::quat beforeRollOrientation{
        beforeRollState.orientation[3],
        beforeRollState.orientation[0],
        beforeRollState.orientation[1],
        beforeRollState.orientation[2],
    };
    const auto beforeRollMatrices = rollingCamera.Matrices(1.0F);

    // A tangential quarter-turn along the virtual sphere's rim is pure roll:
    // view direction and eye position stay fixed while camera-up rotates.
    rollingCamera.OrbitTrackball(
        400.0F,
        800.0F,
        0.0F,
        400.0F,
        800.0F,
        800.0F,
        400.0F,
        400.0F);
    const auto afterRollState = rollingCamera.CaptureState();
    const glm::quat afterRollOrientation{
        afterRollState.orientation[3],
        afterRollState.orientation[0],
        afterRollState.orientation[1],
        afterRollState.orientation[2],
    };
    const auto afterRollMatrices = rollingCamera.Matrices(1.0F);
    const glm::vec3 upBefore = beforeRollOrientation * glm::vec3{0.0F, 1.0F, 0.0F};
    const glm::vec3 upAfter = afterRollOrientation * glm::vec3{0.0F, 1.0F, 0.0F};
    const glm::vec3 forwardBefore = beforeRollOrientation * glm::vec3{0.0F, 0.0F, -1.0F};
    const glm::vec3 forwardAfter = afterRollOrientation * glm::vec3{0.0F, 0.0F, -1.0F};
    CHECK(glm::dot(forwardBefore, forwardAfter) > 0.999F);
    CHECK(std::abs(glm::dot(upBefore, upAfter)) < 0.01F);
    CHECK(glm::length(afterRollMatrices.position - beforeRollMatrices.position) < 0.0001F);
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
    document.proResAlphaPreviewEnabled = true;
    document.eyeDomeLightingThickness = 4.0F;
    document.constantUpdateView = true;
    document.liveVisualEffects = true;
    document.sidePanelPinned = true;
    document.orbitControlMode =
        invisible_places::camera::OrbitControlMode::CloudCompareTrackball;
    document.autoLowerGsplatQualityWhileNavigating = false;
    document.pointCloudPreviewLodMode =
        invisible_places::renderer::pointcloud::PointCloudPreviewLodMode::ForceLod;
    document.interactivePointCap = 12'345'678;
    document.pointCloudRendererMode =
        invisible_places::renderer::pointcloud::PointCloudRendererMode::FastBasic;
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
    document.renderJobSettings.supersampleScale = 2;
    document.renderJobSettings.spatialAntialiasing = true;
    document.renderJobSettings.temporalSupersampling = true;
    document.renderJobSettings.temporalSampleCount = 4;
    document.renderJobSettings.motionBlur = true;
    document.renderJobSettings.motionBlurSampleCount = 6;
    document.renderJobSettings.motionBlurShutterAngleDegrees = 180.0F;
    auto proResPreset = invisible_places::output::MakeProRes4444XqVideoToolboxExportPreset();
    proResPreset.name = "AE ProRes XQ VT";
    proResPreset.settings.outputDirectory = "Saved/renders/AE";
    proResPreset.settings.temporalSampleCount = 6;
    proResPreset.settings.motionBlur = true;
    proResPreset.settings.motionBlurSampleCount = 8;
    document.exportPresets.push_back(proResPreset);
    auto proRes422Preset = invisible_places::output::MakeProRes422VideoToolboxExportPreset();
    proRes422Preset.name = "Base ProRes 422 VT";
    proRes422Preset.settings.outputDirectory = "Saved/renders/Base";
    document.exportPresets.push_back(proRes422Preset);
    auto proRes422HqPreset = invisible_places::output::MakeProRes422HqExportPreset();
    proRes422HqPreset.name = "Base ProRes 422 HQ";
    proRes422HqPreset.settings.outputDirectory = "Saved/renders/BaseHQ";
    document.exportPresets.push_back(proRes422HqPreset);
    auto proRes422HqMattePreset = invisible_places::output::MakeProRes422HqAlphaMatteExportPreset();
    proRes422HqMattePreset.name = "Matte ProRes 422 HQ";
    proRes422HqMattePreset.settings.outputDirectory = "Saved/renders/MatteHQ";
    proRes422HqMattePreset.settings.motionBlur = true;
    proRes422HqMattePreset.settings.motionBlurSampleCount = 7;
    document.exportPresets.push_back(proRes422HqMattePreset);
    auto hevcAlphaPreset = invisible_places::output::MakeHevcAlphaMp4ExportPreset();
    hevcAlphaPreset.name = "H265 Alpha Review";
    hevcAlphaPreset.settings.outputDirectory = "Saved/renders/Alpha";
    hevcAlphaPreset.settings.temporalSupersampling = true;
    hevcAlphaPreset.settings.temporalSampleCount = 3;
    document.exportPresets.push_back(hevcAlphaPreset);
    auto pngStackPreset = invisible_places::output::MakePngStackExportPreset();
    pngStackPreset.name = "Premiere PNG Stack";
    pngStackPreset.settings.outputDirectory = "Saved/renders/PNG";
    pngStackPreset.settings.motionBlur = true;
    pngStackPreset.settings.motionBlurSampleCount = 5;
    document.exportPresets.push_back(pngStackPreset);
    auto fastPngStackPreset = invisible_places::output::MakeFastPngStackExportPreset();
    fastPngStackPreset.name = "Fast PNG Review";
    fastPngStackPreset.settings.outputDirectory = "Saved/renders/FastPNG";
    fastPngStackPreset.settings.temporalSupersampling = true;
    fastPngStackPreset.settings.temporalSampleCount = 2;
    document.exportPresets.push_back(fastPngStackPreset);
    auto testMp4Preset = invisible_places::output::MakeTestMp4ExportPreset();
    testMp4Preset.name = "Optical Flow Review";
    testMp4Preset.settings.outputDirectory = "Saved/renders/TestMP4";
    testMp4Preset.settings.framesPerSecond = 5;
    document.exportPresets.push_back(testMp4Preset);
    document.selectedExportPresetName = proResPreset.name;
    auto editedExportPreset = invisible_places::output::MakeFastPreviewMp4ExportPreset();
    editedExportPreset.name =
        invisible_places::output::EditedExportPresetName(invisible_places::output::kFastPreviewMp4PresetName);
    editedExportPreset.settings.width = 1280;
    document.tempExportPreset = editedExportPreset;
    document.waterSourceSettings = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Detail);
    document.waterSourceSettings.path.pathLength = 4.25F;
    document.waterSourceSettings.path.attractorEnabled = true;
    document.waterSourceSettings.path.attractorPosition = {1.25F, 2.50F, 3.75F};
    document.waterSourceSettings.path.attractorStrength = 0.42F;
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
    document.waterTrailGeometry.trailLengthMeters = 1.65F;
    document.waterTrailGeometry.pointSpacingMeters = 0.033F;
    document.waterTrailGeometry.widthMeters = 0.014F;
    document.waterTrailGeometry.streakLengthMeters = 0.095F;
    document.waterTrailGeometry.startFadeEnabled = true;
    document.waterTrailGeometry.startFadeFullDistanceMeters = 0.31F;
    document.waterTrailGeometry.startFadeRandomBeginDistanceMeters = 0.12F;
    document.waterTrailGeometry.endFadeEnabled = true;
    document.waterTrailGeometry.endFadeFullDistanceMeters = 0.47F;
    document.waterTrailGeometry.endFadeRandomBeginDistanceMeters = 0.19F;
    invisible_places::serialization::WaterPathProfileDocument pathProfile;
    pathProfile.name = "Shelf Path";
    pathProfile.settings = document.waterSourceSettings.path;
    pathProfile.settings.pathLength = 7.5F;
    pathProfile.settings.smoothing = 0.41F;
    document.waterPathProfiles.push_back(pathProfile);
    invisible_places::serialization::WaterLaneProfileDocument laneProfile;
    laneProfile.name = "Braided Custom";
    laneProfile.settings = document.waterFlowTrailSettings;
    laneProfile.settings.trailCountTotal = 1234U;
    laneProfile.settings.laneCount = 9U;
    laneProfile.settings.laneSpreadMeters = 0.42F;
    laneProfile.settings.laneCrossing = 0.57F;
    document.waterLaneProfiles.push_back(laneProfile);
    invisible_places::serialization::WaterTrailProfileDocument flowTrailProfile;
    flowTrailProfile.name = "Silver Trail";
    flowTrailProfile.geometry = document.waterTrailGeometry;
    flowTrailProfile.geometry.widthMeters = 0.021F;
    flowTrailProfile.geometry.streakLengthMeters = 0.13F;
    invisible_places::style::SetScalarConstant(&flowTrailProfile.style.opacity, 0.39F);
    invisible_places::style::SetScalarConstant(&flowTrailProfile.style.emissiveStrength, 1.7F);
    document.waterTrailProfiles.push_back(flowTrailProfile);
    document.selectedWaterPathProfileName = "Shelf Path_edited";
    document.tempWaterPathProfileSettings = pathProfile.settings;
    document.tempWaterPathProfileSettings->pathLength = 8.25F;
    document.selectedWaterLaneProfileName = "Braided Custom_edited";
    document.tempWaterLaneProfileSettings = laneProfile.settings;
    document.tempWaterLaneProfileSettings->speedMetersPerSecond = 0.91F;
    document.selectedWaterTrailProfileName = "Silver Trail_edited";
    document.tempWaterTrailProfile = flowTrailProfile;
    document.tempWaterTrailProfile->geometry.trailLengthMeters = 2.05F;
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
    waterEmitter.pathProfileName = "Shelf Path";
    waterEmitter.laneProfileName = "Braided Custom";
    waterEmitter.trailProfileName = "Silver Trail";
    waterEmitter.pathProfileLocked = true;
    waterEmitter.trailProfileLocked = true;
    document.waterEmitters.push_back(waterEmitter);
    invisible_places::water::WaterManualFlowPathSource manualFlowPath;
    manualFlowPath.id = 18U;
    manualFlowPath.name = "Pool edge";
    manualFlowPath.controlPoints = {{0.0F, 0.0F, 1.0F}, {1.0F, 0.5F, 0.5F}};
    manualFlowPath.laneProfileName = "Braided Custom";
    manualFlowPath.trailProfileName = "Silver Trail";
    manualFlowPath.laneProfileLocked = true;
    document.waterManualFlowPaths.push_back(manualFlowPath);
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
    projectBranch.bakeFingerprint = "emitter=17|roundtrip";
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
    document.waterFlowTrailSettings.trailCountTotal = 321U;
    document.waterFlowTrailSettings.trailWidthMeters = 0.014F;
    document.waterFlowTrailSettings.laneCrossing = 0.44F;
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
    layer.sceneGroupName = "SampleScene";
    layer.sceneRole = "ROCK";
    layer.loaded = true;
    layer.visible = true;
    layer.pointBudgetActivePoints = 2048;

    invisible_places::renderer::pointcloud::PointCloudStyleState pointStyle;
    pointStyle.geometryMode = invisible_places::renderer::pointcloud::PointCloudGeometryMode::WorldSurfels;
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
    pointStyle.waterStreakAspect = 7.5F;
    pointStyle.solidCenters = false;
    pointStyle.flowAnimation = true;
    pointStyle.waterTrailOverlay = true;
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
    auto editedStyle = pointStyle;
    editedStyle.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    editedStyle.solidColor = {0.1F, 0.2F, 0.3F, 1.0F};
    editedStyle.waterStreakAspect = 9.0F;
    document.pointVisuals.push_back({.name = "Warm", .style = pointStyle});
    document.selectedPointVisualName = "Warm";
    document.sceneVisualStates.push_back({
        .sceneGroupName = "SampleScene",
        .visual = {.name = "Warm_SampleScene", .style = editedStyle},
    });
    auto coolStyle = pointStyle;
    coolStyle.exposure = 0.72F;
    document.pointVisuals.push_back({.name = "Cool", .style = coolStyle});
    document.sceneVisualStates.push_back({
        .sceneGroupName = "SampleScene",
        .visual = {.name = "Cool_SampleScene", .style = coolStyle},
    });
    auto scene2Style = pointStyle;
    scene2Style.exposure = 1.15F;
    document.sceneVisualStates.push_back({
        .sceneGroupName = "Scene2",
        .visual = {.name = "Warm_Scene2", .style = scene2Style},
    });
    document.gsplatVisualStyle.colorMode = invisible_places::renderer::gsplat::GaussianSplatColorMode::DcOnly;
    document.gsplatVisualStyle.debugMode = invisible_places::renderer::gsplat::GaussianSplatDebugMode::LayerTint;
    document.gsplatVisualStyle.qualityMode = invisible_places::renderer::gsplat::GaussianSplatQualityMode::High;
    document.gsplatVisualStyle.opacityMultiplier = 0.66F;
    document.gsplatVisualStyle.scaleMultiplier = 1.7F;
    document.gsplatVisualStyle.exposure = 1.35F;
    document.gsplatVisualStyle.saturation = 0.82F;
    document.gsplatVisualStyle.layerTint = {0.2F, 0.4F, 0.6F, 0.8F};
    document.layers.push_back(layer);
    auto scene2Layer = layer;
    scene2Layer.sourcePath = "Data/Site2-SAND-2mm.ply";
    scene2Layer.sceneGroupName = "Scene2";
    scene2Layer.sceneRole = "SAND";
    scene2Layer.loaded = false;
    scene2Layer.visible = false;
    document.layers.push_back(scene2Layer);

    std::string errorMessage;
    std::filesystem::path projectPathCacheSidecar;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, outputPath, &errorMessage));
    {
        std::ifstream savedProject{outputPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedProject},
            std::istreambuf_iterator<char>{}};
        const auto savedJsonDocument = nlohmann::json::parse(savedJson);
        CHECK(savedJson.find("\"render_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"blend_mode\"") != std::string::npos);
        CHECK(savedJson.find("\"active\"") != std::string::npos);
        CHECK(savedJson.find("\"solid_centers\"") != std::string::npos);
        CHECK(savedJson.find("\"stylisation_mode\"") != std::string::npos);
        CHECK(savedJson.find("\"pigment_animation_speed\"") != std::string::npos);
        CHECK(savedJson.find("\"brush_particles\"") != std::string::npos);
        CHECK(savedJson.find("\"npr_preset\"") != std::string::npos);
        CHECK(savedJson.find("\"water_emitters\"") != std::string::npos);
        CHECK(savedJson.find("\"water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"attractor_enabled\"") != std::string::npos);
        CHECK(savedJson.find("\"bake_fingerprint\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_length_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_sample_spacing_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"water_animation_trail_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"Custom Bright Ribbons\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_trail_geometry\"") != std::string::npos);
        CHECK(savedJson.find("\"water_path_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"water_lane_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"water_trail_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_path_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_lane_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_trail_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_path_profile_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_lane_profile_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_trail_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"path_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"lane_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"path_profile_locked\"") != std::string::npos);
        CHECK(savedJson.find("\"lane_profile_locked\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_profile_locked\"") != std::string::npos);
        CHECK(savedJson.find("\"water_streak_aspect\"") != std::string::npos);
        CHECK(savedJson.find("\"water_point_visuals\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_point_visual\"") != std::string::npos);
        CHECK(savedJson.find("\"custom_gradient\"") != std::string::npos);
        CHECK(savedJson.find("\"gradient_start_color\"") != std::string::npos);
        CHECK(savedJson.find("\"gradient_end_color\"") != std::string::npos);
        CHECK(savedJson.find("\"water_point_visual_style\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_point_visual_style\"") == std::string::npos);
        CHECK(savedJson.find("\"target_scene_roles\"") != std::string::npos);
        CHECK(savedJson.find("\"water_flow_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"lane_count\"") != std::string::npos);
        CHECK(savedJson.find("\"preview_tint_mode\"") == std::string::npos);
        CHECK(savedJson.find("\"water_path_cache\"") == std::string::npos);
        CHECK(savedJson.find("\"water_path_cache_manifest\"") != std::string::npos);
        REQUIRE(savedJsonDocument.at("water_scene_states").size() == 1U);
        const auto& pathCacheManifest =
            savedJsonDocument.at("water_scene_states").front().at("water_path_cache_manifest");
        REQUIRE(pathCacheManifest.contains("relative_path"));
        REQUIRE(pathCacheManifest.contains("checksum"));
        const auto relativeSidecar =
            std::filesystem::path{pathCacheManifest.at("relative_path").get<std::string>()};
        projectPathCacheSidecar = relativeSidecar.is_absolute()
                                      ? relativeSidecar
                                      : (outputPath.parent_path() / relativeSidecar).lexically_normal();
        CHECK(projectPathCacheSidecar.extension() == ".flowpathcache");
        CHECK(std::filesystem::is_regular_file(projectPathCacheSidecar));
        CHECK(savedJson.find("\"water_trail_overlay\"") != std::string::npos);
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
        CHECK(savedJson.find("\"export_presets\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_export_preset\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_export_preset\"") != std::string::npos);
        CHECK(savedJson.find("\"mode\": \"prores_422_mov\"") != std::string::npos);
        CHECK(savedJson.find("\"mode\": \"prores_4444_mov\"") != std::string::npos);
        CHECK(savedJson.find("\"mode\": \"fast_preview_mp4\"") != std::string::npos);
        CHECK(savedJson.find("\"quality\"") != std::string::npos);
        CHECK(savedJson.find("\"normal\"") != std::string::npos);
        CHECK(savedJson.find("\"hq\"") != std::string::npos);
        CHECK(savedJson.find("\"xq\"") != std::string::npos);
        CHECK(savedJson.find("\"use_video_toolbox\"") != std::string::npos);
        CHECK(savedJson.find("\"external_alpha_matte\"") != std::string::npos);
        CHECK(savedJson.find("\"prores_422_videotoolbox_mov\"") == std::string::npos);
        CHECK(savedJson.find("\"prores_422_hq_mov\"") == std::string::npos);
        CHECK(savedJson.find("\"prores_422_hq_alpha_matte_mov\"") == std::string::npos);
        CHECK(savedJson.find("\"prores_4444_xq_videotoolbox_mov\"") == std::string::npos);
        CHECK(savedJson.find("\"hevc_alpha_mp4\"") == std::string::npos);
        CHECK(savedJson.find("\"png_stack\"") != std::string::npos);
        CHECK(savedJson.find("\"supersample_scale\"") != std::string::npos);
        CHECK(savedJson.find("\"temporal_supersampling\"") != std::string::npos);
        CHECK(savedJson.find("\"motion_blur_shutter_angle_degrees\"") != std::string::npos);
        CHECK(savedJson.find("\"live_superscale_preview\"") == std::string::npos);
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
        CHECK(savedJson.find("\"orbit_control_mode\": \"cloudcompare_trackball\"") != std::string::npos);
        CHECK(savedJson.find("\"point_cloud_renderer_mode\"") != std::string::npos);
        CHECK(savedJson.find("\"point_visuals\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_point_visual\"") != std::string::npos);
        CHECK(savedJson.find("\"scene_visual_states\"") != std::string::npos);
        CHECK(savedJson.find("\"Warm_SampleScene\"") != std::string::npos);
        CHECK(savedJson.find("\"gsplat_visual_style\"") != std::string::npos);
        CHECK(savedJson.find("\"layer_tint\"") != std::string::npos);
        CHECK(savedJson.find("\"associated_layer_paths\"") != std::string::npos);
        CHECK(savedJson.find("\"saved_animations\"") != std::string::npos);
        CHECK(savedJson.find("\"mode\": \"test_mp4\"") != std::string::npos);
        CHECK(
            savedJson.find(
                std::string{"\"schema_version\": "} +
                std::to_string(invisible_places::serialization::ProjectDocument{}.schemaVersion)) !=
            std::string::npos);
        CHECK(savedJson.find("\"id\": \"camera_entry\"") != std::string::npos);
        CHECK(savedJson.find("\"duration_frames\": 120") == std::string::npos);
    }

    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    REQUIRE(loadedDocument->layers.size() == 2);

    CHECK(loadedDocument->projectName == "Roundtrip");
    CHECK(loadedDocument->sidePanelPinned);
    CHECK(
        loadedDocument->orbitControlMode ==
        invisible_places::camera::OrbitControlMode::CloudCompareTrackball);
    CHECK(!loadedDocument->autoLowerGsplatQualityWhileNavigating);
    CHECK(
        loadedDocument->pointCloudPreviewLodMode ==
        invisible_places::renderer::pointcloud::PointCloudPreviewLodMode::ForceLod);
    CHECK(loadedDocument->interactivePointCap == 12'345'678);
    CHECK(
        loadedDocument->pointCloudRendererMode ==
        invisible_places::renderer::pointcloud::PointCloudRendererMode::FastBasic);
    CHECK(loadedDocument->renderJobSettings.outputDirectory == "Saved/renders/Roundtrip");
    CHECK(loadedDocument->renderJobSettings.width == 3840);
    CHECK(loadedDocument->renderJobSettings.height == 2160);
    CHECK(loadedDocument->renderJobSettings.framesPerSecond == 24);
    CHECK(loadedDocument->renderJobSettings.stillCameraDurationSeconds == Catch::Approx(8.5F));
    CHECK(loadedDocument->renderJobSettings.tileSize == 256);
    CHECK(loadedDocument->renderJobSettings.startFrame == 10);
    CHECK(loadedDocument->renderJobSettings.endFrame == 42);
    CHECK(loadedDocument->renderJobSettings.toShotIndex == 1);
    CHECK(loadedDocument->renderJobSettings.supersampleScale == 2U);
    CHECK(loadedDocument->renderJobSettings.temporalSupersampling);
    CHECK(loadedDocument->renderJobSettings.temporalSampleCount == 4U);
    CHECK(loadedDocument->renderJobSettings.motionBlur);
    CHECK(loadedDocument->renderJobSettings.motionBlurSampleCount == 6U);
    CHECK(loadedDocument->renderJobSettings.motionBlurShutterAngleDegrees == Catch::Approx(180.0F));
    REQUIRE(loadedDocument->exportPresets.size() == 8U);
    CHECK(loadedDocument->exportPresets.front().name == "AE ProRes XQ VT");
    CHECK(
        loadedDocument->exportPresets.front().mode ==
        invisible_places::output::AnimationExportMode::ProRes4444Mov);
    CHECK(
        loadedDocument->exportPresets.front().quality ==
        invisible_places::output::AnimationExportQuality::Xq);
    CHECK(loadedDocument->exportPresets.front().useVideoToolbox);
    CHECK(loadedDocument->exportPresets.front().externalAlphaMatte);
    CHECK(loadedDocument->exportPresets.front().settings.width == 3840U);
    CHECK(loadedDocument->exportPresets.front().settings.height == 2160U);
    CHECK(loadedDocument->exportPresets.front().settings.supersampleScale == 2U);
    CHECK(loadedDocument->exportPresets.front().settings.temporalSampleCount == 6U);
    CHECK(loadedDocument->exportPresets.front().settings.motionBlur);
    CHECK(loadedDocument->exportPresets.front().settings.motionBlurSampleCount == 8U);
    CHECK(loadedDocument->exportPresets[1].name == "Base ProRes 422 VT");
    CHECK(
        loadedDocument->exportPresets[1].mode ==
        invisible_places::output::AnimationExportMode::ProRes422Mov);
    CHECK(
        loadedDocument->exportPresets[1].quality ==
        invisible_places::output::AnimationExportQuality::Normal);
    CHECK(loadedDocument->exportPresets[1].useVideoToolbox);
    CHECK(loadedDocument->exportPresets[1].externalAlphaMatte);
    CHECK(loadedDocument->exportPresets[1].settings.outputDirectory == "Saved/renders/Base");
    CHECK(loadedDocument->exportPresets[2].name == "Base ProRes 422 HQ");
    CHECK(
        loadedDocument->exportPresets[2].mode ==
        invisible_places::output::AnimationExportMode::ProRes422Mov);
    CHECK(
        loadedDocument->exportPresets[2].quality ==
        invisible_places::output::AnimationExportQuality::Hq);
    CHECK_FALSE(loadedDocument->exportPresets[2].useVideoToolbox);
    CHECK(loadedDocument->exportPresets[2].externalAlphaMatte);
    CHECK(loadedDocument->exportPresets[2].settings.outputDirectory == "Saved/renders/BaseHQ");
    CHECK(loadedDocument->exportPresets[3].name == "Matte ProRes 422 HQ");
    CHECK(
        loadedDocument->exportPresets[3].mode ==
        invisible_places::output::AnimationExportMode::ProRes422Mov);
    CHECK(
        loadedDocument->exportPresets[3].quality ==
        invisible_places::output::AnimationExportQuality::Hq);
    CHECK_FALSE(loadedDocument->exportPresets[3].useVideoToolbox);
    CHECK(loadedDocument->exportPresets[3].externalAlphaMatte);
    CHECK(loadedDocument->exportPresets[3].settings.outputDirectory == "Saved/renders/MatteHQ");
    CHECK(loadedDocument->exportPresets[3].settings.motionBlur);
    CHECK(loadedDocument->exportPresets[3].settings.motionBlurSampleCount == 7U);
    CHECK(loadedDocument->exportPresets[4].name == "H265 Alpha Review");
    CHECK(
        loadedDocument->exportPresets[4].mode ==
        invisible_places::output::AnimationExportMode::FastPreviewMp4);
    CHECK(
        loadedDocument->exportPresets[4].quality ==
        invisible_places::output::AnimationExportQuality::Hq);
    CHECK(loadedDocument->exportPresets[4].useVideoToolbox);
    CHECK(loadedDocument->exportPresets[4].externalAlphaMatte);
    CHECK(loadedDocument->exportPresets[4].settings.outputDirectory == "Saved/renders/Alpha");
    CHECK(loadedDocument->exportPresets[4].settings.temporalSupersampling);
    CHECK(loadedDocument->exportPresets[4].settings.temporalSampleCount == 3U);
    CHECK(loadedDocument->exportPresets[5].name == "Premiere PNG Stack");
    CHECK(
        loadedDocument->exportPresets[5].mode ==
        invisible_places::output::AnimationExportMode::PngStack);
    CHECK(loadedDocument->exportPresets[5].settings.outputDirectory == "Saved/renders/PNG");
    CHECK(loadedDocument->exportPresets[5].settings.motionBlur);
    CHECK(loadedDocument->exportPresets[5].settings.motionBlurSampleCount == 5U);
    CHECK(loadedDocument->exportPresets[6].name == "Fast PNG Review");
    CHECK(
        loadedDocument->exportPresets[6].mode ==
        invisible_places::output::AnimationExportMode::FastPngStack);
    CHECK(loadedDocument->exportPresets[6].settings.outputDirectory == "Saved/renders/FastPNG");
    CHECK(loadedDocument->exportPresets[6].settings.temporalSupersampling);
    CHECK(loadedDocument->exportPresets[6].settings.temporalSampleCount == 2U);
    CHECK(loadedDocument->exportPresets[7].name == "Optical Flow Review");
    CHECK(
        loadedDocument->exportPresets[7].mode ==
        invisible_places::output::AnimationExportMode::TestMp4);
    CHECK(loadedDocument->exportPresets[7].settings.outputDirectory == "Saved/renders/TestMP4");
    CHECK(loadedDocument->exportPresets[7].settings.framesPerSecond == 5U);
    CHECK(loadedDocument->exportPresets[7].externalAlphaMatte);
    CHECK(loadedDocument->selectedExportPresetName == "AE ProRes XQ VT");
    REQUIRE(loadedDocument->tempExportPreset.has_value());
    CHECK(
        loadedDocument->tempExportPreset->name ==
        invisible_places::output::EditedExportPresetName(invisible_places::output::kMp4PresetName));
    CHECK(
        loadedDocument->tempExportPreset->mode ==
        invisible_places::output::AnimationExportMode::FastPreviewMp4);
    CHECK(
        loadedDocument->tempExportPreset->quality ==
        invisible_places::output::AnimationExportQuality::Normal);
    CHECK(loadedDocument->tempExportPreset->useVideoToolbox);
    CHECK(loadedDocument->tempExportPreset->externalAlphaMatte);
    CHECK(loadedDocument->tempExportPreset->settings.width == 1280U);
    CHECK(loadedDocument->waterSourceSettings.path.pathLength == Catch::Approx(4.25F));
    CHECK(loadedDocument->waterSourceSettings.path.attractorEnabled);
    CHECK(loadedDocument->waterSourceSettings.path.attractorPosition.x == Catch::Approx(1.25F));
    CHECK(loadedDocument->waterSourceSettings.path.attractorPosition.y == Catch::Approx(2.50F));
    CHECK(loadedDocument->waterSourceSettings.path.attractorPosition.z == Catch::Approx(3.75F));
    CHECK(loadedDocument->waterSourceSettings.path.attractorStrength == Catch::Approx(0.42F));
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
    CHECK(loadedDocument->waterTrailGeometry.trailLengthMeters == Catch::Approx(1.65F));
    CHECK(loadedDocument->waterTrailGeometry.pointSpacingMeters == Catch::Approx(0.033F));
    CHECK(loadedDocument->waterTrailGeometry.widthMeters == Catch::Approx(0.014F));
    CHECK(loadedDocument->waterTrailGeometry.streakLengthMeters == Catch::Approx(0.095F));
    CHECK(loadedDocument->waterTrailGeometry.startFadeEnabled);
    CHECK(loadedDocument->waterTrailGeometry.startFadeFullDistanceMeters == Catch::Approx(0.31F));
    CHECK(loadedDocument->waterTrailGeometry.startFadeRandomBeginDistanceMeters == Catch::Approx(0.12F));
    CHECK(loadedDocument->waterTrailGeometry.endFadeEnabled);
    CHECK(loadedDocument->waterTrailGeometry.endFadeFullDistanceMeters == Catch::Approx(0.47F));
    CHECK(loadedDocument->waterTrailGeometry.endFadeRandomBeginDistanceMeters == Catch::Approx(0.19F));
    REQUIRE(loadedDocument->waterPathProfiles.size() == 1U);
    CHECK(loadedDocument->waterPathProfiles[0].name == "Shelf Path");
    CHECK(loadedDocument->waterPathProfiles[0].settings.pathLength == Catch::Approx(7.5F));
    CHECK(loadedDocument->waterPathProfiles[0].settings.smoothing == Catch::Approx(0.41F));
    REQUIRE(loadedDocument->waterLaneProfiles.size() == 1U);
    CHECK(loadedDocument->waterLaneProfiles[0].name == "Braided Custom");
    CHECK(loadedDocument->waterLaneProfiles[0].settings.trailCountTotal == 1234U);
    CHECK(loadedDocument->waterLaneProfiles[0].settings.laneCount == 9U);
    CHECK(loadedDocument->waterLaneProfiles[0].settings.laneSpreadMeters == Catch::Approx(0.42F));
    REQUIRE(loadedDocument->waterTrailProfiles.size() == 1U);
    CHECK(loadedDocument->waterTrailProfiles[0].name == "Silver Trail");
    CHECK(loadedDocument->waterTrailProfiles[0].geometry.widthMeters == Catch::Approx(0.021F));
    CHECK(loadedDocument->waterTrailProfiles[0].geometry.streakLengthMeters == Catch::Approx(0.13F));
    CHECK(invisible_places::style::ScalarConstant(loadedDocument->waterTrailProfiles[0].style.opacity) ==
          Catch::Approx(0.39F));
    CHECK(loadedDocument->selectedWaterPathProfileName == "Shelf Path_edited");
    CHECK(loadedDocument->selectedWaterLaneProfileName == "Braided Custom_edited");
    CHECK(loadedDocument->selectedWaterTrailProfileName == "Silver Trail_edited");
    REQUIRE(loadedDocument->tempWaterPathProfileSettings.has_value());
    CHECK(loadedDocument->tempWaterPathProfileSettings->pathLength == Catch::Approx(8.25F));
    REQUIRE(loadedDocument->tempWaterLaneProfileSettings.has_value());
    CHECK(loadedDocument->tempWaterLaneProfileSettings->speedMetersPerSecond == Catch::Approx(0.91F));
    REQUIRE(loadedDocument->tempWaterTrailProfile.has_value());
    CHECK(loadedDocument->tempWaterTrailProfile->geometry.trailLengthMeters == Catch::Approx(2.05F));
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
    CHECK(loadedDocument->waterEmitters[0].pathProfileName == "Shelf Path");
    CHECK(loadedDocument->waterEmitters[0].laneProfileName == "Braided Custom");
    CHECK(loadedDocument->waterEmitters[0].trailProfileName == "Silver Trail");
    CHECK(loadedDocument->waterEmitters[0].pathProfileLocked);
    CHECK_FALSE(loadedDocument->waterEmitters[0].laneProfileLocked);
    CHECK(loadedDocument->waterEmitters[0].trailProfileLocked);
    REQUIRE(loadedDocument->waterManualFlowPaths.size() == 1U);
    CHECK(loadedDocument->waterManualFlowPaths[0].name == "Pool edge");
    CHECK(loadedDocument->waterManualFlowPaths[0].laneProfileLocked);
    CHECK_FALSE(loadedDocument->waterManualFlowPaths[0].trailProfileLocked);
    REQUIRE(loadedDocument->waterPathCache.has_value());
    CHECK(loadedDocument->waterPathCache->supportLayerPath == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loadedDocument->waterPathCache->supportSignature == "Data/Site2 -5mm.ply|points=2048");
    CHECK(loadedDocument->waterPathCache->emitterSettingsFingerprint == "roundtrip-fingerprint");
    REQUIRE(loadedDocument->waterPathCache->branches.size() == 1U);
    CHECK(loadedDocument->waterPathCache->branches[0].id == 31U);
    CHECK(loadedDocument->waterPathCache->branches[0].bakeFingerprint == "emitter=17|roundtrip");
    REQUIRE(loadedDocument->waterPathCache->branches[0].rawAnchors.size() == 2U);
    CHECK(loadedDocument->waterPathCache->branches[0].rawAnchors[1].pathDistance == Catch::Approx(2.5F));
    CHECK(loadedDocument->waterPathCache->hiddenBranchIds.empty());
    auto recomputedProjectCache = loadedDocument->waterPathCache.value();
    CHECK_FALSE(invisible_places::water::WaterPathAnalysisCacheCompatible(recomputedProjectCache));
    invisible_places::water::EnsureWaterPathAnalysis(&recomputedProjectCache);
    CHECK(invisible_places::water::WaterPathAnalysisCacheCompatible(recomputedProjectCache));
    CHECK(loadedDocument->waterFlowTrailSettings.trailCountTotal == 321U);
    CHECK(loadedDocument->waterFlowTrailSettings.trailWidthMeters == Catch::Approx(0.014F));
    CHECK(loadedDocument->waterFlowTrailSettings.laneCrossing == Catch::Approx(0.44F));
    CHECK(loadedDocument->cameraPathShotIndices == std::vector<std::size_t>{0, 0});
    CHECK(loadedDocument->cameraPathDurationFrames == 144);
    REQUIRE(loadedDocument->hasSavedAnimationRegistry);
    REQUIRE(loadedDocument->savedAnimations.size() == 1);
    CHECK(loadedDocument->savedAnimations[0].filePath == std::filesystem::path{"Saved/animations/Roundtrip.ipanim.json"});
    REQUIRE(loadedDocument->savedAnimations[0].associatedLayerPaths.size() == 1);
    CHECK(loadedDocument->savedAnimations[0].associatedLayerPaths[0] == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loadedDocument->backgroundColor[2] == Catch::Approx(0.08F));
    CHECK(loadedDocument->eyeDomeLightingEnabled);
    CHECK(loadedDocument->proResAlphaPreviewEnabled);
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
    CHECK(loadedLayer.loaded);
    CHECK(loadedLayer.visible);
    CHECK(loadedLayer.pointBudgetActivePoints == 2048);
    CHECK_FALSE(loadedLayer.pointStyle.has_value());
    CHECK(loadedLayer.pointVisuals.empty());
    CHECK(loadedLayer.selectedPointVisualName == "Unnamed");
    CHECK(loadedLayer.sceneGroupName == "SampleScene");
    CHECK(loadedLayer.sceneRole == "ROCK");
    REQUIRE(loadedDocument->pointVisuals.size() == 2U);
    CHECK(loadedDocument->selectedPointVisualName == "Warm");
    const auto& loadedVisual = loadedDocument->pointVisuals.front();
    CHECK(loadedVisual.name == "Warm");
    CHECK(loadedDocument->pointVisuals[1].name == "Cool");
    CHECK(
        loadedVisual.style.colorMode ==
        invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap);
    CHECK(
        loadedVisual.style.geometryMode ==
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::WorldSurfels);
    CHECK(
        loadedVisual.style.falloffProfile ==
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian);
    CHECK(
        loadedVisual.style.colormap ==
        invisible_places::renderer::pointcloud::PointCloudColormapId::HighContrast);
    CHECK(loadedVisual.style.colorizeColor[0] == Catch::Approx(0.2F));
    CHECK(loadedVisual.style.colorizeColor[1] == Catch::Approx(0.6F));
    CHECK(loadedVisual.style.colorizeColor[2] == Catch::Approx(1.0F));
    CHECK(loadedVisual.style.colorizeAmount == Catch::Approx(0.35F));
    CHECK(
        loadedVisual.style.stylisationMode ==
        invisible_places::renderer::pointcloud::PointCloudStylisationMode::BrushParticles);
    CHECK(
        loadedVisual.style.nprPreset ==
        invisible_places::renderer::pointcloud::PointCloudNprPreset::Cartoon);
    CHECK(loadedVisual.style.stylisationStrength == Catch::Approx(0.8F));
    CHECK(loadedVisual.style.stylisationColorLevels == Catch::Approx(4.0F));
    CHECK(loadedVisual.style.stylisationInkStrength == Catch::Approx(0.55F));
    CHECK(loadedVisual.style.stylisationPaperGrain == Catch::Approx(0.45F));
    CHECK(loadedVisual.style.stylisationPigmentBleed == Catch::Approx(0.6F));
    CHECK(loadedVisual.style.brushAspect == Catch::Approx(3.0F));
    CHECK(loadedVisual.style.strokeJitter == Catch::Approx(0.25F));
    CHECK(loadedVisual.style.hatchStrength == Catch::Approx(0.2F));
    CHECK(loadedVisual.style.strokeOpacityVariance == Catch::Approx(0.4F));
    CHECK(loadedVisual.style.pigmentVariation == Catch::Approx(0.65F));
    CHECK(loadedVisual.style.pigmentAnimationSpeed == Catch::Approx(1.25F));
    CHECK(loadedVisual.style.granulationAngleStrength == Catch::Approx(0.75F));
    CHECK(loadedVisual.style.roughnessMotionStrength == Catch::Approx(0.018F));
    CHECK(loadedVisual.style.roughnessMotionScale == Catch::Approx(2.4F));
    CHECK(loadedVisual.style.roughnessMotionSpeed == Catch::Approx(0.6F));
    CHECK(loadedVisual.style.roughnessMotionThreshold == Catch::Approx(0.62F));
    CHECK(loadedVisual.style.roughnessMotionGroundId == Catch::Approx(0.0F));
    CHECK(loadedVisual.style.exposure == Catch::Approx(2.25F));
    CHECK(loadedVisual.style.innerRadius == Catch::Approx(0.35F));
    CHECK(loadedVisual.style.gaussianSharpness == Catch::Approx(5.5F));
    CHECK(loadedVisual.style.featherPower == Catch::Approx(2.25F));
    CHECK(loadedVisual.style.waterStreakAspect == Catch::Approx(7.5F));
    CHECK(!loadedVisual.style.solidCenters);
    CHECK(loadedVisual.style.flowAnimation);
    CHECK(loadedVisual.style.waterTrailOverlay);
    CHECK(loadedVisual.style.pointSize.fieldMap.fieldSlot == 2);
    CHECK(loadedVisual.style.pointSize.fieldMap.fieldName == "Height");
    CHECK(loadedVisual.style.pointSize.fieldMap.inputMin == Catch::Approx(-2.0F));
    CHECK(loadedVisual.style.pointSize.fieldMap.outputMax == Catch::Approx(8.0F));
    CHECK(invisible_places::style::ScalarConstant(loadedVisual.style.surfelDiameter) == Catch::Approx(0.0125F));
    CHECK(invisible_places::style::ScalarConstant(loadedVisual.style.opacity) == Catch::Approx(0.55F));
    CHECK(!loadedVisual.style.opacity.active);
    CHECK(loadedVisual.style.colormapPosition.fieldMap.fieldName == "Intensity");
    CHECK(!loadedVisual.style.colormapPosition.active);
    REQUIRE(loadedDocument->sceneVisualStates.size() == 3U);
    CHECK(loadedDocument->sceneVisualStates[0].sceneGroupName == "SampleScene");
    CHECK(loadedDocument->sceneVisualStates[0].visual.name == "Warm_SampleScene");
    CHECK(
        loadedDocument->sceneVisualStates[0].visual.style.colorMode ==
        invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor);
    CHECK(loadedDocument->sceneVisualStates[0].visual.style.solidColor[2] == Catch::Approx(0.3F));
    CHECK(loadedDocument->sceneVisualStates[0].visual.style.waterStreakAspect == Catch::Approx(9.0F));
    CHECK(loadedDocument->sceneVisualStates[1].sceneGroupName == "SampleScene");
    CHECK(loadedDocument->sceneVisualStates[1].visual.name == "Cool_SampleScene");
    CHECK(loadedDocument->sceneVisualStates[1].visual.style.exposure == Catch::Approx(0.72F));
    CHECK(loadedDocument->sceneVisualStates[2].sceneGroupName == "Scene2");
    CHECK(loadedDocument->sceneVisualStates[2].visual.name == "Warm_Scene2");
    CHECK(loadedDocument->sceneVisualStates[2].visual.style.exposure == Catch::Approx(1.15F));
    CHECK(
        loadedDocument->gsplatVisualStyle.colorMode ==
        invisible_places::renderer::gsplat::GaussianSplatColorMode::DcOnly);
    CHECK(
        loadedDocument->gsplatVisualStyle.debugMode ==
        invisible_places::renderer::gsplat::GaussianSplatDebugMode::LayerTint);
    CHECK(
        loadedDocument->gsplatVisualStyle.qualityMode ==
        invisible_places::renderer::gsplat::GaussianSplatQualityMode::High);
    CHECK(loadedDocument->gsplatVisualStyle.opacityMultiplier == Catch::Approx(0.66F));
    CHECK(loadedDocument->gsplatVisualStyle.scaleMultiplier == Catch::Approx(1.7F));
    CHECK(loadedDocument->gsplatVisualStyle.exposure == Catch::Approx(1.35F));
    CHECK(loadedDocument->gsplatVisualStyle.saturation == Catch::Approx(0.82F));
    CHECK(loadedDocument->gsplatVisualStyle.layerTint[2] == Catch::Approx(0.6F));

    std::filesystem::remove(projectPathCacheSidecar);
    std::filesystem::remove(outputPath);
}

TEST_CASE("Project document defaults ProRes alpha preview off for older projects", "[serialization][project]") {
    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_legacy_prores_alpha_preview.json";
    {
        std::ofstream output{outputPath, std::ios::trunc};
        output << R"({
  "schema_version": 31,
  "project_name": "Legacy Alpha Preview",
  "background_color": [0.1, 0.2, 0.3, 1.0],
  "layers": []
})";
    }

    std::string errorMessage;
    const auto loadedDocument = invisible_places::serialization::LoadProjectDocument(outputPath, &errorMessage);
    REQUIRE(loadedDocument.has_value());
    CHECK(loadedDocument->schemaVersion == invisible_places::serialization::ProjectDocument{}.schemaVersion);
    CHECK_FALSE(loadedDocument->proResAlphaPreviewEnabled);
    CHECK(
        loadedDocument->orbitControlMode ==
        invisible_places::camera::OrbitControlMode::WorldUp);

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

TEST_CASE("Project document defaults Fast Basic renderer mode for older projects", "[serialization][project]") {
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
        invisible_places::renderer::pointcloud::PointCloudRendererMode::Beauty);
    CHECK(loadedDocument->eyeDomeLightingThickness == Catch::Approx(1.0F));
    std::filesystem::remove(outputPath);
}

TEST_CASE("Legacy Raytraced renderer mode loads as Beauty and is not re-saved", "[serialization][project]") {
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
        invisible_places::renderer::pointcloud::PointCloudRendererMode::Beauty);

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_project_legacy_raytraced_mode_saved.json";
    REQUIRE(invisible_places::serialization::SaveProjectDocument(loadedDocument.value(), outputPath, &errorMessage));
    std::ifstream savedProject{outputPath};
    const std::string savedJson{
        std::istreambuf_iterator<char>{savedProject},
        std::istreambuf_iterator<char>{}};
    CHECK(savedJson.find("\"point_cloud_renderer_mode\": \"beauty\"") != std::string::npos);
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

TEST_CASE("Legacy water stream style keys load as trail overlays", "[serialization][point-style][water]") {
    const auto presetPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_water_stream_style.json";
    std::ofstream output{presetPath, std::ios::trunc};
    output << R"({
  "schema_version": 2,
  "preset_name": "Legacy Water Stream Style",
  "point_style": {
    "water_stream_overlay": true,
    "water_overlay_render_mode": "stream"
  }
})";
    output.close();

    std::string errorMessage;
    const auto preset = invisible_places::serialization::LoadPointCloudStylePreset(presetPath, &errorMessage);
    REQUIRE(preset.has_value());
    CHECK(preset->style.waterTrailOverlay);

    const auto roundTripPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_water_stream_style_roundtrip.json";
    REQUIRE(invisible_places::serialization::SavePointCloudStylePreset(*preset, roundTripPath, &errorMessage));
    {
        std::ifstream savedPreset{roundTripPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedPreset},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find("\"water_trail_overlay\"") != std::string::npos);
        CHECK(savedJson.find("\"water_stream_overlay\"") == std::string::npos);
        CHECK(savedJson.find("\"water_overlay_render_mode\"") == std::string::npos);
    }

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
    document.style.shorelineWaveEnabled = true;
    document.style.shorelineWaveAlgorithm =
        invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm::HeightFoam;
    document.style.shorelineHeightFoam.breakZ = 1.24F;
    document.style.shorelineHeightFoam.offshoreFoamStrength = 0.48F;

    const auto presetPath =
        std::filesystem::temp_directory_path() / "invisible_places_stylisation_style.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SavePointCloudStylePreset(document, presetPath, &errorMessage));
    const auto loaded = invisible_places::serialization::LoadPointCloudStylePreset(presetPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == 3U);
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
    CHECK(loaded->style.shorelineWaveEnabled);
    CHECK(
        loaded->style.shorelineWaveAlgorithm ==
        invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm::HeightFoam);
    CHECK(loaded->style.shorelineHeightFoam.breakZ == Catch::Approx(1.24F));
    CHECK(loaded->style.shorelineHeightFoam.offshoreFoamStrength == Catch::Approx(0.48F));

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

TEST_CASE("Water trail lanes tighten on Z-vertical guide drops", "[water][trail]") {
    const auto makeAnchors = [](bool vertical) {
        invisible_places::water::WaterOverlay anchors;
        for (std::uint32_t index = 0; index < 7U; ++index) {
            invisible_places::water::WaterOverlayPoint point;
            point.position = vertical
                                 ? invisible_places::io::Float3{0.0F, 0.0F, 1.0F - static_cast<float>(index) * 0.045F}
                                 : invisible_places::io::Float3{static_cast<float>(index) * 0.045F, 0.0F, 1.0F};
            point.normal = vertical ? invisible_places::io::Float3{1.0F, 0.0F, 0.0F}
                                    : invisible_places::io::Float3{0.0F, 0.0F, 1.0F};
            point.flowId = 42.0F;
            point.emitterId = 3.0F;
            point.pathDistance = static_cast<float>(index) * 0.045F;
            point.speed = 1.0F;
            point.width = 0.080F;
            point.confidence = 1.0F;
            point.accumulation = static_cast<float>(index) / 6.0F;
            anchors.bounds.Expand(point.position);
            anchors.points.push_back(point);
        }
        return anchors;
    };
    const auto maxLaneOffset = [](const invisible_places::water::WaterOverlay& overlay) {
        float maximum = 0.0F;
        for (const auto& point : overlay.points) {
            if (point.particleRole >= 2.5F && point.particleRole < 3.5F) {
                maximum = std::max(maximum, std::abs(point.trailLateralOffset));
            }
        }
        return maximum;
    };

    auto sourceSettings = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Detail);
    sourceSettings.trailShape.particleJitter = 1.0F;
    sourceSettings.trailShape.trailLooseness = 1.0F;
    sourceSettings.trailShape.trailSmoothness = 0.0F;
    sourceSettings.trailShape.trailLaneCount = 7U;
    sourceSettings.trailShape.splineAnchorSpacing = 0.020F;
    invisible_places::water::WaterAnimationTrailSettings animationSettings;
    animationSettings.particleDensity = 0.0F;

    const auto verticalOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        makeAnchors(true),
        sourceSettings.trailShape,
        animationSettings);
    const auto horizontalOverlay = invisible_places::water::BuildWaterOverlayFromPathAnchors(
        makeAnchors(false),
        sourceSettings.trailShape,
        animationSettings);
    const float verticalMaxOffset = maxLaneOffset(verticalOverlay);
    const float horizontalMaxOffset = maxLaneOffset(horizontalOverlay);

    REQUIRE(verticalMaxOffset > 0.0F);
    REQUIRE(horizontalMaxOffset > 0.0F);
    CHECK(verticalMaxOffset < horizontalMaxOffset * 0.35F);
}

TEST_CASE("Water v2 trails expose deterministic scalar contracts", "[water][v2]") {
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

    invisible_places::water::WaterFlowTrailSettings trailSettings;
    trailSettings.trailCountTotal = 8U;
    trailSettings.trailLengthMeters = 0.36F;
    trailSettings.trailPointSpacingMeters = 0.09F;
    trailSettings.trailWidthMeters = 0.012F;
    trailSettings.trailStreakLengthMeters = 0.050F;
    trailSettings.startFadeEnabled = true;
    trailSettings.startFadeFullDistanceMeters = 0.18F;
    trailSettings.startFadeRandomBeginDistanceMeters = 0.07F;
    trailSettings.endFadeEnabled = true;
    trailSettings.endFadeFullDistanceMeters = 0.24F;
    trailSettings.endFadeRandomBeginDistanceMeters = 0.11F;
    trailSettings.laneSpreadMeters = 0.04F;
    trailSettings.laneCount = 4U;
    trailSettings.laneCrossing = 0.37F;
    trailSettings.turbulence = 0.03F;
    trailSettings.seed = 123U;
    const auto flowA = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, trailSettings);
    const auto flowB = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, trailSettings);
    REQUIRE_FALSE(flowA.samples.empty());
    REQUIRE(flowA.samples.size() == flowB.samples.size());
    CHECK(flowA.samples.front().position.x == Catch::Approx(flowB.samples.front().position.x));
    CHECK(flowA.samples.front().tangent.x == Catch::Approx(flowB.samples.front().tangent.x));
    CHECK(flowA.samples.back().pointSeed == Catch::Approx(flowB.samples.back().pointSeed));

    const auto cloud = invisible_places::water::BuildWaterTrailOverlayPointCloud(
        flowA,
        "Saved/water/test-WaterFlowTrails.generated",
        "flow trails");
    const std::vector<std::string> expectedFields{
        "trail_role",
        "trail_id",
        "source_id",
        "path_id",
        "branch_id",
        "trail_seed",
        "point_seed",
        "trail_distance",
        "trail_length",
        "route_start_index",
        "route_point_count",
        "route_length",
        "trail_start_phase",
        "trail_lateral_offset",
        "point_age",
        "trail_age",
        "trail_speed",
        "trail_width",
        "trail_streak_length",
        "trail_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z",
        "trail_lane_index",
        "trail_lane_count",
        "trail_lane_pitch",
        "trail_lane_span",
        "trail_lane_crossing",
        "trail_cross_seed",
        "endpoint_fade_flags",
        "start_fade_full_distance",
        "start_fade_random_begin_distance",
        "end_fade_full_distance",
        "end_fade_random_begin_distance"};
    REQUIRE(cloud.ScalarFieldCount() == expectedFields.size());
    for (std::size_t index = 0; index < expectedFields.size(); ++index) {
        CHECK(cloud.scalarFields[index].name == expectedFields[index]);
        CHECK(cloud.scalarFields[index].name.find("stream") == std::string::npos);
    }
    REQUIRE(cloud.PointCount() == flowA.samples.size());

    const auto firstRouteSample = std::find_if(
        flowA.samples.begin(),
        flowA.samples.end(),
        [](const invisible_places::water::WaterTrailSample& sample) {
            return sample.trailRole < 0.5F;
        });
    const auto firstVisibleSample = std::find_if(
        flowA.samples.begin(),
        flowA.samples.end(),
        [](const invisible_places::water::WaterTrailSample& sample) {
            return sample.trailRole >= 0.5F;
        });
    REQUIRE(firstRouteSample != flowA.samples.end());
    REQUIRE(firstVisibleSample != flowA.samples.end());
    const auto firstVisibleIndex = static_cast<std::size_t>(
        std::distance(flowA.samples.begin(), firstVisibleSample));
    CHECK(firstRouteSample->routeStartIndex == Catch::Approx(0.0F));
    CHECK(firstRouteSample->routePointCount >= 2.0F);
    CHECK(firstRouteSample->trailConfidence >= 0.0F);
    CHECK(firstVisibleSample->routeStartIndex >= 0.0F);
    CHECK(firstVisibleSample->routePointCount >= 2.0F);
    CHECK(firstVisibleSample->routeStartIndex + firstVisibleSample->routePointCount <=
          static_cast<float>(flowA.samples.size()));
    CHECK(firstVisibleSample->pointAge >= 0.0F);
    CHECK(firstVisibleSample->pointAge <= 1.0F);
    CHECK(firstVisibleSample->trailAge >= 0.0F);
    CHECK(firstVisibleSample->trailAge <= 1.0F);
    CHECK(firstVisibleSample->wetness >= 0.0F);
    CHECK(firstVisibleSample->wetness <= 1.0F);
    CHECK(firstVisibleSample->trailConfidence >= 0.0F);
    CHECK(firstVisibleSample->trailConfidence <= 1.0F);
    const float expectedLanePitch = std::max(trailSettings.trailWidthMeters * 0.5F, 0.00025F);
    const auto expectedLaneCount = trailSettings.laneCount;
    const auto expectedCenterLaneLow = (expectedLaneCount - 1U) / 2U;
    const auto expectedCenterLaneHigh = expectedLaneCount / 2U;
    CHECK(firstVisibleSample->trailLaneIndex >= 0.0F);
    CHECK(firstVisibleSample->trailLaneIndex < static_cast<float>(expectedLaneCount));
    CHECK((firstVisibleSample->trailLaneIndex == Catch::Approx(static_cast<float>(expectedCenterLaneLow)) ||
           firstVisibleSample->trailLaneIndex == Catch::Approx(static_cast<float>(expectedCenterLaneHigh))));
    CHECK(firstVisibleSample->trailLaneCount == Catch::Approx(static_cast<float>(expectedLaneCount)));
    CHECK(firstVisibleSample->trailLanePitch == Catch::Approx(expectedLanePitch));
    CHECK(firstVisibleSample->trailLaneSpan == Catch::Approx(trailSettings.laneSpreadMeters));
    CHECK(firstVisibleSample->trailLaneCrossing == Catch::Approx(trailSettings.laneCrossing));
    CHECK(firstVisibleSample->trailCrossSeed >= 0.0F);
    CHECK(firstVisibleSample->trailCrossSeed <= 1.0F);
    CHECK(firstVisibleSample->endpointFadeFlags == Catch::Approx(3.0F));
    CHECK(firstVisibleSample->startFadeFullDistanceMeters == Catch::Approx(0.18F));
    CHECK(firstVisibleSample->startFadeRandomBeginDistanceMeters == Catch::Approx(0.07F));
    CHECK(firstVisibleSample->endFadeFullDistanceMeters == Catch::Approx(0.24F));
    CHECK(firstVisibleSample->endFadeRandomBeginDistanceMeters == Catch::Approx(0.11F));
    CHECK(std::abs(firstVisibleSample->trailLateralOffset) <= (trailSettings.laneSpreadMeters * 0.5F) + 0.002F);
    CHECK(expectedLaneCount < static_cast<std::uint32_t>(
        std::ceil(trailSettings.laneSpreadMeters / expectedLanePitch)));

    const float firstTrailWidth = cloud.scalarFieldValues[
        cloud.ScalarFieldValueIndex(17, firstVisibleIndex)];
    const float firstTrailStreakLength = cloud.scalarFieldValues[
        cloud.ScalarFieldValueIndex(18, firstVisibleIndex)];
    CHECK(firstTrailWidth >= trailSettings.trailWidthMeters * 0.80F);
    CHECK(firstTrailWidth <= trailSettings.trailWidthMeters * 1.22F);
    CHECK(firstTrailStreakLength >= std::max(trailSettings.trailStreakLengthMeters, trailSettings.trailPointSpacingMeters * 2.5F));
    CHECK(firstTrailStreakLength >= firstTrailWidth * 2.0F);

    auto noCrossSettings = trailSettings;
    noCrossSettings.laneCrossing = 0.0F;
    const auto noCrossA = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, noCrossSettings);
    const auto noCrossB = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, noCrossSettings);
    const auto noCrossVisibleA = std::find_if(
        noCrossA.samples.begin(),
        noCrossA.samples.end(),
        [](const invisible_places::water::WaterTrailSample& sample) {
            return sample.trailRole >= 0.5F;
        });
    const auto noCrossVisibleB = std::find_if(
        noCrossB.samples.begin(),
        noCrossB.samples.end(),
        [](const invisible_places::water::WaterTrailSample& sample) {
            return sample.trailRole >= 0.5F;
        });
    REQUIRE(noCrossVisibleA != noCrossA.samples.end());
    REQUIRE(noCrossVisibleB != noCrossB.samples.end());
    CHECK(noCrossVisibleA->trailLaneCrossing == Catch::Approx(0.0F));
    CHECK(noCrossVisibleA->trailLateralOffset == Catch::Approx(noCrossVisibleB->trailLateralOffset));
}

TEST_CASE("Water Flow lane edits stay outside path bake inputs", "[water][flow][lanes]") {
    const auto source = invisible_places::water::DefaultWaterSourceSettings(
        invisible_places::water::WaterScaleMode::Detail);
    auto refreshOnlySource = source;
    refreshOnlySource.path.smoothing = std::clamp(source.path.smoothing + 0.20F, 0.0F, 1.0F);
    CHECK(invisible_places::water::WaterSourceBakeInputsEqual(source, refreshOnlySource));

    invisible_places::water::WaterFlowTrailSettings lanes;
    auto speedOnly = lanes;
    speedOnly.speedMetersPerSecond *= 1.75F;
    CHECK(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, speedOnly));
    CHECK(invisible_places::water::WaterFlowLaneSpeedOnlyEdit(lanes, speedOnly));

    auto visualOnly = lanes;
    visualOnly.trailWidthMeters *= 1.5F;
    visualOnly.trailStreakLengthMeters *= 1.5F;
    CHECK(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, visualOnly));
    CHECK_FALSE(invisible_places::water::WaterFlowLaneSpeedOnlyEdit(lanes, visualOnly));

    auto speedAndVisual = visualOnly;
    speedAndVisual.speedMetersPerSecond *= 1.75F;
    CHECK(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, speedAndVisual));
    CHECK_FALSE(invisible_places::water::WaterFlowLaneSpeedOnlyEdit(lanes, speedAndVisual));

    auto routeChanging = lanes;
    routeChanging.laneSpreadMeters *= 2.0F;
    CHECK_FALSE(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, routeChanging));
    CHECK_FALSE(invisible_places::water::WaterFlowLaneSpeedOnlyEdit(lanes, routeChanging));
    routeChanging = lanes;
    routeChanging.surfaceFollow *= 0.5F;
    CHECK_FALSE(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, routeChanging));
    routeChanging = lanes;
    routeChanging.downhillPull *= 0.5F;
    CHECK_FALSE(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, routeChanging));
    routeChanging = lanes;
    routeChanging.terrainWidthResponse *= 0.5F;
    CHECK_FALSE(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, routeChanging));
    routeChanging = lanes;
    routeChanging.turbulenceScaleMeters *= 2.0F;
    CHECK_FALSE(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, routeChanging));
    routeChanging = lanes;
    routeChanging.startFadeEnabled = true;
    CHECK_FALSE(invisible_places::water::WaterFlowLaneRouteInputsEqual(lanes, routeChanging));
    CHECK(invisible_places::water::WaterSourceBakeInputsEqual(source, source));

    invisible_places::water::WaterTrailGeometrySettings geometry;
    auto visualGeometry = geometry;
    visualGeometry.widthMeters *= 1.5F;
    visualGeometry.streakLengthMeters *= 1.5F;
    CHECK(invisible_places::water::WaterTrailGeometryGenerationInputsEqual(geometry, visualGeometry));
    CHECK(invisible_places::water::WaterTrailGeometryLiveVisualOnlyEdit(geometry, visualGeometry));

    auto generatedGeometry = geometry;
    generatedGeometry.pointSpacingMeters *= 1.5F;
    CHECK_FALSE(invisible_places::water::WaterTrailGeometryGenerationInputsEqual(geometry, generatedGeometry));
    CHECK_FALSE(invisible_places::water::WaterTrailGeometryLiveVisualOnlyEdit(geometry, generatedGeometry));

    generatedGeometry = geometry;
    generatedGeometry.endFadeEnabled = true;
    generatedGeometry.endFadeFullDistanceMeters = 0.42F;
    CHECK_FALSE(invisible_places::water::WaterTrailGeometryGenerationInputsEqual(geometry, generatedGeometry));
    const auto fadedSettings = invisible_places::water::ApplyWaterTrailGeometryToFlowTrailSettings(
        lanes,
        generatedGeometry);
    CHECK(fadedSettings.endFadeEnabled);
    CHECK(fadedSettings.endFadeFullDistanceMeters == Catch::Approx(0.42F));
}

TEST_CASE("Water Flow trail builds cancel cleanly without publishing partial samples", "[water][flow][performance]") {
    invisible_places::water::WaterOverlay anchors;
    for (std::uint32_t index = 0U; index < 240U; ++index) {
        invisible_places::water::WaterOverlayPoint point;
        point.position = {
            static_cast<float>(index) * 0.006F,
            std::sin(static_cast<float>(index) * 0.08F) * 0.03F,
            1.0F - static_cast<float>(index) * 0.004F,
        };
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = 7.0F;
        point.emitterId = 19.0F;
        point.pathDistance = static_cast<float>(index) * 0.0075F;
        point.confidence = 1.0F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 900U;
    settings.trailLengthMeters = 0.65F;
    settings.trailPointSpacingMeters = 0.008F;

    std::stop_source stopSource;
    stopSource.request_stop();
    const auto stopToken = stopSource.get_token();
    const auto cancelled = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
        anchors,
        settings,
        nullptr,
        invisible_places::water::WaterFlowTrailBuildOptions{
            .stopToken = &stopToken,
        });
    CHECK(cancelled.samples.empty());

    settings.trailCountTotal = 8U;
    const auto completed = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
        anchors,
        settings);
    REQUIRE_FALSE(completed.samples.empty());
    const auto visibleTrailCount = std::count_if(
        completed.samples.begin(),
        completed.samples.end(),
        [](const invisible_places::water::WaterTrailSample& sample) {
            return sample.trailRole >= 0.5F && sample.pointAge <= 1.0e-6F;
        });
    CHECK(visibleTrailCount == 8);

    const auto cancelledCloud = invisible_places::water::BuildWaterTrailOverlayPointCloud(
        completed,
        "cancelled-flow.generated",
        "Cancelled Flow",
        &stopToken);
    CHECK(cancelledCloud.PointCount() == 0U);
    CHECK(cancelledCloud.scalarFields.empty());
}

TEST_CASE("GPU Flow output layout is deterministic and grows source-locally", "[water][flow][gpu]") {
    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 12U;
    settings.laneCount = 3U;
    settings.trailLengthMeters = 0.20F;
    settings.trailPointSpacingMeters = 0.010F;

    const auto first = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        5U,
        1.0F,
        settings);
    REQUIRE(first.Valid());
    CHECK(first.inputPointCount == 5U);
    CHECK(first.laneCount == 3U);
    CHECK(first.routePointCountPerLane == 101U);
    CHECK(first.routePointCountTotal == 303U);
    CHECK(first.samplesPerTrail == 21U);
    CHECK(first.trailPointCountTotal == 252U);
    CHECK(first.pointCount == 555U);
    CHECK(first.pointCapacity == 1024U);

    const auto reused = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        5U,
        1.0F,
        settings,
        2048U);
    REQUIRE(reused.Valid());
    CHECK(reused.pointCount == first.pointCount);
    CHECK(reused.pointCapacity == 2048U);
    CHECK(invisible_places::water::kWaterTrailScalarFieldCount == 36U);
    const auto gpuScalarFields =
        invisible_places::water::WaterTrailOverlayScalarFieldsForPointCount(
            reused.pointCapacity,
            true);
    REQUIRE(gpuScalarFields.size() == 36U);
    CHECK(gpuScalarFields[31U].name == "endpoint_fade_flags");
    CHECK(gpuScalarFields[35U].name == "end_fade_random_begin_distance");

    const auto capped = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        5U,
        1.0F,
        settings,
        0U,
        500U);
    CHECK_FALSE(capped.Valid());
}

TEST_CASE("GPU Flow manual spline input is compact and arc-length aware", "[water][flow][gpu]") {
    const std::vector<invisible_places::io::Float3> controls = {
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
        {0.35F, 0.55F, 0.10F},
        {0.80F, -0.30F, 0.65F},
        {1.20F, 0.10F, 1.10F},
    };
    const auto compact =
        invisible_places::water::BuildWaterFlowGpuManualSplineInput(controls);
    const auto repeated =
        invisible_places::water::BuildWaterFlowGpuManualSplineInput(controls);
    REQUIRE(compact.Valid());
    REQUIRE(compact.points.size() == 4U);
    REQUIRE(repeated.points.size() == compact.points.size());
    CHECK(sizeof(invisible_places::water::WaterFlowGpuCompactInputPoint) <= 64U);
    CHECK(compact.routeLengthMeters == Catch::Approx(repeated.routeLengthMeters));

    float controlChordLength = 0.0F;
    for (std::size_t index = 0U; index + 1U < compact.points.size(); ++index) {
        const auto& point = compact.points[index];
        const auto& next = compact.points[index + 1U];
        CHECK(point.cumulativeDistanceMeters < next.cumulativeDistanceMeters);
        CHECK(point.outgoingSegmentArcDistancesMeters[0U] > 0.0F);
        for (std::size_t checkpoint = 1U; checkpoint < 4U; ++checkpoint) {
            CHECK(
                point.outgoingSegmentArcDistancesMeters[checkpoint] >
                point.outgoingSegmentArcDistancesMeters[checkpoint - 1U]);
        }
        CHECK(
            next.cumulativeDistanceMeters ==
            Catch::Approx(
                point.cumulativeDistanceMeters +
                point.outgoingSegmentArcDistancesMeters[3U]));
        const float dx = next.position.x - point.position.x;
        const float dy = next.position.y - point.position.y;
        const float dz = next.position.z - point.position.z;
        controlChordLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    CHECK(compact.routeLengthMeters >= controlChordLength);
    CHECK(
        compact.points.back().cumulativeDistanceMeters ==
        Catch::Approx(compact.routeLengthMeters));

    const auto compactSource =
        invisible_places::water::BuildWaterFlowGpuManualSplineSourceInput(
            controls,
            77U,
            9U);
    REQUIRE(compactSource.Valid());
    REQUIRE(compactSource.branches.size() == 1U);
    CHECK(compactSource.branches.front().branchId == 77U);
    CHECK(compactSource.branches.front().pathId == 9U);
    CHECK(compactSource.branches.front().inputStart == 0U);
    CHECK(compactSource.branches.front().inputCount == compactSource.points.size());
    CHECK(
        compactSource.branches.front().routeLengthMeters ==
        Catch::Approx(compact.routeLengthMeters));

    invisible_places::water::WaterManualFlowPathSource source;
    source.id = 901U;
    source.controlPoints = controls;
    const auto reference =
        invisible_places::water::BuildManualFlowPathAnchors(source, 0.002F);
    REQUIRE_FALSE(reference.points.empty());
    CHECK(
        compact.routeLengthMeters ==
        Catch::Approx(reference.points.back().pathDistance).epsilon(0.01F));
}

TEST_CASE(
    "GPU Flow compact source keeps branch identities and exact disjoint output ranges",
    "[water][flow][gpu]") {
    invisible_places::water::WaterOverlay anchors;
    const auto addBranch = [&](std::uint32_t branchId, float y, std::uint32_t pointCount) {
        for (std::uint32_t index = 0U; index < pointCount; ++index) {
            invisible_places::water::WaterOverlayPoint point;
            point.position = {static_cast<float>(index) * 0.050F, y, 0.0F};
            point.normal = {0.0F, 0.0F, 1.0F};
            point.flowId = static_cast<float>(branchId);
            point.emitterId = 12.0F;
            point.pathDistance = static_cast<float>(index) * 0.050F;
            point.speed = 1.0F;
            point.width = 0.030F;
            point.confidence = 1.0F;
            point.accumulation = 0.75F;
            anchors.points.push_back(point);
        }
    };
    addBranch(301U, 0.0F, 28U);
    addBranch(302U, 0.25F, 9U);

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 5U;
    settings.laneCount = 4U;
    settings.laneSpreadMeters = 0.030F;
    settings.laneCrossing = 0.45F;
    settings.turbulence = 0.0F;
    settings.speedMetersPerSecond = 0.22F;
    settings.surfaceOffsetMeters = 0.004F;
    settings.seed = 71U;
    settings.trailLengthMeters = 0.18F;
    settings.trailPointSpacingMeters = 0.006F;
    settings.trailWidthMeters = 0.006F;
    settings.trailStreakLengthMeters = 0.030F;

    const auto compact = invisible_places::water::BuildWaterFlowGpuSampledSourceInput(
        anchors.points,
        settings);
    REQUIRE(compact.Valid());
    REQUIRE(compact.branches.size() == 2U);
    CHECK(compact.points.size() == 37U);
    CHECK(compact.branches[0U].inputStart == 0U);
    CHECK(compact.branches[0U].inputCount == 28U);
    CHECK(compact.branches[0U].branchId == 301U);
    CHECK(compact.branches[0U].pathId == 1U);
    CHECK(compact.branches[1U].inputStart == 28U);
    CHECK(compact.branches[1U].inputCount == 9U);
    CHECK(compact.branches[1U].branchId == 302U);
    CHECK(compact.branches[1U].pathId == 2U);

    const auto layout = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        compact,
        settings);
    const auto repeated = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        compact,
        settings);
    REQUIRE(layout.Valid());
    REQUIRE(repeated.Valid());
    REQUIRE(layout.branches.size() == 2U);
    CHECK(layout.branchCount == 2U);
    CHECK(layout.trailCount == settings.trailCountTotal);
    CHECK(layout.branches[0U].trailCount == 3U);
    CHECK(layout.branches[1U].trailCount == 2U);
    CHECK(layout.branches[0U].firstTrailId == 1U);
    CHECK(layout.branches[1U].firstTrailId == 4U);
    CHECK(layout.branches[0U].activeRouteLaneCount == 3U);
    CHECK(layout.branches[1U].activeRouteLaneCount == 2U);
    CHECK(layout.maxActiveRouteLaneCount == 3U);
    CHECK(layout.maxTrailsPerBranch == 3U);

    std::uint32_t expectedRouteStart = 0U;
    std::uint32_t expectedTrailStart = layout.routePointCountTotal;
    for (std::size_t index = 0U; index < layout.branches.size(); ++index) {
        const auto& branch = layout.branches[index];
        const auto& repeatedBranch = repeated.branches[index];
        CHECK(branch.routeStart == expectedRouteStart);
        CHECK(branch.trailOutputStart == expectedTrailStart);
        CHECK(branch.inputStart == compact.branches[index].inputStart);
        CHECK(branch.inputCount == compact.branches[index].inputCount);
        CHECK(branch.branchId == compact.branches[index].branchId);
        CHECK(branch.pathId == compact.branches[index].pathId);
        CHECK(branch.firstTrailId == repeatedBranch.firstTrailId);
        CHECK(branch.trailCount == repeatedBranch.trailCount);
        CHECK(branch.routeStart == repeatedBranch.routeStart);
        CHECK(branch.trailOutputStart == repeatedBranch.trailOutputStart);
        expectedRouteStart += branch.routePointsPerLane * branch.activeRouteLaneCount;
        expectedTrailStart += branch.trailCount * layout.samplesPerTrail;
    }
    CHECK(expectedRouteStart == layout.routePointCountTotal);
    CHECK(expectedTrailStart == layout.pointCount);

    const auto cpuOverlay = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
        anchors,
        settings);
    std::unordered_map<std::uint32_t, std::set<std::uint32_t>> cpuTrailIdsByBranch;
    for (const auto& sample : cpuOverlay.samples) {
        if (sample.trailRole < 0.5F) {
            continue;
        }
        cpuTrailIdsByBranch[static_cast<std::uint32_t>(std::floor(sample.branchId + 0.5F))]
            .insert(static_cast<std::uint32_t>(std::floor(sample.trailId + 0.5F)));
    }
    REQUIRE(cpuTrailIdsByBranch.contains(301U));
    REQUIRE(cpuTrailIdsByBranch.contains(302U));
    CHECK(cpuTrailIdsByBranch.at(301U).size() == layout.branches[0U].trailCount);
    CHECK(cpuTrailIdsByBranch.at(302U).size() == layout.branches[1U].trailCount);

    settings.trailCountTotal = 1U;
    const auto oneTrailCompact =
        invisible_places::water::BuildWaterFlowGpuSampledSourceInput(
            anchors.points,
            settings);
    const auto oneTrailLayout = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        oneTrailCompact,
        settings);
    REQUIRE(oneTrailLayout.Valid());
    REQUIRE(oneTrailLayout.branches.size() == 2U);
    CHECK(oneTrailLayout.branches[0U].trailCount == 1U);
    CHECK(oneTrailLayout.branches[0U].firstTrailId == 1U);
    CHECK(oneTrailLayout.branches[1U].trailCount == 0U);
    CHECK(oneTrailLayout.branches[1U].firstTrailId == 2U);
    CHECK(oneTrailLayout.branches[1U].activeRouteLaneCount == 0U);
    CHECK(
        oneTrailLayout.branches[1U].routeStart ==
        oneTrailLayout.routePointCountTotal);
    CHECK(oneTrailLayout.branches[1U].trailOutputStart == oneTrailLayout.pointCount);

    settings.trailCountTotal = 2U;
    settings.laneCount = 128U;
    const auto wideLaneCompact =
        invisible_places::water::BuildWaterFlowGpuSampledSourceInput(
            anchors.points,
            settings);
    const auto wideLaneLayout = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        wideLaneCompact,
        settings);
    REQUIRE(wideLaneLayout.Valid());
    CHECK(wideLaneLayout.laneCount == 128U);
    REQUIRE(wideLaneLayout.branches.size() == 2U);
    for (const auto& branch : wideLaneLayout.branches) {
        CHECK(branch.potentialLaneCount == 128U);
        CHECK(branch.trailCount == 1U);
        CHECK(branch.activeRouteLaneCount == 1U);
    }
}

TEST_CASE("Manual Flow splines produce deterministic lane-ready anchors and trails", "[water][flow][manual-path]") {
    invisible_places::water::WaterManualFlowPathSource source;
    source.id = 417U;
    source.name = "Vertical ribbon";
    source.laneProfileName = "Narrow Lanes";
    source.trailProfileName = "Silver Threads";
    source.controlPoints = {
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.7F},
        {0.08F, 0.04F, 1.5F},
        {0.12F, 0.20F, 2.1F},
    };

    const auto anchors = invisible_places::water::BuildManualFlowPathAnchors(source, 0.035F);
    const auto repeat = invisible_places::water::BuildManualFlowPathAnchors(source, 0.035F);
    REQUIRE(anchors.points.size() > 20U);
    REQUIRE(repeat.points.size() == anchors.points.size());

    const auto distance = [](const auto& left, const auto& right) {
        const float dx = left.x - right.x;
        const float dy = left.y - right.y;
        const float dz = left.z - right.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    CHECK(distance(anchors.points.front().position, source.controlPoints.front()) < 1.0e-5F);
    CHECK(distance(anchors.points.back().position, source.controlPoints.back()) < 1.0e-5F);
    CHECK(anchors.points.front().pathDistance == Catch::Approx(0.0F));
    CHECK(anchors.points.front().emitterId == Catch::Approx(static_cast<float>(source.id)));
    CHECK(anchors.points.front().flowId == Catch::Approx(static_cast<float>(source.id)));

    for (std::size_t index = 0; index < anchors.points.size(); ++index) {
        const auto& anchor = anchors.points[index];
        const auto& repeatedAnchor = repeat.points[index];
        CHECK(anchor.position.x == Catch::Approx(repeatedAnchor.position.x));
        CHECK(anchor.position.y == Catch::Approx(repeatedAnchor.position.y));
        CHECK(anchor.position.z == Catch::Approx(repeatedAnchor.position.z));
        CHECK(anchor.normal.x == Catch::Approx(repeatedAnchor.normal.x));
        CHECK(anchor.normal.y == Catch::Approx(repeatedAnchor.normal.y));
        CHECK(anchor.normal.z == Catch::Approx(repeatedAnchor.normal.z));
        CHECK(anchor.pathDistance == Catch::Approx(repeatedAnchor.pathDistance));
        if (index > 0U) {
            CHECK(anchor.pathDistance > anchors.points[index - 1U].pathDistance);
        }

        const auto& previous = anchors.points[index > 0U ? index - 1U : index].position;
        const auto& next = anchors.points[index + 1U < anchors.points.size() ? index + 1U : index].position;
        const invisible_places::io::Float3 tangent{
            next.x - previous.x,
            next.y - previous.y,
            next.z - previous.z,
        };
        const float tangentLength = std::max(1.0e-6F, distance(next, previous));
        const float normalLength = std::sqrt(
            anchor.normal.x * anchor.normal.x +
            anchor.normal.y * anchor.normal.y +
            anchor.normal.z * anchor.normal.z);
        CHECK(normalLength == Catch::Approx(1.0F).margin(1.0e-4F));
        const float tangentNormalDot =
            (tangent.x * anchor.normal.x + tangent.y * anchor.normal.y + tangent.z * anchor.normal.z) /
            tangentLength;
        CHECK(std::abs(tangentNormalDot) < 2.0e-3F);
        if (index > 0U) {
            const auto& previousNormal = anchors.points[index - 1U].normal;
            CHECK(
                anchor.normal.x * previousNormal.x +
                    anchor.normal.y * previousNormal.y +
                    anchor.normal.z * previousNormal.z >=
                -1.0e-5F);
        }
    }

    for (const std::size_t controlIndex : {0U, 2U, 3U, 4U}) {
        float nearestDistance = std::numeric_limits<float>::max();
        for (const auto& anchor : anchors.points) {
            nearestDistance = std::min(
                nearestDistance,
                distance(anchor.position, source.controlPoints[controlIndex]));
        }
        CHECK(nearestDistance < 1.0e-4F);
    }

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 17U;
    settings.laneCount = 5U;
    settings.laneSpreadMeters = 0.045F;
    settings.laneCrossing = 0.12F;
    settings.turbulence = 0.0F;
    settings.trailLengthMeters = 0.32F;
    settings.trailPointSpacingMeters = 0.025F;
    settings.trailWidthMeters = 0.006F;
    settings.trailStreakLengthMeters = 0.050F;
    settings.speedMetersPerSecond = 0.21F;
    settings.seed = 91U;
    const auto trails = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    REQUIRE_FALSE(trails.samples.empty());
    std::set<std::uint32_t> visibleTrailIds;
    std::set<std::uint32_t> visibleLaneIndices;
    float minimumLaneOffset = std::numeric_limits<float>::max();
    float maximumLaneOffset = std::numeric_limits<float>::lowest();
    bool checkedStartDirection = false;
    for (const auto& sample : trails.samples) {
        CHECK(sample.sourceId == Catch::Approx(static_cast<float>(source.id)));
        CHECK(sample.trailLaneCount == Catch::Approx(static_cast<float>(settings.laneCount)));
        if (sample.trailRole >= 0.5F) {
            visibleTrailIds.insert(static_cast<std::uint32_t>(std::lround(sample.trailId)));
            visibleLaneIndices.insert(static_cast<std::uint32_t>(std::lround(sample.trailLaneIndex)));
            minimumLaneOffset = std::min(minimumLaneOffset, sample.trailLateralOffset);
            maximumLaneOffset = std::max(maximumLaneOffset, sample.trailLateralOffset);
        } else if (!checkedStartDirection && sample.trailDistance <= 1.0e-5F) {
            const auto& first = anchors.points.front().position;
            const auto& second = anchors.points[1U].position;
            const float forwardDot =
                sample.tangent.x * (second.x - first.x) +
                sample.tangent.y * (second.y - first.y) +
                sample.tangent.z * (second.z - first.z);
            CHECK(forwardDot > 0.0F);
            checkedStartDirection = true;
        }
    }
    CHECK(visibleTrailIds.size() == settings.trailCountTotal);
    CHECK(visibleLaneIndices.size() == settings.laneCount);
    CHECK(*visibleLaneIndices.begin() == 0U);
    CHECK(*visibleLaneIndices.rbegin() == settings.laneCount - 1U);
    CHECK(minimumLaneOffset < -settings.laneSpreadMeters * 0.25F);
    CHECK(maximumLaneOffset > settings.laneSpreadMeters * 0.25F);
    CHECK(checkedStartDirection);
}

TEST_CASE("Manual Flow node lane covers inherit, override, and smoothly scale the global width",
          "[water][flow][manual-path][lane-width]") {
    using invisible_places::water::BuildManualFlowPathAnchors;
    using invisible_places::water::BuildWaterFlowGpuManualSplineInput;
    using invisible_places::water::ResolveWaterManualFlowPathLaneWidth;
    using invisible_places::water::WaterManualFlowPathLaneWidth;
    using invisible_places::water::WaterManualFlowPathLaneWidthMode;
    using invisible_places::water::WaterManualFlowPathSource;

    constexpr float globalLaneCover = 0.50F;
    CHECK(ResolveWaterManualFlowPathLaneWidth(
              {.mode = WaterManualFlowPathLaneWidthMode::Inherit,
               .value = 99.0F},
              globalLaneCover) == Catch::Approx(globalLaneCover));
    CHECK(ResolveWaterManualFlowPathLaneWidth(
              {.mode = WaterManualFlowPathLaneWidthMode::Absolute,
               .value = 0.20F},
              globalLaneCover) == Catch::Approx(0.20F));
    CHECK(ResolveWaterManualFlowPathLaneWidth(
              {.mode = WaterManualFlowPathLaneWidthMode::Relative,
               .value = 0.20F},
              globalLaneCover) == Catch::Approx(0.10F));
    CHECK(ResolveWaterManualFlowPathLaneWidth(
              {.mode = WaterManualFlowPathLaneWidthMode::Relative,
               .value = 2.30F},
              globalLaneCover) == Catch::Approx(1.15F));

    WaterManualFlowPathSource source;
    source.id = 619U;
    source.controlPoints = {
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        {2.0F, 0.0F, 0.0F},
    };
    source.controlPointLaneWidths = {
        {.mode = WaterManualFlowPathLaneWidthMode::Inherit, .value = 1.0F},
        {.mode = WaterManualFlowPathLaneWidthMode::Absolute, .value = 0.20F},
        {.mode = WaterManualFlowPathLaneWidthMode::Relative, .value = 2.30F},
    };

    const auto compact = BuildWaterFlowGpuManualSplineInput(
        source.controlPoints,
        source.controlPointLaneWidths);
    REQUIRE(compact.Valid());
    REQUIRE(compact.points.size() == 3U);
    CHECK(compact.points[0].laneWidth.mode ==
          WaterManualFlowPathLaneWidthMode::Inherit);
    CHECK(compact.points[1].laneWidth.mode ==
          WaterManualFlowPathLaneWidthMode::Absolute);
    CHECK(compact.points[1].laneWidth.value == Catch::Approx(0.20F));
    CHECK(compact.points[2].laneWidth.mode ==
          WaterManualFlowPathLaneWidthMode::Relative);
    CHECK(compact.points[2].laneWidth.value == Catch::Approx(2.30F));

    const auto anchors = BuildManualFlowPathAnchors(
        source,
        0.02F,
        globalLaneCover);
    REQUIRE(anchors.points.size() > 80U);
    const auto closestToX = [&](float x) -> const auto& {
        return *std::min_element(
            anchors.points.begin(),
            anchors.points.end(),
            [x](const auto& left, const auto& right) {
                return std::abs(left.position.x - x) <
                       std::abs(right.position.x - x);
            });
    };
    CHECK(closestToX(0.0F).laneSpanMeters ==
          Catch::Approx(0.50F).margin(1.0e-4F));
    CHECK(closestToX(1.0F).laneSpanMeters ==
          Catch::Approx(0.20F).margin(1.0e-4F));
    CHECK(closestToX(2.0F).laneSpanMeters ==
          Catch::Approx(1.15F).margin(1.0e-4F));
    CHECK(closestToX(0.5F).laneSpanMeters ==
          Catch::Approx(0.35F).margin(0.01F));
    CHECK(closestToX(1.5F).laneSpanMeters ==
          Catch::Approx(0.675F).margin(0.02F));
    for (const auto& anchor : anchors.points) {
        CHECK(anchor.laneSpanMeters >= 0.20F);
        CHECK(anchor.laneSpanMeters <= 1.15F);
    }

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 3U;
    settings.laneCount = 3U;
    settings.laneSpreadMeters = globalLaneCover;
    settings.trailLengthMeters = 2.0F;
    settings.trailPointSpacingMeters = 0.02F;
    settings.trailWidthMeters = 0.004F;
    settings.surfaceFollow = 0.0F;
    settings.turbulence = 0.0F;
    settings.laneCrossing = 0.0F;
    const auto trails =
        invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
            anchors,
            settings);
    REQUIRE_FALSE(trails.samples.empty());
    float narrowSpan = std::numeric_limits<float>::max();
    float wideSpan = 0.0F;
    float startEndpointOffset = 0.0F;
    float endEndpointOffset = 0.0F;
    for (const auto& sample : trails.samples) {
        if (sample.trailRole >= 0.5F) {
            if (sample.position.x < 0.01F) {
                startEndpointOffset = std::max(
                    startEndpointOffset,
                    std::abs(sample.trailLateralOffset));
            }
            if (sample.position.x > 1.99F) {
                endEndpointOffset = std::max(
                    endEndpointOffset,
                    std::abs(sample.trailLateralOffset));
            }
            continue;
        }
        if (sample.position.x > 0.90F && sample.position.x < 1.10F) {
            narrowSpan = std::min(narrowSpan, sample.trailLaneSpan);
        }
        if (sample.position.x > 1.85F) {
            wideSpan = std::max(wideSpan, sample.trailLaneSpan);
        }
    }
    CHECK(narrowSpan < 0.30F);
    CHECK(wideSpan > 0.90F);
    CHECK(startEndpointOffset > 0.10F);
    CHECK(endEndpointOffset > 0.25F);
}

TEST_CASE("Manual Flow turbulence is arc-distance stable and independent of sprite width",
          "[water][flow][manual-path][turbulence]") {
    invisible_places::water::WaterManualFlowPathSource source;
    source.id = 518U;
    source.controlPoints = {
        {0.0F, 0.0F, 0.0F},
        {0.5F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
    };
    const auto anchors = invisible_places::water::BuildManualFlowPathAnchors(source, 0.01F);
    REQUIRE_FALSE(anchors.points.empty());

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 1U;
    settings.laneCount = 1U;
    settings.trailLengthMeters = 1.0F;
    settings.trailPointSpacingMeters = 0.01F;
    settings.trailWidthMeters = 0.001F;
    settings.trailStreakLengthMeters = 0.02F;
    settings.surfaceOffsetMeters = 0.0F;
    settings.laneSpreadMeters = 0.20F;
    settings.laneCrossing = 0.0F;
    settings.trailSmoothness = 0.0F;
    settings.trailLooseness = 0.0F;
    settings.turbulence = 0.80F;
    settings.turbulenceScaleMeters = 0.18F;
    settings.seed = 73U;

    const auto visibleSamples = [](const auto& overlay) {
        std::vector<invisible_places::water::WaterTrailSample> samples;
        for (const auto& sample : overlay.samples) {
            if (sample.trailRole >= 0.5F) {
                samples.push_back(sample);
            }
        }
        return samples;
    };

    const auto fine = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    const auto repeat = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    const auto fineSamples = visibleSamples(fine);
    const auto repeatSamples = visibleSamples(repeat);
    REQUIRE(fineSamples.size() == 101U);
    REQUIRE(repeatSamples.size() == fineSamples.size());
    float maximumWobble = 0.0F;
    for (std::size_t index = 0U; index < fineSamples.size(); ++index) {
        CHECK(fineSamples[index].position.x == Catch::Approx(repeatSamples[index].position.x));
        CHECK(fineSamples[index].position.y == Catch::Approx(repeatSamples[index].position.y));
        CHECK(fineSamples[index].trailLateralOffset ==
              Catch::Approx(repeatSamples[index].trailLateralOffset));
        maximumWobble = std::max(maximumWobble, std::abs(fineSamples[index].trailLateralOffset));
    }
    CHECK(maximumWobble > 0.001F);
    CHECK(fineSamples.front().trailLateralOffset == Catch::Approx(0.0F).margin(1.0e-6F));
    CHECK(fineSamples.back().trailLateralOffset == Catch::Approx(0.0F).margin(1.0e-6F));

    auto coarseSettings = settings;
    coarseSettings.trailPointSpacingMeters = 0.02F;
    const auto coarseSamples = visibleSamples(
        invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, coarseSettings));
    REQUIRE(coarseSamples.size() == 51U);
    for (std::size_t index = 0U; index < coarseSamples.size(); ++index) {
        CAPTURE(index);
        CHECK(coarseSamples[index].trailLateralOffset ==
              Catch::Approx(fineSamples[index * 2U].trailLateralOffset).margin(2.0e-5F));
    }

    auto wideSpriteSettings = settings;
    wideSpriteSettings.trailWidthMeters = 0.020F;
    const auto wideSpriteSamples = visibleSamples(
        invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, wideSpriteSettings));
    REQUIRE(wideSpriteSamples.size() == fineSamples.size());
    for (std::size_t index = 0U; index < fineSamples.size(); ++index) {
        CHECK(wideSpriteSamples[index].trailLateralOffset ==
              Catch::Approx(fineSamples[index].trailLateralOffset).margin(1.0e-6F));
    }
}

TEST_CASE("Flow surface offset remains signed for routes and trails",
          "[water][flow][surface-offset]") {
    invisible_places::water::WaterManualFlowPathSource source;
    source.id = 520U;
    source.controlPoints = {
        {0.0F, 0.0F, 0.0F},
        {0.5F, 0.0F, 0.0F},
    };
    const auto anchors = invisible_places::water::BuildManualFlowPathAnchors(source, 0.02F);
    REQUIRE(anchors.points.size() >= 2U);

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 1U;
    settings.laneCount = 1U;
    settings.trailLengthMeters = 0.25F;
    settings.trailPointSpacingMeters = 0.02F;
    settings.laneSpreadMeters = 0.0F;
    settings.turbulence = 0.0F;
    settings.surfaceOffsetMeters = -0.025F;

    const auto overlay =
        invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    REQUIRE_FALSE(overlay.samples.empty());
    for (const auto& sample : overlay.samples) {
        CHECK(sample.position.z == Catch::Approx(-0.025F).margin(1.0e-5F));
    }
}

TEST_CASE("Manual Flow CPU routes use the shared surface cache with bounded fallback",
          "[water][flow][manual-path][surface-guide]") {
    using invisible_places::water::WaterSurfaceRole;
    using invisible_places::water::WaterSurfaceSample;

    std::vector<WaterSurfaceSample> surfaceSamples;
    for (std::uint32_t station = 0U; station <= 25U; ++station) {
        const float x = static_cast<float>(station) * 0.02F;
        for (std::uint32_t duplicate = 0U; duplicate < 8U; ++duplicate) {
            surfaceSamples.push_back({
                .position = {x, 0.0F, 0.0F},
                .normal = {0.0F, 0.0F, 1.0F},
                .role = WaterSurfaceRole::Rock,
                .roughness = 1.0F,
                .hasRoughness = true,
            });
        }
    }
    const auto surfaceCache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(
        surfaceSamples,
        0.02F);
    REQUIRE_FALSE(surfaceCache.flowSurfaceSurfels.empty());

    invisible_places::water::WaterManualFlowPathSource source;
    source.id = 519U;
    source.controlPoints = {
        {0.0F, 0.0F, 0.04F},
        {0.5F, 0.0F, 0.04F},
        {1.0F, 0.0F, 0.04F},
    };
    const auto anchors = invisible_places::water::BuildManualFlowPathAnchors(source, 0.02F);

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 1U;
    settings.laneCount = 1U;
    settings.trailLengthMeters = 1.0F;
    settings.trailPointSpacingMeters = 0.02F;
    settings.trailWidthMeters = 0.004F;
    settings.trailStreakLengthMeters = 0.02F;
    settings.surfaceOffsetMeters = 0.0F;
    settings.laneSpreadMeters = 0.12F;
    settings.turbulence = 0.0F;
    settings.surfaceFollow = 1.0F;
    settings.downhillPull = 0.0F;
    settings.terrainWidthResponse = 1.0F;

    const auto guided = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
        anchors,
        settings,
        nullptr,
        invisible_places::water::WaterFlowTrailBuildOptions{
            .surfaceCache = &surfaceCache,
            .useSurfaceGuide = true,
        });
    std::vector<invisible_places::water::WaterTrailSample> route;
    for (const auto& sample : guided.samples) {
        if (sample.trailRole < 0.5F) {
            route.push_back(sample);
        }
    }
    REQUIRE(route.size() > 40U);
    CHECK(route.front().position.z == Catch::Approx(0.04F).margin(1.0e-5F));
    CHECK(route.back().position.z == Catch::Approx(0.04F).margin(1.0e-5F));

    float supportedMinimumZ = std::numeric_limits<float>::max();
    float supportedMinimumSpan = settings.laneSpreadMeters;
    bool checkedUnsupportedFallback = false;
    for (const auto& sample : route) {
        if (sample.position.x > 0.15F && sample.position.x < 0.40F) {
            supportedMinimumZ = std::min(supportedMinimumZ, sample.position.z);
            supportedMinimumSpan = std::min(supportedMinimumSpan, sample.trailLaneSpan);
        }
        if (sample.position.x > 0.75F && sample.position.x < 0.90F) {
            CHECK(sample.position.z == Catch::Approx(0.04F).margin(1.0e-4F));
            checkedUnsupportedFallback = true;
        }
        CHECK(sample.position.x >= -1.0e-5F);
        CHECK(sample.position.x <= 1.0F + 1.0e-5F);
        CHECK(sample.normal.z > 0.0F);
    }
    CHECK(supportedMinimumZ < 0.02F);
    CHECK(supportedMinimumSpan < settings.laneSpreadMeters);
    CHECK(checkedUnsupportedFallback);

    const auto unguided = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
        anchors,
        settings,
        nullptr,
        invisible_places::water::WaterFlowTrailBuildOptions{
            .surfaceCache = &surfaceCache,
            .useSurfaceGuide = false,
        });
    for (const auto& sample : unguided.samples) {
        if (sample.trailRole < 0.5F) {
            CHECK(sample.position.z == Catch::Approx(0.04F).margin(1.0e-5F));
        }
    }
}

TEST_CASE("Manual Flow path gizmo math constrains axes and planes and preserves insertion order",
          "[water][flow][manual-path][editor]") {
    using invisible_places::app::manual_flow_path::ClosestRayAxisParameter;
    using invisible_places::app::manual_flow_path::InsertControlPoint;
    using invisible_places::app::manual_flow_path::IntersectRayPlane;
    using invisible_places::app::manual_flow_path::ProjectedAxisDragPoint;
    using invisible_places::app::manual_flow_path::Ray;

    const Ray worldPlaneRay{
        .origin = {1.0F, 2.0F, 5.0F},
        .direction = {0.0F, 0.0F, -1.0F},
    };
    const auto worldPlaneHit = IntersectRayPlane(
        worldPlaneRay,
        glm::vec3{0.0F, 0.0F, 1.0F},
        glm::vec3{0.0F, 0.0F, 1.0F});
    REQUIRE(worldPlaneHit.has_value());
    CHECK(worldPlaneHit->x == Catch::Approx(1.0F));
    CHECK(worldPlaneHit->y == Catch::Approx(2.0F));
    CHECK(worldPlaneHit->z == Catch::Approx(1.0F));

    const glm::vec3 node{1.0F, 2.0F, 3.0F};
    const glm::vec3 camera{5.0F, 6.0F, 9.0F};
    const glm::vec3 cameraPlaneNormal = glm::normalize(camera - node);
    const Ray cameraPlaneRay{
        .origin = camera,
        .direction = glm::normalize(node - camera),
    };
    const auto cameraPlaneHit = IntersectRayPlane(cameraPlaneRay, node, cameraPlaneNormal);
    REQUIRE(cameraPlaneHit.has_value());
    CHECK(cameraPlaneHit->x == Catch::Approx(node.x));
    CHECK(cameraPlaneHit->y == Catch::Approx(node.y));
    CHECK(cameraPlaneHit->z == Catch::Approx(node.z));

    const Ray axisRay{
        .origin = {2.0F, 4.0F, 0.0F},
        .direction = {0.0F, -1.0F, 0.0F},
    };
    const auto axisParameter = ClosestRayAxisParameter(
        axisRay,
        glm::vec3{0.0F, 0.0F, 0.0F},
        glm::vec3{1.0F, 0.0F, 0.0F});
    REQUIRE(axisParameter.has_value());
    CHECK(axisParameter.value() == Catch::Approx(2.0F));
    CHECK_FALSE(ClosestRayAxisParameter(
                    Ray{.origin = {}, .direction = {1.0F, 0.0F, 0.0F}},
                    glm::vec3{},
                    glm::vec3{1.0F, 0.0F, 0.0F})
                    .has_value());

    const auto fallbackPoint = ProjectedAxisDragPoint(
        glm::vec3{1.0F, 2.0F, 3.0F},
        glm::vec3{0.0F, 1.0F, 0.0F},
        glm::vec2{10.0F, 10.0F},
        glm::vec2{10.0F, 30.0F},
        glm::vec2{0.0F, 1.0F},
        10.0F);
    CHECK(fallbackPoint.x == Catch::Approx(1.0F));
    CHECK(fallbackPoint.y == Catch::Approx(4.0F));
    CHECK(fallbackPoint.z == Catch::Approx(3.0F));

    std::vector<invisible_places::io::Float3> controls{
        {0.0F, 0.0F, 0.0F},
        {2.0F, 0.0F, 0.0F},
        {3.0F, 0.0F, 0.0F},
    };
    const auto insertedIndex = InsertControlPoint(
        &controls,
        0U,
        {1.0F, 0.0F, 0.0F});
    REQUIRE(insertedIndex.has_value());
    CHECK(insertedIndex.value() == 1U);
    REQUIRE(controls.size() == 4U);
    CHECK(controls[0].x == Catch::Approx(0.0F));
    CHECK(controls[1].x == Catch::Approx(1.0F));
    CHECK(controls[2].x == Catch::Approx(2.0F));
    CHECK_FALSE(InsertControlPoint(&controls, controls.size(), {4.0F, 0.0F, 0.0F}).has_value());
}

TEST_CASE("Water Flow trail point spacing creates dense moving trail samples", "[water][flow][trail]") {
    invisible_places::water::WaterOverlay anchors;
    constexpr std::uint32_t branchId = 101U;
    for (std::uint32_t index = 0; index <= 80U; ++index) {
        invisible_places::water::WaterOverlayPoint point;
        point.position = {static_cast<float>(index) * 0.020F, 0.0F, 0.0F};
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = static_cast<float>(branchId);
        point.emitterId = 1.0F;
        point.pathDistance = static_cast<float>(index) * 0.020F;
        point.speed = 1.0F;
        point.width = 0.030F;
        point.confidence = 1.0F;
        point.accumulation = 0.75F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 97U;
    settings.laneCount = 39U;
    settings.laneSpreadMeters = 0.030F;
    settings.laneCrossing = 1.0F;
    settings.turbulence = 0.0F;
    settings.speedMetersPerSecond = 0.17F;
    settings.surfaceOffsetMeters = 0.004F;
    settings.seed = 37U;
    settings.trailLengthMeters = 0.24F;
    settings.trailPointSpacingMeters = 0.004F;
    settings.trailWidthMeters = 0.011F;
    settings.trailStreakLengthMeters = 0.022F;

    const auto overlay = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    REQUIRE_FALSE(overlay.samples.empty());

    const auto expectedSamplesPerTrail = std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(
            std::ceil(settings.trailLengthMeters / settings.trailPointSpacingMeters)) +
            1U);
    std::unordered_map<std::uint32_t, std::vector<float>> distancesByTrail;
    std::unordered_map<std::uint32_t, std::vector<float>> visibleStationsByTrail;
    std::size_t visibleSampleCount = 0U;
    for (const auto& sample : overlay.samples) {
        if (sample.trailRole < 0.5F) {
            continue;
        }
        const auto trailId = static_cast<std::uint32_t>(
            std::max(0.0F, std::floor(sample.trailId + 0.5F)));
        distancesByTrail[trailId].push_back(sample.trailDistance);
        ++visibleSampleCount;
        CHECK(sample.trailLaneCount == Catch::Approx(static_cast<float>(settings.laneCount)));
        CHECK(sample.trailWidth >= settings.trailWidthMeters * 0.80F);
        CHECK(sample.trailStreakLength >= settings.trailStreakLengthMeters);
        CHECK(sample.trailStreakLength >= settings.trailPointSpacingMeters * 2.5F);
        const float trailStartPhase =
            sample.trailStartPhase + sample.trailAge -
            std::floor(sample.trailStartPhase + sample.trailAge);
        const float openRoutePhase =
            trailStartPhase + sample.trailDistance / std::max(sample.routeLength, 0.001F);
        if (openRoutePhase <= 1.0F) {
            visibleStationsByTrail[trailId].push_back(openRoutePhase * sample.routeLength);
        }
    }

    CHECK(distancesByTrail.size() == settings.trailCountTotal);
    CHECK(visibleSampleCount == distancesByTrail.size() * expectedSamplesPerTrail);
    for (auto& [trailId, distances] : distancesByTrail) {
        CAPTURE(trailId);
        REQUIRE(distances.size() == expectedSamplesPerTrail);
        std::sort(distances.begin(), distances.end());
        CHECK(distances.front() == Catch::Approx(0.0F));
        CHECK(distances.back() >= settings.trailLengthMeters - settings.trailPointSpacingMeters);
        float largestGap = 0.0F;
        for (std::size_t index = 1U; index < distances.size(); ++index) {
            largestGap = std::max(largestGap, distances[index] - distances[index - 1U]);
        }
        CHECK(largestGap <= settings.trailPointSpacingMeters * 1.01F);
    }
    std::size_t checkedVisibleTrails = 0U;
    for (auto& [trailId, stations] : visibleStationsByTrail) {
        if (stations.size() < 2U) {
            continue;
        }
        CAPTURE(trailId);
        std::sort(stations.begin(), stations.end());
        float largestVisibleGap = 0.0F;
        for (std::size_t index = 1U; index < stations.size(); ++index) {
            largestVisibleGap = std::max(largestVisibleGap, stations[index] - stations[index - 1U]);
        }
        CHECK(largestVisibleGap <= settings.trailPointSpacingMeters * 1.01F);
        ++checkedVisibleTrails;
    }
    CHECK(checkedVisibleTrails > 0U);
}

TEST_CASE("Water Flow trail count is exact per source across branches", "[water][flow][trail]") {
    invisible_places::water::WaterOverlay anchors;
    const auto addBranch = [&](std::uint32_t branchId, float y, std::uint32_t pointCount) {
        for (std::uint32_t index = 0; index < pointCount; ++index) {
            invisible_places::water::WaterOverlayPoint point;
            point.position = {static_cast<float>(index) * 0.050F, y, 0.0F};
            point.normal = {0.0F, 0.0F, 1.0F};
            point.flowId = static_cast<float>(branchId);
            point.emitterId = 12.0F;
            point.pathDistance = static_cast<float>(index) * 0.050F;
            point.speed = 1.0F;
            point.width = 0.030F;
            point.confidence = 1.0F;
            point.accumulation = 0.75F;
            anchors.bounds.Expand(point.position);
            anchors.points.push_back(point);
        }
    };
    addBranch(301U, 0.0F, 28U);
    addBranch(302U, 0.25F, 9U);

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 1U;
    settings.laneCount = 4U;
    settings.laneSpreadMeters = 0.030F;
    settings.laneCrossing = 0.45F;
    settings.turbulence = 0.0F;
    settings.speedMetersPerSecond = 0.22F;
    settings.surfaceOffsetMeters = 0.004F;
    settings.seed = 71U;
    settings.trailLengthMeters = 0.18F;
    settings.trailPointSpacingMeters = 0.006F;
    settings.trailWidthMeters = 0.006F;
    settings.trailStreakLengthMeters = 0.030F;

    const auto oneTrailOverlay =
        invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    std::set<std::uint32_t> oneTrailIds;
    std::set<std::uint32_t> oneTrailBranches;
    for (const auto& sample : oneTrailOverlay.samples) {
        if (sample.trailRole < 0.5F) {
            continue;
        }
        oneTrailIds.insert(static_cast<std::uint32_t>(std::floor(sample.trailId + 0.5F)));
        oneTrailBranches.insert(static_cast<std::uint32_t>(std::floor(sample.branchId + 0.5F)));
    }
    CHECK(oneTrailIds.size() == 1U);
    CHECK(oneTrailBranches == std::set<std::uint32_t>{301U});

    settings.trailCountTotal = 5U;
    const auto multiTrailOverlay =
        invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    const auto repeatOverlay =
        invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(anchors, settings);
    REQUIRE(multiTrailOverlay.samples.size() == repeatOverlay.samples.size());

    std::unordered_map<std::uint32_t, std::set<std::uint32_t>> trailsByBranch;
    std::unordered_map<std::uint32_t, float> startPhaseByTrail;
    for (std::size_t index = 0; index < multiTrailOverlay.samples.size(); ++index) {
        const auto& sample = multiTrailOverlay.samples[index];
        const auto& repeat = repeatOverlay.samples[index];
        CHECK(sample.trailId == Catch::Approx(repeat.trailId));
        CHECK(sample.branchId == Catch::Approx(repeat.branchId));
        CHECK(sample.trailStartPhase == Catch::Approx(repeat.trailStartPhase));
        if (sample.trailRole < 0.5F) {
            continue;
        }
        const auto trailId = static_cast<std::uint32_t>(std::floor(sample.trailId + 0.5F));
        const auto branchId = static_cast<std::uint32_t>(std::floor(sample.branchId + 0.5F));
        trailsByBranch[branchId].insert(trailId);
        startPhaseByTrail.try_emplace(trailId, sample.trailStartPhase);
    }

    std::set<std::uint32_t> allTrailIds;
    for (const auto& [branchId, trailIds] : trailsByBranch) {
        allTrailIds.insert(trailIds.begin(), trailIds.end());
    }
    CHECK(allTrailIds.size() == settings.trailCountTotal);
    REQUIRE(trailsByBranch.contains(301U));
    REQUIRE(trailsByBranch.contains(302U));
    CHECK(trailsByBranch.at(301U).size() > trailsByBranch.at(302U).size());

    std::vector<float> longBranchStartPhases;
    for (const auto trailId : trailsByBranch.at(301U)) {
        longBranchStartPhases.push_back(startPhaseByTrail.at(trailId));
    }
    REQUIRE(longBranchStartPhases.size() >= 2U);
    std::sort(longBranchStartPhases.begin(), longBranchStartPhases.end());
    CHECK(longBranchStartPhases.back() - longBranchStartPhases.front() > 0.20F);
}

TEST_CASE("Water Flow trail auto fit keeps moving trails continuous", "[water][flow][trail]") {
    invisible_places::water::WaterTrailGeometrySettings defaultGeometry;
    defaultGeometry.trailLengthMeters = 0.75F;
    defaultGeometry.widthMeters = 0.006F;
    defaultGeometry.pointSpacingMeters = 0.20F;
    defaultGeometry.streakLengthMeters = 0.002F;

    const auto fittedDefault =
        invisible_places::water::FitWaterTrailGeometryForContinuousLines(defaultGeometry);
    CHECK(fittedDefault.pointSpacingMeters == Catch::Approx(0.0099F));
    CHECK(fittedDefault.streakLengthMeters == Catch::Approx(0.045F));
    CHECK(fittedDefault.streakLengthMeters >= fittedDefault.pointSpacingMeters * 3.0F);

    auto longGeometry = defaultGeometry;
    longGeometry.trailLengthMeters = 5.0F;
    const auto fittedLong =
        invisible_places::water::FitWaterTrailGeometryForContinuousLines(longGeometry);
    const auto longSampleCount =
        static_cast<std::uint32_t>(
            std::ceil(fittedLong.trailLengthMeters / fittedLong.pointSpacingMeters)) +
        1U;
    CHECK(longSampleCount <= 130U);
    CHECK(fittedLong.streakLengthMeters >= fittedLong.pointSpacingMeters * 3.0F);

    auto wideGeometry = defaultGeometry;
    wideGeometry.widthMeters = 0.040F;
    const auto fittedWide =
        invisible_places::water::FitWaterTrailGeometryForContinuousLines(wideGeometry);
    CHECK(fittedWide.pointSpacingMeters >= fittedWide.widthMeters * 1.60F);
    CHECK(fittedWide.streakLengthMeters >= fittedWide.widthMeters * 7.0F);
    CHECK(fittedWide.streakLengthMeters >= fittedWide.pointSpacingMeters * 3.0F);
}

TEST_CASE("Calm Water Flow lanes keep path analysis as a bounded guide", "[water][flow][lanes]") {
    invisible_places::water::WaterOverlay anchors;
    constexpr std::uint32_t branchId = 77U;
    for (std::uint32_t index = 0; index < 8U; ++index) {
        invisible_places::water::WaterOverlayPoint point;
        point.position = {static_cast<float>(index) * 0.08F, 0.0F, 0.0F};
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = static_cast<float>(branchId);
        point.emitterId = 11.0F;
        point.pathDistance = static_cast<float>(index) * 0.08F;
        point.speed = 1.0F;
        point.width = 0.060F;
        point.confidence = 0.95F;
        point.accumulation = 0.70F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::water::WaterPathAnalysisCache analysis;
    invisible_places::water::WaterPathBranchAnalysis branchAnalysis;
    branchAnalysis.branchId = branchId;
    for (std::uint32_t index = 0; index < 8U; ++index) {
        invisible_places::water::WaterPathAnalysisSample sample;
        sample.branchId = branchId;
        sample.sampleIndex = index;
        sample.pathDistance = static_cast<float>(index) * 0.08F;
        sample.slope = 0.18F;
        sample.flatness = 0.82F;
        sample.curvature = 0.68F;
        sample.neighborDensity = 0.95F;
        sample.nearestPathDistance = 0.020F;
        sample.confluence = 0.92F;
        sample.channelWidth = 1.20F;
        sample.speed = 0.55F;
        sample.turbulence = 0.88F;
        sample.eddyPotential = 0.72F;
        sample.ripplePotential = 0.76F;
        branchAnalysis.samples.push_back(sample);
    }
    analysis.branches.push_back(branchAnalysis);

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 24U;
    settings.laneCount = 5U;
    settings.trailLengthMeters = 0.30F;
    settings.trailPointSpacingMeters = 0.05F;
    settings.trailWidthMeters = 0.004F;
    settings.trailStreakLengthMeters = 0.011F;
    settings.laneSpreadMeters = 0.080F;
    settings.laneCrossing = 0.06F;
    settings.turbulence = 0.015F;
    settings.pathAttraction = 0.85F;
    settings.trailLooseness = 0.08F;
    settings.speedMetersPerSecond = 0.24F;
    settings.seed = 11U;

    const auto overlay = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
        anchors,
        settings,
        &analysis);
    REQUIRE_FALSE(overlay.samples.empty());

    float maxSpan = 0.0F;
    float maxAbsOffset = 0.0F;
    std::size_t visibleCount = 0U;
    for (const auto& sample : overlay.samples) {
        if (sample.trailRole < 0.5F) {
            continue;
        }
        maxSpan = std::max(maxSpan, sample.trailLaneSpan);
        maxAbsOffset = std::max(maxAbsOffset, std::abs(sample.trailLateralOffset));
        ++visibleCount;
    }
    REQUIRE(visibleCount > 0U);
    CHECK(maxSpan <= settings.laneSpreadMeters * 1.15F);
    CHECK(maxAbsOffset <= settings.laneSpreadMeters * 0.58F);
}

TEST_CASE("Water Flow lanes consume path analysis width and speed variation", "[water][flow][lanes]") {
    invisible_places::water::WaterOverlay anchors;
    constexpr std::uint32_t branchId = 42U;
    for (std::uint32_t index = 0; index < 9U; ++index) {
        invisible_places::water::WaterOverlayPoint point;
        point.position = {static_cast<float>(index) * 0.10F, 0.0F, 0.0F};
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = static_cast<float>(branchId);
        point.emitterId = 3.0F;
        point.pathDistance = static_cast<float>(index) * 0.10F;
        point.speed = 1.0F;
        point.width = 0.030F;
        point.confidence = 0.95F;
        point.accumulation = 0.65F;
        anchors.bounds.Expand(point.position);
        anchors.points.push_back(point);
    }

    invisible_places::water::WaterPathAnalysisCache analysis;
    analysis.analysisRadiusMeters = 0.20F;
    invisible_places::water::WaterPathBranchAnalysis branchAnalysis;
    branchAnalysis.branchId = branchId;
    for (std::uint32_t index = 0; index < 9U; ++index) {
        invisible_places::water::WaterPathAnalysisSample sample;
        sample.branchId = branchId;
        sample.sampleIndex = index;
        sample.pathDistance = static_cast<float>(index) * 0.10F;
        const bool broadFast = index >= 5U;
        sample.slope = broadFast ? 0.80F : 0.12F;
        sample.flatness = broadFast ? 0.20F : 0.82F;
        sample.curvature = broadFast ? 0.72F : 0.05F;
        sample.neighborDensity = broadFast ? 0.88F : 0.10F;
        sample.nearestPathDistance = broadFast ? 0.035F : 0.25F;
        sample.confluence = broadFast ? 0.82F : 0.04F;
        sample.channelWidth = broadFast ? 0.260F : 0.026F;
        sample.speed = broadFast ? 1.85F : 0.42F;
        sample.turbulence = broadFast ? 0.86F : 0.04F;
        sample.eddyPotential = broadFast ? 0.72F : 0.02F;
        sample.ripplePotential = broadFast ? 0.78F : 0.03F;
        branchAnalysis.samples.push_back(sample);
    }
    analysis.branches.push_back(branchAnalysis);

    invisible_places::water::WaterFlowTrailSettings settings;
    settings.trailCountTotal = 36U;
    settings.laneCount = 9U;
    settings.trailLengthMeters = 0.42F;
    settings.trailPointSpacingMeters = 0.07F;
    settings.trailWidthMeters = 0.010F;
    settings.laneSpreadMeters = 0.050F;
    settings.turbulence = 0.02F;
    settings.laneCrossing = 0.45F;
    settings.pathAttraction = 0.72F;
    settings.trailLooseness = 0.20F;
    settings.speedMetersPerSecond = 0.55F;
    settings.seed = 501U;

    const auto overlay = invisible_places::water::BuildFlowTrailOverlayFromPathAnchors(
        anchors,
        settings,
        &analysis);
    REQUIRE_FALSE(overlay.samples.empty());

    struct LaneStats {
        float spanSum = 0.0F;
        float speedSum = 0.0F;
        float maxAbsOffset = 0.0F;
        std::size_t count = 0U;
    };
    LaneStats upstream;
    LaneStats downstream;
    for (const auto& sample : overlay.samples) {
        if (sample.trailRole < 0.5F) {
            continue;
        }
        const float station = sample.trailStartPhase * sample.routeLength + sample.trailDistance;
        auto* stats = station < 0.30F ? &upstream : (station > 0.56F ? &downstream : nullptr);
        if (stats == nullptr) {
            continue;
        }
        stats->spanSum += sample.trailLaneSpan;
        stats->speedSum += sample.trailSpeed;
        stats->maxAbsOffset = std::max(stats->maxAbsOffset, std::abs(sample.trailLateralOffset));
        ++stats->count;
    }
    REQUIRE(upstream.count > 0U);
    REQUIRE(downstream.count > 0U);
    const float upstreamSpan = upstream.spanSum / static_cast<float>(upstream.count);
    const float downstreamSpan = downstream.spanSum / static_cast<float>(downstream.count);
    const float upstreamSpeed = upstream.speedSum / static_cast<float>(upstream.count);
    const float downstreamSpeed = downstream.speedSum / static_cast<float>(downstream.count);
    CHECK(downstreamSpan > upstreamSpan * 2.0F);
    CHECK(downstreamSpeed > upstreamSpeed * 2.0F);
    CHECK(downstream.maxAbsOffset > upstream.maxAbsOffset * 1.4F);
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

TEST_CASE("Triangle mesh PLY loader reads vertex normals and face lists", "[mesh][water]") {
    const auto meshPath = std::filesystem::temp_directory_path() / "invisible_places_triangle_mesh_loader.ply";
    const std::vector<invisible_places::io::Float3> vertices{
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        {1.0F, 1.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
    };
    const std::vector<invisible_places::io::Float3> normals(vertices.size(), {0.0F, 0.0F, 1.0F});
    WriteSyntheticTriangleMeshPly(
        meshPath,
        vertices,
        {{0U, 1U, 2U}, {0U, 2U, 3U}},
        normals);

    const auto header = invisible_places::io::ParsePlyHeader(meshPath);
    REQUIRE(header.success);
    CHECK(header.header.vertexCount == 4U);
    CHECK(header.header.faceCount == 2U);
    CHECK(header.header.properties.size() == 6U);
    REQUIRE(header.header.faceProperties.size() == 1U);
    CHECK(header.header.faceProperties.front().isList);
    CHECK(header.header.faceProperties.front().listCountType == "uchar");
    CHECK(header.header.faceProperties.front().listValueType == "int");

    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);
    CHECK(loaded.mesh.VertexCount() == 4U);
    CHECK(loaded.mesh.TriangleCount() == 2U);
    CHECK(loaded.mesh.hasNormals);
    CHECK(loaded.mesh.triangles.front().indices[0] == 0U);
    CHECK(loaded.mesh.triangles.front().indices[2] == 2U);
}

TEST_CASE("Mesh surface cache projects flat sloped and ambiguous surfaces", "[mesh][water]") {
    const auto meshPath = std::filesystem::temp_directory_path() / "invisible_places_surface_cache_mesh.ply";
    const std::vector<invisible_places::io::Float3> vertices{
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 1.0F},
        {1.0F, 1.0F, 1.0F},
        {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 0.50F},
        {1.0F, 0.0F, 1.50F},
        {1.0F, 1.0F, 1.50F},
        {0.0F, 1.0F, 0.50F},
    };
    WriteSyntheticTriangleMeshPly(
        meshPath,
        vertices,
        {{0U, 1U, 2U}, {0U, 2U, 3U}, {4U, 5U, 6U}, {4U, 6U, 7U}});
    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);

    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.cacheCellSizeMeters = 0.20F;
    settings.projectionSearchRadiusMeters = 0.30F;
    settings.ambiguityHeightMeters = 0.10F;
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());

    const auto projection = invisible_places::water::ProjectToMeshSurface(cache, {0.50F, 0.50F, 0.0F});
    REQUIRE(projection.hit);
    CHECK(projection.ambiguous);
    CHECK(projection.downhill.x < -0.20F);
}

TEST_CASE("Mesh surface cache covers large triangles beyond centroid", "[mesh][water]") {
    const auto meshPath = std::filesystem::temp_directory_path() / "invisible_places_large_triangle_mesh.ply";
    const std::vector<invisible_places::io::Float3> vertices{
        {0.0F, 0.0F, 0.0F},
        {5.0F, 0.0F, 0.0F},
        {0.0F, 5.0F, 0.0F},
    };
    WriteSyntheticTriangleMeshPly(meshPath, vertices, {{0U, 1U, 2U}});
    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);

    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.cacheCellSizeMeters = 0.10F;
    settings.projectionSearchRadiusMeters = 0.25F;
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());

    const auto projection = invisible_places::water::ProjectToMeshSurface(cache, {0.45F, 0.45F, 0.0F});
    REQUIRE(projection.hit);
    CHECK(projection.position.x == Catch::Approx(0.45F));
    CHECK(projection.position.y == Catch::Approx(0.45F));
}

TEST_CASE("Dynamic mesh flow crosses from coarse triangles into dense tiny rock triangles", "[mesh][water]") {
    const auto meshPath = std::filesystem::temp_directory_path() / "invisible_places_mixed_triangle_scale_mesh.ply";
    std::vector<invisible_places::io::Float3> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
    auto heightAt = [](float x, float y) {
        const float macro = 0.22F - 0.18F * x + 0.018F * y;
        const float rough =
            0.014F *
            std::sin((x - 0.40F) * 157.0796327F) *
            std::sin(y * 157.0796327F);
        return macro + rough;
    };

    vertices.push_back({0.0F, 0.0F, heightAt(0.0F, 0.0F)});
    vertices.push_back({0.40F, 0.0F, heightAt(0.40F, 0.0F)});
    vertices.push_back({0.40F, 0.50F, heightAt(0.40F, 0.50F)});
    vertices.push_back({0.0F, 0.50F, heightAt(0.0F, 0.50F)});
    triangles.push_back({{0U, 1U, 2U}});
    triangles.push_back({{0U, 2U, 3U}});

    constexpr float kPatchStartX = 0.40F;
    constexpr float kPatchStep = 0.01F;
    constexpr std::uint32_t kPatchColumns = 80U;
    constexpr std::uint32_t kPatchRows = 50U;
    const auto patchVertexStart = static_cast<std::uint32_t>(vertices.size());
    for (std::uint32_t row = 0; row <= kPatchRows; ++row) {
        const float y = static_cast<float>(row) * kPatchStep;
        for (std::uint32_t column = 0; column <= kPatchColumns; ++column) {
            const float x = kPatchStartX + static_cast<float>(column) * kPatchStep;
            vertices.push_back({x, y, heightAt(x, y)});
        }
    }
    const std::uint32_t patchStride = kPatchColumns + 1U;
    for (std::uint32_t row = 0; row < kPatchRows; ++row) {
        for (std::uint32_t column = 0; column < kPatchColumns; ++column) {
            const std::uint32_t a = patchVertexStart + row * patchStride + column;
            const std::uint32_t b = a + 1U;
            const std::uint32_t c = a + patchStride;
            const std::uint32_t d = c + 1U;
            triangles.push_back({{a, b, d}});
            triangles.push_back({{a, d, c}});
        }
    }

    WriteSyntheticTriangleMeshPly(meshPath, vertices, triangles);
    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);

    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.enabled = true;
    settings.cacheCellSizeMeters = 0.08F;
    settings.projectionSearchRadiusMeters = 0.18F;
    settings.previewParticleLimit = 12U;
    settings.trailLengthMeters = 0.85F;
    settings.stepMeters = 0.025F;
    settings.downhillWeight = 1.35F;
    settings.attractorWeight = 0.0F;
    settings.sourceVelocityWeight = 0.0F;
    settings.curlStrength = 0.0F;
    settings.branchingStrength = 0.0F;
    settings.eddyStrength = 0.0F;
    settings.inertia = 0.30F;
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());

    float worstProjectionError = 0.0F;
    for (std::uint32_t index = 0; index < 10U; ++index) {
        const float x = 0.44F + static_cast<float>(index) * 0.075F;
        const float y = 0.25F;
        const float expectedZ = heightAt(x, y);
        const auto projection = invisible_places::water::ProjectToMeshSurface(cache, {x, y, expectedZ});
        const int baseCellX = static_cast<int>(std::floor(x / settings.cacheCellSizeMeters));
        const int baseCellY = static_cast<int>(std::floor(y / settings.cacheCellSizeMeters));
        std::size_t nearbyCellCount = 0U;
        for (const auto& cell : cache.cells) {
            if (std::abs(cell.cellX - baseCellX) <= 3 && std::abs(cell.cellY - baseCellY) <= 3) {
                ++nearbyCellCount;
            }
        }
        CAPTURE(index, x, y, expectedZ, baseCellX, baseCellY, nearbyCellCount);
        REQUIRE(projection.hit);
        worstProjectionError = std::max(worstProjectionError, std::abs(projection.position.z - expectedZ));
    }
    CAPTURE(worstProjectionError);
    CHECK(worstProjectionError < 0.035F);

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 77U;
    emitter.name = "coarse to rock";
    emitter.position = {0.12F, 0.25F, heightAt(0.12F, 0.25F)};
    emitter.radius = 0.004F;
    emitter.strength = 1.0F;
    emitter.speed = 1.0F;
    invisible_places::water::WaterDynamicMeshFlowDiagnostics diagnostics;
    const auto overlay = invisible_places::water::BuildDynamicMeshWaterTrailOverlay(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings,
        invisible_places::water::WaterTrailBuildQuality::Preview,
        &diagnostics);
    REQUIRE_FALSE(overlay.samples.empty());

    float maxX = 0.0F;
    std::size_t tinyPatchSampleCount = 0U;
    for (const auto& sample : overlay.samples) {
        maxX = std::max(maxX, sample.position.x);
        if (sample.position.x > 0.45F) {
            ++tinyPatchSampleCount;
        }
    }
    CAPTURE(diagnostics.projectionMissCount, diagnostics.ambiguousHitCount, maxX, tinyPatchSampleCount);
    CHECK(maxX > 0.75F);
    CHECK(tinyPatchSampleCount > 12U);
}

TEST_CASE("Mesh surface ray projection lands on hidden cache under cursor", "[mesh][water]") {
    const auto meshPath = std::filesystem::temp_directory_path() / "invisible_places_mesh_ray_projection.ply";
    const std::vector<invisible_places::io::Float3> vertices{
        {0.0F, 0.0F, 0.0F},
        {4.0F, 0.0F, 0.40F},
        {4.0F, 4.0F, 0.80F},
        {0.0F, 4.0F, 0.40F},
    };
    WriteSyntheticTriangleMeshPly(meshPath, vertices, {{0U, 1U, 2U}, {0U, 2U, 3U}});
    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);

    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.cacheCellSizeMeters = 0.20F;
    settings.projectionSearchRadiusMeters = 0.35F;
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());

    const auto projection = invisible_places::water::ProjectRayToMeshSurface(
        cache,
        {1.20F, 1.50F, 10.0F},
        {0.0F, 0.0F, -1.0F});
    REQUIRE(projection.hit);
    CHECK(projection.position.x == Catch::Approx(1.20F));
    CHECK(projection.position.y == Catch::Approx(1.50F));
    CHECK(projection.position.z == Catch::Approx(0.27F).margin(0.08F));
}

TEST_CASE("SampleScene dynamic mesh cache follows mixed triangle scales", "[mesh][water][sample][.]") {
    const auto meshPath =
        DataRoot() / "SampleScene" / "Site1-Mesh-SampleScene.ply";
    if (!std::filesystem::exists(meshPath)) {
        SKIP("SampleScene mesh fixture is not present in the local Data directory.");
    }

    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);
    REQUIRE_FALSE(loaded.mesh.triangles.empty());

    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.enabled = true;
    settings.cacheCellSizeMeters = 0.08F;
    settings.projectionSearchRadiusMeters = 0.35F;
    settings.previewParticleLimit = 12U;
    settings.trailLengthMeters = 0.80F;
    settings.stepMeters = 0.06F;
    settings.curlStrength = 0.0F;
    settings.branchingStrength = 0.0F;
    settings.eddyStrength = 0.0F;
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());

    struct Candidate {
        std::array<glm::vec3, 3> vertices{};
        glm::vec3 normal{0.0F, 0.0F, 1.0F};
        float edgeXy = 0.0F;
        float verticalRange = 0.0F;
        float score = 0.0F;
    };

    auto toGlm = [](const invisible_places::io::Float3& point) {
        return glm::vec3{point.x, point.y, point.z};
    };
    auto planeZAt = [](const Candidate& candidate, const glm::vec2& xy) {
        const auto& a = candidate.vertices[0];
        return a.z - ((candidate.normal.x * (xy.x - a.x)) + (candidate.normal.y * (xy.y - a.y))) /
                         candidate.normal.z;
    };

    float smallestEdgeXy = std::numeric_limits<float>::max();
    float largestEdgeXy = 0.0F;
    std::optional<Candidate> smallTriangle;
    std::optional<Candidate> largeSlopedTriangle;
    for (const auto& triangle : loaded.mesh.triangles) {
        if (triangle.indices[0] >= loaded.mesh.vertices.size() ||
            triangle.indices[1] >= loaded.mesh.vertices.size() ||
            triangle.indices[2] >= loaded.mesh.vertices.size()) {
            continue;
        }
        Candidate candidate;
        candidate.vertices = {
            toGlm(loaded.mesh.vertices[triangle.indices[0]]),
            toGlm(loaded.mesh.vertices[triangle.indices[1]]),
            toGlm(loaded.mesh.vertices[triangle.indices[2]]),
        };
        glm::vec3 normal = glm::cross(
            candidate.vertices[1] - candidate.vertices[0],
            candidate.vertices[2] - candidate.vertices[0]);
        if (glm::dot(normal, normal) <= 1.0e-12F) {
            continue;
        }
        normal = glm::normalize(normal);
        if (normal.z < 0.0F) {
            normal = -normal;
        }
        if (std::abs(normal.z) <= 0.08F) {
            continue;
        }
        candidate.normal = normal;
        const std::array<float, 3> edges{
            glm::length(glm::vec2{
                candidate.vertices[1].x - candidate.vertices[0].x,
                candidate.vertices[1].y - candidate.vertices[0].y}),
            glm::length(glm::vec2{
                candidate.vertices[2].x - candidate.vertices[1].x,
                candidate.vertices[2].y - candidate.vertices[1].y}),
            glm::length(glm::vec2{
                candidate.vertices[0].x - candidate.vertices[2].x,
                candidate.vertices[0].y - candidate.vertices[2].y}),
        };
        candidate.edgeXy = std::max({edges[0], edges[1], edges[2]});
        smallestEdgeXy = std::min(smallestEdgeXy, candidate.edgeXy);
        largestEdgeXy = std::max(largestEdgeXy, candidate.edgeXy);
        const auto minMaxZ = std::minmax({
            candidate.vertices[0].z,
            candidate.vertices[1].z,
            candidate.vertices[2].z,
        });
        candidate.verticalRange = minMaxZ.second - minMaxZ.first;
        candidate.score = candidate.edgeXy * std::max(candidate.verticalRange, 0.001F);
        if (candidate.edgeXy < 0.035F && !smallTriangle.has_value()) {
            smallTriangle = candidate;
        }
        if (candidate.edgeXy > 0.25F &&
            candidate.verticalRange > 0.015F &&
            (!largeSlopedTriangle.has_value() || candidate.score > largeSlopedTriangle->score)) {
            largeSlopedTriangle = candidate;
        }
    }

    CAPTURE(smallestEdgeXy, largestEdgeXy);
    REQUIRE(smallTriangle.has_value());
    REQUIRE(largeSlopedTriangle.has_value());
    CHECK(smallestEdgeXy < 0.035F);
    CHECK(largestEdgeXy > 0.45F);
    CHECK(largeSlopedTriangle->edgeXy > settings.cacheCellSizeMeters * 3.0F);

    const auto checkProjectionNearPlane = [&](const Candidate& candidate, float maxError) {
        const std::array<glm::vec3, 3> weights{
            glm::vec3{0.55F, 0.25F, 0.20F},
            glm::vec3{0.25F, 0.55F, 0.20F},
            glm::vec3{0.25F, 0.20F, 0.55F},
        };
        float worstError = 0.0F;
        for (const auto& weight : weights) {
            const glm::vec3 point =
                candidate.vertices[0] * weight.x +
                candidate.vertices[1] * weight.y +
                candidate.vertices[2] * weight.z;
            const float expectedZ = planeZAt(candidate, {point.x, point.y});
            const auto projection = invisible_places::water::ProjectToMeshSurface(
                cache,
                {point.x, point.y, expectedZ});
            REQUIRE(projection.hit);
            worstError = std::max(worstError, std::abs(projection.position.z - expectedZ));
        }
        CAPTURE(candidate.edgeXy, candidate.verticalRange, worstError);
        CHECK(worstError < maxError);
    };

    checkProjectionNearPlane(smallTriangle.value(), 0.03F);
    checkProjectionNearPlane(largeSlopedTriangle.value(), 0.08F);

    const auto sourcePoint =
        largeSlopedTriangle->vertices[0] * 0.45F +
        largeSlopedTriangle->vertices[1] * 0.30F +
        largeSlopedTriangle->vertices[2] * 0.25F;
    invisible_places::water::WaterEmitter emitter;
    emitter.id = 41U;
    emitter.name = "Sample mesh source";
    emitter.position = {sourcePoint.x, sourcePoint.y, planeZAt(largeSlopedTriangle.value(), {sourcePoint.x, sourcePoint.y})};
    emitter.radius = 0.01F;
    emitter.strength = 1.0F;
    emitter.speed = 1.0F;
    const auto overlay = invisible_places::water::BuildDynamicMeshWaterTrailOverlay(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings,
        invisible_places::water::WaterTrailBuildQuality::Preview);
    CHECK_FALSE(overlay.samples.empty());
}

TEST_CASE("Dynamic mesh flow bends toward attractor and is deterministic", "[mesh][water]") {
    const auto meshPath = std::filesystem::temp_directory_path() / "invisible_places_dynamic_mesh_flow.ply";
    const std::vector<invisible_places::io::Float3> vertices{
        {0.0F, 0.0F, 0.0F},
        {4.0F, 0.0F, 0.0F},
        {4.0F, 4.0F, 0.0F},
        {0.0F, 4.0F, 0.0F},
    };
    const std::vector<invisible_places::io::Float3> normals(vertices.size(), {0.0F, 0.0F, 1.0F});
    WriteSyntheticTriangleMeshPly(
        meshPath,
        vertices,
        {{0U, 1U, 2U}, {0U, 2U, 3U}},
        normals);
    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);

    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.enabled = true;
    settings.cacheCellSizeMeters = 0.50F;
    settings.projectionSearchRadiusMeters = 0.75F;
    settings.finalParticleLimit = 1U;
    settings.previewParticleLimit = 1U;
    settings.trailLengthMeters = 1.20F;
    settings.stepMeters = 0.08F;
    settings.downhillWeight = 0.0F;
    settings.attractorWeight = 1.0F;
    settings.sourceVelocityWeight = 0.0F;
    settings.curlStrength = 0.0F;
    settings.inertia = 0.0F;
    settings.animationDurationSeconds = 2.0F;
    settings.seed = 17U;
    settings.attractors = {{
        .id = 7U,
        .name = "upper pool",
        .position = {0.25F, 3.0F, 0.0F},
        .radiusMeters = 4.0F,
        .strength = 1.0F,
        .enabled = true,
    }};
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 3U;
    emitter.position = {0.25F, 0.35F, 0.0F};
    emitter.radius = 0.01F;
    emitter.strength = 1.0F;
    emitter.speed = 1.0F;

    invisible_places::water::WaterDynamicMeshFlowDiagnostics diagnostics;
    const auto overlayA = invisible_places::water::BuildDynamicMeshWaterTrailOverlay(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings,
        invisible_places::water::WaterTrailBuildQuality::Final,
        &diagnostics);
    const auto overlayB = invisible_places::water::BuildDynamicMeshWaterTrailOverlay(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings,
        invisible_places::water::WaterTrailBuildQuality::Final);
    REQUIRE_FALSE(overlayA.samples.empty());
    REQUIRE(overlayA.samples.size() == overlayB.samples.size());
    CHECK(diagnostics.emittedPathCount == 1U);

    float minSampleY = std::numeric_limits<float>::max();
    float maxSampleY = -std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < overlayA.samples.size(); ++index) {
        const auto& sampleA = overlayA.samples[index];
        const auto& sampleB = overlayB.samples[index];
        CHECK(sampleA.position.x == Catch::Approx(sampleB.position.x));
        CHECK(sampleA.position.y == Catch::Approx(sampleB.position.y));
        CHECK(sampleA.position.z == Catch::Approx(sampleB.position.z));
        minSampleY = std::min(minSampleY, sampleA.position.y);
        maxSampleY = std::max(maxSampleY, sampleA.position.y);
    }
    CHECK(maxSampleY > minSampleY + 0.60F);

    emitter.position = {1.25F, 0.35F, 0.0F};
    const auto movedOverlay = invisible_places::water::BuildDynamicMeshWaterTrailOverlay(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings,
        invisible_places::water::WaterTrailBuildQuality::Final);
    REQUIRE_FALSE(movedOverlay.samples.empty());
    CHECK(std::abs(movedOverlay.samples.front().position.x - overlayA.samples.front().position.x) > 0.50F);

    auto animatedSettings = settings;
    animatedSettings.sourceVelocityWeight = 1.0F;
    animatedSettings.attractors.front().keyframes = {
        {0.0F, {0.25F, 3.0F, 0.0F}},
        {2.0F, {1.80F, 3.0F, 0.0F}},
    };
    animatedSettings.emitterMotions = {{
        .emitterId = emitter.id,
        .name = "source slide",
        .enabled = true,
        .keyframes = {
            {0.0F, emitter.position},
            {2.0F, {1.25F, emitter.position.y, emitter.position.z}},
        },
    }};
    const auto animatedOverlay = invisible_places::water::BuildDynamicMeshWaterTrailOverlay(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        animatedSettings,
        invisible_places::water::WaterTrailBuildQuality::Final);
    REQUIRE_FALSE(animatedOverlay.samples.empty());
    CHECK(animatedOverlay.samples.back().position.x > overlayA.samples.back().position.x + 0.20F);
}

TEST_CASE("Dynamic mesh particle presets expose laminar branching and turbulent states", "[mesh][water]") {
    const auto defaults = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    const auto laminar = invisible_places::water::ApplyWaterDynamicMeshParticlePreset(defaults, "Laminar");
    const auto branching = invisible_places::water::ApplyWaterDynamicMeshParticlePreset(defaults, "Branching");
    const auto turbulent = invisible_places::water::ApplyWaterDynamicMeshParticlePreset(defaults, "Turbulent");

    CHECK(invisible_places::water::NormalizeWaterDynamicMeshParticlePresetName("laminar") == "Laminar");
    CHECK(laminar.particlePresetName == "Laminar");
    CHECK(branching.particlePresetName == "Branching");
    CHECK(turbulent.particlePresetName == "Turbulent");
    CHECK(laminar.inertia > branching.inertia);
    CHECK(laminar.curlStrength < branching.curlStrength);
    CHECK(branching.branchingStrength > laminar.branchingStrength);
    CHECK(turbulent.eddyStrength > branching.eddyStrength);
    CHECK(turbulent.curlStrength > defaults.curlStrength);
}

TEST_CASE("Dynamic mesh flow settings roundtrip through project JSON", "[mesh][water][serialization]") {
    invisible_places::serialization::ProjectDocument document;
    document.waterDynamicMeshFlowSettings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    document.waterDynamicMeshFlowSettings.enabled = true;
    document.waterDynamicMeshFlowSettings.meshPath = "Data/ExhibitionScene/Site3-MESH.ply";
    document.waterDynamicMeshFlowSettings.previewParticleLimit = 123U;
    document.waterDynamicMeshFlowSettings.gpuPreviewEnabled = true;
    document.waterDynamicMeshFlowSettings.particlePresetName = "Branching";
    document.waterDynamicMeshFlowSettings.branchingStrength = 1.25F;
    document.waterDynamicMeshFlowSettings.eddyStrength = 0.35F;
    document.waterDynamicMeshFlowSettings.topologyResponse = 0.9F;
    document.waterDynamicMeshFlowSettings.trailProfileName = "White Needle Glow_preset";
    document.waterDynamicMeshFlowSettings.attractors = {{
        .id = 42U,
        .name = "moving pull",
        .position = {1.0F, 2.0F, 3.0F},
        .radiusMeters = 2.5F,
        .strength = 0.9F,
        .enabled = true,
        .keyframes = {
            {0.0F, {1.0F, 2.0F, 3.0F}},
            {1.5F, {4.0F, 5.0F, 6.0F}},
        },
    }};
    document.waterDynamicMeshFlowSettings.animationDurationSeconds = 1.5F;
    document.waterDynamicMeshFlowSettings.sourceVelocityWeight = 0.7F;
    document.waterDynamicMeshFlowSettings.emitterMotions = {{
        .emitterId = 9U,
        .name = "moving source",
        .enabled = true,
        .keyframes = {
            {0.0F, {0.0F, 1.0F, 2.0F}},
            {1.5F, {3.0F, 4.0F, 5.0F}},
        },
    }};

    std::string errorMessage;
    const auto projectPath =
        std::filesystem::temp_directory_path() / "invisible_places_dynamic_mesh_flow_project.json";
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, projectPath, &errorMessage));
    {
        std::ifstream savedProject{projectPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedProject},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find("\"mesh_path\"") == std::string::npos);
        CHECK(savedJson.find("\"attractors\"") == std::string::npos);
        CHECK(savedJson.find("\"emitter_motions\"") == std::string::npos);
        CHECK(savedJson.find("\"water_scene_states\"") != std::string::npos);
        CHECK(savedJson.find("\"dynamic_mesh_path\"") != std::string::npos);
        CHECK(savedJson.find("\"dynamic_mesh_attractors\"") == std::string::npos);
        CHECK(savedJson.find("\"dynamic_mesh_emitter_motions\"") == std::string::npos);
    }
    const auto loaded = invisible_places::serialization::LoadProjectDocument(projectPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->waterDynamicMeshFlowSettings.enabled);
    REQUIRE(loaded->waterSceneStates.size() == 1U);
    CHECK(loaded->waterSceneStates.front().dynamicMeshPath == std::filesystem::path{"Data/ExhibitionScene/Site3-MESH.ply"});
    CHECK(loaded->waterDynamicMeshFlowSettings.meshPath == std::filesystem::path{"Data/ExhibitionScene/Site3-MESH.ply"});
    CHECK(loaded->waterDynamicMeshFlowSettings.previewParticleLimit == 123U);
    CHECK(loaded->waterDynamicMeshFlowSettings.gpuPreviewEnabled);
    CHECK(loaded->waterDynamicMeshFlowSettings.particlePresetName == "Branching");
    CHECK(loaded->waterDynamicMeshFlowSettings.branchingStrength == Catch::Approx(1.25F));
    CHECK(loaded->waterDynamicMeshFlowSettings.eddyStrength == Catch::Approx(0.35F));
    CHECK(loaded->waterDynamicMeshFlowSettings.topologyResponse == Catch::Approx(0.9F));
    CHECK(loaded->waterDynamicMeshFlowSettings.trailProfileName == "White Needle Glow_preset");
    CHECK(loaded->waterDynamicMeshFlowSettings.attractors.empty());
    CHECK(loaded->waterDynamicMeshFlowSettings.animationDurationSeconds == Catch::Approx(1.5F));
    CHECK(loaded->waterDynamicMeshFlowSettings.sourceVelocityWeight == Catch::Approx(0.7F));
    CHECK(loaded->waterDynamicMeshFlowSettings.emitterMotions.empty());

    const auto legacyProjectPath =
        std::filesystem::temp_directory_path() / "invisible_places_dynamic_mesh_flow_legacy_project.json";
    {
        std::ofstream legacyProject{legacyProjectPath};
        legacyProject << R"({
  "schema_version": 29,
  "water_dynamic_mesh_flow_settings": {
    "enabled": true,
    "mesh_path": "Data/LegacyScene/Legacy-MESH.ply"
  }
})";
    }
    const auto legacyLoaded =
        invisible_places::serialization::LoadProjectDocument(legacyProjectPath, &errorMessage);
    REQUIRE(legacyLoaded.has_value());
    CHECK(legacyLoaded->waterDynamicMeshFlowSettings.meshPath.generic_string() ==
          "Data/LegacyScene/Legacy-MESH.ply");
}

TEST_CASE("Site3 exhibition mesh can build a dynamic flow preview cache", "[mesh][water][site3][.]") {
    const auto meshPath = DataRoot() / "ExhibitionScene" / "Site3-MESH.ply";
    if (!std::filesystem::exists(meshPath)) {
        SKIP("Site3 exhibition mesh is not present.");
    }
    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.enabled = true;
    settings.meshPath = meshPath;
    settings.cacheCellSizeMeters = 0.25F;
    settings.projectionSearchRadiusMeters = 0.50F;
    settings.previewParticleLimit = 8U;
    settings.trailLengthMeters = 0.20F;
    settings.stepMeters = 0.08F;
    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());
}

TEST_CASE("Site3 default dynamic mesh flow descends from the sample source toward the lower area", "[mesh][water][site3][.]") {
    const auto meshPath = DataRoot() / "ExhibitionScene" / "Site3-MESH.ply";
    if (!std::filesystem::exists(meshPath)) {
        SKIP("Site3 exhibition mesh is not present.");
    }

    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.enabled = true;
    settings.meshPath = meshPath;

    const auto loaded = invisible_places::io::LoadTriangleMesh(meshPath);
    REQUIRE(loaded.success);
    const auto cache = invisible_places::water::BuildMeshSurfaceCache(loaded.mesh, settings);
    REQUIRE_FALSE(cache.cells.empty());

    const invisible_places::io::Float3 sourcePosition{307.622F, 102.514F, 2.066F};
    const auto projectedSource = invisible_places::water::ProjectToMeshSurface(cache, sourcePosition);
    REQUIRE(projectedSource.hit);

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 17U;
    emitter.name = "Source 17";
    emitter.position = sourcePosition;
    emitter.radius = 0.035F;
    emitter.strength = 1.0F;
    emitter.speed = 1.0F;

    invisible_places::water::WaterDynamicMeshFlowDiagnostics diagnostics;
    const auto overlay = invisible_places::water::BuildDynamicMeshWaterTrailOverlay(
        cache,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings,
        invisible_places::water::WaterTrailBuildQuality::Preview,
        &diagnostics);

    REQUIRE_FALSE(overlay.samples.empty());
    CHECK(diagnostics.emittedPathCount > 0U);
    CHECK(diagnostics.emittedSampleCount > diagnostics.emittedPathCount);

    float nearestSourceXy = std::numeric_limits<float>::max();
    float farthestSourceXy = 0.0F;
    float farthestRouteXy = 0.0F;
    float minRouteZ = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();
    std::size_t visibleSampleCount = 0U;
    for (const auto& sample : overlay.samples) {
        const float routeDx = sample.position.x - projectedSource.position.x;
        const float routeDy = sample.position.y - projectedSource.position.y;
        const float routeDistanceXy = std::sqrt(routeDx * routeDx + routeDy * routeDy);
        farthestRouteXy = std::max(farthestRouteXy, routeDistanceXy);
        minRouteZ = std::min(minRouteZ, sample.position.z);
        if (sample.trailRole < 0.5F) {
            continue;
        }
        ++visibleSampleCount;
        const float dx = sample.position.x - projectedSource.position.x;
        const float dy = sample.position.y - projectedSource.position.y;
        const float distanceXy = std::sqrt(dx * dx + dy * dy);
        nearestSourceXy = std::min(nearestSourceXy, distanceXy);
        farthestSourceXy = std::max(farthestSourceXy, distanceXy);
        minZ = std::min(minZ, sample.position.z);
        maxZ = std::max(maxZ, sample.position.z);
    }

    REQUIRE(visibleSampleCount > 0U);
    CAPTURE(diagnostics.projectionMissCount);
    CAPTURE(diagnostics.ambiguousHitCount);
    CAPTURE(nearestSourceXy);
    CAPTURE(farthestSourceXy);
    CAPTURE(farthestRouteXy);
    CAPTURE(minZ);
    CAPTURE(maxZ);
    CAPTURE(minRouteZ);
    CAPTURE(projectedSource.position.z);
    CHECK(nearestSourceXy < 0.35F);
    CHECK(farthestSourceXy > 2.0F);
    CHECK(maxZ > minZ + 0.25F);
    CHECK(minZ < projectedSource.position.z - 0.25F);
}

invisible_places::water::WaterPathBranch MakeSyntheticAnalysisBranch(
    std::uint32_t branchId,
    std::uint32_t emitterId,
    std::vector<invisible_places::io::Float3> positions,
    invisible_places::water::WaterPathBranchRole role = invisible_places::water::WaterPathBranchRole::Main,
    std::optional<std::uint32_t> parentId = std::nullopt) {
    invisible_places::water::WaterPathBranch branch;
    branch.id = branchId;
    branch.emitterId = emitterId;
    branch.role = role;
    branch.parentId = parentId;
    branch.confidence = 1.0F;
    branch.rawAnchors.reserve(positions.size());
    float distance = 0.0F;
    for (std::size_t index = 0U; index < positions.size(); ++index) {
        if (index > 0U) {
            const glm::vec3 previous{
                positions[index - 1U].x,
                positions[index - 1U].y,
                positions[index - 1U].z};
            const glm::vec3 current{positions[index].x, positions[index].y, positions[index].z};
            distance += glm::length(current - previous);
        }
        invisible_places::water::WaterOverlayPoint point;
        point.position = positions[index];
        point.normal = {0.0F, 0.0F, 1.0F};
        point.flowId = static_cast<float>(branchId);
        point.emitterId = static_cast<float>(emitterId);
        point.pathDistance = distance;
        point.width = 0.035F;
        point.speed = 1.0F;
        point.confidence = 1.0F;
        point.pooling = 0.5F;
        branch.rawAnchors.push_back(point);
    }
    branch.length = distance;
    return branch;
}

invisible_places::water::WaterPathCache MakeSyntheticAnalysisCache(
    std::vector<invisible_places::water::WaterPathBranch> branches) {
    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Detail);
    settings.autoTune = false;
    settings.supportVoxelSize = 0.02F;
    settings.maxBridgeDistance = 0.12F;
    settings.pathSampleSpacing = 0.04F;
    settings.pathLength = 1.0F;
    settings.maxSteps = 64U;
    invisible_places::water::WaterPathCache cache;
    cache.supportLayerPath = "synthetic-path-analysis.ply";
    cache.requestedSettings = settings;
    cache.tunedSettings = settings;
    cache.diagnostics.pathSampleSpacing = settings.pathSampleSpacing;
    cache.diagnostics.supportVoxelSize = settings.supportVoxelSize;
    cache.diagnostics.maxBridgeDistance = settings.maxBridgeDistance;
    cache.branches = std::move(branches);
    return cache;
}

struct WaterPathDropStats {
    float totalLength = 0.0F;
    float totalDrop = 0.0F;
    float uphillAmount = 0.0F;
    float lateralDistance = 0.0F;
    float largestUphillStep = 0.0F;
    float largestSidewaysWithoutDrop = 0.0F;
    std::size_t segmentCount = 0U;
    std::size_t downhillSegmentCount = 0U;
};

WaterPathDropStats ComputeWaterPathDropStats(
    const std::vector<invisible_places::water::WaterOverlayPoint>& anchors) {
    WaterPathDropStats stats;
    if (anchors.size() < 2U) {
        return stats;
    }

    for (std::size_t index = 1U; index < anchors.size(); ++index) {
        const glm::vec3 previous{
            anchors[index - 1U].position.x,
            anchors[index - 1U].position.y,
            anchors[index - 1U].position.z};
        const glm::vec3 current{anchors[index].position.x, anchors[index].position.y, anchors[index].position.z};
        const glm::vec3 delta = current - previous;
        const float length = glm::length(delta);
        if (length <= 1.0e-6F) {
            continue;
        }

        const float zDrop = previous.z - current.z;
        const float horizontal = glm::length(glm::vec2{delta.x, delta.y});
        stats.totalLength += length;
        stats.totalDrop += zDrop;
        stats.lateralDistance += horizontal;
        ++stats.segmentCount;
        if (zDrop >= -1.0e-5F) {
            ++stats.downhillSegmentCount;
        } else {
            const float uphill = -zDrop;
            stats.uphillAmount += uphill;
            stats.largestUphillStep = std::max(stats.largestUphillStep, uphill);
        }
        if (zDrop < 0.0015F) {
            stats.largestSidewaysWithoutDrop = std::max(stats.largestSidewaysWithoutDrop, horizontal);
        }
    }
    return stats;
}

float MeanAnalysisValue(
    const invisible_places::water::WaterPathAnalysisCache& analysis,
    float invisible_places::water::WaterPathAnalysisSample::*field) {
    float sum = 0.0F;
    std::size_t count = 0U;
    for (const auto& branch : analysis.branches) {
        for (const auto& sample : branch.samples) {
            sum += sample.*field;
            ++count;
        }
    }
    return count == 0U ? 0.0F : sum / static_cast<float>(count);
}

float MaxAnalysisValue(
    const invisible_places::water::WaterPathAnalysisCache& analysis,
    float invisible_places::water::WaterPathAnalysisSample::*field) {
    float maximum = 0.0F;
    for (const auto& branch : analysis.branches) {
        for (const auto& sample : branch.samples) {
            maximum = std::max(maximum, sample.*field);
        }
    }
    return maximum;
}

bool WaterPathAnalysisSamplesFinite(const invisible_places::water::WaterPathAnalysisCache& analysis) {
    for (const auto& branch : analysis.branches) {
        for (const auto& sample : branch.samples) {
            const std::array<float, 11> values{
                sample.slope,
                sample.flatness,
                sample.curvature,
                sample.neighborDensity,
                sample.nearestPathDistance,
                sample.confluence,
                sample.channelWidth,
                sample.speed,
                sample.turbulence,
                sample.eddyPotential,
                sample.ripplePotential,
            };
            if (!std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); })) {
                return false;
            }
            if (sample.slope < 0.0F || sample.slope > 1.0F ||
                sample.flatness < 0.0F || sample.flatness > 1.0F ||
                sample.curvature < 0.0F || sample.curvature > 1.0F ||
                sample.neighborDensity < 0.0F || sample.neighborDensity > 1.0F ||
                sample.confluence < 0.0F || sample.confluence > 1.0F ||
                sample.turbulence < 0.0F || sample.turbulence > 1.0F ||
                sample.eddyPotential < 0.0F || sample.eddyPotential > 1.0F ||
                sample.ripplePotential < 0.0F || sample.ripplePotential > 1.0F) {
                return false;
            }
            if (sample.nearestPathDistance < 0.0F || sample.channelWidth <= 0.0F || sample.speed < 0.0F) {
                return false;
            }
        }
    }
    return true;
}

TEST_CASE("Water path analysis separates steep and flat flow values", "[water][path-analysis]") {
    const auto steepCache = MakeSyntheticAnalysisCache({
        MakeSyntheticAnalysisBranch(
            1U,
            7U,
            {{0.0F, 0.0F, 1.0F}, {0.0F, 0.18F, 0.78F}, {0.0F, 0.36F, 0.56F}, {0.0F, 0.54F, 0.34F}}),
    });
    const auto flatCache = MakeSyntheticAnalysisCache({
        MakeSyntheticAnalysisBranch(
            1U,
            7U,
            {{0.0F, 0.0F, 1.0F}, {0.0F, 0.18F, 0.99F}, {0.0F, 0.36F, 0.98F}, {0.0F, 0.54F, 0.97F}}),
    });

    const auto steepAnalysis = invisible_places::water::BuildWaterPathAnalysis(steepCache);
    const auto flatAnalysis = invisible_places::water::BuildWaterPathAnalysis(flatCache);

    CHECK(WaterPathAnalysisSamplesFinite(steepAnalysis));
    CHECK(WaterPathAnalysisSamplesFinite(flatAnalysis));
    CHECK(MeanAnalysisValue(steepAnalysis, &invisible_places::water::WaterPathAnalysisSample::slope) >
          MeanAnalysisValue(flatAnalysis, &invisible_places::water::WaterPathAnalysisSample::slope) + 0.40F);
    CHECK(MeanAnalysisValue(flatAnalysis, &invisible_places::water::WaterPathAnalysisSample::flatness) >
          MeanAnalysisValue(steepAnalysis, &invisible_places::water::WaterPathAnalysisSample::flatness) + 0.35F);
    CHECK(MeanAnalysisValue(steepAnalysis, &invisible_places::water::WaterPathAnalysisSample::speed) >
          MeanAnalysisValue(flatAnalysis, &invisible_places::water::WaterPathAnalysisSample::speed) * 1.25F);
    CHECK(MeanAnalysisValue(flatAnalysis, &invisible_places::water::WaterPathAnalysisSample::channelWidth) >
          MeanAnalysisValue(steepAnalysis, &invisible_places::water::WaterPathAnalysisSample::channelWidth) * 1.15F);
}

TEST_CASE("Water path analysis widens nearby confluence branches", "[water][path-analysis]") {
    const auto closeCache = MakeSyntheticAnalysisCache({
        MakeSyntheticAnalysisBranch(
            1U,
            9U,
            {{0.0F, 0.0F, 1.0F}, {0.0F, 0.16F, 0.94F}, {0.0F, 0.32F, 0.88F}, {0.0F, 0.48F, 0.82F}}),
        MakeSyntheticAnalysisBranch(
            2U,
            9U,
            {{0.04F, 0.0F, 1.0F}, {0.04F, 0.16F, 0.94F}, {0.04F, 0.32F, 0.88F}, {0.04F, 0.48F, 0.82F}},
            invisible_places::water::WaterPathBranchRole::Secondary,
            1U),
    });
    const auto isolatedCache = MakeSyntheticAnalysisCache({
        MakeSyntheticAnalysisBranch(
            1U,
            9U,
            {{0.0F, 0.0F, 1.0F}, {0.0F, 0.16F, 0.94F}, {0.0F, 0.32F, 0.88F}, {0.0F, 0.48F, 0.82F}}),
    });

    const auto closeAnalysis = invisible_places::water::BuildWaterPathAnalysis(closeCache);
    const auto isolatedAnalysis = invisible_places::water::BuildWaterPathAnalysis(isolatedCache);

    CHECK(WaterPathAnalysisSamplesFinite(closeAnalysis));
    CHECK(MeanAnalysisValue(closeAnalysis, &invisible_places::water::WaterPathAnalysisSample::neighborDensity) > 0.15F);
    CHECK(MeanAnalysisValue(closeAnalysis, &invisible_places::water::WaterPathAnalysisSample::confluence) > 0.15F);
    CHECK(MeanAnalysisValue(closeAnalysis, &invisible_places::water::WaterPathAnalysisSample::channelWidth) >
          MeanAnalysisValue(isolatedAnalysis, &invisible_places::water::WaterPathAnalysisSample::channelWidth) * 1.25F);
    CHECK(MeanAnalysisValue(closeAnalysis, &invisible_places::water::WaterPathAnalysisSample::nearestPathDistance) <
          MeanAnalysisValue(isolatedAnalysis, &invisible_places::water::WaterPathAnalysisSample::nearestPathDistance));
}

TEST_CASE("Water path analysis raises eddies and ripples on curved paths", "[water][path-analysis]") {
    const auto straightCache = MakeSyntheticAnalysisCache({
        MakeSyntheticAnalysisBranch(
            1U,
            5U,
            {{0.0F, 0.0F, 1.0F}, {0.12F, 0.0F, 0.95F}, {0.24F, 0.0F, 0.90F}, {0.36F, 0.0F, 0.85F}}),
    });
    const auto curvedCache = MakeSyntheticAnalysisCache({
        MakeSyntheticAnalysisBranch(
            1U,
            5U,
            {{0.0F, 0.0F, 1.0F}, {0.12F, 0.0F, 0.95F}, {0.16F, 0.10F, 0.90F}, {0.08F, 0.18F, 0.85F}, {0.0F, 0.20F, 0.80F}}),
    });

    const auto straightAnalysis = invisible_places::water::BuildWaterPathAnalysis(straightCache);
    const auto curvedAnalysis = invisible_places::water::BuildWaterPathAnalysis(curvedCache);

    CHECK(WaterPathAnalysisSamplesFinite(curvedAnalysis));
    CHECK(MaxAnalysisValue(curvedAnalysis, &invisible_places::water::WaterPathAnalysisSample::curvature) >
          MaxAnalysisValue(straightAnalysis, &invisible_places::water::WaterPathAnalysisSample::curvature) + 0.20F);
    CHECK(MaxAnalysisValue(curvedAnalysis, &invisible_places::water::WaterPathAnalysisSample::eddyPotential) >
          MaxAnalysisValue(straightAnalysis, &invisible_places::water::WaterPathAnalysisSample::eddyPotential) + 0.08F);
    CHECK(MaxAnalysisValue(curvedAnalysis, &invisible_places::water::WaterPathAnalysisSample::ripplePotential) >
          MaxAnalysisValue(straightAnalysis, &invisible_places::water::WaterPathAnalysisSample::ripplePotential) + 0.08F);
}

TEST_CASE("Site3 path analysis fixture parses and bakes when sample data exists", "[water][path-analysis][sample][.]") {
    const auto fixturePath = DataRoot().parent_path() / "tests" / "water_flow_path_analysis_site3_fixture.json";
    std::ifstream input{fixturePath};
    REQUIRE(input.good());
    const auto fixture = nlohmann::json::parse(input);
    CHECK(fixture.value("name", std::string{}) == "site3_flow_path_analysis_reference");
    REQUIRE(fixture.contains("emitter"));
    REQUIRE(fixture.contains("path_settings"));

    const auto sourcePath = DataRoot().parent_path() / fixture.at("source_path").get<std::string>();
    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Detail);
    const auto& pathSettings = fixture.at("path_settings");
    settings.autoTune = pathSettings.value("autoTune", settings.autoTune);
    settings.branching = pathSettings.value("branching", settings.branching);
    settings.coverage = pathSettings.value("coverage", settings.coverage);
    settings.gapTolerance = pathSettings.value("gapTolerance", settings.gapTolerance);
    settings.pathLength = pathSettings.value("pathLength", settings.pathLength);
    settings.smoothing = pathSettings.value("smoothing", settings.smoothing);
    settings.supportVoxelSize = pathSettings.value("supportVoxelSize", settings.supportVoxelSize);
    settings.maxBridgeDistance = pathSettings.value("maxBridgeDistance", settings.maxBridgeDistance);
    settings.pathSampleSpacing = pathSettings.value("pathSampleSpacing", settings.pathSampleSpacing);
    settings.maxSteps = pathSettings.value("maxSteps", settings.maxSteps);
    settings.supportSampleLimit = pathSettings.value("supportSampleLimit", settings.supportSampleLimit);

    const auto& emitterJson = fixture.at("emitter");
    invisible_places::water::WaterEmitter emitter;
    emitter.id = emitterJson.value("id", 0U);
    emitter.name = emitterJson.value("name", std::string{"Site3 path analysis source"});
    const auto position = emitterJson.at("position").get<std::array<float, 3>>();
    emitter.position = {position[0], position[1], position[2]};
    emitter.radius = emitterJson.value("radius", emitter.radius);
    emitter.confidence = emitterJson.value("confidence", emitter.confidence);

    if (!std::filesystem::exists(sourcePath)) {
        return;
    }

    const auto loadResult = invisible_places::io::LoadPointCloud(sourcePath);
    REQUIRE(loadResult.success);
    auto cache = invisible_places::water::GenerateWaterPathCache(
        loadResult.cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings);
    REQUIRE_FALSE(cache.branches.empty());
    invisible_places::water::EnsureWaterPathAnalysis(&cache);
    REQUIRE(invisible_places::water::WaterPathAnalysisCacheCompatible(cache));
    REQUIRE(cache.analysis.has_value());
    CHECK(WaterPathAnalysisSamplesFinite(cache.analysis.value()));
    CHECK(MeanAnalysisValue(cache.analysis.value(), &invisible_places::water::WaterPathAnalysisSample::channelWidth) > 0.0F);
    CHECK(MeanAnalysisValue(cache.analysis.value(), &invisible_places::water::WaterPathAnalysisSample::speed) > 0.0F);
}

TEST_CASE("Water path attractor biases a downhill fork without climbing Z", "[water][path]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-attractor-fork.ply";
    cloud.layerName = "synthetic-water-attractor-fork";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;
    cloud.positions.push_back({0.0F, 0.0F, 1.0F});
    cloud.normals.push_back({0.0F, 0.0F, 1.0F});
    cloud.packedColors.push_back(0xFFFFFFFFU);
    cloud.bounds.Expand(cloud.positions.back());
    for (int step = 1; step <= 9; ++step) {
        const float y = static_cast<float>(step) * 0.020F;
        const float z = 1.0F - static_cast<float>(step) * 0.055F;
        for (const auto position : std::array<invisible_places::io::Float3, 2>{
                 invisible_places::io::Float3{-0.040F, y, z},
                 invisible_places::io::Float3{0.040F, y, z}}) {
            cloud.positions.push_back(position);
            cloud.normals.push_back({0.0F, 0.0F, 1.0F});
            cloud.packedColors.push_back(0xFFFFFFFFU);
            cloud.bounds.Expand(position);
        }
    }
    cloud.focusPoint = cloud.positions.front();
    cloud.hasFocusPoint = true;

    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Detail);
    settings.autoTune = false;
    settings.supportVoxelSize = 0.010F;
    settings.maxBridgeDistance = 0.105F;
    settings.pathLength = 0.65F;
    settings.pathSampleSpacing = 0.012F;
    settings.branching = 0.0F;
    settings.coverage = 0.0F;
    settings.gapTolerance = 0.35F;
    settings.attractorEnabled = true;
    settings.attractorPosition = {0.040F, 0.18F, 0.505F};
    settings.attractorStrength = 1.0F;
    settings.maxSteps = 80;
    settings.supportSampleLimit = 128;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 51;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.05F;
    emitter.confidence = 1.0F;

    auto leftAttractorSettings = settings;
    leftAttractorSettings.attractorPosition = {-0.040F, 0.18F, 0.505F};
    const auto leftAttractorCache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        leftAttractorSettings);
    const auto leftMainBranch = std::find_if(
        leftAttractorCache.branches.begin(),
        leftAttractorCache.branches.end(),
        [](const invisible_places::water::WaterPathBranch& branch) {
            return branch.role == invisible_places::water::WaterPathBranchRole::Main;
        });
    REQUIRE(leftMainBranch != leftAttractorCache.branches.end());

    const auto cache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings);
    CHECK(cache.tunedSettings.attractorEnabled);
    CHECK(cache.tunedSettings.attractorStrength == Catch::Approx(1.0F));
    const auto mainBranch = std::find_if(
        cache.branches.begin(),
        cache.branches.end(),
        [](const invisible_places::water::WaterPathBranch& branch) {
            return branch.role == invisible_places::water::WaterPathBranchRole::Main;
    });
    REQUIRE(mainBranch != cache.branches.end());
    REQUIRE(mainBranch->rawAnchors.size() >= 4U);
    const auto averagePathX = [](const invisible_places::water::WaterPathBranch& branch) {
        float sum = 0.0F;
        for (const auto& anchor : branch.rawAnchors) {
            sum += anchor.position.x;
        }
        return branch.rawAnchors.empty() ? 0.0F : sum / static_cast<float>(branch.rawAnchors.size());
    };
    const auto maxPathX = [](const invisible_places::water::WaterPathBranch& branch) {
        float maxX = -std::numeric_limits<float>::max();
        for (const auto& anchor : branch.rawAnchors) {
            maxX = std::max(maxX, anchor.position.x);
        }
        return maxX;
    };
    CHECK(averagePathX(*mainBranch) > averagePathX(*leftMainBranch) + 0.040F);
    CHECK(maxPathX(*mainBranch) > 0.015F);
    CHECK(
        mainBranch->rawAnchors.front().position.z -
        mainBranch->rawAnchors.back().position.z > 0.30F);
    for (std::size_t index = 1; index < mainBranch->rawAnchors.size(); ++index) {
        CHECK(
            mainBranch->rawAnchors[index].position.z <=
            mainBranch->rawAnchors[index - 1U].position.z + 1.0e-4F);
    }
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
    REQUIRE(smoothAnchors.points.size() > rawAnchors.points.size());

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
    CHECK(smoothAnchors.points.front().position.x == Catch::Approx(rawAnchors.points.front().position.x));
    CHECK(smoothAnchors.points.front().position.y == Catch::Approx(rawAnchors.points.front().position.y));
    CHECK(smoothAnchors.points.front().position.z == Catch::Approx(rawAnchors.points.front().position.z));
    CHECK(smoothAnchors.points.back().position.x == Catch::Approx(rawAnchors.points.back().position.x));
    CHECK(smoothAnchors.points.back().position.y == Catch::Approx(rawAnchors.points.back().position.y));
    CHECK(smoothAnchors.points.back().position.z == Catch::Approx(rawAnchors.points.back().position.z));
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
    REQUIRE(smoothAnchors.points.size() > rawAnchors.points.size());

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

    bakeChanging = source;
    bakeChanging.path.attractorEnabled = true;
    bakeChanging.path.attractorPosition = {0.05F, 0.02F, 0.40F};
    bakeChanging.path.attractorStrength = 0.55F;
    CHECK_FALSE(invisible_places::water::WaterSourceBakeInputsEqual(source, bakeChanging));
}

TEST_CASE("Path View diagnostic colour mode changes do not rebuild water paths lanes or trails", "[water][path-view][ui]") {
    const auto modes = invisible_places::app::AllWaterPathDiagnosticColorModes();
    REQUIRE(modes.size() == 11U);

    std::vector<std::string> labels;
    labels.reserve(modes.size());
    for (const auto mode : modes) {
        labels.emplace_back(invisible_places::app::WaterPathDiagnosticColorModeLabel(mode));
    }
    CHECK(labels == std::vector<std::string>{
                        "Branch",
                        "Slope",
                        "Flatness",
                        "Curvature",
                        "Neighbor Density",
                        "Confluence",
                        "Channel Width",
                        "Speed",
                        "Turbulence",
                        "Eddies",
                        "Ripples",
                    });

    invisible_places::app::WaterPathDiagnosticModeChangeStats stats;
    const invisible_places::app::WaterPathDiagnosticRebuildCounters before{
        .pathBakes = 4U,
        .laneBuilds = 8U,
        .trailBuilds = 12U,
    };
    const auto after = before;
    stats = invisible_places::app::RecordWaterPathDiagnosticModeChange(
        stats,
        before,
        after,
        std::chrono::microseconds{275});
    CHECK(stats.changeCount == 1U);
    CHECK(stats.lastLatencyMs == Catch::Approx(0.275));
    CHECK_FALSE(stats.lastChangeTouchedRebuildCounters);
    CHECK(invisible_places::app::WaterPathDiagnosticRebuildCountersEqual(stats.before, stats.after));

    auto rebuilt = after;
    ++rebuilt.trailBuilds;
    stats = invisible_places::app::RecordWaterPathDiagnosticModeChange(
        stats,
        after,
        rebuilt,
        std::chrono::microseconds{100});
    CHECK(stats.changeCount == 2U);
    CHECK(stats.lastChangeTouchedRebuildCounters);
}

TEST_CASE("Water path ranking prefers Z-down support over sideways shelves", "[water]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.sourcePath = "synthetic-water-sideways-shelf.ply";
    cloud.layerName = "synthetic-water-sideways-shelf";
    cloud.hasSourceRgb = true;
    cloud.hasNormals = true;

    const auto appendPoint = [&](invisible_places::io::Float3 position, invisible_places::io::Float3 normal) {
        cloud.positions.push_back(position);
        cloud.normals.push_back(normal);
        cloud.packedColors.push_back(0xFFFFFFFFU);
        cloud.bounds.Expand(position);
    };

    appendPoint({0.0F, 0.0F, 1.0F}, {0.65F, 0.0F, 0.76F});
    for (int index = 1; index <= 14; ++index) {
        appendPoint(
            {0.017F * static_cast<float>(index), 0.0F, 1.0F - 0.0012F * static_cast<float>(index)},
            {0.65F, 0.0F, 0.76F});
    }
    for (int index = 1; index <= 14; ++index) {
        appendPoint(
            {0.004F * static_cast<float>(index % 2),
             -0.024F * static_cast<float>(index),
             1.0F - 0.022F * static_cast<float>(index)},
            {0.0F, -0.62F, 0.78F});
    }

    auto settings = invisible_places::water::DefaultWaterPathGenerationSettings(
        invisible_places::water::WaterScaleMode::Detail);
    settings.autoTune = false;
    settings.supportVoxelSize = 0.006F;
    settings.maxBridgeDistance = 0.052F;
    settings.pathLength = 0.42F;
    settings.pathSampleSpacing = 0.006F;
    settings.branching = 0.20F;
    settings.coverage = 0.70F;
    settings.gapTolerance = 0.50F;
    settings.maxSteps = 96U;
    settings.supportSampleLimit = 256U;

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 71U;
    emitter.position = cloud.positions.front();
    emitter.radius = 0.024F;
    emitter.confidence = 1.0F;

    const auto cache = invisible_places::water::GenerateWaterPathCache(
        cloud,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        settings);
    const auto mainBranch = std::find_if(
        cache.branches.begin(),
        cache.branches.end(),
        [](const invisible_places::water::WaterPathBranch& branch) {
            return branch.role == invisible_places::water::WaterPathBranchRole::Main;
        });
    REQUIRE(mainBranch != cache.branches.end());
    REQUIRE(mainBranch->rawAnchors.size() > 8U);

    const auto stats = ComputeWaterPathDropStats(mainBranch->rawAnchors);
    const auto& start = mainBranch->rawAnchors.front().position;
    const auto& end = mainBranch->rawAnchors.back().position;
    CAPTURE(stats.totalDrop, stats.uphillAmount, stats.lateralDistance, start.x, start.y, start.z, end.x, end.y, end.z);
    CHECK(stats.totalDrop > 0.18F);
    CHECK(stats.downhillSegmentCount >= stats.segmentCount * 8U / 10U);
    CHECK(stats.largestUphillStep < 0.010F);
    CHECK(end.z < start.z - 0.18F);
    CHECK(end.y < start.y - 0.16F);
    CHECK(std::abs(end.x - start.x) < 0.070F);
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

TEST_CASE("SampleScene combined water support source descends in Z", "[water][sample][.]") {
    const auto sampleRoot = DataRoot() / "SampleScene";
    const auto fixturePath = DataRoot().parent_path() / "tests" / "fixtures" /
                             "sample_scene_water_sources.json";
    if (!std::filesystem::exists(sampleRoot) || !std::filesystem::exists(fixturePath)) {
        SKIP("SampleScene multi-cloud fixture is not present in the local Data directory.");
    }

    const auto assetCatalog = invisible_places::io::DiscoverAssets(DataRoot());
    const auto sceneCatalog = invisible_places::scene::SceneCatalog::FromDiscoveredAssets(assetCatalog);
    const auto* sampleScene = sceneCatalog.FindPointCloudGroup("SampleScene");
    REQUIRE(sampleScene != nullptr);
    const auto* displayBundle = sampleScene->FindCompleteDisplayBundle(3'000U);
    REQUIRE(displayBundle != nullptr);
    const auto& rockVariant = displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Rock);
    const auto& sandVariant = displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Sand);
    const auto& vegVariant = displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Vegetation);
    CHECK(rockVariant.sourcePath.filename() ==
          "Site1-ROCK-3mm. SampleScene.ply");
    CHECK(sandVariant.sourcePath.filename() ==
          "Site1-SAND-3mm. SampleScene.ply");
    CHECK(vegVariant.sourcePath.filename() ==
          "Site1-VEG-3mm. SampleScene.ply");

    const auto rockResult = invisible_places::io::LoadPointCloud(rockVariant.sourcePath);
    const auto sandResult = invisible_places::io::LoadPointCloud(sandVariant.sourcePath);
    const auto vegResult = invisible_places::io::LoadPointCloud(vegVariant.sourcePath);
    REQUIRE(rockResult.success);
    REQUIRE(sandResult.success);
    REQUIRE(vegResult.success);

    std::string fixtureError;
    const auto fixture = invisible_places::serialization::LoadWaterSourcesDocument(
        fixturePath,
        &fixtureError);
    INFO(fixtureError);
    REQUIRE(fixture.has_value());
    const auto emitterIt = std::find_if(
        fixture->emitters.begin(),
        fixture->emitters.end(),
        [](const auto& emitter) {
            return emitter.name == "SampleFlowPoint";
        });
    REQUIRE(emitterIt != fixture->emitters.end());

    auto sourceSettings = fixture->sourceSettings;
    sourceSettings.path.autoTune = true;
    sourceSettings.path.supportVoxelSize = 0.006F;
    sourceSettings.path.maxBridgeDistance = 0.065F;
    sourceSettings.path.pathSampleSpacing = 0.006F;
    sourceSettings.path.pathLength = 2.20F;
    sourceSettings.path.branching = 0.55F;
    sourceSettings.path.coverage = 0.85F;
    sourceSettings.path.gapTolerance = 0.70F;
    sourceSettings.path.maxSteps = 1000U;
    sourceSettings.path.supportSampleLimit = 650000U;

    const std::array<invisible_places::water::WaterSceneSupportLayer, 3> layers{
        invisible_places::water::WaterSceneSupportLayer{
            .cloud = &rockResult.cloud,
            .role = "ROCK",
            .pointSpacingMeters = 0.003F,
            .samplingMultiplier = 1.0F},
        invisible_places::water::WaterSceneSupportLayer{
            .cloud = &sandResult.cloud,
            .role = "SAND",
            .pointSpacingMeters = 0.003F,
            .samplingMultiplier = 2.0F},
        invisible_places::water::WaterSceneSupportLayer{
            .cloud = &vegResult.cloud,
            .role = "VEG",
            .pointSpacingMeters = 0.003F,
            .samplingMultiplier = 2.0F},
    };

    const auto emitter = *emitterIt;
    const std::array<invisible_places::io::Float3, 1> sourcePoints{{emitter.position}};

    const auto combined = invisible_places::water::BuildCombinedWaterSupportCloud(
        layers,
        sourceSettings.path,
        sourcePoints);
    REQUIRE(combined.PointCount() > 1000U);
    CHECK(combined.hasNormals);
    const glm::vec3 source{emitter.position.x, emitter.position.y, emitter.position.z};
    const auto sourceNeighbourCount = static_cast<std::size_t>(std::count_if(
        combined.positions.begin(),
        combined.positions.end(),
        [&](const invisible_places::io::Float3& point) {
            const glm::vec3 delta{point.x - source.x, point.y - source.y, point.z - source.z};
            return glm::dot(delta, delta) <= 0.35F * 0.35F;
        }));
    CHECK(sourceNeighbourCount > 96U);

    const auto cache = invisible_places::water::GenerateWaterPathCache(
        combined,
        std::vector<invisible_places::water::WaterEmitter>{emitter},
        sourceSettings);
    const auto mainBranch = std::find_if(
        cache.branches.begin(),
        cache.branches.end(),
        [](const invisible_places::water::WaterPathBranch& branch) {
            return branch.role == invisible_places::water::WaterPathBranchRole::Main;
        });
    REQUIRE(mainBranch != cache.branches.end());
    REQUIRE(mainBranch->rawAnchors.size() > 8U);

    const auto stats = ComputeWaterPathDropStats(mainBranch->rawAnchors);
    const auto& start = mainBranch->rawAnchors.front().position;
    const auto& end = mainBranch->rawAnchors.back().position;
    CAPTURE(
        combined.PointCount(),
        sourceNeighbourCount,
        cache.diagnostics.summary,
        stats.totalLength,
        stats.totalDrop,
        stats.uphillAmount,
        stats.lateralDistance,
        start.x,
        start.y,
        start.z,
        end.x,
        end.y,
        end.z);
    CHECK(cache.diagnostics.estimatedPointSpacing > 0.0F);
    CHECK(cache.tunedSettings.pathSampleSpacing <= 0.008F);
    CHECK(stats.totalDrop > 0.045F);
    CHECK(stats.downhillSegmentCount >= stats.segmentCount * 55U / 100U);
    CHECK(stats.uphillAmount < stats.totalDrop * 0.65F + 0.035F);
    CHECK(stats.largestUphillStep < 0.035F);
    CHECK(end.z < start.z - 0.040F);
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
    REQUIRE(cache.analysis.has_value());
    CHECK(invisible_places::water::WaterPathAnalysisCacheCompatible(cache));
    auto stalePathInputs = cache;
    stalePathInputs.stale = true;
    CHECK(invisible_places::water::WaterPathAnalysisCacheCompatible(stalePathInputs));
    CHECK(WaterPathAnalysisSamplesFinite(cache.analysis.value()));
    CHECK(std::any_of(cache.branches.begin(), cache.branches.end(), [](const auto& branch) {
        return branch.gapCount > 0U || branch.confidence < 0.95F;
    }));
    cache.supportLayerPath = cloud.sourcePath;
    cache.supportSignature = "points=7";
    cache.emitterSettingsFingerprint = "emitter=5";
    cache.hiddenBranchIds.push_back(cache.branches.front().id);

    const auto outputPath =
        std::filesystem::temp_directory_path() / "invisible_places_water_path_cache_test.flowpathcache";
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
    REQUIRE(loaded->analysis.has_value());
    CHECK(invisible_places::water::WaterPathAnalysisCacheCompatible(loaded.value()));
    auto legacyCache = loaded.value();
    legacyCache.analysis.reset();
    CHECK_FALSE(invisible_places::water::WaterPathAnalysisCacheCompatible(legacyCache));
    invisible_places::water::EnsureWaterPathAnalysis(&legacyCache);
    REQUIRE(legacyCache.analysis.has_value());
    CHECK(invisible_places::water::WaterPathAnalysisCacheCompatible(legacyCache));

    const auto truncatedPath =
        std::filesystem::temp_directory_path() / "invisible_places_water_path_cache_truncated.flowpathcache";
    std::filesystem::copy_file(
        outputPath,
        truncatedPath,
        std::filesystem::copy_options::overwrite_existing);
    const auto completeSize = std::filesystem::file_size(truncatedPath);
    REQUIRE(completeSize > 1U);
    std::filesystem::resize_file(truncatedPath, completeSize - 1U);
    CHECK_FALSE(invisible_places::serialization::LoadWaterPathCacheDocument(
                    truncatedPath,
                    &errorMessage)
                    .has_value());

    const auto corruptPath =
        std::filesystem::temp_directory_path() / "invisible_places_water_path_cache_corrupt.flowpathcache";
    std::filesystem::copy_file(
        outputPath,
        corruptPath,
        std::filesystem::copy_options::overwrite_existing);
    {
        std::fstream corrupt{corruptPath, std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(corrupt.is_open());
        corrupt.seekg(-1, std::ios::end);
        const auto finalByte = corrupt.get();
        REQUIRE(finalByte != std::char_traits<char>::eof());
        corrupt.seekp(-1, std::ios::end);
        corrupt.put(static_cast<char>(static_cast<unsigned char>(finalByte) ^ 0xFFU));
    }
    CHECK_FALSE(invisible_places::serialization::LoadWaterPathCacheDocument(
                    corruptPath,
                    &errorMessage)
                    .has_value());

    std::filesystem::remove(corruptPath);
    std::filesystem::remove(truncatedPath);
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
    document.trailGeometry.trailLengthMeters = 1.25F;
    document.trailGeometry.pointSpacingMeters = 0.044F;
    document.trailGeometry.widthMeters = 0.018F;
    document.trailGeometry.streakLengthMeters = 0.12F;
    document.trailGeometry.startFadeEnabled = true;
    document.trailGeometry.startFadeFullDistanceMeters = 0.28F;
    document.trailGeometry.startFadeRandomBeginDistanceMeters = 0.09F;
    document.trailGeometry.endFadeEnabled = true;
    document.trailGeometry.endFadeFullDistanceMeters = 0.39F;
    document.trailGeometry.endFadeRandomBeginDistanceMeters = 0.14F;
    invisible_places::serialization::WaterPathProfileDocument sourcePathProfile;
    sourcePathProfile.name = "Source Path";
    sourcePathProfile.settings = document.sourceSettings.path;
    sourcePathProfile.settings.pathLength = 12.5F;
    document.pathProfiles.push_back(sourcePathProfile);
    invisible_places::serialization::WaterLaneProfileDocument sourceLaneProfile;
    sourceLaneProfile.name = "Sheet Lanes";
    sourceLaneProfile.settings = document.flowTrailSettings;
    sourceLaneProfile.settings.trailCountTotal = 777U;
    sourceLaneProfile.settings.laneCount = 13U;
    sourceLaneProfile.settings.turbulence = 0.42F;
    document.laneProfiles.push_back(sourceLaneProfile);
    invisible_places::serialization::WaterTrailProfileDocument sourceTrailProfile;
    sourceTrailProfile.name = "Mist Trail";
    sourceTrailProfile.geometry = document.trailGeometry;
    sourceTrailProfile.geometry.trailLengthMeters = 2.2F;
    invisible_places::style::SetScalarConstant(&sourceTrailProfile.style.opacity, 0.27F);
    document.trailProfiles.push_back(sourceTrailProfile);
    document.selectedPathProfileName = "Source Path_edited";
    document.tempPathProfileSettings = sourcePathProfile.settings;
    document.tempPathProfileSettings->coverage = 0.88F;
    document.selectedLaneProfileName = "Sheet Lanes_edited";
    document.tempLaneProfileSettings = sourceLaneProfile.settings;
    document.tempLaneProfileSettings->laneCrossing = 0.63F;
    document.selectedTrailProfileName = "Mist Trail_edited";
    document.tempTrailProfile = sourceTrailProfile;
    document.tempTrailProfile->geometry.widthMeters = 0.033F;

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
    emitter.pathProfileName = "Source Path";
    emitter.laneProfileName = "Sheet Lanes";
    emitter.trailProfileName = "Mist Trail";
    emitter.pathProfileLocked = true;
    emitter.trailProfileLocked = true;
    document.emitters.push_back(emitter);

    invisible_places::water::WaterManualFlowPathSource manualFlowPath;
    manualFlowPath.id = 45U;
    manualFlowPath.name = "Authored channel";
    manualFlowPath.controlPoints = {{1.0F, 2.0F, 3.0F}, {2.0F, 2.5F, 2.0F}};
    manualFlowPath.laneProfileName = "Sheet Lanes";
    manualFlowPath.trailProfileName = "Mist Trail";
    manualFlowPath.laneProfileLocked = true;
    document.manualFlowPaths.push_back(manualFlowPath);

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
    document.flowTrailSettings.trailCountTotal = 222U;
    document.flowTrailSettings.trailStreakLengthMeters = 0.052F;
    document.flowTrailSettings.laneCrossing = 0.31F;
    document.flowTrailSettings.startFadeEnabled = true;
    document.flowTrailSettings.startFadeFullDistanceMeters = 0.22F;
    document.flowTrailSettings.startFadeRandomBeginDistanceMeters = 0.08F;
    document.flowTrailSettings.endFadeEnabled = true;
    document.flowTrailSettings.endFadeFullDistanceMeters = 0.41F;
    document.flowTrailSettings.endFadeRandomBeginDistanceMeters = 0.16F;
    document.dynamicMeshFlowSettings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    document.dynamicMeshFlowSettings.enabled = true;
    document.dynamicMeshFlowSettings.meshPath = "Data/ExhibitionScene/Site3-MESH.ply";
    document.dynamicMeshFlowSettings.previewParticleLimit = 444U;
    document.dynamicMeshFlowSettings.gpuPreviewEnabled = true;
    document.dynamicMeshFlowSettings.showTrails = false;
    document.dynamicMeshFlowSettings.automaticSources = false;
    document.dynamicMeshFlowSettings.particleCapacity = 6144U;
    document.dynamicMeshFlowSettings.historyLength = 36U;
    document.dynamicMeshFlowSettings.sourceBandWidthMeters = 0.48F;
    document.dynamicMeshFlowSettings.sharedWindStrength = 0.29F;
    document.dynamicMeshFlowSettings.rockResponse.persistenceSeconds = 5.0F;
    document.dynamicMeshFlowSettings.vegetationResponse.twinkle = 2.1F;
    document.dynamicMeshFlowSettings.particlePresetName = "Turbulent";
    document.dynamicMeshFlowSettings.branchingStrength = 0.7F;
    document.dynamicMeshFlowSettings.eddyStrength = 0.8F;
    document.dynamicMeshFlowSettings.topologyResponse = 1.1F;
    document.dynamicMeshFlowSettings.attractors = {{
        .id = 12U,
        .name = "field pull",
        .position = {1.0F, 2.0F, 3.0F},
        .radiusMeters = 1.8F,
        .strength = 0.65F,
        .enabled = true,
    }};

    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_water_sources.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveWaterSourcesDocument(document, outputPath, &errorMessage));
    {
        std::ifstream savedSources{outputPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedSources},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find(
                  "\"schema_version\": " +
                  std::to_string(invisible_places::serialization::kWaterSourcesDocumentSchemaVersion)) !=
              std::string::npos);
        CHECK(savedJson.find("\"water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_source_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_trail_geometry\"") != std::string::npos);
        CHECK(savedJson.find("\"start_fade_full_distance_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"end_fade_random_begin_distance_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"water_path_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"water_lane_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"water_trail_profiles\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_path_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_lane_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"selected_water_trail_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_path_profile_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_lane_profile_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_trail_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"path_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"lane_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_profile\"") != std::string::npos);
        CHECK(savedJson.find("\"path_profile_locked\"") != std::string::npos);
        CHECK(savedJson.find("\"lane_profile_locked\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_profile_locked\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_lane_count\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_smoothness\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_turbulence\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_momentum\"") != std::string::npos);
        CHECK(savedJson.find("\"normal_turbulence_response\"") != std::string::npos);
        CHECK(savedJson.find("\"settings_assignment\"") != std::string::npos);
        CHECK(savedJson.find("\"linked_settings_emitter_id\"") != std::string::npos);
        CHECK(savedJson.find("\"water_flow_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_dynamic_mesh_flow_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"mesh_path\"") == std::string::npos);
        CHECK(savedJson.find("\"automatic_sources\"") == std::string::npos);
        CHECK(savedJson.find("\"attractors\"") == std::string::npos);
        CHECK(savedJson.find("\"emitter_motions\"") == std::string::npos);
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
    CHECK(loaded->trailGeometry.trailLengthMeters == Catch::Approx(1.25F));
    CHECK(loaded->trailGeometry.pointSpacingMeters == Catch::Approx(0.044F));
    CHECK(loaded->trailGeometry.widthMeters == Catch::Approx(0.018F));
    CHECK(loaded->trailGeometry.streakLengthMeters == Catch::Approx(0.12F));
    CHECK(loaded->trailGeometry.startFadeEnabled);
    CHECK(loaded->trailGeometry.startFadeFullDistanceMeters == Catch::Approx(0.28F));
    CHECK(loaded->trailGeometry.startFadeRandomBeginDistanceMeters == Catch::Approx(0.09F));
    CHECK(loaded->trailGeometry.endFadeEnabled);
    CHECK(loaded->trailGeometry.endFadeFullDistanceMeters == Catch::Approx(0.39F));
    CHECK(loaded->trailGeometry.endFadeRandomBeginDistanceMeters == Catch::Approx(0.14F));
    REQUIRE(loaded->pathProfiles.size() == 1U);
    CHECK(loaded->pathProfiles[0].name == "Source Path");
    CHECK(loaded->pathProfiles[0].settings.pathLength == Catch::Approx(12.5F));
    REQUIRE(loaded->laneProfiles.size() == 1U);
    CHECK(loaded->laneProfiles[0].name == "Sheet Lanes");
    CHECK(loaded->laneProfiles[0].settings.trailCountTotal == 777U);
    CHECK(loaded->laneProfiles[0].settings.laneCount == 13U);
    CHECK(loaded->laneProfiles[0].settings.turbulence == Catch::Approx(0.42F));
    REQUIRE(loaded->trailProfiles.size() == 1U);
    CHECK(loaded->trailProfiles[0].name == "Mist Trail");
    CHECK(loaded->trailProfiles[0].geometry.trailLengthMeters == Catch::Approx(2.2F));
    CHECK(invisible_places::style::ScalarConstant(loaded->trailProfiles[0].style.opacity) ==
          Catch::Approx(0.27F));
    CHECK(loaded->selectedPathProfileName == "Source Path_edited");
    CHECK(loaded->selectedLaneProfileName == "Sheet Lanes_edited");
    CHECK(loaded->selectedTrailProfileName == "Mist Trail_edited");
    REQUIRE(loaded->tempPathProfileSettings.has_value());
    CHECK(loaded->tempPathProfileSettings->coverage == Catch::Approx(0.88F));
    REQUIRE(loaded->tempLaneProfileSettings.has_value());
    CHECK(loaded->tempLaneProfileSettings->laneCrossing == Catch::Approx(0.63F));
    REQUIRE(loaded->tempTrailProfile.has_value());
    CHECK(loaded->tempTrailProfile->geometry.widthMeters == Catch::Approx(0.033F));
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
    CHECK(loaded->emitters[0].pathProfileName == "Source Path");
    CHECK(loaded->emitters[0].laneProfileName == "Sheet Lanes");
    CHECK(loaded->emitters[0].trailProfileName == "Mist Trail");
    CHECK(loaded->emitters[0].pathProfileLocked);
    CHECK_FALSE(loaded->emitters[0].laneProfileLocked);
    CHECK(loaded->emitters[0].trailProfileLocked);
    REQUIRE(loaded->manualFlowPaths.size() == 1U);
    CHECK(loaded->manualFlowPaths[0].name == "Authored channel");
    CHECK(loaded->manualFlowPaths[0].laneProfileLocked);
    CHECK_FALSE(loaded->manualFlowPaths[0].trailProfileLocked);
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
    CHECK(loaded->flowTrailSettings.trailCountTotal == 222U);
    CHECK(loaded->flowTrailSettings.trailStreakLengthMeters == Catch::Approx(0.052F));
    CHECK(loaded->flowTrailSettings.laneCrossing == Catch::Approx(0.31F));
    CHECK(loaded->flowTrailSettings.startFadeEnabled);
    CHECK(loaded->flowTrailSettings.startFadeFullDistanceMeters == Catch::Approx(0.22F));
    CHECK(loaded->flowTrailSettings.startFadeRandomBeginDistanceMeters == Catch::Approx(0.08F));
    CHECK(loaded->flowTrailSettings.endFadeEnabled);
    CHECK(loaded->flowTrailSettings.endFadeFullDistanceMeters == Catch::Approx(0.41F));
    CHECK(loaded->flowTrailSettings.endFadeRandomBeginDistanceMeters == Catch::Approx(0.16F));
    CHECK(loaded->dynamicMeshFlowSettings.enabled);
    CHECK(loaded->dynamicMeshFlowSettings.meshPath.empty());
    CHECK(loaded->dynamicMeshFlowSettings.previewParticleLimit == 444U);
    CHECK(loaded->dynamicMeshFlowSettings.gpuPreviewEnabled);
    CHECK_FALSE(loaded->dynamicMeshFlowSettings.showTrails);
    CHECK(loaded->dynamicMeshFlowSettings.automaticSources);
    CHECK(loaded->dynamicMeshFlowSettings.particleCapacity == 4096U);
    CHECK(loaded->dynamicMeshFlowSettings.historyLength == 24U);
    CHECK(
        loaded->dynamicMeshFlowSettings.sourceBandWidthMeters ==
        Catch::Approx(0.75F));
    CHECK(
        loaded->dynamicMeshFlowSettings.sharedWindStrength ==
        Catch::Approx(0.29F));
    CHECK(
        loaded->dynamicMeshFlowSettings.rockResponse.persistenceSeconds ==
        Catch::Approx(5.0F));
    CHECK(
        loaded->dynamicMeshFlowSettings.vegetationResponse.twinkle ==
        Catch::Approx(2.1F));
    CHECK(loaded->dynamicMeshFlowSettings.particlePresetName == "Turbulent");
    CHECK(loaded->dynamicMeshFlowSettings.branchingStrength == Catch::Approx(0.7F));
    CHECK(loaded->dynamicMeshFlowSettings.eddyStrength == Catch::Approx(0.8F));
    CHECK(loaded->dynamicMeshFlowSettings.topologyResponse == Catch::Approx(1.1F));
    CHECK(loaded->dynamicMeshFlowSettings.attractors.empty());
    CHECK(loaded->dynamicMeshFlowSettings.emitterMotions.empty());

    const auto legacySourcesPath =
        std::filesystem::temp_directory_path() / "invisible_places_water_sources_legacy_mesh_path.json";
    {
        std::ofstream legacySources{legacySourcesPath};
        legacySources << R"({
  "schema_version": 9,
  "water_dynamic_mesh_flow_settings": {
    "enabled": true,
    "mesh_path": "Data/LegacyScene/Legacy-MESH.ply"
  },
  "water_emitters": [{"id": 91, "path_profile": "Default"}],
  "water_manual_flow_paths": [{
    "id": 92,
    "control_points": [[0, 0, 0], [1, 0, 0]],
    "lane_profile": "Default",
    "trail_profile": "Default"
  }]
})";
    }
    const auto legacySources =
        invisible_places::serialization::LoadWaterSourcesDocument(legacySourcesPath, &errorMessage);
    REQUIRE(legacySources.has_value());
    CHECK(legacySources->dynamicMeshFlowSettings.meshPath.generic_string() ==
          "Data/LegacyScene/Legacy-MESH.ply");
    CHECK(legacySources->dynamicMeshFlowSettings.showTrails);
    CHECK(legacySources->dynamicMeshFlowSettings.automaticSources);
    CHECK(legacySources->dynamicMeshFlowSettings.particleCapacity == 4096U);
    CHECK(legacySources->dynamicMeshFlowSettings.historyLength == 24U);
    CHECK(
        legacySources->dynamicMeshFlowSettings.sourceBandWidthMeters ==
        Catch::Approx(0.75F));
    CHECK(
        legacySources->dynamicMeshFlowSettings.sourceBandFraction ==
        Catch::Approx(0.04F));
    CHECK(
        legacySources->dynamicMeshFlowSettings.trailWidthMeters ==
        Catch::Approx(0.0025F));
    CHECK(
        legacySources->dynamicMeshFlowSettings.speedMetersPerSecond ==
        Catch::Approx(0.26F));
    CHECK(
        legacySources->dynamicMeshFlowSettings.particleNoiseStrength ==
        Catch::Approx(0.10F));
    CHECK(
        legacySources->dynamicMeshFlowSettings.sharedWindStrength ==
        Catch::Approx(0.035F));
    REQUIRE(legacySources->emitters.size() == 1U);
    CHECK_FALSE(legacySources->emitters[0].pathProfileLocked);
    CHECK_FALSE(legacySources->emitters[0].laneProfileLocked);
    CHECK_FALSE(legacySources->emitters[0].trailProfileLocked);
    REQUIRE(legacySources->manualFlowPaths.size() == 1U);
    CHECK_FALSE(legacySources->manualFlowPaths[0].laneProfileLocked);
    CHECK_FALSE(legacySources->manualFlowPaths[0].trailProfileLocked);
    REQUIRE(loaded->pathCache.has_value());
    CHECK(loaded->pathCache->supportLayerPath == std::filesystem::path{"Data/Site2 -5mm.ply"});
    CHECK(loaded->pathCache->supportSignature == "Data/Site2 -5mm.ply|points=4096");
    CHECK(loaded->pathCache->emitterSettingsFingerprint == "source-fingerprint");
    REQUIRE(loaded->pathCache->branches.size() == 1U);
    CHECK(loaded->pathCache->branches[0].id == 77U);
    REQUIRE(loaded->pathCache->branches[0].rawAnchors.size() == 2U);
    CHECK(loaded->pathCache->branches[0].rawAnchors[1].pathDistance == Catch::Approx(1.75F));
    auto recomputedSourceCache = loaded->pathCache.value();
    CHECK_FALSE(invisible_places::water::WaterPathAnalysisCacheCompatible(recomputedSourceCache));
    invisible_places::water::EnsureWaterPathAnalysis(&recomputedSourceCache);
    CHECK(invisible_places::water::WaterPathAnalysisCacheCompatible(recomputedSourceCache));
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

TEST_CASE("GPU rain settings and visual profile round-trip while route rain stays disabled", "[water][rain][serialization]") {
    auto rain = invisible_places::water::DefaultRainRuntimeSettings();
    rain.enabled = true;
    rain.impactEffectsEnabled = true;
    rain.sandEffectsEnabled = false;
    rain.intensityPreset = invisible_places::water::RainIntensityPreset::HeavyDownpour;
    rain.visualProfileName = "Rain Downpour";
    rain.activeParticleCount = 18'432U;
    rain.seed = 9876U;
    rain.rainLevel = 0.73F;
    rain.density = 0.81F;
    rain.fallSpeedMetersPerSecond = 12.25F;
    rain.windSpeedMetersPerSecond = 1.75F;
    rain.gustStrength = 0.62F;
    rain.weatherFrontStrength = 0.48F;
    rain.rockEffectScale = 1.35F;
    rain.ringImpact.thicknessScale = 0.50F;
    rain.nearSurface = {
        .approachDistanceMeters = 0.41F,
        .minimumSpeedFactor = 0.22F,
        .squish = 0.83F,
        .normalAlignment = 0.61F,
    };
    rain.rockImpact = {
        .edgeBreakup = 0.58F,
        .spreadSpeed = 0.92F,
        .centreFalloff = 0.44F,
        .heightBias = 1.30F,
        .persistence = 2.20F,
    };
    rain.vegetationImpact = {
        .twinkle = 2.70F,
        .propagationMetersPerSecond = 0.80F,
        .hopSpacingMeters = 0.055F,
        .streamWidthMeters = 0.012F,
        .streamSpread = 1.10F,
    };

    auto visual = invisible_places::water::RainVisualPreset("Rain Downpour");
    visual.colour = {0.31F, 0.52F, 0.86F};
    visual.widthMeters = 0.007F;
    visual.streakLengthMeters = 0.29F;
    visual.opacity = 0.77F;

    invisible_places::serialization::ProjectDocument project;
    project.projectName = "GPU Rain Roundtrip";
    project.waterRainSettings = rain;
    project.waterRainVisualSettings = visual;
    const auto projectPath =
        std::filesystem::temp_directory_path() / "invisible_places_gpu_rain_project.json";
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(project, projectPath, &errorMessage));

    nlohmann::json savedJson;
    {
        std::ifstream input{projectPath};
        input >> savedJson;
    }
    const auto& rainJson = savedJson.at("water_rain_settings");
    CHECK(rainJson.at("version").get<int>() == 3);
    CHECK(rainJson.at("near_surface").at("squish").get<float>() == Catch::Approx(0.83F));
    CHECK(rainJson.at("rock_impact").at("edge_breakup").get<float>() == Catch::Approx(0.58F));
    CHECK(
        rainJson.at("sand_impact").at("ring_thickness_scale").get<float>() ==
        Catch::Approx(0.50F));
    CHECK(
        rainJson.at("vegetation_impact").at("stream_width_meters").get<float>() ==
        Catch::Approx(0.012F));
    CHECK_FALSE(savedJson.contains("selected_water_rain_trail_profile"));
    CHECK_FALSE(savedJson.contains("temp_water_rain_trail_profile"));
    CHECK_FALSE(rainJson.contains("surface_runoff_enabled"));
    CHECK_FALSE(rainJson.contains("route_anchor_count"));

    const auto loaded = invisible_places::serialization::LoadProjectDocument(projectPath, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->waterRainSettings.enabled);
    CHECK_FALSE(loaded->waterRainSettings.sandEffectsEnabled);
    CHECK(loaded->waterRainSettings.intensityPreset ==
          invisible_places::water::RainIntensityPreset::HeavyDownpour);
    CHECK(loaded->waterRainSettings.activeParticleCount == 18'432U);
    CHECK(loaded->waterRainSettings.rainLevel == Catch::Approx(0.73F));
    CHECK(loaded->waterRainSettings.windSpeedMetersPerSecond == Catch::Approx(1.75F));
    CHECK(loaded->waterRainSettings.nearSurface.approachDistanceMeters == Catch::Approx(0.41F));
    CHECK(loaded->waterRainSettings.nearSurface.minimumSpeedFactor == Catch::Approx(0.22F));
    CHECK(loaded->waterRainSettings.nearSurface.squish == Catch::Approx(0.83F));
    CHECK(loaded->waterRainSettings.nearSurface.normalAlignment == Catch::Approx(0.61F));
    CHECK(loaded->waterRainSettings.rockImpact.edgeBreakup == Catch::Approx(0.58F));
    CHECK(loaded->waterRainSettings.rockImpact.spreadSpeed == Catch::Approx(0.92F));
    CHECK(loaded->waterRainSettings.rockImpact.centreFalloff == Catch::Approx(0.44F));
    CHECK(loaded->waterRainSettings.rockImpact.heightBias == Catch::Approx(1.30F));
    CHECK(loaded->waterRainSettings.rockImpact.persistence == Catch::Approx(2.20F));
    CHECK(loaded->waterRainSettings.ringImpact.thicknessScale == Catch::Approx(0.50F));
    CHECK(loaded->waterRainSettings.vegetationImpact.twinkle == Catch::Approx(2.70F));
    CHECK(
        loaded->waterRainSettings.vegetationImpact.propagationMetersPerSecond ==
        Catch::Approx(0.80F));
    CHECK(loaded->waterRainSettings.vegetationImpact.hopSpacingMeters == Catch::Approx(0.055F));
    CHECK(loaded->waterRainSettings.vegetationImpact.streamWidthMeters == Catch::Approx(0.012F));
    CHECK(loaded->waterRainSettings.vegetationImpact.streamSpread == Catch::Approx(1.10F));
    CHECK(loaded->waterRainVisualSettings.widthMeters == Catch::Approx(0.007F));
    CHECK(loaded->waterRainVisualSettings.streakLengthMeters == Catch::Approx(0.29F));
    CHECK(loaded->waterRainVisualSettings.colour[2] == Catch::Approx(0.86F));

    const auto versionTwoPath =
        std::filesystem::temp_directory_path() / "invisible_places_gpu_rain_v2_project.json";
    auto versionTwoJson = savedJson;
    auto& versionTwoRain = versionTwoJson["water_rain_settings"];
    versionTwoRain["version"] = 2;
    versionTwoRain.erase("near_surface");
    versionTwoRain.erase("rock_impact");
    versionTwoRain.erase("vegetation_impact");
    {
        std::ofstream output{versionTwoPath, std::ios::trunc};
        output << versionTwoJson.dump(2);
    }
    const auto versionTwo =
        invisible_places::serialization::LoadProjectDocument(versionTwoPath, &errorMessage);
    REQUIRE(versionTwo.has_value());
    const auto defaults = invisible_places::water::DefaultRainRuntimeSettings();
    CHECK(
        versionTwo->waterRainSettings.nearSurface.approachDistanceMeters ==
        Catch::Approx(defaults.nearSurface.approachDistanceMeters));
    CHECK(
        versionTwo->waterRainSettings.rockImpact.edgeBreakup ==
        Catch::Approx(defaults.rockImpact.edgeBreakup));
    CHECK(
        versionTwo->waterRainSettings.ringImpact.thicknessScale ==
        Catch::Approx(defaults.ringImpact.thicknessScale));
    CHECK(
        versionTwo->waterRainSettings.vegetationImpact.twinkle ==
        Catch::Approx(defaults.vegetationImpact.twinkle));

    const auto legacyPath =
        std::filesystem::temp_directory_path() / "invisible_places_route_rain_legacy.json";
    savedJson["water_rain_settings"] = {
        {"enabled", true},
        {"surface_runoff_enabled", true},
        {"vegetation_interception_enabled", true},
        {"drop_count", 5000},
    };
    {
        std::ofstream output{legacyPath, std::ios::trunc};
        output << savedJson.dump(2);
    }
    const auto legacy = invisible_places::serialization::LoadProjectDocument(legacyPath, &errorMessage);
    REQUIRE(legacy.has_value());
    CHECK_FALSE(legacy->waterRainSettings.enabled);
    CHECK(legacy->waterRainSettings.activeParticleCount ==
          invisible_places::water::DefaultRainRuntimeSettings().activeParticleCount);

    invisible_places::serialization::WaterSourcesDocument sources;
    sources.rainSettings = rain;
    sources.rainVisualSettings = visual;
    const auto sourcesPath =
        std::filesystem::temp_directory_path() / "invisible_places_gpu_rain_sources.json";
    REQUIRE(invisible_places::serialization::SaveWaterSourcesDocument(sources, sourcesPath, &errorMessage));
    const auto loadedSources =
        invisible_places::serialization::LoadWaterSourcesDocument(sourcesPath, &errorMessage);
    REQUIRE(loadedSources.has_value());
    CHECK(loadedSources->rainSettings.seed == 9876U);
    CHECK(loadedSources->rainSettings.nearSurface.squish == Catch::Approx(0.83F));
    CHECK(loadedSources->rainSettings.rockImpact.persistence == Catch::Approx(2.20F));
    CHECK(loadedSources->rainSettings.ringImpact.thicknessScale == Catch::Approx(0.50F));
    CHECK(loadedSources->rainSettings.vegetationImpact.streamSpread == Catch::Approx(1.10F));
    CHECK(loadedSources->rainVisualSettings.opacity == Catch::Approx(0.77F));

    std::filesystem::remove(projectPath);
    std::filesystem::remove(versionTwoPath);
    std::filesystem::remove(legacyPath);
    std::filesystem::remove(sourcesPath);
}

TEST_CASE("Legacy water stream settings load and save as trail settings", "[water][serialization]") {
    const auto projectPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_water_stream_settings_project.json";
    {
        std::ofstream output{projectPath, std::ios::trunc};
        output << R"({
  "schema_version": 7,
  "project_name": "Legacy Stream Settings",
  "water_flow_stream_settings": {
    "enabled": true,
    "stream_count_total": 17,
    "lane_count": 5,
    "stream_length_meters": 0.44,
    "stream_point_spacing_meters": 0.012,
    "stream_width_meters": 0.006,
    "stream_world_length_meters": 0.052,
    "stream_smoothness": 0.71,
    "stream_looseness": 0.14,
    "lane_crossing": 0.33,
    "speed_meters_per_second": 0.27
  }
})";
    }

    std::string errorMessage;
    const auto loadedProject = invisible_places::serialization::LoadProjectDocument(projectPath, &errorMessage);
    REQUIRE(loadedProject.has_value());
    CHECK(loadedProject->waterFlowTrailSettings.trailCountTotal == 17U);
    CHECK(loadedProject->waterFlowTrailSettings.laneCount == 5U);
    CHECK(loadedProject->waterFlowTrailSettings.trailLengthMeters == Catch::Approx(0.44F));
    CHECK(loadedProject->waterFlowTrailSettings.trailPointSpacingMeters == Catch::Approx(0.012F));
    CHECK(loadedProject->waterFlowTrailSettings.trailWidthMeters == Catch::Approx(0.006F));
    CHECK(loadedProject->waterFlowTrailSettings.trailStreakLengthMeters == Catch::Approx(0.052F));
    CHECK(loadedProject->waterFlowTrailSettings.trailSmoothness == Catch::Approx(0.71F));
    CHECK(loadedProject->waterFlowTrailSettings.trailLooseness == Catch::Approx(0.14F));

    const auto projectRoundTripPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_water_stream_settings_project_roundtrip.json";
    REQUIRE(invisible_places::serialization::SaveProjectDocument(*loadedProject, projectRoundTripPath, &errorMessage));
    {
        std::ifstream savedProject{projectRoundTripPath};
        const std::string savedJson{
            std::istreambuf_iterator<char>{savedProject},
            std::istreambuf_iterator<char>{}};
        CHECK(savedJson.find("\"water_flow_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_count_total\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_streak_length_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"water_flow_stream_settings\"") == std::string::npos);
        CHECK(savedJson.find("\"stream_count_total\"") == std::string::npos);
        CHECK(savedJson.find("\"streamline_count\"") == std::string::npos);
        CHECK(savedJson.find("\"stream_world_length_meters\"") == std::string::npos);
    }

    const auto sourcesPath =
        std::filesystem::temp_directory_path() / "invisible_places_legacy_water_stream_settings_sources.json";
    {
        std::ofstream output{sourcesPath, std::ios::trunc};
        output << R"({
  "schema_version": 7,
  "water_flow_stream_settings": {
    "stream_count_total": 11,
    "stream_point_spacing_meters": 0.021,
    "stream_world_length_meters": 0.073
  }
})";
    }
    const auto loadedSources = invisible_places::serialization::LoadWaterSourcesDocument(sourcesPath, &errorMessage);
    REQUIRE(loadedSources.has_value());
    CHECK(loadedSources->flowTrailSettings.trailCountTotal == 11U);
    CHECK(loadedSources->flowTrailSettings.trailPointSpacingMeters == Catch::Approx(0.021F));
    CHECK(loadedSources->flowTrailSettings.trailStreakLengthMeters == Catch::Approx(0.073F));

    std::filesystem::remove(projectPath);
    std::filesystem::remove(projectRoundTripPath);
    std::filesystem::remove(sourcesPath);
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
	    "render_mode": ")" << modeName << R"("
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
        solid.falloffProfile ==
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc);

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

    const auto gaussianSprite = loadLegacyMode("gaussian_point_sprite");
    CHECK(
        gaussianSprite.falloffProfile ==
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::Gaussian);

    std::filesystem::remove(presetPath);
}

TEST_CASE("Point alpha contribution policy is shared by preview and export selection", "[point-style]") {
    CHECK_FALSE(invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(0.0F));
    CHECK_FALSE(invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(-0.01F));
    CHECK(invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(0.01F));
}

TEST_CASE("Point material variant resolver selects simple and unified paths", "[point-style]") {
    using invisible_places::renderer::pointcloud::PointCloudColorMode;
    using invisible_places::renderer::pointcloud::PointCloudDensityCompensation;
    using invisible_places::renderer::pointcloud::PointCloudMaterialVariant;
    using invisible_places::renderer::pointcloud::ResolvePointCloudMaterialVariant;
    using invisible_places::style::ParameterSourceMode;

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.colorMode = PointCloudColorMode::SourceRgb;
    invisible_places::style::SetScalarConstant(&style.opacity, 0.65F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 1.25F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::ConstantSimple);
    CHECK(
        ResolvePointCloudMaterialVariant(
            style,
            PointCloudDensityCompensation{},
            true) == PointCloudMaterialVariant::Unified);

    const invisible_places::renderer::pointcloud::PointCloudStyleState opaqueStyle;
    CHECK(ResolvePointCloudMaterialVariant(opaqueStyle) == PointCloudMaterialVariant::OpaqueHardDisc);
    CHECK(
        ResolvePointCloudMaterialVariant(
            opaqueStyle,
            PointCloudDensityCompensation{},
            true) == PointCloudMaterialVariant::Unified);

    auto fieldOpacity = style;
    fieldOpacity.opacity.mode = ParameterSourceMode::FieldMapped;
    CHECK(ResolvePointCloudMaterialVariant(fieldOpacity) == PointCloudMaterialVariant::Unified);

    auto fieldEmission = style;
    fieldEmission.emissiveStrength.mode = ParameterSourceMode::FieldMapped;
    CHECK(ResolvePointCloudMaterialVariant(fieldEmission) == PointCloudMaterialVariant::Unified);

    auto fieldColormapPosition = style;
    fieldColormapPosition.colormapPosition.mode = ParameterSourceMode::FieldMapped;
    CHECK(ResolvePointCloudMaterialVariant(fieldColormapPosition) == PointCloudMaterialVariant::Unified);

    auto depthFade = style;
    invisible_places::style::SetScalarConstant(&depthFade.depthFade, 0.5F);
    depthFade.depthFade.active = true;
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
    const auto preparedPath = invisible_places::camera::PrepareAnimationPathEvaluation(path);
    REQUIRE(preparedPath.valid);
    const auto preparedEvaluation =
        invisible_places::camera::EvaluatePreparedAnimationPath(preparedPath, 1.0F);
    CHECK(evaluation.camera.position[0] == Catch::Approx(path.keys[1].cameraPosition[0]));
    CHECK(evaluation.camera.position[1] == Catch::Approx(path.keys[1].cameraPosition[1]));
    CHECK(evaluation.camera.position[2] == Catch::Approx(path.keys[1].cameraPosition[2]));
    CHECK(evaluation.focusPoint[0] == Catch::Approx(path.keys[1].focusPoint[0]));
    CHECK(evaluation.focusPoint[1] == Catch::Approx(path.keys[1].focusPoint[1]));
    CHECK(evaluation.focusPoint[2] == Catch::Approx(path.keys[1].focusPoint[2]));
    CHECK(preparedEvaluation.camera.position[0] == Catch::Approx(evaluation.camera.position[0]));
    CHECK(preparedEvaluation.camera.position[1] == Catch::Approx(evaluation.camera.position[1]));
    CHECK(preparedEvaluation.camera.position[2] == Catch::Approx(evaluation.camera.position[2]));
    CHECK(preparedEvaluation.focusPoint[0] == Catch::Approx(evaluation.focusPoint[0]));
    CHECK(preparedEvaluation.focusPoint[1] == Catch::Approx(evaluation.focusPoint[1]));
    CHECK(preparedEvaluation.focusPoint[2] == Catch::Approx(evaluation.focusPoint[2]));
}

TEST_CASE("Animation path reports world-space speeds and retimes from average speed", "[camera][animation]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 60;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 1.0F, 0.0F}, .durationFrames = 30},
        {.cameraPosition = {10.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 5.0F, 0.0F}, .durationFrames = 30},
    };

    const auto preparedPath = invisible_places::camera::PrepareAnimationPathEvaluation(path);
    const auto stats = invisible_places::camera::MeasureAnimationPathMotion(path, 0.5F, 32U);
    const auto preparedStats =
        invisible_places::camera::MeasurePreparedAnimationPathMotion(preparedPath, 0.5F, 32U);
    CHECK(stats.durationSeconds == Catch::Approx(2.0F));
    CHECK(stats.cameraDistance == Catch::Approx(10.0F));
    CHECK(stats.targetDistance == Catch::Approx(4.0F));
    CHECK(stats.averageCameraSpeed == Catch::Approx(5.0F));
    CHECK(stats.averageTargetSpeed == Catch::Approx(2.0F));
    CHECK(stats.currentCameraSpeed == Catch::Approx(5.0F));
    CHECK(stats.currentTargetSpeed == Catch::Approx(2.0F));
    CHECK(preparedStats.durationSeconds == Catch::Approx(stats.durationSeconds));
    CHECK(preparedStats.cameraDistance == Catch::Approx(stats.cameraDistance));
    CHECK(preparedStats.targetDistance == Catch::Approx(stats.targetDistance));
    CHECK(preparedStats.currentCameraSpeed == Catch::Approx(stats.currentCameraSpeed));
    CHECK(preparedStats.currentTargetSpeed == Catch::Approx(stats.currentTargetSpeed));

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

TEST_CASE("Animation path perceived flow favours the close sweep when retiming segments", "[camera][animation]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 120;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 1.0F, 0.0F}, .durationFrames = 60},
        {.cameraPosition = {4.0F, 0.0F, 0.0F}, .focusPoint = {4.0F, 1.0F, 0.0F}, .durationFrames = 60},
        {.cameraPosition = {4.05F, 0.0F, 0.0F}, .focusPoint = {4.05F, 1.0F, 0.0F}, .durationFrames = 60},
    };

    const auto preparedPath = invisible_places::camera::PrepareAnimationPathEvaluation(path);
    REQUIRE(preparedPath.valid);
    const auto flowSamples =
        invisible_places::camera::MeasurePreparedAnimationPathPerceivedFlow(preparedPath, 160U);
    REQUIRE(flowSamples.size() == 160U);
    CHECK(flowSamples.front().normalizedPosition == Catch::Approx(0.0F));
    CHECK(flowSamples.back().normalizedPosition == Catch::Approx(1.0F));

    float sweepFlow = 0.0F;
    float dwellFlow = 0.0F;
    std::size_t sweepCount = 0U;
    std::size_t dwellCount = 0U;
    for (const auto& sample : flowSamples) {
        CHECK(std::isfinite(sample.screenSpeed));
        CHECK(sample.screenSpeed >= 0.0F);
        if (sample.normalizedPosition < 0.5F) {
            sweepFlow += sample.screenSpeed;
            ++sweepCount;
        } else {
            dwellFlow += sample.screenSpeed;
            ++dwellCount;
        }
    }
    REQUIRE(sweepCount > 0U);
    REQUIRE(dwellCount > 0U);
    CHECK(sweepFlow / static_cast<float>(sweepCount) >
          dwellFlow / static_cast<float>(dwellCount));

    const auto segmentFrames =
        invisible_places::camera::ComputeConstantPerceivedSpeedSegmentFrames(path);
    REQUIRE(segmentFrames.size() == 2U);
    CHECK(segmentFrames[0] + segmentFrames[1] == 120U);
    CHECK(segmentFrames[0] > segmentFrames[1]);
    CHECK(segmentFrames[0] >= 1U);
    CHECK(segmentFrames[1] >= 1U);
}

TEST_CASE("Animation path perceived flow rises when the subject is close", "[camera][animation]") {
    const auto buildPath = [](float focusY) {
        invisible_places::camera::AnimationPath path;
        path.durationFrames = 60;
        path.keys = {
            {.cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {1.0F, focusY, 0.0F}, .durationFrames = 30},
            {.cameraPosition = {2.0F, 0.0F, 0.0F}, .focusPoint = {1.0F, focusY, 0.0F}, .durationFrames = 30},
        };
        return path;
    };

    const auto meanFlow = [](const invisible_places::camera::AnimationPath& path) {
        const auto preparedPath = invisible_places::camera::PrepareAnimationPathEvaluation(path);
        REQUIRE(preparedPath.valid);
        const auto flowSamples =
            invisible_places::camera::MeasurePreparedAnimationPathPerceivedFlow(preparedPath, 96U);
        REQUIRE_FALSE(flowSamples.empty());
        float totalFlow = 0.0F;
        for (const auto& sample : flowSamples) {
            CHECK(std::isfinite(sample.screenSpeed));
            CHECK(sample.screenSpeed >= 0.0F);
            CHECK(std::isfinite(sample.screenVelocity[0U]));
            CHECK(std::isfinite(sample.screenVelocity[1U]));
            if (sample.screenSpeed > 1.0e-5F) {
                CHECK(
                    std::hypot(
                        sample.screenVelocity[0U],
                        sample.screenVelocity[1U]) ==
                    Catch::Approx(sample.screenSpeed).epsilon(1.0e-4F));
            }
            totalFlow += sample.screenSpeed;
        }
        return totalFlow / static_cast<float>(flowSamples.size());
    };

    const float closeMeanFlow = meanFlow(buildPath(1.0F));
    const float farMeanFlow = meanFlow(buildPath(50.0F));
    CHECK(closeMeanFlow > farMeanFlow);
}

TEST_CASE("Animation flow diagnostics preserve scalar speed and report signed translation and roll",
          "[camera][animation][flow-diagnostics]") {
    using invisible_places::camera::AnimationPath;
    const auto measureMiddle = [](const AnimationPath& path) {
        const auto context =
            invisible_places::camera::PrepareAnimationPathEvaluation(path);
        REQUIRE(context.valid);
        const auto samples = invisible_places::camera::
            MeasurePreparedAnimationPathPerceivedFlow(context, 33U);
        REQUIRE(samples.size() == 33U);
        return samples[samples.size() / 2U];
    };

    AnimationPath horizontal;
    horizontal.durationFrames = 30U;
    horizontal.keys = {
        {.id = "left",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 10.0F, 0.0F},
         .fovDegrees = 60.0F,
         .durationFrames = 30U},
        {.id = "right",
         .cameraPosition = {1.0F, 0.0F, 0.0F},
         .focusPoint = {1.0F, 10.0F, 0.0F},
         .fovDegrees = 60.0F,
         .durationFrames = 30U},
    };
    const auto horizontalFlow = measureMiddle(horizontal);
    const float legacyScalar =
        (1.0F / 10.0F) / glm::radians(60.0F);
    CHECK(horizontalFlow.screenSpeed ==
          Catch::Approx(legacyScalar).epsilon(2.0e-3F));
    CHECK(horizontalFlow.screenVelocity[0U] < 0.0F);
    CHECK(std::abs(horizontalFlow.screenVelocity[1U]) < 1.0e-4F);
    CHECK(horizontalFlow.middleScreenVelocity[0U] < 0.0F);
    CHECK(horizontalFlow.topScreenVelocity[0U] ==
          Catch::Approx(horizontalFlow.middleScreenVelocity[0U])
              .margin(1.0e-4F));
    CHECK(horizontalFlow.bottomScreenVelocity[0U] ==
          Catch::Approx(horizontalFlow.middleScreenVelocity[0U])
              .margin(1.0e-4F));
    CHECK(std::abs(horizontalFlow.imageRotationDegreesPerSecond) < 1.0e-3F);

    auto vertical = horizontal;
    vertical.keys[1U].cameraPosition = {0.0F, 0.0F, 1.0F};
    vertical.keys[1U].focusPoint = {0.0F, 10.0F, 1.0F};
    const auto verticalFlow = measureMiddle(vertical);
    CHECK(verticalFlow.screenVelocity[1U] < 0.0F);
    CHECK(std::abs(verticalFlow.screenVelocity[0U]) < 1.0e-4F);

    const auto makeRoll = [](float degrees) {
        AnimationPath roll;
        roll.durationFrames = 30U;
        const float halfAngle = glm::radians(degrees) * 0.5F;
        roll.keys = {
            {.id = "roll-start",
             .cameraPosition = {0.0F, 0.0F, 0.0F},
             .focusPoint = {0.0F, 0.0F, -10.0F},
             .hasOrientation = true,
             .orientation = {0.0F, 0.0F, 0.0F, 1.0F},
             .durationFrames = 30U},
            {.id = "roll-end",
             .cameraPosition = {0.0F, 0.0F, 0.0F},
             .focusPoint = {0.0F, 0.0F, -10.0F},
             .hasOrientation = true,
             .orientation = {
                 0.0F,
                 0.0F,
                 std::sin(halfAngle),
                 std::cos(halfAngle)},
             .durationFrames = 30U},
        };
        return roll;
    };
    const auto clockwise = measureMiddle(makeRoll(20.0F));
    const auto counterClockwise = measureMiddle(makeRoll(-20.0F));
    CHECK(std::abs(clockwise.imageRotationDegreesPerSecond) > 10.0F);
    CHECK(std::abs(counterClockwise.imageRotationDegreesPerSecond) > 10.0F);
    CHECK(clockwise.imageRotationDegreesPerSecond *
              counterClockwise.imageRotationDegreesPerSecond <
          0.0F);
    CHECK(clockwise.topScreenVelocity[0U] *
              clockwise.bottomScreenVelocity[0U] <
          0.0F);
}

TEST_CASE("Animation speed equalization preserves the legacy mode and gives framing roll more time",
          "[camera][animation][speed-equalization]") {
    using invisible_places::camera::AnimationPath;
    using invisible_places::camera::AnimationSpeedEqualizationMode;

    AnimationPath path;
    path.durationFrames = 120U;
    const float halfRoll = 0.5F * glm::radians(60.0F);
    path.keys = {
        {.id = "steady-start",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 0.0F, -10.0F},
         .hasOrientation = true,
         .orientation = {0.0F, 0.0F, 0.0F, 1.0F},
         .durationFrames = 60U},
        {.id = "steady-end",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 0.0F, -10.0F},
         .hasOrientation = true,
         .orientation = {0.0F, 0.0F, 0.0F, 1.0F},
         .durationFrames = 60U},
        {.id = "roll-end",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 0.0F, -10.0F},
         .hasOrientation = true,
         .orientation = {
             0.0F,
             0.0F,
             std::sin(halfRoll),
             std::cos(halfRoll)},
         .durationFrames = 60U},
    };

    const auto established = invisible_places::camera::
        ComputeConstantPerceivedSpeedSegmentFrames(path, 32U);
    const auto explicitLegacy = invisible_places::camera::
        ComputeEqualizedAnimationSegmentFrames(
            path,
            {
                .mode = AnimationSpeedEqualizationMode::PerceivedMotion,
                .samplesPerSegment = 32U,
            });
    CHECK(explicitLegacy == established);

    const auto centrePan = invisible_places::camera::
        ComputeEqualizedAnimationSegmentFrames(
            path,
            {
                .mode = AnimationSpeedEqualizationMode::CenterScreenPan,
                .samplesPerSegment = 32U,
            });
    REQUIRE(centrePan.size() == 2U);
    CHECK(centrePan[0U] == 60U);
    CHECK(centrePan[1U] == 60U);

    const auto stabilizedPan = invisible_places::camera::
        ComputeEqualizedAnimationSegmentFrames(
            path,
            {
                .mode = AnimationSpeedEqualizationMode::StabilizedPan,
                .samplesPerSegment = 32U,
            });
    REQUIRE(stabilizedPan.size() == 2U);
    CHECK(stabilizedPan[0U] + stabilizedPan[1U] == 120U);
    CHECK(stabilizedPan[1U] > stabilizedPan[0U]);
}

TEST_CASE(
    "Animation X-velocity equalization moves camera and focus keys without retiming",
    "[camera][animation][speed-equalization][x-velocity]") {
    using invisible_places::camera::AnimationPath;

    AnimationPath path;
    path.durationFrames = 120U;
    path.keys = {
        {.id = "start",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 10.0F, 0.0F},
         .durationFrames = 40U},
        {.id = "slow-a",
         .cameraPosition = {1.0F, 0.0F, 0.0F},
         .focusPoint = {1.0F, 10.0F, 0.0F},
         .durationFrames = 40U},
        {.id = "fast",
         .cameraPosition = {5.0F, 0.0F, 0.0F},
         .focusPoint = {5.0F, 10.0F, 0.0F},
         .durationFrames = 40U},
        {.id = "slow-b",
         .cameraPosition = {6.0F, 0.0F, 0.0F},
         .focusPoint = {6.0F, 10.0F, 0.0F},
         .durationFrames = 40U},
    };
    path.keys[1U].linkedCameraId = "camera-slow-a";
    path.keys[2U].linkedCameraId = "camera-fast";
    const AnimationPath original = path;

    const auto result = invisible_places::camera::
        RedistributeAnimationPathKeysForConstantScreenXVelocity(
            &path,
            1025U);

    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.movedKeyCount == 2U);
    CHECK(result.totalScreenTravel > 0.0F);
    CHECK(path.durationFrames == original.durationFrames);
    REQUIRE(path.keys.size() == original.keys.size());
    for (std::size_t index = 0U;
         index < path.keys.size();
         ++index) {
        CHECK(path.keys[index].durationFrames ==
              original.keys[index].durationFrames);
        CHECK(path.keys[index].id == original.keys[index].id);
        CHECK(path.keys[index].linkedCameraId ==
              original.keys[index].linkedCameraId);
    }
    CHECK(path.keys.front().cameraPosition ==
          original.keys.front().cameraPosition);
    CHECK(path.keys.front().focusPoint ==
          original.keys.front().focusPoint);
    CHECK(path.keys.back().cameraPosition ==
          original.keys.back().cameraPosition);
    CHECK(path.keys.back().focusPoint ==
          original.keys.back().focusPoint);
    CHECK(path.keys[1U].cameraPosition !=
          original.keys[1U].cameraPosition);
    CHECK(path.keys[1U].focusPoint !=
          original.keys[1U].focusPoint);
    CHECK(path.keys[2U].cameraPosition !=
          original.keys[2U].cameraPosition);
    CHECK(path.keys[2U].focusPoint !=
          original.keys[2U].focusPoint);
    CHECK(path.keys[1U].cameraPosition[0U] ==
          Catch::Approx(2.0F).margin(0.03F));
    CHECK(path.keys[1U].focusPoint[0U] ==
          Catch::Approx(2.0F).margin(0.03F));
    CHECK(path.keys[2U].cameraPosition[0U] ==
          Catch::Approx(4.0F).margin(0.03F));
    CHECK(path.keys[2U].focusPoint[0U] ==
          Catch::Approx(4.0F).margin(0.03F));

    const auto originalContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(original);
    REQUIRE(originalContext.valid);
    for (std::size_t keyIndex = 1U;
         keyIndex + 1U < path.keys.size();
         ++keyIndex) {
        float nearestCameraDistance =
            std::numeric_limits<float>::max();
        float nearestFocusDistance =
            std::numeric_limits<float>::max();
        for (std::uint32_t sampleIndex = 0U;
             sampleIndex <= 2000U;
             ++sampleIndex) {
            const auto sampled = invisible_places::camera::
                EvaluatePreparedAnimationPath(
                    originalContext,
                    originalContext.durationSeconds *
                        static_cast<float>(sampleIndex) / 2000.0F);
            const auto distance = [](const auto& left,
                                     const auto& right) {
                return std::hypot(
                    left[0U] - right[0U],
                    std::hypot(
                        left[1U] - right[1U],
                        left[2U] - right[2U]));
            };
            nearestCameraDistance = std::min(
                nearestCameraDistance,
                distance(
                    path.keys[keyIndex].cameraPosition,
                    sampled.camera.position));
            nearestFocusDistance = std::min(
                nearestFocusDistance,
                distance(
                    path.keys[keyIndex].focusPoint,
                    sampled.focusPoint));
        }
        CHECK(nearestCameraDistance < 0.01F);
        CHECK(nearestFocusDistance < 0.01F);
    }

    const auto flow = invisible_places::camera::
        MeasurePreparedAnimationPathPerceivedFlow(
            invisible_places::camera::
                PrepareAnimationPathEvaluation(path),
            241U);
    REQUIRE(flow.size() == 241U);
    float minimumInteriorSpeed =
        std::numeric_limits<float>::max();
    float maximumInteriorSpeed = 0.0F;
    for (std::size_t sampleIndex = 12U;
         sampleIndex + 12U < flow.size();
         ++sampleIndex) {
        const float speed =
            std::abs(flow[sampleIndex].middleScreenVelocity[0U]);
        minimumInteriorSpeed = std::min(minimumInteriorSpeed, speed);
        maximumInteriorSpeed = std::max(maximumInteriorSpeed, speed);
        CHECK(flow[sampleIndex].middleScreenVelocity[0U] < 0.0F);
    }
    CHECK(minimumInteriorSpeed > 1.0e-5F);
    CHECK(maximumInteriorSpeed / minimumInteriorSpeed < 1.35F);
}

TEST_CASE(
    "Animation Y-velocity equalization moves camera and focus keys without retiming",
    "[camera][animation][speed-equalization][y-velocity]") {
    using invisible_places::camera::AnimationPath;

    AnimationPath path;
    path.durationFrames = 120U;
    path.keys = {
        {.id = "start",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 10.0F, 0.0F},
         .durationFrames = 40U},
        {.id = "slow-a",
         .cameraPosition = {0.0F, 0.0F, 1.0F},
         .focusPoint = {0.0F, 10.0F, 1.0F},
         .durationFrames = 40U},
        {.id = "fast",
         .cameraPosition = {0.0F, 0.0F, 5.0F},
         .focusPoint = {0.0F, 10.0F, 5.0F},
         .durationFrames = 40U},
        {.id = "slow-b",
         .cameraPosition = {0.0F, 0.0F, 6.0F},
         .focusPoint = {0.0F, 10.0F, 6.0F},
         .durationFrames = 40U},
    };
    const AnimationPath original = path;

    const auto result = invisible_places::camera::
        RedistributeAnimationPathKeysForConstantScreenYVelocity(
            &path,
            1025U);

    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.movedKeyCount == 2U);
    CHECK(result.totalScreenTravel > 0.0F);
    CHECK(path.durationFrames == original.durationFrames);
    REQUIRE(path.keys.size() == original.keys.size());
    for (std::size_t index = 0U; index < path.keys.size(); ++index) {
        CHECK(path.keys[index].durationFrames ==
              original.keys[index].durationFrames);
        CHECK(path.keys[index].id == original.keys[index].id);
    }
    CHECK(path.keys.front().cameraPosition ==
          original.keys.front().cameraPosition);
    CHECK(path.keys.front().focusPoint ==
          original.keys.front().focusPoint);
    CHECK(path.keys.back().cameraPosition ==
          original.keys.back().cameraPosition);
    CHECK(path.keys.back().focusPoint ==
          original.keys.back().focusPoint);
    CHECK(path.keys[1U].cameraPosition[2U] ==
          Catch::Approx(2.0F).margin(0.03F));
    CHECK(path.keys[1U].focusPoint[2U] ==
          Catch::Approx(2.0F).margin(0.03F));
    CHECK(path.keys[2U].cameraPosition[2U] ==
          Catch::Approx(4.0F).margin(0.03F));
    CHECK(path.keys[2U].focusPoint[2U] ==
          Catch::Approx(4.0F).margin(0.03F));

    const auto flow = invisible_places::camera::
        MeasurePreparedAnimationPathPerceivedFlow(
            invisible_places::camera::
                PrepareAnimationPathEvaluation(path),
            241U);
    REQUIRE(flow.size() == 241U);
    float minimumInteriorSpeed =
        std::numeric_limits<float>::max();
    float maximumInteriorSpeed = 0.0F;
    for (std::size_t sampleIndex = 12U;
         sampleIndex + 12U < flow.size();
         ++sampleIndex) {
        const float speed =
            std::abs(flow[sampleIndex].middleScreenVelocity[1U]);
        minimumInteriorSpeed = std::min(minimumInteriorSpeed, speed);
        maximumInteriorSpeed = std::max(maximumInteriorSpeed, speed);
        CHECK(flow[sampleIndex].middleScreenVelocity[1U] < 0.0F);
    }
    CHECK(minimumInteriorSpeed > 1.0e-5F);
    CHECK(maximumInteriorSpeed / minimumInteriorSpeed < 1.35F);
}

TEST_CASE(
    "Animation rotation smoothing moves only camera keys and reduces reversal",
    "[camera][animation][speed-equalization][rotation]") {
    using invisible_places::camera::AnimationPath;

    AnimationPath path;
    path.durationFrames = 4500U;
    // A shallow, translated slow-pan rig. The position changes are subtle,
    // but their coupled yaw/pitch corrections make the optical-flow curl
    // cross zero repeatedly before smoothing.
    path.keys = {
        {.id = "k0",
         .cameraPosition = {0.401215F, 3.285530F, 1.833562F},
         .focusPoint = {4.316284F, 5.041542F, -0.085180F},
         .fovDegrees = 55.0F,
         .durationFrames = 364U},
        {.id = "k1",
         .cameraPosition = {1.298950F, 1.555252F, 1.832244F},
         .focusPoint = {5.197174F, 3.309097F, -0.086153F},
         .fovDegrees = 55.0F,
         .durationFrames = 675U},
        {.id = "k2",
         .cameraPosition = {2.191345F, -0.176544F, 1.808198F},
         .focusPoint = {6.097473F, 1.586357F, -0.101104F},
         .fovDegrees = 55.0F,
         .durationFrames = 675U},
        {.id = "k3",
         .cameraPosition = {2.901001F, -1.785889F, 1.793716F},
         .focusPoint = {6.819031F, -0.205040F, -0.115999F},
         .fovDegrees = 55.0F,
         .durationFrames = 675U},
        {.id = "k4",
         .cameraPosition = {3.527985F, -3.697243F, 1.768171F},
         .focusPoint = {7.300507F, -2.064919F, -0.168711F},
         .fovDegrees = 55.0F,
         .durationFrames = 675U},
        {.id = "k5",
         .cameraPosition = {3.883026F, -5.257271F, 1.738601F},
         .focusPoint = {7.664276F, -3.920761F, -0.186801F},
         .fovDegrees = 55.0F,
         .durationFrames = 675U},
        {.id = "k6",
         .cameraPosition = {4.365967F, -6.979370F, 1.736156F},
         .focusPoint = {8.128296F, -5.695709F, -0.140995F},
         .fovDegrees = 55.0F,
         .durationFrames = 675U},
    };
    const AnimationPath original = path;

    const auto result = invisible_places::camera::
        OptimizeAnimationCameraKeysForSmoothRotation(
            &path,
            {
                .sampleCount = 97U,
                .optimizationSweeps = 14U,
            });

    CAPTURE(
        result.beforeRotationRmsDegreesPerSecond,
        result.afterRotationRmsDegreesPerSecond,
        result.beforeRotationDirectionChanges,
        result.afterRotationDirectionChanges,
        result.movedKeyCount);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.movedKeyCount > 0U);
    CHECK(result.afterRotationRmsDegreesPerSecond <
          result.beforeRotationRmsDegreesPerSecond);
    CHECK(result.beforeRotationDirectionChanges > 0U);
    CHECK(result.afterRotationDirectionChanges <
          result.beforeRotationDirectionChanges);
    REQUIRE(path.keys.size() == original.keys.size());
    for (std::size_t index = 0U; index < path.keys.size(); ++index) {
        CHECK(path.keys[index].focusPoint ==
              original.keys[index].focusPoint);
        CHECK(path.keys[index].durationFrames ==
              original.keys[index].durationFrames);
        CHECK(path.keys[index].id == original.keys[index].id);
    }
    CHECK(path.keys.front().cameraPosition ==
          original.keys.front().cameraPosition);
    CHECK(path.keys.back().cameraPosition ==
          original.keys.back().cameraPosition);
    const auto originalContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(original);
    REQUIRE(originalContext.valid);
    for (std::size_t keyIndex = 1U;
         keyIndex + 1U < path.keys.size();
         ++keyIndex) {
        float nearestCameraDistance =
            std::numeric_limits<float>::max();
        for (std::uint32_t sampleIndex = 0U;
             sampleIndex <= 2000U;
             ++sampleIndex) {
            const auto sampled = invisible_places::camera::
                EvaluatePreparedAnimationPath(
                    originalContext,
                    originalContext.durationSeconds *
                        static_cast<float>(sampleIndex) / 2000.0F);
            nearestCameraDistance = std::min(
                nearestCameraDistance,
                std::hypot(
                    path.keys[keyIndex].cameraPosition[0U] -
                        sampled.camera.position[0U],
                    std::hypot(
                        path.keys[keyIndex].cameraPosition[1U] -
                            sampled.camera.position[1U],
                        path.keys[keyIndex].cameraPosition[2U] -
                            sampled.camera.position[2U])));
        }
        CHECK(nearestCameraDistance < 0.01F);
    }

    auto combinedPath = original;
    const auto combined = invisible_places::camera::
        OptimizeAnimationCameraKeysForSmoothRotation(
            &combinedPath,
            {
                .equalizeScreenXVelocity = true,
                .sampleCount = 97U,
                .optimizationSweeps = 14U,
            });
    CAPTURE(
        combined.beforeRotationRmsDegreesPerSecond,
        combined.afterRotationRmsDegreesPerSecond,
        combined.beforeXVelocityDeviation,
        combined.afterXVelocityDeviation,
        combined.beforeRotationDirectionChanges,
        combined.afterRotationDirectionChanges);
    REQUIRE(combined.succeeded);
    REQUIRE(combined.changed);
    CHECK(combined.afterXVelocityDeviation <
          combined.beforeXVelocityDeviation);
    CHECK(combined.afterRotationRmsDegreesPerSecond <
          combined.beforeRotationRmsDegreesPerSecond);
    CHECK(combined.afterRotationDirectionChanges <
          combined.beforeRotationDirectionChanges);
    for (std::size_t index = 0U;
         index < combinedPath.keys.size();
         ++index) {
        CHECK(combinedPath.keys[index].focusPoint ==
              original.keys[index].focusPoint);
        CHECK(combinedPath.keys[index].durationFrames ==
              original.keys[index].durationFrames);
    }
}

TEST_CASE("Animation path perceived flow is zero for static paths and retimes evenly", "[camera][animation]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 90;
    path.keys = {
        {.cameraPosition = {1.0F, 2.0F, 0.5F}, .focusPoint = {1.0F, 3.0F, 0.5F}, .durationFrames = 45},
        {.cameraPosition = {1.0F, 2.0F, 0.5F}, .focusPoint = {1.0F, 3.0F, 0.5F}, .durationFrames = 45},
        {.cameraPosition = {1.0F, 2.0F, 0.5F}, .focusPoint = {1.0F, 3.0F, 0.5F}, .durationFrames = 45},
    };

    const auto preparedPath = invisible_places::camera::PrepareAnimationPathEvaluation(path);
    REQUIRE(preparedPath.valid);
    const auto flowSamples =
        invisible_places::camera::MeasurePreparedAnimationPathPerceivedFlow(preparedPath);
    REQUIRE_FALSE(flowSamples.empty());
    for (const auto& sample : flowSamples) {
        CHECK(sample.screenSpeed == Catch::Approx(0.0F).margin(1.0e-3F));
    }

    const auto segmentFrames =
        invisible_places::camera::ComputeConstantPerceivedSpeedSegmentFrames(path);
    REQUIRE(segmentFrames.size() == 2U);
    CHECK(segmentFrames[0] == 45U);
    CHECK(segmentFrames[1] == 45U);
}

TEST_CASE("Loop endpoint corrections leave the complete middle interval unchanged", "[camera][animation][loop]") {
    using invisible_places::camera::AnimationLocalizedKeyCorrection;
    using invisible_places::camera::AnimationPath;
    AnimationPath original;
    original.durationFrames = 150U;
    original.keys = {
        {.id = "a", .cameraPosition = {0.0F, 4.0F, 1.0F}, .focusPoint = {3.0F, 4.0F, 1.0F}, .durationFrames = 30U},
        {.id = "b", .cameraPosition = {0.2F, 3.0F, 1.0F}, .focusPoint = {3.2F, 3.0F, 1.0F}, .durationFrames = 31U},
        {.id = "c", .cameraPosition = {0.3F, 2.0F, 1.0F}, .focusPoint = {3.3F, 2.0F, 1.0F}, .durationFrames = 37U},
        {.id = "d", .cameraPosition = {0.4F, 1.0F, 1.0F}, .focusPoint = {3.4F, 1.0F, 1.0F}, .durationFrames = 39U},
        {.id = "e", .cameraPosition = {0.5F, 0.0F, 1.0F}, .focusPoint = {3.5F, 0.0F, 1.0F}, .durationFrames = 43U},
    };
    auto smoothed = original;
    smoothed.localizedKeyCorrections = {
        AnimationLocalizedKeyCorrection{
            .keyId = "a",
            .splineCameraPosition = original.keys.front().cameraPosition,
            .splineFocusPoint = original.keys.front().focusPoint,
        },
        AnimationLocalizedKeyCorrection{
            .keyId = "e",
            .splineCameraPosition = original.keys.back().cameraPosition,
            .splineFocusPoint = original.keys.back().focusPoint,
        },
    };
    smoothed.keys.front().cameraPosition[0] += 0.08F;
    smoothed.keys.front().focusPoint[1] -= 0.06F;
    smoothed.keys.back().cameraPosition[0] -= 0.07F;
    smoothed.keys.back().focusPoint[1] += 0.05F;

    const auto originalContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(original);
    const auto smoothedContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(smoothed);
    REQUIRE(originalContext.knots.size() == original.keys.size());
    REQUIRE(smoothedContext.knots == originalContext.knots);
    for (std::uint32_t sample = 0U; sample <= 64U; ++sample) {
        const float amount = static_cast<float>(sample) / 64.0F;
        const float time = std::lerp(
            originalContext.knots[1U],
            originalContext.knots[originalContext.knots.size() - 2U],
            amount);
        const auto before = invisible_places::camera::EvaluatePreparedAnimationPath(
            originalContext,
            time);
        const auto after = invisible_places::camera::EvaluatePreparedAnimationPath(
            smoothedContext,
            time);
        for (std::size_t component = 0U; component < 3U; ++component) {
            CHECK(after.camera.position[component] == Catch::Approx(before.camera.position[component]).margin(1.0e-6F));
            CHECK(after.focusPoint[component] == Catch::Approx(before.focusPoint[component]).margin(1.0e-6F));
        }
    }
    CHECK(smoothed.keys.front().durationFrames == original.keys.front().durationFrames);
    CHECK(smoothed.keys[1U].durationFrames == original.keys[1U].durationFrames);
    CHECK(smoothed.keys.back().durationFrames == original.keys.back().durationFrames);
}

TEST_CASE("Loop key corrections remain local to enabled-key neighborhoods",
          "[camera][animation][loop]") {
    using invisible_places::camera::AnimationLocalizedKeyCorrection;
    using invisible_places::camera::AnimationPath;
    AnimationPath original;
    original.durationFrames = 180U;
    for (std::size_t keyIndex = 0U; keyIndex < 7U; ++keyIndex) {
        original.keys.push_back({
            .id = "key-" + std::to_string(keyIndex + 1U),
            .cameraPosition = {
                static_cast<float>(keyIndex) * 0.2F,
                6.0F - static_cast<float>(keyIndex),
                1.0F},
            .focusPoint = {
                3.0F + static_cast<float>(keyIndex) * 0.2F,
                6.0F - static_cast<float>(keyIndex),
                1.0F},
            .durationFrames = 30U,
        });
    }
    auto adjusted = original;
    adjusted.localizedKeyCorrections = {
        AnimationLocalizedKeyCorrection{
            .keyId = original.keys[1U].id,
            .splineCameraPosition = original.keys[1U].cameraPosition,
            .splineFocusPoint = original.keys[1U].focusPoint,
        },
        AnimationLocalizedKeyCorrection{
            .keyId = original.keys[5U].id,
            .splineCameraPosition = original.keys[5U].cameraPosition,
            .splineFocusPoint = original.keys[5U].focusPoint,
        },
    };
    adjusted.keys[1U].cameraPosition[0U] += 0.08F;
    adjusted.keys[1U].focusPoint[1U] -= 0.05F;
    adjusted.keys[5U].cameraPosition[2U] += 0.07F;
    adjusted.keys[5U].focusPoint[0U] -= 0.04F;

    const auto beforeContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(original);
    const auto afterContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(adjusted);
    REQUIRE(beforeContext.knots.size() == original.keys.size());
    REQUIRE(afterContext.knots == beforeContext.knots);
    // Corrections touch only segments adjacent to keys 2 and 6. The dense
    // interval bounded by locked keys 3..5 is exactly the preserved spline.
    for (std::uint32_t sample = 0U; sample <= 96U; ++sample) {
        const float amount = static_cast<float>(sample) / 96.0F;
        const float time = std::lerp(
            beforeContext.knots[2U],
            beforeContext.knots[4U],
            amount);
        const auto before =
            invisible_places::camera::EvaluatePreparedAnimationPath(
                beforeContext,
                time);
        const auto after =
            invisible_places::camera::EvaluatePreparedAnimationPath(
                afterContext,
                time);
        for (std::size_t component = 0U; component < 3U; ++component) {
            CHECK(after.camera.position[component] ==
                  Catch::Approx(before.camera.position[component])
                      .margin(1.0e-6F));
            CHECK(after.focusPoint[component] ==
                  Catch::Approx(before.focusPoint[component])
                      .margin(1.0e-6F));
        }
    }
    CHECK(adjusted.keys[2U].cameraPosition ==
          original.keys[2U].cameraPosition);
    CHECK(adjusted.keys[4U].focusPoint ==
          original.keys[4U].focusPoint);
}

TEST_CASE("Loop smoothing supports explicit terminal keys with unequal path density",
          "[camera][animation][loop]") {
    using invisible_places::camera::AnimationLoopSmoothingOptions;
    using invisible_places::camera::AnimationPath;
    const auto makeKey = [](
                             std::string id,
                             float x,
                             float y,
                             std::uint32_t frames) {
        return invisible_places::camera::AnimationPathKey{
            .id = std::move(id),
            .cameraPosition = {x, y, 1.0F},
            .focusPoint = {x + 3.0F, y, 1.0F},
            .durationFrames = frames,
        };
    };
    AnimationPath first;
    first.name = "Sparse A";
    first.durationFrames = 150U;
    first.keys = {
        makeKey("a1", 0.0F, 4.0F, 30U),
        makeKey("a2", 0.18F, 3.0F, 31U),
        makeKey("a3", 0.28F, 2.0F, 37U),
        makeKey("a4", 0.38F, 1.0F, 39U),
        makeKey("a5", 0.48F, 0.0F, 43U),
    };
    AnimationPath second;
    second.name = "Dense B";
    second.durationFrames = 120U;
    for (std::size_t keyIndex = 0U; keyIndex < 7U; ++keyIndex) {
        const float amount = static_cast<float>(keyIndex) / 6.0F;
        second.keys.push_back(makeKey(
            "b" + std::to_string(keyIndex + 1U),
            0.52F + 0.70F * amount,
            4.0F * (1.0F - amount),
            20U));
    }
    const AnimationPath originalFirst = first;
    const AnimationPath originalSecond = second;
    const auto firstContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(first);
    REQUIRE(firstContext.knots.size() == first.keys.size());
    const float firstStartOverlap =
        firstContext.knots[1U] - firstContext.knots.front();
    const float firstEndOverlap =
        firstContext.knots.back() -
        firstContext.knots[firstContext.knots.size() - 2U];

    const AnimationLoopSmoothingOptions smoothingOptions{
                .maxEndMoveFraction = 0.10F,
                .pairId = "explicit-density-pair",
                .firstFileName = "Sparse_A.ipanim.json",
                .secondFileName = "Dense_B.ipanim.json",
                .useExplicitKeySelection = true,
                .firstMovableKeyIds = {"a1", "a2", "a4", "a5"},
                .secondMovableKeyIds = {"b1", "b2", "b6", "b7"},
                .firstStartOverlapSeconds = firstStartOverlap,
                .firstEndOverlapSeconds = firstEndOverlap,
                .secondStartOverlapSeconds = firstEndOverlap,
                .secondEndOverlapSeconds = firstStartOverlap,
                .horizontalBlend = true,
                .panRight = true,
            };
    const auto result =
        invisible_places::camera::SmoothAnimationLoopTransitions(
            &first,
            &second,
            smoothingOptions);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.afterMismatch < result.beforeMismatch);
    CHECK(result.afterSeamMismatch[0U] <= result.beforeSeamMismatch[0U]);
    CHECK(result.afterSeamMismatch[1U] <= result.beforeSeamMismatch[1U]);
    CHECK(result.terminalSpeedRmsChange[0U] < 0.30F);
    CHECK(result.terminalSpeedRmsChange[1U] < 0.30F);
    REQUIRE(first.localizedKeyCorrections.size() == 4U);
    REQUIRE(second.localizedKeyCorrections.size() == 4U);
    CHECK(first.localizedKeyCorrections[0U].keyId == "a1");
    CHECK(first.localizedKeyCorrections[3U].keyId == "a5");
    CHECK(second.localizedKeyCorrections[0U].keyId == "b1");
    CHECK(second.localizedKeyCorrections[3U].keyId == "b7");
    CHECK(result.keyMovements[0U].size() == 4U);
    CHECK(result.keyMovements[1U].size() == 4U);
    CHECK(result.screenDisplacementSamples[0U].size() == 16U);
    CHECK(result.screenDisplacementSamples[1U].size() == 13U);
    CHECK(result.maxScreenDisplacement[0U] >= 0.0F);
    CHECK(result.maxScreenDisplacement[1U] >= 0.0F);

    const auto appliedMetrics =
        invisible_places::camera::MeasureAnimationLoopTransitions(
            first,
            second,
            smoothingOptions);
    REQUIRE(appliedMetrics.valid);
    REQUIRE(appliedMetrics.hasAppliedSmoothing);
    CHECK(appliedMetrics.beforeMismatch ==
          Catch::Approx(result.beforeMismatch).epsilon(1.0e-5F));
    CHECK(appliedMetrics.afterMismatch ==
          Catch::Approx(result.afterMismatch).epsilon(1.0e-5F));

    CHECK(first.keys[2U].cameraPosition ==
          originalFirst.keys[2U].cameraPosition);
    CHECK(first.keys[2U].focusPoint ==
          originalFirst.keys[2U].focusPoint);
    for (std::size_t keyIndex = 2U; keyIndex <= 4U; ++keyIndex) {
        CHECK(second.keys[keyIndex].cameraPosition ==
              originalSecond.keys[keyIndex].cameraPosition);
        CHECK(second.keys[keyIndex].focusPoint ==
              originalSecond.keys[keyIndex].focusPoint);
    }
    for (std::size_t keyIndex = 0U;
         keyIndex < first.keys.size();
         ++keyIndex) {
        CHECK(first.keys[keyIndex].durationFrames ==
              originalFirst.keys[keyIndex].durationFrames);
    }
    for (std::size_t keyIndex = 0U;
         keyIndex < second.keys.size();
         ++keyIndex) {
        CHECK(second.keys[keyIndex].durationFrames ==
              originalSecond.keys[keyIndex].durationFrames);
    }

    const auto afterFirstContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(first);
    const auto afterSecondContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(second);
    const auto firstMiddleBefore =
        invisible_places::camera::EvaluateAnimationPath(
            originalFirst,
            firstContext.knots[2U]);
    const auto firstMiddleAfter =
        invisible_places::camera::EvaluatePreparedAnimationPath(
            afterFirstContext,
            firstContext.knots[2U]);
    for (std::size_t component = 0U; component < 3U; ++component) {
        CHECK(firstMiddleAfter.camera.position[component] ==
              Catch::Approx(firstMiddleBefore.camera.position[component])
                  .margin(1.0e-6F));
        CHECK(firstMiddleAfter.focusPoint[component] ==
              Catch::Approx(firstMiddleBefore.focusPoint[component])
                  .margin(1.0e-6F));
    }
    const auto originalSecondContext =
        invisible_places::camera::PrepareAnimationPathEvaluation(
            originalSecond);
    for (std::uint32_t sample = 0U; sample <= 64U; ++sample) {
        const float amount = static_cast<float>(sample) / 64.0F;
        const float time = std::lerp(
            originalSecondContext.knots[2U],
            originalSecondContext.knots[4U],
            amount);
        const auto before =
            invisible_places::camera::EvaluatePreparedAnimationPath(
                originalSecondContext,
                time);
        const auto after =
            invisible_places::camera::EvaluatePreparedAnimationPath(
                afterSecondContext,
                time);
        for (std::size_t component = 0U; component < 3U; ++component) {
            CHECK(after.camera.position[component] ==
                  Catch::Approx(before.camera.position[component])
                      .margin(1.0e-6F));
            CHECK(after.focusPoint[component] ==
                  Catch::Approx(before.focusPoint[component])
                      .margin(1.0e-6F));
        }
    }

    const auto restoreLocalizedCorrections = [](AnimationPath* path) {
        REQUIRE(path != nullptr);
        for (const auto& correction : path->localizedKeyCorrections) {
            const auto key = std::find_if(
                path->keys.begin(),
                path->keys.end(),
                [&](const auto& candidate) {
                    return candidate.id == correction.keyId;
                });
            REQUIRE(key != path->keys.end());
            key->cameraPosition = correction.splineCameraPosition;
            key->focusPoint = correction.splineFocusPoint;
        }
        path->localizedKeyCorrections.clear();
    };
    restoreLocalizedCorrections(&first);
    restoreLocalizedCorrections(&second);
    CHECK(invisible_places::serialization::AnimationPathToJson(first) ==
          invisible_places::serialization::AnimationPathToJson(
              originalFirst));
    CHECK(invisible_places::serialization::AnimationPathToJson(second) ==
          invisible_places::serialization::AnimationPathToJson(
              originalSecond));
}

TEST_CASE("Horizontal loop blend regions follow the directional thirds wipe",
          "[camera][animation][loop][horizontal-blend]") {
    using invisible_places::camera::
        ResolveAnimationLoopHorizontalBlendRegions;

    const auto rightStart =
        ResolveAnimationLoopHorizontalBlendRegions(0.0F, true);
    CHECK(rightStart.outgoingVisibleFraction ==
          Catch::Approx(2.0F / 3.0F));
    CHECK(rightStart.incomingVisibleFraction ==
          Catch::Approx(1.0F / 3.0F));
    CHECK(rightStart.outgoingRange[0U] == Catch::Approx(-1.0F));
    CHECK(rightStart.outgoingRange[1U] == Catch::Approx(1.0F / 3.0F));
    CHECK(rightStart.incomingRange[0U] == Catch::Approx(1.0F / 3.0F));
    CHECK(rightStart.incomingRange[1U] == Catch::Approx(1.0F));

    const auto rightMiddle =
        ResolveAnimationLoopHorizontalBlendRegions(0.5F, true);
    CHECK(rightMiddle.outgoingVisibleFraction == Catch::Approx(0.5F));
    CHECK(rightMiddle.incomingVisibleFraction == Catch::Approx(0.5F));
    CHECK(rightMiddle.outgoingRange[0U] == Catch::Approx(-1.0F));
    CHECK(rightMiddle.outgoingRange[1U] == Catch::Approx(0.0F));
    CHECK(rightMiddle.incomingRange[0U] == Catch::Approx(0.0F));
    CHECK(rightMiddle.incomingRange[1U] == Catch::Approx(1.0F));

    const auto rightEnd =
        ResolveAnimationLoopHorizontalBlendRegions(1.0F, true);
    CHECK(rightEnd.outgoingVisibleFraction ==
          Catch::Approx(1.0F / 3.0F));
    CHECK(rightEnd.incomingVisibleFraction ==
          Catch::Approx(2.0F / 3.0F));
    CHECK(rightEnd.outgoingRange[1U] == Catch::Approx(-1.0F / 3.0F));
    CHECK(rightEnd.incomingRange[0U] == Catch::Approx(-1.0F / 3.0F));

    const auto leftStart =
        ResolveAnimationLoopHorizontalBlendRegions(0.0F, false);
    CHECK(leftStart.outgoingRange[0U] == Catch::Approx(-1.0F / 3.0F));
    CHECK(leftStart.outgoingRange[1U] == Catch::Approx(1.0F));
    CHECK(leftStart.incomingRange[0U] == Catch::Approx(-1.0F));
    CHECK(leftStart.incomingRange[1U] == Catch::Approx(-1.0F / 3.0F));
}

TEST_CASE(
    "Selected loop spatial objectives never retime or move locked keys",
    "[camera][animation][loop][spatial-objectives]") {
    using invisible_places::camera::AnimationLoopSmoothingOptions;
    using invisible_places::camera::AnimationLoopSpatialObjective;
    using invisible_places::camera::AnimationPath;
    using invisible_places::camera::AnimationSpeedEqualizationMode;
    const auto makePath = [](std::string prefix, float offset) {
        AnimationPath path;
        path.name = prefix;
        path.durationFrames = 120U;
        for (std::size_t index = 0U; index < 5U; ++index) {
            const float amount = static_cast<float>(index);
            path.keys.push_back({
                .id = prefix + "-" + std::to_string(index),
                .cameraPosition = {
                    offset + amount,
                    0.18F * amount * amount,
                    1.0F + 0.05F * amount,
                },
                .focusPoint = {
                    offset + amount + 4.0F,
                    0.1F * amount,
                    1.0F,
                },
                .durationFrames = index == 0U
                    ? 24U
                    : static_cast<std::uint32_t>(18U + 4U * index),
            });
        }
        return path;
    };
    const auto originalFirst = makePath("objective-A", 0.0F);
    const auto originalSecond = makePath("objective-B", 0.6F);
    const std::array objectives{
        AnimationLoopSpatialObjective::EqualizeScreenXVelocity,
        AnimationLoopSpatialObjective::EqualizeScreenYVelocity,
        AnimationLoopSpatialObjective::MinimizeImageRotation,
        AnimationLoopSpatialObjective::EqualizeScreenXAndRotation,
        AnimationLoopSpatialObjective::EqualizePerceivedSpeed,
    };
    const std::array speedModes{
        AnimationSpeedEqualizationMode::PerceivedMotion,
        AnimationSpeedEqualizationMode::CenterScreenPan,
        AnimationSpeedEqualizationMode::StabilizedPan,
    };
    for (const auto objective : objectives) {
        const std::size_t speedPasses = objective ==
                AnimationLoopSpatialObjective::EqualizePerceivedSpeed
            ? speedModes.size()
            : 1U;
        for (std::size_t speedIndex = 0U;
             speedIndex < speedPasses;
             ++speedIndex) {
            auto first = originalFirst;
            auto second = originalSecond;
            AnimationLoopSmoothingOptions options;
            options.useExplicitKeySelection = true;
            options.spatialObjective = objective;
            options.perceivedSpeedMode = speedModes[speedIndex];
            options.maxOptimizationSweeps = 2U;
            options.minimumStepFraction = 0.125F;
            options.firstStartOverlapSeconds = 1.0F;
            options.firstEndOverlapSeconds = 1.0F;
            options.secondStartOverlapSeconds = 1.0F;
            options.secondEndOverlapSeconds = 1.0F;
            options.selectedNeighborhoodSmoothnessWeight = 1.0F;
            for (const auto index : {0U, 1U, 3U, 4U}) {
                options.firstMovableKeyIds.push_back(first.keys[index].id);
                options.secondMovableKeyIds.push_back(second.keys[index].id);
            }
            const auto result = invisible_places::camera::
                SmoothAnimationLoopTransitions(&first, &second, options);
            INFO(result.errorMessage);
            CHECK(result.succeeded);
            CHECK(first.durationFrames == originalFirst.durationFrames);
            CHECK(second.durationFrames == originalSecond.durationFrames);
            for (std::size_t keyIndex = 0U;
                 keyIndex < first.keys.size();
                 ++keyIndex) {
                CHECK(first.keys[keyIndex].durationFrames ==
                      originalFirst.keys[keyIndex].durationFrames);
                CHECK(second.keys[keyIndex].durationFrames ==
                      originalSecond.keys[keyIndex].durationFrames);
            }
            CHECK(first.keys[2U].cameraPosition ==
                  originalFirst.keys[2U].cameraPosition);
            CHECK(first.keys[2U].focusPoint ==
                  originalFirst.keys[2U].focusPoint);
            CHECK(second.keys[2U].cameraPosition ==
                  originalSecond.keys[2U].cameraPosition);
            CHECK(second.keys[2U].focusPoint ==
                  originalSecond.keys[2U].focusPoint);
        }
    }
}

TEST_CASE("Loop transition smoothing improves both seams without retiming", "[camera][animation][loop]") {
    using invisible_places::camera::AnimationLoopSmoothingOptions;
    using invisible_places::camera::AnimationPath;
    const auto makeKey = [](const char* id, float x, float y, std::uint32_t frames) {
        return invisible_places::camera::AnimationPathKey{
            .id = id,
            .cameraPosition = {x, y, 1.0F},
            .focusPoint = {x + 3.0F, y, 1.0F},
            .durationFrames = frames,
        };
    };
    AnimationPath first;
    first.name = "First";
    first.durationFrames = 150U;
    first.keys = {
        makeKey("a1", 0.0F, 4.0F, 30U),
        makeKey("a2", 0.18F, 3.0F, 31U),
        makeKey("a3", 0.28F, 2.0F, 37U),
        makeKey("a4", 0.38F, 1.0F, 39U),
        makeKey("a5", 0.48F, 0.0F, 43U),
    };
    AnimationPath second;
    second.name = "Second";
    second.durationFrames = 150U;
    second.keys = {
        makeKey("b1", 0.52F, 4.0F, 30U),
        makeKey("b2", 0.92F, 3.0F, 31U),
        makeKey("b3", 1.02F, 2.0F, 37U),
        makeKey("b4", 1.12F, 1.0F, 39U),
        makeKey("b5", 1.22F, 0.0F, 43U),
    };
    const auto originalFirst = first;
    const auto originalSecond = second;
    const auto baselineMetrics =
        invisible_places::camera::MeasureAnimationLoopTransitions(
            first,
            second);
    REQUIRE(baselineMetrics.valid);
    CHECK_FALSE(baselineMetrics.hasAppliedSmoothing);
    CHECK(baselineMetrics.afterMismatch ==
          Catch::Approx(baselineMetrics.beforeMismatch));
    const auto result = invisible_places::camera::SmoothAnimationLoopTransitions(
        &first,
        &second,
        AnimationLoopSmoothingOptions{
            .maxEndMoveFraction = 0.10F,
            .pairId = "test-pair",
            .firstFileName = "First.ipanim.json",
            .secondFileName = "Second.ipanim.json",
        });
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    // The smoother writes only localized key corrections; the app installs
    // the pair's velocity-blend link when the user links the two animations.
    // Install it here the same way so the metric and round-trip sections
    // below exercise link-aware behavior.
    first.velocityBlendLink =
        invisible_places::camera::AnimationVelocityBlendLinkMetadata{
            .pairId = "test-pair",
            .partnerFileName = "Second.ipanim.json",
            .maxEndMoveFraction = 0.10F,
            .movableKeyIds = {"a1", "a5"},
        };
    second.velocityBlendLink =
        invisible_places::camera::AnimationVelocityBlendLinkMetadata{
            .pairId = "test-pair",
            .partnerFileName = "First.ipanim.json",
            .maxEndMoveFraction = 0.10F,
            .movableKeyIds = {"b1", "b5"},
        };
    CHECK(result.afterMismatch < result.beforeMismatch);
    CHECK(result.afterSeamMismatch[0U] < result.beforeSeamMismatch[0U]);
    CHECK(result.afterSeamMismatch[1U] < result.beforeSeamMismatch[1U]);
    CHECK(result.maxCameraCapUsage <= Catch::Approx(1.0F).margin(1.0e-4F));
    CHECK(result.maxFocusCapUsage <= Catch::Approx(1.0F).margin(1.0e-4F));
    CHECK(first.localizedKeyCorrections.size() == 2U);
    CHECK(second.localizedKeyCorrections.size() == 2U);
    CHECK(first.durationFrames == originalFirst.durationFrames);
    CHECK(second.durationFrames == originalSecond.durationFrames);
    for (std::size_t keyIndex = 0U; keyIndex < first.keys.size(); ++keyIndex) {
        CHECK(first.keys[keyIndex].durationFrames == originalFirst.keys[keyIndex].durationFrames);
        CHECK(second.keys[keyIndex].durationFrames == originalSecond.keys[keyIndex].durationFrames);
        if (keyIndex > 0U && keyIndex + 1U < first.keys.size()) {
            CHECK(first.keys[keyIndex].cameraPosition == originalFirst.keys[keyIndex].cameraPosition);
            CHECK(first.keys[keyIndex].focusPoint == originalFirst.keys[keyIndex].focusPoint);
            CHECK(second.keys[keyIndex].cameraPosition == originalSecond.keys[keyIndex].cameraPosition);
            CHECK(second.keys[keyIndex].focusPoint == originalSecond.keys[keyIndex].focusPoint);
        }
    }
    CHECK(result.maxCameraMove <= Catch::Approx(0.11F));
    CHECK(result.maxFocusMove <= Catch::Approx(0.11F));

    const auto weightedTerminalSpeedDeviation = [](
                                                      const AnimationPath& before,
                                                      const AnimationPath& after) {
        const auto beforeContext =
            invisible_places::camera::PrepareAnimationPathEvaluation(before);
        const auto afterContext =
            invisible_places::camera::PrepareAnimationPathEvaluation(after);
        const auto beforeFlow = invisible_places::camera::
            MeasurePreparedAnimationPathPerceivedFlow(beforeContext, 1025U);
        const auto afterFlow = invisible_places::camera::
            MeasurePreparedAnimationPathPerceivedFlow(afterContext, 1025U);
        const float firstEnd = beforeContext.knots[1U] /
                               beforeContext.durationSeconds;
        const float lastStart = beforeContext.knots[
                                    beforeContext.knots.size() - 2U] /
                                beforeContext.durationSeconds;
        float squaredDifference = 0.0F;
        float squaredOriginal = 0.0F;
        for (std::size_t index = 0U; index < beforeFlow.size(); ++index) {
            const float position = beforeFlow[index].normalizedPosition;
            float inward = 2.0F;
            if (position <= firstEnd) {
                inward = position / std::max(firstEnd, 1.0e-6F);
            } else if (position >= lastStart) {
                inward = (1.0F - position) /
                         std::max(1.0F - lastStart, 1.0e-6F);
            }
            if (inward > 1.0F) {
                continue;
            }
            const float weight = (1.0F - inward) * (1.0F - inward);
            const float difference =
                afterFlow[index].screenSpeed - beforeFlow[index].screenSpeed;
            squaredDifference += weight * difference * difference;
            squaredOriginal += weight * beforeFlow[index].screenSpeed *
                               beforeFlow[index].screenSpeed;
        }
        return std::sqrt(
            squaredDifference / std::max(squaredOriginal, 1.0e-8F));
    };
    CHECK(weightedTerminalSpeedDeviation(originalFirst, first) < 0.30F);
    CHECK(weightedTerminalSpeedDeviation(originalSecond, second) < 0.30F);

    const auto appliedMetrics =
        invisible_places::camera::MeasureAnimationLoopTransitions(
            first,
            second);
    REQUIRE(appliedMetrics.valid);
    REQUIRE(appliedMetrics.hasAppliedSmoothing);
    CHECK(appliedMetrics.beforeMismatch ==
          Catch::Approx(result.beforeMismatch).epsilon(1.0e-5F));
    CHECK(appliedMetrics.afterMismatch ==
          Catch::Approx(result.afterMismatch).epsilon(1.0e-5F));
    CHECK(appliedMetrics.afterSeamMismatch[0U] ==
          Catch::Approx(result.afterSeamMismatch[0U]).epsilon(1.0e-5F));
    CHECK(appliedMetrics.afterSeamMismatch[1U] ==
          Catch::Approx(result.afterSeamMismatch[1U]).epsilon(1.0e-5F));
    CHECK(appliedMetrics.terminalSpeedRmsChange[0U] ==
          Catch::Approx(result.terminalSpeedRmsChange[0U]).epsilon(1.0e-5F));
    CHECK(appliedMetrics.terminalSpeedRmsChange[1U] ==
          Catch::Approx(result.terminalSpeedRmsChange[1U]).epsilon(1.0e-5F));
    CHECK(appliedMetrics.maxCameraMove ==
          Catch::Approx(result.maxCameraMove).epsilon(1.0e-5F));
    CHECK(appliedMetrics.maxFocusMove ==
          Catch::Approx(result.maxFocusMove).epsilon(1.0e-5F));

    std::string roundTripError;
    const auto roundTrippedFirst = invisible_places::serialization::
        AnimationPathFromJson(
            invisible_places::serialization::AnimationPathToJson(first),
            &roundTripError);
    const auto roundTrippedSecond = invisible_places::serialization::
        AnimationPathFromJson(
            invisible_places::serialization::AnimationPathToJson(second),
            &roundTripError);
    REQUIRE(roundTrippedFirst.has_value());
    REQUIRE(roundTrippedSecond.has_value());
    const auto roundTrippedMetrics =
        invisible_places::camera::MeasureAnimationLoopTransitions(
            roundTrippedFirst.value(),
            roundTrippedSecond.value());
    REQUIRE(roundTrippedMetrics.valid);
    REQUIRE(roundTrippedMetrics.hasAppliedSmoothing);
    CHECK(roundTrippedMetrics.beforeMismatch ==
          Catch::Approx(appliedMetrics.beforeMismatch).epsilon(1.0e-5F));
    CHECK(roundTrippedMetrics.afterMismatch ==
          Catch::Approx(appliedMetrics.afterMismatch).epsilon(1.0e-5F));

    const auto restoreLocalizedCorrections = [](AnimationPath* path) {
        REQUIRE(path != nullptr);
        for (const auto& correction : path->localizedKeyCorrections) {
            const auto key = std::find_if(
                path->keys.begin(),
                path->keys.end(),
                [&](const auto& candidate) {
                    return candidate.id == correction.keyId;
                });
            REQUIRE(key != path->keys.end());
            key->cameraPosition = correction.splineCameraPosition;
            key->focusPoint = correction.splineFocusPoint;
        }
        path->localizedKeyCorrections.clear();
        path->velocityBlendLink.reset();
    };
    restoreLocalizedCorrections(&first);
    restoreLocalizedCorrections(&second);
    CHECK(first.keys.front().cameraPosition == originalFirst.keys.front().cameraPosition);
    CHECK(first.keys.back().focusPoint == originalFirst.keys.back().focusPoint);
    CHECK(second.keys.front().focusPoint == originalSecond.keys.front().focusPoint);
    CHECK(second.keys.back().cameraPosition == originalSecond.keys.back().cameraPosition);
    CHECK_FALSE(first.velocityBlendLink.has_value());
    CHECK_FALSE(second.velocityBlendLink.has_value());
    CHECK(
        invisible_places::serialization::AnimationPathToJson(first) ==
        invisible_places::serialization::AnimationPathToJson(originalFirst));
    CHECK(
        invisible_places::serialization::AnimationPathToJson(second) ==
        invisible_places::serialization::AnimationPathToJson(originalSecond));
}

TEST_CASE("Loop smoothing reports why an already matched pair is unchanged",
          "[camera][animation][loop]") {
    using invisible_places::camera::AnimationPath;
    AnimationPath first;
    first.name = "Already Matched A";
    first.durationFrames = 120U;
    first.keys = {
        {.id = "a1", .cameraPosition = {0.0F, 4.0F, 1.0F}, .focusPoint = {3.0F, 4.0F, 1.0F}},
        {.id = "a2", .cameraPosition = {0.1F, 3.0F, 1.0F}, .focusPoint = {3.1F, 3.0F, 1.0F}},
        {.id = "a3", .cameraPosition = {0.2F, 2.0F, 1.0F}, .focusPoint = {3.2F, 2.0F, 1.0F}},
        {.id = "a4", .cameraPosition = {0.3F, 1.0F, 1.0F}, .focusPoint = {3.3F, 1.0F, 1.0F}},
        {.id = "a5", .cameraPosition = {0.4F, 0.0F, 1.0F}, .focusPoint = {3.4F, 0.0F, 1.0F}},
    };
    AnimationPath second = first;
    second.name = "Already Matched B";
    for (std::size_t keyIndex = 0U; keyIndex < second.keys.size(); ++keyIndex) {
        second.keys[keyIndex].id = "b" + std::to_string(keyIndex + 1U);
    }
    const auto originalFirstJson =
        invisible_places::serialization::AnimationPathToJson(first);
    const auto originalSecondJson =
        invisible_places::serialization::AnimationPathToJson(second);

    const auto result =
        invisible_places::camera::SmoothAnimationLoopTransitions(
            &first,
            &second,
            {.maxEndMoveFraction = 0.10F});
    REQUIRE(result.succeeded);
    CHECK_FALSE(result.changed);
    CHECK_FALSE(result.errorMessage.empty());
    CHECK(first.localizedKeyCorrections.empty());
    CHECK(second.localizedKeyCorrections.empty());
    CHECK(invisible_places::serialization::AnimationPathToJson(first) ==
          originalFirstJson);
    CHECK(invisible_places::serialization::AnimationPathToJson(second) ==
          originalSecondJson);

    const auto metrics =
        invisible_places::camera::MeasureAnimationLoopTransitions(
            first,
            second);
    REQUIRE(metrics.valid);
    CHECK_FALSE(metrics.hasAppliedSmoothing);
    CHECK(metrics.afterMismatch == Catch::Approx(metrics.beforeMismatch));
}

TEST_CASE("Loop smoothing uses one bounded pose for shared endpoint cameras", "[camera][animation][loop]") {
    using invisible_places::camera::AnimationPath;
    const auto makeKey = [](const char* id, float x, float y) {
        return invisible_places::camera::AnimationPathKey{
            .id = id,
            .cameraPosition = {x, y, 1.0F},
            .focusPoint = {x + 3.0F, y, 1.0F},
            .durationFrames = 30U,
        };
    };
    AnimationPath first;
    first.name = "Shared First";
    first.durationFrames = 120U;
    first.keys = {
        makeKey("a1", 0.0F, 4.0F),
        makeKey("a2", 0.2F, 3.0F),
        makeKey("a3", 0.3F, 2.0F),
        makeKey("a4", 0.4F, 1.0F),
        makeKey("a5", 0.5F, 0.0F),
    };
    AnimationPath second;
    second.name = "Shared Second";
    second.durationFrames = 120U;
    second.keys = {
        makeKey("b1", 0.52F, 0.0F),
        makeKey("b2", 0.9F, 1.0F),
        makeKey("b3", 0.8F, 2.0F),
        makeKey("b4", 0.3F, 3.0F),
        makeKey("b5", 0.02F, 4.0F),
    };
    first.keys.front().linkedCameraId = "shared-endpoint";
    second.keys.back().linkedCameraId = "shared-endpoint";
    second.keys.back().focusPoint[1] += 0.06F;
    const auto originalFirst = first;
    const auto originalSecond = second;

    const auto distance = [](const auto& left, const auto& right) {
        const float x = right[0] - left[0];
        const float y = right[1] - left[1];
        const float z = right[2] - left[2];
        return std::sqrt((x * x) + (y * y) + (z * z));
    };
    const auto result = invisible_places::camera::SmoothAnimationLoopTransitions(
        &first,
        &second,
        {.maxEndMoveFraction = 0.10F});
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(first.keys.front().cameraPosition == second.keys.back().cameraPosition);
    CHECK(first.keys.front().focusPoint == second.keys.back().focusPoint);
    CHECK(
        distance(
            originalFirst.keys.front().cameraPosition,
            first.keys.front().cameraPosition) <=
        0.10F * distance(
                    originalFirst.keys.front().cameraPosition,
                    originalFirst.keys[1U].cameraPosition) +
            1.0e-5F);
    CHECK(
        distance(
            originalSecond.keys.back().focusPoint,
            second.keys.back().focusPoint) <=
        0.10F * distance(
                    originalSecond.keys.back().focusPoint,
                    originalSecond.keys[originalSecond.keys.size() - 2U]
                        .focusPoint) +
            1.0e-5F);

    REQUIRE_FALSE(first.localizedKeyCorrections.empty());
    REQUIRE_FALSE(second.localizedKeyCorrections.empty());
    CHECK(first.localizedKeyCorrections.front().splineCameraPosition ==
          originalFirst.keys.front().cameraPosition);
    CHECK(second.localizedKeyCorrections.back().splineFocusPoint ==
          originalSecond.keys.back().focusPoint);
}

TEST_CASE("Loop smoothing rejects a shared pose outside every movement cap", "[camera][animation][loop]") {
    invisible_places::camera::AnimationPath first;
    first.keys = {
        {.id = "a1", .cameraPosition = {0.0F, 0.0F, 0.0F}, .focusPoint = {0.0F, 1.0F, 0.0F}},
        {.id = "a2", .cameraPosition = {1.0F, 0.0F, 0.0F}, .focusPoint = {1.0F, 1.0F, 0.0F}},
        {.id = "a3", .cameraPosition = {2.0F, 0.0F, 0.0F}, .focusPoint = {2.0F, 1.0F, 0.0F}},
    };
    invisible_places::camera::AnimationPath second = first;
    second.keys.front().id = "b1";
    second.keys[1U].id = "b2";
    second.keys.back().id = "b3";
    first.keys.front().linkedCameraId = "shared";
    second.keys.back().linkedCameraId = "shared";
    second.keys.back().cameraPosition = {20.0F, 0.0F, 0.0F};
    second.keys.back().focusPoint = {20.0F, 1.0F, 0.0F};
    const auto originalFirst = first;
    const auto originalSecond = second;

    const auto result = invisible_places::camera::SmoothAnimationLoopTransitions(
        &first,
        &second,
        {.maxEndMoveFraction = 0.01F});
    CHECK_FALSE(result.succeeded);
    CHECK(first.localizedKeyCorrections.empty());
    CHECK(second.localizedKeyCorrections.empty());
    CHECK(first.keys.front().cameraPosition ==
          originalFirst.keys.front().cameraPosition);
    CHECK(second.keys.back().focusPoint ==
          originalSecond.keys.back().focusPoint);
}

TEST_CASE("Loop smoothing respects independently immovable camera and focus tracks", "[camera][animation][loop]") {
    using invisible_places::camera::AnimationPath;
    const auto makePair = []() {
        AnimationPath first;
        first.durationFrames = 120U;
        first.keys = {
            {.id = "a1", .cameraPosition = {0.0F, 4.0F, 1.0F}, .focusPoint = {3.0F, 4.0F, 1.0F}},
            {.id = "a2", .cameraPosition = {0.2F, 3.0F, 1.0F}, .focusPoint = {3.2F, 3.0F, 1.0F}},
            {.id = "a3", .cameraPosition = {0.3F, 2.0F, 1.0F}, .focusPoint = {3.3F, 2.0F, 1.0F}},
            {.id = "a4", .cameraPosition = {0.4F, 1.0F, 1.0F}, .focusPoint = {3.4F, 1.0F, 1.0F}},
            {.id = "a5", .cameraPosition = {0.5F, 0.0F, 1.0F}, .focusPoint = {3.5F, 0.0F, 1.0F}},
        };
        AnimationPath second;
        second.durationFrames = 120U;
        second.keys = {
            {.id = "b1", .cameraPosition = {0.52F, 4.0F, 1.0F}, .focusPoint = {3.52F, 4.0F, 1.0F}},
            {.id = "b2", .cameraPosition = {0.9F, 3.0F, 1.0F}, .focusPoint = {3.9F, 3.0F, 1.0F}},
            {.id = "b3", .cameraPosition = {1.0F, 2.0F, 1.0F}, .focusPoint = {4.0F, 2.0F, 1.0F}},
            {.id = "b4", .cameraPosition = {1.1F, 1.0F, 1.0F}, .focusPoint = {4.1F, 1.0F, 1.0F}},
            {.id = "b5", .cameraPosition = {1.2F, 0.0F, 1.0F}, .focusPoint = {4.2F, 0.0F, 1.0F}},
        };
        return std::pair{first, second};
    };

    SECTION("zero-length focus end segments remain fixed") {
        auto [first, second] = makePair();
        for (auto* path : {&first, &second}) {
            path->keys.front().focusPoint = {3.0F, 2.0F, 1.0F};
            path->keys[1U].focusPoint = path->keys.front().focusPoint;
            path->keys[path->keys.size() - 2U].focusPoint = {3.0F, 2.0F, 1.0F};
            path->keys.back().focusPoint = path->keys[path->keys.size() - 2U].focusPoint;
        }
        const auto originalFirst = first;
        const auto originalSecond = second;
        const auto result = invisible_places::camera::SmoothAnimationLoopTransitions(
            &first,
            &second,
            {.maxEndMoveFraction = 0.10F});
        REQUIRE(result.succeeded);
        REQUIRE(result.changed);
        CHECK(result.maxFocusMove == Catch::Approx(0.0F).margin(1.0e-7F));
        CHECK(first.keys.front().focusPoint ==
              originalFirst.keys.front().focusPoint);
        CHECK(second.keys.back().focusPoint ==
              originalSecond.keys.back().focusPoint);
    }

    SECTION("zero-length camera end segments remain fixed") {
        auto [first, second] = makePair();
        for (auto* path : {&first, &second}) {
            path->keys.front().cameraPosition = {0.0F, 2.0F, 1.0F};
            path->keys[1U].cameraPosition = path->keys.front().cameraPosition;
            path->keys[path->keys.size() - 2U].cameraPosition = {0.0F, 2.0F, 1.0F};
            path->keys.back().cameraPosition = path->keys[path->keys.size() - 2U].cameraPosition;
        }
        const auto originalFirst = first;
        const auto originalSecond = second;
        const auto result = invisible_places::camera::SmoothAnimationLoopTransitions(
            &first,
            &second,
            {.maxEndMoveFraction = 0.10F});
        REQUIRE(result.succeeded);
        REQUIRE(result.changed);
        CHECK(result.maxCameraMove == Catch::Approx(0.0F).margin(1.0e-7F));
        CHECK(first.keys.front().cameraPosition ==
              originalFirst.keys.front().cameraPosition);
        CHECK(second.keys.back().cameraPosition ==
              originalSecond.keys.back().cameraPosition);
    }
}

TEST_CASE("Lower-frame geometry alignment improves cached common foreground within its key cap",
          "[camera][animation][strong-alignment]") {
    using invisible_places::camera::AnimationPath;
    const auto makeKey = [](
                             std::string id,
                             std::array<float, 3> camera,
                             std::array<float, 3> focus) {
        return invisible_places::camera::AnimationPathKey{
            .id = std::move(id),
            .cameraPosition = camera,
            .focusPoint = focus,
            .fovDegrees = 58.0F,
            .nearPlane = 0.01F,
            .farPlane = 100.0F,
            .durationFrames = 30U,
        };
    };

    AnimationPath reference;
    reference.durationFrames = 90U;
    reference.keys = {
        makeKey("r1", {-2.0F, -10.0F, 5.0F}, {-2.0F, 1.0F, 0.0F}),
        makeKey("r2", {0.0F, -10.0F, 5.0F}, {0.0F, 1.0F, 0.0F}),
        makeKey("r3", {2.0F, -10.0F, 5.0F}, {2.0F, 1.0F, 0.0F}),
    };
    AnimationPath destination;
    destination.durationFrames = 90U;
    destination.keys = {
        makeKey("d1", {-3.5F, -10.0F, 5.0F}, {-3.5F, 1.0F, 0.0F}),
        makeKey("destination", {0.55F, -10.0F, 5.25F},
                {0.40F, 1.0F, 0.20F}),
        makeKey("d3", {4.5F, -10.0F, 5.0F}, {4.5F, 1.0F, 0.0F}),
    };
    const auto originalReference = reference;
    const auto originalDestination = destination;

    std::vector<invisible_places::io::Float3> residentPoints;
    for (std::uint32_t yIndex = 0U; yIndex <= 80U; ++yIndex) {
        const float y = -2.0F + 0.20F * static_cast<float>(yIndex);
        for (std::uint32_t xIndex = 0U; xIndex <= 100U; ++xIndex) {
            const float x = -10.0F + 0.20F * static_cast<float>(xIndex);
            residentPoints.push_back({x, y, 0.0F});
        }
    }

    const auto result = invisible_places::camera::
        StrongAlignAnimationKeyToReference(
            &destination,
            reference,
            residentPoints,
            {
                .destinationKeyId = "destination",
                .referenceNormalizedPosition = 0.5F,
                .aspectRatio = 16.0F / 9.0F,
                .maxMoveFraction = 0.50F,
            });
    INFO(result.errorMessage);
    INFO("foreground samples: " << result.metrics.foregroundSampleCount);
    INFO("destination coverage: " << result.metrics.destinationCoverage);
    INFO("reference coverage: " << result.metrics.referenceCoverage);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.metrics.foregroundSampleCount >= 24U);
    CHECK(result.metrics.destinationOccupiedCellCount >= 12U);
    CHECK(result.metrics.referenceOccupiedCellCount >= 12U);
    CHECK(result.metrics.destinationCoverage > 0.0F);
    CHECK(result.metrics.referenceCoverage > 0.0F);
    CHECK(result.metrics.afterForegroundReprojectionRms1080 <
          result.metrics.beforeForegroundReprojectionRms1080);
    CHECK(result.metrics.afterVerticalOffset1080 <=
          result.metrics.beforeVerticalOffset1080);
    CHECK(result.metrics.cameraCapUsage <=
          Catch::Approx(1.0F).margin(1.0e-5F));
    CHECK(result.metrics.focusCapUsage <=
          Catch::Approx(1.0F).margin(1.0e-5F));
    CHECK(reference.keys[1U].cameraPosition ==
          originalReference.keys[1U].cameraPosition);
    CHECK(reference.keys[1U].focusPoint ==
          originalReference.keys[1U].focusPoint);
    CHECK(destination.keys[0U].cameraPosition ==
          originalDestination.keys[0U].cameraPosition);
    CHECK(destination.keys[2U].focusPoint ==
          originalDestination.keys[2U].focusPoint);
    REQUIRE(destination.localizedKeyCorrections.size() == 1U);
    CHECK(destination.localizedKeyCorrections.front().keyId ==
          "destination");
    CHECK(destination.localizedKeyCorrections.front()
              .splineCameraPosition ==
          originalDestination.keys[1U].cameraPosition);
}

TEST_CASE("Cancelled animation alignment jobs leave their snapshots unchanged",
          "[camera][animation][velocity-blend][strong-alignment]") {
    using invisible_places::camera::AnimationPath;
    const auto makePath = [](std::string prefix, float offset) {
        AnimationPath path;
        path.durationFrames = 90U;
        for (std::size_t keyIndex = 0U; keyIndex < 3U; ++keyIndex) {
            const float x = offset + static_cast<float>(keyIndex);
            path.keys.push_back({
                .id = prefix + std::to_string(keyIndex + 1U),
                .cameraPosition = {x, -4.0F, 2.0F},
                .focusPoint = {x, 2.0F, 0.0F},
                .durationFrames = 30U,
            });
        }
        return path;
    };

    auto first = makePath("a", 0.0F);
    auto second = makePath("b", 4.0F);
    const auto originalFirst = first;
    const auto originalSecond = second;
    std::stop_source stopSource;
    stopSource.request_stop();
    const auto velocityResult = invisible_places::camera::
        SmoothAnimationLoopTransitions(
            &first,
            &second,
            {
                .maxEndMoveFraction = 0.10F,
                .stopToken = stopSource.get_token(),
            });
    CHECK_FALSE(velocityResult.succeeded);
    CHECK(velocityResult.errorMessage.find("cancelled") !=
          std::string::npos);
    CHECK(first.keys.front().cameraPosition ==
          originalFirst.keys.front().cameraPosition);
    CHECK(first.keys.back().focusPoint ==
          originalFirst.keys.back().focusPoint);
    CHECK(second.keys.front().cameraPosition ==
          originalSecond.keys.front().cameraPosition);
    CHECK(second.keys.back().focusPoint ==
          originalSecond.keys.back().focusPoint);
    CHECK(first.localizedKeyCorrections.empty());
    CHECK(second.localizedKeyCorrections.empty());

    auto destination = originalFirst;
    const auto strongResult = invisible_places::camera::
        StrongAlignAnimationKeyToReference(
            &destination,
            originalSecond,
            {},
            {
                .destinationKeyId = "a1",
                .stopToken = stopSource.get_token(),
            });
    CHECK_FALSE(strongResult.succeeded);
    CHECK(strongResult.errorMessage.find("cancelled") !=
          std::string::npos);
    CHECK(destination.keys.front().cameraPosition ==
          originalFirst.keys.front().cameraPosition);
    CHECK(destination.keys.front().focusPoint ==
          originalFirst.keys.front().focusPoint);
    CHECK(destination.localizedKeyCorrections.empty());
}

TEST_CASE("Exhibition velocity pair fixture loads as the 45 second right pan",
          "[camera][animation][velocity-blend][migration][exhibition]") {
    // Schema-pinned copies of the authored Exhibition pair. Never read the
    // live Saved/ authoring output here: its content drifts with authoring.
    const auto fixtureDirectory =
        DataRoot().parent_path() / "tests" / "fixtures";
    const auto firstPath =
        fixtureDirectory / "Exhibition_FIRST.ipanim.json";
    const auto secondPath =
        fixtureDirectory / "Exhibition_SECOND.ipanim.json";
    if (!std::filesystem::is_regular_file(firstPath) ||
        !std::filesystem::is_regular_file(secondPath)) {
        SKIP("The pinned Exhibition pair fixtures are not present.");
    }

    std::string errorMessage;
    const auto first = invisible_places::serialization::LoadAnimationPath(
        firstPath,
        &errorMessage);
    INFO(errorMessage);
    REQUIRE(first.has_value());
    const auto second = invisible_places::serialization::LoadAnimationPath(
        secondPath,
        &errorMessage);
    INFO(errorMessage);
    REQUIRE(second.has_value());
    REQUIRE(first->velocityBlendLink.has_value());
    REQUIRE(second->velocityBlendLink.has_value());
    CHECK(first->velocityBlendLink->pairId ==
          second->velocityBlendLink->pairId);
    CHECK(first->velocityBlendLink->partnerFileName ==
          secondPath.filename().string());
    CHECK(second->velocityBlendLink->partnerFileName ==
          firstPath.filename().string());
    for (const auto* path : {&first.value(), &second.value()}) {
        CHECK(path->durationFrames == 5400U);
        REQUIRE(path->keys.size() == 5U);
        CHECK(path->velocityBlendLink->startOverlapSeconds ==
              Catch::Approx(45.0F));
        CHECK(path->velocityBlendLink->endOverlapSeconds ==
              Catch::Approx(45.0F));
        CHECK(path->velocityBlendLink->horizontalBlend);
        CHECK(path->velocityBlendLink->panRight);
        CHECK(path->velocityBlendLink->movableKeyIds ==
              std::vector<std::string>{
                  "key_1", "key_2", "key_4", "key_5"});
        REQUIRE(path->localizedKeyCorrections.size() == 4U);
    }

    const auto metrics = invisible_places::camera::
        MeasureAnimationLoopTransitions(first.value(), second.value());
    INFO(metrics.errorMessage);
    INFO("Exhibition mismatch " << metrics.beforeMismatch << " -> "
                                 << metrics.afterMismatch);
    REQUIRE(metrics.valid);
    REQUIRE(metrics.hasAppliedSmoothing);
    CHECK(metrics.afterMismatch < metrics.beforeMismatch);
    CHECK(metrics.afterSeamMismatch[0U] <=
          metrics.beforeSeamMismatch[0U] + 1.0e-5F);
    CHECK(metrics.afterSeamMismatch[1U] <=
          metrics.beforeSeamMismatch[1U] + 1.0e-5F);
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
        CHECK(
            savedJson.find(
                "\"schema_version\": " +
                std::to_string(
                    invisible_places::serialization::kAnimationDocumentSchemaVersion)) !=
            std::string::npos);
        CHECK(savedJson.find("\"associated_layer_paths\"") != std::string::npos);
        CHECK(savedJson.find("\"still_camera_duration_seconds\"") != std::string::npos);
        CHECK(savedJson.find("\"linked_camera_id\": \"camera_entry\"") != std::string::npos);
        CHECK(savedJson.find("\"water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_length_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"trail_sample_spacing_meters\"") != std::string::npos);
        CHECK(savedJson.find("\"temp_water_animation_trail_settings\"") != std::string::npos);
        CHECK(savedJson.find("\"water_point_visual_style\"") == std::string::npos);
        CHECK(savedJson.find("\"temp_water_point_visual_style\"") == std::string::npos);
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
    CHECK(
        invisible_places::output::PngStackFramePath(settings.outputDirectory, "Test Animation", 41).generic_string() ==
        "Saved/renders/Test/Test_Animation_0042.png");
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
    CHECK(invisible_places::output::AnimationRenderSequenceFrameCount(path, settings) == frames.size());

    const auto middleEvaluation = invisible_places::camera::EvaluateAnimationPath(path, 1.0F);
    CHECK(frames[30].position[0] == Catch::Approx(middleEvaluation.camera.position[0]));
    CHECK(frames[30].position[1] == Catch::Approx(middleEvaluation.camera.position[1]));
    CHECK(frames[30].target[0] == Catch::Approx(middleEvaluation.camera.target[0]));
    CHECK(frames.back().position[0] == Catch::Approx(path.keys.back().cameraPosition[0]));
    CHECK(frames.back().position[1] == Catch::Approx(path.keys.back().cameraPosition[1]));

    settings.framesPerSecond = 60;
    const auto sixtyFpsFrames = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(sixtyFpsFrames.size() == 121);
    CHECK(invisible_places::output::AnimationRenderSequenceFrameCount(path, settings) == sixtyFpsFrames.size());
    CHECK(sixtyFpsFrames[60].position[0] == Catch::Approx(middleEvaluation.camera.position[0]));
    CHECK(sixtyFpsFrames[60].position[1] == Catch::Approx(middleEvaluation.camera.position[1]));

    settings.framesPerSecond = 24;
    path.durationFrames = 90;
    const auto seventyTwoFrameDuration = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(seventyTwoFrameDuration.size() == 73);
    CHECK(
        invisible_places::output::AnimationRenderSequenceFrameCount(path, settings) ==
        seventyTwoFrameDuration.size());
    path.durationFrames = 45;
    const auto thirtySixFrameDuration = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(thirtySixFrameDuration.size() == 37);
    CHECK(
        invisible_places::output::AnimationRenderSequenceFrameCount(path, settings) ==
        thirtySixFrameDuration.size());

    settings.framesPerSecond = 30;
    path.durationFrames = 60;
    settings.startFrame = 10;
    settings.endFrame = 12;
    const auto rangedFrames = invisible_places::output::BuildAnimationRenderSequence(path, settings);
    REQUIRE(rangedFrames.size() == 3);
    CHECK(invisible_places::output::AnimationRenderSequenceFrameCount(path, settings) == rangedFrames.size());
    CHECK(rangedFrames.front().position[0] == Catch::Approx(frames[10].position[0]));
    CHECK(rangedFrames.back().position[0] == Catch::Approx(frames[12].position[0]));

    invisible_places::camera::AnimationPath singleKeyPath;
    singleKeyPath.durationFrames = 1;
    singleKeyPath.keys = {
        {.cameraPosition = {3.0F, 4.0F, 5.0F}, .focusPoint = {3.0F, 4.0F, 2.0F}},
    };
    settings.startFrame = 0;
    settings.endFrame = 0;
    settings.framesPerSecond = 30;
    const auto singleKeyFrames =
        invisible_places::output::BuildAnimationRenderSequence(singleKeyPath, settings);
    REQUIRE(singleKeyFrames.size() == 2);
    CHECK(
        invisible_places::output::AnimationRenderSequenceFrameCount(singleKeyPath, settings) ==
        singleKeyFrames.size());
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

TEST_CASE("Preview-density export point-size scale follows output viewport ratio", "[output][animation]") {
    CHECK(invisible_places::output::ComputePointSizePixelScale(1920, 1080, 1920, 1080) == Catch::Approx(1.0F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(3840, 2160, 1920, 1080) == Catch::Approx(2.0F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(960, 540, 1920, 1080) == Catch::Approx(0.5F));
    CHECK(invisible_places::output::ComputePointSizePixelScale(3840, 1080, 1920, 1080) == Catch::Approx(std::sqrt(2.0F)));
    CHECK(invisible_places::output::ComputePointSizePixelScale(1920, 1080, 0, 1080) == Catch::Approx(1.0F));
}

TEST_CASE("Export frame sample plans are centered for temporal and motion blur", "[output][animation]") {
    invisible_places::output::RenderJobSettings settings;
    settings.temporalSupersampling = true;
    settings.temporalSampleCount = 4;

    const auto temporalOffsets = invisible_places::output::BuildExportFrameSampleOffsetsFrames(settings);
    REQUIRE(temporalOffsets.size() == 4U);
    CHECK(temporalOffsets[0] == Catch::Approx(-0.375F));
    CHECK(temporalOffsets[1] == Catch::Approx(-0.125F));
    CHECK(temporalOffsets[2] == Catch::Approx(0.125F));
    CHECK(temporalOffsets[3] == Catch::Approx(0.375F));

    settings.temporalSupersampling = false;
    settings.motionBlur = true;
    settings.motionBlurSampleCount = 2;
    settings.motionBlurShutterAngleDegrees = 180.0F;
    const auto motionOffsets = invisible_places::output::BuildExportFrameSampleOffsetsFrames(settings);
    REQUIRE(motionOffsets.size() == 2U);
    CHECK(motionOffsets[0] == Catch::Approx(-0.125F));
    CHECK(motionOffsets[1] == Catch::Approx(0.125F));
}

TEST_CASE("Point-cloud EXR readback masks keep AOV channels independent", "[renderer][export]") {
    using invisible_places::renderer::core::ContainsPointCloudExrReadbacks;
    using invisible_places::renderer::core::HasPointCloudExrReadback;
    using invisible_places::renderer::core::PointCloudExrFrameRequest;
    using invisible_places::renderer::core::PointCloudExrReadbackMask;

    const auto videoMask = PointCloudExrReadbackMask::Color | PointCloudExrReadbackMask::Depth;
    CHECK(HasPointCloudExrReadback(videoMask, PointCloudExrReadbackMask::Color));
    CHECK(HasPointCloudExrReadback(videoMask, PointCloudExrReadbackMask::Depth));
    CHECK_FALSE(HasPointCloudExrReadback(videoMask, PointCloudExrReadbackMask::Normal));
    CHECK_FALSE(HasPointCloudExrReadback(videoMask, PointCloudExrReadbackMask::Albedo));

    const auto proResMask = PointCloudExrReadbackMask::Color;
    CHECK(HasPointCloudExrReadback(proResMask, PointCloudExrReadbackMask::Color));
    CHECK_FALSE(HasPointCloudExrReadback(proResMask, PointCloudExrReadbackMask::Depth));
    CHECK(ContainsPointCloudExrReadbacks(
        PointCloudExrReadbackMask::All,
        proResMask));
    CHECK_FALSE(ContainsPointCloudExrReadbacks(
        proResMask,
        PointCloudExrReadbackMask::All));

    const PointCloudExrFrameRequest defaultRequest;
    CHECK(HasPointCloudExrReadback(defaultRequest.readbackMask, PointCloudExrReadbackMask::Color));
    CHECK(HasPointCloudExrReadback(defaultRequest.readbackMask, PointCloudExrReadbackMask::Depth));
    CHECK(HasPointCloudExrReadback(defaultRequest.readbackMask, PointCloudExrReadbackMask::Normal));
    CHECK(HasPointCloudExrReadback(defaultRequest.readbackMask, PointCloudExrReadbackMask::Albedo));
}

TEST_CASE("Built-in export presets use compact codec settings with legacy aliases", "[output][video]") {
    const auto presets = invisible_places::output::BuiltInExportPresets();
    const auto findPreset = [&presets](std::string_view name) {
        return std::find_if(presets.begin(), presets.end(), [&](const auto& preset) {
            return preset.name == name;
        });
    };
    const auto containsPreset = [&presets](std::string_view name, invisible_places::output::AnimationExportMode mode) {
        return std::any_of(presets.begin(), presets.end(), [&](const auto& preset) {
            return preset.name == name && preset.mode == mode;
        });
    };

    REQUIRE(presets.size() == 7U);
    CHECK(containsPreset(
        invisible_places::output::kMp4PresetName,
        invisible_places::output::AnimationExportMode::FastPreviewMp4));
    CHECK(containsPreset(
        invisible_places::output::kTestMp4PresetName,
        invisible_places::output::AnimationExportMode::TestMp4));
    CHECK(containsPreset(
        invisible_places::output::kPngStackPresetName,
        invisible_places::output::AnimationExportMode::PngStack));
    CHECK(containsPreset(
        invisible_places::output::kFastPngStackPresetName,
        invisible_places::output::AnimationExportMode::FastPngStack));
    CHECK(containsPreset(
        invisible_places::output::kProRes422PresetName,
        invisible_places::output::AnimationExportMode::ProRes422Mov));
    CHECK(containsPreset(
        invisible_places::output::kProRes4444PresetName,
        invisible_places::output::AnimationExportMode::ProRes4444Mov));
    CHECK(containsPreset(
        invisible_places::output::kHqPreviewDensityExrPresetName,
        invisible_places::output::AnimationExportMode::HqPreviewDensityExr));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kMp4PresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kTestMp4PresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kHevcAlphaMp4PresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kPngStackPresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kFastPngStackPresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kProRes422PresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kProRes422HqPresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(
        invisible_places::output::kProRes422AlphaMattePresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(
        invisible_places::output::kProRes422HqAlphaMattePresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(
        invisible_places::output::kProRes422VideoToolboxPresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(
        invisible_places::output::kProRes422HqVideoToolboxPresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(invisible_places::output::kProRes4444XqPresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(
        invisible_places::output::kProRes4444VideoToolboxPresetName));
    CHECK(invisible_places::output::IsBuiltInExportPresetName(
        invisible_places::output::kProRes4444XqVideoToolboxPresetName));
    const auto mp4 = findPreset(invisible_places::output::kMp4PresetName);
    REQUIRE(mp4 != presets.end());
    CHECK(mp4->settings.width == 3840U);
    CHECK(mp4->settings.height == 2160U);
    CHECK(mp4->settings.supersampleScale == 2U);
    CHECK(mp4->quality == invisible_places::output::AnimationExportQuality::Normal);
    CHECK(mp4->useVideoToolbox);
    CHECK(mp4->externalAlphaMatte);
    const auto testMp4 = findPreset(invisible_places::output::kTestMp4PresetName);
    REQUIRE(testMp4 != presets.end());
    CHECK(testMp4->mode == invisible_places::output::AnimationExportMode::TestMp4);
    CHECK(testMp4->settings.width == 3840U);
    CHECK(testMp4->settings.height == 2160U);
    CHECK(testMp4->settings.framesPerSecond == 5U);
    CHECK(testMp4->settings.supersampleScale == 2U);
    CHECK(testMp4->quality == invisible_places::output::AnimationExportQuality::Normal);
    CHECK(testMp4->useVideoToolbox);
    CHECK(testMp4->externalAlphaMatte);
    const auto pngStack = findPreset(invisible_places::output::kPngStackPresetName);
    REQUIRE(pngStack != presets.end());
    CHECK(pngStack->settings.width == 3840U);
    CHECK(pngStack->settings.height == 2160U);
    CHECK(pngStack->settings.supersampleScale == 2U);
    CHECK_FALSE(pngStack->externalAlphaMatte);
    const auto fastPngStack = findPreset(invisible_places::output::kFastPngStackPresetName);
    REQUIRE(fastPngStack != presets.end());
    CHECK(fastPngStack->settings.width == 3840U);
    CHECK(fastPngStack->settings.height == 2160U);
    CHECK(fastPngStack->settings.supersampleScale == 2U);
    const auto proRes422 = findPreset(invisible_places::output::kProRes422PresetName);
    REQUIRE(proRes422 != presets.end());
    CHECK(proRes422->settings.width == 3840U);
    CHECK(proRes422->settings.height == 2160U);
    CHECK(proRes422->quality == invisible_places::output::AnimationExportQuality::Hq);
    CHECK(proRes422->externalAlphaMatte);
    const auto proRes4444 = findPreset(invisible_places::output::kProRes4444PresetName);
    REQUIRE(proRes4444 != presets.end());
    CHECK(proRes4444->settings.width == 3840U);
    CHECK(proRes4444->settings.height == 2160U);
    CHECK(proRes4444->quality == invisible_places::output::AnimationExportQuality::Normal);
    CHECK(proRes4444->externalAlphaMatte);
}

TEST_CASE("Quick MP4 output paths use only animation names and collision suffixes", "[output][video]") {
    const auto outputDirectory = std::filesystem::temp_directory_path() / "invisible_places_quick_mp4_names";
    std::filesystem::remove_all(outputDirectory);
    std::filesystem::create_directories(outputDirectory);
    invisible_places::output::RenderJobSettings settings;
    settings.width = 3840;
    settings.height = 2160;
    settings.framesPerSecond = 24;
    settings.supersampleScale = 2;
    settings.temporalSupersampling = true;
    settings.temporalSampleCount = 4;
    settings.motionBlur = true;
    settings.motionBlurSampleCount = 3;
    settings.motionBlurShutterAngleDegrees = 90.0F;
    const auto stem = std::string{"Site_1"};
    const auto firstPath = outputDirectory / (stem + ".mp4");
    const auto secondPath = outputDirectory / (stem + "_1.mp4");
    const auto reservedPath = outputDirectory / (stem + "_2.mp4");

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
        settings,
        "Painty",
        {reservedPath});
    CHECK(uniquePath == outputDirectory / (stem + "_3.mp4"));

    const auto legacyPath = invisible_places::output::BuildUniqueQuickMp4OutputPath(
        outputDirectory,
        "Site 2",
        "Painty");
    CHECK(legacyPath == outputDirectory / "Site_2.mp4");

    std::filesystem::remove_all(outputDirectory);
}

TEST_CASE("Test MP4 names omit render format settings and visual details", "[output][video]") {
    invisible_places::output::RenderJobSettings settings;
    settings.width = 3840;
    settings.height = 2160;
    settings.framesPerSecond = 5;
    settings.supersampleScale = 2;

    const auto stem = invisible_places::output::BuildAnimationExportFilenameStem(
        "Site 1",
        invisible_places::output::AnimationExportMode::TestMp4,
        invisible_places::output::AnimationExportQuality::Normal,
        true,
        true,
        settings,
        "Painty");
    CHECK(stem == "Site_1_test");

    const auto finalStem = invisible_places::output::BuildAnimationExportFilenameStem(
        "Site 1",
        invisible_places::output::AnimationExportMode::ProRes422Mov,
        invisible_places::output::AnimationExportQuality::Normal,
        true,
        true,
        settings,
        "Painty");
    CHECK(finalStem == "Site_1");
}

TEST_CASE("Test MP4 ffmpeg command motion-interpolates low-rate RGB to one 30 fps H265 file", "[output][video]") {
    const auto command = invisible_places::output::BuildFfmpegTestMp4Command(
        "/opt/homebrew/bin/ffmpeg",
        3840,
        2160,
        5,
        51,
        "/tmp/Invisible Places/test output.mp4",
        true);

    CHECK(command.find("'/opt/homebrew/bin/ffmpeg'") != std::string::npos);
    CHECK(command.find("-f rawvideo") != std::string::npos);
    CHECK(command.find("-pix_fmt rgb48le") != std::string::npos);
    CHECK(command.find("-s:v 3840x2160") != std::string::npos);
    CHECK(command.find("-r 5") != std::string::npos);
    CHECK(command.find("tpad=stop_mode=clone:stop=2") != std::string::npos);
    CHECK(command.find("minterpolate=fps=30:mi_mode=mci:mc_mode=aobmc") != std::string::npos);
    CHECK(command.find(":me_mode=bilat:me=epzs:mb_size=8:") != std::string::npos);
    CHECK(command.find("vsbmc=1") != std::string::npos);
    CHECK(command.find("-c:v hevc_videotoolbox") != std::string::npos);
    CHECK(command.find("-b:v 25000k") != std::string::npos);
    CHECK(command.find("-maxrate 40000k") != std::string::npos);
    CHECK(command.find("-frames:v 306") != std::string::npos);
    CHECK(command.find("-fps_mode cfr") != std::string::npos);
    CHECK(command.find("-pix_fmt yuv420p") != std::string::npos);
    CHECK(command.find("alphaextract") == std::string::npos);
    CHECK(command.find("'/tmp/Invisible Places/test output.mp4'") != std::string::npos);
    CHECK(command.find("'/tmp/Invisible Places/test output.mp4'") ==
          command.rfind("'/tmp/Invisible Places/test output.mp4'"));

    const auto cpuCommand = invisible_places::output::BuildFfmpegTestMp4Command(
        "/opt/homebrew/bin/ffmpeg",
        1920,
        1080,
        5,
        1,
        "/tmp/test-cpu.mp4",
        false);
    CHECK(cpuCommand.find("-c:v libx265") != std::string::npos);
    CHECK(cpuCommand.find("-frames:v 6") != std::string::npos);
}

TEST_CASE("PNG Stack output directories use only animation names and collision suffixes", "[output][png]") {
    const auto outputDirectory = std::filesystem::temp_directory_path() / "invisible_places_png_stack_names";
    std::filesystem::remove_all(outputDirectory);
    std::filesystem::create_directories(outputDirectory);
    invisible_places::output::RenderJobSettings settings;
    settings.width = 3840;
    settings.height = 2160;
    settings.framesPerSecond = 24;
    settings.supersampleScale = 2;
    settings.temporalSupersampling = true;
    settings.temporalSampleCount = 4;
    settings.motionBlur = true;
    settings.motionBlurSampleCount = 3;
    settings.motionBlurShutterAngleDegrees = 90.0F;
    const auto stem = std::string{"Site_1"};
    const auto firstDirectory = outputDirectory / stem;
    const auto reservedDirectory = outputDirectory / (stem + "_1");
    std::filesystem::create_directories(firstDirectory);

    const auto uniqueDirectory = invisible_places::output::BuildUniquePngStackOutputDirectory(
        outputDirectory,
        "Site 1",
        settings,
        "Painty",
        {reservedDirectory});
    CHECK(uniqueDirectory == outputDirectory / (stem + "_2"));
    CHECK(
        invisible_places::output::PngStackFramePath(uniqueDirectory, "Site 1", 0) ==
        uniqueDirectory / "Site_1_0001.png");
    CHECK(
        invisible_places::output::PngStackFramePath(uniqueDirectory, "Site 1", 41) ==
        uniqueDirectory / "Site_1_0042.png");
    const auto fastStem = std::string{"Site_1_1"};
    const auto fastDirectory = invisible_places::output::BuildUniquePngStackOutputDirectory(
        outputDirectory,
        "Site 1",
        settings,
        "Painty",
        {},
        invisible_places::output::AnimationExportMode::FastPngStack);
    CHECK(fastDirectory == outputDirectory / fastStem);
    CHECK(
        invisible_places::output::PngStackFramePattern(fastDirectory, "Site 1") ==
        fastDirectory / "Site_1_%04d.png");

    const auto outputPattern = invisible_places::output::PngStackFramePattern(fastDirectory, "Site 1");
    const auto command = invisible_places::output::BuildFfmpegPngStackCommand(
        "/opt/homebrew/bin/ffmpeg",
        3840,
        2160,
        24,
        outputPattern);
    CHECK(command.find("'/opt/homebrew/bin/ffmpeg'") != std::string::npos);
    CHECK(command.find("-f rawvideo") != std::string::npos);
    CHECK(command.find("-pix_fmt rgba") != std::string::npos);
    CHECK(command.find("-s:v 3840x2160") != std::string::npos);
    CHECK(command.find("-r 24") != std::string::npos);
    CHECK(command.find("-i -") != std::string::npos);
    CHECK(command.find("-threads 0") != std::string::npos);
    CHECK(command.find("-c:v png") != std::string::npos);
    CHECK(command.find("-pred mixed") != std::string::npos);
    CHECK(command.find("-compression_level 3") != std::string::npos);
    CHECK(command.find("-start_number 1") != std::string::npos);
    CHECK(command.find("-f image2") != std::string::npos);
    CHECK(command.find("'" + outputPattern.string() + "'") != std::string::npos);

    std::filesystem::remove_all(outputDirectory);
}

TEST_CASE("Legacy HEVC alpha helpers map to MP4 HQ color plus matte pair", "[output][video]") {
    const auto outputDirectory = std::filesystem::temp_directory_path() / "invisible_places_hevc_alpha_names";
    std::filesystem::remove_all(outputDirectory);
    invisible_places::output::RenderJobSettings settings;
    settings.width = 3840;
    settings.height = 2160;
    settings.framesPerSecond = 24;
    settings.supersampleScale = 2;
    settings.temporalSupersampling = true;
    settings.temporalSampleCount = 4;
    settings.motionBlur = true;
    settings.motionBlurSampleCount = 3;
    settings.motionBlurShutterAngleDegrees = 90.0F;

    const auto path = invisible_places::output::BuildUniqueHevcAlphaMp4OutputPath(
        outputDirectory,
        "Site 1",
        settings,
        "Painty");
    CHECK(
        path ==
        outputDirectory / "Site_1_Colour.mp4");
    const auto outputPaths = invisible_places::output::BuildUniqueHevcAlphaMp4OutputPaths(
        outputDirectory,
        "Site 1",
        settings,
        "Painty");
    CHECK(outputPaths.colorPath == path);
    CHECK(
        outputPaths.alphaMattePath ==
        outputDirectory / "Site_1_Alpha.mp4");

    std::filesystem::create_directories(outputDirectory);
    {
        std::ofstream occupiedAlpha{outputPaths.alphaMattePath};
        occupiedAlpha << "occupied";
    }
    const auto indexedOutputPaths =
        invisible_places::output::BuildUniqueHevcAlphaMp4OutputPaths(
            outputDirectory,
            "Site 1",
            settings,
            "Painty");
    CHECK(indexedOutputPaths.colorPath == outputDirectory / "Site_1_Colour_2.mp4");
    CHECK(indexedOutputPaths.alphaMattePath == outputDirectory / "Site_1_Alpha_2.mp4");

    const auto colorCommand = invisible_places::output::BuildFfmpegHevcColorMp4Command(
        "/opt/homebrew/bin/ffmpeg",
        3840,
        2160,
        24,
        "/tmp/Invisible Places/final color.mp4");
    CHECK(colorCommand.find("'/opt/homebrew/bin/ffmpeg'") != std::string::npos);
    CHECK(colorCommand.find("-f rawvideo") != std::string::npos);
    CHECK(colorCommand.find("-pix_fmt rgba64le") != std::string::npos);
    CHECK(colorCommand.find("-s:v 3840x2160") != std::string::npos);
    CHECK(colorCommand.find("-r 24") != std::string::npos);
    CHECK(colorCommand.find("-i -") != std::string::npos);
    CHECK(colorCommand.find("-vf format=p210le") != std::string::npos);
    CHECK(colorCommand.find("-c:v hevc_videotoolbox") != std::string::npos);
    CHECK(colorCommand.find("-profile:v main42210") != std::string::npos);
    CHECK(colorCommand.find("-b:v 300000k") != std::string::npos);
    CHECK(colorCommand.find("-maxrate 450000k") != std::string::npos);
    CHECK(colorCommand.find("-tag:v hvc1") != std::string::npos);
    CHECK(colorCommand.find("-pix_fmt p210le") != std::string::npos);
    CHECK(colorCommand.find("-allow_sw 1") != std::string::npos);
    CHECK(colorCommand.find("-prio_speed 0") != std::string::npos);
    CHECK(colorCommand.find("-color_primaries bt709") != std::string::npos);
    CHECK(colorCommand.find("-color_trc iec61966-2-1") != std::string::npos);
    CHECK(colorCommand.find("-colorspace bt709") != std::string::npos);
    CHECK(colorCommand.find("'/tmp/Invisible Places/final color.mp4'") != std::string::npos);

    const auto matteCommand = invisible_places::output::BuildFfmpegHevcAlphaMatteMp4Command(
        "/opt/homebrew/bin/ffmpeg",
        3840,
        2160,
        24,
        "/tmp/Invisible Places/final alpha matte.mp4");
    CHECK(matteCommand.find("-f rawvideo") != std::string::npos);
    CHECK(matteCommand.find("-pix_fmt gray16le") != std::string::npos);
    CHECK(matteCommand.find("-vf scale=in_range=full:out_range=full,format=p210le") != std::string::npos);
    CHECK(matteCommand.find("-c:v hevc_videotoolbox") != std::string::npos);
    CHECK(matteCommand.find("-profile:v main42210") != std::string::npos);
    CHECK(matteCommand.find("-b:v 90000k") != std::string::npos);
    CHECK(matteCommand.find("-maxrate 135000k") != std::string::npos);
    CHECK(matteCommand.find("-pix_fmt p210le") != std::string::npos);
    CHECK(matteCommand.find("-prio_speed 0") != std::string::npos);
    CHECK(matteCommand.find("-color_range pc") != std::string::npos);
    CHECK(matteCommand.find("'/tmp/Invisible Places/final alpha matte.mp4'") != std::string::npos);

    const auto combinedCommand = invisible_places::output::BuildFfmpegMp4ColorAndAlphaMatteCommand(
        "/opt/homebrew/bin/ffmpeg",
        3840,
        2160,
        24,
        "/tmp/Invisible Places/final color.mp4",
        "/tmp/Invisible Places/final alpha matte.mp4",
        invisible_places::output::AnimationExportQuality::Hq,
        true);
    CHECK(combinedCommand.find("-pix_fmt rgba64le") != std::string::npos);
    CHECK(combinedCommand.find("-filter_complex") != std::string::npos);
    CHECK(combinedCommand.find("alphaextract") != std::string::npos);
    CHECK(combinedCommand.find("[color_out]") != std::string::npos);
    CHECK(combinedCommand.find("[alpha_out]") != std::string::npos);
    CHECK(combinedCommand.find("format=p210le") != std::string::npos);
    CHECK(combinedCommand.find("-profile:v main42210") != std::string::npos);
    CHECK(combinedCommand.find("-b:v 300000k") != std::string::npos);
    CHECK(combinedCommand.find("-b:v 90000k") != std::string::npos);
    CHECK(combinedCommand.find("-prio_speed 0") != std::string::npos);
    CHECK(combinedCommand.find("'/tmp/Invisible Places/final color.mp4'") != std::string::npos);
    CHECK(combinedCommand.find("'/tmp/Invisible Places/final alpha matte.mp4'") != std::string::npos);
    CHECK(combinedCommand.find("-i -") == combinedCommand.rfind("-i -"));

    std::filesystem::remove_all(outputDirectory);
}

TEST_CASE("ProRes 4444 output paths and ffmpeg command preserve alpha", "[output][video]") {
    const auto outputDirectory = std::filesystem::temp_directory_path() / "invisible_places_prores_names";
    std::filesystem::remove_all(outputDirectory);
    invisible_places::output::RenderJobSettings settings;
    settings.width = 4096;
    settings.height = 2160;
    settings.framesPerSecond = 30;
    settings.supersampleScale = 2;
    const auto proResPath = invisible_places::output::BuildUniqueProRes4444OutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Glow Pass");
    CHECK(
        proResPath ==
        outputDirectory / "Scene_Take.mov");
    const auto proResXqPath = invisible_places::output::BuildUniqueProRes4444XqOutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Glow Pass");
    CHECK(
        proResXqPath ==
        outputDirectory / "Scene_Take.mov");
    const auto proResVtPath = invisible_places::output::BuildUniqueProRes4444VideoToolboxOutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Glow Pass");
    CHECK(
        proResVtPath ==
        outputDirectory / "Scene_Take.mov");
    const auto proResXqVtPath = invisible_places::output::BuildUniqueProRes4444XqVideoToolboxOutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Glow Pass");
    CHECK(
        proResXqVtPath ==
        outputDirectory / "Scene_Take.mov");

    const auto command = invisible_places::output::BuildFfmpegProRes4444Command(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/final alpha.mov");
    CHECK(command.find("'/opt/homebrew/bin/ffmpeg'") != std::string::npos);
    CHECK(command.find("-f rawvideo") != std::string::npos);
    CHECK(command.find("-pix_fmt rgba64le") != std::string::npos);
    CHECK(command.find("-s:v 4096x2160") != std::string::npos);
    CHECK(command.find("-r 30") != std::string::npos);
    CHECK(command.find("-c:v prores_ks") != std::string::npos);
    CHECK(command.find("-profile:v 4") != std::string::npos);
    CHECK(command.find("-pix_fmt yuva444p10le") != std::string::npos);
    CHECK(command.find("-alpha_bits 16") != std::string::npos);
    CHECK(command.find("-color_primaries bt709") != std::string::npos);
    CHECK(command.find("-color_trc iec61966-2-1") != std::string::npos);
    CHECK(command.find("-colorspace bt709") != std::string::npos);
    CHECK(command.find("'/tmp/Invisible Places/final alpha.mov'") != std::string::npos);

    const auto xqCommand = invisible_places::output::BuildFfmpegProRes4444XqCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/final alpha xq.mov");
    CHECK(xqCommand.find("-c:v prores_ks") != std::string::npos);
    CHECK(xqCommand.find("-profile:v 5") != std::string::npos);
    CHECK(xqCommand.find("-pix_fmt yuva444p10le") != std::string::npos);
    CHECK(xqCommand.find("-alpha_bits 16") != std::string::npos);

    const auto videoToolboxCommand = invisible_places::output::BuildFfmpegProRes4444VideoToolboxCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/final alpha vt.mov");
    CHECK(videoToolboxCommand.find("-vf format=ayuv64le") != std::string::npos);
    CHECK(videoToolboxCommand.find("-c:v prores_videotoolbox") != std::string::npos);
    CHECK(videoToolboxCommand.find("-profile:v 4") != std::string::npos);
    CHECK(videoToolboxCommand.find("-pix_fmt ayuv64le") != std::string::npos);
    CHECK(videoToolboxCommand.find("-allow_sw 1") != std::string::npos);
    CHECK(videoToolboxCommand.find("-color_trc iec61966-2-1") != std::string::npos);

    const auto xqVideoToolboxCommand = invisible_places::output::BuildFfmpegProRes4444XqVideoToolboxCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/final alpha xq vt.mov");
    CHECK(xqVideoToolboxCommand.find("-c:v prores_videotoolbox") != std::string::npos);
    CHECK(xqVideoToolboxCommand.find("-profile:v 5") != std::string::npos);
    CHECK(xqVideoToolboxCommand.find("-pix_fmt ayuv64le") != std::string::npos);

    const auto combinedCommand = invisible_places::output::BuildFfmpegProRes4444ColorAndAlphaMatteCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/final alpha.mov",
        "/tmp/Invisible Places/final alpha matte.mov",
        invisible_places::output::AnimationExportQuality::Xq,
        false);
    CHECK(combinedCommand.find("-pix_fmt rgba64le") != std::string::npos);
    CHECK(combinedCommand.find("-filter_complex") != std::string::npos);
    CHECK(combinedCommand.find("alphaextract") != std::string::npos);
    CHECK(combinedCommand.find("format=yuva444p10le") != std::string::npos);
    CHECK(combinedCommand.find("format=yuv422p10le") != std::string::npos);
    CHECK(combinedCommand.find("-profile:v 5") != std::string::npos);
    CHECK(combinedCommand.find("-profile:v 3") != std::string::npos);
    CHECK(combinedCommand.find("-alpha_bits 16") != std::string::npos);
    CHECK(combinedCommand.find("'/tmp/Invisible Places/final alpha.mov'") != std::string::npos);
    CHECK(combinedCommand.find("'/tmp/Invisible Places/final alpha matte.mov'") != std::string::npos);
    CHECK(combinedCommand.find("-i -") == combinedCommand.rfind("-i -"));
}

TEST_CASE("ProRes 422 output paths and ffmpeg commands are opaque", "[output][video]") {
    const auto outputDirectory = std::filesystem::temp_directory_path() / "invisible_places_prores_422_names";
    std::filesystem::remove_all(outputDirectory);
    invisible_places::output::RenderJobSettings settings;
    settings.width = 4096;
    settings.height = 2160;
    settings.framesPerSecond = 30;
    settings.supersampleScale = 2;

    const auto proRes422Path = invisible_places::output::BuildUniqueProRes422OutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Base Layer");
    CHECK(
        proRes422Path ==
        outputDirectory / "Scene_Take.mov");
    const auto proRes422HqPath = invisible_places::output::BuildUniqueProRes422HqOutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Base Layer");
    CHECK(
        proRes422HqPath ==
        outputDirectory / "Scene_Take.mov");
    const auto proRes422AlphaPaths = invisible_places::output::BuildUniqueProRes422AlphaMatteOutputPaths(
        outputDirectory,
        "Scene Take",
        settings,
        "Base Layer");
    CHECK(
        proRes422AlphaPaths.colorPath ==
        outputDirectory / "Scene_Take_Colour.mov");
    CHECK(
        proRes422AlphaPaths.alphaMattePath ==
        outputDirectory / "Scene_Take_Alpha.mov");
    const auto proRes422HqAlphaPaths = invisible_places::output::BuildUniqueProRes422HqAlphaMatteOutputPaths(
        outputDirectory,
        "Scene Take",
        settings,
        "Base Layer",
        {proRes422AlphaPaths.colorPath, proRes422AlphaPaths.alphaMattePath});
    CHECK(
        proRes422HqAlphaPaths.colorPath ==
        outputDirectory / "Scene_Take_Colour_2.mov");
    CHECK(
        proRes422HqAlphaPaths.alphaMattePath ==
        outputDirectory / "Scene_Take_Alpha_2.mov");
    const auto proRes422VtPath = invisible_places::output::BuildUniqueProRes422VideoToolboxOutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Base Layer");
    CHECK(
        proRes422VtPath ==
        outputDirectory / "Scene_Take.mov");
    const auto proRes422HqVtPath = invisible_places::output::BuildUniqueProRes422HqVideoToolboxOutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Base Layer");
    CHECK(
        proRes422HqVtPath ==
        outputDirectory / "Scene_Take.mov");

    const auto reservedPath =
        outputDirectory / "Scene_Take_1.mov";
    std::filesystem::create_directories(outputDirectory);
    {
        std::ofstream existing{proRes422Path, std::ios::trunc};
        existing << "existing";
    }
    const auto collisionPath = invisible_places::output::BuildUniqueProRes422OutputPath(
        outputDirectory,
        "Scene Take",
        settings,
        "Base Layer",
        {reservedPath});
    CHECK(
        collisionPath ==
        outputDirectory / "Scene_Take_2.mov");
    std::filesystem::remove_all(outputDirectory);

    const auto command = invisible_places::output::BuildFfmpegProRes422Command(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/base layer.mov");
    CHECK(command.find("'/opt/homebrew/bin/ffmpeg'") != std::string::npos);
    CHECK(command.find("-f rawvideo") != std::string::npos);
    CHECK(command.find("-pix_fmt rgb48le") != std::string::npos);
    CHECK(command.find("-c:v prores_ks") != std::string::npos);
    CHECK(command.find("-profile:v 2") != std::string::npos);
    CHECK(command.find("-pix_fmt yuv422p10le") != std::string::npos);
    CHECK(command.find("-alpha_bits") == std::string::npos);
    CHECK(command.find("yuva444p10le") == std::string::npos);

    const auto hqCommand = invisible_places::output::BuildFfmpegProRes422HqCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/base layer hq.mov");
    CHECK(hqCommand.find("-c:v prores_ks") != std::string::npos);
    CHECK(hqCommand.find("-profile:v 3") != std::string::npos);
    CHECK(hqCommand.find("-pix_fmt yuv422p10le") != std::string::npos);
    CHECK(hqCommand.find("-alpha_bits") == std::string::npos);
    CHECK(hqCommand.find("yuva444p10le") == std::string::npos);

    const auto videoToolboxCommand = invisible_places::output::BuildFfmpegProRes422VideoToolboxCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/base layer vt.mov");
    CHECK(videoToolboxCommand.find("-pix_fmt rgb48le") != std::string::npos);
    CHECK(videoToolboxCommand.find("-vf format=p210le") != std::string::npos);
    CHECK(videoToolboxCommand.find("-c:v prores_videotoolbox") != std::string::npos);
    CHECK(videoToolboxCommand.find("-profile:v 2") != std::string::npos);
    CHECK(videoToolboxCommand.find("-pix_fmt p210le") != std::string::npos);
    CHECK(videoToolboxCommand.find("-allow_sw 1") != std::string::npos);
    CHECK(videoToolboxCommand.find("-alpha_bits") == std::string::npos);
    CHECK(videoToolboxCommand.find("ayuv64le") == std::string::npos);

    const auto hqVideoToolboxCommand = invisible_places::output::BuildFfmpegProRes422HqVideoToolboxCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/base layer hq vt.mov");
    CHECK(hqVideoToolboxCommand.find("-pix_fmt rgb48le") != std::string::npos);
    CHECK(hqVideoToolboxCommand.find("-vf format=p210le") != std::string::npos);
    CHECK(hqVideoToolboxCommand.find("-c:v prores_videotoolbox") != std::string::npos);
    CHECK(hqVideoToolboxCommand.find("-profile:v 3") != std::string::npos);
    CHECK(hqVideoToolboxCommand.find("-pix_fmt p210le") != std::string::npos);
    CHECK(hqVideoToolboxCommand.find("-allow_sw 1") != std::string::npos);
    CHECK(hqVideoToolboxCommand.find("-alpha_bits") == std::string::npos);
    CHECK(hqVideoToolboxCommand.find("ayuv64le") == std::string::npos);

    const auto combinedCommand = invisible_places::output::BuildFfmpegProRes422ColorAndAlphaMatteCommand(
        "/opt/homebrew/bin/ffmpeg",
        4096,
        2160,
        30,
        "/tmp/Invisible Places/base layer.mov",
        "/tmp/Invisible Places/base layer alpha.mov",
        invisible_places::output::AnimationExportQuality::Hq,
        true);
    CHECK(combinedCommand.find("-pix_fmt rgba64le") != std::string::npos);
    CHECK(combinedCommand.find("-filter_complex") != std::string::npos);
    CHECK(combinedCommand.find("alphaextract") != std::string::npos);
    CHECK(combinedCommand.find("format=p210le") != std::string::npos);
    CHECK(combinedCommand.find("-c:v prores_videotoolbox") != std::string::npos);
    CHECK(combinedCommand.find("-profile:v 3") != std::string::npos);
    CHECK(combinedCommand.find("-color_range pc") != std::string::npos);
    CHECK(combinedCommand.find("'/tmp/Invisible Places/base layer.mov'") != std::string::npos);
    CHECK(combinedCommand.find("'/tmp/Invisible Places/base layer alpha.mov'") != std::string::npos);
    CHECK(combinedCommand.find("-i -") == combinedCommand.rfind("-i -"));
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

TEST_CASE("ProRes conversion downsamples straight alpha without transparent RGB fringes", "[output][video]") {
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
        zero,
        zero,
        zero,
        one,
        zero,
        one,
        one,
        one,
        zero,
    };

    const auto bytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba16(image, 1, 1, true);
    REQUIRE(bytes.size() == 8U);
    const auto readWord = [&bytes](std::size_t offset) {
        return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(bytes[offset]) |
            (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U));
    };
    CHECK(readWord(0) == 65535U);
    CHECK(readWord(2) == 0U);
    CHECK(readWord(4) == 0U);
    CHECK(static_cast<int>(readWord(6)) == Catch::Approx(16384).margin(1));
}

TEST_CASE("Opaque-black rgba conversions bake the display-referred luma matte", "[output][video]") {
    const auto zero = Imath::half{0.0F}.bits();
    const auto half = Imath::half{0.5F}.bits();
    const auto one = Imath::half{1.0F}.bits();

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 2;
    image.height = 2;
    image.rgbaHalf.resize(static_cast<std::size_t>(image.width) * image.height * 4U);
    for (std::size_t pixelIndex = 0; pixelIndex < image.rgbaHalf.size() / 4U; ++pixelIndex) {
        const auto offset = pixelIndex * 4U;
        image.rgbaHalf[offset + 0U] = one;
        image.rgbaHalf[offset + 1U] = half;
        image.rgbaHalf[offset + 2U] = zero;
        image.rgbaHalf[offset + 3U] = half;
    }

    // The matte is applied after the sRGB transfer (After Effects
    // display-referred luma matte) and alpha leaves as fully opaque.
    const auto rgba8 = invisible_places::output::ConvertHalfRgbaToSrgbRgba8OpaqueBlack(
        image,
        1,
        1,
        true);
    REQUIRE(rgba8.size() == 4U);
    CHECK(static_cast<int>(rgba8[0]) == Catch::Approx(128).margin(1));
    CHECK(static_cast<int>(rgba8[1]) == Catch::Approx(94).margin(1));
    CHECK(rgba8[2] == 0U);
    CHECK(rgba8[3] == 255U);

    const auto rgba16 = invisible_places::output::ConvertHalfRgbaToSrgbRgba16OpaqueBlack(
        image,
        1,
        1,
        true);
    REQUIRE(rgba16.size() == 8U);
    const auto readWord = [&rgba16](std::size_t offset) {
        return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(rgba16[offset]) |
            (static_cast<std::uint16_t>(rgba16[offset + 1U]) << 8U));
    };
    CHECK(static_cast<int>(readWord(0)) == Catch::Approx(32768).margin(2));
    CHECK(static_cast<int>(readWord(2)) == Catch::Approx(24096).margin(4));
    CHECK(readWord(4) == 0U);
    CHECK(readWord(6) == 65535U);

    // The rgb48 conversion (Test MP4's) must agree on the colour lanes.
    const auto rgb16 = invisible_places::output::ConvertHalfRgbaToSrgbRgb16OpaqueBlack(
        image,
        1,
        1,
        true);
    REQUIRE(rgb16.size() == 6U);
    const auto readRgbWord = [&rgb16](std::size_t offset) {
        return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(rgb16[offset]) |
            (static_cast<std::uint16_t>(rgb16[offset + 1U]) << 8U));
    };
    CHECK(readRgbWord(0) == readWord(0));
    CHECK(readRgbWord(2) == readWord(2));
    CHECK(readRgbWord(4) == readWord(4));
}

TEST_CASE("Export output name suffix sanitizes and appends before the extension", "[output][video]") {
    using invisible_places::output::AppendExportOutputNameSuffix;
    using invisible_places::output::SanitizeExportOutputNameSuffix;

    CHECK(SanitizeExportOutputNameSuffix("take 2!") == "take_2");
    CHECK(SanitizeExportOutputNameSuffix("   ") == "");
    CHECK(SanitizeExportOutputNameSuffix("_client") == "client");

    const auto temporaryRoot =
        std::filesystem::temp_directory_path() /
        "invisible_places_output_suffix_test";
    std::filesystem::remove_all(temporaryRoot);
    std::filesystem::create_directories(temporaryRoot);

    const auto videoPath = temporaryRoot / "Animation_Colour.mov";
    CHECK(AppendExportOutputNameSuffix(videoPath, "") == videoPath);
    CHECK(AppendExportOutputNameSuffix({}, "take2").empty());
    const auto suffixed = AppendExportOutputNameSuffix(videoPath, "take2");
    CHECK(suffixed == temporaryRoot / "Animation_Colour_take2.mov");

    // A survivor from an earlier suffixed run must not be overwritten.
    { std::ofstream existing{suffixed}; existing << "x"; }
    const auto deduplicated = AppendExportOutputNameSuffix(videoPath, "take2");
    CHECK(deduplicated == temporaryRoot / "Animation_Colour_take2_2.mov");

    // Directory outputs (PNG/EXR stacks) take the suffix on the last
    // component.
    const auto stackDirectory = temporaryRoot / "Animation_PNG";
    CHECK(
        AppendExportOutputNameSuffix(stackDirectory, "take2") ==
        temporaryRoot / "Animation_PNG_take2");

    std::filesystem::remove_all(temporaryRoot);
}

TEST_CASE("SS2x conversion resolves parallel rows without changing uniform color", "[output][video]") {
    const auto zero = Imath::half{0.0F}.bits();
    const auto half = Imath::half{0.5F}.bits();
    const auto one = Imath::half{1.0F}.bits();

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 128;
    image.height = 128;
    image.rgbaHalf.resize(static_cast<std::size_t>(image.width) * image.height * 4U);
    for (std::size_t pixelIndex = 0; pixelIndex < image.rgbaHalf.size() / 4U; ++pixelIndex) {
        const auto offset = pixelIndex * 4U;
        image.rgbaHalf[offset + 0U] = one;
        image.rgbaHalf[offset + 1U] = half;
        image.rgbaHalf[offset + 2U] = zero;
        image.rgbaHalf[offset + 3U] = half;
    }

    const auto bytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(
        image,
        64,
        64,
        {},
        true);
    REQUIRE(bytes.size() == 64U * 64U * 4U);
    for (std::size_t pixelIndex = 0; pixelIndex < bytes.size() / 4U; ++pixelIndex) {
        const auto offset = pixelIndex * 4U;
        CHECK(bytes[offset + 0U] == 255U);
        CHECK(bytes[offset + 1U] == 188U);
        CHECK(bytes[offset + 2U] == 0U);
        CHECK(bytes[offset + 3U] == 128U);
    }
}

TEST_CASE("Non-SS2x conversion preserves uniform half-float RGBA", "[output][video]") {
    const auto quarter = Imath::half{0.25F}.bits();
    const auto half = Imath::half{0.5F}.bits();
    const auto one = Imath::half{1.0F}.bits();

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 9;
    image.height = 9;
    image.rgbaHalf.resize(static_cast<std::size_t>(image.width) * image.height * 4U);
    for (std::size_t pixelIndex = 0; pixelIndex < image.rgbaHalf.size() / 4U; ++pixelIndex) {
        const auto offset = pixelIndex * 4U;
        image.rgbaHalf[offset + 0U] = quarter;
        image.rgbaHalf[offset + 1U] = half;
        image.rgbaHalf[offset + 2U] = one;
        image.rgbaHalf[offset + 3U] = half;
    }

    const auto bytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba16(image, 3, 3, true);
    REQUIRE(bytes.size() == 3U * 3U * 8U);
    const auto readWord = [&bytes](std::size_t offset) {
        return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(bytes[offset]) |
            (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U));
    };
    for (std::size_t pixelIndex = 0; pixelIndex < bytes.size() / 8U; ++pixelIndex) {
        const auto offset = pixelIndex * 8U;
        CHECK(static_cast<int>(readWord(offset + 0U)) == Catch::Approx(35199).margin(8));
        CHECK(static_cast<int>(readWord(offset + 2U)) == Catch::Approx(48192).margin(8));
        CHECK(readWord(offset + 4U) == 65535U);
        CHECK(static_cast<int>(readWord(offset + 6U)) == Catch::Approx(32768).margin(8));
    }
}

TEST_CASE("Test MP4 matches After Effects display-space luma matte over black", "[output][video]") {
    const auto zero = Imath::half{0.0F}.bits();
    const auto half = Imath::half{0.5F}.bits();
    const auto one = Imath::half{1.0F}.bits();

    invisible_places::output::HalfRgbaExrImage image;
    image.width = 2;
    image.height = 1;
    image.rgbaHalf = {
        one,
        zero,
        zero,
        half,
        zero,
        one,
        zero,
        zero,
    };

    const auto bytes = invisible_places::output::ConvertHalfRgbaToSrgbRgb16OpaqueBlack(image, 2, 1, true);
    REQUIRE(bytes.size() == 12U);
    const auto readWord = [&bytes](std::size_t offset) {
        return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(bytes[offset]) |
            (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U));
    };
    // AE's default non-linear composition first encodes opaque red to 1.0,
    // then applies the 0.5 matte, rather than encoding linear 0.5 to ~0.735.
    CHECK(static_cast<int>(readWord(0)) == Catch::Approx(32768).margin(2));
    CHECK(readWord(2) == 0U);
    CHECK(readWord(4) == 0U);
    CHECK(readWord(6) == 0U);
    CHECK(readWord(8) == 0U);
    CHECK(readWord(10) == 0U);
}

TEST_CASE("Legacy MP4 smoothing setting does not alter pixels", "[output][video]") {
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

    const auto defaultBytes = invisible_places::output::ConvertHalfRgbaToSrgbRgba8(image);
    REQUIRE(defaultBytes.size() == 12U);
    CHECK(defaultBytes[4] == 0U);
    CHECK(defaultBytes[7] == 0U);

    invisible_places::output::Mp4SparsePointSmoothingSettings legacySmoothing;
    legacySmoothing.enabled = true;
    const auto legacyBytes =
        invisible_places::output::ConvertHalfRgbaToSrgbRgba8(image, legacySmoothing);
    CHECK(legacyBytes == defaultBytes);
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

    const auto bytes =
        invisible_places::output::ConvertHalfRgbaToSrgbRgba8(image, 1, 1);
    REQUIRE(bytes.size() == 4U);
    CHECK(bytes[0] == 188U);
    CHECK(bytes[1] == 188U);
    CHECK(bytes[2] == 188U);
    CHECK(bytes[3] == 255U);
}

TEST_CASE("PNG writer saves RGBA8 preview frames with alpha", "[output][png]") {
    const auto outputPath = std::filesystem::temp_directory_path() / "invisible_places_preview_frame.png";
    const std::vector<std::uint8_t> rgba{
        255, 0, 0, 255,
        0, 255, 0, 128,
        0, 0, 255, 0,
        255, 255, 255, 64,
    };

    std::string errorMessage;
    REQUIRE(invisible_places::output::WritePngRgba8(outputPath, 2, 2, rgba, &errorMessage));
    CHECK(errorMessage.empty());

    std::ifstream input{outputPath, std::ios::binary};
    REQUIRE(input);
    const std::vector<std::uint8_t> bytes{
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};
    REQUIRE(bytes.size() > 64U);

    const std::array<std::uint8_t, 8> signature{
        0x89U, 0x50U, 0x4EU, 0x47U, 0x0DU, 0x0AU, 0x1AU, 0x0AU};
    CHECK(std::equal(signature.begin(), signature.end(), bytes.begin()));
    CHECK(bytes[8] == 0U);
    CHECK(bytes[9] == 0U);
    CHECK(bytes[10] == 0U);
    CHECK(bytes[11] == 13U);
    CHECK(bytes[12] == static_cast<std::uint8_t>('I'));
    CHECK(bytes[13] == static_cast<std::uint8_t>('H'));
    CHECK(bytes[14] == static_cast<std::uint8_t>('D'));
    CHECK(bytes[15] == static_cast<std::uint8_t>('R'));
    CHECK(bytes[19] == 2U);
    CHECK(bytes[23] == 2U);
    CHECK(bytes[24] == 8U);
    CHECK(bytes[25] == 6U);
    REQUIRE(bytes.size() >= 12U);
    CHECK(bytes[bytes.size() - 8U] == static_cast<std::uint8_t>('I'));
    CHECK(bytes[bytes.size() - 7U] == static_cast<std::uint8_t>('E'));
    CHECK(bytes[bytes.size() - 6U] == static_cast<std::uint8_t>('N'));
    CHECK(bytes[bytes.size() - 5U] == static_cast<std::uint8_t>('D'));

    CHECK_FALSE(invisible_places::output::WritePngRgba8(outputPath, 2, 2, {1, 2, 3}, &errorMessage));
    CHECK(errorMessage.find("RGBA buffer") != std::string::npos);
    std::filesystem::remove(outputPath);
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

TEST_CASE("Offline point renderer stacks opacity emission and falloff", "[output][offline][point-style]") {
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
    invisible_places::style::SetScalarConstant(&stacked.opacity, 0.5F);
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
        style.depthFade.active = true;

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

TEST_CASE("Offline point colourise carries the selected colour lightness", "[output][offline][point-style]") {
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

    auto renderWithColourise = [&](std::array<float, 3> colour,
                                   float amount) {
        invisible_places::renderer::pointcloud::PointCloudStyleState style;
        style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb;
        style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
        style.colorizeColor = {colour[0], colour[1], colour[2]};
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

    const auto unchanged = renderWithColourise({0.0F, 0.0F, 1.0F}, 0.0F);
    const auto brightBlue = renderWithColourise({0.0F, 0.0F, 1.0F}, 1.0F);
    const auto darkBlue = renderWithColourise({0.0F, 0.0F, 0.30F}, 1.0F);
    const auto center = static_cast<std::size_t>(4) * 9U + 4U;
    REQUIRE(unchanged.alpha[center] > 0.0F);
    REQUIRE(brightBlue.alpha[center] > 0.0F);
    REQUIRE(darkBlue.alpha[center] > 0.0F);
    CHECK(unchanged.beautyR[center] > unchanged.beautyB[center]);
    CHECK(brightBlue.beautyB[center] > brightBlue.beautyR[center]);
    CHECK(darkBlue.beautyB[center] > darkBlue.beautyR[center]);
    // The selected colour's lightness participates: a bright selection lifts
    // the dark source point well above its original lightness...
    CHECK(brightBlue.beautyB[center] / brightBlue.alpha[center] >
          unchanged.beautyR[center] / unchanged.alpha[center] + 0.05F);
    // ...and a darker selection of the same hue lands visibly darker than
    // the bright one, so value is no longer invariant.
    CHECK(darkBlue.beautyB[center] / darkBlue.alpha[center] <
          brightBlue.beautyB[center] / brightBlue.alpha[center] - 0.05F);
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
    invisible_places::style::SetScalarConstant(&baseline.depthFade, 0.0F);
    const auto baselineImage = renderWithStyle(baseline);

    invisible_places::renderer::pointcloud::PointCloudStyleState inactive = baseline;
    invisible_places::style::SetScalarConstant(&inactive.opacity, 0.0F);
    invisible_places::style::SetScalarConstant(&inactive.emissiveStrength, 8.0F);
    invisible_places::style::SetScalarConstant(&inactive.depthFade, 1.0F);
    inactive.opacity.active = false;
    inactive.emissiveStrength.active = false;
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
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .generatedWaterOverlay = true,
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

TEST_CASE("Offline water trail overlays use trail tangent and streak length", "[output][offline][water][v2]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.hasSourceRgb = false;
    cloud.bounds.Expand(cloud.positions.front());

    const std::vector<std::string> trailFields{
        "trail_role",
        "trail_id",
        "source_id",
        "path_id",
        "branch_id",
        "trail_seed",
        "point_seed",
        "trail_distance",
        "trail_length",
        "route_start_index",
        "route_point_count",
        "route_length",
        "trail_start_phase",
        "trail_lateral_offset",
        "point_age",
        "trail_age",
        "trail_speed",
        "trail_width",
        "trail_streak_length",
        "trail_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z",
        "trail_lane_index",
        "trail_lane_count",
        "trail_lane_pitch",
        "trail_lane_span",
        "trail_lane_crossing",
        "trail_cross_seed"};
    cloud.scalarFields.reserve(trailFields.size());
    for (const auto& name : trailFields) {
        cloud.scalarFields.push_back({
            .name = name,
            .minimum = 0.0F,
            .maximum = 1.0F,
            .count = cloud.positions.size(),
            .valid = true,
        });
    }
    cloud.scalarFieldValues.assign(trailFields.size() * cloud.positions.size(), 0.0F);
    auto setField = [&cloud](std::size_t fieldSlot, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, 0)] = value;
    };
    setField(0U, 1.0F);
    setField(17U, 0.080F);
    setField(18U, 1.20F);
    setField(19U, 1.0F);
    setField(20U, 1.0F);
    setField(22U, 1.0F);
    setField(25U, 0.0F);
    setField(26U, 1.0F);
    setField(27U, 0.040F);
    setField(28U, 0.0F);
    setField(29U, 0.0F);
    setField(30U, 0.0F);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterTrailOverlay = true;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.01F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .generatedWaterOverlay = true,
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

TEST_CASE("Offline water trail overlays animate through time playback", "[output][offline][water][v2]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {
        {-0.40F, 0.0F, 0.0F},
        {0.40F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
    };
    cloud.hasSourceRgb = false;
    for (const auto& position : cloud.positions) {
        cloud.bounds.Expand(position);
    }

    const std::vector<std::string> trailFields{
        "trail_role",
        "trail_id",
        "source_id",
        "path_id",
        "branch_id",
        "trail_seed",
        "point_seed",
        "trail_distance",
        "trail_length",
        "route_start_index",
        "route_point_count",
        "route_length",
        "trail_start_phase",
        "trail_lateral_offset",
        "point_age",
        "trail_age",
        "trail_speed",
        "trail_width",
        "trail_streak_length",
        "trail_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z",
        "trail_lane_index",
        "trail_lane_count",
        "trail_lane_pitch",
        "trail_lane_span",
        "trail_lane_crossing",
        "trail_cross_seed"};
    cloud.scalarFields.reserve(trailFields.size());
    for (const auto& name : trailFields) {
        cloud.scalarFields.push_back({
            .name = name,
            .minimum = 0.0F,
            .maximum = 1.0F,
            .count = cloud.positions.size(),
            .valid = true,
        });
    }
    cloud.scalarFieldValues.assign(trailFields.size() * cloud.positions.size(), 0.0F);
    auto setField = [&cloud](std::size_t fieldSlot, std::size_t pointIndex, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, pointIndex)] = value;
    };
    for (std::size_t pointIndex = 0; pointIndex < cloud.PointCount(); ++pointIndex) {
        setField(9U, pointIndex, 0.0F);
        setField(10U, pointIndex, 2.0F);
        setField(11U, pointIndex, 0.80F);
        setField(16U, pointIndex, 0.80F);
        setField(17U, pointIndex, 0.04F);
        setField(18U, pointIndex, 0.08F);
        setField(19U, pointIndex, 1.0F);
        setField(20U, pointIndex, 1.0F);
        setField(22U, pointIndex, 1.0F);
        setField(25U, pointIndex, 0.0F);
        setField(26U, pointIndex, 1.0F);
        setField(27U, pointIndex, 0.040F);
        setField(28U, pointIndex, 0.0F);
        setField(29U, pointIndex, 0.0F);
        setField(30U, pointIndex, 0.0F);
    }
    setField(0U, 2U, 1.0F);
    setField(8U, 2U, 0.10F);
    setField(12U, 2U, 0.25F);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterTrailOverlay = true;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.01F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .generatedWaterOverlay = true,
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

    auto renderAlphaCentroid = [&](float timeSeconds) {
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
        float alphaSum = 0.0F;
        float weightedX = 0.0F;
        for (std::uint32_t y = 0; y < image.height; ++y) {
            for (std::uint32_t x = 0; x < image.width; ++x) {
                const auto index = static_cast<std::size_t>(y) * image.width + x;
                alphaSum += image.alpha[index];
                weightedX += image.alpha[index] * static_cast<float>(x);
            }
        }
        return std::pair{alphaSum, weightedX / std::max(0.001F, alphaSum)};
    };

    const auto [frameAAlpha, frameACentroidX] = renderAlphaCentroid(0.0F);
    const auto [frameBAlpha, frameBCentroidX] = renderAlphaCentroid(0.5F);
    CHECK(frameAAlpha > 0.0F);
    CHECK(frameBAlpha > 0.0F);
    CHECK(frameBCentroidX > frameACentroidX + 4.0F);
}

TEST_CASE("Offline water trail placement uses baked lateral offsets", "[output][offline][water][v2]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {
        {0.0F, -0.40F, 0.0F},
        {0.0F, 0.40F, 0.0F},
        {0.0F, 0.0F, 0.0F},
    };
    cloud.hasSourceRgb = false;
    for (const auto& position : cloud.positions) {
        cloud.bounds.Expand(position);
    }

    const std::vector<std::string> trailFields{
        "trail_role",
        "trail_id",
        "source_id",
        "path_id",
        "branch_id",
        "trail_seed",
        "point_seed",
        "trail_distance",
        "trail_length",
        "route_start_index",
        "route_point_count",
        "route_length",
        "trail_start_phase",
        "trail_lateral_offset",
        "point_age",
        "trail_age",
        "trail_speed",
        "trail_width",
        "trail_streak_length",
        "trail_confidence",
        "wetness",
        "feature_type",
        "tangent_x",
        "tangent_y",
        "tangent_z",
        "trail_lane_index",
        "trail_lane_count",
        "trail_lane_pitch",
        "trail_lane_span",
        "trail_lane_crossing",
        "trail_cross_seed"};
    cloud.scalarFields.reserve(trailFields.size());
    for (const auto& name : trailFields) {
        cloud.scalarFields.push_back({
            .name = name,
            .minimum = 0.0F,
            .maximum = 1.0F,
            .count = cloud.positions.size(),
            .valid = true,
        });
    }
    cloud.scalarFieldValues.assign(trailFields.size() * cloud.positions.size(), 0.0F);
    auto setField = [&cloud](std::size_t fieldSlot, std::size_t pointIndex, float value) {
        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldSlot, pointIndex)] = value;
    };
    for (std::size_t pointIndex = 0; pointIndex < cloud.PointCount(); ++pointIndex) {
        setField(9U, pointIndex, 0.0F);
        setField(10U, pointIndex, 2.0F);
        setField(11U, pointIndex, 0.80F);
        setField(16U, pointIndex, 0.80F);
        setField(17U, pointIndex, 0.05F);
        setField(18U, pointIndex, 0.09F);
        setField(19U, pointIndex, 1.0F);
        setField(20U, pointIndex, 1.0F);
        setField(23U, pointIndex, 1.0F);
        setField(25U, pointIndex, 3.0F);
        setField(26U, pointIndex, 7.0F);
        setField(27U, pointIndex, 0.025F);
        setField(28U, pointIndex, 0.60F);
        setField(30U, pointIndex, 0.23F);
    }
    setField(0U, 2U, 1.0F);
    setField(8U, 2U, 0.08F);
    setField(12U, 2U, 0.07F);

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.geometryMode =
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::CameraFacingWorldSprites;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {1.0F, 1.0F, 1.0F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.flowAnimation = true;
    style.waterTrailOverlay = true;
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.01F);
    invisible_places::style::SetScalarConstant(&style.opacity, 1.0F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .generatedWaterOverlay = true,
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

    auto renderAlphaCentroidX = [&](float timeSeconds) {
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
        float alphaSum = 0.0F;
        float weightedX = 0.0F;
        for (std::uint32_t y = 0; y < image.height; ++y) {
            for (std::uint32_t x = 0; x < image.width; ++x) {
                const auto index = static_cast<std::size_t>(y) * image.width + x;
                alphaSum += image.alpha[index];
                weightedX += image.alpha[index] * static_cast<float>(x);
            }
        }
        return std::pair{alphaSum, weightedX / std::max(0.001F, alphaSum)};
    };

    for (std::size_t pointIndex = 0; pointIndex < cloud.PointCount(); ++pointIndex) {
        setField(13U, pointIndex, 0.0F);
        setField(29U, pointIndex, 0.0F);
    }
    const auto [stableAlphaA, stableCentroidA] = renderAlphaCentroidX(0.0F);
    const auto [stableAlphaB, stableCentroidB] = renderAlphaCentroidX(0.5F);
    CHECK(stableAlphaA > 0.0F);
    CHECK(stableAlphaB > 0.0F);
    CHECK(std::abs(stableCentroidB - stableCentroidA) < 0.5F);

    for (std::size_t pointIndex = 0; pointIndex < cloud.PointCount(); ++pointIndex) {
        setField(29U, pointIndex, 1.0F);
    }
    const auto [crossingOnlyAlpha, crossingOnlyCentroid] = renderAlphaCentroidX(0.0F);
    CHECK(crossingOnlyAlpha > 0.0F);
    CHECK(std::abs(crossingOnlyCentroid - stableCentroidA) < 0.5F);
    for (float timeSeconds : {0.08F, 0.14F, 0.21F, 0.33F, 0.47F, 0.62F}) {
        const auto [crossingAlpha, crossingCentroid] = renderAlphaCentroidX(timeSeconds);
        CHECK(crossingAlpha > 0.0F);
        CHECK(std::abs(crossingCentroid - stableCentroidA) < 0.5F);
    }

    setField(13U, 2U, 0.18F);
    const auto [bakedOffsetAlphaA, bakedOffsetCentroidA] = renderAlphaCentroidX(0.0F);
    const auto [bakedOffsetAlphaB, bakedOffsetCentroidB] = renderAlphaCentroidX(0.5F);
    CHECK(bakedOffsetAlphaA > 0.0F);
    CHECK(bakedOffsetAlphaB > 0.0F);
    CHECK(std::abs(bakedOffsetCentroidA - stableCentroidA) > 1.0F);
    CHECK(std::abs(bakedOffsetCentroidB - bakedOffsetCentroidA) < 0.75F);
}

TEST_CASE("Offline point diagnostics record depth pass for point layers", "[output][offline][point-style]") {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, 0.0F}};
    cloud.packedColors = {0xFFFFFFFFU};
    cloud.hasSourceRgb = true;

    invisible_places::renderer::pointcloud::PointCloudStyleState style;
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

    CHECK(diagnostics.depthPassLayers == 1U);
    CHECK(diagnostics.depthVisitedPoints == 1U);
    CHECK(diagnostics.accumulationPassLayers == 1U);
    CHECK(diagnostics.accumulationVisitedPoints == 1U);
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
    CHECK(ScalarConstant(style.depthFade) == Catch::Approx(0.0F));
    CHECK_FALSE(style.depthFade.active);
    CHECK(style.colorizeAmount == Catch::Approx(0.0F));
    CHECK(ResolvePointCloudMaterialVariant(style) == PointCloudMaterialVariant::OpaqueHardDisc);

    auto movingStyle = style;
    movingStyle.roughnessMotionStrength = 0.02F;
    CHECK(ResolvePointCloudMaterialVariant(movingStyle) == PointCloudMaterialVariant::Unified);
}

TEST_CASE("Scene role roughness motion only animates vegetation", "[pointcloud][style][scene]") {
    using invisible_places::renderer::pointcloud::MakePointCloudStyleForSceneRole;
    using invisible_places::renderer::pointcloud::PointCloudSceneRoleAllowsRoughnessMotion;
    using invisible_places::renderer::pointcloud::PointCloudStyleHasActiveRoughnessMotion;
    using invisible_places::renderer::pointcloud::PointCloudStyleHasActiveShorelineWaves;
    using invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm;
    using invisible_places::renderer::pointcloud::PointCloudStyleState;

    PointCloudStyleState style;
    style.roughnessMotionStrength = 0.02F;
    style.roughnessMotionSpeed = 0.5F;
    style.shorelineWaveEnabled = true;
    style.shorelineWaveAlgorithm = PointCloudShorelineWaveAlgorithm::HeightFoam;
    style.shorelineHeightFoam.breakZ = 1.27F;

    CHECK(PointCloudSceneRoleAllowsRoughnessMotion(""));
    CHECK(PointCloudSceneRoleAllowsRoughnessMotion("VEG"));
    CHECK(PointCloudSceneRoleAllowsRoughnessMotion("Vegetation"));
    CHECK_FALSE(PointCloudSceneRoleAllowsRoughnessMotion("ROCK"));
    CHECK_FALSE(PointCloudSceneRoleAllowsRoughnessMotion("SAND"));

    CHECK(PointCloudStyleHasActiveRoughnessMotion(MakePointCloudStyleForSceneRole(style, "")));
    CHECK(PointCloudStyleHasActiveRoughnessMotion(MakePointCloudStyleForSceneRole(style, "VEG")));
    CHECK_FALSE(PointCloudStyleHasActiveRoughnessMotion(MakePointCloudStyleForSceneRole(style, "ROCK")));
    CHECK_FALSE(PointCloudStyleHasActiveRoughnessMotion(MakePointCloudStyleForSceneRole(style, "SAND")));
    CHECK_FALSE(MakePointCloudStyleForSceneRole(style, "").roughnessMotionFullLayer);
    CHECK(MakePointCloudStyleForSceneRole(style, "VEG").roughnessMotionFullLayer);
    CHECK(MakePointCloudStyleForSceneRole(style, "Vegetation").roughnessMotionFullLayer);
    CHECK_FALSE(MakePointCloudStyleForSceneRole(style, "ROCK").roughnessMotionFullLayer);
    CHECK_FALSE(MakePointCloudStyleForSceneRole(style, "SAND").roughnessMotionFullLayer);
    CHECK(MakePointCloudStyleForSceneRole(style, "").shorelineWaveEnabled);
    CHECK(MakePointCloudStyleForSceneRole(style, "SAND").shorelineWaveEnabled);
    CHECK(
        MakePointCloudStyleForSceneRole(style, "SAND").shorelineWaveAlgorithm ==
        PointCloudShorelineWaveAlgorithm::HeightFoam);
    CHECK(
        MakePointCloudStyleForSceneRole(style, "SAND").shorelineHeightFoam.breakZ ==
        Catch::Approx(1.27F));
    CHECK_FALSE(MakePointCloudStyleForSceneRole(style, "ROCK").shorelineWaveEnabled);
    CHECK_FALSE(MakePointCloudStyleForSceneRole(style, "VEG").shorelineWaveEnabled);
    CHECK(PointCloudStyleHasActiveShorelineWaves(MakePointCloudStyleForSceneRole(style, "SAND")));
    CHECK_FALSE(PointCloudStyleHasActiveShorelineWaves(MakePointCloudStyleForSceneRole(style, "ROCK")));

    auto staticShoreline = MakePointCloudStyleForSceneRole(style, "SAND");
    staticShoreline.shorelineHeightFoam.speed = 0.0F;
    CHECK_FALSE(PointCloudStyleHasActiveShorelineWaves(staticShoreline));
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
    style.shorelineWaveEnabled = true;
    style.shorelineWaveAlgorithm =
        invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm::HeightFoam;
    style.shorelineHeightFoam.breakZ = 1.26F;
    SetScalarConstant(&style.pointSize, 12.0F);
    SetScalarConstant(&style.opacity, 0.35F);
    SetScalarConstant(&style.emissiveStrength, 2.0F);
    SetScalarConstant(&style.depthFade, 0.5F);
    style.depthFade.active = true;

    const auto fast = MakeFastBasicPointCloudStyle(style, true);

    CHECK(fast.geometryMode == PointCloudGeometryMode::ScreenSprites);
    CHECK(fast.colorMode == PointCloudColorMode::ScalarColormap);
    CHECK(fast.falloffProfile == PointCloudFalloffProfile::HardDisc);
    CHECK(fast.stylisationMode == PointCloudStylisationMode::Off);
    CHECK_FALSE(fast.flowAnimation);
    CHECK(fast.shorelineWaveEnabled);
    CHECK(
        fast.shorelineWaveAlgorithm ==
        invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm::HeightFoam);
    CHECK(fast.shorelineHeightFoam.breakZ == Catch::Approx(1.26F));
    CHECK(ScalarConstant(fast.pointSize) == Catch::Approx(1.0F));
    CHECK(ScalarConstant(fast.opacity) == Catch::Approx(1.0F));
    CHECK(ScalarConstant(fast.emissiveStrength) == Catch::Approx(0.0F));
    CHECK(ScalarConstant(fast.depthFade) == Catch::Approx(0.0F));
    CHECK_FALSE(fast.depthFade.active);
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
