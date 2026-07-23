#include "io/AssetDiscovery.hpp"
#include "io/PointCloudData.hpp"
#include "renderer/core/VulkanViewportShell.hpp"
#include "scene/PointCloudVariants.hpp"
#include "scene/SceneCatalog.hpp"
#include "water/RainSimulation.hpp"

#include "InvisiblePlacesBuildConfig.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using invisible_places::io::Float3;
using invisible_places::water::WaterSurfaceRole;
using invisible_places::water::WaterSurfaceSample;

struct TemporaryDirectory {
    std::filesystem::path path;

    TemporaryDirectory() {
        path = std::filesystem::temp_directory_path() /
               ("invisible-places-rain-" + std::to_string(
                   std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
};

void WritePointPly(
    const std::filesystem::path& path,
    const std::vector<std::array<float, 6>>& points) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << points.size() << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "property float normal_x\n"
           << "property float normal_y\n"
           << "property float normal_z\n"
           << "property float scalar_unused\n"
           << "end_header\n";
    for (const auto& point : points) {
        output.write(reinterpret_cast<const char*>(point.data()), 3 * sizeof(float));
        constexpr std::array<std::uint8_t, 3> colour{10U, 20U, 30U};
        output.write(reinterpret_cast<const char*>(colour.data()), colour.size());
        output.write(reinterpret_cast<const char*>(point.data() + 3), 3 * sizeof(float));
        constexpr float unused = 42.0F;
        output.write(reinterpret_cast<const char*>(&unused), sizeof(unused));
    }
}

void WritePointPlyWithRoughness(
    const std::filesystem::path& path,
    const std::vector<std::array<float, 7>>& points) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << points.size() << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property float normal_x\n"
           << "property float normal_y\n"
           << "property float normal_z\n"
           << "property float scalar_Roughness\n"
           << "end_header\n";
    for (const auto& point : points) {
        output.write(
            reinterpret_cast<const char*>(point.data()),
            static_cast<std::streamsize>(point.size() * sizeof(float)));
    }
}

std::vector<WaterSurfaceSample> MakeCollisionSamples() {
    return {
        {{0.01F, 0.01F, 0.20F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.01F, 0.01F, 0.18F}, {0.0F, 0.1F, 0.99F}, WaterSurfaceRole::Rock},
        {{0.05F, 0.01F, 0.10F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Sand},
        {{0.10F, 0.00F, 0.44F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Vegetation},
        {{0.10F, 0.00F, 0.42F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Vegetation},
    };
}

invisible_places::water::WaterSurfaceCachePayloadChecksum
ReadPersistedGpuStreamChecksum(
    const invisible_places::water::WaterSurfacePersistedGpuTables& tables) {
    std::ifstream input{tables.filePath, std::ios::binary};
    if (!input.is_open()) {
        throw std::runtime_error{"Unable to open persisted GPU tables."};
    }
    invisible_places::water::WaterSurfaceGpuStreamChecksumBuilder checksum;
    std::array<std::byte, 23U> scratch{};
    const auto readSection = [&](std::uint64_t offset,
                                 std::uint64_t tag,
                                 std::uint64_t count,
                                 std::uint64_t elementBytes) {
        input.clear();
        input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
        if (!input.good()) {
            throw std::runtime_error{"Invalid persisted GPU table offset."};
        }
        checksum.AddPod(tag);
        checksum.AddPod(count);
        std::uint64_t remaining = count * elementBytes;
        while (remaining > 0U) {
            const auto chunkBytes = static_cast<std::size_t>(
                std::min<std::uint64_t>(remaining, scratch.size()));
            input.read(
                reinterpret_cast<char*>(scratch.data()),
                static_cast<std::streamsize>(chunkBytes));
            if (!input.good()) {
                throw std::runtime_error{"Truncated persisted GPU table."};
            }
            checksum.AddBytes(scratch.data(), chunkBytes);
            remaining -= chunkBytes;
        }
    };
    readSection(
        tables.surfaceOffset,
        4U,
        tables.surfaceCount,
        sizeof(invisible_places::water::RainGpuSurfaceSlot));
    readSection(
        tables.vegetationOffset,
        5U,
        tables.vegetationCount,
        sizeof(invisible_places::water::RainGpuVegetationSlot));
    readSection(
        tables.flowSurfaceOffset,
        6U,
        tables.flowSurfaceCount,
        sizeof(invisible_places::water::WaterGpuSurfaceSurfelSlot));
    return checksum.Finish();
}

}  // namespace

TEST_CASE("rain collision input streams only positions and normals", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "points.ply";
    WritePointPly(path, {
        {1.0F, 2.0F, 3.0F, 0.0F, 0.0F, 2.0F},
        {4.0F, 5.0F, 6.0F, 0.0F, 3.0F, 0.0F},
    });

    std::vector<invisible_places::io::PointCloudPositionNormalSample> samples;
    const auto result = invisible_places::io::StreamPointCloudPositionsNormals(
        path,
        [&](const auto& sample, std::uint64_t) {
            samples.push_back(sample);
            return true;
        });

    REQUIRE(result.success);
    CHECK(result.pointCount == 2U);
    REQUIRE(samples.size() == 2U);
    CHECK(samples[0].position.z == Catch::Approx(3.0F));
    CHECK(samples[0].normal.z == Catch::Approx(1.0F));
    CHECK(samples[1].normal.y == Catch::Approx(1.0F));
}

TEST_CASE("water surface input streams optional roughness in the same pass", "[water][rain][flow][cache]") {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "roughness.ply";
    WritePointPlyWithRoughness(path, {
        {1.0F, 2.0F, 3.0F, 0.0F, 0.0F, 1.0F, 0.125F},
        {4.0F, 5.0F, 6.0F, 0.0F, 1.0F, 0.0F, 0.375F},
    });

    std::vector<invisible_places::io::PointCloudPositionNormalSample> samples;
    const auto result = invisible_places::io::StreamPointCloudPositionsNormals(
        path,
        [&](const auto& sample, std::uint64_t) {
            samples.push_back(sample);
            return true;
        });

    REQUIRE(result.success);
    CHECK(result.hasNormals);
    CHECK(result.hasRoughness);
    REQUIRE(samples.size() == 2U);
    CHECK(samples[0].hasRoughness);
    CHECK(samples[0].roughness == Catch::Approx(0.125F));
    CHECK(samples[1].roughness == Catch::Approx(0.375F));
}

TEST_CASE("shared water cache consumes roughness during its source scan", "[water][rain][flow][cache]") {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "rock-2mm.ply";
    WritePointPlyWithRoughness(path, {
        {0.002F, 0.004F, 0.006F, 0.0F, 0.0F, 1.0F, 0.10F},
        {0.008F, 0.008F, 0.008F, 0.0F, 0.0F, 1.0F, 0.30F},
    });
    const std::vector<invisible_places::water::WaterSurfaceSource> sources{{
        .sourcePath = path,
        .role = WaterSurfaceRole::Rock,
        .spacingMicrometres = 2'000U,
    }};

    const auto result = invisible_places::water::BuildWaterSurfaceCache(sources);
    REQUIRE(result.success);
    CHECK(result.diagnostics.sourceScanCount == 1U);
    CHECK(result.diagnostics.gpuTableBuildCount == 1U);
    CHECK(result.diagnostics.fullPayloadHashPassCount == 0U);
    CHECK(result.cache.sourcePointCount == 2U);
    REQUIRE(result.cache.flowSurfaceSurfels.size() == 1U);
    CHECK(result.cache.flowSurfaceSurfels[0].roughness == Catch::Approx(0.20F));
    CHECK_FALSE(result.cache.gpuData.flowSurfaceTable.empty());
}

TEST_CASE("shared water cache retains orientation independent role surfels", "[water][rain][flow][cache]") {
    const std::vector<WaterSurfaceSample> samples{
        {.position = {0.002F, 0.004F, 0.006F},
         .normal = {0.0F, 0.0F, 1.0F},
         .role = WaterSurfaceRole::Rock,
         .roughness = 0.20F,
         .hasRoughness = true},
        {.position = {0.008F, 0.008F, 0.008F},
         .normal = {0.0F, 0.0F, -1.0F},
         .role = WaterSurfaceRole::Rock,
         .roughness = 0.40F,
         .hasRoughness = true},
        {.position = {0.006F, 0.006F, 0.008F},
         .normal = {1.0F, 0.0F, 0.0F},
         .role = WaterSurfaceRole::Sand},
        {.position = {0.006F, 0.006F, 0.026F},
         .normal = {1.0F, 0.0F, 0.0F},
         .role = WaterSurfaceRole::Rock},
    };

    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    CHECK(cache.schemaVersion == invisible_places::water::kWaterSurfaceCacheSchemaVersion);
    // Rain still retains one top-height cell per XY coordinate and role.
    REQUIRE(cache.surfaceCells.size() == 1U);
    CHECK(cache.surfaceCells[0].rockHeight == Catch::Approx(0.026F));
    // Flow retains both surface sheets and keeps ROCK/SAND separate in a shared voxel.
    REQUIRE(cache.flowSurfaceSurfels.size() == 3U);
    const auto firstRock = std::find_if(
        cache.flowSurfaceSurfels.begin(),
        cache.flowSurfaceSurfels.end(),
        [](const auto& surfel) {
            return surfel.cellZ == 0 && surfel.role == WaterSurfaceRole::Rock;
        });
    REQUIRE(firstRock != cache.flowSurfaceSurfels.end());
    CHECK(firstRock->centroid.x == Catch::Approx(0.005F));
    CHECK(firstRock->centroid.y == Catch::Approx(0.006F));
    CHECK(firstRock->centroid.z == Catch::Approx(0.007F));
    CHECK(firstRock->normal.z == Catch::Approx(1.0F));
    CHECK(firstRock->normalCoherence == Catch::Approx(1.0F));
    CHECK(firstRock->normalVariance == Catch::Approx(0.0F));
    CHECK(firstRock->roughness == Catch::Approx(0.30F));
    CHECK(firstRock->sampleCount == 2U);
}

TEST_CASE("ten millimetre cache averages unoriented two millimetre normals deterministically",
          "[water][rain][flow][cache][normals]") {
    const std::vector<WaterSurfaceSample> samples{
        {{0.002F, 0.002F, 0.002F}, {0.6F, 0.0F, 0.8F}, WaterSurfaceRole::Rock},
        {{0.004F, 0.004F, 0.004F}, {-0.6F, 0.0F, -0.8F}, WaterSurfaceRole::Rock},
        {{0.002F, 0.002F, 0.012F}, {0.0F, 1.0F, 0.0F}, WaterSurfaceRole::Rock},
        {{0.004F, 0.004F, 0.014F}, {0.0F, -1.0F, 0.0F}, WaterSurfaceRole::Rock},
        {{0.006F, 0.006F, 0.002F}, {1.0F, 0.0F, 0.0F}, WaterSurfaceRole::Vegetation},
        {{0.008F, 0.008F, 0.004F}, {-1.0F, 0.0F, 0.0F}, WaterSurfaceRole::Vegetation},
    };
    auto reversedSamples = samples;
    std::reverse(reversedSamples.begin(), reversedSamples.end());

    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    const auto reversed =
        invisible_places::water::BuildWaterSurfaceCacheFromSamples(reversedSamples);

    CHECK(cache.resolutionMeters == Catch::Approx(0.010F));
    REQUIRE(cache.surfaceCells.size() == 1U);
    REQUIRE(reversed.surfaceCells.size() == cache.surfaceCells.size());
    CHECK(cache.surfaceCells[0].rockHeight == Catch::Approx(0.014F));
    CHECK(cache.surfaceCells[0].rockNormal.x == Catch::Approx(0.0F));
    CHECK(cache.surfaceCells[0].rockNormal.y == Catch::Approx(1.0F));
    CHECK(cache.surfaceCells[0].rockNormal.z == Catch::Approx(0.0F));
    CHECK(cache.surfaceCells[0].rockSampleCount == 2U);
    CHECK(reversed.surfaceCells[0].rockNormal.y ==
          Catch::Approx(cache.surfaceCells[0].rockNormal.y));

    REQUIRE(cache.flowSurfaceSurfels.size() == 2U);
    REQUIRE(reversed.flowSurfaceSurfels.size() == cache.flowSurfaceSurfels.size());
    const auto& lowerRock = cache.flowSurfaceSurfels[0];
    const auto& upperRock = cache.flowSurfaceSurfels[1];
    CHECK(lowerRock.cellZ == 0);
    CHECK(lowerRock.normal.x == Catch::Approx(0.6F));
    CHECK(lowerRock.normal.z == Catch::Approx(0.8F));
    CHECK(lowerRock.sampleCount == 2U);
    CHECK(upperRock.cellZ == 1);
    CHECK(upperRock.normal.y == Catch::Approx(1.0F));
    CHECK(upperRock.sampleCount == 2U);
    CHECK(reversed.flowSurfaceSurfels[0].normal.x == Catch::Approx(lowerRock.normal.x));
    CHECK(reversed.flowSurfaceSurfels[1].normal.y == Catch::Approx(upperRock.normal.y));

    REQUIRE(cache.vegetationVoxels.size() == 1U);
    CHECK(cache.vegetationVoxels[0].normal.x == Catch::Approx(1.0F));
    CHECK(cache.vegetationVoxels[0].sampleCount == 2U);
    CHECK(reversed.vegetationVoxels[0].normal.x == Catch::Approx(1.0F));
}

TEST_CASE("water surface residency identity distinguishes legacy revision collisions", "[water][rain][flow][cache][identity]") {
    const std::vector<WaterSurfaceSample> firstSamples{
        {{0.001F, 0.001F, 0.001F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.010F, 0.010F, 0.010F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.039F, 0.039F, 0.039F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
    };
    auto secondSamples = firstSamples;
    secondSamples[1].normal = {1.0F, 0.0F, 0.0F};

    const auto first = invisible_places::water::BuildWaterSurfaceCacheFromSamples(firstSamples);
    const auto identical = invisible_places::water::BuildWaterSurfaceCacheFromSamples(firstSamples);
    const auto second = invisible_places::water::BuildWaterSurfaceCacheFromSamples(secondSamples);

    // These are exactly the compact values that the old Vulkan residency check
    // used, including identical bounds. Only the immutable payload differs.
    CHECK(first.revision == second.revision);
    CHECK(first.resolutionMeters == second.resolutionMeters);
    CHECK(first.bounds.valid == second.bounds.valid);
    CHECK(first.bounds.minimum.x == second.bounds.minimum.x);
    CHECK(first.bounds.minimum.y == second.bounds.minimum.y);
    CHECK(first.bounds.minimum.z == second.bounds.minimum.z);
    CHECK(first.bounds.maximum.x == second.bounds.maximum.x);
    CHECK(first.bounds.maximum.y == second.bounds.maximum.y);
    CHECK(first.bounds.maximum.z == second.bounds.maximum.z);

    REQUIRE(first.cacheIdentity.Valid());
    CHECK(first.cacheIdentity.sourceSignature == "memory");
    CHECK(first.gpuData.sourceIdentity == first.cacheIdentity);
    CHECK(identical.cacheIdentity == first.cacheIdentity);
    CHECK(second.cacheIdentity != first.cacheIdentity);
    CHECK(second.gpuData.sourceIdentity == second.cacheIdentity);

    auto differentlyNamed = first;
    differentlyNamed.signature = "another-scene-with-the-same-compact-revision";
    differentlyNamed.revision = first.revision;
    CHECK(
        invisible_places::water::BuildWaterSurfaceCacheIdentity(differentlyNamed) !=
        first.cacheIdentity);
}

TEST_CASE("water surface cache derives roughness from normal variance", "[water][rain][flow][cache]") {
    const std::vector<WaterSurfaceSample> samples{
        {{0.002F, 0.002F, 0.002F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.006F, 0.006F, 0.006F}, {1.0F, 0.0F, 0.0F}, WaterSurfaceRole::Rock},
    };
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    REQUIRE(cache.flowSurfaceSurfels.size() == 1U);
    const auto& surfel = cache.flowSurfaceSurfels.front();
    CHECK(surfel.normalCoherence == Catch::Approx(std::sqrt(0.5F)));
    CHECK(surfel.normalVariance == Catch::Approx(1.0F - std::sqrt(0.5F)));
    CHECK(surfel.roughness == Catch::Approx(surfel.normalVariance));
}

TEST_CASE("water surface CPU queries preserve the continuous surface sheet", "[water][rain][flow][cache]") {
    const std::vector<WaterSurfaceSample> samples{
        // A nearer parallel sheet along the reference normal.
        {{0.0F, 0.0F, 0.021F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        // A slightly farther candidate in the reference tangent plane. Its
        // inverted source normal must be hemisphere-aligned to the query frame.
        {{0.025F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}, WaterSurfaceRole::Rock},
        {{0.001F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Sand},
    };
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);

    const auto continuous = invisible_places::water::QueryWaterSurfaceCache(
        cache,
        {0.0F, 0.0F, 0.0F},
        0.04F,
        {0.0F, 0.0F, 1.0F},
        invisible_places::water::kWaterSurfaceRockRoleMask);
    REQUIRE(continuous.hit);
    CHECK(continuous.surfel.centroid.x == Catch::Approx(0.025F));
    CHECK(continuous.surfel.normal.z == Catch::Approx(1.0F));

    const auto sandOnly = invisible_places::water::QueryWaterSurfaceCache(
        cache,
        {0.0F, 0.0F, 0.0F},
        0.04F,
        {},
        invisible_places::water::kWaterSurfaceSandRoleMask);
    REQUIRE(sandOnly.hit);
    CHECK(sandOnly.surfel.role == WaterSurfaceRole::Sand);
    CHECK(sandOnly.distanceMeters == Catch::Approx(0.001F));

    const auto outsideSupport = invisible_places::water::QueryWaterSurfaceCache(
        cache,
        {1.0F, 1.0F, 1.0F},
        0.01F);
    CHECK_FALSE(outsideSupport.hit);
}

TEST_CASE("water surface sources prefer exact two millimetre complete bundles",
          "[water][rain][cache]") {
    invisible_places::scene::ScenePointCloudGroup group;
    group.variantsByRole[0] = {
        {.role = invisible_places::scene::ScenePointCloudRole::Rock, .spacingMicrometres = 1'000U, .sourcePath = "rock1.ply"},
        {.role = invisible_places::scene::ScenePointCloudRole::Rock, .spacingMicrometres = 2'000U, .sourcePath = "rock2.ply"},
    };
    group.variantsByRole[1] = {
        {.role = invisible_places::scene::ScenePointCloudRole::Sand, .spacingMicrometres = 2'000U, .sourcePath = "sand2.ply"},
    };
    group.variantsByRole[2] = {
        {.role = invisible_places::scene::ScenePointCloudRole::Vegetation, .spacingMicrometres = 1'000U, .sourcePath = "veg1.ply"},
        {.role = invisible_places::scene::ScenePointCloudRole::Vegetation, .spacingMicrometres = 3'000U, .sourcePath = "veg3.ply"},
    };

    CHECK(invisible_places::water::SelectWaterSurfaceSources(group).empty());

    group.completeDisplayBundles.push_back({
        .spacingMicrometres = 3'000U,
        .byRole = {{
            {.role = invisible_places::scene::ScenePointCloudRole::Rock,
             .spacingMicrometres = 3'000U,
             .sourcePath = "rock3.ply"},
            {.role = invisible_places::scene::ScenePointCloudRole::Sand,
             .spacingMicrometres = 3'000U,
             .sourcePath = "sand3.ply"},
            {.role = invisible_places::scene::ScenePointCloudRole::Vegetation,
             .spacingMicrometres = 3'000U,
             .sourcePath = "veg3.ply"},
        }},
    });
    auto distantBundle = group.completeDisplayBundles.back();
    distantBundle.spacingMicrometres = 5'000U;
    for (auto& variant : distantBundle.byRole) {
        variant.spacingMicrometres = 5'000U;
    }
    group.completeDisplayBundles.insert(
        group.completeDisplayBundles.begin(),
        std::move(distantBundle));
    const auto fallbackSources = invisible_places::water::SelectWaterSurfaceSources(group);
    REQUIRE(fallbackSources.size() == 3U);
    for (const auto& source : fallbackSources) {
        CHECK(source.spacingMicrometres == 3'000U);
        CHECK(source.isFallback);
    }

    group.completeDisplayBundles.push_back({
        .spacingMicrometres = 2'000U,
        .byRole = {{
            {.role = invisible_places::scene::ScenePointCloudRole::Rock,
             .spacingMicrometres = 2'000U,
             .sourcePath = "rock2.ply"},
            {.role = invisible_places::scene::ScenePointCloudRole::Sand,
             .spacingMicrometres = 2'000U,
             .sourcePath = "sand2.ply"},
            {.role = invisible_places::scene::ScenePointCloudRole::Vegetation,
             .spacingMicrometres = 2'000U,
             .sourcePath = "veg2.ply"},
        }},
    });
    const auto sources = invisible_places::water::SelectWaterSurfaceSources(group);
    REQUIRE(sources.size() == 3U);
    CHECK(sources[0].sourcePath == "rock2.ply");
    CHECK_FALSE(sources[0].isFallback);
    CHECK(sources[1].sourcePath == "sand2.ply");
    CHECK_FALSE(sources[1].isFallback);
    CHECK(sources[2].sourcePath == "veg2.ply");
    CHECK_FALSE(sources[2].isFallback);
    for (const auto& source : sources) {
        CHECK(source.spacingMicrometres == 2'000U);
    }
}

TEST_CASE("SampleScene water sources select exact two millimetre normal support",
          "[water][rain][cache][sample]") {
    const auto dataRoot = std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR};
    if (!std::filesystem::exists(dataRoot / "SampleScene")) {
        SKIP("SampleScene fixture is not present in the local Data directory.");
    }

    const auto assetCatalog = invisible_places::io::DiscoverAssets(dataRoot);
    const auto sceneCatalog = invisible_places::scene::SceneCatalog::FromDiscoveredAssets(assetCatalog);
    const auto* sampleScene = sceneCatalog.FindPointCloudGroup("SampleScene");
    REQUIRE(sampleScene != nullptr);
    const auto* displayBundle = sampleScene->FindCompleteDisplayBundle(3'000U);
    REQUIRE(displayBundle != nullptr);
    CHECK(displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Rock).sourcePath.filename() ==
          "Site1-ROCK-3mm.Sample.ply");
    CHECK(displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Sand).sourcePath.filename() ==
          "Site1-SAND-3mm.Sample.ply");
    CHECK(displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Vegetation).sourcePath.filename() ==
          "Site1-VEG-3mm.Sample.ply");

    const auto sources = invisible_places::water::SelectWaterSurfaceSources(*sampleScene);
    REQUIRE(sources.size() == 3U);
    CHECK(sources[0].sourcePath.filename() == "Site1-ROCK-2mm.Sample.ply");
    CHECK(sources[1].sourcePath.filename() == "Site1-SAND-2mm.Sample.ply");
    CHECK(sources[2].sourcePath.filename() == "Site1-VEG-2mm.Sample.ply");
    for (const auto& source : sources) {
        CHECK(source.spacingMicrometres == 2'000U);
        CHECK_FALSE(source.isFallback);
    }
}

TEST_CASE("rain collision traces top surfaces and vegetation voxels", "[water][rain][cache]") {
    const auto samples = MakeCollisionSamples();
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);

    const auto rockHit = invisible_places::water::TraceRainCollision(
        cache,
        {0.01F, 0.01F, 0.8F},
        {0.01F, 0.01F, -0.2F});
    REQUIRE(rockHit.hit);
    CHECK(rockHit.role == WaterSurfaceRole::Rock);
    CHECK(rockHit.position.z == Catch::Approx(0.20F).margin(0.001F));

    const auto sandHit = invisible_places::water::TraceRainCollision(
        cache,
        {0.05F, 0.01F, 0.8F},
        {0.05F, 0.01F, -0.2F});
    REQUIRE(sandHit.hit);
    CHECK(sandHit.role == WaterSurfaceRole::Sand);

    const auto vegetationHit = invisible_places::water::TraceRainCollision(
        cache,
        {0.00F, 0.00F, 0.60F},
        {0.16F, 0.00F, 0.328F});
    REQUIRE(vegetationHit.hit);
    CHECK(vegetationHit.role == WaterSurfaceRole::Vegetation);
    CHECK(vegetationHit.segmentTime < 1.0F);
}

TEST_CASE("rain collision DDA cannot tunnel through distant vegetation", "[water][rain][cache][dda]") {
    const std::vector<WaterSurfaceSample> samples{{
        {5.005F, 0.005F, 0.005F},
        {0.0F, 0.0F, 1.0F},
        WaterSurfaceRole::Vegetation,
    }};
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    const auto hit = invisible_places::water::TraceRainCollision(
        cache,
        {0.005F, 0.005F, 10.005F},
        {10.005F, 0.005F, -9.995F});
    REQUIRE(hit.hit);
    CHECK(hit.role == WaterSurfaceRole::Vegetation);
    CHECK(hit.segmentTime == Catch::Approx(0.50F).margin(0.003F));
}

TEST_CASE("rain collision chooses the first role surface", "[water][rain][cache]") {
    const std::vector<WaterSurfaceSample> samples{
        {{0.005F, 0.005F, 0.20F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.005F, 0.005F, 0.40F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Sand},
    };
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    const auto hit = invisible_places::water::TraceRainCollision(
        cache,
        {0.005F, 0.005F, 1.0F},
        {0.005F, 0.005F, -1.0F});
    REQUIRE(hit.hit);
    CHECK(hit.role == WaterSurfaceRole::Sand);
    CHECK(hit.position.z == Catch::Approx(0.40F));
}

TEST_CASE("rain collision cache persistence rejects stale signatures", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(MakeCollisionSamples());
    cache.signature = "expected";
    const auto expectedIdentity =
        invisible_places::water::BuildWaterSurfaceCacheIdentity(cache);
    const auto path = temporary.path / "test.surfacecache";
    std::string error;
    REQUIRE(invisible_places::water::SaveWaterSurfaceCache(cache, path, &error));

    invisible_places::water::WaterSurfaceCache loaded;
    REQUIRE(invisible_places::water::LoadWaterSurfaceCache(path, "expected", &loaded, &error));
    CHECK(loaded.schemaVersion == invisible_places::water::kWaterSurfaceCacheSchemaVersion);
    CHECK(loaded.surfaceCells.size() == cache.surfaceCells.size());
    CHECK(loaded.vegetationVoxels.size() == cache.vegetationVoxels.size());
    REQUIRE(loaded.flowSurfaceSurfels.size() == cache.flowSurfaceSurfels.size());
    CHECK(loaded.flowSurfaceSurfels[0].centroid.x ==
          Catch::Approx(cache.flowSurfaceSurfels[0].centroid.x));
    CHECK(loaded.flowSurfaceSurfels[0].normalCoherence ==
          Catch::Approx(cache.flowSurfaceSurfels[0].normalCoherence));
    CHECK(loaded.gpuData.surfaceTable.empty());
    CHECK(loaded.gpuData.vegetationTable.empty());
    CHECK(loaded.gpuData.flowSurfaceTable.empty());
    CAPTURE(
        loaded.gpuData.persistedTables.filePath,
        loaded.gpuData.persistedTables.fileSize,
        loaded.gpuData.persistedTables.surfaceCount,
        loaded.gpuData.persistedTables.vegetationCount,
        loaded.gpuData.persistedTables.flowSurfaceCount,
        loaded.gpuData.persistedTables.streamChecksum.hashedByteCount,
        loaded.gpuData.persistedTables.streamChecksum.words[0],
        loaded.gpuData.persistedTables.streamChecksum.words[1]);
    REQUIRE(loaded.gpuData.persistedTables.Valid());
    CHECK(loaded.gpuData.persistedTables.filePath == path);
    CHECK(loaded.gpuData.persistedTables.fileSize ==
          std::filesystem::file_size(path));
    CHECK(loaded.gpuData.persistedTables.modificationTicks ==
          static_cast<std::int64_t>(
              std::filesystem::last_write_time(path).time_since_epoch().count()));
    CHECK(loaded.gpuData.persistedTables.surfaceCount == cache.gpuData.surfaceTable.size());
    CHECK(loaded.gpuData.persistedTables.vegetationCount == cache.gpuData.vegetationTable.size());
    CHECK(loaded.gpuData.persistedTables.flowSurfaceCount == cache.gpuData.flowSurfaceTable.size());
    CHECK(loaded.gpuData.persistedTables.surfaceOffset <
          loaded.gpuData.persistedTables.vegetationOffset);
    CHECK(loaded.gpuData.persistedTables.vegetationOffset <
          loaded.gpuData.persistedTables.flowSurfaceOffset);
    CHECK(ReadPersistedGpuStreamChecksum(loaded.gpuData.persistedTables) ==
          loaded.gpuData.persistedTables.streamChecksum);
    CHECK(loaded.gpuData.flowSurfaceMask == cache.gpuData.flowSurfaceMask);
    CHECK(loaded.gpuData.sourceRevision == cache.revision);
    CHECK(loaded.gpuData.payloadChecksum.Valid());
    CHECK(loaded.cacheIdentity == expectedIdentity);
    CHECK(loaded.gpuData.sourceIdentity == expectedIdentity);
    CHECK_FALSE(invisible_places::water::LoadWaterSurfaceCache(path, "changed", &loaded, &error));

    // A renderer may reopen the sidecar after asynchronous validation. Even
    // if an in-place edit preserves both size and timestamp, the direct
    // staging stream checksum must distinguish it before hidden resources are
    // promoted.
    const auto changedDuringUploadPath =
        temporary.path / "changed-during-upload.surfacecache";
    REQUIRE(std::filesystem::copy_file(path, changedDuringUploadPath));
    invisible_places::water::WaterSurfaceCache changedDuringUpload;
    REQUIRE(invisible_places::water::LoadWaterSurfaceCache(
        changedDuringUploadPath,
        "expected",
        &changedDuringUpload,
        &error));
    const auto unchangedWriteTime =
        std::filesystem::last_write_time(changedDuringUploadPath);
    {
        std::fstream edited{
            changedDuringUploadPath,
            std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(edited.is_open());
        edited.seekg(static_cast<std::streamoff>(
            changedDuringUpload.gpuData.persistedTables.surfaceOffset));
        char value = 0;
        edited.read(&value, 1U);
        REQUIRE(edited.good());
        value ^= static_cast<char>(0x5A);
        edited.seekp(static_cast<std::streamoff>(
            changedDuringUpload.gpuData.persistedTables.surfaceOffset));
        edited.write(&value, 1U);
        REQUIRE(edited.good());
    }
    std::filesystem::last_write_time(changedDuringUploadPath, unchangedWriteTime);
    CHECK(changedDuringUpload.gpuData.persistedTables.fileSize ==
          std::filesystem::file_size(changedDuringUploadPath));
    CHECK(changedDuringUpload.gpuData.persistedTables.modificationTicks ==
          static_cast<std::int64_t>(
              std::filesystem::last_write_time(changedDuringUploadPath)
                  .time_since_epoch()
                  .count()));
    CHECK(ReadPersistedGpuStreamChecksum(
              changedDuringUpload.gpuData.persistedTables) !=
          changedDuringUpload.gpuData.persistedTables.streamChecksum);

    // Schema 3 requires its checksum trailer; truncation cannot silently turn a
    // current cache into a trusted warm-load payload.
    const auto truncatedPath = temporary.path / "truncated.surfacecache";
    REQUIRE(std::filesystem::copy_file(path, truncatedPath));
    constexpr std::uintmax_t schema3TrailerBytes =
        8U + sizeof(std::uint32_t) + 8U + 4U * sizeof(std::uint64_t) +
        3U * sizeof(std::uint64_t);
    const auto savedSize = std::filesystem::file_size(truncatedPath);
    REQUIRE(savedSize > schema3TrailerBytes);

    const auto corruptedPath = temporary.path / "corrupted.surfacecache";
    REQUIRE(std::filesystem::copy_file(path, corruptedPath));
    {
        std::fstream corrupted{
            corruptedPath,
            std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(corrupted.is_open());
        const auto payloadByte = static_cast<std::streamoff>(
            savedSize - schema3TrailerBytes - 1U);
        corrupted.seekg(payloadByte);
        char value = 0;
        corrupted.read(&value, 1U);
        value ^= static_cast<char>(0x5A);
        corrupted.seekp(payloadByte);
        corrupted.write(&value, 1U);
    }
    CHECK_FALSE(invisible_places::water::LoadWaterSurfaceCache(
        corruptedPath,
        "expected",
        &loaded,
        &error));

    std::filesystem::resize_file(truncatedPath, savedSize - schema3TrailerBytes);
    CHECK_FALSE(invisible_places::water::LoadWaterSurfaceCache(
        truncatedPath,
        "expected",
        &loaded,
        &error));

    // Schema-2 files without the optional identity trailer remain readable as
    // migration input. Their legacy identity is recovered once, while schema 3
    // also records the fast payload checksum used by a subsequent migration.
    const auto legacyPath = temporary.path / "legacy.raincache";
    REQUIRE(std::filesystem::copy_file(path, legacyPath));
    std::filesystem::resize_file(legacyPath, savedSize - schema3TrailerBytes);
    {
        std::fstream legacy{legacyPath, std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(legacy.is_open());
        constexpr std::string_view legacyMagic = "IPWSC002";
        legacy.write(legacyMagic.data(), static_cast<std::streamsize>(legacyMagic.size()));
        constexpr std::uint32_t legacySchema =
            invisible_places::water::kWaterSurfaceCacheLegacySchemaVersion;
        legacy.write(
            reinterpret_cast<const char*>(&legacySchema),
            sizeof(legacySchema));
    }
    invisible_places::water::WaterSurfaceBuildDiagnostics legacyDiagnostics;
    REQUIRE(invisible_places::water::LoadWaterSurfaceCache(
        legacyPath,
        "expected",
        &loaded,
        &error,
        &legacyDiagnostics));
    CHECK(loaded.schemaVersion == invisible_places::water::kWaterSurfaceCacheLegacySchemaVersion);
    CHECK(loaded.cacheIdentity.Valid());
    CHECK(loaded.gpuData.payloadChecksum.Valid());
    CHECK_FALSE(loaded.gpuData.persistedTables.Valid());
    CHECK_FALSE(loaded.gpuData.surfaceTable.empty());
    CHECK(legacyDiagnostics.sourceScanCount == 0U);
    CHECK(legacyDiagnostics.gpuTableBuildCount == 0U);
    CHECK(legacyDiagnostics.fullPayloadHashPassCount == 1U);
}

TEST_CASE("water surface warm loads do not rescan or rebuild tables", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    const auto sceneDirectory = temporary.path / "Scene";
    std::filesystem::create_directories(sceneDirectory);
    const std::array<std::pair<std::string_view, WaterSurfaceRole>, 3> files{{
        {"rock-2mm.ply", WaterSurfaceRole::Rock},
        {"sand-2mm.ply", WaterSurfaceRole::Sand},
        {"vegetation-2mm.ply", WaterSurfaceRole::Vegetation},
    }};
    std::vector<invisible_places::water::WaterSurfaceSource> sources;
    for (std::size_t index = 0U; index < files.size(); ++index) {
        const auto path = sceneDirectory / files[index].first;
        WritePointPly(path, {{
            static_cast<float>(index) * 0.02F,
            0.0F,
            static_cast<float>(index) * 0.01F,
            0.0F,
            0.0F,
            1.0F,
        }});
        sources.push_back({
            .sourcePath = path,
            .role = files[index].second,
            .spacingMicrometres = 2'000U,
        });
    }

    const auto cold = invisible_places::water::BuildWaterSurfaceCache(
        sources,
        sceneDirectory);
    REQUIRE(cold.success);
    CHECK_FALSE(cold.loadedFromDisk);
    CHECK(cold.cache.resolutionMeters == Catch::Approx(0.010F));
    REQUIRE(cold.cache.sources.size() == 3U);
    for (const auto& source : cold.cache.sources) {
        CHECK(source.spacingMicrometres == 2'000U);
    }
    CHECK(cold.diagnostics.sourceScanCount == 3U);
    CHECK(cold.diagnostics.gpuTableBuildCount == 1U);
    CHECK(cold.diagnostics.fullPayloadHashPassCount == 0U);
    const auto cachePath = invisible_places::water::WaterSurfaceCachePath(
        sceneDirectory,
        cold.cache.signature);
    CHECK(std::filesystem::is_regular_file(cachePath));
    CHECK(cold.persistedPath == cachePath);

    const auto warm = invisible_places::water::BuildWaterSurfaceCache(
        sources,
        sceneDirectory);
    REQUIRE(warm.success);
    CHECK(warm.loadedFromDisk);
    CHECK(warm.diagnostics.sourceScanCount == 0U);
    CHECK(warm.diagnostics.gpuTableBuildCount == 0U);
    CHECK(warm.diagnostics.fullPayloadHashPassCount == 0U);
    CHECK(warm.cache.gpuData.persistedTables.Valid());
    CHECK(warm.cache.gpuData.surfaceTable.empty());
    CHECK(warm.cache.gpuData.vegetationTable.empty());
    CHECK(warm.cache.gpuData.flowSurfaceTable.empty());
    REQUIRE(warm.cache.sources.size() == cold.cache.sources.size());
    for (const auto& source : warm.cache.sources) {
        CHECK(source.spacingMicrometres == 2'000U);
    }
    CHECK(warm.cache.cacheIdentity == cold.cache.cacheIdentity);
    CHECK(warm.persistedPath == cachePath);
}

TEST_CASE("failed cold persistence never publishes a stale surface sidecar path",
          "[water][rain][cache][manifest]") {
    TemporaryDirectory temporary;
    const auto sceneDirectory = temporary.path / "Scene";
    std::filesystem::create_directories(sceneDirectory);
    std::vector<invisible_places::water::WaterSurfaceSource> sources;
    for (const auto& [filename, role] :
         std::array<std::pair<std::string_view, WaterSurfaceRole>, 3>{{
             {"rock-2mm.ply", WaterSurfaceRole::Rock},
             {"sand-2mm.ply", WaterSurfaceRole::Sand},
             {"vegetation-2mm.ply", WaterSurfaceRole::Vegetation},
         }}) {
        const auto path = sceneDirectory / filename;
        WritePointPly(path, {{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F}});
        sources.push_back({
            .sourcePath = path,
            .role = role,
            .spacingMicrometres = 2'000U,
        });
    }
    const auto signature = invisible_places::water::WaterSurfaceCacheSignature(sources);
    const auto cachePath = invisible_places::water::WaterSurfaceCachePath(
        sceneDirectory,
        signature);
    std::filesystem::create_directories(cachePath.parent_path());
    {
        std::ofstream corrupt{cachePath, std::ios::binary};
        REQUIRE(corrupt.is_open());
        corrupt << "not-a-water-surface-cache";
    }
    std::filesystem::create_directory(cachePath.string() + ".tmp");

    const auto rebuilt = invisible_places::water::BuildWaterSurfaceCache(
        sources,
        sceneDirectory);
    REQUIRE(rebuilt.success);
    CHECK_FALSE(rebuilt.loadedFromDisk);
    CHECK(rebuilt.diagnostics.sourceScanCount == 3U);
    CHECK(rebuilt.persistedPath.empty());
    CHECK(std::filesystem::file_size(cachePath) ==
          std::string_view{"not-a-water-surface-cache"}.size());
    CHECK_FALSE(rebuilt.warnings.empty());
}

TEST_CASE("failed cache replacement preserves the last valid target", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(
        MakeCollisionSamples());
    cache.signature = "atomic";
    const auto path = temporary.path / "atomic.surfacecache";
    std::string error;
    REQUIRE(invisible_places::water::SaveWaterSurfaceCache(cache, path, &error));
    const auto originalSize = std::filesystem::file_size(path);

    std::filesystem::create_directory(path.string() + ".tmp");
    CHECK_FALSE(invisible_places::water::SaveWaterSurfaceCache(cache, path, &error));
    CHECK(std::filesystem::is_regular_file(path));
    CHECK(std::filesystem::file_size(path) == originalSize);
    invisible_places::water::WaterSurfaceCache loaded;
    REQUIRE(invisible_places::water::LoadWaterSurfaceCache(
        path,
        "atomic",
        &loaded,
        &error));
    CHECK(loaded.cacheIdentity.Valid());
}

TEST_CASE("water surface cache persists below the project Saved directory", "[water][rain][cache]") {
    const std::filesystem::path savedDirectory{"Saved"};
    CHECK(
        invisible_places::water::WaterSurfaceCachePath(savedDirectory, "abc123") ==
        savedDirectory / ".invisible_places" / "cache" / "water" /
            "abc123.surfacecache");
    CHECK(
        invisible_places::water::WaterSurfaceSceneCachePath(savedDirectory, "abc123") ==
        invisible_places::water::WaterSurfaceCachePath(savedDirectory, "abc123"));
}

TEST_CASE("water surface persistence enforces a five GiB ceiling", "[water][rain][cache]") {
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(
        MakeCollisionSamples());
    const auto estimatedBytes =
        invisible_places::water::WaterSurfaceCacheEstimatedPersistenceBytes(cache);
    REQUIRE(estimatedBytes > 0U);
    CHECK(invisible_places::water::WaterSurfaceCacheFitsPersistenceLimit(cache));
    CHECK_FALSE(invisible_places::water::WaterSurfaceCacheFitsPersistenceLimit(
        cache,
        estimatedBytes - 1U));
    CHECK(invisible_places::water::WaterSurfaceCachePersistenceSizeAllowed(
        invisible_places::water::kWaterSurfaceCacheMaximumPersistenceBytes));
    CHECK_FALSE(invisible_places::water::WaterSurfaceCachePersistenceSizeAllowed(
        invisible_places::water::kWaterSurfaceCacheMaximumPersistenceBytes + 1U));
}

TEST_CASE("water surface upload staging has a fixed sixty four MiB ceiling", "[water][rain][cache][gpu]") {
    using invisible_places::renderer::core::ViewportDiagnostics;
    using invisible_places::renderer::core::kWaterSurfaceUploadStagingLimitBytes;
    CHECK(kWaterSurfaceUploadStagingLimitBytes == 64ULL * 1024ULL * 1024ULL);
    CHECK(ViewportDiagnostics{}.waterSurfacePeakStagingBytes == 0U);
}

TEST_CASE("water surface signatures survive moving a scene folder", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    const auto firstDirectory = temporary.path / "first";
    const auto secondDirectory = temporary.path / "second";
    const auto firstRock = firstDirectory / "support" / "rock" / "points.ply";
    const auto firstSand = firstDirectory / "support" / "sand" / "points.ply";
    const auto secondRock = secondDirectory / "support" / "rock" / "points.ply";
    const auto secondSand = secondDirectory / "support" / "sand" / "points.ply";
    std::filesystem::create_directories(firstRock.parent_path());
    std::filesystem::create_directories(firstSand.parent_path());
    std::filesystem::create_directories(secondRock.parent_path());
    std::filesystem::create_directories(secondSand.parent_path());
    WritePointPly(firstRock, {{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F}});
    REQUIRE(std::filesystem::copy_file(firstRock, firstSand));
    REQUIRE(std::filesystem::copy_file(firstRock, secondRock));
    REQUIRE(std::filesystem::copy_file(firstRock, secondSand));
    for (const auto& path : {firstSand, secondRock, secondSand}) {
        std::filesystem::last_write_time(
            path,
            std::filesystem::last_write_time(firstRock));
    }
    const std::vector<invisible_places::water::WaterSurfaceSource> firstSources{
        {
            .sourcePath = firstRock,
            .role = WaterSurfaceRole::Rock,
            .spacingMicrometres = 5'000U,
        },
        {
            .sourcePath = firstSand,
            .role = WaterSurfaceRole::Sand,
            .spacingMicrometres = 5'000U,
        },
    };
    auto secondSources = firstSources;
    secondSources[0].sourcePath = secondRock;
    secondSources[1].sourcePath = secondSand;
    CHECK(
        invisible_places::water::WaterSurfaceCacheSignature(firstSources) ==
        invisible_places::water::WaterSurfaceCacheSignature(secondSources));

    // The common-parent-relative path remains part of the signature, so two
    // nested files with the same basename cannot alias each other.
    secondSources[1].sourcePath = secondRock;
    CHECK(
        invisible_places::water::WaterSurfaceCacheSignature(firstSources) !=
        invisible_places::water::WaterSurfaceCacheSignature(secondSources));
}

TEST_CASE("water surface signatures invalidate for every terrain source input",
          "[water][rain][cache][invalidation]") {
    TemporaryDirectory temporary;
    const auto sourcePath = temporary.path / "ROCK" / "rock-5mm.ply";
    std::filesystem::create_directories(sourcePath.parent_path());
    WritePointPly(sourcePath, {{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F}});
    std::vector<invisible_places::water::WaterSurfaceSource> sources{{
        .sourcePath = sourcePath,
        .role = WaterSurfaceRole::Rock,
        .spacingMicrometres = 5'000U,
    }};
    const auto baseline = invisible_places::water::WaterSurfaceCacheSignature(sources);
    CHECK(invisible_places::water::kWaterSurfaceCacheAlgorithmId ==
          std::string_view{"water-surface-10mm-normal-average-v1"});
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(sources, 0.020F) != baseline);

    auto changedRole = sources;
    changedRole.front().role = WaterSurfaceRole::Sand;
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(changedRole) != baseline);

    auto changedSpacing = sources;
    changedSpacing.front().spacingMicrometres = 3'000U;
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(changedSpacing) != baseline);

    auto changedTransform = sources;
    changedTransform.front().hasTransform = true;
    changedTransform.front().localToWorld.values[12] = 0.125;
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(changedTransform) != baseline);

    const auto originalWriteTime = std::filesystem::last_write_time(sourcePath);
    std::filesystem::last_write_time(sourcePath, originalWriteTime + std::chrono::seconds{2});
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(sources) != baseline);
    std::filesystem::last_write_time(sourcePath, originalWriteTime);

    const auto renamedPath = sourcePath.parent_path() / "rock-renamed-5mm.ply";
    REQUIRE(std::filesystem::copy_file(sourcePath, renamedPath));
    std::filesystem::last_write_time(renamedPath, originalWriteTime);
    auto renamedSource = sources;
    renamedSource.front().sourcePath = renamedPath;
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(renamedSource) != baseline);
}

TEST_CASE("rain collision signatures include source file identity", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "rock5.ply";
    WritePointPly(path, {{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F}});
    const std::vector<invisible_places::water::WaterSurfaceSource> sources{{
        .sourcePath = path,
        .role = WaterSurfaceRole::Rock,
        .spacingMicrometres = 5'000U,
    }};
    const auto before = invisible_places::water::WaterSurfaceCacheSignature(sources);
    WritePointPly(path, {
        {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.02F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    });
    const auto after = invisible_places::water::WaterSurfaceCacheSignature(sources);
    CHECK(before != after);
}

TEST_CASE("rain GPU collision tables remain sparse and bounded", "[water][rain][gpu]") {
    auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(MakeCollisionSamples());
    cache.revision = 77U;
    const auto gpu = invisible_places::water::BuildWaterSurfaceGpuData(cache);

    REQUIRE_FALSE(gpu.surfaceTable.empty());
    REQUIRE_FALSE(gpu.vegetationTable.empty());
    REQUIRE_FALSE(gpu.flowSurfaceTable.empty());
    CHECK(cache.surfaceCells.size() <= static_cast<std::size_t>(gpu.surfaceTable.size() * 0.65F));
    CHECK(cache.vegetationVoxels.size() <= static_cast<std::size_t>(gpu.vegetationTable.size() * 0.65F));
    CHECK(cache.flowSurfaceSurfels.size() <=
          static_cast<std::size_t>(gpu.flowSurfaceTable.size() * 0.80F));
    CHECK(gpu.maximumProbeCount <= 32U);
    CHECK(gpu.flowMaximumProbeCount <= 32U);
    CHECK(gpu.sourceRevision == 77U);

    const auto populatedFlowSlots = std::count_if(
        gpu.flowSurfaceTable.begin(),
        gpu.flowSurfaceTable.end(),
        [](const auto& slot) {
            return slot.cellX != std::numeric_limits<std::int32_t>::min();
        });
    CHECK(populatedFlowSlots == static_cast<std::ptrdiff_t>(cache.flowSurfaceSurfels.size()));
}

TEST_CASE("rain intensity modifies visuals without touching collision data", "[water][rain][preset]") {
    const auto light = invisible_places::water::RainIntensityValues(
        invisible_places::water::RainIntensityPreset::LightMist);
    const auto rain = invisible_places::water::RainIntensityValues(
        invisible_places::water::RainIntensityPreset::Rain);
    const auto heavy = invisible_places::water::RainIntensityValues(
        invisible_places::water::RainIntensityPreset::HeavyDownpour);
    CHECK(light.density < rain.density);
    CHECK(rain.density <= heavy.density);
    CHECK(light.width < rain.width);
    CHECK(rain.width < heavy.width);
    CHECK(light.speed < rain.speed);
    CHECK(rain.speed < heavy.speed);
    CHECK(light.windResponse > rain.windResponse);
    CHECK(rain.windResponse > heavy.windResponse);

    auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(MakeCollisionSamples());
    cache.revision = 19U;
    const auto before = invisible_places::water::BuildWaterSurfaceGpuData(cache);
    auto settings = invisible_places::water::DefaultRainRuntimeSettings();
    CHECK(settings.nearSurface.approachDistanceMeters == Catch::Approx(0.18F));
    CHECK(settings.nearSurface.minimumSpeedFactor == Catch::Approx(0.30F));
    CHECK(settings.nearSurface.squish == Catch::Approx(0.65F));
    CHECK(settings.nearSurface.normalAlignment == Catch::Approx(0.75F));
    CHECK(settings.rockImpact.edgeBreakup == Catch::Approx(0.35F));
    CHECK(settings.rockImpact.spreadSpeed == Catch::Approx(1.60F));
    CHECK(settings.rockImpact.centreFalloff == Catch::Approx(0.65F));
    CHECK(settings.rockImpact.heightBias == Catch::Approx(0.75F));
    CHECK(settings.rockImpact.persistence == Catch::Approx(1.35F));
    CHECK(settings.vegetationImpact.twinkle == Catch::Approx(1.80F));
    CHECK(settings.vegetationImpact.propagationMetersPerSecond == Catch::Approx(0.65F));
    CHECK(settings.vegetationImpact.hopSpacingMeters == Catch::Approx(0.070F));
    CHECK(settings.vegetationImpact.streamWidthMeters == Catch::Approx(0.010F));
    CHECK(settings.vegetationImpact.streamSpread == Catch::Approx(0.65F));
    settings.density = 0.12F;
    settings.windSpeedMetersPerSecond = 3.0F;
    settings.opacityScale = 0.2F;
    const auto after = invisible_places::water::BuildWaterSurfaceGpuData(cache);
    CHECK(before.sourceRevision == after.sourceRevision);
    CHECK(before.surfaceTable.size() == after.surfaceTable.size());
    CHECK(before.vegetationTable.size() == after.vegetationTable.size());

    const auto mistVisual = invisible_places::water::RainVisualPreset("Rain Mist");
    const auto fineVisual = invisible_places::water::RainVisualPreset("Rain Fine Lines");
    const auto downpourVisual = invisible_places::water::RainVisualPreset("Rain Downpour");
    CHECK(mistVisual.widthMeters < fineVisual.widthMeters);
    CHECK(fineVisual.widthMeters < downpourVisual.widthMeters);
    CHECK(mistVisual.streakLengthMeters < fineVisual.streakLengthMeters);
    CHECK(fineVisual.streakLengthMeters < downpourVisual.streakLengthMeters);
    CHECK(mistVisual.opacity < fineVisual.opacity);
    CHECK(fineVisual.opacity < downpourVisual.opacity);

    auto gridSettings = invisible_places::water::DefaultRainRuntimeSettings();
    gridSettings.spawnRadiusMeters = 80.0F;
    CHECK(invisible_places::water::RainImpactGridWorldSpan(gridSettings) == Catch::Approx(192.0F));
}

TEST_CASE("near-surface rain becomes a slowed widened ellipse", "[water][rain][visual]") {
    const invisible_places::water::RainNearSurfaceSettings settings{};
    const auto airborne = invisible_places::water::EvaluateRainParticleVisualShape(
        0.003F,
        0.16F,
        0.0F,
        settings);
    const auto approaching = invisible_places::water::EvaluateRainParticleVisualShape(
        0.003F,
        0.16F,
        1.0F,
        settings);

    CHECK(airborne.widthMeters == Catch::Approx(0.003F));
    CHECK(airborne.lengthMeters == Catch::Approx(0.16F));
    CHECK(airborne.ellipseBlend == Catch::Approx(0.0F));
    CHECK(approaching.widthMeters == Catch::Approx(0.00495F));
    CHECK(approaching.lengthMeters < airborne.lengthMeters * 0.07F);
    CHECK(approaching.lengthMeters < approaching.widthMeters * 2.0F);
    CHECK(approaching.ellipseBlend > 0.70F);

    auto fullSquish = settings;
    fullSquish.squish = 1.0F;
    const auto settled = invisible_places::water::EvaluateRainParticleVisualShape(
        0.003F,
        0.16F,
        1.0F,
        fullSquish);
    CHECK(settled.widthMeters == Catch::Approx(0.006F));
    CHECK(settled.lengthMeters == Catch::Approx(settled.widthMeters));
    CHECK(settled.ellipseBlend == Catch::Approx(1.0F));
}

TEST_CASE("rain simulator is deterministic and skips events when effects are off", "[water][rain][simulation]") {
    std::vector<WaterSurfaceSample> samples;
    for (int y = -30; y <= 30; ++y) {
        for (int x = -30; x <= 30; ++x) {
            samples.push_back({
                {x * 0.02F, y * 0.02F, 0.0F},
                {0.0F, 0.0F, 1.0F},
                WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    invisible_places::water::RainSimulationFrame frame;
    frame.settings.enabled = true;
    frame.settings.activeParticleCount = 32U;
    frame.settings.density = 1.0F;
    frame.settings.spawnRadiusMeters = 0.4F;
    frame.settings.spawnHeightMeters = 0.15F;
    frame.settings.cameraDeathDistanceMeters = 100.0F;
    frame.settings.seed = 54U;
    frame.cameraPosition = {0.0F, 0.0F, 1.0F};
    frame.spawnCentre = {0.0F, 0.0F, 0.0F};
    frame.deltaSeconds = 1.0F / 30.0F;

    invisible_places::water::RainSimulator first{32U};
    invisible_places::water::RainSimulator second{32U};
    for (int step = 0; step < 8; ++step) {
        frame.timeSeconds = step * frame.deltaSeconds;
        (void)first.Advance(frame, cache);
        (void)second.Advance(frame, cache);
    }
    REQUIRE(first.Particles().size() == second.Particles().size());
    CHECK(first.Particles()[0].position.x == Catch::Approx(second.Particles()[0].position.x));
    CHECK(first.Particles()[0].position.z == Catch::Approx(second.Particles()[0].position.z));

    frame.settings.impactEffectsEnabled = false;
    invisible_places::water::RainSimulator effectsOff{32U};
    std::uint32_t collisionCount = 0U;
    std::uint32_t eventCount = 0U;
    for (int step = 0; step < 8; ++step) {
        frame.timeSeconds = step * frame.deltaSeconds;
        const auto diagnostics = effectsOff.Advance(frame, cache);
        collisionCount += diagnostics.collisionCount;
        eventCount += diagnostics.emittedEvents;
    }
    CHECK(collisionCount > 0U);
    CHECK(eventCount == 0U);
}

TEST_CASE("rain slows once near cached terrain and keeps the averaged normal", "[water][rain][simulation]") {
    std::vector<WaterSurfaceSample> samples;
    for (int y = -30; y <= 30; ++y) {
        for (int x = -30; x <= 30; ++x) {
            samples.push_back({
                {x * 0.02F, y * 0.02F, 0.0F},
                {0.60F, 0.0F, 0.80F},
                WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    invisible_places::water::RainSimulationFrame frame;
    frame.settings.enabled = true;
    frame.settings.activeParticleCount = 1U;
    frame.settings.density = 1.0F;
    frame.settings.weatherFrontStrength = 0.0F;
    frame.settings.windSpeedMetersPerSecond = 0.0F;
    frame.settings.turbulence = 0.0F;
    frame.settings.gustStrength = 0.0F;
    frame.settings.spawnRadiusMeters = 0.10F;
    frame.settings.spawnHeightMeters = 0.10F;
    frame.settings.cameraDeathDistanceMeters = 100.0F;
    frame.settings.nearSurface.approachDistanceMeters = 0.24F;
    frame.settings.nearSurface.minimumSpeedFactor = 0.25F;
    frame.spawnCentre = {0.0F, 0.0F, 0.0F};
    frame.cameraPosition = {0.0F, 0.0F, 2.0F};
    frame.deltaSeconds = 0.01F;

    invisible_places::water::RainSimulator simulator{1U};
    bool observedApproach = false;
    bool observedCollision = false;
    for (int step = 0; step < 160; ++step) {
        const auto previous = simulator.Particles().front();
        frame.timeSeconds = step * frame.deltaSeconds;
        const auto diagnostics = simulator.Advance(frame, cache);
        const auto particle = simulator.Particles().front();
        if (particle.surfaceProximity > 0.01F && diagnostics.collisionCount == 0U &&
            previous.active && particle.generation == previous.generation) {
            observedApproach = true;
            const float fullStep = frame.settings.fallSpeedMetersPerSecond * frame.deltaSeconds;
            CHECK(previous.position.z - particle.position.z < fullStep - 1.0e-4F);
            CHECK(particle.surfaceNormal.x > 0.50F);
            CHECK(particle.surfaceNormal.z > 0.70F);
        }
        if (diagnostics.collisionCount > 0U) {
            observedCollision = true;
            CHECK(diagnostics.emittedEvents == 1U);
            break;
        }
    }
    CHECK(observedApproach);
    CHECK(observedCollision);
}

TEST_CASE("rain simulator respawns particles outside the camera range", "[water][rain][simulation]") {
    const std::vector<WaterSurfaceSample> samples{{
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        WaterSurfaceRole::Rock,
    }};
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    invisible_places::water::RainSimulationFrame frame;
    frame.settings.enabled = true;
    frame.settings.impactEffectsEnabled = false;
    frame.settings.activeParticleCount = 4U;
    frame.settings.density = 1.0F;
    frame.settings.weatherFrontStrength = 0.0F;
    frame.settings.spawnRadiusMeters = 0.1F;
    frame.settings.spawnHeightMeters = 1.0F;
    frame.settings.cameraDeathDistanceMeters = 1.0F;
    frame.spawnCentre = {5.0F, 0.0F, 0.0F};
    frame.cameraPosition = {0.0F, 0.0F, 0.0F};
    frame.deltaSeconds = 1.0F / 30.0F;

    invisible_places::water::RainSimulator simulator{4U};
    const auto diagnostics = simulator.Advance(frame, cache);
    CHECK(diagnostics.escapedParticles == 4U);
    CHECK(diagnostics.respawnCount >= 8U);
    CHECK(simulator.Particles()[0].generation >= 2U);
}

TEST_CASE("rain impact grid bounds work and isolates scene roles", "[water][rain][effects]") {
    std::vector<invisible_places::water::RainImpactEvent> events;
    for (std::uint32_t index = 0; index < 12U; ++index) {
        events.push_back({
            .position = {0.0F, 0.0F, 0.0F},
            .birthTimeSeconds = 0.0F,
            .normal = {0.0F, 0.0F, 1.0F},
            .radiusMeters = 0.10F,
            .role = WaterSurfaceRole::Rock,
            .lifetimeSeconds = 5.0F,
            .energy = 1.0F,
            .seed = index,
        });
    }
    events.push_back({
        .position = {0.20F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = WaterSurfaceRole::Sand,
        .lifetimeSeconds = 1.0F,
        .energy = 1.0F,
    });
    events.push_back({
        .position = {-0.20F, 0.0F, 1.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = WaterSurfaceRole::Vegetation,
        .lifetimeSeconds = 2.0F,
        .energy = 1.0F,
        .seed = 4U,
    });

    const auto grid = invisible_places::water::BuildRainImpactGrid(events, {}, 0.25F, 4.0F);
    CHECK(grid.overflowCount > 0U);
    const auto rock = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Rock, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.25F);
    const auto sandAtRock = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Sand, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.25F);
    const auto vegetationLowerEarly = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Vegetation, {-0.20F, 0.0F, 0.50F}, {0.0F, 0.0F, 1.0F}, 0.10F);
    float vegetationTopMaximum = 0.0F;
    for (int y = -20; y <= 20; ++y) {
        for (int x = -20; x <= 20; ++x) {
            const auto vegetationTop = invisible_places::water::EvaluateRainImpact(
                grid,
                WaterSurfaceRole::Vegetation,
                {-0.20F + static_cast<float>(x) * 0.004F,
                 static_cast<float>(y) * 0.004F,
                 0.95F},
                {0.0F, 0.0F, 1.0F},
                0.10F);
            vegetationTopMaximum = std::max(vegetationTopMaximum, vegetationTop.emission);
        }
    }
    CHECK(rock.colourBlend > 0.0F);
    CHECK(sandAtRock.opacity == Catch::Approx(0.0F));
    CHECK(vegetationTopMaximum > vegetationLowerEarly.emission);
}

TEST_CASE("rock rain impact uses the reduced slow-growing footprint", "[water][rain][effects]") {
    constexpr float radius = 0.12F;
    const float effectiveRadius = radius * std::sqrt(2.0F / 3.0F);
    constexpr float pi = 3.14159265358979323846F;
    CHECK(
        pi * effectiveRadius * effectiveRadius ==
        Catch::Approx((2.0F / 3.0F) * pi * radius * radius).epsilon(1.0e-6F));
    const std::vector<invisible_places::water::RainImpactEvent> events{
        {.position = {0.0F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = radius,
         .role = WaterSurfaceRole::Rock,
         .lifetimeSeconds = 5.0F,
         .energy = 1.0F,
         .seed = 91U},
    };
    auto smoothRock = invisible_places::water::RainRockImpactSettings{};
    smoothRock.edgeBreakup = 0.0F;
    smoothRock.centreFalloff = 0.0F;
    const auto grid = invisible_places::water::BuildRainImpactGrid(
        events,
        {},
        1.0F,
        2.0F,
        smoothRock);
    const auto centreAtBirth = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Rock, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.08F);
    const auto outerAtOldGrowthTime = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {effectiveRadius * 0.70F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        0.90F);
    const auto outerAtDoubledGrowthTime = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {effectiveRadius * 0.70F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.80F);
    const auto insideReducedFootprint = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {effectiveRadius * 0.90F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.80F);
    const auto insideOldButOutsideReducedFootprint = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {radius * 0.95F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.80F);
    const auto evaluatorInsideReducedEdge =
        invisible_places::water::EvaluateRockRainImpactValue(
            events.front(),
            {effectiveRadius * 0.975F, 0.0F, 0.0F},
            {0.0F, 0.0F, 1.0F},
            1.80F,
            smoothRock);
    const auto evaluatorOutsideReducedEdge =
        invisible_places::water::EvaluateRockRainImpactValue(
            events.front(),
            {effectiveRadius * 1.025F, 0.0F, 0.0F},
            {0.0F, 0.0F, 1.0F},
            1.80F,
            smoothRock);
    const auto flatLeftLate = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {-effectiveRadius * 0.40F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        4.0F);
    const auto flatRightLate = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {effectiveRadius * 0.40F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        4.0F);

    CHECK(centreAtBirth.colourBlend > 0.30F);
    CHECK(outerAtOldGrowthTime.colourBlend < 0.01F);
    CHECK(outerAtDoubledGrowthTime.colourBlend > 0.30F);
    CHECK(insideReducedFootprint.colourBlend > 0.30F);
    CHECK(insideOldButOutsideReducedFootprint.colourBlend < 0.01F);
    CHECK(evaluatorInsideReducedEdge > 0.79F);
    CHECK(evaluatorOutsideReducedEdge == Catch::Approx(0.0F).margin(1.0e-6F));
    CHECK(flatLeftLate.colourBlend == Catch::Approx(flatRightLate.colourBlend));
}

TEST_CASE("rock rain edge breakup is seeded, irregular, and inward only", "[water][rain][effects]") {
    constexpr float pi = 3.14159265358979323846F;
    constexpr float radius = 0.12F;
    const float effectiveRadius = radius * std::sqrt(2.0F / 3.0F);
    invisible_places::water::RainImpactEvent event{
        .position = {0.0F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = radius,
        .role = WaterSurfaceRole::Rock,
        .lifetimeSeconds = 5.0F,
        .energy = 1.0F,
        .seed = 0xA51U,
    };
    const auto settings = invisible_places::water::RainRockImpactSettings{};
    float minimumEdge = std::numeric_limits<float>::infinity();
    float maximumEdge = 0.0F;
    float seedDifference = 0.0F;
    for (int sample = 0; sample < 64; ++sample) {
        const float angle = 2.0F * pi * static_cast<float>(sample) / 64.0F;
        const Float3 point{
            std::cos(angle) * effectiveRadius * 0.92F,
            std::sin(angle) * effectiveRadius * 0.92F,
            0.0F,
        };
        const float first = invisible_places::water::EvaluateRockRainImpactValue(
            event,
            point,
            {0.0F, 0.0F, 1.0F},
            1.80F,
            settings);
        auto secondEvent = event;
        secondEvent.seed ^= 0x9E3779B9U;
        const float second = invisible_places::water::EvaluateRockRainImpactValue(
            secondEvent,
            point,
            {0.0F, 0.0F, 1.0F},
            1.80F,
            settings);
        minimumEdge = std::min(minimumEdge, first);
        maximumEdge = std::max(maximumEdge, first);
        seedDifference = std::max(seedDifference, std::abs(first - second));

        const Float3 outside{
            std::cos(angle) * radius * 1.001F,
            std::sin(angle) * radius * 1.001F,
            0.0F,
        };
        CHECK(invisible_places::water::EvaluateRockRainImpactValue(
                  event,
                  outside,
                  {0.0F, 0.0F, 1.0F},
                  1.80F,
                  settings) == Catch::Approx(0.0F).margin(1.0e-6F));
    }
    CHECK(maximumEdge > 0.10F);
    CHECK(minimumEdge < maximumEdge * 0.35F);
    CHECK(seedDifference > 0.10F);
}

TEST_CASE("rock rain impact favours lower points and settles downhill", "[water][rain][effects]") {
    constexpr float radius = 0.12F;
    constexpr float lifetime = 5.0F;
    const float effectiveRadius = radius * std::sqrt(2.0F / 3.0F);
    auto smoothRock = invisible_places::water::RainRockImpactSettings{};
    smoothRock.edgeBreakup = 0.0F;
    smoothRock.centreFalloff = 0.0F;
    const std::vector<invisible_places::water::RainImpactEvent> verticalEvents{
        {.position = {0.0F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {1.0F, 0.0F, 0.0F},
         .radiusMeters = radius,
         .role = WaterSurfaceRole::Rock,
         .lifetimeSeconds = lifetime,
         .energy = 1.0F,
         .seed = 92U},
    };
    const auto verticalGrid = invisible_places::water::BuildRainImpactGrid(
        verticalEvents,
        {},
        1.8F,
        2.0F,
        smoothRock);
    const auto highAtGrowth = invisible_places::water::EvaluateRainImpact(
        verticalGrid,
        WaterSurfaceRole::Rock,
        {0.0F, 0.0F, effectiveRadius * 0.50F},
        {1.0F, 0.0F, 0.0F},
        1.80F);
    const auto centreAtGrowth = invisible_places::water::EvaluateRainImpact(
        verticalGrid,
        WaterSurfaceRole::Rock,
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        1.80F);
    const auto lowAtGrowth = invisible_places::water::EvaluateRainImpact(
        verticalGrid,
        WaterSurfaceRole::Rock,
        {0.0F, 0.0F, -effectiveRadius * 0.50F},
        {1.0F, 0.0F, 0.0F},
        1.80F);
    const auto highAfterEarlyFade = invisible_places::water::EvaluateRainImpact(
        verticalGrid,
        WaterSurfaceRole::Rock,
        {0.0F, 0.0F, effectiveRadius * 0.50F},
        {1.0F, 0.0F, 0.0F},
        3.0F);
    const auto lowBeforeLateFade = invisible_places::water::EvaluateRainImpact(
        verticalGrid,
        WaterSurfaceRole::Rock,
        {0.0F, 0.0F, -effectiveRadius * 0.50F},
        {1.0F, 0.0F, 0.0F},
        3.0F);

    CHECK(highAtGrowth.colourBlend == Catch::Approx(0.42F * 0.8F).margin(0.002F));
    CHECK(centreAtGrowth.colourBlend == Catch::Approx(0.42F).margin(0.002F));
    CHECK(lowAtGrowth.colourBlend == Catch::Approx(0.42F * 1.2F).margin(0.002F));
    CHECK(lowBeforeLateFade.colourBlend > highAfterEarlyFade.colourBlend * 1.8F);

    constexpr float inverseSqrtTwo = 0.70710678118F;
    const std::vector<invisible_places::water::RainImpactEvent> slopedEvents{
        {.position = {0.0F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
         .radiusMeters = radius,
         .role = WaterSurfaceRole::Rock,
         .lifetimeSeconds = lifetime,
         .energy = 1.0F,
         .seed = 93U},
    };
    const auto slopedGrid = invisible_places::water::BuildRainImpactGrid(
        slopedEvents,
        {},
        4.5F,
        2.0F,
        smoothRock);
    const float probeDistance = effectiveRadius * 1.05F;
    const Float3 downhillProbe{
        inverseSqrtTwo * probeDistance,
        0.0F,
        -inverseSqrtTwo * probeDistance,
    };
    const Float3 uphillProbe{
        -downhillProbe.x,
        0.0F,
        -downhillProbe.z,
    };
    const auto downhillAtGrowth = invisible_places::water::EvaluateRainImpact(
        slopedGrid,
        WaterSurfaceRole::Rock,
        downhillProbe,
        {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
        1.80F);
    const auto downhillLate = invisible_places::water::EvaluateRainImpact(
        slopedGrid,
        WaterSurfaceRole::Rock,
        downhillProbe,
        {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
        4.50F);
    const auto uphillLate = invisible_places::water::EvaluateRainImpact(
        slopedGrid,
        WaterSurfaceRole::Rock,
        uphillProbe,
        {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
        4.50F);
    const auto expired = invisible_places::water::EvaluateRainImpact(
        slopedGrid,
        WaterSurfaceRole::Rock,
        downhillProbe,
        {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
        5.01F);
    const auto insideBroadPhase = invisible_places::water::EvaluateRockRainImpactValue(
        slopedEvents.front(),
        {inverseSqrtTwo * radius * 0.99F,
         0.0F,
        -inverseSqrtTwo * radius * 0.99F},
        {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
        4.50F,
        smoothRock);
    const auto outsideBroadPhase = invisible_places::water::EvaluateRockRainImpactValue(
        slopedEvents.front(),
        {inverseSqrtTwo * radius * 1.001F,
         0.0F,
        -inverseSqrtTwo * radius * 1.001F},
        {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
        4.50F,
        smoothRock);

    CHECK(downhillAtGrowth.colourBlend < 0.04F);
    CHECK(downhillLate.colourBlend > downhillAtGrowth.colourBlend * 3.0F);
    CHECK(uphillLate.colourBlend < 0.005F);
    CHECK(expired.colourBlend == Catch::Approx(0.0F));
    CHECK(insideBroadPhase > 0.0F);
    CHECK(outsideBroadPhase == Catch::Approx(0.0F).margin(1.0e-6F));
}

TEST_CASE("vegetation rain impacts form visible crown hops and downward streams", "[water][rain][effects]") {
    constexpr float radius = 0.065F;
    const std::vector<invisible_places::water::RainImpactEvent> events{
        {.position = {0.0F, 0.0F, 1.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = radius,
         .role = WaterSurfaceRole::Vegetation,
         .lifetimeSeconds = 2.0F,
         .energy = 1.0F,
         .seed = 0x5A17U},
    };
    const auto grid = invisible_places::water::BuildRainImpactGrid(events, {}, 0.65F, 2.0F);

    struct PlaneStats {
        std::uint32_t candidates = 0U;
        std::uint32_t active = 0U;
        std::array<bool, 8> activeSectors{};
        float maximumOpacity = 0.0F;
        float maximumEmission = 0.0F;
        float maximumSize = 1.0F;
    };
    const auto scanPlane = [&](float z, float timeSeconds) {
        PlaneStats stats;
        constexpr int steps = 26;
        for (int y = -steps; y <= steps; ++y) {
            for (int x = -steps; x <= steps; ++x) {
                const float px = radius * static_cast<float>(x) / static_cast<float>(steps);
                const float py = radius * static_cast<float>(y) / static_cast<float>(steps);
                if (std::hypot(px, py) > radius) {
                    continue;
                }
                ++stats.candidates;
                const auto effect = invisible_places::water::EvaluateRainImpact(
                    grid,
                    WaterSurfaceRole::Vegetation,
                    {px, py, z},
                    {0.0F, 0.0F, 1.0F},
                    timeSeconds);
                stats.maximumOpacity = std::max(stats.maximumOpacity, effect.opacity);
                stats.maximumEmission = std::max(stats.maximumEmission, effect.emission);
                stats.maximumSize = std::max(stats.maximumSize, effect.sizeScale);
                if (effect.emission <= 0.002F) {
                    continue;
                }
                ++stats.active;
                float angle = std::atan2(py, px);
                if (angle < 0.0F) {
                    angle += 2.0F * 3.14159265358979323846F;
                }
                const auto sector = std::min<std::size_t>(
                    stats.activeSectors.size() - 1U,
                    static_cast<std::size_t>(angle / (2.0F * 3.14159265358979323846F) *
                                             static_cast<float>(stats.activeSectors.size())));
                stats.activeSectors[sector] = true;
            }
        }
        return stats;
    };

    const auto topEarly = scanPlane(0.96F, 0.10F);
    const auto lowerEarly = scanPlane(0.45F, 0.10F);
    const auto lowerLater = scanPlane(0.45F, 1.05F);
    const auto lowerExpired = scanPlane(0.45F, 2.01F);
    CHECK(topEarly.active > 0U);
    CHECK(topEarly.active < topEarly.candidates / 3U);
    CHECK(lowerEarly.active == 0U);
    CHECK(lowerLater.active > 3U);
    CHECK(std::count(lowerLater.activeSectors.begin(), lowerLater.activeSectors.end(), true) >= 2);
    CHECK(lowerLater.maximumOpacity < lowerLater.maximumEmission);
    CHECK(lowerLater.maximumEmission > 0.20F);
    CHECK(lowerLater.maximumSize > 1.08F);
    CHECK(lowerExpired.active == 0U);
}

TEST_CASE("rain impact lifetimes produce a short sand ring and slow rock fade", "[water][rain][effects]") {
    const std::vector<invisible_places::water::RainImpactEvent> events{
        {.position = {0.0F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = 0.10F,
         .role = WaterSurfaceRole::Sand,
         .lifetimeSeconds = 1.0F,
         .energy = 1.0F},
        {.position = {0.4F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = 0.12F,
         .role = WaterSurfaceRole::Rock,
         .lifetimeSeconds = 5.0F,
         .energy = 1.0F},
    };
    const auto grid = invisible_places::water::BuildRainImpactGrid(events, {}, 0.5F, 4.0F);
    const auto sandMid = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Sand, {0.056F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.5F);
    const auto sandDead = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Sand, {0.10F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 1.1F);
    const auto rockEarly = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Rock, {0.4F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.5F);
    const auto rockLate = invisible_places::water::EvaluateRainImpact(
        grid, WaterSurfaceRole::Rock, {0.4F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 4.8F);
    CHECK(sandMid.opacity > 0.05F);
    CHECK(sandDead.opacity == Catch::Approx(0.0F));
    CHECK(rockEarly.colourBlend > rockLate.colourBlend);
}

TEST_CASE("Scene1 builds and reloads the exact two millimetre water surface cache", "[.rain-data]") {
    const auto sceneDirectory = std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR} / "Scene1";
    const std::vector<invisible_places::water::WaterSurfaceSource> sources{
        {.sourcePath = sceneDirectory / "Site1-ROCK-2mm.ply",
         .role = WaterSurfaceRole::Rock,
         .spacingMicrometres = 2'000U},
        {.sourcePath = sceneDirectory / "Site1-SAND-2mm.ply",
         .role = WaterSurfaceRole::Sand,
         .spacingMicrometres = 2'000U},
        {.sourcePath = sceneDirectory / "Site1-VEG-2mm.ply",
         .role = WaterSurfaceRole::Vegetation,
         .spacingMicrometres = 2'000U},
    };
    for (const auto& source : sources) {
        REQUIRE(std::filesystem::is_regular_file(source.sourcePath));
        CHECK_FALSE(source.isFallback);
    }

    const auto savedDirectory = sceneDirectory.parent_path().parent_path() / "Saved";
    const auto first = invisible_places::water::BuildWaterSurfaceCache(sources, savedDirectory);
    REQUIRE(first.success);
    CHECK(first.cache.bounds.valid);
    CHECK(first.cache.resolutionMeters == Catch::Approx(0.010F));
    CHECK_FALSE(first.cache.surfaceCells.empty());
    CHECK_FALSE(first.cache.vegetationVoxels.empty());
    CHECK(first.cache.sourcePointCount > 0U);
    CHECK(first.cache.sourcePointCount == 39'409'886U);
    CHECK(first.cache.sources.size() == 3U);

    const auto gpu = invisible_places::water::BuildWaterSurfaceGpuData(first.cache);
    CHECK(first.cache.surfaceCells.size() <= static_cast<std::size_t>(gpu.surfaceTable.size() * 0.65F));
    CHECK(first.cache.vegetationVoxels.size() <= static_cast<std::size_t>(gpu.vegetationTable.size() * 0.65F));
    CHECK(gpu.maximumProbeCount <= 32U);

    const auto second = invisible_places::water::BuildWaterSurfaceCache(sources, savedDirectory);
    REQUIRE(second.success);
    CHECK(second.loadedFromDisk);
    CHECK(second.cache.signature == first.cache.signature);
    CHECK(second.cache.surfaceCells.size() == first.cache.surfaceCells.size());
    CHECK(second.cache.vegetationVoxels.size() == first.cache.vegetationVoxels.size());
    CHECK(second.cache.flowSurfaceSurfels.size() == first.cache.flowSurfaceSurfels.size());
    CHECK(second.cache.gpuData.surfaceTable.size() == first.cache.gpuData.surfaceTable.size());
    CHECK(second.cache.gpuData.vegetationTable.size() == first.cache.gpuData.vegetationTable.size());
    CHECK(second.cache.gpuData.flowSurfaceTable.size() == first.cache.gpuData.flowSurfaceTable.size());
}
