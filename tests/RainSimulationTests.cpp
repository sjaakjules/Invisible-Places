#include "io/PointCloudData.hpp"
#include "scene/PointCloudVariants.hpp"
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
#include <string>
#include <vector>

namespace {

using invisible_places::io::Float3;
using invisible_places::water::RainCollisionRole;
using invisible_places::water::RainCollisionSample;

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

std::vector<RainCollisionSample> MakeCollisionSamples() {
    return {
        {{0.01F, 0.01F, 0.20F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Rock},
        {{0.01F, 0.01F, 0.18F}, {0.0F, 0.1F, 0.99F}, RainCollisionRole::Rock},
        {{0.05F, 0.01F, 0.10F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Sand},
        {{0.10F, 0.00F, 0.44F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Vegetation},
        {{0.10F, 0.00F, 0.42F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Vegetation},
    };
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
    const auto path = temporary.path / "rock-5mm.ply";
    WritePointPlyWithRoughness(path, {
        {0.002F, 0.004F, 0.006F, 0.0F, 0.0F, 1.0F, 0.10F},
        {0.010F, 0.012F, 0.014F, 0.0F, 0.0F, 1.0F, 0.30F},
    });
    const std::vector<invisible_places::water::WaterSurfaceSource> sources{{
        .sourcePath = path,
        .role = RainCollisionRole::Rock,
        .spacingMicrometres = 5'000U,
    }};

    const auto result = invisible_places::water::BuildWaterSurfaceCache(sources);
    REQUIRE(result.success);
    CHECK(result.cache.sourcePointCount == 2U);
    REQUIRE(result.cache.flowSurfaceSurfels.size() == 1U);
    CHECK(result.cache.flowSurfaceSurfels[0].roughness == Catch::Approx(0.20F));
    CHECK_FALSE(result.cache.gpuData.flowSurfaceTable.empty());
}

TEST_CASE("shared water cache retains orientation independent role surfels", "[water][rain][flow][cache]") {
    const std::vector<RainCollisionSample> samples{
        {.position = {0.002F, 0.004F, 0.006F},
         .normal = {0.0F, 0.0F, 1.0F},
         .role = RainCollisionRole::Rock,
         .roughness = 0.20F,
         .hasRoughness = true},
        {.position = {0.010F, 0.012F, 0.014F},
         .normal = {0.0F, 0.0F, -1.0F},
         .role = RainCollisionRole::Rock,
         .roughness = 0.40F,
         .hasRoughness = true},
        {.position = {0.006F, 0.006F, 0.010F},
         .normal = {1.0F, 0.0F, 0.0F},
         .role = RainCollisionRole::Sand},
        {.position = {0.006F, 0.006F, 0.046F},
         .normal = {1.0F, 0.0F, 0.0F},
         .role = RainCollisionRole::Rock},
    };

    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    CHECK(cache.schemaVersion == 2U);
    // Rain still retains one top-height cell per XY coordinate and role.
    REQUIRE(cache.surfaceCells.size() == 1U);
    CHECK(cache.surfaceCells[0].rockHeight == Catch::Approx(0.046F));
    // Flow retains both surface sheets and keeps ROCK/SAND separate in a shared voxel.
    REQUIRE(cache.flowSurfaceSurfels.size() == 3U);
    const auto firstRock = std::find_if(
        cache.flowSurfaceSurfels.begin(),
        cache.flowSurfaceSurfels.end(),
        [](const auto& surfel) {
            return surfel.cellZ == 0 && surfel.role == RainCollisionRole::Rock;
        });
    REQUIRE(firstRock != cache.flowSurfaceSurfels.end());
    CHECK(firstRock->centroid.x == Catch::Approx(0.006F));
    CHECK(firstRock->centroid.y == Catch::Approx(0.008F));
    CHECK(firstRock->centroid.z == Catch::Approx(0.010F));
    CHECK(firstRock->normal.z == Catch::Approx(1.0F));
    CHECK(firstRock->normalCoherence == Catch::Approx(1.0F));
    CHECK(firstRock->normalVariance == Catch::Approx(0.0F));
    CHECK(firstRock->roughness == Catch::Approx(0.30F));
    CHECK(firstRock->sampleCount == 2U);
}

TEST_CASE("water surface residency identity distinguishes legacy revision collisions", "[water][rain][flow][cache][identity]") {
    const std::vector<RainCollisionSample> firstSamples{
        {{0.001F, 0.001F, 0.001F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Rock},
        {{0.010F, 0.010F, 0.010F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Rock},
        {{0.039F, 0.039F, 0.039F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Rock},
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
    const std::vector<RainCollisionSample> samples{
        {{0.002F, 0.002F, 0.002F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Rock},
        {{0.006F, 0.006F, 0.006F}, {1.0F, 0.0F, 0.0F}, RainCollisionRole::Rock},
    };
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    REQUIRE(cache.flowSurfaceSurfels.size() == 1U);
    const auto& surfel = cache.flowSurfaceSurfels.front();
    CHECK(surfel.normalCoherence == Catch::Approx(std::sqrt(0.5F)));
    CHECK(surfel.normalVariance == Catch::Approx(1.0F - std::sqrt(0.5F)));
    CHECK(surfel.roughness == Catch::Approx(surfel.normalVariance));
}

TEST_CASE("water surface CPU queries preserve the continuous surface sheet", "[water][rain][flow][cache]") {
    const std::vector<RainCollisionSample> samples{
        // A nearer parallel sheet along the reference normal.
        {{0.0F, 0.0F, 0.021F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Rock},
        // A slightly farther candidate in the reference tangent plane. Its
        // inverted source normal must be hemisphere-aligned to the query frame.
        {{0.025F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}, RainCollisionRole::Rock},
        {{0.001F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Sand},
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
    CHECK(sandOnly.surfel.role == RainCollisionRole::Sand);
    CHECK(sandOnly.distanceMeters == Catch::Approx(0.001F));

    const auto outsideSupport = invisible_places::water::QueryWaterSurfaceCache(
        cache,
        {1.0F, 1.0F, 1.0F},
        0.01F);
    CHECK_FALSE(outsideSupport.hit);
}

TEST_CASE("rain sources prefer exact five millimetre data per role", "[water][rain][cache]") {
    invisible_places::scene::ScenePointCloudGroup group;
    group.variantsByRole[0] = {
        {.role = invisible_places::scene::ScenePointCloudRole::Rock, .spacingMicrometres = 1'000U, .sourcePath = "rock1.ply"},
        {.role = invisible_places::scene::ScenePointCloudRole::Rock, .spacingMicrometres = 5'000U, .sourcePath = "rock5.ply"},
    };
    group.variantsByRole[1] = {
        {.role = invisible_places::scene::ScenePointCloudRole::Sand, .spacingMicrometres = 2'000U, .sourcePath = "sand2.ply"},
    };
    group.variantsByRole[2] = {
        {.role = invisible_places::scene::ScenePointCloudRole::Vegetation, .spacingMicrometres = 1'000U, .sourcePath = "veg1.ply"},
        {.role = invisible_places::scene::ScenePointCloudRole::Vegetation, .spacingMicrometres = 3'000U, .sourcePath = "veg3.ply"},
    };

    const auto sources = invisible_places::water::SelectRainCollisionSources(group);
    REQUIRE(sources.size() == 3U);
    CHECK(sources[0].sourcePath == "rock5.ply");
    CHECK_FALSE(sources[0].isFallback);
    CHECK(sources[1].sourcePath == "sand2.ply");
    CHECK(sources[1].isFallback);
    CHECK(sources[2].sourcePath == "veg3.ply");
    CHECK(sources[2].isFallback);
}

TEST_CASE("rain collision traces top surfaces and vegetation voxels", "[water][rain][cache]") {
    const auto samples = MakeCollisionSamples();
    const auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(samples);

    const auto rockHit = invisible_places::water::TraceRainCollision(
        cache,
        {0.01F, 0.01F, 0.8F},
        {0.01F, 0.01F, -0.2F});
    REQUIRE(rockHit.hit);
    CHECK(rockHit.role == RainCollisionRole::Rock);
    CHECK(rockHit.position.z == Catch::Approx(0.20F).margin(0.001F));

    const auto sandHit = invisible_places::water::TraceRainCollision(
        cache,
        {0.05F, 0.01F, 0.8F},
        {0.05F, 0.01F, -0.2F});
    REQUIRE(sandHit.hit);
    CHECK(sandHit.role == RainCollisionRole::Sand);

    const auto vegetationHit = invisible_places::water::TraceRainCollision(
        cache,
        {0.00F, 0.00F, 0.60F},
        {0.16F, 0.00F, 0.328F});
    REQUIRE(vegetationHit.hit);
    CHECK(vegetationHit.role == RainCollisionRole::Vegetation);
    CHECK(vegetationHit.segmentTime < 1.0F);
}

TEST_CASE("rain collision DDA cannot tunnel through distant vegetation", "[water][rain][cache][dda]") {
    const std::vector<RainCollisionSample> samples{{
        {5.005F, 0.005F, 0.005F},
        {0.0F, 0.0F, 1.0F},
        RainCollisionRole::Vegetation,
    }};
    const auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(samples);
    const auto hit = invisible_places::water::TraceRainCollision(
        cache,
        {0.005F, 0.005F, 10.005F},
        {10.005F, 0.005F, -9.995F});
    REQUIRE(hit.hit);
    CHECK(hit.role == RainCollisionRole::Vegetation);
    CHECK(hit.segmentTime == Catch::Approx(0.50F).margin(0.003F));
}

TEST_CASE("rain collision chooses the first role surface", "[water][rain][cache]") {
    const std::vector<RainCollisionSample> samples{
        {{0.005F, 0.005F, 0.20F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Rock},
        {{0.005F, 0.005F, 0.40F}, {0.0F, 0.0F, 1.0F}, RainCollisionRole::Sand},
    };
    const auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(samples);
    const auto hit = invisible_places::water::TraceRainCollision(
        cache,
        {0.005F, 0.005F, 1.0F},
        {0.005F, 0.005F, -1.0F});
    REQUIRE(hit.hit);
    CHECK(hit.role == RainCollisionRole::Sand);
    CHECK(hit.position.z == Catch::Approx(0.40F));
}

TEST_CASE("rain collision cache persistence rejects stale signatures", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(MakeCollisionSamples());
    cache.signature = "expected";
    const auto expectedIdentity =
        invisible_places::water::BuildWaterSurfaceCacheIdentity(cache);
    const auto path = temporary.path / "test.raincache";
    std::string error;
    REQUIRE(invisible_places::water::SaveRainCollisionCache(cache, path, &error));

    invisible_places::water::RainCollisionCache loaded;
    REQUIRE(invisible_places::water::LoadRainCollisionCache(path, "expected", &loaded, &error));
    CHECK(loaded.schemaVersion == 2U);
    CHECK(loaded.surfaceCells.size() == cache.surfaceCells.size());
    CHECK(loaded.vegetationVoxels.size() == cache.vegetationVoxels.size());
    REQUIRE(loaded.flowSurfaceSurfels.size() == cache.flowSurfaceSurfels.size());
    CHECK(loaded.flowSurfaceSurfels[0].centroid.x ==
          Catch::Approx(cache.flowSurfaceSurfels[0].centroid.x));
    CHECK(loaded.flowSurfaceSurfels[0].normalCoherence ==
          Catch::Approx(cache.flowSurfaceSurfels[0].normalCoherence));
    CHECK(loaded.gpuData.surfaceTable.size() == cache.gpuData.surfaceTable.size());
    CHECK(loaded.gpuData.vegetationTable.size() == cache.gpuData.vegetationTable.size());
    CHECK(loaded.gpuData.flowSurfaceTable.size() == cache.gpuData.flowSurfaceTable.size());
    CHECK(loaded.gpuData.flowSurfaceMask == cache.gpuData.flowSurfaceMask);
    CHECK(loaded.gpuData.sourceRevision == cache.revision);
    CHECK(loaded.cacheIdentity == expectedIdentity);
    CHECK(loaded.gpuData.sourceIdentity == expectedIdentity);
    CHECK_FALSE(invisible_places::water::LoadRainCollisionCache(path, "changed", &loaded, &error));

    // Schema-v2 files written before the identity trailer remain loadable. The
    // exact same identity is deterministically recovered from their payload.
    constexpr std::uintmax_t trailerMagicBytes = 8U;
    const auto trailerBytes = trailerMagicBytes + sizeof(std::uint32_t) +
                              expectedIdentity.sourceSignature.size() +
                              expectedIdentity.contentDigest.size() * sizeof(std::uint64_t);
    const auto savedSize = std::filesystem::file_size(path);
    REQUIRE(savedSize > trailerBytes);
    std::filesystem::resize_file(path, savedSize - trailerBytes);
    REQUIRE(invisible_places::water::LoadRainCollisionCache(path, "expected", &loaded, &error));
    CHECK(loaded.cacheIdentity == expectedIdentity);
    CHECK(loaded.gpuData.sourceIdentity == expectedIdentity);
}

TEST_CASE("rain collision cache persists below the project Saved directory", "[water][rain][cache]") {
    const std::filesystem::path savedDirectory{"Saved"};
    CHECK(
        invisible_places::water::RainCollisionCachePath(savedDirectory, "abc123") ==
        savedDirectory / "cache" / "rain" / "abc123.raincache");
}

TEST_CASE("rain collision signatures include source file identity", "[water][rain][cache]") {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "rock5.ply";
    WritePointPly(path, {{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F}});
    const std::vector<invisible_places::water::RainCollisionSource> sources{{
        .sourcePath = path,
        .role = RainCollisionRole::Rock,
        .spacingMicrometres = 5'000U,
    }};
    const auto before = invisible_places::water::RainCollisionCacheSignature(sources);
    WritePointPly(path, {
        {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.02F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    });
    const auto after = invisible_places::water::RainCollisionCacheSignature(sources);
    CHECK(before != after);
}

TEST_CASE("rain GPU collision tables remain sparse and bounded", "[water][rain][gpu]") {
    auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(MakeCollisionSamples());
    cache.revision = 77U;
    const auto gpu = invisible_places::water::BuildRainCollisionGpuData(cache);

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

    auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(MakeCollisionSamples());
    cache.revision = 19U;
    const auto before = invisible_places::water::BuildRainCollisionGpuData(cache);
    auto settings = invisible_places::water::DefaultRainRuntimeSettings();
    settings.density = 0.12F;
    settings.windSpeedMetersPerSecond = 3.0F;
    settings.opacityScale = 0.2F;
    const auto after = invisible_places::water::BuildRainCollisionGpuData(cache);
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

TEST_CASE("rain simulator is deterministic and skips events when effects are off", "[water][rain][simulation]") {
    std::vector<RainCollisionSample> samples;
    for (int y = -30; y <= 30; ++y) {
        for (int x = -30; x <= 30; ++x) {
            samples.push_back({
                {x * 0.02F, y * 0.02F, 0.0F},
                {0.0F, 0.0F, 1.0F},
                RainCollisionRole::Rock,
            });
        }
    }
    const auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(samples);
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

TEST_CASE("rain simulator respawns particles outside the camera range", "[water][rain][simulation]") {
    const std::vector<RainCollisionSample> samples{{
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        RainCollisionRole::Rock,
    }};
    const auto cache = invisible_places::water::BuildRainCollisionCacheFromSamples(samples);
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
            .role = RainCollisionRole::Rock,
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
        .role = RainCollisionRole::Sand,
        .lifetimeSeconds = 1.0F,
        .energy = 1.0F,
    });
    events.push_back({
        .position = {-0.20F, 0.0F, 1.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = RainCollisionRole::Vegetation,
        .lifetimeSeconds = 2.0F,
        .energy = 1.0F,
        .seed = 4U,
    });

    const auto grid = invisible_places::water::BuildRainImpactGrid(events, {}, 0.25F, 4.0F);
    CHECK(grid.overflowCount > 0U);
    const auto rock = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Rock, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.25F);
    const auto sandAtRock = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Sand, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.25F);
    const auto vegetationLowerEarly = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Vegetation, {-0.20F, 0.0F, 0.50F}, {0.0F, 0.0F, 1.0F}, 0.10F);
    float vegetationTopMaximum = 0.0F;
    for (int y = -20; y <= 20; ++y) {
        for (int x = -20; x <= 20; ++x) {
            const auto vegetationTop = invisible_places::water::EvaluateRainImpact(
                grid,
                RainCollisionRole::Vegetation,
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

TEST_CASE("rock rain impact expands from its contact point", "[water][rain][effects]") {
    const std::vector<invisible_places::water::RainImpactEvent> events{
        {.position = {0.0F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = 0.12F,
         .role = RainCollisionRole::Rock,
         .lifetimeSeconds = 5.0F,
         .energy = 1.0F,
         .seed = 91U},
    };
    const auto grid = invisible_places::water::BuildRainImpactGrid(events, {}, 1.0F, 2.0F);
    const auto centreAtBirth = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Rock, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.08F);
    const auto outerAtBirth = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Rock, {0.09F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.08F);
    const auto outerAfterGrowth = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Rock, {0.09F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.90F);

    CHECK(centreAtBirth.colourBlend > 0.30F);
    CHECK(outerAtBirth.colourBlend < 0.01F);
    CHECK(outerAfterGrowth.colourBlend > 0.30F);
}

TEST_CASE("vegetation rain impacts form sparse wandering downward sparkles", "[water][rain][effects]") {
    constexpr float radius = 0.065F;
    const std::vector<invisible_places::water::RainImpactEvent> events{
        {.position = {0.0F, 0.0F, 1.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = radius,
         .role = RainCollisionRole::Vegetation,
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
                    RainCollisionRole::Vegetation,
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
    const auto lowerLater = scanPlane(0.45F, 0.60F);
    CHECK(topEarly.active > 0U);
    CHECK(topEarly.active < topEarly.candidates / 4U);
    CHECK(lowerEarly.active == 0U);
    CHECK(lowerLater.active > 3U);
    CHECK(std::count(lowerLater.activeSectors.begin(), lowerLater.activeSectors.end(), true) >= 2);
    CHECK(lowerLater.maximumOpacity < lowerLater.maximumEmission);
    CHECK(lowerLater.maximumSize < 1.08F);
}

TEST_CASE("rain impact lifetimes produce a short sand ring and slow rock fade", "[water][rain][effects]") {
    const std::vector<invisible_places::water::RainImpactEvent> events{
        {.position = {0.0F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = 0.10F,
         .role = RainCollisionRole::Sand,
         .lifetimeSeconds = 1.0F,
         .energy = 1.0F},
        {.position = {0.4F, 0.0F, 0.0F},
         .birthTimeSeconds = 0.0F,
         .normal = {0.0F, 0.0F, 1.0F},
         .radiusMeters = 0.12F,
         .role = RainCollisionRole::Rock,
         .lifetimeSeconds = 5.0F,
         .energy = 1.0F},
    };
    const auto grid = invisible_places::water::BuildRainImpactGrid(events, {}, 0.5F, 4.0F);
    const auto sandMid = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Sand, {0.056F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.5F);
    const auto sandDead = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Sand, {0.10F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 1.1F);
    const auto rockEarly = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Rock, {0.4F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.5F);
    const auto rockLate = invisible_places::water::EvaluateRainImpact(
        grid, RainCollisionRole::Rock, {0.4F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 4.8F);
    CHECK(sandMid.opacity > 0.05F);
    CHECK(sandDead.opacity == Catch::Approx(0.0F));
    CHECK(rockEarly.colourBlend > rockLate.colourBlend);
}

TEST_CASE("Scene1 builds and reloads the exact five millimetre rain cache", "[.rain-data]") {
    const auto sceneDirectory = std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR} / "Scene1";
    const std::vector<invisible_places::water::RainCollisionSource> sources{
        {.sourcePath = sceneDirectory / "Site1-ROCK-5mm.ply",
         .role = RainCollisionRole::Rock,
         .spacingMicrometres = 5'000U},
        {.sourcePath = sceneDirectory / "Site1-SAND-5mm.ply",
         .role = RainCollisionRole::Sand,
         .spacingMicrometres = 5'000U},
        {.sourcePath = sceneDirectory / "Site1-VEG-5mm.ply",
         .role = RainCollisionRole::Vegetation,
         .spacingMicrometres = 5'000U},
    };
    for (const auto& source : sources) {
        REQUIRE(std::filesystem::is_regular_file(source.sourcePath));
        CHECK_FALSE(source.isFallback);
    }

    const auto savedDirectory = sceneDirectory.parent_path().parent_path() / "Saved";
    const auto first = invisible_places::water::BuildRainCollisionCache(sources, savedDirectory);
    REQUIRE(first.success);
    CHECK(first.cache.bounds.valid);
    CHECK(first.cache.resolutionMeters == Catch::Approx(0.020F));
    CHECK_FALSE(first.cache.surfaceCells.empty());
    CHECK_FALSE(first.cache.vegetationVoxels.empty());
    CHECK(first.cache.sourcePointCount > 0U);
    CHECK(first.cache.sources.size() == 3U);

    const auto gpu = invisible_places::water::BuildRainCollisionGpuData(first.cache);
    CHECK(first.cache.surfaceCells.size() <= static_cast<std::size_t>(gpu.surfaceTable.size() * 0.65F));
    CHECK(first.cache.vegetationVoxels.size() <= static_cast<std::size_t>(gpu.vegetationTable.size() * 0.65F));
    CHECK(gpu.maximumProbeCount <= 32U);

    const auto second = invisible_places::water::BuildRainCollisionCache(sources, savedDirectory);
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
