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
#include <bit>
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
    if (tables.GroundValid()) {
        readSection(
            tables.groundOffset,
            8U,
            tables.groundCount,
            sizeof(invisible_places::water::WaterGpuGroundSlot));
    }
    return checksum.Finish();
}

template <typename T>
bool WriteTestPod(std::ofstream& output, const T& value) {
    output.write(reinterpret_cast<const char*>(&value), sizeof(value));
    return output.good();
}

bool WriteTestString(std::ofstream& output, std::string_view value) {
    const auto size = static_cast<std::uint32_t>(value.size());
    return WriteTestPod(output, size) &&
           (output.write(
                value.data(),
                static_cast<std::streamsize>(value.size())),
            output.good());
}

template <typename T>
void AddTestChecksumArray(
    invisible_places::water::WaterSurfaceGpuStreamChecksumBuilder* checksum,
    std::uint64_t tag,
    const std::vector<T>& values) {
    checksum->AddPod(tag);
    checksum->AddPod(static_cast<std::uint64_t>(values.size()));
    if (!values.empty()) {
        checksum->AddBytes(values.data(), values.size() * sizeof(T));
    }
}

template <typename T>
bool WriteTestArray(std::ofstream& output, const std::vector<T>& values) {
    if (!values.empty()) {
        output.write(
            reinterpret_cast<const char*>(values.data()),
            static_cast<std::streamsize>(values.size() * sizeof(T)));
    }
    return output.good();
}

invisible_places::water::WaterSurfaceCachePayloadChecksum
Schema3PayloadChecksum(
    const invisible_places::water::WaterSurfaceCache& cache) {
    invisible_places::water::WaterSurfaceGpuStreamChecksumBuilder checksum;
    AddTestChecksumArray(&checksum, 1U, cache.surfaceCells);
    AddTestChecksumArray(&checksum, 2U, cache.vegetationVoxels);
    AddTestChecksumArray(&checksum, 3U, cache.flowSurfaceSurfels);
    AddTestChecksumArray(&checksum, 4U, cache.gpuData.surfaceTable);
    AddTestChecksumArray(&checksum, 5U, cache.gpuData.vegetationTable);
    AddTestChecksumArray(&checksum, 6U, cache.gpuData.flowSurfaceTable);
    return checksum.Finish();
}

void WritePreGroundSurfaceCache(
    const std::filesystem::path& path,
    const invisible_places::water::WaterSurfaceCache& source,
    std::uint32_t schemaVersion) {
    auto cache = source;
    cache.schemaVersion = schemaVersion;
    cache.groundCells.clear();
    cache.groundSourcePointCount = 0U;
    cache.gpuData = {};
    cache.gpuData = invisible_places::water::BuildWaterSurfaceGpuData(cache);
    const auto payloadChecksum = Schema3PayloadChecksum(cache);
    cache.gpuData.payloadChecksum = payloadChecksum;
    cache.cacheIdentity = {};
    cache.gpuData.sourceIdentity = {};
    const auto identity =
        invisible_places::water::BuildWaterSurfaceCacheIdentity(cache);

    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    REQUIRE(output.is_open());
    const std::string_view magic = schemaVersion ==
            invisible_places::water::kWaterSurfaceCachePreviousSchemaVersion
        ? "IPWSC003"
        : "IPWSC002";
    output.write(magic.data(), static_cast<std::streamsize>(magic.size()));
    const auto sourceCount = static_cast<std::uint32_t>(cache.sources.size());
    REQUIRE((
        WriteTestPod(output, schemaVersion) &&
        WriteTestPod(output, cache.resolutionMeters) &&
        WriteTestString(output, cache.signature) &&
        WriteTestPod(output, cache.bounds) &&
        WriteTestPod(output, cache.sourcePointCount) &&
        WriteTestPod(output, cache.revision) &&
        WriteTestPod(output, sourceCount)));
    for (const auto& sourceMetadata : cache.sources) {
        const auto role = static_cast<std::uint32_t>(sourceMetadata.role);
        REQUIRE((
            WriteTestString(
                output,
                sourceMetadata.sourcePath.generic_string()) &&
            WriteTestPod(output, role) &&
            WriteTestPod(output, sourceMetadata.spacingMicrometres) &&
            WriteTestPod(output, sourceMetadata.fileSize) &&
            WriteTestPod(output, sourceMetadata.modificationTicks) &&
            WriteTestPod(output, sourceMetadata.isFallback)));
    }
    const auto surfaceCount =
        static_cast<std::uint64_t>(cache.surfaceCells.size());
    const auto vegetationCount =
        static_cast<std::uint64_t>(cache.vegetationVoxels.size());
    const auto flowCount =
        static_cast<std::uint64_t>(cache.flowSurfaceSurfels.size());
    const auto gpuSurfaceCount =
        static_cast<std::uint64_t>(cache.gpuData.surfaceTable.size());
    const auto gpuVegetationCount =
        static_cast<std::uint64_t>(cache.gpuData.vegetationTable.size());
    const auto gpuFlowCount =
        static_cast<std::uint64_t>(cache.gpuData.flowSurfaceTable.size());
    REQUIRE((
        WriteTestPod(output, surfaceCount) &&
        WriteTestPod(output, vegetationCount) &&
        WriteTestPod(output, flowCount) &&
        WriteTestArray(output, cache.surfaceCells) &&
        WriteTestArray(output, cache.vegetationVoxels) &&
        WriteTestArray(output, cache.flowSurfaceSurfels) &&
        WriteTestPod(output, gpuSurfaceCount) &&
        WriteTestPod(output, gpuVegetationCount) &&
        WriteTestPod(output, gpuFlowCount) &&
        WriteTestPod(output, cache.gpuData.surfaceMask) &&
        WriteTestPod(output, cache.gpuData.vegetationMask) &&
        WriteTestPod(output, cache.gpuData.flowSurfaceMask) &&
        WriteTestPod(output, cache.gpuData.maximumProbeCount) &&
        WriteTestPod(output, cache.gpuData.flowMaximumProbeCount) &&
        WriteTestPod(output, cache.gpuData.sourceRevision) &&
        WriteTestArray(output, cache.gpuData.surfaceTable) &&
        WriteTestArray(output, cache.gpuData.vegetationTable) &&
        WriteTestArray(output, cache.gpuData.flowSurfaceTable)));
    if (schemaVersion ==
        invisible_places::water::kWaterSurfaceCachePreviousSchemaVersion) {
        constexpr std::string_view trailerMagic = "WSCID003";
        output.write(
            trailerMagic.data(),
            static_cast<std::streamsize>(trailerMagic.size()));
        REQUIRE(WriteTestString(output, identity.sourceSignature));
        for (const auto word : identity.contentDigest) {
            REQUIRE(WriteTestPod(output, word));
        }
        for (const auto word : payloadChecksum.words) {
            REQUIRE(WriteTestPod(output, word));
        }
        REQUIRE(WriteTestPod(output, payloadChecksum.hashedByteCount));
    }
    output.flush();
    REQUIRE(output.good());
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

TEST_CASE(
    "sampled Ground retains deterministic connected upper and vegetation-supported cells",
    "[water][cache][ground]") {
    std::vector<WaterSurfaceSample> samples{
        // Authored point totals remain separate from the sampled Ground total.
        {{0.001F, 0.001F, 0.100F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.031F, 0.001F, 0.100F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.101F, 0.001F, 0.100F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Rock},
        {{0.201F, 0.001F, 0.301F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Vegetation},

        // Only the highest occupied Z cell at XY=(0,0) contributes. Its two
        // opposing scanner normals hemisphere-align to the same slope.
        {{0.001F, 0.001F, 0.145F}, {0.6F, 0.0F, 0.8F}, WaterSurfaceRole::Ground},
        {{0.002F, 0.002F, 0.149F}, {-0.6F, 0.0F, -0.8F}, WaterSurfaceRole::Ground},
        {{0.003F, 0.003F, 0.115F}, {-0.8F, 0.0F, 0.6F}, WaterSurfaceRole::Ground},
        // No authored terrain is required once a cell is connected to an
        // upper/vegetation-supported component seed.
        {{0.011F, 0.001F, 0.140F}, {0.6F, 0.0F, 0.8F}, WaterSurfaceRole::Ground},
        {{0.021F, 0.001F, 0.140F}, {0.6F, 0.0F, 0.8F}, WaterSurfaceRole::Ground},
        // More than 20 mm below authored terrain is rejected.
        {{0.031F, 0.001F, 0.050F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Ground},
        // A disconnected terminal-only component has no upstream seed.
        {{0.101F, 0.001F, 0.110F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Ground},
        // A disconnected vegetation-supported sample is retained.
        {{0.201F, 0.001F, 0.301F}, {0.0F, 0.0F, 1.0F}, WaterSurfaceRole::Ground},
    };

    const auto cache =
        invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    auto reversedSamples = samples;
    std::reverse(reversedSamples.begin(), reversedSamples.end());
    const auto reversed =
        invisible_places::water::BuildWaterSurfaceCacheFromSamples(
            reversedSamples);

    CHECK(cache.sourcePointCount == 4U);
    CHECK(cache.groundSourcePointCount == 8U);
    REQUIRE(cache.groundCells.size() == 4U);
    REQUIRE(reversed.groundCells.size() == cache.groundCells.size());
    for (std::size_t index = 0U; index < cache.groundCells.size(); ++index) {
        const auto& cell = cache.groundCells[index];
        const auto& reverseCell = reversed.groundCells[index];
        CHECK(cell.cellX == reverseCell.cellX);
        CHECK(cell.cellY == reverseCell.cellY);
        CHECK(cell.height == Catch::Approx(reverseCell.height));
        CHECK(cell.normal.x == Catch::Approx(reverseCell.normal.x));
        CHECK(cell.normal.y == Catch::Approx(reverseCell.normal.y));
        CHECK(cell.normal.z == Catch::Approx(reverseCell.normal.z));
        CHECK(cell.downhill.x == Catch::Approx(reverseCell.downhill.x));
        CHECK(cell.downhill.y == Catch::Approx(reverseCell.downhill.y));
        CHECK(cell.downhill.z == Catch::Approx(reverseCell.downhill.z));
        CHECK(cell.flags == reverseCell.flags);
        CHECK(cell.componentId == reverseCell.componentId);
        CHECK(cell.connectivityMask == reverseCell.connectivityMask);
    }

    const auto& upper = cache.groundCells[0];
    CHECK(upper.cellX == 0);
    CHECK(upper.height == Catch::Approx(0.149F));
    CHECK(upper.sampleCount == 2U);
    CHECK((upper.flags & invisible_places::water::kWaterGroundUpperFlag) != 0U);
    CHECK(upper.normal.x == Catch::Approx(0.6F).margin(1.0e-5F));
    CHECK(upper.normal.z == Catch::Approx(0.8F).margin(1.0e-5F));
    CHECK(upper.downhill.x == Catch::Approx(0.8F).margin(1.0e-5F));
    CHECK(upper.downhill.z == Catch::Approx(-0.6F).margin(1.0e-5F));
    CHECK(upper.connectivityMask == (1U << 2U));
    CHECK(cache.groundCells[1].connectivityMask ==
          ((1U << 2U) | (1U << 6U)));
    CHECK(cache.groundCells[2].connectivityMask == (1U << 6U));
    CHECK(
        (cache.groundCells[3].flags &
         invisible_places::water::kWaterGroundVegetationSupportedFlag) != 0U);
    CHECK(cache.groundCells[3].componentId != upper.componentId);

    const auto query = invisible_places::water::QueryWaterGroundCache(
        cache,
        {0.015F, 0.005F, 0.140F},
        0.012F);
    REQUIRE(query.hit);
    CHECK(query.cell.cellX == 1);
    CHECK(query.distanceMeters <= 0.012F);
    CHECK_FALSE(invisible_places::water::QueryWaterGroundCache(
                    cache,
                    {1.0F, 1.0F, 1.0F},
                    0.01F)
                    .hit);
}

TEST_CASE(
    "sampled Ground associates elevated VEG support by bounded vertical column",
    "[water][cache][ground][vegetation]") {
    constexpr float maximumAssociation = invisible_places::water::
        kWaterGroundVegetationAssociationMaximumHeightMeters;
    std::vector<WaterSurfaceSample> samples{
        // Authored terrain retains this upper two-cell Ground component. A
        // canopy point two metres above marks only its own retained column.
        {{0.001F, 0.001F, 2.001F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Vegetation},
        {{0.001F, 0.001F, -0.050F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Rock},
        {{0.001F, 0.001F, 0.001F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Ground},
        {{0.011F, 0.001F, 0.001F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Ground},

        // A separate authored upper column just inside the vertical cap is
        // independently flagged and receives a different component identity.
        {{0.201F, 0.001F, 0.501F + maximumAssociation - 0.010F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Vegetation},
        {{0.201F, 0.001F, 0.450F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Rock},
        {{0.201F, 0.001F, 0.501F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Ground},

        // Unrelated vertically stacked and below-Ground VEG must not seed
        // their disconnected Ground candidates.
        {{0.401F, 0.001F, maximumAssociation + 0.011F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Vegetation},
        {{0.401F, 0.001F, 0.001F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Ground},
        {{0.601F, 0.001F, 0.901F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Vegetation},
        {{0.601F, 0.001F, 1.001F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Ground},

        // Elevated association enriches retained topology only. It must not
        // admit a new component that lacks the legacy upper/near-VEG seed.
        {{0.801F, 0.001F, 2.001F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Vegetation},
        {{0.801F, 0.001F, 0.001F},
         {0.0F, 0.0F, 1.0F},
         WaterSurfaceRole::Ground},
    };

    const auto cache =
        invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    auto reversedSamples = samples;
    std::reverse(reversedSamples.begin(), reversedSamples.end());
    const auto reversed =
        invisible_places::water::BuildWaterSurfaceCacheFromSamples(
            reversedSamples);

    REQUIRE(cache.groundCells.size() == 3U);
    REQUIRE(reversed.groundCells.size() == cache.groundCells.size());
    for (std::size_t index = 0U; index < cache.groundCells.size(); ++index) {
        CHECK(cache.groundCells[index].cellX ==
              reversed.groundCells[index].cellX);
        CHECK(cache.groundCells[index].cellY ==
              reversed.groundCells[index].cellY);
        CHECK(cache.groundCells[index].flags ==
              reversed.groundCells[index].flags);
        CHECK(cache.groundCells[index].componentId ==
              reversed.groundCells[index].componentId);
        CHECK(cache.groundCells[index].connectivityMask ==
              reversed.groundCells[index].connectivityMask);
    }

    const auto& canopyColumn = cache.groundCells[0];
    const auto& connectedNeighbour = cache.groundCells[1];
    const auto& cappedColumn = cache.groundCells[2];
    CHECK(canopyColumn.cellX == 0);
    CHECK(
        (canopyColumn.flags &
         invisible_places::water::kWaterGroundVegetationSupportedFlag) != 0U);
    CHECK(
        (connectedNeighbour.flags &
         invisible_places::water::kWaterGroundVegetationSupportedFlag) == 0U);
    CHECK(canopyColumn.componentId == connectedNeighbour.componentId);
    CHECK(canopyColumn.connectivityMask == (1U << 2U));
    CHECK(connectedNeighbour.connectivityMask == (1U << 6U));
    CHECK(cappedColumn.cellX == 20);
    CHECK(
        (cappedColumn.flags &
         invisible_places::water::kWaterGroundVegetationSupportedFlag) != 0U);
    CHECK(cappedColumn.componentId != canopyColumn.componentId);

    CHECK(std::none_of(
        cache.groundCells.begin(),
        cache.groundCells.end(),
        [](const auto& cell) {
            return cell.cellX == 40 ||
                   cell.cellX == 60 ||
                   cell.cellX == 80;
        }));
}

TEST_CASE(
    "sampled Ground convergence and GPU hash ABI are bounded",
    "[water][cache][ground][gpu]") {
    std::vector<WaterSurfaceSample> samples{{
        {0.001F, 0.001F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        WaterSurfaceRole::Rock,
    }};
    for (std::int32_t y = -1; y <= 1; ++y) {
        for (std::int32_t x = -1; x <= 1; ++x) {
            const float horizontalLength =
                std::sqrt(static_cast<float>(x * x + y * y));
            const Float3 normal = horizontalLength > 0.0F
                ? Float3{
                      -static_cast<float>(x) / horizontalLength * 0.6F,
                      -static_cast<float>(y) / horizontalLength * 0.6F,
                      0.8F,
                  }
                : Float3{0.0F, 0.0F, 1.0F};
            samples.push_back({
                {
                    (static_cast<float>(x) + 0.5F) * 0.010F,
                    (static_cast<float>(y) + 0.5F) * 0.010F,
                    0.100F,
                },
                normal,
                WaterSurfaceRole::Ground,
            });
        }
    }
    const auto cache =
        invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    REQUIRE(cache.groundCells.size() == 9U);
    const auto centre = std::find_if(
        cache.groundCells.begin(),
        cache.groundCells.end(),
        [](const auto& cell) {
            return cell.cellX == 0 && cell.cellY == 0;
        });
    REQUIRE(centre != cache.groundCells.end());
    CHECK(centre->convergence > 0.45F);
    CHECK(centre->connectivityMask == 0xFFU);

    const auto gpu =
        invisible_places::water::BuildWaterSurfaceGpuData(cache);
    static_assert(
        sizeof(invisible_places::water::WaterGpuGroundSlot) == 32U);
    REQUIRE_FALSE(gpu.groundTable.empty());
    CHECK(std::has_single_bit(gpu.groundTable.size()));
    CHECK(gpu.groundMask == gpu.groundTable.size() - 1U);
    CHECK(gpu.groundMaximumProbeCount <= 32U);
    const auto populated = std::count_if(
        gpu.groundTable.begin(),
        gpu.groundTable.end(),
        [](const auto& slot) {
            return slot.cellX != std::numeric_limits<std::int32_t>::min();
        });
    CHECK(
        populated ==
        static_cast<std::ptrdiff_t>(cache.groundCells.size()));
    const auto packedCentre = std::find_if(
        gpu.groundTable.begin(),
        gpu.groundTable.end(),
        [](const auto& slot) {
            return slot.cellX == 0 && slot.cellY == 0;
        });
    REQUIRE(packedCentre != gpu.groundTable.end());
    CHECK((packedCentre->componentAndConnectivity & 0xFFU) == 0xFFU);
    CHECK((packedCentre->flagsAndSampleCount >> 8U) == 1U);
    CHECK(packedCentre->packedConvergenceConfidence != 0U);
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
          "Site1-ROCK-3mm. SampleScene.ply");
    CHECK(displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Sand).sourcePath.filename() ==
          "Site1-SAND-3mm. SampleScene.ply");
    CHECK(displayBundle->Find(invisible_places::scene::ScenePointCloudRole::Vegetation).sourcePath.filename() ==
          "Site1-VEG-3mm. SampleScene.ply");

    const auto sources = invisible_places::water::SelectWaterSurfaceSources(*sampleScene);
    REQUIRE(sources.size() == 3U);
    CHECK(sources[0].sourcePath.filename() ==
          "Site1-ROCK-2mm. SampleScene.ply");
    CHECK(sources[1].sourcePath.filename() ==
          "Site1-SAND-2mm. SampleScene.ply");
    CHECK(sources[2].sourcePath.filename() ==
          "Site1-VEG-2mm. SampleScene.ply");
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
    auto samples = MakeCollisionSamples();
    samples.push_back({
        {0.011F, 0.011F, 0.25F},
        {0.35F, 0.0F, 0.94F},
        WaterSurfaceRole::Ground,
    });
    auto cache =
        invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
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
    REQUIRE(loaded.groundCells.size() == cache.groundCells.size());
    CHECK(loaded.sourcePointCount == cache.sourcePointCount);
    CHECK(loaded.groundSourcePointCount == cache.groundSourcePointCount);
    CHECK(loaded.flowSurfaceSurfels[0].centroid.x ==
          Catch::Approx(cache.flowSurfaceSurfels[0].centroid.x));
    CHECK(loaded.flowSurfaceSurfels[0].normalCoherence ==
          Catch::Approx(cache.flowSurfaceSurfels[0].normalCoherence));
    CHECK(loaded.gpuData.surfaceTable.empty());
    CHECK(loaded.gpuData.vegetationTable.empty());
    CHECK(loaded.gpuData.flowSurfaceTable.empty());
    CHECK(loaded.gpuData.groundTable.empty());
    CAPTURE(
        loaded.gpuData.persistedTables.filePath,
        loaded.gpuData.persistedTables.fileSize,
        loaded.gpuData.persistedTables.surfaceCount,
        loaded.gpuData.persistedTables.vegetationCount,
        loaded.gpuData.persistedTables.flowSurfaceCount,
        loaded.gpuData.persistedTables.groundCount,
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
    CHECK(loaded.gpuData.persistedTables.groundCount ==
          cache.gpuData.groundTable.size());
    CHECK(loaded.gpuData.persistedTables.surfaceOffset <
          loaded.gpuData.persistedTables.vegetationOffset);
    CHECK(loaded.gpuData.persistedTables.vegetationOffset <
          loaded.gpuData.persistedTables.flowSurfaceOffset);
    CHECK(loaded.gpuData.persistedTables.flowSurfaceOffset <
          loaded.gpuData.persistedTables.groundOffset);
    CHECK(loaded.gpuData.persistedTables.GroundValid());
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

    // Schema 4 requires its checksum trailer; truncation cannot silently turn a
    // current cache into a trusted warm-load payload.
    const auto truncatedPath = temporary.path / "truncated.surfacecache";
    REQUIRE(std::filesystem::copy_file(path, truncatedPath));
    constexpr std::uintmax_t schema4TrailerBytes =
        8U + sizeof(std::uint32_t) + 8U + 4U * sizeof(std::uint64_t) +
        3U * sizeof(std::uint64_t);
    const auto savedSize = std::filesystem::file_size(truncatedPath);
    REQUIRE(savedSize > schema4TrailerBytes);

    const auto corruptedPath = temporary.path / "corrupted.surfacecache";
    REQUIRE(std::filesystem::copy_file(path, corruptedPath));
    {
        std::fstream corrupted{
            corruptedPath,
            std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(corrupted.is_open());
        const auto payloadByte = static_cast<std::streamoff>(
            savedSize - schema4TrailerBytes - 1U);
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

    std::filesystem::resize_file(truncatedPath, savedSize - schema4TrailerBytes);
    CHECK_FALSE(invisible_places::water::LoadWaterSurfaceCache(
        truncatedPath,
        "expected",
        &loaded,
        &error));

    // Schema 3 remains recoverable through its validated streaming reader, but
    // has no Ground payload and therefore cannot alias a schema-4 sidecar.
    const auto previousPath = temporary.path / "previous.surfacecache";
    WritePreGroundSurfaceCache(
        previousPath,
        cache,
        invisible_places::water::kWaterSurfaceCachePreviousSchemaVersion);
    invisible_places::water::WaterSurfaceBuildDiagnostics previousDiagnostics;
    REQUIRE(invisible_places::water::LoadWaterSurfaceCache(
        previousPath,
        "expected",
        &loaded,
        &error,
        &previousDiagnostics));
    CHECK(
        loaded.schemaVersion ==
        invisible_places::water::kWaterSurfaceCachePreviousSchemaVersion);
    CHECK(loaded.groundCells.empty());
    CHECK(loaded.groundSourcePointCount == 0U);
    CHECK(loaded.cacheIdentity.Valid());
    CHECK(loaded.gpuData.persistedTables.Valid());
    CHECK_FALSE(loaded.gpuData.persistedTables.GroundValid());
    CHECK(previousDiagnostics.fullPayloadHashPassCount == 0U);

    // Schema-2 files without the optional identity trailer remain directly
    // readable as one-time recovery input.
    const auto legacyPath = temporary.path / "legacy.raincache";
    WritePreGroundSurfaceCache(
        legacyPath,
        cache,
        invisible_places::water::kWaterSurfaceCacheLegacySchemaVersion);
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
    const std::array<std::pair<std::string_view, WaterSurfaceRole>, 4> files{{
        {"rock-2mm.ply", WaterSurfaceRole::Rock},
        {"sand-2mm.ply", WaterSurfaceRole::Sand},
        {"vegetation-2mm.ply", WaterSurfaceRole::Vegetation},
        {"ground-5mm.ply", WaterSurfaceRole::Ground},
    }};
    std::vector<invisible_places::water::WaterSurfaceSource> sources;
    for (std::size_t index = 0U; index < files.size(); ++index) {
        const auto path = sceneDirectory / files[index].first;
        const bool isGround = files[index].second == WaterSurfaceRole::Ground;
        WritePointPly(path, {{
            isGround ? 0.0F : static_cast<float>(index) * 0.02F,
            0.0F,
            isGround ? 0.10F : static_cast<float>(index) * 0.01F,
            0.0F,
            0.0F,
            1.0F,
        }});
        sources.push_back({
            .sourcePath = path,
            .role = files[index].second,
            .spacingMicrometres =
                isGround
                    ? 5'000U
                    : 2'000U,
        });
    }

    const auto cold = invisible_places::water::BuildWaterSurfaceCache(
        sources,
        sceneDirectory);
    REQUIRE(cold.success);
    CHECK_FALSE(cold.loadedFromDisk);
    CHECK(cold.cache.resolutionMeters == Catch::Approx(0.010F));
    REQUIRE(cold.cache.sources.size() == 4U);
    for (const auto& source : cold.cache.sources) {
        CHECK(
            source.spacingMicrometres ==
            (source.role == WaterSurfaceRole::Ground ? 5'000U : 2'000U));
    }
    CHECK(cold.cache.sourcePointCount == 3U);
    CHECK(cold.cache.groundSourcePointCount == 1U);
    CHECK(cold.diagnostics.sourceScanCount == 4U);
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
    CHECK(warm.cache.gpuData.groundTable.empty());
    CHECK(warm.cache.gpuData.persistedTables.GroundValid());
    REQUIRE(warm.cache.sources.size() == cold.cache.sources.size());
    for (const auto& source : warm.cache.sources) {
        CHECK(
            source.spacingMicrometres ==
            (source.role == WaterSurfaceRole::Ground ? 5'000U : 2'000U));
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
          std::string_view{"water-surface-10mm-normal-average-ground-v3"});
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(sources, 0.020F) != baseline);

    auto changedRole = sources;
    changedRole.front().role = WaterSurfaceRole::Sand;
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(changedRole) != baseline);

    auto changedSpacing = sources;
    changedSpacing.front().spacingMicrometres = 3'000U;
    CHECK(invisible_places::water::WaterSurfaceCacheSignature(changedSpacing) != baseline);

    const auto groundPath =
        sourcePath.parent_path() / "mesh-sampled-5mm.ply";
    WritePointPly(
        groundPath,
        {{0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 1.0F}});
    auto withGround = sources;
    withGround.push_back({
        .sourcePath = groundPath,
        .role = WaterSurfaceRole::Ground,
        .spacingMicrometres =
            invisible_places::water::kWaterGroundSourceSpacingMicrometres,
    });
    const auto withGroundSignature =
        invisible_places::water::WaterSurfaceCacheSignature(withGround);
    CHECK(withGroundSignature != baseline);
    auto changedGroundSpacing = withGround;
    changedGroundSpacing.back().spacingMicrometres = 10'000U;
    CHECK(
        invisible_places::water::WaterSurfaceCacheSignature(
            changedGroundSpacing) != withGroundSignature);

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
    for (std::uint32_t index = 0; index < 20U; ++index) {
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

TEST_CASE(
    "ROCK impact reservoir compacts event indices that share a modulo lane",
    "[water][rain][effects][grid]") {
    std::vector<invisible_places::water::RainImpactEvent> events(17U);
    const invisible_places::water::RainImpactEvent impact{
        .position = {0.0F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = WaterSurfaceRole::Rock,
        .lifetimeSeconds = 5.0F,
        .energy = 1.0F,
        .seed = 3U,
    };
    events[0] = impact;
    events[16] = impact;
    events[16].position.x = 0.005F;
    events[16].seed = 19U;

    const auto grid = invisible_places::water::BuildRainImpactGrid(
        events,
        {},
        0.5F,
        4.0F);
    const auto cellX = static_cast<std::uint32_t>(
        std::floor((0.0F - grid.origin.x) / grid.cellSizeMeters));
    const auto cellY = static_cast<std::uint32_t>(
        std::floor((0.0F - grid.origin.y) / grid.cellSizeMeters));
    const auto& cell = grid.cells[
        static_cast<std::size_t>(cellY) * grid.dimension + cellX];

    CHECK(grid.overflowCount == 0U);
    CHECK(cell.rockCount == 2U);
    CHECK(cell.rockMask == 0x3U);
    CHECK(std::find(cell.rock.begin(), cell.rock.end(), 0U) !=
          cell.rock.end());
    CHECK(std::find(cell.rock.begin(), cell.rock.end(), 16U) !=
          cell.rock.end());
}

TEST_CASE(
    "SAND impact storage keeps sparse event indices until capacity is full",
    "[water][rain][effects][grid][sand]") {
    std::vector<invisible_places::water::RainImpactEvent> events(9U);
    const invisible_places::water::RainImpactEvent impact{
        .position = {0.0F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = WaterSurfaceRole::Sand,
        .lifetimeSeconds = 1.0F,
        .energy = 1.0F,
        .seed = 3U,
    };
    events[0] = impact;
    events[8] = impact;
    events[8].position.x = 0.005F;
    events[8].seed = 11U;

    const auto grid = invisible_places::water::BuildRainImpactGrid(
        events,
        {},
        0.25F,
        4.0F);
    const auto cellX = static_cast<std::uint32_t>(
        std::floor((0.0F - grid.origin.x) / grid.cellSizeMeters));
    const auto cellY = static_cast<std::uint32_t>(
        std::floor((0.0F - grid.origin.y) / grid.cellSizeMeters));
    const auto& cell = grid.cells[
        static_cast<std::size_t>(cellY) * grid.dimension + cellX];

    CHECK(grid.overflowCount == 0U);
    CHECK(cell.sandCount == 2U);
    CHECK(cell.sandMask == 0x3U);
    CHECK(std::find(cell.sand.begin(), cell.sand.end(), 0U) !=
          cell.sand.end());
    CHECK(std::find(cell.sand.begin(), cell.sand.end(), 8U) !=
          cell.sand.end());
}

TEST_CASE(
    "VEG impact storage keeps sparse event indices until capacity is full",
    "[water][rain][effects][grid][vegetation]") {
    std::vector<invisible_places::water::RainImpactEvent> events(5U);
    const invisible_places::water::RainImpactEvent impact{
        .position = {0.0F, 0.0F, 0.5F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = WaterSurfaceRole::Vegetation,
        .lifetimeSeconds = 2.0F,
        .energy = 1.0F,
        .seed = 5U,
    };
    events[0] = impact;
    events[4] = impact;
    events[4].position.x = 0.005F;
    events[4].seed = 9U;

    const auto grid = invisible_places::water::BuildRainImpactGrid(
        events,
        {},
        0.25F,
        4.0F);
    const auto cellX = static_cast<std::uint32_t>(
        std::floor((0.0F - grid.origin.x) / grid.cellSizeMeters));
    const auto cellY = static_cast<std::uint32_t>(
        std::floor((0.0F - grid.origin.y) / grid.cellSizeMeters));
    const auto& cell = grid.cells[
        static_cast<std::size_t>(cellY) * grid.dimension + cellX];

    CHECK(grid.overflowCount == 0U);
    CHECK(cell.vegetationCount == 2U);
    CHECK(cell.vegetationMask == 0x3U);
    CHECK(
        std::find(cell.vegetation.begin(), cell.vegetation.end(), 0U) !=
        cell.vegetation.end());
    CHECK(
        std::find(cell.vegetation.begin(), cell.vegetation.end(), 4U) !=
        cell.vegetation.end());
}

TEST_CASE(
    "ROCK impact reservoir ranks physical distance then age energy and index",
    "[water][rain][effects][grid][reservoir]") {
    using invisible_places::water::RainImpactEvent;
    using invisible_places::water::WaterSurfaceRole;

    constexpr float kWorldSpanMeters = 4.0F;
    constexpr float kCellSizeMeters =
        kWorldSpanMeters /
        static_cast<float>(
            invisible_places::water::kRainImpactGridDimension);
    constexpr float kCellMinimum = 0.0F;
    constexpr float kCellMaximum = kCellMinimum + kCellSizeMeters;
    constexpr float kCellCentre =
        (kCellMinimum + kCellMaximum) * 0.5F;
    constexpr float kTimeSeconds = 5.0F;

    const RainImpactEvent base{
        .position = {kCellCentre, kCellCentre, 0.0F},
        .birthTimeSeconds = kTimeSeconds,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.16F,
        .role = WaterSurfaceRole::Rock,
        .lifetimeSeconds = 6.0F,
        .energy = 2.5F,
        .seed = 1U,
    };
    const auto targetCell = [](const auto& grid) -> const auto& {
        const auto cellX = static_cast<std::uint32_t>(std::floor(
            (kCellCentre - grid.origin.x) / grid.cellSizeMeters));
        const auto cellY = static_cast<std::uint32_t>(std::floor(
            (kCellCentre - grid.origin.y) / grid.cellSizeMeters));
        return grid.cells[
            static_cast<std::size_t>(cellY) * grid.dimension + cellX];
    };
    const auto contains = [](const auto& cell, std::uint32_t eventIndex) {
        return std::find(
                   cell.rock.begin(),
                   cell.rock.begin() + cell.rockCount,
                   eventIndex) !=
               cell.rock.begin() + cell.rockCount;
    };
    const auto makeDominantEvents = [&]() {
        return std::vector<RainImpactEvent>(15U, base);
    };
    const auto build = [&](std::span<const RainImpactEvent> events) {
        return invisible_places::water::BuildRainImpactGrid(
            events,
            {},
            kTimeSeconds,
            kWorldSpanMeters);
    };

    SECTION("physical cell-bound distance is independent of drop radius") {
        auto events = makeDominantEvents();
        auto nearerSmall = base;
        nearerSmall.position.x = kCellMaximum + 0.020F;
        nearerSmall.radiusMeters = 0.025F;
        nearerSmall.seed = 15U;
        auto fartherLarge = base;
        fartherLarge.position.x = kCellMaximum + 0.030F;
        fartherLarge.radiusMeters = 0.16F;
        fartherLarge.seed = 16U;
        events.push_back(nearerSmall);
        events.push_back(fartherLarge);

        const auto grid = build(events);
        const auto& cell = targetCell(grid);
        REQUIRE(cell.rockCount == 16U);
        CHECK(cell.rockMask == 0xFFFFU);
        CHECK(contains(cell, 15U));
        CHECK_FALSE(contains(cell, 16U));
        CHECK(grid.overflowCount > 0U);
    }

    SECTION("absolute age is independent of event lifetime") {
        auto events = makeDominantEvents();
        auto youngerShortLived = base;
        youngerShortLived.birthTimeSeconds = kTimeSeconds - 1.0F;
        youngerShortLived.lifetimeSeconds = 1.2F;
        youngerShortLived.seed = 15U;
        auto olderLongLived = base;
        olderLongLived.birthTimeSeconds = kTimeSeconds - 2.0F;
        olderLongLived.lifetimeSeconds = 6.0F;
        olderLongLived.seed = 16U;
        events.push_back(youngerShortLived);
        events.push_back(olderLongLived);

        const auto grid = build(events);
        const auto& cell = targetCell(grid);
        REQUIRE(cell.rockCount == 16U);
        CHECK(contains(cell, 15U));
        CHECK_FALSE(contains(cell, 16U));
    }

    SECTION("higher energy wins after equal distance and age") {
        auto events = makeDominantEvents();
        auto higherEnergy = base;
        higherEnergy.energy = 2.0F;
        higherEnergy.seed = 15U;
        auto lowerEnergy = base;
        lowerEnergy.energy = 0.2F;
        lowerEnergy.seed = 16U;
        events.push_back(higherEnergy);
        events.push_back(lowerEnergy);

        const auto grid = build(events);
        const auto& cell = targetCell(grid);
        REQUIRE(cell.rockCount == 16U);
        CHECK(contains(cell, 15U));
        CHECK_FALSE(contains(cell, 16U));
    }

    SECTION("event index is the final deterministic tie break") {
        std::vector<RainImpactEvent> events(17U, base);
        const auto grid = build(events);
        const auto& cell = targetCell(grid);
        REQUIRE(cell.rockCount == 16U);
        CHECK(contains(cell, 0U));
        CHECK(contains(cell, 15U));
        CHECK_FALSE(contains(cell, 16U));
    }

    SECTION("overflow begins only after all sixteen ROCK slots are occupied") {
        std::vector<RainImpactEvent> sixteen(16U, base);
        const auto settled = build(sixteen);
        CHECK(targetCell(settled).rockCount == 16U);
        CHECK(settled.overflowCount == 0U);

        sixteen.push_back(base);
        const auto saturated = build(sixteen);
        CHECK(targetCell(saturated).rockCount == 16U);
        CHECK(saturated.overflowCount > 0U);
    }
}

TEST_CASE(
    "overlapping ROCK impacts form an order-independent peak-preserving soft union",
    "[water][rain][effects][rock]") {
    auto settings = invisible_places::water::RainRockImpactSettings{};
    settings.edgeBreakup = 0.0F;
    settings.spreadSpeed = 6.0F;
    settings.centreFalloff = 1.0F;
    settings.heightBias = 0.0F;
    const invisible_places::water::RainImpactEvent first{
        .position = {-0.010F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.12F,
        .role = WaterSurfaceRole::Rock,
        .lifetimeSeconds = 5.0F,
        .energy = 0.55F,
        .seed = 7U,
    };
    auto second = first;
    second.position.x = 0.010F;
    second.seed = 11U;
    const std::vector<invisible_places::water::RainImpactEvent> firstOnly{
        first};
    const std::vector<invisible_places::water::RainImpactEvent> both{
        first,
        second};
    const std::vector<invisible_places::water::RainImpactEvent> reversed{
        second,
        first};
    const auto firstGrid = invisible_places::water::BuildRainImpactGrid(
        firstOnly,
        {},
        1.0F,
        4.0F,
        settings);
    const auto bothGrid = invisible_places::water::BuildRainImpactGrid(
        both,
        {},
        1.0F,
        4.0F,
        settings);
    const auto reversedGrid = invisible_places::water::BuildRainImpactGrid(
        reversed,
        {},
        1.0F,
        4.0F,
        settings);
    const Float3 point{0.0F, 0.0F, 0.0F};
    const Float3 normal{0.0F, 0.0F, 1.0F};
    const auto single = invisible_places::water::EvaluateRainImpact(
        firstGrid,
        WaterSurfaceRole::Rock,
        point,
        normal,
        1.0F);
    const auto combined = invisible_places::water::EvaluateRainImpact(
        bothGrid,
        WaterSurfaceRole::Rock,
        point,
        normal,
        1.0F);
    const auto combinedReversed = invisible_places::water::EvaluateRainImpact(
        reversedGrid,
        WaterSurfaceRole::Rock,
        point,
        normal,
        1.0F);
    const float firstValue =
        invisible_places::water::EvaluateRockRainImpactValue(
            first,
            point,
            normal,
            1.0F,
            settings) *
        first.energy;
    const float secondValue =
        invisible_places::water::EvaluateRockRainImpactValue(
            second,
            point,
            normal,
            1.0F,
            settings) *
        second.energy;
    const float expectedUnion = std::max(
        std::max(firstValue, secondValue),
        1.0F -
            (1.0F - std::clamp(firstValue, 0.0F, 1.0F)) *
                (1.0F - std::clamp(secondValue, 0.0F, 1.0F)));

    CHECK(single.opacity == Catch::Approx(firstValue * 0.18F));
    CHECK(single.emission == Catch::Approx(firstValue * 0.11F));
    CHECK(single.sizeScale == Catch::Approx(1.0F + firstValue * 0.16F));
    CHECK(single.colourBlend == Catch::Approx(firstValue * 0.42F));
    CHECK(combined.opacity == Catch::Approx(expectedUnion * 0.18F));
    CHECK(combined.emission == Catch::Approx(expectedUnion * 0.11F));
    CHECK(combined.sizeScale ==
          Catch::Approx(1.0F + expectedUnion * 0.16F));
    CHECK(combined.colourBlend ==
          Catch::Approx(expectedUnion * 0.42F));
    CHECK(combined.opacity > single.opacity);
    CHECK(combined.emission > single.emission);
    CHECK(combined.sizeScale > single.sizeScale);
    CHECK(combined.colourBlend > single.colourBlend);
    CHECK(combined.opacity ==
          Catch::Approx(combinedReversed.opacity).margin(1.0e-6F));
    CHECK(combined.emission ==
          Catch::Approx(combinedReversed.emission).margin(1.0e-6F));
    CHECK(combined.sizeScale ==
          Catch::Approx(combinedReversed.sizeScale).margin(1.0e-6F));
    CHECK(combined.colourBlend ==
          Catch::Approx(combinedReversed.colourBlend).margin(1.0e-6F));

    for (const float time : {0.65F, 1.0F, 2.5F, 4.0F}) {
        const auto singleAtTime =
            invisible_places::water::BuildRainImpactGrid(
                firstOnly,
                {},
                time,
                4.0F,
                settings);
        const auto bothAtTime =
            invisible_places::water::BuildRainImpactGrid(
                both,
                {},
                time,
                4.0F,
                settings);
        for (int sample = -6; sample <= 6; ++sample) {
            const Float3 samplePoint{
                static_cast<float>(sample) * 0.01F,
                0.0F,
                0.0F};
            const auto prior = invisible_places::water::EvaluateRainImpact(
                singleAtTime,
                WaterSurfaceRole::Rock,
                samplePoint,
                normal,
                time);
            const auto added = invisible_places::water::EvaluateRainImpact(
                bothAtTime,
                WaterSurfaceRole::Rock,
                samplePoint,
                normal,
                time);
            INFO("time=" << time << ", sample=" << sample);
            CHECK(added.opacity + 1.0e-7F >= prior.opacity);
            CHECK(added.emission + 1.0e-7F >= prior.emission);
            CHECK(added.sizeScale + 1.0e-7F >= prior.sizeScale);
            CHECK(added.colourBlend + 1.0e-7F >= prior.colourBlend);
        }
    }

    auto highEnergy = first;
    highEnergy.position = {};
    highEnergy.energy = 2.5F;
    const std::vector<invisible_places::water::RainImpactEvent> highEvents{
        highEnergy};
    const auto highGrid = invisible_places::water::BuildRainImpactGrid(
        highEvents,
        {},
        0.8F,
        4.0F,
        settings);
    const float raw = invisible_places::water::EvaluateRockRainImpactValue(
        highEnergy,
        {},
        normal,
        0.8F,
        settings) * highEnergy.energy;
    REQUIRE(raw > 1.0F);
    const auto high = invisible_places::water::EvaluateRainImpact(
        highGrid,
        WaterSurfaceRole::Rock,
        {},
        normal,
        0.8F);
    CHECK(high.opacity == Catch::Approx(raw * 0.18F));
    CHECK(high.emission == Catch::Approx(raw * 0.11F));
    CHECK(high.sizeScale == Catch::Approx(1.0F + raw * 0.16F));
    CHECK(high.colourBlend == Catch::Approx(raw * 0.42F));
}

TEST_CASE(
    "saturated ROCK cells retain a continuous high-priority impact across grid boundaries",
    "[water][rain][effects][grid]") {
    std::vector<invisible_places::water::RainImpactEvent> events;
    constexpr float y = 0.007F;
    for (std::uint32_t index = 0U; index < 20U; ++index) {
        events.push_back({
            .position = {-0.020F, y, 0.0F},
            .birthTimeSeconds = 0.0F,
            .normal = {0.0F, 0.0F, 1.0F},
            .radiusMeters = 0.019F,
            .role = WaterSurfaceRole::Rock,
            .lifetimeSeconds = 5.0F,
            .energy = 0.05F,
            .seed = index,
        });
    }
    const auto primaryIndex = static_cast<std::uint32_t>(events.size());
    events.push_back({
        .position = {0.0F, y, 0.0F},
        .birthTimeSeconds = 0.20F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = WaterSurfaceRole::Rock,
        .lifetimeSeconds = 5.0F,
        .energy = 2.5F,
        .seed = 77U,
    });

    auto smooth = invisible_places::water::RainRockImpactSettings{};
    smooth.edgeBreakup = 0.0F;
    smooth.spreadSpeed = 6.0F;
    smooth.centreFalloff = 1.0F;
    const auto grid = invisible_places::water::BuildRainImpactGrid(
        events,
        {},
        1.0F,
        4.0F,
        smooth);
    REQUIRE(grid.overflowCount > 0U);
    const auto cellAt = [&](float x) -> const auto& {
        const auto cellX = static_cast<std::uint32_t>(
            std::floor((x - grid.origin.x) / grid.cellSizeMeters));
        const auto cellY = static_cast<std::uint32_t>(
            std::floor((y - grid.origin.y) / grid.cellSizeMeters));
        return grid.cells[
            static_cast<std::size_t>(cellY) * grid.dimension + cellX];
    };
    const auto containsPrimary = [&](const auto& cell) {
        return std::find(
                   cell.rock.begin(),
                   cell.rock.end(),
                   primaryIndex) !=
               cell.rock.end();
    };
    CHECK(containsPrimary(cellAt(-0.001F)));
    CHECK(containsPrimary(cellAt(0.001F)));

    const auto left = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {-0.001F, y, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.0F);
    const auto right = invisible_places::water::EvaluateRainImpact(
        grid,
        WaterSurfaceRole::Rock,
        {0.001F, y, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.0F);
    CHECK(left.colourBlend > 0.05F);
    CHECK(right.colourBlend > 0.05F);
    CHECK(left.colourBlend == Catch::Approx(right.colourBlend).margin(0.01F));
}

TEST_CASE(
    "SAND rain separates flooded ripples from compact dry uphill splashes",
    "[water][rain][effects][sand]") {
    const invisible_places::water::RainImpactEvent event{
        .position = {0.0F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.10F,
        .role = WaterSurfaceRole::Sand,
        .lifetimeSeconds = 1.0F,
        .energy = 1.0F,
        .seed = 11U,
    };
    constexpr float time = 0.20F;
    const auto wetRing = invisible_places::water::EvaluateSandRainImpactValue(
        event,
        {0.030F, 0.0F, 0.0F},
        time,
        1.0F);
    const auto dryCentre = invisible_places::water::EvaluateSandRainImpactValue(
        event,
        {0.0F, 0.0F, 0.0F},
        time,
        0.0F);
    const auto dryMid = invisible_places::water::EvaluateSandRainImpactValue(
        event,
        {0.012F, 0.0F, -0.001F},
        time,
        0.0F);
    const auto dryOutside = invisible_places::water::EvaluateSandRainImpactValue(
        event,
        {0.026F, 0.0F, 0.0F},
        time,
        0.0F);
    const auto blended = invisible_places::water::EvaluateSandRainImpactValue(
        event,
        {0.030F, 0.0F, 0.0F},
        time,
        0.5F);
    CHECK(wetRing > 0.25F);
    CHECK(dryCentre > dryMid);
    CHECK(dryMid > dryOutside);
    CHECK(dryOutside == Catch::Approx(0.0F).margin(1.0e-5F));
    CHECK(blended > dryOutside);
    CHECK(blended < wetRing);
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

TEST_CASE(
    "ROCK rain response fades smoothly from impact centre to edge",
    "[water][rain][effects][rock]") {
    invisible_places::water::RainRockImpactSettings settings;
    settings.edgeBreakup = 0.0F;
    settings.spreadSpeed = 6.0F;
    settings.centreFalloff = 1.0F;
    settings.heightBias = 0.0F;
    const invisible_places::water::RainImpactEvent event{
        .position = {0.0F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.12F,
        .role = WaterSurfaceRole::Rock,
        .lifetimeSeconds = 5.0F,
        .energy = 1.0F,
        .seed = 4U,
    };
    const float effectiveRadius =
        event.radiusMeters * std::sqrt(2.0F / 3.0F);
    const float centre = invisible_places::water::EvaluateRockRainImpactValue(
        event,
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.5F,
        settings);
    const float middle = invisible_places::water::EvaluateRockRainImpactValue(
        event,
        {effectiveRadius * 0.45F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.5F,
        settings);
    const float edge = invisible_places::water::EvaluateRockRainImpactValue(
        event,
        {effectiveRadius * 0.82F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        1.5F,
        settings);
    CHECK(centre > middle);
    CHECK(middle > edge);
    CHECK(edge >= 0.0F);
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
        {.sourcePath = sceneDirectory / "Site1-MESHSampled-5mm.ply",
         .role = WaterSurfaceRole::Ground,
         .spacingMicrometres = 5'000U},
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
    CHECK(first.cache.groundSourcePointCount > 0U);
    CHECK(first.cache.sources.size() == 4U);
    CHECK_FALSE(first.cache.groundCells.empty());

    const auto gpu = invisible_places::water::BuildWaterSurfaceGpuData(first.cache);
    CHECK(first.cache.surfaceCells.size() <= static_cast<std::size_t>(gpu.surfaceTable.size() * 0.65F));
    CHECK(first.cache.vegetationVoxels.size() <= static_cast<std::size_t>(gpu.vegetationTable.size() * 0.65F));
    CHECK(first.cache.groundCells.size() <=
          static_cast<std::size_t>(gpu.groundTable.size() * 0.80F));
    CHECK(gpu.maximumProbeCount <= 32U);
    CHECK(gpu.groundMaximumProbeCount <= 32U);

    const auto second = invisible_places::water::BuildWaterSurfaceCache(sources, savedDirectory);
    REQUIRE(second.success);
    CHECK(second.loadedFromDisk);
    CHECK(second.cache.signature == first.cache.signature);
    CHECK(second.cache.surfaceCells.size() == first.cache.surfaceCells.size());
    CHECK(second.cache.vegetationVoxels.size() == first.cache.vegetationVoxels.size());
    CHECK(second.cache.flowSurfaceSurfels.size() == first.cache.flowSurfaceSurfels.size());
    CHECK(second.cache.groundCells.size() == first.cache.groundCells.size());
    CHECK(second.cache.gpuData.persistedTables.surfaceCount ==
          first.cache.gpuData.surfaceTable.size());
    CHECK(second.cache.gpuData.persistedTables.vegetationCount ==
          first.cache.gpuData.vegetationTable.size());
    CHECK(second.cache.gpuData.persistedTables.flowSurfaceCount ==
          first.cache.gpuData.flowSurfaceTable.size());
    CHECK(second.cache.gpuData.persistedTables.groundCount ==
          first.cache.gpuData.groundTable.size());
}
