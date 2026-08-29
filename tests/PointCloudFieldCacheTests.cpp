#include "io/PointCloudFieldCache.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

using invisible_places::io::LoadPointCloud;
using invisible_places::io::LoadPointCloudWithFieldCache;
using invisible_places::io::PointCloudFieldCacheDirectory;
using invisible_places::io::PointCloudScalarFieldFilter;
using invisible_places::io::RebuildPointCloudFieldCache;

template <typename T>
void WriteBinaryValue(std::ofstream* output, const T& value) {
    output->write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(T)));
}

constexpr std::size_t kPointCount = 5U;

// Three points-worth of scalar variety: two float fields and normals, no
// RGB, so the cache round-trip covers the hasSourceRgb=false path too.
void WriteCacheFixturePly(
    const std::filesystem::path& path,
    float heightBias) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    REQUIRE(output.is_open());
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << kPointCount << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property float nx\n"
           << "property float ny\n"
           << "property float nz\n"
           << "property float scalar_Height\n"
           << "property float scalar_Flow_Rate\n"
           << "end_header\n";
    for (std::size_t pointIndex = 0; pointIndex < kPointCount; ++pointIndex) {
        const auto base = static_cast<float>(pointIndex);
        WriteBinaryValue(&output, base);
        WriteBinaryValue(&output, -base);
        WriteBinaryValue(&output, base * 0.5F);
        WriteBinaryValue(&output, 0.0F);
        WriteBinaryValue(&output, 1.0F);
        WriteBinaryValue(&output, 0.0F);
        WriteBinaryValue(&output, heightBias + base);
        WriteBinaryValue(&output, base * base);
    }
}

std::filesystem::path FixtureDirectory(const std::string& name) {
    const auto directory =
        std::filesystem::temp_directory_path() /
        "invisible-places-field-cache" / name;
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    return directory;
}

std::vector<float> FieldColumn(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::string& name) {
    for (std::size_t slot = 0; slot < cloud.scalarFields.size(); ++slot) {
        if (cloud.scalarFields[slot].name != name) {
            continue;
        }
        std::vector<float> column(cloud.PointCount());
        for (std::size_t pointIndex = 0; pointIndex < cloud.PointCount(); ++pointIndex) {
            column[pointIndex] =
                cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(slot, pointIndex)];
        }
        return column;
    }
    return {};
}

// Rewrites one cached field file with a sentinel so a subsequent load can
// prove it came from the cache rather than the PLY.
void TamperCachedField(
    const std::filesystem::path& sourcePath,
    std::uint32_t sourceIndex,
    const std::string& token,
    float sentinel) {
    const auto directory = PointCloudFieldCacheDirectory(sourcePath);
    const auto fieldPath =
        directory /
        ("field_" + std::to_string(sourceIndex) + "_" + token + ".bin");
    REQUIRE(std::filesystem::exists(fieldPath));
    std::vector<float> values(kPointCount, sentinel);
    std::ofstream output{fieldPath, std::ios::binary | std::ios::trunc};
    output.write(
        reinterpret_cast<const char*>(values.data()),
        static_cast<std::streamsize>(values.size() * sizeof(float)));
}

}  // namespace

TEST_CASE("Field cache round-trips a cloud and serves later loads",
          "[pointcloud][fieldcache]") {
    const auto directory = FixtureDirectory("round-trip");
    const auto plyPath = directory / "cloud.ply";
    WriteCacheFixturePly(plyPath, 10.0F);

    const auto first = LoadPointCloudWithFieldCache(plyPath);
    REQUIRE(first.success);
    const auto cacheDirectory = PointCloudFieldCacheDirectory(plyPath);
    CHECK(std::filesystem::exists(cacheDirectory / "manifest.json"));
    CHECK(std::filesystem::exists(cacheDirectory / "geometry.bin"));

    const auto second = LoadPointCloudWithFieldCache(plyPath);
    REQUIRE(second.success);
    CHECK(second.cloud.PointCount() == first.cloud.PointCount());
    CHECK(second.cloud.hasNormals == first.cloud.hasNormals);
    CHECK(second.cloud.hasSourceRgb == first.cloud.hasSourceRgb);
    CHECK(second.cloud.bounds.valid);
    CHECK(second.cloud.bounds.maximum.x ==
          Catch::Approx(first.cloud.bounds.maximum.x));
    CHECK(second.cloud.focusPoint.x == Catch::Approx(first.cloud.focusPoint.x));
    REQUIRE(second.cloud.scalarFields.size() == 2U);
    CHECK(second.cloud.scalarFields[0].name == "Height");
    CHECK(second.cloud.scalarFields[0].sourceIndex == 0);
    CHECK(second.cloud.scalarFields[0].valid);
    CHECK(second.cloud.scalarFields[0].minimum == Catch::Approx(10.0F));
    CHECK(FieldColumn(second.cloud, "Height") ==
          FieldColumn(first.cloud, "Height"));
    CHECK(FieldColumn(second.cloud, "Flow_Rate") ==
          FieldColumn(first.cloud, "Flow_Rate"));
    REQUIRE(second.cloud.positions.size() == kPointCount);
    CHECK(second.cloud.positions[3].x == Catch::Approx(3.0F));
    CHECK(second.cloud.normals[2].y == Catch::Approx(1.0F));
    CHECK(second.cloud.availableScalarFields.size() == 2U);

    // Prove the second-style load reads cache bytes, not the PLY.
    TamperCachedField(plyPath, 0U, "height", 777.0F);
    const auto third = LoadPointCloudWithFieldCache(plyPath);
    REQUIRE(third.success);
    CHECK(FieldColumn(third.cloud, "Height")[0] == Catch::Approx(777.0F));
    // The PLY loader itself remains the source of truth when bypassing the
    // cache.
    const auto direct = LoadPointCloud(plyPath);
    REQUIRE(direct.success);
    CHECK(FieldColumn(direct.cloud, "Height")[0] == Catch::Approx(10.0F));
}

TEST_CASE("A rewritten source invalidates the cache and prunes stale fields",
          "[pointcloud][fieldcache]") {
    const auto directory = FixtureDirectory("stale");
    const auto plyPath = directory / "cloud.ply";
    WriteCacheFixturePly(plyPath, 10.0F);
    REQUIRE(LoadPointCloudWithFieldCache(plyPath).success);
    TamperCachedField(plyPath, 0U, "height", 999.0F);

    // Rewrite with different content but deliberately restore the exact old
    // timestamp. Size and mtime alone cannot catch this replacement; schema
    // plus sampled content fingerprints must invalidate the cache.
    const auto originalWriteTime = std::filesystem::last_write_time(plyPath);
    WriteCacheFixturePly(plyPath, 50.0F);
    std::filesystem::last_write_time(plyPath, originalWriteTime);

    const auto reloaded = LoadPointCloudWithFieldCache(plyPath);
    REQUIRE(reloaded.success);
    const auto heights = FieldColumn(reloaded.cloud, "Height");
    REQUIRE(heights.size() == kPointCount);
    CHECK(heights[0] == Catch::Approx(50.0F));
    CHECK(reloaded.cloud.scalarFields[0].minimum == Catch::Approx(50.0F));

    // And the refreshed cache serves the new values.
    const auto cached = LoadPointCloudWithFieldCache(plyPath);
    REQUIRE(cached.success);
    CHECK(FieldColumn(cached.cloud, "Height")[0] == Catch::Approx(50.0F));
}

TEST_CASE("Filtered cache loads stream missing fields and write them through",
          "[pointcloud][fieldcache]") {
    const auto directory = FixtureDirectory("filtered");
    const auto plyPath = directory / "cloud.ply";
    WriteCacheFixturePly(plyPath, 10.0F);

    PointCloudScalarFieldFilter heightOnly;
    heightOnly.mode = PointCloudScalarFieldFilter::Mode::Selected;
    heightOnly.names = {"Height"};
    const auto first = LoadPointCloudWithFieldCache(plyPath, heightOnly);
    REQUIRE(first.success);
    REQUIRE(first.cloud.scalarFields.size() == 1U);
    CHECK(first.cloud.availableScalarFields.size() == 2U);
    const auto cacheDirectory = PointCloudFieldCacheDirectory(plyPath);
    CHECK(std::filesystem::exists(
        cacheDirectory / "field_0_height.bin"));
    CHECK_FALSE(std::filesystem::exists(
        cacheDirectory / "field_1_flow-rate.bin"));

    // Selecting the other field on a cache hit streams it from the PLY and
    // caches it for next time.
    PointCloudScalarFieldFilter flowOnly;
    flowOnly.mode = PointCloudScalarFieldFilter::Mode::Selected;
    flowOnly.names = {"Flow_Rate"};
    const auto second = LoadPointCloudWithFieldCache(plyPath, flowOnly);
    REQUIRE(second.success);
    REQUIRE(second.cloud.scalarFields.size() == 1U);
    CHECK(second.cloud.scalarFields[0].name == "Flow_Rate");
    CHECK(second.cloud.scalarFields[0].sourceIndex == 1);
    const auto flowValues = FieldColumn(second.cloud, "Flow_Rate");
    CHECK(flowValues[4] == Catch::Approx(16.0F));
    CHECK(std::filesystem::exists(
        cacheDirectory / "field_1_flow-rate.bin"));

    TamperCachedField(plyPath, 1U, "flow-rate", -5.0F);
    const auto third = LoadPointCloudWithFieldCache(plyPath, flowOnly);
    REQUIRE(third.success);
    CHECK(FieldColumn(third.cloud, "Flow_Rate")[0] == Catch::Approx(-5.0F));
}

TEST_CASE("Explicit field-cache rebuild materializes all cold columns in one pass",
          "[pointcloud][fieldcache]") {
    const auto directory = FixtureDirectory("explicit-rebuild");
    const auto plyPath = directory / "cloud.ply";
    WriteCacheFixturePly(plyPath, 10.0F);

    PointCloudScalarFieldFilter geometryOnly;
    geometryOnly.mode = PointCloudScalarFieldFilter::Mode::Selected;
    REQUIRE(LoadPointCloudWithFieldCache(plyPath, geometryOnly).success);
    const auto cacheDirectory = PointCloudFieldCacheDirectory(plyPath);
    CHECK(std::filesystem::exists(cacheDirectory / "geometry.bin"));
    CHECK_FALSE(std::filesystem::exists(
        cacheDirectory / "field_0_height.bin"));
    CHECK_FALSE(std::filesystem::exists(
        cacheDirectory / "field_1_flow-rate.bin"));

    const auto rebuilt = RebuildPointCloudFieldCache(plyPath);
    REQUIRE(rebuilt.success);
    REQUIRE(rebuilt.cloud.scalarFields.size() == 2U);
    CHECK(std::filesystem::exists(
        cacheDirectory / "field_0_height.bin"));
    CHECK(std::filesystem::exists(
        cacheDirectory / "field_1_flow-rate.bin"));

    // Rebuild is source-authoritative, not an incremental reuse of a
    // previously tampered cold column.
    TamperCachedField(plyPath, 0U, "height", 444.0F);
    const auto refreshed = RebuildPointCloudFieldCache(plyPath);
    REQUIRE(refreshed.success);
    CHECK(FieldColumn(refreshed.cloud, "Height")[0] ==
          Catch::Approx(10.0F));
    const auto cached = LoadPointCloudWithFieldCache(plyPath);
    REQUIRE(cached.success);
    CHECK(FieldColumn(cached.cloud, "Height")[0] ==
          Catch::Approx(10.0F));
}

TEST_CASE("Single cached fields read and write through the manifest",
          "[pointcloud][fieldcache]") {
    const auto directory = FixtureDirectory("single-field");
    const auto plyPath = directory / "cloud.ply";
    WriteCacheFixturePly(plyPath, 10.0F);

    PointCloudScalarFieldFilter geometryOnly;
    geometryOnly.mode = PointCloudScalarFieldFilter::Mode::Selected;
    REQUIRE(LoadPointCloudWithFieldCache(plyPath, geometryOnly).success);

    std::vector<float> values(kPointCount, 0.0F);
    invisible_places::io::ScalarFieldStats stats;
    // Nothing cached yet for Flow_Rate.
    CHECK_FALSE(invisible_places::io::ReadPointCloudCachedField(
        plyPath,
        "Flow_Rate",
        values,
        &stats));

    invisible_places::io::ScalarFieldStats written;
    written.name = "Flow_Rate";
    written.sourceIndex = 1;
    std::vector<float> column(kPointCount);
    for (std::size_t index = 0; index < kPointCount; ++index) {
        column[index] = static_cast<float>(index) * 2.0F;
        written.Include(column[index]);
    }
    REQUIRE(invisible_places::io::WritePointCloudCachedField(
        plyPath,
        written,
        column));

    REQUIRE(invisible_places::io::ReadPointCloudCachedField(
        plyPath,
        "Flow_Rate",
        values,
        &stats));
    CHECK(values == column);
    CHECK(stats.sourceIndex == 1);
    CHECK(stats.valid);
    CHECK(stats.maximum == Catch::Approx(8.0F));

    // Wrong-size reads are rejected.
    std::vector<float> wrongSize(kPointCount + 1U, 0.0F);
    CHECK_FALSE(invisible_places::io::ReadPointCloudCachedField(
        plyPath,
        "Flow_Rate",
        wrongSize,
        nullptr));
}
