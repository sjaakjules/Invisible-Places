#include "io/AdaptiveHqCache.hpp"
#include "io/PlyHeader.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t kAdaptiveFixturePointCount = 300'000U;

template <typename Value>
void WriteBinary(std::ostream* output, const Value& value) {
    output->write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(value)));
}

std::filesystem::path FixtureRoot(std::string_view name) {
    const auto root = std::filesystem::temp_directory_path() /
        "invisible-places-adaptive-hq-tests" / name;
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    return root;
}

void WriteAdaptiveFixture(const std::filesystem::path& path) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    REQUIRE(output.is_open());
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << kAdaptiveFixturePointCount << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "property float scalar_Heat\n"
           << "end_header\n";
    for (std::uint32_t sourceIndex = 0U;
         sourceIndex < kAdaptiveFixturePointCount;
         ++sourceIndex) {
        // Deliberately not Morton ordered: source order advances primarily
        // through Z, then Y, then X.
        const float x = static_cast<float>((sourceIndex / 10'000U) % 30U);
        const float y = static_cast<float>((sourceIndex / 100U) % 100U) *
            0.1F;
        const float z = static_cast<float>(sourceIndex % 100U) * 0.01F;
        const std::uint8_t red =
            static_cast<std::uint8_t>(sourceIndex & 0xffU);
        const std::uint8_t green =
            static_cast<std::uint8_t>((sourceIndex >> 4U) & 0xffU);
        const std::uint8_t blue =
            static_cast<std::uint8_t>((sourceIndex >> 8U) & 0xffU);
        const float heat = static_cast<float>(sourceIndex) * 0.5F;
        WriteBinary(&output, x);
        WriteBinary(&output, y);
        WriteBinary(&output, z);
        WriteBinary(&output, red);
        WriteBinary(&output, green);
        WriteBinary(&output, blue);
        WriteBinary(&output, heat);
    }
    output.flush();
    REQUIRE(output.good());
}

std::vector<float> FieldValues(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::string_view name) {
    for (std::size_t field = 0U;
         field < cloud.scalarFields.size();
         ++field) {
        if (cloud.scalarFields[field].name != name) {
            continue;
        }
        std::vector<float> values;
        values.reserve(cloud.PointCount());
        for (std::size_t point = 0U;
             point < cloud.PointCount();
             ++point) {
            values.push_back(cloud.scalarFieldValues[
                cloud.ScalarFieldValueIndex(field, point)]);
        }
        return values;
    }
    return {};
}

}  // namespace

TEST_CASE(
    "Adaptive HQ builds local Morton geometry and scalar columns and reuses its sidecar",
    "[pointcloud][adaptive-hq-cache]") {
    const auto fixtureRoot = FixtureRoot("build-and-reuse");
    const auto sourcePath = fixtureRoot / "export-quality-1mm.ply";
    const auto cacheRoot = fixtureRoot / "Saved" / ".invisible_places" /
        "cache" / "adaptive_hq";
    WriteAdaptiveFixture(sourcePath);
    const auto originalSize = std::filesystem::file_size(sourcePath);

    const auto inspected =
        invisible_places::io::InspectAdaptiveHqSource(sourcePath);
    REQUIRE(inspected.success);
    CHECK(inspected.identity.pointCount == kAdaptiveFixturePointCount);
    CHECK(inspected.identity.recordSize == 19U);
    const auto cloudRootRejected =
        invisible_places::io::OpenOrBuildAdaptiveHqCache(
            fixtureRoot / "OneDrive-Simulated" / "Saved" /
                ".invisible_places" / "cache" / "adaptive_hq",
            inspected.identity);
    CHECK_FALSE(cloudRootRejected.success);
    CHECK(cloudRootRejected.errorMessage.find("OneDrive") !=
          std::string::npos);

    std::vector<float> progress;
    const auto first = invisible_places::io::OpenOrBuildAdaptiveHqCache(
        cacheRoot,
        inspected.identity,
        {},
        [&](float value) { progress.push_back(value); });
    REQUIRE(first.success);
    CHECK(first.built);
    REQUIRE_FALSE(first.index.blocks.empty());
    CHECK(first.index.blocks.size() >= 2U);
    CHECK(first.index.dataPath.parent_path().parent_path() ==
          std::filesystem::weakly_canonical(cacheRoot));
    CHECK(first.index.dataPath != sourcePath);
    CHECK(std::filesystem::exists(first.index.indexPath));
    CHECK(std::filesystem::exists(first.index.dataPath));
    CHECK(std::filesystem::exists(first.index.scalarDataPath));
    CHECK(first.index.dataPath.extension() == ".bin");
    CHECK(first.index.scalarDataPath.extension() == ".bin");
    CHECK(first.index.cachedRecordSize == 20U);
    REQUIRE(first.index.scalarFields.size() == 1U);
    CHECK(first.index.scalarFields[0].name == "Heat");
    CHECK(first.index.scalarFields[0].sourceIndex == 0U);
    CHECK(first.index.scalarFields[0].stats.minimum == Catch::Approx(0.0F));
    CHECK(first.index.scalarFields[0].stats.maximum ==
          Catch::Approx(
              static_cast<float>(kAdaptiveFixturePointCount - 1U) * 0.5F));
    CHECK(first.index.dataByteSize ==
          static_cast<std::uint64_t>(kAdaptiveFixturePointCount) * 20U);
    CHECK(first.index.scalarDataByteSize ==
          static_cast<std::uint64_t>(kAdaptiveFixturePointCount) *
              sizeof(float));
    CHECK(std::filesystem::file_size(sourcePath) == originalSize);
    CHECK_FALSE(progress.empty());
    CHECK(progress.back() == Catch::Approx(1.0F));

    std::uint64_t expectedPoint = 0U;
    std::uint64_t expectedOffset = first.index.dataOffsetBytes;
    std::uint64_t expectedScalarOffset = 0U;
    for (const auto& block : first.index.blocks) {
        CHECK(block.firstPoint == expectedPoint);
        CHECK(block.fileOffsetBytes == expectedOffset);
        CHECK(block.byteSize ==
              block.pointCount * first.index.cachedRecordSize);
        CHECK(block.byteSize <=
              invisible_places::io::kAdaptiveHqCacheMaximumBlockBytes);
        CHECK(block.byteSize >=
              invisible_places::io::kAdaptiveHqCacheMinimumBlockBytes);
        CHECK(block.scalarFileOffsetBytes == expectedScalarOffset);
        CHECK(block.scalarByteSize ==
              block.pointCount * sizeof(float));
        CHECK(block.bounds.valid);
        expectedPoint += block.pointCount;
        expectedOffset += block.byteSize;
        expectedScalarOffset += block.scalarByteSize;
    }
    CHECK(expectedPoint == kAdaptiveFixturePointCount);

    std::ifstream indexInput{first.index.indexPath};
    REQUIRE(indexInput.is_open());
    const auto sidecar = nlohmann::json::parse(indexInput);
    CHECK(sidecar.at("algorithm") ==
          invisible_places::io::kAdaptiveHqCacheAlgorithmId);
    CHECK(sidecar.at("source").at("schema_fingerprint") ==
          inspected.identity.schemaFingerprint);
    CHECK(sidecar.at("blocks").size() == first.index.blocks.size());
    CHECK(sidecar.at("scalar_fields").size() == 1U);
    CHECK(sidecar.at("scalar_data_file") ==
          first.index.scalarDataPath.filename().string());

    const auto second = invisible_places::io::OpenOrBuildAdaptiveHqCache(
        cacheRoot,
        inspected.identity);
    REQUIRE(second.success);
    CHECK_FALSE(second.built);
    CHECK(second.index.dataPath == first.index.dataPath);
    CHECK(second.index.scalarDataPath == first.index.scalarDataPath);
    CHECK(second.index.cacheFingerprint == first.index.cacheFingerprint);
}

TEST_CASE(
    "Adaptive HQ seeks selected blocks and restores original point order",
    "[pointcloud][adaptive-hq-cache]") {
    const auto fixtureRoot = FixtureRoot("block-seek");
    const auto sourcePath = fixtureRoot / "source.ply";
    const auto cacheRoot = fixtureRoot / "Saved" / ".invisible_places" /
        "cache" / "adaptive_hq";
    WriteAdaptiveFixture(sourcePath);
    const auto inspected =
        invisible_places::io::InspectAdaptiveHqSource(sourcePath);
    REQUIRE(inspected.success);
    const auto opened = invisible_places::io::OpenOrBuildAdaptiveHqCache(
        cacheRoot,
        inspected.identity);
    REQUIRE(opened.success);
    REQUIRE(opened.index.blocks.size() >= 2U);

    invisible_places::io::PointCloudScalarFieldFilter filter;
    filter.mode = invisible_places::io::PointCloudScalarFieldFilter::Mode::
        Selected;
    filter.names = {"Heat"};
    const std::vector<std::uint32_t> activeBlocks{
        0U,
        static_cast<std::uint32_t>(opened.index.blocks.size() - 1U),
    };
    std::vector<invisible_places::io::AdaptiveHqResidentBlock> resident;
    invisible_places::io::PointCloudScalarFieldFilter geometryOnly;
    geometryOnly.mode = invisible_places::io::PointCloudScalarFieldFilter::
        Mode::Selected;
    const auto geometryBlock =
        invisible_places::io::LoadAdaptiveHqCacheBlock(
            opened.index,
            activeBlocks.front(),
            geometryOnly);
    REQUIRE(geometryBlock.success);
    CHECK(geometryBlock.cloud.scalarFields.empty());
    CHECK(geometryBlock.cloud.scalarFieldValues.empty());
    CHECK(geometryBlock.cloud.availableScalarFields.size() == 1U);
    for (const auto blockIndex : activeBlocks) {
        auto loaded = invisible_places::io::LoadAdaptiveHqCacheBlock(
            opened.index,
            blockIndex,
            filter);
        REQUIRE(loaded.success);
        CHECK(loaded.cloud.PointCount() ==
              opened.index.blocks[blockIndex].pointCount);
        resident.push_back({
            .blockIndex = blockIndex,
            .points = std::make_shared<const
                invisible_places::io::PointCloudSubsetLoadResult>(
                    std::move(loaded)),
        });
    }

    const auto assembled =
        invisible_places::io::AssembleAdaptiveHqCacheSubset(
            opened.index,
            activeBlocks,
            resident,
            [](const invisible_places::io::Float3& point) {
                return point.y < 5.0F;
            });
    REQUIRE(assembled.success);
    REQUIRE_FALSE(assembled.sourcePointIndices.empty());
    CHECK(std::is_sorted(
        assembled.sourcePointIndices.begin(),
        assembled.sourcePointIndices.end()));
    CHECK(std::adjacent_find(
              assembled.sourcePointIndices.begin(),
              assembled.sourcePointIndices.end()) ==
          assembled.sourcePointIndices.end());
    CHECK(std::all_of(
        assembled.cloud.positions.begin(),
        assembled.cloud.positions.end(),
        [](const auto& point) { return point.y < 5.0F; }));
    const auto heat = FieldValues(assembled.cloud, "Heat");
    REQUIRE(heat.size() == assembled.sourcePointIndices.size());
    for (std::size_t point = 0U; point < heat.size(); ++point) {
        CHECK(heat[point] == Catch::Approx(
            static_cast<float>(assembled.sourcePointIndices[point]) * 0.5F));
    }

    std::vector<std::uint32_t> expectedMortonOrder;
    for (const auto& block : resident) {
        REQUIRE(block.points != nullptr);
        for (std::size_t point = 0U;
             point < block.points->cloud.PointCount();
             ++point) {
            if (block.points->cloud.positions[point].y < 5.0F) {
                expectedMortonOrder.push_back(
                    block.points->sourcePointIndices[point]);
            }
        }
    }
    const auto mortonAssembled =
        invisible_places::io::AssembleAdaptiveHqCacheSubset(
            opened.index,
            activeBlocks,
            resident,
            [](const invisible_places::io::Float3& point) {
                return point.y < 5.0F;
            },
            {},
            {},
            false);
    REQUIRE(mortonAssembled.success);
    CHECK(mortonAssembled.sourcePointIndices == expectedMortonOrder);
    const auto mortonHeat = FieldValues(
        mortonAssembled.cloud,
        "Heat");
    REQUIRE(mortonHeat.size() == expectedMortonOrder.size());
    for (std::size_t point = 0U;
         point < mortonHeat.size();
         ++point) {
        CHECK(mortonHeat[point] == Catch::Approx(
            static_cast<float>(expectedMortonOrder[point]) * 0.5F));
    }

    for (auto& block : resident) {
        block.microBlocks = std::make_shared<const std::vector<
            invisible_places::io::AdaptiveHqResidentBlock::MicroBlock>>(
            invisible_places::io::BuildAdaptiveHqMicroBlocks(
                block.points->cloud.positions,
                1024U));
        REQUIRE_FALSE(block.microBlocks->empty());
    }
    std::atomic<std::size_t> pointPredicateCalls{0U};
    const auto microBlockAssembled =
        invisible_places::io::AssembleAdaptiveHqCacheSubset(
            opened.index,
            activeBlocks,
            resident,
            [&](const invisible_places::io::Float3&) {
                pointPredicateCalls.fetch_add(1U);
                return false;
            },
            {},
            {},
            false,
            [](const invisible_places::io::Bounds3f& bounds) {
                return bounds.minimum.y < 5.0F;
            });
    REQUIRE(microBlockAssembled.success);
    CHECK(pointPredicateCalls.load() == 0U);
    CHECK(microBlockAssembled.cloud.PointCount() >=
          mortonAssembled.cloud.PointCount());
    std::set<std::uint32_t> microBlockSourceIndices{
        microBlockAssembled.sourcePointIndices.begin(),
        microBlockAssembled.sourcePointIndices.end()};
    CHECK(std::all_of(
        expectedMortonOrder.begin(),
        expectedMortonOrder.end(),
        [&](std::uint32_t sourceIndex) {
            return microBlockSourceIndices.contains(sourceIndex);
        }));
    const auto microBlockHeat = FieldValues(
        microBlockAssembled.cloud,
        "Heat");
    REQUIRE(microBlockHeat.size() ==
            microBlockAssembled.sourcePointIndices.size());
    for (std::size_t point = 0U;
         point < microBlockHeat.size();
         ++point) {
        CHECK(microBlockHeat[point] == Catch::Approx(
            static_cast<float>(
                microBlockAssembled.sourcePointIndices[point]) *
            0.5F));
    }

    const auto& firstBounds = opened.index.blocks.front().bounds;
    const auto selected =
        invisible_places::io::SelectAdaptiveHqCacheBlocks(
            opened.index,
            [&](const invisible_places::io::Bounds3f& bounds) {
                return bounds.minimum.x == firstBounds.minimum.x &&
                       bounds.minimum.y == firstBounds.minimum.y &&
                       bounds.minimum.z == firstBounds.minimum.z &&
                       bounds.maximum.x == firstBounds.maximum.x &&
                       bounds.maximum.y == firstBounds.maximum.y &&
                       bounds.maximum.z == firstBounds.maximum.z;
            });
    CHECK_FALSE(selected.empty());
    CHECK(selected.front() == 0U);
    const auto ranges = invisible_places::io::AdaptiveHqCacheBlockRanges(
        opened.index,
        selected);
    CHECK_FALSE(ranges.empty());
}

TEST_CASE(
    "Adaptive HQ invalidates a same-size changed source without replacing it",
    "[pointcloud][adaptive-hq-cache]") {
    const auto fixtureRoot = FixtureRoot("invalidate");
    const auto sourcePath = fixtureRoot / "source.ply";
    const auto cacheRoot = fixtureRoot / "Saved" / ".invisible_places" /
        "cache" / "adaptive_hq";
    WriteAdaptiveFixture(sourcePath);
    const auto firstIdentity =
        invisible_places::io::InspectAdaptiveHqSource(sourcePath);
    REQUIRE(firstIdentity.success);
    const auto first = invisible_places::io::OpenOrBuildAdaptiveHqCache(
        cacheRoot,
        firstIdentity.identity);
    REQUIRE(first.success);
    const auto firstScalarDataPath = first.index.scalarDataPath;

    const auto header = invisible_places::io::ParsePlyHeader(sourcePath);
    REQUIRE(header.success);
    {
        std::fstream source{
            sourcePath,
            std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(source.is_open());
        source.seekp(
            static_cast<std::streamoff>(header.header.dataOffsetBytes),
            std::ios::beg);
        const float changedX = 123.0F;
        WriteBinary(&source, changedX);
        source.flush();
        REQUIRE(source.good());
    }
    std::filesystem::last_write_time(
        sourcePath,
        std::filesystem::last_write_time(sourcePath) +
            std::chrono::seconds{2});
    const auto changedIdentity =
        invisible_places::io::InspectAdaptiveHqSource(sourcePath);
    REQUIRE(changedIdentity.success);
    CHECK(changedIdentity.identity.byteSize ==
          firstIdentity.identity.byteSize);
    CHECK(changedIdentity.identity.contentFingerprint !=
          firstIdentity.identity.contentFingerprint);
    CHECK(changedIdentity.identity != firstIdentity.identity);

    const auto rebuilt = invisible_places::io::OpenOrBuildAdaptiveHqCache(
        cacheRoot,
        changedIdentity.identity);
    REQUIRE(rebuilt.success);
    CHECK(rebuilt.built);
    CHECK(rebuilt.index.dataPath != first.index.dataPath);
    CHECK(rebuilt.index.scalarDataPath != firstScalarDataPath);
    CHECK_FALSE(std::filesystem::exists(first.index.dataPath));
    CHECK_FALSE(std::filesystem::exists(firstScalarDataPath));
    CHECK(std::filesystem::exists(sourcePath));
    CHECK(std::filesystem::file_size(sourcePath) ==
          firstIdentity.identity.byteSize);
}
