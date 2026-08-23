#include "app/LinkedHighQualityPreview.hpp"
#include "io/PointCloudData.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/ext/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

namespace {

template <typename T>
void WriteBinary(std::ofstream* output, const T& value) {
    output->write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(T)));
}

std::filesystem::path WriteSubsetFixture(std::string_view stem) {
    const auto path =
        std::filesystem::temp_directory_path() /
        "invisible-places-linked-hq" /
        (std::string{stem} + ".ply");
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    REQUIRE(output.is_open());
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex 6\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "property float nx\n"
           << "property float ny\n"
           << "property float nz\n"
           << "property float scalar_Size\n"
           << "property float scalar_Opacity\n"
           << "end_header\n";
    for (std::uint32_t index = 0U; index < 6U; ++index) {
        const float x = static_cast<float>(index) - 2.0F;
        const float y = static_cast<float>(index) * 0.25F;
        const float z = 0.0F;
        const auto red = static_cast<std::uint8_t>(20U + index);
        const auto green = static_cast<std::uint8_t>(40U + index);
        const auto blue = static_cast<std::uint8_t>(60U + index);
        const float normalX = 0.0F;
        const float normalY = 0.0F;
        const float normalZ = 2.0F;
        const float size = 10.0F + static_cast<float>(index);
        const float opacity = 0.1F * static_cast<float>(index);
        WriteBinary(&output, x);
        WriteBinary(&output, y);
        WriteBinary(&output, z);
        WriteBinary(&output, red);
        WriteBinary(&output, green);
        WriteBinary(&output, blue);
        WriteBinary(&output, normalX);
        WriteBinary(&output, normalY);
        WriteBinary(&output, normalZ);
        WriteBinary(&output, size);
        WriteBinary(&output, opacity);
    }
    return path;
}

std::vector<float> FieldValues(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::string_view name) {
    for (std::size_t slot = 0U; slot < cloud.scalarFields.size(); ++slot) {
        if (cloud.scalarFields[slot].name != name) {
            continue;
        }
        std::vector<float> values;
        values.reserve(cloud.PointCount());
        for (std::size_t point = 0U; point < cloud.PointCount(); ++point) {
            values.push_back(cloud.scalarFieldValues[
                cloud.ScalarFieldValueIndex(slot, point)]);
        }
        return values;
    }
    return {};
}

}  // namespace

TEST_CASE(
    "Linked HQ midpoint union pads only the viewport and partitions exactly",
    "[pointcloud][linked-hq]") {
    invisible_places::app::LinkedHqFrustumUnion frustums;
    frustums.midpointViewProjections[0U] = glm::mat4{1.0F};
    frustums.midpointViewProjections[1U] = glm::translate(
        glm::mat4{1.0F},
        glm::vec3{-3.0F, 0.0F, 0.0F});

    CHECK(frustums.Contains({1.0F, 0.0F, 0.0F}));
    CHECK(frustums.Contains({1.099F, 0.0F, 0.0F}));
    CHECK_FALSE(frustums.Contains({1.101F, 0.0F, 0.0F}));
    CHECK(frustums.Contains({3.0F, 0.0F, 0.0F}));
    CHECK(frustums.Contains({4.099F, 0.0F, 0.0F}));
    CHECK_FALSE(frustums.Contains({4.101F, 0.0F, 0.0F}));
    CHECK_FALSE(frustums.Contains({0.0F, 0.0F, 1.001F}));

    const std::array<invisible_places::io::Float3, 7U> points{{
        {-2.0F, 0.0F, 0.0F},
        {-1.1F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
        {1.1F, 0.0F, 0.0F},
        {2.0F, 0.0F, 0.0F},
        {3.0F, 0.0F, 0.0F},
        {5.0F, 0.0F, 0.0F},
    }};
    const auto partition =
        invisible_places::app::PartitionLinkedHqIndices(points, frustums);
    REQUIRE(partition.inside.size() + partition.outside.size() == points.size());
    std::vector<std::uint32_t> all = partition.inside;
    all.insert(all.end(), partition.outside.begin(), partition.outside.end());
    std::sort(all.begin(), all.end());
    CHECK(all == std::vector<std::uint32_t>{0U, 1U, 2U, 3U, 4U, 5U, 6U});
    std::vector<std::uint32_t> overlap;
    std::set_intersection(
        partition.inside.begin(),
        partition.inside.end(),
        partition.outside.begin(),
        partition.outside.end(),
        std::back_inserter(overlap));
    CHECK(overlap.empty());

    std::stop_source cancelled;
    cancelled.request_stop();
    std::vector<std::uint64_t> cancelledProgress;
    const auto interrupted =
        invisible_places::app::PartitionLinkedHqIndices(
            points,
            frustums,
            cancelled.get_token(),
            [&](std::uint64_t completed, std::uint64_t) {
                cancelledProgress.push_back(completed);
            });
    CHECK(interrupted.cancelled);
    CHECK(interrupted.inside.empty());
    CHECK(interrupted.outside.empty());
    CHECK(cancelledProgress == std::vector<std::uint64_t>{0U, 0U});
}

TEST_CASE(
    "Filtered PLY loading preserves compact attributes and source indices",
    "[pointcloud][linked-hq][io]") {
    const auto path = WriteSubsetFixture("filtered-subset");
    invisible_places::io::PointCloudSubsetLoadOptions options;
    options.fieldFilter.mode =
        invisible_places::io::PointCloudScalarFieldFilter::Mode::Selected;
    options.fieldFilter.names = {"Size", "Opacity"};
    options.includePoint = [](const invisible_places::io::Float3& point) {
        return point.x >= -1.0F && point.x <= 2.0F;
    };
    std::vector<std::uint64_t> progress;
    options.progress = [&](std::uint64_t completed, std::uint64_t total) {
        CHECK(completed <= total);
        progress.push_back(completed);
    };

    const auto result =
        invisible_places::io::LoadPointCloudSubset(path, options);
    REQUIRE(result.success);
    CHECK_FALSE(result.cancelled);
    CHECK(result.sourcePointCount == 6U);
    CHECK(result.sourcePointIndices ==
          std::vector<std::uint32_t>{1U, 2U, 3U, 4U});
    REQUIRE(result.cloud.PointCount() == 4U);
    REQUIRE(result.cloud.normals.size() == 4U);
    REQUIRE(result.cloud.packedColors.size() == 4U);
    CHECK(result.cloud.positions.front().x == Catch::Approx(-1.0F));
    CHECK(result.cloud.positions.back().x == Catch::Approx(2.0F));
    CHECK(result.cloud.normals.front().z == Catch::Approx(1.0F));
    CHECK((result.cloud.packedColors.front() & 0xFFU) == 21U);
    CHECK(FieldValues(result.cloud, "Size") ==
          std::vector<float>{11.0F, 12.0F, 13.0F, 14.0F});
    CHECK(FieldValues(result.cloud, "Opacity") ==
          std::vector<float>{0.1F, 0.2F, 0.3F, 0.4F});
    REQUIRE_FALSE(progress.empty());
    CHECK(progress.front() == 0U);
    CHECK(progress.back() == 6U);
    CHECK(std::is_sorted(progress.begin(), progress.end()));
}

TEST_CASE(
    "Range-parallel filtered PLY loading matches the sequential scan",
    "[pointcloud][linked-hq][io]") {
    const auto path = WriteSubsetFixture("parallel-subset");
    const auto load = [&](unsigned threadCount) {
        invisible_places::io::PointCloudSubsetLoadOptions options;
        options.fieldFilter.mode =
            invisible_places::io::PointCloudScalarFieldFilter::Mode::Selected;
        options.fieldFilter.names = {"Size", "Opacity"};
        options.includePoint =
            [](const invisible_places::io::Float3& point) {
                return point.x >= -2.0F && point.x <= 2.0F &&
                       point.x != 0.0F;
            };
        options.threadCount = threadCount;
        return invisible_places::io::LoadPointCloudSubset(path, options);
    };
    const auto sequential = load(1U);
    REQUIRE(sequential.success);
    CHECK(sequential.sourcePointIndices ==
          std::vector<std::uint32_t>{0U, 1U, 3U, 4U});

    // Four ranges over six points leaves some ranges with one point and the
    // rejected point on a range boundary.
    for (const unsigned threadCount : {2U, 4U, 6U}) {
        const auto parallel = load(threadCount);
        REQUIRE(parallel.success);
        CHECK_FALSE(parallel.cancelled);
        CHECK(parallel.sourcePointCount == sequential.sourcePointCount);
        CHECK(parallel.sourcePointIndices == sequential.sourcePointIndices);
        REQUIRE(parallel.cloud.PointCount() == sequential.cloud.PointCount());
        for (std::size_t point = 0U;
             point < sequential.cloud.PointCount();
             ++point) {
            CHECK(parallel.cloud.positions[point].x ==
                  sequential.cloud.positions[point].x);
            CHECK(parallel.cloud.positions[point].y ==
                  sequential.cloud.positions[point].y);
            CHECK(parallel.cloud.normals[point].z ==
                  sequential.cloud.normals[point].z);
            CHECK(parallel.cloud.packedColors[point] ==
                  sequential.cloud.packedColors[point]);
        }
        CHECK(FieldValues(parallel.cloud, "Size") ==
              FieldValues(sequential.cloud, "Size"));
        CHECK(FieldValues(parallel.cloud, "Opacity") ==
              FieldValues(sequential.cloud, "Opacity"));
        REQUIRE(parallel.cloud.scalarFields.size() ==
                sequential.cloud.scalarFields.size());
        for (std::size_t slot = 0U;
             slot < sequential.cloud.scalarFields.size();
             ++slot) {
            CHECK(parallel.cloud.scalarFields[slot].minimum ==
                  sequential.cloud.scalarFields[slot].minimum);
            CHECK(parallel.cloud.scalarFields[slot].maximum ==
                  sequential.cloud.scalarFields[slot].maximum);
            CHECK(parallel.cloud.scalarFields[slot].count ==
                  sequential.cloud.scalarFields[slot].count);
        }
        CHECK(parallel.cloud.bounds.minimum.x ==
              sequential.cloud.bounds.minimum.x);
        CHECK(parallel.cloud.bounds.maximum.x ==
              sequential.cloud.bounds.maximum.x);
        CHECK(parallel.cloud.focusPoint.x == sequential.cloud.focusPoint.x);
        CHECK(parallel.cloud.focusPoint.y == sequential.cloud.focusPoint.y);
    }
}

TEST_CASE(
    "Filtered PLY loading and indexed field gathering are cancellable",
    "[pointcloud][linked-hq][io]") {
    const auto path = WriteSubsetFixture("cancellable-streams");
    std::stop_source cancelled;
    cancelled.request_stop();
    invisible_places::io::PointCloudSubsetLoadOptions options;
    options.stopToken = cancelled.get_token();
    const auto subset =
        invisible_places::io::LoadPointCloudSubset(path, options);
    CHECK_FALSE(subset.success);
    CHECK(subset.cancelled);

    std::stop_source progressCancellation;
    invisible_places::io::PointCloudSubsetLoadOptions duringScan;
    duringScan.stopToken = progressCancellation.get_token();
    duringScan.progress =
        [&](std::uint64_t completed, std::uint64_t) {
            if (completed > 0U) {
                progressCancellation.request_stop();
            }
        };
    const auto cancelledFromProgress =
        invisible_places::io::LoadPointCloudSubset(path, duringScan);
    CHECK_FALSE(cancelledFromProgress.success);
    CHECK(cancelledFromProgress.cancelled);

    const std::array<std::uint32_t, 3U> indices{0U, 3U, 5U};
    std::vector<std::uint64_t> indexedProgress;
    const auto gathered =
        invisible_places::io::LoadPointCloudSelectedValuesAtIndices(
            path,
            {
                .source = invisible_places::io::
                    PointCloudSelectedValueSource::ScalarField,
                .scalarFieldName = "Opacity",
            },
            indices,
            [&](std::uint64_t completed, std::uint64_t total) {
                CHECK(completed <= total);
                indexedProgress.push_back(completed);
            });
    REQUIRE(gathered.success);
    CHECK(gathered.values == std::vector<float>{0.0F, 0.3F, 0.5F});
    CHECK(gathered.stats.minimum == Catch::Approx(0.0F));
    CHECK(gathered.stats.maximum == Catch::Approx(0.5F));
    CHECK(gathered.stats.sourceIndex == 1);
    REQUIRE_FALSE(indexedProgress.empty());
    CHECK(indexedProgress.front() == 0U);
    CHECK(indexedProgress.back() == 6U);
    CHECK(std::is_sorted(
        indexedProgress.begin(),
        indexedProgress.end()));

    const std::array<std::uint32_t, 2U> unordered{3U, 2U};
    const auto rejectedOrder =
        invisible_places::io::LoadPointCloudSelectedValuesAtIndices(
            path,
            {
                .source = invisible_places::io::
                    PointCloudSelectedValueSource::ScalarField,
                .scalarFieldName = "Opacity",
            },
            unordered);
    CHECK_FALSE(rejectedOrder.success);
    CHECK_FALSE(rejectedOrder.errorMessage.empty());

    const auto cancelledGather =
        invisible_places::io::LoadPointCloudSelectedValuesAtIndices(
            path,
            {
                .source = invisible_places::io::
                    PointCloudSelectedValueSource::ScalarField,
                .scalarFieldName = "Opacity",
            },
            indices,
            {},
            cancelled.get_token());
    CHECK_FALSE(cancelledGather.success);
    CHECK(cancelledGather.cancelled);
}

TEST_CASE(
    "Linked HQ source fingerprints and stage progress support invalidation",
    "[pointcloud][linked-hq]") {
    const auto path = WriteSubsetFixture("source-fingerprint");
    invisible_places::app::LinkedHqSourceFingerprint before;
    REQUIRE(invisible_places::app::TryFingerprintLinkedHqSource(
        path,
        &before));
    invisible_places::app::LinkedHqFrustumUnion frustums;
    const std::array sources{before};
    const auto first =
        invisible_places::app::BuildLinkedHqSelectionFingerprint(
            "pair-a-b",
            frustums,
            sources);
    frustums.borderFraction = 0.06F;
    const auto borderChanged =
        invisible_places::app::BuildLinkedHqSelectionFingerprint(
            "pair-a-b",
            frustums,
            sources);
    CHECK(first != borderChanged);

    {
        std::ofstream append{path, std::ios::binary | std::ios::app};
        REQUIRE(append.is_open());
        const std::uint8_t marker = 0U;
        WriteBinary(&append, marker);
    }
    invisible_places::app::LinkedHqSourceFingerprint afterSourceChange;
    REQUIRE(invisible_places::app::TryFingerprintLinkedHqSource(
        path,
        &afterSourceChange));
    CHECK(before != afterSourceChange);
    const std::array changedSources{afterSourceChange};
    CHECK(first !=
          invisible_places::app::BuildLinkedHqSelectionFingerprint(
              "pair-a-b",
              invisible_places::app::LinkedHqFrustumUnion{},
              changedSources));

    CHECK(invisible_places::app::LinkedHqOverallProgress(
              invisible_places::app::LinkedHqPreparationStage::Scanning,
              1.0F) <=
          invisible_places::app::LinkedHqOverallProgress(
              invisible_places::app::LinkedHqPreparationStage::Organising,
              0.0F));
    CHECK(invisible_places::app::LinkedHqOverallProgress(
              invisible_places::app::LinkedHqPreparationStage::Ready,
              0.0F) == Catch::Approx(1.0F));

    float displayed = 0.0F;
    for (const auto& [stage, local] : std::array{
             std::pair{
                 invisible_places::app::LinkedHqPreparationStage::Scanning,
                 0.6F},
             std::pair{
                 invisible_places::app::LinkedHqPreparationStage::Scanning,
                 0.2F},
             std::pair{
                 invisible_places::app::LinkedHqPreparationStage::Organising,
                 0.0F},
             std::pair{
                 invisible_places::app::LinkedHqPreparationStage::Uploading,
                 0.5F},
             std::pair{
                 invisible_places::app::LinkedHqPreparationStage::Ready,
                 0.0F}}) {
        const float prior = displayed;
        displayed = std::max(
            displayed,
            invisible_places::app::LinkedHqOverallProgress(
                stage,
                local));
        CHECK(displayed >= prior);
    }
    CHECK(displayed == Catch::Approx(1.0F));
}
