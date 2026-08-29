#include "app/LinkedHighQualityPreview.hpp"
#include "io/PointCloudData.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
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
    "Animation HQ retains a full unlinked view set and a linked pair",
    "[pointcloud][linked-hq]") {
    const auto viewAt = [](float x) {
        return glm::translate(
            glm::mat4{1.0F},
            glm::vec3{-x, 0.0F, 0.0F});
    };
    const std::array fullPathViews{
        viewAt(0.0F),
        viewAt(3.0F),
        viewAt(6.0F),
        viewAt(9.0F),
        viewAt(12.0F),
        viewAt(0.0F),
    };
    const auto fullPath =
        invisible_places::app::BuildAnimationHqFrustumUnion(
            fullPathViews);
    REQUIRE(fullPath.viewProjections.size() == 5U);
    CHECK(fullPath.viewProjections.front() == fullPathViews.front());
    CHECK(fullPath.viewProjections.back() == fullPathViews[4U]);
    CHECK(fullPath.Contains({0.0F, 0.0F, 0.0F}));
    CHECK(fullPath.Contains({6.0F, 0.0F, 0.0F}));
    // This point is visible only in a view beyond the old fixed two slots.
    CHECK(fullPath.Contains({12.0F, 0.0F, 0.0F}));

    const std::array linkedMidpoints{viewAt(2.0F), viewAt(-3.0F)};
    const auto linked = invisible_places::app::BuildAnimationHqFrustumUnion(
        linkedMidpoints);
    REQUIRE(linked.viewProjections.size() == 2U);
    CHECK(linked.viewProjections[0U] == linkedMidpoints[0U]);
    CHECK(linked.viewProjections[1U] == linkedMidpoints[1U]);
    CHECK(linked.Contains({-3.0F, 0.0F, 0.0F}));
    CHECK(linked.Contains({2.0F, 0.0F, 0.0F}));
}

TEST_CASE(
    "Unlinked animation HQ samples the complete path and authored keys",
    "[pointcloud][linked-hq][unlinked-hq]") {
    const std::array authoredKeyTimes{0.1F, 0.5F, 0.9F};
    const auto times =
        invisible_places::app::BuildUnlinkedAnimationHqSampleTimes(
            authoredKeyTimes,
            1.0F);
    REQUIRE(times.size() ==
            invisible_places::app::kUnlinkedAnimationHqUniformViewCount +
                2U);
    CHECK(std::is_sorted(times.begin(), times.end()));
    CHECK(times.front() == Catch::Approx(0.0F));
    CHECK(times.back() == Catch::Approx(1.0F));
    CHECK(std::count_if(
              times.begin(),
              times.end(),
              [](float time) {
                  return time == Catch::Approx(0.1F);
              }) == 1);
    CHECK(std::count_if(
              times.begin(),
              times.end(),
              [](float time) {
                  return time == Catch::Approx(0.5F);
              }) == 1);
    CHECK(std::count_if(
              times.begin(),
              times.end(),
              [](float time) {
                  return time == Catch::Approx(0.9F);
              }) == 1);

    const auto still =
        invisible_places::app::BuildUnlinkedAnimationHqSampleTimes(
            authoredKeyTimes,
            0.0F);
    CHECK(still == std::vector<float>{0.0F});
}

TEST_CASE(
    "Linked HQ midpoint union pads only the viewport and partitions exactly",
    "[pointcloud][linked-hq]") {
    invisible_places::app::LinkedHqFrustumUnion frustums;
    frustums.viewProjections = {
        glm::mat4{1.0F},
        glm::translate(
            glm::mat4{1.0F},
            glm::vec3{-3.0F, 0.0F, 0.0F}),
    };

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
    "Adaptive HQ guard covers its camera and limits fine view depth",
    "[pointcloud][adaptive-hq]") {
    const glm::mat4 view = glm::lookAt(
        glm::vec3{0.0F, 0.0F, 0.0F},
        glm::vec3{0.0F, 0.0F, -1.0F},
        glm::vec3{0.0F, 1.0F, 0.0F});
    const glm::mat4 projection = glm::perspective(
        glm::radians(60.0F),
        1.0F,
        0.1F,
        100.0F);
    auto guard = invisible_places::app::BuildAnimationHqFrustumUnion(
        std::array{projection * view},
        invisible_places::app::kAdaptiveHqViewportGuardFraction);
    guard.maximumViewDepthMeters = 10.0F;

    CHECK(invisible_places::app::LinkedHqFrustumUnionCoversView(
        guard,
        view,
        projection,
        8.0F));
    CHECK(invisible_places::app::LinkedHqFrustumUnionCoversView(
        guard,
        view,
        projection,
        8.0F,
        invisible_places::app::kAdaptiveHqRefreshBorderFraction));
    CHECK_FALSE(invisible_places::app::LinkedHqFrustumUnionCoversView(
        guard,
        view,
        projection,
        8.0F,
        invisible_places::app::kAdaptiveHqViewportGuardFraction + 0.01F));
    CHECK(guard.Contains({0.0F, 0.0F, -5.0F}));
    CHECK_FALSE(guard.Contains({0.0F, 0.0F, -11.0F}));
    CHECK(guard.IntersectsBounds({
        .minimum = {-0.5F, -0.5F, -5.5F},
        .maximum = {0.5F, 0.5F, -4.5F},
        .valid = true,
    }));
    CHECK(guard.IntersectsBounds({
        .minimum = {-0.5F, -0.5F, -10.5F},
        .maximum = {0.5F, 0.5F, -9.5F},
        .valid = true,
    }));
    CHECK_FALSE(guard.IntersectsBounds({
        .minimum = {50.0F, -0.5F, -5.5F},
        .maximum = {51.0F, 0.5F, -4.5F},
        .valid = true,
    }));
    CHECK_FALSE(guard.IntersectsBounds({
        .minimum = {-0.5F, -0.5F, -12.0F},
        .maximum = {0.5F, 0.5F, -11.0F},
        .valid = true,
    }));

    const glm::mat4 movedView = glm::lookAt(
        glm::vec3{20.0F, 0.0F, 0.0F},
        glm::vec3{20.0F, 0.0F, -1.0F},
        glm::vec3{0.0F, 1.0F, 0.0F});
    CHECK_FALSE(invisible_places::app::LinkedHqFrustumUnionCoversView(
        guard,
        movedView,
        projection,
        8.0F));
}

TEST_CASE(
    "Adaptive HQ retains the prior fine patch over complete coarse fallback",
    "[pointcloud][adaptive-hq]") {
    using invisible_places::app::ResolveLinkedHqPatchDrawPolicy;

    const auto hidden = ResolveLinkedHqPatchDrawPolicy(
        false,
        true,
        false,
        true);
    CHECK_FALSE(hidden.renderFinePatches);
    CHECK_FALSE(hidden.applyFineAdaptiveDensity);
    CHECK_FALSE(hidden.applyCoarseAdaptiveDensity);
    CHECK_FALSE(hidden.retainingFineDuringRefresh);

    const auto fixed = ResolveLinkedHqPatchDrawPolicy(
        true,
        false,
        false,
        false);
    CHECK(fixed.renderFinePatches);
    CHECK_FALSE(fixed.applyFineAdaptiveDensity);
    CHECK_FALSE(fixed.applyCoarseAdaptiveDensity);
    CHECK_FALSE(fixed.retainingFineDuringRefresh);

    const auto paired = ResolveLinkedHqPatchDrawPolicy(
        true,
        true,
        true,
        true);
    CHECK(paired.renderFinePatches);
    CHECK(paired.applyFineAdaptiveDensity);
    CHECK(paired.applyCoarseAdaptiveDensity);
    CHECK_FALSE(paired.retainingFineDuringRefresh);

    const auto refreshing = ResolveLinkedHqPatchDrawPolicy(
        true,
        true,
        false,
        true);
    CHECK(refreshing.renderFinePatches);
    CHECK(refreshing.applyFineAdaptiveDensity);
    CHECK_FALSE(refreshing.applyCoarseAdaptiveDensity);
    CHECK(refreshing.retainingFineDuringRefresh);

    const auto invalidTransition = ResolveLinkedHqPatchDrawPolicy(
        true,
        true,
        false,
        false);
    CHECK(invalidTransition.renderFinePatches);
    CHECK_FALSE(invalidTransition.applyFineAdaptiveDensity);
    CHECK_FALSE(invalidTransition.applyCoarseAdaptiveDensity);
    CHECK(invalidTransition.retainingFineDuringRefresh);
}

TEST_CASE(
    "Adaptive HQ resident retention uses a byte-bounded LRU working set",
    "[pointcloud][adaptive-hq][retention]") {
    const auto makeBlock = [](
        std::uint32_t index,
        std::uint64_t serial,
        std::size_t pointCount,
        std::size_t scalarCount = 0U) {
        invisible_places::io::PointCloudSubsetLoadResult subset;
        subset.cloud.positions.resize(pointCount);
        subset.cloud.normals.resize(pointCount);
        subset.cloud.packedColors.resize(pointCount);
        subset.sourcePointIndices.resize(pointCount);
        subset.cloud.scalarFieldValues.resize(pointCount * scalarCount);
        return invisible_places::io::AdaptiveHqResidentBlock{
            .blockIndex = index,
            .points = std::make_shared<const
                invisible_places::io::PointCloudSubsetLoadResult>(
                    std::move(subset)),
            .lastUsedSerial = serial,
        };
    };

    std::vector<invisible_places::io::AdaptiveHqResidentBlock> candidates;
    candidates.push_back(makeBlock(4U, 1U, 10U));
    candidates.push_back(makeBlock(1U, 5U, 10U, 1U));
    candidates.push_back(makeBlock(3U, 4U, 10U));
    candidates.push_back(makeBlock(2U, 3U, 10U));
    const std::array<std::uint32_t, 1U> active{4U};
    const auto oneScalarBytes =
        invisible_places::app::AdaptiveHqResidentBlockPayloadBytes(
            candidates[1U]);
    const auto geometryBytes =
        invisible_places::app::AdaptiveHqResidentBlockPayloadBytes(
            candidates[2U]);
    CHECK(oneScalarBytes == 360U);
    CHECK(geometryBytes == 320U);

    const auto retained =
        invisible_places::app::RetainAdaptiveHqResidentBlocks(
            candidates,
            active,
            oneScalarBytes + geometryBytes);
    REQUIRE(retained.blocks.size() == 3U);
    CHECK(retained.blocks[0U].blockIndex == 4U);
    CHECK(retained.blocks[1U].blockIndex == 1U);
    CHECK(retained.blocks[2U].blockIndex == 3U);
    CHECK(retained.inactiveBlockCount == 2U);
    CHECK(retained.inactivePayloadBytes ==
          oneScalarBytes + geometryBytes);
    CHECK(retained.activePayloadBytes == geometryBytes);

    // Active data is never evicted even when the inactive budget is zero.
    const auto activeOnly =
        invisible_places::app::RetainAdaptiveHqResidentBlocks(
            candidates,
            active,
            0U);
    REQUIRE(activeOnly.blocks.size() == 1U);
    CHECK(activeOnly.blocks.front().blockIndex == 4U);
    CHECK(activeOnly.inactivePayloadBytes == 0U);
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
    "Linked HQ patch spacing choices map to a kept fraction",
    "[linked-hq]") {
    using Catch::Approx;
    using invisible_places::app::LinkedHqPatchKeepFraction;
    using invisible_places::app::SanitizeLinkedHqPatchSpacing;
    CHECK(SanitizeLinkedHqPatchSpacing(0U) == 1'000U);
    CHECK(SanitizeLinkedHqPatchSpacing(1'000U) == 1'000U);
    CHECK(SanitizeLinkedHqPatchSpacing(1'500U) == 1'000U);
    CHECK(SanitizeLinkedHqPatchSpacing(2'000U) == 2'000U);
    CHECK(SanitizeLinkedHqPatchSpacing(2'999U) == 2'000U);
    CHECK(SanitizeLinkedHqPatchSpacing(3'000U) == 3'000U);
    CHECK(SanitizeLinkedHqPatchSpacing(5'000U) == 3'000U);
    CHECK(LinkedHqPatchKeepFraction(1'000U) == Approx(1.0F));
    CHECK(LinkedHqPatchKeepFraction(2'000U) == Approx(0.25F));
    CHECK(LinkedHqPatchKeepFraction(3'000U) == Approx(1.0F / 9.0F));
    CHECK(LinkedHqPatchKeepFraction(999'999U) == Approx(1.0F / 9.0F));
}

namespace {

// A 1 mm lattice on a plane tilted out of every axis, so cells straddle the
// surface at varying occupancy like a real scan does.
std::filesystem::path WriteLatticeFixture(
    std::string_view stem,
    std::uint32_t side) {
    const auto path =
        std::filesystem::temp_directory_path() /
        "invisible-places-linked-hq" /
        (std::string{stem} + ".ply");
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    REQUIRE(output.is_open());
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << (side * side) << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "property float scalar_Height\n"
           << "end_header\n";
    for (std::uint32_t row = 0U; row < side; ++row) {
        for (std::uint32_t column = 0U; column < side; ++column) {
            const float x = 0.0003F + static_cast<float>(column) * 0.001F;
            const float y = 0.0007F + static_cast<float>(row) * 0.001F;
            const float z = 0.37F * x + 0.21F * y + 0.0001F;
            const auto shade = static_cast<std::uint8_t>((row * 7U + column * 3U) % 256U);
            WriteBinary(&output, x);
            WriteBinary(&output, y);
            WriteBinary(&output, z);
            WriteBinary(&output, shade);
            WriteBinary(&output, shade);
            WriteBinary(&output, shade);
            WriteBinary(&output, z);
        }
    }
    return path;
}

}  // namespace

TEST_CASE(
    "Grid decimation keeps one centred parent per populated cell",
    "[pointcloud][linked-hq][io]") {
    using Catch::Approx;
    constexpr std::uint32_t kSide = 48U;
    const auto path = WriteLatticeFixture("grid-decimation", kSide);
    const auto load = [&](float cellSize, float keepFraction, unsigned threadCount) {
        invisible_places::io::PointCloudSubsetLoadOptions options;
        options.fieldFilter.mode =
            invisible_places::io::PointCloudScalarFieldFilter::Mode::Selected;
        options.fieldFilter.names = {"Height"};
        options.gridDecimation = {
            .cellSizeMeters = cellSize,
            .keepFraction = keepFraction,
        };
        options.threadCount = threadCount;
        return invisible_places::io::LoadPointCloudSubset(path, options);
    };

    const auto full = load(0.0F, 1.0F, 1U);
    REQUIRE(full.success);
    CHECK(full.includedPointCount == kSide * kSide);
    CHECK(full.cloud.PointCount() == kSide * kSide);

    // 2 mm cells over a 1 mm lattice hold up to four parents (the tilted
    // plane also crosses z-cell boundaries, leaving some cells with fewer),
    // so no cell keeps more than one point and the dithered quotas make the
    // total a quarter of the lattice within a few points.
    const auto two = load(0.002F, 0.25F, 1U);
    REQUIRE(two.success);
    CHECK(two.includedPointCount == kSide * kSide);
    const double expectedTwo = static_cast<double>(kSide * kSide) / 4.0;
    CHECK(static_cast<double>(two.cloud.PointCount()) ==
          Approx(expectedTwo).margin(expectedTwo * 0.05));
    CHECK(two.sourcePointIndices.size() == two.cloud.PointCount());
    CHECK(std::is_sorted(two.sourcePointIndices.begin(), two.sourcePointIndices.end()));
    // Kept points keep their own attributes (no averaging).
    for (std::size_t point = 0U; point < two.cloud.PointCount(); ++point) {
        const auto source = two.sourcePointIndices[point];
        CHECK(two.cloud.positions[point].x == full.cloud.positions[source].x);
        CHECK(two.cloud.packedColors[point] == full.cloud.packedColors[source]);
        CHECK(FieldValues(two.cloud, "Height")[point] ==
              FieldValues(full.cloud, "Height")[source]);
    }
    // At most one point per 2 mm cell (half-offset cubic grid).
    std::set<std::array<int, 3U>> cells;
    for (const auto& position : two.cloud.positions) {
        cells.insert({
            static_cast<int>(std::floor((position.x - 0.001F) / 0.002F)),
            static_cast<int>(std::floor((position.y - 0.001F) / 0.002F)),
            static_cast<int>(std::floor((position.z - 0.001F) / 0.002F)),
        });
    }
    CHECK(cells.size() == two.cloud.PointCount());
    REQUIRE(two.cloud.scalarFields.size() == 1U);
    CHECK(two.cloud.scalarFields.front().count == two.cloud.PointCount());
    CHECK(two.cloud.bounds.valid);

    // Thread/bucket count must not change the selection.
    const auto twoParallel = load(0.002F, 0.25F, 5U);
    REQUIRE(twoParallel.success);
    CHECK(twoParallel.sourcePointIndices == two.sourcePointIndices);
    CHECK(FieldValues(twoParallel.cloud, "Height") == FieldValues(two.cloud, "Height"));

    // 3 mm cells: interior cells hold up to nine parents (one output); edge
    // and z-split cells hold fewer and round by the dithered quota, so the
    // total stays near a ninth of the lattice.
    const auto three = load(0.003F, 1.0F / 9.0F, 3U);
    REQUIRE(three.success);
    const double expectedThree = static_cast<double>(kSide * kSide) / 9.0;
    CHECK(static_cast<double>(three.cloud.PointCount()) ==
          Approx(expectedThree).margin(expectedThree * 0.08));
    CHECK(three.cloud.PointCount() < two.cloud.PointCount());
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
    auto viewsChanged = frustums;
    viewsChanged.viewProjections.push_back(glm::translate(
        glm::mat4{1.0F},
        glm::vec3{-3.0F, 0.0F, 0.0F}));
    CHECK(first !=
          invisible_places::app::BuildLinkedHqSelectionFingerprint(
              "pair-a-b",
              viewsChanged,
              sources));
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

TEST_CASE(
    "Animation section frustum samples cover every playback frame with boundary padding",
    "[pointcloud][animation][frustum-culling]") {
    const auto times = invisible_places::app::
        BuildAnimationSectionFrustumSampleTimes(
            10.0F,
            0.40F,
            0.50F,
            30U,
            2U);
    REQUIRE_FALSE(times.empty());
    CHECK(std::is_sorted(times.begin(), times.end()));
    CHECK(times.front() == Catch::Approx(118.0F / 30.0F));
    CHECK(times.back() == Catch::Approx(152.0F / 30.0F));
    CHECK(std::find_if(
              times.begin(),
              times.end(),
              [](float time) {
                  return std::abs(time - 4.0F) <= 1.0e-6F;
              }) != times.end());
    CHECK(std::find_if(
              times.begin(),
              times.end(),
              [](float time) {
                  return std::abs(time - 5.0F) <= 1.0e-6F;
              }) != times.end());

    const auto reversed = invisible_places::app::
        BuildAnimationSectionFrustumSampleTimes(
            10.0F,
            0.50F,
            0.40F,
            30U,
            2U);
    CHECK(reversed == times);

    const auto degenerate = invisible_places::app::
        BuildAnimationSectionFrustumSampleTimes(
            0.0F,
            0.25F,
            0.75F);
    REQUIRE(degenerate.size() == 1U);
    CHECK(degenerate.front() == 0.0F);
}
