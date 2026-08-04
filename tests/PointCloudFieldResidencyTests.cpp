#include "app/ScalarFieldResidencyPolicy.hpp"
#include "app/UsedScalarFields.hpp"
#include "io/PointCloudData.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

using invisible_places::io::LoadPointCloud;
using invisible_places::io::PointCloudScalarFieldFilter;

template <typename T>
void WriteBinaryValue(std::ofstream* output, const T& value) {
    output->write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(T)));
}

// Four points with RGB, normals, and four scalar fields of mixed on-disk
// types, mirroring the CloudCompare export shape the app loads (a double
// Intensity beside float fields).
constexpr std::size_t kFixturePointCount = 4U;
constexpr std::array<const char*, 4> kFixtureFieldNames{
    "Height",
    "Intensity",
    "Roughness",
    "A_Slope_deg",
};

std::filesystem::path WriteFieldFixturePly(const std::string& fileName) {
    const auto path =
        std::filesystem::temp_directory_path() /
        "invisible-places-field-residency" / fileName;
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    REQUIRE(output.is_open());
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "comment field residency fixture\n"
           << "element vertex " << kFixturePointCount << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "property float nx\n"
           << "property float ny\n"
           << "property float nz\n"
           << "property float scalar_Height\n"
           << "property double scalar_Intensity\n"
           << "property float scalar_Roughness\n"
           << "property float scalar_A_Slope_deg\n"
           << "end_header\n";
    for (std::size_t pointIndex = 0; pointIndex < kFixturePointCount; ++pointIndex) {
        const auto base = static_cast<float>(pointIndex);
        WriteBinaryValue(&output, base);               // x
        WriteBinaryValue(&output, base * 2.0F);        // y
        WriteBinaryValue(&output, base * -1.0F);       // z
        const auto channel = static_cast<std::uint8_t>(40U * pointIndex);
        WriteBinaryValue(&output, channel);            // red
        WriteBinaryValue(&output, channel);            // green
        WriteBinaryValue(&output, channel);            // blue
        WriteBinaryValue(&output, 0.0F);               // nx
        WriteBinaryValue(&output, 0.0F);               // ny
        WriteBinaryValue(&output, 1.0F);               // nz
        WriteBinaryValue(&output, 10.0F + base);       // Height
        WriteBinaryValue(&output, 100.0 + static_cast<double>(pointIndex));  // Intensity
        WriteBinaryValue(&output, 0.25F * base);       // Roughness
        WriteBinaryValue(&output, 5.0F - base);        // A_Slope_deg
    }
    return path;
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

}  // namespace

TEST_CASE("Unfiltered loads keep every field and record the full catalog",
          "[pointcloud][fields]") {
    const auto path = WriteFieldFixturePly("all-fields.ply");
    const auto result = LoadPointCloud(path);
    REQUIRE(result.success);
    const auto& cloud = result.cloud;

    REQUIRE(cloud.PointCount() == kFixturePointCount);
    REQUIRE(cloud.scalarFields.size() == kFixtureFieldNames.size());
    REQUIRE(cloud.availableScalarFields.size() == kFixtureFieldNames.size());
    for (std::size_t index = 0; index < kFixtureFieldNames.size(); ++index) {
        CHECK(cloud.scalarFields[index].name == kFixtureFieldNames[index]);
        CHECK(cloud.scalarFields[index].sourceIndex == static_cast<std::int32_t>(index));
        CHECK(cloud.scalarFields[index].valid);
        CHECK(cloud.availableScalarFields[index].name == kFixtureFieldNames[index]);
        CHECK(cloud.availableScalarFields[index].sourceIndex == index);
    }
    // The double Intensity column widens to float.
    const auto intensity = FieldColumn(cloud, "Intensity");
    REQUIRE(intensity.size() == kFixturePointCount);
    CHECK(intensity[3] == Catch::Approx(103.0F));
    CHECK(cloud.scalarFields[0].minimum == Catch::Approx(10.0F));
    CHECK(cloud.scalarFields[0].maximum == Catch::Approx(13.0F));
}

TEST_CASE("Filtered loads keep file order, full catalog, and exact values",
          "[pointcloud][fields]") {
    const auto path = WriteFieldFixturePly("filtered-fields.ply");
    const auto full = LoadPointCloud(path);
    REQUIRE(full.success);

    PointCloudScalarFieldFilter filter;
    filter.mode = PointCloudScalarFieldFilter::Mode::Selected;
    // Case-insensitive, order-independent, unknown and generated names
    // ignored.
    filter.names = {"a_slope_deg", "water_effect_value", "HEIGHT"};
    const auto filtered = LoadPointCloud(path, filter);
    REQUIRE(filtered.success);
    const auto& cloud = filtered.cloud;

    REQUIRE(cloud.scalarFields.size() == 2U);
    CHECK(cloud.scalarFields[0].name == "Height");
    CHECK(cloud.scalarFields[0].sourceIndex == 0);
    CHECK(cloud.scalarFields[1].name == "A_Slope_deg");
    CHECK(cloud.scalarFields[1].sourceIndex == 3);
    CHECK(cloud.scalarFieldValues.size() == 2U * kFixturePointCount);
    CHECK(cloud.availableScalarFields.size() == kFixtureFieldNames.size());

    CHECK(FieldColumn(cloud, "Height") == FieldColumn(full.cloud, "Height"));
    CHECK(FieldColumn(cloud, "A_Slope_deg") ==
          FieldColumn(full.cloud, "A_Slope_deg"));
    REQUIRE(cloud.positions.size() == full.cloud.positions.size());
    CHECK(cloud.positions[2].x == Catch::Approx(full.cloud.positions[2].x));
    CHECK(cloud.packedColors == full.cloud.packedColors);
    CHECK(cloud.hasNormals);
    CHECK(cloud.scalarFields[0].minimum == Catch::Approx(10.0F));
    CHECK(cloud.scalarFields[0].maximum == Catch::Approx(13.0F));

    CHECK(cloud.ResidentSlotForSourceIndex(0).value() == 0U);
    CHECK_FALSE(cloud.ResidentSlotForSourceIndex(1).has_value());
    CHECK_FALSE(cloud.ResidentSlotForSourceIndex(2).has_value());
    CHECK(cloud.ResidentSlotForSourceIndex(3).value() == 1U);
    CHECK_FALSE(cloud.ResidentSlotForSourceIndex(-1).has_value());
}

TEST_CASE("Filter source indices and contains patterns select fields",
          "[pointcloud][fields]") {
    const auto path = WriteFieldFixturePly("index-pattern-fields.ply");

    PointCloudScalarFieldFilter filter;
    filter.mode = PointCloudScalarFieldFilter::Mode::Selected;
    // Index 1 mimics a persisted caustic file-order slot; the pattern keeps
    // the renderer's always-resident roughness lookup satisfied.
    filter.sourceIndices = {1U};
    filter.containsPatterns = {"roughness"};
    const auto result = LoadPointCloud(path, filter);
    REQUIRE(result.success);
    const auto& cloud = result.cloud;

    REQUIRE(cloud.scalarFields.size() == 2U);
    CHECK(cloud.scalarFields[0].name == "Intensity");
    CHECK(cloud.scalarFields[0].sourceIndex == 1);
    CHECK(cloud.scalarFields[1].name == "Roughness");
    CHECK(cloud.scalarFields[1].sourceIndex == 2);
}

TEST_CASE("An empty selected filter loads geometry only",
          "[pointcloud][fields]") {
    const auto path = WriteFieldFixturePly("geometry-only.ply");

    PointCloudScalarFieldFilter filter;
    filter.mode = PointCloudScalarFieldFilter::Mode::Selected;
    const auto result = LoadPointCloud(path, filter);
    REQUIRE(result.success);
    const auto& cloud = result.cloud;

    CHECK(cloud.scalarFields.empty());
    CHECK(cloud.scalarFieldValues.empty());
    CHECK(cloud.availableScalarFields.size() == kFixtureFieldNames.size());
    CHECK(cloud.PointCount() == kFixturePointCount);
    CHECK(cloud.hasSourceRgb);
    CHECK(cloud.hasNormals);
    CHECK(cloud.bounds.valid);
}

TEST_CASE("Streaming a selected field matches the loaded column",
          "[pointcloud][fields]") {
    const auto path = WriteFieldFixturePly("stream-equivalence.ply");
    const auto full = LoadPointCloud(path);
    REQUIRE(full.success);
    const auto expected = FieldColumn(full.cloud, "A_Slope_deg");
    REQUIRE(expected.size() == kFixturePointCount);

    std::vector<float> streamed(kFixturePointCount, 0.0F);
    invisible_places::io::ScalarFieldStats stats;
    stats.name = "A_Slope_deg";
    const auto streamResult =
        invisible_places::io::StreamPointCloudSelectedValues(
            path,
            {.source = invisible_places::io::PointCloudSelectedValueSource::
                 ScalarField,
             .scalarFieldName = "A_Slope_deg"},
            [&](float value, std::uint64_t pointIndex) {
                streamed[static_cast<std::size_t>(pointIndex)] = value;
                stats.Include(value);
                return true;
            });
    REQUIRE(streamResult.success);
    CHECK(streamResult.pointCount == kFixturePointCount);
    CHECK(streamed == expected);
    CHECK(stats.minimum == Catch::Approx(2.0F));
    CHECK(stats.maximum == Catch::Approx(5.0F));
}

TEST_CASE("Used scalar field sets aggregate bindings and Visual Features",
          "[pointcloud][fields][used]") {
    invisible_places::app::UsedScalarFieldSet used;
    CHECK(used.Empty());

    invisible_places::style::RenderParameterBinding mapped;
    mapped.mode = invisible_places::style::ParameterSourceMode::FieldMapped;
    mapped.fieldMap.fieldName = "Height";
    used.AddBinding(mapped);

    // A binding parked on Constant still contributes its remembered field:
    // flipping the mode back must not wait on a disk load.
    invisible_places::style::RenderParameterBinding constant;
    constant.mode = invisible_places::style::ParameterSourceMode::Constant;
    constant.fieldMap.fieldName = "A_Slope_deg";
    used.AddBinding(constant);

    invisible_places::style::RenderParameterBinding unnamed;
    used.AddBinding(unnamed);

    invisible_places::timing::TimingColouriseEffect scalarEffect;
    scalarEffect.enabled = false;  // dormant effects still contribute
    scalarEffect.field.source =
        invisible_places::timing::TimingColouriseFieldSource::Scalar;
    scalarEffect.field.scalarFieldName = "Intensity";
    used.AddColouriseEffect(scalarEffect);

    invisible_places::timing::TimingColouriseEffect normalEffect;
    normalEffect.field.source =
        invisible_places::timing::TimingColouriseFieldSource::NormalX;
    normalEffect.field.scalarFieldName = "IgnoredForNormals";
    used.AddColouriseEffect(normalEffect);

    REQUIRE(used.Names().size() == 3U);
    CHECK(used.Names()[0] == "Height");
    CHECK(used.Names()[1] == "A_Slope_deg");
    CHECK(used.Names()[2] == "Intensity");
    CHECK(used.Contains("height"));
    CHECK(used.Contains("a_slope_deg"));
    CHECK(used.Contains("ASlopeDeg"));  // normalization ignores punctuation
    CHECK_FALSE(used.Contains("IgnoredForNormals"));
    CHECK_FALSE(used.Contains("Roughness"));

    // Duplicate spellings of an existing entry collapse.
    used.AddFieldName("HEIGHT");
    CHECK(used.Names().size() == 3U);

    const auto& patterns =
        invisible_places::app::AlwaysResidentScalarFieldPatterns();
    CHECK(std::find(patterns.begin(), patterns.end(), "roughness") !=
          patterns.end());
}

TEST_CASE("The legacy water-shader span keeps file fields at their file slots",
          "[pointcloud][fields][compat]") {
    // The point shaders read hard-coded slots (water jitter seed 12 up to
    // feature type 15) on flow-animated clouds, so filtered loads pin file
    // fields 0..15 to resident slots 0..15 by whitelisting their source
    // indices. This fixture has 20 fields with values encoding their file
    // index, so both alignment and content are checkable.
    constexpr std::size_t kFieldCount = 20U;
    constexpr std::size_t kPoints = 3U;
    const auto path =
        std::filesystem::temp_directory_path() /
        "invisible-places-field-residency" / "legacy-span.ply";
    std::filesystem::create_directories(path.parent_path());
    {
        std::ofstream output{path, std::ios::binary | std::ios::trunc};
        REQUIRE(output.is_open());
        output << "ply\n"
               << "format binary_little_endian 1.0\n"
               << "element vertex " << kPoints << "\n"
               << "property float x\n"
               << "property float y\n"
               << "property float z\n";
        for (std::size_t field = 0; field < kFieldCount; ++field) {
            output << "property float scalar_F"
                   << (field < 10U ? "0" : "") << field << "\n";
        }
        output << "end_header\n";
        for (std::size_t point = 0; point < kPoints; ++point) {
            WriteBinaryValue(&output, static_cast<float>(point));
            WriteBinaryValue(&output, 0.0F);
            WriteBinaryValue(&output, 0.0F);
            for (std::size_t field = 0; field < kFieldCount; ++field) {
                WriteBinaryValue(
                    &output,
                    static_cast<float>(field) * 1000.0F +
                        static_cast<float>(point));
            }
        }
    }

    PointCloudScalarFieldFilter filter;
    filter.mode = PointCloudScalarFieldFilter::Mode::Selected;
    filter.names = {"F18"};
    for (std::uint32_t sourceIndex = 0U;
         sourceIndex <
         invisible_places::app::kLegacyWaterShaderCompatibilitySourceIndexCount;
         ++sourceIndex) {
        filter.sourceIndices.push_back(sourceIndex);
    }
    const auto result = LoadPointCloud(path, filter);
    REQUIRE(result.success);
    const auto& cloud = result.cloud;

    REQUIRE(cloud.scalarFields.size() ==
            invisible_places::app::kLegacyWaterShaderCompatibilitySourceIndexCount + 1U);
    for (std::uint32_t slot = 0U;
         slot <
         invisible_places::app::kLegacyWaterShaderCompatibilitySourceIndexCount;
         ++slot) {
        // Resident slot == file index across the whole compatibility span,
        // so the shader's fixed-slot reads hit the same fields they always
        // did.
        CHECK(cloud.scalarFields[slot].sourceIndex ==
              static_cast<std::int32_t>(slot));
        CHECK(cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(slot, 1U)] ==
              Catch::Approx(static_cast<float>(slot) * 1000.0F + 1.0F));
    }
    const auto& tail = cloud.scalarFields.back();
    CHECK(tail.name == "F18");
    CHECK(tail.sourceIndex == 18);
}

TEST_CASE("Parallel PLY parsing matches the single-threaded result exactly",
          "[pointcloud][fields][parallel]") {
    // Enough points that four forced ranges each cover thousands of
    // records, with values varied per point and a NaN mixed in so the
    // stats-merge and non-finite paths are exercised.
    constexpr std::size_t kParallelPointCount = 10'000U;
    const auto path =
        std::filesystem::temp_directory_path() /
        "invisible-places-field-residency" / "parallel-parse.ply";
    std::filesystem::create_directories(path.parent_path());
    {
        std::ofstream output{path, std::ios::binary | std::ios::trunc};
        REQUIRE(output.is_open());
        output << "ply\n"
               << "format binary_little_endian 1.0\n"
               << "element vertex " << kParallelPointCount << "\n"
               << "property float x\n"
               << "property float y\n"
               << "property float z\n"
               << "property uchar red\n"
               << "property uchar green\n"
               << "property uchar blue\n"
               << "property float nx\n"
               << "property float ny\n"
               << "property float nz\n"
               << "property float scalar_Height\n"
               << "property double scalar_Intensity\n"
               << "end_header\n";
        for (std::size_t pointIndex = 0; pointIndex < kParallelPointCount; ++pointIndex) {
            const auto base = static_cast<float>(pointIndex);
            WriteBinaryValue(&output, std::sin(base * 0.37F) * 40.0F);
            WriteBinaryValue(&output, std::cos(base * 0.11F) * 25.0F);
            WriteBinaryValue(&output, base * 0.001F);
            WriteBinaryValue(&output, static_cast<std::uint8_t>(pointIndex % 251U));
            WriteBinaryValue(&output, static_cast<std::uint8_t>(pointIndex % 83U));
            WriteBinaryValue(&output, static_cast<std::uint8_t>(pointIndex % 17U));
            WriteBinaryValue(&output, std::sin(base));
            WriteBinaryValue(&output, std::cos(base));
            WriteBinaryValue(&output, 0.5F);
            WriteBinaryValue(
                &output,
                pointIndex == 4'321U
                    ? std::numeric_limits<float>::quiet_NaN()
                    : std::fmod(base * 1.7F, 90.0F));
            WriteBinaryValue(&output, 1000.0 + static_cast<double>(pointIndex % 777U));
        }
    }

    const auto sequential = LoadPointCloud(path, {}, 1U);
    const auto parallel = LoadPointCloud(path, {}, 4U);
    REQUIRE(sequential.success);
    REQUIRE(parallel.success);

    CHECK(parallel.cloud.PointCount() == kParallelPointCount);
    // Bitwise equality: the fixture plants a NaN, which float == would
    // reject even for identical payloads.
    REQUIRE(parallel.cloud.scalarFieldValues.size() ==
            sequential.cloud.scalarFieldValues.size());
    CHECK(std::memcmp(
              parallel.cloud.scalarFieldValues.data(),
              sequential.cloud.scalarFieldValues.data(),
              sequential.cloud.scalarFieldValues.size() * sizeof(float)) == 0);
    CHECK(parallel.cloud.packedColors == sequential.cloud.packedColors);
    REQUIRE(parallel.cloud.positions.size() ==
            sequential.cloud.positions.size());
    for (std::size_t index = 0; index < parallel.cloud.positions.size(); ++index) {
        CHECK(parallel.cloud.positions[index].x ==
              sequential.cloud.positions[index].x);
        CHECK(parallel.cloud.normals[index].y ==
              sequential.cloud.normals[index].y);
    }
    REQUIRE(parallel.cloud.scalarFields.size() ==
            sequential.cloud.scalarFields.size());
    for (std::size_t slot = 0; slot < parallel.cloud.scalarFields.size(); ++slot) {
        const auto& left = parallel.cloud.scalarFields[slot];
        const auto& right = sequential.cloud.scalarFields[slot];
        CHECK(left.name == right.name);
        CHECK(left.sourceIndex == right.sourceIndex);
        CHECK(left.minimum == right.minimum);
        CHECK(left.maximum == right.maximum);
        CHECK(left.count == right.count);  // NaN skipped identically
        CHECK(left.valid == right.valid);
    }
    CHECK(parallel.cloud.bounds.minimum.x == sequential.cloud.bounds.minimum.x);
    CHECK(parallel.cloud.bounds.maximum.z == sequential.cloud.bounds.maximum.z);
    CHECK(parallel.cloud.focusPoint.x == sequential.cloud.focusPoint.x);
    CHECK(parallel.cloud.focusPoint.y == sequential.cloud.focusPoint.y);
    CHECK(parallel.cloud.focusPoint.z == sequential.cloud.focusPoint.z);

    // Filtered parallel loads agree with filtered sequential loads too.
    PointCloudScalarFieldFilter filter;
    filter.mode = PointCloudScalarFieldFilter::Mode::Selected;
    filter.names = {"Intensity"};
    const auto filteredSequential = LoadPointCloud(path, filter, 1U);
    const auto filteredParallel = LoadPointCloud(path, filter, 3U);
    REQUIRE(filteredSequential.success);
    REQUIRE(filteredParallel.success);
    CHECK(filteredParallel.cloud.scalarFieldValues ==
          filteredSequential.cloud.scalarFieldValues);
    REQUIRE(filteredParallel.cloud.scalarFields.size() == 1U);
    CHECK(filteredParallel.cloud.scalarFields[0].minimum ==
          filteredSequential.cloud.scalarFields[0].minimum);
}

TEST_CASE("Scalar-field eviction frees the least recently referenced fields",
          "[pointcloud][fields][residency]") {
    using invisible_places::app::ScalarFieldResidencyCandidate;
    using invisible_places::app::SelectScalarFieldEvictions;

    const std::vector<ScalarFieldResidencyCandidate> candidates{
        // index 0: required — never selected, however old.
        {.sessionIndex = 0, .fieldName = "Height", .bytes = 100,
         .lastRequiredTick = 1, .required = true, .evictable = true},
        // index 1: generated — not evictable.
        {.sessionIndex = 0, .fieldName = "water_effect_value", .bytes = 100,
         .lastRequiredTick = 0, .required = false, .evictable = false},
        // index 2: oldest evictable.
        {.sessionIndex = 0, .fieldName = "A_Slope_deg", .bytes = 100,
         .lastRequiredTick = 2, .required = false, .evictable = true},
        // index 3: newer evictable on another session.
        {.sessionIndex = 1, .fieldName = "Curvature", .bytes = 100,
         .lastRequiredTick = 9, .required = false, .evictable = true},
        // index 4: never stamped — evicts before everything stamped.
        {.sessionIndex = 1, .fieldName = "Anisotropy", .bytes = 100,
         .lastRequiredTick = 0, .required = false, .evictable = true},
    };

    SECTION("a zero budget disables eviction") {
        CHECK(SelectScalarFieldEvictions(candidates, 500, 0).empty());
    }
    SECTION("under budget selects nothing") {
        CHECK(SelectScalarFieldEvictions(candidates, 500, 500).empty());
    }
    SECTION("evicts oldest-first until the projection meets the budget") {
        const auto selected = SelectScalarFieldEvictions(candidates, 500, 350);
        REQUIRE(selected.size() == 2U);
        CHECK(selected[0] == 4U);  // never stamped
        CHECK(selected[1] == 2U);  // oldest stamped
    }
    SECTION("required and non-evictable fields survive any deficit") {
        const auto selected = SelectScalarFieldEvictions(candidates, 500, 1);
        REQUIRE(selected.size() == 3U);
        CHECK(selected[0] == 4U);
        CHECK(selected[1] == 2U);
        CHECK(selected[2] == 3U);
    }
    SECTION("empty candidate lists are harmless") {
        CHECK(SelectScalarFieldEvictions({}, 500, 10).empty());
    }
}
