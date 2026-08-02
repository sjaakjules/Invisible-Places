#include "timing/TimingColouriseHistogram.hpp"

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
#include <stop_token>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using invisible_places::timing::TimingColouriseFieldFamily;
using invisible_places::timing::TimingColouriseFieldSelector;
using invisible_places::timing::TimingColouriseFieldSource;
using invisible_places::timing::TimingColouriseHistogramFingerprintInput;
using invisible_places::timing::TimingColouriseHistogramSourceIdentity;
using invisible_places::timing::TimingColouriseLayerFieldSet;

invisible_places::timing::TimingColouriseHistogram MakeHistogram(
    float minimum,
    float maximum,
    const std::array<
        std::uint64_t,
        invisible_places::timing::
            kTimingColouriseHistogramBinCount>& bins) {
    invisible_places::timing::TimingColouriseHistogram histogram{
        .minimum = minimum,
        .maximum = maximum,
        .bins = std::vector<std::uint64_t>(
            bins.begin(),
            bins.end()),
    };
    for (const auto count : bins) {
        histogram.finiteValueCount += count;
    }
    return histogram;
}

const TimingColouriseFieldFamily* FindFamily(
    const std::vector<TimingColouriseFieldFamily>& catalog,
    std::string_view name) {
    const auto found = std::find_if(
        catalog.begin(),
        catalog.end(),
        [&](const auto& family) {
            return family.name == name;
        });
    return found == catalog.end() ? nullptr : &*found;
}

invisible_places::io::LoadedPointCloud MakeCloud(
    std::vector<float> values,
    std::vector<invisible_places::io::Float3> normals = {}) {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions.resize(values.size());
    cloud.scalarFields.push_back(
        invisible_places::io::ScalarFieldStats{.name = "Field"});
    cloud.scalarFieldValues = std::move(values);
    cloud.normals = std::move(normals);
    cloud.hasNormals = cloud.normals.size() == cloud.positions.size();
    return cloud;
}

struct TemporaryHistogramDirectory {
    TemporaryHistogramDirectory() {
        const auto sequence =
            std::chrono::steady_clock::now().time_since_epoch().count();
        path =
            std::filesystem::temp_directory_path() /
            ("invisible_places_timing_colourise_histogram_" +
             std::to_string(sequence));
        std::filesystem::create_directories(path);
    }

    ~TemporaryHistogramDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }

    std::filesystem::path path;
};

using HistogramPlyPoint = std::array<float, 8>;

void WriteHistogramPly(
    const std::filesystem::path& path,
    const std::vector<HistogramPlyPoint>& points) {
    std::ofstream output{
        path,
        std::ios::binary | std::ios::trunc};
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << points.size() << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property float normal_x\n"
           << "property float normal_y\n"
           << "property float normal_z\n"
           << "property float scalar_Field\n"
           << "property float scalar_Ignored\n"
           << "end_header\n";
    for (const auto& point : points) {
        output.write(
            reinterpret_cast<const char*>(point.data()),
            static_cast<std::streamsize>(
                point.size() * sizeof(float)));
    }
}

TimingColouriseHistogramFingerprintInput FingerprintFixture() {
    return {
        .schemaVersion = invisible_places::timing::
            kTimingColouriseHistogramCacheSchemaVersion,
        .sceneGroupName = "Scene 01",
        .selector =
            TimingColouriseFieldSelector{
                .source = TimingColouriseFieldSource::Scalar,
                .scalarFieldName = "Roughness_Fine",
            },
        .sources =
            std::array<TimingColouriseHistogramSourceIdentity, 3>{
                TimingColouriseHistogramSourceIdentity{
                    .sourcePath = "Sand.ply",
                    .fileSize = 100U,
                    .modificationTimeNanoseconds = 10,
                },
                TimingColouriseHistogramSourceIdentity{
                    .sourcePath = "Rock.ply",
                    .fileSize = 200U,
                    .modificationTimeNanoseconds = 20,
                },
                TimingColouriseHistogramSourceIdentity{
                    .sourcePath = "Veg.ply",
                    .fileSize = 300U,
                    .modificationTimeNanoseconds = 30,
                },
            },
        .displaySpacingMicrometres = 5'000U,
    };
}

}  // namespace

TEST_CASE(
    "Timing Colourise field catalog intersects and groups authored fields",
    "[timing][colourise][histogram][fields]") {
    const std::vector<std::string> shared{
        "A_R_MeanCurvature_Fine",
        "A_R_MeanCurvature_Medium",
        "A_R_MeanCurvature_Broad",
        "A_R_MeanCurvature_Combined",
        "Downhill_X",
        "Downhill_Y",
        "Downhill_Z",
        "DownhillMagnitude",
        "RainExposure_Lower",
        "UnknownScalar",
        "water_effect_value",
        "ripple_phase",
    };
    std::array<TimingColouriseLayerFieldSet, 3> layers{
        TimingColouriseLayerFieldSet{
            .scalarFieldNames = shared,
            .hasNormals = true,
        },
        TimingColouriseLayerFieldSet{
            .scalarFieldNames = shared,
            .hasNormals = true,
        },
        TimingColouriseLayerFieldSet{
            .scalarFieldNames = shared,
            .hasNormals = true,
        },
    };
    layers[0].scalarFieldNames.push_back("SandOnly");
    layers[1].scalarFieldNames.push_back("RockOnly");
    layers[2].scalarFieldNames.push_back("VegOnly");

    const auto catalog =
        invisible_places::timing::BuildTimingColouriseFieldCatalog(
            layers);
    const auto* curvature = FindFamily(catalog, "A R MeanCurvature");
    REQUIRE(curvature != nullptr);
    REQUIRE(curvature->variants.size() == 4U);
    CHECK(curvature->variants[0].name == "Fine");
    CHECK(curvature->variants[1].name == "Medium");
    CHECK(curvature->variants[2].name == "Broad");
    CHECK(curvature->variants[3].name == "Combined");
    CHECK(
        curvature->variants[2].selector.scalarFieldName ==
        "A_R_MeanCurvature_Broad");

    const auto* downhill = FindFamily(catalog, "Downhill");
    REQUIRE(downhill != nullptr);
    REQUIRE(downhill->variants.size() == 4U);
    CHECK(downhill->variants[0].name == "X");
    CHECK(downhill->variants[1].name == "Y");
    CHECK(downhill->variants[2].name == "Z");
    CHECK(downhill->variants[3].name == "Magnitude");
    CHECK(
        downhill->variants[3].selector.scalarFieldName ==
        "DownhillMagnitude");

    const auto* unknown = FindFamily(catalog, "UnknownScalar");
    REQUIRE(unknown != nullptr);
    REQUIRE(unknown->variants.size() == 1U);
    CHECK(unknown->variants.front().name == "Value");

    const auto* normals = FindFamily(catalog, "Normal");
    REQUIRE(normals != nullptr);
    REQUIRE(normals->variants.size() == 3U);
    CHECK(
        normals->variants.front().selector.source ==
        TimingColouriseFieldSource::NormalX);

    CHECK(FindFamily(catalog, "SandOnly") == nullptr);
    CHECK(FindFamily(catalog, "water effect value") == nullptr);
    CHECK(FindFamily(catalog, "ripple phase") == nullptr);
    CHECK(
        FindFamily(catalog, "RainExposure Lower") !=
        nullptr);

    layers[2].hasNormals = false;
    const auto withoutCompleteNormals =
        invisible_places::timing::BuildTimingColouriseFieldCatalog(
            layers);
    CHECK(FindFamily(withoutCompleteNormals, "Normal") == nullptr);
}

TEST_CASE(
    "Timing Colourise generated implementation fields are excluded narrowly",
    "[timing][colourise][histogram][fields]") {
    using invisible_places::timing::
        IsGeneratedTimingColouriseScalarField;
    CHECK(IsGeneratedTimingColouriseScalarField("water_effect_value"));
    CHECK(IsGeneratedTimingColouriseScalarField("scalar_ripple_phase"));
    CHECK(IsGeneratedTimingColouriseScalarField("mesh-flow-contact"));
    CHECK_FALSE(IsGeneratedTimingColouriseScalarField("RainExposure_Lower"));
    CHECK_FALSE(IsGeneratedTimingColouriseScalarField("WaterIndex"));
}

TEST_CASE(
    "Timing Colourise histogram exactly aggregates resident authored layers",
    "[timing][colourise][histogram]") {
    auto sand = MakeCloud({
        -1.0F,
        0.0F,
        std::numeric_limits<float>::quiet_NaN(),
    });
    auto rock = MakeCloud({
        1.0F,
        2.0F,
        std::numeric_limits<float>::infinity(),
    });
    auto veg = MakeCloud({
        3.0F,
        -std::numeric_limits<float>::infinity(),
        1.0F,
    });
    const invisible_places::timing::TimingColouriseResidentCloudBundle
        clouds{&sand, &rock, &veg};
    const auto result =
        invisible_places::timing::ComputeTimingColouriseHistogram(
            clouds,
            TimingColouriseFieldSelector{
                .source = TimingColouriseFieldSource::Scalar,
                .scalarFieldName = "Field",
            });
    REQUIRE(result.success);
    CHECK_FALSE(result.cancelled);
    CHECK(result.histogram.minimum == Catch::Approx(-1.0F));
    CHECK(result.histogram.maximum == Catch::Approx(3.0F));
    CHECK(result.histogram.finiteValueCount == 6U);
    constexpr auto binCount = invisible_places::timing::
        kTimingColouriseHistogramBinCount;
    CHECK(result.histogram.bins[0U] == 1U);
    CHECK(result.histogram.bins[binCount / 4U] == 1U);
    CHECK(result.histogram.bins[binCount / 2U] == 2U);
    CHECK(result.histogram.bins[binCount * 3U / 4U] == 1U);
    CHECK(result.histogram.bins[binCount - 1U] == 1U);

    std::uint64_t total = 0U;
    for (const auto count : result.histogram.bins) {
        total += count;
    }
    CHECK(total == result.histogram.finiteValueCount);
}

TEST_CASE(
    "Timing Colourise high-resolution bins stay off the worker stack",
    "[timing][colourise][histogram][storage]") {
    using invisible_places::timing::TimingColouriseHistogram;
    using invisible_places::timing::kTimingColouriseHistogramBinCount;

    // Keep the owning object small enough to copy through the background job
    // without reserving the 16,384-bin payload in each stack-frame temporary.
    CHECK(sizeof(TimingColouriseHistogram) < 1'024U);
    const TimingColouriseHistogram histogram;
    CHECK(histogram.bins.size() == kTimingColouriseHistogramBinCount);
}

TEST_CASE(
    "Timing Colourise histogram resolves narrow concentrations inside outliers",
    "[timing][colourise][histogram][resolution]") {
    std::vector<float> concentratedValues{-1.0F, 1.0F};
    for (std::size_t index = 0U; index <= 32U; ++index) {
        concentratedValues.push_back(
            std::lerp(
                -0.001F,
                0.001F,
                static_cast<float>(index) / 32.0F));
    }
    auto sand = MakeCloud(std::move(concentratedValues));
    auto rock = MakeCloud({0.0F});
    auto veg = MakeCloud({0.0F});
    const invisible_places::timing::TimingColouriseResidentCloudBundle
        clouds{&sand, &rock, &veg};
    const auto result =
        invisible_places::timing::ComputeTimingColouriseHistogram(
            clouds,
            TimingColouriseFieldSelector{
                .source = TimingColouriseFieldSource::Scalar,
                .scalarFieldName = "Field",
            });

    REQUIRE(result.success);
    CHECK(result.histogram.minimum == Catch::Approx(-1.0F));
    CHECK(result.histogram.maximum == Catch::Approx(1.0F));
    const auto occupiedBinCount = static_cast<std::size_t>(
        std::count_if(
            result.histogram.bins.begin(),
            result.histogram.bins.end(),
            [](std::uint64_t count) {
                return count > 0U;
            }));
    // The two outlier bins plus at least sixteen distinct bins across the
    // narrow central concentration. The former 256-bin cache produced only
    // one central bucket for this same distribution.
    CHECK(occupiedBinCount >= 18U);
}

TEST_CASE(
    "Scalar field statistics exclude non-finite samples",
    "[io][point-cloud][scalar-fields][timing-colourise]") {
    invisible_places::io::ScalarFieldStats stats;
    stats.Include(std::numeric_limits<float>::quiet_NaN());
    stats.Include(std::numeric_limits<float>::infinity());
    stats.Include(-std::numeric_limits<float>::infinity());
    CHECK_FALSE(stats.valid);
    CHECK(stats.count == 0U);

    stats.Include(-2.0F);
    stats.Include(std::numeric_limits<float>::quiet_NaN());
    stats.Include(3.0F);
    REQUIRE(stats.valid);
    CHECK(stats.minimum == Catch::Approx(-2.0F));
    CHECK(stats.maximum == Catch::Approx(3.0F));
    CHECK(stats.count == 2U);
}

TEST_CASE(
    "Timing Colourise histogram display stretches its occupied bucket range",
    "[timing][colourise][histogram][display]") {
    using invisible_places::timing::
        TimingColouriseHistogramDisplayHeight;

    CHECK(
        TimingColouriseHistogramDisplayHeight(10U, 10U, 100U) ==
        Catch::Approx(0.0F));
    CHECK(
        TimingColouriseHistogramDisplayHeight(100U, 10U, 100U) ==
        Catch::Approx(1.0F));
    CHECK(
        TimingColouriseHistogramDisplayHeight(40U, 10U, 100U) ==
        Catch::Approx(1.0F / 3.0F));

    // Empty bins participate in the range. Uniform non-empty histograms
    // remain visible because no relative distribution can be inferred.
    CHECK(
        TimingColouriseHistogramDisplayHeight(0U, 0U, 100U) ==
        Catch::Approx(0.0F));
    CHECK(
        TimingColouriseHistogramDisplayHeight(7U, 7U, 7U) ==
        Catch::Approx(1.0F));
    CHECK(
        TimingColouriseHistogramDisplayHeight(0U, 0U, 0U) ==
        Catch::Approx(0.0F));
}

TEST_CASE(
    "Timing Colourise display density preserves a broad peak around endpoint spikes",
    "[timing][colourise][histogram][display]") {
    constexpr auto binCount = invisible_places::timing::
        kTimingColouriseHistogramBinCount;
    constexpr auto middle = binCount / 2U;
    std::array<std::uint64_t, binCount> bins{};
    bins.fill(100U);
    bins.front() = 53'400U;
    bins.back() = 45'744U;
    for (std::size_t index = middle - 64U;
         index <= middle + 64U;
         ++index) {
        const auto distance =
            index > middle ? index - middle : middle - index;
        bins[index] = 2'400U - distance * 10U;
    }
    const auto histogram = MakeHistogram(-1.0F, 1.0F, bins);
    const auto displayBins = invisible_places::timing::
        BuildTimingColouriseHistogramDisplayBins(histogram);

    REQUIRE(displayBins.size() == bins.size());
    std::uint64_t expectedFirstWindow = 0U;
    constexpr auto windowSize =
        binCount /
        invisible_places::timing::
            kTimingColouriseHistogramReferenceDisplayBinCount;
    for (std::size_t index = 0U; index < windowSize; ++index) {
        expectedFirstWindow += bins[index];
    }
    CHECK(displayBins.front() == expectedFirstWindow);
    CHECK(displayBins[middle] > displayBins.front());
    CHECK(displayBins[middle] > displayBins.back());
    const auto maximum = std::max_element(
        displayBins.begin(),
        displayBins.end());
    REQUIRE(maximum != displayBins.end());
    const auto maximumIndex = static_cast<std::size_t>(
        std::distance(displayBins.begin(), maximum));
    CHECK(maximumIndex >= middle - 64U);
    CHECK(maximumIndex <= middle + 64U);
}

TEST_CASE(
    "Timing Colourise distribution axis is monotone invertible and zero anchored",
    "[timing][colourise][histogram][axis]") {
    using invisible_places::timing::
        BuildTimingColouriseHistogramAxis;
    using invisible_places::timing::
        TimingColouriseHistogramAxisMode;
    using invisible_places::timing::
        TimingColouriseHistogramAxisShape;

    std::array<
        std::uint64_t,
        invisible_places::timing::
            kTimingColouriseHistogramBinCount>
        bins{};
    bins.fill(1U);
    constexpr auto middleBin = invisible_places::timing::
        kTimingColouriseHistogramBinCount / 2U;
    bins[middleBin - 1U] = 20'000U;
    bins[middleBin] = 20'000U;
    const auto histogram =
        MakeHistogram(-1.0F, 1.0F, bins);
    const auto axis =
        BuildTimingColouriseHistogramAxis(
            histogram,
            TimingColouriseHistogramAxisMode::
                DistributionSpread);

    REQUIRE(axis.validRange);
    REQUIRE(axis.UsesDistributionSpread());
    CHECK(
        axis.shape ==
        TimingColouriseHistogramAxisShape::Centred);
    CHECK(axis.zeroUnit == Catch::Approx(0.5F));
    CHECK(axis.RawToUnit(0.0F) == Catch::Approx(0.5F));
    CHECK(axis.UnitToRaw(0.5F) == Catch::Approx(0.0F));
    CHECK(
        axis.RawToUnit(
            std::numeric_limits<float>::quiet_NaN()) ==
        Catch::Approx(0.0F));
    CHECK(
        axis.RawToUnit(
            -std::numeric_limits<float>::infinity()) ==
        Catch::Approx(0.0F));
    CHECK(
        axis.RawToUnit(
            std::numeric_limits<float>::infinity()) ==
        Catch::Approx(1.0F));
    CHECK(
        axis.UnitToRaw(
            std::numeric_limits<float>::quiet_NaN()) ==
        Catch::Approx(-1.0F));
    CHECK(
        axis.UnitToRaw(
            -std::numeric_limits<float>::infinity()) ==
        Catch::Approx(-1.0F));
    CHECK(
        axis.UnitToRaw(
            std::numeric_limits<float>::infinity()) ==
        Catch::Approx(1.0F));

    REQUIRE(axis.knotCount > 2U);
    for (std::size_t index = 1U;
         index < axis.knotCount;
         ++index) {
        CHECK(
            axis.rawKnots[index] >
            axis.rawKnots[index - 1U]);
        CHECK(
            axis.unitKnots[index] >
            axis.unitKnots[index - 1U]);
    }
    constexpr std::array rawSamples{
        -1.0F,
        -0.73F,
        -0.1F,
        -0.001F,
        0.0F,
        0.001F,
        0.1F,
        0.73F,
        1.0F,
    };
    float previousUnit = -1.0F;
    for (const float rawValue : rawSamples) {
        const float unit = axis.RawToUnit(rawValue);
        CHECK(unit >= previousUnit);
        CHECK(
            axis.UnitToRaw(unit) ==
            Catch::Approx(rawValue).margin(2.0e-5F));
        previousUnit = unit;
    }
}

TEST_CASE(
    "Timing Colourise distribution axis expands concentrated bins",
    "[timing][colourise][histogram][axis]") {
    using invisible_places::timing::
        BuildTimingColouriseHistogramAxis;
    using invisible_places::timing::
        TimingColouriseHistogramAxisMode;

    std::array<
        std::uint64_t,
        invisible_places::timing::
            kTimingColouriseHistogramBinCount>
        bins{};
    bins.fill(1U);
    constexpr auto middleBin = invisible_places::timing::
        kTimingColouriseHistogramBinCount / 2U;
    bins[middleBin - 1U] = 50'000U;
    bins[middleBin] = 50'000U;
    const auto histogram =
        MakeHistogram(-1.0F, 1.0F, bins);
    const auto rawAxis =
        BuildTimingColouriseHistogramAxis(
            histogram,
            TimingColouriseHistogramAxisMode::Raw);
    const auto spreadAxis =
        BuildTimingColouriseHistogramAxis(
            histogram,
            TimingColouriseHistogramAxisMode::
                DistributionSpread);
    const auto rawBinEdge =
        [](std::size_t index) {
            return std::lerp(
                -1.0F,
                1.0F,
                static_cast<float>(index) /
                    static_cast<float>(
                        invisible_places::timing::
                            kTimingColouriseHistogramBinCount));
        };
    const float lower = rawBinEdge(middleBin - 1U);
    const float upper = rawBinEdge(middleBin);
    const float rawWidth =
        rawAxis.RawToUnit(upper) -
        rawAxis.RawToUnit(lower);
    const float spreadWidth =
        spreadAxis.RawToUnit(upper) -
        spreadAxis.RawToUnit(lower);
    CHECK(spreadWidth > rawWidth * 20.0F);
}

TEST_CASE(
    "Timing Colourise roughness-like histograms use a reversible one-sided axis",
    "[timing][colourise][histogram][axis]") {
    using invisible_places::timing::
        BuildTimingColouriseHistogramAxis;
    using invisible_places::timing::
        TimingColouriseHistogramAxisMode;
    using invisible_places::timing::
        TimingColouriseHistogramAxisShape;

    std::array<
        std::uint64_t,
        invisible_places::timing::
            kTimingColouriseHistogramBinCount>
        bins{};
    // Roughness-like range: most samples occupy the shallow negative lobe,
    // while the positive lobe has a much larger robust extent. The extent
    // imbalance should present as one 0..1 distribution rather than assigning
    // half the mouse width to each side of raw zero.
    constexpr auto binCount = invisible_places::timing::
        kTimingColouriseHistogramBinCount;
    constexpr auto denseEnd = binCount * 12U / 256U;
    constexpr auto shoulderEnd = binCount / 2U;
    for (std::size_t index = 0U; index < denseEnd; ++index) {
        bins[index] = 600U;
    }
    for (std::size_t index = denseEnd;
         index < shoulderEnd;
         ++index) {
        bins[index] = 24U;
    }
    bins.back() = 16U;
    const auto histogram =
        MakeHistogram(-0.05F, 1.0F, bins);
    const auto axis =
        BuildTimingColouriseHistogramAxis(
            histogram,
            TimingColouriseHistogramAxisMode::
                DistributionSpread);

    REQUIRE(axis.UsesDistributionSpread());
    CHECK(
        axis.shape ==
        TimingColouriseHistogramAxisShape::
            PositiveOneSided);
    CHECK(axis.RawToUnit(-0.05F) == Catch::Approx(0.0F));
    CHECK(axis.RawToUnit(1.0F) == Catch::Approx(1.0F));
    // The full CDF is retained: raw zero is not forced to the middle or to
    // the left edge, and its dense negative lobe remains easy to edit.
    CHECK(axis.zeroUnit > 0.5F);
    CHECK(
        axis.UnitToRaw(axis.zeroUnit) ==
        Catch::Approx(0.0F).margin(2.0e-5F));
    for (const float rawValue :
         {-0.04F, -0.01F, 0.0F, 0.1F, 0.45F, 0.9F}) {
        CHECK(
            axis.UnitToRaw(
                axis.RawToUnit(rawValue)) ==
            Catch::Approx(rawValue).margin(2.0e-5F));
    }
}

TEST_CASE(
    "Timing Colourise invalid and constant histograms fall back to Raw",
    "[timing][colourise][histogram][axis]") {
    using invisible_places::timing::
        BuildTimingColouriseHistogramAxis;
    using invisible_places::timing::
        TimingColouriseHistogramAxisMode;
    using invisible_places::timing::
        TimingColouriseHistogramAxisShape;

    const invisible_places::timing::
        TimingColouriseHistogram invalid;
    const auto invalidAxis =
        BuildTimingColouriseHistogramAxis(
            invalid,
            TimingColouriseHistogramAxisMode::
                DistributionSpread);
    CHECK_FALSE(invalidAxis.validRange);
    CHECK_FALSE(invalidAxis.UsesDistributionSpread());
    CHECK(
        invalidAxis.shape ==
        TimingColouriseHistogramAxisShape::Raw);

    std::array<
        std::uint64_t,
        invisible_places::timing::
            kTimingColouriseHistogramBinCount>
        constantBins{};
    constantBins.front() = 10U;
    const auto constant =
        MakeHistogram(2.0F, 2.0F, constantBins);
    const auto constantAxis =
        BuildTimingColouriseHistogramAxis(
            constant,
            TimingColouriseHistogramAxisMode::
                DistributionSpread);
    CHECK_FALSE(constantAxis.validRange);
    CHECK_FALSE(constantAxis.UsesDistributionSpread());
    CHECK(constantAxis.RawToUnit(2.0F) == Catch::Approx(0.5F));
    CHECK(constantAxis.UnitToRaw(0.9F) == Catch::Approx(2.0F));
}

TEST_CASE(
    "Timing Colourise histogram supports normals, zero spans, and cancellation",
    "[timing][colourise][histogram]") {
    const std::vector<invisible_places::io::Float3> normals{
        {.x = -1.0F, .y = 0.0F, .z = 1.0F},
        {.x = 0.0F, .y = 0.5F, .z = 1.0F},
    };
    auto sand = MakeCloud({2.0F, 2.0F}, normals);
    auto rock = MakeCloud({2.0F, 2.0F}, normals);
    auto veg = MakeCloud({2.0F, 2.0F}, normals);
    const invisible_places::timing::TimingColouriseResidentCloudBundle
        clouds{&sand, &rock, &veg};

    const auto constant =
        invisible_places::timing::ComputeTimingColouriseHistogram(
            clouds,
            TimingColouriseFieldSelector{
                .source = TimingColouriseFieldSource::Scalar,
                .scalarFieldName = "Field",
            });
    REQUIRE(constant.success);
    CHECK(constant.histogram.minimum == Catch::Approx(2.0F));
    CHECK(constant.histogram.maximum == Catch::Approx(2.0F));
    CHECK(constant.histogram.bins.front() == 6U);

    const auto normal =
        invisible_places::timing::ComputeTimingColouriseHistogram(
            clouds,
            TimingColouriseFieldSelector{
                .source = TimingColouriseFieldSource::NormalX,
            });
    REQUIRE(normal.success);
    CHECK(normal.histogram.minimum == Catch::Approx(-1.0F));
    CHECK(normal.histogram.maximum == Catch::Approx(0.0F));
    CHECK(normal.histogram.bins.front() == 3U);
    CHECK(normal.histogram.bins.back() == 3U);

    std::stop_source cancellation;
    cancellation.request_stop();
    const auto cancelled =
        invisible_places::timing::ComputeTimingColouriseHistogram(
            clouds,
            TimingColouriseFieldSelector{
                .source = TimingColouriseFieldSource::NormalX,
            },
            cancellation.get_token());
    CHECK_FALSE(cancelled.success);
    CHECK(cancelled.cancelled);
}

TEST_CASE(
    "Selected point-cloud values stream arbitrary scalars and normalized normals",
    "[io][pointcloud][stream][timing-colourise]") {
    TemporaryHistogramDirectory temporary;
    const auto path = temporary.path / "selected-values.ply";
    WriteHistogramPly(
        path,
        {
            HistogramPlyPoint{
                0.0F,
                0.0F,
                0.0F,
                0.0F,
                0.0F,
                2.0F,
                -1.0F,
                100.0F},
            HistogramPlyPoint{
                1.0F,
                0.0F,
                0.0F,
                0.0F,
                3.0F,
                0.0F,
                std::numeric_limits<float>::quiet_NaN(),
                200.0F},
            HistogramPlyPoint{
                2.0F,
                0.0F,
                0.0F,
                4.0F,
                0.0F,
                0.0F,
                3.0F,
                300.0F},
        });

    std::vector<float> values;
    const auto scalar =
        invisible_places::io::StreamPointCloudSelectedValues(
            path,
            invisible_places::io::
                PointCloudSelectedValueSelector{
                    .source =
                        invisible_places::io::
                            PointCloudSelectedValueSource::
                                ScalarField,
                    .scalarFieldName = "Field",
                },
            [&](float value, std::uint64_t) {
                values.push_back(value);
                return true;
            });
    REQUIRE(scalar.success);
    CHECK_FALSE(scalar.cancelled);
    CHECK(scalar.pointCount == 3U);
    REQUIRE(values.size() == 3U);
    CHECK(values[0U] == Catch::Approx(-1.0F));
    CHECK(std::isnan(values[1U]));
    CHECK(values[2U] == Catch::Approx(3.0F));

    std::vector<float> normalY;
    const auto normal =
        invisible_places::io::StreamPointCloudSelectedValues(
            path,
            invisible_places::io::
                PointCloudSelectedValueSelector{
                    .source =
                        invisible_places::io::
                            PointCloudSelectedValueSource::NormalY,
                },
            [&](float value, std::uint64_t) {
                normalY.push_back(value);
                return true;
            });
    REQUIRE(normal.success);
    REQUIRE(normalY.size() == 3U);
    CHECK(normalY[0U] == Catch::Approx(0.0F));
    CHECK(normalY[1U] == Catch::Approx(1.0F));
    CHECK(normalY[2U] == Catch::Approx(0.0F));

    std::uint64_t visited = 0U;
    const auto cancelled =
        invisible_places::io::StreamPointCloudSelectedValues(
            path,
            invisible_places::io::
                PointCloudSelectedValueSelector{
                    .source =
                        invisible_places::io::
                            PointCloudSelectedValueSource::
                                ScalarField,
                    .scalarFieldName = "Ignored",
                },
            [&](float, std::uint64_t) {
                ++visited;
                return visited < 2U;
            });
    CHECK_FALSE(cancelled.success);
    CHECK(cancelled.cancelled);
    CHECK(cancelled.pointCount == 2U);

    const auto missing =
        invisible_places::io::StreamPointCloudSelectedValues(
            path,
            invisible_places::io::
                PointCloudSelectedValueSelector{
                    .source =
                        invisible_places::io::
                            PointCloudSelectedValueSource::
                                ScalarField,
                    .scalarFieldName = "Missing",
                },
            {});
    CHECK_FALSE(missing.success);
    CHECK_FALSE(missing.cancelled);
    CHECK(
        missing.errorMessage.find("Missing") !=
        std::string::npos);
}

TEST_CASE(
    "Timing Colourise streaming histogram matches resident and mixes released layers",
    "[timing][colourise][histogram][stream]") {
    TemporaryHistogramDirectory temporary;
    const auto sandPath = temporary.path / "sand.ply";
    const auto rockPath = temporary.path / "rock.ply";
    const auto vegPath = temporary.path / "veg.ply";
    WriteHistogramPly(
        sandPath,
        {
            HistogramPlyPoint{
                0.0F, 0.0F, 0.0F, -2.0F, 0.0F, 0.0F,
                -1.0F, 10.0F},
            HistogramPlyPoint{
                1.0F, 0.0F, 0.0F, -2.0F, 0.0F, 0.0F,
                0.0F, 20.0F},
            HistogramPlyPoint{
                2.0F, 0.0F, 0.0F, -2.0F, 0.0F, 0.0F,
                std::numeric_limits<float>::quiet_NaN(),
                30.0F},
        });
    WriteHistogramPly(
        rockPath,
        {
            HistogramPlyPoint{
                0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 2.0F,
                1.0F, 40.0F},
            HistogramPlyPoint{
                1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 2.0F,
                2.0F, 50.0F},
            HistogramPlyPoint{
                2.0F, 1.0F, 0.0F, 0.0F, 0.0F, 2.0F,
                std::numeric_limits<float>::infinity(),
                60.0F},
        });
    WriteHistogramPly(
        vegPath,
        {
            HistogramPlyPoint{
                0.0F, 2.0F, 0.0F, 3.0F, 0.0F, 0.0F,
                3.0F, 70.0F},
            HistogramPlyPoint{
                1.0F, 2.0F, 0.0F, 3.0F, 0.0F, 0.0F,
                -std::numeric_limits<float>::infinity(),
                80.0F},
            HistogramPlyPoint{
                2.0F, 2.0F, 0.0F, 3.0F, 0.0F, 0.0F,
                1.0F, 90.0F},
        });

    auto sand = invisible_places::io::LoadPointCloud(sandPath);
    auto rock = invisible_places::io::LoadPointCloud(rockPath);
    auto veg = invisible_places::io::LoadPointCloud(vegPath);
    REQUIRE(sand.success);
    REQUIRE(rock.success);
    REQUIRE(veg.success);
    const invisible_places::timing::
        TimingColouriseResidentCloudBundle resident{
            &sand.cloud,
            &rock.cloud,
            &veg.cloud,
        };
    const invisible_places::timing::
        TimingColouriseHistogramSourceBundle streamed{
            invisible_places::timing::
                TimingColouriseHistogramLayerSource{
                    .sourcePath = sandPath},
            invisible_places::timing::
                TimingColouriseHistogramLayerSource{
                    .sourcePath = rockPath},
            invisible_places::timing::
                TimingColouriseHistogramLayerSource{
                    .sourcePath = vegPath},
        };
    const TimingColouriseFieldSelector scalarSelector{
        .source = TimingColouriseFieldSource::Scalar,
        .scalarFieldName = "Field",
    };
    const auto residentScalar =
        invisible_places::timing::
            ComputeTimingColouriseHistogram(
                resident,
                scalarSelector);
    const auto streamedScalar =
        invisible_places::timing::
            ComputeTimingColouriseHistogramFromSources(
                streamed,
                scalarSelector);
    REQUIRE(residentScalar.success);
    REQUIRE(streamedScalar.success);
    CHECK(
        streamedScalar.histogram.minimum ==
        Catch::Approx(residentScalar.histogram.minimum));
    CHECK(
        streamedScalar.histogram.maximum ==
        Catch::Approx(residentScalar.histogram.maximum));
    CHECK(
        streamedScalar.histogram.finiteValueCount ==
        residentScalar.histogram.finiteValueCount);
    CHECK(
        streamedScalar.histogram.bins ==
        residentScalar.histogram.bins);

    const TimingColouriseFieldSelector normalSelector{
        .source = TimingColouriseFieldSource::NormalX,
    };
    const auto residentNormal =
        invisible_places::timing::
            ComputeTimingColouriseHistogram(
                resident,
                normalSelector);
    const auto streamedNormal =
        invisible_places::timing::
            ComputeTimingColouriseHistogramFromSources(
                streamed,
                normalSelector);
    REQUIRE(residentNormal.success);
    REQUIRE(streamedNormal.success);
    CHECK(
        streamedNormal.histogram.minimum ==
        Catch::Approx(residentNormal.histogram.minimum));
    CHECK(
        streamedNormal.histogram.maximum ==
        Catch::Approx(residentNormal.histogram.maximum));
    CHECK(
        streamedNormal.histogram.bins ==
        residentNormal.histogram.bins);

    auto releasedRock = rock.cloud;
    releasedRock.positions.clear();
    releasedRock.normals.clear();
    releasedRock.packedColors.clear();
    releasedRock.scalarFieldValues.clear();
    const invisible_places::timing::
        TimingColouriseHistogramSourceBundle mixed{
            invisible_places::timing::
                TimingColouriseHistogramLayerSource{
                    .residentCloud = &sand.cloud,
                    .sourcePath =
                        temporary.path / "unused-sand.ply"},
            invisible_places::timing::
                TimingColouriseHistogramLayerSource{
                    .residentCloud = &releasedRock,
                    .sourcePath = rockPath},
            invisible_places::timing::
                TimingColouriseHistogramLayerSource{
                    .sourcePath = vegPath},
        };
    const auto mixedScalar =
        invisible_places::timing::
            ComputeTimingColouriseHistogramFromSources(
                mixed,
                scalarSelector);
    REQUIRE(mixedScalar.success);
    CHECK(
        mixedScalar.histogram.bins ==
        residentScalar.histogram.bins);

    std::stop_source cancellation;
    cancellation.request_stop();
    const auto cancelled =
        invisible_places::timing::
            ComputeTimingColouriseHistogramFromSources(
                streamed,
                scalarSelector,
                cancellation.get_token());
    CHECK_FALSE(cancelled.success);
    CHECK(cancelled.cancelled);
}

TEST_CASE(
    "Timing Colourise histogram fingerprint covers selector and source state",
    "[timing][colourise][histogram][cache]") {
    const auto fixture = FingerprintFixture();
    const auto baseline =
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(fixture);
    CHECK(baseline.size() == 16U);
    CHECK(
        baseline ==
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(fixture));

    auto changed = fixture;
    ++changed.schemaVersion;
    CHECK(
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(changed) !=
        baseline);
    changed = fixture;
    changed.sceneGroupName = "Scene 02";
    CHECK(
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(changed) !=
        baseline);
    changed = fixture;
    changed.selector.scalarFieldName = "Roughness_Broad";
    CHECK(
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(changed) !=
        baseline);
    changed = fixture;
    changed.selector.source = TimingColouriseFieldSource::NormalX;
    CHECK(
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(changed) !=
        baseline);
    changed = fixture;
    ++changed.displaySpacingMicrometres;
    CHECK(
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(changed) !=
        baseline);
    changed = fixture;
    ++changed.sources[1].fileSize;
    CHECK(
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(changed) !=
        baseline);
    changed = fixture;
    ++changed.sources[2].modificationTimeNanoseconds;
    CHECK(
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(changed) !=
        baseline);
}

TEST_CASE(
    "Timing Colourise binary histogram cache round trips and rejects stale data",
    "[timing][colourise][histogram][cache]") {
    TemporaryHistogramDirectory temporary;
    const auto fingerprint =
        invisible_places::timing::
            BuildTimingColouriseHistogramFingerprint(
                FingerprintFixture());
    const auto cachePath =
        invisible_places::timing::TimingColouriseHistogramCachePath(
            temporary.path,
            fingerprint);
    CHECK(
        cachePath.parent_path() ==
        temporary.path / ".invisible_places" / "cache" /
            "colourise_histograms");

    invisible_places::timing::TimingColouriseHistogram histogram{
        .minimum = -2.0F,
        .maximum = 4.0F,
        .finiteValueCount = 5U,
    };
    histogram.bins[0U] = 2U;
    histogram.bins[histogram.bins.size() / 2U] = 1U;
    histogram.bins.back() = 2U;
    std::string error;
    REQUIRE(
        invisible_places::timing::
            SaveTimingColouriseHistogramCache(
                cachePath,
                fingerprint,
                histogram,
                &error));
    CHECK(error.empty());
    const auto loaded =
        invisible_places::timing::
            LoadTimingColouriseHistogramCache(
                cachePath,
                fingerprint,
                &error);
    REQUIRE(loaded.has_value());
    CHECK(loaded->minimum == Catch::Approx(histogram.minimum));
    CHECK(loaded->maximum == Catch::Approx(histogram.maximum));
    CHECK(loaded->finiteValueCount == 5U);
    CHECK(loaded->bins == histogram.bins);

    CHECK_FALSE(
        invisible_places::timing::
            LoadTimingColouriseHistogramCache(
                cachePath,
                "different-fingerprint",
                &error)
            .has_value());
    CHECK_FALSE(error.empty());

    {
        std::ofstream corrupt{
            cachePath,
            std::ios::binary | std::ios::app};
        corrupt.put('x');
    }
    CHECK_FALSE(
        invisible_places::timing::
            LoadTimingColouriseHistogramCache(
                cachePath,
                fingerprint,
                &error)
            .has_value());
    CHECK(error.find("trailing") != std::string::npos);

    const auto confined =
        invisible_places::timing::TimingColouriseHistogramCachePath(
            temporary.path,
            "../outside");
    CHECK(
        confined.parent_path() ==
        invisible_places::timing::
            TimingColouriseHistogramCacheDirectory(temporary.path));
}
