#include "water/WaterSeepagePulseField.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

namespace {

using Catch::Approx;
using invisible_places::water::BuildWaterSeepagePulseField;
using invisible_places::water::WaterSeepagePulseFieldQuality;
using invisible_places::water::WaterSeepagePulseFieldSettings;
using invisible_places::water::kWaterSeepagePulseFieldCapacity;

WaterSeepagePulseFieldSettings PulseSettings() {
    WaterSeepagePulseFieldSettings settings;
    settings.seed = 2027808452U;
    settings.nodeId = 4U;
    settings.stableSpanMeters = 3.125F;
    settings.timeSeconds = 17.25F;
    return settings;
}

float SampleSum(const std::array<float, kWaterSeepagePulseFieldCapacity>& samples) {
    return std::accumulate(samples.begin(), samples.end(), 0.0F);
}

}  // namespace

TEST_CASE(
    "Seepage pulse fields use fixed quality capacities and clear unused storage",
    "[water][seepage][pulse-field]") {
    auto settings = PulseSettings();
    const std::array qualities{
        WaterSeepagePulseFieldQuality::Low,
        WaterSeepagePulseFieldQuality::Balanced,
        WaterSeepagePulseFieldQuality::High,
    };
    const std::array<std::uint32_t, 3U> expectedCounts{32U, 64U, 128U};

    for (std::size_t index = 0U; index < qualities.size(); ++index) {
        settings.quality = qualities[index];
        const auto field = BuildWaterSeepagePulseField(settings);
        CHECK(field.sampleCount == expectedCounts[index]);
        CHECK(field.stableSpanMeters == Approx(settings.stableSpanMeters));
        CHECK(std::all_of(
            field.samples.begin() + field.sampleCount,
            field.samples.end(),
            [](float sample) { return sample == 0.0F; }));
    }
}

TEST_CASE(
    "Seepage pulse fields are deterministic but evolve with time and identity",
    "[water][seepage][pulse-field]") {
    const auto settings = PulseSettings();
    const auto first = BuildWaterSeepagePulseField(settings);
    const auto repeated = BuildWaterSeepagePulseField(settings);
    CHECK(first.samples == repeated.samples);

    auto laterSettings = settings;
    laterSettings.timeSeconds += 0.83F;
    const auto later = BuildWaterSeepagePulseField(laterSettings);
    CHECK(first.samples != later.samples);

    auto otherNodeSettings = settings;
    ++otherNodeSettings.nodeId;
    const auto otherNode = BuildWaterSeepagePulseField(otherNodeSettings);
    CHECK(first.samples != otherNode.samples);
}

TEST_CASE(
    "Contour Pulse reference launch starts visibly at the source",
    "[water][seepage][pulse-field][source-launch]") {
    auto settings = PulseSettings();
    settings.timeSeconds = 0.0F;
    settings.waveCount = 1.0F;
    settings.quality = WaterSeepagePulseFieldQuality::Low;

    const auto field = BuildWaterSeepagePulseField(settings);
    REQUIRE(field.sampleCount == 32U);
    CHECK(field.Sample(0.0F) > 0.10F);
    CHECK(field.Sample(0.0F) > field.Sample(settings.widthMeters * 3.0F));
}

TEST_CASE(
    "Fractional Seepage pulse counts fade the final wave instead of popping",
    "[water][seepage][pulse-field]") {
    auto settings = PulseSettings();
    settings.waveCount = 7.0F;
    const auto seven = BuildWaterSeepagePulseField(settings);
    settings.waveCount = 7.5F;
    const auto halfway = BuildWaterSeepagePulseField(settings);
    settings.waveCount = 8.0F;
    const auto eight = BuildWaterSeepagePulseField(settings);

    CHECK(SampleSum(halfway.samples) >= SampleSum(seven.samples));
    CHECK(SampleSum(halfway.samples) <= SampleSum(eight.samples));
    CHECK(halfway.samples != seven.samples);
    CHECK(halfway.samples != eight.samples);
}

TEST_CASE(
    "Seepage pulse lookup clamps its immutable longitudinal domain safely",
    "[water][seepage][pulse-field]") {
    const auto field = BuildWaterSeepagePulseField(PulseSettings());
    CHECK(field.Sample(-10.0F) == Approx(field.samples.front()));
    CHECK(field.Sample(field.stableSpanMeters + 10.0F) ==
          Approx(field.samples[field.sampleCount - 1U]));
    CHECK(field.Sample(std::numeric_limits<float>::quiet_NaN()) == 0.0F);
    CHECK(field.Sample(field.stableSpanMeters * 0.5F) >= 0.0F);
    CHECK(field.Sample(field.stableSpanMeters * 0.5F) <= 1.0F);
}
