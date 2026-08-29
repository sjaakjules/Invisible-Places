#include "timing/TimingColourisePresets.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <set>
#include <string_view>

namespace {

using Catch::Approx;
using invisible_places::timing::BuiltInTimingColourisePalettePresets;
using invisible_places::timing::CopyBuiltInTimingColourisePalettePreset;
using invisible_places::timing::FindBuiltInTimingColourisePalettePreset;
using invisible_places::timing::TimingColourisePaletteStop;
using invisible_places::timing::kBuiltInTimingColourisePalettePresetCount;

constexpr std::array<std::string_view, 37> kExpectedPresetIds{
    "seaborn-mako",
    "seaborn-rocket",
    "seaborn-crest",
    "seaborn-viridis",
    "seaborn-icefire",
    "seaborn-vlag",
    "seaborn-cubehelix",
    "matplotlib-inferno",
    "matplotlib-twilight",
    "matplotlib-twilight-blue",
    "matplotlib-twilight-red",
    "matplotlib-twilight-shifted",
    "matplotlib-terrain",
    "matplotlib-ocean",
    "matplotlib-gist-earth",
    "colorbrewer-ylgnbu",
    "colorbrewer-purd",
    "colorbrewer-blues",
    "crameri-batlow",
    "crameri-lipari",
    "crameri-romao",
    "crameri-corko",
    "cmocean-haline",
    "cmocean-solar",
    "cmocean-ice",
    "cmocean-deep",
    "cmocean-dense",
    "cmocean-algae",
    "cmocean-turbid",
    "cmocean-speed",
    "cmocean-rain",
    "cmocean-phase",
    "cmocean-topo",
    "cmocean-delta",
    "cmocean-curl",
    "cmocean-diff",
    "cmocean-tarn",
};

}  // namespace

TEST_CASE(
    "Built-in Timing Colourise palette catalog is complete and unique",
    "[timing][colourise][palette][preset]") {
    const auto presets = BuiltInTimingColourisePalettePresets();
    REQUIRE(presets.size() == kBuiltInTimingColourisePalettePresetCount);
    REQUIRE(presets.size() == kExpectedPresetIds.size());

    std::set<std::string_view> ids;
    std::set<std::string_view> names;
    for (const auto& preset : presets) {
        CHECK_FALSE(preset.id.empty());
        CHECK_FALSE(preset.name.empty());
        CHECK(ids.insert(preset.id).second);
        CHECK(names.insert(preset.name).second);
    }

    for (const auto expectedId : kExpectedPresetIds) {
        INFO("Missing preset " << expectedId);
        CHECK(FindBuiltInTimingColourisePalettePreset(expectedId) != nullptr);
    }
    CHECK(FindBuiltInTimingColourisePalettePreset("not-a-preset") == nullptr);
}

TEST_CASE(
    "Built-in Timing Colourise palettes have compact valid stops",
    "[timing][colourise][palette][preset]") {
    for (const auto& preset : BuiltInTimingColourisePalettePresets()) {
        INFO("Invalid preset " << preset.id);
        REQUIRE_FALSE(preset.palette.stops.empty());
        REQUIRE(preset.palette.stops.size() <= 5U);
        CHECK(preset.palette.stops.front().position == Approx(0.0F));
        CHECK(preset.palette.stops.back().position == Approx(1.0F));

        float previousPosition = -1.0F;
        for (const auto& stop : preset.palette.stops) {
            CHECK(std::isfinite(stop.position));
            CHECK(stop.position >= 0.0F);
            CHECK(stop.position <= 1.0F);
            CHECK(stop.position > previousPosition);
            CHECK(stop.colouriseAmount == Approx(1.0F));
            previousPosition = stop.position;

            for (const float channel : stop.colour) {
                CHECK(std::isfinite(channel));
                CHECK(channel >= 0.0F);
                CHECK(channel <= 1.0F);
            }
        }
    }
}

TEST_CASE(
    "Requested Timing Colourise palettes retain their source landmarks",
    "[timing][colourise][palette][preset]") {
    struct ExpectedLandmarks {
        std::string_view id;
        std::size_t stopCount;
        std::array<std::uint8_t, 3> first;
        std::array<std::uint8_t, 3> middle;
        std::array<std::uint8_t, 3> last;
    };
    constexpr std::array<ExpectedLandmarks, 12> kExpected{{
        {"matplotlib-inferno", 5U, {0, 0, 4}, {188, 55, 84}, {252, 255, 164}},
        {"seaborn-rocket", 5U, {3, 5, 26}, {203, 27, 79}, {250, 235, 221}},
        {"colorbrewer-ylgnbu", 5U, {255, 255, 217}, {65, 182, 196}, {8, 29, 88}},
        {"colorbrewer-purd", 5U, {247, 244, 249}, {223, 101, 176}, {103, 0, 31}},
        {"colorbrewer-blues", 5U, {247, 251, 255}, {107, 174, 214}, {8, 48, 107}},
        {"matplotlib-gist-earth", 5U, {0, 0, 0}, {94, 160, 75}, {253, 251, 251}},
        {"matplotlib-twilight-blue", 3U, {226, 217, 226}, {100, 116, 186}, {31, 37, 69}},
        {"matplotlib-twilight-red", 3U, {31, 37, 69}, {153, 46, 104}, {226, 217, 226}},
        {"crameri-batlow", 5U, {1, 25, 89}, {130, 130, 49}, {250, 204, 250}},
        {"crameri-lipari", 5U, {3, 19, 38}, {165, 98, 103}, {253, 245, 218}},
        {"crameri-romao", 5U, {115, 57, 87}, {203, 225, 179}, {115, 57, 87}},
        {"crameri-corko", 5U, {63, 62, 58}, {175, 203, 188}, {63, 62, 58}},
    }};

    const auto checkColour =
        [](const TimingColourisePaletteStop& stop,
           const std::array<std::uint8_t, 3>& expected) {
            constexpr float kByteToUnit = 1.0F / 255.0F;
            for (std::size_t channel = 0U;
                 channel < expected.size();
                 ++channel) {
                CHECK(
                    stop.colour[channel] ==
                    Approx(
                        static_cast<float>(expected[channel]) *
                        kByteToUnit));
            }
        };

    for (const auto& expected : kExpected) {
        CAPTURE(expected.id);
        const auto* preset =
            FindBuiltInTimingColourisePalettePreset(expected.id);
        REQUIRE(preset != nullptr);
        REQUIRE(preset->palette.stops.size() == expected.stopCount);
        checkColour(preset->palette.stops.front(), expected.first);
        checkColour(
            preset->palette.stops[preset->palette.stops.size() / 2U],
            expected.middle);
        checkColour(preset->palette.stops.back(), expected.last);
    }
}

TEST_CASE(
    "Built-in Timing Colourise presets copy into independent snapshots",
    "[timing][colourise][palette][preset]") {
    auto firstCopy = CopyBuiltInTimingColourisePalettePreset("seaborn-mako");
    auto secondCopy = CopyBuiltInTimingColourisePalettePreset("seaborn-mako");
    REQUIRE(firstCopy.has_value());
    REQUIRE(secondCopy.has_value());
    REQUIRE(firstCopy->stops.size() >= 2U);

    const auto* builtIn =
        FindBuiltInTimingColourisePalettePreset("seaborn-mako");
    REQUIRE(builtIn != nullptr);
    const float originalRed = builtIn->palette.stops.front().colour[0];
    const std::size_t originalStopCount = builtIn->palette.stops.size();

    firstCopy->stops.front().colour[0] = 1.0F;
    firstCopy->stops.pop_back();

    CHECK(builtIn->palette.stops.size() == originalStopCount);
    CHECK(builtIn->palette.stops.front().colour[0] == Approx(originalRed));
    CHECK(secondCopy->stops.size() == originalStopCount);
    CHECK(secondCopy->stops.front().colour[0] == Approx(originalRed));
    CHECK_FALSE(CopyBuiltInTimingColourisePalettePreset("not-a-preset"));
}
