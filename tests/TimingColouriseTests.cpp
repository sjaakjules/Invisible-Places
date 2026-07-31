#include "serialization/ProjectDocument.hpp"
#include "timing/TimingColourise.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using Catch::Approx;
using invisible_places::timing::TimingColouriseBounds;
using invisible_places::timing::TimingColouriseBoundsHandle;
using invisible_places::timing::TimingColouriseBoundsKeyMode;
using invisible_places::timing::TimingColouriseBoundsParameter;
using invisible_places::timing::TimingColouriseEffect;
using invisible_places::timing::TimingColourisePalette;
using invisible_places::timing::TimingColourisePaletteKeyModel;
using invisible_places::timing::TimingColourisePaletteSourceKind;
using invisible_places::timing::TimingColourisePaletteStop;
using invisible_places::timing::TimingColourisePaletteStopParameter;
using invisible_places::water::WaterScenarioInterpolation;

struct TemporaryTimingColouriseFile {
    explicit TemporaryTimingColouriseFile(std::string filename)
        : path(
              std::filesystem::temp_directory_path() /
              std::move(filename)) {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }

    ~TemporaryTimingColouriseFile() {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }

    std::filesystem::path path;
};

TimingColourisePalette Solid(
    std::array<float, 3> colour,
    float amount = 1.0F) {
    return TimingColourisePalette{
        .stops = {
            TimingColourisePaletteStop{
                .position = 0.0F,
                .colour = colour,
                .colouriseAmount = amount,
            },
        },
    };
}

}  // namespace

TEST_CASE(
    "Timing Colourise sanitizes palettes and produces deterministic LUTs",
    "[timing][colourise]") {
    TimingColourisePalette palette{
        .stops = {
            {.position = 1.4F,
             .colour = {2.0F, -1.0F, 0.5F},
             .colouriseAmount = 2.0F},
            {.position = -0.2F,
             .colour = {0.0F, 0.25F, 1.0F},
             .colouriseAmount = -1.0F},
        },
    };
    const auto sanitized =
        invisible_places::timing::SanitizeTimingColourisePalette(
            std::move(palette));
    REQUIRE(sanitized.stops.size() == 2U);
    CHECK(sanitized.stops.front().position == Approx(0.0F));
    CHECK(sanitized.stops.front().colouriseAmount == Approx(0.0F));
    CHECK(sanitized.stops.back().position == Approx(1.0F));
    CHECK(sanitized.stops.back().colour[0] == Approx(1.0F));
    CHECK(sanitized.stops.back().colour[1] == Approx(0.0F));

    const auto lut =
        invisible_places::timing::CompileTimingColourisePaletteLut(
            sanitized);
    CHECK(lut.size() == 64U);
    CHECK(lut.front()[2] == Approx(1.0F));
    CHECK(lut.front()[3] == Approx(0.0F));
    CHECK(lut.back()[0] == Approx(1.0F));
    CHECK(lut.back()[2] == Approx(0.5F));
    CHECK(lut.back()[3] == Approx(1.0F));

    const auto solidLut =
        invisible_places::timing::CompileTimingColourisePaletteLut(
            Solid({0.2F, 0.4F, 0.6F}, 0.3F));
    CHECK(solidLut.front() == solidLut.back());
    CHECK(solidLut[17][1] == Approx(0.4F));
    CHECK(solidLut[17][3] == Approx(0.3F));
}

TEST_CASE(
    "Timing Colourise palette stops receive stable unique identities",
    "[timing][colourise][palette][ids]") {
    TimingColourisePalette palette{
        .stops = {
            {.id = "mineral", .position = 0.8F},
            {.position = 0.2F},
            {.id = "mineral", .position = 0.5F},
            {.id = "palette-stop-1", .position = 0.1F},
        },
    };
    const auto sanitized =
        invisible_places::timing::SanitizeTimingColourisePalette(
            palette);
    REQUIRE(sanitized.stops.size() == 4U);
    CHECK(sanitized.stops[0].id == "palette-stop-1");
    CHECK(sanitized.stops[1].id == "palette-stop-2");
    // Duplicate repair follows canonical palette order, so the first stop at
    // that id retains it and the later duplicate receives a fresh identity.
    CHECK(sanitized.stops[2].id == "mineral");
    CHECK(sanitized.stops[3].id != "mineral");
    CHECK(
        invisible_places::timing::AllocateTimingColourisePaletteStopId(
            sanitized) == "palette-stop-4");

    auto moved = sanitized;
    const auto mineral = std::find_if(
        moved.stops.begin(),
        moved.stops.end(),
        [](const auto& stop) { return stop.id == "mineral"; });
    REQUIRE(mineral != moved.stops.end());
    mineral->position = 0.0F;
    moved = invisible_places::timing::SanitizeTimingColourisePalette(
        std::move(moved));
    CHECK(moved.stops.front().id == "mineral");
}

TEST_CASE(
    "Timing Colourise independently evaluates stop position colour and amount",
    "[timing][colourise][palette][parameters]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {{
                .id = "drifting-stop",
                .position = 0.15F,
                .colour = {0.2F, 0.2F, 0.2F},
                .colouriseAmount = 0.65F,
            }},
        });
    const auto linear = WaterScenarioInterpolation::Linear;
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "drifting-stop",
                TimingColourisePaletteStopParameter::Position,
                0.1F,
                0.0F,
                linear));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "drifting-stop",
                TimingColourisePaletteStopParameter::Position,
                0.3F,
                0.3F,
                linear));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "drifting-stop",
                TimingColourisePaletteStopParameter::Position,
                0.5F,
                0.0F,
                linear));
    const std::array firstColour{
        26.0F / 255.0F,
        87.0F / 255.0F,
        242.0F / 255.0F};
    const std::array secondColour{
        23.0F / 255.0F,
        250.0F / 255.0F,
        1.0F};
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "drifting-stop",
                0.2F,
                firstColour,
                linear));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "drifting-stop",
                0.5F,
                secondColour,
                linear));

    CHECK(effect.paletteKeyModel ==
          TimingColourisePaletteKeyModel::StopParameters);
    const auto atQuarter =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.25F);
    REQUIRE(atQuarter.stops.size() == 1U);
    CHECK(atQuarter.stops[0].position == Approx(0.225F));
    CHECK(
        atQuarter.stops[0].colour[0] ==
        Approx(std::lerp(firstColour[0], secondColour[0], 1.0F / 6.0F)));
    CHECK(atQuarter.stops[0].colouriseAmount == Approx(0.65F));

    const auto atFourTenths =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.4F);
    REQUIRE(atFourTenths.stops.size() == 1U);
    CHECK(atFourTenths.stops[0].position == Approx(0.15F));
    CHECK(
        atFourTenths.stops[0].colour[1] ==
        Approx(std::lerp(firstColour[1], secondColour[1], 2.0F / 3.0F)));

    const auto atSharedEnd =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.5F);
    CHECK(atSharedEnd.stops[0].position == Approx(0.0F));
    CHECK(atSharedEnd.stops[0].colour == secondColour);
    CHECK(
        invisible_places::timing::
            TimingColourisePaletteStopParameterKeyCountAtPosition(
                effect,
                "drifting-stop",
                TimingColourisePaletteStopParameter::Position,
                0.5F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColourisePaletteStopParameterKeyCountAtPosition(
                effect,
                "drifting-stop",
                TimingColourisePaletteStopParameter::Colour,
                0.5F) == 1U);
}

TEST_CASE(
    "Timing Colourise palette union moves only properties present at a time",
    "[timing][colourise][palette][parameters][move]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {{.id = "stop-a"}},
        });
    const auto addPosition = [&](float time, float value) {
        return invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::Position,
                time,
                value,
                WaterScenarioInterpolation::Linear);
    };
    const auto addColour = [&](float time, float red) {
        return invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "stop-a",
                time,
                {red, 0.0F, 0.0F},
                WaterScenarioInterpolation::Linear);
    };
    REQUIRE(addPosition(0.1F, 0.0F));
    REQUIRE(addColour(0.2F, 0.2F));
    REQUIRE(addPosition(0.5F, 0.5F));
    REQUIRE(addColour(0.5F, 0.5F));

    CHECK(
        invisible_places::timing::MoveTimingColourisePaletteKey(
            &effect,
            0.5F,
            0.6F));
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyCountAtPosition(
            effect,
            0.6F) == 2U);
    CHECK(
        invisible_places::timing::MoveTimingColourisePaletteKey(
            &effect,
            0.1F,
            0.25F));
    CHECK(
        invisible_places::timing::
            TimingColourisePaletteStopParameterKeyCountAtPosition(
                effect,
                "stop-a",
                TimingColourisePaletteStopParameter::Colour,
                0.2F) == 1U);

    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::ColouriseAmount,
                0.7F,
                0.4F));
    CHECK(
        invisible_places::timing::CanMoveTimingColourisePaletteKeysAtPosition(
            effect,
            0.6F,
            0.7F));
    CHECK(
        invisible_places::timing::MoveTimingColourisePaletteKey(
            &effect,
            0.6F,
            0.7F));
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyCountAtPosition(
            effect,
            0.7F) == 3U);

    REQUIRE(addPosition(0.9F, 0.9F));
    CHECK_FALSE(
        invisible_places::timing::CanMoveTimingColourisePaletteKeysAtPosition(
            effect,
            0.7F,
            0.9F));
    CHECK_FALSE(
        invisible_places::timing::MoveTimingColourisePaletteKey(
            &effect,
            0.7F,
            0.9F));
    // The failed union move is atomic: all three source properties remain.
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyCountAtPosition(
            effect,
            0.7F) == 3U);
}

TEST_CASE(
    "Timing Colourise palette union rejects legacy and same-track collisions",
    "[timing][colourise][palette][parameters][legacy][move]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {{.id = "stop-a"}},
        });
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::Position,
                0.2F,
                0.3F));
    // The compatibility API deliberately switches to the legacy model, but
    // retains dormant property data for a future explicit conversion.
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.8F,
        Solid({0.8F, 0.2F, 0.1F}));
    CHECK(effect.paletteKeyModel ==
          TimingColourisePaletteKeyModel::LegacySnapshots);
    CHECK_FALSE(
        invisible_places::timing::CanMoveTimingColourisePaletteKeysAtPosition(
            effect,
            0.2F,
            0.8F));
    CHECK_FALSE(
        invisible_places::timing::CanMoveTimingColourisePaletteKeysAtPosition(
            effect,
            0.8F,
            0.2F));
    CHECK_FALSE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "stop-a",
                0.4F,
                {0.0F, 1.0F, 0.0F}));

    const auto legacyHalfway =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.5F);
    // A single legacy snapshot endpoint-holds its own value. The dormant
    // stop-position key must not affect legacy evaluation.
    CHECK(legacyHalfway.front()[0] == Approx(0.8F));
    CHECK(legacyHalfway.front()[1] == Approx(0.2F));
}

TEST_CASE(
    "Timing Colourise palette key union navigates deletes and names by time order",
    "[timing][colourise][palette][parameters][navigation]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {{.id = "stop-a"}},
        });
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::Position,
                0.2F,
                0.1F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "stop-a",
                0.20005F,
                {1.0F, 0.0F, 0.0F}));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::ColouriseAmount,
                0.6F,
                0.5F));
    const auto positions =
        invisible_places::timing::TimingColourisePaletteKeyPositions(
            effect);
    REQUIRE(positions.size() == 2U);
    CHECK(positions[0] == Approx(0.2F));
    CHECK(positions[1] == Approx(0.6F));
    CHECK(
        invisible_places::timing::NextTimingColourisePaletteKeyPosition(
            effect,
            0.2F) == Approx(0.6F));
    CHECK(
        invisible_places::timing::PreviousTimingColourisePaletteKeyPosition(
            effect,
            0.6F) == Approx(0.2F));
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyStateName(
            "Mineral_edited",
            0U) == "Mineral_Run01");
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyStateName(
            "Mineral",
            11U) == "Mineral_Run12");
    CHECK(
        invisible_places::timing::RemoveTimingColourisePaletteKeysAtPosition(
            &effect,
            0.2F) == 2U);
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyPositions(effect) ==
        std::vector<float>{0.6F});
}

TEST_CASE(
    "Timing Colourise refuses to remove keyed palette topology",
    "[timing][colourise][palette][topology]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {
                {.id = "keyed", .position = 0.0F},
                {.id = "unkeyed", .position = 1.0F},
            },
        });
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "keyed",
                0.4F,
                {1.0F, 0.0F, 0.0F}));
    CHECK_FALSE(
        invisible_places::timing::CanRemoveTimingColourisePaletteStop(
            effect,
            "keyed"));
    CHECK_FALSE(
        invisible_places::timing::RemoveTimingColourisePaletteStop(
            &effect,
            "keyed"));
    CHECK(
        invisible_places::timing::CanRemoveTimingColourisePaletteStop(
            effect,
            "unkeyed"));
    CHECK(
        invisible_places::timing::RemoveTimingColourisePaletteStop(
            &effect,
            "unkeyed"));
    CHECK_FALSE(
        invisible_places::timing::RemoveTimingColourisePaletteStop(
            &effect,
            "keyed"));
}

TEST_CASE(
    "Timing Colourise palette source provenance remains evaluation metadata",
    "[timing][colourise][palette][source]") {
    TimingColouriseEffect effect;
    effect.paletteSourceKind = TimingColourisePaletteSourceKind::Saved;
    effect.paletteSourceId = "colourise-palette-7";
    effect.paletteSourceName = "Mineral";
    effect.paletteEdited = true;
    const auto sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    CHECK(sanitized.paletteSourceKind ==
          TimingColourisePaletteSourceKind::Saved);
    CHECK(sanitized.paletteSourceId == "colourise-palette-7");
    CHECK(sanitized.paletteSourceName == "Mineral");
    CHECK(sanitized.paletteEdited);

    effect.paletteSourceKind = TimingColourisePaletteSourceKind::Custom;
    const auto custom =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    CHECK(custom.paletteSourceId.empty());
    CHECK(custom.paletteSourceName == "Mineral");
}

TEST_CASE(
    "Timing Colourise interpolates different palette topologies as RGBA LUTs",
    "[timing][colourise][keys]") {
    TimingColouriseEffect effect;
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.0F,
        Solid({1.0F, 0.0F, 0.0F}, 0.2F),
        WaterScenarioInterpolation::Linear);
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        1.0F,
        TimingColourisePalette{
            .stops = {
                {.position = 0.0F,
                 .colour = {0.0F, 0.0F, 1.0F},
                 .colouriseAmount = 0.6F},
                {.position = 1.0F,
                 .colour = {0.0F, 1.0F, 0.0F},
                 .colouriseAmount = 1.0F},
            },
        });

    const auto halfway =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.5F);
    CHECK(halfway.front()[0] == Approx(0.5F));
    CHECK(halfway.front()[2] == Approx(0.5F));
    CHECK(halfway.front()[3] == Approx(0.4F));
    CHECK(halfway.back()[0] == Approx(0.5F));
    CHECK(halfway.back()[1] == Approx(0.5F));
    CHECK(halfway.back()[3] == Approx(0.6F));

    effect.paletteKeys.front().interpolation =
        WaterScenarioInterpolation::Hold;
    const auto held =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.999F);
    CHECK(held.front()[0] == Approx(1.0F));
    CHECK(held.front()[2] == Approx(0.0F));
    CHECK(
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            1.0F)
            .front()[2] == Approx(1.0F));
}

TEST_CASE(
    "Timing Colourise bounds key independently and fade only inside their range",
    "[timing][colourise][bounds]") {
    TimingColouriseEffect effect;
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        0.0F,
        {.lower = 10.0F, .upper = 20.0F, .edgeFade = 0.1F},
        WaterScenarioInterpolation::Linear);
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        1.0F,
        {.lower = 20.0F, .upper = 40.0F, .edgeFade = 0.3F});

    const auto bounds =
        invisible_places::timing::EvaluateTimingColouriseBounds(
            effect,
            0.5F);
    CHECK(bounds.lower == Approx(15.0F));
    CHECK(bounds.upper == Approx(30.0F));
    CHECK(bounds.edgeFade == Approx(0.2F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 14.9F) ==
        Approx(0.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 15.0F) ==
        Approx(0.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 16.5F) ==
        Approx(0.5F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 22.0F) ==
        Approx(1.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(
            {.lower = 1.0F, .upper = 1.0F, .edgeFade = 0.0F},
            1.0F) == Approx(0.0F));
}

TEST_CASE(
    "Timing Colourise bounds parameterisations key independent non-conflicting controls",
    "[timing][colourise][bounds][parameters]") {
    TimingColouriseEffect effect;
    effect.baseBounds = {
        .lower = 10.0F,
        .upper = 20.0F,
        .edgeFade = 0.1F};

    SECTION("Lower and Upper") {
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Lower,
                    0.0F,
                    0.0F,
                    WaterScenarioInterpolation::Linear));
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Lower,
                    1.0F,
                    10.0F));
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Upper,
                    0.5F,
                    30.0F));
        CHECK_FALSE(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Spread,
                    0.5F,
                    5.0F));

        const auto halfway =
            invisible_places::timing::EvaluateTimingColouriseBounds(
                effect,
                0.5F);
        CHECK(halfway.lower == Approx(5.0F));
        CHECK(halfway.upper == Approx(30.0F));
        CHECK(halfway.edgeFade == Approx(0.1F));
    }

    SECTION("Crossing endpoints collapse without exchanging identities") {
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Lower,
                    0.5F,
                    30.0F));
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Upper,
                    0.5F,
                    10.0F));
        const auto crossing =
            invisible_places::timing::EvaluateTimingColouriseBounds(
                effect,
                0.5F);
        CHECK(crossing.lower == Approx(20.0F));
        CHECK(crossing.upper == Approx(20.0F));
    }

    SECTION("Centre and Spread") {
        REQUIRE(
            invisible_places::timing::SetTimingColouriseBoundsKeyMode(
                &effect,
                TimingColouriseBoundsKeyMode::CentreSpread));
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Centre,
                    0.0F,
                    10.0F,
                    WaterScenarioInterpolation::Linear));
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Centre,
                    1.0F,
                    30.0F));
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Spread,
                    0.0F,
                    4.0F,
                    WaterScenarioInterpolation::Linear));
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Spread,
                    1.0F,
                    8.0F));
        CHECK(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::EdgeFade,
                    0.5F,
                    0.75F));

        const auto halfway =
            invisible_places::timing::EvaluateTimingColouriseBounds(
                effect,
                0.5F);
        CHECK(halfway.lower == Approx(17.0F));
        CHECK(halfway.upper == Approx(23.0F));
        CHECK(halfway.edgeFade == Approx(0.5F));
        CHECK_FALSE(
            invisible_places::timing::SetTimingColouriseBoundsKeyMode(
                &effect,
                TimingColouriseBoundsKeyMode::LowerSpread));
    }

    SECTION("Lower plus Spread and Upper plus Spread") {
        REQUIRE(
            invisible_places::timing::SetTimingColouriseBoundsKeyMode(
                &effect,
                TimingColouriseBoundsKeyMode::LowerSpread));
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Lower,
                    0.25F,
                    4.0F));
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Spread,
                    0.25F,
                    6.0F));
        const auto lowerSpread =
            invisible_places::timing::EvaluateTimingColouriseBounds(
                effect,
                0.25F);
        CHECK(lowerSpread.lower == Approx(4.0F));
        CHECK(lowerSpread.upper == Approx(10.0F));

        effect.boundsParameterKeys.clear();
        REQUIRE(
            invisible_places::timing::SetTimingColouriseBoundsKeyMode(
                &effect,
                TimingColouriseBoundsKeyMode::UpperSpread));
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Upper,
                    0.25F,
                    16.0F));
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Spread,
                    0.25F,
                    -4.0F));
        const auto upperSpread =
            invisible_places::timing::EvaluateTimingColouriseBounds(
                effect,
                0.25F);
        CHECK(upperSpread.lower == Approx(16.0F));
        CHECK(upperSpread.upper == Approx(16.0F));
    }
}

TEST_CASE(
    "Timing Colourise histogram handles follow the selected bounds coordinates",
    "[timing][colourise][bounds][handles]") {
    const TimingColouriseBounds current{
        .lower = 2.0F,
        .upper = 6.0F,
        .edgeFade = 0.2F};
    const auto resolve =
        [&](TimingColouriseBoundsKeyMode mode,
            TimingColouriseBoundsHandle handle,
            float target) {
            const auto edit =
                invisible_places::timing::
                    ResolveTimingColouriseBoundsHandleEdit(
                        mode,
                        handle,
                        current,
                        target,
                        0.0F,
                        10.0F);
            REQUIRE(edit.has_value());
            CHECK(edit->bounds.edgeFade == Approx(0.2F));
            return edit.value();
        };

    SECTION("Lower and Upper edit endpoints while Centre translates") {
        const auto lower = resolve(
            TimingColouriseBoundsKeyMode::LowerUpper,
            TimingColouriseBoundsHandle::Lower,
            1.0F);
        CHECK(lower.bounds.lower == Approx(1.0F));
        CHECK(lower.bounds.upper == Approx(6.0F));
        REQUIRE(lower.parameterCount == 1U);
        CHECK(
            lower.parameters[0] ==
            TimingColouriseBoundsParameter::Lower);

        const auto upper = resolve(
            TimingColouriseBoundsKeyMode::LowerUpper,
            TimingColouriseBoundsHandle::Upper,
            8.0F);
        CHECK(upper.bounds.lower == Approx(2.0F));
        CHECK(upper.bounds.upper == Approx(8.0F));
        REQUIRE(upper.parameterCount == 1U);
        CHECK(
            upper.parameters[0] ==
            TimingColouriseBoundsParameter::Upper);

        const auto centre = resolve(
            TimingColouriseBoundsKeyMode::LowerUpper,
            TimingColouriseBoundsHandle::Centre,
            7.0F);
        CHECK(centre.bounds.lower == Approx(5.0F));
        CHECK(centre.bounds.upper == Approx(9.0F));
        CHECK(centre.parameterCount == 2U);
    }

    SECTION("Centre and Spacing mirror either endpoint") {
        const auto lower = resolve(
            TimingColouriseBoundsKeyMode::CentreSpread,
            TimingColouriseBoundsHandle::Lower,
            1.0F);
        CHECK(lower.bounds.lower == Approx(1.0F));
        CHECK(lower.bounds.upper == Approx(7.0F));
        REQUIRE(lower.parameterCount == 1U);
        CHECK(
            lower.parameters[0] ==
            TimingColouriseBoundsParameter::Spread);

        const auto upper = resolve(
            TimingColouriseBoundsKeyMode::CentreSpread,
            TimingColouriseBoundsHandle::Upper,
            7.0F);
        CHECK(upper.bounds.lower == Approx(1.0F));
        CHECK(upper.bounds.upper == Approx(7.0F));
        REQUIRE(upper.parameterCount == 1U);
        CHECK(
            upper.parameters[0] ==
            TimingColouriseBoundsParameter::Spread);

        const auto centre = resolve(
            TimingColouriseBoundsKeyMode::CentreSpread,
            TimingColouriseBoundsHandle::Centre,
            7.0F);
        CHECK(centre.bounds.lower == Approx(5.0F));
        CHECK(centre.bounds.upper == Approx(9.0F));
        REQUIRE(centre.parameterCount == 1U);
        CHECK(
            centre.parameters[0] ==
            TimingColouriseBoundsParameter::Centre);
    }

    SECTION("Lower and Spacing translate from Lower and resize from Upper") {
        const auto lower = resolve(
            TimingColouriseBoundsKeyMode::LowerSpread,
            TimingColouriseBoundsHandle::Lower,
            4.0F);
        CHECK(lower.bounds.lower == Approx(4.0F));
        CHECK(lower.bounds.upper == Approx(8.0F));
        REQUIRE(lower.parameterCount == 1U);
        CHECK(
            lower.parameters[0] ==
            TimingColouriseBoundsParameter::Lower);

        const auto upper = resolve(
            TimingColouriseBoundsKeyMode::LowerSpread,
            TimingColouriseBoundsHandle::Upper,
            9.0F);
        CHECK(upper.bounds.lower == Approx(2.0F));
        CHECK(upper.bounds.upper == Approx(9.0F));
        REQUIRE(upper.parameterCount == 1U);
        CHECK(
            upper.parameters[0] ==
            TimingColouriseBoundsParameter::Spread);

        const auto centre = resolve(
            TimingColouriseBoundsKeyMode::LowerSpread,
            TimingColouriseBoundsHandle::Centre,
            7.0F);
        CHECK(centre.bounds.lower == Approx(5.0F));
        CHECK(centre.bounds.upper == Approx(9.0F));
        REQUIRE(centre.parameterCount == 1U);
        CHECK(
            centre.parameters[0] ==
            TimingColouriseBoundsParameter::Lower);
    }

    SECTION("Upper and Spacing translate from Upper and resize from Lower") {
        const auto lower = resolve(
            TimingColouriseBoundsKeyMode::UpperSpread,
            TimingColouriseBoundsHandle::Lower,
            1.0F);
        CHECK(lower.bounds.lower == Approx(1.0F));
        CHECK(lower.bounds.upper == Approx(6.0F));
        REQUIRE(lower.parameterCount == 1U);
        CHECK(
            lower.parameters[0] ==
            TimingColouriseBoundsParameter::Spread);

        const auto upper = resolve(
            TimingColouriseBoundsKeyMode::UpperSpread,
            TimingColouriseBoundsHandle::Upper,
            8.0F);
        CHECK(upper.bounds.lower == Approx(4.0F));
        CHECK(upper.bounds.upper == Approx(8.0F));
        REQUIRE(upper.parameterCount == 1U);
        CHECK(
            upper.parameters[0] ==
            TimingColouriseBoundsParameter::Upper);

        const auto centre = resolve(
            TimingColouriseBoundsKeyMode::UpperSpread,
            TimingColouriseBoundsHandle::Centre,
            7.0F);
        CHECK(centre.bounds.lower == Approx(5.0F));
        CHECK(centre.bounds.upper == Approx(9.0F));
        REQUIRE(centre.parameterCount == 1U);
        CHECK(
            centre.parameters[0] ==
            TimingColouriseBoundsParameter::Upper);
    }

    SECTION("Translations clamp as an interval without changing spacing") {
        const auto lower = resolve(
            TimingColouriseBoundsKeyMode::LowerSpread,
            TimingColouriseBoundsHandle::Lower,
            9.0F);
        CHECK(lower.bounds.lower == Approx(6.0F));
        CHECK(lower.bounds.upper == Approx(10.0F));

        const auto mirrored = resolve(
            TimingColouriseBoundsKeyMode::CentreSpread,
            TimingColouriseBoundsHandle::Lower,
            -5.0F);
        CHECK(mirrored.bounds.lower == Approx(0.0F));
        CHECK(mirrored.bounds.upper == Approx(8.0F));
    }
}

TEST_CASE(
    "Timing Colourise local key moves are isolated to their own lane",
    "[timing][colourise][keys][local-move]") {
    TimingColouriseEffect effect;
    REQUIRE(
        invisible_places::timing::
            SetTimingColouriseBoundsKeyMode(
                &effect,
                TimingColouriseBoundsKeyMode::CentreSpread));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Centre,
                0.2F,
                1.0F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Centre,
                0.8F,
                2.0F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Spread,
                0.4F,
                3.0F));

    CHECK(
        invisible_places::timing::
            MoveTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Centre,
                0.2F,
                0.4F));
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Centre,
                0.4F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Spread,
                0.4F) == 1U);
    CHECK_FALSE(
        invisible_places::timing::
            MoveTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Centre,
                0.4F,
                0.8F));
    CHECK_FALSE(
        invisible_places::timing::
            MoveTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Centre,
                0.4F,
                -0.1F));

    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.1F,
        Solid({1.0F, 0.0F, 0.0F}));
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.9F,
        Solid({0.0F, 0.0F, 1.0F}));
    CHECK(
        invisible_places::timing::
            MoveTimingColourisePaletteKey(
                &effect,
                0.1F,
                0.4F));
    CHECK_FALSE(
        invisible_places::timing::
            MoveTimingColourisePaletteKey(
                &effect,
                0.4F,
                0.9F));
}

TEST_CASE(
    "Timing Colourise parameter keys preserve legacy bounds and join bounds navigation",
    "[timing][colourise][bounds][parameters][navigation]") {
    TimingColouriseEffect effect;
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        0.0F,
        {.lower = 0.0F, .upper = 10.0F, .edgeFade = 0.1F},
        WaterScenarioInterpolation::Linear);
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        1.0F,
        {.lower = 10.0F, .upper = 30.0F, .edgeFade = 0.3F});
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Lower,
                0.25F,
                7.0F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Upper,
                0.75F,
                40.0F));

    const auto start =
        invisible_places::timing::EvaluateTimingColouriseBounds(
            effect,
            0.0F);
    const auto end =
        invisible_places::timing::EvaluateTimingColouriseBounds(
            effect,
            1.0F);
    CHECK(start.lower == Approx(0.0F));
    CHECK(start.upper == Approx(10.0F));
    CHECK(start.edgeFade == Approx(0.1F));
    CHECK(end.lower == Approx(10.0F));
    CHECK(end.upper == Approx(30.0F));
    CHECK(end.edgeFade == Approx(0.3F));
    CHECK(
        invisible_places::timing::
            PreviousTimingColouriseBoundsKeyPosition(effect, 0.5F) ==
        Approx(0.25F));
    CHECK(
        invisible_places::timing::NextTimingColouriseBoundsKeyPosition(
            effect,
            0.5F) == Approx(0.75F));
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.25F) == 1U);
    CHECK_FALSE(
        invisible_places::timing::MoveTimingColouriseBoundsKey(
            &effect,
            0.25F,
            0.75F));
    CHECK(
        invisible_places::timing::MoveTimingColouriseBoundsKey(
            &effect,
            0.25F,
            0.4F));
    CHECK(
        invisible_places::timing::
            PreviousTimingColouriseBoundsParameterKeyPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.5F) == Approx(0.4F));
    CHECK(
        invisible_places::timing::
            RemoveTimingColouriseBoundsParameterKeysAtPosition(
                &effect,
                TimingColouriseBoundsParameter::Lower,
                0.4F) == 1U);
    CHECK(
        invisible_places::timing::RemoveTimingColouriseBoundsKeysAtPosition(
            &effect,
            1.0F) == 3U);
}

TEST_CASE(
    "Timing Colourise key helpers keep local tracks separate and union them for global controls",
    "[timing][colourise][navigation]") {
    TimingColouriseEffect effect;
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.2F,
        Solid({1.0F, 0.0F, 0.0F}));
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.8F,
        Solid({0.0F, 1.0F, 0.0F}));
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        0.5F,
        {.lower = 0.1F, .upper = 0.9F, .edgeFade = 0.1F});
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        0.80005F,
        {.lower = 0.2F, .upper = 0.8F, .edgeFade = 0.2F});

    CHECK(
        invisible_places::timing::
            PreviousTimingColouriseEffectKeyPosition(effect, 0.7F) ==
        Approx(0.5F));
    CHECK(
        invisible_places::timing::NextTimingColouriseEffectKeyPosition(
            effect,
            0.5F) == Approx(0.8F));
    CHECK(
        invisible_places::timing::TimingColouriseEffectKeyCountAtPosition(
            effect,
            0.8F) == 2U);
    CHECK(
        invisible_places::timing::RemoveTimingColourisePaletteKeysAtPosition(
            &effect,
            0.8F) == 1U);
    CHECK(
        invisible_places::timing::TimingColouriseBoundsKeyCountAtPosition(
            effect,
            0.8F) == 1U);
    CHECK(
        invisible_places::timing::MoveTimingColouriseBoundsKey(
            &effect,
            0.8F,
            0.9F));
    CHECK_FALSE(
        invisible_places::timing::MoveTimingColouriseBoundsKey(
            &effect,
            0.9F,
            0.5F));
}

TEST_CASE(
    "Timing Colourise stack gives the top list item final priority",
    "[timing][colourise][stack]") {
    const std::array samples{
        invisible_places::timing::TimingColouriseLayerSample{
            .colour = {1.0F, 0.0F, 0.0F},
            .colouriseAmount = 0.5F},
        invisible_places::timing::TimingColouriseLayerSample{
            .colour = {0.0F, 0.0F, 1.0F},
            .colouriseAmount = 1.0F},
    };
    const auto result =
        invisible_places::timing::ApplyTimingColouriseStack(
            {0.0F, 1.0F, 0.0F},
            samples);
    CHECK(result[0] == Approx(0.5F));
    CHECK(result[1] == Approx(0.0F));
    CHECK(result[2] == Approx(0.5F));
}

TEST_CASE(
    "Timing take scene state keeps stable effect order and compound identity",
    "[timing][colourise][state]") {
    std::vector<invisible_places::timing::TimingTakeSceneState> states;
    auto* siteA =
        invisible_places::timing::EnsureTimingTakeSceneState(
            &states,
            "",
            "Site A");
    REQUIRE(siteA != nullptr);
    CHECK(
        siteA->takeId ==
        invisible_places::timing::kAuthoredTimingTakeId);
    auto* siteB =
        invisible_places::timing::EnsureTimingTakeSceneState(
            &states,
            "",
            "Site B");
    REQUIRE(siteB != nullptr);
    REQUIRE(states.size() == 2U);

    for (std::size_t index = 0U; index < 6U; ++index) {
        siteA = &states.front();
        siteA->colouriseEffects.push_back({
            .id = "effect-" + std::to_string(index),
            .name = "Effect " + std::to_string(index),
        });
    }
    auto sanitized =
        invisible_places::timing::SanitizeTimingTakeSceneState(*siteA);
    REQUIRE(
        sanitized.colouriseEffects.size() ==
        invisible_places::timing::kMaximumTimingColouriseEffects);
    CHECK(sanitized.colouriseEffects.front().id == "effect-0");
    CHECK(sanitized.colouriseEffects.back().id == "effect-4");

    std::uint32_t sequence = 1U;
    sanitized.colouriseEffects.front().id = "colourise-effect-1";
    CHECK(
        invisible_places::timing::AllocateTimingColouriseEffectId(
            sanitized.colouriseEffects,
            &sequence) == "colourise-effect-2");
    CHECK(sequence == 3U);
}

TEST_CASE(
    "Empty timing takes are water-neutral and saved palettes copy by value",
    "[timing][colourise][state][water]") {
    invisible_places::timing::TimingTakeSceneState state{
        .takeId = "empty-take",
        .sceneGroupName = "Site A",
    };
    invisible_places::water::WaterScenarioState authored;
    authored.rainLevel = 0.37F;
    authored.meshFlowLevel = 0.61F;
    authored.shorelineLevel = 0.82F;
    const auto overlay =
        invisible_places::water::BuildWaterFeatureTimingOverlay(
            state.waterFeatureTimingRuns,
            0.75F);
    CHECK(overlay.samples.empty());
    invisible_places::water::
        ApplyWaterFeatureTimingOverlayToScenario(
            overlay,
            &authored);
    CHECK(authored.rainLevel == Approx(0.37F));
    CHECK(authored.meshFlowLevel == Approx(0.61F));
    CHECK(authored.shorelineLevel == Approx(0.82F));

    invisible_places::timing::TimingColourisePaletteDefinition saved{
        .id = "saved-palette",
        .name = "Saved Palette",
        .palette =
            Solid({0.15F, 0.35F, 0.75F}, 0.6F),
    };
    TimingColouriseEffect effect;
    effect.basePalette = saved.palette;
    saved.palette.stops.front().colour = {
        1.0F,
        0.0F,
        0.0F};
    saved.name = "Renamed Palette";
    CHECK(
        effect.basePalette.stops.front().colour[0] ==
        Approx(0.15F));
    CHECK(
        effect.basePalette.stops.front().colouriseAmount ==
        Approx(0.6F));
}

TEST_CASE(
    "Timing takes and saved palettes persist compound scene state",
    "[timing][colourise][project][serialization]") {
    invisible_places::serialization::ProjectDocument document;
    document.projectName = "Timing Colourise";
    document.timingTakes.push_back(
        {.id = "timing-take-7", .name = "Storm build"});
    document.selectedTimingTakeId = "timing-take-7";
    document.timingColourisePalettes.push_back({
        .id = "colourise-palette-2",
        .name = "Mineral",
        .palette = Solid({0.1F, 0.4F, 0.8F}, 0.75F),
    });

    invisible_places::timing::TimingTakeSceneState state{
        .takeId = "timing-take-7",
        .sceneGroupName = "Site A",
    };
    invisible_places::water::WaterFeatureTimingRun run;
    run.id = 3U;
    run.name = "Rain run";
    state.waterFeatureTimingRuns.push_back(run);
    TimingColouriseEffect effect;
    effect.id = "colourise-effect-4";
    effect.name = "Roughness";
    effect.field.scalarFieldName = "SurfaceRoughness-Sml";
    effect.baseBounds = {.lower = -2.0F, .upper = 4.0F, .edgeFade = 0.2F};
    effect.boundsKeyMode =
        invisible_places::timing::
            TimingColouriseBoundsKeyMode::CentreSpread;
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.25F,
        Solid({1.0F, 0.2F, 0.1F}, 0.4F));
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        0.75F,
        {.lower = -1.0F, .upper = 2.0F, .edgeFade = 0.1F});
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                invisible_places::timing::
                    TimingColouriseBoundsParameter::Centre,
                0.5F,
                1.25F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                invisible_places::timing::
                    TimingColouriseBoundsParameter::Spread,
                0.5F,
                3.5F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                invisible_places::timing::
                    TimingColouriseBoundsParameter::EdgeFade,
                0.5F,
                0.15F));
    state.colouriseEffects.push_back(effect);
    document.timingTakeStates.push_back(state);

    TemporaryTimingColouriseFile file{
        "invisible_places_timing_colourise_round_trip.ipproj"};
    std::string error;
    REQUIRE(
        invisible_places::serialization::SaveProjectDocument(
            document,
            file.path,
            &error));
    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(
            file.path,
            &error);
    REQUIRE(loaded.has_value());
    CHECK(
        loaded->schemaVersion ==
        invisible_places::serialization::kProjectDocumentSchemaVersion);
    CHECK(loaded->selectedTimingTakeId == "timing-take-7");
    REQUIRE(loaded->timingTakeStates.size() == 1U);
    const auto& loadedState = loaded->timingTakeStates.front();
    CHECK(loadedState.takeId == "timing-take-7");
    CHECK(loadedState.sceneGroupName == "Site A");
    REQUIRE(loadedState.waterFeatureTimingRuns.size() == 1U);
    REQUIRE(loadedState.colouriseEffects.size() == 1U);
    CHECK(
        loadedState.colouriseEffects.front().field.scalarFieldName ==
        "SurfaceRoughness-Sml");
    CHECK(
        loadedState.colouriseEffects.front().paletteKeys.front().palette
            .stops.front()
            .colouriseAmount == Approx(0.4F));
    CHECK(
        loadedState.colouriseEffects.front().boundsKeys.front().bounds.upper ==
        Approx(2.0F));
    CHECK(
        loadedState.colouriseEffects.front().boundsKeyMode ==
        invisible_places::timing::
            TimingColouriseBoundsKeyMode::CentreSpread);
    REQUIRE(
        loadedState.colouriseEffects.front()
            .boundsParameterKeys.size() == 6U);
    CHECK(
        loadedState.colouriseEffects.front()
            .boundsParameterKeys.front()
            .parameter ==
        invisible_places::timing::
            TimingColouriseBoundsParameter::Centre);
    CHECK(
        loadedState.colouriseEffects.front()
            .boundsParameterKeys.front()
            .value == Approx(1.25F));
    REQUIRE(loaded->timingColourisePalettes.size() == 1U);
    CHECK(loaded->timingColourisePalettes.front().name == "Mineral");
}

TEST_CASE(
    "Authored Seepage and Mesh Flow rain response controls round-trip",
    "[timing][authored-water][project][serialization]") {
    invisible_places::serialization::ProjectDocument document;
    document.projectName = "Authored water response";
    invisible_places::water::WaterSeepageNode node;
    node.id = 17U;
    node.name = "Delayed wetting";
    node.rainDelaySeconds = 2.5F;
    node.rainRiseSeconds = 7.0F;
    node.rainRecessionSeconds = 31.0F;
    document.waterSeepageNodes.push_back(node);

    document.waterDynamicMeshFlowSettings.activity = 0.35F;
    document.waterDynamicMeshFlowSettings.rainGain = 1.75F;
    document.waterDynamicMeshFlowSettings.moisturePersistenceMultiplier =
        2.25F;
    document.waterDynamicMeshFlowSettings.rainRiseSeconds = 4.5F;
    document.waterDynamicMeshFlowSettings.rainRecessionSeconds = 28.0F;

    TemporaryTimingColouriseFile file{
        "invisible_places_authored_water_response.ipproj"};
    std::string error;
    REQUIRE(
        invisible_places::serialization::SaveProjectDocument(
            document,
            file.path,
            &error));
    {
        std::ifstream input{file.path};
        REQUIRE(input.is_open());
        const auto saved = nlohmann::json::parse(input);
        const auto& savedNode = saved.at("water_seepage_nodes").front();
        CHECK(savedNode.at("rain_delay_seconds") == Approx(2.5F));
        CHECK(savedNode.at("rain_rise_seconds") == Approx(7.0F));
        CHECK(
            savedNode.at("rain_recession_seconds") ==
            Approx(31.0F));
        const auto& mesh =
            saved.at("water_dynamic_mesh_flow_settings");
        CHECK(mesh.at("activity") == Approx(0.35F));
        CHECK(mesh.at("rain_gain") == Approx(1.75F));
        CHECK(
            mesh.at("moisture_persistence_multiplier") ==
            Approx(2.25F));
        CHECK(mesh.at("rain_rise_seconds") == Approx(4.5F));
        CHECK(
            mesh.at("rain_recession_seconds") ==
            Approx(28.0F));
    }

    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(
            file.path,
            &error);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterSeepageNodes.size() == 1U);
    CHECK(
        loaded->waterSeepageNodes.front().rainDelaySeconds ==
        Approx(2.5F));
    CHECK(
        loaded->waterSeepageNodes.front().rainRiseSeconds ==
        Approx(7.0F));
    CHECK(
        loaded->waterSeepageNodes.front().rainRecessionSeconds ==
        Approx(31.0F));
    CHECK(
        loaded->waterDynamicMeshFlowSettings.activity ==
        Approx(0.35F));
    CHECK(
        loaded->waterDynamicMeshFlowSettings.rainGain ==
        Approx(1.75F));
    CHECK(
        loaded->waterDynamicMeshFlowSettings
            .moisturePersistenceMultiplier == Approx(2.25F));
    CHECK(
        loaded->waterDynamicMeshFlowSettings.rainRiseSeconds ==
        Approx(4.5F));
    CHECK(
        loaded->waterDynamicMeshFlowSettings.rainRecessionSeconds ==
        Approx(28.0F));
}

TEST_CASE(
    "Legacy scenarios and feature runs migrate into timing takes without being discarded",
    "[timing][migration][project][animation]") {
    TemporaryTimingColouriseFile projectFile{
        "invisible_places_timing_take_legacy.ipproj"};
    {
        std::ofstream output{projectFile.path};
        const nlohmann::json legacy{
            {"schema_version", 48U},
            {"project_name", "Legacy timing"},
            {"active_water_scene_group", "Site B"},
            {"selected_water_scenario", "storm"},
            {"water_scenarios",
             nlohmann::json::array(
                 {{{"id", "storm"}, {"name", "Storm"}}})},
            {"water_feature_timing_runs",
             nlohmann::json::array(
                 {{{"scenario_id", "storm"},
                   {"runs",
                    nlohmann::json::array(
                        {{{"id", 9U},
                          {"name", "Legacy run"},
                          {"features", nlohmann::json::array()}}})}}})},
        };
        output << legacy.dump(2);
    }

    std::string error;
    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(
            projectFile.path,
            &error);
    REQUIRE(loaded.has_value());
    CHECK(loaded->selectedTimingTakeId == "storm");
    REQUIRE(
        invisible_places::timing::FindTimingTakeDefinition(
            loaded->timingTakes,
            invisible_places::timing::kAuthoredTimingTakeId) != nullptr);
    const auto* storm =
        invisible_places::timing::FindTimingTakeDefinition(
            loaded->timingTakes,
            "storm");
    REQUIRE(storm != nullptr);
    CHECK(storm->name == "Storm");
    const auto* state =
        invisible_places::timing::FindTimingTakeSceneState(
            loaded->timingTakeStates,
            "storm",
            "Site B");
    REQUIRE(state != nullptr);
    REQUIRE(state->waterFeatureTimingRuns.size() == 1U);
    CHECK(state->waterFeatureTimingRuns.front().id == 9U);
    // The compatibility representation is retained for the next save.
    REQUIRE(loaded->waterFeatureTimingRuns.size() == 1U);
    CHECK(loaded->waterFeatureTimingRuns.front().scenarioId == "storm");

    TemporaryTimingColouriseFile migratedProjectFile{
        "invisible_places_timing_take_migrated.ipproj"};
    REQUIRE(
        invisible_places::serialization::SaveProjectDocument(
            *loaded,
            migratedProjectFile.path,
            &error));
    {
        std::ifstream input{migratedProjectFile.path};
        REQUIRE(input.is_open());
        const auto migratedJson = nlohmann::json::parse(input);
        CHECK(
            migratedJson.at("schema_version") ==
            invisible_places::serialization::
                kProjectDocumentSchemaVersion);
        REQUIRE(
            migratedJson.at("water_feature_timing_runs").size() == 1U);
        REQUIRE(migratedJson.at("timing_take_states").size() == 1U);
        CHECK(
            migratedJson.at("timing_take_states")[0].at("take_id") ==
            "storm");
    }

    TemporaryTimingColouriseFile animationFile{
        "invisible_places_timing_take_legacy.ipanim"};
    {
        std::ofstream output{animationFile.path};
        output << nlohmann::json{
            {"schema_version", 13U},
            {"name", "Legacy animation"},
            {"selected_water_scenario_id", "storm"},
            {"keys", nlohmann::json::array()},
        }.dump(2);
    }
    const auto animation =
        invisible_places::serialization::LoadAnimationPath(
            animationFile.path,
            &error);
    REQUIRE(animation.has_value());
    CHECK(animation->selectedTimingTakeId == "storm");

    invisible_places::camera::AnimationPath authoredAnimation;
    authoredAnimation.selectedTimingTakeId.clear();
    TemporaryTimingColouriseFile authoredAnimationFile{
        "invisible_places_authored_timing.ipanim"};
    REQUIRE(
        invisible_places::serialization::SaveAnimationPath(
            authoredAnimation,
            authoredAnimationFile.path,
            &error));
    {
        std::ifstream input{authoredAnimationFile.path};
        REQUIRE(input.is_open());
        const auto savedAnimationJson = nlohmann::json::parse(input);
        CHECK(
            savedAnimationJson.at("schema_version") ==
            invisible_places::serialization::
                kAnimationDocumentSchemaVersion);
        CHECK(
            savedAnimationJson.at("selected_timing_take_id") ==
            invisible_places::timing::kAuthoredTimingTakeId);
    }
    const auto authoredLoaded =
        invisible_places::serialization::LoadAnimationPath(
            authoredAnimationFile.path,
            &error);
    REQUIRE(authoredLoaded.has_value());
    CHECK(
        authoredLoaded->selectedTimingTakeId ==
        invisible_places::timing::kAuthoredTimingTakeId);
}
