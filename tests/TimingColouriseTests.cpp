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
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using Catch::Approx;
using invisible_places::timing::TimingColouriseBounds;
using invisible_places::timing::TimingColouriseBoundsHandle;
using invisible_places::timing::TimingColouriseBoundsKeyMode;
using invisible_places::timing::TimingColouriseBoundsParameter;
using invisible_places::timing::TimingColouriseAmountOverrideMode;
using invisible_places::timing::TimingColouriseEffect;
using invisible_places::timing::TimingColouriseEffectParameter;
using invisible_places::timing::TimingColouriseFieldBoundsMemory;
using invisible_places::timing::TimingColouriseLocalPaletteEdit;
using invisible_places::timing::TimingColourisePalette;
using invisible_places::timing::TimingColourisePaletteDefinition;
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
    "Timing Colourise activation ranges are inclusive and safely sanitized",
    "[timing][colourise][activation]") {
    using invisible_places::timing::TimingColouriseActivationRange;

    TimingColouriseEffect effect;
    CHECK(effect.activationRange.start == Approx(0.0F));
    CHECK(effect.activationRange.end == Approx(1.0F));
    CHECK(invisible_places::timing::TimingColouriseEffectIsActiveAt(
        effect,
        0.0F));
    CHECK(invisible_places::timing::TimingColouriseEffectIsActiveAt(
        effect,
        1.0F));

    effect.activationRange = {.start = 0.75F, .end = 0.25F};
    auto sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    CHECK(sanitized.activationRange.start == Approx(0.25F));
    CHECK(sanitized.activationRange.end == Approx(0.75F));

    effect.activationRange = {
        .start = std::numeric_limits<float>::quiet_NaN(),
        .end = std::numeric_limits<float>::infinity(),
    };
    sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    CHECK(sanitized.activationRange.start == Approx(0.0F));
    CHECK(sanitized.activationRange.end == Approx(1.0F));

    const auto clamped =
        invisible_places::timing::SanitizeTimingColouriseActivationRange(
            {.start = -2.0F, .end = 3.0F});
    CHECK(clamped.start == Approx(0.0F));
    CHECK(clamped.end == Approx(1.0F));

    const TimingColouriseActivationRange partial{
        .start = 0.25F,
        .end = 0.75F,
    };
    CHECK_FALSE(
        invisible_places::timing::TimingColouriseActivationRangeContains(
            partial,
            0.249F));
    CHECK(
        invisible_places::timing::TimingColouriseActivationRangeContains(
            partial,
            0.25F));
    CHECK(
        invisible_places::timing::TimingColouriseActivationRangeContains(
            partial,
            0.75F));
    CHECK_FALSE(
        invisible_places::timing::TimingColouriseActivationRangeContains(
            partial,
            0.751F));
    CHECK_FALSE(
        invisible_places::timing::TimingColouriseActivationRangeContains(
            partial,
            std::numeric_limits<float>::quiet_NaN()));

    effect.activationRange = {.start = 0.5F, .end = 0.5F};
    CHECK(invisible_places::timing::TimingColouriseEffectIsActiveAt(
        effect,
        0.5F));
    CHECK_FALSE(invisible_places::timing::TimingColouriseEffectIsActiveAt(
        effect,
        0.5001F));
    effect.enabled = false;
    CHECK_FALSE(invisible_places::timing::TimingColouriseEffectIsActiveAt(
        effect,
        0.5F));
}

TEST_CASE("Timing Colourise cyclic evaluation interpolates through loop zero",
          "[timing][colourise][cyclic]") {
    using invisible_places::timing::TimingColouriseEffectParameter;
    using invisible_places::water::WaterScenarioInterpolation;

    TimingColouriseEffect effect;
    effect.emissiveEnabled = true;
    effect.emissiveLevel = 0.0F;
    effect.effectParameterKeys = {
        {.parameter = TimingColouriseEffectParameter::EmissiveLevel,
         .position = 0.25F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::Smooth},
        {.parameter = TimingColouriseEffectParameter::EmissiveLevel,
         .position = 0.75F,
         .value = 1.0F,
         .interpolation = WaterScenarioInterpolation::Smooth},
    };
    CHECK(invisible_places::timing::EvaluateTimingEmissiveLevel(
              effect,
              0.0F) == Approx(0.0F));
    CHECK(invisible_places::timing::EvaluateTimingEmissiveLevel(
              effect,
              0.0F,
              true) == Approx(0.5F));
    CHECK(invisible_places::timing::EvaluateTimingEmissiveLevel(
              effect,
              1.0F,
              true) == Approx(0.5F));
    CHECK(invisible_places::timing::EvaluateTimingEmissiveLevel(
              effect,
              -0.1F,
              true) ==
          Approx(invisible_places::timing::EvaluateTimingEmissiveLevel(
              effect,
              0.9F,
              true)));
}

TEST_CASE(
    "Timing Colourise activation boundaries derive from every authored key",
    "[timing][colourise][activation][keys]") {
    TimingColouriseEffect effect;
    effect.activationRange = {.start = 0.3F, .end = 0.7F};
    effect.basePalette = TimingColourisePalette{
        .stops = {{.id = "tracked", .position = 0.5F}},
    };
    effect.effectParameterKeys = {
        {.parameter = TimingColouriseEffectParameter::AmountOverride,
         .position = 0.1F,
         .value = 0.0F},
        {.parameter = TimingColouriseEffectParameter::AmountOverride,
         .position = 0.9F,
         .value = 1.0F},
    };
    effect.paletteStopParameterKeys = {
        {.stopId = "tracked",
         .parameter =
             TimingColourisePaletteStopParameter::ColouriseAmount,
         .position = 0.1F,
         .scalarValue = 0.0F},
        {.stopId = "tracked",
         .parameter =
             TimingColourisePaletteStopParameter::ColouriseAmount,
         .position = 0.9F,
         .scalarValue = 1.0F},
    };
    effect.boundsParameterKeys = {
        {.parameter = TimingColouriseBoundsParameter::Lower,
         .position = 0.1F,
         .value = 0.0F},
        {.parameter = TimingColouriseBoundsParameter::Lower,
         .position = 0.9F,
         .value = 1.0F},
        {.parameter = TimingColouriseBoundsParameter::Upper,
         .position = 0.1F,
         .value = 2.0F},
        {.parameter = TimingColouriseBoundsParameter::Upper,
         .position = 0.9F,
         .value = 4.0F},
    };

    const auto sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    REQUIRE(sanitized.effectParameterKeys.size() == 2U);
    REQUIRE(sanitized.paletteStopParameterKeys.size() == 2U);
    REQUIRE(sanitized.boundsParameterKeys.size() == 4U);
    CHECK(sanitized.effectParameterKeys.front().position == Approx(0.1F));
    CHECK(sanitized.effectParameterKeys.back().position == Approx(0.9F));

    constexpr float kStartSmoothAmount = 0.15625F;
    constexpr float kEndSmoothAmount = 0.84375F;
    CHECK(invisible_places::timing::EvaluateTimingColouriseEffectParameter(
              sanitized,
              TimingColouriseEffectParameter::AmountOverride,
              sanitized.activationRange.start) ==
          Approx(kStartSmoothAmount));
    CHECK(invisible_places::timing::EvaluateTimingColouriseEffectParameter(
              sanitized,
              TimingColouriseEffectParameter::AmountOverride,
              sanitized.activationRange.end) ==
          Approx(kEndSmoothAmount));

    const auto startPalette =
        invisible_places::timing::EvaluateTimingColourisePalette(
            sanitized,
            sanitized.activationRange.start);
    const auto endPalette =
        invisible_places::timing::EvaluateTimingColourisePalette(
            sanitized,
            sanitized.activationRange.end);
    REQUIRE(startPalette.stops.size() == 1U);
    REQUIRE(endPalette.stops.size() == 1U);
    CHECK(startPalette.stops.front().colouriseAmount ==
          Approx(kStartSmoothAmount));
    CHECK(endPalette.stops.front().colouriseAmount ==
          Approx(kEndSmoothAmount));

    const auto startBounds =
        invisible_places::timing::EvaluateTimingColouriseBounds(
            sanitized,
            sanitized.activationRange.start);
    const auto endBounds =
        invisible_places::timing::EvaluateTimingColouriseBounds(
            sanitized,
            sanitized.activationRange.end);
    CHECK(startBounds.lower == Approx(kStartSmoothAmount));
    CHECK(startBounds.upper == Approx(2.0F + 2.0F * kStartSmoothAmount));
    CHECK(endBounds.lower == Approx(kEndSmoothAmount));
    CHECK(endBounds.upper == Approx(2.0F + 2.0F * kEndSmoothAmount));

    // Activation boundaries are derived display markers, not authored keys.
    CHECK(invisible_places::timing::TimingColouriseEffectKeyCountAtPosition(
              sanitized,
              sanitized.activationRange.start) == 0U);
    CHECK(invisible_places::timing::TimingColouriseEffectKeyCountAtPosition(
              sanitized,
              sanitized.activationRange.end) == 0U);

    TimingColouriseEffect legacyPaletteEffect;
    legacyPaletteEffect.activationRange = sanitized.activationRange;
    legacyPaletteEffect.paletteKeyModel =
        TimingColourisePaletteKeyModel::LegacySnapshots;
    legacyPaletteEffect.paletteKeys = {
        {.position = 0.1F, .palette = Solid({0.0F, 0.0F, 0.0F})},
        {.position = 0.9F, .palette = Solid({1.0F, 1.0F, 1.0F})},
    };
    const auto legacyStart =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            legacyPaletteEffect,
            legacyPaletteEffect.activationRange.start);
    const auto legacyEnd =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            legacyPaletteEffect,
            legacyPaletteEffect.activationRange.end);
    CHECK(legacyStart.front()[0] == Approx(kStartSmoothAmount));
    CHECK(legacyEnd.front()[0] == Approx(kEndSmoothAmount));
}

TEST_CASE(
    "Visual Feature aspects gate parameters and keep emissive scalar-only",
    "[timing][colourise][emissive][effect-parameters]") {
    using invisible_places::timing::TimingColouriseFieldSource;

    const TimingColouriseEffect defaultEffect;
    CHECK(defaultEffect.colouriseEnabled);
    CHECK_FALSE(defaultEffect.emissiveEnabled);
    CHECK(defaultEffect.emissiveLevel == Approx(1.0F));
    CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
        true,
        false,
        TimingColouriseEffectParameter::PalettePhase));
    CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
        true,
        false,
        TimingColouriseEffectParameter::AmountOverride));
    CHECK_FALSE(invisible_places::timing::TimingEffectParameterIsSupported(
        true,
        false,
        TimingColouriseEffectParameter::EmissiveLevel));
    CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
        false,
        true,
        TimingColouriseEffectParameter::EmissiveLevel));
    CHECK_FALSE(invisible_places::timing::TimingEffectParameterIsSupported(
        false,
        true,
        TimingColouriseEffectParameter::PalettePhase));
    CHECK_FALSE(invisible_places::timing::TimingEffectParameterIsSupported(
        false,
        true,
        TimingColouriseEffectParameter::AmountOverride));
    // A dual-aspect feature supports every effect parameter at once.
    CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
        true,
        true,
        TimingColouriseEffectParameter::PalettePhase));
    CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
        true,
        true,
        TimingColouriseEffectParameter::AmountOverride));
    CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
        true,
        true,
        TimingColouriseEffectParameter::EmissiveLevel));
    CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
        defaultEffect,
        TimingColouriseEffectParameter::PalettePhase));
    CHECK_FALSE(invisible_places::timing::TimingEffectParameterIsSupported(
        defaultEffect,
        TimingColouriseEffectParameter::EmissiveLevel));

    // Sanitize refuses a feature with no aspect at all.
    TimingColouriseEffect neither;
    neither.colouriseEnabled = false;
    neither.emissiveEnabled = false;
    CHECK(invisible_places::timing::SanitizeTimingColouriseEffect(neither)
              .colouriseEnabled);

    TimingColouriseEffect emissive;
    emissive.colouriseEnabled = false;
    emissive.emissiveEnabled = true;
    emissive.name.clear();
    emissive.field.source = TimingColouriseFieldSource::NormalX;
    emissive.field.scalarFieldName = "dormant-field";
    emissive.emissiveLevel = -4.0F;
    auto sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(emissive);
    CHECK(sanitized.name == "Visual Feature");
    CHECK(sanitized.field.source == TimingColouriseFieldSource::Scalar);
    CHECK(sanitized.field.scalarFieldName.empty());
    CHECK(sanitized.emissiveLevel == Approx(0.0F));

    // A dual-aspect feature legitimately follows colourise onto a normal
    // source; only its emissive aspect is dropped rather than the field.
    TimingColouriseEffect dual;
    dual.emissiveEnabled = true;
    dual.field.source = TimingColouriseFieldSource::NormalZ;
    const auto dualSanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(dual);
    CHECK(dualSanitized.colouriseEnabled);
    CHECK_FALSE(dualSanitized.emissiveEnabled);
    CHECK(dualSanitized.field.source == TimingColouriseFieldSource::NormalZ);

    emissive.field.source = TimingColouriseFieldSource::Scalar;
    emissive.field.scalarFieldName = "Heat";
    emissive.emissiveLevel = std::numeric_limits<float>::quiet_NaN();
    sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(emissive);
    CHECK(sanitized.field.scalarFieldName == "Heat");
    CHECK(sanitized.emissiveLevel == Approx(1.0F));

    emissive.emissiveLevel = 2.0F;
    emissive.activationRange = {.start = 0.3F, .end = 0.7F};
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &emissive,
                    TimingColouriseEffectParameter::EmissiveLevel,
                    0.1F,
                    0.0F,
                    WaterScenarioInterpolation::Hold));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &emissive,
                    TimingColouriseEffectParameter::EmissiveLevel,
                    0.9F,
                    4.0F,
                    WaterScenarioInterpolation::Linear));
    CHECK_FALSE(invisible_places::timing::
                    AddOrUpdateTimingColouriseEffectParameterKey(
                        &emissive,
                        TimingColouriseEffectParameter::PalettePhase,
                        0.5F,
                        1.0F));
    TimingColouriseEffect colourise;
    CHECK_FALSE(invisible_places::timing::
                    AddOrUpdateTimingColouriseEffectParameterKey(
                        &colourise,
                        TimingColouriseEffectParameter::EmissiveLevel,
                        0.5F,
                        1.0F));

    CHECK(invisible_places::timing::EvaluateTimingEmissiveLevel(
              emissive,
              emissive.activationRange.start) == Approx(0.625F));
    CHECK(invisible_places::timing::EvaluateTimingEmissiveLevel(
              emissive,
              emissive.activationRange.end) == Approx(3.375F));
    CHECK(invisible_places::timing::TimingColouriseEffectKeyCountAtPosition(
              emissive,
              emissive.activationRange.start) == 0U);
    CHECK(invisible_places::timing::TimingColouriseEffectKeyCountAtPosition(
              emissive,
              emissive.activationRange.end) == 0U);
}

TEST_CASE(
    "Emissive-only Visual Feature key operations preserve dormant colourise data",
    "[timing][colourise][emissive][keys]") {
    TimingColouriseEffect effect;
    effect.colouriseEnabled = false;
    effect.emissiveEnabled = true;
    effect.basePalette = TimingColourisePalette{
        .stops = {{.id = "tracked", .position = 0.5F}},
    };
    effect.paletteKeys = {
        {.position = 0.5F, .palette = Solid({0.2F, 0.4F, 0.8F})},
    };
    effect.paletteStopParameterKeys = {
        {.stopId = "tracked",
         .parameter = TimingColourisePaletteStopParameter::ColouriseAmount,
         .position = 0.5F,
         .scalarValue = 0.25F},
    };
    effect.effectParameterKeys = {
        {.parameter = TimingColouriseEffectParameter::AmountOverride,
         .position = 0.5F,
         .value = 0.25F},
        {.parameter = TimingColouriseEffectParameter::EmissiveLevel,
         .position = 0.5F,
         .value = 3.0F},
    };
    effect.boundsParameterKeys = {
        {.parameter = TimingColouriseBoundsParameter::Lower,
         .position = 0.5F,
         .value = 0.2F},
    };
    effect =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);

    REQUIRE(effect.paletteKeys.size() == 1U);
    REQUIRE(effect.paletteStopParameterKeys.size() == 1U);
    REQUIRE(effect.effectParameterKeys.size() == 2U);
    CHECK(invisible_places::timing::TimingColourisePaletteKeyCountAtPosition(
              effect,
              0.5F) == 2U);
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterUnionKeyCountAtPosition(
                  effect,
                  0.5F) == 1U);
    CHECK(invisible_places::timing::TimingColouriseEffectKeyCountAtPosition(
              effect,
              0.5F) == 2U);
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterKeyCountAtPosition(
                  effect,
                  TimingColouriseEffectParameter::AmountOverride,
                  0.5F) == 0U);
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterKeyPositions(effect) ==
          std::vector<float>{0.5F});
    CHECK(invisible_places::timing::PreviousTimingColouriseEffectKeyPosition(
              effect,
              0.6F) == std::optional<float>{0.5F});
    CHECK(invisible_places::timing::NextTimingColouriseEffectKeyPosition(
              effect,
              0.4F) == std::optional<float>{0.5F});

    REQUIRE(invisible_places::timing::
                MoveTimingColouriseEffectParameterKeys(
                    &effect,
                    0.5F,
                    0.6F));
    CHECK(effect.effectParameterKeys[0].position == Approx(0.5F));
    CHECK(effect.effectParameterKeys[1].position == Approx(0.6F));
    REQUIRE(invisible_places::timing::
                MoveTimingColouriseEffectParameterKeys(
                    &effect,
                    0.6F,
                    0.5F));
    CHECK_FALSE(invisible_places::timing::
                    MoveTimingColouriseEffectParameterKey(
                        &effect,
                        TimingColouriseEffectParameter::AmountOverride,
                        0.5F,
                        0.6F));

    CHECK(invisible_places::timing::RemoveTimingColouriseEffectKeysAtPosition(
              &effect,
              0.5F) == 2U);
    REQUIRE(effect.effectParameterKeys.size() == 1U);
    CHECK(effect.effectParameterKeys.front().parameter ==
          TimingColouriseEffectParameter::AmountOverride);
    CHECK(effect.paletteKeys.size() == 1U);
    CHECK(effect.paletteStopParameterKeys.size() == 1U);
    CHECK(effect.boundsParameterKeys.empty());
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
    "Timing Colourise reverses palettes without exchanging stop properties",
    "[timing][colourise][palette][reverse]") {
    const TimingColourisePalette palette{
        .stops = {
            {.id = "warm",
             .position = 0.1F,
             .colour = {0.9F, 0.2F, 0.1F},
             .colouriseAmount = 0.25F},
            {.id = "middle",
             .position = 0.5F,
             .colour = {0.4F, 0.5F, 0.6F},
             .colouriseAmount = 0.5F},
            {.id = "cool",
             .position = 0.8F,
             .colour = {0.1F, 0.3F, 0.95F},
             .colouriseAmount = 0.85F},
        },
    };

    const auto reversed =
        invisible_places::timing::ReverseTimingColourisePalette(
            palette);
    REQUIRE(reversed.stops.size() == 3U);
    CHECK(reversed.stops[0].id == "cool");
    CHECK(reversed.stops[0].position == Approx(0.2F));
    CHECK(reversed.stops[0].colour ==
          std::array<float, 3>{0.1F, 0.3F, 0.95F});
    CHECK(reversed.stops[0].colouriseAmount == Approx(0.85F));
    CHECK(reversed.stops[1].id == "middle");
    CHECK(reversed.stops[1].position == Approx(0.5F));
    CHECK(reversed.stops[2].id == "warm");
    CHECK(reversed.stops[2].position == Approx(0.9F));
    CHECK(reversed.stops[2].colour ==
          std::array<float, 3>{0.9F, 0.2F, 0.1F});
    CHECK(reversed.stops[2].colouriseAmount == Approx(0.25F));

    const auto restored =
        invisible_places::timing::ReverseTimingColourisePalette(
            reversed);
    REQUIRE(restored.stops.size() == palette.stops.size());
    for (std::size_t index = 0U; index < restored.stops.size(); ++index) {
        CHECK(restored.stops[index].id == palette.stops[index].id);
        CHECK(restored.stops[index].position ==
              Approx(palette.stops[index].position));
        CHECK(restored.stops[index].colour ==
              palette.stops[index].colour);
        CHECK(restored.stops[index].colouriseAmount ==
              Approx(palette.stops[index].colouriseAmount));
    }

    const auto originalLut =
        invisible_places::timing::CompileTimingColourisePaletteLut(
            palette);
    const auto reversedLut =
        invisible_places::timing::CompileTimingColourisePaletteLut(
            reversed);
    for (std::size_t index = 0U; index < originalLut.size(); ++index) {
        const auto mirroredIndex = originalLut.size() - 1U - index;
        for (std::size_t channel = 0U; channel < 4U; ++channel) {
            CHECK(reversedLut[index][channel] ==
                  Approx(originalLut[mirroredIndex][channel]));
        }
    }

    const TimingColourisePalette duplicatePositions{
        .stops = {
            {.id = "first-at-break",
             .position = 0.25F,
             .colour = {1.0F, 0.0F, 0.0F}},
            {.id = "second-at-break",
             .position = 0.25F,
             .colour = {0.0F, 1.0F, 0.0F}},
            {.id = "far-edge",
             .position = 0.75F,
             .colour = {0.0F, 0.0F, 1.0F}},
        },
    };
    const auto reversedDuplicates =
        invisible_places::timing::ReverseTimingColourisePalette(
            duplicatePositions);
    REQUIRE(reversedDuplicates.stops.size() == 3U);
    CHECK(reversedDuplicates.stops[0].id == "far-edge");
    CHECK(reversedDuplicates.stops[0].position == Approx(0.25F));
    // Equal-position authored order must reverse along with the axis or the
    // discontinuity would face the original direction.
    CHECK(reversedDuplicates.stops[1].id == "second-at-break");
    CHECK(reversedDuplicates.stops[2].id == "first-at-break");
    CHECK(reversedDuplicates.stops[1].position == Approx(0.75F));
    CHECK(reversedDuplicates.stops[2].position == Approx(0.75F));
}

TEST_CASE(
    "Timing Colourise reverses an unkeyed preset without marking it edited",
    "[timing][colourise][palette][reverse][base]") {
    TimingColouriseEffect effect;
    effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
    effect.paletteSourceId = "seaborn-mako";
    effect.paletteSourceName = "Mako";
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {
                {.id = "dark",
                 .position = 0.1F,
                 .colour = {0.05F, 0.1F, 0.2F},
                 .colouriseAmount = 0.4F},
                {.id = "light",
                 .position = 0.9F,
                 .colour = {0.8F, 0.9F, 1.0F},
                 .colouriseAmount = 0.7F},
            },
        });

    CHECK_FALSE(
        invisible_places::timing::CanReverseTimingColourisePaletteAtPosition(
            effect,
            std::numeric_limits<float>::quiet_NaN()));
    CHECK_FALSE(
        invisible_places::timing::CanReverseTimingColourisePaletteAtPosition(
            effect,
            -0.01F));
    CHECK_FALSE(
        invisible_places::timing::ReverseTimingColourisePaletteAtPosition(
            &effect,
            1.01F));
    CHECK(
        invisible_places::timing::CanReverseTimingColourisePaletteAtPosition(
            effect,
            0.4F));
    REQUIRE(
        invisible_places::timing::ReverseTimingColourisePaletteAtPosition(
            &effect,
            0.4F));

    CHECK(effect.paletteSourceKind ==
          TimingColourisePaletteSourceKind::Preset);
    CHECK(effect.paletteSourceId == "seaborn-mako");
    CHECK_FALSE(effect.paletteEdited);
    CHECK(effect.localPaletteEdits.empty());
    CHECK(effect.paletteKeys.empty());
    CHECK(effect.paletteStopParameterKeys.empty());
    REQUIRE(effect.basePalette.stops.size() == 2U);
    CHECK(effect.basePalette.stops[0].id == "light");
    CHECK(effect.basePalette.stops[0].position == Approx(0.1F));
    CHECK(effect.basePalette.stops[0].colouriseAmount == Approx(0.7F));
    CHECK(effect.basePalette.stops[1].id == "dark");
    CHECK(effect.basePalette.stops[1].position == Approx(0.9F));
}

TEST_CASE(
    "Timing Colourise flipping an active local preset edit keeps its private snapshot",
    "[timing][colourise][palette][reverse][local-edit]") {
    const TimingColourisePaletteDefinition preset{
        .id = "seaborn-mako",
        .name = "Mako",
        .palette = {
            .stops = {
                {.id = "dark",
                 .position = 0.1F,
                 .colour = {0.05F, 0.1F, 0.2F},
                 .colouriseAmount = 0.4F},
                {.id = "light",
                 .position = 0.8F,
                 .colour = {0.8F, 0.9F, 1.0F},
                 .colouriseAmount = 0.75F},
            },
        },
    };
    TimingColouriseEffect effect;
    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseOriginalPreset(
                    &effect,
                    preset));
    auto edited = preset.palette;
    edited.stops.front().colouriseAmount = 0.2F;
    REQUIRE(invisible_places::timing::
                UpsertTimingColouriseLocalPaletteEdit(
                    &effect,
                    edited));
    REQUIRE(effect.paletteEdited);

    REQUIRE(
        invisible_places::timing::ReverseTimingColourisePaletteAtPosition(
            &effect,
            0.5F));
    CHECK(effect.paletteEdited);
    const auto* flippedEdit = invisible_places::timing::
        FindTimingColouriseLocalPaletteEdit(effect, preset.id);
    REQUIRE(flippedEdit != nullptr);
    REQUIRE(flippedEdit->palette.stops.size() == 2U);
    CHECK(flippedEdit->palette.stops[0].id == "light");
    CHECK(flippedEdit->palette.stops[0].position == Approx(0.2F));
    CHECK(flippedEdit->palette.stops[1].id == "dark");
    CHECK(flippedEdit->palette.stops[1].position == Approx(0.9F));
    CHECK(flippedEdit->palette.stops[1].colouriseAmount == Approx(0.2F));

    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseOriginalPreset(
                    &effect,
                    preset));
    CHECK_FALSE(effect.paletteEdited);
    CHECK(effect.basePalette.stops.front().id == "dark");
    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseLocalPaletteEdit(
                    &effect,
                    preset.id));
    CHECK(effect.paletteEdited);
    CHECK(effect.basePalette.stops.front().id == "light");
    CHECK(effect.basePalette.stops.front().position == Approx(0.2F));
}

TEST_CASE(
    "Timing Colourise keyed reversal authors only stop positions",
    "[timing][colourise][palette][reverse][parameters]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {
                {.id = "warm",
                 .position = 0.1F,
                 .colour = {1.0F, 0.2F, 0.1F},
                 .colouriseAmount = 0.3F},
                {.id = "cool",
                 .position = 0.8F,
                 .colour = {0.1F, 0.2F, 1.0F},
                 .colouriseAmount = 0.9F},
            },
        });
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "warm",
                TimingColourisePaletteStopParameter::Position,
                0.4F,
                0.25F,
                WaterScenarioInterpolation::Hold));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "warm",
                0.2F,
                {0.6F, 0.4F, 0.2F},
                WaterScenarioInterpolation::Linear));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "cool",
                TimingColourisePaletteStopParameter::ColouriseAmount,
                0.6F,
                0.55F,
                WaterScenarioInterpolation::Linear));

    const auto propertyKeyCount = effect.paletteStopParameterKeys.size();
    REQUIRE(
        invisible_places::timing::ReverseTimingColourisePaletteAtPosition(
            &effect,
            0.4F));
    CHECK_FALSE(effect.paletteEdited);
    CHECK(effect.paletteStopParameterKeys.size() == propertyKeyCount + 1U);
    CHECK(
        invisible_places::timing::
            TimingColourisePaletteStopParameterKeyCountAtPosition(
                effect,
                "warm",
                TimingColourisePaletteStopParameter::Colour,
                0.2F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColourisePaletteStopParameterKeyCountAtPosition(
                effect,
                "cool",
                TimingColourisePaletteStopParameter::ColouriseAmount,
                0.6F) == 1U);

    const auto warmPositionKey = std::find_if(
        effect.paletteStopParameterKeys.begin(),
        effect.paletteStopParameterKeys.end(),
        [](const auto& key) {
            return key.stopId == "warm" &&
                   key.parameter ==
                       TimingColourisePaletteStopParameter::Position;
        });
    const auto coolPositionKey = std::find_if(
        effect.paletteStopParameterKeys.begin(),
        effect.paletteStopParameterKeys.end(),
        [](const auto& key) {
            return key.stopId == "cool" &&
                   key.parameter ==
                       TimingColourisePaletteStopParameter::Position;
        });
    REQUIRE(warmPositionKey != effect.paletteStopParameterKeys.end());
    REQUIRE(coolPositionKey != effect.paletteStopParameterKeys.end());
    CHECK(warmPositionKey->scalarValue == Approx(0.75F));
    CHECK(warmPositionKey->interpolation ==
          WaterScenarioInterpolation::Smooth);
    CHECK(coolPositionKey->scalarValue == Approx(0.2F));
    CHECK(coolPositionKey->interpolation ==
          WaterScenarioInterpolation::Smooth);

    const auto positionsById = [](const TimingColourisePalette& palette,
                                  std::string_view id) {
        const auto stop = std::find_if(
            palette.stops.begin(),
            palette.stops.end(),
            [&](const auto& candidate) { return candidate.id == id; });
        REQUIRE(stop != palette.stops.end());
        return stop->position;
    };
    // A single position key holds on either side of its authored time.
    const auto before =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.0F);
    const auto after =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            1.0F);
    CHECK(positionsById(before, "warm") == Approx(0.75F));
    CHECK(positionsById(after, "warm") == Approx(0.75F));
    CHECK(positionsById(before, "cool") == Approx(0.2F));
    CHECK(positionsById(after, "cool") == Approx(0.2F));

    // A second flip at the same animation position updates those position
    // keys in place, while the independent colour and amount keys survive.
    REQUIRE(
        invisible_places::timing::ReverseTimingColourisePaletteAtPosition(
            &effect,
            0.4F));
    CHECK(effect.paletteStopParameterKeys.size() == propertyKeyCount + 1U);
    const auto restored =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.4F);
    CHECK(positionsById(restored, "warm") == Approx(0.25F));
    CHECK(positionsById(restored, "cool") == Approx(0.8F));
}

TEST_CASE(
    "Timing Colourise legacy reversal requires an exact snapshot key",
    "[timing][colourise][palette][reverse][legacy]") {
    const auto gradient = [](std::string firstId,
                             std::string secondId,
                             std::array<float, 3> firstColour,
                             std::array<float, 3> secondColour) {
        return invisible_places::timing::
            SanitizeTimingColourisePalette({
                .stops = {
                    {.id = std::move(firstId),
                     .position = 0.0F,
                     .colour = firstColour},
                    {.id = std::move(secondId),
                     .position = 1.0F,
                     .colour = secondColour},
                },
            });
    };
    TimingColouriseEffect effect;
    effect.paletteKeyModel = TimingColourisePaletteKeyModel::LegacySnapshots;
    effect.basePalette = gradient(
        "base-left",
        "base-right",
        {0.1F, 0.2F, 0.3F},
        {0.8F, 0.9F, 1.0F});

    SECTION("base palettes remain editable until a snapshot exists") {
        REQUIRE(
            invisible_places::timing::
                ReverseTimingColourisePaletteAtPosition(
                    &effect,
                    0.5F));
        CHECK(effect.paletteEdited);
        CHECK(effect.basePalette.stops.front().id == "base-right");
        CHECK(effect.paletteKeys.empty());
    }

    SECTION("unkeyed presets remain presets without an edited state") {
        effect.paletteSourceKind =
            TimingColourisePaletteSourceKind::Preset;
        effect.paletteSourceId = "legacy-preset";
        effect.paletteSourceName = "Legacy Preset";
        REQUIRE(
            invisible_places::timing::
                ReverseTimingColourisePaletteAtPosition(
                    &effect,
                    0.5F));
        CHECK(effect.paletteSourceKind ==
              TimingColourisePaletteSourceKind::Preset);
        CHECK(effect.paletteSourceId == "legacy-preset");
        CHECK_FALSE(effect.paletteEdited);
        CHECK(effect.basePalette.stops.front().id == "base-right");
        CHECK(effect.paletteKeys.empty());
    }

    SECTION("only the exact legacy snapshot is reversed") {
        invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
            &effect,
            0.2F,
            gradient(
                "first-left",
                "first-right",
                {1.0F, 0.0F, 0.0F},
                {0.0F, 1.0F, 0.0F}),
            WaterScenarioInterpolation::Linear);
        invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
            &effect,
            0.8F,
            gradient(
                "second-left",
                "second-right",
                {0.0F, 0.0F, 1.0F},
                {1.0F, 1.0F, 0.0F}),
            WaterScenarioInterpolation::Hold);
        const auto baseBefore = effect.basePalette;
        const auto secondBefore = effect.paletteKeys[1].palette;

        CHECK_FALSE(
            invisible_places::timing::
                CanReverseTimingColourisePaletteAtPosition(
                    effect,
                    0.5F));
        CHECK_FALSE(
            invisible_places::timing::
                ReverseTimingColourisePaletteAtPosition(
                    &effect,
                    0.5F));
        CHECK(effect.paletteKeys[0].palette.stops.front().id ==
              "first-left");

        CHECK(
            invisible_places::timing::
                CanReverseTimingColourisePaletteAtPosition(
                    effect,
                    0.20005F));
        REQUIRE(
            invisible_places::timing::
                ReverseTimingColourisePaletteAtPosition(
                    &effect,
                    0.20005F));
        CHECK(effect.paletteKeys[0].palette.stops.front().id ==
              "first-right");
        CHECK(effect.paletteKeys[0].interpolation ==
              WaterScenarioInterpolation::Smooth);
        CHECK(effect.paletteKeys[1].palette.stops.front().id ==
              secondBefore.stops.front().id);
        CHECK(effect.basePalette.stops.front().id ==
              baseBefore.stops.front().id);
        CHECK_FALSE(effect.paletteEdited);
    }
}

TEST_CASE(
    "Timing Colourise independently evaluates stop position colour and amount",
    "[timing][colourise][palette][parameters]") {
    const auto smooth = [](float amount) {
        return amount * amount * (3.0F - 2.0F * amount);
    };
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
    CHECK(
        atQuarter.stops[0].position ==
        Approx(std::lerp(0.0F, 0.3F, smooth(0.75F))));
    CHECK(
        atQuarter.stops[0].colour[0] ==
        Approx(std::lerp(
            firstColour[0],
            secondColour[0],
            smooth(1.0F / 6.0F))));
    CHECK(atQuarter.stops[0].colouriseAmount == Approx(0.65F));

    const auto atFourTenths =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.4F);
    REQUIRE(atFourTenths.stops.size() == 1U);
    CHECK(atFourTenths.stops[0].position == Approx(0.15F));
    CHECK(
        atFourTenths.stops[0].colour[1] ==
        Approx(std::lerp(
            firstColour[1],
            secondColour[1],
            smooth(2.0F / 3.0F))));

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
    "Timing Colourise palette unions transitive tolerance chains",
    "[timing][colourise][palette][parameters][navigation][tolerance]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {
                {.id = "stop-a", .position = 0.0F},
                {.id = "stop-b", .position = 1.0F},
            },
        });
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::Position,
                0.0F,
                0.1F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopColourKey(
                &effect,
                "stop-a",
                0.00009F,
                {1.0F, 0.0F, 0.0F}));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::ColouriseAmount,
                0.00018F,
                0.5F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColourisePaletteStopScalarKey(
                &effect,
                "stop-a",
                TimingColourisePaletteStopParameter::Position,
                0.4F,
                0.8F));

    const auto positions =
        invisible_places::timing::TimingColourisePaletteKeyPositions(
            effect);
    REQUIRE(positions.size() == 2U);
    CHECK(positions[0] == Approx(0.0F));
    CHECK(positions[1] == Approx(0.4F));
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyCountAtPosition(
            effect,
            0.0F) == 3U);
    CHECK(
        invisible_places::timing::TimingColourisePaletteKeyCountAtPosition(
            effect,
            0.00018F) == 3U);
    CHECK_FALSE(
        invisible_places::timing::
            PreviousTimingColourisePaletteKeyPosition(
                effect,
                0.00018F)
                .has_value());
    CHECK(
        invisible_places::timing::NextTimingColourisePaletteKeyPosition(
            effect,
            0.00018F) == Approx(0.4F));

    SECTION("moving the marker moves every property in its cluster") {
        REQUIRE(
            invisible_places::timing::MoveTimingColourisePaletteKey(
                &effect,
                0.0F,
                0.25F));
        CHECK(
            invisible_places::timing::TimingColourisePaletteKeyPositions(
                effect) == std::vector<float>{0.25F, 0.4F});
        CHECK(
            invisible_places::timing::
                TimingColourisePaletteKeyCountAtPosition(
                    effect,
                    0.25F) == 3U);
        CHECK(
            invisible_places::timing::
                TimingColourisePaletteKeyCountAtPosition(
                    effect,
                    0.0F) == 0U);
    }

    SECTION("deleting the marker deletes every property in its cluster") {
        CHECK(
            invisible_places::timing::
                RemoveTimingColourisePaletteKeysAtPosition(
                    &effect,
                    0.0F) == 3U);
        CHECK(
            invisible_places::timing::TimingColourisePaletteKeyPositions(
                effect) == std::vector<float>{0.4F});
    }

    SECTION("moving detects a same-track collision at a cluster tail") {
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColourisePaletteStopScalarKey(
                    &effect,
                    "stop-b",
                    TimingColourisePaletteStopParameter::Position,
                    0.5F,
                    0.2F));
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColourisePaletteStopColourKey(
                    &effect,
                    "stop-b",
                    0.50009F,
                    {0.0F, 1.0F, 0.0F}));
        REQUIRE(
            invisible_places::timing::
                AddOrUpdateTimingColourisePaletteStopScalarKey(
                    &effect,
                    "stop-a",
                    TimingColourisePaletteStopParameter::Position,
                    0.50018F,
                    0.9F));
        CHECK_FALSE(
            invisible_places::timing::
                CanMoveTimingColourisePaletteKeysAtPosition(
                    effect,
                    0.0F,
                    0.5F));
        CHECK_FALSE(
            invisible_places::timing::MoveTimingColourisePaletteKey(
                &effect,
                0.0F,
                0.5F));
        CHECK(
            invisible_places::timing::
                TimingColourisePaletteKeyCountAtPosition(
                    effect,
                    0.0F) == 3U);
    }
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
    "Timing Colourise keeps independent effect-local edits while switching presets",
    "[timing][colourise][palette][source][local-edit]") {
    const TimingColourisePaletteDefinition mako{
        .id = "seaborn-mako",
        .name = "Mako",
        .palette = Solid({0.1F, 0.2F, 0.3F}, 1.0F),
    };
    const TimingColourisePaletteDefinition viridis{
        .id = "matplotlib-viridis",
        .name = "Viridis",
        .palette = Solid({0.2F, 0.7F, 0.4F}, 1.0F),
    };
    auto makoEdit = mako.palette;
    makoEdit.stops.front().colour = {0.8F, 0.1F, 0.2F};
    makoEdit.stops.front().colouriseAmount = 0.35F;
    auto viridisEdit = viridis.palette;
    viridisEdit.stops.front().colour = {0.4F, 0.2F, 0.9F};
    viridisEdit.stops.front().colouriseAmount = 0.6F;

    TimingColouriseEffect effect;
    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseOriginalPreset(
                    &effect,
                    mako));
    REQUIRE(invisible_places::timing::
                UpsertTimingColouriseLocalPaletteEdit(
                    &effect,
                    makoEdit));
    CHECK(effect.paletteEdited);
    REQUIRE(effect.localPaletteEdits.size() == 1U);
    // Active authoring may mutate the base snapshot in place. Switching
    // versions must synchronize that latest value into the private entry.
    effect.basePalette.stops.front().colouriseAmount = 0.42F;

    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseOriginalPreset(
                    &effect,
                    mako));
    CHECK_FALSE(effect.paletteEdited);
    CHECK(effect.basePalette.stops.front().colour[0] == Approx(0.1F));
    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseLocalPaletteEdit(
                    &effect,
                    mako.id));
    CHECK(effect.paletteEdited);
    CHECK(effect.basePalette.stops.front().colour[0] == Approx(0.8F));
    CHECK(effect.basePalette.stops.front().colouriseAmount ==
          Approx(0.42F));

    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseOriginalPreset(
                    &effect,
                    viridis));
    REQUIRE(invisible_places::timing::
                UpsertTimingColouriseLocalPaletteEdit(
                    &effect,
                    viridisEdit));
    REQUIRE(effect.localPaletteEdits.size() == 2U);
    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseLocalPaletteEdit(
                    &effect,
                    mako.id));
    CHECK(effect.paletteSourceId == mako.id);
    CHECK(effect.basePalette.stops.front().colouriseAmount ==
          Approx(0.42F));
    const auto* retainedViridis = invisible_places::timing::
        FindTimingColouriseLocalPaletteEdit(effect, viridis.id);
    REQUIRE(retainedViridis != nullptr);
    CHECK(retainedViridis->palette.stops.front().colouriseAmount ==
          Approx(0.6F));

    REQUIRE(invisible_places::timing::
                DiscardTimingColouriseLocalPaletteEdit(
                    &effect,
                    mako));
    CHECK_FALSE(effect.paletteEdited);
    CHECK(effect.paletteSourceId == mako.id);
    CHECK(effect.basePalette.stops.front().colour[0] == Approx(0.1F));
    CHECK(invisible_places::timing::FindTimingColouriseLocalPaletteEdit(
              effect,
              mako.id) == nullptr);
    CHECK(invisible_places::timing::FindTimingColouriseLocalPaletteEdit(
              effect,
              viridis.id) != nullptr);
}

TEST_CASE(
    "Timing Colourise promotes one local edit without losing another",
    "[timing][colourise][palette][source][local-edit][promote]") {
    TimingColouriseEffect effect;
    effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
    effect.paletteSourceId = "preset-a";
    effect.paletteSourceName = "Preset A";
    REQUIRE(invisible_places::timing::
                UpsertTimingColouriseLocalPaletteEdit(
                    &effect,
                    Solid({0.8F, 0.2F, 0.1F}, 0.4F)));
    REQUIRE(invisible_places::timing::
                ActivateTimingColouriseOriginalPreset(
                    &effect,
                    TimingColourisePaletteDefinition{
                        .id = "preset-b",
                        .name = "Preset B",
                        .palette = Solid({0.1F, 0.2F, 0.8F}),
                    }));
    REQUIRE(invisible_places::timing::
                UpsertTimingColouriseLocalPaletteEdit(
                    &effect,
                    Solid({0.3F, 0.9F, 0.4F}, 0.7F)));

    const auto promoted = invisible_places::timing::
        PromoteTimingColouriseLocalPaletteEdit(
            &effect,
            "preset-a",
            "colourise-palette-12",
            "Warm Study");
    REQUIRE(promoted.has_value());
    CHECK(promoted->id == "colourise-palette-12");
    CHECK(promoted->name == "Warm Study");
    CHECK(promoted->palette.stops.front().colouriseAmount ==
          Approx(0.4F));
    CHECK(effect.paletteSourceKind ==
          TimingColourisePaletteSourceKind::Saved);
    CHECK(effect.paletteSourceId == "colourise-palette-12");
    CHECK(effect.paletteSourceName == "Warm Study");
    CHECK_FALSE(effect.paletteEdited);
    CHECK(effect.basePalette.stops.front().colouriseAmount ==
          Approx(0.4F));
    CHECK(invisible_places::timing::FindTimingColouriseLocalPaletteEdit(
              effect,
              "preset-a") == nullptr);
    CHECK(invisible_places::timing::FindTimingColouriseLocalPaletteEdit(
              effect,
              "preset-b") != nullptr);
}

TEST_CASE(
    "Timing Colourise sanitizes and migrates effect-local preset edits",
    "[timing][colourise][palette][source][local-edit][sanitize]") {
    SECTION("duplicates coalesce by preset id and retain private stop keys") {
        TimingColouriseEffect effect;
        effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
        effect.paletteSourceId = "original";
        effect.basePalette = TimingColourisePalette{
            .stops = {{.id = "original-stop", .position = 0.0F}},
        };
        effect.localPaletteEdits = {
            TimingColouriseLocalPaletteEdit{
                .presetId = "",
                .presetName = "Invalid",
                .palette = Solid({1.0F, 0.0F, 0.0F}),
            },
            TimingColouriseLocalPaletteEdit{
                .presetId = "duplicate",
                .presetName = "First",
                .palette = Solid({0.1F, 0.1F, 0.1F}, 0.2F),
            },
            TimingColouriseLocalPaletteEdit{
                .presetId = "duplicate",
                .presetName = "Latest",
                .palette = TimingColourisePalette{
                    .stops = {{.id = "private-stop",
                               .position = 2.0F,
                               .colouriseAmount = 2.0F}},
                },
            },
        };
        effect.paletteStopParameterKeys.push_back({
            .stopId = "private-stop",
            .parameter = TimingColourisePaletteStopParameter::Position,
            .position = 0.4F,
            .scalarValue = 0.5F,
        });
        const auto sanitized =
            invisible_places::timing::SanitizeTimingColouriseEffect(
                effect);
        REQUIRE(sanitized.localPaletteEdits.size() == 1U);
        CHECK(sanitized.localPaletteEdits.front().presetId ==
              "duplicate");
        CHECK(sanitized.localPaletteEdits.front().presetName ==
              "Latest");
        CHECK(sanitized.localPaletteEdits.front()
                  .palette.stops.front().position == Approx(1.0F));
        CHECK(sanitized.localPaletteEdits.front()
                  .palette.stops.front().colouriseAmount == Approx(1.0F));
        REQUIRE(sanitized.paletteStopParameterKeys.size() == 1U);
        CHECK(sanitized.paletteStopParameterKeys.front().stopId ==
              "private-stop");
    }

    SECTION("legacy active preset edits synthesize when provenance is safe") {
        TimingColouriseEffect effect;
        effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
        effect.paletteSourceId = "legacy-preset";
        effect.paletteSourceName.clear();
        effect.paletteEdited = true;
        effect.basePalette = Solid({0.7F, 0.2F, 0.4F}, 0.45F);
        const auto sanitized =
            invisible_places::timing::SanitizeTimingColouriseEffect(
                effect);
        REQUIRE(sanitized.localPaletteEdits.size() == 1U);
        CHECK(sanitized.localPaletteEdits.front().presetId ==
              "legacy-preset");
        CHECK(sanitized.localPaletteEdits.front().presetName ==
              "legacy-preset");
        CHECK(sanitized.paletteSourceName == "legacy-preset");
        CHECK(sanitized.localPaletteEdits.front()
                  .palette.stops.front().colouriseAmount == Approx(0.45F));
    }

    SECTION("missing preset identity is not synthesized") {
        TimingColouriseEffect effect;
        effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
        effect.paletteEdited = true;
        const auto sanitized =
            invisible_places::timing::SanitizeTimingColouriseEffect(
                effect);
        CHECK(sanitized.localPaletteEdits.empty());
        CHECK(sanitized.paletteEdited);
    }
}

TEST_CASE(
    "Timing Colourise amount override caps or scales evaluated palette amounts",
    "[timing][colourise][palette][amount-override]") {
    TimingColouriseEffect effect;
    effect.basePalette = TimingColourisePalette{
        .stops = {
            {.position = 0.0F,
             .colour = {1.0F, 0.0F, 0.0F},
             .colouriseAmount = 0.0F},
            {.position = 0.5F,
             .colour = {0.0F, 1.0F, 0.0F},
             .colouriseAmount = 0.3F},
            {.position = 1.0F,
             .colour = {0.0F, 0.0F, 1.0F},
             .colouriseAmount = 0.5F},
        },
    };
    const auto raw =
        invisible_places::timing::CompileTimingColourisePaletteLut(
            effect.basePalette);

    effect.colouriseAmountOverrideMode =
        TimingColouriseAmountOverrideMode::Maximum;
    effect.colouriseAmountOverride = 0.5F;
    const auto maximumNeutral =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.0F);
    for (std::size_t index = 0U; index < raw.size(); ++index) {
        CHECK(maximumNeutral[index][0] == Approx(raw[index][0]));
        CHECK(maximumNeutral[index][1] == Approx(raw[index][1]));
        CHECK(maximumNeutral[index][2] == Approx(raw[index][2]));
        CHECK(maximumNeutral[index][3] == Approx(raw[index][3]));
    }

    effect.colouriseAmountOverride = 0.2F;
    const auto capped =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.0F);
    for (std::size_t index = 0U; index < raw.size(); ++index) {
        CHECK(capped[index][3] ==
              Approx(std::min(raw[index][3], 0.2F)));
    }

    effect.colouriseAmountOverrideMode =
        TimingColouriseAmountOverrideMode::Scale;
    effect.colouriseAmountOverride = 0.5F;
    const auto scaled =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.0F);
    for (std::size_t index = 0U; index < raw.size(); ++index) {
        CHECK(scaled[index][0] == Approx(raw[index][0]));
        CHECK(scaled[index][1] == Approx(raw[index][1]));
        CHECK(scaled[index][2] == Approx(raw[index][2]));
        CHECK(scaled[index][3] == Approx(raw[index][3] * 0.5F));
    }

    // The override is an effect-level evaluation control. It must not bake
    // itself into the authored or independently keyed stop amounts.
    const auto authored =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.0F);
    REQUIRE(authored.stops.size() == 3U);
    CHECK(authored.stops[0].colouriseAmount == Approx(0.0F));
    CHECK(authored.stops[1].colouriseAmount == Approx(0.3F));
    CHECK(authored.stops[2].colouriseAmount == Approx(0.5F));
}

TEST_CASE(
    "Timing Colourise amount override sanitizes invalid authoring data",
    "[timing][colourise][palette][amount-override]") {
    TimingColouriseEffect effect;
    effect.colouriseAmountOverrideMode =
        static_cast<TimingColouriseAmountOverrideMode>(255U);
    effect.colouriseAmountOverride =
        std::numeric_limits<float>::quiet_NaN();
    effect.palettePhaseOffset =
        std::numeric_limits<float>::quiet_NaN();
    effect.effectParameterKeys = {
        {.parameter =
             static_cast<TimingColouriseEffectParameter>(255U),
         .position = 0.5F,
         .value = 0.5F},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = -1.0F,
         .value = 2.0F},
    };
    auto sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    CHECK(sanitized.colouriseAmountOverrideMode ==
          TimingColouriseAmountOverrideMode::Maximum);
    CHECK(sanitized.colouriseAmountOverride == Approx(1.0F));
    CHECK(sanitized.palettePhaseOffset == Approx(0.0F));
    REQUIRE(sanitized.effectParameterKeys.size() == 1U);
    CHECK(sanitized.effectParameterKeys.front().position ==
          Approx(0.0F));
    CHECK(sanitized.effectParameterKeys.front().value ==
          Approx(1.0F));

    effect.colouriseAmountOverrideMode =
        TimingColouriseAmountOverrideMode::Scale;
    effect.colouriseAmountOverride = -0.5F;
    sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    CHECK(sanitized.colouriseAmountOverrideMode ==
          TimingColouriseAmountOverrideMode::Scale);
    CHECK(sanitized.colouriseAmountOverride == Approx(0.0F));

    effect.colouriseAmountOverride = 1.5F;
    sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    CHECK(sanitized.colouriseAmountOverride == Approx(1.0F));
}

TEST_CASE(
    "Timing Colourise phase shifts evaluated colours cyclically without moving stops",
    "[timing][colourise][palette][phase]") {
    TimingColouriseEffect effect;
    effect.basePalette = TimingColourisePalette{
        .stops = {
            {.id = "red",
             .position = 0.0F,
             .colour = {1.0F, 0.0F, 0.0F},
             .colouriseAmount = 0.2F},
            {.id = "green",
             .position = 0.5F,
             .colour = {0.0F, 1.0F, 0.0F},
             .colouriseAmount = 0.6F},
            {.id = "blue",
             .position = 1.0F,
             .colour = {0.0F, 0.0F, 1.0F},
             .colouriseAmount = 1.0F},
        },
    };
    const auto raw =
        invisible_places::timing::CompileTimingColourisePaletteLut(
            effect.basePalette);
    CHECK(
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            0.0F) == raw);
    CHECK(
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            1.0F) == raw);
    CHECK(
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            2.0F) == raw);

    const auto shifted =
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            0.25F);
    CHECK(
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            1.25F) == shifted);
    CHECK(
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            -0.75F) == shifted);
    CHECK(
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            -0.25F) ==
        invisible_places::timing::ApplyTimingColourisePalettePhase(
            raw,
            0.75F));
    const auto expectedAtSeam =
        invisible_places::timing::SampleTimingColouriseLut(
            raw,
            0.75F);
    CHECK(shifted.front()[0] == Approx(expectedAtSeam.colour[0]));
    CHECK(shifted.front()[1] == Approx(expectedAtSeam.colour[1]));
    CHECK(shifted.front()[2] == Approx(expectedAtSeam.colour[2]));
    CHECK(shifted.front()[3] ==
          Approx(expectedAtSeam.colouriseAmount));
    CHECK(shifted.back() == shifted.front());

    effect.palettePhaseOffset = 0.25F;
    const auto evaluated =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.0F);
    CHECK(evaluated.front() == shifted.front());
    const auto authored =
        invisible_places::timing::EvaluateTimingColourisePalette(
            effect,
            0.0F);
    REQUIRE(authored.stops.size() == 3U);
    CHECK(authored.stops[0].id == "red");
    CHECK(authored.stops[0].position == Approx(0.0F));
    CHECK(authored.stops[1].id == "green");
    CHECK(authored.stops[1].position == Approx(0.5F));
    CHECK(authored.stops[2].id == "blue");
    CHECK(authored.stops[2].position == Approx(1.0F));
}

TEST_CASE(
    "Timing Colourise phase and amount own independent animated control tracks",
    "[timing][colourise][palette][effect-parameters]") {
    TimingColouriseEffect effect;
    effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
    effect.paletteSourceId = "viridis";
    effect.basePalette = Solid({0.2F, 0.4F, 0.8F}, 1.0F);
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.2F,
                    0.0F,
                    WaterScenarioInterpolation::Linear));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.8F,
                    1.0F,
                    WaterScenarioInterpolation::Linear));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::AmountOverride,
                    0.2F,
                    0.2F,
                    WaterScenarioInterpolation::Linear));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::AmountOverride,
                    0.8F,
                    0.6F,
                    WaterScenarioInterpolation::Linear));

    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.5F) == Approx(0.5F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::AmountOverride,
                  0.5F) == Approx(0.4F));
    const auto evaluated =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.5F);
    CHECK(evaluated.front()[3] == Approx(0.4F));
    CHECK(evaluated.back()[3] == Approx(0.4F));

    CHECK(invisible_places::timing::
              TimingColourisePaletteKeyPositions(effect).empty());
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterKeyPositions(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase) ==
          std::vector<float>{0.2F, 0.8F});
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterKeyPositions(effect) ==
          std::vector<float>{0.2F, 0.8F});
    CHECK(invisible_places::timing::
              PreviousTimingColouriseEffectKeyPosition(
                  effect,
                  0.5F) == std::optional<float>{0.2F});
    CHECK(invisible_places::timing::
              NextTimingColouriseEffectKeyPosition(
                  effect,
                  0.5F) == std::optional<float>{0.8F});
    CHECK(invisible_places::timing::
              TimingColouriseEffectKeyCountAtPosition(
                  effect,
                  0.2F) == 2U);

    REQUIRE(invisible_places::timing::
                MoveTimingColouriseEffectParameterKeys(
                    &effect,
                    0.2F,
                    0.3F));
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterUnionKeyCountAtPosition(
                  effect,
                  0.3F) == 2U);
    CHECK(invisible_places::timing::
              RemoveTimingColouriseEffectParameterKeysAtPosition(
                  &effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.3F) == 1U);
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterUnionKeyCountAtPosition(
                  effect,
                  0.3F) == 1U);
    CHECK(invisible_places::timing::
              RemoveTimingColouriseEffectParameterKeysAtPosition(
                  &effect,
                  0.3F) == 1U);
}

TEST_CASE(
    "Timing Colourise relative phase deltas accumulate into unwrapped turns",
    "[timing][colourise][palette][phase][effect-parameters]") {
    TimingColouriseEffect effect;
    effect.basePalette = TimingColourisePalette{
        .stops = {
            {.id = "start",
             .position = 0.0F,
             .colour = {1.0F, 0.0F, 0.0F}},
            {.id = "middle",
             .position = 0.4F,
             .colour = {0.0F, 1.0F, 0.0F}},
            {.id = "end",
             .position = 1.0F,
             .colour = {0.0F, 0.0F, 1.0F}},
        },
    };
    for (const auto [position, delta] :
         std::array{
             std::pair{0.0F, 0.0F},
             std::pair{0.5F, 1.0F},
             std::pair{1.0F, 1.0F},
         }) {
        REQUIRE(invisible_places::timing::
                    AddOrUpdateTimingColouriseEffectParameterKey(
                        &effect,
                        TimingColouriseEffectParameter::PalettePhase,
                        position,
                        delta,
                        WaterScenarioInterpolation::Smooth));
    }

    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.0F) == Approx(0.0F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.25F) == Approx(0.5F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.5F) == Approx(1.0F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.75F) == Approx(1.5F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  1.0F) == Approx(2.0F));

    const auto raw =
        invisible_places::timing::CompileTimingColourisePaletteLut(
            effect.basePalette);
    CHECK(invisible_places::timing::
              EvaluateTimingColourisePaletteLut(effect, 0.0F) == raw);
    CHECK(invisible_places::timing::
              EvaluateTimingColourisePaletteLut(effect, 0.5F) == raw);
    CHECK(invisible_places::timing::
              EvaluateTimingColourisePaletteLut(effect, 1.0F) == raw);
}

TEST_CASE(
    "Timing Colourise Monotone Spline carries speed through continuing keys and rests at reversals",
    "[timing][colourise][palette][phase][velocity]") {
    TimingColouriseEffect continuing;
    continuing.effectParameterKeys = {
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 0.0F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 0.25F,
         .value = 0.5F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 1.0F,
         .value = 1.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
    };
    const auto continuingValue = [&](float position) {
        return invisible_places::timing::
            EvaluateTimingColouriseEffectParameter(
                continuing,
                TimingColouriseEffectParameter::PalettePhase,
                position);
    };
    CHECK(continuingValue(0.249F) < 0.5F);
    CHECK(continuingValue(0.25F) == Approx(0.50F).margin(1.0e-5F));
    CHECK(continuingValue(0.251F) > 0.5F);
    const float incomingVelocity =
        (continuingValue(0.25F) - continuingValue(0.249F)) /
        0.001F;
    const float outgoingVelocity =
        (continuingValue(0.251F) - continuingValue(0.25F)) /
        0.001F;
    CHECK(incomingVelocity > 1.0F);
    CHECK(outgoingVelocity > 1.0F);
    CHECK(incomingVelocity ==
          Approx(outgoingVelocity).margin(5.0e-3F));

    auto resting = continuing;
    for (auto& key : resting.effectParameterKeys) {
        key.interpolation = WaterScenarioInterpolation::Smooth;
    }
    const auto restingValue = [&](float position) {
        return invisible_places::timing::
            EvaluateTimingColouriseEffectParameter(
                resting,
                TimingColouriseEffectParameter::PalettePhase,
                position);
    };
    const float restingIncomingVelocity =
        (restingValue(0.25F) - restingValue(0.249F)) /
        0.001F;
    const float restingOutgoingVelocity =
        (restingValue(0.251F) - restingValue(0.25F)) /
        0.001F;
    CHECK(std::abs(restingIncomingVelocity) < 0.05F);
    CHECK(std::abs(restingOutgoingVelocity) < 0.05F);

    TimingColouriseEffect reversing;
    reversing.effectParameterKeys = {
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 0.0F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 0.5F,
         .value = 1.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 1.0F,
         .value = -1.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
    };
    const auto reversingValue = [&](float position) {
        return invisible_places::timing::
            EvaluateTimingColouriseEffectParameter(
                reversing,
                TimingColouriseEffectParameter::PalettePhase,
                position);
    };
    const float peak = reversingValue(0.5F);
    const float approachVelocity =
        (peak - reversingValue(0.499F)) / 0.001F;
    const float departureVelocity =
        (reversingValue(0.501F) - peak) / 0.001F;
    CHECK(peak == Approx(1.0F));
    CHECK(std::abs(approachVelocity) < 0.02F);
    CHECK(std::abs(departureVelocity) < 0.02F);
}

TEST_CASE(
    "Timing Take retiming preserves every keyed frame and full-range activation sentinel",
    "[timing][colourise][pan-extension]") {
    constexpr std::uint32_t sourceFrames = 90U;
    constexpr std::uint32_t destinationFrames = 135U;
    constexpr float scale =
        static_cast<float>(sourceFrames) /
        static_cast<float>(destinationFrames);

    invisible_places::timing::TimingTakeSceneState state;
    invisible_places::water::WaterFeatureTimingRun waterRun;
    waterRun.enabled = false;
    invisible_places::water::WaterFeatureTimeline waterFeature;
    waterFeature.feature = {
        .kind = invisible_places::water::WaterKeyedFeatureKind::Rain,
    };
    invisible_places::water::WaterKeyedSettingTrack waterSetting;
    waterSetting.settingId = "strength";
    waterSetting.active = false;
    waterSetting.keys = {
        {.position = 0.18F, .value = 0.2F},
        {.position = 0.91F, .value = 0.8F},
    };
    waterFeature.settings.push_back(std::move(waterSetting));
    waterRun.features.push_back(std::move(waterFeature));
    state.waterFeatureTimingRuns.push_back(std::move(waterRun));

    TimingColouriseEffect effect;
    effect.enabled = false;
    effect.activationRange = {.start = 0.12F, .end = 1.0F};
    effect.basePalette.stops = {
        {.id = "base-stop", .position = 0.77F},
    };
    effect.effectParameterKeys = {
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 0.0F,
         .value = -0.2F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 0.37F,
         .value = 0.9F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 0.72F,
         .value = 0.1F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.parameter = TimingColouriseEffectParameter::PalettePhase,
         .position = 1.0F,
         .value = 0.6F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
    };
    effect.paletteKeys = {
        {.position = 0.31F,
         .palette = {.stops = {{.id = "snapshot-stop",
                               .position = 0.66F}}}},
    };
    effect.paletteStopParameterKeys = {
        {.stopId = "base-stop",
         .parameter = TimingColourisePaletteStopParameter::Position,
         .position = 0.42F,
         .scalarValue = 0.81F},
    };
    effect.boundsParameterKeys = {
        {.parameter = TimingColouriseBoundsParameter::Lower,
         .position = 0.51F,
         .value = 0.2F},
    };
    effect.boundsKeys = {
        {.position = 0.60F, .bounds = {.lower = 0.1F, .upper = 0.9F}},
    };
    TimingColouriseFieldBoundsMemory memory;
    memory.boundsParameterKeys = {
        {.parameter = TimingColouriseBoundsParameter::Upper,
         .position = 0.69F,
         .value = 0.9F},
    };
    memory.boundsKeys = {
        {.position = 0.78F, .bounds = {.lower = 0.2F, .upper = 0.8F}},
    };
    effect.fieldBoundsMemory.push_back(std::move(memory));
    state.colouriseEffects.push_back(effect);

    TimingColouriseEffect partialRange;
    partialRange.activationRange = {.start = 0.24F, .end = 0.81F};
    state.colouriseEffects.push_back(std::move(partialRange));

    const auto original = state;
    REQUIRE(invisible_places::timing::
                RetimeTimingTakeSceneStateNormalizedPositions(
                    &state,
                    sourceFrames,
                    destinationFrames));

    REQUIRE(state.waterFeatureTimingRuns.size() == 1U);
    REQUIRE(state.waterFeatureTimingRuns.front().features.size() == 1U);
    const auto& retimedWater = state.waterFeatureTimingRuns.front()
                                   .features.front()
                                   .settings.front();
    CHECK_FALSE(state.waterFeatureTimingRuns.front().enabled);
    CHECK_FALSE(retimedWater.active);
    REQUIRE(retimedWater.keys.size() == 2U);
    CHECK(retimedWater.keys[0U].position == Approx(0.18F * scale));
    CHECK(retimedWater.keys[1U].position == Approx(0.91F * scale));
    CHECK(retimedWater.keys[1U].value == Approx(0.8F));

    REQUIRE(state.colouriseEffects.size() == 2U);
    const auto& retimed = state.colouriseEffects[0U];
    CHECK_FALSE(retimed.enabled);
    CHECK(retimed.activationRange.start == Approx(0.12F * scale));
    CHECK(retimed.activationRange.end == Approx(1.0F));
    CHECK(state.colouriseEffects[1U].activationRange.start ==
          Approx(0.24F * scale));
    CHECK(state.colouriseEffects[1U].activationRange.end ==
          Approx(0.81F * scale));
    REQUIRE(retimed.effectParameterKeys.size() == 4U);
    CHECK(retimed.effectParameterKeys[1U].position ==
          Approx(0.37F * scale));
    CHECK(retimed.effectParameterKeys.back().position == Approx(scale));
    REQUIRE(retimed.paletteKeys.size() == 1U);
    CHECK(retimed.paletteKeys.front().position == Approx(0.31F * scale));
    REQUIRE(retimed.paletteStopParameterKeys.size() == 1U);
    CHECK(retimed.paletteStopParameterKeys.front().position ==
          Approx(0.42F * scale));
    REQUIRE(retimed.boundsParameterKeys.size() == 1U);
    CHECK(retimed.boundsParameterKeys.front().position ==
          Approx(0.51F * scale));
    REQUIRE(retimed.boundsKeys.size() == 1U);
    CHECK(retimed.boundsKeys.front().position == Approx(0.60F * scale));
    REQUIRE(retimed.fieldBoundsMemory.size() == 1U);
    CHECK(retimed.fieldBoundsMemory.front()
              .boundsParameterKeys.front()
              .position == Approx(0.69F * scale));
    CHECK(retimed.fieldBoundsMemory.front().boundsKeys.front().position ==
          Approx(0.78F * scale));

    // Palette-space coordinates and authored values are not animation-time
    // coordinates and must not be touched by the retime.
    CHECK(retimed.basePalette.stops.front().position == Approx(0.77F));
    CHECK(retimed.paletteKeys.front().palette.stops.front().position ==
          Approx(0.66F));
    CHECK(retimed.paletteStopParameterKeys.front().scalarValue ==
          Approx(0.81F));

    const auto evaluate = [](const TimingColouriseEffect& value,
                             float position) {
        return invisible_places::timing::
            EvaluateTimingColouriseEffectParameter(
                value,
                TimingColouriseEffectParameter::PalettePhase,
                position);
    };

    for (std::uint32_t frame = 0U; frame <= sourceFrames; ++frame) {
        const float sourcePosition = static_cast<float>(frame) /
            static_cast<float>(sourceFrames);
        const float destinationPosition = static_cast<float>(frame) /
            static_cast<float>(destinationFrames);
        CHECK(evaluate(retimed, destinationPosition) ==
              Approx(evaluate(
                         original.colouriseEffects.front(),
                         sourcePosition))
                  .margin(1.0e-6F));
    }
    const float terminal = evaluate(
        original.colouriseEffects.front(),
        1.0F);
    for (std::uint32_t frame = sourceFrames + 1U;
         frame <= destinationFrames;
         ++frame) {
        CHECK(evaluate(
                  retimed,
                  static_cast<float>(frame) /
                      static_cast<float>(destinationFrames)) ==
                  Approx(terminal).margin(1.0e-6F));
    }

    SECTION("A prepended camera span shifts keyed frames without shifting range sentinels") {
        constexpr std::uint32_t prependedFrames = 27U;
        constexpr std::uint32_t bidirectionalDestinationFrames = 162U;
        auto shifted = original;
        TimingColouriseEffect fullRange;
        fullRange.activationRange = {.start = 0.0F, .end = 1.0F};
        shifted.colouriseEffects.push_back(fullRange);
        REQUIRE(invisible_places::timing::
                    RetimeTimingTakeSceneStateNormalizedPositions(
                        &shifted,
                        sourceFrames,
                        bidirectionalDestinationFrames,
                        prependedFrames));
        const auto shiftedPosition = [](float position) {
            return (27.0F + position * 90.0F) / 162.0F;
        };
        CHECK(shifted.waterFeatureTimingRuns.front()
                  .features.front()
                  .settings.front()
                  .keys.front()
                  .position == Approx(shiftedPosition(0.18F)));
        CHECK(shifted.colouriseEffects.front()
                  .effectParameterKeys.front()
                  .position == Approx(shiftedPosition(0.0F)));
        CHECK(shifted.colouriseEffects.front().activationRange.start ==
              Approx(shiftedPosition(0.12F)));
        CHECK(shifted.colouriseEffects.front().activationRange.end ==
              Approx(1.0F));
        CHECK(shifted.colouriseEffects.back().activationRange.start ==
              Approx(0.0F));
        CHECK(shifted.colouriseEffects.back().activationRange.end ==
              Approx(1.0F));
        CHECK_FALSE(invisible_places::timing::
                        RetimeTimingTakeSceneStateNormalizedPositions(
                            &shifted,
                            sourceFrames,
                            sourceFrames,
                            1U));
    }

    const float unchangedPosition =
        state.colouriseEffects.front().effectParameterKeys[1U].position;
    CHECK_FALSE(invisible_places::timing::
                    RetimeTimingTakeSceneStateNormalizedPositions(
                        &state,
                        sourceFrames,
                        0U));
    CHECK_FALSE(invisible_places::timing::
                    RetimeTimingTakeSceneStateNormalizedPositions(
                        &state,
                        0U,
                        destinationFrames));
    CHECK(state.colouriseEffects.front()
              .effectParameterKeys[1U]
              .position == Approx(unchangedPosition));
    CHECK_FALSE(invisible_places::timing::
                    RetimeTimingTakeSceneStateNormalizedPositions(
                        nullptr,
                        sourceFrames,
                        destinationFrames));
}

TEST_CASE(
    "Timing Colourise phase keys are relative one-turn deltas and split cleanly on insertion",
    "[timing][colourise][palette][phase][relative]") {
    TimingColouriseEffect effect;
    effect.palettePhaseOffset = 0.25F;
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.2F,
                    0.5F,
                    WaterScenarioInterpolation::Smooth));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.8F,
                    0.5F,
                    WaterScenarioInterpolation::Smooth));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.2F) == Approx(0.75F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.8F) == Approx(1.25F));

    const float insertionDelta = invisible_places::timing::
        TimingColourisePalettePhaseDeltaFromPrevious(effect, 0.5F);
    CHECK(insertionDelta == Approx(0.25F));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.5F,
                    insertionDelta,
                    WaterScenarioInterpolation::Smooth));
    REQUIRE(effect.effectParameterKeys.size() == 3U);
    CHECK(effect.effectParameterKeys[1].value == Approx(0.25F));
    CHECK(effect.effectParameterKeys[2].value == Approx(0.25F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.8F) == Approx(1.25F));

    CHECK(invisible_places::timing::
              RemoveTimingColouriseEffectParameterKeysAtPosition(
                  &effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.5F) == 1U);
    REQUIRE(effect.effectParameterKeys.size() == 2U);
    CHECK(effect.effectParameterKeys[1].value == Approx(0.5F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.8F) == Approx(1.25F));

    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    1.0F,
                    4.0F,
                    WaterScenarioInterpolation::Smooth));
    CHECK(effect.effectParameterKeys.back().value == Approx(1.0F));
}

TEST_CASE(
    "Timing Colourise Monotone Spline is shared by bounds and effect scalar tracks",
    "[timing][colourise][bounds][emissive][velocity]") {
    TimingColouriseEffect effect;
    effect.emissiveEnabled = true;
    for (const auto [position, value] :
         std::array{
             std::pair{0.0F, 0.0F},
             std::pair{0.25F, 0.2F},
             std::pair{1.0F, 1.0F},
         }) {
        REQUIRE(invisible_places::timing::
                    AddOrUpdateTimingColouriseEffectParameterKey(
                        &effect,
                        TimingColouriseEffectParameter::EmissiveLevel,
                        position,
                        value,
                        WaterScenarioInterpolation::SmoothVelocity));
        REQUIRE(invisible_places::timing::
                    AddOrUpdateTimingColouriseBoundsParameterKey(
                        &effect,
                        TimingColouriseBoundsParameter::Lower,
                        position,
                        value,
                        WaterScenarioInterpolation::SmoothVelocity));
    }

    const auto emissiveAt = [&](float position) {
        return invisible_places::timing::EvaluateTimingEmissiveLevel(
            effect,
            position);
    };
    const auto lowerAt = [&](float position) {
        return invisible_places::timing::EvaluateTimingColouriseBounds(
                   effect,
                   position)
            .lower;
    };
    const auto checkContinuousVelocity = [&](const auto& valueAt) {
        const float incoming =
            (valueAt(0.25F) - valueAt(0.249F)) / 0.001F;
        const float outgoing =
            (valueAt(0.251F) - valueAt(0.25F)) / 0.001F;
        CHECK(incoming > 0.5F);
        CHECK(outgoing > 0.5F);
        CHECK(incoming == Approx(outgoing).margin(0.01F));
    };
    checkContinuousVelocity(emissiveAt);
    checkContinuousVelocity(lowerAt);
    CHECK(std::all_of(
        effect.effectParameterKeys.begin(),
        effect.effectParameterKeys.end(),
        [](const auto& key) {
            return key.interpolation ==
                   WaterScenarioInterpolation::SmoothVelocity;
        }));
    CHECK(std::all_of(
        effect.boundsParameterKeys.begin(),
        effect.boundsParameterKeys.end(),
        [](const auto& key) {
            return key.interpolation ==
                   WaterScenarioInterpolation::SmoothVelocity;
        }));
}

TEST_CASE(
    "Timing Colourise Centripetal Catmull-Rom is C1 across uneven scalar keys",
    "[timing][colourise][bounds][emissive][catmull-rom]") {
    TimingColouriseEffect effect;
    effect.emissiveEnabled = true;
    for (const auto [position, value] :
         std::array{
             std::pair{0.0F, 0.05F},
             std::pair{0.18F, 0.42F},
             std::pair{0.55F, 0.64F},
             std::pair{1.0F, 0.90F},
         }) {
        REQUIRE(invisible_places::timing::
                    AddOrUpdateTimingColouriseEffectParameterKey(
                        &effect,
                        TimingColouriseEffectParameter::EmissiveLevel,
                        position,
                        value,
                        WaterScenarioInterpolation::
                            CentripetalCatmullRom));
        REQUIRE(invisible_places::timing::
                    AddOrUpdateTimingColouriseBoundsParameterKey(
                        &effect,
                        TimingColouriseBoundsParameter::Lower,
                        position,
                        value,
                        WaterScenarioInterpolation::
                            CentripetalCatmullRom));
    }
    const auto emissiveAt = [&](float position) {
        return invisible_places::timing::EvaluateTimingEmissiveLevel(
            effect,
            position);
    };
    const auto lowerAt = [&](float position) {
        return invisible_places::timing::EvaluateTimingColouriseBounds(
                   effect,
                   position)
            .lower;
    };
    const auto checkC1 = [&](const auto& valueAt, float keyPosition) {
        constexpr float epsilon = 1.0e-4F;
        const float value = valueAt(keyPosition);
        const float incoming =
            (value - valueAt(keyPosition - epsilon)) / epsilon;
        const float outgoing =
            (valueAt(keyPosition + epsilon) - value) / epsilon;
        CHECK(std::isfinite(incoming));
        CHECK(std::isfinite(outgoing));
        CHECK(incoming == Approx(outgoing).margin(0.03F));
    };
    for (const float keyPosition : {0.18F, 0.55F}) {
        checkC1(emissiveAt, keyPosition);
        checkC1(lowerAt, keyPosition);
    }
    CHECK(emissiveAt(0.18F) == Approx(0.42F).margin(1.0e-5F));
    CHECK(emissiveAt(0.55F) == Approx(0.64F).margin(1.0e-5F));

    auto smoothStep = effect;
    for (auto& key : smoothStep.effectParameterKeys) {
        key.interpolation = WaterScenarioInterpolation::Smooth;
    }
    CHECK(std::abs(
              emissiveAt(0.36F) -
              invisible_places::timing::EvaluateTimingEmissiveLevel(
                  smoothStep,
                  0.36F)) > 1.0e-3F);
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

    effect.colouriseAmountOverrideMode =
        TimingColouriseAmountOverrideMode::Scale;
    effect.colouriseAmountOverride = 0.5F;
    const auto scaledHalfway =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.5F);
    CHECK(scaledHalfway.front()[3] == Approx(0.2F));
    CHECK(scaledHalfway.back()[3] == Approx(0.3F));
    effect.colouriseAmountOverrideMode =
        TimingColouriseAmountOverrideMode::Maximum;
    effect.colouriseAmountOverride = 1.0F;

    effect.paletteKeys.front().interpolation =
        WaterScenarioInterpolation::Hold;
    const auto smoothed =
        invisible_places::timing::EvaluateTimingColourisePaletteLut(
            effect,
            0.25F);
    CHECK(smoothed.front()[0] == Approx(0.84375F));
    CHECK(smoothed.front()[2] == Approx(0.15625F));
}

TEST_CASE(
    "Timing Colourise signed edge fade works inward and outward",
    "[timing][colourise][bounds]") {
    CHECK(TimingColouriseBounds{}.edgeFade == Approx(0.10F));
    CHECK(
        invisible_places::timing::SanitizeTimingColouriseBounds(
            {.edgeFade = -0.75F})
            .edgeFade == Approx(-0.5F));
    CHECK(
        invisible_places::timing::SanitizeTimingColouriseBounds(
            {.edgeFade = 0.75F})
            .edgeFade == Approx(0.5F));
    CHECK(
        invisible_places::timing::SanitizeTimingColouriseBounds(
            {.edgeFade = 0.0F})
            .edgeFade == Approx(0.0F));
    CHECK(
        invisible_places::timing::SanitizeTimingColouriseBounds(
            {.edgeFade = std::numeric_limits<float>::quiet_NaN()})
            .edgeFade == Approx(0.10F));

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

    auto bounds =
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

    bounds.edgeFade = -0.2F;
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 11.9F) ==
        Approx(0.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 12.0F) ==
        Approx(0.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 13.5F) ==
        Approx(0.5F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 15.0F) ==
        Approx(1.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 30.0F) ==
        Approx(1.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 31.5F) ==
        Approx(0.5F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 33.0F) ==
        Approx(0.0F));
    CHECK(
        invisible_places::timing::TimingColouriseBoundsMask(bounds, 33.1F) ==
        Approx(0.0F));
}

TEST_CASE(
    "Timing Colourise stores all supported smooth interpolation styles",
    "[timing][colourise][keys][interpolation]") {
    TimingColouriseEffect effect;
    effect.basePalette = invisible_places::timing::
        SanitizeTimingColourisePalette({
            .stops = {{.id = "stop", .position = 0.5F}},
        });

    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColourisePaletteStopScalarKey(
                    &effect,
                    "stop",
                    TimingColourisePaletteStopParameter::Position,
                    0.1F,
                    0.25F,
                    WaterScenarioInterpolation::Hold));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColourisePaletteStopColourKey(
                    &effect,
                    "stop",
                    0.2F,
                    {0.1F, 0.2F, 0.3F},
                    WaterScenarioInterpolation::Linear));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.3F,
                    0.4F,
                    WaterScenarioInterpolation::Hold));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &effect,
                    TimingColouriseBoundsParameter::Lower,
                    0.4F,
                    0.2F,
                    WaterScenarioInterpolation::Linear));
    invisible_places::timing::AddOrUpdateTimingColouriseBoundsKey(
        &effect,
        0.5F,
        {.lower = 0.2F, .upper = 0.8F},
        WaterScenarioInterpolation::Hold);
    invisible_places::timing::AddOrUpdateTimingColourisePaletteKey(
        &effect,
        0.6F,
        Solid({0.2F, 0.4F, 0.8F}),
        WaterScenarioInterpolation::Linear);

    const auto allSmooth = [](const auto& keys) {
        return std::all_of(
            keys.begin(),
            keys.end(),
            [](const auto& key) {
                return key.interpolation ==
                       WaterScenarioInterpolation::Smooth;
            });
    };
    CHECK(allSmooth(effect.paletteKeys));
    CHECK(allSmooth(effect.paletteStopParameterKeys));
    CHECK(allSmooth(effect.effectParameterKeys));
    CHECK(allSmooth(effect.boundsKeys));
    CHECK(allSmooth(effect.boundsParameterKeys));

    const auto setAllSmoothVelocity = [](auto* keys) {
        for (auto& key : *keys) {
            key.interpolation =
                WaterScenarioInterpolation::SmoothVelocity;
        }
    };
    setAllSmoothVelocity(&effect.paletteKeys);
    setAllSmoothVelocity(&effect.paletteStopParameterKeys);
    setAllSmoothVelocity(&effect.effectParameterKeys);
    setAllSmoothVelocity(&effect.boundsKeys);
    setAllSmoothVelocity(&effect.boundsParameterKeys);
    const auto sanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    const auto allSmoothVelocity = [](const auto& keys) {
        return std::all_of(
            keys.begin(),
            keys.end(),
            [](const auto& key) {
                return key.interpolation ==
                       WaterScenarioInterpolation::SmoothVelocity;
            });
    };
    CHECK(allSmoothVelocity(sanitized.paletteKeys));
    CHECK(allSmoothVelocity(sanitized.paletteStopParameterKeys));
    CHECK(allSmoothVelocity(sanitized.effectParameterKeys));
    CHECK(allSmoothVelocity(sanitized.boundsKeys));
    CHECK(allSmoothVelocity(sanitized.boundsParameterKeys));

    const auto setAllCentripetalCatmullRom = [](auto* keys) {
        for (auto& key : *keys) {
            key.interpolation =
                WaterScenarioInterpolation::CentripetalCatmullRom;
        }
    };
    setAllCentripetalCatmullRom(&effect.paletteKeys);
    setAllCentripetalCatmullRom(&effect.paletteStopParameterKeys);
    setAllCentripetalCatmullRom(&effect.effectParameterKeys);
    setAllCentripetalCatmullRom(&effect.boundsKeys);
    setAllCentripetalCatmullRom(&effect.boundsParameterKeys);
    const auto catmullSanitized =
        invisible_places::timing::SanitizeTimingColouriseEffect(effect);
    const auto allCentripetalCatmullRom = [](const auto& keys) {
        return std::all_of(
            keys.begin(),
            keys.end(),
            [](const auto& key) {
                return key.interpolation == WaterScenarioInterpolation::
                                                CentripetalCatmullRom;
            });
    };
    CHECK(allCentripetalCatmullRom(catmullSanitized.paletteKeys));
    CHECK(allCentripetalCatmullRom(
        catmullSanitized.paletteStopParameterKeys));
    CHECK(allCentripetalCatmullRom(
        catmullSanitized.effectParameterKeys));
    CHECK(allCentripetalCatmullRom(catmullSanitized.boundsKeys));
    CHECK(allCentripetalCatmullRom(
        catmullSanitized.boundsParameterKeys));

    effect.effectParameterKeys.front().interpolation =
        static_cast<WaterScenarioInterpolation>(255);
    CHECK(
        invisible_places::timing::SanitizeTimingColouriseEffect(effect)
            .effectParameterKeys.front()
            .interpolation == WaterScenarioInterpolation::Smooth);
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
    "Timing Colourise noncoincident local keys stay isolated to their own lane",
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
    "Timing Colourise coincident geometric bounds keys move together",
    "[timing][colourise][bounds][keys][local-move]") {
    TimingColouriseEffect effect;
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Lower,
                0.2F,
                1.0F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Upper,
                0.2F,
                4.0F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::EdgeFade,
                0.2F,
                0.1F));
    REQUIRE(
        invisible_places::timing::
            AddOrUpdateTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Upper,
                0.8F,
                5.0F));

    CHECK(
        invisible_places::timing::
            CanMoveTimingColouriseBoundsParameterKeysAtPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.2F,
                0.4F));
    REQUIRE(
        invisible_places::timing::
            MoveTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Lower,
                0.2F,
                0.4F));
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.4F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Upper,
                0.4F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::EdgeFade,
                0.2F) == 1U);

    REQUIRE(
        invisible_places::timing::
            MoveTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Upper,
                0.4F,
                0.6F));
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.6F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Upper,
                0.6F) == 1U);
    REQUIRE(
        invisible_places::timing::
            MoveTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::EdgeFade,
                0.2F,
                0.3F));
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::EdgeFade,
                0.3F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.6F) == 1U);

    CHECK_FALSE(
        invisible_places::timing::
            CanMoveTimingColouriseBoundsParameterKeysAtPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.6F,
                0.8F));
    CHECK_FALSE(
        invisible_places::timing::
            MoveTimingColouriseBoundsParameterKey(
                &effect,
                TimingColouriseBoundsParameter::Lower,
                0.6F,
                0.8F));
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Lower,
                0.6F) == 1U);
    CHECK(
        invisible_places::timing::
            TimingColouriseBoundsParameterKeyCountAtPosition(
                effect,
                TimingColouriseBoundsParameter::Upper,
                0.6F) == 1U);
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
    REQUIRE(sanitized.colouriseEffects.size() == 6U);
    CHECK(sanitized.colouriseEffects.front().id == "effect-0");
    CHECK(sanitized.colouriseEffects.back().id == "effect-5");

    std::uint32_t sequence = 1U;
    sanitized.colouriseEffects.front().id = "colourise-effect-1";
    CHECK(
        invisible_places::timing::AllocateTimingColouriseEffectId(
            sanitized.colouriseEffects,
            &sequence) == "colourise-effect-2");
    CHECK(sequence == 3U);
}

TEST_CASE(
    "Timing Colourise effects move by final list position with their keys",
    "[timing][colourise][state][reorder]") {
    using invisible_places::timing::TimingColouriseEffect;
    using invisible_places::timing::TimingColouriseEffectParameter;
    using invisible_places::timing::TimingColouriseEffectParameterKey;

    const auto effect = [](std::string id, float keyedValue) {
        TimingColouriseEffect result;
        result.id = std::move(id);
        result.effectParameterKeys.push_back(
            TimingColouriseEffectParameterKey{
                .parameter =
                    TimingColouriseEffectParameter::PalettePhase,
                .position = 0.5F,
                .value = keyedValue,
            });
        return result;
    };
    std::vector<TimingColouriseEffect> effects{
        effect("a", 1.0F),
        effect("b", 2.0F),
        effect("c", 3.0F),
        effect("d", 4.0F),
    };

    REQUIRE(
        invisible_places::timing::MoveTimingColouriseEffect(
            &effects,
            0U,
            3U));
    CHECK(effects[0].id == "b");
    CHECK(effects[1].id == "c");
    CHECK(effects[2].id == "d");
    CHECK(effects[3].id == "a");
    REQUIRE(effects[3].effectParameterKeys.size() == 1U);
    CHECK(effects[3].effectParameterKeys.front().value == Approx(1.0F));

    REQUIRE(
        invisible_places::timing::MoveTimingColouriseEffect(
            &effects,
            3U,
            1U));
    CHECK(effects[0].id == "b");
    CHECK(effects[1].id == "a");
    CHECK(effects[2].id == "c");
    CHECK(effects[3].id == "d");
    REQUIRE(effects[1].effectParameterKeys.size() == 1U);
    CHECK(effects[1].effectParameterKeys.front().value == Approx(1.0F));

    CHECK_FALSE(
        invisible_places::timing::MoveTimingColouriseEffect(
            &effects,
            1U,
            1U));
    CHECK_FALSE(
        invisible_places::timing::MoveTimingColouriseEffect(
            &effects,
            4U,
            0U));
    CHECK_FALSE(
        invisible_places::timing::MoveTimingColouriseEffect(
            nullptr,
            0U,
            1U));
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
    effect.activationRange = {.start = 0.2F, .end = 0.85F};
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
    {
        std::ifstream input{file.path};
        REQUIRE(input.is_open());
        const auto saved = nlohmann::json::parse(input);
        const auto& stateJson = saved.at("timing_take_states")[0];
        REQUIRE(stateJson.at("timing_effects").size() == 1U);
        REQUIRE(stateJson.at("colourise_effects").size() == 1U);
        CHECK(stateJson.at("timing_effect_sequence") ==
              stateJson.at("colourise_effect_sequence"));
        // The legacy kind is still written so pre-Visual-Feature readers
        // can open this document; current readers use the aspect flags.
        CHECK(stateJson.at("timing_effects")[0].at("kind") ==
              "colourise");
        CHECK(stateJson.at("timing_effects")[0].at("colourise_enabled") ==
              true);
        CHECK(stateJson.at("timing_effects")[0].at("emissive_enabled") ==
              false);
        const auto& activation =
            stateJson.at("timing_effects")[0]
                .at("activation_range");
        CHECK(activation.at("start") == Approx(0.2F));
        CHECK(activation.at("end") == Approx(0.85F));
    }
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
    CHECK(loadedState.colouriseEffects.front().activationRange.start ==
          Approx(0.2F));
    CHECK(loadedState.colouriseEffects.front().activationRange.end ==
          Approx(0.85F));
    CHECK(
        loadedState.colouriseEffects.front().field.scalarFieldName ==
        "SurfaceRoughness-Sml");
    // Aspect-flagged round-trips stay attached to the shared Global
    // bounds unless the document says otherwise.
    CHECK_FALSE(loadedState.colouriseEffects.front().boundsEdited);
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
    "Legacy single-aspect timing effects merge into Visual Features on load",
    "[timing][colourise][emissive][project][serialization][migration]") {
    using invisible_places::timing::TimingColouriseEffectParameterKey;

    invisible_places::serialization::ProjectDocument document;
    invisible_places::timing::TimingTakeSceneState state{
        .takeId = "timing-take-mixed",
        .sceneGroupName = "Site A",
    };
    state.colouriseEffectSequence = 12U;

    const auto makeColourise = [](std::string id,
                                  std::string field,
                                  float start,
                                  float end) {
        TimingColouriseEffect effect;
        effect.id = std::move(id);
        effect.field.scalarFieldName = std::move(field);
        effect.activationRange = {.start = start, .end = end};
        return effect;
    };
    const auto makeEmissive = [](std::string id,
                                 std::string field,
                                 float start,
                                 float end,
                                 float level) {
        TimingColouriseEffect effect;
        effect.id = std::move(id);
        effect.colouriseEnabled = false;
        effect.emissiveEnabled = true;
        effect.field.scalarFieldName = std::move(field);
        effect.activationRange = {.start = start, .end = end};
        effect.emissiveLevel = level;
        return effect;
    };

    // Each matrix case lives in its own disjoint activation window so the
    // renderer slot-capacity gate never engages inside a merge candidate's
    // window (concurrency stays far below eight slots).
    // (a) Same field, exactly equal window, identical bounds authoring,
    // both enabled: the pair merges into one dual-aspect feature.
    auto mergeColourise =
        makeColourise("colourise-merge", "Mineral", 0.0F, 0.1F);
    mergeColourise.name = "Minerals";
    mergeColourise.basePalette = Solid({0.1F, 0.5F, 0.8F}, 0.6F);
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &mergeColourise,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.5F,
                    1.5F));
    // A dormant EmissiveLevel key on the colourise object never rendered;
    // the merge must replace it with the partner's authored track.
    mergeColourise.effectParameterKeys.push_back(
        TimingColouriseEffectParameterKey{
            .parameter = TimingColouriseEffectParameter::EmissiveLevel,
            .position = 0.15F,
            .value = 9.0F,
        });
    state.colouriseEffects.push_back(mergeColourise);
    auto mergeEmissive =
        makeEmissive("emissive-merge", "Mineral", 0.0F, 0.1F, 2.5F);
    mergeEmissive.name = "Heat glow";
    // Dormant colourise-side data on the emissive partner never rendered
    // and is dropped by the merge; only its emissive level and
    // EmissiveLevel keys survive.
    mergeEmissive.effectParameterKeys.push_back(
        TimingColouriseEffectParameterKey{
            .parameter = TimingColouriseEffectParameter::AmountOverride,
            .position = 0.3F,
            .value = 0.4F,
        });
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &mergeEmissive,
                    TimingColouriseEffectParameter::EmissiveLevel,
                    0.6F,
                    4.0F));
    state.colouriseEffects.push_back(mergeEmissive);

    // (b) Same field but overlapping-yet-different windows: no merge.
    state.colouriseEffects.push_back(
        makeColourise("colourise-window", "Silt", 0.15F, 0.25F));
    state.colouriseEffects.push_back(
        makeEmissive("emissive-window", "Silt", 0.18F, 0.25F, 1.5F));

    // (b2) Windows equal within the former 1e-4 key tolerance but not
    // exactly equal: no merge under the exact-equality rule.
    state.colouriseEffects.push_back(
        makeColourise("colourise-tolerance", "Basalt", 0.3F, 0.4F));
    state.colouriseEffects.push_back(makeEmissive(
        "emissive-tolerance",
        "Basalt",
        0.30005F,
        0.39995F,
        2.25F));

    // (c) Identical windows on different fields: no merge.
    state.colouriseEffects.push_back(
        makeColourise("colourise-field", "Clay", 0.45F, 0.5F));
    state.colouriseEffects.push_back(
        makeEmissive("emissive-field", "Heat", 0.45F, 0.5F, 3.0F));

    // (d) Enabled mismatch: no merge.
    state.colouriseEffects.push_back(
        makeColourise("colourise-enabled", "Iron", 0.55F, 0.6F));
    auto mismatchEmissive = makeEmissive(
        "emissive-enabled-mismatch",
        "Iron",
        0.55F,
        0.6F,
        2.0F);
    mismatchEmissive.enabled = false;
    state.colouriseEffects.push_back(mismatchEmissive);

    // (d2) Both disabled: a disabled pair no longer merges either.
    auto disabledColourise =
        makeColourise("colourise-both-disabled", "Copper", 0.55F, 0.6F);
    disabledColourise.enabled = false;
    state.colouriseEffects.push_back(disabledColourise);
    auto disabledEmissive = makeEmissive(
        "emissive-both-disabled",
        "Copper",
        0.55F,
        0.6F,
        1.25F);
    disabledEmissive.enabled = false;
    state.colouriseEffects.push_back(disabledEmissive);

    // (g) Differing base bounds: bounds gate emissive output exactly as
    // they gate colourise, so the pair cannot share one merged bounds.
    state.colouriseEffects.push_back(
        makeColourise("colourise-bounds", "Quartz", 0.65F, 0.7F));
    auto boundsEmissive =
        makeEmissive("emissive-bounds", "Quartz", 0.65F, 0.7F, 1.5F);
    boundsEmissive.baseBounds = {
        .lower = 0.0F,
        .upper = 0.9F,
        .edgeFade = 0.1F,
    };
    state.colouriseEffects.push_back(boundsEmissive);

    // (h) Differing bounds parameter keys: no merge.
    auto boundsKeyColourise =
        makeColourise("colourise-bounds-keys", "Mica", 0.75F, 0.8F);
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseBoundsParameterKey(
                    &boundsKeyColourise,
                    TimingColouriseBoundsParameter::Lower,
                    0.5F,
                    0.2F));
    state.colouriseEffects.push_back(boundsKeyColourise);
    state.colouriseEffects.push_back(
        makeEmissive("emissive-bounds-keys", "Mica", 0.75F, 0.8F, 1.5F));

    // (e) One emissive with two colourise candidates: only the first
    // candidate in list order absorbs it.
    state.colouriseEffects.push_back(
        makeColourise("colourise-first", "Moss", 0.85F, 0.9F));
    state.colouriseEffects.push_back(
        makeColourise("colourise-second", "Moss", 0.85F, 0.9F));
    state.colouriseEffects.push_back(
        makeEmissive("emissive-shared", "Moss", 0.85F, 0.9F, 1.75F));

    document.timingTakeStates.push_back(state);

    TemporaryTimingColouriseFile file{
        "invisible_places_mixed_timing_effects_round_trip.ipproj"};
    std::string error;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(
        document,
        file.path,
        &error));

    nlohmann::json saved;
    {
        std::ifstream input{file.path};
        REQUIRE(input.is_open());
        saved = nlohmann::json::parse(input);
    }
    const auto& savedState = saved.at("timing_take_states")[0];
    REQUIRE(savedState.at("timing_effects").size() == 19U);
    // The legacy mirror only carries colourise-capable effects.
    REQUIRE(savedState.at("colourise_effects").size() == 10U);
    CHECK(savedState.at("timing_effect_sequence") == 12U);
    CHECK(savedState.at("colourise_effect_sequence") == 12U);
    CHECK(savedState.at("timing_effects")[0].at("kind") == "colourise");
    CHECK(savedState.at("timing_effects")[0].at("colourise_enabled") ==
          true);
    CHECK(savedState.at("timing_effects")[0].at("emissive_enabled") ==
          false);
    CHECK(savedState.at("timing_effects")[1].at("kind") == "emissive");
    CHECK(savedState.at("timing_effects")[1].at("colourise_enabled") ==
          false);
    CHECK(savedState.at("timing_effects")[1].at("emissive_enabled") ==
          true);
    CHECK(savedState.at("timing_effects")[1].at("emissive_level") ==
          Approx(2.5F));

    // (f) Aspect-flagged documents reload verbatim; the merge never runs
    // and the effects stay attached to the shared Global bounds.
    const auto flagged =
        invisible_places::serialization::LoadProjectDocument(
            file.path,
            &error);
    REQUIRE(flagged.has_value());
    REQUIRE(flagged->timingTakeStates.size() == 1U);
    const auto& flaggedEffects =
        flagged->timingTakeStates.front().colouriseEffects;
    REQUIRE(flaggedEffects.size() == 19U);
    CHECK(flaggedEffects[0].colouriseEnabled);
    CHECK_FALSE(flaggedEffects[0].emissiveEnabled);
    CHECK_FALSE(flaggedEffects[0].boundsEdited);
    CHECK_FALSE(flaggedEffects[1].colouriseEnabled);
    CHECK(flaggedEffects[1].emissiveEnabled);
    CHECK(flaggedEffects[1].emissiveLevel == Approx(2.5F));
    CHECK_FALSE(flaggedEffects[1].boundsEdited);

    // Strip the aspect flags to recreate a pre-Visual-Feature document
    // that only carries the legacy kind discriminator.
    auto legacy = saved;
    legacy["schema_version"] = 59U;
    for (auto& effectJson :
         legacy.at("timing_take_states")[0].at("timing_effects")) {
        effectJson.erase("colourise_enabled");
        effectJson.erase("emissive_enabled");
    }
    TemporaryTimingColouriseFile legacyFile{
        "invisible_places_legacy_kind_timing_effects.ipproj"};
    {
        std::ofstream output{legacyFile.path};
        REQUIRE(output.is_open());
        output << legacy.dump(2);
    }
    const auto legacyLoaded =
        invisible_places::serialization::LoadProjectDocument(
            legacyFile.path,
            &error);
    REQUIRE(legacyLoaded.has_value());
    REQUIRE(legacyLoaded->timingTakeStates.size() == 1U);
    const auto& mergedEffects =
        legacyLoaded->timingTakeStates.front().colouriseEffects;
    REQUIRE(mergedEffects.size() == 17U);

    const auto& merged = mergedEffects[0];
    CHECK(merged.id == "colourise-merge");
    CHECK(merged.name == "Minerals");
    CHECK(merged.colouriseEnabled);
    CHECK(merged.emissiveEnabled);
    CHECK(merged.activationRange.start == Approx(0.0F));
    CHECK(merged.activationRange.end == Approx(0.1F));
    CHECK(merged.emissiveLevel == Approx(2.5F));
    CHECK(merged.basePalette.stops.front().colour[2] == Approx(0.8F));
    // Legacy authored bounds load detached so later Global edits can
    // never silently overwrite them.
    CHECK(merged.boundsEdited);
    // The colourise object's own PalettePhase key survives; its dormant
    // EmissiveLevel key is replaced by the partner's authored track; the
    // partner's dormant AmountOverride key is dropped with the partner.
    REQUIRE(merged.effectParameterKeys.size() == 2U);
    CHECK(merged.effectParameterKeys[0].parameter ==
          TimingColouriseEffectParameter::PalettePhase);
    CHECK(merged.effectParameterKeys[0].position == Approx(0.5F));
    CHECK(merged.effectParameterKeys[1].parameter ==
          TimingColouriseEffectParameter::EmissiveLevel);
    CHECK(merged.effectParameterKeys[1].position == Approx(0.6F));
    CHECK(merged.effectParameterKeys[1].value == Approx(4.0F));

    CHECK(mergedEffects[1].id == "colourise-window");
    CHECK(mergedEffects[1].colouriseEnabled);
    CHECK_FALSE(mergedEffects[1].emissiveEnabled);
    CHECK(mergedEffects[1].boundsEdited);
    CHECK(mergedEffects[2].id == "emissive-window");
    CHECK_FALSE(mergedEffects[2].colouriseEnabled);
    CHECK(mergedEffects[2].emissiveEnabled);
    CHECK(mergedEffects[2].emissiveLevel == Approx(1.5F));
    CHECK(mergedEffects[3].id == "colourise-tolerance");
    CHECK_FALSE(mergedEffects[3].emissiveEnabled);
    CHECK(mergedEffects[4].id == "emissive-tolerance");
    CHECK_FALSE(mergedEffects[4].colouriseEnabled);
    CHECK(mergedEffects[4].emissiveEnabled);
    CHECK(mergedEffects[5].id == "colourise-field");
    CHECK_FALSE(mergedEffects[5].emissiveEnabled);
    CHECK(mergedEffects[6].id == "emissive-field");
    CHECK_FALSE(mergedEffects[6].colouriseEnabled);
    CHECK(mergedEffects[7].id == "colourise-enabled");
    CHECK_FALSE(mergedEffects[7].emissiveEnabled);
    CHECK(mergedEffects[8].id == "emissive-enabled-mismatch");
    CHECK_FALSE(mergedEffects[8].enabled);
    CHECK(mergedEffects[8].emissiveEnabled);
    CHECK_FALSE(mergedEffects[8].colouriseEnabled);
    CHECK(mergedEffects[9].id == "colourise-both-disabled");
    CHECK_FALSE(mergedEffects[9].enabled);
    CHECK_FALSE(mergedEffects[9].emissiveEnabled);
    CHECK(mergedEffects[10].id == "emissive-both-disabled");
    CHECK_FALSE(mergedEffects[10].enabled);
    CHECK_FALSE(mergedEffects[10].colouriseEnabled);
    CHECK(mergedEffects[11].id == "colourise-bounds");
    CHECK_FALSE(mergedEffects[11].emissiveEnabled);
    CHECK(mergedEffects[12].id == "emissive-bounds");
    CHECK_FALSE(mergedEffects[12].colouriseEnabled);
    CHECK(mergedEffects[12].baseBounds.upper == Approx(0.9F));
    CHECK(mergedEffects[13].id == "colourise-bounds-keys");
    CHECK_FALSE(mergedEffects[13].emissiveEnabled);
    CHECK(mergedEffects[14].id == "emissive-bounds-keys");
    CHECK_FALSE(mergedEffects[14].colouriseEnabled);
    const auto& firstShared = mergedEffects[15];
    CHECK(firstShared.id == "colourise-first");
    CHECK(firstShared.colouriseEnabled);
    CHECK(firstShared.emissiveEnabled);
    CHECK(firstShared.emissiveLevel == Approx(1.75F));
    CHECK(mergedEffects[16].id == "colourise-second");
    CHECK(mergedEffects[16].colouriseEnabled);
    CHECK_FALSE(mergedEffects[16].emissiveEnabled);

    // Mixed document: one effect loses its aspect flags (legacy) while
    // the identical-in-every-way pair keeps them. The presence gate runs
    // the merge, but aspect-authored features are never eligible, so the
    // flagged pair stays separate.
    auto mixed = saved;
    mixed.at("timing_take_states")[0]
        .at("timing_effects")[2]
        .erase("colourise_enabled");
    mixed.at("timing_take_states")[0]
        .at("timing_effects")[2]
        .erase("emissive_enabled");
    TemporaryTimingColouriseFile mixedFile{
        "invisible_places_mixed_aspect_timing_effects.ipproj"};
    {
        std::ofstream output{mixedFile.path};
        REQUIRE(output.is_open());
        output << mixed.dump(2);
    }
    const auto mixedLoaded =
        invisible_places::serialization::LoadProjectDocument(
            mixedFile.path,
            &error);
    REQUIRE(mixedLoaded.has_value());
    const auto& mixedEffects =
        mixedLoaded->timingTakeStates.front().colouriseEffects;
    REQUIRE(mixedEffects.size() == 19U);
    CHECK(mixedEffects[0].id == "colourise-merge");
    CHECK(mixedEffects[0].colouriseEnabled);
    CHECK_FALSE(mixedEffects[0].emissiveEnabled);
    CHECK_FALSE(mixedEffects[0].boundsEdited);
    CHECK(mixedEffects[1].id == "emissive-merge");
    // Only the kind-only effect loads detached from Global bounds.
    CHECK(mixedEffects[2].id == "colourise-window");
    CHECK(mixedEffects[2].boundsEdited);

    // A kind this build does not understand is skipped rather than being
    // guessed at, so its would-be partner stays a single-aspect feature.
    auto unknownKind = legacy;
    unknownKind.at("timing_take_states")[0]
        .at("timing_effects")[1]["kind"] = "future_effect";
    TemporaryTimingColouriseFile unknownKindFile{
        "invisible_places_unknown_timing_effect_kind.ipproj"};
    {
        std::ofstream output{unknownKindFile.path};
        REQUIRE(output.is_open());
        output << unknownKind.dump(2);
    }
    const auto unknownLoaded =
        invisible_places::serialization::LoadProjectDocument(
            unknownKindFile.path,
            &error);
    REQUIRE(unknownLoaded.has_value());
    const auto& unknownEffects =
        unknownLoaded->timingTakeStates.front().colouriseEffects;
    REQUIRE(unknownEffects.size() == 17U);
    CHECK(unknownEffects[0].id == "colourise-merge");
    CHECK(unknownEffects[0].colouriseEnabled);
    CHECK_FALSE(unknownEffects[0].emissiveEnabled);
    CHECK(std::none_of(
        unknownEffects.begin(),
        unknownEffects.end(),
        [](const TimingColouriseEffect& effect) {
            return effect.id == "emissive-merge";
        }));
}

TEST_CASE(
    "Legacy aspect merging respects renderer slot capacity and eligibility",
    "[timing][colourise][emissive][migration][capacity]") {
    const auto makePair = [](std::string field)
        -> std::pair<TimingColouriseEffect, TimingColouriseEffect> {
        TimingColouriseEffect colourise;
        colourise.id = "c-" + field;
        colourise.field.scalarFieldName = field;
        TimingColouriseEffect emissive;
        emissive.id = "e-" + field;
        emissive.colouriseEnabled = false;
        emissive.emissiveEnabled = true;
        emissive.field.scalarFieldName = field;
        emissive.emissiveLevel = 2.0F;
        return {std::move(colourise), std::move(emissive)};
    };
    const auto makeEffects = [&] {
        // Nine single-aspect enabled effects, all active across the full
        // [0, 1] window: four exactly mergeable pairs plus one extra
        // colourise effect. That is nine concurrent renderer slots.
        std::vector<TimingColouriseEffect> effects;
        for (const auto* field : {"F1", "F2", "F3", "F4"}) {
            auto [colourise, emissive] = makePair(field);
            effects.push_back(std::move(colourise));
            effects.push_back(std::move(emissive));
        }
        TimingColouriseEffect extra;
        extra.id = "c-extra";
        extra.field.scalarFieldName = "F-Extra";
        effects.push_back(std::move(extra));
        return effects;
    };

    // Nine concurrent slots exceed the default eight-slot renderer
    // capacity somewhere (everywhere) inside every pair's window, so
    // merging is refused wholesale: cap selection is position dependent
    // and a merged pair would change which slots the renderer keeps.
    auto overCapacity = makeEffects();
    CHECK(invisible_places::timing::MergeLegacyTimingEffectAspects(
              &overCapacity) == 0U);
    CHECK(overCapacity.size() == 9U);

    // The same authoring merges once the capacity accommodates every
    // concurrently active slot.
    auto withinCapacity = makeEffects();
    CHECK(invisible_places::timing::MergeLegacyTimingEffectAspects(
              &withinCapacity,
              nullptr,
              16U) == 4U);
    REQUIRE(withinCapacity.size() == 5U);
    CHECK(withinCapacity[0].id == "c-F1");
    CHECK(withinCapacity[0].colouriseEnabled);
    CHECK(withinCapacity[0].emissiveEnabled);
    CHECK(withinCapacity[0].emissiveLevel == Approx(2.0F));
    CHECK(withinCapacity[4].id == "c-extra");
    CHECK_FALSE(withinCapacity[4].emissiveEnabled);

    // An eligibility mask excludes aspect-authored effects: the masked
    // pair stays separate while every eligible pair still merges.
    auto masked = makeEffects();
    std::vector<bool> eligibility(masked.size(), true);
    eligibility[0] = false;
    eligibility[1] = false;
    CHECK(invisible_places::timing::MergeLegacyTimingEffectAspects(
              &masked,
              &eligibility,
              16U) == 3U);
    REQUIRE(masked.size() == 6U);
    CHECK(masked[0].id == "c-F1");
    CHECK_FALSE(masked[0].emissiveEnabled);
    CHECK(masked[1].id == "e-F1");
    CHECK_FALSE(masked[1].colouriseEnabled);
    CHECK(masked[2].id == "c-F2");
    CHECK(masked[2].emissiveEnabled);
}

TEST_CASE(
    "Dual-aspect Visual Features round-trip aspect flags and bounds memory",
    "[timing][colourise][emissive][bounds][project][serialization]") {
    using invisible_places::timing::TimingColouriseFieldBoundsMemory;
    using invisible_places::timing::TimingColouriseFieldSource;

    invisible_places::serialization::ProjectDocument document;
    invisible_places::timing::TimingTakeSceneState state{
        .takeId = "timing-take-dual",
        .sceneGroupName = "Site A",
    };

    TimingColouriseEffect effect;
    effect.id = "visual-feature-1";
    effect.name = "Mineral heat";
    // Colourise stays default-enabled, making this a dual-aspect feature.
    effect.emissiveEnabled = true;
    effect.field.scalarFieldName = "Mineral";
    effect.basePalette = Solid({0.2F, 0.6F, 0.9F}, 0.7F);
    effect.emissiveLevel = 3.0F;
    effect.boundsEdited = true;
    effect.boundsAdoptedGlobalRevision = 4U;
    effect.fieldBoundsMemory.push_back(TimingColouriseFieldBoundsMemory{
        .selector = {.source = TimingColouriseFieldSource::Scalar,
                     .scalarFieldName = "Heat"},
        .bounds = {.lower = 0.15F, .upper = 0.85F, .edgeFade = 0.05F},
        .boundsKeyMode = TimingColouriseBoundsKeyMode::CentreSpread,
        .boundsParameterKeys = {{.parameter =
                                     TimingColouriseBoundsParameter::Centre,
                                 .position = 0.5F,
                                 .value = 0.4F}},
        .boundsKeys = {{.position = 0.25F,
                        .bounds = {.lower = 0.2F,
                                   .upper = 0.6F,
                                   .edgeFade = 0.1F}}},
        .edited = true,
        .adoptedGlobalRevision = 3U,
    });
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::PalettePhase,
                    0.2F,
                    0.5F));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::AmountOverride,
                    0.4F,
                    0.8F));
    REQUIRE(invisible_places::timing::
                AddOrUpdateTimingColouriseEffectParameterKey(
                    &effect,
                    TimingColouriseEffectParameter::EmissiveLevel,
                    0.6F,
                    2.0F));
    state.colouriseEffects.push_back(effect);
    document.timingTakeStates.push_back(state);

    TemporaryTimingColouriseFile file{
        "invisible_places_dual_aspect_round_trip.ipproj"};
    std::string error;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(
        document,
        file.path,
        &error));

    nlohmann::json saved;
    {
        std::ifstream input{file.path};
        REQUIRE(input.is_open());
        saved = nlohmann::json::parse(input);
    }
    CHECK(
        saved.at("schema_version") ==
        invisible_places::serialization::kProjectDocumentSchemaVersion);
    const auto& savedEffect =
        saved.at("timing_take_states")[0].at("timing_effects")[0];
    // Dual-aspect features degrade to plain colourise for legacy readers.
    CHECK(savedEffect.at("kind") == "colourise");
    CHECK(savedEffect.at("colourise_enabled") == true);
    CHECK(savedEffect.at("emissive_enabled") == true);
    CHECK(savedEffect.at("emissive_level") == Approx(3.0F));
    REQUIRE(savedEffect.at("effect_parameter_keys").size() == 3U);
    CHECK(savedEffect.at("bounds_edited") == true);
    CHECK(savedEffect.at("bounds_adopted_global_revision") == 4U);
    REQUIRE(savedEffect.at("field_bounds_memory").size() == 1U);
    const auto& savedMemory = savedEffect.at("field_bounds_memory")[0];
    CHECK(savedMemory.at("field").at("scalar_field_name") == "Heat");
    CHECK(savedMemory.at("bounds").at("lower") == Approx(0.15F));
    CHECK(savedMemory.at("bounds_key_mode") == "centre_spread");
    REQUIRE(savedMemory.at("bounds_parameter_keys").size() == 1U);
    REQUIRE(savedMemory.at("bounds_keys").size() == 1U);
    CHECK(savedMemory.at("edited") == true);
    CHECK(savedMemory.at("adopted_global_revision") == 3U);

    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(
            file.path,
            &error);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->timingTakeStates.size() == 1U);
    REQUIRE(
        loaded->timingTakeStates.front().colouriseEffects.size() == 1U);
    const auto& loadedEffect =
        loaded->timingTakeStates.front().colouriseEffects.front();
    CHECK(loadedEffect.colouriseEnabled);
    CHECK(loadedEffect.emissiveEnabled);
    CHECK(loadedEffect.emissiveLevel == Approx(3.0F));
    // Every effect parameter is live on a dual-aspect feature.
    for (const auto parameter : {
             TimingColouriseEffectParameter::PalettePhase,
             TimingColouriseEffectParameter::AmountOverride,
             TimingColouriseEffectParameter::EmissiveLevel,
         }) {
        CHECK(invisible_places::timing::TimingEffectParameterIsSupported(
            loadedEffect,
            parameter));
    }
    REQUIRE(loadedEffect.effectParameterKeys.size() == 3U);
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterKeyCountAtPosition(
                  loadedEffect,
                  TimingColouriseEffectParameter::PalettePhase,
                  0.2F) == 1U);
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterKeyCountAtPosition(
                  loadedEffect,
                  TimingColouriseEffectParameter::AmountOverride,
                  0.4F) == 1U);
    CHECK(invisible_places::timing::
              TimingColouriseEffectParameterKeyCountAtPosition(
                  loadedEffect,
                  TimingColouriseEffectParameter::EmissiveLevel,
                  0.6F) == 1U);
    CHECK(loadedEffect.boundsEdited);
    CHECK(loadedEffect.boundsAdoptedGlobalRevision == 4U);
    REQUIRE(loadedEffect.fieldBoundsMemory.size() == 1U);
    const auto& loadedMemory = loadedEffect.fieldBoundsMemory.front();
    CHECK(loadedMemory.selector.source ==
          TimingColouriseFieldSource::Scalar);
    CHECK(loadedMemory.selector.scalarFieldName == "Heat");
    CHECK(loadedMemory.bounds.lower == Approx(0.15F));
    CHECK(loadedMemory.bounds.upper == Approx(0.85F));
    CHECK(loadedMemory.bounds.edgeFade == Approx(0.05F));
    CHECK(loadedMemory.boundsKeyMode ==
          TimingColouriseBoundsKeyMode::CentreSpread);
    REQUIRE(loadedMemory.boundsParameterKeys.size() == 1U);
    CHECK(loadedMemory.boundsParameterKeys.front().parameter ==
          TimingColouriseBoundsParameter::Centre);
    CHECK(loadedMemory.boundsParameterKeys.front().value ==
          Approx(0.4F));
    REQUIRE(loadedMemory.boundsKeys.size() == 1U);
    CHECK(loadedMemory.boundsKeys.front().bounds.upper == Approx(0.6F));
    CHECK(loadedMemory.edited);
    CHECK(loadedMemory.adoptedGlobalRevision == 3U);
}

TEST_CASE(
    "Visual Feature field selection stashes and restores bounds authoring",
    "[timing][colourise][bounds][field-memory]") {
    using invisible_places::timing::TimingColouriseFieldSelector;
    using invisible_places::timing::TimingColouriseFieldSource;
    using invisible_places::timing::TimingScalarBoundsStore;

    TimingColouriseEffect effect;
    effect.field.scalarFieldName = "SurfaceRoughness";
    effect.baseBounds = {.lower = -2.0F, .upper = 4.0F, .edgeFade = 0.2F};
    effect.boundsKeyMode = TimingColouriseBoundsKeyMode::CentreSpread;
    effect.boundsParameterKeys = {
        {.parameter = TimingColouriseBoundsParameter::Centre,
         .position = 0.5F,
         .value = 1.25F},
    };
    effect.boundsKeys = {
        {.position = 0.75F,
         .bounds = {.lower = -1.0F, .upper = 2.0F, .edgeFade = 0.1F}},
    };
    effect.boundsEdited = true;
    effect.boundsAdoptedGlobalRevision = 2U;

    const TimingColouriseFieldSelector heat{
        .source = TimingColouriseFieldSource::Scalar,
        .scalarFieldName = "Heat",
    };
    const TimingColouriseBounds fallback{
        .lower = 0.0F,
        .upper = 10.0F,
        .edgeFade = 0.1F,
    };
    invisible_places::timing::ApplyTimingColouriseFieldSelection(
        &effect,
        heat,
        fallback,
        nullptr);
    CHECK(effect.field.scalarFieldName == "Heat");
    CHECK(effect.baseBounds.lower == Approx(0.0F));
    CHECK(effect.baseBounds.upper == Approx(10.0F));
    CHECK(effect.boundsKeyMode ==
          TimingColouriseBoundsKeyMode::LowerUpper);
    CHECK(effect.boundsParameterKeys.empty());
    CHECK(effect.boundsKeys.empty());
    CHECK_FALSE(effect.boundsEdited);
    CHECK(effect.boundsAdoptedGlobalRevision == 0U);
    REQUIRE(effect.fieldBoundsMemory.size() == 1U);
    CHECK(effect.fieldBoundsMemory.front().selector.scalarFieldName ==
          "SurfaceRoughness");
    CHECK(effect.fieldBoundsMemory.front().edited);

    // Re-selecting the active selector is a no-op.
    invisible_places::timing::ApplyTimingColouriseFieldSelection(
        &effect,
        heat,
        fallback,
        nullptr);
    CHECK(effect.fieldBoundsMemory.size() == 1U);

    // A first visit with a shared store adopts its Global bounds instead
    // of the fallback and remembers the adopted revision.
    TimingScalarBoundsStore store;
    store.selector = TimingColouriseFieldSelector{
        .source = TimingColouriseFieldSource::Scalar,
        .scalarFieldName = "Moss",
    };
    store.globalBounds = {.lower = 0.3F, .upper = 0.7F, .edgeFade = 0.15F};
    store.revision = 5U;
    invisible_places::timing::ApplyTimingColouriseFieldSelection(
        &effect,
        store.selector,
        fallback,
        &store);
    CHECK(effect.field.scalarFieldName == "Moss");
    CHECK(effect.baseBounds.lower == Approx(0.3F));
    CHECK(effect.baseBounds.upper == Approx(0.7F));
    CHECK_FALSE(effect.boundsEdited);
    CHECK(effect.boundsAdoptedGlobalRevision == 5U);

    // Returning to the original selector restores its complete bounds
    // authoring: base bounds, key mode, both key vectors, edited state.
    invisible_places::timing::ApplyTimingColouriseFieldSelection(
        &effect,
        TimingColouriseFieldSelector{
            .source = TimingColouriseFieldSource::Scalar,
            .scalarFieldName = "SurfaceRoughness",
        },
        fallback,
        nullptr);
    CHECK(effect.field.scalarFieldName == "SurfaceRoughness");
    CHECK(effect.baseBounds.lower == Approx(-2.0F));
    CHECK(effect.baseBounds.upper == Approx(4.0F));
    CHECK(effect.baseBounds.edgeFade == Approx(0.2F));
    CHECK(effect.boundsKeyMode ==
          TimingColouriseBoundsKeyMode::CentreSpread);
    REQUIRE(effect.boundsParameterKeys.size() == 1U);
    CHECK(effect.boundsParameterKeys.front().parameter ==
          TimingColouriseBoundsParameter::Centre);
    CHECK(effect.boundsParameterKeys.front().value == Approx(1.25F));
    REQUIRE(effect.boundsKeys.size() == 1U);
    CHECK(effect.boundsKeys.front().bounds.upper == Approx(2.0F));
    CHECK(effect.boundsEdited);
    CHECK(effect.boundsAdoptedGlobalRevision == 2U);
    // Every visited selector keeps a remembered entry.
    CHECK(effect.fieldBoundsMemory.size() == 3U);
}

TEST_CASE(
    "Unedited Visual Feature bounds follow the latest Global edit",
    "[timing][colourise][bounds][field-memory]") {
    using invisible_places::timing::TimingScalarBoundsStore;

    std::vector<TimingScalarBoundsStore> stores;
    TimingColouriseEffect editor;
    editor.field.scalarFieldName = "Heat";
    editor.baseBounds = {.lower = 0.1F, .upper = 0.9F, .edgeFade = 0.1F};
    TimingColouriseEffect follower;
    follower.field.scalarFieldName = "Heat";
    TimingColouriseEffect otherField;
    otherField.field.scalarFieldName = "Moss";

    invisible_places::timing::RecordTimingScalarBoundsEdit(
        &stores,
        &editor);
    CHECK(editor.boundsEdited);
    CHECK(editor.boundsAdoptedGlobalRevision == 1U);
    REQUIRE(stores.size() == 1U);
    CHECK(stores.front().selector.scalarFieldName == "Heat");
    CHECK(stores.front().revision == 1U);
    CHECK(stores.front().globalBounds.lower == Approx(0.1F));

    CHECK(invisible_places::timing::RefreshTimingColouriseBoundsFromGlobal(
        &follower,
        stores));
    CHECK(follower.baseBounds.lower == Approx(0.1F));
    CHECK(follower.baseBounds.upper == Approx(0.9F));
    CHECK(follower.boundsAdoptedGlobalRevision == 1U);
    CHECK_FALSE(follower.boundsEdited);
    // Already current: nothing further to adopt.
    CHECK_FALSE(
        invisible_places::timing::RefreshTimingColouriseBoundsFromGlobal(
            &follower,
            stores));
    // A feature bound to another selector is untouched.
    CHECK_FALSE(
        invisible_places::timing::RefreshTimingColouriseBoundsFromGlobal(
            &otherField,
            stores));

    editor.baseBounds = {.lower = 0.2F, .upper = 0.6F, .edgeFade = 0.1F};
    invisible_places::timing::RecordTimingScalarBoundsEdit(
        &stores,
        &editor);
    REQUIRE(stores.size() == 1U);
    CHECK(stores.front().revision == 2U);
    CHECK(invisible_places::timing::RefreshTimingColouriseBoundsFromGlobal(
        &follower,
        stores));
    CHECK(follower.baseBounds.upper == Approx(0.6F));

    // A locally edited feature stays detached from later Global edits.
    follower.boundsEdited = true;
    editor.baseBounds = {.lower = 0.05F, .upper = 0.4F, .edgeFade = 0.1F};
    invisible_places::timing::RecordTimingScalarBoundsEdit(
        &stores,
        &editor);
    CHECK(stores.front().revision == 3U);
    CHECK_FALSE(
        invisible_places::timing::RefreshTimingColouriseBoundsFromGlobal(
            &follower,
            stores));
    CHECK(follower.baseBounds.upper == Approx(0.6F));

    auto* mutableStore =
        invisible_places::timing::FindTimingScalarBoundsStore(
            &stores,
            editor.field);
    REQUIRE(mutableStore != nullptr);
    const auto& constStores = stores;
    const auto* constStore =
        invisible_places::timing::FindTimingScalarBoundsStore(
            constStores,
            editor.field);
    CHECK(constStore == mutableStore);
    CHECK(invisible_places::timing::FindTimingScalarBoundsStore(
              constStores,
              otherField.field) == nullptr);
}

TEST_CASE(
    "Timing Colourise activation range migration preserves authored effects",
    "[timing][colourise][activation][project][serialization][migration]") {
    invisible_places::serialization::ProjectDocument document;
    invisible_places::timing::TimingTakeSceneState state{
        .takeId = "timing-take-many",
        .sceneGroupName = "Site A",
    };
    for (std::size_t index = 0U; index < 6U; ++index) {
        TimingColouriseEffect effect;
        effect.id = "effect-" + std::to_string(index);
        if (index == 0U) {
            effect.activationRange = {.start = 0.2F, .end = 0.8F};
        }
        state.colouriseEffects.push_back(std::move(effect));
    }
    document.timingTakeStates.push_back(std::move(state));

    TemporaryTimingColouriseFile currentFile{
        "invisible_places_timing_colourise_activation_current.ipproj"};
    std::string error;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(
        document,
        currentFile.path,
        &error));

    nlohmann::json saved;
    {
        std::ifstream input{currentFile.path};
        REQUIRE(input.is_open());
        saved = nlohmann::json::parse(input);
    }
    REQUIRE(saved.at("timing_take_states")[0]
                .at("colourise_effects")
                .size() == 6U);
    REQUIRE(saved.at("timing_take_states")[0]
                .at("timing_effects")
                .size() == 6U);
    const auto current =
        invisible_places::serialization::LoadProjectDocument(
            currentFile.path,
            &error);
    REQUIRE(current.has_value());
    REQUIRE(current->timingTakeStates.size() == 1U);
    REQUIRE(current->timingTakeStates.front().colouriseEffects.size() ==
            6U);
    CHECK(current->timingTakeStates.front()
              .colouriseEffects.front()
              .activationRange.start == Approx(0.2F));
    CHECK(current->timingTakeStates.front()
              .colouriseEffects.front()
              .activationRange.end == Approx(0.8F));

    auto legacy = saved;
    legacy["schema_version"] = 58U;
    auto& legacyState = legacy.at("timing_take_states")[0];
    legacyState.erase("timing_effects");
    legacyState.erase("timing_effect_sequence");
    legacyState.at("colourise_effects")[0]
        .erase("activation_range");
    TemporaryTimingColouriseFile legacyFile{
        "invisible_places_timing_colourise_activation_legacy.ipproj"};
    {
        std::ofstream output{legacyFile.path};
        REQUIRE(output.is_open());
        output << legacy.dump(2);
    }
    const auto migrated =
        invisible_places::serialization::LoadProjectDocument(
            legacyFile.path,
            &error);
    REQUIRE(migrated.has_value());
    REQUIRE(migrated->timingTakeStates.size() == 1U);
    REQUIRE(migrated->timingTakeStates.front()
                .colouriseEffects.size() == 6U);
    CHECK(migrated->timingTakeStates.front()
              .colouriseEffects.front()
              .activationRange.start == Approx(0.0F));
    CHECK(migrated->timingTakeStates.front()
              .colouriseEffects.front()
              .activationRange.end == Approx(1.0F));

    auto malformed = saved;
    malformed["schema_version"] = 58U;
    auto& malformedState = malformed.at("timing_take_states")[0];
    malformedState.erase("timing_effects");
    malformedState.erase("timing_effect_sequence");
    malformedState.at("colourise_effects")[0]["activation_range"] = {
        {"start", "not-a-number"},
        {"end", 0.6F},
    };
    malformedState.at("colourise_effects")[1]["activation_range"] = {
        {"start", 0.9F},
        {"end", 0.3F},
    };
    TemporaryTimingColouriseFile malformedFile{
        "invisible_places_timing_colourise_activation_malformed.ipproj"};
    {
        std::ofstream output{malformedFile.path};
        REQUIRE(output.is_open());
        output << malformed.dump(2);
    }
    const auto repaired =
        invisible_places::serialization::LoadProjectDocument(
            malformedFile.path,
            &error);
    REQUIRE(repaired.has_value());
    const auto& repairedEffects =
        repaired->timingTakeStates.front().colouriseEffects;
    REQUIRE(repairedEffects.size() == 6U);
    CHECK(repairedEffects[0].activationRange.start == Approx(0.0F));
    CHECK(repairedEffects[0].activationRange.end == Approx(0.6F));
    CHECK(repairedEffects[1].activationRange.start == Approx(0.3F));
    CHECK(repairedEffects[1].activationRange.end == Approx(0.9F));
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
