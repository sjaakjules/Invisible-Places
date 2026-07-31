#pragma once

#include "water/WaterFlow.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::timing {

inline constexpr std::string_view kAuthoredTimingTakeId = "authored-timing";
inline constexpr std::string_view kAuthoredTimingTakeName = "Authored Timing";
inline constexpr std::size_t kMaximumTimingColouriseEffects = 5U;
inline constexpr std::size_t kMaximumTimingColourisePaletteStops = 8U;
inline constexpr std::size_t kTimingColouriseLutSampleCount = 64U;
inline constexpr float kTimingColouriseKeyTolerance = 1.0e-4F;

enum class TimingColouriseFieldSource : std::uint8_t {
    Scalar = 0,
    NormalX,
    NormalY,
    NormalZ,
};

struct TimingColouriseFieldSelector {
    TimingColouriseFieldSource source = TimingColouriseFieldSource::Scalar;
    // Stable, exact field name used by each point-cloud layer. Synthetic
    // normal components leave this empty.
    std::string scalarFieldName;

    friend auto operator<=>(
        const TimingColouriseFieldSelector&,
        const TimingColouriseFieldSelector&) = default;
};

struct TimingColourisePaletteStop {
    // Stable within one palette/effect. Animation tracks refer to this id so
    // sorting or moving stops never exchanges their authored properties.
    std::string id;
    float position = 0.0F;
    std::array<float, 3> colour{1.0F, 1.0F, 1.0F};
    // Mix amount for the sampled colour. This never changes point opacity.
    float colouriseAmount = 1.0F;
};

struct TimingColourisePalette {
    std::vector<TimingColourisePaletteStop> stops{
        TimingColourisePaletteStop{}};
};

// Whole-palette snapshots are retained for projects authored before
// independent stop-property keying. New effects use StopParameters: their
// stop topology is static and each stop property owns its own curve.
enum class TimingColourisePaletteKeyModel : std::uint8_t {
    LegacySnapshots = 0,
    StopParameters,
};

enum class TimingColourisePaletteSourceKind : std::uint8_t {
    Custom = 0,
    Preset,
    Saved,
};

enum class TimingColourisePaletteStopParameter : std::uint8_t {
    Position = 0,
    Colour,
    ColouriseAmount,
};

using TimingColouriseLut =
    std::array<std::array<float, 4>, kTimingColouriseLutSampleCount>;

struct TimingColouriseBounds {
    float lower = 0.0F;
    float upper = 1.0F;
    // Fraction of the selected span faded at each edge, in [0, 0.5].
    float edgeFade = 0.0F;
};

// An effect chooses exactly one bounds parameterisation. This makes the two
// geometric controls unambiguous while still allowing either control to be
// keyed independently.
enum class TimingColouriseBoundsKeyMode : std::uint8_t {
    LowerUpper = 0,
    CentreSpread,
    LowerSpread,
    UpperSpread,
};

enum class TimingColouriseBoundsParameter : std::uint8_t {
    Lower = 0,
    Upper,
    Centre,
    Spread,
    EdgeFade,
};

enum class TimingColouriseBoundsHandle : std::uint8_t {
    Lower = 0,
    Upper,
    Centre,
};

struct TimingColouriseBoundsHandleEdit {
    TimingColouriseBounds bounds{};
    std::array<TimingColouriseBoundsParameter, 2> parameters{
        TimingColouriseBoundsParameter::EdgeFade,
        TimingColouriseBoundsParameter::EdgeFade};
    std::size_t parameterCount = 0U;
};

struct TimingColouriseBoundsParameterKey {
    TimingColouriseBoundsParameter parameter =
        TimingColouriseBoundsParameter::Lower;
    float position = 0.0F;
    float value = 0.0F;
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColourisePaletteKey {
    float position = 0.0F;
    TimingColourisePalette palette{};
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColourisePaletteStopParameterKey {
    std::string stopId;
    TimingColourisePaletteStopParameter parameter =
        TimingColourisePaletteStopParameter::Position;
    // Normalized animation position, not the stop's position in the palette.
    float position = 0.0F;
    // Position and ColouriseAmount use scalarValue; Colour uses colourValue.
    float scalarValue = 0.0F;
    std::array<float, 3> colourValue{1.0F, 1.0F, 1.0F};
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColouriseBoundsKey {
    float position = 0.0F;
    TimingColouriseBounds bounds{};
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColouriseEffect {
    std::string id;
    std::string name = "Colourise";
    bool enabled = true;
    TimingColouriseFieldSelector field{};
    TimingColourisePalette basePalette{};
    TimingColouriseBounds baseBounds{};
    TimingColourisePaletteKeyModel paletteKeyModel =
        TimingColourisePaletteKeyModel::StopParameters;
    TimingColourisePaletteSourceKind paletteSourceKind =
        TimingColourisePaletteSourceKind::Custom;
    // Provenance is authoring metadata only. Evaluation always uses the
    // copied palette stored on this effect, never a live library reference.
    std::string paletteSourceId;
    std::string paletteSourceName;
    bool paletteEdited = false;
    // Legacy whole-palette snapshots remain active only in LegacySnapshots
    // mode. New authoring writes the independent tracks below.
    std::vector<TimingColourisePaletteKey> paletteKeys;
    std::vector<TimingColourisePaletteStopParameterKey>
        paletteStopParameterKeys;
    TimingColouriseBoundsKeyMode boundsKeyMode =
        TimingColouriseBoundsKeyMode::LowerUpper;
    // New independently keyed bounds controls. Legacy whole-bounds keys remain
    // below so existing projects retain their exact animation and edge fade.
    // Parameter keys override the corresponding evaluated legacy/base value.
    std::vector<TimingColouriseBoundsParameterKey> boundsParameterKeys;
    // Legacy v1 snapshot keys. New authoring should use boundsParameterKeys.
    std::vector<TimingColouriseBoundsKey> boundsKeys;
};

struct TimingColourisePaletteDefinition {
    std::string id;
    std::string name = "Palette";
    TimingColourisePalette palette{};
};

struct TimingTakeDefinition {
    std::string id;
    std::string name = "Timing Take";
};

struct TimingTakeSceneState {
    std::string takeId = std::string{kAuthoredTimingTakeId};
    std::string sceneGroupName = "Default";
    std::vector<invisible_places::water::WaterFeatureTimingRun>
        waterFeatureTimingRuns;
    // List order is visual priority: index zero is the top layer and is
    // applied last.
    std::vector<TimingColouriseEffect> colouriseEffects;
    std::uint32_t waterFeatureTimingRunSequence = 1U;
    std::uint32_t colouriseEffectSequence = 1U;
};

struct TimingColouriseLayerSample {
    std::array<float, 3> colour{1.0F, 1.0F, 1.0F};
    float colouriseAmount = 0.0F;
};

[[nodiscard]] std::string NormalizeTimingTakeId(std::string_view takeId);
[[nodiscard]] TimingTakeDefinition AuthoredTimingTakeDefinition();

[[nodiscard]] TimingColourisePalette SanitizeTimingColourisePalette(
    TimingColourisePalette palette);
[[nodiscard]] std::string AllocateTimingColourisePaletteStopId(
    const TimingColourisePalette& palette);
[[nodiscard]] TimingColouriseBounds SanitizeTimingColouriseBounds(
    TimingColouriseBounds bounds);
[[nodiscard]] bool TimingColouriseBoundsParameterIsAllowed(
    TimingColouriseBoundsKeyMode mode,
    TimingColouriseBoundsParameter parameter);
[[nodiscard]] std::array<TimingColouriseBoundsParameter, 2>
TimingColouriseBoundsParametersForMode(TimingColouriseBoundsKeyMode mode);
[[nodiscard]] float TimingColouriseBoundsParameterValue(
    const TimingColouriseBounds& bounds,
    TimingColouriseBoundsParameter parameter);
// Resolves a direct histogram-handle drag into the coordinate track(s) owned
// by the selected keying mode. Translation handles preserve spacing; a
// Centre+Spacing endpoint mirrors its partner around the fixed centre.
[[nodiscard]] std::optional<TimingColouriseBoundsHandleEdit>
ResolveTimingColouriseBoundsHandleEdit(
    TimingColouriseBoundsKeyMode mode,
    TimingColouriseBoundsHandle handle,
    TimingColouriseBounds currentBounds,
    float targetValue,
    float rangeMinimum,
    float rangeMaximum);
// Refuses a mode change while it would invalidate an existing geometric
// parameter track. Edge Fade is independent and never blocks a mode change.
[[nodiscard]] bool SetTimingColouriseBoundsKeyMode(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsKeyMode mode);
[[nodiscard]] TimingColouriseEffect SanitizeTimingColouriseEffect(
    TimingColouriseEffect effect);
[[nodiscard]] TimingColourisePaletteDefinition
SanitizeTimingColourisePaletteDefinition(
    TimingColourisePaletteDefinition definition);
[[nodiscard]] TimingTakeDefinition SanitizeTimingTakeDefinition(
    TimingTakeDefinition definition);
[[nodiscard]] TimingTakeSceneState SanitizeTimingTakeSceneState(
    TimingTakeSceneState state);

[[nodiscard]] TimingColouriseLut CompileTimingColourisePaletteLut(
    const TimingColourisePalette& palette);
[[nodiscard]] TimingColourisePalette EvaluateTimingColourisePalette(
    const TimingColouriseEffect& effect,
    float normalizedPosition);
[[nodiscard]] TimingColouriseLut EvaluateTimingColourisePaletteLut(
    const TimingColouriseEffect& effect,
    float normalizedPosition);
[[nodiscard]] TimingColouriseBounds EvaluateTimingColouriseBounds(
    const TimingColouriseEffect& effect,
    float normalizedPosition);
[[nodiscard]] TimingColouriseLayerSample SampleTimingColouriseLut(
    const TimingColouriseLut& lut,
    float normalizedFieldValue);
[[nodiscard]] float TimingColouriseBoundsMask(
    const TimingColouriseBounds& bounds,
    float fieldValue);

// Samples are supplied in list order (top first). The helper composites from
// the bottom toward index zero, so the top list item has highest priority.
[[nodiscard]] std::array<float, 3> ApplyTimingColouriseStack(
    std::array<float, 3> baseColour,
    std::span<const TimingColouriseLayerSample> samples);

void AddOrUpdateTimingColourisePaletteKey(
    TimingColouriseEffect* effect,
    float position,
    TimingColourisePalette palette,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool AddOrUpdateTimingColourisePaletteStopScalarKey(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool AddOrUpdateTimingColourisePaletteStopColourKey(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    float position,
    std::array<float, 3> colour,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
void AddOrUpdateTimingColouriseBoundsKey(
    TimingColouriseEffect* effect,
    float position,
    TimingColouriseBounds bounds,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool AddOrUpdateTimingColouriseBoundsParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool MoveTimingColourisePaletteKey(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool CanMoveTimingColourisePaletteKeysAtPosition(
    const TimingColouriseEffect& effect,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool MoveTimingColouriseBoundsKey(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool MoveTimingColouriseBoundsParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] std::size_t TimingColourisePaletteKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::size_t
TimingColourisePaletteStopParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position);
[[nodiscard]] std::size_t
TimingColouriseBoundsParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::size_t TimingColouriseBoundsKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::size_t TimingColouriseEffectKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::size_t RemoveTimingColourisePaletteKeysAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::size_t
RemoveTimingColourisePaletteStopParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position);
[[nodiscard]] std::size_t RemoveTimingColouriseBoundsKeysAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::size_t
RemoveTimingColouriseBoundsParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::size_t RemoveTimingColouriseEffectKeysAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::optional<float>
PreviousTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float> NextTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::vector<float> TimingColourisePaletteKeyPositions(
    const TimingColouriseEffect& effect);
// Stop topology is static for independent property tracks. Removing a stop
// is refused while it has keys, preserving dormant animation data.
[[nodiscard]] bool CanRemoveTimingColourisePaletteStop(
    const TimingColouriseEffect& effect,
    std::string_view stopId);
[[nodiscard]] bool RemoveTimingColourisePaletteStop(
    TimingColouriseEffect* effect,
    std::string_view stopId);
[[nodiscard]] std::string TimingColourisePaletteKeyStateName(
    std::string_view paletteName,
    std::size_t orderedPosition);
[[nodiscard]] std::optional<float>
PreviousTimingColouriseBoundsKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float> NextTimingColouriseBoundsKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float>
PreviousTimingColouriseBoundsParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::optional<float>
NextTimingColouriseBoundsParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::optional<float> PreviousTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float> NextTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position);

[[nodiscard]] const TimingTakeDefinition* FindTimingTakeDefinition(
    std::span<const TimingTakeDefinition> takes,
    std::string_view takeId);
[[nodiscard]] TimingTakeDefinition* FindTimingTakeDefinition(
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId);
[[nodiscard]] const TimingTakeSceneState* FindTimingTakeSceneState(
    std::span<const TimingTakeSceneState> states,
    std::string_view takeId,
    std::string_view sceneGroupName);
[[nodiscard]] TimingTakeSceneState* FindTimingTakeSceneState(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view sceneGroupName);
[[nodiscard]] TimingTakeSceneState* EnsureTimingTakeSceneState(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view sceneGroupName);
[[nodiscard]] bool AssignWaterFeatureToTimingRun(
    TimingTakeSceneState* state,
    const invisible_places::water::WaterKeyedFeatureId& feature,
    std::uint32_t targetRunId);

[[nodiscard]] std::string AllocateTimingTakeId(
    std::span<const TimingTakeDefinition> takes,
    std::uint32_t* nextSequence);
[[nodiscard]] std::string AllocateTimingColouriseEffectId(
    std::span<const TimingColouriseEffect> effects,
    std::uint32_t* nextSequence);
[[nodiscard]] std::string AllocateTimingColourisePaletteId(
    std::span<const TimingColourisePaletteDefinition> palettes,
    std::uint32_t* nextSequence);

}  // namespace invisible_places::timing
