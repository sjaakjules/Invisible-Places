#include "InvisiblePlacesBuildConfig.hpp"
#include "serialization/ProjectDocument.hpp"
#include "timing/TimingColourise.hpp"
#include "timing/TimelineView.hpp"
#include "water/WaterFlow.hpp"
#include "water/WaterSeepagePulseField.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using Catch::Approx;
using invisible_places::water::AddOrUpdateWaterTimingKey;
using invisible_places::water::ApplyWaterTimingLevelToScenarioState;
using invisible_places::water::CompileWaterTimingScenarioKeys;
using invisible_places::water::EvaluateWaterScenarioTrack;
using invisible_places::water::EvaluateWaterTimingRun;
using invisible_places::water::SanitizeWaterTimingRun;
using invisible_places::water::WaterScenarioDefinition;
using invisible_places::water::WaterScenarioInterpolation;
using invisible_places::water::WaterScenarioState;
using invisible_places::water::WaterScenarioTrack;
using invisible_places::water::WaterTimingFeature;
using invisible_places::water::WaterTimingKey;
using invisible_places::water::WaterTimingLevelFromScenarioState;
using invisible_places::water::WaterTimingRun;
using invisible_places::water::WaterTimingRunAssignment;

WaterTimingKey Key(
    float position,
    float level,
    WaterScenarioInterpolation interpolation = WaterScenarioInterpolation::Smooth) {
    WaterTimingKey key;
    key.position = position;
    key.level = level;
    key.interpolation = interpolation;
    return key;
}

WaterTimingRun Run(
    WaterTimingFeature feature,
    std::vector<WaterTimingKey> keys,
    std::string name = "Run") {
    WaterTimingRun run;
    run.id = "timing_run_test";
    run.name = std::move(name);
    run.feature = feature;
    run.keys = std::move(keys);
    return run;
}

struct TemporaryTimingFile {
    explicit TemporaryTimingFile(std::string filename)
        : path(std::filesystem::temp_directory_path() / std::move(filename)) {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }

    ~TemporaryTimingFile() {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }

    std::filesystem::path path;
};

}  // namespace

TEST_CASE(
    "Timeline view ranges default full and map zoomed feature-run positions",
    "[water][timing][timeline-view]") {
    using invisible_places::timing::SanitizeTimelineViewRange;
    using invisible_places::timing::TimelinePositionIsInView;
    using invisible_places::timing::TimelinePositionToViewFraction;
    using invisible_places::timing::TimelineViewFractionToPosition;
    using invisible_places::timing::TimelineViewRange;
    using invisible_places::timing::TimelineViewRangeIsFull;

    const TimelineViewRange full;
    CHECK(TimelineViewRangeIsFull(full));
    CHECK(TimelinePositionToViewFraction(full, 0.35F) ==
          Approx(0.35F));

    const auto zoomed = SanitizeTimelineViewRange({0.75F, 0.25F});
    CHECK(zoomed.start == Approx(0.25F));
    CHECK(zoomed.end == Approx(0.75F));
    CHECK_FALSE(TimelineViewRangeIsFull(zoomed));
    CHECK_FALSE(TimelinePositionIsInView(zoomed, 0.20F));
    CHECK(TimelinePositionIsInView(zoomed, 0.25F));
    CHECK(TimelinePositionIsInView(zoomed, 0.50F));
    CHECK(TimelinePositionIsInView(zoomed, 0.75F));
    CHECK_FALSE(TimelinePositionIsInView(zoomed, 0.80F));
    CHECK(TimelinePositionToViewFraction(zoomed, 0.25F) ==
          Approx(0.0F));
    CHECK(TimelinePositionToViewFraction(zoomed, 0.50F) ==
          Approx(0.5F));
    CHECK(TimelinePositionToViewFraction(zoomed, 0.75F) ==
          Approx(1.0F));
    CHECK(TimelineViewFractionToPosition(zoomed, 0.0F) ==
          Approx(0.25F));
    CHECK(TimelineViewFractionToPosition(zoomed, 0.5F) ==
          Approx(0.50F));
    CHECK(TimelineViewFractionToPosition(zoomed, 1.0F) ==
          Approx(0.75F));

    const auto collapsed =
        SanitizeTimelineViewRange({0.90F, 0.90F}, 0.20F);
    CHECK(collapsed.start == Approx(0.80F));
    CHECK(collapsed.end == Approx(1.00F));
    const auto nonFinite = SanitizeTimelineViewRange(
        {std::numeric_limits<float>::quiet_NaN(),
         std::numeric_limits<float>::infinity()});
    CHECK(TimelineViewRangeIsFull(nonFinite));
}

TEST_CASE("Timing run evaluation holds endpoints and interpolates", "[water][timing]") {
    const auto run = Run(
        WaterTimingFeature::Rain,
        {Key(0.3F, 0.0F), Key(0.5F, 1.0F), Key(0.6F, 0.0F)});

    CHECK(EvaluateWaterTimingRun(run, 0.0F, 0.75F) == Approx(0.0F));
    CHECK(EvaluateWaterTimingRun(run, 0.3F, 0.75F) == Approx(0.0F));
    CHECK(EvaluateWaterTimingRun(run, 0.5F, 0.75F) == Approx(1.0F));
    CHECK(EvaluateWaterTimingRun(run, 0.6F, 0.75F) == Approx(0.0F));
    CHECK(EvaluateWaterTimingRun(run, 1.0F, 0.75F) == Approx(0.0F));

    // Smooth interpolation matches the scenario-track smoothstep shaping.
    const float amount = 0.25F;
    const float eased = amount * amount * (3.0F - 2.0F * amount);
    CHECK(EvaluateWaterTimingRun(run, 0.3F + amount * 0.2F, 0.75F) == Approx(eased));
}

TEST_CASE(
    "Timing Take retiming preserves every nonlinear water frame and holds its terminal value",
    "[water][timing][pan-extension]") {
    constexpr std::uint32_t sourceFrames = 120U;
    constexpr std::uint32_t destinationFrames = 180U;
    invisible_places::timing::TimingTakeSceneState state;
    invisible_places::water::WaterKeyedSettingTrack setting;
    setting.settingId = "rain-rate";
    // SmoothVelocity is a time-domain spline and therefore has exact affine
    // retime equivalence. Centripetal Catmull-Rom keeps its keys and segment
    // semantics, but its 2D time/value chord lengths intentionally make its
    // between-key shape sensitive to horizontal timeline scaling.
    setting.defaultInterpolation =
        WaterScenarioInterpolation::SmoothVelocity;
    setting.keys = {
        {.position = 0.0F,
         .value = 0.1F,
         .interpolation = WaterScenarioInterpolation::TrackDefault},
        {.position = 0.23F,
         .value = 0.9F,
         .interpolation = WaterScenarioInterpolation::TrackDefault},
        {.position = 0.61F,
         .value = 0.3F,
         .interpolation = WaterScenarioInterpolation::TrackDefault},
        {.position = 1.0F,
         .value = 0.75F,
         .interpolation = WaterScenarioInterpolation::TrackDefault},
    };
    invisible_places::water::WaterFeatureTimeline feature;
    feature.feature = {
        .kind = invisible_places::water::WaterKeyedFeatureKind::Rain,
    };
    feature.settings.push_back(setting);
    invisible_places::water::WaterFeatureTimingRun run;
    run.features.push_back(std::move(feature));
    state.waterFeatureTimingRuns.push_back(std::move(run));

    const auto originalTrack = setting;
    REQUIRE(invisible_places::timing::
                RetimeTimingTakeSceneStateNormalizedPositions(
                    &state,
                    sourceFrames,
                    destinationFrames));
    const auto& retimedTrack = state.waterFeatureTimingRuns.front()
                                   .features.front()
                                   .settings.front();
    REQUIRE(retimedTrack.keys.size() == originalTrack.keys.size());
    for (std::size_t index = 0U;
         index < originalTrack.keys.size();
         ++index) {
        CHECK(retimedTrack.keys[index].position ==
              Approx(
                  originalTrack.keys[index].position *
                  static_cast<float>(sourceFrames) /
                  static_cast<float>(destinationFrames)));
        CHECK(retimedTrack.keys[index].value ==
              Approx(originalTrack.keys[index].value));
        CHECK(retimedTrack.keys[index].interpolation ==
              originalTrack.keys[index].interpolation);
    }

    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    for (std::uint32_t frame = 0U; frame <= sourceFrames; ++frame) {
        const auto originalValue = EvaluateWaterKeyedSettingTrack(
            originalTrack,
            static_cast<float>(frame) /
                static_cast<float>(sourceFrames));
        const auto retimedValue = EvaluateWaterKeyedSettingTrack(
            retimedTrack,
            static_cast<float>(frame) /
                static_cast<float>(destinationFrames));
        REQUIRE(originalValue.has_value());
        REQUIRE(retimedValue.has_value());
        CHECK(*retimedValue == Approx(*originalValue).margin(1.0e-6F));
    }
    const auto terminalValue =
        EvaluateWaterKeyedSettingTrack(originalTrack, 1.0F);
    REQUIRE(terminalValue.has_value());
    for (std::uint32_t frame = sourceFrames + 1U;
         frame <= destinationFrames;
         ++frame) {
        const auto value = EvaluateWaterKeyedSettingTrack(
            retimedTrack,
            static_cast<float>(frame) /
                static_cast<float>(destinationFrames));
        REQUIRE(value.has_value());
        CHECK(*value == Approx(*terminalValue).margin(1.0e-6F));
    }
}

TEST_CASE(
    "Paused camera keys stay fixed while procedural effects use independent time",
    "[water][timing][clock][pan-extension]") {
    invisible_places::water::WaterKeyedSettingTrack keyedRain;
    keyedRain.settingId = "level";
    keyedRain.defaultInterpolation = WaterScenarioInterpolation::Linear;
    keyedRain.keys = {
        {.position = 0.0F,
         .value = 0.2F,
         .interpolation = WaterScenarioInterpolation::TrackDefault},
        {.position = 2.0F / 3.0F,
         .value = 0.8F,
         .interpolation = WaterScenarioInterpolation::TrackDefault},
    };

    // This is a paused camera in the added tail of a 120 -> 180 frame
    // extension. Its keyed value is selected only by camera position.
    constexpr float pausedCameraPosition = 5.0F / 6.0F;
    const auto keyedBefore =
        invisible_places::water::EvaluateWaterKeyedSettingTrack(
            keyedRain,
            pausedCameraPosition);
    const auto keyedLater =
        invisible_places::water::EvaluateWaterKeyedSettingTrack(
            keyedRain,
            pausedCameraPosition);
    REQUIRE(keyedBefore.has_value());
    REQUIRE(keyedLater.has_value());
    CHECK(*keyedBefore == Approx(0.8F));
    CHECK(*keyedLater == Approx(*keyedBefore));

    invisible_places::water::WaterSeepagePulseFieldSettings pulse;
    pulse.seed = 2027808452U;
    pulse.nodeId = 4U;
    pulse.stableSpanMeters = 3.125F;
    const auto samplePulse = [&](float effectTimeSeconds) {
        pulse.timeSeconds = effectTimeSeconds;
        return invisible_places::water::BuildWaterSeepagePulseField(pulse)
            .samples;
    };

    // Live time keeps advancing while the camera position above remains fixed.
    const auto liveBefore = samplePulse(4.0F);
    const auto liveLater = samplePulse(4.83F);
    CHECK(liveLater != liveBefore);

    // Export samples the full frame/subframe time, including past the old
    // four-second camera end, and is repeatable for the same output sample.
    constexpr float outputTimeSeconds = (151.0F + 0.5F) / 30.0F;
    static_assert(outputTimeSeconds > 120.0F / 30.0F);
    const auto exported = samplePulse(outputTimeSeconds);
    CHECK(exported == samplePulse(outputTimeSeconds));
    CHECK(exported != samplePulse(120.0F / 30.0F));
}

TEST_CASE("Timing run evaluation respects Linear and Hold modes", "[water][timing]") {
    const auto linear = Run(
        WaterTimingFeature::Flow,
        {Key(0.0F, 0.0F, WaterScenarioInterpolation::Linear), Key(1.0F, 1.0F)});
    CHECK(EvaluateWaterTimingRun(linear, 0.25F, 0.0F) == Approx(0.25F));

    const auto hold = Run(
        WaterTimingFeature::Flow,
        {Key(0.2F, 0.8F, WaterScenarioInterpolation::Hold), Key(0.8F, 0.1F)});
    CHECK(EvaluateWaterTimingRun(hold, 0.5F, 0.0F) == Approx(0.8F));
    CHECK(EvaluateWaterTimingRun(hold, 0.79F, 0.0F) == Approx(0.8F));
    CHECK(EvaluateWaterTimingRun(hold, 0.8F, 0.0F) == Approx(0.1F));
}

TEST_CASE("Empty timing run returns the fallback level", "[water][timing]") {
    const auto empty = Run(WaterTimingFeature::Seepage, {});
    CHECK(EvaluateWaterTimingRun(empty, 0.4F, 0.65F) == Approx(0.65F));
}

TEST_CASE("Sampling exactly at a key after a Hold segment yields the post-step level", "[water][timing]") {
    const auto run = Run(
        WaterTimingFeature::Rain,
        {Key(0.0F, 0.0F, WaterScenarioInterpolation::Hold),
         Key(0.5F, 1.0F, WaterScenarioInterpolation::Hold),
         Key(1.0F, 0.3F)});

    CHECK(EvaluateWaterTimingRun(run, 0.25F, 0.0F) == Approx(0.0F));
    CHECK(EvaluateWaterTimingRun(run, 0.5F, 0.0F) == Approx(1.0F));
    CHECK(EvaluateWaterTimingRun(run, 0.75F, 0.0F) == Approx(1.0F));
    CHECK(EvaluateWaterTimingRun(run, 1.0F, 0.0F) == Approx(0.3F));
}

TEST_CASE("Compiled tracks reproduce Hold steps followed by interior keys", "[water][timing][compile]") {
    WaterScenarioState base;
    base.rainLevel = 0.0F;

    const auto run = Run(
        WaterTimingFeature::Rain,
        {Key(0.2F, 1.0F, WaterScenarioInterpolation::Hold),
         Key(0.6F, 0.0F, WaterScenarioInterpolation::Linear),
         Key(1.0F, 0.0F)});
    const std::vector<WaterTimingRun> runs{run};
    const auto keys = CompileWaterTimingScenarioKeys(base, runs);

    REQUIRE(!keys.empty());
    WaterScenarioTrack track;
    track.scenarioId = "test";
    track.keys = keys;
    WaterScenarioDefinition definition;
    definition.id = "test";
    definition.state = base;
    for (const float position : {0.3F, 0.59F, 0.61F, 0.7F, 0.8F, 0.95F}) {
        const auto evaluated = EvaluateWaterScenarioTrack(track, definition, position);
        CHECK(evaluated.rainLevel ==
              Approx(EvaluateWaterTimingRun(run, position, 0.0F)).margin(1.0e-4));
    }
}

TEST_CASE("Later runs win when two target the same feature", "[water][timing][compile]") {
    WaterScenarioState base;
    base.rainLevel = 0.0F;

    const auto first = Run(WaterTimingFeature::Rain, {Key(0.5F, 0.25F)});
    const auto second = Run(WaterTimingFeature::Rain, {Key(0.5F, 0.75F)});
    const std::vector<WaterTimingRun> runs{first, second};
    const auto keys = CompileWaterTimingScenarioKeys(base, runs);

    REQUIRE(keys.size() == 1U);
    CHECK(keys.front().state.rainLevel == Approx(0.75F));
}

TEST_CASE("Timing key updates replace within tolerance and stay sorted", "[water][timing]") {
    auto run = Run(WaterTimingFeature::Rain, {Key(0.6F, 0.2F)});
    AddOrUpdateWaterTimingKey(&run, Key(0.2F, 0.5F));
    AddOrUpdateWaterTimingKey(&run, Key(0.60004F, 0.9F));

    REQUIRE(run.keys.size() == 2U);
    CHECK(run.keys[0].position == Approx(0.2F));
    CHECK(run.keys[1].level == Approx(0.9F));
}

TEST_CASE("Sanitizing a timing run clamps and orders keys", "[water][timing]") {
    auto run = Run(
        WaterTimingFeature::MeshFlow,
        {Key(1.4F, 2.0F), Key(-0.5F, -1.0F), Key(0.5F, 0.5F)});
    run = SanitizeWaterTimingRun(std::move(run));

    REQUIRE(run.keys.size() == 3U);
    CHECK(run.keys.front().position == Approx(0.0F));
    CHECK(run.keys.front().level == Approx(0.0F));
    CHECK(run.keys.back().position == Approx(1.0F));
    CHECK(run.keys.back().level == Approx(1.0F));
}

TEST_CASE("Timing levels map onto every scenario channel", "[water][timing]") {
    WaterScenarioState state;
    ApplyWaterTimingLevelToScenarioState(WaterTimingFeature::Shoreline, 0.1F, &state);
    ApplyWaterTimingLevelToScenarioState(WaterTimingFeature::Seepage, 0.2F, &state);
    ApplyWaterTimingLevelToScenarioState(WaterTimingFeature::Rain, 0.3F, &state);
    ApplyWaterTimingLevelToScenarioState(WaterTimingFeature::Flow, 0.4F, &state);
    ApplyWaterTimingLevelToScenarioState(WaterTimingFeature::MeshFlow, 0.5F, &state);

    CHECK(state.shorelineLevel == Approx(0.1F));
    CHECK(state.seepageLevel == Approx(0.2F));
    CHECK(state.rainLevel == Approx(0.3F));
    CHECK(state.flowLevel == Approx(0.4F));
    CHECK(state.meshFlowLevel == Approx(0.5F));
    CHECK(WaterTimingLevelFromScenarioState(WaterTimingFeature::Shoreline, state) == Approx(0.1F));
    CHECK(WaterTimingLevelFromScenarioState(WaterTimingFeature::MeshFlow, state) == Approx(0.5F));
}

TEST_CASE("Scenario states default Shoreline to fully visible", "[water][timing][scenario]") {
    const WaterScenarioState state;
    CHECK(state.shorelineLevel == Approx(1.0F));
}

TEST_CASE("Compiling one run reproduces its keys exactly", "[water][timing][compile]") {
    WaterScenarioState base;
    base.rainLevel = 0.0F;
    base.flowLevel = 0.75F;

    const auto run = Run(
        WaterTimingFeature::Rain,
        {Key(0.3F, 0.0F), Key(0.5F, 1.0F), Key(0.6F, 0.0F)});
    const std::vector<WaterTimingRun> runs{run};
    const auto keys = CompileWaterTimingScenarioKeys(base, runs);

    REQUIRE(keys.size() == 3U);
    CHECK(keys[0].position == Approx(0.3F));
    CHECK(keys[0].interpolation == WaterScenarioInterpolation::Smooth);
    CHECK(keys[0].state.rainLevel == Approx(0.0F));
    CHECK(keys[1].position == Approx(0.5F));
    CHECK(keys[1].state.rainLevel == Approx(1.0F));
    CHECK(keys[2].position == Approx(0.6F));
    CHECK(keys[2].interpolation == WaterScenarioInterpolation::Hold);
    CHECK(keys[2].state.rainLevel == Approx(0.0F));
    for (const auto& key : keys) {
        CHECK(key.state.flowLevel == Approx(0.75F));
    }

    // The compiled track evaluates like the run itself.
    WaterScenarioTrack track;
    track.scenarioId = "test";
    track.keys = keys;
    WaterScenarioDefinition definition;
    definition.id = "test";
    definition.state = base;
    for (const float position : {0.0F, 0.35F, 0.42F, 0.55F, 0.8F}) {
        const auto evaluated = EvaluateWaterScenarioTrack(track, definition, position);
        CHECK(evaluated.rainLevel ==
              Approx(EvaluateWaterTimingRun(run, position, 0.0F)).margin(1.0e-4));
        CHECK(evaluated.flowLevel == Approx(0.75F));
    }
}

TEST_CASE("Compiling merged runs stays close to each authored curve", "[water][timing][compile]") {
    WaterScenarioState base;
    base.rainLevel = 0.0F;
    base.flowLevel = 1.0F;

    const auto rain = Run(
        WaterTimingFeature::Rain,
        {Key(0.3F, 0.0F), Key(0.5F, 1.0F), Key(0.6F, 0.0F)});
    const auto flow = Run(
        WaterTimingFeature::Flow,
        {Key(0.4F, 1.0F, WaterScenarioInterpolation::Linear),
         Key(0.55F, 0.0F, WaterScenarioInterpolation::Linear)});
    const std::vector<WaterTimingRun> runs{rain, flow};
    const auto keys = CompileWaterTimingScenarioKeys(base, runs);

    REQUIRE(!keys.empty());
    WaterScenarioTrack track;
    track.scenarioId = "test";
    track.keys = keys;
    WaterScenarioDefinition definition;
    definition.id = "test";
    definition.state = base;

    for (float position = 0.0F; position <= 1.0F; position += 0.01F) {
        const auto evaluated = EvaluateWaterScenarioTrack(track, definition, position);
        CHECK(evaluated.rainLevel ==
              Approx(EvaluateWaterTimingRun(rain, position, 0.0F)).margin(0.02));
        CHECK(evaluated.flowLevel ==
              Approx(EvaluateWaterTimingRun(flow, position, 1.0F)).margin(0.02));
    }
}

TEST_CASE("Compiling ignores empty runs", "[water][timing][compile]") {
    const WaterScenarioState base;
    const std::vector<WaterTimingRun> runs{Run(WaterTimingFeature::Rain, {})};
    CHECK(CompileWaterTimingScenarioKeys(base, runs).empty());
}

TEST_CASE("Animation round trip preserves timing assignments", "[water][timing][serialization]") {
    invisible_places::camera::AnimationPath path;
    path.name = "TimingRoundTrip";
    path.durationFrames = 3600;
    path.selectedWaterScenarioId = "pre-colonisation-wet";

    WaterScenarioTrack track;
    track.scenarioId = "pre-colonisation-wet";
    track.scenarioName = "Pre-Colonisation Wet";
    track.fallbackScenario.id = "pre-colonisation-wet";
    track.fallbackScenario.state.rainLevel = 0.0F;

    WaterTimingRunAssignment assignment;
    assignment.feature = WaterTimingFeature::Rain;
    assignment.runId = "timing_run_1";
    assignment.runName = "Storm Build";
    assignment.fallbackRun = Run(
        WaterTimingFeature::Rain,
        {Key(0.3F, 0.0F), Key(0.5F, 1.0F), Key(0.6F, 0.0F, WaterScenarioInterpolation::Linear)},
        "Storm Build");
    assignment.fallbackRun.id = "timing_run_1";
    track.timingAssignments.push_back(assignment);
    track.keys = CompileWaterTimingScenarioKeys(
        track.fallbackScenario.state,
        std::vector<WaterTimingRun>{assignment.fallbackRun});
    path.waterScenarioTracks.push_back(std::move(track));

    TemporaryTimingFile file{"invisible_places_timing_roundtrip.ipanim.json"};
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveAnimationPath(path, file.path, &errorMessage));

    const auto loaded = invisible_places::serialization::LoadAnimationPath(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterScenarioTracks.size() == 1U);
    const auto& loadedTrack = loaded->waterScenarioTracks.front();
    REQUIRE(loadedTrack.timingAssignments.size() == 1U);
    const auto& loadedAssignment = loadedTrack.timingAssignments.front();
    CHECK(loadedAssignment.feature == WaterTimingFeature::Rain);
    CHECK(loadedAssignment.runId == "timing_run_1");
    CHECK(loadedAssignment.runName == "Storm Build");
    REQUIRE(loadedAssignment.fallbackRun.keys.size() == 3U);
    CHECK(loadedAssignment.fallbackRun.keys[1].position == Approx(0.5F));
    CHECK(loadedAssignment.fallbackRun.keys[1].level == Approx(1.0F));
    CHECK(loadedAssignment.fallbackRun.keys[2].interpolation ==
          WaterScenarioInterpolation::Linear);
    CHECK(loadedTrack.keys.size() == path.waterScenarioTracks.front().keys.size());
}

TEST_CASE("Animation tracks without timing keys stay compatible", "[water][timing][serialization]") {
    // A minimal pre-schema-13 style document: no timing_assignments and no
    // shoreline_level anywhere. Both must default silently.
    const std::string legacyJson = R"({
        "schema_version": 12,
        "name": "Legacy",
        "duration_frames": 900,
        "keys": [],
        "selected_water_scenario_id": "contemporary-managed",
        "water_scenario_tracks": [
            {
                "scenario_id": "contemporary-managed",
                "scenario_name": "Contemporary Managed",
                "keys": [
                    {
                        "id": "water_key_1",
                        "position": 0.25,
                        "state": {"rain_level": 0.6},
                        "interpolation": "smooth"
                    }
                ]
            }
        ]
    })";

    TemporaryTimingFile file{"invisible_places_timing_legacy.ipanim.json"};
    {
        std::ofstream output{file.path};
        REQUIRE(output.good());
        output << legacyJson;
    }

    std::string errorMessage;
    const auto loaded = invisible_places::serialization::LoadAnimationPath(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterScenarioTracks.size() == 1U);
    const auto& track = loaded->waterScenarioTracks.front();
    CHECK(track.timingAssignments.empty());
    REQUIRE(track.keys.size() == 1U);
    CHECK(track.keys.front().state.rainLevel == Approx(0.6F));
    CHECK(track.keys.front().state.shorelineLevel == Approx(1.0F));
}

TEST_CASE("Project round trip preserves the timing run library", "[water][timing][serialization]") {
    invisible_places::serialization::ProjectDocument document;
    document.projectName = "Timing Library";
    document.waterTimingRuns.push_back(Run(
        WaterTimingFeature::Flow,
        {Key(0.1F, 0.0F), Key(0.9F, 1.0F, WaterScenarioInterpolation::Hold)},
        "Future Flow"));
    document.waterTimingRuns.front().id = "timing_run_7";

    TemporaryTimingFile file{"invisible_places_timing_project.json"};
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, file.path, &errorMessage));

    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterTimingRuns.size() == 1U);
    const auto& run = loaded->waterTimingRuns.front();
    CHECK(run.id == "timing_run_7");
    CHECK(run.name == "Future Flow");
    CHECK(run.feature == WaterTimingFeature::Flow);
    REQUIRE(run.keys.size() == 2U);
    CHECK(run.keys[1].position == Approx(0.9F));
    CHECK(run.keys[1].interpolation == WaterScenarioInterpolation::Hold);
}

TEST_CASE("Old default scenario names migrate to Past/Future and Current", "[water][timing][serialization][migration]") {
    const std::string legacyJson = R"({
        "schema_version": 45,
        "project_name": "Legacy Names",
        "water_scenarios": [
            {
                "id": "pre-colonisation-wet",
                "name": "Pre-Colonisation Wet",
                "state": {"seepage_level": 1.0}
            },
            {
                "id": "contemporary-managed",
                "name": "Contemporary Managed",
                "state": {"seepage_level": 0.5}
            },
            {
                "id": "custom-take",
                "name": "Contemporary Managed",
                "state": {"seepage_level": 0.25}
            }
        ]
    })";

    TemporaryTimingFile file{"invisible_places_scenario_rename.json"};
    {
        std::ofstream output{file.path};
        REQUIRE(output.good());
        output << legacyJson;
    }

    std::string errorMessage;
    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterScenarios.size() == 3U);
    CHECK(loaded->waterScenarios[0].name == "Past/Future");
    CHECK(loaded->waterScenarios[1].name == "Current");
    // Custom scenarios keep their authored names even when they reuse an old
    // default display name, because migration is keyed on the default ids.
    CHECK(loaded->waterScenarios[2].name == "Contemporary Managed");
}

TEST_CASE("Edited scenario shadows round-trip through temp_water_scenario", "[water][timing][serialization]") {
    invisible_places::serialization::ProjectDocument document;
    document.projectName = "Shadow";
    WaterScenarioDefinition edited;
    edited.id = "pre-colonisation-wet";
    edited.name = "Past/Future_edited";
    edited.state.seepageLevel = 0.42F;
    edited.state.rainLevel = 0.9F;
    document.tempWaterScenario = edited;

    TemporaryTimingFile file{"invisible_places_scenario_shadow.json"};
    std::string errorMessage;
    REQUIRE(invisible_places::serialization::SaveProjectDocument(document, file.path, &errorMessage));

    const auto loaded =
        invisible_places::serialization::LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->tempWaterScenario.has_value());
    CHECK(loaded->tempWaterScenario->id == "pre-colonisation-wet");
    CHECK(loaded->tempWaterScenario->name == "Past/Future_edited");
    CHECK(loaded->tempWaterScenario->state.seepageLevel == Approx(0.42F));
    CHECK(loaded->tempWaterScenario->state.rainLevel == Approx(0.9F));
}

TEST_CASE("Keyed setting tracks hold endpoints and step at exact Hold keys",
          "[water][timing][keyed]") {
    using Catch::Approx;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterKeyedSettingTrack track;
    track.settingId = "strength";
    CHECK_FALSE(EvaluateWaterKeyedSettingTrack(track, 0.5F).has_value());

    track.keys = {
        {.position = 0.20F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::Hold},
        {.position = 0.40F,
         .value = 1.2F,
         .interpolation = WaterScenarioInterpolation::Linear},
        {.position = 0.60F,
         .value = 0.4F,
         .interpolation = WaterScenarioInterpolation::Smooth},
        {.position = 0.80F, .value = 0.0F},
    };
    // Endpoint hold both sides.
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.0F).value() ==
          Approx(0.0F));
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.95F).value() ==
          Approx(0.0F));
    // Hold segment keeps the left value strictly inside it...
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.30F).value() ==
          Approx(0.0F));
    // ...but sampling exactly AT the next key reads the post-step value —
    // the step lands on the key, never one sample late.
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.40F).value() ==
          Approx(1.2F));
    // Linear midpoint.
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.50F).value() ==
          Approx(0.8F));
    // Smoothstep midpoint equals the linear midpoint by symmetry.
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.70F).value() ==
          Approx(0.2F));
}

TEST_CASE("Cyclic keyed settings use virtual neighbours across the loop seam",
          "[water][timing][keyed][cyclic]") {
    using Catch::Approx;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::EvaluateWaterKeyedSettingTrackCyclic;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterKeyedSettingTrack smooth;
    smooth.settingId = "rain.level";
    smooth.keys = {
        {.position = 0.25F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::Smooth},
        {.position = 0.75F,
         .value = 1.0F,
         .interpolation = WaterScenarioInterpolation::Smooth},
    };
    CHECK(EvaluateWaterKeyedSettingTrack(smooth, 0.0F).value() ==
          Approx(0.0F));
    CHECK(EvaluateWaterKeyedSettingTrackCyclic(smooth, 0.0F).value() ==
          Approx(0.5F));
    CHECK(EvaluateWaterKeyedSettingTrackCyclic(smooth, 1.0F).value() ==
          Approx(0.5F));
    CHECK(EvaluateWaterKeyedSettingTrackCyclic(smooth, -0.1F).value() ==
          Approx(EvaluateWaterKeyedSettingTrackCyclic(smooth, 0.9F).value()));
    // Loop zero is only a camera-window boundary, not an authored key. The
    // virtual 0.75 -> 1.25 Smooth Step segment therefore crosses it with the
    // same non-zero derivative on both sides instead of slowing to rest.
    constexpr float epsilon = 1.0e-3F;
    const float beforeZero =
        EvaluateWaterKeyedSettingTrackCyclic(smooth, -epsilon).value();
    const float atZero =
        EvaluateWaterKeyedSettingTrackCyclic(smooth, 0.0F).value();
    const float afterZero =
        EvaluateWaterKeyedSettingTrackCyclic(smooth, epsilon).value();
    const float incomingRate = (atZero - beforeZero) / epsilon;
    const float outgoingRate = (afterZero - atZero) / epsilon;
    CHECK(incomingRate == Approx(outgoingRate).margin(0.02F));
    CHECK(std::abs(incomingRate) > 1.0F);

    WaterKeyedSettingTrack hold = smooth;
    hold.keys[0U].interpolation = WaterScenarioInterpolation::Hold;
    hold.keys[1U].interpolation = WaterScenarioInterpolation::Hold;
    CHECK(EvaluateWaterKeyedSettingTrackCyclic(hold, 0.0F).value() ==
          Approx(1.0F));
    CHECK(EvaluateWaterKeyedSettingTrackCyclic(hold, 0.25F).value() ==
          Approx(0.0F));

    WaterKeyedSettingTrack handles = smooth;
    for (auto& key : handles.keys) {
        key.interpolation = WaterScenarioInterpolation::SplineHandles;
    }
    CHECK_FALSE(invisible_places::water::
                    ResolveWaterSettingSplineHandlePoint(
                        handles,
                        0.25F,
                        invisible_places::water::
                            WaterSettingSplineHandleSide::Incoming)
                    .has_value());
    const auto cyclicIncoming = invisible_places::water::
        ResolveWaterSettingSplineHandlePoint(
            handles,
            0.25F,
            invisible_places::water::
                WaterSettingSplineHandleSide::Incoming,
            true);
    const auto cyclicOutgoing = invisible_places::water::
        ResolveWaterSettingSplineHandlePoint(
            handles,
            0.75F,
            invisible_places::water::
                WaterSettingSplineHandleSide::Outgoing,
            true);
    REQUIRE(cyclicIncoming.has_value());
    REQUIRE(cyclicOutgoing.has_value());
    CHECK(cyclicIncoming->controlPosition == Approx(1.0F / 12.0F));
    CHECK(cyclicOutgoing->controlPosition == Approx(11.0F / 12.0F));
}

TEST_CASE("Keyed setting interpolation retains precision beyond UI display",
          "[water][timing][keyed][precision]") {
    using invisible_places::water::AddOrUpdateWaterSettingKey;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterKeyedSettingTrack track;
    track.settingId = "normalised_amount";
    AddOrUpdateWaterSettingKey(
        &track,
        0.20F,
        0.123456F,
        WaterScenarioInterpolation::Linear);
    AddOrUpdateWaterSettingKey(
        &track,
        0.80F,
        0.987654F,
        WaterScenarioInterpolation::Linear);

    REQUIRE(track.keys.size() == 2U);
    CHECK(track.keys[0].value == Approx(0.123456F).margin(1.0e-7F));
    CHECK(track.keys[1].value == Approx(0.987654F).margin(1.0e-7F));
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.35F).value() ==
          Approx(0.3395055F).margin(1.0e-6F));
}

TEST_CASE("Track-default keys follow the track's default interpolation",
          "[water][timing][keyed][interpolation]") {
    using Catch::Approx;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;

    const auto makeTrack = [](WaterScenarioInterpolation keyMode,
                              WaterScenarioInterpolation defaultMode) {
        WaterKeyedSettingTrack track;
        track.settingId = "strength";
        track.defaultInterpolation = defaultMode;
        track.keys = {
            {.position = 0.00F, .value = 0.0F, .interpolation = keyMode},
            {.position = 0.40F, .value = 1.0F, .interpolation = keyMode},
            {.position = 0.60F, .value = 1.4F, .interpolation = keyMode},
            {.position = 1.00F, .value = 0.2F, .interpolation = keyMode},
        };
        return track;
    };

    // TrackDefault keys evaluate exactly like keys carrying the default
    // concretely — here the fluid Catmull-Rom spline...
    const auto inherited = makeTrack(
        WaterScenarioInterpolation::TrackDefault,
        WaterScenarioInterpolation::CentripetalCatmullRom);
    const auto concrete = makeTrack(
        WaterScenarioInterpolation::CentripetalCatmullRom,
        WaterScenarioInterpolation::Smooth);
    for (const float position : {0.1F, 0.35F, 0.5F, 0.62F, 0.9F}) {
        CHECK(EvaluateWaterKeyedSettingTrack(inherited, position).value() ==
              Approx(EvaluateWaterKeyedSettingTrack(concrete, position)
                         .value()));
    }
    // ...and restyling the default alone restyles the whole track: the same
    // keys under a Smooth default match concrete Smooth keys.
    const auto smoothDefault = makeTrack(
        WaterScenarioInterpolation::TrackDefault,
        WaterScenarioInterpolation::Smooth);
    const auto smoothConcrete = makeTrack(
        WaterScenarioInterpolation::Smooth,
        WaterScenarioInterpolation::CentripetalCatmullRom);
    for (const float position : {0.1F, 0.5F, 0.9F}) {
        CHECK(
            EvaluateWaterKeyedSettingTrack(smoothDefault, position).value() ==
            Approx(EvaluateWaterKeyedSettingTrack(smoothConcrete, position)
                       .value()));
    }
    // The spline default carries speed through a monotonic interior key
    // (values still rising on both sides at 0.40) where the Smooth default
    // has eased to a standstill. Reversals like the 0.60 peak still rest —
    // that part of the behaviour is deliberate.
    const float nearKey = 0.395F;
    const float atKey = 0.40F;
    const float splineStep = std::abs(
        EvaluateWaterKeyedSettingTrack(inherited, atKey).value() -
        EvaluateWaterKeyedSettingTrack(inherited, nearKey).value());
    const float smoothStep = std::abs(
        EvaluateWaterKeyedSettingTrack(smoothDefault, atKey).value() -
        EvaluateWaterKeyedSettingTrack(smoothDefault, nearKey).value());
    CHECK(splineStep > smoothStep * 2.0F);
}

TEST_CASE("New setting tracks default to monotone spline interpolation",
          "[water][timing][keyed][interpolation][defaults]") {
    using Catch::Approx;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::SanitizeWaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;
    using invisible_places::water::WaterSettingKey;

    WaterKeyedSettingTrack track;
    CHECK(track.defaultInterpolation ==
          WaterScenarioInterpolation::SmoothVelocity);
    CHECK(WaterSettingKey{}.interpolation ==
          WaterScenarioInterpolation::TrackDefault);

    track.settingId = "strength";
    track.keys = {
        {.position = 0.0F, .value = 0.0F},
        {.position = 0.4F, .value = 1.0F},
        {.position = 0.7F, .value = 1.5F},
        {.position = 1.0F, .value = 0.2F},
    };
    auto concrete = track;
    for (auto& key : concrete.keys) {
        key.interpolation = WaterScenarioInterpolation::SmoothVelocity;
    }
    for (const float position : {0.1F, 0.35F, 0.55F, 0.85F}) {
        REQUIRE(EvaluateWaterKeyedSettingTrack(track, position).has_value());
        CHECK(EvaluateWaterKeyedSettingTrack(track, position).value() ==
              Approx(EvaluateWaterKeyedSettingTrack(concrete, position)
                         .value()));
    }

    // TrackDefault is not a valid track-level mode. Sanitizing a malformed
    // new record resolves it to the same monotone default rather than the
    // older Catmull-Rom fallback.
    track.defaultInterpolation = WaterScenarioInterpolation::TrackDefault;
    const auto sanitized = SanitizeWaterKeyedSettingTrack(std::move(track));
    CHECK(sanitized.defaultInterpolation ==
          WaterScenarioInterpolation::SmoothVelocity);
}

TEST_CASE("Adding a key between keys preserves surrounding blends and order",
          "[water][timing][keyed]") {
    using Catch::Approx;
    using invisible_places::water::AddOrUpdateWaterSettingKey;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::NextWaterSettingKeyPosition;
    using invisible_places::water::PreviousWaterSettingKeyPosition;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterKeyedSettingTrack track;
    track.settingId = "prominence";
    AddOrUpdateWaterSettingKey(
        &track, 0.25F, 0.5F, WaterScenarioInterpolation::Linear);
    AddOrUpdateWaterSettingKey(
        &track, 0.40F, 1.0F, WaterScenarioInterpolation::Linear);
    // Insert between the two at the blended value: the curve through the
    // new key is unchanged, and later edits only bend it locally.
    const float blended =
        EvaluateWaterKeyedSettingTrack(track, 0.325F).value();
    AddOrUpdateWaterSettingKey(
        &track, 0.325F, blended, WaterScenarioInterpolation::Linear);
    REQUIRE(track.keys.size() == 3U);
    CHECK(track.keys[1].position == Approx(0.325F));
    // The value at 0.30 still lies on the ORIGINAL 0.25->0.40 line: the
    // inserted key changed the curve nowhere.
    CHECK(EvaluateWaterKeyedSettingTrack(track, 0.30F).value() ==
          Approx(0.5F + (0.30F - 0.25F) / 0.15F * 0.5F).margin(1.0e-4F));

    // Replacement within tolerance updates in place instead of duplicating.
    AddOrUpdateWaterSettingKey(
        &track, 0.32505F, 0.9F, WaterScenarioInterpolation::Smooth);
    REQUIRE(track.keys.size() == 3U);
    CHECK(track.keys[1].value == Approx(0.9F));

    // Prev/next navigation is strict and bounded.
    CHECK(PreviousWaterSettingKeyPosition(track, 0.325F).value() ==
          Approx(0.25F));
    CHECK(NextWaterSettingKeyPosition(track, 0.325F).value() ==
          Approx(0.40F));
    CHECK_FALSE(PreviousWaterSettingKeyPosition(track, 0.25F).has_value());
    CHECK_FALSE(NextWaterSettingKeyPosition(track, 0.40F).has_value());
}

TEST_CASE("Dormant setting tracks preserve keys without evaluating or navigating",
          "[water][timing][keyed][dormant]") {
    using invisible_places::water::BuildWaterFeatureTimingOverlay;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::NextWaterFeatureKeyPosition;
    using invisible_places::water::PreviousWaterFeatureKeyPosition;
    using invisible_places::water::WaterFeatureProfileKeyPositions;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline timeline{
        .feature = {
            .kind = WaterKeyedFeatureKind::SeepageNode,
            .objectId = 12U},
        .settings = {
            {.settingId = "look.density",
             .active = false,
             .label = "Coverage",
             .profileGroup = "seepage_look",
             .profileName = "Wet Rock",
             .keys = {
                 {.position = 0.20F, .value = 0.25F},
                 {.position = 0.70F, .value = 0.85F},
             }},
            {.settingId = "look.glisten",
             .active = true,
             .label = "Glisten",
             .profileGroup = "seepage_look",
             .profileName = "Wet Rock",
             .keys = {
                 {.position = 0.40F, .value = 0.30F},
                 {.position = 0.70F, .value = 0.90F},
             }},
        }};

    CHECK_FALSE(
        EvaluateWaterKeyedSettingTrack(
            timeline.settings.front(),
            0.5F)
            .has_value());
    CHECK(
        PreviousWaterFeatureKeyPosition(timeline, 0.65F).value() ==
        Approx(0.40F));
    CHECK(
        NextWaterFeatureKeyPosition(timeline, 0.65F).value() ==
        Approx(0.70F));

    const auto positions =
        WaterFeatureProfileKeyPositions(
            timeline,
            "seepage_look");
    REQUIRE(positions.size() == 2U);
    CHECK(positions[0] == Approx(0.40F));
    CHECK(positions[1] == Approx(0.70F));

    WaterFeatureTimingRun run;
    run.id = 2U;
    run.features.push_back(timeline);
    const auto overlay =
        BuildWaterFeatureTimingOverlay(
            std::span{&run, 1},
            0.5F);
    CHECK(
        overlay.Find(
            timeline.feature,
            "look.density") == nullptr);
    REQUIRE(
        overlay.Find(
            timeline.feature,
            "look.glisten") != nullptr);
}

TEST_CASE("Focused feature run assignment preserves its complete timeline",
          "[water][timing][keyed][runs]") {
    using invisible_places::water::AssignWaterFeatureToTimingRun;
    using invisible_places::water::FindWaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterScenarioFeatureRuns;

    const WaterKeyedFeatureId feature{
        .kind = WaterKeyedFeatureKind::FlowSource,
        .objectId = 9U};
    WaterScenarioFeatureRuns entry;
    entry.scenarioId = "wet";
    entry.runs = {
        WaterFeatureTimingRun{
            .id = 1U,
            .name = "First",
            .features = {{
                .feature = feature,
                .settings = {{
                    .settingId = "strength",
                    .active = false,
                    .keys = {{
                        .position = 0.3F,
                        .value = 0.6F}},
                }},
            }}},
        WaterFeatureTimingRun{.id = 2U, .name = "Second"},
    };

    REQUIRE(
        AssignWaterFeatureToTimingRun(
            &entry,
            feature,
            2U));
    CHECK(
        FindWaterFeatureTimeline(
            &entry.runs[0],
            feature) == nullptr);
    const auto* moved =
        FindWaterFeatureTimeline(
            &entry.runs[1],
            feature);
    REQUIRE(moved != nullptr);
    REQUIRE(moved->settings.size() == 1U);
    CHECK_FALSE(moved->settings.front().active);
    REQUIRE(moved->settings.front().keys.size() == 1U);
    CHECK(
        moved->settings.front().keys.front().value ==
        Approx(0.6F));

    const WaterKeyedFeatureId newFeature{
        .kind = WaterKeyedFeatureKind::Rain};
    REQUIRE(
        AssignWaterFeatureToTimingRun(
            &entry,
            newFeature,
            2U));
    REQUIRE(
        FindWaterFeatureTimeline(
            &entry.runs[1],
            newFeature) != nullptr);
    CHECK_FALSE(
        AssignWaterFeatureToTimingRun(
            &entry,
            newFeature,
            99U));
}

TEST_CASE("Deleting a water feature removes only its timelines and keeps runs",
          "[water][timing][keyed][runs][delete]") {
    using invisible_places::water::RemoveWaterFeatureFromTimingRuns;
    using invisible_places::water::WaterFeatureSettingsClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterSettingKey;

    const WaterKeyedFeatureId deleted{
        .kind = WaterKeyedFeatureKind::FlowPath,
        .objectId = 34U};
    const WaterKeyedFeatureId retained{
        .kind = WaterKeyedFeatureKind::FlowPath,
        .objectId = 35U};
    const WaterKeyedFeatureId sameObjectOtherKind{
        .kind = WaterKeyedFeatureKind::FlowSource,
        .objectId = 34U};
    const WaterKeyedSettingTrack retainedTrack{
        .settingId = "strength",
        .active = false,
        .label = "Maximum Flow Strength",
        .profileGroup = "flow_path",
        .profileName = "Trail",
        .keys = {
            WaterSettingKey{
                .position = 0.2F,
                .value = 0.4F,
                .clipId = 7U},
            WaterSettingKey{
                .position = 0.8F,
                .value = 0.9F,
                .clipId = 7U},
        },
    };
    const WaterFeatureSettingsClip retainedClip{
        .id = 7U,
        .name = "Retained Clip",
        .start = 0.2F,
        .end = 0.8F,
        .sourceProfileName = "Retained Package",
    };
    std::vector<WaterFeatureTimingRun> runs{
        WaterFeatureTimingRun{
            .id = 10U,
            .name = "Mixed",
            .enabled = false,
            .features = {
                WaterFeatureTimeline{.feature = deleted},
                WaterFeatureTimeline{
                    .feature = retained,
                    .settings = {retainedTrack},
                    .clips = {retainedClip},
                    .clipMembershipExplicit = true},
                WaterFeatureTimeline{
                    .feature = sameObjectOtherKind,
                    .settings = {retainedTrack},
                    .clips = {retainedClip},
                    .clipMembershipExplicit = true},
            }},
        WaterFeatureTimingRun{
            .id = 11U,
            .name = "Becomes Empty",
            .features = {WaterFeatureTimeline{.feature = deleted}}},
    };

    CHECK(RemoveWaterFeatureFromTimingRuns(runs, deleted) == 2U);
    REQUIRE(runs.size() == 2U);
    CHECK(runs[0].id == 10U);
    CHECK(runs[0].name == "Mixed");
    CHECK_FALSE(runs[0].enabled);
    REQUIRE(runs[0].features.size() == 2U);
    CHECK(runs[0].features[0].feature == retained);
    CHECK(runs[0].features[1].feature == sameObjectOtherKind);

    for (const auto& timeline : runs[0].features) {
        REQUIRE(timeline.settings.size() == 1U);
        const auto& track = timeline.settings.front();
        CHECK(track.settingId == "strength");
        CHECK_FALSE(track.active);
        CHECK(track.label == "Maximum Flow Strength");
        CHECK(track.profileGroup == "flow_path");
        CHECK(track.profileName == "Trail");
        REQUIRE(track.keys.size() == 2U);
        CHECK(track.keys[0].position == Approx(0.2F));
        CHECK(track.keys[0].value == Approx(0.4F));
        CHECK(track.keys[0].clipId == 7U);
        CHECK(track.keys[1].position == Approx(0.8F));
        CHECK(track.keys[1].value == Approx(0.9F));
        CHECK(track.keys[1].clipId == 7U);
        REQUIRE(timeline.clips.size() == 1U);
        CHECK(timeline.clips.front().id == 7U);
        CHECK(timeline.clips.front().name == "Retained Clip");
        CHECK(timeline.clips.front().sourceProfileName == "Retained Package");
        CHECK(timeline.clipMembershipExplicit);
    }

    CHECK(runs[1].id == 11U);
    CHECK(runs[1].name == "Becomes Empty");
    CHECK(runs[1].features.empty());
    CHECK(RemoveWaterFeatureFromTimingRuns(runs, deleted) == 0U);
}

TEST_CASE("Feature key navigation finds the nearest key across every setting",
          "[water][timing][keyed][navigation]") {
    using Catch::Approx;
    using invisible_places::water::NextWaterFeatureKeyPosition;
    using invisible_places::water::PreviousWaterFeatureKeyPosition;
    using invisible_places::water::WaterFeatureTimeline;

    WaterFeatureTimeline timeline;
    CHECK_FALSE(PreviousWaterFeatureKeyPosition(timeline, 0.5F).has_value());
    CHECK_FALSE(NextWaterFeatureKeyPosition(timeline, 0.5F).has_value());

    timeline.settings = {
        {.settingId = "activity",
         .keys = {
             {.position = 0.10F, .value = 0.0F},
             {.position = 0.50F, .value = 1.0F},
             {.position = 0.90F, .value = 0.0F},
         }},
        {.settingId = "prominence",
         .keys = {
             {.position = 0.25F, .value = 0.5F},
             {.position = 0.60F, .value = 1.5F},
         }},
        {.settingId = "empty"},
    };

    CHECK(PreviousWaterFeatureKeyPosition(timeline, 0.55F).value() ==
          Approx(0.50F));
    CHECK(NextWaterFeatureKeyPosition(timeline, 0.55F).value() ==
          Approx(0.60F));
    CHECK(PreviousWaterFeatureKeyPosition(timeline, 0.50F).value() ==
          Approx(0.25F));
    CHECK(NextWaterFeatureKeyPosition(timeline, 0.50F).value() ==
          Approx(0.60F));
    CHECK_FALSE(
        PreviousWaterFeatureKeyPosition(timeline, 0.10F).has_value());
    CHECK_FALSE(NextWaterFeatureKeyPosition(timeline, 0.90F).has_value());

    // Positions within the replacement tolerance count as the current key.
    CHECK(PreviousWaterFeatureKeyPosition(timeline, 0.50005F).value() ==
          Approx(0.25F));
    CHECK(NextWaterFeatureKeyPosition(timeline, 0.50005F).value() ==
          Approx(0.60F));
}

TEST_CASE("Key positions move without overwriting another keyed value",
          "[water][timing][keyed][edit]") {
    using invisible_places::water::MoveWaterSettingKey;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterKeyedSettingTrack track{
        .settingId = "level",
        .keys = {
            {.position = 0.20F,
             .value = 0.25F,
             .interpolation = WaterScenarioInterpolation::Hold},
            {.position = 0.50F,
             .value = 0.75F,
             .interpolation = WaterScenarioInterpolation::Linear},
            {.position = 0.80F,
             .value = 1.00F,
             .interpolation = WaterScenarioInterpolation::Smooth},
        },
    };

    REQUIRE(MoveWaterSettingKey(&track, 0.50F, 0.65F));
    REQUIRE(track.keys.size() == 3U);
    CHECK(track.keys[1].position == Approx(0.65F));
    CHECK(track.keys[1].value == Approx(0.75F));
    CHECK(
        track.keys[1].interpolation ==
        WaterScenarioInterpolation::Linear);

    // Source matching uses the same tolerance as marker navigation, while
    // destinations outside the normalized range or on another key fail
    // without mutating the track.
    CHECK(MoveWaterSettingKey(&track, 0.65005F, 0.40F));
    CHECK_FALSE(MoveWaterSettingKey(&track, 0.40F, 0.80005F));
    CHECK_FALSE(MoveWaterSettingKey(&track, 0.40F, -0.01F));
    CHECK_FALSE(MoveWaterSettingKey(&track, 0.40F, 1.01F));
    CHECK_FALSE(MoveWaterSettingKey(&track, 0.99F, 0.30F));
    CHECK(track.keys[0].position == Approx(0.20F));
    CHECK(track.keys[1].position == Approx(0.40F));
    CHECK(track.keys[2].position == Approx(0.80F));
}

TEST_CASE("Setting and feature key deletion only affects the current position",
          "[water][timing][keyed][delete]") {
    using invisible_places::water::RemoveWaterFeatureKeysAtPosition;
    using invisible_places::water::RemoveWaterSettingKeysAtPosition;
    using invisible_places::water::WaterFeatureKeyCountAtPosition;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterSettingKeyCountAtPosition;

    WaterFeatureTimeline timeline;
    timeline.settings = {
        {.settingId = "activity",
         .keys = {
             {.position = 0.20F, .value = 0.1F},
             {.position = 0.50F, .value = 0.8F},
             {.position = 0.80F, .value = 0.2F},
         }},
        {.settingId = "prominence",
         .keys = {
             {.position = 0.50F, .value = 1.5F},
             {.position = 0.70F, .value = 0.5F},
         }},
    };

    CHECK(
        WaterSettingKeyCountAtPosition(
            timeline.settings.front(),
            0.50005F) == 1U);
    CHECK(WaterFeatureKeyCountAtPosition(timeline, 0.50F) == 2U);
    CHECK(WaterFeatureKeyCountAtPosition(timeline, 0.51F) == 0U);

    CHECK(
        RemoveWaterSettingKeysAtPosition(
            &timeline.settings.front(),
            0.50005F) == 1U);
    CHECK(timeline.settings.front().keys.size() == 2U);
    CHECK(WaterFeatureKeyCountAtPosition(timeline, 0.50F) == 1U);

    CHECK(RemoveWaterFeatureKeysAtPosition(&timeline, 0.50F) == 1U);
    CHECK(WaterFeatureKeyCountAtPosition(timeline, 0.50F) == 0U);
    CHECK(timeline.settings[0].keys.size() == 2U);
    CHECK(timeline.settings[1].keys.size() == 1U);
    CHECK(timeline.settings[0].keys[0].position == Approx(0.20F));
    CHECK(timeline.settings[0].keys[1].position == Approx(0.80F));
    CHECK(timeline.settings[1].keys[0].position == Approx(0.70F));
    CHECK(RemoveWaterFeatureKeysAtPosition(nullptr, 0.70F) == 0U);
}

TEST_CASE("Feature timing overlay samples every keyed setting and drives scenario channels",
          "[water][timing][keyed][overlay]") {
    using Catch::Approx;
    using invisible_places::water::ApplyWaterFeatureTimingOverlayToScenario;
    using invisible_places::water::BuildWaterFeatureTimingOverlay;
    using invisible_places::water::FindWaterFeatureRunContaining;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterScenarioInterpolation;
    using invisible_places::water::WaterScenarioState;

    WaterFeatureTimingRun rainRun;
    rainRun.id = 1U;
    rainRun.name = "Storm";
    rainRun.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::Rain},
        .settings = {{
            .settingId = "level",
            .keys = {
                {.position = 0.0F,
                 .value = 0.0F,
                 .interpolation = WaterScenarioInterpolation::Linear},
                {.position = 0.5F,
                 .value = 0.5F,
                 .interpolation = WaterScenarioInterpolation::Linear},
            },
        }},
    });
    rainRun.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageNode,
                    .objectId = 4U},
        .settings = {{
            .settingId = "strength",
            .keys = {{.position = 0.2F, .value = 1.5F}},
        }},
    });

    const std::vector<WaterFeatureTimingRun> runs{rainRun};
    auto overlay = BuildWaterFeatureTimingOverlay(runs, 0.25F);
    REQUIRE(overlay.samples.size() == 2U);
    const auto* rain = overlay.Find(
        {.kind = WaterKeyedFeatureKind::Rain}, "level");
    REQUIRE(rain != nullptr);
    CHECK(*rain == Approx(0.25F));
    const auto* strength = overlay.Find(
        {.kind = WaterKeyedFeatureKind::SeepageNode, .objectId = 4U},
        "strength");
    REQUIRE(strength != nullptr);
    CHECK(*strength == Approx(1.5F));
    CHECK(overlay.Find(
              {.kind = WaterKeyedFeatureKind::SeepageNode, .objectId = 9U},
              "strength") == nullptr);

    // Only the keyed global channel moves; everything else is untouched.
    WaterScenarioState state;
    state.rainLevel = 1.0F;
    state.flowLevel = 0.7F;
    state.seepageLevel = 0.6F;
    overlay.samples.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::FlowGlobal},
        .settingId = "level",
        .value = 0.1F,
    });
    overlay.samples.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageGlobal},
        .settingId = "level",
        .value = 0.2F,
    });
    ApplyWaterFeatureTimingOverlayToScenario(overlay, &state);
    CHECK(state.rainLevel == Approx(0.25F));
    CHECK(state.flowLevel == Approx(0.7F));
    CHECK(state.seepageLevel == Approx(0.6F));

    const auto* containing = FindWaterFeatureRunContaining(
        runs,
        {.kind = WaterKeyedFeatureKind::SeepageNode, .objectId = 4U});
    REQUIRE(containing != nullptr);
    CHECK(containing->name == "Storm");
    CHECK(FindWaterFeatureRunContaining(
              runs,
              {.kind = WaterKeyedFeatureKind::FlowSource, .objectId = 1U}) ==
          nullptr);
}

TEST_CASE("Muted timing runs keep their keys but stop driving water features",
          "[water][timing][keyed][overlay][mute]") {
    using Catch::Approx;
    using invisible_places::water::BuildWaterFeatureTimingOverlay;
    using invisible_places::water::BuildWaterMeshFlowRainEnvelope;
    using invisible_places::water::FindWaterFeatureRunContaining;
    using invisible_places::water::WaterDynamicMeshFlowSettings;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterFeatureTimingRun stormRun;
    stormRun.id = 1U;
    stormRun.name = "Storm";
    stormRun.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::Rain},
        .settings = {{
            .settingId = "level",
            .keys = {
                {.position = 0.0F,
                 .value = 1.0F,
                 .interpolation = WaterScenarioInterpolation::Linear},
                {.position = 1.0F,
                 .value = 1.0F,
                 .interpolation = WaterScenarioInterpolation::Linear},
            },
        }},
    });
    WaterFeatureTimingRun seepRun;
    seepRun.id = 2U;
    seepRun.name = "Seep";
    seepRun.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageNode,
                    .objectId = 4U},
        .settings = {{
            .settingId = "strength",
            .keys = {{.position = 0.2F, .value = 1.5F}},
        }},
    });

    std::vector<WaterFeatureTimingRun> runs{stormRun, seepRun};
    runs[0].enabled = false;

    // The muted Storm run contributes no samples; the enabled Seep run is
    // untouched.
    const auto overlay = BuildWaterFeatureTimingOverlay(runs, 0.5F);
    CHECK(overlay.Find(
              {.kind = WaterKeyedFeatureKind::Rain}, "level") == nullptr);
    REQUIRE(overlay.Find(
                {.kind = WaterKeyedFeatureKind::SeepageNode, .objectId = 4U},
                "strength") != nullptr);

    // "Which run drives this feature" resolves nothing for a muted run, but
    // membership checks that pass includeDisabled still see the claim, so
    // muting cannot free a feature for another run.
    CHECK(FindWaterFeatureRunContaining(
              runs,
              {.kind = WaterKeyedFeatureKind::Rain}) == nullptr);
    const auto* claimed = FindWaterFeatureRunContaining(
        runs,
        {.kind = WaterKeyedFeatureKind::Rain},
        /*includeDisabled=*/true);
    REQUIRE(claimed != nullptr);
    CHECK(claimed->name == "Storm");

    // Rain envelopes fall back to the authored level once the keyed rain
    // run is muted.
    const WaterDynamicMeshFlowSettings meshSettings{};
    const auto mutedEnvelope = BuildWaterMeshFlowRainEnvelope(
        meshSettings,
        runs,
        0.0F,
        10.0F);
    for (const float sample : mutedEnvelope.samples) {
        CHECK(sample == Approx(0.0F).margin(1.0e-4F));
    }
    runs[0].enabled = true;
    const auto drivenEnvelope = BuildWaterMeshFlowRainEnvelope(
        meshSettings,
        runs,
        0.0F,
        10.0F);
    REQUIRE(!drivenEnvelope.samples.empty());
    CHECK(drivenEnvelope.samples.back() > 0.5F);
    CHECK(mutedEnvelope.fingerprint != drivenEnvelope.fingerprint);
}

TEST_CASE("Authored water response settings are keyable and overlays stay transient",
          "[water][timing][keyed][authored]") {
    using Catch::Approx;
    using invisible_places::water::
        ApplyWaterFeatureTimingOverlayToDynamicMeshFlowSettings;
    using invisible_places::water::
        ApplyWaterFeatureTimingOverlayToSeepageNode;
    using invisible_places::water::FindWaterKeyableSetting;
    using invisible_places::water::WaterDynamicMeshFlowSettings;
    using invisible_places::water::WaterFeatureTimingOverlay;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterSeepageNode;

    const auto* delay = FindWaterKeyableSetting(
        WaterKeyedFeatureKind::SeepageNode,
        "rain_delay_seconds");
    REQUIRE(delay != nullptr);
    CHECK(delay->minimum == Approx(0.0F));
    CHECK(delay->maximum == Approx(120.0F));
    const auto* recession = FindWaterKeyableSetting(
        WaterKeyedFeatureKind::SeepageNode,
        "rain_recession_seconds");
    REQUIRE(recession != nullptr);
    CHECK(recession->maximum == Approx(300.0F));

    const auto* activity = FindWaterKeyableSetting(
        WaterKeyedFeatureKind::MeshFlow,
        "level");
    REQUIRE(activity != nullptr);
    CHECK(std::string_view{activity->label} == "Activity");
    REQUIRE(
        FindWaterKeyableSetting(
            WaterKeyedFeatureKind::MeshFlow,
            "rain_gain") != nullptr);
    REQUIRE(
        FindWaterKeyableSetting(
            WaterKeyedFeatureKind::MeshFlow,
            "moisture_persistence") != nullptr);
    CHECK(
        invisible_places::water::WaterKeyableSettings(
            WaterKeyedFeatureKind::SeepageGlobal)
            .empty());
    CHECK(
        invisible_places::water::WaterKeyableSettings(
            WaterKeyedFeatureKind::FlowGlobal)
            .empty());
    const auto pointSourceSettings =
        invisible_places::water::WaterKeyableSettings(
            WaterKeyedFeatureKind::FlowSource);
    CHECK(pointSourceSettings.size() == 2U);
    CHECK(
        FindWaterKeyableSetting(
            WaterKeyedFeatureKind::FlowSource,
            "speed") == nullptr);
    const auto pathSourceSettings =
        invisible_places::water::WaterKeyableSettings(
            WaterKeyedFeatureKind::FlowPath);
    CHECK(pathSourceSettings.size() == 5U);
    for (const char* settingId :
         {"strength",
          "rain_response",
          "speed",
          "trail_width",
          "trail_streak_length"}) {
        REQUIRE(
            FindWaterKeyableSetting(
                WaterKeyedFeatureKind::FlowPath,
                settingId) != nullptr);
    }

    WaterFeatureTimingOverlay overlay;
    overlay.samples = {
        {{.kind = WaterKeyedFeatureKind::SeepageNode, .objectId = 9U},
         "rain_delay_seconds",
         4.5F},
        {{.kind = WaterKeyedFeatureKind::SeepageNode, .objectId = 9U},
         "rain_rise_seconds",
         -2.0F},
        {{.kind = WaterKeyedFeatureKind::MeshFlow},
         "level",
         0.25F},
        {{.kind = WaterKeyedFeatureKind::MeshFlow},
         "rain_gain",
         1.5F},
        {{.kind = WaterKeyedFeatureKind::MeshFlow},
         "moisture_persistence",
         2.0F},
    };

    WaterSeepageNode node;
    node.id = 9U;
    node.rainDelaySeconds = 1.0F;
    node.rainRiseSeconds = 3.0F;
    ApplyWaterFeatureTimingOverlayToSeepageNode(overlay, &node);
    CHECK(node.rainDelaySeconds == Approx(4.5F));
    CHECK(node.rainRiseSeconds == Approx(0.0F));
    CHECK(node.rainRecessionSeconds == Approx(0.0F));

    WaterDynamicMeshFlowSettings mesh;
    ApplyWaterFeatureTimingOverlayToDynamicMeshFlowSettings(
        overlay,
        &mesh);
    CHECK(mesh.activity == Approx(0.25F));
    CHECK(mesh.rainGain == Approx(1.5F));
    CHECK(mesh.moisturePersistenceMultiplier == Approx(2.0F));
}

TEST_CASE("Rain runtime uniforms key smoothly without changing authored lifecycle state",
          "[water][rain][timing][keyed]") {
    using Catch::Approx;
    using invisible_places::water::ApplyWaterFeatureTimingOverlayToRainSettings;
    using invisible_places::water::BuildWaterFeatureTimingOverlay;
    using invisible_places::water::FindWaterKeyableSetting;
    using invisible_places::water::RainIntensityPreset;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterRainSettings;
    using invisible_places::water::WaterRainVisualSettings;
    using invisible_places::water::WaterScenarioInterpolation;

    const auto rainSettings = invisible_places::water::WaterKeyableSettings(
        WaterKeyedFeatureKind::Rain);
    CHECK(rainSettings.size() > 40U);
    const auto* levelInfo = FindWaterKeyableSetting(
        WaterKeyedFeatureKind::Rain,
        "level");
    REQUIRE(levelInfo != nullptr);
    CHECK(levelInfo->showUnauthoredInTimeline);
    const auto* densityInfo = FindWaterKeyableSetting(
        WaterKeyedFeatureKind::Rain,
        "density");
    REQUIRE(densityInfo != nullptr);
    CHECK_FALSE(densityInfo->showUnauthoredInTimeline);
    for (const char* authoredOnly :
         {"seed",
          "particle_limit",
          "intensity_preset",
          "spawn_height",
          "spawn_radius",
          "camera_death_distance",
          "impact_effects_enabled"}) {
        CHECK(
            FindWaterKeyableSetting(
                WaterKeyedFeatureKind::Rain,
                authoredOnly) == nullptr);
    }

    const auto track = [](const char* settingId,
                          float first,
                          float second,
                          WaterScenarioInterpolation interpolation =
                              WaterScenarioInterpolation::Linear) {
        WaterKeyedSettingTrack result;
        result.settingId = settingId;
        result.keys = {
            {.position = 0.0F,
             .value = first,
             .interpolation = interpolation},
            {.position = 1.0F,
             .value = second,
             .interpolation = interpolation},
        };
        return result;
    };

    WaterFeatureTimingRun run;
    run.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::Rain},
        .settings = {
            track(
                "level",
                0.0F,
                1.0F,
                WaterScenarioInterpolation::Smooth),
            track("density", 0.20F, 1.0F),
            track("fall_speed", 4.0F, 12.0F),
            track("visual.width", 0.002F, 0.006F),
            track("visual.colour_red", 0.2F, 1.0F),
            track("weather.wind_direction_x", -1.0F, 1.0F),
            track("weather.wind_speed", 0.0F, 4.0F),
            track("near_surface.squish", 0.20F, 0.80F),
            track("effects.wetness_response", 0.0F, 2.0F),
            track("wetness.spread_speed", 0.50F, 2.50F),
            track("droplets.propagation", 0.10F, 1.30F),
            track("rings.band_max_z", 1.0F, 3.0F),
        },
    });
    const std::array runs{run};
    const auto overlay = BuildWaterFeatureTimingOverlay(runs, 0.25F);

    WaterRainSettings authored;
    authored.enabled = false;
    authored.activeParticleCount = 12'345U;
    authored.seed = 987U;
    authored.intensityPreset = RainIntensityPreset::HeavyDownpour;
    authored.spawnHeightMeters = 9.0F;
    authored.spawnRadiusMeters = 11.0F;
    authored.cameraDeathDistanceMeters = 42.0F;
    authored.impactEffectsEnabled = false;
    WaterRainVisualSettings authoredVisual;
    authoredVisual.colour = {0.4F, 0.5F, 0.6F};
    const auto originalAuthored = authored;
    const auto originalVisual = authoredVisual;

    auto resolved = authored;
    auto resolvedVisual = authoredVisual;
    ApplyWaterFeatureTimingOverlayToRainSettings(
        overlay,
        &resolved,
        &resolvedVisual);

    // Smooth Step at one quarter is 0.15625; the remaining tracks are linear.
    CHECK(resolved.enabled);
    CHECK(resolved.rainLevel == Approx(0.15625F));
    CHECK(resolved.density == Approx(0.40F));
    CHECK(resolved.fallSpeedMetersPerSecond == Approx(6.0F));
    CHECK(resolved.windDirectionX == Approx(-0.50F));
    CHECK(resolved.windSpeedMetersPerSecond == Approx(1.0F));
    CHECK(resolved.nearSurface.squish == Approx(0.35F));
    CHECK(resolved.rockEffectScale == Approx(0.50F));
    CHECK(resolved.rockImpact.spreadSpeed == Approx(1.0F));
    CHECK(
        resolved.vegetationImpact.propagationMetersPerSecond ==
        Approx(0.40F));
    CHECK(resolved.sandImpactBand.maxZ == Approx(1.50F));
    CHECK(resolvedVisual.widthMeters == Approx(0.003F));
    CHECK(resolvedVisual.colour[0] == Approx(0.40F));

    CHECK(resolved.activeParticleCount == originalAuthored.activeParticleCount);
    CHECK(resolved.seed == originalAuthored.seed);
    CHECK(resolved.intensityPreset == originalAuthored.intensityPreset);
    CHECK(resolved.spawnHeightMeters == Approx(originalAuthored.spawnHeightMeters));
    CHECK(resolved.spawnRadiusMeters == Approx(originalAuthored.spawnRadiusMeters));
    CHECK(
        resolved.cameraDeathDistanceMeters ==
        Approx(originalAuthored.cameraDeathDistanceMeters));
    CHECK(
        resolved.impactEffectsEnabled ==
        originalAuthored.impactEffectsEnabled);

    // Evaluation is transient: the project-owned base values stay untouched.
    CHECK(authored.enabled == originalAuthored.enabled);
    CHECK(authored.density == Approx(originalAuthored.density));
    CHECK(authoredVisual.widthMeters == Approx(originalVisual.widthMeters));
    CHECK(authoredVisual.colour == originalVisual.colour);
}

TEST_CASE(
    "Water keyed setting display identity is canonical across storage and clips",
    "[water][timing][keyed][colour]") {
    using invisible_places::water::WaterFeatureClipPrimarySettingDisplayIndex;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyableSettings;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingDisplayIndex;
    using invisible_places::water::WaterKeyedSettingTrack;

    const auto rainRegistry =
        WaterKeyableSettings(WaterKeyedFeatureKind::Rain);
    REQUIRE(rainRegistry.size() > 3U);

    // Registry ordinals never compress around hidden unauthored settings.
    // This is the common identity consumed by embedded/global graphs, keyed
    // controls, markers, and pins even when only one Rain track is stored.
    CHECK(WaterKeyedSettingDisplayIndex(
              WaterKeyedFeatureKind::Rain,
              "level") == 0U);
    CHECK(WaterKeyedSettingDisplayIndex(
              WaterKeyedFeatureKind::Rain,
              "density") == 1U);
    CHECK(WaterKeyedSettingDisplayIndex(
              WaterKeyedFeatureKind::Rain,
              "fall_speed") == 2U);

    const std::vector<WaterKeyedSettingTrack> firstOrder{
        {.settingId = "zeta.profile"},
        {.settingId = "density"},
        {.settingId = "alpha.profile"},
    };
    const std::vector<WaterKeyedSettingTrack> secondOrder{
        {.settingId = "alpha.profile"},
        {.settingId = "density"},
        {.settingId = "zeta.profile"},
    };
    for (const auto& tracks : {firstOrder, secondOrder}) {
        CHECK(WaterKeyedSettingDisplayIndex(
                  WaterKeyedFeatureKind::Rain,
                  "density",
                  tracks) == 1U);
        CHECK(WaterKeyedSettingDisplayIndex(
                  WaterKeyedFeatureKind::Rain,
                  "alpha.profile",
                  tracks) == rainRegistry.size());
        CHECK(WaterKeyedSettingDisplayIndex(
                  WaterKeyedFeatureKind::Rain,
                  "zeta.profile",
                  tracks) == rainRegistry.size() + 1U);
    }

    WaterFeatureTimeline timeline;
    timeline.feature = {.kind = WaterKeyedFeatureKind::Rain};
    timeline.settings = {
        // Storage order and active state deliberately disagree with display
        // order. Explicit ownership, not visibility, determines clip colour.
        {.settingId = "zeta.profile",
         .keys = {{.position = 0.4F, .clipId = 8U}}},
        {.settingId = "density",
         .active = false,
         .keys = {{.position = 0.2F, .clipId = 8U}}},
        {.settingId = "alpha.profile",
         .keys = {
             {.position = 0.1F, .clipId = 0U},
             {.position = 0.6F, .clipId = 9U},
         }},
        {.settingId = "level",
         .keys = {{.position = 0.8F, .clipId = 10U}}},
    };
    timeline.clips = {
        {.id = 8U, .name = "Mixed"},
        {.id = 9U, .name = "Dynamic"},
        {.id = 10U, .name = "Level"},
        {.id = 11U, .name = "Empty"},
    };

    CHECK(WaterFeatureClipPrimarySettingDisplayIndex(timeline, 8U) ==
          WaterKeyedSettingDisplayIndex(
              WaterKeyedFeatureKind::Rain,
              "density",
              timeline.settings));
    CHECK(WaterFeatureClipPrimarySettingDisplayIndex(timeline, 9U) ==
          rainRegistry.size());
    CHECK(WaterFeatureClipPrimarySettingDisplayIndex(timeline, 10U) == 0U);
    CHECK(WaterFeatureClipPrimarySettingDisplayIndex(timeline, 0U) ==
          rainRegistry.size());
    CHECK_FALSE(
        WaterFeatureClipPrimarySettingDisplayIndex(timeline, 11U)
            .has_value());
}

TEST_CASE(
    "Water clip semantic lanes are stable and preserve every owned span",
    "[water][timing][keyed][clips][lanes]") {
    using invisible_places::water::BuildWaterFeatureClipLaneLayout;
    using invisible_places::water::WaterFeatureClipConciseDisplayName;
    using invisible_places::water::WaterFeatureClipSettingSignatureForId;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline timeline;
    timeline.feature = {.kind = WaterKeyedFeatureKind::Rain};
    // Deliberately not time/id ordered: lane identity must not inherit clip
    // vector order. Start and Finish share Density and are separated in time.
    timeline.clips = {
        {.id = 6U, .name = "Empty", .start = 0.60F, .end = 0.70F},
        {.id = 3U, .name = "Touching", .start = 0.20F, .end = 0.30F},
        {.id = 5U, .name = "Mixed", .start = 0.31F, .end = 0.38F},
        {.id = 2U, .name = "Finish", .start = 0.40F, .end = 0.50F},
        {.id = 4U, .name = "Level", .start = 0.75F, .end = 0.82F},
        {.id = 1U, .name = "Start", .start = 0.10F, .end = 0.20F},
        {.id = 7U, .name = "Empty Later", .start = 0.80F, .end = 0.90F},
    };
    timeline.settings = {
        {.settingId = "fall_speed",
         .keys = {
             {.position = 0.34F, .clipId = 5U},
             {.position = 0.92F, .clipId = 0U},
         }},
        // Dormancy changes evaluation/visibility, never semantic ownership.
        {.settingId = "density",
         .active = false,
         .keys = {
             {.position = 0.10F, .clipId = 1U},
             {.position = 0.20F, .clipId = 1U},
             {.position = 0.40F, .clipId = 2U},
             {.position = 0.50F, .clipId = 2U},
             {.position = 0.20F, .clipId = 3U},
             {.position = 0.30F, .clipId = 3U},
             {.position = 0.35F, .clipId = 5U},
         }},
        {.settingId = "level",
         .keys = {{.position = 0.78F, .clipId = 4U}}},
    };

    CHECK(
        WaterFeatureClipSettingSignatureForId(timeline, 1U).settingIds ==
        std::vector<std::string>{"density"});
    CHECK(
        WaterFeatureClipSettingSignatureForId(timeline, 5U).settingIds ==
        std::vector<std::string>{"density", "fall_speed"});
    CHECK(
        WaterFeatureClipSettingSignatureForId(timeline, 0U).settingIds ==
        std::vector<std::string>{"fall_speed"});
    CHECK(
        WaterFeatureClipSettingSignatureForId(timeline, 6U)
            .settingIds.empty());
    CHECK_FALSE(
        WaterFeatureClipSettingSignatureForId(timeline, 6U)
            .minimumDisplayIndex.has_value());

    const auto layout = BuildWaterFeatureClipLaneLayout(timeline);
    const auto assignment = [&](std::uint32_t clipId) -> const auto& {
        const auto found = std::find_if(
            layout.assignments.begin(),
            layout.assignments.end(),
            [&](const auto& candidate) {
                return candidate.clipId == clipId;
            });
        REQUIRE(found != layout.assignments.end());
        return *found;
    };
    CHECK(layout.signatureBandCount == 5U);
    CHECK(layout.laneCount == 6U);
    CHECK(assignment(4U).signatureBandIndex == 0U);  // Level.
    CHECK(assignment(1U).signatureBandIndex == 1U);  // Density.
    CHECK(assignment(5U).signatureBandIndex == 2U);  // Density + Fall.
    CHECK(assignment(0U).signatureBandIndex == 3U);  // Loose Fall.
    CHECK(assignment(6U).signatureBandIndex == 4U);  // Dedicated empty.
    CHECK(assignment(1U).laneIndex == assignment(2U).laneIndex);
    CHECK(assignment(1U).spillLaneIndex == 0U);
    // Exact-touch is an overlap for grabbing purposes.
    CHECK(assignment(3U).signatureBandIndex ==
          assignment(1U).signatureBandIndex);
    CHECK(assignment(3U).spillLaneIndex == 1U);
    CHECK(assignment(3U).laneIndex != assignment(1U).laneIndex);
    // Empty markers share their dedicated band and reuse when disjoint.
    CHECK(assignment(6U).laneIndex == assignment(7U).laneIndex);

    auto permuted = timeline;
    std::ranges::reverse(permuted.clips);
    CHECK(BuildWaterFeatureClipLaneLayout(permuted) == layout);
    std::ranges::reverse(permuted.settings);
    CHECK(BuildWaterFeatureClipLaneLayout(permuted) == layout);

    // The loose ghost's minimum display width participates in allocation.
    // At the end of the domain it expands backwards and overlaps this clip.
    WaterFeatureTimeline endTouch;
    endTouch.feature = {.kind = WaterKeyedFeatureKind::Rain};
    endTouch.clips = {
        {.id = 20U, .name = "At End", .start = 0.998F, .end = 0.9995F},
    };
    endTouch.settings = {
        {.settingId = "density",
         .keys = {
             {.position = 0.998F, .clipId = 20U},
             {.position = 1.0F, .clipId = 0U},
         }},
    };
    const auto endLayout = BuildWaterFeatureClipLaneLayout(endTouch);
    REQUIRE(endLayout.assignments.size() == 2U);
    CHECK(endLayout.signatureBandCount == 1U);
    CHECK(endLayout.laneCount == 2U);
    CHECK(endLayout.assignments[0].clipId == 0U);
    CHECK(endLayout.assignments[1].clipId == 20U);
    CHECK(endLayout.assignments[0].laneIndex !=
          endLayout.assignments[1].laneIndex);

    CHECK(WaterFeatureClipConciseDisplayName(
              "1-1-Seepage Front Edge - Start - Node Strength",
              "1-2-Seepage Gooves") == "Start - Node Strength");
    CHECK(WaterFeatureClipConciseDisplayName(
              "1-1-Seepage Front Edge — Finish — Seepage Width",
              "1-1-Seepage Front Edge") == "Finish - Seepage Width");
    CHECK(WaterFeatureClipConciseDisplayName(
              "First-01: Slow Spread",
              "First-01") == "Slow Spread");
    CHECK(WaterFeatureClipConciseDisplayName(
              "First-010 Slow Spread",
              "First-01") == "First-010 Slow Spread");
    CHECK(WaterFeatureClipConciseDisplayName(
              "A Beautiful Start - Node Strength",
              "First-01") == "A Beautiful Start - Node Strength");
}

TEST_CASE("Per-scenario feature timing runs round-trip through the project document",
          "[water][timing][keyed][serialization]") {
    using Catch::Approx;
    using invisible_places::serialization::LoadProjectDocument;
    using invisible_places::serialization::ProjectDocument;
    using invisible_places::serialization::SaveProjectDocument;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterScenarioInterpolation;

    ProjectDocument document;
    document.projectName = "Feature timing runs";
    invisible_places::water::WaterScenarioFeatureRuns entry;
    entry.scenarioId = "pre-colonisation-wet";
    invisible_places::water::WaterFeatureTimingRun run;
    run.id = 7U;
    run.name = "Rain + Seeps";
    run.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageNode,
                    .objectId = 4U},
        .settings = {{
            .settingId = "strength",
            .active = false,
            .label = "Node Strength",
            .profileGroup = "seepage_node_settings",
            .profileName = "Wide Footprint",
            .keys = {
                {.position = 0.20F,
                 .value = 0.0F,
                 .interpolation = WaterScenarioInterpolation::Hold},
                {.position = 0.35F,
                 .value = 1.2F,
                 .interpolation = WaterScenarioInterpolation::Smooth},
            },
        }},
    });
    run.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::Rain},
        .settings = {
            {.settingId = "density",
             .label = "Density",
             .keys = {
                 {.position = 0.0F, .value = 0.10F},
                 {.position = 1.0F, .value = 0.90F},
             }},
            {.settingId = "visual.width",
             .label = "Width",
             .keys = {{.position = 0.5F, .value = 0.004F}}},
        },
    });
    entry.runs.push_back(run);
    document.waterFeatureTimingRuns.push_back(entry);
    document.waterKeyedSettingsProfiles.push_back({
        .name = "Calm Lanes_Creek Path",
        .baseProfileName = "Calm Lanes",
        .ownerObjectName = "Creek Path",
        .ownerObjectId = 42U,
        .featureKind = WaterKeyedFeatureKind::FlowPath,
        .settings = {{
            .settingId = "speed",
            .active = true,
            .label = "Speed",
            .profileGroup = "flow_path",
            .profileName = "Calm Lanes",
            .keys = {
                {.position = 0.0F, .value = 0.2F},
                {.position = 1.0F, .value = 0.8F},
            },
        }},
    });
    document.waterFeatureTimingRunSequence = 8U;

    TemporaryTimingFile file{"invisible_places_feature_timing_runs.json"};
    std::string errorMessage;
    REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
    const auto loaded = LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterFeatureTimingRuns.size() == 1U);
    const auto& loadedEntry = loaded->waterFeatureTimingRuns.front();
    CHECK(loadedEntry.scenarioId == "pre-colonisation-wet");
    REQUIRE(loadedEntry.runs.size() == 1U);
    const auto& loadedRun = loadedEntry.runs.front();
    CHECK(loadedRun.id == 7U);
    CHECK(loadedRun.name == "Rain + Seeps");
    REQUIRE(loadedRun.features.size() == 2U);
    const auto& timeline = loadedRun.features.front();
    CHECK(timeline.feature.kind == WaterKeyedFeatureKind::SeepageNode);
    CHECK(timeline.feature.objectId == 4U);
    REQUIRE(timeline.settings.size() == 1U);
    CHECK_FALSE(timeline.settings.front().active);
    CHECK(timeline.settings.front().label == "Node Strength");
    CHECK(
        timeline.settings.front().profileGroup ==
        "seepage_node_settings");
    CHECK(timeline.settings.front().profileName == "Wide Footprint");
    REQUIRE(timeline.settings.front().keys.size() == 2U);
    CHECK(timeline.settings.front().keys[0].position == Approx(0.20F));
    CHECK(timeline.settings.front().keys[0].interpolation ==
          WaterScenarioInterpolation::Hold);
    CHECK(timeline.settings.front().keys[1].value == Approx(1.2F));
    const auto& rainTimeline = loadedRun.features[1U];
    CHECK(rainTimeline.feature.kind == WaterKeyedFeatureKind::Rain);
    REQUIRE(rainTimeline.settings.size() == 2U);
    CHECK(rainTimeline.settings[0].settingId == "density");
    REQUIRE(rainTimeline.settings[0].keys.size() == 2U);
    CHECK(rainTimeline.settings[0].keys.back().value == Approx(0.90F));
    CHECK(rainTimeline.settings[1].settingId == "visual.width");
    CHECK(rainTimeline.settings[1].keys.front().value == Approx(0.004F));
    CHECK(loaded->waterFeatureTimingRunSequence == 8U);
    REQUIRE(loaded->waterKeyedSettingsProfiles.size() == 1U);
    const auto& keyedProfile =
        loaded->waterKeyedSettingsProfiles.front();
    CHECK(keyedProfile.name == "Calm Lanes_Creek Path");
    CHECK(keyedProfile.baseProfileName == "Calm Lanes");
    CHECK(keyedProfile.ownerObjectName == "Creek Path");
    CHECK(keyedProfile.ownerObjectId == 42U);
    CHECK_FALSE(keyedProfile.edited);
    REQUIRE(keyedProfile.settings.size() == 1U);
    CHECK(keyedProfile.settings.front().settingId == "speed");
    REQUIRE(keyedProfile.settings.front().keys.size() == 2U);
    CHECK(keyedProfile.settings.front().keys.back().value == Approx(0.8F));
}

TEST_CASE("Flow Path keyed profile names and shadows are object-owned",
          "[water][timing][keyed][profiles]") {
    using Catch::Approx;
    using invisible_places::water::SanitizeWaterKeyedSettingsProfile;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfileEditedName;
    using invisible_places::water::WaterKeyedSettingsProfileSavedName;

    CHECK(
        WaterKeyedSettingsProfileSavedName("  Calm Lanes  ", " Creek ") ==
        "Calm Lanes_Creek");
    CHECK(
        WaterKeyedSettingsProfileEditedName("Calm Lanes", "Creek") ==
        "Calm Lanes_Creek_edited");

    auto profile = SanitizeWaterKeyedSettingsProfile({
        .baseProfileName = "  Calm Lanes ",
        .ownerObjectName = " Creek ",
        .sourceProfileName = " Source_Path ",
        .ownerObjectId = 8U,
        .featureKind = WaterKeyedFeatureKind::FlowPath,
        .edited = true,
        .settings = {
            {.settingId = " speed ",
             .keys = {
                 {.position = 1.3F, .value = 0.8F},
                 {.position = -0.2F, .value = 0.2F},
             }},
            {.settingId = "speed", .keys = {{.position = 0.5F, .value = 4.0F}}},
        },
    });
    CHECK(profile.name == "Calm Lanes_Creek_edited");
    CHECK(profile.baseProfileName == "Calm Lanes");
    CHECK(profile.ownerObjectName == "Creek");
    CHECK(profile.sourceProfileName == "Source_Path");
    REQUIRE(profile.settings.size() == 1U);
    CHECK(profile.settings.front().settingId == "speed");
    REQUIRE(profile.settings.front().keys.size() == 2U);
    CHECK(profile.settings.front().keys.front().position == Approx(0.0F));
    CHECK(profile.settings.front().keys.back().position == Approx(1.0F));
}

TEST_CASE("Flow Path keyed profiles round-trip with standalone Water sources",
          "[water][timing][keyed][profiles][serialization]") {
    using invisible_places::serialization::LoadWaterSourcesDocument;
    using invisible_places::serialization::SaveWaterSourcesDocument;
    using invisible_places::serialization::WaterSourcesDocument;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterSourcesDocument document;
    document.manualFlowPaths.push_back({
        .id = 17U,
        .name = "Lower Creek",
        .keyedSettingsProfileName =
            "Calm Lanes_Lower Creek_edited",
    });
    document.keyedSettingsProfiles.push_back({
        .name = "Calm Lanes_Lower Creek_edited",
        .baseProfileName = "Calm Lanes",
        .ownerObjectName = "Lower Creek",
        .sourceProfileName = "Calm Lanes_Upper Creek",
        .ownerObjectId = 17U,
        .featureKind = WaterKeyedFeatureKind::FlowPath,
        .edited = true,
        .settings = {{
            .settingId = "trail_width",
            .active = true,
            .label = "Trail Width",
            .profileGroup = "flow_path",
            .profileName = "Calm Lanes",
            .keys = {{.position = 0.4F, .value = 0.012F}},
        }},
    });

    TemporaryTimingFile file{
        "invisible_places_flow_path_keyed_profiles.json"};
    std::string errorMessage;
    REQUIRE(SaveWaterSourcesDocument(
        document,
        file.path,
        &errorMessage));
    const auto loaded = LoadWaterSourcesDocument(
        file.path,
        &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->manualFlowPaths.size() == 1U);
    CHECK(
        loaded->manualFlowPaths.front().keyedSettingsProfileName ==
        "Calm Lanes_Lower Creek_edited");
    REQUIRE(loaded->keyedSettingsProfiles.size() == 1U);
    const auto& profile = loaded->keyedSettingsProfiles.front();
    CHECK(profile.edited);
    CHECK(profile.sourceProfileName == "Calm Lanes_Upper Creek");
    CHECK(profile.ownerObjectId == 17U);
    REQUIRE(profile.settings.size() == 1U);
    CHECK(profile.settings.front().settingId == "trail_width");
}

TEST_CASE("Project keyed-settings cleanup preserves packages for every water feature",
          "[water][timing][keyed][profiles][packages]") {
    using invisible_places::water::SanitizeWaterKeyedSettingsProfileLibrary;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;

    const std::array kinds{
        WaterKeyedFeatureKind::Rain,
        WaterKeyedFeatureKind::MeshFlow,
        WaterKeyedFeatureKind::Shoreline,
        WaterKeyedFeatureKind::SeepageGlobal,
        WaterKeyedFeatureKind::FlowGlobal,
        WaterKeyedFeatureKind::SeepageNode,
        WaterKeyedFeatureKind::FlowSource,
        WaterKeyedFeatureKind::FlowPath,
        WaterKeyedFeatureKind::ShorelineInstance,
    };
    std::vector<WaterKeyedSettingsProfile> profiles;
    profiles.reserve(kinds.size() + 1U);
    for (std::size_t index = 0U; index < kinds.size(); ++index) {
        profiles.push_back({
            .name = " Package " + std::to_string(index) + " ",
            .featureKind = kinds[index],
            .nativeLengthFraction = 0.25F,
            .settings = {{
                .settingId = " level ",
                .keys = {
                    {.position = 0.0F, .value = 0.0F},
                    {.position = 1.0F, .value = 1.0F},
                },
            }},
        });
    }
    profiles.push_back(profiles.front());

    const auto sanitized =
        SanitizeWaterKeyedSettingsProfileLibrary(std::move(profiles));
    REQUIRE(sanitized.size() == kinds.size());
    for (std::size_t index = 0U; index < kinds.size(); ++index) {
        CHECK(sanitized[index].name ==
              "Package " + std::to_string(index));
        CHECK(sanitized[index].featureKind == kinds[index]);
        REQUIRE(sanitized[index].settings.size() == 1U);
        CHECK(sanitized[index].settings.front().settingId == "level");
    }
}

TEST_CASE("Keyed-settings package identity includes feature kind",
          "[water][timing][keyed][profiles][packages]") {
    using invisible_places::water::FindWaterKeyedSettingsProfileIndex;
    using invisible_places::water::SanitizeWaterKeyedSettingsProfileLibrary;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;

    std::vector<WaterKeyedSettingsProfile> profiles{
        {
            .name = " Start ",
            .featureKind = WaterKeyedFeatureKind::SeepageNode,
            .settings = {{
                .settingId = "strength",
                .keys = {{.position = 0.0F, .value = 0.0F}},
            }},
        },
        {
            .name = "Start",
            .featureKind = WaterKeyedFeatureKind::FlowPath,
            .settings = {{
                .settingId = "trail_width",
                .keys = {{.position = 0.0F, .value = 0.25F}},
            }},
        },
        {
            .name = "Start",
            .featureKind = WaterKeyedFeatureKind::SeepageNode,
            .settings = {{
                .settingId = "prominence",
                .keys = {{.position = 0.0F, .value = 1.0F}},
            }},
        },
    };

    const auto sanitized =
        SanitizeWaterKeyedSettingsProfileLibrary(std::move(profiles));
    REQUIRE(sanitized.size() == 2U);

    const auto seepage = FindWaterKeyedSettingsProfileIndex(
        sanitized,
        WaterKeyedFeatureKind::SeepageNode,
        " Start ");
    const auto flowPath = FindWaterKeyedSettingsProfileIndex(
        sanitized,
        WaterKeyedFeatureKind::FlowPath,
        "Start");
    REQUIRE(seepage.has_value());
    REQUIRE(flowPath.has_value());
    CHECK(seepage.value() != flowPath.value());
    CHECK(sanitized[seepage.value()].settings.front().settingId ==
          "strength");
    CHECK(sanitized[flowPath.value()].settings.front().settingId ==
          "trail_width");
    CHECK_FALSE(
        FindWaterKeyedSettingsProfileIndex(
            sanitized,
            WaterKeyedFeatureKind::FlowSource,
            "Start")
            .has_value());
}

TEST_CASE("Keyed profile equality includes interpolation defaults and spline handles",
          "[water][timing][keyed][profiles][interpolation]") {
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrackProfileEqual;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterKeyedSettingTrack baseline{
        .settingId = "strength",
        .defaultInterpolation = WaterScenarioInterpolation::SmoothVelocity,
        .keys = {{
            .position = 0.25F,
            .value = 0.4F,
            .interpolation = WaterScenarioInterpolation::SplineHandles,
            .incomingHandleTime = 0.2F,
            .incomingHandleValue = -0.1F,
            .outgoingHandleTime = 0.4F,
            .outgoingHandleValue = 0.3F,
        }},
    };
    auto edited = baseline;
    CHECK(WaterKeyedSettingTrackProfileEqual(baseline, edited));

    edited.defaultInterpolation = WaterScenarioInterpolation::Smooth;
    CHECK_FALSE(WaterKeyedSettingTrackProfileEqual(baseline, edited));
    edited = baseline;
    edited.keys.front().incomingHandleTime += 0.01F;
    CHECK_FALSE(WaterKeyedSettingTrackProfileEqual(baseline, edited));
    edited = baseline;
    edited.keys.front().incomingHandleValue += 0.01F;
    CHECK_FALSE(WaterKeyedSettingTrackProfileEqual(baseline, edited));
    edited = baseline;
    edited.keys.front().outgoingHandleTime += 0.01F;
    CHECK_FALSE(WaterKeyedSettingTrackProfileEqual(baseline, edited));
    edited = baseline;
    edited.keys.front().outgoingHandleValue += 0.01F;
    CHECK_FALSE(WaterKeyedSettingTrackProfileEqual(baseline, edited));

    // Clip membership is independent authoring metadata and does not make a
    // whole-timeline keyed profile dirty by itself.
    edited = baseline;
    edited.keys.front().clipId = 42U;
    CHECK(WaterKeyedSettingTrackProfileEqual(baseline, edited));
}

TEST_CASE("Flow profile assignment rewrites are exact and kind scoped",
          "[water][flow][profiles][references]") {
    using invisible_places::water::ReplaceWaterFlowProfileAssignments;
    using invisible_places::water::WaterDynamicMeshFlowSettings;
    using invisible_places::water::WaterEmitter;
    using invisible_places::water::WaterFlowProfileKind;
    using invisible_places::water::WaterManualFlowPathSource;

    std::vector<WaterEmitter> emitters(3U);
    emitters[0].id = 11U;
    emitters[0].pathProfileName = " Path Copy ";
    emitters[0].laneProfileName = "Shared Copy";
    emitters[0].trailProfileName = "Shared Copy";
    emitters[0].pathProfileLocked = true;
    emitters[0].laneProfileLocked = true;
    emitters[0].trailProfileLocked = true;
    emitters[1].id = 12U;
    emitters[1].pathProfileName = "Path Copy";
    emitters[1].laneProfileName = "Other";
    emitters[1].trailProfileName = "Shared Copy";
    emitters[2].id = 13U;
    emitters[2].pathProfileName.clear();

    std::vector<WaterManualFlowPathSource> manualPaths(2U);
    manualPaths[0].id = 21U;
    manualPaths[0].laneProfileName = " Shared Copy ";
    manualPaths[0].trailProfileName = "Shared Copy";
    manualPaths[0].laneProfileLocked = true;
    manualPaths[0].trailProfileLocked = true;
    manualPaths[1].id = 22U;
    manualPaths[1].laneProfileName = "Other";
    manualPaths[1].trailProfileName = "Other";
    WaterDynamicMeshFlowSettings mesh;
    mesh.trailProfileName = "Shared Copy";

    const auto path = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Path,
        "Path Copy",
        "Saved Path");
    CHECK(path.pointSourceIds == std::vector<std::uint32_t>{11U, 12U});
    CHECK(path.manualPathSourceIds.empty());
    CHECK_FALSE(path.dynamicMeshTrailChanged);
    CHECK(emitters[0].pathProfileName == "Saved Path");
    CHECK(emitters[1].pathProfileName == "Saved Path");
    CHECK(emitters[2].pathProfileName.empty());
    CHECK(manualPaths[0].laneProfileName == " Shared Copy ");

    const auto lane = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Lane,
        "Shared Copy",
        "Saved Lane");
    CHECK(lane.pointSourceIds == std::vector<std::uint32_t>{11U});
    CHECK(lane.manualPathSourceIds == std::vector<std::uint32_t>{21U});
    CHECK_FALSE(lane.dynamicMeshTrailChanged);
    CHECK(emitters[0].laneProfileName == "Saved Lane");
    CHECK(emitters[0].trailProfileName == "Shared Copy");
    CHECK(manualPaths[0].laneProfileName == "Saved Lane");
    CHECK(manualPaths[0].trailProfileName == "Shared Copy");
    CHECK(mesh.trailProfileName == "Shared Copy");

    const auto trail = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Trail,
        "Shared Copy",
        "Saved Trail");
    CHECK(trail.pointSourceIds ==
          std::vector<std::uint32_t>{11U, 12U});
    CHECK(trail.manualPathSourceIds ==
          std::vector<std::uint32_t>{21U});
    CHECK(trail.dynamicMeshTrailChanged);
    CHECK(emitters[0].trailProfileName == "Saved Trail");
    CHECK(emitters[0].laneProfileName == "Saved Lane");
    CHECK(manualPaths[0].trailProfileName == "Saved Trail");
    CHECK(manualPaths[0].laneProfileName == "Saved Lane");
    CHECK(mesh.trailProfileName == "Saved Trail");

    mesh.trailProfileName = "Owner Trail_edited";
    const auto editedMesh = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Trail,
        "Owner Trail",
        "Saved Owner Trail",
        "Owner Trail_edited");
    CHECK(editedMesh.dynamicMeshTrailChanged);
    CHECK(editedMesh.dynamicMeshEditedTrailProfileMatched);
    CHECK(mesh.trailProfileName == "Saved Owner Trail");

    mesh.trailProfileName = "Unrelated_edited";
    const auto unrelatedMeshEdit = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Trail,
        "Owner Trail",
        "Ignored",
        "Unrelated_edited");
    CHECK_FALSE(unrelatedMeshEdit.dynamicMeshTrailChanged);
    CHECK_FALSE(
        unrelatedMeshEdit.dynamicMeshEditedTrailProfileMatched);
    CHECK(mesh.trailProfileName == "Unrelated_edited");

    const auto staleOwnerShadow = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Trail,
        "Owner Trail",
        "Ignored",
        "Owner Trail_edited");
    CHECK_FALSE(staleOwnerShadow.dynamicMeshTrailChanged);
    CHECK_FALSE(
        staleOwnerShadow.dynamicMeshEditedTrailProfileMatched);
    CHECK(mesh.trailProfileName == "Unrelated_edited");

    mesh.trailProfileName = "Other";
    const auto staleEditedProfile = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Trail,
        "Owner Trail",
        "Ignored",
        "Owner Trail_edited");
    CHECK_FALSE(staleEditedProfile.dynamicMeshTrailChanged);
    CHECK_FALSE(
        staleEditedProfile.dynamicMeshEditedTrailProfileMatched);
    CHECK(mesh.trailProfileName == "Other");

    mesh.trailProfileName = "Owner Trail";
    const auto exactMeshWithUnrelatedEdit =
        ReplaceWaterFlowProfileAssignments(
            emitters,
            manualPaths,
            &mesh,
            WaterFlowProfileKind::Trail,
            "Owner Trail",
            "Saved Owner Trail",
            "Unrelated_edited");
    CHECK(exactMeshWithUnrelatedEdit.dynamicMeshTrailChanged);
    CHECK_FALSE(
        exactMeshWithUnrelatedEdit
            .dynamicMeshEditedTrailProfileMatched);
    CHECK(mesh.trailProfileName == "Saved Owner Trail");

    const auto global = ReplaceWaterFlowProfileAssignments(
        emitters,
        manualPaths,
        &mesh,
        WaterFlowProfileKind::Path,
        "Global",
        "Default");
    CHECK(global.pointSourceIds == std::vector<std::uint32_t>{13U});
    CHECK(emitters[2].pathProfileName == "Default");
    CHECK_FALSE(ReplaceWaterFlowProfileAssignments(
                    emitters,
                    manualPaths,
                    &mesh,
                    WaterFlowProfileKind::Path,
                    "",
                    "Ignored")
                    .changed());

    CHECK(emitters[0].pathProfileLocked);
    CHECK(emitters[0].laneProfileLocked);
    CHECK(emitters[0].trailProfileLocked);
    CHECK(manualPaths[0].laneProfileLocked);
    CHECK(manualPaths[0].trailProfileLocked);
}

TEST_CASE("Modern Flow object-copy metadata outranks legacy edited suffixes",
          "[water][flow][profiles][persistence][migration]") {
    using invisible_places::water::
        WaterObjectProfileNameIsLegacyEditedShadow;

    constexpr std::array<std::string_view, 3U> ownedCopies{
        "Aerial_Creek_edited",
        "Calm Lanes_Creek_edited",
        "Water Flow_Creek_edited",
    };
    for (const auto name : ownedCopies) {
        CAPTURE(name);
        CHECK(WaterObjectProfileNameIsLegacyEditedShadow(name, false));
        CHECK_FALSE(WaterObjectProfileNameIsLegacyEditedShadow(name, true));
    }
    CHECK(WaterObjectProfileNameIsLegacyEditedShadow(
        "Legacy Working_Edited",
        false));
    CHECK_FALSE(WaterObjectProfileNameIsLegacyEditedShadow(
        "Legacy Working_Edited",
        true));
    CHECK_FALSE(WaterObjectProfileNameIsLegacyEditedShadow(
        "Creek edited",
        false));
    CHECK(WaterObjectProfileNameIsLegacyEditedShadow("_edited", false));
    CHECK(WaterObjectProfileNameIsLegacyEditedShadow("_Edited", false));
    CHECK_FALSE(WaterObjectProfileNameIsLegacyEditedShadow("_edited", true));
    CHECK_FALSE(WaterObjectProfileNameIsLegacyEditedShadow("_Edited", true));
}

TEST_CASE("Flow owner-copy promotion planning is safe and collision free",
          "[water][flow][profiles][promotion]") {
    using invisible_places::water::AllocateUniqueWaterObjectProfileName;
    using invisible_places::water::PlanWaterObjectProfilePromotion;
    using invisible_places::water::WaterObjectProfileBaseKind;
    using invisible_places::water::WaterObjectProfileEditDescriptor;
    using invisible_places::water::WaterObjectProfilePromotionFailure;
    using invisible_places::water::WaterObjectProfilePromotionOperation;

    WaterObjectProfileEditDescriptor owned{
        .assignedProfileName = "Calm Lanes_Creek",
        .exactBaseProfileName = "Calm Lanes",
        .suggestedSaveProfileName = "Calm Lanes",
        .removableWorkingProfileName = "Calm Lanes_Creek",
        .assignedObjectCopy = true,
        .ownedObjectCopy = true,
    };
    const std::vector<std::string> reserved{
        "Default",
        "Global",
        "Calm Lanes_preset",
        "Calm Lanes_Creek",
        "Storm",
        "Storm 2",
    };

    const auto saveShared = PlanWaterObjectProfilePromotion(
        owned,
        WaterObjectProfilePromotionOperation::Save,
        WaterObjectProfileBaseKind::Shared,
        "Ignored",
        reserved);
    REQUIRE(saveShared.allowed());
    CHECK(saveShared.targetProfileName == "Calm Lanes");
    CHECK(saveShared.overwriteExisting);
    CHECK_FALSE(saveShared.createShared);
    CHECK(saveShared.eraseWorkingCopy);

    auto defaultOwned = owned;
    defaultOwned.exactBaseProfileName = "Default";
    defaultOwned.suggestedSaveProfileName = "Default";
    const auto saveDefault = PlanWaterObjectProfilePromotion(
        defaultOwned,
        WaterObjectProfilePromotionOperation::Save,
        WaterObjectProfileBaseKind::Default,
        "Ignored",
        reserved);
    REQUIRE(saveDefault.allowed());
    CHECK(saveDefault.targetProfileName == "Default");
    CHECK(saveDefault.overwriteExisting);

    struct RejectedSave {
        WaterObjectProfileBaseKind baseKind;
        WaterObjectProfilePromotionFailure failure;
    };
    constexpr std::array rejectedSaves{
        RejectedSave{
            WaterObjectProfileBaseKind::Protected,
            WaterObjectProfilePromotionFailure::
                ProtectedBaseRequiresSaveAs},
        RejectedSave{
            WaterObjectProfileBaseKind::Missing,
            WaterObjectProfilePromotionFailure::
                MissingBaseRequiresSaveAs},
        RejectedSave{
            WaterObjectProfileBaseKind::ObjectCopy,
            WaterObjectProfilePromotionFailure::
                ObjectCopyBaseRequiresSaveAs},
    };
    for (const auto& rejected : rejectedSaves) {
        const auto plan = PlanWaterObjectProfilePromotion(
            owned,
            WaterObjectProfilePromotionOperation::Save,
            rejected.baseKind,
            "Ignored",
            reserved);
        CHECK_FALSE(plan.allowed());
        CHECK(plan.failure == rejected.failure);
        CHECK_FALSE(plan.eraseWorkingCopy);
    }

    const auto saveAs = PlanWaterObjectProfilePromotion(
        owned,
        WaterObjectProfilePromotionOperation::SaveAs,
        WaterObjectProfileBaseKind::Protected,
        " Storm_preset_edited ",
        reserved);
    REQUIRE(saveAs.allowed());
    CHECK(saveAs.targetProfileName == "Storm 3");
    CHECK_FALSE(saveAs.overwriteExisting);
    CHECK(saveAs.createShared);
    CHECK(saveAs.eraseWorkingCopy);

    const std::vector<std::string> denseReserved{
        "Storm",
        "Storm 2",
        "Storm 3",
        "Storm 4",
        "Storm Copy",
    };
    const auto denseSaveAs = PlanWaterObjectProfilePromotion(
        owned,
        WaterObjectProfilePromotionOperation::SaveAs,
        WaterObjectProfileBaseKind::Shared,
        "Storm",
        denseReserved);
    REQUIRE(denseSaveAs.allowed());
    CHECK(denseSaveAs.targetProfileName == "Storm 5");
    CHECK(std::find(
              denseReserved.begin(),
              denseReserved.end(),
              denseSaveAs.targetProfileName) == denseReserved.end());
    CHECK(AllocateUniqueWaterObjectProfileName("Storm", denseReserved) ==
          "Storm 5");
    for (const std::string_view protectedName : {
             "Aerial_preset",
             "Calm Lanes_preset",
             "Water Flow_preset",
             "Wet Rock_preset",
         }) {
        INFO(protectedName);
        CHECK(AllocateUniqueWaterObjectProfileName(protectedName, {}) ==
              std::string{protectedName} + " 2");
    }

    const auto blankSaveAs = PlanWaterObjectProfilePromotion(
        owned,
        WaterObjectProfilePromotionOperation::SaveAs,
        WaterObjectProfileBaseKind::Missing,
        "_edited",
        reserved);
    REQUIRE(blankSaveAs.allowed());
    CHECK(blankSaveAs.targetProfileName == "Calm Lanes");

    const auto discardPreset = PlanWaterObjectProfilePromotion(
        WaterObjectProfileEditDescriptor{
            .assignedProfileName = "Calm_Creek",
            .exactBaseProfileName = "Calm Lanes_preset",
            .suggestedSaveProfileName = "Calm Lanes",
            .removableWorkingProfileName = "Calm_Creek",
            .assignedObjectCopy = true,
            .ownedObjectCopy = true,
        },
        WaterObjectProfilePromotionOperation::Discard,
        WaterObjectProfileBaseKind::Protected,
        "Ignored",
        reserved);
    REQUIRE(discardPreset.allowed());
    CHECK(discardPreset.targetProfileName == "Calm Lanes_preset");
    CHECK(discardPreset.eraseWorkingCopy);

    const auto missingDiscard = PlanWaterObjectProfilePromotion(
        owned,
        WaterObjectProfilePromotionOperation::Discard,
        WaterObjectProfileBaseKind::Missing,
        "Ignored",
        reserved);
    CHECK_FALSE(missingDiscard.allowed());
    CHECK(missingDiscard.failure ==
          WaterObjectProfilePromotionFailure::MissingDiscardBase);

    auto foreign = owned;
    foreign.ownedObjectCopy = false;
    foreign.removableWorkingProfileName.clear();
    const auto foreignSaveAs = PlanWaterObjectProfilePromotion(
        foreign,
        WaterObjectProfilePromotionOperation::SaveAs,
        WaterObjectProfileBaseKind::Shared,
        "Foreign",
        reserved);
    CHECK_FALSE(foreignSaveAs.allowed());
    CHECK(foreignSaveAs.failure ==
          WaterObjectProfilePromotionFailure::NotOwnedWorkingCopy);
}

TEST_CASE("Flow owner-copy promotion transaction writes rewrites then erases exact owner",
          "[water][flow][profiles][promotion]") {
    using invisible_places::water::PlanWaterObjectProfilePromotion;
    using invisible_places::water::RunWaterObjectProfilePromotionTransaction;
    using invisible_places::water::WaterObjectProfileBaseKind;
    using invisible_places::water::WaterObjectProfileEditDescriptor;
    using invisible_places::water::WaterObjectProfilePromotionOperation;

    struct TestProfile {
        std::string name;
        int value = 0;
        bool objectOverride = false;
        std::uint32_t ownerObjectId = 0U;
    };
    std::vector<TestProfile> profiles{
        {.name = "Calm", .value = 1},
        {
            .name = "Calm_Creek",
            .value = 17,
            .objectOverride = true,
            .ownerObjectId = 41U,
        },
        {
            .name = "Calm_Creek",
            .value = 23,
            .objectOverride = true,
            .ownerObjectId = 99U,
        },
    };
    std::vector<std::string> references{
        "Calm_Creek",
        "Other",
        "Calm_Creek",
    };
    const std::vector<std::string> reserved{
        "Default",
        "Global",
        "Calm",
        "Calm_Creek",
    };
    const auto plan = PlanWaterObjectProfilePromotion(
        WaterObjectProfileEditDescriptor{
            .assignedProfileName = "Calm_Creek",
            .exactBaseProfileName = "Calm",
            .suggestedSaveProfileName = "Calm",
            .removableWorkingProfileName = "Calm_Creek",
            .assignedObjectCopy = true,
            .ownedObjectCopy = true,
        },
        WaterObjectProfilePromotionOperation::Save,
        WaterObjectProfileBaseKind::Shared,
        "Ignored",
        reserved);
    REQUIRE(plan.allowed());
    REQUIRE(plan.targetProfileName == "Calm");

    const int snapshottedValue = profiles[1].value;
    std::vector<std::string> phases;
    const bool applied = RunWaterObjectProfilePromotionTransaction(
        plan,
        {
            .writeTarget = [&](std::string_view target) {
                phases.emplace_back("write");
                const auto found = std::find_if(
                    profiles.begin(),
                    profiles.end(),
                    [&](const TestProfile& profile) {
                        return !profile.objectOverride &&
                               profile.name == target;
                    });
                if (found == profiles.end()) {
                    return false;
                }
                found->value = snapshottedValue;
                return true;
            },
            .rewriteReferences = [&] (
                std::string_view previous,
                std::string_view next) {
                phases.emplace_back("rewrite");
                REQUIRE(std::any_of(
                    profiles.begin(),
                    profiles.end(),
                    [&](const TestProfile& profile) {
                        return profile.name == next &&
                               profile.value == snapshottedValue;
                    }));
                for (auto& reference : references) {
                    if (reference == previous) {
                        reference = next;
                    }
                }
                return true;
            },
            .eraseWorkingCopy = [&](std::string_view working) {
                phases.emplace_back("erase");
                const auto found = std::find_if(
                    profiles.begin(),
                    profiles.end(),
                    [&](const TestProfile& profile) {
                        return profile.objectOverride &&
                               profile.ownerObjectId == 41U &&
                               profile.name == working;
                    });
                if (found == profiles.end()) {
                    return false;
                }
                profiles.erase(found);
                return true;
            },
        });

    REQUIRE(applied);
    CHECK(phases == std::vector<std::string>{"write", "rewrite", "erase"});
    CHECK(references ==
          std::vector<std::string>{"Calm", "Other", "Calm"});
    CHECK(profiles.front().value == snapshottedValue);
    CHECK(std::none_of(
        profiles.begin(),
        profiles.end(),
        [](const TestProfile& profile) {
            return profile.objectOverride &&
                   profile.ownerObjectId == 41U;
        }));
    CHECK(std::any_of(
        profiles.begin(),
        profiles.end(),
        [](const TestProfile& profile) {
            return profile.objectOverride &&
                   profile.ownerObjectId == 99U &&
                   profile.name == "Calm_Creek";
        }));

    SECTION("Discard skips target write") {
        auto discard = plan;
        discard.operation = WaterObjectProfilePromotionOperation::Discard;
        std::vector<std::string> discardPhases;
        const bool discarded = RunWaterObjectProfilePromotionTransaction(
            discard,
            {
                .writeTarget = [&](std::string_view) {
                    discardPhases.emplace_back("write");
                    return true;
                },
                .rewriteReferences = [&] (
                    std::string_view,
                    std::string_view) {
                    discardPhases.emplace_back("rewrite");
                    return true;
                },
                .eraseWorkingCopy = [&](std::string_view) {
                    discardPhases.emplace_back("erase");
                    return true;
                },
            });
        CHECK(discarded);
        CHECK(discardPhases ==
              std::vector<std::string>{"rewrite", "erase"});
    }

    SECTION("A failed callback suppresses only later phases") {
        struct FailedRun {
            bool result = false;
            std::vector<std::string> attempted;
            int targetValue = 0;
            std::string reference = "Calm_Creek";
            bool erased = false;
        };
        const auto runWithFailure = [&](std::string_view failedPhase) {
            FailedRun run;
            run.result = RunWaterObjectProfilePromotionTransaction(
                plan,
                {
                    .writeTarget = [&](std::string_view) {
                        run.attempted.emplace_back("write");
                        if (failedPhase == "write") {
                            return false;
                        }
                        run.targetValue = snapshottedValue;
                        return true;
                    },
                    .rewriteReferences = [&] (
                        std::string_view,
                        std::string_view next) {
                        run.attempted.emplace_back("rewrite");
                        if (failedPhase == "rewrite") {
                            return false;
                        }
                        run.reference = next;
                        return true;
                    },
                    .eraseWorkingCopy = [&](std::string_view) {
                        run.attempted.emplace_back("erase");
                        if (failedPhase == "erase") {
                            return false;
                        }
                        run.erased = true;
                        return true;
                    },
                });
            return run;
        };

        const auto writeFailure = runWithFailure("write");
        CHECK_FALSE(writeFailure.result);
        CHECK(writeFailure.attempted ==
              std::vector<std::string>{"write"});
        CHECK(writeFailure.targetValue == 0);
        CHECK(writeFailure.reference == "Calm_Creek");
        CHECK_FALSE(writeFailure.erased);

        const auto rewriteFailure = runWithFailure("rewrite");
        CHECK_FALSE(rewriteFailure.result);
        CHECK(rewriteFailure.attempted ==
              std::vector<std::string>{"write", "rewrite"});
        CHECK(rewriteFailure.targetValue == snapshottedValue);
        CHECK(rewriteFailure.reference == "Calm_Creek");
        CHECK_FALSE(rewriteFailure.erased);

        const auto eraseFailure = runWithFailure("erase");
        CHECK_FALSE(eraseFailure.result);
        CHECK(eraseFailure.attempted ==
              std::vector<std::string>{"write", "rewrite", "erase"});
        CHECK(eraseFailure.targetValue == snapshottedValue);
        CHECK(eraseFailure.reference == "Calm");
        CHECK_FALSE(eraseFailure.erased);
    }

    SECTION("Missing callbacks fail preflight before any mutation") {
        std::vector<std::string> attempted;
        const bool result = RunWaterObjectProfilePromotionTransaction(
            plan,
            {
                .writeTarget = [&](std::string_view) {
                    attempted.emplace_back("write");
                    return true;
                },
                .rewriteReferences = [&] (
                    std::string_view,
                    std::string_view) {
                    attempted.emplace_back("rewrite");
                    return true;
                },
                .eraseWorkingCopy = {},
            });
        CHECK_FALSE(result);
        CHECK(attempted.empty());
    }
}

TEST_CASE("Flow keyed profile base rewrites preserve package identity and provenance",
          "[water][flow][timing][keyed][profiles][references]") {
    using invisible_places::water::
        ReplaceWaterKeyedSettingsProfileBaseReferences;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;

    std::vector<WaterKeyedSettingsProfile> profiles{
        {
            .name = "Working_Path A",
            .baseProfileName = " Working ",
            .ownerObjectName = "Path A",
            .sourceProfileName = "Template A",
            .ownerObjectId = 10U,
            .featureKind = WaterKeyedFeatureKind::FlowPath,
            .edited = false,
            .nativeLengthFraction = 0.4F,
            .settings = {{
                .settingId = "speed",
                .profileGroup = "flow_path",
                .profileName = "Working",
                .keys = {{.position = 0.25F, .value = 0.5F}},
            }},
        },
        {
            .name = "Working_Path A_edited",
            .baseProfileName = "Working",
            .ownerObjectName = "Path A",
            .sourceProfileName = "Working_Path A",
            .ownerObjectId = 10U,
            .featureKind = WaterKeyedFeatureKind::FlowPath,
            .edited = true,
        },
        {
            .name = "Seepage Working",
            .baseProfileName = "Working",
            .featureKind = WaterKeyedFeatureKind::SeepageNode,
        },
    };

    const auto originalName = profiles[0].name;
    const auto originalSource = profiles[0].sourceProfileName;
    const auto originalKey = profiles[0].settings[0].keys[0];
    CHECK(ReplaceWaterKeyedSettingsProfileBaseReferences(
              profiles,
              WaterKeyedFeatureKind::FlowPath,
              "Working",
              "Saved Lane") == 2U);
    CHECK(profiles[0].baseProfileName == "Saved Lane");
    CHECK(profiles[1].baseProfileName == "Saved Lane");
    CHECK(profiles[2].baseProfileName == "Working");
    CHECK(profiles[0].name == originalName);
    CHECK(profiles[0].sourceProfileName == originalSource);
    CHECK(profiles[0].ownerObjectId == 10U);
    CHECK(profiles[0].nativeLengthFraction == Catch::Approx(0.4F));
    CHECK(profiles[0].settings[0].profileName == "Working");
    CHECK(profiles[0].settings[0].keys[0].position ==
          Catch::Approx(originalKey.position));
    CHECK(profiles[0].settings[0].keys[0].value ==
          Catch::Approx(originalKey.value));
}

TEST_CASE("Dynamic keyed tracks follow renamed Seepage profiles",
          "[water][timing][keyed][profiles][references]") {
    using invisible_places::water::
        ReplaceWaterKeyedSettingProfileReferences;
    using invisible_places::water::WaterKeyedSettingTrack;

    std::vector<WaterKeyedSettingTrack> tracks{
        {.settingId = "look.trickle_width",
         .profileGroup = "seepage_look",
         .profileName = " Old Look "},
        {.settingId = "look.breakup",
         .profileGroup = "seepage_look",
         .profileName = "Another Look"},
        {.settingId = "response.intensity",
         .profileGroup = "seepage_response",
         .profileName = "Old Look"},
        {.settingId = "strength",
         .profileName = "Old Look"},
        {.settingId = "strength",
         .profileGroup = "seepage_node_settings",
         .profileName = " Old Settings "},
    };

    CHECK(ReplaceWaterKeyedSettingProfileReferences(
              tracks,
              "seepage_look",
              "Old Look",
              "Renamed Look") == 1U);
    CHECK(tracks[0].profileName == "Renamed Look");
    CHECK(tracks[1].profileName == "Another Look");
    CHECK(tracks[2].profileName == "Old Look");
    CHECK(tracks[3].profileName == "Old Look");
    CHECK(tracks[4].profileName == " Old Settings ");

    CHECK(ReplaceWaterKeyedSettingProfileReferences(
              tracks,
              "seepage_node_settings",
              "Old Settings",
              "Renamed Settings") == 1U);
    CHECK(tracks[0].profileName == "Renamed Look");
    CHECK(tracks[2].profileName == "Old Look");
    CHECK(tracks[3].profileName == "Old Look");
    CHECK(tracks[4].profileName == "Renamed Settings");

    CHECK(ReplaceWaterKeyedSettingProfileReferences(
              tracks,
              "seepage_look",
              "",
              "Ignored") == 0U);
    CHECK(tracks[0].profileName == "Renamed Look");
    CHECK(tracks[1].profileName == "Another Look");
}

TEST_CASE("Seepage profile reference transactions include dormant tracks and packages",
          "[water][timing][keyed][profiles][references]") {
    using invisible_places::timing::ReplaceTimingWaterProfileReferences;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingsProfile;
    using invisible_places::water::WaterScenarioFeatureRuns;

    const auto feature = invisible_places::water::WaterKeyedFeatureId{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 20U,
    };
    WaterFeatureTimingRun legacyRun{
        .id = 1U,
        .features = {{
            .feature = feature,
            .settings = {
                {.settingId = "look.wetness",
                 .active = true,
                 .profileGroup = "seepage_look",
                 .profileName = "Old Look",
                 .keys = {{.position = 0.2F, .value = 0.4F}}},
                {.settingId = "look.glisten",
                 .active = false,
                 .profileGroup = "seepage_look",
                 .profileName = "Old Look",
                 .keys = {{.position = 0.7F, .value = 0.8F}}},
                {.settingId = "response.intensity",
                 .active = false,
                 .profileGroup = "seepage_response",
                 .profileName = "Old Look"},
            },
        }},
    };
    std::vector<WaterScenarioFeatureRuns> legacyScenarios{{
        .scenarioId = "legacy",
        .runs = {legacyRun},
    }};
    std::vector<TimingTakeSceneState> takeStates{{
        .takeId = "take-1",
        .waterFeatureTimingRuns = {{
            .id = 2U,
            .features = {{
                .feature = feature,
                .settings = {{
                    .settingId = "look.breakup",
                    .active = false,
                    .profileGroup = "seepage_look",
                    .profileName = "Old Look",
                    .keys = {{.position = 0.5F, .value = 0.6F}},
                }},
            }},
        }},
    }};
    std::vector<WaterKeyedSettingsProfile> packages{{
        .name = "Finish",
        .featureKind = WaterKeyedFeatureKind::SeepageNode,
        .settings = {{
            .settingId = "look.density",
            .active = false,
            .profileGroup = "seepage_look",
            .profileName = "Old Look",
            .keys = {{.position = 1.0F, .value = 0.0F}},
        }},
    }};

    const auto counts = ReplaceTimingWaterProfileReferences(
        legacyScenarios,
        takeStates,
        packages,
        "seepage_look",
        "Old Look",
        "Saved Look");
    CHECK(counts.legacyScenarioTracks == 2U);
    CHECK(counts.timingTakeTracks == 1U);
    CHECK(counts.keyedPackageTracks == 1U);
    CHECK(counts.total() == 4U);
    CHECK(legacyScenarios[0].runs[0].features[0].settings[0].profileName ==
          "Saved Look");
    CHECK(legacyScenarios[0].runs[0].features[0].settings[1].profileName ==
          "Saved Look");
    CHECK_FALSE(
        legacyScenarios[0].runs[0].features[0].settings[1].active);
    CHECK(legacyScenarios[0].runs[0].features[0].settings[1].keys[0].position ==
          Catch::Approx(0.7F));
    CHECK(legacyScenarios[0].runs[0].features[0].settings[2].profileName ==
          "Old Look");
    CHECK(takeStates[0].waterFeatureTimingRuns[0]
              .features[0]
              .settings[0]
              .profileName == "Saved Look");
    CHECK_FALSE(takeStates[0]
                    .waterFeatureTimingRuns[0]
                    .features[0]
                    .settings[0]
                    .active);
    CHECK(packages[0].settings[0].profileName == "Saved Look");
    CHECK_FALSE(packages[0].settings[0].active);
    CHECK(packages[0].settings[0].keys[0].value == Catch::Approx(0.0F));
}

TEST_CASE("Seepage transactions classify blank legacy metadata in every persisted store",
          "[water][timing][keyed][profiles][references][migration]") {
    using invisible_places::timing::ReplaceTimingWaterProfileReferences;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;
    using invisible_places::water::WaterScenarioFeatureRuns;

    struct ProfileHalfCase {
        std::string_view group;
        std::string_view settingId;
        std::string_view unrelatedSettingId;
    };
    constexpr std::array profileHalves{
        ProfileHalfCase{
            .group = "seepage_node_settings",
            .settingId = "strength",
            .unrelatedSettingId = "look.wetness",
        },
        ProfileHalfCase{
            .group = "seepage_look",
            .settingId = "look.wetness",
            .unrelatedSettingId = "response.intensity",
        },
        ProfileHalfCase{
            .group = "seepage_response",
            .settingId = "response.intensity",
            .unrelatedSettingId = "look.wetness",
        },
    };

    const WaterKeyedFeatureId seepageFeature{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 20U,
    };
    const WaterKeyedFeatureId flowFeature{
        .kind = WaterKeyedFeatureKind::FlowPath,
        .objectId = 30U,
    };
    for (const auto& profileHalf : profileHalves) {
        CAPTURE(profileHalf.group, profileHalf.settingId);
        std::vector<WaterScenarioFeatureRuns> legacyScenarios{{
            .scenarioId = "legacy",
            .runs = {{
                .id = 1U,
                .features = {
                    {
                        .feature = seepageFeature,
                        .settings = {
                            {.settingId = std::string{profileHalf.settingId},
                             .active = false,
                             .profileName = "Working"},
                            {.settingId = std::string{
                                 profileHalf.unrelatedSettingId},
                             .active = false,
                             .profileName = "Working"},
                        },
                    },
                    {
                        .feature = flowFeature,
                        .settings = {{
                            .settingId = std::string{profileHalf.settingId},
                            .active = false,
                            .profileName = "Working",
                        }},
                    },
                },
            }},
        }};
        std::vector<TimingTakeSceneState> takeStates{{
            .takeId = "take-1",
            .waterFeatureTimingRuns = {{
                .id = 2U,
                .features = {{
                    .feature = seepageFeature,
                    .settings = {
                        {.settingId = std::string{profileHalf.settingId},
                         .active = false,
                         .profileName = "Working"},
                        {.settingId = std::string{
                             profileHalf.unrelatedSettingId},
                         .active = false,
                         .profileName = "Working"},
                    },
                }},
            }},
        }};
        std::vector<WaterKeyedSettingsProfile> packages{
            {
                .name = "Seepage Finish",
                .featureKind = WaterKeyedFeatureKind::SeepageNode,
                .settings = {
                    {.settingId = std::string{profileHalf.settingId},
                     .active = false,
                     .profileName = "Working"},
                    {.settingId = std::string{
                         profileHalf.unrelatedSettingId},
                     .active = false,
                     .profileName = "Working"},
                },
            },
            {
                .name = "Flow Finish",
                .featureKind = WaterKeyedFeatureKind::FlowPath,
                .settings = {{
                    .settingId = std::string{profileHalf.settingId},
                    .active = false,
                    .profileName = "Working",
                }},
            },
        };

        const auto counts = ReplaceTimingWaterProfileReferences(
            legacyScenarios,
            takeStates,
            packages,
            profileHalf.group,
            "Working",
            "Saved");
        CHECK(counts.legacyScenarioTracks == 1U);
        CHECK(counts.timingTakeTracks == 1U);
        CHECK(counts.keyedPackageTracks == 1U);

        const auto& legacySettings =
            legacyScenarios[0].runs[0].features[0].settings;
        CHECK(legacySettings[0].profileGroup == profileHalf.group);
        CHECK(legacySettings[0].profileName == "Saved");
        CHECK_FALSE(legacySettings[0].active);
        CHECK(legacySettings[1].profileGroup.empty());
        CHECK(legacySettings[1].profileName == "Working");
        CHECK(legacyScenarios[0]
                  .runs[0]
                  .features[1]
                  .settings[0]
                  .profileGroup.empty());
        CHECK(legacyScenarios[0]
                  .runs[0]
                  .features[1]
                  .settings[0]
                  .profileName == "Working");

        const auto& takeSettings = takeStates[0]
                                       .waterFeatureTimingRuns[0]
                                       .features[0]
                                       .settings;
        CHECK(takeSettings[0].profileGroup == profileHalf.group);
        CHECK(takeSettings[0].profileName == "Saved");
        CHECK_FALSE(takeSettings[0].active);
        CHECK(takeSettings[1].profileGroup.empty());
        CHECK(takeSettings[1].profileName == "Working");

        CHECK(packages[0].settings[0].profileGroup == profileHalf.group);
        CHECK(packages[0].settings[0].profileName == "Saved");
        CHECK_FALSE(packages[0].settings[0].active);
        CHECK(packages[0].settings[1].profileGroup.empty());
        CHECK(packages[0].settings[1].profileName == "Working");
        CHECK(packages[1].settings[0].profileGroup.empty());
        CHECK(packages[1].settings[0].profileName == "Working");
    }
}

TEST_CASE("Flow Path transactions classify blank legacy metadata in every persisted store",
          "[water][flow][timing][keyed][profiles][references][migration]") {
    using invisible_places::timing::ReplaceTimingWaterProfileReferences;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;
    using invisible_places::water::WaterScenarioFeatureRuns;

    const auto flowPath = invisible_places::water::WaterKeyedFeatureId{
        .kind = WaterKeyedFeatureKind::FlowPath,
        .objectId = 30U,
    };
    const auto flowSource = invisible_places::water::WaterKeyedFeatureId{
        .kind = WaterKeyedFeatureKind::FlowSource,
        .objectId = 31U,
    };
    std::vector<WaterScenarioFeatureRuns> legacyScenarios{{
        .scenarioId = "legacy",
        .runs = {{
            .id = 1U,
            .features = {
                {
                    .feature = flowPath,
                    .settings = {
                        {.settingId = "speed",
                         .active = false,
                         .profileName = "Working Lane",
                         .keys = {{.position = 0.2F, .value = 0.4F}}},
                        {.settingId = "not_a_flow_path_setting",
                         .active = false,
                         .profileName = "Working Lane"},
                        {.settingId = "trail_width",
                         .active = false,
                         .profileGroup = "trail",
                         .profileName = "Working Lane"},
                    },
                },
                {
                    .feature = flowSource,
                    .settings = {{
                        .settingId = "strength",
                        .active = false,
                        .profileName = "Working Lane",
                    }},
                },
            },
        }},
    }};
    std::vector<TimingTakeSceneState> takeStates{{
        .takeId = "take-1",
        .waterFeatureTimingRuns = {{
            .id = 2U,
            .features = {{
                .feature = flowPath,
                .settings = {{
                    .settingId = "trail_streak_length",
                    .active = false,
                    .profileName = "Working Lane",
                    .keys = {{.position = 0.7F, .value = 0.8F}},
                }},
            }},
        }},
    }};
    std::vector<WaterKeyedSettingsProfile> packages{
        {
            .name = "Flow Finish",
            .baseProfileName = "Working Lane",
            .sourceProfileName = "Flow Start",
            .ownerObjectId = 30U,
            .featureKind = WaterKeyedFeatureKind::FlowPath,
            .nativeLengthFraction = 0.3F,
            .settings = {{
                .settingId = "trail_width",
                .active = false,
                .profileName = "Working Lane",
                .keys = {{.position = 1.0F, .value = 0.0F}},
            }},
        },
        {
            .name = "Seepage Finish",
            .baseProfileName = "Working Lane",
            .featureKind = WaterKeyedFeatureKind::SeepageNode,
            .settings = {{
                .settingId = "strength",
                .active = false,
                .profileName = "Working Lane",
            }},
        },
    };

    const auto counts = ReplaceTimingWaterProfileReferences(
        legacyScenarios,
        takeStates,
        packages,
        "flow_path",
        "Working Lane",
        "Saved Lane");
    CHECK(counts.legacyScenarioTracks == 1U);
    CHECK(counts.timingTakeTracks == 1U);
    CHECK(counts.keyedPackageTracks == 1U);
    const auto& legacyTracks =
        legacyScenarios[0].runs[0].features[0].settings;
    CHECK(legacyTracks[0].profileGroup == "flow_path");
    CHECK(legacyTracks[0].profileName == "Saved Lane");
    CHECK_FALSE(legacyTracks[0].active);
    CHECK(legacyTracks[0].keys[0].position == Catch::Approx(0.2F));
    CHECK(legacyTracks[1].profileGroup.empty());
    CHECK(legacyTracks[1].profileName == "Working Lane");
    CHECK(legacyTracks[2].profileGroup == "trail");
    CHECK(legacyTracks[2].profileName == "Working Lane");
    CHECK(legacyScenarios[0]
              .runs[0]
              .features[1]
              .settings[0]
              .profileName == "Working Lane");
    const auto& takeTrack = takeStates[0]
                                .waterFeatureTimingRuns[0]
                                .features[0]
                                .settings[0];
    CHECK(takeTrack.profileGroup == "flow_path");
    CHECK(takeTrack.profileName == "Saved Lane");
    CHECK(takeTrack.keys[0].value == Catch::Approx(0.8F));
    CHECK(packages[0].settings[0].profileGroup == "flow_path");
    CHECK(packages[0].settings[0].profileName == "Saved Lane");
    CHECK(packages[0].name == "Flow Finish");
    CHECK(packages[0].sourceProfileName == "Flow Start");
    CHECK(packages[0].nativeLengthFraction == Catch::Approx(0.3F));
    CHECK(packages[0].baseProfileName == "Working Lane");
    CHECK(packages[1].settings[0].profileGroup.empty());
    CHECK(packages[1].settings[0].profileName == "Working Lane");
}

TEST_CASE("Legacy uppercase Seepage edits promote every exact reference before removal",
          "[water][timing][keyed][profiles][references][migration]") {
    using invisible_places::timing::ReplaceTimingWaterProfileReferences;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::DescribeWaterObjectProfileEdit;
    using invisible_places::water::ReplaceWaterSeepageNodeProfileReferences;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;
    using invisible_places::water::WaterScenarioFeatureRuns;
    using invisible_places::water::WaterSeepageNode;
    using invisible_places::water::WaterSeepageProfileHalf;

    const auto descriptor = DescribeWaterObjectProfileEdit(
        " Wet Rock_Edited ",
        20U);
    REQUIRE(descriptor.legacyEditedShadow);
    CHECK(descriptor.removableWorkingProfileName == "Wet Rock_Edited");
    CHECK(descriptor.exactBaseProfileName == "Wet Rock");

    const auto feature = invisible_places::water::WaterKeyedFeatureId{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 20U,
    };
    std::vector<WaterSeepageNode> nodes(2U);
    nodes[0].id = 20U;
    nodes[0].lookProfileName = " Wet Rock_Edited ";
    nodes[1].id = 21U;
    nodes[1].lookProfileName = "Wet Rock_edited";
    std::vector<WaterScenarioFeatureRuns> legacyScenarios{{
        .scenarioId = "legacy",
        .runs = {{
            .id = 1U,
            .features = {{
                .feature = feature,
                .settings = {{
                    .settingId = "look.wetness",
                    .active = false,
                    .profileGroup = "seepage_look",
                    .profileName = "Wet Rock_Edited",
                }, {
                    .settingId = "look.density",
                    .active = false,
                    .profileGroup = "seepage_look",
                    .profileName = "Wet Rock_edited",
                }},
            }},
        }},
    }};
    std::vector<TimingTakeSceneState> takeStates{{
        .takeId = "take-1",
        .waterFeatureTimingRuns = {{
            .id = 2U,
            .features = {{
                .feature = feature,
                .settings = {{
                    .settingId = "look.glisten",
                    .active = true,
                    .profileGroup = "seepage_look",
                    .profileName = "Wet Rock_Edited",
                }},
            }},
        }},
    }};
    std::vector<WaterKeyedSettingsProfile> packages{{
        .name = "Finish",
        .featureKind = WaterKeyedFeatureKind::SeepageNode,
        .settings = {{
            .settingId = "look.density",
            .active = false,
            .profileGroup = "seepage_look",
            .profileName = "Wet Rock_Edited",
        }, {
            .settingId = "look.breakup",
            .active = false,
            .profileGroup = "seepage_look",
            .profileName = "Wet Rock_edited",
        }},
    }};

    CHECK(ReplaceWaterSeepageNodeProfileReferences(
              nodes,
              WaterSeepageProfileHalf::Look,
              descriptor.removableWorkingProfileName,
              descriptor.exactBaseProfileName) == 1U);
    const auto counts = ReplaceTimingWaterProfileReferences(
        legacyScenarios,
        takeStates,
        packages,
        "seepage_look",
        descriptor.removableWorkingProfileName,
        descriptor.exactBaseProfileName);
    CHECK(counts.legacyScenarioTracks == 1U);
    CHECK(counts.timingTakeTracks == 1U);
    CHECK(counts.keyedPackageTracks == 1U);
    CHECK(nodes[0].lookProfileName == "Wet Rock");
    CHECK(nodes[1].lookProfileName == "Wet Rock_edited");
    CHECK(legacyScenarios[0].runs[0].features[0].settings[0].profileName ==
          "Wet Rock");
    CHECK(legacyScenarios[0].runs[0].features[0].settings[1].profileName ==
          "Wet Rock_edited");
    CHECK(takeStates[0].waterFeatureTimingRuns[0]
              .features[0]
              .settings[0]
              .profileName == "Wet Rock");
    CHECK(packages[0].settings[0].profileName == "Wet Rock");
    CHECK(packages[0].settings[1].profileName == "Wet Rock_edited");
}

TEST_CASE("Blank legacy Seepage track metadata is classified per profile half",
          "[water][timing][keyed][profiles][references][migration]") {
    using invisible_places::water::CanonicalizeWaterFeatureProfileMetadata;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;

    const WaterKeyedFeatureId selected{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 20U,
    };
    std::vector<WaterFeatureTimingRun> runs{{
        .id = 1U,
        .features = {{
            .feature = selected,
            .settings = {
                {.settingId = "strength",
                 .active = false,
                 .profileName = "Stale Settings"},
                {.settingId = "look.trickle_width",
                 .active = false,
                 .profileName = "Stale Look"},
                {.settingId = "response.intensity",
                 .active = false,
                 .profileName = "Stale Response"},
            },
        }},
    }};
    const std::array features{selected};

    CHECK(CanonicalizeWaterFeatureProfileMetadata(
              runs,
              features,
              "seepage_node_settings",
              "Saved Settings") == 1U);
    CHECK(runs[0].features[0].settings[0].profileGroup ==
          "seepage_node_settings");
    CHECK(runs[0].features[0].settings[0].profileName == "Saved Settings");
    CHECK(runs[0].features[0].settings[1].profileGroup.empty());
    CHECK(runs[0].features[0].settings[2].profileGroup.empty());

    CHECK(CanonicalizeWaterFeatureProfileMetadata(
              runs,
              features,
              "seepage_look",
              "Saved Look") == 1U);
    CHECK(runs[0].features[0].settings[1].profileGroup == "seepage_look");
    CHECK(runs[0].features[0].settings[1].profileName == "Saved Look");
    CHECK(runs[0].features[0].settings[2].profileGroup.empty());

    CHECK(CanonicalizeWaterFeatureProfileMetadata(
              runs,
              features,
              "seepage_response",
              "Saved Response") == 1U);
    CHECK(runs[0].features[0].settings[2].profileGroup ==
          "seepage_response");
    CHECK(runs[0].features[0].settings[2].profileName ==
          "Saved Response");
    CHECK_FALSE(runs[0].features[0].settings[0].active);
    CHECK_FALSE(runs[0].features[0].settings[1].active);
    CHECK_FALSE(runs[0].features[0].settings[2].active);
}

TEST_CASE("Schema 46 timing tracks migrate with active legacy defaults",
          "[water][timing][keyed][serialization][migration]") {
    using invisible_places::serialization::kProjectDocumentSchemaVersion;
    using invisible_places::serialization::LoadProjectDocument;
    using invisible_places::serialization::SaveProjectDocument;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterScenarioInterpolation;

    // Schema 46 introduced per-feature timing tracks, before dormant-state
    // and dynamic profile display metadata became persistent in schema 47.
    const std::string schema46Json = R"({
        "schema_version": 46,
        "project_name": "Schema 46 Feature Timing",
        "water_feature_timing_runs": [
            {
                "scenario_id": "pre-colonisation-wet",
                "runs": [
                    {
                        "id": 3,
                        "name": "Legacy Seepage",
                        "features": [
                            {
                                "kind": "seepage_node",
                                "object_id": 42,
                                "settings": [
                                    {
                                        "id": "strength",
                                        "keys": [
                                            {
                                                "position": 0.25,
                                                "value": 0.6,
                                                "interpolation": "linear"
                                            }
                                        ]
                                    }
                                ]
                            }
                        ]
                    }
                ]
            }
        ],
        "water_feature_timing_run_sequence": 4
    })";

    TemporaryTimingFile file{"invisible_places_feature_timing_schema46.json"};
    {
        std::ofstream output{file.path};
        REQUIRE(output.good());
        output << schema46Json;
    }

    std::string errorMessage;
    auto loaded = LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == kProjectDocumentSchemaVersion);
    REQUIRE(loaded->waterFeatureTimingRuns.size() == 1U);
    REQUIRE(loaded->waterFeatureTimingRuns.front().runs.size() == 1U);
    const auto& run = loaded->waterFeatureTimingRuns.front().runs.front();
    CHECK(run.marks.empty());
    REQUIRE(run.features.size() == 1U);
    CHECK(run.features.front().feature.kind ==
          WaterKeyedFeatureKind::SeepageNode);
    CHECK(run.features.front().feature.objectId == 42U);
    REQUIRE(run.features.front().settings.size() == 1U);
    const auto& setting = run.features.front().settings.front();
    CHECK(setting.settingId == "strength");
    CHECK(setting.active);
    CHECK(setting.label.empty());
    CHECK(setting.profileGroup.empty());
    CHECK(setting.profileName.empty());
    REQUIRE(setting.keys.size() == 1U);
    CHECK(setting.keys.front().position == Approx(0.25F));
    CHECK(setting.keys.front().value == Approx(0.6F));
    CHECK(setting.keys.front().interpolation ==
          WaterScenarioInterpolation::Linear);

    // Rewriting the migrated project persists the schema-47 defaults, so a
    // second load is identical and no legacy key data is lost.
    REQUIRE(SaveProjectDocument(*loaded, file.path, &errorMessage));
    loaded = LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    CHECK(loaded->schemaVersion == kProjectDocumentSchemaVersion);
    const auto& roundTrippedSetting =
        loaded->waterFeatureTimingRuns.front()
            .runs.front()
            .features.front()
            .settings.front();
    CHECK(roundTrippedSetting.active);
    CHECK(roundTrippedSetting.label.empty());
    CHECK(roundTrippedSetting.profileGroup.empty());
    CHECK(roundTrippedSetting.profileName.empty());
    REQUIRE(roundTrippedSetting.keys.size() == 1U);
    CHECK(roundTrippedSetting.keys.front().value == Approx(0.6F));
}

TEST_CASE("Shoreline instances key level and visual response scalars", "[water][timing][shoreline]") {
    using invisible_places::water::FindWaterKeyableSetting;
    using invisible_places::water::ParseWaterKeyedFeatureKindName;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedFeatureKindIsGlobal;
    using invisible_places::water::WaterKeyedFeatureKindName;

    const auto settings = invisible_places::water::WaterKeyableSettings(
        WaterKeyedFeatureKind::ShorelineInstance);
    REQUIRE(settings.size() == 6U);
    const auto* level = FindWaterKeyableSetting(
        WaterKeyedFeatureKind::ShorelineInstance,
        "level");
    REQUIRE(level != nullptr);
    CHECK(level->maximum == Approx(1.0F));
    CHECK(level->defaultValue == Approx(1.0F));
    for (const char* settingId :
         {"intensity",
          "emission_add",
          "opacity_add",
          "point_size_multiply",
          "colour_mix"}) {
        REQUIRE(
            FindWaterKeyableSetting(
                WaterKeyedFeatureKind::ShorelineInstance,
                settingId) != nullptr);
    }
    const auto* opacityAdd = FindWaterKeyableSetting(
        WaterKeyedFeatureKind::ShorelineInstance,
        "opacity_add");
    REQUIRE(opacityAdd != nullptr);
    CHECK(opacityAdd->minimum == Approx(-1.0F));

    // Instances are per-object features whose serialized kind name
    // round-trips; unknown names still parse to nothing.
    CHECK_FALSE(
        WaterKeyedFeatureKindIsGlobal(
            WaterKeyedFeatureKind::ShorelineInstance));
    CHECK(
        WaterKeyedFeatureKindName(
            WaterKeyedFeatureKind::ShorelineInstance) ==
        "shoreline_instance");
    const auto parsed = ParseWaterKeyedFeatureKindName("shoreline_instance");
    REQUIRE(parsed.has_value());
    CHECK(parsed.value() == WaterKeyedFeatureKind::ShorelineInstance);
}

TEST_CASE("Keyed setting Monotone Spline passes keys, stays monotone, and rests at reversals",
          "[water][timing][keyed][velocity]") {
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrack;

    WaterKeyedSettingTrack track;
    track.settingId = "strength";
    track.keys = {
        {.position = 0.0F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.position = 0.25F,
         .value = 0.5F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.position = 1.0F,
         .value = 1.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
    };
    const auto valueAt = [&](float position) {
        return EvaluateWaterKeyedSettingTrack(track, position).value();
    };
    // Passes exactly through every key.
    CHECK(valueAt(0.0F) == Approx(0.0F).margin(1.0e-5F));
    CHECK(valueAt(0.25F) == Approx(0.5F).margin(1.0e-5F));
    CHECK(valueAt(1.0F) == Approx(1.0F).margin(1.0e-5F));
    // Between increasing keys the curve is monotone: dense samples never
    // step backwards and never overshoot the surrounding key values.
    float previous = valueAt(0.0F);
    for (int sample = 1; sample <= 400; ++sample) {
        const float position = static_cast<float>(sample) / 400.0F;
        const float value = valueAt(position);
        CHECK(value >= previous - 1.0e-4F);
        CHECK(value >= -1.0e-4F);
        CHECK(value <= 1.0F + 1.0e-4F);
        if (position <= 0.25F) {
            CHECK(value <= 0.5F + 1.0e-4F);
        } else {
            CHECK(value >= 0.5F - 1.0e-4F);
        }
        previous = value;
    }
    // Velocity carries through the continuing interior key (C1-ish): the
    // finite-difference slopes on both sides agree and stay near the
    // incoming speed instead of easing to rest like Smooth would.
    const float incoming = (valueAt(0.25F) - valueAt(0.249F)) / 0.001F;
    const float outgoing = (valueAt(0.251F) - valueAt(0.25F)) / 0.001F;
    CHECK(incoming > 1.0F);
    CHECK(outgoing > 1.0F);
    CHECK(incoming == Approx(outgoing).margin(1.0e-2F));

    WaterKeyedSettingTrack reversing;
    reversing.settingId = "strength";
    reversing.keys = {
        {.position = 0.0F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.position = 0.5F,
         .value = 1.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
        {.position = 1.0F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::SmoothVelocity},
    };
    const auto reversingAt = [&](float position) {
        return EvaluateWaterKeyedSettingTrack(reversing, position).value();
    };
    // The direction reversal comes to rest: C0 through the peak key with a
    // flat top on both sides.
    const float peak = reversingAt(0.5F);
    CHECK(peak == Approx(1.0F));
    CHECK(reversingAt(0.499F) == Approx(peak).margin(1.0e-3F));
    CHECK(reversingAt(0.501F) == Approx(peak).margin(1.0e-3F));
    const float approachVelocity = (peak - reversingAt(0.499F)) / 0.001F;
    const float departureVelocity = (reversingAt(0.501F) - peak) / 0.001F;
    CHECK(std::abs(approachVelocity) < 0.05F);
    CHECK(std::abs(departureVelocity) < 0.05F);
}

TEST_CASE("Keyed setting Centripetal Catmull-Rom stays finite through uneven keys",
          "[water][timing][keyed][catmull-rom]") {
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrack;

    WaterKeyedSettingTrack track;
    track.settingId = "strength";
    track.keys = {
        {.position = 0.0F,
         .value = 0.05F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
        {.position = 0.18F,
         .value = 0.42F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
        {.position = 0.55F,
         .value = 0.64F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
        {.position = 1.0F,
         .value = 0.90F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
    };
    const auto valueAt = [&](float position) {
        return EvaluateWaterKeyedSettingTrack(track, position).value();
    };
    // Passes exactly through the interior keys.
    CHECK(valueAt(0.18F) == Approx(0.42F).margin(1.0e-5F));
    CHECK(valueAt(0.55F) == Approx(0.64F).margin(1.0e-5F));
    // Uneven key spacing still produces finite, continuous values across the
    // whole timeline.
    float previous = valueAt(0.0F);
    for (int sample = 1; sample <= 500; ++sample) {
        const float position = static_cast<float>(sample) / 500.0F;
        const float value = valueAt(position);
        REQUIRE(std::isfinite(value));
        CHECK(std::abs(value - previous) < 0.02F);
        previous = value;
    }

    // A two-key track degrades gracefully through endpoint reflection.
    WaterKeyedSettingTrack pair;
    pair.settingId = "strength";
    pair.keys = {
        {.position = 0.2F,
         .value = 0.3F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
        {.position = 0.8F,
         .value = 0.7F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
    };
    const auto pairAt = [&](float position) {
        return EvaluateWaterKeyedSettingTrack(pair, position).value();
    };
    CHECK(pairAt(0.2F) == Approx(0.3F).margin(1.0e-5F));
    CHECK(pairAt(0.8F) == Approx(0.7F).margin(1.0e-5F));
    for (int sample = 0; sample <= 100; ++sample) {
        const float position =
            0.2F + 0.6F * static_cast<float>(sample) / 100.0F;
        const float value = pairAt(position);
        REQUIRE(std::isfinite(value));
        CHECK(value >= 0.3F - 1.0e-3F);
        CHECK(value <= 0.7F + 1.0e-3F);
    }
}

TEST_CASE("Keyed setting Spline Handles shape Bezier segments without losing handles",
          "[water][timing][keyed][spline-handles]") {
    using invisible_places::water::AddOrUpdateWaterSettingKey;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::MoveWaterSettingKey;
    using invisible_places::water::MoveWaterSettingSplineHandlePoint;
    using invisible_places::water::ResolveWaterSettingSplineHandlePoint;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterSettingSplineHandleSide;

    WaterKeyedSettingTrack track;
    track.settingId = "strength";
    track.defaultInterpolation = WaterScenarioInterpolation::SplineHandles;
    track.keys = {
        {.position = 0.0F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::TrackDefault},
        {.position = 1.0F,
         .value = 1.0F,
         .interpolation = WaterScenarioInterpolation::Linear},
    };

    // Default one-third handles reproduce Smooth Step exactly while still
    // exposing an editable outgoing and incoming control point.
    REQUIRE(EvaluateWaterKeyedSettingTrack(track, 0.25F).has_value());
    CHECK(
        EvaluateWaterKeyedSettingTrack(track, 0.25F).value() ==
        Approx(0.15625F).margin(1.0e-5F));
    const auto outgoing = ResolveWaterSettingSplineHandlePoint(
        track,
        0.0F,
        WaterSettingSplineHandleSide::Outgoing);
    const auto incoming = ResolveWaterSettingSplineHandlePoint(
        track,
        1.0F,
        WaterSettingSplineHandleSide::Incoming);
    REQUIRE(outgoing.has_value());
    REQUIRE(incoming.has_value());
    CHECK(outgoing->controlPosition == Approx(1.0F / 3.0F));
    CHECK(outgoing->controlValue == Approx(0.0F));
    CHECK(incoming->controlPosition == Approx(2.0F / 3.0F));
    CHECK(incoming->controlValue == Approx(1.0F));

    REQUIRE(MoveWaterSettingSplineHandlePoint(
        &track,
        0.0F,
        WaterSettingSplineHandleSide::Outgoing,
        1.0F / 3.0F,
        1.0F));
    CHECK(
        EvaluateWaterKeyedSettingTrack(track, 0.5F).value() ==
        Approx(0.875F).margin(1.0e-5F));

    // Ordinary value/interpolation edits preserve the authored handle, and
    // retiming the neighbour carries its segment-relative time coordinate.
    AddOrUpdateWaterSettingKey(
        &track,
        0.0F,
        0.2F,
        WaterScenarioInterpolation::TrackDefault);
    CHECK(track.keys.front().outgoingHandleValue == Approx(1.0F));
    REQUIRE(MoveWaterSettingKey(&track, 1.0F, 0.8F));
    const auto retimedIncoming = ResolveWaterSettingSplineHandlePoint(
        track,
        0.8F,
        WaterSettingSplineHandleSide::Incoming);
    REQUIRE(retimedIncoming.has_value());
    CHECK(retimedIncoming->controlPosition == Approx(0.8F * 2.0F / 3.0F));
}

TEST_CASE("Mixed Hold and Catmull-Rom segments step exactly at the shared key",
          "[water][timing][keyed][catmull-rom]") {
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::WaterKeyedSettingTrack;

    WaterKeyedSettingTrack track;
    track.settingId = "strength";
    track.keys = {
        {.position = 0.2F,
         .value = 0.0F,
         .interpolation = WaterScenarioInterpolation::Hold},
        {.position = 0.4F,
         .value = 1.2F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
        {.position = 0.7F,
         .value = 0.4F,
         .interpolation = WaterScenarioInterpolation::CentripetalCatmullRom},
        {.position = 0.9F,
         .value = 0.8F,
         .interpolation = WaterScenarioInterpolation::Linear},
    };
    const auto valueAt = [&](float position) {
        return EvaluateWaterKeyedSettingTrack(track, position).value();
    };
    // Strictly inside the Hold segment the left value holds...
    CHECK(valueAt(0.399F) == Approx(0.0F));
    // ...and sampling exactly AT the key after the Hold reads the spline
    // segment that key starts, so the step lands on the key.
    CHECK(valueAt(0.4F) == Approx(1.2F).margin(1.0e-5F));
    CHECK(valueAt(0.7F) == Approx(0.4F).margin(1.0e-5F));
    for (int sample = 0; sample <= 100; ++sample) {
        const float position =
            0.4F + 0.3F * static_cast<float>(sample) / 100.0F;
        REQUIRE(std::isfinite(valueAt(position)));
    }
}

TEST_CASE("Timing runs evaluate Monotone Spline and Catmull-Rom within clamped levels",
          "[water][timing][velocity][catmull-rom]") {
    const auto velocityRun = Run(
        WaterTimingFeature::Rain,
        {Key(0.0F, 0.0F, WaterScenarioInterpolation::SmoothVelocity),
         Key(0.25F, 0.5F, WaterScenarioInterpolation::SmoothVelocity),
         Key(1.0F, 1.0F, WaterScenarioInterpolation::SmoothVelocity)});
    CHECK(EvaluateWaterTimingRun(velocityRun, 0.25F, 0.75F) ==
          Approx(0.5F).margin(1.0e-5F));
    // Velocity is continuous through the continuing interior key.
    const float incoming =
        (EvaluateWaterTimingRun(velocityRun, 0.25F, 0.75F) -
         EvaluateWaterTimingRun(velocityRun, 0.249F, 0.75F)) /
        0.001F;
    const float outgoing =
        (EvaluateWaterTimingRun(velocityRun, 0.251F, 0.75F) -
         EvaluateWaterTimingRun(velocityRun, 0.25F, 0.75F)) /
        0.001F;
    CHECK(incoming > 1.0F);
    CHECK(incoming == Approx(outgoing).margin(1.0e-2F));

    const auto catmullRun = Run(
        WaterTimingFeature::Seepage,
        {Key(0.0F, 0.0F, WaterScenarioInterpolation::CentripetalCatmullRom),
         Key(0.3F, 1.0F, WaterScenarioInterpolation::CentripetalCatmullRom),
         Key(0.6F, 1.0F, WaterScenarioInterpolation::CentripetalCatmullRom),
         Key(1.0F, 0.2F, WaterScenarioInterpolation::CentripetalCatmullRom)});
    CHECK(EvaluateWaterTimingRun(catmullRun, 0.3F, 0.5F) ==
          Approx(1.0F).margin(1.0e-5F));
    CHECK(EvaluateWaterTimingRun(catmullRun, 0.6F, 0.5F) ==
          Approx(1.0F).margin(1.0e-5F));
    // Run output stays clamped 0..1 for both modes even where the raw
    // spline would overshoot the keyed levels.
    for (int sample = 0; sample <= 200; ++sample) {
        const float position = static_cast<float>(sample) / 200.0F;
        for (const auto* run : {&velocityRun, &catmullRun}) {
            const float value = EvaluateWaterTimingRun(*run, position, 0.5F);
            REQUIRE(std::isfinite(value));
            CHECK(value >= 0.0F);
            CHECK(value <= 1.0F);
        }
    }
}

TEST_CASE("Water trail Catmull-Rom implementations retain required knot extrapolation",
          "[water][flow][spline][shader]") {
    const auto sourceRoot =
        std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR}.parent_path();
    const auto readSource = [](const std::filesystem::path& path) {
        std::ifstream input{path};
        REQUIRE(input.good());
        return std::string{
            std::istreambuf_iterator<char>{input},
            std::istreambuf_iterator<char>{}};
    };

    const std::vector<std::filesystem::path> shaderPaths{
        sourceRoot / "shaders" / "water_flow_gpu_common.glsl",
        sourceRoot / "shaders" / "pointcloud_preview.vert",
        sourceRoot / "shaders" / "pointcloud_fast_basic.vert",
        sourceRoot / "shaders" / "pointcloud_surfel.vert",
    };
    for (const auto& shaderPath : shaderPaths) {
        const auto source = readSource(shaderPath);
        CAPTURE(shaderPath);
        CHECK(source.find("mix(a, b, (t - ta) / denominator)") !=
              std::string::npos);
        CHECK(source.find(
                  "mix(a, b, clamp((t - ta) / denominator") ==
              std::string::npos);
    }

    const auto offlineSource =
        readSource(sourceRoot / "src" / "output" / "OfflinePointRenderer.cpp");
    CHECK(offlineSource.find("glm::mix(a, b, (t - ta) / denominator)") !=
          std::string::npos);
    CHECK(offlineSource.find(
              "glm::mix(a, b, std::clamp((t - ta) / denominator") ==
          std::string::npos);
}

TEST_CASE("Manual Flow lane handle drags preserve explicit width modes",
          "[water][flow][manual-path][lane-width][handles]") {
    using invisible_places::water::ApplyWaterManualFlowPathLaneWidthHandleDrag;
    using invisible_places::water::WaterManualFlowPathLaneWidth;
    using invisible_places::water::WaterManualFlowPathLaneWidthMode;

    const auto absolute = ApplyWaterManualFlowPathLaneWidthHandleDrag(
        {.mode = WaterManualFlowPathLaneWidthMode::Absolute, .value = 0.20F},
        0.35F,
        0.50F);
    CHECK(absolute.mode == WaterManualFlowPathLaneWidthMode::Absolute);
    CHECK(absolute.value == Approx(0.35F));

    const auto relative = ApplyWaterManualFlowPathLaneWidthHandleDrag(
        {.mode = WaterManualFlowPathLaneWidthMode::Relative, .value = 2.30F},
        0.35F,
        0.50F);
    CHECK(relative.mode == WaterManualFlowPathLaneWidthMode::Relative);
    CHECK(relative.value == Approx(0.70F));

    const auto standard = ApplyWaterManualFlowPathLaneWidthHandleDrag(
        WaterManualFlowPathLaneWidth{},
        0.35F,
        0.50F);
    CHECK(standard.mode == WaterManualFlowPathLaneWidthMode::Relative);
    CHECK(standard.value == Approx(0.70F));

    const auto zeroBasisRelative =
        ApplyWaterManualFlowPathLaneWidthHandleDrag(
            {.mode = WaterManualFlowPathLaneWidthMode::Relative,
             .value = 2.30F},
            0.35F,
            0.0F);
    CHECK(
        zeroBasisRelative.mode ==
        WaterManualFlowPathLaneWidthMode::Relative);
    CHECK(zeroBasisRelative.value == Approx(2.30F));
}

TEST_CASE("Settings-clip span transforms offset and stretch grouped keys",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::TransformWaterFeatureTimelineSpan;
    using invisible_places::water::WaterFeatureSettingsClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimelineSpanLimits;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.20F, .value = 0.0F},
             {.position = 0.30F, .value = 1.0F},
             {.position = 0.40F, .value = 0.0F},
         }},
        // Dormant tracks move with their clip so re-enabling stays aligned.
        {.settingId = "prominence",
         .active = false,
         .keys = {{.position = 0.25F, .value = 1.5F}}},
    };
    timeline.clips = {{.id = 1U, .name = "On", .start = 0.20F, .end = 0.40F}};

    // Offset and double the span: 0.2..0.4 -> 0.5..0.9.
    REQUIRE(TransformWaterFeatureTimelineSpan(
        &timeline,
        0.20F,
        0.40F,
        0.50F,
        0.90F));
    CHECK(timeline.settings[0].keys[0].position == Approx(0.50F));
    CHECK(timeline.settings[0].keys[1].position == Approx(0.70F));
    CHECK(timeline.settings[0].keys[2].position == Approx(0.90F));
    CHECK(timeline.settings[1].keys[0].position == Approx(0.60F));
    CHECK(timeline.clips.front().start == Approx(0.50F));
    CHECK(timeline.clips.front().end == Approx(0.90F));

    // Destinations outside the normalized domain are refused unchanged.
    CHECK_FALSE(TransformWaterFeatureTimelineSpan(
        &timeline,
        0.50F,
        0.90F,
        0.70F,
        1.10F));
    CHECK(timeline.settings[0].keys[2].position == Approx(0.90F));

    // A remapped key may not land on a key outside the moved span.
    timeline.settings[0].keys.push_back({.position = 0.10F, .value = 0.5F});
    CHECK_FALSE(TransformWaterFeatureTimelineSpan(
        &timeline,
        0.50F,
        0.90F,
        0.10F,
        0.50F));

    // Other keys and clip windows do not clamp the drag range; only an exact
    // same-track landing collision rejects a particular transform.
    const auto limits = WaterFeatureTimelineSpanLimits(
        timeline,
        0.50F,
        0.90F);
    CHECK(limits.minimumStart == Approx(0.0F));
    CHECK(limits.maximumEnd == Approx(1.0F));
}

TEST_CASE("Settings clips capture, apply, and duplicate as packages",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::ApplyWaterKeyedSettingsClip;
    using invisible_places::water::CaptureWaterKeyedSettingsClip;
    using invisible_places::water::DuplicateWaterFeatureClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.20F,
              .value = 0.0F,
              .interpolation = WaterScenarioInterpolation::Hold},
             {.position = 0.30F, .value = 1.0F},
             {.position = 0.40F, .value = 0.0F},
         }},
        {.settingId = "prominence",
         .active = false,
         .keys = {{.position = 0.30F, .value = 1.5F}}},
    };

    const auto package = CaptureWaterKeyedSettingsClip(
        timeline,
        0.20F,
        0.40F);
    CHECK(package.featureKind == WaterKeyedFeatureKind::SeepageNode);
    CHECK(package.nativeLengthFraction == Approx(0.20F));
    REQUIRE(package.settings.size() == 2U);
    CHECK(package.settings[0].keys[0].position == Approx(0.0F));
    CHECK(package.settings[0].keys[0].interpolation ==
          WaterScenarioInterpolation::Hold);
    CHECK(package.settings[0].keys[1].position == Approx(0.5F));
    CHECK(package.settings[0].keys[2].position == Approx(1.0F));
    CHECK_FALSE(package.settings[1].active);

    // Applying into a window on a fresh feature creates the tracks, maps
    // the keys across the window, and adds a provenance clip.
    WaterFeatureTimeline target;
    target.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 9U};
    target.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.05F, .value = 0.7F},
             {.position = 0.65F, .value = 0.3F},
         }},
    };
    auto applied = ApplyWaterKeyedSettingsClip(
        &target,
        package,
        0.50F,
        0.60F,
        "First Burst");
    REQUIRE(applied.has_value());
    REQUIRE(target.clips.size() == 1U);
    CHECK(target.clips.front().id == applied.value());
    CHECK(target.clips.front().name == "First Burst");
    CHECK(target.clips.front().start == Approx(0.50F));
    CHECK(target.clips.front().end == Approx(0.60F));
    REQUIRE(target.settings.size() == 2U);
    // Keys outside the window survive; the window key at 0.55 replaced the
    // package midpoint position.
    REQUIRE(target.settings[0].keys.size() == 5U);
    CHECK(target.settings[0].keys[0].position == Approx(0.05F));
    CHECK(target.settings[0].keys[1].position == Approx(0.50F));
    CHECK(target.settings[0].keys[2].position == Approx(0.55F));
    CHECK(target.settings[0].keys[3].position == Approx(0.60F));
    CHECK(target.settings[0].keys[4].position == Approx(0.65F));
    CHECK_FALSE(target.settings[1].active);

    // Duplicating reproduces bounds and values at the new start.
    const auto duplicate = DuplicateWaterFeatureClip(
        &target,
        applied.value(),
        0.80F);
    REQUIRE(duplicate.has_value());
    REQUIRE(target.clips.size() == 2U);
    CHECK(target.clips.back().name == "First Burst");
    CHECK(target.clips.back().start == Approx(0.80F));
    CHECK(target.clips.back().end == Approx(0.90F));
    const auto& strengthKeys = target.settings[0].keys;
    REQUIRE(strengthKeys.size() == 8U);
    CHECK(strengthKeys[5].position == Approx(0.80F));
    CHECK(strengthKeys[6].position == Approx(0.85F));
    CHECK(strengthKeys[7].position == Approx(0.90F));
    CHECK(strengthKeys[6].value == Approx(1.0F));
}

TEST_CASE("Package application rolls back when a dense window cannot fit every key",
          "[water][timing][keyed][clips][packages][collision]") {
    using invisible_places::water::ApplyWaterKeyedSettingsClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {{
        .settingId = "strength",
        .active = false,
        .label = "Node Strength",
        .defaultInterpolation = WaterScenarioInterpolation::Smooth,
    }};
    for (int index = 0; index <= 20; ++index) {
        timeline.settings.front().keys.push_back({
            .position = 0.5F + static_cast<float>(index) * 0.00005F,
            .value = static_cast<float>(index) * 0.1F,
            .interpolation = WaterScenarioInterpolation::Hold,
            .clipId = 0U,
            .incomingHandleTime = 0.2F,
            .incomingHandleValue = -0.1F,
            .outgoingHandleTime = 0.4F,
            .outgoingHandleValue = 0.3F,
        });
    }
    timeline.clips = {{
        .id = 4U,
        .name = "Existing marker",
        .start = 0.1F,
        .end = 0.2F,
        .sourceProfileName = "Earlier",
    }};
    timeline.clipMembershipExplicit = false;
    const auto before = timeline;

    WaterKeyedSettingsProfile package{
        .name = "Dense Start",
        .baseProfileName = "Dense Start",
        .featureKind = WaterKeyedFeatureKind::SeepageNode,
        .nativeLengthFraction = 0.001F,
        .settings = {
            {
                .settingId = "prominence",
                .keys = {{.position = 0.0F, .value = 1.25F}},
            },
            {
                .settingId = "strength",
                .keys = {{.position = 0.5F, .value = 2.0F}},
            },
        },
    };

    CHECK_FALSE(ApplyWaterKeyedSettingsClip(
                    &timeline,
                    package,
                    0.5F,
                    0.501F,
                    "Should Not Appear")
                    .has_value());

    CHECK(timeline.feature.kind == before.feature.kind);
    CHECK(timeline.feature.objectId == before.feature.objectId);
    CHECK(timeline.clipMembershipExplicit == before.clipMembershipExplicit);
    REQUIRE(timeline.clips.size() == before.clips.size());
    for (std::size_t index = 0U; index < timeline.clips.size(); ++index) {
        const auto& actual = timeline.clips[index];
        const auto& expected = before.clips[index];
        CHECK(actual.id == expected.id);
        CHECK(actual.name == expected.name);
        CHECK(actual.start == expected.start);
        CHECK(actual.end == expected.end);
        CHECK(actual.sourceProfileName == expected.sourceProfileName);
    }
    REQUIRE(timeline.settings.size() == before.settings.size());
    for (std::size_t settingIndex = 0U;
         settingIndex < timeline.settings.size();
         ++settingIndex) {
        const auto& actual = timeline.settings[settingIndex];
        const auto& expected = before.settings[settingIndex];
        CHECK(actual.settingId == expected.settingId);
        CHECK(actual.active == expected.active);
        CHECK(actual.label == expected.label);
        CHECK(actual.profileGroup == expected.profileGroup);
        CHECK(actual.profileName == expected.profileName);
        CHECK(actual.defaultInterpolation == expected.defaultInterpolation);
        REQUIRE(actual.keys.size() == expected.keys.size());
        for (std::size_t keyIndex = 0U;
             keyIndex < actual.keys.size();
             ++keyIndex) {
            const auto& actualKey = actual.keys[keyIndex];
            const auto& expectedKey = expected.keys[keyIndex];
            CHECK(actualKey.position == expectedKey.position);
            CHECK(actualKey.value == expectedKey.value);
            CHECK(actualKey.interpolation == expectedKey.interpolation);
            CHECK(actualKey.clipId == expectedKey.clipId);
            CHECK(actualKey.incomingHandleTime ==
                  expectedKey.incomingHandleTime);
            CHECK(actualKey.incomingHandleValue ==
                  expectedKey.incomingHandleValue);
            CHECK(actualKey.outgoingHandleTime ==
                  expectedKey.outgoingHandleTime);
            CHECK(actualKey.outgoingHandleValue ==
                  expectedKey.outgoingHandleValue);
        }
    }
}

TEST_CASE("Applied packages preserve their track-default interpolation when stretched",
          "[water][timing][keyed][clips][packages][interpolation]") {
    using Catch::Approx;
    using invisible_places::water::ApplyWaterKeyedSettingsClip;
    using invisible_places::water::EvaluateWaterKeyedSettingTrack;
    using invisible_places::water::TransformWaterFeatureClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterKeyedSettingsProfile package{
        .name = "Monotone Start",
        .featureKind = WaterKeyedFeatureKind::SeepageNode,
        .nativeLengthFraction = 0.4F,
        .settings = {{
            .settingId = "strength",
            .defaultInterpolation =
                WaterScenarioInterpolation::SmoothVelocity,
            .keys = {
                {.position = 0.0F, .value = 0.0F},
                {.position = 0.2F, .value = 0.05F},
                {.position = 0.6F, .value = 0.35F},
                {.position = 1.0F, .value = 1.0F},
            },
        }},
    };
    WaterFeatureTimeline target;
    target.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 9U};
    target.settings = {{
        .settingId = "strength",
        .defaultInterpolation = WaterScenarioInterpolation::Smooth,
    }};

    const auto applied = ApplyWaterKeyedSettingsClip(
        &target,
        package,
        0.2F,
        0.6F,
        "Start");
    REQUIRE(applied.has_value());
    REQUIRE(target.settings.front().keys.size() == 4U);
    for (const auto& key : target.settings.front().keys) {
        CHECK(key.interpolation ==
              WaterScenarioInterpolation::SmoothVelocity);
    }
    for (int sample = 0; sample <= 100; ++sample) {
        const float amount = static_cast<float>(sample) / 100.0F;
        const auto expected = EvaluateWaterKeyedSettingTrack(
            package.settings.front(),
            amount);
        const auto actual = EvaluateWaterKeyedSettingTrack(
            target.settings.front(),
            0.2F + amount * 0.4F);
        REQUIRE(expected.has_value());
        REQUIRE(actual.has_value());
        CHECK(actual.value() == Approx(expected.value()).margin(1.0e-5F));
    }

    REQUIRE(TransformWaterFeatureClip(
        &target,
        applied.value(),
        0.1F,
        0.9F));
    for (int sample = 0; sample <= 100; ++sample) {
        const float amount = static_cast<float>(sample) / 100.0F;
        const auto expected = EvaluateWaterKeyedSettingTrack(
            package.settings.front(),
            amount);
        const auto actual = EvaluateWaterKeyedSettingTrack(
            target.settings.front(),
            0.1F + amount * 0.8F);
        REQUIRE(expected.has_value());
        REQUIRE(actual.has_value());
        CHECK(actual.value() == Approx(expected.value()).margin(1.0e-5F));
    }
}

TEST_CASE("Settings clips transfer between features of one kind only",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::RemoveWaterFeatureClip;
    using invisible_places::water::TransferWaterFeatureClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline source;
    source.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    source.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.20F, .value = 0.0F},
             {.position = 0.30F, .value = 1.0F},
             {.position = 0.40F, .value = 0.0F},
             {.position = 0.70F, .value = 0.5F},
         }},
    };
    source.clips = {
        {.id = 3U, .name = "Response", .start = 0.20F, .end = 0.40F}};

    WaterFeatureTimeline sibling;
    sibling.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 9U};

    WaterFeatureTimeline rain;
    rain.feature = {.kind = WaterKeyedFeatureKind::Rain};

    // Cross-kind transfers are refused.
    CHECK_FALSE(TransferWaterFeatureClip(&source, 3U, &rain, false)
                    .has_value());

    // A copy leaves the source untouched.
    const auto copied = TransferWaterFeatureClip(
        &source,
        3U,
        &sibling,
        /*removeFromSource=*/false);
    REQUIRE(copied.has_value());
    REQUIRE(sibling.clips.size() == 1U);
    CHECK(sibling.clips.front().start == Approx(0.20F));
    CHECK(sibling.clips.front().end == Approx(0.40F));
    REQUIRE(sibling.settings.size() == 1U);
    REQUIRE(sibling.settings.front().keys.size() == 3U);
    CHECK(source.settings.front().keys.size() == 4U);
    REQUIRE(source.clips.size() == 1U);

    // A move removes the clip and its keys from the source; the earlier copy
    // remains as an independent overlapping clip on the destination.
    const auto moved = TransferWaterFeatureClip(
        &source,
        3U,
        &sibling,
        /*removeFromSource=*/true);
    REQUIRE(moved.has_value());
    CHECK(source.clips.empty());
    REQUIRE(source.settings.front().keys.size() == 1U);
    CHECK(source.settings.front().keys.front().position == Approx(0.70F));
    REQUIRE(sibling.clips.size() == 2U);
    REQUIRE(sibling.settings.front().keys.size() == 6U);

    // Removing one overlapping clip without deleting keys keeps only its
    // three members loose; the first copied clip remains intact.
    REQUIRE(RemoveWaterFeatureClip(&sibling, moved.value(), false));
    REQUIRE(sibling.clips.size() == 1U);
    CHECK(sibling.clips.front().id == copied.value());
    CHECK(sibling.settings.front().keys.size() == 6U);
    CHECK(std::count_if(
              sibling.settings.front().keys.begin(),
              sibling.settings.front().keys.end(),
              [](const auto& key) { return key.clipId == 0U; }) == 3);

    // Loose keys regroup into a clip even across an existing clip's window.
    const auto regrouped =
        invisible_places::water::CreateWaterFeatureClipFromSpan(
            &sibling,
            0.20F,
            0.40F,
            "Regrouped");
    REQUIRE(regrouped.has_value());
    REQUIRE(RemoveWaterFeatureClip(&sibling, regrouped.value(), true));
    REQUIRE(sibling.clips.size() == 1U);
    REQUIRE(sibling.settings.front().keys.size() == 3U);
    REQUIRE(RemoveWaterFeatureClip(
        &sibling,
        copied.value(),
        /*removeKeys=*/true));
    CHECK(sibling.clips.empty());
    CHECK(sibling.settings.front().keys.empty());
}

TEST_CASE("Feature timing run sanitize repairs stored clips",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::SanitizeWaterFeatureTimingRun;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimingRun run;
    run.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageNode,
                    .objectId = 4U},
        .settings = {},
        .clips = {
            {.id = 2U, .name = "Late", .start = 0.60F, .end = 0.80F},
            // Reversed bounds are swapped; the duplicate id is reassigned.
            {.id = 2U, .name = "Early", .start = 0.30F, .end = 0.10F},
            // Zero ids are reassigned; degenerate spans get the minimum
            // length.
            {.id = 0U, .name = "", .start = 0.50F, .end = 0.50F},
        },
    });
    const auto sanitized = SanitizeWaterFeatureTimingRun(run);
    const auto& clips = sanitized.features.front().clips;
    REQUIRE(clips.size() == 3U);
    CHECK(clips[0].name == "Early");
    CHECK(clips[0].start == Approx(0.10F));
    CHECK(clips[0].end == Approx(0.30F));
    CHECK(clips[1].name == "Clip");
    CHECK(clips[1].end - clips[1].start ==
          Approx(1.0e-3F).margin(1.0e-5F));
    CHECK(clips[2].name == "Late");
    // The first clip in sorted order keeps the contested id; later
    // duplicates and zero ids are reassigned uniquely.
    CHECK(clips[0].id == 2U);
    CHECK(clips[1].id != 0U);
    CHECK(clips[2].id != 0U);
    CHECK(clips[0].id != clips[1].id);
    CHECK(clips[0].id != clips[2].id);
    CHECK(clips[1].id != clips[2].id);
}

TEST_CASE("Feature Run marks sanitize and edit with stable unique identities",
          "[water][timing][marks]") {
    using Catch::Approx;
    using invisible_places::water::AllocateWaterFeatureRunMarkId;
    using invisible_places::water::AllocateWaterFeatureRunMarkName;
    using invisible_places::water::FindWaterFeatureRunMark;
    using invisible_places::water::MoveWaterFeatureRunMark;
    using invisible_places::water::RemoveWaterFeatureRunMark;
    using invisible_places::water::RenameWaterFeatureRunMark;
    using invisible_places::water::SanitizeWaterFeatureTimingRun;
    using invisible_places::water::WaterFeatureTimingRun;

    WaterFeatureTimingRun run;
    run.marks = {
        {.id = 2U, .text = " Mark 00 ", .position = 1.4F},
        {.id = 2U, .text = "Mark 02", .position = -0.3F},
        {.id = 0U,
         .text = "   ",
         .position = std::numeric_limits<float>::quiet_NaN()},
    };
    run = SanitizeWaterFeatureTimingRun(std::move(run));
    REQUIRE(run.marks.size() == 3U);
    CHECK(run.marks[0].position == Approx(0.0F));
    CHECK(run.marks[1].position == Approx(0.0F));
    CHECK(run.marks[2].position == Approx(1.0F));
    CHECK(run.marks[0].id != 0U);
    CHECK(run.marks[1].id != 0U);
    CHECK(run.marks[2].id != 0U);
    CHECK(run.marks[0].id != run.marks[1].id);
    CHECK(run.marks[0].id != run.marks[2].id);
    CHECK(run.marks[1].id != run.marks[2].id);
    CHECK(run.marks[1].text == "Mark");

    std::vector<WaterFeatureTimingRun> runs{run, {}};
    runs[1].marks = {{.id = 1U, .text = "Mark 01", .position = 0.5F}};
    CHECK(AllocateWaterFeatureRunMarkName(runs) == "Mark 03");
    const auto nextId = AllocateWaterFeatureRunMarkId(runs[0]);
    REQUIRE(nextId != 0U);
    runs[0].marks.push_back({
        .id = nextId,
        .text = "Mark 03",
        .position = 0.25F,
    });
    REQUIRE(FindWaterFeatureRunMark(&runs[0], nextId) != nullptr);
    CHECK(MoveWaterFeatureRunMark(&runs[0], nextId, 0.75F));
    CHECK(FindWaterFeatureRunMark(&runs[0], nextId)->position ==
          Approx(0.75F));
    CHECK(RenameWaterFeatureRunMark(&runs[0], nextId, "  Peak flow  "));
    CHECK(FindWaterFeatureRunMark(&runs[0], nextId)->text == "Peak flow");
    CHECK_FALSE(RenameWaterFeatureRunMark(&runs[0], nextId, "   "));
    CHECK(RemoveWaterFeatureRunMark(&runs[0], nextId));
    CHECK(FindWaterFeatureRunMark(&runs[0], nextId) == nullptr);
    CHECK_FALSE(RemoveWaterFeatureRunMark(&runs[0], nextId));
}

TEST_CASE("Settings clips and package lengths round-trip through the project document",
          "[water][timing][keyed][clips][marks][serialization]") {
    using Catch::Approx;
    using invisible_places::serialization::LoadProjectDocument;
    using invisible_places::serialization::ProjectDocument;
    using invisible_places::serialization::SaveProjectDocument;
    using invisible_places::water::WaterKeyedFeatureKind;

    ProjectDocument document;
    document.projectName = "Clips";
    invisible_places::water::WaterScenarioFeatureRuns entry;
    entry.scenarioId = "authored-timing";
    invisible_places::water::WaterFeatureTimingRun run;
    run.id = 3U;
    run.name = "Bursts";
    run.marks = {
        {.id = 3U, .text = "Rain begins", .position = 0.18F},
        {.id = 7U, .text = "Pool full", .position = 0.72F},
    };
    run.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageNode,
                    .objectId = 4U},
        .settings = {{
            .settingId = "strength",
            .keys = {
                {.position = 0.20F, .value = 0.0F, .clipId = 5U},
                {.position = 0.40F, .value = 1.0F, .clipId = 5U},
                {.position = 0.70F, .value = 0.25F},
            },
        }},
        .clips = {
            {.id = 5U,
             .name = "First Burst",
             .start = 0.20F,
             .end = 0.40F,
             .sourceProfileName = "Seep On Off"},
        },
        .clipMembershipExplicit = true,
    });
    entry.runs.push_back(run);
    document.waterFeatureTimingRuns.push_back(entry);
    document.timingTakeStates.push_back({
        .takeId = "timing-take-marks",
        .sceneGroupName = "Scene3",
        .waterFeatureTimingRuns = {run},
    });
    document.waterKeyedSettingsProfiles.push_back({
        .name = "Seep On Off",
        .baseProfileName = "Seep On Off",
        .featureKind = WaterKeyedFeatureKind::SeepageNode,
        .nativeLengthFraction = 0.20F,
        .settings = {{
            .settingId = "strength",
            .keys = {
                {.position = 0.0F, .value = 0.0F},
                {.position = 1.0F, .value = 1.0F},
            },
        }},
    });

    TemporaryTimingFile file{"invisible_places_settings_clips.json"};
    std::string errorMessage;
    REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
    {
        std::ifstream input{file.path};
        const std::string saved{
            std::istreambuf_iterator<char>{input},
            std::istreambuf_iterator<char>{}};
        CHECK(saved.find("\"clip_id\": 5") != std::string::npos);
        CHECK(saved.find("\"clip_id\": 0") != std::string::npos);
    }
    const auto loaded = LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->waterFeatureTimingRuns.size() == 1U);
    const auto& loadedRun =
        loaded->waterFeatureTimingRuns.front().runs.front();
    REQUIRE(loadedRun.marks.size() == 2U);
    CHECK(loadedRun.marks[0].id == 3U);
    CHECK(loadedRun.marks[0].text == "Rain begins");
    CHECK(loadedRun.marks[0].position == Approx(0.18F));
    CHECK(loadedRun.marks[1].id == 7U);
    CHECK(loadedRun.marks[1].text == "Pool full");
    REQUIRE(loaded->timingTakeStates.size() == 1U);
    REQUIRE(loaded->timingTakeStates.front()
                .waterFeatureTimingRuns.front()
                .marks.size() == 2U);
    CHECK(loaded->timingTakeStates.front()
              .waterFeatureTimingRuns.front()
              .marks[1]
              .text == "Pool full");
    REQUIRE(loadedRun.features.size() == 1U);
    const auto& clips = loadedRun.features.front().clips;
    REQUIRE(clips.size() == 1U);
    CHECK(clips.front().id == 5U);
    CHECK(clips.front().name == "First Burst");
    CHECK(clips.front().start == Approx(0.20F));
    CHECK(clips.front().end == Approx(0.40F));
    CHECK(clips.front().sourceProfileName == "Seep On Off");
    REQUIRE(loadedRun.features.front().settings.front().keys.size() == 3U);
    CHECK(loadedRun.features.front().settings.front().keys[0].clipId == 5U);
    CHECK(loadedRun.features.front().settings.front().keys[1].clipId == 5U);
    CHECK(loadedRun.features.front().settings.front().keys[2].clipId == 0U);
    CHECK(loadedRun.features.front().clipMembershipExplicit);
    REQUIRE(loaded->waterKeyedSettingsProfiles.size() == 1U);
    CHECK(loaded->waterKeyedSettingsProfiles.front().nativeLengthFraction ==
          Approx(0.20F));
}

TEST_CASE("Timing Take retiming carries settings clips with their keys",
          "[water][timing][keyed][clips][marks][pan-extension]") {
    using Catch::Approx;
    using invisible_places::timing::RetimeTimingTakeSceneStateNormalizedPositions;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterKeyedFeatureKind;

    TimingTakeSceneState state;
    invisible_places::water::WaterFeatureTimingRun run;
    run.marks = {{.id = 4U, .text = "Event", .position = 0.40F}};
    run.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageNode,
                    .objectId = 4U},
        .settings = {{
            .settingId = "strength",
            .keys = {
                {.position = 0.25F, .value = 0.0F},
                {.position = 0.75F, .value = 1.0F},
            },
        }},
        .clips = {
            {.id = 1U, .name = "On", .start = 0.25F, .end = 0.75F}},
    });
    state.waterFeatureTimingRuns.push_back(std::move(run));

    // 100 source frames appended after 100 frames of new pre-roll inside a
    // 200-frame destination: every coordinate halves and shifts to 0.5+.
    REQUIRE(RetimeTimingTakeSceneStateNormalizedPositions(
        &state,
        100U,
        200U,
        100U));
    const auto& timeline = state.waterFeatureTimingRuns.front().features.front();
    REQUIRE(state.waterFeatureTimingRuns.front().marks.size() == 1U);
    CHECK(state.waterFeatureTimingRuns.front().marks.front().position ==
          Approx(0.70F));
    CHECK(timeline.settings.front().keys.front().position ==
          Approx(0.625F));
    CHECK(timeline.settings.front().keys.back().position ==
          Approx(0.875F));
    CHECK(timeline.clips.front().start == Approx(0.625F));
    CHECK(timeline.clips.front().end == Approx(0.875F));
}

TEST_CASE("Timing Take merge keeps Feature Run marks with feature owners",
          "[water][timing][marks][merge]") {
    using Catch::Approx;
    using invisible_places::timing::MergeTimingTakeSceneStateKeepingFirst;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;

    const WaterKeyedFeatureId feature{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 8U,
    };
    TimingTakeSceneState destination;
    destination.waterFeatureTimingRuns = {WaterFeatureTimingRun{
        .id = 1U,
        .name = "First Run",
        .features = {{.feature = feature}},
        .marks = {{.id = 1U, .text = "Shared", .position = 0.20F}},
    }};
    TimingTakeSceneState source;
    source.waterFeatureTimingRuns = {WaterFeatureTimingRun{
        .id = 1U,
        .name = "Other Run",
        .features = {{.feature = feature}},
        .marks = {
            {.id = 1U, .text = "Second event", .position = 0.60F},
            {.id = 2U, .text = "Shared", .position = 0.20F},
        },
    }};

    MergeTimingTakeSceneStateKeepingFirst(&destination, source);
    REQUIRE(destination.waterFeatureTimingRuns.size() == 2U);
    const auto owner = std::find_if(
        destination.waterFeatureTimingRuns.begin(),
        destination.waterFeatureTimingRuns.end(),
        [&](const WaterFeatureTimingRun& run) {
            return std::any_of(
                run.features.begin(),
                run.features.end(),
                [&](const auto& timeline) {
                    return timeline.feature == feature;
                });
        });
    REQUIRE(owner != destination.waterFeatureTimingRuns.end());
    REQUIRE(owner->marks.size() == 2U);
    CHECK(owner->marks[0].text == "Shared");
    CHECK(owner->marks[0].position == Approx(0.20F));
    CHECK(owner->marks[1].text == "Second event");
    CHECK(owner->marks[1].position == Approx(0.60F));
    CHECK(owner->marks[0].id != owner->marks[1].id);
    const auto emptyRun = std::find_if(
        destination.waterFeatureTimingRuns.begin(),
        destination.waterFeatureTimingRuns.end(),
        [](const WaterFeatureTimingRun& run) {
            return run.name == "Other Run";
        });
    REQUIRE(emptyRun != destination.waterFeatureTimingRuns.end());
    CHECK(emptyRun->features.empty());
    CHECK(emptyRun->marks.empty());
}

TEST_CASE("Timing Take merge removes marks from repaired empty duplicate runs",
          "[water][timing][marks][merge][migration]") {
    using invisible_places::timing::MergeTimingTakeSceneStateKeepingFirst;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureId;
    using invisible_places::water::WaterKeyedFeatureKind;

    const WaterKeyedFeatureId feature{
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 12U,
    };
    TimingTakeSceneState destination;
    destination.waterFeatureTimingRuns = {
        WaterFeatureTimingRun{
            .id = 1U,
            .name = "Owner",
            .features = {{.feature = feature}},
        },
        WaterFeatureTimingRun{
            .id = 2U,
            .name = "Duplicate",
            .features = {{.feature = feature}},
            .marks = {{.id = 1U, .text = "Repair me", .position = 0.4F}},
        },
    };

    MergeTimingTakeSceneStateKeepingFirst(
        &destination,
        TimingTakeSceneState{});
    REQUIRE(destination.waterFeatureTimingRuns.size() == 2U);
    CHECK(destination.waterFeatureTimingRuns[0].features.size() == 1U);
    REQUIRE(destination.waterFeatureTimingRuns[0].marks.size() == 1U);
    CHECK(destination.waterFeatureTimingRuns[0].marks[0].text ==
          "Repair me");
    CHECK(destination.waterFeatureTimingRuns[1].features.empty());
    CHECK(destination.waterFeatureTimingRuns[1].marks.empty());
}

TEST_CASE("Adjacent clips keep their boundary keys through span operations",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::RemoveWaterFeatureClip;
    using invisible_places::water::TransformWaterFeatureClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.20F, .value = 0.0F, .clipId = 1U},
             {.position = 0.35F, .value = 1.0F, .clipId = 1U},
             {.position = 0.50F, .value = 0.4F, .clipId = 2U},
             {.position = 0.65F, .value = 1.0F, .clipId = 2U},
             {.position = 0.80F, .value = 0.0F, .clipId = 2U},
         }},
    };
    timeline.clips = {
        {.id = 1U, .name = "First", .start = 0.20F, .end = 0.35F},
        {.id = 2U, .name = "Second", .start = 0.50F, .end = 0.80F},
    };
    timeline.clipMembershipExplicit = true;

    // Moving the first clip leaves the second clip's explicitly owned
    // opening key behind instead of capturing it by destination span.
    REQUIRE(TransformWaterFeatureClip(
        &timeline,
        1U,
        0.10F,
        0.40F));
    REQUIRE(timeline.settings.front().keys.size() == 5U);
    CHECK(timeline.settings.front().keys[0].position == Approx(0.10F));
    CHECK(timeline.settings.front().keys[1].position == Approx(0.40F));
    CHECK(timeline.settings.front().keys[2].position == Approx(0.50F));
    CHECK(timeline.settings.front().keys[2].value == Approx(0.4F));
    CHECK(timeline.settings.front().keys[3].position == Approx(0.65F));
    CHECK(timeline.clips[0].start == Approx(0.10F));
    CHECK(timeline.clips[0].end == Approx(0.40F));
    CHECK(timeline.clips[1].start == Approx(0.50F));

    // Deleting it affects only its explicit members; the second clip's
    // opening key survives.
    REQUIRE(RemoveWaterFeatureClip(&timeline, 1U, /*removeKeys=*/true));
    REQUIRE(timeline.settings.front().keys.size() == 3U);
    CHECK(timeline.settings.front().keys[0].position == Approx(0.50F));
    CHECK(timeline.settings.front().keys[0].value == Approx(0.4F));
}

TEST_CASE("Applying a package beside a clip preserves the neighbour boundary key",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::ApplyWaterKeyedSettingsClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingsProfile;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.20F, .value = 0.3F},
             {.position = 0.50F, .value = 0.9F},
         }},
    };
    timeline.clips = {
        {.id = 1U, .name = "First", .start = 0.20F, .end = 0.50F}};

    WaterKeyedSettingsProfile package;
    package.featureKind = WaterKeyedFeatureKind::SeepageNode;
    package.nativeLengthFraction = 0.30F;
    package.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.0F, .value = 0.0F},
             {.position = 1.0F, .value = 1.0F},
         }},
    };
    const auto applied = ApplyWaterKeyedSettingsClip(
        &timeline,
        package,
        0.50F,
        0.80F,
        "Second");
    REQUIRE(applied.has_value());
    // The touching clip survives with its closing key and value intact;
    // the package's opening key lands nudged just inside the new window.
    REQUIRE(timeline.clips.size() == 2U);
    const auto& keys = timeline.settings.front().keys;
    REQUIRE(keys.size() == 4U);
    CHECK(keys[1].position == Approx(0.50F));
    CHECK(keys[1].value == Approx(0.9F));
    CHECK(keys[2].position > 0.50F);
    CHECK(keys[2].position <= Approx(0.5010F));
    CHECK(keys[2].value == Approx(0.0F));
    CHECK(keys[3].position == Approx(0.80F));
    CHECK(keys[3].value == Approx(1.0F));
}

TEST_CASE("Duplicating a clip supports overlapping destinations",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::DuplicateWaterFeatureClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.70F, .value = 0.0F},
             {.position = 0.82F, .value = 1.0F},
             {.position = 0.95F, .value = 0.0F},
         }},
    };
    timeline.clips = {
        {.id = 1U, .name = "Burst", .start = 0.70F, .end = 0.95F}};

    // The clamped destination overlaps the source, but remains an
    // independent clip whose same-track keys are minimally separated.
    const auto overlapping =
        DuplicateWaterFeatureClip(&timeline, 1U, 0.951F);
    REQUIRE(overlapping.has_value());
    CHECK(timeline.clips.size() == 2U);
    CHECK(timeline.settings.front().keys.size() == 6U);
    CHECK(std::count_if(
              timeline.settings.front().keys.begin(),
              timeline.settings.front().keys.end(),
              [&](const auto& key) {
                  return key.clipId == overlapping.value();
              }) == 3);

    // A clear window in front still works independently.
    const auto duplicated =
        DuplicateWaterFeatureClip(&timeline, 1U, 0.10F);
    REQUIRE(duplicated.has_value());
    CHECK(timeline.clips.size() == 3U);
    CHECK(timeline.settings.front().keys.size() == 9U);
}

TEST_CASE("Clip moves and sanitize preserve overlapping entries",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::SanitizeWaterFeatureTimingRun;
    using invisible_places::water::TransformWaterFeatureClip;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimelineSpanLimits;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.10F, .value = 0.0F, .clipId = 1U},
             {.position = 0.20F, .value = 1.0F, .clipId = 1U},
             {.position = 0.50F, .value = 0.3F, .clipId = 2U},
             {.position = 0.60F, .value = 0.7F, .clipId = 2U},
         }},
    };
    timeline.clips = {
        {.id = 1U, .name = "Keyed", .start = 0.10F, .end = 0.20F},
        {.id = 2U, .name = "Marker", .start = 0.50F, .end = 0.60F},
    };
    timeline.clipMembershipExplicit = true;

    const auto limits = WaterFeatureTimelineSpanLimits(
        timeline,
        0.10F,
        0.20F);
    CHECK(limits.minimumStart == Approx(0.0F));
    CHECK(limits.maximumEnd == Approx(1.0F));
    REQUIRE(TransformWaterFeatureClip(
        &timeline,
        1U,
        0.45F,
        0.55F));
    const auto* moved = invisible_places::water::FindWaterFeatureClip(
        &timeline,
        1U);
    REQUIRE(moved != nullptr);
    CHECK(moved->start == Approx(0.45F));
    CHECK(moved->end == Approx(0.55F));
    CHECK(timeline.settings.front().keys[1].clipId == 2U);
    CHECK(timeline.settings.front().keys[2].clipId == 1U);

    // Overlapping and fully nested clips from hand-edited documents survive
    // sanitization; only bounds and ids are repaired.
    WaterFeatureTimingRun run;
    run.features.push_back({
        .feature = {.kind = WaterKeyedFeatureKind::SeepageNode,
                    .objectId = 4U},
        .settings = {},
        .clips = {
            {.id = 1U, .name = "A", .start = 0.10F, .end = 0.50F},
            {.id = 2U, .name = "B", .start = 0.30F, .end = 0.70F},
            {.id = 3U, .name = "C", .start = 0.32F, .end = 0.40F},
        },
    });
    const auto sanitized = SanitizeWaterFeatureTimingRun(run);
    const auto& clips = sanitized.features.front().clips;
    REQUIRE(clips.size() == 3U);
    CHECK(clips[0].name == "A");
    CHECK(clips[0].end == Approx(0.50F));
    CHECK(clips[1].name == "B");
    CHECK(clips[1].start == Approx(0.30F));
    CHECK(clips[1].end == Approx(0.70F));
    CHECK(clips[2].name == "C");
    CHECK(clips[2].start == Approx(0.32F));
    CHECK(clips[2].end == Approx(0.40F));
}

TEST_CASE("Explicit clip membership routes new keys and derives clip bounds",
          "[water][timing][keyed][clips]") {
    using Catch::Approx;
    using invisible_places::water::AddOrUpdateWaterSettingKey;
    using invisible_places::water::AddOrUpdateWaterTimelineSettingKey;
    using invisible_places::water::CaptureWaterKeyedSettingsClipById;
    using invisible_places::water::SynchronizeWaterFeatureClipBounds;
    using invisible_places::water::WaterFeatureLooseKeySpan;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 7U};
    timeline.settings = {
        {.settingId = "strength",
         .keys = {
             {.position = 0.20F, .value = 0.0F, .clipId = 7U},
             {.position = 0.40F, .value = 1.0F, .clipId = 7U},
             {.position = 0.70F, .value = 0.4F},
         }},
    };
    timeline.clips = {
        {.id = 7U, .name = "Selected", .start = 0.20F, .end = 0.40F},
    };
    timeline.clipMembershipExplicit = true;

    // This is the model call used when the UI has one stored clip selected.
    AddOrUpdateWaterSettingKey(
        &timeline.settings.front(),
        0.60F,
        0.5F,
        WaterScenarioInterpolation::Linear,
        7U);
    REQUIRE(SynchronizeWaterFeatureClipBounds(&timeline, 7U));
    const auto* clip = invisible_places::water::FindWaterFeatureClip(
        &timeline,
        7U);
    REQUIRE(clip != nullptr);
    CHECK(clip->start == Approx(0.20F));
    CHECK(clip->end == Approx(0.60F));

    // With no selected clip id, a genuinely new key stays loose even though
    // its time can overlap the selected clip's window.
    AddOrUpdateWaterSettingKey(
        &timeline.settings.front(),
        0.30F,
        0.25F);
    const auto looseSpan = WaterFeatureLooseKeySpan(timeline);
    REQUIRE(looseSpan.has_value());
    CHECK(looseSpan->first == Approx(0.30F));
    CHECK(looseSpan->second == Approx(0.70F));

    const auto package = CaptureWaterKeyedSettingsClipById(timeline, 7U);
    REQUIRE(package.settings.size() == 1U);
    REQUIRE(package.settings.front().keys.size() == 3U);
    CHECK(package.settings.front().keys.front().position == Approx(0.0F));
    CHECK(package.settings.front().keys.back().position == Approx(1.0F));
    CHECK(std::all_of(
        package.settings.front().keys.begin(),
        package.settings.front().keys.end(),
        [](const auto& key) { return key.clipId == 0U; }));

    timeline.clips.push_back(
        {.id = 8U, .name = "Other", .start = 0.80F, .end = 0.90F});
    AddOrUpdateWaterTimelineSettingKey(
        &timeline,
        &timeline.settings.front(),
        0.20F,
        0.9F,
        WaterScenarioInterpolation::SmoothVelocity,
        8U);
    const auto edited = std::find_if(
        timeline.settings.front().keys.begin(),
        timeline.settings.front().keys.end(),
        [](const auto& key) {
            return std::abs(key.position - 0.20F) <= 1.0e-4F;
        });
    REQUIRE(edited != timeline.settings.front().keys.end());
    CHECK(edited->value == Approx(0.9F));
    CHECK(edited->clipId == 7U);
}

TEST_CASE(
    "legacy Rain snapshot becomes one shared Timing Take profile without losing edited visual values",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::EnsureLegacyWaterRainProfile;
    using invisible_places::timing::ResolveTimingTakeRainProfile;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::RainRuntimeSettings;
    using invisible_places::water::WaterRainProfile;
    using invisible_places::water::WaterRainVisualSettings;

    RainRuntimeSettings legacy;
    legacy.enabled = false;
    legacy.activeParticleCount = 32'768U;
    legacy.visualProfileName = "Rain Fine Lines";
    legacy.density = 0.40F;
    legacy.opacityScale = 1.58F;
    WaterRainVisualSettings editedVisual;
    editedVisual.widthMeters = 0.0017F;
    editedVisual.softness = 0.92F;
    editedVisual.opacity = 0.08F;
    editedVisual.emission = 0.01F;
    editedVisual.minimumScreenPixels = 0.0F;
    editedVisual.maximumScreenPixels = 10.09F;

    std::vector<WaterRainProfile> profiles;
    std::vector<TimingTakeDefinition> takes{
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "timing-take-8", .name = "First-01"},
        {.id = "timing-take-9", .name = "First-02"},
    };
    const auto baseId = EnsureLegacyWaterRainProfile(
        &profiles,
        &takes,
        legacy,
        editedVisual);

    REQUIRE(baseId == invisible_places::timing::kLegacyWaterRainProfileId);
    REQUIRE(profiles.size() == 1U);
    CHECK(profiles.front().name == "Project Rain");
    CHECK_FALSE(profiles.front().objectOverride);
    CHECK(profiles.front().settings == legacy);
    CHECK(profiles.front().visual == editedVisual);
    for (const auto& take : takes) {
        CHECK(take.assignedRainProfileId == baseId);
        CHECK(take.assignedRainProfileName == "Project Rain");
        CHECK(take.baseRainProfileId == baseId);
        REQUIRE(ResolveTimingTakeRainProfile(profiles, take) != nullptr);
        CHECK(ResolveTimingTakeRainProfile(profiles, take)->visual ==
              editedVisual);
    }

    // Re-running migration does not manufacture one profile per take and does
    // not overwrite a base that has subsequently been authored.
    profiles.front().settings.density = 0.73F;
    CHECK(EnsureLegacyWaterRainProfile(
              &profiles,
              &takes,
              legacy,
              editedVisual) == baseId);
    REQUIRE(profiles.size() == 1U);
    CHECK(profiles.front().settings.density == Approx(0.73F));
}

TEST_CASE(
    "Rain authored value resolver covers the complete keyable registry",
    "[water][rain][profiles][timing]") {
    using invisible_places::water::ResolveWaterRainSettingValue;
    using invisible_places::water::WaterKeyableSettings;
    using invisible_places::water::WaterKeyedFeatureKind;

    auto settings = invisible_places::water::DefaultWaterRainSettings();
    auto visual = invisible_places::water::RainVisualPreset(
        "Rain Fine Lines");
    settings.rainLevel = 0.37F;
    settings.rockImpact.downhillStretch = 1.41F;
    visual.colour[1] = 0.63F;

    const auto registry = WaterKeyableSettings(
        WaterKeyedFeatureKind::Rain);
    REQUIRE_FALSE(registry.empty());
    for (const auto& info : registry) {
        CAPTURE(info.id);
        CHECK(ResolveWaterRainSettingValue(
                  settings,
                  visual,
                  info.id)
                  .has_value());
    }
    CHECK(ResolveWaterRainSettingValue(
              settings,
              visual,
              "level") == Catch::Approx(0.37F));
    CHECK(ResolveWaterRainSettingValue(
              settings,
              visual,
              "wetness.downhill_stretch") ==
          Catch::Approx(1.41F));
    CHECK(ResolveWaterRainSettingValue(
              settings,
              visual,
              "visual.colour_green") == Catch::Approx(0.63F));
    CHECK_FALSE(ResolveWaterRainSettingValue(
                    settings,
                    visual,
                    "unknown")
                    .has_value());
}

TEST_CASE(
    "standalone Rain import merges stable ids without creating takes or conflating names",
    "[water][rain][profiles][timing][import]") {
    using invisible_places::timing::MergeImportedTimingTakeRainProfiles;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile localSameName;
    localSameName.id = "local-shared";
    localSameName.name = "Shared Rain";
    localSameName.settings.density = 0.10F;
    WaterRainProfile updated;
    updated.id = "updated-by-id";
    updated.name = "Updated Rain";
    updated.settings.density = 0.20F;
    WaterRainProfile unrelated;
    unrelated.id = "unrelated";
    unrelated.name = "Unrelated Rain";
    unrelated.settings.density = 0.30F;
    WaterRainProfile dependentOwner = updated;
    dependentOwner.id = "dependent-owner";
    dependentOwner.name = "Updated Rain_Mirror";
    dependentOwner.objectOverride = true;
    dependentOwner.ownerTimingTakeId = "mirror";
    dependentOwner.baseProfileId = updated.id;
    dependentOwner.baseProfileName = updated.name;
    std::vector<WaterRainProfile> profiles{
        localSameName,
        updated,
        unrelated,
        dependentOwner,
    };
    std::vector<TimingTakeDefinition> takes{
        {.id = "take-a",
         .name = "Take A",
         .assignedRainProfileId = localSameName.id,
         .assignedRainProfileName = localSameName.name,
         .baseRainProfileId = localSameName.id,
         .baseRainProfileName = localSameName.name},
        {.id = "observer", .name = "Observer"},
        {.id = "mirror",
         .name = "Mirror",
         .assignedRainProfileId = updated.id,
         .assignedRainProfileName = updated.name,
         .baseRainProfileId = updated.id,
         .baseRainProfileName = updated.name},
        {.id = "untouched",
         .name = "Untouched",
         .assignedRainProfileId = unrelated.id,
         .assignedRainProfileName = unrelated.name,
         .baseRainProfileId = unrelated.id,
         .baseRainProfileName = unrelated.name},
    };
    const auto untouchedTake = takes.back();

    WaterRainProfile importedBase;
    importedBase.id = " imported-shared ";
    importedBase.name = " Shared Rain ";
    importedBase.settings.density = 0.80F;
    WaterRainProfile importedOwner = importedBase;
    importedOwner.id = "imported-owner";
    importedOwner.name = "Shared Rain_Take A";
    importedOwner.objectOverride = true;
    importedOwner.ownerTimingTakeId = "take-a";
    importedOwner.baseProfileId = "imported-shared";
    importedOwner.baseProfileName = "Shared Rain";
    importedOwner.settings.density = 0.91F;
    WaterRainProfile observedOwner = importedOwner;
    observedOwner.id = "observed-owner";
    observedOwner.name = "Shared Rain_Remote";
    observedOwner.ownerTimingTakeId = "remote-owner";
    observedOwner.settings.density = 0.67F;
    WaterRainProfile orphanOwner = observedOwner;
    orphanOwner.id = "orphan-owner";
    orphanOwner.name = "Shared Rain_Orphan";
    orphanOwner.ownerTimingTakeId = "missing-owner";
    WaterRainProfile importedUpdate = updated;
    importedUpdate.name = "Updated Rain Renamed";
    importedUpdate.settings.density = 0.74F;

    const std::vector<WaterRainProfile> importedProfiles{
        importedBase,
        importedOwner,
        observedOwner,
        orphanOwner,
        importedUpdate,
    };
    const std::vector<TimingTakeDefinition> assignments{
        {.id = "take-a",
         .assignedRainProfileId = " imported-owner ",
         .assignedRainProfileName = importedOwner.name,
         .baseRainProfileId = " imported-shared ",
         .baseRainProfileName = importedBase.name},
        {.id = "observer",
         .assignedRainProfileId = observedOwner.id,
         .assignedRainProfileName = observedOwner.name,
         .baseRainProfileId = "imported-shared",
         .baseRainProfileName = importedBase.name},
        {.id = "missing-take",
         .assignedRainProfileId = importedBase.id,
         .assignedRainProfileName = importedBase.name,
         .baseRainProfileId = importedBase.id,
         .baseRainProfileName = importedBase.name},
    };

    const auto result = MergeImportedTimingTakeRainProfiles(
        &profiles,
        &takes,
        importedProfiles,
        assignments);
    CHECK(result.profilesInserted == 3U);
    CHECK(result.profilesUpdated == 2U);
    CHECK(result.orphanOwnerProfilesSkipped == 1U);
    CHECK(result.assignmentsApplied == 3U);
    CHECK(result.changed());
    REQUIRE(takes.size() == 4U);
    CHECK(invisible_places::timing::FindTimingTakeDefinition(
              takes,
              "missing-take") == nullptr);

    const auto* preservedLocal =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            localSameName.id);
    REQUIRE(preservedLocal != nullptr);
    CHECK(*preservedLocal == localSameName);
    const auto* mergedBase =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            "imported-shared");
    REQUIRE(mergedBase != nullptr);
    CHECK(mergedBase->name == "Shared Rain 2");
    CHECK(mergedBase->settings.density == Catch::Approx(0.80F));
    const auto* mergedOwner =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            importedOwner.id);
    REQUIRE(mergedOwner != nullptr);
    CHECK(mergedOwner->baseProfileId == mergedBase->id);
    CHECK(mergedOwner->baseProfileName == mergedBase->name);
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              orphanOwner.id) == nullptr);
    REQUIRE(invisible_places::water::FindWaterRainProfileById(
                profiles,
                observedOwner.id) != nullptr);
    REQUIRE(invisible_places::water::FindWaterRainProfileById(
                profiles,
                updated.id) != nullptr);
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              updated.id)
              ->settings.density == Catch::Approx(0.74F));
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              updated.id)
              ->name == importedUpdate.name);
    const auto* repairedDependent =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            dependentOwner.id);
    REQUIRE(repairedDependent != nullptr);
    CHECK(repairedDependent->baseProfileName == importedUpdate.name);

    const auto* assignedTake =
        invisible_places::timing::FindTimingTakeDefinition(
            takes,
            "take-a");
    REQUIRE(assignedTake != nullptr);
    CHECK(assignedTake->name == "Take A");
    CHECK(assignedTake->assignedRainProfileId == mergedOwner->id);
    CHECK(assignedTake->assignedRainProfileName == mergedOwner->name);
    CHECK(assignedTake->baseRainProfileId == mergedBase->id);
    CHECK(assignedTake->baseRainProfileName == mergedBase->name);
    const auto* repairedMirror =
        invisible_places::timing::FindTimingTakeDefinition(
            takes,
            "mirror");
    REQUIRE(repairedMirror != nullptr);
    CHECK(repairedMirror->assignedRainProfileName == importedUpdate.name);
    CHECK(repairedMirror->baseRainProfileName == importedUpdate.name);
    CHECK(takes.back().id == untouchedTake.id);
    CHECK(takes.back().name == untouchedTake.name);
    CHECK(takes.back().assignedRainProfileId ==
          untouchedTake.assignedRainProfileId);
    CHECK(takes.back().assignedRainProfileName ==
          untouchedTake.assignedRainProfileName);
    CHECK(takes.back().baseRainProfileId ==
          untouchedTake.baseRainProfileId);
    CHECK(takes.back().baseRainProfileName ==
          untouchedTake.baseRainProfileName);
}

TEST_CASE(
    "standalone Rain export captures all assignments and the active effective compatibility pair",
    "[water][rain][profiles][timing][export]") {
    using invisible_places::timing::BuildTimingTakeRainStandaloneExportState;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile base;
    base.id = "base";
    base.name = "Saved Rain";
    base.settings.density = 0.22F;
    base.visual.opacity = 0.15F;
    WaterRainProfile owner = base;
    owner.id = "owner";
    owner.name = "Saved Rain_First-01";
    owner.objectOverride = true;
    owner.ownerTimingTakeId = "take-a";
    owner.baseProfileId = base.id;
    owner.baseProfileName = base.name;
    owner.settings.density = 0.81F;
    owner.visual.opacity = 0.63F;
    const std::vector<WaterRainProfile> profiles{base, owner};
    const std::vector<TimingTakeDefinition> takes{
        {.id = std::string{
             invisible_places::timing::kAuthoredTimingTakeId},
         .name = std::string{
             invisible_places::timing::kAuthoredTimingTakeName},
         .assignedRainProfileId = base.id,
         .assignedRainProfileName = base.name,
         .baseRainProfileId = base.id,
         .baseRainProfileName = base.name},
        {.id = "take-a",
         .name = "First-01",
         .assignedRainProfileId = owner.id,
         .assignedRainProfileName = owner.name,
         .baseRainProfileId = base.id,
         .baseRainProfileName = base.name},
    };
    auto fallbackSettings = base.settings;
    auto fallbackVisual = base.visual;
    fallbackSettings.density = 0.01F;
    fallbackVisual.opacity = 0.02F;

    const auto exported = BuildTimingTakeRainStandaloneExportState(
        profiles,
        takes,
        "take-a",
        fallbackSettings,
        fallbackVisual);
    CHECK(exported.profiles == profiles);
    REQUIRE(exported.assignments.size() == takes.size());
    CHECK(exported.assignments[0].id == takes[0].id);
    CHECK(exported.assignments[1].id == takes[1].id);
    CHECK(exported.assignments[1].assignedRainProfileId == owner.id);
    CHECK(exported.assignments[1].baseRainProfileId == base.id);
    CHECK(exported.compatibilitySettings == owner.settings);
    CHECK(exported.compatibilityVisual == owner.visual);

    const auto authoredFallback = BuildTimingTakeRainStandaloneExportState(
        profiles,
        takes,
        "missing-take",
        fallbackSettings,
        fallbackVisual);
    CHECK(authoredFallback.compatibilitySettings == base.settings);
    CHECK(authoredFallback.compatibilityVisual == base.visual);
}

TEST_CASE(
    "legacy standalone Rain import rebinds only the requested existing active take",
    "[water][rain][profiles][timing][import]") {
    using invisible_places::timing::MergeImportedTimingTakeRainProfiles;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile local;
    local.id = std::string{
        invisible_places::timing::kLegacyWaterRainProfileId};
    local.name = "Project Rain";
    local.settings.density = 0.12F;
    WaterRainProfile legacy;
    legacy.id = std::string{
        invisible_places::timing::kLegacyWaterRainProfileId};
    legacy.name = "Project Rain";
    legacy.settings.density = 0.88F;
    std::vector<WaterRainProfile> profiles{local};
    std::vector<TimingTakeDefinition> takes{
        {.id = "active",
         .name = "Active",
         .assignedRainProfileId = local.id,
         .assignedRainProfileName = local.name,
         .baseRainProfileId = local.id,
         .baseRainProfileName = local.name},
        {.id = "other",
         .name = "Other",
         .assignedRainProfileId = local.id,
         .assignedRainProfileName = local.name,
         .baseRainProfileId = local.id,
         .baseRainProfileName = local.name},
    };

    const auto result = MergeImportedTimingTakeRainProfiles(
        &profiles,
        &takes,
        std::span<const WaterRainProfile>{&legacy, 1U},
        {},
        "active");
    CHECK(result.profilesInserted == 1U);
    CHECK(result.assignmentsApplied == 1U);
    const auto* imported =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            takes[0].assignedRainProfileId);
    REQUIRE(imported != nullptr);
    CHECK(imported->id != local.id);
    CHECK(imported->name == "Project Rain 2");
    CHECK(imported->settings.density == Catch::Approx(0.88F));
    CHECK(takes[0].assignedRainProfileId == imported->id);
    CHECK(takes[0].assignedRainProfileName == imported->name);
    CHECK(takes[1].assignedRainProfileId == local.id);
    const auto* preservedLocal =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            local.id);
    REQUIRE(preservedLocal != nullptr);
    CHECK(preservedLocal->settings.density == Catch::Approx(0.12F));

    // Schema 31 passes no compatibility take id. An empty assignment array
    // then merges the reusable profile only and does not change a take.
    std::vector<WaterRainProfile> modernProfiles{local};
    auto modernTakes = takes;
    modernTakes[0].assignedRainProfileId = local.id;
    modernTakes[0].assignedRainProfileName = local.name;
    modernTakes[0].baseRainProfileId = local.id;
    modernTakes[0].baseRainProfileName = local.name;
    const auto modernResult = MergeImportedTimingTakeRainProfiles(
        &modernProfiles,
        &modernTakes,
        std::span<const WaterRainProfile>{&legacy, 1U},
        {});
    CHECK(modernResult.profilesInserted == 0U);
    CHECK(modernResult.profilesUpdated == 1U);
    CHECK(modernResult.assignmentsApplied == 0U);
    CHECK(modernTakes[0].assignedRainProfileId == local.id);
    CHECK(modernProfiles.front().settings.density ==
          Catch::Approx(0.88F));
}

TEST_CASE(
    "Timing Take Rain export snapshots retain stable identity and non-preset visuals after mutation",
    "[water][rain][profiles][timing][snapshot]") {
    using invisible_places::timing::CaptureTimingTakeRainProfileSnapshot;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile base;
    base.id = "snapshot-base";
    base.name = "Snapshot Rain";
    base.settings.density = 0.19F;
    WaterRainProfile owner = base;
    owner.id = "snapshot-owner";
    owner.name = "Snapshot Rain_Take";
    owner.objectOverride = true;
    owner.ownerTimingTakeId = "snapshot-take";
    owner.baseProfileId = base.id;
    owner.baseProfileName = base.name;
    owner.settings.enabled = true;
    owner.settings.visualProfileName = "Fine Lines, edited by hand";
    owner.settings.activeParticleCount = 7'321U;
    owner.settings.seed = 456U;
    owner.settings.density = 0.713F;
    owner.settings.rockImpact.downhillStretch = 2.31F;
    owner.visual.colour = {0.13F, 0.57F, 0.89F};
    owner.visual.widthMeters = 0.0063F;
    owner.visual.streakLengthMeters = 0.271F;
    owner.visual.softness = 0.23F;
    owner.visual.opacity = 0.79F;
    owner.visual.emission = 0.44F;
    owner.visual.minimumScreenPixels = 1.17F;
    owner.visual.maximumScreenPixels = 7.43F;
    const auto expected = owner;
    std::vector<WaterRainProfile> profiles{base, owner};
    std::vector<TimingTakeDefinition> takes{
        {.id = "snapshot-take",
         .name = "Snapshot Take",
         .assignedRainProfileId = owner.id,
         .assignedRainProfileName = owner.name,
         .baseRainProfileId = base.id,
         .baseRainProfileName = base.name},
    };

    const auto captured = CaptureTimingTakeRainProfileSnapshot(
        profiles,
        takes,
        "snapshot-take");
    REQUIRE(captured.has_value());
    CHECK(captured.value() == expected);

    profiles[1].id = "mutated-owner";
    profiles[1].name = "Mutated Rain";
    profiles[1].settings.density = 0.02F;
    profiles[1].visual.colour = {1.0F, 0.0F, 0.0F};
    profiles[1].visual.opacity = 0.01F;
    takes[0].assignedRainProfileId = base.id;
    takes[0].assignedRainProfileName = base.name;
    CHECK(captured->id == expected.id);
    CHECK(captured->name == expected.name);
    CHECK(captured->settings == expected.settings);
    CHECK(captured->visual == expected.visual);
    CHECK_FALSE(CaptureTimingTakeRainProfileSnapshot(
                    profiles,
                    takes,
                    "missing-take")
                    .has_value());
}

TEST_CASE(
    "Quick MP4 mixed Timing Takes keep per-item Rain and reparameterize a shared Seepage topology",
    "[water][rain][profiles][timing][snapshot][quick-mp4][seepage]") {
    using invisible_places::timing::CaptureTimingTakeRainProfileSnapshot;
    using invisible_places::timing::ProjectTimingTakeRainToScenarioSnapshot;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile shared;
    shared.id = "quick-shared";
    shared.name = "Shared Shower";
    shared.settings.enabled = true;
    shared.settings.rainLevel = 0.18F;
    shared.settings.density = 0.24F;
    shared.settings.activeParticleCount = 2'345U;
    shared.visual.colour = {0.16F, 0.42F, 0.76F};
    shared.visual.opacity = 0.31F;
    shared.visual.widthMeters = 0.0037F;

    auto owner = shared;
    owner.id = "quick-owner";
    owner.name = "Shared Shower_Owner Take";
    owner.objectOverride = true;
    owner.ownerTimingTakeId = "owner-take";
    owner.baseProfileId = shared.id;
    owner.baseProfileName = shared.name;
    owner.settings.rainLevel = 0.84F;
    owner.settings.density = 0.79F;
    owner.settings.activeParticleCount = 9'876U;
    owner.visual.colour = {0.83F, 0.36F, 0.19F};
    owner.visual.opacity = 0.72F;
    owner.visual.widthMeters = 0.0061F;
    owner.visual.streakLengthMeters = 0.287F;

    const auto expectedShared = shared;
    const auto expectedOwner = owner;
    auto liveSettings = shared.settings;
    auto liveVisual = shared.visual;
    std::vector<WaterRainProfile> profiles{shared, owner};
    std::vector<TimingTakeDefinition> takes{
        {.id = "shared-take",
         .name = "Shared Take",
         .assignedRainProfileId = shared.id,
         .assignedRainProfileName = shared.name,
         .baseRainProfileId = shared.id,
         .baseRainProfileName = shared.name},
        {.id = "owner-take",
         .name = "Owner Take",
         .assignedRainProfileId = owner.id,
         .assignedRainProfileName = owner.name,
         .baseRainProfileId = shared.id,
         .baseRainProfileName = shared.name},
    };

    std::vector<WaterRainProfile> queuedItems;
    for (const auto takeId : {"shared-take", "owner-take"}) {
        const auto captured = CaptureTimingTakeRainProfileSnapshot(
            profiles,
            takes,
            takeId);
        REQUIRE(captured.has_value());
        queuedItems.push_back(captured.value());
    }

    // A non-null shared scenario used to mask every later queue item's Rain.
    // Keep its Seepage lane, but project each frozen profile's Rain mirror.
    invisible_places::water::WaterScenarioState queueScenario;
    queueScenario.seepageLevel = 0.62F;
    queueScenario.rainLevel = 0.97F;
    invisible_places::water::WaterSeepageNode node;
    node.id = 42U;
    node.strength = 1.25F;
    const std::vector<invisible_places::water::WaterSeepageNode> nodes{node};
    const auto baseGrid = invisible_places::water::BuildWaterSeepageSpatialGrid(
        nodes,
        {},
        invisible_places::water::DefaultWaterSeepageLookSettings(),
        "ROCK",
        true,
        invisible_places::water::DefaultRainRuntimeSettings(),
        100'000'000ULL,
        {},
        queueScenario);
    REQUIRE(baseGrid.nodes.size() == 1U);

    // Mutate both the reusable library and assignment mirrors after queueing.
    // Starting either item must consume only its by-value queued snapshot.
    profiles[0].settings.rainLevel = 1.0F;
    profiles[0].visual.colour = {0.0F, 1.0F, 0.0F};
    profiles[1].id = "mutated-owner";
    profiles[1].settings.rainLevel = 0.01F;
    profiles[1].visual.opacity = 0.02F;
    takes[0].assignedRainProfileId = profiles[1].id;
    takes[1].assignedRainProfileId = profiles[0].id;
    liveSettings.rainLevel = 0.99F;
    liveSettings.density = 0.01F;
    liveVisual.colour = {1.0F, 0.0F, 1.0F};
    liveVisual.opacity = 0.01F;

    struct FrozenQuickJobRain {
        WaterRainProfile profile;
        invisible_places::water::WaterSeepageSpatialGrid seepage;
        std::optional<invisible_places::water::WaterScenarioState> scenario;
    };
    const auto startQueuedItem = [&](const WaterRainProfile& profile) {
        auto scenario = ProjectTimingTakeRainToScenarioSnapshot(
            queueScenario,
            profile.settings);
        auto seepage = baseGrid;
        invisible_places::water::ApplyWaterSeepageScenarioParameters(
            &seepage,
            scenario,
            profile.settings,
            100'000'000ULL);
        return FrozenQuickJobRain{
            .profile = profile,
            .seepage = std::move(seepage),
            .scenario = std::move(scenario),
        };
    };
    const auto sharedJob = startQueuedItem(queuedItems[0]);
    const auto ownerJob = startQueuedItem(queuedItems[1]);

    CHECK(sharedJob.profile == expectedShared);
    CHECK(ownerJob.profile == expectedOwner);
    CHECK(sharedJob.profile.settings != liveSettings);
    CHECK(sharedJob.profile.visual != liveVisual);
    REQUIRE(sharedJob.scenario.has_value());
    REQUIRE(ownerJob.scenario.has_value());
    CHECK(sharedJob.scenario->seepageLevel == Catch::Approx(0.62F));
    CHECK(ownerJob.scenario->seepageLevel == Catch::Approx(0.62F));
    CHECK(sharedJob.scenario->rainLevel == Catch::Approx(0.18F));
    CHECK(ownerJob.scenario->rainLevel == Catch::Approx(0.84F));
    REQUIRE(sharedJob.seepage.nodes.size() == 1U);
    REQUIRE(ownerJob.seepage.nodes.size() == 1U);
    CHECK(sharedJob.seepage.nodes.front().rainVisualStrength ==
          Catch::Approx(0.18F));
    CHECK(ownerJob.seepage.nodes.front().rainVisualStrength ==
          Catch::Approx(0.84F));
    CHECK(sharedJob.seepage.nodes.front().strength ==
          Catch::Approx(ownerJob.seepage.nodes.front().strength));
    CHECK(ownerJob.seepage.nodes.front().reachMeters >
          sharedJob.seepage.nodes.front().reachMeters);

    auto disabled = expectedOwner.settings;
    disabled.enabled = false;
    disabled.rainLevel = 0.99F;
    const auto disabledScenario = ProjectTimingTakeRainToScenarioSnapshot(
        queueScenario,
        disabled);
    REQUIRE(disabledScenario.has_value());
    CHECK(disabledScenario->rainLevel == Catch::Approx(0.0F));
}

TEST_CASE(
    "live Rain synchronization resets only for identity changes or forced refresh",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::ResolveTimingTakeRainLiveSyncDecision;

    CHECK(ResolveTimingTakeRainLiveSyncDecision(
              "take-a",
              "profile-a",
              "take-a",
              "profile-a") ==
          invisible_places::timing::TimingTakeRainLiveSyncDecision{});

    const auto takeChanged = ResolveTimingTakeRainLiveSyncDecision(
        "take-a",
        "profile-a",
        "take-b",
        "profile-a");
    CHECK(takeChanged.copyProfile);
    CHECK(takeChanged.resetRuntime);

    const auto profileChanged = ResolveTimingTakeRainLiveSyncDecision(
        "take-a",
        "profile-a",
        "take-a",
        "profile-b");
    CHECK(profileChanged.copyProfile);
    CHECK(profileChanged.resetRuntime);

    const auto forced = ResolveTimingTakeRainLiveSyncDecision(
        "take-a",
        "profile-a",
        "take-a",
        "profile-a",
        true);
    CHECK(forced.copyProfile);
    CHECK(forced.resetRuntime);
}

TEST_CASE(
    "Rain Timing Take metadata canonicalizes empty scalar and legacy colour tracks",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::RewriteTimingTakeRainTrackProfileMetadata;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;

    WaterFeatureTimeline rainTimeline;
    rainTimeline.feature.kind = WaterKeyedFeatureKind::Rain;
    rainTimeline.settings = {
        {.settingId = "density"},
        {.settingId = "visual.colour_red",
         .profileGroup = "rain_visual",
         .profileName = "Rain Fine Lines"},
    };
    WaterFeatureTimeline meshTimeline;
    meshTimeline.feature.kind = WaterKeyedFeatureKind::MeshFlow;
    meshTimeline.settings = {
        {.settingId = "level",
         .profileGroup = "mesh",
         .profileName = "Mesh Base"},
    };
    TimingTakeSceneState state;
    state.takeId = "take-a";
    state.waterFeatureTimingRuns = {
        WaterFeatureTimingRun{
            .id = 1U,
            .name = "Rain Run",
            .features = {rainTimeline, meshTimeline},
        },
    };
    std::vector<TimingTakeSceneState> states{state};

    CHECK(RewriteTimingTakeRainTrackProfileMetadata(
              &states,
              "take-a",
              "Project Rain_Take A") == 2U);
    const auto& rewritten =
        states.front().waterFeatureTimingRuns.front().features;
    REQUIRE(rewritten.front().settings.size() == 2U);
    for (const auto& track : rewritten.front().settings) {
        CHECK(track.profileGroup == "rain");
        CHECK(track.profileName == "Project Rain_Take A");
    }
    CHECK(rewritten.back().settings.front().profileGroup == "mesh");
    CHECK(rewritten.back().settings.front().profileName == "Mesh Base");
    CHECK(RewriteTimingTakeRainTrackProfileMetadata(
              &states,
              "take-a",
              "Project Rain_Take A") == 0U);
}

TEST_CASE(
    "legacy scenario Rain metadata follows mapped takes without changing keys",
    "[water][rain][profiles][timing][migration]") {
    using invisible_places::timing::RewriteLegacyScenarioRainTrackProfileMetadata;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterFeatureTimingRun;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterRainProfile;
    using invisible_places::water::WaterScenarioFeatureRuns;
    using invisible_places::water::WaterScenarioInterpolation;

    WaterRainProfile base;
    base.id = "base";
    base.name = "Base Rain";
    WaterRainProfile owner = base;
    owner.id = "owner";
    owner.name = "Base Rain_Take A";
    owner.objectOverride = true;
    owner.ownerTimingTakeId = "take-a";
    owner.baseProfileId = base.id;
    owner.baseProfileName = base.name;
    const std::vector<WaterRainProfile> profiles{base, owner};
    const std::vector<TimingTakeDefinition> takes{
        {.id = std::string{
             invisible_places::timing::kAuthoredTimingTakeId},
         .name = std::string{
             invisible_places::timing::kAuthoredTimingTakeName},
         .assignedRainProfileId = base.id,
         .assignedRainProfileName = base.name,
         .baseRainProfileId = base.id,
         .baseRainProfileName = base.name},
        {.id = "take-a",
         .name = "Take A",
         .assignedRainProfileId = owner.id,
         .assignedRainProfileName = owner.name,
         .baseRainProfileId = base.id,
         .baseRainProfileName = base.name},
    };

    WaterFeatureTimeline mappedRain;
    mappedRain.feature.kind = WaterKeyedFeatureKind::Rain;
    mappedRain.settings = {
        {.settingId = "density",
         .keys = {
             {.position = 0.20F,
              .value = 0.30F,
              .interpolation = WaterScenarioInterpolation::SmoothVelocity,
              .clipId = 7U},
             {.position = 0.80F,
              .value = 0.90F,
              .interpolation = WaterScenarioInterpolation::SmoothVelocity,
              .clipId = 7U},
         }},
    };
    WaterFeatureTimeline legacyRain = mappedRain;
    legacyRain.settings.front().profileGroup = "rain_visual";
    legacyRain.settings.front().profileName = "Rain Fine Lines";
    std::vector<WaterScenarioFeatureRuns> scenarios{
        {.scenarioId = "take-a",
         .runs = {WaterFeatureTimingRun{
             .id = 1U,
             .features = {mappedRain},
         }}},
        {.scenarioId = "legacy-scenario",
         .runs = {WaterFeatureTimingRun{
             .id = 2U,
             .features = {legacyRain},
         }}},
    };
    const auto beforeKeys =
        scenarios.front().runs.front().features.front().settings.front().keys;

    CHECK(RewriteLegacyScenarioRainTrackProfileMetadata(
              &scenarios,
              profiles,
              takes) == 2U);
    const auto& mappedTrack = scenarios.front()
                                  .runs.front()
                                  .features.front()
                                  .settings.front();
    CHECK(mappedTrack.profileGroup == "rain");
    CHECK(mappedTrack.profileName == owner.name);
    const auto& fallbackTrack = scenarios.back()
                                    .runs.front()
                                    .features.front()
                                    .settings.front();
    CHECK(fallbackTrack.profileGroup == "rain");
    CHECK(fallbackTrack.profileName == base.name);
    REQUIRE(mappedTrack.keys.size() == beforeKeys.size());
    for (std::size_t index = 0U; index < beforeKeys.size(); ++index) {
        CHECK(mappedTrack.keys[index].position ==
              Catch::Approx(beforeKeys[index].position));
        CHECK(mappedTrack.keys[index].value ==
              Catch::Approx(beforeKeys[index].value));
        CHECK(mappedTrack.keys[index].interpolation ==
              beforeKeys[index].interpolation);
        CHECK(mappedTrack.keys[index].clipId == beforeKeys[index].clipId);
    }
}

TEST_CASE(
    "Timing Take Rain owner profiles follow assign edit rename duplicate and delete lifecycle",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::AssignTimingTakeRainBaseProfile;
    using invisible_places::timing::DuplicateTimingTakeRainProfileAssignment;
    using invisible_places::timing::RemoveTimingTakeRainOwnerProfiles;
    using invisible_places::timing::RenameTimingTakeRainOwnerProfile;
    using invisible_places::timing::ResolveTimingTakeRainProfile;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::timing::UpsertTimingTakeRainOwnerProfile;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile base;
    base.id = "rain-profile-fine";
    base.name = "Fine Lines";
    base.settings.density = 0.40F;
    base.visual.opacity = 0.08F;
    WaterRainProfile nameCollision;
    nameCollision.id = "rain-profile-collision";
    nameCollision.name = "Fine Lines_First-01";
    std::vector<WaterRainProfile> profiles{base, nameCollision};
    std::vector<TimingTakeDefinition> takes{
        {.id = "timing-take-8", .name = "First-01"},
        {.id = "timing-take-9", .name = "First-02"},
    };
    REQUIRE(AssignTimingTakeRainBaseProfile(
        &takes[0], profiles, base.id));
    REQUIRE(AssignTimingTakeRainBaseProfile(
        &takes[1], profiles, base.id));

    auto editedSettings = base.settings;
    auto editedVisual = base.visual;
    editedSettings.density = 0.91F;
    editedVisual.opacity = 0.42F;
    auto* firstCopy = UpsertTimingTakeRainOwnerProfile(
        &profiles,
        &takes[0],
        editedSettings,
        editedVisual);
    REQUIRE(firstCopy != nullptr);
    const auto firstCopyId = firstCopy->id;
    CHECK(firstCopy->name == "Fine Lines_First-01 2");
    CHECK(firstCopy->objectOverride);
    CHECK(firstCopy->ownerTimingTakeId == "timing-take-8");
    CHECK(firstCopy->baseProfileId == base.id);
    CHECK(ResolveTimingTakeRainProfile(profiles, takes[0])->settings.density ==
          Approx(0.91F));
    CHECK(ResolveTimingTakeRainProfile(profiles, takes[1])->settings.density ==
          Approx(0.40F));

    // A second take may inspect the copy by stable id; rename updates its
    // human-readable mirror without changing the assignment identity.
    TimingTakeDefinition observer{
        .id = "observer",
        .name = "Observer",
        .assignedRainProfileId = firstCopyId,
        .assignedRainProfileName = firstCopy->name,
        .baseRainProfileId = base.id,
        .baseRainProfileName = base.name,
    };
    takes.push_back(observer);
    takes[0].name = "First Rain";
    REQUIRE(RenameTimingTakeRainOwnerProfile(
        &profiles,
        &takes,
        takes[0].id));
    firstCopy = invisible_places::water::FindWaterRainProfileById(
        &profiles,
        firstCopyId);
    REQUIRE(firstCopy != nullptr);
    CHECK(firstCopy->name == "Fine Lines_First Rain");
    CHECK(takes[2].assignedRainProfileName == firstCopy->name);

    TimingTakeDefinition duplicate{
        .id = "timing-take-10",
        .name = "First Rain Copy",
    };
    REQUIRE(DuplicateTimingTakeRainProfileAssignment(
        &profiles,
        takes[0],
        &duplicate));
    REQUIRE(duplicate.assignedRainProfileId != firstCopyId);
    const auto* duplicateProfile = ResolveTimingTakeRainProfile(
        profiles,
        duplicate);
    REQUIRE(duplicateProfile != nullptr);
    const auto duplicateProfileId = duplicateProfile->id;
    CHECK(duplicateProfile->objectOverride);
    CHECK(duplicateProfile->ownerTimingTakeId == duplicate.id);
    CHECK(duplicateProfile->settings == editedSettings);
    CHECK(duplicateProfile->visual == editedVisual);
    takes.push_back(duplicate);

    CHECK(RemoveTimingTakeRainOwnerProfiles(
              &profiles,
              &takes,
              takes[0].id) == 1U);
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              firstCopyId) == nullptr);
    CHECK(takes[2].assignedRainProfileId == base.id);
    CHECK(ResolveTimingTakeRainProfile(profiles, takes.back())->id ==
          duplicateProfileId);
}

TEST_CASE(
    "Rain profile sanitation deterministically repairs duplicate ids names and references",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::SanitizeWaterRainProfileLibrary;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile first;
    first.id = "shared-duplicate";
    first.name = "Rain";
    WaterRainProfile second;
    second.id = "shared-duplicate";
    second.name = "Storm";
    WaterRainProfile owner;
    owner.id = "owner-copy";
    owner.name = "Rain";
    owner.objectOverride = true;
    owner.ownerTimingTakeId = "timing-take-8";
    owner.baseProfileId = "shared-duplicate";
    owner.baseProfileName = "Storm";
    WaterRainProfile reservedSuffix;
    reservedSuffix.id = "shared-duplicate-2";
    reservedSuffix.name = "Rain 2";
    std::vector<WaterRainProfile> profiles{
        first,
        second,
        owner,
        reservedSuffix,
    };
    std::vector<TimingTakeDefinition> takes{
        {.id = "timing-take-8",
         .name = "First-01",
         .assignedRainProfileId = "owner-copy",
         .assignedRainProfileName = "Rain",
         .baseRainProfileId = "shared-duplicate",
         .baseRainProfileName = "Storm"},
        {.id = "timing-take-9",
         .name = "First-02",
         .assignedRainProfileId = "shared-duplicate",
         .assignedRainProfileName = "Storm",
         .baseRainProfileId = "shared-duplicate",
         .baseRainProfileName = "Storm"},
    };

    SanitizeWaterRainProfileLibrary(&profiles, &takes);
    REQUIRE(profiles.size() == 4U);
    CHECK(profiles[0].id == "shared-duplicate");
    CHECK(profiles[1].id == "shared-duplicate-3");
    CHECK(profiles[2].id == "owner-copy");
    CHECK(profiles[3].id == "shared-duplicate-2");
    CHECK(profiles[0].name == "Rain");
    CHECK(profiles[1].name == "Storm");
    CHECK(profiles[2].name == "Rain 3");
    CHECK(profiles[3].name == "Rain 2");
    CHECK(profiles[2].baseProfileId == profiles[1].id);
    CHECK(profiles[2].baseProfileName == profiles[1].name);
    CHECK(takes[0].assignedRainProfileId == profiles[2].id);
    CHECK(takes[0].assignedRainProfileName == profiles[2].name);
    CHECK(takes[0].baseRainProfileId == profiles[1].id);
    CHECK(takes[1].assignedRainProfileId == profiles[1].id);

    const auto once = profiles;
    const auto onceTakes = takes;
    SanitizeWaterRainProfileLibrary(&profiles, &takes);
    CHECK(profiles == once);
    REQUIRE(takes.size() == onceTakes.size());
    for (std::size_t index = 0U; index < takes.size(); ++index) {
        CHECK(takes[index].id == onceTakes[index].id);
        CHECK(takes[index].name == onceTakes[index].name);
        CHECK(takes[index].assignedRainProfileId ==
              onceTakes[index].assignedRainProfileId);
        CHECK(takes[index].assignedRainProfileName ==
              onceTakes[index].assignedRainProfileName);
        CHECK(takes[index].baseRainProfileId ==
              onceTakes[index].baseRainProfileId);
        CHECK(takes[index].baseRainProfileName ==
              onceTakes[index].baseRainProfileName);
    }
}

TEST_CASE(
    "Timing Take Rain rename updates every base-specific owner copy",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::AssignTimingTakeRainBaseProfile;
    using invisible_places::timing::RenameTimingTakeRainOwnerProfile;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::timing::UpsertTimingTakeRainOwnerProfile;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile fine;
    fine.id = "fine";
    fine.name = "Fine";
    WaterRainProfile heavy;
    heavy.id = "heavy";
    heavy.name = "Heavy";
    std::vector<WaterRainProfile> profiles{fine, heavy};
    std::vector<TimingTakeDefinition> takes{
        {.id = "take-a", .name = "Before"},
        {.id = "observer-a", .name = "Observer A"},
        {.id = "observer-b", .name = "Observer B"},
    };

    REQUIRE(AssignTimingTakeRainBaseProfile(&takes[0], profiles, fine.id));
    REQUIRE(UpsertTimingTakeRainOwnerProfile(
                &profiles,
                &takes[0],
                fine.settings,
                fine.visual) != nullptr);
    const auto fineCopyId = takes[0].assignedRainProfileId;
    takes[1] = takes[0];
    takes[1].id = "observer-a";
    takes[1].name = "Observer A";

    REQUIRE(AssignTimingTakeRainBaseProfile(&takes[0], profiles, heavy.id));
    REQUIRE(UpsertTimingTakeRainOwnerProfile(
                &profiles,
                &takes[0],
                heavy.settings,
                heavy.visual) != nullptr);
    const auto heavyCopyId = takes[0].assignedRainProfileId;
    takes[2] = takes[0];
    takes[2].id = "observer-b";
    takes[2].name = "Observer B";

    takes[0].name = "After";
    REQUIRE(RenameTimingTakeRainOwnerProfile(
        &profiles,
        &takes,
        takes[0].id));
    const auto* fineCopy = invisible_places::water::FindWaterRainProfileById(
        profiles,
        fineCopyId);
    const auto* heavyCopy = invisible_places::water::FindWaterRainProfileById(
        profiles,
        heavyCopyId);
    REQUIRE(fineCopy != nullptr);
    REQUIRE(heavyCopy != nullptr);
    CHECK(fineCopy->name == "Fine_After");
    CHECK(heavyCopy->name == "Heavy_After");
    CHECK(takes[1].assignedRainProfileName == fineCopy->name);
    CHECK(takes[2].assignedRainProfileName == heavyCopy->name);
}

TEST_CASE(
    "Timing Take Rain Save and Discard return owner edits to shared profiles",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::AssignTimingTakeRainBaseProfile;
    using invisible_places::timing::DiscardTimingTakeRainOwnerProfile;
    using invisible_places::timing::ResolveTimingTakeRainProfile;
    using invisible_places::timing::SaveTimingTakeRainOwnerProfileAsShared;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::timing::UpsertTimingTakeRainOwnerProfile;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile base;
    base.id = "base";
    base.name = "Base";
    base.settings.density = 0.2F;
    std::vector<WaterRainProfile> profiles{base};
    std::vector<TimingTakeDefinition> takes{
        {.id = "take", .name = "Take"},
        {.id = "observer", .name = "Observer"},
    };
    REQUIRE(AssignTimingTakeRainBaseProfile(&takes[0], profiles, base.id));
    REQUIRE(AssignTimingTakeRainBaseProfile(&takes[1], profiles, base.id));

    auto edited = base.settings;
    edited.density = 0.7F;
    REQUIRE(UpsertTimingTakeRainOwnerProfile(
                &profiles,
                &takes[0],
                edited,
                base.visual) != nullptr);
    const auto discardedOwnerId = takes[0].assignedRainProfileId;
    takes[1].assignedRainProfileId = discardedOwnerId;
    takes[1].assignedRainProfileName = takes[0].assignedRainProfileName;
    REQUIRE(DiscardTimingTakeRainOwnerProfile(
        &profiles,
        &takes,
        takes[0].id));
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              discardedOwnerId) == nullptr);
    CHECK(takes[0].assignedRainProfileId == base.id);
    CHECK(takes[1].assignedRainProfileId == base.id);
    CHECK(ResolveTimingTakeRainProfile(profiles, takes[0])->settings.density ==
          Catch::Approx(0.2F));

    REQUIRE(UpsertTimingTakeRainOwnerProfile(
                &profiles,
                &takes[0],
                edited,
                base.visual) != nullptr);
    const auto savedOwnerId = takes[0].assignedRainProfileId;
    takes[1].assignedRainProfileId = savedOwnerId;
    takes[1].assignedRainProfileName = takes[0].assignedRainProfileName;
    auto* saved = SaveTimingTakeRainOwnerProfileAsShared(
        &profiles,
        &takes,
        takes[0].id,
        "Saved Rain");
    REQUIRE(saved != nullptr);
    const auto savedId = saved->id;
    CHECK(saved->name == "Saved Rain");
    CHECK_FALSE(saved->objectOverride);
    CHECK(saved->settings.density == Catch::Approx(0.7F));
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              savedOwnerId) == nullptr);
    CHECK(takes[0].assignedRainProfileId == savedId);
    CHECK(takes[1].assignedRainProfileId == savedId);

    auto overwritten = saved->settings;
    const auto overwrittenVisual = saved->visual;
    overwritten.density = 0.9F;
    REQUIRE(UpsertTimingTakeRainOwnerProfile(
                &profiles,
                &takes[0],
                overwritten,
                overwrittenVisual) != nullptr);
    auto* promoted = SaveTimingTakeRainOwnerProfileAsShared(
        &profiles,
        &takes,
        takes[0].id,
        "Saved Rain",
        true);
    REQUIRE(promoted != nullptr);
    CHECK(promoted->id == savedId);
    CHECK(promoted->settings.density == Catch::Approx(0.9F));
}

TEST_CASE(
    "Timing Take Rain Save targets the resolved base while Save As stays collision safe",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::AssignTimingTakeRainBaseProfile;
    using invisible_places::timing::SaveTimingTakeRainOwnerProfileAsShared;
    using invisible_places::timing::SaveTimingTakeRainOwnerProfileToBase;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::timing::UpsertTimingTakeRainOwnerProfile;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile baseA;
    baseA.id = "base-a";
    baseA.name = "Base A";
    baseA.settings.density = 0.2F;
    WaterRainProfile baseB;
    baseB.id = "base-b";
    baseB.name = "Base B";
    baseB.settings.density = 0.4F;
    baseB.settings.fallSpeedMetersPerSecond = 13.0F;
    const auto baseBBefore = baseB;
    std::vector<WaterRainProfile> profiles{baseA, baseB};
    std::vector<TimingTakeDefinition> takes{
        {.id = "take", .name = "Take"},
    };
    REQUIRE(AssignTimingTakeRainBaseProfile(
        &takes[0],
        profiles,
        baseA.id));

    auto edited = baseA.settings;
    edited.density = 0.7F;
    const auto* owner = UpsertTimingTakeRainOwnerProfile(
        &profiles,
        &takes[0],
        edited,
        baseA.visual);
    REQUIRE(owner != nullptr);
    const auto ownerId = owner->id;
    takes[0].baseRainProfileId = "missing-base";
    takes[0].baseRainProfileName = baseB.name;

    // Both the editable Save As name and the take's stale base mirrors point
    // away from Base A. The owned copy's stable base id remains authoritative.
    auto* saved = SaveTimingTakeRainOwnerProfileAsShared(
        &profiles,
        &takes,
        takes[0].id,
        baseB.name,
        true);
    REQUIRE(saved != nullptr);
    CHECK(saved->id == baseA.id);
    CHECK(saved->name == baseA.name);
    CHECK(saved->settings.density == Catch::Approx(0.7F));
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              ownerId) == nullptr);
    CHECK(takes[0].assignedRainProfileId == baseA.id);
    CHECK(takes[0].baseRainProfileId == baseA.id);
    const auto* unchangedB =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            baseB.id);
    REQUIRE(unchangedB != nullptr);
    CHECK(*unchangedB == baseBBefore);

    auto saveAsEdit = saved->settings;
    const auto saveAsVisual = saved->visual;
    saveAsEdit.density = 0.85F;
    REQUIRE(UpsertTimingTakeRainOwnerProfile(
                &profiles,
                &takes[0],
                saveAsEdit,
                saveAsVisual) != nullptr);
    auto* savedAs = SaveTimingTakeRainOwnerProfileAsShared(
        &profiles,
        &takes,
        takes[0].id,
        baseB.name);
    REQUIRE(savedAs != nullptr);
    const auto savedAsId = savedAs->id;
    const auto savedAsName = savedAs->name;
    CHECK(savedAsId != baseA.id);
    CHECK(savedAsId != baseB.id);
    CHECK(savedAsName == "Base B 2");
    unchangedB = invisible_places::water::FindWaterRainProfileById(
        profiles,
        baseB.id);
    REQUIRE(unchangedB != nullptr);
    CHECK(*unchangedB == baseBBefore);

    auto postSaveAsEdit = savedAs->settings;
    const auto postSaveAsVisual = savedAs->visual;
    postSaveAsEdit.density = 0.95F;
    owner = UpsertTimingTakeRainOwnerProfile(
        &profiles,
        &takes[0],
        postSaveAsEdit,
        postSaveAsVisual);
    REQUIRE(owner != nullptr);
    const auto postSaveAsOwnerId = owner->id;
    CHECK(owner->name == savedAsName + "_" + takes[0].name);
    CHECK(owner->baseProfileId == savedAsId);
    CHECK(takes[0].assignedRainProfileId == owner->id);
    CHECK(takes[0].baseRainProfileId == savedAsId);

    auto* savedBackToBase = SaveTimingTakeRainOwnerProfileToBase(
        &profiles,
        &takes,
        takes[0].id);
    REQUIRE(savedBackToBase != nullptr);
    CHECK(savedBackToBase->id == savedAsId);
    CHECK(savedBackToBase->settings.density == Catch::Approx(0.95F));
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              postSaveAsOwnerId) == nullptr);
    CHECK(takes[0].assignedRainProfileId == savedAsId);
}

TEST_CASE(
    "Timing Take Rain Save fails closed on unresolved owner base identity",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::AssignTimingTakeRainBaseProfile;
    using invisible_places::timing::SaveTimingTakeRainOwnerProfileToBase;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::timing::UpsertTimingTakeRainOwnerProfile;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile baseA;
    baseA.id = "base-a";
    baseA.name = "Base A";
    baseA.settings.density = 0.2F;
    WaterRainProfile baseB;
    baseB.id = "base-b";
    baseB.name = "Base B";
    baseB.settings.density = 0.4F;
    std::vector<WaterRainProfile> profiles{baseA, baseB};
    std::vector<TimingTakeDefinition> takes{
        {.id = "take", .name = "Take"},
    };
    REQUIRE(AssignTimingTakeRainBaseProfile(
        &takes[0],
        profiles,
        baseA.id));

    auto edited = baseA.settings;
    edited.density = 0.8F;
    auto* owner = UpsertTimingTakeRainOwnerProfile(
        &profiles,
        &takes[0],
        edited,
        baseA.visual);
    REQUIRE(owner != nullptr);
    const auto ownerId = owner->id;
    owner->baseProfileId = "missing-base";
    owner->baseProfileName = baseB.name;
    takes[0].baseRainProfileId = baseB.id;
    takes[0].baseRainProfileName = baseB.name;
    const auto baseABefore = baseA;
    const auto baseBBefore = baseB;

    CHECK(SaveTimingTakeRainOwnerProfileToBase(
              &profiles,
              &takes,
              takes[0].id) == nullptr);
    const auto* unchangedA =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            baseA.id);
    const auto* unchangedB =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            baseB.id);
    REQUIRE(unchangedA != nullptr);
    REQUIRE(unchangedB != nullptr);
    CHECK(*unchangedA == baseABefore);
    CHECK(*unchangedB == baseBBefore);
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              ownerId) != nullptr);
    CHECK(takes[0].assignedRainProfileId == ownerId);
}

TEST_CASE(
    "Timing Take Rain Save rejects a foreign take owner copy",
    "[water][rain][profiles][timing]") {
    using invisible_places::timing::AssignTimingTakeRainBaseProfile;
    using invisible_places::timing::SaveTimingTakeRainOwnerProfileToBase;
    using invisible_places::timing::TimingTakeDefinition;
    using invisible_places::timing::UpsertTimingTakeRainOwnerProfile;
    using invisible_places::water::WaterRainProfile;

    WaterRainProfile base;
    base.id = "base";
    base.name = "Base";
    base.settings.density = 0.2F;
    std::vector<WaterRainProfile> profiles{base};
    std::vector<TimingTakeDefinition> takes{
        {.id = "owner-take", .name = "Owner"},
        {.id = "observer-take", .name = "Observer"},
    };
    REQUIRE(AssignTimingTakeRainBaseProfile(
        &takes[0],
        profiles,
        base.id));
    REQUIRE(AssignTimingTakeRainBaseProfile(
        &takes[1],
        profiles,
        base.id));

    auto edited = base.settings;
    edited.density = 0.8F;
    const auto* owner = UpsertTimingTakeRainOwnerProfile(
        &profiles,
        &takes[0],
        edited,
        base.visual);
    REQUIRE(owner != nullptr);
    const auto ownerId = owner->id;
    takes[1].assignedRainProfileId = owner->id;
    takes[1].assignedRainProfileName = owner->name;
    const auto baseBefore = base;

    CHECK(SaveTimingTakeRainOwnerProfileToBase(
              &profiles,
              &takes,
              takes[1].id) == nullptr);
    const auto* unchangedBase =
        invisible_places::water::FindWaterRainProfileById(
            profiles,
            base.id);
    REQUIRE(unchangedBase != nullptr);
    CHECK(*unchangedBase == baseBefore);
    CHECK(invisible_places::water::FindWaterRainProfileById(
              profiles,
              ownerId) != nullptr);
    CHECK(takes[0].assignedRainProfileId == ownerId);
    CHECK(takes[1].assignedRainProfileId == ownerId);
}
