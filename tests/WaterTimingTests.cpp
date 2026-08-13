#include "InvisiblePlacesBuildConfig.hpp"
#include "serialization/ProjectDocument.hpp"
#include "timing/TimingColourise.hpp"
#include "timing/TimelineView.hpp"
#include "water/WaterFlow.hpp"
#include "water/WaterSeepagePulseField.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
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
            .profileGroup = "seepage_look",
            .profileName = "Wet Rock",
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
    CHECK(timeline.settings.front().profileGroup == "seepage_look");
    CHECK(timeline.settings.front().profileName == "Wet Rock");
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

TEST_CASE("Settings clips and package lengths round-trip through the project document",
          "[water][timing][keyed][clips][serialization]") {
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
          "[water][timing][keyed][clips][pan-extension]") {
    using Catch::Approx;
    using invisible_places::timing::RetimeTimingTakeSceneStateNormalizedPositions;
    using invisible_places::timing::TimingTakeSceneState;
    using invisible_places::water::WaterKeyedFeatureKind;

    TimingTakeSceneState state;
    invisible_places::water::WaterFeatureTimingRun run;
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
    CHECK(timeline.settings.front().keys.front().position ==
          Approx(0.625F));
    CHECK(timeline.settings.front().keys.back().position ==
          Approx(0.875F));
    CHECK(timeline.clips.front().start == Approx(0.625F));
    CHECK(timeline.clips.front().end == Approx(0.875F));
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
}
