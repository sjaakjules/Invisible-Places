#include "InvisiblePlacesBuildConfig.hpp"
#include "serialization/ProjectDocument.hpp"
#include "timing/TimelineView.hpp"
#include "water/WaterFlow.hpp"

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
