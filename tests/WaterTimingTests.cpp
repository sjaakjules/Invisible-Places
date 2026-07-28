#include "serialization/ProjectDocument.hpp"
#include "water/WaterFlow.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
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
    const auto overlay = BuildWaterFeatureTimingOverlay(runs, 0.25F);
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
    ApplyWaterFeatureTimingOverlayToScenario(overlay, &state);
    CHECK(state.rainLevel == Approx(0.25F));
    CHECK(state.flowLevel == Approx(0.7F));

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
    entry.runs.push_back(run);
    document.waterFeatureTimingRuns.push_back(entry);
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
    REQUIRE(loadedRun.features.size() == 1U);
    const auto& timeline = loadedRun.features.front();
    CHECK(timeline.feature.kind == WaterKeyedFeatureKind::SeepageNode);
    CHECK(timeline.feature.objectId == 4U);
    REQUIRE(timeline.settings.size() == 1U);
    REQUIRE(timeline.settings.front().keys.size() == 2U);
    CHECK(timeline.settings.front().keys[0].position == Approx(0.20F));
    CHECK(timeline.settings.front().keys[0].interpolation ==
          WaterScenarioInterpolation::Hold);
    CHECK(timeline.settings.front().keys[1].value == Approx(1.2F));
    CHECK(loaded->waterFeatureTimingRunSequence == 8U);
}
