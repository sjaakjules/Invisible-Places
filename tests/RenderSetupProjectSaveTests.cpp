#include "app/RenderSetupProjectSave.hpp"

#include <catch2/catch_test_macros.hpp>

#include <string_view>

namespace {

using invisible_places::serialization::ProjectDocument;
using invisible_places::timing::TimingTakeSceneState;
using invisible_places::water::WaterFeatureTimingRun;
using invisible_places::water::WaterFeatureTimeline;
using invisible_places::water::WaterKeyedFeatureKind;
using invisible_places::water::WaterKeyedSettingTrack;
using invisible_places::water::WaterKeyedSettingsProfile;
using invisible_places::water::WaterScenarioInterpolation;
using invisible_places::water::WaterSettingKey;

WaterKeyedSettingTrack MakeStrengthTrack(float value) {
    WaterKeyedSettingTrack track;
    track.settingId = "strength";
    track.defaultInterpolation =
        WaterScenarioInterpolation::SmoothVelocity;
    track.keys = {
        WaterSettingKey{.position = 0.0F, .value = 0.0F},
        WaterSettingKey{.position = 1.0F, .value = value},
    };
    return track;
}

WaterKeyedSettingsProfile MakePackage(
    std::string name,
    float value) {
    WaterKeyedSettingsProfile profile;
    profile.name = std::move(name);
    profile.baseProfileName = "Default";
    profile.featureKind = WaterKeyedFeatureKind::SeepageNode;
    profile.settings = {MakeStrengthTrack(value)};
    return profile;
}

TimingTakeSceneState MakeTimingState(
    std::string takeId,
    float value) {
    WaterFeatureTimeline timeline;
    timeline.feature = {
        .kind = WaterKeyedFeatureKind::SeepageNode,
        .objectId = 20U,
    };
    timeline.settings = {MakeStrengthTrack(value)};

    WaterFeatureTimingRun run;
    run.id = 1U;
    run.name = "Seepage progression";
    run.features = {std::move(timeline)};

    TimingTakeSceneState state;
    state.takeId = std::move(takeId);
    state.sceneGroupName = "First-01";
    state.waterFeatureTimingRuns = {std::move(run)};
    state.waterFeatureTimingRunSequence = 2U;
    return state;
}

const WaterKeyedSettingsProfile* FindPackage(
    const ProjectDocument& project,
    std::string_view name) {
    const auto index = invisible_places::water::
        FindWaterKeyedSettingsProfileIndex(
            project.waterKeyedSettingsProfiles,
            WaterKeyedFeatureKind::SeepageNode,
            name);
    return index.has_value()
               ? &project.waterKeyedSettingsProfiles[index.value()]
               : nullptr;
}

WaterKeyedSettingsProfile* FindPackage(
    ProjectDocument& project,
    std::string_view name) {
    return const_cast<WaterKeyedSettingsProfile*>(
        FindPackage(
            static_cast<const ProjectDocument&>(project),
            name));
}

float PackageEndValue(const WaterKeyedSettingsProfile& profile) {
    REQUIRE(profile.settings.size() == 1U);
    REQUIRE(profile.settings.front().keys.size() == 2U);
    return profile.settings.front().keys.back().value;
}

float TimingStateEndValue(const TimingTakeSceneState& state) {
    REQUIRE(state.waterFeatureTimingRuns.size() == 1U);
    REQUIRE(state.waterFeatureTimingRuns.front().features.size() == 1U);
    const auto& settings =
        state.waterFeatureTimingRuns.front().features.front().settings;
    REQUIRE(settings.size() == 1U);
    REQUIRE(settings.front().keys.size() == 2U);
    return settings.front().keys.back().value;
}

TEST_CASE(
    "active render setup save keeps new packages and timing authoring without baking the preview",
    "[render-setup][project-save]") {
    ProjectDocument underlying;
    underlying.backgroundColor = {0.1F, 0.2F, 0.3F, 1.0F};
    underlying.selectedPointVisualName = "Project visual";
    underlying.waterKeyedSettingsProfiles = {
        MakePackage("Project seepage", 0.2F),
    };
    underlying.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "project-take", .name = "Project take"},
    };
    underlying.selectedTimingTakeId = "project-take";
    underlying.timingTakeStates = {
        MakeTimingState("project-take", 0.25F),
    };
    underlying.timingTakeSequence = 4U;
    invisible_places::camera::CameraState underlyingCamera;
    underlyingCamera.position = {1.0F, 2.0F, 3.0F};
    underlying.cameraState = underlyingCamera;
    underlying.activeAnimationPath = "animations/project.ipanim.json";

    auto previewBaseline = underlying;
    previewBaseline.backgroundColor = {0.8F, 0.7F, 0.6F, 1.0F};
    previewBaseline.selectedPointVisualName = "Setup visual";
    previewBaseline.waterKeyedSettingsProfiles = {
        MakePackage("Setup-only seepage", 0.8F),
    };
    previewBaseline.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "setup-take", .name = "Setup take"},
    };
    previewBaseline.selectedTimingTakeId = "setup-take";
    previewBaseline.timingTakeStates = {
        MakeTimingState("setup-take", 0.8F),
    };

    auto live = previewBaseline;
    live.waterKeyedSettingsProfiles.push_back(
        MakePackage("Authored while previewing", 0.55F));
    live.timingTakes.push_back(
        {.id = "new-take", .name = "Authored while previewing"});
    live.timingTakeStates.push_back(
        MakeTimingState("new-take", 0.65F));
    live.timingTakeSequence = 9U;
    invisible_places::camera::CameraState liveCamera;
    liveCamera.position = {9.0F, 8.0F, 7.0F};
    live.cameraState = liveCamera;
    live.activeAnimationPath = "animations/live-edit.ipanim.json";
    live.activeAnimationPosition = 0.75F;

    const auto saved =
        invisible_places::app::MergeRenderSetupProjectForSave(
            underlying,
            previewBaseline,
            std::move(live));

    REQUIRE(FindPackage(saved, "Project seepage") != nullptr);
    CHECK(
        PackageEndValue(*FindPackage(saved, "Project seepage")) ==
        0.2F);
    REQUIRE(FindPackage(saved, "Authored while previewing") != nullptr);
    CHECK(
        PackageEndValue(
            *FindPackage(saved, "Authored while previewing")) ==
        0.55F);
    CHECK(FindPackage(saved, "Setup-only seepage") == nullptr);

    CHECK(
        invisible_places::timing::FindTimingTakeDefinition(
            saved.timingTakes,
            "project-take") != nullptr);
    CHECK(
        invisible_places::timing::FindTimingTakeDefinition(
            saved.timingTakes,
            "new-take") != nullptr);
    CHECK(
        invisible_places::timing::FindTimingTakeDefinition(
            saved.timingTakes,
            "setup-take") == nullptr);
    CHECK(
        invisible_places::timing::FindTimingTakeSceneState(
            saved.timingTakeStates,
            "project-take",
            "First-01") != nullptr);
    CHECK(
        invisible_places::timing::FindTimingTakeSceneState(
            saved.timingTakeStates,
            "new-take",
            "First-01") != nullptr);
    CHECK(
        invisible_places::timing::FindTimingTakeSceneState(
            saved.timingTakeStates,
            "setup-take",
            "First-01") == nullptr);
    CHECK(saved.timingTakeSequence == 9U);

    CHECK(saved.backgroundColor == underlying.backgroundColor);
    CHECK(saved.selectedPointVisualName == "Project visual");
    CHECK(saved.selectedTimingTakeId == "project-take");
    REQUIRE(saved.cameraState.has_value());
    CHECK(saved.cameraState->position == liveCamera.position);
    CHECK(
        saved.activeAnimationPath ==
        std::filesystem::path{"animations/live-edit.ipanim.json"});
    CHECK(saved.activeAnimationPosition == 0.75F);
}

TEST_CASE(
    "active render setup save applies modified and removed package and timing deltas",
    "[render-setup][project-save]") {
    ProjectDocument underlying;
    underlying.waterKeyedSettingsProfiles = {
        MakePackage("Shared package", 0.2F),
        MakePackage("Removed package", 0.3F),
    };
    underlying.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "shared-take", .name = "Underlying name"},
        {.id = "removed-take", .name = "Removed"},
    };
    underlying.timingTakeStates = {
        MakeTimingState("shared-take", 0.2F),
        MakeTimingState("removed-take", 0.3F),
    };

    auto previewBaseline = underlying;
    previewBaseline.waterKeyedSettingsProfiles = {
        MakePackage("Shared package", 0.6F),
        MakePackage("Removed package", 0.7F),
    };
    previewBaseline.timingTakes[1U].name = "Setup name";
    *invisible_places::timing::FindTimingTakeSceneState(
        &previewBaseline.timingTakeStates,
        "shared-take",
        "First-01") = MakeTimingState("shared-take", 0.6F);

    auto live = previewBaseline;
    FindPackage(live, "Shared package")->settings.front().keys.back().value =
        0.9F;
    live.waterKeyedSettingsProfiles.erase(
        live.waterKeyedSettingsProfiles.begin() + 1);
    invisible_places::timing::FindTimingTakeDefinition(
        &live.timingTakes,
        "shared-take")->name = "Authored name";
    live.timingTakes.erase(live.timingTakes.begin() + 2);
    auto* liveState = invisible_places::timing::FindTimingTakeSceneState(
        &live.timingTakeStates,
        "shared-take",
        "First-01");
    REQUIRE(liveState != nullptr);
    liveState->waterFeatureTimingRuns.front()
        .features.front()
        .settings.front()
        .keys.back()
        .value = 0.95F;
    std::erase_if(
        live.timingTakeStates,
        [](const TimingTakeSceneState& state) {
            return state.takeId == "removed-take";
        });

    const auto saved =
        invisible_places::app::MergeRenderSetupProjectForSave(
            underlying,
            previewBaseline,
            std::move(live));

    REQUIRE(FindPackage(saved, "Shared package") != nullptr);
    CHECK(
        PackageEndValue(*FindPackage(saved, "Shared package")) ==
        0.9F);
    CHECK(FindPackage(saved, "Removed package") == nullptr);
    const auto* savedTake =
        invisible_places::timing::FindTimingTakeDefinition(
            saved.timingTakes,
            "shared-take");
    REQUIRE(savedTake != nullptr);
    CHECK(savedTake->name == "Authored name");
    CHECK(
        invisible_places::timing::FindTimingTakeDefinition(
            saved.timingTakes,
            "removed-take") == nullptr);
    const auto* savedState =
        invisible_places::timing::FindTimingTakeSceneState(
            saved.timingTakeStates,
            "shared-take",
            "First-01");
    REQUIRE(savedState != nullptr);
    CHECK(TimingStateEndValue(*savedState) == 0.95F);
    CHECK(
        invisible_places::timing::FindTimingTakeSceneState(
            saved.timingTakeStates,
            "removed-take",
            "First-01") == nullptr);
}

}  // namespace
