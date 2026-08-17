#include "app/RenderSetupProjectSave.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string_view>

namespace {

using invisible_places::serialization::ProjectDocument;
using invisible_places::serialization::WaterSourcesDocument;
using invisible_places::timing::TimingTakeSceneState;
using invisible_places::water::WaterFeatureTimingRun;
using invisible_places::water::WaterFeatureTimeline;
using invisible_places::water::WaterKeyedFeatureKind;
using invisible_places::water::WaterKeyedSettingTrack;
using invisible_places::water::WaterKeyedSettingsProfile;
using invisible_places::water::WaterRainProfile;
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

WaterRainProfile MakeRainProfile(
    std::string id,
    std::string name,
    float density) {
    WaterRainProfile profile;
    profile.id = std::move(id);
    profile.name = std::move(name);
    profile.settings.density = density;
    return profile;
}

const WaterRainProfile* FindRainProfile(
    const ProjectDocument& project,
    std::string_view id) {
    return invisible_places::water::FindWaterRainProfileById(
        project.waterRainProfiles,
        id);
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
    "active render setup Rain profile deltas use stable ids and keep setup-only same-name profiles isolated",
    "[render-setup][project-save][rain-profile]") {
    ProjectDocument underlying;
    underlying.waterRainProfiles = {
        MakeRainProfile("underlying-rain", "Shared Rain", 0.20F),
    };
    underlying.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "shared-take",
         .name = "Shared take",
         .assignedRainProfileId = "underlying-rain",
         .assignedRainProfileName = "Shared Rain",
         .baseRainProfileId = "underlying-rain",
         .baseRainProfileName = "Shared Rain"},
    };

    auto previewBaseline = underlying;
    previewBaseline.waterRainProfiles = {
        MakeRainProfile("setup-rain", "Shared Rain", 0.80F),
    };
    auto* previewTake = invisible_places::timing::FindTimingTakeDefinition(
        &previewBaseline.timingTakes,
        "shared-take");
    REQUIRE(previewTake != nullptr);
    previewTake->assignedRainProfileId = "setup-rain";
    previewTake->assignedRainProfileName = "Shared Rain";
    previewTake->baseRainProfileId = "setup-rain";
    previewTake->baseRainProfileName = "Shared Rain";

    auto live = previewBaseline;
    live.waterRainProfiles = {
        MakeRainProfile("authored-rain", "Shared Rain", 0.65F),
    };
    auto* liveTake = invisible_places::timing::FindTimingTakeDefinition(
        &live.timingTakes,
        "shared-take");
    REQUIRE(liveTake != nullptr);
    liveTake->assignedRainProfileId = "authored-rain";
    liveTake->assignedRainProfileName = "Shared Rain";
    liveTake->baseRainProfileId = "authored-rain";
    liveTake->baseRainProfileName = "Shared Rain";

    const auto saved =
        invisible_places::app::MergeRenderSetupProjectForSave(
            underlying,
            previewBaseline,
            std::move(live));

    REQUIRE(FindRainProfile(saved, "underlying-rain") != nullptr);
    CHECK(FindRainProfile(saved, "underlying-rain")->settings.density ==
          Catch::Approx(0.20F));
    REQUIRE(FindRainProfile(saved, "authored-rain") != nullptr);
    CHECK(FindRainProfile(saved, "authored-rain")->settings.density ==
          Catch::Approx(0.65F));
    CHECK(FindRainProfile(saved, "setup-rain") == nullptr);
    REQUIRE(saved.waterRainProfiles.size() == 2U);

    const auto* savedTake = invisible_places::timing::FindTimingTakeDefinition(
        saved.timingTakes,
        "shared-take");
    REQUIRE(savedTake != nullptr);
    CHECK(savedTake->assignedRainProfileId == "authored-rain");
    CHECK(savedTake->baseRainProfileId == "authored-rain");
}

TEST_CASE(
    "render setup Rain reconstruction selects the stable-id owner over same-name shared profiles",
    "[render-setup][project-reconstruction][rain-profile]") {
    WaterSourcesDocument authoredWater;
    auto firstShared =
        MakeRainProfile("rain-shared-first", "Shared Rain", 0.14F);
    firstShared.visual.opacity = 0.21F;
    auto selectedBase =
        MakeRainProfile("rain-shared-selected", "Shared Rain", 0.38F);
    selectedBase.visual.opacity = 0.42F;
    auto selectedOwner = selectedBase;
    selectedOwner.id = "rain-owner-selected";
    selectedOwner.name = "Shared Rain_Selected";
    selectedOwner.objectOverride = true;
    selectedOwner.ownerTimingTakeId = "selected-take";
    selectedOwner.baseProfileId = selectedBase.id;
    selectedOwner.baseProfileName = selectedBase.name;
    selectedOwner.settings.density = 0.86F;
    selectedOwner.settings.activeParticleCount = 12'345U;
    selectedOwner.visual.colour = {0.17F, 0.63F, 0.91F};
    selectedOwner.visual.widthMeters = 0.0047F;
    selectedOwner.visual.opacity = 0.73F;

    auto unreachableOwner = firstShared;
    unreachableOwner.id = "rain-owner-missing";
    unreachableOwner.name = "Shared Rain_Missing";
    unreachableOwner.objectOverride = true;
    unreachableOwner.ownerTimingTakeId = "missing-take";
    unreachableOwner.baseProfileId = firstShared.id;
    unreachableOwner.baseProfileName = firstShared.name;
    authoredWater.rainProfiles = {
        firstShared,
        selectedBase,
        selectedOwner,
        unreachableOwner,
    };
    authoredWater.rainTimingTakeAssignments = {
        {.id = std::string{
             invisible_places::timing::kAuthoredTimingTakeId},
         .name = std::string{
             invisible_places::timing::kAuthoredTimingTakeName},
         .assignedRainProfileId = firstShared.id,
         .assignedRainProfileName = firstShared.name,
         .baseRainProfileId = firstShared.id,
         .baseRainProfileName = firstShared.name},
        {.id = "selected-take",
         .name = "Selected",
         .assignedRainProfileId = selectedOwner.id,
         .assignedRainProfileName = selectedOwner.name,
         .baseRainProfileId = selectedBase.id,
         .baseRainProfileName = selectedBase.name},
        {.id = "missing-take",
         .name = "Missing",
         .assignedRainProfileId = unreachableOwner.id,
         .assignedRainProfileName = unreachableOwner.name,
         .baseRainProfileId = firstShared.id,
         .baseRainProfileName = firstShared.name},
    };
    // This arbitrary compatibility projection must not beat the selected
    // take's owner assignment.
    authoredWater.rainSettings.density = 0.03F;
    authoredWater.rainVisualSettings.opacity = 0.04F;

    ProjectDocument project;
    project.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "selected-take", .name = "Selected"},
    };
    project.waterRainProfiles = {
        MakeRainProfile("live-only", "Live Rain", 0.99F),
    };

    invisible_places::app::RebuildRenderSetupRainProject(
        authoredWater,
        "selected-take",
        &project);

    REQUIRE(project.timingTakes.size() == 2U);
    CHECK(
        invisible_places::timing::FindTimingTakeDefinition(
            project.timingTakes,
            "missing-take") == nullptr);
    CHECK(FindRainProfile(project, "live-only") == nullptr);
    CHECK(FindRainProfile(project, unreachableOwner.id) == nullptr);
    REQUIRE(FindRainProfile(project, firstShared.id) != nullptr);
    REQUIRE(FindRainProfile(project, selectedBase.id) != nullptr);
    CHECK(FindRainProfile(project, firstShared.id)->name == "Shared Rain");
    CHECK(FindRainProfile(project, selectedBase.id)->name ==
          "Shared Rain 2");
    const auto* selectedTake =
        invisible_places::timing::FindTimingTakeDefinition(
            project.timingTakes,
            "selected-take");
    REQUIRE(selectedTake != nullptr);
    CHECK(selectedTake->assignedRainProfileId == selectedOwner.id);
    CHECK(selectedTake->baseRainProfileId == selectedBase.id);
    CHECK(selectedTake->baseRainProfileName == "Shared Rain 2");
    const auto* effective = invisible_places::timing::
        ResolveTimingTakeRainProfile(
            project.waterRainProfiles,
            *selectedTake);
    REQUIRE(effective != nullptr);
    CHECK(effective->id == selectedOwner.id);
    CHECK(effective->settings == selectedOwner.settings);
    CHECK(effective->visual == selectedOwner.visual);
    CHECK(project.waterRainSettings == selectedOwner.settings);
    CHECK(project.waterRainVisualSettings == selectedOwner.visual);

    const auto backgroundSnapshot = invisible_places::app::
        CaptureRenderSetupRainProfileSnapshot(
            authoredWater,
            "selected-take");
    REQUIRE(backgroundSnapshot.has_value());
    CHECK(backgroundSnapshot->id == selectedOwner.id);
    CHECK(backgroundSnapshot->settings == selectedOwner.settings);
    CHECK(backgroundSnapshot->visual == selectedOwner.visual);
    auto* mutableOwner =
        invisible_places::water::FindWaterRainProfileById(
            &authoredWater.rainProfiles,
            selectedOwner.id);
    REQUIRE(mutableOwner != nullptr);
    mutableOwner->settings.density = 0.01F;
    mutableOwner->visual.colour = {1.0F, 0.0F, 0.0F};
    mutableOwner->visual.opacity = 0.02F;
    CHECK(backgroundSnapshot->settings == selectedOwner.settings);
    CHECK(backgroundSnapshot->visual == selectedOwner.visual);
}

TEST_CASE(
    "legacy render setup Rain reconstruction preserves its exact compatibility snapshot",
    "[render-setup][project-reconstruction][rain-profile][legacy]") {
    WaterSourcesDocument authoredWater;
    authoredWater.schemaVersion =
        invisible_places::serialization::
            kWaterRainProfilesSourcesSchemaVersion - 1U;
    authoredWater.rainProfiles.clear();
    authoredWater.rainTimingTakeAssignments.clear();
    authoredWater.rainSettings.enabled = true;
    authoredWater.rainSettings.visualProfileName =
        "Unlisted captured look";
    authoredWater.rainSettings.activeParticleCount = 4'321U;
    authoredWater.rainSettings.seed = 876U;
    authoredWater.rainSettings.density = 0.731F;
    authoredWater.rainSettings.windDirectionX = -0.37F;
    authoredWater.rainSettings.rockImpact.downhillStretch = 2.17F;
    authoredWater.rainVisualSettings.colour = {0.12F, 0.34F, 0.87F};
    authoredWater.rainVisualSettings.widthMeters = 0.0073F;
    authoredWater.rainVisualSettings.streakLengthMeters = 0.287F;
    authoredWater.rainVisualSettings.softness = 0.19F;
    authoredWater.rainVisualSettings.opacity = 0.843F;
    authoredWater.rainVisualSettings.emission = 0.617F;
    authoredWater.rainVisualSettings.minimumScreenPixels = 1.13F;
    authoredWater.rainVisualSettings.maximumScreenPixels = 7.71F;

    ProjectDocument project;
    project.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "legacy-selected", .name = "Legacy selected"},
    };
    project.waterRainProfiles = {
        MakeRainProfile(
            std::string{
                invisible_places::timing::kLegacyWaterRainProfileId},
            "Project Rain",
            0.02F),
    };

    invisible_places::app::RebuildRenderSetupRainProject(
        authoredWater,
        "legacy-selected",
        &project);

    REQUIRE(project.timingTakes.size() == 2U);
    const auto* selectedTake =
        invisible_places::timing::FindTimingTakeDefinition(
            project.timingTakes,
            "legacy-selected");
    REQUIRE(selectedTake != nullptr);
    const auto* effective = invisible_places::timing::
        ResolveTimingTakeRainProfile(
            project.waterRainProfiles,
            *selectedTake);
    REQUIRE(effective != nullptr);
    CHECK(effective->settings == authoredWater.rainSettings);
    CHECK(effective->visual == authoredWater.rainVisualSettings);
    CHECK(project.waterRainSettings == authoredWater.rainSettings);
    CHECK(project.waterRainVisualSettings ==
          authoredWater.rainVisualSettings);
}

TEST_CASE(
    "modern render setup Rain reconstruction isolates compatibility when the selected assignment is unusable",
    "[render-setup][project-reconstruction][rain-profile]") {
    WaterSourcesDocument authoredWater;
    auto firstShared =
        MakeRainProfile("first-shared", "First shared", 0.16F);
    firstShared.visual.opacity = 0.22F;
    auto secondShared =
        MakeRainProfile("second-shared", "Second shared", 0.41F);
    secondShared.visual.opacity = 0.48F;
    authoredWater.rainProfiles = {firstShared, secondShared};
    authoredWater.rainSettings.enabled = true;
    authoredWater.rainSettings.density = 0.79F;
    authoredWater.rainSettings.activeParticleCount = 8'765U;
    authoredWater.rainVisualSettings.colour = {0.81F, 0.27F, 0.43F};
    authoredWater.rainVisualSettings.widthMeters = 0.0061F;
    authoredWater.rainVisualSettings.opacity = 0.68F;

    SECTION("empty selected assignment") {
        authoredWater.rainTimingTakeAssignments.clear();
    }
    SECTION("invalid stable id does not fall through to its name mirror") {
        authoredWater.rainTimingTakeAssignments = {
            {.id = "selected-take",
             .name = "Selected",
             .assignedRainProfileId = "missing-stable-id",
             .assignedRainProfileName = firstShared.name,
             .baseRainProfileId = firstShared.id,
             .baseRainProfileName = firstShared.name},
        };
    }

    ProjectDocument project;
    project.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition(),
        {.id = "selected-take", .name = "Selected"},
    };
    invisible_places::app::RebuildRenderSetupRainProject(
        authoredWater,
        "selected-take",
        &project);

    REQUIRE(project.timingTakes.size() == 2U);
    const auto* selectedTake =
        invisible_places::timing::FindTimingTakeDefinition(
            project.timingTakes,
            "selected-take");
    REQUIRE(selectedTake != nullptr);
    const auto* effective = invisible_places::timing::
        ResolveTimingTakeRainProfile(
            project.waterRainProfiles,
            *selectedTake);
    REQUIRE(effective != nullptr);
    CHECK(effective->objectOverride);
    CHECK(effective->ownerTimingTakeId == "selected-take");
    CHECK(effective->settings == authoredWater.rainSettings);
    CHECK(effective->visual == authoredWater.rainVisualSettings);
    CHECK(project.waterRainSettings == authoredWater.rainSettings);
    CHECK(project.waterRainVisualSettings ==
          authoredWater.rainVisualSettings);
    REQUIRE(FindRainProfile(project, firstShared.id) != nullptr);
    CHECK(FindRainProfile(project, firstShared.id)->settings ==
          firstShared.settings);
    CHECK(FindRainProfile(project, firstShared.id)->visual ==
          firstShared.visual);
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
