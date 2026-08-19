#include "app/PointVisualSelection.hpp"
#include "app/ProjectWorkspace.hpp"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

TEST_CASE(
    "Animation Visual selection preserves an existing edited registry shadow",
    "[animation][point-visual][edit-shadow]") {
    using invisible_places::camera::AnimationPath;
    using invisible_places::camera::AnimationPathKey;
    using invisible_places::camera::AnimationVelocityBlendLinkMetadata;

    AnimationPath saved;
    saved.name = "Linked Loop A";
    saved.durationFrames = 120U;
    saved.selectedTimingTakeId = "timing-saved";
    saved.selectedPointVisualName = "Visual Saved";
    saved.keys = {
        AnimationPathKey{
            .id = "saved-key",
            .cameraPosition = {1.0F, 2.0F, 3.0F},
        },
    };

    auto edited = saved;
    edited.durationFrames = 360U;
    edited.selectedTimingTakeId = "timing-edited";
    edited.keys.front().id = "edited-camera-key";
    edited.keys.front().cameraPosition = {9.0F, 8.0F, 7.0F};
    edited.preferredBlendPartnerFileName = "Linked Loop B.ipanim.json";
    edited.velocityBlendLink = AnimationVelocityBlendLinkMetadata{
        .pairId = "linked-pair-edited",
        .partnerFileName = "Linked Loop B.ipanim.json",
        .timingCycleFrames = 720U,
        .timingWindowStartFrame = 360,
    };
    std::optional<AnimationPath> editedShadow{edited};

    const auto result =
        invisible_places::app::point_visual::ApplyAnimationSelectionEdit(
            &saved,
            &editedShadow,
            false,
            "Visual Rain");

    CHECK(result.promotedEditedShadow);
    CHECK(result.selectionChanged);
    CHECK(saved.selectedPointVisualName == "Visual Rain");
    CHECK(saved.durationFrames == 360U);
    CHECK(saved.selectedTimingTakeId == "timing-edited");
    REQUIRE(saved.keys.size() == 1U);
    CHECK(saved.keys.front().id == "edited-camera-key");
    const std::array<float, 3> expectedEditedCamera{9.0F, 8.0F, 7.0F};
    CHECK(saved.keys.front().cameraPosition == expectedEditedCamera);
    CHECK(saved.preferredBlendPartnerFileName ==
          "Linked Loop B.ipanim.json");
    REQUIRE(saved.velocityBlendLink.has_value());
    CHECK(saved.velocityBlendLink->pairId == "linked-pair-edited");
    CHECK(saved.velocityBlendLink->timingCycleFrames == 720U);
    CHECK(saved.velocityBlendLink->timingWindowStartFrame == 360);

    REQUIRE(editedShadow.has_value());
    CHECK(editedShadow->selectedPointVisualName == "Visual Rain");
    CHECK(editedShadow->durationFrames == 360U);
    CHECK(editedShadow->selectedTimingTakeId == "timing-edited");
    CHECK(editedShadow->keys.front().id == "edited-camera-key");
    REQUIRE(editedShadow->velocityBlendLink.has_value());
    CHECK(editedShadow->velocityBlendLink->pairId ==
          "linked-pair-edited");
}

TEST_CASE(
    "Missing Timing repair promotes the complete edited animation shadow",
    "[animation][point-visual][edit-shadow][timing]") {
    using invisible_places::camera::AnimationPath;
    using invisible_places::camera::AnimationPathKey;
    using invisible_places::camera::AnimationVelocityBlendLinkMetadata;

    AnimationPath saved;
    saved.name = "Saved comparison";
    saved.durationFrames = 120U;
    saved.selectedTimingTakeId = "deleted-saved-take";
    saved.selectedPointVisualName = "Saved Visual";
    saved.keys = {AnimationPathKey{.id = "saved-key"}};

    auto edited = saved;
    edited.durationFrames = 480U;
    edited.selectedTimingTakeId = "edited-take";
    edited.selectedPointVisualName = "Edited Visual";
    edited.keys.front().id = "edited-camera-key";
    edited.keys.front().cameraPosition = {4.0F, 5.0F, 6.0F};
    edited.preferredBlendPartnerFileName = "Partner.ipanim.json";
    edited.velocityBlendLink = AnimationVelocityBlendLinkMetadata{
        .pairId = "edited-pair",
        .partnerFileName = "Partner.ipanim.json",
        .timingCycleFrames = 960U,
        .timingWindowStartFrame = 480,
    };

    const auto result =
        invisible_places::app::point_visual::ResolveAnimationLoadAuthority(
            saved,
            std::optional<AnimationPath>{edited},
            false,
            true);

    CHECK(result.usesEditedPath);
    CHECK(result.promotedEditedShadow);
    CHECK(result.path.durationFrames == 480U);
    CHECK(result.path.selectedTimingTakeId == "edited-take");
    CHECK(result.path.selectedPointVisualName == "Edited Visual");
    REQUIRE(result.path.keys.size() == 1U);
    CHECK(result.path.keys.front().id == "edited-camera-key");
    const std::array<float, 3> expectedCamera{4.0F, 5.0F, 6.0F};
    CHECK(result.path.keys.front().cameraPosition == expectedCamera);
    CHECK(result.path.preferredBlendPartnerFileName ==
          "Partner.ipanim.json");
    REQUIRE(result.path.velocityBlendLink.has_value());
    CHECK(result.path.velocityBlendLink->pairId == "edited-pair");
    CHECK(result.path.velocityBlendLink->timingCycleFrames == 960U);
    CHECK(result.path.velocityBlendLink->timingWindowStartFrame == 480);

    const auto noRepair =
        invisible_places::app::point_visual::ResolveAnimationLoadAuthority(
            saved,
            std::optional<AnimationPath>{edited},
            false,
            false);
    CHECK_FALSE(noRepair.usesEditedPath);
    CHECK_FALSE(noRepair.promotedEditedShadow);
    CHECK(noRepair.path.keys.front().id == "saved-key");
    CHECK(noRepair.path.selectedPointVisualName == "Saved Visual");
}

TEST_CASE(
    "Animation save keeps the authoritative edited or render-setup Visual",
    "[animation][point-visual][save-authority]") {
    using invisible_places::camera::AnimationPath;

    AnimationPath prepared;
    prepared.selectedPointVisualName = "Edited Visual";

    CHECK_FALSE(
        invisible_places::app::point_visual::ApplyAnimationSelectionForSave(
            &prepared,
            "Saved viewport Visual",
            true,
            false));
    CHECK(prepared.selectedPointVisualName == "Edited Visual");

    CHECK(
        invisible_places::app::point_visual::ApplyAnimationSelectionForSave(
            &prepared,
            "  Active edited Visual  ",
            true,
            true));
    CHECK(prepared.selectedPointVisualName == "Active edited Visual");

    AnimationPath renderSetupAuthority;
    renderSetupAuthority.selectedPointVisualName =
        "Underlying edited Visual";
    prepared.selectedPointVisualName = "Temporary setup Visual";
    CHECK(
        invisible_places::app::point_visual::ApplyAnimationSelectionForSave(
            &prepared,
            "Temporary setup Visual",
            true,
            true,
            &renderSetupAuthority));
    CHECK(prepared.selectedPointVisualName ==
          "Underlying edited Visual");
}

TEST_CASE(
    "Animation Visual durability requires a coupled project save",
    "[animation][point-visual][save-authority]") {
    const std::vector<std::string> durableNames{
        "Unnamed",
        "Rock Study",
        "Scene Rain",
    };

    CHECK_FALSE(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave("", durableNames));
    CHECK_FALSE(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave(
                "  Rock Study  ",
                durableNames));
    CHECK_FALSE(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave(
                "Scene Rain",
                durableNames));
    CHECK(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave(
                "New unsaved Visual",
                durableNames));
}

TEST_CASE(
    "Direct animation writes reject a Visual absent from the saved project",
    "[animation][point-visual][save-authority][direct-write]") {
    using invisible_places::camera::AnimationPath;
    using invisible_places::serialization::ProjectDocument;
    using invisible_places::serialization::ProjectLayerDocument;

    ProjectDocument trackedProject;
    trackedProject.pointVisuals = {
        ProjectLayerDocument::PointVisual{.name = "Durable Visual"},
    };
    const auto trackedNames =
        invisible_places::app::point_visual::ProjectVisualNames(
            trackedProject);

    AnimationPath directWrite;
    directWrite.selectedPointVisualName = "Runtime-only Visual";
    CHECK(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave(
                directWrite.selectedPointVisualName,
                trackedNames));

    directWrite.selectedPointVisualName = "Durable Visual";
    CHECK_FALSE(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave(
                directWrite.selectedPointVisualName,
                trackedNames));
}

TEST_CASE(
    "Remote Visual deletion invalidates staged animations after project merge",
    "[animation][point-visual][workspace][conflict][merge]") {
    using invisible_places::app::workspace::MergeJsonDocuments;
    using invisible_places::camera::AnimationPath;
    using invisible_places::serialization::ProjectDocument;
    using invisible_places::serialization::ProjectLayerDocument;

    const nlohmann::json baseline = {
        {"schema_version", 78U},
        {"project_name", "Baseline"},
        {"point_visuals",
         nlohmann::json::array({
             {{"name", "Fog"}},
             {{"name", "Stone"}},
         })},
    };
    auto local = baseline;
    local["project_name"] = "Local camera edit";
    auto remote = baseline;
    remote["point_visuals"].erase(
        remote["point_visuals"].begin());

    const auto merge = MergeJsonDocuments(baseline, local, remote);
    REQUIRE(merge.conflicts.empty());
    REQUIRE(merge.merged.at("point_visuals").size() == 1U);

    // SaveSelectedChanges validates the deserialized post-merge document.
    // Build that authority from the actual merged definitions here so a
    // conflict-free remote deletion cannot leave a dangling animation name.
    ProjectDocument mergedProject;
    mergedProject.pointVisuals.clear();
    for (const auto& visual : merge.merged.at("point_visuals")) {
        mergedProject.pointVisuals.push_back(
            ProjectLayerDocument::PointVisual{
                .name = visual.at("name").get<std::string>(),
            });
    }
    const auto mergedNames =
        invisible_places::app::point_visual::ProjectVisualNames(
            mergedProject);

    AnimationPath stagedAnimation;
    stagedAnimation.selectedPointVisualName = "Fog";
    CHECK(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave(
                stagedAnimation.selectedPointVisualName,
                mergedNames));

    stagedAnimation.selectedPointVisualName = "Stone";
    CHECK_FALSE(
        invisible_places::app::point_visual::
            AnimationVisualRequiresProjectSave(
                stagedAnimation.selectedPointVisualName,
                mergedNames));
}
