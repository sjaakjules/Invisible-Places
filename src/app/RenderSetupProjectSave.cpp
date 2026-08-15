#include "app/RenderSetupProjectSave.hpp"

#include "serialization/ProjectDocumentJson.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstddef>
#include <span>
#include <utility>

namespace invisible_places::app {
namespace {

using invisible_places::serialization::ProjectDocument;
using invisible_places::timing::TimingTakeDefinition;
using invisible_places::timing::TimingTakeSceneState;
using invisible_places::water::WaterKeyedSettingsProfile;

bool WaterKeyedSettingsProfilesEqual(
    const WaterKeyedSettingsProfile& leftInput,
    const WaterKeyedSettingsProfile& rightInput) {
    const auto left = invisible_places::water::
        SanitizeWaterKeyedSettingsProfile(leftInput);
    const auto right = invisible_places::water::
        SanitizeWaterKeyedSettingsProfile(rightInput);
    if (left.name != right.name ||
        left.baseProfileName != right.baseProfileName ||
        left.ownerObjectName != right.ownerObjectName ||
        left.sourceProfileName != right.sourceProfileName ||
        left.ownerObjectId != right.ownerObjectId ||
        left.featureKind != right.featureKind ||
        left.edited != right.edited ||
        left.nativeLengthFraction != right.nativeLengthFraction ||
        left.settings.size() != right.settings.size()) {
        return false;
    }
    for (std::size_t index = 0U; index < left.settings.size(); ++index) {
        if (!invisible_places::water::
                WaterKeyedSettingTrackProfileEqual(
                    left.settings[index],
                    right.settings[index])) {
            return false;
        }
    }
    return true;
}

void EraseWaterKeyedSettingsProfile(
    std::vector<WaterKeyedSettingsProfile>* profiles,
    const WaterKeyedSettingsProfile& profile) {
    if (profiles == nullptr) {
        return;
    }
    const auto index = invisible_places::water::
        FindWaterKeyedSettingsProfileIndex(
            *profiles,
            profile.featureKind,
            profile.name);
    if (index.has_value()) {
        profiles->erase(
            profiles->begin() +
            static_cast<std::ptrdiff_t>(index.value()));
    }
}

void UpsertWaterKeyedSettingsProfile(
    std::vector<WaterKeyedSettingsProfile>* profiles,
    const WaterKeyedSettingsProfile& profile) {
    if (profiles == nullptr) {
        return;
    }
    const auto index = invisible_places::water::
        FindWaterKeyedSettingsProfileIndex(
            *profiles,
            profile.featureKind,
            profile.name);
    if (index.has_value()) {
        (*profiles)[index.value()] = profile;
    } else {
        profiles->push_back(profile);
    }
}

void ApplyWaterKeyedSettingsProfileDelta(
    std::span<const WaterKeyedSettingsProfile> baseline,
    std::span<const WaterKeyedSettingsProfile> live,
    std::vector<WaterKeyedSettingsProfile>* destination) {
    for (const auto& profile : baseline) {
        if (!invisible_places::water::
                 FindWaterKeyedSettingsProfileIndex(
                     live,
                     profile.featureKind,
                     profile.name)
                 .has_value()) {
            EraseWaterKeyedSettingsProfile(destination, profile);
        }
    }
    for (const auto& profile : live) {
        const auto baselineIndex = invisible_places::water::
            FindWaterKeyedSettingsProfileIndex(
                baseline,
                profile.featureKind,
                profile.name);
        if (!baselineIndex.has_value() ||
            !WaterKeyedSettingsProfilesEqual(
                baseline[baselineIndex.value()],
                profile)) {
            UpsertWaterKeyedSettingsProfile(destination, profile);
        }
    }
}

bool TimingTakeDefinitionsEqual(
    const TimingTakeDefinition& left,
    const TimingTakeDefinition& right) {
    const auto sanitizedLeft =
        invisible_places::timing::SanitizeTimingTakeDefinition(left);
    const auto sanitizedRight =
        invisible_places::timing::SanitizeTimingTakeDefinition(right);
    return sanitizedLeft.id == sanitizedRight.id &&
           sanitizedLeft.name == sanitizedRight.name;
}

void ApplyTimingTakeDefinitionDelta(
    std::span<const TimingTakeDefinition> baseline,
    std::span<const TimingTakeDefinition> live,
    std::vector<TimingTakeDefinition>* destination) {
    if (destination == nullptr) {
        return;
    }
    for (const auto& take : baseline) {
        if (invisible_places::timing::FindTimingTakeDefinition(
                live,
                take.id) != nullptr) {
            continue;
        }
        const auto normalizedId =
            invisible_places::timing::NormalizeTimingTakeId(take.id);
        std::erase_if(
            *destination,
            [&](const TimingTakeDefinition& candidate) {
                return invisible_places::timing::NormalizeTimingTakeId(
                           candidate.id) == normalizedId;
            });
    }
    for (const auto& take : live) {
        const auto* baselineTake =
            invisible_places::timing::FindTimingTakeDefinition(
                baseline,
                take.id);
        if (baselineTake != nullptr &&
            TimingTakeDefinitionsEqual(*baselineTake, take)) {
            continue;
        }
        if (auto* existing =
                invisible_places::timing::FindTimingTakeDefinition(
                    destination,
                    take.id);
            existing != nullptr) {
            *existing = take;
        } else {
            destination->push_back(take);
        }
    }
}

bool TimingTakeSceneStatesEqual(
    const TimingTakeSceneState& left,
    const TimingTakeSceneState& right) {
    return invisible_places::serialization::
               TimingTakeSceneStateToJson(left) ==
           invisible_places::serialization::
               TimingTakeSceneStateToJson(right);
}

void ApplyTimingTakeSceneStateDelta(
    std::span<const TimingTakeSceneState> baseline,
    std::span<const TimingTakeSceneState> live,
    std::vector<TimingTakeSceneState>* destination) {
    if (destination == nullptr) {
        return;
    }
    for (const auto& state : baseline) {
        if (invisible_places::timing::FindTimingTakeSceneState(
                live,
                state.takeId,
                state.sceneGroupName) != nullptr) {
            continue;
        }
        const auto normalizedId =
            invisible_places::timing::NormalizeTimingTakeId(state.takeId);
        std::erase_if(
            *destination,
            [&](const TimingTakeSceneState& candidate) {
                return invisible_places::timing::NormalizeTimingTakeId(
                           candidate.takeId) == normalizedId &&
                       candidate.sceneGroupName == state.sceneGroupName;
            });
    }
    for (const auto& state : live) {
        const auto* baselineState =
            invisible_places::timing::FindTimingTakeSceneState(
                baseline,
                state.takeId,
                state.sceneGroupName);
        if (baselineState != nullptr &&
            TimingTakeSceneStatesEqual(*baselineState, state)) {
            continue;
        }
        if (auto* existing =
                invisible_places::timing::FindTimingTakeSceneState(
                    destination,
                    state.takeId,
                    state.sceneGroupName);
            existing != nullptr) {
            *existing = state;
        } else {
            destination->push_back(state);
        }
    }
}

}  // namespace

ProjectDocument MergeRenderSetupProjectForSave(
    const ProjectDocument& underlyingProject,
    const ProjectDocument& previewBaseline,
    ProjectDocument liveProject) {
    auto document = underlyingProject;

    // Apply only authoring deltas made after the setup became active.  An
    // unchanged setup package or Timing Take remains part of the preview and
    // cannot replace (or be added to) the underlying project by accident.
    ApplyWaterKeyedSettingsProfileDelta(
        previewBaseline.waterKeyedSettingsProfiles,
        liveProject.waterKeyedSettingsProfiles,
        &document.waterKeyedSettingsProfiles);
    ApplyTimingTakeDefinitionDelta(
        previewBaseline.timingTakes,
        liveProject.timingTakes,
        &document.timingTakes);
    ApplyTimingTakeSceneStateDelta(
        previewBaseline.timingTakeStates,
        liveProject.timingTakeStates,
        &document.timingTakeStates);
    if (liveProject.timingTakeSequence !=
        previewBaseline.timingTakeSequence) {
        document.timingTakeSequence = liveProject.timingTakeSequence;
    }

    document.cameraState = liveProject.cameraState;
    document.orbitControlMode = liveProject.orbitControlMode;
    document.cameraShots = std::move(liveProject.cameraShots);
    document.cameraPathShotIndices =
        std::move(liveProject.cameraPathShotIndices);
    document.cameraPathDurationFrames =
        liveProject.cameraPathDurationFrames;
    document.liveViewWindowWidth = liveProject.liveViewWindowWidth;
    document.liveViewWindowHeight = liveProject.liveViewWindowHeight;
    document.lockLiveViewWindowSize =
        liveProject.lockLiveViewWindowSize;
    document.lastAnimationPath = liveProject.lastAnimationPath;
    document.activeAnimationPath = liveProject.activeAnimationPath;
    document.activeAnimationPosition =
        liveProject.activeAnimationPosition;
    document.savedAnimations = std::move(liveProject.savedAnimations);
    document.hasSavedAnimationRegistry =
        liveProject.hasSavedAnimationRegistry;
    return document;
}

}  // namespace invisible_places::app
