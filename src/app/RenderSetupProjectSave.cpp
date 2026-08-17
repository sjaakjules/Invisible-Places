#include "app/RenderSetupProjectSave.hpp"

#include "serialization/ProjectDocumentJson.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>

namespace invisible_places::app {
namespace {

using invisible_places::serialization::ProjectDocument;
using invisible_places::timing::TimingTakeDefinition;
using invisible_places::timing::TimingTakeSceneState;
using invisible_places::water::WaterKeyedSettingsProfile;
using invisible_places::water::WaterRainProfile;

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

std::optional<std::size_t> FindWaterRainProfileIndex(
    std::span<const WaterRainProfile> profiles,
    const WaterRainProfile& profile) {
    if (!profile.id.empty()) {
        if (const auto* found =
                invisible_places::water::FindWaterRainProfileById(
                    profiles,
                    profile.id);
            found != nullptr) {
            return static_cast<std::size_t>(found - profiles.data());
        }
        // Stable ids are authoritative. A reused display name must not merge
        // two different profiles; name fallback is legacy empty-id only.
        return std::nullopt;
    }
    if (const auto* found =
            invisible_places::water::FindWaterRainProfileByName(
                profiles,
                profile.name);
        found != nullptr) {
        return static_cast<std::size_t>(found - profiles.data());
    }
    return std::nullopt;
}

void ApplyWaterRainProfileDelta(
    std::span<const WaterRainProfile> baseline,
    std::span<const WaterRainProfile> live,
    std::vector<WaterRainProfile>* destination) {
    if (destination == nullptr) {
        return;
    }
    for (const auto& profile : baseline) {
        if (FindWaterRainProfileIndex(live, profile).has_value()) {
            continue;
        }
        if (const auto index =
                FindWaterRainProfileIndex(*destination, profile);
            index.has_value()) {
            destination->erase(
                destination->begin() +
                static_cast<std::ptrdiff_t>(index.value()));
        }
    }
    for (const auto& profile : live) {
        const auto sanitized =
            invisible_places::water::SanitizeWaterRainProfile(profile);
        const auto baselineIndex = FindWaterRainProfileIndex(
            baseline,
            sanitized);
        if (baselineIndex.has_value() &&
            invisible_places::water::SanitizeWaterRainProfile(
                baseline[baselineIndex.value()]) == sanitized) {
            continue;
        }
        if (const auto destinationIndex = FindWaterRainProfileIndex(
                *destination,
                sanitized);
            destinationIndex.has_value()) {
            (*destination)[destinationIndex.value()] = sanitized;
        } else {
            destination->push_back(sanitized);
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
           sanitizedLeft.name == sanitizedRight.name &&
           sanitizedLeft.assignedRainProfileId ==
               sanitizedRight.assignedRainProfileId &&
           sanitizedLeft.assignedRainProfileName ==
               sanitizedRight.assignedRainProfileName &&
           sanitizedLeft.baseRainProfileId ==
               sanitizedRight.baseRainProfileId &&
           sanitizedLeft.baseRainProfileName ==
               sanitizedRight.baseRainProfileName;
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

void RebuildRenderSetupRainProject(
    const invisible_places::serialization::WaterSourcesDocument&
        authoredWater,
    std::string_view selectedTimingTakeId,
    ProjectDocument* project) {
    if (project == nullptr) {
        return;
    }

    const auto normalizedTakeId =
        invisible_places::timing::NormalizeTimingTakeId(
            selectedTimingTakeId);
    const auto* capturedSelectedAssignment =
        invisible_places::timing::FindTimingTakeDefinition(
            authoredWater.rainTimingTakeAssignments,
            normalizedTakeId);
    const invisible_places::water::WaterRainProfile*
        capturedSelectedProfile = nullptr;
    if (capturedSelectedAssignment != nullptr) {
        if (!capturedSelectedAssignment->assignedRainProfileId.empty()) {
            capturedSelectedProfile =
                invisible_places::water::FindWaterRainProfileById(
                    authoredWater.rainProfiles,
                    capturedSelectedAssignment->assignedRainProfileId);
        } else if (!capturedSelectedAssignment
                        ->assignedRainProfileName.empty()) {
            capturedSelectedProfile =
                invisible_places::water::FindWaterRainProfileByName(
                    authoredWater.rainProfiles,
                    capturedSelectedAssignment->assignedRainProfileName);
        }
    }

    auto importedProfiles = authoredWater.rainProfiles;
    auto importedAssignments =
        authoredWater.rainTimingTakeAssignments;
    (void)invisible_places::timing::EnsureLegacyWaterRainProfile(
        &importedProfiles,
        &importedAssignments,
        authoredWater.rainSettings,
        authoredWater.rainVisualSettings);

    // A setup is an isolated authored-water snapshot, so the captured library
    // replaces the arbitrary live project's library. The merge helper then
    // applies only assignment mirrors whose take ids already exist in the
    // synthetic setup project.
    project->waterRainProfiles.clear();
    const bool legacyCompatibility =
        authoredWater.schemaVersion <
        invisible_places::serialization::
            kWaterRainProfilesSourcesSchemaVersion;
    const std::string legacyCompatibilityTakeId =
        legacyCompatibility ? normalizedTakeId : std::string{};
    (void)invisible_places::timing::
        MergeImportedTimingTakeRainProfiles(
            &project->waterRainProfiles,
            &project->timingTakes,
            importedProfiles,
            importedAssignments,
            legacyCompatibilityTakeId);

    auto* selectedTake =
        invisible_places::timing::FindTimingTakeDefinition(
            &project->timingTakes,
            normalizedTakeId);
    const auto* appliedSelectedProfile =
        selectedTake == nullptr
            ? nullptr
            : invisible_places::water::FindWaterRainProfileById(
                  project->waterRainProfiles,
                  selectedTake->assignedRainProfileId);
    bool selectedAssignmentApplied = false;
    if (legacyCompatibility) {
        selectedAssignmentApplied =
            appliedSelectedProfile != nullptr &&
            appliedSelectedProfile->settings ==
                authoredWater.rainSettings &&
            appliedSelectedProfile->visual ==
                authoredWater.rainVisualSettings;
    } else if (capturedSelectedProfile != nullptr) {
        const auto* normalizedSelectedAssignment =
            invisible_places::timing::FindTimingTakeDefinition(
                importedAssignments,
                normalizedTakeId);
        selectedAssignmentApplied =
            normalizedSelectedAssignment != nullptr &&
            appliedSelectedProfile != nullptr &&
            appliedSelectedProfile->id ==
                normalizedSelectedAssignment->assignedRainProfileId;
    }
    if (!selectedAssignmentApplied && selectedTake != nullptr) {
        // The compatibility pair is the exact effective selected-take
        // snapshot captured by current writers. Preserve it as an isolated
        // owner when a schema31 assignment is absent or cannot be resolved;
        // never silently substitute an arbitrary first shared profile.
        (void)invisible_places::timing::
            UpsertTimingTakeRainOwnerProfile(
                &project->waterRainProfiles,
                selectedTake,
                authoredWater.rainSettings,
                authoredWater.rainVisualSettings);
    }
    (void)invisible_places::timing::EnsureLegacyWaterRainProfile(
        &project->waterRainProfiles,
        &project->timingTakes,
        authoredWater.rainSettings,
        authoredWater.rainVisualSettings);

    project->waterRainSettings = authoredWater.rainSettings;
    project->waterRainVisualSettings =
        authoredWater.rainVisualSettings;
    if (const auto* take =
            invisible_places::timing::FindTimingTakeDefinition(
                project->timingTakes,
                normalizedTakeId);
        take != nullptr) {
        if (const auto* effective = invisible_places::timing::
                ResolveTimingTakeRainProfile(
                    project->waterRainProfiles,
                    *take);
            effective != nullptr) {
            project->waterRainSettings = effective->settings;
            project->waterRainVisualSettings = effective->visual;
        }
    }
}

std::optional<invisible_places::water::WaterRainProfile>
CaptureRenderSetupRainProfileSnapshot(
    const invisible_places::serialization::WaterSourcesDocument&
        authoredWater,
    std::string_view selectedTimingTakeId) {
    ProjectDocument project;
    project.timingTakes = {
        invisible_places::timing::AuthoredTimingTakeDefinition()};
    const auto normalizedTakeId =
        invisible_places::timing::NormalizeTimingTakeId(
            selectedTimingTakeId);
    if (normalizedTakeId !=
        invisible_places::timing::kAuthoredTimingTakeId) {
        project.timingTakes.push_back({
            .id = normalizedTakeId,
            .name = normalizedTakeId,
        });
    }
    RebuildRenderSetupRainProject(
        authoredWater,
        normalizedTakeId,
        &project);
    return invisible_places::timing::
        CaptureTimingTakeRainProfileSnapshot(
            project.waterRainProfiles,
            project.timingTakes,
            normalizedTakeId);
}

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
    ApplyWaterRainProfileDelta(
        previewBaseline.waterRainProfiles,
        liveProject.waterRainProfiles,
        &document.waterRainProfiles);
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
