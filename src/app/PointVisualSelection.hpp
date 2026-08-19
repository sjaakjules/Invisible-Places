#pragma once

#include "camera/AnimationPath.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "serialization/ProjectDocument.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::app::point_visual {

using PointCloudStyleState = invisible_places::renderer::pointcloud::PointCloudStyleState;

inline constexpr std::string_view kDefaultName = "Unnamed";
inline constexpr std::string_view kPresetSuffix = "_preset";
inline constexpr std::string_view kEditedSuffix = "_edited";
inline constexpr std::string_view kLegacyEditedSuffix = "_Edited";

struct VisualState {
    std::string name = std::string{kDefaultName};
    PointCloudStyleState style{};
};

struct AnimationSelectionEditResult {
    bool selectionChanged = false;
    bool promotedEditedShadow = false;
};

struct AnimationLoadAuthorityResult {
    invisible_places::camera::AnimationPath path{};
    bool usesEditedPath = false;
    bool promotedEditedShadow = false;
};

[[nodiscard]] std::string NormalizeName(std::string_view name);
[[nodiscard]] bool IsPresetName(std::string_view name);
[[nodiscard]] bool IsEditedName(std::string_view name);
[[nodiscard]] std::string BaseName(std::string_view name);
[[nodiscard]] std::string PresetName(std::string_view baseName);
[[nodiscard]] std::string EditedName(std::string_view baseName);
[[nodiscard]] std::optional<std::size_t> FindIndex(
    const std::vector<VisualState>& visuals,
    std::string_view name);

void Upsert(
    std::vector<VisualState>* visuals,
    std::string_view name,
    const PointCloudStyleState& style);
void Remove(std::vector<VisualState>* visuals, std::string_view name);
void SyncNameBuffer(std::string* nameBuffer, std::string_view selectedName);
void Ensure(
    std::vector<VisualState>* visuals,
    std::string* selectedName,
    std::string* nameBuffer,
    const PointCloudStyleState& fallbackStyle);
[[nodiscard]] bool Select(
    std::vector<VisualState>* visuals,
    std::string* selectedName,
    std::string* nameBuffer,
    PointCloudStyleState* activeStyle,
    std::string_view name,
    const PointCloudStyleState& fallbackStyle);

// Applies a Visual selection to the authoritative current animation edit.
// When the UI is temporarily displaying the saved path but an edited registry
// shadow already exists, that complete shadow is promoted before this one
// field is changed so camera, timing, and linked-loop edits are not lost.
[[nodiscard]] AnimationSelectionEditResult ApplyAnimationSelectionEdit(
    invisible_places::camera::AnimationPath* currentPath,
    std::optional<invisible_places::camera::AnimationPath>* editedShadow,
    bool currentPathUsesEdited,
    std::string_view visualName);

// A missing dependency on the requested saved path is itself an edit. If a
// richer edited shadow already exists, promote it before repairing that field
// so the saved baseline cannot replace unrelated camera/timing/link work.
[[nodiscard]] AnimationLoadAuthorityResult ResolveAnimationLoadAuthority(
    const invisible_places::camera::AnimationPath& requestedPath,
    const std::optional<invisible_places::camera::AnimationPath>& editedShadow,
    bool requestedEditedPath,
    bool requestedPathNeedsRepair);

// Chooses the Visual field for one prepared animation without replacing the
// authoritative edited shadow merely because the UI is comparing Saved.
// A render-setup override supplies its corresponding underlying authority.
[[nodiscard]] bool ApplyAnimationSelectionForSave(
    invisible_places::camera::AnimationPath* preparedPath,
    std::string_view liveVisualName,
    bool sourceHasEditedVersion,
    bool currentPathUsesEdited,
    const invisible_places::camera::AnimationPath*
        renderSetupUnderlyingAuthority = nullptr);

// Animation files persist a Visual name, not its style. Referencing a name
// absent from the tracked project baseline therefore requires the project to
// participate in the same save transaction.
[[nodiscard]] bool AnimationVisualRequiresProjectSave(
    std::string_view animationVisualName,
    std::span<const std::string> durableProjectVisualNames);

// The deserialized project document is the authority after a three-way
// merge: legacy schemas have already migrated and remote deletions have
// already been applied. Return every Visual name an animation may reference.
[[nodiscard]] std::vector<std::string> ProjectVisualNames(
    const invisible_places::serialization::ProjectDocument& project);

}  // namespace invisible_places::app::point_visual
