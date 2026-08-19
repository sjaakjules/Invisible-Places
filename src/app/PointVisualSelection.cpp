#include "app/PointVisualSelection.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

namespace invisible_places::app::point_visual {

namespace {

std::string TrimText(std::string_view value) {
    std::size_t begin = 0;
    while (begin < value.size() &&
           std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
        ++begin;
    }

    std::size_t end = value.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1U])) != 0) {
        --end;
    }

    return std::string{value.substr(begin, end - begin)};
}

bool EndsWith(std::string_view value, std::string_view suffix) {
    return value.size() >= suffix.size() &&
           value.substr(value.size() - suffix.size()) == suffix;
}

}  // namespace

std::string NormalizeName(std::string_view name) {
    auto trimmed = TrimText(name);
    if (EndsWith(trimmed, kLegacyEditedSuffix)) {
        trimmed.replace(
            trimmed.size() - kLegacyEditedSuffix.size(),
            kLegacyEditedSuffix.size(),
            kEditedSuffix);
    }
    return trimmed.empty() ? std::string{kDefaultName} : trimmed;
}

bool IsPresetName(std::string_view name) {
    const auto normalized = NormalizeName(name);
    return EndsWith(normalized, kPresetSuffix);
}

bool IsEditedName(std::string_view name) {
    const auto normalized = NormalizeName(name);
    return EndsWith(normalized, kEditedSuffix);
}

std::string BaseName(std::string_view name) {
    auto normalized = NormalizeName(name);
    if (EndsWith(normalized, kEditedSuffix)) {
        normalized.erase(normalized.size() - kEditedSuffix.size());
    }
    if (EndsWith(normalized, kPresetSuffix)) {
        normalized.erase(normalized.size() - kPresetSuffix.size());
    }
    return NormalizeName(normalized);
}

std::string PresetName(std::string_view baseName) {
    return BaseName(baseName) + std::string{kPresetSuffix};
}

std::string EditedName(std::string_view baseName) {
    return BaseName(baseName) + std::string{kEditedSuffix};
}

std::optional<std::size_t> FindIndex(
    const std::vector<VisualState>& visuals,
    std::string_view name) {
    const auto normalized = NormalizeName(name);
    for (std::size_t index = 0; index < visuals.size(); ++index) {
        if (NormalizeName(visuals[index].name) == normalized) {
            return index;
        }
    }
    return std::nullopt;
}

void Upsert(
    std::vector<VisualState>* visuals,
    std::string_view name,
    const PointCloudStyleState& style) {
    if (visuals == nullptr) {
        return;
    }

    const auto normalized = NormalizeName(name);
    if (const auto existingIndex = FindIndex(*visuals, normalized); existingIndex.has_value()) {
        (*visuals)[existingIndex.value()].name = normalized;
        (*visuals)[existingIndex.value()].style = style;
        return;
    }

    visuals->push_back({.name = normalized, .style = style});
}

void Remove(std::vector<VisualState>* visuals, std::string_view name) {
    if (visuals == nullptr) {
        return;
    }

    const auto normalized = NormalizeName(name);
    visuals->erase(
        std::remove_if(
            visuals->begin(),
            visuals->end(),
            [&normalized](const VisualState& visual) {
                return NormalizeName(visual.name) == normalized;
            }),
        visuals->end());
}

void SyncNameBuffer(std::string* nameBuffer, std::string_view selectedName) {
    if (nameBuffer == nullptr) {
        return;
    }
    *nameBuffer = BaseName(selectedName);
}

void Ensure(
    std::vector<VisualState>* visuals,
    std::string* selectedName,
    std::string* nameBuffer,
    const PointCloudStyleState& fallbackStyle) {
    if (visuals == nullptr || selectedName == nullptr || nameBuffer == nullptr) {
        return;
    }

    if (visuals->empty()) {
        visuals->push_back({.name = std::string{kDefaultName}, .style = fallbackStyle});
    }

    for (auto& visual : *visuals) {
        visual.name = NormalizeName(visual.name);
    }

    *selectedName = NormalizeName(*selectedName);
    if (!FindIndex(*visuals, *selectedName).has_value()) {
        *selectedName = visuals->front().name;
    }
    if (nameBuffer->empty()) {
        SyncNameBuffer(nameBuffer, *selectedName);
    }
}

bool Select(
    std::vector<VisualState>* visuals,
    std::string* selectedName,
    std::string* nameBuffer,
    PointCloudStyleState* activeStyle,
    std::string_view name,
    const PointCloudStyleState& fallbackStyle) {
    if (visuals == nullptr || selectedName == nullptr || nameBuffer == nullptr || activeStyle == nullptr) {
        return false;
    }

    const std::string requestedName = NormalizeName(name);
    Ensure(visuals, selectedName, nameBuffer, fallbackStyle);
    const auto index = FindIndex(*visuals, requestedName);
    if (!index.has_value()) {
        return false;
    }

    *selectedName = (*visuals)[index.value()].name;
    *activeStyle = (*visuals)[index.value()].style;
    SyncNameBuffer(nameBuffer, *selectedName);
    return true;
}

AnimationSelectionEditResult ApplyAnimationSelectionEdit(
    invisible_places::camera::AnimationPath* currentPath,
    std::optional<invisible_places::camera::AnimationPath>* editedShadow,
    bool currentPathUsesEdited,
    std::string_view visualName) {
    AnimationSelectionEditResult result;
    if (currentPath == nullptr || visualName.empty()) {
        return result;
    }

    if (!currentPathUsesEdited && editedShadow != nullptr &&
        editedShadow->has_value()) {
        *currentPath = editedShadow->value();
        result.promotedEditedShadow = true;
    }

    const auto normalized = NormalizeName(visualName);
    if (currentPath->selectedPointVisualName == normalized) {
        return result;
    }

    currentPath->selectedPointVisualName = normalized;
    result.selectionChanged = true;
    if (editedShadow != nullptr) {
        *editedShadow = *currentPath;
    }
    return result;
}

AnimationLoadAuthorityResult ResolveAnimationLoadAuthority(
    const invisible_places::camera::AnimationPath& requestedPath,
    const std::optional<invisible_places::camera::AnimationPath>& editedShadow,
    bool requestedEditedPath,
    bool requestedPathNeedsRepair) {
    AnimationLoadAuthorityResult result{
        .path = requestedPath,
        .usesEditedPath = requestedEditedPath,
    };
    if (!requestedEditedPath && requestedPathNeedsRepair &&
        editedShadow.has_value()) {
        result.path = editedShadow.value();
        result.usesEditedPath = true;
        result.promotedEditedShadow = true;
    }
    return result;
}

bool ApplyAnimationSelectionForSave(
    invisible_places::camera::AnimationPath* preparedPath,
    std::string_view liveVisualName,
    bool sourceHasEditedVersion,
    bool currentPathUsesEdited,
    const invisible_places::camera::AnimationPath*
        renderSetupUnderlyingAuthority) {
    if (preparedPath == nullptr) {
        return false;
    }

    std::optional<std::string> selectedVisual;
    if (renderSetupUnderlyingAuthority != nullptr) {
        selectedVisual =
            renderSetupUnderlyingAuthority->selectedPointVisualName;
    } else if ((!sourceHasEditedVersion || currentPathUsesEdited) &&
               !liveVisualName.empty()) {
        selectedVisual = NormalizeName(liveVisualName);
    }
    if (!selectedVisual.has_value() ||
        preparedPath->selectedPointVisualName == selectedVisual.value()) {
        return false;
    }
    preparedPath->selectedPointVisualName =
        std::move(selectedVisual.value());
    return true;
}

bool AnimationVisualRequiresProjectSave(
    std::string_view animationVisualName,
    std::span<const std::string> durableProjectVisualNames) {
    if (animationVisualName.empty()) {
        return false;
    }
    const auto normalized = NormalizeName(animationVisualName);
    return std::none_of(
        durableProjectVisualNames.begin(),
        durableProjectVisualNames.end(),
        [&](const auto& candidate) {
            return NormalizeName(candidate) == normalized;
        });
}

std::vector<std::string> ProjectVisualNames(
    const invisible_places::serialization::ProjectDocument& project) {
    std::vector<std::string> names;
    names.reserve(
        project.pointVisuals.size() +
        project.sceneVisualStates.size() + 1U);
    names.push_back(std::string{kDefaultName});
    for (const auto& visual : project.pointVisuals) {
        if (!visual.name.empty()) {
            names.push_back(visual.name);
        }
    }
    for (const auto& state : project.sceneVisualStates) {
        if (!state.visual.name.empty()) {
            names.push_back(state.visual.name);
        }
    }
    return names;
}

}  // namespace invisible_places::app::point_visual
