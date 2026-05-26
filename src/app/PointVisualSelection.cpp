#include "app/PointVisualSelection.hpp"

#include <algorithm>
#include <cctype>

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

}  // namespace invisible_places::app::point_visual
