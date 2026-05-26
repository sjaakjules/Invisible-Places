#pragma once

#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <cstddef>
#include <optional>
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

}  // namespace invisible_places::app::point_visual
