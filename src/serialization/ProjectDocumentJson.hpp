#pragma once

#include "serialization/ProjectDocument.hpp"

#include <nlohmann/json_fwd.hpp>

#include <optional>
#include <string>

namespace invisible_places::serialization {

// Canonical JSON value codecs shared by compound documents such as render
// setups. File-level project, animation, preset, and water-source persistence
// continues to use the same implementations.
[[nodiscard]] nlohmann::json AnimationPathToJson(
    const invisible_places::camera::AnimationPath& path);
[[nodiscard]] std::optional<invisible_places::camera::AnimationPath>
AnimationPathFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

[[nodiscard]] nlohmann::json ExportPresetToJson(
    const invisible_places::output::ExportPreset& preset);
[[nodiscard]] std::optional<invisible_places::output::ExportPreset>
ExportPresetFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

[[nodiscard]] nlohmann::json PointCloudStyleToJson(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style);
[[nodiscard]] std::optional<
    invisible_places::renderer::pointcloud::PointCloudStyleState>
PointCloudStyleFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

[[nodiscard]] nlohmann::json TimingTakeSceneStateToJson(
    const invisible_places::timing::TimingTakeSceneState& state);
[[nodiscard]] std::optional<invisible_places::timing::TimingTakeSceneState>
TimingTakeSceneStateFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

// Converts pre-schema-62 Palette Phase keys from absolute unwrapped turns to
// signed deltas before the current timing sanitizer constrains them to one
// turn. Render-setup schema migration shares this JSON-level conversion.
void MigrateAbsoluteTimingColourisePalettePhaseKeys(
    nlohmann::json* timingTakeSceneStateJson);

[[nodiscard]] nlohmann::json WaterAnimationTrailSettingsToJson(
    const invisible_places::water::WaterAnimationTrailSettings& settings);
[[nodiscard]] std::optional<
    invisible_places::water::WaterAnimationTrailSettings>
WaterAnimationTrailSettingsFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

[[nodiscard]] nlohmann::json WaterAnimationTrailProfileToJson(
    const WaterAnimationTrailProfileDocument& profile);
[[nodiscard]] std::optional<WaterAnimationTrailProfileDocument>
WaterAnimationTrailProfileFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

[[nodiscard]] nlohmann::json PointCloudVisualToJson(
    const ProjectLayerDocument::PointVisual& visual);
[[nodiscard]] std::optional<ProjectLayerDocument::PointVisual>
PointCloudVisualFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

[[nodiscard]] nlohmann::json WaterSourcesDocumentToJson(
    const WaterSourcesDocument& document);
[[nodiscard]] std::optional<WaterSourcesDocument> WaterSourcesDocumentFromJson(
    const nlohmann::json& value,
    std::string* errorMessage = nullptr);

}  // namespace invisible_places::serialization
